use crate::{fs, RingBuf};
use crate::utils::Fraction;
use crate::AsyncMutex;

use crate::rhythm::RhythmData;

use alloc::vec;
use alloc::vec::Vec;

use defmt::{Debug2Format, Format};
use micromath::F32Ext;

use embedded_sdmmc::{Mode, RawFile};
use embedded_sdmmc::filesystem::ShortFileName;

use embassy_stm32::peripherals::{DMA2_CH3, SDIO};
use embassy_stm32::time::Hertz;
use embassy_time::Instant;

// import end ------------------------------------------------------------------

pub const GRAIN_LEN: usize = 256;
pub const SAMPLE_RATE: Hertz = Hertz::khz(32);

pub type VolMgr<'a> = embedded_sdmmc::VolumeManager<fs::SdioCard<'a, SDIO, DMA2_CH3>, fs::DummyTimesource, 4, 4, 1>;

#[derive(Debug, Format)]
pub enum Error {
    NoFileLoaded,
    FileOutOfBounds,
    PadOutOfBounds,
    StepOutOfBounds,
    SdmmcError(embedded_sdmmc::Error<embassy_stm32::sdmmc::Error>),
}

impl From<embedded_sdmmc::Error<embassy_stm32::sdmmc::Error>> for Error {
    fn from(value: embedded_sdmmc::Error<embassy_stm32::sdmmc::Error>) -> Self {
        Self::SdmmcError(value)
    }
}

#[derive(Debug, PartialEq, Eq, Format)]
pub enum Event {
    Sync,
    Hold { step: u32 },
    Loop { step: u32, length: Fraction },
}

#[derive(Debug, PartialEq, Eq, Format)]
enum Desync {
    Hold,
    Loop { length: Fraction },
}

#[derive(Debug, PartialEq, Eq, Format)]
enum State {
    Sync,
    Desync { inner: Desync, step: u32, counter: u32 }
}

/// audio file with additional information
#[derive(Debug, Clone)]
struct Wav {
    name: ShortFileName,
    file: RawFile,
    rhythm: RhythmData,
}

/// segment of source Wav
#[derive(Debug, Clone)]
struct Cut {
    src_file: RawFile,
    src_rhythm: RhythmData,
    step_offset: u8,
    step_count: u8,
}

pub struct Steps<'a> {
    state: State,
    event_buf: RingBuf<Event, 2>,
    vol_mgr: VolMgr<'a>,
    root: embedded_sdmmc::RawDirectory,
    files: Vec<Wav>,
    pads: [Option<Cut>; crate::PAD_COUNT as usize],
    pad_index: Option<usize>,
    anchor_tempo: Option<f32>,
    sync_speed: f32,
    tap_speed: f32,
    speed_mod: Option<f32>,
    running: bool,
    led: embassy_stm32::gpio::Output<'a>,
}

impl<'a> Steps<'a> {
    pub async fn new(led: embassy_stm32::gpio::Output<'a>, mut vol_mgr: VolMgr<'a>) -> Result<Self, Error> {
        let volume = vol_mgr.open_raw_volume(embedded_sdmmc::VolumeIdx(0)).await?;
        let root = vol_mgr.open_root_dir(volume)?;
        let files = {
            let mut file_infos = Vec::new();
            vol_mgr.iterate_dir(root, |x| {
                file_infos.push((x.name.clone(), x.ctime));
            }).await?;
            file_infos.sort_by_key(|(_, ctime)| *ctime);

            let mut files = Vec::new();
            for name in file_infos.iter().map(|(name, _)| name) {
                let file = vol_mgr.open_file_in_dir(root, name, Mode::ReadOnly).await?;
                let rhythm = RhythmData::new(&mut vol_mgr, &file).await?;

                files.push(Wav { name: name.clone(), file, rhythm });
            }
            files
        };
        defmt::info!("files: {}", Debug2Format(&files));

        Ok(Self {
            state: State::Sync,
            event_buf: RingBuf::new(),
            vol_mgr,
            root,
            files,
            pads: core::array::from_fn(|_| None),
            pad_index: None,
            anchor_tempo: None,
            sync_speed: 1.0,
            tap_speed: 1.0,
            speed_mod: None,
            running: true,
            led,
        })
    }

    /// assign `Cut` from file to specified pad
    ///
    /// returns `true` if enough `Cut`s have been assigned to represent the whole file
    pub async fn assign_pad(&mut self, file_index: &usize, pad_index: &u8, step_count: &u8) -> Result<bool, Error> {
        static REMAINING_STEPS: AsyncMutex<Option<u8>> = AsyncMutex::new(None);

        if let Some(file) = self.files.get(*file_index) {
            if let Some(pad) = self.pads.get_mut(*pad_index as usize) {
                let mut rem_opt = REMAINING_STEPS.lock().await;
                let rem = rem_opt.get_or_insert(file.rhythm.step_count());

                let step_offset = file.rhythm.step_count() - *rem;
                // track steps now accounted for
                *rem = rem.checked_sub(*step_count).ok_or(Error::StepOutOfBounds)?;
            
                // assign steps to pad
                *pad = Some(Cut {
                    src_file: file.file,
                    src_rhythm: file.rhythm,
                    step_offset,
                    step_count: *step_count
                });

                defmt::info!("assigned {} steps from file {} to pad {}!",
                    step_count,
                    Debug2Format(&file.file),
                    pad_index,
                );

                if *rem == 0 {
                    rem_opt.take();

                    return Ok(true);
                } else {
                    return Ok(false);
                }
            } else {
                return Err(Error::PadOutOfBounds);
            }
        }

        REMAINING_STEPS.lock().await.take();
        Err(Error::FileOutOfBounds)
    }

    pub async fn load_pad(&mut self, pad_index: &usize) -> Result<(), Error> {
        if let Some(Some(cut)) = self.pads.get(*pad_index) {
            if let Some(anchor) = self.anchor_tempo {
                // sync to global tempo
                self.sync_speed = anchor / cut.src_rhythm.tempo();
            } else {
                // anchor global tempo
                self.anchor_tempo = Some(cut.src_rhythm.tempo());
                defmt::info!("anchor tempo: {}", self.anchor_tempo);
            }

            self.pad_index = Some(*pad_index);

            self.cut_seek_from_start(0)?;

            self.event_buf = RingBuf::new();
            self.state = State::Sync;

            return Ok(());
        }

        Err(Error::PadOutOfBounds)
    }

    pub async fn read_grain(&mut self) -> Result<[u8; GRAIN_LEN], Error> {
        // handle state
        if self.is_quantum().await? {
            self.update().await?;
            self.led.toggle();

            // retrig
            if let State::Desync {
                inner: Desync::Loop { length }, step, ..
            } = self.state {
                let start = step * self.step_len()?;
                let end = u32::from((length + step) * Fraction::from(self.step_len()?));
                let offset = self.cut_offset()?;

                if offset > end ||
                    offset < start &&
                    offset > (start + end) % self.cut_length()?
                {
                    defmt::debug!("retrig: init_seek to {}", start);
                    self.cut_seek_from_start(start)?;
                }
            }
        }

        // read grain
        let mut out = [u8::MAX / 2; GRAIN_LEN];
        let speed = self.speed();
        let mut buf = vec![0; (GRAIN_LEN as f32 * speed) as usize].into_boxed_slice();

        if let Some(Some(cut)) = self.pad_index.and_then(|v| self.pads.get_mut(v)) {
            let bytes = self.vol_mgr.read(cut.src_file, &mut buf).await;

            if bytes.is_err()
                || bytes.as_ref().is_ok_and(|x| *x < buf.len())
                || self.cut_offset()? > self.cut_length()?
            {
                // if reached eof or end of cut, seek to cut start and finish read
                self.cut_seek_from_start(0)?;
                self.file_read(&mut buf[bytes.unwrap_or(0)..]).await?;
            }
        }

        if self.running || !matches!(self.state, State::Sync) {
            // resample via linear interpolation
            for (i, x) in out.iter_mut().enumerate() {
                let buf_i = i as f32 * speed;
                *x = (buf_i.fract() * *buf.get(buf_i as usize).unwrap_or(&(u8::MAX / 2)) as f32 +
                    (1.0 - buf_i.fract()) * *buf.get(buf_i as usize + 1).unwrap_or(&(u8::MAX / 2)) as f32
                ) as u8;
            }
        }
        
        // sync desync
        let length = self.cut_length()?;
        if let State::Desync { ref mut counter, .. } = self.state {
            *counter = (*counter + buf.len() as u32) % length;
        }

        Ok(out)
    }

    pub fn buffer(&mut self, event: Event) {
        self.event_buf.push_back(event);
    }

    pub async fn tap(&mut self, past: Option<Instant>, cancel: bool) -> Result<Option<Instant>, Error> {
        static ANCHOR: AsyncMutex<Option<u32>> = AsyncMutex::new(None);

        let now = Instant::now();
        if cancel {
            *ANCHOR.lock().await = None;
            return Ok(None)
        }
        if let Some(past) = past {
            let millis = now.duration_since(past).as_millis();

            if millis > 17 {
                let beat_len = self.step_len()?;
                let speed = beat_len as f32 / (millis as f32 * (SAMPLE_RATE.0 / 1000) as f32);
                self.tap_speed = speed;
                let offset = (self.cut_offset()? / beat_len) * beat_len + crate::lock_async_ref!(ANCHOR);
                defmt::info!("offset: {}", offset);
                self.cut_seek_from_start(offset)?;
            }
        } else {
            ANCHOR.lock().await.replace(self.cut_offset()? % self.step_len()?);
        }
        Ok(Some(now))
    }

    pub fn set_running(&mut self, running: bool) -> Result<(), Error> {
        self.running = running;
        self.cut_seek_from_start(0)
    }

    pub fn set_speed_mod(&mut self, speed: Option<f32>) {
        self.speed_mod = speed;
    }

    pub fn file_count(&self) -> usize {
        self.files.len()
    }

    pub fn step_len(&self) -> Result<u32, Error> {
        Ok(self.cut_length()? / self.step_count().ok_or(Error::NoFileLoaded)? as u32)
    }

    pub fn speed(&self) -> f32 {
        self.sync_speed * self.tap_speed * self.speed_mod.unwrap_or(1.0)
    }

    pub fn step_count(&self) -> Option<u8> {
        self.pads.get(self.pad_index?).and_then(|c| c.clone().map(|p| p.step_count))
    }

    fn cut_offset(&self) -> Result<u32, Error> {
        if let Some(Some(cut)) = self.pad_index.and_then(|v| self.pads.get(v)) {
            let pcm_length = self.vol_mgr.file_length(cut.src_file)? - 0x2c;
            let start = 0x2c + pcm_length * cut.step_offset as u32 / cut.src_rhythm.step_count() as u32;
            return self.vol_mgr.file_offset(cut.src_file)?.checked_sub(start).ok_or(Error::StepOutOfBounds);
        }
        Err(Error::NoFileLoaded)
    }

    fn cut_length(&self) -> Result<u32, Error> {
        if let Some(Some(cut)) = self.pad_index.and_then(|v| self.pads.get(v)) {
            return Ok((self.vol_mgr.file_length(cut.src_file)? - 0x2c) * cut.step_count as u32 / cut.src_rhythm.step_count() as u32);
        }
        Err(Error::NoFileLoaded)
    }

    fn cut_seek_from_start(&mut self, offset: u32) -> Result<(), Error> {
        if let Some(Some(cut)) = self.pad_index.and_then(|v| self.pads.get(v)) {
            let pcm_length = self.vol_mgr.file_length(cut.src_file)? - 0x2c;
            let start = 0x2c + pcm_length * cut.step_offset as u32 / cut.src_rhythm.step_count() as u32;
            return Ok(self.vol_mgr.file_seek_from_start(cut.src_file, start + offset)?);
        }
        Err(Error::NoFileLoaded)
    }

    async fn file_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if let Some(Some(file)) = self.pad_index.and_then(|v| self.pads.get(v)) {
            return Ok(self.vol_mgr.read(file.src_file, buffer).await?);
        }
        Err(Error::NoFileLoaded)
    }

    async fn update(&mut self) -> Result<(), Error> {
        if let Some(event) = self.event_buf.front() {
            self.state = match *event {
                Event::Sync => {
                    match self.state {
                        State::Desync { inner: Desync::Hold, counter, .. } => {
                            // let i = (self.file_offset()?.saturating_sub(0x2c)) % (self.step_len()?) < GRAIN_LEN as u32;
                            // defmt::info!("sync: init_seek to {}, i: {}", counter, i);
                            self.cut_seek_from_start(counter)?;

                            State::Sync
                        },
                        State::Desync {
                            inner: Desync::Loop { .. },
                            step,
                            counter
                        } => {
                            // defmt::info!("sync: hold step {}", step);

                            State::Desync { inner: Desync::Hold, step, counter }
                        },
                        _ => State::Sync
                    }
                },
                Event::Hold { step } => {
                    // defmt::info!("hold: init_seek...");

                    let counter = self.cut_offset()?;
                    self.cut_seek_from_start(self.step_len()? * step)?;

                    State::Desync {
                        inner: Desync::Hold,
                        step,
                        counter,
                    }
                },
                Event::Loop { step, length } => {
                    let counter = match self.state {
                        State::Desync { counter, .. } => counter,
                        _ => self.cut_offset()?,
                    };
                    self.cut_seek_from_start(self.step_len()? * step)?;

                    State::Desync {
                        inner: Desync::Loop { length },
                        step,
                        counter,
                    }
                },
            };
            self.event_buf.pop_front();
        }

        Ok(())
    }

    async fn is_quantum(&self) -> Result<bool, Error> {
        if matches!(self.event_buf.front(), Some(Event::Loop { .. })) ||
            matches!(self.event_buf.front(), Some(Event::Sync)) && !self.running ||
            matches!(self.event_buf.front(), Some(Event::Hold {..})) && !self.running ||
            matches!(self.state, State::Desync { inner: Desync::Loop { .. }, .. }) {
            Ok(self.cut_offset()? % (self.step_len()? / 16) < (GRAIN_LEN as f32 * self.speed_mod.unwrap_or(self.speed())) as u32)
        } else {
            Ok(self.cut_offset()? % (self.step_len()? / 2) < (GRAIN_LEN as f32 * self.speed_mod.unwrap_or(self.speed())) as u32)
        }
    }
}
