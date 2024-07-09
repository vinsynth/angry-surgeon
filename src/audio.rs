use alloc::vec;
use alloc::vec::Vec;

use crate::{fs, RingBuf};
use crate::utils::Fraction;

use crate::rhythm::RhythmData;

use defmt::{Debug2Format, Format};
use micromath::F32Ext;

use embedded_sdmmc::filesystem::ToShortFileName;

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
    FilenameError,
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

#[derive(Debug, Clone)]
pub struct Wav {
    name: embedded_sdmmc::ShortFileName,
    rhythm: RhythmData,
}

pub struct Steps<'a> {
    state: State,
    event_buf: RingBuf<Event, 2>,
    vol_mgr: VolMgr<'a>,
    root: embedded_sdmmc::RawDirectory,
    file: Option<embedded_sdmmc::RawFile>,
    file_names: Vec<embedded_sdmmc::ShortFileName>,
    pads: [Option<Wav>; 16],
    pad_index: Option<usize>,
    anchor_tempo: f32,
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
        let file_names = {
            let mut file_infos = Vec::new();
            vol_mgr.iterate_dir(root, |x| {
                file_infos.push((x.name.clone(), x.ctime));
            }).await?;
            file_infos.sort_by_key(|(_, ctime)| *ctime);
            file_infos.iter().map(|(name, _)| name.clone()).collect::<Vec<_>>()
        };
        defmt::info!("files: {}", Debug2Format(&file_names));

        Ok(Self {
            state: State::Sync,
            event_buf: RingBuf::new(),
            vol_mgr,
            root,
            file: None,
            file_names,
            pads: core::array::from_fn(|_| None),
            pad_index: None,
            anchor_tempo: 0.0,
            sync_speed: 1.0,
            tap_speed: 1.0,
            speed_mod: None,
            running: true,
            led,
        })
    }

    pub async fn assign_pad<N: ToShortFileName + Clone>(&mut self, pad_index: &u8, file_name: N) -> Result<(), Error> {
        defmt::assert!(*pad_index < 16);

        let name = file_name.to_short_filename().map_err(|_| Error::FilenameError)?;
        // reuse rhythm from other instance of file, otherwise calculate rhythm
        let rhythm = if let Some(wav) = self.pads.iter().flatten().find(|p| p.name == name) {
            wav.rhythm
        } else {
            let file = self.vol_mgr.open_file_in_dir(
                self.root,
                name.clone(),
                embedded_sdmmc::Mode::ReadOnly
            ).await?;
            let rhythm = RhythmData::new(&mut self.vol_mgr, &file).await?;
            self.vol_mgr.close_file(file).await?;
            rhythm
        };

        if let Some(pad) = self.pads.get_mut(*pad_index as usize) {
            *pad = Some(Wav { name, rhythm });
        }

        Ok(())
    }

    pub async fn load_pad(&mut self, pad_index: &usize) -> Result<(), Error> {
        defmt::assert!(*pad_index < 16);

        if let Some(Some(pad)) = self.pads.get(*pad_index) {
            if let Some(file) = self.file {
                self.vol_mgr.close_file(file).await?;
            }

            if self.file.is_some() {
                // sync to global tempo
                self.sync_speed = self.anchor_tempo / pad.rhythm.tempo()
            } else {
                // anchor global tempo
                self.anchor_tempo = pad.rhythm.tempo();
                defmt::info!("anchor tempo: {}", self.anchor_tempo);
            }
            self.file = Some(self.vol_mgr.open_file_in_dir(
                self.root,
                pad.name.clone(),
                embedded_sdmmc::Mode::ReadOnly
            ).await?);


            self.file_seek_from_start(0x2c)?;
            self.event_buf = RingBuf::new();
            self.state = State::Sync;
            self.pad_index = Some(*pad_index);

            return Ok(());
        }

        Err(Error::NoFileLoaded)
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
                let offset = self.file_offset()? - 0x2c;

                if offset > end ||
                    offset < start &&
                    offset > (start + end) % (self.file_length()? - 0x2c)
                {
                    defmt::debug!("retrig: init_seek to {}", start);
                    self.file_seek_from_start(0x2c + start)?;
                }
            }
        }

        // read grain
        let mut out = [u8::MAX / 2; GRAIN_LEN];
        let speed = self.speed();
        let mut buf = vec![0; (GRAIN_LEN as f32 * speed) as usize].into_boxed_slice();

        if let Some(file) = self.file {
            let bytes = self.vol_mgr.read(file, &mut buf).await;
            if bytes.is_err() || bytes.as_ref().is_ok_and(|x| *x < buf.len()) {
                // if reached eof, seek to start and finish read
                self.file_seek_from_start(0x2c)?;
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
        let length = self.file_length()?;
        if let State::Desync { ref mut counter, .. } = self.state {
            *counter = (*counter + buf.len() as u32) % (length - 0x2c);
        }

        Ok(out)
    }

    pub fn buffer(&mut self, event: Event) {
        self.event_buf.push_back(event);
    }

    pub async fn tap(&mut self, past: Option<Instant>, cancel: bool) -> Result<Option<Instant>, Error> {
        static ANCHOR: crate::AsyncMutex<Option<u32>> = crate::AsyncMutex::new(None);

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
                let offset = (self.file_offset()? / beat_len) * beat_len + crate::lock_async_ref!(ANCHOR);
                defmt::info!("offset: {}", offset);
                self.file_seek_from_start(offset)?;
            }
        } else {
            ANCHOR.lock().await.replace(self.file_offset()? % self.step_len()?);
        }
        Ok(Some(now))
    }

    pub fn set_running(&mut self, running: bool) -> Result<(), Error> {
        self.running = running;
        self.file_seek_from_start(0x2c)
    }

    pub fn set_speed_mod(&mut self, speed: Option<f32>) {
        self.speed_mod = speed;
    }

    pub fn step_len(&self) -> Result<u32, Error> {
        Ok((self.file_length()? - 0x2c) / self.step_count().ok_or(Error::NoFileLoaded)? as u32)
    }

    pub fn speed(&self) -> f32 {
        self.sync_speed * self.tap_speed * self.speed_mod.unwrap_or(1.0)
    }

    pub fn step_count(&self) -> Option<u8> {
        self.pads.get(self.pad_index?).and_then(|p| p.clone().map(|p| p.rhythm.step_count()))
    }

    pub fn file_names(&self) -> Vec<embedded_sdmmc::ShortFileName> {
        self.file_names.clone()
    }

    fn file_offset(&self) -> Result<u32, Error> {
        if let Some(file) = self.file {
            return Ok(self.vol_mgr.file_offset(file)?)
        }
        Err(Error::NoFileLoaded)
    }

    fn file_length(&self) -> Result<u32, Error> {
        if let Some(file) = self.file {
            return Ok(self.vol_mgr.file_length(file)?)
        }
        Err(Error::NoFileLoaded)
    }

    fn file_seek_from_start(&mut self, offset: u32) -> Result<(), Error> {
        if let Some(file) = self.file {
            return Ok(self.vol_mgr.file_seek_from_start(file, offset)?)
        }
        Err(Error::NoFileLoaded)
    }

    async fn file_read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if let Some(file) = self.file {
            return Ok(self.vol_mgr.read(file, buffer).await?)
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
                            self.file_seek_from_start(0x2c + counter)?;

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

                    let counter = self.file_offset()? - 0x2c;
                    self.file_seek_from_start(0x2c + self.step_len()? * step)?;

                    State::Desync {
                        inner: Desync::Hold,
                        step,
                        counter,
                    }
                },
                Event::Loop { step, length } => {
                    let counter = match self.state {
                        State::Desync { counter, .. } => counter,
                        _ => self.file_offset()? - 0x2c,
                    };
                    self.file_seek_from_start(0x2c + self.step_len()? * step)?;

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
            Ok((self.file_offset()?.saturating_sub(0x2c)) % (self.step_len()? / 16) < (GRAIN_LEN as f32 * self.speed_mod.unwrap_or(self.speed())) as u32)
        } else {
            Ok((self.file_offset()?.saturating_sub(0x2c)) % (self.step_len()? / 2) < (GRAIN_LEN as f32 * self.speed_mod.unwrap_or(self.speed())) as u32)
        }
    }
}
