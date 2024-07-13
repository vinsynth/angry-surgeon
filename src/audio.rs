use crate::fs;
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

#[derive(Debug, PartialEq, Eq)]
pub enum Event {
    Sync,
    Hold { step: u32 },
    Loop { step: u32, length: Fraction },
    Load { cuts: Vec<u8> },
}

#[derive(Debug, PartialEq, Eq, Format)]
enum Desync {
    Hold,
    Loop { length: Fraction },
}

#[derive(Debug, PartialEq, Eq, Format)]
enum State {
    Sync,
    Desync { inner: Desync, step: u32, index: usize, counter: u32 }
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
    event_buf: Option<Event>,
    vol_mgr: VolMgr<'a>,
    files: Vec<Wav>,
    cuts: [Option<Cut>; crate::PAD_COUNT as usize],
    seq: Vec<u8>,
    seq_index: Option<usize>,
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
            event_buf: None,
            vol_mgr,
            files,
            cuts: core::array::from_fn(|_| None),
            seq: Vec::new(),
            seq_index: None,
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
    pub async fn assign_pad(&mut self, file_index: Option<usize>, pad_index: Option<u8>, step_count: Option<u8>) -> Result<bool, Error> {
        // remaining steps in file to be assigned
        static REM_4_FILE: AsyncMutex<Option<(u8, usize)>> = AsyncMutex::new(None);

        defmt::info!("step_count: {}", step_count);

        if let (Some(file_index), Some(pad_index), Some(step_count)) = (file_index, pad_index, step_count) {
            if let Some(file) = self.files.get(file_index) {
                if let Some(pad) = self.cuts.get_mut(pad_index as usize) {
                    let mut rem_opt = REM_4_FILE.lock().await;
                    if rem_opt.is_some_and(|(_, i)| i != file_index) {
                        *rem_opt = Some((file.rhythm.step_count(), file_index));
                    }
                    let (rem, _) = rem_opt.get_or_insert((file.rhythm.step_count(), file_index));

                    let step_offset = file.rhythm.step_count() - *rem;
                    // track steps now accounted for
                    *rem = rem.checked_sub(step_count).ok_or(Error::StepOutOfBounds)?;
            
                    // assign steps to pad
                    *pad = Some(Cut {
                        src_file: file.file,
                        src_rhythm: file.rhythm,
                        step_offset,
                        step_count,
                    });

                    defmt::info!("assigned {} steps from file {} to pad {}!",
                        step_count,
                        Debug2Format(&file.file),
                        pad_index,
                    );

                    // reload if currently targeting pad
                    if let Some(i) = self.seq_index {
                        if self.seq.get(i).is_some_and(|&i| i == pad_index) {
                            self.load_seq_index(i).await?;
                        }
                    }

                    if *rem == 0 {
                        rem_opt.take();

                        return Ok(true);
                    }
                    return Ok(false);
                }
                return Err(Error::PadOutOfBounds);
            }
            REM_4_FILE.lock().await.take();
            return Err(Error::FileOutOfBounds)
        }
        REM_4_FILE.lock().await.take();
        Ok(true)
    }

    pub async fn tap(&mut self, past: Option<Instant>, cancel: bool) -> Result<Option<Instant>, Error> {
        if !self.is_file_loaded() {
            return Err(Error::NoFileLoaded);
        }
        // relative offset [0..1] within step
        static ANCHOR: AsyncMutex<Option<f32>> = AsyncMutex::new(None);

        let now = Instant::now();
        if cancel {
            *ANCHOR.lock().await = None;
            return Ok(None)
        }
        if let Some(past) = past {
            let millis = now.duration_since(past).as_millis();

            if millis > 17 {
                let speed = self.step_len()? as f32 / (millis as f32 * (SAMPLE_RATE.0 / 1000) as f32);
                self.tap_speed = speed;
                let offset = (*crate::lock_async_ref!(ANCHOR) * self.step_len()? as f32) as u32
                    + (self.cut_offset()? - self.cut_offset()? % self.step_len()?);
                defmt::info!("offset: {}", offset);
                self.cut_seek_from_start(offset)?;
            }
        } else {
            ANCHOR.lock().await.replace((self.cut_offset()?  as f32 / self.step_len()? as f32).fract());
        }
        Ok(Some(now))
    }

    pub async fn read_grain(&mut self) -> Result<[u8; GRAIN_LEN], Error> {
        // handle state
        if self.is_quantum().await? {
            self.update().await?;
            self.led.toggle();

            // retrig
            if let State::Desync {
                inner: Desync::Loop { length }, step, index, ..
            } = self.state {
                if let Some(curr_idx) = self.seq_index.and_then(|i| self.seq.get(i)) {
                    let start = self.seq_length_until_index(index)? + step * self.step_len()?;
                    let end = self.seq_length_until_index(index)? + u32::from((length + step) * Fraction::from(self.step_len()?));
                    let offset = self.seq_length_until_index(*curr_idx as usize)? + self.cut_offset()?;

                    if offset > end
                        || offset < start
                        && offset > (start + end) % self.cut_length()?
                    {
                        self.load_seq_index(index).await?;
                        self.cut_seek_from_start(step * self.step_len()?)?;
                    }
                }
            }
        }

        let mut out = [u8::MAX / 2; GRAIN_LEN];
        let speed = self.speed();
        let mut buf = vec![0; (GRAIN_LEN as f32 * speed) as usize].into_boxed_slice();

        // read cut(s) until buffer full
        let mut bytes = self.read_file(&mut buf).await;
        while bytes.is_err()
            || bytes.as_ref().is_ok_and(|x| *x < buf.len())
            || self.cut_offset()? > self.cut_length()?
        {
            self.next_seq_index().await?;

            bytes = self.read_file(&mut buf).await;
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
        let length = self.seq_length_until_index(self.seq.len())?;
        if let State::Desync { ref mut counter, .. } = self.state {
            // convert desync counter to seq_index
            let seq_index = (0..self.seq.len())
                .try_fold(None, |res, idx| -> Result<Option<usize>, Error> {
                    match res {
                        None => {
                            let cut_end = {
                                let seq_index = idx + 1;
                                self.seq[..seq_index]
                                    .iter()
                                    .try_fold(0, |acc, i| {
                                        if let Some(Some(cut)) = self.cuts.get(*i as usize) {
                                            let pcm_length = self.vol_mgr.file_length(cut.src_file)? - 0x2c;
                                            return Ok(acc + pcm_length * cut.step_count as u32 / cut.src_rhythm.step_count() as u32);
                                        }
                                        Err(Error::PadOutOfBounds)
                                    })
                            }?;
                            if cut_end > *counter {
                                return Ok(Some(idx));
                            }
                            Ok(None)
                        },
                        opt => Ok(opt),
                    }
                })?.ok_or(Error::NoFileLoaded)?;
            // add len of theoretical sync buffer to counter
            if let Some(Some(cut)) = self.seq.get(seq_index).and_then(|i| self.cuts.get(*i as usize)) {
                if let Some(anchor) = self.anchor_tempo {
                    let sync_speed = anchor / cut.src_rhythm.tempo();
                    let buf_len = GRAIN_LEN as f32 * sync_speed * self.tap_speed * self.speed_mod.unwrap_or(1.0);
                    *counter = (*counter + buf_len as u32).checked_rem(length).unwrap_or(*counter + buf_len as u32);
                }
            } else {
                return Err(Error::NoFileLoaded);
            }
        }
        Ok(out)
    }

    pub fn buffer(&mut self, event: Event) -> Result<(), Error> {
        if !self.is_file_loaded() && matches!(self.event_buf, Some(Event::Load { .. })) {
            return Err(Error::NoFileLoaded);
        }
        self.event_buf = Some(event);
        Ok(())
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

    pub fn step_count(&self) -> Result<u8, Error> {
        if let Some(Some(cut)) = self.seq_index.and_then(|v| self.seq.get(v).and_then(|v| self.cuts.get(*v as usize))) {
            return Ok(cut.step_count);
        }
        Err(Error::NoFileLoaded)
    }

    pub fn step_len(&self) -> Result<u32, Error> {
        if let Some(Some(cut)) = self.seq_index.and_then(|v| self.seq.get(v).and_then(|v| self.cuts.get(*v as usize))) {
            return Ok(self.cut_length()? / cut.step_count as u32);
        }
        Err(Error::NoFileLoaded)
    }

    pub fn speed(&self) -> f32 {
        self.sync_speed * self.tap_speed * self.speed_mod.unwrap_or(1.0)
    }

    /// goto next non-empty step of sequence, if available
    async fn next_seq_index(&mut self) -> Result<(), Error> {
        if let Some(init) = self.seq_index {
            for addend in  1..=self.seq.len() {
                let i = (init + addend) % self.seq.len();
                if self.seq.get(i).is_some_and(|i| self.cuts.get(*i as usize).is_some_and(|c| c.is_some())) {
                    defmt::info!("skipping to step {}!", i);
                    self.load_seq_index(i).await?;

                    return Ok(());
                }
            }
        }
        Err(Error::NoFileLoaded)
    }

    async fn load_seq_index(&mut self, index: usize) -> Result<(), Error> {
        if index >= self.cuts.len() {
            return Err(Error::PadOutOfBounds);
        }
        self.seq_index = Some(index);
        defmt::info!("self.seq_index: {}", self.seq_index);
        if let Some(Some(cut)) = self.seq.get(index).and_then(|i| self.cuts.get(*i as usize)) {
            if let Some(anchor) = self.anchor_tempo {
                // sync to global tempo
                self.sync_speed = anchor / cut.src_rhythm.tempo();
            } else {
                // anchor global tempo
                self.anchor_tempo = Some(cut.src_rhythm.tempo());
                defmt::info!("anchor tempo: {}", self.anchor_tempo);
            }

            self.cut_seek_from_start(0)?;
            return Ok(());
        }
        Err(Error::NoFileLoaded)
    }

    async fn read_file(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if let Some(Some(cut)) = self.seq_index.and_then(|v| self.seq.get(v).and_then(|v| self.cuts.get(*v as usize))) {
            return Ok(self.vol_mgr.read(cut.src_file, buffer).await?);
        }
        Err(Error::NoFileLoaded)
    }

    async fn update(&mut self) -> Result<(), Error> {
        if let Some(event) = self.event_buf.as_ref() {
            self.state = match event {
                &Event::Sync => {
                    if let State::Desync { counter, .. } = self.state {
                        let seq_index = self.counter_to_seq_index(counter)?;
                        let cut_start = self.seq_length_until_index(seq_index)?;

                        self.load_seq_index(seq_index).await?;
                        self.cut_seek_from_start(counter - cut_start)?;
                        defmt::info!("sync to {} (index {})!", counter, seq_index);
                    }
                    State::Sync
                },
                &Event::Hold { step } => {
                    let (counter, index) = match self.state {
                        State::Desync { inner: Desync::Loop { .. }, counter, index, .. } => (counter, index),
                        _ => {
                            let index = self.seq_index.ok_or(Error::NoFileLoaded)?;
                            let counter = self.seq_length_until_index(index)? + self.cut_offset()?;
                            self.cut_seek_from_start(self.step_len()? * step)?;
                            (counter, index)
                        },
                    };
                    defmt::info!("hold from {}!", counter);

                    State::Desync {
                        inner: Desync::Hold,
                        step,
                        counter,
                        index,
                    }
                },
                &Event::Loop { step, length } => {
                    let (counter, index) = match self.state {
                        State::Desync { counter, index, .. } => (counter, index),
                        _ => {
                            let index = self.seq_index.ok_or(Error::NoFileLoaded)?;
                            let counter = self.seq_length_until_index(index)? + self.cut_offset()?;
                            (counter, index)
                        },
                    };
                    self.cut_seek_from_start(self.step_len()? * step)?;

                    State::Desync {
                        inner: Desync::Loop { length },
                        step,
                        counter,
                        index,
                    }
                },
                Event::Load { cuts } => {
                    let cuts = cuts
                        .iter()
                        .copied()
                        .filter(|&v| self.cuts.len() > v as usize)
                        .collect::<Vec<_>>();
                    self.seq.clone_from(&cuts);
                    match self.load_seq_index(0).await {
                        Err(Error::NoFileLoaded) => (),
                        Err(e) => return Err(e),
                        _ => (),
                    };
                    defmt::info!("loaded sequence {}!", Debug2Format(&cuts));

                    State::Sync
                },
            };
            self.event_buf.take();
        }

        Ok(())
    }

    async fn is_quantum(&self) -> Result<bool, Error> {
        if !self.is_file_loaded() {
            return Ok(true);
        }
        if matches!(self.event_buf, Some(Event::Loop { .. }))
            || matches!(self.event_buf, Some(Event::Sync)) && !self.running
            || matches!(self.event_buf, Some(Event::Hold {..})) && !self.running
            || matches!(self.state, State::Desync { inner: Desync::Loop { .. }, .. })
        {
            return Ok(self.cut_offset()? % (self.step_len()? / 16) < (GRAIN_LEN as f32 * self.speed()) as u32)
        }
        Ok(self.cut_offset()? % (self.step_len()? / 2) < (GRAIN_LEN as f32 * self.speed()) as u32)
    }

    /// convert desync counter to seq_index
    fn counter_to_seq_index(&self, counter: u32) -> Result<usize, Error> {
        (0..self.seq.len())
            .try_fold(None, |res, idx| -> Result<Option<usize>, Error> {
                match res {
                    None => {
                        let cut_end = self.seq_length_until_index(idx + 1)?;
                        if cut_end > counter {
                            return Ok(Some(idx));
                        }
                        Ok(None)
                    },
                    opt => Ok(opt),
                }
            })?.ok_or(Error::NoFileLoaded)
    }

    fn cut_length(&self) -> Result<u32, Error> {
        if let Some(Some(cut)) = self.seq_index.and_then(|v| self.seq.get(v).and_then(|v| self.cuts.get(*v as usize))) {
            return Ok((self.vol_mgr.file_length(cut.src_file)? - 0x2c) * cut.step_count as u32 / cut.src_rhythm.step_count() as u32);
        }
        Err(Error::NoFileLoaded)
    }

    fn seq_length_until_index(&self, seq_index: usize) -> Result<u32, Error> {
        if seq_index >= self.cuts.len()  {
            return Err(Error::PadOutOfBounds);
        }
        self.seq[..seq_index]
            .iter()
            .try_fold(0, |acc, i| {
                if let Some(Some(cut)) = self.cuts.get(*i as usize) {
                    let pcm_length = self.vol_mgr.file_length(cut.src_file)? - 0x2c;
                    return Ok(acc + pcm_length * cut.step_count as u32 / cut.src_rhythm.step_count() as u32);
                }
                Ok(acc)
            })
    }

    fn cut_offset(&self) -> Result<u32, Error> {
        if let Some(Some(cut)) = self.seq_index.and_then(|v| self.seq.get(v).and_then(|v| self.cuts.get(*v as usize))) {
            let pcm_length = self.vol_mgr.file_length(cut.src_file)? - 0x2c;
            let start = 0x2c + pcm_length * cut.step_offset as u32 / cut.src_rhythm.step_count() as u32;
            return self.vol_mgr.file_offset(cut.src_file)?.checked_sub(start).ok_or(Error::StepOutOfBounds);
        }
        Err(Error::NoFileLoaded)
    }

    fn cut_seek_from_start(&mut self, offset: u32) -> Result<(), Error> {
        if let Some(Some(cut)) = self.seq_index.and_then(|v| self.seq.get(v).and_then(|v| self.cuts.get(*v as usize))) {
            let pcm_length = self.vol_mgr.file_length(cut.src_file)? - 0x2c;
            let start = 0x2c + pcm_length * cut.step_offset as u32 / cut.src_rhythm.step_count() as u32;
            return Ok(self.vol_mgr.file_seek_from_start(cut.src_file, start + offset)?);
        }
        Err(Error::NoFileLoaded)
    }

    fn is_file_loaded(&self) -> bool {
        self.seq_index.is_some_and(|i| self.seq.get(i).is_some_and(|i| self.cuts.get(*i as usize).is_some_and(|c| c.is_some())))
    }
}
