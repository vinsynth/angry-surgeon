use alloc::vec;
use alloc::vec::Vec;

use crate::{fs, RingBuf};
use crate::utils::Fraction;

use crate::timestretch::Onsets;

use defmt::{Debug2Format, Format};
use fugit::HertzU32;
use micromath::F32Ext;

use embedded_sdmmc::filesystem::ToShortFileName;

use embassy_stm32::peripherals::{DMA2_CH3, SDIO};
use embassy_time::Instant;

// import end ------------------------------------------------------------------

pub const GRAIN_LEN: usize = 256;
pub const SAMPLE_RATE: HertzU32 = HertzU32::from_raw(32_000_000);
pub const STEPS_PER_BEAT: usize = 1;

pub type VolMgr<'a> = embedded_sdmmc::VolumeManager<fs::SdioCard<'a, SDIO, DMA2_CH3>, fs::DummyTimesource, 4, 4, 1>;

#[derive(Debug, Format)]
pub enum Error {
    NoFileLoaded,
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

pub struct Steps<'a> {
    state: State,
    event_buf: RingBuf<Event, 2>,
    vol_mgr: VolMgr<'a>,
    root: embedded_sdmmc::RawDirectory,
    file: Option<embedded_sdmmc::RawFile>,
    step_count: u32,
    speed: f32,
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
            // file_infos.sort_by_key(|(_, ctime)| *ctime);
            file_infos.iter().map(|(name, _)| name.clone()).collect::<Vec<_>>()
        };
        defmt::info!("files: {}", Debug2Format(&file_names));

        // default to youngest file
        let file = if let Some(name) = file_names.last() {
            Some(vol_mgr.open_file_in_dir(root, name, embedded_sdmmc::Mode::ReadOnly).await?)
        } else {
            None
        };

        let onsets = if let Some(file) = file {
            Some(Onsets::new(&mut vol_mgr, &file).await)
        } else {
            None
        };
        let _ = file.map(|f| vol_mgr.file_seek_from_start(f, 0x2c));

        Ok(Self {
            state: State::Sync,
            event_buf: RingBuf::new(),
            vol_mgr,
            root,
            file,
            step_count: 0,
            speed: 1.0,
            speed_mod: None,
            running: true,
            led,
        })
    }

    pub async fn load_file<N: ToShortFileName>(&mut self, file_name: N, step_count: u32) -> Result<(), Error> {
        match self.vol_mgr.open_file_in_dir(self.root, file_name, embedded_sdmmc::Mode::ReadOnly).await {
            Ok(file) => {
                if let Some(file) = self.file {
                    self.vol_mgr.close_file(file).await?;
                }
                self.file = Some(file);
                self.step_count = step_count;
            },
            Err(embedded_sdmmc::Error::FileAlreadyOpen) => (),
            Err(e) => defmt::panic!("failed to open file: {}", Debug2Format(&e)),
        }

        self.file_seek_from_start(0x2c)?;
        self.event_buf = RingBuf::new();
        self.state = State::Sync;

        Ok(())
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
                let start = step * self.step_len().await?;
                let end = u32::from((length + step) * Fraction::from(self.step_len().await?));
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
        let speed = self.speed_mod.unwrap_or(self.speed);
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
                let speed = self.step_len().await? as f32 / (millis as f32 * 32.0) * STEPS_PER_BEAT as f32;
                defmt::info!("speed: {}", speed);
                self.speed = speed;
                let offset = (self.file_offset()? / self.step_len().await?) * self.step_len().await? + crate::lock_async_ref!(ANCHOR);
                defmt::info!("offset: {}", offset);
                self.file_seek_from_start(offset)?;
            }
        } else {
            ANCHOR.lock().await.replace(self.file_offset()? % self.step_len().await?);
        }
        Ok(Some(now))
    }

    pub fn set_speed_mod(&mut self, speed: Option<f32>) {
        self.speed_mod = speed;
    }

    pub async fn set_running(&mut self, running: bool) -> Result<(), Error> {
        self.running = running;
        self.file_seek_from_start(0x2c)
    }

    pub async fn step_len(&self) -> Result<u32, Error> {
        Ok((self.file_length()? - 0x2c) / self.step_count)
    }

    pub fn speed(&self) -> f32 {
        self.speed
    }

    pub fn step_count(&self) -> u32 {
        self.step_count
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
                            let i = (self.file_offset()?.saturating_sub(0x2c)) % (self.step_len().await?) < GRAIN_LEN as u32;
                            defmt::info!("sync: init_seek to {}, i: {}", counter, i);
                            self.file_seek_from_start(0x2c + counter)?;

                            State::Sync
                        },
                        State::Desync {
                            inner: Desync::Loop { .. },
                            step,
                            counter
                        } => {
                            defmt::info!("sync: hold step {}", step);

                            State::Desync { inner: Desync::Hold, step, counter }
                        },
                        _ => State::Sync
                    }
                },
                Event::Hold { step } => {
                    defmt::info!("hold: init_seek...");

                    let counter = self.file_offset()? - 0x2c;
                    self.file_seek_from_start(0x2c + self.step_len().await? * step)?;

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
                    self.file_seek_from_start(0x2c + self.step_len().await? * step)?;

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
            Ok((self.file_offset()?.saturating_sub(0x2c)) % (self.step_len().await? / 16) < (GRAIN_LEN as f32 * self.speed_mod.unwrap_or(self.speed)) as u32)
        } else {
            Ok((self.file_offset()?.saturating_sub(0x2c)) % (self.step_len().await? / 2) < (GRAIN_LEN as f32 * self.speed_mod.unwrap_or(self.speed)) as u32)
        }
    }
}
