use crate::utils::Fraction;
use super::rhythm::RhythmData;

use alloc::collections::VecDeque;
use alloc::{format, vec};
use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use block_device_adapters::{StreamSlice, StreamSliceError};
use embedded_fatfs::{Dir, Error, File, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, Seek, SeekFrom};
use micromath::F32Ext;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::Sender;

pub type StepsSender<'a, T, const N: usize> = Sender<'a, RawMutex, T, N>;

pub enum Command<'a> {
    SetTempo { tempo: f32 },
    RecordEvents,
    BakeRecording,
    PushEvent { event: Event },
    PushToOneshots { path: String, pad_index: usize },
    PlayOneshot { pad_index: Option<usize> },
    PushToCuts { path: String, step_count: usize, pad_index: usize },
    BakeCuts,
    PushToSequence { pad_index: usize },
    BakeSequence,
    StepCount { sender: StepsSender<'a, usize, 1> },
    Paths { sender: StepsSender<'a, Vec<String>, 1> },
    ReadCut { sender: StepsSender<'a, [u16; crate::GRAIN_LEN], 1> },
}
impl<'a> core::fmt::Debug for Command<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let text = match self {
            Command::SetTempo { .. } => "SetTempo",
            Command::RecordEvents => "RecordEvents",
            Command::BakeRecording => "BakeRecording",
            Command::PushEvent { .. } => "PushEvent",
            Command::PushToOneshots { .. } => "PushToOneshots",
            Command::PlayOneshot { .. } => "PlayOneshot",
            Command::PushToCuts { .. } => "PushToCuts",
            Command::BakeCuts => "BakeCuts",
            Command::PushToSequence { .. } => "PushToSequence",
            Command::BakeSequence => "BakeSequence",
            Command::StepCount { .. } => "StepCount",
            Command::Paths { .. } => "Paths",
            Command::ReadCut { .. } => "ReadCut",
        };
        write!(f, "{}", text)
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug, defmt::Format)]
pub enum Event {
    Sync,
    Hold { step: usize },
    Loop { step: usize, length: Fraction },
}

#[derive(Eq, PartialEq)]
enum Desync {
    Hold,
    Loop { step: usize, length: Fraction },
}

#[derive(PartialEq)]
enum State {
    Sync,
    Desync { inner: Desync, seq_index: usize, sync_step: f32 },
}

#[derive(PartialEq)]
enum LogState {
    Record,
    Recall { active: bool },
}

#[derive(Clone, Debug)]
struct LoggedEvent {
    event: Event,
    call_step: f32,
}

struct EventLog {
    pub state: LogState,
    seqs: VecDeque<Vec<LoggedEvent>>,
    seqs_index: usize,
}

impl EventLog {
    pub fn new() -> Self {
        Self {
            state: LogState::Record,
            seqs: VecDeque::from(vec![Vec::new()]),
            seqs_index: 0,
        }
    }

    pub fn advance_seq(&mut self) {
        if self.state == LogState::Record {
            self.seqs.push_back(Vec::new());
        } else {
            self.seqs_index += 1;
            self.seqs_index %= self.seqs.len();
        }
        defmt::info!("seqs: {}", defmt::Debug2Format(&self.seqs));
    }

    pub fn try_push(&mut self, step_offset: f32, event: Event) {
        if self.state == LogState::Record {
            if let Some(seq) = self.seqs.back_mut() {
                seq.push(LoggedEvent {
                    event,
                    call_step: step_offset,
                });
            }
        }
    }

    pub fn try_pop(&mut self, step_offset: f32) -> Option<Event> {
        if matches!(self.state, LogState::Recall { active: true }) {
            return self.seqs.get_mut(self.seqs_index).and_then(|e| {
                e
                    .iter()
                    .rfind(|e| {
                        e.call_step > step_offset - 1.0 / 16.0
                            && e.call_step < step_offset
                    })
                    .map(|e| e.event)
            });
        }
        None
    }

    pub fn bake(&mut self) {
        // compact sequences starting on pickup
        let start_step = self.seqs.front().and_then(|s| s.first().map(|e| e.call_step));
        let end_step = self.seqs.back().and_then(|s| s.last().map(|e| e.call_step));
        if start_step.is_some_and(|s| end_step.is_some_and(|e| e < s))
            && self.seqs.len() > 1
        {
            let first = self.seqs.pop_front().unwrap();
            self.seqs.back_mut().unwrap().extend_from_slice(&first);
        }
        self.seqs_index = self.seqs.len() - 1;
        self.state = LogState::Recall { active: true };
    }
}

struct SequenceBuilder {
    pad_indices: Vec<usize>,
}

impl SequenceBuilder {
    pub fn new() -> Self {
        Self { pad_indices: Vec::new() }
    }
}

struct CutsBuilder<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    cuts_indices: Vec<(Cut<'a, IO, TP, OCC>, usize)>,
    paths_remainders: Vec<(String, usize)>,
}

impl<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> CutsBuilder<'a, IO, TP, OCC> {
    pub fn new() -> Self {
        Self {
            cuts_indices: Vec::new(),
            paths_remainders: Vec::new(),
        }
    }

    pub async fn push(
        &mut self,
        path: &str,
        file: File<'a, IO, TP, OCC>,
        file_len: u64,
        file_rhythm: RhythmData,
        cut_step_count: usize,
        pad_index: usize,
    ) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        let remainder = if let Some((_, rem)) = self.paths_remainders.iter_mut().find(|(p, _)| p.as_str() == path) {
            if *rem == 0 {
                *rem = file_rhythm.step_count();
            }
            rem
        } else {
            self.paths_remainders.push((path.to_string(), file_rhythm.step_count()));
            let (_, rem) = self.paths_remainders.last_mut().unwrap();
            rem
        };

        let cut_step_count = cut_step_count.min(*remainder);

        let step_offset = file_rhythm.step_count() - *remainder;
        let step_len = (file_len - 0x2c) / file_rhythm.step_count() as u64;

        let start_offset = 0x2c + step_offset as u64 * step_len;
        let end_offset = 0x2c + (step_offset + cut_step_count) as u64 * step_len;

        let slice = StreamSlice::new(
            file.clone(),
            start_offset,
            end_offset,
        ).await?;
        let cut = Cut {
            slice,
            slice_len: end_offset - start_offset,
            slice_step_count: cut_step_count,
            file_rhythm,
        };
        self.cuts_indices.push((cut, pad_index));

        *remainder -= cut_step_count;

        Ok(())
    }
}

struct Cut<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    slice: StreamSlice<File<'a, IO, TP, OCC>>,
    slice_len: u64,
    slice_step_count: usize,
    file_rhythm: RhythmData,
}

pub struct Steps<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    root: Dir<'a, IO, TP, OCC>,
    metadatas: Vec<super::WavMetadata>,
    cuts_builder: Option<CutsBuilder<'a, IO, TP, OCC>>,
    cuts: [Option<Cut<'a, IO, TP, OCC>>; crate::PAD_COUNT],
    seq_builder: Option<SequenceBuilder>,
    seq: Vec<usize>,
    seq_index: usize,
    anchor_tempo: Option<f32>,
    sync_speed: f32,
    oneshots: [Option<StreamSlice<File<'a, IO, TP, OCC>>>; crate::PAD_COUNT],
    oneshot_index: Option<usize>,
    state: State,
    event_buf: Option<Event>,
    event_log: Option<EventLog>,
    led: embassy_stm32::gpio::Output<'a>,
}

impl<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> Steps<'a, IO, TP, OCC> {
    async fn paths_recursive(dir: &Dir<'a, IO, TP, OCC>) -> Result<Vec<String>, Error<IO::Error>> {
        let mut children = Vec::new();
        let mut iter = dir.iter();
        while let Some(entry) = iter.next().await {
            let entry = entry?;
            let ucs2 = entry.long_file_name_as_ucs2_units();
            if let Some(Ok(name)) = ucs2.map(alloc::string::String::from_utf16) {
                if !entry.attributes().contains(embedded_fatfs::FileAttributes::HIDDEN)
                    && !name.starts_with(".")
                {
                    if entry.is_dir() {
                        let grandchildren = Box::pin(Self::paths_recursive(&dir.open_dir(&name).await?)).await?;
                        for grandchild in grandchildren {
                            children.push(format!("{}/{}", name, grandchild));
                        }
                    }
                    children.push(name);
                }
            }
        }
        Ok(children)
    }

    pub async fn new(mut led: embassy_stm32::gpio::Output<'a>, root: Dir<'a, IO, TP, OCC>) -> Result<Self, Error<IO::Error>> {
        for _ in 0..4 {
            led.toggle();
            embassy_time::Timer::after_millis(100).await;
        }

        let paths = Self::paths_recursive(&root).await?;
        defmt::info!("paths: {}", defmt::Debug2Format(&paths));

        let mut metadatas = Vec::new();
        for path in paths {
            match root.open_file(&path).await {
                Err(Error::InvalidInput) => (),
                Err(e) => return Err(e),
                Ok(mut file) => {
                    let len = root.open_meta(&path).await?.len();
                    let rhythm = super::rhythm::RhythmData::new(&mut file, len).await?;

                    metadatas.push(super::WavMetadata { path, rhythm });

                    for _ in 0..4 {
                        led.toggle();
                        embassy_time::Timer::after_millis(50).await;
                    }
                }
            }
        }
        for _ in 0..4 {
            led.toggle();
            embassy_time::Timer::after_millis(100).await;
        }

        Ok(Self {
            root,
            metadatas,
            cuts_builder: None,
            cuts: core::array::from_fn(|_| None),
            seq_builder: None,
            seq: Vec::new(),
            seq_index: 0,
            anchor_tempo: None,
            sync_speed: 1.0,
            oneshots: core::array::from_fn(|_| None),
            oneshot_index: None,
            state: State::Sync,
            event_buf: None,
            event_log: None,
            led,
        })
    }

    pub fn set_tempo(&mut self, tempo: f32) -> Result<(), Error<IO::Error>> {
        self.anchor_tempo = Some(tempo);
        self.sync_tempo()
    }

    pub async fn push_event(&mut self, event: Event) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        self.event_buf = Some(event);
        let step_offset = match self.state {
            State::Desync { sync_step, .. } => sync_step,
            _ => self.seq_step_offset().await?,
        };
        if let Some(event_log) = self.event_log.as_mut() {
            event_log.try_push(step_offset, event);

            if let LogState::Recall { ref mut active } = event_log.state {
                *active = matches!(event, Event::Sync);
            }
        }

        Ok(())
    }

    pub async fn recall_event(&mut self) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        let step_offset = match self.state {
            State::Desync { sync_step, .. } => sync_step,
            _ => self.seq_step_offset().await?,
        };
        if let Some(event_log) = self.event_log.as_mut() {
            // advance recording on synced sequence loop
            if step_offset < 1.0 / 16.0 {
                event_log.advance_seq();
            }
            // recall event recording
            if self.event_buf.is_none() {
                self.event_buf = event_log.try_pop(step_offset);
                if let Some(event) = self.event_buf {
                    defmt::info!("step: {} | event: {}", step_offset, event);
                }
            }
        }
        Ok(())
    }

    pub fn record_events(&mut self) {
        self.event_log = Some(EventLog::new());
    }

    pub fn bake_recording(&mut self) {
        if let Some(event_log) = self.event_log.as_mut() {
            event_log.bake();
        }
        if self.event_log.as_ref().is_some_and(|e| {
            e.seqs.iter().all(|s| s.is_empty())
        }) {
            self.event_log = None;
        }
    }

    pub async fn push_to_oneshots(&mut self, path: String, pad_index: usize) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        let file = self.root.open_file(&path).await?;
        let len = self.root.open_meta(&path).await?.len();
        let stream = StreamSlice::new(file, 0x2c, len).await?;

        if let Some(oneshot) = self.oneshots.get_mut(pad_index) {
            *oneshot = Some(stream);
        }
        Ok(())
    }

    pub async fn play_oneshot(&mut self, pad_index: Option<usize>) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        if let Some(pad_index) = pad_index {
            if pad_index < self.oneshots.len() {
                self.oneshot_index = Some(pad_index);
                if let Some(Some(oneshot)) = self.oneshots.get_mut(pad_index) {
                    oneshot.rewind().await?;
                }
            }
        } else {
            self.oneshot_index = None;
        }
        Ok(())
    }

    pub async fn read_oneshot(&mut self) -> Result<Box<[u8]>, StreamSliceError<Error<IO::Error>>> {
        let mut output = vec![u8::MAX / 2; self.grain_len().unwrap_or(512)].into_boxed_slice();

        if let Some(Some(oneshot)) = self.oneshot_index.and_then(|i| self.oneshots.get_mut(i)) {
            if oneshot.read_exact(&mut output).await.is_err() {
                self.oneshot_index = None;
            }
        } else {
            return Err(StreamSliceError::Other(Error::NotFound));
        }

        Ok(output)
    }

    /// buffer assignment of cut to specified pad
    pub async fn push_to_cuts(&mut self, path: String, step_count: usize, pad_index: usize) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        let file_rhythm = self.metadatas
            .iter()
            .find(|m| m.path == path)
            .map(|m| m.rhythm);
        if let Some(file_rhythm) = file_rhythm {
            let builder = self.cuts_builder.get_or_insert_with(CutsBuilder::new);
            builder.push(
                &path,
                self.root.open_file(&path).await?,
                self.root.open_meta(&path).await?.len(),
                file_rhythm,
                step_count,
                pad_index,
            ).await?;

            return Ok(());
        }
        Err(StreamSliceError::Other(Error::NotFound))
    }

    /// assign buffered cuts to pads
    pub fn bake_cuts(&mut self) -> Result<(), Error<IO::Error>> {
        if let Some(cuts_buf) = self.cuts_builder.take().map(|b| b.cuts_indices) {
            let cuts_buf = cuts_buf
                .into_iter()
                .filter(|&(_, i)| i < self.cuts.len())
                .collect::<Vec<_>>();
            for (cut, pad_idx) in cuts_buf.into_iter() {
                self.cuts[pad_idx] = Some(cut);
            }

            self.state = State::Sync;
            self.event_buf = None;
            defmt::info!("assigned to pads!");

            self.sync_tempo()?;
        }
        Ok(())
    }

    /// push pad index to sequence buffer
    pub fn push_to_sequence(&mut self, pad_index: usize) {
        let builder = self.seq_builder.get_or_insert_with(SequenceBuilder::new);
        builder.pad_indices.push(pad_index);
    }

    /// build sequence from buffered pad indices
    #[allow(clippy::unwrap_or_default)]
    pub async fn bake_sequence(&mut self) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        let pad_indices = self.seq_builder
            .take()
            .map(|b| b.pad_indices)
            .unwrap_or(Vec::new())
            .into_iter()
            .filter(|&i| i < self.cuts.len())
            .collect::<Vec<_>>();
        self.seq = pad_indices;

        self.state = State::Sync;
        self.event_buf = None;

        defmt::info!("loaded sequence {}!", defmt::Debug2Format(&self.seq));

        if self.cut_ref().is_none() {
            self.advance_seq().await?;
        }
        self.sync_tempo()?;

        Ok(())
    }

    pub fn step_count(&self) -> Result<usize, Error<IO::Error>> {
        self.cut_ref().map(|c| c.slice_step_count).ok_or(Error::NotFound)
    }

    pub fn paths(&self) -> Vec<String> {
        self.metadatas.iter().map(|m| m.path.clone()).collect::<Vec<_>>()
    }

    /// read grain of current cut(s) in sequence
    pub async fn read_cut(&mut self, max_duty: u16, buffer: &mut [u16]) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        // handle event recording
        if self.event_log.is_some() {
            self.recall_event().await?;
        }

        // handle state
        if self.is_quantum().await? {
            self.led.toggle();
            self.update().await?;

            // loop
            if let State::Desync {
                inner: Desync::Loop { step, length }, seq_index, ..
            } = self.state {
                let step_offset = self.seq_step_offset().await?;
                let step_count = self.steps_until_seq_index(self.seq.len()).ok_or(Error::NotFound)? as f32;

                let start = (self.steps_until_seq_index(seq_index).ok_or(Error::NotFound)? + step) as f32;
                let end = f32::from(length + start as u32);

                if step_offset > end
                    || step_offset < start
                    && step_offset  > (start + end) % step_count
                {
                    self.index_seq(seq_index).await?;

                    let offset = step as u64 * self.step_len().ok_or(Error::NotFound)? as u64;
                    self.cut_mut().ok_or(Error::NotFound)?.slice.seek(SeekFrom::Start(offset)).await?;
                }
            }
        }

        let mut bytes = vec![u8::MAX / 2; (buffer.len() as f32 * self.speed()) as usize].into_boxed_slice();

        // read cut(s) in seq until buffer full
        let mut read = match self.cut_mut() {
            Some(c) => Some(c.slice.read_exact(&mut bytes).await),
            None => None,
        };
        while read.as_ref().is_some_and(|r| r.is_err()) {
            self.advance_seq().await?;
            read = match self.cut_mut() {
                Some(c) => Some(c.slice.read_exact(&mut bytes).await),
                None => None,
            };
        }

        // resample via linear interpolation
        for (buf_idx, buf) in buffer.iter_mut().enumerate() {
            let bytes_idx = buf_idx as f32 * self.speed();
            *buf = ((bytes_idx.fract() * *bytes.get(bytes_idx as usize).unwrap_or(&(u8::MAX / 2)) as f32
                + (1.0 - bytes_idx.fract()) * *bytes.get(bytes_idx as usize + 1).unwrap_or(&(u8::MAX / 2)) as f32
            ) / u8::MAX as f32 * max_duty as f32) as u16;
        }

        // sync desync
        let step_len = self.step_len().ok_or(Error::NotFound)?;
        let seq_step_count = self.steps_until_seq_index(self.seq.len()).ok_or(Error::NotFound)? as f32;
        if let State::Desync { ref mut sync_step, .. } = self.state {
            let sync_seq_index = {
                let mut step_count = 0;
                self.seq
                    .iter()
                    .enumerate()
                    .find(|(_, &cuts_idx)| {
                        step_count += self.cuts.get(cuts_idx).and_then(|c| {
                            c.as_ref().map(|c| c.slice_step_count)
                        }).unwrap_or(0);
                        step_count > *sync_step as usize
                    })
                    .map(|(seq_idx, _)| seq_idx)
            }.unwrap();
            let sync_tempo = self.cuts.get(sync_seq_index).and_then(|c| {
                c.as_ref().map(|c| c.file_rhythm.tempo())
            }).ok_or(Error::NotFound)?;
            *sync_step += bytes.len() as f32
                / self.sync_speed
                * self.anchor_tempo.ok_or(Error::NotFound)? / sync_tempo
                / step_len as f32;
            *sync_step %= seq_step_count;
        }

        Ok(())
    }

    async fn advance_seq(&mut self) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        let init = self.seq_index;
        if let Some(cut) = self.cut_mut() {
            cut.slice.rewind().await?;
        }

        for addend in 1..=self.seq.len() {
            let i = (init + addend) % self.seq.len();
            if self.cuts.get(self.seq[i]).is_some_and(|c| c.is_some()) {
                return self.index_seq(i).await;
            }
        }
        Err(StreamSliceError::Other(Error::NotFound))
    }

    async fn index_seq(&mut self, index: usize) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        if index < self.cuts.len() {
            self.seq_index = index;
            if let Some(cut) = self.cut_mut() {
                cut.slice.rewind().await?;
                self.sync_tempo()?;

                return Ok(());
            }
        }
        Err(StreamSliceError::Other(Error::InvalidInput))
    }

    fn sync_tempo(&mut self) -> Result<(), Error<IO::Error>> {
        let cut_tempo = self.cut_ref().ok_or(Error::NotFound)?.file_rhythm.tempo();
        if let Some(anchor) = self.anchor_tempo {
            // sync to global tempo
            self.sync_speed = anchor / cut_tempo;
        } else {
            // anchor to global tempo
            self.anchor_tempo = Some(cut_tempo);
            defmt::info!("anchor tempo: {}", cut_tempo);
        }
        Ok(())
    }

    async fn update(&mut self) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        if let Some(event) = self.event_buf.as_ref() {
            self.state = match event {
                Event::Sync => {
                    if let State::Desync { sync_step, .. } = self.state {
                        let seq_index = (0..self.seq.len())
                            .filter(|&i| {
                                self.steps_until_seq_index(i).is_some_and(|s| s as f32 <= sync_step)
                            })
                            .last()
                            .ok_or(Error::NotFound)?;
                        self.index_seq(seq_index).await?;
                        let step_offset = sync_step - self.steps_until_seq_index(seq_index).ok_or(Error::NotFound)? as f32;

                        let offset = (step_offset * self.step_len().ok_or(Error::NotFound)? as f32) as u64;
                        self.cut_mut().ok_or(Error::NotFound)?.slice.seek(SeekFrom::Start(offset)).await?;
                    }
                    State::Sync
                },
                &Event::Hold { step } => {
                    let (seq_index, sync_step) = match self.state {
                        State::Desync { seq_index, sync_step, .. } => (seq_index, sync_step),
                        _ => {
                            let sync_step = self.seq_step_offset().await?;

                            let offset = (step as f32 * self.step_len().ok_or(Error::NotFound)? as f32) as u64;
                            self.cut_mut().ok_or(Error::NotFound)?.slice.seek(SeekFrom::Start(offset)).await?;

                            (self.seq_index, sync_step)
                        },
                    };
                    
                    State::Desync { inner: Desync::Hold, seq_index, sync_step }
                },
                &Event::Loop { step, length } => {
                    let (seq_index, sync_step) = match self.state {
                        State::Desync { seq_index, sync_step, .. } => (seq_index, sync_step),
                        _ => {
                            let sync_step = self.seq_step_offset().await?;
                            (self.seq_index, sync_step)
                        },
                    };
                    let offset = (step as f32 * self.step_len().ok_or(Error::NotFound)? as f32) as u64;
                    self.cut_mut().ok_or(Error::NotFound)?.slice.seek(SeekFrom::Start(offset)).await?;

                    State::Desync {
                        inner: Desync::Loop { step, length },
                        seq_index,
                        sync_step,
                    }
                },
            };
            self.event_buf = None;
        }

        Ok(())
    }

    async fn is_quantum(&mut self) -> Result<bool, StreamSliceError<Error<IO::Error>>> {
        let pos = self.cut_mut().ok_or(Error::NotFound)?.slice.stream_position().await?;
        let divisor = if matches!(self.event_buf, Some(Event::Loop { .. }))
            || matches!(self.state, State::Desync { inner: Desync::Loop { .. }, .. })
        { 16 } else { 2 };
        Ok(pos % (self.step_len().ok_or(Error::NotFound)? as u64 / divisor) < (self.grain_len().ok_or(Error::NotFound)? as f32 * self.speed()) as u64)
    }

    fn speed(&self) -> f32 {
        self.sync_speed
    }

    fn step_len(&self) -> Option<usize> {
        Some(self.cut_ref()?.slice_len as usize / self.cut_ref()?.slice_step_count)
    }

    fn grain_len(&self) -> Option<usize> {
        Some(self.step_len()? / 16)
    }

    fn steps_until_seq_index(&self, index: usize) -> Option<usize> {
        if let Some(indices) = self.seq.get(..index) {
            return Some(indices
                .iter()
                .fold(0, |acc, &i| {
                    if let Some(Some(cut)) = self.cuts.get(i) {
                        return acc + cut.slice_step_count;
                    }
                    acc
                }));
        }
        None
    }

    async fn seq_step_offset(&mut self) -> Result<f32, StreamSliceError<Error<IO::Error>>> {
        Ok(
            self.steps_until_seq_index(self.seq_index).ok_or(Error::NotFound)? as f32
                + self.cut_mut().ok_or(Error::NotFound)?.slice.stream_position().await? as f32
                / self.step_len().ok_or(Error::NotFound)? as f32
        )
    }

    fn cut_mut(&mut self) -> Option<&mut Cut<'a, IO, TP, OCC>> {
        if let Some(opt) = self.seq.get(self.seq_index).and_then(|&i| self.cuts.get_mut(i)) {
            return opt.as_mut();
        }
        None
    }

    fn cut_ref(&self) -> Option<&Cut<'a, IO, TP, OCC>> {
        if let Some(opt) = self.seq.get(self.seq_index).and_then(|&i| self.cuts.get(i)) {
            return opt.as_ref();
        }
        None
    }
}
