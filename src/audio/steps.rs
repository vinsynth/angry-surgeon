use super::rhythm::RhythmData;

use alloc::{format, vec};
use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;

use block_device_adapters::{StreamSlice, StreamSliceError};
use embedded_fatfs::{Dir, Error, File, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, Seek, SeekFrom};
use micromath::F32Ext;

struct Cut<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    slice: StreamSlice<File<'a, IO, TP, OCC>>,
    step_length: f32,
}

pub struct Steps<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    root: Dir<'a, IO, TP, OCC>,
    metadatas: Vec<super::WavMetadata>,
    cuts: Vec<Cut<'a, IO, TP, OCC>>,
    cut_index: usize,
    anchor_step_length: Option<f32>,
}

impl<'a, IO, TP, OCC> Steps<'a, IO, TP, OCC>
where
    IO: ReadWriteSeek,
    TP: TimeProvider,
    OCC: OemCpConverter,
{
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

    pub async fn new(root: Dir<'a, IO, TP, OCC>) -> Result<Self, Error<IO::Error>> {
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
                    defmt::info!("{} pulses: {}", rhythm.pulses.len(), defmt::Debug2Format(&rhythm.pulses[..8]));
                    defmt::info!("tempo: {} || step_length: {}", rhythm.tempo, rhythm.step_length);

                    metadatas.push(super::WavMetadata { path, rhythm });
                }
            }
        }

        let file = root.open_file(&metadatas[0].path).await?;
        let len = root.open_meta(&metadatas[0].path).await?.len();
        let slice = StreamSlice::new(file, 44, len).await.unwrap();

        let cut = Cut { slice, step_length: metadatas[0].rhythm.step_length };
        let step_length = cut.step_length;

        Ok(Self {
            root,
            metadatas,
            cuts: vec![cut],
            cut_index: 0,
            anchor_step_length: Some(step_length),
        })
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        if self.cut_ref().is_none() {
            return Err(Error::NotFound.into());
        }

        let speed = self.cut_ref().unwrap().step_length / self.anchor_step_length.unwrap();
        let mut pcm_buffer = vec![0; (buffer.len() as f32 * speed) as usize + 2].into_boxed_slice();

        let mut read = self.cut_mut().unwrap().slice.read_exact(&mut pcm_buffer).await;
        while read.is_err() {
            self.advance_cuts().await?;
            read = self.cut_mut().unwrap().slice.read_exact(&mut pcm_buffer).await;
        }
        // resync from reading extra word for interpolation
        self.cut_mut().unwrap().slice.seek(SeekFrom::Current(-2)).await?;

        // resample via linear interpolation
        for word_idx in 0..buffer.len() / 2 {
            let pcm_idx = word_idx as f32 * speed;
            let mut i16_buffer = [0; 2];

            i16_buffer.copy_from_slice(&pcm_buffer[pcm_idx as usize * 2..][0..2]);
            let word_a = i16::from_le_bytes(i16_buffer);
            i16_buffer.copy_from_slice(&pcm_buffer[pcm_idx as usize * 2..][2..4]);
            let word_b = i16::from_le_bytes(i16_buffer);

            let interpolated = pcm_idx.fract() * word_a as f32 + (1. - pcm_idx.fract()) * word_b as f32;
            buffer[word_idx * 2..][0..2].copy_from_slice(&(interpolated as i16).to_le_bytes());
        }

        Ok(())
    }

    async fn advance_cuts(&mut self) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        if self.cuts.is_empty() {
            return Err(StreamSliceError::Other(Error::NotFound));
        }
        self.index_cuts((self.cut_index + 1) % self.cuts.len()).await
    }

    async fn index_cuts(&mut self, index: usize) -> Result<(), StreamSliceError<Error<IO::Error>>> {
        self.cut_index = index;
        if let Some(cut) = self.cut_mut() {
            cut.slice.rewind().await?;
            self.sync_tempo();

            return Ok(());
        }
        Err(StreamSliceError::Other(Error::NotFound))
    }

    fn sync_tempo(&mut self) {
        if self.anchor_step_length.is_none() {
            self.anchor_step_length = self.cut_ref().map(|c| c.step_length);
        }
    }

    fn cut_mut(&mut self) -> Option<&mut Cut<'a, IO, TP, OCC>> {
        self.cuts.get_mut(self.cut_index)
    }

    fn cut_ref(&mut self) -> Option<&Cut<'a, IO, TP, OCC>> {
        self.cuts.get(self.cut_index)
    }
}
