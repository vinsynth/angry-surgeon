use crate::fs::VolMgr;
use rhythm::RhythmData;

use alloc::string::ToString;
use alloc::vec::Vec;

use defmt::Format;

use embedded_sdmmc::{Mode, RawDirectory, RawFile};
use embedded_sdmmc::filesystem::ShortFileName;

use embassy_stm32::time::Hertz;

pub const SAMPLE_RATE: Hertz = Hertz::khz(32);

#[allow(clippy::enum_variant_names)]
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

#[derive(Debug, Clone)]
pub struct WavWriter {
    name: ShortFileName,
    file: RawFile,
}

impl WavWriter {
    pub async fn new(
        vol_mgr: &mut VolMgr<'_>,
        directory: &RawDirectory,
        name: &str,
    ) -> Result<Self, Error> {
        // try to add number to end of name as needed to avoid duplicate filenames
        let mut prev_indices = Vec::new();
        vol_mgr.iterate_dir(*directory, |f| {
            if let Ok(Some(suffix)) = core::str::from_utf8(f.name.base_name())
                .map(|n| n.strip_prefix(name))
            {
                if suffix.is_empty() {
                    prev_indices.push(None);
                } else if let Ok(index) = suffix.parse::<u32>() {
                    prev_indices.push(Some(index));
                }
            }
        }).await?;
        prev_indices.sort();

        let index = if prev_indices.is_empty() || !prev_indices.contains(&None) {
            None
        } else {
            let max = prev_indices.iter().filter_map(|&v| v).max().unwrap_or(0);

            (0..=max + 1)
                .find(|&v| !prev_indices.contains(&Some(v)))
        };

        let mut name = name.to_string();
        if let Some(index) = index {
            name.push_str(&index.to_string());
        }
        name.push_str(".wav");
        let name = ShortFileName::create_from_str_mixed_case(&name)
            .map_err(|e| Error::SdmmcError(embedded_sdmmc::Error::FilenameError(e)))?;

        let file = vol_mgr.open_file_in_dir(*directory, name.clone(), Mode::ReadWriteCreate).await?;

        // write placeholder WAV header to output file
        let header = WavHeader::empty();
        vol_mgr.write(file, &header).await?;

        Ok(Self {
            name,
            file,
        })
    }

    pub async fn write(&mut self, vol_mgr: &mut VolMgr<'_>, samples: &[u8]) -> Result<(), Error> {
        Ok(vol_mgr.write(self.file, samples).await?)
    }

    pub async fn release(self, vol_mgr: &mut VolMgr<'_>, directory: &RawDirectory) -> Result<Wav, Error> {
        // rewrite header to include file & PCM size
        vol_mgr.file_seek_from_start(self.file, 0)?;
        let header = WavHeader::sized(vol_mgr, self.file)?;
        vol_mgr.write(self.file, &header).await?;

        defmt::info!("acutal sample count: {}", vol_mgr.file_length(self.file)? - 44);

        vol_mgr.close_file(self.file).await?;
        let file = vol_mgr.open_file_in_dir(*directory, self.name.clone(), Mode::ReadOnly).await?;
        Wav::new(vol_mgr, self.name, file).await
    }
}

/// WAV file header creator
#[derive(Debug, Clone, Copy, Format)]
struct WavHeader;

impl WavHeader {
    /// WAV file header for empty PCM
    pub fn empty() -> [u8; 44] {
        let riff_chunk = [
            "RIFF".as_bytes(),
            &36u32.to_le_bytes(), // file size - 8 bytes (so header len - 8)
            "WAVE".as_bytes(),
        ].concat();

        let channel_count = 1;
        let bits_per_sample: u16 = 8;
        let bytes_per_block: u16 = channel_count * bits_per_sample / 8;
        let bytes_per_sec: u32 = SAMPLE_RATE.0 * bytes_per_block as u32;
        let format_chunk = [
            "fmt ".as_bytes(),
            &16u32.to_le_bytes(), // block size in bytes
            &1u16.to_le_bytes(), // data format = PCM integer
            &1u16.to_le_bytes(), // number of channels
            &SAMPLE_RATE.0.to_le_bytes(),
            &bytes_per_sec.to_le_bytes(),
            &bytes_per_block.to_le_bytes(),
            &bits_per_sample.to_le_bytes(), // 8 bits/sample
        ].concat();

        let data_chunk_header = [
            "data".as_bytes(),
            &0u32.to_le_bytes(), // data size (initially 0)
        ].concat();

        crate::expect!(
            riff_chunk
                .into_iter()
                .chain(format_chunk)
                .chain(data_chunk_header)
                .collect::<Vec<_>>()
                .try_into(),
            "failed to write header"
        )
    }

    pub fn sized(vol_mgr: &VolMgr<'_>, file: RawFile) -> Result<[u8; 44], Error> {
        let file_size = vol_mgr.file_length(file)? - 8; // file len  minus 8 bytes
        let data_size = vol_mgr.file_length(file)? - 44; // PCM len

        let mut header = WavHeader::empty();

        header[4..8].copy_from_slice(&file_size.to_le_bytes());
        header[40..44].copy_from_slice(&data_size.to_le_bytes());

        Ok(header)
    }
}

/// audio file with additional information
#[derive(Debug, Clone)]
pub struct Wav {
    pub name: ShortFileName,
    pub file: RawFile,
    pub rhythm: RhythmData,
}

impl Wav {
    pub async fn new(vol_mgr: &mut VolMgr<'_>, name: ShortFileName, file: RawFile) -> Result<Self, Error> {
        Ok(Self {
            name,
            file,
            rhythm: RhythmData::new(vol_mgr, &file).await?,
        })
    }
}

// pub mod record;
pub mod steps;
mod rhythm;
