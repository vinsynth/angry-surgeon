use block_device_adapters::{BufStream, BufStreamError, StreamSlice, StreamSliceError};
use block_device_driver::BlockDevice;
use embedded_io_async::{Read, Seek, SeekFrom};

use embassy_stm32::sdmmc::{DataBlock, Instance, Sdmmc, SdmmcDma};
use embassy_stm32::sdmmc::Error as SdmmcError;

#[derive(Debug)]
pub enum SdioCardError {
    ReadExact(embedded_io_async::ReadExactError<BufStreamError<SdmmcError>>),
    StreamSlice(StreamSliceError<BufStreamError<SdmmcError>>),
}

impl From<BufStreamError<SdmmcError>> for SdioCardError {
    fn from(value: BufStreamError<SdmmcError>) -> Self {
        Self::StreamSlice(StreamSliceError::from(value))
    }
}
impl From<embedded_io_async::ReadExactError<BufStreamError<SdmmcError>>> for SdioCardError {
    fn from(value: embedded_io_async::ReadExactError<BufStreamError<SdmmcError>>) -> Self {
        Self::ReadExact(value)
    }
}
impl From<StreamSliceError<BufStreamError<SdmmcError>>> for SdioCardError {
    fn from(value: StreamSliceError<BufStreamError<SdmmcError>>) -> Self {
        Self::StreamSlice(value)
    }
}

pub struct SdioCard<'a, T: Instance, D: SdmmcDma<T>> {
    sdmmc: Sdmmc<'a, T, D>,
}

impl<'a, T: Instance, D: SdmmcDma<T>> SdioCard<'a, T, D> {
    pub fn new(sdmmc: Sdmmc<'a, T, D>) -> Self {
        Self { sdmmc }
    }

    pub async fn into_stream_slice(self) -> Result<StreamSlice<BufStream<SdioCard<'a, T, D>, 512>>, SdioCardError> {
        let mut stream = BufStream::new(self);

        let mut buf = [0u8; 8];
        // assume LBA block/sector size = 512 bytes
        let _ = stream.seek(SeekFrom::Start(512 + 0x48)).await?;
        stream.read_exact(&mut buf).await?;
        let partition_array_lba = u64::from_le_bytes(buf);

        let _ = stream
            .seek(SeekFrom::Start(partition_array_lba * 512 + 0x20))
            .await?;
        stream.read_exact(&mut buf).await?;
        let partition_lba_first = u64::from_le_bytes(buf);
        stream.read_exact(&mut buf).await?;
        let partition_lba_last = u64::from_le_bytes(buf); // inclusive

        stream.rewind().await?;

        Ok(StreamSlice::new(
            stream,
            partition_lba_first * 512,
            (partition_lba_last + 1) * 512, // exclusive
        ).await?)
    }
}

impl <'a, T: Instance, D: SdmmcDma<T>> BlockDevice<512> for SdioCard<'a, T, D> {
    type Error = SdmmcError;
    type Align = aligned::A4;

    async fn read(
        &mut self,
        mut block_address: u32,
        data: &mut [aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        for block in data.iter_mut() {
            let mut buf = DataBlock(**block);

            self.sdmmc.read_block(
                block_address,
                &mut buf,
            ).await?;

            **block = buf.0;
            block_address += block.len() as u32;
        }
        Ok(())
    }

    async fn write(
        &mut self,
        mut block_address: u32,
        data: &[aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        for block in data.iter() {
            self.sdmmc.write_block(
                block_address,
                &DataBlock(**block),
            ).await?;

            block_address += block.len() as u32;
        }
        Ok(())
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        Ok(self.sdmmc.card()?.size())
    }
}
