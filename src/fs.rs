use crate::sdio::Sdio;
use crate::sdio::Error as SdioError;

use block_device_adapters::{
    BufStream,
    BufStreamError,
    StreamSlice,
    StreamSliceError,
};
use block_device_driver::BlockDevice;
use embedded_io_async::{Read, ReadExactError, Seek, SeekFrom};

use embassy_rp::pio::Instance;

#[derive(Debug)]
pub enum Error {
    ReadExact(ReadExactError<BufStreamError<SdioError>>),
    StreamSlice(StreamSliceError<BufStreamError<SdioError>>),
}
impl From<BufStreamError<SdioError>> for Error {
    fn from(value: BufStreamError<SdioError>) -> Self {
        Self::StreamSlice(value.into())
    }
}
impl From<ReadExactError<BufStreamError<SdioError>>> for Error {
    fn from(value: ReadExactError<BufStreamError<SdioError>>) -> Self {
        Self::ReadExact(value)
    }
}
impl From<StreamSliceError<BufStreamError<SdioError>>> for Error {
    fn from(value: StreamSliceError<BufStreamError<SdioError>>) -> Self {
        Self::StreamSlice(value)
    }
}

pub struct SdioDevice<'d, T: Instance> {
    sdio: Sdio<'d, T>,
}

impl<'d, T: Instance> SdioDevice<'d, T> {
    pub async fn into_stream_slice(self) -> Result<StreamSlice<BufStream<Self, 512>>, Error> {
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

    pub fn new(sdio: Sdio<'d, T>) -> Self {
        Self { sdio }
    }
}

impl<'d, T: Instance> BlockDevice<512> for SdioDevice<'d, T> {
    type Error = SdioError;
    type Align = aligned::A4;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        for block in data.iter_mut() {
            self.sdio.read_block(block_address, block).await?;
        }

        Ok(())
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[aligned::Aligned<Self::Align, [u8; 512]>],
    ) -> Result<(), Self::Error> {
        defmt::unimplemented!()
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        Ok(self.sdio.block_count())
    }
}
