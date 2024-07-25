use core::result::Result;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use embassy_stm32::sdmmc::{DataBlock, Instance, Sdmmc, SdmmcDma};
use embassy_stm32::sdmmc::Error as SdmmcError;

use embedded_sdmmc::{Block, BlockDevice, BlockIdx, Timestamp, BlockCount};

pub struct SdioCard<'a, T: Instance, D: SdmmcDma<T>> {
    inner: Mutex<CriticalSectionRawMutex, SdioCardInner<'a, T, D>>
}

impl<'a, T: Instance, D: SdmmcDma<T>> SdioCard<'a, T, D> {
    pub fn new(sdmmc: Sdmmc<'a, T, D>) -> Self {
        Self { inner: Mutex::new(SdioCardInner { sdmmc }) }
    }

    pub async fn num_bytes(&self) -> Result<u64, SdmmcError> {
        self.inner.lock().await.sdmmc.card().map(|c| c.size())
    }
}

impl<'a, T: Instance, D: SdmmcDma<T>> BlockDevice for SdioCard<'a, T, D> {
    type Error = SdmmcError;

    async fn read(
        &self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
        _reason: &str,
    ) -> Result<(), Self::Error> {
        self.inner.lock().await.read(blocks, start_block_idx).await
    }

    async fn write(
        &self,
        blocks: &[Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Self::Error> {
        self.inner.lock().await.write(blocks, start_block_idx).await
    }

    async fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        self.inner.lock().await.sdmmc.card().map(|c| BlockCount(c.csd.block_count()))
    }

    async fn reset(&self) -> Result<(), Self::Error> {
        let mut inner = self.inner.lock().await;

        let freq = inner.sdmmc.clock();
        inner.sdmmc.init_card(freq).await
    }
}

pub struct SdioCardInner<'a, T: Instance, D: SdmmcDma<T>> {
    sdmmc: Sdmmc<'a, T, D>,
}

impl<'a, T: Instance, D: SdmmcDma<T>> SdioCardInner<'a, T, D> {
    async fn read(
        &mut self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), SdmmcError> {
        for block in blocks.iter_mut() {
            let mut buf = DataBlock(block.contents);
            // let mut buf = DataBlock([0; 512]);

            self.sdmmc.read_block(
                start_block_idx.0,
                &mut buf
            ).await?;

            block.contents = buf.0;
        }
        Ok(())
    }

    async fn write(
        &mut self,
        blocks: &[Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), SdmmcError> {
        for block in blocks.iter() {
            self.sdmmc.write_block(
                start_block_idx.0,
                &DataBlock(block.contents)
            ).await?;
        }
        Ok(())
    }
}

#[derive(Default)]
pub struct DummyTimesource();

impl embedded_sdmmc::filesystem::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
