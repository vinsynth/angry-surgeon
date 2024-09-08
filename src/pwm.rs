use embassy_stm32::Peripheral;
use embassy_stm32::dma::word::Word;
use embassy_stm32::dma::WritableRingBuffer;
use embassy_stm32::pac::timer::vals::Ccds;
use embassy_stm32::time::Hertz;

use embassy_stm32::timer;
use timer::GeneralInstance4Channel;
use timer::Channel;
use timer::low_level::{OutputCompareMode, Timer as LLTimer};
use timer::simple_pwm::{Ch1, Ch2, Ch3, Ch4, PwmPin};

#[derive(Debug, PartialEq)]
pub struct OverrunError;

pub struct RingBufferedPwm<'d, T: GeneralInstance4Channel, W: Word> {
    tim: LLTimer<'d, T>,
    ch1: Option<WritableRingBuffer<'d, W>>,
    ch2: Option<WritableRingBuffer<'d, W>>,
    ch3: Option<WritableRingBuffer<'d, W>>,
    ch4: Option<WritableRingBuffer<'d, W>>,
}

impl<'d, T: GeneralInstance4Channel, W: Word> RingBufferedPwm<'d, T, W> {
    pub fn empty(
        tim: impl Peripheral<P = T> + 'd,
        freq: Hertz,
        counting_mode: timer::low_level::CountingMode,
    ) -> Self {
        let ret = Self {
            tim: LLTimer::new(tim),
            ch1: None,
            ch2: None,
            ch3: None,
            ch4: None,
        };

        ret.tim.set_counting_mode(counting_mode);
        ret.tim.set_frequency(freq);
        ret.tim.enable_outputs();
        ret.tim.start();

        ret
    }

    pub fn set_frequency(&mut self, freq: Hertz) {
        self.tim.set_frequency(freq);
    }

    pub fn get_max_duty(&self) -> u32 {
        self.tim.get_max_compare_value() + 1
    }

    /// Queues a new buffer of values to be sent over running PWM channel.
    pub async fn write(
        &mut self,
        channel: Channel,
        dma_buf: &mut [W],
    ) -> Result<(), OverrunError> {
        let ring_buf = match channel {
            Channel::Ch1 => self.ch1.as_mut(),
            Channel::Ch2 => self.ch2.as_mut(),
            Channel::Ch3 => self.ch3.as_mut(),
            Channel::Ch4 => self.ch4.as_mut(),
        };

        if let Some(ring_buf) = ring_buf {
            ring_buf.write_exact(dma_buf).await.map_err(|_| OverrunError)?;
        }

        Ok(())
    }

    /// Stops circular DMA transfer and disables PWM channel.
    pub async fn stop(
        &mut self,
        channel: Channel,
    ) {
        let mut ring_buf = match channel {
            Channel::Ch1 => self.ch1.as_mut(),
            Channel::Ch2 => self.ch2.as_mut(),
            Channel::Ch3 => self.ch3.as_mut(),
            Channel::Ch4 => self.ch4.as_mut(),
        };

        if let Some(ring_buf) = ring_buf.take() {
            ring_buf.stop().await;
        }
        self.tim.enable_channel(channel, false);
    }
}

macro_rules! impl_start_chx {
    ($fn_name:ident, $dma_ch: ident, $cc_ch:ident) => {
        impl<'d, T: GeneralInstance4Channel, W: Word> RingBufferedPwm<'d, T, W> {
            /// Starts a circular double-buffered DMA transfer over PWM channel.
            pub fn $fn_name(
                &mut self,
                _pin: PwmPin<T, $cc_ch>,
                dma: impl Peripheral<P = impl timer::$dma_ch<T>> + 'd,
                dma_buf: &'d mut [W],
            ) {
                embassy_stm32::into_ref!(dma);

                let cc_channel = Channel::$cc_ch;
                let req = dma.request();

                self.tim.set_output_compare_mode(cc_channel, OutputCompareMode::PwmMode1);
                self.tim.set_output_compare_preload(cc_channel, true);
                self.tim.enable_channel(cc_channel, true);

                self.tim.set_cc_dma_selection(Ccds::ONUPDATE);
                self.tim.set_cc_dma_enable_state(cc_channel, true);

                unsafe {
                    use embassy_stm32::dma::{Burst, FifoThreshold, TransferOptions, WritableRingBuffer};

                    let mut dma_transfer_option = TransferOptions::default();
                    dma_transfer_option.fifo_threshold = Some(FifoThreshold::Half);
                    dma_transfer_option.mburst = Burst::Incr4;

                    let mut ring_buf = WritableRingBuffer::new(
                        dma,
                        req,
                        self.tim.regs_gp16().ccr(cc_channel.index()).as_ptr() as *mut _,
                        dma_buf,
                        dma_transfer_option,
                    );
                    ring_buf.start();
                    match cc_channel {
                        Channel::Ch1 => self.ch1 = Some(ring_buf),
                        Channel::Ch2 => self.ch2 = Some(ring_buf),
                        Channel::Ch3 => self.ch3 = Some(ring_buf),
                        Channel::Ch4 => self.ch4 = Some(ring_buf),
                    }
                }
            }
        }
    }
}

impl_start_chx!(start_ch1, Ch1Dma, Ch1);
impl_start_chx!(start_ch2, Ch2Dma, Ch2);
impl_start_chx!(start_ch3, Ch3Dma, Ch3);
impl_start_chx!(start_ch4, Ch4Dma, Ch4);
