use embassy_stm32::Peripheral;
use embassy_stm32::time::Hertz;

use embassy_stm32::timer;
use timer::GeneralInstance4Channel;
use timer::low_level::{OutputCompareMode, Timer as LLTimer};
use timer::simple_pwm::{Ch1, Ch2, Ch3, Ch4, PwmPin};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error{
    ChannelNotInUse,
}

pub struct InterruptPwm<'d, T: GeneralInstance4Channel> {
    tim: LLTimer<'d, T>,
    ch1: Option<PwmPin<'d, T, Ch1>>,
    ch2: Option<PwmPin<'d, T, Ch2>>,
    ch3: Option<PwmPin<'d, T, Ch3>>,
    ch4: Option<PwmPin<'d, T, Ch4>>,
}

impl<'d, T: GeneralInstance4Channel> InterruptPwm<'d, T> {
    pub fn new(
        tim: impl Peripheral<P = T> + 'd,
        ch1: Option<PwmPin<'d, T, Ch1>>,
        ch2: Option<PwmPin<'d, T, Ch2>>,
        ch3: Option<PwmPin<'d, T, Ch3>>,
        ch4: Option<PwmPin<'d, T, Ch4>>,
        freq: Hertz,
    ) -> Self {
        let ret = Self {
            tim: LLTimer::new(tim),
            ch1,
            ch2,
            ch3,
            ch4,
        };

        ret.tim.set_frequency(freq);
        ret.tim.start();

        let r = ret.tim.regs_gp16();
        if ret.ch1.is_some() {
            r.ccmr_output(0)
                .modify(|w| w.set_ocm(0, OutputCompareMode::PwmMode1.into()));
        }
        if ret.ch2.is_some() {
            r.ccmr_output(0)
                .modify(|w| w.set_ocm(1, OutputCompareMode::PwmMode1.into()));
        }
        if ret.ch3.is_some() {
            r.ccmr_output(1)
                .modify(|w| w.set_ocm(0, OutputCompareMode::PwmMode1.into()));
        }
        if ret.ch4.is_some() {
            r.ccmr_output(1)
                .modify(|w| w.set_ocm(1, OutputCompareMode::PwmMode1.into()));
        }

        ret
    }

    pub fn enable(&mut self, channel: timer::Channel) -> Result<(), Error> {
        self.tim.regs_gp16().ccer().modify(|w| w.set_cce(channel.index(), true));
        self.tim.enable_update_interrupt(true);

        Ok(())
    }

    pub fn disable(&mut self, channel: timer::Channel) -> Result<(), Error> {
        if match channel {
            timer::Channel::Ch1 => self.ch1.is_none(),
            timer::Channel::Ch2 => self.ch2.is_none(),
            timer::Channel::Ch3 => self.ch3.is_none(),
            timer::Channel::Ch4 => self.ch4.is_none(),
        } {
            return Err(Error::ChannelNotInUse);
        }

        self.tim.regs_gp16().ccer().modify(|w| w.set_cce(channel.index(), false));
        self.tim.enable_update_interrupt(false);

        Ok(())
    }

    pub fn clear_update_interrupt(&mut self) -> bool {
        self.tim.clear_update_interrupt()
    }

    pub fn set_frequency(&mut self, freq: Hertz) {
        self.tim.set_frequency(freq);
    }

    pub fn get_max_duty(&self) -> u32 {
        self.tim.regs_gp16().arr().read().0
    }

    pub fn set_duty(&mut self, channel: timer::Channel, duty: u32) -> Result<(), Error> {
        if match channel {
            timer::Channel::Ch1 => self.ch1.is_none(),
            timer::Channel::Ch2 => self.ch2.is_none(),
            timer::Channel::Ch3 => self.ch3.is_none(),
            timer::Channel::Ch4 => self.ch4.is_none(),
        } {
            return Err(Error::ChannelNotInUse);
        }

        defmt::assert!(duty < self.get_max_duty());
        self.tim.regs_gp16().ccr(channel.index()).write_value(embassy_stm32::pac::timer::regs::Ccr1ch(duty));

        Ok(())
    }
}
