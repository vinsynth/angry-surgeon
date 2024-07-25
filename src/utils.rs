use defmt::Format;

#[macro_export]
macro_rules! expect {
    ( $result:expr, $message:expr ) => {
        match $result {
            Ok(v) => v,
            Err(e) => defmt::panic!("{}: {}", $message, defmt::Debug2Format(&e))
        }
    }
}

/// for `AsyncMutex<Option<..>>`
#[macro_export]
macro_rules! lock_async_mut {
    ( $mutex:expr ) => {
        $mutex
            .lock()
            .await
            .as_mut()
            .unwrap()
    }
}

/// for `AsyncMutex<Option<..>>`
#[macro_export]
macro_rules! lock_async_ref {
    ( $mutex:expr ) => {
        $mutex
            .lock()
            .await
            .as_ref()
            .unwrap()
    }
}

pub type AsyncMutex<T> = embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, T>;

/// fraction backed by `u32` numerator and denominator
#[derive(Clone, Copy, Debug, PartialEq, Eq, Format)]
pub struct Fraction {
    numerator: u32,
    denominator: u32,
}

impl Fraction {
    pub fn new(numerator: u32, denominator: u32) -> Self {
        Self {
            numerator,
            denominator,
        }
    }
}

impl From<Fraction> for f32 {
    fn from(frac: Fraction) -> Self {
        frac.numerator as f32 / frac.denominator as f32
    }
}

impl From<u32> for Fraction {
    fn from(uint: u32) -> Self {
        Self {
            numerator: uint,
            denominator: 1,
        }
    }
}

impl From<Fraction> for u32 {
    fn from(frac: Fraction) -> Self {
        frac.numerator / frac.denominator
    }
}

impl core::ops::Add<u32> for Fraction {
    type Output = Fraction;

    fn add(self, other: u32) -> Fraction {
        Fraction {
            numerator: self.numerator + other * self.denominator,
            denominator: self.denominator,
        }
    }
}

impl core::ops::Mul<Fraction> for Fraction {
    type Output = Fraction;

    fn mul(self, other: Fraction) -> Fraction {
        Fraction {
            numerator: self.numerator * other.numerator,
            denominator: self.denominator * other.denominator,
        }
    }
}

impl core::ops::Mul<u32> for Fraction {
    type Output = Fraction;

    fn mul(self, other: u32) -> Fraction {
        Fraction {
            numerator: self.numerator * other,
            denominator: self.denominator,
        }
    }
}

#[allow(clippy::suspicious_arithmetic_impl)]
impl core::ops::Div<u32> for Fraction {
    type Output = Fraction;

    fn div(self, other: u32) -> Fraction {
        Fraction {
            numerator: self.numerator,
            denominator: self.denominator * other,
        }
    }
}
