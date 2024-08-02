use alloc::string::String;
use embassy_stm32::time::Hertz;

pub const SAMPLE_RATE: Hertz = Hertz::khz(32);

/// audio file metadata
#[derive(Debug, Clone)]
struct WavMetadata {
    path: String,
    rhythm: rhythm::RhythmData,
}

pub mod fx;
pub mod steps;
mod rhythm;
