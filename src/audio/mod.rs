use alloc::string::String;

pub const SAMPLE_RATE: u32 = 44100;
pub const GRAIN_LEN: usize = 256;

struct WavMetadata {
    path: String,
    rhythm: rhythm::RhythmData,
}

pub mod steps;
pub mod rhythm;
