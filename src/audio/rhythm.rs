use alloc::vec::Vec;

use biquad::Biquad;
use micromath::F32Ext;

use embedded_fatfs::{Error, File, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, Seek, SeekFrom};

const DOWNSAMPLE: usize = 2;
const FFT_LEN: usize = 256 / DOWNSAMPLE;
const READ_LEN: usize = 256 * 2;
const SEEK_LEN: usize = READ_LEN / 2;
const SPECTRUM_LEN: usize = FFT_LEN.ilog2() as usize;

struct Onset {
    position: u64,
    spectrum: [f32; SPECTRUM_LEN],
}

#[derive(Debug)]
pub enum OnsetType {
    Kick,
    Snare,
}

/// onset with absolute position
#[derive(Debug)]
pub struct Pulse {
    pub onset_type: OnsetType,
    pub position: u64,
}

/// onset of relative length
#[derive(Debug)]
pub struct Stroke {
    onset_type: OnsetType,
    length: usize,
}

pub struct ClaveBuilder {
    onsets: Vec<Pulse>,
}

impl ClaveBuilder {
    pub fn new() -> Self {
        Self { onsets: Vec::new() }
    }

    pub fn push(&mut self, pulse: Pulse) {
        self.onsets.push(pulse);
    }

    pub fn bake(self) -> Vec<Stroke> {
        let intervals = self.onsets
            .windows(2)
            .flat_map(<&[Pulse; 2]>::try_from)
            .map(|[a, b]| b.position - a.position)
            .collect::<Vec<_>>();
        let mut fundamental_appeal = (0, 0.);
        for period in 50..intervals.iter().max().copied().unwrap_or(0) { // start with millis period of 20Hz
            let (mut mean_sin, mut mean_cos) = intervals.iter().fold((0., 0.), |acc, i| {
                let (sin, cos) = (2. * core::f32::consts::PI * *i as f32 / period as f32).sin_cos();
                (acc.0 + sin, acc.1 + cos)
            });
            mean_sin /= intervals.len() as f32;
            mean_cos /= intervals.len() as f32;

            let appeal = 1. - (mean_sin.powi(2) + (mean_cos - 1.).powi(2)).sqrt() / 2.;

            if appeal > fundamental_appeal.1 {
                fundamental_appeal = (period, appeal);
            }
        }

        let mut strokes = Vec::new();
        for (i, onset) in self.onsets.into_iter().take(intervals.len()).enumerate() {
            let onset_type = onset.onset_type;
            let length = (intervals[i] as f32 / fundamental_appeal.0 as f32).round() as usize;
            strokes.push(Stroke { onset_type, length });
        }
        strokes
    }
}

pub struct RhythmData {
    pub pulses: Vec<Pulse>,
    pub step_length: f32,
    pub tempo: f32,
}

impl RhythmData {
    pub async fn new<IO, TP, OCC>(
        file: &mut File<'_, IO, TP, OCC>,
        file_length: u64,
    ) -> Result<Self, Error<IO::Error>>
    where
        IO: ReadWriteSeek,
        TP: TimeProvider,
        OCC: OemCpConverter,
    {
        let mut rhythm = Self {
            pulses: Vec::new(),
            step_length: 0.,
            tempo: 0.,
        };

        let hann = {
            let mut hann = [0.; FFT_LEN];
            for (i, hann) in hann.iter_mut().enumerate() {
                *hann = (core::f32::consts::PI * i as f32 / FFT_LEN as f32).sin().powi(2);
            }
            hann
        };
        let mut filter = {
            use biquad::{Coefficients, DirectForm1, Hertz, Type};

            let coeff = Coefficients::<f32>::from_params(
                Type::BandPass,
                Hertz::<f32>::from_hz(
                    super::SAMPLE_RATE as f32 / DOWNSAMPLE as f32 / FFT_LEN as f32
                ).unwrap(),
                Hertz::<f32>::from_hz(3.).unwrap(),
                1.,
            ).unwrap();
            DirectForm1::<f32>::new(coeff)
        };
        let mut onsets: Vec<Onset> = Vec::new();
        let mut last_onset: Option<Onset> = None;
        let mut last_spectrum = [0.; SPECTRUM_LEN];

        file.seek(SeekFrom::Start(44)).await?; // seek to PCM start

        let mut byte_buffer = [0u8; READ_LEN];
        while file.read_exact(&mut byte_buffer).await.is_ok() {
            file.seek(
                SeekFrom::Current(SEEK_LEN as i64 - READ_LEN as i64)
            ).await?;

            // parse i16 byte stream
            let mut samples = [0.; FFT_LEN];
            for (i, sample) in samples.iter_mut().enumerate() {
                let mut i16_buffer = [0; 2];
                i16_buffer.copy_from_slice(&byte_buffer[i * 2 * DOWNSAMPLE..][0..2]);
                *sample = i16::from_le_bytes(i16_buffer) as f32 * hann[i];
            }
            let complex = microfft::real::rfft_128(&mut samples);

            // collect into logarithmic bins of HFC
            let mut sum = 0.;
            for i in 0..SPECTRUM_LEN {
                let energy = complex[2usize.pow(i as u32 - 1)..2usize.pow(i as u32)]
                    .iter()
                    .map(|v| v.norm_sqr().sqrt())
                    .sum::<f32>();
                sum += energy * i as f32 / SPECTRUM_LEN as f32;
                last_spectrum[i] = energy;
            }
            // filter out low-freq swells and high-freq noise to get transients
            let detect = filter.run(sum).clamp(0., f32::MAX);

            if detect > 0. {
                if let Some(onset) = last_onset.as_mut() {
                    for (i, e) in onset.spectrum.iter_mut().enumerate() {
                        *e += last_spectrum[i];
                    }
                } else {
                    last_onset = Some(Onset {
                        position: file.stream_position().await? / SEEK_LEN as u64,
                        spectrum: last_spectrum,
                    });
                }
            } else if let Some(onset) = last_onset.take() {
                onsets.push(onset);
            }
        }

        // retain strongest (most beat-significant) onsets
        let mean_strength = onsets.iter().fold(0., |acc, o| {
            acc + o.spectrum.iter().sum::<f32>()
        }) / onsets.len() as f32;
        onsets.retain(|o| o.spectrum.iter().sum::<f32>() > mean_strength);
        // snares with more energy in high bins, kicks in low bins
        let skews = onsets.iter().map(|o| {
            let sum = o.spectrum.iter().sum::<f32>();
            o.spectrum.iter().enumerate().fold(0., |acc, (i, e)| {
                acc + (i + 1) as f32 * e
            }) / sum - 1.
        }).collect::<Vec<_>>();
        let mid = skews.iter().sum::<f32>() / skews.len() as f32;
        for (i, &skew) in skews.iter().enumerate() {
            if skew > mid {
                rhythm.pulses.push(Pulse {
                    onset_type: OnsetType::Snare,
                    position: onsets[i].position * SEEK_LEN as u64,
                });
            } else {
                rhythm.pulses.push(Pulse {
                    onset_type: OnsetType::Kick,
                    position: onsets[i].position * SEEK_LEN as u64,
                });
            }
        }

        // calculate approximate greatest common divisor for onset intervals
        let intervals = onsets
            .windows(2)
            .flat_map(<&[Onset; 2]>::try_from)
            .map(|[a, b]| b.position - a.position)
            .collect::<Vec<_>>();
        let mut fundamental_appeal = (0, 0.);
        for period in 9..*intervals.iter().max().unwrap() { // start at hop period of 20Hz
            let (mut mean_sin, mut mean_cos) = intervals.iter().fold((0., 0.), |acc, i| {
                let (sin, cos) = (2. * core::f32::consts::PI * *i as f32 / period as f32).sin_cos();
                (acc.0 + sin, acc.1 + cos)
            });
            mean_sin /= intervals.len() as f32;
            mean_cos /= intervals.len() as f32;

            let appeal = 1. - (mean_sin.powi(2) + (mean_cos - 1.).powi(2)).sqrt() / 2.;

            if appeal > fundamental_appeal.1 {
                fundamental_appeal = (period, appeal);
            }
        }
        let quantum = ((file_length - 44) as f32 / fundamental_appeal.0 as f32).round() as u64;

        rhythm.step_length = (file_length - 44) as f32 / quantum as f32;
        rhythm.tempo = super::SAMPLE_RATE as f32
            / rhythm.step_length
            * 60.
            / (SEEK_LEN / 2) as f32;

        Ok(rhythm)
    }
}
