use alloc::vec::Vec;

use biquad::Biquad;
use micromath::F32Ext;

use embedded_fatfs::{Error, File, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, Seek, SeekFrom};

const FFT_LEN: usize = 256;
const HOP_LEN: usize = 128;
const SPECTRUM_LEN: usize = 8; // log2(FFT_LEN / 2)

struct Onset {
    offset: u64,
    spectrum: [f32; SPECTRUM_LEN],
}

#[derive(Debug)]
pub struct KickOnset(u64);

#[derive(Debug)]
pub struct SnareOnset(u64);

pub struct RhythmData {
    pub kicks: Vec<KickOnset>,
    pub snares: Vec<SnareOnset>,
}

impl RhythmData {
    pub async fn new<IO, TP, OCC>(
        file: &mut File<'_, IO, TP, OCC>,
    ) -> Result<Self, Error<IO::Error>>
    where
        IO: ReadWriteSeek,
        TP: TimeProvider,
        OCC: OemCpConverter,
    {
        let mut rhythm = Self {
            kicks: Vec::new(),
            snares: Vec::new(),
        };
        
        let hann = {
            let mut hann = [0.; FFT_LEN];
            for (i, hann) in hann.iter_mut().enumerate() {
                *hann = (core::f32::consts::PI * i as f32 / (FFT_LEN - 1) as f32).sin().powi(2);
            }
            hann
        };
        let mut filter = {
            use biquad::{Coefficients, DirectForm1, Hertz, Type};

            let coeff = Coefficients::<f32>::from_params(
                Type::BandPass,
                Hertz::<f32>::from_hz(
                    crate::SAMPLE_RATE as f32 / FFT_LEN as f32
                ).unwrap(),
                Hertz::<f32>::from_hz(2.).unwrap(),
                1.,
            ).unwrap();
            DirectForm1::<f32>::new(coeff)
        };
        let mut last_onset: Option<Onset> = None;
        let mut last_spectrum = [0.; SPECTRUM_LEN];

        file.seek(SeekFrom::Start(44)).await?; // seek to PCM start

        let mut byte_buffer = [0u8; FFT_LEN * 2];
        while file.read_exact(&mut byte_buffer).await.is_ok() {
        // loop {
            // match file.read_exact(&mut byte_buffer).await {
            //     Err(embedded_io_async::ReadExactError::UnexpectedEof) => break,
            //     Err(embedded_io_async::ReadExactError::Other(e)) => {
            //         defmt::info!("{}", defmt::Debug2Format(&e));
            //         continue;
            //     },
            //     _ => (),
            // }

            file.seek(
                SeekFrom::Current(HOP_LEN as i64 * 2 - FFT_LEN as i64 * 2)
            ).await?;

            // parse i16 byte stream
            let mut samples = [0.; FFT_LEN];
            for (i, sample) in samples.iter_mut().enumerate() {
                let mut i16_buffer = [0; 2];
                i16_buffer.copy_from_slice(&byte_buffer[i * 2..][0..2]);
                *sample = i16::from_le_bytes(i16_buffer) as f32 * hann[i];
            }
            let complex = microfft::real::rfft_256(&mut samples);

            // collect into logarithmic bins of Duxbury spectral flux
            let mut sum = 0.;
            for i in 0..SPECTRUM_LEN {
                let energy = complex[2usize.pow(i as u32 - 1)..2usize.pow(i as u32)]
                    .iter()
                    .map(|v| v.norm_sqr().sqrt())
                    .sum::<f32>();
                let diff = energy - last_spectrum[i];
                let rect = diff.clamp(0.0, f32::MAX);
                sum += rect;

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
                        offset: file.stream_position().await?,
                        spectrum: last_spectrum,
                    });
                }
            } else if let Some(onset) = last_onset.take() {
                let kicky = onset
                    .spectrum
                    .iter()
                    .sum::<f32>();
                let snarey = onset
                    .spectrum
                    .iter()
                    .enumerate()
                    .fold(0., |acc, (i, e)| {
                        acc + e * i as f32 / (SPECTRUM_LEN - 1) as f32 * 2.
                    });
                if kicky > snarey {
                    rhythm.kicks.push(KickOnset(onset.offset));
                } else {
                    rhythm.snares.push(SnareOnset(onset.offset));
                }
            }
        }

        Ok(rhythm)
    }
}
