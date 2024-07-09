use crate::audio::SAMPLE_RATE;
use crate::audio::VolMgr;
use crate::audio::Error;

use defmt::*;

use micromath::F32Ext;

#[derive(Debug, Copy, Clone)]
pub struct RhythmData {
    step_count: u8,
    tempo: f32,
}

impl RhythmData {
    pub async fn new(vol_mgr: &mut VolMgr<'_>, file: &embedded_sdmmc::RawFile) -> Result<Self, Error> {
        vol_mgr.file_seek_from_start(*file, 0x2c)?;
        const FFT_LEN: usize = 64;
        const HOP_SIZE: usize = 32;

        info!("calculating energies...");
        let mut correlations = [0.0; 63];
        let mut buf = [0; FFT_LEN];
        let mut prev_spectrum = None;
        while vol_mgr.read(*file, &mut buf).await.is_ok_and(|n| n == FFT_LEN) {
            vol_mgr.file_seek_from_current(*file, HOP_SIZE as i32 - FFT_LEN as i32)?;

            let mut samples = [0.0; FFT_LEN];
            samples
                .iter_mut()
                .zip(buf)
                .for_each(|(s, b)| *s = b as f32);

            let complex = microfft::real::rfft_64(&mut samples);
            complex[0].im = 0.0;
            let real = complex.map(|v| v.l1_norm());

            if let Some(prev) = prev_spectrum {
                // Duxbury flux
                let energy: f32 = real
                    .iter()
                    .zip(prev)
                    .map(|(s, p)| {
                        let diff: f32 = *s - p;
                        ((diff + diff.abs()) / 2.0).powi(2)
                    })
                    .sum();

                // wavelet correlation
                let pcm_length = vol_mgr.file_length(*file)? - 0x2c;
                let pcm_offset = vol_mgr.file_offset(*file)? - 0x2c;
                correlations
                    .iter_mut()
                    .enumerate()
                    .for_each(|(mut i, v)| {
                        i += 1;
                        let wavelet_len = pcm_length as f32 / i as f32;
                        let t = (pcm_offset as f32 / wavelet_len).fract();
                        let wavelet = (t - 1.0).powi(2);
                        *v += wavelet * energy
                    });
            }
            prev_spectrum = Some(real);
        }
        vol_mgr.file_seek_from_start(*file, 0x2c)?;
        let step_count = correlations
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(i, _)| (i as u8 + 1) / 2)
            .unwrap();

        let tempo = step_count as f32 /
            (vol_mgr.file_length(*file)? - 0x2c) as f32 *
            SAMPLE_RATE.0 as f32 *
            60.0;
        info!("step_count: {}, tempo: {}", step_count, tempo);

        Ok(RhythmData { step_count, tempo })
    }

    pub fn step_count(&self) -> u8 {
        self.step_count
    }

    pub fn tempo(&self) -> f32 {
        self.tempo
    }
}

// const ENERGY_FFT_LEN: usize = 256;
// const RHYTHM_FFT_LEN: usize = 4096;
// const HOP_SIZE: usize = 2;

// let mut energies = [0.0; RHYTHM_FFT_LEN];
// let mut buf = [0; ENERGY_FFT_LEN];
// info!("file_length: {}, file_offset: {}, product: {}",
//     vol_mgr.file_length(*file)?,
//     vol_mgr.file_offset(*file)?,
//     RHYTHM_FFT_LEN as u32 * HOP_SIZE as u32
// );
// while vol_mgr.file_length(*file)? - vol_mgr.file_offset(*file)? >= RHYTHM_FFT_LEN as u32 * HOP_SIZE as u32 {
//     'e: for energy in energies.iter_mut() {
//         if !vol_mgr.read(*file, &mut buf).await.is_ok_and(|n| n == ENERGY_FFT_LEN) {
//             break 'e;
//         }
//         let mut samples = [0.0; ENERGY_FFT_LEN];
//         samples
//             .iter_mut()
//             .zip(buf)
//             .for_each(|(s, b)| *s = b as f32);
//         let spectrum = microfft::real::rfft_256(&mut samples);
//         spectrum[0].im = 0.0;

//         // Masri HFC
//         *energy = spectrum
//             .iter()
//             .enumerate()
//             .map(|(i, v)| i as f32 / ENERGY_FFT_LEN as f32 * v.norm_sqr().sqrt())
//             .sum::<f32>();

//         vol_mgr.file_seek_from_current(*file, HOP_SIZE as i32 - ENERGY_FFT_LEN as i32)?;
//     }
//     let spectrum = microfft::real::rfft_4096(&mut energies);
//     let bin_mode = spectrum
//         .iter()
//         .map(|v| v.norm_sqr().sqrt())
//         .enumerate()
//         .max_by(|(_, e0), (_, e1)| e0.total_cmp(e1))
//         .map(|(i, _)| i)
//         .unwrap();
//     info!("bin_mode: {}", bin_mode);

//     // ioi mode in samples
//     let ioi_mode = crate::audio::SAMPLE_RATE.to_Hz() as f32 * bin_mode as f32 / spectrum.len() as f32;

//     let bpm = crate::audio::SAMPLE_RATE.to_Hz() as f32 / ioi_mode as f32 * 60.0;
//     info!("bpm: {}", bpm);
// }

// const FIR_WINDOW_LEN: usize = 256;
// const MED_WINDOW_LEN: usize = 256;
// let mut energies: Vec<f32> = Vec::new();
// let mut buf = [0; FFT_WINDOW_LEN];
// while vol_mgr.read(*file, &mut buf).await.is_ok_and(|n| n == FFT_WINDOW_LEN) && energies.len() < FFT_WINDOW_COUNT {
//     let left = vol_mgr.file_length(*file)? - vol_mgr.file_offset(*file)?;
//     let hop_size = left / (FFT_WINDOW_COUNT - energies.len()) as u32;
//     let mut samples = [0.0; FFT_WINDOW_LEN];
//     samples
//         .iter_mut()
//         .zip(buf)
//         .for_each(|(s, b)| *s = b as f32);

//     let spectrum = microfft::real::rfft_256(&mut samples);
//     let length = spectrum.len();
//     spectrum[0].im = 0.0;

//     // Masri HFC
//     let energy = spectrum
//         .iter()
//         .enumerate()
//         .map(|(i, v)| i as f32 / length as f32 * v.norm_sqr().sqrt())
//         .sum::<f32>();
//     energies.push(energy);

//     vol_mgr.file_seek_from_current(*file, hop_size as i32 - FFT_WINDOW_LEN as i32)?;
// }
// vol_mgr.file_seek_from_start(*file, 0x2c)?;

// // onset frequencies
// let mut buf: [f32; FFT_WINDOW_COUNT] = crate::expect!(
//     energies[..FFT_WINDOW_COUNT].try_into(),
//     "failed to convert vec into array"
// );
// let spectrum = microfft::real::rfft_2048(&mut buf);

// let bin_mode = spectrum
//     .iter()
//     .map(|x| x.norm_sqr().sqrt())
//     .enumerate()
//     .max_by(|(_, e0), (_, e1)| e0.total_cmp(e1))
//     .map(|(i, _)| i)
//     .unwrap();
// info!("bin mode: {}", bin_mode);

// // ioi mode in samples
// let ioi_mode = crate::audio::SAMPLE_RATE.to_Hz() as f32 * bin_mode as f32 / spectrum.len() as f32;

// let bpm = crate::audio::SAMPLE_RATE.to_Hz() as f32 / ioi_mode as f32 * 60.0;
// info!("bpm: {}", bpm);

// info!("onset start!");
// let onsets = {
//     // weighted moving average
//     let mut wmas = (0..energies.len())
//         .map(|n| {
//             let start = n.saturating_sub(FIR_WINDOW_LEN / 2);
//             let end = (n + FIR_WINDOW_LEN / 2).min(energies.len());

//             energies[start..end]
//                 .iter()
//                 .enumerate()
//                 .fold(0.0, |acc, (i, v)| {
//                     let mid = (end - start) / 2;
//                     let diff = i.abs_diff(mid);
//                     let m = 1.0 - diff as f32 / mid as f32;
//                     acc + v * m
//                 })
//         })
//         .collect::<Vec<_>>();

//     // normalize to [-1, 1] about mean at 0
//     let mean = wmas
//         .iter()
//         .sum::<f32>() / wmas.len() as f32;
//     let max_abs_dev = wmas
//         .iter()
//         .map(|v| (v - mean).abs())
//         .fold(f32::NEG_INFINITY, |a, b| a.max(b));
//     let _ = wmas
//         .iter_mut()
//         .map(|&mut v| (v - mean) / max_abs_dev);

//     // moving median
//     let thresholds = (0..wmas.len())
//         .map(|i| {
//             let start = i.saturating_sub(MED_WINDOW_LEN / 2);
//             let end = (i + MED_WINDOW_LEN / 2).min(wmas.len());

//             let mut window = wmas[start..end].to_vec();
//             window.sort_by(|a, b| a.partial_cmp(b).unwrap());

//             window[window.len() / 2]
//         })
//         .collect::<Vec<_>>();

//     // onset indices
//     let hop_size = (vol_mgr.file_length(*file)? - 0x2c) as f32 / FFT_WINDOW_COUNT as f32;

//     let mut diffs = wmas
//         .iter()
//         .zip(thresholds.clone())
//         .map(|(w, t)| w - t)
//         .enumerate();
//     let mut buf = Vec::new();
//     while let Some((i0, _)) = diffs.by_ref().find(|&(_, v)| v > 0.0) {
//         let i = diffs
//             .by_ref()
//             .take_while(|&(_, v)| v > 0.0)
//             .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
//             .map(|(i1, _)| i1)
//             .unwrap_or(i0) as f32 * hop_size;
//         buf.push(i as usize);
//     }
//     buf
// };
// info!("onsets: {}", Debug2Format(&onsets));
// let mut iois = onsets
//     .iter()
//     .tuple_windows::<(_, _)>()
//     .map(|(a, b)| b - a)
//     .collect::<Vec<_>>();
// iois.sort();

// // tirmean ioi in samples
// let ioi_mean = (iois.get(iois.len() / 4).unwrap() +
//     iois.get(iois.len() / 2).unwrap() * 2 +
//     iois.get(3 * iois.len() / 4).unwrap()) / 4;
// info!("ioi_med: {}", ioi_mean);

// let step_count = vol_mgr.file_length(*file).unwrap() as f32 / ioi_mean as f32;
// info!("step count: {}", step_count);

// let bpm = crate::audio::SAMPLE_RATE.to_Hz() as f32 / ioi_mean as f32 * 60.0;
// info!("bpm: {}", bpm);

// Ok(Self { inner: Vec::new(), ioi: ioi_mode as f32 })
