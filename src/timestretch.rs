use crate::audio::VolMgr;
use crate::audio::Error;

extern crate alloc;
use alloc::vec::Vec;

use defmt::*;

use micromath::F32Ext;

pub struct Onsets {
    inner: Vec<usize>,
    ioi: f32,
}

impl Onsets {
    pub async fn new(vol_mgr: &mut VolMgr<'_>, file: &embedded_sdmmc::RawFile) -> Result<Self, Error> {
        info!("start onset detection...");
        const FFT_WINDOW_LEN: usize = 2048;
        const FFT_WINDOW_COUNT: usize = 2048;
        const FIR_WINDOW_LEN: usize = 256;
        const MED_WINDOW_LEN: usize = 256;
        let hop_size = vol_mgr.file_length(*file)? as f32 / FFT_WINDOW_COUNT as f32;
        info!("hop_size: {}", hop_size);

        let mut energies: Vec<f32> = Vec::new();
        let mut buf = [0; FFT_WINDOW_LEN];
        while vol_mgr.read(*file, &mut buf).await.is_ok_and(|n| n == FFT_WINDOW_LEN) {
            let hop_size = (vol_mgr.file_offset(*file)? as f32 * hop_size) as usize / vol_mgr.file_offset(*file)? as usize;
            info!("hop size: {}", hop_size);
            let mut samples = [0.0; FFT_WINDOW_LEN];
            samples
                .iter_mut()
                .zip(buf)
                .for_each(|(s, b)| *s = b as f32);

            let spectrum = microfft::real::rfft_2048(&mut samples);
            let length = spectrum.len();
            spectrum[0].im = 0.0;

            // Masri HFC
            let energy = spectrum
                .iter()
                .enumerate()
                .map(|(i, v)| i as f32 / length as f32 * v.norm_sqr().sqrt())
                .sum::<f32>();
            energies.push(energy);

            vol_mgr.file_seek_from_current(*file, hop_size as i32 - FFT_WINDOW_LEN as i32)?;
        }
        vol_mgr.file_seek_from_start(*file, 0x2c)?;

        info!("onset start!");
        // weighted moving average
        let mut wmas = (0..energies.len())
            .map(|n| {
                let start = n.saturating_sub(FIR_WINDOW_LEN / 2);
                let end = (n + FIR_WINDOW_LEN / 2).min(energies.len());

                energies[start..end]
                    .iter()
                    .enumerate()
                    .fold(0.0, |acc, (i, v)| {
                        let mid = (end - start) / 2;
                        let diff = i.abs_diff(mid);
                        let m = 1.0 - diff as f32 / mid as f32;
                        acc + v * m
                    })
            })
            .collect::<Vec<_>>();

        // normalize to [-1, 1] about mean at 0
        let mean = wmas
            .iter()
            .sum::<f32>() / wmas.len() as f32;
        let max_abs_dev = wmas
            .iter()
            .map(|v| (v - mean).abs())
            .fold(f32::NEG_INFINITY, |a, b| a.max(b));
        let _ = wmas
            .iter_mut()
            .map(|&mut v| (v - mean) / max_abs_dev);

        // moving median
        let thresholds = (0..wmas.len())
            .map(|i| {
                let start = i.saturating_sub(MED_WINDOW_LEN / 2);
                let end = (i + MED_WINDOW_LEN / 2).min(wmas.len());

                let mut window = wmas[start..end].to_vec();
                window.sort_by(|a, b| a.partial_cmp(b).unwrap());

                window[window.len() / 2]
            })
            .collect::<Vec<_>>();

        // onset indices
        let onsets = {
            let mut diffs = wmas
                .iter()
                .zip(thresholds.clone())
                .map(|(w, t)| w - t)
                .enumerate();
            let mut buf = Vec::new();
            while let Some((i0, _)) = diffs.by_ref().find(|&(_, v)| v > 0.0) {
                let i = diffs
                    .by_ref()
                    .take_while(|&(_, v)| v > 0.0)
                    .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                    .map(|(i1, _)| i1)
                    .unwrap_or(i0) * hop_size as usize;
                buf.push(i);
            }
            buf
        };

        // onset frequencies
        info!("energies.len(): {}", energies.len());
        let mut buf: [f32; FFT_WINDOW_COUNT] = crate::expect!(
            energies[..FFT_WINDOW_COUNT].try_into(),
            "failed to convert vec into array"
        );
        info!("almost...");
        let spectrum = microfft::real::rfft_2048(&mut buf);
        spectrum[0].im = 0.0;

        let bin_mode = spectrum
            .iter()
            .map(|x| x.norm_sqr().sqrt())
            .enumerate()
            .max_by(|(_, e0), (_, e1)| e0.total_cmp(e1))
            .map(|(i, _)| i)
            .unwrap();
        let ioi_mode = crate::audio::SAMPLE_RATE.to_Hz() as f32 / energies.len() as f32 * bin_mode as f32;

        info!("tempo: {}", ioi_mode * 60.0);

        Ok(Self { inner: onsets, ioi: ioi_mode })
    }
}
