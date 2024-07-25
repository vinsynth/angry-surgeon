use crate::audio::SAMPLE_RATE;
use crate::audio::Error;
use crate::fs::VolMgr;

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
            .map(|(i, _)| i as u8 + 1)
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
