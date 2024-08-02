use micromath::F32Ext;

/// bitcrush effect
///
/// parameters as `f32` proportions of unaffected features
pub struct Bitcrush {
    bit_depth: f32,
    rate: f32,
}

impl Bitcrush {
    pub const fn new() -> Self {
        Self { bit_depth: 1.0, rate: 1.0 }
    }

    pub fn set_bit_depth(&mut self, bit_prop: f32) {
        self.bit_depth = bit_prop;
    }

    pub fn set_rate(&mut self, rate_prop: f32) {
        self.rate = rate_prop;
    }

    pub fn crush(&self, samples: &mut [u8]) {
        // bit reduction
        samples
            .iter_mut()
            .for_each(|v| {
                let scale = 2.0.powf(6.0 * (self.bit_depth - 1.0));
                *v = ((*v as f32 * scale).floor() / scale) as u8;
            });
        // downsampling
        let (mut last, mut curr) = (0.0, samples.get(1).copied().unwrap_or(u8::MAX / 2));
        samples
            .iter_mut()
            .enumerate()
            .for_each(|(i, v)| {
                let scaled = (i as f32 * self.rate).floor() / self.rate;
                if scaled != last {
                    last = scaled;
                    curr = *v;
                }
                *v = curr;
            });
    }
}
