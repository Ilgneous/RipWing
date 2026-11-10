//! Noise measurement: the input to choosing a window length.
//!
//! The lag/noise table in [`crate::average`] only answers half the question.
//! It says what each window *costs*; it cannot say what window you *need*,
//! because that depends on how noisy the signal actually is — and on a
//! multirotor that is dominated by motor and prop vibration, which only shows
//! up on the real airframe with the motors spinning. So the window choice is
//! a measurement, not a guess, and this is the instrument.
//!
//! Feed it raw samples and filtered samples side by side and compare the two
//! standard deviations. That is the filter's *achieved* noise factor on the
//! real signal, to hold against the table's white-noise prediction.

/// Running mean and standard deviation over three independent axes.
///
/// Uses Welford's algorithm rather than accumulating sum and sum-of-squares.
/// The naive form computes variance as a difference of two large, nearly
/// equal numbers, which in `f32` cancels catastrophically when the mean is
/// large relative to the spread — exactly the accelerometer's situation
/// (mean ~9.8 m/s², spread ~0.01). Welford stays accurate there.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoiseMeter3 {
    n: u32,
    mean: [f32; 3],
    m2: [f32; 3],
}

impl NoiseMeter3 {
    pub const fn new() -> Self {
        Self {
            n: 0,
            mean: [0.0; 3],
            m2: [0.0; 3],
        }
    }

    /// Accumulate one sample.
    #[inline]
    pub fn update(&mut self, x: [f32; 3]) {
        self.n = self.n.saturating_add(1);
        let n = self.n as f32;
        for i in 0..3 {
            let delta = x[i] - self.mean[i];
            self.mean[i] += delta / n;
            self.m2[i] += delta * (x[i] - self.mean[i]);
        }
    }

    /// Samples accumulated since the last reset.
    pub fn count(&self) -> u32 {
        self.n
    }

    /// Per-axis mean.
    pub fn mean(&self) -> [f32; 3] {
        self.mean
    }

    /// Per-axis population variance. Zero until at least two samples.
    pub fn variance(&self) -> [f32; 3] {
        if self.n < 2 {
            return [0.0; 3];
        }
        let n = self.n as f32;
        [self.m2[0] / n, self.m2[1] / n, self.m2[2] / n]
    }

    /// Per-axis standard deviation.
    pub fn std_dev(&self) -> [f32; 3] {
        let v = self.variance();
        [sqrt(v[0]), sqrt(v[1]), sqrt(v[2])]
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// `sqrt` without `std` or a math crate: a few Newton iterations from a
/// bit-level initial guess. Runs once per report (not per sample), so
/// speed is irrelevant; it only needs to be accurate to well under the
/// precision anyone will read off a noise figure.
fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    // Halving the exponent gives an initial guess within a factor of ~1.4.
    let mut y = f32::from_bits((x.to_bits() >> 1) + 0x1fc0_0000);
    for _ in 0..4 {
        y = 0.5 * (y + x / y);
    }
    y
}
