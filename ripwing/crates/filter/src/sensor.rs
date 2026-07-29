//! The composed sensor filter: median-of-3 followed by an `N`-tap average.
//!
//! Order matters. The median runs *first* so that outliers are removed before
//! they can enter the linear stage. Reversed, the average would smear each
//! spike across `N` samples and the median would then be looking at `N`
//! mildly-wrong values instead of one obviously-wrong one, with nothing to
//! reject.
//!
//! Total group delay is roughly `1 + (N-1)/2` samples: about one from the
//! median's step response plus the average's exact group delay.
//!
//! # Filter for control, log the raw for detection
//!
//! These filters take a sample and *return* the filtered value; they never
//! consume or overwrite the caller's raw data. That is deliberate. The
//! control path wants the clean signal, but the anomaly detector wants the
//! spike that was rejected — a burst of outliers is itself a symptom
//! (failing sensor, connector vibrating loose, EMI from a failing ESC).
//! Filtering ahead of the detector would erase the evidence, so the driver
//! should log `raw` and hand `filtered` downstream.

use crate::average::MovingAverage;
use crate::median::MedianOf3;
use ripwing_common::ImuSample;

/// One axis: median-of-3 into an `N`-tap moving average.
#[derive(Clone, Copy, Debug)]
pub struct ScalarFilter<const N: usize> {
    median: MedianOf3,
    average: MovingAverage<N>,
}

impl<const N: usize> ScalarFilter<N> {
    pub const fn new() -> Self {
        Self {
            median: MedianOf3::new(),
            average: MovingAverage::new(),
        }
    }

    /// Feed one raw sample, get the conditioned value.
    #[inline]
    pub fn update(&mut self, raw: f32) -> f32 {
        let despiked = self.median.update(raw);
        self.average.update(despiked)
    }

    pub fn reset(&mut self) {
        self.median.reset();
        self.average.reset();
    }

    /// Approximate total group delay in samples (median + average).
    ///
    /// The median term is an estimate: a nonlinear filter has no single true
    /// group delay, but one sample matches its step response closely enough
    /// for phase budgeting.
    pub fn group_delay_samples() -> f32 {
        // Fully qualified: this associated function shares a name with the
        // free function in `average`, and a bare call here would be needlessly
        // ambiguous to read even though Rust resolves it to the free one.
        1.0 + crate::average::group_delay_samples(N)
    }

    /// Approximate total group delay in seconds at a given sample rate.
    pub fn group_delay_seconds(sample_rate_hz: f32) -> f32 {
        Self::group_delay_samples() / sample_rate_hz
    }
}

impl<const N: usize> Default for ScalarFilter<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Three independent axes, e.g. gyro x/y/z or accel x/y/z.
///
/// The axes are filtered separately and never mixed: a spike on the roll gyro
/// must not perturb pitch or yaw.
#[derive(Clone, Copy, Debug)]
pub struct Vec3Filter<const N: usize> {
    axes: [ScalarFilter<N>; 3],
}

impl<const N: usize> Vec3Filter<N> {
    pub const fn new() -> Self {
        Self {
            axes: [ScalarFilter::new(); 3],
        }
    }

    #[inline]
    pub fn update(&mut self, raw: [f32; 3]) -> [f32; 3] {
        [
            self.axes[0].update(raw[0]),
            self.axes[1].update(raw[1]),
            self.axes[2].update(raw[2]),
        ]
    }

    pub fn reset(&mut self) {
        for a in self.axes.iter_mut() {
            a.reset();
        }
    }
}

impl<const N: usize> Default for Vec3Filter<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Conditions a whole IMU sample: gyro and accel, three axes each.
///
/// Gyro and accel get independent window lengths because they feed different
/// consumers with different bandwidth needs. The gyro drives the inner rate
/// loop and is the most lag-sensitive signal in the vehicle, so it wants the
/// shortest window that tames the noise. The accel is used for attitude
/// reference, changes more slowly, and is far noisier under vibration, so it
/// tolerates — and benefits from — a longer window.
#[derive(Clone, Copy, Debug)]
pub struct ImuFilter<const GYRO_N: usize, const ACCEL_N: usize> {
    gyro: Vec3Filter<GYRO_N>,
    accel: Vec3Filter<ACCEL_N>,
}

impl<const GYRO_N: usize, const ACCEL_N: usize> ImuFilter<GYRO_N, ACCEL_N> {
    pub const fn new() -> Self {
        Self {
            gyro: Vec3Filter::new(),
            accel: Vec3Filter::new(),
        }
    }

    /// Filter a raw sample, returning a new conditioned sample.
    ///
    /// The input is borrowed, not consumed: the caller keeps `raw` for the
    /// flight log and the anomaly detector.
    #[inline]
    pub fn update(&mut self, raw: &ImuSample) -> ImuSample {
        ImuSample {
            gyro: self.gyro.update(raw.gyro),
            accel: self.accel.update(raw.accel),
            // The timestamp is passed through unchanged. It marks when the
            // sample was *taken*, which is what staleness checks and the
            // estimator need; it is not adjusted for filter delay.
            timestamp_us: raw.timestamp_us,
        }
    }

    pub fn reset(&mut self) {
        self.gyro.reset();
        self.accel.reset();
    }
}

impl<const GYRO_N: usize, const ACCEL_N: usize> Default for ImuFilter<GYRO_N, ACCEL_N> {
    fn default() -> Self {
        Self::new()
    }
}
