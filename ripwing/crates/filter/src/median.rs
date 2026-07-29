//! Sliding median-of-3: nonlinear spike rejection.
//!
//! Why a median rather than more low-pass filtering: a single corrupted
//! sample (SPI glitch, ESD event, a bit flip on the bus) is an *outlier*, not
//! noise. A linear filter smears an outlier across its whole window — a
//! 1000 deg/s spike fed into a 5-tap average becomes a 200 deg/s error
//! lasting 5 samples. A median simply discards it: the corrupted value is
//! never the middle of three, so it never reaches the output at all.
//!
//! The cost is that a median is nonlinear, so it has no clean frequency
//! response and cannot be reasoned about with Bode plots. That is why it sits
//! *before* the linear stage rather than replacing it: median kills outliers,
//! the moving average handles genuine broadband noise, and only the linear
//! stage needs to enter the phase budget.
//!
//! Step behaviour worth knowing: a real step (the vehicle actually rotating)
//! passes through with roughly one sample of delay, because after two samples
//! at the new level the median follows. It is not a low-pass — sustained
//! changes get through, isolated ones do not.

/// Median of exactly three values.
///
/// Implemented as "clamp `c` into the range spanned by `a` and `b`", which is
/// branch-light and allocation-free. Equivalent to sorting and taking the
/// middle element.
///
/// NaN handling: comparisons against NaN are all false, so a NaN input may
/// propagate. Sensor drivers should reject NaN before this stage rather than
/// relying on the median to absorb it.
#[inline]
pub fn median3(a: f32, b: f32, c: f32) -> f32 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    if c < lo {
        lo
    } else if c > hi {
        hi
    } else {
        c
    }
}

/// A sliding median-of-3 over a stream of samples.
///
/// Holds the two previous samples; each `update` returns the median of
/// (previous-previous, previous, current).
#[derive(Clone, Copy, Debug, Default)]
pub struct MedianOf3 {
    prev2: f32,
    prev1: f32,
    primed: bool,
}

impl MedianOf3 {
    pub const fn new() -> Self {
        Self {
            prev2: 0.0,
            prev1: 0.0,
            primed: false,
        }
    }

    /// Feed one sample, get the filtered value.
    ///
    /// On the very first sample the history is *primed* with that sample
    /// rather than left at zero. Without priming, the filter would ramp up
    /// from 0 over the first few samples, injecting a startup transient right
    /// at arm time — exactly when a spurious control action is least welcome.
    #[inline]
    pub fn update(&mut self, sample: f32) -> f32 {
        if !self.primed {
            self.prev2 = sample;
            self.prev1 = sample;
            self.primed = true;
            return sample;
        }
        let out = median3(self.prev2, self.prev1, sample);
        self.prev2 = self.prev1;
        self.prev1 = sample;
        out
    }

    /// Clear history. The next sample re-primes the filter.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}
