//! Boxcar moving average: the linear smoothing stage.
//!
//! This is where the phase budget is spent, so the tradeoff is explicit.
//! For white noise the output standard deviation falls as `1/sqrt(N)`, while
//! group delay grows linearly as `(N-1)/2` samples. Noise reduction has
//! diminishing returns; lag does not.
//!
//! At a 1 kHz sample rate:
//!
//! ```text
//!   N   delay      noise     -3 dB
//!       (ms)       factor    cutoff
//!   2   0.50       0.71      222 Hz
//!   3   1.00       0.58      148 Hz
//!   4   1.50       0.50      111 Hz
//!   5   2.00       0.45       89 Hz
//!   6   2.50       0.41       74 Hz
//!   8   3.50       0.35       55 Hz
//!  10   4.50       0.32       44 Hz
//!  16   7.50       0.25       28 Hz
//! ```
//!
//! Doubling the window from 4 to 8 buys 30% less noise for 2 ms more lag.
//! Delay inside a feedback loop is phase margin spent, so pick the smallest
//! `N` that makes the noise tolerable rather than the largest the CPU allows.
//! Add roughly 1 more sample of delay for the median stage ahead of this one.
//!
//! `N` is a const generic so the buffer is a fixed-size array with no
//! allocation, and the window length is fixed at compile time — which is what
//! makes execution time constant and the whole thing usable on the hard
//! real-time path.

/// Group delay of an `N`-tap moving average, in samples.
///
/// Returned as `f32` because the true value is `(N-1)/2`, which is a half
/// sample for even `N`.
///
/// Not a `const fn`: floating-point arithmetic in const context is a
/// comparatively recent stabilization, and nothing here needs a compile-time
/// value, so a plain `fn` avoids a needless toolchain-version constraint.
#[inline]
pub fn group_delay_samples(n: usize) -> f32 {
    (n as f32 - 1.0) / 2.0
}

/// Group delay in seconds at a given sample rate. Use this to check the
/// filter against the loop's phase budget.
#[inline]
pub fn group_delay_seconds(n: usize, sample_rate_hz: f32) -> f32 {
    group_delay_samples(n) / sample_rate_hz
}

/// An `N`-tap boxcar moving average.
///
/// `N` must be at least 1. `N == 0` is meaningless (an empty window) and
/// would divide by zero in `update`; since `N` is a const generic, choosing it
/// is a deliberate compile-time act, so this is documented rather than
/// defended against at runtime. `N == 1` is a valid pass-through.
#[derive(Clone, Copy, Debug)]
pub struct MovingAverage<const N: usize> {
    buf: [f32; N],
    idx: usize,
    primed: bool,
}

impl<const N: usize> MovingAverage<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0.0; N],
            idx: 0,
            primed: false,
        }
    }

    /// Feed one sample, get the filtered value.
    ///
    /// On the first sample the whole window is primed with that value, so the
    /// filter starts settled instead of ramping from zero. See the note in
    /// `MedianOf3::update` — a startup transient at arm time is the failure
    /// this avoids.
    ///
    /// The window is summed in full on every call rather than maintained as a
    /// running sum. A running sum is O(1) instead of O(N), but repeatedly
    /// adding and subtracting `f32` values accumulates rounding error without
    /// bound, and at 1 kHz a few minutes of flight is hundreds of thousands of
    /// updates. For the small `N` this filter is designed around, an exact
    /// O(N) sum is both cheap and drift-free, and its constant execution time
    /// is easier to bound for WCET analysis.
    #[inline]
    pub fn update(&mut self, sample: f32) -> f32 {
        if !self.primed {
            self.buf = [sample; N];
            self.idx = 0;
            self.primed = true;
            return sample;
        }

        self.buf[self.idx] = sample;
        self.idx = (self.idx + 1) % N;

        let mut sum = 0.0f32;
        let mut i = 0;
        while i < N {
            sum += self.buf[i];
            i += 1;
        }
        sum / (N as f32)
    }

    /// Clear the window. The next sample re-primes it.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Window length, for callers doing phase-budget arithmetic.
    pub const fn window(&self) -> usize {
        N
    }
}

impl<const N: usize> Default for MovingAverage<N> {
    fn default() -> Self {
        Self::new()
    }
}
