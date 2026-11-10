//! Vertical channel: barometer altitude fused with vertical acceleration.
//!
//! The barometer measures height directly but is noisy and slow (50 Hz,
//! centimetres of noise, metres of error in prop wash). The accelerometer
//! sees every change in vertical motion instantly, but integrating it twice
//! turns a 0.01 m/s² bias into metres of drift within seconds. Fusing them
//! keeps the accelerometer's response and the barometer's long-term truth.
//!
//! # Third-order complementary filter
//!
//! States: height `h`, climb rate `v`, and accelerometer bias `b`.
//!
//! ```text
//!   predict (every IMU sample):  v += (a_up − b)·dt      h += v·dt
//!   correct (every baro sample): e  = h_baro − h
//!                                h += k1·e·Δt   v += k2·e·Δt   b −= k3·e·Δt
//!   k1 = 3/τ,  k2 = 3/τ²,  k3 = 1/τ³
//! ```
//!
//! `a_up` is the specific force rotated into the world frame, minus gravity:
//! the vertical acceleration the airframe actually has. The error dynamics
//! are `(s + 1/τ)³`: three poles at −1/τ, so `τ` is the one tuning knob, as
//! `1/kp` is for the attitude filter. Below 1/τ the output follows the
//! barometer; above it, the accelerometer. The bias state means a constant
//! accelerometer offset ends up in `b` instead of in the height.
//!
//! # Reference
//!
//! Height is relative to the pressure averaged over the first
//! `reference_samples` readings after boot, so 0 m is where the vehicle sat
//! when it was powered. Weather moves that by about a metre an hour; set a
//! new reference at arming if that matters.

/// Pressure altitude [m] above the reference pressure `p0_pa`
/// (international barometric formula, standard atmosphere).
pub fn pressure_altitude_m(pressure_pa: f32, p0_pa: f32) -> f32 {
    44_330.0 * (1.0 - libm::powf(pressure_pa / p0_pa, 1.0 / 5.255))
}

#[derive(Clone, Copy, Debug)]
pub struct VerticalConfig {
    /// Crossover time constant [s]. Larger: smoother height, slower to
    /// follow the barometer through accelerometer errors.
    pub tau_s: f32,
    /// Barometer readings averaged into the ground reference.
    pub reference_samples: u16,
    /// Accelerometer bias the filter may learn [m/s²]. A larger "bias" is
    /// a fault, not something to cancel silently.
    pub max_bias_mps2: f32,
    /// Without a barometer reading for this long [s], the estimate is
    /// withdrawn (`state()` returns `None`); the accelerometer alone would
    /// drift away within seconds.
    pub timeout_s: f32,
}

impl Default for VerticalConfig {
    fn default() -> Self {
        Self {
            // 2 s: the BMP581 at 50 Hz with IIR is quiet enough that the
            // barometer can be trusted on that timescale.
            tau_s: 2.0,
            // 0.4 s at 50 Hz.
            reference_samples: 20,
            max_bias_mps2: 1.0,
            timeout_s: 0.5,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct VerticalFilter {
    cfg: VerticalConfig,
    h: f32,
    v: f32,
    bias: f32,
    /// Ground reference [Pa]; `None` while averaging.
    p0: Option<f32>,
    p0_sum: f32,
    p0_count: u16,
    last_baro_us: Option<u32>,
    /// Seconds of IMU time since the last barometer reading.
    since_baro_s: f32,
}

impl VerticalFilter {
    pub const fn new(cfg: VerticalConfig) -> Self {
        Self {
            cfg,
            h: 0.0,
            v: 0.0,
            bias: 0.0,
            p0: None,
            p0_sum: 0.0,
            p0_count: 0,
            last_baro_us: None,
            since_baro_s: 0.0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new(self.cfg);
    }

    /// Height (m) and climb rate (m/s), if there is a reference and the
    /// barometer is current.
    pub fn state(&self) -> Option<(f32, f32)> {
        (self.p0.is_some() && self.since_baro_s <= self.cfg.timeout_s).then_some((self.h, self.v))
    }

    /// Learned accelerometer bias [m/s²].
    pub fn bias(&self) -> f32 {
        self.bias
    }

    /// Ground reference pressure (Pa), once set.
    pub fn reference_pa(&self) -> Option<f32> {
        self.p0
    }

    /// Propagate with the world-frame vertical acceleration `a_up` [m/s²]
    /// (gravity removed) over `dt` (s).
    pub fn predict(&mut self, a_up: f32, dt: f32) {
        self.since_baro_s += dt;
        if self.p0.is_none() || !a_up.is_finite() {
            return;
        }
        self.v += (a_up - self.bias) * dt;
        self.h += self.v * dt;
    }

    /// Correct toward one barometer reading.
    pub fn correct(&mut self, pressure_pa: f32, timestamp_us: u32) {
        if !pressure_pa.is_finite() || pressure_pa <= 0.0 {
            return;
        }
        // Interval from the barometer's own timestamps; nominal 50 Hz for
        // the first, and clamped so a gap cannot become one huge step.
        let dt = match self.last_baro_us.replace(timestamp_us) {
            Some(prev) => (timestamp_us.wrapping_sub(prev) as f32 * 1e-6).clamp(0.0, 0.1),
            None => 0.02,
        };
        self.since_baro_s = 0.0;

        let Some(p0) = self.p0 else {
            self.p0_sum += pressure_pa;
            self.p0_count += 1;
            if self.p0_count >= self.cfg.reference_samples.max(1) {
                self.p0 = Some(self.p0_sum / self.p0_count as f32);
                self.h = 0.0;
                self.v = 0.0;
                self.bias = 0.0;
            }
            return;
        };

        let tau = self.cfg.tau_s;
        let e = pressure_altitude_m(pressure_pa, p0) - self.h;
        self.h += 3.0 / tau * e * dt;
        self.v += 3.0 / (tau * tau) * e * dt;
        self.bias -= 1.0 / (tau * tau * tau) * e * dt;
        self.bias = self.bias.clamp(-self.cfg.max_bias_mps2, self.cfg.max_bias_mps2);
    }
}
