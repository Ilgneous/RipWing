//! Complementary attitude filter.
//!
//! The idea in one line: the gyro is right at high frequency and drifts at
//! low frequency; the accelerometer is right at low frequency (on average it
//! points at gravity) and is useless at high frequency (vibration, maneuver
//! accelerations). So integrate the gyro, and continuously nudge the result
//! toward what the accelerometer says, slowly enough that the accel's
//! high-frequency garbage is filtered out but fast enough to cancel drift.
//!
//! Implemented on a quaternion rather than on Euler angles. The Euler form is
//! tempting — `roll += p·dt` — but that is only correct near level: body
//! rates are not Euler-angle rates once the vehicle is tilted, and the
//! mismatch shows up as cross-coupling exactly when you are maneuvering. The
//! quaternion form has no such approximation and no singularity, and it is
//! barely longer. Euler angles come out at the end for the consumers.
//!
//! The accel correction is the explicit complementary filter (Mahony
//! without the integral term):
//!
//! ```text
//!   v      = estimated "up" in body frame (from q)
//!   a      = measured specific force, normalized
//!   e      = a × v                     (rotation that takes v toward a)
//!   ω'     = ω_gyro + kp · e
//!   q     += ½ q ⊗ (0, ω') · dt
//! ```
//!
//! For small errors this is a first-order low-pass on the accel path with
//! time constant `τ = 1 / kp` and the complementary high-pass on the gyro
//! path — hence the name. `kp` is the one tuning knob.
//!
//! Known limits, deliberately accepted for a first estimator:
//!
//! * **Yaw is gyro-only.** Gravity says nothing about heading, so yaw drifts
//!   at the gyro bias rate. The rate loop does not use yaw angle, so this
//!   does not affect flight today; a magnetometer (or the EKF) fixes it.
//! * **Sustained acceleration biases tilt.** In a coordinated turn the
//!   accelerometer does not point at gravity. The norm gate below rejects
//!   the obvious cases (|a| far from 1 g); slow, near-1 g errors get through
//!   and are what the EKF's motion model is for.
//! * **No gyro bias estimation.** A constant bias `b` leaves a steady-state
//!   tilt error of about `b · τ` (0.5 °/s × 1 s ≈ 0.5°). Acceptable to fly;
//!   the EKF estimates bias as a state.

use crate::quat::Quat;
use crate::{accel_roll_pitch, AttitudeEstimator, GRAVITY};
use ripwing_common::{ImuSample, StateEstimate};

/// Tuning for [`ComplementaryFilter`].
#[derive(Clone, Copy, Debug)]
pub struct ComplementaryConfig {
    /// Accel correction gain [1/s]. Time constant of the tilt correction is
    /// `1 / kp` seconds. Larger: drift cancelled faster, more vibration
    /// leaks into attitude. Smaller: smoother, slower to correct.
    pub kp: f32,
    /// Accel is trusted only when its magnitude is within this fraction of
    /// 1 g (0.15 = 0.85 g..1.15 g). Outside that, the vehicle is clearly
    /// accelerating and the reading is not a gravity reference; the filter
    /// coasts on the gyro for that sample.
    pub accel_gate: f32,
    /// Largest `dt` [s] integrated in one step. A longer gap (stalled
    /// sensor, debugger halt) is clamped to this rather than integrating one
    /// stale rate over the whole gap.
    pub max_dt: f32,
}

impl Default for ComplementaryConfig {
    fn default() -> Self {
        Self {
            // τ = 1 s: crossover at ~0.16 Hz. Well below any vibration
            // frequency, well above gyro drift. A conventional starting
            // point; revisit with flight logs.
            kp: 1.0,
            accel_gate: 0.15,
            // 10 samples at 1 kHz.
            max_dt: 0.010,
        }
    }
}

/// Quaternion complementary filter. See the module docs.
#[derive(Clone, Copy, Debug)]
pub struct ComplementaryFilter {
    cfg: ComplementaryConfig,
    q: Quat,
    initialized: bool,
    /// Whether the most recent sample's accel passed the gate. Exposed for
    /// telemetry: a filter that never accepts accel is a gyro integrator.
    accel_used: bool,
}

impl ComplementaryFilter {
    pub const fn new(cfg: ComplementaryConfig) -> Self {
        Self {
            cfg,
            q: Quat::IDENTITY,
            initialized: false,
            accel_used: false,
        }
    }

    /// True once the filter has seen a usable accel reading and seeded its
    /// attitude from it. Until then its output is not an attitude estimate,
    /// and the caller must not publish it — an unpublished estimate goes
    /// stale, which is what keeps the safety monitor from arming on it.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Whether the last update applied an accel correction.
    pub fn accel_used(&self) -> bool {
        self.accel_used
    }

    pub fn quaternion(&self) -> Quat {
        self.q
    }

    /// Normalized accel if it is a believable gravity reference.
    fn gravity_reference(&self, accel: [f32; 3]) -> Option<[f32; 3]> {
        let [ax, ay, az] = accel;
        let norm = libm::sqrtf(ax * ax + ay * ay + az * az);
        if !norm.is_finite() {
            return None;
        }
        let ratio = norm / GRAVITY;
        if libm::fabsf(ratio - 1.0) > self.cfg.accel_gate {
            return None;
        }
        let inv = 1.0 / norm;
        Some([ax * inv, ay * inv, az * inv])
    }
}

impl AttitudeEstimator for ComplementaryFilter {
    fn update(&mut self, sample: &ImuSample, dt: f32) -> StateEstimate {
        let reference = self.gravity_reference(sample.accel);
        self.accel_used = reference.is_some();

        if !self.initialized {
            // Seed from the accelerometer instead of starting at level and
            // converging over several τ. Starting at level would report
            // "level" while the vehicle sits tilted on the bench — and
            // "level and still" is precisely the arming precondition.
            if let Some(a) = reference {
                let (roll, pitch) = accel_roll_pitch(a);
                self.q = Quat::from_euler(roll, pitch, 0.0);
                self.initialized = true;
            }
        } else {
            let dt = if dt.is_finite() && dt > 0.0 {
                dt.min(self.cfg.max_dt)
            } else {
                0.0
            };

            let mut omega = sample.gyro;
            if let Some(a) = reference {
                let v = self.q.up_in_body();
                // e = a × v
                let e = [
                    a[1] * v[2] - a[2] * v[1],
                    a[2] * v[0] - a[0] * v[2],
                    a[0] * v[1] - a[1] * v[0],
                ];
                for i in 0..3 {
                    omega[i] += self.cfg.kp * e[i];
                }
            }
            // A non-finite gyro would poison q permanently; skip it. The
            // driver should already have rejected such a sample.
            if omega.iter().all(|w| w.is_finite()) {
                self.q.integrate(omega, dt);
            }
        }

        StateEstimate {
            attitude: self.q.to_euler(),
            body_rates: sample.gyro,
            timestamp_us: sample.timestamp_us,
        }
    }

    fn reset(&mut self) {
        self.q = Quat::IDENTITY;
        self.initialized = false;
        self.accel_used = false;
    }
}
