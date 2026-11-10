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
//! # Heading from the magnetometer
//!
//! Gravity says nothing about heading, so without help yaw drifts at the
//! gyro bias rate. [`update_mag`](AttitudeEstimator::update_mag) corrects
//! it toward the magnetometer:
//!
//! ```text
//!   m_w  = R(q) · m_body             field in the world frame (tilt removed)
//!   err  = −wrap(atan2(m_w.y, m_w.x) + declination)
//!   q    = Rz(kp_mag · err · Δt) ⊗ q
//! ```
//!
//! The correction is a rotation about the *world vertical* only, so the
//! magnetometer cannot touch roll or pitch: a field bent by motor current
//! or nearby steel can at worst pull heading, never tilt. The very first
//! accepted reading snaps heading instead of converging over several τ,
//! which happens at boot, before arming. Later corrections are rate-limited
//! by `kp_mag`, so a magnetometer that comes back mid-flight cannot jerk
//! the yaw estimate (and with it the heading loop).
//!
//! Readings are gated on field strength: outside the Earth's plausible
//! range, or far from the strength seen when heading was first referenced,
//! the field is disturbed and the sample is ignored.
//!
//! # Altitude from the barometer
//!
//! The vertical channel ([`VerticalFilter`]) runs alongside: every IMU
//! sample propagates it with the world-frame vertical acceleration (which
//! needs this filter's attitude), and every barometer reading corrects it.
//!
//! Known limits, deliberately accepted for a first estimator:
//!
//! * **Sustained acceleration biases tilt.** In a coordinated turn the
//!   accelerometer does not point at gravity. The norm gate below rejects
//!   the obvious cases (|a| far from 1 g); slow, near-1 g errors get through
//!   and are what the EKF's motion model is for.
//! * **No gyro bias estimation.** A constant bias `b` leaves a steady-state
//!   tilt error of about `b · τ` (0.5 °/s × 1 s ≈ 0.5°). Acceptable to fly;
//!   the EKF estimates bias as a state.

use crate::quat::Quat;
use crate::vertical::{VerticalConfig, VerticalFilter};
use crate::{accel_roll_pitch, AttitudeEstimator, GRAVITY};
use core::f32::consts::{PI, TAU};
use ripwing_common::{BaroSample, ImuSample, MagSample, StateEstimate, VerticalState};

/// Hard- and soft-iron correction for the magnetometer, applied before
/// fusion: `m = soft_iron · (raw − hard_iron)`. Both come from a
/// calibration flight or bench rotation of the assembled airframe (motors,
/// battery and wiring in place). Identity means uncalibrated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagCalibration {
    /// Offset from fields that rotate with the airframe [µT].
    pub hard_iron_ut: [f32; 3],
    /// Correction for distortion by nearby ferrous material (row-major).
    pub soft_iron: [[f32; 3]; 3],
}

impl MagCalibration {
    pub const IDENTITY: Self = Self {
        hard_iron_ut: [0.0; 3],
        soft_iron: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub fn apply(&self, raw: [f32; 3]) -> [f32; 3] {
        let d = [0, 1, 2].map(|i| raw[i] - self.hard_iron_ut[i]);
        self.soft_iron.map(|row| row[0] * d[0] + row[1] * d[1] + row[2] * d[2])
    }
}

/// Magnetometer fusion settings.
#[derive(Clone, Copy, Debug)]
pub struct MagConfig {
    /// Heading correction gain [1/s]; heading converges with τ = 1/kp.
    pub kp: f32,
    /// Accept a reading only if its strength is within this fraction of the
    /// strength seen when heading was first referenced (0.25 = ±25 %).
    pub strength_gate: f32,
    /// Plausible Earth field strength (µT): about 22 to 67 everywhere.
    /// Readings outside are disturbed (or uncalibrated) and ignored.
    pub min_field_ut: f32,
    pub max_field_ut: f32,
    /// After this many consecutive strength-gate rejections, take the
    /// current strength as the new reference: the old one was wrong (e.g.
    /// referenced next to a steel bench), not every reading since.
    pub reseed_after: u16,
    /// Magnetic declination (rad), east positive: magnetic north lies this
    /// far east of true north. With it set, yaw is relative to true north
    /// (pointing at magnetic north gives yaw = −declination, since yaw is
    /// counter-clockwise). 0 makes yaw relative to magnetic north.
    pub declination_rad: f32,
    pub calibration: MagCalibration,
    /// Without an accepted reading for this long (s), heading is reported
    /// as no longer referenced.
    pub timeout_s: f32,
}

impl Default for MagConfig {
    fn default() -> Self {
        Self {
            // τ = 5 s: slower than the accel correction, because magnetic
            // disturbances are larger and slower than vibration.
            kp: 0.2,
            strength_gate: 0.25,
            min_field_ut: 20.0,
            max_field_ut: 70.0,
            // 2 s at 100 Hz.
            reseed_after: 200,
            declination_rad: 0.0,
            calibration: MagCalibration::IDENTITY,
            timeout_s: 0.5,
        }
    }
}

fn wrap_pi(x: f32) -> f32 {
    let r = libm::fmodf(x + PI, TAU);
    (if r < 0.0 { r + TAU } else { r }) - PI
}

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
    /// Largest `dt` (s) integrated in one step. A longer gap (stalled
    /// sensor, debugger halt) is clamped to this rather than integrating one
    /// stale rate over the whole gap.
    pub max_dt: f32,
    pub mag: MagConfig,
    pub vertical: VerticalConfig,
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
            mag: MagConfig::default(),
            vertical: VerticalConfig::default(),
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
    /// Heading has been snapped to the magnetometer at least once.
    heading_seeded: bool,
    /// Field strength when heading was first referenced [µT].
    mag_ref_ut: Option<f32>,
    mag_rejected_run: u16,
    last_mag_us: Option<u32>,
    /// Seconds of IMU time since the last accepted magnetometer reading.
    since_mag_s: f32,
    vertical: VerticalFilter,
}

impl ComplementaryFilter {
    pub const fn new(cfg: ComplementaryConfig) -> Self {
        Self {
            cfg,
            q: Quat::IDENTITY,
            initialized: false,
            accel_used: false,
            heading_seeded: false,
            mag_ref_ut: None,
            mag_rejected_run: 0,
            last_mag_us: None,
            since_mag_s: 0.0,
            vertical: VerticalFilter::new(cfg.vertical),
        }
    }

    /// Heading is currently referenced to the magnetometer.
    pub fn heading_referenced(&self) -> bool {
        self.heading_seeded && self.since_mag_s <= self.cfg.mag.timeout_s
    }

    /// The vertical channel, for telemetry (learned bias, reference).
    pub fn vertical(&self) -> &VerticalFilter {
        &self.vertical
    }

    fn estimate(&self, sample: &ImuSample) -> StateEstimate {
        StateEstimate {
            attitude: self.q.to_euler(),
            body_rates: sample.gyro,
            timestamp_us: sample.timestamp_us,
            heading_referenced: self.heading_referenced(),
            vertical: self
                .vertical
                .state()
                .map(|(altitude_m, climb_rate_mps)| VerticalState { altitude_m, climb_rate_mps }),
        }
    }

    /// Strength gate. Returns whether a reading of strength `n` [µT] may be
    /// fused, updating the reference as needed.
    fn accept_strength(&mut self, n: f32) -> bool {
        let m = &self.cfg.mag;
        if !(m.min_field_ut..=m.max_field_ut).contains(&n) {
            return false;
        }
        let Some(r) = self.mag_ref_ut else {
            self.mag_ref_ut = Some(n);
            return true;
        };
        if libm::fabsf(n / r - 1.0) <= m.strength_gate {
            self.mag_rejected_run = 0;
            return true;
        }
        self.mag_rejected_run = self.mag_rejected_run.saturating_add(1);
        if self.mag_rejected_run >= m.reseed_after {
            self.mag_ref_ut = Some(n);
            self.mag_rejected_run = 0;
            return true;
        }
        false
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

            // Vertical channel: specific force in the world frame, minus
            // the 1 g that a vehicle at rest reads.
            let a_up = self.q.body_to_ref(sample.accel)[2] - GRAVITY;
            self.vertical.predict(a_up, dt);
            self.since_mag_s += dt;
        }

        self.estimate(sample)
    }

    fn update_mag(&mut self, mag: &MagSample) {
        // Heading needs tilt to remove: no attitude, no heading.
        if !self.initialized {
            return;
        }
        let m = self.cfg.mag.calibration.apply(mag.field_ut);
        let n = libm::sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        if !n.is_finite() || !self.accept_strength(n) {
            return;
        }
        // Tilt-compensated field. Near the magnetic poles (or with a badly
        // wrong attitude) almost all of it is vertical and the horizontal
        // bearing is noise.
        let mw = self.q.body_to_ref(m);
        if libm::sqrtf(mw[0] * mw[0] + mw[1] * mw[1]) < 0.1 * n {
            return;
        }
        let err = -wrap_pi(libm::atan2f(mw[1], mw[0]) + self.cfg.mag.declination_rad);

        let turn = if self.heading_seeded {
            let dt = match self.last_mag_us {
                Some(prev) => (mag.timestamp_us.wrapping_sub(prev) as f32 * 1e-6).clamp(0.0, 0.1),
                None => 0.01,
            };
            self.cfg.mag.kp * err * dt
        } else {
            self.heading_seeded = true;
            err
        };
        self.last_mag_us = Some(mag.timestamp_us);
        self.since_mag_s = 0.0;
        self.q = Quat::about_z(turn).mul(&self.q);
        self.q.normalize();
    }

    fn update_baro(&mut self, baro: &BaroSample) {
        self.vertical.correct(baro.pressure_pa, baro.timestamp_us);
    }

    fn reset(&mut self) {
        *self = Self::new(self.cfg);
    }
}
