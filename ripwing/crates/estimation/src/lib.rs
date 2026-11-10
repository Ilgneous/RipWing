//! Attitude estimation for RipWing.
//!
//! `no_std`, no HAL — depends only on `ripwing-common` and `libm`, so every
//! estimator here runs under `cargo test` on the host before it flies.
//!
//! The firmware calls into this crate through the [`AttitudeEstimator`]
//! trait, the same way it calls the controller through `Controller`. The
//! first implementation is a [`ComplementaryFilter`]: simple enough to debug
//! by inspection, good enough to fly. The EKF that replaces it implements
//! the same trait, and the firmware does not change.
//!
//! # Frame and unit conventions
//!
//! Everything here uses the IMU's *body frame* as delivered by the driver
//! (after any board-mounting rotation), right-handed, z up through the top
//! of the board:
//!
//! * gyro: body angular rate [rad/s], positive by the right-hand rule.
//! * accel: specific force [m/s²]. At rest and level this reads
//!   `[0, 0, +g]` — the sensor measures the reaction to gravity, which
//!   points *up*.
//! * attitude: `[roll, pitch, yaw]` [rad], ZYX Euler angles of the body
//!   relative to a level, yaw-zero reference frame. Each angle is positive
//!   by the right-hand rule about its body axis, the same sign as the gyro
//!   rate that produces it; [`accel_roll_pitch`] gives the exact relation
//!   to the accelerometer, and the tests pin it down. Mapping these to
//!   "roll right / nose up" depends on how the board is mounted, which is
//!   the driver's business, not this crate's.

#![no_std]

pub mod complementary;
pub mod quat;

pub use complementary::{ComplementaryConfig, ComplementaryFilter};
pub use quat::Quat;

use ripwing_common::{ImuSample, StateEstimate};

/// Standard gravity [m/s²].
pub const GRAVITY: f32 = 9.806_65;

/// The seam between firmware and estimation logic.
pub trait AttitudeEstimator {
    /// Fold one conditioned IMU sample into the estimate.
    ///
    /// * `sample` — filtered gyro + accel, with the time it was *taken*.
    /// * `dt`     — seconds since the previous sample, measured by the
    ///   caller from the sensor's own data-ready edges. Passing the real
    ///   interval (rather than a nominal constant) keeps the integration
    ///   correct when the IMU's oscillator is a percent off, or a sample is
    ///   dropped and this one covers two periods.
    fn update(&mut self, sample: &ImuSample, dt: f32) -> StateEstimate;

    /// Forget everything. The next sample re-initializes the estimate.
    fn reset(&mut self);
}

/// Roll and pitch implied by an accelerometer reading alone, assuming the
/// only specific force is the reaction to gravity.
///
/// `roll = atan2(ay, az)`, `pitch = atan2(-ax, sqrt(ay² + az²))`.
/// Yaw is unobservable from gravity and not returned.
pub fn accel_roll_pitch(accel: [f32; 3]) -> (f32, f32) {
    let [ax, ay, az] = accel;
    let roll = libm::atan2f(ay, az);
    let pitch = libm::atan2f(-ax, libm::sqrtf(ay * ay + az * az));
    (roll, pitch)
}

#[cfg(test)]
mod tests;
