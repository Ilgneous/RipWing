//! Shared, hardware-free data types for RipWing.
//!
//! This crate is `no_std` and depends on nothing embedded, so it compiles
//! for both the host (unit tests, simulator harness) and the target
//! (firmware). It is the vocabulary the two share: if a type crosses the
//! boundary between control logic and firmware, it lives here.
//!
//! What is NOT here: the `SeverityFlag` atomic container. That is a
//! concurrency primitive tied to how firmware shares state between tasks,
//! and the pure control logic should never see it. Control consumes a
//! plain `Severity` value; firmware owns the flag that stores it.

#![no_std]

/// Estimated vehicle state produced by sensor fusion, consumed by the
/// attitude controller. Widen to the full state vector (attitude
/// quaternion, body rates, position, velocity) as the estimator matures.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StateEstimate {
    /// Roll, pitch, yaw estimates [rad].
    pub attitude: [f32; 3],
    /// Body angular rates [rad/s].
    pub body_rates: [f32; 3],
    /// Monotonic sample timestamp [µs], for staleness checks.
    pub timestamp_us: u32,
}

/// Desired *attitude* (angles) plus thrust — the command consumed by the
/// outer attitude loop. This is what a pilot stick or position loop produces.
///
/// The outer loop compares this against the estimated attitude and produces a
/// `RateSetpoint` for the inner loop. Until the outer loop exists, nothing
/// produces this type yet; it is defined now so the cascade is unambiguous
/// when the outer loop is added.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AttitudeSetpoint {
    /// Desired roll, pitch, yaw *angles* [rad].
    pub angles: [f32; 3],
    /// Desired collective thrust [normalized 0.0..=1.0].
    pub thrust: f32,
}

/// Desired *body rates* plus thrust — the command consumed by the inner rate
/// loop. In a full cascade this is produced by the outer attitude loop; in a
/// pure rate mode (acro) it comes straight from the pilot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateSetpoint {
    /// Desired roll, pitch, yaw *rates* [rad/s].
    pub rates: [f32; 3],
    /// Desired collective thrust [normalized 0.0..=1.0].
    pub thrust: f32,
}

/// Four normalized motor commands [0.0..=1.0], the mixer's output.
/// These become DShot frames at the ESC boundary.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorCommand {
    pub throttle: [f32; 4],
}

/// One raw IMU sample as it leaves the driver, before filtering.
/// Logged raw for the anomaly detector even after the median rejects a
/// spike (§6.4: "filter for control, log the raw for detection").
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub timestamp_us: u32,
}

/// Discrete RC / pilot events. Carried on a lock-free SPSC queue in firmware.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RcEvent {
    Arm,
    Disarm,
    FlightModeChange(u8),
    FailsafeEntry,
}

/// Severity level published by the ML anomaly detector and read by the
/// control loop. This is the plain *value*; firmware wraps it in an atomic
/// container to share it between tasks. The control crate only ever sees
/// this enum, never the container.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Severity {
    #[default]
    Nominal = 0,
    Advisory = 1,
    Warning = 2,
    Critical = 3,
}

impl Severity {
    /// Reconstruct a `Severity` from a raw byte (e.g. after an atomic load).
    /// Any unexpected value is treated as `Nominal`, the safe default.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Severity::Advisory,
            2 => Severity::Warning,
            3 => Severity::Critical,
            _ => Severity::Nominal,
        }
    }
}
