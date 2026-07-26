//! Inter-task transport types: the data that crosses task boundaries.
//!
//! Per the architecture (case study §3.2), each link is chosen for its job:
//!   - a lock-free SPSC queue for discrete RC events,
//!   - a byte/record ring buffer for the variable-rate telemetry stream
//!     fanning out to three independent consumers,
//!   - a single atomic for the ML severity flag the control loop reads
//!     on its hot path.
//!
//! The key invariant (§3.3): slow consumers drop data explicitly rather
//! than back-pressuring the 1 kHz producer. Nothing here may block the
//! control chain.

use core::sync::atomic::{AtomicU8, Ordering};

/// Estimated vehicle state produced by sensor fusion, consumed by the
/// attitude controller. Placeholder shape — widen to the full state
/// vector (attitude quaternion, body rates, position, velocity) as the
/// estimator comes online.
#[derive(Clone, Copy, Debug, Default)]
pub struct StateEstimate {
    /// Roll, pitch, yaw estimates [rad].
    pub attitude: [f32; 3],
    /// Body angular rates [rad/s].
    pub body_rates: [f32; 3],
    /// Monotonic sample timestamp [µs], for staleness checks.
    pub timestamp_us: u32,
}

/// Setpoint the pilot / outer loop commands, consumed by attitude control.
#[derive(Clone, Copy, Debug, Default)]
pub struct Setpoint {
    /// Desired roll, pitch, yaw [rad] (or rates, depending on flight mode).
    pub attitude: [f32; 3],
    /// Desired collective thrust [normalized 0.0..=1.0].
    pub thrust: f32,
}

/// Four normalized motor commands [0.0..=1.0], the mixer's output.
/// These become DShot frames at the ESC boundary.
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorCommand {
    pub throttle: [f32; 4],
}

/// One raw IMU sample as it leaves the driver, before filtering.
/// Logged raw for the anomaly detector even after the median rejects a
/// spike (§6.4: "filter for control, log the raw for detection").
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuSample {
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub timestamp_us: u32,
}

/// Discrete RC / pilot events. Carried on a lock-free SPSC queue.
#[derive(Clone, Copy, Debug)]
pub enum RcEvent {
    Arm,
    Disarm,
    FlightModeChange(u8),
    FailsafeEntry,
}

/// Severity level published by the ML anomaly detector and read by the
/// control loop on its hot path. A single atomic byte: the control task
/// does one relaxed load, never a lock. The detector *monitors*; it never
/// actuates (§7.6) — this flag only gates a deterministic fallback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Severity {
    Nominal = 0,
    Advisory = 1,
    Warning = 2,
    Critical = 3,
}

impl Severity {
    fn from_u8(v: u8) -> Self {
        match v {
            1 => Severity::Advisory,
            2 => Severity::Warning,
            3 => Severity::Critical,
            _ => Severity::Nominal,
        }
    }
}

/// Lock-free severity channel: one atomic byte shared between the anomaly
/// task (writer) and the control task (reader).
pub struct SeverityFlag {
    raw: AtomicU8,
}

impl SeverityFlag {
    pub const fn new() -> Self {
        Self {
            raw: AtomicU8::new(Severity::Nominal as u8),
        }
    }

    /// Writer side (anomaly task). Relaxed is sufficient: this is a status
    /// hint, not a synchronization point guarding other memory.
    #[inline]
    pub fn set(&self, s: Severity) {
        self.raw.store(s as u8, Ordering::Relaxed);
    }

    /// Reader side (control task hot path). Single relaxed load, no lock.
    #[inline]
    pub fn get(&self) -> Severity {
        Severity::from_u8(self.raw.load(Ordering::Relaxed))
    }
}

impl Default for SeverityFlag {
    fn default() -> Self {
        Self::new()
    }
}
