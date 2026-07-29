//! Firmware-side inter-task transport.
//!
//! The data *types* that cross task boundaries now live in `ripwing-common`
//! so the host-testable control crate can use them too. This module
//! re-exports them for convenience and adds the one thing that is genuinely
//! firmware-only: the `SeverityFlag` atomic container.
//!
//! Why the flag lives here and not in `common`: it is a concurrency
//! primitive tied to how firmware shares state between RTIC tasks. The pure
//! control logic never touches it — it receives a plain `Severity` value.
//! Keeping the atomic out of `common` keeps `common` free of any
//! synchronization concern.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

// Re-export the shared data vocabulary so existing `crate::transport::Foo`
// paths keep working.
// Re-export the shared data vocabulary so existing `crate::transport::Foo`
// paths keep working. Some (ImuSample, RcEvent) are not consumed yet but are
// part of the intended vocabulary; allow the unused-import warning on them.
#[allow(unused_imports)]
pub use ripwing_common::{
    AttitudeSetpoint, ImuSample, MotorCommand, RateSetpoint, RcEvent, Severity, StateEstimate,
};

/// Lock-free severity channel: one atomic byte shared between the anomaly
/// task (writer) and the control task (reader). The control loop reads this
/// on its hot path with a single relaxed load — no lock, no blocking.
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

/// Per-task iteration counters for bring-up diagnostics.
///
/// Each periodic task increments its counter once per loop. A low-priority
/// diagnostics task samples and clears them every second, so the printed
/// value *is* the task's measured rate in Hz. This turns the rate-monotonic
/// schedule (case study §4.3) from a design assumption into a measurement,
/// with no oscilloscope required.
///
/// Plain atomics rather than RTIC resources: incrementing must be as close
/// to free as possible on the 1 kHz hot paths, and a counter needs no
/// priority-ceiling arbitration.
pub struct TaskCounters {
    pub safety: AtomicU32,
    pub control: AtomicU32,
    pub fusion: AtomicU32,
    pub anomaly: AtomicU32,
    pub logging: AtomicU32,
    pub telemetry: AtomicU32,
}

impl TaskCounters {
    pub const fn new() -> Self {
        Self {
            safety: AtomicU32::new(0),
            control: AtomicU32::new(0),
            fusion: AtomicU32::new(0),
            anomaly: AtomicU32::new(0),
            logging: AtomicU32::new(0),
            telemetry: AtomicU32::new(0),
        }
    }
}

impl Default for TaskCounters {
    fn default() -> Self {
        Self::new()
    }
}

/// Increment a counter. Relaxed: we only need the count to be correct
/// eventually, not to synchronize other memory.
#[inline]
pub fn tick(counter: &AtomicU32) {
    counter.fetch_add(1, Ordering::Relaxed);
}

/// Read a counter and reset it to zero, returning the count since the last
/// call. Over a 1 s sampling window this is the task's rate in Hz.
#[inline]
pub fn take(counter: &AtomicU32) -> u32 {
    counter.swap(0, Ordering::Relaxed)
}
