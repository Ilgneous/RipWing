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

use core::sync::atomic::{AtomicU8, Ordering};

// Re-export the shared data vocabulary so existing `crate::transport::Foo`
// paths keep working.
// Re-export the shared data vocabulary so existing `crate::transport::Foo`
// paths keep working. Some (ImuSample, RcEvent) are not consumed yet but are
// part of the intended vocabulary; allow the unused-import warning on them.
#[allow(unused_imports)]
pub use ripwing_common::{ImuSample, MotorCommand, RcEvent, Setpoint, Severity, StateEstimate};

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
