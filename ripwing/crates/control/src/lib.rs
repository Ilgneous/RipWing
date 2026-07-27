//! Pure flight control math for RipWing.
//!
//! `no_std`, no HAL, no RTIC — depends only on `ripwing-common`. Everything
//! here compiles and runs on the host, so the control logic is exercised by
//! `cargo test` against the simulator plant before it ever reaches the F411.
//! The firmware calls into this crate through the [`Controller`] trait.

#![no_std]

pub mod attitude;
pub mod controller;
pub mod mixer;
pub mod pid;

pub use attitude::{AttitudeGains, AttitudeRateController};
pub use controller::Controller;
pub use pid::{Pid, PidConfig};

// Tests live in a separate file pulled in only when testing. They use std
// (for the plant model and assertions) even though the crate is no_std —
// that is fine because tests build for the host.
#[cfg(test)]
mod tests;
