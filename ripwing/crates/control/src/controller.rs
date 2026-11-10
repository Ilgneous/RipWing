//! The `Controller` trait: the seam between firmware and control logic.
//!
//! The firmware's `attitude_control` task calls `Controller::step` once per
//! tick and knows nothing about what is inside. This is what lets the same
//! controller run in `cargo test` against the simulator plant and on the
//! F411 unchanged — the firmware depends on the trait, not the concrete
//! PID. Swap in an MPC later (case study §7.4) by implementing this trait;
//! the firmware doesn't change.

use ripwing_common::{AttitudeSetpoint, MotorCommand, RateSetpoint, Severity, StateEstimate};

/// An inner-loop flight controller: maps (rate command, state, health) to
/// motor commands.
///
/// This trait describes the *inner rate loop* specifically — it consumes a
/// `RateSetpoint` (desired body rates) and produces motor commands. The
/// outer attitude loop is a separate stage, [`AttitudeLoop`], that produces
/// the `RateSetpoint` this consumes.
pub trait Controller {
    /// Advance one control step.
    ///
    /// * `state`     — latest fused vehicle state
    /// * `rate_cmd`  — desired body rates + thrust (from outer loop or pilot)
    /// * `severity`  — current ML anomaly severity (a *hint*; the controller
    ///   may switch to a conservative mode, but the ML never
    ///   actuates directly — §7.6)
    /// * `dt`        — timestep in seconds
    fn step(
        &mut self,
        state: &StateEstimate,
        rate_cmd: &RateSetpoint,
        severity: Severity,
        dt: f32,
    ) -> MotorCommand;

    /// Reset internal state (integrators, filters). Called on arm / mode
    /// change so stale accumulation never carries across a discontinuity.
    fn reset(&mut self);
}

/// The outer attitude loop: maps (attitude setpoint, state) to the rate
/// setpoint the inner [`Controller`] consumes.
///
/// A separate stage rather than part of [`Controller`] so the two can run
/// at different rates later, and so a rate (acro) mode can bypass it and
/// feed the inner loop directly.
pub trait AttitudeLoop {
    /// Advance one step.
    ///
    /// * `state` — latest fused vehicle state
    /// * `sp`    — desired attitude angles + collective
    /// * `dt`    — timestep in seconds
    fn step(&mut self, state: &StateEstimate, sp: &AttitudeSetpoint, dt: f32) -> RateSetpoint;

    /// Reset internal state. Called with [`Controller::reset`].
    fn reset(&mut self);
}
