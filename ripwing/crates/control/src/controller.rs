//! The `Controller` trait: the seam between firmware and control logic.
//!
//! The firmware's `attitude_control` task calls `Controller::step` once per
//! tick and knows nothing about what is inside. This is what lets the same
//! controller run in `cargo test` against the simulator plant and on the
//! F411 unchanged — the firmware depends on the trait, not the concrete
//! PID. Swap in an MPC later (case study §7.4) by implementing this trait;
//! the firmware doesn't change.

use ripwing_common::{MotorCommand, Setpoint, Severity, StateEstimate};

/// A flight controller: maps (state, setpoint, health) to motor commands.
pub trait Controller {
    /// Advance one control step.
    ///
    /// * `state`    — latest fused vehicle state
    /// * `setpoint` — commanded setpoint
    /// * `severity` — current ML anomaly severity (a *hint*; the controller
    ///                may switch to a conservative mode, but the ML never
    ///                actuates directly — §7.6)
    /// * `dt`       — timestep in seconds
    fn step(
        &mut self,
        state: &StateEstimate,
        setpoint: &Setpoint,
        severity: Severity,
        dt: f32,
    ) -> MotorCommand;

    /// Reset internal state (integrators, filters). Called on arm / mode
    /// change so stale accumulation never carries across a discontinuity.
    fn reset(&mut self);
}
