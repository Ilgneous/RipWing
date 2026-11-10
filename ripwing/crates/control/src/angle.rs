//! Outer attitude loop: attitude setpoint (angles) -> rate setpoint.
//!
//! The cascade's first stage. It compares the commanded roll, pitch and
//! yaw angles with the estimate and asks the inner rate loop
//! ([`crate::AttitudeRateController`]) for body rates that close the error:
//!
//! ```text
//!   AttitudeSetpoint --[ this ]--> RateSetpoint --[ rate PIDs + mixer ]--> MotorCommand
//! ```
//!
//! The law is QuadSim's (`CascadedPIDController`): a proportional gain per
//! axis, `rate = k_att · (angle_d − angle)`, with the heading error wrapped
//! so the vehicle always turns the short way round. Gains are unit-free
//! (rad/s per rad), so the tuned values carry over from the simulator as
//! they are (`crate::gains::QUADSIM_7IN_ANGLE`).
//!
//! Two additions QuadSim's controller does not have:
//!
//! * A rate-demand clamp per axis. A large error (a 180° heading change, a
//!   stick slammed to full tilt) would otherwise ask for rates the airframe
//!   cannot stop from in time, which shows up as overshoot. The clamp sets
//!   the fastest the vehicle is ever asked to rotate.
//! * Tilt compensation of thrust: collective is divided by cos φ·cos θ so
//!   the vertical component stays what the pilot commanded as the vehicle
//!   tilts. The divisor is floored at 0.5 (QuadSim's floor, at 60° of
//!   combined tilt) so a near-inverted estimate cannot ask for unbounded
//!   thrust. QuadSim applies the same factor in its thrust feed-forward.
//!
//! Frame: body FLU and ZYX Euler angles, like everything else (see
//! `ripwing_common::StateEstimate`). The angle errors map directly to body
//! rates, which is QuadSim's small-angle simplification. It is the same
//! approximation the tuned gains were validated with.

use crate::controller::AttitudeLoop;
use core::f32::consts::{PI, TAU};
use ripwing_common::{AttitudeSetpoint, RateSetpoint, StateEstimate};

/// Smallest tilt-compensation divisor, cos φ·cos θ (QuadSim's floor).
pub const MIN_TILT_COS: f32 = 0.5;

/// Gains and limits for the outer attitude loop.
#[derive(Clone, Copy, Debug)]
pub struct AngleLoopConfig {
    /// Proportional gain per axis [roll, pitch, yaw]: rad/s of rate demand
    /// per rad of angle error.
    pub k_att: [f32; 3],
    /// Symmetric clamp on the rate demand per axis [rad/s].
    pub max_rate: [f32; 3],
    /// Divide collective by cos φ·cos θ (floored at [`MIN_TILT_COS`]).
    pub tilt_compensation: bool,
}

/// Outer attitude loop. Stateless today (pure P), but owned and reset like
/// any other stage so an integrator or feed-forward filter can be added
/// without changing callers.
pub struct AttitudeAngleController {
    cfg: AngleLoopConfig,
}

impl AttitudeAngleController {
    pub fn new(cfg: AngleLoopConfig) -> Self {
        Self { cfg }
    }

    pub fn config(&self) -> &AngleLoopConfig {
        &self.cfg
    }
}

/// Wrap an angle into [-π, π) (QuadSim's `wrap_pi`). Non-finite input
/// stays non-finite.
pub fn wrap_pi(x: f32) -> f32 {
    let r = libm::fmodf(x + PI, TAU);
    // fmodf keeps the dividend's sign; shift negatives into [0, 2π).
    let r = if r < 0.0 { r + TAU } else { r };
    r - PI
}

impl AttitudeLoop for AttitudeAngleController {
    fn step(&mut self, state: &StateEstimate, sp: &AttitudeSetpoint, _dt: f32) -> RateSetpoint {
        let att = state.attitude;
        let err = [
            sp.angles[0] - att[0],
            sp.angles[1] - att[1],
            wrap_pi(sp.angles[2] - att[2]),
        ];
        let mut rates = [0.0; 3];
        for i in 0..3 {
            let lim = self.cfg.max_rate[i];
            rates[i] = (self.cfg.k_att[i] * err[i]).clamp(-lim, lim);
        }

        let thrust = if self.cfg.tilt_compensation {
            let tilt = (libm::cosf(att[0]) * libm::cosf(att[1])).max(MIN_TILT_COS);
            (sp.thrust / tilt).clamp(0.0, 1.0)
        } else {
            sp.thrust
        };

        RateSetpoint { rates, thrust }
    }

    fn reset(&mut self) {}
}
