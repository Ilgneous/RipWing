//! Attitude rate controller: three PID axes (roll, pitch, yaw) plus mixing.
//!
//! This is the concrete `Controller` the firmware runs during bring-up. It
//! is a *rate* controller — it drives the body angular rates toward the
//! commanded rates. The outer attitude-angle loop wraps this later; per the
//! roadmap you bring the inner rate loop up first.

use crate::controller::Controller;
use crate::mixer;
use crate::pid::{Pid, PidConfig};
use ripwing_common::{MotorCommand, Setpoint, Severity, StateEstimate};

/// Gains for all three axes, straight from the simulator's optimizer.
#[derive(Clone, Copy, Debug)]
pub struct AttitudeGains {
    pub roll: PidConfig,
    pub pitch: PidConfig,
    pub yaw: PidConfig,
}

/// Rate controller: one PID per body axis, feeding the mixer.
pub struct AttitudeRateController {
    roll: Pid,
    pitch: Pid,
    yaw: Pid,
    /// Base thrust passthrough is handled in the mixer; kept here for
    /// possible thrust shaping under degraded severity.
    _reserved: (),
}

impl AttitudeRateController {
    pub fn new(gains: AttitudeGains) -> Self {
        Self {
            roll: Pid::new(gains.roll),
            pitch: Pid::new(gains.pitch),
            yaw: Pid::new(gains.yaw),
            _reserved: (),
        }
    }
}

impl Controller for AttitudeRateController {
    fn step(
        &mut self,
        state: &StateEstimate,
        setpoint: &Setpoint,
        severity: Severity,
        dt: f32,
    ) -> MotorCommand {
        // Commanded rates come from the setpoint; measured rates from the
        // estimator. (In the pure rate loop, setpoint.attitude carries
        // desired body rates.)
        let roll_out = self.roll.update(setpoint.attitude[0], state.body_rates[0], dt);
        let pitch_out = self.pitch.update(setpoint.attitude[1], state.body_rates[1], dt);
        let yaw_out = self.yaw.update(setpoint.attitude[2], state.body_rates[2], dt);

        // Optional conservative response under a Critical anomaly: scale
        // thrust back so a suspected fault has less energy to work with.
        // The ML only *gates* this deterministic action; it never commands
        // motors directly (§7.6).
        let thrust = match severity {
            Severity::Critical => setpoint.thrust * 0.8,
            _ => setpoint.thrust,
        };

        mixer::mix_quad_x(thrust, roll_out, pitch_out, yaw_out)
    }

    fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
    }
}
