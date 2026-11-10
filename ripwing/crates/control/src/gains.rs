//! Tuned gains from QuadSim, converted to the firmware's units.
//!
//! Source: `QuadSim/results/tuning_data/tuned_gains.json`, produced by
//! `QuadSim/tuning/pid_tuning.py` (roll/pitch, stages 1–2) and
//! `QuadSim/tuning/yaw_tuning.py` (yaw, stage 3) on the 10-DOF model of the
//! QuadSim airframe (1.2 kg, 225 mm arms, 7" props, 35 ms motor lag).
//!
//! Two stages: [`QUADSIM_7IN_ANGLE`] for the outer attitude loop and
//! [`QUADSIM_7IN`] for the inner rate loop. The outer loop's P gains are
//! unit-free and carry over as they are. Its rate clamp ([`MAX_RATE`]) is
//! not from QuadSim; it was sized on the ported plant.
//!
//! Rate-loop conversion (see `tests/quadsim/harness.rs`). QuadSim's rate PIDs output
//! torque [N·m]; these output normalized mixer demand. kp, ki, kd and the
//! output limit are divided by the torque per unit demand: 13.52 N·m on
//! roll/pitch and 1.083 N·m on yaw. The integral limit (on ∫error dt) carries
//! over unchanged, and the D low-pass keeps QuadSim's 30 Hz corner,
//! re-derived for the firmware's 1 kHz loop.
//!
//! `tests/quadsim_firmware_gains.rs` recomputes these from the exported
//! QuadSim data and fails if they drift, so after a retune: rerun
//! `python -m QuadSim.analysis.export_step_references`, run that test,
//! and copy the values it prints.
//!
//! Validity: these are for the simulated airframe. The step-response tests
//! show they reproduce QuadSim's responses at 1 kHz, but the numbers are only
//! as good as the plant: re-identify motor lag, thrust and drag coefficients
//! and inertia on the real vehicle before flying them.

use crate::{AngleLoopConfig, AttitudeGains, PidConfig};

/// Rate-loop period these constants are derived for, in seconds (D filter only).
pub const LOOP_DT: f32 = 0.001;

/// Roll/pitch rate PID. kd is effectively zero: with gyro noise priced in,
/// the tuner found D cost more in motor thrash than it bought in damping.
pub const ROLL_PITCH_RATE: PidConfig = PidConfig {
    kp: 0.027_689_47,
    ki: 0.042_528_95,
    kd: 4.650_591_2e-7,
    integral_limit: 0.5,
    output_limit: 0.147_912_04,
    d_filter_alpha: 0.158_600_14,
};

/// Yaw rate PID. Integral-dominated (zero at ki/kp ≈ 70 rad/s): the
/// integrator is what cancels a constant yaw-torque bias such as a tilted
/// motor. The output limit is half of the full drag-torque envelope, so yaw
/// never takes thrust or roll/pitch authority from the mixer.
pub const YAW_RATE: PidConfig = PidConfig {
    kp: 0.023_306_485,
    ki: 1.629_911_9,
    kd: 4.397_530_4e-10,
    integral_limit: 0.5,
    output_limit: 0.25,
    d_filter_alpha: 0.158_600_14,
};

/// Rate-loop gains for [`crate::AttitudeRateController`].
pub const QUADSIM_7IN: AttitudeGains = AttitudeGains {
    roll: ROLL_PITCH_RATE,
    pitch: ROLL_PITCH_RATE,
    yaw: YAW_RATE,
};

/// Attitude P gains \[roll, pitch, yaw\]: angle error (rad) to rate setpoint
/// [rad/s]. Unit-free, so they carry over from QuadSim unchanged.
pub const QUADSIM_7IN_ATTITUDE_P: [f32; 3] = [8.054_744, 8.054_744, 9.166_991];

/// Rate-demand clamp for the outer loop [rad/s]: [roll, pitch, yaw] =
/// 344, 344, 229 °/s. QuadSim has no clamp, so this is not part of its tune.
/// It was sized on the ported plant at 1 kHz (`tests/quadsim_step_response.rs`,
/// section 4):
///
/// * Yaw needs it. Drag torque gives yaw little authority, and while the yaw
///   rate PID sits at its output limit its large integrator winds up.
///   Unclamped, a 45° heading step overshoots 15 % and 90° overshoots 16 %
///   with the motors saturated 18 % of the time. At 4 rad/s, steps of any
///   size settle without overshoot (90°: 0.49 s against 0.74 s), and the
///   tuned 30° step reaches 90 % only 10 ms later.
/// * Roll/pitch barely need it: unclamped steps up to 60° overshoot at most
///   0.5 %. 6 rad/s only engages above ~43° of error and caps the rate any
///   stick input can ask for.
pub const MAX_RATE: [f32; 3] = [6.0, 6.0, 4.0];

/// Outer attitude loop for [`crate::AttitudeAngleController`].
pub const QUADSIM_7IN_ANGLE: AngleLoopConfig = AngleLoopConfig {
    k_att: QUADSIM_7IN_ATTITUDE_P,
    max_rate: MAX_RATE,
    tilt_compensation: true,
};
