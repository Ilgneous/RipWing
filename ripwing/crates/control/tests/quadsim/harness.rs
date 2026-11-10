//! Closed loop: the firmware's control cascade driving the ported plant,
//! set up to be comparable with QuadSim's `CascadedPIDController` attitude
//! loops.
//!
//! What is the code under test and what is test scaffolding:
//!
//! * Under test, in f32, exactly as the firmware calls them:
//!   - `AttitudeAngleController::step`: the outer loop (angle error ->
//!     rate setpoint, heading wrap, rate clamp, tilt compensation);
//!   - `AttitudeRateController::step`: the three rate PIDs and `mix_quad_x`.
//! * Scaffolding, mirroring QuadSim so the comparison is like-for-like:
//!   - the collective: the hover fraction m·g / (4·T_max). The outer loop's
//!     tilt compensation turns it into QuadSim's m·g/max(cosφ·cosθ, 0.5)
//!     feed-forward (its altitude PID is off in the reference data);
//!   - the ESC model: throttle = ω²/ω_max² (thrust-linear), which makes the
//!     mixer the exact inverse of QuadSim's ControlAllocator.
//!
//! Unit conversion. QuadSim's rate PIDs output torque [N·m]; the firmware's
//! output a normalized per-axis mixer demand. With the ESC model above, a
//! roll/pitch demand `u` gives torque 4·u·T_max·d (d = arm/√2) and a yaw
//! demand gives 4·u·k_m·ω_max², so every gain, and the output clamp, is
//! divided by that factor. The integral clamp is on ∫error dt in both
//! implementations and carries over unscaled.
//!
//! Known structural difference: QuadSim differentiates the rate ERROR, the
//! firmware PID differentiates the MEASUREMENT (no derivative kick). Inside
//! the cascade the error-derivative adds kd·k_att·(dθ_d/dt − dθ/dt): a
//! one-sample kick at each setpoint step plus extra rate damping of
//! kd·k_att, which is 0.02% of kp for the tuned gains but 16% for QuadSim's
//! defaults. [`DTerm`] selects whether to run the firmware's PID as wired
//! (`AsFirmware`) or to feed it QuadSim's derivative signal
//! (`PythonEquivalent`) so every remaining difference must be a port error.

use super::fixture::Case;
use super::plant::{Plant, PlantParams, State};
use ripwing_common::{AttitudeSetpoint, RateSetpoint, Severity, StateEstimate};
use ripwing_control::gains::MAX_RATE;
use ripwing_control::{
    AngleLoopConfig, AttitudeAngleController, AttitudeGains, AttitudeLoop, AttitudeRateController,
    Controller, PidConfig,
};
use std::f64::consts::PI;

/// Firmware motor index (0 FR, 1 RR, 2 RL, 3 FL — see `mixer.rs`) to
/// QuadSim motor index (0 FR, 1 FL, 2 RL, 3 RR).
pub const FIRMWARE_TO_PLANT: [usize; 4] = [0, 3, 2, 1];

/// Torque produced per unit of mixer demand, per axis [N·m].
pub fn torque_per_unit(plant: &Plant) -> [f64; 3] {
    let d = plant.p.arm_length / 2f64.sqrt();
    let rp = 4.0 * plant.max_motor_thrust() * d;
    let y = 4.0 * plant.p.k_m * plant.p.max_omega * plant.p.max_omega;
    [rp, rp, y]
}

/// QuadSim's D-term low-pass coefficient at this dt.
pub fn lpf_alpha(dt: f64, hz: f64) -> f64 {
    dt / (dt + 1.0 / (2.0 * PI * hz))
}

/// Translate one QuadSim rate PID (torque units) into a firmware `PidConfig`.
pub fn pid_config(case: &Case, axis: &str, unit: f64, dt: f64) -> PidConfig {
    let g = |k: &str| case.get(&format!("{axis}.{k}"));
    PidConfig {
        kp: (g("kp") / unit) as f32,
        ki: (g("ki") / unit) as f32,
        kd: (g("kd") / unit) as f32,
        integral_limit: g("integ_limit") as f32,
        output_limit: (g("out_limit") / unit) as f32,
        d_filter_alpha: lpf_alpha(dt, g("d_lpf_hz")) as f32,
    }
}

/// How the rate PIDs are fed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DTerm {
    /// As the firmware runs: setpoint = attitude-loop rate demand,
    /// measurement = gyro rate. Derivative acts on the measurement.
    AsFirmware,
    /// Same PID code, inputs rewritten so its derivative sees exactly what
    /// QuadSim's does: setpoint 0, measurement = rate − rate_demand. Then
    /// error, integral and P are unchanged, and −d(measurement)/dt equals
    /// QuadSim's d(error)/dt, kick included.
    PythonEquivalent,
}

/// One simulated step, same layout as a fixture row.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub t: f64,
    pub state: State,
    /// Commanded rotor speeds, QuadSim order [rad/s].
    pub cmd: [f64; 4],
}

/// Both stages' configuration for one run.
#[derive(Clone, Copy, Debug)]
pub struct Setup {
    pub rate: AttitudeGains,
    pub angle: AngleLoopConfig,
}

/// The gains QuadSim used for `case`, converted for control period `dt`.
/// No rate clamp, like QuadSim; see [`Setup::with_firmware_limits`].
pub fn setup_from_case(case: &Case, dt: f64) -> Setup {
    let plant = Plant::new(PlantParams::default());
    let unit = torque_per_unit(&plant);
    Setup {
        rate: AttitudeGains {
            roll: pid_config(case, "p", unit[0], dt),
            pitch: pid_config(case, "q", unit[1], dt),
            yaw: pid_config(case, "r", unit[2], dt),
        },
        angle: AngleLoopConfig {
            k_att: ["k_att_roll", "k_att_pitch", "k_att_yaw"].map(|k| case.get(k) as f32),
            max_rate: [f32::INFINITY; 3],
            tilt_compensation: true,
        },
    }
}

impl Setup {
    /// Add the firmware's outer-loop rate clamp ([`MAX_RATE`]), so the run
    /// uses the configuration the firmware flies rather than QuadSim's.
    pub fn with_firmware_limits(mut self) -> Self {
        self.angle.max_rate = MAX_RATE;
        self
    }
}

/// Run the firmware controller against the plant over `case`'s setpoint
/// sequence, at control period `dt` (the case's own dt to reproduce it;
/// another value to ask "do these gains survive a rate change?"). `setup`
/// is normally [`setup_from_case`] for the same `dt`.
///
/// The setpoint is read from the fixture as a zero-order hold, so `dt`
/// need not equal the fixture's.
pub fn run(case: &Case, setup: &Setup, dt: f64, dterm: DTerm) -> Vec<Sample> {
    let ref_dt = case.dt();
    let duration = case.rows.len() as f64 * ref_dt;
    let sp = |t: f64| {
        // Zero-order-hold lookup; the epsilon keeps exact multiples of the
        // reference period (k*dt == j*ref_dt) on the right sample despite
        // float rounding.
        let j = (((t + 1e-9) / ref_dt).floor() as usize).min(case.rows.len() - 1);
        case.rows[j].sp
    };
    simulate(setup, sp, duration, dt, dterm)
}

/// Run the cascade against the plant from hover, following the attitude
/// setpoint `sp(t)` = [roll, pitch, yaw] [rad] at a hover collective.
pub fn simulate(
    setup: &Setup,
    sp: impl Fn(f64) -> [f64; 3],
    duration: f64,
    dt: f64,
    dterm: DTerm,
) -> Vec<Sample> {
    let plant = Plant::new(PlantParams::default());
    let mut outer = AttitudeAngleController::new(setup.angle);
    let mut ctrl = AttitudeRateController::new(setup.rate);
    outer.reset();
    ctrl.reset();
    let full_thrust = 4.0 * plant.max_motor_thrust();
    let hover = (plant.p.mass * plant.p.g / full_thrust) as f32;

    let mut x = plant.hover_state();
    let mut out = Vec::new();
    let n = (duration / dt).round() as usize;
    for k in 0..n {
        let t = k as f64 * dt;
        let est = StateEstimate {
            attitude: [x[0] as f32, x[1] as f32, x[2] as f32],
            body_rates: [x[3] as f32, x[4] as f32, x[5] as f32],
            timestamp_us: 0,
        };

        // ---- code under test (f32, as on the F411) -----------------------
        let att_sp = AttitudeSetpoint { angles: sp(t).map(|a| a as f32), thrust: hover };
        let rate_cmd = outer.step(&est, &att_sp, dt as f32);
        let (est, rate_cmd) = match dterm {
            DTerm::AsFirmware => (est, rate_cmd),
            DTerm::PythonEquivalent => (
                StateEstimate {
                    body_rates: [0, 1, 2].map(|i| est.body_rates[i] - rate_cmd.rates[i]),
                    ..est
                },
                RateSetpoint { rates: [0.0; 3], ..rate_cmd },
            ),
        };
        let motors = ctrl.step(&est, &rate_cmd, Severity::Nominal, dt as f32);

        // ---- ESC model: thrust-linear throttle -> rotor speed command ----
        let mut w_cmd = [0.0; 4];
        for (fw, &pl) in FIRMWARE_TO_PLANT.iter().enumerate() {
            w_cmd[pl] = plant.p.max_omega * (motors.throttle[fw] as f64).sqrt();
        }

        out.push(Sample { t, state: x, cmd: w_cmd });
        x = plant.step_rk4(&x, &w_cmd, dt);
    }
    out
}
