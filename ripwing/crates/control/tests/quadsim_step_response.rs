//! Layer 2: the firmware controller reproduces QuadSim's step responses.
//!
//! Each case replays the setpoint sequence QuadSim logged, runs the Rust
//! `AttitudeRateController` (f32 rate PIDs + mixer) on the ported plant,
//! and compares against what QuadSim's own controller did on the same
//! plant. See `quadsim/harness.rs` for what is under test versus
//! scaffolding.
//!
//! Three questions, in order:
//!
//! 1. `*_python_equivalent`: fed the same derivative signal as QuadSim's
//!    PID, does the firmware controller produce the SAME trajectory? This
//!    checks everything except the D-term structure (gain scaling, mixer
//!    signs and geometry, integrator and its clamp, output clamp, D filter,
//!    f32), so the tolerance is f32 noise. Any port error fails here.
//! 2. `*_as_firmware`: wired as the firmware actually runs it (derivative on
//!    measurement), does it give the same step response? For the tuned
//!    gains it must. QuadSim's untuned defaults are also checked, with the
//!    expected (and explained) extra overshoot.
//! 3. `*_at_firmware_rate`: the gains were tuned at 200 Hz; the firmware
//!    runs at the IMU's 1 kHz. Does the tuned response survive the change?

mod quadsim;

use quadsim::fixture::{self, Case};
use quadsim::harness::{self, DTerm, Sample};

const RAD2DEG: f64 = 180.0 / std::f64::consts::PI;

/// Firmware control rate: one step per ICM-42688-P sample.
const FIRMWARE_DT: f64 = 0.001;

/// 10→90 % rise time, overshoot and end error of one axis's response to
/// the first setpoint step.
#[derive(Debug, Clone, Copy)]
struct StepMetrics {
    rise_s: f64,
    overshoot_pct: f64,
    /// |angle − target| at the end of the step [deg].
    end_err_deg: f64,
}

fn step_metrics(t: &[f64], y: &[f64], sp: &[f64]) -> StepMetrics {
    let start = sp.iter().position(|&s| s != 0.0).expect("no step");
    let end = start + sp[start..].iter().position(|&s| s != sp[start]).unwrap_or(sp.len() - start);
    let target = sp[start];
    let cross = |frac: f64| (start..end).find(|&i| y[i] / target >= frac).map_or(f64::NAN, |i| t[i]);
    let peak = y[start..end].iter().map(|v| v / target).fold(f64::MIN, f64::max);
    StepMetrics {
        rise_s: cross(0.9) - cross(0.1),
        overshoot_pct: 100.0 * (peak - 1.0).max(0.0),
        end_err_deg: (y[end - 1] - target).abs() * RAD2DEG,
    }
}

struct Comparison {
    /// Worst |Rust − Python| per Euler angle over the whole run [deg].
    angle_err_deg: [f64; 3],
    /// Worst |Rust − Python| per body rate [deg/s].
    rate_err_dps: [f64; 3],
    /// Worst rotor-speed command difference, as a fraction of hover speed.
    /// Only meaningful when both run at the same rate (stride 1).
    cmd_err_rel: f64,
    py: StepMetrics,
    rs: StepMetrics,
}

impl Comparison {
    fn max_angle_err(&self) -> f64 {
        self.angle_err_deg.iter().copied().fold(0.0, f64::max)
    }
    fn max_rate_err(&self) -> f64 {
        self.rate_err_dps.iter().copied().fold(0.0, f64::max)
    }
}

/// Compare a Rust run against the case's Python rows. `rs` may run faster
/// than the reference; it is sampled at the reference instants.
fn compare(case: &Case, axis: usize, rs: &[Sample]) -> Comparison {
    let stride = rs.len() / case.rows.len();
    assert_eq!(rs.len(), stride * case.rows.len(), "rate must be an integer multiple");
    let rs: Vec<Sample> = rs.iter().step_by(stride).copied().collect();
    let hover = quadsim::plant::Plant::new(Default::default()).omega_hover();

    let (mut angle, mut rate, mut cmd) = ([0.0f64; 3], [0.0f64; 3], 0.0f64);
    for (r, s) in case.rows.iter().zip(&rs) {
        assert!((r.t - s.t).abs() < 1e-9, "time misaligned: {} vs {}", r.t, s.t);
        for j in 0..3 {
            angle[j] = angle[j].max((s.state[j] - r.euler[j]).abs() * RAD2DEG);
            rate[j] = rate[j].max((s.state[3 + j] - r.rates[j]).abs() * RAD2DEG);
        }
        if stride == 1 {
            for m in 0..4 {
                cmd = cmd.max((s.cmd[m] - r.cmd[m]).abs() / hover);
            }
        }
    }
    let t: Vec<f64> = case.rows.iter().map(|r| r.t).collect();
    let sp: Vec<f64> = case.rows.iter().map(|r| r.sp[axis]).collect();
    let py: Vec<f64> = case.rows.iter().map(|r| r.euler[axis]).collect();
    let ry: Vec<f64> = rs.iter().map(|s| s.state[axis]).collect();
    let c = Comparison {
        angle_err_deg: angle,
        rate_err_dps: rate,
        cmd_err_rel: cmd,
        py: step_metrics(&t, &py, &sp),
        rs: step_metrics(&t, &ry, &sp),
    };
    println!(
        "{} (stride {stride}): max |Δangle| {:.2e}°  max |Δrate| {:.2e}°/s  max |Δcmd| {:.2e} hover\n  \
         python: rise {:.3} s, overshoot {:.2} %, end err {:.3}°\n  \
         rust:   rise {:.3} s, overshoot {:.2} %, end err {:.3}°",
        case.name, c.max_angle_err(), c.max_rate_err(), c.cmd_err_rel,
        c.py.rise_s, c.py.overshoot_pct, c.py.end_err_deg,
        c.rs.rise_s, c.rs.overshoot_pct, c.rs.end_err_deg,
    );
    c
}

fn run(name: &str, axis: usize, dt: Option<f64>, dterm: DTerm) -> Comparison {
    let fx = fixture::load();
    let case = fx.case(name);
    let rs = harness::run(case, dt.unwrap_or(case.dt()), dterm);
    compare(case, axis, &rs)
}

// ---------------------------------------------------------------------------
// 1. Same derivative signal as QuadSim: must be the same trajectory.
// ---------------------------------------------------------------------------
//
// Measured: < 1e-5° and < 1e-4 °/s on every case (f32 rounding in the
// controller). The bounds are ~100x that, and still orders of magnitude
// below any real port error (a wrong gain scale, sign or clamp shows up as
// degrees).

fn assert_identical(name: &str, axis: usize) {
    let c = run(name, axis, None, DTerm::PythonEquivalent);
    assert!(c.max_angle_err() < 1e-3, "{name}: angle diverged {:.2e}°", c.max_angle_err());
    assert!(c.max_rate_err() < 1e-2, "{name}: rate diverged {:.2e}°/s", c.max_rate_err());
    assert!(c.cmd_err_rel < 1e-4, "{name}: motor cmd diverged {:.2e}", c.cmd_err_rel);
}

#[test]
fn roll_doublet_tuned_python_equivalent() {
    assert_identical("roll_tuned", 0);
}

#[test]
fn pitch_doublet_tuned_python_equivalent() {
    assert_identical("pitch_tuned", 1);
}

#[test]
fn roll_doublet_default_gains_python_equivalent() {
    assert_identical("roll_default", 0);
}

/// 30° yaw step with the stage-3 yaw gains (QuadSim/tuning/yaw_tuning.py).
/// Checks the yaw path's gain scaling, output clamp and mixer signs. Before
/// QuadSim's rotor-reaction sign fix this case ran away to ~210°.
#[test]
fn yaw_step_python_equivalent() {
    assert_identical("yaw_tuned", 2);
}

// ---------------------------------------------------------------------------
// 2. As the firmware runs it (derivative on measurement).
// ---------------------------------------------------------------------------

/// Tuned gains: the tuner drives kd to ~0 on every axis (kd·k_att is
/// 0.02 % of kp on roll/pitch, ~1e-7 % on yaw), so dropping the
/// error-derivative barely matters. Measured: 0.002° worst-case on
/// roll/pitch, 0.0000° on yaw, identical rise/overshoot.
fn assert_reproduces(c: &Comparison) {
    assert!(c.max_angle_err() < 0.1, "trajectory differs by {:.3}°", c.max_angle_err());
    assert!((c.rs.rise_s - c.py.rise_s).abs() <= 0.010, "rise {} vs {}", c.rs.rise_s, c.py.rise_s);
    assert!((c.rs.overshoot_pct - c.py.overshoot_pct).abs() <= 1.0, "overshoot {} vs {}", c.rs.overshoot_pct, c.py.overshoot_pct);
    assert!((c.rs.end_err_deg - c.py.end_err_deg).abs() <= 0.05, "end err {} vs {}", c.rs.end_err_deg, c.py.end_err_deg);
}

#[test]
fn roll_doublet_tuned_as_firmware() {
    assert_reproduces(&run("roll_tuned", 0, None, DTerm::AsFirmware));
}

#[test]
fn pitch_doublet_tuned_as_firmware() {
    assert_reproduces(&run("pitch_tuned", 1, None, DTerm::AsFirmware));
}

#[test]
fn yaw_step_as_firmware() {
    assert_reproduces(&run("yaw_tuned", 2, None, DTerm::AsFirmware));
}

/// QuadSim's untuned defaults (kd·k_att = 16 % of kp) do NOT transfer
/// exactly, and this test pins down how: the firmware PID lacks the
/// error-derivative's kd·k_att damping, so it overshoots more (measured
/// 42.2 % vs 35.2 %; ~1.5° worst-case). Same rise time, because the first
/// part of the step is P-dominated. If this starts failing in the other
/// direction, something other than the D structure changed.
#[test]
fn roll_doublet_default_gains_as_firmware() {
    let c = run("roll_default", 0, None, DTerm::AsFirmware);
    let extra_overshoot = c.rs.overshoot_pct - c.py.overshoot_pct;
    assert!((3.0..15.0).contains(&extra_overshoot), "extra overshoot {extra_overshoot:.1} pp");
    assert!((c.rs.rise_s - c.py.rise_s).abs() <= 0.010);
    assert!(c.max_angle_err() < 2.5, "trajectory differs by {:.2}°", c.max_angle_err());
}

// ---------------------------------------------------------------------------
// 3. At the firmware's loop rate.
// ---------------------------------------------------------------------------
//
// QuadSim's tuning note: "Keep the same loop rate on target, or re-verify
// after changing it." This is that re-verification. Only the D low-pass
// coefficient is re-derived for the new dt (same 30 Hz corner); the gains
// are continuous-time and carry over. Measured: rise 0.120 s vs 0.115 s,
// overshoot 0 % vs 0.2 %, end error unchanged; the trajectory differs by up
// to 0.36° (yaw: 0.40°, same rise and overshoot) because the 1 kHz loop
// reacts 4 ms sooner after each step.

fn assert_survives_rate_change(name: &str, axis: usize) {
    let c = run(name, axis, Some(FIRMWARE_DT), DTerm::AsFirmware);
    assert!((c.rs.rise_s - c.py.rise_s).abs() <= 0.010, "rise {} vs {}", c.rs.rise_s, c.py.rise_s);
    assert!(c.rs.overshoot_pct <= c.py.overshoot_pct + 2.0, "overshoot {}", c.rs.overshoot_pct);
    assert!(c.rs.end_err_deg <= c.py.end_err_deg + 0.05, "end err {}", c.rs.end_err_deg);
    assert!(c.max_angle_err() < 0.5, "trajectory differs by {:.3}°", c.max_angle_err());
}

#[test]
fn roll_doublet_tuned_at_firmware_rate() {
    assert_survives_rate_change("roll_tuned", 0);
}

#[test]
fn pitch_doublet_tuned_at_firmware_rate() {
    assert_survives_rate_change("pitch_tuned", 1);
}

#[test]
fn yaw_step_at_firmware_rate() {
    assert_survives_rate_change("yaw_tuned", 2);
}
