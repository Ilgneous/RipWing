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
//!    runs at the IMU's 1 kHz with an outer-loop rate clamp QuadSim lacks.
//!    Does the tuned response survive both?
//! 4. Large steps QuadSim never tested, as flown: does the clamp do its job,
//!    and does the heading wrap the short way?

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

/// Reproduce a QuadSim case at its own rate with QuadSim's controller
/// structure (no outer-loop rate clamp).
fn run(name: &str, axis: usize, dterm: DTerm) -> Comparison {
    let fx = fixture::load();
    let case = fx.case(name);
    let setup = harness::setup_from_case(case, case.dt());
    compare(case, axis, &harness::run(case, &setup, case.dt(), dterm))
}

/// Run a QuadSim case's setpoints through the controller exactly as the
/// firmware flies it: 1 kHz, derivative on measurement, rate clamp on.
fn run_as_flown(name: &str, axis: usize) -> Comparison {
    let fx = fixture::load();
    let case = fx.case(name);
    let setup = harness::setup_from_case(case, FIRMWARE_DT).with_firmware_limits();
    compare(case, axis, &harness::run(case, &setup, FIRMWARE_DT, DTerm::AsFirmware))
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
    let c = run(name, axis, DTerm::PythonEquivalent);
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
    assert_reproduces(&run("roll_tuned", 0, DTerm::AsFirmware));
}

#[test]
fn pitch_doublet_tuned_as_firmware() {
    assert_reproduces(&run("pitch_tuned", 1, DTerm::AsFirmware));
}

#[test]
fn yaw_step_as_firmware() {
    assert_reproduces(&run("yaw_tuned", 2, DTerm::AsFirmware));
}

/// QuadSim's untuned defaults (kd·k_att = 16 % of kp) do NOT transfer
/// exactly, and this test pins down how: the firmware PID lacks the
/// error-derivative's kd·k_att damping, so it overshoots more (measured
/// 42.2 % vs 35.2 %; ~1.5° worst-case). Same rise time, because the first
/// part of the step is P-dominated. If this starts failing in the other
/// direction, something other than the D structure changed.
#[test]
fn roll_doublet_default_gains_as_firmware() {
    let c = run("roll_default", 0, DTerm::AsFirmware);
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
// after changing it." This is that re-verification, with the controller
// configured exactly as the firmware flies it, including the outer loop's
// rate clamp (which QuadSim does not have). Only the D low-pass
// coefficient is re-derived for the new dt (same 30 Hz corner); the gains
// are continuous-time and carry over.
//
// Roll/pitch: rise 0.120 s vs 0.115 s, overshoot 0 % vs 0.2 %, end error
// unchanged. The trajectory differs by up to 0.36° because the 1 kHz loop
// reacts 4 ms sooner after each step. The clamp does not engage. Yaw: the
// 4 rad/s clamp engages briefly at the start of the 30° step (which asks
// for 4.8 rad/s).

fn assert_survives_rate_change(name: &str, axis: usize) {
    let c = run_as_flown(name, axis);
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

/// Yaw as flown: the 4 rad/s clamp caps the first ~0.1 s of the 30° step
/// (QuadSim asks for 4.8 rad/s). Measured: rise 0.135 s vs 0.125 s, the
/// heading trails QuadSim's by up to 1.45° while slewing, and overshoot
/// (0 %) and end error are unchanged. That lag is what buys the overshoot-
/// free large steps in section 4. A clamp that bit harder would show here
/// as a slower rise.
#[test]
fn yaw_step_at_firmware_rate() {
    let c = run_as_flown("yaw_tuned", 2);
    assert!(c.rs.rise_s <= c.py.rise_s + 0.015, "rise {} vs {}", c.rs.rise_s, c.py.rise_s);
    assert!(c.rs.overshoot_pct <= c.py.overshoot_pct + 1.0, "overshoot {}", c.rs.overshoot_pct);
    assert!(c.rs.end_err_deg <= c.py.end_err_deg + 0.05, "end err {}", c.rs.end_err_deg);
    assert!(c.max_angle_err() < 2.0, "trajectory differs by {:.3}°", c.max_angle_err());
}

// ---------------------------------------------------------------------------
// 4. Beyond QuadSim's cases: large steps, as flown.
// ---------------------------------------------------------------------------
//
// QuadSim only tunes on 15° tilts and 30° of heading. A pilot can ask for
// more, and that is where the outer loop's rate clamp matters. These run
// the firmware configuration (1 kHz, rate clamp) from hover on the ported
// plant. There is no Python reference; the checks are absolute.

/// Step of `deg` on `axis` at t = 0.1 s, held for 2.9 s.
fn big_step(axis: usize, deg: f64) -> Vec<Sample> {
    let fx = fixture::load();
    let setup = harness::setup_from_case(fx.case("yaw_tuned"), FIRMWARE_DT).with_firmware_limits();
    let target = deg / RAD2DEG;
    let sp = move |t: f64| {
        let mut a = [0.0; 3];
        if t >= 0.1 {
            a[axis] = target;
        }
        a
    };
    harness::simulate(&setup, sp, 3.0, FIRMWARE_DT, DTerm::AsFirmware)
}

/// Overshoot [%] and 2 % settling time [s] of a [`big_step`].
fn overshoot_and_settle(r: &[Sample], axis: usize, deg: f64) -> (f64, f64) {
    let target = deg / RAD2DEG;
    let peak = r.iter().map(|s| s.state[axis]).fold(f64::MIN, f64::max);
    let settle = r
        .iter()
        .rev()
        .find(|s| (s.state[axis] - target).abs() > 0.02 * target)
        .map_or(0.0, |s| s.t - 0.1);
    (100.0 * (peak / target - 1.0), settle)
}

/// Unclamped, these overshoot by 0.5 % (60° roll), 16 % (90° yaw) and 10 %
/// (170° yaw), with the yaw rate PID wound up against its output limit.
/// Measured with the clamp: no overshoot; settling 0.42 s, 0.49 s, 0.79 s.
#[test]
fn large_steps_settle_without_overshoot() {
    for (axis, deg, max_settle) in [(0, 60.0, 0.5), (1, 60.0, 0.5), (2, 90.0, 0.6), (2, 170.0, 0.9)] {
        let r = big_step(axis, deg);
        let (ovs, settle) = overshoot_and_settle(&r, axis, deg);
        println!("axis {axis} {deg}°: overshoot {ovs:.2} %, settle {settle:.3} s");
        assert!(ovs < 0.5, "axis {axis} {deg}°: overshoot {ovs:.2} %");
        assert!(settle < max_settle, "axis {axis} {deg}°: settle {settle:.3} s");
    }
}

/// The clamp holds the demand, so the rate the vehicle actually reaches
/// is bounded too (the rate loop overshoots its setpoint a little).
/// Measured peaks: 378 °/s roll (clamp 344), 273 °/s yaw (clamp 229).
#[test]
fn large_steps_respect_rate_clamp() {
    let lim = ripwing_control::gains::MAX_RATE;
    for (axis, deg) in [(0, 60.0), (2, 170.0)] {
        let r = big_step(axis, deg);
        let peak = r.iter().map(|s| s.state[3 + axis].abs()).fold(0.0, f64::max);
        assert!(peak < 1.25 * lim[axis] as f64, "axis {axis}: peak rate {:.0} °/s", peak * RAD2DEG);
    }
}

/// Heading 170° -> -170° goes the short way, through 180° (20° of turn),
/// not back through 0 (340°). The plant's yaw is not wrapped, so the short
/// way ends at +190°.
#[test]
fn heading_wraps_the_short_way() {
    let fx = fixture::load();
    let setup = harness::setup_from_case(fx.case("yaw_tuned"), FIRMWARE_DT).with_firmware_limits();
    let (a, b) = (170.0 / RAD2DEG, -170.0 / RAD2DEG);
    let r = harness::simulate(&setup, |t| [0.0, 0.0, if t < 1.5 { a } else { b }], 3.0, FIRMWARE_DT, DTerm::AsFirmware);
    let after: Vec<f64> = r.iter().filter(|s| s.t >= 1.5).map(|s| s.state[2] * RAD2DEG).collect();
    let lowest = after.iter().copied().fold(f64::MAX, f64::min);
    let end = *after.last().unwrap();
    assert!(lowest > 165.0, "turned the long way: heading reached {lowest:.1}°");
    assert!((end - 190.0).abs() < 0.5, "ended at {end:.2}°, expected 190°");
}
