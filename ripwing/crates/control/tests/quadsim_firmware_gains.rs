//! The gains the firmware flies (`ripwing_control::gains`) are QuadSim's
//! tuned gains, converted exactly as the step-response tests convert them.
//!
//! If this fails after a retune, the message prints the new values to paste
//! into `crates/control/src/gains.rs`.

mod quadsim;

use quadsim::fixture;
use quadsim::harness::{pid_config, torque_per_unit};
use quadsim::plant::{Plant, PlantParams};
use ripwing_control::gains::{LOOP_DT, QUADSIM_7IN, QUADSIM_7IN_ATTITUDE_P};
use ripwing_control::PidConfig;

/// f32 round-trip of a decimal literal is good to ~1e-7 relative.
fn close(a: f32, b: f32) -> bool {
    (a - b).abs() <= 1e-6 * a.abs().max(b.abs()).max(1e-12)
}

fn same(a: &PidConfig, b: &PidConfig) -> bool {
    close(a.kp, b.kp)
        && close(a.ki, b.ki)
        && close(a.kd, b.kd)
        && close(a.integral_limit, b.integral_limit)
        && close(a.output_limit, b.output_limit)
        && close(a.d_filter_alpha, b.d_filter_alpha)
}

#[test]
fn firmware_gains_match_quadsim_tune() {
    let fx = fixture::load();
    // Every closed-loop "_tuned" case carries the full tuned gain set; the
    // yaw case is the one whose yaw gains were exercised.
    let case = fx.case("yaw_tuned");
    let unit = torque_per_unit(&Plant::new(PlantParams::default()));
    let dt = LOOP_DT as f64;

    let expect = [
        ("roll", pid_config(case, "p", unit[0], dt), QUADSIM_7IN.roll),
        ("pitch", pid_config(case, "q", unit[1], dt), QUADSIM_7IN.pitch),
        ("yaw", pid_config(case, "r", unit[2], dt), QUADSIM_7IN.yaw),
    ];
    for (axis, want, have) in &expect {
        assert!(same(want, have), "{axis}: gains.rs has\n  {have:?}\nQuadSim tune converts to\n  {want:?}");
    }

    let k_att = ["k_att_roll", "k_att_pitch", "k_att_yaw"].map(|k| case.get(k) as f32);
    for i in 0..3 {
        assert!(close(k_att[i], QUADSIM_7IN_ATTITUDE_P[i]), "attitude P: gains.rs has {QUADSIM_7IN_ATTITUDE_P:?}, QuadSim tune has {k_att:?}");
    }
}
