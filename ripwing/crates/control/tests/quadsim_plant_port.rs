//! Layer 1: the plant port on its own.
//!
//! Before trusting any closed-loop comparison, check that the Rust plant IS
//! the Python plant: drive it with the exact command sequence QuadSim logged
//! in the `openloop` case and require agreement to near machine precision.
//! A mismatch here is a porting bug in `quadsim/plant.rs`, never a control
//! problem.
//!
//! Also pins the firmware mixer's signs to the plant's frame.

mod quadsim;

use quadsim::fixture;
use quadsim::harness::{torque_per_unit, FIRMWARE_TO_PLANT};
use quadsim::plant::{Plant, PlantParams};
use ripwing_control::mixer::mix_quad_x;

#[test]
fn plant_params_match_quadsim() {
    let fx = fixture::load();
    let p = PlantParams::default();
    let plant = Plant::new(p);
    let ours = [
        ("mass", p.mass),
        ("g", p.g),
        ("arm_length", p.arm_length),
        ("Ixx", p.ixx),
        ("Iyy", p.iyy),
        ("Izz", p.izz),
        ("k_f", p.k_f),
        ("k_m", p.k_m),
        ("max_omega", p.max_omega),
        ("omega_hover", plant.omega_hover()),
        ("motor_time_constant", p.motor_time_constant),
        ("rotor_rate_damp_coeff", p.rotor_rate_damp_coeff),
        ("rotor_inertia", p.rotor_inertia),
    ];
    assert_eq!(fx.plant.len(), ours.len(), "QuadSim exported parameters this port does not model");
    for (k, v) in ours {
        let py = fx.plant[k];
        assert!(
            (v - py).abs() <= 1e-12 * py.abs(),
            "plant parameter {k}: Rust {v} vs QuadSim {py}"
        );
    }
}

#[test]
fn openloop_trajectory_matches_python() {
    let fx = fixture::load();
    let case = fx.case("openloop");
    let plant = Plant::new(PlantParams::default());
    let dt = case.dt();

    let mut x = plant.hover_state();
    let mut worst = [0.0f64; 3]; // angle [rad], rate [rad/s], rotor [rad/s]
    for row in &case.rows {
        let py: [f64; 10] = [
            row.euler[0], row.euler[1], row.euler[2],
            row.rates[0], row.rates[1], row.rates[2],
            row.rotor[0], row.rotor[1], row.rotor[2], row.rotor[3],
        ];
        for i in 0..10 {
            let g = if i < 3 { 0 } else if i < 6 { 1 } else { 2 };
            worst[g] = worst[g].max((x[i] - py[i]).abs());
        }
        x = plant.step_rk4(&x, &row.cmd, dt);
    }

    // The trajectory reaches tens of degrees, several rad/s and full rotor
    // saturation, so these bounds leave room only for float summation-order
    // differences between NumPy and Rust.
    println!("openloop worst |Rust - Python|: angle {:.2e} rad, rate {:.2e} rad/s, rotor {:.2e} rad/s",
        worst[0], worst[1], worst[2]);
    assert!(worst[0] < 1e-9, "angle mismatch {:e} rad", worst[0]);
    assert!(worst[1] < 1e-9, "rate mismatch {:e} rad/s", worst[1]);
    assert!(worst[2] < 1e-7, "rotor mismatch {:e} rad/s", worst[2]);

    // The scripted sequence must actually exercise the nonlinear regime,
    // otherwise the tight match proves little.
    let peak = |f: fn(&fixture::Row) -> f64| case.rows.iter().map(f).fold(0.0f64, |a, v| a.max(v.abs()));
    assert!(peak(|r| r.euler[0]) > 0.2 && peak(|r| r.euler[1]) > 0.2 && peak(|r| r.euler[2]) > 0.2);
    assert!(case.rows.iter().any(|r| r.rotor.iter().any(|&w| w > 0.99 * plant.p.max_omega)));
}

#[test]
fn hover_is_an_equilibrium() {
    let plant = Plant::new(PlantParams::default());
    let x = plant.hover_state();
    let w = plant.omega_hover();
    let d = plant.derivatives(&x, &[w; 4]);
    assert!(d.iter().all(|v| v.abs() < 1e-12), "{d:?}");
}

/// Positive mixer demand on each axis must produce positive steady torque
/// about that axis on the plant (FLU frame), and nothing on the others.
/// "Steady" = rotors already at the commanded speed, so the motor-lag
/// reaction term is zero and only the thrust/drag geometry is measured.
#[test]
fn mixer_signs_match_plant_frame() {
    let plant = Plant::new(PlantParams::default());
    let unit = torque_per_unit(&plant);
    let inertia = [plant.p.ixx, plant.p.iyy, plant.p.izz];
    let hover = (plant.omega_hover() / plant.p.max_omega).powi(2) as f32;
    let u = 0.02f32;

    for axis in 0..3 {
        let mut demand = [0.0f32; 3];
        demand[axis] = u;
        let m = mix_quad_x(hover, demand[0], demand[1], demand[2]);
        let mut x = plant.hover_state();
        let mut w = [0.0; 4];
        for (fw, &pl) in FIRMWARE_TO_PLANT.iter().enumerate() {
            w[pl] = plant.p.max_omega * (m.throttle[fw] as f64).sqrt();
            x[6 + pl] = w[pl];
        }
        let d = plant.derivatives(&x, &w);
        for j in 0..3 {
            let torque = d[3 + j] * inertia[j];
            if j == axis {
                // Exact linear scaling, not just the right sign.
                let want = u as f64 * unit[axis];
                assert!(
                    (torque - want).abs() < 1e-3 * want,
                    "axis {axis}: torque {torque} N·m, expected {want}"
                );
            } else {
                assert!(torque.abs() < 1e-9, "axis {axis} leaks into axis {j}: {torque}");
            }
        }
    }
}
