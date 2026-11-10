//! Host-side validation of the control logic.
//!
//! These build for the host (std available) even though the crate is
//! no_std. The point of this file is the workflow: run the Rust controller
//! against a plant and assert on its behaviour, so a porting bug surfaces
//! in `cargo test` in seconds rather than on the drone.
//!
//! The plant here is a deliberately simple first-order angular-rate model,
//! for unit-level PID behaviour. The apples-to-apples check against the
//! Python simulator — a port of QuadSim's 10-DOF rotational dynamics and
//! its step responses — lives in `tests/quadsim_plant_port.rs` and
//! `tests/quadsim_step_response.rs`.

use crate::pid::{Pid, PidConfig};

/// First-order angular-rate plant: a single axis whose rate responds to
/// torque with time constant `tau`. rate' = (k*u - rate) / tau.
/// Crude, but enough to exercise convergence and anti-windup.
struct RatePlant {
    rate: f32,
    tau: f32,
    k: f32,
}

impl RatePlant {
    fn new(tau: f32, k: f32) -> Self {
        Self { rate: 0.0, tau, k }
    }

    /// Advance the plant one step under control input `u`.
    fn step(&mut self, u: f32, dt: f32) {
        let rate_dot = (self.k * u - self.rate) / self.tau;
        self.rate += rate_dot * dt;
    }
}

/// Run a closed loop for `n` steps and return the final measured rate.
fn simulate(mut pid: Pid, mut plant: RatePlant, setpoint: f32, dt: f32, n: usize) -> f32 {
    let mut measurement = plant.rate;
    for _ in 0..n {
        let u = pid.update(setpoint, measurement, dt);
        plant.step(u, dt);
        measurement = plant.rate;
    }
    measurement
}

fn test_gains() -> PidConfig {
    // Placeholder gains. Replace with your optimizer's actual values, then
    // assert the response matches your Python results.
    PidConfig {
        kp: 2.0,
        ki: 4.0,
        kd: 0.05,
        integral_limit: 5.0,
        output_limit: 1.0,
        d_filter_alpha: 0.2,
    }
}

#[test]
fn converges_to_setpoint() {
    // With integral action, steady-state error should vanish.
    let pid = Pid::new(test_gains());
    let plant = RatePlant::new(0.05, 1.0);
    let setpoint = 0.5;
    let dt = 0.001;

    let final_rate = simulate(pid, plant, setpoint, dt, 5000);
    approx::assert_abs_diff_eq!(final_rate, setpoint, epsilon = 0.02);
}

#[test]
fn integral_respects_windup_clamp() {
    // Drive a large, persistent error so the integral wants to run away,
    // then confirm it never exceeds the configured clamp.
    let mut pid = Pid::new(test_gains());
    let dt = 0.001;

    // Feed a constant large error for many steps.
    for _ in 0..10_000 {
        let _ = pid.update(10.0, 0.0, dt);
    }

    assert!(
        pid.state_integral().abs() <= test_gains().integral_limit + 1e-6,
        "integral {} exceeded clamp {}",
        pid.state_integral(),
        test_gains().integral_limit
    );
}

#[test]
fn no_derivative_kick_on_setpoint_step() {
    // Derivative-on-measurement should NOT produce a spike when the
    // setpoint steps (measurement hasn't moved yet). Compare the first
    // output after a setpoint jump against a pure-P expectation: the D
    // contribution should be ~0 on that first step.
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        ki: 0.0,
        kd: 10.0, // large D to make any kick obvious
        integral_limit: f32::INFINITY,
        output_limit: f32::INFINITY,
        d_filter_alpha: 1.0,
        ..Default::default()
    });
    let dt = 0.001;

    // First call establishes the measurement baseline (measurement = 0).
    let _ = pid.update(0.0, 0.0, dt);
    // Now step the setpoint. Measurement is still 0, so derivative-on-
    // measurement is 0 -> output should be just kp*error = 1.0*1.0 = 1.0,
    // with no huge D spike.
    let out = pid.update(1.0, 0.0, dt);

    approx::assert_abs_diff_eq!(out, 1.0, epsilon = 1e-3);
}

#[test]
fn reset_clears_state() {
    let mut pid = Pid::new(test_gains());
    let dt = 0.001;
    for _ in 0..1000 {
        let _ = pid.update(1.0, 0.0, dt);
    }
    assert!(pid.state_integral().abs() > 0.0);
    pid.reset();
    approx::assert_abs_diff_eq!(pid.state_integral(), 0.0, epsilon = 1e-9);
}

// ---------------------------------------------------------------------------
// Outer attitude loop (angle.rs)
// ---------------------------------------------------------------------------

mod angle_loop {
    use crate::angle::{wrap_pi, AngleLoopConfig, AttitudeAngleController, MIN_TILT_COS};
    use crate::controller::AttitudeLoop;
    use approx::assert_abs_diff_eq;
    use core::f32::consts::PI;
    use ripwing_common::{AttitudeSetpoint, StateEstimate};

    fn ctrl(tilt_compensation: bool) -> AttitudeAngleController {
        AttitudeAngleController::new(AngleLoopConfig {
            k_att: [8.0, 7.0, 3.0],
            max_rate: [6.0, 6.0, 4.0],
            tilt_compensation,
        })
    }

    fn at(attitude: [f32; 3]) -> StateEstimate {
        StateEstimate { attitude, ..Default::default() }
    }

    fn sp(angles: [f32; 3], thrust: f32) -> AttitudeSetpoint {
        AttitudeSetpoint { angles, thrust }
    }

    #[test]
    fn proportional_on_each_axis() {
        let out = ctrl(false).step(&at([0.1, -0.05, 0.2]), &sp([0.2, 0.05, 0.5], 0.4), 0.001);
        assert_abs_diff_eq!(out.rates[0], 8.0 * 0.1, epsilon = 1e-6);
        assert_abs_diff_eq!(out.rates[1], 7.0 * 0.1, epsilon = 1e-6);
        assert_abs_diff_eq!(out.rates[2], 3.0 * 0.3, epsilon = 1e-6);
        assert_eq!(out.thrust, 0.4);
    }

    #[test]
    fn rate_demand_is_clamped_per_axis() {
        let out = ctrl(false).step(&at([0.0; 3]), &sp([1.0, -1.0, 2.0], 0.5), 0.001);
        assert_eq!(out.rates, [6.0, -6.0, 4.0]);
    }

    #[test]
    fn heading_error_takes_the_short_way() {
        // 170° -> -170° is +20° through 180°, not -340°.
        let out = ctrl(false).step(&at([0.0, 0.0, 170f32.to_radians()]), &sp([0.0, 0.0, -170f32.to_radians()], 0.5), 0.001);
        assert_abs_diff_eq!(out.rates[2], 3.0 * 20f32.to_radians(), epsilon = 1e-5);
        // And the mirror image.
        let out = ctrl(false).step(&at([0.0, 0.0, -170f32.to_radians()]), &sp([0.0, 0.0, 170f32.to_radians()], 0.5), 0.001);
        assert_abs_diff_eq!(out.rates[2], -3.0 * 20f32.to_radians(), epsilon = 1e-5);
    }

    #[test]
    fn wrap_pi_matches_quadsim() {
        // QuadSim: (x + π) mod 2π − π, i.e. [-π, π).
        assert_abs_diff_eq!(wrap_pi(0.0), 0.0);
        assert_abs_diff_eq!(wrap_pi(PI), -PI, epsilon = 1e-6);
        assert_abs_diff_eq!(wrap_pi(-PI), -PI, epsilon = 1e-6);
        assert_abs_diff_eq!(wrap_pi(3.0 * PI / 2.0), -PI / 2.0, epsilon = 1e-6);
        assert_abs_diff_eq!(wrap_pi(-3.0 * PI / 2.0), PI / 2.0, epsilon = 1e-6);
        assert_abs_diff_eq!(wrap_pi(7.0 * PI + 0.25), -PI + 0.25, epsilon = 1e-5);
        assert!(wrap_pi(f32::NAN).is_nan());
    }

    #[test]
    fn tilt_compensation_keeps_vertical_thrust() {
        let (r, p) = (0.3f32, -0.2f32);
        let out = ctrl(true).step(&at([r, p, 0.0]), &sp([r, p, 0.0], 0.4), 0.001);
        assert_abs_diff_eq!(out.thrust * r.cos() * p.cos(), 0.4, epsilon = 1e-6);
        // Off: passthrough.
        let out = ctrl(false).step(&at([r, p, 0.0]), &sp([r, p, 0.0], 0.4), 0.001);
        assert_eq!(out.thrust, 0.4);
    }

    #[test]
    fn tilt_compensation_is_floored_and_bounded() {
        // 80° of bank: cos = 0.17, floored to 0.5, so at most 2x.
        let out = ctrl(true).step(&at([80f32.to_radians(), 0.0, 0.0]), &sp([0.0; 3], 0.3), 0.001);
        assert_abs_diff_eq!(out.thrust, 0.3 / MIN_TILT_COS, epsilon = 1e-6);
        // Never past full throttle.
        let out = ctrl(true).step(&at([0.5, 0.5, 0.0]), &sp([0.0; 3], 0.9), 0.001);
        assert_eq!(out.thrust, 1.0);
    }
}
