//! Host-side validation of the control logic.
//!
//! These build for the host (std available) even though the crate is
//! no_std. The point of this file is the workflow: run the Rust controller
//! against a plant and assert on its behaviour, so a porting bug surfaces
//! in `cargo test` in seconds rather than on the drone.
//!
//! The plant here is a deliberately simple first-order angular-rate model.
//! Replace / augment it with a port of your 10-DOF simulator plant to get a
//! true apples-to-apples check against the Python step responses.

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
