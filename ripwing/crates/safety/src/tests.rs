//! Host-side validation of the failsafe state machine and checks.
//!
//! These tests are the reason the logic lives in a pure crate: every safety
//! decision is exercised deterministically, including the adversarial edges
//! (timestamp in the future, exactly-at-limit, fault clearing but latch
//! holding) that are painful or impossible to reproduce on hardware.

use super::*;
use ripwing_common::StateEstimate;

// ---- Helpers ------------------------------------------------------------

fn limits() -> SafetyLimits {
    SafetyLimits::default()
}

/// A "healthy, level, still" input at time `now`, with fresh state and RC.
fn healthy_inputs(now_us: u32) -> SafetyInputs {
    SafetyInputs {
        now_us,
        state: StateEstimate {
            attitude: [0.0, 0.0, 0.0],
            body_rates: [0.0, 0.0, 0.0],
            timestamp_us: now_us, // perfectly fresh
        },
        last_rc_heartbeat_us: now_us, // fresh link
        battery_volts: 12.0,          // healthy 3S
    }
}

/// Arm a monitor from a healthy state, asserting it worked.
fn armed_monitor(now_us: u32) -> SafetyMonitor {
    let mut m = SafetyMonitor::new(limits());
    let s = m.try_arm(&healthy_inputs(now_us));
    assert_eq!(s, ArmState::Armed, "precondition: should arm from healthy");
    m
}

// ---- Default state ------------------------------------------------------

#[test]
fn powers_on_disarmed_and_motors_cut() {
    let mut m = SafetyMonitor::new(limits());
    assert_eq!(m.arm_state(), ArmState::Disarmed);
    // Even with perfectly healthy inputs, disarmed means motors cut.
    assert_eq!(m.update(&healthy_inputs(1_000)), MotorPermission::Cut);
}

// ---- Arming preconditions ----------------------------------------------

#[test]
fn arms_from_healthy_level_still_state() {
    let mut m = SafetyMonitor::new(limits());
    assert_eq!(m.try_arm(&healthy_inputs(1_000)), ArmState::Armed);
}

#[test]
fn refuses_to_arm_while_tilted() {
    let mut m = SafetyMonitor::new(limits());
    let mut inp = healthy_inputs(1_000);
    inp.state.attitude[0] = 0.5; // ~29°, well past the 5° arm limit
    assert_eq!(m.try_arm(&inp), ArmState::Disarmed);
}

#[test]
fn refuses_to_arm_while_rotating() {
    let mut m = SafetyMonitor::new(limits());
    let mut inp = healthy_inputs(1_000);
    inp.state.body_rates[2] = 1.0; // ~57°/s yaw, past the arm rate limit
    assert_eq!(m.try_arm(&inp), ArmState::Disarmed);
}

#[test]
fn refuses_to_arm_with_stale_state() {
    let mut m = SafetyMonitor::new(limits());
    let mut inp = healthy_inputs(100_000);
    inp.state.timestamp_us = 100_000 - 10_000; // 10 ms old > 5 ms limit
    assert_eq!(m.try_arm(&inp), ArmState::Disarmed);
    assert_eq!(m.last_fault(), Some(SafetyFault::StaleState));
}

#[test]
fn refuses_to_arm_on_low_battery() {
    let mut m = SafetyMonitor::new(limits());
    let mut inp = healthy_inputs(1_000);
    inp.battery_volts = 9.0; // below 10.5 floor
    assert_eq!(m.try_arm(&inp), ArmState::Disarmed);
    assert_eq!(m.last_fault(), Some(SafetyFault::LowBattery));
}

#[test]
fn cannot_arm_from_failsafe_without_disarm() {
    // Drive into failsafe, then a healthy arm attempt should be refused
    // because we are not Disarmed. Only an explicit disarm clears the latch.
    let mut m = armed_monitor(1_000);
    let mut bad = healthy_inputs(2_000);
    bad.battery_volts = 8.0;
    assert_eq!(m.update(&bad), MotorPermission::Cut);
    assert_eq!(m.arm_state(), ArmState::Failsafe);

    // Healthy arm attempt from Failsafe: refused.
    assert_eq!(m.try_arm(&healthy_inputs(3_000)), ArmState::Failsafe);

    // Explicit disarm, then arm: allowed.
    m.disarm();
    assert_eq!(m.arm_state(), ArmState::Disarmed);
    assert_eq!(m.try_arm(&healthy_inputs(4_000)), ArmState::Armed);
}

// ---- In-flight checks trip failsafe ------------------------------------

#[test]
fn armed_healthy_runs() {
    let mut m = armed_monitor(1_000);
    assert_eq!(m.update(&healthy_inputs(2_000)), MotorPermission::Run);
    assert_eq!(m.arm_state(), ArmState::Armed);
}

#[test]
fn stale_state_trips_failsafe() {
    let mut m = armed_monitor(1_000);
    let mut inp = healthy_inputs(100_000);
    inp.state.timestamp_us = 100_000 - 6_000; // 6 ms old > 5 ms limit
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.arm_state(), ArmState::Failsafe);
    assert_eq!(m.last_fault(), Some(SafetyFault::StaleState));
}

#[test]
fn excessive_tilt_trips_failsafe() {
    let mut m = armed_monitor(1_000);
    let mut inp = healthy_inputs(2_000);
    inp.state.attitude[1] = 1.5; // ~86° pitch > 80° limit
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.last_fault(), Some(SafetyFault::AttitudeLimit));
}

#[test]
fn excessive_rate_trips_failsafe() {
    let mut m = armed_monitor(1_000);
    let mut inp = healthy_inputs(2_000);
    inp.state.body_rates[0] = 20.0; // > ~17.45 limit
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.last_fault(), Some(SafetyFault::RateLimit));
}

#[test]
fn rc_link_loss_trips_failsafe() {
    let mut m = armed_monitor(1_000);
    let mut inp = healthy_inputs(1_000_000);
    inp.last_rc_heartbeat_us = 1_000_000 - 600_000; // 600 ms silence > 500 ms
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.last_fault(), Some(SafetyFault::RcLinkLoss));
}

#[test]
fn low_battery_trips_failsafe() {
    let mut m = armed_monitor(1_000);
    let mut inp = healthy_inputs(2_000);
    inp.battery_volts = 10.0; // < 10.5 floor
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.last_fault(), Some(SafetyFault::LowBattery));
}

// ---- The latch: failsafe holds even after the fault clears --------------

#[test]
fn failsafe_latches_after_fault_clears() {
    // This is the safety-critical property: a fault that momentarily clears
    // must NOT let the vehicle silently re-arm mid-air. Once tripped, we
    // stay cut until a human disarms.
    let mut m = armed_monitor(1_000);

    // Trip on a rate spike.
    let mut spike = healthy_inputs(2_000);
    spike.state.body_rates[0] = 25.0;
    assert_eq!(m.update(&spike), MotorPermission::Cut);
    assert_eq!(m.arm_state(), ArmState::Failsafe);

    // Next tick everything is perfectly healthy again...
    assert_eq!(m.update(&healthy_inputs(3_000)), MotorPermission::Cut);
    // ...and yet we are STILL cut and STILL in failsafe. Latched.
    assert_eq!(m.arm_state(), ArmState::Failsafe);
}

// ---- Adversarial edges --------------------------------------------------

#[test]
fn future_timestamp_does_not_underflow() {
    // A state timestamp *ahead* of now (clock glitch) must read as fresh,
    // not wrap around to a huge age via unsigned underflow.
    let mut m = armed_monitor(10_000);
    let mut inp = healthy_inputs(10_000);
    inp.state.timestamp_us = 20_000; // 10 ms in the "future"
    // saturating_sub -> age 0 -> fresh -> runs.
    assert_eq!(m.update(&inp), MotorPermission::Run);
}

#[test]
fn exactly_at_state_age_limit_is_ok() {
    // Age == limit should pass; only age > limit trips. Boundary check.
    let mut m = armed_monitor(100_000);
    let mut inp = healthy_inputs(100_000);
    inp.state.timestamp_us = 100_000 - limits().max_state_age_us; // exactly at limit
    assert_eq!(m.update(&inp), MotorPermission::Run);
}

#[test]
fn one_micro_past_state_age_limit_trips() {
    let mut m = armed_monitor(100_000);
    let mut inp = healthy_inputs(100_000);
    inp.state.timestamp_us = 100_000 - limits().max_state_age_us - 1; // one µs past
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.last_fault(), Some(SafetyFault::StaleState));
}

#[test]
fn disarm_always_cuts_and_clears_fault() {
    let mut m = armed_monitor(1_000);
    m.disarm();
    assert_eq!(m.arm_state(), ArmState::Disarmed);
    assert_eq!(m.last_fault(), None);
    assert_eq!(m.update(&healthy_inputs(2_000)), MotorPermission::Cut);
}

#[test]
fn check_priority_state_age_reported_first() {
    // When multiple faults are present at once, the first in run_all_checks
    // order (staleness) should be the reported one. Documents the priority.
    let mut m = armed_monitor(100_000);
    let mut inp = healthy_inputs(100_000);
    inp.state.timestamp_us = 100_000 - 50_000; // stale
    inp.battery_volts = 5.0; // also low battery
    inp.state.attitude[0] = 2.0; // also over-tilt
    assert_eq!(m.update(&inp), MotorPermission::Cut);
    assert_eq!(m.last_fault(), Some(SafetyFault::StaleState));
}
