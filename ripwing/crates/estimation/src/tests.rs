//! Host-side validation of the attitude estimators.
//!
//! What matters is behaviour: does it start at the right attitude, does it
//! track a real rotation, does it reject drift, does it refuse to be fooled
//! by an accelerometer that is not measuring gravity.

use super::*;
use approx::assert_abs_diff_eq;
use core::f32::consts::FRAC_PI_2;
use ripwing_common::ImuSample;

const DT: f32 = 0.001; // 1 kHz
const DEG: f32 = core::f32::consts::PI / 180.0;

/// Specific force the accelerometer reads at rest at a given roll/pitch.
fn accel_at(roll: f32, pitch: f32) -> [f32; 3] {
    let up = Quat::from_euler(roll, pitch, 0.0).up_in_body();
    [up[0] * GRAVITY, up[1] * GRAVITY, up[2] * GRAVITY]
}

fn sample(gyro: [f32; 3], accel: [f32; 3]) -> ImuSample {
    ImuSample {
        gyro,
        accel,
        timestamp_us: 0,
    }
}

fn level() -> ImuSample {
    sample([0.0; 3], [0.0, 0.0, GRAVITY])
}

fn cf() -> ComplementaryFilter {
    ComplementaryFilter::new(ComplementaryConfig::default())
}

// ---- conventions ----------------------------------------------------------

#[test]
fn quat_euler_round_trip() {
    for &(r, p, y) in &[
        (0.0, 0.0, 0.0),
        (0.3, -0.2, 1.0),
        (-1.2, 0.7, -2.5),
        (0.05, 1.3, 0.4),
    ] {
        let e = Quat::from_euler(r, p, y).to_euler();
        assert_abs_diff_eq!(e[0], r, epsilon = 1e-5);
        assert_abs_diff_eq!(e[1], p, epsilon = 1e-5);
        assert_abs_diff_eq!(e[2], y, epsilon = 1e-5);
    }
}

#[test]
fn accel_roll_pitch_inverts_accel_at() {
    // The accel model used throughout these tests and the closed-form
    // roll/pitch must agree, or every other test is checking the wrong thing.
    for &(r, p) in &[(0.0, 0.0), (0.4, 0.0), (0.0, -0.3), (-0.6, 0.5)] {
        let (er, ep) = accel_roll_pitch(accel_at(r, p));
        assert_abs_diff_eq!(er, r, epsilon = 1e-5);
        assert_abs_diff_eq!(ep, p, epsilon = 1e-5);
    }
}

#[test]
fn positive_roll_rate_produces_positive_roll() {
    // Sign convention: the angle moves the same way as the gyro rate.
    let mut q = Quat::IDENTITY;
    for _ in 0..100 {
        q.integrate([1.0, 0.0, 0.0], DT);
    }
    assert_abs_diff_eq!(q.to_euler()[0], 0.1, epsilon = 1e-4);
}

// ---- initialization -------------------------------------------------------

#[test]
fn first_sample_seeds_attitude_from_accel() {
    // Sitting tilted on the bench, the very first estimate must say so. A
    // filter that started at level would report "level" here — the arming
    // precondition — while the vehicle is actually tilted.
    let mut f = cf();
    let s = f.update(&sample([0.0; 3], accel_at(0.3, -0.2)), DT);
    assert!(f.is_initialized());
    assert_abs_diff_eq!(s.attitude[0], 0.3, epsilon = 1e-4);
    assert_abs_diff_eq!(s.attitude[1], -0.2, epsilon = 1e-4);
}

#[test]
fn does_not_initialize_on_an_implausible_accel() {
    // A 3 g reading is not a gravity reference; seeding from it would be a
    // guess. The filter waits, and the caller withholds publication.
    let mut f = cf();
    f.update(&sample([0.0; 3], [0.0, 0.0, 3.0 * GRAVITY]), DT);
    assert!(!f.is_initialized());
    f.update(&level(), DT);
    assert!(f.is_initialized());
}

#[test]
fn reset_requires_reinitialization() {
    let mut f = cf();
    f.update(&level(), DT);
    f.reset();
    assert!(!f.is_initialized());
    let s = f.update(&sample([0.0; 3], accel_at(-0.4, 0.1)), DT);
    assert_abs_diff_eq!(s.attitude[0], -0.4, epsilon = 1e-4);
}

// ---- steady state ---------------------------------------------------------

#[test]
fn stays_level_when_still() {
    let mut f = cf();
    let mut s = f.update(&level(), DT);
    for _ in 0..10_000 {
        s = f.update(&level(), DT);
    }
    for a in s.attitude {
        assert_abs_diff_eq!(a, 0.0, epsilon = 1e-5);
    }
}

#[test]
fn passes_body_rates_and_timestamp_through() {
    let mut f = cf();
    f.update(&level(), DT);
    let mut smp = sample([0.1, -0.2, 0.3], [0.0, 0.0, GRAVITY]);
    smp.timestamp_us = 4242;
    let s = f.update(&smp, DT);
    assert_eq!(s.body_rates, [0.1, -0.2, 0.3]);
    assert_eq!(s.timestamp_us, 4242);
}

// ---- tracking -------------------------------------------------------------

#[test]
fn tracks_a_rotation_with_consistent_gyro_and_accel() {
    // Roll at 1 rad/s for 0.5 s with a truthful accelerometer: the estimate
    // must follow the true roll closely throughout, not lag behind by τ.
    let mut f = cf();
    f.update(&level(), DT);
    let rate = 1.0;
    let mut truth = 0.0;
    let mut est = [0.0; 3];
    for _ in 0..500 {
        truth += rate * DT;
        est = f
            .update(&sample([rate, 0.0, 0.0], accel_at(truth, 0.0)), DT)
            .attitude;
    }
    assert_abs_diff_eq!(est[0], 0.5, epsilon = 2e-3);
    assert_abs_diff_eq!(est[1], 0.0, epsilon = 1e-3);
}

#[test]
fn uses_real_dt_not_a_nominal_one() {
    // A dropped sample means this one covers two periods. Integrating with
    // the true interval keeps the angle right.
    let mut a = cf();
    let mut b = cf();
    a.update(&level(), DT);
    b.update(&level(), DT);
    // Gyro-only comparison: make accel unusable so only integration counts.
    let spin = sample([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    for _ in 0..100 {
        a.update(&spin, DT);
    }
    for _ in 0..50 {
        b.update(&spin, 2.0 * DT);
    }
    let ea = a.update(&spin, 0.0).attitude[0];
    let eb = b.update(&spin, 0.0).attitude[0];
    assert_abs_diff_eq!(ea, eb, epsilon = 1e-4);
    assert_abs_diff_eq!(ea, 0.1, epsilon = 1e-4);
}

// ---- drift and correction -------------------------------------------------

#[test]
fn accel_corrects_a_wrong_estimate_with_time_constant_one_over_kp() {
    // Seed at level, then hold the vehicle still at 10° roll. The estimate
    // should converge first-order with τ = 1/kp: after one τ the remaining
    // error is ~e^-1 of the initial error, after 5τ under 1%.
    let cfg = ComplementaryConfig::default();
    let tau = 1.0 / cfg.kp;
    let mut f = ComplementaryFilter::new(cfg);
    f.update(&level(), DT);

    let tilted = sample([0.0; 3], accel_at(10.0 * DEG, 0.0));
    let steps_per_tau = (tau / DT) as usize;
    let mut roll = 0.0;
    for _ in 0..steps_per_tau {
        roll = f.update(&tilted, DT).attitude[0];
    }
    let remaining = (10.0 * DEG - roll) / (10.0 * DEG);
    assert_abs_diff_eq!(remaining, (-1.0f32).exp(), epsilon = 0.03);

    for _ in 0..4 * steps_per_tau {
        roll = f.update(&tilted, DT).attitude[0];
    }
    assert_abs_diff_eq!(roll, 10.0 * DEG, epsilon = 0.01 * 10.0 * DEG);
}

#[test]
fn gyro_bias_error_is_bounded_by_bias_times_tau() {
    // Pure integration would drift without bound; the accel correction
    // holds tilt error near bias·τ.
    let cfg = ComplementaryConfig::default();
    let bias = 0.5 * DEG; // 0.5 °/s, typical MEMS
    let mut f = ComplementaryFilter::new(cfg);
    f.update(&level(), DT);
    let mut roll = 0.0;
    for _ in 0..30_000 {
        // 30 s
        roll = f
            .update(&sample([bias, 0.0, 0.0], [0.0, 0.0, GRAVITY]), DT)
            .attitude[0];
    }
    assert_abs_diff_eq!(roll, bias / cfg.kp, epsilon = 0.1 * bias / cfg.kp);
}

#[test]
fn rejects_accel_that_is_not_gravity() {
    // 2 g pointing sideways: the vehicle is accelerating, the accel is not a
    // tilt reference. The filter must coast on the gyro, not lurch.
    let mut f = cf();
    f.update(&level(), DT);
    let mut s = f.update(&level(), DT);
    for _ in 0..1000 {
        s = f.update(&sample([0.0; 3], [0.0, 2.0 * GRAVITY, 0.0]), DT);
        assert!(!f.accel_used());
    }
    for a in s.attitude {
        assert_abs_diff_eq!(a, 0.0, epsilon = 1e-6);
    }
}

#[test]
fn zero_mean_vibration_does_not_bias_attitude() {
    // Large symmetric vibration on the accel (within the gate) averages out
    // through the 1/kp low-pass.
    let mut f = cf();
    f.update(&level(), DT);
    let mut s = f.update(&level(), DT);
    for i in 0..20_000 {
        let v = if i % 2 == 0 { 1.0 } else { -1.0 };
        s = f.update(&sample([0.0; 3], [v, -v, GRAVITY]), DT);
    }
    assert_abs_diff_eq!(s.attitude[0], 0.0, epsilon = 1e-3);
    assert_abs_diff_eq!(s.attitude[1], 0.0, epsilon = 1e-3);
}

// ---- robustness -----------------------------------------------------------

#[test]
fn bad_dt_and_long_gaps_are_contained() {
    let mut f = cf();
    f.update(&level(), DT);
    let spin = sample([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]); // accel gated out
    // Non-finite and negative dt: no integration.
    let s = f.update(&spin, f32::NAN);
    assert_abs_diff_eq!(s.attitude[0], 0.0, epsilon = 1e-6);
    let s = f.update(&spin, -1.0);
    assert_abs_diff_eq!(s.attitude[0], 0.0, epsilon = 1e-6);
    // A 1 s gap is clamped to max_dt, not integrated as a 1 rad jump.
    let s = f.update(&spin, 1.0);
    assert_abs_diff_eq!(
        s.attitude[0],
        ComplementaryConfig::default().max_dt,
        epsilon = 1e-5
    );
}

#[test]
fn non_finite_gyro_does_not_poison_the_state() {
    let mut f = cf();
    f.update(&level(), DT);
    f.update(&sample([f32::NAN, 0.0, 0.0], [0.0, 0.0, GRAVITY]), DT);
    let s = f.update(&level(), DT);
    for a in s.attitude {
        assert!(a.is_finite());
    }
    assert_abs_diff_eq!(f.quaternion().norm(), 1.0, epsilon = 1e-5);
}

#[test]
fn quaternion_stays_unit_length_under_sustained_rotation() {
    let mut f = cf();
    f.update(&level(), DT);
    for _ in 0..60_000 {
        f.update(&sample([3.0, -2.0, 5.0], [0.0, 0.0, 0.0]), DT);
    }
    assert_abs_diff_eq!(f.quaternion().norm(), 1.0, epsilon = 1e-5);
}

#[test]
fn pitch_near_vertical_stays_finite() {
    // The asin clamp: rounding must not produce NaN at the singularity.
    let e = Quat::from_euler(0.0, FRAC_PI_2, 0.0).to_euler();
    assert!(e.iter().all(|a| a.is_finite()));
}
