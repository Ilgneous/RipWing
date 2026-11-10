//! Magnetometer and barometer fusion: behaviour on simulated sensors.

use super::*;
use approx::assert_abs_diff_eq;
use ripwing_common::{BaroSample, ImuSample, MagSample};

const DT: f32 = 0.001; // IMU at 1 kHz
const DEG: f32 = core::f32::consts::PI / 180.0;
/// Earth field in the world frame (x north, y west, z up), mid-latitude:
/// 20 µT north, 45 µT down.
const EARTH: [f32; 3] = [20.0, 0.0, -45.0];
const P0: f32 = 101_325.0;

fn conj(q: Quat) -> Quat {
    Quat { w: q.w, x: -q.x, y: -q.y, z: -q.z }
}

/// World-frame vector seen in the body frame at this attitude.
fn to_body(roll: f32, pitch: f32, yaw: f32, v: [f32; 3]) -> [f32; 3] {
    conj(Quat::from_euler(roll, pitch, yaw)).body_to_ref(v)
}

fn imu(roll: f32, pitch: f32, gyro: [f32; 3], t_us: u32) -> ImuSample {
    ImuSample { gyro, accel: to_body(roll, pitch, 0.0, [0.0, 0.0, GRAVITY]), timestamp_us: t_us }
}

fn mag(roll: f32, pitch: f32, yaw: f32, t_us: u32) -> MagSample {
    MagSample { field_ut: to_body(roll, pitch, yaw, EARTH), timestamp_us: t_us }
}

/// Pressure at height `h` above `P0` (inverse of `pressure_altitude_m`).
fn pressure_at(h: f32) -> f32 {
    P0 * libm::powf(1.0 - h / 44_330.0, 5.255)
}

fn baro(h: f32, t_us: u32) -> BaroSample {
    BaroSample { pressure_pa: pressure_at(h), temperature_c: 20.0, timestamp_us: t_us }
}

fn filter() -> ComplementaryFilter {
    ComplementaryFilter::new(ComplementaryConfig::default())
}

fn yaw_deg(e: &StateEstimate) -> f32 {
    e.attitude[2] / DEG
}

/// Deterministic noise in [-1, 1].
struct Lcg(u32);
impl Lcg {
    fn next(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.0 >> 8) as f32 / (1u32 << 23) as f32 - 1.0
    }
}

/// Run `secs` of a vehicle holding (roll, pitch, true_yaw) still, with the
/// magnetometer at 100 Hz. `gyro_bias` is added to the yaw rate.
fn hold_with_mag(f: &mut ComplementaryFilter, roll: f32, pitch: f32, true_yaw: f32, secs: f32, gyro_bias: f32, t0: &mut u32) -> StateEstimate {
    let mut e = StateEstimate::default();
    for k in 0..(secs / DT) as u32 {
        *t0 += 1000;
        e = f.update(&imu(roll, pitch, [0.0, 0.0, gyro_bias], *t0), DT);
        if k % 10 == 0 {
            f.update_mag(&mag(roll, pitch, true_yaw, *t0));
        }
    }
    e
}

// ---------------------------------------------------------------------------
// Heading
// ---------------------------------------------------------------------------

#[test]
fn first_mag_reading_snaps_heading() {
    let mut f = filter();
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 0), DT);
    assert!(!e.heading_referenced);
    assert_abs_diff_eq!(yaw_deg(&e), 0.0);
    f.update_mag(&mag(0.0, 0.0, 60.0 * DEG, 0));
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 1000), DT);
    assert!(e.heading_referenced);
    assert_abs_diff_eq!(yaw_deg(&e), 60.0, epsilon = 0.05);
}

#[test]
fn heading_is_tilt_compensated() {
    // 30° roll, −20° pitch: an uncompensated compass would be off by tens
    // of degrees with this much field inclination.
    let (r, p) = (30.0 * DEG, -20.0 * DEG);
    let mut f = filter();
    f.update(&imu(r, p, [0.0; 3], 0), DT);
    f.update_mag(&mag(r, p, 135.0 * DEG, 0));
    let e = f.update(&imu(r, p, [0.0; 3], 1000), DT);
    assert_abs_diff_eq!(yaw_deg(&e), 135.0, epsilon = 0.05);
    assert_abs_diff_eq!(e.attitude[0], r, epsilon = 1e-4);
    assert_abs_diff_eq!(e.attitude[1], p, epsilon = 1e-4);
}

#[test]
fn heading_converges_with_time_constant_one_over_kp() {
    // Heading referenced at 0°, then the magnetometer says 20° while the
    // gyro reports no rotation: the estimate follows with τ = 1/kp = 5 s.
    let mut f = filter();
    let mut t = 0;
    hold_with_mag(&mut f, 0.0, 0.0, 0.0, 0.1, 0.0, &mut t);
    let e = hold_with_mag(&mut f, 0.0, 0.0, 20.0 * DEG, 5.0, 0.0, &mut t);
    let expected = 20.0 * (1.0 - libm::expf(-1.0));
    assert_abs_diff_eq!(yaw_deg(&e), expected, epsilon = 0.5);
}

#[test]
fn magnetometer_never_moves_roll_or_pitch() {
    // A field bent hard (by motor current, say) but of plausible strength.
    let (r, p) = (15.0 * DEG, 10.0 * DEG);
    let mut f = filter();
    let mut t = 0;
    hold_with_mag(&mut f, r, p, 0.0, 0.1, 0.0, &mut t);
    for k in 0..10_000u32 {
        t += 1000;
        let e = f.update(&imu(r, p, [0.0; 3], t), DT);
        if k % 10 == 0 {
            let bent = to_body(r + 0.7, p - 0.5, 1.3, EARTH);
            f.update_mag(&MagSample { field_ut: bent, timestamp_us: t });
        }
        assert_abs_diff_eq!(e.attitude[0], r, epsilon = 1e-3);
        assert_abs_diff_eq!(e.attitude[1], p, epsilon = 1e-3);
    }
}

#[test]
fn gyro_bias_drift_is_bounded_by_bias_times_tau() {
    // 0.5 °/s of yaw-gyro bias for 60 s: 30° of drift without the
    // magnetometer, about bias · τ = 2.5° with it.
    let bias = 0.5 * DEG;
    let mut with = filter();
    let mut t = 0;
    let e = hold_with_mag(&mut with, 0.0, 0.0, 0.0, 60.0, bias, &mut t);
    assert_abs_diff_eq!(yaw_deg(&e), 2.5, epsilon = 0.3);

    let mut without = filter();
    let mut e = StateEstimate::default();
    for k in 0..60_000u32 {
        e = without.update(&imu(0.0, 0.0, [0.0, 0.0, bias], k * 1000), DT);
    }
    assert_abs_diff_eq!(yaw_deg(&e), 30.0, epsilon = 0.5);
    assert!(!e.heading_referenced);
}

#[test]
fn strength_gate_rejects_a_disturbed_field() {
    let mut f = filter();
    let mut t = 0;
    hold_with_mag(&mut f, 0.0, 0.0, 0.0, 0.1, 0.0, &mut t);
    // Twice the strength, pointing 90° away: a magnet next to the board.
    for k in 0..1000u32 {
        t += 1000;
        f.update(&imu(0.0, 0.0, [0.0; 3], t), DT);
        if k % 10 == 0 {
            let m = to_body(0.0, 0.0, 90.0 * DEG, EARTH).map(|x| 2.0 * x);
            f.update_mag(&MagSample { field_ut: m, timestamp_us: t });
        }
    }
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], t), DT);
    assert_abs_diff_eq!(yaw_deg(&e), 0.0, epsilon = 1e-3);
}

#[test]
fn implausible_strength_never_references_heading() {
    let mut f = filter();
    f.update(&imu(0.0, 0.0, [0.0; 3], 0), DT);
    // 5 µT: an uncalibrated sensor or a dead axis, not the Earth.
    f.update_mag(&MagSample { field_ut: [5.0, 0.0, 0.0], timestamp_us: 0 });
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 1000), DT);
    assert!(!e.heading_referenced);
}

#[test]
fn a_wrong_strength_reference_is_replaced() {
    // Referenced at 49 µT (say, on a steel bench); in the air the field is
    // a steady 33 µT pointing at 40°. After 2 s of rejections the new
    // strength is accepted and heading follows.
    let mut f = filter();
    let mut t = 0;
    hold_with_mag(&mut f, 0.0, 0.0, 0.0, 0.1, 0.0, &mut t);
    let weaker = |yaw: f32| to_body(0.0, 0.0, yaw, EARTH).map(|x| x * 0.67);
    let mut e = StateEstimate::default();
    for k in 0..8000u32 {
        t += 1000;
        e = f.update(&imu(0.0, 0.0, [0.0; 3], t), DT);
        if k % 10 == 0 {
            f.update_mag(&MagSample { field_ut: weaker(40.0 * DEG), timestamp_us: t });
        }
        if k == 1500 {
            assert_abs_diff_eq!(yaw_deg(&e), 0.0, epsilon = 1e-3);
        }
    }
    assert!(yaw_deg(&e) > 25.0, "heading did not follow: {}", yaw_deg(&e));
}

#[test]
fn declination_makes_yaw_true() {
    // Nose on magnetic north with 10° east declination: true heading is
    // 10° clockwise of north, i.e. yaw −10°.
    let cfg = ComplementaryConfig {
        mag: MagConfig { declination_rad: 10.0 * DEG, ..Default::default() },
        ..Default::default()
    };
    let mut f = ComplementaryFilter::new(cfg);
    f.update(&imu(0.0, 0.0, [0.0; 3], 0), DT);
    f.update_mag(&mag(0.0, 0.0, 0.0, 0));
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 1000), DT);
    assert_abs_diff_eq!(yaw_deg(&e), -10.0, epsilon = 0.05);
}

#[test]
fn iron_calibration_is_applied() {
    let offset = [12.0, -30.0, 8.0];
    let soft = [[1.1, 0.0, 0.0], [0.0, 0.9, 0.0], [0.0, 0.0, 1.0]];
    let truth = to_body(0.0, 0.0, 70.0 * DEG, EARTH);
    // What the sensor reads: distorted, then offset.
    let raw: [f32; 3] = core::array::from_fn(|i| truth[i] / soft[i][i] + offset[i]);
    let cfg = ComplementaryConfig {
        mag: MagConfig { calibration: MagCalibration { hard_iron_ut: offset, soft_iron: soft }, ..Default::default() },
        ..Default::default()
    };
    let mut f = ComplementaryFilter::new(cfg);
    f.update(&imu(0.0, 0.0, [0.0; 3], 0), DT);
    f.update_mag(&MagSample { field_ut: raw, timestamp_us: 0 });
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 1000), DT);
    assert_abs_diff_eq!(yaw_deg(&e), 70.0, epsilon = 0.05);
}

#[test]
fn heading_correction_takes_the_short_way() {
    // Referenced at 170°, then the field says −170°: 20° through 180°, not
    // 340° back through 0.
    let mut f = filter();
    let mut t = 0;
    hold_with_mag(&mut f, 0.0, 0.0, 170.0 * DEG, 0.1, 0.0, &mut t);
    for k in 0..20_000u32 {
        t += 1000;
        let e = f.update(&imu(0.0, 0.0, [0.0; 3], t), DT);
        if k % 10 == 0 {
            f.update_mag(&mag(0.0, 0.0, -170.0 * DEG, t));
        }
        let y = yaw_deg(&e);
        assert!(y >= 169.9 || y <= -169.9, "went the long way: {y}");
    }
}

#[test]
fn heading_correction_takes_the_short_way_with_declination() {
    // atan2 already returns the field bearing in (−180°, 180°]; adding the
    // declination can push the error past ±180°, which must be wrapped.
    // Referenced at 0°, then the field says the vehicle is at −160°, with
    // −30° declination: turn −160°, not +200°.
    let cfg = ComplementaryConfig {
        mag: MagConfig { declination_rad: -30.0 * DEG, ..Default::default() },
        ..Default::default()
    };
    let mut f = ComplementaryFilter::new(cfg);
    let mut t = 0;
    // The field the vehicle sees at true yaw ψ: magnetic north sits at
    // azimuth −D, so the body sees the Earth field rotated by ψ + D.
    let field = |yaw: f32| to_body(0.0, 0.0, yaw - 30.0 * DEG, EARTH);
    f.update(&imu(0.0, 0.0, [0.0; 3], 0), DT);
    f.update_mag(&MagSample { field_ut: field(0.0), timestamp_us: 0 });
    let mut e = StateEstimate::default();
    for k in 0..3000u32 {
        t += 1000;
        e = f.update(&imu(0.0, 0.0, [0.0; 3], t), DT);
        if k % 10 == 0 {
            f.update_mag(&MagSample { field_ut: field(-160.0 * DEG), timestamp_us: t });
        }
        assert!(yaw_deg(&e) < 0.1, "turning the long way: {}", yaw_deg(&e));
    }
    assert!(yaw_deg(&e) < -60.0, "not converging: {}", yaw_deg(&e));
}

#[test]
fn magnetometer_waits_for_attitude() {
    let mut f = filter();
    f.update_mag(&mag(0.0, 0.0, 60.0 * DEG, 0)); // before any IMU sample
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 1000), DT);
    assert!(!e.heading_referenced);
    assert_abs_diff_eq!(yaw_deg(&e), 0.0);
}

#[test]
fn heading_reference_times_out_without_the_magnetometer() {
    let mut f = filter();
    let mut t = 0;
    let e = hold_with_mag(&mut f, 0.0, 0.0, 0.0, 0.1, 0.0, &mut t);
    assert!(e.heading_referenced);
    let mut e = e;
    for _ in 0..600 {
        t += 1000;
        e = f.update(&imu(0.0, 0.0, [0.0; 3], t), DT);
    }
    assert!(!e.heading_referenced, "still referenced 0.6 s after the last reading");
}

// ---------------------------------------------------------------------------
// Altitude
// ---------------------------------------------------------------------------

/// Run the vertical channel: IMU at 1 kHz with world-frame vertical
/// acceleration `a(t)` (vehicle level), barometer at 50 Hz reporting
/// `h(t) + noise(t)`. Calls `check` on every estimate.
fn fly(f: &mut ComplementaryFilter, secs: f32, a: impl Fn(f32) -> f32, h: impl Fn(f32) -> f32, accel_bias: f32, mut noise: impl FnMut() -> f32, mut check: impl FnMut(f32, &StateEstimate)) {
    for k in 0..(secs / DT) as u32 {
        let t = k as f32 * DT;
        let t_us = k * 1000;
        let s = ImuSample { gyro: [0.0; 3], accel: [0.0, 0.0, GRAVITY + a(t) + accel_bias], timestamp_us: t_us };
        let e = f.update(&s, DT);
        if k % 20 == 0 {
            f.update_baro(&baro(h(t) + noise(), t_us));
        }
        check(t, &e);
    }
}

#[test]
fn vertical_waits_for_a_ground_reference() {
    let mut f = filter();
    let mut first = None;
    fly(&mut f, 1.0, |_| 0.0, |_| 0.0, 0.0, || 0.0, |t, e| {
        if first.is_none() && e.vertical.is_some() {
            first = Some(t);
        }
    });
    // 20 readings at 50 Hz: the reference exists after ~0.38 s, not before.
    let t = first.expect("never produced a vertical estimate");
    assert!((0.37..0.42).contains(&t), "reference at {t} s");
    let v = f.update(&imu(0.0, 0.0, [0.0; 3], 1_000_000), DT).vertical.unwrap();
    assert_abs_diff_eq!(v.altitude_m, 0.0, epsilon = 1e-3);
}

#[test]
fn tracks_a_climb_and_descent() {
    // h = 5 (1 − cos ωt): up to 10 m and back every 8 s, with the matching
    // acceleration. Baro and accel agree; the estimate must follow both.
    let w = 2.0 * core::f32::consts::PI / 8.0;
    let h = move |t: f32| if t < 1.0 { 0.0 } else { 5.0 * (1.0 - libm::cosf(w * (t - 1.0))) };
    let a = move |t: f32| if t < 1.0 { 0.0 } else { 5.0 * w * w * libm::cosf(w * (t - 1.0)) };
    let v = move |t: f32| if t < 1.0 { 0.0 } else { 5.0 * w * libm::sinf(w * (t - 1.0)) };
    let mut f = filter();
    let (mut worst_h, mut worst_v) = (0.0f32, 0.0f32);
    fly(&mut f, 20.0, a, h, 0.0, || 0.0, |t, e| {
        if t > 1.0 {
            let s = e.vertical.unwrap();
            worst_h = worst_h.max((s.altitude_m - h(t)).abs());
            worst_v = worst_v.max((s.climb_rate_mps - v(t)).abs());
        }
    });
    assert!(worst_h < 0.05, "height error {worst_h} m");
    assert!(worst_v < 0.05, "climb-rate error {worst_v} m/s");
}

#[test]
fn accelerometer_bias_is_learned_not_integrated() {
    // 0.2 m/s² of accel bias, vehicle still. Double-integrated alone that
    // is 90 m after 30 s.
    let mut f = filter();
    let mut last = StateEstimate::default();
    fly(&mut f, 30.0, |_| 0.0, |_| 0.0, 0.2, || 0.0, |_, e| last = *e);
    let v = last.vertical.unwrap();
    assert!(v.altitude_m.abs() < 0.05, "height drifted to {} m", v.altitude_m);
    assert!(v.climb_rate_mps.abs() < 0.02);
    assert_abs_diff_eq!(f.vertical().bias(), 0.2, epsilon = 0.01);
}

#[test]
fn barometer_noise_is_smoothed() {
    // ±0.5 m uniform noise (σ ≈ 0.29 m), vehicle still.
    let mut f = filter();
    let mut rng = Lcg(7);
    let (mut sum2, mut n) = (0.0f32, 0u32);
    fly(&mut f, 20.0, |_| 0.0, |_| 0.0, 0.0, || 0.5 * rng.next(), |t, e| {
        if t > 5.0 {
            let h = e.vertical.unwrap().altitude_m;
            sum2 += h * h;
            n += 1;
        }
    });
    let rms = libm::sqrtf(sum2 / n as f32);
    assert!(rms < 0.1, "height noise {rms} m (baro σ ≈ 0.29 m)");
}

#[test]
fn tilted_acceleration_is_rotated_into_the_world_frame() {
    // Banked 25° and accelerating sideways while holding height: the body
    // z axis reads more than 1 g, but none of it is vertical.
    let roll = 25.0 * DEG;
    let f_world = [0.0, GRAVITY * libm::tanf(roll), GRAVITY];
    let f_body = to_body(roll, 0.0, 0.0, f_world);
    // The turn reads 1.10 g along body z. Under the default 15 % gravity
    // gate the attitude filter would take that as "down" and level itself
    // (the documented sustained-acceleration limit), which is not what this
    // test is about. A 5 % gate rejects it, so attitude stays at the bank.
    let cfg = ComplementaryConfig { accel_gate: 0.05, ..Default::default() };
    let mut f = ComplementaryFilter::new(cfg);
    let mut last = StateEstimate::default();
    for k in 0..10_000u32 {
        // Attitude from a consistent gravity reading first, then the bank.
        let s = if k < 10 {
            imu(roll, 0.0, [0.0; 3], k * 1000)
        } else {
            ImuSample { gyro: [0.0; 3], accel: f_body, timestamp_us: k * 1000 }
        };
        last = f.update(&s, DT);
        if k % 20 == 0 {
            f.update_baro(&baro(0.0, k * 1000));
        }
    }
    let v = last.vertical.unwrap();
    // Read as body-z specific force, this would be a 0.8 m/s² climb.
    assert!(v.climb_rate_mps.abs() < 0.02, "climb rate {}", v.climb_rate_mps);
    assert!(v.altitude_m.abs() < 0.02, "height {}", v.altitude_m);
}

#[test]
fn vertical_is_withdrawn_when_the_barometer_stops() {
    let mut f = filter();
    fly(&mut f, 1.0, |_| 0.0, |_| 0.0, 0.0, || 0.0, |_, _| {});
    let mut e = StateEstimate::default();
    for k in 0..600u32 {
        e = f.update(&imu(0.0, 0.0, [0.0; 3], 1_000_000 + k * 1000), DT);
    }
    assert!(e.vertical.is_none(), "still reported 0.6 s after the last baro reading");
    f.update_baro(&baro(0.0, 1_700_000));
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 1_701_000), DT);
    assert!(e.vertical.is_some(), "did not come back with the barometer");
}

#[test]
fn reset_clears_heading_and_vertical() {
    let mut f = filter();
    let mut t = 0;
    hold_with_mag(&mut f, 0.0, 0.0, 45.0 * DEG, 0.1, 0.0, &mut t);
    fly(&mut f, 1.0, |_| 0.0, |_| 0.0, 0.0, || 0.0, |_, _| {});
    f.reset();
    let e = f.update(&imu(0.0, 0.0, [0.0; 3], 0), DT);
    assert!(!e.heading_referenced);
    assert!(e.vertical.is_none());
    assert_abs_diff_eq!(yaw_deg(&e), 0.0);
}

#[test]
fn pressure_altitude() {
    assert_abs_diff_eq!(pressure_altitude_m(P0, P0), 0.0, epsilon = 1e-3);
    // Near sea level, 1 hPa is about 8.4 m.
    assert_abs_diff_eq!(pressure_altitude_m(P0 - 100.0, P0), 8.4, epsilon = 0.1);
    // Standard atmosphere: 1000 m is 89 875 Pa.
    assert_abs_diff_eq!(pressure_altitude_m(89_875.0, P0), 1000.0, epsilon = 2.0);
    // And the test helper inverts it.
    assert_abs_diff_eq!(pressure_altitude_m(pressure_at(123.4), P0), 123.4, epsilon = 0.05);
}
