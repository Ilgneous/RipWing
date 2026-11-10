//! Battery monitor behaviour.

use crate::battery::*;

const DT: f32 = 0.002; // safety task samples each channel at 500 Hz

fn monitor() -> BatteryMonitor {
    BatteryMonitor::new(BatteryConfig::default())
}

/// Feed `volts(t)` for `secs`, return the last status and every level seen.
fn run(b: &mut BatteryMonitor, secs: f32, volts: impl Fn(f32) -> f32) -> (BatteryStatus, std::vec::Vec<BatteryLevel>) {
    let mut seen = std::vec::Vec::new();
    let mut s = b.status();
    for k in 0..(secs / DT) as u32 {
        s = b.update(volts(k as f32 * DT), None, DT);
        if seen.last() != Some(&s.level) {
            seen.push(s.level);
        }
    }
    (s, seen)
}

extern crate std;

#[test]
fn usb_power_only_is_absent() {
    let mut b = monitor();
    for v in [0.0, 0.4, 2.4, f32::NAN] {
        let s = b.update(v, None, DT);
        assert_eq!(s.level, BatteryLevel::Absent, "{v} V");
        assert!(!s.ready_to_arm);
    }
}

#[test]
fn counts_cells_from_any_charged_or_storage_pack() {
    for (volts, cells) in [(8.4, 2), (7.6, 2), (12.6, 3), (11.4, 3), (10.5, 3), (16.8, 4), (15.2, 4), (25.2, 6), (22.8, 6), (17.4, 4)] {
        let (s, _) = run(&mut monitor(), 1.0, |_| volts);
        assert_eq!(s.cells, cells, "{volts} V");
        assert_eq!(s.level, BatteryLevel::Ok, "{volts} V");
    }
}

#[test]
fn a_fixed_cell_count_wins() {
    // A 4S pack drained to 3.25 V/cell would be counted as 3S.
    let (s, _) = run(&mut monitor(), 1.0, |_| 13.0);
    assert_eq!(s.cells, 3);
    let mut b = BatteryMonitor::new(BatteryConfig { cells: Some(4), ..Default::default() });
    let (s, _) = run(&mut b, 0.01, |_| 13.0);
    assert_eq!(s.cells, 4);
    assert!(!s.ready_to_arm, "3.25 V/cell must not arm");
}

#[test]
fn counts_only_after_the_pack_settles() {
    let mut b = monitor();
    let (s, _) = run(&mut b, 0.4, |_| 12.4);
    assert_eq!(s.level, BatteryLevel::Detecting);
    assert!(!s.ready_to_arm);
    let (s, _) = run(&mut b, 0.2, |_| 12.4);
    assert_eq!(s.level, BatteryLevel::Ok);
    assert!(s.ready_to_arm);
}

#[test]
fn filter_starts_at_the_first_reading() {
    // Not ramping up from 0 V, which would read as a dead pack.
    let s = monitor().update(12.4, None, DT);
    assert_eq!(s.volts, 12.4);
}

#[test]
fn throttle_sag_never_raises_a_warning() {
    // 3S at 3.8 V/cell resting, punched to 3.2 V/cell (below critical) for
    // 0.6 s every 3 s, for a minute.
    let mut b = monitor();
    let (s, seen) = run(&mut b, 60.0, |t| if t > 1.0 && t % 3.0 < 0.6 { 9.6 } else { 11.4 });
    assert_eq!(seen, [BatteryLevel::Detecting, BatteryLevel::Ok]);
    assert_eq!(s.level, BatteryLevel::Ok);
}

#[test]
fn a_long_climb_below_critical_is_still_sag() {
    // 1.5 s full-throttle climbs at 3.0 V/cell every 6 s: the filtered
    // voltage dips under the 3.3 V critical line for ~0.7 s each time,
    // inside the 2 s hold, and the pack is fine (3.8 V/cell resting).
    let mut b = monitor();
    let (s, seen) = run(&mut b, 60.0, |t| if t > 1.0 && t % 6.0 < 1.5 { 9.0 } else { 11.4 });
    assert_eq!(seen, [BatteryLevel::Detecting, BatteryLevel::Ok]);
    assert_eq!(s.level, BatteryLevel::Ok);
}

#[test]
fn sustained_low_is_reported_after_the_hold() {
    // Resting 3.8 V/cell, then a steady 3.4 V/cell from t = 1 s.
    let mut b = monitor();
    let mut low_at = None;
    for k in 0..(10.0 / DT) as u32 {
        let t = k as f32 * DT;
        let s = b.update(if t < 1.0 { 11.4 } else { 10.2 }, None, DT);
        if low_at.is_none() && s.level == BatteryLevel::Low {
            low_at = Some(t);
        }
    }
    // Filter crosses 3.5 V/cell ~1.4 s after the drop, then 2 s of hold.
    let t = low_at.expect("never reported Low");
    assert!((4.0..5.0).contains(&t), "Low at {t} s");
}

#[test]
fn critical_follows_low() {
    let mut b = monitor();
    let (s, seen) = run(&mut b, 15.0, |t| if t < 1.0 { 11.4 } else if t < 6.0 { 10.2 } else { 9.6 });
    assert_eq!(seen, [BatteryLevel::Detecting, BatteryLevel::Ok, BatteryLevel::Low, BatteryLevel::Critical]);
    assert_eq!(s.level, BatteryLevel::Critical);
    assert!(!s.ready_to_arm);
}

#[test]
fn levels_latch_until_the_pack_is_unplugged() {
    let mut b = monitor();
    run(&mut b, 1.0, |_| 11.4);
    let (s, _) = run(&mut b, 5.0, |_| 10.2);
    assert_eq!(s.level, BatteryLevel::Low);
    // Load off: the voltage recovers, the warning does not flicker away.
    let (s, _) = run(&mut b, 10.0, |_| 11.4);
    assert_eq!(s.level, BatteryLevel::Low);
    assert!(!s.ready_to_arm);
    // Swap packs.
    let s = b.update(0.0, None, DT);
    assert_eq!(s.level, BatteryLevel::Absent);
    let (s, _) = run(&mut b, 1.0, |_| 12.6);
    assert_eq!(s.level, BatteryLevel::Ok);
    assert!(s.ready_to_arm);
}

#[test]
fn arming_needs_a_rested_voltage_above_the_floor() {
    // 3.45 V/cell resting: present, counted, not yet Low (needs 2 s of
    // hold), but below the 3.5 V/cell arming floor.
    let (s, _) = run(&mut monitor(), 1.0, |_| 10.35);
    assert_eq!(s.level, BatteryLevel::Ok);
    assert!(!s.ready_to_arm);
    let (s, _) = run(&mut monitor(), 1.0, |_| 10.6);
    assert!(s.ready_to_arm);
}

#[test]
fn integrates_current_into_mah() {
    let mut b = monitor();
    let mut s = b.status();
    for _ in 0..(36.0 / DT) as u32 {
        s = b.update(12.0, Some(10.0), DT);
    }
    // 10 A for 36 s = 100 mAh.
    assert!((s.mah_used - 100.0).abs() < 0.1, "{} mAh", s.mah_used);
    assert_eq!(s.amps, Some(10.0));
}

#[test]
fn no_current_sensor_or_a_bad_reading() {
    let mut b = monitor();
    let s = b.update(12.0, None, DT);
    assert_eq!((s.amps, s.mah_used), (None, 0.0));
    let s = b.update(12.0, Some(f32::NAN), DT);
    assert_eq!((s.amps, s.mah_used), (None, 0.0));
}

#[test]
fn bad_dt_is_ignored() {
    let mut b = monitor();
    run(&mut b, 1.0, |_| 12.0);
    let before = b.status();
    for dt in [f32::NAN, -1.0, 0.0] {
        let s = b.update(12.0, Some(50.0), dt);
        assert_eq!(s.mah_used, before.mah_used);
        assert_eq!(s.level, BatteryLevel::Ok);
    }
}
