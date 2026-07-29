//! Host-side validation of the sensor filters.
//!
//! The properties worth pinning down are behavioural, not just numerical:
//! does the median actually discard an outlier, does it still let a genuine
//! manoeuvre through, does the average have unity DC gain, and does the whole
//! chain start settled rather than ramping from zero at arm time.

use super::*;
use ripwing_common::ImuSample;

// ---- median3 primitive ---------------------------------------------------

#[test]
fn median3_is_correct_for_every_ordering() {
    // Same three values in all six orders must give the same median.
    assert_eq!(median3(1.0, 2.0, 3.0), 2.0);
    assert_eq!(median3(1.0, 3.0, 2.0), 2.0);
    assert_eq!(median3(2.0, 1.0, 3.0), 2.0);
    assert_eq!(median3(2.0, 3.0, 1.0), 2.0);
    assert_eq!(median3(3.0, 1.0, 2.0), 2.0);
    assert_eq!(median3(3.0, 2.0, 1.0), 2.0);
}

#[test]
fn median3_handles_duplicates_and_negatives() {
    assert_eq!(median3(5.0, 5.0, 5.0), 5.0);
    assert_eq!(median3(0.0, 0.0, 100.0), 0.0);
    assert_eq!(median3(-1.0, -5.0, -3.0), -3.0);
}

// ---- sliding median ------------------------------------------------------

#[test]
fn median_rejects_an_isolated_spike() {
    // The whole point: one corrupted sample must never reach the output.
    let mut m = MedianOf3::new();
    m.update(0.0);
    m.update(0.0);

    let out = m.update(1000.0); // the spike
    assert_eq!(out, 0.0, "spike leaked through the median");

    // And it must not contaminate the following samples either.
    assert_eq!(m.update(0.0), 0.0);
    assert_eq!(m.update(0.0), 0.0);
}

#[test]
fn median_passes_a_genuine_step() {
    // A median is not a low-pass. A sustained change (the vehicle actually
    // rotating) must get through, delayed by about one sample.
    let mut m = MedianOf3::new();
    m.update(0.0);

    assert_eq!(m.update(10.0), 0.0, "step should lag one sample");
    assert_eq!(m.update(10.0), 10.0, "step should have arrived by now");
    assert_eq!(m.update(10.0), 10.0);
}

#[test]
fn median_primes_on_first_sample_no_startup_ramp() {
    // Without priming the history would sit at 0.0 and the first outputs
    // would ramp up from zero — a transient at exactly the wrong moment.
    let mut m = MedianOf3::new();
    assert_eq!(m.update(42.0), 42.0, "first sample must pass through");
    assert_eq!(m.update(42.0), 42.0);
}

#[test]
fn median_reset_reprimes() {
    let mut m = MedianOf3::new();
    m.update(100.0);
    m.reset();
    assert_eq!(m.update(7.0), 7.0, "after reset the next sample re-primes");
}

// ---- moving average ------------------------------------------------------

#[test]
fn average_has_unity_dc_gain() {
    // A constant in must give the same constant out, or the filter is
    // scaling the signal — which would silently rescale every gain downstream.
    let mut a: MovingAverage<4> = MovingAverage::new();
    for _ in 0..20 {
        approx::assert_abs_diff_eq!(a.update(5.0), 5.0, epsilon = 1e-6);
    }
}

#[test]
fn average_primes_on_first_sample() {
    let mut a: MovingAverage<8> = MovingAverage::new();
    approx::assert_abs_diff_eq!(a.update(10.0), 10.0, epsilon = 1e-6);
}

#[test]
fn average_converges_over_exactly_n_samples() {
    // Step from 0 to 10 with N=4: output should climb 2.5, 5.0, 7.5, 10.0
    // and arrive exactly N samples after the step.
    let mut a: MovingAverage<4> = MovingAverage::new();
    a.update(0.0); // prime at zero

    approx::assert_abs_diff_eq!(a.update(10.0), 2.5, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(a.update(10.0), 5.0, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(a.update(10.0), 7.5, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(a.update(10.0), 10.0, epsilon = 1e-6);
}

#[test]
fn average_cancels_alternating_noise() {
    // Perfectly alternating +/-1 through an even-length window should average
    // to zero once the window has filled.
    let mut a: MovingAverage<4> = MovingAverage::new();
    a.update(1.0); // prime

    let mut out = 0.0;
    for i in 0..12 {
        out = a.update(if i % 2 == 0 { -1.0 } else { 1.0 });
    }
    approx::assert_abs_diff_eq!(out, 0.0, epsilon = 1e-6);
}

#[test]
fn average_group_delay_matches_theory() {
    // (N-1)/2 samples.
    approx::assert_abs_diff_eq!(group_delay_samples(1), 0.0, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(group_delay_samples(4), 1.5, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(group_delay_samples(5), 2.0, epsilon = 1e-6);

    // 2 samples at 1 kHz is 2 ms.
    approx::assert_abs_diff_eq!(
        group_delay_seconds(5, 1000.0),
        0.002,
        epsilon = 1e-9
    );
}

#[test]
fn average_reset_reprimes() {
    let mut a: MovingAverage<4> = MovingAverage::new();
    for _ in 0..10 {
        a.update(100.0);
    }
    a.reset();
    approx::assert_abs_diff_eq!(a.update(3.0), 3.0, epsilon = 1e-6);
}

// ---- composed scalar filter ---------------------------------------------

#[test]
fn scalar_filter_rejects_spike_and_stays_quiet() {
    // The headline behaviour: a large isolated outlier on an otherwise steady
    // signal must not move the output at all.
    let mut f: ScalarFilter<4> = ScalarFilter::new();
    f.update(0.0);
    f.update(0.0);

    let out = f.update(1000.0);
    approx::assert_abs_diff_eq!(out, 0.0, epsilon = 1e-6);
}

#[test]
fn scalar_filter_still_tracks_a_real_change() {
    // Rejecting spikes is worthless if genuine motion is also rejected.
    // Hold a new level and the output must settle there.
    let mut f: ScalarFilter<4> = ScalarFilter::new();
    f.update(0.0);

    let mut out = 0.0;
    for _ in 0..12 {
        out = f.update(10.0);
    }
    approx::assert_abs_diff_eq!(out, 10.0, epsilon = 1e-6);
}

#[test]
fn scalar_filter_primes_clean() {
    let mut f: ScalarFilter<8> = ScalarFilter::new();
    approx::assert_abs_diff_eq!(f.update(-3.5), -3.5, epsilon = 1e-6);
}

#[test]
fn scalar_filter_reports_total_group_delay() {
    // 1 sample (median) + (N-1)/2 (average).
    approx::assert_abs_diff_eq!(
        ScalarFilter::<4>::group_delay_samples(),
        2.5,
        epsilon = 1e-6
    );
    // At 1 kHz that is 2.5 ms of lag charged to the phase budget.
    approx::assert_abs_diff_eq!(
        ScalarFilter::<4>::group_delay_seconds(1000.0),
        0.0025,
        epsilon = 1e-9
    );
}

// ---- three-axis ----------------------------------------------------------

#[test]
fn vec3_axes_are_independent() {
    // A spike on roll must not perturb pitch or yaw. Cross-talk here would
    // couple a single bad sample into all three control axes.
    let mut f: Vec3Filter<4> = Vec3Filter::new();
    f.update([0.0, 1.0, 2.0]);
    f.update([0.0, 1.0, 2.0]);

    let out = f.update([1000.0, 1.0, 2.0]);
    approx::assert_abs_diff_eq!(out[0], 0.0, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(out[1], 1.0, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(out[2], 2.0, epsilon = 1e-6);
}

// ---- IMU sample ----------------------------------------------------------

#[test]
fn imu_filter_passes_timestamp_through_untouched() {
    // The timestamp marks when the sample was taken. Staleness checks in the
    // safety monitor depend on it, so the filter must not alter it.
    let mut f: ImuFilter<4, 8> = ImuFilter::new();
    let raw = ImuSample {
        gyro: [0.1, 0.2, 0.3],
        accel: [0.0, 0.0, 9.81],
        timestamp_us: 123_456,
    };
    let filtered = f.update(&raw);
    assert_eq!(filtered.timestamp_us, 123_456);
}

#[test]
fn imu_filter_leaves_raw_sample_intact() {
    // "Filter for control, log the raw for detection" — the caller must still
    // have the unmodified sample after filtering.
    let mut f: ImuFilter<4, 8> = ImuFilter::new();
    let raw = ImuSample {
        gyro: [1.0, 2.0, 3.0],
        accel: [4.0, 5.0, 6.0],
        timestamp_us: 1,
    };
    let _ = f.update(&raw);
    assert_eq!(raw.gyro, [1.0, 2.0, 3.0]);
    assert_eq!(raw.accel, [4.0, 5.0, 6.0]);
}

#[test]
fn imu_filter_rejects_a_gyro_spike_without_touching_accel() {
    let mut f: ImuFilter<4, 4> = ImuFilter::new();
    let steady = ImuSample {
        gyro: [0.0, 0.0, 0.0],
        accel: [0.0, 0.0, 9.81],
        timestamp_us: 0,
    };
    f.update(&steady);
    f.update(&steady);

    let spiked = ImuSample {
        gyro: [500.0, 0.0, 0.0],
        ..steady
    };
    let out = f.update(&spiked);

    approx::assert_abs_diff_eq!(out.gyro[0], 0.0, epsilon = 1e-6);
    approx::assert_abs_diff_eq!(out.accel[2], 9.81, epsilon = 1e-5);
}
