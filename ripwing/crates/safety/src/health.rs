//! Task liveness monitoring: the decision half of the hardware watchdog.
//!
//! The naive watchdog has one task petting the timer on a schedule. That only
//! proves *that* task is alive — every other task could be dead and the
//! watchdog would happily keep the chip running.
//!
//! This instead aggregates health across tasks. Each monitored task
//! increments a free-running counter every iteration; this monitor samples
//! those counters periodically and confirms each one advanced by at least a
//! required amount. Only if every task passes does the caller pet the
//! hardware watchdog. If any task has died or badly degraded, the pet is
//! withheld and the watchdog resets the chip.
//!
//! Requiring a *minimum delta* rather than merely "changed" is deliberate: a
//! task that is running but at a fraction of its design rate is missing
//! deadlines just as surely as one that has stopped, and a bare
//! liveness check would not catch it.
//!
//! This module is pure — no timers, no peripherals, no clock. Counts come in
//! as arguments, so the whole thing is deterministic and host-testable.

/// Which task fell short, and by how much. Reporting the specific task (and
/// the numbers) rather than a bare boolean means the failure is diagnosable
/// from a single log line before the reset lands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StallReport {
    /// Index into the caller's monitored-task array.
    pub index: usize,
    /// Ticks actually observed in the interval.
    pub observed: u32,
    /// Ticks that were required.
    pub required: u32,
}

/// Liveness monitor over `N` task counters.
///
/// `N` is a const generic so the snapshot array is fixed-size with no
/// allocation, and the monitored set is fixed at compile time.
#[derive(Clone, Copy, Debug)]
pub struct HealthMonitor<const N: usize> {
    last: [u32; N],
    min_delta: [u32; N],
    primed: bool,
}

impl<const N: usize> HealthMonitor<N> {
    /// Build a monitor with a per-task minimum tick count per check interval.
    ///
    /// Size each threshold from the task's design rate and the check period,
    /// with margin for jitter. A 1 kHz task checked every 50 ms should tick
    /// about 50 times; requiring 25 catches a task running at half rate while
    /// tolerating ordinary scheduling jitter.
    pub const fn new(min_delta: [u32; N]) -> Self {
        Self {
            last: [0; N],
            min_delta,
            primed: false,
        }
    }

    /// Sample the counters and decide whether the system is healthy.
    ///
    /// Returns `Ok(())` if every task met its threshold, or the first
    /// shortfall found. The caller pets the hardware watchdog only on `Ok`.
    ///
    /// The first call primes the baseline and always returns `Ok`: there is
    /// no previous sample to measure against, and tripping a reset at boot
    /// before any task has had a chance to run would be a guaranteed
    /// reset loop.
    pub fn check(&mut self, counts: &[u32; N]) -> Result<(), StallReport> {
        if !self.primed {
            self.last = *counts;
            self.primed = true;
            return Ok(());
        }

        let mut i = 0;
        while i < N {
            // wrapping_sub so a counter rolling over u32::MAX (about 50 days
            // at 1 kHz) yields the true delta rather than a huge bogus one.
            let delta = counts[i].wrapping_sub(self.last[i]);
            if delta < self.min_delta[i] {
                let report = StallReport {
                    index: i,
                    observed: delta,
                    required: self.min_delta[i],
                };
                // Re-baseline even on failure. If the caller chooses to keep
                // running (logging the stall rather than resetting), the next
                // check measures the next interval rather than compounding
                // the same shortfall forever.
                self.last = *counts;
                return Err(report);
            }
            i += 1;
        }

        self.last = *counts;
        Ok(())
    }

    /// Drop the baseline. The next `check` re-primes and returns `Ok`.
    /// Use after a deliberate pause in task execution so the resumption is
    /// not misread as a stall.
    pub fn reset(&mut self) {
        self.last = [0; N];
        self.primed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Three tasks, each required to tick at least 25 times per interval.
    fn monitor() -> HealthMonitor<3> {
        HealthMonitor::new([25, 25, 25])
    }

    #[test]
    fn first_check_primes_and_passes() {
        // Tripping at boot, before any task has run, would be a reset loop.
        let mut m = monitor();
        assert_eq!(m.check(&[0, 0, 0]), Ok(()));
    }

    #[test]
    fn all_tasks_advancing_passes() {
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();
        assert_eq!(m.check(&[50, 50, 50]), Ok(()));
        assert_eq!(m.check(&[100, 100, 100]), Ok(()));
    }

    #[test]
    fn frozen_task_is_caught() {
        // The headline case: one task dies, its counter stops advancing.
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();

        let err = m.check(&[50, 0, 50]).unwrap_err();
        assert_eq!(err.index, 1);
        assert_eq!(err.observed, 0);
        assert_eq!(err.required, 25);
    }

    #[test]
    fn degraded_task_is_caught_not_just_dead_ones() {
        // Running at 40% of design rate is still missing deadlines. A bare
        // "did it change" check would pass this; a minimum delta does not.
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();

        let err = m.check(&[50, 50, 20]).unwrap_err();
        assert_eq!(err.index, 2);
        assert_eq!(err.observed, 20);
    }

    #[test]
    fn exactly_at_threshold_passes() {
        // Boundary: delta == required is healthy; only strictly less trips.
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();
        assert_eq!(m.check(&[25, 25, 25]), Ok(()));
    }

    #[test]
    fn one_below_threshold_trips() {
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();
        let err = m.check(&[25, 24, 25]).unwrap_err();
        assert_eq!(err.index, 1);
        assert_eq!(err.observed, 24);
    }

    #[test]
    fn lowest_index_reported_when_several_stall() {
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();
        let err = m.check(&[0, 0, 50]).unwrap_err();
        assert_eq!(err.index, 0, "should report the first stalled task");
    }

    #[test]
    fn counter_wraparound_is_handled() {
        // At 1 kHz a u32 counter wraps after ~50 days. A plain subtraction
        // would underflow to a huge delta and silently pass; worse, if the
        // comparison went the other way it would trigger a spurious reset
        // mid-flight. wrapping_sub gives the true delta.
        let mut m = monitor();
        let near_max = u32::MAX - 10;
        m.check(&[near_max, near_max, near_max]).unwrap();

        // Each advanced by 40, wrapping past the top.
        let wrapped = near_max.wrapping_add(40);
        assert_eq!(m.check(&[wrapped, wrapped, wrapped]), Ok(()));
    }

    #[test]
    fn rebaselines_after_a_stall() {
        // A stall must not poison every later check. Once re-baselined, a
        // recovered task passes again.
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();
        m.check(&[50, 0, 50]).unwrap_err();

        // Task 1 recovers and now advances normally.
        assert_eq!(m.check(&[100, 50, 100]), Ok(()));
    }

    #[test]
    fn reset_reprimes() {
        let mut m = monitor();
        m.check(&[0, 0, 0]).unwrap();
        m.reset();
        // Post-reset the first call primes again, so even a frozen counter
        // passes once rather than tripping immediately.
        assert_eq!(m.check(&[999, 999, 999]), Ok(()));
    }

    #[test]
    fn per_task_thresholds_are_independent() {
        // A 1 kHz task and a 20 Hz task cannot share a threshold.
        let mut m: HealthMonitor<2> = HealthMonitor::new([25, 1]);
        m.check(&[0, 0]).unwrap();

        // Fast task ticks 50, slow task ticks 1: both healthy.
        assert_eq!(m.check(&[50, 1]), Ok(()));

        // Slow task stops: caught even though its threshold is tiny.
        let err = m.check(&[100, 1]).unwrap_err();
        assert_eq!(err.index, 1);
    }
}
