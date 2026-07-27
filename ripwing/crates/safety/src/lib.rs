//! Failsafe state machine and safety checks.
//!
//! Pure decision logic: given a snapshot of vehicle inputs, decide the arm
//! state and whether motors may run. No hardware, no HAL, no clock — time is
//! passed in as a parameter so the whole thing is deterministic and
//! host-testable.
//!
//! Design constraint (case study §3.3, line 159): the failsafe must not
//! depend on any other task being healthy. We honour that by having every
//! check derive its verdict from raw inputs the monitor can independently
//! judge — a timestamp it compares against *now*, a raw attitude, a raw
//! battery voltage — rather than trusting a status flag some other task set.
//! If sensor fusion silently dies, the staleness check catches it because
//! the timestamp stops advancing; we never ask fusion "are you ok?".

#![no_std]

use ripwing_common::StateEstimate;

/// Arm state of the vehicle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmState {
    /// Motors inhibited. The safe default and power-on state.
    Disarmed,
    /// Motors permitted; normal flight.
    Armed,
    /// A safety check tripped while armed. Motors cut. Latched until an
    /// explicit disarm — we do not auto-recover into flight, because a
    /// momentarily-cleared fault does not mean the vehicle is safe.
    Failsafe,
}

/// Why the monitor cut (or refused to arm). Useful for telemetry and for
/// asserting the *reason* in tests, not just the outcome.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SafetyFault {
    /// State estimate is older than the allowed age (fusion stalled/died).
    StaleState,
    /// Attitude exceeded the survivable tilt limit.
    AttitudeLimit,
    /// Body rate exceeded the plausible limit (spin / sensor fault).
    RateLimit,
    /// RC/pilot link heartbeat went stale (lost control).
    RcLinkLoss,
    /// Battery below the floor (brownout risk).
    LowBattery,
}

/// The action the control task must take this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorPermission {
    /// Motors may run normally.
    Run,
    /// Motors must be cut this tick.
    Cut,
}

/// A snapshot of everything the monitor needs to reach a verdict. The task
/// assembles this each tick from shared state and raw sensor/link inputs.
///
/// Times are in microseconds on the same monotonic clock as
/// `StateEstimate.timestamp_us`. `now_us` is the current time; the monitor
/// compares timestamps against it rather than reading a clock itself, which
/// is what keeps it deterministic and testable.
#[derive(Clone, Copy, Debug)]
pub struct SafetyInputs {
    pub now_us: u32,
    /// Latest fused state (carries its own sample timestamp).
    pub state: StateEstimate,
    /// Timestamp of the last RC/pilot heartbeat [µs].
    pub last_rc_heartbeat_us: u32,
    /// Battery voltage [V].
    pub battery_volts: f32,
}

/// Configurable safety limits. Set once for the airframe.
#[derive(Clone, Copy, Debug)]
pub struct SafetyLimits {
    /// Max age of a state estimate before it is considered stale [µs].
    pub max_state_age_us: u32,
    /// Max age of an RC heartbeat before link-loss [µs].
    pub max_rc_age_us: u32,
    /// Max absolute roll/pitch before failsafe [rad].
    pub max_tilt_rad: f32,
    /// Max absolute body rate on any axis before failsafe [rad/s].
    pub max_body_rate_rad_s: f32,
    /// Battery floor below which we refuse to run [V].
    pub min_battery_volts: f32,
    /// Arming requires attitude within this of level [rad].
    pub arm_max_tilt_rad: f32,
    /// Arming requires body rates below this [rad/s].
    pub arm_max_rate_rad_s: f32,
}

impl Default for SafetyLimits {
    fn default() -> Self {
        // Conservative starting values. Tune to the airframe. These are the
        // kind of numbers you set deliberately and revisit on the bench,
        // not magic constants — every one is a policy decision.
        Self {
            max_state_age_us: 5_000,      // 5 ms: >5 missed 1 kHz updates
            max_rc_age_us: 500_000,       // 500 ms of RC silence = link loss
            max_tilt_rad: 1.396,          // ~80°: past this, recovery unlikely
            max_body_rate_rad_s: 17.45,   // ~1000°/s: implausible in normal flight
            min_battery_volts: 10.5,      // 3S LiPo floor; set per pack
            arm_max_tilt_rad: 0.087,      // ~5°: must be near level to arm
            arm_max_rate_rad_s: 0.175,    // ~10°/s: must be near still to arm
        }
    }
}

/// A single check's outcome: either fine, or a specific fault.
type CheckResult = Result<(), SafetyFault>;

/// The monitor. Holds the current arm state and the limits; the verdict
/// logic is otherwise stateless (all inputs come in per tick).
#[derive(Clone, Copy, Debug)]
pub struct SafetyMonitor {
    state: ArmState,
    limits: SafetyLimits,
    /// The fault that caused the current Failsafe, if any (for telemetry).
    last_fault: Option<SafetyFault>,
}

impl SafetyMonitor {
    pub fn new(limits: SafetyLimits) -> Self {
        Self {
            state: ArmState::Disarmed,
            limits,
            last_fault: None,
        }
    }

    pub fn arm_state(&self) -> ArmState {
        self.state
    }

    pub fn last_fault(&self) -> Option<SafetyFault> {
        self.last_fault
    }

    // ---- Individual checks (each independently verifiable) ---------------
    // Split out so each can be unit-tested in isolation and so the reason
    // for a cut is always a specific SafetyFault, never a vague boolean.

    fn check_state_age(&self, inp: &SafetyInputs) -> CheckResult {
        // Guard against a timestamp in the future (clock glitch) by using a
        // saturating diff; a future timestamp reads as age 0, not underflow.
        let age = inp.now_us.saturating_sub(inp.state.timestamp_us);
        if age > self.limits.max_state_age_us {
            Err(SafetyFault::StaleState)
        } else {
            Ok(())
        }
    }

    fn check_attitude(&self, inp: &SafetyInputs) -> CheckResult {
        let roll = abs(inp.state.attitude[0]);
        let pitch = abs(inp.state.attitude[1]);
        if roll > self.limits.max_tilt_rad || pitch > self.limits.max_tilt_rad {
            Err(SafetyFault::AttitudeLimit)
        } else {
            Ok(())
        }
    }

    fn check_rates(&self, inp: &SafetyInputs) -> CheckResult {
        for &r in inp.state.body_rates.iter() {
            if abs(r) > self.limits.max_body_rate_rad_s {
                return Err(SafetyFault::RateLimit);
            }
        }
        Ok(())
    }

    fn check_rc_link(&self, inp: &SafetyInputs) -> CheckResult {
        let age = inp.now_us.saturating_sub(inp.last_rc_heartbeat_us);
        if age > self.limits.max_rc_age_us {
            Err(SafetyFault::RcLinkLoss)
        } else {
            Ok(())
        }
    }

    fn check_battery(&self, inp: &SafetyInputs) -> CheckResult {
        if inp.battery_volts < self.limits.min_battery_volts {
            Err(SafetyFault::LowBattery)
        } else {
            Ok(())
        }
    }

    /// Run every in-flight check. Returns the first fault found, or Ok.
    /// Order encodes priority: the most immediately dangerous first.
    fn run_all_checks(&self, inp: &SafetyInputs) -> CheckResult {
        self.check_state_age(inp)?;
        self.check_attitude(inp)?;
        self.check_rates(inp)?;
        self.check_rc_link(inp)?;
        self.check_battery(inp)?;
        Ok(())
    }

    // ---- Public transitions ---------------------------------------------

    /// Attempt to arm. Succeeds only from Disarmed and only if every arming
    /// precondition holds. Returns the resulting arm state.
    ///
    /// Arming is stricter than staying armed: we require the vehicle to be
    /// near-level and near-still, not merely within the (wider) in-flight
    /// limits, so you cannot arm mid-tumble.
    pub fn try_arm(&mut self, inp: &SafetyInputs) -> ArmState {
        if self.state != ArmState::Disarmed {
            return self.state;
        }
        // All in-flight checks must pass...
        if let Err(f) = self.run_all_checks(inp) {
            self.last_fault = Some(f);
            return self.state; // stays Disarmed
        }
        // ...plus the stricter arming preconditions.
        let tilt_ok = abs(inp.state.attitude[0]) <= self.limits.arm_max_tilt_rad
            && abs(inp.state.attitude[1]) <= self.limits.arm_max_tilt_rad;
        let still_ok = inp
            .state
            .body_rates
            .iter()
            .all(|&r| abs(r) <= self.limits.arm_max_rate_rad_s);

        if tilt_ok && still_ok {
            self.state = ArmState::Armed;
            self.last_fault = None;
        }
        self.state
    }

    /// Explicit disarm. Always allowed, from any state. This is also how you
    /// clear a latched Failsafe (deliberate human action).
    pub fn disarm(&mut self) {
        self.state = ArmState::Disarmed;
        self.last_fault = None;
    }

    /// The per-tick update the control task calls. Runs the checks when
    /// armed, latches Failsafe on any fault, and returns whether motors may
    /// run this tick.
    ///
    /// Verdict rules:
    ///   - Disarmed  -> Cut (motors inhibited).
    ///   - Armed     -> run checks; on fault, latch Failsafe + Cut; else Run.
    ///   - Failsafe  -> Cut, latched until explicit disarm.
    pub fn update(&mut self, inp: &SafetyInputs) -> MotorPermission {
        match self.state {
            ArmState::Disarmed => MotorPermission::Cut,
            ArmState::Failsafe => MotorPermission::Cut,
            ArmState::Armed => match self.run_all_checks(inp) {
                Ok(()) => MotorPermission::Run,
                Err(fault) => {
                    self.state = ArmState::Failsafe;
                    self.last_fault = Some(fault);
                    MotorPermission::Cut
                }
            },
        }
    }
}

/// `f32::abs` lives in `std`; in `no_std` we spell it out. Avoids pulling a
/// math crate in for one trivial operation.
#[inline]
fn abs(x: f32) -> f32 {
    if x < 0.0 {
        -x
    } else {
        x
    }
}

#[cfg(test)]
mod tests;
