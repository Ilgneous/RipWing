#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod anomaly;
mod board;
mod drivers;
mod telemetry;
mod transport;

// NOTE: crate imports for the control and safety logic live INSIDE `mod app`
// below (next to the transport import), because the code that uses them is
// inside that module. A `use` at this outer scope would not be visible there.

// Rate-monotonic priority ladder (RTIC: higher number = higher priority).
// Periods drive the assignment; ties at equal period are broken by
// criticality (safety > control > fusion). See case study §4.3.
//
//   Task              Period   Rate     Priority
//   safety_monitor    1 ms     1 kHz    6  (highest)
//   attitude_control  1 ms     1 kHz    5
//   sensor_fusion     1 ms     1 kHz    4
//   anomaly           20 ms    50 Hz    3
//   logging           10 ms    100 Hz   2
//   telemetry         50 ms    20 Hz    1  (lowest)

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI3, SPI4, SPI5, USART1, USART2, USART6])]
mod app {
    use crate::board;
    use crate::transport::{
        take, tick, MotorCommand, RateSetpoint, Severity, SeverityFlag, StateEstimate,
        TaskCounters,
    };
    // The control logic is an external, host-testable crate (§3.1); firmware
    // calls into it through the Controller trait.
    use ripwing_control::{AttitudeGains, AttitudeRateController, Controller, PidConfig};
    use ripwing_safety::{MotorPermission, SafetyInputs, SafetyLimits, SafetyMonitor};
    use rtic_monotonics::systick::prelude::*;
    use stm32f4xx_hal::{
        gpio::{Output, PushPull, PC13},
        prelude::*,
    };

    systick_monotonic!(Mono, 1_000); // 1 kHz systick

    // ---- Task periods, one place to change them --------------------------
    const CONTROL_PERIOD_MS: u32 = 1; // 1 kHz control chain
    const ANOMALY_PERIOD_MS: u32 = 20; // 50 Hz ML monitor
    const LOGGING_PERIOD_MS: u32 = 10; // 100 Hz recorder drain
    const TELEMETRY_PERIOD_MS: u32 = 50; // 20 Hz downlink

    // ---- Shared resources (accessed by more than one task) ---------------
    // RTIC arbitrates access by priority ceiling; no manual locks needed.
    #[shared]
    struct Shared {
        /// Latest fused state: written by fusion, read by control + safety.
        state: StateEstimate,
        /// Current rate command, read by the inner control loop. Written by
        /// the pilot/RC path today; will be written by the outer attitude
        /// loop once that stage exists. Typed as RateSetpoint so the inner
        /// loop's input is unambiguous.
        rate_setpoint: RateSetpoint,
        /// Latest mixer output: written by control, read by safety monitor.
        motor_cmd: MotorCommand,
        /// Safety verdict: written by safety_monitor, read by attitude_control.
        /// When false, control must output a zeroed motor command regardless
        /// of what the PID computed. This is how the priority-6 monitor's
        /// decision reaches the actuation path.
        motors_enabled: bool,
    }

    // ---- Local resources (owned by exactly one task) ---------------------
    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        /// The attitude controller. Owned solely by `attitude_control`, so
        /// it is a local resource — no lock needed, direct access. Its
        /// integrators and filters persist across ticks here.
        controller: AttitudeRateController,
        /// The failsafe state machine. Owned solely by `safety_monitor`, so
        /// it is local — its arm state and latch persist across ticks here.
        safety: SafetyMonitor,
        // Instrumentation pins (§4.7): reserved purely for scope timing.
        // Assign real GPIOs in board.rs when wiring is decided.
        // dbg_control: DbgPin,
        // dbg_sample:  DbgPin,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let rcc = dp.RCC.constrain();

        // WeAct BlackPill: 25 MHz HSE crystal.
        let clocks = rcc.cfgr.use_hse(board::HSE_FREQ_MHZ.MHz()).sysclk(board::SYSCLK_MHZ.MHz()).freeze();
        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());

        let gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output();

        if board::LED_ACTIVE_LOW {
            led.set_high(); // LED off
        } else {
            led.set_low(); // LED off
        }

        defmt::info!("RipWing FC booted");

        // Build the controller with the gains from the simulator's
        // optimizer. These placeholder values MUST be replaced with your
        // actual tuned gains; the axis limits are airframe safety bounds.
        // Because these gains are validated host-side in the control crate's
        // tests, dropping them in here is the whole point of that workflow.
        let axis = PidConfig {
            kp: 0.14,
            ki: 0.02,
            kd: 0.008,
            integral_limit: 0.3,
            output_limit: 1.0,
            d_filter_alpha: 0.2,
        };
        let controller = AttitudeRateController::new(AttitudeGains {
            roll: axis,
            pitch: axis,
            yaw: PidConfig { kd: 0.0, ..axis }, // yaw usually needs no D
        });

        // Failsafe state machine with airframe safety limits. Tune the
        // limits (SafetyLimits::default gives conservative starting values)
        // to the actual vehicle before flight.
        let safety = SafetyMonitor::new(SafetyLimits::default());

        // Kick off every periodic task. Each re-arms itself at its period.
        safety_monitor::spawn().ok();
        attitude_control::spawn().ok();
        sensor_fusion::spawn().ok();
        anomaly::spawn().ok();
        logging::spawn().ok();
        telemetry::spawn().ok();
        heartbeat::spawn().ok();
        diagnostics::spawn().ok();

        (
            Shared {
                state: StateEstimate::default(),
                rate_setpoint: RateSetpoint::default(),
                motor_cmd: MotorCommand::default(),
                // Motors start disabled; only an armed, healthy safety
                // verdict enables them.
                motors_enabled: false,
            },
            Local { led, controller, safety },
        )
    }

    /// Static severity channel: ML writes, control reads on its hot path.
    /// A single atomic, so it lives outside the RTIC resource system.
    static SEVERITY: SeverityFlag = SeverityFlag::new();

    /// Per-task iteration counters, sampled once a second by `diagnostics`.
    /// Bring-up instrumentation: proves each task hits its designed rate.
    static COUNTERS: TaskCounters = TaskCounters::new();

    // =====================================================================
    // PRIORITY 6 — Safety monitor (1 kHz, highest)
    // Failsafe logic lives here and must preempt everything. Per §3.3 its
    // correctness must not depend on any other task being healthy, so it
    // reads state/motor_cmd directly and can override outputs.
    // =====================================================================
    #[task(priority = 6, shared = [state, motors_enabled], local = [safety])]
    async fn safety_monitor(mut cx: safety_monitor::Context) {
        let mut next = Mono::now();
        loop {
            // Snapshot the latest state (brief lock, copy out).
            let state = cx.shared.state.lock(|s| *s);

            // Assemble the inputs. now_us comes from the monotonic clock.
            //
            // NOT YET WIRED (safe placeholders): battery_volts and the RC
            // heartbeat need real sources — an ADC read and the RC-link
            // driver. Until those exist, we feed values that PASS their
            // checks, so the monitor does not false-trip during bring-up.
            // These MUST be replaced with real sensor reads before flight,
            // or the low-battery and RC-loss protections are inert.
            // now_us from the monotonic clock. NOTE: the exact rtic-monotonics
            // time API varies by version — if this line fails to compile, the
            // fix is here. Alternatives seen across 2.x:
            //   Mono::now().duration_since_epoch().to_micros()
            //   Mono::now().ticks()   (ticks are ms at our 1 kHz rate)
            let now_us = Mono::now().duration_since_epoch().to_micros();
            let inputs = SafetyInputs {
                now_us,
                state,
                last_rc_heartbeat_us: now_us, // TODO: real RC link timestamp
                battery_volts: 12.0,          // TODO: real ADC battery read
            };

            // Run the failsafe logic and publish the verdict.
            let permission = cx.local.safety.update(&inputs);
            let enabled = matches!(permission, MotorPermission::Run);
            cx.shared.motors_enabled.lock(|m| *m = enabled);

            tick(&COUNTERS.safety);

            next += CONTROL_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 5 — Attitude control (1 kHz)
    // The hard real-time inner RATE loop. Reads state + rate command, calls
    // the pure no_std control crate, writes motor_cmd. No controller math
    // lives here — it lives in the host-testable control crate (§3.1).
    // =====================================================================
    #[task(priority = 5, shared = [state, rate_setpoint, motor_cmd, motors_enabled], local = [controller])]
    async fn attitude_control(mut cx: attitude_control::Context) {
        // Control timestep in seconds, matching the 1 kHz period.
        const DT: f32 = CONTROL_PERIOD_MS as f32 / 1000.0;
        let mut next = Mono::now();
        loop {
            // dbg_control pin HIGH here (scope measures loop WCET, §4.7).

            // Snapshot shared inputs (lock briefly, copy out, release).
            let state = cx.shared.state.lock(|s| *s);
            let rate_cmd = cx.shared.rate_setpoint.lock(|sp| *sp);
            let enabled = cx.shared.motors_enabled.lock(|m| *m);
            let severity = SEVERITY.get();

            // Always run the controller so its integrators/filters keep
            // tracking, but gate the OUTPUT on the safety verdict. If the
            // monitor has cut motors, emit a zeroed command no matter what
            // the PID produced. Resetting on the disabled->enabled edge is a
            // refinement for when arming is wired up.
            let computed = cx.local.controller.step(&state, &rate_cmd, severity, DT);
            let cmd = if enabled {
                computed
            } else {
                MotorCommand::default() // all motors zero
            };

            cx.shared.motor_cmd.lock(|m| *m = cmd);
            // TODO: emit DShot frame from cmd.

            // dbg_control pin LOW here.
            tick(&COUNTERS.control);
            next += CONTROL_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 4 — Sensor fusion (1 kHz)
    // Consumes filtered IMU samples, runs the estimator, publishes state.
    // Same 1 ms period as control; ranked just below it.
    // =====================================================================
    #[task(priority = 4, shared = [state])]
    async fn sensor_fusion(mut cx: sensor_fusion::Context) {
        let mut next = Mono::now();
        loop {
            // TODO: pull filtered sample, run EKF, produce new estimate.
            let new_state = StateEstimate::default();
            cx.shared.state.lock(|s| *s = new_state);

            tick(&COUNTERS.fusion);
            next += CONTROL_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 3 — Anomaly detection (50 Hz)
    // ML monitor. Observes telemetry, classifies, publishes a severity
    // flag. Never actuates (§7.6). Lower priority than the control chain
    // so it can be preempted freely.
    // =====================================================================
    #[task(priority = 3)]
    async fn anomaly(_cx: anomaly::Context) {
    let mut next = Mono::now();
        loop {
            // TODO: run quantized inference on the recent telemetry window.
            SEVERITY.set(Severity::Nominal);

            tick(&COUNTERS.anomaly);
            next += ANOMALY_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 2 — Logging (100 Hz)
    // Drains the telemetry ring buffer to the flight recorder via DMA.
    // A slow flash erase here must never reach the control loop (§4.1),
    // which preemption structurally guarantees.
    // =====================================================================
    #[task(priority = 2)]
    async fn logging(_cx: logging::Context) {
        let mut next = Mono::now();
        loop {
            // TODO: drain ring buffer -> append-only log over DMA.
            tick(&COUNTERS.logging);
            let mut next = Mono::now();
            next += LOGGING_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 1 — Telemetry downlink (20 Hz, lowest)
    // =====================================================================
    #[task(priority = 1)]
    async fn telemetry(_cx: telemetry::Context) {
        let mut next = Mono::now();
        loop {
            // TODO: pack + send a downlink frame.
            tick(&COUNTERS.telemetry);
            next += TELEMETRY_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // Heartbeat — lowest-value diagnostic blink, shares priority 1.
    // Confirms the scheduler is alive without a scope.
    // =====================================================================
    #[task(priority = 1, local = [led])]
    async fn heartbeat(cx: heartbeat::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(500.millis()).await;
        }
    }

    // =====================================================================
    // Diagnostics — bring-up instrumentation, priority 1 (lowest).
    //
    // Samples the per-task counters once a second and reports them. Because
    // the window is exactly 1 s, each printed count IS that task's measured
    // rate in Hz. Expected, per the §4.3 schedule:
    //
    //   safety 1000, control 1000, fusion 1000,
    //   anomaly 50, logging 100, telemetry 20
    //
    // Sustained shortfall on a high-priority task means the schedule is not
    // being met; shortfall only on low-priority tasks means they are being
    // starved by the ones above (which is the design working as intended if
    // the CPU is genuinely saturated).
    //
    // Remove or feature-gate this task once bring-up is done — it costs a
    // little CPU and flash for no flight benefit.
    // =====================================================================
    #[task(priority = 1)]
    async fn diagnostics(_cx: diagnostics::Context) {
        // Let the system settle before the first sample so startup transients
        // do not show up as a bad first reading.
        Mono::delay(1_000.millis()).await;

        loop {
            let safety = take(&COUNTERS.safety);
            let control = take(&COUNTERS.control);
            let fusion = take(&COUNTERS.fusion);
            let anomaly = take(&COUNTERS.anomaly);
            let logging = take(&COUNTERS.logging);
            let telemetry = take(&COUNTERS.telemetry);

            defmt::info!(
                "rates Hz: safety={} control={} fusion={} anomaly={} logging={} telemetry={}",
                safety,
                control,
                fusion,
                anomaly,
                logging,
                telemetry
            );

            Mono::delay(1_000.millis()).await;
        }
    }
}
