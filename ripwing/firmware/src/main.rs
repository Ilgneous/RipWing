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
    use crate::transport::{
        MotorCommand, Setpoint, Severity, SeverityFlag, StateEstimate,
    };
    // The control logic is an external, host-testable crate (§3.1); firmware
    // calls into it through the Controller trait.
    use ripwing_control::{AttitudeGains, AttitudeRateController, Controller, PidConfig};
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
        /// Current setpoint: written by RC/outer loop, read by control.
        setpoint: Setpoint,
        /// Latest mixer output: written by control, read by safety monitor.
        motor_cmd: MotorCommand,
    }

    // ---- Local resources (owned by exactly one task) ---------------------
    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
        /// The attitude controller. Owned solely by `attitude_control`, so
        /// it is a local resource — no lock needed, direct access. Its
        /// integrators and filters persist across ticks here.
        controller: AttitudeRateController,
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
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(96.MHz()).freeze();
        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());

        let gpioc = dp.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output();

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

        // Kick off every periodic task. Each re-arms itself at its period.
        safety_monitor::spawn().ok();
        attitude_control::spawn().ok();
        sensor_fusion::spawn().ok();
        anomaly::spawn().ok();
        logging::spawn().ok();
        telemetry::spawn().ok();
        heartbeat::spawn().ok();

        (
            Shared {
                state: StateEstimate::default(),
                setpoint: Setpoint::default(),
                motor_cmd: MotorCommand::default(),
            },
            Local { led, controller },
        )
    }

    /// Static severity channel: ML writes, control reads on its hot path.
    /// A single atomic, so it lives outside the RTIC resource system.
    static SEVERITY: SeverityFlag = SeverityFlag::new();

    // =====================================================================
    // PRIORITY 6 — Safety monitor (1 kHz, highest)
    // Failsafe logic lives here and must preempt everything. Per §3.3 its
    // correctness must not depend on any other task being healthy, so it
    // reads state/motor_cmd directly and can override outputs.
    // =====================================================================
    #[task(priority = 6, shared = [state, motor_cmd])]
    async fn safety_monitor(mut cx: safety_monitor::Context) {
        loop {
            // TODO (feat: safety monitor): staleness check on
            //   state.timestamp_us, attitude limits, battery, RC link loss
            //   -> cut motors / enter failsafe. Empty shell for now.
            let _sev = SEVERITY.get();
            cx.shared.state.lock(|_s| { /* inspect */ });
            cx.shared.motor_cmd.lock(|_m| { /* override if unsafe */ });

            Mono::delay(CONTROL_PERIOD_MS.millis()).await;
        }
    }

    // =====================================================================
    // PRIORITY 5 — Attitude control (1 kHz)
    // The hard real-time loop. Reads state + setpoint, calls the pure
    // no_std control crate, writes motor_cmd. No controller math lives
    // here — it lives in the host-testable control crate (§3.1).
    // =====================================================================
    #[task(priority = 5, shared = [state, setpoint, motor_cmd], local = [controller])]
    async fn attitude_control(mut cx: attitude_control::Context) {
        // Control timestep in seconds, matching the 1 kHz period.
        const DT: f32 = CONTROL_PERIOD_MS as f32 / 1000.0;

        loop {
            // dbg_control pin HIGH here (scope measures loop WCET, §4.7).

            // Snapshot shared inputs (lock briefly, copy out, release).
            let state = cx.shared.state.lock(|s| *s);
            let setpoint = cx.shared.setpoint.lock(|sp| *sp);
            let severity = SEVERITY.get();

            // Call into the pure, host-tested control crate. No control math
            // lives in firmware — this is just the seam.
            let cmd = cx.local.controller.step(&state, &setpoint, severity, DT);

            cx.shared.motor_cmd.lock(|m| *m = cmd);
            // TODO: emit DShot frame from cmd.

            // dbg_control pin LOW here.
            Mono::delay(CONTROL_PERIOD_MS.millis()).await;
        }
    }

    // =====================================================================
    // PRIORITY 4 — Sensor fusion (1 kHz)
    // Consumes filtered IMU samples, runs the estimator, publishes state.
    // Same 1 ms period as control; ranked just below it.
    // =====================================================================
    #[task(priority = 4, shared = [state])]
    async fn sensor_fusion(mut cx: sensor_fusion::Context) {
        loop {
            // TODO: pull filtered sample, run EKF, produce new estimate.
            let new_state = StateEstimate::default();
            cx.shared.state.lock(|s| *s = new_state);

            Mono::delay(CONTROL_PERIOD_MS.millis()).await;
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
        loop {
            // TODO: run quantized inference on the recent telemetry window.
            SEVERITY.set(Severity::Nominal);

            Mono::delay(ANOMALY_PERIOD_MS.millis()).await;
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
        loop {
            // TODO: drain ring buffer -> append-only log over DMA.
            Mono::delay(LOGGING_PERIOD_MS.millis()).await;
        }
    }

    // =====================================================================
    // PRIORITY 1 — Telemetry downlink (20 Hz, lowest)
    // =====================================================================
    #[task(priority = 1)]
    async fn telemetry(_cx: telemetry::Context) {
        loop {
            // TODO: pack + send a downlink frame.
            Mono::delay(TELEMETRY_PERIOD_MS.millis()).await;
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
}
