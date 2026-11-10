#![no_std]
#![no_main]

use defmt_rtt as _;

mod anomaly;
mod board;
mod drivers;
mod panic;
mod telemetry;
mod transport;

// NOTE: crate imports for the control and safety logic live INSIDE `mod app`
// below (next to the transport import), because the code that uses them is
// inside that module. A `use` at this outer scope would not be visible there.

// Priority ladder (RTIC: higher number = higher priority). Periods drive
// the assignment; ties at equal period are broken by criticality
// (safety > control > fusion). See case study §4.3.
//
//   Task              Trigger                  Rate     Priority
//   imu_drdy          IMU INT1 (EXTI1)         1 kHz    7  } µs-scale ISRs:
//   imu_dma           SPI1 RX DMA complete     1 kHz    7  } sample timing
//   safety_monitor    SysTick, 1 ms            1 kHz    6
//   attitude_control  spawned by fusion        1 kHz    5
//   sensor_fusion     spawned by imu_dma       1 kHz    4
//   anomaly           SysTick, 20 ms           50 Hz    3
//   logging           SysTick, 10 ms           100 Hz   2
//   telemetry         SysTick, 50 ms           20 Hz    1  (lowest)
//
// The control chain is SENSOR-driven: the IMU samples on its own clock and
// raises data-ready; that edge starts the SPI DMA, the DMA completion hands
// the sample to fusion, and fusion hands the fresh state to control. Each
// stage runs as soon as its input exists, so "sample taken" to "motor
// command ready" is a fixed chain of work rather than a phase relationship
// between two free-running clocks. (Previously the loop woke on SysTick
// while the sensor sampled on its own oscillator; the phase between them
// drifted, so sample age at use wandered anywhere from 0 to a full period.)
//
// The safety monitor deliberately stays on SysTick. Its job includes
// noticing that the sensor chain has STOPPED, and it cannot do that if it
// only runs when the sensor chain runs (§3.3: the failsafe must not depend
// on any other task being healthy).

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI3, SPI4, SPI5, USART1, USART2, USART6])]
mod app {
    use crate::board;
    use crate::drivers::imu::{
        self, Completion, DrdyStamp, FrontEnd, ImuDma, ImuFrame, NoiseReport, StartResult,
        FRAME_LEN,
    };
    use crate::telemetry::logger::{RawLogConsumer, RawLogQueue};
    use crate::transport::{
        tick, ControlTick, ImuStats, MotorCommand, RateSetpoint, Severity, SeverityFlag,
        StateEstimate, TaskCounters,
    };
    use cortex_m::peripheral::DWT;
    // The control and estimation logic are external, host-testable crates
    // (§3.1); firmware calls into them through the Controller and
    // AttitudeEstimator traits.
    use ripwing_control::{gains, AttitudeRateController, Controller};
    use ripwing_estimation::{AttitudeEstimator, ComplementaryConfig, ComplementaryFilter};
    use ripwing_safety::{
        HealthMonitor, MotorPermission, SafetyInputs, SafetyLimits, SafetyMonitor
    };
    use stm32f4xx_hal::watchdog::IndependentWatchdog;
    use rtic_monotonics::systick::prelude::*;
    use stm32f4xx_hal::{
        gpio::{Edge, Output, PushPull, Speed, PC13},
        prelude::*,
        spi::Spi,
    };

    systick_monotonic!(Mono, 1_000); // 1 kHz systick

    // ---- Task periods, one place to change them --------------------------
    // The control chain has no period here: it runs at the IMU's ODR
    // (`imu::ODR_HZ`), paced by data-ready.
    const SAFETY_PERIOD_MS: u32 = 1; // 1 kHz, independent of the sensor
    /// Longest `dt` the controller will integrate over. A longer gap (lost
    /// samples, a debugger halt) is clamped so the PID integrator doesn't
    /// take one giant step on resumption. Ten sample periods.
    const MAX_DT_S: f32 = 10.0 / imu::ODR_HZ;
    const ANOMALY_PERIOD_MS: u32 = 20; // 50 Hz ML monitor
    const LOGGING_PERIOD_MS: u32 = 10; // 100 Hz recorder drain
    const TELEMETRY_PERIOD_MS: u32 = 50; // 20 Hz downlink
    const WATCHDOG_PERIOD_MS: u32 = 50; // 20 Hz watchdog refresh
    const WATCHDOG_TIMEOUT_MS: u32 = 200; // 5 Hz watchdog timeout
    const WATCHDOG_MIN_TICKS: u32 = 25; // 20 Hz watchdog refresh minimum

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
        /// SPI1 + DMA + CS for the IMU. Shared by the data-ready handler
        /// (starts a transfer) and the DMA handler (finishes it). Both run at
        /// priority 7, so the "lock" compiles to nothing.
        imu: ImuDma,
        /// Latest raw-vs-filtered noise figures: written by fusion once a
        /// second, printed by diagnostics.
        noise: NoiseReport,
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

        iwdg: IndependentWatchdog,

        /// IMU INT1 (data-ready). Owned by its EXTI handler, which clears it.
        drdy: board::ImuDrdyPin,
        /// Driver boundary: logs raw, returns filtered. Owned by fusion.
        front_end: FrontEnd,
        /// Attitude estimator. Owned by fusion; swap for the EKF here.
        estimator: ComplementaryFilter,
        /// Drain side of the raw-sample log queue. Owned by logging.
        raw_log: RawLogConsumer,
        /// DWT cycles per second (the real SYSCLK, as configured).
        cycles_per_s: f32,
        /// DWT cycles per microsecond, for diagnostics.
        cycles_per_us: u32,
    }

    #[init(local = [
        // DMA receive buffers: one owned by the RX stream, one spare.
        // 'static because DMA writes to them outside any borrow's lifetime.
        rx_a: [u8; FRAME_LEN] = [0; FRAME_LEN],
        rx_b: [u8; FRAME_LEN] = [0; FRAME_LEN],
        raw_log_q: RawLogQueue = RawLogQueue::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut dp = cx.device;
        let mut core = cx.core;
        let rcc = dp.RCC.constrain();

        // WeAct BlackPill: 25 MHz HSE crystal.
        let clocks = rcc.cfgr.use_hse(board::HSE_FREQ_MHZ.MHz()).sysclk(board::SYSCLK_MHZ.MHz()).freeze();
        let sysclk_hz = clocks.sysclk().to_Hz();
        Mono::start(core.SYST, sysclk_hz);

        // Cycle counter: timestamps data-ready edges to sub-µs, which gives
        // the true sample interval and the end-to-end latency.
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output();

        if board::LED_ACTIVE_LOW {
            led.set_high(); // LED off
        } else {
            led.set_low(); // LED off
        }

        defmt::info!("RipWing FC booted");

        if let Some(rec)= crate::panic::take_panic_record() {
            defmt::error!(
                "previous boot ended in a panic at line {} (consecutive: {})", 
                rec.line, 
                rec.consecutive
            );
        }

        // ---- IMU ---------------------------------------------------------
        // Configure over blocking SPI first, then hand the bus to DMA.
        // Done before the watchdog starts: if the IMU is missing, the panic
        // below should reach the panic handler's halt-after-3 logic rather
        // than be pre-empted by a watchdog reset loop.
        let gpioa = dp.GPIOA.split();
        let mut imu_cs = gpioa.pa4.into_push_pull_output();
        imu_cs.set_high();
        let mut imu_spi = Spi::new(
            dp.SPI1,
            (
                gpioa.pa5.into_alternate().speed(Speed::VeryHigh),
                gpioa.pa6.into_alternate().speed(Speed::VeryHigh),
                gpioa.pa7.into_alternate().speed(Speed::VeryHigh),
            ),
            embedded_hal::spi::MODE_0,
            board::IMU_SPI_MHZ.MHz(),
            &clocks,
        );
        if let Err(e) = imu::configure(&mut imu_spi, &mut imu_cs, sysclk_hz) {
            // No IMU, no flight. Nothing downstream can run meaningfully.
            defmt::panic!("IMU init failed: {}", e);
        }
        defmt::info!("IMU: ICM-42688-P configured, {} Hz ODR", imu::ODR_HZ);
        let imu = ImuDma::new(imu_spi, imu_cs, dp.DMA2, cx.local.rx_a, cx.local.rx_b);

        // Data-ready on INT1 -> EXTI1, rising edge. Enabled last, once the
        // DMA is ready to be kicked; RTIC unmasks the interrupt after init.
        let mut syscfg = dp.SYSCFG.constrain();
        let mut drdy = gpioa.pa1.into_pull_down_input();
        drdy.make_interrupt_source(&mut syscfg);
        drdy.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        drdy.enable_interrupt(&mut dp.EXTI);

        let (raw_log_tx, raw_log) = cx.local.raw_log_q.split();
        let front_end = FrontEnd::new(raw_log_tx);
        let estimator = ComplementaryFilter::new(ComplementaryConfig::default());

        // Independent watchdog: clocked from the LSI
        let mut iwdg = IndependentWatchdog::new(dp.IWDG);
        iwdg.start(WATCHDOG_TIMEOUT_MS.millis());

        // Rate-loop gains from QuadSim's tune of its 7" airframe (roll/pitch
        // and yaw), converted to mixer units for this 1 kHz loop. See
        // ripwing_control::gains for the source, the unit conversion and the
        // test that keeps them in sync. They are for the SIMULATED airframe:
        // re-identify the plant on the real vehicle before trusting them.
        let controller = AttitudeRateController::new(gains::QUADSIM_7IN);

        // Failsafe state machine with airframe safety limits. Tune the
        // limits (SafetyLimits::default gives conservative starting values)
        // to the actual vehicle before flight.
        let safety = SafetyMonitor::new(SafetyLimits::default());

        // Kick off every periodic task. Each re-arms itself at its period.
        // sensor_fusion and attitude_control are NOT spawned here: they are
        // event-driven, spawned once per IMU sample by the chain above them.
        safety_monitor::spawn().ok();
        anomaly::spawn().ok();
        logging::spawn().ok();
        telemetry::spawn().ok();
        heartbeat::spawn().ok();
        diagnostics::spawn().ok();
        watchdog::spawn().ok();

        (
            Shared {
                state: StateEstimate::default(),
                rate_setpoint: RateSetpoint::default(),
                motor_cmd: MotorCommand::default(),
                // Motors start disabled; only an armed, healthy safety
                // verdict enables them.
                motors_enabled: false,
                imu,
                noise: NoiseReport::default(),
            },
            Local {
                led,
                controller,
                safety,
                iwdg,
                drdy,
                front_end,
                estimator,
                raw_log,
                cycles_per_s: sysclk_hz as f32,
                cycles_per_us: sysclk_hz / 1_000_000,
            },
        )
    }

    /// Static severity channel: ML writes, control reads on its hot path.
    /// A single atomic, so it lives outside the RTIC resource system.
    static SEVERITY: SeverityFlag = SeverityFlag::new();

    /// Per-task iteration counters, sampled once a second by `diagnostics`.
    /// Bring-up instrumentation: proves each task hits its designed rate.
    static COUNTERS: TaskCounters = TaskCounters::new();

    /// Sensor-chain counters and latency, sampled by `diagnostics`.
    static IMU_STATS: ImuStats = ImuStats::new();

    // =====================================================================
    // PRIORITY 7 — IMU data-ready (EXTI1)
    // The sensor has a new sample. Timestamp it and start the SPI DMA read.
    // A few hundred cycles; highest priority so the timestamp and the start
    // of the transfer have minimal jitter relative to the actual sample.
    // =====================================================================
    #[task(binds = EXTI1, priority = 7, local = [drdy], shared = [imu])]
    fn imu_drdy(mut cx: imu_drdy::Context) {
        // Capture time first: everything after this is latency we measure.
        let cycles = DWT::cycle_count();
        let timestamp_us = Mono::now().duration_since_epoch().to_micros();
        cx.local.drdy.clear_interrupt_pending_bit();
        tick(&IMU_STATS.drdy);

        let stamp = DrdyStamp { cycles, timestamp_us };
        match cx.shared.imu.lock(|imu| imu.start(stamp)) {
            StartResult::Started => {}
            StartResult::Busy => tick(&IMU_STATS.dma_busy),
            StartResult::Recovered => tick(&IMU_STATS.bus_errors),
        }
    }

    // =====================================================================
    // PRIORITY 7 — IMU DMA complete (DMA2 stream 0 = SPI1 RX)
    // The burst has landed. Parse it and hand the sample to fusion. The RX
    // DMA is re-armed on the other buffer before this returns, so the next
    // sample's transfer can run while fusion/control compute this one.
    // =====================================================================
    #[task(binds = DMA2_STREAM0, priority = 7, shared = [imu])]
    fn imu_dma(mut cx: imu_dma::Context) {
        match cx.shared.imu.lock(|imu| imu.complete()) {
            Completion::Sample(frame) => {
                // spawn fails only if fusion is still running on the
                // previous sample: compute has overrun the sample period.
                if sensor_fusion::spawn(frame).is_err() {
                    tick(&IMU_STATS.chain_overruns);
                }
            }
            Completion::Rejected => tick(&IMU_STATS.rejected),
            Completion::BusError => tick(&IMU_STATS.bus_errors),
            Completion::Spurious => {}
        }
    }

    #[task(priority = 6, local = [iwdg, health: HealthMonitor<3> = HealthMonitor::new([WATCHDOG_MIN_TICKS; 3])])]
    async fn watchdog(cx: watchdog::Context) {
        let mut next = Mono::now();
        loop {
            let counts = COUNTERS.snapshot();
            let control_chain = [counts[0], counts[1], counts[2]]; // safety, control, fusion
            
            match cx.local.health.check(&control_chain) {
                Ok(()) => cx.local.iwdg.feed(),
                Err(stall) => {
                    defmt::error!(
                        "task {} stalled: {} ticks, needd {} - withholding watchdog pet", 
                        stall.index,
                        stall.observed,
                        stall.required
                    );
                }
            }

            next += WATCHDOG_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 6 — Safety monitor (1 kHz, highest)
    // Failsafe logic lives here and must preempt everything. Per §3.3 its
    // correctness must not depend on any other task being healthy, so it
    // reads state/motor_cmd directly and can override outputs.
    //
    // Stays on SysTick, NOT on data-ready: if the IMU or its bus dies, the
    // sensor chain stops, and this is the task that must notice (via the
    // state timestamp going stale) and cut the motors.
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

            next += SAFETY_PERIOD_MS.millis();
            Mono::delay_until(next).await;
        }
    }

    // =====================================================================
    // PRIORITY 5 — Attitude control (per IMU sample)
    // The hard real-time inner RATE loop. Spawned by fusion the moment a
    // fresh estimate exists, so it always consumes the newest sample — one
    // sample, once. Reads state + rate command, calls the pure no_std
    // control crate, writes motor_cmd. No controller math lives here (§3.1).
    // =====================================================================
    #[task(priority = 5, shared = [state, rate_setpoint, motor_cmd, motors_enabled], local = [controller])]
    async fn attitude_control(mut cx: attitude_control::Context, input: ControlTick) {
        // dbg_control pin HIGH here (scope measures loop WCET, §4.7).

        // Snapshot shared inputs (lock briefly, copy out, release).
        let state = cx.shared.state.lock(|s| *s);
        let rate_cmd = cx.shared.rate_setpoint.lock(|sp| *sp);
        let enabled = cx.shared.motors_enabled.lock(|m| *m);
        let severity = SEVERITY.get();

        // Always run the controller so its integrators/filters keep
        // tracking, but gate the OUTPUT on the safety verdict. If the
        // monitor has cut motors, emit a zeroed command no matter what
        // the PID computed. Resetting on the disabled->enabled edge is a
        // refinement for when arming is wired up.
        //
        // dt is the measured interval between this sample and the last, so
        // the PID's integral and derivative are scaled by real time even if
        // the IMU's oscillator is off-nominal or a sample was lost.
        let computed = cx.local.controller.step(&state, &rate_cmd, severity, input.dt);
        let cmd = if enabled {
            computed
        } else {
            MotorCommand::default() // all motors zero
        };

        cx.shared.motor_cmd.lock(|m| *m = cmd);
        // TODO: emit DShot frame from cmd.

        // End of the chain: data-ready edge -> motor command ready.
        IMU_STATS.record_latency(DWT::cycle_count().wrapping_sub(input.drdy_cycles));

        // dbg_control pin LOW here.
        tick(&COUNTERS.control);
    }

    // =====================================================================
    // PRIORITY 4 — Sensor fusion (per IMU sample)
    // Spawned by the DMA-complete handler with the raw sample. Runs the
    // driver boundary (log raw, filter), the attitude estimator, publishes
    // state, and spawns control. Ranked below control: control preempts it
    // the instant it is spawned, which is exactly the order we want.
    // =====================================================================
    #[task(
        priority = 4,
        shared = [state, noise],
        local = [front_end, estimator, cycles_per_s, last_drdy: Option<u32> = None]
    )]
    async fn sensor_fusion(mut cx: sensor_fusion::Context, frame: ImuFrame) {
        // Sample interval from the sensor's own edges. First sample: nominal.
        let dt = match cx.local.last_drdy.replace(frame.stamp.cycles) {
            Some(prev) => frame.stamp.cycles.wrapping_sub(prev) as f32 / *cx.local.cycles_per_s,
            None => 1.0 / imu::ODR_HZ,
        }
        .min(MAX_DT_S);

        // Driver boundary: raw is logged, filtered goes downstream.
        let (filtered, logged) = cx.local.front_end.condition(&frame.raw);
        if !logged {
            tick(&IMU_STATS.log_drops);
        }
        if let Some(report) = cx.local.front_end.take_noise_report() {
            cx.shared.noise.lock(|n| *n = report);
        }

        let estimate = cx.local.estimator.update(&filtered, dt);
        // Publish only a real estimate. Until the estimator has seeded from
        // a believable accel reading, the shared state stays stale, and the
        // safety monitor's staleness check keeps the vehicle unarmable.
        if cx.local.estimator.is_initialized() {
            cx.shared.state.lock(|s| *s = estimate);
        }

        tick(&COUNTERS.fusion);

        // Control runs for every sample regardless (its output is gated by
        // the safety verdict), so its liveness reflects the sensor chain's.
        let next = ControlTick {
            dt,
            drdy_cycles: frame.stamp.cycles,
        };
        if attitude_control::spawn(next).is_err() {
            tick(&IMU_STATS.chain_overruns);
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
    #[task(priority = 2, local = [raw_log])]
    async fn logging(cx: logging::Context) {
        let mut next = Mono::now();
        loop {
            // Drain raw IMU samples captured at the driver boundary.
            // TODO: append to the flight recorder over DMA. Until then they
            // end here (visible with DEFMT_LOG=trace).
            let mut drained = 0u32;
            while let Some(s) = cx.local.raw_log.dequeue() {
                defmt::trace!("raw t={} g={} a={}", s.timestamp_us, s.gyro, s.accel);
                drained += 1;
            }
            IMU_STATS
                .logged
                .fetch_add(drained, core::sync::atomic::Ordering::Relaxed);

            tick(&COUNTERS.logging);
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
    // Once a second, reports:
    //
    // 1. Per-task rates. The window is exactly 1 s, so each count IS that
    //    task's rate in Hz. Expected:
    //
    //      safety 1000 (SysTick), control ≈ fusion ≈ IMU ODR,
    //      anomaly 50, logging 100, telemetry 20
    //
    //    control/fusion now follow the IMU's oscillator, not ours: expect
    //    ~1000 give or take the sensor's clock tolerance (a percent or so),
    //    and expect them to equal each other and the `drdy` count. Control
    //    below drdy means samples are being lost somewhere in the chain;
    //    the error counters on the next line say where.
    //
    // 2. Sensor chain health: latency from data-ready edge to motor command
    //    ready (worst and most recent over the last second), and the
    //    cumulative error counters.
    //
    // 3. Noise: raw vs filtered std dev per axis over the last second. This
    //    is the measurement the filter window choice in drivers/imu.rs is
    //    checked against. Read it at rest, then with motors spinning.
    //
    // Remove or feature-gate this task once bring-up is done.
    // =====================================================================
    #[task(
        priority = 1,
        shared = [noise],
        local = [cycles_per_us, last: [u32; 6] = [0; 6], last_drdy: u32 = 0]
    )]
    async fn diagnostics(mut cx: diagnostics::Context) {
        use crate::transport::read;
        const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

        let mut next = Mono::now() + 1_000u32.millis();
        Mono::delay_until(next).await;

        loop {
            let now = COUNTERS.snapshot();
            let prev = *cx.local.last;
            *cx.local.last = now;

            let d: [u32; 6] = core::array::from_fn(|i| now[i].wrapping_sub(prev[i]));

            let drdy = read(&IMU_STATS.drdy);
            let drdy_rate = drdy.wrapping_sub(*cx.local.last_drdy);
            *cx.local.last_drdy = drdy;

            defmt::info!(
                "rates Hz: drdy={} safety={} control={} fusion={} anomaly={} logging={} telemetry={}",
                drdy_rate,
                d[0],
                d[1],
                d[2],
                d[3],
                d[4],
                d[5]
            );

            let (lat_max, lat_last) = IMU_STATS.take_latency();
            let cpu = *cx.local.cycles_per_us;
            defmt::info!(
                "imu: latency us max={} last={} | dma_busy={} overrun={} rejected={} bus_err={} log_drop={} logged={}",
                lat_max / cpu,
                lat_last / cpu,
                read(&IMU_STATS.dma_busy),
                read(&IMU_STATS.chain_overruns),
                read(&IMU_STATS.rejected),
                read(&IMU_STATS.bus_errors),
                read(&IMU_STATS.log_drops),
                read(&IMU_STATS.logged)
            );

            let n = cx.shared.noise.lock(|n| *n);
            if n.samples > 0 {
                let deg = |v: [f32; 3]| v.map(|x| x * RAD_TO_DEG);
                defmt::info!(
                    "noise std: gyro deg/s raw={} filt={} | accel m/s2 raw={} filt={}",
                    deg(n.gyro_raw),
                    deg(n.gyro_filt),
                    n.accel_raw,
                    n.accel_filt
                );
            }

            next += 1_000u32.millis();
            Mono::delay_until(next).await;
        }
    }
}
