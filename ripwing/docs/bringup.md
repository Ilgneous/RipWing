# Hardware Bring-Up Checklist

First-power procedure for the WeAct BlackPill (STM32F411CEU6) + ST-Link V2.
Work top to bottom; each step assumes the previous one passed.

## 0. Wiring

Four wires, ST-Link to BlackPill:

| ST-Link | BlackPill | Note                   |
|---------|-----------|------------------------|
| SWDIO   | PA13      | labeled DIO / SWDIO    |
| SWCLK   | PA14      | labeled CLK / SWCLK    |
| GND     | GND       |                        |
| 3.3V    | 3V3       | do NOT also use USB 5V |

Some ST-Link clones mislabel SWDIO/SWCLK. If detection fails, swap those two
before assuming the probe is dead.

## 1. Probe sees the chip

    probe-rs list      # ST-Link appears
    probe-rs info      # identifies the F411 core

PASS: the core is identified.
FAIL: check wiring, then try updating the clone's firmware with ST's
"ST-Link Upgrade" utility.

## 2. First flash

    cargo rfw          # build + flash + stream defmt

PASS: `RipWing FC booted` prints AND the onboard LED blinks at 1 Hz.

FAIL — nothing prints, no blink, no error:
  Likely the HSE crystal. `.freeze()` blocks forever waiting for an HSE that
  never stabilizes. Confirm by switching to the internal oscillator: in
  `init`, replace
      rcc.cfgr.use_hse(25.MHz()).sysclk(96.MHz()).freeze()
  with
      rcc.cfgr.sysclk(84.MHz()).freeze()
  If it boots on HSI, the crystal (or its frequency) is the problem. See
  `board.rs::HSE_FREQ_MHZ`.

FAIL — LED blinks at the wrong rate:
  Clock misconfiguration. The ratio of observed to expected (1 Hz) tells you
  how far off the actual SYSCLK is from the assumed 96 MHz.

## 3. Schedule verification

Requires the IMU wired per `docs/hardware.md`: the control chain is paced
by the IMU's data-ready line, so without an IMU, init panics with
`IMU init failed: WhoAmI(..)` (0x00 / 0xFF there means wiring, not a bad
chip). After three consecutive panics the board halts instead of resetting.

With diagnostics enabled, once a second you should see:

    rates Hz: drdy=1000 safety=1000 control=1000 fusion=1000 anomaly=50 logging=100 telemetry=20
    imu: latency us max=.. last=.. | dma_busy=0 overrun=0 rejected=N bus_err=0 log_drop=0 logged=..
    noise std: gyro deg/s raw=[..] filt=[..] | accel m/s2 raw=[..] filt=[..]

`drdy`, `control` and `fusion` follow the IMU's oscillator, not the MCU's,
so they read ~1000 ± the sensor's clock tolerance — but they must equal
each other. `control` below `drdy` means samples are being lost; the
`imu:` line says where (`dma_busy`: SPI too slow; `overrun`: compute longer
than a sample period; `bus_err`: DMA faults). `rejected` counts a handful of
invalid frames while the gyro starts up (~30 ms); it must stop growing after
boot. A single "task stalled" watchdog message at boot is from that same
startup window.

`latency` is data-ready edge to motor command ready. It should be small
(tens of µs) and, more importantly, STABLE: max close to last. That
stability is what the switch from SysTick to data-ready buys.

The `noise` line is the input to the filter window choice (see the
`GYRO_WINDOW`/`ACCEL_WINDOW` comment in `firmware/src/drivers/imu.rs`).
Record it at rest on the bench, then again with motors spinning.

These are the §4.3 design rates, measured. Interpretation:

- All at expected values: the rate-monotonic schedule is being met.
- High-priority tasks short (safety/control/fusion below ~1000): the system
  cannot meet its deadlines. Something is taking too long. Investigate before
  adding any more work to the hot path.
- Only low-priority tasks short (logging/telemetry): they are being starved
  by higher-priority work. Expected if the CPU is saturated; this is the
  design working as intended, not a fault.
- A task at 0: it is not running at all. Check it was spawned in `init`.

Counts will be approximate (±1 or so) because the sampling window is not
perfectly aligned to task periods. Sustained deviation is what matters.

## 4. Scope instrumentation (optional, needs an oscilloscope)

Wire a free GPIO (PB0 suggested) and toggle it high at the start of the
control loop body, low at the end. Scope it to read directly:

- period between rising edges  -> actual loop rate
- pulse width                  -> execution time (WCET when you take the max)

Feed measured WCET into the rate-monotonic schedulability check to turn the
§4.3 priority assignment from an assumption into a proof.

## What cannot be tested yet

Without ESCs or an RC link: motor output, closed-loop flight, and the
RC-loss failsafe check (its input is a placeholder that always passes; see
`safety_monitor`). The battery input is real: wire a 10k/1k divider from
the pack to PB0, set `VBAT_TRIM` against a multimeter, and check that
diagnostics reports the right cell count. The current reading needs the
ESC's current pin on PB1 and its scale in `board::CURRENT_SCALE`. The IMU axis mapping
(`board::IMU_AXIS`/`IMU_SIGN`) must be set for the airframe before the
control loop is closed: a wrong sign there inverts a feedback loop.
