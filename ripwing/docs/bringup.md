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

With diagnostics enabled, once a second you should see:

    rates Hz: safety=1000 control=1000 fusion=1000 anomaly=50 logging=100 telemetry=20

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

Without an IMU, ESCs, or an RC link: sensor sampling, the control loop with
real inputs, motor output, and the low-battery / RC-loss failsafe checks
(their inputs are placeholders that always pass — see `safety_monitor`).
