# RipWing

Fault-tolerant quadcopter flight controller with edge-deployed anomaly
detection. Firmware in Rust on STM32, scheduled with RTIC. Control logic is
pure and host-testable, validated against a Python simulator before it
reaches hardware.

## Workspace layout

    crates/common     Shared data types. no_std, no HAL. Host + target.
    crates/control    Pure control math (PID, mixer, Controller trait).
                      no_std, host-testable. Depends only on `common`.
    crates/filter     Sensor conditioning: median-of-3 spike rejection into
                      an N-tap moving average. no_std, host-testable.
    crates/estimation Attitude estimation: quaternion complementary filter
                      behind an AttitudeEstimator trait (EKF later).
                      no_std, host-testable.
    crates/safety     Failsafe state machine and safety checks. no_std,
                      host-testable. Depends only on `common`.
    firmware          RTIC binary for the STM32F411. Depends on the crates,
                      adds the HAL, drivers, scheduler, and DShot output.

The dependency graph enforces the layering (case study §3.1): `control`
cannot reach a HAL or RTIC type because it does not depend on them, so
"keep control logic host-testable" is a structural fact, not a discipline.

## Build & test

Host-testable control logic (no hardware needed):

    cargo test -p ripwing-control
    cargo test -p ripwing-filter
    cargo test -p ripwing-estimation
    cargo test -p ripwing-safety
    cargo test -p ripwing-common

Firmware (cross-compiles to ARM; needs the target installed):

    cargo fw          # alias: build -p ripwing-firmware --target thumbv7em-none-eabihf
    cargo rfw         # alias: run   (flash + stream defmt logs) — needs a probe

Toolchain setup:

    rustup target add thumbv7em-none-eabihf
    cargo install probe-rs-tools --locked
    cargo install flip-link

## Target

- Bring-up: STM32F411CEUx (WeAct BlackPill), 25 MHz HSE.
- Planned: STM32H743 (ML compute headroom) on a custom PCB.

## Status

- Scheduler: six-task RTIC graph at rate-monotonic priorities. Done.
- Control crate: two-stage cascade. The outer attitude loop (P per axis,
  shortest-way heading error, rate-demand clamp, tilt-compensated thrust)
  feeds the rate-loop PIDs (anti-windup, derivative-on-measurement
  filtering), then the quad-X mixer (body FLU frame). The firmware runs
  both stages per IMU sample from an `AttitudeSetpoint`, which stays level
  at zero thrust until the RC driver writes it. Heading hold follows the
  estimator's gyro-only yaw, so it drifts with gyro bias until there is a
  magnetometer or EKF. Cross-checked against QuadSim:
  a Rust port of the 10-DOF plant's rotational dynamics matches the Python
  to ~1e-15, and the firmware controller reproduces QuadSim's tuned step
  responses on it, roll, pitch and yaw, at 200 Hz and at the firmware's
  1 kHz, and large steps QuadSim never tested settle without overshoot
  (`crates/control/tests/quadsim_*.rs`). The firmware flies QuadSim's
  tuned gains (`crates/control/src/gains.rs`); a test fails if they drift
  from the tune. Regenerate the reference data with
  `python -m QuadSim.analysis.export_step_references` from the repo root
  after changing the Python plant, controller or gains.
- Safety crate: failsafe state machine (Disarmed/Armed/Failsafe, latched)
  with staleness, attitude, rate, RC-link, and battery checks. 19 host
  tests. Wired into the priority-6 task; gates the control output.
  Battery and RC-link inputs are placeholders until those drivers exist.
- Filter crate: median-then-average sensor conditioning with documented
  lag-versus-noise tradeoff, plus a noise meter. Wired in at the IMU
  driver boundary (gyro N=2, accel N=8): raw samples are logged, filtered
  samples go downstream.
- IMU: ICM-42688-P on SPI1 with DMA. The control chain is sensor-driven:
  data-ready -> DMA burst read -> fusion -> control, each stage spawned by
  the previous one. Transfers overlap computation. The safety monitor stays
  on SysTick so it can detect a dead sensor chain.
- Estimation crate: quaternion complementary filter (τ = 1 s, accel norm
  gate, seeded from accel at boot), 18 host tests. Yaw is gyro-only until
  a magnetometer or the EKF.
- Next: bench-verify the IMU path on hardware and measure noise with motors
  spinning; RC driver writing the attitude setpoint (and a rate/acro mode
  that bypasses the outer loop); EKF.

## Design notes

- Control loop is highest priority after the safety monitor; ML inference
  and telemetry are lower and may be preempted.
- ML monitors and flags; classical control retains flight-critical
  actuation. The severity flag is a single lock-free atomic the control
  loop reads each cycle.
- The Controller trait is the firmware/control seam: swap in an MPC later
  by implementing the trait, without touching the firmware.
