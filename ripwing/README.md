# RipWing

Fault-tolerant quadcopter flight controller with edge-deployed anomaly
detection. Firmware in Rust on STM32, scheduled with RTIC. Control logic is
pure and host-testable, validated against a Python simulator before it
reaches hardware.

## Workspace layout

    crates/common     Shared data types. no_std, no HAL. Host + target.
    crates/control    Pure control math (PID, mixer, Controller trait).
                      no_std, host-testable. Depends only on `common`.
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
- Control crate: rate-loop PID with anti-windup, derivative-on-measurement
  filtering, quad-X mixer, and a host test harness. Gains from the
  simulator's optimizer drop into `firmware` init.
- Safety crate: failsafe state machine (Disarmed/Armed/Failsafe, latched)
  with staleness, attitude, rate, RC-link, and battery checks. 19 host
  tests. Wired into the priority-6 task; gates the control output.
  Battery and RC-link inputs are placeholders until those drivers exist.
- Next: port the full simulator plant into the control tests for an
  apples-to-apples match against the Python step responses; IMU driver
  bring-up when hardware arrives.

## Design notes

- Control loop is highest priority after the safety monitor; ML inference
  and telemetry are lower and may be preempted.
- ML monitors and flags; classical control retains flight-critical
  actuation. The severity flag is a single lock-free atomic the control
  loop reads each cycle.
- The Controller trait is the firmware/control seam: swap in an MPC later
  by implementing the trait, without touching the firmware.
