# UAV Flight Controller

Fault-tolerant quadcopter flight controller with edge-deployed anomaly
detection. Firmware in Rust on STM32, scheduled with RTIC.

## Status
Scaffold + RTIC skeleton. Blinks an LED and runs an empty 1 kHz control loop.

## Target
- Bring-up: STM32F411CEUx (BlackPill)
- Planned: STM32H743 (ML compute) on custom PCB

## Toolchain
    rustup target add thumbv7em-none-eabihf
    cargo install probe-rs-tools --locked
    cargo install flip-link

Requires a debug probe (ST-Link V2 clone is fine).

## Build & flash
    cargo build            # cross-compile
    cargo run --release    # flash + stream defmt logs

Update the `--chip` string in `.cargo/config.toml` for your exact part
(`probe-rs chip list | grep STM32`).

## Layout
    src/main.rs      RTIC app: tasks, resources, priorities
    src/board.rs     pin assignments (change here when PCB changes)
    src/drivers/     IMU, barometer
    src/control/     AHRS, PID, motor mixer
    src/telemetry/   flight data logging
    src/anomaly/     edge ML inference (monitors only)
    tools/           Python: log parsing + model training
    docs/            hardware notes

## Design notes
- Control loop is highest priority; ML inference and telemetry are lower
  and may be preempted.
- ML monitors and flags; classical control retains flight-critical actuation.
- Telemetry log schema is shared between `src/telemetry/` and `tools/`.
