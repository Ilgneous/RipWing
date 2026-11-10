//! Secondary sensors: BMI088 (IMU2), BMP581 barometer, MMC5983MA
//! magnetometer. Firmware glue around the `ripwing-sensors` chip drivers.
//!
//! The chip logic (register maps, init sequences, unit conversion) lives in
//! `crates/sensors` and is tested on the host. This file only does what is
//! board-specific: the buses, the delay, the axis maps, the polling
//! schedule, and health tracking.
//!
//! # Optional by design
//!
//! None of these sensors is needed to fly the inner loops; the ICM-42688-P
//! is. So each is probed at boot (chip ID), and a missing or failing one is
//! logged and skipped instead of stopping the boot. On the BlackPill they
//! are breakouts that may simply not be connected.
//!
//! # Polling
//!
//! One task polls all three at 500 Hz: IMU2 every tick, the magnetometer
//! every 5th (100 Hz, its continuous rate) and the barometer every 10th
//! (50 Hz, its ODR). Interrupts are wired on the PCB but not needed at these
//! rates, and on the BlackPill the baro's INT pin would collide with the
//! ICM's EXTI line.
//!
//! # I²C hazards
//!
//! The F4 HAL's blocking I²C has no timeouts: with SDA or SCL held low it
//! spins forever waiting for START. Two defences:
//!
//! * [`recover_i2c`] checks the lines before the peripheral is set up and
//!   clocks out a slave stuck mid-byte (the standard 9-pulse recovery). If
//!   the bus is still stuck, no I²C sensor is used at all.
//! * After [`MAX_CONSECUTIVE_ERRORS`] failed reads a sensor is marked failed
//!   and no longer polled, so a dead device stops costing bus time.
//!
//! A slave that clamps the bus *during* a transfer can still hang this task.
//! It runs at priority 2, so control, fusion and safety keep running, but
//! logging (also priority 2) and everything below would stop.

use crate::board::{self, Imu2AccelCsPin, Imu2GyroCsPin};
use embedded_hal::delay::DelayNs;
use ripwing_common::{BaroSample, ImuSample, MagSample};
use ripwing_sensors::{
    bmi088::{self, Bmi088},
    bmp581::{self, Bmp581},
    mmc5983ma::{self, Mmc5983ma},
    remap, Error,
};
use stm32f4xx_hal::{
    gpio::{OpenDrain, Output, PB8, PB9},
    i2c::I2c,
    pac::{I2C1, SPI2},
    spi::Spi,
};

pub type Imu2 = Bmi088<Spi<SPI2>, Imu2AccelCsPin, Imu2GyroCsPin>;

/// Consecutive read failures before a sensor is given up on.
pub const MAX_CONSECUTIVE_ERRORS: u16 = 50;

/// Busy-wait delay from the core clock, for init only (the drivers need
/// DelayNs; SysTick belongs to the RTIC monotonic).
pub struct CycleDelay {
    pub cycles_per_us: u32,
}

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * self.cycles_per_us as u64).div_ceil(1000);
        cortex_m::asm::delay(cycles.clamp(1, u32::MAX as u64) as u32);
    }
}

/// Check the I²C lines and free them if a slave is holding SDA low (it was
/// interrupted mid-byte, e.g. by a reset). Up to nine SCL pulses let it
/// finish the byte, then a STOP releases the bus. Returns true if both
/// lines idle high afterwards.
pub fn recover_i2c(
    scl: &mut PB8<Output<OpenDrain>>,
    sda: &mut PB9<Output<OpenDrain>>,
    cycles_per_us: u32,
) -> bool {
    let half_period = 5 * cycles_per_us; // ~100 kHz
    scl.set_high();
    sda.set_high();
    cortex_m::asm::delay(half_period);
    for _ in 0..9 {
        if sda.is_high() {
            break;
        }
        scl.set_low();
        cortex_m::asm::delay(half_period);
        scl.set_high();
        cortex_m::asm::delay(half_period);
    }
    // STOP: SDA rises while SCL is high.
    sda.set_low();
    cortex_m::asm::delay(half_period);
    scl.set_high();
    cortex_m::asm::delay(half_period);
    sda.set_high();
    cortex_m::asm::delay(half_period);
    scl.is_high() && sda.is_high()
}

/// Health of one sensor, as shown by `diagnostics`.
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct Health {
    /// Found and configured at boot.
    pub present: bool,
    /// Given up after too many consecutive errors.
    pub failed: bool,
    pub reads: u32,
    pub errors: u32,
    pub consecutive: u16,
    pub last_error: Option<Error>,
}

impl Health {
    fn active(&self) -> bool {
        self.present && !self.failed
    }

    /// Record one read. Returns the value on success.
    fn record<T>(&mut self, name: &str, r: Result<T, Error>) -> Option<T> {
        self.reads = self.reads.wrapping_add(1);
        match r {
            Ok(v) => {
                self.consecutive = 0;
                Some(v)
            }
            Err(e) => {
                self.errors = self.errors.wrapping_add(1);
                self.consecutive = self.consecutive.saturating_add(1);
                self.last_error = Some(e);
                if self.consecutive >= MAX_CONSECUTIVE_ERRORS {
                    self.failed = true;
                    defmt::error!("{}: {} consecutive errors (last {}), no longer polled", name, self.consecutive, e);
                }
                None
            }
        }
    }
}

/// Latest readings, body frame. Shared with `diagnostics`; the estimator
/// will consume it when the mag and baro are fused.
#[derive(Clone, Copy, Debug, Default)]
pub struct Readings {
    pub imu2: Option<ImuSample>,
    pub baro: Option<BaroSample>,
    pub mag: Option<MagSample>,
    /// [imu2, baro, mag]
    pub health: [Health; 3],
}

pub struct SecondarySensors {
    i2c: Option<I2c<I2C1>>,
    imu2: Option<Imu2>,
    baro: Bmp581,
    mag: Mmc5983ma,
    health: [Health; 3],
    readings: Readings,
    tick: u32,
}

const IMU2: usize = 0;
const BARO: usize = 1;
const MAG: usize = 2;

impl SecondarySensors {
    /// Probe and configure every sensor that answers. `i2c` is `None` when
    /// [`recover_i2c`] could not free the bus. Blocks ~90 ms when all three
    /// are present (resets and the mag's offset calibration).
    pub fn init(i2c: Option<I2c<I2C1>>, mut imu2: Imu2, delay: &mut CycleDelay) -> Self {
        let mut health = [Health::default(); 3];

        let imu2 = match imu2.init(delay, &bmi088::Config::default()) {
            Ok(()) => {
                defmt::info!("IMU2: BMI088 configured (accel 800 Hz ±24 g, gyro 1 kHz ±2000 dps)");
                health[IMU2].present = true;
                Some(imu2)
            }
            Err(e) => {
                defmt::warn!("IMU2: BMI088 not available ({}), skipped", e);
                None
            }
        };

        let mut baro = Bmp581::new(bmp581::ADDR_SDO_LOW);
        let mut mag = Mmc5983ma::new();
        let mut i2c = i2c;
        match i2c.as_mut() {
            Some(bus) => {
                match baro.init(bus, delay, &bmp581::Config::default()) {
                    Ok(()) => {
                        defmt::info!("baro: BMP581 configured (50 Hz, OSR x16, IIR 3)");
                        health[BARO].present = true;
                    }
                    Err(e) => defmt::warn!("baro: BMP581 not available ({}), skipped", e),
                }
                match mag.init(bus, delay, &mmc5983ma::Config::default()) {
                    Ok(()) => {
                        defmt::info!("mag: MMC5983MA configured (100 Hz), bridge offset uT {}", mag.offset_ut());
                        health[MAG].present = true;
                    }
                    Err(e) => defmt::warn!("mag: MMC5983MA not available ({}), skipped", e),
                }
            }
            None => defmt::error!("I2C1 stuck low after recovery: baro and mag skipped"),
        }
        // No I²C sensor answered: drop the bus so polling never touches it.
        if !health[BARO].present && !health[MAG].present {
            i2c = None;
        }

        Self { i2c, imu2, baro, mag, health, readings: Readings { health, ..Default::default() }, tick: 0 }
    }

    /// One 500 Hz poll. `now_us` stamps whatever was read.
    pub fn poll(&mut self, now_us: u32) -> Readings {
        let t = self.tick;
        self.tick = t.wrapping_add(1);

        if self.health[IMU2].active() {
            if let Some(imu) = self.imu2.as_mut() {
                if let Some(r) = self.health[IMU2].record("IMU2", imu.read()) {
                    self.readings.imu2 = Some(ImuSample {
                        gyro: remap(r.gyro_rps, board::IMU2_AXIS, board::IMU2_SIGN),
                        accel: remap(r.accel_mps2, board::IMU2_AXIS, board::IMU2_SIGN),
                        timestamp_us: now_us,
                    });
                }
            }
        }

        if let Some(bus) = self.i2c.as_mut() {
            if t.is_multiple_of(5) && self.health[MAG].active() {
                if let Some(f) = self.health[MAG].record("mag", self.mag.read_sample(bus)) {
                    self.readings.mag = Some(MagSample {
                        field_ut: remap(f, board::MAG_AXIS, board::MAG_SIGN),
                        timestamp_us: now_us,
                    });
                }
            }
            // Offset by one tick so the two I²C reads never share a tick.
            if t % 10 == 1 && self.health[BARO].active() {
                let r = self.baro.read_sample(bus);
                // Until the first conversion lands the result is NotReady;
                // that is not a fault. After it, it is.
                if r != Err(Error::NotReady) || self.readings.baro.is_some() {
                    if let Some(b) = self.health[BARO].record("baro", r) {
                        self.readings.baro = Some(BaroSample {
                            pressure_pa: b.pressure_pa,
                            temperature_c: b.temperature_c,
                            timestamp_us: now_us,
                        });
                    }
                }
            }
        }

        self.readings.health = self.health;
        self.readings
    }
}
