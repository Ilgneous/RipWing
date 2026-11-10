//! Chip drivers for RipWing's secondary sensors.
//!
//! * [`bmi088`] — Bosch BMI088 6-axis IMU (IMU2, redundant to the
//!   ICM-42688-P), SPI. Separate accelerometer and gyroscope dies, each
//!   with its own chip select.
//! * [`bmp581`] — Bosch BMP581 barometer, I²C.
//! * [`mmc5983ma`] — MEMSIC MMC5983MA magnetometer, I²C.
//!
//! `no_std`, no HAL: every driver is written against the `embedded-hal` 1.0
//! traits, so the same code runs on the BlackPill F411 today and the H723
//! PCB later, and is tested on the host against register-level fakes of
//! each chip (`tests/`).
//!
//! Bus ownership. The BMI088 owns its SPI bus and both chip-select pins (on
//! the PCB it is alone on SPI2), so no bus-sharing layer is needed. The two
//! I²C drivers do not own the bus: every call takes `&mut I2c`, so the
//! barometer and magnetometer share I2C1 without a mutex or `RefCell`.
//!
//! Register maps were checked against Bosch's `bmp5_defs.h` and
//! `bmi08_defs.h` (BMP5_SensorAPI, BMI08x-Sensor-API) and SparkFun's
//! MMC5983MA library constants.

#![no_std]

pub mod bmi088;
pub mod bmp581;
pub mod mmc5983ma;

/// What went wrong on the bus, independent of the HAL's error type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusError {
    /// I²C: no device acknowledged (absent, wrong address, unpowered).
    NoAcknowledge,
    /// I²C: another master or a glitch took the bus.
    ArbitrationLoss,
    /// Bus-level fault (I²C bus error, SPI mode fault, CS pin error).
    Bus,
    /// Data arrived faster than it was read.
    Overrun,
    Other,
}

impl BusError {
    pub fn from_i2c(e: &impl embedded_hal::i2c::Error) -> Self {
        use embedded_hal::i2c::ErrorKind as K;
        match e.kind() {
            K::NoAcknowledge(_) => Self::NoAcknowledge,
            K::ArbitrationLoss => Self::ArbitrationLoss,
            K::Bus => Self::Bus,
            K::Overrun => Self::Overrun,
            _ => Self::Other,
        }
    }

    pub fn from_spi(e: &impl embedded_hal::spi::Error) -> Self {
        use embedded_hal::spi::ErrorKind as K;
        match e.kind() {
            K::Overrun => Self::Overrun,
            K::ModeFault | K::FrameFormat | K::ChipSelectFault => Self::Bus,
            _ => Self::Other,
        }
    }
}

/// Driver error, shared by all three chips.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The bus transaction failed.
    Bus(BusError),
    /// The ID register did not hold the expected value: a different chip,
    /// nothing connected (SPI reads back 0x00 or 0xFF), or a wiring fault.
    WrongChip { found: u8 },
    /// The chip did not come out of reset or finish a measurement in time.
    NotReady,
    /// The chip rejected the configuration (BMI088 error register, BMP581
    /// ODR/oversampling combination), or a register read back differently
    /// from what was written.
    Config,
}

/// Map a raw sensor vector into the body frame:
/// `out[i] = sign[i] * v[axis[i]]`. The same convention as the ICM-42688-P
/// path (`board::IMU_AXIS` / `IMU_SIGN`).
pub fn remap(v: [f32; 3], axis: [usize; 3], sign: [f32; 3]) -> [f32; 3] {
    [sign[0] * v[axis[0]], sign[1] * v[axis[1]], sign[2] * v[axis[2]]]
}
