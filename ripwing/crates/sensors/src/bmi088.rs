//! Bosch BMI088: 6-axis IMU, accelerometer and gyroscope as two dies.
//!
//! On RipWing it is IMU2, redundant to the ICM-42688-P. It is on SPI2 with
//! the PS pin tied to GND (SPI mode), and each die has its own chip select
//! (CSB1 accel, CSB2 gyro). The driver owns the bus and both CS pins.
//!
//! The two dies differ on the wire:
//!
//! * **Accel reads return a dummy byte** before the data. Every accel read
//!   clocks one extra byte and discards it. Gyro reads do not.
//! * **The accel powers up in I²C mode** and only switches to SPI on a
//!   rising CS edge, so `init` does a throwaway read first, and again after
//!   the soft reset, which puts it back in I²C mode.
//! * **The accel powers up suspended**: it reads zeros until `PWR_CTRL` is
//!   turned on. While suspended it needs 1 ms between writes.
//!
//! Interrupts: `init` routes accel data-ready to INT1 and gyro data-ready to
//! INT3 (both active high, push-pull), as wired on the PCB. On the BlackPill
//! the firmware polls instead, so the pins are configured but unused.

use crate::{BusError, Error};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;

mod acc {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR: u8 = 0x02;
    pub const DATA: u8 = 0x12; // X_LSB .. Z_MSB
    pub const TEMP_MSB: u8 = 0x22; // MSB, LSB
    pub const CONF: u8 = 0x40;
    pub const RANGE: u8 = 0x41;
    pub const INT1_IO_CONF: u8 = 0x53;
    pub const INT_MAP_DATA: u8 = 0x58;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const SOFTRESET: u8 = 0x7E;

    pub const CHIP_ID_VALUE: u8 = 0x1E;
    pub const PWR_CONF_ACTIVE: u8 = 0x00;
    pub const PWR_CTRL_ON: u8 = 0x04;
    /// Bandwidth parameter "normal" in ACC_CONF[7:4].
    pub const BWP_NORMAL: u8 = 0x0A << 4;
    /// INT1: output enabled (bit 3), active high (bit 1), push-pull.
    pub const INT1_OUT_ACTIVE_HIGH: u8 = 0x08 | 0x02;
    pub const INT1_DRDY: u8 = 0x04;
    /// ERR_REG: fatal error (bit 0) or a non-zero error code (bits 4:2).
    pub const ERR_MASK: u8 = 0x01 | 0x1C;
}

mod gyr {
    pub const CHIP_ID: u8 = 0x00;
    pub const DATA: u8 = 0x02; // RATE_X_LSB .. RATE_Z_MSB
    pub const RANGE: u8 = 0x0F;
    pub const BANDWIDTH: u8 = 0x10;
    pub const LPM1: u8 = 0x11;
    pub const SOFTRESET: u8 = 0x14;
    pub const INT_CTRL: u8 = 0x15;
    pub const INT3_INT4_IO_CONF: u8 = 0x16;
    pub const INT3_INT4_IO_MAP: u8 = 0x18;

    pub const CHIP_ID_VALUE: u8 = 0x0F;
    pub const LPM1_NORMAL: u8 = 0x00;
    pub const INT_CTRL_DRDY: u8 = 0x80;
    /// INT3 active high (bit 0), push-pull.
    pub const INT3_ACTIVE_HIGH: u8 = 0x01;
    pub const INT3_DRDY: u8 = 0x01;
}

const READ: u8 = 0x80;
const SOFTRESET_CMD: u8 = 0xB6;
const STANDARD_GRAVITY: f32 = 9.806_65;

/// Accelerometer full scale. Register values from the BMI088 datasheet;
/// full scale = 3 g · 2^value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelRange {
    G3 = 0x00,
    G6 = 0x01,
    G12 = 0x02,
    G24 = 0x03,
}

impl AccelRange {
    pub fn full_scale_g(self) -> f32 {
        3.0 * (1u32 << self as u8) as f32
    }
}

/// Accelerometer output data rate (with the "normal" filter setting).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelOdr {
    Hz100 = 0x08,
    Hz200 = 0x09,
    Hz400 = 0x0A,
    Hz800 = 0x0B,
    Hz1600 = 0x0C,
}

/// Gyroscope full scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroRange {
    Dps2000 = 0x00,
    Dps1000 = 0x01,
    Dps500 = 0x02,
    Dps250 = 0x03,
    Dps125 = 0x04,
}

impl GyroRange {
    pub fn full_scale_dps(self) -> f32 {
        2000.0 / (1u32 << self as u8) as f32
    }
}

/// Gyroscope output data rate and filter bandwidth (one register).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroBandwidth {
    Odr2000Bw532 = 0x00,
    Odr2000Bw230 = 0x01,
    Odr1000Bw116 = 0x02,
    Odr400Bw47 = 0x03,
    Odr200Bw23 = 0x04,
    Odr100Bw12 = 0x05,
    Odr200Bw64 = 0x06,
    Odr100Bw32 = 0x07,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub accel_range: AccelRange,
    pub accel_odr: AccelOdr,
    pub gyro_range: GyroRange,
    pub gyro_bandwidth: GyroBandwidth,
}

impl Default for Config {
    /// ±24 g, because vibration on a quad easily clips ±6 g; ±2000 °/s like
    /// the ICM-42688-P. Gyro 1 kHz with a 116 Hz filter and accel 800 Hz, so
    /// polling at 500 Hz or reading on data-ready at 1 kHz both work.
    fn default() -> Self {
        Self {
            accel_range: AccelRange::G24,
            accel_odr: AccelOdr::Hz800,
            gyro_range: GyroRange::Dps2000,
            gyro_bandwidth: GyroBandwidth::Odr1000Bw116,
        }
    }
}

/// One reading in the chip frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reading {
    /// Specific force [m/s²].
    pub accel_mps2: [f32; 3],
    /// Angular rate [rad/s].
    pub gyro_rps: [f32; 3],
}

pub struct Bmi088<SPI, CSA, CSG> {
    spi: SPI,
    cs_accel: CSA,
    cs_gyro: CSG,
    /// m/s² per LSB.
    accel_scale: f32,
    /// rad/s per LSB.
    gyro_scale: f32,
}

#[derive(Clone, Copy)]
enum Die {
    Accel,
    Gyro,
}

impl<SPI, CSA, CSG> Bmi088<SPI, CSA, CSG>
where
    SPI: SpiBus,
    CSA: OutputPin,
    CSG: OutputPin,
{
    /// Take the bus and both chip selects. Does not touch the chip; call
    /// [`init`](Self::init).
    pub fn new(spi: SPI, cs_accel: CSA, cs_gyro: CSG) -> Self {
        let cfg = Config::default();
        Self {
            spi,
            cs_accel,
            cs_gyro,
            accel_scale: accel_scale(cfg.accel_range),
            gyro_scale: gyro_scale(cfg.gyro_range),
        }
    }

    /// Give back the bus and pins.
    pub fn release(self) -> (SPI, CSA, CSG) {
        (self.spi, self.cs_accel, self.cs_gyro)
    }

    /// Reset both dies, check their IDs, and configure them. Blocks for
    /// about 45 ms (two soft resets and the accel power-up).
    pub fn init(&mut self, delay: &mut impl DelayNs, cfg: &Config) -> Result<(), Error> {
        self.cs_accel.set_high().map_err(|_| Error::Bus(BusError::Bus))?;
        self.cs_gyro.set_high().map_err(|_| Error::Bus(BusError::Bus))?;

        // ---- Accelerometer ---------------------------------------------
        // Rising CS edge: switch the accel die from I²C to SPI mode.
        self.read_reg(Die::Accel, acc::CHIP_ID)?;
        check_id(self.read_reg(Die::Accel, acc::CHIP_ID)?, acc::CHIP_ID_VALUE)?;
        self.write_reg(Die::Accel, acc::SOFTRESET, SOFTRESET_CMD)?;
        delay.delay_ms(1);
        self.read_reg(Die::Accel, acc::CHIP_ID)?; // back to SPI after the reset
        check_id(self.read_reg(Die::Accel, acc::CHIP_ID)?, acc::CHIP_ID_VALUE)?;

        // Power up: active mode, then sensor on (Bosch API order, 5 ms each).
        self.write_reg(Die::Accel, acc::PWR_CONF, acc::PWR_CONF_ACTIVE)?;
        delay.delay_ms(5);
        self.write_reg(Die::Accel, acc::PWR_CTRL, acc::PWR_CTRL_ON)?;
        delay.delay_ms(5);

        let conf = acc::BWP_NORMAL | cfg.accel_odr as u8;
        self.write_checked(Die::Accel, acc::CONF, conf, 0xFF, delay)?;
        self.write_checked(Die::Accel, acc::RANGE, cfg.accel_range as u8, 0x03, delay)?;
        self.write_checked(Die::Accel, acc::INT1_IO_CONF, acc::INT1_OUT_ACTIVE_HIGH, 0x1E, delay)?;
        self.write_checked(Die::Accel, acc::INT_MAP_DATA, acc::INT1_DRDY, 0xFF, delay)?;
        if self.read_reg(Die::Accel, acc::ERR)? & acc::ERR_MASK != 0 {
            return Err(Error::Config);
        }

        // ---- Gyroscope -------------------------------------------------
        check_id(self.read_reg(Die::Gyro, gyr::CHIP_ID)?, gyr::CHIP_ID_VALUE)?;
        self.write_reg(Die::Gyro, gyr::SOFTRESET, SOFTRESET_CMD)?;
        delay.delay_ms(30);
        check_id(self.read_reg(Die::Gyro, gyr::CHIP_ID)?, gyr::CHIP_ID_VALUE)?;

        self.write_checked(Die::Gyro, gyr::LPM1, gyr::LPM1_NORMAL, 0xFF, delay)?;
        self.write_checked(Die::Gyro, gyr::RANGE, cfg.gyro_range as u8, 0xFF, delay)?;
        // BANDWIDTH bit 7 always reads 1; compare the code only.
        self.write_checked(Die::Gyro, gyr::BANDWIDTH, cfg.gyro_bandwidth as u8, 0x7F, delay)?;
        self.write_checked(Die::Gyro, gyr::INT_CTRL, gyr::INT_CTRL_DRDY, 0xFF, delay)?;
        self.write_checked(Die::Gyro, gyr::INT3_INT4_IO_CONF, gyr::INT3_ACTIVE_HIGH, 0x03, delay)?;
        self.write_checked(Die::Gyro, gyr::INT3_INT4_IO_MAP, gyr::INT3_DRDY, 0xFF, delay)?;

        self.accel_scale = accel_scale(cfg.accel_range);
        self.gyro_scale = gyro_scale(cfg.gyro_range);
        Ok(())
    }

    /// Latest accel and gyro data, chip frame, SI units. Two short bursts
    /// (7 + 6 bytes after the address).
    pub fn read(&mut self) -> Result<Reading, Error> {
        let mut a = [0u8; 6];
        let mut g = [0u8; 6];
        self.read_regs(Die::Accel, acc::DATA, &mut a)?;
        self.read_regs(Die::Gyro, gyr::DATA, &mut g)?;
        Ok(Reading {
            accel_mps2: le3(&a).map(|v| v as f32 * self.accel_scale),
            gyro_rps: le3(&g).map(|v| v as f32 * self.gyro_scale),
        })
    }

    /// Accelerometer die temperature [°C], updated every 1.28 s.
    pub fn read_temperature_c(&mut self) -> Result<f32, Error> {
        let mut t = [0u8; 2];
        self.read_regs(Die::Accel, acc::TEMP_MSB, &mut t)?;
        // 11-bit two's complement: MSB holds bits 10:3, LSB[7:5] bits 2:0.
        let raw = ((t[0] as u16) << 3) | ((t[1] as u16) >> 5);
        let raw = if raw > 1023 { raw as i32 - 2048 } else { raw as i32 };
        Ok(raw as f32 * 0.125 + 23.0)
    }

    // ---- register access ---------------------------------------------------

    fn select(&mut self, die: Die, on: bool) -> Result<(), Error> {
        // The two pins are different types, so map each error on its own.
        let ok = match (die, on) {
            (Die::Accel, true) => self.cs_accel.set_low().is_ok(),
            (Die::Accel, false) => self.cs_accel.set_high().is_ok(),
            (Die::Gyro, true) => self.cs_gyro.set_low().is_ok(),
            (Die::Gyro, false) => self.cs_gyro.set_high().is_ok(),
        };
        if ok {
            Ok(())
        } else {
            Err(Error::Bus(BusError::Bus))
        }
    }

    /// One CS-framed transfer. CS is released even if the bus fails.
    fn frame(&mut self, die: Die, buf: &mut [u8]) -> Result<(), Error> {
        self.select(die, true)?;
        let r = self.spi.transfer_in_place(buf).and_then(|_| self.spi.flush());
        self.select(die, false)?;
        r.map_err(|e| Error::Bus(BusError::from_spi(&e)))
    }

    fn read_regs(&mut self, die: Die, reg: u8, out: &mut [u8]) -> Result<(), Error> {
        // Address byte, one dummy byte on the accel die, then the data.
        let skip = match die {
            Die::Accel => 2,
            Die::Gyro => 1,
        };
        let mut buf = [0u8; 8];
        let n = skip + out.len();
        assert!(n <= buf.len(), "burst too long");
        buf[0] = reg | READ;
        self.frame(die, &mut buf[..n])?;
        out.copy_from_slice(&buf[skip..n]);
        Ok(())
    }

    fn read_reg(&mut self, die: Die, reg: u8) -> Result<u8, Error> {
        let mut v = [0u8; 1];
        self.read_regs(die, reg, &mut v)?;
        Ok(v[0])
    }

    fn write_reg(&mut self, die: Die, reg: u8, val: u8) -> Result<(), Error> {
        let mut buf = [reg & !READ, val];
        self.frame(die, &mut buf)
    }

    /// Write, wait out the accel's write-to-write gap, and read back.
    fn write_checked(
        &mut self,
        die: Die,
        reg: u8,
        val: u8,
        mask: u8,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error> {
        self.write_reg(die, reg, val)?;
        delay.delay_ms(1);
        if self.read_reg(die, reg)? & mask != val & mask {
            return Err(Error::Config);
        }
        Ok(())
    }
}

fn check_id(found: u8, expected: u8) -> Result<(), Error> {
    if found == expected {
        Ok(())
    } else {
        Err(Error::WrongChip { found })
    }
}

fn accel_scale(r: AccelRange) -> f32 {
    r.full_scale_g() * STANDARD_GRAVITY / 32768.0
}

fn gyro_scale(r: GyroRange) -> f32 {
    r.full_scale_dps().to_radians() / 32768.0
}

/// Three little-endian i16s.
fn le3(b: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([b[0], b[1]]),
        i16::from_le_bytes([b[2], b[3]]),
        i16::from_le_bytes([b[4], b[5]]),
    ]
}
