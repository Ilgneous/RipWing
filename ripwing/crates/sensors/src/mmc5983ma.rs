//! MEMSIC MMC5983MA 3-axis magnetometer, I²C.
//!
//! On the PCB: I2C1 at address 0x30 (fixed), with SPI_CS tied to VDD
//! (I²C mode) and INT to PB0 (MAG_INT). The KiCad symbol is the MMC5883MA's;
//! the part is the MMC5983MA.
//!
//! # Offset removal (SET/RESET)
//!
//! The sensor is an AMR bridge. Each output carries a bridge offset that
//! drifts with temperature and after strong fields. A SET pulse magnetizes
//! the bridge one way and a RESET pulse the other, which flips the sign of
//! the field term but not of the offset:
//!
//! ```text
//!   after SET:   m_set   = +H + offset
//!   after RESET: m_reset = −H + offset
//!   offset = (m_set + m_reset) / 2
//! ```
//!
//! [`calibrate_offset`](Mmc5983ma::calibrate_offset) measures this once
//! (`init` does it) and every reading has it subtracted. Continuous mode
//! then runs with automatic SET pulses, so the polarity stays SET. This is
//! the chip's own offset; the airframe's hard- and soft-iron distortion is
//! a separate calibration, done by the estimator.
//!
//! The control registers are write-only (reads return garbage), so the
//! driver cannot verify them by reading back. It keeps its own copy instead.

use crate::{BusError, Error};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

/// Fixed I²C address.
pub const ADDR: u8 = 0x30;

mod reg {
    pub const XOUT0: u8 = 0x00; // X0 X1 Y0 Y1 Z0 Z1 XYZ2
    pub const STATUS: u8 = 0x08;
    pub const CTRL0: u8 = 0x09;
    pub const CTRL1: u8 = 0x0A;
    pub const CTRL2: u8 = 0x0B;
    pub const PRODUCT_ID: u8 = 0x2F;
}

pub const PRODUCT_ID: u8 = 0x30;

const STATUS_MEAS_M_DONE: u8 = 0x01;
const CTRL0_TM_M: u8 = 0x01;
const CTRL0_INT_MEAS_DONE_EN: u8 = 0x04;
const CTRL0_SET: u8 = 0x08;
const CTRL0_RESET: u8 = 0x10;
const CTRL0_AUTO_SR_EN: u8 = 0x20;
const CTRL1_SW_RST: u8 = 0x80;
const CTRL2_CMM_EN: u8 = 0x08;
const CTRL2_EN_PRD_SET: u8 = 0x80;

/// 18-bit output: zero field reads 2^17, and 16384 counts per gauss.
const NULL_COUNTS: i32 = 131_072;
const COUNTS_PER_GAUSS: f32 = 16_384.0;
const UT_PER_GAUSS: f32 = 100.0;

/// Measurement time per sample (filter bandwidth).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Bandwidth {
    /// 8 ms, lowest noise. Continuous mode up to 100 Hz.
    Ms8 = 0b00,
    /// 4 ms. Needed for 200 Hz.
    Ms4 = 0b01,
    Ms2 = 0b10,
    /// 0.5 ms. Needed for 1000 Hz.
    Ms0_5 = 0b11,
}

impl Bandwidth {
    fn measure_ms(self) -> u32 {
        match self {
            Self::Ms8 => 8,
            Self::Ms4 => 4,
            Self::Ms2 => 2,
            Self::Ms0_5 => 1,
        }
    }
}

/// Continuous-mode rate (CTRL2 Cm_freq).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Rate {
    Hz1 = 0b001,
    Hz10 = 0b010,
    Hz20 = 0b011,
    Hz50 = 0b100,
    Hz100 = 0b101,
}

/// How often continuous mode fires an automatic SET pulse (CTRL2 Prd_set).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum SetPeriod {
    Every1 = 0b000,
    Every25 = 0b001,
    Every75 = 0b010,
    Every100 = 0b011,
    Every250 = 0b100,
    Every500 = 0b101,
    Every1000 = 0b110,
    Every2000 = 0b111,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub bandwidth: Bandwidth,
    pub rate: Rate,
    pub set_period: SetPeriod,
    /// Pulse INT when a measurement completes.
    pub data_ready_int: bool,
}

impl Default for Config {
    /// 100 Hz with the lowest-noise filter, re-SET every 25 samples (4 Hz)
    /// so a strong field (motor current, a magnet) cannot leave the bridge
    /// flipped for long.
    fn default() -> Self {
        Self {
            bandwidth: Bandwidth::Ms8,
            rate: Rate::Hz100,
            set_period: SetPeriod::Every25,
            data_ready_int: true,
        }
    }
}

pub struct Mmc5983ma {
    /// Bridge offset [counts from null], removed from every reading.
    offset: [f32; 3],
    cfg: Config,
}

impl Mmc5983ma {
    pub fn new() -> Self {
        Self { offset: [0.0; 3], cfg: Config::default() }
    }

    /// The bridge offset currently removed from readings, in µT.
    pub fn offset_ut(&self) -> [f32; 3] {
        self.offset.map(|c| c / COUNTS_PER_GAUSS * UT_PER_GAUSS)
    }

    /// Reset, check the product ID, measure the bridge offset, and start
    /// continuous measurement. Blocks for about 35 ms.
    pub fn init<I: I2c>(&mut self, i2c: &mut I, delay: &mut impl DelayNs, cfg: &Config) -> Result<(), Error> {
        self.check_id(i2c)?;
        self.write(i2c, reg::CTRL1, CTRL1_SW_RST)?;
        delay.delay_ms(10);
        self.check_id(i2c)?;

        self.cfg = *cfg;
        self.write(i2c, reg::CTRL1, cfg.bandwidth as u8)?;
        self.calibrate_offset(i2c, delay)
    }

    /// Measure the bridge offset with a SET/RESET pair, then (re)start
    /// continuous measurement. Safe to call in flight to track the offset's
    /// temperature drift; readings pause for about 20 ms.
    pub fn calibrate_offset<I: I2c>(&mut self, i2c: &mut I, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.measure_offset(i2c, delay)?;
        self.start_continuous(i2c)
    }

    fn measure_offset<I: I2c>(&mut self, i2c: &mut I, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.write(i2c, reg::CTRL2, 0)?; // continuous off
        let m_set = self.pulse_and_measure(i2c, delay, CTRL0_SET)?;
        let m_reset = self.pulse_and_measure(i2c, delay, CTRL0_RESET)?;
        for i in 0..3 {
            self.offset[i] = (m_set[i] + m_reset[i]) as f32 / 2.0;
        }
        // Leave the bridge in SET polarity, which continuous mode keeps.
        self.write(i2c, reg::CTRL0, CTRL0_SET)?;
        delay.delay_ms(1);
        Ok(())
    }

    fn start_continuous<I: I2c>(&mut self, i2c: &mut I) -> Result<(), Error> {
        let mut ctrl0 = CTRL0_AUTO_SR_EN;
        if self.cfg.data_ready_int {
            ctrl0 |= CTRL0_INT_MEAS_DONE_EN;
        }
        self.write(i2c, reg::CTRL0, ctrl0)?;
        let ctrl2 = CTRL2_EN_PRD_SET | ((self.cfg.set_period as u8) << 4) | CTRL2_CMM_EN | self.cfg.rate as u8;
        self.write(i2c, reg::CTRL2, ctrl2)
    }

    /// Latest continuous-mode result, chip frame, offset removed (µT).
    pub fn read_sample<I: I2c>(&mut self, i2c: &mut I) -> Result<[f32; 3], Error> {
        let raw = self.read_raw(i2c)?;
        Ok(core::array::from_fn(|i| {
            (raw[i] as f32 - self.offset[i]) / COUNTS_PER_GAUSS * UT_PER_GAUSS
        }))
    }

    /// One-shot measurement after a SET or RESET pulse, centred on zero.
    fn pulse_and_measure<I: I2c>(&mut self, i2c: &mut I, delay: &mut impl DelayNs, pulse: u8) -> Result<[i32; 3], Error> {
        self.write(i2c, reg::CTRL0, pulse)?;
        delay.delay_ms(1);
        self.write(i2c, reg::CTRL0, CTRL0_TM_M)?;
        // Measurement time plus margin, polled in 1 ms steps.
        for _ in 0..(self.cfg.bandwidth.measure_ms() + 5) {
            delay.delay_ms(1);
            if self.read(i2c, reg::STATUS)? & STATUS_MEAS_M_DONE != 0 {
                return self.read_raw(i2c);
            }
        }
        Err(Error::NotReady)
    }

    /// The three 18-bit outputs, centred on zero [counts].
    fn read_raw<I: I2c>(&mut self, i2c: &mut I) -> Result<[i32; 3], Error> {
        let mut b = [0u8; 7];
        i2c.write_read(ADDR, &[reg::XOUT0], &mut b)
            .map_err(|e| Error::Bus(BusError::from_i2c(&e)))?;
        // Bits 17:10 and 9:2 in the axis registers; bits 1:0 packed in
        // XYZOUT2 (X in 7:6, Y in 5:4, Z in 3:2).
        let axis = |hi: u8, lo: u8, shift: u8| -> i32 {
            ((hi as i32) << 10 | (lo as i32) << 2 | ((b[6] >> shift) & 0x03) as i32) - NULL_COUNTS
        };
        Ok([axis(b[0], b[1], 6), axis(b[2], b[3], 4), axis(b[4], b[5], 2)])
    }

    fn check_id<I: I2c>(&mut self, i2c: &mut I) -> Result<(), Error> {
        match self.read(i2c, reg::PRODUCT_ID)? {
            PRODUCT_ID => Ok(()),
            found => Err(Error::WrongChip { found }),
        }
    }

    fn read<I: I2c>(&mut self, i2c: &mut I, reg: u8) -> Result<u8, Error> {
        let mut v = [0u8; 1];
        i2c.write_read(ADDR, &[reg], &mut v)
            .map_err(|e| Error::Bus(BusError::from_i2c(&e)))?;
        Ok(v[0])
    }

    fn write<I: I2c>(&mut self, i2c: &mut I, reg: u8, val: u8) -> Result<(), Error> {
        i2c.write(ADDR, &[reg, val]).map_err(|e| Error::Bus(BusError::from_i2c(&e)))
    }
}

impl Default for Mmc5983ma {
    fn default() -> Self {
        Self::new()
    }
}
