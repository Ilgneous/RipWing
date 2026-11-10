//! Bosch BMP581 barometer, I²C.
//!
//! On the PCB: I2C1 with CSB tied to VDD (I²C mode) and SDO to GND, so the
//! address is 0x46 ([`ADDR_SDO_LOW`]). INT goes to PB1 (BARO_INT).
//!
//! The driver runs the chip in normal mode at a fixed output rate and reads
//! the latest result on demand. Pressure and temperature come out already
//! compensated (the BMP581 does it on-chip), so there is no calibration
//! table to read, unlike the BMP280 family.
//!
//! Like the I²C magnetometer driver, it does not own the bus: every call
//! takes `&mut I2c`, so it can share I2C1.

use crate::{BusError, Error};
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

/// I²C address with SDO tied low (the PCB).
pub const ADDR_SDO_LOW: u8 = 0x46;
/// I²C address with SDO tied high.
pub const ADDR_SDO_HIGH: u8 = 0x47;

mod reg {
    pub const CHIP_ID: u8 = 0x01;
    pub const INT_CONFIG: u8 = 0x14;
    pub const INT_SOURCE: u8 = 0x15;
    pub const TEMP_DATA_XLSB: u8 = 0x1D; // 3 bytes temp, then 3 bytes pressure
    pub const INT_STATUS: u8 = 0x27;
    pub const STATUS: u8 = 0x28;
    pub const DSP_CONFIG: u8 = 0x30;
    pub const DSP_IIR: u8 = 0x31;
    pub const OSR_CONFIG: u8 = 0x36;
    pub const ODR_CONFIG: u8 = 0x37;
    pub const OSR_EFF: u8 = 0x38;
    pub const CMD: u8 = 0x7E;
}

/// BMP581. (0x51 is the BMP585, same register map; accepted too.)
pub const CHIP_ID_BMP581: u8 = 0x50;
pub const CHIP_ID_BMP585: u8 = 0x51;

const SOFT_RESET_CMD: u8 = 0xB6;
const STATUS_NVM_RDY: u8 = 0x02;
const STATUS_NVM_ERR: u8 = 0x04;
const INT_STATUS_POR: u8 = 0x10;
const OSR_PRESS_EN: u8 = 0x40;
const ODR_DEEP_DISABLE: u8 = 0x80;
const OSR_EFF_ODR_IS_VALID: u8 = 0x80;
/// DSP_CONFIG: route the IIR-filtered temperature (bit 3) and pressure
/// (bit 5) to the data registers. Without these the registers hold the
/// unfiltered values and the IIR setting does nothing.
const DSP_SHDW_IIR_T: u8 = 0x08;
const DSP_SHDW_IIR_P: u8 = 0x20;
/// INT_CONFIG: enabled (bit 3), active high (bit 1), push-pull, pulsed.
const INT_EN_ACTIVE_HIGH: u8 = 0x08 | 0x02;
const INT_SOURCE_DRDY: u8 = 0x01;

const PWR_STANDBY: u8 = 0x00;
const PWR_NORMAL: u8 = 0x01;

/// Oversampling: more samples per result, less noise, longer conversion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Oversampling {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
    X64 = 6,
    X128 = 7,
}

/// IIR low-pass coefficient applied on-chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Iir {
    Bypass = 0,
    Coeff1 = 1,
    Coeff3 = 2,
    Coeff7 = 3,
    Coeff15 = 4,
    Coeff31 = 5,
    Coeff63 = 6,
    Coeff127 = 7,
}

/// Output data rate in normal mode (a subset of the chip's table).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Odr {
    Hz240 = 0x00,
    Hz100 = 0x0A,
    Hz50 = 0x0F,
    Hz25 = 0x14,
    Hz10 = 0x17,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub odr: Odr,
    pub pressure_osr: Oversampling,
    pub temperature_osr: Oversampling,
    pub pressure_iir: Iir,
    pub temperature_iir: Iir,
    /// Pulse INT on each new result.
    pub data_ready_int: bool,
}

impl Default for Config {
    /// 50 Hz with 16x pressure oversampling and a light IIR: enough for an
    /// altitude-hold loop, with the chip's own check confirming the
    /// combination fits in the 20 ms period.
    fn default() -> Self {
        Self {
            odr: Odr::Hz50,
            pressure_osr: Oversampling::X16,
            temperature_osr: Oversampling::X2,
            pressure_iir: Iir::Coeff3,
            temperature_iir: Iir::Coeff1,
            data_ready_int: true,
        }
    }
}

/// One compensated result.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Reading {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

pub struct Bmp581 {
    addr: u8,
}

impl Bmp581 {
    pub fn new(addr: u8) -> Self {
        Self { addr }
    }

    /// Soft-reset, check the chip came up cleanly, configure it, and start
    /// continuous measurement. Blocks for about 5 ms.
    pub fn init<I: I2c>(&mut self, i2c: &mut I, delay: &mut impl DelayNs, cfg: &Config) -> Result<(), Error> {
        self.check_id(i2c)?;
        self.write(i2c, reg::CMD, SOFT_RESET_CMD)?;
        delay.delay_us(2_000);
        self.check_id(i2c)?;

        // Power-up check (Bosch API): NVM loaded without error, and the
        // POR/soft-reset-complete flag set. Reading INT_STATUS clears it.
        let status = self.read(i2c, reg::STATUS)?;
        if status & STATUS_NVM_RDY == 0 || status & STATUS_NVM_ERR != 0 {
            return Err(Error::NotReady);
        }
        if self.read(i2c, reg::INT_STATUS)? & INT_STATUS_POR == 0 {
            return Err(Error::NotReady);
        }

        // Configure in standby, with deep standby disabled so the settings
        // survive. ODR is written here too so OSR_EFF can validate it.
        let odr_standby = ODR_DEEP_DISABLE | ((cfg.odr as u8) << 2) | PWR_STANDBY;
        self.write(i2c, reg::ODR_CONFIG, odr_standby)?;
        delay.delay_us(2_500);

        let osr = OSR_PRESS_EN | ((cfg.pressure_osr as u8) << 3) | cfg.temperature_osr as u8;
        self.write_checked(i2c, reg::OSR_CONFIG, osr)?;
        let dsp = self.read(i2c, reg::DSP_CONFIG)? | DSP_SHDW_IIR_T | DSP_SHDW_IIR_P;
        self.write_checked(i2c, reg::DSP_CONFIG, dsp)?;
        let iir = ((cfg.pressure_iir as u8) << 3) | cfg.temperature_iir as u8;
        self.write_checked(i2c, reg::DSP_IIR, iir)?;

        if cfg.data_ready_int {
            self.write_checked(i2c, reg::INT_SOURCE, INT_SOURCE_DRDY)?;
            // Keep the upper nibble (INT pad drive strength).
            let int_cfg = (self.read(i2c, reg::INT_CONFIG)? & 0xF0) | INT_EN_ACTIVE_HIGH;
            self.write_checked(i2c, reg::INT_CONFIG, int_cfg)?;
        }

        // The chip's verdict on whether the oversampling fits the ODR. If
        // not, it would quietly use a lower effective oversampling.
        if self.read(i2c, reg::OSR_EFF)? & OSR_EFF_ODR_IS_VALID == 0 {
            return Err(Error::Config);
        }

        let odr_normal = ODR_DEEP_DISABLE | ((cfg.odr as u8) << 2) | PWR_NORMAL;
        self.write_checked(i2c, reg::ODR_CONFIG, odr_normal)?;
        Ok(())
    }

    /// Latest result. Returns [`Error::NotReady`] until the first
    /// conversion completes (pressure still reads 0).
    pub fn read_sample<I: I2c>(&mut self, i2c: &mut I) -> Result<Reading, Error> {
        let mut b = [0u8; 6];
        i2c.write_read(self.addr, &[reg::TEMP_DATA_XLSB], &mut b)
            .map_err(|e| Error::Bus(BusError::from_i2c(&e)))?;
        // Temperature: signed 24-bit, 1/65536 °C. Pressure: unsigned
        // 24-bit, 1/64 Pa.
        let t_raw = i32::from_le_bytes([b[0], b[1], b[2], if b[2] & 0x80 != 0 { 0xFF } else { 0 }]);
        let p_raw = u32::from_le_bytes([b[3], b[4], b[5], 0]);
        if p_raw == 0 {
            return Err(Error::NotReady);
        }
        Ok(Reading {
            pressure_pa: p_raw as f32 / 64.0,
            temperature_c: t_raw as f32 / 65536.0,
        })
    }

    fn check_id<I: I2c>(&mut self, i2c: &mut I) -> Result<(), Error> {
        match self.read(i2c, reg::CHIP_ID)? {
            CHIP_ID_BMP581 | CHIP_ID_BMP585 => Ok(()),
            found => Err(Error::WrongChip { found }),
        }
    }

    fn read<I: I2c>(&mut self, i2c: &mut I, reg: u8) -> Result<u8, Error> {
        let mut v = [0u8; 1];
        i2c.write_read(self.addr, &[reg], &mut v)
            .map_err(|e| Error::Bus(BusError::from_i2c(&e)))?;
        Ok(v[0])
    }

    fn write<I: I2c>(&mut self, i2c: &mut I, reg: u8, val: u8) -> Result<(), Error> {
        i2c.write(self.addr, &[reg, val])
            .map_err(|e| Error::Bus(BusError::from_i2c(&e)))
    }

    fn write_checked<I: I2c>(&mut self, i2c: &mut I, reg: u8, val: u8) -> Result<(), Error> {
        self.write(i2c, reg, val)?;
        if self.read(i2c, reg)? != val {
            return Err(Error::Config);
        }
        Ok(())
    }
}
