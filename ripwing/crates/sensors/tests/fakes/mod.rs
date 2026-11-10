//! Register-level fakes of the three chips, for host tests.
//!
//! Each fake models the behaviour a driver bug would trip on (power-up
//! modes, dummy bytes, write-only registers, timing), not just a register
//! array. They share a simulated clock with [`FakeDelay`], so a driver that
//! skips a required delay fails its test.

#![allow(dead_code)]

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::convert::Infallible;
use std::rc::Rc;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{ErrorType as PinErrorType, OutputPin};
use embedded_hal::i2c::{self, ErrorKind, I2c, NoAcknowledgeSource, Operation};
use embedded_hal::spi::{ErrorType as SpiErrorType, SpiBus};

// ---- clock -----------------------------------------------------------------

/// Simulated time [ns], advanced only by [`FakeDelay`].
#[derive(Clone, Default)]
pub struct Clock(Rc<Cell<u64>>);

impl Clock {
    pub fn now_ns(&self) -> u64 {
        self.0.get()
    }
}

pub struct FakeDelay(pub Clock);

impl DelayNs for FakeDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.0 .0.set(self.0 .0.get() + ns as u64);
    }
}

// ---- I2C -------------------------------------------------------------------

/// A device on the fake I²C bus: the first written byte sets the register
/// pointer, which auto-increments on further writes and reads.
pub trait I2cDevice {
    fn write_reg(&mut self, reg: u8, val: u8);
    fn read_reg(&mut self, reg: u8) -> u8;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Nack;

impl i2c::Error for Nack {
    fn kind(&self) -> ErrorKind {
        ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
    }
}

/// Fake I²C bus. Addresses with no device NACK.
#[derive(Default)]
pub struct FakeI2c {
    pub devices: HashMap<u8, Rc<RefCell<dyn I2cDevice>>>,
    pointer: HashMap<u8, u8>,
}

impl FakeI2c {
    pub fn attach(&mut self, addr: u8, dev: Rc<RefCell<dyn I2cDevice>>) {
        self.devices.insert(addr, dev);
    }
}

impl i2c::ErrorType for FakeI2c {
    type Error = Nack;
}

impl I2c for FakeI2c {
    fn transaction(&mut self, addr: u8, ops: &mut [Operation<'_>]) -> Result<(), Nack> {
        let dev = self.devices.get(&addr).ok_or(Nack)?.clone();
        let mut dev = dev.borrow_mut();
        let ptr = self.pointer.entry(addr).or_insert(0);
        for op in ops {
            match op {
                Operation::Write(bytes) => {
                    if let Some((&first, rest)) = bytes.split_first() {
                        *ptr = first;
                        for &b in rest {
                            dev.write_reg(*ptr, b);
                            *ptr = ptr.wrapping_add(1);
                        }
                    }
                }
                Operation::Read(buf) => {
                    for b in buf.iter_mut() {
                        *b = dev.read_reg(*ptr);
                        *ptr = ptr.wrapping_add(1);
                    }
                }
            }
        }
        Ok(())
    }
}

// ---- BMP581 ----------------------------------------------------------------

pub struct FakeBmp581 {
    pub regs: [u8; 256],
    /// What the chip is measuring.
    pub pressure_pa: f64,
    pub temperature_c: f64,
    /// Hold back the power-on-reset flag (a chip that did not reset).
    pub suppress_por: bool,
}

impl FakeBmp581 {
    pub fn new() -> Self {
        let mut f = Self { regs: [0; 256], pressure_pa: 101_325.0, temperature_c: 21.5, suppress_por: false };
        f.reset();
        f
    }

    fn reset(&mut self) {
        self.regs = [0; 256];
        self.regs[0x01] = 0x50; // CHIP_ID
        self.regs[0x14] = 0x30; // INT_CONFIG: pad drive strength in the upper nibble
        self.regs[0x28] = 0x02; // STATUS: nvm_rdy
        self.regs[0x27] = if self.suppress_por { 0 } else { 0x10 }; // INT_STATUS: POR
        self.regs[0x30] = 0x03; // DSP_CONFIG reset value (shadow bits clear)
        self.regs[0x36] = 0x00; // OSR_CONFIG: pressure disabled
        self.regs[0x37] = 0x70; // ODR_CONFIG
    }

    fn normal(&self) -> bool {
        self.regs[0x37] & 0x03 == 0x01
    }

    /// The chip's ODR/OSR check, modelled as: a conversion of 2^osr_p +
    /// 2^osr_t samples at ~0.6 ms each must fit in the ODR period.
    fn odr_is_valid(&self) -> bool {
        let odr_hz = match (self.regs[0x37] >> 2) & 0x1F {
            0x00 => 240.0,
            0x0A => 100.2,
            0x0F => 50.0,
            0x14 => 25.0,
            0x17 => 10.0,
            _ => 1.0,
        };
        let osr = self.regs[0x36];
        let samples = (1u32 << ((osr >> 3) & 7)) + (1u32 << (osr & 7));
        samples as f64 * 0.6e-3 <= 1.0 / odr_hz
    }

    /// Data registers: zero until running in normal mode with pressure on.
    fn data(&self, reg: u8) -> u8 {
        if !self.normal() {
            return 0;
        }
        let t = (self.temperature_c * 65536.0).round() as i32 as u32;
        let p = if self.regs[0x36] & 0x40 != 0 { (self.pressure_pa * 64.0).round() as u32 } else { 0 };
        match reg {
            0x1D..=0x1F => (t >> (8 * (reg - 0x1D))) as u8,
            0x20..=0x22 => (p >> (8 * (reg - 0x20))) as u8,
            _ => unreachable!(),
        }
    }
}

impl I2cDevice for FakeBmp581 {
    fn write_reg(&mut self, reg: u8, val: u8) {
        match reg {
            0x7E if val == 0xB6 => self.reset(),
            0x01 | 0x27 | 0x28 | 0x38 => {} // read-only
            _ => self.regs[reg as usize] = val,
        }
    }

    fn read_reg(&mut self, reg: u8) -> u8 {
        match reg {
            0x1D..=0x22 => self.data(reg),
            0x27 => std::mem::take(&mut self.regs[0x27]), // clear on read
            0x38 => if self.odr_is_valid() { 0x80 } else { 0 },
            _ => self.regs[reg as usize],
        }
    }
}

// ---- MMC5983MA -------------------------------------------------------------

pub struct FakeMmc5983ma {
    /// True field [counts from null] and the bridge offset [counts].
    pub field: [i32; 3],
    pub offset: [i32; 3],
    /// +1 after a SET pulse, −1 after RESET.
    pub polarity: i32,
    /// Last write to each control register (they are write-only).
    pub ctrl: [u8; 4],
    pub product_id: u8,
    /// Never report a finished one-shot measurement.
    pub stuck: bool,
    out: [u8; 7],
    done: bool,
}

impl FakeMmc5983ma {
    pub fn new(field: [i32; 3], offset: [i32; 3]) -> Self {
        Self {
            field,
            offset,
            // Power-up polarity is undefined; start the "wrong" way so a
            // driver that never SETs is caught.
            polarity: -1,
            ctrl: [0; 4],
            product_id: 0x30,
            stuck: false,
            out: [0; 7],
            done: false,
        }
    }

    fn measure(&mut self) {
        let v: [u32; 3] = std::array::from_fn(|i| (self.polarity * self.field[i] + self.offset[i] + 131_072) as u32);
        self.out = [
            (v[0] >> 10) as u8,
            (v[0] >> 2) as u8,
            (v[1] >> 10) as u8,
            (v[1] >> 2) as u8,
            (v[2] >> 10) as u8,
            (v[2] >> 2) as u8,
            ((v[0] & 3) << 6 | (v[1] & 3) << 4 | (v[2] & 3) << 2) as u8,
        ];
    }

    fn continuous(&self) -> bool {
        self.ctrl[2] & 0x08 != 0
    }
}

impl I2cDevice for FakeMmc5983ma {
    fn write_reg(&mut self, reg: u8, val: u8) {
        match reg {
            0x09 => {
                self.ctrl[0] = val;
                if val & 0x08 != 0 {
                    self.polarity = 1;
                }
                if val & 0x10 != 0 {
                    self.polarity = -1;
                }
                if val & 0x01 != 0 {
                    self.measure();
                    self.done = !self.stuck;
                }
            }
            0x0A => {
                self.ctrl[1] = val;
                if val & 0x80 != 0 {
                    self.ctrl = [0; 4];
                }
            }
            0x0B => self.ctrl[2] = val,
            0x0C => self.ctrl[3] = val,
            _ => {}
        }
    }

    fn read_reg(&mut self, reg: u8) -> u8 {
        match reg {
            0x00..=0x06 => {
                if self.continuous() && reg == 0x00 {
                    // Auto SET keeps continuous results in SET polarity.
                    if self.ctrl[0] & 0x20 != 0 && self.ctrl[2] & 0x80 != 0 {
                        self.polarity = 1;
                    }
                    self.measure();
                }
                self.out[reg as usize]
            }
            0x08 => self.done as u8,
            0x2F => self.product_id,
            0x09..=0x0C => 0xA5, // write-only: garbage
            _ => 0,
        }
    }
}

// ---- BMI088 (SPI) ------------------------------------------------------------

pub struct Bmi088State {
    pub clock: Clock,
    pub acc: [u8; 128],
    pub gyr: [u8; 64],
    /// Accel die talks SPI only after a rising CS edge.
    pub acc_spi_mode: bool,
    /// The CS edge that ends the soft-reset frame does not count.
    acc_reset_pending: bool,
    acc_last_write_ns: Option<u64>,
    gyr_reset_ns: Option<u64>,
    pub cs_acc_low: bool,
    pub cs_gyr_low: bool,
    /// Simulated raw data (LSB), chip frame.
    pub accel_raw: [i16; 3],
    pub gyro_raw: [i16; 3],
    /// Nothing on the bus: MISO idles high.
    pub absent: bool,
}

impl Bmi088State {
    pub fn new(clock: Clock) -> Self {
        let mut s = Self {
            clock,
            acc: [0; 128],
            gyr: [0; 64],
            acc_spi_mode: false,
            acc_reset_pending: false,
            acc_last_write_ns: None,
            gyr_reset_ns: None,
            cs_acc_low: false,
            cs_gyr_low: false,
            accel_raw: [0; 3],
            gyro_raw: [0; 3],
            absent: false,
        };
        s.reset_acc();
        s.reset_gyr();
        s.acc_reset_pending = false; // power-up, not a reset frame
        s
    }

    fn reset_acc(&mut self) {
        self.acc = [0; 128];
        self.acc[0x00] = 0x1E;
        self.acc[0x40] = 0xA8;
        self.acc[0x41] = 0x01;
        self.acc[0x7C] = 0x03; // suspended
        self.acc_spi_mode = false;
        self.acc_reset_pending = true;
    }

    fn reset_gyr(&mut self) {
        self.gyr = [0; 64];
        self.gyr[0x00] = 0x0F;
    }

    fn acc_powered(&self) -> bool {
        self.acc[0x7D] == 0x04 && self.acc[0x7C] == 0x00
    }

    fn acc_read(&mut self, reg: u8) -> u8 {
        match reg {
            0x12..=0x17 if self.acc_powered() => {
                let v = self.accel_raw[((reg - 0x12) / 2) as usize].to_le_bytes();
                v[((reg - 0x12) % 2) as usize]
            }
            0x12..=0x17 => 0,
            0x02 => {
                // Error code 1 when ACC_CONF holds an invalid setting.
                let conf = self.acc[0x40];
                let bwp_ok = (0x08..=0x0A).contains(&(conf >> 4));
                let odr_ok = (0x05..=0x0C).contains(&(conf & 0x0F));
                if bwp_ok && odr_ok { 0 } else { 0x04 }
            }
            r => self.acc[r as usize & 0x7F],
        }
    }

    fn acc_write(&mut self, reg: u8, val: u8) {
        let now = self.clock.now_ns();
        let suspended = self.acc[0x7C] == 0x03;
        // In suspend mode, writes closer than 1000 µs are lost.
        if suspended {
            if let Some(t) = self.acc_last_write_ns {
                if now - t < 1_000_000 {
                    return;
                }
            }
        }
        self.acc_last_write_ns = Some(now);
        if reg == 0x7E && val == 0xB6 {
            self.reset_acc();
        } else if reg != 0x00 && reg != 0x02 {
            self.acc[reg as usize] = val;
        }
    }

    fn gyr_ready(&self) -> bool {
        match self.gyr_reset_ns {
            Some(t) => self.clock.now_ns() - t >= 30_000_000,
            None => true,
        }
    }

    fn gyr_read(&mut self, reg: u8) -> u8 {
        if !self.gyr_ready() {
            return 0;
        }
        match reg {
            0x02..=0x07 => {
                let v = self.gyro_raw[((reg - 0x02) / 2) as usize].to_le_bytes();
                v[((reg - 0x02) % 2) as usize]
            }
            0x10 => self.gyr[0x10] | 0x80, // bit 7 reads as 1
            r => self.gyr[r as usize & 0x3F],
        }
    }

    fn gyr_write(&mut self, reg: u8, val: u8) {
        if !self.gyr_ready() {
            return;
        }
        if reg == 0x14 && val == 0xB6 {
            self.reset_gyr();
            self.gyr_reset_ns = Some(self.clock.now_ns());
        } else if reg != 0x00 {
            self.gyr[reg as usize] = val;
        }
    }

    /// One CS-framed transfer, in place, on whichever die is selected.
    fn transfer(&mut self, buf: &mut [u8]) {
        if self.absent || buf.is_empty() {
            buf.fill(0xFF);
            return;
        }
        let (acc, gyr) = (self.cs_acc_low, self.cs_gyr_low);
        assert!(acc != gyr, "exactly one BMI088 die must be selected");
        let addr = buf[0];
        let read = addr & 0x80 != 0;
        let reg = addr & 0x7F;
        buf[0] = 0xFF;
        if acc {
            if !self.acc_spi_mode {
                // Still in I²C mode: the SPI frame means nothing to it.
                buf[1..].fill(0xFF);
                return;
            }
            if read {
                buf[1] = 0x5A; // the dummy byte
                for (i, b) in buf[2..].iter_mut().enumerate() {
                    *b = self.acc_read(reg + i as u8);
                }
            } else {
                for (i, &v) in buf[1..].to_vec().iter().enumerate() {
                    self.acc_write(reg + i as u8, v);
                }
            }
        } else if read {
            for (i, b) in buf[1..].iter_mut().enumerate() {
                *b = self.gyr_read(reg + i as u8);
            }
        } else {
            for (i, &v) in buf[1..].to_vec().iter().enumerate() {
                self.gyr_write(reg + i as u8, v);
            }
        }
    }
}

pub type Shared = Rc<RefCell<Bmi088State>>;

pub struct FakeSpi(pub Shared);

impl SpiErrorType for FakeSpi {
    type Error = Infallible;
}

impl SpiBus for FakeSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        words.fill(0);
        self.0.borrow_mut().transfer(words);
        Ok(())
    }
    fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
        let mut w = words.to_vec();
        self.0.borrow_mut().transfer(&mut w);
        Ok(())
    }
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Infallible> {
        let mut w = write.to_vec();
        w.resize(read.len().max(write.len()), 0);
        self.0.borrow_mut().transfer(&mut w);
        read.copy_from_slice(&w[..read.len()]);
        Ok(())
    }
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        self.0.borrow_mut().transfer(words);
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

/// Chip-select pin. A rising edge on the accel CS switches that die to SPI.
pub struct FakeCs {
    pub state: Shared,
    pub accel: bool,
}

impl PinErrorType for FakeCs {
    type Error = Infallible;
}

impl OutputPin for FakeCs {
    fn set_low(&mut self) -> Result<(), Infallible> {
        let mut s = self.state.borrow_mut();
        if self.accel { s.cs_acc_low = true } else { s.cs_gyr_low = true }
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Infallible> {
        let mut s = self.state.borrow_mut();
        if self.accel {
            if s.cs_acc_low {
                if s.acc_reset_pending {
                    s.acc_reset_pending = false;
                } else {
                    s.acc_spi_mode = true;
                }
            }
            s.cs_acc_low = false;
        } else {
            s.cs_gyr_low = false;
        }
        Ok(())
    }
}
