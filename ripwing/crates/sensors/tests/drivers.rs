//! Driver tests against the register-level fakes in `fakes/`.

mod fakes;

use approx::assert_abs_diff_eq;
use fakes::*;
use ripwing_sensors::{bmi088, bmp581, mmc5983ma, BusError, Error};
use std::cell::RefCell;
use std::rc::Rc;

// ---------------------------------------------------------------------------
// BMI088
// ---------------------------------------------------------------------------

fn bmi088() -> (bmi088::Bmi088<FakeSpi, FakeCs, FakeCs>, Shared, FakeDelay) {
    let clock = Clock::default();
    let st: Shared = Rc::new(RefCell::new(Bmi088State::new(clock.clone())));
    let dev = bmi088::Bmi088::new(
        FakeSpi(st.clone()),
        FakeCs { state: st.clone(), accel: true },
        FakeCs { state: st.clone(), accel: false },
    );
    (dev, st, FakeDelay(clock))
}

#[test]
fn bmi088_init_configures_both_dies() {
    let (mut imu, st, mut d) = bmi088();
    imu.init(&mut d, &bmi088::Config::default()).unwrap();
    let s = st.borrow();
    assert!(s.acc_spi_mode, "accel never switched to SPI");
    assert_eq!(s.acc[0x7C], 0x00, "accel not active");
    assert_eq!(s.acc[0x7D], 0x04, "accel not powered on");
    assert_eq!(s.acc[0x40], 0xAB, "ACC_CONF: normal filter, 800 Hz");
    assert_eq!(s.acc[0x41], 0x03, "ACC_RANGE: ±24 g");
    assert_eq!(s.acc[0x53], 0x0A, "INT1: output, active high, push-pull");
    assert_eq!(s.acc[0x58], 0x04, "accel data-ready on INT1");
    assert_eq!(s.gyr[0x0F], 0x00, "GYRO_RANGE: ±2000 °/s");
    assert_eq!(s.gyr[0x10], 0x02, "GYRO_BANDWIDTH: 1 kHz / 116 Hz");
    assert_eq!(s.gyr[0x15], 0x80, "gyro data-ready interrupt");
    assert_eq!(s.gyr[0x16], 0x01, "INT3 active high, push-pull");
    assert_eq!(s.gyr[0x18], 0x01, "gyro data-ready on INT3");
}

#[test]
fn bmi088_init_takes_the_required_time() {
    // 1 ms + 5 + 5 for the accel, 30 ms gyro reset: at least 41 ms.
    let (mut imu, st, mut d) = bmi088();
    imu.init(&mut d, &bmi088::Config::default()).unwrap();
    assert!(st.borrow().clock.now_ns() >= 41_000_000);
}

#[test]
fn bmi088_read_scales_and_skips_the_dummy_byte() {
    let (mut imu, st, mut d) = bmi088();
    imu.init(&mut d, &bmi088::Config::default()).unwrap();
    // ±24 g: 1 g = 32768 / 24 LSB. ±2000 °/s: 1000 °/s = 16384 LSB.
    st.borrow_mut().accel_raw = [1365, -1365, 2731];
    st.borrow_mut().gyro_raw = [16384, -8192, 0];
    let r = imu.read().unwrap();
    let g = 9.806_65;
    assert_abs_diff_eq!(r.accel_mps2[0], 1365.0 * 24.0 * g / 32768.0, epsilon = 1e-4);
    assert_abs_diff_eq!(r.accel_mps2[0], g, epsilon = 0.01);
    assert_abs_diff_eq!(r.accel_mps2[1], -r.accel_mps2[0], epsilon = 1e-6);
    assert_abs_diff_eq!(r.accel_mps2[2], 2.0 * g, epsilon = 0.01);
    assert_abs_diff_eq!(r.gyro_rps[0], 1000f32.to_radians(), epsilon = 1e-4);
    assert_abs_diff_eq!(r.gyro_rps[1], -500f32.to_radians(), epsilon = 1e-4);
    assert_eq!(r.gyro_rps[2], 0.0);
}

#[test]
fn bmi088_range_sets_the_scale() {
    let (mut imu, st, mut d) = bmi088();
    let cfg = bmi088::Config {
        accel_range: bmi088::AccelRange::G6,
        gyro_range: bmi088::GyroRange::Dps500,
        ..Default::default()
    };
    imu.init(&mut d, &cfg).unwrap();
    st.borrow_mut().accel_raw = [5461, 0, 0]; // 1 g at ±6 g
    st.borrow_mut().gyro_raw = [16384, 0, 0]; // 250 °/s at ±500 °/s
    let r = imu.read().unwrap();
    assert_abs_diff_eq!(r.accel_mps2[0], 9.806_65, epsilon = 0.01);
    assert_abs_diff_eq!(r.gyro_rps[0], 250f32.to_radians(), epsilon = 1e-4);
}

#[test]
fn bmi088_absent_is_wrong_chip() {
    let (mut imu, st, mut d) = bmi088();
    st.borrow_mut().absent = true;
    assert_eq!(imu.init(&mut d, &Default::default()), Err(Error::WrongChip { found: 0xFF }));
}

#[test]
fn bmi088_temperature() {
    let (mut imu, st, mut d) = bmi088();
    imu.init(&mut d, &Default::default()).unwrap();
    for (msb, lsb, want) in [(0x00, 0x00, 23.0), (0x02, 0x00, 25.0), (0xFF, 0xE0, 22.875), (0x80, 0x00, -105.0)] {
        st.borrow_mut().acc[0x22] = msb;
        st.borrow_mut().acc[0x23] = lsb;
        assert_abs_diff_eq!(imu.read_temperature_c().unwrap(), want, epsilon = 1e-6);
    }
}

// ---------------------------------------------------------------------------
// BMP581
// ---------------------------------------------------------------------------

fn bmp581() -> (bmp581::Bmp581, FakeI2c, Rc<RefCell<FakeBmp581>>, FakeDelay) {
    let chip = Rc::new(RefCell::new(FakeBmp581::new()));
    let mut bus = FakeI2c::default();
    bus.attach(bmp581::ADDR_SDO_LOW, chip.clone());
    (bmp581::Bmp581::new(bmp581::ADDR_SDO_LOW), bus, chip, FakeDelay(Clock::default()))
}

#[test]
fn bmp581_init_configures_and_starts() {
    let (mut baro, mut bus, chip, mut d) = bmp581();
    baro.init(&mut bus, &mut d, &bmp581::Config::default()).unwrap();
    let c = chip.borrow();
    assert_eq!(c.regs[0x36], 0x40 | (4 << 3) | 1, "OSR: pressure on, x16 / x2");
    assert_eq!(c.regs[0x31], (2 << 3) | 1, "IIR: pressure coeff 3, temp coeff 1");
    assert_eq!(c.regs[0x30] & 0x28, 0x28, "filtered results routed to the data registers");
    assert_eq!(c.regs[0x37], 0x80 | (0x0F << 2) | 0x01, "50 Hz, normal mode, deep standby off");
    assert_eq!(c.regs[0x15], 0x01, "data-ready interrupt source");
    assert_eq!(c.regs[0x14], 0x30 | 0x0A, "INT enabled, active high; drive strength kept");
}

#[test]
fn bmp581_reads_pressure_and_temperature() {
    let (mut baro, mut bus, chip, mut d) = bmp581();
    baro.init(&mut bus, &mut d, &Default::default()).unwrap();
    chip.borrow_mut().pressure_pa = 98_765.25;
    chip.borrow_mut().temperature_c = -10.5;
    let r = baro.read_sample(&mut bus).unwrap();
    assert_abs_diff_eq!(r.pressure_pa, 98_765.25, epsilon = 0.02);
    assert_abs_diff_eq!(r.temperature_c, -10.5, epsilon = 1e-4);
}

#[test]
fn bmp581_before_init_is_not_ready() {
    // Standby, pressure disabled: the data registers read zero.
    let (mut baro, mut bus, _, _) = bmp581();
    assert_eq!(baro.read_sample(&mut bus), Err(Error::NotReady));
}

#[test]
fn bmp581_absent_nacks() {
    let mut bus = FakeI2c::default();
    let mut baro = bmp581::Bmp581::new(bmp581::ADDR_SDO_LOW);
    let mut d = FakeDelay(Clock::default());
    assert_eq!(baro.init(&mut bus, &mut d, &Default::default()), Err(Error::Bus(BusError::NoAcknowledge)));
}

#[test]
fn bmp581_rejects_an_odr_the_oversampling_does_not_fit() {
    let (mut baro, mut bus, _, mut d) = bmp581();
    let cfg = bmp581::Config {
        odr: bmp581::Odr::Hz240,
        pressure_osr: bmp581::Oversampling::X128,
        ..Default::default()
    };
    assert_eq!(baro.init(&mut bus, &mut d, &cfg), Err(Error::Config));
}

#[test]
fn bmp581_without_power_on_reset_is_not_ready() {
    let (mut baro, mut bus, chip, mut d) = bmp581();
    chip.borrow_mut().suppress_por = true;
    assert_eq!(baro.init(&mut bus, &mut d, &Default::default()), Err(Error::NotReady));
}

// ---------------------------------------------------------------------------
// MMC5983MA
// ---------------------------------------------------------------------------

fn mmc(field: [i32; 3], offset: [i32; 3]) -> (mmc5983ma::Mmc5983ma, FakeI2c, Rc<RefCell<FakeMmc5983ma>>, FakeDelay) {
    let chip = Rc::new(RefCell::new(FakeMmc5983ma::new(field, offset)));
    let mut bus = FakeI2c::default();
    bus.attach(mmc5983ma::ADDR, chip.clone());
    (mmc5983ma::Mmc5983ma::new(), bus, chip, FakeDelay(Clock::default()))
}

/// 16384 counts per gauss, 100 µT per gauss.
fn ut(counts: i32) -> f32 {
    counts as f32 / 163.84
}

#[test]
fn mmc5983ma_removes_the_bridge_offset() {
    let field = [8192, -4096, 30_000]; // 50 µT, −25 µT, 183 µT
    let offset = [700, -350, 1200];
    let (mut mag, mut bus, _, mut d) = mmc(field, offset);
    mag.init(&mut bus, &mut d, &Default::default()).unwrap();
    let off = mag.offset_ut();
    for i in 0..3 {
        assert_abs_diff_eq!(off[i], ut(offset[i]), epsilon = 1e-3);
    }
    let m = mag.read_sample(&mut bus).unwrap();
    for i in 0..3 {
        assert_abs_diff_eq!(m[i], ut(field[i]), epsilon = 1e-3);
    }
}

#[test]
fn mmc5983ma_continuous_mode_settings() {
    let (mut mag, mut bus, chip, mut d) = mmc([0; 3], [0; 3]);
    mag.init(&mut bus, &mut d, &Default::default()).unwrap();
    let c = chip.borrow();
    assert_eq!(c.ctrl[0], 0x20 | 0x04, "auto SET/RESET, measurement-done interrupt");
    assert_eq!(c.ctrl[1], 0x00, "8 ms bandwidth");
    assert_eq!(c.ctrl[2], 0x80 | (0b001 << 4) | 0x08 | 0b101, "100 Hz continuous, SET every 25");
    assert_eq!(c.polarity, 1, "left in SET polarity");
}

#[test]
fn mmc5983ma_wrong_product_id() {
    let (mut mag, mut bus, chip, mut d) = mmc([0; 3], [0; 3]);
    chip.borrow_mut().product_id = 0x00;
    assert_eq!(mag.init(&mut bus, &mut d, &Default::default()), Err(Error::WrongChip { found: 0x00 }));
}

#[test]
fn mmc5983ma_measurement_timeout() {
    let (mut mag, mut bus, chip, mut d) = mmc([0; 3], [0; 3]);
    chip.borrow_mut().stuck = true;
    assert_eq!(mag.init(&mut bus, &mut d, &Default::default()), Err(Error::NotReady));
}

#[test]
fn mmc5983ma_recalibrates_in_flight() {
    let (mut mag, mut bus, chip, mut d) = mmc([1000, 0, 0], [100, 0, 0]);
    mag.init(&mut bus, &mut d, &Default::default()).unwrap();
    chip.borrow_mut().offset = [-400, 50, 0]; // drifted with temperature
    assert_abs_diff_eq!(mag.read_sample(&mut bus).unwrap()[0], ut(1000 - 400 - 100), epsilon = 1e-3);
    mag.calibrate_offset(&mut bus, &mut d).unwrap();
    let m = mag.read_sample(&mut bus).unwrap();
    assert_abs_diff_eq!(m[0], ut(1000), epsilon = 1e-3);
    assert_abs_diff_eq!(m[1], 0.0, epsilon = 1e-3);
    assert_eq!(chip.borrow().ctrl[2] & 0x08, 0x08, "continuous mode restarted");
}

// ---------------------------------------------------------------------------
// Shared bus
// ---------------------------------------------------------------------------

#[test]
fn baro_and_mag_share_one_bus() {
    let baro_chip = Rc::new(RefCell::new(FakeBmp581::new()));
    let mag_chip = Rc::new(RefCell::new(FakeMmc5983ma::new([1638, 0, 0], [0; 3])));
    let mut bus = FakeI2c::default();
    bus.attach(bmp581::ADDR_SDO_LOW, baro_chip);
    bus.attach(mmc5983ma::ADDR, mag_chip);
    let mut d = FakeDelay(Clock::default());
    let mut baro = bmp581::Bmp581::new(bmp581::ADDR_SDO_LOW);
    let mut mag = mmc5983ma::Mmc5983ma::new();
    baro.init(&mut bus, &mut d, &Default::default()).unwrap();
    mag.init(&mut bus, &mut d, &Default::default()).unwrap();
    assert!(baro.read_sample(&mut bus).is_ok());
    assert_abs_diff_eq!(mag.read_sample(&mut bus).unwrap()[0], 10.0, epsilon = 0.01);
}

#[test]
fn remap_axes() {
    let v = [1.0, 2.0, 3.0];
    assert_eq!(ripwing_sensors::remap(v, [0, 1, 2], [1.0; 3]), v);
    assert_eq!(ripwing_sensors::remap(v, [1, 0, 2], [1.0, -1.0, -1.0]), [2.0, -1.0, -3.0]);
}
