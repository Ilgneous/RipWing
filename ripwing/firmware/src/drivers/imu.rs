//! ICM-42688-P IMU driver: data-ready triggered, DMA-transferred.
//!
//! Three layers, in the order a sample passes through them:
//!
//! 1. **Configuration** ([`configure`]) — blocking SPI, once, in `init`.
//!    Puts the sensor on its own 1 kHz clock with INT1 pulsing on every new
//!    sample. From then on the *sensor* sets the system's pace.
//!
//! 2. **Transfer** ([`ImuDma`]) — on each data-ready edge, start an SPI DMA
//!    burst read of the whole data block; on DMA completion, parse it. The
//!    CPU spends a few hundred cycles arming and a few hundred finishing; the
//!    ~12 µs on the wire is the DMA controller's time, not ours.
//!
//! 3. **Driver boundary** ([`FrontEnd`]) — raw sample in, filtered sample
//!    out. The raw sample is logged *here*, before filtering, because the
//!    anomaly detector wants the spikes the median throws away.
//!
//! # Why the transfer overlaps computation
//!
//! The DMA-complete handler parses the frame into an `ImuSample` *by value*
//! and hands that copy to the estimator task. The RX DMA is re-armed on the
//! other buffer before the handler returns. So the moment compute for sample
//! `n` starts, the hardware is already free to fetch sample `n+1` — if the
//! next data-ready edge arrives while the estimator or controller are still
//! running, its (higher-priority) handler preempts them, kicks the DMA, and
//! returns, and the transfer proceeds underneath the computation instead of
//! in series with it. The critical path per sample becomes
//! `max(transfer, compute)`, not `transfer + compute`.

use crate::board::{ImuCsPin, IMU_AXIS, IMU_SIGN};
use crate::telemetry::logger::RawLogProducer;
use ripwing_common::ImuSample;
use ripwing_filter::{ImuFilter, NoiseMeter3};
use stm32f4xx_hal::{
    dma::{
        config::DmaConfig, DMAError, DmaFlag, MemoryToPeripheral, PeripheralToMemory, Stream0,
        Stream3, StreamsTuple, Transfer,
    },
    pac::{DMA2, SPI1},
    spi::{Rx, Spi, Tx},
    ClearFlags, ReadFlags,
};

// ---- Register map (bank 0) --------------------------------------------------

mod reg {
    pub const DEVICE_CONFIG: u8 = 0x11;
    pub const INT_CONFIG: u8 = 0x14;
    pub const TEMP_DATA1: u8 = 0x1D; // first byte of the burst
    pub const PWR_MGMT0: u8 = 0x4E;
    pub const GYRO_CONFIG0: u8 = 0x4F;
    pub const ACCEL_CONFIG0: u8 = 0x50;
    pub const INT_CONFIG1: u8 = 0x64;
    pub const INT_SOURCE0: u8 = 0x65;
    pub const WHO_AM_I: u8 = 0x75;
}

const READ: u8 = 0x80;
const WHO_AM_I_VALUE: u8 = 0x47;

/// ±2000 °/s full scale, 1 kHz ODR.
const GYRO_CONFIG0_VALUE: u8 = 0b0000_0110; // FS_SEL[7:5]=000, ODR[3:0]=0110
/// ±16 g full scale, 1 kHz ODR. The widest range, deliberately: an
/// accelerometer that clips under prop vibration reports a biased mean, and
/// the attitude estimator integrates that bias into a tilt error.
const ACCEL_CONFIG0_VALUE: u8 = 0b0000_0110; // FS_SEL[7:5]=000, ODR[3:0]=0110
/// Gyro and accel both in low-noise mode.
const PWR_MGMT0_VALUE: u8 = 0b0000_1111;
/// INT1: pulsed, push-pull, active high.
const INT_CONFIG_VALUE: u8 = 0b0000_0011;
/// INT_ASYNC_RESET cleared (the datasheet requires this for INT1 to behave),
/// 100 µs pulse.
const INT_CONFIG1_VALUE: u8 = 0x00;
/// Route UI data-ready to INT1, nothing else.
const INT_SOURCE0_VALUE: u8 = 0b0000_1000;

/// Sensor output data rate. The whole control chain now runs at this rate,
/// on the sensor's clock.
pub const ODR_HZ: f32 = 1000.0;

/// 1 / sensitivity at ±2000 °/s (16.4 LSB per °/s), converted to rad/s.
const GYRO_SCALE: f32 = (1.0 / 16.4) * (core::f32::consts::PI / 180.0);
/// 1 / sensitivity at ±16 g (2048 LSB per g), converted to m/s².
const ACCEL_SCALE: f32 = 9.806_65 / 2048.0;

/// The sensor writes this into a data register that holds no valid sample
/// (e.g. the ~30 ms after the gyro is switched on).
const INVALID_RAW: i16 = i16::MIN;

// ---- Burst frame ------------------------------------------------------------

/// One burst: 1 address byte + TEMP_DATA1 (0x1D) through INT_STATUS (0x2D).
/// Ending on INT_STATUS clears the interrupt status as a side effect of the
/// read, so no second transaction is needed.
pub const FRAME_LEN: usize = 1 + 17;

/// What the TX DMA clocks out: a read command at the start register, then
/// dummy bytes to generate clocks for the auto-incrementing read.
static READ_CMD: [u8; FRAME_LEN] = {
    let mut cmd = [0u8; FRAME_LEN];
    cmd[0] = reg::TEMP_DATA1 | READ;
    cmd
};

// Byte offsets within the received frame (offset 0 is the address slot).
const ACCEL_OFFSET: usize = 3;
const GYRO_OFFSET: usize = 9;

type TxTransfer = Transfer<Stream3<DMA2>, 3, Tx<SPI1>, MemoryToPeripheral, &'static [u8; FRAME_LEN]>;
type RxTransfer =
    Transfer<Stream0<DMA2>, 3, Rx<SPI1>, PeripheralToMemory, &'static mut [u8; FRAME_LEN]>;

// ---- Configuration ----------------------------------------------------------

#[derive(Debug, defmt::Format)]
pub enum InitError {
    /// Wrong or no device on the bus. `0x00`/`0xFF` usually means wiring.
    WhoAmI(u8),
    /// A configuration register did not read back what was written.
    Readback { reg: u8, wrote: u8, read: u8 },
    Spi,
}

/// Reset and configure the sensor over blocking SPI. Call once in `init`,
/// before the bus is handed to DMA.
pub fn configure(spi: &mut Spi<SPI1>, cs: &mut ImuCsPin, sysclk_hz: u32) -> Result<(), InitError> {
    let ms = |n: u32| cortex_m::asm::delay(n * (sysclk_hz / 1000));

    cs.set_high();
    ms(2); // power-on to first register access: 1 ms

    write_reg(spi, cs, reg::DEVICE_CONFIG, 0x01)?; // soft reset
    ms(2); // 1 ms after soft reset before any access

    let id = read_reg(spi, cs, reg::WHO_AM_I)?;
    if id != WHO_AM_I_VALUE {
        return Err(InitError::WhoAmI(id));
    }

    for &(r, v) in &[
        (reg::INT_CONFIG, INT_CONFIG_VALUE),
        (reg::INT_CONFIG1, INT_CONFIG1_VALUE),
        (reg::INT_SOURCE0, INT_SOURCE0_VALUE),
        (reg::GYRO_CONFIG0, GYRO_CONFIG0_VALUE),
        (reg::ACCEL_CONFIG0, ACCEL_CONFIG0_VALUE),
    ] {
        write_checked(spi, cs, r, v)?;
    }

    // Sensors on. No register access for 200 µs afterwards.
    write_reg(spi, cs, reg::PWR_MGMT0, PWR_MGMT0_VALUE)?;
    ms(1);
    let read = read_reg(spi, cs, reg::PWR_MGMT0)?;
    if read != PWR_MGMT0_VALUE {
        return Err(InitError::Readback {
            reg: reg::PWR_MGMT0,
            wrote: PWR_MGMT0_VALUE,
            read,
        });
    }
    Ok(())
}

fn write_checked(spi: &mut Spi<SPI1>, cs: &mut ImuCsPin, r: u8, v: u8) -> Result<(), InitError> {
    write_reg(spi, cs, r, v)?;
    let read = read_reg(spi, cs, r)?;
    if read != v {
        return Err(InitError::Readback { reg: r, wrote: v, read });
    }
    Ok(())
}

fn write_reg(spi: &mut Spi<SPI1>, cs: &mut ImuCsPin, r: u8, v: u8) -> Result<(), InitError> {
    let mut buf = [r & !READ, v];
    cs.set_low();
    let res = spi.transfer_in_place(&mut buf);
    cs.set_high();
    res.map_err(|_| InitError::Spi)
}

fn read_reg(spi: &mut Spi<SPI1>, cs: &mut ImuCsPin, r: u8) -> Result<u8, InitError> {
    let mut buf = [r | READ, 0];
    cs.set_low();
    let res = spi.transfer_in_place(&mut buf);
    cs.set_high();
    res.map_err(|_| InitError::Spi)?;
    Ok(buf[1])
}

// ---- DMA transfer engine ------------------------------------------------------

/// When a sample was taken: captured in the data-ready handler, carried with
/// the transfer, and attached to the sample when it lands.
#[derive(Clone, Copy, Debug)]
pub struct DrdyStamp {
    /// DWT cycle count at the data-ready edge. Sub-µs resolution, for
    /// measuring the sample interval (`dt`) and end-to-end latency.
    pub cycles: u32,
    /// Monotonic time [µs] at the edge — the same clock the safety monitor
    /// uses, so its staleness check compares like with like.
    pub timestamp_us: u32,
}

/// A parsed sample plus the edge it came from.
#[derive(Clone, Copy, Debug)]
pub struct ImuFrame {
    pub raw: ImuSample,
    pub stamp: DrdyStamp,
}

/// Outcome of a data-ready edge.
pub enum StartResult {
    Started,
    /// The previous transfer has not completed. This edge's sample is lost
    /// (the next edge overwrites the registers).
    Busy,
    /// The previous transfer was stuck for several edges and has been
    /// aborted; the bus is re-armed for the next edge.
    Recovered,
}

/// Outcome of a DMA interrupt.
pub enum Completion {
    Sample(ImuFrame),
    /// Transfer finished but the data was invalid (sensor still starting,
    /// or a corrupted frame).
    Rejected,
    /// DMA transfer error. Bus re-armed.
    BusError,
    /// Interrupt with nothing to do.
    Spurious,
}

/// After this many consecutive busy edges, assume the transfer is lost
/// (e.g. a DMA error interrupt that never came) and reset the bus.
const STUCK_EDGES: u8 = 3;

/// Owns SPI1 + its two DMA streams + CS, and runs one burst read per
/// data-ready edge.
///
/// The RX stream is always armed (on one of two buffers) between transfers.
/// With SPI as master, an armed RX stream is inert — nothing arrives until TX
/// generates clocks — so arming it early costs nothing and means the
/// data-ready handler only has to drop CS and kick TX.
pub struct ImuDma {
    cs: ImuCsPin,
    tx: TxTransfer,
    rx: RxTransfer,
    /// The buffer not currently owned by the RX stream.
    spare: Option<&'static mut [u8; FRAME_LEN]>,
    in_flight: Option<DrdyStamp>,
    busy_edges: u8,
}

impl ImuDma {
    pub fn new(
        spi: Spi<SPI1>,
        cs: ImuCsPin,
        dma2: DMA2,
        buf_a: &'static mut [u8; FRAME_LEN],
        buf_b: &'static mut [u8; FRAME_LEN],
    ) -> Self {
        let streams = StreamsTuple::new(dma2);
        let (tx_dir, rx_dir) = spi.use_dma().txrx();

        let mut rx = Transfer::init_peripheral_to_memory(
            streams.0,
            rx_dir,
            buf_a,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .transfer_complete_interrupt(true)
                .transfer_error_interrupt(true)
                // Above TX, so a received byte is always drained before the
                // next one can overrun the SPI data register.
                .priority(stm32f4xx_hal::dma::config::Priority::VeryHigh),
        );
        let tx = Transfer::init_memory_to_peripheral(
            streams.3,
            tx_dir,
            &READ_CMD,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .priority(stm32f4xx_hal::dma::config::Priority::High),
        );
        // Arm RX now; TX is started per edge.
        rx.start(|_| {});

        Self {
            cs,
            tx,
            rx,
            spare: Some(buf_b),
            in_flight: None,
            busy_edges: 0,
        }
    }

    /// Data-ready edge: start a burst read. Called from the EXTI handler.
    pub fn start(&mut self, stamp: DrdyStamp) -> StartResult {
        if self.in_flight.is_some() {
            self.busy_edges = self.busy_edges.saturating_add(1);
            if self.busy_edges < STUCK_EDGES {
                return StartResult::Busy;
            }
            self.abort();
            return StartResult::Recovered;
        }
        self.busy_edges = 0;
        self.in_flight = Some(stamp);
        self.cs.set_low();
        // Non-double-buffered `next_transfer` reloads the length and
        // re-enables the stream — i.e. restarts the clocks. It cannot fail in
        // this mode.
        let _ = self.tx.next_transfer(&READ_CMD);
        StartResult::Started
    }

    /// RX DMA interrupt: finish the transfer and parse the frame.
    pub fn complete(&mut self) -> Completion {
        let flags = self.rx.flags();
        self.rx
            .clear_flags(DmaFlag::TransferComplete | DmaFlag::TransferError | DmaFlag::HalfTransfer);

        if flags.contains(DmaFlag::TransferError) {
            self.abort();
            return Completion::BusError;
        }
        if !flags.contains(DmaFlag::TransferComplete) {
            return Completion::Spurious;
        }

        // RX complete means the last byte has been clocked in: the bus is
        // idle, so release CS.
        self.cs.set_high();
        let Some(stamp) = self.in_flight.take() else {
            return Completion::Spurious;
        };

        // Swap buffers: RX re-arms on the spare, we get the filled one.
        let Some(filled) = self.rearm_rx() else {
            return Completion::BusError;
        };
        let parsed = parse(filled, stamp.timestamp_us);
        self.spare = Some(filled);

        match parsed {
            Some(raw) => Completion::Sample(ImuFrame { raw, stamp }),
            None => Completion::Rejected,
        }
    }

    /// Re-arm RX on the spare buffer and return the one it held.
    fn rearm_rx(&mut self) -> Option<&'static mut [u8; FRAME_LEN]> {
        let spare = self.spare.take()?;
        match self.rx.next_transfer(spare) {
            Ok((filled, _)) => Some(filled),
            // Unreachable without double buffering, but never lose a buffer.
            Err(DMAError::NotReady(b) | DMAError::SmallBuffer(b) | DMAError::Overrun(b)) => {
                self.spare = Some(b);
                None
            }
        }
    }

    /// Abandon whatever is in flight and leave the bus ready for the next
    /// edge.
    fn abort(&mut self) {
        self.cs.set_high();
        self.tx.pause(|_| {});
        self.rx.pause(|_| {});
        // With both streams stopped, flush the SPI receiver. A byte left in
        // the data register would be taken by the re-armed RX DMA as byte 0
        // of the next frame, misaligning every frame from then on.
        //
        // SAFETY: read-only access to SPI1 status/data while neither DMA
        // stream is enabled; this struct owns SPI1, so nothing else is using
        // it. Reading DR then SR is the reference-manual sequence that clears
        // RXNE and OVR.
        let spi = unsafe { &*SPI1::ptr() };
        let mut spins = 0u32;
        while spi.sr().read().bsy().bit_is_set() && spins < 1_000 {
            spins += 1;
        }
        let _ = spi.dr().read();
        let _ = spi.sr().read();
        if let Some(old) = self.rearm_rx() {
            self.spare = Some(old);
        }
        self.in_flight = None;
        self.busy_edges = 0;
    }
}

/// Decode a burst into SI units in the body frame. `None` if the sensor
/// flagged any axis invalid.
fn parse(buf: &[u8; FRAME_LEN], timestamp_us: u32) -> Option<ImuSample> {
    let word = |off: usize, axis: usize| i16::from_be_bytes([buf[off + 2 * axis], buf[off + 2 * axis + 1]]);
    let mut accel_chip = [0i16; 3];
    let mut gyro_chip = [0i16; 3];
    for i in 0..3 {
        accel_chip[i] = word(ACCEL_OFFSET, i);
        gyro_chip[i] = word(GYRO_OFFSET, i);
    }
    if accel_chip.contains(&INVALID_RAW) || gyro_chip.contains(&INVALID_RAW) {
        return None;
    }

    let mut accel = [0.0f32; 3];
    let mut gyro = [0.0f32; 3];
    for i in 0..3 {
        accel[i] = IMU_SIGN[i] * accel_chip[IMU_AXIS[i]] as f32 * ACCEL_SCALE;
        gyro[i] = IMU_SIGN[i] * gyro_chip[IMU_AXIS[i]] as f32 * GYRO_SCALE;
    }
    Some(ImuSample {
        gyro,
        accel,
        timestamp_us,
    })
}

// ---- Driver boundary: raw in, filtered out --------------------------------------

// Window lengths, chosen against the lag/noise table in
// `ripwing_filter::average` (1 kHz: N=2 → 0.5 ms / 0.71×, N=8 → 3.5 ms /
// 0.35×; the median-of-3 ahead of either adds ~1 ms).
//
// Measured noise, as of this commit, is the datasheet's — no airframe yet.
// Assuming ~390 Hz noise bandwidth for the sensor's reset-default on-chip
// filter at 1 kHz ODR (an estimate; the bench measurement replaces it):
//
//   gyro   2.8 m°/s/√Hz × √390 Hz ≈ 0.055 °/s RMS
//          vs 1 LSB = 0.061 °/s at ±2000 °/s  → noise ≈ 1 LSB
//   accel  70 µg/√Hz  × √390 Hz   ≈ 1.4 mg RMS
//          vs 1 LSB = 0.49 mg at ±16 g        → noise ≈ 3 LSB
//
// GYRO_WINDOW = 2. Electronic noise is already at quantization level, so a
// long window would buy nothing against it while every sample of delay costs
// phase in the rate loop (1 ms ≈ 18° at a 50 Hz crossover). N=2 is chosen
// over N=1 for where it puts its zero: exactly at Nyquist (500 Hz), which is
// where the PID's D-term gain on noise peaks. 0.5 ms for that is worth it;
// N=4 (+1 ms more, a second zero at 250 Hz) is not, until measured.
//
// ACCEL_WINDOW = 8. The accel feeds only the complementary filter, whose
// tilt correction is itself a 1 s (0.16 Hz) low-pass; 3.5 ms is 0.35% of
// that and invisible. The accel window therefore isn't phase-limited — its
// job is to keep vibration from pushing |a| outside the estimator's ±15%
// gate, which would make it coast on gyro alone. N=8 puts the first zero at
// 125 Hz, under typical motor fundamentals.
//
// Revisit with motors spinning, using the noise figures `diagnostics`
// prints once a second (raw vs filtered std per axis):
//   - filtered accel std approaching ~0.5 m/s² (a third of the gate):
//     go to 16. Lag still negligible next to τ.
//   - filtered gyro std high enough to show up in motor output: fix the
//     vibration mechanically or add a notch at the measured frequency
//     before going past N=4. A longer boxcar is the most expensive way to
//     remove a narrowband peak.
pub const GYRO_WINDOW: usize = 2;
pub const ACCEL_WINDOW: usize = 8;

/// Noise statistics over one reporting window, std dev per axis.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoiseReport {
    pub samples: u32,
    pub gyro_raw: [f32; 3],
    pub gyro_filt: [f32; 3],
    pub accel_raw: [f32; 3],
    pub accel_filt: [f32; 3],
}

/// Samples per noise report: one second at ODR.
const NOISE_WINDOW: u32 = ODR_HZ as u32;

/// The driver boundary. Everything upstream sees raw samples; everything
/// downstream sees filtered ones. The raw sample is logged here and nowhere
/// else.
pub struct FrontEnd {
    filter: ImuFilter<GYRO_WINDOW, ACCEL_WINDOW>,
    log: RawLogProducer,
    gyro_raw: NoiseMeter3,
    gyro_filt: NoiseMeter3,
    accel_raw: NoiseMeter3,
    accel_filt: NoiseMeter3,
}

impl FrontEnd {
    pub fn new(log: RawLogProducer) -> Self {
        Self {
            filter: ImuFilter::new(),
            log,
            gyro_raw: NoiseMeter3::new(),
            gyro_filt: NoiseMeter3::new(),
            accel_raw: NoiseMeter3::new(),
            accel_filt: NoiseMeter3::new(),
        }
    }

    /// Log the raw sample, return the filtered one.
    ///
    /// Returns whether the raw sample made it into the log queue; a full
    /// queue drops the record rather than blocking the control chain.
    #[inline]
    pub fn condition(&mut self, raw: &ImuSample) -> (ImuSample, bool) {
        let logged = self.log.enqueue(*raw).is_ok();
        let filtered = self.filter.update(raw);

        self.gyro_raw.update(raw.gyro);
        self.gyro_filt.update(filtered.gyro);
        self.accel_raw.update(raw.accel);
        self.accel_filt.update(filtered.accel);

        (filtered, logged)
    }

    /// A noise report once per window, then the meters restart.
    pub fn take_noise_report(&mut self) -> Option<NoiseReport> {
        if self.gyro_raw.count() < NOISE_WINDOW {
            return None;
        }
        let report = NoiseReport {
            samples: self.gyro_raw.count(),
            gyro_raw: self.gyro_raw.std_dev(),
            gyro_filt: self.gyro_filt.std_dev(),
            accel_raw: self.accel_raw.std_dev(),
            accel_filt: self.accel_filt.std_dev(),
        };
        self.gyro_raw.reset();
        self.gyro_filt.reset();
        self.accel_raw.reset();
        self.accel_filt.reset();
        Some(report)
    }
}
