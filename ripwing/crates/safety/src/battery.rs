//! Battery monitor: pack voltage and current into a judged battery status.
//!
//! Input is the pack voltage and (optionally) current, already in volts and
//! amps; the firmware owns the ADC, the divider ratio and the current
//! sensor's scale. This module decides what they mean.
//!
//! # Sag, and why a raw floor does not work
//!
//! A LiPo's terminal voltage drops under load: a 3S pack at 60 % can read
//! 10.2 V for the half second of a hard climb and 11.4 V again a moment
//! later. A floor compared against the raw reading would fire on every
//! punch-out. So the levels are judged on a low-passed voltage
//! (`filter_tau_s`), and a level is only declared once that filtered
//! voltage has stayed below its threshold for `hold_s`.
//!
//! # Levels, and what they do
//!
//! * [`BatteryLevel::Low`] / [`BatteryLevel::Critical`] are *warnings*.
//!   The safety monitor refuses to arm on them but never cuts the motors
//!   for them in flight: there is no auto-land, so a cut is a crash, while
//!   flying on a low pack is a risk the pilot can still act on.
//! * Levels latch: once Low, the pack stays Low even when the voltage
//!   recovers as the load comes off, so the warning does not flicker with
//!   throttle. Unplugging the pack ([`BatteryLevel::Absent`]) clears it.
//!
//! # Cell count
//!
//! Thresholds are per cell. The count is fixed in config or detected once
//! the pack has been present for `detect_settle_s`, as the smallest count
//! that keeps every cell at or below `max_cell_volts`
//! (`cells = ceil(V / 4.35)`). That is right for any charged or storage
//! pack; a pack drained below ~3.3 V/cell can be miscounted one low, which
//! is why a fixed count is better once the pack is known.

/// What the pack voltage says.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BatteryLevel {
    /// No pack: the board runs from USB, or the reading is not usable.
    #[default]
    Absent,
    /// Present, cell count still being determined.
    Detecting,
    Ok,
    /// Below `low_cell_volts`, sustained: land soon.
    Low,
    /// Below `critical_cell_volts`, sustained: land now.
    Critical,
}

/// The judged battery state, published each tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BatteryStatus {
    pub level: BatteryLevel,
    /// Pack voltage, low-passed (sag filtered) (V).
    pub volts: f32,
    /// Pack voltage as last measured (V).
    pub volts_raw: f32,
    /// Pack current (A), if a current sensor is fitted.
    pub amps: Option<f32>,
    /// Charge drawn since the pack was connected (mAh) (0 without a
    /// current sensor).
    pub mah_used: f32,
    /// Series cell count; 0 until known.
    pub cells: u8,
    /// Healthy enough to arm: present, counted, `Ok`, and resting at or
    /// above `arm_min_cell_volts`.
    pub ready_to_arm: bool,
}

impl BatteryStatus {
    /// Filtered voltage per cell (V), if the cell count is known.
    pub fn cell_volts(&self) -> Option<f32> {
        (self.cells > 0).then(|| self.volts / self.cells as f32)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BatteryConfig {
    /// Fixed series cell count, or `None` to detect it.
    pub cells: Option<u8>,
    /// Below this (V) the pack is treated as unplugged.
    pub present_min_volts: f32,
    /// Pack must be present this long (s) before the cell count is
    /// detected (lets the filter settle on the resting voltage).
    pub detect_settle_s: f32,
    /// Highest believable cell voltage, for detection (V). HV packs charge
    /// to 4.35 V per cell.
    pub max_cell_volts: f32,
    /// Arming needs at least this much, resting (V/cell).
    pub arm_min_cell_volts: f32,
    /// Warning threshold (V/cell).
    pub low_cell_volts: f32,
    /// Critical threshold (V/cell).
    pub critical_cell_volts: f32,
    /// Sag filter time constant (s).
    pub filter_tau_s: f32,
    /// Filtered voltage must stay below a threshold this long (s).
    pub hold_s: f32,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            cells: None,
            present_min_volts: 2.5,
            detect_settle_s: 0.5,
            max_cell_volts: 4.35,
            // 3.5 V/cell resting is the old 10.5 V floor on a 3S pack.
            arm_min_cell_volts: 3.5,
            // Under load: 3.5 is "land soon", 3.3 is "land now" (the pack
            // has little left and is close to the voltage knee).
            low_cell_volts: 3.5,
            critical_cell_volts: 3.3,
            // Long enough to ride through a punch-out; the hold adds the
            // rest. A slow drain still shows within a few seconds.
            filter_tau_s: 1.0,
            hold_s: 2.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BatteryMonitor {
    cfg: BatteryConfig,
    status: BatteryStatus,
    /// Time since the pack was connected (s).
    present_s: f32,
    /// How long the filtered voltage has been below each threshold (s).
    below_low_s: f32,
    below_critical_s: f32,
}

impl BatteryMonitor {
    pub const fn new(cfg: BatteryConfig) -> Self {
        Self {
            cfg,
            status: BatteryStatus {
                level: BatteryLevel::Absent,
                volts: 0.0,
                volts_raw: 0.0,
                amps: None,
                mah_used: 0.0,
                cells: 0,
                ready_to_arm: false,
            },
            present_s: 0.0,
            below_low_s: 0.0,
            below_critical_s: 0.0,
        }
    }

    pub fn status(&self) -> BatteryStatus {
        self.status
    }

    /// Fold in one measurement taken `dt` (s) after the previous one.
    pub fn update(&mut self, volts: f32, amps: Option<f32>, dt: f32) -> BatteryStatus {
        let dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
        let cfg = self.cfg;

        // Unplugged (or an unusable reading): forget the pack entirely, so
        // the next one is judged afresh.
        if !volts.is_finite() || volts < cfg.present_min_volts {
            *self = Self::new(cfg);
            self.status.volts_raw = if volts.is_finite() { volts } else { 0.0 };
            return self.status;
        }

        let s = &mut self.status;
        s.volts_raw = volts;
        if s.level == BatteryLevel::Absent {
            // Just connected: start the filter at the reading, not at 0.
            s.level = BatteryLevel::Detecting;
            s.volts = volts;
        } else {
            let alpha = dt / (cfg.filter_tau_s + dt);
            s.volts += alpha * (volts - s.volts);
        }
        self.present_s += dt;

        s.amps = amps.filter(|a| a.is_finite());
        if let Some(a) = s.amps {
            // A·s to mAh.
            s.mah_used += a * dt / 3.6;
        }

        if s.cells == 0 {
            if let Some(n) = cfg.cells {
                s.cells = n;
            } else if self.present_s >= cfg.detect_settle_s {
                // ceil(V / max_cell_volts), without a math crate.
                let n = (s.volts / cfg.max_cell_volts) as u8;
                s.cells = if n as f32 * cfg.max_cell_volts < s.volts { n + 1 } else { n };
            }
            if s.cells > 0 {
                s.level = BatteryLevel::Ok;
            }
        }

        if let Some(cell) = s.cell_volts() {
            self.below_low_s = if cell < cfg.low_cell_volts { self.below_low_s + dt } else { 0.0 };
            self.below_critical_s = if cell < cfg.critical_cell_volts { self.below_critical_s + dt } else { 0.0 };
            // Levels only ever get worse while the pack is connected.
            if self.below_critical_s >= cfg.hold_s {
                s.level = BatteryLevel::Critical;
            } else if self.below_low_s >= cfg.hold_s && s.level == BatteryLevel::Ok {
                s.level = BatteryLevel::Low;
            }
            s.ready_to_arm = s.level == BatteryLevel::Ok && cell >= cfg.arm_min_cell_volts;
        }
        *s
    }
}
