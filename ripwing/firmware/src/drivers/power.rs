//! Battery voltage and current sense on ADC1.
//!
//! Read by the safety monitor itself, at its own 1 kHz, so the battery
//! verdict does not depend on any other task being alive (the safety
//! crate's design rule).
//!
//! # Never blocking the safety task
//!
//! A conversion takes ~20 µs (480-cycle sample at 24 MHz). Instead of
//! waiting for it, [`PowerAdc::poll`] collects the conversion started on the
//! previous tick (1 ms ago, long finished) and starts the next channel. The
//! two channels alternate, so each is sampled at 500 Hz, and a tick costs a
//! few register accesses.
//!
//! # Accuracy
//!
//! The ADC measures against VDDA. At boot [`PowerAdc::new`] measures VDDA
//! itself against the internal reference and its factory calibration, so a
//! 3.25 V supply is not read as a 1.5 % low battery. The pin's 100 nF and
//! the divider's 909 Ω source impedance leave the long sample time
//! ample settling.

use crate::board::{CurrentScale, CurrentSensePin, VbatSensePin, CURRENT_SCALE, VBAT_DIVIDER, VBAT_TRIM};
use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, Clock, SampleTime, Sequence},
        Adc,
    },
    pac::ADC1,
};

/// One finished measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PowerReading {
    /// Pack voltage [V].
    BatteryVolts(f32),
    /// Pack current [A], scaled by the ESC's current sensor.
    Amps(f32),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Channel {
    Vbat,
    Current,
}

pub struct PowerAdc {
    adc: Adc<ADC1>,
    vbat: VbatSensePin,
    current: CurrentSensePin,
    /// Measured VDDA [V].
    vdda: f32,
    /// The channel whose conversion is in flight, if any.
    in_flight: Option<Channel>,
}

impl PowerAdc {
    pub fn new(adc1: ADC1, vbat: VbatSensePin, current: CurrentSensePin) -> Self {
        // PCLK2 is 96 MHz: the HAL's default /2 would clock the ADC at
        // 48 MHz, past the F411's 36 MHz maximum. /4 gives 24 MHz.
        let cfg = AdcConfig::default()
            .clock(Clock::Pclk2_div_4)
            .default_sample_time(SampleTime::Cycles_480);
        let mut adc = Adc::adc1(adc1, true, cfg);
        adc.calibrate(); // VDDA from VREFINT and its factory calibration
        let vdda = adc.reference_voltage() as f32 / 1000.0;
        Self { adc, vbat, current, vdda, in_flight: None }
    }

    /// Measured VDDA [V], for diagnostics.
    pub fn vdda(&self) -> f32 {
        self.vdda
    }

    /// Collect the previous tick's conversion and start the next one.
    /// Returns the collected reading (every call after the first). The
    /// current channel is skipped when no current sensor is configured.
    pub fn poll(&mut self) -> Option<PowerReading> {
        let done = self.in_flight.take().map(|ch| {
            // Started a tick ago and finished ~20 µs after that: this
            // returns at once.
            self.adc.wait_for_conversion_sequence();
            let pin_v = self.adc.current_sample() as f32 * self.vdda / 4095.0;
            match ch {
                Channel::Vbat => PowerReading::BatteryVolts(pin_v * VBAT_DIVIDER * VBAT_TRIM),
                // Only started when a scale exists; NaN (ignored by the
                // battery monitor) rather than a panic in the safety task.
                Channel::Current => PowerReading::Amps(CURRENT_SCALE.map_or(f32::NAN, |s: CurrentScale| {
                    (pin_v - s.offset_volts) * s.amps_per_volt
                })),
            }
        });

        let next = match done {
            Some(PowerReading::BatteryVolts(_)) if CURRENT_SCALE.is_some() => Channel::Current,
            _ => Channel::Vbat,
        };
        match next {
            Channel::Vbat => self.adc.configure_channel(&self.vbat, Sequence::One, SampleTime::Cycles_480),
            Channel::Current => self.adc.configure_channel(&self.current, Sequence::One, SampleTime::Cycles_480),
        }
        self.adc.start_conversion();
        self.in_flight = Some(next);
        done
    }
}
