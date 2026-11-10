//! High-rate flight data logging (e.g., to onboard SD/flash).
//!
//! IMPORTANT: the log record format defined here is also parsed by
//! the Python training tools in `tools/`. Keep a single source of
//! truth for the schema so firmware and analysis never drift.
//!
//! Current state: the *capture* side exists, the *storage* side does not.
//! Raw IMU samples are pushed into a lock-free SPSC queue at the driver
//! boundary (before filtering) and drained by the low-priority `logging`
//! task. Until the flight recorder exists, the drain is where they stop;
//! the on-disk record format is deliberately not defined yet so the Python
//! tools don't grow a dependency on a placeholder.

use heapless::spsc::{Consumer, Producer, Queue};
use ripwing_common::ImuSample;

/// Queue slots (capacity is one less). The logging task drains every 10 ms,
/// i.e. ~10 samples at 1 kHz; 64 gives several drain periods of slack
/// before a delayed drain starts dropping records.
pub const RAW_LOG_SLOTS: usize = 64;

pub type RawLogQueue = Queue<ImuSample, RAW_LOG_SLOTS>;
/// Producer half: owned by the driver boundary (sensor chain).
pub type RawLogProducer = Producer<'static, ImuSample, RAW_LOG_SLOTS>;
/// Consumer half: owned by the logging task.
pub type RawLogConsumer = Consumer<'static, ImuSample, RAW_LOG_SLOTS>;
