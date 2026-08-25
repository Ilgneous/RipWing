//! Panic handling.
//!
//! `panic-probe` halts the core, which is ideal on the bench and catastrophic
//! in flight: the timer peripherals keep driving whatever the ESCs were last
//! commanded, so a frozen board means motors stuck at their last throttle. A
//! vehicle that resets and falls is bad; a vehicle that freezes at 70% thrust
//! and flies into something is worse.
//!
//! So this handler, in order:
//!
//! 1. Forces outputs to a safe state. First, before anything that could
//!    itself fail.
//! 2. Records where the panic happened in RAM that survives a soft reset, so
//!    the next boot can report it.
//! 3. Reports over RTT if a probe is attached.
//! 4. Resets.
//!
//! To be clear about what this buys: a reset takes on the order of 100 ms and
//! the safety monitor correctly comes up `Disarmed`, so an airborne vehicle
//! is not saved by any of this. The handler makes a bad outcome less bad and
//! leaves evidence. The actual defense is not panicking: no `unwrap`, no
//! unchecked indexing, no arithmetic that can overflow on the flight path.

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::ptr::addr_of_mut;
use core::sync::atomic::{compiler_fence, Ordering};

/// Marks the record as ours rather than uninitialized RAM. Arbitrary; just
/// needs to be a value garbage is unlikely to match.
const PANIC_MAGIC: u32 = 0x5249_5057; // "RIPW"

/// After this many consecutive panic-resets, stop resetting and halt.
///
/// A panic inside `init` would otherwise reset, panic, reset forever, and the
/// loop is hard to observe because each cycle is short. Halting after a few
/// attempts leaves the board in a state you can actually attach to.
const MAX_PANIC_RESETS: u32 = 3;

/// What survives the reset. `repr(C)` with only `u32` fields, deliberately:
/// every bit pattern is a valid `u32`, so reading this out of uninitialized
/// RAM on a cold boot is well defined rather than UB.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PanicRecord {
    magic: u32,
    /// Source line of the panic, or 0 if the location was unavailable.
    pub line: u32,
    /// Consecutive panic-resets so far.
    pub consecutive: u32,
}

/// Lives in `.uninit`, which `cortex-m-rt` does not zero at startup, so the
/// contents survive a soft reset. (`.bss` and `.data` are both initialized on
/// boot and would lose this.)
#[link_section = ".uninit.RIPWING_PANIC"]
static mut PANIC_RECORD: MaybeUninit<PanicRecord> = MaybeUninit::uninit();

/// Read and clear the panic record. Call once early in `init`.
///
/// Returns `Some` if the previous boot ended in a panic. Clearing on read
/// means a later clean boot does not keep re-reporting an old fault.
pub fn take_panic_record() -> Option<PanicRecord> {
    // SAFETY: single-threaded context (called from `init` before any task
    // runs), and every field is a `u32`, so any bit pattern read from
    // uninitialized RAM is a valid value rather than UB.
    unsafe {
        let ptr = addr_of_mut!(PANIC_RECORD) as *mut PanicRecord;
        let record = ptr.read_volatile();

        let valid = record.magic == PANIC_MAGIC;

        // Clear the magic either way: on a cold boot this turns garbage into
        // a definitively-empty record.
        ptr.write_volatile(PanicRecord {
            magic: 0,
            line: 0,
            consecutive: if valid { record.consecutive } else { 0 },
        });

        if valid {
            Some(record)
        } else {
            None
        }
    }
}

/// Force every actuator output to its safe state.
///
/// Called first thing in the panic handler, so it must not allocate, must not
/// lock, and must not panic. It cannot use RTIC resources — there is no task
/// context here — so it reaches the peripherals through `steal()`.
#[inline(always)]
fn safe_outputs() {
    // TODO: implement when DShot output exists. The shape will be:
    //
    //   let dp = unsafe { stm32f4xx_hal::pac::Peripherals::steal() };
    //   // Disable the timer driving the ESC outputs, then drive the pins to
    //   // the level the ESCs read as "no signal" so they disarm, rather than
    //   // leaving them at the last commanded duty cycle.
    //
    // Until motor output exists there is nothing to safe, but this call site
    // is deliberately first so the ordering is right the moment it is filled
    // in. Getting outputs safe must never sit behind logging or bookkeeping
    // that might itself fault.
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // 1. Outputs first, always.
    safe_outputs();

    let line = info.location().map(|l| l.line()).unwrap_or(0);

    // 2. Record the fault for the next boot, and count consecutive panics.
    // SAFETY: interrupts are irrelevant here — we never return, and nothing
    // else will observe this memory before the reset.
    let consecutive = unsafe {
        let ptr = addr_of_mut!(PANIC_RECORD) as *mut PanicRecord;
        let previous = ptr.read_volatile();
        let count = if previous.magic == PANIC_MAGIC {
            previous.consecutive.saturating_add(1)
        } else {
            1
        };
        ptr.write_volatile(PanicRecord {
            magic: PANIC_MAGIC,
            line,
            consecutive: count,
        });
        count
    };

    // Make sure the record is committed before we reset.
    compiler_fence(Ordering::SeqCst);

    // 3. Report, if a probe is listening.
    defmt::error!("PANIC at line {} (consecutive: {})", line, consecutive);

    if consecutive >= MAX_PANIC_RESETS {
        defmt::error!("{} consecutive panics — halting instead of resetting", consecutive);
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // 4. Spin briefly so the host has a chance to drain the RTT buffer, then
    // reset. RTT is a memory buffer the probe polls; resetting instantly can
    // discard the message we just wrote.
    for _ in 0..2_000_000 {
        cortex_m::asm::nop();
    }

    cortex_m::peripheral::SCB::sys_reset()
}
