#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod board;
mod control;
mod drivers;
mod telemetry;
mod anomaly;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    use stm32f4xx_hal::{
        gpio::{Output, PushPull, PC13},
        prelude::*,
    };

    systick_monotonic!(Mono, 1_000); // 1 kHz tick

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let rcc = dp.RCC.constrain();

        // BlackPill has a 25 MHz HSE crystal. Many Nucleo/Discovery
        // boards use 8 MHz — adjust use_hse() to match your board.
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(96.MHz()).freeze();

        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());

        let gpioc = dp.GPIOC.split();
        let led = gpioc.pc13.into_push_pull_output();

        defmt::info!("UAV FC booted");

        blink::spawn().ok();
        control_loop::spawn().ok();

        (Shared {}, Local { led })
    }

    /// Low-priority heartbeat. Confirms the scheduler is alive.
    #[task(local = [led], priority = 1)]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(500.millis()).await;
        }
    }

    /// High-priority control loop. Runs at 1 kHz.
    ///
    /// Keep this the highest-priority periodic task: it must meet a
    /// hard deadline every tick. ML inference and telemetry run at
    /// lower priority so they can be preempted by this.
    #[task(priority = 3)]
    async fn control_loop(_cx: control_loop::Context) {
        loop {
            // TODO: read IMU -> run AHRS -> run PID -> write ESC mixer
            Mono::delay(1.millis()).await;
        }
    }
}
