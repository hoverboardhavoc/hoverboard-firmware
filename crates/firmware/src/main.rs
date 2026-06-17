//! hoverboard-firmware, early bring-up.
//!
//! First bench target is a GD32F103C8T6 (F10x family, Cortex-M3, no FPU). For now
//! this drives the on-board LED on PC13 (active-low) to validate the toolchain and
//! the build -> ST-Link -> SWD flash path. Real control logic grows from here.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f1xx_hal::{pac, prelude::*};

// Firmware-side bindings of the generic link layer to runtime-hal's USART (Phase 2). Declared here
// so it is compiled and link-checked with the binary even though `main` stays the blinky bring-up;
// none of it is called from `main` yet (the scheduler wiring is the next on-hardware step).
mod link_glue;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Default clocks: internal 8 MHz HSI (avoids the GD32 HSE/PLL setup quirk).
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let mut delay = cp.SYST.delay(&clocks);

    loop {
        led.set_low(); // PC13 active-low: LED on
        delay.delay_ms(250u16);
        led.set_high(); // LED off
        delay.delay_ms(250u16);
    }
}
