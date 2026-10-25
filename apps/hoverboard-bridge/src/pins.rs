//! The SWD pins (`specs/hoverboard-bridge.md`, "Hardware and wiring"): SWCLK on GPIO6 push-pull, SWDIO on
//! GPIO7 with its direction switched by the output enable and the internal pull-up on, so the line
//! reads high in the turnarounds and with no target answering.

use esp_hal::gpio::{Flex, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::time::Instant;
use hoverboard_bridge_core::swd::SwdPins;

/// Nops per half clock period. The rate it gives is measured at boot and reported in `ESP_INFO`;
/// lowering the rate (raising this) is the first response to a high error count.
pub const HALF_PERIOD_NOPS: u32 = 25;

/// GPIO6/GPIO7 as the SWD port.
pub struct EspPins {
    clk: Output<'static>,
    dio: Flex<'static>,
}

impl EspPins {
    /// Take the two pins; SWCLK idles high, SWDIO driven high.
    pub fn new(
        clk: esp_hal::peripherals::GPIO6<'static>,
        dio: esp_hal::peripherals::GPIO7<'static>,
    ) -> Self {
        let clk = Output::new(clk, Level::High, OutputConfig::default());
        let mut dio = Flex::new(dio);
        // The pull is one IO_MUX setting (`FUN_WPU`) that both configs write, and the output
        // enable leaves it alone, so it holds through every direction change. Both configs carry
        // it: `OutputConfig::default()` has no pull and would clear it.
        dio.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
        dio.apply_output_config(&OutputConfig::default().with_pull(Pull::Up));
        dio.set_input_enable(true);
        dio.set_high();
        dio.set_output_enable(true);
        EspPins { clk, dio }
    }

    #[inline(always)]
    fn half(&self) {
        for _ in 0..HALF_PERIOD_NOPS {
            // SAFETY: a no-op instruction, used as a delay.
            unsafe { core::arch::asm!("nop") };
        }
    }
}

impl SwdPins for EspPins {
    fn swdio_out(&mut self) {
        self.dio.set_output_enable(true);
    }

    fn swdio_in(&mut self) {
        self.dio.set_output_enable(false);
    }

    fn write_bit(&mut self, bit: bool) {
        if bit {
            self.dio.set_high();
        } else {
            self.dio.set_low();
        }
        self.clk.set_low();
        self.half();
        self.clk.set_high();
        self.half();
    }

    fn read_bit(&mut self) -> bool {
        self.clk.set_low();
        self.half();
        let b = self.dio.is_high();
        self.clk.set_high();
        self.half();
        b
    }

    fn clock(&mut self) {
        self.clk.set_low();
        self.half();
        self.clk.set_high();
        self.half();
    }

    fn now_us(&mut self) -> u64 {
        Instant::now().duration_since_epoch().as_micros()
    }
}
