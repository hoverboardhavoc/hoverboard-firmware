//! The SWD pins (`specs/hoverboard-bridge.md`, "Hardware and wiring"): SWCLK on GPIO6 push-pull, SWDIO on
//! GPIO7 with its direction switched by the output enable and the internal pull-up on, so the line
//! reads high in the turnarounds and with no target answering, and NRST on GPIO5 open-drain and
//! released except during a connect under reset (`specs/bridge-nrst.md` section 1).

use esp_hal::delay::Delay;
use esp_hal::gpio::{
    DriveMode, DriveStrength, Flex, InputConfig, Level, Output, OutputConfig, Pull,
};
use esp_hal::time::Instant;
use hoverboard_bridge_core::swd::SwdPins;

/// Nops per half clock period. The rate it gives is measured at boot and reported in `ESP_INFO`;
/// lowering the rate (raising this) is the first response to a high error count.
pub const HALF_PERIOD_NOPS: u32 = 25;

/// GPIO6/GPIO7 as the SWD port, GPIO5 as NRST.
pub struct EspPins {
    nrst: Output<'static>,
    clk: Output<'static>,
    dio: Flex<'static>,
}

impl EspPins {
    /// Take the three pins; NRST released, SWCLK idles high, SWDIO driven high. Called before the
    /// scheduler, the radio or any service starts, so NRST is released from that point on.
    pub fn new(
        nrst: esp_hal::peripherals::GPIO5<'static>,
        clk: esp_hal::peripherals::GPIO6<'static>,
        dio: esp_hal::peripherals::GPIO7<'static>,
    ) -> Self {
        // Open-drain with no pull: high is released, and the board's own NRST pull-up (about
        // 40 kOhm inside the GD32) holds the line. A board with no NRST wired, or whose own reset
        // circuit drives the line, sees nothing.
        let nrst = Output::new(
            nrst,
            Level::High,
            OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::None),
        );
        // Both pins at the weakest drive strength, as the probe firmware sets them
        // (`DAP_config.h`, `GPIO_DRIVE_CAP_0`): at the default 20 mA the edges ring on a long
        // harness and the target sees extra clock edges. The rover master (a GD32F103 on the
        // rover's leads) answered nothing at 20 mA and attaches cleanly at 5 mA (2026-10-25).
        let clk = Output::new(
            clk,
            Level::High,
            OutputConfig::default().with_drive_strength(DriveStrength::_5mA),
        );
        let mut dio = Flex::new(dio);
        // The pull is one IO_MUX setting (`FUN_WPU`) that both configs write, and the output
        // enable leaves it alone, so it holds through every direction change. Both configs carry
        // it: `OutputConfig::default()` has no pull and would clear it.
        dio.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
        dio.apply_output_config(
            &OutputConfig::default()
                .with_pull(Pull::Up)
                .with_drive_strength(DriveStrength::_5mA),
        );
        dio.set_input_enable(true);
        dio.set_high();
        dio.set_output_enable(true);
        EspPins { nrst, clk, dio }
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
    fn assert_reset(&mut self, asserted: bool) {
        if asserted {
            self.nrst.set_low();
        } else {
            // Open-drain: high releases the line.
            self.nrst.set_high();
        }
    }

    fn delay_us(&mut self, us: u32) {
        Delay::new().delay_micros(us);
    }

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
