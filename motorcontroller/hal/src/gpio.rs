// SPDX-License-Identifier: GPL-3.0-or-later
//! GPIO: typed pins, the input group, and the pin-mode writes the bring-ups make.
//!
//! Two register models, one per build (specs/hal.md, section 3.1):
//!
//! - `crl` (GD32F10x): the `CTL0`/`CTL1` nibbles (ST's `CRL`/`CRH`) on APB2; alternate functions
//!   are fixed per pin, moved only by the AFIO remap ([`crate::afio`]).
//! - `ctl` (GD32F1x0): `CTL`/`OMODE`/`OSPD`/`PUD` on AHB with the per-pin `AFSEL0`/`AFSEL1` mux.
//! - `mm32` (MM32SPIN05): the `crl` model's nibbles (MindMotion's `CRL`/`CRH`, the same `IDR` and
//!   `BSRR` offsets) on AHB, with the per-pin `AFRL`/`AFRH` mux the `crl` file adds for it.
//!
//! Both models expose the same crate-internal functions, so the code above them names no model.

use core::marker::PhantomData;

#[cfg(any(hal_gpio = "crl", hal_gpio = "mm32"))]
#[path = "gpio/crl.rs"]
mod model;
#[cfg(hal_gpio = "ctl")]
#[path = "gpio/ctl.rs"]
mod model;

pub(crate) use model::{
    configure_af, configure_analog, configure_input, configure_output, configure_output_od,
    read_pin, set_pin, ISTAT,
};

use crate::reg::Reg32;

/// A pin as the store and the settings file name it (`specs/config-read-once.md` section 3): its
/// port, `0` = A to `5` = F, and its number within the port, `0..=15`. The HAL writes these into
/// the registers and checks neither.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinId {
    /// The port, `0` = A to `5` = F.
    pub port: u8,
    /// The pin number within the port.
    pub pin: u8,
}

/// A pin and the alternate function it is routed to: the per-pin mux number the GD32F1x0
/// (`AFSEL`) and the MM32 (`AFR`) write. The GD32F10x has no per-pin mux and never reads `af`; its
/// routing is the AFIO remap ([`crate::afio`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinAf {
    /// The port, `0` = A to `5` = F.
    pub port: u8,
    /// The pin number within the port.
    pub pin: u8,
    /// The alternate-function number.
    pub af: u8,
}

impl PinAf {
    /// The pin without its alternate function.
    #[inline]
    pub const fn id(self) -> PinId {
        PinId {
            port: self.port,
            pin: self.pin,
        }
    }
}

/// What an alternate-function pin's mode registers are set to; the AF number is the pin's own
/// ([`PinAf::af`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AfMode {
    /// Driven by the peripheral, push-pull at 50 MHz: a USART TX, a timer output, SPI SCK/MOSI.
    PushPull,
    /// Driven by the peripheral, open-drain at 50 MHz with the pull-up (the F1x0 `PUD`; the F10x
    /// AF modes have no pull): an I2C line.
    OpenDrainPullUp,
    /// An input the peripheral reads (F10x and MM32 floating input; F1x0 AF mode, no pull): a
    /// USART RX, SPI MISO.
    Input,
}

/// The pull an input takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum InputPull {
    Floating,
    PullUp,
    PullDown,
}

/// Type state: a floating input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Floating;
/// Type state: an input with the pull-up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PullUp;
/// Type state: an input with the pull-down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PullDown;
/// Type state: a push-pull output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushPull;
/// Type state: an input with pull `PULL`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Input<PULL> {
    _pull: PhantomData<PULL>,
}
/// Type state: an output of type `OTYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Output<OTYPE> {
    _otype: PhantomData<OTYPE>,
}

/// A GPIO pin in mode `MODE`, its port clock already enabled ([`crate::Chip::pin`]). `pin` is the
/// number within the port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pin<MODE> {
    port_base: u32,
    pin: u8,
    _mode: PhantomData<MODE>,
}

impl<MODE> Pin<MODE> {
    #[inline]
    pub(crate) const fn new(port_base: u32, pin: u8) -> Self {
        Self {
            port_base,
            pin,
            _mode: PhantomData,
        }
    }

    /// As a push-pull output. On the GD32F10x a JTAG-only pin (PA15, PB3, PB4) is freed first
    /// ([`crate::afio`]), keeping SWD.
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<Output<PushPull>> {
        #[cfg(hal_gpio = "crl")]
        crate::afio::free_if_jtag(self.port_base, self.pin);
        configure_output(self.port_base, self.pin);
        Pin::new(self.port_base, self.pin)
    }

    /// As a floating input.
    #[inline]
    pub fn into_floating_input(self) -> Pin<Input<Floating>> {
        configure_input(self.port_base, self.pin, InputPull::Floating);
        Pin::new(self.port_base, self.pin)
    }

    /// As an input with the pull-up.
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<Input<PullUp>> {
        configure_input(self.port_base, self.pin, InputPull::PullUp);
        Pin::new(self.port_base, self.pin)
    }

    /// As an input with the pull-down.
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<Input<PullDown>> {
        configure_input(self.port_base, self.pin, InputPull::PullDown);
        Pin::new(self.port_base, self.pin)
    }
}

impl embedded_hal::digital::ErrorType for Pin<Output<PushPull>> {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for Pin<Output<PushPull>> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        set_pin(self.port_base, self.pin, true);
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        set_pin(self.port_base, self.pin, false);
        Ok(())
    }
}

impl<PULL> embedded_hal::digital::ErrorType for Pin<Input<PULL>> {
    type Error = core::convert::Infallible;
}

impl<PULL> embedded_hal::digital::InputPin for Pin<Input<PULL>> {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read_pin(self.port_base, self.pin))
    }

    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!read_pin(self.port_base, self.pin))
    }
}

/// Lines in an [`InputGroup`].
pub const INPUT_GROUP_LINES: usize = 3;

/// Up to three input lines read together: line `i` is bit `i` of [`InputGroup::read`]. Resolved
/// once at bring-up to a port input register and a bit per line, so a read is three loads (the
/// period ISR reads the halls through one).
#[derive(Debug, Clone, Copy)]
pub struct InputGroup {
    lines: [(Reg32, u8); INPUT_GROUP_LINES],
}

impl InputGroup {
    pub(crate) fn resolve(lines: [(u32, u8); INPUT_GROUP_LINES]) -> Self {
        let mut out = [(Reg32::new(0, ISTAT), 0u8); INPUT_GROUP_LINES];
        for (slot, &(base, pin)) in out.iter_mut().zip(lines.iter()) {
            *slot = (Reg32::new(base, ISTAT), pin);
        }
        Self { lines: out }
    }

    /// The lines' levels as a code: bit `i` is line `i`.
    #[inline]
    pub fn read(&self) -> u8 {
        let mut code = 0u8;
        for (i, (istat, pin)) in self.lines.iter().enumerate() {
            code |= (((istat.read() >> u32::from(*pin)) & 1) as u8) << i;
        }
        code
    }
}
