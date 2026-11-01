// SPDX-License-Identifier: GPL-3.0-or-later
//! GPIO: typed pins, the input group, and the pin-mode writes the bring-ups make.
//!
//! Two register models, one per build (specs/hal.md, section 3.1):
//!
//! - `crl` (GD32F10x): the `CTL0`/`CTL1` nibbles (ST's `CRL`/`CRH`) on APB2; alternate functions
//!   are fixed per pin, moved only by the AFIO remap ([`crate::afio`]).
//! - `ctl` (GD32F1x0): `CTL`/`OMODE`/`OSPD`/`PUD` on AHB with the per-pin `AFSEL0`/`AFSEL1` mux.
//!
//! Both models expose the same crate-internal functions, so the code above them names no model.

use core::marker::PhantomData;

#[cfg(hal_gpio = "crl")]
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

/// The alternate-function number a USART pin takes on the F1x0 mux (`AFSEL` nibble). The F10x has
/// no per-pin mux and ignores it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsartAf {
    /// AF0 (USART0 on PB6/PB7).
    Af0,
    /// AF1 (USART1 on PA2/PA3).
    Af1,
}

impl UsartAf {
    /// The `AFSEL` nibble.
    #[inline]
    pub(crate) const fn nibble(self) -> u32 {
        match self {
            UsartAf::Af0 => 0,
            UsartAf::Af1 => 1,
        }
    }
}

/// The role an alternate-function pin is configured for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinRole {
    /// A USART transmit pin: AF push-pull.
    Tx(UsartAf),
    /// A USART receive pin: an input (F10x floating input; F1x0 AF mode, no pull).
    Rx(UsartAf),
    /// An I2C pin: AF open-drain with the pull-up (F1x0 AF1).
    I2cAfOpenDrain,
    /// SPI SCK, MOSI or a hardware NSS: AF push-pull (F1x0 AF0).
    SpiAfPushPull,
    /// SPI MISO: an input (F1x0 AF0).
    SpiInput,
    /// An advanced-timer gate pin: AF push-pull (F1x0 AF2).
    TimerAfPushPull,
    /// A general-timer PWM pin: AF push-pull (F1x0 AF2).
    GenTimerAfPushPull,
}

/// What a [`PinRole`] asks of the pin's mode registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PinConfig {
    /// Driven by the peripheral (else an input).
    pub output: bool,
    /// Open-drain (else push-pull), for an output.
    pub open_drain: bool,
    /// The pull-up on (the F1x0 `PUD`; the F10x AF modes have no pull).
    pub pull_up: bool,
    /// The F1x0 `AFSEL` nibble.
    pub af: u32,
}

impl PinRole {
    pub(crate) const fn config(self) -> PinConfig {
        let (output, open_drain, pull_up, af) = match self {
            PinRole::Tx(af) => (true, false, false, af.nibble()),
            PinRole::Rx(af) => (false, false, false, af.nibble()),
            PinRole::I2cAfOpenDrain => (true, true, true, 1),
            PinRole::SpiAfPushPull => (true, false, false, 0),
            PinRole::SpiInput => (false, false, false, 0),
            PinRole::TimerAfPushPull | PinRole::GenTimerAfPushPull => (true, false, false, 2),
        };
        PinConfig {
            output,
            open_drain,
            pull_up,
            af,
        }
    }
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

/// A GPIO pin in mode `MODE`, its port clock already enabled ([`crate::Chip::pin`]).
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

    /// The packed pin byte.
    #[inline]
    pub const fn pin(&self) -> u8 {
        self.pin
    }

    /// The port's register base.
    #[inline]
    pub(crate) const fn port_base(&self) -> u32 {
        self.port_base
    }

    /// As a push-pull output.
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<Output<PushPull>> {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roles_ask_for_the_af_numbers_of_the_f1x0_mux() {
        assert_eq!(PinRole::Tx(UsartAf::Af1).config().af, 1);
        assert_eq!(PinRole::Rx(UsartAf::Af0).config().af, 0);
        assert_eq!(PinRole::I2cAfOpenDrain.config().af, 1);
        assert_eq!(PinRole::SpiAfPushPull.config().af, 0);
        assert_eq!(PinRole::TimerAfPushPull.config().af, 2);
        assert_eq!(PinRole::GenTimerAfPushPull.config().af, 2);
    }

    #[test]
    fn only_i2c_is_open_drain_and_pulled_up() {
        for role in [
            PinRole::Tx(UsartAf::Af1),
            PinRole::Rx(UsartAf::Af1),
            PinRole::SpiAfPushPull,
            PinRole::SpiInput,
            PinRole::TimerAfPushPull,
            PinRole::GenTimerAfPushPull,
        ] {
            let c = role.config();
            assert!(!c.open_drain && !c.pull_up, "{role:?}");
        }
        let i2c = PinRole::I2cAfOpenDrain.config();
        assert!(i2c.output && i2c.open_drain && i2c.pull_up);
    }

    #[test]
    fn inputs_are_not_driven() {
        assert!(!PinRole::Rx(UsartAf::Af1).config().output);
        assert!(!PinRole::SpiInput.config().output);
    }
}
