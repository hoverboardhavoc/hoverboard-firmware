// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F10x GPIO model: one 4-bit `MD`/`CTL` nibble per pin in `CTL0` (pins 0-7) and `CTL1`
//! (pins 8-15), set/reset through `BOP`, levels in `ISTAT` (GD32F10x User Manual, section 7).
//!
//! The MM32SPIN05 (`hal_gpio = "mm32"`) has the same nibbles, offsets and encodings (MindMotion's
//! `reg_gpio.h`: `CRL` `0x00`, `CRH` `0x04`, `IDR` `0x08`, `BSRR` `0x10`) and adds a per-pin
//! alternate-function mux, `AFRL` (`0x20`, pins 0-7) and `AFRH` (`0x24`, pins 8-15), a 4-bit AF
//! number per pin. Its alternate-function pins take the role's mux number as well as the nibble;
//! an AF input keeps the floating-input nibble, as MindMotion's own UART samples configure RX.

use super::{AfMode, InputPull};
use crate::reg::Reg32;

const CTL0: u32 = 0x00;
const CTL1: u32 = 0x04;
/// The input-level register.
pub(crate) const ISTAT: u32 = 0x08;
const BOP: u32 = 0x10;

/// Nibbles (`CTL << 2 | MD`).
const ANALOG: u32 = 0x0;
const OUTPUT_PP_50MHZ: u32 = 0x3;
const INPUT_FLOATING: u32 = 0x4;
const OUTPUT_OD_50MHZ: u32 = 0x7;
const INPUT_PULL: u32 = 0x8;
const AF_PP_50MHZ: u32 = 0xB;
const AF_OD_50MHZ: u32 = 0xF;

/// Write pin `pin`'s nibble.
#[inline]
fn set_nibble(port_base: u32, pin: u8, nibble: u32) {
    let n = u32::from(pin & 0x0F);
    let (offset, within) = if n < 8 { (CTL0, n) } else { (CTL1, n - 8) };
    let shift = 4 * within;
    Reg32::new(port_base, offset).modify(0xF << shift, nibble << shift);
}

/// The MM32 mux: pin `pin`'s AF number in `AFRL`/`AFRH`.
#[cfg(hal_gpio = "mm32")]
const AFRL: u32 = 0x20;
#[cfg(hal_gpio = "mm32")]
const AFRH: u32 = 0x24;

/// Write pin `pin`'s AF number into the MM32 mux.
#[cfg(hal_gpio = "mm32")]
#[inline]
fn set_af(port_base: u32, pin: u8, af: u32) {
    let n = u32::from(pin & 0x0F);
    let (offset, within) = if n < 8 { (AFRL, n) } else { (AFRH, n - 8) };
    let shift = 4 * within;
    Reg32::new(port_base, offset).modify(0xF << shift, af << shift);
}

/// An alternate-function pin. The F10x has no per-pin mux: `af` is not read, the mode picks the
/// nibble, and the AFIO remap is the caller's. The MM32 writes `af` into its mux first, then the
/// nibble.
#[cfg_attr(hal_gpio = "crl", allow(unused_variables))]
pub(crate) fn configure_af(port_base: u32, pin: u8, mode: AfMode, af: u8) {
    #[cfg(hal_gpio = "mm32")]
    set_af(port_base, pin, u32::from(af & 0x0F));
    let nibble = match mode {
        AfMode::Input => INPUT_FLOATING,
        AfMode::OpenDrainPullUp => AF_OD_50MHZ,
        AfMode::PushPull => AF_PP_50MHZ,
    };
    set_nibble(port_base, pin, nibble);
}

/// An input. The pull direction of the pulled mode is the output latch: `BOP` sets it for a
/// pull-up and resets it for a pull-down.
pub(crate) fn configure_input(port_base: u32, pin: u8, pull: InputPull) {
    let n = u32::from(pin & 0x0F);
    match pull {
        InputPull::Floating => set_nibble(port_base, pin, INPUT_FLOATING),
        InputPull::PullUp => {
            set_nibble(port_base, pin, INPUT_PULL);
            Reg32::new(port_base, BOP).write(1 << n);
        }
        InputPull::PullDown => {
            set_nibble(port_base, pin, INPUT_PULL);
            Reg32::new(port_base, BOP).write(1 << (n + 16));
        }
    }
}

/// Analog mode (nibble 0).
pub(crate) fn configure_analog(port_base: u32, pin: u8) {
    set_nibble(port_base, pin, ANALOG);
}

/// A push-pull output at 50 MHz.
pub(crate) fn configure_output(port_base: u32, pin: u8) {
    set_nibble(port_base, pin, OUTPUT_PP_50MHZ);
}

/// An open-drain output at 50 MHz (the I2C bus-recovery clocking). The F10x output modes have no
/// pull; the bus's external pull-ups hold the lines.
pub(crate) fn configure_output_od(port_base: u32, pin: u8, _pull_up: bool) {
    set_nibble(port_base, pin, OUTPUT_OD_50MHZ);
}

/// The pin's input level.
#[inline]
pub(crate) fn read_pin(port_base: u32, pin: u8) -> bool {
    Reg32::new(port_base, ISTAT).read() & (1 << (pin & 0x0F)) != 0
}

/// Drive the pin high or low through `BOP`.
#[inline]
pub(crate) fn set_pin(port_base: u32, pin: u8, high: bool) {
    let n = u32::from(pin & 0x0F);
    Reg32::new(port_base, BOP).write(if high { 1 << n } else { 1 << (n + 16) });
}
