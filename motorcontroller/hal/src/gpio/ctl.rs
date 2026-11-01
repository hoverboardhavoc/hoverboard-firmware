// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F1x0 GPIO model: two-bit `CTL` mode, `OMODE` output type, two-bit `OSPD` speed and
//! `PUD` pull per pin, the per-pin alternate-function mux in `AFSEL0` (pins 0-7) and `AFSEL1`
//! (pins 8-15), set/reset through `BOP`, levels in `ISTAT` (GD32F1x0 User Manual, section 8).

use super::{InputPull, PinRole};
use crate::reg::Reg32;

const CTL: u32 = 0x00;
const OMODE: u32 = 0x04;
const OSPD: u32 = 0x08;
const PUD: u32 = 0x0C;
/// The input-level register.
pub(crate) const ISTAT: u32 = 0x10;
const BOP: u32 = 0x18;
const AFSEL0: u32 = 0x20;
const AFSEL1: u32 = 0x24;

const MODE_INPUT: u32 = 0;
const MODE_OUTPUT: u32 = 1;
const MODE_AF: u32 = 2;
const MODE_ANALOG: u32 = 3;
const OSPEED_50MHZ: u32 = 3;
const PUD_NONE: u32 = 0;
const PUD_PULLUP: u32 = 1;
const PUD_PULLDOWN: u32 = 2;

#[inline]
fn field2(port_base: u32, offset: u32, n: u32, value: u32) {
    Reg32::new(port_base, offset).modify(0x3 << (2 * n), value << (2 * n));
}

#[inline]
fn omode(port_base: u32, n: u32, open_drain: bool) {
    Reg32::new(port_base, OMODE).modify(1 << n, u32::from(open_drain) << n);
}

/// An alternate-function pin: AF mode, the role's pull, its `AFSEL` nibble, and for an output its
/// type and 50 MHz speed.
pub(crate) fn configure_af(port_base: u32, pin: u8, role: PinRole) {
    let n = u32::from(pin & 0x0F);
    let cfg = role.config();
    field2(port_base, CTL, n, MODE_AF);
    field2(
        port_base,
        PUD,
        n,
        if cfg.pull_up { PUD_PULLUP } else { PUD_NONE },
    );
    let (offset, within) = if n < 8 { (AFSEL0, n) } else { (AFSEL1, n - 8) };
    let shift = 4 * within;
    Reg32::new(port_base, offset).modify(0xF << shift, cfg.af << shift);
    if cfg.output {
        omode(port_base, n, cfg.open_drain);
        field2(port_base, OSPD, n, OSPEED_50MHZ);
    }
}

/// An input with the requested pull.
pub(crate) fn configure_input(port_base: u32, pin: u8, pull: InputPull) {
    let n = u32::from(pin & 0x0F);
    field2(port_base, CTL, n, MODE_INPUT);
    let pud = match pull {
        InputPull::Floating => PUD_NONE,
        InputPull::PullUp => PUD_PULLUP,
        InputPull::PullDown => PUD_PULLDOWN,
    };
    field2(port_base, PUD, n, pud);
}

/// Analog mode, no pull.
pub(crate) fn configure_analog(port_base: u32, pin: u8) {
    let n = u32::from(pin & 0x0F);
    field2(port_base, CTL, n, MODE_ANALOG);
    field2(port_base, PUD, n, PUD_NONE);
}

/// A push-pull output at 50 MHz.
pub(crate) fn configure_output(port_base: u32, pin: u8) {
    let n = u32::from(pin & 0x0F);
    field2(port_base, CTL, n, MODE_OUTPUT);
    omode(port_base, n, false);
    field2(port_base, OSPD, n, OSPEED_50MHZ);
}

/// An open-drain output at 50 MHz, with the internal pull-up if asked (the I2C bus-recovery
/// clocking).
pub(crate) fn configure_output_od(port_base: u32, pin: u8, pull_up: bool) {
    let n = u32::from(pin & 0x0F);
    field2(port_base, CTL, n, MODE_OUTPUT);
    omode(port_base, n, true);
    field2(port_base, OSPD, n, OSPEED_50MHZ);
    field2(
        port_base,
        PUD,
        n,
        if pull_up { PUD_PULLUP } else { PUD_NONE },
    );
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
