// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F10x USART model (`gd32f10x_usart.h`; GD32F10x User Manual, section 16): `STAT` at the
//! start of the block, one `DATA` register for both directions, `UEN` at `CTL0` bit 13, no
//! oversampling select.

#[cfg(target_arch = "arm")]
use crate::reg::Reg32;

pub(crate) const STAT: u32 = 0x00;
/// `DATA`: the transmit and the receive data register are one.
pub(crate) const TDATA: u32 = 0x04;
pub(crate) const RDATA: u32 = 0x04;
pub(crate) const BAUD: u32 = 0x08;
pub(crate) const CTL0: u32 = 0x0C;
pub(crate) const CTL1: u32 = 0x10;
pub(crate) const CTL2: u32 = 0x14;
pub(crate) const CTL0_UEN: u32 = 1 << 13;
/// `CTL0` frame bits beyond `WL`/`PM`/`PCEN` that a bring-up clears: none (bit 15 is reserved).
pub(crate) const CTL0_FRAME_EXTRA: u32 = 0;

/// Clear the sticky `STAT` flags the caller has just read (the line errors and `IDLEF`): the
/// clearing sequence is a `STAT` read followed by a `DATA` read, and the caller's read was the
/// first half. The byte read is discarded.
#[cfg(target_arch = "arm")]
#[inline(always)]
pub(crate) fn clear_flags(base: u32, _flags: u32) {
    let _ = Reg32::new(base, RDATA).read();
}
