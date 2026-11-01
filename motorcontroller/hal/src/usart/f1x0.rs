// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F1x0 USART model (`gd32f1x0_usart.h`; GD32F1x0 User Manual, section 17): the `CTL`
//! registers at the start of the block, `STAT` at `0x1C`, separate `RDATA`/`TDATA`, `UEN` at
//! `CTL0` bit 0, and the sticky flags cleared through `INTC`.

#[cfg(target_arch = "arm")]
use crate::reg::Reg32;

pub(crate) const CTL0: u32 = 0x00;
pub(crate) const CTL1: u32 = 0x04;
pub(crate) const CTL2: u32 = 0x08;
pub(crate) const BAUD: u32 = 0x0C;
pub(crate) const STAT: u32 = 0x1C;
#[cfg(target_arch = "arm")]
const INTC: u32 = 0x20;
pub(crate) const RDATA: u32 = 0x24;
pub(crate) const TDATA: u32 = 0x28;
pub(crate) const CTL0_UEN: u32 = 1 << 0;
/// `CTL0` frame bits beyond `WL`/`PM`/`PCEN` that a bring-up clears: `OVSMOD` (bit 15), so the
/// receiver samples by 16 as the divisor assumes.
pub(crate) const CTL0_FRAME_EXTRA: u32 = 1 << 15;

/// Clear the sticky `STAT` flags in `flags` (the line errors and `IDLEF`). `INTC`'s `PEC`, `FEC`,
/// `NEC`, `OREC` and `IDLEC` sit at the positions of the flags they clear (bits 0-4), so the flags
/// are the clear mask. It is the only way to clear `NERR` on this model; there is no data-register
/// read clear.
#[cfg(target_arch = "arm")]
#[inline(always)]
pub(crate) fn clear_flags(base: u32, flags: u32) {
    Reg32::new(base, INTC).write(flags);
}
