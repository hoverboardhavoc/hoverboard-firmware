// SPDX-License-Identifier: GPL-3.0-or-later
//! The debug-hold bits in `DBG_CTL0` (`0xE004_2004`, the same bits on the GD32F10x and GD32F1x0).

use crate::reg::Reg32;

const DBG_BASE: u32 = 0xE004_2000;
const DBG_CTL0: u32 = 0x04;
const FWDGT_HOLD: u32 = 1 << 8;
const TIMER0_HOLD: u32 = 1 << 10;

/// Stop TIMER0's counter while the core is halted by a debugger, so a halt freezes the PWM
/// instead of leaving the bridge at its last duties with the counter running.
pub fn debug_hold_timer0() {
    Reg32::new(DBG_BASE, DBG_CTL0).set_bits(TIMER0_HOLD);
}

/// Stop the free watchdog while the core is halted by a debugger.
pub(crate) fn debug_hold_fwdgt() {
    Reg32::new(DBG_BASE, DBG_CTL0).set_bits(FWDGT_HOLD);
}
