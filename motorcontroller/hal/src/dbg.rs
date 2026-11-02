// SPDX-License-Identifier: GPL-3.0-or-later
//! The debug-hold bits in `DBG_CTL0` (`0xE004_2004`, the same bits on the GD32F10x and GD32F1x0)
//! or, on the MM32SPIN05, `DBGMCU_CR` (`0x4001_3404`, MindMotion's `reg_dbg.h`), which has the
//! same bits: `IWDG_STOP` 8 and `TIM1_STOP` 10. The MM32's register takes a write only with its
//! clock on, `RCC_APB2ENR.DBGMCU` (bit 22), which MindMotion's `DBGMCU_Config` does not set: on the
//! bench MM32SPIN05 a write of `0x500` read back 0 until bit 22 was set, then `0x500`
//! (`specs/bench-work/mm32/hal-step3/logs/v02-dbgmcu-clock.txt`). Every write here sets it first.
//!
//! `TIMER7_HOLD` is bit 17 of the GD32F10x `DBG_CTL0` (GD32F10x User Manual Rev2.6 section 10.4.2).
//! It is set only on a chip whose descriptor carries TIMER7: on the GD32F1x0 the same bit is
//! `I2C2_HOLD` (GD32F1x0 User Manual Rev3.6 section 9.4.2), and the MM32SPIN05 has no TIMER7.

use crate::addr::PeriphLabel;
use crate::descriptor::DESCRIPTOR;
use crate::reg::Reg32;

#[cfg(hal_dbg = "gd")]
const DBG_BASE: u32 = 0xE004_2000;
#[cfg(hal_dbg = "mm32")]
const DBG_BASE: u32 = 0x4001_3400;
const DBG_CTL0: u32 = 0x04;
const FWDGT_HOLD: u32 = 1 << 8;
const TIMER0_HOLD: u32 = 1 << 10;
const TIMER7_HOLD: u32 = 1 << 17;

/// The timer holds of the compiled chip: `TIMER0_HOLD`, and `TIMER7_HOLD` where the chip carries
/// TIMER7.
const TIMER_HOLDS: u32 = if DESCRIPTOR.addrs.get(PeriphLabel::Timer7).is_some() {
    TIMER0_HOLD | TIMER7_HOLD
} else {
    TIMER0_HOLD
};

/// The MM32's debug-block clock, `RCC_APB2ENR.DBGMCU`, without which `DBGMCU_CR` ignores writes.
#[cfg(hal_dbg = "mm32")]
#[inline]
fn enable_dbg_clock() {
    const RCC_APB2ENR: u32 = 0x4002_1018;
    const APB2ENR_DBGMCU: u32 = 1 << 22;
    Reg32::new(RCC_APB2ENR, 0).set_bits(APB2ENR_DBGMCU);
}

/// Stop every advanced timer's counter (TIMER0, and TIMER7 where the chip carries it) while the
/// core is halted by a debugger, so a halt freezes the PWM instead of leaving a bridge at its last
/// duties with the counter running. One read-modify-write of `DBG_CTL0`.
pub fn debug_hold_timers() {
    #[cfg(hal_dbg = "mm32")]
    enable_dbg_clock();
    Reg32::new(DBG_BASE, DBG_CTL0).set_bits(TIMER_HOLDS);
}

/// Stop the free watchdog while the core is halted by a debugger.
pub(crate) fn debug_hold_fwdgt() {
    #[cfg(hal_dbg = "mm32")]
    enable_dbg_clock();
    Reg32::new(DBG_BASE, DBG_CTL0).set_bits(FWDGT_HOLD);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timer7_hold_is_set_exactly_where_the_chip_carries_timer7() {
        let two = DESCRIPTOR.adv_timers == 2;
        assert_eq!(TIMER_HOLDS & TIMER7_HOLD != 0, two);
        assert_ne!(TIMER_HOLDS & TIMER0_HOLD, 0);
        assert_eq!(TIMER_HOLDS & !(TIMER0_HOLD | TIMER7_HOLD), 0);
    }
}
