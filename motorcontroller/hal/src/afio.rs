// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F10x AFIO remaps (the F10x has no per-pin AF mux; a peripheral moves to its other pins
//! by a field in `AFIO_PCF0`). Compiled only with the `crl` GPIO model.

use crate::reg::Reg32;

const AFIO_BASE: u32 = 0x4001_0000;
const PCF0: u32 = 0x04;
const RCU_APB2EN: u32 = 0x18;
const APB2EN_AFEN: u32 = 1 << 0;

const PCF0_USART0_REMAP: u32 = 1 << 2;
const PCF0_TIMER1_REMAP: u32 = 0b11 << 8;
const PCF0_TIMER1_PARTIAL0: u32 = 0b01 << 8;
const PCF0_SWJ_CFG: u32 = 0b111 << 24;
/// JTAG off, SW-DP on: frees PA15, PB3 and PB4 and keeps PA13/PA14 for SWD.
const PCF0_SWJ_SW_ONLY: u32 = 0b010 << 24;

#[inline]
fn enable_clock(rcu_base: u32) {
    Reg32::new(rcu_base, RCU_APB2EN).set_bits(APB2EN_AFEN);
}

/// Free the JTAG-only pins (PA15, PB3, PB4) and keep SWD.
pub(crate) fn set_swj_sw_only(rcu_base: u32) {
    enable_clock(rcu_base);
    Reg32::new(AFIO_BASE, PCF0).modify(PCF0_SWJ_CFG, PCF0_SWJ_SW_ONLY);
}

/// TIMER1 partial remap 0: CH0 on PA15, CH1 on PB3.
pub(crate) fn remap_timer1_partial0(rcu_base: u32) {
    enable_clock(rcu_base);
    Reg32::new(AFIO_BASE, PCF0).modify(PCF0_TIMER1_REMAP, PCF0_TIMER1_PARTIAL0);
}

/// USART0 on PB6/PB7 instead of PA9/PA10.
pub(crate) fn set_usart0_remap(rcu_base: u32) {
    enable_clock(rcu_base);
    Reg32::new(AFIO_BASE, PCF0).set_bits(PCF0_USART0_REMAP);
}
