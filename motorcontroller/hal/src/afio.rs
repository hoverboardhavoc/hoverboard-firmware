// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F10x AFIO remaps (the F10x has no per-pin AF mux; a peripheral moves to its other pins
//! by a field in `AFIO_PCF0`). Compiled only with the `crl` GPIO model.
//!
//! `PCF0.SWJ_CFG` is write-only (GD32F10x User Manual 7.5.9: "These bits are write-only"), so a
//! read-modify-write of `PCF0` cannot keep it: the read gives an undefined value and the write puts
//! that value back, which can re-enable the JTAG-DP and take PA15/PB3/PB4 back from a remap. Every
//! `PCF0` write therefore goes through [`write_pcf0`], which writes the SWJ value this HAL last
//! configured instead of the one it read. Found by `tools/golden-trace`'s
//! `gpio/route_general_pwm_pin_pb3_f10x`: with the field read as 0, the TIMER1 remap's
//! read-modify-write after the SWJ write left `SWJ_CFG` at 0b000, JTAG back on PB3.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::addr::PeriphLabel;
use crate::reg::Reg32;

const AFIO_BASE: u32 = 0x4001_0000;
const PCF0: u32 = 0x04;
const RCU_APB2EN: u32 = 0x18;
const APB2EN_AFEN: u32 = 1 << 0;

const PCF0_TIMER1_REMAP: u32 = 0b11 << 8;
const PCF0_TIMER1_PARTIAL0: u32 = 0b01 << 8;
const PCF0_SWJ_CFG: u32 = 0b111 << 24;
/// JTAG off, SW-DP on: frees PA15, PB3 and PB4 and keeps PA13/PA14 for SWD.
const PCF0_SWJ_SW_ONLY: u32 = 0b010 << 24;

/// The `SWJ_CFG` field every `PCF0` write carries: the reset value (full SWJ) until
/// [`set_swj_sw_only`] changes it.
static SWJ: AtomicU32 = AtomicU32::new(0);

#[inline]
fn enable_clock(rcu_base: u32) {
    Reg32::new(rcu_base, RCU_APB2EN).set_bits(APB2EN_AFEN);
}

/// Replace the bits in `mask` with `value`, keeping the other read-write fields and writing the
/// configured `SWJ_CFG` rather than the undefined value it reads.
fn write_pcf0(mask: u32, value: u32) {
    let pcf0 = Reg32::new(AFIO_BASE, PCF0);
    let kept = pcf0.read() & !(mask | PCF0_SWJ_CFG);
    pcf0.write(kept | (value & mask) | SWJ.load(Ordering::Relaxed));
}

/// Free the JTAG-only pins (PA15, PB3, PB4) and keep SWD.
pub(crate) fn set_swj_sw_only(rcu_base: u32) {
    enable_clock(rcu_base);
    SWJ.store(PCF0_SWJ_SW_ONLY, Ordering::Relaxed);
    write_pcf0(0, 0);
}

/// TIMER1 partial remap 0: CH0 on PA15, CH1 on PB3.
pub(crate) fn remap_timer1_partial0(rcu_base: u32) {
    enable_clock(rcu_base);
    write_pcf0(PCF0_TIMER1_REMAP, PCF0_TIMER1_PARTIAL0);
}

/// Write a peripheral's remap field in `PCF0` (`specs/config-read-once.md` section 3): the stored
/// `remap` key of the group that uses it, `0` = the peripheral's default pins. The fields (GD32F10x
/// User Manual 7.5.9): I2C0 bit 1, USART0 bit 2, USART1 bit 3, USART2 bits 5:4, TIMER0 bits 7:6.
/// A peripheral with no remap field is left alone. The value is written as stored, masked to its
/// field; `SWJ_CFG` keeps what this HAL configured ([`write_pcf0`]), so no remap can re-enable the
/// JTAG-DP.
pub fn remap(label: PeriphLabel, value: u8) {
    let (shift, width) = match label {
        PeriphLabel::I2c0 => (1, 1),
        PeriphLabel::Usart0 => (2, 1),
        PeriphLabel::Usart1 => (3, 1),
        PeriphLabel::Usart2 => (4, 2),
        PeriphLabel::Timer0 => (6, 2),
        _ => return,
    };
    let mask = ((1u32 << width) - 1) << shift;
    enable_clock(RCU_BASE);
    write_pcf0(mask, u32::from(value) << shift);
}

/// The RCU base, from the descriptor.
const RCU_BASE: u32 = match crate::descriptor::DESCRIPTOR.addrs.get(PeriphLabel::Rcu) {
    Some(b) => b,
    None => panic!("every descriptor carries the RCU base"),
};
