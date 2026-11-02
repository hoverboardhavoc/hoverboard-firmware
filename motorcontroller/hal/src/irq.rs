// SPDX-License-Identifier: GPL-3.0-or-later
//! Interrupts: the compiled chip's flash vector table, the NVIC priorities the motor-era execution
//! model needs, and the ISR entry counters.
//!
//! The table is the vendor's vector list for the part (build.rs), linked by cortex-m-rt's `device`
//! feature. Nothing is copied to RAM and `VTOR` is never written (specs/hal.md, section 5). An image
//! defines its own handlers by name (`#[interrupt] fn ADC0_1()`, `#[exception] fn SysTick()`); the
//! HAL defines the vectors its serial drivers own: the USART vectors in [`crate::usart_rx`] and
//! the DMA channel vectors in the crate's `dma` module.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::chip::IrqLayout;
use crate::descriptor::DESCRIPTOR;
use crate::reg::Reg32;

include!(concat!(env!("OUT_DIR"), "/interrupts.rs"));

/// Motor period ISR: the most urgent (lower is more urgent; the parts implement the top 4 bits).
#[cfg(hal_core = "m3")]
pub const MOTOR_ISR_PRIORITY: u8 = 0x10;
/// The USART receive vectors (IDLE and line errors).
#[cfg(hal_core = "m3")]
pub const USART_RX_PRIORITY: u8 = 0x40;
/// The DMA receive and transmit channel vectors.
#[cfg(hal_core = "m3")]
pub const DMA_RX_PRIORITY: u8 = 0x70;
/// SysTick: the least urgent, so the communication vectors preempt the tick.
#[cfg(hal_core = "m3")]
pub const SYSTICK_PRIORITY: u8 = 0xF0;

// The Cortex-M0 implements the top 2 priority bits, so the four rungs of the ladder are its four
// levels: on the GD32 values the USART (0x40) and DMA (0x70) rungs would both read back 0x40 and
// the motor ISR (0x10) 0x00.
/// Motor period ISR: the most urgent of the Cortex-M0's four levels.
#[cfg(hal_core = "m0")]
pub const MOTOR_ISR_PRIORITY: u8 = 0x00;
/// The USART receive vectors (line errors).
#[cfg(hal_core = "m0")]
pub const USART_RX_PRIORITY: u8 = 0x40;
/// The DMA receive and transmit channel vectors.
#[cfg(hal_core = "m0")]
pub const DMA_RX_PRIORITY: u8 = 0x80;
/// SysTick: the least urgent.
#[cfg(hal_core = "m0")]
pub const SYSTICK_PRIORITY: u8 = 0xC0;

const NVIC_IPR: u32 = 0xE000_E400;
const SCB_SHPR3: u32 = 0xE000_ED20;

/// Set an external interrupt's priority (its byte in `NVIC_IPR`). The priority grouping is left at
/// reset, which on these 4-bit parts makes every bit a preemption bit.
pub fn set_irq_priority(irq: Interrupt, priority: u8) {
    let n = irq as u32;
    let shift = (n % 4) * 8;
    Reg32::new(NVIC_IPR, (n / 4) * 4).modify(0xFF << shift, u32::from(priority) << shift);
}

/// Set SysTick's priority (the top byte of `SCB_SHPR3`).
pub fn set_systick_priority(priority: u8) {
    Reg32::new(SCB_SHPR3, 0).modify(0xFF00_0000, u32::from(priority) << 24);
}

/// The vector of the motor period ISR (the ADC vector, which carries the injected end of
/// conversion).
#[inline]
pub const fn period_irq(_layout: IrqLayout) -> Interrupt {
    DESCRIPTOR.irqs.period
}

/// Apply the motor-era priority ladder: the period ISR above the USART receive vectors, above the
/// DMA channel vectors, above SysTick.
pub fn apply_motor_era_priorities(_layout: IrqLayout) {
    set_irq_priority(DESCRIPTOR.irqs.period, MOTOR_ISR_PRIORITY);
    for v in DESCRIPTOR.irqs.usart.iter().flatten() {
        set_irq_priority(v.rx, USART_RX_PRIORITY);
        set_irq_priority(v.dma_rx, DMA_RX_PRIORITY);
        set_irq_priority(v.dma_tx, DMA_RX_PRIORITY);
    }
    set_systick_priority(SYSTICK_PRIORITY);
}

/// Unmask an interrupt in the NVIC.
#[inline]
pub fn unmask_irq(irq: Interrupt) {
    // SAFETY: the caller has set the vector's priority and the state its handler reads; no
    // critical section in this crate relies on the vector staying masked.
    unsafe { cortex_m::peripheral::NVIC::unmask(irq) }
}

#[cfg(hal_core = "m3")]
const DWT_CYCCNT: u32 = 0xE000_1004;

/// Start the DWT cycle counter (`DEMCR.TRCENA`, `DWT_CTRL.CYCCNTENA`), for pass-scale timing
/// windows read by an attached bench, and the transmit deadline of [`crate::usart_tx::DmaTx`].
/// It zeroes the counter. Not on the Cortex-M0 build, which has no DWT cycle counter.
#[cfg(hal_core = "m3")]
pub fn enable_cycle_counter() {
    const DEMCR: u32 = 0xE000_EDFC;
    const DWT_CTRL: u32 = 0xE000_1000;
    Reg32::new(DEMCR, 0).set_bits(1 << 24);
    Reg32::new(DWT_CYCCNT, 0).write(0);
    Reg32::new(DWT_CTRL, 0).set_bits(1);
}

/// The DWT cycle counter, free-running at the core clock once [`enable_cycle_counter`] has
/// started it.
#[cfg(hal_core = "m3")]
#[inline]
pub(crate) fn cycle_count() -> u32 {
    Reg32::new(DWT_CYCCNT, 0).read()
}

/// The Cortex-M0's stand-in for the DWT cycle counter: the 32-bit TIMER1 (the MM32's TIM2, the
/// descriptor's `Timer1`), counting up at the core clock with no prescaler and the full 32-bit
/// reload, so a `wrapping_sub` of two reads is elapsed core cycles as on the M3. The HAL owns the
/// timer from here: no bring-up routes a TIMER1 pin on this build. Its clock is APB1 doubled by
/// the timer clock rule whenever APB1 is divided, which on the reference tree (APB1 / 2) is the
/// 72 MHz core clock. Started by [`crate::usart_tx::DmaTx::new`], the one user.
#[cfg(hal_core = "m0")]
pub(crate) fn start_cycle_timer() {
    const CR1: u32 = 0x00;
    const EGR: u32 = 0x14;
    const PSC: u32 = 0x28;
    const ARR: u32 = 0x2C;
    let Some(base) = DESCRIPTOR.addrs.get(crate::addr::PeriphLabel::Timer1) else {
        return;
    };
    if let Some(rcu) = DESCRIPTOR.addrs.get(crate::addr::PeriphLabel::Rcu) {
        let _ = crate::clock::enable_general_timer(
            rcu,
            crate::chip::ClockPath,
            crate::addr::PeriphLabel::Timer1,
        );
    }
    let reg = |offset| Reg32::new(base, offset);
    reg(CR1).write(0);
    reg(PSC).write(0);
    reg(ARR).write(u32::MAX);
    // An update event loads the prescaler and the reload and zeroes the counter.
    reg(EGR).write(1);
    reg(CR1).write(1);
}

/// TIMER1's counter, free-running at the core clock once [`start_cycle_timer`] has started it.
#[cfg(hal_core = "m0")]
#[inline]
pub(crate) fn cycle_count() -> u32 {
    const CNT: u32 = 0x24;
    const BASE: u32 = match DESCRIPTOR.addrs.get(crate::addr::PeriphLabel::Timer1) {
        Some(b) => b,
        None => panic!("the Cortex-M0 build's cycle timer is TIMER1"),
    };
    Reg32::new(BASE, CNT).read()
}

/// An ISR entry counter. Each counted vector has one writer (its own handler, at a fixed priority,
/// so it never preempts itself), which is why a load and a store are the whole update.
pub struct IsrMetric {
    entries: AtomicU32,
}

impl IsrMetric {
    const fn new() -> Self {
        Self {
            entries: AtomicU32::new(0),
        }
    }

    /// Count one entry. Call only from the vector this metric counts.
    #[inline]
    pub fn record(&self) {
        let n = self.entries.load(Ordering::Relaxed);
        self.entries.store(n.wrapping_add(1), Ordering::Relaxed);
    }

    /// Entries so far.
    #[inline]
    pub fn entries(&self) -> u32 {
        self.entries.load(Ordering::Relaxed)
    }
}

/// SysTick entries (the image's SysTick handler records them).
pub static SYSTICK_ISR_METRIC: IsrMetric = IsrMetric::new();
/// USART1 receive-vector entries.
pub static USART1_RX_ISR_METRIC: IsrMetric = IsrMetric::new();
/// DMA receive-ring vector entries.
pub static DMA_RX_ISR_METRIC: IsrMetric = IsrMetric::new();

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_period_vector_is_the_adc_vector_of_the_vendor_table() {
        let n = DESCRIPTOR.irqs.period as u16;
        // ADC0_1 = 18 on the F10x, ADC_CMP = 12 on the F1x0 (IRQn_Type).
        let want = if cfg!(any(hal_vectors = "f1x0", hal_vectors = "mm32")) {
            12
        } else {
            18
        };
        assert_eq!(n, want);
    }

    #[test]
    fn the_usart_vectors_are_the_vendor_numbers() {
        let u1 = DESCRIPTOR.irqs.usart[1].expect("USART1 on every chip");
        if cfg!(any(hal_vectors = "f1x0", hal_vectors = "mm32")) {
            // USART1 on the F1x0, UART2 on the MM32: the same numbers.
            assert_eq!(
                (u1.rx as u16, u1.dma_rx as u16, u1.dma_tx as u16),
                (28, 11, 11)
            );
        } else {
            assert_eq!(
                (u1.rx as u16, u1.dma_rx as u16, u1.dma_tx as u16),
                (38, 16, 17)
            );
        }
    }

    #[test]
    fn the_priority_ladder_is_four_distinct_rungs_the_core_implements() {
        let bits: u8 = if cfg!(hal_core = "m0") { 0xC0 } else { 0xF0 };
        let ladder = [
            MOTOR_ISR_PRIORITY,
            USART_RX_PRIORITY,
            DMA_RX_PRIORITY,
            SYSTICK_PRIORITY,
        ];
        for w in ladder.windows(2) {
            assert!(w[0] & bits < w[1] & bits, "{ladder:?}");
        }
        for p in ladder {
            assert_eq!(p & !bits, 0, "{p:#04x} has bits the core drops");
        }
    }

    #[test]
    fn the_metric_counts_entries() {
        let m = IsrMetric::new();
        m.record();
        m.record();
        assert_eq!(m.entries(), 2);
    }
}
