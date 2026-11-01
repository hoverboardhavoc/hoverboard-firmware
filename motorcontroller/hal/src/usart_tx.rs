// SPDX-License-Identifier: GPL-3.0-or-later
//! The one-shot DMA USART transmitter, [`DmaTx`].
//!
//! The CPU copies a frame into an owned buffer, programs a channel and leaves; the channel feeds
//! the transmit data register and raises one interrupt at the end, whose body (in the `dma`
//! module) stops the channel and clears the in-flight flag. A send while a transfer is in flight is
//! refused: it queues nothing and drops nothing, and the caller's policy decides.
//!
//! A transfer whose completion never comes (a wedged channel, a lost vector) would hold the port
//! for good, so the refusal carries a deadline: the frame's wire time at the configured baud times
//! [`BUDGET_SLACK`], measured on the DWT cycle counter. Past it, the next send abandons the
//! transfer, counts it in [`DmaTx`]'s timeouts, and takes the new frame.
//!
//! Completion means the last byte reached the data register, not the wire; one character time of
//! shifting remains, which [`crate::serial::SplitSerial`]'s `flush` waits out on `TC`.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::chip::Chip;
use crate::dma::{self, Channel};
use crate::error::DescriptorError;
use crate::irq;
use crate::usart::UsartTx;

/// Bit times of one 8N1 character: start, eight data, stop (the only frame [`crate::usart::Usart::new`]
/// configures).
const BITS_PER_BYTE: u64 = 10;

/// The deadline's slack over the bare wire time. It absorbs a real but slow transfer (the
/// completion vector delayed behind higher-priority interrupts, the last character still shifting)
/// and stays small enough that a wedged port recovers within a few main-loop passes: at 115200 a
/// 27 B frame's deadline is 9.4 ms.
const BUDGET_SLACK: u64 = 4;

/// The ceiling on a deadline: half the cycle counter's range, so the wrapping comparison against
/// the free-running counter stays meaningful.
const MAX_BUDGET_CYCLES: u32 = u32::MAX / 2;

/// Clamp a cycle count to `1..=MAX_BUDGET_CYCLES` (a zero deadline would expire at once).
const fn clamp_cycles(c: u64) -> u32 {
    if c > MAX_BUDGET_CYCLES as u64 {
        MAX_BUDGET_CYCLES
    } else if c == 0 {
        1
    } else {
        c as u32
    }
}

/// Deadline cycles per byte at `baud` on a `core_hz` core, slack included. A zero baud gives the
/// ceiling rather than a division by zero.
const fn cycles_per_byte(core_hz: u32, baud: u32) -> u32 {
    if baud == 0 {
        return MAX_BUDGET_CYCLES;
    }
    clamp_cycles(core_hz as u64 * BITS_PER_BYTE / baud as u64 * BUDGET_SLACK)
}

/// The deadline in cycles for a transfer of `len` bytes.
const fn budget_cycles(per_byte: u32, len: usize) -> u32 {
    clamp_cycles((per_byte as u64).saturating_mul(len as u64))
}

/// A one-shot DMA transmitter for one USART.
///
/// Built from the transmit half of a split [`crate::usart::Usart`], which it holds. The `'static`
/// buffer is owned rather than borrowed per send because the channel reads it for as long as the
/// transfer runs; the owned copy and the in-flight refusal together keep the CPU out of the bytes
/// the channel is reading. Its length is the caller's largest frame and is a runtime field, so
/// every port's transmitter is one type.
pub struct DmaTx {
    channel: Channel,
    in_flight: &'static AtomicBool,
    buf: &'static mut [u8],
    tx: UsartTx,
    paddr: u32,
    per_byte: u32,
    /// The cycle count when the transfer in flight was started, and its deadline in cycles.
    started: u32,
    budget: u32,
    /// Transfers abandoned at their deadline, saturating.
    timeouts: u16,
}

impl DmaTx {
    /// Arm DMA transmission on `tx`, sending out of `buf`.
    ///
    /// In order: check the buffer (1 to 65535 bytes); resolve the instance's channel and vectors;
    /// enable the DMA clock; stop the channel in case it runs; the write-back self-check; `DENT` on
    /// the USART; clear the in-flight flag; start the cycle counter the deadline is measured on
    /// ([`irq::enable_cycle_counter`], which zeroes it); unmask the transmit vector. No transfer is
    /// programmed until the first send. `core_hz` and `baud` size the deadline.
    ///
    /// Refusals: [`DescriptorError::Unsupported`], before any register is written, for a buffer
    /// out of range or a USART with no transmit channel; [`DescriptorError::SelfCheckFailed`] if
    /// the channel does not respond, with the DMA clock on and the channel stopped but nothing
    /// armed.
    pub fn new(
        chip: &Chip,
        tx: UsartTx,
        buf: &'static mut [u8],
        core_hz: u32,
        baud: u32,
    ) -> Result<DmaTx, DescriptorError> {
        if buf.is_empty() || buf.len() > usize::from(u16::MAX) {
            return Err(DescriptorError::Unsupported);
        }
        let usart_dma = dma::resolve(chip, tx.label())?;
        let in_flight = dma::tx_in_flight(tx.label()).ok_or(DescriptorError::Unsupported)?;
        let rcu = chip.rcu_base()?;
        let channel = usart_dma.tx;

        crate::clock::enable_dma(rcu);
        channel.disable();
        if !channel.self_check() {
            return Err(DescriptorError::SelfCheckFailed);
        }
        tx.enable_dma();
        in_flight.store(false, Ordering::Release);
        irq::enable_cycle_counter();
        irq::unmask_irq(usart_dma.irqs.dma_tx);

        Ok(DmaTx {
            channel,
            in_flight,
            buf,
            paddr: tx.tdata_addr(),
            tx,
            per_byte: cycles_per_byte(core_hz, baud),
            started: 0,
            budget: 0,
            timeouts: 0,
        })
    }

    /// Hand a frame to the channel: copy what fits the owned buffer, program the channel and start
    /// it, and answer how many bytes were taken (a frame longer than the buffer is truncated; an
    /// empty one starts nothing). `None` while a transfer is in flight and inside its deadline;
    /// past the deadline the transfer is abandoned (channel stopped, its flags cleared, counted)
    /// and the new frame taken.
    pub(crate) fn try_send(&mut self, frame: &[u8]) -> Option<usize> {
        if frame.is_empty() {
            return Some(0);
        }
        if self.in_flight.load(Ordering::Acquire) {
            if !self.expired() {
                return None;
            }
            // Stop the channel before clearing its flags, so a completion racing the abandon
            // leaves nothing set behind the clear; the in-flight flag last, as the vector does.
            self.channel.disable();
            self.channel.clear_flags();
            self.in_flight.store(false, Ordering::Release);
            self.timeouts = self.timeouts.saturating_add(1);
        }
        let n = frame.len().min(self.buf.len());
        self.buf[..n].copy_from_slice(&frame[..n]);
        self.budget = budget_cycles(self.per_byte, n);
        self.started = irq::cycle_count();
        // Set before `CHEN`: a completion landing between the start and this store would clear a
        // flag not yet set and leave the port held until the deadline.
        self.in_flight.store(true, Ordering::Release);
        // SAFETY: `self.buf` is the caller's `'static` buffer, owned by this transmitter; the
        // in-flight refusal keeps the CPU out of its first `n` bytes until the channel is done.
        // `n` fits `u16`: `new` refused a longer buffer.
        unsafe {
            self.channel
                .start_oneshot_tx(self.paddr, self.buf.as_ptr() as u32, n as u16)
        };
        Some(n)
    }

    /// A transfer is programmed and has not reported its completion.
    #[inline]
    pub(crate) fn in_flight(&self) -> bool {
        self.in_flight.load(Ordering::Acquire)
    }

    /// The transfer in flight has outlived its deadline (a wrapping difference against the
    /// free-running counter; the deadline is at most half its range).
    #[inline]
    fn expired(&self) -> bool {
        irq::cycle_count().wrapping_sub(self.started) >= self.budget
    }

    /// Transfers abandoned at their deadline, saturating. Non-zero on a working port means a real
    /// transmit fault, not a slow consumer.
    #[inline]
    pub(crate) fn timeouts(&self) -> u16 {
        self.timeouts
    }

    /// `TC`: the last byte has left the shift register.
    #[inline]
    pub(crate) fn tx_complete(&self) -> bool {
        self.tx.tx_complete()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_deadline_is_four_wire_times() {
        // 72 MHz, 115200 baud: 6250 cycles a byte on the wire, 25000 with the slack.
        assert_eq!(cycles_per_byte(72_000_000, 115_200), 25_000);
        // 27 B: 675_000 cycles = 9.375 ms at 72 MHz.
        assert_eq!(budget_cycles(25_000, 27), 675_000);
        // 460800 baud: 1562 cycles a byte (rounded down), 6248 with the slack.
        assert_eq!(cycles_per_byte(72_000_000, 460_800), 6_248);
    }

    #[test]
    fn the_deadline_is_never_zero_and_never_past_half_the_counter() {
        assert_eq!(cycles_per_byte(72_000_000, 0), MAX_BUDGET_CYCLES);
        assert_eq!(cycles_per_byte(1, 1_000_000), 1);
        assert_eq!(budget_cycles(25_000, 0), 1);
        assert_eq!(budget_cycles(MAX_BUDGET_CYCLES, 3), MAX_BUDGET_CYCLES);
        assert_eq!(budget_cycles(u32::MAX, usize::MAX), MAX_BUDGET_CYCLES);
    }
}
