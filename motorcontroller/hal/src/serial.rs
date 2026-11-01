// SPDX-License-Identifier: GPL-3.0-or-later
//! The `embedded-io` 0.6 endpoint over a split USART: [`SplitSerial`], a DMA-ring receiver and a
//! DMA transmitter.
//!
//! The contract the link's `SerialTransport` relies on:
//!
//! - **`Read` never blocks; `Ok(0)` means nothing is waiting.** A UART has no end of file; the
//!   transport gates its reads on `ReadReady`.
//! - **`Error = Infallible`.** Line errors and laps are absorbed below the trait: the receiver
//!   resynchronises in place and this adapter counts each, apart, so a bench can tell a wire
//!   disturbance ([`SplitSerial::line_errors`]) from a consumer that fell a ring behind
//!   ([`SplitSerial::lap_overruns`]).
//! - **`ReadReady` means a read will make progress**: bytes waiting, or a condition the read would
//!   absorb.
//! - **Every wait on the write path is bounded**, by a budget derived from the core clock
//!   ([`worst_case_write_block_us`]). The main loop writing through this has a 16 ms motor coast
//!   and a 500 ms watchdog, and an unbounded wait on a transmit flag answered with eleven watchdog
//!   resets in one bench session (2026-10-22). A write whose budget runs out drops the frame, counts
//!   it in [`SplitSerial::tx_timeouts`] and reports the buffer consumed: `Ok(0)` on a non-empty
//!   buffer panics `embedded-io`'s `write_all`, and L2 is best-effort (a truncated frame fails its
//!   receiver's CRC).

use core::convert::Infallible;

use embedded_io::{ErrorType, Read, ReadReady, Write};

use crate::error::UsartError;
use crate::usart_rx::RingBufferedRx;
use crate::usart_tx::DmaTx;

/// Core cycles one write-path poll iteration costs, as an upper bound (an over-statement
/// under-states the iteration count, so the real block ends early): a status load over the APB
/// bridge, a test, the countdown and a taken branch, with the flash wait states on the fetch.
const WRITE_POLL_CYCLES: u32 = 24;

/// The wall-clock ceiling on the polling one write call may do. Above the longest legitimate wait
/// (a 27 B frame is 2.3 ms of wire at 115200) and a quarter of the 16 ms motor coast.
const WRITE_BUDGET_US: u32 = 4_000;

/// Poll iterations that fit [`WRITE_BUDGET_US`] at `core_hz`, at least one.
const fn write_budget(core_hz: u32) -> u32 {
    let n = (core_hz / 1_000_000) * WRITE_BUDGET_US / WRITE_POLL_CYCLES;
    if n == 0 {
        1
    } else {
        n
    }
}

/// The longest one write call can block its caller, in microseconds, at `core_hz`. Const, so a
/// caller with a deadline asserts its own budget against it at compile time.
pub const fn worst_case_write_block_us(core_hz: u32) -> u32 {
    let mhz = core_hz / 1_000_000;
    if mhz == 0 {
        return WRITE_BUDGET_US;
    }
    write_budget(core_hz) * WRITE_POLL_CYCLES / mhz
}

/// Spin until `cond` holds or `left` runs out; `false` if it ran out. The one place a write-path
/// wait is written, so none can be added without a bound.
#[inline]
fn wait_bounded(left: &mut u32, mut cond: impl FnMut() -> bool) -> bool {
    while !cond() {
        if *left == 0 {
            return false;
        }
        *left -= 1;
    }
    true
}

/// A non-blocking `embedded-io` endpoint over a split USART: the receiver `R` (the DMA ring,
/// [`RingBufferedRx`]) and a [`DmaTx`]. Built from already-armed halves; it brings nothing up.
pub struct SplitSerial<R> {
    tx: DmaTx,
    rx: R,
    line_errors: u16,
    lap_overruns: u16,
    /// Poll iterations one write call may spend ([`write_budget`] at the core clock).
    write_budget: u32,
    tx_timeouts: u16,
}

impl SplitSerial<RingBufferedRx> {
    /// Join an armed transmitter and receiver. `core_hz` is the core clock the write budget is
    /// derived against (a wall-clock bound: the same iteration count is 4 ms at 72 MHz and 36 ms at
    /// 8 MHz).
    #[inline]
    pub fn new(tx: DmaTx, rx: RingBufferedRx, core_hz: u32) -> Self {
        Self {
            tx,
            rx,
            line_errors: 0,
            lap_overruns: 0,
            write_budget: write_budget(core_hz),
            tx_timeouts: 0,
        }
    }

    /// Line errors (overrun, framing, noise on the wire) absorbed by `read`, saturating.
    #[inline]
    pub fn line_errors(&self) -> u16 {
        self.line_errors
    }

    /// Laps (the reader fell a whole ring behind and unread bytes were overwritten) absorbed by
    /// `read`, saturating.
    #[inline]
    pub fn lap_overruns(&self) -> u16 {
        self.lap_overruns
    }

    /// Abandoned writes, saturating: this adapter's write budget running out, plus the
    /// transmitter's transfers abandoned at their deadline. Each event is counted once, where it
    /// happens.
    #[inline]
    pub fn tx_timeouts(&self) -> u16 {
        self.tx_timeouts.saturating_add(self.tx.timeouts())
    }

    /// Record one line error in the receiver exactly as the USART vector does: the next `read`
    /// absorbs it, counts it in [`line_errors`](Self::line_errors) and resynchronises, the path a
    /// real wire disturbance takes. The firmware's controlled injection over SWD.
    #[inline]
    pub fn inject_line_error(&self) {
        self.rx.inject_line_error();
    }
}

impl ErrorType for SplitSerial<RingBufferedRx> {
    type Error = Infallible;
}

impl Read for SplitSerial<RingBufferedRx> {
    /// Drain what the receiver has; `Ok(0)` when nothing is waiting. An absorbed condition is
    /// counted and the drain retried, so one call still returns the bytes behind it. Bounded: each
    /// condition is taken as it is reported, and four attempts cover a line error and a lap
    /// together.
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
        for _ in 0..4 {
            match self.rx.read(out) {
                Ok(n) => return Ok(n),
                Err(UsartError::RingOverrun) => {
                    self.lap_overruns = self.lap_overruns.saturating_add(1);
                }
                Err(_) => {
                    self.line_errors = self.line_errors.saturating_add(1);
                }
            }
        }
        Ok(0)
    }
}

impl ReadReady for SplitSerial<RingBufferedRx> {
    #[inline]
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx.ready())
    }
}

impl Write for SplitSerial<RingBufferedRx> {
    /// Hand the whole frame to one DMA transfer and return how many bytes were taken. On a busy
    /// channel it retries within the write budget rather than answer `Ok(0)`: the deadline that
    /// recovers a wedged channel is checked inside the send, so retrying is what reaches it. A
    /// frame whose budget runs out is dropped, counted and reported consumed.
    fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
        if data.is_empty() {
            return Ok(0);
        }
        let mut sent = 0;
        let mut left = self.write_budget;
        let tx = &mut self.tx;
        let taken = wait_bounded(&mut left, || match tx.try_send(data) {
            Some(n) => {
                sent = n;
                true
            }
            None => false,
        });
        if taken {
            Ok(sent)
        } else {
            self.tx_timeouts = self.tx_timeouts.saturating_add(1);
            Ok(data.len())
        }
    }

    /// Wait for the transfer to complete and the last byte to leave the shift register (`TC`),
    /// within the write budget; a timeout is counted. What a caller does before reconfiguring the
    /// port, and not on every frame.
    fn flush(&mut self) -> Result<(), Self::Error> {
        let mut left = self.write_budget;
        let tx = &self.tx;
        if !wait_bounded(&mut left, || !tx.in_flight() && tx.tx_complete()) {
            self.tx_timeouts = self.tx_timeouts.saturating_add(1);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_write_budget_is_4_ms_at_72_mhz() {
        assert_eq!(write_budget(72_000_000), 12_000);
        assert_eq!(worst_case_write_block_us(72_000_000), 4_000);
    }

    #[test]
    fn the_worst_case_never_exceeds_the_ceiling() {
        for mhz in [1, 7, 8, 24, 48, 64, 72, 108] {
            let us = worst_case_write_block_us(mhz * 1_000_000);
            assert!(us <= WRITE_BUDGET_US, "{mhz} MHz: {us}");
            assert!(us + WRITE_POLL_CYCLES >= WRITE_BUDGET_US, "{mhz} MHz: {us}");
        }
        // Under 1 MHz the budget is one iteration and the answer is the ceiling.
        assert_eq!(write_budget(500_000), 1);
        assert_eq!(worst_case_write_block_us(0), WRITE_BUDGET_US);
    }

    #[test]
    fn two_writes_fit_the_motor_coast() {
        // The firmware's compile-time check: two write calls inside the 16 ms coast.
        const COAST_US: u32 = 16_000;
        assert!(worst_case_write_block_us(72_000_000) * 2 < COAST_US);
    }

    #[test]
    fn a_bounded_wait_gives_up_after_its_budget() {
        let mut left = 3;
        let mut polls = 0;
        assert!(!wait_bounded(&mut left, || {
            polls += 1;
            false
        }));
        // The first poll, then one per budget unit.
        assert_eq!((polls, left), (4, 0));
        let mut left = 3;
        let mut n = 0;
        assert!(wait_bounded(&mut left, || {
            n += 1;
            n == 2
        }));
        assert_eq!(left, 2);
    }
}
