// SPDX-License-Identifier: GPL-3.0-or-later
//! The DMA-ring USART receiver, [`RingBufferedRx`], and the USART vectors.
//!
//! A circular DMA channel fills an application-owned `'static` buffer while the CPU does no
//! per-byte work. The reader drains the bytes behind the live write position (`len - CHCNT`) and
//! detects a lapped cursor from the lap count the channel's vector keeps. The USART's own vector
//! takes the idle-line (not on the MM32, which has none) and line-error interrupts: it clears their flags (the vector is
//! level-sensitive) and records a line error for the reader.
//!
//! Both loss conditions are recovered in place with the channel left running: the reader drops the
//! disturbed or overwritten bytes by moving its cursor to the live write position and reports which
//! it was ([`UsartError::LineError`] for a wire disturbance, [`UsartError::RingOverrun`] for a lap),
//! so the framer above resynchronises on the next frame and the link heals with no re-arm.

use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU32, Ordering};

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::dma::{self, Channel};
use crate::error::{DescriptorError, UsartError};
use crate::irq;
use crate::usart::UsartRx;

/// Per USART (at the label's index), a line error the USART vector saw and the reader has not yet
/// taken. Set by the vector (and by [`RingBufferedRx::inject_line_error`]), taken by the reader.
static LINE_ERROR: [AtomicBool; 3] = [const { AtomicBool::new(false) }; 3];

/// Whether this chip can receive on `usart` with a DMA ring: the chip carries the USART, its
/// vectors and a DMA channel for its receive. Pure: no register access. It is exactly the check
/// [`RingBufferedRx::new`] makes before touching anything, so the two cannot disagree.
pub fn supports_rx(_chip: &Chip, usart: PeriphLabel) -> bool {
    dma::usart_channels(usart).is_some()
}

/// Read and clear `flag`. The load comes first on its own, so the common case (nothing set) costs
/// no critical section; the take itself is a load and a store inside one.
#[inline]
fn take(flag: &AtomicBool) -> bool {
    if !flag.load(Ordering::Acquire) {
        return false;
    }
    cortex_m::interrupt::free(|_| {
        let set = flag.load(Ordering::Acquire);
        flag.store(false, Ordering::Release);
        set
    })
}

// --- the cursor arithmetic ------------------------------------------------------------------------

/// The reader's position in a ring of `len` bytes: `total` counts the bytes consumed (modulo 2^32)
/// and `pos` is where the next one is in the buffer.
///
/// The live write position is `wraps * len + (len - remaining)`, also modulo 2^32, so the bytes
/// waiting are a wrapping difference that stays exact while the reader is less than 2^32 bytes
/// behind (about a day at 460800 baud) and a lap is any difference above `len`. `pos` is carried
/// beside `total` rather than derived as `total % len`, which is wrong once `total` wraps unless
/// `len` divides 2^32, and costs a division on every read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Cursor {
    len: u32,
    total: u32,
    pos: u32,
}

impl Cursor {
    const fn new(len: u16) -> Self {
        Self {
            len: len as u32,
            total: 0,
            pos: 0,
        }
    }

    /// The live write position for a channel snapshot (`wraps` laps counted, `remaining` in
    /// `CHCNT`).
    const fn write_total(&self, wraps: u32, remaining: u16) -> u32 {
        wraps
            .wrapping_mul(self.len)
            .wrapping_add(self.len.saturating_sub(remaining as u32))
    }

    /// Bytes waiting behind `write_total`, or `None` if the writer has lapped the reader (more than
    /// a whole ring ahead, so unread bytes were overwritten).
    const fn waiting(&self, write_total: u32) -> Option<u32> {
        let n = write_total.wrapping_sub(self.total);
        if n > self.len {
            None
        } else {
            Some(n)
        }
    }

    /// Move to the live write position, dropping every byte behind it.
    fn resync(&mut self, write_total: u32, remaining: u16) {
        self.total = write_total;
        let pos = self.len.saturating_sub(u32::from(remaining));
        self.pos = if pos >= self.len { 0 } else { pos };
    }

    /// Consume `n` bytes (at most `len`).
    fn advance(&mut self, n: u32) {
        self.total = self.total.wrapping_add(n);
        self.pos += n;
        if self.pos >= self.len {
            self.pos -= self.len;
        }
    }
}

// --- the receiver ---------------------------------------------------------------------------------

/// A USART receiver on a circular DMA channel into an application-owned `'static` buffer.
///
/// Built from the receive half of a split [`crate::usart::Usart`], which it holds for as long as it
/// lives. The buffer is not a type parameter, so every port's receiver is one type (and so is the
/// [`crate::serial::SplitSerial`] above it).
pub struct RingBufferedRx {
    channel: Channel,
    buf: *mut u8,
    cursor: Cursor,
    wraps: &'static AtomicU32,
    line_error: &'static AtomicBool,
    _rx: UsartRx,
}

impl RingBufferedRx {
    /// Arm circular DMA reception on `rx`, the receive half of `instance`, into `dma_buf`.
    ///
    /// In order: resolve the instance's channel and vectors; check the buffer (1 to 65535 bytes,
    /// the reach of `CHCNT`); enable the DMA clock; stop the channel in case it runs; the
    /// write-back self-check; reset the lap count and the line-error flag; program and start the
    /// channel (circular, half- and full-transfer interrupts); on the USART `DENR`, `ERRIE` and
    /// `IDLEIE`; then unmask the DMA receive vector and the USART vector. The caller sets their
    /// priorities first ([`irq::apply_motor_era_priorities`]).
    ///
    /// Refusals: [`DescriptorError::Unsupported`], before any register is written, for an instance
    /// [`supports_rx`] refuses, a half of another instance, or a buffer out of range;
    /// [`DescriptorError::SelfCheckFailed`] if the channel does not respond, with the DMA clock on
    /// and the channel stopped but nothing armed.
    pub fn new(
        chip: &Chip,
        rx: UsartRx,
        instance: PeriphLabel,
        dma_buf: &'static mut [u8],
    ) -> Result<RingBufferedRx, DescriptorError> {
        if rx.label() != instance {
            return Err(DescriptorError::Unsupported);
        }
        let usart_dma = dma::resolve(chip, instance)?;
        let (Some(wraps), Some(line_error)) =
            (dma::rx_wraps(instance), LINE_ERROR.get(instance.index()))
        else {
            return Err(DescriptorError::Unsupported);
        };
        let len = match u16::try_from(dma_buf.len()) {
            Ok(n) if n > 0 => n,
            _ => return Err(DescriptorError::Unsupported),
        };
        let rcu = chip.rcu_base()?;
        let channel = usart_dma.rx;

        crate::clock::enable_dma(rcu);
        channel.disable();
        if !channel.self_check() {
            return Err(DescriptorError::SelfCheckFailed);
        }
        // Both vectors are still masked: nothing else writes these yet.
        wraps.store(0, Ordering::Release);
        line_error.store(false, Ordering::Release);
        let buf = dma_buf.as_mut_ptr();
        // SAFETY: `buf` is the caller's `'static` buffer of `len` bytes, owned by this receiver
        // for as long as the channel runs; the CPU only reads bytes behind the write position.
        unsafe { channel.start_circular_rx(rx.rdata_addr(), buf as u32, len) };
        rx.enable_dma_ring();
        irq::unmask_irq(usart_dma.irqs.dma_rx);
        irq::unmask_irq(usart_dma.irqs.rx);

        Ok(RingBufferedRx {
            channel,
            buf,
            cursor: Cursor::new(len),
            wraps,
            line_error,
            _rx: rx,
        })
    }

    /// A consistent (laps, remaining) for the live write position.
    ///
    /// Two hazards at a lap: a lap the vector counts during the snapshot (the count is read on both
    /// sides and the snapshot retried on a change), and a lap it has not counted yet (the hardware
    /// has reloaded `CHCNT` and set `FTFIF`, the vector has not run). The second is read from the
    /// pending flag and counted here, or the reloaded `CHCNT` would put the write position a whole
    /// ring behind the reader and report a lap that never happened. A `CHCNT` that went up between
    /// its two reads with the flag read clear is a reload that slipped past the flag read: retry.
    fn snapshot(&self) -> (u32, u16) {
        loop {
            let w1 = self.wraps.load(Ordering::Acquire);
            let a = self.channel.remaining();
            let pending = self.channel.ftf_pending();
            let b = self.channel.remaining();
            let w2 = self.wraps.load(Ordering::Acquire);
            if w1 != w2 {
                continue;
            }
            if pending {
                return (w1.wrapping_add(1), b);
            }
            if b <= a {
                return (w1, b);
            }
        }
    }

    /// The live write position.
    #[inline]
    fn write_total(&self) -> (u32, u16) {
        let (w, rem) = self.snapshot();
        (self.cursor.write_total(w, rem), rem)
    }

    /// Drain the bytes waiting into `out`; `Ok(0)` when there are none. A recorded line error is
    /// [`UsartError::LineError`] and a lapped cursor [`UsartError::RingOverrun`]; either way the
    /// cursor has moved to the live write position and the next read continues from there.
    pub(crate) fn read(&mut self, out: &mut [u8]) -> Result<usize, UsartError> {
        if take(self.line_error) {
            let (live, rem) = self.write_total();
            self.cursor.resync(live, rem);
            return Err(UsartError::LineError);
        }
        let (live, rem) = self.write_total();
        let Some(waiting) = self.cursor.waiting(live) else {
            self.cursor.resync(live, rem);
            return Err(UsartError::RingOverrun);
        };
        let n = (waiting as usize).min(out.len());
        // The buffer reads must not be hoisted above the `CHCNT` snapshot.
        compiler_fence(Ordering::Acquire);
        let len = self.cursor.len as usize;
        let mut pos = self.cursor.pos as usize;
        for slot in out.iter_mut().take(n) {
            // SAFETY: `pos < len`, inside the `'static` buffer, and behind the write position, so
            // the channel has written this byte and is not writing it now.
            *slot = unsafe { core::ptr::read_volatile(self.buf.add(pos)) };
            pos += 1;
            if pos == len {
                pos = 0;
            }
        }
        self.cursor.advance(n as u32);
        Ok(n)
    }

    /// Whether a [`read`](Self::read) would make progress: bytes waiting, a lap, or a line error to
    /// report. Consumes nothing.
    pub(crate) fn ready(&self) -> bool {
        if self.line_error.load(Ordering::Acquire) {
            return true;
        }
        let (live, _) = self.write_total();
        self.cursor.waiting(live) != Some(0)
    }

    /// Record one line error exactly as the USART vector does, so the next read reports it and
    /// resynchronises: the firmware's controlled injection of the self-heal path. It fabricates no
    /// channel state.
    #[inline]
    pub(crate) fn inject_line_error(&self) {
        self.line_error.store(true, Ordering::Release);
    }
}

// --- the USART vectors ----------------------------------------------------------------------------

/// The USART whose own vector is `v`, by label index. Evaluated at compile time for each handler;
/// a handler for a vector no carried USART names fails the build.
#[cfg(any(test, target_arch = "arm"))]
const fn usart_of_vector(v: irq::Interrupt) -> usize {
    let mut i = 0;
    while i < 3 {
        if let Some(irqs) = dma::usart_irqs(crate::usart::USARTS[i]) {
            if irqs.rx as u16 == v as u16 {
                return i;
            }
        }
        i += 1;
    }
    panic!("not a USART vector of the compiled chip")
}

/// USART `I`'s vector body: clear its flags and record a line error for the reader.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn on_usart_vector<const I: usize>() {
    const fn base(i: usize) -> u32 {
        match crate::descriptor::DESCRIPTOR
            .addrs
            .get(crate::usart::USARTS[i])
        {
            Some(b) => b,
            None => panic!("a USART vector's instance is carried"),
        }
    }
    let base: u32 = const { base(I) };
    if crate::usart::service_rx_flags(base) {
        LINE_ERROR[I].store(true, Ordering::Release);
    }
    if I == 1 {
        irq::USART1_RX_ISR_METRIC.record();
    }
}

/// The USART vectors, by vendor name: one handler each on the board target, and the list itself
/// for the tests.
macro_rules! usart_vectors {
    ($($name:ident),+ $(,)?) => {
        #[cfg(test)]
        const VECTORS: &[irq::Interrupt] = &[$(irq::Interrupt::$name),+];

        #[cfg(target_arch = "arm")]
        mod handlers {
            $(
                /// A USART vector of the ring receiver (`crate::usart_rx`).
                ///
                /// # Safety
                ///
                /// Entered by the hardware through the flash vector table only.
                #[no_mangle]
                #[allow(non_snake_case)]
                pub unsafe extern "C" fn $name() {
                    super::on_usart_vector::<
                        { super::usart_of_vector(crate::irq::Interrupt::$name) },
                    >();
                }
            )+
        }
    };
}

#[cfg(hal_usart = "f10x")]
usart_vectors!(USART0, USART1, USART2);
#[cfg(hal_usart = "f1x0")]
usart_vectors!(USART0, USART1);
#[cfg(hal_usart = "mm32")]
usart_vectors!(UART1, UART2);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usart::USARTS;

    fn chip() -> Chip {
        // SAFETY: the pure queries touch no register.
        unsafe { Chip::steal() }
    }

    #[test]
    fn supports_rx_is_every_carried_usart() {
        let carried = |l| crate::descriptor::DESCRIPTOR.addrs.get(l).is_some();
        for usart in USARTS {
            assert_eq!(supports_rx(&chip(), usart), carried(usart), "{usart:?}");
        }
        assert!(supports_rx(&chip(), PeriphLabel::Usart0));
        assert!(supports_rx(&chip(), PeriphLabel::Usart1));
        // USART2 on the F10x only.
        assert_eq!(
            supports_rx(&chip(), PeriphLabel::Usart2),
            cfg!(hal_usart = "f10x")
        );
        for other in [PeriphLabel::I2c0, PeriphLabel::Gpioa, PeriphLabel::Dma0] {
            assert!(!supports_rx(&chip(), other), "{other:?}");
        }
    }

    #[test]
    fn the_handlers_cover_every_carried_usart_vector() {
        for usart in USARTS {
            if let Some(irqs) = dma::usart_irqs(usart) {
                assert!(VECTORS.contains(&irqs.rx), "{usart:?}");
                assert_eq!(usart_of_vector(irqs.rx), usart.index());
            }
        }
        assert_eq!(
            VECTORS.len(),
            USARTS
                .iter()
                .filter(|&&u| dma::usart_irqs(u).is_some())
                .count()
        );
    }

    #[test]
    fn the_cursor_reads_what_the_channel_wrote() {
        let mut c = Cursor::new(128);
        // Fresh: CHCNT = 128, nothing written.
        assert_eq!(c.waiting(c.write_total(0, 128)), Some(0));
        // 20 bytes in.
        let live = c.write_total(0, 108);
        assert_eq!(c.waiting(live), Some(20));
        c.advance(20);
        assert_eq!((c.total, c.pos), (20, 20));
        // Across the lap: 120 bytes later the channel is 12 into its second lap.
        let live = c.write_total(1, 116);
        assert_eq!(c.waiting(live), Some(120));
        c.advance(120);
        assert_eq!((c.total, c.pos), (140, 12));
    }

    #[test]
    fn exactly_one_ring_behind_is_not_a_lap_and_one_more_byte_is() {
        let c = Cursor::new(128);
        assert_eq!(c.waiting(c.write_total(1, 128)), Some(128));
        assert_eq!(c.waiting(c.write_total(1, 127)), None);
    }

    #[test]
    fn a_lap_resyncs_to_the_live_position() {
        let mut c = Cursor::new(128);
        let live = c.write_total(3, 100);
        assert_eq!(c.waiting(live), None);
        c.resync(live, 100);
        assert_eq!((c.total, c.pos), (3 * 128 + 28, 28));
        assert_eq!(c.waiting(c.write_total(3, 100)), Some(0));
        // A resync at a fresh reload (CHCNT = len) lands at the start of the buffer.
        c.resync(c.write_total(4, 128), 128);
        assert_eq!(c.pos, 0);
    }

    #[test]
    fn the_position_survives_the_byte_count_wrapping() {
        // A ring that does not divide 2^32: `total % len` would jump at the wrap, `pos` does not.
        let len = 100u16;
        let mut c = Cursor::new(len);
        // 42_949_672 laps = 4_294_967_200 bytes, 96 short of the wrap.
        let laps = u32::MAX / 100;
        c.resync(c.write_total(laps, len), len);
        assert_eq!(c.pos, 0);
        // 150 bytes later the count has wrapped; a reader that waited that long was lapped.
        assert_eq!(c.waiting(c.write_total(laps + 1, 50)), None);
        // Read in two steps, as a reader keeping up would.
        let live = c.write_total(laps, 30);
        assert_eq!(c.waiting(live), Some(70));
        c.advance(70);
        assert_eq!(c.pos, 70);
        let live = c.write_total(laps + 1, 50);
        assert_eq!(c.waiting(live), Some(80));
        c.advance(80);
        assert_eq!(c.pos, 50);
        assert!(c.total < 100, "the count wrapped: {}", c.total);
        assert_eq!(c.waiting(c.write_total(laps + 1, 50)), Some(0));
    }
}
