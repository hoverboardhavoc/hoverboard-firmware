// SPDX-License-Identifier: GPL-3.0-or-later
//! The DMA channels the USART drivers use: the per-USART channel map, the channel programming
//! (circular peripheral-to-memory for receive, one-shot memory-to-peripheral for transmit), and
//! the DMA vectors with their service bodies.
//!
//! The register block is the same on both models (`gd32f10x_dma.h`, `gd32f1x0_dma.h`): `INTF` at
//! `0x00` and `INTC` at `0x04` with four flags per channel (`GIF`, `FTFIF`, `HTFIF`, `ERRIF` at
//! `4 * ch + 0..3`), then per channel at a `0x14` stride from channel 0: `CHCTL` `0x08`, `CHCNT`
//! `0x0C`, `CHPADDR` `0x10`, `CHMADDR` `0x14`. What differs is the request map, the `hal_dma` model:
//!
//! | USART  | F10x receive / transmit       | F1x0 receive / transmit          |
//! |--------|-------------------------------|----------------------------------|
//! | USART0 | channel 4 / channel 3         | channel 2 / channel 1 (one vector) |
//! | USART1 | channel 5 / channel 6         | channel 4 / channel 3 (one vector) |
//! | USART2 | channel 2 / channel 1         | not carried                      |
//!
//! The MM32SPIN05's DMA1 (`hal_dma = "mm32"`) has the same block (MindMotion's `reg_dma.h`: `ISR`,
//! `IFCR`, then `CCR`/`CNDTR`/`CPAR`/`CMAR` per channel at the same stride, channel 1 at the HAL's
//! channel 0) and the F1x0 request map in the HAL's numbering: UART1 (`Usart0`) receive on channel
//! 2 and transmit on 1 (MindMotion's 3 and 2, its UART_DMA samples), UART2 (`Usart1`) receive on 4
//! and transmit on 3 (its 5 and 4, the map AILIFE's MM32 firmware runs its UART on), with the
//! vectors `DMA1_Channel2_3` and `DMA1_Channel4_5`.
//!
//! (GD32F10x User Manual table 9-3; GD32F1x0 User Manual table 8-3, with the `SYSCFG_CFG0` USART0
//! DMA remap bits at reset, which this HAL never changes: remapped, USART0 would collide with
//! USART1 on channels 3 and 4.) The vector each channel raises is the descriptor's
//! (`irqs.usart[..].dma_rx` / `dma_tx`); the tests check the two agree.
//!
//! On the F1x0 one vector carries both channels of a USART. Its handler runs the receive and the
//! transmit service on every entry, each acting only on its own channel's flags, so an entry with
//! both flagged services both: a handler that stopped at the first match would drop a transmit
//! completion, and the in-flight flag it leaves set holds the port until the force-restart
//! deadline.

use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU32, Ordering};

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::{UsartIrqs, DESCRIPTOR};
use crate::error::DescriptorError;
use crate::reg::Reg32;

const INTF: u32 = 0x00;
const INTC: u32 = 0x04;
const CH_STRIDE: u32 = 0x14;
const CHCTL: u32 = 0x08;
const CHCNT: u32 = 0x0C;
const CHPADDR: u32 = 0x10;
const CHMADDR: u32 = 0x14;

const CHCTL_CHEN: u32 = 1 << 0;
const CHCTL_FTFIE: u32 = 1 << 1;
const CHCTL_HTFIE: u32 = 1 << 2;
/// Memory to peripheral (clear: peripheral to memory).
const CHCTL_DIR: u32 = 1 << 4;
/// Circular mode.
const CHCTL_CMEN: u32 = 1 << 5;
/// Memory address increments.
const CHCTL_MNAGA: u32 = 1 << 7;

/// The value the bring-up self-check writes to `CHMADDR` and reads back.
const SELF_CHECK: u32 = 0xA5A5_5A5A;

/// The receive and transmit channel of one USART.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct UsartChannels {
    pub rx: u8,
    pub tx: u8,
}

/// The request map of the model (the table in the module docs), before the descriptor's say.
#[cfg(hal_dma = "f10x")]
const fn model_channels(usart: PeriphLabel) -> Option<UsartChannels> {
    match usart {
        PeriphLabel::Usart0 => Some(UsartChannels { rx: 4, tx: 3 }),
        PeriphLabel::Usart1 => Some(UsartChannels { rx: 5, tx: 6 }),
        PeriphLabel::Usart2 => Some(UsartChannels { rx: 2, tx: 1 }),
        _ => None,
    }
}

/// The request map of the model (the table in the module docs), before the descriptor's say.
#[cfg(any(hal_dma = "f1x0", hal_dma = "mm32"))]
const fn model_channels(usart: PeriphLabel) -> Option<UsartChannels> {
    match usart {
        PeriphLabel::Usart0 => Some(UsartChannels { rx: 2, tx: 1 }),
        PeriphLabel::Usart1 => Some(UsartChannels { rx: 4, tx: 3 }),
        _ => None,
    }
}

/// The vectors of a USART the compiled chip carries, or `None` for any other label.
pub(crate) const fn usart_irqs(usart: PeriphLabel) -> Option<UsartIrqs> {
    if !usart.is_usart() || DESCRIPTOR.addrs.get(usart).is_none() {
        return None;
    }
    DESCRIPTOR.irqs.usart[usart.index()]
}

/// The DMA channels of `usart` on the compiled chip: `None` unless the chip carries the USART, its
/// vectors and the DMA controller, and the model maps it.
pub(crate) const fn usart_channels(usart: PeriphLabel) -> Option<UsartChannels> {
    if usart_irqs(usart).is_none() || DESCRIPTOR.addrs.get(PeriphLabel::Dma0).is_none() {
        return None;
    }
    model_channels(usart)
}

/// What a USART's DMA drivers need: its vectors and its two channels.
pub(crate) struct UsartDma {
    pub irqs: UsartIrqs,
    pub rx: Channel,
    pub tx: Channel,
}

/// Resolve `usart`'s DMA, or [`DescriptorError::Unsupported`] where the chip has none for it.
pub(crate) fn resolve(chip: &Chip, usart: PeriphLabel) -> Result<UsartDma, DescriptorError> {
    let (Some(irqs), Some(ch)) = (usart_irqs(usart), usart_channels(usart)) else {
        return Err(DescriptorError::Unsupported);
    };
    let base = chip.base(PeriphLabel::Dma0)?;
    Ok(UsartDma {
        irqs,
        rx: Channel::new(base, ch.rx),
        tx: Channel::new(base, ch.tx),
    })
}

// --- one channel's registers ----------------------------------------------------------------------

/// One DMA channel: the controller base and the channel number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Channel {
    base: u32,
    ch: u32,
}

impl Channel {
    #[inline]
    const fn new(base: u32, ch: u8) -> Self {
        Self {
            base,
            ch: ch as u32,
        }
    }

    #[inline(always)]
    fn reg(self, offset: u32) -> Reg32 {
        Reg32::new(self.base, offset + CH_STRIDE * self.ch)
    }

    /// The channel's flag `flag` (0 = `GIF` .. 3 = `ERRIF`) as an `INTF`/`INTC` mask.
    #[inline(always)]
    const fn flag(self, flag: u32) -> u32 {
        1 << (4 * self.ch + flag)
    }

    /// Stop the channel (clear `CHEN`).
    #[inline]
    pub(crate) fn disable(self) {
        self.reg(CHCTL).clear_bits(CHCTL_CHEN);
    }

    /// Write a sentinel to `CHMADDR` and read it back: `true` if the channel responds (the DMA
    /// clock is on). It proves the channel is there, not that it is the one the USART requests;
    /// only a bench loopback proves the map. The real address overwrites the sentinel when the
    /// channel is programmed.
    #[inline]
    pub(crate) fn self_check(self) -> bool {
        let maddr = self.reg(CHMADDR);
        maddr.write(SELF_CHECK);
        maddr.read() == SELF_CHECK
    }

    /// Clear all four of the channel's flags in `INTC`, touching no other channel's.
    #[inline]
    pub(crate) fn clear_flags(self) {
        Reg32::new(self.base, INTC).write(0xF << (4 * self.ch));
    }

    /// The remaining-transfer count `CHCNT`, which counts down as the channel moves bytes.
    #[inline]
    pub(crate) fn remaining(self) -> u16 {
        (self.reg(CHCNT).read() & 0xFFFF) as u16
    }

    /// The channel's full-transfer flag is set and not yet serviced: in circular mode, a lap the
    /// receive vector has not counted.
    #[inline]
    pub(crate) fn ftf_pending(self) -> bool {
        Reg32::new(self.base, INTF).read() & self.flag(1) != 0
    }

    /// Program `paddr`, `maddr`, `len` and the control word `ctl`, then set `CHEN` last, after a
    /// compiler fence so the setup (and the buffer it points at) is not reordered past the start.
    ///
    /// # Safety
    ///
    /// `maddr` must address `len` bytes that stay valid, and that nothing else writes (receive) or
    /// changes (transmit), for as long as the channel runs.
    #[inline]
    unsafe fn start(self, paddr: u32, maddr: u32, len: u16, ctl: u32) {
        self.disable();
        self.reg(CHPADDR).write(paddr);
        self.reg(CHMADDR).write(maddr);
        self.reg(CHCNT).write(u32::from(len));
        // A plain write, so every bit the previous use left (circular, half-transfer) is cleared.
        self.reg(CHCTL).write(ctl);
        compiler_fence(Ordering::Release);
        self.reg(CHCTL).set_bits(CHCTL_CHEN);
    }

    /// Start circular peripheral-to-memory byte reception from `paddr` into `maddr[..len]`, with
    /// the half- and full-transfer interrupts.
    ///
    /// # Safety
    ///
    /// As [`Channel::start`]: the buffer is the channel's for as long as it runs.
    #[inline]
    pub(crate) unsafe fn start_circular_rx(self, paddr: u32, maddr: u32, len: u16) {
        self.start(
            paddr,
            maddr,
            len,
            CHCTL_MNAGA | CHCTL_CMEN | CHCTL_FTFIE | CHCTL_HTFIE,
        );
    }

    /// Start a one-shot memory-to-peripheral byte transfer of `maddr[..len]` to `paddr`, with the
    /// full-transfer interrupt only.
    ///
    /// # Safety
    ///
    /// As [`Channel::start`]: the buffer must not change until the transfer completes.
    #[inline]
    pub(crate) unsafe fn start_oneshot_tx(self, paddr: u32, maddr: u32, len: u16) {
        self.start(paddr, maddr, len, CHCTL_DIR | CHCTL_MNAGA | CHCTL_FTFIE);
    }
}

// --- the state the vectors share with the drivers -------------------------------------------------

/// Per USART (at the label's index), the laps its receive channel has completed. The receive
/// vector is the one writer, so a load and a store are the update.
static RX_WRAPS: [AtomicU32; 3] = [const { AtomicU32::new(0) }; 3];

/// Per USART, whether a transmit transfer is programmed and has not completed. Set by the
/// transmitter before `CHEN`, cleared by the transmit vector (or by the transmitter abandoning a
/// transfer past its deadline). Stores only.
static TX_IN_FLIGHT: [AtomicBool; 3] = [const { AtomicBool::new(false) }; 3];

/// `usart`'s lap counter.
#[inline]
pub(crate) fn rx_wraps(usart: PeriphLabel) -> Option<&'static AtomicU32> {
    RX_WRAPS.get(usart.index())
}

/// `usart`'s in-flight flag.
#[inline]
pub(crate) fn tx_in_flight(usart: PeriphLabel) -> Option<&'static AtomicBool> {
    TX_IN_FLIGHT.get(usart.index())
}

// --- the vectors ----------------------------------------------------------------------------------

/// What one DMA vector carries: per USART (at the label's index), the receive channel and the
/// transmit channel it raises, and whether it is USART1's receive vector (the one
/// `irq::DMA_RX_ISR_METRIC` counts).
#[cfg(any(test, target_arch = "arm"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct VectorUse {
    rx: [Option<u8>; 3],
    tx: [Option<u8>; 3],
    metric: bool,
}

/// Which channels the vector numbered `v` carries, from the descriptor's vectors and the model's
/// channels. Evaluated at compile time for each handler.
#[cfg(any(test, target_arch = "arm"))]
const fn vector_use(v: u16) -> VectorUse {
    let mut u = VectorUse {
        rx: [None; 3],
        tx: [None; 3],
        metric: false,
    };
    let mut i = 0;
    while i < 3 {
        let usart = crate::usart::USARTS[i];
        if let (Some(irqs), Some(ch)) = (usart_irqs(usart), usart_channels(usart)) {
            if irqs.dma_rx as u16 == v {
                u.rx[i] = Some(ch.rx);
                u.metric = u.metric || matches!(usart, PeriphLabel::Usart1);
            }
            if irqs.dma_tx as u16 == v {
                u.tx[i] = Some(ch.tx);
            }
        }
        i += 1;
    }
    u
}

/// The DMA controller base, for the handlers.
#[cfg(target_arch = "arm")]
const DMA_BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Dma0) {
    Some(b) => b,
    None => panic!("a chip with DMA vectors carries the DMA controller"),
};

/// A receive channel's vector body: a full transfer is a lap, counted; the half- and
/// full-transfer flags are cleared. Only this channel's flags are read or cleared.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn service_rx(ch: Channel, wraps: &AtomicU32) {
    let intf = Reg32::new(ch.base, INTF).read();
    let (gif, ftf, htf) = (ch.flag(0), ch.flag(1), ch.flag(2));
    let mut clear = 0;
    if intf & ftf != 0 {
        wraps.store(
            wraps.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Release,
        );
        clear |= ftf | gif;
    }
    if intf & htf != 0 {
        clear |= htf;
    }
    if clear != 0 {
        Reg32::new(ch.base, INTC).write(clear);
    }
}

/// A transmit channel's vector body: on this channel's full transfer, clear its flags, stop the
/// channel and clear the in-flight flag, last, because it is what lets the transmitter program the
/// next transfer. Anything else is not this channel's event. It starts nothing.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn service_tx(ch: Channel, in_flight: &AtomicBool) {
    let ftf = ch.flag(1);
    if Reg32::new(ch.base, INTF).read() & ftf == 0 {
        return;
    }
    Reg32::new(ch.base, INTC).write(ftf | ch.flag(0));
    ch.disable();
    in_flight.store(false, Ordering::Release);
}

/// USART `I`'s channels on vector `V`: each is a compile-time constant, so a handler is
/// straight-line code for exactly the channels it carries.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn serve<const I: usize, const V: u16>() {
    if let Some(ch) = const { vector_use(V).rx[I] } {
        service_rx(Channel::new(DMA_BASE, ch), &RX_WRAPS[I]);
    }
    if let Some(ch) = const { vector_use(V).tx[I] } {
        service_tx(Channel::new(DMA_BASE, ch), &TX_IN_FLIGHT[I]);
    }
}

/// The body of DMA vector `V`: every receive and transmit channel it carries, receive first, then
/// the metric.
#[cfg(target_arch = "arm")]
#[inline(always)]
fn on_vector<const V: u16>() {
    serve::<0, V>();
    serve::<1, V>();
    serve::<2, V>();
    if const { vector_use(V).metric } {
        crate::irq::DMA_RX_ISR_METRIC.record();
    }
}

/// The DMA vectors the serial drivers own, by vendor name: one handler each on the board target,
/// and the list itself for the tests.
macro_rules! dma_vectors {
    ($($name:ident),+ $(,)?) => {
        #[cfg(test)]
        const VECTORS: &[crate::irq::Interrupt] = &[$(crate::irq::Interrupt::$name),+];

        #[cfg(target_arch = "arm")]
        mod handlers {
            $(
                /// A DMA vector of the serial drivers (`crate::dma`).
                ///
                /// # Safety
                ///
                /// Entered by the hardware through the flash vector table only.
                #[no_mangle]
                #[allow(non_snake_case)]
                pub unsafe extern "C" fn $name() {
                    super::on_vector::<{ crate::irq::Interrupt::$name as u16 }>();
                }
            )+
        }
    };
}

#[cfg(hal_dma = "f10x")]
dma_vectors!(
    DMA0_Channel1,
    DMA0_Channel2,
    DMA0_Channel3,
    DMA0_Channel4,
    DMA0_Channel5,
    DMA0_Channel6,
);
#[cfg(hal_dma = "f1x0")]
dma_vectors!(DMA_Channel1_2, DMA_Channel3_4);
#[cfg(hal_dma = "mm32")]
dma_vectors!(DMA1_Channel2_3, DMA1_Channel4_5);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usart::USARTS;

    /// The vendor's vector for a DMA channel (`IRQn_Type`): one per channel on the F10x DMA0
    /// (`DMA0_Channel0_IRQn` = 11), grouped on the F1x0 (`DMA_Channel0` = 9, `1_2` = 10, `3_4` =
    /// 11, `5_6` = 48) and on the MM32 (`DMA1_Channel1` = 9, `2_3` = 10, `4_5` = 11, the HAL's
    /// channels 0, 1-2, 3-4).
    fn vendor_vector(ch: u8) -> u16 {
        if cfg!(hal_dma = "f10x") {
            11 + u16::from(ch)
        } else if cfg!(hal_dma = "mm32") {
            match ch {
                0 => 9,
                1 | 2 => 10,
                _ => 11,
            }
        } else {
            match ch {
                0 => 9,
                1 | 2 => 10,
                3 | 4 => 11,
                _ => 48,
            }
        }
    }

    #[test]
    fn the_channel_map_raises_the_descriptors_vectors() {
        for usart in USARTS {
            let Some(ch) = usart_channels(usart) else {
                continue;
            };
            let irqs = usart_irqs(usart).expect("a mapped USART has vectors");
            assert_eq!(vendor_vector(ch.rx), irqs.dma_rx as u16, "{usart:?} rx");
            assert_eq!(vendor_vector(ch.tx), irqs.dma_tx as u16, "{usart:?} tx");
            assert_ne!(ch.rx, ch.tx);
        }
    }

    #[test]
    fn every_carried_usart_has_channels_and_no_two_share_one() {
        let mut used = [false; 8];
        for usart in USARTS {
            let carried = DESCRIPTOR.addrs.get(usart).is_some();
            let ch = usart_channels(usart);
            assert_eq!(ch.is_some(), carried, "{usart:?}");
            if let Some(ch) = ch {
                for c in [ch.rx, ch.tx] {
                    assert!(c < 7, "{usart:?} channel {c}");
                    assert!(!used[c as usize], "channel {c} twice");
                    used[c as usize] = true;
                }
            }
        }
        // Not a USART: no channels.
        assert_eq!(usart_channels(PeriphLabel::I2c0), None);
    }

    #[test]
    fn the_handlers_cover_every_usart_dma_vector_and_nothing_else() {
        for usart in USARTS {
            if let Some(irqs) = usart_irqs(usart) {
                assert!(VECTORS.contains(&irqs.dma_rx), "{usart:?} rx");
                assert!(VECTORS.contains(&irqs.dma_tx), "{usart:?} tx");
            }
        }
        for &v in VECTORS {
            let u = vector_use(v as u16);
            let carries = u.rx.iter().chain(u.tx.iter()).any(Option::is_some);
            assert!(carries, "{v:?} services nothing");
        }
    }

    #[test]
    fn a_grouped_vector_services_both_halves_of_its_usart() {
        let u1 = usart_irqs(PeriphLabel::Usart1).expect("USART1 everywhere");
        let u = vector_use(u1.dma_rx as u16);
        let ch = usart_channels(PeriphLabel::Usart1).expect("USART1 everywhere");
        assert_eq!(u.rx[1], Some(ch.rx));
        assert!(u.metric);
        if cfg!(any(hal_dma = "f1x0", hal_dma = "mm32")) {
            // One vector, both channels, and nothing of USART0's.
            assert_eq!(u.tx[1], Some(ch.tx));
            assert_eq!((u.rx[0], u.tx[0]), (None, None));
        } else {
            assert_eq!(u.tx[1], None);
            assert!(!vector_use(u1.dma_tx as u16).metric);
        }
    }

    #[test]
    fn channel_flags_are_four_per_channel() {
        let ch = Channel::new(0x4002_0000, 4);
        assert_eq!(ch.flag(0), 1 << 16);
        assert_eq!(ch.flag(1), 1 << 17);
        assert_eq!(ch.flag(2), 1 << 18);
        assert_eq!(Channel::new(0x4002_0000, 6).flag(1), 1 << 25);
    }
}
