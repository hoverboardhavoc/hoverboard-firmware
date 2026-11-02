// SPDX-License-Identifier: GPL-3.0-or-later
//! The USART: bring-up, the split into transmit and receive halves, and the register operations
//! the DMA drivers in [`crate::usart_rx`] and [`crate::usart_tx`] make through those halves.
//!
//! Two register models, one per build (specs/hal.md, section 3.1):
//!
//! - `f10x` (GD32F10x, `gd32f10x_usart.h`): `STAT` first, one `DATA` register for both directions,
//!   `UEN` at `CTL0` bit 13, and the sticky flags cleared by a `STAT` read then a `DATA` read.
//! - `f1x0` (GD32F1x0, `gd32f1x0_usart.h`): the `CTL` registers first, separate `RDATA`/`TDATA`,
//!   `UEN` at `CTL0` bit 0, and the sticky flags cleared by writing them to `INTC`.
//!
//! The bit positions inside `CTL0`/`CTL1`/`CTL2` and `STAT` that this driver uses are the same on
//! both, so they live here; the models carry the offsets, the enable bit and the flag clear.
//!
//! - `mm32` (MM32SPIN05, MindMotion's `reg_uart.h`): another register set altogether (`TDR`, `RDR`,
//!   `CSR`, `ISR`, `IER`, `ICR`, `GCR`, `CCR`, `BRR`, `FRA`), one DMA enable for both directions and
//!   no idle-line detection. `usart/mm32.rs` carries the divisor, the programming, the flags and the
//!   halves' register operations; the pin routing and the handles here are shared.

#[cfg(hal_usart = "f10x")]
#[path = "usart/f10x.rs"]
mod model;
#[cfg(hal_usart = "f1x0")]
#[path = "usart/f1x0.rs"]
mod model;
#[cfg(hal_usart = "mm32")]
#[path = "usart/mm32.rs"]
mod model;

#[cfg(all(hal_usart = "mm32", target_arch = "arm"))]
pub(crate) use model::service_rx_flags;
#[cfg(hal_usart = "mm32")]
use model::{divisor_for, program};


use crate::addr::PeriphLabel;
use crate::chip::{Chip, ClockPath};
use crate::clock::ClockConfig;
use crate::config::{Oversampling, UsartConfig, UsartFrame};
#[cfg(not(hal_usart = "mm32"))]
use crate::config::{Parity, WordLen};
use crate::error::DescriptorError;
use crate::gpio::{configure_af, AfMode, PinAf};
#[cfg(not(hal_usart = "mm32"))]
use crate::reg::Reg32;

#[cfg(not(hal_usart = "mm32"))]
const CTL0_REN: u32 = 1 << 2;
#[cfg(not(hal_usart = "mm32"))]
const CTL0_TEN: u32 = 1 << 3;
/// The IDLE-line interrupt enable.
#[cfg(not(hal_usart = "mm32"))]
const CTL0_IDLEIE: u32 = 1 << 4;
#[cfg(not(hal_usart = "mm32"))]
const CTL0_PM: u32 = 1 << 9;
#[cfg(not(hal_usart = "mm32"))]
const CTL0_PCEN: u32 = 1 << 10;
#[cfg(not(hal_usart = "mm32"))]
const CTL0_WL: u32 = 1 << 12;
#[cfg(not(hal_usart = "mm32"))]
const CTL1_STB: u32 = 0b11 << 12;
/// The error interrupt enable: under DMA reception it raises the USART vector on an overrun, a
/// framing error or a noise error.
#[cfg(not(hal_usart = "mm32"))]
const CTL2_ERRIE: u32 = 1 << 0;
/// DMA reception enable.
#[cfg(not(hal_usart = "mm32"))]
const CTL2_DENR: u32 = 1 << 6;
/// DMA transmission enable.
#[cfg(not(hal_usart = "mm32"))]
const CTL2_DENT: u32 = 1 << 7;

/// Parity, framing, noise and overrun errors (`STAT` bits 0-3).
#[cfg(not(hal_usart = "mm32"))]
#[cfg(target_arch = "arm")]
pub(crate) const STAT_ERRORS: u32 = 0b1111;
/// The IDLE line detected flag.
#[cfg(not(hal_usart = "mm32"))]
#[cfg(target_arch = "arm")]
pub(crate) const STAT_IDLEF: u32 = 1 << 4;
#[cfg(not(hal_usart = "mm32"))]
const STAT_TC: u32 = 1 << 6;
#[cfg(not(hal_usart = "mm32"))]
const STAT_TBE: u32 = 1 << 7;

/// The USART labels at their index (`PeriphLabel::index`), the index the per-USART state of the
/// receive and transmit drivers is kept at.
#[cfg(any(test, target_arch = "arm"))]
pub(crate) const USARTS: [PeriphLabel; 3] = [
    PeriphLabel::Usart0,
    PeriphLabel::Usart1,
    PeriphLabel::Usart2,
];

// --- the baud divisor -----------------------------------------------------------------------------

/// The clock feeding `usart`'s baud generator: APB2 for USART0, APB1 for USART1 and USART2 (the
/// clock `usart_baudrate_set` reads: `CK_APB2` / `CK_USART` at its reset select for USART0,
/// `CK_APB1` for the others). `None` for a label that is not a USART.
const fn input_clock(clock: &ClockConfig, usart: PeriphLabel) -> Option<u32> {
    match usart {
        PeriphLabel::Usart0 => Some(clock.apb2_hz()),
        PeriphLabel::Usart1 | PeriphLabel::Usart2 => Some(clock.apb1_hz()),
        _ => None,
    }
}

/// The `BAUD` value for `baud` from a `uclk` clock at oversampling by 16: the SPL's round-to-nearest
/// `(uclk + baud / 2) / baud`, whose low 16 bits are `INTDIV` (15:4) and `FRADIV` (3:0). `None`
/// where the divider cannot express it: a zero baud, `INTDIV` of 0 (a baud above `uclk / 16`), or
/// a divisor past 16 bits.
#[cfg(not(hal_usart = "mm32"))]
const fn baud_divisor(uclk: u32, baud: u32) -> Option<u32> {
    if baud == 0 {
        return None;
    }
    let udiv = (uclk as u64 + baud as u64 / 2) / baud as u64;
    if udiv < 16 || udiv > 0xFFFF {
        None
    } else {
        Some(udiv as u32)
    }
}

/// The `BAUD` value for a bring-up, or [`DescriptorError::UsartConfig`]. Oversampling by 8 is
/// refused: the F10x USART has no `OVSMOD`, and no port on either chip uses it.
#[cfg(not(hal_usart = "mm32"))]
const fn divisor_for(
    clock: &ClockConfig,
    usart: PeriphLabel,
    baud: u32,
    oversampling: Oversampling,
) -> Result<u32, DescriptorError> {
    if !matches!(oversampling, Oversampling::By16) {
        return Err(DescriptorError::UsartConfig);
    }
    let Some(uclk) = input_clock(clock, usart) else {
        return Err(DescriptorError::UsartConfig);
    };
    match baud_divisor(uclk, baud) {
        Some(d) => Ok(d),
        None => Err(DescriptorError::UsartConfig),
    }
}

// --- the handles ----------------------------------------------------------------------------------

/// A brought-up USART. Not `Copy`: it is the one handle to the instance. [`Usart::split`] gives
/// the transmit and receive halves to their owners.
#[derive(Debug)]
pub struct Usart {
    label: PeriphLabel,
    base: u32,
}

/// The transmit half of a split [`Usart`], consumed by [`crate::usart_tx::DmaTx`].
#[derive(Debug)]
pub struct UsartTx {
    label: PeriphLabel,
    base: u32,
}

/// The receive half of a split [`Usart`], consumed by [`crate::usart_rx::RingBufferedRx`].
#[derive(Debug)]
pub struct UsartRx {
    label: PeriphLabel,
    base: u32,
}

impl Usart {
    /// Bring up `instance` on its pins at `baud`, 8N1, oversampling by 16.
    ///
    /// The values are the stored ones (`specs/config-read-once.md` section 5) and nothing here
    /// checks them: the divisor (a baud the clock cannot reach is refused before any register is
    /// written), the USART clock, TX as AF push-pull and RX as an AF input on their stored
    /// alternate functions, then the registers as [`Usart::bring_up`] writes them. On the F10x the
    /// routing is the AFIO remap the caller writes from the stored `remap` key ([`crate::afio`]).
    pub fn new(
        chip: &Chip,
        clock: &ClockConfig,
        instance: PeriphLabel,
        tx: PinAf,
        rx: PinAf,
        baud: u32,
    ) -> Result<Usart, DescriptorError> {
        let base = chip.base(instance)?;
        let rcu = chip.rcu_base()?;
        let brr = divisor_for(clock, instance, baud, Oversampling::By16)?;

        crate::clock::enable_usart(rcu, ClockPath, instance)?;
        configure_af(chip.clocked_port_base(tx.port), tx.pin, AfMode::PushPull, tx.af);
        configure_af(chip.clocked_port_base(rx.port), rx.pin, AfMode::Input, rx.af);

        program(base, brr, UsartFrame::EIGHT_N_ONE);
        Ok(Usart {
            label: instance,
            base,
        })
    }

    /// Program `cfg.usart`'s registers alone: no clock enable and no pin routing (the golden-trace
    /// bring-ups compare exactly these registers against the SPL's `usart_*` calls).
    ///
    /// The writes: clear `UEN` (`usart_disable`), `BAUD` (`usart_baudrate_set`), the frame fields
    /// `WL`, `STB`, `PM`/`PCEN` (`usart_word_length_set`, `usart_stop_bit_set`,
    /// `usart_parity_config`), `REN`/`TEN`, then `UEN` (`usart_enable`).
    pub fn bring_up(
        chip: &Chip,
        clock: &ClockConfig,
        cfg: &UsartConfig,
    ) -> Result<Usart, DescriptorError> {
        if !cfg.usart.is_usart() {
            return Err(DescriptorError::Unsupported);
        }
        let base = chip.base(cfg.usart)?;
        let brr = divisor_for(clock, cfg.usart, cfg.baud, cfg.oversampling)?;
        #[cfg(hal_usart = "mm32")]
        model::check_frame(&cfg.frame)?;
        program(base, brr, cfg.frame);
        Ok(Usart {
            label: cfg.usart,
            base,
        })
    }

    /// Write one byte and wait until it has left the shift register: poll `TBE`, write the data
    /// register, poll `TC`. Both waits are unbounded (a bring-up primitive; the link's write path
    /// is [`crate::serial::SplitSerial`], whose every wait is bounded).
    #[cfg(not(hal_usart = "mm32"))]
    pub fn write_byte(&self, byte: u8) {
        let stat = Reg32::new(self.base, model::STAT);
        while stat.read() & STAT_TBE == 0 {}
        Reg32::new(self.base, model::TDATA).write(u32::from(byte));
        while stat.read() & STAT_TC == 0 {}
    }

    /// Split into the transmit and receive halves.
    #[inline]
    pub fn split(self) -> (UsartTx, UsartRx) {
        (
            UsartTx {
                label: self.label,
                base: self.base,
            },
            UsartRx {
                label: self.label,
                base: self.base,
            },
        )
    }
}

/// The register sequence of a bring-up, after every check has passed.
#[cfg(not(hal_usart = "mm32"))]
fn program(base: u32, brr: u32, frame: UsartFrame) {
    let ctl0 = Reg32::new(base, model::CTL0);
    ctl0.clear_bits(model::CTL0_UEN);
    Reg32::new(base, model::BAUD).write(brr);
    let wl = match frame.word_len {
        WordLen::Eight => 0,
        WordLen::Nine => CTL0_WL,
    };
    let parity = match frame.parity {
        Parity::None => 0,
        Parity::Even => CTL0_PCEN,
        Parity::Odd => CTL0_PCEN | CTL0_PM,
    };
    ctl0.modify(
        CTL0_WL | CTL0_PM | CTL0_PCEN | model::CTL0_FRAME_EXTRA,
        wl | parity,
    );
    Reg32::new(base, model::CTL1).modify(CTL1_STB, (frame.stop as u32) << 12);
    ctl0.set_bits(CTL0_REN | CTL0_TEN);
    ctl0.set_bits(model::CTL0_UEN);
}

impl UsartTx {
    /// The instance.
    #[inline]
    pub(crate) fn label(&self) -> PeriphLabel {
        self.label
    }

    /// The absolute address of the transmit data register (the DMA destination).
    #[cfg(not(hal_usart = "mm32"))]
    #[inline]
    pub(crate) fn tdata_addr(&self) -> u32 {
        self.base + model::TDATA
    }

    /// Route the transmitter's byte fetches to the DMA controller (`DENT`).
    #[cfg(not(hal_usart = "mm32"))]
    #[inline]
    pub(crate) fn enable_dma(&self) {
        Reg32::new(self.base, model::CTL2).set_bits(CTL2_DENT);
    }

    /// `TC`: the last byte has left the shift register.
    #[cfg(not(hal_usart = "mm32"))]
    #[inline]
    pub(crate) fn tx_complete(&self) -> bool {
        Reg32::new(self.base, model::STAT).read() & STAT_TC != 0
    }
}

impl UsartRx {
    /// The instance.
    #[inline]
    pub(crate) fn label(&self) -> PeriphLabel {
        self.label
    }

    /// The absolute address of the receive data register (the DMA source).
    #[cfg(not(hal_usart = "mm32"))]
    #[inline]
    pub(crate) fn rdata_addr(&self) -> u32 {
        self.base + model::RDATA
    }

    /// Hand received bytes to the DMA controller (`DENR`), raise the USART vector on a line error
    /// (`ERRIE`) and on an idle line (`IDLEIE`).
    #[cfg(not(hal_usart = "mm32"))]
    #[inline]
    pub(crate) fn enable_dma_ring(&self) {
        Reg32::new(self.base, model::CTL2).set_bits(CTL2_DENR | CTL2_ERRIE);
        Reg32::new(self.base, model::CTL0).set_bits(CTL0_IDLEIE);
    }
}

/// The USART receive vector's register work for the instance at `base`: read `STAT`, clear every
/// line-error and IDLE flag it shows, and answer whether a line error was among them.
///
/// The vector is level-sensitive, so a flag it raised and this left set re-enters it at once and
/// for ever: every `ERRIE` source (overrun, framing, noise) and `IDLEF` is cleared. Parity is
/// cleared with them for the same reason, though the 8N1 frame never raises it.
#[cfg(all(not(hal_usart = "mm32"), target_arch = "arm"))]
#[inline(always)]
pub(crate) fn service_rx_flags(base: u32) -> bool {
    let stat = Reg32::new(base, model::STAT).read();
    let flags = stat & (STAT_ERRORS | STAT_IDLEF);
    if flags != 0 {
        model::clear_flags(base, flags);
    }
    stat & STAT_ERRORS != 0
}

#[cfg(all(test, not(hal_usart = "mm32")))]
mod tests {
    use super::*;

    const TREE: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;

    #[test]
    fn usart1_at_115200_on_the_72_mhz_tree_is_0x139() {
        // The golden bring-up's headline value: APB1 = 36 MHz, (36e6 + 57600) / 115200 = 313.
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::Usart1, 115_200, Oversampling::By16),
            Ok(0x139)
        );
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::Usart2, 115_200, Oversampling::By16),
            Ok(0x139)
        );
    }

    #[test]
    fn usart0_divides_the_apb2_clock() {
        // APB2 = 72 MHz: (72e6 + 57600) / 115200 = 625.
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::Usart0, 115_200, Oversampling::By16),
            Ok(625)
        );
    }

    #[test]
    fn the_divisor_rounds_to_nearest() {
        // 36e6 / 460800 = 78.125 -> 78; 36e6 / 9600 = 3750 exactly; 36e6 / 230400 = 156.25 -> 156.
        assert_eq!(baud_divisor(36_000_000, 460_800), Some(78));
        assert_eq!(baud_divisor(36_000_000, 9_600), Some(3750));
        assert_eq!(baud_divisor(36_000_000, 230_400), Some(156));
        // Half a step rounds up: 115 / 7 = 16.43 -> 16, 116 / 7 = 16.57 -> 17.
        assert_eq!(baud_divisor(115, 7), Some(16));
        assert_eq!(baud_divisor(116, 7), Some(17));
    }

    #[test]
    fn unreachable_bauds_are_refused() {
        let refuse = |baud| divisor_for(&TREE, PeriphLabel::Usart1, baud, Oversampling::By16);
        assert_eq!(refuse(0), Err(DescriptorError::UsartConfig));
        // Above APB1 / 16: INTDIV would be 0.
        assert_eq!(refuse(36_000_000 / 15), Err(DescriptorError::UsartConfig));
        assert_eq!(refuse(36_000_000 / 16), Ok(16));
        // Below APB1 / 65535: the divisor would not fit 16 bits.
        assert_eq!(refuse(500), Err(DescriptorError::UsartConfig));
        assert_eq!(refuse(550), Ok(65_455));
    }

    #[test]
    fn oversampling_by_8_and_non_usart_labels_are_refused() {
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::Usart1, 115_200, Oversampling::By8),
            Err(DescriptorError::UsartConfig)
        );
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::I2c0, 115_200, Oversampling::By16),
            Err(DescriptorError::UsartConfig)
        );
    }

    #[test]
    fn the_model_offsets_are_the_vendor_headers() {
        // gd32f10x_usart.h / gd32f1x0_usart.h.
        let (stat, tdata, rdata, baud, ctl0, ctl1, ctl2, uen) = (
            model::STAT,
            model::TDATA,
            model::RDATA,
            model::BAUD,
            model::CTL0,
            model::CTL1,
            model::CTL2,
            model::CTL0_UEN,
        );
        if cfg!(hal_usart = "f10x") {
            assert_eq!(
                (stat, tdata, rdata, baud, ctl0, ctl1, ctl2, uen),
                (0x00, 0x04, 0x04, 0x08, 0x0C, 0x10, 0x14, 1 << 13)
            );
        } else {
            assert_eq!(
                (stat, tdata, rdata, baud, ctl0, ctl1, ctl2, uen),
                (0x1C, 0x28, 0x24, 0x0C, 0x00, 0x04, 0x08, 1 << 0)
            );
        }
    }
}
