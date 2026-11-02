// SPDX-License-Identifier: GPL-3.0-or-later
//! The MM32SPIN05 UART model (MindMotion's `reg_uart.h` and `hal_uart.c`, LibSamples V1.13).
//!
//! | reg   | offset | fields the HAL uses                                                  |
//! |-------|--------|----------------------------------------------------------------------|
//! | `TDR` | `0x00` | the byte to send                                                     |
//! | `RDR` | `0x04` | the byte received                                                    |
//! | `CSR` | `0x08` | TXC(0) transmit complete, TXFULL(2) the transmit buffer full          |
//! | `ISR` | `0x0C` | RXOERR(3), RXPERR(4), RXFERR(5), RXBRK(6), the line-error flags      |
//! | `IER` | `0x10` | the same bits, their interrupt enables                               |
//! | `ICR` | `0x14` | the same bits, a written 1 clears the flag                           |
//! | `GCR` | `0x18` | UART(0) enable, DMA(1) both directions, AUTOFLOW(2), RX(3), TX(4)    |
//! | `CCR` | `0x1C` | PEN(0), PSEL(1), SPB0(2), CHAR(5:4), SPB1(6)                         |
//! | `BRR` | `0x20` | the integer part of `pclk / baud / 16`                               |
//! | `FRA` | `0x24` | the fraction, in sixteenths                                          |
//!
//! The bring-up is `UART_Init` then `UART_Cmd`: `CCR` (`CHAR`, the stop bits, the parity), `GCR`
//! (`TX`/`RX`, flow control off), `BRR` and `FRA`, then `GCR.UART`. The divisor is the vendor's,
//! `pclk / baud` truncated, split into `BRR = div / 16` and `FRA = div % 16` (no rounding, unlike
//! the GD32 SPL's). The block has one DMA enable for both directions and no idle-line flag, so the
//! receive vector here takes the line errors only.

use super::{input_clock, Usart, UsartRx, UsartTx};
use crate::addr::PeriphLabel;
use crate::clock::ClockConfig;
use crate::config::{Oversampling, Parity, StopBits, UsartFrame, WordLen};
use crate::error::DescriptorError;
use crate::reg::Reg32;

pub(crate) const TDR: u32 = 0x00;
pub(crate) const RDR: u32 = 0x04;
const CSR: u32 = 0x08;
#[cfg(target_arch = "arm")]
const ISR: u32 = 0x0C;
const IER: u32 = 0x10;
#[cfg(target_arch = "arm")]
const ICR: u32 = 0x14;
const GCR: u32 = 0x18;
const CCR: u32 = 0x1C;
const BRR: u32 = 0x20;
const FRA: u32 = 0x24;

const CSR_TXC: u32 = 1 << 0;
const CSR_TXFULL: u32 = 1 << 2;

/// The line errors: receive overrun, parity, framing and break (`ISR`/`IER`/`ICR` bits 3-6).
const LINE_ERRORS: u32 = 0b1111 << 3;

const GCR_UART: u32 = 1 << 0;
const GCR_DMA: u32 = 1 << 1;
const GCR_AUTOFLOW: u32 = 1 << 2;
const GCR_RX: u32 = 1 << 3;
const GCR_TX: u32 = 1 << 4;

const CCR_PEN: u32 = 1 << 0;
const CCR_PSEL: u32 = 1 << 1;
const CCR_SPB0: u32 = 1 << 2;
const CCR_CHAR: u32 = 0b11 << 4;
const CCR_CHAR_8B: u32 = 0b11 << 4;
const CCR_SPB1: u32 = 1 << 6;

/// The divisor `pclk / baud` (truncated, as `UART_Init` computes it), or `None` where `BRR` cannot
/// express it: a zero baud, a baud above `pclk / 16` (`BRR` of 0), or `BRR` past 16 bits.
const fn baud_divisor(pclk: u32, baud: u32) -> Option<u32> {
    if baud == 0 {
        return None;
    }
    let div = pclk / baud;
    if div < 16 || div / 16 > 0xFFFF {
        None
    } else {
        Some(div)
    }
}

/// The divisor for a bring-up, or [`DescriptorError::UsartConfig`]. The block samples by 16 only,
/// so oversampling by 8 is refused.
pub(super) const fn divisor_for(
    clock: &ClockConfig,
    usart: PeriphLabel,
    baud: u32,
    oversampling: Oversampling,
) -> Result<u32, DescriptorError> {
    if !matches!(oversampling, Oversampling::By16) {
        return Err(DescriptorError::UsartConfig);
    }
    let Some(pclk) = input_clock(clock, usart) else {
        return Err(DescriptorError::UsartConfig);
    };
    match baud_divisor(pclk, baud) {
        Some(d) => Ok(d),
        None => Err(DescriptorError::UsartConfig),
    }
}

/// Refuse a frame this model does not program: nine data bits (the block's ninth bit is its
/// `B8` synchronous-frame feature, not a data bit the SPL's frame means).
pub(super) const fn check_frame(frame: &UsartFrame) -> Result<(), DescriptorError> {
    match frame.word_len {
        WordLen::Eight => Ok(()),
        WordLen::Nine => Err(DescriptorError::UsartConfig),
    }
}

/// The `CCR` bits of a frame (eight data bits; [`check_frame`] refused nine).
const fn ccr_bits(frame: UsartFrame) -> (u32, u32) {
    let stop = match frame.stop {
        StopBits::One => 0,
        StopBits::Two => CCR_SPB0,
        StopBits::Half => CCR_SPB1,
        StopBits::OneAndHalf => CCR_SPB0 | CCR_SPB1,
    };
    let parity = match frame.parity {
        Parity::None => 0,
        Parity::Even => CCR_PEN | CCR_PSEL,
        Parity::Odd => CCR_PEN,
    };
    (stop, parity)
}

/// The register sequence of a bring-up, after every check has passed: `UART_Init`'s `CCR` and
/// `GCR` fields and the divisor, then `UART_Cmd`.
pub(super) fn program(base: u32, div: u32, frame: UsartFrame) {
    let ccr = Reg32::new(base, CCR);
    let gcr = Reg32::new(base, GCR);
    let (stop, parity) = ccr_bits(frame);
    ccr.modify(CCR_CHAR, CCR_CHAR_8B);
    ccr.modify(CCR_SPB0 | CCR_SPB1, stop);
    ccr.modify(CCR_PEN | CCR_PSEL, parity);
    gcr.modify(GCR_TX | GCR_RX, GCR_TX | GCR_RX);
    gcr.modify(GCR_AUTOFLOW, 0);
    Reg32::new(base, BRR).write(div / 16);
    Reg32::new(base, FRA).write(div % 16);
    gcr.modify(GCR_UART, GCR_UART);
}

impl Usart {
    /// Write one byte and wait until it has left the shift register: wait for room in the transmit
    /// buffer (`CSR.TXFULL` clear), write `TDR`, wait for `CSR.TXC`. Both waits are unbounded (a
    /// bring-up primitive; the link's write path is [`crate::serial::SplitSerial`]).
    pub fn write_byte(&self, byte: u8) {
        let csr = Reg32::new(self.base, CSR);
        while csr.read() & CSR_TXFULL != 0 {}
        Reg32::new(self.base, TDR).write(u32::from(byte));
        while csr.read() & CSR_TXC == 0 {}
    }
}

impl UsartTx {
    /// The absolute address of `TDR` (the DMA destination).
    #[inline]
    pub(crate) fn tdata_addr(&self) -> u32 {
        self.base + TDR
    }

    /// Hand the transmitter's byte fetches to the DMA controller (`GCR.DMA`, shared with the
    /// receiver).
    #[inline]
    pub(crate) fn enable_dma(&self) {
        Reg32::new(self.base, GCR).set_bits(GCR_DMA);
    }

    /// `CSR.TXC`: the last byte has left the shift register.
    #[inline]
    pub(crate) fn tx_complete(&self) -> bool {
        Reg32::new(self.base, CSR).read() & CSR_TXC != 0
    }
}

impl UsartRx {
    /// The absolute address of `RDR` (the DMA source).
    #[inline]
    pub(crate) fn rdata_addr(&self) -> u32 {
        self.base + RDR
    }

    /// Hand received bytes to the DMA controller (`GCR.DMA`) and raise the UART vector on a line
    /// error (`IER` overrun, parity, framing, break). The block has no idle-line interrupt.
    #[inline]
    pub(crate) fn enable_dma_ring(&self) {
        Reg32::new(self.base, GCR).set_bits(GCR_DMA);
        Reg32::new(self.base, IER).set_bits(LINE_ERRORS);
    }
}

/// The UART receive vector's register work for the instance at `base`: read `ISR`, clear every
/// line-error flag it shows through `ICR` (a written 1 clears), and answer whether one was set. The
/// vector is level-sensitive, so a flag left set would re-enter it for ever.
#[cfg(target_arch = "arm")]
#[inline(always)]
pub(crate) fn service_rx_flags(base: u32) -> bool {
    let flags = Reg32::new(base, ISR).read() & LINE_ERRORS;
    if flags != 0 {
        Reg32::new(base, ICR).write(flags);
    }
    flags != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    const TREE: ClockConfig = ClockConfig::REFERENCE_72M_HSI;

    #[test]
    fn uart2_at_115200_on_the_hsi_tree() {
        // APB1 72 MHz: 72e6 / 115200 = 625, BRR 39, FRA 1.
        let d = divisor_for(&TREE, PeriphLabel::Usart1, 115_200, Oversampling::By16);
        assert_eq!(d, Ok(625));
        assert_eq!((625 / 16, 625 % 16), (39, 1));
        // 36 MHz: 312.5 truncates to 312.
        assert_eq!(baud_divisor(36_000_000, 115_200), Some(312));
    }

    #[test]
    fn uart1_divides_the_apb2_clock() {
        // APB2 72 MHz: 72e6 / 115200 = 625.
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::Usart0, 115_200, Oversampling::By16),
            Ok(625)
        );
    }

    #[test]
    fn unreachable_bauds_and_frames_are_refused() {
        let refuse = |baud| divisor_for(&TREE, PeriphLabel::Usart1, baud, Oversampling::By16);
        assert_eq!(refuse(0), Err(DescriptorError::UsartConfig));
        assert_eq!(refuse(72_000_000 / 15), Err(DescriptorError::UsartConfig));
        assert_eq!(refuse(72_000_000 / 16), Ok(16));
        assert_eq!(
            divisor_for(&TREE, PeriphLabel::Usart1, 115_200, Oversampling::By8),
            Err(DescriptorError::UsartConfig)
        );
        let nine = UsartFrame {
            word_len: WordLen::Nine,
            ..UsartFrame::EIGHT_N_ONE
        };
        assert_eq!(check_frame(&nine), Err(DescriptorError::UsartConfig));
        assert_eq!(check_frame(&UsartFrame::EIGHT_N_ONE), Ok(()));
    }

    #[test]
    fn frame_bits_are_the_vendor_codes() {
        // UART_StopBits_2 = SPB0, UART_StopBits_1_5 = SPB0 | SPB1, UART_Parity_Even = PEN | PSEL.
        let f = UsartFrame {
            word_len: WordLen::Eight,
            parity: Parity::Even,
            stop: StopBits::OneAndHalf,
        };
        assert_eq!(ccr_bits(f), (CCR_SPB0 | CCR_SPB1, CCR_PEN | CCR_PSEL));
        assert_eq!(ccr_bits(UsartFrame::EIGHT_N_ONE), (0, 0));
    }
}
