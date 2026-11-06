// SPDX-License-Identifier: GPL-3.0-or-later
//! Error types.

use crate::addr::PeriphLabel;

/// A bring-up refused: the request names something the chip does not carry, or a configuration
/// the HAL cannot express. The stored pin values are not checked here
/// (`specs/config-read-once.md` section 5); the CLI checks them before they are written.
///
/// The firmware records these in its boot report through [`DescriptorError::code`]. The codes are
/// the numbering `runtime-hal` used for the variants that remain, so a boot report reads the same
/// across the move; the two that described a mismatched runtime selector (codes 1 and 2) went with
/// the selectors (specs/hal.md, section 4), and the four pin and claim refusals (codes 5, 6, 9 and
/// 10) with the checks (`specs/config-read-once.md` section 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DescriptorError {
    /// The chip has no base address for this peripheral.
    MissingBase(PeriphLabel),
    /// The serial configuration cannot be expressed (a baud the divider cannot reach).
    UsartConfig,
    /// The capability exists in the request but not in the HAL.
    Unsupported,
    /// A bring-up's read-back did not confirm its configuration.
    SelfCheckFailed,
}

impl DescriptorError {
    /// The boot report's code for the error.
    pub const fn code(&self) -> u8 {
        match self {
            DescriptorError::MissingBase(_) => 3,
            DescriptorError::UsartConfig => 4,
            DescriptorError::Unsupported => 7,
            DescriptorError::SelfCheckFailed => 8,
        }
    }
}

/// A USART receive error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum UsartError {
    Overrun,
    /// The DMA ring was lapped: the reader fell a whole ring behind the writer.
    RingOverrun,
    /// A line error (framing, noise or overrun) the receive interrupt recorded.
    LineError,
    Framing,
    Parity,
    Noise,
    Other,
}

impl core::fmt::Display for UsartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self, f)
    }
}

impl core::error::Error for UsartError {}

impl embedded_io::Error for UsartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// An I2C transfer error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum I2cError {
    Bus,
    ArbitrationLoss,
    NoAcknowledgeAddress,
    NoAcknowledgeData,
    Overrun,
    Timeout,
    Other,
    /// The bus clear ([`crate::i2c::I2c::bus_clear`]) ran and SDA still read low: a slave holds
    /// the data line through the clock pulses and the STOP.
    BusHeld,
}

impl I2cError {
    /// A fixed byte per variant for the firmware's status record (`board.status`). The high bit is
    /// set on every one, so a byte that carries either this or a [`DescriptorError::code`] (which
    /// counts up from 1) says which it is. The numbers are runtime-hal's, so a boot report reads the
    /// same on either image; a new variant takes the next number.
    pub const fn code(&self) -> u8 {
        match self {
            I2cError::Bus => 0x81,
            I2cError::ArbitrationLoss => 0x82,
            I2cError::NoAcknowledgeAddress => 0x83,
            I2cError::NoAcknowledgeData => 0x84,
            I2cError::Overrun => 0x85,
            I2cError::Timeout => 0x86,
            I2cError::Other => 0x87,
            I2cError::BusHeld => 0x88,
        }
    }
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            // A data line held through the bus clear is a bus-level fault, the same kind as BERR.
            I2cError::Bus | I2cError::BusHeld => ErrorKind::Bus,
            I2cError::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            I2cError::NoAcknowledgeAddress => {
                ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
            }
            I2cError::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            I2cError::Overrun => ErrorKind::Overrun,
            I2cError::Timeout | I2cError::Other => ErrorKind::Other,
        }
    }
}

/// An SPI transfer error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum SpiError {
    ModeFault,
    Overrun,
    Crc,
    Other,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        use embedded_hal::spi::ErrorKind;
        match self {
            SpiError::ModeFault => ErrorKind::ModeFault,
            SpiError::Overrun => ErrorKind::Overrun,
            SpiError::Crc | SpiError::Other => ErrorKind::Other,
        }
    }
}

/// An ADC error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum AdcError {
    /// A conversion or calibration did not complete within its spin budget.
    Timeout,
    Other,
}

/// A clock-tree error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum ClockError {
    SourceNotStable,
    PllNotLocked,
    SwitchNotConfirmed,
    InvalidPll,
    InvalidPrescaler,
    /// The requested system clock is above what the part's flash wait states allow.
    InvalidWaitStates,
}

/// A watchdog error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum WatchdogError {
    /// The IRC40K oscillator did not report stable.
    LsiNotStable,
    /// A prescaler or reload update was not acknowledged within its spin budget.
    Timeout,
}

impl From<ClockError> for WatchdogError {
    fn from(_: ClockError) -> Self {
        WatchdogError::LsiNotStable
    }
}

/// A PWM error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum PwmError {
    DutyOutOfRange,
    BadTimerBase,
    BadPin,
    /// [`crate::timer::PwmTimer::follow`]: the trigger matrix gives the slave no input that carries
    /// the master's TRGO.
    NoInternalTrigger,
    Other,
}

/// A flash-controller error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FmcError {
    /// An address outside the flash extent, or misaligned for the operation.
    BadArg,
    /// A program over a half-word that is not erased.
    NotErased,
    /// The span's last half-word is the high half of a 32-bit word whose successor in the same
    /// page is already programmed, which closes it until the page is erased: the controller would
    /// refuse it with `PGERR` while it reads erased. The page was programmed out of ascending order.
    Closed,
    /// The page is write-protected (`WPERR`).
    WriteProtect,
    /// The controller reported a program error (`PGERR`).
    ProgramError,
    /// The controller stayed busy past its spin budget.
    Timeout,
}

/// A combined bring-up error (the PWM and ADC bring-ups).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum BringUpError {
    Pwm(PwmError),
    Adc(AdcError),
    Descriptor(DescriptorError),
}

impl From<PwmError> for BringUpError {
    fn from(e: PwmError) -> Self {
        BringUpError::Pwm(e)
    }
}

impl From<AdcError> for BringUpError {
    fn from(e: AdcError) -> Self {
        BringUpError::Adc(e)
    }
}

impl From<DescriptorError> for BringUpError {
    fn from(e: DescriptorError) -> Self {
        BringUpError::Descriptor(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn i2c_error_codes_are_fixed_distinct_and_high_bit() {
        let all = [
            (I2cError::Bus, 0x81),
            (I2cError::ArbitrationLoss, 0x82),
            (I2cError::NoAcknowledgeAddress, 0x83),
            (I2cError::NoAcknowledgeData, 0x84),
            (I2cError::Overrun, 0x85),
            (I2cError::Timeout, 0x86),
            (I2cError::Other, 0x87),
            (I2cError::BusHeld, 0x88),
        ];
        for (i, (e, want)) in all.iter().enumerate() {
            assert_eq!(e.code(), *want, "{e:?}");
            assert_ne!(e.code() & 0x80, 0);
            for (other, _) in &all[i + 1..] {
                assert_ne!(e.code(), other.code());
            }
        }
    }

    #[test]
    fn descriptor_error_codes_keep_their_numbers_and_are_distinct() {
        let all = [
            (DescriptorError::MissingBase(PeriphLabel::Usart0), 3),
            (DescriptorError::UsartConfig, 4),
            (DescriptorError::Unsupported, 7),
            (DescriptorError::SelfCheckFailed, 8),
        ];
        for (i, (e, want)) in all.iter().enumerate() {
            assert_eq!(e.code(), *want, "{e:?}");
            for (other, _) in &all[i + 1..] {
                assert_ne!(e.code(), other.code());
            }
        }
    }
}
