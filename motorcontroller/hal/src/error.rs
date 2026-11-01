// SPDX-License-Identifier: GPL-3.0-or-later
//! Error types.

use crate::addr::PeriphLabel;

/// A bring-up refused for its wiring: the request names something the chip does not carry, a pin
/// another function owns, or a role the pin cannot take.
///
/// The firmware records these in its boot report through [`DescriptorError::code`]. The codes are
/// the numbering `runtime-hal` used for the variants that remain, so a boot report reads the same
/// across the move; the two that described a mismatched runtime selector (codes 1 and 2) went with
/// the selectors (specs/hal.md, section 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DescriptorError {
    /// The chip has no base address for this peripheral.
    MissingBase(PeriphLabel),
    /// The serial configuration cannot be expressed (a baud the divider cannot reach).
    UsartConfig,
    /// The pin cannot take the requested role.
    UnsupportedRole,
    /// The pin belongs to another function (a gate pin for anything but a gate, or the reverse).
    PinOwnership,
    /// The capability exists in the request but not in the HAL.
    Unsupported,
    /// A bring-up's read-back did not confirm its configuration.
    SelfCheckFailed,
    /// The pin is PA13 or PA14, the SWD pins, which nothing may claim.
    SwdPin,
    /// The instance is already in use by another bring-up.
    InUse,
}

impl DescriptorError {
    /// The boot report's code for the error.
    pub const fn code(&self) -> u8 {
        match self {
            DescriptorError::MissingBase(_) => 3,
            DescriptorError::UsartConfig => 4,
            DescriptorError::UnsupportedRole => 5,
            DescriptorError::PinOwnership => 6,
            DescriptorError::Unsupported => 7,
            DescriptorError::SelfCheckFailed => 8,
            DescriptorError::SwdPin => 9,
            DescriptorError::InUse => 10,
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
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            I2cError::Bus => ErrorKind::Bus,
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
    Other,
}

/// A flash-controller error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FmcError {
    /// An address outside the flash extent, or misaligned for the operation.
    BadArg,
    /// A program over a half-word that is not erased.
    NotErased,
    /// The controller did not unlock.
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
    fn descriptor_error_codes_keep_their_numbers_and_are_distinct() {
        let all = [
            (DescriptorError::MissingBase(PeriphLabel::Usart0), 3),
            (DescriptorError::UsartConfig, 4),
            (DescriptorError::UnsupportedRole, 5),
            (DescriptorError::PinOwnership, 6),
            (DescriptorError::Unsupported, 7),
            (DescriptorError::SelfCheckFailed, 8),
            (DescriptorError::SwdPin, 9),
            (DescriptorError::InUse, 10),
        ];
        for (i, (e, want)) in all.iter().enumerate() {
            assert_eq!(e.code(), *want, "{e:?}");
            for (other, _) in &all[i + 1..] {
                assert_ne!(e.code(), other.code());
            }
        }
    }
}
