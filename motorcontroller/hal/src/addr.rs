// SPDX-License-Identifier: GPL-3.0-or-later
//! Peripheral labels and the base-address table a descriptor carries.

use crate::error::DescriptorError;

/// A peripheral instance, named as GigaDevice numbers them (`Usart0` is ST's USART1).
///
/// The same set on every chip; a chip that lacks an instance has no base for it in its descriptor
/// ([`AddrTable::resolve`] answers `MissingBase`), and the bring-up that would use it refuses.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum PeriphLabel {
    Usart0 = 0,
    Usart1 = 1,
    Usart2 = 2,
    Gpioa = 3,
    Gpiob = 4,
    Gpioc = 5,
    Gpiod = 6,
    Gpiof = 7,
    Rcu = 8,
    I2c0 = 9,
    I2c1 = 10,
    Spi0 = 11,
    Adc0 = 12,
    Adc1 = 13,
    Adc2 = 14,
    Timer0 = 15,
    Timer7 = 16,
    Fwdgt = 17,
    Timer1 = 18,
    UniqueId = 19,
    /// The DMA controller (DMA0 on the F10x, the one DMA on the F1x0).
    Dma0 = 20,
}

impl PeriphLabel {
    /// How many labels there are (the table length).
    pub const COUNT: usize = 21;

    /// The label's table index.
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// A GPIO port.
    #[inline]
    pub const fn is_gpio(self) -> bool {
        matches!(
            self,
            PeriphLabel::Gpioa
                | PeriphLabel::Gpiob
                | PeriphLabel::Gpioc
                | PeriphLabel::Gpiod
                | PeriphLabel::Gpiof
        )
    }

    /// An I2C instance.
    #[inline]
    pub const fn is_i2c(self) -> bool {
        matches!(self, PeriphLabel::I2c0 | PeriphLabel::I2c1)
    }

    /// An ADC instance.
    #[inline]
    pub const fn is_adc(self) -> bool {
        matches!(
            self,
            PeriphLabel::Adc0 | PeriphLabel::Adc1 | PeriphLabel::Adc2
        )
    }

    /// An advanced timer (TIMER0, TIMER7).
    #[inline]
    pub const fn is_timer(self) -> bool {
        matches!(self, PeriphLabel::Timer0 | PeriphLabel::Timer7)
    }

    /// A USART instance.
    #[inline]
    pub const fn is_usart(self) -> bool {
        matches!(
            self,
            PeriphLabel::Usart0 | PeriphLabel::Usart1 | PeriphLabel::Usart2
        )
    }
}

/// The base address of each peripheral instance the chip carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AddrTable {
    bases: [Option<u32>; PeriphLabel::COUNT],
}

impl AddrTable {
    /// An empty table.
    pub const fn new() -> Self {
        Self {
            bases: [None; PeriphLabel::COUNT],
        }
    }

    /// The table with `label` at `base` (a const builder for the descriptors).
    #[must_use]
    pub const fn with(mut self, label: PeriphLabel, base: u32) -> Self {
        self.bases[label.index()] = Some(base);
        self
    }

    /// The base of `label`, or `None` when the chip does not carry it.
    #[inline]
    pub const fn get(&self, label: PeriphLabel) -> Option<u32> {
        self.bases[label.index()]
    }

    /// The base of `label`, or [`DescriptorError::MissingBase`].
    #[inline]
    pub const fn resolve(&self, label: PeriphLabel) -> Result<u32, DescriptorError> {
        match self.bases[label.index()] {
            Some(b) => Ok(b),
            None => Err(DescriptorError::MissingBase(label)),
        }
    }
}

impl Default for AddrTable {
    fn default() -> Self {
        Self::new()
    }
}
