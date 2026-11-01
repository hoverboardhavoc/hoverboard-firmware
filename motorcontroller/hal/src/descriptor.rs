// SPDX-License-Identifier: GPL-3.0-or-later
//! The chip descriptor: the facts of one exact part, as a `const`.
//!
//! A descriptor holds only chip facts (specs/hal.md, section 3.1): the base-address table, the flash
//! and page size, the RAM size, the advanced-timer and ADC instance counts, the pin bonding, the
//! interrupt numbers the HAL's drivers use, and the two identity bytes the firmware reports. Each
//! value is per exact part and is to be confirmed on silicon by the probe validator (section 8.4).
//!
//! Exactly one descriptor is compiled: [`DESCRIPTOR`], the chip the enabled feature names.

use crate::addr::{AddrTable, PeriphLabel};
use crate::irq::Interrupt;

// FLASH_KIB, FLASH_PAGE, RAM_KIB: the chip's memory sizes, owned by build.rs's chip table (which
// also hands them to the images' linker scripts).
include!(concat!(env!("OUT_DIR"), "/memory.rs"));

/// The flash page size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PageSize {
    /// 1 KiB pages (the 64 KiB GD32F103C8 and GD32F130C8).
    K1,
    /// 2 KiB pages (the GD32F10x parts above 128 KiB, the 256 KiB GD32F103RC).
    K2,
}

impl PageSize {
    /// The page size in bytes.
    #[inline]
    pub const fn bytes(self) -> u32 {
        match self {
            PageSize::K1 => 1024,
            PageSize::K2 => 2048,
        }
    }
}

/// The interrupts a USART's drivers use: its own vector (the receive IDLE and line errors) and the
/// vectors of the DMA channels that carry its receive and transmit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsartIrqs {
    /// The USART's own vector.
    pub rx: Interrupt,
    /// The receive DMA channel's vector.
    pub dma_rx: Interrupt,
    /// The transmit DMA channel's vector (the same vector as `dma_rx` where the vendor groups the
    /// two channels).
    pub dma_tx: Interrupt,
}

/// The interrupt numbers the HAL's drivers and the firmware's motor ISR use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IrqMap {
    /// The ADC vector, which carries the injected end of conversion: the motor period ISR.
    pub period: Interrupt,
    /// Per USART instance (`Usart0`, `Usart1`, `Usart2`), its vectors, or `None` where the chip
    /// does not carry the instance.
    pub usart: [Option<UsartIrqs>; 3],
}

/// The facts of one part.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuDescriptor {
    /// The part name, as GigaDevice marks it.
    pub name: &'static str,
    /// Peripheral base addresses; an instance the part lacks has none.
    pub addrs: AddrTable,
    /// Flash size in KiB.
    pub flash_kib: u16,
    /// Flash page size.
    pub flash_page: PageSize,
    /// SRAM size in KiB.
    pub ram_kib: u16,
    /// Advanced timers carried (TIMER0, and TIMER7 on the high-density F10x).
    pub adv_timers: u8,
    /// ADC instances carried.
    pub adc_count: u8,
    /// Bonded GPIO pins per port, as a bit mask per port index (A = 0, B = 1, C = 2, D = 3,
    /// E = 4, F = 5): bit `n` set means pin `n` is bonded out in this part's package.
    pub bonding: [u16; 6],
    /// The interrupt numbers the drivers use.
    pub irqs: IrqMap,
    /// The `NODE_HELLO` mcu byte for this part (`net::walk::MCU_F10X` = 1 for the GD32F10x,
    /// `MCU_F1X0` = 2 for the GD32F1x0).
    pub node_mcu: u8,
    /// The cyclic payload's chip tag (`linkctl::ChipTag`: 1 = GD32F103C8, 2 = GD32F130C8,
    /// 3 = GD32F103RC).
    pub chip_tag: u8,
}

impl McuDescriptor {
    /// Flash size in bytes.
    #[inline]
    pub const fn flash_size_bytes(&self) -> u32 {
        self.flash_kib as u32 * 1024
    }

    /// Whether packed pin `pin` (`(port << 4) | n`) is bonded out on this part.
    #[inline]
    pub const fn bonded(&self, pin: u8) -> bool {
        let port = (pin >> 4) as usize;
        port < self.bonding.len() && self.bonding[port] & (1 << (pin & 0x0F)) != 0
    }
}

// --- base addresses, from the GigaDevice user manuals' memory maps and the SPL headers ----------

/// The peripherals at the same base on every GD32 this HAL drives.
const COMMON: AddrTable = AddrTable::new()
    // USART0 (ST USART1) on APB2, USART1 on APB1.
    .with(PeriphLabel::Usart0, 0x4001_3800)
    .with(PeriphLabel::Usart1, 0x4000_4400)
    .with(PeriphLabel::Rcu, 0x4002_1000)
    .with(PeriphLabel::I2c0, 0x4000_5400)
    .with(PeriphLabel::Adc0, 0x4001_2400)
    .with(PeriphLabel::Timer0, 0x4001_2C00)
    .with(PeriphLabel::Fwdgt, 0x4000_3000)
    .with(PeriphLabel::Timer1, 0x4000_0000)
    // SPI0 on APB2 on both: the F1x0 SPL puts it at `SPI_BASE + 0xF800` = 0x4001_3000
    // (gd32f1x0_spi.h); 0x4000_3800 is SPI1 there.
    .with(PeriphLabel::Spi0, 0x4001_3000);

/// The GD32F10x bases: GPIO on APB2 (0x400 stride, port E not carried), USART2, the second I2C, and the factory unique ID at `0x1FFF_F7E8` (GD32F10x User Manual 1.5.2).
#[cfg(any(feature = "gd32f103c8", feature = "gd32f103rc"))]
const F10X: AddrTable = COMMON
    .with(PeriphLabel::Usart2, 0x4000_4800)
    .with(PeriphLabel::Gpioa, 0x4001_0800)
    .with(PeriphLabel::Gpiob, 0x4001_0C00)
    .with(PeriphLabel::Gpioc, 0x4001_1000)
    .with(PeriphLabel::Gpiod, 0x4001_1400)
    .with(PeriphLabel::Gpiof, 0x4001_1C00)
    .with(PeriphLabel::I2c1, 0x4000_5800)
    .with(PeriphLabel::UniqueId, 0x1FFF_F7E8);

/// GD32F103C8: medium density, 64 KiB flash, 20 KiB SRAM, LQFP48. One advanced timer (TIMER0) and
/// two ADCs, as `runtime-hal` measured on the bench F103 (its README); PA, PB, PC13-15 and PD0-1
/// bonded (GD32F103xx datasheet, LQFP48 pinout).
#[cfg(feature = "gd32f103c8")]
pub const DESCRIPTOR: McuDescriptor = McuDescriptor {
    name: "GD32F103C8",
    addrs: F10X.with(PeriphLabel::Adc1, 0x4001_2800),
    flash_kib: FLASH_KIB,
    flash_page: FLASH_PAGE,
    ram_kib: RAM_KIB,
    adv_timers: 1,
    adc_count: 2,
    bonding: [0xFFFF, 0xFFFF, 0xE000, 0x0003, 0, 0],
    irqs: F10X_IRQS,
    node_mcu: 1,
    chip_tag: 1,
};

/// GD32F103RC: high density, 256 KiB flash in 2 KiB pages, 48 KiB SRAM, LQFP64 (the 12-FET
/// mainboard). Two advanced timers (TIMER0, TIMER7) and three ADCs, as `runtime-hal`'s probe
/// measured on the 12-FET; PA, PB, PC and PD0-2 bonded.
#[cfg(feature = "gd32f103rc")]
pub const DESCRIPTOR: McuDescriptor = McuDescriptor {
    name: "GD32F103RC",
    addrs: F10X
        .with(PeriphLabel::Adc1, 0x4001_2800)
        // ADC2 = ADC_BASE + 0x1800 (gd32f10x_adc.h), past the APB2 timer/USART block.
        .with(PeriphLabel::Adc2, 0x4001_3C00)
        .with(PeriphLabel::Timer7, 0x4001_3400),
    flash_kib: FLASH_KIB,
    flash_page: FLASH_PAGE,
    ram_kib: RAM_KIB,
    adv_timers: 2,
    adc_count: 3,
    bonding: [0xFFFF, 0xFFFF, 0xFFFF, 0x0007, 0, 0],
    irqs: F10X_IRQS,
    node_mcu: 1,
    chip_tag: 3,
};

/// GD32F130C8: 64 KiB flash, 8 KiB SRAM, LQFP48. GPIO on AHB (0x400 stride, no port E), one
/// advanced timer and one ADC, the second I2C (the x8 parts carry
/// it, GD32F130xx datasheet table 2-10 note 3), and the factory unique ID at `0x1FFF_F7AC`
/// (GD32F1x0 User Manual 1.6.2). PA, PB, PC13-15, PF0-1 and PF6-7 bonded.
#[cfg(feature = "gd32f130c8")]
pub const DESCRIPTOR: McuDescriptor = McuDescriptor {
    name: "GD32F130C8",
    addrs: COMMON
        .with(PeriphLabel::Gpioa, 0x4800_0000)
        .with(PeriphLabel::Gpiob, 0x4800_0400)
        .with(PeriphLabel::Gpioc, 0x4800_0800)
        .with(PeriphLabel::Gpiod, 0x4800_0C00)
        .with(PeriphLabel::Gpiof, 0x4800_1400)
        .with(PeriphLabel::I2c1, 0x4000_5800)
        .with(PeriphLabel::UniqueId, 0x1FFF_F7AC),
    flash_kib: FLASH_KIB,
    flash_page: FLASH_PAGE,
    ram_kib: RAM_KIB,
    adv_timers: 1,
    adc_count: 1,
    bonding: [0xFFFF, 0xFFFF, 0xE000, 0, 0, 0x00C3],
    irqs: IrqMap {
        period: Interrupt::ADC_CMP,
        usart: [
            // USART0: DMA channel 2 receive and channel 1 transmit share the grouped Ch1/2 vector.
            Some(UsartIrqs {
                rx: Interrupt::USART0,
                dma_rx: Interrupt::DMA_Channel1_2,
                dma_tx: Interrupt::DMA_Channel1_2,
            }),
            // USART1: DMA channel 4 receive and channel 3 transmit share the grouped Ch3/4 vector.
            Some(UsartIrqs {
                rx: Interrupt::USART1,
                dma_rx: Interrupt::DMA_Channel3_4,
                dma_tx: Interrupt::DMA_Channel3_4,
            }),
            None,
        ],
    },
    node_mcu: 2,
    chip_tag: 2,
};

/// The GD32F10x driver interrupts (the same numbers at medium and high density).
#[cfg(any(feature = "gd32f103c8", feature = "gd32f103rc"))]
const F10X_IRQS: IrqMap = IrqMap {
    period: Interrupt::ADC0_1,
    usart: [
        // USART0: DMA0 channel 4 receive, channel 3 transmit.
        Some(UsartIrqs {
            rx: Interrupt::USART0,
            dma_rx: Interrupt::DMA0_Channel4,
            dma_tx: Interrupt::DMA0_Channel3,
        }),
        // USART1: DMA0 channel 5 receive, channel 6 transmit.
        Some(UsartIrqs {
            rx: Interrupt::USART1,
            dma_rx: Interrupt::DMA0_Channel5,
            dma_tx: Interrupt::DMA0_Channel6,
        }),
        // USART2: DMA0 channel 2 receive, channel 1 transmit.
        Some(UsartIrqs {
            rx: Interrupt::USART2,
            dma_rx: Interrupt::DMA0_Channel2,
            dma_tx: Interrupt::DMA0_Channel1,
        }),
    ],
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn page_size_follows_the_flash_size() {
        // The GD32F10x parts above 128 KiB erase in 2 KiB pages; every other part in 1 KiB.
        let want = if DESCRIPTOR.flash_kib > 128 {
            PageSize::K2
        } else {
            PageSize::K1
        };
        assert_eq!(DESCRIPTOR.flash_page, want);
    }

    #[test]
    fn instance_counts_agree_with_the_base_table() {
        let has = |l| DESCRIPTOR.addrs.get(l).is_some();
        let adcs = [PeriphLabel::Adc0, PeriphLabel::Adc1, PeriphLabel::Adc2]
            .into_iter()
            .filter(|&l| has(l))
            .count();
        assert_eq!(adcs, DESCRIPTOR.adc_count as usize);
        let timers = [PeriphLabel::Timer0, PeriphLabel::Timer7]
            .into_iter()
            .filter(|&l| has(l))
            .count();
        assert_eq!(timers, DESCRIPTOR.adv_timers as usize);
    }

    #[test]
    fn swd_pins_and_the_motor_pins_are_bonded() {
        // PA13/PA14 (SWD) and the TIMER0 gate set PA8-10 / PB13-15 exist on every part.
        for pin in [0x0D, 0x0E, 0x08, 0x09, 0x0A, 0x1D, 0x1E, 0x1F] {
            assert!(DESCRIPTOR.bonded(pin), "{pin:#04x}");
        }
        // Port E is never bonded, nor an out-of-range port byte.
        assert!(!DESCRIPTOR.bonded(0x40));
        assert!(!DESCRIPTOR.bonded(0x60));
    }

    #[test]
    fn bonded_gpio_counts_match_the_datasheets() {
        let n: u32 = DESCRIPTOR.bonding.iter().map(|m| m.count_ones()).sum();
        let want = match DESCRIPTOR.name {
            "GD32F103C8" => 37,
            "GD32F130C8" => 39,
            "GD32F103RC" => 51,
            other => panic!("no datasheet count for {other}"),
        };
        assert_eq!(n, want);
    }

    #[test]
    fn identity_bytes_name_the_part() {
        let (mcu, tag) = match DESCRIPTOR.name {
            "GD32F103C8" => (1, 1),
            "GD32F130C8" => (2, 2),
            "GD32F103RC" => (1, 3),
            other => panic!("no identity for {other}"),
        };
        assert_eq!((DESCRIPTOR.node_mcu, DESCRIPTOR.chip_tag), (mcu, tag));
    }

    #[test]
    fn usart_irqs_exist_exactly_for_the_carried_usarts() {
        for (i, label) in [
            PeriphLabel::Usart0,
            PeriphLabel::Usart1,
            PeriphLabel::Usart2,
        ]
        .into_iter()
        .enumerate()
        {
            assert_eq!(
                DESCRIPTOR.irqs.usart[i].is_some(),
                DESCRIPTOR.addrs.get(label).is_some(),
                "{label:?}"
            );
        }
    }
}
