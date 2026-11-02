// SPDX-License-Identifier: GPL-3.0-or-later
//! The MM32SPIN05 RCC model (MindMotion's `reg_rcc.h` and `system_mm32spin0x_q.c`, LibSamples
//! V1.13): the enable-bit map, the flash wait states and the tree. The ADC clock divider is the
//! ADC's own (`ADCFG.PRE`, `adc/mm32.rs`).
//!
//! The part has no PLL. Its system clock is the HSI's 72 MHz output (`CR.HSI_72M`, bit 20) switched
//! in by `CFGR.SW = 0b10`, and the vendor trims the HSI from a factory word at `0x1FFF_F7FA` on the
//! "q" die before it switches. [`configure_tree`] is MindMotion's `SetSysClockTo72_HSI` for that
//! die, with the bus dividers taken from the [`ClockConfig`]. The `AHBENR`, `APB2ENR` and
//! `APB1ENR` offsets and every enable bit this HAL sets are the GD32F1x0 RCU's (`AHBENR.GPIOA` 17
//! to `GPIOD` 20, `APB2ENR.ADC1` 9, `TIM1` 11, `UART1` 14, `APB1ENR.TIM2` 0, `UART2` 17, `I2C1`
//! 21, `AHBENR.DMA1` 0); the part has no port F.

use super::{ahb_psc_code, apb_psc_code, EnableBit, AHBEN, APB2EN, CFG0};
use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::descriptor::DESCRIPTOR;
use crate::error::ClockError;
use crate::reg::{Reg16, Reg32};

/// The highest system clock the part runs, and the only one this model builds: the HSI's 72 MHz
/// output.
pub(crate) const MAX_SYSCLK_HZ: u32 = 72_000_000;

/// Flash wait states (`FLASH_ACR.LATENCY`): none at the 8 MHz reset clock, two at 72 MHz, as the
/// vendor's `SetSysClockTo72_HSI` programs them.
pub(crate) const fn wait_states(sysclk_hz: u32) -> u32 {
    if sysclk_hz <= 8_000_000 {
        0
    } else {
        2
    }
}

/// The GPIO port enable: `AHBENR.GPIOx`, port A at bit 17, ports A to D.
pub(crate) const fn gpio_enable(port: PeriphLabel) -> Option<EnableBit> {
    let bit = match port {
        PeriphLabel::Gpioa => 17,
        PeriphLabel::Gpiob => 18,
        PeriphLabel::Gpioc => 19,
        PeriphLabel::Gpiod => 20,
        _ => return None,
    };
    Some(EnableBit { reg: AHBEN, bit })
}

/// The UART enable: UART1 (`Usart0`) on APB2 bit 14, UART2 (`Usart1`) on APB1 bit 17.
pub(crate) const fn usart_enable(usart: PeriphLabel) -> Option<EnableBit> {
    match usart {
        PeriphLabel::Usart0 => Some(EnableBit {
            reg: APB2EN,
            bit: 14,
        }),
        PeriphLabel::Usart1 => Some(EnableBit {
            reg: super::APB1EN,
            bit: 17,
        }),
        _ => None,
    }
}

/// The advanced-timer enable: TIM1 (`Timer0`) at APB2 bit 11.
pub(crate) const fn timer_enable(timer: PeriphLabel) -> Option<EnableBit> {
    match timer {
        PeriphLabel::Timer0 => Some(EnableBit {
            reg: APB2EN,
            bit: 11,
        }),
        _ => None,
    }
}

/// A clock tree: the HSI's 72 MHz and the bus prescalers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    /// The system clock, Hz: 72 MHz, the HSI's high output, the one tree this model builds.
    pub sysclk_hz: u32,
    /// The AHB divider (1, 2, 4, ..., 512, without 32).
    pub ahb_psc: u16,
    /// The APB1 divider (1, 2, 4, 8, 16).
    pub apb1_psc: u16,
    /// The APB2 divider (1, 2, 4, 8, 16).
    pub apb2_psc: u16,
}

impl ClockConfig {
    /// The tree MindMotion's `SetSysClockTo72_HSI` leaves: HSI 72 MHz, AHB, APB1 and APB2 all
    /// / 1. The vendor function first writes APB1 / 2, then walks `HPRE` through an `unsigned
    /// char`, whose write-back clears every `CFGR` bit above 7, `PPRE1` with them, so the part ends
    /// with APB1 at 72 MHz (the golden trace `clock/tree_72m_hsi_mm32`). AILIFE's MM32 firmware
    /// runs its I2C on that 72 MHz PCLK1. The GD32 reference tree runs APB1 at 36 MHz.
    pub const REFERENCE_72M_HSI: ClockConfig = ClockConfig {
        sysclk_hz: 72_000_000,
        ahb_psc: 1,
        apb1_psc: 1,
        apb2_psc: 1,
    };

    /// Check the tree: the 72 MHz system clock (`InvalidPll`: the part has no PLL, and this model
    /// builds no other source) and the prescaler values.
    pub fn validate(&self) -> Result<(), ClockError> {
        if self.sysclk_hz != MAX_SYSCLK_HZ {
            return Err(ClockError::InvalidPll);
        }
        if ahb_psc_code(self.ahb_psc).is_none()
            || apb_psc_code(self.apb1_psc).is_none()
            || apb_psc_code(self.apb2_psc).is_none()
        {
            return Err(ClockError::InvalidPrescaler);
        }
        Ok(())
    }

    /// The APB1 clock, Hz.
    #[inline]
    pub const fn apb1_hz(&self) -> u32 {
        self.sysclk_hz / self.ahb_psc as u32 / self.apb1_psc as u32
    }

    /// The APB2 clock, Hz.
    #[inline]
    pub const fn apb2_hz(&self) -> u32 {
        self.sysclk_hz / self.ahb_psc as u32 / self.apb2_psc as u32
    }
}

const CR: u32 = 0x00;
const CR_HSION: u32 = 1 << 0;
const CR_HSIRDY: u32 = 1 << 1;
/// `CR.HSITEN` (bit 2), which the vendor clears before the switch.
const CR_HSITEN: u32 = 1 << 2;
/// `CR.HSI_72M`: the HSI's 72 MHz output.
const CR_HSI_72M: u32 = 1 << 20;
const CFGR_SW: u32 = 0b11;
const CFGR_SW_HSI: u32 = 0b10;
const CFGR_SWS: u32 = 0b11 << 2;
const CFGR_SWS_HSI: u32 = 0b10 << 2;
const CFGR_HPRE: u32 = 0b1111 << 4;
const CFGR_PPRE1: u32 = 0b111 << 8;
const CFGR_PPRE2: u32 = 0b111 << 11;
/// `HPRE` /4 and /2, the steps the vendor walks the AHB clock through around the switch.
const HPRE_DIV4: u32 = 0b1001 << 4;
const HPRE_DIV2: u32 = 0b1000 << 4;

/// `FLASH_ACR` (the FMC's `WS` offset): the prefetch buffer `PRFTBE` (bit 4) beside `LATENCY`
/// (2:0).
const ACR_PRFTBE: u32 = 1 << 4;

/// The factory HSI calibration half-word of the "q" die (`SetSysClockTo72_HSI`).
const HSI_TRIM: u32 = 0x1FFF_F7FA;
/// The low byte of `CR` the vendor writes with the calibration: `0xF8`.
const CR_TRIM_LOW: u32 = 0xF8;

/// Program the tree, in MindMotion's `SetSysClockTo72_HSI` order for the "q" die: HSI on and its
/// ready poll; the 72 MHz output selected and `HSITEN` cleared; `CFGR` written with the APB1
/// divider; the flash wait states with the prefetch buffer; AHB at /4 and the APB2 divider; the
/// switch field cleared; the factory calibration into `CR` (bits 15:8, and `0xF8` in the low byte);
/// the switch to the HSI with its status poll; then AHB to /2 and to the configured divider. The
/// vendor's `HPRE` steps clear the APB dividers as a side effect (see
/// [`ClockConfig::REFERENCE_72M_HSI`]); these write each field, so the dividers end as configured.
///
/// The vendor reads `SCB_CPUID` and the device ID before the calibration to tell the "q" die from
/// its others; this build is for that die, so it does not. Each poll spins until its flag is up.
pub fn configure_tree(_chip: &Chip, clock: &ClockConfig) -> Result<(), ClockError> {
    clock.validate()?;
    let rcc = DESCRIPTOR
        .addrs
        .get(PeriphLabel::Rcu)
        .ok_or(ClockError::SourceNotStable)?;
    let cr = Reg32::new(rcc, CR);
    let cfgr = Reg32::new(rcc, CFG0);
    // validate() checked every code exists.
    let code = |c: Option<u32>| c.unwrap_or(0);

    cr.set_bits(CR_HSION);
    while cr.read() & CR_HSIRDY == 0 {}
    cr.set_bits(CR_HSI_72M);
    cr.clear_bits(CR_HSITEN);
    cfgr.write(code(apb_psc_code(clock.apb1_psc)) << 8);
    Reg32::new(crate::fmc::FMC_BASE, crate::fmc::WS).write(wait_states(clock.sysclk_hz) | ACR_PRFTBE);
    cfgr.set_bits(HPRE_DIV4);
    cfgr.modify(CFGR_PPRE2, code(apb_psc_code(clock.apb2_psc)) << 11);
    cfgr.modify(CFGR_PPRE1, code(apb_psc_code(clock.apb1_psc)) << 8);
    cfgr.clear_bits(CFGR_SW);

    // The calibration: the low byte of the factory half-word if it is programmed, else its high
    // byte, into CR bits 15:8.
    let factory = u32::from(Reg16::new(HSI_TRIM, 0).read());
    let trim = if factory & 0xFF == 0xFF {
        factory & 0xFF00
    } else {
        (factory & 0xFF) << 8
    };
    cr.write(trim | (cr.read() & 0xFFFF_00FF) | CR_TRIM_LOW);

    cfgr.set_bits(CFGR_SW_HSI);
    while cfgr.read() & CFGR_SWS != CFGR_SWS_HSI {}
    cfgr.modify(CFGR_HPRE, HPRE_DIV2);
    cfgr.modify(CFGR_HPRE, code(ahb_psc_code(clock.ahb_psc)) << 4);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_hsi_tree_validates_and_runs_both_buses_at_72() {
        let c = ClockConfig::REFERENCE_72M_HSI;
        assert_eq!(c.validate(), Ok(()));
        assert_eq!((c.apb2_hz(), c.apb1_hz()), (72_000_000, 72_000_000));
    }

    #[test]
    fn only_the_72_mhz_tree_and_valid_prescalers_are_accepted() {
        let base = ClockConfig::REFERENCE_72M_HSI;
        let slow = ClockConfig {
            sysclk_hz: 48_000_000,
            ..base
        };
        assert_eq!(slow.validate(), Err(ClockError::InvalidPll));
        let apb = ClockConfig {
            apb1_psc: 3,
            ..base
        };
        assert_eq!(apb.validate(), Err(ClockError::InvalidPrescaler));
    }

    #[test]
    fn the_part_has_no_port_f() {
        assert!(gpio_enable(PeriphLabel::Gpiof).is_none());
        assert!(gpio_enable(PeriphLabel::Gpiod).is_some());
    }
}
