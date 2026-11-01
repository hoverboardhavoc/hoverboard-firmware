// SPDX-License-Identifier: GPL-3.0-or-later
//! The clock tree and the peripheral clock enables.
//!
//! The tree (`CTL`, `CFG0`, the PLL) has the same layout on the F10x RCC and the F1x0 RCU; what
//! differs is the model's enable-bit map, the flash wait states, the top system clock and the ADC
//! clock select, which live in `clock/f10x.rs` and `clock/f1x0.rs`.

#[cfg(hal_clock = "f10x")]
#[path = "clock/f10x.rs"]
mod model;
#[cfg(hal_clock = "f1x0")]
#[path = "clock/f1x0.rs"]
mod model;

use crate::addr::PeriphLabel;
use crate::chip::{Chip, ClockPath};
use crate::error::{ClockError, DescriptorError};
use crate::reg::Reg32;

const CTL: u32 = 0x00;
const CFG0: u32 = 0x04;
const AHBEN: u32 = 0x14;
const APB2EN: u32 = 0x18;
const APB1EN: u32 = 0x1C;
const RSTSCK: u32 = 0x24;

const CFG0_ADCPSC: u32 = 0b11 << 14;
const ADCPSC_DIV6: u32 = 2 << 14;

/// The system clock source the PLL runs from.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSource {
    /// The internal 8 MHz RC oscillator, halved into the PLL.
    Irc8m = 0,
    /// The external crystal.
    Hxtal = 1,
}

/// A clock tree: the system clock, the PLL source and multiplier, and the bus prescalers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    /// The resulting system clock, Hz (what every delay and divider converts against).
    pub sysclk_hz: u32,
    /// The PLL source.
    pub source: ClockSource,
    /// The PLL multiplier, 2..=32.
    pub pll_mul: u8,
    /// The AHB divider (1, 2, 4, ..., 512, without 32).
    pub ahb_psc: u16,
    /// The APB1 divider (1, 2, 4, 8, 16).
    pub apb1_psc: u16,
    /// The APB2 divider (1, 2, 4, 8, 16).
    pub apb2_psc: u16,
}

impl ClockConfig {
    /// The fleet's tree: IRC8M / 2 x 18 = 72 MHz, AHB / 1, APB1 / 2 (36 MHz), APB2 / 1.
    pub const REFERENCE_72M_IRC8M: ClockConfig = ClockConfig {
        sysclk_hz: 72_000_000,
        source: ClockSource::Irc8m,
        pll_mul: 18,
        ahb_psc: 1,
        apb1_psc: 2,
        apb2_psc: 1,
    };

    /// Check the tree against the compiled chip: the PLL multiplier, the prescaler values and the
    /// part's top system clock.
    pub fn validate(&self) -> Result<(), ClockError> {
        if !(2..=32).contains(&self.pll_mul) {
            return Err(ClockError::InvalidPll);
        }
        if ahb_psc_code(self.ahb_psc).is_none()
            || apb_psc_code(self.apb1_psc).is_none()
            || apb_psc_code(self.apb2_psc).is_none()
        {
            return Err(ClockError::InvalidPrescaler);
        }
        if self.sysclk_hz > model::MAX_SYSCLK_HZ {
            return Err(ClockError::InvalidWaitStates);
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

/// The `CFG0.AHBPSC` code for an AHB divider.
const fn ahb_psc_code(div: u16) -> Option<u32> {
    Some(match div {
        1 => 0,
        2 => 8,
        4 => 9,
        8 => 10,
        16 => 11,
        64 => 12,
        128 => 13,
        256 => 14,
        512 => 15,
        _ => return None,
    })
}

/// The `CFG0.APBxPSC` code for an APB divider.
const fn apb_psc_code(div: u16) -> Option<u32> {
    Some(match div {
        1 => 0,
        2 => 4,
        4 => 5,
        8 => 6,
        16 => 7,
        _ => return None,
    })
}

/// The `CFG0.PLLMF` bits for a multiplier: bits 21:18 hold `mul - 2` for 2..=16, and bit 27 extends
/// the field for 17..=32 (`mul - 17` in 21:18).
const fn pll_mul_bits(mul: u8) -> u32 {
    let m = mul as u32;
    if m <= 16 {
        (m - 2) << 18
    } else {
        (1 << 27) | ((m - 17) << 18)
    }
}

/// A peripheral clock-enable bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct EnableBit {
    pub reg: u32,
    pub bit: u8,
}

impl EnableBit {
    #[inline]
    fn apply(self, rcu_base: u32) {
        Reg32::new(rcu_base, self.reg).set_bits(1 << self.bit);
    }
}

/// Set `label`'s enable bit, refusing an instance the chip does not carry (the model's register
/// may have the bit, as the F10x RCC has TIMER7's, while the part lacks the instance).
#[inline]
fn enable(
    rcu_base: u32,
    bit: Option<EnableBit>,
    label: PeriphLabel,
) -> Result<(), DescriptorError> {
    crate::descriptor::DESCRIPTOR.addrs.resolve(label)?;
    bit.ok_or(DescriptorError::MissingBase(label))?
        .apply(rcu_base);
    Ok(())
}

/// Enable a GPIO port's clock.
pub fn enable_gpio_port(
    rcu_base: u32,
    _path: ClockPath,
    port: PeriphLabel,
) -> Result<(), DescriptorError> {
    enable(rcu_base, model::gpio_enable(port), port)
}

/// Enable a USART's clock.
pub fn enable_usart(
    rcu_base: u32,
    _path: ClockPath,
    usart: PeriphLabel,
) -> Result<(), DescriptorError> {
    enable(rcu_base, model::usart_enable(usart), usart)
}

/// Enable an I2C instance's clock (`APB1EN` bit 21 for I2C0, 22 for I2C1, on both models).
pub fn enable_i2c(
    rcu_base: u32,
    _path: ClockPath,
    i2c: PeriphLabel,
) -> Result<(), DescriptorError> {
    let bit = match i2c {
        PeriphLabel::I2c0 => Some(EnableBit {
            reg: APB1EN,
            bit: 21,
        }),
        PeriphLabel::I2c1 => Some(EnableBit {
            reg: APB1EN,
            bit: 22,
        }),
        _ => None,
    };
    enable(rcu_base, bit, i2c)
}

/// Enable SPI0's clock (`APB2EN` bit 12 on both models).
pub fn enable_spi(
    rcu_base: u32,
    _path: ClockPath,
    spi: PeriphLabel,
) -> Result<(), DescriptorError> {
    let bit = match spi {
        PeriphLabel::Spi0 => Some(EnableBit {
            reg: APB2EN,
            bit: 12,
        }),
        _ => None,
    };
    enable(rcu_base, bit, spi)
}

/// Enable an ADC's clock and set the ADC clock divider and source.
pub fn enable_adc(
    rcu_base: u32,
    _path: ClockPath,
    adc: PeriphLabel,
) -> Result<(), DescriptorError> {
    let bit = match adc {
        PeriphLabel::Adc0 => Some(EnableBit {
            reg: APB2EN,
            bit: 9,
        }),
        PeriphLabel::Adc1 => Some(EnableBit {
            reg: APB2EN,
            bit: 10,
        }),
        PeriphLabel::Adc2 => Some(EnableBit {
            reg: APB2EN,
            bit: 15,
        }),
        _ => None,
    };
    enable(rcu_base, bit, adc)?;
    model::adc_clock(rcu_base);
    Ok(())
}

/// Enable an advanced timer's clock.
pub fn enable_timer(
    rcu_base: u32,
    _path: ClockPath,
    timer: PeriphLabel,
) -> Result<(), DescriptorError> {
    enable(rcu_base, model::timer_enable(timer), timer)
}

/// Enable general TIMER1's clock (`APB1EN` bit 0 on both models).
pub fn enable_general_timer(
    rcu_base: u32,
    _path: ClockPath,
    timer: PeriphLabel,
) -> Result<(), DescriptorError> {
    let bit = match timer {
        PeriphLabel::Timer1 => Some(EnableBit {
            reg: APB1EN,
            bit: 0,
        }),
        _ => None,
    };
    enable(rcu_base, bit, timer)
}

/// Enable the DMA controller's clock (`AHBEN.DMAEN`, bit 0, on both models).
pub(crate) fn enable_dma(rcu_base: u32) {
    Reg32::new(rcu_base, AHBEN).set_bits(1 << 0);
}

const RSTSCK_IRC40KEN: u32 = 1 << 0;
const RSTSCK_IRC40KSTB: u32 = 1 << 1;
const RSTSCK_RSTFC: u32 = 1 << 24;
/// The reset-cause flags, bits 31:25 (low-power, window watchdog, free watchdog, software,
/// power-on, external pin) and bit 23 on the parts that carry it.
const RSTSCK_CAUSE_MASK: u32 = 0xFE80_0000;
/// `RSTSCK.FWDGTRSTF` (bit 29 on both models): the free watchdog caused the last reset. One of the
/// [`reset_cause_flags`].
pub const RESET_CAUSE_FWDGT: u32 = 1 << 29;
const LSI_STAB_SPINS: u32 = 100_000;

/// Start the IRC40K oscillator (the free watchdog's clock) and wait for it to report stable.
pub(crate) fn enable_lsi(rcu_base: u32) -> Result<(), ClockError> {
    let rstsck = Reg32::new(rcu_base, RSTSCK);
    rstsck.set_bits(RSTSCK_IRC40KEN);
    let mut budget = LSI_STAB_SPINS;
    while rstsck.read() & RSTSCK_IRC40KSTB == 0 {
        budget -= 1;
        if budget == 0 {
            return Err(ClockError::SourceNotStable);
        }
    }
    Ok(())
}

/// The reset-cause flags of `RSTSCK`, masked to the cause bits.
#[inline]
pub fn reset_cause_flags(rcu_base: u32) -> u32 {
    Reg32::new(rcu_base, RSTSCK).read() & RSTSCK_CAUSE_MASK
}

/// Clear the reset-cause flags (`RSTSCK.RSTFC`).
#[inline]
pub fn clear_reset_cause(rcu_base: u32) {
    Reg32::new(rcu_base, RSTSCK).set_bits(RSTSCK_RSTFC);
}

const CTL_IRC8MEN: u32 = 1 << 0;
const CTL_IRC8MSTB: u32 = 1 << 1;
const CTL_HXTALEN: u32 = 1 << 16;
const CTL_HXTALSTB: u32 = 1 << 17;
const CTL_PLLEN: u32 = 1 << 24;
const CTL_PLLSTB: u32 = 1 << 25;
const CFG0_SCS: u32 = 0b11;
const CFG0_SCSS: u32 = 0b11 << 2;
const CFG0_AHBPSC: u32 = 0b1111 << 4;
const CFG0_APB1PSC: u32 = 0b111 << 8;
const CFG0_APB2PSC: u32 = 0b111 << 11;
const CFG0_PLLSEL: u32 = 1 << 16;
const CFG0_PLLMF: u32 = (1 << 27) | (0b1111 << 18);
const SCS_PLL: u32 = 0b10;
const SCSS_PLL: u32 = 0b10 << 2;

const FMC_WS_WSCNT: u32 = 0b111;

/// Program the tree: flash wait states first, then the source and its stable poll, the bus
/// prescalers, the PLL source and multiplier, PLL enable and its lock poll, and the switch to the
/// PLL with its read-back confirm. Each poll spins until its flag is up (a source that never comes
/// up is a board fault the image cannot run without).
pub fn configure_tree(chip: &Chip, clock: &ClockConfig) -> Result<(), ClockError> {
    clock.validate()?;
    let rcu = match chip.rcu_base() {
        Ok(b) => b,
        Err(_) => return Err(ClockError::SourceNotStable),
    };
    let ctl = Reg32::new(rcu, CTL);
    let cfg0 = Reg32::new(rcu, CFG0);

    Reg32::new(crate::fmc::FMC_BASE, crate::fmc::WS)
        .modify(FMC_WS_WSCNT, model::wait_states(clock.sysclk_hz));

    let (en, stb) = match clock.source {
        ClockSource::Irc8m => (CTL_IRC8MEN, CTL_IRC8MSTB),
        ClockSource::Hxtal => (CTL_HXTALEN, CTL_HXTALSTB),
    };
    ctl.set_bits(en);
    while ctl.read() & stb == 0 {}

    // validate() checked every code exists.
    let code = |c: Option<u32>| c.unwrap_or(0);
    cfg0.modify(CFG0_AHBPSC, code(ahb_psc_code(clock.ahb_psc)) << 4);
    cfg0.modify(CFG0_APB2PSC, code(apb_psc_code(clock.apb2_psc)) << 11);
    cfg0.modify(CFG0_APB1PSC, code(apb_psc_code(clock.apb1_psc)) << 8);
    let pllsel = match clock.source {
        ClockSource::Irc8m => 0,
        ClockSource::Hxtal => CFG0_PLLSEL,
    };
    cfg0.modify(CFG0_PLLSEL, pllsel);
    cfg0.modify(CFG0_PLLMF, pll_mul_bits(clock.pll_mul));

    ctl.set_bits(CTL_PLLEN);
    while ctl.read() & CTL_PLLSTB == 0 {}

    cfg0.modify(CFG0_SCS, SCS_PLL);
    while cfg0.read() & CFG0_SCSS != SCSS_PLL {}
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_fleet_tree_validates_and_runs_the_buses_at_72_and_36() {
        let c = ClockConfig::REFERENCE_72M_IRC8M;
        assert_eq!(c.validate(), Ok(()));
        assert_eq!(c.apb2_hz(), 72_000_000);
        assert_eq!(c.apb1_hz(), 36_000_000);
        // IRC8M / 2 x 18.
        assert_eq!(8_000_000 / 2 * c.pll_mul as u32, c.sysclk_hz);
    }

    #[test]
    fn bad_trees_are_refused() {
        let base = ClockConfig::REFERENCE_72M_IRC8M;
        let pll = ClockConfig { pll_mul: 1, ..base };
        assert_eq!(pll.validate(), Err(ClockError::InvalidPll));
        let pll = ClockConfig {
            pll_mul: 33,
            ..base
        };
        assert_eq!(pll.validate(), Err(ClockError::InvalidPll));
        let ahb = ClockConfig {
            ahb_psc: 32,
            ..base
        };
        assert_eq!(ahb.validate(), Err(ClockError::InvalidPrescaler));
        let apb = ClockConfig {
            apb1_psc: 3,
            ..base
        };
        assert_eq!(apb.validate(), Err(ClockError::InvalidPrescaler));
        let fast = ClockConfig {
            sysclk_hz: model::MAX_SYSCLK_HZ + 1,
            ..base
        };
        assert_eq!(fast.validate(), Err(ClockError::InvalidWaitStates));
    }

    #[test]
    fn pll_multiplier_codes_follow_the_split_field() {
        assert_eq!(pll_mul_bits(2), 0);
        assert_eq!(pll_mul_bits(16), 14 << 18);
        assert_eq!(pll_mul_bits(17), 1 << 27);
        // x18, the fleet multiplier: bit 27 plus 1 in 21:18 (the SPL's RCU_PLL_MUL18).
        assert_eq!(pll_mul_bits(18), (1 << 27) | (1 << 18));
        assert_eq!(pll_mul_bits(32), (1 << 27) | (15 << 18));
    }

    #[test]
    fn wait_states_are_zero_at_reset_speed_and_the_models_value_above() {
        assert_eq!(model::wait_states(8_000_000), 0);
        let want = if cfg!(hal_clock = "f10x") { 2 } else { 1 };
        assert_eq!(model::wait_states(72_000_000), want);
    }

    #[test]
    fn every_bonded_port_has_a_clock_enable() {
        use crate::descriptor::DESCRIPTOR;
        let ports = [
            PeriphLabel::Gpioa,
            PeriphLabel::Gpiob,
            PeriphLabel::Gpioc,
            PeriphLabel::Gpiod,
            PeriphLabel::Gpioc,
            PeriphLabel::Gpiof,
        ];
        for (i, port) in ports.into_iter().enumerate() {
            if i != 4 && DESCRIPTOR.bonding[i] != 0 {
                assert!(model::gpio_enable(port).is_some(), "{port:?}");
            }
        }
    }

    #[test]
    fn every_carried_instance_has_an_enable_bit() {
        use crate::descriptor::DESCRIPTOR;
        for usart in [
            PeriphLabel::Usart0,
            PeriphLabel::Usart1,
            PeriphLabel::Usart2,
        ] {
            if DESCRIPTOR.addrs.get(usart).is_some() {
                assert!(model::usart_enable(usart).is_some(), "{usart:?}");
            }
        }
        for timer in [PeriphLabel::Timer0, PeriphLabel::Timer7] {
            if DESCRIPTOR.addrs.get(timer).is_some() {
                assert!(model::timer_enable(timer).is_some(), "{timer:?}");
            }
        }
    }
}
