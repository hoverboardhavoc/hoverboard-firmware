// SPDX-License-Identifier: GPL-3.0-or-later
//! Pin capability queries: what a packed pin byte can be on the compiled chip.
//!
//! Pure functions of the descriptor and the tables here: no register access. A capability the HAL
//! cannot express (a remapped gate set, a remapped I2C pair) answers `None` rather than a guess.
//! Pins are packed bytes, `(port << 4) | pin`, port A = 0, B = 1, C = 2, D = 3, F = 5.

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::error::DescriptorError;
use crate::gpio::UsartAf;

/// TIMER0 CH0..CH2 on PA8..PA10, CH0N..CH2N on PB13..PB15.
const TIMER0_CH: [u8; 3] = [0x08, 0x09, 0x0A];
const TIMER0_CHN: [u8; 3] = [0x1D, 0x1E, 0x1F];
/// TIMER7 CH0..CH2 on PC6..PC8, CH0N on PA7, CH1N/CH2N on PB0/PB1 (the high-density F10x).
const TIMER7_CH: [u8; 3] = [0x26, 0x27, 0x28];
const TIMER7_CHN: [u8; 3] = [0x07, 0x10, 0x11];
/// I2C0 on PB6 (SCL) / PB7 (SDA), I2C1 on PB10 / PB11.
const I2C0_PAIR: (u8, u8) = (0x16, 0x17);
const I2C1_PAIR: (u8, u8) = (0x1A, 0x1B);
/// PA13 (SWDIO) and PA14 (SWCLK).
const SWD_PINS: [u8; 2] = [0x0D, 0x0E];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Dir {
    Tx,
    Rx,
}

/// One USART pin the HAL can route.
struct UsartPin {
    pin: u8,
    usart: PeriphLabel,
    dir: Dir,
    /// The F1x0 mux number.
    af: UsartAf,
    /// The F10x remap it needs.
    remap: UsartRemap,
}

/// The USART pins: USART1 on PA2/PA3, USART0 on PB6/PB7 (F10x through the USART0 remap), USART2 on
/// PB10/PB11. Whether the chip carries the instance is the descriptor's answer.
const USART_PINS: [UsartPin; 6] = [
    UsartPin {
        pin: 0x02,
        usart: PeriphLabel::Usart1,
        dir: Dir::Tx,
        af: UsartAf::Af1,
        remap: UsartRemap::Default,
    },
    UsartPin {
        pin: 0x03,
        usart: PeriphLabel::Usart1,
        dir: Dir::Rx,
        af: UsartAf::Af1,
        remap: UsartRemap::Default,
    },
    UsartPin {
        pin: 0x16,
        usart: PeriphLabel::Usart0,
        dir: Dir::Tx,
        af: UsartAf::Af0,
        remap: UsartRemap::Usart0,
    },
    UsartPin {
        pin: 0x17,
        usart: PeriphLabel::Usart0,
        dir: Dir::Rx,
        af: UsartAf::Af0,
        remap: UsartRemap::Usart0,
    },
    UsartPin {
        pin: 0x1A,
        usart: PeriphLabel::Usart2,
        dir: Dir::Tx,
        af: UsartAf::Af1,
        remap: UsartRemap::Default,
    },
    UsartPin {
        pin: 0x1B,
        usart: PeriphLabel::Usart2,
        dir: Dir::Rx,
        af: UsartAf::Af1,
        remap: UsartRemap::Default,
    },
];

/// The AFIO remap a USART pin pair needs on the F10x (the F1x0 routes through the per-pin mux and
/// always answers `Default`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsartRemap {
    /// The instance's default pins.
    Default,
    /// USART0 on PB6/PB7 (`AFIO_PCF0.USART0_REMAP`).
    Usart0,
}

/// A routable USART pin pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsartMapping {
    /// The instance.
    pub usart: PeriphLabel,
    /// The transmit pin's F1x0 mux number.
    pub tx_af: UsartAf,
    /// The receive pin's F1x0 mux number.
    pub rx_af: UsartAf,
    /// The F10x remap.
    pub remap: UsartRemap,
}

/// Whether the pin is bonded out on the chip's package.
pub fn pin_exists(chip: &Chip, pin: u8) -> bool {
    chip.descriptor().bonded(pin)
}

/// Whether the pin is an advanced-timer gate pin of a timer the chip carries.
pub fn gate_capable(chip: &Chip, pin: u8) -> bool {
    TIMER0_CH.contains(&pin)
        || TIMER0_CHN.contains(&pin)
        || (chip.has_advanced_timer(PeriphLabel::Timer7)
            && (TIMER7_CH.contains(&pin) || TIMER7_CHN.contains(&pin)))
}

/// The timer a full gate set (three high, three low pins, channel order) drives, or `None`.
pub fn gate_set(chip: &Chip, hi: [u8; 3], lo: [u8; 3]) -> Option<PeriphLabel> {
    if hi == TIMER0_CH && lo == TIMER0_CHN {
        return Some(PeriphLabel::Timer0);
    }
    if chip.has_advanced_timer(PeriphLabel::Timer7) && hi == TIMER7_CH && lo == TIMER7_CHN {
        return Some(PeriphLabel::Timer7);
    }
    None
}

/// The ADC channel on the pin (the F1 map: PA0-7 = 0-7, PB0-1 = 8-9, PC0-5 = 10-15), or `None`
/// when the pin is not an ADC input or not bonded.
pub fn adc_channel(chip: &Chip, pin: u8) -> Option<u8> {
    if !pin_exists(chip, pin) {
        return None;
    }
    match (pin >> 4, pin & 0x0F) {
        (0, n) if n <= 7 => Some(n),
        (1, n) if n <= 1 => Some(8 + n),
        (2, n) if n <= 5 => Some(10 + n),
        _ => None,
    }
}

/// The I2C instance on an (SCL, SDA) pair, or `None`.
pub fn i2c_instance(chip: &Chip, scl: u8, sda: u8) -> Option<PeriphLabel> {
    if (scl, sda) == I2C0_PAIR {
        return Some(PeriphLabel::I2c0);
    }
    if (scl, sda) == I2C1_PAIR && chip.has_i2c(PeriphLabel::I2c1) {
        return Some(PeriphLabel::I2c1);
    }
    None
}

/// The USART a (TX, RX) pair routes to on this chip, or `None`.
pub fn usart_pins(chip: &Chip, tx: u8, rx: u8) -> Option<UsartMapping> {
    let row = |pin: u8, dir: Dir| USART_PINS.iter().find(|r| r.pin == pin && r.dir == dir);
    let (t, r) = (row(tx, Dir::Tx)?, row(rx, Dir::Rx)?);
    if t.usart != r.usart || chip.descriptor().addrs.get(t.usart).is_none() {
        return None;
    }
    // The F1x0 mux needs no remap; on the F10x both pins share the instance's remap by
    // construction of the table.
    let remap = if cfg!(hal_gpio = "ctl") {
        UsartRemap::Default
    } else {
        t.remap
    };
    Some(UsartMapping {
        usart: t.usart,
        tx_af: t.af,
        rx_af: r.af,
        remap,
    })
}

/// Refuse PA13 and PA14.
#[inline]
pub(crate) fn check_not_swd(pin: u8) -> Result<(), DescriptorError> {
    if SWD_PINS.contains(&pin) {
        Err(DescriptorError::SwdPin)
    } else {
        Ok(())
    }
}

/// What a bring-up claims a pin for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinClaim {
    /// A gate output.
    Gate,
    /// Anything else.
    NonGate,
}

/// The ownership backstop: a gate pin is only ever a gate, and a gate claim only ever takes a gate
/// pin; the SWD pins are nobody's.
pub fn check_pin_ownership(chip: &Chip, pin: u8, claim: PinClaim) -> Result<(), DescriptorError> {
    check_not_swd(pin)?;
    if gate_capable(chip, pin) == matches!(claim, PinClaim::Gate) {
        Ok(())
    } else {
        Err(DescriptorError::PinOwnership)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::descriptor::DESCRIPTOR;

    fn chip() -> Chip {
        // SAFETY: the pure queries touch no register.
        unsafe { Chip::steal() }
    }

    fn all_pins() -> impl Iterator<Item = u8> {
        (0u8..=5).flat_map(|p| (0u8..16).map(move |n| (p << 4) | n))
    }

    #[test]
    fn the_timer0_set_is_a_gate_set_everywhere() {
        assert_eq!(
            gate_set(&chip(), TIMER0_CH, TIMER0_CHN),
            Some(PeriphLabel::Timer0)
        );
        // Swapped or partial sets are not.
        assert_eq!(gate_set(&chip(), TIMER0_CHN, TIMER0_CH), None);
        assert_eq!(gate_set(&chip(), [0x08, 0x09, 0x0B], TIMER0_CHN), None);
    }

    #[test]
    fn the_timer7_set_follows_the_descriptor() {
        let want = (DESCRIPTOR.adv_timers >= 2).then_some(PeriphLabel::Timer7);
        assert_eq!(gate_set(&chip(), TIMER7_CH, TIMER7_CHN), want);
        assert_eq!(gate_capable(&chip(), 0x26), want.is_some());
    }

    #[test]
    fn swd_pins_are_refused_for_every_claim() {
        for pin in SWD_PINS {
            for claim in [PinClaim::Gate, PinClaim::NonGate] {
                assert_eq!(
                    check_pin_ownership(&chip(), pin, claim),
                    Err(DescriptorError::SwdPin)
                );
            }
        }
    }

    #[test]
    fn a_gate_pin_is_only_a_gate() {
        for pin in all_pins().filter(|&p| !SWD_PINS.contains(&p)) {
            let gate = gate_capable(&chip(), pin);
            assert_eq!(
                check_pin_ownership(&chip(), pin, PinClaim::Gate).is_ok(),
                gate,
                "{pin:#04x}"
            );
            assert_eq!(
                check_pin_ownership(&chip(), pin, PinClaim::NonGate).is_ok(),
                !gate,
                "{pin:#04x}"
            );
        }
    }

    #[test]
    fn adc_channels_are_the_f1_map_on_bonded_pins_only() {
        assert_eq!(adc_channel(&chip(), 0x00), Some(0));
        assert_eq!(adc_channel(&chip(), 0x07), Some(7));
        assert_eq!(adc_channel(&chip(), 0x10), Some(8));
        assert_eq!(adc_channel(&chip(), 0x11), Some(9));
        assert_eq!(adc_channel(&chip(), 0x12), None);
        // PC0-5 are channels 10-15 only where PC0-5 are bonded (the LQFP64 GD32F103RC).
        let pc0 = DESCRIPTOR.bonded(0x20).then_some(10);
        assert_eq!(adc_channel(&chip(), 0x20), pc0);
        for pin in all_pins() {
            if adc_channel(&chip(), pin).is_some() {
                assert!(DESCRIPTOR.bonded(pin), "{pin:#04x}");
            }
        }
    }

    #[test]
    fn i2c_pairs_need_the_instance() {
        assert_eq!(i2c_instance(&chip(), 0x16, 0x17), Some(PeriphLabel::I2c0));
        assert_eq!(i2c_instance(&chip(), 0x17, 0x16), None);
        let i2c1 = DESCRIPTOR
            .addrs
            .get(PeriphLabel::I2c1)
            .map(|_| PeriphLabel::I2c1);
        assert_eq!(i2c_instance(&chip(), 0x1A, 0x1B), i2c1);
    }

    #[test]
    fn usart_pairs_resolve_per_chip() {
        let m = usart_pins(&chip(), 0x02, 0x03).expect("USART1 on PA2/PA3 everywhere");
        assert_eq!(m.usart, PeriphLabel::Usart1);
        assert_eq!(m.remap, UsartRemap::Default);
        let m = usart_pins(&chip(), 0x16, 0x17).expect("USART0 on PB6/PB7 everywhere");
        assert_eq!(m.usart, PeriphLabel::Usart0);
        let want = if cfg!(hal_gpio = "crl") {
            UsartRemap::Usart0
        } else {
            UsartRemap::Default
        };
        assert_eq!(m.remap, want);
        assert_eq!((m.tx_af, m.rx_af), (UsartAf::Af0, UsartAf::Af0));
        // USART2 exists only on the F10x.
        let usart2 = usart_pins(&chip(), 0x1A, 0x1B).map(|m| m.usart);
        let want = DESCRIPTOR
            .addrs
            .get(PeriphLabel::Usart2)
            .map(|_| PeriphLabel::Usart2);
        assert_eq!(usart2, want);
        // Mixed instances and reversed directions are no mapping.
        assert_eq!(usart_pins(&chip(), 0x02, 0x17), None);
        assert_eq!(usart_pins(&chip(), 0x03, 0x02), None);
        // USART0's default PA9/PA10 are gate pins and not in the table.
        assert_eq!(usart_pins(&chip(), 0x09, 0x0A), None);
    }
}
