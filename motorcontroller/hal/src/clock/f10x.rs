// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F10x RCC model: the enable-bit map, the flash wait states and the ADC clock
//! (GD32F10x User Manual, section 5).

use super::{EnableBit, APB1EN, APB2EN};
use crate::addr::PeriphLabel;
use crate::reg::Reg32;

/// The highest system clock the part runs (GD32F103: 108 MHz).
pub(crate) const MAX_SYSCLK_HZ: u32 = 108_000_000;

/// Flash wait states above 8 MHz. Two covers the 72 MHz tree (GD32F10x `FMC_WS`: 0 up to 24 MHz,
/// 1 up to 48, 2 up to 72), and is what the SPL's 72 MHz system init writes.
pub(crate) const fn wait_states(sysclk_hz: u32) -> u32 {
    if sysclk_hz <= 8_000_000 {
        0
    } else {
        2
    }
}

/// The GPIO port enable for port `port` (`0` = A): the port A enable bit, `APB2EN` bit 2, plus
/// the port number.
pub(crate) const fn gpio_enable(port: u8) -> EnableBit {
    EnableBit {
        reg: APB2EN,
        bit: 2 + port,
    }
}

/// The USART enable: USART0 on APB2 bit 14, USART1 and USART2 on APB1 bits 17 and 18.
pub(crate) const fn usart_enable(usart: PeriphLabel) -> Option<EnableBit> {
    match usart {
        PeriphLabel::Usart0 => Some(EnableBit {
            reg: APB2EN,
            bit: 14,
        }),
        PeriphLabel::Usart1 => Some(EnableBit {
            reg: APB1EN,
            bit: 17,
        }),
        PeriphLabel::Usart2 => Some(EnableBit {
            reg: APB1EN,
            bit: 18,
        }),
        _ => None,
    }
}

/// The advanced-timer enable: TIMER0 at APB2 bit 11, TIMER7 at bit 13.
pub(crate) const fn timer_enable(timer: PeriphLabel) -> Option<EnableBit> {
    match timer {
        PeriphLabel::Timer0 => Some(EnableBit {
            reg: APB2EN,
            bit: 11,
        }),
        PeriphLabel::Timer7 => Some(EnableBit {
            reg: APB2EN,
            bit: 13,
        }),
        _ => None,
    }
}

/// The ADC clock: `CFG0.ADCPSC` = PCLK2 / 6 (12 MHz from the 72 MHz tree, under the 14 MHz limit).
pub(crate) fn adc_clock(rcu_base: u32) {
    Reg32::new(rcu_base, super::CFG0).modify(super::CFG0_ADCPSC, super::ADCPSC_DIV6);
}
