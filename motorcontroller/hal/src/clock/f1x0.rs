// SPDX-License-Identifier: GPL-3.0-or-later
//! The GD32F1x0 RCU model: the enable-bit map, the flash wait states and the ADC clock
//! (GD32F1x0 User Manual, section 6).

use super::{EnableBit, AHBEN, APB1EN, APB2EN};
use crate::addr::PeriphLabel;
use crate::reg::Reg32;

/// The highest system clock the part runs (GD32F130: 72 MHz).
pub(crate) const MAX_SYSCLK_HZ: u32 = 72_000_000;

/// Flash wait states above 8 MHz: one (GD32F1x0 `FMC_WS`: 0 up to 24 MHz, 1 up to 48, 2 up to 72
/// on the parts that take it; this HAL keeps the one wait state `runtime-hal` ran the fleet on).
pub(crate) const fn wait_states(sysclk_hz: u32) -> u32 {
    if sysclk_hz <= 8_000_000 {
        0
    } else {
        1
    }
}

/// The GPIO port enable: `AHBEN.PxEN`, port A at bit 17.
pub(crate) const fn gpio_enable(port: PeriphLabel) -> Option<EnableBit> {
    let bit = match port {
        PeriphLabel::Gpioa => 17,
        PeriphLabel::Gpiob => 18,
        PeriphLabel::Gpioc => 19,
        PeriphLabel::Gpiod => 20,
        PeriphLabel::Gpiof => 22,
        _ => return None,
    };
    Some(EnableBit { reg: AHBEN, bit })
}

/// The USART enable: USART0 on APB2 bit 14, USART1 on APB1 bit 17. The F1x0 has no USART2.
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
        _ => None,
    }
}

/// The advanced-timer enable: TIMER0 at APB2 bit 11. The F1x0 has no TIMER7.
pub(crate) const fn timer_enable(timer: PeriphLabel) -> Option<EnableBit> {
    match timer {
        PeriphLabel::Timer0 => Some(EnableBit {
            reg: APB2EN,
            bit: 11,
        }),
        _ => None,
    }
}

const CFG2: u32 = 0x30;
/// `CFG2.ADCSEL`: the ADC clock from the APB2 prescaler (rather than the 14 MHz IRC14M).
const CFG2_ADCSEL: u32 = 1 << 8;

/// The ADC clock: `CFG0.ADCPSC` = PCLK2 / 6, selected by `CFG2.ADCSEL`.
pub(crate) fn adc_clock(rcu_base: u32) {
    Reg32::new(rcu_base, super::CFG0).modify(super::CFG0_ADCPSC, super::ADCPSC_DIV6);
    Reg32::new(rcu_base, CFG2).set_bits(CFG2_ADCSEL);
}
