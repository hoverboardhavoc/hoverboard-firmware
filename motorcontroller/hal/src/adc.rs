// SPDX-License-Identifier: GPL-3.0-or-later
//! The ADC: the timer-triggered group the period ISR reads, the regular group's polled single
//! conversion, and on the F10x the ADC0/ADC1 regular-simultaneous mode.
//!
//! Three register models (`hal_adc`), one per build (specs/hal.md, section 3.1):
//!
//! - `f10x` and `f1x0` (GD32F10x and GD32F1x0, `adc/gd.rs`): one register model for the regular
//!   and injected groups; the F10x adds the ADC0/ADC1 sync mode (`DualAdc`).
//! - `mm32` (MM32SPIN05, `adc/mm32.rs`): another register set (`ADCFG`/`ADCR`/`ADCHS`/`ADSTA`, a
//!   data register per channel, no injected group), whose triggered scan stands in for the
//!   injected group.
//!
//! Each model carries [`Adc`], [`InjectedHandle`] and the [`TriggeredAdc`] implementation of
//! [`InjectedAdcController`]; this file holds the trait, the controller, ADC0's base and the poll
//! bound they share. The ADC clock is [`crate::clock::enable_adc`]'s.

#[cfg(any(hal_adc = "f10x", hal_adc = "f1x0"))]
#[path = "adc/gd.rs"]
mod model;
#[cfg(hal_adc = "mm32")]
#[path = "adc/mm32.rs"]
mod model;

pub use model::*;

use crate::addr::PeriphLabel;
use crate::chip::Chip;
use crate::config::InjectedAdcConfig;
use crate::descriptor::DESCRIPTOR;
use crate::error::BringUpError;

/// Polls of a calibration bit or a conversion flag before the wait gives up. It counts loop
/// iterations, not time; a working ADC finishes well inside it at any clock this HAL runs.
const ADC_TIMEOUT: u32 = 100_000;

/// ADC0's base (the MM32's ADC1), which every chip carries.
const ADC0_BASE: u32 = match DESCRIPTOR.addrs.get(PeriphLabel::Adc0) {
    Some(b) => b,
    None => panic!("the descriptor carries no ADC0"),
};

/// A timer-triggered ADC group that configures into a per-period handle.
pub trait TriggeredAdc {
    /// The per-period handle the configuration resolves into.
    type Handle: Copy;

    /// Configure the group from `cfg` and return its handle.
    fn configure(&self, chip: &Chip, cfg: &InjectedAdcConfig)
        -> Result<Self::Handle, BringUpError>;
}

/// The triggered group's bring-up: the injected group on the GD32 models, the triggered scan on
/// the MM32.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InjectedAdcController;

impl InjectedAdcController {
    /// The controller; the ADC instance comes from the configuration.
    #[inline]
    pub const fn new() -> Self {
        InjectedAdcController
    }
}
