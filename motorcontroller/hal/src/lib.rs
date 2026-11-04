// SPDX-License-Identifier: GPL-3.0-or-later
//! The hoverboard HAL (specs/hal.md).
//!
//! One source drives the GD32F103C8, the GD32F130C8, the GD32F103RC and the MM32SPIN05. The chip is chosen at
//! compile time by exactly one cargo feature; a call resolves to that chip's register model with no
//! runtime selector. A chip is its [`descriptor::DESCRIPTOR`] plus one register model per
//! peripheral, mapped in build.rs.
//!
//! The API is the one the firmware called on `runtime-hal`, by name and shape, less what served
//! detection (section 4.2): [`Chip::take`] returns the context for the compiled chip.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

#[cfg(not(any(
    feature = "gd32f103c8",
    feature = "gd32f130c8",
    feature = "gd32f103rc",
    feature = "mm32spin05",
    feature = "pincap"
)))]
compile_error!(
    "hal: enable exactly one chip feature (gd32f103c8, gd32f130c8, gd32f103rc or mm32spin05), or `pincap` for the host's pin tables"
);

#[cfg(any(
    all(feature = "gd32f103c8", feature = "gd32f130c8"),
    all(feature = "gd32f103c8", feature = "gd32f103rc"),
    all(feature = "gd32f103c8", feature = "mm32spin05"),
    all(feature = "gd32f130c8", feature = "gd32f103rc"),
    all(feature = "gd32f130c8", feature = "mm32spin05"),
    all(feature = "gd32f103rc", feature = "mm32spin05"),
))]
compile_error!("hal: more than one chip feature is enabled; enable exactly one");

#[cfg(hal_chip)]
pub mod adc;
#[cfg(hal_chip)]
pub mod addr;
#[cfg(hal_chip)]
#[cfg(hal_gpio = "crl")]
pub mod afio;
#[cfg(hal_chip)]
pub mod chip;
#[cfg(hal_chip)]
pub mod clock;
#[cfg(hal_chip)]
pub mod config;
#[cfg(hal_chip)]
pub mod dbg;
#[cfg(hal_chip)]
pub mod descriptor;
#[cfg(hal_chip)]
mod dma;
#[cfg(hal_chip)]
pub mod error;
#[cfg(hal_chip)]
pub mod fmc;
#[cfg(hal_chip)]
pub mod gpio;
#[cfg(hal_chip)]
pub mod i2c;
#[cfg(hal_chip)]
pub mod irq;
#[cfg(feature = "pincap")]
pub mod pincap;
#[cfg(hal_chip)]
pub mod reg;
#[cfg(hal_chip)]
pub mod serial;
#[cfg(hal_chip)]
#[cfg(hal_spi = "gd")]
pub mod spi;
#[cfg(hal_chip)]
pub mod timer;
#[cfg(hal_chip)]
pub mod usart;
#[cfg(hal_chip)]
pub mod usart_rx;
#[cfg(hal_chip)]
pub mod usart_tx;
#[cfg(hal_chip)]
pub mod watchdog;

#[cfg(hal_chip)]
pub use adc::{Adc, InjectedAdcController, InjectedHandle, TriggeredAdc};
#[cfg(hal_chip)]
#[cfg(hal_adc = "f10x")]
pub use adc::{DualAdc, InjectedParallelHandle};
#[cfg(hal_chip)]
pub use addr::{AddrTable, PeriphLabel};
#[cfg(hal_chip)]
pub use chip::{Chip, ClockPath, IrqLayout, UniqueId};
#[cfg(hal_chip)]
pub use clock::{clear_reset_cause, reset_cause_flags, ClockConfig};
#[cfg(hal_chip)]
#[cfg(target_arch = "arm")]
pub use cortex_m_rt::interrupt;
#[cfg(hal_chip)]
pub use dbg::debug_hold_timers;
#[cfg(hal_chip)]
pub use descriptor::{McuDescriptor, PageSize};
#[cfg(hal_chip)]
pub use error::{DescriptorError, FmcError};
#[cfg(hal_chip)]
pub use gpio::{
    Floating, Input, InputGroup, Output, Pin, PinAf, PinId, PullDown, PullUp, PushPull,
};
#[cfg(hal_chip)]
pub use i2c::{I2c, I2cMode};
/// The interrupt enum under the name cortex-m-rt's `#[interrupt]` attribute looks for, next to
/// the attribute itself, so an image writes `use hal::interrupt;` and `#[interrupt] fn ADC0_1()`.
#[cfg(hal_chip)]
pub use irq::Interrupt as interrupt;
#[cfg(hal_chip)]
pub use irq::Interrupt;
#[cfg(hal_chip)]
pub use serial::SplitSerial;
#[cfg(hal_chip)]
#[cfg(hal_spi = "gd")]
pub use spi::Spi;
#[cfg(hal_chip)]
pub use timer::{ArmGate, PwmHandle, PwmTimer};
#[cfg(hal_chip)]
pub use usart::Usart;
#[cfg(hal_chip)]
pub use usart_rx::{supports_rx, RingBufferedRx};
#[cfg(hal_chip)]
pub use usart_tx::DmaTx;
#[cfg(hal_chip)]
pub use watchdog::{FreeWatchdog, WdgTimeout};
