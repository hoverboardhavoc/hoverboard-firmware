// SPDX-License-Identifier: GPL-3.0-or-later
//! The hoverboard HAL (specs/hal.md).
//!
//! One source drives the GD32F103C8, the GD32F130C8 and the GD32F103RC. The chip is chosen at
//! compile time by exactly one cargo feature; a call resolves to that chip's register model with no
//! runtime selector. A chip is its [`descriptor::DESCRIPTOR`] plus one register model per
//! peripheral, mapped in build.rs.
//!
//! The API is the one the firmware called on `runtime-hal`, by name and shape, less what served
//! detection (section 4.2): [`Chip::take`] returns the context for the compiled chip.

#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

#[cfg(not(any(feature = "gd32f103c8", feature = "gd32f130c8", feature = "gd32f103rc")))]
compile_error!("hal: enable exactly one chip feature (gd32f103c8, gd32f130c8 or gd32f103rc)");

#[cfg(any(
    all(feature = "gd32f103c8", feature = "gd32f130c8"),
    all(feature = "gd32f103c8", feature = "gd32f103rc"),
    all(feature = "gd32f130c8", feature = "gd32f103rc"),
))]
compile_error!("hal: more than one chip feature is enabled; enable exactly one");

pub mod addr;
#[cfg(hal_gpio = "crl")]
mod afio;
pub mod chip;
pub mod clock;
pub mod config;
pub mod dbg;
pub mod descriptor;
pub mod error;
pub mod gpio;
pub mod irq;
pub mod pincap;
pub mod reg;

pub use addr::{AddrTable, PeriphLabel};
pub use chip::{Chip, ClockPath, IrqLayout};
pub use clock::{clear_reset_cause, reset_cause_flags, ClockConfig};
#[cfg(target_arch = "arm")]
pub use cortex_m_rt::interrupt;
pub use dbg::debug_hold_timer0;
pub use descriptor::{McuDescriptor, PageSize};
pub use error::{DescriptorError, FmcError};
pub use gpio::{Floating, Input, InputGroup, Output, Pin, PullDown, PullUp, PushPull};
/// The interrupt enum under the name cortex-m-rt's `#[interrupt]` attribute looks for, next to
/// the attribute itself, so an image writes `use hal::interrupt;` and `#[interrupt] fn ADC0_1()`.
pub use irq::Interrupt as interrupt;
pub use irq::Interrupt;
