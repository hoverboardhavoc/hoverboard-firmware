// SPDX-License-Identifier: GPL-3.0-or-later
//! Hoverboard input conditioning: discrete-line debounce, combo/edge derivation, and the
//! rider-present foot-pad field. A pure producer of shared state, owning no actuator and no
//! hardware. The caller samples the GPIO line levels and the pad levels; this crate turns them into
//! debounced flags, combo flags, and the 2-bit pad field.
//!
//! The consumer is the integration input task (`specs/integration.md`, "The input task": the
//! debounced power button and the foot pads). This crate fixes the BEHAVIOR and the exact reference
//! CONSTANTS. Every concrete pin assignment, polarity, and combo-pair membership is
//! board-definition config the caller resolves (the `BoardPlan`'s pin fields); here the count of
//! debounced lines and the combo memberships are parameters, and the machine is replicated per
//! line. Recovered from the archived implementation
//! (`archive/accumulated-build:crates/inputs`, commit `74b7773`) per `specs/integration.md`'s
//! sources section.
//!
//! Everything here runs at 16 ms (every 4th scheduler tick), and is pure integer/boolean: no-FPU by
//! construction, with no Q-format carry anywhere in the crate.
//!
//! A 4 ms `throttle` module lived here too, scaling and IIR-filtering the raw ADC throttle word a
//! board's own hardware produces. It is DELETED (`specs/todo.md` part 3): the only thing that ever
//! fed it was the remote `INPUTS.throttle` mirror, no board here has a physical throttle, and
//! nothing read its output. Demand arrives as `DRIVE_CMD` and is conditioned by
//! `crates/control/src/throttle.rs`, which is a different filter on a different field.
//!
//! The reference constants are preserved exactly.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod combo;
pub mod debounce;
pub mod pad;

// Common re-exports.
pub use combo::{combined_button, ComboPair, ComboSet, ComboState};
pub use debounce::{DebounceLine, DebouncePhase, LineBank, MAX_LINES};
pub use pad::{PadBank, PadField, PAD_A_BIT, PAD_B_BIT};

#[cfg(test)]
mod tests;
