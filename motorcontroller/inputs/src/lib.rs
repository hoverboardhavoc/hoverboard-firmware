// SPDX-License-Identifier: GPL-3.0-or-later
//! Hoverboard input conditioning: discrete-line debounce, combo/edge derivation, and the
//! rider-present foot-pad field. A pure producer of shared state, owning no actuator and no
//! hardware. The caller samples the GPIO line levels and the pad levels; this crate turns them into
//! debounced flags, combo flags, and the 2-bit pad field.
//!
//! The consumer is the integration input task (`specs/integration.md`, "The input task": the
//! debounced power button and the foot pads). This crate fixes the BEHAVIOR and the exact reference
//! CONSTANTS. Every concrete pin assignment, polarity, and combo-pair membership is
//! board-definition config the caller resolves (the `BoardPlan`'s pin fields, and
//! `store::BOARD_INPUT_POLARITY` through [`asserted`], the one mapping this crate does hold);
//! here the count of debounced lines and the combo memberships are parameters, and the machine is
//! replicated per line. Recovered from the archived implementation
//! (`archive/accumulated-build:motorcontroller/inputs`, commit `74b7773`) per `specs/integration.md`'s
//! sources section.
//!
//! Everything here runs at 16 ms (every 4th scheduler tick), and is pure integer/boolean: no-FPU by
//! construction, with no Q-format carry anywhere in the crate.
//!
//! A 4 ms `throttle` module lived here too, scaling and IIR-filtering the raw ADC throttle word a
//! board's own hardware produces. It is DELETED (`specs/link-control.md`, "`INPUTS` (2 B): remote input mirror"): the only thing that ever
//! fed it was the remote `INPUTS.throttle` mirror, no board here has a physical throttle, and
//! nothing read its output. Demand arrives as `DRIVE_CMD` and is conditioned by
//! `motorcontroller/control/src/throttle.rs`, which is a different filter on a different field.
//!
//! The reference constants are preserved exactly.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod combo;
pub mod debounce;
pub mod pad;

/// Bit 0 of the board's discrete input word: the power button.
pub const BUTTON_LINE: u8 = 0b001;
/// Bit 1: foot pad A.
pub const PAD_A_LINE: u8 = 0b010;
/// Bit 2: foot pad B.
pub const PAD_B_LINE: u8 = 0b100;
/// The three lines a board wires, as a mask.
pub const INPUT_LINES: u8 = BUTTON_LINE | PAD_A_LINE | PAD_B_LINE;

/// Resolve one sampled level code against the board's polarity word: the returned bits are the
/// lines sitting at their ASSERTED level, in the same bit order.
///
/// This is the "polarity mapping the caller does" that [`debounce`] and [`pad`] have always named
/// and never held. `levels` is the pin read (a set bit = the line reads logic 1) and `polarity` is
/// `store::BOARD_INPUT_POLARITY` as the board stores it (a set bit = that line is ACTIVE HIGH), so
/// a line asserts when its level equals its polarity bit. Lines the board does not wire are the
/// caller's to drop: an unconfigured line reads its idle level here like any other.
///
/// Bits 3..7 of the polarity word are reserved and inert, which this mask is where it is enforced:
/// a word with one set cannot make a fourth line, and cannot change any of the three.
pub const fn asserted(levels: u8, polarity: u8) -> u8 {
    (levels ^ !polarity) & INPUT_LINES
}

// Common re-exports.
pub use combo::{combined_button, ComboPair, ComboSet, ComboState};
pub use debounce::{DebounceLine, DebouncePhase, LineBank, MAX_LINES};
pub use pad::{PadBank, PadField, PAD_A_BIT, PAD_B_BIT};

#[cfg(test)]
mod tests;
