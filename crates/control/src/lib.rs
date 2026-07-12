//! Cascaded hoverboard outer control (`specs/control.md`): the 250 Hz outer control cascade as
//! no-FPU fixed-point math, MCU-independent and host-testable.
//!
//! The crate fixes the ALGORITHM STRUCTURE and the exact reference constants (the per-tick
//! computation order, the saturation order, the state-machine transitions/thresholds, the
//! envelope/slew rates, the IIR structure, and the anti-windup orientation), while keeping the
//! gain profiles and per-board limits as config/tuning inputs with the recovered stock values as
//! defaults. Recovered from the archived implementation (`archive/accumulated-build`, commit
//! `74b7773`) per the spec's provenance section; the normative stock contract is Declassyfied
//! `control.md` (section references "Section N" below), and the PI record + step recovered there
//! were already relocated to `base::pi` in Phase B.
//!
//! Layout (one module per stock spec section; the later modules arrive one slice at a time per
//! `specs/control.md`, "Step-2 implementation slicing"):
//! - [`helpers`]  Section 8: clamp/abs/ramp + the round-toward-zero shift (Section 9).
//! - [`config`]   Section 0/6: gain profiles + the fixed contract constants.
//!
//! The reference constants are preserved exactly. Float steps the original computed in IEEE
//! float are flagged "(float in original)" at their use site; the pure-integer paths are
//! bit-exact.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod config;
pub mod helpers;

// Common re-exports (the archived list, minus the base::pi relocation, scoped to the built
// slices).
pub use config::{select_profile, GainProfile, GainTriple, PROFILE_B, RUN_PROFILE_A, STANDBY_SET};
pub use helpers::{clamp, clamp_sym, iabs, ramp_step, RampRecord};

#[cfg(test)]
mod tests;
