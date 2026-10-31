// SPDX-License-Identifier: GPL-3.0-or-later
//! Hoverboard outer control (`specs/control.md`, `specs/state-feedback.md`): the 250 Hz balance
//! law, the engagement machine and the throttle conditioner as no-FPU fixed-point math,
//! MCU-independent and host-testable.
//!
//! The engagement machine and its constants are recovered from the stock binary (the normative
//! stock contract is Declassyfied `control.md`, "Section N" below); the balance law is OURS, the
//! four-term state feedback of `specs/state-feedback.md`, which replaced the recovered PID cascade
//! (pitch shaping, the balance PID, the speed loop, the outer speed loop and the commanded lean).
//!
//! Layout:
//! - [`helpers`]  Section 8: clamp/abs/ramp + the round-toward-zero shift (Section 9).
//! - [`config`]   The gain shadow and the tune lane's allowlist, plus the fixed contract
//!   constants.
//! - [`sf`]       The four-term state-feedback balance law, its reference and the side mirror
//!   [`wheel_forward`].
//! - [`drive`]    The rigid frame's turn input and steering split.
//! - [`gating`]   The engagement machine's gating/pickup input: the conditioned up-axis accel
//!   channel (Section 7's `> 500` / `< 0` cell, producer recovered).
//! - [`fsm`]      Section 7: the engagement machine (rebuilt to the binary, slice-5 re-cut) +
//!   the Section-7.3 torque-setpoint output stage.
//! - [`throttle`] The throttle mode's EFeru-parity input conditioning (spec (b); oracle-pinned).
//! - [`mode`]     The control-mode model + dispatch (`CONTROL_MODE`, the validation seam with
//!   fallback + fault, the switch discipline).
//!
//! The reference constants are preserved exactly. Float steps the original computed in IEEE
//! float are flagged "(float in original)" at their use site; the pure-integer paths are
//! bit-exact.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod config;
pub mod drive;
pub mod fsm;
pub mod gating;
pub mod helpers;
pub mod mode;
pub mod sf;
pub mod throttle;

pub use config::{
    GainProfile, GainShadow, GainTriple, TuneError, DEFAULT_GAIN_MAX, GAIN_MIN, REFERENCE_FIELD,
    SF_GAINS_FIELD, STANDBY_SET, TUNED,
};
pub use drive::{wheel_split, DriveTurn};
pub use fsm::{fsm_step, tilt_cutout, FsmInputs, FsmState, SubState};
pub use gating::GatingFilter;
pub use helpers::{clamp, clamp_sym, iabs, ramp_step, RampRecord};
pub use mode::{select_mode, ControlDispatch, ControlMode, Frame, ModeSelection, Producer};
pub use sf::{sf_step, wheel_forward, SfConfig, SfInputs, SfState};
pub use throttle::{throttle_tick, ThrottleConfig, ThrottleOutput, ThrottleState};

#[cfg(test)]
mod tests;
