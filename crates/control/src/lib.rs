//! Cascaded hoverboard balance control: the 250 Hz outer control cascade as no-FPU fixed-point
//! math, MCU-independent and host-testable.
//!
//! Per [todo/control.md](../../../todo/control.md), this crate fixes the ALGORITHM STRUCTURE and
//! the exact reference constants (the per-tick computation order, the saturation order,
//! the state-machine transitions/thresholds, the envelope/slew rates, the IIR structure, and the
//! anti-windup orientation), while keeping the gain profiles and per-board limits as config/tuning
//! inputs (struct fields / a config struct) with the spec values as defaults.
//!
//! Layout (one module per spec section):
//! - [`helpers`]  Section 8: clamp/abs/ramp/PI integrator (anti-windup orientation is load-bearing).
//! - [`config`]   Section 0/6: gain profiles + the fixed contract constants.
//! - [`shaping`]  Section 4: pitch-target shaping (commanded lean).
//! - [`pid`]      Section 3: balance PID (pitch error -> torque) + the 0.99/0.01 reference IIR.
//! - [`speed`]    Section 5: speed/steer outer loop + the speed-setpoint helper.
//! - [`fsm`]      Section 7: balance state machine + the final torque output.
//!
//! Uses the reference constants; constants preserved exactly. Float steps the
//! original computed in IEEE float are flagged "(float in original)" at their use site and mapped
//! to Q-format via the `fixed` crate; the pure-integer paths are bit-exact.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod config;
pub mod fsm;
pub mod helpers;
pub mod pid;
pub mod shaping;
pub mod speed;

// Common re-exports.
pub use config::{
    select_profile, GainProfile, GainTriple, PROFILE_B, RUN_PROFILE_A, STANDBY_SET,
};
pub use fsm::{fsm_step, FsmInputs, FsmState, SubState};
pub use helpers::{clamp, clamp_sym, iabs, pi_step, ramp_step, PiRecord, RampRecord};
pub use pid::{balance_pid, IirCarry, PidInputs, PidOutputs};
pub use shaping::{shape_pitch_target, ShapingInputs, ShapingState};
pub use speed::{speed_loop, speed_setpoint, SpeedInputs, SpeedState};

#[cfg(test)]
mod tests;
