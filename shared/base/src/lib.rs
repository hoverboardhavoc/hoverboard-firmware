// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared no_std primitives for the hoverboard firmware (Layer 0: Foundations).
//!
//! `base` is the bedrock every later layer can depend on without pulling in the link
//! layer or the HAL. It carries the one shared CRC, the fixed-point Q-format
//! conventions, and a lean shared error vocabulary. It is HAL-independent: nothing
//! here touches a peripheral or can fail on hardware.
//!
//! Layout:
//! - [`crc16`]  CRC-16/MODBUS, used by the config store (Layer 1) and the link framer (Layer 3).
//! - [`fixed`]  the Q-format type aliases and the `assert_close` test discipline.
//! - [`lane`]   the three-slot selector both hot-path permutations (SVPWM channel order, IMU axis
//!   roles) are built from, so neither can index out of range.
//! - [`error`]  the shared error/result vocabulary (e.g. [`error::FlashError`]).
//! - [`pi`]     the PI step and its gains, the FOC arm's two current loops (Layer 8).
//! - [`TORQUE_FULL_SCALE`] the torque word's full scale, which the control layer clamps to and the
//!   commutation layer scales its current reference by.

#![no_std]
// The host test harness needs std (for `assert!`, formatting, etc.); the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod crc16;
pub mod error;
pub mod fixed;
pub mod lane;
pub mod pi;

/// The full-scale stock-native torque word (`specs/control.md` (a)): the signed `i16` demand the
/// control layer hands the motor is clamped to +-28,500, negative for forward drive.
///
/// Here, and not in `control`, because two layers need the number and `base` is the one both depend
/// on (`specs/foc-torque.md` section 6): the control crate caps a wheel demand and its steering
/// split at it, and the commutation crate's FOC arm divides the demand by it to scale the q-axis
/// current reference, so a full-scale word asks for exactly the arm's full-scale current. The
/// commutation crate cannot depend on `control` (it sits below it).
pub const TORQUE_FULL_SCALE: i32 = 28_500;
