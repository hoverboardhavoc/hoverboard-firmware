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
//! - [`error`]  the shared error/result vocabulary (e.g. [`error::FlashError`]).

#![no_std]
// The host test harness needs std (for `assert!`, formatting, etc.); the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod crc16;
pub mod error;
pub mod fixed;
