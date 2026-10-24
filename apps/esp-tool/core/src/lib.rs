//! The hardware-free half of the ESP32-C3 board tool (`specs/esp-tool.md`).
//!
//! | Module | Owns |
//! | --- | --- |
//! | [`swd`] | the transfers, retries, connect, memory access and the counters, over [`swd::SwdPins`] |
//! | [`target`] | identify, armed check, disarm, halt, resume, reset, debug hold, release |
//! | [`flash`] | page erase, ascending program, error checks, verify, clearing protection |
//! | [`mailbox`] | the mailbox session, `MemAp` on [`swd`] |
//! | [`ble`] | the L2 stream decoder on the BLE bytes, notification splitting, routing |
//! | [`svc`] | the ESP commands and the bulk transfer window |
//!
//! The firmware crate (`apps/esp-tool`) supplies the pins, the GATT peripheral and the main loop.
#![no_std]

pub mod ble;
pub mod flash;
pub mod mailbox;
#[cfg(any(test, feature = "sim"))]
pub mod sim;
pub mod svc;
pub mod swd;
pub mod target;

#[cfg(test)]
mod tests;
