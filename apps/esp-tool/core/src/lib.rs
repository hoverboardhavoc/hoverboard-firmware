//! The hardware-free half of the ESP32-C3 board tool (`specs/esp-tool.md`).
//!
//! | Module | Owns |
//! | --- | --- |
//! | [`swd`] | the transfers, retries, connect, memory access and the counters, over [`swd::SwdPins`] |
//! | [`target`] | identify, armed check, disarm, halt, resume, reset, debug hold, release |
//! | [`flash`] | page erase, ascending program, error checks, verify, clearing protection |
//! | [`mailbox`] | the mailbox session, `MemAp` on [`swd`] |
//! | [`router`] | the L2 stream decoders on the BLE and USB bytes, routing between them, the mailbox and the commands |
//! | [`svc`] | the ESP commands and the bulk transfer window |
//!
//! The firmware crate (`apps/esp-tool`) supplies the pins, the GATT peripheral, the USB console and
//! the main loop.
#![no_std]

pub mod flash;
pub mod mailbox;
pub mod router;
#[cfg(any(test, feature = "sim"))]
pub mod sim;
pub mod svc;
pub mod swd;
pub mod target;

#[cfg(test)]
mod tests;
