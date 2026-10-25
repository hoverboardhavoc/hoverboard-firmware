//! The hardware-free half of the ESP32-C3 board tool (`specs/esp-tool.md`).
//!
//! | Module | Owns |
//! | --- | --- |
//! | [`swd`] | the transfers, retries, connect, memory access and the counters, over [`swd::SwdPins`] |
//! | [`target`] | identify, armed check, disarm, halt, resume, reset, debug hold, release |
//! | [`flash`] | page erase, ascending program, error checks, verify, clearing protection |
//! | [`mailbox`] | the mailbox session, `MemAp` on [`swd`] |
//! | [`router`] | the L2 stream decoders on the BLE, USB and TCP bytes |
//! | [`node`] | the ESP as an L3 node: the walk responder, the forwarding table, `node.class`, its store, dispatch of the frames addressed to it |
//! | [`svc`] | the ESP commands and the bulk transfer window |
//! | [`update`] | the ESP's own slot update: the transfer into the inactive slot, its checks, activation |
//!
//! The firmware crate (`apps/esp-tool`) supplies the pins, the GATT peripheral, the USB console,
//! the Wi-Fi station with its TCP console and mDNS name, and the main loop.
#![no_std]

pub mod flash;
pub mod mailbox;
pub mod node;
pub mod router;
#[cfg(any(test, feature = "sim"))]
pub mod sim;
pub mod svc;
pub mod swd;
pub mod target;
pub mod update;

#[cfg(test)]
mod tests;
