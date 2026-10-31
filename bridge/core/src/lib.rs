//! The hardware-free half of the hoverboard bridge (`specs/hoverboard-bridge.md`).
//!
//! | Module | Owns |
//! | --- | --- |
//! | [`swd`] | the transfers, retries, connect, memory access and the counters, over [`swd::SwdPins`] |
//! | [`target`] | identify, armed check, disarm, halt, step, resume, reset, debug hold, release, the flash patch and DWT comparators |
//! | [`gdb`] | the GDB server: the `gdbstub` target on the board operations, the monitor commands, the armed-breakpoint rule |
//! | [`flash`] | page erase, programming through the RAM loader, verify, clearing protection |
//! | [`loader`] | the RAM loader: its code, its SRAM layout, the bridge's side of its ring protocol |
//! | [`mailbox`] | the mailbox session, `MemAp` on [`swd`] |
//! | [`router`] | the L2 stream decoders on the BLE, USB and TCP bytes |
//! | [`node`] | the bridge as an L3 node: the walk responder, the forwarding table, `node.class`, its store, dispatch of the frames addressed to it |
//! | [`svc`] | the ESP commands and the bulk transfer window |
//! | [`update`] | the bridge's own slot update: the transfer into the inactive slot, its checks, activation |
//!
//! The firmware crate (`apps/hoverboard-bridge`) supplies the pins, the GATT peripheral, the USB console,
//! the Wi-Fi station with its TCP console, GDB listener and mDNS name, the heap the GDB session's
//! buffers come from, and the main loop.
#![no_std]

pub mod flash;
pub mod gdb;
pub mod loader;
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
