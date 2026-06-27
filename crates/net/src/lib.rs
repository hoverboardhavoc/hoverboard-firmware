//! L3: the network layer (`specs/l3.md`) on top of L2 (`crates/link`).
//!
//! It adds node **addressing** (`src`/`dst`), the **controller-driven discovery + address-assignment
//! walk**, **source-learned multi-hop forwarding**, the **two delivery classes**, and `CONFIG_*` (the
//! wire face of the `store`). It carries the L7 control/telemetry payloads but **never interprets
//! them** - L3 is addressing and routing only.
//!
//! HAL-free: the board-side logic runs over L2's `send` / `poll_recv` (`crates/link`'s `Link`) from
//! the cooperative scheduler; the firmware wires it to real links. This crate is the Tier-1 host core,
//! tested over an in-memory mesh of mock L2 links.
//!
//! Layout (built smallest-first, per the spec's Test plan):
//! - [`pdu`]     the `[opcode][src][dst][payload]` PDU codec + addressing helpers.
//! - [`forward`] the source-learned forwarder (the `address -> port` table, unknown-`dst` flood,
//!   split-horizon).
//! - [`walk`]    the controller-driven discovery + address-assignment walk (board [`Responder`] +
//!   host [`Controller`]).

#![no_std]
// The host test harness needs std (mesh collections, Vec); the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod forward;
pub mod pdu;
pub mod walk;

pub use forward::{Forwarder, RoutingTable, NO_PORT};
pub use pdu::{is_board, is_controller, is_unicast, Opcode, Pdu, PduError, BROADCAST, NO_ADDRESS};
pub use walk::{Controller, Responder};
