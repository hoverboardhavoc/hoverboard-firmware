//! L2: the per-link data-link layer (framing, fragmentation, integrity).
//!
//! Per `specs/l2.md`: carry one **opaque packet** across **one link**, hiding that link's MTU.
//! Addressing/routing (L3), reliability/end-to-end integrity (L4), and opcodes (L7) live above. This
//! crate is the HAL-free host core (Tier 1 of the spec's test plan): the `frag-hdr` frame codec, the
//! byte-stream [`StreamFramer`], and fragmentation/reassembly, all pure over byte buffers, depending
//! only on `base` (for `crc16`). The transport adapters (BLE, inter-board UART) wire the HAL last.
//!
//! Layout:
//! - [`frag`]   the one-byte fragmentation header (`frag-hdr`).
//! - [`framer`] the byte-stream transport frame (SOF/len/CRC) and its resyncing decoder.
//! - [`reasm`]  fragmentation (TX) and atomic-or-discard reassembly (RX).
//! - [`link`]   the [`Link`] service over a [`Transport`], parameterized per link.
//! - [`serial`] a [`Transport`] over any `embedded-io` serial (the shared UART / SWD-mailbox shim).

#![no_std]
// The host test harness needs std (collections, Vec, formatting); the crate itself is no_std.
#[cfg(test)]
extern crate std;

pub mod frag;
pub mod framer;
pub mod link;
pub mod reasm;
pub mod serial;

/// The inter-board UART link's baud, pinned by `specs/l2.md`'s transport-instance table. One
/// owner for the fleet's number, the `ble::at::BAUD` pattern: the crate that implements the wire
/// contract carries the contract's constant. The firmware and the Tier-2 bench both read it from
/// here. 460800 since 2026-07-18 (was the M1-proven 115200): the 250 Hz cyclic emission is a
/// blocking polled TX, and its ~2.5 ms per frame at 115200 broke the 4 ms control budget
/// (`specs/l2.md`, "Baud raised"); ~0.6 ms at 460800. BRR error +0.16%/board at the 36 MHz APB1
/// input, inter-board skew IRC8M-dominated (0.37% measured), inside 8N1 margin.
pub const INTER_BOARD_BAUD: u32 = 460_800;

pub use frag::{FragHdr, MAX_FRAGMENTS, MAX_FRAG_IDX, MAX_PID};
pub use framer::{encode as encode_stream_frame, FrameError, StreamFramer, SOF};
pub use link::{Link, SendError, Transport};
pub use reasm::{fragment, FragError, Reassembler};
pub use serial::SerialTransport;
