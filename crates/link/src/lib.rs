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

pub use frag::{FragHdr, MAX_FRAGMENTS, MAX_FRAG_IDX, MAX_PID};
pub use framer::{encode as encode_stream_frame, FrameError, StreamFramer, SOF};
pub use link::{Link, SendError, Transport};
pub use reasm::{fragment, FragError, Reassembler};
pub use serial::SerialTransport;
