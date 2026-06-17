//! Standalone link layer: frame codec + stream framer.
//!
//! One frame format (`SOF 0x5A | ver | opcode | src | dst | len | payload | CRC-16/MODBUS`,
//! little-endian) shared by the inter-board USART link and the BLE app link. This crate knows the
//! frame, the opcodes, the payloads, and the stream framer, with zero dependency on `runtime-hal`
//! or any transport. It is the single piece both transports reuse.
//!
//! `no_std`; host tests in `#[cfg(test)]` modules link `std` via the host target.

#![no_std]

pub mod crc16;
pub mod frame;
pub mod framer;
pub mod item;
pub mod opcode;
pub mod payload;
pub mod transport;

// Public re-exports for the common types.
pub use frame::{
    decode, encode, DecodeError, DecodedFrame, EncodeError, FrameHeader, BROADCAST, CRC_LEN,
    HEADER_LEN, MAX_FRAME, MAX_PAYLOAD, PROTO_VER, SOF,
};
pub use framer::StreamFramer;
pub use item::{DataItem, ItemSet};
pub use opcode::Opcode;
pub use payload::{
    ConfigRead, ConfigResp, ConfigWrite, CyclicState, DriveCmd, Event, Fault, Inputs, NodeHello,
    PayloadError, Telemetry,
};
