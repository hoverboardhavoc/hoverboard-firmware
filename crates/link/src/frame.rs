//! Frame layout, encode, and decode.
//!
//! Wire format (all multi-byte fields little-endian):
//!
//! | offset | field    | meaning                                        |
//! |--------|----------|------------------------------------------------|
//! | 0      | `SOF`    | start of frame, 0x5A                            |
//! | 1      | `ver`    | protocol version, 1                             |
//! | 2      | `opcode` | see `opcode::Opcode`                            |
//! | 3      | `src`    | source node id                                  |
//! | 4      | `dst`    | destination node id (0xFF = broadcast)          |
//! | 5      | `len`    | payload length in bytes (0..=255)               |
//! | 6      | payload  | exactly `len` bytes                             |
//! | 6+len  | `crc`    | CRC-16/MODBUS over bytes 0..(6+len), LE         |
//!
//! 8 bytes of overhead. The CRC covers the header AND the payload (bytes `0..6+len`).

use crate::crc16;
use crate::opcode::Opcode;

/// Start-of-frame marker.
pub const SOF: u8 = 0x5A;
/// Protocol version this build speaks.
pub const PROTO_VER: u8 = 1;
/// Fixed header length: SOF, ver, opcode, src, dst, len.
pub const HEADER_LEN: usize = 6;
/// Trailing CRC length.
pub const CRC_LEN: usize = 2;
/// Maximum payload (the `len` field is a single byte).
pub const MAX_PAYLOAD: usize = 255;
/// Maximum total frame length on the wire.
pub const MAX_FRAME: usize = HEADER_LEN + MAX_PAYLOAD + CRC_LEN;
/// Broadcast destination address.
pub const BROADCAST: u8 = 0xFF;

/// The fixed 6-byte header, parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeader {
    pub ver: u8,
    pub opcode: Opcode,
    pub src: u8,
    pub dst: u8,
    pub len: u8,
}

/// A decoded frame: the header plus a borrowed slice of the payload bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodedFrame<'a> {
    pub header: FrameHeader,
    pub payload: &'a [u8],
}

/// Reasons `encode` can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// `payload.len()` exceeds `MAX_PAYLOAD`.
    PayloadTooLong,
    /// The supplied `out` buffer is smaller than the encoded frame.
    OutTooSmall,
}

/// Reasons `decode` can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Byte 0 is not `SOF`.
    BadSof,
    /// `ver` is not `PROTO_VER`.
    BadVersion,
    /// Fewer than a full header (or full declared frame) of bytes.
    TooShort,
    /// The byte count does not match the declared `len`.
    LenMismatch,
    /// The trailing CRC does not match the computed CRC.
    CrcMismatch,
}

/// Encode `hdr` + `payload` into `out`, appending a little-endian CRC-16/MODBUS over the header and
/// payload. Returns the total encoded length. `hdr.ver` and `hdr.len` are taken from `hdr` but the
/// SOF and the actual payload length are authoritative: the written `len` byte equals
/// `payload.len()`.
pub fn encode(hdr: &FrameHeader, payload: &[u8], out: &mut [u8]) -> Result<usize, EncodeError> {
    let len = payload.len();
    if len > MAX_PAYLOAD {
        return Err(EncodeError::PayloadTooLong);
    }
    let total = HEADER_LEN + len + CRC_LEN;
    if out.len() < total {
        return Err(EncodeError::OutTooSmall);
    }

    out[0] = SOF;
    out[1] = hdr.ver;
    out[2] = hdr.opcode.to_u8();
    out[3] = hdr.src;
    out[4] = hdr.dst;
    out[5] = len as u8;
    out[HEADER_LEN..HEADER_LEN + len].copy_from_slice(payload);

    let crc = crc16::modbus(&out[..HEADER_LEN + len]);
    out[HEADER_LEN + len] = (crc & 0x00FF) as u8;
    out[HEADER_LEN + len + 1] = (crc >> 8) as u8;

    Ok(total)
}

/// Decode a single frame from the front of `bytes`.
///
/// Validates SOF, version, that enough bytes are present for the declared `len`, and the trailing
/// CRC. Trailing bytes beyond the declared frame are not consumed and not an error here (the framer
/// handles coalesced frames); only the first `6 + len + 2` bytes are inspected.
pub fn decode(bytes: &[u8]) -> Result<DecodedFrame<'_>, DecodeError> {
    if bytes.len() < HEADER_LEN {
        return Err(DecodeError::TooShort);
    }
    if bytes[0] != SOF {
        return Err(DecodeError::BadSof);
    }
    if bytes[1] != PROTO_VER {
        return Err(DecodeError::BadVersion);
    }
    let len = bytes[5] as usize;
    let total = HEADER_LEN + len + CRC_LEN;
    if bytes.len() < total {
        return Err(DecodeError::LenMismatch);
    }

    let crc_calc = crc16::modbus(&bytes[..HEADER_LEN + len]);
    let crc_wire = u16::from_le_bytes([bytes[HEADER_LEN + len], bytes[HEADER_LEN + len + 1]]);
    if crc_calc != crc_wire {
        return Err(DecodeError::CrcMismatch);
    }

    let header = FrameHeader {
        ver: bytes[1],
        opcode: Opcode::from_u8(bytes[2]),
        src: bytes[3],
        dst: bytes[4],
        len: bytes[5],
    };
    Ok(DecodedFrame {
        header,
        payload: &bytes[HEADER_LEN..HEADER_LEN + len],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hdr(opcode: Opcode, len: u8) -> FrameHeader {
        FrameHeader { ver: PROTO_VER, opcode, src: 0x02, dst: 0x03, len }
    }

    #[test]
    fn round_trip_nonempty() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let mut out = [0u8; MAX_FRAME];
        let n = encode(&hdr(Opcode::CyclicState, 0), &payload, &mut out).unwrap();
        assert_eq!(n, HEADER_LEN + payload.len() + CRC_LEN);
        let f = decode(&out[..n]).unwrap();
        assert_eq!(f.header.opcode, Opcode::CyclicState);
        assert_eq!(f.header.src, 0x02);
        assert_eq!(f.header.dst, 0x03);
        assert_eq!(f.payload, &payload);
    }

    #[test]
    fn round_trip_empty() {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(&hdr(Opcode::Fault, 0), &[], &mut out).unwrap();
        assert_eq!(n, HEADER_LEN + CRC_LEN);
        let f = decode(&out[..n]).unwrap();
        assert_eq!(f.header.len, 0);
        assert_eq!(f.payload, &[] as &[u8]);
    }

    #[test]
    fn crc_reject_on_flip() {
        let payload = [1u8, 2, 3];
        let mut out = [0u8; MAX_FRAME];
        let n = encode(&hdr(Opcode::DriveCmd, 0), &payload, &mut out).unwrap();
        // Flip a payload byte, leave CRC untouched.
        out[HEADER_LEN] ^= 0xFF;
        assert_eq!(decode(&out[..n]), Err(DecodeError::CrcMismatch));
    }

    #[test]
    fn bad_sof_and_version() {
        let mut out = [0u8; MAX_FRAME];
        let n = encode(&hdr(Opcode::DriveCmd, 0), &[1, 2], &mut out).unwrap();
        let mut bad = out;
        bad[0] = 0x00;
        assert_eq!(decode(&bad[..n]), Err(DecodeError::BadSof));
        let mut bad2 = out;
        bad2[1] = 0x09;
        // Version mismatch is caught before CRC.
        assert_eq!(decode(&bad2[..n]), Err(DecodeError::BadVersion));
    }

    #[test]
    fn too_short_and_len_mismatch() {
        assert_eq!(decode(&[SOF, PROTO_VER, 0x10]), Err(DecodeError::TooShort));
        // Header says len=5 but not enough bytes follow.
        let buf = [SOF, PROTO_VER, 0x10, 0, 0, 5, 0, 0];
        assert_eq!(decode(&buf), Err(DecodeError::LenMismatch));
    }

    #[test]
    fn unknown_opcode_decodes() {
        let mut out = [0u8; MAX_FRAME];
        // Craft a frame with an unknown opcode byte by hand-building then fixing CRC via encode.
        let h = FrameHeader { ver: PROTO_VER, opcode: Opcode::Unknown(0x7E), src: 1, dst: 2, len: 0 };
        let n = encode(&h, &[9, 9], &mut out).unwrap();
        let f = decode(&out[..n]).unwrap();
        assert_eq!(f.header.opcode, Opcode::Unknown(0x7E));
    }
}
