//! The L3 PDU codec (`specs/l3.md`, "The L3 PDU"): one PDU rides inside one L2 packet.
//!
//! ```text
//! [ opcode : 1 ][ src : 1 ][ dst : 1 ][ payload : ... ]      one L3 PDU == one L2 packet
//! ```
//!
//! L2 owns framing, length, and integrity, so the PDU has **no SOF, no len, no CRC, no version byte**.
//! `0x00` and `0xFF` are never valid opcodes; the protocol version is learned once per node from
//! `NODE_HELLO.proto_ver`; an **unknown opcode is ignored** (not an error) once L2 has validated the
//! frame, so later specs add opcodes without breaking a base-only build. This module decodes the
//! header and the opcode space; deciding to ignore an unknown opcode is the node's job (the codec
//! decodes any structurally valid PDU).

/// `dst = 0xFF` is broadcast.
pub const BROADCAST: u8 = 0xFF;
/// `0x00` means "no address yet" (an unassigned board's `src`, or "the one peer" on a point-to-point
/// link). It is never a routable destination and is never learned into a routing table.
pub const NO_ADDRESS: u8 = 0x00;

/// The fixed PDU header length (`opcode` + `src` + `dst`).
pub const HEADER_LEN: usize = 3;

/// Is `a` a board address (`0x01..=0x7F`, persistent, assigned once)?
pub fn is_board(a: u8) -> bool {
    (0x01..=0x7F).contains(&a)
}

/// Is `a` a controller / guest address (`0x80..=0xFE`, transient, session-only)?
pub fn is_controller(a: u8) -> bool {
    (0x80..=0xFE).contains(&a)
}

/// Is `a` a unicast, routable, learnable address (`0x01..=0xFE`: a board or a guest)? `0x00`
/// (no-address) and `0xFF` (broadcast) are excluded - the two the forwarder must never learn.
pub fn is_unicast(a: u8) -> bool {
    is_board(a) || is_controller(a)
}

/// The L3 opcodes this layer interprets (`specs/l3.md`, the opcode table). An opcode outside this set
/// is **not** an error: L3 forwards it by `dst` without interpreting it (the L7 control/telemetry
/// payloads, `0x10..0x2F` / `0x40..0x6F`), or ignores it if addressed to self.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Opcode {
    /// First contact, merged identity + guest-address grant.
    NodeHello = 0x01,
    /// "Map your links": probe each port and report.
    ProbePorts = 0x02,
    /// The `PROBE_PORTS` reply: per-port kind + neighbour state.
    Ports = 0x03,
    /// Hand out + persist an address (directly, or via a relay's `egress_port`).
    Assign = 0x06,
    /// The board confirms it persisted the address.
    AssignAck = 0x07,
    /// Read a config field from an addressed board.
    ConfigRead = 0x30,
    /// Write a config field (provisioning).
    ConfigWrite = 0x31,
    /// Read result / write ack.
    ConfigResp = 0x32,
    /// Write a whole board definition in a few PDUs.
    ConfigWriteMulti = 0x33,
}

impl Opcode {
    /// The opcode's wire byte.
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Decode a known L3 opcode, or `None` for one L3 does not interpret (forward-by-`dst` / ignore).
    /// `0x00` / `0xFF` are not opcodes at all and never reach here (the codec rejects them).
    pub const fn from_u8(b: u8) -> Option<Opcode> {
        match b {
            0x01 => Some(Opcode::NodeHello),
            0x02 => Some(Opcode::ProbePorts),
            0x03 => Some(Opcode::Ports),
            0x06 => Some(Opcode::Assign),
            0x07 => Some(Opcode::AssignAck),
            0x30 => Some(Opcode::ConfigRead),
            0x31 => Some(Opcode::ConfigWrite),
            0x32 => Some(Opcode::ConfigResp),
            0x33 => Some(Opcode::ConfigWriteMulti),
            _ => None,
        }
    }
}

/// Why [`Pdu::decode`] / [`Pdu::encode`] failed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PduError {
    /// Fewer than [`HEADER_LEN`] bytes: not even a full header.
    TooShort,
    /// The opcode byte is `0x00` or `0xFF`, which are never valid opcodes.
    InvalidOpcode,
    /// The `out` buffer is smaller than the encoded PDU.
    OutTooSmall,
}

/// A borrowed view of one decoded PDU: the header fields plus a borrow of the payload bytes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Pdu<'a> {
    /// The raw opcode byte (`0x01..=0xFE`). Map with [`Opcode::from_u8`]; an unknown one is forwarded
    /// by `dst`, not an error.
    pub opcode: u8,
    /// The source node address.
    pub src: u8,
    /// The destination node address (`0xFF` = broadcast).
    pub dst: u8,
    /// The opaque L7 payload (L3 never interprets it; the opcode handler does).
    pub payload: &'a [u8],
}

impl<'a> Pdu<'a> {
    /// Build a PDU from its parts (`opcode` must be `0x01..=0xFE`).
    pub fn new(opcode: u8, src: u8, dst: u8, payload: &'a [u8]) -> Result<Self, PduError> {
        if opcode == 0x00 || opcode == 0xFF {
            return Err(PduError::InvalidOpcode);
        }
        Ok(Pdu {
            opcode,
            src,
            dst,
            payload,
        })
    }

    /// Build a PDU from a known [`Opcode`] (infallible: a known opcode is always valid).
    pub fn from_op(opcode: Opcode, src: u8, dst: u8, payload: &'a [u8]) -> Self {
        Pdu {
            opcode: opcode.to_u8(),
            src,
            dst,
            payload,
        }
    }

    /// Decode a PDU from one L2 packet. Rejects a too-short buffer and the `0x00`/`0xFF` opcodes; any
    /// other opcode decodes fine (an unknown one is the caller's to ignore/forward).
    pub fn decode(buf: &'a [u8]) -> Result<Self, PduError> {
        if buf.len() < HEADER_LEN {
            return Err(PduError::TooShort);
        }
        let opcode = buf[0];
        if opcode == 0x00 || opcode == 0xFF {
            return Err(PduError::InvalidOpcode);
        }
        Ok(Pdu {
            opcode,
            src: buf[1],
            dst: buf[2],
            payload: &buf[HEADER_LEN..],
        })
    }

    /// Encode the PDU into `out`, returning the encoded length (`HEADER_LEN + payload`).
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PduError> {
        if self.opcode == 0x00 || self.opcode == 0xFF {
            return Err(PduError::InvalidOpcode);
        }
        let total = HEADER_LEN + self.payload.len();
        if out.len() < total {
            return Err(PduError::OutTooSmall);
        }
        out[0] = self.opcode;
        out[1] = self.src;
        out[2] = self.dst;
        out[HEADER_LEN..total].copy_from_slice(self.payload);
        Ok(total)
    }

    /// The known L3 opcode this PDU carries, or `None` if L3 does not interpret it.
    pub fn known(&self) -> Option<Opcode> {
        Opcode::from_u8(self.opcode)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_every_known_opcode() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        for op in [
            Opcode::NodeHello,
            Opcode::ProbePorts,
            Opcode::Ports,
            Opcode::Assign,
            Opcode::AssignAck,
            Opcode::ConfigRead,
            Opcode::ConfigWrite,
            Opcode::ConfigResp,
            Opcode::ConfigWriteMulti,
        ] {
            let pdu = Pdu::from_op(op, 0x01, 0x80, &payload);
            let mut buf = [0u8; 16];
            let n = pdu.encode(&mut buf).unwrap();
            assert_eq!(n, HEADER_LEN + payload.len());
            let got = Pdu::decode(&buf[..n]).unwrap();
            assert_eq!(got, pdu);
            assert_eq!(got.known(), Some(op));
            assert_eq!(got.opcode, op.to_u8());
        }
    }

    #[test]
    fn empty_payload_round_trips() {
        let pdu = Pdu::from_op(Opcode::ProbePorts, 0x80, 0x01, &[]);
        let mut buf = [0u8; 8];
        let n = pdu.encode(&mut buf).unwrap();
        assert_eq!(n, HEADER_LEN);
        let got = Pdu::decode(&buf[..n]).unwrap();
        assert_eq!(got.payload, &[] as &[u8]);
        assert_eq!(got, pdu);
    }

    #[test]
    fn unknown_opcode_decodes_to_ignore_not_error() {
        // An opcode L3 does not interpret (an L7 DRIVE in 0x10..0x2F) decodes fine; `known()` is None,
        // which is the node's signal to forward-by-dst / ignore, NOT a decode error.
        let buf = [0x10u8, 0x02, 0x03, 0x99];
        let got = Pdu::decode(&buf).unwrap();
        assert_eq!(got.opcode, 0x10);
        assert_eq!(got.known(), None);
        assert_eq!(got.src, 0x02);
        assert_eq!(got.dst, 0x03);
        assert_eq!(got.payload, &[0x99]);
    }

    #[test]
    fn opcode_0x00_and_0xff_are_rejected() {
        assert_eq!(Pdu::decode(&[0x00, 1, 2]), Err(PduError::InvalidOpcode));
        assert_eq!(Pdu::decode(&[0xFF, 1, 2]), Err(PduError::InvalidOpcode));
        assert_eq!(Pdu::new(0x00, 1, 2, &[]), Err(PduError::InvalidOpcode));
        assert_eq!(Pdu::new(0xFF, 1, 2, &[]), Err(PduError::InvalidOpcode));
        // Encode guards too.
        let pdu = Pdu {
            opcode: 0xFF,
            src: 1,
            dst: 2,
            payload: &[],
        };
        let mut out = [0u8; 8];
        assert_eq!(pdu.encode(&mut out), Err(PduError::InvalidOpcode));
    }

    #[test]
    fn too_short_buffer_is_rejected() {
        assert_eq!(Pdu::decode(&[]), Err(PduError::TooShort));
        assert_eq!(Pdu::decode(&[0x01]), Err(PduError::TooShort));
        assert_eq!(Pdu::decode(&[0x01, 0x02]), Err(PduError::TooShort));
    }

    #[test]
    fn encode_rejects_small_out() {
        let pdu = Pdu::from_op(Opcode::Assign, 0x80, 0x01, &[0xAA, 0xBB]);
        let mut tiny = [0u8; 4]; // need 5
        assert_eq!(pdu.encode(&mut tiny), Err(PduError::OutTooSmall));
    }

    #[test]
    fn addressing_range_helpers() {
        assert_eq!(NO_ADDRESS, 0x00);
        assert_eq!(BROADCAST, 0xFF);
        // boards
        assert!(is_board(0x01) && is_board(0x7F));
        assert!(!is_board(0x00) && !is_board(0x80));
        // controllers / guests
        assert!(is_controller(0x80) && is_controller(0xFE));
        assert!(!is_controller(0x7F) && !is_controller(0xFF));
        // learnable unicast = boards + guests, never 0x00 / 0xFF
        assert!(is_unicast(0x01) && is_unicast(0xFE));
        assert!(!is_unicast(NO_ADDRESS) && !is_unicast(BROADCAST));
    }
}
