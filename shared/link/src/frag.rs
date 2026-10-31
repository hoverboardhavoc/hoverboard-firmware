//! The one-byte L2 fragmentation header (`frag-hdr`).
//!
//! Per `specs/l2.md` ("The L2 frame"), this is the **only** metadata L2 adds and it is identical on
//! every transport. Bit layout:
//!
//! ```text
//! bit 7      MORE       1 = more fragments of this packet follow, 0 = last (or only) fragment
//! bits 6..4  PID        packet id, 0..7, increments per packet on this link (wraps); groups a set
//! bits 3..0  FRAG_IDX   fragment index within the packet, 0..15
//! ```
//!
//! It carries **no addressing** (that is L3); the chunk it prefixes is an opaque L3 packet slice.

/// `MORE` bit (bit 7): more fragments of this packet follow.
pub const MORE_BIT: u8 = 0b1000_0000;
/// Shift to the `PID` field (bits 6..4).
const PID_SHIFT: u8 = 4;
/// `PID` is 3 bits, so the largest packet id is 7; also the mask for wrapping.
pub const MAX_PID: u8 = 0b0000_0111;
/// `FRAG_IDX` is 4 bits, so the largest fragment index is 15.
pub const MAX_FRAG_IDX: u8 = 0b0000_1111;
/// A packet is bounded to 16 fragments (`FRAG_IDX` 0..15).
pub const MAX_FRAGMENTS: usize = 16;

/// A parsed fragmentation header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FragHdr {
    /// `MORE`: true if more fragments of this packet follow.
    pub more: bool,
    /// `PID`: packet id, 0..=7.
    pub pid: u8,
    /// `FRAG_IDX`: fragment index within the packet, 0..=15.
    pub frag_idx: u8,
}

impl FragHdr {
    /// Pack into the single wire byte. `pid` and `frag_idx` are masked to their fields, so an
    /// out-of-range value cannot bleed into a neighbouring field.
    #[inline]
    pub const fn encode(&self) -> u8 {
        let more = if self.more { MORE_BIT } else { 0 };
        more | ((self.pid & MAX_PID) << PID_SHIFT) | (self.frag_idx & MAX_FRAG_IDX)
    }

    /// Unpack the single wire byte. Every byte is a valid header (all three fields are fixed-width).
    #[inline]
    pub const fn decode(b: u8) -> FragHdr {
        FragHdr {
            more: (b & MORE_BIT) != 0,
            pid: (b >> PID_SHIFT) & MAX_PID,
            frag_idx: b & MAX_FRAG_IDX,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_frame_header_is_zero() {
        // The common case: MORE=0, PID=0, FRAG_IDX=0 -> the all-zero byte.
        let h = FragHdr {
            more: false,
            pid: 0,
            frag_idx: 0,
        };
        assert_eq!(h.encode(), 0x00);
    }

    #[test]
    fn bit_positions() {
        // MORE in bit 7.
        assert_eq!(
            FragHdr {
                more: true,
                pid: 0,
                frag_idx: 0
            }
            .encode(),
            0b1000_0000
        );
        // PID in bits 6..4.
        assert_eq!(
            FragHdr {
                more: false,
                pid: 0b101,
                frag_idx: 0
            }
            .encode(),
            0b0101_0000
        );
        // FRAG_IDX in bits 3..0.
        assert_eq!(
            FragHdr {
                more: false,
                pid: 0,
                frag_idx: 0b1011
            }
            .encode(),
            0b0000_1011
        );
        // All three together.
        assert_eq!(
            FragHdr {
                more: true,
                pid: 7,
                frag_idx: 15
            }
            .encode(),
            0b1111_1111
        );
    }

    #[test]
    fn round_trip_all_bytes() {
        for b in 0u8..=255 {
            assert_eq!(
                FragHdr::decode(b).encode(),
                b,
                "byte {b:#04x} did not round-trip"
            );
        }
    }

    #[test]
    fn fields_are_masked_on_encode() {
        // Over-range pid/frag_idx are confined to their fields, never corrupting a neighbour.
        let h = FragHdr {
            more: false,
            pid: 0xFF,
            frag_idx: 0xFF,
        };
        assert_eq!(h.encode(), 0b0111_1111);
    }
}
