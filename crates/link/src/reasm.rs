//! Fragmentation (TX) and reassembly (RX), transport-agnostic.
//!
//! Per `specs/l2.md` ("The L2 frame"): a packet is split into chunks, each prefixed with a
//! [`FragHdr`](crate::frag::FragHdr), and reassembled on the far side under the **atomic-or-discard**
//! rule. All fragments of a packet share one `PID`; `FRAG_IDX` runs 0,1,2,...; `MORE` is set on every
//! fragment except the last. A packet is delivered only when **every** fragment of the **same** packet
//! has arrived; a torn set (a dropped fragment, a `FRAG_IDX` skip, or fragments from two packets
//! interleaved) is discarded whole, and no partial packet is ever surfaced.

use heapless::Vec;

use crate::frag::{FragHdr, MAX_FRAGMENTS, MAX_PID};

/// Reassembly buffer bound: the FORMAT worst case (16 fragments x the 254-byte chunk a maximal
/// 255-byte frame capacity permits). Every shipped instance is far smaller (`specs/l2.md`,
/// "Transport instances": 127/95/15-byte chunks, and the firmware bounds delivered packets at
/// `PACKET` = 72), so real links pick a small `N`; this default only serves callers that do not care.
pub const MAX_PACKET: usize = MAX_FRAGMENTS * 254;

/// Reason [`fragment`] can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FragError {
    /// The packet needs more than [`MAX_FRAGMENTS`] fragments at this link's chunk capacity.
    PacketTooLarge,
    /// `chunk_cap` was 0 (a frame capacity of <= 1 leaves no room for a chunk).
    ZeroChunkCap,
}

/// Split `packet` into fragments at `chunk_cap` bytes per chunk, calling `emit(frag_hdr, chunk)` once
/// per fragment in order. All fragments carry `pid`; `FRAG_IDX` runs 0..; `MORE` is set on all but
/// the last. An empty packet yields exactly one fragment with an empty chunk (`MORE=0, FRAG_IDX=0`),
/// preserving the one-byte-of-overhead single-frame case.
pub fn fragment(
    packet: &[u8],
    chunk_cap: usize,
    pid: u8,
    mut emit: impl FnMut(FragHdr, &[u8]),
) -> Result<(), FragError> {
    if chunk_cap == 0 {
        return Err(FragError::ZeroChunkCap);
    }
    let pid = pid & MAX_PID;

    if packet.is_empty() {
        emit(
            FragHdr {
                more: false,
                pid,
                frag_idx: 0,
            },
            &[],
        );
        return Ok(());
    }

    let n_frags = packet.len().div_ceil(chunk_cap);
    if n_frags > MAX_FRAGMENTS {
        return Err(FragError::PacketTooLarge);
    }
    for (i, chunk) in packet.chunks(chunk_cap).enumerate() {
        let more = i + 1 < n_frags;
        emit(
            FragHdr {
                more,
                pid,
                frag_idx: i as u8,
            },
            chunk,
        );
    }
    Ok(())
}

/// Reassembles fragments into whole packets under the atomic-or-discard rule.
///
/// `N` is the reassembly-buffer bound, i.e. the largest packet this instance delivers. It is
/// per-instance (`specs/l2.md`, `mtu_hint`: the delivered-packet bound is the receiver's `N`,
/// distinct from the fragmentation bound): the firmware sizes every link's `N` to `PACKET` = 72
/// (the <= 64 B L3 PDUs + margin, inside the 8 KiB-SRAM budget); bench instances pick their own. It
/// defaults to the format-worst-case [`MAX_PACKET`] so callers that do not care keep a safe bound.
pub struct Reassembler<const N: usize = MAX_PACKET> {
    /// Whether a fragment set is currently being assembled.
    active: bool,
    /// The `PID` of the set in progress.
    pid: u8,
    /// The next expected `FRAG_IDX`.
    next_idx: u8,
    /// Accumulated chunk bytes of the set in progress.
    buf: Vec<u8, N>,
}

impl<const N: usize> Default for Reassembler<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Reassembler<N> {
    /// A fresh reassembler with no set in progress.
    pub const fn new() -> Reassembler<N> {
        Reassembler {
            active: false,
            pid: 0,
            next_idx: 0,
            buf: Vec::new(),
        }
    }

    /// Drop any set in progress and return to idle.
    pub fn reset(&mut self) {
        self.active = false;
        self.next_idx = 0;
        self.buf.clear();
    }

    /// Feed one L2 frame's `frag-hdr` byte and its chunk. Returns `Some(packet)` exactly when this
    /// fragment completes a packet (the borrowed slice is valid until the next call), otherwise
    /// `None` (more fragments expected, or the frame was discarded as torn/stray).
    pub fn push(&mut self, hdr_byte: u8, chunk: &[u8]) -> Option<&[u8]> {
        let h = FragHdr::decode(hdr_byte);

        if h.frag_idx == 0 {
            // FRAG_IDX 0 always starts a fresh set, discarding any set in progress (a new packet
            // arriving mid-reassembly torpedoes the old one: atomic-or-discard, the old is dropped).
            self.buf.clear();
            if self.buf.extend_from_slice(chunk).is_err() {
                self.reset();
                return None;
            }
            self.pid = h.pid;
            if h.more {
                self.active = true;
                self.next_idx = 1;
                return None;
            }
            // Single-fragment packet: complete immediately.
            self.active = false;
            self.next_idx = 0;
            return Some(&self.buf[..]);
        }

        // FRAG_IDX > 0: a continuation. Without an active set it is a stray fragment (we never saw
        // index 0), so drop it.
        if !self.active {
            return None;
        }
        // A different PID, or a skipped index, means the set for the PID in progress is torn: discard
        // it whole. The offending continuation cannot start a set (its index is > 0), so drop it too.
        if h.pid != self.pid || h.frag_idx != self.next_idx {
            self.reset();
            return None;
        }
        if self.buf.extend_from_slice(chunk).is_err() {
            self.reset();
            return None;
        }
        self.next_idx += 1;
        if h.more {
            None
        } else {
            self.active = false;
            self.next_idx = 0;
            Some(&self.buf[..])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Run `packet` through fragment() then push the fragments back through a Reassembler, returning
    // the reassembled packet (or None if nothing completed).
    fn round_trip(packet: &[u8], chunk_cap: usize, pid: u8) -> Option<std::vec::Vec<u8>> {
        let mut frames: std::vec::Vec<(u8, std::vec::Vec<u8>)> = std::vec::Vec::new();
        fragment(packet, chunk_cap, pid, |h, c| {
            frames.push((h.encode(), c.to_vec()))
        })
        .unwrap();
        let mut r: Reassembler = Reassembler::new();
        let mut out = None;
        for (hdr, chunk) in &frames {
            if let Some(p) = r.push(*hdr, chunk) {
                out = Some(p.to_vec());
            }
        }
        out
    }

    #[test]
    fn single_frame_one_byte_overhead() {
        let packet = [1u8, 2, 3, 4];
        let mut frames: std::vec::Vec<(FragHdr, std::vec::Vec<u8>)> = std::vec::Vec::new();
        fragment(&packet, 19, 0, |h, c| frames.push((h, c.to_vec()))).unwrap();
        // Fits one fragment: MORE=0, FRAG_IDX=0, chunk = whole packet, exactly one byte of overhead.
        assert_eq!(frames.len(), 1);
        assert_eq!(
            frames[0].0,
            FragHdr {
                more: false,
                pid: 0,
                frag_idx: 0
            }
        );
        assert_eq!(&frames[0].1[..], &packet);
        assert_eq!(round_trip(&packet, 19, 0).as_deref(), Some(&packet[..]));
    }

    #[test]
    fn empty_packet_round_trips() {
        assert_eq!(round_trip(&[], 19, 3).as_deref(), Some(&[][..]));
    }

    #[test]
    fn larger_than_capacity_splits_and_reassembles() {
        // 50 bytes over a 19-byte chunk capacity -> 3 fragments.
        let packet: std::vec::Vec<u8> = (0..50u8).collect();
        let mut frames: std::vec::Vec<FragHdr> = std::vec::Vec::new();
        fragment(&packet, 19, 5, |h, _| frames.push(h)).unwrap();
        assert_eq!(frames.len(), 3);
        assert_eq!(
            frames[0],
            FragHdr {
                more: true,
                pid: 5,
                frag_idx: 0
            }
        );
        assert_eq!(
            frames[1],
            FragHdr {
                more: true,
                pid: 5,
                frag_idx: 1
            }
        );
        assert_eq!(
            frames[2],
            FragHdr {
                more: false,
                pid: 5,
                frag_idx: 2
            }
        );
        assert_eq!(round_trip(&packet, 19, 5).as_deref(), Some(&packet[..]));
    }

    #[test]
    fn dropped_middle_fragment_discards_whole_packet() {
        // Build a 3-fragment packet, then drop the middle one. The index jumps 0 -> 2, a skip, so the
        // whole set is discarded (atomic): nothing is delivered.
        let packet: std::vec::Vec<u8> = (0..50u8).collect();
        let mut frames: std::vec::Vec<(u8, std::vec::Vec<u8>)> = std::vec::Vec::new();
        fragment(&packet, 19, 1, |h, c| frames.push((h.encode(), c.to_vec()))).unwrap();
        let mut r: Reassembler = Reassembler::new();
        let mut out = None;
        // Push frag 0 and frag 2, skipping frag 1.
        for (hdr, chunk) in [&frames[0], &frames[2]] {
            if let Some(p) = r.push(*hdr, chunk) {
                out = Some(p.to_vec());
            }
        }
        assert_eq!(out, None);
    }

    #[test]
    fn frag_idx_skip_rejected() {
        // frag 0 (MORE) then frag 2 (MORE=0): a skip past index 1. Discarded, nothing delivered.
        let mut r: Reassembler = Reassembler::new();
        let f0 = FragHdr {
            more: true,
            pid: 2,
            frag_idx: 0,
        }
        .encode();
        let f2 = FragHdr {
            more: false,
            pid: 2,
            frag_idx: 2,
        }
        .encode();
        assert_eq!(r.push(f0, &[1, 2, 3]), None);
        assert_eq!(r.push(f2, &[7, 8, 9]), None);
    }

    #[test]
    fn interleaved_different_pid_does_not_corrupt() {
        // Reassembling PID=1; a continuation arrives with PID=2 at the expected index. Without the
        // PID check it would be appended and falsely complete a corrupt packet. The PID mismatch must
        // discard the set instead: nothing delivered.
        let mut r: Reassembler = Reassembler::new();
        let a0 = FragHdr {
            more: true,
            pid: 1,
            frag_idx: 0,
        }
        .encode();
        let b1 = FragHdr {
            more: false,
            pid: 2,
            frag_idx: 1,
        }
        .encode();
        assert_eq!(r.push(a0, &[0xAA, 0xAA]), None);
        assert_eq!(r.push(b1, &[0xBB, 0xBB]), None);
    }

    #[test]
    fn new_packet_mid_reassembly_supersedes_old() {
        // PID=1 starts (incomplete), then PID=2 begins at FRAG_IDX 0: the old set is dropped and the
        // new single-fragment packet is delivered cleanly.
        let mut r: Reassembler = Reassembler::new();
        let a0 = FragHdr {
            more: true,
            pid: 1,
            frag_idx: 0,
        }
        .encode();
        let b0 = FragHdr {
            more: false,
            pid: 2,
            frag_idx: 0,
        }
        .encode();
        assert_eq!(r.push(a0, &[0xAA, 0xAA]), None);
        assert_eq!(
            r.push(b0, &[0xBB, 0xCC]).map(|p| p.to_vec()),
            Some(std::vec![0xBB, 0xCC])
        );
    }

    #[test]
    fn stray_continuation_without_start_ignored() {
        // A FRAG_IDX > 0 with no set in progress is a stray; ignore it (no panic, nothing delivered).
        let mut r: Reassembler = Reassembler::new();
        let f3 = FragHdr {
            more: false,
            pid: 0,
            frag_idx: 3,
        }
        .encode();
        assert_eq!(r.push(f3, &[1, 2]), None);
    }

    #[test]
    fn full_16_fragment_packet() {
        // The maximum: 16 fragments of 19 bytes = 304-byte packet over the BLE chunk capacity.
        let packet: std::vec::Vec<u8> = (0..(16 * 19)).map(|i| i as u8).collect();
        let mut count = 0;
        fragment(&packet, 19, 0, |_, _| count += 1).unwrap();
        assert_eq!(count, 16);
        assert_eq!(round_trip(&packet, 19, 0).as_deref(), Some(&packet[..]));
    }

    #[test]
    fn over_16_fragments_rejected() {
        // 17 fragments' worth at a 19-byte chunk capacity exceeds the 16-fragment bound.
        let packet: std::vec::Vec<u8> = (0..(16 * 19 + 1)).map(|i| i as u8).collect();
        let r = fragment(&packet, 19, 0, |_, _| {});
        assert_eq!(r, Err(FragError::PacketTooLarge));
    }

    #[test]
    fn zero_chunk_cap_rejected() {
        assert_eq!(
            fragment(&[1, 2], 0, 0, |_, _| {}),
            Err(FragError::ZeroChunkCap)
        );
    }
}
