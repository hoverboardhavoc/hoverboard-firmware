//! Stream framer with a resync state machine.
//!
//! Consumes arbitrary split/coalesced byte chunks and emits zero or more complete, CRC-valid
//! frames. There is NO idle-line assumption: the framer works identically whether bytes arrive one
//! per idle burst (inter-board) or chopped/merged by a BLE bridge. This single framer is what
//! unifies the two transports.
//!
//! Resync rule: on a CRC failure or any structural error, do NOT discard the whole buffer. Re-scan
//! from the byte AFTER the presumed SOF, so a false 0x5A inside garbage still resyncs to the next
//! real frame.

use heapless::Vec;

use crate::frame::{self, DecodedFrame, HEADER_LEN, MAX_FRAME, PROTO_VER, SOF};

/// A resyncing stream framer over a bounded `heapless` buffer.
pub struct StreamFramer {
    /// Bytes accumulated since the current candidate SOF (buf[0] is always the candidate SOF once
    /// non-empty).
    buf: Vec<u8, MAX_FRAME>,
}

impl Default for StreamFramer {
    fn default() -> Self {
        Self::new()
    }
}

impl StreamFramer {
    /// A fresh framer in the hunt state.
    pub const fn new() -> StreamFramer {
        StreamFramer { buf: Vec::new() }
    }

    /// Reset to the hunt state, dropping any partial frame.
    pub fn reset(&mut self) {
        self.buf.clear();
    }

    /// Feed an arbitrary chunk of bytes, calling `sink` once per complete CRC-valid frame, in
    /// order.
    pub fn feed(&mut self, bytes: &[u8], sink: &mut impl FnMut(DecodedFrame)) {
        for &b in bytes {
            self.push_byte(b, sink);
        }
    }

    /// Advance the state machine by one byte, delivering a completed frame to `sink`. Internal:
    /// `feed` drives this per byte.
    fn push_byte(&mut self, b: u8, sink: &mut impl FnMut(DecodedFrame)) {
        // If the buffer is empty we are hunting for a SOF. Drop any non-SOF byte.
        if self.buf.is_empty() {
            if b != SOF {
                return;
            }
            // Start a candidate frame at this SOF.
            let _ = self.buf.push(b);
            return;
        }

        // Buffer is non-empty: buf[0] is the candidate SOF. Append the new byte. The buffer can
        // never overflow because once we have HEADER_LEN bytes we know the exact target length and
        // process/resync as soon as it is reached; the only unbounded case (len byte not yet seen)
        // is capped at HEADER_LEN bytes.
        if self.buf.push(b).is_err() {
            // Defensive: should be unreachable given the length-driven processing below, but never
            // panic. Resync past the candidate SOF.
            self.resync_after_sof(sink);
            return;
        }

        // Validate the version byte (offset 1) as soon as it arrives: a wrong version means this is
        // not a real frame start, so resync immediately rather than waiting for a full length.
        if self.buf.len() == 2 && self.buf[1] != PROTO_VER {
            self.resync_after_sof(sink);
            return;
        }

        // Until we have the full header we cannot know the target length.
        if self.buf.len() < HEADER_LEN {
            return;
        }

        // We have at least the header: target total = HEADER_LEN + len + CRC_LEN.
        let len = self.buf[5] as usize;
        let total = HEADER_LEN + len + frame::CRC_LEN;
        if self.buf.len() < total {
            return; // need more bytes
        }

        // We have a full candidate frame's worth of bytes. Try to decode exactly `total` bytes.
        // (The buffer length equals `total` here: we process the instant we reach it, so it cannot
        // exceed it.)
        match frame::decode(&self.buf[..total]) {
            Ok(decoded) => {
                sink(decoded);
                // Consume the whole frame; continue hunting for the next one (coalesced frames are
                // delivered to `feed` byte by byte, so subsequent bytes re-enter via push_byte).
                self.buf.clear();
            }
            Err(_) => {
                // CRC or structural failure on a full-length candidate. Resync from the byte after
                // the presumed SOF so a false 0x5A still locks onto the next real frame.
                self.resync_after_sof(sink);
            }
        }
    }

    /// Drop the candidate SOF at buf[0] and re-feed the remaining buffered bytes as if freshly
    /// arrived, so an embedded real SOF is found. This preserves bytes after a false start instead
    /// of discarding the whole buffer.
    fn resync_after_sof(&mut self, sink: &mut impl FnMut(DecodedFrame)) {
        // Take everything after the candidate SOF (buf[0]) and replay it through the hunt logic.
        // Copy into a temporary since we clear the buffer before replaying.
        let mut tail: Vec<u8, MAX_FRAME> = Vec::new();
        // buf is non-empty here; skip index 0 (the bad SOF).
        for &b in &self.buf[1..] {
            // tail can never exceed MAX_FRAME-1, which fits MAX_FRAME.
            let _ = tail.push(b);
        }
        self.buf.clear();
        for &b in &tail {
            self.push_byte(b, sink);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{encode, FrameHeader, MAX_FRAME, PROTO_VER};
    use crate::opcode::Opcode;

    fn make_frame(opcode: Opcode, payload: &[u8], src: u8, dst: u8) -> ([u8; MAX_FRAME], usize) {
        let mut out = [0u8; MAX_FRAME];
        let hdr = FrameHeader { ver: PROTO_VER, opcode, src, dst, len: payload.len() as u8 };
        let n = encode(&hdr, payload, &mut out).unwrap();
        (out, n)
    }

    // Collect emitted frames into owned tuples so we can assert after the borrow ends.
    fn run(chunks: &[&[u8]]) -> heapless::Vec<(Opcode, heapless::Vec<u8, 64>), 8> {
        let mut framer = StreamFramer::new();
        let mut got: heapless::Vec<(Opcode, heapless::Vec<u8, 64>), 8> = heapless::Vec::new();
        for chunk in chunks {
            framer.feed(chunk, &mut |f| {
                let mut p: heapless::Vec<u8, 64> = heapless::Vec::new();
                p.extend_from_slice(f.payload).unwrap();
                got.push((f.header.opcode, p)).unwrap();
            });
        }
        got
    }

    #[test]
    fn single_frame_whole() {
        let (buf, n) = make_frame(Opcode::CyclicState, &[1, 2, 3, 4], 1, 2);
        let got = run(&[&buf[..n]]);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].0, Opcode::CyclicState);
        assert_eq!(&got[0].1[..], &[1, 2, 3, 4]);
    }

    #[test]
    fn split_in_two() {
        let (buf, n) = make_frame(Opcode::DriveCmd, &[9, 8, 7], 3, 4);
        // Split inside the header (after 3 bytes).
        let got = run(&[&buf[..3], &buf[3..n]]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0].1[..], &[9, 8, 7]);
    }

    #[test]
    fn split_in_three_incl_crc() {
        let (buf, n) = make_frame(Opcode::Telemetry, &[0xAA, 0xBB], 5, 6);
        // Split inside header (1 byte), inside payload, and inside the CRC (last byte alone).
        let got = run(&[&buf[..1], &buf[1..n - 1], &buf[n - 1..n]]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0].1[..], &[0xAA, 0xBB]);
    }

    #[test]
    fn coalesced_two() {
        let (a, an) = make_frame(Opcode::CyclicState, &[1], 1, 2);
        let (b, bn) = make_frame(Opcode::DriveCmd, &[2, 3], 3, 4);
        let mut joined: heapless::Vec<u8, { MAX_FRAME * 2 }> = heapless::Vec::new();
        joined.extend_from_slice(&a[..an]).unwrap();
        joined.extend_from_slice(&b[..bn]).unwrap();
        let got = run(&[&joined]);
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].0, Opcode::CyclicState);
        assert_eq!(got[1].0, Opcode::DriveCmd);
    }

    #[test]
    fn coalesced_three() {
        let (a, an) = make_frame(Opcode::CyclicState, &[], 1, 2);
        let (b, bn) = make_frame(Opcode::DriveCmd, &[7], 3, 4);
        let (c, cn) = make_frame(Opcode::Fault, &[1, 1], 5, 6);
        let mut joined: heapless::Vec<u8, { MAX_FRAME * 3 }> = heapless::Vec::new();
        joined.extend_from_slice(&a[..an]).unwrap();
        joined.extend_from_slice(&b[..bn]).unwrap();
        joined.extend_from_slice(&c[..cn]).unwrap();
        let got = run(&[&joined]);
        assert_eq!(got.len(), 3);
        assert_eq!(got[0].0, Opcode::CyclicState);
        assert_eq!(got[1].0, Opcode::DriveCmd);
        assert_eq!(got[2].0, Opcode::Fault);
    }

    #[test]
    fn crc_fail_emits_nothing() {
        let (mut buf, n) = make_frame(Opcode::DriveCmd, &[1, 2, 3], 1, 2);
        buf[HEADER_LEN] ^= 0xFF; // corrupt payload, leave CRC
        let got = run(&[&buf[..n]]);
        assert_eq!(got.len(), 0);
    }

    #[test]
    fn resync_past_garbage_and_false_sof() {
        let (buf, n) = make_frame(Opcode::CyclicState, &[0x11, 0x22], 1, 2);
        // Garbage including a stray 0x5A that is NOT a real frame start, then the real frame.
        let mut stream: heapless::Vec<u8, { MAX_FRAME + 16 }> = heapless::Vec::new();
        stream.extend_from_slice(&[0x00, 0xFF, SOF, 0x99, 0x01, 0x02]).unwrap();
        stream.extend_from_slice(&buf[..n]).unwrap();
        let got = run(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0].1[..], &[0x11, 0x22]);
    }

    #[test]
    fn resync_false_sof_with_bad_version() {
        // A 0x5A immediately followed by a wrong version byte should resync fast.
        let (buf, n) = make_frame(Opcode::DriveCmd, &[0x42], 1, 2);
        let mut stream: heapless::Vec<u8, { MAX_FRAME + 8 }> = heapless::Vec::new();
        stream.extend_from_slice(&[SOF, 0xEE, 0x00]).unwrap(); // false SOF, bad version
        stream.extend_from_slice(&buf[..n]).unwrap();
        let got = run(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0].1[..], &[0x42]);
    }

    #[test]
    fn back_to_back_two_in_two_chunks() {
        let (a, an) = make_frame(Opcode::CyclicState, &[1, 2], 1, 2);
        let (b, bn) = make_frame(Opcode::DriveCmd, &[3, 4], 3, 4);
        // First frame plus head of second, then tail of second.
        let mut first: heapless::Vec<u8, { MAX_FRAME * 2 }> = heapless::Vec::new();
        first.extend_from_slice(&a[..an]).unwrap();
        first.extend_from_slice(&b[..2]).unwrap();
        let got = run(&[&first, &b[2..bn]]);
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].0, Opcode::CyclicState);
        assert_eq!(got[1].0, Opcode::DriveCmd);
    }
}
