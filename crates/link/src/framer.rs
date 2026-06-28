//! Byte-stream transport framing: a self-delimiting, CRC-protected frame and a resyncing decoder.
//!
//! Per `specs/l2.md` ("Byte-stream transport (inter-board UART)"), a raw stream has no boundaries
//! and no integrity, so L2 supplies both:
//!
//! ```text
//! [ SOF : 1 = 0x5A ][ len : 1 ][ frag-hdr : 1 ][ chunk : len-1 ][ CRC-16 : 2 ]
//! ```
//!
//! - `len` = bytes from `frag-hdr` through end of `chunk` (so `len == 1 + chunk.len()`); the inner
//!   `[ frag-hdr ][ chunk ]` is the **L2 frame** this module carries, identical to the datagram one.
//! - CRC-16/MODBUS (`base::crc16`) over `SOF..chunk`, little-endian on the wire.
//!
//! The resync state-machine skeleton is lifted from the prior `link` crate's framer (git history),
//! but the frame format is new: no addressing (that is L3), and the inner L2 frame is a `frag-hdr` +
//! chunk the old single-byte-stream framer never had.

use base::crc16;
use heapless::Vec;

/// Start-of-frame marker.
pub const SOF: u8 = 0x5A;
/// Fixed leading bytes before the L2 frame: `SOF` then `len`.
pub const STREAM_HEADER_LEN: usize = 2;
/// Trailing CRC length.
pub const STREAM_CRC_LEN: usize = 2;
/// Largest `len` value (the `len` field is one byte), i.e. the largest inner L2 frame.
pub const MAX_L2_LEN: usize = 255;
/// Largest total stream frame on the wire: `SOF + len + (frag-hdr..chunk) + CRC`.
pub const MAX_STREAM_FRAME: usize = STREAM_HEADER_LEN + MAX_L2_LEN + STREAM_CRC_LEN;

/// Reasons [`encode`] can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// The L2 frame is empty (no `frag-hdr`) or longer than [`MAX_L2_LEN`].
    BadLen,
    /// The supplied `out` buffer is smaller than the encoded stream frame.
    OutTooSmall,
}

/// Wrap one L2 frame (`[ frag-hdr ][ chunk ]`) into a stream frame in `out`, appending a
/// little-endian CRC-16/MODBUS over `SOF..chunk`. Returns the total encoded length.
pub fn encode(l2: &[u8], out: &mut [u8]) -> Result<usize, FrameError> {
    let len = l2.len();
    if len == 0 || len > MAX_L2_LEN {
        return Err(FrameError::BadLen);
    }
    let total = STREAM_HEADER_LEN + len + STREAM_CRC_LEN;
    if out.len() < total {
        return Err(FrameError::OutTooSmall);
    }
    out[0] = SOF;
    out[1] = len as u8;
    out[STREAM_HEADER_LEN..STREAM_HEADER_LEN + len].copy_from_slice(l2);
    let crc = crc16::modbus(&out[..STREAM_HEADER_LEN + len]);
    out[STREAM_HEADER_LEN + len] = (crc & 0x00FF) as u8;
    out[STREAM_HEADER_LEN + len + 1] = (crc >> 8) as u8;
    Ok(total)
}

/// A resyncing stream framer over a bounded `heapless` buffer.
///
/// Eats arbitrary byte chunks, resyncs on `SOF`, validates `len` + CRC, and calls `sink` once per
/// whole CRC-valid L2 frame. A bad CRC drops that frame and the framer resyncs at the next `SOF`.
///
/// `N` is the buffer size - the largest stream frame this framer can hold. It defaults to
/// [`MAX_STREAM_FRAME`] (a maximal 255-byte L2 frame), but a small-MTU carrier (the SWD mailbox / the
/// inter-board UART: <=128-byte frames) sets a small `N` to keep the framer within a tight RAM budget.
/// A candidate whose declared length exceeds `N` cannot be a real frame on that carrier, so it is
/// resynced past as a false `SOF` (at the default `N` this can never trigger - `len` is one byte).
pub struct StreamFramer<const N: usize = MAX_STREAM_FRAME> {
    /// Bytes accumulated since the current candidate `SOF` (`buf[0]` is always the candidate `SOF`
    /// once non-empty).
    buf: Vec<u8, N>,
}

impl<const N: usize> Default for StreamFramer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> StreamFramer<N> {
    /// A fresh framer in the hunt state.
    pub const fn new() -> StreamFramer<N> {
        StreamFramer { buf: Vec::new() }
    }

    /// Reset to the hunt state, dropping any partial frame.
    pub fn reset(&mut self) {
        self.buf.clear();
    }

    /// Feed an arbitrary chunk of bytes, calling `sink` once per complete CRC-valid L2 frame
    /// (`[ frag-hdr ][ chunk ]`), in order.
    pub fn feed(&mut self, bytes: &[u8], sink: &mut impl FnMut(&[u8])) {
        for &b in bytes {
            // Invariant: `process` always leaves `buf.len() < total <= MAX_STREAM_FRAME`, so after a
            // single push the buffer can never exceed `MAX_STREAM_FRAME`; the push cannot fail.
            let _ = self.buf.push(b);
            self.process(sink);
        }
    }

    /// Drain the buffer iteratively, emitting every complete CRC-valid frame it now holds. Resync is
    /// a bounded loop (drop one byte, re-scan), **never recursion**: on silicon a recursive resync
    /// would stack-allocate a max-frame buffer per level and blow the GD32 stack (see `specs/l2.md`,
    /// "On silicon the resync must be iterative or depth-bounded").
    fn process(&mut self, sink: &mut impl FnMut(&[u8])) {
        loop {
            // Hunt: drop leading non-SOF bytes in one shift so garbage never accumulates.
            let mut hunt = 0;
            while hunt < self.buf.len() && self.buf[hunt] != SOF {
                hunt += 1;
            }
            if hunt > 0 {
                self.discard_front(hunt);
            }

            // Need at least SOF + len to know the target length.
            if self.buf.len() < STREAM_HEADER_LEN {
                return;
            }

            // A real frame always carries a frag-hdr, so len >= 1. A len == 0 is a false SOF: drop it
            // and re-scan from the next byte.
            let len = self.buf[1] as usize;
            if len == 0 {
                self.discard_front(1);
                continue;
            }

            let total = STREAM_HEADER_LEN + len + STREAM_CRC_LEN;
            if total > N {
                // The declared frame is longer than this framer's buffer can ever hold, so it cannot be
                // a real frame on this carrier - a false `SOF`. Drop it and re-scan. (At the default
                // `N == MAX_STREAM_FRAME` this is unreachable: `len` is a single byte, so `total <= N`.)
                self.discard_front(1);
                continue;
            }
            if self.buf.len() < total {
                return; // need more bytes for this candidate
            }

            // A full candidate frame. Validate the CRC over SOF..chunk.
            let crc_calc = crc16::modbus(&self.buf[..STREAM_HEADER_LEN + len]);
            let crc_wire = u16::from_le_bytes([
                self.buf[STREAM_HEADER_LEN + len],
                self.buf[STREAM_HEADER_LEN + len + 1],
            ]);
            if crc_calc == crc_wire {
                sink(&self.buf[STREAM_HEADER_LEN..STREAM_HEADER_LEN + len]);
                self.discard_front(total);
            } else {
                // CRC failure on a full-length candidate: drop just the candidate SOF and re-scan, so
                // a false 0x5A still locks onto the next real frame (resync past the false start).
                self.discard_front(1);
            }
            // Loop to process any bytes left in the buffer (coalesced frames, or the replayed tail
            // after a resync).
        }
    }

    /// Drop the first `n` bytes of the buffer, shifting the remainder to the front. Bounded by
    /// `MAX_STREAM_FRAME`; this is the iterative resync's only buffer manipulation (no recursion, no
    /// per-level allocation).
    fn discard_front(&mut self, n: usize) {
        let len = self.buf.len();
        debug_assert!(n <= len);
        self.buf.copy_within(n.., 0);
        self.buf.truncate(len - n);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Collect emitted L2 frames into owned vectors so assertions outlive the borrow.
    fn run(chunks: &[&[u8]]) -> std::vec::Vec<std::vec::Vec<u8>> {
        let mut framer: StreamFramer = StreamFramer::new();
        let mut got: std::vec::Vec<std::vec::Vec<u8>> = std::vec::Vec::new();
        for chunk in chunks {
            framer.feed(chunk, &mut |f| got.push(f.to_vec()));
        }
        got
    }

    // Encode an L2 frame (frag-hdr + chunk) into a stream frame.
    fn frame(l2: &[u8]) -> std::vec::Vec<u8> {
        let mut out = [0u8; MAX_STREAM_FRAME];
        let n = encode(l2, &mut out).unwrap();
        out[..n].to_vec()
    }

    #[test]
    fn round_trip_nonempty() {
        // L2 frame = frag-hdr 0x00 + chunk [1,2,3,4].
        let l2 = [0x00, 1, 2, 3, 4];
        let got = run(&[&frame(&l2)]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn round_trip_empty_payload() {
        // Empty chunk: the L2 frame is a lone frag-hdr (len == 1). This is the spec's "empty
        // payload" framer case.
        let l2 = [0x00];
        let f = frame(&l2);
        assert_eq!(f.len(), STREAM_HEADER_LEN + 1 + STREAM_CRC_LEN); // SOF len hdr CRC CRC
        let got = run(&[&f]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn bad_crc_dropped() {
        let mut f = frame(&[0x00, 0xAA, 0xBB]);
        let hdr_off = STREAM_HEADER_LEN; // first byte of the L2 frame (frag-hdr)
        f[hdr_off + 1] ^= 0xFF; // corrupt a chunk byte, leave CRC
        let got = run(&[&f]);
        assert_eq!(got.len(), 0);
    }

    #[test]
    fn split_across_read_chunks() {
        let l2 = [0x00, 9, 8, 7, 6, 5];
        let f = frame(&l2);
        // Split inside the header, inside the chunk, and inside the CRC (last byte alone).
        let got = run(&[&f[..1], &f[1..3], &f[3..f.len() - 1], &f[f.len() - 1..]]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn coalesced_frames() {
        let a = frame(&[0x00, 1]);
        let b = frame(&[0x11, 2, 3]);
        let c = frame(&[0x22]); // empty chunk
        let mut joined = std::vec::Vec::new();
        joined.extend_from_slice(&a);
        joined.extend_from_slice(&b);
        joined.extend_from_slice(&c);
        let got = run(&[&joined]);
        assert_eq!(got.len(), 3);
        assert_eq!(&got[0][..], &[0x00, 1]);
        assert_eq!(&got[1][..], &[0x11, 2, 3]);
        assert_eq!(&got[2][..], &[0x22]);
    }

    #[test]
    fn resync_past_garbage_and_false_sof() {
        let l2 = [0x00, 0x11, 0x22];
        // Leading garbage that contains a stray 0x5A which is NOT a real frame start. The stray SOF
        // declares len=1, so its candidate frame (5A 01 77 00 00) completes within the garbage, fails
        // CRC, and the framer resyncs past it, then decodes the real frame that follows.
        let mut stream = std::vec::Vec::new();
        stream.extend_from_slice(&[0x00, 0xFF, SOF, 0x01, 0x77, 0x00, 0x00]);
        stream.extend_from_slice(&frame(&l2));
        let got = run(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn false_sof_with_large_len_delays_then_recovers() {
        // The format has no magic/version byte after SOF, so a stray 0x5A whose `len` byte is large
        // declares a long frame that absorbs the following real frame as payload. Recovery is only
        // DELAYED, not lost: once the declared window fills the CRC fails, the framer resyncs by
        // replaying the buffered tail, and the embedded real frame is found. (Within a truncated
        // stream the window never fills, so nothing is emitted; this documents that boundary.)
        let l2 = [0x00, 0x11, 0x22];
        let real = frame(&l2);

        // Truncated: the false 153-byte window never fills, so the real frame stays buffered.
        let mut short = std::vec::Vec::new();
        short.extend_from_slice(&[SOF, 0x99]); // stray SOF, len = 153
        short.extend_from_slice(&real);
        assert_eq!(run(&[&short]).len(), 0);

        // Continued: enough trailing bytes to fill the false window. The candidate CRC fails, the
        // framer replays the tail, and the real frame is recovered.
        let mut long = short.clone();
        long.extend_from_slice(&[0u8; 160]); // pad past the 153-byte window
        let got = run(&[&long]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn iterative_resync_terminates_on_pathological_garbage() {
        // A run of SOF bytes far longer than MAX_STREAM_FRAME, each a false frame start that resyncs
        // a byte at a time. The resync must be iterative/bounded: a recursive resync would recurse
        // per dropped byte and blow the GD32 stack. The properties pinned here: it terminates (no
        // crash, no stack growth), it emits no false frame for the garbage, and the framer is not
        // wedged afterward (once the false window is flushed, a real frame is recovered).
        let garbage = std::vec![SOF; MAX_STREAM_FRAME * 4];
        let l2 = [0x00, 0xA5, 0x5A];

        let mut framer: StreamFramer = StreamFramer::new();
        framer.feed(&garbage, &mut |_| panic!("garbage must not emit a frame"));

        // Flush the trailing false-SOF window, then feed a real frame: it is recovered. (An all-SOF
        // run never self-completes its declared window, so recovery is delayed until non-SOF bytes
        // flush it - the documented byte-stream resync-delay property.)
        let mut got: std::vec::Vec<std::vec::Vec<u8>> = std::vec::Vec::new();
        framer.feed(&[0x00; MAX_STREAM_FRAME], &mut |_| {
            panic!("flush must not emit a frame")
        });
        framer.feed(&frame(&l2), &mut |f| got.push(f.to_vec()));
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn resync_false_sof_then_real_frame() {
        // A false SOF immediately followed by a len that points past the real frame; the CRC fails
        // and the framer must still recover the following real frame.
        let l2 = [0x33, 0x42];
        let mut stream = std::vec::Vec::new();
        stream.extend_from_slice(&[SOF, 0x02, 0xDE, 0xAD]); // false SOF, len=2, junk that fails CRC
        stream.extend_from_slice(&frame(&l2));
        let got = run(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn trailing_bytes_tolerated() {
        let l2 = [0x00, 0x55];
        let mut stream = frame(&l2);
        // Trailing non-SOF garbage after a complete frame must not break or fabricate a frame.
        stream.extend_from_slice(&[0x00, 0x01, 0x02, 0xFF]);
        let got = run(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn len_zero_is_rejected() {
        // A frame claiming len == 0 (no frag-hdr) is structurally invalid: resync, deliver nothing,
        // and still recover a following real frame.
        let l2 = [0x00, 0x77];
        let mut stream = std::vec::Vec::new();
        stream.extend_from_slice(&[SOF, 0x00, 0x12, 0x34]);
        stream.extend_from_slice(&frame(&l2));
        let got = run(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn encode_rejects_empty_and_oversize() {
        let mut out = [0u8; MAX_STREAM_FRAME];
        assert_eq!(encode(&[], &mut out), Err(FrameError::BadLen));
        let too_long = [0u8; MAX_L2_LEN + 1];
        assert_eq!(encode(&too_long, &mut out), Err(FrameError::BadLen));
        // Out buffer too small.
        let mut tiny = [0u8; 3];
        assert_eq!(encode(&[0x00, 1], &mut tiny), Err(FrameError::OutTooSmall));
    }
}
