//! Byte-stream transport framing: a self-delimiting, CRC-protected frame and a resyncing decoder.
//!
//! Per `specs/l2.md` ("Carrying the L2 frame on the wire (one framing, every link)"), a raw stream
//! has no boundaries and no integrity, so L2 supplies both - and every shipped link (SWD mailbox,
//! inter-board UART, BLE) carries this same frame:
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
    /// (`[ frag-hdr ][ chunk ]`), in order. Emit-all: consumes the whole input.
    ///
    /// This is the emit-all variant (the datagram tests and the Kotlin/host mirrors use it). The
    /// receive drain uses [`feed_one`](Self::feed_one) instead, which stops at the first frame.
    pub fn feed(&mut self, bytes: &[u8], sink: &mut impl FnMut(&[u8])) {
        self.drain(bytes, false, sink);
    }

    /// Feed bytes, stopping as soon as ONE complete CRC-valid frame is emitted (or the input is
    /// exhausted). Returns the number of input bytes consumed; the caller advances its cursor by
    /// that count and calls again for the next frame, carrying any un-consumed remainder.
    ///
    /// This is the receive-drain entry (`SerialTransport::recv_l2_frame`'s one-frame-per-call pull
    /// contract, `specs/l2.md` "Frame-at-a-time decode"). It bulk-copies the body run rather than
    /// pushing one byte at a time.
    pub fn feed_one(&mut self, bytes: &[u8], sink: &mut impl FnMut(&[u8])) -> usize {
        self.drain(bytes, true, sink)
    }

    /// Consume `bytes`, appending to the candidate buffer in **bulk body runs** (one copy per frame
    /// body, `specs/l2.md` "Frame-at-a-time decode") rather than byte-by-byte, and emitting every
    /// complete CRC-valid frame via `sink`. Returns the number of input bytes consumed.
    ///
    /// When `stop_after_one`, returns the moment one frame is emitted (the one-frame-per-call pull
    /// path); otherwise consumes all of `bytes`. Resync is a bounded loop (drop one byte, re-scan),
    /// **never recursion**: on silicon a recursive resync would stack-allocate a max-frame buffer per
    /// level and blow the GD32 stack (`specs/l2.md`, "The resync is iterative over a bounded buffer").
    fn drain(&mut self, bytes: &[u8], stop_after_one: bool, sink: &mut impl FnMut(&[u8])) -> usize {
        let mut pos = 0;
        loop {
            // Hunt: drop leading non-SOF bytes already buffered (after a resync `buf[0]` can be
            // non-SOF), then, if the buffer is empty, bulk-skip leading non-SOF bytes in the input so
            // garbage never even enters the buffer.
            let mut hunt = 0;
            while hunt < self.buf.len() && self.buf[hunt] != SOF {
                hunt += 1;
            }
            if hunt > 0 {
                self.discard_front(hunt);
            }
            if self.buf.is_empty() {
                while pos < bytes.len() && bytes[pos] != SOF {
                    pos += 1;
                }
                if pos >= bytes.len() {
                    return pos; // no SOF in the remaining input
                }
                // A SOF is at bytes[pos]; the fill steps below copy it in as buf[0].
            }

            // Fill up to the 2-byte header so `len` (and thus the target length) is known.
            if self.buf.len() < STREAM_HEADER_LEN {
                let take = (STREAM_HEADER_LEN - self.buf.len()).min(bytes.len() - pos);
                // Bounded: STREAM_HEADER_LEN <= N (the smallest carrier N is >= frame_capacity + 4),
                // so the push cannot overflow.
                let _ = self.buf.extend_from_slice(&bytes[pos..pos + take]);
                pos += take;
                if self.buf.len() < STREAM_HEADER_LEN {
                    return pos; // input exhausted before the header completed
                }
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

            // Bulk-copy the body run: everything the current candidate still needs, capped by what the
            // input holds, in one shift. This is the per-frame (not per-byte) cost structure.
            if self.buf.len() < total {
                let take = (total - self.buf.len()).min(bytes.len() - pos);
                // Bounded: buf.len() + take <= total <= N, so the push cannot overflow.
                let _ = self.buf.extend_from_slice(&bytes[pos..pos + take]);
                pos += take;
                if self.buf.len() < total {
                    return pos; // need more bytes for this candidate
                }
            }

            // A full candidate frame. Validate the CRC over SOF..chunk as one contiguous slice.
            let crc_calc = crc16::modbus(&self.buf[..STREAM_HEADER_LEN + len]);
            let crc_wire = u16::from_le_bytes([
                self.buf[STREAM_HEADER_LEN + len],
                self.buf[STREAM_HEADER_LEN + len + 1],
            ]);
            if crc_calc == crc_wire {
                sink(&self.buf[STREAM_HEADER_LEN..STREAM_HEADER_LEN + len]);
                self.discard_front(total);
                if stop_after_one {
                    return pos;
                }
            } else {
                // CRC failure on a full-length candidate: drop just the candidate SOF and re-scan, so
                // a false 0x5A still locks onto the next real frame (resync past the false start).
                self.discard_front(1);
            }
            // Loop to process any bytes left in the buffer or the input (coalesced frames, or the
            // replayed tail after a resync).
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

    // Drive `feed_one` repeatedly across each feed slice, collecting frames via the
    // one-frame-per-call path (the receive-drain contract), advancing by the reported consumed count.
    fn run_feed_one(feeds: &[&[u8]]) -> std::vec::Vec<std::vec::Vec<u8>> {
        let mut framer: StreamFramer = StreamFramer::new();
        let mut got: std::vec::Vec<std::vec::Vec<u8>> = std::vec::Vec::new();
        for feed in feeds {
            let mut pos = 0;
            loop {
                let mut one = None;
                let consumed = framer.feed_one(&feed[pos..], &mut |f| one = Some(f.to_vec()));
                pos += consumed;
                let had_frame = one.is_some();
                if let Some(f) = one {
                    got.push(f);
                }
                // Done with this feed once it is fully consumed and produced no further frame.
                if !had_frame && (pos >= feed.len() || consumed == 0) {
                    break;
                }
            }
        }
        got
    }

    #[test]
    fn feed_one_returns_one_frame_per_call() {
        // The one-frame-per-call contract: each call returns exactly ONE frame and consumes only that
        // frame's bytes, leaving the coalesced remainder for the next call.
        let a = frame(&[0x00, 0x11]);
        let b = frame(&[0x11, 0x22, 0x33]);
        let mut joined = std::vec::Vec::new();
        joined.extend_from_slice(&a);
        joined.extend_from_slice(&b);

        let mut framer: StreamFramer = StreamFramer::new();
        let mut one = None;
        let consumed = framer.feed_one(&joined, &mut |f| one = Some(f.to_vec()));
        assert_eq!(consumed, a.len(), "consumed exactly the first frame");
        assert_eq!(one.unwrap(), &[0x00, 0x11]);
        let mut two = None;
        let consumed2 = framer.feed_one(&joined[consumed..], &mut |f| two = Some(f.to_vec()));
        assert_eq!(consumed2, b.len());
        assert_eq!(two.unwrap(), &[0x11, 0x22, 0x33]);
    }

    #[test]
    fn feed_one_header_split_across_feeds() {
        // SOF alone in the first feed, len + rest in the second: the bulk body copy must resume
        // correctly after the header completes on a later feed.
        let l2 = [0x00u8, 1, 2, 3, 4, 5, 6, 7];
        let f = frame(&l2);
        let got = run_feed_one(&[&f[..1], &f[1..]]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn feed_one_body_split_across_feeds() {
        // Header + partial body in the first feed, the rest (incl CRC) in the second: two bulk copies
        // accumulate the one frame across the boundary.
        let l2 = [0x00u8, 10, 20, 30, 40, 50, 60, 70, 80, 90];
        let f = frame(&l2);
        let mid = f.len() - 3; // split inside the body, before the CRC
        let got = run_feed_one(&[&f[..mid], &f[mid..]]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn feed_one_resyncs_past_garbage_and_false_sof() {
        // Leading garbage + a false SOF (len=1, fails CRC) then the real frame, in one feed: feed_one
        // bulk-skips the garbage, drops the false candidate, and delivers the real frame.
        let l2 = [0x00u8, 0xAB, 0xCD];
        let mut stream = std::vec::Vec::new();
        stream.extend_from_slice(&[0x00, 0xFF, 0x13, SOF, 0x01, 0x77, 0x00, 0x00]);
        stream.extend_from_slice(&frame(&l2));
        let got = run_feed_one(&[&stream]);
        assert_eq!(got.len(), 1);
        assert_eq!(&got[0][..], &l2);
    }

    #[test]
    fn feed_one_matches_feed_on_coalesced_stream() {
        // Equivalence: the one-frame-per-call path yields the same frames, same order, as the
        // emit-all `feed` for a coalesced multi-frame stream (behaviour-neutral refactor).
        let a = frame(&[0x00, 1]);
        let b = frame(&[0x11, 2, 3, 4]);
        let c = frame(&[0x22]); // empty chunk
        let mut joined = std::vec::Vec::new();
        for f in [&a, &b, &c] {
            joined.extend_from_slice(f);
        }
        let via_feed = run(&[&joined]);
        let via_feed_one = run_feed_one(&[&joined]);
        assert_eq!(via_feed, via_feed_one);
        assert_eq!(via_feed.len(), 3);
    }

    // --- The differential property test (round-16 audit item) --------------------------------
    //
    // Over many pseudo-random byte streams (real frames + garbage + explicit false SOFs + SOF
    // runs), split into random chunks of <= 24 B (the `recv_l2_frame` PULL_CHUNK stage), the two
    // decode paths must agree EXACTLY: the emit-all `feed` and the iterated one-frame-per-call
    // `feed_one` produce the identical frame sequence, and `feed_one` accounts for every input
    // byte. Fully deterministic: a fixed-seed xorshift, no wall-clock / OS randomness.

    /// A tiny deterministic xorshift64 (seed must be nonzero); reproducible across runs/platforms.
    struct Rng(u64);
    impl Rng {
        fn new(seed: u64) -> Self {
            Rng(seed | 1)
        }
        fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            (x >> 32) as u32
        }
        /// A value in `0..n` (n > 0).
        fn below(&mut self, n: u32) -> u32 {
            self.next_u32() % n
        }
        fn byte(&mut self) -> u8 {
            self.next_u32() as u8
        }
    }

    /// Build one pseudo-random wire stream mixing valid frames, garbage, false SOFs, and SOF runs.
    fn build_stream(rng: &mut Rng) -> std::vec::Vec<u8> {
        let mut s = std::vec::Vec::new();
        let segments = 3 + rng.below(12);
        for _ in 0..segments {
            match rng.below(4) {
                0 => {
                    // A valid frame with a random-length, random-content L2 body.
                    let l2len = 1 + rng.below(24) as usize;
                    let mut l2 = std::vec::Vec::with_capacity(l2len);
                    for _ in 0..l2len {
                        l2.push(rng.byte());
                    }
                    let mut out = [0u8; MAX_STREAM_FRAME];
                    let n = encode(&l2, &mut out).unwrap();
                    s.extend_from_slice(&out[..n]);
                }
                1 => {
                    // Random garbage (may itself contain stray SOF bytes).
                    let g = 1 + rng.below(10);
                    for _ in 0..g {
                        s.push(rng.byte());
                    }
                }
                2 => {
                    // An explicit false SOF with a random len byte and random junk body.
                    s.push(SOF);
                    s.push(rng.byte());
                    let j = rng.below(8);
                    for _ in 0..j {
                        s.push(rng.byte());
                    }
                }
                _ => {
                    // A run of SOF bytes (the pathological resync-a-byte-at-a-time case).
                    let r = 1 + rng.below(6) as usize;
                    s.resize(s.len() + r, SOF);
                }
            }
        }
        s
    }

    /// Split `s` into random chunks of size 1..=24 (the recv-drain stage width).
    fn chunk<'a>(s: &'a [u8], rng: &mut Rng) -> std::vec::Vec<&'a [u8]> {
        let mut chunks = std::vec::Vec::new();
        let mut i = 0;
        while i < s.len() {
            let max = 24.min(s.len() - i) as u32;
            let sz = 1 + rng.below(max) as usize;
            chunks.push(&s[i..i + sz]);
            i += sz;
        }
        chunks
    }

    /// Drive iterated `feed_one` over the chunk sequence, collecting frames and total consumed.
    fn via_feed_one(chunks: &[&[u8]]) -> (std::vec::Vec<std::vec::Vec<u8>>, usize) {
        let mut framer: StreamFramer = StreamFramer::new();
        let mut got: std::vec::Vec<std::vec::Vec<u8>> = std::vec::Vec::new();
        let mut total = 0usize;
        for c in chunks {
            let mut pos = 0;
            loop {
                let mut one = None;
                let consumed = framer.feed_one(&c[pos..], &mut |f| one = Some(f.to_vec()));
                pos += consumed;
                total += consumed;
                let had = one.is_some();
                if let Some(f) = one {
                    got.push(f);
                }
                if !had && (pos >= c.len() || consumed == 0) {
                    break;
                }
            }
        }
        (got, total)
    }

    #[test]
    fn feed_and_feed_one_agree_on_random_chunked_streams() {
        // Iterate many deterministic streams from a fixed root seed.
        let root = Rng::new(0x9E37_79B9_7F4A_7C15);
        for iter in 0..2000u64 {
            let mut sr = Rng::new(root.0 ^ iter.wrapping_mul(0x2545_F491_4F6C_DD1D));
            let stream = build_stream(&mut sr);

            // Same chunk boundaries feed both decoders (apples-to-apples).
            let chunks = chunk(&stream, &mut sr);

            let via_feed = {
                let mut framer: StreamFramer = StreamFramer::new();
                let mut got: std::vec::Vec<std::vec::Vec<u8>> = std::vec::Vec::new();
                for c in &chunks {
                    framer.feed(c, &mut |f| got.push(f.to_vec()));
                }
                got
            };
            let (via_one, consumed) = via_feed_one(&chunks);

            assert_eq!(
                via_feed,
                via_one,
                "feed vs iterated feed_one disagree on stream {iter} (len {})",
                stream.len()
            );
            assert_eq!(
                consumed,
                stream.len(),
                "feed_one must account for every input byte on stream {iter}"
            );
        }
    }
}
