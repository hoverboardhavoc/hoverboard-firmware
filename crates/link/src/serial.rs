//! A byte-stream [`Transport`] over any `embedded-io` serial (`Read + Write + ReadReady`).
//!
//! L2's link seam is the frame-oriented [`Transport`] trait (`send_l2_frame` / `recv_l2_frame`);
//! `embedded-io` is the device-driver seam (a UART, the BLE [`Pipe`], the SWD `MailboxSerial`). This
//! shim bridges the two: it runs the existing [`StreamFramer`] over a serial so any serial becomes a
//! byte-stream L2 link. It invents no framing - it reuses [`encode`] on the send side and
//! [`StreamFramer`] on the receive side verbatim.
//!
//! It is **not** mailbox-specific: the production inter-board UART transport is the same shim over a
//! HAL serial, and the SWD mailbox is the same shim over a `MailboxSerial`. Each instance is given the
//! `frame_capacity` its carrier permits (255 for a UART, below the ring size for the mailbox).
//!
//! [`Pipe`]: https://docs.rs/ble

use embedded_io::{Read, ReadReady, Write};

use crate::framer::{encode, StreamFramer, MAX_STREAM_FRAME};
use crate::link::Transport;

/// Bytes pulled from the serial per chunked read (the drain-cost lever, round-8 slice 1).
///
/// `recv_l2_frame` pulls up to this many bytes per `serial.read` call into a staging buffer, then
/// feeds the framer byte-by-byte from the stage. The expensive backend chain
/// (`RingBufferedRx::read`'s DMA-ring snapshot + the one hoisted u64 modulo + the IDLE / line-error
/// atomics) runs once per chunk instead of once per byte; only the cheap in-SRAM framer scan stays
/// per byte. Sized to hold a back-to-back double frame (the inter-board UART's measured 19.0 B wire
/// frame, `bytes_max_call` = 38 = two frames) in one read so a busy poll amortizes over ~a frame's
/// worth of bytes. 24 B per instance keeps the single-frame amortization (a 19 B wire frame still
/// fits one chunk) while trimming the RAM cost for the stack budget (round-8 audit; round-9 slice 3
/// re-paint): a back-to-back double frame (38 B) simply takes two chunked reads instead of one.
const PULL_CHUNK: usize = 24;

/// A [`Transport`] that wraps an `embedded-io` serial and frames the L2 byte stream with
/// [`StreamFramer`] (SOF / len / frag-hdr / chunk / CRC-16/MODBUS).
///
/// `N` is the framer + send buffer size (the largest stream frame). It defaults to
/// [`MAX_STREAM_FRAME`]; a small-MTU carrier sets a small `N` (>= its `frame_capacity` + 4) to fit a
/// tight RAM budget. `N` must be at least `frame_capacity + STREAM_HEADER_LEN + STREAM_CRC_LEN`.
pub struct SerialTransport<S, const N: usize = MAX_STREAM_FRAME> {
    serial: S,
    framer: StreamFramer<N>,
    frame_capacity: usize,
    /// Bytes pulled from `serial` in one chunked read but not yet fed to the framer. `stage_pos..
    /// stage_len` is the unfed remainder, carried across `recv_l2_frame` calls so the
    /// one-frame-per-call pull contract holds while the serial read is chunked.
    stage: [u8; PULL_CHUNK],
    stage_len: usize,
    stage_pos: usize,
}

impl<S, const N: usize> SerialTransport<S, N> {
    /// Wrap `serial`, advertising `frame_capacity` (the largest L2 frame this carrier puts in one
    /// stream frame; the usable chunk is `frame_capacity - 1`).
    pub fn new(serial: S, frame_capacity: usize) -> Self {
        SerialTransport {
            serial,
            framer: StreamFramer::new(),
            frame_capacity,
            stage: [0u8; PULL_CHUNK],
            stage_len: 0,
            stage_pos: 0,
        }
    }

    /// Reset the receive framer, dropping any partial frame. The SWD mailbox calls this on an epoch
    /// change so a half-written frame from a previous bridge session is never fed as a stale partial
    /// (`specs/swd-mailbox.md`, "Attach + session flush").
    pub fn reset(&mut self) {
        self.framer.reset();
        // Drop any chunk-staged bytes too: an epoch flush must not replay bytes pulled from the
        // previous bridge session's ring as a stale partial.
        self.stage_len = 0;
        self.stage_pos = 0;
    }

    /// Borrow the inner serial.
    pub fn serial(&self) -> &S {
        &self.serial
    }

    /// Borrow the inner serial mutably.
    pub fn serial_mut(&mut self) -> &mut S {
        &mut self.serial
    }
}

impl<S: Read + Write + ReadReady, const N: usize> Transport for SerialTransport<S, N> {
    fn frame_capacity(&self) -> usize {
        self.frame_capacity
    }

    fn send_l2_frame(&mut self, l2: &[u8]) {
        // Best-effort, per the Transport contract: a serial write error drops the frame (L2 is
        // best-effort and a higher layer retransmits the control plane).
        let mut buf = [0u8; N];
        if let Ok(n) = encode(l2, &mut buf) {
            let _ = self.serial.write_all(&buf[..n]);
            let _ = self.serial.flush();
        }
    }

    fn recv_l2_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        // Non-blocking. Two levels: refill a staging buffer from the serial in ONE chunked read
        // (amortizing the backend's per-call cost over many bytes), then drive the framer over the
        // staged run with `feed_one`, which BULK-copies the known-length body (`specs/l2.md`,
        // "Frame-at-a-time decode") and stops at the first completed frame, reporting how many staged
        // bytes it consumed. The unfed remainder is carried across calls, so the one-frame-per-call
        // pull contract holds while the read is chunked.
        loop {
            if self.stage_pos < self.stage_len {
                let mut got = None;
                let consumed =
                    self.framer
                        .feed_one(&self.stage[self.stage_pos..self.stage_len], &mut |f| {
                            let n = f.len().min(out.len());
                            out[..n].copy_from_slice(&f[..n]);
                            got = Some(n);
                        });
                self.stage_pos += consumed;
                if let Some(n) = got {
                    return Some(n);
                }
                // No frame: `feed_one` consumed the whole staged run (the stage is now empty). Fall
                // through to refill.
            }
            // Stage drained. Refill in one chunked read; stop when the serial has nothing ready or a
            // read makes no progress (an absorbed condition returns Ok(0) after its own retry).
            if !self.serial.read_ready().unwrap_or(false) {
                return None;
            }
            match self.serial.read(&mut self.stage) {
                Ok(n) if n > 0 => {
                    self.stage_len = n;
                    self.stage_pos = 0;
                }
                _ => return None,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A mock serial: a byte queue delivering AT MOST `burst` bytes per `read` call, so the tests
    /// exercise the pull loop against dribbling deliveries as well as bulk ones.
    struct MockSerial {
        rx: VecDeque<u8>,
        burst: usize,
    }
    impl embedded_io::ErrorType for MockSerial {
        type Error = core::convert::Infallible;
    }
    impl Read for MockSerial {
        fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error> {
            let n = out.len().min(self.burst).min(self.rx.len());
            for slot in out.iter_mut().take(n) {
                *slot = self.rx.pop_front().unwrap();
            }
            Ok(n)
        }
    }
    impl Write for MockSerial {
        fn write(&mut self, data: &[u8]) -> Result<usize, Self::Error> {
            Ok(data.len())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
    impl ReadReady for MockSerial {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.rx.is_empty())
        }
    }

    /// Encode an L2 frame (frag-hdr + chunk) the way `send_l2_frame` does.
    fn wire_frame(l2: &[u8]) -> std::vec::Vec<u8> {
        let mut buf = [0u8; MAX_STREAM_FRAME];
        let n = encode(l2, &mut buf).unwrap();
        buf[..n].to_vec()
    }

    /// Pull contract: several frames buffered at once are returned by SUCCESSIVE `recv_l2_frame`
    /// calls, in order, none lost. Impl-agnostic (holds for the per-byte pull and must keep
    /// holding for any future chunked pull; round-4 slice 1 pins it before that rework returns).
    #[test]
    fn buffered_frames_all_delivered_in_order() {
        let mut rx = VecDeque::new();
        for tag in [0xA1u8, 0xB2, 0xC3] {
            rx.extend(wire_frame(&[0x01, tag, tag ^ 0xFF]));
        }
        let serial = MockSerial { rx, burst: 64 };
        let mut t: SerialTransport<_, MAX_STREAM_FRAME> = SerialTransport::new(serial, 96);
        let mut out = [0u8; 64];
        for tag in [0xA1u8, 0xB2, 0xC3] {
            let n = t.recv_l2_frame(&mut out).expect("frame delivered");
            assert_eq!(&out[..n], &[0x01, tag, tag ^ 0xFF], "in order, none lost");
        }
        assert!(t.recv_l2_frame(&mut out).is_none(), "queue drained");
    }

    /// A frame arriving in dribbles (1 byte per serial read) still reassembles across calls.
    #[test]
    fn frame_split_across_reads_reassembles() {
        let payload: std::vec::Vec<u8> = (0..40).map(|i| i as u8).collect();
        let mut rx = VecDeque::new();
        rx.extend(wire_frame(&payload));
        let serial = MockSerial { rx, burst: 1 };
        let mut t: SerialTransport<_, MAX_STREAM_FRAME> = SerialTransport::new(serial, 96);
        let mut out = [0u8; 96];
        let n = t.recv_l2_frame(&mut out).expect("reassembled");
        assert_eq!(&out[..n], &payload[..]);
    }

    /// Chunked-pull property (round-8 slice 1): a burst whose total exceeds `PULL_CHUNK` forces the
    /// staging buffer to refill mid-drain, with individual frames straddling a refill boundary. Every
    /// frame must still surface exactly once, in order (no byte lost across a stage refill).
    #[test]
    fn frames_spanning_chunk_boundary_all_delivered() {
        // Nine 7-byte wire frames = 63 B > PULL_CHUNK (24), so the 24 B stage refills ~twice and
        // several frames cross a refill boundary.
        let tags: [u8; 9] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99];
        let mut rx = VecDeque::new();
        for &tag in &tags {
            rx.extend(wire_frame(&[0x01, tag, tag ^ 0xFF]));
        }
        // burst=64 > PULL_CHUNK, so each serial read is capped by the 24 B stage, not the mock burst.
        let serial = MockSerial { rx, burst: 64 };
        let mut t: SerialTransport<_, MAX_STREAM_FRAME> = SerialTransport::new(serial, 96);
        let mut out = [0u8; 64];
        for &tag in &tags {
            let n = t
                .recv_l2_frame(&mut out)
                .expect("frame delivered across refill");
            assert_eq!(&out[..n], &[0x01, tag, tag ^ 0xFF], "in order, none lost");
        }
        assert!(t.recv_l2_frame(&mut out).is_none(), "queue drained");
    }

    /// `reset()` drops any partial state (the epoch-flush contract): bytes buffered before a
    /// reset must never surface as a stale partial afterwards.
    #[test]
    fn reset_drops_partial_state() {
        let good: std::vec::Vec<u8> = wire_frame(&[0x01, 0x42, 0x43]);
        let mut rx = VecDeque::new();
        rx.extend(&good[..good.len() - 2]);
        let serial = MockSerial { rx, burst: 64 };
        let mut t: SerialTransport<_, MAX_STREAM_FRAME> = SerialTransport::new(serial, 96);
        let mut out = [0u8; 64];
        assert!(t.recv_l2_frame(&mut out).is_none(), "partial: no frame yet");
        t.reset();
        t.serial_mut().rx.extend(&good[good.len() - 2..]);
        t.serial_mut().rx.extend(wire_frame(&[0x01, 0x77, 0x78]));
        let n = t.recv_l2_frame(&mut out).expect("fresh frame after reset");
        assert_eq!(
            &out[..n],
            &[0x01, 0x77, 0x78],
            "stale partial never surfaces"
        );
    }
}
