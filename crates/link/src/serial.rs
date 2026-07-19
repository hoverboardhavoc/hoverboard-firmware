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
}

impl<S, const N: usize> SerialTransport<S, N> {
    /// Wrap `serial`, advertising `frame_capacity` (the largest L2 frame this carrier puts in one
    /// stream frame; the usable chunk is `frame_capacity - 1`).
    pub fn new(serial: S, frame_capacity: usize) -> Self {
        SerialTransport {
            serial,
            framer: StreamFramer::new(),
            frame_capacity,
        }
    }

    /// Reset the receive framer, dropping any partial frame. The SWD mailbox calls this on an epoch
    /// change so a half-written frame from a previous bridge session is never fed as a stale partial
    /// (`specs/swd-mailbox.md`, "Attach + session flush").
    pub fn reset(&mut self) {
        self.framer.reset();
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
        // Non-blocking: pull only ready bytes, one at a time, feeding each through the framer.
        // Feeding a single byte completes at most one frame (a frame completes exactly on its final
        // byte, and the framer drains every buffered frame after each fed byte), so the closure
        // captures at most one emitted frame per iteration.
        while self.serial.read_ready().unwrap_or(false) {
            let mut one = [0u8; 1];
            match self.serial.read(&mut one) {
                Ok(1) => {
                    let mut got = None;
                    self.framer.feed(&one, &mut |f| {
                        let n = f.len().min(out.len());
                        out[..n].copy_from_slice(&f[..n]);
                        got = Some(n);
                    });
                    if let Some(n) = got {
                        return Some(n);
                    }
                }
                _ => break,
            }
        }
        None
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
