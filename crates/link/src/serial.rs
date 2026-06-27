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
pub struct SerialTransport<S> {
    serial: S,
    framer: StreamFramer,
    frame_capacity: usize,
}

impl<S> SerialTransport<S> {
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

impl<S: Read + Write + ReadReady> Transport for SerialTransport<S> {
    fn frame_capacity(&self) -> usize {
        self.frame_capacity
    }

    fn send_l2_frame(&mut self, l2: &[u8]) {
        // Best-effort, per the Transport contract: a serial write error drops the frame (L2 is
        // best-effort and a higher layer retransmits the control plane).
        let mut buf = [0u8; MAX_STREAM_FRAME];
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
