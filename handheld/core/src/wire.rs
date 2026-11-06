//! The L2 stream between the handheld and the bridge's GATT pair (`specs/handheld-remote.md`
//! section 3.5): `link::Link` over `link::SerialTransport` on an in-RAM `embedded-io` serial whose
//! reads drain the notification bytes and whose writes collect the bytes for `rx`. The CLI's
//! `Framed` (`tools/hoverboard-cli/src/wire.rs`) is the model.

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use heapless::{Deque, Vec};
use link::{Link, SerialTransport};

/// The L2 frame capacity, the value the CLI and the bridge's controller links use
/// (`tools/hoverboard-cli/src/wire.rs`, `FRAME`; `bridge/core/src/router.rs`).
pub const FRAME: usize = 128;
/// The stream frame buffer: the frame capacity plus the stream header and CRC.
const STREAM: usize = FRAME + 4;
/// The largest packet reassembled: an L3 PDU here is at most `net::walk::MAX_PDU`.
const PACKET: usize = 256;
/// Notification bytes held before the framer takes them.
pub const RX_CAP: usize = 1024;
/// Stream bytes held before they are written to `rx`.
pub const TX_CAP: usize = 512;

/// The in-process serial between the framing and the GATT pair.
#[derive(Default)]
pub struct Bytes {
    rx: Deque<u8, RX_CAP>,
    tx: Vec<u8, TX_CAP>,
}

/// The serial never fails; a full buffer drops bytes, which the stream framer resyncs over.
#[derive(Debug)]
pub struct Never;

impl core::fmt::Display for Never {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("never")
    }
}

impl core::error::Error for Never {}

impl embedded_io::Error for Never {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for Bytes {
    type Error = Never;
}

impl Read for Bytes {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Never> {
        let mut n = 0;
        while n < buf.len() {
            match self.rx.pop_front() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        Ok(n)
    }
}

impl Write for Bytes {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Never> {
        // A frame that does not fit is dropped whole by the caller's check (`Wire::send`); a partial
        // write here would put a torn frame on the stream.
        if self.tx.extend_from_slice(buf).is_err() {
            return Ok(0);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Never> {
        Ok(())
    }
}

impl ReadReady for Bytes {
    fn read_ready(&mut self) -> Result<bool, Never> {
        Ok(!self.rx.is_empty())
    }
}

/// Whole L3 PDUs in and out over the byte stream.
pub struct Wire {
    link: Link<SerialTransport<Bytes, STREAM>, PACKET>,
}

impl Default for Wire {
    fn default() -> Self {
        Self::new()
    }
}

impl Wire {
    /// An empty stream.
    pub fn new() -> Self {
        Wire {
            link: Link::new(SerialTransport::new(Bytes::default(), FRAME)),
        }
    }

    /// Bytes from one `tx` notification. Bytes past [`RX_CAP`] are dropped; the framer resyncs on
    /// the next start of frame.
    pub fn feed(&mut self, bytes: &[u8]) {
        let rx = &mut self.link.transport_mut().serial_mut().rx;
        for &b in bytes {
            if rx.push_back(b).is_err() {
                break;
            }
        }
    }

    /// Frame one PDU onto the outgoing stream. Returns whether the whole frame was queued.
    pub fn send(&mut self, pdu: &[u8]) -> bool {
        self.link.send(pdu).is_ok()
    }

    /// Outgoing stream bytes waiting for `rx`.
    pub fn pending(&self) -> usize {
        self.link.transport().serial().tx.len()
    }

    /// Move up to `out.len()` outgoing stream bytes into `out`, oldest first; the count moved.
    pub fn take(&mut self, out: &mut [u8]) -> usize {
        let tx = &mut self.link.transport_mut().serial_mut().tx;
        let n = out.len().min(tx.len());
        out[..n].copy_from_slice(&tx[..n]);
        let rest = tx.len() - n;
        tx.copy_within(n.., 0);
        tx.truncate(rest);
        n
    }

    /// Drop every queued byte both ways (a new connection starts a new stream).
    pub fn reset(&mut self) {
        let t = self.link.transport_mut();
        t.reset();
        let s = t.serial_mut();
        s.rx.clear();
        s.tx.clear();
    }

    /// The next whole PDU received, copied into `buf`.
    pub fn recv<'a>(&mut self, buf: &'a mut [u8]) -> Option<&'a [u8]> {
        self.link.poll_recv(buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pdu_crosses_the_stream_in_twenty_byte_notifications() {
        let mut a = Wire::new();
        let mut b = Wire::new();
        let pdu = [
            0x10, 0x01, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        ];
        assert!(a.send(&pdu));
        let mut chunk = [0u8; 20];
        loop {
            let n = a.take(&mut chunk);
            if n == 0 {
                break;
            }
            b.feed(&chunk[..n]);
        }
        let mut buf = [0u8; 256];
        assert_eq!(b.recv(&mut buf), Some(&pdu[..]));
        assert_eq!(b.recv(&mut buf), None);
    }

    #[test]
    fn take_moves_the_oldest_bytes_first() {
        let mut a = Wire::new();
        assert!(a.send(&[1, 2, 3]));
        let total = a.pending();
        let mut first = [0u8; 2];
        assert_eq!(a.take(&mut first), 2);
        assert_eq!(a.pending(), total - 2);
    }
}
