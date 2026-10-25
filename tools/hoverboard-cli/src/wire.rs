//! Packets over a byte stream: the L2 stream framing (`link`) between the client and a link that
//! moves bytes (the BLE characteristic pair, the USB console directly or over TCP, or a test's
//! in-memory bridge).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use link::{Link, SerialTransport};

/// The client's view of the link: whole packets (L3 PDUs) out and in.
pub trait Wire {
    /// Send one packet.
    fn send(&mut self, pdu: &[u8]);
    /// Send packets in order, as one write where the link can (a bulk transfer's window).
    fn send_all(&mut self, pdus: &[Vec<u8>]) {
        for p in pdus {
            self.send(p);
        }
    }
    /// The next packet, or `None` once `timeout` passes with none.
    fn recv(&mut self, timeout: Duration) -> Option<Vec<u8>>;
    /// Bulk chunks in flight on this link ([`espsvc::WINDOW_BLE`] or [`espsvc::WINDOW_STREAM`]).
    fn window(&self) -> usize;
}

/// A link that moves bytes.
pub trait ByteLink {
    /// Write bytes (the link splits them as it must).
    fn write(&mut self, bytes: &[u8]);
    /// Bytes that arrived, waiting up to `timeout` for the first.
    fn read(&mut self, timeout: Duration) -> Vec<u8>;
    /// Bulk chunks in flight on this carrier (`specs/hoverboard-bridge.md`, "Bulk transfers").
    fn window(&self) -> usize;
}

impl<T: ByteLink + ?Sized> ByteLink for Box<T> {
    fn write(&mut self, bytes: &[u8]) {
        (**self).write(bytes)
    }
    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        (**self).read(timeout)
    }
    fn window(&self) -> usize {
        (**self).window()
    }
}

/// The L2 frame capacity this tool sends with: one PDU per frame, as the bridge's link does.
pub const FRAME: usize = 128;

/// An in-process serial between the framing and the byte link.
#[derive(Default)]
struct Bytes {
    rx: VecDeque<u8>,
    tx: Vec<u8>,
}

#[derive(Debug)]
struct Never;

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
        let n = buf.len().min(self.rx.len());
        for b in buf.iter_mut().take(n) {
            *b = self.rx.pop_front().unwrap_or(0);
        }
        Ok(n)
    }
}

impl Write for Bytes {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Never> {
        self.tx.extend_from_slice(buf);
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

/// [`Wire`] over a [`ByteLink`] with the L2 stream framing.
pub struct Framed<B: ByteLink> {
    bytes: B,
    link: Link<SerialTransport<Bytes>>,
}

impl<B: ByteLink> Framed<B> {
    /// Frame packets over `bytes`.
    pub fn new(bytes: B) -> Self {
        Framed {
            bytes,
            link: Link::new(SerialTransport::new(Bytes::default(), FRAME)),
        }
    }

    /// The byte link.
    pub fn inner(&mut self) -> &mut B {
        &mut self.bytes
    }
}

impl<B: ByteLink> Wire for Framed<B> {
    fn send(&mut self, pdu: &[u8]) {
        self.send_all(&[pdu.to_vec()]);
    }

    fn send_all(&mut self, pdus: &[Vec<u8>]) {
        for p in pdus {
            let _ = self.link.send(p);
        }
        let out = std::mem::take(&mut self.link.transport_mut().serial_mut().tx);
        self.bytes.write(&out);
    }

    fn window(&self) -> usize {
        self.bytes.window()
    }

    fn recv(&mut self, timeout: Duration) -> Option<Vec<u8>> {
        let deadline = Instant::now() + timeout;
        let mut buf = [0u8; 256];
        loop {
            if let Some(p) = self.link.poll_recv(&mut buf) {
                return Some(p.to_vec());
            }
            let left = deadline.saturating_duration_since(Instant::now());
            let got = self.bytes.read(left);
            if got.is_empty() && Instant::now() >= deadline {
                return None;
            }
            self.link.transport_mut().serial_mut().rx.extend(got);
        }
    }
}
