//! The controller links' framing (`specs/hoverboard-bridge.md`, "The BLE link" and "The terminal tool"):
//! the BLE link (L3 port 0) and the console (port 2) carry the same L2 stream. The console has two
//! carriers, the C3's USB-Serial-JTAG and a client of the Wi-Fi console's TCP port, and both are
//! served whenever present, each with its own stream. The GATT peripheral, the USB driver and the
//! TCP listener are the firmware's (`apps/hoverboard-bridge/src/ble.rs`, `usb.rs`, `wifi.rs`).
//!
//! The bytes on each link are a byte stream: a phone or the CC2541-shaped central coalesces and
//! splits them at arbitrary points, and so does a USB host or a TCP client. Each link feeds its own
//! L2 stream decoder, and [`Router::recv`] hands back each complete packet; what to do with it is
//! the node's ([`crate::node`]), which sends its answers and forwarded frames back through
//! [`Router::send`].

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use heapless::Deque;
use link::{Link, SerialTransport};

use crate::mailbox::MAX_PACKET;

/// The largest BLE notification payload: the CC2541's, which the app's transport is built around.
pub const NOTIFY_MAX: usize = 20;
/// The L2 frame capacity on a controller link: one PDU per frame, as on the mailbox.
pub const FRAME: usize = 128;
/// A port's receive buffer. The service loop moves a carrier's bytes in only while
/// [`Router::room`] holds the carrier's next read, so a console's bulk window (64 chunk frames,
/// 3,840 B) waits in the carrier's own buffers and flow control rather than overflowing this.
const RX_BYTES: usize = 1024;
const TX_BYTES: usize = 2048;

/// A controller link of the bridge: the BLE port, and the console's two carriers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    /// The GATT characteristic pair (L3 port 0).
    Ble = 0,
    /// The console on the USB-Serial-JTAG (L3 port 2).
    Usb = 1,
    /// The console on a client of the Wi-Fi console's TCP port (L3 port 2).
    Tcp = 2,
}

/// Every controller link, in the order the node serves them.
pub const PORTS: [Port; 3] = [Port::Ble, Port::Usb, Port::Tcp];

/// A port's bytes as a byte-stream serial: what arrives fills `rx`, what goes out drains `tx`.
pub struct StreamSerial {
    rx: Deque<u8, RX_BYTES>,
    tx: Deque<u8, TX_BYTES>,
}

/// The outbound buffer has no room for a whole frame (the controller is not reading).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxFull;

impl embedded_io::Error for TxFull {
    fn kind(&self) -> ErrorKind {
        ErrorKind::WriteZero
    }
}

impl ErrorType for StreamSerial {
    type Error = TxFull;
}

impl Read for StreamSerial {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TxFull> {
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

impl Write for StreamSerial {
    fn write(&mut self, buf: &[u8]) -> Result<usize, TxFull> {
        // All or nothing, and never Ok(0) for a non-empty buffer (`write_all` panics on it).
        if self.tx.capacity() - self.tx.len() < buf.len() {
            return Err(TxFull);
        }
        for &b in buf {
            let _ = self.tx.push_back(b);
        }
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), TxFull> {
        Ok(())
    }
}

impl ReadReady for StreamSerial {
    fn read_ready(&mut self) -> Result<bool, TxFull> {
        Ok(!self.rx.is_empty())
    }
}

type StreamLink = Link<SerialTransport<StreamSerial, { FRAME + 4 }>, MAX_PACKET, FRAME>;

struct Attached {
    link: StreamLink,
    attached: bool,
}

impl Attached {
    fn new() -> Self {
        Attached {
            link: Link::new(SerialTransport::new(
                StreamSerial {
                    rx: Deque::new(),
                    tx: Deque::new(),
                },
                FRAME,
            )),
            attached: false,
        }
    }

    fn reset(&mut self) {
        let t = self.link.transport_mut();
        t.reset();
        t.serial_mut().rx.clear();
        t.serial_mut().tx.clear();
    }

    fn send(&mut self, packet: &[u8]) {
        if self.attached {
            let _ = self.link.send(packet);
        }
    }
}

/// The controller links' streams.
pub struct Router {
    ports: [Attached; 3],
    /// Bytes from a controller that did not fit its port's receive buffer.
    pub rx_overflow: u32,
}

impl Default for Router {
    fn default() -> Self {
        Self::new()
    }
}

impl Router {
    /// A router with no controller attached.
    pub fn new() -> Self {
        Router {
            ports: [Attached::new(), Attached::new(), Attached::new()],
            rx_overflow: 0,
        }
    }

    /// A controller attached to `port` (a BLE central, a TCP client): start from a clean stream.
    pub fn connect(&mut self, port: Port) {
        let p = &mut self.ports[port as usize];
        p.reset();
        p.attached = true;
    }

    /// The controller on `port` went away: drop both directions' bytes and any partial frame.
    pub fn disconnect(&mut self, port: Port) {
        let p = &mut self.ports[port as usize];
        p.reset();
        p.attached = false;
    }

    /// Drop a partial frame from `port`'s controller, keeping the port attached and its outbound
    /// bytes. For the USB console, whose host can go away mid-frame with no event: the frame's
    /// declared length would otherwise swallow the next host's first frames.
    pub fn drop_partial(&mut self, port: Port) {
        let t = self.ports[port as usize].link.transport_mut();
        t.reset();
        t.serial_mut().rx.clear();
    }

    /// Free bytes in `port`'s receive buffer: what [`Router::on_bytes`] takes without overflow.
    pub fn room(&self, port: Port) -> usize {
        RX_BYTES - self.ports[port as usize].link.transport().serial().rx.len()
    }

    /// Whether a controller is attached to `port`.
    pub fn attached(&self, port: Port) -> bool {
        self.ports[port as usize].attached
    }

    /// Bytes that arrived on `port`. A port with no controller attached is attached by its first
    /// bytes: the USB console has no connect event, and stays attached from then on, since the C3
    /// cannot tell a host that left the port open from one that went away.
    pub fn on_bytes(&mut self, port: Port, data: &[u8]) {
        if !self.attached(port) {
            self.connect(port);
        }
        let rx = &mut self.ports[port as usize]
            .link
            .transport_mut()
            .serial_mut()
            .rx;
        for &b in data {
            if rx.push_back(b).is_err() {
                self.rx_overflow += 1;
            }
        }
    }

    /// The next bytes to send out of `port`, at most `out.len()` (a BLE notification, a USB
    /// packet, a TCP write); 0 when nothing is waiting.
    pub fn next_out(&mut self, port: Port, out: &mut [u8]) -> usize {
        let tx = &mut self.ports[port as usize]
            .link
            .transport_mut()
            .serial_mut()
            .tx;
        let mut n = 0;
        while n < out.len() {
            match tx.pop_front() {
                Some(b) => {
                    out[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    /// The next complete packet from `port`'s controller, copied into `buf`; its length.
    pub fn recv(&mut self, port: Port, buf: &mut [u8]) -> Option<usize> {
        self.ports[port as usize]
            .link
            .poll_recv(buf)
            .map(|p| p.len())
    }

    /// Send `packet` to `port`'s controller; nothing when none is attached.
    pub fn send(&mut self, port: Port, packet: &[u8]) {
        self.ports[port as usize].send(packet);
    }
}
