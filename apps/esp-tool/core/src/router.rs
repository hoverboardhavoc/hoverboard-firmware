//! The controller links' framing and routing (`specs/esp-tool.md`, "The BLE link" and "The
//! terminal tool"): the BLE link (L3 port 0) and the console (port 2) carry the same L2 stream.
//! The console has two carriers, the C3's USB-Serial-JTAG and a TCP client on the Wi-Fi console's
//! port, and is one port on whichever of them a client is attached: a client on the other carrier
//! while one is attached is refused ([`Router::open`], [`Router::on_bytes`]). The GATT peripheral,
//! the USB driver and the TCP listener are the firmware's (`apps/esp-tool/src/ble.rs`, `usb.rs`,
//! `wifi.rs`).
//!
//! The bytes on either port are a byte stream: a phone or the CC2541-shaped central coalesces and
//! splits them at arbitrary points, and so does a USB host or a TCP client. Each port feeds its
//! own L2 stream decoder; each complete packet addressed to the ESP (`dst = 0x00` with an opcode
//! in the bridge range, this version's only address for the ESP) goes to the command service and
//! its replies go back out of the port it came in on, and every other packet is forwarded into the
//! mailbox whole. Packets from the board go out of every attached port: this version has no node
//! role, so the ESP cannot tell which controller a board packet is for.

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use heapless::Deque;
use link::{Link, SerialTransport};
use net::pdu::{Pdu, NO_ADDRESS};

use crate::mailbox::{MailboxPort, MAX_PACKET};
use crate::svc::{Platform, Service};
use crate::swd::SwdPins;

/// The largest BLE notification payload: the CC2541's, which the app's transport is built around.
pub const NOTIFY_MAX: usize = 20;
/// The L2 frame capacity on a controller link: one PDU per frame, as on the mailbox.
pub const FRAME: usize = 128;
const RX_BYTES: usize = 1024;
const TX_BYTES: usize = 2048;

/// A controller port of the ESP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    /// The GATT characteristic pair (L3 port 0).
    Ble = 0,
    /// The console (L3 port 2), on whichever [`Via`] holds it.
    Console = 1,
}

const PORTS: [Port; 2] = [Port::Ble, Port::Console];

/// Where a controller's bytes come in: a port and, for the console, the carrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Via {
    /// The BLE central.
    Ble,
    /// The console on the USB-Serial-JTAG.
    Usb,
    /// The console on a TCP client of the Wi-Fi console.
    Tcp,
}

impl Via {
    fn port(self) -> Port {
        match self {
            Via::Ble => Port::Ble,
            Via::Usb | Via::Tcp => Port::Console,
        }
    }
}

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

/// Routes packets between the controller links, the command service and the mailbox port.
pub struct Router {
    ports: [Attached; 2],
    /// The carrier the console is attached on.
    console: Option<Via>,
    /// The port the last `FLASH_BEGIN` came in on: where the flash job's reports go.
    job_port: Port,
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
            ports: [Attached::new(), Attached::new()],
            console: None,
            job_port: Port::Ble,
            rx_overflow: 0,
        }
    }

    /// A controller on `via` connected (a BLE central, a TCP client): attach its port on a clean
    /// stream. False when the console is attached on the other carrier: the caller closes the
    /// newcomer at once, and the attached client is left alone.
    pub fn open(&mut self, via: Via) -> bool {
        if !self.holds(via) {
            return false;
        }
        if via != Via::Ble {
            self.console = Some(via);
        }
        let p = &mut self.ports[via.port() as usize];
        p.reset();
        p.attached = true;
        true
    }

    /// The controller on `via` went away: drop both directions' bytes and any partial frame. A
    /// carrier that does not hold the console changes nothing.
    pub fn gone(&mut self, via: Via) {
        if via != Via::Ble {
            if self.console != Some(via) {
                return;
            }
            self.console = None;
        }
        let p = &mut self.ports[via.port() as usize];
        p.reset();
        p.attached = false;
    }

    /// Drop a partial frame from `via`'s controller, keeping the port attached and its outbound
    /// bytes. For the USB console, whose host can go away mid-frame with no event: the frame's
    /// declared length would otherwise swallow the next host's first frames.
    pub fn drop_partial(&mut self, via: Via) {
        if !self.holds(via) {
            return;
        }
        let t = self.ports[via.port() as usize].link.transport_mut();
        t.reset();
        t.serial_mut().rx.clear();
    }

    /// Whether a controller is attached to `port`.
    pub fn attached(&self, port: Port) -> bool {
        self.ports[port as usize].attached
    }

    /// The carrier the console is attached on, where its outbound bytes go.
    pub fn console(&self) -> Option<Via> {
        self.console
    }

    /// `via` may use its port: always for BLE; for a console carrier, when the console is free
    /// or already on it.
    fn holds(&self, via: Via) -> bool {
        via == Via::Ble || self.console.is_none_or(|c| c == via)
    }

    /// Bytes that arrived on `via`. A port with no controller attached is attached by its first
    /// bytes (the USB console has no connect event; a BLE central connects before it writes).
    /// False, and the bytes dropped, when the console is attached on the other carrier.
    pub fn on_bytes(&mut self, via: Via, data: &[u8]) -> bool {
        if !self.attached(via.port()) && !self.open(via) {
            return false;
        }
        if !self.holds(via) {
            return false;
        }
        let rx = &mut self.ports[via.port() as usize]
            .link
            .transport_mut()
            .serial_mut()
            .rx;
        for &b in data {
            if rx.push_back(b).is_err() {
                self.rx_overflow += 1;
            }
        }
        true
    }

    /// The next bytes to send out of `port`, at most `out.len()` (a BLE notification, a USB
    /// packet, a TCP write); 0 when nothing is waiting. The console's go to [`Router::console`].
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

    /// One pass: route every complete packet from each port, service the mailbox port (its
    /// packets go to every attached port), and advance the command service (its job reports go to
    /// the port the flash began on).
    pub fn pump<P: SwdPins, Pl: Platform>(
        &mut self,
        port: &mut MailboxPort<P>,
        svc: &mut Service<Pl>,
    ) {
        let mut buf = [0u8; MAX_PACKET];
        for from in PORTS {
            while let Some(n) = self.ports[from as usize]
                .link
                .poll_recv(&mut buf)
                .map(|p| p.len())
            {
                let pkt = &buf[..n];
                match Pdu::decode(pkt) {
                    Ok(p) if p.dst == NO_ADDRESS && espsvc::is_esp_opcode(p.opcode) => {
                        if p.opcode == espsvc::OP_FLASH_BEGIN {
                            self.job_port = from;
                        }
                        let back = &mut self.ports[from as usize];
                        svc.handle(&p, port, &mut |reply| back.send(reply));
                    }
                    Ok(_) => port.send(pkt),
                    Err(_) => {}
                }
            }
        }
        let ports = &mut self.ports;
        port.poll(|pkt| {
            for p in ports.iter_mut() {
                p.send(pkt);
            }
        });
        let back = &mut self.ports[self.job_port as usize];
        svc.poll(port, &mut |reply| back.send(reply));
    }
}
