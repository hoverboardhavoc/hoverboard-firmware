//! The BLE link's framing and routing (`specs/esp-tool.md`, "The BLE link"); the GATT peripheral
//! itself is the firmware's (`apps/esp-tool/src/ble.rs`).
//!
//! Bytes written to the write characteristic are a byte stream: a phone or the CC2541-shaped
//! central coalesces and splits them at arbitrary points. They feed an L2 stream decoder; each
//! complete packet addressed to the ESP (`dst = 0x00` with an opcode in the bridge range, this
//! version's only address for the ESP) goes to the command service, and every other packet is
//! forwarded into the mailbox whole. Frames to the central are split into notifications of at most
//! [`NOTIFY_MAX`] bytes.

use embedded_io::{ErrorKind, ErrorType, Read, ReadReady, Write};
use heapless::Deque;
use link::{Link, SerialTransport};
use net::pdu::{Pdu, NO_ADDRESS};

use crate::mailbox::{MailboxPort, MAX_PACKET};
use crate::svc::Service;
use crate::swd::SwdPins;

/// The largest notification payload: the CC2541's, which the app's transport is built around.
pub const NOTIFY_MAX: usize = 20;
/// The L2 frame capacity on the BLE link: one PDU per frame, as on the mailbox.
pub const FRAME: usize = 128;
const RX_BYTES: usize = 1024;
const TX_BYTES: usize = 2048;

/// The BLE characteristic pair as a byte-stream serial: GATT writes fill `rx`, notifications drain
/// `tx`.
pub struct BleSerial {
    rx: Deque<u8, RX_BYTES>,
    tx: Deque<u8, TX_BYTES>,
}

/// The outbound buffer has no room for a whole frame (the central is not reading).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxFull;

impl embedded_io::Error for TxFull {
    fn kind(&self) -> ErrorKind {
        ErrorKind::WriteZero
    }
}

impl ErrorType for BleSerial {
    type Error = TxFull;
}

impl Read for BleSerial {
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

impl Write for BleSerial {
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

impl ReadReady for BleSerial {
    fn read_ready(&mut self) -> Result<bool, TxFull> {
        Ok(!self.rx.is_empty())
    }
}

type BleLink = Link<SerialTransport<BleSerial, { FRAME + 4 }>, MAX_PACKET, FRAME>;

/// Routes packets between the BLE link, the command service and the mailbox port.
pub struct Router {
    link: BleLink,
    connected: bool,
    /// Bytes written by the central that did not fit the receive buffer.
    pub rx_overflow: u32,
}

impl Default for Router {
    fn default() -> Self {
        Self::new()
    }
}

impl Router {
    /// A router with no central.
    pub fn new() -> Self {
        Router {
            link: Link::new(SerialTransport::new(
                BleSerial {
                    rx: Deque::new(),
                    tx: Deque::new(),
                },
                FRAME,
            )),
            connected: false,
            rx_overflow: 0,
        }
    }

    /// A central connected: start from a clean stream.
    pub fn connect(&mut self) {
        self.reset();
        self.connected = true;
    }

    /// The central went away: drop both directions' bytes and any partial frame.
    pub fn disconnect(&mut self) {
        self.reset();
        self.connected = false;
    }

    fn reset(&mut self) {
        let t = self.link.transport_mut();
        t.reset();
        t.serial_mut().rx.clear();
        t.serial_mut().tx.clear();
    }

    /// Bytes the central wrote to the write characteristic.
    pub fn on_write(&mut self, data: &[u8]) {
        let rx = &mut self.link.transport_mut().serial_mut().rx;
        for &b in data {
            if rx.push_back(b).is_err() {
                self.rx_overflow += 1;
            }
        }
    }

    /// The next notification, at most [`NOTIFY_MAX`] bytes; 0 when nothing is waiting.
    pub fn next_notification(&mut self, out: &mut [u8; NOTIFY_MAX]) -> usize {
        let tx = &mut self.link.transport_mut().serial_mut().tx;
        let mut n = 0;
        while n < NOTIFY_MAX {
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

    /// Send a packet to the central (dropped when none is connected).
    pub fn send(&mut self, packet: &[u8]) {
        if self.connected {
            let _ = self.link.send(packet);
        }
    }

    /// One pass: route every complete packet from the central, service the mailbox port (its
    /// packets go to the central), and advance the command service.
    pub fn pump<P: SwdPins>(&mut self, port: &mut MailboxPort<P>, svc: &mut Service) {
        let mut buf = [0u8; MAX_PACKET];
        while let Some(n) = self.link.poll_recv(&mut buf).map(|p| p.len()) {
            let pkt = &buf[..n];
            match Pdu::decode(pkt) {
                Ok(p) if p.dst == NO_ADDRESS && espsvc::is_esp_opcode(p.opcode) => {
                    svc.handle(&p, port, &mut |reply| self.send(reply));
                }
                Ok(_) => port.send(pkt),
                Err(_) => {}
            }
        }
        port.poll(|pkt| self.send(pkt));
        svc.poll(port, &mut |reply| self.send(reply));
    }
}
