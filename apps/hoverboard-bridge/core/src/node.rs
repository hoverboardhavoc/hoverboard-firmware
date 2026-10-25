//! The bridge as an L3 node (`specs/hoverboard-bridge.md`, "The bridge as a node"): a bridge with three ports, the
//! `net` crate's walk responder and forwarding table on them, its keys in its own store, and the
//! dispatch of the frames addressed to it.
//!
//! | L3 port | Kind | Carriers |
//! | --- | --- | --- |
//! | 0 | BLE | the GATT characteristic pair ([`Port::Ble`]) |
//! | 1 | SWD mailbox | the board's RAM mailbox ([`MailboxPort`]) |
//! | 2 | UART | the console: the USB-Serial-JTAG ([`Port::Usb`]) and the Wi-Fi console's TCP client ([`Port::Tcp`]) |
//!
//! Every frame from a controller link or the mailbox goes through [`Responder::ingest`] with its
//! ingress port, which learns its source, forwards it by the table (flooding an unknown `dst` to
//! every port but the ingress one) and answers the walk and `CONFIG_*` opcodes addressed to the
//! bridge. What it hands back is the bridge's to act on:
//!
//! - an ESP command whose `dst` is `0x00` on a controller link (the one peer, as in round 1) or the
//!   bridge's own address from anywhere goes to the command service, and its replies go back on the
//!   carrier the request arrived on;
//! - a dst-`0x00` `CYCLIC_STATE` from the mailbox (the board's telemetry for its one peer, its own
//!   sample and its peer's forward) is re-published on every controller link with `src` unchanged.
//!
//! The responder's emissions go out by port. Port 2 is two carriers: a frame the forwarder sends
//! out of it goes on both, and a reply the bridge makes to a request that came in on one of them goes
//! back on that one. A forwarded frame never leaves by its ingress port, so an emission on port 2
//! while ingesting a frame from port 2 is always the bridge's own reply; the `PORTS` reply to a
//! `PROBE_PORTS`, which leaves after the probe window, goes where the probe came from.

use heapless::Vec;
use linkctl::OP_CYCLIC_STATE;
use net::pdu::{Pdu, NO_ADDRESS};
use net::walk::{
    Emits, Responder, CLASS_BRIDGE, MAX_PORTS, MCU_ESP32C3, PORT_BLE, PORT_SWD, PORT_UART,
};
use store::{Flash, Store};

use crate::mailbox::{MailboxPort, MAX_PACKET};
use crate::router::{Port, Router, PORTS};
use crate::svc::{Identity, Platform, Service};
use crate::swd::SwdPins;

/// L3 port 0: the BLE link.
pub const L3_BLE: u8 = 0;
/// L3 port 1: the board's mailbox.
pub const L3_MAILBOX: u8 = 1;
/// L3 port 2: the console, on its two carriers.
pub const L3_CONSOLE: u8 = 2;
const N_PORTS: u8 = 3;
const PORT_KINDS: [u8; MAX_PORTS] = [PORT_BLE, PORT_SWD, PORT_UART, 0];

/// How long a `PROBE_PORTS` waits for the neighbours' answers before `PORTS` goes out: the
/// boards' window (`POLL_WINDOW_TICKS`, 125 ticks at 250 Hz in `crates/firmware`).
pub const PROBE_WINDOW_US: u64 = 500_000;

/// The most replies one command produces (a final chunk: its acknowledgement and the status).
const REPLIES: usize = 4;

/// Where a frame arrived, and where a reply to it goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Carrier {
    /// A controller link: BLE, or one of the console's two carriers.
    Link(Port),
    /// The board's mailbox.
    Mailbox,
}

impl Carrier {
    /// The L3 port the carrier belongs to.
    pub fn l3_port(self) -> u8 {
        match self {
            Carrier::Link(Port::Ble) => L3_BLE,
            Carrier::Link(Port::Usb | Port::Tcp) => L3_CONSOLE,
            Carrier::Mailbox => L3_MAILBOX,
        }
    }
}

/// The bridge as a node.
pub struct Node<'f, F: Flash> {
    resp: Responder,
    store: Store<'f, F>,
    store_mounted: bool,
    /// The in-flight `PROBE_PORTS`: the carrier it came on and when the window opened.
    probe: Option<(Carrier, u64)>,
    /// Where the flash job's reports go: the carrier the last `FLASH_BEGIN` came in on.
    job: Carrier,
}

impl<'f, F: Flash> Node<'f, F> {
    /// The node over its `store`, with the address the store holds. `store_mounted` says whether
    /// the store is the bridge's own (`false`: it did not mount and the firmware gave the node an
    /// empty one, so the bridge runs unassigned and `ESP_INFO` says so). `fw_ver` is what
    /// `NODE_HELLO` reports.
    pub fn new(store: Store<'f, F>, store_mounted: bool, fw_ver: u16) -> Self {
        let mut resp = Responder::new(N_PORTS, PORT_KINDS, MCU_ESP32C3, CLASS_BRIDGE, fw_ver);
        resp.restore_addr(&store);
        Node {
            resp,
            store,
            store_mounted,
            probe: None,
            job: Carrier::Link(Port::Ble),
        }
    }

    /// The bridge's node address, `0x00` while unassigned.
    pub fn addr(&self) -> u8 {
        self.resp.addr()
    }

    /// The bridge as `ESP_INFO` reports it.
    pub fn identity(&self) -> Identity {
        Identity {
            addr: self.addr(),
            store_mounted: self.store_mounted,
        }
    }

    /// The store, for reading the keys.
    pub fn store(&self) -> &Store<'f, F> {
        &self.store
    }

    /// The responder, for routing-table inspection.
    pub fn responder(&self) -> &Responder {
        &self.resp
    }

    /// One pass: every complete packet from each controller link and from the mailbox through the
    /// node, the probe window, and the flash job's next step.
    pub fn pump<P: SwdPins, Pl: Platform>(
        &mut self,
        router: &mut Router,
        port: &mut MailboxPort<P>,
        svc: &mut Service<Pl>,
    ) {
        let mut buf = [0u8; MAX_PACKET];
        for link in PORTS {
            while let Some(n) = router.recv(link, &mut buf) {
                self.ingest(Carrier::Link(link), &buf[..n], router, port, svc);
            }
        }
        port.poll_session();
        while let Some(n) = port.recv(&mut buf) {
            self.ingest(Carrier::Mailbox, &buf[..n], router, port, svc);
        }

        if let Some((from, since)) = self.probe {
            if !self.resp.probing() {
                self.probe = None;
            } else if port.swd().now_us().saturating_sub(since) >= PROBE_WINDOW_US {
                self.probe = None;
                let mut emits = Emits::new();
                self.resp.poll_probe(&mut emits);
                send_emits(&emits, from, router, port);
            }
        }

        let addr = self.addr();
        let mut replies = Replies::new();
        svc.poll(port, &mut |dst, m| replies.push(addr, dst, m));
        replies.send(self.job, router, port);
    }

    fn ingest<P: SwdPins, Pl: Platform>(
        &mut self,
        from: Carrier,
        pkt: &[u8],
        router: &mut Router,
        port: &mut MailboxPort<P>,
        svc: &mut Service<Pl>,
    ) {
        let Ok(dst) = Pdu::decode(pkt).map(|p| p.dst) else {
            return;
        };
        let probing = self.resp.probing();
        let mut emits = Emits::new();
        let delivered = self
            .resp
            .ingest(from.l3_port(), pkt, &mut self.store, &mut emits);
        if !probing && self.resp.probing() {
            self.probe = Some((from, port.swd().now_us()));
        }
        send_emits(&emits, from, router, port);

        let Some(d) = delivered else {
            return;
        };
        let to_me = match from {
            Carrier::Link(_) => true,
            Carrier::Mailbox => dst != NO_ADDRESS && dst == self.addr(),
        };
        if espsvc::is_esp_opcode(d.opcode) && to_me {
            let Ok(pdu) = Pdu::new(d.opcode, d.src, dst, &d.payload) else {
                return;
            };
            if d.opcode == espsvc::OP_FLASH_BEGIN {
                self.job = from;
            }
            let me = self.identity();
            let mut replies = Replies::new();
            svc.handle(&pdu, me, port, &mut |to, m| replies.push(me.addr, to, m));
            replies.send(from, router, port);
        } else if from == Carrier::Mailbox && dst == NO_ADDRESS && d.opcode == OP_CYCLIC_STATE {
            for link in PORTS {
                router.send(link, pkt);
            }
        }
    }
}

/// Put the responder's emissions on their ports. `from` is the carrier of the frame that caused
/// them: an emission on port 2 goes back on it when it is one of the console's carriers (the bridge's
/// own reply), and on both console carriers otherwise.
fn send_emits<P: SwdPins>(
    emits: &Emits,
    from: Carrier,
    router: &mut Router,
    port: &mut MailboxPort<P>,
) {
    for e in emits {
        match e.port {
            L3_BLE => router.send(Port::Ble, &e.bytes),
            L3_MAILBOX => port.send(&e.bytes),
            L3_CONSOLE => match from {
                Carrier::Link(c @ (Port::Usb | Port::Tcp)) => router.send(c, &e.bytes),
                _ => {
                    router.send(Port::Usb, &e.bytes);
                    router.send(Port::Tcp, &e.bytes);
                }
            },
            _ => {}
        }
    }
}

/// The command service's replies, encoded from the bridge's address and held until the service has
/// let go of the mailbox port, so a reply can go out on it.
struct Replies(Vec<Vec<u8, { espsvc::MAX_PDU }>, REPLIES>);

impl Replies {
    fn new() -> Self {
        Replies(Vec::new())
    }

    fn push(&mut self, src: u8, dst: u8, m: &espsvc::Msg) {
        let mut buf = [0u8; espsvc::MAX_PDU];
        if let Ok(n) = m.encode(src, dst, &mut buf) {
            if let Ok(v) = Vec::from_slice(&buf[..n]) {
                let _ = self.0.push(v);
            }
        }
    }

    fn send<P: SwdPins>(&self, to: Carrier, router: &mut Router, port: &mut MailboxPort<P>) {
        for r in &self.0 {
            match to {
                Carrier::Link(l) => router.send(l, r),
                Carrier::Mailbox => port.send(r),
            }
        }
    }
}
