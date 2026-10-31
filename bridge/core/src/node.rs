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

use espsvc::WifiState;
use heapless::{String, Vec};
use linkctl::OP_CYCLIC_STATE;
use net::pdu::{Pdu, NO_ADDRESS};
pub use net::walk::BRIDGE_NAME_MAX;
use net::walk::{
    bridge_name_usable, bridge_value_usable, Emits, Responder, CLASS_BRIDGE, MAX_PORTS,
    MCU_ESP32C3, PORT_BLE, PORT_SWD, PORT_UART, WIFI_PASS_MAX, WIFI_SSID_MAX,
};
use store::{
    Flash, Store, StrField, Type, Value, DEVICE_NAME, WIFI_PASS, WIFI_SSID, WIFI_TX_POWER,
};

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
/// boards' window (`POLL_WINDOW_TICKS`, 125 ticks at 250 Hz in `motorcontroller/firmware`).
pub const PROBE_WINDOW_US: u64 = 500_000;

/// The most replies one command produces (a final chunk: its acknowledgement and the status).
const REPLIES: usize = 4;

/// The name until one is written: `hb-bridge-` and the last four hex digits of the MAC. It is the
/// node's own and is never written to the store (`specs/hoverboard-bridge.md`, "Its store"): the
/// responder answers it for a read of an unstored `device.name` ([`Responder::set_default_name`]).
pub const fn default_name(mac: [u8; 6]) -> [u8; 14] {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut buf = *b"hb-bridge-0000";
    let mut i = 0;
    while i < 2 {
        let b = mac[4 + i];
        buf[10 + 2 * i] = HEX[(b >> 4) as usize];
        buf[11 + 2 * i] = HEX[(b & 0xF) as usize];
        i += 1;
    }
    buf
}

/// The name BLE advertises and mDNS answers for, copied into `buf`; returns its length: the
/// `device.name` the store holds, or `default` when it holds none. A `CONFIG_WRITE` of a name that
/// fails [`bridge_name_usable`] is refused, but a store written before that refusal existed may
/// still hold one: it stays in the store and the default is used. Nothing is written: the default
/// is answered by the node, not stored, so an un-stored name stays un-stored across a reboot.
pub fn advertised_name<F: Flash>(
    store: &Store<'_, F>,
    default: &[u8],
    buf: &mut [u8; BRIDGE_NAME_MAX],
) -> usize {
    let mut usable: Option<usize> = None;
    store.for_each_in(&[DEVICE_NAME.key()], |_, tag, v| {
        usable = (tag == Type::Str.tag() && bridge_name_usable(v)).then(|| {
            buf[..v.len()].copy_from_slice(v);
            v.len()
        });
    });
    usable.unwrap_or_else(|| {
        buf[..default.len()].copy_from_slice(default);
        default.len()
    })
}

/// The bridge's Wi-Fi station as its store sets it up at boot (`specs/bridge-onboarding.md`
/// sections 2.1 and 2.4): the network in `wifi.ssid` and `wifi.pass` and the transmit power cap in
/// `wifi.tx_power`, or none. Its `Debug` prints the SSID, whether a password is set, never the
/// password, and the power.
#[derive(Clone, PartialEq, Eq)]
pub enum Station {
    /// No network: the station is not started, and `ESP_INFO` reads [`WifiState::NotConfigured`].
    Off,
    /// Join this network.
    Join {
        /// The network's name, 1 to [`WIFI_SSID_MAX`] bytes.
        ssid: String<WIFI_SSID_MAX>,
        /// Its password, empty for an open network.
        pass: String<WIFI_PASS_MAX>,
        /// The transmit power cap, applied before every join.
        power: TxPower,
    },
}

/// The station's transmit power cap as the store sets it at boot (`specs/bridge-onboarding.md`
/// section 2.4).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxPower {
    /// The cap the station applies, in quarter-dBm: `wifi.tx_power` when a bridge takes it
    /// ([`bridge_value_usable`]), else the field's default.
    pub applied: u8,
    /// A stored value the bridge does not take (written by an image without the check), which is
    /// not applied.
    pub refused: Option<u8>,
}

/// The transmit power cap the store sets, read once at boot: `wifi.tx_power`, unstored its default
/// (34, 8 dBm on the C3). A stored value outside the range is not applied, so it never leaves the bridge off
/// Wi-Fi: the default is, and [`TxPower::refused`] names the value for the boot log.
pub fn tx_power<F: Flash>(store: &Store<'_, F>) -> TxPower {
    let stored = store.get(WIFI_TX_POWER);
    if bridge_value_usable(WIFI_TX_POWER.key(), &Value::U8(stored)) {
        TxPower {
            applied: stored,
            refused: None,
        }
    } else {
        TxPower {
            applied: WIFI_TX_POWER.default(),
            refused: Some(stored),
        }
    }
}

impl core::fmt::Debug for Station {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Station::Off => f.write_str("Off"),
            Station::Join { ssid, pass, power } => f
                .debug_struct("Join")
                .field("ssid", &ssid.as_str())
                .field(
                    "pass",
                    &format_args!("{}", if pass.is_empty() { "(empty)" } else { "(set)" }),
                )
                .field("power", power)
                .finish(),
        }
    }
}

impl Station {
    /// The network to join, as the text the radio's station configuration takes: `None` for
    /// [`Station::Off`].
    pub fn network(&self) -> Option<(&str, &str)> {
        match self {
            Station::Off => None,
            Station::Join { ssid, pass, .. } => Some((ssid.as_str(), pass.as_str())),
        }
    }

    /// The transmit power cap: `None` for [`Station::Off`].
    pub fn power(&self) -> Option<TxPower> {
        match self {
            Station::Off => None,
            Station::Join { power, .. } => Some(*power),
        }
    }

    /// The station's state in `ESP_INFO` before it has run: not configured, or not joined yet.
    pub fn state(&self) -> WifiState {
        match self {
            Station::Off => WifiState::NotConfigured,
            Station::Join { .. } => WifiState::Down,
        }
    }
}

/// The station the store sets up, read once at boot: [`Station::Join`] when `wifi.ssid` holds a name
/// and both fields hold values a bridge takes ([`bridge_value_usable`], which takes only UTF-8), at
/// the power [`tx_power`] reads, else [`Station::Off`]. A value the bridge would refuse can be in a store written before the check
/// existed; it leaves the station off, as an empty name does.
pub fn station<F: Flash>(store: &Store<'_, F>) -> Station {
    fn text<const N: usize>(field: StrField, v: &[u8]) -> Option<String<N>> {
        if !bridge_value_usable(field.key(), &Value::Str(v)) {
            return None;
        }
        String::try_from(core::str::from_utf8(v).ok()?).ok()
    }
    let ssid = store.get_text(WIFI_SSID);
    if ssid.is_empty() {
        return Station::Off;
    }
    match (
        text(WIFI_SSID, ssid),
        text(WIFI_PASS, store.get_text(WIFI_PASS)),
    ) {
        (Some(ssid), Some(pass)) => Station::Join {
            ssid,
            pass,
            power: tx_power(store),
        },
        _ => Station::Off,
    }
}

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
    /// `NODE_HELLO` reports; `default_name` is what a read of an unstored `device.name` answers
    /// ([`default_name`]).
    pub fn new(
        store: Store<'f, F>,
        store_mounted: bool,
        fw_ver: u16,
        default_name: &'static [u8],
    ) -> Self {
        let mut resp = Responder::new(N_PORTS, PORT_KINDS, MCU_ESP32C3, CLASS_BRIDGE, fw_ver);
        resp.set_default_name(default_name);
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
    /// node, the probe window, the command service's idle work when no controller packet came
    /// ([`Service::idle`]: the replies of the pass before have gone out), and the flash job's next
    /// step.
    pub fn pump<P: SwdPins, Pl: Platform>(
        &mut self,
        router: &mut Router,
        port: &mut MailboxPort<P>,
        svc: &mut Service<Pl>,
    ) {
        let mut buf = [0u8; MAX_PACKET];
        let mut from_links = false;
        for link in PORTS {
            while let Some(n) = router.recv(link, &mut buf) {
                from_links = true;
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

        if !from_links {
            svc.idle();
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
