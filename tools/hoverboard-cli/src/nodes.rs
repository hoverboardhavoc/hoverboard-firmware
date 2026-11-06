//! The walk and the configuration requests (`specs/hoverboard-bridge.md`, "The terminal tool"): the
//! whole walk with `net`'s [`Controller`], then `IDENTIFY` and `device.name` asked of every
//! address; `CONFIG_READ`, `CONFIG_WRITE` (a value or an un-store), `CONFIG_COMPACT` and
//! `STATUS_READ`; and where the walk placed every node.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use net::pdu::{Opcode, Pdu};
use net::walk::{Controller, PduBuf, CLASS_BRIDGE};
pub use net::walk::{Identity, CLASS_MOTOR};
use store::{Key, Type, Value, DEVICE_NAME};

use crate::wire::Wire;

/// How long one walk or configuration reply may take. A `PROBE_PORTS` is answered after the
/// node's probe window (half a second on a board and on the bridge), and the walk's requests to the
/// far board cross the mailbox and the inter-board UART.
pub const REPLY_WAIT: Duration = Duration::from_secs(3);
/// How many times a walk or configuration request is sent before giving up.
const TRIES: u32 = 3;
/// The longest one wait for a packet inside the walk, so a resend is not late by much.
const POLL: Duration = Duration::from_millis(50);

/// A node the walk found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Found {
    /// Its address.
    pub addr: u8,
    /// Its `IDENTITY`, if it answered (`specs/machine-settings.md` section 6.1).
    pub identity: Option<Identity>,
    /// Its `device.name`, if it answered.
    pub name: Option<String>,
}

impl Found {
    /// Its class, if it answered `IDENTIFY`.
    pub fn class(&self) -> Option<u8> {
        self.identity.map(|i| i.class)
    }
}

/// Where the walk placed every node: the node the link is attached to, each node's walked
/// neighbours (`Controller::neighbours`), and each node's mailbox port, its `PORT_SWD` port
/// (`Controller::port_of_kind`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Topology {
    /// The node on the far end of the tool's own link (`Controller::gateway_addr`).
    pub gateway: Option<u8>,
    /// Each node's walked neighbours.
    pub neighbours: BTreeMap<u8, Vec<u8>>,
    /// Each node's mailbox port, where it has one.
    pub mailbox: BTreeMap<u8, u8>,
}

impl Topology {
    /// The placement `walked` recorded for each of its nodes.
    pub fn of(walked: &Controller) -> Self {
        let addrs = walked.assigned_addrs();
        Topology {
            gateway: walked.gateway_addr(),
            neighbours: addrs
                .iter()
                .map(|&a| (a, walked.neighbours(a).to_vec()))
                .collect(),
            mailbox: addrs
                .iter()
                .filter_map(|&a| Some((a, walked.port_of_kind(a, net::walk::PORT_SWD)?)))
                .collect(),
        }
    }

    /// The walked neighbours of `addr`.
    pub fn neighbours(&self, addr: u8) -> &[u8] {
        self.neighbours.get(&addr).map_or(&[], Vec::as_slice)
    }

    /// The mailbox port of `addr`.
    pub fn mailbox(&self, addr: u8) -> Option<u8> {
        self.mailbox.get(&addr).copied()
    }
}

/// A whole walk: the tool's guest address, every node with its `IDENTITY` and name, and where the
/// walk placed them.
#[derive(Debug, Clone)]
pub struct Walked {
    /// The tool's guest address.
    pub src: u8,
    /// Every node, in address order.
    pub found: Vec<Found>,
    /// Where they sit.
    pub topology: Topology,
}

/// The walk's first two exchanges and no more: first contact ([`hello`]), then the attached node's
/// own `PROBE_PORTS` ([`ports`]). What the reply placed is enough to tell the board on the attached
/// bridge's mailbox (`target::wired_here`); [`finish`] walks on from here.
pub fn start<W: Wire>(wire: &mut W) -> Result<Controller, String> {
    let c = hello(wire)?;
    ports(wire, c)
}

/// The walk's first contact alone: the attached node's `NODE_HELLO` reply, which grants the guest
/// address. A route by `uid` asks the attached node its `IDENTIFY` and `TARGET_ID` from here
/// (`specs/board-route.md` section 1); [`ports`] goes on.
pub fn hello<W: Wire>(wire: &mut W) -> Result<Controller, String> {
    walk_until(wire, Controller::new(), REPLY_WAIT, |_, replies| {
        replies >= 1
    })
}

/// The walk's next exchange after [`hello`]: the attached node's own `PROBE_PORTS`. An attached
/// node with no address sends nothing, since the walk's next request would assign it.
pub fn ports<W: Wire>(wire: &mut W, c: Controller) -> Result<Controller, String> {
    walk_until(wire, c, REPLY_WAIT, |c, replies| {
        replies >= 1 || c.gateway_addr().is_none()
    })
}

/// The rest of the walk [`start`] began, then ask every node its `IDENTITY` and read its name
/// ([`survey`]).
pub fn finish<W: Wire>(wire: &mut W, c: Controller) -> Result<Walked, String> {
    let c = walk_until(wire, c, REPLY_WAIT, |_, _| false)?;
    Ok(Walked {
        src: c.guest_addr(),
        found: survey(wire, &c),
        topology: Topology::of(&c),
    })
}

/// A factory unique ID as 24 lowercase hex digits (`specs/machine-settings.md` section 6.2).
pub fn uid_hex(uid: &[u8; net::walk::UID_LEN]) -> String {
    uid.iter().map(|b| format!("{b:02x}")).collect()
}

/// The name of an `IDENTITY` class.
pub fn class_name(class: Option<u8>) -> &'static str {
    match class {
        Some(CLASS_MOTOR) => "motor controller",
        Some(CLASS_BRIDGE) => "bridge",
        Some(_) => "unknown class",
        None => "no answer",
    }
}

/// The name of a `CONFIG_RESP` status byte (`shared/net/src/walk.rs`, the `CFG_*` statuses).
pub fn status_name(status: u8) -> String {
    use net::walk::*;
    match status {
        CFG_OK => "ok".into(),
        CFG_BAD => "bad request".into(),
        CFG_UNKNOWN_KEY => "unknown key".into(),
        CFG_TYPE_MISMATCH => "type mismatch".into(),
        CFG_STORE_ERR => "store error".into(),
        CFG_ARMED => "armed".into(),
        CFG_NO_SPACE => "no space".into(),
        CFG_COMMIT_FAILED => "commit failed".into(),
        CFG_DEFAULT => "default".into(),
        CFG_SECRET => "secret, not readable".into(),
        s => format!("status {s}"),
    }
}

/// The status of a `device.name` write to a node of `class`, by name. A bridge answers
/// [`net::walk::CFG_BAD`] for a name it cannot advertise ([`net::walk::bridge_name_usable`]), so
/// that status names the rule.
pub fn name_status(class: Option<u8>, status: u8) -> String {
    use net::walk::{BRIDGE_NAME_MAX, CFG_BAD};
    if class == Some(CLASS_BRIDGE) && status == CFG_BAD {
        format!("name must be 1 to {BRIDGE_NAME_MAX} letters, digits or '-'")
    } else {
        status_name(status)
    }
}

/// Run the whole walk over `wire`: first contact, then every node addressed and probed. Answers
/// the probes of the tool's own link as the walk goes. A request with no answer in
/// [`REPLY_WAIT`] is sent again, [`TRIES`] times in all.
pub fn walk<W: Wire>(wire: &mut W) -> Result<Controller, String> {
    walk_within(wire, REPLY_WAIT)
}

/// [`walk`] with the resend timeout `reply_wait`.
///
/// The timeout runs off the outstanding request's own age, so a request is resent when it is due
/// whether or not other packets arrived meanwhile (`specs/l3.md`, "A reply is bound to the request
/// it answers"): a board streams `CYCLIC_STATE` at 5 Hz, and a loop that resent only on a quiet
/// `recv` never resent a request whose reply was lost. A packet the controller does not take as
/// the outstanding request's reply (telemetry, a duplicate or late reply) is dropped here; the
/// walk runs before `watch` and `drive` start counting.
fn walk_within<W: Wire>(wire: &mut W, reply_wait: Duration) -> Result<Controller, String> {
    walk_until(wire, Controller::new(), reply_wait, |_, _| false)
}

/// Walk on from `c` until it is complete or `stop` says so, given the controller and how many
/// replies this call has taken.
fn walk_until<W: Wire>(
    wire: &mut W,
    mut c: Controller,
    reply_wait: Duration,
    stop: impl Fn(&Controller, u32) -> bool,
) -> Result<Controller, String> {
    let mut last: Option<(Vec<u8>, Instant, u32)> = None;
    let mut replies = 0;
    while !c.is_complete() && !stop(&c, replies) {
        if let Some(req) = c.next_request() {
            wire.send(&req);
            last = Some((req.to_vec(), Instant::now(), 1));
        }
        if let Some(pkt) = wire.recv(POLL) {
            if let Some(r) = c.reply_to_probe(&pkt) {
                wire.send(&r);
            } else if c.on_reply(&pkt) {
                last = None;
                replies += 1;
            }
        }
        if let Some((req, at, tries)) = last.as_mut() {
            if at.elapsed() > reply_wait {
                if *tries >= TRIES {
                    return Err(format!(
                        "the walk got no answer to {}",
                        describe(req).unwrap_or_default()
                    ));
                }
                wire.send(req);
                *at = Instant::now();
                *tries += 1;
            }
        }
    }
    Ok(c)
}

/// A request named for an error message: its opcode and destination.
fn describe(pdu: &[u8]) -> Option<String> {
    let p = Pdu::decode(pdu).ok()?;
    Some(format!("{:?} to {:#04x}", p.known()?, p.dst))
}

/// A `CONFIG_RESP`: its status, and the value's type tag and bytes when there is one. The owned
/// form of [`net::walk::ConfigResp`], which decodes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigResp {
    /// The `CFG_*` status.
    pub status: u8,
    /// The type tag, `0` with no value.
    pub tag: u8,
    /// The value bytes.
    pub bytes: Vec<u8>,
}

impl ConfigResp {
    /// The value, decoded.
    pub fn value(&self) -> Option<Value<'_>> {
        Value::decode(Type::from_tag(self.tag)?, &self.bytes)
    }
}

/// Send the request `pdu` from `src` to `dst` (encoded by [`net::walk::request`], `None` when it
/// did not fit one PDU) and wait for the reply `answer` takes: a `reply` frame to that request, as
/// [`net::walk::is_reply`] matches it. Sent [`TRIES`] times, each waiting [`REPLY_WAIT`].
fn ask<W: Wire, T>(
    wire: &mut W,
    pdu: Option<PduBuf>,
    reply: Opcode,
    (src, dst): (u8, u8),
    answer: impl Fn(&[u8]) -> Option<T>,
) -> Result<T, String> {
    let pdu = pdu.ok_or_else(|| "the value does not fit one PDU".to_string())?;
    for _ in 0..TRIES {
        wire.send(&pdu);
        let deadline = Instant::now() + REPLY_WAIT;
        while let Some(pkt) = wire.recv(deadline.saturating_duration_since(Instant::now())) {
            let Ok(p) = Pdu::decode(&pkt) else { continue };
            if net::walk::is_reply(&p, reply, src, dst) {
                if let Some(t) = answer(p.payload) {
                    return Ok(t);
                }
            }
        }
    }
    Err(format!("no {reply:?} from {dst:#04x}"))
}

/// Send the `CONFIG_*` request `pdu` from `src` to `dst` and wait for the `CONFIG_RESP` whose first
/// two bytes are `echo` (the key, or `[0, 0]` for the acks that name none), as
/// [`net::walk::ConfigResp::answering`] matches it.
fn exchange<W: Wire>(
    wire: &mut W,
    pdu: Option<PduBuf>,
    src: u8,
    dst: u8,
    echo: [u8; 2],
) -> Result<ConfigResp, String> {
    ask(wire, pdu, Opcode::ConfigResp, (src, dst), |p| {
        net::walk::ConfigResp::echoing(p, echo).map(|r| ConfigResp {
            status: r.status,
            tag: r.tag,
            bytes: r.bytes.to_vec(),
        })
    })
    .map_err(|_| format!("no CONFIG_RESP from {dst:#04x}"))
}

/// `IDENTIFY` to `dst`: its `IDENTITY` (`specs/machine-settings.md` section 6.1).
pub fn identify<W: Wire>(wire: &mut W, src: u8, dst: u8) -> Result<Identity, String> {
    let pdu = net::walk::request(Opcode::Identify, src, dst, &[]);
    ask(wire, pdu, Opcode::Identity, (src, dst), Identity::decode)
}

/// `STATUS_READ` to `dst`: the boot status record's entries, four bytes each
/// (`specs/configured-links.md` section 5).
pub fn status_read<W: Wire>(wire: &mut W, src: u8, dst: u8) -> Result<Vec<u8>, String> {
    let pdu = net::walk::request(Opcode::StatusRead, src, dst, &[]);
    ask(wire, pdu, Opcode::Status, (src, dst), |p| Some(p.to_vec()))
}

/// Send a `CONFIG_*` request for `key` and wait for its `CONFIG_RESP`.
fn config<W: Wire>(
    wire: &mut W,
    op: Opcode,
    src: u8,
    dst: u8,
    key: Key,
    value: &[u8],
) -> Result<ConfigResp, String> {
    let pdu = net::walk::config_request(op, src, dst, key, value);
    exchange(wire, pdu, src, dst, [key.field_id, key.index])
}

/// `CONFIG_READ` of `key` at `dst`.
pub fn read<W: Wire>(wire: &mut W, src: u8, dst: u8, key: Key) -> Result<ConfigResp, String> {
    config(wire, Opcode::ConfigRead, src, dst, key, &[])
}

/// `CONFIG_WRITE` of a `kind` value, `bytes` as the wire carries it, to `key` at `dst`.
pub fn write<W: Wire>(
    wire: &mut W,
    src: u8,
    dst: u8,
    key: Key,
    kind: Type,
    bytes: &[u8],
) -> Result<ConfigResp, String> {
    let mut v = vec![kind.tag()];
    v.extend_from_slice(bytes);
    config(wire, Opcode::ConfigWrite, src, dst, key, &v)
}

/// `CONFIG_WRITE` of the tombstone tag with no value: un-store `key` at `dst`
/// (`specs/cli-config.md` section 6.2).
pub fn unstore<W: Wire>(wire: &mut W, src: u8, dst: u8, key: Key) -> Result<ConfigResp, String> {
    config(
        wire,
        Opcode::ConfigWrite,
        src,
        dst,
        key,
        &[store::ERASED_TAG],
    )
}

/// `CONFIG_COMPACT` at `dst` (`specs/cli-config.md` section 6.3): the status of its
/// `[0, 0, status, 0]` ack.
pub fn compact<W: Wire>(wire: &mut W, src: u8, dst: u8) -> Result<u8, String> {
    let pdu = net::walk::request(Opcode::ConfigCompact, src, dst, &[]);
    exchange(wire, pdu, src, dst, [0, 0]).map(|r| r.status)
}

/// A directed `ASSIGN(egress, new_addr)` from `src` to `relay`, as the walk assigns a frontier node
/// (`specs/l3.md`, "Discovery and address assignment", step 3), and the status of its
/// `ASSIGN_ACK`. Resent on [`REPLY_WAIT`], [`TRIES`] times in all: a repeated `ASSIGN` re-persists
/// the same address and is acknowledged again.
pub fn assign<W: Wire>(
    wire: &mut W,
    src: u8,
    relay: u8,
    egress: u8,
    new_addr: u8,
) -> Result<u8, String> {
    let req = net::walk::assign_request(src, relay, egress, new_addr);
    for _ in 0..TRIES {
        wire.send(&req);
        let deadline = Instant::now() + REPLY_WAIT;
        while let Some(pkt) = wire.recv(deadline.saturating_duration_since(Instant::now())) {
            let Ok(p) = Pdu::decode(&pkt) else { continue };
            if let Some(status) = net::walk::assign_ack_status(&p, src, new_addr) {
                return Ok(status);
            }
        }
    }
    Err(format!("no ASSIGN_ACK from {new_addr:#04x}"))
}

/// Ask every address the walk found for its `IDENTITY` and read its `device.name`, in address
/// order (`specs/machine-settings.md` section 6.1).
pub fn survey<W: Wire>(wire: &mut W, walked: &Controller) -> Vec<Found> {
    let src = walked.guest_addr();
    let mut addrs = walked.assigned_addrs().to_vec();
    addrs.sort_unstable();
    addrs
        .into_iter()
        .map(|addr| {
            let identity = identify(wire, src, addr).ok();
            let name =
                read(wire, src, addr, DEVICE_NAME.key())
                    .ok()
                    .and_then(|r| match r.value() {
                        Some(Value::Str(s)) => Some(String::from_utf8_lossy(s).into_owned()),
                        _ => None,
                    });
            Found {
                addr,
                identity,
                name,
            }
        })
        .collect()
}

/// A number: decimal, or hexadecimal with `0x`.
pub fn parse_int(s: &str) -> Option<i128> {
    let (neg, digits) = match s.strip_prefix('-') {
        Some(d) => (true, d),
        None => (false, s),
    };
    let v = match digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        Some(h) => i128::from_str_radix(h, 16).ok()?,
        None => digits.parse::<i128>().ok()?,
    };
    Some(if neg { -v } else { v })
}

/// A field argument: `<id>` or `<id>.<index>`, each a number.
pub fn parse_key(s: &str) -> Result<Key, String> {
    let (id, index) = s.split_once('.').unwrap_or((s, "0"));
    let byte = |t: &str| {
        parse_int(t)
            .and_then(|v| u8::try_from(v).ok())
            .ok_or_else(|| format!("not a field: {s}"))
    };
    Ok(Key {
        field_id: byte(id)?,
        index: byte(index)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use net::pdu::NO_ADDRESS;
    use net::walk::{NB_ASSIGNED, NB_EMPTY, PORT_BLE, PORT_SWD, PORT_UART, PROTO_VER};
    use std::collections::VecDeque;

    /// One board at `0x01` behind a link that streams telemetry without pause and loses the
    /// reply to the first `PROBE_PORTS`.
    struct Chatty {
        replies: VecDeque<Vec<u8>>,
        probes: u32,
    }

    fn encode(op: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; net::walk::MAX_PDU];
        let n = Pdu::new(op, src, dst, payload)
            .unwrap()
            .encode(&mut buf)
            .unwrap();
        buf[..n].to_vec()
    }

    impl Wire for Chatty {
        fn send(&mut self, pdu: &[u8]) {
            let p = Pdu::decode(pdu).unwrap();
            match p.known() {
                Some(Opcode::NodeHello) => self.replies.push_back(encode(
                    Opcode::NodeHello.to_u8(),
                    0x01,
                    p.src,
                    &[0x01, PROTO_VER, 0x88],
                )),
                Some(Opcode::ProbePorts) => {
                    self.probes += 1;
                    if self.probes > 1 {
                        self.replies.push_back(encode(
                            Opcode::Ports.to_u8(),
                            0x01,
                            p.src,
                            &[1, 0, PORT_UART, NB_ASSIGNED, 0x88],
                        ));
                    }
                }
                _ => {}
            }
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            Some(self.replies.pop_front().unwrap_or_else(|| {
                encode(linkctl::OP_CYCLIC_STATE, 0x01, net::pdu::BROADCAST, &[0; 8])
            }))
        }
        fn window(&self) -> usize {
            1
        }
    }

    #[test]
    fn a_lost_probe_reply_is_resent_while_telemetry_keeps_arriving() {
        let mut wire = Chatty {
            replies: VecDeque::new(),
            probes: 0,
        };
        let c = walk_within(&mut wire, Duration::from_millis(100)).expect("the walk completes");
        assert_eq!(wire.probes, 2, "the probe was resent once");
        assert_eq!(c.assigned_addrs().to_vec(), [0x01]);
        assert_eq!(c.guest_addr(), 0x88);
    }

    /// A bridge on the tool's link holding `addr` (`NO_ADDRESS` for none): `NODE_HELLO` granting
    /// guest `0x88`, its own `PROBE_PORTS` answered with `ports`, and any other node's with no
    /// ports. Counts the requests by opcode.
    struct Bridge {
        addr: u8,
        ports: Vec<u8>,
        replies: VecDeque<Vec<u8>>,
        sent: Vec<Opcode>,
    }

    impl Bridge {
        /// A bridge at `0x03`: BLE empty, the mailbox (port 1) neighbour `swd`, the tool on its
        /// console.
        fn at_0x03(swd: (u8, u8)) -> Self {
            Bridge {
                addr: 0x03,
                ports: vec![
                    3,
                    0,
                    PORT_BLE,
                    NB_EMPTY,
                    0,
                    1,
                    PORT_SWD,
                    swd.0,
                    swd.1,
                    2,
                    PORT_UART,
                    NB_ASSIGNED,
                    0x88,
                ],
                replies: VecDeque::new(),
                sent: Vec::new(),
            }
        }
    }

    impl Wire for Bridge {
        fn send(&mut self, pdu: &[u8]) {
            let p = Pdu::decode(pdu).unwrap();
            let op = p.known().unwrap();
            self.sent.push(op);
            let reply = match op {
                Opcode::NodeHello => encode(
                    Opcode::NodeHello.to_u8(),
                    self.addr,
                    p.src,
                    &[self.addr, PROTO_VER, 0x88],
                ),
                Opcode::ProbePorts if p.dst == self.addr => {
                    encode(Opcode::Ports.to_u8(), self.addr, p.src, &self.ports)
                }
                Opcode::ProbePorts => encode(Opcode::Ports.to_u8(), p.dst, p.src, &[0]),
                _ => return,
            };
            self.replies.push_back(reply);
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            self.replies.pop_front()
        }
        fn window(&self) -> usize {
            1
        }
    }

    #[test]
    fn start_stops_at_the_attached_bridges_ports_and_names_the_board_on_its_mailbox() {
        let mut wire = Bridge::at_0x03((NB_ASSIGNED, 0x01));
        let c = start(&mut wire).unwrap();
        assert_eq!(wire.sent, [Opcode::NodeHello, Opcode::ProbePorts]);
        assert_eq!(crate::target::wired_here(&c, 0x01), Some((0x03, 1)));
        // Another address, the bridge's own and the tool's are not on its mailbox.
        for other in [0x02, 0x03, 0x88] {
            assert_eq!(crate::target::wired_here(&c, other), None, "{other:#04x}");
        }
    }

    #[test]
    fn a_mailbox_with_no_neighbour_names_no_board_and_the_walk_goes_on_from_the_start() {
        for swd in [(NB_EMPTY, 0), (net::walk::NB_UNASSIGNED, 0)] {
            let mut wire = Bridge::at_0x03(swd);
            let c = start(&mut wire).unwrap();
            assert_eq!(crate::target::wired_here(&c, 0x01), None, "{swd:?}");
        }
        // Another board on the mailbox: no route to 0x01, and finishing probes that board without
        // a second first contact.
        let mut wire = Bridge::at_0x03((NB_ASSIGNED, 0x02));
        let c = start(&mut wire).unwrap();
        assert_eq!(crate::target::wired_here(&c, 0x01), None);
        let w = finish(&mut wire, c).unwrap();
        let addrs: Vec<u8> = w.found.iter().map(|f| f.addr).collect();
        assert_eq!(addrs, [0x02, 0x03]);
        assert_eq!(
            wire.sent[..3],
            [Opcode::NodeHello, Opcode::ProbePorts, Opcode::ProbePorts]
        );
        assert_eq!(
            wire.sent
                .iter()
                .filter(|&&o| o == Opcode::NodeHello)
                .count(),
            1
        );
    }

    #[test]
    fn start_sends_nothing_after_first_contact_with_a_node_holding_no_address() {
        let mut wire = Bridge::at_0x03((NB_ASSIGNED, 0x01));
        wire.addr = NO_ADDRESS;
        let c = start(&mut wire).unwrap();
        assert_eq!(wire.sent, [Opcode::NodeHello]);
        assert_eq!(crate::target::wired_here(&c, 0x01), None);
    }
}
