//! One connection's L3 conversation (`specs/handheld-remote.md` section 3.5): the full walk with
//! `net::walk::Controller`, `IDENTIFY` at every address, then the ride facts from every motor
//! controller, one request at a time with a 1 s reply timeout and three tries each (the app's
//! `L3Session`), and the re-walk every 3 s while no motor controller is known.
//!
//! Sans-IO: the caller feeds it every received PDU ([`Session::on_pdu`]) and polls it with the
//! time ([`Session::poll`]); each hands back the PDUs to send.

use heapless::Vec;
use net::pdu::{Opcode, Pdu};
use net::walk::{config_read, is_reply, request, ConfigResp, Controller, Identity, PduBuf};
pub use net::walk::{CLASS_BRIDGE, CLASS_MOTOR};
use store::{Key, Value};

/// How long a request waits for its reply before it is sent again, ms.
pub const REPLY_WAIT_MS: u32 = 1000;
/// Sends of one request before it is given up.
pub const TRIES: u8 = 3;
/// The re-walk period while no motor controller is known, ms.
pub const REWALK_MS: u32 = 3000;
/// The most nodes a walk places (`net::walk::MAX_NODES`).
pub const MAX_NODES: usize = net::walk::MAX_NODES;
/// The most motor controllers whose facts are held.
pub const MAX_MCS: usize = 4;

/// A ride fact (section 3.5), in the order they are read.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Fact {
    Frame,
    Side,
    Mode,
    PolePairs,
    WheelDiameter,
    CurrentLimit,
    CurrentCal,
    Cells,
    Chemistry,
    Floor,
    ImuModel,
}

/// The ride facts, read in this order, with their store keys (`shared/store/src/field.rs`).
pub const FACTS: [(Fact, Key); 11] = [
    (Fact::Frame, store::CONTROL_FRAME.key()),
    (Fact::Side, store::CONTROL_SIDE.key()),
    (Fact::Mode, store::CONTROL_MODE.key()),
    (Fact::PolePairs, store::field::MOTOR_POLE_PAIRS.key()),
    (Fact::WheelDiameter, store::field::WHEEL_DIAMETER_MM.key()),
    (Fact::CurrentLimit, store::MOTOR_CURRENT_LIMIT.key()),
    (Fact::CurrentCal, store::MOTOR_CURRENT_CAL.key()),
    (Fact::Cells, store::BOARD_BATTERY_CELLS.key()),
    (Fact::Chemistry, store::field::BOARD_BATTERY_CHEMISTRY.key()),
    (Fact::Floor, store::BOARD_BATTERY_FLOOR.key()),
    (Fact::ImuModel, store::IMU_MODEL.key()),
];

/// One node the walk placed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Node {
    pub addr: u8,
    /// Its `IDENTITY` class (`specs/machine-settings.md` section 6.1), `None` while unread or after
    /// its `IDENTIFY` failed.
    pub class: Option<u8>,
}

/// One motor controller's ride facts, `None` where unread.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Facts {
    pub addr: u8,
    pub values: [Option<i64>; FACTS.len()],
}

impl Facts {
    /// The value of `f`, if read.
    pub fn get(&self, f: Fact) -> Option<i64> {
        let i = FACTS.iter().position(|(g, _)| *g == f)?;
        self.values[i]
    }
}

/// Where the conversation is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    /// The walk is running.
    Walking,
    /// Asking every node its `IDENTITY`, `n` of `total` done.
    Classes { n: u8, total: u8 },
    /// Reading the ride facts, `n` of `total` done.
    Facts { n: u8, total: u8 },
    /// Everything read (some reads may have failed).
    Ready,
    /// The walk failed; it runs again at `at`.
    Retry { at: u32 },
}

/// What is outstanding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Waiting {
    Walk,
    Class { node: usize },
    Fact { mc: usize, fact: usize },
}

#[derive(Clone, Debug)]
struct Out {
    pdu: PduBuf,
    at: u32,
    tries: u8,
    what: Waiting,
}

/// PDUs to send, a step's worth.
pub type Sends = Vec<PduBuf, 4>;

/// The conversation.
pub struct Session {
    walk: Controller,
    phase: Phase,
    nodes: Vec<Node, MAX_NODES>,
    facts: Vec<Facts, MAX_MCS>,
    out: Option<Out>,
    /// When the node list was last found without a motor controller (the re-walk clock).
    quiet_since: Option<u32>,
}

impl Default for Session {
    fn default() -> Self {
        Session::new()
    }
}

/// An integer stored value.
fn int(v: Value) -> Option<i64> {
    Some(match v {
        Value::U8(x) => x.into(),
        Value::U16(x) => x.into(),
        Value::U32(x) => x.into(),
        Value::U64(x) => x as i64,
        Value::I16(x) => x.into(),
        Value::I32(x) => x.into(),
        Value::I64(x) => x,
        Value::Bool(b) => b.into(),
        Value::Str(_) | Value::Bytes(_) | Value::Pin(_) | Value::PinAf(_) | Value::PinAdc(_) => {
            return None
        }
    })
}

impl Session {
    /// A new connection, about to make first contact.
    pub fn new() -> Self {
        Session {
            walk: Controller::new(),
            phase: Phase::Walking,
            nodes: Vec::new(),
            facts: Vec::new(),
            out: None,
            quiet_since: None,
        }
    }

    /// Where the conversation is.
    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// The guest address the bridge granted, once it has.
    pub fn src(&self) -> Option<u8> {
        self.walk.granted()
    }

    /// The nodes the last walk placed, by address.
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    /// The motor controllers' facts.
    pub fn facts(&self) -> &[Facts] {
        &self.facts
    }

    /// The facts of the motor controller at `addr`.
    pub fn facts_of(&self, addr: u8) -> Option<&Facts> {
        self.facts.iter().find(|f| f.addr == addr)
    }

    /// Whether the walk, the classes and the ride facts are done.
    pub fn ready(&self) -> bool {
        self.phase == Phase::Ready
    }

    /// Whether any node's class is unread (a possible motor controller).
    pub fn class_unread(&self) -> bool {
        self.nodes.iter().any(|n| n.class.is_none())
    }

    /// The motor controllers, by address.
    pub fn motor_controllers(&self) -> impl Iterator<Item = u8> + '_ {
        self.nodes
            .iter()
            .filter(|n| n.class == Some(CLASS_MOTOR))
            .map(|n| n.addr)
    }

    /// Take one received PDU. Returns the PDUs to send in answer; `consumed` is whether it was a
    /// reply this session took (a probe of its own port, a walk reply, or an `IDENTITY` or
    /// `CONFIG_RESP` it waited for), so the caller files everything else as telemetry.
    pub fn on_pdu(&mut self, frame: &[u8], now: u32, sends: &mut Sends) -> bool {
        if let Some(r) = self.walk.reply_to_probe(frame) {
            let _ = sends.push(r);
            return true;
        }
        let Some(o) = self.out.as_ref() else {
            return false;
        };
        match o.what {
            Waiting::Walk => {
                if !self.walk.on_reply(frame) {
                    return false;
                }
                self.out = None;
                self.advance(now, sends);
                true
            }
            Waiting::Class { node } => {
                let addr = self.nodes[node].addr;
                let Some(identity) = self.identity(frame, addr) else {
                    return false;
                };
                self.nodes[node].class = identity.map(|i| i.class);
                self.out = None;
                self.advance(now, sends);
                true
            }
            Waiting::Fact { mc, fact } => {
                let addr = self.facts[mc].addr;
                let Some(v) = self.answer(frame, addr, FACTS[fact].1) else {
                    return false;
                };
                self.facts[mc].values[fact] = v;
                self.out = None;
                self.advance(now, sends);
                true
            }
        }
    }

    /// The identity in `frame` if it is the `IDENTITY` answering our `IDENTIFY` of `addr`:
    /// `Some(None)` for a reply that does not decode, `None` for a frame that is not the reply.
    fn identity(&self, frame: &[u8], addr: u8) -> Option<Option<Identity>> {
        let src = self.walk.granted()?;
        let pdu = Pdu::decode(frame).ok()?;
        is_reply(&pdu, Opcode::Identity, src, addr).then(|| Identity::decode(pdu.payload))
    }

    /// The value in `frame` if it is the `CONFIG_RESP` to our read of `key` at `addr`: `Some(None)`
    /// for a reply with no usable value, `None` for a frame that is not the reply.
    fn answer(&self, frame: &[u8], addr: u8, key: Key) -> Option<Option<i64>> {
        let src = self.walk.granted()?;
        let pdu = Pdu::decode(frame).ok()?;
        let r = ConfigResp::answering(&pdu, src, addr, [key.field_id, key.index])?;
        Some(
            if r.status == net::walk::CFG_OK || r.status == net::walk::CFG_DEFAULT {
                r.value().and_then(int)
            } else {
                None
            },
        )
    }

    /// Run the clock at `now`: resend a request whose reply is late, give one up after
    /// [`TRIES`], start the re-walk when it is due. `armed` holds every re-walk off;
    /// `mc_heard` is whether a motor controller is in the inbox.
    pub fn poll(&mut self, now: u32, armed: bool, mc_heard: bool, sends: &mut Sends) {
        if let Some(o) = self.out.as_mut() {
            if now.wrapping_sub(o.at) >= REPLY_WAIT_MS {
                if o.tries < TRIES {
                    o.tries += 1;
                    o.at = now;
                    let _ = sends.push(o.pdu.clone());
                } else {
                    let what = o.what;
                    self.out = None;
                    self.give_up(what, now, sends);
                }
            }
            return;
        }
        match self.phase {
            Phase::Walking => self.advance(now, sends),
            Phase::Retry { at } if now.wrapping_sub(at) as i32 >= 0 && !armed => {
                self.restart(now, sends)
            }
            Phase::Ready => {
                let known = self.motor_controllers().next().is_some() || self.class_unread();
                if known || mc_heard || armed {
                    self.quiet_since = None;
                    return;
                }
                let since = *self.quiet_since.get_or_insert(now);
                if now.wrapping_sub(since) >= REWALK_MS {
                    self.restart(now, sends);
                }
            }
            _ => {}
        }
    }

    /// Walk again from first contact, and read everything again.
    fn restart(&mut self, now: u32, sends: &mut Sends) {
        *self = Session::new();
        self.advance(now, sends);
    }

    /// A request got no reply in [`TRIES`] sends.
    fn give_up(&mut self, what: Waiting, now: u32, sends: &mut Sends) {
        match what {
            // The walk cannot go on without its reply: it runs again from first contact later.
            Waiting::Walk => {
                self.phase = Phase::Retry {
                    at: now.wrapping_add(REWALK_MS),
                }
            }
            // A read that fails its three tries leaves that value unread.
            Waiting::Class { .. } | Waiting::Fact { .. } => self.advance(now, sends),
        }
    }

    fn send(&mut self, pdu: PduBuf, what: Waiting, now: u32, sends: &mut Sends) {
        let _ = sends.push(pdu.clone());
        self.out = Some(Out {
            pdu,
            at: now,
            tries: 1,
            what,
        });
    }

    /// Send the next request the conversation needs, moving through its phases.
    fn advance(&mut self, now: u32, sends: &mut Sends) {
        if self.phase == Phase::Walking {
            if let Some(req) = self.walk.next_request() {
                self.send(req, Waiting::Walk, now, sends);
                return;
            }
            if !self.walk.is_complete() {
                return;
            }
            let mut addrs = self.walk.assigned_addrs();
            addrs.sort_unstable();
            self.nodes = addrs
                .iter()
                .map(|&addr| Node { addr, class: None })
                .collect();
            self.phase = Phase::Classes {
                n: 0,
                total: self.nodes.len() as u8,
            };
        }
        let Some(src) = self.walk.granted() else {
            return;
        };
        if let Phase::Classes { n, total } = self.phase {
            if (n as usize) < self.nodes.len() {
                let node = n as usize;
                self.phase = Phase::Classes { n: n + 1, total };
                // An empty request always fits.
                let pdu =
                    request(Opcode::Identify, src, self.nodes[node].addr, &[]).unwrap_or_default();
                self.send(pdu, Waiting::Class { node }, now, sends);
                return;
            }
            self.facts = self
                .nodes
                .iter()
                .filter(|n| n.class == Some(CLASS_MOTOR))
                .map(|n| Facts {
                    addr: n.addr,
                    values: [None; FACTS.len()],
                })
                .collect();
            self.phase = Phase::Facts {
                n: 0,
                total: (self.facts.len() * FACTS.len()) as u8,
            };
        }
        if let Phase::Facts { n, total } = self.phase {
            if n < total {
                let (mc, fact) = (n as usize / FACTS.len(), n as usize % FACTS.len());
                self.phase = Phase::Facts { n: n + 1, total };
                let pdu = config_read(src, self.facts[mc].addr, FACTS[fact].1);
                self.send(pdu, Waiting::Fact { mc, fact }, now, sends);
                return;
            }
            self.phase = Phase::Ready;
        }
    }

    /// Whether `addr` is a bridge by its class.
    pub fn is_bridge(&self, addr: u8) -> bool {
        self.nodes
            .iter()
            .any(|n| n.addr == addr && n.class == Some(CLASS_BRIDGE))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec as StdVec;

    /// A machine the walk finds whole on its first request: a bridge at 0x01 that holds its address
    /// and reports no neighbours, and two motor controllers at 0x02 and 0x03 reached through it.
    /// It answers what the session sends, and is told which requests to ignore.
    struct Machine {
        guest: u8,
        drop_identifies: u8,
    }

    fn pdu(op: Opcode, src: u8, dst: u8, payload: &[u8]) -> StdVec<u8> {
        let mut b = [0u8; 64];
        let n = Pdu::from_op(op, src, dst, payload).encode(&mut b).unwrap();
        b[..n].to_vec()
    }

    /// An `IDENTITY` payload of `class`.
    fn identity(class: u8) -> [u8; Identity::LEN] {
        Identity {
            class,
            mcu: net::walk::MCU_F1X0,
            fw_ver: 1,
            uid: [class; net::walk::UID_LEN],
        }
        .encode()
    }

    impl Machine {
        fn answer(&mut self, req: &[u8]) -> Option<StdVec<u8>> {
            let p = Pdu::decode(req).unwrap();
            match p.known()? {
                Opcode::NodeHello => {
                    Some(pdu(Opcode::NodeHello, 0x01, p.src, &[0x01, 1, self.guest]))
                }
                Opcode::ProbePorts => {
                    // 0x01 reports 0x02 on port 0 and the guest on port 1; 0x02 reports 0x03.
                    let entries: StdVec<[u8; 4]> = match p.dst {
                        0x01 => std::vec![[0, 0, 2, 0x02], [1, 1, 2, self.guest]],
                        0x02 => std::vec![[0, 0, 2, 0x01], [1, 0, 2, 0x03]],
                        _ => std::vec![[0, 0, 2, 0x02]],
                    };
                    let mut pl = std::vec![entries.len() as u8];
                    for e in entries {
                        pl.extend_from_slice(&e);
                    }
                    Some(pdu(Opcode::Ports, p.dst, p.src, &pl))
                }
                Opcode::Identify => {
                    if self.drop_identifies > 0 {
                        self.drop_identifies -= 1;
                        return None;
                    }
                    Some(pdu(
                        Opcode::Identity,
                        p.dst,
                        p.src,
                        &identity(if p.dst == 0x01 {
                            CLASS_BRIDGE
                        } else {
                            CLASS_MOTOR
                        }),
                    ))
                }
                Opcode::ConfigRead => {
                    let key = [p.payload[0], p.payload[1]];
                    let value = key[0]; // each fact reads back its own field id
                    Some(pdu(
                        Opcode::ConfigResp,
                        p.dst,
                        p.src,
                        &[
                            key[0],
                            key[1],
                            net::walk::CFG_OK,
                            store::Type::U8.tag(),
                            value,
                        ],
                    ))
                }
                _ => None,
            }
        }
    }

    /// Run the session against `m` until it is ready or `ms` passes, 10 ms a step.
    fn run(s: &mut Session, m: &mut Machine, start: u32, ms: u32) -> (u32, usize) {
        let mut now = start;
        let mut sent = 0;
        let mut inbox: StdVec<StdVec<u8>> = StdVec::new();
        while now < start + ms && !s.ready() {
            let mut sends = Sends::new();
            for f in inbox.drain(..) {
                s.on_pdu(&f, now, &mut sends);
            }
            s.poll(now, false, false, &mut sends);
            for req in sends {
                sent += 1;
                if let Some(r) = m.answer(&req) {
                    inbox.push(r);
                }
            }
            now += 10;
        }
        (now, sent)
    }

    #[test]
    fn the_walk_then_the_classes_then_the_ride_facts() {
        let mut s = Session::new();
        let mut m = Machine {
            guest: 0x81,
            drop_identifies: 0,
        };
        let (_, sent) = run(&mut s, &mut m, 0, 5000);
        assert!(s.ready());
        assert_eq!(s.src(), Some(0x81));
        let nodes: StdVec<(u8, Option<u8>)> = s.nodes().iter().map(|n| (n.addr, n.class)).collect();
        assert_eq!(
            nodes,
            [
                (0x01, Some(CLASS_BRIDGE)),
                (0x02, Some(CLASS_MOTOR)),
                (0x03, Some(CLASS_MOTOR))
            ]
        );
        assert_eq!(s.motor_controllers().collect::<StdVec<_>>(), [0x02, 0x03]);
        let f = s.facts_of(0x03).unwrap();
        assert_eq!(
            f.get(Fact::Mode),
            Some(store::CONTROL_MODE.key().field_id as i64)
        );
        assert_eq!(
            f.get(Fact::ImuModel),
            Some(store::IMU_MODEL.key().field_id as i64)
        );
        // Hello, three probes, three IDENTIFYs, 22 facts.
        assert_eq!(sent, 1 + 3 + 3 + 2 * FACTS.len());
    }

    #[test]
    fn a_lost_reply_is_sent_again_after_a_second_and_three_losses_leave_it_unread() {
        let mut s = Session::new();
        let mut m = Machine {
            guest: 0x81,
            drop_identifies: 2,
        };
        run(&mut s, &mut m, 0, 10_000);
        assert!(s.ready());
        assert_eq!(
            s.nodes()[0].class,
            Some(CLASS_BRIDGE),
            "the third try answered"
        );

        let mut s = Session::new();
        let mut m = Machine {
            guest: 0x81,
            drop_identifies: 3,
        };
        run(&mut s, &mut m, 0, 10_000);
        assert!(s.ready());
        assert_eq!(s.nodes()[0].class, None, "three losses leave it unread");
        assert!(s.class_unread());
    }

    #[test]
    fn a_walk_with_no_answer_retries_after_the_rewalk_period() {
        let mut s = Session::new();
        let mut sends = Sends::new();
        s.poll(0, false, false, &mut sends);
        assert_eq!(sends.len(), 1, "NODE_HELLO");
        for t in [1000, 2000] {
            let mut sends = Sends::new();
            s.poll(t, false, false, &mut sends);
            assert_eq!(sends.len(), 1, "resent at {t}");
        }
        let mut sends = Sends::new();
        s.poll(3000, false, false, &mut sends);
        assert!(sends.is_empty());
        assert_eq!(s.phase(), Phase::Retry { at: 6000 });
        let mut sends = Sends::new();
        s.poll(6000, true, false, &mut sends);
        assert!(sends.is_empty(), "never while armed");
        s.poll(6010, false, false, &mut sends);
        assert_eq!(sends.len(), 1);
        assert_eq!(s.phase(), Phase::Walking);
    }

    #[test]
    fn a_machine_with_no_motor_controller_is_walked_again_every_three_seconds() {
        // A bridge alone: it reports no neighbour but the guest.
        struct Alone;
        impl Alone {
            fn answer(req: &[u8]) -> Option<StdVec<u8>> {
                let p = Pdu::decode(req).unwrap();
                match p.known()? {
                    Opcode::NodeHello => {
                        Some(pdu(Opcode::NodeHello, 0x01, p.src, &[0x01, 1, 0x81]))
                    }
                    Opcode::ProbePorts => {
                        Some(pdu(Opcode::Ports, 0x01, p.src, &[1, 1, 1, 2, 0x81]))
                    }
                    Opcode::Identify => {
                        Some(pdu(Opcode::Identity, 0x01, p.src, &identity(CLASS_BRIDGE)))
                    }
                    _ => None,
                }
            }
        }
        let mut s = Session::new();
        let mut now = 0;
        let mut hellos = StdVec::new();
        let mut inbox: StdVec<StdVec<u8>> = StdVec::new();
        while now < 7000 {
            let mut sends = Sends::new();
            for f in inbox.drain(..) {
                s.on_pdu(&f, now, &mut sends);
            }
            s.poll(now, false, false, &mut sends);
            for req in sends {
                if Pdu::decode(&req).unwrap().known() == Some(Opcode::NodeHello) {
                    hellos.push(now);
                }
                if let Some(r) = Alone::answer(&req) {
                    inbox.push(r);
                }
            }
            now += 10;
        }
        assert_eq!(hellos.len(), 3, "{hellos:?}");
        assert!(hellos[1] - hellos[0] >= REWALK_MS);
    }

    #[test]
    fn telemetry_is_not_taken_as_a_reply() {
        let mut s = Session::new();
        let mut sends = Sends::new();
        s.poll(0, false, false, &mut sends);
        let mut b = [0u8; 64];
        let n = Pdu::new(linkctl::OP_CYCLIC_STATE, 0x02, 0x00, &[0; 32])
            .unwrap()
            .encode(&mut b)
            .unwrap();
        assert!(!s.on_pdu(&b[..n], 5, &mut sends));
        assert_eq!(s.phase(), Phase::Walking);
    }
}
