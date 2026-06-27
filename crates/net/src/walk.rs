//! The controller-driven discovery + address-assignment walk (`specs/l3.md`, "Discovery and address
//! assignment").
//!
//! Two halves:
//! - [`Responder`] - the **board side**: a passive responder that answers `NODE_HELLO` with its
//!   identity, probes its own ports on `PROBE_PORTS` and reports `PORTS`, and takes an `ASSIGN`
//!   (persisting `node_address` to the `store`) or relays a directed `ASSIGN` out an egress port with
//!   `dst` rewritten to `0x00`. It never self-assigns and never reads a hardware id - identity is
//!   positional during discovery, the persisted address after.
//! - [`Controller`] - the **host side** (transient controller): holds the address map and drives the
//!   whole process, working outward to the leaves (first contact -> assign -> probe -> assign the
//!   frontier -> recurse).
//!
//! Both are HAL-free and link-agnostic: they consume and produce whole (L2-reassembled) PDUs by port
//! index. The firmware (and the Tier-1 harness) does the L2 `send`/`poll_recv` around them. The board
//! side wraps a [`Forwarder`] so source-learning + multi-hop forwarding happen on the same path.

use heapless::Vec;
use store::{DynError, Flash, Key, Store, Type, Value, NODE_ADDRESS};

use crate::forward::Forwarder;
use crate::pdu::{self, Opcode, Pdu};

/// The most local ports any board in the fleet has (a 12-FET gateway: BLE + two sideboard UARTs, plus
/// headroom). Sizes the per-port arrays and the probe state.
pub const MAX_PORTS: usize = 4;
/// The largest walk/CONFIG PDU. A `PORTS` reply for [`MAX_PORTS`] ports is `3 + 1 + 4*4 = 20` bytes;
/// `CONFIG_*` values are small. 64 leaves headroom and stays one L2 fragment on the mock link.
pub const MAX_PDU: usize = 64;
/// The most emissions one ingest can produce (a flood / probe over every port, plus a reply).
pub const MAX_EMIT: usize = MAX_PORTS + 2;

/// A bounded encoded-PDU buffer.
pub type PduBuf = Vec<u8, MAX_PDU>;

/// One thing to put on the wire: the egress port and the encoded PDU.
#[derive(Clone)]
pub struct Emission {
    /// The local port to send it out.
    pub port: u8,
    /// The encoded PDU bytes.
    pub bytes: PduBuf,
}

/// The bounded set of emissions one step produces.
pub type Emits = Vec<Emission, MAX_EMIT>;

// ---- wire constants (`specs/l3.md`, the opcode table) ----

/// `NODE_HELLO` request `kind`: a transient controller making first contact (grant it a guest address).
pub const KIND_CONTROLLER: u8 = 0x01;
/// `NODE_HELLO` request `kind`: a board probing a neighbour on the controller's behalf (no grant).
pub const KIND_PROBE: u8 = 0x02;

/// `PORTS` neighbour state: nothing wired to this port.
pub const NB_EMPTY: u8 = 0;
/// `PORTS` neighbour state: a board with no address yet (`node_id == 0x00`).
pub const NB_UNASSIGNED: u8 = 1;
/// `PORTS` neighbour state: a board (or guest) that already has an address (`neighbour_addr`).
pub const NB_ASSIGNED: u8 = 2;

/// `PORTS` port medium tag (the minimal default; `specs/l3.md` open question): a UART.
pub const PORT_UART: u8 = 0;
/// `PORTS` port medium tag: the BLE link.
pub const PORT_BLE: u8 = 1;
/// `PORTS` port medium tag: an SWD mailbox.
pub const PORT_SWD: u8 = 2;

/// `ASSIGN` `egress_port` meaning "the addressed board itself" (assign directly, not via a relay).
pub const EGRESS_SELF: u8 = 0xFF;

/// `ASSIGN_ACK` / `CONFIG_RESP` status: success.
pub const STATUS_OK: u8 = 0;
/// `ASSIGN_ACK` / `CONFIG_RESP` status: failure (e.g. a flash error).
pub const STATUS_ERR: u8 = 1;

/// The L3 protocol version a `NODE_HELLO` reply reports.
pub const PROTO_VER: u8 = 1;

/// `CONFIG_RESP` status: success.
pub const CFG_OK: u8 = 0;
/// `CONFIG_RESP` status: a malformed request (unknown type tag, bad value bytes, truncated PDU).
pub const CFG_BAD: u8 = 1;
/// `CONFIG_RESP` status: no field declares this `field_id`.
pub const CFG_UNKNOWN_KEY: u8 = 2;
/// `CONFIG_RESP` status: the value's type did not match the field's registered type.
pub const CFG_TYPE_MISMATCH: u8 = 3;
/// `CONFIG_RESP` status: the store write failed (flash full / error).
pub const CFG_STORE_ERR: u8 = 4;

/// Map a store [`DynError`] to a `CONFIG_RESP` status byte.
fn cfg_status(e: DynError) -> u8 {
    match e {
        DynError::UnknownKey => CFG_UNKNOWN_KEY,
        DynError::TypeMismatch => CFG_TYPE_MISMATCH,
        DynError::Store(_) => CFG_STORE_ERR,
    }
}

/// Append `[type_tag, value-bytes...]` of `v` to a `CONFIG_RESP` payload.
fn push_value(out: &mut PduBuf, v: &Value) {
    let _ = out.push(v.kind().tag());
    let mut tmp = [0u8; MAX_PDU];
    let n = v.encode(&mut tmp);
    let _ = out.extend_from_slice(&tmp[..n]);
}

/// Encode `pdu` and push it as an emission out `port` (best-effort; an over-long PDU is dropped).
fn emit(out: &mut Emits, port: u8, pdu: &Pdu) {
    let mut tmp = [0u8; MAX_PDU];
    if let Ok(n) = pdu.encode(&mut tmp) {
        let mut b: PduBuf = Vec::new();
        if b.extend_from_slice(&tmp[..n]).is_ok() {
            let _ = out.push(Emission { port, bytes: b });
        }
    }
}

/// One board's in-flight `PROBE_PORTS`: who to reply `PORTS` to, and the per-port neighbour states
/// gathered from the probe replies (initialised to `empty`; a reply upgrades a slot).
struct Probe {
    reply_to: u8,
    n_ports: u8,
    states: [(u8, u8); MAX_PORTS], // (neighbour_state, neighbour_addr)
}

/// The board-side walk responder. Wraps a [`Forwarder`] (so learning + forwarding run on the same
/// path) and adds the walk opcode handling. `no_std`, persists `node_address` through the `store`.
pub struct Responder {
    fwd: Forwarder,
    port_kinds: [u8; MAX_PORTS],
    fw_ver: u16,
    mcu: u8,
    guest_next: u8,
    probe: Option<Probe>,
}

impl Responder {
    /// A fresh, unconfigured board (`addr = 0x00`) with `n_ports` ports of the given medium kinds.
    pub fn new(n_ports: u8, port_kinds: [u8; MAX_PORTS], mcu: u8, fw_ver: u16) -> Self {
        // The per-port arrays (`port_kinds`, the probe `states`) are sized to MAX_PORTS; a board
        // declaring more would index out of bounds in `poll_probe`. The fleet max is 4, so this never
        // fires in practice - it guards a future part with more ports.
        debug_assert!(n_ports as usize <= MAX_PORTS, "n_ports exceeds MAX_PORTS");
        Responder {
            fwd: Forwarder::new(pdu::NO_ADDRESS, n_ports),
            port_kinds,
            fw_ver,
            mcu,
            guest_next: 0x80, // grant guests from the controller range
            probe: None,
        }
    }

    /// This board's current L3 address (`0x00` until assigned).
    pub fn addr(&self) -> u8 {
        self.fwd.addr()
    }

    /// The forwarder (for routing-table inspection in tests).
    pub fn forwarder(&self) -> &Forwarder {
        &self.fwd
    }

    /// Restore the persisted `node_address` from flash into the live address (a reboot: the board
    /// re-reads its address and resumes; `specs/l3.md`, "Recovery after a reboot").
    pub fn restore_addr<F: Flash>(&mut self, store: &Store<F>) {
        if let Ok(Value::U8(a)) = store.get_value(NODE_ADDRESS.key()) {
            self.fwd.set_addr(a);
        }
    }

    /// Is a `PROBE_PORTS` in flight (awaiting its probe replies / a tick to report)?
    pub fn probing(&self) -> bool {
        self.probe.is_some()
    }

    /// A reboot (`specs/l3.md`, "Recovery after a reboot"): the routing table is soft state, so it is
    /// **cleared** and re-learned from traffic; the `node_address` is **re-read from flash** (it
    /// persists). The board then resumes its cyclic peer traffic, which re-populates every table.
    pub fn reboot<F: Flash>(&mut self, store: &Store<F>) {
        self.fwd.clear_routes();
        self.restore_addr(store);
    }

    /// Originate a frame from this board (its resumed cyclic peer traffic, or a board-to-board control
    /// frame): `src =` this board's address, routed by the learned table (flood if `dst` is unknown).
    /// `opcode` is any valid opcode (an L7 `DRIVE` is forwarded best-effort, never interpreted here).
    pub fn originate(&self, opcode: u8, dst: u8, payload: &[u8], out: &mut Emits) {
        if let Ok(pdu) = Pdu::new(opcode, self.addr(), dst, payload) {
            self.fwd.originate(&pdu, &mut |port, f| emit(out, port, f));
        }
    }

    /// Ingest one received frame on `ingress`: learn + forward (multi-hop), and if it is for this node,
    /// handle the walk opcode. Emissions (forwarded frames + walk replies/probes) are appended to `out`.
    pub fn ingest<F: Flash>(
        &mut self,
        ingress: u8,
        frame: &[u8],
        store: &mut Store<F>,
        out: &mut Emits,
    ) {
        let Ok(pdu) = Pdu::decode(frame) else {
            return;
        };
        // Run the forwarder: collect any forwarded copies and the delivered-to-self PDU separately, so
        // the walk handling does not nest inside the forwarder's borrow.
        let mut delivered: Option<PduBuf> = None;
        let mut forwarded: Emits = Vec::new();
        self.fwd.ingest(
            ingress,
            &pdu,
            &mut |d| {
                let mut b: PduBuf = Vec::new();
                let mut tmp = [0u8; MAX_PDU];
                if let Ok(n) = d.encode(&mut tmp) {
                    let _ = b.extend_from_slice(&tmp[..n]);
                    delivered = Some(b);
                }
            },
            &mut |port, f| emit(&mut forwarded, port, f),
        );
        for e in forwarded {
            let _ = out.push(e);
        }
        if let Some(buf) = delivered {
            if let Ok(p) = Pdu::decode(&buf) {
                self.handle_local(ingress, &p, store, out);
            }
        }
    }

    /// Complete an in-flight probe (the firmware poll's "probe window elapsed" tick): emit the `PORTS`
    /// reply with the gathered per-port states, routed toward the controller.
    pub fn poll_probe(&mut self, out: &mut Emits) {
        let Some(probe) = self.probe.take() else {
            return;
        };
        let mut payload: PduBuf = Vec::new();
        let _ = payload.push(probe.n_ports);
        for p in 0..probe.n_ports {
            let (state, addr) = probe.states[p as usize];
            let _ = payload.push(p);
            let _ = payload.push(self.port_kinds[p as usize]);
            let _ = payload.push(state);
            let _ = payload.push(addr);
        }
        let r = Pdu::from_op(Opcode::Ports, self.addr(), probe.reply_to, &payload);
        self.fwd.originate(&r, &mut |port, f| emit(out, port, f));
    }

    fn handle_local<F: Flash>(
        &mut self,
        ingress: u8,
        pdu: &Pdu,
        store: &mut Store<F>,
        out: &mut Emits,
    ) {
        match Opcode::from_u8(pdu.opcode) {
            Some(Opcode::NodeHello) => self.on_hello(ingress, pdu, out),
            Some(Opcode::ProbePorts) => self.on_probe_ports(pdu, out),
            Some(Opcode::Assign) => self.on_assign(pdu, store, out),
            Some(Opcode::ConfigRead) => self.on_config_read(pdu, store, out),
            Some(Opcode::ConfigWrite) => self.on_config_write(pdu, store, out),
            Some(Opcode::ConfigWriteMulti) => self.on_config_write_multi(pdu, store, out),
            // PORTS / ASSIGN_ACK / CONFIG_RESP are controller-bound; a stray one to a board is ignored.
            _ => {}
        }
    }

    /// `CONFIG_READ [field_id, index]` -> `CONFIG_RESP [field_id, index, status, type_tag, value...]`,
    /// routed back toward the requester (the dynamic `store.get_value` path; `specs/l3.md`).
    fn on_config_read<F: Flash>(&mut self, pdu: &Pdu, store: &Store<F>, out: &mut Emits) {
        if pdu.payload.len() < 2 {
            return;
        }
        let key = Key {
            field_id: pdu.payload[0],
            index: pdu.payload[1],
        };
        let mut resp: PduBuf = Vec::new();
        let _ = resp.push(key.field_id);
        let _ = resp.push(key.index);
        match store.get_value(key) {
            Ok(v) => {
                let _ = resp.push(CFG_OK);
                push_value(&mut resp, &v);
            }
            Err(e) => {
                let _ = resp.push(cfg_status(e));
                let _ = resp.push(0); // no type tag on an error
            }
        }
        self.reply_config(pdu.src, &resp, out);
    }

    /// `CONFIG_WRITE [field_id, index, type_tag, value...]` -> persist (`store.set_value`) and
    /// `CONFIG_RESP [field_id, index, status, type_tag, value...]` echoing the stored value.
    fn on_config_write<F: Flash>(&mut self, pdu: &Pdu, store: &mut Store<F>, out: &mut Emits) {
        if pdu.payload.len() < 3 {
            return;
        }
        let key = Key {
            field_id: pdu.payload[0],
            index: pdu.payload[1],
        };
        let status = self.apply_write(store, key, pdu.payload[2], &pdu.payload[3..]);

        let mut resp: PduBuf = Vec::new();
        let _ = resp.push(key.field_id);
        let _ = resp.push(key.index);
        let _ = resp.push(status);
        if status == CFG_OK {
            if let Ok(v) = store.get_value(key) {
                push_value(&mut resp, &v);
            }
        } else {
            let _ = resp.push(0);
        }
        self.reply_config(pdu.src, &resp, out);
    }

    /// `CONFIG_WRITE_MULTI [count, (field_id, index, type_tag, vlen, value[vlen])*]`: apply each entry
    /// (a whole board definition in one PDU), then a single `CONFIG_RESP [0, 0, status, 0]` ack.
    fn on_config_write_multi<F: Flash>(
        &mut self,
        pdu: &Pdu,
        store: &mut Store<F>,
        out: &mut Emits,
    ) {
        let p = pdu.payload;
        if p.is_empty() {
            return;
        }
        let count = p[0] as usize;
        let mut off = 1;
        let mut status = CFG_OK;
        for _ in 0..count {
            if off + 4 > p.len() {
                status = CFG_BAD;
                break;
            }
            let field_id = p[off];
            let index = p[off + 1];
            let type_tag = p[off + 2];
            let vlen = p[off + 3] as usize;
            off += 4;
            if off + vlen > p.len() {
                status = CFG_BAD;
                break;
            }
            let key = Key { field_id, index };
            let s = self.apply_write(store, key, type_tag, &p[off..off + vlen]);
            off += vlen;
            if s != CFG_OK {
                status = s; // first failure wins; keep applying the rest is not required
                break;
            }
        }
        let resp = [0u8, 0, status, 0];
        self.reply_config(pdu.src, &resp, out);
    }

    /// Decode + validate a value of `type_tag` and persist it to `key`, returning the `CONFIG_RESP`
    /// status (`OK` / `UNKNOWN_KEY` / `TYPE_MISMATCH` / `BAD`).
    fn apply_write<F: Flash>(
        &mut self,
        store: &mut Store<F>,
        key: Key,
        type_tag: u8,
        bytes: &[u8],
    ) -> u8 {
        let Some(kind) = Type::from_tag(type_tag) else {
            return CFG_BAD;
        };
        let Some(value) = Value::decode(kind, bytes) else {
            return CFG_BAD;
        };
        match store.set_value(key, value) {
            Ok(()) => CFG_OK,
            Err(e) => cfg_status(e),
        }
    }

    /// Emit a `CONFIG_RESP` toward `dst` (the requester), routed by the learned table.
    fn reply_config(&self, dst: u8, payload: &[u8], out: &mut Emits) {
        let r = Pdu::from_op(Opcode::ConfigResp, self.addr(), dst, payload);
        self.fwd.originate(&r, &mut |port, f| emit(out, port, f));
    }

    fn on_hello(&mut self, ingress: u8, pdu: &Pdu, out: &mut Emits) {
        if pdu.payload.len() == 1 {
            // A request ("identify yourself"): reply identity out the ingress port (toward the asker).
            let kind = pdu.payload[0];
            let your_addr = if kind == KIND_CONTROLLER {
                let g = self.guest_next;
                self.guest_next = self.guest_next.wrapping_add(1);
                g
            } else {
                pdu::NO_ADDRESS
            };
            let node_id = self.addr();
            let fw = self.fw_ver.to_le_bytes();
            let reply = [node_id, PROTO_VER, fw[0], fw[1], self.mcu, your_addr];
            let r = Pdu::from_op(Opcode::NodeHello, node_id, pdu.src, &reply);
            emit(out, ingress, &r);
        } else if let Some(probe) = self.probe.as_mut() {
            // A reply to my own probe: classify the neighbour on `ingress` by its node_id.
            if (ingress as usize) < MAX_PORTS {
                let node_id = pdu.payload[0];
                let state = if node_id == pdu::NO_ADDRESS {
                    NB_UNASSIGNED
                } else {
                    NB_ASSIGNED
                };
                probe.states[ingress as usize] = (state, node_id);
            }
        }
    }

    fn on_probe_ports(&mut self, pdu: &Pdu, out: &mut Emits) {
        let n = self.fwd.n_ports();
        self.probe = Some(Probe {
            reply_to: pdu.src,
            n_ports: n,
            states: [(NB_EMPTY, 0u8); MAX_PORTS],
        });
        // Probe every port: a directed NODE_HELLO(kind=PROBE) to "the one peer" on each link.
        let req = [KIND_PROBE];
        for p in 0..n {
            let r = Pdu::from_op(Opcode::NodeHello, self.addr(), pdu::NO_ADDRESS, &req);
            emit(out, p, &r);
        }
    }

    fn on_assign<F: Flash>(&mut self, pdu: &Pdu, store: &mut Store<F>, out: &mut Emits) {
        if pdu.payload.len() < 2 {
            return;
        }
        let egress = pdu.payload[0];
        let new_addr = pdu.payload[1];
        if egress == EGRESS_SELF {
            // For this board: persist node_address, adopt it, and ACK back toward the controller.
            // Re-assigning the same address just re-persists and re-ACKs (idempotent).
            let status = match store.set_value(NODE_ADDRESS.key(), Value::U8(new_addr)) {
                Ok(()) => {
                    self.fwd.set_addr(new_addr);
                    STATUS_OK
                }
                Err(_) => STATUS_ERR,
            };
            let ack = [new_addr, status];
            let r = Pdu::from_op(Opcode::AssignAck, new_addr, pdu.src, &ack);
            self.fwd.originate(&r, &mut |port, f| emit(out, port, f));
        } else {
            // I am the relay: forward the ASSIGN out `egress` to the one peer, dst rewritten to 0x00 and
            // egress rewritten to SELF (the neighbour takes it as its own), src kept = the controller so
            // the neighbour's ACK routes back.
            let fwd_payload = [EGRESS_SELF, new_addr];
            let r = Pdu::from_op(Opcode::Assign, pdu.src, pdu::NO_ADDRESS, &fwd_payload);
            emit(out, egress, &r);
        }
    }
}

// ---------------------------------------------------------------------------------------------------
// The controller (host side): drives the walk, holds the address map.
// ---------------------------------------------------------------------------------------------------

/// The most nodes a Tier-1 walk addresses (the two worked topologies are <= 3 boards; headroom).
pub const MAX_NODES: usize = 8;
/// The pending-task queue bound.
pub const MAX_TASKS: usize = 16;

/// One step of the walk the controller still owes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Task {
    /// First contact on the attach link.
    Hello,
    /// Assign the board on the attach link directly (the gateway): `ASSIGN(egress=SELF)`.
    AssignGateway { new_addr: u8 },
    /// `PROBE_PORTS(addr)`.
    Probe { addr: u8 },
    /// Directed `ASSIGN(egress, new_addr)` to `relay` for its unassigned/colliding neighbour.
    AssignNeighbor { relay: u8, egress: u8, new_addr: u8 },
}

/// The transient controller. Sequential request/response (the acknowledged delivery class): it holds
/// one outstanding request, advances on its reply, and works the queue to quiescence. All requests
/// leave on the single attach port; the gateway forwards them onward by `dst`.
pub struct Controller {
    guest_addr: u8,
    next_board: u8,
    // (addr, relay, egress) for each board addressed this walk - its positional identity.
    assigned: Vec<(u8, u8, u8), MAX_NODES>,
    queue: Vec<Task, MAX_TASKS>,
    outstanding: Option<Task>,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    /// A controller about to make first contact. Its provisional guest address is `0x80` (adopted from
    /// the gateway's grant on the first reply).
    pub fn new() -> Self {
        let mut queue: Vec<Task, MAX_TASKS> = Vec::new();
        let _ = queue.push(Task::Hello);
        Controller {
            guest_addr: 0x80,
            next_board: 0x01,
            assigned: Vec::new(),
            queue,
            outstanding: None,
        }
    }

    /// The controller's (guest) address.
    pub fn guest_addr(&self) -> u8 {
        self.guest_addr
    }

    /// The board addresses handed out this walk.
    pub fn assigned_addrs(&self) -> Vec<u8, MAX_NODES> {
        self.assigned.iter().map(|&(a, _, _)| a).collect()
    }

    /// The walk is finished: nothing queued and nothing outstanding.
    pub fn is_complete(&self) -> bool {
        self.queue.is_empty() && self.outstanding.is_none()
    }

    /// The next request to send out the attach port, or `None` while a reply is still outstanding or
    /// the walk is complete.
    pub fn next_request(&mut self) -> Option<PduBuf> {
        if self.outstanding.is_some() {
            return None;
        }
        // Take from the front (BFS-ish: assign before descending keeps the walk outward).
        if self.queue.is_empty() {
            return None;
        }
        let task = self.queue.remove(0);
        let buf = self.build_request(&task);
        self.outstanding = Some(task);
        Some(buf)
    }

    /// Reply to a `NODE_HELLO` probe of the controller's own port (a board probing it on the walk's
    /// behalf): the controller is a node too, so it answers with its guest identity. Returns the reply
    /// PDU to send back out the attach port, or `None` if `frame` is not such a probe.
    pub fn reply_to_probe(&self, frame: &[u8]) -> Option<PduBuf> {
        let pdu = Pdu::decode(frame).ok()?;
        if pdu.known() == Some(Opcode::NodeHello) && pdu.payload.len() == 1 {
            // node_id = my guest addr (a controller range address): the prober records "assigned(guest)".
            let reply = [self.guest_addr, PROTO_VER, 0, 0, 0, pdu::NO_ADDRESS];
            let r = Pdu::from_op(Opcode::NodeHello, self.guest_addr, pdu.src, &reply);
            let mut tmp = [0u8; MAX_PDU];
            let n = r.encode(&mut tmp).ok()?;
            let mut b: PduBuf = Vec::new();
            b.extend_from_slice(&tmp[..n]).ok()?;
            return Some(b);
        }
        None
    }

    /// Feed a reply to the outstanding request, advancing the walk.
    pub fn on_reply(&mut self, frame: &[u8]) {
        let Ok(pdu) = Pdu::decode(frame) else {
            return;
        };
        let Some(task) = self.outstanding else {
            return;
        };
        let op = pdu.known();
        match (task, op) {
            (Task::Hello, Some(Opcode::NodeHello)) if pdu.payload.len() >= 6 => {
                self.outstanding = None;
                let node_id = pdu.payload[0];
                let your_addr = pdu.payload[5];
                self.guest_addr = your_addr; // adopt the granted guest address
                if node_id == pdu::NO_ADDRESS {
                    let na = self.alloc();
                    self.enqueue(Task::AssignGateway { new_addr: na });
                } else {
                    self.record(node_id, 0, EGRESS_SELF);
                    self.enqueue(Task::Probe { addr: node_id });
                }
            }
            (Task::AssignGateway { new_addr }, Some(Opcode::AssignAck))
                if pdu.payload.len() >= 2 =>
            {
                self.outstanding = None;
                let acked = pdu.payload[0];
                debug_assert_eq!(acked, new_addr);
                self.record(acked, 0, EGRESS_SELF);
                self.enqueue(Task::Probe { addr: acked });
            }
            (
                Task::AssignNeighbor {
                    relay,
                    egress,
                    new_addr,
                },
                Some(Opcode::AssignAck),
            ) if pdu.payload.len() >= 2 => {
                self.outstanding = None;
                let acked = pdu.payload[0];
                debug_assert_eq!(acked, new_addr);
                self.record(acked, relay, egress);
                self.enqueue(Task::Probe { addr: acked });
            }
            (Task::Probe { addr }, Some(Opcode::Ports)) => {
                self.outstanding = None;
                self.on_ports(addr, &pdu);
            }
            _ => {
                // An unexpected / wrong reply: keep the task outstanding for a retransmit (slice 6).
            }
        }
    }

    fn on_ports(&mut self, probed: u8, pdu: &Pdu) {
        if pdu.payload.is_empty() {
            return;
        }
        // The board we probed was reached through its parent (the relay we assigned it via). Its
        // upstream port reports that parent as an `assigned` neighbour - which is the link we came in
        // on, not a board to act on.
        let parent = self.parent_of(probed);
        let n = pdu.payload[0] as usize;
        for i in 0..n {
            let base = 1 + i * 4;
            if base + 3 >= pdu.payload.len() {
                break;
            }
            let port = pdu.payload[base];
            let state = pdu.payload[base + 2];
            let naddr = pdu.payload[base + 3];
            match state {
                NB_UNASSIGNED => {
                    let na = self.alloc();
                    self.enqueue(Task::AssignNeighbor {
                        relay: probed,
                        egress: port,
                        new_addr: na,
                    });
                }
                NB_ASSIGNED => {
                    if pdu::is_controller(naddr) {
                        // The controller/guest itself, reached back through a probe: ignore.
                    } else if Some(naddr) == parent {
                        // The upstream link back to the parent we descended from: ignore.
                    } else if let Some(pos) = self.position_of(naddr) {
                        if pos != (probed, port) {
                            // Same address at a different position: a collision. Reassign this one.
                            let na = self.alloc();
                            self.enqueue(Task::AssignNeighbor {
                                relay: probed,
                                egress: port,
                                new_addr: na,
                            });
                        }
                        // else: already known at this position - nothing to do (a re-walk).
                    } else {
                        // A board carrying a stale address from a past session: adopt it as-is and
                        // descend (a valid, persisted board; reassigned only if it later collides).
                        self.record(naddr, probed, port);
                        self.enqueue(Task::Probe { addr: naddr });
                    }
                }
                _ => {} // empty
            }
        }
    }

    /// The address of the relay through which `addr` was reached (its parent in the walk tree), or
    /// `None` for the gateway (reached on the controller's own link).
    fn parent_of(&self, addr: u8) -> Option<u8> {
        self.assigned
            .iter()
            .find(|&&(a, _, _)| a == addr)
            .map(|&(_, relay, _)| relay)
            .filter(|&r| pdu::is_unicast(r))
    }

    fn build_request(&self, task: &Task) -> PduBuf {
        let (op, dst, payload): (Opcode, u8, Vec<u8, 4>) = match *task {
            Task::Hello => (Opcode::NodeHello, pdu::NO_ADDRESS, one(KIND_CONTROLLER)),
            Task::AssignGateway { new_addr } => {
                (Opcode::Assign, pdu::NO_ADDRESS, two(EGRESS_SELF, new_addr))
            }
            Task::Probe { addr } => (Opcode::ProbePorts, addr, Vec::new()),
            Task::AssignNeighbor {
                relay,
                egress,
                new_addr,
            } => (Opcode::Assign, relay, two(egress, new_addr)),
        };
        let r = Pdu::from_op(op, self.guest_addr, dst, &payload);
        let mut tmp = [0u8; MAX_PDU];
        let n = r.encode(&mut tmp).unwrap_or(0);
        let mut b: PduBuf = Vec::new();
        let _ = b.extend_from_slice(&tmp[..n]);
        b
    }

    /// Allocate the next free board address (`0x01..=0x7F`), skipping any already handed out.
    fn alloc(&mut self) -> u8 {
        loop {
            let a = self.next_board;
            self.next_board = self.next_board.wrapping_add(1);
            if pdu::is_board(a) && self.position_of(a).is_none() {
                return a;
            }
        }
    }

    fn record(&mut self, addr: u8, relay: u8, egress: u8) {
        if self.position_of(addr).is_none() {
            let _ = self.assigned.push((addr, relay, egress));
        }
    }

    fn position_of(&self, addr: u8) -> Option<(u8, u8)> {
        self.assigned
            .iter()
            .find(|&&(a, _, _)| a == addr)
            .map(|&(_, r, e)| (r, e))
    }

    fn enqueue(&mut self, task: Task) {
        let _ = self.queue.push(task);
    }
}

/// A one-byte payload.
fn one(b: u8) -> Vec<u8, 4> {
    let mut v = Vec::new();
    let _ = v.push(b);
    v
}

/// A two-byte payload.
fn two(a: u8, b: u8) -> Vec<u8, 4> {
    let mut v = Vec::new();
    let _ = v.push(a);
    let _ = v.push(b);
    v
}

#[cfg(test)]
mod tests;
