//! Tier-1 host tests for the discovery walk (`specs/l3.md`, Test plan Tier 1 item 3): the controller
//! driver + the board responder over an in-memory mesh of **real L2 links** (`link::Link` over a mock
//! datagram `Transport`), persisting `node_address` to a real `store` over an in-test mock flash.
//!
//! It recurses the two worked topologies to the leaves and asserts every board is addressed and
//! persisted, a re-walk reports the assigned addresses (no re-assign), a collision is reassigned, a
//! dropped `ASSIGN_ACK` recovers via an idempotent re-`ASSIGN`, and **no hardware id is read anywhere**
//! (two identical boards still provision by position).

use super::*;
use base::error::FlashError;
use link::{Link, Transport};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec as StdVec;

use crate::pdu::NO_ADDRESS;
use store::{
    Flash, Key, Store, Type, Value, DEVICE_NAME, MOTOR_CURRENT_LIMIT, MOTOR_METHOD, NODE_ADDRESS,
};

const PS: usize = 1024;
const CAP: usize = 255;

/// A minimal in-RAM [`Flash`] for the walk tests: a two-page region, erased to `0xFFFF`, with
/// halfword-aligned write-once `program` (the silicon rules the store relies on). Independent of
/// store's `#[cfg(test)]` `MockFlash` (which is internal to that crate).
struct TestFlash {
    page_size: usize,
    bytes: StdVec<u8>,
}

impl TestFlash {
    fn erased(page_size: usize) -> Self {
        TestFlash {
            page_size,
            bytes: std::vec![0xFFu8; 2 * page_size],
        }
    }
}

impl Flash for TestFlash {
    fn page_size(&self) -> usize {
        self.page_size
    }
    fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }
    fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
        let start = page * self.page_size;
        let end = start + self.page_size;
        if end > self.bytes.len() {
            return Err(FlashError::OutOfBounds);
        }
        for b in &mut self.bytes[start..end] {
            *b = 0xFF;
        }
        Ok(())
    }
    fn program(&mut self, off: usize, bytes: &[u8]) -> Result<(), FlashError> {
        if !off.is_multiple_of(2) || !bytes.len().is_multiple_of(2) {
            return Err(FlashError::Misaligned);
        }
        if off + bytes.len() > self.bytes.len() {
            return Err(FlashError::OutOfBounds);
        }
        // Write-once at halfword granularity: every target halfword must still be erased.
        for (i, &b) in bytes.iter().enumerate() {
            if self.bytes[off + i] != 0xFF && b != self.bytes[off + i] {
                return Err(FlashError::ProgramFailed);
            }
        }
        self.bytes[off..off + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}

// ---- a mock point-to-point L2 datagram link ----

type Wire = Rc<RefCell<VecDeque<StdVec<u8>>>>;

struct MockPort {
    tx: Wire,
    rx: Wire,
}

impl Transport for MockPort {
    fn frame_capacity(&self) -> usize {
        CAP
    }
    fn send_l2_frame(&mut self, l2: &[u8]) {
        self.tx.borrow_mut().push_back(l2.to_vec());
    }
    fn recv_l2_frame(&mut self, out: &mut [u8]) -> Option<usize> {
        let f = self.rx.borrow_mut().pop_front()?;
        out[..f.len()].copy_from_slice(&f);
        Some(f.len())
    }
}

type Port = Link<MockPort>;

struct Board {
    resp: Responder,
    flash: TestFlash,
    ports: StdVec<Option<Port>>,
}

struct CtrlNode {
    ctrl: Controller,
    link: Option<Port>,
    /// CONFIG_RESP frames that arrived at the controller (the walk's on_reply ignores them).
    inbox: StdVec<StdVec<u8>>,
}

struct Mesh {
    boards: StdVec<Board>,
    ctrl: CtrlNode,
}

impl Mesh {
    fn new() -> Self {
        Mesh {
            boards: StdVec::new(),
            ctrl: CtrlNode {
                ctrl: Controller::new(),
                link: None,
                inbox: StdVec::new(),
            },
        }
    }

    /// Add a board with `n_ports` ports; returns its index.
    fn add_board(&mut self, n_ports: u8) -> usize {
        let kinds = [PORT_UART; MAX_PORTS];
        let mut ports = StdVec::new();
        for _ in 0..n_ports {
            ports.push(None);
        }
        self.boards.push(Board {
            resp: Responder::new(n_ports, kinds, /*mcu*/ 0x10, /*fw*/ 0x0001),
            flash: TestFlash::erased(PS),
            ports,
        });
        self.boards.len() - 1
    }

    /// Pre-persist a board's `node_address` (a stale address from a past session) and boot it there.
    fn preassign(&mut self, board: usize, addr: u8) {
        let b = &mut self.boards[board];
        let mut s = Store::mount(&mut b.flash).unwrap();
        s.set_value(NODE_ADDRESS.key(), Value::U8(addr)).unwrap();
        b.resp.restore_addr(&s);
    }

    /// Wire `(a, pa) <-> (b, pb)` as a point-to-point link.
    fn wire(&mut self, a: usize, pa: u8, b: usize, pb: u8) {
        let w1: Wire = Default::default();
        let w2: Wire = Default::default();
        self.boards[a].ports[pa as usize] = Some(Link::new(MockPort {
            tx: w1.clone(),
            rx: w2.clone(),
        }));
        self.boards[b].ports[pb as usize] = Some(Link::new(MockPort { tx: w2, rx: w1 }));
    }

    /// Attach the controller to `(gateway, port)`.
    fn attach_controller(&mut self, gateway: usize, port: u8) {
        let w1: Wire = Default::default();
        let w2: Wire = Default::default();
        self.ctrl.link = Some(Link::new(MockPort {
            tx: w1.clone(),
            rx: w2.clone(),
        }));
        self.boards[gateway].ports[port as usize] = Some(Link::new(MockPort { tx: w2, rx: w1 }));
    }

    /// Drive the walk to completion.
    fn run_walk(&mut self) {
        for _ in 0..1000 {
            if self.ctrl.ctrl.is_complete() {
                return;
            }
            if let Some(req) = self.ctrl.ctrl.next_request() {
                self.ctrl.link.as_mut().unwrap().send(&req).unwrap();
            }
            self.settle();
        }
        panic!("walk did not complete");
    }

    /// Run the firmware loop to quiescence: drain the controller and every board, route emissions, and
    /// fire probe ticks, until nothing moves.
    fn settle(&mut self) {
        let mut buf = [0u8; MAX_PDU];
        for _ in 0..10_000 {
            let mut moved = false;

            // Controller inbound: answer a probe-of-me inline; feed a reply to the walk.
            loop {
                let frame = self
                    .ctrl
                    .link
                    .as_mut()
                    .unwrap()
                    .poll_recv(&mut buf)
                    .map(|f| f.to_vec());
                let Some(frame) = frame else { break };
                moved = true;
                if let Some(reply) = self.ctrl.ctrl.reply_to_probe(&frame) {
                    self.ctrl.link.as_mut().unwrap().send(&reply).unwrap();
                } else if Pdu::decode(&frame).map(|p| p.known()) == Ok(Some(Opcode::ConfigResp)) {
                    self.ctrl.inbox.push(frame);
                } else {
                    self.ctrl.ctrl.on_reply(&frame);
                }
            }

            // Boards: ingest each ready frame, route emissions.
            for bi in 0..self.boards.len() {
                let nports = self.boards[bi].ports.len();
                for p in 0..nports {
                    loop {
                        let frame = match self.boards[bi].ports[p].as_mut() {
                            Some(link) => link.poll_recv(&mut buf).map(|f| f.to_vec()),
                            None => None,
                        };
                        let Some(frame) = frame else { break };
                        moved = true;
                        let mut emits = Emits::new();
                        {
                            let board = &mut self.boards[bi];
                            let mut store = Store::mount(&mut board.flash).unwrap();
                            board.resp.ingest(p as u8, &frame, &mut store, &mut emits);
                        }
                        self.send_emits(bi, &emits);
                    }
                }
            }

            if moved {
                continue;
            }

            // No L2 movement: fire any pending probe ticks (the firmware poll's "probe window elapsed").
            let mut ticked = false;
            for bi in 0..self.boards.len() {
                if self.boards[bi].resp.probing() {
                    let mut emits = Emits::new();
                    self.boards[bi].resp.poll_probe(&mut emits);
                    self.send_emits(bi, &emits);
                    ticked = true;
                }
            }
            if !ticked {
                return;
            }
        }
        panic!("settle did not quiesce");
    }

    fn send_emits(&mut self, bi: usize, emits: &Emits) {
        for e in emits {
            if let Some(Some(link)) = self.boards[bi].ports.get_mut(e.port as usize) {
                link.send(&e.bytes).unwrap();
            }
        }
    }

    /// Send a PDU (any opcode) from the controller (out its attach link) and settle, returning the
    /// captured CONFIG_RESP if one came back.
    fn controller_send(&mut self, opcode: u8, dst: u8, payload: &[u8]) -> Option<StdVec<u8>> {
        let src = self.ctrl.ctrl.guest_addr();
        let pdu = Pdu::new(opcode, src, dst, payload).unwrap();
        let mut buf = [0u8; MAX_PDU];
        let n = pdu.encode(&mut buf).unwrap();
        self.ctrl.link.as_mut().unwrap().send(&buf[..n]).unwrap();
        self.settle();
        self.ctrl.inbox.pop()
    }

    /// `CONFIG_WRITE(dst, key, value)` -> the CONFIG_RESP.
    fn config_write(&mut self, dst: u8, key: store::Key, value: Value) -> StdVec<u8> {
        let mut payload = StdVec::new();
        payload.push(key.field_id);
        payload.push(key.index);
        payload.push(value.kind().tag());
        let mut vb = [0u8; MAX_PDU];
        let vn = value.encode(&mut vb);
        payload.extend_from_slice(&vb[..vn]);
        self.controller_send(Opcode::ConfigWrite.to_u8(), dst, &payload)
            .expect("a CONFIG_RESP")
    }

    /// `CONFIG_READ(dst, key)` -> the CONFIG_RESP.
    fn config_read(&mut self, dst: u8, key: store::Key) -> StdVec<u8> {
        self.controller_send(Opcode::ConfigRead.to_u8(), dst, &[key.field_id, key.index])
            .expect("a CONFIG_RESP")
    }

    /// Reboot every board: clear its routing table (soft state) and re-read its persisted address.
    fn reboot_all(&mut self) {
        for b in &mut self.boards {
            let s = Store::mount(&mut b.flash).unwrap();
            b.resp.reboot(&s);
        }
    }

    /// A board originates a frame (resumed cyclic traffic / a board-to-board control frame) and the
    /// mesh settles.
    fn board_originate(&mut self, board: usize, opcode: u8, dst: u8, payload: &[u8]) {
        let mut emits = Emits::new();
        self.boards[board]
            .resp
            .originate(opcode, dst, payload, &mut emits);
        self.send_emits(board, &emits);
        self.settle();
    }

    fn port_toward(&self, board: usize, dst: u8) -> Option<u8> {
        self.boards[board].resp.forwarder().port_toward(dst)
    }

    fn live_addr(&self, board: usize) -> u8 {
        self.boards[board].resp.addr()
    }

    fn persisted_addr(&mut self, board: usize) -> u8 {
        let s = Store::mount(&mut self.boards[board].flash).unwrap();
        match s.get_value(NODE_ADDRESS.key()).unwrap() {
            Value::U8(a) => a,
            _ => panic!("node_address not a u8"),
        }
    }
}

// ---------------------------------------------------------------------------------------------------
// Topology (a): a 12-FET gateway + two attitude sideboards.
// ---------------------------------------------------------------------------------------------------

#[test]
fn topology_a_addresses_and_persists_every_board() {
    let mut m = Mesh::new();
    let gw = m.add_board(3); // attach(0) + two sideboard ports(1,2)
    let s1 = m.add_board(1);
    let s2 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.wire(gw, 2, s2, 0);

    m.run_walk();

    // The gateway is 0x01; the two sideboards 0x02 / 0x03 by position (distinct ports on 0x01).
    assert_eq!(m.live_addr(gw), 0x01);
    let a1 = m.live_addr(s1);
    let a2 = m.live_addr(s2);
    assert!(pdu::is_board(a1) && pdu::is_board(a2));
    assert_ne!(a1, a2);
    assert_eq!(
        {
            let mut v = [a1, a2];
            v.sort_unstable();
            v
        },
        [0x02, 0x03]
    );

    // Every address is PERSISTED to flash (survives a reboot), not just live.
    assert_eq!(m.persisted_addr(gw), 0x01);
    assert_eq!(m.persisted_addr(s1), a1);
    assert_eq!(m.persisted_addr(s2), a2);

    // The controller's map holds exactly the three boards.
    let mut handed = m.ctrl.ctrl.assigned_addrs().to_vec();
    handed.sort_unstable();
    assert_eq!(handed, [0x01, 0x02, 0x03]);
}

#[test]
fn topology_b_master_slave_pair() {
    let mut m = Mesh::new();
    let gw = m.add_board(2); // attach(0) + inter-board(1)
    let slave = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, slave, 0);

    m.run_walk();

    assert_eq!(m.live_addr(gw), 0x01);
    assert_eq!(m.live_addr(slave), 0x02);
    assert_eq!(m.persisted_addr(gw), 0x01);
    assert_eq!(m.persisted_addr(slave), 0x02);
}

#[test]
fn a_re_walk_reports_assigned_and_assigns_nothing_new() {
    // First walk addresses everyone; a fresh controller re-walks the SAME (already-addressed) boards.
    let mut m = Mesh::new();
    let gw = m.add_board(3);
    let s1 = m.add_board(1);
    let s2 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.wire(gw, 2, s2, 0);
    m.run_walk();
    let first: StdVec<u8> = {
        let mut v = m.ctrl.ctrl.assigned_addrs().to_vec();
        v.sort_unstable();
        v
    };

    // A second controller attaches and walks again. Boards keep their persisted addresses.
    m.ctrl.ctrl = Controller::new();
    m.run_walk();
    let second: StdVec<u8> = {
        let mut v = m.ctrl.ctrl.assigned_addrs().to_vec();
        v.sort_unstable();
        v
    };

    assert_eq!(first, [0x01, 0x02, 0x03]);
    assert_eq!(second, first); // same addresses reported, none newly minted
                               // Addresses unchanged on the boards.
    assert_eq!(m.persisted_addr(gw), 0x01);
    assert!(m.persisted_addr(s1) != m.persisted_addr(s2));
}

#[test]
fn two_identical_boards_provision_by_position_no_id_read() {
    // The two sideboards are byte-for-byte identical responders (same mcu, same fw, no device id).
    // They still get distinct addresses - distinguished only by which gateway port they sit on. Nothing
    // in the walk reads a hardware id (the Responder has no id field at all).
    let mut m = Mesh::new();
    let gw = m.add_board(3);
    let s1 = m.add_board(1);
    let s2 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.wire(gw, 2, s2, 0);
    m.run_walk();
    assert_ne!(m.live_addr(s1), m.live_addr(s2));
}

#[test]
fn an_assigned_collision_is_reassigned_and_re_persisted() {
    // The gateway is assigned 0x01; a sideboard boots with a STALE persisted 0x01 (a past session).
    // The walk detects the collision (0x01 at two positions) and reassigns the sideboard, re-persisting.
    let mut m = Mesh::new();
    let gw = m.add_board(2);
    let s1 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.preassign(s1, 0x01); // collides with the gateway's eventual 0x01

    m.run_walk();

    assert_eq!(m.live_addr(gw), 0x01);
    let a1 = m.live_addr(s1);
    assert_ne!(a1, 0x01); // reassigned off the collision
    assert!(pdu::is_board(a1));
    assert_eq!(m.persisted_addr(s1), a1); // and re-persisted
}

// ---------------------------------------------------------------------------------------------------
// Responder-level: idempotent re-ASSIGN (a dropped ASSIGN_ACK recovers).
// ---------------------------------------------------------------------------------------------------

#[test]
fn re_assigning_the_same_address_re_persists_and_re_acks_idempotently() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, 0x0001);

    // First ASSIGN(egress=SELF, 0x05) arrives directly (dst=0x00, the one peer). It persists + ACKs;
    // imagine the ACK is dropped on the wire.
    let assign = Pdu::from_op(Opcode::Assign, 0x80, NO_ADDRESS, &[EGRESS_SELF, 0x05]);
    let mut buf = [0u8; MAX_PDU];
    let n = assign.encode(&mut buf).unwrap();

    let ack1 = {
        let mut s = Store::mount(&mut flash).unwrap();
        let mut emits = Emits::new();
        resp.ingest(0, &buf[..n], &mut s, &mut emits);
        emits
    };
    assert_eq!(resp.addr(), 0x05);
    assert_eq!(ack_status(&ack1), Some((0x05, STATUS_OK)));

    // The controller retransmits the SAME ASSIGN. It re-persists and re-ACKs OK (idempotent).
    let ack2 = {
        let mut s = Store::mount(&mut flash).unwrap();
        let mut emits = Emits::new();
        resp.ingest(0, &buf[..n], &mut s, &mut emits);
        emits
    };
    assert_eq!(resp.addr(), 0x05);
    assert_eq!(ack_status(&ack2), Some((0x05, STATUS_OK)));

    // The persisted value is 0x05 either way.
    let s = Store::mount(&mut flash).unwrap();
    assert_eq!(s.get_value(NODE_ADDRESS.key()).unwrap(), Value::U8(0x05));
}

/// Pull `(new_addr, status)` out of an emitted ASSIGN_ACK, if present.
fn ack_status(emits: &Emits) -> Option<(u8, u8)> {
    for e in emits {
        if let Ok(p) = Pdu::decode(&e.bytes) {
            if p.known() == Some(Opcode::AssignAck) && p.payload.len() >= 2 {
                return Some((p.payload[0], p.payload[1]));
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------------------------------
// Slice 5: CONFIG_* over the store Key/Value (the dynamic set/get), incl. a two-hop relayed write.
// ---------------------------------------------------------------------------------------------------

// A captured CONFIG_RESP is a full PDU frame; its payload is `[field_id, index, status, type_tag,
// value...]`.
fn resp_status(frame: &[u8]) -> u8 {
    Pdu::decode(frame).unwrap().payload[2]
}
fn resp_value(frame: &[u8]) -> Option<Value<'_>> {
    let payload = Pdu::decode(frame).ok()?.payload;
    let kind = Type::from_tag(*payload.get(3)?)?;
    Value::decode(kind, &payload[4..])
}

fn walked_pair() -> Mesh {
    let mut m = Mesh::new();
    let gw = m.add_board(2);
    let slave = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, slave, 0);
    m.run_walk();
    m
}

#[test]
fn config_write_then_read_round_trips_on_the_addressed_board() {
    let mut m = walked_pair(); // gateway 0x01, slave 0x02
    let key = MOTOR_CURRENT_LIMIT.key();

    let w = m.config_write(0x01, key, Value::U32(15_000));
    assert_eq!(resp_status(&w), CFG_OK);

    let r = m.config_read(0x01, key);
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_value(&r), Some(Value::U32(15_000)));
}

#[test]
fn config_read_of_an_unwritten_field_returns_its_default() {
    let mut m = walked_pair();
    let r = m.config_read(0x01, DEVICE_NAME.key());
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_value(&r), Some(Value::Str("hoverboard")));
}

#[test]
fn a_two_hop_config_write_is_relayed_through_the_gateway() {
    // The headline forwarding+CONFIG case: a CONFIG_WRITE addressed to the slave (0x02) reaches it
    // through the gateway (source-learning populated the gateway's table during the walk) and the
    // CONFIG_RESP routes back. A read-back through the same two hops confirms it persisted on the slave.
    let mut m = walked_pair();
    let key = MOTOR_METHOD.key();

    let w = m.config_write(0x02, key, Value::U8(3));
    assert_eq!(resp_status(&w), CFG_OK);

    let r = m.config_read(0x02, key);
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_value(&r), Some(Value::U8(3)));

    // And it really is on the slave's flash (index 1 is the slave).
    let on_flash = {
        let s = Store::mount(&mut m.boards[1].flash).unwrap();
        match s.get_value(key).unwrap() {
            Value::U8(v) => v,
            other => panic!("unexpected {other:?}"),
        }
    };
    assert_eq!(on_flash, 3);
}

#[test]
fn config_type_mismatch_and_unknown_key_are_rejected() {
    let mut m = walked_pair();
    // Wrong type for a U32 field.
    let w = m.config_write(0x01, MOTOR_CURRENT_LIMIT.key(), Value::U16(5));
    assert_eq!(resp_status(&w), CFG_TYPE_MISMATCH);
    // An undeclared field_id.
    let bogus = Key {
        field_id: 0x99,
        index: 0,
    };
    let r = m.config_read(0x01, bogus);
    assert_eq!(resp_status(&r), CFG_UNKNOWN_KEY);
    let w2 = m.config_write(0x01, bogus, Value::U8(1));
    assert_eq!(resp_status(&w2), CFG_UNKNOWN_KEY);
}

#[test]
fn config_write_multi_writes_a_board_definition_in_one_pdu() {
    // A whole little board definition (three keys) in one CONFIG_WRITE_MULTI, then read each back.
    let mut m = walked_pair();
    let defs: [(Key, Value); 3] = [
        (MOTOR_CURRENT_LIMIT.key(), Value::U32(12_345)),
        (MOTOR_METHOD.key(), Value::U8(2)),
        (DEVICE_NAME.key(), Value::Str("rig-9")),
    ];

    // payload = [count, (field_id, index, type_tag, vlen, value)*]
    let mut payload = StdVec::new();
    payload.push(defs.len() as u8);
    for (k, v) in &defs {
        let mut vb = [0u8; MAX_PDU];
        let vn = v.encode(&mut vb);
        payload.push(k.field_id);
        payload.push(k.index);
        payload.push(v.kind().tag());
        payload.push(vn as u8);
        payload.extend_from_slice(&vb[..vn]);
    }
    let ack = m
        .controller_send(Opcode::ConfigWriteMulti.to_u8(), 0x01, &payload)
        .expect("a CONFIG_RESP ack");
    assert_eq!(resp_status(&ack), CFG_OK);

    for (k, v) in &defs {
        let r = m.config_read(0x01, *k);
        assert_eq!(resp_status(&r), CFG_OK);
        assert_eq!(resp_value(&r).as_ref(), Some(v));
    }
}

// ---------------------------------------------------------------------------------------------------
// Slice 6: delivery classes (best-effort vs acknowledged) + reboot recovery (re-learn from traffic).
// ---------------------------------------------------------------------------------------------------

/// An L7 best-effort opcode in the reserved 0x10..0x2F control block L3 forwards but never
/// interprets: the real `CYCLIC_STATE` allocation (`specs/link-control.md`, "Opcode allocation";
/// formerly the placeholder `DRIVE`, renamed on touch per that spec).
const CYCLIC_STATE: u8 = 0x10;

#[test]
fn best_effort_drive_is_fire_and_forget_no_ack() {
    // A best-effort CYCLIC_STATE to the slave is forwarded by the gateway and delivered, but produces NO
    // response (fire-and-forget, latest-wins). The acknowledged CONFIG_WRITE, by contrast, always
    // answers - the observable delivery-class difference.
    let mut m = walked_pair(); // gateway 0x01, slave 0x02

    // No CONFIG_RESP / ack comes back for a CYCLIC_STATE, even sent twice (the second simply supersedes).
    assert!(m
        .controller_send(CYCLIC_STATE, 0x02, &[0x11, 0x22])
        .is_none());
    assert!(m
        .controller_send(CYCLIC_STATE, 0x02, &[0x33, 0x44])
        .is_none());

    // An acknowledged CONFIG_WRITE to the same board does answer.
    let w = m.config_write(0x02, MOTOR_METHOD.key(), Value::U8(1));
    assert_eq!(resp_status(&w), CFG_OK);
}

#[test]
fn an_acknowledged_write_is_idempotent_under_retransmit() {
    // The acknowledged class: request -> response, and the controller retransmits on a (notionally)
    // dropped ACK. Re-applying the same CONFIG_WRITE just re-persists and re-ACKs OK - idempotent, the
    // value is the written one, not corrupted or doubled.
    let mut m = walked_pair();
    let key = MOTOR_CURRENT_LIMIT.key();

    let first = m.config_write(0x02, key, Value::U32(7_000));
    assert_eq!(resp_status(&first), CFG_OK);
    // Pretend the first ACK was lost: retransmit the identical write.
    let retry = m.config_write(0x02, key, Value::U32(7_000));
    assert_eq!(resp_status(&retry), CFG_OK);

    let r = m.config_read(0x02, key);
    assert_eq!(resp_value(&r), Some(Value::U32(7_000)));
}

#[test]
fn reboot_recovery_relearns_routes_from_cyclic_traffic() {
    // Walk the full tree, then power-cycle every board: addresses persist, the routing tables do not.
    let mut m = Mesh::new();
    let gw = m.add_board(3);
    let s1 = m.add_board(1);
    let s2 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.wire(gw, 2, s2, 0);
    m.run_walk();
    let (a1, a2) = (m.live_addr(s1), m.live_addr(s2)); // 0x02 / 0x03 by position

    m.reboot_all();

    // Addresses survive the reboot (re-read from flash); the tables are wiped.
    assert_eq!(m.live_addr(gw), 0x01);
    assert_eq!(m.live_addr(s1), a1);
    assert_eq!(m.live_addr(s2), a2);
    assert!(m.port_toward(gw, a1).is_none());
    assert!(m.port_toward(gw, a2).is_none());

    // A cold cross-tree send: s1 -> s2 with both tables empty. It floods (dst unknown), reaches the far
    // node, and teaches the source's direction at every hop.
    m.board_originate(s1, CYCLIC_STATE, a2, &[0x01]);
    assert_eq!(m.port_toward(s2, a1), Some(0)); // the flood REACHED s2 (it learned s1 on its port 0)
    assert_eq!(m.port_toward(gw, a1), Some(1)); // the gateway learned s1's direction

    // The reply teaches the reverse path; after one cyclic exchange every table is re-learned.
    m.board_originate(s2, CYCLIC_STATE, a1, &[0x02]);
    assert_eq!(m.port_toward(gw, a2), Some(2));
    assert_eq!(m.port_toward(s1, a2), Some(0));

    // Routes are now known: a CONFIG addressed across the tree works without re-running the walk.
    let r = m.config_read(a2, MOTOR_METHOD.key());
    // (sent from the controller, which is still wired; the gateway now routes by its re-learned table)
    assert_eq!(resp_status(&r), CFG_OK);
}

// ---------------------------------------------------------------------------------------------------
// Integration seams (specs/integration.md): the delivered-PDU hand-back + the R4 armed write gate.
// ---------------------------------------------------------------------------------------------------

/// Feed one raw frame straight into a lone responder, returning the hand-back and the emissions.
fn ingest_one(
    resp: &mut Responder,
    flash: &mut TestFlash,
    frame: &[u8],
) -> (Option<DeliveredPdu>, Emits) {
    let mut s = Store::mount(flash).unwrap();
    let mut emits = Emits::new();
    let handed = resp.ingest(0, frame, &mut s, &mut emits);
    (handed, emits)
}

#[test]
fn a_delivered_control_block_pdu_is_handed_back() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, 0x0001);

    // A dst=0x00 point-to-point PDU in the reserved control block is delivered locally (the
    // forwarder's point-to-point rule) and handed back uninterpreted: opcode, src, payload.
    let pdu = Pdu::new(CYCLIC_STATE, 0x02, NO_ADDRESS, &[1, 2, 3]).unwrap();
    let mut buf = [0u8; MAX_PDU];
    let n = pdu.encode(&mut buf).unwrap();
    let (handed, emits) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    let handed = handed.expect("delivered-but-unhandled PDU handed back");
    assert_eq!(handed.opcode, CYCLIC_STATE);
    assert_eq!(handed.src, 0x02);
    assert_eq!(&handed.payload[..], &[1, 2, 3]);
    assert!(
        emits.is_empty(),
        "point-to-point delivery, nothing forwarded"
    );

    // Anywhere else in the 0x10..0x2F block: same hand-back, still uninterpreted by net.
    let pdu = Pdu::new(0x2E, 0x05, NO_ADDRESS, &[]).unwrap();
    let n = pdu.encode(&mut buf).unwrap();
    let (handed, _) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    let handed = handed.expect("whole control block hands back");
    assert_eq!(handed.opcode, 0x2E);
    assert_eq!(handed.src, 0x05);
    assert_eq!(&handed.payload[..], &[] as &[u8]);

    // A stray controller-bound walk opcode (CONFIG_RESP) delivered to a board is unhandled too:
    // handed back for the caller to drop (the previous silent-ignore, now the caller's decision).
    let pdu = Pdu::from_op(Opcode::ConfigResp, 0x80, NO_ADDRESS, &[0, 0, CFG_OK, 0]);
    let n = pdu.encode(&mut buf).unwrap();
    let (handed, _) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    assert_eq!(
        handed.map(|h| h.opcode),
        Some(Opcode::ConfigResp.to_u8()),
        "stray controller-bound opcode hands back"
    );
}

#[test]
fn a_handled_walk_opcode_returns_no_hand_back() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, 0x0001);

    // A handled opcode (ASSIGN to self) is consumed by the walk layer: emissions, no hand-back.
    let assign = Pdu::from_op(Opcode::Assign, 0x80, NO_ADDRESS, &[EGRESS_SELF, 0x05]);
    let mut buf = [0u8; MAX_PDU];
    let n = assign.encode(&mut buf).unwrap();
    let (handed, emits) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    assert!(handed.is_none(), "handled opcodes are not handed back");
    assert!(!emits.is_empty(), "the ASSIGN_ACK was emitted");
    assert_eq!(resp.addr(), 0x05);
}

#[test]
fn hand_back_is_bounded_by_max_pdu() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, 0x0001);

    // The largest deliverable frame: header + (MAX_PDU - 3) payload bytes == MAX_PDU. The whole
    // payload comes back intact (a full bounded copy, no truncation).
    let payload = [0xA5u8; MAX_PDU - 3];
    let mut frame = StdVec::new();
    frame.extend_from_slice(&[CYCLIC_STATE, 0x02, NO_ADDRESS]);
    frame.extend_from_slice(&payload);
    assert_eq!(frame.len(), MAX_PDU);
    let (handed, _) = ingest_one(&mut resp, &mut flash, &frame);
    let handed = handed.expect("a MAX_PDU frame hands back");
    assert_eq!(handed.payload.len(), MAX_PDU - 3);
    assert_eq!(&handed.payload[..], &payload[..]);

    // One byte more: the frame exceeds MAX_PDU, so the delivered copy cannot be taken and the
    // PDU is dropped whole (best-effort class), never truncated.
    frame.push(0xA5);
    let (handed, emits) = ingest_one(&mut resp, &mut flash, &frame);
    assert!(
        handed.is_none(),
        "an over-MAX_PDU frame is dropped, not truncated"
    );
    assert!(emits.is_empty());
}

#[test]
fn an_armed_board_rejects_config_writes_reads_unaffected() {
    // R4 driven end to end through a loopback Responder: controller -> L2 link -> gateway board.
    let mut m = walked_pair(); // gateway 0x01, slave 0x02
    let key = MOTOR_CURRENT_LIMIT.key();

    // Baseline: disarmed, a write persists.
    let w = m.config_write(0x01, key, Value::U32(9_000));
    assert_eq!(resp_status(&w), CFG_OK);

    // Arm the gateway (the firmware samples mode.any_moe_allowed() into the responder each loop
    // pass; the test plays that caller).
    m.boards[0].resp.set_armed(true);
    assert!(m.boards[0].resp.armed());

    // Writes now answer CFG_ARMED and persist nothing.
    let w = m.config_write(0x01, key, Value::U32(11_000));
    assert_eq!(resp_status(&w), CFG_ARMED);

    // Reads are unaffected and still see the pre-arm value.
    let r = m.config_read(0x01, key);
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_value(&r), Some(Value::U32(9_000)));

    // Disarm: the same write goes through.
    m.boards[0].resp.set_armed(false);
    let w = m.config_write(0x01, key, Value::U32(11_000));
    assert_eq!(resp_status(&w), CFG_OK);
    assert_eq!(
        resp_value(&m.config_read(0x01, key)),
        Some(Value::U32(11_000))
    );
}

#[test]
fn an_armed_board_rejects_write_multi_and_gates_before_decode() {
    let mut m = walked_pair();
    let key = MOTOR_METHOD.key();

    // Pre-arm value.
    let w = m.config_write(0x01, key, Value::U8(2));
    assert_eq!(resp_status(&w), CFG_OK);
    m.boards[0].resp.set_armed(true);

    // CONFIG_WRITE_MULTI while armed: a single CFG_ARMED ack, nothing persisted.
    // payload = [count, field_id, index, type_tag, vlen, value].
    let payload = std::vec![
        1u8,
        key.field_id,
        key.index,
        Value::U8(3).kind().tag(),
        1,
        3
    ];
    let ack = m
        .controller_send(Opcode::ConfigWriteMulti.to_u8(), 0x01, &payload)
        .expect("a CONFIG_RESP ack");
    assert_eq!(resp_status(&ack), CFG_ARMED);

    // The gate runs before value decode: a malformed write (bad type tag) still answers
    // CFG_ARMED, not CFG_BAD, while armed. An armed board persists nothing regardless of shape.
    let w = m
        .controller_send(
            Opcode::ConfigWrite.to_u8(),
            0x01,
            &[key.field_id, key.index, 0xEE],
        )
        .expect("a CONFIG_RESP");
    assert_eq!(resp_status(&w), CFG_ARMED);

    // Still the pre-arm value.
    m.boards[0].resp.set_armed(false);
    assert_eq!(resp_value(&m.config_read(0x01, key)), Some(Value::U8(2)));
}

#[test]
fn an_armed_board_refuses_assign_no_persist_no_adopt() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, 0x0001);
    resp.set_armed(true);

    // ASSIGN(egress=SELF, 0x05) delivered to an armed board is refused whole: the ACK carries
    // STATUS_ERR (a retryable refusal), the address is unchanged (no in-RAM adopt), and the
    // flash is byte-for-byte untouched (no program, no erase; storage-layer.md's armed rule).
    let assign = Pdu::from_op(Opcode::Assign, 0x80, NO_ADDRESS, &[EGRESS_SELF, 0x05]);
    let mut buf = [0u8; MAX_PDU];
    let n = assign.encode(&mut buf).unwrap();
    let before = flash.bytes.clone();
    let (handed, emits) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    assert!(
        handed.is_none(),
        "ASSIGN is handled (refused), not handed back"
    );
    assert_eq!(ack_status(&emits), Some((0x05, STATUS_ERR)));
    assert_eq!(resp.addr(), NO_ADDRESS, "no in-RAM adopt while armed");
    assert_eq!(flash.bytes, before, "no flash program while armed");

    // Disarm: the controller's retry of the SAME ASSIGN succeeds (persists, adopts, ACKs OK).
    resp.set_armed(false);
    let (_, emits) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    assert_eq!(ack_status(&emits), Some((0x05, STATUS_OK)));
    assert_eq!(resp.addr(), 0x05);
    let s = Store::mount(&mut flash).unwrap();
    assert_eq!(s.get_value(NODE_ADDRESS.key()).unwrap(), Value::U8(0x05));
}

#[test]
fn armed_assign_refused_end_to_end_and_the_relay_stays_ungated() {
    let mut m = walked_pair(); // gateway 0x01, slave 0x02 (board indices 0, 1)

    // Arm the SLAVE: a directed ASSIGN relayed through the gateway is refused at the slave.
    // Address unchanged, slave flash byte-for-byte untouched.
    m.boards[1].resp.set_armed(true);
    let before = m.boards[1].flash.bytes.clone();
    let none = m.controller_send(Opcode::Assign.to_u8(), 0x01, &[1, 0x55]);
    assert!(none.is_none()); // the refusal is an ASSIGN_ACK, not a CONFIG_RESP
    assert_eq!(m.live_addr(1), 0x02, "address unchanged while armed");
    assert_eq!(m.persisted_addr(1), 0x02);
    assert_eq!(
        m.boards[1].flash.bytes, before,
        "no flash write while armed"
    );

    // The relay branch stays ungated (no flash on the relay): arm the GATEWAY instead; the same
    // directed ASSIGN still relays through it, the now-disarmed slave accepts and persists, and
    // the armed gateway's flash is untouched.
    m.boards[1].resp.set_armed(false);
    m.boards[0].resp.set_armed(true);
    let gw_before = m.boards[0].flash.bytes.clone();
    m.controller_send(Opcode::Assign.to_u8(), 0x01, &[1, 0x55]);
    assert_eq!(
        m.live_addr(1),
        0x55,
        "the retried ASSIGN succeeds after disarm"
    );
    assert_eq!(m.persisted_addr(1), 0x55);
    assert_eq!(m.boards[0].flash.bytes, gw_before, "relaying is flash-free");
}

#[test]
fn a_controller_walk_against_an_armed_board_records_nothing_and_probes_nothing() {
    // The walk DRIVER half of the armed refusal: the controller sends ASSIGN, the armed board
    // ACKs STATUS_ERR, and the controller must treat that as a refusal, not an assignment: the
    // address stays out of its map and no probe of the never-adopted address is enqueued (the
    // walk runs to completion with nothing recorded; retry policy stays with the host).
    let mut m = Mesh::new();
    let gw = m.add_board(1);
    m.attach_controller(gw, 0);
    m.boards[gw].resp.set_armed(true);

    m.run_walk();

    assert!(m.ctrl.ctrl.is_complete());
    assert!(
        m.ctrl.ctrl.assigned_addrs().is_empty(),
        "a refused address is not recorded as assigned"
    );
    assert!(
        !m.boards[gw].resp.probing(),
        "no probe of the never-adopted address"
    );
    assert_eq!(m.live_addr(gw), NO_ADDRESS);
    assert_eq!(m.persisted_addr(gw), 0x00); // the NODE_ADDRESS default: nothing persisted

    // Disarm and re-drive the walk (a fresh controller session, the host's retry): assigns.
    m.boards[gw].resp.set_armed(false);
    m.ctrl.ctrl = Controller::new();
    m.run_walk();
    assert_eq!(m.ctrl.ctrl.assigned_addrs().to_vec(), std::vec![0x01]);
    assert_eq!(m.live_addr(gw), 0x01);
    assert_eq!(m.persisted_addr(gw), 0x01);
}
