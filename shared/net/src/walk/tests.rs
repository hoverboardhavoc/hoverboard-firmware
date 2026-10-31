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

use crate::pdu::{is_unicast, BROADCAST, NO_ADDRESS};
use store::{
    Flash, Key, Store, StoreError, Type, Value, DEVICE_NAME, MOTOR_CURRENT_LIMIT, MOTOR_METHOD,
    NODE_ADDRESS,
};

const PS: usize = 1024;
const CAP: usize = 255;
/// A board's class, the registry default of `node.class`.
const MOTOR: u8 = store::NODE_CLASS.default();

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
    fn send_l2_frame(&mut self, l2: &[u8]) -> bool {
        self.tx.borrow_mut().push_back(l2.to_vec());
        true
    }
    fn encode_l2_frame(&self, l2: &[u8], out: &mut [u8]) -> Option<usize> {
        if out.len() < l2.len() {
            return None;
        }
        out[..l2.len()].copy_from_slice(l2);
        Some(l2.len())
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
    /// Every request the controller sent, in order.
    sent: StdVec<StdVec<u8>>,
    /// The frames `on_reply` refused as not the outstanding request's reply, each with the
    /// request that was outstanding when it arrived.
    refused: StdVec<(StdVec<u8>, Option<Task>)>,
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
                sent: StdVec::new(),
                refused: StdVec::new(),
            },
        }
    }

    /// Add a board with `n_ports` ports; returns its index.
    fn add_board(&mut self, n_ports: u8) -> usize {
        self.add_board_kinds(n_ports, [PORT_UART; MAX_PORTS])
    }

    /// Add a board whose ports carry the given medium tags (the real fleet's master is port 0 =
    /// SWD mailbox, port 1 = the inter-board UART, port 2 = BLE).
    fn add_board_kinds(&mut self, n_ports: u8, kinds: [u8; MAX_PORTS]) -> usize {
        self.add_node(n_ports, kinds, MOTOR)
    }

    /// Add a node of `class` (a bridge, or a motor controller) whose ports carry `kinds`.
    fn add_node(&mut self, n_ports: u8, kinds: [u8; MAX_PORTS], class: u8) -> usize {
        let mut ports = StdVec::new();
        for _ in 0..n_ports {
            ports.push(None);
        }
        self.boards.push(Board {
            resp: Responder::new(n_ports, kinds, /*mcu*/ 0x10, class, /*fw*/ 0x0001),
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
        self.run_walk_hooked(|_, _| {});
    }

    /// Drive the walk to completion, calling `hook` with each request right after it is sent and
    /// before the mesh settles (a test injects a retransmitted copy of an earlier request there).
    fn run_walk_hooked(&mut self, mut hook: impl FnMut(&mut Mesh, &[u8])) {
        for _ in 0..1000 {
            if self.ctrl.ctrl.is_complete() {
                return;
            }
            if let Some(req) = self.ctrl.ctrl.next_request() {
                self.ctrl.link.as_mut().unwrap().send(&req).unwrap();
                self.ctrl.sent.push(req.to_vec());
                hook(self, &req);
            }
            self.settle();
        }
        panic!("walk did not complete");
    }

    /// Send `frame` out the controller's attach link, as a retransmitted request does.
    fn ctrl_send_raw(&mut self, frame: &[u8]) {
        self.ctrl.link.as_mut().unwrap().send(frame).unwrap();
    }

    /// The opcodes of every request the controller sent.
    fn sent_ops(&self) -> StdVec<Option<Opcode>> {
        self.ctrl
            .sent
            .iter()
            .map(|f| Pdu::decode(f).unwrap().known())
            .collect()
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
                    let outstanding = self.ctrl.ctrl.outstanding;
                    if !self.ctrl.ctrl.on_reply(&frame) {
                        self.ctrl.refused.push((frame, outstanding));
                    }
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
        match s.get_value(NODE_ADDRESS.key()).unwrap().0 {
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
fn attached_node_resolves_to_the_gateway_when_it_persisted_a_higher_address() {
    // THE 2026-07-31 ARM-SESSION TOPOLOGY, exactly. The attached master had persisted 0x02 in an
    // earlier session, so it answers first contact already holding 0x02; fresh allocation starts at
    // 0x01, so the SLAVE gets 0x01. A bench command aimed at the literal `0x01` therefore reaches
    // the slave, which is how that session's first power request armed the wrong board.
    let mut m = Mesh::new();
    let master = m.add_board_kinds(2, [PORT_SWD, PORT_UART, PORT_UART, PORT_UART]);
    let slave = m.add_board(1);
    m.preassign(master, 0x02);
    m.attach_controller(master, 0);
    m.wire(master, 1, slave, 0);

    m.run_walk();

    // The trap itself: the low address is the FAR board.
    assert_eq!(m.live_addr(master), 0x02);
    assert_eq!(m.live_addr(slave), 0x01);

    // The resolution: first contact answered from the master, so it is the attached node.
    assert_eq!(m.ctrl.ctrl.gateway_addr(), Some(0x02));

    // And the fleet's own corroboration: the master reports the host on its mailbox port.
    let hl = m
        .ctrl
        .ctrl
        .host_link()
        .expect("the master's PORTS reply names the controller on its mailbox port");
    assert_eq!(hl.node, 0x02);
    assert_eq!(hl.port, 0);
    assert_eq!(hl.kind, PORT_SWD);
}

#[test]
fn host_link_names_the_attached_board_not_a_board_behind_it() {
    // The far board's port table shows only the master (an inter-board UART neighbour), never the
    // controller, so the host-link evidence can never point past the board the host is plugged into.
    let mut m = Mesh::new();
    let gw = m.add_board_kinds(2, [PORT_SWD, PORT_UART, PORT_UART, PORT_UART]);
    let far = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, far, 0);

    m.run_walk();

    let hl = m.ctrl.ctrl.host_link().expect("gateway reports the host");
    assert_eq!(hl.node, m.live_addr(gw));
    assert_ne!(hl.node, m.live_addr(far));
    assert_eq!(m.ctrl.ctrl.gateway_addr(), Some(m.live_addr(gw)));
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
fn a_node_assigned_this_walk_moves_off_an_address_a_neighbour_already_held() {
    // The gateway is fresh and is assigned 0x01; a sideboard boots holding 0x01 from a past session.
    // The walk sees 0x01 at two positions and moves the node IT assigned (the gateway), so the
    // sideboard keeps the address it held (`specs/l3.md`, "Discovery and address assignment").
    let mut m = Mesh::new();
    let gw = m.add_board(2);
    let s1 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.preassign(s1, 0x01);

    m.run_walk();

    assert_eq!(m.live_addr(s1), 0x01, "the neighbour keeps what it held");
    assert_eq!(m.live_addr(gw), 0x02, "the node this walk assigned moves");
    assert_eq!(m.persisted_addr(gw), 0x02); // and re-persists
    assert_eq!(m.persisted_addr(s1), 0x01);
    assert_eq!(m.ctrl.ctrl.gateway_addr(), Some(0x02));
    let mut handed = m.ctrl.ctrl.assigned_addrs().to_vec();
    handed.sort_unstable();
    assert_eq!(handed, [0x01, 0x02]);
}

#[test]
fn two_nodes_that_both_held_one_address_reassign_the_one_reached_second() {
    // Neither address came from this walk: the gateway and the sideboard both answer holding 0x05
    // (two clones assigned separately). The neighbour reached second is reassigned and re-persists.
    let mut m = Mesh::new();
    let gw = m.add_board(2);
    let s1 = m.add_board(1);
    m.attach_controller(gw, 0);
    m.wire(gw, 1, s1, 0);
    m.preassign(gw, 0x05);
    m.preassign(s1, 0x05);

    m.run_walk();

    assert_eq!(m.live_addr(gw), 0x05);
    let a1 = m.live_addr(s1);
    assert_ne!(a1, 0x05); // reassigned off the collision
    assert!(pdu::is_board(a1));
    assert_eq!(m.persisted_addr(s1), a1); // and re-persisted
}

#[test]
fn a_fresh_bridge_in_front_of_an_addressed_pair_ends_at_0x03_and_the_boards_keep_theirs() {
    // `specs/hoverboard-bridge.md`, "A fresh bridge in front of an addressed machine": the bridge (ports BLE,
    // mailbox, console) boots unassigned while its master holds 0x01 and the slave 0x02. First
    // contact assigns the bridge 0x01, its mailbox probe finds the master at 0x01, it moves to
    // 0x02, collides with the slave behind the master, and ends at 0x03.
    let mut m = Mesh::new();
    let bridge = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    let master = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    let slave = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    m.preassign(master, 0x01);
    m.preassign(slave, 0x02);
    m.attach_controller(bridge, 2);
    m.wire(bridge, 1, master, 0);
    m.wire(master, 1, slave, 1);

    m.run_walk();

    assert_eq!(m.live_addr(bridge), 0x03);
    assert_eq!(m.persisted_addr(bridge), 0x03);
    assert_eq!(
        (m.live_addr(master), m.persisted_addr(master)),
        (0x01, 0x01)
    );
    assert_eq!((m.live_addr(slave), m.persisted_addr(slave)), (0x02, 0x02));
    let mut handed = m.ctrl.ctrl.assigned_addrs().to_vec();
    handed.sort_unstable();
    assert_eq!(handed, [0x01, 0x02, 0x03]);
    assert_eq!(m.ctrl.ctrl.gateway_addr(), Some(0x03));
    let hl = m
        .ctrl
        .ctrl
        .host_link()
        .expect("the bridge reports the host");
    assert_eq!((hl.node, hl.port, hl.kind), (0x03, 2, PORT_UART));

    // Each node reports its class: 2 for the bridge, 1 for the boards.
    for (addr, class) in [(0x03, CLASS_BRIDGE), (0x01, MOTOR), (0x02, MOTOR)] {
        let r = m.config_read(addr, store::NODE_CLASS.key());
        assert_eq!(resp_status(&r), CFG_OK, "{addr:#04x}");
        assert_eq!(resp_value(&r), Some(Value::U8(class)), "{addr:#04x}");
    }
    // A class write is refused by the bridge too.
    let w = m.config_write(0x03, store::NODE_CLASS.key(), Value::U8(0x01));
    assert_eq!(resp_status(&w), CFG_READ_ONLY);

    // A second walk changes nothing.
    m.ctrl.ctrl = Controller::new();
    m.run_walk();
    let mut again = m.ctrl.ctrl.assigned_addrs().to_vec();
    again.sort_unstable();
    assert_eq!(again, [0x01, 0x02, 0x03]);
    assert_eq!(m.live_addr(bridge), 0x03);
}

#[test]
fn a_bridge_refuses_a_name_it_cannot_advertise_and_keeps_the_old_one() {
    // A bridge advertises `device.name` over BLE and answers mDNS for it, so a name that fails
    // `bridge_name_usable` is refused with CFG_BAD and the store keeps its value. A board takes it.
    let mut m = Mesh::new();
    let bridge = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    let board = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    m.attach_controller(bridge, 2);
    m.wire(bridge, 1, board, 0);
    m.run_walk();
    let b = m.live_addr(bridge);
    let read_name = |m: &mut Mesh, addr: u8| {
        let r = m.config_read(addr, DEVICE_NAME.key());
        assert_eq!(resp_status(&r), CFG_OK);
        match resp_value(&r) {
            Some(Value::Str(s)) => s.to_vec(),
            other => panic!("unexpected {other:?}"),
        }
    };

    let w = m.config_write(b, DEVICE_NAME.key(), Value::Str(b"rover-esp"));
    assert_eq!(resp_status(&w), CFG_OK);
    let longest = [b'a'; BRIDGE_NAME_MAX];
    let w = m.config_write(b, DEVICE_NAME.key(), Value::Str(&longest));
    assert_eq!(resp_status(&w), CFG_OK);
    let w = m.config_write(b, DEVICE_NAME.key(), Value::Str(b"rover-esp"));
    assert_eq!(resp_status(&w), CFG_OK);

    let too_long = [b'a'; BRIDGE_NAME_MAX + 1];
    let unusable: [&[u8]; 4] = [b"Offroad master", &too_long, b"", "r\u{f8}ver".as_bytes()];
    for name in unusable {
        let w = m.config_write(b, DEVICE_NAME.key(), Value::Str(name));
        assert_eq!(resp_status(&w), CFG_BAD, "{name:?}");
        assert_eq!(read_name(&mut m, b), b"rover-esp", "{name:?}");
    }

    // The same refusal through CONFIG_WRITE_MULTI.
    let name = b"Offroad master";
    let mut payload = std::vec![1, DEVICE_NAME.key().field_id, DEVICE_NAME.key().index];
    payload.extend_from_slice(&[Type::Str.tag(), name.len() as u8]);
    payload.extend_from_slice(name);
    let ack = m
        .controller_send(Opcode::ConfigWriteMulti.to_u8(), b, &payload)
        .expect("a CONFIG_RESP ack");
    assert_eq!(resp_status(&ack), CFG_BAD);
    assert_eq!(read_name(&mut m, b), b"rover-esp");

    // A board does not advertise its name and takes any string.
    let addr = m.live_addr(board);
    let w = m.config_write(addr, DEVICE_NAME.key(), Value::Str(b"Offroad master"));
    assert_eq!(resp_status(&w), CFG_OK);
    assert_eq!(read_name(&mut m, addr), b"Offroad master");
}

// ---------------------------------------------------------------------------------------------------
// Responder-level: idempotent re-ASSIGN (a dropped ASSIGN_ACK recovers).
// ---------------------------------------------------------------------------------------------------

#[test]
fn re_assigning_the_same_address_re_persists_and_re_acks_idempotently() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);

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
    assert_eq!(s.get_value(NODE_ADDRESS.key()).unwrap().0, Value::U8(0x05));
}

#[test]
fn an_assign_that_fills_the_page_compacts_and_still_persists() {
    // The audit-caught sibling of the CONFIG_WRITE compaction defect (D1, balance-prep round):
    // ASSIGN is the store's OTHER remote writer, and its persist path lacked the same
    // "compact() then retry" that CONFIG_WRITE got, so a board whose active page had filled
    // would refuse re-addressing over the wire, permanently. Fill the page through the wire
    // path, then ASSIGN, and require the persist to succeed via compaction.
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);

    // Fill the active page TO THE BRINK by direct store writes (deliberately not the wire: the
    // wire's config path now compacts for itself, which would leave room and make this test
    // pass with or without the ASSIGN fix, as the negative control proved. The property under
    // test is the ASSIGN handler's own Full handling, so the fill stops at the first Full
    // WITHOUT compacting).
    {
        let mut s = Store::mount(&mut flash).unwrap();
        let cw_key = MOTOR_CURRENT_LIMIT.key();
        let mut i = 0u32;
        loop {
            match s.set_value(cw_key, Value::U32(2_000 + i)) {
                Ok(()) => i += 1,
                Err(DynError::Store(StoreError::Full)) => break,
                Err(e) => panic!("unexpected store error while filling: {e:?}"),
            }
            assert!(i < 4_096, "page never filled");
        }
    }

    // Now the ASSIGN. Without the retry this returns STATUS_ERR on a Full page; with it, the
    // compaction runs (disarmed: the armed gate above the persist is not in play here) and the
    // address both persists and is adopted.
    let assign = Pdu::from_op(Opcode::Assign, 0x80, NO_ADDRESS, &[EGRESS_SELF, 0x07]);
    let mut buf = [0u8; MAX_PDU];
    let n = assign.encode(&mut buf).unwrap();
    let ack = {
        let mut s = Store::mount(&mut flash).unwrap();
        let mut emits = Emits::new();
        resp.ingest(0, &buf[..n], &mut s, &mut emits);
        emits
    };
    assert_eq!(ack_status(&ack), Some((0x07, STATUS_OK)));
    assert_eq!(resp.addr(), 0x07);
    let s = Store::mount(&mut flash).unwrap();
    assert_eq!(s.get_value(NODE_ADDRESS.key()).unwrap().0, Value::U8(0x07));
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
fn config_read_of_an_unwritten_field_answers_cfg_default_with_its_default() {
    let mut m = walked_pair();
    let r = m.config_read(0x01, DEVICE_NAME.key());
    assert_eq!(resp_status(&r), CFG_DEFAULT);
    assert_eq!(CFG_DEFAULT, 9);
    assert_eq!(resp_value(&r), Some(Value::Str(b"Hoverboard")));
    // Once written, the same read answers CFG_OK.
    let w = m.config_write(0x01, DEVICE_NAME.key(), Value::Str(b"rover-left"));
    assert_eq!(resp_status(&w), CFG_OK);
    let r = m.config_read(0x01, DEVICE_NAME.key());
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_value(&r), Some(Value::Str(b"rover-left")));
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
        match s.get_value(key).unwrap().0 {
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
        (DEVICE_NAME.key(), Value::Str(b"rig-9")),
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
    assert_eq!(resp_status(&r), CFG_DEFAULT);
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
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);

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
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);

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
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);

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
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);
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
    assert_eq!(s.get_value(NODE_ADDRESS.key()).unwrap().0, Value::U8(0x05));
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

#[test]
fn a_config_write_that_fills_the_page_compacts_and_keeps_accepting() {
    // The bench defect (2026-07-31): staging six fields on the master wrote two and then returned
    // CFG_STORE_ERR for the rest. The store's `Full` is not a failure, it is its documented
    // "compact() then retry" -- the value fits a clean page, just not the active page's remaining
    // space -- and this writer (one of the store's TWO remote writers; the ASSIGN persist is the other, audit-corrected census) was not doing the retry. A board would
    // simply stop accepting configuration once its active page filled, permanently, with no way
    // to fix it over the wire.
    //
    // Drive the page full through the wire path itself (not by poking the store), then keep going.
    let mut m = walked_pair();
    let key = MOTOR_CURRENT_LIMIT.key();

    // Enough distinct writes of the same key to overrun a page several times over: an append-only
    // log appends a record per write, so this is what fills it.
    for i in 0..400u32 {
        let w = m.config_write(0x01, key, Value::U32(1_000 + i));
        assert_eq!(
            resp_status(&w),
            CFG_OK,
            "write {i} must be accepted (compaction is the store's own remedy for a full page)"
        );
    }

    // Still correct after all that churn: the latest value is what reads back, through the log's
    // latest-wins rule across however many compactions ran.
    let r = m.config_read(0x01, key);
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_value(&r), Some(Value::U32(1_399)));

    // And a DIFFERENT field written after the churn still lands, i.e. compaction preserved the
    // rest of the log rather than trading one field's survival for another's.
    let other = MOTOR_METHOD.key();
    assert_eq!(
        resp_status(&m.config_write(0x01, other, Value::U8(1))),
        CFG_OK
    );
    assert_eq!(resp_value(&m.config_read(0x01, other)), Some(Value::U8(1)));
    assert_eq!(
        resp_value(&m.config_read(0x01, key)),
        Some(Value::U32(1_399))
    );
}

#[test]
fn an_armed_board_still_refuses_the_write_before_any_compaction_can_run() {
    // Compaction erases and programs flash, stalling the main loop for 60 ms on a full 1 KiB page
    // (measured, `specs/storage-layer.md`, "Compaction and the watchdog window"). That is only
    // acceptable because the armed gate refuses the write FIRST, so a compaction can never run
    // under live torque. This pins the ordering.
    let mut m = walked_pair();
    m.boards[0].resp.set_armed(true);
    let w = m.config_write(0x01, MOTOR_CURRENT_LIMIT.key(), Value::U32(15_000));
    assert_eq!(
        resp_status(&w),
        CFG_ARMED,
        "armed refusal, not a store attempt"
    );
}

// ---------------------------------------------------------------------------------------------------
// Un-storing a key and compacting on request (`specs/cli-config.md` sections 6.2 and 6.3).
// ---------------------------------------------------------------------------------------------------

/// `CONFIG_WRITE [field_id, index, 0x00]`: the tombstone tag with no value bytes.
fn unstore(m: &mut Mesh, dst: u8, key: Key) -> StdVec<u8> {
    m.controller_send(
        Opcode::ConfigWrite.to_u8(),
        dst,
        &[key.field_id, key.index, store::ERASED_TAG],
    )
    .expect("a CONFIG_RESP")
}

/// `CONFIG_COMPACT []` -> the `[0, 0, status, 0]` ack's status.
fn compact(m: &mut Mesh, dst: u8) -> u8 {
    let r = m
        .controller_send(Opcode::ConfigCompact.to_u8(), dst, &[])
        .expect("a CONFIG_RESP");
    let p = Pdu::decode(&r).unwrap().payload.to_vec();
    assert_eq!(p.len(), 4);
    assert_eq!((p[0], p[1], p[3]), (0, 0, 0));
    p[2]
}

/// The keys of every record on `flash`'s active page, in log order (committed or not).
fn records(flash: &TestFlash) -> StdVec<Key> {
    let ps = flash.page_size;
    let b = &flash.bytes;
    // The active side is the one with the newer seq; this test only needs the one with a magic and
    // a committed seq, and after a compaction the old side is erased.
    let base = (0..2)
        .map(|p| p * ps)
        .filter(|&o| {
            b[o..o + 4] == store::record::MAGIC.to_le_bytes() && b[o + 4..o + 6] != [0xFF; 2]
        })
        .max_by_key(|&o| u16::from_le_bytes([b[o + 4], b[o + 5]]))
        .expect("a valid side");
    let mut keys = StdVec::new();
    let mut off = base + store::record::PAGE_HEADER_LEN;
    while let store::record::HeaderScan::Good(h) = store::record::parse_header(b, off) {
        keys.push(Key {
            field_id: h.field_id,
            index: h.index,
        });
        off += store::record::record_size(h.len as usize);
    }
    keys
}

#[test]
fn an_unstoring_config_write_returns_the_key_to_its_default() {
    let mut m = walked_pair();
    let key = MOTOR_CURRENT_LIMIT.key();
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::U32(15_000))),
        CFG_OK
    );
    assert!(m.boards[0].resp.take_config_persisted());

    let w = unstore(&mut m, 0x01, key);
    assert_eq!(resp_status(&w), CFG_OK);
    // The echo is the value the key reads now, the default.
    assert_eq!(resp_value(&w), Some(Value::U32(10_000)));
    assert!(
        m.boards[0].resp.take_config_persisted(),
        "a stored value moved"
    );
    let r = m.config_read(0x01, key);
    assert_eq!(resp_status(&r), CFG_DEFAULT);
    assert_eq!(resp_value(&r), Some(Value::U32(10_000)));

    // A key with nothing stored: answered OK, nothing written.
    let before = m.boards[0].flash.bytes.clone();
    assert_eq!(resp_status(&unstore(&mut m, 0x01, key)), CFG_OK);
    assert!(m.boards[0].flash.bytes == before);

    // The tombstone tag with value bytes is malformed; an unregistered id is unknown.
    let bad = m
        .controller_send(
            Opcode::ConfigWrite.to_u8(),
            0x01,
            &[key.field_id, key.index, store::ERASED_TAG, 0x01],
        )
        .unwrap();
    assert_eq!(resp_status(&bad), CFG_BAD);
    let unknown = Key {
        field_id: 0x99,
        index: 0,
    };
    assert_eq!(
        resp_status(&unstore(&mut m, 0x01, unknown)),
        CFG_UNKNOWN_KEY
    );
}

#[test]
fn a_write_multi_entry_with_the_tombstone_tag_unstores_its_key() {
    let mut m = walked_pair();
    let (a, b) = (MOTOR_CURRENT_LIMIT.key(), MOTOR_METHOD.key());
    assert_eq!(
        resp_status(&m.config_write(0x01, a, Value::U32(15_000))),
        CFG_OK
    );
    // Un-store `a` and write `b` in one PDU.
    let multi = [
        2,
        a.field_id,
        a.index,
        store::ERASED_TAG,
        0,
        b.field_id,
        b.index,
        Type::U8.tag(),
        1,
        2,
    ];
    let r = m
        .controller_send(Opcode::ConfigWriteMulti.to_u8(), 0x01, &multi)
        .unwrap();
    assert_eq!(resp_status(&r), CFG_OK);
    assert_eq!(resp_status(&m.config_read(0x01, a)), CFG_DEFAULT);
    let r = m.config_read(0x01, b);
    assert_eq!(
        (resp_status(&r), resp_value(&r)),
        (CFG_OK, Some(Value::U8(2)))
    );
}

#[test]
fn an_armed_board_refuses_an_unstore_and_writes_nothing() {
    let mut m = walked_pair();
    let key = MOTOR_CURRENT_LIMIT.key();
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::U32(15_000))),
        CFG_OK
    );
    let _ = m.boards[0].resp.take_config_persisted();
    m.boards[0].resp.set_armed(true);
    let before = m.boards[0].flash.bytes.clone();

    assert_eq!(resp_status(&unstore(&mut m, 0x01, key)), CFG_ARMED);
    let multi = [1, key.field_id, key.index, store::ERASED_TAG, 0];
    let r = m
        .controller_send(Opcode::ConfigWriteMulti.to_u8(), 0x01, &multi)
        .unwrap();
    assert_eq!(resp_status(&r), CFG_ARMED);

    assert!(m.boards[0].flash.bytes == before, "no flash touched");
    assert!(!m.boards[0].resp.take_config_persisted());
    let r = m.config_read(0x01, key);
    assert_eq!(
        (resp_status(&r), resp_value(&r)),
        (CFG_OK, Some(Value::U32(15_000)))
    );
}

#[test]
fn an_unstore_on_a_full_page_compacts_and_retries() {
    let mut m = walked_pair();
    let key = MOTOR_METHOD.key();
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::U8(2))),
        CFG_OK
    );
    // Fill the active page with another key, through the store, stopping at the first Full.
    {
        let mut s = Store::mount(&mut m.boards[0].flash).unwrap();
        while s.set(MOTOR_CURRENT_LIMIT, 1).is_ok() {}
        // An empty name is a 10-byte record, a tombstone's size: once one no longer fits, neither
        // does the tombstone.
        while s.set_str(DEVICE_NAME, "").is_ok() {}
        assert_eq!(s.set_str(DEVICE_NAME, ""), Err(StoreError::Full));
    }
    assert_eq!(resp_status(&unstore(&mut m, 0x01, key)), CFG_OK);
    assert_eq!(resp_status(&m.config_read(0x01, key)), CFG_DEFAULT);
}

#[test]
fn config_compact_leaves_only_the_latest_record_of_each_stored_key() {
    let mut m = walked_pair();
    let (a, b, c) = (
        MOTOR_CURRENT_LIMIT.key(),
        MOTOR_METHOD.key(),
        DEVICE_NAME.key(),
    );
    for v in [11_000, 12_000, 13_000] {
        assert_eq!(resp_status(&m.config_write(0x01, a, Value::U32(v))), CFG_OK);
    }
    assert_eq!(resp_status(&m.config_write(0x01, b, Value::U8(1))), CFG_OK);
    assert_eq!(
        resp_status(&m.config_write(0x01, c, Value::Str(b"x"))),
        CFG_OK
    );
    assert_eq!(resp_status(&unstore(&mut m, 0x01, c)), CFG_OK);
    assert!(records(&m.boards[0].flash).len() > 4);

    assert_eq!(compact(&mut m, 0x01), CFG_OK);
    let mut kept = records(&m.boards[0].flash);
    kept.sort();
    let mut want = std::vec![NODE_ADDRESS.key(), a, b];
    want.sort();
    assert_eq!(
        kept, want,
        "the latest of each stored key; the un-stored one is gone"
    );
    let r = m.config_read(0x01, a);
    assert_eq!(
        (resp_status(&r), resp_value(&r)),
        (CFG_OK, Some(Value::U32(13_000)))
    );
    assert_eq!(resp_status(&m.config_read(0x01, c)), CFG_DEFAULT);
    assert_eq!(m.persisted_addr(0), 0x01);
}

#[test]
fn config_compact_is_refused_while_armed_and_erases_nothing() {
    let mut m = walked_pair();
    for v in [11_000, 12_000] {
        let w = m.config_write(0x01, MOTOR_CURRENT_LIMIT.key(), Value::U32(v));
        assert_eq!(resp_status(&w), CFG_OK);
    }
    m.boards[0].resp.set_armed(true);
    let before = m.boards[0].flash.bytes.clone();
    assert_eq!(compact(&mut m, 0x01), CFG_ARMED);
    assert!(m.boards[0].flash.bytes == before, "no erase, no program");
    m.boards[0].resp.set_armed(false);
    assert_eq!(compact(&mut m, 0x01), CFG_OK);
    assert!(m.boards[0].flash.bytes != before);
}

#[test]
fn a_nodes_own_default_name_answers_an_unstored_device_name() {
    // The bridge's default name is the node's, not the registry's (`specs/cli-config.md` 6.2).
    let mut m = walked_pair();
    m.boards[0].resp.set_default_name(b"hb-bridge-ab12");
    let key = DEVICE_NAME.key();
    let r = m.config_read(0x01, key);
    assert_eq!(
        (resp_status(&r), resp_value(&r)),
        (CFG_DEFAULT, Some(Value::Str(b"hb-bridge-ab12")))
    );
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::Str(b"rover-esp"))),
        CFG_OK
    );
    let r = m.config_read(0x01, key);
    assert_eq!(
        (resp_status(&r), resp_value(&r)),
        (CFG_OK, Some(Value::Str(b"rover-esp")))
    );
    // An un-store echoes the node's default, and the board without one keeps the registry's.
    let w = unstore(&mut m, 0x01, key);
    assert_eq!(resp_value(&w), Some(Value::Str(b"hb-bridge-ab12")));
    let r = m.config_read(0x02, key);
    assert_eq!(resp_value(&r), Some(Value::Str(b"Hoverboard")));
}

/// `specs/bridge-onboarding.md` section 2.2: a stored secret reads back as `CFG_SECRET` with no
/// value, on a board and on a bridge alike; an unstored one answers `CFG_DEFAULT` with the empty
/// default, and a write's echo carries no value either.
#[test]
fn a_stored_secret_answers_cfg_secret_with_no_value_on_a_board_and_on_a_bridge() {
    assert_eq!(CFG_SECRET, 10);
    let mut m = Mesh::new();
    let bridge = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    let board = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    m.attach_controller(bridge, 2);
    m.wire(bridge, 1, board, 0);
    m.run_walk();
    let key = store::WIFI_PASS.key();
    for addr in [m.live_addr(bridge), m.live_addr(board)] {
        let r = m.config_read(addr, key);
        assert_eq!(
            (resp_status(&r), resp_value(&r)),
            (CFG_DEFAULT, Some(Value::Str(b""))),
            "{addr:#04x}"
        );
        let w = m.config_write(addr, key, Value::Str(b"hunter2hunter2"));
        let p = Pdu::decode(&w).unwrap().payload.to_vec();
        assert_eq!(
            p,
            [key.field_id, 0, CFG_OK, 0],
            "{addr:#04x}: the echo has no value"
        );
        let r = m.config_read(addr, key);
        let p = Pdu::decode(&r).unwrap().payload.to_vec();
        assert_eq!(p, [key.field_id, 0, CFG_SECRET, 0], "{addr:#04x}");
        // The SSID is no secret: it reads back.
        let ssid = store::WIFI_SSID.key();
        m.config_write(addr, ssid, Value::Str(b"bench"));
        let r = m.config_read(addr, ssid);
        assert_eq!(
            (resp_status(&r), resp_value(&r)),
            (CFG_OK, Some(Value::Str(b"bench")))
        );
        // Un-stored, it is the default again.
        assert_eq!(resp_status(&unstore(&mut m, addr, key)), CFG_OK);
        let r = m.config_read(addr, key);
        assert_eq!(
            (resp_status(&r), resp_value(&r)),
            (CFG_DEFAULT, Some(Value::Str(b"")))
        );
    }
}

/// `specs/bridge-onboarding.md` section 2.1: a bridge refuses Wi-Fi credentials its station could
/// not use with `CFG_BAD` and keeps what it held, a value that is not UTF-8 among them; a board
/// stores them. A password longer than one `CONFIG_WRITE` carries cannot be sent at all; the
/// bridge's check on the longer ones is exercised by the bridge core's boot read of a store written
/// before the check existed.
#[test]
fn a_bridge_refuses_wifi_credentials_its_station_cannot_use_and_a_board_stores_them() {
    let mut m = Mesh::new();
    let bridge = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    let board = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    m.attach_controller(bridge, 2);
    m.wire(bridge, 1, board, 0);
    m.run_walk();
    let (b, d) = (m.live_addr(bridge), m.live_addr(board));
    let ssid = store::WIFI_SSID.key();
    let pass = store::WIFI_PASS.key();
    let writable = CONFIG_WRITE_VALUE_MAX;
    assert!(writable < WIFI_PASS_MAX);
    let long_ssid = [b's'; WIFI_SSID_MAX + 1];
    let short = [b'p'; WIFI_PASS_MIN - 1];
    // Not UTF-8, at lengths the length checks take.
    let raw_ssid: &[u8] = b"bench\xff";
    let raw_pass: &[u8] = b"hunter2\xc3(";
    let refused: [(Key, &[u8]); 5] = [
        (ssid, &long_ssid),
        (pass, b"x"),
        (pass, &short),
        (ssid, raw_ssid),
        (pass, raw_pass),
    ];
    for (key, v) in refused {
        let w = m.config_write(b, key, Value::Str(v));
        assert_eq!(resp_status(&w), CFG_BAD, "{key:?} {}", v.len());
        assert_eq!(
            resp_status(&m.config_read(b, key)),
            CFG_DEFAULT,
            "nothing stored"
        );
        let w = m.config_write(d, key, Value::Str(v));
        assert_eq!(
            resp_status(&w),
            CFG_OK,
            "a board stores {key:?} {}",
            v.len()
        );
    }
    let longest = std::vec![b'p'; writable];
    let taken: [(Key, &[u8]); 5] = [
        (ssid, &[b's'; WIFI_SSID_MAX]),
        (ssid, b""),
        (pass, b""),
        (pass, &[b'p'; WIFI_PASS_MIN]),
        (pass, &longest),
    ];
    for (key, v) in taken {
        let w = m.config_write(b, key, Value::Str(v));
        assert_eq!(resp_status(&w), CFG_OK, "{key:?} {}", v.len());
    }
}

/// `specs/bridge-onboarding.md` section 2.4: a bridge refuses a `wifi.tx_power` outside 8 to 84
/// quarter-dBm with `CFG_BAD` and keeps what it held; a board stores any value.
#[test]
fn a_bridge_refuses_a_tx_power_outside_its_range_and_a_board_stores_it() {
    let mut m = Mesh::new();
    let bridge = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    let board = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    m.attach_controller(bridge, 2);
    m.wire(bridge, 1, board, 0);
    m.run_walk();
    let (b, d) = (m.live_addr(bridge), m.live_addr(board));
    let key = store::WIFI_TX_POWER.key();
    for v in [0u8, 7, 85, 255] {
        let w = m.config_write(b, key, Value::U8(v));
        assert_eq!(resp_status(&w), CFG_BAD, "{v}");
        let r = m.config_read(b, key);
        assert_eq!(
            (resp_status(&r), resp_value(&r)),
            (CFG_DEFAULT, Some(Value::U8(34))),
            "nothing stored after {v}"
        );
        let w = m.config_write(d, key, Value::U8(v));
        assert_eq!(resp_status(&w), CFG_OK, "a board stores {v}");
    }
    for v in [8u8, 34, 84] {
        let w = m.config_write(b, key, Value::U8(v));
        assert_eq!(resp_status(&w), CFG_OK, "{v}");
        let r = m.config_read(b, key);
        assert_eq!(
            (resp_status(&r), resp_value(&r)),
            (CFG_OK, Some(Value::U8(v)))
        );
    }
    // A bridge refuses an out-of-range value after an in-range one and keeps the in-range one.
    let w = m.config_write(b, key, Value::U8(85));
    assert_eq!(resp_status(&w), CFG_BAD);
    assert_eq!(resp_value(&m.config_read(b, key)), Some(Value::U8(84)));
    // The wrong type is the store's to refuse, on either class.
    let w = m.config_write(b, key, Value::U16(34));
    assert_eq!(resp_status(&w), CFG_TYPE_MISMATCH);
}

/// The value limits are what the PDU encoder takes: a `CONFIG_WRITE` of [`CONFIG_WRITE_VALUE_MAX`]
/// value bytes and a `CONFIG_RESP` of [`CONFIG_RESP_VALUE_MAX`] fill one PDU, and a byte more
/// fits neither.
#[test]
fn the_config_value_limits_fill_one_pdu() {
    let fits = |op: Opcode, head: usize, value: usize| {
        let payload = std::vec![0u8; head + value];
        let mut buf = [0u8; MAX_PDU];
        Pdu::from_op(op, 0x80, 0x01, &payload).encode(&mut buf).ok()
    };
    assert_eq!(
        fits(Opcode::ConfigWrite, 3, CONFIG_WRITE_VALUE_MAX),
        Some(MAX_PDU)
    );
    assert_eq!(
        fits(Opcode::ConfigWrite, 3, CONFIG_WRITE_VALUE_MAX + 1),
        None
    );
    assert_eq!(
        fits(Opcode::ConfigResp, 4, CONFIG_RESP_VALUE_MAX),
        Some(MAX_PDU)
    );
    assert_eq!(fits(Opcode::ConfigResp, 4, CONFIG_RESP_VALUE_MAX + 1), None);
}

#[test]
fn node_class_answers_ok_and_refuses_an_unstore_read_only() {
    let mut m = walked_pair();
    let before = m.boards[0].flash.bytes.clone();
    assert_eq!(
        resp_status(&unstore(&mut m, 0x01, store::NODE_CLASS.key())),
        CFG_READ_ONLY
    );
    assert!(m.boards[0].flash.bytes == before);
    let r = m.config_read(0x01, store::NODE_CLASS.key());
    assert_eq!(
        (resp_status(&r), resp_value(&r)),
        (CFG_OK, Some(Value::U8(MOTOR)))
    );
}

// ---------------------------------------------------------------------------------------------------
// Guest-address allocation (`specs/l3.md`, "Addressing"): the grant range and its wrap.
// ---------------------------------------------------------------------------------------------------

/// Drive one `NODE_HELLO(kind=CONTROLLER)` request into a lone responder and return the address it
/// granted (`your_addr`, the reply's last payload byte).
fn grant_guest(resp: &mut Responder, flash: &mut TestFlash) -> u8 {
    let hello = Pdu::from_op(
        Opcode::NodeHello,
        NO_ADDRESS,
        NO_ADDRESS,
        &[KIND_CONTROLLER],
    );
    let mut buf = [0u8; MAX_PDU];
    let n = hello.encode(&mut buf).unwrap();
    let (_, emits) = ingest_one(resp, flash, &buf[..n]);
    let e = emits.first().expect("a hello request is answered");
    let reply = Pdu::decode(&e.bytes).expect("the identity reply decodes");
    assert_eq!(reply.payload.len(), 6, "identity reply shape");
    reply.payload[5]
}

/// A lone responder booted at a persisted `addr` (the boot-time restore), for the slice tests.
fn responder_at(addr: u8, flash: &mut TestFlash) -> Responder {
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);
    let mut s = Store::mount(flash).unwrap();
    s.set_value(NODE_ADDRESS.key(), Value::U8(addr)).unwrap();
    resp.restore_addr(&s);
    assert_eq!(resp.addr(), addr);
    resp
}

/// `n` consecutive grants from `resp`.
fn grants(resp: &mut Responder, flash: &mut TestFlash, n: usize) -> StdVec<u8> {
    (0..n).map(|_| grant_guest(resp, flash)).collect()
}

#[test]
fn an_unassigned_node_grants_from_slice_0_and_wraps_there() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);
    let g = grants(&mut resp, &mut flash, 10);
    assert_eq!(
        g,
        [0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x80, 0x81]
    );
}

#[test]
fn a_node_at_0x01_grants_0x88_to_0x8f_and_wraps_there() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = responder_at(0x01, &mut flash);
    let g = grants(&mut resp, &mut flash, 10);
    assert_eq!(
        g,
        [0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x88, 0x89]
    );
}

#[test]
fn the_top_slice_is_seven_addresses_and_never_reaches_broadcast() {
    // 0x0F and 0x1F share their low four bits, so both grant from slice 15, which stops at
    // GUEST_LAST because 0xFF is broadcast.
    for addr in [0x0F, 0x1F] {
        let mut flash = TestFlash::erased(PS);
        let mut resp = responder_at(addr, &mut flash);
        let g = grants(&mut resp, &mut flash, 9);
        assert_eq!(
            g,
            [0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xF8, 0xF9],
            "node {addr:#04x}"
        );
    }
}

#[test]
fn an_assign_moves_the_allocator_to_the_new_slice() {
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);
    assert_eq!(grant_guest(&mut resp, &mut flash), 0x80);

    let assign = Pdu::from_op(Opcode::Assign, 0x80, NO_ADDRESS, &[EGRESS_SELF, 0x02]);
    let mut buf = [0u8; MAX_PDU];
    let n = assign.encode(&mut buf).unwrap();
    let (_, emits) = ingest_one(&mut resp, &mut flash, &buf[..n]);
    assert_eq!(ack_status(&emits), Some((0x02, STATUS_OK)));

    // Slice 2 is 0x90..=0x97. The slot carries over (the next grant is 0x91, not 0x90): the slice
    // is what the address chooses, the position inside it is just the counter.
    let g = grants(&mut resp, &mut flash, 8);
    assert_eq!(g, [0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x90]);
}

#[test]
fn every_grant_from_every_node_address_stays_inside_the_guest_range() {
    // The three ways an unguarded allocator escapes: 0xFF is broadcast, 0x00 is no-address (neither
    // is source-learned, so those sessions limp along on flood delivery), and anything below 0x80 is
    // a BOARD address, which IS learned and can capture a node's route. Two laps of every slice,
    // from every address a node can hold, plus the unassigned 0x00.
    for addr in 0x00..GUEST_FIRST {
        let mut flash = TestFlash::erased(PS);
        let mut resp = if addr == NO_ADDRESS {
            Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001)
        } else {
            responder_at(addr, &mut flash)
        };
        let first = GUEST_FIRST + 8 * (addr & 0x0F);
        for (i, g) in grants(&mut resp, &mut flash, 16).into_iter().enumerate() {
            assert!(
                (GUEST_FIRST..=GUEST_LAST).contains(&g) && is_unicast(g) && g != BROADCAST,
                "node {addr:#04x} grant {i} = {g:#04x} left the guest range"
            );
            assert!(
                (first..first.saturating_add(8)).contains(&g),
                "node {addr:#04x} grant {i} = {g:#04x} left its slice at {first:#04x}"
            );
        }
    }
}

/// A raw controller link on `(board, port)`: the controller end, for a second controller beside
/// the mesh's walking one.
fn raw_guest_link(m: &mut Mesh, board: usize, port: u8) -> Port {
    let w1: Wire = Default::default();
    let w2: Wire = Default::default();
    m.boards[board].ports[port as usize] = Some(Link::new(MockPort {
        tx: w2.clone(),
        rx: w1.clone(),
    }));
    Link::new(MockPort { tx: w1, rx: w2 })
}

/// Encode `pdu` into an owned frame.
fn frame_of(pdu: &Pdu) -> StdVec<u8> {
    let mut buf = [0u8; MAX_PDU];
    let n = pdu.encode(&mut buf).unwrap();
    buf[..n].to_vec()
}

/// Send `frame` from a raw controller link, settle the mesh, and return every frame that came back.
fn raw_exchange(m: &mut Mesh, link: &mut Port, frame: &[u8]) -> StdVec<StdVec<u8>> {
    let mut buf = [0u8; MAX_PDU];
    link.send(frame).unwrap();
    m.settle();
    let mut got = StdVec::new();
    while let Some(f) = link.poll_recv(&mut buf) {
        got.push(f.to_vec());
    }
    got
}

#[test]
fn two_controllers_attached_at_two_nodes_hold_different_guest_addresses() {
    // The bug `specs/l3.md` ("Addressing") names: one counter per node starting at 0x80 granted
    // both controllers 0x80, and source learning then sent each one's replies out whichever port
    // last carried a frame from 0x80. A line W - G - S, walked from W: G (0x02) carries controller
    // A on its port 2 and S (0x03) controller B on its port 1. Neither G nor S has granted before,
    // so the old counter handed both 0x80.
    let mut m = Mesh::new();
    let w = m.add_board(2);
    let g = m.add_board(3);
    let sl = m.add_board(2);
    m.attach_controller(w, 0);
    m.wire(w, 1, g, 0);
    m.wire(g, 1, sl, 0);
    let mut a = raw_guest_link(&mut m, g, 2);
    let mut b = raw_guest_link(&mut m, sl, 1);
    m.run_walk();
    assert_eq!(
        (m.live_addr(w), m.live_addr(g), m.live_addr(sl)),
        (0x01, 0x02, 0x03)
    );
    // The walk's probes reached the raw links unanswered; drop them.
    let mut buf = [0u8; MAX_PDU];
    while a.poll_recv(&mut buf).is_some() {}
    while b.poll_recv(&mut buf).is_some() {}

    let hello = frame_of(&Pdu::from_op(
        Opcode::NodeHello,
        NO_ADDRESS,
        NO_ADDRESS,
        &[KIND_CONTROLLER],
    ));
    let grant = |frames: StdVec<StdVec<u8>>| {
        let f = frames.first().expect("the hello is answered").clone();
        Pdu::decode(&f).unwrap().payload[5]
    };
    let ga = grant(raw_exchange(&mut m, &mut a, &hello));
    let gb = grant(raw_exchange(&mut m, &mut b, &hello));
    assert_eq!((ga, gb), (0x90, 0x98), "slices 2 and 3");

    // Both controllers talk to S at once. A's PROBE_PORTS is answered only after S's probe window,
    // so B's CONFIG_READ reaches S in between. With both guests at one address, S's table would by
    // then route that address to B's port, and A's PORTS would go to B. Each controller must get
    // exactly its own reply. (B's link also carries S's neighbour probe, a NODE_HELLO; that is not
    // a reply and is left out.)
    let probe = frame_of(&Pdu::from_op(Opcode::ProbePorts, ga, 0x03, &[]));
    let class = [store::NODE_CLASS.id(), 0];
    let read = frame_of(&Pdu::from_op(Opcode::ConfigRead, gb, 0x03, &class));
    a.send(&probe).unwrap();
    b.send(&read).unwrap();
    m.settle();
    let replies = |link: &mut Port| {
        let mut buf = [0u8; MAX_PDU];
        let mut got = StdVec::new();
        while let Some(f) = link.poll_recv(&mut buf) {
            let p = Pdu::decode(f).unwrap();
            if p.known() != Some(Opcode::NodeHello) {
                got.push((p.opcode, p.src, p.dst));
            }
        }
        got
    };
    assert_eq!(replies(&mut a), [(Opcode::Ports.to_u8(), 0x03, ga)]);
    assert_eq!(replies(&mut b), [(Opcode::ConfigResp.to_u8(), 0x03, gb)]);
}

// ---------------------------------------------------------------------------------------------------
// `node.class` (`specs/l3.md`, "Node class"): read-only, never stored.
// ---------------------------------------------------------------------------------------------------

#[test]
fn node_class_reads_motor_controller() {
    let mut m = walked_pair();
    for addr in [0x01, 0x02] {
        let r = m.config_read(addr, store::NODE_CLASS.key());
        assert_eq!(resp_status(&r), CFG_OK);
        assert_eq!(resp_value(&r), Some(Value::U8(0x01)));
    }
}

#[test]
fn a_node_class_write_is_refused_read_only_and_touches_no_flash() {
    let mut m = walked_pair();
    let before = m.boards[0].flash.bytes.clone();

    let w = m.config_write(0x01, store::NODE_CLASS.key(), Value::U8(0x02));
    assert_eq!(resp_status(&w), CFG_READ_ONLY);
    assert_eq!(CFG_READ_ONLY, 6);

    // CONFIG_WRITE_MULTI routes through the same refusal.
    let multi = [1, store::NODE_CLASS.id(), 0, Type::U8.tag(), 1, 0x02];
    let r = m
        .controller_send(Opcode::ConfigWriteMulti.to_u8(), 0x01, &multi)
        .expect("a CONFIG_RESP");
    assert_eq!(resp_status(&r), CFG_READ_ONLY);

    assert!(
        m.boards[0].flash.bytes == before,
        "a refused write appended nothing"
    );
    assert!(!m.boards[0].resp.take_config_persisted());
    let r = m.config_read(0x01, store::NODE_CLASS.key());
    assert_eq!(resp_value(&r), Some(Value::U8(0x01)));
}

// ---------------------------------------------------------------------------------------------------
// `PORTS` neighbour_state (`specs/l3.md`, "Tier-1 wire encoding"): bits 0-1 carry the state, bit 7
// the link-group flag. A board never sets bit 7 yet, but a decoder must not misread a reply that does.
// ---------------------------------------------------------------------------------------------------

#[test]
fn the_controller_reads_the_neighbour_state_from_bits_0_and_1_only() {
    const GROUP: u8 = 0x80; // bit 7: member of the node's link group
    let mut c = Controller::new();
    let reply = |op: Opcode, src: u8, payload: &[u8]| {
        let mut buf = [0u8; MAX_PDU];
        let n = Pdu::from_op(op, src, 0x80, payload)
            .encode(&mut buf)
            .unwrap();
        buf[..n].to_vec()
    };
    // Attach to an unassigned gateway, assign it 0x01, and probe it.
    c.next_request().unwrap();
    c.on_reply(&reply(
        Opcode::NodeHello,
        0x00,
        &[0x00, PROTO_VER, 0, 0, 0x10, 0x80],
    ));
    c.next_request().unwrap();
    c.on_reply(&reply(Opcode::AssignAck, 0x01, &[0x01, STATUS_OK]));
    let probe = c.next_request().unwrap();
    assert_eq!(
        Pdu::decode(&probe).unwrap().known(),
        Some(Opcode::ProbePorts)
    );

    // Port 0 names the controller itself, port 1 an unassigned neighbour, port 2 is empty: every
    // state byte carries bit 7.
    let ports = [
        3,
        0,
        PORT_BLE,
        NB_ASSIGNED | GROUP,
        0x80,
        1,
        PORT_UART,
        NB_UNASSIGNED | GROUP,
        0x00,
        2,
        PORT_UART,
        NB_EMPTY | GROUP,
        0x00,
    ];
    c.on_reply(&reply(Opcode::Ports, 0x01, &ports));

    let link = c
        .host_link()
        .expect("port 0 still reads as assigned to the controller");
    assert_eq!((link.node, link.port), (0x01, 0));
    let assign = c.next_request().expect("port 1 still reads as unassigned");
    let p = Pdu::decode(&assign).unwrap();
    assert_eq!(p.known(), Some(Opcode::Assign));
    assert_eq!((p.dst, p.payload[0]), (0x01, 1));
    c.on_reply(&reply(Opcode::AssignAck, 0x02, &[p.payload[1], STATUS_OK]));
    // The new neighbour is probed next, and the empty port queued nothing.
    let next = Pdu::decode(&c.next_request().unwrap()).unwrap().known();
    assert_eq!(next, Some(Opcode::ProbePorts));
}

#[test]
fn a_board_never_sets_the_link_group_bit() {
    // A lone 3-port board at 0x01 probed by 0x80: port 0 finds an assigned neighbour, port 1 an
    // unassigned one, port 2 nothing.
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(3, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);
    let mut s = Store::mount(&mut flash).unwrap();
    s.set_value(NODE_ADDRESS.key(), Value::U8(0x01)).unwrap();
    resp.restore_addr(&s);
    let mut feed = |resp: &mut Responder, port: u8, frame: StdVec<u8>| {
        let mut emits = Emits::new();
        resp.ingest(port, &frame, &mut s, &mut emits);
    };
    feed(
        &mut resp,
        0,
        frame_of(&Pdu::from_op(Opcode::ProbePorts, 0x80, 0x01, &[])),
    );
    let hello_from = |id: u8| {
        let payload = [id, PROTO_VER, 0, 0, 0x10, NO_ADDRESS];
        frame_of(&Pdu::from_op(Opcode::NodeHello, id, 0x01, &payload))
    };
    feed(&mut resp, 0, hello_from(0x05));
    feed(&mut resp, 1, hello_from(NO_ADDRESS));
    let mut emits = Emits::new();
    resp.poll_probe(&mut emits);
    let ports = emits
        .iter()
        .filter_map(|e| Pdu::decode(&e.bytes).ok())
        .find(|p| p.known() == Some(Opcode::Ports))
        .expect("a PORTS reply");
    let states: StdVec<u8> = (0..3).map(|i| ports.payload[1 + i * 4 + 2]).collect();
    assert_eq!(states, [NB_ASSIGNED, NB_UNASSIGNED, NB_EMPTY]);
}

// ---------------------------------------------------------------------------------------------------
// The live tune lane (specs/rider-ui.md section 4, prerequisite P1): the wire codec, and the two
// properties the walk layer itself is responsible for -- that a tune PDU reaches the caller who owns
// the RAM, and that persisting is reported so a shadow can re-read.
// ---------------------------------------------------------------------------------------------------

/// A `TUNE_WRITE` payload: the `CONFIG_WRITE` shape, `I16` value.
fn tune_write_payload(key: Key, value: i16) -> StdVec<u8> {
    let mut p = StdVec::new();
    p.push(key.field_id);
    p.push(key.index);
    p.push(Type::I16.tag());
    p.extend_from_slice(&value.to_le_bytes());
    p
}

#[test]
fn the_tune_codec_reads_the_wire_and_leaves_the_rest_to_the_shadow() {
    let key = Key {
        field_id: 0x71,
        index: 1,
    };
    // A read names a key and asks for nothing else.
    assert_eq!(
        decode_tune(OP_TUNE_READ, &[0x71, 1]),
        Some((key, TuneOp::Read))
    );
    // A well-formed write carries the value through; the wire does not judge its range, and a key
    // the wire has never heard of is still decoded (the shadow owns the allowlist).
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &tune_write_payload(key, -1234)),
        Some((key, TuneOp::Write(-1234)))
    );
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &tune_write_payload(key, 32767)),
        Some((key, TuneOp::Write(32767)))
    );
    let odd = Key {
        field_id: 0xAB,
        index: 9,
    };
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &tune_write_payload(odd, 1)),
        Some((odd, TuneOp::Write(1)))
    );

    // What the wire DOES judge: the type tag and the value's width.
    let mut wrong_type = tune_write_payload(key, 5);
    wrong_type[2] = Type::U32.tag();
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &wrong_type),
        Some((key, TuneOp::Refuse(CFG_TYPE_MISMATCH)))
    );
    let mut bad_tag = tune_write_payload(key, 5);
    bad_tag[2] = 0x7F;
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &bad_tag),
        Some((key, TuneOp::Refuse(CFG_BAD)))
    );
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &[0x71, 1, Type::I16.tag(), 0]),
        Some((key, TuneOp::Refuse(CFG_BAD))),
        "a truncated value is refused, not read as half a number"
    );
    assert_eq!(
        decode_tune(OP_TUNE_WRITE, &[0x71, 1]),
        Some((key, TuneOp::Refuse(CFG_BAD)))
    );

    // Unanswerable or not ours: dropped.
    assert_eq!(decode_tune(OP_TUNE_READ, &[0x71]), None);
    assert_eq!(decode_tune(OP_TUNE_READ, &[]), None);
    assert_eq!(decode_tune(Opcode::ConfigRead.to_u8(), &[0x71, 1]), None);
}

#[test]
fn a_tune_reply_is_an_ordinary_config_resp() {
    // A walked pair, so the reply routes by the learned table exactly as a CONFIG_RESP does, and
    // the assertion is on the FRAME that reaches the controller rather than on a payload builder.
    let mut m = walked_pair();
    let key = Key {
        field_id: 0x72,
        index: 2,
    };

    let mut emits = Emits::new();
    m.boards[0]
        .resp
        .reply_tune(m.ctrl.ctrl.guest_addr(), key, CFG_OK, Some(-2), &mut emits);
    m.send_emits(0, &emits);
    m.settle();
    let frame = m.ctrl.inbox.pop().expect("a CONFIG_RESP");
    assert_eq!(
        Pdu::decode(&frame).unwrap().opcode,
        Opcode::ConfigResp.to_u8()
    );
    assert_eq!(
        Pdu::decode(&frame).unwrap().payload,
        &[0x72, 2, CFG_OK, Type::I16.tag(), 0xFE, 0xFF]
    );
    // Read back by the SAME reader the CONFIG_* lane's replies go through: one parser, both lanes.
    assert_eq!(resp_status(&frame), CFG_OK);
    assert_eq!(resp_value(&frame), Some(Value::I16(-2)));

    let mut emits = Emits::new();
    m.boards[0]
        .resp
        .reply_tune(m.ctrl.ctrl.guest_addr(), key, CFG_BAD, None, &mut emits);
    m.send_emits(0, &emits);
    m.settle();
    let frame = m.ctrl.inbox.pop().expect("a CONFIG_RESP");
    assert_eq!(Pdu::decode(&frame).unwrap().payload, &[0x72, 2, CFG_BAD, 0]);
    assert_eq!(resp_status(&frame), CFG_BAD);
    assert_eq!(resp_value(&frame), None, "no value on a refusal");
}

#[test]
fn a_tune_pdu_is_handed_back_whether_or_not_the_board_is_armed() {
    // The armed-gate property, from the walk layer's side: the tune opcodes are not walk opcodes,
    // so `apply_write`'s refusal never sees them and there is no armed exception to write. An
    // ARMED board hands a TUNE_WRITE straight to the caller that owns the RAM, exactly as a
    // disarmed one does, and no flash path is entered on either.
    let mut flash = TestFlash::erased(PS);
    let mut resp = Responder::new(1, [PORT_UART; MAX_PORTS], 0x10, MOTOR, 0x0001);
    let key = Key {
        field_id: 0x71,
        index: 0,
    };
    let payload = tune_write_payload(key, 9000);

    for armed in [false, true] {
        resp.set_armed(armed);
        for opcode in [OP_TUNE_WRITE, OP_TUNE_READ] {
            let pdu = Pdu::new(opcode, 0x80, NO_ADDRESS, &payload).unwrap();
            let mut buf = [0u8; MAX_PDU];
            let n = pdu.encode(&mut buf).unwrap();
            let (handed, emits) = ingest_one(&mut resp, &mut flash, &buf[..n]);
            let handed = handed.expect("a tune PDU is handed back, never answered here");
            assert_eq!(handed.opcode, opcode);
            assert_eq!(handed.src, 0x80);
            assert_eq!(&handed.payload[..], &payload[..]);
            assert!(emits.is_empty(), "the walk layer replies to nothing here");
            assert!(
                !resp.take_config_persisted(),
                "armed={armed}: a tune PDU must never persist anything"
            );
        }
    }
}

#[test]
fn persisting_is_reported_once_and_only_for_writes_that_landed() {
    let mut m = walked_pair();
    let key = MOTOR_CURRENT_LIMIT.key();

    // The walk itself persisted node_address, which is not a CONFIG write; clear the slate and
    // assert the level starts down.
    m.boards[0].resp.take_config_persisted();
    assert!(!m.boards[0].resp.take_config_persisted());

    // A read persists nothing.
    let _ = m.config_read(0x01, key);
    assert!(!m.boards[0].resp.take_config_persisted());

    // A successful write reports once, then clears.
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::U32(15_000))),
        CFG_OK
    );
    assert!(m.boards[0].resp.take_config_persisted());
    assert!(
        !m.boards[0].resp.take_config_persisted(),
        "the level is taken, not sticky"
    );

    // A REFUSED write reports nothing: an armed board wrote no flash, so a shadow has nothing to
    // re-read.
    m.boards[0].resp.set_armed(true);
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::U32(1))),
        CFG_ARMED
    );
    assert!(!m.boards[0].resp.take_config_persisted());
    m.boards[0].resp.set_armed(false);

    // Nor does a type-mismatched one.
    assert_eq!(
        resp_status(&m.config_write(0x01, key, Value::U8(1))),
        CFG_TYPE_MISMATCH
    );
    assert!(!m.boards[0].resp.take_config_persisted());

    // And the two-hop case reports on the board that actually wrote, not on the relay.
    m.boards[1].resp.take_config_persisted();
    assert_eq!(
        resp_status(&m.config_write(0x02, MOTOR_METHOD.key(), Value::U8(3))),
        CFG_OK
    );
    assert!(m.boards[1].resp.take_config_persisted());
    assert!(
        !m.boards[0].resp.take_config_persisted(),
        "the gateway only relayed"
    );
}

#[test]
fn the_controller_reports_its_grant_only_once_the_hello_is_answered() {
    let mut c = Controller::new();
    let hello = c.next_request().unwrap();
    assert_eq!(
        Pdu::decode(&hello).unwrap().known(),
        Some(Opcode::NodeHello)
    );
    assert_eq!(c.granted(), None);
    // A reply of another kind leaves it unanswered.
    let mut buf = [0u8; MAX_PDU];
    let n = Pdu::from_op(Opcode::AssignAck, 0x01, 0x80, &[0x01, STATUS_OK])
        .encode(&mut buf)
        .unwrap();
    c.on_reply(&buf[..n]);
    assert_eq!(c.granted(), None);
    // The board 0x01 grants 0x88 from its slice.
    let n = Pdu::from_op(
        Opcode::NodeHello,
        0x01,
        0x80,
        &[0x01, PROTO_VER, 0, 0, 0x10, 0x88],
    )
    .encode(&mut buf)
    .unwrap();
    c.on_reply(&buf[..n]);
    assert_eq!(c.granted(), Some(0x88));
    assert_eq!(c.guest_addr(), 0x88);
}

// ---------------------------------------------------------------------------------------------------
// A reply is bound to the request it answers (`specs/l3.md`, "Discovery and address assignment"):
// the two 2026-10-25 renumberings replayed, and each half of the binding rule on its own.
// ---------------------------------------------------------------------------------------------------

/// A PDU from `src` to `dst`, encoded.
fn pdu_bytes(op: Opcode, src: u8, dst: u8, payload: &[u8]) -> StdVec<u8> {
    frame_of(&Pdu::from_op(op, src, dst, payload))
}

/// Whether `frame` is a `PORTS` from `src`.
fn is_ports_from(frame: &[u8], src: u8) -> bool {
    Pdu::decode(frame).is_ok_and(|p| p.known() == Some(Opcode::Ports) && p.src == src)
}

#[test]
fn a_duplicate_ports_from_the_slave_is_not_the_masters_reply_trace_1() {
    // The phone attached through the slave's BLE port. The Hello is answered by 0x02 and
    // Probe(0x02) is retransmitted on a stall; the slave's first PORTS places the master at
    // (0x02, port 1) and Probe(0x01) goes out; then the retransmitted copy reaches the slave and it
    // answers a second PORTS(0x02) while Probe(0x01) is outstanding. Read as the master's port
    // table, the slave's port 1 holding 0x01 looked like a collision and the slave was renumbered
    // to 0x03.
    let mut m = Mesh::new();
    // The slave first in the mesh, so its duplicate reaches the controller before the master's
    // PORTS does.
    let slave = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    let master = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    m.preassign(master, 0x01);
    m.preassign(slave, 0x02);
    m.attach_controller(slave, 2);
    m.wire(slave, 1, master, 1);

    let mut probe_slave: Option<StdVec<u8>> = None;
    let mut replayed = false;
    m.run_walk_hooked(|m, req| {
        let p = Pdu::decode(req).unwrap();
        if p.known() != Some(Opcode::ProbePorts) {
            return;
        }
        match p.dst {
            0x02 if !replayed => probe_slave = Some(req.to_vec()),
            // Once: the stalled copy is delivered a single time.
            0x01 if !replayed => {
                m.ctrl_send_raw(probe_slave.as_ref().expect("probed first"));
                replayed = true;
            }
            _ => {}
        }
    });

    assert!(
        m.ctrl
            .refused
            .iter()
            .any(|(f, out)| is_ports_from(f, 0x02) && *out == Some(Task::Probe { addr: 0x01 })),
        "the duplicate PORTS(0x02) arrived while Probe(0x01) was outstanding and was refused"
    );
    assert!(
        !m.sent_ops().contains(&Some(Opcode::Assign)),
        "no ASSIGN was sent"
    );
    assert_eq!((m.live_addr(slave), m.persisted_addr(slave)), (0x02, 0x02));
    assert_eq!(
        (m.live_addr(master), m.persisted_addr(master)),
        (0x01, 0x01)
    );
    let mut handed = m.ctrl.ctrl.assigned_addrs().to_vec();
    handed.sort_unstable();
    assert_eq!(handed, [0x01, 0x02]);
}

#[test]
fn a_duplicate_ports_from_the_master_is_not_the_slaves_reply_trace_2() {
    // The phone attached at bridge 0x03 (ports 0 BLE, 1 mailbox to the master, 2 console); the
    // slave's own bridge 0x04 sits on the slave's mailbox. Probe(0x01) is retransmitted, and the
    // duplicate PORTS(0x01) arrives while Probe(0x02) is outstanding. Read as the slave's port
    // table it showed two false collisions: ASSIGN(0x02, egress 0, 0x04), which bridge 0x04 took
    // as a re-persist of its own address, and ASSIGN(0x02, egress 1, 0x05), which renumbered the
    // master to 0x05.
    let mut m = Mesh::new();
    let near = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    let master = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    let slave = m.add_board_kinds(3, [PORT_SWD, PORT_UART, PORT_BLE, 0]);
    let far = m.add_node(3, [PORT_BLE, PORT_SWD, PORT_UART, 0], CLASS_BRIDGE);
    m.preassign(near, 0x03);
    m.preassign(master, 0x01);
    m.preassign(slave, 0x02);
    m.preassign(far, 0x04);
    m.attach_controller(near, 0);
    m.wire(near, 1, master, 0);
    m.wire(master, 1, slave, 1);
    m.wire(slave, 0, far, 1);

    let mut probe_master: Option<StdVec<u8>> = None;
    let mut replayed = false;
    m.run_walk_hooked(|m, req| {
        let p = Pdu::decode(req).unwrap();
        if p.known() != Some(Opcode::ProbePorts) {
            return;
        }
        match p.dst {
            0x01 if !replayed => probe_master = Some(req.to_vec()),
            // Once: the stalled copy is delivered a single time.
            0x02 if !replayed => {
                m.ctrl_send_raw(probe_master.as_ref().expect("probed first"));
                replayed = true;
            }
            _ => {}
        }
    });

    assert!(
        m.ctrl
            .refused
            .iter()
            .any(|(f, out)| is_ports_from(f, 0x01) && *out == Some(Task::Probe { addr: 0x02 })),
        "the duplicate PORTS(0x01) arrived while Probe(0x02) was outstanding and was refused"
    );
    assert!(
        !m.sent_ops().contains(&Some(Opcode::Assign)),
        "no ASSIGN was sent"
    );
    for (node, addr) in [(near, 0x03), (master, 0x01), (slave, 0x02), (far, 0x04)] {
        assert_eq!(
            (m.live_addr(node), m.persisted_addr(node)),
            (addr, addr),
            "{addr:#04x}"
        );
    }
    let mut handed = m.ctrl.ctrl.assigned_addrs().to_vec();
    handed.sort_unstable();
    assert_eq!(handed, [0x01, 0x02, 0x03, 0x04]);
    assert_eq!(m.ctrl.ctrl.gateway_addr(), Some(0x03));
}

#[test]
fn a_late_ports_from_a_previous_probe_is_ignored() {
    // The first copy of PORTS(0x01) is held up past its retransmit; the retransmit's answer moves
    // the walk on to assign 0x02 and probe it, and then the held-up copy arrives. It still shows
    // port 1 unassigned: read as 0x02's port table it would queue an ASSIGN behind 0x02.
    let mut c = Controller::new();
    c.next_request().unwrap();
    assert!(c.on_reply(&pdu_bytes(
        Opcode::NodeHello,
        NO_ADDRESS,
        0x80,
        &[NO_ADDRESS, PROTO_VER, 0, 0, 0x10, 0x80],
    )));
    c.next_request().unwrap();
    assert!(c.on_reply(&pdu_bytes(
        Opcode::AssignAck,
        0x01,
        0x80,
        &[0x01, STATUS_OK]
    )));
    c.next_request().unwrap(); // Probe(0x01)
    #[rustfmt::skip]
    let gateway_ports = pdu_bytes(Opcode::Ports, 0x01, 0x80, &[
        2,
        0, PORT_BLE, NB_ASSIGNED, 0x80,
        1, PORT_UART, NB_UNASSIGNED, 0x00,
    ]);
    assert!(c.on_reply(&gateway_ports));
    let assign = Pdu::decode(&c.next_request().unwrap())
        .unwrap()
        .payload
        .to_vec();
    assert_eq!(assign, [1, 0x02]);
    assert!(c.on_reply(&pdu_bytes(
        Opcode::AssignAck,
        0x02,
        0x80,
        &[0x02, STATUS_OK]
    )));
    let probe = Pdu::decode(&c.next_request().unwrap()).unwrap().dst;
    assert_eq!(probe, 0x02);

    // The late copy is not Probe(0x02)'s reply: refused, still outstanding, nothing queued.
    assert!(!c.on_reply(&gateway_ports));
    assert_eq!(c.outstanding, Some(Task::Probe { addr: 0x02 }));
    assert!(c.queue.is_empty(), "no ASSIGN queued");

    // The real reply completes the walk.
    #[rustfmt::skip]
    let far_ports = pdu_bytes(Opcode::Ports, 0x02, 0x80, &[1, 0, PORT_UART, NB_ASSIGNED, 0x01]);
    assert!(c.on_reply(&far_ports));
    assert!(c.is_complete());
    assert_eq!(&c.assigned_addrs()[..], &[0x01, 0x02]);
}

#[test]
fn an_assign_ack_naming_another_address_is_ignored() {
    let mut c = Controller::new();
    c.next_request().unwrap();
    assert!(c.on_reply(&pdu_bytes(
        Opcode::NodeHello,
        NO_ADDRESS,
        0x80,
        &[NO_ADDRESS, PROTO_VER, 0, 0, 0x10, 0x80],
    )));
    c.next_request().unwrap(); // AssignGateway(0x01)

    // From another address, naming another address, either alone, or to another guest.
    for (src, dst, named) in [
        (0x02, 0x80, 0x02),
        (0x01, 0x80, 0x02),
        (0x02, 0x80, 0x01),
        (0x01, 0x81, 0x01),
    ] {
        let ack = pdu_bytes(Opcode::AssignAck, src, dst, &[named, STATUS_OK]);
        assert!(
            !c.on_reply(&ack),
            "src {src:#04x} dst {dst:#04x} names {named:#04x}"
        );
        assert_eq!(c.outstanding, Some(Task::AssignGateway { new_addr: 0x01 }));
        assert!(c.assigned_addrs().is_empty());
    }
    assert!(c.on_reply(&pdu_bytes(
        Opcode::AssignAck,
        0x01,
        0x80,
        &[0x01, STATUS_OK]
    )));

    // The neighbour arm the same way.
    c.next_request().unwrap(); // Probe(0x01)
    #[rustfmt::skip]
    let ports = pdu_bytes(Opcode::Ports, 0x01, 0x80, &[1, 0, PORT_UART, NB_UNASSIGNED, 0x00]);
    assert!(c.on_reply(&ports));
    c.next_request().unwrap(); // ASSIGN(dst 0x01, egress 0, 0x02)
    assert!(!c.on_reply(&pdu_bytes(
        Opcode::AssignAck,
        0x03,
        0x80,
        &[0x03, STATUS_OK]
    )));
    assert!(!c.on_reply(&pdu_bytes(
        Opcode::AssignAck,
        0x02,
        0x80,
        &[0x03, STATUS_OK]
    )));
    assert_eq!(&c.assigned_addrs()[..], &[0x01]);
    assert!(c.on_reply(&pdu_bytes(
        Opcode::AssignAck,
        0x02,
        0x80,
        &[0x02, STATUS_OK]
    )));
    assert_eq!(&c.assigned_addrs()[..], &[0x01, 0x02]);
}

#[test]
fn a_hello_reply_to_a_stale_guest_address_is_ignored() {
    // The Hello goes out from 0x80; a reply addressed to 0x81 (a grant from an earlier attach) is
    // not its answer, and its grant is not adopted.
    let mut c = Controller::new();
    let hello = Pdu::decode(&c.next_request().unwrap()).unwrap().src;
    assert_eq!(hello, 0x80);
    let stale = pdu_bytes(
        Opcode::NodeHello,
        0x01,
        0x81,
        &[0x01, PROTO_VER, 0, 0, 0x10, 0x89],
    );
    assert!(!c.on_reply(&stale));
    assert_eq!(c.granted(), None);
    assert_eq!(c.guest_addr(), 0x80);
    assert!(c.next_request().is_none(), "the Hello is still outstanding");

    assert!(c.on_reply(&pdu_bytes(
        Opcode::NodeHello,
        0x01,
        0x80,
        &[0x01, PROTO_VER, 0, 0, 0x10, 0x88],
    )));
    assert_eq!(c.granted(), Some(0x88));
}
