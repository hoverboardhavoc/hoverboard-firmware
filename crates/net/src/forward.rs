//! Source-learned multi-hop forwarding (`specs/l3.md`, "Forwarding"): the same mechanism a transparent
//! Ethernet switch uses for MAC addresses, on a tree of any depth with no route-distribution step.
//!
//! - **Learn from the source.** On receiving any frame on port P, record `src -> P`: "a frame from that
//!   address arrived on P, so that address is reachable out P." It is the **source**, not the
//!   destination, that is learned. **Never learn `src = 0x00`** (several unassigned boards share it) -
//!   only the unicast range `0x01..=0xFE`.
//! - **Forward by the table.** For `dst = Y`: deliver if Y is self (or `Y = 0x00`, "the one peer" on a
//!   point-to-point link); else if `table[Y] = P`, forward out P; else flood every port but the
//!   ingress (split-horizon) and **stop at the first node that already knows Y** (a knowing node
//!   unicasts, so the flood ends there).
//!
//! **No TTL / hop counter.** The L3 PDU is three bytes (`opcode`/`src`/`dst`); it carries no
//! version/seq/ttl. Loop-freedom is **structural**: split-horizon (never echo out the ingress) on a
//! **tree** cannot loop (`specs/l3.md`: "On a tree, flooding cannot loop"). A mis-wired **cycle** is a
//! wiring fault the **walk** detects and reports (a board reached twice / an address that flaps between
//! ports), not something forwarding bounds with a per-frame hop count.
//!
//! The table is **soft state, never flashed**: it churns on every learned frame, is fully re-learnable
//! on boot, and a stale persisted route would be a liability after a re-wire.

use crate::pdu::{self, Pdu, BROADCAST, NO_ADDRESS};

/// The `port_toward[address]` sentinel for "direction unknown". No board has 255 ports, so `0xFF` can
/// never collide with a real port index.
pub const NO_PORT: u8 = 0xFF;

/// The dense `address -> port` routing table: one byte per address (`0xFF` = unknown), 256 entries to
/// cover boards (`0x01..=0x7F`) and guests (`0x80..=0xFE`). O(1) lookup, no eviction logic, pure soft
/// state (`specs/l3.md`, "Storing it").
pub struct RoutingTable {
    port_toward: [u8; 256],
}

impl Default for RoutingTable {
    fn default() -> Self {
        Self::new()
    }
}

impl RoutingTable {
    /// A fresh table: every address unknown.
    pub const fn new() -> Self {
        RoutingTable {
            port_toward: [NO_PORT; 256],
        }
    }

    /// Learn `src -> ingress`. A no-op for `src = 0x00` (no-address, shared by many boards) and for
    /// broadcast - only the routable unicast range `0x01..=0xFE` is learned.
    pub fn learn(&mut self, src: u8, ingress: u8) {
        if pdu::is_unicast(src) {
            self.port_toward[src as usize] = ingress;
        }
    }

    /// The learned egress port toward `dst`, or `None` if not yet learned.
    pub fn port_for(&self, dst: u8) -> Option<u8> {
        let p = self.port_toward[dst as usize];
        if p == NO_PORT {
            None
        } else {
            Some(p)
        }
    }

    /// Clear all routes (a simulated reboot: the table is soft state and re-learns from traffic).
    pub fn clear(&mut self) {
        self.port_toward = [NO_PORT; 256];
    }
}

/// One node's L3 forwarding engine: its own address, its port count, and the routing table. HAL-free
/// and link-agnostic - it consumes whole (L2-reassembled) PDUs and emits whole PDUs by port index; the
/// firmware does the L2 `send`/`poll_recv` plumbing around it.
pub struct Forwarder {
    addr: u8,
    n_ports: u8,
    table: RoutingTable,
}

impl Forwarder {
    /// A node with address `addr` (`0x00` until assigned) and `n_ports` local ports.
    pub fn new(addr: u8, n_ports: u8) -> Self {
        Forwarder {
            addr,
            n_ports,
            table: RoutingTable::new(),
        }
    }

    /// This node's L3 address.
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// The number of local ports.
    pub fn n_ports(&self) -> u8 {
        self.n_ports
    }

    /// Set this node's address (the walk's `ASSIGN`, slice 4, persists it and sets it here).
    pub fn set_addr(&mut self, addr: u8) {
        self.addr = addr;
    }

    /// The learned egress port toward `dst` (table inspection, e.g. a route-matches-wiring assertion).
    pub fn port_toward(&self, dst: u8) -> Option<u8> {
        self.table.port_for(dst)
    }

    /// Clear the routing table (simulated reboot). Addresses persist in flash; the table re-learns.
    pub fn clear_routes(&mut self) {
        self.table.clear();
    }

    /// Process a PDU received on `ingress`: learn its `src`, then deliver-to-self or forward. `deliver`
    /// is called when the PDU is for this node (`dst` is self, or `0x00` = the one peer, or broadcast);
    /// `forward(port, pdu)` is called once per egress port (a unicast forward, or once per flooded
    /// port). The forwarded PDU is unchanged (`src`/`dst`/`payload` preserved).
    pub fn ingest(
        &mut self,
        ingress: u8,
        pdu: &Pdu,
        deliver: &mut impl FnMut(&Pdu),
        forward: &mut impl FnMut(u8, &Pdu),
    ) {
        // Learn first (the one mutation), then make the read-only routing decision.
        self.table.learn(pdu.src, ingress);
        self.decide(Some(ingress), pdu, deliver, forward);
    }

    /// Originate a PDU from this node (no ingress port, so nothing is learned and split-horizon
    /// excludes nothing): unicast out the learned port toward `dst`, or flood every port if unknown.
    /// Read-only (a node does not learn from its own emission).
    pub fn originate(&self, pdu: &Pdu, forward: &mut impl FnMut(u8, &Pdu)) {
        self.decide(None, pdu, &mut |_| {}, forward);
    }

    fn decide(
        &self,
        ingress: Option<u8>,
        pdu: &Pdu,
        deliver: &mut impl FnMut(&Pdu),
        forward: &mut impl FnMut(u8, &Pdu),
    ) {
        let dst = pdu.dst;

        // "The one peer" on a point-to-point link: a directed ASSIGN is forwarded with dst rewritten
        // to 0x00, and the single neighbour - the only thing on the link - takes it. This is a
        // *received-frame* rule (an originator does not loopback a 0x00 frame to itself).
        if ingress.is_some() && dst == NO_ADDRESS {
            deliver(pdu);
            return;
        }
        // For this node (only an assigned node matches its own address).
        if dst == self.addr && self.addr != NO_ADDRESS {
            deliver(pdu);
            return;
        }
        // Broadcast: deliver locally (when received from a port) and re-flood, split-horizon.
        if dst == BROADCAST {
            if ingress.is_some() {
                deliver(pdu);
            }
            self.flood(ingress, pdu, forward);
            return;
        }
        // Unicast to another node.
        match self.table.port_for(dst) {
            // Known route, not back the way it came: unicast onward (this is what stops a flood at the
            // first node that knows `dst`).
            Some(p) if Some(p) != ingress => forward(p, pdu),
            // Learned route points back at the ingress: drop (split-horizon; a tree never needs it).
            Some(_) => {}
            // Unknown: flood every port but the ingress.
            None => self.flood(ingress, pdu, forward),
        }
    }

    fn flood(&self, ingress: Option<u8>, pdu: &Pdu, forward: &mut impl FnMut(u8, &Pdu)) {
        for p in 0..self.n_ports {
            if Some(p) != ingress {
                forward(p, pdu);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pdu::Opcode;
    use std::collections::VecDeque;
    use std::vec::Vec;

    // ---- routing-table unit checks ----

    #[test]
    fn learn_skips_no_address_and_broadcast() {
        let mut t = RoutingTable::new();
        t.learn(NO_ADDRESS, 2); // 0x00 is shared by many boards: never learned
        t.learn(BROADCAST, 2); // 0xFF is not a node
        assert_eq!(t.port_for(NO_ADDRESS), None);
        assert_eq!(t.port_for(BROADCAST), None);
        t.learn(0x05, 1);
        assert_eq!(t.port_for(0x05), Some(1));
        t.learn(0x90, 0); // a guest address is learnable
        assert_eq!(t.port_for(0x90), Some(0));
    }

    #[test]
    fn learn_is_last_write_wins_so_a_rewire_is_relearned() {
        let mut t = RoutingTable::new();
        t.learn(0x05, 1);
        t.learn(0x05, 0); // the node moved: re-learn from the next frame
        assert_eq!(t.port_for(0x05), Some(0));
    }

    // ---- a multi-node mock mesh of L2-style links ----
    //
    // Each undirected link is two byte-frame queues (one per direction). A node port owns a (tx, rx)
    // pair; the peer port owns the mirror. This is a datagram mock (each send delivers one whole PDU,
    // as L2 would after reassembly), so the forwarding logic is exercised on whole PDUs.

    type Queue = std::rc::Rc<std::cell::RefCell<VecDeque<Vec<u8>>>>;

    #[derive(Clone)]
    struct OwnedPdu {
        opcode: u8,
        src: u8,
        dst: u8,
        payload: Vec<u8>,
    }
    impl OwnedPdu {
        fn of(p: &Pdu) -> Self {
            OwnedPdu {
                opcode: p.opcode,
                src: p.src,
                dst: p.dst,
                payload: p.payload.to_vec(),
            }
        }
        fn bytes(&self) -> Vec<u8> {
            let pdu = Pdu {
                opcode: self.opcode,
                src: self.src,
                dst: self.dst,
                payload: &self.payload,
            };
            let mut out = std::vec![0u8; pdu::HEADER_LEN + self.payload.len()];
            let n = pdu.encode(&mut out).unwrap();
            out.truncate(n);
            out
        }
    }

    struct Port {
        tx: Queue,
        rx: Queue,
    }

    struct Node {
        fwd: Forwarder,
        ports: Vec<Port>,
        delivered: Vec<OwnedPdu>,
    }

    struct Mesh {
        nodes: Vec<Node>,
    }

    impl Mesh {
        /// Build `n` nodes with the given addresses; each gets `port_count` empty ports.
        fn new(addrs: &[u8], port_count: &[u8]) -> Self {
            let nodes = addrs
                .iter()
                .zip(port_count)
                .map(|(&a, &pc)| Node {
                    fwd: Forwarder::new(a, pc),
                    ports: Vec::new(),
                    delivered: Vec::new(),
                })
                .collect();
            Mesh { nodes }
        }

        /// Wire `(node_a, port_a) <-> (node_b, port_b)` as a point-to-point link.
        fn link(&mut self, na: usize, pa: u8, nb: usize, pb: u8) {
            let a2b: Queue = Default::default();
            let b2a: Queue = Default::default();
            self.set_port(na, pa, a2b.clone(), b2a.clone());
            self.set_port(nb, pb, b2a, a2b);
        }

        fn set_port(&mut self, n: usize, p: u8, tx: Queue, rx: Queue) {
            let ports = &mut self.nodes[n].ports;
            while ports.len() <= p as usize {
                ports.push(Port {
                    tx: Default::default(),
                    rx: Default::default(),
                });
            }
            ports[p as usize] = Port { tx, rx };
        }

        /// Node `n` explicitly emits a PDU out one chosen `port` (the relay's directed forward: e.g.
        /// the walk's `ASSIGN` re-emitted with `dst` rewritten to `0x00`). Bypasses the routing table.
        fn send_out(&mut self, n: usize, port: u8, opcode: Opcode, dst: u8, payload: &[u8]) {
            let src = self.nodes[n].fwd.addr();
            let e = OwnedPdu {
                opcode: opcode.to_u8(),
                src,
                dst,
                payload: payload.to_vec(),
            };
            self.nodes[n].ports[port as usize]
                .tx
                .borrow_mut()
                .push_back(e.bytes());
        }

        /// Node `n` originates a PDU (the controller/source injecting the first frame).
        fn originate(&mut self, n: usize, opcode: Opcode, dst: u8, payload: &[u8]) {
            let src = self.nodes[n].fwd.addr();
            let pdu = Pdu::from_op(opcode, src, dst, payload);
            let mut emit: Vec<(u8, OwnedPdu)> = Vec::new();
            self.nodes[n]
                .fwd
                .originate(&pdu, &mut |port, p| emit.push((port, OwnedPdu::of(p))));
            for (port, e) in emit {
                self.nodes[n].ports[port as usize]
                    .tx
                    .borrow_mut()
                    .push_back(e.bytes());
            }
        }

        /// Run the firmware loop to quiescence: drain every port, ingest, route emissions onto the
        /// wires, repeat until no frame moves. A pump cap guards against a non-terminating flood (a
        /// loop on a tree would be a bug; the cap turns it into a test failure, not a hang).
        fn pump(&mut self) {
            for _ in 0..10_000 {
                let mut moved = false;
                for n in 0..self.nodes.len() {
                    for p in 0..self.nodes[n].ports.len() as u8 {
                        loop {
                            let frame = self.nodes[n].ports[p as usize].rx.borrow_mut().pop_front();
                            let Some(frame) = frame else { break };
                            moved = true;
                            let pdu = Pdu::decode(&frame).unwrap();
                            let mut emit: Vec<(u8, OwnedPdu)> = Vec::new();
                            let mut delivered: Option<OwnedPdu> = None;
                            self.nodes[n].fwd.ingest(
                                p,
                                &pdu,
                                &mut |d| delivered = Some(OwnedPdu::of(d)),
                                &mut |port, f| emit.push((port, OwnedPdu::of(f))),
                            );
                            if let Some(d) = delivered {
                                self.nodes[n].delivered.push(d);
                            }
                            for (port, e) in emit {
                                self.nodes[n].ports[port as usize]
                                    .tx
                                    .borrow_mut()
                                    .push_back(e.bytes());
                            }
                        }
                    }
                }
                if !moved {
                    return;
                }
            }
            panic!("mesh did not quiesce: a flood looped (not a tree?)");
        }

        fn delivered(&self, n: usize) -> &[OwnedPdu] {
            &self.nodes[n].delivered
        }
    }

    // A 3-node line: 0 (addr 0x80, controller) -p0-p0- 1 (0x01) -p1-p0- 2 (0x02).
    fn line() -> Mesh {
        let mut m = Mesh::new(&[0x80, 0x01, 0x02], &[1, 2, 1]);
        m.link(0, 0, 1, 0); // node0.port0 <-> node1.port0
        m.link(1, 1, 2, 0); // node1.port1 <-> node2.port0
        m
    }

    #[test]
    fn unknown_dst_floods_reaches_far_node_then_reply_unicasts() {
        let mut m = line();
        // Node 0 -> node 2 (addr 0x02), unknown route: floods. On a line it reaches node 2 exactly once.
        m.originate(0, Opcode::ConfigRead, 0x02, &[0xAA]);
        m.pump();
        assert_eq!(m.delivered(2).len(), 1);
        assert_eq!(m.delivered(2)[0].opcode, Opcode::ConfigRead.to_u8());
        assert_eq!(m.delivered(2)[0].src, 0x80);

        // Every node on the path learned the source's direction.
        assert_eq!(m.nodes[1].fwd.port_toward(0x80), Some(0)); // node1 learned controller out port0
        assert_eq!(m.nodes[2].fwd.port_toward(0x80), Some(0)); // node2 learned controller out port0

        // Node 2 replies to the controller: it already learned 0x80, so this is a UNICAST, not a flood.
        m.originate(2, Opcode::ConfigResp, 0x80, &[0xBB]);
        m.pump();
        assert_eq!(m.delivered(0).len(), 1);
        assert_eq!(m.delivered(0)[0].src, 0x02);

        // The reply taught node 1 and the controller node 2's direction.
        assert_eq!(m.nodes[1].fwd.port_toward(0x02), Some(1));
        assert_eq!(m.nodes[0].fwd.port_toward(0x02), Some(0));
    }

    #[test]
    fn once_learned_a_unicast_is_relayed_without_flooding() {
        let mut m = line();
        // Prime the tables: a round-trip so node 0 learns 0x02 and node 1 learns both.
        m.originate(0, Opcode::ConfigRead, 0x02, &[0x01]);
        m.pump();
        m.originate(2, Opcode::ConfigResp, 0x80, &[0x02]);
        m.pump();

        // Now node 0 -> node 2 again: node 0 knows the route, so it unicasts out port 0; node 1 relays
        // out its learned port 1. Node 2 gets exactly one delivery; node 1 never delivers (not for it).
        let before2 = m.delivered(2).len();
        let before1 = m.delivered(1).len();
        m.originate(0, Opcode::ConfigRead, 0x02, &[0x03]);
        m.pump();
        assert_eq!(m.delivered(2).len(), before2 + 1);
        assert_eq!(m.delivered(1).len(), before1); // relayed, never delivered to the relay
        assert_eq!(m.delivered(2).last().unwrap().payload, &[0x03]);
    }

    #[test]
    fn flood_is_converted_to_unicast_at_the_first_knowing_node() {
        // The headline "stops at the first node that already knows Y" mechanism, made explicit. A line
        //   0 (0x80) -p0-p0- 1 (0x01) -p1-p0- 2 (0x02) -p1-p0- 3 (0x03)
        // where the relay node 2 ALSO has a dead-end leaf  2 -p2-p0- 4 (0x04).
        // Node 2 alone has learned 0x03 (from prior 0x03->0x02 traffic that stopped at node 2). When a
        // flood for 0x03 reaches node 2, it must UNICAST out the known port - not flood - so the leaf
        // (node 4) never sees the frame. The leaf is what makes flood vs unicast observable on a line.
        let mut m = Mesh::new(&[0x80, 0x01, 0x02, 0x03, 0x04], &[1, 2, 3, 1, 1]);
        m.link(0, 0, 1, 0);
        m.link(1, 1, 2, 0);
        m.link(2, 1, 3, 0);
        m.link(2, 2, 4, 0); // the dead-end leaf off the relay

        // Prime ONLY node 2 with 0x03's direction: node 3 talks to node 2; the frame is delivered at
        // node 2 and goes no further, so nodes 0/1/4 never learn 0x03.
        m.originate(3, Opcode::ConfigResp, 0x02, &[0x01]);
        m.pump();
        assert_eq!(m.nodes[2].fwd.port_toward(0x03), Some(1)); // node 2 knows 0x03 out port 1
        assert_eq!(m.nodes[1].fwd.port_toward(0x03), None); // node 1 does NOT
        assert_eq!(m.nodes[0].fwd.port_toward(0x03), None); // controller does NOT
        assert!(m.delivered(4).is_empty()); // the leaf saw nothing from the priming

        // Now the controller floods for 0x03 (it does not know the route). node 1 floods onward; node 2
        // KNOWS 0x03 and converts the flood to a unicast out port 1.
        let node2_before = m.delivered(2).len(); // node 2 already holds the priming frame (dst 0x02)
        m.originate(0, Opcode::ConfigRead, 0x03, &[0xAA]);
        m.pump();

        assert_eq!(m.delivered(3).len(), 1); // reached the destination
        assert_eq!(m.delivered(3)[0].src, 0x80);
        assert_eq!(m.delivered(2).len(), node2_before); // node 2 relayed the 0x03 frame, never delivered it
                                                        // The proof it was a UNICAST at node 2, not a flood: the leaf off node 2 got nothing. A flood
                                                        // would have emitted out port 2 (split-horizon only excludes the ingress port 0) and hit it.
        assert!(m.delivered(4).is_empty());
    }

    #[test]
    fn flood_does_not_loop_on_a_tree() {
        // A branching tree (a hub with two leaves plus a chain) and a flood for a never-heard address.
        // The pump cap would fire (panic) if the flood looped; reaching quiescence proves it does not.
        //   node0 (0x80) p0 - p0 node1 (0x01) p1 - p0 node2 (0x02)
        //                              node1 p2 - p0 node3 (0x03)
        let mut m = Mesh::new(&[0x80, 0x01, 0x02, 0x03], &[1, 3, 1, 1]);
        m.link(0, 0, 1, 0);
        m.link(1, 1, 2, 0);
        m.link(1, 2, 3, 0);

        // Flood for 0x7F, which exists nowhere: it must fan out, hit every leaf once, and stop.
        m.originate(0, Opcode::ConfigRead, 0x7F, &[0xEE]);
        m.pump(); // returns iff the flood terminated (no loop)

        // Nobody delivered it (no such node), and no node saw it twice (split-horizon on a tree).
        for n in 0..4 {
            assert!(m.delivered(n).is_empty());
        }
    }

    #[test]
    fn directed_dst_zero_is_delivered_to_the_one_peer() {
        // The walk's directed-ASSIGN hop: a relay (node 0) emits out its egress port a frame with dst
        // rewritten to 0x00; the one peer (node 1) takes it locally regardless of its own address
        // (here unassigned, 0x00). An originator never loopbacks a 0x00 frame to itself.
        let mut m = Mesh::new(&[0x01, 0x00], &[1, 1]);
        m.link(0, 0, 1, 0);
        m.send_out(0, 0, Opcode::Assign, NO_ADDRESS, &[0x02]);
        m.pump();
        assert_eq!(m.delivered(1).len(), 1);
        assert_eq!(m.delivered(1)[0].dst, NO_ADDRESS);
        // Node 1 still learned the sender (src 0x01) so its reply can route back.
        assert_eq!(m.nodes[1].fwd.port_toward(0x01), Some(0));
    }

    #[test]
    fn table_matches_the_wiring_after_a_full_exchange() {
        let mut m = line();
        m.originate(0, Opcode::ConfigRead, 0x02, &[1]);
        m.pump();
        m.originate(2, Opcode::ConfigResp, 0x80, &[2]);
        m.pump();
        // node1 is the relay: controller out port0, node2 out port1 - exactly the wiring.
        assert_eq!(m.nodes[1].fwd.port_toward(0x80), Some(0));
        assert_eq!(m.nodes[1].fwd.port_toward(0x02), Some(1));
        // leaves point upstream out their single port.
        assert_eq!(m.nodes[2].fwd.port_toward(0x80), Some(0));
        assert_eq!(m.nodes[0].fwd.port_toward(0x02), Some(0));
    }
}
