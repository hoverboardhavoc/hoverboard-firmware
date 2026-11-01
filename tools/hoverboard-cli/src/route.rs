//! How a `board` command that uses SWD reaches the bridge wired to its board
//! (`specs/board-route.md` sections 1, 3 and 5; `specs/machine-settings.md` section 6.3, the
//! target order and the address skip).
//!
//! - A `uid` prefix goes straight through the attached bridge when the `uid` its SWD target reports
//!   (`TARGET_ID`) starts with it. A halted board answers no walk, but its bridge still reads it.
//! - An address goes straight through the attached bridge when that bridge's `PROBE_PORTS` names
//!   the address on its mailbox, and the attached node answers `IDENTIFY` as a bridge.
//! - Anything else walks, and a walk that finds no such node names the attached bridge's SWD
//!   target. `board erase` never goes by `uid`, and `board halt` is refused over a route through
//!   the board.

use espsvc::TargetId;
use net::pdu::NO_ADDRESS;
use net::walk::CLASS_BRIDGE;

use crate::cli::Noun;
use crate::client::Client;
use crate::nodes::{self, uid_hex, Walked};
use crate::target::{self, Target};
use crate::wire::Wire;

/// What a route is for: `board erase` and `board halt` each add a rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Purpose {
    /// `board erase`: the board's own bridge ([`target::erase_bridge`]), never by `uid`, since the
    /// erase needs the board's address for the reads before it and the address restore after it.
    Erase,
    /// `board halt`: refused over a route through the board ([`target::halt_bridge`]).
    Halt,
    /// Every other SWD command.
    Other,
}

/// How a route reached the bridge.
#[derive(Debug)]
pub enum Via {
    /// The attached bridge's SWD target reports a `uid` that starts with the prefix. The board's
    /// address is not known.
    Uid,
    /// The attached bridge's `PROBE_PORTS` named the address `board` on its mailbox, port `port`;
    /// `bridge` is the attached bridge's address.
    Mailbox {
        /// The board's address.
        board: u8,
        /// The bridge's address.
        bridge: u8,
        /// The bridge's mailbox port.
        port: u8,
    },
    /// The walk found the board, and the bridge wired to it.
    Walk {
        /// The walk.
        walked: Walked,
        /// The board's address.
        board: u8,
        /// The bridge's address.
        bridge: u8,
        /// The bridge's mailbox port.
        port: u8,
    },
}

/// The route an SWD command takes: the tool's guest address, the bridge's address as the tool
/// sends to it ([`NO_ADDRESS`] for the attached node), and how it was found.
#[derive(Debug)]
pub struct Route {
    /// The tool's guest address.
    pub src: u8,
    /// Where the command goes.
    pub dst: u8,
    /// How.
    pub via: Via,
}

/// Whether the attached node answers `IDENTIFY` as a bridge. A motor controller does not answer
/// the ESP commands at all, and its mailbox is an SWD port too, so a `PORTS` reply alone does not
/// tell it from a bridge.
fn attached_bridge<W: Wire>(wire: &mut W, src: u8) -> bool {
    nodes::identify(wire, src, NO_ADDRESS).is_ok_and(|i| i.class == CLASS_BRIDGE)
}

/// The attached bridge's `TARGET_ID`, or `None` when it reads no target.
fn target_id<W: Wire>(wire: &mut W, src: u8) -> Option<TargetId> {
    Client::addressed(wire, src, NO_ADDRESS).target_id().ok()
}

/// Route an SWD command for `purpose` to the board `spec` names, over the link `wire` reaches.
pub fn swd<W: Wire>(wire: &mut W, spec: &str, purpose: Purpose) -> Result<Route, String> {
    let target = target::parse(spec)?;
    let hello = nodes::hello(wire)?;
    let src = hello.guest_addr();
    // The attached bridge's SWD target, once read.
    let mut read = None;
    if let Target::Lookup(prefix) = target {
        if purpose != Purpose::Erase && target::is_uid_prefix(prefix) && attached_bridge(wire, src)
        {
            read = target_id(wire, src);
            if read.is_some_and(|id| target::uid_starts(&id.uid, prefix)) {
                return Ok(Route {
                    src,
                    dst: NO_ADDRESS,
                    via: Via::Uid,
                });
            }
        }
    }
    let started = nodes::ports(wire, hello)?;
    if let Target::Address(board) = target {
        if let Some((bridge, port)) = target::wired_here(&started, board) {
            if attached_bridge(wire, src) {
                return Ok(Route {
                    src,
                    dst: NO_ADDRESS,
                    via: Via::Mailbox {
                        board,
                        bridge,
                        port,
                    },
                });
            }
        }
    }
    let walked = nodes::finish(wire, started)?;
    let board = match target::resolve(spec, &walked.found)
        .and_then(|a| target::at(&walked.found, a).cloned())
    {
        Ok(b) => b,
        Err(e) if target::names_none(spec, &walked.found) => {
            return Err(missed(wire, &walked, read, e))
        }
        Err(e) => return Err(e),
    };
    target::check_class(&board, Noun::Board)?;
    let (found, topo) = (&walked.found, &walked.topology);
    let (bridge, port) = if purpose == Purpose::Erase {
        target::erase_bridge(found, topo, board.addr)?
    } else {
        target::wired_bridge(found, topo, board.addr)?
    };
    if purpose == Purpose::Halt {
        target::halt_bridge(found, topo, board.addr, bridge)?;
    }
    let dst = if topo.gateway == Some(bridge) {
        NO_ADDRESS
    } else {
        bridge
    };
    Ok(Route {
        src: walked.src,
        dst,
        via: Via::Walk {
            board: board.addr,
            walked,
            bridge,
            port,
        },
    })
}

/// The walk's miss `e`, with the SWD target of the bridge this link reaches added when it is a
/// bridge and reads one (`read`, or one `TARGET_ID` now). A halted board answers no walk, but the
/// tool does not guess that the two are the same board.
fn missed<W: Wire>(wire: &mut W, walked: &Walked, read: Option<TargetId>, e: String) -> String {
    let bridge = walked
        .topology
        .gateway
        .and_then(|g| target::at(&walked.found, g).ok())
        .is_some_and(|f| f.class() == Some(CLASS_BRIDGE));
    if !bridge {
        return e;
    }
    match read.or_else(|| target_id(wire, walked.src)) {
        Some(id) => format!(
            "{e}; this bridge's SWD target is {} (halted or not running?): name it by its uid \
             prefix",
            uid_hex(&id.uid)
        ),
        None => e,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use espsvc::{Msg, OP_TARGET_ID_REQ};
    use net::pdu::{Opcode, Pdu};
    use net::walk::{
        Identity, CFG_OK, CLASS_MOTOR, MCU_ESP32C3, MCU_F1X0, NB_ASSIGNED, NB_EMPTY, PORT_BLE,
        PORT_SWD, PORT_UART, PROTO_VER, UID_LEN,
    };
    use std::collections::VecDeque;
    use std::time::Duration;

    /// The tool's guest address, which every attached node grants.
    const GUEST: u8 = 0x88;

    fn uid(hex: &str) -> [u8; UID_LEN] {
        let mut u = [0u8; UID_LEN];
        for (i, b) in u.iter_mut().enumerate() {
            *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap();
        }
        u
    }

    const MASTER: &str = "58224535393631174a353930";
    const SLAVE: &str = "51ce39393333370134313739";

    /// One node: its address, class, name, `IDENTITY` uid, ports (kind and neighbour) and, for a
    /// bridge, the uid its SWD target reports.
    struct Node {
        addr: u8,
        class: u8,
        name: &'static str,
        uid: [u8; UID_LEN],
        ports: Vec<(u8, Option<u8>)>,
        swd: Option<[u8; UID_LEN]>,
    }

    /// The offroad pair as a fake machine: bridge `0x03` on the master's mailbox, bridge `0x04` on
    /// the slave's, the boards linked by UART; the tool attached to the node `attached`, on its
    /// last port. Records every request as its opcode and destination.
    struct Fleet {
        nodes: Vec<Node>,
        attached: u8,
        replies: VecDeque<Vec<u8>>,
        sent: Vec<(u8, u8)>,
    }

    impl Fleet {
        fn offroad(attached: u8) -> Self {
            let tool = |a: u8| if a == attached { Some(GUEST) } else { None };
            let nodes = vec![
                Node {
                    addr: 0x01,
                    class: CLASS_MOTOR,
                    name: "offroad-master",
                    uid: uid(MASTER),
                    ports: vec![
                        (PORT_SWD, Some(0x03)),
                        (PORT_UART, Some(0x02)),
                        (PORT_UART, tool(0x01)),
                    ],
                    swd: None,
                },
                Node {
                    addr: 0x02,
                    class: CLASS_MOTOR,
                    name: "offroad-slave",
                    uid: uid(SLAVE),
                    ports: vec![
                        (PORT_SWD, Some(0x04)),
                        (PORT_UART, Some(0x01)),
                        (PORT_UART, tool(0x02)),
                    ],
                    swd: None,
                },
                Node {
                    addr: 0x03,
                    class: CLASS_BRIDGE,
                    name: "offroad-master-bridge",
                    uid: uid("aabbccddeeff000000000003"),
                    ports: vec![
                        (PORT_BLE, None),
                        (PORT_SWD, Some(0x01)),
                        (PORT_UART, tool(0x03)),
                    ],
                    swd: Some(uid(MASTER)),
                },
                Node {
                    addr: 0x04,
                    class: CLASS_BRIDGE,
                    name: "offroad-slave-bridge",
                    uid: uid("aabbccddeeff000000000004"),
                    ports: vec![
                        (PORT_BLE, None),
                        (PORT_SWD, Some(0x02)),
                        (PORT_UART, tool(0x04)),
                    ],
                    swd: Some(uid(SLAVE)),
                },
            ];
            Fleet {
                nodes,
                attached,
                replies: VecDeque::new(),
                sent: Vec::new(),
            }
        }

        /// The master halted: it answers nothing, and its bridge's mailbox shows no neighbour.
        fn master_halted(mut self) -> Self {
            self.nodes.retain(|n| n.addr != 0x01);
            self.nodes[1].ports[1].1 = None;
            self
        }

        /// The opcodes sent, in order.
        fn ops(&self) -> Vec<u8> {
            self.sent.iter().map(|&(op, _)| op).collect()
        }

        fn count(&self, op: u8) -> usize {
            self.ops().iter().filter(|&&o| o == op).count()
        }

        fn first(&self, op: u8) -> Option<usize> {
            self.ops().iter().position(|&o| o == op)
        }
    }

    fn reply(op: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; net::walk::MAX_PDU];
        let n = Pdu::new(op, src, dst, payload)
            .unwrap()
            .encode(&mut buf)
            .unwrap();
        buf[..n].to_vec()
    }

    impl Wire for Fleet {
        fn send(&mut self, pdu: &[u8]) {
            let p = Pdu::decode(pdu).unwrap();
            self.sent.push((p.opcode, p.dst));
            let to = if p.dst == NO_ADDRESS {
                self.attached
            } else {
                p.dst
            };
            let Some(n) = self.nodes.iter().find(|n| n.addr == to) else {
                return;
            };
            let out = match p.known() {
                Some(Opcode::NodeHello) => {
                    reply(p.opcode, n.addr, p.src, &[n.addr, PROTO_VER, GUEST])
                }
                Some(Opcode::ProbePorts) => {
                    let mut b = vec![n.ports.len() as u8];
                    for (i, &(kind, nb)) in n.ports.iter().enumerate() {
                        let (state, addr) = nb.map_or((NB_EMPTY, 0), |a| (NB_ASSIGNED, a));
                        b.extend_from_slice(&[i as u8, kind, state, addr]);
                    }
                    reply(Opcode::Ports.to_u8(), n.addr, p.src, &b)
                }
                Some(Opcode::Identify) => {
                    let mcu = if n.class == CLASS_BRIDGE {
                        MCU_ESP32C3
                    } else {
                        MCU_F1X0
                    };
                    let id = Identity {
                        class: n.class,
                        mcu,
                        fw_ver: 1,
                        uid: n.uid,
                    };
                    reply(Opcode::Identity.to_u8(), n.addr, p.src, &id.encode())
                }
                Some(Opcode::ConfigRead) => {
                    let mut b = vec![p.payload[0], p.payload[1], CFG_OK, store::Type::Str.tag()];
                    b.extend_from_slice(n.name.as_bytes());
                    reply(Opcode::ConfigResp.to_u8(), n.addr, p.src, &b)
                }
                _ if p.opcode == OP_TARGET_ID_REQ => {
                    // A motor controller drops the ESP commands.
                    let Some(swd) = n.swd else { return };
                    let m = Msg::TargetId(TargetId {
                        uid: swd,
                        ..TargetId::default()
                    });
                    let mut buf = [0u8; espsvc::MAX_PDU];
                    let k = m.encode(n.addr, p.src, &mut buf).unwrap();
                    buf[..k].to_vec()
                }
                _ => return,
            };
            self.replies.push_back(out);
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            self.replies.pop_front()
        }
        fn window(&self) -> usize {
            1
        }
    }

    const HELLO: u8 = 0x01;

    fn op(o: Opcode) -> u8 {
        o.to_u8()
    }

    /// Section 6, test 1: a `uid` prefix attached to the bridge whose SWD target matches goes
    /// straight: first contact, `IDENTIFY`, `TARGET_ID`, and no walk or `PROBE_PORTS`.
    #[test]
    fn a_uid_prefix_the_attached_bridge_reads_goes_straight() {
        assert_eq!(op(Opcode::NodeHello), HELLO);
        for purpose in [Purpose::Other, Purpose::Halt] {
            let mut f = Fleet::offroad(0x03);
            let r = swd(&mut f, "5822", purpose).unwrap();
            assert!(matches!(r.via, Via::Uid), "{:?}", r.via);
            assert_eq!((r.src, r.dst), (GUEST, NO_ADDRESS));
            assert_eq!(
                f.ops(),
                [HELLO, op(Opcode::Identify), OP_TARGET_ID_REQ],
                "{purpose:?}"
            );
            assert_eq!(f.sent[1].1, NO_ADDRESS, "IDENTIFY to the attached node");
        }
        // A halted board is reached the same way: its bridge still reads it.
        let mut f = Fleet::offroad(0x03).master_halted();
        assert!(matches!(
            swd(&mut f, "5822", Purpose::Other).unwrap().via,
            Via::Uid
        ));
    }

    /// Section 6, test 1: attached to a board, no `TARGET_ID` is sent and it walks; a `uid` the
    /// attached bridge does not read walks; an address or a name sends neither first.
    #[test]
    fn anything_else_walks_or_takes_the_mailbox() {
        let probe = op(Opcode::ProbePorts);
        let identify = op(Opcode::Identify);
        // Attached to the master: asked IDENTIFY, which says it is no bridge; then the walk.
        let mut f = Fleet::offroad(0x01);
        let r = swd(&mut f, "5822", Purpose::Other).unwrap();
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
        assert_eq!(f.ops()[..3], [HELLO, identify, probe]);
        assert!(
            matches!(
                r.via,
                Via::Walk {
                    board: 0x01,
                    bridge: 0x03,
                    ..
                }
            ),
            "{:?}",
            r.via
        );
        assert_eq!(r.dst, 0x03);
        // The slave's uid from the master's bridge: TARGET_ID reads the master, so the walk finds
        // the slave's own bridge.
        let mut f = Fleet::offroad(0x03);
        let r = swd(&mut f, "51ce", Purpose::Other).unwrap();
        assert_eq!(f.ops()[..4], [HELLO, identify, OP_TARGET_ID_REQ, probe]);
        assert!(
            matches!(
                r.via,
                Via::Walk {
                    board: 0x02,
                    bridge: 0x04,
                    ..
                }
            ),
            "{:?}",
            r.via
        );
        assert_eq!(r.dst, 0x04);
        // An address on the attached bridge's mailbox: PROBE_PORTS, then IDENTIFY of the attached
        // node (section 5), and no walk.
        let mut f = Fleet::offroad(0x03);
        let r = swd(&mut f, "0x01", Purpose::Other).unwrap();
        assert_eq!(f.ops(), [HELLO, probe, identify]);
        assert!(
            matches!(
                r.via,
                Via::Mailbox {
                    board: 0x01,
                    bridge: 0x03,
                    port: 1
                }
            ),
            "{:?}",
            r.via
        );
        // A name: the walk, with nothing asked before its PROBE_PORTS.
        let mut f = Fleet::offroad(0x03);
        let r = swd(&mut f, "offroad-master", Purpose::Other).unwrap();
        assert_eq!(f.ops()[..2], [HELLO, probe]);
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
        assert!(matches!(r.via, Via::Walk { board: 0x01, .. }));
        // An address on another bridge's mailbox: the walk.
        let mut f = Fleet::offroad(0x03);
        let r = swd(&mut f, "0x02", Purpose::Other).unwrap();
        assert_eq!(f.ops()[..2], [HELLO, probe]);
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
        assert!(matches!(
            r.via,
            Via::Walk {
                board: 0x02,
                bridge: 0x04,
                ..
            }
        ));
    }

    /// Section 6, test 2: `board erase` with a `uid` prefix takes the section 6.3 route.
    #[test]
    fn an_erase_never_goes_by_uid() {
        let mut f = Fleet::offroad(0x03);
        let r = swd(&mut f, "5822", Purpose::Erase).unwrap();
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
        assert_eq!(f.ops()[..2], [HELLO, op(Opcode::ProbePorts)]);
        assert!(matches!(
            r.via,
            Via::Walk {
                board: 0x01,
                bridge: 0x03,
                port: 1,
                ..
            }
        ));
        // Its own-bridge rule still holds.
        let mut f = Fleet::offroad(0x03);
        let e = swd(&mut f, "51ce", Purpose::Erase).unwrap_err();
        assert!(
            e.contains("board erase goes only through the board's own bridge"),
            "{e}"
        );
    }

    /// Section 6, test 3: a walk that does not find an address names the attached bridge's SWD
    /// target, from one `TARGET_ID`; a walk from a board adds nothing.
    #[test]
    fn a_walk_that_misses_names_the_bridges_swd_target() {
        let mut f = Fleet::offroad(0x03).master_halted();
        let e = swd(&mut f, "0x01", Purpose::Other).unwrap_err();
        assert_eq!(
            e,
            format!(
                "the walk found no node 0x01; this bridge's SWD target is {MASTER} (halted or not \
                 running?): name it by its uid prefix"
            )
        );
        assert_eq!(f.count(OP_TARGET_ID_REQ), 1);
        // A name, the same.
        let mut f = Fleet::offroad(0x03).master_halted();
        let e = swd(&mut f, "offroad-master", Purpose::Other).unwrap_err();
        assert!(e.starts_with("no node named offroad-master"), "{e}");
        assert!(
            e.contains(&format!("this bridge's SWD target is {MASTER}")),
            "{e}"
        );
        // Attached to a board: the miss as it is, and no TARGET_ID.
        let mut f = Fleet::offroad(0x01);
        let e = swd(&mut f, "0x05", Purpose::Other).unwrap_err();
        assert_eq!(e, "the walk found no node 0x05");
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
    }

    /// Section 6, test 4: `board halt` over a route through the board is refused naming the bridge
    /// to connect to; on a direct route it goes; resume and reset are never refused.
    #[test]
    fn a_halt_through_the_board_is_refused() {
        let mut f = Fleet::offroad(0x03);
        assert_eq!(
            swd(&mut f, "0x02", Purpose::Halt).unwrap_err(),
            "offroad-slave's bridge is reached through offroad-slave: a halt there cuts its own \
             reply, including an armed refusal; connect to offroad-slave-bridge and halt it there"
        );
        let mut f = Fleet::offroad(0x03);
        assert!(swd(&mut f, "offroad-slave", Purpose::Halt).is_err());
        for (attached, spec) in [(0x03, "0x01"), (0x03, "offroad-master"), (0x04, "0x02")] {
            let mut f = Fleet::offroad(attached);
            assert!(
                swd(&mut f, spec, Purpose::Halt).is_ok(),
                "{spec} from {attached:#04x}"
            );
        }
        // Resume, reset and reset-halt are Purpose::Other: the same route goes.
        let mut f = Fleet::offroad(0x03);
        let r = swd(&mut f, "0x02", Purpose::Other).unwrap();
        assert_eq!(r.dst, 0x04);
    }

    /// Section 6, test 7: the address skip needs a bridge to answer. A board's mailbox is an SWD
    /// port too, so a bridge's address on it walks and is refused as a bridge; so does a node whose
    /// `PORTS` names no SWD port.
    #[test]
    fn the_address_skip_needs_a_bridge() {
        let refused = "0x03 (offroad-master-bridge, aabb...) is a bridge, not a board: use the \
                       bridge commands";
        let mut f = Fleet::offroad(0x01);
        assert_eq!(swd(&mut f, "0x03", Purpose::Other).unwrap_err(), refused);
        assert!(f.count(op(Opcode::ProbePorts)) > 1, "it walked");
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
        let mut f = Fleet::offroad(0x01);
        f.nodes[0].ports[0].0 = PORT_UART;
        assert_eq!(swd(&mut f, "0x03", Purpose::Other).unwrap_err(), refused);
        assert!(f.count(op(Opcode::ProbePorts)) > 1, "it walked");
        assert_eq!(f.first(op(Opcode::Identify)), Some(5), "{:02x?}", f.ops());
    }
}
