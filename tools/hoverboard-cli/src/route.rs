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
//! - `board recover` takes no target: it acts on the SWD header of the bridge the link reaches
//!   (`specs/bridge-nrst.md` section 3, [`recover`]).
//! - Any other SWD command given no target acts on the attached bridge's SWD target, and `board
//!   erase` on the board on its mailbox (`specs/cli-onboarding.md` part B, [`attached`]).

use espsvc::{Status, TargetId};
use net::pdu::NO_ADDRESS;
use net::walk::{Controller, CLASS_BRIDGE, PORT_SWD};
use store::{Value, DEVICE_NAME};

use crate::cli::Noun;
use crate::client::{Client, ClientError};
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
    /// `board update`, `board flash`, `board clear-protection` and `board target-id`: a walk that
    /// misses while the attached bridge reads no SWD target ends with [`RECOVER_HINT`].
    Identify,
    /// Every other SWD command.
    Other,
}

/// How a route reached the bridge.
#[derive(Debug)]
pub enum Via {
    /// The attached bridge's SWD target reports a `uid` that starts with the prefix. The board's
    /// address is not known.
    Uid,
    /// No target was given: the attached bridge's SWD target, as its `TARGET_ID` reports it. The
    /// board's address is not known.
    Attached(TargetId),
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
            return Err(missed(wire, &walked, read, purpose, e))
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

/// Route an SWD command given no target (`specs/cli-onboarding.md` part B): straight through the
/// attached bridge, with no walk. The attached node has to answer `IDENTIFY` as a bridge. `board
/// erase` takes the board on that bridge's mailbox from its `PROBE_PORTS`, since the erase needs the
/// board's address; every other command the bridge's SWD target, from its `TARGET_ID`.
pub fn attached<W: Wire>(wire: &mut W, purpose: Purpose) -> Result<Route, String> {
    let hello = nodes::hello(wire)?;
    let src = hello.guest_addr();
    if !attached_bridge(wire, src) {
        return Err(format!(
            "this link ends at a board ({}): name the board",
            attached_name(wire, src, &hello, "the node")
        ));
    }
    if purpose == Purpose::Erase {
        let started = nodes::ports(wire, hello)?;
        let on_mailbox = started.gateway_addr().and_then(|bridge| {
            let port = started.port_of_kind(bridge, PORT_SWD)?;
            Some((started.neighbour_on(bridge, port)?, bridge, port))
        });
        return match on_mailbox {
            Some((board, bridge, port)) => Ok(Route {
                src,
                dst: NO_ADDRESS,
                via: Via::Mailbox {
                    board,
                    bridge,
                    port,
                },
            }),
            None => Err(format!(
                "no board on {}'s mailbox: is it running this firmware?",
                attached_name(wire, src, &started, "the bridge")
            )),
        };
    }
    match Client::addressed(&mut *wire, src, NO_ADDRESS).target_id() {
        Ok(id) => Ok(Route {
            src,
            dst: NO_ADDRESS,
            via: Via::Attached(id),
        }),
        Err(ClientError::Status(Status::NoTarget)) => Err(with_recover_hint(format!(
            "no board on {}'s SWD header: check its power and the SWCLK, SWDIO and GND leads",
            attached_name(wire, src, &hello, "the bridge")
        ))),
        Err(e) => Err(e.to_string()),
    }
}

/// The attached node's `device.name`; its address, or `none`, when it has none.
fn attached_name<W: Wire>(wire: &mut W, src: u8, hello: &Controller, none: &str) -> String {
    nodes::read(wire, src, NO_ADDRESS, DEVICE_NAME.key())
        .ok()
        .and_then(|r| match r.value() {
            Some(Value::Str(s)) => Some(String::from_utf8_lossy(s).into_owned()),
            _ => None,
        })
        .unwrap_or_else(|| {
            hello
                .gateway_addr()
                .map_or_else(|| none.into(), |a| format!("{a:#04x}"))
        })
}

/// The walk's miss `e`, with the SWD target of the bridge this link reaches added when it is a
/// bridge and reads one (`read`, or one `TARGET_ID` now). A halted board answers no walk, but the
/// tool does not guess that the two are the same board.
fn missed<W: Wire>(
    wire: &mut W,
    walked: &Walked,
    read: Option<TargetId>,
    purpose: Purpose,
    e: String,
) -> String {
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
        None if purpose == Purpose::Identify => with_recover_hint(e),
        None => e,
    }
}

/// The end of a refusal from `board update`, `board flash`, `board clear-protection` and `board
/// target-id` when the bridge reads no SWD target (`specs/bridge-nrst.md` section 3).
pub const RECOVER_HINT: &str = "if the board is powered and wired, its firmware may block SWD: \
                                wire NRST to GPIO5 and run board recover";

/// `board recover` refused over a link that ends at a board.
pub const RECOVER_AT_BOARD: &str =
    "board recover acts on the bridge this link reaches; connect to the board's own bridge";

/// `board recover` when the debug port does not answer with the board held in reset.
pub const RECOVER_NO_ANSWER: &str = "no answer with the board held in reset: check that NRST is \
                                     wired to GPIO5 and the board is powered";

/// `e` with [`RECOVER_HINT`] at its end.
pub fn with_recover_hint(e: String) -> String {
    format!("{e}; {RECOVER_HINT}")
}

/// A bridge's refusal `e` of `board update`, `board flash`, `board clear-protection` or `board
/// target-id` as text: [`RECOVER_HINT`] at its end when the bridge reads no SWD target.
pub fn identify_refusal(e: &ClientError) -> String {
    match e {
        ClientError::Status(Status::NoTarget) | ClientError::Flash(_, Status::NoTarget) => {
            with_recover_hint(e.to_string())
        }
        e => e.to_string(),
    }
}

/// What `board recover` reached: the bridge's `device.name` (its address, or "the bridge", when it
/// has none) and the SWD target it read with the board held in reset.
#[derive(Debug)]
pub struct Recovered {
    /// The bridge.
    pub bridge: String,
    /// The board, halted at its reset vector.
    pub id: TargetId,
}

/// `board recover` (`specs/bridge-nrst.md` section 3): `TARGET_CTRL` `ConnectUnderReset` to the
/// bridge the link reaches, which holds the board on its SWD header in reset, attaches and halts it
/// at its reset vector. Refused when the link ends at a board.
pub fn recover<W: Wire>(wire: &mut W) -> Result<Recovered, String> {
    let hello = nodes::hello(wire)?;
    let src = hello.guest_addr();
    if !attached_bridge(wire, src) {
        return Err(RECOVER_AT_BOARD.into());
    }
    let bridge = attached_name(wire, src, &hello, "the bridge");
    match Client::addressed(wire, src, NO_ADDRESS).connect_under_reset() {
        Ok(id) => Ok(Recovered { bridge, id }),
        Err(ClientError::Status(Status::NoTarget)) => Err(RECOVER_NO_ANSWER.into()),
        Err(e) => Err(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use espsvc::{Ctrl, Msg, Stage, OP_TARGET_CTRL, OP_TARGET_ID_REQ};
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
                _ if p.opcode == OP_TARGET_ID_REQ
                    || (p.opcode == OP_TARGET_CTRL
                        && p.payload == [Ctrl::ConnectUnderReset as u8]) =>
                {
                    // A motor controller drops the ESP commands; a bridge that reads no SWD
                    // target refuses with `NoTarget`.
                    if n.class != CLASS_BRIDGE {
                        return;
                    }
                    let m = match n.swd {
                        Some(swd) => Msg::TargetId(TargetId {
                            uid: swd,
                            ..TargetId::default()
                        }),
                        None => Msg::Result {
                            op: p.opcode,
                            status: Status::NoTarget,
                        },
                    };
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

    /// `specs/cli-onboarding.md` test B2: no target, attached to a bridge whose `TARGET_ID`
    /// answers: first contact, `IDENTIFY`, `TARGET_ID`, and no walk or `PROBE_PORTS`, for every
    /// purpose but the erase. Attached to a board: refused, naming it, with no `TARGET_ID`.
    #[test]
    fn no_target_goes_straight_through_the_attached_bridge() {
        for purpose in [Purpose::Other, Purpose::Halt, Purpose::Identify] {
            let mut f = Fleet::offroad(0x03);
            let r = attached(&mut f, purpose).unwrap();
            match r.via {
                Via::Attached(id) => assert_eq!(id.uid, uid(MASTER)),
                v => panic!("{v:?}"),
            }
            assert_eq!((r.src, r.dst), (GUEST, NO_ADDRESS));
            assert_eq!(
                f.ops(),
                [HELLO, op(Opcode::Identify), OP_TARGET_ID_REQ],
                "{purpose:?}"
            );
            assert!(f.sent.iter().all(|&(_, dst)| dst == NO_ADDRESS));
        }
        // A halted board: its bridge still reads it.
        let mut f = Fleet::offroad(0x03).master_halted();
        assert!(matches!(
            attached(&mut f, Purpose::Other).unwrap().via,
            Via::Attached(_)
        ));
        for purpose in [Purpose::Other, Purpose::Erase] {
            let mut f = Fleet::offroad(0x01);
            assert_eq!(
                attached(&mut f, purpose).unwrap_err(),
                "this link ends at a board (offroad-master): name the board"
            );
            assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
            assert_eq!(f.count(op(Opcode::ProbePorts)), 0);
        }
    }

    /// `specs/cli-onboarding.md` test B3: no target, the bridge reads no SWD target: refused with
    /// the wiring hint and the recover hint, for every purpose but the erase.
    #[test]
    fn no_target_and_no_swd_target_is_refused_with_the_wiring_hint() {
        for purpose in [Purpose::Other, Purpose::Halt, Purpose::Identify] {
            let mut f = master_blocks_swd();
            assert_eq!(
                attached(&mut f, purpose).unwrap_err(),
                format!(
                    "no board on offroad-master-bridge's SWD header: check its power and the \
                     SWCLK, SWDIO and GND leads; {RECOVER_HINT}"
                ),
                "{purpose:?}"
            );
            assert_eq!(f.count(op(Opcode::ProbePorts)), 0);
        }
    }

    /// `specs/cli-onboarding.md` test B4: `board erase` with no target takes the board on the
    /// attached bridge's mailbox from its `PROBE_PORTS`, with no walk and no `TARGET_ID`; an empty
    /// mailbox is refused.
    #[test]
    fn an_erase_with_no_target_takes_the_board_on_the_mailbox() {
        let mut f = Fleet::offroad(0x03);
        let r = attached(&mut f, Purpose::Erase).unwrap();
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
        assert_eq!(
            f.ops(),
            [HELLO, op(Opcode::Identify), op(Opcode::ProbePorts)]
        );
        let mut f = Fleet::offroad(0x03).master_halted();
        assert_eq!(
            attached(&mut f, Purpose::Erase).unwrap_err(),
            "no board on offroad-master-bridge's mailbox: is it running this firmware?"
        );
        assert_eq!(f.count(OP_TARGET_ID_REQ), 0);
    }

    /// The master's firmware blocks SWD: it answers no walk, and its bridge reads no SWD target.
    fn master_blocks_swd() -> Fleet {
        let mut f = Fleet::offroad(0x03).master_halted();
        f.nodes[1].swd = None;
        f
    }

    /// `specs/bridge-nrst.md` section 5: `board recover` over a link that ends at a bridge sends
    /// `ConnectUnderReset` to the attached bridge and returns its name and the uid it read; over a
    /// link that ends at a board it is refused, and nothing is sent to the board.
    #[test]
    fn board_recover_acts_on_the_attached_bridge_only() {
        let mut f = Fleet::offroad(0x03);
        let r = recover(&mut f).unwrap();
        assert_eq!(r.bridge, "offroad-master-bridge");
        assert_eq!(r.id.uid, uid(MASTER));
        assert_eq!(
            f.ops(),
            [
                HELLO,
                op(Opcode::Identify),
                op(Opcode::ConfigRead),
                OP_TARGET_CTRL
            ]
        );
        assert!(f.sent.iter().all(|&(_, dst)| dst == NO_ADDRESS));

        let mut f = Fleet::offroad(0x01);
        assert_eq!(recover(&mut f).unwrap_err(), RECOVER_AT_BOARD);
        assert_eq!(f.count(OP_TARGET_CTRL), 0);

        // The bridge reads nothing even with the board held in reset: the wiring refusal.
        let mut f = master_blocks_swd();
        assert_eq!(recover(&mut f).unwrap_err(), RECOVER_NO_ANSWER);
    }

    /// `specs/bridge-nrst.md` section 5: `board update` and `board target-id` (`Purpose::Identify`)
    /// end a miss with the recover hint when the attached bridge reads no SWD target; the other SWD
    /// commands do not, and neither does a miss while the bridge reads one.
    #[test]
    fn a_miss_with_no_swd_target_ends_with_the_recover_hint() {
        for spec in ["0x01", "offroad-master", "b37b"] {
            let mut f = master_blocks_swd();
            let e = swd(&mut f, spec, Purpose::Identify).unwrap_err();
            assert!(e.ends_with(&format!("; {RECOVER_HINT}")), "{spec}: {e}");
            let mut f = master_blocks_swd();
            let e = swd(&mut f, spec, Purpose::Other).unwrap_err();
            assert!(!e.contains("board recover"), "{spec}: {e}");
        }
        let mut f = Fleet::offroad(0x03).master_halted();
        let e = swd(&mut f, "0x01", Purpose::Identify).unwrap_err();
        assert!(!e.contains("board recover"), "{e}");
        // After the route: a bridge refusal of NoTarget carries the hint, any other does not.
        assert_eq!(
            identify_refusal(&ClientError::Status(Status::NoTarget)),
            format!("no target: SWD did not answer; {RECOVER_HINT}")
        );
        assert!(
            identify_refusal(&ClientError::Flash(Stage::Erasing, Status::NoTarget))
                .ends_with(RECOVER_HINT)
        );
        assert!(!identify_refusal(&ClientError::Status(Status::Armed)).contains("recover"));
    }
}
