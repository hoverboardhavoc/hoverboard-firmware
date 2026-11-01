//! Which node a command acts on (`specs/machine-settings.md` section 6.3, "A target"): an address,
//! else an exact `device.name`, else a prefix of a node's factory unique ID; and, for a board, the
//! bridge wired to it, which carries the commands that use SWD.

use net::walk::{Controller, CLASS_BRIDGE, CLASS_MOTOR, PORT_SWD, UID_LEN};

use crate::cli::Noun;
use crate::nodes::{class_name, uid_hex, Found, Topology};

/// A target as the command line gives it, before the walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target<'a> {
    /// An address in `0x01..=0x7F`: no walk needed.
    Address(u8),
    /// A `device.name` or a `uid` prefix, which only the walk can resolve.
    Lookup(&'a str),
}

/// The fewest and most hex digits a `uid` prefix has.
const PREFIX_DIGITS: std::ops::RangeInclusive<usize> = 4..=2 * UID_LEN;

/// Read `spec` as a target: `0x` and hex, or a decimal, in `0x01..=0x7F` is an address; anything
/// else is looked up after the walk. A decimal outside the range (`5822`) is looked up, since it
/// may be a `uid` prefix; `0x` with hex outside it is refused.
pub fn parse(spec: &str) -> Result<Target<'_>, String> {
    let board = |v: u32| u8::try_from(v).ok().filter(|&a| net::pdu::is_board(a));
    if let Some(h) = spec.strip_prefix("0x").or_else(|| spec.strip_prefix("0X")) {
        return u32::from_str_radix(h, 16)
            .ok()
            .and_then(board)
            .map(Target::Address)
            .ok_or_else(|| format!("not a node address (0x01..0x7f): {spec}"));
    }
    if !spec.is_empty() && spec.bytes().all(|b| b.is_ascii_digit()) {
        if let Some(a) = spec.parse::<u32>().ok().and_then(board) {
            return Ok(Target::Address(a));
        }
    }
    Ok(Target::Lookup(spec))
}

/// Whether `s` has the form of a `uid` prefix: 4 to 24 hex digits.
pub fn is_uid_prefix(s: &str) -> bool {
    PREFIX_DIGITS.contains(&s.len()) && s.bytes().all(|b| b.is_ascii_hexdigit())
}

/// Whether `uid` is all zero or all `0xFF`, which no client matches by.
fn blank(uid: &[u8; UID_LEN]) -> bool {
    uid.iter().all(|&b| b == 0) || uid.iter().all(|&b| b == 0xFF)
}

/// `f`'s `uid` if a client may match by it (`specs/machine-settings.md` section 6.1): not all
/// zero, not all `0xFF`, and reported by no other node of `found`.
pub fn usable_uid(f: &Found, found: &[Found]) -> Option<[u8; UID_LEN]> {
    let uid = f.identity?.uid;
    let shared = found
        .iter()
        .filter(|o| o.identity.is_some_and(|i| i.uid == uid))
        .count()
        > 1;
    (!blank(&uid) && !shared).then_some(uid)
}

/// Whether `uid` starts with the `uid` prefix `prefix`, in either case; a blank `uid` never does.
pub fn uid_starts(uid: &[u8; UID_LEN], prefix: &str) -> bool {
    !blank(uid) && uid_hex(uid).starts_with(&prefix.to_ascii_lowercase())
}

/// A node for messages: its address, and its name and the start of its `uid` where known.
pub fn label(f: &Found) -> String {
    let uid = f.identity.map(|i| format!("{}...", &uid_hex(&i.uid)[..4]));
    match (&f.name, uid) {
        (Some(n), Some(u)) => format!("{:#04x} ({n}, {u})", f.addr),
        (Some(n), None) => format!("{:#04x} ({n})", f.addr),
        (None, Some(u)) => format!("{:#04x} ({u})", f.addr),
        (None, None) => format!("{:#04x}", f.addr),
    }
}

/// The nodes of `found` whose `device.name` is `s`, and those whose usable `uid` starts with it.
fn matching<'f>(s: &str, found: &'f [Found]) -> (Vec<&'f Found>, Vec<&'f Found>) {
    let named = found
        .iter()
        .filter(|f| f.name.as_deref() == Some(s))
        .collect();
    let prefixed = if is_uid_prefix(s) {
        found
            .iter()
            .filter(|f| usable_uid(f, found).is_some_and(|u| uid_starts(&u, s)))
            .collect()
    } else {
        Vec::new()
    };
    (named, prefixed)
}

/// Whether `spec` names none of `found`: an address no node holds, or a name and a `uid` prefix
/// that match none. A walk that misses its target says so (`specs/board-route.md` section 1).
pub fn names_none(spec: &str, found: &[Found]) -> bool {
    match parse(spec) {
        Ok(Target::Address(a)) => !found.iter().any(|f| f.addr == a),
        Ok(Target::Lookup(s)) => {
            let (named, prefixed) = matching(s, found);
            named.is_empty() && prefixed.is_empty()
        }
        Err(_) => false,
    }
}

/// The node `spec` names among `found`, in the order of section 6.3: an address, then an exact
/// `device.name`, then a `uid` prefix. A name that is also a prefix of another node's `uid` is
/// refused naming both, and so is a spec that matches no node or more than one.
pub fn resolve(spec: &str, found: &[Found]) -> Result<u8, String> {
    let s = match parse(spec)? {
        Target::Address(a) => return Ok(a),
        Target::Lookup(s) => s,
    };
    let (named, prefixed) = matching(s, found);
    let list = |v: &[&Found]| v.iter().map(|f| label(f)).collect::<Vec<_>>().join(", ");
    match (&named[..], &prefixed[..]) {
        ([_, _, ..], _) => Err(format!("more than one node is named {s}: {}", list(&named))),
        ([n], p) => match p.iter().find(|f| f.addr != n.addr) {
            Some(o) => Err(format!(
                "{s} is the name of {} and the start of the uid of {}; give an address",
                label(n),
                label(o)
            )),
            None => Ok(n.addr),
        },
        ([], [p]) => Ok(p.addr),
        ([], [_, _, ..]) => Err(format!(
            "{s} is the start of more than one uid: {}",
            list(&prefixed)
        )),
        ([], []) => {
            let names: Vec<&str> = found.iter().filter_map(|f| f.name.as_deref()).collect();
            let what = if is_uid_prefix(s) {
                format!("no node named {s} and no uid starting {s}")
            } else {
                format!("no node named {s}")
            };
            Err(format!("{what} (nodes: {})", names.join(", ")))
        }
    }
}

/// The node at `addr` among `found`.
pub fn at(found: &[Found], addr: u8) -> Result<&Found, String> {
    found
        .iter()
        .find(|f| f.addr == addr)
        .ok_or_else(|| format!("the walk found no node {addr:#04x}"))
}

/// The bridge a board's SWD commands go through: the bridge whose walked neighbour the board is
/// (a bridge's one walked neighbour is the board on its mailbox, `Controller::neighbours`), and that
/// bridge's mailbox port. A board no bridge is wired to is refused, naming it.
pub fn wired_bridge(found: &[Found], topo: &Topology, board: u8) -> Result<(u8, u8), String> {
    let b = at(found, board)?;
    if b.class() != Some(CLASS_MOTOR) {
        return Err(format!(
            "{} is a {}, not a board",
            label(b),
            class_name(b.class())
        ));
    }
    let bridge = found
        .iter()
        .filter(|f| f.class() == Some(CLASS_BRIDGE))
        .find(|f| topo.neighbours(f.addr).contains(&board))
        .ok_or_else(|| {
            format!(
                "no bridge is wired to {}: its SWD commands go through the bridge on its mailbox",
                label(b)
            )
        })?;
    let port = topo
        .mailbox(bridge.addr)
        .ok_or_else(|| format!("{} reported no mailbox port", label(bridge)))?;
    Ok((bridge.addr, port))
}

/// The bridge an SWD command to the address `board` goes through without a walk
/// (`specs/machine-settings.md` section 6.3): the attached node and its mailbox port, when
/// [`crate::nodes::start`]'s `PROBE_PORTS` reply named `board` on that node's [`PORT_SWD`] port.
/// `None` sends the command the walked way ([`wired_bridge`]). The attached node is the one this
/// link reaches, so the route also passes [`erase_bridge`]'s rule.
pub fn wired_here(started: &Controller, board: u8) -> Option<(u8, u8)> {
    let node = started.gateway_addr()?;
    let port = started.port_of_kind(node, PORT_SWD)?;
    (started.neighbour_on(node, port) == Some(board)).then_some((node, port))
}

/// `board halt`'s rule (`specs/board-route.md` section 3): the halt goes only where the route to
/// `bridge`, the bridge wired to `board`, does not pass through `board`. A bridge's one walked
/// neighbour is the board on its mailbox, so a bridge other than the node this link reaches is
/// reached through that board, which a halt stops before its reply (an armed refusal included)
/// comes back: refused before sending, naming the bridge to connect to. The test is
/// [`erase_bridge`]'s.
pub fn halt_bridge(found: &[Found], topo: &Topology, board: u8, bridge: u8) -> Result<(), String> {
    if topo.gateway == Some(bridge) {
        return Ok(());
    }
    let b = node_name(at(found, board)?);
    Err(format!(
        "{b}'s bridge is reached through {b}: a halt there cuts its own reply, including an \
         armed refusal; connect to {} and halt it there",
        node_name(at(found, bridge)?)
    ))
}

/// The name lines about a node start with: its `device.name`, or its address when it has none.
pub fn node_name(f: &Found) -> String {
    f.name.clone().unwrap_or_else(|| format!("{:#04x}", f.addr))
}

/// The class a `board` or `bridge` command acts on.
pub fn class_of(noun: Noun) -> u8 {
    if noun == Noun::Bridge {
        CLASS_BRIDGE
    } else {
        CLASS_MOTOR
    }
}

/// Refuse `f` unless it is of the class `noun` acts on, naming the noun that takes it.
pub fn check_class(f: &Found, noun: Noun) -> Result<(), String> {
    if f.class() == Some(class_of(noun)) {
        return Ok(());
    }
    let other = match f.class() {
        Some(CLASS_BRIDGE) => ": use the bridge commands",
        Some(CLASS_MOTOR) => ": use the board commands",
        _ => "",
    };
    Err(format!(
        "{} is a {}, not a {}{other}",
        label(f),
        class_name(f.class()),
        noun.word()
    ))
}

/// The bridge `board erase` goes through: the board's own bridge, which has to be the node this
/// link is attached to. Erasing a board reached over another board's link would erase the link's
/// pins, so the address restore could not reach it: refused before anything is erased, naming the
/// bridge to connect to.
pub fn erase_bridge(found: &[Found], topo: &Topology, board: u8) -> Result<(u8, u8), String> {
    let (bridge, port) = wired_bridge(found, topo, board)?;
    if topo.gateway != Some(bridge) {
        return Err(format!(
            "{} is wired to bridge {}, not to the node this link reaches: board erase goes only \
             through the board's own bridge, so connect to {}",
            label(at(found, board)?),
            label(at(found, bridge)?),
            label(at(found, bridge)?)
        ));
    }
    Ok((bridge, port))
}

#[cfg(test)]
mod tests {
    use super::*;
    use net::walk::{Identity, MCU_ESP32C3, MCU_F1X0};
    use std::collections::BTreeMap;

    fn uid(hex: &str) -> [u8; UID_LEN] {
        let mut u = [0u8; UID_LEN];
        for (i, b) in u.iter_mut().enumerate() {
            *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap();
        }
        u
    }

    fn node(addr: u8, class: u8, name: &str, hex: &str) -> Found {
        Found {
            addr,
            identity: Some(Identity {
                class,
                mcu: if class == CLASS_BRIDGE {
                    MCU_ESP32C3
                } else {
                    MCU_F1X0
                },
                fw_ver: 1,
                uid: uid(hex),
            }),
            name: Some(name.into()),
        }
    }

    /// The offroad machine as section 8 names its boards, with a bridge whose name is `cafe` and a
    /// board whose name is another board's `uid` prefix.
    fn found() -> Vec<Found> {
        vec![
            node(
                0x01,
                CLASS_MOTOR,
                "offroad-slave",
                "51ce39393333370134313739",
            ),
            node(
                0x02,
                CLASS_MOTOR,
                "offroad-master",
                "58224535393631174a353930",
            ),
            node(0x03, CLASS_BRIDGE, "cafe", "aabbccddeeff000000000000"),
            node(0x04, CLASS_MOTOR, "51ce", "cafe0102030405060708090a"),
        ]
    }

    #[test]
    fn an_address_comes_first_and_needs_no_walk() {
        assert_eq!(parse("0x02"), Ok(Target::Address(2)));
        assert_eq!(parse("3"), Ok(Target::Address(3)));
        assert_eq!(parse("127"), Ok(Target::Address(0x7F)));
        assert!(parse("0x80").is_err());
        assert!(parse("0x00").is_err());
        // A decimal outside the range may be a uid prefix.
        assert_eq!(parse("5822"), Ok(Target::Lookup("5822")));
        assert_eq!(parse("rover-left"), Ok(Target::Lookup("rover-left")));
        assert_eq!(resolve("0x05", &[]), Ok(5));
    }

    #[test]
    fn a_name_then_a_uid_prefix() {
        let f = found();
        assert_eq!(resolve("offroad-master", &f), Ok(0x02));
        // `5822` is no address and no name: the offroad master's uid starts with it.
        assert_eq!(resolve("5822", &f), Ok(0x02));
        assert_eq!(resolve("58224535393631174A353930", &f), Ok(0x02));
        // `cafe` is the bridge's name and the start of board 0x04's uid: refused, naming both.
        assert_eq!(
            resolve("cafe", &f),
            Err(
                "cafe is the name of 0x03 (cafe, aabb...) and the start of the uid of 0x04 \
                 (51ce, cafe...); give an address"
                    .into()
            )
        );
        // `51ce` is board 0x04's name and the start of board 0x01's uid.
        assert!(resolve("51ce", &f).unwrap_err().contains("give an address"));
        assert_eq!(
            resolve("beef", &f),
            Err(
                "no node named beef and no uid starting beef (nodes: offroad-slave, \
                 offroad-master, cafe, 51ce)"
                    .into()
            )
        );
        assert_eq!(
            resolve("rover-lft", &f),
            Err(
                "no node named rover-lft (nodes: offroad-slave, offroad-master, cafe, 51ce)".into()
            )
        );
    }

    #[test]
    fn a_spec_that_matches_more_than_one_node_is_refused() {
        let mut f = found();
        f.push(node(
            0x05,
            CLASS_MOTOR,
            "offroad-slave",
            "5823000000000000000000ff",
        ));
        assert!(resolve("offroad-slave", &f)
            .unwrap_err()
            .starts_with("more than one node is named offroad-slave"));
        assert!(resolve("582", &f).is_err(), "three digits is no prefix");
        assert!(resolve("5822", &f).is_ok());
        assert!(resolve("582", &f)
            .unwrap_err()
            .starts_with("no node named 582 (nodes"));
        f[4].identity.as_mut().unwrap().uid = uid("58220000000000000000000f");
        assert!(resolve("5822", &f)
            .unwrap_err()
            .starts_with("5822 is the start of more than one uid"));
    }

    #[test]
    fn a_shared_or_blank_uid_never_matches() {
        let mut f = found();
        f.push(node(0x05, CLASS_MOTOR, "twin", "58224535393631174a353930"));
        assert_eq!(usable_uid(&f[1], &f), None, "two nodes report it");
        assert!(resolve("5822", &f).is_err());
        assert_eq!(
            resolve("offroad-master", &f),
            Ok(0x02),
            "a name still matches"
        );
        let zero = node(0x06, CLASS_MOTOR, "z", "000000000000000000000000");
        let ones = node(0x07, CLASS_MOTOR, "o", "ffffffffffffffffffffffff");
        let f = vec![zero.clone(), ones.clone()];
        assert_eq!(usable_uid(&zero, &f), None);
        assert_eq!(usable_uid(&ones, &f), None);
        assert!(resolve("0000", &f).is_err());
        assert!(resolve("ffff", &f).is_err());
    }

    /// Two bridges, each on one board's mailbox; the link is attached to bridge 0x03, the bridge
    /// on board 0x01.
    fn two_bridges() -> (Vec<Found>, Topology) {
        let mut f = found();
        f[3] = node(0x04, CLASS_BRIDGE, "bridge-r", "aabbccddee11000000000000");
        let topo = Topology {
            gateway: Some(0x03),
            neighbours: BTreeMap::from([
                (0x03, vec![0x01]),
                (0x01, vec![0x03, 0x02]),
                (0x02, vec![0x01, 0x04]),
                (0x04, vec![0x02]),
            ]),
            mailbox: BTreeMap::from([(0x03, 0), (0x04, 0)]),
        };
        (f, topo)
    }

    #[test]
    fn a_boards_swd_commands_go_to_the_bridge_wired_to_it() {
        let (f, topo) = two_bridges();
        assert_eq!(wired_bridge(&f, &topo, 0x01), Ok((0x03, 0)));
        assert_eq!(wired_bridge(&f, &topo, 0x02), Ok((0x04, 0)));
        assert!(wired_bridge(&f, &topo, 0x03)
            .unwrap_err()
            .contains("is a bridge, not a board"));
        let mut lone = topo.clone();
        lone.neighbours.insert(0x04, Vec::new());
        assert_eq!(
            wired_bridge(&f, &lone, 0x02),
            Err(
                "no bridge is wired to 0x02 (offroad-master, 5822...): its SWD commands go \
                 through the bridge on its mailbox"
                    .into()
            )
        );
    }

    #[test]
    fn board_erase_through_another_boards_bridge_is_refused() {
        let (f, topo) = two_bridges();
        assert_eq!(erase_bridge(&f, &topo, 0x01), Ok((0x03, 0)));
        let e = erase_bridge(&f, &topo, 0x02).unwrap_err();
        assert!(e.ends_with("so connect to 0x04 (bridge-r, aabb...)"), "{e}");
    }
}
