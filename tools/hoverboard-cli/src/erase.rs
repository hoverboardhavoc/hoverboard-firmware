//! `board erase` (`specs/hoverboard-bridge.md`, "Erasing the store"): what the erase loses that a
//! `dump` leaves out, printed before the question, the question's guard, and the board's old
//! address given back after the erase. The board is the one behind the bridge; `main.rs` finds it
//! with the walk, reads it, asks, sends `STORE_ERASE`, waits for the mailbox and restores the
//! address.

use net::walk::{CFG_OK, STATUS_OK};
use store::{Value, NODE_ADDRESS};

use crate::config::{board_owned_keys, shown, Reads};
use crate::install::Guard;
use crate::names::path_of;
use crate::nodes;
use crate::wire::Wire;

/// How long the board may take after the erase to boot and write its mailbox header again.
pub const MAILBOX_WAIT: std::time::Duration = std::time::Duration::from_secs(5);

/// The lines printed before the question: each board-owned key the board answered
/// (`node.address`, each motor's `odometer`), and under each stored odometer the `board set`
/// line that writes it back. `name` is how the lines and the `board set` line name the board: its
/// `device.name`, which a board file restores, or its address.
pub fn lines(name: &str, reads: &Reads) -> Vec<String> {
    let mut out = Vec::new();
    for key in board_owned_keys() {
        let Some(r) = reads.get(&key) else { continue };
        out.push(format!("{name}: {} = {}", path_of(key), shown(key, r)));
        if key.field_id != store::MOTOR_ODOMETER.id() || r.status != CFG_OK {
            continue;
        }
        if let Some(Value::U32(n)) = r.value() {
            out.push(format!(
                "  write it back with: hoverboard-cli board set {name} motors.{}.odometer {n} --force",
                key.index
            ));
        }
    }
    out
}

/// The guard before the erase: asked about, unless `yes`; refused without a `terminal` on stdin
/// unless `yes`, as `replace` and `bridge install` refuse.
pub fn guard(name: &str, yes: bool, terminal: bool) -> Guard {
    if yes {
        Guard::Proceed
    } else if !terminal {
        Guard::Refuse(format!(
            "{name}: board erase erases every stored setting; give --yes to do it without a terminal"
        ))
    } else {
        Guard::Ask(format!(
            "Take a dump first (hoverboard-cli board dump {name} <file>): nothing brings the settings \
             back. Erase every stored setting on {name}?"
        ))
    }
}

/// The address to give the board back after the erase: its `node.address` as read before it, when
/// the board answered a stored board address (`CFG_OK`); `None` when the read failed or the address
/// was unset.
pub fn old_address(reads: &Reads) -> Option<u8> {
    let r = reads.get(&NODE_ADDRESS.key())?;
    match (r.status, r.value()) {
        (CFG_OK, Some(Value::U8(a))) if net::pdu::is_board(a) => Some(a),
        _ => None,
    }
}

/// Give the erased board its old address back (`old_address` of `reads`): a directed `ASSIGN` from
/// `src` to the `bridge`, out its mailbox `port`, as the walk assigns a frontier node. Without an
/// old address nothing is sent. The line to print, or the error that fails the command, which
/// says the store is erased and how to restore the address by hand.
pub fn restore<W: Wire>(
    wire: &mut W,
    src: u8,
    bridge: u8,
    port: u8,
    name: &str,
    reads: &Reads,
) -> Result<String, String> {
    let Some(n) = old_address(reads) else {
        return Ok(format!(
            "{name}: node.address was not read or was unset; the board is left unassigned"
        ));
    };
    let why = match nodes::assign(wire, src, bridge, port, n) {
        Ok(STATUS_OK) => return Ok(format!("node.address {n} restored")),
        Ok(status) => format!("the board refused node.address {n} (ASSIGN_ACK status {status})"),
        Err(e) => e,
    };
    Err(format!(
        "{name}: the store is erased, but its address was not restored ({why}); restore it with \
         hoverboard-cli board set <board> node.address {n} --force and a reset, <board> being \
         the address hoverboard-cli machine nodes gives the board"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nodes::ConfigResp;
    use net::pdu::{Opcode, Pdu};
    use net::walk::{CFG_DEFAULT, STATUS_ERR};
    use std::collections::VecDeque;
    use std::time::Duration;
    use store::{Key, Type, MOTOR_ODOMETER};

    fn resp(status: u8, ty: Type, bytes: &[u8]) -> ConfigResp {
        ConfigResp {
            status,
            tag: ty.tag(),
            bytes: bytes.to_vec(),
        }
    }

    fn reads() -> Reads {
        let mut r = Reads::new();
        r.insert(NODE_ADDRESS.key(), resp(CFG_OK, Type::U8, &[0x01]));
        r.insert(
            MOTOR_ODOMETER.at(0).key(),
            resp(CFG_OK, Type::U32, &123_456u32.to_le_bytes()),
        );
        r.insert(
            MOTOR_ODOMETER.at(1).key(),
            resp(CFG_DEFAULT, Type::U32, &0u32.to_le_bytes()),
        );
        r
    }

    #[test]
    fn the_lines_name_the_address_and_each_odometer_with_the_line_that_writes_a_stored_one_back() {
        let l = lines("rover-master", &reads());
        let k: Key = NODE_ADDRESS.key();
        let addr = format!("rover-master: node.address = {}", shown(k, &reads()[&k]));
        assert_eq!(
            l,
            [
                addr.as_str(),
                "rover-master: motors[0].odometer = 123456",
                "  write it back with: hoverboard-cli board set rover-master motors.0.odometer \
                 123456 --force",
                "rover-master: motors[1].odometer = 0 (default)",
            ]
        );
    }

    #[test]
    fn the_written_back_line_parses_to_the_odometer_key() {
        let (key, _) = crate::names::parse_path("motors.0.odometer").unwrap();
        assert_eq!(key, MOTOR_ODOMETER.at(0).key());
    }

    #[test]
    fn the_erase_asks_on_a_terminal_and_without_one_needs_yes() {
        match guard("rover-master", false, true) {
            Guard::Ask(q) => {
                assert!(q.contains("hoverboard-cli board dump rover-master"), "{q}");
                assert!(
                    q.ends_with("Erase every stored setting on rover-master?"),
                    "{q}"
                );
            }
            g => panic!("{g:?}"),
        }
        assert!(matches!(
            guard("rover-master", false, false),
            Guard::Refuse(why) if why.contains("give --yes")
        ));
        assert_eq!(guard("rover-master", true, false), Guard::Proceed);
        assert_eq!(guard("rover-master", true, true), Guard::Proceed);
    }

    fn encode(op: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; net::walk::MAX_PDU];
        let n = Pdu::new(op, src, dst, payload)
            .unwrap()
            .encode(&mut buf)
            .unwrap();
        buf[..n].to_vec()
    }

    /// The bridge on the link and the erased board behind it: every packet sent is kept, and an
    /// `ASSIGN` is answered with the board's telemetry and then an `ASSIGN_ACK` of status `ack`
    /// from the address assigned, or with telemetry alone when `ack` is `None`.
    struct Bridge {
        sent: Vec<Vec<u8>>,
        ack: Option<u8>,
        replies: VecDeque<Vec<u8>>,
    }

    impl Bridge {
        fn new(ack: Option<u8>) -> Self {
            Bridge {
                sent: Vec::new(),
                ack,
                replies: VecDeque::new(),
            }
        }
    }

    impl Wire for Bridge {
        fn send(&mut self, pdu: &[u8]) {
            self.sent.push(pdu.to_vec());
            let p = Pdu::decode(pdu).unwrap();
            if p.known() != Some(Opcode::Assign) {
                return;
            }
            let a = p.payload[1];
            self.replies.push_back(encode(
                linkctl::OP_CYCLIC_STATE,
                a,
                net::pdu::BROADCAST,
                &[0; 8],
            ));
            if let Some(status) = self.ack {
                self.replies
                    .push_back(encode(Opcode::AssignAck.to_u8(), a, p.src, &[a, status]));
            }
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            self.replies.pop_front()
        }
        fn window(&self) -> usize {
            1
        }
    }

    #[test]
    fn the_old_address_goes_back_by_a_directed_assign_out_the_bridges_mailbox_port() {
        let mut w = Bridge::new(Some(STATUS_OK));
        let got = restore(&mut w, 0x88, 0x03, 1, "rover-master", &reads());
        assert_eq!(got, Ok("node.address 1 restored".to_string()));
        assert_eq!(w.sent.len(), 1, "one ASSIGN, acknowledged");
        let p = Pdu::decode(&w.sent[0]).unwrap();
        assert_eq!(p.known(), Some(Opcode::Assign));
        // From the tool's guest address to the bridge: egress its mailbox port, the old address.
        assert_eq!((p.src, p.dst, p.payload), (0x88, 0x03, &[1u8, 0x01][..]));
    }

    #[test]
    fn a_board_whose_address_was_not_read_or_was_unset_is_left_unassigned() {
        let mut unread = reads();
        unread.remove(&NODE_ADDRESS.key());
        let mut unset = reads();
        unset.insert(NODE_ADDRESS.key(), resp(CFG_DEFAULT, Type::U8, &[0x00]));
        for r in [unread, unset] {
            let mut w = Bridge::new(Some(STATUS_OK));
            let got = restore(&mut w, 0x88, 0x03, 1, "rover-master", &r);
            assert_eq!(
                got,
                Ok(
                    "rover-master: node.address was not read or was unset; the board is left \
                    unassigned"
                        .to_string()
                )
            );
            assert!(w.sent.is_empty(), "nothing sent");
        }
    }

    #[test]
    fn no_assign_ack_fails_saying_the_store_is_erased_and_how_to_restore_the_address() {
        let hand =
            "restore it with hoverboard-cli board set <board> node.address 1 --force and a reset";
        let mut w = Bridge::new(None);
        let e = restore(&mut w, 0x88, 0x03, 1, "rover-master", &reads()).unwrap_err();
        assert!(e.starts_with("rover-master: the store is erased"), "{e}");
        assert!(e.contains("no ASSIGN_ACK from 0x01"), "{e}");
        assert!(e.contains(hand), "{e}");
        assert_eq!(w.sent.len(), 3, "the ASSIGN is resent before giving up");
        // A refusal fails the same way.
        let mut w = Bridge::new(Some(STATUS_ERR));
        let e = restore(&mut w, 0x88, 0x03, 1, "rover-master", &reads()).unwrap_err();
        assert!(e.contains("ASSIGN_ACK status 1"), "{e}");
        assert!(e.contains(hand), "{e}");
    }
}
