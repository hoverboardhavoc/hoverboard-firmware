//! `erase-store` (`specs/hoverboard-bridge.md`, "Erasing the store"): what the erase loses that a
//! `dump` leaves out, printed before the question, and the question's guard. The board is the one
//! behind the bridge; `main.rs` finds it with the walk, reads it, asks, sends `STORE_ERASE` and
//! waits for the mailbox.

use net::walk::CFG_OK;
use store::Value;

use crate::config::{board_owned_keys, shown, Reads};
use crate::install::Guard;
use crate::names::path_of;

/// How long the board may take after the erase to boot and write its mailbox header again.
pub const MAILBOX_WAIT: std::time::Duration = std::time::Duration::from_secs(5);

/// The lines printed before the question: each board-owned key the board answered
/// (`node.address`, each motor's `odometer`), and under each stored odometer the `config` line
/// that writes it back. `name` is how the lines and the `config` line name the board: its
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
                "  write it back with: hoverboard-cli config {name} motors.{}.odometer {n} --force",
                key.index
            ));
        }
    }
    out
}

/// The guard before the erase: asked about, unless `yes`; refused without a `terminal` on stdin
/// unless `yes`, as `replace` and `install-esp` refuse.
pub fn guard(name: &str, yes: bool, terminal: bool) -> Guard {
    if yes {
        Guard::Proceed
    } else if !terminal {
        Guard::Refuse(format!(
            "{name}: erase-store erases every stored setting; give --yes to do it without a terminal"
        ))
    } else {
        Guard::Ask(format!(
            "Take a dump first (hoverboard-cli dump {name} <file>): nothing brings the settings \
             back. Erase every stored setting on {name}?"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nodes::ConfigResp;
    use net::walk::CFG_DEFAULT;
    use store::{Key, Type, MOTOR_ODOMETER, NODE_ADDRESS};

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
                "  write it back with: hoverboard-cli config rover-master motors.0.odometer \
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
                assert!(q.contains("hoverboard-cli dump rover-master"), "{q}");
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
}
