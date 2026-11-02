//! The boot report in words (`specs/configured-links.md` section 5): what a board's bring-up left
//! absent, and why, one line per entry of its boot status record, as `STATUS_READ` answers it
//! (`specs/machine-settings.md` section 6.1). The record's layout and its reason codes are the
//! firmware's (`motorcontroller/board/src/status.rs`); the field names are the name table's.

use board::status::{decode, Entry, Reason};
use store::Key;

use crate::names::path_of;

/// The function an entry names, as a line starts with it: `links[1]`, `motors[0] gate`, `imu`, or
/// the field's path.
fn function(e: &Entry) -> String {
    let id = e.field;
    let m = e.index;
    match id {
        _ if id == store::LINK_TX.id() => format!("links[{m}]"),
        _ if id == store::MOTOR_GATE_HI_A.id() => format!("motors[{m}] gate"),
        _ if id == store::IMU_SCL_PIN.id() || id == store::IMU_MODEL.id() => "imu".into(),
        _ => path_of(Key {
            field_id: id,
            index: m,
        }),
    }
}

/// Why, in words. The configuration is checked before it is written (`specs/config-read-once.md`
/// section 4), so the record says only that a peripheral did not come up.
fn reason(e: &Entry) -> String {
    match Reason::from_u8(e.reason) {
        Some(Reason::NotUp) if e.detail == 0 && e.field == store::IMU_SCL_PIN.id() => {
            "the IMU did not answer its probe".into()
        }
        Some(Reason::NotUp) if e.field == store::IMU_MODEL.id() => {
            "imu.model names no model the firmware knows".into()
        }
        Some(Reason::NotUp) if e.detail == 0 => "did not come up".into(),
        Some(Reason::NotUp) => format!("did not come up (HAL error {})", e.detail),
        None => format!("reason {} (detail {:#04x})", e.reason, e.detail),
    }
}

/// One line per entry of a `STATUS` reply, each starting with `name`, or `<name>: ok` when there
/// is none. `link_pins(i)` answers link `i`'s stored TX and RX pins as text, which a link's line
/// names.
pub fn lines(
    name: &str,
    value: &[u8],
    link_pins: impl Fn(u8) -> Option<(String, String)>,
) -> Vec<String> {
    let entries: Vec<Entry> = decode(value).collect();
    if entries.is_empty() {
        return vec![format!("{name}: ok")];
    }
    entries
        .iter()
        .map(|e| {
            let mut f = function(e);
            if e.field == store::LINK_TX.id() {
                if let Some((tx, rx)) = link_pins(e.index) {
                    f = format!("{f} ({tx}/{rx})");
                }
            }
            format!("{name}: {f}: {}", reason(e))
        })
        .collect()
}

/// The line `apply` and `replace` end with when a changed field is a pin: what the board's
/// bring-up made of it at the next boot is in the boot report.
pub const PIN_HINT: &str = "check the boot report with \"board status\" after the next boot";

#[cfg(test)]
mod tests {
    use super::*;

    fn line(field: u8, index: u8, detail: u8) -> String {
        let pins = |i: u8| (i == 1).then(|| ("PB6".to_string(), "PB7".to_string()));
        let mut l = lines(
            "rover-left",
            &[field, index, Reason::NotUp as u8, detail],
            pins,
        );
        assert_eq!(l.len(), 1);
        l.remove(0)
    }

    #[test]
    fn an_empty_status_is_ok() {
        assert_eq!(lines("rover-left", &[], |_| None), ["rover-left: ok"]);
    }

    #[test]
    fn each_function_in_words() {
        assert_eq!(
            line(store::LINK_TX.id(), 1, 3),
            "rover-left: links[1] (PB6/PB7): did not come up (HAL error 3)"
        );
        assert_eq!(
            line(store::IMU_SCL_PIN.id(), 0, 0),
            "rover-left: imu: the IMU did not answer its probe"
        );
        assert_eq!(
            line(store::IMU_MODEL.id(), 0, 0),
            "rover-left: imu: imu.model names no model the firmware knows"
        );
        assert_eq!(
            line(store::MOTOR_GATE_HI_A.id(), 0, 0),
            "rover-left: motors[0] gate: did not come up"
        );
        assert_eq!(
            lines("b", &[0x40, 0, 2, 0x41], |_| None),
            ["b: motors[0].hall.a: reason 2 (detail 0x41)"]
        );
    }

    #[test]
    fn several_entries_are_several_lines() {
        let v = [
            store::LINK_TX.id(),
            0,
            Reason::NotUp as u8,
            3,
            store::IMU_SCL_PIN.id(),
            0,
            Reason::NotUp as u8,
            0,
        ];
        assert_eq!(lines("b", &v, |_| None).len(), 2);
    }
}
