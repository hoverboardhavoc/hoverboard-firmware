//! `board.status` in words (`specs/configured-links.md` section 5): what a board's bring-up left
//! absent, and why, one line per entry of its boot status record. The record's layout and its
//! reason codes are the firmware's (`motorcontroller/board/src/status.rs`); the field names are the
//! name table's.

use board::status::{decode, Entry, Reason};
use board::DEAD_TIME_MIN_DTG;
use store::{Key, Value};

use crate::names::{path_of, pin_name};

/// The function an entry names, as a line starts with it: `link[1]`, `motors[0] gate`, `imu`, or
/// the field's path.
fn function(e: &Entry) -> String {
    let id = e.field;
    let m = e.index;
    match id {
        _ if id == store::LINK_TX.id() => format!("link[{m}]"),
        _ if id == store::MOTOR_GATE_HI_A.id() => format!("motors[{m}] gate"),
        _ if id == store::MOTOR_HALL_A.id() => format!("motors[{m}] hall"),
        _ if id == store::MOTOR_PHASE_A.id() => format!("motors[{m}] current sense"),
        _ if [
            store::IMU_SCL_PIN.id(),
            store::IMU_MODEL.id(),
            store::IMU_AXIS_SIGN.id(),
            store::IMU_AXIS_ROLE.id(),
        ]
        .contains(&id) =>
        {
            "imu".into()
        }
        _ => path_of(Key {
            field_id: id,
            index: m,
        }),
    }
}

/// Why, in words.
fn reason(e: &Entry) -> String {
    let link = e.field == store::LINK_TX.id();
    match Reason::from_u8(e.reason) {
        Some(Reason::NotAPin) => format!("stored byte {:#04x} is not a pin", e.detail),
        Some(Reason::Incomplete) => {
            let unset = path_of(Key {
                field_id: e.detail,
                index: e.index,
            });
            format!("incomplete, {unset} is not set")
        }
        Some(Reason::NotRouted) if link => "no USART routes this pair on this chip".into(),
        Some(Reason::NotRouted) if e.field == store::MOTOR_GATE_HI_A.id() => format!(
            "no advanced timer drives these six pins on this chip (first {})",
            pin_name(e.detail)
        ),
        Some(Reason::NotRouted) if e.field == store::IMU_SCL_PIN.id() => {
            "no I2C instance routes this pair on this chip".into()
        }
        Some(Reason::NotRouted) => {
            format!("{} has no ADC channel on this chip", pin_name(e.detail))
        }
        Some(Reason::SwdPin) => format!("{} is an SWD pin", pin_name(e.detail)),
        Some(Reason::DeadTime) => format!(
            "dead time {} is below the floor of {DEAD_TIME_MIN_DTG}",
            e.detail
        ),
        Some(Reason::NotUp) if e.detail == 0 && e.field == store::IMU_SCL_PIN.id() => {
            "the IMU did not answer its probe".into()
        }
        Some(Reason::NotUp) if e.detail == 0 => "did not come up".into(),
        Some(Reason::NotUp) => format!("did not come up (HAL error {})", e.detail),
        Some(Reason::ImuFrame) if e.field == store::IMU_AXIS_ROLE.id() => {
            "the axis roles are not two distinct axes".into()
        }
        Some(Reason::ImuFrame) => format!(
            "the frame is not a rotation (imu.axis_sign from index {})",
            e.detail
        ),
        None => format!("reason {} (detail {:#04x})", e.reason, e.detail),
    }
}

/// One line per entry of a `board.status` value, each starting with `name`, or `<name>: ok` when
/// there is none. `link_pins(i)` answers link `i`'s stored (TX, RX) pins, which a link's line
/// names.
pub fn lines(name: &str, value: &[u8], link_pins: impl Fn(u8) -> Option<(u8, u8)>) -> Vec<String> {
    let entries: Vec<Entry> = decode(value).collect();
    if entries.is_empty() {
        return vec![format!("{name}: ok")];
    }
    entries
        .iter()
        .map(|e| {
            let mut f = function(e);
            if e.field == store::LINK_TX.id() {
                match link_pins(e.index) {
                    Some((tx, rx)) if tx != store::PIN_ABSENT && rx != store::PIN_ABSENT => {
                        f = format!("{f} ({}/{})", pin_name(tx), pin_name(rx));
                    }
                    _ => {}
                }
            }
            format!("{name}: {f}: {}", reason(e))
        })
        .collect()
}

/// The entries of a `board.status` read, or `None` when the answer is not one.
pub fn entries_of(v: Option<Value<'_>>) -> Option<Vec<u8>> {
    match v {
        Some(Value::Bytes(b)) => Some(b.to_vec()),
        _ => None,
    }
}

/// The line `apply` and `replace` end with when a written field is a pin: a pin field is read
/// only at boot, and what the board made of it is in `board.status`.
pub const PIN_HINT: &str =
    "pin fields take effect at the next boot; check with \"status\" after it";

#[cfg(test)]
mod tests {
    use super::*;

    fn line(field: u8, index: u8, reason: Reason, detail: u8) -> String {
        let pins = |i: u8| (i == 1).then_some((0x16, 0x17));
        let mut l = lines("rover-left", &[field, index, reason as u8, detail], pins);
        assert_eq!(l.len(), 1);
        l.remove(0)
    }

    #[test]
    fn an_empty_status_is_ok() {
        assert_eq!(lines("rover-left", &[], |_| None), ["rover-left: ok"]);
    }

    /// The two examples `specs/configured-links.md` section 5 gives, and one line per reason.
    #[test]
    fn each_reason_in_words() {
        let link = store::LINK_TX.id();
        let gate = store::MOTOR_GATE_HI_A.id();
        assert_eq!(
            line(link, 1, Reason::NotRouted, 0x16),
            "rover-left: link[1] (PB6/PB7): no USART routes this pair on this chip"
        );
        assert_eq!(
            line(gate, 0, Reason::DeadTime, 10),
            "rover-left: motors[0] gate: dead time 10 is below the floor of 18"
        );
        assert_eq!(
            line(store::MOTOR_HALL_A.id(), 0, Reason::NotAPin, 0x47),
            "rover-left: motors[0] hall: stored byte 0x47 is not a pin"
        );
        assert_eq!(
            line(link, 0, Reason::Incomplete, store::LINK_RX.id()),
            "rover-left: link[0]: incomplete, link.rx[0] is not set"
        );
        assert_eq!(
            line(store::BOARD_BATTERY_PIN.id(), 0, Reason::NotRouted, 0x1C),
            "rover-left: board.battery.pin: PB12 has no ADC channel on this chip"
        );
        assert_eq!(
            line(store::LED_GREEN.id(), 0, Reason::SwdPin, 0x0D),
            "rover-left: led.green: PA13 is an SWD pin"
        );
        assert_eq!(
            line(link, 1, Reason::NotUp, 10),
            "rover-left: link[1] (PB6/PB7): did not come up (HAL error 10)"
        );
        assert_eq!(
            line(store::IMU_SCL_PIN.id(), 0, Reason::NotUp, 0),
            "rover-left: imu: the IMU did not answer its probe"
        );
        assert_eq!(
            line(store::IMU_AXIS_SIGN.id(), 0, Reason::ImuFrame, 3),
            "rover-left: imu: the frame is not a rotation (imu.axis_sign from index 3)"
        );
        assert_eq!(
            line(gate, 0, Reason::NotRouted, 0x08),
            "rover-left: motors[0] gate: no advanced timer drives these six pins on this chip (first PA8)"
        );
    }

    #[test]
    fn several_entries_are_several_lines() {
        let v = [
            store::LINK_TX.id(),
            0,
            Reason::Incomplete as u8,
            store::LINK_RX.id(),
            store::MOTOR_GATE_HI_A.id(),
            0,
            Reason::DeadTime as u8,
            10,
        ];
        assert_eq!(lines("b", &v, |_| None).len(), 2);
    }
}
