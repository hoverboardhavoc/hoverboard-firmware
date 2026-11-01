//! The Ride view's two board slots (`specs/remote-ui.md` 3.1, "Board slots"), ported from the
//! app's `model/RideReadings.kt` (`rideSlots`) and `ui/screens/RideText.kt` (`slotLabel`).

use core::fmt::Write;

use heapless::String;

use super::SIDE_RIGHT;

/// Which side of the screen a slot is on (`RideReadings.kt`, `SlotSide`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotSide {
    /// The left slot.
    Left,
    /// The right slot.
    Right,
}

/// One board slot: the board at `address`, labelled by its side when `by_side`, else by its
/// address (`RideReadings.kt`, `RideSlot`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RideSlot {
    /// The board's L3 address.
    pub address: u8,
    /// Which slot it is drawn in.
    pub side: SlotSide,
    /// Whether it is labelled "L"/"R" rather than by address.
    pub by_side: bool,
}

impl RideSlot {
    /// The slot's label (`RideText.kt`, `slotLabel`, with `boardHex`): "L", "R", or the address
    /// as "0x01".
    pub fn label(&self) -> String<4> {
        let mut out = String::new();
        match (self.by_side, self.side) {
            (true, SlotSide::Left) => {
                let _ = out.push('L');
            }
            (true, SlotSide::Right) => {
                let _ = out.push('R');
            }
            // "0x" and two hex digits: four bytes.
            (false, _) => {
                let _ = write!(out, "0x{:02X}", self.address);
            }
        }
        out
    }
}

/// The two slots; an empty one is `None` and draws nothing (`RideReadings.kt`, `RideSlots`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RideSlots {
    /// The left slot.
    pub left: Option<RideSlot>,
    /// The right slot.
    pub right: Option<RideSlot>,
}

impl RideSlots {
    /// No motor controller: nothing drawn.
    pub const NONE: RideSlots = RideSlots {
        left: None,
        right: None,
    };

    /// The slot the board at `address` is drawn in, or `None` when it is in neither
    /// (`RideReadings.kt`, `RideSlots.of`).
    pub fn of(&self, address: u8) -> Option<RideSlot> {
        [self.left, self.right]
            .into_iter()
            .flatten()
            .find(|s| s.address == address)
    }
}

fn side_of(byte: u8) -> SlotSide {
    if byte == SIDE_RIGHT {
        SlotSide::Right
    } else {
        SlotSide::Left
    }
}

/// The slots for the `motor_controllers` (any order, distinct addresses), each board's stored
/// `control.side` from `side` (`None` while unread; [`SIDE_RIGHT`] is the right wheel and every
/// other byte the left) (`RideReadings.kt`, `rideSlots`; `specs/remote-ui.md` 3.1, "Board slots"):
///
/// - two boards whose sides read one left and one right: by side, labelled "L" and "R";
/// - one board: on the side its `control.side` names and labelled by it, or while that is unread
///   the left slot, labelled with its address;
/// - anything else (a side unread, both sides the same, more than two boards): the two lowest
///   addresses, left then right, labelled with their addresses.
pub fn ride_slots(motor_controllers: &[u8], side: impl Fn(u8) -> Option<u8>) -> RideSlots {
    let Some(&a) = motor_controllers.iter().min() else {
        return RideSlots::NONE;
    };
    let second = motor_controllers.iter().copied().filter(|&x| x > a).min();
    let Some(b) = second else {
        let slot = match side(a).map(side_of) {
            Some(s) => RideSlot {
                address: a,
                side: s,
                by_side: true,
            },
            None => RideSlot {
                address: a,
                side: SlotSide::Left,
                by_side: false,
            },
        };
        return match slot.side {
            SlotSide::Left => RideSlots {
                left: Some(slot),
                right: None,
            },
            SlotSide::Right => RideSlots {
                left: None,
                right: Some(slot),
            },
        };
    };
    let slot = |address, side, by_side| {
        Some(RideSlot {
            address,
            side,
            by_side,
        })
    };
    if motor_controllers.len() == 2 {
        match (side(a).map(side_of), side(b).map(side_of)) {
            (Some(SlotSide::Left), Some(SlotSide::Right)) => {
                return RideSlots {
                    left: slot(a, SlotSide::Left, true),
                    right: slot(b, SlotSide::Right, true),
                }
            }
            (Some(SlotSide::Right), Some(SlotSide::Left)) => {
                return RideSlots {
                    left: slot(b, SlotSide::Left, true),
                    right: slot(a, SlotSide::Right, true),
                }
            }
            _ => {}
        }
    }
    RideSlots {
        left: slot(a, SlotSide::Left, false),
        right: slot(b, SlotSide::Right, false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::readings::SIDE_LEFT;

    fn sides(map: &[(u8, u8)]) -> impl Fn(u8) -> Option<u8> + '_ {
        move |a| map.iter().find(|(x, _)| *x == a).map(|(_, s)| *s)
    }

    fn s(address: u8, side: SlotSide, by_side: bool) -> Option<RideSlot> {
        Some(RideSlot {
            address,
            side,
            by_side,
        })
    }

    /// `RideReadingsTest.kt`, `boards are slotted by side, alone, or by address`.
    #[test]
    fn boards_are_slotted_by_side_alone_or_by_address() {
        assert_eq!(
            ride_slots(
                &[0x01, 0x02],
                sides(&[(0x01, SIDE_RIGHT), (0x02, SIDE_LEFT)])
            ),
            RideSlots {
                left: s(0x02, SlotSide::Left, true),
                right: s(0x01, SlotSide::Right, true)
            }
        );
        let alone = [0x01];
        assert_eq!(
            ride_slots(&alone, sides(&[(0x01, SIDE_RIGHT)])),
            RideSlots {
                left: None,
                right: s(0x01, SlotSide::Right, true)
            }
        );
        assert_eq!(
            ride_slots(&alone, sides(&[(0x01, SIDE_LEFT)])),
            RideSlots {
                left: s(0x01, SlotSide::Left, true),
                right: None
            }
        );
        assert_eq!(
            ride_slots(&alone, |_| None),
            RideSlots {
                left: s(0x01, SlotSide::Left, false),
                right: None
            }
        );
        let by_address = RideSlots {
            left: s(0x01, SlotSide::Left, false),
            right: s(0x02, SlotSide::Right, false),
        };
        assert_eq!(
            ride_slots(
                &[0x02, 0x01],
                sides(&[(0x01, SIDE_LEFT), (0x02, SIDE_LEFT)])
            ),
            by_address,
            "both the same"
        );
        assert_eq!(
            ride_slots(&[0x01, 0x02], sides(&[(0x01, SIDE_RIGHT)])),
            by_address,
            "a side unread"
        );
        assert_eq!(
            ride_slots(
                &[0x03, 0x02, 0x01],
                sides(&[(0x01, SIDE_LEFT), (0x02, SIDE_RIGHT)])
            ),
            by_address,
            "three boards"
        );
        assert_eq!(ride_slots(&[], |_| None), RideSlots::NONE);
    }

    /// `specs/remote-ui.md` 3.1, "Board slots" (labels "L", "R", `boardHex` "0x01") and
    /// `RideSlots.of`; no Kotlin model test covers either. The slots are `RideReadingsTest.kt`'s.
    #[test]
    fn labels_and_lookup() {
        let by_side = ride_slots(
            &[0x01, 0x02],
            sides(&[(0x01, SIDE_RIGHT), (0x02, SIDE_LEFT)]),
        );
        assert_eq!(by_side.left.unwrap().label(), "L");
        assert_eq!(by_side.right.unwrap().label(), "R");
        assert_eq!(by_side.of(0x01), by_side.right);
        assert_eq!(by_side.of(0x05), None);
        let alone = ride_slots(&[0x0A], |_| None);
        assert_eq!(alone.left.unwrap().label(), "0x0A");
    }
}
