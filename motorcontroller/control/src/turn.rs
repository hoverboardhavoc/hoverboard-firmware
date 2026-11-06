//! The heading hold's arm-time bounds (`specs/heading-hold.md` section 6): the seam that turns the
//! stored `store::CONTROL_TURN_CLAMP` row (0x2F) into the `turn_max` and `head_clamp` the
//! differential half runs at (`crate::sf::TurnConfig`). The law itself is in [`crate::sf`]; this
//! is the one place the row's ranges are enforced, read at boot and at every arm, as
//! [`crate::sf::pos_clamp`] is for the common half's clamp.

use base::TORQUE_FULL_SCALE;

/// The `turn_max` seam ceiling, word counts: the torque word's full scale. The steering split
/// ([`crate::wheel_split`]) then clamps the differential again to the balance headroom, so this
/// bounds what the loop may ask for and the split decides what fits.
pub const TURN_MAX_MAX: i16 = TORQUE_FULL_SCALE as i16;

/// The `head_clamp` seam ceiling, edges: [`crate::sf::POS_CLAMP_MAX`]'s 450, so the clamped
/// heading error has the position error's overflow bound (`450 * 20480` fits `i32`), and a held
/// wheel winds at most about eight turns of heading on the rover.
pub const HEAD_CLAMP_MAX: i16 = crate::sf::POS_CLAMP_MAX;

const _: () = assert!(TURN_MAX_MAX as i32 == TORQUE_FULL_SCALE);

/// The validated `(turn_max, head_clamp)` from the stored `CONTROL_TURN_CLAMP` row, each clamped
/// into `0..=` its ceiling. The store validates type only, so a hand-poked value is clamped here
/// rather than refused: this is boot or an arm, and the board still has to run. `turn_max` 0 is no
/// differential; `head_clamp` 0 disables the heading term.
pub const fn clamp(stored: [i16; 2]) -> (i16, i16) {
    (
        bound(stored[0], TURN_MAX_MAX),
        bound(stored[1], HEAD_CLAMP_MAX),
    )
}

const fn bound(v: i16, max: i16) -> i16 {
    if v < 0 {
        0
    } else if v > max {
        max
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_clamp_seam_clamps_each_index_into_zero_to_its_ceiling() {
        assert_eq!((TURN_MAX_MAX, HEAD_CLAMP_MAX), (28500, 450));
        for (stored, want) in [
            ([i16::MIN, i16::MIN], (0, 0)),
            ([-1, -1], (0, 0)),
            ([0, 0], (0, 0)),
            ([12000, 45], (12000, 45)),
            ([28500, 450], (28500, 450)),
            ([28501, 451], (28500, 450)),
            ([i16::MAX, i16::MAX], (28500, 450)),
        ] {
            assert_eq!(clamp(stored), want, "{stored:?}");
        }
    }
}
