// SPDX-License-Identifier: GPL-3.0-or-later
//! Which of three slots a per-board permutation names: a selector that cannot be out of range.
//!
//! Two hot paths carry a three-element permutation resolved from board configuration: the
//! commutation crate's `DutyOrder` (which of the SVPWM numbers `(base, c1, c2)` drives each timer
//! channel, the one stock phase-ordering degree of freedom) and the IMU's axis roles (which chip
//! axis becomes each body axis). Both used to hold the permutation as `[u8; 3]` and index a
//! three-element array with it, which makes the compiler emit `panic_bounds_check` even where the
//! bytes were validated at construction, because the TYPE still admits a 4. In the 16 kHz period
//! ISR that stub is not a halt: `panic-halt` spins where the panic fires, which leaves the bridge
//! energized at the last duties with `MOE` set and nothing left to take them down
//! (`specs/panic-free.md`).
//!
//! [`Lane`] is that selector as a type: three values, no fourth, so the bound has nothing to
//! prove. Selection goes through [`Lane::pick`], which reads a CONSTANT index, so both call sites
//! share one owner of "turn a selector into an element" rather than each writing its own match.
//! Validation happens once, where a stored or measured byte becomes a `Lane`
//! ([`Lane::from_index`]), never per period.

/// One of three slots: the selector a three-element permutation is built from.
///
/// `A`/`B`/`C` are slots 0/1/2 of whatever triple the consumer holds (SVPWM `(base, c1, c2)`, the
/// IMU's chip axes `(x, y, z)`). The discriminants ARE the indices, so [`Lane::index`] is a cast.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Lane {
    /// Slot 0.
    A = 0,
    /// Slot 1.
    B = 1,
    /// Slot 2.
    C = 2,
}

impl Lane {
    /// The identity permutation `[A, B, C]`: every consumer's "no permutation" default.
    pub const IDENTITY: [Lane; 3] = [Lane::A, Lane::B, Lane::C];

    /// The lane a 0-based index names, or `None` outside `0..=2`. The ONE validation point: a
    /// stored byte or a derived index is checked here, at construction, and every later use is
    /// in-range by type.
    #[inline]
    pub const fn from_index(i: u8) -> Option<Lane> {
        match i {
            0 => Some(Lane::A),
            1 => Some(Lane::B),
            2 => Some(Lane::C),
            _ => None,
        }
    }

    /// This lane as a 0-based index, for the arithmetic that genuinely needs the number (the IMU's
    /// permutation-parity check). Never for indexing: that is [`Lane::pick`].
    #[inline]
    pub const fn index(self) -> u8 {
        self as u8
    }

    /// The element this lane names. Three constant indices, so no bounds check is emitted and no
    /// input can make one fail.
    #[inline]
    pub fn pick<T: Copy>(self, of: &[T; 3]) -> T {
        match self {
            Lane::A => of[0],
            Lane::B => of[1],
            Lane::C => of[2],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Lane;

    #[test]
    fn from_index_accepts_only_the_three_slots() {
        assert_eq!(Lane::from_index(0), Some(Lane::A));
        assert_eq!(Lane::from_index(1), Some(Lane::B));
        assert_eq!(Lane::from_index(2), Some(Lane::C));
        for i in 3..=255u8 {
            assert_eq!(Lane::from_index(i), None, "index {i} must not name a lane");
        }
    }

    #[test]
    fn index_round_trips_and_matches_the_discriminant() {
        for (i, lane) in Lane::IDENTITY.iter().enumerate() {
            assert_eq!(lane.index() as usize, i);
            assert_eq!(Lane::from_index(lane.index()), Some(*lane));
        }
    }

    #[test]
    fn pick_selects_by_slot() {
        let triple = [10u16, 20, 30];
        assert_eq!(Lane::A.pick(&triple), 10);
        assert_eq!(Lane::B.pick(&triple), 20);
        assert_eq!(Lane::C.pick(&triple), 30);
        // The identity permutation reproduces the triple in order.
        let got = Lane::IDENTITY.map(|l| l.pick(&triple));
        assert_eq!(got, triple);
    }

    #[test]
    fn pick_applies_a_permutation() {
        // A transposition of the last two slots.
        let perm = [Lane::A, Lane::C, Lane::B];
        assert_eq!(perm.map(|l| l.pick(&[1i16, 2, 3])), [1, 3, 2]);
    }
}
