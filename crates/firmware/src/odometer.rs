//! The odometer: this wheel's lifetime travel in absolute hall edges, kept across power cycles
//! (`specs/odometer.md`).
//!
//! The count itself is the commutator's (`commutation::foc::Commutation::edges_total`, published
//! by the period ISR into [`crate::motor::EDGES`]); the store holds the total as of the last
//! write. This module is the third piece: the running total, and the decision of WHEN that total
//! is allowed to reach flash.
//!
//! # Why the decision is a type and not an `if`
//!
//! The store carries no arm state and never will (`specs/storage-layer.md`, "Who enforces writes
//! only while disarmed"), so an internal writer carries the same obligation the L3 `CONFIG_WRITE`
//! path carries through `CFG_ARMED`: the CALLER gates it. Putting the whole verdict in one total
//! function makes that obligation testable over its entire input space rather than reviewable, and
//! the test below does exactly that: [`Odometer::due`] answers `None` whenever the board is armed,
//! for every other input and every state this struct can hold.
//!
//! The second half of the proof is confinement, and it is a test too: `store::MOTOR_ODOMETER` is
//! named in exactly one place in the firmware crate outside this file, and that place is inside
//! the branch `due` opens.
//!
//! # Why a LEVEL and not a disarm edge
//!
//! The service loop samples the arm fact once per pass. A disarm edge can therefore be missed (a
//! fault that arms and shuts down inside one pass is a pair of edges the loop never sees), and a
//! missed edge is a lost ride. The four conditions below are all levels, so whatever ended the
//! ride, a rider releasing the button or a fault running the shutdown sequence, the board ends up
//! disarmed with a wheel that stops and the next pass that sees it writes.

// The service loop is the only caller and it is target-only, so the host build reads this as dead
// code; the tests below are what exercise it there.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

/// The running total and the write decision.
///
/// One instance, in the service loop's frame, for motor 0. Motor 1 gets one when motor 1 gets a
/// bring-up (`crate::motor`, the single-motor rule).
#[derive(Clone, Copy, Debug)]
pub struct Odometer {
    /// The live total: what the store last held plus every edge folded since boot. This is the
    /// number the wire carries and the number a write persists.
    total: u32,
    /// The [`crate::motor::EDGES`] value already folded into [`Self::total`].
    folded: u32,
    /// [`Self::total`] as the store last recorded it.
    written: u32,
    /// Whether the board has been armed since the last write attempt: the "this was a ride"
    /// condition, and the thing that bounds a failing write to one attempt per ride.
    ride: bool,
}

impl Odometer {
    /// Start from the stored total and the edge count the ISR has reached by now.
    ///
    /// `edges_now` is taken rather than assumed zero so that no edge is counted twice if the
    /// motor was brought up before this ran. Nothing is owed the other way: edges turned before
    /// the store was mounted belong to this boot and are folded at the first [`Self::fold`].
    pub const fn new(stored: u32, edges_now: u32) -> Self {
        Odometer {
            total: stored,
            folded: edges_now,
            written: stored,
            ride: false,
        }
    }

    /// The live total.
    pub const fn total(&self) -> u32 {
        self.total
    }

    /// Fold the ISR's boot counter into the total, and record whether the board is armed right
    /// now (the ride latch, which only ever gets set here: a board cannot have ridden without a
    /// pass having seen it armed).
    ///
    /// The delta is a WRAPPING subtraction and the accumulation is a SATURATING add. The first is
    /// why the ISR counter's wrap costs nothing: only the difference is consumed, and a difference
    /// is right across the wrap. The second is the answer to the total's own ceiling: at the top
    /// the total STOPS, because a wrapped lifetime total reads as a nearly new machine and nothing
    /// tells that from the truth, while a stuck maximum is visibly wrong. `u32::MAX` edges is
    /// about 24,751 km on a 6.5 inch hub at 15 pole pairs, so neither case is reachable; the point
    /// is that the behaviour is stated.
    pub fn fold(&mut self, edges_now: u32, armed: bool) {
        let delta = edges_now.wrapping_sub(self.folded);
        self.folded = edges_now;
        self.total = self.total.saturating_add(delta);
        if armed {
            self.ride = true;
        }
    }

    /// The whole write decision: the total to persist, or `None`.
    ///
    /// - `armed`: the mode machine's arm fact this pass. **`true` is always `None`**, and that is
    ///   the property the host test proves exhaustively.
    /// - `at_rest`: the wheel is demonstrably stopped. The caller passes `!arm::off_inhibit(..)`,
    ///   the OFF interlock's own predicate, so there is one owner of "the wheel is at rest" rather
    ///   than two that can disagree. It captures the coast-down, and it confines the write (and
    ///   any compaction it triggers) to a stationary machine.
    ///
    /// The two state conditions are the throttle: `ride` means this was a ride rather than a
    /// nudge across a workshop, and `total != written` means something moved.
    pub const fn due(&self, armed: bool, at_rest: bool) -> Option<u32> {
        if armed || !at_rest || !self.ride || self.total == self.written {
            None
        } else {
            Some(self.total)
        }
    }

    /// Record the outcome of the attempt [`Self::due`] called for.
    ///
    /// The ride latch clears either way. A board whose region is genuinely out of space therefore
    /// retries once per ride instead of once per loop pass, which is what keeps a failing write
    /// from becoming a flash-churn loop.
    pub fn settled(&mut self, total: u32, stored: bool) {
        self.ride = false;
        if stored {
            self.written = total;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every state this struct can reach, for the exhaustive sweeps below: a few totals, both
    /// written values that matter (equal and not), and both ride states.
    fn states() -> impl Iterator<Item = Odometer> {
        [0u32, 1, 4_096, u32::MAX - 1, u32::MAX]
            .into_iter()
            .flat_map(|total| {
                [total, total.wrapping_sub(1), 0]
                    .into_iter()
                    .flat_map(move |written| {
                        [false, true].into_iter().map(move |ride| Odometer {
                            total,
                            folded: 12_345,
                            written,
                            ride,
                        })
                    })
            })
    }

    /// **An armed board cannot be told to write, whatever else is true** (`specs/odometer.md`,
    /// "How an armed write is made impossible"). This is the first half of the proof that an
    /// internal writer honours the rule the L3 path honours through `CFG_ARMED`; the other half is
    /// `the_odometer_write_is_confined_to_one_site` below.
    #[test]
    fn an_armed_board_is_never_due_a_write() {
        for o in states() {
            for at_rest in [false, true] {
                assert_eq!(
                    o.due(true, at_rest),
                    None,
                    "armed, at_rest={at_rest}, total={}, written={}, ride={}",
                    o.total,
                    o.written,
                    o.ride
                );
            }
        }
    }

    /// The other three conditions, each one necessary on its own.
    #[test]
    fn a_write_needs_rest_a_ride_and_something_moved() {
        let base = Odometer {
            total: 500,
            folded: 0,
            written: 100,
            ride: true,
        };
        assert_eq!(base.due(false, true), Some(500), "all four conditions met");
        assert_eq!(base.due(false, false), None, "a turning wheel waits");
        assert_eq!(
            Odometer {
                ride: false,
                ..base
            }
            .due(false, true),
            None,
            "a nudge that followed no ride is not written"
        );
        assert_eq!(
            Odometer {
                written: 500,
                ..base
            }
            .due(false, true),
            None,
            "nothing moved since the last write"
        );
    }

    /// A ride, from boot to the record: fold while armed, disarm, coast, write once, and then
    /// stay quiet however many passes follow.
    #[test]
    fn one_ride_writes_exactly_one_record() {
        let mut o = Odometer::new(1_000, 0);
        o.fold(40, true); // armed and turning
        o.fold(90, true);
        assert_eq!(o.due(true, false), None, "nothing is written while armed");
        o.fold(120, false); // disarmed, still coasting
        assert_eq!(
            o.due(false, false),
            None,
            "the coast-down is not lost, it is waited for"
        );
        o.fold(131, false); // at rest
        assert_eq!(o.total(), 1_131, "the coast-down edges are in the total");
        let due = o.due(false, true).expect("a ride ended at rest is due");
        assert_eq!(due, 1_131);
        o.settled(due, true);
        for _ in 0..1_000 {
            o.fold(131, false);
            assert_eq!(o.due(false, true), None, "the record is not rewritten");
        }
    }

    /// A ride that turns no wheel costs no record, which is the "nothing moved" throttle.
    #[test]
    fn arming_without_moving_writes_nothing() {
        let mut o = Odometer::new(7, 55);
        o.fold(55, true);
        o.fold(55, false);
        assert_eq!(o.due(false, true), None);
        assert_eq!(o.total(), 7);
    }

    /// Travel while the board was never armed waits for the next ride rather than appending a
    /// record of its own, and it is not discarded: the ride that follows carries it.
    #[test]
    fn travel_while_disarmed_is_carried_into_the_next_ride() {
        let mut o = Odometer::new(0, 0);
        o.fold(300, false); // pushed across the workshop
        assert_eq!(o.due(false, true), None, "a push is not a ride");
        assert_eq!(o.total(), 300, "but it IS travel, and it is counted");
        o.fold(300, true); // armed
        o.fold(500, false); // rode, then stopped
        assert_eq!(
            o.due(false, true),
            Some(500),
            "the push is in the ride's record"
        );
    }

    /// A failed write costs ONE attempt, not one per pass.
    #[test]
    fn a_failed_write_retries_at_the_next_ride_not_the_next_pass() {
        let mut o = Odometer::new(0, 0);
        o.fold(10, true);
        o.fold(10, false);
        let due = o.due(false, true).unwrap();
        o.settled(due, false); // the region refused it
        assert_eq!(o.due(false, true), None, "no second attempt this ride");
        o.fold(10, true); // the next ride
        o.fold(20, false);
        assert_eq!(
            o.due(false, true),
            Some(20),
            "and it carries the unwritten travel"
        );
    }

    /// The ISR counter wraps; the total does not notice, because only the delta is consumed.
    #[test]
    fn the_boot_counters_wrap_is_folded_as_a_delta() {
        let mut o = Odometer::new(1_000, u32::MAX - 2);
        o.fold(2, false); // five edges across the wrap: MAX-1, MAX, 0, 1, 2
        assert_eq!(o.total(), 1_005);
    }

    /// The total stops at the top rather than wrapping onto a plausible small number.
    #[test]
    fn the_total_saturates_at_the_ceiling() {
        let mut o = Odometer::new(u32::MAX - 3, 0);
        o.fold(10, false);
        assert_eq!(o.total(), u32::MAX);
        o.fold(1_000_000, false);
        assert_eq!(o.total(), u32::MAX, "and it stops there");
    }

    /// **The odometer write is confined to one site** (`specs/odometer.md`, "How an armed write is
    /// made impossible"), which is the half of the proof the exhaustive `due` sweep cannot give:
    /// that no OTHER code path reaches the field with a different opinion about arming. The token
    /// is assembled from pieces so this test's own source does not contain it, and comment lines
    /// are stripped first, so prose may name the field while code may not.
    #[test]
    fn the_odometer_write_is_confined_to_one_site() {
        let strip = |src: &str| -> std::string::String {
            src.lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .collect::<std::vec::Vec<_>>()
                .join("\n")
        };
        let field = concat!("MOTOR_", "ODOMETER");

        // This file holds the decision and never the field: it cannot reach flash at all.
        assert!(
            !strip(include_str!("odometer.rs")).contains(field),
            "odometer.rs decides; it does not write"
        );
        assert!(
            !strip(include_str!("motor.rs")).contains(field),
            "the ISR counts edges; it does not write flash"
        );
        assert!(
            !strip(include_str!("arm.rs")).contains(field),
            "the arming layer does not write the odometer"
        );
        // And `main.rs` names it exactly twice: the boot read that seeds the total, and the one
        // write the decision above opens.
        assert_eq!(
            strip(include_str!("main.rs")).matches(field).count(),
            2,
            "one boot read and one write, and nothing else in the crate touches the field"
        );
    }
}
