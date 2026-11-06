//! The DIFFERENTIAL per-wheel mix (`specs/handheld-remote.md` section 4.8): the app's `DriveMix`
//! (`apps/remote/app/src/main/java/com/hoverboard/remote/model/DriveMix.kt`), ported with its tests
//! (`apps/remote/app/src/test/java/com/hoverboard/remote/DriveMixTest.kt`).
//!
//! `left = v + s/2`, `right = v - s/2`. When a wheel passes the limit, both move by one offset so
//! their difference (the turn) survives and the travel they share gives way. A turn wider than the
//! whole range puts the wheels on opposite rails in the direction asked. Positive `steer` runs the
//! left wheel faster, which turns the machine right.

/// The wire's full scale (`DriveCmd.FULL_SCALE`, `i16::MAX`).
pub const FULL_SCALE: i32 = i16::MAX as i32;

/// What one deflection asks of each wheel.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct WheelPair {
    pub left: i32,
    pub right: i32,
}

impl WheelPair {
    /// How far apart the two wheels are told to run: the commanded turn.
    pub fn difference(&self) -> i32 {
        self.left - self.right
    }
}

/// The pair for a forward demand `forward` and a turn `steer`, both on the `limit` scale (the
/// app's `DriveMix.of`). `steer / 2` truncates toward zero, as Kotlin's integer division does.
pub fn of(forward: i32, steer: i32, limit: i32) -> WheelPair {
    let half = steer / 2;
    let sign = |v: i32| if v < 0 { -1 } else { 1 };
    // Wider than the range itself: the rails are as far apart as two demands get.
    if half.abs() > limit {
        return WheelPair {
            left: sign(half) * limit,
            right: -sign(half) * limit,
        };
    }
    let left = forward + half;
    let right = forward - half;
    // One offset for both wheels, so the difference comes through untouched. Only one of the two
    // rails can be past: the pair is at most 2 * limit wide by the guard above.
    let offset = if left.max(right) > limit {
        limit - left.max(right)
    } else if left.min(right) < -limit {
        -limit - left.min(right)
    } else {
        0
    };
    WheelPair {
        left: left + offset,
        right: right + offset,
    }
}

#[cfg(test)]
mod tests {
    //! Each test is the `DriveMixTest.kt` test of the same name, with its vectors.
    use super::*;

    const MAX: i32 = FULL_SCALE;

    fn mix(v: i32, s: i32) -> WheelPair {
        of(v, s, MAX)
    }

    /// What the stick asked for, which the pair must still differ by.
    fn asked_difference(steer: i32) -> i32 {
        2 * (steer / 2)
    }

    fn pair(left: i32, right: i32) -> WheelPair {
        WheelPair { left, right }
    }

    #[test]
    fn no_turn_tells_both_wheels_the_same_thing() {
        for v in [-MAX, -1000, 0, 1000, MAX] {
            let p = mix(v, 0);
            assert_eq!(p, pair(v, v), "v = {v}");
            assert_eq!(p.difference(), 0);
        }
    }

    #[test]
    fn a_turn_on_the_spot_runs_the_wheels_opposite_ways() {
        let p = mix(0, 1000);
        assert_eq!(p, pair(500, -500));
        assert_eq!(p.difference(), 1000);
    }

    #[test]
    fn positive_steer_runs_the_left_wheel_faster_which_is_a_turn_to_the_right() {
        let p = mix(4000, 2000);
        assert_eq!(p, pair(5000, 3000));
        assert!(p.left > p.right);
    }

    #[test]
    fn a_mix_that_fits_is_the_mix_untouched() {
        for v in [-10_000, 0, 500, 10_000] {
            for s in [-4000, -1, 0, 1, 4000] {
                let p = mix(v, s);
                assert_eq!(p.left, v + s / 2, "v = {v}, s = {s}");
                assert_eq!(p.right, v - s / 2, "v = {v}, s = {s}");
            }
        }
    }

    #[test]
    fn one_wheel_clipping_moves_both_and_keeps_the_whole_turn() {
        let p = mix(MAX, MAX);
        assert_eq!(p.left, MAX);
        assert_eq!(p.right, MAX - asked_difference(MAX));
        assert_eq!(p.difference(), asked_difference(MAX));
        assert!(p.right.abs() <= MAX);
        assert_ne!(p.difference(), 0);
    }

    #[test]
    fn clipping_in_reverse_keeps_the_turn_as_well_and_its_direction() {
        let p = mix(-MAX, MAX);
        assert_eq!(p.right, -MAX);
        assert_eq!(p.left, -MAX + asked_difference(MAX));
        assert_eq!(p.difference(), asked_difference(MAX));
        assert!(
            p.left > p.right,
            "a right turn still runs the left wheel faster"
        );
    }

    #[test]
    fn a_forward_demand_past_full_scale_is_held_at_it() {
        assert_eq!(mix(2 * MAX, 0), pair(MAX, MAX));
    }

    #[test]
    fn every_deflection_keeps_the_asked_difference_inside_the_range() {
        let axis = [-MAX, -MAX / 2, -33, 0, 33, MAX / 3, MAX / 2, MAX];
        for v in axis {
            for s in axis {
                let p = mix(v, s);
                assert_eq!(p.difference(), asked_difference(s), "v = {v}, s = {s}");
                assert!(p.left.abs() <= MAX, "left out of range at v = {v}, s = {s}");
                assert!(
                    p.right.abs() <= MAX,
                    "right out of range at v = {v}, s = {s}"
                );
            }
        }
    }

    #[test]
    fn the_mix_is_symmetric_under_negation() {
        for v in [0, 1000, MAX / 2, MAX] {
            for s in [0, 1, 1000, MAX] {
                let p = mix(v, s);
                assert_eq!(mix(-v, -s), pair(-p.left, -p.right), "v = {v}, s = {s}");
            }
        }
    }

    #[test]
    fn a_turn_wider_than_the_range_goes_to_the_rails_and_loses_the_forward_demand() {
        let right = of(10, 1000, 100);
        assert_eq!(right, pair(100, -100));
        assert_eq!(right.difference(), 200);
        assert!(right.difference() < asked_difference(1000));
        assert_eq!(of(10, -1000, 100), pair(-100, 100));
    }

    #[test]
    fn a_full_scale_stick_never_reaches_the_case_the_difference_cannot_hold() {
        for s in [-MAX, -MAX + 1, MAX - 1, MAX] {
            assert_eq!(mix(0, s).difference(), asked_difference(s), "s = {s}");
        }
    }
}
