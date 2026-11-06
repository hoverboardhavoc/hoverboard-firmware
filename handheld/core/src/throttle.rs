//! The relative throttle's mapping (`specs/handheld-remote.md` section 4.1): the touch-down point
//! is zero, dragging up is forward, and a full demand is [`FULL_PX`] of travel. In DIFFERENTIAL
//! the horizontal drag is the turn by the same rule (section 4.8): right is positive, and a full
//! turn is [`FULL_PX_X`] of travel.

use crate::touch::SUB;

/// Pixels of travel per full demand.
pub const FULL_PX: i32 = 120;
/// Pixels of horizontal travel per full turn (section 4.8).
pub const FULL_PX_X: i32 = 100;
/// Pixels of travel that command nothing (a resistive panel's rest jitter).
pub const DEAD_PX: i32 = 3;
/// The full `DRIVE_CMD.value`: `i16::MAX`.
pub const FULL_SCALE: i32 = i16::MAX as i32;

/// The demand for a touch that went down at `y0` and is now at `yf`, both in sixteenths of a pixel
/// (y down): `sign(dy) * round_half_away(mag * 32767)` with `dy = y0 - yf` and
/// `mag = max(|dy| - DEAD_PX, 0) / (FULL_PX - DEAD_PX)`, at most 1. Linear, symmetric, never past
/// +-32767 (the app's `Throttle.speedFor` rounding, `Throttle.kt`).
pub fn demand(y0: i32, yf: i32) -> i16 {
    scaled(y0 - yf, FULL_PX)
}

/// The turn for a touch that went down at `x0` and is now at `xf`, sixteenths of a pixel (section
/// 4.8): `dx = xf - x0`, right positive, with the dead band and rounding of [`demand`] and a full
/// turn at [`FULL_PX_X`].
pub fn turn(x0: i32, xf: i32) -> i16 {
    scaled(xf - x0, FULL_PX_X)
}

/// `sign(d) * round_half_away(mag * 32767)`, `mag = max(|d| - DEAD_PX, 0) / (full_px - DEAD_PX)`
/// at most 1, `d` in sixteenths of a pixel.
fn scaled(d: i32, full_px: i32) -> i16 {
    let excess = (d.abs() - DEAD_PX * SUB).max(0) as i64;
    let den = ((full_px - DEAD_PX) * SUB) as i64;
    // Round half away from zero on the magnitude: (2 n + d) / 2 d.
    let mag = ((2 * excess * FULL_SCALE as i64 + den) / (2 * den)).min(FULL_SCALE as i64) as i16;
    if d < 0 {
        -mag
    } else {
        mag
    }
}

/// A demand or turn as a whole percent of full scale, rounded half away from zero.
pub fn percent(v: i16) -> i8 {
    let v = i32::from(v);
    ((v * 100 + v.signum() * FULL_SCALE / 2) / FULL_SCALE) as i8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn px(p: i32) -> i32 {
        p * SUB
    }

    #[test]
    fn zero_at_the_touch_down_point() {
        assert_eq!(demand(px(160), px(160)), 0);
    }

    #[test]
    fn the_dead_band_edges() {
        assert_eq!(demand(px(160), px(160 - DEAD_PX)), 0);
        assert_eq!(demand(px(160), px(160 + DEAD_PX)), 0);
        // One sixteenth past the band: 32767 / (117 * 16) = 17.5, rounded away to 18.
        assert_eq!(demand(px(160), px(160 - DEAD_PX) - 1), 18);
        assert_eq!(demand(px(160), px(160 + DEAD_PX) + 1), -18);
    }

    #[test]
    fn full_at_full_px_and_clamped_past_it() {
        assert_eq!(demand(px(200), px(200 - FULL_PX)), 32767);
        assert_eq!(demand(px(200), px(200 + FULL_PX)), -32767);
        assert_eq!(demand(px(300), px(0)), 32767);
        assert_eq!(demand(px(0), px(300)), -32767);
    }

    #[test]
    fn rounding_is_symmetric_and_half_away() {
        // The app's half-travel case (`ThrottleTest.kt`, the symmetric rounding cases), rescaled to
        // pixels: half the travel past the band is 32767 / 2 = 16383.5, which rounds to 16384 both
        // ways rather than 16384 forward and -16383 in reverse.
        let half = DEAD_PX + (FULL_PX - DEAD_PX) / 2; // 61 px: 58 of 117 past the band
        let f = demand(px(200), px(200 - half));
        let r = demand(px(200), px(200 + half));
        assert_eq!(f, -r);
        let exact = demand(
            px(200),
            px(200) - (DEAD_PX * SUB + (FULL_PX - DEAD_PX) * SUB / 2),
        );
        assert_eq!(exact, 16384);
        assert_eq!(
            demand(
                px(200),
                px(200) + (DEAD_PX * SUB + (FULL_PX - DEAD_PX) * SUB / 2)
            ),
            -16384
        );
        for d in 0..(FULL_PX * SUB + 40) {
            assert_eq!(
                demand(1000 * SUB, 1000 * SUB - d),
                -demand(1000 * SUB, 1000 * SUB + d)
            );
        }
    }

    #[test]
    fn the_turn_is_zero_at_x0_has_the_dead_band_and_is_full_at_full_px_x() {
        assert_eq!(turn(px(120), px(120)), 0);
        assert_eq!(turn(px(120), px(120 + DEAD_PX)), 0);
        assert_eq!(turn(px(120), px(120 - DEAD_PX)), 0);
        // One sixteenth past the band: 32767 / (97 * 16) = 21.1, rounded to 21.
        assert_eq!(turn(px(120), px(120 + DEAD_PX) + 1), 21);
        assert_eq!(turn(px(120), px(120 + FULL_PX_X)), 32767);
        assert_eq!(turn(px(120), px(120 - FULL_PX_X)), -32767);
        assert_eq!(turn(px(120), px(239)), 32767, "clamped past full");
        // Halfway past the band, both ways the same.
        let half = DEAD_PX * SUB + (FULL_PX_X - DEAD_PX) * SUB / 2;
        assert_eq!(turn(px(120), px(120) + half), 16384);
        assert_eq!(turn(px(120), px(120) - half), -16384);
    }

    #[test]
    fn right_is_a_positive_turn() {
        assert!(turn(px(100), px(150)) > 0);
        assert!(turn(px(100), px(50)) < 0);
    }

    #[test]
    fn percent_rounds_half_away() {
        assert_eq!(percent(0), 0);
        assert_eq!(percent(32767), 100);
        assert_eq!(percent(-32767), -100);
        assert_eq!(percent(16384), 50);
        assert_eq!(percent(-16384), -50);
        assert_eq!(percent(164), 1, "0.5 percent rounds up");
        assert_eq!(percent(-164), -1);
    }

    #[test]
    fn never_past_full_scale_and_monotonic() {
        let mut prev = 0;
        for d in 0..(400 * SUB) {
            let v = demand(400 * SUB + d, 400 * SUB) as i32;
            assert!(v >= prev && v <= 32767);
            prev = v;
        }
    }
}
