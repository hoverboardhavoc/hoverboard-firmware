// SPDX-License-Identifier: GPL-3.0-or-later
//! The current limit's arithmetic (`specs/motor-integration.md`, "The current limit"): the
//! calibration and noise-floor seams, the count window, and the conversions the firmware's bring-up
//! and arm make from the staged milliamps (`firmware::motor` re-exports them). Here rather than in
//! the firmware so the configuration checks (`tools/hoverboard-cli/src/checks.rs`, `limit_floor`,
//! `specs/config-read-once.md` section 4) refuse a limit by the same arithmetic the board enforces.
//! Pure and `no_std`; the firmware's host tests exercise it through the re-export.

/// The current-sense calibration's seam range, stock current counts per amp of phase current.
///
/// The count is the unit `commutation::foc::current_from_adc` defines and the FOC arm consumes
/// (`offset - 2*sample` over the left-aligned injected word, so one 12-bit ADC LSB of deviation is
/// 16 counts and the sensor's full scale is +-32767 counts); nothing is rescaled, the limit is
/// converted INTO it. The scale itself is per-board data, read at boot from
/// `store::MOTOR_CURRENT_CAL` (0x4C) by the arm row and carried into [`limit_counts`]:
/// it is a property of the shunt and amplifier chain fitted, not a constant
/// (`specs/motor-integration.md`, "The current-sense calibration").
///
/// The seam clamps it HERE, because the store validates type only (the
/// `orchestrator::battery::VbattCal::new` precedent), and the band is a PLAUSIBILITY one on a board
/// fact, the same class as the battery divider's slope clamp. It is no longer an arithmetic bound:
/// 819 was once derived from a 40 A milliamp ceiling fitting the `i16` (`40_000 * 819 / 1000 <=
/// i16::MAX`), and that ceiling is gone, because [`MAX_LIMIT_COUNTS`] bounds the CONVERTED count
/// instead and nothing the conversion computes can leave the `i16` whatever the stored scale is.
///
/// What the band still buys, which the count window cannot: it bounds how far a typoed scale can
/// move the ENFORCED limit away from the LABELLED one. The count ceiling bounds the limit in
/// absolute terms; the ratio is the seam's. Without the upper bound a stored 8,190 would convert a
/// deliberately conservative 5 A staging to the widest limit the window admits (16,383 counts,
/// 36 A at this chain's measured scale) with nothing said; with it, the factor a wrong scale can
/// introduce is at most `819 / true`, and the two scales any source has claimed for this hardware
/// class both sit inside the band (455 measured here, EFeru's 800). The lower bound keeps a zero or
/// a typo from collapsing every staged limit onto the noise floor, which reads as "no drive".
pub const CURRENT_CAL_MIN: u16 = 100;
/// The calibration's upper seam bound; see [`CURRENT_CAL_MIN`].
pub const CURRENT_CAL_MAX: u16 = 819;

/// The lowest noise floor a board may claim, in counts (1,000): the bottom of the plausibility band
/// the boot seam clamps `store::MOTOR_NOISE_FLOOR` (0x6B) into, whose top is [`MAX_LIMIT_COUNTS`].
///
/// **The floor itself is per-board data and no longer a constant.** It is the rest noise of the
/// shunt and amplifier chain fitted, measured in the same counts as `motor.current_cal`, and it was
/// a compiled constant of 2,100, taken from the high-water of five rest-floor reads on ONE board in
/// ONE session (1,444 / 1,863 / 1,910 / 1,941 / 2,097 counts, medians around 1,050). That spread is
/// the argument. One number for every chain is too low for the noisy end, where it chops on the
/// chain's own noise (this constant was 2,000 until an armed STILL soak on 2026-10-09 chopped in 2
/// of 94 samples with demand and duty 0 throughout), and too high for the quiet end, where it
/// silently raises the minimum enforceable current. 2,100 is now the FIELD's default, so a board
/// with no record enforces exactly what the image enforced before
/// (`specs/store-field-audit.md`, the 2026-10-09 sweep, Tier 1 item 1).
///
/// **Why 1,000 is the bottom.** A claim below it is under the quietest rest reading any bench board
/// has produced (1,054 median, 1,444 worst of the five), so it is a typo rather than a measurement,
/// and the consequence of accepting it is a board that trips at rest. The band is a PLAUSIBILITY
/// one at the seam rather than a validated type, the shape `CURRENT_CAL_MIN` and the dead-time floor
/// both use: the store validates type only, so [`noise_floor_counts`] is where a hand-poked flash
/// value stops.
pub const NOISE_FLOOR_MIN: i16 = 1_000;

/// The converted limit's CEILING, in counts (16,383): half the sensor's full scale, the largest
/// soft limit whose hard trip ([`hard_trip_counts`], twice the limit) is still expressible in the
/// `i16` the comparison holds.
///
/// A count for the same reason as the noise floor (see [`NOISE_FLOOR_MIN`]), and one step further:
/// this bound is a property of the COMPARISON, so it is not any board's number at all. It replaces a 40,000 mA
/// ceiling, which is a count only once a scale is assumed, and which was the wrong half of the
/// constraint: it checked that the SOFT limit fits the `i16` and never that the hard trip does. At
/// 800 counts per amp a 40 A request converted to 32,000 counts, [`hard_trip_counts`] saturated at
/// 32,767, and the 2x margin the trip's whole justification rests on degenerated to 1.02x with no
/// diagnostic (`specs/store-field-audit.md`, the 2026-10-09 board-specifics sweep, Tier 1 item 2).
///
/// Clamping the converted count makes the 2x hold for every limit the window admits, and the real
/// ceiling then follows the board's own scale, which is the point: 36.0 A at the measured 455
/// counts per amp, 20.5 A at EFeru's 800, 20.0 A at the top of the calibration seam.
///
/// It is also the TOP of the noise floor's plausibility band, which is what keeps the 2x true now
/// the floor is per-board: a floor clamped to at most this bound leaves every limit
/// [`limit_counts`] can produce inside `0..=MAX_LIMIT_COUNTS` whatever a board claims. At the very
/// top the window is one value wide and the limit has no range left, which is the diagnostic a
/// board staging a floor there has asked for.
pub const MAX_LIMIT_COUNTS: i16 = i16::MAX / 2;

// What MAX_LIMIT_COUNTS is FOR, pinned here so it cannot drift from the function that depends on
// it: the hard trip of the largest admissible limit is still an i16, so `hard_trip_counts`'
// saturation is unreachable for every value `limit_counts` can produce. The runtime clamp in
// `limit_counts` is what holds the property for any stored request and scale; this pins the bound
// it clamps to.
const _: () = assert!(
    2 * MAX_LIMIT_COUNTS as i32 <= i16::MAX as i32,
    "the hard trip of the largest admissible limit no longer fits the comparison's i16"
);
// The noise floor's band, which with a per-board floor is what the 2x assert above rests on: a
// clamped floor never exceeds the ceiling, so `limit_counts`' clamp is well-formed (`i16::clamp`
// panics on an inverted pair) and its result is bounded by `MAX_LIMIT_COUNTS` for every value a
// board can stage.
const _: () = assert!(
    NOISE_FLOOR_MIN < MAX_LIMIT_COUNTS,
    "the noise floor's plausibility band is empty: its bottom has reached the limit ceiling"
);

/// The largest staged request [`limit_counts`] multiplies: an OVERFLOW guard for its 32-bit
/// arithmetic, not a policy ceiling, and derived from the window rather than from an assumed scale.
/// Capping the request here cannot change what any board enforces, because even at the lowest scale
/// the calibration seam admits this many milliamps already converts above the window's top: both
/// properties are pinned below, so neither can drift if a bound moves.
pub const LIMIT_REQUEST_CAP_MA: u32 =
    (MAX_LIMIT_COUNTS as u32 + 1) * 1_000 / CURRENT_CAL_MIN as u32;
const _: () = assert!(
    LIMIT_REQUEST_CAP_MA
        .checked_mul(CURRENT_CAL_MAX as u32)
        .is_some(),
    "the capped request times the widest scale no longer fits the conversion's u32"
);
const _: () = assert!(
    LIMIT_REQUEST_CAP_MA * CURRENT_CAL_MIN as u32 / 1_000 > MAX_LIMIT_COUNTS as u32,
    "the request cap is low enough to move an enforced limit, so it is a ceiling and not a guard"
);

/// What this board's staged noise floor (`store::MOTOR_NOISE_FLOOR`, 0x6B) means to the limit: the
/// stored word clamped into [`NOISE_FLOOR_MIN`]`..=`[`MAX_LIMIT_COUNTS`], the plausibility band.
///
/// The ONE owner of that band, so [`limit_counts`] and anything that reports what the board will do
/// read the same number. The store validates type only, so this is the seam where a hand-poked or
/// typoed flash value stops, the `CURRENT_CAL_MIN` shape and the `orchestrator::battery::VbattCal`
/// precedent. Clamped rather than refused, for the reason [`limit_counts`] gives: a boot seam has
/// nowhere to refuse a value TO, and a board that refused to enforce any limit would be worse than
/// one enforcing a plausible floor.
///
/// The result is positive and at most [`MAX_LIMIT_COUNTS`], which is what [`limit_counts`]' clamp
/// needs of it (see the const asserts beside the bounds).
#[inline]
pub fn noise_floor_counts(stored: u16) -> i16 {
    stored.clamp(NOISE_FLOOR_MIN as u16, MAX_LIMIT_COUNTS as u16) as i16
}

/// The staged request in counts before [`limit_counts`]' window: `ma * clamp(cal, 100, 819) /
/// 1000`, the request capped at [`LIMIT_REQUEST_CAP_MA`]. The configuration checks report it
/// beside the floor when a limit converts at or below it (`specs/config-read-once.md` section 4,
/// `limit_floor`).
#[inline]
pub fn request_counts(ma: u32, cal: u16) -> u32 {
    let cal = cal.clamp(CURRENT_CAL_MIN, CURRENT_CAL_MAX) as u32;
    // The request is bounded only so the 32-bit multiply cannot overflow; see
    // `LIMIT_REQUEST_CAP_MA` for why it is not a ceiling in disguise.
    ma.min(LIMIT_REQUEST_CAP_MA) * cal / 1_000
}

/// Convert the staged `MOTOR_CURRENT_LIMIT` (milliamps) into the soft limit in stock current
/// counts, once, at bring-up: `ma * clamp(cal, 100, 819) / 1000`, clamped into
/// [`noise_floor_counts`]`(noise_floor)..=`[`MAX_LIMIT_COUNTS`].
///
/// **Both bounds are COUNTS, the domain the comparison works in**, and the clamp is on the RESULT,
/// not on the request: the request is milliamps, what a milliamp buys is the board's scale, and a
/// bound in milliamps is therefore a bound at one assumed scale (see [`MAX_LIMIT_COUNTS`] for what
/// that cost). A request the board's scale cannot express is clamped DOWN to the ceiling, exactly
/// as a request under the sense chain's noise is clamped UP to the floor: an out-of-range staged
/// value is bounded here, never refused, and there is nowhere at a boot seam to refuse it to.
///
/// **Two of the three inputs are this board's own measured facts, not constants.** `cal` is its
/// counts per amp (`store::MOTOR_CURRENT_CAL`, read with the arm row) and `noise_floor` is
/// its sense chain's rest noise (`store::MOTOR_NOISE_FLOOR`, carried on `firmware::motor::BootFixed`); this is the
/// boot seam that clamps both, because the store validates type only, so a hand-poked out-of-range
/// flash value cannot reach the comparison.
///
/// **The floor moves, the ceiling does not, and the hard trip stays twice the limit.** A board may
/// raise its own minimum enforceable current; it cannot widen the window past
/// [`MAX_LIMIT_COUNTS`], because its floor is clamped to at most that bound. So every limit this
/// can return is still in `0..=MAX_LIMIT_COUNTS` and [`hard_trip_counts`]' doubling is still exact
/// at every calibration and every claimed floor.
#[inline]
pub fn limit_counts(ma: u32, cal: u16, noise_floor: u16) -> i16 {
    let counts = request_counts(ma, cal);
    // The window, both ends visible in one expression. The cast cannot truncate: the clamped floor
    // is positive and at most MAX_LIMIT_COUNTS, so the clamped value is in 0..=MAX_LIMIT_COUNTS.
    counts.clamp(
        noise_floor_counts(noise_floor) as u32,
        MAX_LIMIT_COUNTS as u32,
    ) as i16
}

/// The FOC arm's current-reference full scale, in counts: what a full-scale torque word (+-28,500,
/// `base::TORQUE_FULL_SCALE`) asks the q-axis loop for (`specs/foc-torque.md` section 4). The soft
/// limit [`limit_counts`] produced, less this board's noise floor ([`noise_floor_counts`]),
/// floored at zero.
///
/// **Derived, not a field**, so its three inputs (`motor.current_limit` 0x20, `motor.current_cal`
/// 0x67, `motor.noise_floor` 0x6B) each keep their one owner: a separate scale field would need a
/// validation rule against the limit, because a reference above the chop threshold makes the chop
/// and the loop fight, and one field cannot disagree with itself. EFeru's torque mode makes its
/// current limit the reference ceiling the same way.
///
/// **Why the floor comes off.** The soft chop acts on this period's phase MAGNITUDE against the
/// limit, unfiltered, and the loop regulates the vector's amplitude, whose phase peaks touch the
/// magnitude six times per electrical turn; a reference AT the limit would chop on measurement
/// noise at full demand, and one floated period moves the current by amps, which the loop then
/// chases. The noise floor is the board's own figure for exactly that band, so `limit - floor` is
/// the highest reference at which the chop stays silent and remains the backstop it is meant to be.
///
/// At the fleet defaults (10,000 mA, 455 counts per amp, floor 2,100) that is 4,550 - 2,100 = 2,450
/// counts, 5.4 A of phase-current amplitude. The limit is never below the clamped floor, so this
/// reaches zero exactly when the limit sits AT the floor, which is the board saying it cannot hold
/// any current above its noise.
#[inline]
pub fn full_scale_counts(limit_counts: i16, noise_floor: u16) -> i16 {
    // In i32, so a pair outside the window `limit_counts` produces still subtracts without wrapping;
    // the result is in `0..=MAX_LIMIT_COUNTS` for every pair it can produce.
    (limit_counts as i32 - noise_floor_counts(noise_floor) as i32)
        .max(0)
        .min(i16::MAX as i32) as i16
}

/// The hard trip's magnitude: twice the soft limit, saturated to the sensor's full scale. A
/// working chop cannot be holding a current this far over its limit (one period of float moves a
/// few amps); a shorted phase or a wrong hall table into a locked rotor can.
///
/// **The saturation is unreachable for every limit [`limit_counts`] produces**, which is what
/// [`MAX_LIMIT_COUNTS`] exists to guarantee and what the const assert beside it pins: the 2x is
/// exact across the whole admissible window, including at its top. It stays as the guard for an
/// argument this `pub fn`'s `i16` admits and the conversion cannot hand it.
#[inline]
pub const fn hard_trip_counts(limit_counts: i16) -> i16 {
    let h = 2 * limit_counts as i32;
    if h > i16::MAX as i32 {
        i16::MAX
    } else {
        h as i16
    }
}
