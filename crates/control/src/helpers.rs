//! Shared helper routines (Section 8). Pure, deterministic, no_std.
//!
//! These are load-bearing: the clamp ORDER and the anti-windup orientation are part of the
//! contract, not incidental. The fixed-point right shifts use the round-toward-zero correction
//! the original's EABI float-to-int behaviour and arithmetic shifts require (Section 9).
//!
//! The Section 8.1 PI record + step recovered alongside these were relocated to `base::pi`
//! in Phase B (two independent layers consume them: the commutation crate's q-axis current PI
//! and this crate's balance loop); the stock inner-current-loop seed lives with its consumer
//! (the commutation q-PI), not here. This module recovers the remainder.

/// Section 8.3: symmetric clamp. `+limit` if `value > limit`, `-limit` if `value < -limit`,
/// else `value`. `limit` is non-negative.
#[inline]
pub fn clamp_sym(value: i32, limit: i32) -> i32 {
    if value > limit {
        limit
    } else if value < -limit {
        -limit
    } else {
        value
    }
}

/// Section 8.4: bounded clamp. Returns `lo` if `value < lo`, `hi` if `value > hi`, else `value`.
/// (Spec writes it in-place via a pointer; here it returns the clamped value.)
#[inline]
pub fn clamp(value: i32, lo: i32, hi: i32) -> i32 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

/// Section 8.5: integer absolute value.
#[inline]
pub fn iabs(x: i32) -> i32 {
    if x < 0 {
        -x
    } else {
        x
    }
}

/// Arithmetic right shift by `n` with round-toward-zero correction (Section 9):
/// `(x + ((x >> 31) >> (32 - n))) >> n`. This reproduces the original's truncate-toward-zero
/// behaviour for negative values, NOT a plain floor shift.
#[inline]
pub fn shr_round_to_zero(x: i32, n: u32) -> i32 {
    // (Archive form `n >= 1 && n <= 31`; clippy::manual_range_contains forces the rewrite.)
    debug_assert!((1..=31).contains(&n));
    // The ARM idiom: `asr #31` gives 0 (non-negative) or -1 (negative); the second shift is a
    // LOGICAL right shift (`lsr #(32-n)`), so for a negative value it yields the bias 2^n - 1, and
    // for a non-negative value it yields 0. Adding that bias before the arithmetic `>> n` makes the
    // division truncate toward zero. Using an arithmetic second shift (Rust's `>>` on i32) would
    // give -1 instead of the bias and over-correct; do the second shift on the u32 view.
    let correction = (((x >> 31) as u32) >> (32 - n)) as i32;
    (x.wrapping_add(correction)) >> n
}

/// Section 8.2: slew limiter / ramp record. The fixed step is 0x20 = 32, the counter cap is
/// 0xFA = 250.
#[derive(Clone, Copy, Debug, Default)]
pub struct RampRecord {
    pub current_value: i32,
    pub step_threshold: i32,
    pub step_bound: i32,
    pub small_step: i32,
    pub counter: i32,
}

/// The fixed slew step (Section 8.2).
pub const RAMP_STEP: i32 = 0x20; // 32
/// The saturating counter cap (Section 8.2).
pub const RAMP_COUNTER_CAP: i32 = 0xFA; // 250

/// Section 8.2: one ramp step. Moves `current_value` toward `target` by a bounded step.
pub fn ramp_step(target: i32, current_speed: i32, record: &mut RampRecord) -> i32 {
    // 1. Fast-snap region.
    if current_speed / 1000 <= record.step_threshold {
        record.step_bound = RAMP_STEP;
        record.current_value += record.small_step;
        return record.current_value;
    }
    // 2. Saturating counter (cap 250).
    if record.counter < RAMP_COUNTER_CAP {
        record.counter += 1;
    }
    // 3. Bounded step toward the target.
    let b = record.step_bound;
    let cv = record.current_value;
    record.current_value = if b < cv {
        cv - b
    } else if cv > 0 {
        cv - RAMP_STEP
    } else if -b < cv {
        cv + RAMP_STEP
    } else {
        cv + b
    };
    let _ = target; // target selects direction in the full inner-loop use; kept for signature parity.
    record.current_value
}
