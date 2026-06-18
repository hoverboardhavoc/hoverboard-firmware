//! Shared helper routines (Section 8). Pure, deterministic, no_std.
//!
//! These are load-bearing: the clamp ORDER and the anti-windup orientation are part of the
//! contract, not incidental. The fixed-point right shifts use the round-toward-zero correction
//! the original's EABI float-to-int behaviour and arithmetic shifts require (Section 9).

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
    debug_assert!(n >= 1 && n <= 31);
    // The ARM idiom: `asr #31` gives 0 (non-negative) or -1 (negative); the second shift is a
    // LOGICAL right shift (`lsr #(32-n)`), so for a negative value it yields the bias 2^n - 1, and
    // for a non-negative value it yields 0. Adding that bias before the arithmetic `>> n` makes the
    // division truncate toward zero. Using an arithmetic second shift (Rust's `>>` on i32) would
    // give -1 instead of the bias and over-correct; do the second shift on the u32 view.
    let correction = (((x >> 31) as u32) >> (32 - n)) as i32;
    (x.wrapping_add(correction)) >> n
}

/// Inner current-loop PI record (Section 3.1 / Section 8.1). Halfword indices preserved as
/// named fields. The integral clamp fields are seeded INVERTED relative to their names
/// (Section 14): `int_max` holds the NEGATIVE value and is used as the LOW bound; `int_min`
/// holds the POSITIVE value and is used as the HIGH bound. Clamp BY VALUE, not by field name.
#[derive(Clone, Copy, Debug)]
pub struct PiRecord {
    /// record[0]
    pub kp: i32,
    /// record[1], unsigned 16-bit divisor
    pub kp_divisor: i32,
    /// record[2]
    pub ki: i32,
    /// record[3], unsigned 16-bit divisor
    pub ki_divisor: i32,
    /// record[4]
    pub out_min: i32,
    /// record[5]
    pub out_max: i32,
    /// record[6..7]: seeded NEGATIVE (0xF0002000 = -268427264); used as the LOW bound.
    pub int_max: i64,
    /// record[8..9]: seeded POSITIVE (+268427264); used as the HIGH bound.
    pub int_min: i64,
    /// record[10..11]: integral accumulator, used 64-bit wide.
    pub accumulator: i64,
}

impl PiRecord {
    /// The Section 3.1 seed instantiation (the reference inner FOC PI record). These are the
    /// defaults; per the deliverable, gains/limits are tunable inputs with the spec values as
    /// defaults.
    pub const fn seed() -> Self {
        // 0xF0002000 interpreted as a signed 32-bit value is -268427264.
        const INT_MAX: i64 = 0xF000_2000u32 as i32 as i64; // -268427264 (negative; LOW bound)
        Self {
            kp: 100,
            kp_divisor: 0x400,   // 1024, proportional divisor (index [1])
            ki: 0x32,            // 50, integral gain (index [2])
            ki_divisor: 0x2000,  // 8192
            out_min: 0x8001u16 as i16 as i32, // -32767
            out_max: 0x7FFF,                  // +32767
            int_max: INT_MAX,    // -268427264, used as LOW bound
            int_min: -INT_MAX,   // +268427264, used as HIGH bound
            accumulator: 0,
        }
    }
}

/// Section 8.1: PI integrator with anti-windup. Returns the int16 output and mutates the
/// record's accumulator. The clamp interval is `[int_max, int_min]` = `[-268427264, +268427264]`
/// (low = the negative field, high = the positive field) by VALUE.
pub fn pi_step(setpoint: i32, measured: i32, record: &mut PiRecord) -> i16 {
    let e = setpoint - measured;

    if record.ki == 0 {
        // Step 1: clear the accumulator and skip integration.
        record.accumulator = 0;
    } else {
        // Step 2: accumulate in 64-bit, then clamp by value into [int_max, int_min].
        let acc = record.accumulator + (e as i64) * (record.ki as i64);
        // Exact branch form from the spec (int_min = positive HIGH bound, int_max = negative LOW):
        //   if int_min >= acc: accumulator = acc if acc >= int_max else int_max
        //   else: accumulator = int_min
        record.accumulator = if record.int_min >= acc {
            if acc >= record.int_max {
                acc
            } else {
                record.int_max
            }
        } else {
            record.int_min
        };
    }

    // Step 3: out = accumulator / Ki_divisor + (e * Kp) / Kp_divisor (integer divide, toward zero).
    let i_term = record.accumulator / (record.ki_divisor as i64);
    let p_term = ((e * record.kp) / record.kp_divisor) as i64;
    let out = i_term + p_term;

    // Step 4: clamp out into [out_min, out_max] and return as int16.
    let clamped = clamp(out as i32, record.out_min, record.out_max);
    clamped as i16
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
