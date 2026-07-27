//! The recovered PI-regulator record and step (`specs/commutation.md`, "Fixed-point formats").
//!
//! Recovered from the archived `control::helpers` (`archive/accumulated-build`, commit
//! `74b7773`) and re-pinned against the stock DECOMPILE itself (BalanceAgain
//! `pid_compute_clamped` @ 0x08004834 + `balance_config_init` @ 0x080047cc): the record layout,
//! the clamp ORDER / anti-windup orientation, and the FIELD WIDTHS are part of the recovered
//! stock contract. It lives in `base` because two independent layers consume it: the commutation
//! crate's q-axis current PI (now) and the Phase-D balance loop (later); neither should depend
//! on the other for a shared primitive.
//!
//! **Widths (the stock contract, and the 16 kHz cycle budget's load-bearing fact):** the stock
//! record stores the integral accumulator and both integral bounds as **32-bit** values
//! (byte offsets +12/+16/+20, 4-byte spacing; the decompile loads and stores 32 bits). The
//! accumulate step is evaluated in 64-bit (the decompile's `adds/adc` sign-extend idiom) so the
//! bound compare cannot overflow, and the clamped result always fits 32 bits because the bounds
//! do. The output divisions are **32-bit** signed divides by zero-extended 16-bit divisors:
//! hardware SDIV on this core, never a 64-bit software-division call. An earlier reading of the
//! archived record as "64-bit accumulator" mis-took the two HALFWORD indices `record[10..11]`
//! for two words; the i64 widening it caused put the `__aeabi_ldivmod` chain into the 16 kHz FOC
//! path (measured ~275 cycles for the q-PI block at zero demand, ~512 stalled, of a 900-cycle
//! ISR budget).
//!
//! The record is GENERIC: gains, divisors, and bounds are the caller's (nothing here assumes the
//! q-PI). The stock inner-current-loop seed values live with their consumer (the commutation
//! crate's q-PI), not here.

/// PI record (recovered stock contract; halfword indices preserved as named fields).
///
/// The integral clamp fields are seeded INVERTED relative to their names in the stock record:
/// `int_max` holds the NEGATIVE value and is used as the LOW bound; `int_min` holds the POSITIVE
/// value and is used as the HIGH bound. [`pi_step`] clamps BY VALUE, not by field name; the field
/// names preserve the recovered record layout.
///
/// The divisors are the stock record's zero-extended 16-bit values: POSITIVE, `1..=65535`
/// (`pid_compute_clamped` reads them as `ushort`). A zero divisor panics (as it always did); a
/// negative divisor is outside the recovered contract.
#[derive(Clone, Copy, Debug)]
pub struct PiRecord {
    /// record[0]: proportional gain.
    pub kp: i32,
    /// record[1]: proportional divisor (positive; unsigned 16-bit in the stock record).
    pub kp_divisor: i32,
    /// record[2]: integral gain. `ki == 0` clears the accumulator each step.
    pub ki: i32,
    /// record[3]: integral divisor (positive; unsigned 16-bit in the stock record).
    pub ki_divisor: i32,
    /// record[4]: output clamp, low.
    pub out_min: i32,
    /// record[5]: output clamp, high.
    pub out_max: i32,
    /// record[6..7] (bytes +12..+16, ONE 32-bit word): seeded NEGATIVE in the stock record; used
    /// as the LOW accumulator bound.
    pub int_max: i32,
    /// record[8..9] (bytes +16..+20, ONE 32-bit word): seeded POSITIVE in the stock record; used
    /// as the HIGH accumulator bound.
    pub int_min: i32,
    /// record[10..11] (bytes +20..+24, ONE 32-bit word): integral accumulator. Stored 32-bit;
    /// the accumulate step's intermediate is 64-bit (see [`pi_accumulate`]).
    pub accumulator: i32,
}

/// Steps 1-2 of the recovered PI step: integrate `e * ki` and clamp BY VALUE.
///
/// The sum is evaluated in 64-bit exactly as the stock code does (`adds/adc` in the decompile),
/// so the bound compare is exact even when `accumulator + e*ki` momentarily exceeds 32 bits; the
/// stored result is the clamped value, which fits 32 bits because the bounds do.
#[inline]
pub fn pi_accumulate(e: i32, record: &mut PiRecord) {
    if record.ki == 0 {
        // Step 1: clear the accumulator and skip integration.
        record.accumulator = 0;
    } else {
        // Step 2: accumulate in 64-bit, then clamp by value into [int_max, int_min].
        let acc = record.accumulator as i64 + (e as i64) * (record.ki as i64);
        // Exact branch form from the recovered contract (int_min = positive HIGH bound,
        // int_max = negative LOW bound):
        //   if int_min >= acc: accumulator = acc if acc >= int_max else int_max
        //   else: accumulator = int_min
        record.accumulator = if record.int_min as i64 >= acc {
            if acc >= record.int_max as i64 {
                acc as i32
            } else {
                record.int_max
            }
        } else {
            record.int_min
        };
    }
}

/// Steps 3-4 of the recovered PI step: the output from the CURRENT accumulator and the error.
///
/// `out = accumulator / Ki_divisor + (e * Kp) / Kp_divisor` (32-bit integer divides, toward
/// zero, the stock form), then clamp into `[out_min, out_max]` and return as `i16`. The sum
/// wraps on 32-bit overflow (the pre-narrowing code truncated its 64-bit sum to 32 bits before
/// the clamp; `wrapping_add` preserves that behavior bit-exactly).
#[inline]
pub fn pi_output(e: i32, record: &PiRecord) -> i16 {
    let i_term = record.accumulator / record.ki_divisor;
    let p_term = (e * record.kp) / record.kp_divisor;
    let out = i_term.wrapping_add(p_term);
    out.clamp(record.out_min, record.out_max) as i16
}

/// One PI step with anti-windup (recovered stock step order). Returns the clamped int16 output
/// and mutates the record's accumulator.
///
/// 1. `ki == 0`: clear the accumulator and skip integration.
/// 2. Else accumulate `e * ki` (64-bit intermediate) and clamp BY VALUE into
///    `[int_max, int_min]` (low = the negative field, high = the positive field; the exact
///    recovered branch form). See [`pi_accumulate`].
/// 3. `out = accumulator / ki_divisor + (e * kp) / kp_divisor` (32-bit divides, toward zero).
/// 4. Clamp into `[out_min, out_max]`, return as `i16`. See [`pi_output`].
#[inline]
pub fn pi_step(setpoint: i32, measured: i32, record: &mut PiRecord) -> i16 {
    let e = setpoint - measured;
    pi_accumulate(e, record);
    pi_output(e, record)
}

#[cfg(test)]
mod tests {
    use super::{pi_accumulate, pi_output, pi_step, PiRecord};

    /// The recovered stock inner-current-loop record, as TEST DATA (provenance: the Declassyfied
    /// contract's section-3.1 seed; the production seed const belongs to the commutation q-PI,
    /// slice 4). 0xF0002000 as signed 32-bit is -268427264.
    fn ref_record() -> PiRecord {
        const INT_LOW: i32 = 0xF000_2000u32 as i32; // -268427264 (negative; LOW bound)
        PiRecord {
            kp: 100,
            kp_divisor: 0x400,
            ki: 0x32,
            ki_divisor: 0x2000,
            out_min: -32767,
            out_max: 32767,
            int_max: INT_LOW,
            int_min: -INT_LOW,
            accumulator: 0,
        }
    }

    #[test]
    fn pi_accumulator_grows_by_e_times_ki() {
        // setpoint 0, measured swept; e = -measured. accumulator grows by e*Ki (Ki = 50).
        let mut rec = ref_record();
        assert_eq!(rec.ki, 50);
        assert_eq!(rec.kp_divisor, 1024);
        let _ = pi_step(0, -10, &mut rec); // e = 10 -> acc += 10*50 = 500
        assert_eq!(rec.accumulator, 500);
        let _ = pi_step(0, -10, &mut rec); // acc += 500 -> 1000
        assert_eq!(rec.accumulator, 1000);
    }

    #[test]
    fn pi_output_formula() {
        // out = accumulator/8192 + (e*Kp)/1024.
        let mut rec = ref_record();
        let out = pi_step(0, -200, &mut rec); // e = 200 ; acc = 200*50 = 10000
                                              // i_term = 10000/8192 = 1 ; p_term = (200*100)/1024 = 19 ; out = 20.
        assert_eq!(rec.accumulator, 10000);
        assert_eq!(out, 20);
    }

    #[test]
    fn pi_antiwindup_holds_at_positive_high_bound() {
        // Large positive error repeatedly; the accumulator clamps at +268427264 (int_min, the
        // positive HIGH bound, by VALUE), not the negative rail.
        let mut rec = ref_record();
        for _ in 0..1000 {
            let _ = pi_step(1_000_000, 0, &mut rec);
        }
        assert_eq!(
            rec.accumulator, 268_427_264,
            "anti-windup HIGH bound (by value)"
        );
    }

    #[test]
    fn pi_antiwindup_holds_at_negative_low_bound() {
        let mut rec = ref_record();
        for _ in 0..1000 {
            let _ = pi_step(-1_000_000, 0, &mut rec);
        }
        assert_eq!(
            rec.accumulator, -268_427_264,
            "anti-windup LOW bound (by value)"
        );
    }

    #[test]
    fn pi_ki_zero_clears_accumulator() {
        let mut rec = ref_record();
        rec.accumulator = 12345;
        rec.ki = 0;
        let _ = pi_step(100, 0, &mut rec);
        assert_eq!(rec.accumulator, 0);
    }

    #[test]
    fn pi_output_clamps_to_record_bounds() {
        // A record with tight output bounds clamps the returned value, independent of the
        // accumulator bounds (step 4 is its own clamp).
        let mut rec = ref_record();
        rec.out_min = -100;
        rec.out_max = 100;
        let out = pi_step(1_000_000, 0, &mut rec);
        assert_eq!(out, 100);
        let out = pi_step(-10_000_000, 0, &mut rec);
        assert_eq!(out, -100);
    }

    #[test]
    fn pi_step_is_accumulate_then_output() {
        // The split helpers ARE the step: running them by hand equals pi_step on the same record.
        let mut a = ref_record();
        let mut b = ref_record();
        for (sp, m) in [(0, 1000), (500, -200), (0, 0), (-32767, 32767)] {
            let out_step = pi_step(sp, m, &mut a);
            let e = sp - m;
            pi_accumulate(e, &mut b);
            let out_split = pi_output(e, &b);
            assert_eq!(out_step, out_split);
            assert_eq!(a.accumulator, b.accumulator);
        }
    }

    // ============================================================================================
    // Equivalence against the pre-narrowing implementation (the i64-record pi_step this module
    // shipped before the stock-width fix). The old step is reproduced here verbatim as the
    // REFERENCE; the property is bit-exact agreement of output AND accumulator over the domain
    // the 32-bit record can express.
    // ============================================================================================

    struct OldPiRecord {
        kp: i32,
        kp_divisor: i32,
        ki: i32,
        ki_divisor: i32,
        out_min: i32,
        out_max: i32,
        int_max: i64,
        int_min: i64,
        accumulator: i64,
    }

    /// The pre-narrowing `pi_step`, verbatim (i64 accumulator/bounds, `__aeabi_ldivmod` on
    /// target). Kept ONLY as the equivalence oracle.
    fn old_pi_step(setpoint: i32, measured: i32, record: &mut OldPiRecord) -> i16 {
        let e = setpoint - measured;
        if record.ki == 0 {
            record.accumulator = 0;
        } else {
            let acc = record.accumulator + (e as i64) * (record.ki as i64);
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
        let i_term = record.accumulator / (record.ki_divisor as i64);
        let p_term = ((e * record.kp) / record.kp_divisor) as i64;
        let out = i_term + p_term;
        (out as i32).clamp(record.out_min, record.out_max) as i16
    }

    fn old_from(rec: &PiRecord) -> OldPiRecord {
        OldPiRecord {
            kp: rec.kp,
            kp_divisor: rec.kp_divisor,
            ki: rec.ki,
            ki_divisor: rec.ki_divisor,
            out_min: rec.out_min,
            out_max: rec.out_max,
            int_max: rec.int_max as i64,
            int_min: rec.int_min as i64,
            accumulator: rec.accumulator as i64,
        }
    }

    /// Deterministic 32-bit LCG (Numerical Recipes constants) for the randomized sweeps.
    fn lcg(state: &mut u32) -> u32 {
        *state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        *state
    }

    #[test]
    fn narrowed_step_matches_old_i64_step_on_the_foc_seed_domain() {
        // The FOC q-PI domain: e = -q_meas with q_meas an i16 (the forward-Park output is
        // mac_q15-saturated), long runs so the accumulator walks its whole clamped range.
        let mut new = ref_record();
        let mut old = old_from(&new);
        let mut s = 0x1234_5678u32;
        for i in 0..200_000u32 {
            // A mix of noise and rail dwell, so the accumulator crosses both clamps.
            let q: i32 = match i % 5 {
                0 => (lcg(&mut s) as i32) >> 16, // full i16 noise
                1 => 32767,                      // + rail (winds down)
                2 => -32767,                     // - rail (winds up)
                3 => (lcg(&mut s) as i32) >> 26, // small noise around zero
                _ => 0,
            };
            let out_new = pi_step(0, q, &mut new);
            let out_old = old_pi_step(0, q, &mut old);
            assert_eq!(out_new, out_old, "output diverged at iteration {i} (q={q})");
            assert_eq!(
                new.accumulator as i64, old.accumulator,
                "accumulator diverged at iteration {i} (q={q})"
            );
        }
    }

    #[test]
    fn narrowed_step_matches_old_i64_step_on_random_generic_records() {
        // Generic records over the domain the narrowed type can express: positive 16-bit
        // divisors (the stock contract's ushort), i32 bounds, gains up to |2^15|.
        let mut s = 0xDEAD_BEEFu32;
        for rec_i in 0..200u32 {
            let ki = (!lcg(&mut s).is_multiple_of(3)) as i32 * ((lcg(&mut s) & 0x7FFF) as i32);
            let bound = 1 + (lcg(&mut s) % 0x3FFF_FFFF) as i32; // positive HIGH bound
            let mut new = PiRecord {
                kp: ((lcg(&mut s) & 0xFFFF) as i32) - 0x8000,
                kp_divisor: 1 + (lcg(&mut s) & 0xFFFE) as i32,
                ki,
                ki_divisor: 1 + (lcg(&mut s) & 0xFFFE) as i32,
                out_min: -((lcg(&mut s) & 0x7FFF) as i32),
                out_max: (lcg(&mut s) & 0x7FFF) as i32,
                int_max: -bound,
                int_min: bound,
                accumulator: 0,
            };
            let mut old = old_from(&new);
            for i in 0..2_000u32 {
                let sp = (lcg(&mut s) as i32) >> 16;
                let m = (lcg(&mut s) as i32) >> 16;
                let out_new = pi_step(sp, m, &mut new);
                let out_old = old_pi_step(sp, m, &mut old);
                assert_eq!(
                    out_new, out_old,
                    "record {rec_i} output diverged at step {i}"
                );
                assert_eq!(
                    new.accumulator as i64, old.accumulator,
                    "record {rec_i} accumulator diverged at step {i}"
                );
            }
        }
    }
}
