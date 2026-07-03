//! The recovered PI-regulator record and step (`specs/commutation.md`, "Fixed-point formats").
//!
//! Recovered verbatim from the archived `control::helpers` (`archive/accumulated-build`,
//! commit `74b7773`); the record layout and the clamp ORDER / anti-windup orientation are part of
//! the recovered stock contract, not incidental. It lives in `base` because two independent layers
//! consume it: the commutation crate's q-axis current PI (now) and the Phase-D balance loop
//! (later); neither should depend on the other for a shared primitive.
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
#[derive(Clone, Copy, Debug)]
pub struct PiRecord {
    /// record[0]: proportional gain.
    pub kp: i32,
    /// record[1]: proportional divisor (unsigned 16-bit in the stock record).
    pub kp_divisor: i32,
    /// record[2]: integral gain. `ki == 0` clears the accumulator each step.
    pub ki: i32,
    /// record[3]: integral divisor (unsigned 16-bit in the stock record).
    pub ki_divisor: i32,
    /// record[4]: output clamp, low.
    pub out_min: i32,
    /// record[5]: output clamp, high.
    pub out_max: i32,
    /// record[6..7]: seeded NEGATIVE in the stock record; used as the LOW accumulator bound.
    pub int_max: i64,
    /// record[8..9]: seeded POSITIVE in the stock record; used as the HIGH accumulator bound.
    pub int_min: i64,
    /// record[10..11]: integral accumulator, 64-bit wide.
    pub accumulator: i64,
}

/// One PI step with anti-windup (recovered stock step order). Returns the clamped int16 output
/// and mutates the record's accumulator.
///
/// 1. `ki == 0`: clear the accumulator and skip integration.
/// 2. Else accumulate `e * ki` in 64-bit and clamp BY VALUE into `[int_max, int_min]`
///    (low = the negative field, high = the positive field; the exact recovered branch form).
/// 3. `out = accumulator / ki_divisor + (e * kp) / kp_divisor` (integer divides, toward zero).
/// 4. Clamp into `[out_min, out_max]`, return as `i16`.
pub fn pi_step(setpoint: i32, measured: i32, record: &mut PiRecord) -> i16 {
    let e = setpoint - measured;

    if record.ki == 0 {
        // Step 1: clear the accumulator and skip integration.
        record.accumulator = 0;
    } else {
        // Step 2: accumulate in 64-bit, then clamp by value into [int_max, int_min].
        let acc = record.accumulator + (e as i64) * (record.ki as i64);
        // Exact branch form from the recovered contract (int_min = positive HIGH bound,
        // int_max = negative LOW bound):
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
    (out as i32).clamp(record.out_min, record.out_max) as i16
}

#[cfg(test)]
mod tests {
    use super::{pi_step, PiRecord};

    /// The recovered stock inner-current-loop record, as TEST DATA (provenance: the Declassyfied
    /// contract's section-3.1 seed; the production seed const belongs to the commutation q-PI,
    /// slice 4). 0xF0002000 as signed 32-bit is -268427264.
    fn ref_record() -> PiRecord {
        const INT_LOW: i64 = 0xF000_2000u32 as i32 as i64; // -268427264 (negative; LOW bound)
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
}
