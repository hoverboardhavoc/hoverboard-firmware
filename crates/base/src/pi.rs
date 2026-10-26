//! The PI regulator the FOC arm's two current loops run on (`specs/foc-torque.md` section 5).
//!
//! Reshaped for its one consumer. The stock record this module used to carry (`PiRecord`, the
//! recovered `pid_compute_clamped` layout with its integral bounds seeded inverted relative to
//! their names, and the i64 equivalence oracle that pinned its narrowing) existed for the recovered
//! stock q-axis PI, and that arm is gone: the torque-mode arm regulates both rotor axes with one
//! shared set of gains and one accumulator per axis, so the gains and the state are separate here.
//! The control crate's PID and outer loop that once shared this record went with
//! `specs/state-feedback.md`.
//!
//! It lives in `base` for the layering reason it always did: the commutation crate consumes it and
//! must not depend on anything above `base`.
//!
//! **No 64-bit divide anywhere.** The accumulate step is evaluated in 64 bits so its clamp compare
//! is exact (a multiply-accumulate and a compare on this core, no call), and both output divides
//! are 32-bit hardware `SDIV`. The i64 widening of the stock record put the `__aeabi_ldivmod` chain
//! into the 16 kHz period ISR and cost 275 cycles (`specs/silicon-queue.md`, "FOC ISR-BUDGET
//! ROUND", 2026-07-27); this step is read twice per period.
//!
//! **Defined for every input, never a panic** (`specs/panic-free.md` requirements 1 and 3), because
//! the gains are a live bench knob in RAM (`commutation::foc::FOC_GAINS`) that a session can poke
//! to anything: the clamps are `max().min()` (no `i32::clamp`, whose `min <= max` assert cannot
//! fold over RAM-resident bounds), a zero divisor drops its term, the divides are `wrapping_div`
//! (so `i32::MIN / -1` cannot trap), and the error and the proportional product wrap rather than
//! overflow-check.

/// The gains and bounds of one PI step, shared by every axis that runs it (`specs/foc-torque.md`
/// section 5: the q and d current loops use one `PiGains` and hold one accumulator each).
///
/// `repr(C)` so the field order is the layout: six consecutive words, the order the bench knob
/// `commutation::foc::FocGains` mirrors.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PiGains {
    /// Proportional gain, applied as `e * kp / kp_divisor`.
    pub kp: i32,
    /// Proportional divisor. Positive by intent; zero drops the proportional term.
    pub kp_divisor: i32,
    /// Integral gain per step: the accumulator grows by `e * ki`.
    pub ki: i32,
    /// Integral divisor: the integral term is `acc / ki_divisor`. Zero drops the integral term.
    pub ki_divisor: i32,
    /// Symmetric output clamp, positive: the output is held in `-out_max..=out_max`. Taken as its
    /// nearest end of `0..=32767`, so the `i16` result can neither wrap nor be the `-32768`
    /// sentinel the stock saturations reserve.
    pub out_max: i32,
    /// Symmetric accumulator clamp, positive: the accumulator is held in `-int_max..=int_max`.
    pub int_max: i32,
}

/// One PI step: `e = reference - measured`, the accumulator integrates `e * ki` and is clamped to
/// `+-int_max` (64-bit intermediate, stored 32-bit), and the output is `acc / ki_divisor +
/// e * kp / kp_divisor` (32-bit divides, toward zero) clamped to `+-out_max`.
///
/// The accumulator is updated BEFORE the output reads it, so the first step from a zeroed
/// accumulator already carries one period of integral (`specs/foc-torque.md` section 9 pins what
/// that is at the start gains). `ki == 0` leaves the accumulator where it is (clamped): the stock
/// record cleared it on `ki == 0`, and that rule went with the record.
#[inline]
pub fn pi_step(reference: i32, measured: i32, g: &PiGains, acc: &mut i32) -> i16 {
    let e = reference.wrapping_sub(measured);
    // The bound pair in 64 bits, so negating it cannot overflow even for a poked `i32::MIN`. For
    // any `int_max >= 0` the result is in `-int_max..=int_max` and fits the `i32` it is stored in;
    // for a negative (inverted) one `max().min()` returns `int_max` itself, which also fits.
    let hi = g.int_max as i64;
    let sum = *acc as i64 + e as i64 * g.ki as i64;
    *acc = sum.max(-hi).min(hi) as i32;
    let out = div_defined(*acc, g.ki_divisor)
        .wrapping_add(div_defined(e.wrapping_mul(g.kp), g.kp_divisor));
    // The output bound held inside the `i16` before it is used, so a poked `out_max` above 32,767
    // cannot wrap the cast into a voltage of the other sign. Constant bounds: no assert to fold.
    let hi = g.out_max.max(0).min(i16::MAX as i32);
    out.max(-hi).min(hi) as i16
}

/// `num / divisor`, defined for every pair instead of panicking on two of them.
///
/// `/` has two panicking inputs, a zero divisor and `i32::MIN / -1`, and both are a panic stub
/// reachable from `period_isr`, where a spin holds the bridge at its last duties with no controller
/// (`specs/panic-free.md`, requirement 3: a defaulting branch, never an `unwrap` and never a
/// `debug_assert` that leaves the release path unchecked). A zero divisor yields 0, the neutral
/// term; `i32::MIN / -1` wraps to `i32::MIN` instead of trapping.
#[inline]
fn div_defined(num: i32, divisor: i32) -> i32 {
    if divisor == 0 {
        0
    } else {
        num.wrapping_div(divisor)
    }
}

#[cfg(test)]
mod tests {
    use super::{pi_step, PiGains};

    /// The check-value gains: the stock inner-current-loop record's gain pair and divisors (kp
    /// 100 / 1024, ki 50 / 8192), carried over as plain numbers so the arithmetic is pinned against
    /// the figures it was always pinned against. The bounds are the full `i16` output and an
    /// accumulator that cannot command past it.
    fn gains() -> PiGains {
        PiGains {
            kp: 100,
            kp_divisor: 1024,
            ki: 50,
            ki_divisor: 8192,
            out_max: 32767,
            int_max: 32767 * 8192,
        }
    }

    #[test]
    fn the_check_values() {
        // e = 200: acc = 200 * 50 = 10000; i_term = 10000 / 8192 = 1, p_term = 20000 / 1024 = 19.
        let mut acc = 0;
        assert_eq!(pi_step(200, 0, &gains(), &mut acc), 20);
        assert_eq!(acc, 10_000);
        // The error is reference - measured, so the same error from the other side is the same step.
        let mut acc2 = 0;
        assert_eq!(pi_step(0, -200, &gains(), &mut acc2), 20);
        assert_eq!(acc2, 10_000);
        // A second step integrates again before the output reads it: acc 20000 -> i_term 2.
        assert_eq!(pi_step(200, 0, &gains(), &mut acc), 21);
        assert_eq!(acc, 20_000);
        // Toward zero on both divides: e = -200 gives -1 + -19.
        let mut acc = 0;
        assert_eq!(pi_step(-200, 0, &gains(), &mut acc), -20);
        assert_eq!(acc, -10_000);
    }

    #[test]
    fn the_accumulator_clamps_at_plus_and_minus_int_max_by_value() {
        let g = PiGains {
            int_max: 1_000_000,
            ..gains()
        };
        let mut acc = 0;
        for _ in 0..1000 {
            let _ = pi_step(30_000, 0, &g, &mut acc);
        }
        assert_eq!(acc, 1_000_000);
        for _ in 0..1000 {
            let _ = pi_step(-30_000, 0, &g, &mut acc);
        }
        assert_eq!(acc, -1_000_000);
        // A step whose 32-bit sum would overflow is still clamped exactly (the 64-bit intermediate).
        let g = PiGains {
            ki: i32::MAX,
            int_max: i32::MAX,
            ..gains()
        };
        let mut acc = i32::MAX - 1;
        let _ = pi_step(65_534, 0, &g, &mut acc);
        assert_eq!(acc, i32::MAX);
        let _ = pi_step(-65_534, 0, &g, &mut acc);
        let _ = pi_step(-65_534, 0, &g, &mut acc);
        assert_eq!(acc, -i32::MAX);
    }

    #[test]
    fn the_output_clamps_at_plus_and_minus_out_max() {
        let g = PiGains {
            out_max: 100,
            ..gains()
        };
        let mut acc = 0;
        assert_eq!(pi_step(1_000_000, 0, &g, &mut acc), 100);
        let mut acc = 0;
        assert_eq!(pi_step(-1_000_000, 0, &g, &mut acc), -100);
        // The output clamp is its own: the accumulator is bounded by int_max, not by out_max.
        assert_eq!(acc, -50_000_000);
        // A poked out_max past the i16 cannot wrap the result into the other sign.
        let g = PiGains {
            out_max: 40_000,
            kp: 1024,
            ki: 0,
            ..gains()
        };
        let mut acc = 0;
        assert_eq!(pi_step(50_000, 0, &g, &mut acc), 32767);
        assert_eq!(pi_step(-50_000, 0, &g, &mut acc), -32767);
        // A negative (inverted) bound is the zero output, not a wrap.
        let g = PiGains {
            out_max: -5,
            ..gains()
        };
        let mut acc = 0;
        assert_eq!(pi_step(1_000, 0, &g, &mut acc), 0);
    }

    #[test]
    fn a_zero_divisor_drops_its_term_instead_of_panicking() {
        // ki_divisor 0: i_term 0, p_term 200 * 100 / 1024 = 19.
        let g = PiGains {
            ki_divisor: 0,
            ..gains()
        };
        let mut acc = 1 << 20;
        assert_eq!(pi_step(200, 0, &g, &mut acc), 19);
        // kp_divisor 0: p_term 0, i_term (8192 * 7 + 10000) / 8192 = 8.
        let g = PiGains {
            kp_divisor: 0,
            ..gains()
        };
        let mut acc = 8192 * 7;
        assert_eq!(pi_step(200, 0, &g, &mut acc), 8);
    }

    #[test]
    fn min_over_minus_one_returns_instead_of_trapping() {
        // The other panicking divide. e = i32::MIN - 0, e * kp = i32::MIN, / -1 wraps to i32::MIN,
        // and the output clamp holds it at -32767. What matters is that the step returns.
        let g = PiGains {
            kp: 1,
            kp_divisor: -1,
            ki: 0,
            ..gains()
        };
        let mut acc = 0;
        assert_eq!(pi_step(i32::MIN, 0, &g, &mut acc), -32767);
        // The integral divide too: an int_max of i32::MIN holds the accumulator AT i32::MIN (the
        // inverted pair returns its high end), and i32::MIN / -1 wraps.
        let g = PiGains {
            kp: 0,
            ki_divisor: -1,
            int_max: i32::MIN,
            ..gains()
        };
        let mut acc = 0;
        assert_eq!(pi_step(0, 0, &g, &mut acc), -32767);
        assert_eq!(acc, i32::MIN);
    }

    #[test]
    fn ki_zero_leaves_the_accumulator_alone() {
        // The stock record CLEARED its accumulator on ki == 0; that rule went with the record. Here
        // ki == 0 integrates nothing, so the accumulator holds and keeps contributing its term.
        let g = PiGains { ki: 0, ..gains() };
        let mut acc = 8192 * 3;
        assert_eq!(pi_step(200, 0, &g, &mut acc), 3 + 19);
        assert_eq!(acc, 8192 * 3);
        assert_eq!(pi_step(-200, 0, &g, &mut acc), 3 - 19);
        assert_eq!(acc, 8192 * 3);
    }
}
