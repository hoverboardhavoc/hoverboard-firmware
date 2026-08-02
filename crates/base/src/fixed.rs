//! Fixed-point Q-format conventions (the no-FPU numeric basis).
//!
//! The whole firmware runs on a Cortex-M3 with no hardware float, so every numeric layer uses these
//! fixed-point types from the `fixed` crate and the same test discipline. The aliases keep call
//! sites reading uniformly:
//!
//! - [`Fix`] (I32F32) for precision-critical state: attitude, control integrators.
//! - [`Out`] (I16F16) for outputs.
//! - [`Q15`] (I1F15) for the FOC vector math.
//!
//! Trig is `cordic` (e.g. `asin` for attitude), never `libm` or the FPU; `cordic` is re-exported
//! here so math layers reach it through `base`.

pub use cordic;

/// Precision-critical state (attitude, control integrators). 32 integer + 32 fractional bits.
pub type Fix = fixed::types::I32F32;

/// Outputs. 16 integer + 16 fractional bits.
pub type Out = fixed::types::I16F16;

/// FOC vector math. 1 integer + 15 fractional bits (signed Q15).
pub type Q15 = fixed::types::I1F15;

/// The **one** [`Fix`] division body in the image. Every `Fix / Fix` on the 250 Hz control path
/// goes through here rather than writing `a / b` at the site.
///
/// `<I32F32 as Div>::div` pre-shifts the numerator by the 32-bit fraction width, which pushes it
/// past 64 bits, and LLVM has no narrower lowering for the result: each `/` expands **inline** to a
/// full 128-bit clz-normalise plus shift-subtract loop, ~730 B of flash, and no shared
/// `__divti3`-style helper is ever linked. Measured on the `717efe0` image: 12 static expansions
/// across 4 source sites, 8,748 B, all of it inside the placed `.hotcode` window
/// (`specs/wide-division-menu.md`). `#[inline(never)]` collapses those 12 instantiations to 1 and
/// is worth **-5,408 B** of flashed span, measured.
///
/// `#[inline(never)]` is therefore load-bearing semantics here, not a hint: without it LLVM
/// re-inlines the body at every call site and the whole lever evaporates with the build still green.
///
/// `.hotcode` placement is the other half, and it is not optional. Outlined but left to link order,
/// the body measured at `0x0800a45e`, i.e. **above** the GD32F1x0's 32 KiB zero-wait flash boundary
/// (`crates/firmware/memory.x`), which would make all 9 divisions per control tick fetch their body
/// at 2 wait states with no prefetch. It costs 728 B (measured; an earlier draft said 708) of the 9,344 B the window reclaims. The
/// attribute travels with the item, so it cannot go stale on a rename the way a `memory.x` name
/// anchor can.
///
/// Semantics are the `/` operator's, by construction: same expression, one instantiation. The
/// bit-exactness property test below asserts it anyway, since this is a codegen-class change.
#[inline(never)]
#[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
pub fn div(a: Fix, b: Fix) -> Fix {
    a / b
}

/// Test-only helper: assert that a fixed-point value agrees with an `f64` reference oracle within
/// `tol`. This is the discipline every math layer reuses: compute a quantity in both the
/// fixed-point path and in `f64`, then assert agreement within a stated tolerance.
///
/// `value` is any `fixed` type (all of [`Fix`], [`Out`], [`Q15`] qualify); it is converted to
/// `f64` for the comparison. I32F32 has more fractional bits than an `f64` mantissa can hold, so
/// the conversion is lossy by design, which is fine for a tolerance check.
#[cfg(test)]
pub fn assert_close<F>(value: F, reference: f64, tol: f64)
where
    F: fixed::traits::Fixed,
{
    let v: f64 = value.to_num();
    let diff = (v - reference).abs();
    assert!(
        diff <= tol,
        "assert_close failed: value={v} reference={reference} diff={diff} tol={tol}",
    );
}

#[cfg(test)]
mod tests {
    use super::{assert_close, Fix, Out, Q15};
    use fixed::traits::ToFixed;

    #[test]
    fn aliases_round_trip_through_f64() {
        // Each alias should hold its reference within its own resolution.
        assert_close(Fix::from_num(1.5), 1.5, 1e-9);
        assert_close(Out::from_num(-2.25), -2.25, 1.0 / 65536.0);
        assert_close(Q15::from_num(0.5), 0.5, 1.0 / 32768.0);
    }

    #[test]
    fn assert_close_admits_quantization_error() {
        // 0.1 is not exactly representable in Q15; it must still land within one LSB.
        let q = Q15::from_num(0.1);
        assert_close(q, 0.1, 1.0 / 32768.0);
    }

    #[test]
    #[should_panic]
    fn assert_close_rejects_a_real_miss() {
        assert_close(Out::from_num(1.0), 2.0, 1e-6);
    }

    #[test]
    fn cordic_asin_matches_f64() {
        // The attitude filter leans on cordic::asin over Fix; check it against f64 asin.
        let x = 0.5_f64;
        let got = super::cordic::asin(Fix::from_num(x));
        assert_close(got, x.asin(), 1e-3);
    }

    #[test]
    fn to_fixed_trait_is_reachable() {
        // Sanity: the `fixed` conversion traits are usable through the dependency.
        let v: Fix = 3.0_f64.to_fixed();
        assert_close(v, 3.0, 1e-9);
    }

    /// Deterministic xorshift64 (nonzero seed), the repo's fixed-seed sampling idiom
    /// (`link::framer`): reproducible across runs and platforms, no wall-clock or OS randomness.
    fn xorshift(state: &mut u64) -> u64 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        *state = x;
        x
    }

    /// **The S3 oracle** (`specs/wide-division-menu.md`): the outlined [`super::div`] is bit-exact
    /// with the `/` operator it replaces, over 300k random pairs plus structured corners.
    ///
    /// The operator is the oracle, exactly as in the FOC-fix pattern: the shipping image before this
    /// slice wrote `a / b` at each of the four sites, so agreeing with `/` on every bit IS the
    /// no-behaviour-change proof. `to_bits()` comparison, never `f64`: `I32F32` carries more
    /// fractional bits than an `f64` mantissa holds, so a float comparison would hide low-bit drift.
    #[test]
    fn outlined_div_is_bit_exact_with_the_operator() {
        // Structured corners first: signs, unity, the truncation-toward-zero boundary, the extremes.
        let corners = [
            Fix::ZERO,
            Fix::from_bits(1),
            Fix::from_bits(-1),
            Fix::ONE,
            -Fix::ONE,
            Fix::from_num(0.5),
            Fix::from_num(-0.5),
            Fix::from_num(1e-9),
            Fix::from_num(16384),
            Fix::from_num(-16384),
            Fix::MAX,
            Fix::MIN + Fix::from_bits(1),
        ];
        for a in corners {
            for b in corners {
                if b == Fix::ZERO {
                    continue; // division by zero panics identically either way; not a bit question
                }
                // Skip the one overflow corner: MIN/-1 (and MAX-class quotients) panic in debug
                // through BOTH paths, so it is not a divergence, just an untestable pair here.
                if a.to_bits().checked_div(b.to_bits()).is_none() {
                    continue;
                }
                if a.checked_div(b).is_none() {
                    continue;
                }
                assert_eq!(
                    super::div(a, b).to_bits(),
                    (a / b).to_bits(),
                    "outlined div diverged from `/` at a={a} b={b}"
                );
            }
        }

        // 300k random pairs across the whole magnitude range the filter and PID reach.
        let mut s: u64 = 0x5EED_0D1F_F1CE_D111;
        let mut checked = 0u32;
        for _ in 0..300_000 {
            // Vary the exponent as well as the mantissa so the shift-subtract loop is exercised at
            // every normalisation depth, not just near-unit operands.
            let a_shift = (xorshift(&mut s) % 63) as u32;
            let b_shift = (xorshift(&mut s) % 63) as u32;
            let a = Fix::from_bits((xorshift(&mut s) as i64) >> a_shift);
            let b = Fix::from_bits((xorshift(&mut s) as i64) >> b_shift);
            if b == Fix::ZERO {
                continue;
            }
            // Only compare where the quotient is representable; an overflowing quotient panics
            // through both paths identically and says nothing about the outlining.
            if a.checked_div(b).is_none() {
                continue;
            }
            assert_eq!(
                super::div(a, b).to_bits(),
                (a / b).to_bits(),
                "outlined div diverged from `/` at a={a} b={b}"
            );
            checked += 1;
        }
        // Guard the guard: a sampler that filtered everything out would pass vacuously.
        assert!(checked > 200_000, "too few live samples: {checked}");
    }
}
