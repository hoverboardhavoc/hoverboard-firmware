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
}
