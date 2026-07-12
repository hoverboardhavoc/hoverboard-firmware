//! Balance PID (Section 3) and the reference smoothing IIR (Section 3.2 step 7).
//!
//! The pitch->torque math is pure integer arithmetic except step 2's `kd` product and step 7's
//! 0.99/0.01 one-pole IIR, which are genuinely fractional and reproduced in Q
//! (`base::fixed::Fix`, flagged below). The clamp ORDER is load-bearing: derivative two-sided
//! clamp, then raw-demand clamp, then a SECOND clamp to the same bounds, then the IIR.
//!
//! Every float->int conversion models the EABI d2iz (`specs/control.md`, Fixed-point: "every
//! stock float->int conversion models the EABI d2iz unless the binary shows an explicit
//! rounding step"): the decompiled PID (board20 `FUN_08004034`) converts through the shared
//! d2iz helper `FUN_080006e0` at step 1 and step 2, and step 7 narrows the sum to single
//! (`FUN_08000774`) and converts THAT float with the f2iz `FUN_080006ae`; no rounding step
//! exists anywhere in the routine. See [`q_to_int_d2iz`] for the archive divergence this
//! corrects.

use crate::config::pid;
use crate::helpers::{clamp_sym, q_to_int_d2iz};
use base::fixed::Fix;

/// Inputs to the balance PID per tick (Section 3.2). Field names carry the original byte offsets
/// in comments as provenance only.
#[derive(Clone, Copy, Debug)]
pub struct PidInputs {
    /// @0x9c: pitch gyro rate (NOT battery).
    pub bv: i32,
    /// @0x5c: FSM-selected rate coefficient `bk` (2000 in RUN).
    pub bk: i32,
    /// @0x34: fore/aft pitch command `pp` (int16, sign-extended).
    pub pp: i16,
    /// @0x58: FSM-selected proportional gain `kp`.
    pub kp: i32,
    /// @0x60: derivative rate word `pr`.
    pub pr: i32,
    /// @0xb4: derivative coefficient `kd`. Single-precision FLOAT in the original; reproduced
    /// here as a Q-format `base::fixed::Fix` coefficient (no-FPU; software float is banned from
    /// the hot path).
    pub kd: Fix,
    /// @0x68: shaped pitch target / commanded lean `off`.
    pub off: i32,
    /// @0x20: filtered battery voltage in centivolts, the normalization divisor `scale`.
    pub scale: i16,
}

/// Outputs / intermediates of the balance PID (provenance offsets in comments).
#[derive(Clone, Copy, Debug, Default)]
pub struct PidOutputs {
    /// @0x78: combined battery + proportional term.
    pub t78: i32,
    /// @0x7c: derivative term (clamped two-sided to +-30473).
    pub t7c: i32,
    /// @0x80 / @0xa8: the balance PID output (clamped to +-28500).
    pub out: i32,
    /// @0x2a: scheduled secondary scale (800 or 1600).
    pub secondary_scale: i32,
    /// @0xa4: the smoothed balance-PID reference (low 16 bits of the IIR, sign-extended).
    pub smoothed_ref: i16,
}

/// The persistent IIR carry (@0xbc): a high-precision running accumulator between ticks.
/// FLAGGED: float in original -> Q here, WIDER than the stock: the stock narrows the carry to a
/// stored SINGLE each tick (`FUN_08000774` d2f into @0xbc) and converts that float for the i16
/// output; this port keeps the Q32.32 sum as the carry and converts it directly (the archive's
/// recovered shape, within the spec's Q-modeling envelope). The <=1-count stock-vs-port bound
/// is the ANALYTIC argument recorded in the spec's PID fidelity-bounds text (an f32 ulp at the
/// 28500 ceiling is ~0.002, so the narrowing can cross at most one truncation boundary); the
/// f64 reference test bounds the port against the f64 model, not against the stock narrowing.
#[derive(Clone, Copy, Debug, Default)]
pub struct IirCarry {
    pub carry: Fix,
}

/// Section 3.2: the per-tick pitch->torque computation, in exact order. Mutates the IIR carry
/// and returns the intermediates plus the smoothed reference. The state machine mirrors
/// `outputs.smoothed_ref` (@0xa4), NOT `outputs.out` (@0xa8).
pub fn balance_pid(inp: &PidInputs, iir: &mut IirCarry) -> PidOutputs {
    let mut o = PidOutputs::default();

    // Step 1: combined battery + proportional term -> @0x78.
    // 32-bit integer products formed first, then summed and converted to int32 EXACTLY ONCE.
    // (double in original; here the divides are exact-rational since both numerators are integers
    // and the only fractional residue is the /10000 and /100 truncation, which we keep as a single
    // truncate-toward-zero of the summed rational, the d2iz model of the decompile's one
    // FUN_080006e0 call.)
    //
    // t_batt = (bv*bk)/10000.0 ; t_prop = ((int16)pp * kp)/100.0 ; @0x78 = (i32)(t_batt + t_prop)
    // To convert the SUM exactly once with truncation toward zero, compute the sum over a common
    // denominator (10000) and truncate the single rational.
    // bv*bk and pp*kp: the 32-bit products, widened; then the common denominator 10000
    // (t_batt = batt_num/10000, t_prop = prop_num/100 = prop_num*100/10000) so the sum
    // converts once.
    let batt_num = (inp.bv as i64) * (inp.bk as i64);
    let prop_num = (inp.pp as i64) * (inp.kp as i64);
    let sum_over_10000 = batt_num + prop_num * 100;
    // single truncate-toward-zero conversion of the summed value.
    o.t78 = trunc_div(sum_over_10000, pid::BATT_DIVISOR as i64) as i32;

    // Step 2: derivative term -> @0x7c. (single-then-double in original; Q-format here.)
    // convert pr int32->wide, multiply by kd, widen, /100.0, then the d2iz conversion to i32
    // (the decompile's i2f -> single-mul -> f2d -> double-div -> FUN_080006e0 chain).
    // FLAGGED: float-in-original (single-precision kd) -> Q (`base::fixed::Fix`).
    let kd_term = Fix::from_num(inp.pr) * inp.kd;
    let deriv = q_to_int_d2iz(kd_term / Fix::from_num(pid::DERIV_DIVISOR)) as i32;
    // clamp @0x7c symmetrically to +-30473 (two-sided).
    o.t7c = clamp_sym(deriv, pid::DERIV_CLAMP);

    // Step 3: raw demand -> @0x80. Signed integer arithmetic only.
    // @0x80 = ((@0x78 + @0x7c) - off) * 3900 / (int16)scale ; 32-bit add/sub/mul first, signed
    // divide truncates toward zero.
    let numer =
        (((o.t78 as i64) + (o.t7c as i64)) - (inp.off as i64)) * (pid::RAW_NUMERATOR as i64);
    let raw = trunc_div(numer, inp.scale as i64) as i32;

    // Step 4: clamp @0x80 to [-28500, +28500]; copy into @0xa8.
    let clamped1 = clamp_sym(raw, pid::OUTPUT_CLAMP);
    o.out = clamped1;

    // Step 5: scale hysteresis. If (i16)scale < 3500 -> @0x2a = 800; else 1600.
    o.secondary_scale = if (inp.scale as i32) < pid::SCALE_THRESHOLD {
        pid::SECONDARY_SCALE_LOW
    } else {
        pid::SECONDARY_SCALE_HIGH
    };

    // Step 6: second clamp of @0xa8 to the SAME bounds.
    o.out = clamp_sym(o.out, pid::OUTPUT_CLAMP);

    // Step 7: reference smoothing (0.99/0.01 one-pole IIR). (double in original -> Q here.)
    // s = @0xa8 * 0.99 + @0xbc * 0.01 ; store s at @0xbc; low 16 bits (sign-extended) at @0xa4,
    // via the d2iz-model conversion (the stock stores the sum narrowed to single at @0xbc and
    // f2iz-converts that float; see the IirCarry doc for the width nuance).
    let w_new = Fix::from_num(o.out) * Fix::from_num(0.99); // FLAGGED: float-in-original -> Q
    let w_old = iir.carry * Fix::from_num(0.01); // FLAGGED: float-in-original -> Q
    let s = w_new + w_old;
    iir.carry = s;
    // convert to int (toward zero) and store low 16 bits, sign-extended.
    let s_int = q_to_int_d2iz(s);
    o.smoothed_ref = s_int as i16;

    o
}

/// Truncate-toward-zero integer divide for i64 (Rust's `/` already truncates toward zero, but
/// this names the contract and guards against accidental flooring helpers).
#[inline]
fn trunc_div(num: i64, den: i64) -> i64 {
    num / den
}

// `q_to_int_d2iz` (the shared EABI d2iz model for the Q paths) moved to `crate::helpers` when
// the speed loop became its second consumer (slice 4); the correction history lives on it there.
