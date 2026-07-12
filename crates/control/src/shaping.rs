//! Pitch-target shaping (Section 4): the commanded lean the balance loop balances to. Runs
//! per tick BEFORE the balance PID. Exact order preserved.

use crate::config::shaping;
use crate::helpers::clamp_sym;

/// Persistent shaping state between ticks.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShapingState {
    /// The running shaped target (last tick's value), used by the slew limiter.
    pub last_target: i32,
    /// Previous steering input, latched for edge use next tick (step 6).
    pub prev_steer: i16,
}

/// Inputs to the shaper (Section 4). `roll_a`/`roll_b` are the local/peer roll-mirror words.
#[derive(Clone, Copy, Debug)]
pub struct ShapingInputs {
    /// Local roll-mirror word.
    pub roll_a: i16,
    /// Peer roll-mirror word (degrades to the local-only case when no peer; pass roll_a).
    pub roll_b: i16,
    /// Steering input.
    pub steer: i16,
    /// Left/right role flag: when true, the steer sign is inverted (role-dependent, Section 4).
    /// Does NOT affect step 1 (fb is the absolute value either way).
    pub role_right: bool,
}

/// Section 4: produce the shaped pitch target. Mutates the persistent state and returns the
/// int32 shaped target consumed by the balance PID the same tick.
pub fn shape_pitch_target(inp: &ShapingInputs, st: &mut ShapingState) -> i32 {
    // Step 1: differential feedback term (absolute value).
    // fb = abs( (roll_a - roll_b)/10 ): widen to i32, signed divide toward zero, then abs.
    let diff = (inp.roll_a as i32) - (inp.roll_b as i32);
    let fb = (diff / 10).abs(); // always non-negative

    // Step 2: base target with gain. base = fb*18 + 3500. Because fb >= 0, base >= 3500.
    let base = fb * shaping::SPEED_TO_LEAN_GAIN + shaping::CENTER_LEAN_OFFSET;

    // Step 3: steering contribution. steer_term = trunc_toward_zero((double)(steer*3) * 0.5),
    // net steer*1.5 TRUNCATED toward zero. (float in original: the decompiled chain is
    // i2d -> ldexp(-1), an exact *0.5 exponent step -> d2iz (board20 decompile, shaper
    // FUN_08006524 @7330 calling FUN_080006e0), and d2iz TRUNCATES: |x| < 1.0 returns 0, the
    // general path is a plain mantissa shift with no rounding increment, sign applied after.
    // Declassyfied section 9 states this truncation for the steering-scale step; its section-4
    // "round_to_int" label is a source-side contradiction recorded in specs/control.md (d).
    // The f64 reference test sweeps the whole i16 range.) Role flips the sign.
    let mut steer = inp.steer as i32;
    if inp.role_right {
        steer = -steer; // role-dependent steer sign (Section 4)
    }
    // trunc(steer*3 / 2) == trunc(steer*1.5); then clamp symmetrically to +-base and store as
    // the running shaped target.
    let steer_term = trunc_half(steer * 3);
    let mut target = clamp_sym(steer_term, base);

    // Step 4: absolute clamp to +-7000.
    target = clamp_sym(target, shaping::ABS_CLAMP);

    // Step 5: slew limit, per-tick change at most +-250.
    let delta = target - st.last_target;
    let limited = if delta > shaping::SLEW_LIMIT {
        st.last_target + shaping::SLEW_LIMIT
    } else if delta < -shaping::SLEW_LIMIT {
        st.last_target - shaping::SLEW_LIMIT
    } else {
        target
    };
    st.last_target = limited;

    // Step 6: latch the steering input for edge use next tick.
    st.prev_steer = inp.steer;

    limited
}

/// Truncate-toward-zero halving (`x / 2`): the EABI d2iz model of the stock's
/// `(double)(steer*3) * 0.5` conversion (i2d -> ldexp(-1) -> d2iz; the *0.5 is a lossless
/// exponent step, so the d2iz truncation of `x/2` is all that remains, and Rust integer
/// division already truncates toward zero). `pub(crate)` so the exhaustive f64 reference test
/// sweeps the exact function the shaper calls.
///
/// DELIBERATE STOCK-CORRECTNESS CORRECTION (slice-2 audit): the archive carried
/// round-half-away-from-zero here (matching the stock doc's section-4 "round_to_int" label),
/// which diverges from the binary by 1 LSB on every odd steer; the d2iz truncation is what the
/// silicon runs.
#[inline]
pub(crate) fn trunc_half(x: i32) -> i32 {
    x / 2
}
