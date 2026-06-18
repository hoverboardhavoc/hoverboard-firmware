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

    // Step 3: steering contribution. steer_term = round(steer * 3 * 0.5) = round(steer*1.5).
    // (float in original; reproduced as integer round-half of steer*3.) Role flips the sign.
    let mut steer = inp.steer as i32;
    if inp.role_right {
        steer = -steer; // role-dependent steer sign (Section 4)
    }
    let steer_term = round_half(steer * 3); // round(steer*3/2) == round(steer*1.5)
    // clamp symmetrically to +-base, store as the running shaped target.
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

/// Round-half-away-from-zero of `x/2` (the `* 0.5` with rounding). For `round(steer*1.5)` we pass
/// `steer*3` and halve with rounding.
#[inline]
fn round_half(x: i32) -> i32 {
    if x >= 0 {
        (x + 1) / 2
    } else {
        (x - 1) / 2
    }
}
