//! The four-term state-feedback balance law (`specs/state-feedback.md`): torque from pitch, pitch
//! rate, position error and speed error, each gain multiplying its own measured term and nothing
//! else, with the stick moving a slewed speed reference and the position reference it integrates.
//!
//! OURS, not stock: the form of US5971091A (`T = K1 theta + K2 theta' + K3 x + K4 x'`) and of
//! Ascento, with the starting gains from an LQR design on the rover model (`tools/lqr-gains.py`,
//! section 7). There is no integrator anywhere: a standing pitch error parks the machine
//! `gp * p_err / (100 * gx)` edges from the reference, and a held push is bounded by `pos_clamp`.
//!
//! **Signs** (section 2). `pitch_word` and `pitch_rate` are NEGATIVE for a forward lean, the torque
//! word is NEGATIVE for forward drive, and position and speed are FORWARD POSITIVE after
//! [`wheel_forward`]. With both errors taken as reference minus measured, all four gains are
//! positive. A forward push gives a NEGATIVE position and speed term, forward torque, which tips
//! the body back so the pitch term then drives it back: the inverted pendulum's non-minimum-phase
//! response, and the sign the LQR produces.

use crate::config::pid::{OUTPUT_CLAMP, RAW_NUMERATOR};
use crate::config::sf::{
    E_V_CLAMP, FILT_DIV, GAINS, HALF_EDGE, POS_Q, RAMP_CAPS, STICK_DIV, TERM_DIV,
};
use crate::helpers::clamp_sym;

/// This board's wheel rate or position in its FORWARD-POSITIVE sense, from the block's word in the
/// counter's own sense and the board's side (`store::CONTROL_SIDE`, `BlockWords::role_right`).
///
/// # Why the mirror is the SIDE's and not `motor.direction`'s
///
/// The rotor-side edge counter counts hall transitions, so its sign is the wheel's physical
/// rotation and nothing else: `motor.direction` picks which of the two mirror-image electrical
/// sequences a positive demand drives, and the counter never sees it. What fixes the sign of
/// machine-forward in the count is which wheel the board is bolted to, because the right wheel
/// and the left wheel of one machine turn opposite ways for the same forward. Measured on the
/// rover master, the RIGHT wheel, 2026-10-24 (`specs/bench-evidence/2026-10-24/speed-loop-sign`):
/// forward reads NEGATIVE, on a board holding `motor.direction = 1`. The left wheel is the mirror
/// by construction and has not been measured (`specs/state-feedback.md` gate 1 reads it). The
/// peer is the other wheel of the same machine, so its words go through the OPPOSITE side.
///
/// A sign error here is positive feedback (the derivation this replaced, which read the byte, was
/// exactly that on the rover: a forward push read as backward, the loop leaned into it, and the
/// machine drove off), so the first gate is a hand push with the board DISARMED.
#[inline]
pub const fn wheel_forward(word: i32, right_wheel: bool) -> i32 {
    if right_wheel {
        -word
    } else {
        word
    }
}

/// What the law reads on one tick, in the block's own senses (the mirror is applied here).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SfInputs {
    /// `block.pitch_word`, centidegrees, negative forward.
    pub pitch_word: i16,
    /// `block.pitch_rate`, rad/s x 10000, negative forward.
    pub pitch_rate: i32,
    /// `block.wheel_rate_q`, this board's wheel, 1/4096 count in the counter's sense.
    pub wheel_rate_q: i32,
    /// `block.wheel_pos`, this board's wheel, edges in the counter's sense, wrapping.
    pub wheel_pos: i32,
    /// The peer wheel's unwrapped position (`LinkInbox::peer_wheel_pos`), edges in the SENDER's
    /// counter sense. It moves only on a tick that accepted a frame, so the fold sees zero peer
    /// motion on every other tick.
    pub peer_pos: i32,
    /// This board is the right wheel (`BlockWords::role_right`).
    pub role_right: bool,
    /// The effective `DRIVE_CMD.value`, `+-32767`, already neutral once stale.
    pub value: i16,
    /// The effective battery word, centivolts, 0 = UNKNOWN.
    pub scale: i16,
    /// The engagement machine is in RUN (sub-state 3) this pass.
    pub in_run: bool,
}

/// The configuration the law runs at: the shadow's gain row (`CONTROL_SF_GAINS`, 0x79), the
/// reference row (`CONTROL_REFERENCE`, 0x7B) and the position clamp (`CONTROL_POS_CLAMP`, 0x7C).
/// The ranges are the seams' (section 6); the arithmetic here holds for any `i16`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SfConfig {
    /// `[gp, gr, gx, gv]`, the targets the live gains ramp toward in RUN.
    pub gains: [i16; GAINS],
    /// Sixteenths of a count at full stick.
    pub speed_max: i16,
    /// 1/4096 count per tick.
    pub speed_slew: i16,
    /// Edges; 0 disables the position term.
    pub pos_clamp: i16,
}

/// The law's carries. The reference cells are zeroed on every tick the engagement machine is not
/// in RUN (the recovered integrator's rule, so disarm, the tilt latch, the fallen dwell, comms loss
/// and the soft start are all resets); the filter is a measurement and the position latches are
/// re-latched every tick, so the first RUN tick folds one tick's motion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SfState {
    /// Filtered forward wheel speed, 1/4096 count.
    pub v_filt: i32,
    /// The slewed speed reference, 1/4096 count. RUN only.
    pub v_ref: i32,
    /// The position ERROR `x_ref - x`, 1/20480 edge. RUN only.
    pub pos_err: i32,
    /// The live gains the terms multiply: the shadow's outside RUN, stepping toward it inside.
    pub live: [i16; GAINS],
    own_prev: i32,
    peer_prev: i32,
}

// The four products and the output product at the widest inputs the law can be handed: every gain
// and the position clamp at `i16::MAX`, the pitch word at `i16::MIN`, the rate at `i32::MAX` (the
// gyro's +-500 deg/s is 87,266), the speed error at its clamp. Formed in `i64`, and the sum of the
// four untruncated products times 3900 still fits, so no step of the law can overflow.
const _: () = {
    let g = i16::MAX as i64;
    let pos_max = i16::MAX as i64 * POS_Q as i64;
    let sum = g * 32768 + g * i32::MAX as i64 + g * pos_max + g * E_V_CLAMP as i64;
    assert!(
        sum <= i64::MAX / RAW_NUMERATOR as i64,
        "numer * 3900 fits i64"
    );
    assert!(
        pos_max <= i32::MAX as i64,
        "the clamped position error fits i32"
    );
};

/// The four terms in numerator counts (section 2), each truncated toward zero once.
pub fn terms(g: [i16; GAINS], pitch: i16, rate: i32, pos_err: i32, e_v: i32) -> [i64; GAINS] {
    let s = [pitch as i64, rate as i64, pos_err as i64, e_v as i64];
    core::array::from_fn(|i| g[i] as i64 * s[i] / TERM_DIV[i])
}

/// The torque word from the summed numerator: `numer * 3900 / scale`, clamped to the output
/// stage's full scale, and 0 on an UNKNOWN battery word.
pub fn word(numer: i64, scale: i16) -> i16 {
    if scale == 0 {
        return 0;
    }
    clamp_sym64(
        numer * RAW_NUMERATOR as i64 / scale as i64,
        OUTPUT_CLAMP as i64,
    ) as i16
}

/// One pass of the live gain ramp (section 6): each gain steps toward the shadow's by at most its
/// [`RAMP_CAPS`] entry, so a tuning write in RUN reaches the torque at a bounded rate per pass.
pub fn ramp(live: [i16; GAINS], target: [i16; GAINS]) -> [i16; GAINS] {
    core::array::from_fn(|i| {
        let l = live[i] as i32;
        (l + clamp_sym(target[i] as i32 - l, RAMP_CAPS[i] as i32)) as i16
    })
}

/// [`clamp_sym`] in `i64`, written out rather than `Ord::clamp`, which carries a panic for a
/// negative bound.
fn clamp_sym64(v: i64, lim: i64) -> i64 {
    if v > lim {
        lim
    } else if v < -lim {
        -lim
    } else {
        v
    }
}

/// One 250 Hz pass: the fold, the filter, the gain ramp, the reference, the law. Returns the torque
/// word, negative forward.
pub fn sf_step(cfg: &SfConfig, inp: &SfInputs, st: &mut SfState) -> i16 {
    // The fold: twice the mean wheel motion this tick, in half-edges. The mean cancels a turn's
    // differential by construction (section 3).
    let own = wheel_forward(inp.wheel_pos, inp.role_right);
    let peer = wheel_forward(inp.peer_pos, !inp.role_right);
    let dx2 = own
        .wrapping_sub(st.own_prev)
        .wrapping_add(peer.wrapping_sub(st.peer_prev));
    st.own_prev = own;
    st.peer_prev = peer;

    let v = wheel_forward(inp.wheel_rate_q, inp.role_right);
    st.v_filt += (v - st.v_filt) / FILT_DIV;

    st.live = if inp.in_run {
        ramp(st.live, cfg.gains)
    } else {
        cfg.gains
    };

    if inp.in_run {
        let target = inp.value as i32 * cfg.speed_max as i32 / STICK_DIV;
        st.v_ref += clamp_sym(target - st.v_ref, cfg.speed_slew as i32);
        let p = st.pos_err as i64 + st.v_ref as i64 - dx2 as i64 * HALF_EDGE as i64;
        st.pos_err = clamp_sym64(p, cfg.pos_clamp as i64 * POS_Q as i64) as i32;
    } else {
        st.v_ref = 0;
        st.pos_err = 0;
    }
    let e_v = clamp_sym(st.v_ref - st.v_filt, E_V_CLAMP);

    let t = terms(st.live, inp.pitch_word, inp.pitch_rate, st.pos_err, e_v);
    word(t[0] + t[1] + t[2] + t[3], inp.scale)
}

#[cfg(test)]
mod tests;
