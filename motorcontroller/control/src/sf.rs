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
//!
//! **The differential half** (`specs/heading-hold.md`) runs in the same pass on the rigid LEADER:
//! the same two position latches folded as a DIFFERENCE are the machine's heading, and a heading
//! error, a heading-rate error and a stick feedforward make the differential torque the steering
//! split ([`crate::wheel_split`]) adds to one wheel and takes from the other. Its signs are RIGHT
//! TURN POSITIVE, the stick's and the split's, so its gains are positive too (section 2).

use crate::config::sf::{
    E_V_CLAMP, FILT_DIV, GAINS, HALF_EDGE, POS_Q, RAMP_CAPS, RAW_NUMERATOR, STICK_DIV, TERM_DIV,
};
use crate::config::turn;
use crate::helpers::clamp_sym;
use base::TORQUE_FULL_SCALE;

/// The `pos_clamp` seam ceiling, edges (`store::CONTROL_POS_CLAMP`, 0x2A; section 6): 450 edges is
/// five wheel turns, 2.6 m on the rover's 165 mm wheel. The clamp is the whole bound on what a
/// held push or a stuck wheel can wind into the position term, so the ceiling keeps that bound a
/// few metres; it also sizes `P` (`450 * 20480` fits `i32` with room, the overflow bound of
/// section 2).
pub const POS_CLAMP_MAX: i16 = 450;

/// The validated position clamp from the stored `store::CONTROL_POS_CLAMP` word: clamped into
/// `0..=`[`POS_CLAMP_MAX`] (the store validates type only, so a hand-poked value is clamped here
/// rather than refused; this is boot or an arm, and the board still has to run). 0 disables the
/// position term.
pub const fn pos_clamp(stored: i16) -> i16 {
    if stored < 0 {
        0
    } else if stored > POS_CLAMP_MAX {
        POS_CLAMP_MAX
    } else {
        stored
    }
}

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
    /// `block.wheel_rate_q12`, this board's wheel, 1/4096 count in the counter's sense.
    pub wheel_rate_q12: i32,
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
    /// The output stage's divisor, centivolts, 0 = UNKNOWN: the effective battery word while the
    /// word is a duty, [`NOMINAL_SCALE`](crate::config::sf::NOMINAL_SCALE) while it is a current
    /// (the orchestrator's choice, `BlockWords::torque_is_current`; `specs/foc-torque.md` section
    /// 4).
    pub scale: i16,
    /// The engagement machine is in RUN (sub-state 3) this pass.
    pub in_run: bool,
    /// The peer wheel's period-based rate (`CyclicState::wheel_rate`, `LinkInbox::peer_wheel_rate`),
    /// 1/4096 count in the SENDER's counter sense, unscaled: the differential half's heading rate
    /// (`specs/heading-hold.md` section 3). The last accepted frame's word, so it updates at the
    /// peer's 125 Hz.
    pub peer_rate_q12: i32,
    /// The effective `DRIVE_CMD.steer`, `+-32767`, positive RIGHT, already neutral once stale.
    pub steer: i16,
    /// This board is the rigid LEADER, the one producer whose output is split into two wheels
    /// (`specs/heading-hold.md` section 5). False on the pivoting producer: the differential half
    /// forms nothing there and its cells stay zero.
    pub leader: bool,
}

/// The configuration the law runs at: the shadow's gain row (`CONTROL_SF_GAINS`, 0x27), the
/// reference row (`CONTROL_REFERENCE`, 0x29) and the position clamp (`CONTROL_POS_CLAMP`, 0x2A).
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
    /// The differential half's configuration (`specs/heading-hold.md` section 6).
    pub turn: TurnConfig,
}

/// The differential half's configuration: the shadow's turn gain row (`CONTROL_TURN_GAINS`, 0x2C),
/// the turn reference row (`CONTROL_TURN_REFERENCE`, 0x2D) and the clamp row
/// (`CONTROL_TURN_CLAMP`, 0x2E, through `control::turn::clamp`). The ranges are the seams'
/// (`specs/heading-hold.md` section 6); the arithmetic here holds for any non-negative `i16`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TurnConfig {
    /// `[gh, ghv, gff]`, the targets the live turn gains ramp toward in RUN.
    pub gains: [i16; turn::GAINS],
    /// Sixteenths of a count per wheel at full stick; 0 is a dead steer axis.
    pub rate_max: i16,
    /// 1/4096 count per tick.
    pub rate_slew: i16,
    /// Word counts, the differential's authority; 0 is no differential at all.
    pub turn_max: i16,
    /// Edges; 0 disables the heading term.
    pub head_clamp: i16,
}

impl TurnConfig {
    /// The inert configuration, every word 0: no differential whatever else is staged.
    pub const OFF: TurnConfig = TurnConfig {
        gains: [0; turn::GAINS],
        rate_max: 0,
        rate_slew: 0,
        turn_max: 0,
        head_clamp: 0,
    };
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
    /// This pass's four terms as torque words (`term_p`, `term_r`, `term_x`, `term_v`): each
    /// through [`word`] on its own, so in the output's unit and clamped at its full scale. The
    /// observation's (section 8); live on every pass, armed or not.
    pub terms: [i16; GAINS],
    /// Filtered heading rate, half the left wheel's forward rate less the right's, 1/4096 count,
    /// right turn positive (`specs/heading-hold.md` section 3). A measurement: never reset.
    pub vh_filt: i32,
    /// The slewed heading-rate reference, 1/4096 count. RUN on a leader only.
    pub v_href: i32,
    /// The heading ERROR `H_ref - H`, 1/20480 edge. RUN on a leader only.
    pub head_err: i32,
    /// The live turn gains `[gh, ghv, gff]`: the shadow's outside RUN, stepping toward it inside.
    pub turn_live: [i16; turn::GAINS],
    /// This pass's heading and heading-rate terms as torque words (`term_h`, `term_hv`), each
    /// through [`word`] on its own: the observation's word 52 (section 8). Live on every leader
    /// pass, armed or not; 0 on the pivoting producer.
    pub turn_terms: [i16; 2],
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

// The differential half's three products at its widest inputs (`specs/heading-hold.md` section
// 2): every gain and the heading clamp at `i16::MAX`, the rate error at its clamp and the
// reference at the full-stick target of the widest `rate_max`. The sum times 3900 fits `i64`, and
// the clamped heading error fits `i32`.
const _: () = {
    let g = i16::MAX as i64;
    let head_max = i16::MAX as i64 * POS_Q as i64;
    let v_href_max = 32767 * i16::MAX as i64 / STICK_DIV as i64;
    let sum = g * head_max + g * E_V_CLAMP as i64 + g * v_href_max;
    assert!(
        sum <= i64::MAX / RAW_NUMERATOR as i64,
        "the differential's numer * 3900 fits i64"
    );
    assert!(
        head_max <= i32::MAX as i64,
        "the clamped heading error fits i32"
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
    // `wrapping_div`, not `/`: the only overflow `/` checks for is `i64::MIN / -1`, which the
    // const assertions above put out of reach, and the check is a panic path in the 250 Hz
    // control task (`specs/panic-free.md`, `tools/panic-reach.py`).
    clamp_sym64(
        (numer * RAW_NUMERATOR as i64).wrapping_div(scale as i64),
        TORQUE_FULL_SCALE as i64,
    ) as i16
}

/// The differential half's three terms in numerator counts (`specs/heading-hold.md` section 2),
/// `[term_h, term_hv, term_ff]`, each truncated toward zero once, as [`terms`] does.
pub fn turn_terms(g: [i16; turn::GAINS], head_err: i32, e_hv: i32, v_href: i32) -> [i64; 3] {
    let s = [head_err as i64, e_hv as i64, v_href as i64];
    core::array::from_fn(|i| g[i] as i64 * s[i] / turn::TERM_DIV[i])
}

/// One pass of the live gain ramp (section 6): each gain steps toward the shadow's by at most its
/// [`RAMP_CAPS`] entry, so a tuning write in RUN reaches the torque at a bounded rate per pass.
pub fn ramp(live: [i16; GAINS], target: [i16; GAINS]) -> [i16; GAINS] {
    ramp_by(live, target, RAMP_CAPS)
}

/// The turn gains' ramp (`specs/heading-hold.md` section 6): [`ramp`]'s rule at
/// [`turn::RAMP_CAPS`].
pub fn turn_ramp(live: [i16; turn::GAINS], target: [i16; turn::GAINS]) -> [i16; turn::GAINS] {
    ramp_by(live, target, turn::RAMP_CAPS)
}

/// Each of `live` stepped toward `target` by at most its `caps` entry.
fn ramp_by<const N: usize>(live: [i16; N], target: [i16; N], caps: [i16; N]) -> [i16; N] {
    core::array::from_fn(|i| {
        let l = live[i] as i32;
        (l + clamp_sym(target[i] as i32 - l, caps[i] as i32)) as i16
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

/// One 250 Hz pass: the fold, the filters, the gain ramps, the references, the law. Returns the
/// torque word, negative forward, and the differential (`specs/heading-hold.md` section 2), word
/// counts, positive turning RIGHT, bounded by `turn_max`: the steering split's input on a leader
/// and 0 everywhere else.
pub fn sf_step(cfg: &SfConfig, inp: &SfInputs, st: &mut SfState) -> (i16, i32) {
    // The fold: twice the mean wheel motion this tick, in half-edges. The mean cancels a turn's
    // differential by construction (section 3). The same two deltas differenced are twice the
    // heading change (`specs/heading-hold.md` section 3), which cancels straight motion.
    let own = wheel_forward(inp.wheel_pos, inp.role_right);
    let peer = wheel_forward(inp.peer_pos, !inp.role_right);
    let d_own = own.wrapping_sub(st.own_prev);
    let d_peer = peer.wrapping_sub(st.peer_prev);
    let dx2 = d_own.wrapping_add(d_peer);
    st.own_prev = own;
    st.peer_prev = peer;

    let v = wheel_forward(inp.wheel_rate_q12, inp.role_right);
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
    st.terms = core::array::from_fn(|i| word(t[i], inp.scale));
    let w = word(t[0] + t[1] + t[2] + t[3], inp.scale);

    let v_peer = wheel_forward(inp.peer_rate_q12, !inp.role_right);
    (w, turn_step(&cfg.turn, inp, st, d_own, d_peer, v, v_peer))
}

/// The differential half (`specs/heading-hold.md` sections 2 to 4), from this tick's forward
/// position deltas and forward rates of the two wheels. LEFT and RIGHT are chosen by
/// `role_right`: the heading is half the left wheel's forward travel less the right's, so a right
/// turn grows it.
fn turn_step(
    cfg: &TurnConfig,
    inp: &SfInputs,
    st: &mut SfState,
    d_own: i32,
    d_peer: i32,
    v_own: i32,
    v_peer: i32,
) -> i32 {
    let (dh2, v_l, v_r) = if inp.role_right {
        (d_peer.wrapping_sub(d_own), v_peer, v_own)
    } else {
        (d_own.wrapping_sub(d_peer), v_own, v_peer)
    };
    // Half the difference in `i64`, so no pair of `i32` rates can overflow it; the half always
    // fits back.
    let v_h = ((v_l as i64 - v_r as i64) / 2) as i32;
    st.vh_filt += (v_h - st.vh_filt) / FILT_DIV;

    if !inp.leader {
        // The pivoting producer: no split, so no differential (section 5). The cells hold zero.
        st.turn_live = cfg.gains;
        st.v_href = 0;
        st.head_err = 0;
        st.turn_terms = [0; 2];
        return 0;
    }

    st.turn_live = if inp.in_run {
        turn_ramp(st.turn_live, cfg.gains)
    } else {
        cfg.gains
    };

    if inp.in_run {
        let target = inp.steer as i32 * cfg.rate_max as i32 / STICK_DIV;
        st.v_href += clamp_sym(target - st.v_href, cfg.rate_slew as i32);
        let h = st.head_err as i64 + st.v_href as i64 - dh2 as i64 * HALF_EDGE as i64;
        st.head_err = clamp_sym64(h, cfg.head_clamp as i64 * POS_Q as i64) as i32;
    } else {
        st.v_href = 0;
        st.head_err = 0;
    }
    let e_hv = clamp_sym(st.v_href - st.vh_filt, E_V_CLAMP);

    let t = turn_terms(st.turn_live, st.head_err, e_hv, st.v_href);
    st.turn_terms = [word(t[0], inp.scale), word(t[1], inp.scale)];
    // Applied in RUN only: outside it the engagement machine's setpoint is 0, and a nonzero
    // differential split around a zero would pivot an unengaged machine (section 4, "Armed but
    // not engaged"). The terms above stay live for the disarmed sign gate.
    if !inp.in_run || inp.scale == 0 {
        return 0;
    }
    let lim = cfg.turn_max as i64;
    clamp_sym64(
        ((t[0] + t[1] + t[2]) * RAW_NUMERATOR as i64).wrapping_div(inp.scale as i64),
        if lim < 0 { 0 } else { lim },
    ) as i32
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod turn_tests;
