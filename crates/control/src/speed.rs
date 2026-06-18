//! Speed/steering outer loop (Section 5) plus the speed-setpoint helper (Section 5.1).
//!
//! Active only when system mode == RUN (3) and run sub-state == 3; otherwise the loop forces its
//! outputs to zero. The fractional coefficients (0.4, 0.6, 0.9996, 1.2) are "(float in original)"
//! and reproduced in Q (flagged). The peer-speed blend weight/sign is link-role/config-dependent;
//! with no link role configured the blend term is zero.

use crate::helpers::clamp_sym;
use fixed::types::I32F32;

/// Persistent speed-loop state.
#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedState {
    /// `acc`: the leaky speed/steer integrator (@ speed-loop integral accumulator).
    pub acc: i32,
    /// Running correction (the slew-limited speed setpoint mirror).
    pub correction: i32,
    /// Direction / brake word.
    pub direction: i32,
}

/// Inputs to the speed/steer loop (Section 5).
#[derive(Clone, Copy, Debug)]
pub struct SpeedInputs {
    /// Base term `B`.
    pub base: i32,
    /// Throttle/steer field `T`.
    pub throttle: i32,
    /// Steering input `s1`.
    pub s1: i32,
    /// Secondary input `s2`.
    pub s2: i32,
    /// Window halfword `W` (unsigned-range; deadband half-width is W/5).
    pub window: i32,
    /// Local motor speed.
    pub local_speed: i32,
    /// Peer motor speed.
    pub peer_speed: i32,
    /// Direction/brake correction field `D`.
    pub d: i32,
    /// Peer-speed blend weight (Q). Zero when no link role is configured.
    pub peer_blend: I32F32,
    /// True when system mode == RUN (3) AND run sub-state == 3.
    pub run_active: bool,
}

impl Default for SpeedInputs {
    fn default() -> Self {
        Self {
            base: 0,
            throttle: 0,
            s1: 0,
            s2: 0,
            window: 0,
            local_speed: 0,
            peer_speed: 0,
            d: 0,
            peer_blend: I32F32::from_num(0),
            run_active: false,
        }
    }
}

/// Section 5: run the speed/steer loop for one tick. Mutates the persistent state.
pub fn speed_loop(inp: &SpeedInputs, st: &mut SpeedState) {
    // Step 5: when sub-state != 3 (or mode != RUN), force loop outputs (acc, direction) to 0.
    if !inp.run_active {
        st.acc = 0;
        st.direction = 0;
        st.correction = 0;
        return;
    }

    // Step 1: proportional speed term. corr = round(B*0.4) + round(T*0.6). (float in original.)
    let corr_b = round_q(I32F32::from_num(inp.base) * I32F32::from_num(0.4)); // FLAGGED: float->Q
    let corr_t = round_q(I32F32::from_num(inp.throttle) * I32F32::from_num(0.6)); // FLAGGED
    let mut correction = corr_b + corr_t;

    // Step 3: peer-speed blend (weighted local-minus-peer). Sign/weight is config-dependent; zero
    // when no link role configured. Added into the steering/correction field.
    let blend = round_q(
        I32F32::from_num(inp.local_speed - inp.peer_speed) * inp.peer_blend,
    );
    correction += blend;
    st.correction = correction;

    // Step 2: conditional integrator (RUN only). thr = W/5 (integer divide; W unsigned).
    let thr = inp.window / 5;
    // Branch by signs/magnitudes of s1, s2 relative to +-thr (center deadband half-width W/5).
    let decayed = I32F32::from_num(st.acc) * I32F32::from_num(0.9996); // FLAGGED: float->Q
    let branch = select_integrator_branch(inp.s1, inp.s2, thr);
    st.acc = match branch {
        IntegratorBranch::Add => round_q(decayed + I32F32::from_num(1.2)), // FLAGGED: 1.2 float->Q
        IntegratorBranch::Subtract => round_q(decayed - I32F32::from_num(1.2)),
        IntegratorBranch::PassThrough => round_q(decayed),
    };

    // Step 4: direction / brake word (RUN only). Compare D against +-30.
    st.direction = if inp.d > 30 {
        -derived_term(inp.d) // positive direction word: negate of a derived term
    } else if inp.d > -30 {
        -inp.d // negated value
    } else {
        0
    };
}

/// Which integrator update branch (Section 5 step 2). The precise left/right branch assignment is
/// role-dependent and flagged un-pinned in the spec (Section 14); the structure (add/sub/pass-
/// through against a +-W/5 deadband) is fixed. Here: outside +thr -> Add, outside -thr -> Subtract,
/// inside the deadband -> PassThrough.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegratorBranch {
    Add,
    Subtract,
    PassThrough,
}

fn select_integrator_branch(s1: i32, s2: i32, thr: i32) -> IntegratorBranch {
    // Magnitude/sign test relative to +-thr. (Structural; left/right assignment to be pinned per
    // board definition, Section 14.)
    let m = if s1.abs() >= s2.abs() { s1 } else { s2 };
    if m > thr {
        IntegratorBranch::Add
    } else if m < -thr {
        IntegratorBranch::Subtract
    } else {
        IntegratorBranch::PassThrough
    }
}

/// The "derived term" negated in the positive-direction case (Section 5 step 4). The exact
/// derivation is role-dependent and structural; the threshold (+-30.0) is the fixed constant.
#[inline]
fn derived_term(d: i32) -> i32 {
    d
}

/// Section 5.1: speed-setpoint helper. For each axis: sp = measured_speed - 2*target, clamped
/// symmetrically to +-0x8000 (+-32768). Returns the two clamped setpoints.
pub fn speed_setpoint(measured: [i32; 2], target: [i32; 2]) -> [i32; 2] {
    const CLAMP: i32 = 0x8000; // 32768
    [
        clamp_sym(measured[0] - 2 * target[0], CLAMP),
        clamp_sym(measured[1] - 2 * target[1], CLAMP),
    ]
}

/// Round-to-nearest (away from zero on .5) of a Q value to i32. The original rounds its float
/// products (`round(...)`); reproduce round, not truncate, for these speed-loop steps.
#[inline]
fn round_q(v: I32F32) -> i32 {
    let half = I32F32::from_num(0.5);
    if v >= 0 {
        (v + half).to_num::<i64>() as i32
    } else {
        (v - half).to_num::<i64>() as i32
    }
}
