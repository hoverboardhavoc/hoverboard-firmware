//! Balance state machine (Section 7): gating, soft-start envelope, slew, and the final torque
//! setpoint output (Section 7.3). Owns the balance run sub-state (0..3).
//!
//! The FSM is the SOLE writer of the live balance-PID gain fields @0x58/@0x5c/@0x60 (Section
//! 7.2.1): every `setup <- {x,y,z}` means `kp<-x; bk<-y; pr<-z`, taking effect on the next PID
//! tick. The torque envelope mirrors the SMOOTHED reference (@0xa4), never the raw PID output.

use crate::config::{envelope, GainProfile, GainTriple, ARMING_SEED_ORIENT_NZ, STANDBY_SET};
use crate::helpers::{iabs, shr_round_to_zero};

/// The balance run sub-state (Section 7.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubState {
    /// 0: not balancing; envelope decays to zero.
    Idle = 0,
    /// 1: ARMING / soft-start; envelope ramps up from zero.
    Arming = 1,
    /// 2: engaged, alternate (standby) profile.
    AltEngaged = 2,
    /// 3: RUN; full balancing.
    Run = 3,
}

/// Counter caps (Section 7.2).
const PROMOTE_DEBOUNCE_CAP: i32 = 0x8ACE;
const PICKUP_COUNTER_CAP: i32 = 100;
const WINDDOWN_DEBOUNCE_CAP: i32 = 0x8ACE;

/// Engage-condition inputs and gating consumed by the FSM (Section 7.2). Fault/engage gating is
/// owned by safety.md and consumed here.
#[derive(Clone, Copy, Debug, Default)]
pub struct FsmInputs {
    /// Orientation / role field (offset 0x26): selects the engage-parameter set and the upright
    /// threshold branch. != 0 takes the alternate (board-20) path.
    pub orientation_nz: bool,
    /// The smoothed balance-PID reference (@0xa4, Section 3.2 step 7) mirrored into the output.
    pub smoothed_ref: i32,
    /// Scaled upright reference for the active branch (reference * 100.0). Branch 1 (orient==0)
    /// tests |scaled_ref_1| > 2499; branch 2 (orient!=0) tests |scaled_ref_2| > 7499.
    pub scaled_ref_1: i32,
    pub scaled_ref_2: i32,
    /// Positive-edge gating field: engage requires gating_field > 500.
    pub gating_field: i32,
    /// Rider present.
    pub rider_present: bool,
    /// Latched fault flags (any true inhibits engage and forces immediate IDLE where noted).
    pub over_current: bool,
    pub stall: bool,
    pub comms_loss: bool,
    pub tilt: bool,
    /// Relevant enable bytes clear (true means clear/permitting engage).
    pub enable_bytes_clear: bool,
    /// Power/enable byte set.
    pub power_enable: bool,
    /// A stop byte (Section 7 aborts).
    pub stop_byte: bool,
    /// Debounced tilt/pickup field (int16); negative triggers the pickup counter (RUN).
    pub pickup_field: i16,
    /// Two enable bytes clear -> step-off/wind-down debounce increments (RUN).
    pub winddown_enables_clear: bool,
    /// The condition byte for the ALT(2) promotion debounce (clear -> increment).
    pub promote_condition_clear: bool,
    /// Sub-state-2 reference inputs: @0x9c, @0x34, @0x36.
    pub ref_9c: i32,
    pub ref_34: i32,
    pub ref_36: i32,
    /// Measured-current/feedback word `fb` for the next-tick delta (Section 7.3 step 4).
    pub feedback_fb: i32,
}

/// Persistent FSM state.
#[derive(Clone, Copy, Debug)]
pub struct FsmState {
    pub sub_state: SubState,
    pub env: i32,
    /// The output mirror (pre-envelope torque field). Holds @0xa4 in sub-states 1/2/3, 0 in IDLE.
    pub out_mirror: i32,
    /// The live balance-PID gain fields @0x58/@0x5c/@0x60 (the "setup words").
    pub gains: GainTriple,
    pub balancing_active: bool,
    /// Debounce counters.
    pub promote_counter: i32,
    pub pickup_counter: i32,
    pub winddown_counter: i32,
    /// Shadow sub-state (telemetry, Section 7.3 step 1).
    pub sub_state_shadow: SubState,
    /// Stored feedback for the tracking-error delta (Section 7.3 step 4).
    pub prev_fb: i32,
    pub tracking_delta: i32,
    /// The final torque setpoint (the contract boundary to the inner loop).
    pub torque_setpoint: i16,
}

impl Default for FsmState {
    fn default() -> Self {
        Self {
            sub_state: SubState::Idle,
            env: 0,
            out_mirror: 0,
            gains: STANDBY_SET,
            balancing_active: false,
            promote_counter: 0,
            pickup_counter: 0,
            winddown_counter: 0,
            sub_state_shadow: SubState::Idle,
            prev_fb: 0,
            tracking_delta: 0,
            torque_setpoint: 0,
        }
    }
}

/// One FSM tick (Section 7). `profile` is the rider-gated profile selected in Section 6 (its
/// coeff1/2/3 are the (coeff1,coeff2,coeff3) the FSM copies on a full promote). Returns the
/// written torque setpoint (also stored in `st.torque_setpoint`).
pub fn fsm_step(inp: &FsmInputs, profile: &GainProfile, st: &mut FsmState) -> i16 {
    let run_triple = profile.as_triple();

    match st.sub_state {
        SubState::Idle => idle(inp, profile, st),
        SubState::Arming => arming(inp, profile, run_triple, st),
        SubState::AltEngaged => alt_engaged(inp, run_triple, st),
        SubState::Run => run(inp, profile, run_triple, st),
    }

    // Section 7.3: final torque-setpoint output (every tick, all sub-states).
    final_output(inp, st)
}

fn idle(inp: &FsmInputs, profile: &GainProfile, st: &mut FsmState) {
    // Zero the output mirror.
    st.out_mirror = 0;

    // Decay the envelope toward zero by 1000/tick (symmetric, +-1000 deadband).
    if st.env >= 0x3E9 {
        st.env -= envelope::DECAY;
    } else if st.env <= -1000 {
        st.env += envelope::DECAY;
    } else {
        st.env = 0;
    }

    // Engage test: upright check for the active branch + all gating conditions.
    let upright = if inp.orientation_nz {
        iabs(inp.scaled_ref_2) > 0x1D4B // 7499 (alternate branch)
    } else {
        iabs(inp.scaled_ref_1) > 0x9C3 // 2499 (first branch)
    };
    let faults_clear =
        !inp.over_current && !inp.stall && !inp.comms_loss && !inp.tilt && inp.enable_bytes_clear;
    let engage = upright
        && inp.gating_field > 500
        && inp.rider_present
        && faults_clear
        && inp.power_enable;

    if engage {
        st.sub_state = SubState::Arming;
        // clear the two integrator/envelope state words, set active, zero envelope.
        st.balancing_active = true;
        st.env = 0;
        st.promote_counter = 0;
        st.winddown_counter = 0;
        // seed engage params by orientation (Section 7.2).
        // orient == 0: setup <- @0x4C, @0x50, 0 (coeff1, coeff2, 0).
        // orient != 0: setup <- 1000, 300, 0.
        st.gains = if inp.orientation_nz {
            ARMING_SEED_ORIENT_NZ
        } else {
            GainTriple::new(profile.coeff1, profile.coeff2, 0)
        };
    }
}

fn arming(inp: &FsmInputs, profile: &GainProfile, run_triple: GainTriple, st: &mut FsmState) {
    // Mirror the SMOOTHED reference (@0xa4) into the output mirror.
    st.out_mirror = inp.smoothed_ref;

    // Abort to IDLE if (comms-loss OR stop OR over-current) AND orientation != 0.
    if (inp.comms_loss || inp.stop_byte || inp.over_current) && inp.orientation_nz {
        st.balancing_active = false;
        st.sub_state = SubState::Idle;
        st.promote_counter = 0;
        st.winddown_counter = 0;
        return;
    }

    // Ramp the envelope up by 200/tick until the cap 0x6F54.
    if st.env < 0x6F55 {
        st.env += envelope::RAMP_UP;
    }
    if st.env >= envelope::CAP {
        st.env = envelope::CAP;
        // promote by orientation.
        if inp.orientation_nz {
            // sub-state <- 2; setup <- {50, 20, 0}.
            st.sub_state = SubState::AltEngaged;
            st.gains = STANDBY_SET;
        } else {
            // sub-state <- 3 (RUN); setup <- (coeff1, coeff2, coeff3).
            st.sub_state = SubState::Run;
            st.gains = run_triple;
        }
    }
    let _ = profile;
}

fn alt_engaged(inp: &FsmInputs, run_triple: GainTriple, st: &mut FsmState) {
    // ref = round( (@0x9C * 0x50)/10000.0 + ((@0x34*7 + @0x36*-6) * 0x32)/100 )
    let batt = (inp.ref_9c as i64) * 0x50; // *80
    let mix = (((inp.ref_34 * 7) + (inp.ref_36 * -6)) as i64) * 0x32; // *50
    // sum over a common denominator 10000 then round-to-nearest of the rational.
    let sum_over_10000 = batt + mix * 100; // mix/100 == mix*100/10000
    let mut refv = round_div(sum_over_10000, 10000) as i32;

    // clamp ref into [-28500, 0x6F54] = [-28500, +28500], store at @0xa4 (overwriting), copy to mirror.
    refv = clamp(refv, -envelope::CAP, envelope::CAP);
    st.out_mirror = refv;

    // recompute the envelope from the magnitude helper on the reference.
    st.env = iabs(refv);

    // abort to IDLE on (comms-loss OR stop OR over-current).
    if inp.comms_loss || inp.stop_byte || inp.over_current {
        st.balancing_active = false;
        st.sub_state = SubState::Idle;
        st.promote_counter = 0;
        st.winddown_counter = 0;
        return;
    }

    // promotion debounce: while the condition byte is clear, increment (cap 0x8ACE); once > 5,
    // promote to RUN with run engage params. Reset when set.
    if inp.promote_condition_clear {
        if st.promote_counter < PROMOTE_DEBOUNCE_CAP {
            st.promote_counter += 1;
        }
        if st.promote_counter > 5 {
            st.sub_state = SubState::Run;
            st.gains = run_triple;
            st.env = envelope::CAP;
        }
    } else {
        st.promote_counter = 0;
    }
}

fn run(inp: &FsmInputs, profile: &GainProfile, run_triple: GainTriple, st: &mut FsmState) {
    // Mirror the smoothed reference (@0xa4) into the output mirror.
    st.out_mirror = inp.smoothed_ref;

    // Immediate stop: comms-loss OR over-current.
    if inp.comms_loss || inp.over_current {
        st.balancing_active = false;
        st.sub_state = SubState::Idle;
        st.promote_counter = 0;
        st.winddown_counter = 0;
        return;
    }

    // Pickup / step-off detection.
    if inp.pickup_field < 0 {
        if st.pickup_counter < PICKUP_COUNTER_CAP {
            st.pickup_counter += 1;
        }
        if st.pickup_counter > 0x14 {
            st.sub_state = SubState::Idle;
        }
    } else {
        st.pickup_counter = 0;
    }

    // Envelope maintenance: while orientation != 0, ramp up by 200/tick toward 0x6F54 and clamp.
    if inp.orientation_nz {
        if st.env < 0x6F55 {
            st.env += envelope::RAMP_UP;
        }
        if st.env > envelope::CAP {
            st.env = envelope::CAP;
        }
    }

    // Step-off / wind-down debounce: while two enable bytes clear, increment (cap 0x8ACE); once
    // > 10, disengage. Reset when either enable byte is set.
    if inp.winddown_enables_clear {
        if st.winddown_counter < WINDDOWN_DEBOUNCE_CAP {
            st.winddown_counter += 1;
        }
        if st.winddown_counter > 10 {
            if inp.orientation_nz {
                // sub-state <- 2 with alternate setup {50, 20, 0}.
                st.sub_state = SubState::AltEngaged;
                st.gains = STANDBY_SET;
            } else {
                // sub-state <- 0 (IDLE), reseed idle ref/env set (setup <- coeff1, coeff2, 0).
                st.sub_state = SubState::Idle;
                st.gains = GainTriple::new(profile.coeff1, profile.coeff2, 0);
            }
            st.env = envelope::CAP;
        }
    } else {
        st.winddown_counter = 0;
    }
    let _ = run_triple;
}

/// Section 7.3: final torque-setpoint output, unconditionally each tick.
fn final_output(inp: &FsmInputs, st: &mut FsmState) -> i16 {
    // 1. shadow the sub-state.
    st.sub_state_shadow = st.sub_state;

    // Default / safe state for IDLE: the mirror already zeroed in idle(); for any unexpected
    // case force zero. (IDLE/default hold zero mirror per Section 7.3.)
    if st.sub_state == SubState::Idle {
        st.out_mirror = 0;
    }

    // 2. envelope clamp (soft-start amplitude limit). Clamp out symmetrically to +-env.
    let env = st.env;
    let mut out = st.out_mirror;
    if out > env {
        out = env;
    } else if out < -env {
        out = -env;
    }

    // 3. write the torque setpoint = low 16 bits of the enveloped out.
    st.torque_setpoint = out as i16;

    // 4. update the PID delta term for next tick: delta = (torque_setpoint - fb) >> 6 with
    // round-toward-zero correction.
    let fb = inp.feedback_fb;
    st.prev_fb = fb;
    let x = (st.torque_setpoint as i32) - fb;
    st.tracking_delta = shr_round_to_zero(x, 6);

    st.torque_setpoint
}

/// Section 8.4 clamp, local alias for readability.
#[inline]
fn clamp(v: i32, lo: i32, hi: i32) -> i32 {
    crate::helpers::clamp(v, lo, hi)
}

/// Round-to-nearest integer divide (round half away from zero) for the sub-state-2 `round(...)`.
#[inline]
fn round_div(num: i64, den: i64) -> i64 {
    if (num >= 0) == (den >= 0) {
        (num + den / 2) / den
    } else {
        (num - den / 2) / den
    }
}
