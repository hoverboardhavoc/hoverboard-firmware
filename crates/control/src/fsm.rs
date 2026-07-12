//! Balance engagement machine (Section 7 / `FUN_08003be8`): gating, soft-start envelope, and
//! the final torque-setpoint output (Section 7.3). Owns the balance run sub-state (0..3), the
//! byte Phase C's fault latch consumes as `a_substate`.
//!
//! REBUILT TO THE BINARY (slice-5 audit ruling; `specs/control.md` (c) is the folded contract,
//! whose source note records how stock section 7.2 and the archive both mis-described the
//! machine, including the safety-class INVERTED upright window). The FSM is the SOLE writer of
//! the torque setpoint and of the live gain QUADRUPLE: the base-coefficient float cell (Q here)
//! plus the balance-PID triple @0x58/@0x5c/@0x60; every `setup <- {base; x,y,z}` takes effect
//! on the next PID tick. The torque envelope mirrors the SMOOTHED reference (@0xa4), never the
//! raw PID output.
//!
//! Ordering is the binary's: no early returns inside the ARMING / sub-2 / RUN arms; the
//! abort/stop checks run AFTER the promote/wind-down blocks in their arms, so same-tick
//! combinations resolve by write order exactly as the silicon does (pinned by tests).

use crate::config::{
    envelope, fsm as fsmc, GainProfile, GainTriple, ARMING_SEED_ORIENT_NZ, STANDBY_SET,
};
use crate::helpers::{clamp, iabs, q_to_int_d2iz, shr_round_to_zero};
use base::fixed::Fix;

/// The balance run sub-state (Section 7.1). The discriminants ARE the wire values; `as i8`
/// yields the `a_substate` byte the Phase-C fault latch consumes.
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

/// Engage-condition inputs and gating consumed by the FSM (Section 7.2, parameterized per the
/// `ModeInputs` precedent). Fault/engage gating is owned by safety.md and consumed here.
#[derive(Clone, Copy, Debug, Default)]
pub struct FsmInputs {
    /// Orientation / role field (offset 0x26): selects the engage-parameter set and the upright
    /// threshold branch. != 0 takes the alternate path.
    pub orientation_nz: bool,
    /// The ONE shared upright reference (the `DAT_08003fc4` float cell; float -> Q). The
    /// x100.0f scale and the d2iz conversion are modeled INSIDE (spec (c)); the binary scales
    /// this same cell twice per tick to the same value.
    pub upright_ref: Fix,
    /// The master pre-gate byte == 0 (true = clear = the upright+engage evaluation runs at
    /// all). Producer unknown (inputs work); the binary skips the whole evaluation when set.
    pub pre_gate_clear: bool,
    /// The smoothed balance-PID reference (@0xa4, Section 3.2 step 7) mirrored into the output.
    pub smoothed_ref: i32,
    /// The shared gating/pickup halfword (the ONE cell @+4 of its record): engage requires
    /// `> 500` (IDLE); the RUN pickup counter counts while it is `< 0`.
    pub gating_field: i16,
    /// Rider present.
    pub rider_present: bool,
    /// Latched fault flags (any true inhibits engage; comms/over-current force IDLE in RUN).
    pub over_current: bool,
    /// See [`FsmInputs::over_current`].
    pub stall: bool,
    /// See [`FsmInputs::over_current`].
    pub comms_loss: bool,
    /// See [`FsmInputs::over_current`].
    pub tilt: bool,
    /// Relevant enable bytes clear (true means clear/permitting engage; the aggregate of the
    /// binary's orientation-conditioned byte soup, per the parameterized-inputs contract).
    pub enable_bytes_clear: bool,
    /// Power/enable byte set.
    pub power_enable: bool,
    /// A stop byte (the ARMING / sub-2 aborts).
    pub stop_byte: bool,
    /// Two enable bytes clear -> step-off/wind-down debounce increments (RUN).
    pub winddown_enables_clear: bool,
    /// The sub-2 promotion condition byte: while HELD (nonzero in the binary), the promotion
    /// debounce increments; clear resets it. (The archive's `promote_condition_clear` name had
    /// the polarity backwards; renamed.)
    pub promote_condition: bool,
    /// Sub-2 reference input @0x9c (i32, the battery-scaled term's source).
    pub ref_9c: i32,
    /// Sub-2 mix input s16@0x34 (a HALFWORD in the binary; the archive widened it to i32).
    pub ref_34: i16,
    /// Sub-2 mix input s16@0x36 (halfword, as above).
    pub ref_36: i16,
    /// Measured-feedback halfword `fb` for the next-tick delta (Section 7.3 step 4; a SHORT
    /// read in the binary, the archive widened it to i32).
    pub feedback_fb: i16,
}

/// Persistent FSM state (the binary's cells, offsets in the field docs).
#[derive(Clone, Copy, Debug)]
pub struct FsmState {
    /// @5: the balance run sub-state.
    pub sub_state: SubState,
    /// @0xac: the soft-start/authority envelope.
    pub env: i32,
    /// @0xc0: the output mirror (pre-envelope torque field). @0xa4 in sub-states 1/3, the sub-2
    /// reference in 2, zero in IDLE.
    pub out_mirror: i32,
    /// @0x58/@0x5c/@0x60: the live balance-PID gain triple (the "setup words").
    pub gains: GainTriple,
    /// The live base-coefficient float cell (the quadruple's fourth word; float -> Q). Written
    /// on every seed/promote/wind-down: the field@0x48 copy (0.4f, both profiles) on orient==0
    /// paths and the sub-2 promote, 3.0f on the orient!=0 engage, 0.4f (`DAT_08003fb8`,
    /// decoded 0x3ECCCCCD) on orient!=0 promote/wind-down. Consumer unidentified (spec (e)).
    pub base_coeff: Fix,
    /// The balancing-active flag.
    pub balancing_active: bool,
    /// @0x84: opaque state word; the FSM only ever ZEROES it (engage + every abort/stop).
    /// Consumer unknown ("the two integrator/envelope state words", stock 7.2).
    pub state_word_84: i32,
    /// @0x88: the second opaque state word, zeroed with @0x84.
    pub state_word_88: i32,
    /// @0x8c: opaque word zeroed at ARMING cap-entry and RUN wind-down; never otherwise
    /// touched here.
    pub state_word_8c: i32,
    /// @0x90: the sub-2 promotion debounce counter (cap 0x8ACE; also zeroed at RUN wind-down).
    pub promote_counter: i32,
    /// @0x94: the RUN wind-down debounce counter (cap 0x8ACE; also zeroed at sub-2 promote).
    pub winddown_counter: i32,
    /// @7: the RUN pickup counter (a byte in the binary; cap 100 keeps the i32 equivalent).
    pub pickup_counter: i32,
    /// @6: shadow sub-state (telemetry, Section 7.3 step 1).
    pub sub_state_shadow: SubState,
    /// @0x44: the latched feedback halfword (Section 7.3 step 4).
    pub prev_fb: i16,
    /// @0x46: the `>> 6` tracking delta HALFWORD (the archive widened it to i32).
    pub tracking_delta: i16,
    /// The final torque setpoint (the atomic i16 write; the contract boundary, +-28500).
    pub torque_setpoint: i16,
}

impl Default for FsmState {
    fn default() -> Self {
        Self {
            sub_state: SubState::Idle,
            env: 0,
            out_mirror: 0,
            gains: STANDBY_SET,
            base_coeff: Fix::ZERO,
            balancing_active: false,
            state_word_84: 0,
            state_word_88: 0,
            state_word_8c: 0,
            promote_counter: 0,
            winddown_counter: 0,
            pickup_counter: 0,
            sub_state_shadow: SubState::Idle,
            prev_fb: 0,
            tracking_delta: 0,
            torque_setpoint: 0,
        }
    }
}

/// One FSM tick (Section 7). `profile` is the rider-gated profile selected in Section 6 (its
/// coeff1/2/3 are what the FSM copies on a full promote; the base-coefficient copies are the
/// flagged per-path float constants). Returns the written torque setpoint (also stored in
/// `st.torque_setpoint`).
pub fn fsm_step(inp: &FsmInputs, profile: &GainProfile, st: &mut FsmState) -> i16 {
    let run_triple = profile.as_triple();

    match st.sub_state {
        SubState::Idle => idle(inp, profile, st),
        SubState::Arming => arming(inp, run_triple, st),
        SubState::AltEngaged => alt_engaged(inp, run_triple, st),
        SubState::Run => run(inp, profile, st),
    }

    // Section 7.3: final torque-setpoint output (every tick, all sub-states; the IDLE
    // upright-skip goto lands here too).
    final_output(inp, st)
}

fn idle(inp: &FsmInputs, profile: &GainProfile, st: &mut FsmState) {
    // Zero the output mirror.
    st.out_mirror = 0;

    // Decay the envelope toward zero by 1000/tick (symmetric, +-1000 deadband; the binary's
    // branch route, value-identical to the archive's).
    if st.env < 0x3E9 {
        if st.env < -1000 {
            st.env += envelope::DECAY;
        } else {
            st.env = 0;
        }
    } else {
        st.env -= envelope::DECAY;
    }

    // The whole upright+engage evaluation sits under the master pre-gate byte == 0.
    if !inp.pre_gate_clear {
        return;
    }

    // The upright test: the ONE shared reference scaled x100.0f (FLAGGED: single-precision
    // float in original -> Q; the (f) boundary bound), f2iz to a SHORT (the binary narrows the
    // conversion result to a halfword), abs. The `>` comparison guards the SKIP to the output
    // stage: engage is reachable only when the magnitude is AT OR BELOW the branch threshold
    // (the corrected window; stock 7.2 had it inverted, see the spec's source note).
    let scaled = q_to_int_d2iz(inp.upright_ref * Fix::from_num(100)) as i16;
    let mag = iabs(scaled as i32);
    let skip = if inp.orientation_nz {
        mag > fsmc::UPRIGHT_LIMIT_ALT
    } else {
        mag > fsmc::UPRIGHT_LIMIT
    };
    if skip {
        return;
    }

    // The engage conjunction (aggregate-parameterized per the spec's inputs contract).
    let faults_clear =
        !inp.over_current && !inp.stall && !inp.comms_loss && !inp.tilt && inp.enable_bytes_clear;
    let engage = inp.gating_field > fsmc::GATING_THRESHOLD
        && inp.rider_present
        && faults_clear
        && inp.power_enable;

    if engage {
        st.sub_state = SubState::Arming;
        // Clear the two opaque state words (the clear map), set active, zero the envelope.
        st.state_word_84 = 0;
        st.state_word_88 = 0;
        st.balancing_active = true;
        st.env = 0;
        // Seed the live QUADRUPLE by orientation (Section 7.2):
        // orient == 0: base <- the field@0x48 copy (0.4f); setup <- coeff1, coeff2, 0.
        // orient != 0: base <- 3.0f (0x40400000); setup <- 1000, 300, 0.
        if inp.orientation_nz {
            st.base_coeff = Fix::from_num(3.0); // FLAGGED: 3.0f in original -> Q
            st.gains = ARMING_SEED_ORIENT_NZ;
        } else {
            st.base_coeff = Fix::from_num(0.4); // FLAGGED: the @0x48 0.4f copy -> Q
            st.gains = GainTriple::new(profile.coeff1, profile.coeff2, 0);
        }
    }
}

fn arming(inp: &FsmInputs, run_triple: GainTriple, st: &mut FsmState) {
    // Mirror the SMOOTHED reference (@0xa4) into the output mirror.
    st.out_mirror = inp.smoothed_ref;

    // Ramp while env < 0x6F55; the promote happens on the NEXT tick in the else branch, so env
    // transiently holds 28600 for one tick (visible to the output clamp below).
    if st.env < 0x6F55 {
        st.env += envelope::RAMP_UP;
    } else {
        st.env = envelope::CAP;
        st.state_word_8c = 0; // cap-entry clear (the clear map)
        if inp.orientation_nz {
            // sub-state <- 2; base <- 0.4f (DAT_08003fb8, decoded); setup <- {50, 20, 0}.
            st.sub_state = SubState::AltEngaged;
            st.base_coeff = Fix::from_num(0.4); // FLAGGED: 0.4f -> Q
            st.gains = STANDBY_SET;
        } else {
            // sub-state <- 3 (RUN); base <- the @0x48 copy; setup <- (coeff1, coeff2, coeff3).
            st.sub_state = SubState::Run;
            st.base_coeff = Fix::from_num(0.4); // FLAGGED: the @0x48 0.4f copy -> Q
            st.gains = run_triple;
        }
    }

    // Abort to IDLE if (comms-loss OR stop OR over-current) AND orientation != 0. Checked
    // AFTER the ramp/promote (the binary's order): a same-tick promote is overridden.
    if (inp.comms_loss || inp.stop_byte || inp.over_current) && inp.orientation_nz {
        st.balancing_active = false;
        st.sub_state = SubState::Idle;
        st.state_word_84 = 0;
        st.state_word_88 = 0;
    }
}

fn alt_engaged(inp: &FsmInputs, run_triple: GainTriple, st: &mut FsmState) {
    // The sub-2 reference (the spec (c) formula): the mix term is INTEGER-truncated (/100)
    // BEFORE the double add (halfword inputs), then the SUM converts once (d2iz). Modeled as
    // the exact rational over 10000 with a single truncation (the PID step-1 fidelity-bounds
    // class); the battery term keeps full precision into the sum.
    let batt_num = (inp.ref_9c as i64) * 0x50; // *80, i32 product widened
    let mix_term = ((((inp.ref_34 as i32) * 7) + ((inp.ref_36 as i32) * -6)) * 0x32 / 100) as i64;
    let sum_over_10000 = batt_num + mix_term * 10000;
    let mut refv = (sum_over_10000 / 10000) as i32; // the ONE d2iz (trunc toward zero)

    // Clamp into [-28500, +28500], store at @0xa4 (overwriting the PID's IIR value this tick),
    // copy into the mirror; the envelope recomputes from the reference magnitude.
    refv = clamp(refv, -envelope::CAP, envelope::CAP);
    st.out_mirror = refv;
    st.env = iabs(refv);

    // Abort to IDLE on (comms-loss OR stop OR over-current). NO early return: the binary falls
    // through to the promotion debounce, whose same-tick promote overrides the abort's
    // sub-state write (pinned by test).
    if inp.comms_loss || inp.stop_byte || inp.over_current {
        st.balancing_active = false;
        st.sub_state = SubState::Idle;
        st.state_word_84 = 0;
        st.state_word_88 = 0;
    }

    // Promotion debounce: while the condition byte is HELD, increment (cap 0x8ACE); once > 5,
    // promote to RUN: base <- the @0x48 copy, the full triple, env reseeded from the
    // just-written reference magnitude (NOT the cap), and the wind-down counter @0x94 cleared.
    if inp.promote_condition {
        if st.promote_counter < fsmc::DEBOUNCE_CAP {
            st.promote_counter += 1;
        }
        if st.promote_counter > fsmc::PROMOTE_TRIP {
            st.sub_state = SubState::Run;
            st.base_coeff = Fix::from_num(0.4); // FLAGGED: the @0x48 0.4f copy -> Q
            st.gains = run_triple;
            st.env = iabs(refv);
            st.winddown_counter = 0;
        }
    } else {
        st.promote_counter = 0;
    }
}

fn run(inp: &FsmInputs, profile: &GainProfile, st: &mut FsmState) {
    // Mirror the smoothed reference (@0xa4) into the output mirror.
    st.out_mirror = inp.smoothed_ref;

    // Pickup / step-off detection (the shared gating/pickup halfword, negative side).
    if inp.gating_field < 0 {
        if st.pickup_counter < fsmc::PICKUP_CAP {
            st.pickup_counter += 1;
        }
        if st.pickup_counter > fsmc::PICKUP_TRIP {
            st.sub_state = SubState::Idle;
        }
    } else {
        st.pickup_counter = 0;
    }

    // Envelope maintenance: while orientation != 0, ramp by 200/tick with the same else-branch
    // clamp as ARMING (the one-tick 28600 overshoot pattern).
    if inp.orientation_nz {
        if st.env < 0x6F55 {
            st.env += envelope::RAMP_UP;
        } else {
            st.env = envelope::CAP;
        }
    }

    // Step-off / wind-down debounce: while the two enable bytes are clear, increment (cap
    // 0x8ACE); once > 10, disengage, clearing @0x8C + @0x90 (the clear map) and writing the
    // quadruple; env <- the cap.
    if inp.winddown_enables_clear {
        if st.winddown_counter < fsmc::DEBOUNCE_CAP {
            st.winddown_counter += 1;
        }
        if st.winddown_counter > fsmc::WINDDOWN_TRIP {
            st.state_word_8c = 0;
            st.promote_counter = 0;
            if inp.orientation_nz {
                // sub-state <- 2; base <- 0.4f (DAT_08003fb8); alternate setup {50, 20, 0}.
                st.sub_state = SubState::AltEngaged;
                st.base_coeff = Fix::from_num(0.4); // FLAGGED: 0.4f -> Q
                st.gains = STANDBY_SET;
            } else {
                // sub-state <- 0 (IDLE); base <- the @0x48 copy; setup <- (coeff1, coeff2, 0).
                st.sub_state = SubState::Idle;
                st.base_coeff = Fix::from_num(0.4); // FLAGGED: the @0x48 0.4f copy -> Q
                st.gains = GainTriple::new(profile.coeff1, profile.coeff2, 0);
            }
            st.env = envelope::CAP;
        }
    } else {
        st.winddown_counter = 0;
    }

    // Immediate stop: comms-loss OR over-current, checked LAST (the binary's order: a
    // same-tick wind-down target is overridden to IDLE).
    if inp.comms_loss || inp.over_current {
        st.balancing_active = false;
        st.sub_state = SubState::Idle;
        st.state_word_84 = 0;
        st.state_word_88 = 0;
    }
}

/// Section 7.3: final torque-setpoint output, unconditionally each tick. The mirror is NOT
/// re-zeroed here (the IDLE arm zeroes it at entry; a mid-tick drop to IDLE, e.g. the RUN
/// pickup trip, still emits the enveloped mirror ONCE, exactly as the binary does).
fn final_output(inp: &FsmInputs, st: &mut FsmState) -> i16 {
    // 1. shadow the sub-state.
    st.sub_state_shadow = st.sub_state;

    // 2. envelope clamp (soft-start amplitude limit): clamp the mirror symmetrically to +-env
    // (the binary's comparison form, equivalent to the symmetric clamp for env >= 0).
    let env = st.env;
    let mut out = st.out_mirror;
    if env < out {
        out = env;
    } else if out < -env {
        out = -env;
    }

    // 3. write the torque setpoint = low 16 bits of the enveloped out (the atomic i16 write).
    st.torque_setpoint = out as i16;

    // 4. update the tracking delta for next tick: latch fb (a halfword), then
    // delta = (torque_setpoint - fb) >> 6 with round-toward-zero correction, stored as a
    // halfword.
    st.prev_fb = inp.feedback_fb;
    let x = (st.torque_setpoint as i32) - (inp.feedback_fb as i32);
    st.tracking_delta = shr_round_to_zero(x, 6) as i16;

    st.torque_setpoint
}
