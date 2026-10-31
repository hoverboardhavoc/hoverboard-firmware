//! Host tests (`specs/control.md`, section (f) validation). Pure math/logic; no hardware. Where
//! the contract gives an exact constant, the assertion checks the exact value.
//!
//! The recovered archive vectors come across byte-identical (the shift-helper and engagement
//! tests, modulo the I32F32 -> base::fixed::Fix type rename). The clamp/ramp helper tests are NEW
//! (the archive carried no vectors for them), hand-derived and marked as such. The recovered PID
//! cascade's vectors (shaping, PID, speed loop, outer speed loop, commanded lean) went with the
//! cascade (`specs/state-feedback.md`); the state-feedback law's own tests are `sf/tests.rs`.

use crate::config::fsm as fsmc;
use crate::config::sf::{REFERENCE_DEFAULT, REFERENCE_MAX, REFERENCE_MIN};
use crate::config::{
    ramp, GainProfile, GainShadow, GainTriple, TuneError, DEFAULT_GAIN_MAX, GAIN_MIN,
    REFERENCE_FIELD, SF_GAINS_FIELD, STANDBY_SET,
};
use crate::fsm::{fsm_step, tilt_cutout, FsmInputs, FsmState, SubState};
use crate::gating::GatingFilter;
use crate::helpers::{
    clamp, clamp_sym, iabs, ramp_step, shr_round_to_zero, RampRecord, RAMP_COUNTER_CAP,
};
use crate::mode::{select_mode, ControlDispatch, ControlMode, Frame, Producer};
use crate::sf::wheel_forward;
use crate::throttle::{
    filt_low_pass32, mixer_fcn, rate_limiter16, throttle_tick, ThrottleConfig, ThrottleState,
};
use base::fixed::Fix;

/// The recovered stock RUN profile {6000, 2000, 40}: the engagement machine's vectors seed and
/// ramp a NON-trivial profile, so a seed that copied the wrong word shows. The production
/// producers hand the machine [`STANDBY_SET`] (`specs/state-feedback.md` section 5).
const RUN_PROFILE_A: GainTriple = GainTriple::new(6000, 2000, 40);
/// The recovered stock low-authority profile {3000, 1000, 30}, the ramp vectors' second target.
const PROFILE_B: GainTriple = GainTriple::new(3000, 1000, 30);

/// Local f64-reference tolerance check (the assert_close discipline of specs/control.md (f);
/// base's own helper is `#[cfg(test)]`-internal to base, so the discipline is restated here).
fn assert_close(value: f64, reference: f64, tol: f64) {
    let diff = (value - reference).abs();
    assert!(
        diff <= tol,
        "assert_close failed: value={value} reference={reference} diff={diff} tol={tol}"
    );
}

// ---- shift helper ----

#[test]
fn shr_round_to_zero_truncates_toward_zero() {
    // Plain arithmetic >> floors; the corrected shift truncates toward zero for negatives.
    assert_eq!(shr_round_to_zero(64, 6), 1);
    assert_eq!(shr_round_to_zero(-64, 6), -1);
    assert_eq!(
        shr_round_to_zero(-63, 6),
        0,
        "-63/64 truncates toward zero to 0"
    );
    assert_eq!(
        (-63i32) >> 6,
        -1,
        "plain shift floors to -1 (the thing we avoid)"
    );
}

// ---- clamps + abs (Sections 8.3-8.5; NEW vectors, the archive carried none) ----

#[test]
fn clamp_and_clamp_sym_bite_on_both_sides() {
    // Section 8.4 bounded clamp: below -> lo, above -> hi, inside -> unchanged.
    assert_eq!(clamp(5, 0, 10), 5);
    assert_eq!(clamp(-1, 0, 10), 0);
    assert_eq!(clamp(11, 0, 10), 10);
    assert_eq!(clamp(0, 0, 10), 0, "boundary values pass through");
    assert_eq!(clamp(10, 0, 10), 10);
    // Section 8.3 symmetric clamp: +-limit outside, unchanged inside (boundary inclusive).
    assert_eq!(clamp_sym(5, 10), 5);
    assert_eq!(clamp_sym(15, 10), 10);
    assert_eq!(clamp_sym(-15, 10), -10);
    assert_eq!(clamp_sym(10, 10), 10);
    assert_eq!(clamp_sym(-10, 10), -10);
}

#[test]
fn iabs_folds_sign() {
    // Section 8.5.
    assert_eq!(iabs(0), 0);
    assert_eq!(iabs(5), 5);
    assert_eq!(iabs(-5), 5);
    assert_eq!(iabs(i32::MIN + 1), i32::MAX);
}

// ---- slew limiter / ramp (Section 8.2; NEW vectors, the archive carried none) ----

#[test]
fn ramp_fast_snap_sets_bound_and_adds_small_step() {
    // Step 1: current_speed/1000 <= step_threshold -> step_bound := 0x20 and the value moves by
    // small_step; the saturating counter is untouched.
    let mut rec = RampRecord {
        current_value: 100,
        step_threshold: 5,
        step_bound: 0,
        small_step: 7,
        counter: 0,
    };
    assert_eq!(
        ramp_step(0, 5000, &mut rec),
        107,
        "5000/1000 = 5 <= 5 snaps"
    );
    assert_eq!(rec.step_bound, 0x20, "snap region arms the 0x20 bound");
    assert_eq!(rec.counter, 0, "counter untouched in the snap region");
    // Negative speeds divide toward zero and stay in the snap region too.
    assert_eq!(ramp_step(0, -10_000, &mut rec), 114);
}

#[test]
fn ramp_bounded_step_walks_all_four_arms() {
    // Step 3's four branch arms, outside the snap region (speed/1000 > threshold), with the
    // armed bound b = 0x20 = 32:
    let mut rec = RampRecord {
        current_value: 100,
        step_threshold: 0,
        step_bound: 0x20,
        small_step: 0,
        counter: 0,
    };
    // b < current_value: subtract the bound (100 -> 68 -> 36 -> 4).
    assert_eq!(ramp_step(0, 1001, &mut rec), 68);
    assert_eq!(ramp_step(0, 1001, &mut rec), 36);
    assert_eq!(ramp_step(0, 1001, &mut rec), 4);
    // current_value > 0 (but not above the bound): subtract the fixed 32 (4 -> -28).
    assert_eq!(ramp_step(0, 1001, &mut rec), -28);
    // -b < current_value (inside the negative band): add the fixed 32 (-28 -> 4).
    assert_eq!(ramp_step(0, 1001, &mut rec), 4);
    // current_value <= -b: add the bound (-40 -> -8, the fourth arm).
    rec.current_value = -40;
    assert_eq!(ramp_step(0, 1001, &mut rec), -8);
}

#[test]
fn ramp_counter_saturates_at_cap() {
    // Step 2: the counter increments once per non-snap call and saturates at 0xFA = 250.
    let mut rec = RampRecord {
        current_value: 0,
        step_threshold: 0,
        step_bound: 0x20,
        small_step: 0,
        counter: RAMP_COUNTER_CAP - 1,
    };
    let _ = ramp_step(0, 1001, &mut rec);
    assert_eq!(rec.counter, RAMP_COUNTER_CAP);
    let _ = ramp_step(0, 1001, &mut rec);
    assert_eq!(
        rec.counter, RAMP_COUNTER_CAP,
        "saturated, no further growth"
    );
}

// ---- the gating/pickup producer (the conditioned up-axis accel channel) ------------------
//
// The engagement machine's `> 500` / `< 0` cell, whose stock producer was recovered from the
// binary: `state = 0.02*az + 0.98*state`, d2iz to s16, in raw +-4 g counts (8192 per g). The
// vectors are hand-derived from that recurrence and stated to the count.

/// One g on the up axis at the stock +-4 g scale.
const ONE_G: i32 = 8192;

#[test]
fn gating_filter_climbs_into_the_engage_band_from_a_level_sample() {
    // From the cold-boot zero, a level 1 g sample: 0.02*8192 = 163.84 -> d2iz 163, then the
    // 0.98 accumulation. The exact first four counts, and the tick the engage edge falls on.
    let mut f = GatingFilter::default();
    let g = Fix::from_num(ONE_G);
    assert_eq!(f.tick(g), 163, "tick 1: 163.84 truncated toward zero");
    assert_eq!(f.tick(g), 324, "tick 2: 324.42");
    assert_eq!(f.tick(g), 481, "tick 3: 481.77");
    assert_eq!(f.tick(g), 635, "tick 4: 635.98");

    // Tick 4 is the first sample past the engage threshold: a board cannot engage on the first
    // sample after reset, and it can within ~16 ms of a level one.
    assert!(635 > fsmc::GATING_THRESHOLD as i32);
    assert!(481 <= fsmc::GATING_THRESHOLD as i32);
}

#[test]
fn gating_filter_converges_on_the_sample_it_is_fed() {
    // Unity gain: the cell IS the accel count in the steady state (no scaling, no clamp), so
    // the 500 threshold really is 500/8192 = 0.061 g.
    let mut f = GatingFilter::default();
    let g = Fix::from_num(ONE_G);
    let mut last = 0;
    for _ in 0..2000 {
        last = f.tick(g);
    }
    assert!(
        (ONE_G - last as i32).abs() <= 1,
        "converged to the fed count, got {last}"
    );
}

#[test]
fn gating_filter_holds_a_shallow_sample_below_the_engage_edge() {
    // A deck tilted so far that the up-axis reads under 0.061 g never opens the gate, however
    // long it is held. 400 counts = 0.049 g.
    let mut f = GatingFilter::default();
    let shallow = Fix::from_num(400);
    for _ in 0..2000 {
        assert!(
            f.tick(shallow) <= fsmc::GATING_THRESHOLD,
            "a 0.049 g up-axis must never clear the engage gate"
        );
    }
}

#[test]
fn gating_filter_goes_negative_for_an_inverted_deck_and_truncates_toward_zero() {
    // The pickup side. The recurrence is symmetric, and d2iz truncates TOWARD ZERO on both
    // signs (-163, not -164), which is the binary's conversion, not a floor.
    let mut f = GatingFilter::default();
    let inverted = Fix::from_num(-ONE_G);
    assert_eq!(f.tick(inverted), -163);
    assert_eq!(f.tick(inverted), -324);
    let mut last = 0;
    for _ in 0..2000 {
        last = f.tick(inverted);
    }
    assert!(last < 0, "an inverted deck holds the pickup side negative");
}

#[test]
fn gating_filter_needs_many_ticks_to_cross_the_pickup_edge_after_a_flip() {
    // The IIR is the anti-chatter filter: from an engaged, level state, an instantaneous flip
    // takes several ticks to drive the cell negative, and only THEN does the FSM's 20-tick
    // pickup debounce start. A single bad sample cannot disengage a running board.
    let mut f = GatingFilter::default();
    let g = Fix::from_num(ONE_G);
    for _ in 0..2000 {
        f.tick(g);
    }
    let inverted = Fix::from_num(-ONE_G);
    let mut ticks = 0;
    while f.tick(inverted) >= 0 {
        ticks += 1;
        assert!(ticks < 500, "must cross zero eventually");
    }
    assert!(
        ticks > 20,
        "the flip takes {ticks} ticks to reach the pickup side, before any debounce"
    );
}

// ---- engagement machine (Section 7, rebuilt to the binary per the slice-5 re-cut) ----
//
// Vector disposition of the four archived FSM tests (the slice-5 audit ruling): the
// idle->arming->run ramp vector is REPLACED (its engage fixture rode the inverted upright
// window and its 143-tick same-tick promote is the archive misfold; the corrected vector pins
// tick-144 promotion with the one-tick 28600 overshoot); the fault and enveloped-mirror
// vectors survive with corrected engage fixtures; the decay vector survives as-is. The rest is
// NEW, decompile-derived.

/// The corrected engage fixture: orient == 0, upright (2000 <= 2499), every gate open.
fn engage_fsm_inputs() -> FsmInputs {
    FsmInputs {
        orientation_nz: false,
        upright_ref: Fix::from_num(20), // x100.0f = 2000, inside the window
        pre_gate_clear: true,
        smoothed_ref: 1000,
        gating_field: 600,
        rider_present: true,
        battery_ok: true,
        enable_bytes_clear: true,
        power_enable: true,
        ..Default::default()
    }
}

/// Drive a default state to RUN through the corrected engage + the tick-144 promote.
fn engage_to_run(profile: &GainProfile) -> FsmState {
    let mut st = FsmState::default();
    let inp = engage_fsm_inputs();
    let _ = fsm_step(&inp, profile, &mut st);
    assert_eq!(st.sub_state, SubState::Arming);
    let mut ticks = 0;
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, profile, &mut st);
        ticks += 1;
        assert!(ticks < 200, "must promote within bound");
    }
    assert_eq!(st.sub_state, SubState::Run);
    st
}

#[test]
fn fsm_engage_window_is_below_threshold_both_branches() {
    // The corrected upright window (the slice-5 safety-class fix): engage is reachable only
    // when |f2iz(ref*100.0f)| <= 2499 (orient==0) / <= 7499 (orient!=0); the archive (and
    // stock 7.2) required strictly ABOVE. Both sides of both thresholds, exact edges included
    // (the edge refs are dyadic, so ref*100 is exact in Q and the d2iz is boundary-safe).
    let profile = GainProfile::of(RUN_PROFILE_A);

    // orient == 0: 2499 engages, 2500 does not.
    for (ref_val, engages) in [
        (24.9921875, true),
        (25.0, false),
        (-20.0, true),
        // Ride-along (slice 6): the literal negative edges. d2iz is toward zero, so
        // -24.9921875 scales to -2499 (magnitude 2499, engages) and -25.0 to -2500 (skips).
        (-24.9921875, true),
        (-25.0, false),
    ] {
        let mut st = FsmState {
            state_word_84: 7,
            state_word_88: 9,
            state_word_8c: 3,
            ..Default::default()
        };
        let mut inp = engage_fsm_inputs();
        inp.upright_ref = Fix::from_num(ref_val);
        let _ = fsm_step(&inp, &profile, &mut st);
        if engages {
            assert_eq!(st.sub_state, SubState::Arming, "ref {ref_val} engages");
            assert!(st.balancing_active);
            assert_eq!(st.env, 0);
            // The quadruple, orient == 0: base <- the @0x48 0.4f copy; setup (c1, c2, 0).
            assert_eq!(st.gains, crate::config::GainTriple::new(6000, 2000, 0));
            assert_eq!(st.base_coeff, Fix::from_num(0.4));
            // The clear map: engage zeroes @0x84/@0x88 only; @0x8C untouched.
            assert_eq!((st.state_word_84, st.state_word_88), (0, 0));
            assert_eq!(st.state_word_8c, 3);
        } else {
            assert_eq!(st.sub_state, SubState::Idle, "ref {ref_val} skips engage");
            assert_eq!(st.state_word_84, 7, "skip leaves the state words alone");
        }
    }

    // orient != 0: 7499 engages (with the 3.0f base + {1000, 300, 0} seed), 7500 does not.
    for (ref_val, engages) in [(74.9921875, true), (75.0, false)] {
        let mut st = FsmState::default();
        let mut inp = engage_fsm_inputs();
        inp.orientation_nz = true;
        inp.upright_ref = Fix::from_num(ref_val);
        let _ = fsm_step(&inp, &profile, &mut st);
        if engages {
            assert_eq!(st.sub_state, SubState::Arming);
            assert_eq!(st.gains, crate::config::GainTriple::new(1000, 300, 0));
            assert_eq!(st.base_coeff, Fix::from_num(3.0), "the 3.0f engage seed");
        } else {
            assert_eq!(st.sub_state, SubState::Idle);
        }
    }

    // The master pre-gate byte: set (not clear) skips the whole evaluation.
    let mut st = FsmState::default();
    let mut inp = engage_fsm_inputs();
    inp.pre_gate_clear = false;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Idle, "pre-gate byte blocks engage");
}

#[test]
fn fsm_arming_promotes_next_tick_with_one_tick_overshoot() {
    // The corrected promote timing: ramp while env < 0x6F55, promote on the NEXT tick in the
    // else branch. From env 0: ramp tick 143 leaves env = 28600 (the one-tick overshoot,
    // visible to the output clamp); tick 144 promotes with env = 28500. (The archive promoted
    // same-tick at 143 with no overshoot.)
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = FsmState {
        state_word_8c: 5, // observe the cap-entry clear
        ..Default::default()
    };
    let inp = engage_fsm_inputs();
    let _ = fsm_step(&inp, &profile, &mut st); // IDLE -> ARMING
    let mut ticks = 0;
    let mut peak_env = 0;
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, &profile, &mut st);
        ticks += 1;
        peak_env = peak_env.max(st.env);
        assert!(ticks < 200, "must promote within bound");
    }
    assert_eq!(
        ticks, 144,
        "promotion on the tick AFTER the ramp passes the cap"
    );
    assert_eq!(peak_env, 28600, "the one-tick overshoot");
    assert_eq!(st.env, 28500, "cap on the promote tick");
    assert_eq!(st.sub_state, SubState::Run);
    // The promote installs kp <- coeff1, bk <- coeff2 (already the engage seed's on this path) and
    // leaves `pr` to the RUN ramp (`specs/remote-ui.md` section 4): (coeff1, coeff2, 0) stands on
    // the promote tick, and with `kd` zero the first RUN pass takes `pr` the rest of the way (the
    // term it multiplies is zero).
    assert_eq!(st.gains, GainTriple::new(6000, 2000, 0));
    assert_eq!(st.base_coeff, Fix::from_num(0.4));
    assert_eq!(st.state_word_8c, 0, "cap-entry clears @0x8C");
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.gains, RUN_PROFILE_A);
}

#[test]
fn fsm_arming_abort_ordering_and_orientation_gate() {
    let profile = GainProfile::of(RUN_PROFILE_A);

    // orient == 0: the ARMING abort is orientation-gated; comms loss mid-ramp changes nothing.
    let mut st = FsmState::default();
    let mut inp = engage_fsm_inputs();
    let _ = fsm_step(&inp, &profile, &mut st);
    inp.comms_loss = true;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Arming, "abort requires orient != 0");

    // orient != 0 at the cap-entry tick WITH an abort condition: the promote-to-2 writes land
    // first (the binary's order), then the abort overrides the sub-state; the promote's gain
    // writes stand.
    let mut st = FsmState {
        sub_state: SubState::Arming,
        env: 28600,
        balancing_active: true,
        ..Default::default()
    };
    let mut inp = engage_fsm_inputs();
    inp.orientation_nz = true;
    inp.comms_loss = true;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(
        st.sub_state,
        SubState::Idle,
        "abort overrides the same-tick promote"
    );
    assert!(!st.balancing_active);
    assert_eq!(st.env, 28500, "the promote's env write stands");
    assert_eq!(st.gains, STANDBY_SET, "the promote's gain write stands");
}

#[test]
fn fsm_sub2_reference_pretruncation_and_d2iz() {
    // The sub-2 formula (spec (c)): the mix is INTEGER-truncated (/100) BEFORE the double add,
    // then ONE d2iz. Discriminators against the archive's round-half common-denominator fold:
    //  - 9c=75, s34=s36=3:  mix_term = trunc(150/100) = 1; ref = trunc(0.6 + 1) = 1
    //    (the fold gave round(2.1) = 2).
    //  - 9c=-75, s34=s36=-3: ref = trunc(-1.6) = -1 (the fold gave -2; a floor gives -3).
    //  - 9c=0, s34=s36=5:   mix_term = trunc(250/100) = 2; ref = 2 (round-half gave 3).
    let profile = GainProfile::of(RUN_PROFILE_A);
    for (r9c, s, want) in [(75i32, 3i16, 1i32), (-75, -3, -1), (0, 5, 2)] {
        let mut st = FsmState {
            sub_state: SubState::AltEngaged,
            ..Default::default()
        };
        let inp = FsmInputs {
            orientation_nz: true,
            ref_9c: r9c,
            ref_34: s,
            ref_36: s,
            ..Default::default()
        };
        let torque = fsm_step(&inp, &profile, &mut st);
        assert_eq!(st.out_mirror, want, "9c={r9c} s={s}");
        assert_eq!(st.env, want.abs(), "envelope recomputed from |ref|");
        assert_eq!(torque as i32, want, "enveloped output equals the reference");
    }

    // The clamp: a huge battery term saturates the reference at +-28500.
    let mut st = FsmState {
        sub_state: SubState::AltEngaged,
        ..Default::default()
    };
    let inp = FsmInputs {
        ref_9c: 100_000_000,
        ..Default::default()
    };
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.out_mirror, 28500);
    assert_eq!(st.env, 28500);
}

#[test]
fn fsm_sub2_promote_reseeds_env_from_reference() {
    // Promote (counter > 5): env reseeds to |@0xa4| (the just-written reference), NOT the cap
    // (the archive wrote CAP); the wind-down counter @0x94 clears; the quadruple gets the
    // @0x48 0.4f copy, and kp/bk get the profile's coeff1/coeff2 as the binary installs them.
    // `pr` is left to the RUN ramp (the binary's coeff3 install is not carried,
    // `specs/remote-ui.md` section 4): the standby seed's 0 stands on the promote tick.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = FsmState {
        sub_state: SubState::AltEngaged,
        promote_counter: 5,
        winddown_counter: 7,
        ..Default::default()
    };
    let inp = FsmInputs {
        promote_condition: true,
        ref_34: 5,
        ref_36: 5, // reference = 2 (the vector above)
        pid_scale: 3600,
        pid_kd: Fix::from_num(100),
        ..Default::default()
    };
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Run);
    assert_eq!(st.env, 2, "env = |reference|, not the cap");
    assert_eq!(st.winddown_counter, 0, "@0x94 cleared on promote");
    assert_eq!(
        st.gains,
        GainTriple::new(profile.coeff1, profile.coeff2, STANDBY_SET.pr),
        "kp/bk installed, the standby seed's pr stands on the promote tick"
    );
    assert_eq!(st.base_coeff, Fix::from_num(0.4));
    // The RUN passes after it carry pr toward coeff3 by its cap (9 at 3600 with kd 100), and
    // kp/bk, already at the profile, stay there.
    assert_eq!(ramp::caps(3600, Fix::from_num(100))[2], 9);
    for want_pr in [9, 18, 27, 36, 40, 40] {
        let _ = fsm_step(&inp, &profile, &mut st);
        assert_eq!(st.sub_state, SubState::Run);
        assert_eq!(
            st.gains,
            GainTriple::new(profile.coeff1, profile.coeff2, want_pr)
        );
    }
}

#[test]
fn fsm_sub2_abort_does_not_early_return() {
    // The binary's sub-2 arm has NO early return: an abort's sub-state write is overridden by
    // a same-tick promote (sequential writes, last wins), while the abort's active-flag and
    // state-word clears stand. Pinned as the binary's behavior.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = FsmState {
        sub_state: SubState::AltEngaged,
        promote_counter: 6,
        balancing_active: true,
        state_word_84: 4,
        state_word_88: 4,
        ..Default::default()
    };
    let inp = FsmInputs {
        promote_condition: true,
        comms_loss: true,
        ..Default::default()
    };
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(
        st.sub_state,
        SubState::Run,
        "the promote write lands after the abort's"
    );
    assert!(!st.balancing_active, "the abort's flag clear stands");
    assert_eq!((st.state_word_84, st.state_word_88), (0, 0));
}

#[test]
fn fsm_fault_forces_idle_immediately() {
    // (Archive vector, corrected engage fixture.) RUN + over-current -> immediate IDLE. The
    // drop tick still emits the enveloped mirror ONCE (no output-stage re-zero, the binary's
    // behavior); the next tick's IDLE arm zeroes it.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);
    let mut inp = engage_fsm_inputs();
    inp.over_current = true;
    let t_drop = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Idle);
    assert!(!st.balancing_active);
    assert_eq!(
        t_drop, 1000,
        "the drop tick still emits the enveloped mirror"
    );
    let t_next = fsm_step(&inp, &profile, &mut st);
    assert_eq!(t_next, 0, "IDLE zeroes the mirror the next tick");
}

#[test]
fn tilt_cutout_trips_past_the_limit_on_either_axis() {
    // The RUN cutout's predicate (`specs/control.md` (l)), at its edges and on both axes: the
    // limit is a magnitude, 4500 centidegrees is still inside it, 4501 is not. The engage window
    // (2499) passes here, which is the hysteresis the decision is built on: the machine can start
    // at 25 degrees and is not cut until 45.
    assert_eq!(crate::config::fsm::TILT_LIMIT, 4500);
    for inside in [0i16, 2499, -2499, 4500, -4500] {
        assert!(!tilt_cutout(inside, 0), "pitch {inside} is not a fall");
        assert!(!tilt_cutout(0, inside), "roll {inside} is not a fall");
    }
    for fallen in [4501i16, -4501, 9000, -9000, 18000, -18000] {
        assert!(tilt_cutout(fallen, 0), "pitch {fallen} is a fall");
        assert!(tilt_cutout(0, fallen), "roll {fallen} is a fall");
    }
    // A rigid machine on its side: roll alone trips it with the pitch word at rest.
    assert!(tilt_cutout(120, 9000));
}

#[test]
fn fsm_tilt_forces_idle_in_run() {
    // The missing half of the upright window: a machine that falls past the limit while RUNNING
    // disengages on the tick the cutout asserts, the `over_current` shape exactly (the drop tick
    // still emits the enveloped mirror once, the binary's output stage; the next tick is zero).
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);
    let mut inp = engage_fsm_inputs();
    inp.tilt = true;
    let t_drop = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Idle);
    assert!(!st.balancing_active, "the immediate stop clears the flag");
    assert_eq!((st.state_word_84, st.state_word_88), (0, 0));
    assert_eq!(
        t_drop, 1000,
        "the drop tick still emits the enveloped mirror"
    );
    assert_eq!(fsm_step(&inp, &profile, &mut st), 0, "and then zero");
}

#[test]
fn fsm_tilt_blocks_the_re_engage_while_it_is_held() {
    // IDLE with every other gate open: the cutout's latch holds the machine out of ARMING, so a
    // machine that rights itself cannot drive off while the latch is still set. Released, the
    // same inputs engage on the next tick.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = FsmState::default();
    let mut inp = engage_fsm_inputs();
    inp.tilt = true;
    for _ in 0..50 {
        let _ = fsm_step(&inp, &profile, &mut st);
        assert_eq!(st.sub_state, SubState::Idle);
    }
    inp.tilt = false;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Arming, "released, it engages");
}

#[test]
fn fsm_run_pickup_drop() {
    // RUN pickup: the shared gating/pickup halfword negative for > 20 ticks drops to IDLE
    // (counter trip on tick 21); no flags or state words are touched by this path.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);
    let mut inp = engage_fsm_inputs();
    inp.gating_field = -1;
    for k in 1..=20 {
        let _ = fsm_step(&inp, &profile, &mut st);
        assert_eq!(st.sub_state, SubState::Run, "tick {k}: still RUN");
    }
    let t_drop = fsm_step(&inp, &profile, &mut st);
    assert_eq!(
        st.sub_state,
        SubState::Idle,
        "tick 21 trips the pickup counter"
    );
    assert!(
        st.balancing_active,
        "pickup drop does not clear the active flag"
    );
    assert_eq!(
        t_drop, 1000,
        "the drop tick still emits the enveloped mirror"
    );
}

#[test]
fn fsm_run_winddown_paths() {
    let profile = GainProfile::of(RUN_PROFILE_A);

    // orient == 0: > 10 ticks of cleared enables -> IDLE with (c1, c2, 0), base 0.4f, env at
    // the cap, @0x8C + @0x90 cleared.
    let mut st = engage_to_run(&profile);
    st.state_word_8c = 5;
    st.promote_counter = 6;
    let mut inp = engage_fsm_inputs();
    inp.winddown_enables_clear = true;
    for _ in 1..=10 {
        let _ = fsm_step(&inp, &profile, &mut st);
        assert_eq!(st.sub_state, SubState::Run);
    }
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Idle, "tick 11 trips the wind-down");
    assert_eq!(st.gains, crate::config::GainTriple::new(6000, 2000, 0));
    assert_eq!(st.base_coeff, Fix::from_num(0.4));
    assert_eq!(st.env, 28500);
    assert_eq!(
        (st.state_word_8c, st.promote_counter),
        (0, 0),
        "@0x8C + @0x90 cleared"
    );

    // orient != 0: the same trip lands in sub-state 2 with the standby set.
    let mut st = engage_to_run(&profile);
    let mut inp = engage_fsm_inputs();
    inp.orientation_nz = true;
    inp.winddown_enables_clear = true;
    for _ in 1..=11 {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    assert_eq!(st.sub_state, SubState::AltEngaged);
    assert_eq!(st.gains, STANDBY_SET);
}

#[test]
fn fsm_idle_envelope_decays_to_zero() {
    // (Archive vector, fixture updated to the new inputs shape.) No engage (rider absent):
    // -1000/tick with the deadband collapse; and the symmetric negative-side decay.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = FsmState {
        env: 28500,
        ..Default::default()
    };
    let mut inp = engage_fsm_inputs();
    inp.rider_present = false;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.env, 27500);
    for _ in 0..40 {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    assert_eq!(st.env, 0);
    assert_eq!(st.torque_setpoint, 0);

    // The negative side (defensive in the binary, recovered as-is): -2500 -> -1500 -> -500 -> 0.
    let mut st = FsmState {
        env: -2500,
        ..Default::default()
    };
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.env, -1500);
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.env, -500);
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.env, 0);
}

#[test]
fn fsm_torque_setpoint_follows_enveloped_smoothed_ref() {
    // (Archive vector, corrected engage fixture.) In RUN with env at cap the setpoint IS the
    // smoothed reference; a smaller envelope clamps it, both signs.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);
    assert_eq!(st.torque_setpoint, 1000);

    st.env = 500;
    let mut big = engage_fsm_inputs();
    big.smoothed_ref = 5000;
    let _ = fsm_step(&big, &profile, &mut st);
    assert_eq!(st.torque_setpoint, 500, "enveloped to +-env");
    st.env = 500;
    big.smoothed_ref = -5000;
    let _ = fsm_step(&big, &profile, &mut st);
    assert_eq!(st.torque_setpoint, -500);
}

#[test]
fn fsm_tracking_delta_rounds_toward_zero() {
    // Section 7.3 step 4: delta = (setpoint - fb) >> 6 with the round-toward-zero correction,
    // stored as a halfword; fb latches.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);
    let mut inp = engage_fsm_inputs();
    inp.smoothed_ref = 100;
    inp.feedback_fb = 163; // 100 - 163 = -63 -> trunc(-63/64) = 0 (a floor gives -1)
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.prev_fb, 163);
    assert_eq!(st.tracking_delta, 0, "round-toward-zero shift");
}

#[test]
fn fsm_substate_drives_the_phase_c_fault_latch() {
    // The latch-substate tie (spec (c)): the FSM's sub-state byte IS the a_substate the
    // Phase-C FaultLatch consumes; a scripted scenario drives the real latch. In RUN (3) with
    // the wheel moving the latch is HEALTHY (counter pinned at 0); after a comms fault drops
    // the FSM to IDLE (0) with the wheel still moving, the latch counts UNHEALTHY ticks to the
    // 150000-tick threshold and fires.
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);

    let mut latch = state::FaultLatch::new();
    latch.running_enable = 1;
    latch.b_motion = 500; // wheel moving
    latch.a_substate = st.sub_state as i8;
    assert_eq!(latch.a_substate, 3, "RUN is the byte value 3");
    for _ in 0..1000 {
        latch.tick();
    }
    assert!(latch.is_healthy());
    assert!(!latch.is_latched());
    assert_eq!(latch.fault_counter, 0);

    // Comms fault: the FSM drops to IDLE; the latch sees a_substate 0 with motion.
    let mut inp = engage_fsm_inputs();
    inp.comms_loss = true;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Idle);
    latch.a_substate = st.sub_state as i8;
    assert!(
        !latch.is_healthy(),
        "IDLE + moving wheel is the UNHEALTHY shape"
    );
    for _ in 0..state::LATCH_THRESHOLD {
        latch.tick();
    }
    assert!(latch.is_latched(), "the persistent inconsistency latches");
}

#[test]
fn fsm_upright_scale_tracks_f64_reference() {
    // The x100.0f upright scale (spec (f): flagged-fractional, the <=1-count boundary bound).
    // Dyadic refs make ref*100 exact in BOTH Q and f64, so the d2iz agrees exactly; the
    // mid-cell refs sit 0.5 off the boundary (the slice-4 off-boundary practice).
    for r in [-80.5, -24.25, -3.25, 0.0, 7.75, 24.5, 74.5, -3.145, 24.505] {
        let q_scaled = Fix::from_num(r) * Fix::from_num(100);
        let q_int = crate::helpers::q_to_int_d2iz(q_scaled) as i16;
        let f_ref = r * 100.0_f64;
        assert_close(q_scaled.to_num::<f64>(), f_ref, 1e-3);
        assert_eq!(q_int, f_ref.trunc() as i16, "ref {r}");
    }
}

// ---- throttle mode + dispatch (Section (b), slice 6: the phase's one new construction) ----
//
// EFERU FIXTURE PROVENANCE (the Phase-B harness practice): the vectors below replay the
// observed behavior of EFeru's own util.c functions, compiled VERBATIM (sliced from the
// checkout, never transcribed) by the gitignored reference/efferu-oracle/throttle_harness.c.
//   - EFeru checkout: reference/efferu-hoverboard @ a0751d589fd43d8975eda3683fac21a44bbfe8fa
//   - Slice: Src/util.c lines 1642-1723; constants RATE=480, FILTER=6553,
//     SPEED_COEFFICIENT=16384, STEER_COEFFICIENT=8192, INPUT_MIN/MAX=-1000/1000 (the adopted
//     defaults, cited in config::throttle).
//   - Generated 2026-07-13; regenerate per the harness header if extending.
// The fixtures record BEHAVIOR (input -> output vectors of the running model), never EFeru's
// tables as expected-data; shared-semantics assertions only.

#[test]
fn throttle_rate_limiter_matches_the_oracle() {
    // Oracle: "rate up (u=1000, rate=480): 480 960 1440 1920 2400 settled=16000 after 34 calls"
    let mut y = 0i16;
    let mut seq = std::vec::Vec::new();
    for _ in 0..5 {
        rate_limiter16(1000, 480, &mut y);
        seq.push(y);
    }
    assert_eq!(seq, [480, 960, 1440, 1920, 2400]);
    let mut calls = 5;
    while y != 1000 << 4 {
        rate_limiter16(1000, 480, &mut y);
        calls += 1;
        assert!(calls < 200);
    }
    assert_eq!((y, calls), (16000, 34), "settles at u<<4 after 34 calls");
    // Oracle: "rate rev (u=-1000 from settled): 15520 15040 14560 14080 13600"
    let mut rev = std::vec::Vec::new();
    for _ in 0..5 {
        rate_limiter16(-1000, 480, &mut y);
        rev.push(y);
    }
    assert_eq!(rev, [15520, 15040, 14560, 14080, 13600]);
    // Oracle: "rate down (u=-1000, rate=480): -480 -960 -1440 -1920 -2400"
    let mut yn = 0i16;
    let mut down = std::vec::Vec::new();
    for _ in 0..5 {
        rate_limiter16(-1000, 480, &mut yn);
        down.push(yn);
    }
    assert_eq!(down, [-480, -960, -1440, -1920, -2400]);
}

#[test]
fn throttle_low_pass_matches_the_oracle_including_the_floor_asymmetry() {
    // Oracle: "filter (u=1000, coef=6553): y32 = 6553000 12451109 17759448 22536994 26836581
    // 30706537 34189456 37323837"; ">>16 reaches 999 at call 66 (y32=65475494)".
    let mut y = 0i32;
    let mut seq = std::vec::Vec::new();
    for _ in 0..8 {
        filt_low_pass32(1000, 6553, &mut y);
        seq.push(y);
    }
    assert_eq!(
        seq,
        [6553000, 12451109, 17759448, 22536994, 26836581, 30706537, 34189456, 37323837]
    );
    let mut calls = 8;
    while (y >> 16) != 999 {
        filt_low_pass32(1000, 6553, &mut y);
        calls += 1;
        assert!(calls < 500);
    }
    assert_eq!((calls, y), (66, 65475494));
    // Oracle: "filter (u=-1000): y32 = -6553000 -12450700 -17758630 -22535767". The negative
    // trajectory is NOT the mirror of the positive one (EFeru's arithmetic >>12/>>4 FLOOR on
    // negatives); the asymmetry pins the shift semantics.
    let mut yn = 0i32;
    let mut nseq = std::vec::Vec::new();
    for _ in 0..4 {
        filt_low_pass32(-1000, 6553, &mut yn);
        nseq.push(yn);
    }
    assert_eq!(nseq, [-6553000, -12450700, -17758630, -22535767]);
    assert_ne!(
        nseq[1], -seq[1],
        "floor asymmetry: -12450700 vs -(12451109)"
    );
}

#[test]
fn throttle_mixer_matches_the_oracle() {
    // Oracle vectors (inputs already <<4 as at EFeru main.c:350; SPEED 1.0 / STEER 0.5):
    let vectors: [(i16, i16, i16, i16); 8] = [
        (1000, 0, 1000, 1000),
        (0, 1000, -500, 500),
        (500, 200, 400, 600),
        (-500, 200, -600, -400),
        (1000, 1000, 500, 1000), // L hits the +-1000 command clamp
        (-1000, -1000, -500, -1000),
        (123, -457, 351, -106), // the >>4 FLOOR on the negative side (-1688 >> 4 = -106)
        (0, 0, 0, 0),
    ];
    for (sp, st, want_r, want_l) in vectors {
        let (r, l) = mixer_fcn(sp << 4, st << 4, 16384, 8192);
        assert_eq!((r, l), (want_r, want_l), "sp={sp} st={st}");
    }
}

#[test]
fn throttle_pipeline_matches_the_oracle_through_the_frame_adapters() {
    // Oracle: "pipeline (speed_cmd=1000, steer_cmd=200): k1:(1,3) k5:(19,58) k20:(279,445)
    // k60:(884,1000) k120:(900,1000)". Replayed through throttle_tick, whose frame-in adapter
    // maps 32767 -> 1000 and 6554 -> 200 exactly (rail + truncation), so the EFeru core sees
    // the oracle's inputs.
    let cfg = ThrottleConfig::default();
    let mut st = ThrottleState::default();
    let mut samples = std::vec::Vec::new();
    for k in 1..=120 {
        let out = crate::throttle::throttle_tick(&cfg, 32767, 6554, &mut st);
        if [1, 5, 20, 60, 120].contains(&k) {
            samples.push((out.cmd_right, out.cmd_left));
        }
        // The +-28500 contract holds every tick.
        assert!(out.ref_right.abs() <= 28500 && out.ref_left.abs() <= 28500);
    }
    assert_eq!(
        samples,
        [(1, 3), (19, 58), (279, 445), (884, 1000), (900, 1000)]
    );
    // The frame-out adapter at the observed steady state: 900 -> 25650, 1000 -> 28500.
    let out = throttle_tick(&cfg, 32767, 6554, &mut st);
    assert_eq!((out.ref_right, out.ref_left), (25650, 28500));
}

#[test]
fn throttle_frame_adapters_are_exact_at_the_rails() {
    // Full forward on the +-32767 frame settles to the +-1000 command and the +-28500 word
    // exactly (spec (b)'s frames); the negative rail mirrors.
    let cfg = ThrottleConfig::default();
    let mut st = ThrottleState::default();
    let mut out = crate::throttle::ThrottleOutput::default();
    for _ in 0..300 {
        out = throttle_tick(&cfg, 32767, 0, &mut st);
    }
    assert_eq!((out.cmd_right, out.cmd_left), (1000, 1000));
    assert_eq!((out.ref_right, out.ref_left), (28500, 28500));
    let mut st = ThrottleState::default();
    for _ in 0..300 {
        out = throttle_tick(&cfg, -32767, 0, &mut st);
    }
    assert_eq!((out.ref_right, out.ref_left), (-28500, -28500));
}

#[test]
fn throttle_low_pass_settling_tracks_f64_reference() {
    // Spec (f) assert_close discipline for the conditioning: the filter's step response vs the
    // ideal first-order model y_k = u * (1 - (1 - c)^k) with c = 6553/65536. The fixed-point
    // path quantizes (the >>12/>>4 floors), so a small absolute band on the +-1000-scale
    // output covers it.
    let c = 6553.0_f64 / 65536.0;
    let mut y = 0i32;
    let mut model = 0.0_f64;
    for k in 1..=100 {
        filt_low_pass32(1000, 6553, &mut y);
        model += (1000.0 - model) * c;
        assert_close((y >> 16) as f64, model, 2.0);
        let _ = k;
    }
}

#[test]
fn control_mode_decode_and_fallback_seam() {
    // from_u8: 0 -> Throttle, 1 -> Balance, unknown -> Throttle (the fail-safe default).
    assert_eq!(ControlMode::from_u8(0), ControlMode::Throttle);
    assert_eq!(ControlMode::from_u8(1), ControlMode::Balance);
    assert_eq!(ControlMode::from_u8(2), ControlMode::Throttle);
    assert_eq!(ControlMode::from_u8(255), ControlMode::Throttle);
    // The validation seam (the commutation Foc precedent): Balance without a configured IMU
    // demotes to Throttle AND raises the fault; with the IMU it stands; Throttle never faults.
    let demoted = select_mode(1, 0, false);
    assert_eq!(demoted.active, Producer::Throttle);
    assert!(demoted.fault);
    let ok = select_mode(1, 0, true);
    assert_eq!(ok.active, Producer::Balance);
    assert!(!ok.fault);
    let thr = select_mode(0, 0, false);
    assert_eq!(thr.active, Producer::Throttle);
    assert!(!thr.fault);
}

/// **The resolution table, every row** (`specs/rover-one-imu.md` section 2): the two stored bytes
/// and the board's IMU fact resolve to one producer, and only the pivoting Balance-without-IMU
/// row is a demotion with the fault. An unknown frame byte reads as pivoting, the fail-safe that
/// keeps an unstaged board byte-identical to the image before the field.
#[test]
fn the_resolution_table_every_row() {
    assert_eq!(Frame::from_u8(0), Frame::Pivoting);
    assert_eq!(Frame::from_u8(1), Frame::Rigid);
    assert_eq!(Frame::from_u8(2), Frame::Pivoting);
    assert_eq!(Frame::from_u8(255), Frame::Pivoting);
    let rows: [(u8, u8, bool, Producer, bool); 8] = [
        // mode, frame, imu, producer, fault
        (0, 0, true, Producer::Throttle, false),
        (0, 0, false, Producer::Throttle, false),
        (0, 1, true, Producer::Throttle, false),
        (0, 1, false, Producer::Throttle, false),
        (1, 0, true, Producer::Balance, false),
        (1, 0, false, Producer::Throttle, true),
        (1, 1, true, Producer::Leader, false),
        (1, 1, false, Producer::Follower, false),
    ];
    for (mode, frame, imu, producer, fault) in rows {
        let sel = select_mode(mode, frame, imu);
        assert_eq!(sel.active, producer, "mode {mode} frame {frame} imu {imu}");
        assert_eq!(sel.fault, fault, "mode {mode} frame {frame} imu {imu}");
        // The boot seam and the arm-time re-apply resolve by the same table.
        let d = ControlDispatch::new(mode, frame, imu, 1, 2400);
        assert_eq!(d.mode(), producer);
        assert_eq!(d.mode_fault(), fault);
    }
    // An unknown frame byte is pivoting on both rows that can tell.
    assert_eq!(select_mode(1, 7, true).active, Producer::Balance);
    assert!(select_mode(1, 7, false).fault);
    // The discriminants are the CTRL_OBS control-mode byte.
    assert_eq!(
        [
            Producer::Throttle as u8,
            Producer::Balance as u8,
            Producer::Leader as u8,
            Producer::Follower as u8
        ],
        [0, 1, 2, 3]
    );
}

/// **A frame change at the arm is a producer change** (`specs/rover-one-imu.md` section 5:
/// `control.frame` "changes the ACTIVE mode and therefore already triggers
/// `re_apply_control_values`'s engagement reset"): Balance -> Leader on an IMU board reports the
/// change and replaces the records; the demoted Throttle -> Follower on a board without an IMU
/// reports it too and the demotion fault clears, because a rigid follower is not a demotion.
#[test]
fn a_frame_change_at_the_arm_is_a_producer_change() {
    let cfg = ThrottleConfig::default();
    let mut d = ControlDispatch::new(1, 0, true, 1, 2400);
    assert_eq!(d.mode(), Producer::Balance);
    assert!(
        d.re_apply_values(1, 1, true, 1, 2400),
        "pivoting -> rigid moved the producer"
    );
    assert_eq!(d.mode(), Producer::Leader);
    assert!(!d.mode_fault());
    assert!(
        !d.re_apply_values(1, 1, true, 1, 2400),
        "the same bytes again move nothing"
    );
    assert!(d.re_apply_values(1, 0, true, 1, 2400), "and back");
    assert_eq!(d.mode(), Producer::Balance);

    let mut d = ControlDispatch::new(1, 0, false, 1, 2400);
    assert_eq!(d.mode(), Producer::Throttle);
    assert!(d.mode_fault(), "pivoting Balance without an IMU is demoted");
    d.throttle_reference(&cfg, 32767, 0);
    assert_ne!(d.throttle.speed_rate_fixdt, 0, "records carry state");
    assert!(
        d.re_apply_values(1, 1, false, 1, 2400),
        "the same board becomes the follower"
    );
    assert_eq!(d.mode(), Producer::Follower);
    assert!(!d.mode_fault(), "which is not a demotion");
    assert_eq!(d.throttle.speed_rate_fixdt, 0, "records replaced wholesale");
}

/// **`CONTROL_MODE` is installed by the arm-time re-apply, and the throttle records are replaced
/// exactly when the ACTIVE mode changed** (`specs/integration.md`, "When a stored value takes
/// effect: the arm-time re-read"; the `switch_method` reset discipline, now keyed on the change
/// rather than on a switch call). The validation seam re-runs on every call, so a demotion raises
/// the fault and a corrected byte clears it as at boot.
///
/// The condition is the point: every arm runs this, so a reset on every call would wipe the
/// conditioning carries of a board that is merely re-arming in the mode it was already in.
#[test]
fn the_arm_re_apply_installs_the_mode_and_resets_the_records_only_on_a_change() {
    let cfg = ThrottleConfig::default();
    let mut d = ControlDispatch::new(0, 0, false, 1, 2400);
    assert_eq!(d.mode(), Producer::Throttle);
    assert!(!d.mode_fault());
    for _ in 0..10 {
        let _ = d.throttle_reference(&cfg, 32767, 0);
    }
    assert_ne!(d.throttle.speed_rate_fixdt, 0, "records carry state");

    // The same mode byte re-read: no change, so the carries stand (this is the common arm).
    let before = d.throttle;
    assert!(
        !d.re_apply_values(0, 0, true, 1, 2400),
        "the mode did not move"
    );
    assert_eq!(d.mode(), Producer::Throttle);
    assert_eq!(d.throttle.speed_rate_fixdt, before.speed_rate_fixdt);

    // A fresh byte: applies, records replaced, the seam re-validates (with IMU -> Balance, no
    // fault), and the change is REPORTED so the caller can replace the records it owns.
    assert!(d.re_apply_values(1, 0, true, 1, 2400), "the mode moved");
    assert_eq!(d.mode(), Producer::Balance);
    assert!(!d.mode_fault());
    assert_eq!(d.throttle.speed_rate_fixdt, 0, "records replaced wholesale");

    // A demoted request raises the fault exactly as at boot; an unknown byte lands Throttle.
    assert!(d.re_apply_values(1, 0, false, 1, 2400));
    assert_eq!(d.mode(), Producer::Throttle);
    assert!(d.mode_fault());
    // Balance-without-IMU and Throttle both RUN Throttle, so the active mode does not move between
    // them; the demotion fault still has to follow the byte, which is why the seam re-runs
    // unconditionally rather than under the `changed` branch.
    assert!(
        !d.re_apply_values(7, 0, true, 1, 2400),
        "Throttle either way"
    );
    assert_eq!(d.mode(), Producer::Throttle);
    assert!(!d.mode_fault(), "the unknown byte is not a demotion");
}

#[test]
fn the_rider_requirement_is_decoded_at_the_boot_seam_and_moved_only_by_the_arm_re_read() {
    // `specs/control.md` (i): the CONTROL_RIDER_REQUIRED byte is decoded by the constructor (0
    // waives, anything else requires: the default 1 and a corrupt byte both keep the rider gate).
    // The one thing that moves it is the arm-time re-read (`specs/integration.md`), which decodes a
    // fresh byte by the SAME rule, and the mode byte riding in the same call cannot disturb it.
    for (byte, required) in [(1u8, true), (0, false), (2, true), (0xFF, true)] {
        let mut d = ControlDispatch::new(1, 0, true, byte, 2400);
        assert_eq!(d.rider_required(), required, "byte {byte}");
        // A mode that moves under it leaves the decision alone: the two travel together through
        // the re-apply, and each is decoded by its own rule.
        for (m, imu) in [(0u8, true), (1, true), (1, false), (7, true)] {
            d.re_apply_values(m, 0, imu, byte, 2400);
            assert_eq!(d.rider_required(), required, "survives the mode byte {m}");
        }
        // The arm-time re-apply: the same byte vocabulary, the same decode as `new`.
        for (fresh, fresh_required) in [(1u8, true), (0, false), (2, true), (0xFF, true)] {
            d.re_apply_values(1, 0, true, fresh, 2400);
            assert_eq!(
                d.rider_required(),
                fresh_required,
                "re-applied byte {fresh} must decode as `new` decodes it"
            );
            assert_eq!(
                d.rider_required(),
                ControlDispatch::new(1, 0, true, fresh, 2400).rider_required(),
                "the two decodes cannot drift"
            );
        }
    }
}

/// **The arm-time value re-apply moves exactly three fields** (`specs/integration.md`, "When a
/// stored value takes effect: the arm-time re-read"): the mode, the rider requirement and the
/// battery floor. With the mode byte unchanged it moves the other two and NOTHING else, the
/// throttle producer's records included.
#[test]
fn the_arm_time_re_apply_moves_the_rider_and_floor_and_nothing_else() {
    // A board that asked for Balance without an IMU: demoted, with the fault raised, so a
    // re-apply that re-ran the validation seam would be visible either way.
    let mut d = ControlDispatch::new(1, 0, false, 1, 2400);
    assert_eq!(d.mode(), Producer::Throttle);
    assert!(d.mode_fault());
    // Give the throttle producer a non-default carry, so a reset of its records would show.
    let cfg = ThrottleConfig::default();
    d.throttle_reference(&cfg, 32767, -32767);
    let carry = |d: &ControlDispatch| {
        (
            d.throttle.steer_rate_fixdt,
            d.throttle.speed_rate_fixdt,
            d.throttle.steer_fixdt,
            d.throttle.speed_fixdt,
        )
    };
    let before = carry(&d);
    assert_ne!(before, (0, 0, 0, 0), "the fixture must have a live carry");

    // The stored mode byte, unchanged: still the demoted Balance request this board holds.
    assert!(
        !d.re_apply_values(1, 0, false, 0, 3000),
        "the mode did not move"
    );
    assert!(!d.rider_required(), "the fresh byte waives the requirement");
    assert!(!d.battery_ok(2999), "under the fresh floor");
    assert!(d.battery_ok(3000), "at it");
    assert_eq!(d.mode(), Producer::Throttle, "the demotion stands");
    assert!(d.mode_fault(), "and so does its fault");
    assert_eq!(carry(&d), before, "the throttle records are not reset");

    // And back, including the floor's "no floor" state.
    assert!(!d.re_apply_values(1, 0, false, 1, 0));
    assert!(d.rider_required());
    assert!(d.battery_ok(1), "a floor <= 0 is no floor");
    assert!(!d.battery_ok(0), "an UNKNOWN word still refuses");
    assert!(d.mode_fault());
    assert_eq!(carry(&d), before);
}

#[test]
fn end_to_end_both_modes_drive_the_shared_fsm_on_the_28500_contract() {
    // One engagement shell + output stage, two reference producers (spec (b)); the FSM is
    // mode-agnostic (throttle parameterizes the balance-only upright gate off with a zero
    // reference). Both modes' setpoints land on the +-28500 contract.
    let profile = GainProfile::of(RUN_PROFILE_A);

    // THROTTLE: condition full forward to the settled +-28500 word, feed it as the mirror.
    let cfg = ThrottleConfig::default();
    let mut d = ControlDispatch::new(0, 0, false, 1, 2400);
    let mut reference = 0i32;
    for _ in 0..300 {
        reference = d.throttle_reference(&cfg, 32767, 0).ref_left;
    }
    assert_eq!(reference, 28500);
    let mut st = FsmState::default();
    let mut inp = engage_fsm_inputs();
    inp.upright_ref = Fix::ZERO; // the balance-only gate parameterized off (mag 0 <= 2499)
    inp.smoothed_ref = reference;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Arming);
    let mut torque = 0i16;
    while st.sub_state == SubState::Arming {
        torque = fsm_step(&inp, &profile, &mut st);
    }
    assert_eq!(st.sub_state, SubState::Run);
    let final_torque = fsm_step(&inp, &profile, &mut st);
    assert_eq!(
        final_torque, 28500,
        "throttle reference enveloped to the cap, never above"
    );
    assert!(
        torque.abs() <= 28600,
        "soft-start stays within the transient envelope"
    );

    // BALANCE: the state-feedback word through the same shell (`specs/state-feedback.md` section
    // 2): a one-degree backward lean at the section-7 start gains, pitch term alone,
    // 3491 * 100 / 100 * 3900 / 2960 = 4599.
    let sel = select_mode(1, 0, true);
    assert_eq!(sel.active, Producer::Balance);
    let cfg = crate::sf::SfConfig {
        gains: [3491, 4002, 197, 926],
        speed_max: 0,
        speed_slew: 41,
        pos_clamp: 45,
    };
    let w = crate::sf::sf_step(
        &cfg,
        &crate::sf::SfInputs {
            pitch_word: 100,
            scale: 2960,
            ..Default::default()
        },
        &mut crate::sf::SfState::default(),
    );
    assert_eq!(w, 4599);
    let mut st = FsmState::default();
    let mut inp = engage_fsm_inputs();
    inp.smoothed_ref = w as i32;
    let _ = fsm_step(&inp, &profile, &mut st);
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    let torque = fsm_step(&inp, &profile, &mut st);
    assert_eq!(torque, 4599, "the balance word rides the same output stage");
    assert!(torque.abs() <= 28500);
}

// ---- the live gain shadow and the tune lane (`specs/state-feedback.md` section 6) ----

/// A shadow's flat store row from its two halves: the four gains, then the reference row.
fn row(g: [i16; 4], r: [i16; 2]) -> [i16; 6] {
    [g[0], g[1], g[2], g[3], r[0], r[1]]
}

#[test]
fn an_unstaged_shadow_is_inert() {
    // `specs/state-feedback.md` section 6, "Defaults are the inert state": zero gains (a zero
    // word in balance mode), the stick dead (`speed_max` 0), the default slew and maxima.
    let s = GainShadow::default();
    assert_eq!(s.gains(), [0; 4]);
    assert_eq!(s.reference(), [0, 41]);
    assert_eq!(REFERENCE_DEFAULT, [0, 41]);
    assert_eq!(DEFAULT_GAIN_MAX, [20000, 20000, 2000, 10000]);
    for i in 0..4 {
        assert_eq!(s.stored(SF_GAINS_FIELD, i), Some(0));
    }
    assert_eq!(s.stored(REFERENCE_FIELD, 1), Some(41));
}

#[test]
fn the_tune_seam_owns_the_allowlist_and_the_ranges() {
    // The lane's allowlist is exactly 0x79 indices 0..3 and 0x7B indices 0..1; the gain row is
    // bounded by the stored maxima and the reference row by the compiled ones.
    assert_eq!((SF_GAINS_FIELD, REFERENCE_FIELD), (0x79, 0x7B));
    let mut s = GainShadow::default();
    for i in 0..4u8 {
        let max = DEFAULT_GAIN_MAX[i as usize];
        assert_eq!(s.set(SF_GAINS_FIELD, i, max), Ok(()));
        assert_eq!(s.get(SF_GAINS_FIELD, i), Some(max));
        assert_eq!(
            s.set(SF_GAINS_FIELD, i, max + 1),
            Err(TuneError::OutOfRange)
        );
        assert_eq!(
            s.set(SF_GAINS_FIELD, i, GAIN_MIN - 1),
            Err(TuneError::OutOfRange)
        );
        assert_eq!(
            s.get(SF_GAINS_FIELD, i),
            Some(max),
            "a refusal moves nothing"
        );
    }
    for i in 0..2u8 {
        let (lo, hi) = (REFERENCE_MIN[i as usize], REFERENCE_MAX[i as usize]);
        assert_eq!(s.set(REFERENCE_FIELD, i, hi), Ok(()));
        assert_eq!(s.set(REFERENCE_FIELD, i, lo), Ok(()));
        assert_eq!(
            s.set(REFERENCE_FIELD, i, hi + 1),
            Err(TuneError::OutOfRange)
        );
        assert_eq!(
            s.set(REFERENCE_FIELD, i, lo - 1),
            Err(TuneError::OutOfRange)
        );
    }
    assert_eq!(REFERENCE_MIN, [0, 1]);
    assert_eq!(REFERENCE_MAX, [512, 4096]);
    assert_eq!(s.reference(), REFERENCE_MIN);
    // Past the row, a retired id, the position clamp (an arm-time value, not a lane key) and the
    // maxima field are all unknown keys.
    for (id, i) in [
        (SF_GAINS_FIELD, 4),
        (REFERENCE_FIELD, 2),
        (0x71, 0),
        (0x76, 0),
        (0x7A, 0),
        (0x7C, 0),
    ] {
        assert_eq!(s.set(id, i, 1), Err(TuneError::UnknownKey), "{id:#x}/{i}");
        assert_eq!(s.get(id, i), None);
        assert_eq!(s.stored(id, i), None);
    }
}

#[test]
fn a_stored_value_outside_its_range_is_clamped_before_it_reaches_the_law() {
    let s = GainShadow::of_stored(
        row([-5, 30000, 2500, 300], [-1, 0]),
        [20000, 20000, 2000, 10000],
    );
    assert_eq!(s.gains(), [0, 20000, 2000, 300]);
    assert_eq!(s.reference(), [0, 1], "speed_slew floors at 1");
    let s = GainShadow::of_stored(row([0; 4], [600, 5000]), DEFAULT_GAIN_MAX);
    assert_eq!(s.reference(), REFERENCE_MAX);
}

#[test]
fn a_stored_maximum_bounds_the_boot_clamp_the_lane_and_the_reconcile() {
    let max = [1000, 900, 50, 40];
    let mut s = GainShadow::of_stored(row([3491, 4002, 197, 926], [80, 41]), max);
    assert_eq!(s.gains(), max, "the boot clamp");
    assert_eq!(s.set(SF_GAINS_FIELD, 2, 51), Err(TuneError::OutOfRange));
    assert_eq!(s.set(SF_GAINS_FIELD, 2, 50), Ok(()));
    s.reconcile(row([3491, 10, 197, 926], [80, 41]));
    assert_eq!(s.gains(), [1000, 10, 50, 40], "the reconcile clamps too");
    // A negative maximum reads as zero rather than an empty range.
    let s = GainShadow::of_stored(row([5; 4], [0, 41]), [-1, 5, 5, 5]);
    assert_eq!(s.gains(), [0, 5, 5, 5]);
}

#[test]
fn the_arm_time_maxima_re_apply_clamps_live_and_leaves_stored_as_read() {
    let mut s = GainShadow::of_stored(row([3491, 4002, 197, 926], [80, 41]), DEFAULT_GAIN_MAX);
    s.set(SF_GAINS_FIELD, 0, 5000).unwrap();
    s.re_apply_max([4000, 20000, 100, 10000]);
    assert_eq!(s.gains(), [4000, 4002, 100, 926]);
    assert_eq!(
        s.stored(SF_GAINS_FIELD, 2),
        Some(197),
        "stored is what flash held"
    );
    assert_eq!(s.set(SF_GAINS_FIELD, 2, 101), Err(TuneError::OutOfRange));
    // The reference row's compiled maxima are not moved by an arm.
    assert_eq!(s.set(REFERENCE_FIELD, 0, 512), Ok(()));
    // Unchanged flash after the shrink: nothing moves, because the comparison is against the
    // unclamped-by-the-new-bound record.
    s.reconcile(row([3491, 4002, 197, 926], [80, 41]));
    assert_eq!(s.gains(), [4000, 4002, 100, 926]);
    // A maximum that grows back moves no live value by itself.
    s.re_apply_max(DEFAULT_GAIN_MAX);
    assert_eq!(s.gains(), [4000, 4002, 100, 926]);
}

#[test]
fn a_reconcile_follows_flash_only_where_flash_moved() {
    let mut s = GainShadow::of_stored(row([3491, 4002, 197, 926], [80, 41]), DEFAULT_GAIN_MAX);
    s.set(SF_GAINS_FIELD, 0, 3600).unwrap();
    s.set(REFERENCE_FIELD, 1, 60).unwrap();
    // A save of an unrelated field re-reads the same flash: the live tunes stay.
    s.reconcile(row([3491, 4002, 197, 926], [80, 41]));
    assert_eq!(s.gains(), [3600, 4002, 197, 926]);
    assert_eq!(s.reference(), [80, 60]);
    // A save of gx and speed_max moves exactly those two.
    s.reconcile(row([3491, 4002, 250, 926], [100, 41]));
    assert_eq!(s.gains(), [3600, 4002, 250, 926]);
    assert_eq!(s.reference(), [100, 60]);
    assert_eq!(s.stored(SF_GAINS_FIELD, 2), Some(250));
}

/// The top of the RETIRED PID gain table (stock x3, `{20000, 10000, 1000}`): the furthest the
/// engagement machine's dead triple was ever ramped, kept as the ramp vectors' far target.
fn top_of_range() -> GainTriple {
    GainTriple::new(20000, 10000, 1000)
}

/// The passes [`ramp::ramp_toward`] needs from `from` to `to` at these inputs: the slowest gain's
/// `ceil(distance / max(cap, 1))`.
fn passes_needed(from: GainTriple, to: GainTriple, scale: i16, kd: Fix) -> u32 {
    let caps = ramp::caps(scale, kd);
    let d = [
        (to.kp - from.kp).unsigned_abs(),
        (to.bk - from.bk).unsigned_abs(),
        (to.pr - from.pr).unsigned_abs(),
    ];
    (0..3).map(|i| d[i].div_ceil(caps[i].max(1))).max().unwrap()
}

/// The derivation pinned as arithmetic: at every battery word from 757 (below which `bk`'s floor
/// of 1 exceeds its derived cap) to `i16::MAX`, and every `kd` up to the floor's limit at that
/// word, the three capped steps' exact torque contributions at the worst-case inputs stay inside
/// their shares, and the shares plus the truncation slack inside [`ramp::SLEW_LIMIT`]. Below
/// 757 down to 586 (the module doc's violation threshold) the floored steps' SUM plus the slack
/// stays inside the limit, and at 585 it does not.
#[test]
fn the_ramp_caps_keep_each_gain_inside_its_share() {
    assert_eq!(
        ramp::KP_SHARE + ramp::BK_SHARE + ramp::PR_SHARE,
        235,
        "the shares leave 15 counts for the truncations"
    );
    // Each check is `cap * input / divisor * 3900 / scale <= share`, cross-multiplied so it is
    // exact.
    let within = |cap: u32, input: i64, div: i64, share: i32, scale: i64| {
        cap as i64 * input * 3900 <= share as i64 * div * scale
    };
    // The whole bound, exact (cross-multiplied by `scale * 10000`): the floored steps' torque
    // contributions plus two pre-divide truncations of 3900/scale each, one at the divide and one
    // in the IIR.
    let total_within = |scale: i64, kd: i64| {
        let c = ramp::caps(scale as i16, Fix::from_num(kd)).map(|c| c.max(1) as i64);
        let pr = if kd == 0 { 0 } else { c[2] * kd * 3900 * 100 };
        c[0] * ramp::PP_BOUND as i64 * 3900 * 100
            + c[1] * ramp::BV_BOUND as i64 * 3900
            + pr
            + 2 * 3900 * 10000
            + 2 * scale * 10000
            <= ramp::SLEW_LIMIT as i64 * scale * 10000
    };
    for scale in 586i64..757 {
        let kd_limit = scale * 1000 / 3900;
        for kd in [0i64, 1, 7, 100, kd_limit] {
            assert!(total_within(scale, kd), "scale {scale} kd {kd}");
        }
    }
    assert!(
        !total_within(585, 1),
        "585 is the first word below the threshold"
    );
    let mut scale = 757i64;
    while scale <= i16::MAX as i64 {
        let kd_limit = scale * 1000 / 3900; // the largest kd whose pr cap is >= 1
        for kd in [0i64, 1, 7, 100, kd_limit] {
            let c = ramp::caps(scale as i16, Fix::from_num(kd));
            assert!(
                c[0] >= 1 && c[1] >= 1 && c[2] >= 1,
                "scale {scale} kd {kd}: {c:?}"
            );
            assert!(within(
                c[0],
                ramp::PP_BOUND as i64,
                100,
                ramp::KP_SHARE,
                scale
            ));
            assert!(within(
                c[1],
                ramp::BV_BOUND as i64,
                10000,
                ramp::BK_SHARE,
                scale
            ));
            if kd != 0 {
                assert!(within(c[2], kd, 100, ramp::PR_SHARE, scale), "{scale} {kd}");
            }
        }
        // The shares plus the truncation slack (two truncations before the divide, one at it,
        // one in the IIR): 235 + 2 * 3900 / scale + 2 <= 250.
        assert!(235 * scale + 2 * 3900 + 2 * scale <= 250 * scale);
        assert!(total_within(scale, kd_limit), "scale {scale}");
        scale += 1;
    }
    // A fractional kd is charged at its ceiling.
    assert_eq!(
        ramp::caps(2400, Fix::from_num(0.25)),
        ramp::caps(2400, Fix::from_num(1))
    );
    assert_eq!(
        ramp::caps(2400, Fix::from_num(-100)),
        ramp::caps(2400, Fix::from_num(100))
    );
    // At the LEV50-8 floor and a full pack.
    assert_eq!(ramp::caps(2400, Fix::from_num(100)), [4, 3, 6]);
    assert_eq!(ramp::caps(3300, Fix::from_num(100)), [6, 4, 8]);
    assert_eq!(ramp::caps(2400, Fix::ZERO)[2], u32::MAX);
}

/// Convergence: every differing gain moves at least one count per pass (the floor), never
/// overshoots, ramps down as well as up, and the derived pass counts hold.
#[test]
fn the_ramp_converges_and_never_overshoots() {
    // The standby seed to profile A at 2400: kp's 5,950 counts at 4 per pass, 1,488 passes.
    assert_eq!(
        passes_needed(STANDBY_SET, RUN_PROFILE_A, 2400, Fix::ZERO),
        1488
    );
    for (from, to, scale, kd) in [
        (STANDBY_SET, RUN_PROFILE_A, 2400i16, Fix::ZERO),
        (RUN_PROFILE_A, top_of_range(), 2400, Fix::from_num(100)),
        (
            top_of_range(),
            GainTriple::new(0, 0, 0),
            3300,
            Fix::from_num(3),
        ),
        // UNKNOWN battery (scale 0): every cap computes 0, and the floor still converges.
        (RUN_PROFILE_A, PROFILE_B, 0, Fix::from_num(100)),
    ] {
        let want = passes_needed(from, to, scale, kd);
        let mut g = from;
        let mut n = 0;
        while g != to {
            let next = ramp::ramp_toward(g, to, scale, kd);
            for (a, b, t) in [
                (g.kp, next.kp, to.kp),
                (g.bk, next.bk, to.bk),
                (g.pr, next.pr, to.pr),
            ] {
                if a != t {
                    assert!(
                        (b - a).signum() == (t - a).signum(),
                        "moves toward the target"
                    );
                    assert!((t - b).signum() != -(t - a).signum(), "never past it");
                } else {
                    assert_eq!(b, t, "a converged gain holds");
                }
            }
            g = next;
            n += 1;
            assert!(
                n <= want,
                "{from:?} -> {to:?} at {scale}: past {want} passes"
            );
        }
        assert_eq!(n, want);
    }
    // Scale 0: kp 3,000 counts at the floor of 1.
    assert_eq!(passes_needed(RUN_PROFILE_A, PROFILE_B, 0, Fix::ZERO), 3000);
}

// ---- the battery word's UNKNOWN (`specs/sensing-and-safety.md`, "The battery word") ----

/// `battery_ok` joins the engage conjunction: with every other gate open, a refused battery
/// does not engage, and a permitted one does.
#[test]
fn fsm_unknown_battery_blocks_engage() {
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = FsmState::default();
    let inp = FsmInputs {
        battery_ok: false,
        ..engage_fsm_inputs()
    };
    for _ in 0..10 {
        let torque = fsm_step(&inp, &profile, &mut st);
        assert_eq!(st.sub_state, SubState::Idle, "unknown battery: no engage");
        assert_eq!(torque, 0);
    }
    let _ = fsm_step(&engage_fsm_inputs(), &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Arming, "known battery: engages");
}

/// The low-battery floor's decision (`specs/sensing-and-safety.md`, "The low-battery floor"):
/// `battery != 0 && (floor <= 0 || battery >= floor)`, the floor taken as written.
#[test]
fn battery_ok_is_known_and_at_or_above_the_floor() {
    let d = ControlDispatch::new(1, 0, true, 1, 2400);
    assert!(!d.battery_ok(0), "UNKNOWN");
    assert!(!d.battery_ok(2399), "under the default floor");
    assert!(d.battery_ok(2400), "at it");
    assert!(d.battery_ok(i16::MAX));
    assert!(
        !d.battery_ok(-1),
        "a negative word is under a positive floor"
    );
    // 0 = no floor: any nonzero word. A negative floor behaves as 0.
    for floor in [0i16, -1, i16::MIN] {
        let d = ControlDispatch::new(1, 0, true, 1, floor);
        assert!(!d.battery_ok(0), "floor {floor}: UNKNOWN still refuses");
        for word in [1i16, 2399, i16::MAX] {
            assert!(d.battery_ok(word), "floor {floor}, word {word}");
        }
    }
    // No clamp beyond the type: a floor above any reachable word refuses every word.
    let d = ControlDispatch::new(1, 0, true, 1, i16::MAX);
    assert!(!d.battery_ok(i16::MAX - 1));
    // The floor moves only with its own byte: a mode change carried by the same re-apply call
    // leaves it as it was.
    let mut d = ControlDispatch::new(1, 0, true, 1, 2400);
    assert!(d.re_apply_values(0, 0, true, 1, 2400));
    assert!(d.re_apply_values(1, 0, true, 1, 2400));
    assert!(!d.battery_ok(2399));
}

/// `battery_ok` is read only by the engage conjunction: a RUN machine whose battery term goes
/// false (the word sagging under the floor) stays in RUN.
#[test]
fn fsm_battery_refusal_in_run_never_disengages() {
    let profile = GainProfile::of(RUN_PROFILE_A);
    let mut st = engage_to_run(&profile);
    let sagged = FsmInputs {
        battery_ok: false,
        ..engage_fsm_inputs()
    };
    for k in 0..500 {
        let _ = fsm_step(&sagged, &profile, &mut st);
        assert_eq!(st.sub_state, SubState::Run, "RUN holds at tick {k}");
    }
}

#[test]
fn d2iz_bit_shift_matches_to_num() {
    // Pins the shrink-round spelling of `q_to_int_d2iz` (raw bits shifted down after the
    // round-toward-zero) to the `to_num::<i64>()` form it replaced, over the edges and a sweep.
    let check = |x: Fix| {
        assert_eq!(
            crate::helpers::q_to_int_d2iz(x),
            x.round_to_zero().to_num::<i64>(),
            "x = {x}"
        );
    };
    for x in [
        Fix::MIN,
        Fix::MAX,
        Fix::ZERO,
        Fix::DELTA,
        -Fix::DELTA,
        Fix::from_num(-1.5),
        Fix::from_num(1.5),
        Fix::from_num(-150) / Fix::from_num(100),
    ] {
        check(x);
    }
    let mut bits: i64 = 0x1234_5678_9abc_def1;
    for _ in 0..10_000 {
        bits = bits
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        check(Fix::from_bits(bits));
        check(Fix::from_bits(bits >> 20));
    }
}

// --- The rigid frame's turn and the steering split (`specs/rover-one-imu.md` section 5) -------

#[test]
fn drive_turn_seam_clamps_both_indices() {
    use crate::drive::{DriveTurn, TURN_SLEW_MAX, TURN_SLEW_MIN};
    use base::TORQUE_FULL_SCALE;
    let d = DriveTurn::new(3000, 24);
    assert_eq!((d.turn_max(), d.turn_slew()), (3000, 24));
    assert_eq!(DriveTurn::new(28_500, 24).turn_max(), 28_500);
    assert_eq!(
        DriveTurn::new(i16::MAX, 24).turn_max() as i32,
        TORQUE_FULL_SCALE,
        "the ceiling is the torque word, which the saturation rule then protects"
    );
    assert_eq!(DriveTurn::new(-1, 24).turn_max(), 0, "negative is disabled");
    assert_eq!(DriveTurn::new(i16::MIN, 24).turn_max(), 0);

    assert_eq!(DriveTurn::new(3000, 0).turn_slew(), TURN_SLEW_MIN);
    assert_eq!(DriveTurn::new(3000, -7).turn_slew(), 1);
    assert_eq!(DriveTurn::new(3000, 1).turn_slew(), 1);
    assert_eq!(DriveTurn::new(3000, 1000).turn_slew(), 1000);
    assert_eq!(DriveTurn::new(3000, 1001).turn_slew(), TURN_SLEW_MAX);

    // The unstaged board, which is also the store's defaults: disabled, with the rate the rover's
    // first staged value traverses in half a second.
    assert_eq!(DriveTurn::default(), DriveTurn::new(0, 24));
    assert_eq!(DriveTurn::default().turn_max(), 0);
}

#[test]
fn drive_turn_cmd_is_turn_max_at_the_rails_and_truncates_toward_zero() {
    use crate::drive::DriveTurn;
    let d = DriveTurn::new(3000, 24);
    assert_eq!(d.turn_cmd(32767), 3000);
    assert_eq!(d.turn_cmd(-32767), -3000);
    assert_eq!(d.turn_cmd(0), 0);
    // Symmetric truncation: the same magnitude either side of zero.
    for steer in [1i16, 7, 123, 5000, 16384, 32766] {
        assert_eq!(d.turn_cmd(steer), -d.turn_cmd(-steer), "steer {steer}");
    }
    // Half stick is half the bound, within the truncation.
    assert_eq!(d.turn_cmd(16384), (16384 * 3000) / 32767);
    // Disabled: every stick position is zero, which is what makes the default inert.
    let off = DriveTurn::default();
    for steer in [i16::MIN, -1000, 0, 1000, i16::MAX] {
        assert_eq!(off.turn_cmd(steer), 0);
    }
}

#[test]
fn drive_turn_slews_by_exactly_turn_slew_both_ways() {
    use crate::drive::DriveTurn;
    // turn_max 3000, slew 24: full stick takes 125 ticks up (0.5 s at 250 Hz) and 125 back.
    let d = DriveTurn::new(3000, 24);
    let mut turn = 0;
    for k in 1..=125 {
        let prev = turn;
        assert_eq!(d.step(32767, &mut turn), 24 * k);
        assert_eq!(turn - prev, 24);
    }
    assert_eq!(turn, 3000);
    assert_eq!(d.step(32767, &mut turn), 3000, "holds at the command");
    // A stale drive command arrives as steer 0, so the slew IS the decay ramp.
    for k in 1..=125 {
        assert_eq!(d.step(0, &mut turn), 3000 - 24 * k);
    }
    assert_eq!(turn, 0);
    // A step smaller than the slew lands exactly, no overshoot.
    let mut turn = 2990;
    assert_eq!(d.step(32767, &mut turn), 3000);
    // The ceiling rate crosses the whole word in 29 ticks.
    let fast = crate::drive::DriveTurn::new(28_500, 1000);
    let mut turn = 0;
    for _ in 0..29 {
        fast.step(32767, &mut turn);
    }
    assert_eq!(turn, 28_500);
}

/// **The split preserves the common torque exactly, everywhere, including saturation**
/// (`specs/rover-one-imu.md` section 8: the property the saturation rule exists to provide).
/// `left + right == 2T` is the fore-aft torque on the body, which is what holds the machine up;
/// the app's throttle mixer gives that up to keep the turn, and here the turn is what gives way.
#[test]
fn the_wheel_split_keeps_the_common_torque_across_the_whole_range() {
    use crate::drive::wheel_split;
    use base::TORQUE_FULL_SCALE;
    let full = TORQUE_FULL_SCALE as i16;
    for t in [-full, -28_000, -14_250, -1, 0, 1, 14_250, 28_000, full] {
        for turn in [
            -32_767i32, -28_500, -14_250, -1, 0, 1, 14_250, 28_500, 32_767,
        ] {
            for side_right in [false, true] {
                let (own, peer) = wheel_split(t, turn, side_right);
                assert_eq!(
                    own as i32 + peer as i32,
                    2 * t as i32,
                    "t {t}, turn {turn}, right {side_right}"
                );
                assert!(
                    own.unsigned_abs() as i32 <= TORQUE_FULL_SCALE
                        && peer.unsigned_abs() as i32 <= TORQUE_FULL_SCALE,
                    "neither wheel clips: t {t}, turn {turn}"
                );
            }
        }
    }
}

/// **The turn is what gives way, and the sides are the right way round.** Positive steer turns
/// RIGHT, so the right wheel runs less FORWARD than the left, which in this backward-positive
/// word (`specs/control.md` (a), "Which sign is forward") is the more positive demand; at full
/// balance authority the turn is zero, because a machine spending all its torque on not falling
/// has none to spare for yawing.
#[test]
fn the_split_gives_up_the_turn_and_never_the_common_torque() {
    use crate::drive::wheel_split;
    use base::TORQUE_FULL_SCALE;
    // Unsaturated: the turn arrives in full, and the right wheel is the one eased off forward.
    assert_eq!(wheel_split(10_000, 2_000, true), (12_000, 8_000));
    assert_eq!(
        wheel_split(10_000, 2_000, false),
        (8_000, 12_000),
        "the leader on the left wall keeps the OTHER half"
    );
    // A negative turn (steer left) mirrors it.
    assert_eq!(wheel_split(10_000, -2_000, true), (8_000, 12_000));

    // Saturating: at 28,000 of common torque only 500 of turn fits, and the commanded 5,000 is
    // cut to it rather than the pair being offset down.
    assert_eq!(wheel_split(28_000, 5_000, true), (28_500, 27_500));
    assert_eq!(wheel_split(-28_000, 5_000, true), (-27_500, -28_500));
    // At full authority the turn is zero, both wheels carry T, and the machine goes straight.
    let full = TORQUE_FULL_SCALE as i16;
    assert_eq!(wheel_split(full, 28_500, true), (full, full));
    assert_eq!(wheel_split(-full, -28_500, false), (-full, -full));
    // Disabled (`turn_max` 0 keeps the carry at 0): both halves are the setpoint, which is what
    // makes a board with no turn staged byte-identical to one before the field existed.
    for t in [-28_500i16, -1, 0, 1, 28_500] {
        assert_eq!(wheel_split(t, 0, true), (t, t));
        assert_eq!(wheel_split(t, 0, false), (t, t));
    }
}

/// **A right turn drives the LEFT wheel forward, in the frame the wheels actually use**
/// (`specs/control.md` (a), "Which sign is forward"): negative demand is forward, so the wheel
/// that leads a right turn is the one with the negative number. Taken at `t = 0`, a machine
/// balancing in place and asked to yaw, where the common torque cannot hide the sense of it.
///
/// This is the one the rover disproved on 2026-10-23 and the reason the app carried a negation:
/// the split was written as "the right wheel gets less torque", which is less POSITIVE, which in
/// this word is more forward, so a positive steer yawed the machine left.
#[test]
fn a_positive_steer_yaws_right_in_the_backward_positive_word() {
    use crate::drive::wheel_split;
    for (side_right, own_is_right) in [(true, true), (false, false)] {
        let (own, peer) = wheel_split(0, 2_000, side_right);
        let (right, left) = if own_is_right {
            (own, peer)
        } else {
            (peer, own)
        };
        assert_eq!(right, 2_000, "the right wheel is pushed backward");
        assert_eq!(left, -2_000, "and the left wheel forward: a right turn");
    }
}

/// The measurement's frame: the ISR counts rotor rotation, so the sign of machine-forward in the
/// word is the wheel's SIDE (a right wheel and a left wheel turn opposite ways for one forward),
/// measured negative on the right wheel. The loop works in a forward-positive frame, so the mirror
/// is applied at the one place that reads the word.
#[test]
fn the_wheel_word_is_read_into_a_forward_positive_frame_through_the_side() {
    assert_eq!(
        wheel_forward(-40, true),
        40,
        "right wheel: forward counts down (measured 2026-10-24)"
    );
    assert_eq!(wheel_forward(40, true), -40);
    assert_eq!(wheel_forward(40, false), 40, "left wheel: the mirror");
    assert_eq!(wheel_forward(-40, false), -40);
    assert_eq!(
        wheel_forward(0, true),
        0,
        "a stationary wheel is 0 either way"
    );
    assert_eq!(wheel_forward(0, false), 0);
}
