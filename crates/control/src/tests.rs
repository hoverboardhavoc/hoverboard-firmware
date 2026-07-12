//! Host tests (`specs/control.md`, section (f) validation). Pure math/logic; no hardware. Where
//! the contract gives an exact constant, the assertion checks the exact value.
//!
//! The recovered archive vectors come across byte-identical (this slice: the gain-schedule and
//! shift-helper tests); the clamp/ramp helper tests are NEW (the archive carried no vectors for
//! them), hand-derived from the recovered Section 8.2-8.5 semantics and marked as such.

use crate::config::{GainProfile, RUN_PROFILE_A, STANDBY_SET};
use crate::helpers::{
    clamp, clamp_sym, iabs, ramp_step, shr_round_to_zero, RampRecord, RAMP_COUNTER_CAP,
};

// ---- gain schedule (Section 6) ----

#[test]
fn gain_schedule_steps_standby_to_run_on_pad() {
    // Rider/pad flag false -> Profile B; true -> Profile A {6000,2000,40}.
    let pa = crate::config::select_profile(true);
    assert_eq!(pa, GainProfile::profile_a());
    assert_eq!(pa.as_triple(), RUN_PROFILE_A);
    let pb = crate::config::select_profile(false);
    assert_eq!(pb, GainProfile::profile_b());
    // The standby seed is {50,20,0} and the RUN/Profile-A triple is {6000,2000,40}.
    assert_eq!(STANDBY_SET, crate::config::GainTriple::new(50, 20, 0));
    assert_eq!(
        RUN_PROFILE_A,
        crate::config::GainTriple::new(6000, 2000, 40)
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
