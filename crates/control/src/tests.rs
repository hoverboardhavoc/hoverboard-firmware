//! Host tests (`specs/control.md`, section (f) validation). Pure math/logic; no hardware. Where
//! the contract gives an exact constant, the assertion checks the exact value.
//!
//! The recovered archive vectors come across byte-identical (slice 1: the gain-schedule and
//! shift-helper tests; slice 2: the five shaping tests; slice 3: the seven PID/IIR tests,
//! modulo the I32F32 -> base::fixed::Fix type rename). The clamp/ramp helper tests, the shaping
//! f64 steer-scale reference, and the PID d2iz + f64-reference tests are NEW (the archive
//! carried no vectors for them), hand-derived and marked as such.

use crate::config::{pid as pidc, GainProfile, RUN_PROFILE_A, STANDBY_SET};
use crate::helpers::{
    clamp, clamp_sym, iabs, ramp_step, shr_round_to_zero, RampRecord, RAMP_COUNTER_CAP,
};
use crate::pid::{balance_pid, IirCarry, PidInputs};
use crate::shaping::{shape_pitch_target, ShapingInputs, ShapingState};
use base::fixed::Fix;

/// Local f64-reference tolerance check (the assert_close discipline of specs/control.md (f);
/// base's own helper is `#[cfg(test)]`-internal to base, so the discipline is restated here).
fn assert_close(value: f64, reference: f64, tol: f64) {
    let diff = (value - reference).abs();
    assert!(
        diff <= tol,
        "assert_close failed: value={value} reference={reference} diff={diff} tol={tol}"
    );
}

// ---- helpers to build PID inputs with the RUN gains ----

fn run_pid_inputs() -> PidInputs {
    PidInputs {
        bv: 0,
        bk: RUN_PROFILE_A.bk, // 2000
        pp: 0,
        kp: RUN_PROFILE_A.kp, // 6000
        pr: 0,
        kd: Fix::from_num(1),
        off: 0,
        scale: 4176, // 41.76 V
    }
}

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

// ---- pitch shaping (Section 4) ----

#[test]
fn shaping_fb_is_absolute_value() {
    // fb = abs((roll_a - roll_b)/10). Negative and positive differentials of equal magnitude give
    // the same base.
    let mut st_pos = ShapingState::default();
    let mut st_neg = ShapingState::default();
    let pos = shape_pitch_target(
        &ShapingInputs {
            roll_a: 100,
            roll_b: 0,
            steer: 0,
            role_right: false,
        },
        &mut st_pos,
    );
    let neg = shape_pitch_target(
        &ShapingInputs {
            roll_a: 0,
            roll_b: 100,
            steer: 0,
            role_right: false,
        },
        &mut st_neg,
    );
    // base = fb*18 + 3500 ; fb = 10 -> base = 180 + 3500 = 3680 ; steer 0 -> target 0 -> slew to 0.
    assert_eq!(pos, neg);
}

#[test]
fn shaping_base_and_steer_clamp() {
    // fb = abs((100-0)/10) = 10 -> base = 10*18+3500 = 3680. steer = 100 -> steer_term =
    // round(100*1.5) = 150, clamped to +-3680 -> 150. Then +-7000 clamp (no-op), slew from 0
    // limits to +250. So first tick = 150 (<=250) -> 150.
    let mut st = ShapingState::default();
    let out = shape_pitch_target(
        &ShapingInputs {
            roll_a: 100,
            roll_b: 0,
            steer: 100,
            role_right: false,
        },
        &mut st,
    );
    assert_eq!(out, 150);
}

#[test]
fn shaping_steer_sign_flips_with_role() {
    let mut st_l = ShapingState::default();
    let mut st_r = ShapingState::default();
    let left = shape_pitch_target(
        &ShapingInputs {
            roll_a: 0,
            roll_b: 0,
            steer: 100,
            role_right: false,
        },
        &mut st_l,
    );
    let right = shape_pitch_target(
        &ShapingInputs {
            roll_a: 0,
            roll_b: 0,
            steer: 100,
            role_right: true,
        },
        &mut st_r,
    );
    assert_eq!(left, 150);
    assert_eq!(right, -150, "role flips steer sign");
}

#[test]
fn shaping_slew_caps_per_tick_delta() {
    // Drive a large steer so the target wants to jump far; the per-tick change must cap at +-250.
    let mut st = ShapingState::default();
    // big base so the steer term isn't clamped first.
    let inp = ShapingInputs {
        roll_a: 10000,
        roll_b: 0,
        steer: 1000,
        role_right: false,
    };
    let t1 = shape_pitch_target(&inp, &mut st);
    assert_eq!(t1, 250, "first tick slews up by at most +250 from 0");
    let t2 = shape_pitch_target(&inp, &mut st);
    assert_eq!(t2, 500, "second tick +250 more");
}

#[test]
fn shaping_absolute_clamp_7000() {
    // Force a target above 7000 and confirm the +-7000 clamp, observed across enough ticks that
    // the slew reaches it.
    // (Archive form was default-then-assign; clippy::field_reassign_with_default forces the
    // struct-literal rewrite. Same state: last_target 6900, near the cap already.)
    let mut st = ShapingState {
        last_target: 6900,
        ..Default::default()
    };
    let inp = ShapingInputs {
        roll_a: 30000,
        roll_b: 0,
        steer: 30000,
        role_right: false,
    };
    let t = shape_pitch_target(&inp, &mut st);
    // target before slew clamps to +7000; slew from 6900 by +100 -> 7000.
    assert_eq!(t, 7000);
}

#[test]
fn shaping_steer_scale_matches_f64_reference_exhaustively() {
    // NEW (specs/control.md section (f): the x1.5 steer scale is a flagged-fractional stock
    // float). The stock computes (double)(steer * 3) * 0.5 and converts with the EABI d2iz,
    // which TRUNCATES toward zero (slice-2 audit, board20 decompile FUN_08006524 ->
    // FUN_080006e0: no rounding increment, sign applied after). Rust's `as i32` f64 cast IS
    // truncate-toward-zero, so it is the honest d2iz model, and the agreement is EXACT
    // (steer*3 fits 17 bits so the double is lossless and *0.5 is an exponent step): equality,
    // not assert_close, over the ENTIRE i16 steer range.
    for steer in i16::MIN..=i16::MAX {
        let scaled = (steer as i32) * 3;
        let reference = ((scaled as f64) * 0.5) as i32;
        assert_eq!(
            crate::shaping::trunc_half(scaled),
            reference,
            "steer = {steer}"
        );
    }
}

#[test]
fn shaping_odd_steer_truncates_toward_zero() {
    // NEW (slice-2 audit): odd steer values are where truncation and rounding diverge (steer*3
    // odd -> a x.5 half). steer = 101 -> trunc(151.5) = 151 (round-half would give 152); the
    // negative side truncates TOWARD ZERO: steer = -101 -> trunc(-151.5) = -151 (a floor would
    // give -152). Both within the +-base clamp (3680) and the +-250 first-tick slew.
    let mut st = ShapingState::default();
    let out = shape_pitch_target(
        &ShapingInputs {
            roll_a: 100,
            roll_b: 0,
            steer: 101,
            role_right: false,
        },
        &mut st,
    );
    assert_eq!(out, 151);
    let mut st_n = ShapingState::default();
    let out_n = shape_pitch_target(
        &ShapingInputs {
            roll_a: 100,
            roll_b: 0,
            steer: -101,
            role_right: false,
        },
        &mut st_n,
    );
    assert_eq!(out_n, -151);
}

// ---- balance PID + IIR (Section 3.2) ----

#[test]
fn level_pitch_run_gains_zero_torque() {
    // A level pitch (pp=0, bv=0, off=0) with the RUN gains yields ~zero torque.
    let inp = run_pid_inputs();
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.t78, 0);
    assert_eq!(o.t7c, 0);
    assert_eq!(o.out, 0, "level pitch must give zero balance-PID output");
}

#[test]
fn proportional_torque_matches_hand_computed() {
    // pp=100, kp=6000 -> t_prop = 100*6000/100 = 6000 -> t78=6000.
    // raw = (6000 - 0) * 3900 / 4176 = 23400000/4176 = 5603 (truncate toward zero).
    let mut inp = run_pid_inputs();
    inp.pp = 100;
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.t78, 6000, "proportional sub-term @0x78");
    assert_eq!(o.t7c, 0, "derivative term zero with pr=0");
    assert_eq!(o.out, 5603, "hand-computed proportional torque");
}

#[test]
fn battery_plus_proportional_single_truncation() {
    // bv=12345, bk=2000 -> t_batt = 12345*2000/10000 = 2469.0 ; pp=37, kp=6000 -> t_prop = 2220.
    // sum = 2469 + 2220 = 4689 (single truncation of the exact rational).
    let mut inp = run_pid_inputs();
    inp.bv = 12345;
    inp.pp = 37;
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    // 12345*2000 = 24690000 /10000 = 2469.0 ; 37*6000/100 = 2220 ; sum 4689.
    assert_eq!(o.t78, 4689);
}

#[test]
fn derivative_clamp_bites_both_sides() {
    // pr*kd/100 driven well above +30473 and below -30473; both bounds must bite.
    let mut inp = run_pid_inputs();
    inp.kd = Fix::from_num(1);
    inp.pr = 10_000_000; // /100 = 100000 -> clamp +30473
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.t7c, pidc::DERIV_CLAMP, "upper derivative clamp = +30473");
    assert_eq!(o.t7c, 30473);

    inp.pr = -10_000_000; // -> clamp -30473
    let mut iir2 = IirCarry::default();
    let o2 = balance_pid(&inp, &mut iir2);
    assert_eq!(
        o2.t7c,
        -pidc::DERIV_CLAMP,
        "lower derivative clamp = -30473"
    );
    assert_eq!(o2.t7c, -30473);
}

#[test]
fn output_clamp_bites_at_28500() {
    let mut inp = run_pid_inputs();
    // (Archive literal 100_00; clippy::inconsistent_digit_grouping forces 10_000, same value.)
    inp.pp = 10_000; // pp*kp/100 = 10000*6000/100 = 600000 ; raw huge -> clamp +28500
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.out, pidc::OUTPUT_CLAMP, "output clamp +28500");
    assert_eq!(o.out, 28500);

    inp.pp = -10_000;
    let mut iir2 = IirCarry::default();
    let o2 = balance_pid(&inp, &mut iir2);
    assert_eq!(o2.out, -pidc::OUTPUT_CLAMP, "output clamp -28500");
}

#[test]
fn scale_hysteresis_threshold() {
    let mut inp = run_pid_inputs();
    inp.scale = 3499; // < 3500
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.secondary_scale, 800);

    inp.scale = 3500; // == 3500 -> not less than -> high
    let mut iir2 = IirCarry::default();
    let o2 = balance_pid(&inp, &mut iir2);
    assert_eq!(o2.secondary_scale, 1600);
}

#[test]
fn iir_transient_smooths_99_01() {
    // Step the PID output from old to new; the smoothed reference is 0.99*new + 0.01*old_smoothed,
    // not new. Steady state cannot distinguish, so use a transient.
    let mut iir = IirCarry::default();
    // First tick: produce a known nonzero out and let the carry settle.
    let mut inp = run_pid_inputs();
    inp.pp = 100; // out = 5603
    let o1 = balance_pid(&inp, &mut iir);
    assert_eq!(o1.out, 5603);
    // smoothed = 0.99*5603 + 0.01*0 = 5546.97 -> int16 5546.
    assert_eq!(o1.smoothed_ref, 5546);

    // Second tick: same out (5603). smoothed = 0.99*5603 + 0.01*5546.97 = 5602.4397 -> 5602.
    let o2 = balance_pid(&inp, &mut iir);
    assert_eq!(o2.out, 5603);
    assert_eq!(o2.smoothed_ref, 5602);
    assert!(
        o2.smoothed_ref != o2.out as i16,
        "transient: smoothed != raw"
    );
}

#[test]
fn pid_derivative_d2iz_truncates_toward_zero() {
    // NEW (slice-3 d2iz correction): the derivative conversion truncates TOWARD ZERO (the
    // decompile's FUN_080006e0; specs/control.md Fixed-point clause). pr=-150, kd=1 ->
    // -150/100 = -1.5 -> d2iz -1; the archive's bare `fixed` to_num FLOORED this to -2.
    let mut inp = run_pid_inputs();
    inp.pr = -150;
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.t7c, -1, "trunc(-1.5) = -1, not floor's -2");

    inp.pr = 150; // +1.5 -> 1 (both models agree on the positive side)
    let mut iir2 = IirCarry::default();
    let o2 = balance_pid(&inp, &mut iir2);
    assert_eq!(o2.t7c, 1);
}

#[test]
fn pid_smoothed_ref_negative_transient_truncates_toward_zero() {
    // NEW (slice-3 d2iz correction): the negative mirror of iir_transient_smooths_99_01. The
    // stock converts the IIR value with a truncating float->int (f2iz of the @0xbc float); a
    // floor would land one count lower on every negative fractional value.
    let mut iir = IirCarry::default();
    let mut inp = run_pid_inputs();
    inp.pp = -100; // out = -5603
    let o1 = balance_pid(&inp, &mut iir);
    assert_eq!(o1.out, -5603);
    // smoothed = 0.99*-5603 = -5546.97 -> toward zero -5546 (floor would give -5547).
    assert_eq!(o1.smoothed_ref, -5546);
    let o2 = balance_pid(&inp, &mut iir);
    assert_eq!(o2.smoothed_ref, -5602, "trunc(-5602.4397) = -5602");
}

#[test]
fn pid_flagged_fractional_paths_track_f64_references() {
    // NEW (specs/control.md (f): f64 references under the assert_close discipline for the
    // flagged-fractional Q paths: the kd derivative product and the 0.99/0.01 IIR).
    //
    // Derivative with a genuinely fractional kd: pr=12345, kd=0.37 ->
    // 12345*0.37/100 = 45.6765 -> trunc 45. The value sits 0.32 from the nearest boundary,
    // dwarfing the Q32.32-vs-double representation gap (~1e-9), so exact int equality holds.
    let mut inp = run_pid_inputs();
    inp.pr = 12345;
    inp.kd = Fix::from_num(0.37);
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    let deriv_ref = (12345.0_f64 * 0.37) / 100.0;
    assert_eq!(o.t7c, deriv_ref.trunc() as i32);
    assert_close(o.t7c as f64, deriv_ref, 1.0);

    // IIR over a varying multi-tick transient: track the f64 model of the same pipeline
    // (s = out*0.99 + s_prev*0.01; out is integer-exact in both models). The Q carry must stay
    // within 1e-3 of the f64 reference (Q32.32 coefficient quantization ~1e-9 relative/tick),
    // and the emitted i16 within 1 count (a truncation boundary may sit between the models).
    let mut iir = IirCarry::default();
    let mut s_ref = 0.0_f64;
    for &pp in &[100i16, -40, 250, 0, -180, 77] {
        let mut inp = run_pid_inputs();
        inp.pp = pp;
        let o = balance_pid(&inp, &mut iir);
        s_ref = (o.out as f64) * 0.99 + s_ref * 0.01;
        assert_close(iir.carry.to_num::<f64>(), s_ref, 1e-3);
        assert!(
            ((o.smoothed_ref as f64) - s_ref.trunc()).abs() <= 1.0,
            "smoothed_ref {} vs f64 model {}",
            o.smoothed_ref,
            s_ref
        );
    }
}
