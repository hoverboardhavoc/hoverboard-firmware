//! Host tests (`specs/control.md`, section (f) validation). Pure math/logic; no hardware. Where
//! the contract gives an exact constant, the assertion checks the exact value.
//!
//! The recovered archive vectors come across byte-identical (slice 1: the gain-schedule and
//! shift-helper tests; slice 2: the five shaping tests; slice 3: the seven PID/IIR tests,
//! modulo the I32F32 -> base::fixed::Fix type rename). The clamp/ramp helper tests, the shaping
//! f64 steer-scale reference, and the PID d2iz + f64-reference tests are NEW (the archive
//! carried no vectors for them), hand-derived and marked as such. The three archived Section-5
//! vectors are DISPOSED per the slice-4 audit ruling (see the speed section below): the
//! rebuilt-to-the-binary speed loop carries decompile-derived vectors instead.

use crate::config::fsm as fsmc;
use crate::config::{
    pid as pidc, ramp, select_profile, shaping, GainProfile, GainShadow, GainTriple, TuneError,
    DEFAULT_GAIN_MAX, GAIN_FIELD_A, GAIN_FIELD_B, GAIN_MIN, PROFILE_B, RUN_PROFILE_A, STANDBY_SET,
};
use crate::fsm::{fsm_step, FsmInputs, FsmState, SubState};
use crate::gating::GatingFilter;
use crate::helpers::{
    clamp, clamp_sym, iabs, ramp_step, shr_round_to_zero, RampRecord, RAMP_COUNTER_CAP,
};
use crate::mode::{select_mode, ControlDispatch, ControlMode};
use crate::pid::{balance_pid, IirCarry, PidInputs};
use crate::shaping::{shape_pitch_target, ShapingInputs, ShapingState};
use crate::speed::{speed_loop, speed_setpoint, SpeedInputs, SpeedState};
use crate::throttle::{
    filt_low_pass32, mixer_fcn, rate_limiter16, throttle_tick, ThrottleConfig, ThrottleState,
};
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
    // Rider/pad flag false -> Profile B; true -> Profile A {6000,2000,40}. An untuned board's
    // shadow IS the compiled pair, so the selection is what it was before the gains became fields.
    let shadow = crate::config::GainShadow::default();
    let pa = crate::config::select_profile(true, &shadow);
    assert_eq!(pa, GainProfile::profile_a());
    assert_eq!(pa.as_triple(), RUN_PROFILE_A);
    let pb = crate::config::select_profile(false, &shadow);
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
            drive_off: 0,
        },
        &mut st_pos,
    );
    let neg = shape_pitch_target(
        &ShapingInputs {
            roll_a: 0,
            roll_b: 100,
            steer: 0,
            role_right: false,
            drive_off: 0,
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
            drive_off: 0,
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
            drive_off: 0,
        },
        &mut st_l,
    );
    let right = shape_pitch_target(
        &ShapingInputs {
            roll_a: 0,
            roll_b: 0,
            steer: 100,
            role_right: true,
            drive_off: 0,
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
        drive_off: 0,
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
        drive_off: 0,
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
            drive_off: 0,
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
            drive_off: 0,
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

// ---- speed/steer loop (Section 5, rebuilt to the binary per the slice-4 re-cut) ----
//
// Vector disposition of the three archived Section-5 tests (the slice-4 audit ruling): the
// forced-zero and clamp-at-32768 vectors are OBSOLETE (amendments A1/A3: the correction runs
// every tick; the 5.1 helper saturates to +-0x7FFF, values the old vector asserted are not
// even i16-packable); the proportional vector survives only REINTERPRETED as a blend vector
// (below). Everything else here is NEW, decompile-derived.

#[test]
fn speed_blend_and_correction_run_every_tick() {
    // Amendment A1: the blend + correction sum are unconditional; only the integrator and
    // direction cells are zeroed outside RUN sub-state 3.
    let mut st = SpeedState {
        acc: Fix::from_num(50),
        direction: Fix::from_num(5),
        ..Default::default()
    };
    let inp = SpeedInputs {
        blend_input: Fix::from_num(11),
        trim: 3,
        run_active: false,
        ..Default::default()
    };
    speed_loop(&inp, &mut st);
    assert_eq!(st.acc, Fix::ZERO, "integrator CELL zeroed outside RUN");
    assert_eq!(st.direction, Fix::ZERO, "direction zeroed outside RUN");
    // blend = 0.4*11 + 0.6*0 = 4.4 -> f2iz 4; correction = 4 + trim(3) + acc(0) = 7, NOT 0.
    // (Input 11, not 10: 0.4*10 lands ON the integer boundary, where the Q 0.4 sits a hair
    // under and the stock double a hair over, the (f) coefficient-quantization bound; a
    // boundary case makes a bad plain vector.)
    assert_eq!(st.correction, 7, "correction still updates outside RUN");
    assert!(
        st.blend > Fix::ZERO,
        "blend carry still updates outside RUN"
    );
}

#[test]
fn speed_blend_reinterprets_the_archive_proportional_vector() {
    // The archived speed_loop_proportional_terms vector (B=10, T=10 -> 10) survives only
    // reinterpreted per the re-cut: the two "terms" are one one-pole blend over the carry, so
    // carry=10 + input=10 -> 0.4*10 + 0.6*10 = 10 -> f2iz 10 (the Q 0.4/0.6 pair sums to
    // exactly 1.0, so the value is exact).
    let mut st = SpeedState {
        blend: Fix::from_num(10),
        ..Default::default()
    };
    let inp = SpeedInputs {
        blend_input: Fix::from_num(10),
        run_active: true,
        ..Default::default()
    };
    speed_loop(&inp, &mut st);
    assert_eq!(st.correction, 10);
}

#[test]
fn speed_blend_d2iz_negative_fraction() {
    // NEW d2iz discriminator: blend_input = -3.75 -> blend ~= -1.5 (just inside, the Q 0.4 is
    // a hair under 0.4) -> f2iz -1; a floor would give -2.
    let mut st = SpeedState::default();
    let inp = SpeedInputs {
        blend_input: Fix::from_num(-3.75),
        run_active: true,
        ..Default::default()
    };
    speed_loop(&inp, &mut st);
    assert_eq!(st.correction, -1, "trunc toward zero, not floor");
}

#[test]
fn speed_integrator_opposing_signs_predicate() {
    // The pinned predicate (spec (d) item 2): Add iff gate && s1 > W/5 && s2 < -(W/5);
    // Subtract mirrored; else decay only. W = 50 -> thr = 10; edges are STRICT.
    let base = SpeedInputs {
        window: 50,
        gate: true,
        run_active: true,
        ..Default::default()
    };

    // Add: s1 = 11, s2 = -11 -> acc = 0*0.9996 + 1.2 -> consumption trunc 1.
    let mut st = SpeedState::default();
    speed_loop(
        &SpeedInputs {
            s1: 11,
            s2: -11,
            ..base
        },
        &mut st,
    );
    assert!(st.acc > Fix::from_num(1.19) && st.acc < Fix::from_num(1.21));
    assert_eq!(st.correction, 1);

    // Subtract: mirrored.
    let mut st = SpeedState::default();
    speed_loop(
        &SpeedInputs {
            s1: -11,
            s2: 11,
            ..base
        },
        &mut st,
    );
    assert!(st.acc < Fix::from_num(-1.19));
    assert_eq!(st.correction, -1);

    // Same-sign inputs: decay only (no step), even though both are outside the deadband.
    let mut st = SpeedState::default();
    speed_loop(
        &SpeedInputs {
            s1: 11,
            s2: 11,
            ..base
        },
        &mut st,
    );
    assert_eq!(st.acc, Fix::ZERO);

    // Edge values are strict: s1 == thr does not Add; s2 == -thr does not Add.
    let mut st = SpeedState::default();
    speed_loop(
        &SpeedInputs {
            s1: 10,
            s2: -11,
            ..base
        },
        &mut st,
    );
    assert_eq!(st.acc, Fix::ZERO);
    let mut st = SpeedState::default();
    speed_loop(
        &SpeedInputs {
            s1: 11,
            s2: -10,
            ..base
        },
        &mut st,
    );
    assert_eq!(st.acc, Fix::ZERO);

    // Gate byte off: the opposing-signs pair decays only.
    let mut st = SpeedState::default();
    speed_loop(
        &SpeedInputs {
            s1: 11,
            s2: -11,
            gate: false,
            ..base
        },
        &mut st,
    );
    assert_eq!(st.acc, Fix::ZERO);
}

#[test]
fn speed_integrator_float_carry_decays_where_an_int_cell_locks() {
    // The lock-up discriminator (the slice-4 finding): at acc = 100 the float model decays by
    // 0.04/tick; a rounding int cell would return 100 forever and a truncating one would
    // over-decay by 1/tick. The Q carry must land at 99.96 (within Q quantization) and the
    // consumption at trunc = 99.
    let mut st = SpeedState {
        acc: Fix::from_num(100),
        ..Default::default()
    };
    let inp = SpeedInputs {
        run_active: true,
        ..Default::default()
    };
    speed_loop(&inp, &mut st);
    assert!(st.acc < Fix::from_num(100), "the carry decays");
    assert_close(st.acc.to_num::<f64>(), 99.96, 1e-6);
    assert_eq!(st.correction, 99, "d2iz consumption of the decayed carry");
}

#[test]
fn speed_direction_band_and_opposing_accumulate() {
    // Spec (d) item 4: opposing wheel-speed signs accumulate via float add; otherwise the
    // +-30 band applies with in-band -> 0; the out-of-band results are the parameterized
    // inputs (the decompile's argument-dropped calls); the cell is zeroed outside RUN.
    let base = SpeedInputs {
        run_active: true,
        dir_step: Fix::from_num(1.5),
        dir_out_pos: Fix::from_num(-7.5),
        dir_out_neg: Fix::from_num(7.5),
        ..Default::default()
    };

    // Opposing signs accumulate: 0 -> 1.5 -> 3.0.
    let mut st = SpeedState::default();
    let opp = SpeedInputs {
        wheel_a: 5,
        wheel_b: -5,
        ..base
    };
    speed_loop(&opp, &mut st);
    assert_eq!(st.direction, Fix::from_num(1.5));
    speed_loop(&opp, &mut st);
    assert_eq!(st.direction, Fix::from_num(3.0));

    // Non-opposing + in-band (|3.0| <= 30) -> 0.
    speed_loop(&base, &mut st);
    assert_eq!(st.direction, Fix::ZERO);

    // Out-of-band positive / negative -> the parameterized results.
    st.direction = Fix::from_num(31);
    speed_loop(&base, &mut st);
    assert_eq!(st.direction, Fix::from_num(-7.5));
    st.direction = Fix::from_num(-31);
    speed_loop(&base, &mut st);
    assert_eq!(st.direction, Fix::from_num(7.5));

    // Outside RUN the cell is zeroed regardless.
    st.direction = Fix::from_num(31);
    speed_loop(
        &SpeedInputs {
            run_active: false,
            ..base
        },
        &mut st,
    );
    assert_eq!(st.direction, Fix::ZERO);
}

#[test]
fn speed_setpoint_saturates_to_7fff_never_8000() {
    // Amendment A3 (FUN_08004c2c): saturation VALUES +-0x7FFF at the +-0x8000 thresholds; the
    // -0x8000 halfword never appears (the 0x8001 pattern).
    // Far out of range, both sides.
    assert_eq!(speed_setpoint([0, 0], [100_000, -100_000]), [-32767, 32767]);
    // Mid-range passes through; measured is UNSIGNED (u16): 65535 stays positive.
    assert_eq!(speed_setpoint([1000, 65535], [100, 16384]), [800, 32767]);
    assert_eq!(speed_setpoint([1000, 1000], [100, -100]), [800, 1200]);
    // The exact thresholds saturate...
    assert_eq!(speed_setpoint([0x8000, 0], [0, 0x4000]), [32767, -32767]);
    // ...and one inside passes through untouched (threshold-inclusive semantics pinned).
    assert_eq!(speed_setpoint([0x7FFF, 1], [0, 0x4000]), [32767, -32767]);
}

#[test]
fn speed_fractional_paths_track_f64_references() {
    // NEW (spec (f) assert_close discipline): the blend and integrator against their f64
    // models over a varying multi-tick run.
    let mut st = SpeedState::default();
    let mut blend_ref = 0.0_f64;
    let mut acc_ref = 0.0_f64;
    for (k, &x) in [3.0_f64, -7.25, 12.5, 0.0, 42.0, -1.0].iter().enumerate() {
        let inp = SpeedInputs {
            blend_input: Fix::from_num(x),
            run_active: true,
            gate: true,
            window: 50,
            // Alternate Add / decay-only ticks to exercise both integrator paths.
            s1: if k % 2 == 0 { 11 } else { 0 },
            s2: if k % 2 == 0 { -11 } else { 0 },
            ..Default::default()
        };
        speed_loop(&inp, &mut st);
        blend_ref = 0.4 * x + 0.6 * blend_ref;
        acc_ref *= 0.9996;
        if k % 2 == 0 {
            acc_ref += 1.2;
        }
        assert_close(st.blend.to_num::<f64>(), blend_ref, 1e-6);
        assert_close(st.acc.to_num::<f64>(), acc_ref, 1e-6);
        // The consumed ints stay within one count of the f64 model's truncations.
        let corr_ref = blend_ref.trunc() + acc_ref.trunc();
        assert!(
            ((st.correction as f64) - corr_ref).abs() <= 1.0,
            "correction {} vs f64 model {}",
            st.correction,
            corr_ref
        );
    }
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
    let profile = GainProfile::profile_a();

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
    let profile = GainProfile::profile_a();
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
    // leaves `pr` to the RUN ramp (`specs/rider-ui.md` section 4): (coeff1, coeff2, 0) stands on
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
    let profile = GainProfile::profile_a();

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
    let profile = GainProfile::profile_a();
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
    // `specs/rider-ui.md` section 4): the standby seed's 0 stands on the promote tick.
    let profile = GainProfile::profile_a();
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
    let profile = GainProfile::profile_a();
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
    let profile = GainProfile::profile_a();
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
fn fsm_run_pickup_drop() {
    // RUN pickup: the shared gating/pickup halfword negative for > 20 ticks drops to IDLE
    // (counter trip on tick 21); no flags or state words are touched by this path.
    let profile = GainProfile::profile_a();
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
    let profile = GainProfile::profile_a();

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
    let profile = GainProfile::profile_a();
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
    let profile = GainProfile::profile_a();
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
    let profile = GainProfile::profile_a();
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
    let profile = GainProfile::profile_a();
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
    let demoted = select_mode(1, false);
    assert_eq!(demoted.active, ControlMode::Throttle);
    assert!(demoted.fault);
    let ok = select_mode(1, true);
    assert_eq!(ok.active, ControlMode::Balance);
    assert!(!ok.fault);
    let thr = select_mode(0, false);
    assert_eq!(thr.active, ControlMode::Throttle);
    assert!(!thr.fault);
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
    let mut d = ControlDispatch::new(0, false, 1, 2400);
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert!(!d.mode_fault());
    for _ in 0..10 {
        let _ = d.throttle_reference(&cfg, 32767, 0);
    }
    assert_ne!(d.throttle.speed_rate_fixdt, 0, "records carry state");

    // The same mode byte re-read: no change, so the carries stand (this is the common arm).
    let before = d.throttle;
    assert!(
        !d.re_apply_values(0, true, 1, 2400),
        "the mode did not move"
    );
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert_eq!(d.throttle.speed_rate_fixdt, before.speed_rate_fixdt);

    // A fresh byte: applies, records replaced, the seam re-validates (with IMU -> Balance, no
    // fault), and the change is REPORTED so the caller can replace the records it owns.
    assert!(d.re_apply_values(1, true, 1, 2400), "the mode moved");
    assert_eq!(d.mode(), ControlMode::Balance);
    assert!(!d.mode_fault());
    assert_eq!(d.throttle.speed_rate_fixdt, 0, "records replaced wholesale");

    // A demoted request raises the fault exactly as at boot; an unknown byte lands Throttle.
    assert!(d.re_apply_values(1, false, 1, 2400));
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert!(d.mode_fault());
    // Balance-without-IMU and Throttle both RUN Throttle, so the active mode does not move between
    // them; the demotion fault still has to follow the byte, which is why the seam re-runs
    // unconditionally rather than under the `changed` branch.
    assert!(!d.re_apply_values(7, true, 1, 2400), "Throttle either way");
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert!(!d.mode_fault(), "the unknown byte is not a demotion");
}

#[test]
fn the_rider_requirement_is_decoded_at_the_boot_seam_and_moved_only_by_the_arm_re_read() {
    // `specs/control.md` (i): the CONTROL_RIDER_REQUIRED byte is decoded by the constructor (0
    // waives, anything else requires: the default 1 and a corrupt byte both keep the rider gate).
    // The one thing that moves it is the arm-time re-read (`specs/integration.md`), which decodes a
    // fresh byte by the SAME rule, and the mode byte riding in the same call cannot disturb it.
    for (byte, required) in [(1u8, true), (0, false), (2, true), (0xFF, true)] {
        let mut d = ControlDispatch::new(1, true, byte, 2400);
        assert_eq!(d.rider_required(), required, "byte {byte}");
        // A mode that moves under it leaves the decision alone: the two travel together through
        // the re-apply, and each is decoded by its own rule.
        for (m, imu) in [(0u8, true), (1, true), (1, false), (7, true)] {
            d.re_apply_values(m, imu, byte, 2400);
            assert_eq!(d.rider_required(), required, "survives the mode byte {m}");
        }
        // The arm-time re-apply: the same byte vocabulary, the same decode as `new`.
        for (fresh, fresh_required) in [(1u8, true), (0, false), (2, true), (0xFF, true)] {
            d.re_apply_values(1, true, fresh, 2400);
            assert_eq!(
                d.rider_required(),
                fresh_required,
                "re-applied byte {fresh} must decode as `new` decodes it"
            );
            assert_eq!(
                d.rider_required(),
                ControlDispatch::new(1, true, fresh, 2400).rider_required(),
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
    let mut d = ControlDispatch::new(1, false, 1, 2400);
    assert_eq!(d.mode(), ControlMode::Throttle);
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
        !d.re_apply_values(1, false, 0, 3000),
        "the mode did not move"
    );
    assert!(!d.rider_required(), "the fresh byte waives the requirement");
    assert!(!d.battery_ok(2999), "under the fresh floor");
    assert!(d.battery_ok(3000), "at it");
    assert_eq!(d.mode(), ControlMode::Throttle, "the demotion stands");
    assert!(d.mode_fault(), "and so does its fault");
    assert_eq!(carry(&d), before, "the throttle records are not reset");

    // And back, including the floor's "no floor" state.
    assert!(!d.re_apply_values(1, false, 1, 0));
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
    let profile = GainProfile::profile_a();

    // THROTTLE: condition full forward to the settled +-28500 word, feed it as the mirror.
    let cfg = ThrottleConfig::default();
    let mut d = ControlDispatch::new(0, false, 1, 2400);
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

    // BALANCE: the PID's smoothed reference through the same shell.
    let sel = select_mode(1, true);
    assert_eq!(sel.active, ControlMode::Balance);
    let mut iir = IirCarry::default();
    let mut pid_in = run_pid_inputs();
    pid_in.pp = 100; // out 5603, smoothed 5546 (the slice-3 vector)
    let o = balance_pid(&pid_in, &mut iir);
    let mut st = FsmState::default();
    let mut inp = engage_fsm_inputs();
    inp.smoothed_ref = o.smoothed_ref as i32;
    let _ = fsm_step(&inp, &profile, &mut st);
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    let torque = fsm_step(&inp, &profile, &mut st);
    assert_eq!(
        torque, 5546,
        "the balance smoothed reference rides the same output stage"
    );
    assert!(torque.abs() <= 28500);
}

// ---- the live gain shadow and the tune lane (`specs/rider-ui.md` section 4, prerequisite P1) ----

#[test]
fn an_untuned_shadow_is_the_compiled_profiles() {
    let s = GainShadow::default();
    assert_eq!(s.a(), RUN_PROFILE_A);
    assert_eq!(s.b(), PROFILE_B);
    // Both halves start agreeing: nothing is "unsaved" on a board nobody has tuned.
    for (field, triple) in [(GAIN_FIELD_A, RUN_PROFILE_A), (GAIN_FIELD_B, PROFILE_B)] {
        let want = [triple.kp, triple.bk, triple.pr];
        for (i, w) in want.iter().enumerate() {
            assert_eq!(s.get(field, i as u8), Some(*w as i16));
            assert_eq!(s.stored(field, i as u8), Some(*w as i16));
        }
    }
}

#[test]
fn a_stored_gain_outside_its_range_is_clamped_before_it_reaches_the_loop() {
    // The boot seam's posture: a hand-poked flash value cannot put the loop outside the range the
    // tune seam enforces, and a board with a bad stored gain still runs.
    let s = GainShadow::of_stored([[30000, -5, 4000], [-1, 20000, 1001]], DEFAULT_GAIN_MAX);
    assert_eq!(s.a(), GainTriple::new(20000, 0, 1000));
    assert_eq!(s.b(), GainTriple::new(0, 10000, 1000));
    // The clamp applies to the stored half too, so a reconcile against the same flash is inert.
    let mut t = s;
    t.reconcile([[30000, -5, 4000], [-1, 20000, 1001]]);
    assert_eq!(t, s);
}

/// The maxima are the shadow's own (`specs/rider-ui.md` section 4, "Ranges"): the defaults
/// reproduce the old constant table `0..=20000 / 0..=10000 / 0..=1000` bit for bit.
#[test]
fn the_default_maxima_reproduce_the_old_range_table() {
    assert_eq!(GAIN_MIN, 0);
    assert_eq!(DEFAULT_GAIN_MAX, [20000, 10000, 1000]);
    let defaults = [[6000, 2000, 40], [3000, 1000, 30]];
    assert_eq!(
        GainShadow::default(),
        GainShadow::of_stored(defaults, DEFAULT_GAIN_MAX)
    );
    // Every edge of the old table, against the default shadow.
    for (i, hi) in [20000i16, 10000, 1000].iter().enumerate() {
        let mut s = GainShadow::default();
        assert_eq!(s.set(GAIN_FIELD_A, i as u8, 0), Ok(()));
        assert_eq!(s.set(GAIN_FIELD_A, i as u8, *hi), Ok(()));
        assert_eq!(s.set(GAIN_FIELD_A, i as u8, -1), Err(TuneError::OutOfRange));
        assert_eq!(
            s.set(GAIN_FIELD_A, i as u8, hi + 1),
            Err(TuneError::OutOfRange)
        );
    }
}

/// A maximum below a stored gain clamps it at boot; the lane then refuses at `max + 1` and takes
/// `max`; a reconcile clamps against the same maxima.
#[test]
fn a_stored_maximum_bounds_the_boot_clamp_the_lane_and_the_reconcile() {
    let flash = [[6000, 2000, 40], [3000, 1000, 30]];
    let mut s = GainShadow::of_stored(flash, [4000, 1500, 35]);
    assert_eq!(s.a(), GainTriple::new(4000, 1500, 35), "clamped at boot");
    assert_eq!(
        s.b(),
        GainTriple::new(3000, 1000, 30),
        "under the maxima: unchanged"
    );
    assert_eq!(s.stored(GAIN_FIELD_A, 0), Some(4000), "the stored half too");
    for (i, max) in [4000i16, 1500, 35].iter().enumerate() {
        let i = i as u8;
        assert_eq!(s.set(GAIN_FIELD_B, i, max + 1), Err(TuneError::OutOfRange));
        assert_eq!(s.set(GAIN_FIELD_B, i, *max), Ok(()));
        assert_eq!(s.get(GAIN_FIELD_B, i), Some(*max));
    }
    // A maximum above the old table is honoured: the owner widened it.
    let mut w = GainShadow::of_stored(flash, [30000, 10000, 1000]);
    assert_eq!(w.set(GAIN_FIELD_A, 0, 25000), Ok(()));
    assert_eq!(w.set(GAIN_FIELD_A, 0, 30001), Err(TuneError::OutOfRange));
    // A flash write from elsewhere clamps against the boot-read maxima.
    s.reconcile([[9000, 2000, 40], [3000, 1000, 30]]);
    assert_eq!(s.a().kp, 4000);
}

/// **The maxima are re-read at the next ARM** (`specs/integration.md`, "When a stored value takes
/// effect: the arm-time re-read"), not at the next power-cycle: a shrunk maximum clamps a live gain
/// that was above it, a grown one admits a value the tune lane had been refusing, and the
/// negative-maximum floor is `of_stored`'s own.
///
/// `stored` is left exactly as read throughout: it is a record of FLASH, and `reconcile` decides
/// "flash changed" by comparing a freshly clamped read against it, so clamping the record by a
/// bound that can move would read as a change that never happened (see `re_apply_max`).
#[test]
fn the_arm_time_maxima_re_apply_clamps_live_and_leaves_stored_as_read() {
    let flash = [[6000, 2000, 40], [3000, 1000, 30]];
    let mut s = GainShadow::of_stored(flash, DEFAULT_GAIN_MAX);
    // The tune lane puts a value in range of the boot maxima...
    assert_eq!(s.set(GAIN_FIELD_A, 0, 15000), Ok(()));
    assert_eq!(s.get(GAIN_FIELD_A, 0), Some(15000));
    // ...and a SHRUNK maximum must not leave it there.
    s.re_apply_max([5000, 1500, 35]);
    assert_eq!(
        s.get(GAIN_FIELD_A, 0),
        Some(5000),
        "the live gain is clamped"
    );
    assert_eq!(
        s.stored(GAIN_FIELD_A, 0),
        Some(6000),
        "the stored half is a record of flash and is left as read"
    );
    assert_eq!(
        s.b(),
        GainTriple::new(3000, 1000, 30),
        "under it: unchanged"
    );
    assert_eq!(
        s.set(GAIN_FIELD_A, 0, 5001),
        Err(TuneError::OutOfRange),
        "the new range is what the lane enforces"
    );
    // A GROWN maximum admits what the lane was refusing, and changes no value by itself.
    s.re_apply_max([30000, 10000, 1000]);
    assert_eq!(
        s.get(GAIN_FIELD_A, 0),
        Some(5000),
        "a grown range moves nothing"
    );
    assert_eq!(s.set(GAIN_FIELD_A, 0, 25000), Ok(()));
    assert_eq!(s.set(GAIN_FIELD_A, 0, 30001), Err(TuneError::OutOfRange));
    // The negative floor is the constructor's: a re-applied negative maximum reads as 0, exactly
    // as a boot-read one does, so the live gains pin at 0 and the lane takes 0 and refuses 1.
    let mut n = GainShadow::of_stored(flash, DEFAULT_GAIN_MAX);
    n.re_apply_max([-1, i16::MIN, 0]);
    assert_eq!(n.a(), GainTriple::new(0, 0, 0));
    assert_eq!(n.b(), GainTriple::new(0, 0, 0));
    assert_eq!(
        n.a(),
        GainShadow::of_stored(flash, [0, 0, 0]).a(),
        "the same live result a boot-read 0 maximum gives"
    );
    for i in 0..3u8 {
        assert_eq!(n.set(GAIN_FIELD_A, i, 0), Ok(()));
        assert_eq!(n.set(GAIN_FIELD_A, i, 1), Err(TuneError::OutOfRange));
        assert_eq!(
            n.stored(GAIN_FIELD_A, i),
            Some(flash[0][i as usize]),
            "and the flash record is untouched by the floor"
        );
    }
}

/// **A maximum that grows back does not, by itself, move a live tuned gain** (`specs/integration.md`,
/// the arm-time re-read). This is the defect `re_apply_max` not clamping `stored` prevents: with
/// `stored` left as the flash value, the next `reconcile`'s freshly clamped read AGREES with it, so
/// there is no false "flash changed" to write over the tune.
///
/// What this does NOT claim is that a tuned gain survives every later save: it does not, and
/// `re_apply_max`'s doc records why (`stored` is still a clamped record in `of_stored` and
/// `reconcile`, and closing that is a `specs/rider-ui.md` section 4 decision). The property pinned
/// here is the one this seam owns.
#[test]
fn a_maximum_that_grows_back_does_not_move_a_live_tune() {
    // Flash kp 6000, boot maximum well above it: `stored` is the flash value.
    let flash = [[6000, 2000, 40], [3000, 1000, 30]];
    let mut s = GainShadow::of_stored(flash, DEFAULT_GAIN_MAX);
    assert_eq!(s.stored(GAIN_FIELD_A, 0), Some(6000));
    // An arm with a SHRUNK maximum clamps the live gain and leaves the record alone.
    s.re_apply_max([5000, 1500, 35]);
    assert_eq!(s.get(GAIN_FIELD_A, 0), Some(5000));
    assert_eq!(s.stored(GAIN_FIELD_A, 0), Some(6000));
    // An arm with the maximum raised back, then a live tune the wider range admits.
    s.re_apply_max(DEFAULT_GAIN_MAX);
    assert_eq!(s.set(GAIN_FIELD_A, 0, 15000), Ok(()));
    // An unrelated disarmed save: the firmware re-reads the gains and reconciles. Flash has not
    // moved, and the record still says so, so the tune stands.
    s.reconcile(flash);
    assert_eq!(
        s.get(GAIN_FIELD_A, 0),
        Some(15000),
        "a reconcile against unchanged flash must not touch a live tune"
    );
    assert_eq!(s.stored(GAIN_FIELD_A, 0), Some(6000));
    // And a flash value that genuinely DID move still lands, as the lane's save path requires.
    s.reconcile([[6500, 2000, 40], [3000, 1000, 30]]);
    assert_eq!(s.get(GAIN_FIELD_A, 0), Some(6500));
}

/// A negative stored maximum reads as 0: the range is `0..=0`, the gain pins at 0, and the lane
/// takes 0 and refuses 1 (never an empty range that refuses everything).
#[test]
fn a_negative_stored_maximum_reads_as_zero() {
    let flash = [[6000, 2000, 40], [3000, 1000, 30]];
    let mut s = GainShadow::of_stored(flash, [-1, i16::MIN, 0]);
    assert_eq!(s.a(), GainTriple::new(0, 0, 0));
    assert_eq!(s.b(), GainTriple::new(0, 0, 0));
    assert_eq!(s, GainShadow::of_stored(flash, [0, 0, 0]));
    for i in 0..3u8 {
        assert_eq!(s.set(GAIN_FIELD_A, i, 0), Ok(()));
        assert_eq!(s.set(GAIN_FIELD_A, i, 1), Err(TuneError::OutOfRange));
    }
}

#[test]
fn the_tune_seam_owns_the_allowlist_and_the_ranges() {
    let mut s = GainShadow::default();
    // In range: taken, live, and the STORED half is untouched (no flash was written).
    assert_eq!(s.set(GAIN_FIELD_A, 0, 12345), Ok(()));
    assert_eq!(s.a().kp, 12345);
    assert_eq!(s.stored(GAIN_FIELD_A, 0), Some(RUN_PROFILE_A.kp as i16));
    // Out of range: REFUSED, not clamped, and the live value stands.
    for (i, hi) in DEFAULT_GAIN_MAX.iter().enumerate() {
        let i = i as u8;
        assert_eq!(s.set(GAIN_FIELD_B, i, GAIN_MIN), Ok(()));
        assert_eq!(s.set(GAIN_FIELD_B, i, *hi), Ok(()));
        assert_eq!(
            s.set(GAIN_FIELD_B, i, GAIN_MIN - 1),
            Err(TuneError::OutOfRange)
        );
        assert_eq!(s.set(GAIN_FIELD_B, i, hi + 1), Err(TuneError::OutOfRange));
        assert_eq!(s.get(GAIN_FIELD_B, i), Some(*hi));
    }
    // The allowlist is exactly the two profile ids and exactly three indices: every other key is
    // unknown to the lane, including the store fields either side of the pair.
    assert_eq!(s.set(GAIN_FIELD_A, 3, 0), Err(TuneError::UnknownKey));
    assert_eq!(s.set(0x70, 0, 0), Err(TuneError::UnknownKey));
    assert_eq!(s.set(0x73, 0, 0), Err(TuneError::UnknownKey));
    assert_eq!(s.set(0x74, 0, 0), Err(TuneError::UnknownKey));
    assert_eq!(s.get(0x70, 0), None);
    assert_eq!(s.get(GAIN_FIELD_A, 9), None);
}

#[test]
fn a_reconcile_follows_flash_only_where_flash_moved() {
    // The Save round trip (`specs/rider-ui.md` D3): a live tune stands until the FLASH under it
    // changes. Writing an unrelated field re-reads the same gains and must not revert the tune.
    let mut s = GainShadow::default();
    s.set(GAIN_FIELD_A, 0, 9000).unwrap();
    let flash = [[6000, 2000, 40], [3000, 1000, 30]];
    s.reconcile(flash);
    assert_eq!(
        s.a().kp,
        9000,
        "an unchanged flash value must not revert a live tune"
    );

    // A Save of that same tuned value: flash moves to 9000, so both halves converge and the live
    // value is (still) 9000 - the reboot-reverts rule now has nothing to revert.
    s.reconcile([[9000, 2000, 40], [3000, 1000, 30]]);
    assert_eq!(s.a().kp, 9000);
    assert_eq!(s.stored(GAIN_FIELD_A, 0), Some(9000));

    // A flash write from somewhere else (a host tool staging a value) DOES take the live gain,
    // clamped on the way in.
    s.reconcile([[32000, 2000, 40], [3000, 1000, 30]]);
    assert_eq!(s.a().kp, DEFAULT_GAIN_MAX[0] as i32);
}

#[test]
fn select_profile_reads_the_live_shadow() {
    let mut s = GainShadow::default();
    s.set(GAIN_FIELD_A, 0, 7777).unwrap();
    s.set(GAIN_FIELD_B, 2, 7).unwrap();
    assert_eq!(
        select_profile(true, &s).as_triple(),
        GainTriple::new(7777, 2000, 40)
    );
    assert_eq!(
        select_profile(false, &s).as_triple(),
        GainTriple::new(3000, 1000, 7)
    );
}

/// One cascade tick at the fidelity the gain question needs: the balance PID off the machine's
/// LIVE gains, its smoothed reference into the engagement machine, the machine's torque setpoint
/// out. This is `orchestrator::dispatch`'s balance arm with every producer the test does not vary
/// held constant, and `pp` chosen to sit in the PID's linear region so the output is sensitive to
/// `kp` rather than pinned at the +-28500 clamp.
fn cascade_tick(
    st: &mut FsmState,
    iir: &mut IirCarry,
    profile: &GainProfile,
    fault: bool,
    pp: i16,
) -> i16 {
    let w = Inputs {
        pp,
        bv: 100,
        kd: Fix::ZERO,
        off: 0,
        scale: 3600,
    };
    let out = pid_from_live_gains(st, iir, &w);
    let inp = FsmInputs {
        smoothed_ref: out as i32,
        comms_loss: fault,
        pid_scale: w.scale,
        pid_kd: w.kd,
        ..engage_fsm_inputs()
    };
    fsm_step(&inp, profile, st)
}

/// The PID inputs a ramp test holds constant, so the only thing that moves the output is the
/// gains.
#[derive(Clone, Copy)]
struct Inputs {
    pp: i16,
    bv: i32,
    kd: Fix,
    off: i32,
    scale: i16,
}

/// The balance PID off the machine's live triple (the dispatch's "the PREVIOUS pass's setup").
fn pid_from_live_gains(st: &FsmState, iir: &mut IirCarry, w: &Inputs) -> i16 {
    let g = st.gains;
    balance_pid(
        &PidInputs {
            bv: w.bv,
            bk: g.bk,
            pp: w.pp,
            kp: g.kp,
            pr: g.pr,
            kd: w.kd,
            off: w.off,
            scale: w.scale,
        },
        iir,
    )
    .smoothed_ref
}

/// What a pass does to the live triple. `Ramp` is the machine as built; the other two are the
/// negative controls, applied after the real pass: `CopyEveryPass` is the unramped copy (the
/// triple set to the shadow's profile on every RUN pass), `Frozen` is the behaviour before the
/// ramp (the promote installs the full triple and RUN never writes it again, so a shadow write
/// waits for the next engage), and `PromoteInstalls` is the binary's promote alone (the full
/// triple installed on the tick the machine enters RUN, the PROMOTE hazard).
#[derive(Clone, Copy, PartialEq, Debug)]
enum Variant {
    Ramp,
    CopyEveryPass,
    Frozen,
    PromoteInstalls,
}

/// One cascade tick (PID into FSM) at the held inputs `w`, under `variant`.
fn ramp_tick(
    st: &mut FsmState,
    iir: &mut IirCarry,
    profile: &GainProfile,
    w: &Inputs,
    variant: Variant,
) -> i16 {
    let before = *st;
    let out = pid_from_live_gains(st, iir, w);
    let inp = FsmInputs {
        smoothed_ref: out as i32,
        pid_scale: w.scale,
        pid_kd: w.kd,
        ..engage_fsm_inputs()
    };
    let t = fsm_step(&inp, profile, st);
    let promoted = before.sub_state == SubState::Arming && st.sub_state == SubState::Run;
    match variant {
        Variant::Ramp => {}
        Variant::CopyEveryPass if st.sub_state == SubState::Run => st.gains = profile.as_triple(),
        Variant::Frozen | Variant::PromoteInstalls if promoted => st.gains = profile.as_triple(),
        Variant::Frozen if before.sub_state == SubState::Run => st.gains = before.gains,
        _ => {}
    }
    t
}

/// The top of every gain's seam range: the furthest a tune write can send the shadow.
fn top_of_range() -> GainTriple {
    GainTriple::new(
        DEFAULT_GAIN_MAX[0] as i32,
        DEFAULT_GAIN_MAX[1] as i32,
        DEFAULT_GAIN_MAX[2] as i32,
    )
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

/// The live-write test of `specs/rider-ui.md` section 4, on the cascade's own linear-region
/// inputs (`cascade_tick`): a shadow write mid-RUN of every gain to the top of its range reaches
/// the RUNNING loop, the torque output moves by no more than [`shaping::SLEW_LIMIT`] on any tick
/// while it converges, and the next engage starts from the tuned gains under the soft-start
/// envelope.
///
/// Before the ramp this test asserted the opposite half: RUN never re-copied the profile, so the
/// write reached nothing until a re-engage. Its negative control is unchanged in kind: re-copying
/// the profile on every RUN pass (`Variant::CopyEveryPass`) steps the setpoint 5,025 -> 16,673 in
/// one tick, and still fails the bound (pinned in
/// `the_gain_ramp_holds_the_slew_limit_at_the_worst_case_inputs`).
#[test]
fn a_live_gain_write_cannot_step_the_torque_output_beyond_the_slew_limit() {
    const PP: i16 = 77; // the linear-region demand (see `cascade_tick`)
    let mut shadow = GainShadow::default();
    let mut st = FsmState::default();
    let mut iir = IirCarry::default();

    // Engage and settle in RUN on the default profile A.
    let mut profile = select_profile(true, &shadow);
    for _ in 0..400 {
        cascade_tick(&mut st, &mut iir, &profile, false, PP);
    }
    assert_eq!(st.sub_state, SubState::Run);
    assert_eq!(st.gains, RUN_PROFILE_A);
    let settled = st.torque_setpoint;
    assert_eq!(settled, 5025, "the linear-region setpoint");

    // The live write, mid-run: every gain of the ACTIVE profile to the top of its range.
    for (i, hi) in DEFAULT_GAIN_MAX.iter().enumerate() {
        shadow.set(GAIN_FIELD_A, i as u8, *hi).unwrap();
    }
    profile = select_profile(true, &shadow);

    // The negative control: the unramped copy (the triple set to the shadow on this pass) steps
    // the setpoint 5,025 -> 16,673 in one tick.
    let (mut cst, mut ciir) = (st, iir);
    cst.gains = profile.as_triple();
    assert_eq!(
        cascade_tick(&mut cst, &mut ciir, &profile, false, PP),
        16673
    );

    // At this battery word (3600) and kd (0) the caps are kp 6, bk 4, pr unlimited: kp's 14,000
    // counts take 2,334 passes, the slowest.
    let want = passes_needed(RUN_PROFILE_A, top_of_range(), 3600, Fix::ZERO);
    assert_eq!(want, 2334);
    let mut prev = settled;
    let mut converged_at = None;
    for tick in 1..=want + 50 {
        let out = cascade_tick(&mut st, &mut iir, &profile, false, PP);
        assert!(
            (out as i32 - prev as i32).abs() <= shaping::SLEW_LIMIT,
            "tick {tick}: a live gain write stepped the torque output {prev} -> {out}, past the \
             {} count slew limit",
            shaping::SLEW_LIMIT
        );
        prev = out;
        if converged_at.is_none() && st.gains == top_of_range() {
            converged_at = Some(tick);
        }
    }
    assert_eq!(
        converged_at,
        Some(want),
        "the write reaches the RUNNING loop in the derived number of passes"
    );
    // Where the ramp arrives, smoothly: the settled setpoint at the top of the range.
    assert_eq!(st.torque_setpoint, 16790);

    // The next engage: the seed is the tuned profile, and the re-entry is bounded by the soft-start
    // envelope.
    let out = cascade_tick(&mut st, &mut iir, &profile, true, PP); // comms loss -> IDLE
    assert_eq!(st.sub_state, SubState::Idle);
    assert_eq!(
        out, prev,
        "the abort tick still emits its mirror once (the binary's order)"
    );
    // The fault stop itself is a hard zero on the next tick, deliberately and unrelated to gains:
    // an immediate stop is not slew-limited (IDLE zeroes the mirror at entry). The bound this test
    // is about resumes from there.
    assert_eq!(cascade_tick(&mut st, &mut iir, &profile, false, PP), 0);
    prev = 0;
    for tick in 0..400 {
        let out = cascade_tick(&mut st, &mut iir, &profile, false, PP);
        assert!(
            (out as i32 - prev as i32).abs() <= shaping::SLEW_LIMIT,
            "re-engage tick {tick}: {prev} -> {out} past the slew limit"
        );
        prev = out;
    }
    assert_eq!(st.sub_state, SubState::Run);
    assert_eq!(
        st.gains,
        top_of_range(),
        "the tuned gains hold across a re-engage"
    );
}

/// The largest per-tick torque-setpoint delta of one scenario, from the tick the machine enters
/// RUN, and the pass on which the live triple reached the top of the range (`None` if it never
/// did). Engages on the default profile A at the held inputs `w`, settles until profile A is
/// reached, then writes every gain of the shadow to the top of its range and runs `run_for`
/// passes.
fn worst_case_scenario(w: &Inputs, variant: Variant, run_for: u32) -> (i32, Option<u32>) {
    let mut shadow = GainShadow::default();
    let mut st = FsmState::default();
    let mut iir = IirCarry::default();
    let mut profile = select_profile(true, &shadow);
    let mut prev: Option<i16> = None;
    let mut worst = 0;
    let track = |st: &FsmState, out: i16, prev: &mut Option<i16>, worst: &mut i32| {
        if st.sub_state == SubState::Run {
            if let Some(p) = *prev {
                *worst = (*worst).max((out as i32 - p as i32).abs());
            }
            *prev = Some(out);
        }
    };
    // Engage, promote, and let the ramp take `pr` from the engage seed's 0 to profile A.
    for _ in 0..600 {
        let out = ramp_tick(&mut st, &mut iir, &profile, w, variant);
        track(&st, out, &mut prev, &mut worst);
    }
    assert_eq!(st.sub_state, SubState::Run);
    assert_eq!(st.gains, RUN_PROFILE_A);

    for (i, hi) in DEFAULT_GAIN_MAX.iter().enumerate() {
        shadow.set(GAIN_FIELD_A, i as u8, *hi).unwrap();
    }
    profile = select_profile(true, &shadow);
    let mut converged_at = None;
    for tick in 1..=run_for {
        let out = ramp_tick(&mut st, &mut iir, &profile, w, variant);
        track(&st, out, &mut prev, &mut worst);
        if converged_at.is_none() && st.gains == top_of_range() {
            converged_at = Some(tick);
        }
    }
    (worst, converged_at)
}

/// The PID numerator `t78 + t7c` at gains `g` and inputs `w` (exact enough to place `off`).
fn numerator(g: GainTriple, w: &Inputs) -> i64 {
    let t78 = (w.bv as i64 * g.bk as i64 + w.pp as i64 * g.kp as i64 * 100) / 10000;
    let t7c = (Fix::from_num(g.pr) * w.kd / Fix::from_num(100)).to_num::<i64>();
    t78 + t7c
}

/// The `off` values that place the PID's linear region (+-28500 at this `scale`) over every part
/// of the ramp from profile A to the top of the range, so no phase of the ramp is only ever seen
/// at the clamp, where a gain step moves nothing.
fn offsets_covering_the_ramp(w: &Inputs) -> std::vec::Vec<i32> {
    let half = 28500i64 * w.scale as i64 / 3900; // the linear half-width, in numerator counts
    let (lo, hi) = (numerator(RUN_PROFILE_A, w), numerator(top_of_range(), w));
    let mut v = std::vec::Vec::new();
    let mut off = lo - half / 2;
    while off <= hi + half / 2 {
        v.push(off as i32);
        off += half;
    }
    v
}

/// THE property (`specs/rider-ui.md` section 4): with the cascade (PID into FSM) in RUN at the
/// worst-case inputs the cap is derived at (`|pp|` = 2499, the upright window; `|bv|` = 87,266,
/// the gyro full scale; `kd` 0 and 100, the latter the spec's simulated producer), a shadow write
/// of every gain to the top of its range moves the torque setpoint by at most
/// [`shaping::SLEW_LIMIT`] on every tick while the ramp converges, at every battery word from the
/// lowest the caps are derived for without the floor (757) to above a full LEV50-8 pack, and the
/// ramp converges in exactly the derived number of passes.
///
/// `off` is swept so the PID's linear region sits over every phase of the ramp in turn (a gain
/// step at the +-28500 clamp moves nothing and would prove nothing), and the observed worst delta
/// must come close to the shares' sum, so the test is seeing the cap and not a clamp.
///
/// Negative controls at the same inputs: the unramped copy breaks the bound, and the pre-ramp
/// behaviour (RUN never writes the triple) never converges.
#[test]
fn the_gain_ramp_holds_the_slew_limit_at_the_worst_case_inputs() {
    for scale in [757i16, 2400, 2502, 3300, 4200] {
        for kd in [Fix::ZERO, Fix::from_num(100)] {
            let base = Inputs {
                pp: ramp::PP_BOUND as i16,
                bv: ramp::BV_BOUND,
                kd,
                off: 0,
                scale,
            };
            let want = passes_needed(RUN_PROFILE_A, top_of_range(), scale, kd);
            let mut seen = 0;
            for off in offsets_covering_the_ramp(&base) {
                let w = Inputs { off, ..base };
                let (worst, converged) = worst_case_scenario(&w, Variant::Ramp, want + 20);
                assert!(
                    worst <= shaping::SLEW_LIMIT,
                    "scale {scale} kd {kd} off {off}: a ramp step moved the torque {worst} counts"
                );
                assert_eq!(converged, Some(want), "scale {scale} kd {kd} off {off}");
                seen = seen.max(worst);
            }
            // Each gain's exact contribution at these inputs, at the caps: the bound's sharpness.
            let c = ramp::caps(scale, kd);
            let exact = (c[0].max(1) as f64 * 24.99
                + c[1].max(1) as f64 * 8.7266
                + if kd == Fix::ZERO { 0.0 } else { c[2] as f64 })
                * 3900.0
                / scale as f64;
            assert!(
                seen as f64 >= exact * 0.95,
                "scale {scale} kd {kd}: the sweep saw {seen} of the cap's {exact:.0}"
            );
        }
    }
    // The derived passes, profile A to the top of the range: 3,500 at 2400 (14.0 s at 250 Hz),
    // 2,334 at 3300.
    assert_eq!(
        passes_needed(RUN_PROFILE_A, top_of_range(), 2400, Fix::ZERO),
        3500
    );
    assert_eq!(
        passes_needed(RUN_PROFILE_A, top_of_range(), 3300, Fix::ZERO),
        2334
    );

    // Negative controls, at 2400 with kd 100, over the same sweep.
    let base = Inputs {
        pp: ramp::PP_BOUND as i16,
        bv: ramp::BV_BOUND,
        kd: Fix::from_num(100),
        off: 0,
        scale: 2400,
    };
    let want = passes_needed(RUN_PROFILE_A, top_of_range(), 2400, base.kd);
    let mut copy_worst = 0;
    for off in offsets_covering_the_ramp(&base) {
        let w = Inputs { off, ..base };
        let (worst, _) = worst_case_scenario(&w, Variant::CopyEveryPass, want + 20);
        copy_worst = copy_worst.max(worst);
        let (_, converged) = worst_case_scenario(&w, Variant::Frozen, want + 20);
        assert_eq!(
            converged, None,
            "pre-ramp: a RUN-time write reaches nothing"
        );
    }
    assert!(
        copy_worst > shaping::SLEW_LIMIT,
        "the unramped copy must break the bound, or this test cannot see a step ({copy_worst})"
    );
}

/// The PROMOTE case (`specs/rider-ui.md` section 4, `specs/todo.md`): with a `kd` producer
/// simulated (`kd` = 100) and `pr` tuned to the top of its range BEFORE the engage, the binary's
/// promote installed the full triple with the envelope already at its cap, stepping the setpoint
/// about 1,074 counts. With the promote leaving `pr` to the ramp, `pr` climbs from the engage
/// seed's 0 to 1000 in the derived number of passes and no tick exceeds the slew limit (the
/// worst is 10 counts).
#[test]
fn the_promote_leaves_pr_to_the_ramp() {
    let w = Inputs {
        pp: 77,
        bv: 100,
        kd: Fix::from_num(100),
        off: 0,
        scale: 3600,
    };
    let run = |variant: Variant| {
        let mut shadow = GainShadow::default();
        shadow.set(GAIN_FIELD_A, 2, DEFAULT_GAIN_MAX[2]).unwrap();
        let profile = select_profile(true, &shadow);
        let mut st = FsmState::default();
        let mut iir = IirCarry::default();
        let mut prev = 0i16;
        let mut worst = 0;
        let mut promoted_at = None;
        let mut converged_at = None;
        for tick in 1..=400u32 {
            let was = st.sub_state;
            let out = ramp_tick(&mut st, &mut iir, &profile, &w, variant);
            if was == SubState::Run {
                worst = worst.max((out as i32 - prev as i32).abs());
            }
            prev = out;
            if was == SubState::Arming && st.sub_state == SubState::Run {
                promoted_at = Some(tick);
                if variant == Variant::Ramp {
                    assert_eq!(
                        st.gains.pr, 0,
                        "the engage seed's pr stands on the promote tick"
                    );
                }
            }
            if converged_at.is_none() && st.gains.pr == DEFAULT_GAIN_MAX[2] as i32 {
                converged_at = Some(tick);
            }
        }
        (worst, promoted_at.unwrap(), converged_at.unwrap())
    };

    let (worst, promoted, converged) = run(Variant::Ramp);
    assert_eq!(worst, 10, "ramp: well inside the slew limit");
    // pr's cap at 3600 with kd 100 is 9 per pass: 1000 counts take 112 passes after the promote.
    assert_eq!(ramp::caps(3600, w.kd)[2], 9);
    assert_eq!(converged - promoted, 112);

    // The binary's promote: pr arrives at full authority in one tick.
    let (worst, promoted, converged) = run(Variant::PromoteInstalls);
    assert_eq!(converged, promoted);
    assert_eq!(
        worst, 1074,
        "the installed promote steps the setpoint 1,074, 4.3x the slew limit (the spec's number)"
    );
}

/// The derivation pinned as arithmetic: at every battery word from 757 (below which `bk`'s floor
/// of 1 exceeds its derived cap) to `i16::MAX`, and every `kd` up to the floor's limit at that
/// word, the three capped steps' exact torque contributions at the worst-case inputs stay inside
/// their shares, and the shares plus the truncation slack inside [`shaping::SLEW_LIMIT`]. Below
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
            <= shaping::SLEW_LIMIT as i64 * scale * 10000
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

/// `scale == 0` (an UNKNOWN battery word) makes the raw PID output 0 whatever the terms are, the
/// stated step-3 contract; the step-5 hysteresis keeps its arithmetic (0 selects the low scale).
#[test]
fn pid_unknown_battery_scale_gives_zero_output() {
    // A large demand under a known word, so the zero below is the contract and not the inputs.
    let mut known = run_pid_inputs();
    known.bv = 5000;
    known.pp = 300;
    known.off = -200;
    let k = balance_pid(&known, &mut IirCarry::default());
    assert_ne!(k.out, 0, "the fixture produces torque under a known word");

    let unknown = PidInputs { scale: 0, ..known };
    let mut iir = IirCarry::default();
    let o = balance_pid(&unknown, &mut iir);
    assert_eq!(o.out, 0, "raw output 0 on an unknown battery");
    assert_eq!(o.smoothed_ref, 0, "and the reference it smooths stays 0");
    assert_eq!(o.secondary_scale, 800, "0 < 3500 selects the low scale");
    // The terms before the divide are still computed (observation is unchanged).
    assert_eq!((o.t78, o.t7c), (k.t78, k.t7c));
    // Held at UNKNOWN, the output never leaves 0.
    for _ in 0..50 {
        assert_eq!(balance_pid(&unknown, &mut iir).smoothed_ref, 0);
    }
}

/// `battery_ok` joins the engage conjunction: with every other gate open, a refused battery
/// does not engage, and a permitted one does.
#[test]
fn fsm_unknown_battery_blocks_engage() {
    let profile = GainProfile::profile_a();
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
    let d = ControlDispatch::new(1, true, 1, 2400);
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
        let d = ControlDispatch::new(1, true, 1, floor);
        assert!(!d.battery_ok(0), "floor {floor}: UNKNOWN still refuses");
        for word in [1i16, 2399, i16::MAX] {
            assert!(d.battery_ok(word), "floor {floor}, word {word}");
        }
    }
    // No clamp beyond the type: a floor above any reachable word refuses every word.
    let d = ControlDispatch::new(1, true, 1, i16::MAX);
    assert!(!d.battery_ok(i16::MAX - 1));
    // The floor moves only with its own byte: a mode change carried by the same re-apply call
    // leaves it as it was.
    let mut d = ControlDispatch::new(1, true, 1, 2400);
    assert!(d.re_apply_values(0, true, 1, 2400));
    assert!(d.re_apply_values(1, true, 1, 2400));
    assert!(!d.battery_ok(2399));
}

/// `battery_ok` is read only by the engage conjunction: a RUN machine whose battery term goes
/// false (the word sagging under the floor) stays in RUN.
#[test]
fn fsm_battery_refusal_in_run_never_disengages() {
    let profile = GainProfile::profile_a();
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

// ---- the balance-mode drive input (`specs/control.md` (h)) ----

#[test]
fn drive_lean_seam_clamps_both_indices() {
    use crate::drive::{DriveLean, LEAN_MAX_CEIL, LEAN_SLEW_MAX, LEAN_SLEW_MIN};
    // In range: unchanged.
    let d = DriveLean::new(300, 4);
    assert_eq!((d.lean_max(), d.lean_slew()), (300, 4));
    // lean_max: 0..1500. A negative store value is not a reversed lean; it clamps to disabled.
    assert_eq!(DriveLean::new(1500, 4).lean_max(), 1500);
    assert_eq!(DriveLean::new(1501, 4).lean_max(), LEAN_MAX_CEIL);
    assert_eq!(DriveLean::new(i16::MAX, 4).lean_max(), 1500);
    assert_eq!(DriveLean::new(-1, 4).lean_max(), 0);
    assert_eq!(DriveLean::new(i16::MIN, 4).lean_max(), 0);
    // lean_slew: 1..100. Zero clamps to 1, so a staged lean_max is never silently dead.
    assert_eq!(DriveLean::new(300, 0).lean_slew(), LEAN_SLEW_MIN);
    assert_eq!(DriveLean::new(300, -7).lean_slew(), 1);
    assert_eq!(DriveLean::new(300, 1).lean_slew(), 1);
    assert_eq!(DriveLean::new(300, 100).lean_slew(), 100);
    assert_eq!(DriveLean::new(300, 101).lean_slew(), LEAN_SLEW_MAX);
    // The unstaged default: disabled, 4 centidegrees per tick.
    assert_eq!(DriveLean::default(), DriveLean::new(0, 4));
}

#[test]
fn drive_lean_cmd_is_lean_max_at_the_rails_and_truncates_toward_zero() {
    use crate::drive::DriveLean;
    let d = DriveLean::new(1500, 4);
    assert_eq!(d.lean_cmd(0), 0);
    assert_eq!(d.lean_cmd(32767), 1500);
    assert_eq!(d.lean_cmd(-32767), -1500);
    // The one value past the negative rail still reads the rail (trunc(-1500.05) = -1500).
    assert_eq!(d.lean_cmd(i16::MIN), -1500);
    // Truncation toward zero, symmetric: 21 * 1500 / 32767 = 0.96 -> 0; 22 -> 1.007 -> 1.
    assert_eq!(d.lean_cmd(21), 0);
    assert_eq!(d.lean_cmd(-21), 0);
    assert_eq!(d.lean_cmd(22), 1);
    assert_eq!(d.lean_cmd(-22), -1);
    // Half stick: 16384 * 1500 / 32767 = 750.02 -> 750.
    assert_eq!(d.lean_cmd(16384), 750);
    assert_eq!(d.lean_cmd(-16384), -750);
    // Against the exact rational over the whole input range.
    for v in i16::MIN..=i16::MAX {
        let exact = (v as f64) * 1500.0 / 32767.0;
        assert_eq!(d.lean_cmd(v), exact.trunc() as i32, "value {v}");
    }
    // Disabled: every value discards to zero.
    let off = DriveLean::default();
    for v in [i16::MIN, -1, 0, 1, i16::MAX] {
        assert_eq!(off.lean_cmd(v), 0);
    }
}

#[test]
fn drive_lean_slews_by_exactly_lean_slew_both_ways() {
    use crate::drive::DriveLean;
    // lean_max 500, slew 4: full stick takes 125 ticks up, 125 back, every step exactly 4.
    let d = DriveLean::new(500, 4);
    let mut lean = 0;
    for k in 1..=125 {
        let prev = lean;
        assert_eq!(d.step(32767, &mut lean), 4 * k);
        assert_eq!(lean - prev, 4);
    }
    assert_eq!(lean, 500);
    assert_eq!(d.step(32767, &mut lean), 500, "holds at the command");
    for k in 1..=125 {
        assert_eq!(d.step(0, &mut lean), 500 - 4 * k);
    }
    assert_eq!(lean, 0);
    // Through zero to the other rail: 250 ticks, the step never exceeding the slew.
    let mut ticks = 0;
    let mut lean = 500;
    while lean != -500 {
        let prev = lean;
        d.step(-32767, &mut lean);
        assert_eq!(prev - lean, 4);
        ticks += 1;
    }
    assert_eq!(ticks, 250);
    // A step smaller than the slew lands exactly (no overshoot): from 498 toward 500 is +2.
    let mut lean = 498;
    assert_eq!(d.step(32767, &mut lean), 500);
    // The ceiling rate: slew 100 reaches 1500 in 15 ticks.
    let fast = DriveLean::new(1500, 100);
    let mut lean = 0;
    for _ in 0..15 {
        fast.step(32767, &mut lean);
    }
    assert_eq!(lean, 1500);
}

#[test]
fn drive_off_is_the_lean_through_kp_with_one_truncation_and_the_sign_flip() {
    use crate::drive::drive_off;
    // off = -(kp * lean * 100) / 10000 = -kp * lean / 100: a positive lean is a negative off.
    assert_eq!(drive_off(6000, 100), -6000);
    assert_eq!(drive_off(6000, -100), 6000);
    assert_eq!(drive_off(600, 100), -600);
    assert_eq!(drive_off(6000, 0), 0);
    assert_eq!(drive_off(0, 1500), 0);
    // One truncation toward zero, symmetric: 50 * 3 / 100 = 1.5 -> -1 / +1.
    assert_eq!(drive_off(50, 3), -1);
    assert_eq!(drive_off(50, -3), 1);
    // The widest seam values (kp 20000, lean 1500) need the i64 product and fit i32 after.
    assert_eq!(drive_off(20000, 1500), -300_000);
    assert_eq!(drive_off(20000, -1500), 300_000);
    // The same unit as the proportional path: with PP_PER_DEGREE (= 100, pp in centidegrees) a
    // lean of L centidegrees converts to exactly the negated proportional term the PID forms for
    // pp = L (`(pp * kp) / 100`, pid step 1 with bv = 0), so the two cancel at pitch -L.
    assert_eq!(crate::config::speed::PP_PER_DEGREE, 100);
    for kp in [50, 600, 6000, 20000] {
        for lean in [-1500i16, -101, -1, 1, 99, 1500] {
            let t78 = balance_pid(
                &PidInputs {
                    bv: 0,
                    bk: 0,
                    pp: lean,
                    kp,
                    pr: 0,
                    kd: Fix::ZERO,
                    off: 0,
                    scale: 3600,
                },
                &mut IirCarry::default(),
            )
            .t78;
            assert_eq!(drive_off(kp, lean as i32), -t78, "kp {kp} lean {lean}");
        }
    }
    // Against the exact rational.
    for kp in [0, 1, 7, 50, 600, 6000, 20000] {
        for lean in [-1500, -333, -1, 0, 1, 77, 1500] {
            let exact = -(kp as f64) * (lean as f64) / 100.0;
            assert_eq!(
                drive_off(kp, lean),
                exact.trunc() as i32,
                "kp {kp} lean {lean}"
            );
        }
    }
}

#[test]
fn shaping_adds_drive_off_after_the_slew_and_outside_the_steer_latch() {
    // Spec (h): the steer path runs the stock steps (clamp to +-base, +-7000, +-250 slew) and
    // latches its own slewed value; drive_off is added to the RETURNED target after step 5, so
    // none of the stock bounds touch it. roll 0 -> base 3500; steer 30000 -> steer_term 45000,
    // clamped to 3500.
    let inp = |steer, drive_off| ShapingInputs {
        roll_a: 0,
        roll_b: 0,
        steer,
        role_right: false,
        drive_off,
    };
    // 3500 + 2000 = 5500: past the steer bound (so not inside step 3); the latch keeps 3500.
    let mut st = ShapingState {
        last_target: 3400,
        ..Default::default()
    };
    assert_eq!(shape_pitch_target(&inp(30000, 2000), &mut st), 5500);
    assert_eq!(st.last_target, 3500, "the latch is the steer path's value");
    // 3500 + 5000 = 8500: past the +-7000 clamp (so not before step 4).
    let mut st = ShapingState {
        last_target: 3500,
        ..Default::default()
    };
    assert_eq!(shape_pitch_target(&inp(30000, 5000), &mut st), 8500);
    assert_eq!(st.last_target, 3500);
    // And after the slew: from 0 the steer path moves 250, the term arrives whole.
    let mut st = ShapingState::default();
    assert_eq!(shape_pitch_target(&inp(30000, -9000), &mut st), 250 - 9000);
    assert_eq!(st.last_target, 250);
    // The largest term the seams allow (kp 20000, lean 1500 -> 300,000) passes unclamped.
    let mut st = ShapingState::default();
    assert_eq!(shape_pitch_target(&inp(0, -300_000), &mut st), -300_000);
    assert_eq!(st.last_target, 0);
    // The shaper never touches the drive carry (the orchestrator's DriveLean::step owns it).
    assert_eq!(st.drive_lean, 0);
}

#[test]
fn a_drive_term_does_not_feed_the_steer_slew() {
    // Spec (h): step 5's `last` stays the STEER path's slewed value, not the sum. A tick with a
    // drive term and no steer leaves the latch at 0, so a full steer step on the next tick is
    // slewed from 0 (to 250), not from the previous returned target 4000 (which would give
    // 3500 - 4000 = -500 -> a 3750 steer path).
    let inp = |steer, drive_off| ShapingInputs {
        roll_a: 0,
        roll_b: 0,
        steer,
        role_right: false,
        drive_off,
    };
    let mut st = ShapingState::default();
    assert_eq!(shape_pitch_target(&inp(0, 4000), &mut st), 4000);
    assert_eq!(st.last_target, 0, "the drive term is not latched");
    assert_eq!(shape_pitch_target(&inp(30000, 4000), &mut st), 250 + 4000);
    assert_eq!(st.last_target, 250);
    // And the steer path then walks to its bound at 250 per tick, the term riding on top.
    for k in 2..=14 {
        assert_eq!(
            shape_pitch_target(&inp(30000, 4000), &mut st),
            (250 * k).min(3500) + 4000
        );
    }
    assert_eq!(st.last_target, 3500);
    // Removing the term leaves the steer path where it was: no step in the latch.
    assert_eq!(shape_pitch_target(&inp(30000, 0), &mut st), 3500);
}

/// One tick of the balance cascade exactly as `orchestrator::dispatch::balance_step` chains it
/// (speed loop -> drive lean -> shaper -> PID), disarmed and level-rolled, at a fixed live `kp`:
/// the real functions, not a model of them. Returns the PID's clamped raw output.
fn drive_cascade_tick(
    pitch_word: i16,
    value: i16,
    kp: i32,
    cfg: &crate::drive::DriveLean,
    st: &mut (SpeedState, ShapingState, IirCarry),
) -> i32 {
    speed_loop(
        &SpeedInputs {
            blend_input: Fix::from_num(pitch_word),
            trim: 0,
            gate: false,
            s1: 0,
            s2: 0,
            window: 0,
            wheel_a: 0,
            wheel_b: 0,
            dir_step: Fix::ZERO,
            dir_out_pos: Fix::ZERO,
            dir_out_neg: Fix::ZERO,
            run_active: false,
        },
        &mut st.0,
    );
    let lean = cfg.step(value, &mut st.1.drive_lean);
    let off = shape_pitch_target(
        &ShapingInputs {
            roll_a: 0,
            roll_b: 0,
            steer: 0,
            role_right: false,
            drive_off: crate::drive::drive_off(kp, lean),
        },
        &mut st.1,
    );
    balance_pid(
        &PidInputs {
            bv: 0,
            bk: RUN_PROFILE_A.bk,
            pp: st.0.correction,
            kp,
            pr: 0,
            kd: Fix::ZERO,
            off,
            scale: 3600,
        },
        &mut st.2,
    )
    .out
}

#[test]
fn a_staged_lean_moves_the_pid_zero_crossing_to_minus_lean_for_any_kp() {
    // Spec (h): off = kp * L / 100 shifts the equilibrium pitch by exactly L in pp's unit, for any
    // kp, so the physical lean a stick commands does not change while kp is hunted. On the real
    // cascade: with lean_max 100 (1 degree) at full stick, the PID output crosses zero at pitch
    // -100 centidegrees for kp 6000 AND for kp 600, within one pp quantum (the blend's settled
    // value sits on its truncation boundary, (j)). The SIGN is the other half: +value commands a
    // NEGATIVE equilibrium pitch (lean forward), -value a positive one.
    //
    // The lean is 5 degrees, past what the stock +-7000 clamp would allow at kp 6000
    // (7000 * 100 / 6000 = 116 centidegrees) had the term been added before step 4: added after
    // the slew, the stock bounds act on the steer path only and the full lean reaches the PID.
    use crate::drive::DriveLean;
    for (lean, kp) in [(100, 6000), (100, 600), (500, 6000), (500, 600)] {
        let cfg = DriveLean::new(lean as i16, 4);
        for (value, eq) in [(32767i16, -lean), (-32767, lean)] {
            let mut zeros = std::vec::Vec::new();
            for p in (eq - 30)..=(eq + 30) {
                let mut st = Default::default();
                let mut out = 0;
                for _ in 0..(lean / 4 + 40) {
                    out = drive_cascade_tick(p as i16, value, kp, &cfg, &mut st);
                }
                assert_eq!(
                    st.1.drive_lean,
                    value.signum() as i32 * lean,
                    "the lean settled"
                );
                // Monotone in pitch, with the zero within one quantum of the staged equilibrium.
                if (p - eq).abs() >= 2 {
                    assert_eq!(
                        out.signum(),
                        (p - eq).signum(),
                        "kp {kp} value {value}: pitch {p} gives output {out}"
                    );
                }
                if out == 0 {
                    zeros.push(p);
                }
            }
            // The output does reach zero: the crossing is there, not jumped over.
            assert!(
                !zeros.is_empty() && zeros.iter().all(|z| (z - eq).abs() <= 1),
                "kp {kp} value {value}: zero crossing at {zeros:?}, want {eq} +-1"
            );
        }
    }
}

#[test]
fn a_zero_drive_off_leaves_the_cascade_unchanged() {
    // The disabled default (lean_max 0) produces drive_off 0 at every stick position and kp, so
    // the cascade output is the stock one tick for tick.
    use crate::drive::DriveLean;
    let off = DriveLean::default();
    for p in [-700i16, -100, 0, 33, 900] {
        let mut a = Default::default();
        let mut b = Default::default();
        for _ in 0..60 {
            let x = drive_cascade_tick(p, 32767, 6000, &off, &mut a);
            let y = drive_cascade_tick(p, 0, 6000, &off, &mut b);
            assert_eq!(x, y);
            assert_eq!(a.1.drive_lean, 0);
        }
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
