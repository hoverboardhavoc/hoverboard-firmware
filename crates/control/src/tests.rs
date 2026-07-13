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

use crate::config::{pid as pidc, GainProfile, RUN_PROFILE_A, STANDBY_SET};
use crate::fsm::{fsm_step, FsmInputs, FsmState, SubState};
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
    assert_eq!(st.gains, RUN_PROFILE_A);
    assert_eq!(st.base_coeff, Fix::from_num(0.4));
    assert_eq!(st.state_word_8c, 0, "cap-entry clears @0x8C");
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
    // @0x48 0.4f copy + the full profile triple.
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
        ..Default::default()
    };
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Run);
    assert_eq!(st.env, 2, "env = |reference|, not the cap");
    assert_eq!(st.winddown_counter, 0, "@0x94 cleared on promote");
    assert_eq!(st.gains, RUN_PROFILE_A);
    assert_eq!(st.base_coeff, Fix::from_num(0.4));
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

#[test]
fn mode_switch_applies_only_disarmed_and_resets_records() {
    // The switch seam mirrors commutation's switch_method discipline: disarmed-only, records
    // replaced wholesale on apply.
    let cfg = ThrottleConfig::default();
    let mut d = ControlDispatch::new(0, false);
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert!(!d.mode_fault());
    for _ in 0..10 {
        let _ = d.throttle_reference(&cfg, 32767, 0);
    }
    assert_ne!(d.throttle.speed_rate_fixdt, 0, "records carry state");

    // Armed: refused, nothing touched.
    let before = d.throttle;
    assert!(!d.switch_mode(1, true, false));
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert_eq!(d.throttle.speed_rate_fixdt, before.speed_rate_fixdt);

    // Disarmed: applies, records reset, the seam re-validates (with IMU -> Balance, no fault).
    assert!(d.switch_mode(1, true, true));
    assert_eq!(d.mode(), ControlMode::Balance);
    assert!(!d.mode_fault());
    assert_eq!(d.throttle.speed_rate_fixdt, 0, "records replaced wholesale");

    // A demoting switch raises the fault exactly as at boot; an unknown byte lands Throttle.
    assert!(d.switch_mode(1, false, true));
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert!(d.mode_fault());
    assert!(d.switch_mode(7, true, true));
    assert_eq!(d.mode(), ControlMode::Throttle);
    assert!(!d.mode_fault());
}

#[test]
fn end_to_end_both_modes_drive_the_shared_fsm_on_the_28500_contract() {
    // One engagement shell + output stage, two reference producers (spec (b)); the FSM is
    // mode-agnostic (throttle parameterizes the balance-only upright gate off with a zero
    // reference). Both modes' setpoints land on the +-28500 contract.
    let profile = GainProfile::profile_a();

    // THROTTLE: condition full forward to the settled +-28500 word, feed it as the mirror.
    let cfg = ThrottleConfig::default();
    let mut d = ControlDispatch::new(0, false);
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
