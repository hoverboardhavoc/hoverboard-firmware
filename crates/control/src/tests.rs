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
use crate::helpers::{
    clamp, clamp_sym, iabs, ramp_step, shr_round_to_zero, RampRecord, RAMP_COUNTER_CAP,
};
use crate::pid::{balance_pid, IirCarry, PidInputs};
use crate::shaping::{shape_pitch_target, ShapingInputs, ShapingState};
use crate::speed::{speed_loop, speed_setpoint, SpeedInputs, SpeedState};
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
