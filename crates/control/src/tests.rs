//! Host tests (Section 11). Pure math/logic; no hardware. Where the spec gives an exact constant,
//! the assertion checks the exact value.

use crate::config::{pid as pidc, GainProfile, RUN_PROFILE_A, STANDBY_SET};
use crate::fsm::{fsm_step, FsmInputs, FsmState, SubState};
use crate::helpers::{pi_step, shr_round_to_zero, PiRecord};
use crate::pid::{balance_pid, IirCarry, PidInputs};
use crate::shaping::{shape_pitch_target, ShapingInputs, ShapingState};
use crate::speed::{speed_loop, speed_setpoint, SpeedInputs, SpeedState};
use fixed::types::I32F32;

// ---- helpers to build PID inputs with the RUN gains ----

fn run_pid_inputs() -> PidInputs {
    PidInputs {
        bv: 0,
        bk: RUN_PROFILE_A.bk,   // 2000
        pp: 0,
        kp: RUN_PROFILE_A.kp,   // 6000
        pr: 0,
        kd: I32F32::from_num(1),
        off: 0,
        scale: 4176, // 41.76 V
    }
}

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
    inp.kd = I32F32::from_num(1);
    inp.pr = 10_000_000; // /100 = 100000 -> clamp +30473
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.t7c, pidc::DERIV_CLAMP, "upper derivative clamp = +30473");
    assert_eq!(o.t7c, 30473);

    inp.pr = -10_000_000; // -> clamp -30473
    let mut iir2 = IirCarry::default();
    let o2 = balance_pid(&inp, &mut iir2);
    assert_eq!(o2.t7c, -pidc::DERIV_CLAMP, "lower derivative clamp = -30473");
    assert_eq!(o2.t7c, -30473);
}

#[test]
fn output_clamp_bites_at_28500() {
    let mut inp = run_pid_inputs();
    inp.pp = 100_00; // pp*kp/100 = 10000*6000/100 = 600000 ; raw huge -> clamp +28500
    let mut iir = IirCarry::default();
    let o = balance_pid(&inp, &mut iir);
    assert_eq!(o.out, pidc::OUTPUT_CLAMP, "output clamp +28500");
    assert_eq!(o.out, 28500);

    inp.pp = -100_00;
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
    assert!(o2.smoothed_ref != o2.out as i16, "transient: smoothed != raw");
}

// ---- inner PI integrator (Section 8.1) ----

#[test]
fn pi_accumulator_grows_by_e_times_ki() {
    // setpoint 0, measured swept; e = -measured. accumulator grows by e*Ki (Ki = record[2] = 50).
    let mut rec = PiRecord::seed();
    assert_eq!(rec.ki, 50);
    assert_eq!(rec.kp_divisor, 1024);
    let _ = pi_step(0, -10, &mut rec); // e = 10 -> acc += 10*50 = 500
    assert_eq!(rec.accumulator, 500);
    let _ = pi_step(0, -10, &mut rec); // acc += 500 -> 1000
    assert_eq!(rec.accumulator, 1000);
}

#[test]
fn pi_output_formula() {
    // out = accumulator/8192 + (e*Kp)/1024. With e=10, after one step acc=500:
    // 500/8192 = 0 ; (10*100)/1024 = 0 -> out 0. Use larger e for a nonzero P term.
    let mut rec = PiRecord::seed();
    let out = pi_step(0, -200, &mut rec); // e = 200 ; acc = 200*50 = 10000
    // i_term = 10000/8192 = 1 ; p_term = (200*100)/1024 = 20000/1024 = 19 ; out = 20.
    assert_eq!(rec.accumulator, 10000);
    assert_eq!(out, 20);
}

#[test]
fn pi_antiwindup_holds_at_positive_high_bound() {
    // Drive a large positive error repeatedly; the accumulator must clamp at +268427264
    // (int_min, the positive HIGH bound), not the negative rail.
    let mut rec = PiRecord::seed();
    for _ in 0..1000 {
        let _ = pi_step(1_000_000, 0, &mut rec); // e huge positive
    }
    assert_eq!(rec.accumulator, 268_427_264, "anti-windup HIGH bound (by value)");
}

#[test]
fn pi_antiwindup_holds_at_negative_low_bound() {
    let mut rec = PiRecord::seed();
    for _ in 0..1000 {
        let _ = pi_step(-1_000_000, 0, &mut rec);
    }
    assert_eq!(rec.accumulator, -268_427_264, "anti-windup LOW bound (by value)");
}

#[test]
fn pi_ki_zero_clears_accumulator() {
    let mut rec = PiRecord::seed();
    rec.accumulator = 12345;
    rec.ki = 0;
    let _ = pi_step(100, 0, &mut rec);
    assert_eq!(rec.accumulator, 0);
}

// ---- pitch shaping (Section 4) ----

#[test]
fn shaping_fb_is_absolute_value() {
    // fb = abs((roll_a - roll_b)/10). Negative and positive differentials of equal magnitude give
    // the same base.
    let mut st_pos = ShapingState::default();
    let mut st_neg = ShapingState::default();
    let pos = shape_pitch_target(
        &ShapingInputs { roll_a: 100, roll_b: 0, steer: 0, role_right: false },
        &mut st_pos,
    );
    let neg = shape_pitch_target(
        &ShapingInputs { roll_a: 0, roll_b: 100, steer: 0, role_right: false },
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
        &ShapingInputs { roll_a: 100, roll_b: 0, steer: 100, role_right: false },
        &mut st,
    );
    assert_eq!(out, 150);
}

#[test]
fn shaping_steer_sign_flips_with_role() {
    let mut st_l = ShapingState::default();
    let mut st_r = ShapingState::default();
    let left = shape_pitch_target(
        &ShapingInputs { roll_a: 0, roll_b: 0, steer: 100, role_right: false },
        &mut st_l,
    );
    let right = shape_pitch_target(
        &ShapingInputs { roll_a: 0, roll_b: 0, steer: 100, role_right: true },
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
    let inp = ShapingInputs { roll_a: 10000, roll_b: 0, steer: 1000, role_right: false };
    let t1 = shape_pitch_target(&inp, &mut st);
    assert_eq!(t1, 250, "first tick slews up by at most +250 from 0");
    let t2 = shape_pitch_target(&inp, &mut st);
    assert_eq!(t2, 500, "second tick +250 more");
}

#[test]
fn shaping_absolute_clamp_7000() {
    // Force a target above 7000 and confirm the +-7000 clamp, observed across enough ticks that
    // the slew reaches it.
    let mut st = ShapingState::default();
    st.last_target = 6900; // near the cap already
    let inp = ShapingInputs { roll_a: 30000, roll_b: 0, steer: 30000, role_right: false };
    let t = shape_pitch_target(&inp, &mut st);
    // target before slew clamps to +7000; slew from 6900 by +100 -> 7000.
    assert_eq!(t, 7000);
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
    assert_eq!(RUN_PROFILE_A, crate::config::GainTriple::new(6000, 2000, 40));
}

// ---- speed/steer loop (Section 5) ----

#[test]
fn speed_loop_forced_zero_outside_run() {
    let mut st = SpeedState { acc: 999, correction: 5, direction: 7 };
    let inp = SpeedInputs { run_active: false, ..SpeedInputs::default() };
    speed_loop(&inp, &mut st);
    assert_eq!(st.acc, 0);
    assert_eq!(st.direction, 0);
    assert_eq!(st.correction, 0);
}

#[test]
fn speed_setpoint_clamps_at_32768() {
    // sp = measured - 2*target, clamped to +-32768.
    let out = speed_setpoint([0, 0], [100000, -100000]);
    assert_eq!(out[0], -32768);
    assert_eq!(out[1], 32768);
    let mid = speed_setpoint([1000, 1000], [100, -100]);
    assert_eq!(mid[0], 1000 - 200);
    assert_eq!(mid[1], 1000 + 200);
}

#[test]
fn speed_loop_proportional_terms() {
    // corr = round(B*0.4) + round(T*0.6). B=10 -> 4 ; T=10 -> 6 ; sum 10. No peer blend.
    let mut st = SpeedState::default();
    let inp = SpeedInputs {
        base: 10,
        throttle: 10,
        run_active: true,
        ..SpeedInputs::default()
    };
    speed_loop(&inp, &mut st);
    assert_eq!(st.correction, 10);
}

// ---- balance state machine (Section 7) ----

fn engage_inputs() -> FsmInputs {
    FsmInputs {
        orientation_nz: false, // orient == 0 path: ARMING -> RUN
        smoothed_ref: 1000,
        scaled_ref_1: 3000, // > 2499 -> upright passes
        scaled_ref_2: 0,
        gating_field: 600, // > 500
        rider_present: true,
        over_current: false,
        stall: false,
        comms_loss: false,
        tilt: false,
        enable_bytes_clear: true,
        power_enable: true,
        stop_byte: false,
        pickup_field: 0,
        winddown_enables_clear: false,
        promote_condition_clear: false,
        ref_9c: 0,
        ref_34: 0,
        ref_36: 0,
        feedback_fb: 0,
    }
}

#[test]
fn fsm_idle_to_arming_to_run_with_envelope_ramp() {
    let profile = GainProfile::profile_a();
    let mut st = FsmState::default();
    assert_eq!(st.sub_state, SubState::Idle);

    // First tick: engage conditions hold -> IDLE -> ARMING.
    let inp = engage_inputs();
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Arming);
    assert!(st.balancing_active);
    // ARMING seed for orient==0: (coeff1, coeff2, 0).
    assert_eq!(st.gains, crate::config::GainTriple::new(6000, 2000, 0));

    // Ramp the envelope +200/tick until cap 28500 -> promote to RUN.
    // After the IDLE->ARMING tick the envelope is still 0 (set on entry); subsequent ARMING ticks
    // ramp it. 28500/200 = 142.5 -> 143 ramp ticks to reach the cap.
    let mut ticks = 0;
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, &profile, &mut st);
        ticks += 1;
        assert!(ticks < 200, "must promote within bound");
    }
    assert_eq!(st.sub_state, SubState::Run);
    assert_eq!(st.env, 28500, "envelope at cap 0x6F54");
    // RUN gains: full profile triple.
    assert_eq!(st.gains, RUN_PROFILE_A);
    // ~143 ramp ticks of +200 reach 28500.
    assert_eq!(ticks, 143);
}

#[test]
fn fsm_fault_forces_idle_immediately() {
    let profile = GainProfile::profile_a();
    let mut st = FsmState::default();
    // get into RUN first.
    let inp = engage_inputs();
    let _ = fsm_step(&inp, &profile, &mut st);
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    assert_eq!(st.sub_state, SubState::Run);

    // trip over-current -> immediate IDLE, active cleared.
    let mut fault = inp;
    fault.over_current = true;
    let _ = fsm_step(&fault, &profile, &mut st);
    assert_eq!(st.sub_state, SubState::Idle);
    assert!(!st.balancing_active);
}

#[test]
fn fsm_torque_setpoint_follows_enveloped_smoothed_ref() {
    let profile = GainProfile::profile_a();
    let mut st = FsmState::default();
    // Engage and reach RUN.
    let inp = engage_inputs();
    let _ = fsm_step(&inp, &profile, &mut st);
    while st.sub_state == SubState::Arming {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    assert_eq!(st.sub_state, SubState::Run);
    // In RUN with env at cap, out_mirror = smoothed_ref (1000), within +-env -> torque = 1000.
    assert_eq!(st.torque_setpoint, 1000);

    // Envelope clamp: if env smaller than the mirror, the torque is limited to +-env.
    st.env = 500;
    let mut big = inp;
    big.smoothed_ref = 5000;
    let _ = fsm_step(&big, &profile, &mut st);
    assert_eq!(st.torque_setpoint, 500, "enveloped to +-env");
}

#[test]
fn fsm_idle_envelope_decays_to_zero() {
    let profile = GainProfile::profile_a();
    let mut st = FsmState::default();
    st.env = 28500;
    // No engage (rider absent) -> stays IDLE, decays -1000/tick.
    let mut inp = engage_inputs();
    inp.rider_present = false;
    let _ = fsm_step(&inp, &profile, &mut st);
    assert_eq!(st.env, 27500);
    // run it down; deadband collapses the last <1000 to zero.
    for _ in 0..40 {
        let _ = fsm_step(&inp, &profile, &mut st);
    }
    assert_eq!(st.env, 0);
    assert_eq!(st.torque_setpoint, 0);
}

// ---- shift helper ----

#[test]
fn shr_round_to_zero_truncates_toward_zero() {
    // Plain arithmetic >> floors; the corrected shift truncates toward zero for negatives.
    assert_eq!(shr_round_to_zero(64, 6), 1);
    assert_eq!(shr_round_to_zero(-64, 6), -1);
    assert_eq!(shr_round_to_zero(-63, 6), 0, "-63/64 truncates toward zero to 0");
    assert_eq!((-63i32) >> 6, -1, "plain shift floors to -1 (the thing we avoid)");
}
