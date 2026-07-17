//! Host tests for the inputs subsystem. The debounce/combo/pad logic is exercised with square-wave
//! and glitch patterns to pin the load-bearing asymmetries; the throttle is checked for the exact
//! scale, the design tau lag, the +200 rest, and Q-vs-f64 agreement.

use crate::combo::{combined_button, ComboPair, ComboSet, ComboState};
use crate::debounce::{DebouncePhase, LineBank};
use crate::pad::{PadBank, PAD_A_BIT, PAD_B_BIT};
use crate::throttle::{scaled_throttle, ThrottleFilter, ThrottleRefF64, KA, KB, OUTPUT_BIAS};

// --- Debounce: two-call press, one-call release ---------------------------------------------

#[test]
fn press_needs_two_consecutive_asserts() {
    let mut bank = LineBank::new(1);
    // First assert: phase 0 -> 1 (candidate), NOT yet pressed.
    bank.update(0b1);
    assert!(!bank.pressed(0), "one assert must not press");
    assert_eq!(bank.line(0).phase(), DebouncePhase::Candidate);
    // Second consecutive assert: phase 1 -> 2 (held), pressed rises.
    bank.update(0b1);
    assert!(bank.pressed(0), "two consecutive asserts press");
    assert_eq!(bank.line(0).phase(), DebouncePhase::Held);
}

#[test]
fn release_is_one_sample() {
    let mut bank = LineBank::new(1);
    bank.update(0b1);
    bank.update(0b1);
    assert!(bank.pressed(0));
    // Single de-assert drops the flag immediately (no release-confirm interval).
    bank.update(0b0);
    assert!(!bank.pressed(0), "release within one call");
    assert_eq!(bank.line(0).phase(), DebouncePhase::Idle);
}

#[test]
fn single_call_glitch_never_presses() {
    let mut bank = LineBank::new(1);
    // A lone assert followed by de-assert is a spike rejected in phase 1.
    bank.update(0b1);
    assert!(!bank.pressed(0));
    bank.update(0b0);
    assert!(!bank.pressed(0), "single-call glitch never sets the flag");
    assert_eq!(bank.line(0).phase(), DebouncePhase::Idle);
}

#[test]
fn square_wave_flag_rises_after_two_and_drops_after_one() {
    let mut bank = LineBank::new(1);
    // Square wave: assert for 3 calls, de-assert for 3, repeated. The flag should rise on the 2nd
    // consecutive assert and drop on the 1st de-assert.
    let mut history = std::vec::Vec::new();
    for cycle in 0..3 {
        for k in 0..3 {
            bank.update(0b1);
            history.push((cycle, "hi", k, bank.pressed(0)));
        }
        for k in 0..3 {
            bank.update(0b0);
            history.push((cycle, "lo", k, bank.pressed(0)));
        }
    }
    // Within every high run: not pressed at k=0, pressed from k=1 onward.
    for &(_, phase, k, pressed) in &history {
        match (phase, k) {
            ("hi", 0) => assert!(!pressed, "k0 high: not yet pressed"),
            ("hi", _) => assert!(pressed, "k>=1 high: pressed"),
            ("lo", _) => assert!(!pressed, "any low: released"),
            _ => {}
        }
    }
}

#[test]
fn packed_flags_byte_set_means_idle() {
    // 3 lines: press line 1 only. Bit set = idle/released, bit clear = pressed.
    let mut bank = LineBank::new(3);
    bank.update(0b010);
    bank.update(0b010);
    assert!(bank.pressed(1));
    let b = bank.flags_byte();
    // line1 pressed -> bit1 clear; lines 0,2 idle -> bits 0,2 set; lines 3..7 absent -> set.
    assert_eq!(b & 1, 1, "line0 idle -> bit set");
    assert_eq!(b & 0b010, 0, "line1 pressed -> bit clear");
    assert_eq!(b & 0b100, 0b100, "line2 idle -> bit set");
    assert_eq!(b, 0b1111_1101);
}

// --- Combined button + brake/secondary -------------------------------------------------------

#[test]
fn combined_button_a_b_assignment() {
    let mut st = ComboState::new();
    // Held, brake inactive -> A set, combined true.
    assert!(combined_button(&mut st, true, false));
    assert!(st.a && !st.b);
    // Held, brake active -> B set too.
    assert!(combined_button(&mut st, true, true));
    assert!(st.a && st.b);
    // Not held, brake inactive -> clears A only.
    assert!(combined_button(&mut st, false, false));
    assert!(!st.a && st.b, "release with brake inactive clears A only");
    // Not held, brake active -> clears B.
    assert!(!combined_button(&mut st, false, true));
    assert!(!st.a && !st.b);
}

// --- Two-key combos --------------------------------------------------------------------------

#[test]
fn combo_only_when_both_members_held() {
    let mut bank = LineBank::new(4);
    // Power combo = lines (0,1), mode combo = lines (2,3).
    let combos = ComboSet::new(ComboPair::new(0, 1), ComboPair::new(2, 3));

    // Hold only line 0 (two calls). Power combo must be false (line 1 not held).
    bank.update(0b0001);
    bank.update(0b0001);
    assert!(bank.pressed(0) && !bank.pressed(1));
    assert!(!combos.evaluate(&bank).power, "one member is not a combo");

    // Now also hold line 1 (two calls), keeping line 0 held.
    bank.update(0b0011);
    bank.update(0b0011);
    assert!(bank.pressed(0) && bank.pressed(1));
    let f = combos.evaluate(&bank);
    assert!(f.power, "both power members held -> power combo");
    assert!(!f.mode, "mode members not held");

    // Release line 1 (one call): combo drops with its member.
    bank.update(0b0001);
    assert!(
        !combos.evaluate(&bank).power,
        "combo drops when a member releases"
    );
}

// --- Foot-pad debounce -----------------------------------------------------------------------

#[test]
fn pad_engages_on_one_high_off_on_two_low() {
    let mut pads = PadBank::new();
    // Single high on pad A engages immediately.
    let f = pads.update(true, false);
    assert_eq!(f & PAD_A_BIT, PAD_A_BIT, "1 high engages pad A");
    assert_eq!(f & PAD_B_BIT, 0);
    // One low sample does NOT disengage yet.
    let f = pads.update(false, false);
    assert_eq!(f & PAD_A_BIT, PAD_A_BIT, "1 low keeps pad A on");
    // Second consecutive low disengages.
    let f = pads.update(false, false);
    assert_eq!(f & PAD_A_BIT, 0, "2 lows disengage pad A");
}

#[test]
fn pad_high_resets_low_run() {
    let mut pads = PadBank::new();
    pads.update(true, true); // both on
    pads.update(false, false); // one low each (low_run = 1)
                               // A high sample re-engages and must reset the low run, so a single later low cannot disengage.
    pads.update(true, true);
    pads.update(false, false); // low_run back to 1
    let f = pads.field();
    assert_eq!(
        f,
        PAD_A_BIT | PAD_B_BIT,
        "high resets the low run; 1 low does not drop"
    );
}

#[test]
fn pad_field_both_pads_full_rider() {
    let mut pads = PadBank::new();
    let f = pads.update(true, true);
    assert_eq!(
        f,
        PAD_A_BIT | PAD_B_BIT,
        "both pads -> 0b11 full rider field"
    );
    assert_eq!(f, 0b11);
}

// --- Throttle scale --------------------------------------------------------------------------

#[test]
fn throttle_scaled_is_raw_times_0_3122() {
    // scaled = (raw * 0x27F6) >> 15 ~= raw * 0.3122.
    for &raw in &[0u16, 1000, 12345, 32768, 50000, 65535] {
        let got = scaled_throttle(raw);
        let expect = ((raw as u32 * 0x27F6) >> 15) as i16;
        assert_eq!(got, expect);
        // Within ~0.5 LSB of the 0.3122 ratio.
        let ratio = 0x27F6 as f64 / 0x8000 as f64;
        let approx = (raw as f64 * ratio) as i32;
        assert!((got as i32 - approx).abs() <= 1, "scaled ~ raw*0.3122");
    }
    // Sanity: max raw stays within signed-16.
    assert!(scaled_throttle(65535) > 0);
}

// --- Throttle IIR ----------------------------------------------------------------------------

#[test]
fn iir_first_call_is_baseline_plus_bias() {
    let mut f = ThrottleFilter::new();
    let raw = 30000u16;
    let s = scaled_throttle(raw) as i32;
    let out = f.step(raw);
    assert!(f.is_initialized());
    // First call: state == baseline == s; output = (int16)s + 200.
    assert_eq!(out as i32, s + OUTPUT_BIAS);
}

#[test]
fn iir_rests_at_plus_200() {
    // Held at zero scaled input: filtered stays 0, output rests at exactly +200.
    let mut f = ThrottleFilter::new();
    for _ in 0..10_000 {
        let out = f.step(0);
        assert_eq!(out as i32, OUTPUT_BIAS, "zero input rests at +200");
    }
}

#[test]
fn iir_lags_with_design_tau() {
    // tau = dt / (1 - Kb) = 0.004 / 0.0003 ~= 13.333 s. After one tau (3333 calls) a step response
    // from baseline 0 reaches ~63.2% of the step. Check the Q filter tracks that target closely.
    let mut f = ThrottleFilter::new();
    // Init baseline at 0.
    f.step(0);
    let step_raw = 60000u16;
    let s = scaled_throttle(step_raw) as f64;
    // 1 - Kb = Ka = 0.0003. calls for one tau = round(1/Ka) = 3333.
    let n_tau = (1.0 / KA).round() as usize;
    for _ in 0..n_tau {
        f.step(step_raw);
    }
    let reached = f.baseline_f64();
    let frac = reached / s;
    // First-order step response at one tau is 1 - e^-1 ~= 0.632.
    assert!(
        (frac - 0.6321).abs() < 0.01,
        "after one tau filter reaches ~63.2% of the step, got {:.4}",
        frac
    );
    // And it clearly LAGS: nowhere near the full step.
    assert!(
        frac < 0.7,
        "filter lags, must not reach the step in one tau"
    );
}

#[test]
fn iir_q_matches_f64_reference() {
    // Drive a varied raw sequence through both the Q filter and the f64 reference; the Q output must
    // track the f64 reference within a tight tolerance over many thousands of slow-tau steps.
    let mut q = ThrottleFilter::new();
    let mut r = ThrottleRefF64::default();

    let mut max_out_err: i32 = 0;
    let mut max_state_err: f64 = 0.0;
    for i in 0..60_000u32 {
        // A slow ramp with a square component, kept within raw range.
        let raw = (((i / 100) * 137) % 60000) as u16;
        let qo = q.step(raw) as i32;
        let ro = r.step(raw) as i32;
        max_out_err = max_out_err.max((qo - ro).abs());
        max_state_err = max_state_err.max((q.baseline_f64() - r.baseline).abs());
    }
    // The output (after +200 and truncation) must match within 1 LSB; the carry within a tiny
    // absolute drift given 32 fractional bits over 60k slow steps.
    assert!(
        max_out_err <= 1,
        "Q output within 1 LSB of f64 ref, got {}",
        max_out_err
    );
    assert!(
        max_state_err < 0.05,
        "Q carry tracks f64 carry, max drift {:.6}",
        max_state_err
    );
}

#[test]
fn iir_coefficients_sum_to_one() {
    // Ka + Kb = 1.0 exactly in the reference; the Q reproductions should sum essentially to 1.
    assert!((KA + KB - 1.0).abs() < 1e-12);
}
