//! Host tests, slice 1: the shared primitives (`specs/commutation.md`, "Validation discipline").
//!
//! Recovered check values are asserted verbatim; blocks with real arithmetic (the sine table)
//! additionally track an f64 reference. The recovered vectors' provenance is the archived suite
//! (`archive/accumulated-build`) implementing the stock contract.

use super::foc::*;
use super::{MotorOutput, PhaseCmd, ARR, MID_RAIL};

// ---------------------------------------------------------------------------------------------
// Duty scale and angle constants (recovered relations).
// ---------------------------------------------------------------------------------------------

#[test]
fn duty_scale_constants() {
    // The stock timer contract: ARR 2250 at 72 MHz center-aligned = 16 kHz; mid-rail is the SVPWM
    // centering constant 0x465.
    assert_eq!(ARR, 2250);
    assert_eq!(MID_RAIL, 1125);
    assert_eq!(MID_RAIL, 0x465);
    assert_eq!(72_000_000 / (2 * ARR as u32), 16_000);
}

#[test]
fn angle_constants_are_the_recovered_relations() {
    // 60 deg = 65536/6 truncated; 90 deg = a quarter revolution.
    assert_eq!(SECTOR_ANGLE, 0x2AAA);
    assert_eq!(SECTOR_ANGLE, (65536u32 / 6) as u16);
    assert_eq!(ANGLE_90, 0x4000);
    assert_eq!(ANGLE_90 as u32, 65536 / 4);
}

// ---------------------------------------------------------------------------------------------
// The stock MAC / RND / sat16 forms (rounding, wrap, saturation, sentinel).
// ---------------------------------------------------------------------------------------------

#[test]
fn sat16_bounds_and_sentinel() {
    assert_eq!(sat16(0), 0);
    assert_eq!(sat16(32767), 32767);
    assert_eq!(sat16(32768), 32767);
    assert_eq!(sat16(i32::MAX), 32767);
    assert_eq!(sat16(-32767), -32767);
    // The -32768 sentinel is reserved: both exact and below map to -32767.
    assert_eq!(sat16(-32768), -32767);
    assert_eq!(sat16(i32::MIN), -32767);
}

#[test]
fn rnd_q15_rounds_half_away_from_zero() {
    // Positive: plain arithmetic >> 15 (no bias). 1.5 * 2^15 = 49152 -> 1 (truncates down).
    assert_eq!(rnd_q15(1 << 15), 1);
    assert_eq!(rnd_q15((1 << 15) - 1), 0);
    assert_eq!(rnd_q15(49152), 1);
    // Negative: the logical-shift bias adds 2^15 - 1, making the shift truncate toward zero
    // (round-half-away-from-zero overall). -1 * 2^15 -> -1; -(2^15 - 1) -> 0 (toward zero).
    assert_eq!(rnd_q15(-(1 << 15)), -1);
    assert_eq!(rnd_q15(-((1 << 15) - 1)), 0);
    assert_eq!(rnd_q15(-49152), -1);
}

#[test]
fn rnd_q15_wraps_over_range_no_saturate() {
    // The RND form deliberately WRAPS mod 2^16 (used by inverse Park; defined behavior).
    // 40000 * 2^15 >> 15 = 40000, which as i16 wraps to 40000 - 65536 = -25536.
    let acc = 40_000i32 << 15;
    assert_eq!(rnd_q15(acc), (40_000u16 as i16));
    assert_eq!(rnd_q15(acc), -25_536);
}

#[test]
fn mac_q15_saturates_where_rnd_wraps() {
    // Same over-range input: MAC saturates to +32767 instead of wrapping.
    let acc = 40_000i32 << 15;
    assert_eq!(mac_q15(acc), 32767);
    let acc = -(40_000i32 << 15);
    assert_eq!(mac_q15(acc), -32767);
    // In-range values agree with RND.
    for &v in &[0i32, 1, -1, 12345, -12345, 32767, -32767] {
        assert_eq!(mac_q15(v << 15), rnd_q15(v << 15));
    }
}

// ---------------------------------------------------------------------------------------------
// The sine table and quadrant-folded lookup (recovered check values + full f64 re-derivation).
// ---------------------------------------------------------------------------------------------

#[test]
fn sine_table_bitexact() {
    // The recovered endpoint/midpoint check values.
    assert_eq!(SIN_QUARTER[0], 0);
    assert_eq!(SIN_QUARTER[1], 201);
    assert_eq!(SIN_QUARTER[127], 23027);
    assert_eq!(SIN_QUARTER[128], 23170);
    assert_eq!(SIN_QUARTER[255], 32766);
}

#[test]
fn sine_table_matches_f64_reference_every_entry() {
    // Every entry is round(32767 * sin((i/256) * pi/2)), exactly.
    for (i, &v) in SIN_QUARTER.iter().enumerate() {
        let want = (32767.0 * ((i as f64) / 256.0 * core::f64::consts::FRAC_PI_2).sin()).round();
        assert_eq!(v as f64, want, "entry {i}");
    }
}

#[test]
fn lookup_check_values() {
    // The recovered quadrant vectors.
    assert_eq!(lookup_sincos(0x0000), (0, 32766));
    assert_eq!(lookup_sincos(0x1000), (12539, 30195));
    assert_eq!(lookup_sincos(0x4000), (32766, 0));
    assert_eq!(lookup_sincos(0x5FC0), (23170, -23027));
    assert_eq!(lookup_sincos(0x8000), (0, -32766));
    assert_eq!(lookup_sincos(0xC000), (-32766, 0));
    assert_eq!(lookup_sincos(0xE000), (-23027, 23170));
}

#[test]
fn lookup_tracks_f64_sincos_over_the_full_circle() {
    // The quadrant folding must track f64 sin/cos over all 65536 angles within the table's
    // quantization (the table is 256 entries per quadrant with truncating index math, so allow
    // one index step of slack: sin changes by at most ~201 per entry).
    let tol = 210.0;
    for step in 0..1024u32 {
        let theta = (step * 64) as u16;
        let (s, c) = lookup_sincos(theta);
        let rad = (theta as f64) / 65536.0 * core::f64::consts::TAU;
        let want_s = 32767.0 * rad.sin();
        let want_c = 32767.0 * rad.cos();
        assert!(
            (s as f64 - want_s).abs() < tol,
            "sin at {theta:#06x}: got {s}, want {want_s:.0}"
        );
        assert!(
            (c as f64 - want_c).abs() < tol,
            "cos at {theta:#06x}: got {c}, want {want_c:.0}"
        );
    }
}

#[test]
fn lookup_sin_cos_quadrature_relation() {
    // cos(theta) == sin(theta + 90 deg) exactly, by the folding construction.
    for step in 0..256u32 {
        let theta = (step * 257) as u16;
        let (_, c) = lookup_sincos(theta);
        let (s_shifted, _) = lookup_sincos(theta.wrapping_add(super::foc::ANGLE_90));
        assert_eq!(c, s_shifted, "at {theta:#06x}");
    }
}

// ---------------------------------------------------------------------------------------------
// The output vocabulary.
// ---------------------------------------------------------------------------------------------

#[test]
fn phase_cmd_vocabulary_carries_duty_and_float() {
    // The vocabulary is data: a drive count on the duty scale, or Float; MOE is not expressible.
    let out = MotorOutput {
        phases: [
            PhaseCmd::Drive(MID_RAIL),
            PhaseCmd::Drive(0),
            PhaseCmd::Float,
        ],
    };
    assert_eq!(out.phases[0], PhaseCmd::Drive(1125));
    assert_ne!(out.phases[1], PhaseCmd::Float);
    assert_eq!(out.phases[2], PhaseCmd::Float);
}

// ---------------------------------------------------------------------------------------------
// Slice 2: the shared hall front-end (recovered check values + properties).
// ---------------------------------------------------------------------------------------------

/// Drive a bare `Commutation` through a code sequence, one period per code (the recovered test
/// helper).
fn step_sequence(order: &[u8]) -> std::vec::Vec<u16> {
    let mut c = Commutation::new();
    let mut out = std::vec::Vec::new();
    for &code in order {
        out.push(c.step(code));
    }
    out
}

/// Raw hall lines for a 3-bit code (code = A | B<<1 | C<<2).
fn lines(code: u8) -> [u8; 3] {
    [code & 1, (code >> 1) & 1, (code >> 2) & 1]
}

#[test]
fn hall_base_angle_anchors_exact() {
    // The recovered anchors, bench-confirmed against live stock.
    assert_eq!(BASE_ANGLE[1], 0x9554);
    assert_eq!(BASE_ANGLE[2], 0xEAAB);
    assert_eq!(BASE_ANGLE[3], 0xBFFF);
    assert_eq!(BASE_ANGLE[4], 0x4000);
    assert_eq!(BASE_ANGLE[5], 0x6AAA);
    assert_eq!(BASE_ANGLE[6], 0x1556);
    // Spacing: the six anchors are ~0x2AAA apart in ascending order 6,4,5,1,3,2 (60 deg =
    // 10922.67 rounds to 0x2AAA..0x2AAC across the circle, within 2 LSB).
    let ascending = [
        BASE_ANGLE[6],
        BASE_ANGLE[4],
        BASE_ANGLE[5],
        BASE_ANGLE[1],
        BASE_ANGLE[3],
        BASE_ANGLE[2],
    ];
    for w in ascending.windows(2) {
        let delta = w[1].wrapping_sub(w[0]) as i32;
        assert!(
            (delta - 0x2AAA).abs() <= 2,
            "anchor spacing {delta} off 0x2AAA"
        );
    }
}

#[test]
fn interp_forward_reverse_check_values() {
    // Forward order 1 -> 3 -> 2 -> 6 -> 4 -> 5 (dir = +1). After warm-up the published angle is
    // base + dir*0x1555. Two full laps establish direction + interval; check the second lap.
    let fwd_order: std::vec::Vec<u8> = [1u8, 3, 2, 6, 4, 5]
        .iter()
        .cloned()
        .cycle()
        .take(18)
        .collect();
    let res = step_sequence(&fwd_order);
    let expected_fwd: std::collections::BTreeMap<u8, u16> = [
        (1u8, 0xAAA9u16),
        (3, 0xD554),
        (2, 0x0000),
        (6, 0x2AAB),
        (4, 0x5555),
        (5, 0x7FFF),
    ]
    .into_iter()
    .collect();
    let last_codes = &fwd_order[12..18];
    for (idx, &code) in last_codes.iter().enumerate() {
        let got = res[12 + idx];
        assert_eq!(
            got, expected_fwd[&code],
            "forward code {} published 0x{:04X}, expected 0x{:04X}",
            code, got, expected_fwd[&code]
        );
    }

    // Reverse order 1 -> 5 -> 4 -> 6 -> 2 -> 3 (dir = -1): published = base - 0x1555.
    let rev_order: std::vec::Vec<u8> = [1u8, 5, 4, 6, 2, 3]
        .iter()
        .cloned()
        .cycle()
        .take(18)
        .collect();
    let rres = step_sequence(&rev_order);
    let expected_rev: std::collections::BTreeMap<u8, u16> = [
        (1u8, 0x7FFFu16),
        (5, 0x5555),
        (4, 0x2AAB),
        (6, 0x0001),
        (2, 0xD556),
        (3, 0xAAAA),
    ]
    .into_iter()
    .collect();
    let last_rcodes = &rev_order[12..18];
    for (idx, &code) in last_rcodes.iter().enumerate() {
        let got = rres[12 + idx];
        assert_eq!(
            got, expected_rev[&code],
            "reverse code {} published 0x{:04X}, expected 0x{:04X}",
            code, got, expected_rev[&code]
        );
    }
}

#[test]
fn hall_fault_after_persistent_invalid() {
    let mut c = Commutation::new();
    // A single invalid sample must not fault.
    c.step(0);
    assert!(!c.hall_fault);
    // Persistent invalid (> 64) faults.
    for _ in 0..70 {
        c.step(7);
    }
    assert!(c.hall_fault);
}

#[test]
fn hall_fault_dwell_threshold_relation() {
    // fault when dwell * 250 > 16000, i.e. dwell > 64: the 65th consecutive invalid faults.
    assert_eq!(HALL_FAULT_DWELL_LIMIT / HALL_FAULT_DWELL_MUL, 64);
    let mut c = Commutation::new();
    for _ in 0..64 {
        c.step(0);
    }
    assert!(!c.hall_fault, "64 invalid periods must not fault yet");
    c.step(0);
    assert!(c.hall_fault, "the 65th invalid period faults");
}

#[test]
fn debounce_assembles_code_and_locks_out_bounce() {
    let mut d = HallDebounce::new();
    assert_eq!(d.reload, HALL_DEBOUNCE_RELOAD);
    assert_eq!(HALL_DEBOUNCE_RELOAD, 150);

    // From all-low, raise line A: the edge is accepted immediately (lockout starts at 0) and the
    // code assembles as A | B<<1 | C<<2.
    assert_eq!(d.step([1, 0, 0]), 0b001);
    assert!(d.changed[0] && !d.changed[1] && !d.changed[2]);

    // A bounce back on the same line is IGNORED for exactly `reload` periods...
    for k in 0..149 {
        assert_eq!(d.step([0, 0, 0]), 0b001, "still locked at period {k}");
    }
    // ...and the level change is accepted once the lockout has drained.
    assert_eq!(d.step([0, 0, 0]), 0b000);
}

#[test]
fn debounce_lines_are_independent() {
    let mut d = HallDebounce::new();
    let _ = d.step([1, 0, 0]); // A edge: A locked
                               // B edges while A is locked: B has its own lockout and is accepted.
    assert_eq!(d.step([1, 1, 0]), 0b011);
    // C likewise.
    assert_eq!(d.step([1, 1, 1]), 0b111);
}

#[test]
fn speed_window_latches_signed_edge_count() {
    // Steady forward rotation, one commutation edge every K periods: after the 320-period window
    // latches, |speed| tracks the f64 expectation (321/K edges per window) and the sign follows
    // the direction.
    let k = 10usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let expect = 321.0_f64 / k as f64;

    let mut c = Commutation::new();
    let mut idx = 0usize;
    for period in 0..2000usize {
        if period % k == 0 {
            idx += 1;
        }
        c.step(fwd[idx % 6]);
    }
    assert!(
        (c.speed as f64 - expect).abs() <= 1.5,
        "forward speed {} vs expected ~{expect:.1}",
        c.speed
    );

    let rev = [1u8, 5, 4, 6, 2, 3];
    let mut c = Commutation::new();
    let mut idx = 0usize;
    for period in 0..2000usize {
        if period % k == 0 {
            idx += 1;
        }
        c.step(rev[idx % 6]);
    }
    assert!(
        (c.speed as f64 + expect).abs() <= 1.5,
        "reverse speed {} vs expected ~-{expect:.1}",
        c.speed
    );
}

#[test]
fn interpolation_slope_tracks_f64_rate_between_edges() {
    // Steady forward rotation at interval K: between edges the published angle ramps by the
    // per-period increment dir * (0x2AAA / blend); with blend converged to K that integer slope
    // must match the f64 electrical rate 65536 / (6 * K) within 1 count/period.
    let k = 8usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut c = Commutation::new();
    let mut published = std::vec::Vec::new();
    let mut idx = 0usize;
    for period in 0..(k * 24) {
        if period % k == 0 {
            idx += 1;
        }
        published.push(c.step(fwd[idx % 6]));
    }
    // Steady by the third lap. Check the deltas inside one inter-edge run (skip the edge period
    // itself, where the base snaps).
    let want_slope = 65536.0 / (6.0 * k as f64);
    let start = k * 18 + 1;
    for i in start..start + (k - 2) {
        let delta = published[i + 1].wrapping_sub(published[i]) as i32;
        assert!(
            (delta as f64 - want_slope).abs() < 1.0,
            "slope {delta} at {i} vs f64 {want_slope:.2}"
        );
        assert_eq!(
            delta, c.increment,
            "the steady slope is the integer increment"
        );
    }
    // And the integer increment is the recovered formula at the converged blend.
    assert_eq!(c.increment, SECTOR_ANGLE as i32 / k as i32);
}

#[test]
fn front_end_shares_state_and_survives_a_consumer_switch() {
    // The mode-model contract, the front-end's side: RotorFrontEnd has NO per-mode reset; a
    // method switch changes only who consumes RotorState. Simulate a switch mid-run (the same
    // input stream, consumers reading different fields before and after) and assert the angle
    // stream stays continuous across the boundary: the sample-to-sample delta at the switch is
    // the same bounded per-period step as everywhere else, not a snap to a reset state.
    // K = 50 periods per sector: each hall LINE toggles every 3K = 150 periods, exactly the
    // debounce lockout, so the stream is the fastest the reference debounce passes cleanly.
    let k = 50usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut fe = RotorFrontEnd::new();
    let mut idx = 0usize;
    let mut prev: Option<u16> = None;
    let mut max_delta_before = 0i32;
    let mut delta_at_switch = 0i32;
    let switch_at = k * 18; // steady state
    for period in 0..(k * 24) {
        if period % k == 0 {
            idx += 1;
        }
        let st = fe.step(lines(fwd[idx % 6]));
        // "Consumers": six-step reads the code before the switch, sine/FOC read the angle after.
        if period < switch_at {
            let _ = st.code;
        } else {
            let _ = st.angle;
        }
        if let Some(p) = prev {
            let delta = (st.angle.wrapping_sub(p) as i16 as i32).abs();
            if period > k * 12 && period < switch_at {
                max_delta_before = max_delta_before.max(delta);
            }
            if period == switch_at {
                delta_at_switch = delta;
            }
        }
        prev = Some(st.angle);
    }
    assert!(max_delta_before > 0);
    assert!(
        delta_at_switch <= max_delta_before,
        "switch delta {delta_at_switch} exceeds steady bound {max_delta_before}: state was reset"
    );
}

#[test]
fn rotor_state_mirrors_the_estimator() {
    // RotorFrontEnd::step is exactly hall.step then comm.step (the recovered FOC sequence); the
    // snapshot mirrors the estimator's fields.
    let mut fe = RotorFrontEnd::new();
    let st = fe.step(lines(4));
    assert_eq!(st.code, 4);
    assert_eq!(st.angle, fe.comm.angle);
    assert_eq!(st.speed, fe.comm.speed);
    assert_eq!(st.in_window, fe.comm.in_window);
    assert_eq!(st.hall_fault, fe.comm.hall_fault);
}
