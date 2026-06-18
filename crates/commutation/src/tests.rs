//! Host tests for the FOC math layer (the pure functions). Where the spec gives an exact constant
//! the exact value is asserted. The key new test is the stall-aware anti-windup test (the
//! open-gaps fix).

use crate::foc::*;

// --- Section 5.2: the hall -> base-angle anchors, asserted EXACTLY ----------------------------

#[test]
fn hall_base_angle_anchors_exact() {
    assert_eq!(BASE_ANGLE[1], 0x9554);
    assert_eq!(BASE_ANGLE[2], 0xEAAB);
    assert_eq!(BASE_ANGLE[3], 0xBFFF);
    assert_eq!(BASE_ANGLE[4], 0x4000);
    assert_eq!(BASE_ANGLE[5], 0x6AAA);
    assert_eq!(BASE_ANGLE[6], 0x1556);
    // Spacing: the six anchors are ~0x2AAA apart in ascending order 6,4,5,1,3,2 (the reference
    // anchors are the verbatim values; the 60 deg = 10922.67 spacing rounds to 0x2AAA/0x2AAB/0x2AAC
    // across the circle, within 2 LSB).
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
        assert!((delta - 0x2AAA).abs() <= 2, "anchor spacing {} off 0x2AAA", delta);
    }
}

// --- Section 5.4: the forward / reverse single-step published check values --------------------

/// Drive one period per code in the given order with one period per code (interval blend = 1),
/// returning the published angle at each code.
fn step_sequence(order: &[u8]) -> std::vec::Vec<u16> {
    let mut c = Commutation::new();
    let mut out = std::vec::Vec::new();
    for &code in order {
        out.push(c.step(code));
    }
    out
}

#[test]
fn interp_forward_reverse_check_values() {
    // Forward order 1 -> 3 -> 2 -> 6 -> 4 -> 5 (dir = +1). After warm-up the published angle is
    // base + dir*0x1555. We run two full laps so direction + interval are established, and check
    // the second lap.
    let fwd_order: std::vec::Vec<u8> = [1u8, 3, 2, 6, 4, 5]
        .iter()
        .cloned()
        .cycle()
        .take(18)
        .collect();
    let res = step_sequence(&fwd_order);
    // Map the last full lap's codes to expected published values.
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
    // The last 6 entries are a steady lap.
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

// --- Section 7: sine table + lookup check values ----------------------------------------------

#[test]
fn sine_table_bitexact() {
    assert_eq!(SIN_QUARTER[0], 0);
    assert_eq!(SIN_QUARTER[1], 201);
    assert_eq!(SIN_QUARTER[127], 23027);
    assert_eq!(SIN_QUARTER[128], 23170);
    assert_eq!(SIN_QUARTER[255], 32766);
}

#[test]
fn q15_typed_view_round_trips() {
    // The FOC vectors are Q15; the typed view wraps a raw i16 and recovers it bit-for-bit.
    let (s, c) = lookup_sincos(0x4000);
    assert_eq!(q15(s).to_bits(), s);
    assert_eq!(q15(c).to_bits(), c);
    // 0x4000 (90 deg): sin ~= +1.0, cos ~= 0.
    assert!(q15(s) > Q15::from_num(0.99));
    assert!(q15(c).abs() < Q15::from_num(0.01));
}

#[test]
fn lookup_check_values() {
    assert_eq!(lookup_sincos(0x0000), (0, 32766));
    assert_eq!(lookup_sincos(0x1000), (12539, 30195));
    assert_eq!(lookup_sincos(0x4000), (32766, 0));
    assert_eq!(lookup_sincos(0x5FC0), (23170, -23027));
    assert_eq!(lookup_sincos(0x8000), (0, -32766));
    assert_eq!(lookup_sincos(0xC000), (-32766, 0));
    assert_eq!(lookup_sincos(0xE000), (-23027, 23170));
}

// --- Section 6 step 2: Clarke / Park check values + round-trip --------------------------------

#[test]
fn park_forward_check_values() {
    // alpha=19660, beta=0 at theta=0 -> (d, q) = (19658, 0).
    assert_eq!(park_forward(19660, 0, 0x0000), (19658, 0));
    // same input at theta=0x4000 (90 deg) -> (d, q) = (0, 19658).
    assert_eq!(park_forward(19660, 0, 0x4000), (0, 19658));
}

#[test]
fn park_inverse_check_values() {
    // d=19660, q=0 at theta=0 -> (alpha, beta) = (19658, 0).
    assert_eq!(park_inverse(19660, 0, 0x0000), (19658, 0));
    // d = q = 32767 at theta=0x2000 (45 deg) -> alpha = -19341 (wrap of +46195), beta = -142.
    assert_eq!(park_inverse(32767, 32767, 0x2000), (-19341, -142));
}

#[test]
fn clarke_park_round_trip() {
    // Clarke then forward Park then inverse Park should reconstruct (alpha, beta) within rounding.
    let i_a = 12000i16;
    let i_b = -4000i16;
    let cl = clarke(i_a, i_b);
    for &theta in &[0x0000u16, 0x1234, 0x4000, 0x9ABC, 0xC000, 0xE321] {
        let (d, q) = park_forward(cl.alpha, cl.beta, theta);
        let (a2, b2) = park_inverse(d, q, theta);
        // Round-trip: forward then inverse Park is a proper rotation, so it reconstructs (alpha,
        // beta) up to the Q15 rounding through two rotations. The reference sine table peaks at
        // 32766 (not 32768), so c^2 + s^2 is ~0.9956, a ~0.45% systematic shrink per round-trip;
        // the bound is set as a fraction of the input magnitude (the circular limiter accounts for
        // the modulation-depth loss downstream).
        let tol = (cl.alpha.unsigned_abs() as i32 + cl.beta.unsigned_abs() as i32) / 100 + 8;
        assert!(
            (a2 as i32 - cl.alpha as i32).abs() <= tol,
            "alpha round-trip theta=0x{:04X}: {} vs {}",
            theta,
            a2,
            cl.alpha
        );
        assert!(
            (b2 as i32 - cl.beta as i32).abs() <= tol,
            "beta round-trip theta=0x{:04X}: {} vs {}",
            theta,
            b2,
            cl.beta
        );
    }
}

#[test]
fn clarke_constants_exact() {
    assert_eq!(CLARKE_A, 0x49E6);
    assert_eq!(CLARKE_B, 0x93CC);
    // alpha passes straight through.
    assert_eq!(clarke(5000, 1234).alpha, 5000);
}

// --- Section 8: offset cal with the window ----------------------------------------------------

#[test]
fn current_from_adc_formula() {
    // current = offset - 2*sample, saturated.
    assert_eq!(current_from_adc(0x7FB8, 0x3FDC), 0); // 0x7FB8 - 2*0x3FDC = 0
    assert_eq!(current_from_adc(0x8000, 0x1000), 0x6000);
    // Saturation: very small sample drives above +0x7FFF.
    assert_eq!(current_from_adc(0xFFFF, 0), 0x7FFF);
    // -0x8000 sentinel maps to -0x7FFF.
    assert_eq!(current_from_adc(0, 0x4000), -0x7FFF); // 0 - 0x8000 = -0x8000 -> -0x7FFF
}

#[test]
fn offset_window_check() {
    assert_eq!(CAL_WINDOW_LO, 0x7531);
    assert_eq!(CAL_WINDOW_HI, 0x86C4);
    // The bench-measured healthy offsets are inside the window.
    assert!(offset_in_window(0x7FB8));
    assert!(offset_in_window(0x7DAE));
    // Boundaries: lo inclusive, hi exclusive.
    assert!(offset_in_window(0x7531));
    assert!(!offset_in_window(0x86C4));
    assert!(!offset_in_window(0x7530));
}

#[test]
fn calibrate_offset_accumulation() {
    // 16 samples of a mid-scale left-aligned reading accumulate (sample>>3) to ~2x the average.
    // A constant left-aligned sample of 0x3FDC: (0x3FDC>>3)=0x7FB, *16 = 0x7FB0 ~ inside window.
    let samples = [0x3FDCu16; 16];
    let off = calibrate_offset(&samples);
    assert_eq!(off, ((0x3FDCu16 >> 3) as u32 * 16) as u16);
    assert!(offset_in_window(off));
}

// --- Section 6 step 3: the q/d PI hand-computed output ----------------------------------------

#[test]
fn q_pi_hand_computed_output() {
    // With a fresh integrator and a known q error, the first-period output is deterministic.
    // pi_step(0, q_meas): e = -q_meas. Kp=100, P_div=1024, Ki=50, I_div=8192.
    // I += e*Ki = -q_meas*50 (clamped well inside +-0x0FFFE000 for a small q).
    // out = I/8192 + (e*100)/1024.
    let mut pi = QAxisPi::new();
    // Use the non-stalled path (rotating) so it is the stock PI.
    let q_meas = 1000i32;
    let out = pi.step(q_meas, /*rotating*/ true, /*commanded*/ true);
    let e = -q_meas;
    let i_acc = (e as i64) * 50; // -50000
    let i_term = i_acc / 8192; // -6
    let p_term = ((e * 100) / 1024) as i64; // -97
    let expected = (i_term + p_term) as i16; // -103
    assert_eq!(out, expected, "q-PI out {} expected {}", out, expected);
    assert_eq!(pi.pi.accumulator, i_acc);
}

// --- THE KEY NEW TEST: stall-aware anti-windup (the open-gaps fix) -----------------------------

#[test]
fn stall_aware_antiwindup_does_not_peg() {
    // Reproduce the open-gaps pathology: a nonzero torque command (commanded=true) but the rotor
    // does NOT rotate (rotating=false), with a small residual q_meas bias the PI can never null.
    // Stock would wind the integrator to its clamp (+-0x0FFFE000) and peg the output to ~32767
    // (~1 A). The stall-aware anti-windup must keep the integrator bounded and the output small.
    let mut pi = QAxisPi::new();
    let residual_q: i32 = 50; // a small persistent bias

    let int_clamp = 0x0FFFE000i64; // +-this is the stock integrator clamp
    let mut max_abs_int: i64 = 0;
    let mut max_abs_out: i32 = 0;

    // Run many periods of the stalled, commanded case.
    for _ in 0..100_000 {
        let out = pi.step(residual_q, /*rotating*/ false, /*commanded*/ true);
        max_abs_int = max_abs_int.max(pi.pi.accumulator.abs());
        max_abs_out = max_abs_out.max((out as i32).abs());
    }

    // The integrator must NOT wind to its clamp.
    assert!(
        max_abs_int < int_clamp / 4,
        "stalled q integrator wound to {} (clamp {}); anti-windup failed",
        max_abs_int,
        int_clamp
    );
    // The output must NOT peg to ~full-scale (~1 A). Bound it well below the +-32767 clamp.
    assert!(
        max_abs_out < 1000,
        "stalled q-PI output pegged to {} (clamp 32767); anti-windup failed",
        max_abs_out
    );

    // Contrast: the SAME residual on a ROTATING rotor runs the stock PI, which winds (this is the
    // case stock relies on micro-movements to escape; here we only assert the anti-windup path
    // diverges from the stock path, i.e. the fix actually changes behavior).
    let mut stock = QAxisPi::new();
    for _ in 0..100_000 {
        stock.step(residual_q, /*rotating*/ true, /*commanded*/ true);
    }
    assert!(
        stock.pi.accumulator.abs() > max_abs_int,
        "stock (rotating) integrator {} should exceed the stalled-bounded {}",
        stock.pi.accumulator.abs(),
        max_abs_int
    );
}

#[test]
fn stall_antiwindup_not_active_when_not_commanded() {
    // When NOT commanded (demand 0), the stock PI runs even if not rotating (the loop still
    // regulates q to zero; there is no breakaway demand to wind on). Verify the stock path is used.
    let mut pi = QAxisPi::new();
    let out = pi.step(1000, /*rotating*/ false, /*commanded*/ false);
    // Same as the hand-computed stock first step.
    let e = -1000i32;
    let expected = ((e as i64 * 50 / 8192) + ((e * 100 / 1024) as i64)) as i16;
    assert_eq!(out, expected);
}

// --- Section 6 step 5: circular magnitude limit -----------------------------------------------

#[test]
fn circular_limit_clamps() {
    assert_eq!(CIRC_THRESH, 0x3D75_9621);
    assert_eq!(CIRC_GAIN.len(), 67);
    // Inside the circle: pass through unchanged.
    let (d, q) = circular_limit(1000, 1000);
    assert_eq!((d, q), (1000, 1000));
    // Outside the circle: magnitude is reduced, ratio (angle) approximately preserved.
    let din = 32767i16;
    let qin = 32767i16;
    let sq_in = din as i64 * din as i64 + qin as i64 * qin as i64;
    assert!(sq_in as u32 > CIRC_THRESH);
    let (d2, q2) = circular_limit(din, qin);
    let sq_out = d2 as i64 * d2 as i64 + q2 as i64 * q2 as i64;
    assert!(
        (sq_out as u32) <= CIRC_THRESH + (CIRC_THRESH / 50),
        "limited magnitude {} not within ~the circle {}",
        sq_out,
        CIRC_THRESH
    );
    // Equal d,q in -> still approximately equal out (ratio preserved).
    assert!((d2 as i32 - q2 as i32).abs() <= 2);
    // First gain-table entry is the smallest over-threshold bin.
    assert_eq!(CIRC_GAIN[0], 32494);
    assert_eq!(CIRC_GAIN[66], 22661);
}

// --- Section 9: SVPWM sector selection + duties -----------------------------------------------

#[test]
fn svpwm_constants_exact() {
    assert_eq!(SVPWM_BETA, 9000);
    assert_eq!(SVPWM_ALPHA, 0x3CE4);
    assert_eq!(SVPWM_CENTER, 0x465);
}

#[test]
fn svpwm_sector_selection() {
    // beta = 0, alpha > 0: p = (15588*alpha)/2 > 0, q = (-15588*alpha)/2 < 0, beta_term = 0 <= 0
    //   -> p>=0, q<0, beta_term<=0 -> sector 6.
    assert_eq!(svpwm_sector(10000, 0), 6);
    // beta = 0, alpha < 0: p<0, q>=0, beta_term<=0 -> sector 4.
    assert_eq!(svpwm_sector(-10000, 0), 4);
    // beta > 0 (B = -9000*beta < 0), alpha = 0: p = B/2 < 0, q = B/2 < 0 -> sector 5.
    assert_eq!(svpwm_sector(0, 10000), 5);
    // beta < 0 (B > 0), alpha = 0: p = B/2 > 0, q = B/2 > 0 -> sector 2.
    assert_eq!(svpwm_sector(0, -10000), 2);
    // beta < 0, large +alpha: p>=0, q<0, beta_term = -9000*beta > 0 (not <=0) -> sector 1.
    assert_eq!(svpwm_sector(10000, -1000), 1);
    // beta < 0, large -alpha: p<0, q>=0, beta_term > 0 -> sector 3.
    assert_eq!(svpwm_sector(-10000, -1000), 3);
}

#[test]
fn svpwm_duties_centered_at_zero_vector() {
    // The zero vector (alpha=beta=0) gives all three compares near the half-period 0x465 (1125).
    let s = svpwm(0, 0);
    // base = rsh18(9000) + 0x465; rsh18(9000) = 9000 >> 18 = 0. So base = 0x465.
    assert_eq!(s.base, 0x465);
    assert_eq!(s.c1, 0x465);
    assert_eq!(s.c2, 0x465);
}

#[test]
fn svpwm_duties_known_vector() {
    // A representative in-range vector: assert the duties are on the 0..2250 scale and the sector
    // matches the selection function.
    let alpha = 8000i16;
    let beta = 4000i16;
    let s = svpwm(alpha, beta);
    assert_eq!(s.sector, svpwm_sector(alpha, beta));
    for d in [s.base, s.c1, s.c2] {
        assert!(d <= 2250, "duty {} out of 0..2250", d);
    }
}

// --- rsh17 / rsh18 toward-zero rounding -------------------------------------------------------

#[test]
fn rsh_round_toward_zero() {
    // Negative operands round toward zero (the bias).
    assert_eq!(rsh17(-(1 << 17)), -1);
    assert_eq!(rsh17(-(1 << 17) + 1), 0); // truncates toward zero
    assert_eq!(rsh18(-(1 << 18)), -1);
    assert_eq!(rsh18((1 << 18) - 1), 0);
    assert_eq!(rsh17(1 << 17), 1);
}

// --- d-axis ramp (no zero-torque deadband) ----------------------------------------------------

#[test]
fn d_ramp_constants_and_relax_branch() {
    assert_eq!(RAMP_THRESH, 800);
    assert_eq!(RAMP_STEP, 0);
    // Demand 0 -> relax branch holds s (STEP = 0) and resets the counter to 0x20.
    let mut r = DRamp::default();
    r.s = 1234;
    let out = r.step(0);
    assert_eq!(out, 1234); // held, no deadband, no ramp in relax
    assert_eq!(r.counter, 0x20);
}

// --- full foc_step smoke (the orchestration wires together) -----------------------------------

#[test]
fn foc_step_smoke() {
    let mut st = FocState::new(MotorParams::default());
    // A mid-scale current sample (near zero current) and hall code 1.
    let raw = [1u8, 0, 0]; // code 1
    let out = foc_step(&mut st, raw, 0x3FDC, 0x3FDC, 0);
    // Angle should be the base anchor for code 1 (or its interpolation), duties on the scale.
    for d in [out.svpwm.base, out.svpwm.c1, out.svpwm.c2] {
        assert!(d <= 2250);
    }
    assert!(!out.hall_fault);
}

// --- hall fault on persistent invalid code ----------------------------------------------------

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
