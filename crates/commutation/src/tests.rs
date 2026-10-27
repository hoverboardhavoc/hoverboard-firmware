//! Host tests, slice 1: the shared primitives (`specs/commutation.md`, "Validation discipline").
//!
//! Recovered check values are asserted verbatim; blocks with real arithmetic (the sine table)
//! additionally track an f64 reference. The recovered vectors' provenance is the archived suite
//! (`archive/accumulated-build`) implementing the stock contract.

extern crate std;
use std::vec::Vec;

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

// The three front-end constructors now take the period-ISR rate they will be stepped at, because
// the hall debounce window is a TIME (`foc::HALL_DEBOUNCE_US`). Every test runs at the reference
// rate the recovered constants were measured at, so it is named once here rather than at 21 call
// sites; `hall_debounce_is_a_time_not_a_period_count` is the test that varies it.
fn debounce() -> HallDebounce {
    HallDebounce::new(REF_PERIOD_HZ)
}

fn front_end() -> RotorFrontEnd {
    RotorFrontEnd::new(REF_PERIOD_HZ)
}

fn commutator(method: super::MethodState) -> super::Commutator {
    commutator_with(method, super::Direction::Forward)
}

fn commutator_with(method: super::MethodState, direction: super::Direction) -> super::Commutator {
    commutator_aligned(method, direction, 0)
}

/// A commutator for a motor whose hall-to-phase alignment is `(direction, align_offset)`: the
/// dispatch installs the same decode the six-step records are built from, so one helper serves
/// both paths.
fn commutator_aligned(
    method: super::MethodState,
    direction: super::Direction,
    align_offset: u8,
) -> super::Commutator {
    super::Commutator::new(method, REF_PERIOD_HZ, SixStep::new(direction, align_offset))
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
    let mut d = debounce();
    // At the reference 16 kHz the window lowers to the bit-exact recovered period count.
    assert_eq!(d.reload, 150, "0x96, the recovered reload at 16 kHz");
    assert_eq!(d.reload, hall_debounce_periods(REF_PERIOD_HZ));

    // From all-low, raise line A: the edge is accepted immediately (lockout starts at 0) and the
    // code assembles as A | B<<1 | C<<2.
    assert_eq!(d.step([1, 0, 0]), 0b001);
    assert!(d.changed[0] && !d.changed[1] && !d.changed[2]);

    // A bounce back on the same line, with no other line moving, is IGNORED for exactly `reload`
    // periods...
    for k in 0..149 {
        assert_eq!(d.step([0, 0, 0]), 0b001, "still locked at period {k}");
    }
    // ...and the level change is accepted once the lockout has drained.
    assert_eq!(d.step([0, 0, 0]), 0b000);
}

#[test]
fn debounce_lines_are_independent() {
    let mut d = debounce();
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

// ---------------------------------------------------------------------------------------------
// The period-based wheel rate (`specs/control.md` (o)).
// ---------------------------------------------------------------------------------------------

/// Drive `c` through `codes` holding each for `hold` periods (the first code is entered on the
/// first call), returning the rate after every period.
fn rate_trace(c: &mut Commutation, codes: &[u8], hold: usize) -> Vec<i32> {
    let mut out = Vec::new();
    for &code in codes {
        for _ in 0..hold {
            c.step(code);
            out.push(c.rate_q);
        }
    }
    out
}

/// A constant interval gives exactly `RATE_Q_NUM / interval` from the second edge on, and nothing
/// after the first: the first counted edge establishes the direction, the second completes the
/// pair. An interval of exactly one window is exactly one count, `RATE_Q_UNIT`, which is what
/// keeps the outer loop's gains meaning what they meant against the edge count.
#[test]
fn the_period_rate_is_the_reciprocal_of_a_same_direction_pair() {
    assert_eq!(RATE_Q_NUM, 4096 * 320);
    let fwd = [1u8, 3, 2, 6, 4, 5, 1, 3];
    for k in [1usize, 7, 92, 320, 1_840] {
        let mut c = Commutation::new();
        let trace = rate_trace(&mut c, &fwd, k);
        // Code 1 (no edge yet), code 3 (first edge: direction only), then code 2 (the pair).
        assert!(
            trace[..2 * k].iter().all(|&r| r == 0),
            "k={k}: nothing before the pair"
        );
        let expect = RATE_Q_NUM / k as i32;
        assert_eq!(
            trace[2 * k],
            expect,
            "k={k}: the pair's edge publishes the reciprocal"
        );
        assert!(
            trace[2 * k..].iter().all(|&r| r == expect),
            "k={k}: steady rotation holds the estimate between edges"
        );
    }
    let mut c = Commutation::new();
    rate_trace(&mut c, &[1, 3, 2], 320);
    assert_eq!(c.rate_q, RATE_Q_UNIT, "one edge per window is one count");

    // Reverse rotation: the same magnitude, the counter's other sign.
    let rev = [1u8, 5, 4, 6, 2, 3];
    let mut c = Commutation::new();
    rate_trace(&mut c, &rev, 10);
    assert_eq!(c.rate_q, -(RATE_Q_NUM / 10));
}

/// Between edges the estimate decays as `RATE_Q_NUM / max(interval, since_change)`, period by
/// period, so a wheel that stops reads a speed falling as `1/t`: half the reading one interval
/// after the last edge, below one count `RATE_Q_NUM / RATE_Q_UNIT` = 320 periods after it, and a
/// few thousandths of a count once `since_change` sticks. No separate stop timeout.
#[test]
fn the_period_rate_decays_as_one_over_t_after_the_last_edge() {
    let mut c = Commutation::new();
    let k = 40usize;
    rate_trace(&mut c, &[1, 3], k);
    // The pair's edge, stepped by hand so the periods after it are counted from it.
    c.step(2);
    assert_eq!(c.rate_q, RATE_Q_NUM / k as i32);
    // Hold code 2: `since_change` was reset on the edge, so it reads n after n more periods.
    for n in 1..=4_000u32 {
        c.step(2);
        let expect = RATE_Q_NUM / (k as u32).max(n) as i32;
        assert_eq!(c.rate_q, expect, "period {n} after the last edge");
    }
    assert_eq!(
        c.rate_q,
        RATE_Q_NUM / 4_000,
        "a fortieth of a count 0.25 s in"
    );
    // Below one count exactly 320 periods after the edge, which is the arithmetic and not a
    // timeout.
    let mut c = Commutation::new();
    rate_trace(&mut c, &[1, 3], 10);
    c.step(2);
    for _ in 0..320 {
        c.step(2);
    }
    assert_eq!(c.rate_q, RATE_Q_UNIT);
    c.step(2);
    assert!(c.rate_q < RATE_Q_UNIT);
    // And it sticks near zero with `since_change`.
    for _ in 0..70_000 {
        c.step(2);
    }
    assert_eq!(c.since_change, 0xFFFF);
    assert_eq!(
        c.rate_q,
        RATE_Q_NUM / 0xFFFE,
        "the last value before the counter stuck"
    );
    assert!(c.rate_q < RATE_Q_UNIT / 100);
}

/// A reversal publishes 0 on the reversing edge and the new rate on the next, because a rate is a
/// pair of same-direction edges and the reversing edge is the first of its direction.
#[test]
fn a_reversal_reads_zero_until_the_next_same_direction_pair() {
    let mut c = Commutation::new();
    rate_trace(&mut c, &[1, 3, 2, 6], 20);
    assert_eq!(c.rate_q, RATE_Q_NUM / 20);
    // Back from 6 to 2: the reverse neighbour.
    c.step(2);
    assert_eq!(c.rate_q, 0, "the reversing edge reads zero");
    for _ in 0..30 {
        c.step(2);
        assert_eq!(c.rate_q, 0, "and nothing decays from zero");
    }
    // 2 -> 3 is the next reverse edge: the pair, 31 periods after the reversal.
    c.step(3);
    assert_eq!(c.rate_q, -(RATE_Q_NUM / 31));
}

/// THE twitch case (`specs/control.md` (o)): a balancing machine at rest rocks its wheel between
/// two hall codes, forward edge, backward edge, forward edge. No two consecutive edges agree, so
/// the rate reads exactly 0 throughout, where the edge count read `0, 1, 0, -1` and kicked the
/// torque on every one.
#[test]
fn a_rest_rock_between_two_hall_codes_reads_zero_throughout() {
    let mut c = Commutation::new();
    let rock: Vec<u8> = core::iter::repeat_n([1u8, 3], 50).flatten().collect();
    let trace = rate_trace(&mut c, &rock, 25);
    assert!(trace.iter().all(|&r| r == 0), "{:?}", &trace[..60]);
    // The edge count meanwhile still sees the edges (it is the recovered word and is not changed
    // by this): the window after a 1->3 edge holds a nonzero count.
    assert_ne!(c.edges_total, 0);
}

/// The recovered words are byte-identical beside the new one: the count per window and the
/// absolute edge count on the same drive as `speed_window_latches_signed_edge_count` read what they
/// read, and a `Commutation` fresh from `new` has the rate cells at zero.
#[test]
fn the_rate_leaves_the_count_per_window_and_the_odometer_alone() {
    let fresh = Commutation::new();
    assert_eq!(
        (fresh.rate_q, fresh.rate_dir, fresh.rate_interval),
        (0, 0, 0)
    );
    let k = 10usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut c = Commutation::new();
    let mut idx = 0usize;
    for period in 0..2000usize {
        if period % k == 0 {
            idx += 1;
        }
        c.step(fwd[idx % 6]);
    }
    assert!((c.speed as f64 - 321.0 / k as f64).abs() <= 1.5);
    // 200 code changes, the first of which enters code 3 from the fresh 0 and is not an edge.
    assert_eq!(c.edges_total, 199);
    assert_eq!(c.rate_q, RATE_Q_NUM / k as i32);
}

/// **The absolute edge count adds in BOTH directions, where the signed one cancels**
/// (`specs/odometer.md`, "Where the count comes from"). The same hundred edges forward and back
/// leave `edge_acc`/`speed` near zero and `edges_total` at the full count: that is the whole
/// reason the odometer has a word of its own rather than integrating the speed window.
#[test]
fn absolute_edges_count_both_directions_where_the_signed_window_cancels() {
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let rev = [1u8, 5, 4, 6, 2, 3];
    let mut c = Commutation::new();

    // Settle on code 1 first, so the first transition has a valid predecessor to be counted from.
    c.step(fwd[0]);
    for i in 1..=60usize {
        c.step(fwd[i % 6]);
    }
    assert_eq!(c.edges_total, 60, "sixty forward edges");
    let forward_acc = c.edge_acc;

    // The same number of edges back the other way. `rev` walks the reverse neighbour chain from
    // whichever code the forward run left, so each step is one counted reverse transition.
    let left_on = fwd[60 % 6];
    let start = rev.iter().position(|&c| c == left_on).unwrap();
    for i in 1..=60usize {
        c.step(rev[(start + i) % 6]);
    }
    assert_eq!(
        c.edges_total, 120,
        "reversing ADDS; it never winds the total back"
    );
    assert!(
        c.edge_acc.abs() < forward_acc.abs(),
        "the signed accumulator cancelled ({} after, {forward_acc} before)",
        c.edge_acc
    );
}

/// A code that is neither neighbour is noise and is not travel: the absolute count only moves on
/// the transitions the direction logic recognises, which is the same gate `edge_acc` has.
#[test]
fn an_unrecognised_hall_transition_adds_no_absolute_edge() {
    let mut c = Commutation::new();
    c.step(1);
    c.step(3); // a valid forward neighbour
    assert_eq!(c.edges_total, 1);
    c.step(4); // 3 -> 4 is neither FWD_NEXT nor REV_NEXT: a skipped state
    assert_eq!(c.edges_total, 1, "a skipped state is not an edge");
    c.step(0); // invalid code
    assert_eq!(c.edges_total, 1, "an invalid code is not an edge");
}

/// **The signed position steps with the direction and is never cleared**
/// (`specs/state-feedback.md` section 3): sixty forward edges read +60 across several speed
/// windows (each of which clears `edge_acc`), and sixty reverse edges bring it back to exactly 0
/// while `edges_total` reads 120.
#[test]
fn pos_edges_steps_with_the_direction_and_never_clears() {
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let rev = [1u8, 5, 4, 6, 2, 3];
    let mut c = Commutation::new();
    assert_eq!(c.pos_edges, 0);
    c.step(fwd[0]);
    // Ten periods per code, so the sixty edges span 600 periods: past the 320-period window that
    // latches and clears `edge_acc`.
    for i in 1..=60usize {
        for _ in 0..10 {
            c.step(fwd[i % 6]);
        }
        assert_eq!(c.pos_edges, i as i32, "forward edge {i}");
    }
    let left_on = fwd[60 % 6];
    let start = rev.iter().position(|&c| c == left_on).unwrap();
    for i in 1..=60usize {
        for _ in 0..10 {
            c.step(rev[(start + i) % 6]);
        }
        assert_eq!(c.pos_edges, 60 - i as i32, "reverse edge {i}");
    }
    assert_eq!(c.pos_edges, 0, "the same travel back is position zero");
    assert_eq!(
        c.edges_total, 120,
        "while the absolute count added both ways"
    );
    // A long rest clears nothing.
    for _ in 0..5_000 {
        c.step(rev[start % 6]);
    }
    assert_eq!(c.pos_edges, 0);
}

/// The position moves only on a counted neighbour transition, the gate `edge_acc` and
/// `edges_total` have: a skipped state and an invalid code are not travel.
#[test]
fn pos_edges_ignores_a_skipped_state_and_an_invalid_code() {
    let mut c = Commutation::new();
    c.step(1);
    c.step(3);
    assert_eq!(c.pos_edges, 1);
    c.step(4); // neither neighbour of 3
    assert_eq!(c.pos_edges, 1);
    c.step(0); // invalid
    assert_eq!(c.pos_edges, 1);
    c.step(2); // a reverse neighbour of 3, but the last valid code is 4: neither neighbour of 4
    c.step(3); // 2 -> 3 is the reverse step
    assert_eq!(c.pos_edges, 0);
}

/// It WRAPS rather than saturating or panicking (the consumer folds deltas): one forward edge
/// past `i32::MAX` is `i32::MIN`, and one back returns it.
#[test]
fn pos_edges_wraps() {
    let mut c = Commutation::new();
    c.step(1);
    c.pos_edges = i32::MAX;
    c.step(3);
    assert_eq!(c.pos_edges, i32::MIN);
    c.step(1);
    assert_eq!(c.pos_edges, i32::MAX);
}

/// The new cell changes none of the recovered words: the same drive from two commutators whose
/// positions start apart produces identical `speed`, `edges_total` and `rate_q` on every period.
#[test]
fn pos_edges_leaves_speed_edges_total_and_rate_q_alone() {
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let rev = [1u8, 5, 4, 6, 2, 3];
    let mut a = Commutation::new();
    let mut b = Commutation::new();
    b.pos_edges = 123_456;
    let mut idx = 0usize;
    for period in 0..4000usize {
        if period % 7 == 0 {
            idx += 1;
        }
        let code = if period < 2000 {
            fwd[idx % 6]
        } else {
            rev[(6 - idx % 6) % 6]
        };
        a.step(code);
        b.step(code);
        assert_eq!(
            (a.speed, a.edges_total, a.rate_q, a.edge_acc, a.angle),
            (b.speed, b.edges_total, b.rate_q, b.edge_acc, b.angle),
            "period {period}"
        );
    }
    assert_eq!(b.pos_edges.wrapping_sub(a.pos_edges), 123_456);
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
    // K = 50 periods per sector, a slow walk chosen so the angle interpolator has many periods
    // between sector changes to produce a steady slope over. (It predates the debounce fix, when
    // it was also the fastest rate the front end passed cleanly; that constraint is gone.)
    let k = 50usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut fe = front_end();
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
    let mut fe = front_end();
    let st = fe.step(lines(4));
    assert_eq!(st.code, 4);
    assert_eq!(st.angle, fe.comm.angle);
    assert_eq!(st.speed, fe.comm.speed);
    assert_eq!(st.in_window, fe.comm.in_window);
    assert_eq!(st.hall_fault, fe.comm.hall_fault);
}

// ---------------------------------------------------------------------------------------------
// Slice 3: the six-step arm (the example contract).
// ---------------------------------------------------------------------------------------------

use super::sixstep::{
    demand_to_duty, sixstep_step, PhaseDrive, SixStep, SixStepState, COAST, HALL_TO_SECTOR, STATES,
};
use super::{angle_path_offset, sine, CommutationMethod, Direction, MethodState};

/// f64 drive-vector angle (degrees) of a per-phase weight triple (A at 0, B at 120, C at 240).
fn vector_angle_deg(w: [f64; 3]) -> f64 {
    let (mut x, mut y) = (0.0, 0.0);
    for (i, wi) in w.iter().enumerate() {
        let ph = (i as f64) * 120.0_f64.to_radians();
        x += wi * ph.cos();
        y += wi * ph.sin();
    }
    y.atan2(x).to_degrees()
}

/// Per-phase weight of a decode pattern (+1 source, -1 sink, 0 float).
fn pattern_weights(p: [PhaseDrive; 3]) -> [f64; 3] {
    p.map(|d| match d {
        PhaseDrive::Pwm => 1.0,
        PhaseDrive::Sink => -1.0,
        PhaseDrive::Float => 0.0,
    })
}

#[test]
fn every_state_has_one_pwm_one_sink_one_float() {
    // The example's structural invariant, recovered as-is.
    for (i, st) in STATES.iter().enumerate() {
        let pwm = st.iter().filter(|d| **d == PhaseDrive::Pwm).count();
        let sink = st.iter().filter(|d| **d == PhaseDrive::Sink).count();
        let float = st.iter().filter(|d| **d == PhaseDrive::Float).count();
        assert_eq!((pwm, sink, float), (1, 1, 1), "state {i}");
    }
}

#[test]
fn sector_table_follows_the_front_end_forward_order() {
    // Ascending sector must follow the shared front-end's forward code order 1->3->2->6->4->5,
    // so a forward rotor advances the drive state by one per hall step.
    let fwd = [1u8, 3, 2, 6, 4, 5];
    for (want_sector, &code) in fwd.iter().enumerate() {
        assert_eq!(
            HALL_TO_SECTOR[code as usize] as usize, want_sector,
            "code {code}"
        );
    }
    assert_eq!(HALL_TO_SECTOR[0], 0xFF);
    assert_eq!(HALL_TO_SECTOR[7], 0xFF);
}

#[test]
fn drive_vector_advances_60_deg_per_forward_hall_step() {
    // The spec's consistency test: walking the front-end's forward sequence advances the decoded
    // drive vector by exactly +60 deg per step (f64 vector angles on the pattern weights).
    let decode = SixStep::new(Direction::Forward, 0);
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let angles: std::vec::Vec<f64> = fwd
        .iter()
        .map(|&c| vector_angle_deg(pattern_weights(decode.pattern(c).unwrap())))
        .collect();
    for i in 0..6 {
        let d = (angles[(i + 1) % 6] - angles[i]).rem_euclid(360.0);
        assert!((d - 60.0).abs() < 1e-9, "step {i}: delta {d}");
    }
    // Reverse decode flips the vector 180 deg (source/sink swap), same float phase.
    let rev = SixStep::new(Direction::Reverse, 0);
    for &c in &fwd {
        let f = decode.pattern(c).unwrap();
        let r = rev.pattern(c).unwrap();
        let df = (vector_angle_deg(pattern_weights(r)) - vector_angle_deg(pattern_weights(f)))
            .rem_euclid(360.0);
        assert!((df - 180.0).abs() < 1e-9);
        // Same float phase.
        for i in 0..3 {
            assert_eq!(f[i] == PhaseDrive::Float, r[i] == PhaseDrive::Float);
        }
    }
}

#[test]
fn align_offset_rotates_the_state_assignment() {
    let base = SixStep::new(Direction::Forward, 0);
    for off in 0..6u8 {
        let shifted = SixStep::new(Direction::Forward, off);
        for code in 1..=6u8 {
            let sector = HALL_TO_SECTOR[code as usize];
            let want = STATES[((sector + off) % 6) as usize];
            assert_eq!(shifted.pattern(code).unwrap(), want);
        }
        // offset is taken mod 6.
        assert_eq!(SixStep::new(Direction::Forward, off + 6).offset(), off);
    }
    let _ = base;
}

#[test]
fn sixstep_zero_demand_and_invalid_codes_coast() {
    let st = SixStepState::new(SixStep::new(Direction::Forward, 0));
    // Zero demand: all-float coast, regardless of code validity.
    for code in 0..8u8 {
        assert_eq!(sixstep_step(&st, code, 0), COAST, "code {code}");
    }
    // Invalid codes coast at any demand.
    assert_eq!(sixstep_step(&st, 0, 20_000), COAST);
    assert_eq!(sixstep_step(&st, 7, -20_000), COAST);
}

#[test]
fn sixstep_output_maps_pwm_sink_float_and_scales_duty() {
    let st = SixStepState::new(SixStep::new(Direction::Forward, 0));
    for code in 1..=6u8 {
        for &demand in &[500i32, 12_345, 32_767] {
            let out = sixstep_step(&st, code, demand);
            let duty = demand_to_duty(demand);
            // f64 reference for the scaling.
            let want = ((demand as f64) * 2250.0 / 32767.0).floor();
            assert_eq!(duty as f64, want);
            let pattern = st.decode.pattern(code).unwrap();
            let mut floats = 0;
            for (drive, phase) in pattern.iter().zip(out.phases.iter()) {
                match drive {
                    PhaseDrive::Pwm => assert_eq!(*phase, PhaseCmd::Drive(duty)),
                    PhaseDrive::Sink => assert_eq!(*phase, PhaseCmd::Drive(0)),
                    PhaseDrive::Float => {
                        assert_eq!(*phase, PhaseCmd::Float);
                        floats += 1;
                    }
                }
            }
            assert_eq!(floats, 1, "exactly one float per valid sector");
        }
    }
    // Saturation at ARR (a demand beyond full scale cannot leave the duty range).
    assert_eq!(demand_to_duty(i32::MAX), ARR);
    assert_eq!(demand_to_duty(-i32::MAX), ARR);
}

#[test]
fn sixstep_negative_demand_flips_the_effective_direction() {
    let fwd_cfg = SixStepState::new(SixStep::new(Direction::Forward, 2));
    let rev_cfg = SixStepState::new(SixStep::new(Direction::Reverse, 2));
    for code in 1..=6u8 {
        // Forward config driven negative == reverse config driven positive (same magnitude).
        assert_eq!(
            sixstep_step(&fwd_cfg, code, -9000),
            sixstep_step(&rev_cfg, code, 9000),
            "code {code}"
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Slice 3: the sine arm (recovered).
// ---------------------------------------------------------------------------------------------

/// Unwrap a PhaseCmd that must be driven (the sine arm drives all three phases).
fn duty_of(p: PhaseCmd) -> u16 {
    match p {
        PhaseCmd::Drive(d) => d,
        PhaseCmd::Float => panic!("sine phase must be driven"),
    }
}

#[test]
fn sine_zero_demand_is_all_mid_rail() {
    for theta in [0u16, 0x1234, 0x8000, 0xFFFF] {
        let out = sine::sine_step(theta, 0);
        for p in out.phases {
            assert_eq!(p, PhaseCmd::Drive(MID_RAIL));
        }
    }
}

#[test]
fn sine_matches_f64_reference_within_tolerance() {
    // duty = MID_RAIL + sign * round(sin_table(theta) * amp / 32767); reference: f64 sin. The
    // table+index quantization bounds the error at one table step through the amplitude scale.
    let demand = 20_000i32;
    let amp = sine::demand_to_amplitude(demand) as f64;
    let tol = 210.0 / 32767.0 * amp + 1.0;
    for step in 0..512u32 {
        let theta = (step * 128) as u16;
        let out = sine::sine_step(theta, demand);
        let rad = (theta as f64) / 65536.0 * core::f64::consts::TAU;
        let offs = [
            0.0,
            -((sine::PHASE_120 as f64) / 65536.0 * core::f64::consts::TAU),
            (sine::PHASE_120 as f64) / 65536.0 * core::f64::consts::TAU,
        ];
        for (i, off) in offs.iter().enumerate() {
            let want = MID_RAIL as f64 + (rad + off).sin() * amp;
            let got = duty_of(out.phases[i]) as f64;
            assert!(
                (got - want).abs() <= tol,
                "phase {i} at {theta:#06x}: got {got}, want {want:.1}"
            );
        }
    }
}

#[test]
fn sine_phases_are_120_degrees_apart() {
    // Phase B at theta equals phase A at theta - 120 deg; phase C likewise +120 deg.
    let demand = 15_000i32;
    for step in 0..256u32 {
        let theta = (step * 256) as u16;
        let out = sine::sine_step(theta, demand);
        let a_at_b = sine::sine_step(theta.wrapping_sub(sine::PHASE_120), demand);
        let a_at_c = sine::sine_step(theta.wrapping_add(sine::PHASE_120), demand);
        assert_eq!(out.phases[1], a_at_b.phases[0]);
        assert_eq!(out.phases[2], a_at_c.phases[0]);
    }
}

#[test]
fn sine_peak_scales_with_demand_and_stays_in_range() {
    let mut prev_peak = 0u16;
    for &demand in &[4000i32, 12_000, 24_000, 32_767] {
        let mut peak = 0u16;
        for step in 0..256u32 {
            let theta = (step * 256) as u16;
            let out = sine::sine_step(theta, demand);
            for p in out.phases {
                let d = duty_of(p);
                assert!(d <= ARR, "duty {d} out of range");
                peak = peak.max(d);
            }
        }
        assert!(peak > prev_peak, "peak must grow with demand");
        prev_peak = peak;
    }
    // Negative demand mirrors (same range bound).
    let out = sine::sine_step(0x2000, -32_767);
    for p in out.phases {
        assert!(duty_of(p) <= ARR);
    }
}

// ---------------------------------------------------------------------------------------------
// Slice 3: the mode model + dispatch.
// ---------------------------------------------------------------------------------------------

#[test]
fn commutation_method_default_and_byte_round_trip() {
    assert_eq!(CommutationMethod::default(), CommutationMethod::SixStep);
    for m in [
        CommutationMethod::SixStep,
        CommutationMethod::Sine,
        CommutationMethod::Foc,
    ] {
        assert_eq!(CommutationMethod::from_u8(m.to_u8()), m);
    }
    assert_eq!(CommutationMethod::SixStep.to_u8(), 0); // the MOTOR_METHOD field default
                                                       // Unknown bytes select the no-current-sensing default.
    for b in 3..=255u8 {
        assert_eq!(CommutationMethod::from_u8(b), CommutationMethod::SixStep);
    }
}

#[test]
fn dispatch_selects_the_expected_arm() {
    let cfg = SixStepState::new(SixStep::new(Direction::Forward, 0));
    let mut six = commutator(MethodState::SixStep(cfg));
    let mut sin = commutator(MethodState::Sine);
    assert_eq!(six.method(), CommutationMethod::SixStep);
    assert_eq!(sin.method(), CommutationMethod::Sine);

    let raw = lines(4);
    let out6 = six.step(raw, (0, 0), 10_000, true, false);
    let outs = sin.step(raw, (0, 0), 10_000, true, false);
    // Six-step floats exactly one phase; sine drives all three.
    assert_eq!(
        out6.phases
            .iter()
            .filter(|p| **p == PhaseCmd::Float)
            .count(),
        1
    );
    assert!(outs.phases.iter().all(|p| !matches!(p, PhaseCmd::Float)));
}

#[test]
fn open_loop_arms_ignore_the_currents_and_the_drive_gate() {
    // The currents and the drive gate are FOC-only: identical outputs for wildly different
    // currents and either gate, on both open-loop arms (the coast is the ISR's, on the enables).
    let cfg = SixStepState::new(SixStep::new(Direction::Forward, 0));
    for method in [MethodState::SixStep(cfg), MethodState::Sine] {
        let mut a = commutator(method);
        let mut b = commutator(method);
        for k in 0..300u32 {
            let raw = lines([1u8, 3, 2, 6, 4, 5][(k / 50) as usize % 6]);
            assert_eq!(
                a.step(raw, (0, 0), 9000, true, false),
                b.step(raw, (-32767, 0x1234), 9000, k % 2 == 0, false)
            );
        }
    }
}

#[test]
fn method_switch_resets_records_but_keeps_the_front_end() {
    // The spec's angle-continuity property through the REAL dispatch: commutator B runs six-step,
    // then switches to sine mid-run; reference commutator A runs sine the whole time on the same
    // input stream. Sine has no per-mode state, so if (and only if) the front-end survived the
    // switch, B's post-switch outputs are IDENTICAL to A's.
    let k = 50usize; // a slow walk: many periods per sector for the interpolator to run over
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let cfg = SixStepState::new(SixStep::new(Direction::Forward, 0));
    let mut a = commutator(MethodState::Sine);
    let mut b = commutator(MethodState::SixStep(cfg));
    let switch_at = k * 18;
    let mut idx = 0usize;
    for period in 0..(k * 24) {
        if period % k == 0 {
            idx += 1;
        }
        let raw = lines(fwd[idx % 6]);
        let out_a = a.step(raw, (0, 0), 11_000, true, false);
        if period == switch_at {
            b.switch_method(MethodState::Sine, SixStep::new(Direction::Forward, 0));
            assert_eq!(b.method(), CommutationMethod::Sine);
        }
        let out_b = b.step(raw, (0, 0), 11_000, true, false);
        if period >= switch_at {
            assert_eq!(out_a, out_b, "diverged at period {period}");
        }
    }
}

/// **The direction installed through the switch seam is live on the angle path, and the front-end
/// survives it** (`specs/integration.md`, "When a stored value takes effect": `motor.direction` is
/// in the value row, so an arm installs it beside the records). Same shape as the method-switch
/// test above: reference A runs sine REVERSE the whole way; B runs sine Forward, then has
/// `(Sine, Reverse)` installed mid-run on the same input stream. Before the install B is the
/// mirror of A (the direction is in force); from the install on B is IDENTICAL to A, which holds
/// only if the direction landed AND the interpolating front-end was left alone. A second install
/// of the direction already in force changes nothing.
#[test]
fn a_direction_installed_through_the_switch_seam_is_live_and_keeps_the_front_end() {
    let k = 50usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut a = commutator_with(MethodState::Sine, Direction::Reverse);
    let mut b = commutator(MethodState::Sine);
    let switch_at = k * 18;
    let resettle_at = k * 21;
    let mut idx = 0usize;
    for period in 0..(k * 24) {
        if period % k == 0 {
            idx += 1;
        }
        let raw = lines(fwd[idx % 6]);
        let out_a = a.step(raw, (0, 0), 11_000, true, false);
        if period == switch_at {
            b.switch_method(MethodState::Sine, SixStep::new(Direction::Reverse, 0));
        }
        if period == resettle_at {
            // The direction already in force, installed again: a no-op by construction.
            b.switch_method(MethodState::Sine, SixStep::new(Direction::Reverse, 0));
        }
        let out_b = b.step(raw, (0, 0), 11_000, true, false);
        assert_eq!(
            a.front().comm.angle,
            b.front().comm.angle,
            "period {period}: the published angle is the rotor's and the direction does not move it"
        );
        if period >= switch_at {
            assert_eq!(out_a, out_b, "diverged at period {period}");
        } else {
            for i in 0..3 {
                assert_eq!(
                    duty_of(out_a.phases[i]) + duty_of(out_b.phases[i]),
                    2 * MID_RAIL,
                    "period {period} phase {i}: before the install B drives A's mirror"
                );
            }
        }
    }
}

#[test]
fn every_arm_keeps_duties_on_the_arr_scale() {
    // The spec's duty-range property, across arms, demands, and codes (Drive counts <= ARR).
    let cfg = SixStepState::new(SixStep::new(Direction::Forward, 3));
    for method in [MethodState::SixStep(cfg), MethodState::Sine] {
        let mut c = commutator(method);
        for k in 0..600u32 {
            let raw = lines((k % 8) as u8);
            let demand = ((k as i32 * 7919) % 65535) - 32767;
            let out = c.step(raw, (0, 0), demand, true, false);
            for p in out.phases {
                if let PhaseCmd::Drive(d) = p {
                    assert!(d <= ARR, "duty {d} out of range");
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Slice 3: the EFeru second-oracle fixtures (behavior recordings, never tables/constants).
//
// Provenance: reference/efferu-hoverboard @ a0751d589fd43d8975eda3683fac21a44bbfe8fa, driven by
// the local harness reference/efferu-oracle/harness.c (gitignored, never committed; see its
// README), generated 2026-07-04. Model: BLDC_controller_step in COM (six-step) mode
// (z_ctrlTypSel=0), voltage control mode (z_ctrlModReq=1), diagnostics off, motor enabled,
// 200 settle steps per sector; speed runs 6000 steps at 25 steps/sector. Halls use THEIR
// convention (sum = hallA<<2 | hallB<<1 | hallC); the sequence below is their forward order
// (ascending commutation position). These are observed input->output vectors of the RUNNING
// model: behavior, not EFeru data tables.
// ---------------------------------------------------------------------------------------------

/// (their_hall_sum, [DC_phaA, DC_phaB, DC_phaC]) per forward sector, at each input amplitude.
const EFERU_COM_AMPS: [i16; 3] = [200, 500, 1000];
const EFERU_COM_FWD: [[(u8, [i16; 3]); 6]; 3] = [
    [
        (2, [-200, 200, 0]),
        (3, [-200, 0, 200]),
        (1, [0, -200, 200]),
        (5, [200, -200, 0]),
        (4, [200, 0, -200]),
        (6, [0, 200, -200]),
    ],
    [
        (2, [-500, 500, 0]),
        (3, [-500, 0, 500]),
        (1, [0, -500, 500]),
        (5, [500, -500, 0]),
        (4, [500, 0, -500]),
        (6, [0, 500, -500]),
    ],
    [
        (2, [-1000, 1000, 0]),
        (3, [-1000, 0, 1000]),
        (1, [0, -1000, 1000]),
        (5, [1000, -1000, 0]),
        (4, [1000, 0, -1000]),
        (6, [0, 1000, -1000]),
    ],
];
/// Observed n_mot after driving their forward / reverse hall sequences (sign = direction).
const EFERU_N_MOT_FWD: i32 = 426;
const EFERU_N_MOT_REV: i32 = -427;

#[test]
fn efferu_fixture_shares_the_sixstep_structure() {
    // Shared semantic: per sector exactly one positive (source), one negative (sink), and one
    // zero (idle) phase, in both designs.
    for per_amp in &EFERU_COM_FWD {
        for (sum, dc) in per_amp {
            let pos = dc.iter().filter(|v| **v > 0).count();
            let neg = dc.iter().filter(|v| **v < 0).count();
            let zero = dc.iter().filter(|v| **v == 0).count();
            assert_eq!((pos, neg, zero), (1, 1, 1), "their sum {sum}");
        }
    }
    // Ours: every valid sector decodes to one Pwm / one Sink / one Float (the structural test
    // above pins STATES; this pins it through the fixture's lens, per decoded code).
    let decode = SixStep::new(Direction::Forward, 0);
    for code in 1..=6u8 {
        let w = pattern_weights(decode.pattern(code).unwrap());
        assert_eq!(w.iter().filter(|v| **v > 0.0).count(), 1);
        assert_eq!(w.iter().filter(|v| **v < 0.0).count(), 1);
        assert_eq!(w.iter().filter(|v| **v == 0.0).count(), 1);
    }
}

#[test]
fn efferu_fixture_and_ours_rotate_the_drive_vector_in_the_same_sense() {
    // Shared semantic: advancing each design's own forward hall sequence advances the drive
    // voltage vector by +60 deg per step (same rotational sense). Their forward sequence is
    // pinned as "forward" by the recorded positive n_mot.
    const { assert!(EFERU_N_MOT_FWD > 0 && EFERU_N_MOT_REV < 0) };
    let theirs: std::vec::Vec<f64> = EFERU_COM_FWD[2]
        .iter()
        .map(|(_, dc)| vector_angle_deg([dc[0] as f64, dc[1] as f64, dc[2] as f64]))
        .collect();
    for i in 0..6 {
        let d = (theirs[(i + 1) % 6] - theirs[i]).rem_euclid(360.0);
        assert!((d - 60.0).abs() < 1e-9, "EFeru step {i}: delta {d}");
    }
    // Ours advances +60 deg per forward step too (re-checked here beside the fixture so the
    // parity is asserted in one place).
    let decode = SixStep::new(Direction::Forward, 0);
    let ours: std::vec::Vec<f64> = [1u8, 3, 2, 6, 4, 5]
        .iter()
        .map(|&c| vector_angle_deg(pattern_weights(decode.pattern(c).unwrap())))
        .collect();
    for i in 0..6 {
        let d = (ours[(i + 1) % 6] - ours[i]).rem_euclid(360.0);
        assert!((d - 60.0).abs() < 1e-9, "ours step {i}: delta {d}");
    }
}

#[test]
fn efferu_fixture_and_ours_scale_amplitude_monotonically() {
    // Shared semantic: a larger drive input produces a strictly larger phase amplitude.
    let mut prev = 0i16;
    for (i, per_amp) in EFERU_COM_FWD.iter().enumerate() {
        let peak = per_amp
            .iter()
            .flat_map(|(_, dc)| dc.iter().map(|v| v.abs()))
            .max()
            .unwrap();
        assert!(peak > prev, "EFeru amp {} peak {peak}", EFERU_COM_AMPS[i]);
        prev = peak;
    }
    let mut prev = 0u16;
    for demand in [6000i32, 15_000, 30_000] {
        let duty = demand_to_duty(demand);
        assert!(duty > prev);
        prev = duty;
    }
}

// ---------------------------------------------------------------------------------------------
// The FOC chain: the recovered transforms' check values in the torque-mode frame
// (`specs/foc-torque.md` section 3), the structural cal gate, and the arm.
// ---------------------------------------------------------------------------------------------

use super::foc::{
    calibrate_offset, circular_limit, clarke, current_from_adc, foc_step, iq_reference,
    offset_in_window, park_forward, park_inverse, rsh17, rsh18, svpwm, svpwm_sector, DutyOrder,
    FocState, PhaseOffsets, CAL_WINDOW_HI, CAL_WINDOW_LO, CIRC_GAIN, CIRC_THRESH, CLARKE_A,
    CLARKE_B, FOC_GAINS, FOC_GAINS_START, SVPWM_ALPHA, SVPWM_BETA, SVPWM_CENTER,
};

/// The bench-measured reference offset pair (the archived `MotorParams::default()` values,
/// carried as TEST DATA since the gated `PhaseOffsets` replaced that Default).
fn ref_offsets() -> PhaseOffsets {
    PhaseOffsets::try_new(0x7FB8, 0x7DAE).unwrap()
}

/// The reference full scale the fleet defaults derive (`firmware::motor::full_scale_counts`:
/// 10,000 mA at 455 counts per amp, less the 2,100 floor), for the tests that only need a record.
const FULL_SCALE: i16 = 2_450;

/// A fresh FOC record at [`FULL_SCALE`] in the direct output order.
fn foc_state() -> FocState {
    FocState::new(ref_offsets(), DutyOrder::DIRECT, FULL_SCALE)
}

#[test]
fn clarke_constants_exact() {
    assert_eq!(CLARKE_A, 0x49E6);
    assert_eq!(CLARKE_B, 0x93CC);
    // alpha passes straight through.
    assert_eq!(clarke(5000, 1234).alpha, 5000);
}

#[test]
fn park_forward_check_values() {
    // alpha=19660, beta=0 at theta=0 -> (iq, id) = (19658, 0): the vector along theta is torque.
    assert_eq!(park_forward(19660, 0, 0x0000), (19658, 0));
    // same input at theta=0x4000 (90 deg) -> (iq, id) = (0, 19658).
    assert_eq!(park_forward(19660, 0, 0x4000), (0, 19658));
}

#[test]
fn park_inverse_check_values() {
    // vq=19660, vd=0 at theta=0 -> (alpha, beta) = (19658, 0).
    assert_eq!(park_inverse(19660, 0, 0x0000), (19658, 0));
    // vq = vd = 32767 at theta=0x2000 (45 deg) -> alpha = -19341 (wrap of +46195), beta = -142.
    assert_eq!(park_inverse(32767, 32767, 0x2000), (-19341, -142));
}

#[test]
fn clarke_park_round_trip() {
    // Clarke then forward Park then inverse Park should reconstruct (alpha, beta) within rounding.
    let i_a = 12000i16;
    let i_b = -4000i16;
    let cl = clarke(i_a, i_b);
    for &theta in &[0x0000u16, 0x1234, 0x4000, 0x9ABC, 0xC000, 0xE321] {
        let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
        let (a2, b2) = park_inverse(iq, id, theta);
        // Round-trip: forward then inverse Park is a proper rotation, so it reconstructs (alpha,
        // beta) up to the Q15 rounding through two rotations. The reference sine table peaks at
        // 32766 (not 32768), so c^2 + s^2 is ~0.9956, a ~0.45% systematic shrink per round-trip;
        // the bound is set as a fraction of the input magnitude (the circular limiter accounts
        // for the modulation-depth loss downstream).
        let tol = (cl.alpha.unsigned_abs() as i32 + cl.beta.unsigned_abs() as i32) / 100 + 8;
        assert!(
            (a2 as i32 - cl.alpha as i32).abs() <= tol,
            "alpha round-trip theta=0x{theta:04X}: {a2} vs {}",
            cl.alpha
        );
        assert!(
            (b2 as i32 - cl.beta as i32).abs() <= tol,
            "beta round-trip theta=0x{theta:04X}: {b2} vs {}",
            cl.beta
        );
    }
}

#[test]
fn park_forward_tracks_f64_rotation() {
    // The forward Park against the f64 rotation matrix, within the table quantization through
    // the Q15 scale (the spec's f64 discipline for the real arithmetic).
    let (alpha, beta) = (11000i16, -7000i16);
    for step in 0..64u32 {
        let theta = (step * 1024) as u16;
        let (iq, id) = park_forward(alpha, beta, theta);
        let rad = (theta as f64) / 65536.0 * core::f64::consts::TAU;
        let want_iq = (alpha as f64 * rad.cos() - beta as f64 * rad.sin()) * (32766.0 / 32768.0);
        let want_id = (alpha as f64 * rad.sin() + beta as f64 * rad.cos()) * (32766.0 / 32768.0);
        let tol = 220.0; // one table step through the input magnitude
        assert!((iq as f64 - want_iq).abs() < tol, "iq at {theta:#06x}");
        assert!((id as f64 - want_id).abs() < tol, "id at {theta:#06x}");
    }
}

/// The phase currents, in counts, of a PHYSICAL current vector of amplitude `amp` at `deg`
/// degrees, in the frame [`vector_angle_deg`] reads drive vectors in (phase A at 0, B at 120, C at
/// 240): `i_k = amp * cos(deg - 120 k)`. Two of them, as the board senses two.
fn phase_currents_at(amp: f64, deg: f64) -> (i16, i16) {
    let at = |k: f64| (amp * (deg - 120.0 * k).to_radians().cos()).round() as i16;
    (at(0.0), at(1.0))
}

/// **The frame, read from the measurement side** (`specs/foc-torque.md` section 3): a physical
/// current vector ALONG `theta` reads `iq = |i|`, `id = 0`, and one a quarter turn away reads
/// `id = +-|i|`, `iq = 0`, at every angle, to the table's Q15 rounding.
///
/// Which quarter turn reads `id` POSITIVE: the one 90 degrees BEHIND `theta` (lower angle, the
/// direction the forward hall sequence does not advance), which is where the textbook d axis
/// sits, 90 degrees behind q. The spec's test list says "90 degrees ahead"; that is the other
/// sign, and its own check value (`park_forward(19660, 0, 0x4000)` reading `id` +19658, the stock
/// frame's vector at 0 seen from a `theta` of 90) is this one. The d loop's reference is zero, so
/// the sign is a label; what the loop needs is that `vd` drives the vector `id` reads positive,
/// which `the_d_command_drives_the_axis_id_reads_positive` pins.
#[test]
fn park_forward_reads_iq_along_theta_and_id_a_quarter_turn_behind() {
    let amp = 10_000.0;
    // Table + rounding through two Q15 products: 32766 / 32768 of the amplitude, plus a table step.
    let tol = amp * 0.01;
    for step in 0..96u32 {
        let theta = (step * 683) as u16;
        let deg = theta as f64 * 360.0 / 65536.0;
        let (ia, ib) = phase_currents_at(amp, deg);
        let cl = clarke(ia, ib);
        let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
        assert!(
            (iq as f64 - amp).abs() < tol,
            "along theta {theta:#06x}: iq {iq}"
        );
        assert!((id as f64).abs() < tol, "along theta {theta:#06x}: id {id}");

        let (ia, ib) = phase_currents_at(amp, deg - 90.0);
        let cl = clarke(ia, ib);
        let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
        assert!(
            (id as f64 - amp).abs() < tol,
            "behind {theta:#06x}: id {id}"
        );
        assert!((iq as f64).abs() < tol, "behind {theta:#06x}: iq {iq}");

        let (ia, ib) = phase_currents_at(amp, deg + 90.0);
        let cl = clarke(ia, ib);
        let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
        assert!((id as f64 + amp).abs() < tol, "ahead {theta:#06x}: id {id}");
        assert!((iq as f64).abs() < tol, "ahead {theta:#06x}: iq {iq}");
    }
}

/// The voltage side of the same frame: `vq` alone drives the stator vector AT `theta`, and `vd`
/// alone drives it 90 degrees behind, the direction a positive `id` is read in (above). So each
/// loop's command moves its own measurement the way it pushes, and neither loop is sign-inverted
/// against its measurement by the frame (the sense polarity is the bench's question, gate O2).
#[test]
fn the_d_command_drives_the_axis_id_reads_positive() {
    for step in 0..96u32 {
        let theta = (step * 683) as u16;
        let deg = theta as f64 * 360.0 / 65536.0;
        for (vq, vd, want) in [(12_000i16, 0i16, deg), (0, 12_000, deg - 90.0)] {
            let (alpha, beta) = park_inverse(vq, vd, theta);
            let out = MotorOutput {
                phases: DutyOrder::DIRECT
                    .apply(svpwm(alpha, beta))
                    .map(PhaseCmd::Drive),
            };
            let delta = (centered_vector_deg(out) - want).rem_euclid(360.0);
            let delta = if delta > 180.0 { delta - 360.0 } else { delta };
            assert!(
                delta.abs() < 0.6,
                "theta {theta:#06x}, (vq, vd) = ({vq}, {vd}): {delta} deg off"
            );
        }
    }
}

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
    let samples = [0x3FDCu16; 16];
    let off = calibrate_offset(&samples);
    assert_eq!(off, ((0x3FDCu16 >> 3) as u32 * 16) as u16);
    assert!(offset_in_window(off));
}

#[test]
fn cal_gate_is_structural() {
    // The refuses-run rule at the type level: an out-of-window offset cannot yield PhaseOffsets,
    // and FocState (hence MethodState::Foc) requires one. In-window pairs construct.
    assert!(PhaseOffsets::try_new(0x7FB8, 0x7DAE).is_some());
    assert!(PhaseOffsets::try_new(0x7531, 0x86C3).is_some()); // window edges (lo incl, hi-1)
    assert!(PhaseOffsets::try_new(0x7530, 0x7FB8).is_none()); // A below the window
    assert!(PhaseOffsets::try_new(0x7FB8, 0x86C4).is_none()); // B at the exclusive top
    assert!(PhaseOffsets::try_new(0, 0xFFFF).is_none());
    // The full path: a garbage calibration is refused end to end.
    let bad = calibrate_offset(&[0u16; 16]); // 0, far below the window
    assert!(PhaseOffsets::try_new(bad, bad).is_none());
    // A healthy calibration passes end to end.
    let good = calibrate_offset(&[0x3FDCu16; 16]);
    let offs = PhaseOffsets::try_new(good, good).unwrap();
    let _runnable = MethodState::Foc(FocState::new(offs, DutyOrder::DIRECT, FULL_SCALE));
}

#[test]
fn circular_limit_clamps() {
    assert_eq!(CIRC_THRESH, 0x3D75_9621);
    assert_eq!(CIRC_GAIN.len(), 67);
    // Inside the circle: pass through unchanged.
    let (vq, vd) = circular_limit(1000, 1000);
    assert_eq!((vq, vd), (1000, 1000));
    // Outside the circle: magnitude is reduced, ratio (angle) approximately preserved.
    let qin = 32767i16;
    let din = 32767i16;
    let sq_in = qin as i64 * qin as i64 + din as i64 * din as i64;
    assert!(sq_in as u32 > CIRC_THRESH);
    let (q2, d2) = circular_limit(qin, din);
    let sq_out = q2 as i64 * q2 as i64 + d2 as i64 * d2 as i64;
    assert!(
        (sq_out as u32) <= CIRC_THRESH + (CIRC_THRESH / 50),
        "limited magnitude {sq_out} not within ~the circle {CIRC_THRESH}"
    );
    // Equal vq,vd in -> still approximately equal out (ratio preserved).
    assert!((q2 as i32 - d2 as i32).abs() <= 2);
    // First and last gain-table entries.
    assert_eq!(CIRC_GAIN[0], 32494);
    assert_eq!(CIRC_GAIN[66], 22661);
}

#[test]
fn svpwm_constants_exact() {
    assert_eq!(SVPWM_BETA, 9000);
    assert_eq!(SVPWM_ALPHA, 0x3CE4);
    assert_eq!(SVPWM_CENTER, 0x465);
}

#[test]
fn svpwm_sector_selection() {
    assert_eq!(svpwm_sector(10000, 0), 6);
    assert_eq!(svpwm_sector(-10000, 0), 4);
    assert_eq!(svpwm_sector(0, 10000), 5);
    assert_eq!(svpwm_sector(0, -10000), 2);
    assert_eq!(svpwm_sector(10000, -1000), 1);
    assert_eq!(svpwm_sector(-10000, -1000), 3);
}

#[test]
fn svpwm_duties_centered_at_zero_vector() {
    // The zero vector (alpha=beta=0) gives all three compares at the half-period 0x465 (1125).
    let s = svpwm(0, 0);
    assert_eq!(s.base, 0x465);
    assert_eq!(s.c1, 0x465);
    assert_eq!(s.c2, 0x465);
}

#[test]
fn svpwm_duties_known_vector() {
    // A representative in-range vector: duties on the 0..2250 scale, sector matches selection.
    let alpha = 8000i16;
    let beta = 4000i16;
    let s = svpwm(alpha, beta);
    assert_eq!(s.sector, svpwm_sector(alpha, beta));
    for d in [s.base, s.c1, s.c2] {
        assert!(d <= 2250, "duty {d} out of 0..2250");
    }
}

#[test]
fn rsh_round_toward_zero() {
    assert_eq!(rsh17(-(1 << 17)), -1);
    assert_eq!(rsh17(-(1 << 17) + 1), 0); // truncates toward zero
    assert_eq!(rsh18(-(1 << 18)), -1);
    assert_eq!(rsh18((1 << 18) - 1), 0);
    assert_eq!(rsh17(1 << 17), 1);
}

#[test]
fn duty_order_permutes_the_svpwm_channels() {
    let s = svpwm(8000, 4000);
    assert_eq!(DutyOrder::DIRECT.apply(s), [s.base, s.c1, s.c2]);
}

// --- The arm: the gains, the reference scale and the drive gate ------------------------------

/// The live knob boots at the section-5 start values, word for word, and loads them back.
#[test]
fn the_gains_knob_starts_at_the_design_values() {
    assert_eq!(FOC_GAINS_START.kp, 1910);
    assert_eq!(FOC_GAINS_START.kp_divisor, 1024);
    assert_eq!(FOC_GAINS_START.ki, 57);
    assert_eq!(FOC_GAINS_START.ki_divisor, 1024);
    assert_eq!(FOC_GAINS_START.out_max, 32767);
    assert_eq!(FOC_GAINS_START.int_max, 32767 * 1024);
    // No test writes the knob (it is one static every test thread shares), so it still reads the
    // start values here.
    assert_eq!(FOC_GAINS.load(), FOC_GAINS_START);
    // Six words, in `PiGains`' order: the bench addresses each at a fixed offset.
    assert_eq!(core::mem::size_of_val(&FOC_GAINS), 24);
    assert_eq!(core::mem::size_of::<base::pi::PiGains>(), 24);
}

/// **The demand-to-reference scale** (`specs/foc-torque.md` section 4): a full-scale word is the
/// full-scale reference with the word's sign, a word past full scale is clamped to it, zero is
/// zero, the map is monotone, and it rounds toward zero.
#[test]
fn the_reference_is_the_demand_scaled_to_the_full_scale() {
    for fs in [FULL_SCALE, 1, 0, 5_999, i16::MAX] {
        assert_eq!(iq_reference(28_500, fs), fs, "full scale {fs}");
        assert_eq!(iq_reference(-28_500, fs), -fs, "full scale {fs}");
        assert_eq!(iq_reference(40_000, fs), fs, "clamped, full scale {fs}");
        assert_eq!(iq_reference(-40_000, fs), -fs, "clamped, full scale {fs}");
        assert_eq!(iq_reference(i32::MAX, fs), fs);
        assert_eq!(iq_reference(i32::MIN, fs), -fs);
        assert_eq!(iq_reference(0, fs), 0);
        let mut prev = iq_reference(-40_000, fs);
        for demand in (-40_000..=40_000).step_by(7) {
            let r = iq_reference(demand, fs);
            assert!(r >= prev, "monotone at demand {demand}, full scale {fs}");
            prev = r;
            // Toward zero: the f64 product truncated.
            let want = (demand.clamp(-28_500, 28_500) as f64 * fs as f64 / 28_500.0).trunc();
            assert_eq!(r as f64, want, "demand {demand}, full scale {fs}");
        }
    }
    // A word just under one count's worth rounds to zero from both sides.
    assert_eq!(iq_reference(11, FULL_SCALE), 0);
    assert_eq!(iq_reference(-11, FULL_SCALE), 0);
    assert_eq!(iq_reference(12, FULL_SCALE), 1);
    assert_eq!(iq_reference(-12, FULL_SCALE), -1);
}

/// **A floated period outputs the zero vector and zeroes both accumulators, whatever the state**,
/// and latches a zero reference; the measurement is still summed (the observation reads a
/// disarmed board's currents).
#[test]
fn a_floated_period_is_the_zero_vector_with_both_accumulators_zero() {
    let mut st = foc_state();
    // Wind both loops: a reference the zero measurement never meets, and a d current.
    for _ in 0..200 {
        foc_step(&mut st, 0x2345, (800, -300), 20_000, true, false);
    }
    assert_ne!(st.q_acc(), 0);
    assert_ne!(st.d_acc(), 0);
    assert_ne!(st.iq_ref(), 0);
    let before = st.sums();
    for (theta, currents, demand) in [
        (0x2345u16, (800i16, -300i16), 20_000),
        (0x0000, (-32767, 32767), -28_500),
        (0xBFFF, (0, 0), 0),
        (0x9554, (12_000, 9_000), i32::MAX),
    ] {
        let out = foc_step(&mut st, theta, currents, demand, false, false);
        assert_eq!(
            out.phases,
            [PhaseCmd::Drive(MID_RAIL); 3],
            "theta {theta:#06x}"
        );
        assert_eq!((st.q_acc(), st.d_acc()), (0, 0), "theta {theta:#06x}");
        assert_eq!(st.iq_ref(), 0);
    }
    let after = st.sums();
    assert_ne!(
        after.iq, before.iq,
        "a floated period still sums the measured current"
    );
    assert_eq!(after.vq, before.vq, "and sums a zero voltage");
}

// --- Observation: the window the period ISR closes (`specs/foc-torque.md` section 7) ----------

/// **`take_window` returns `sum >> 6` and resets**: each mean is its sum shifted by the window
/// length, arithmetic (so a negative mean rounds toward minus infinity, by at most one count), and
/// the sums restart, so a second close of the same window reads zero means.
#[test]
fn take_window_returns_the_shifted_sums_and_resets_them() {
    assert_eq!(1 << FOC_WINDOW_SHIFT, 64);
    let mut st = foc_state();
    // A driven window over moving currents and angles, so all three sums are nonzero and differ.
    for period in 0..64i16 {
        let theta = (period as u16).wrapping_mul(0x0411);
        foc_step(
            &mut st,
            theta,
            (period * 37 - 900, 410 - period * 11),
            6_000,
            true,
            false,
        );
    }
    let sums = st.sums();
    assert!(sums.iq != 0 && sums.id != 0 && sums.vq != 0, "{sums:?}");
    let w = st.take_window();
    assert_eq!(
        (w.iq, w.id, w.vq),
        (
            (sums.iq >> 6) as i16,
            (sums.id >> 6) as i16,
            (sums.vq >> 6) as i16
        )
    );
    assert_eq!(st.sums(), FocSums::default(), "the sums restarted");
    let again = st.take_window();
    assert_eq!((again.iq, again.id, again.vq), (0, 0, 0));
    assert_eq!(again.iq_ref, w.iq_ref, "the latch is not a sum: it stays");

    // The rounding: one period of -1 count in a window is a sum of -1, and its mean is -1, not 0.
    // (An `ia` of -2 at theta 0 is an `iq` of -1 through the table's 32,766; the input filter is
    // settled on it first, since one period of -1 through a fresh stage rounds to 0.)
    let mut st = foc_state();
    for _ in 0..200 {
        foc_step(&mut st, 0, (-2, 0), 0, false, false);
    }
    let _ = st.take_window();
    foc_step(&mut st, 0, (-2, 0), 0, false, false);
    assert_eq!(st.sums().iq, -1);
    assert_eq!(st.take_window().iq, -1, "toward minus infinity");
}

/// **The latched reference is the last period's**: the window's `iq_ref` is the reference of the
/// period that closed it, not a mean, and a floated last period reads 0.
#[test]
fn the_window_reference_is_the_closing_periods() {
    let mut st = foc_state();
    for demand in [-28_500, 4_000, 19_000] {
        foc_step(&mut st, 0x1000, (0, 0), demand, true, false);
    }
    assert_eq!(st.take_window().iq_ref, iq_reference(19_000, FULL_SCALE));
    foc_step(&mut st, 0x1000, (0, 0), 19_000, true, false);
    foc_step(&mut st, 0x1000, (0, 0), 19_000, false, false);
    assert_eq!(st.take_window().iq_ref, 0, "the closing period floated");
}

/// **64 periods of a constant `iq` read that constant**: a static current vector at a fixed angle
/// on floated periods (so `vq` is exactly zero), and the window's means are that period's own
/// `iq`, `id` and `vq`, which is what makes a 10 Hz read of the word mean what one period meant.
/// The input filter (section 5) is settled on the constant first: its 16-period memory is what a
/// window opened on a fresh stage would read short by.
#[test]
fn a_constant_window_reads_its_constant() {
    let theta = 0x2345u16;
    let (ia, ib) = (1_200i16, -700i16);
    let cl = clarke(ia, ib);
    let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
    assert!(iq != 0 && id != 0);
    let mut st = foc_state();
    for _ in 0..200 {
        foc_step(&mut st, theta, (ia, ib), 0, false, false);
    }
    let _ = st.take_window();
    for _ in 0..64 {
        foc_step(&mut st, theta, (ia, ib), 0, false, false);
    }
    assert_eq!(
        st.take_window(),
        FocWindow {
            iq,
            id,
            vq: 0,
            iq_ref: 0
        }
    );
}

/// **Zero demand is zero CURRENT, driven**: with `drive` the zero vector comes out only because
/// the measured current is zero; a measured current with zero demand is regulated against
/// (`specs/foc-torque.md` section 2). Not driven, the zero vector comes out unconditionally.
#[test]
fn foc_step_smoke_and_zero_demand_is_mid_rail() {
    let mut st = foc_state();
    let out = foc_step(&mut st, BASE_ANGLE[1], (0, 0), 0, true, false);
    assert_eq!(out.phases, [PhaseCmd::Drive(MID_RAIL); 3]);
    // The same zero demand against a measured current pushes back on it: every phase driven, and
    // the vector is not the zero one.
    let mut st = foc_state();
    let out = foc_step(&mut st, BASE_ANGLE[1], (900, -300), 0, true, false);
    assert!(out.phases.iter().all(|p| matches!(p, PhaseCmd::Drive(_))));
    assert_ne!(out.phases, [PhaseCmd::Drive(MID_RAIL); 3]);
    // Not driven: mid-rail whatever the current and the demand.
    let mut st = foc_state();
    let out = foc_step(&mut st, BASE_ANGLE[1], (900, -300), 20_000, false, false);
    assert_eq!(out.phases, [PhaseCmd::Drive(MID_RAIL); 3]);
}

/// **The first driven period after a floated one starts from zero history**: its output is
/// exactly the step a zeroed accumulator gives, `e * kp / kp_divisor` plus ONE period's integral
/// `e * ki / ki_divisor`, and each accumulator holds exactly one period of `e * ki`.
///
/// The spec's test list says "the P term alone"; that is not what its own `pi_step` (section 5:
/// the accumulator is updated before the output reads it) produces, and this pins the step as
/// specified. At the start gains and an error of 1,000 counts that is 3,906 + 116.
#[test]
fn the_first_driven_period_after_a_floated_one_carries_no_history() {
    let mut st = foc_state();
    for _ in 0..500 {
        foc_step(&mut st, 0x1556, (0, 0), 28_500, true, false);
    }
    assert!(st.q_acc() > 1_000_000, "wound: {}", st.q_acc());
    foc_step(&mut st, 0x1556, (0, 0), 28_500, false, false);
    let theta = 0x1556;
    let (ia, ib) = (-640i16, 410i16);
    let demand = 11_632;
    let out = foc_step(&mut st, theta, (ia, ib), demand, true, false);
    let cl = clarke(ia, ib);
    let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
    // The PIs see the filtered pair: the stages were at zero through the 500 zero-current
    // periods, so the first driven period's input is one shift of the measured pair.
    let shift = FOC_FILTER_SHIFT.load(core::sync::atomic::Ordering::Relaxed);
    let iq = Lowpass::new().step(iq, shift);
    let id = Lowpass::new().step(id, shift);
    let g = FOC_GAINS_START;
    let eq = iq_reference(demand, FULL_SCALE) as i32 - iq as i32;
    let ed = -(id as i32);
    let fresh = |e: i32| e * g.kp / g.kp_divisor + e * g.ki / g.ki_divisor;
    let (vq, vd) = circular_limit(fresh(eq) as i16, fresh(ed) as i16);
    let (alpha, beta) = park_inverse(vq, vd, theta);
    assert_eq!(
        out.phases,
        DutyOrder::DIRECT
            .apply(svpwm(alpha, beta))
            .map(PhaseCmd::Drive)
    );
    assert_eq!(st.q_acc(), eq * g.ki);
    assert_eq!(st.d_acc(), ed * g.ki);
    // The worked figure from the docs: an error of 1,000 at the start gains.
    assert_eq!(fresh(1_000), 1_865 + 55);
}

#[test]
fn foc_duties_stay_on_the_arr_scale_through_dispatch() {
    // The spec's duty-range property through the full FOC dispatch, over a realistic rotating
    // stream with nonzero currents, full-scale demands both ways and the gate toggling (the
    // circular limit + SVPWM keep the compares on the 0..2250 scale).
    let k = 50usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut c = commutator(MethodState::Foc(foc_state()));
    let mut idx = 0usize;
    for period in 0..(k * 24) {
        if period % k == 0 {
            idx += 1;
        }
        let ripple = ((period as i32 % 41) - 20) * 16; // small current ripple
        let demand = if period % 300 < 150 { 28_500 } else { -40_000 };
        let drive = period % 97 != 0;
        let out = c.step(
            lines(fwd[idx % 6]),
            (ripple as i16, -ripple as i16),
            demand,
            drive,
            false,
        );
        for p in out.phases {
            match p {
                PhaseCmd::Drive(d) => assert!(d <= ARR, "duty {d} out of range at {period}"),
                PhaseCmd::Float => panic!("FOC never floats"),
            }
        }
    }
}

#[test]
fn dispatch_foc_arm_is_foc_step_at_the_aligned_angle() {
    // The dispatch's FOC arm equals foc_step over an identical front-end stream at the aligned
    // angle, with the currents and the gate passed straight through (the arm adds nothing and
    // loses nothing through Commutator::step).
    let k = 50usize;
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut via_dispatch = commutator(MethodState::Foc(foc_state()));
    let mut fe = front_end();
    let mut st = foc_state();
    let mut idx = 0usize;
    for period in 0..(k * 12) {
        if period % k == 0 {
            idx += 1;
        }
        let raw = lines(fwd[idx % 6]);
        let currents = ((period as i16 % 37) * 20 - 360, 150);
        let drive = period % 61 > 3;
        let a = via_dispatch.step(raw, currents, 9_000, drive, false);
        let rotor = fe.step(raw);
        let aligned = rotor
            .angle
            .wrapping_add(angle_path_offset(SixStep::new(Direction::Forward, 0)));
        let b = foc_step(&mut st, aligned, currents, 9_000, drive, false);
        assert_eq!(a, b, "diverged at period {period}");
    }
    // The window the dispatch closes is the arm's own.
    assert_eq!(via_dispatch.take_foc_window(), Some(st.take_window()));
}

/// The dispatch closes the FOC window only when FOC runs: six-step and sine keep no sums and
/// answer `None`, which the period ISR publishes as zeros (`specs/foc-torque.md` section 7).
#[test]
fn only_the_foc_arm_has_a_window_to_close() {
    let mut six = commutator(MethodState::SixStep(SixStepState::new(SixStep::new(
        Direction::Forward,
        0,
    ))));
    let mut sine = commutator(MethodState::Sine);
    let mut foc = commutator(MethodState::Foc(foc_state()));
    for c in [&mut six, &mut sine, &mut foc] {
        c.step(lines(1), (900, -300), 9_000, true, false);
    }
    assert_eq!(six.take_foc_window(), None);
    assert_eq!(sine.take_foc_window(), None);
    assert!(foc.take_foc_window().is_some_and(|w| w.iq_ref != 0));
}

/// **A held period regulates on the last fresh rotor-frame pair** (`specs/foc-torque.md` section
/// 14): with `hold` set, whatever currents are handed in, the PIs see the `(iq, id)` of the last
/// period that had both phases fresh, the sums take that pair, and everything else (the
/// reference, the drive gate, the limit, the duties) runs as on any period. The first fresh
/// period after stores anew.
#[test]
fn a_held_period_regulates_on_the_last_fresh_rotor_frame_pair() {
    let theta = BASE_ANGLE[3];
    let fresh = (800i16, -300i16);
    let cl = clarke(fresh.0, fresh.1);
    let dq = park_forward(cl.alpha, cl.beta, theta);
    let mut st = foc_state();
    foc_step(&mut st, theta, fresh, 9_000, true, false);
    assert_eq!(st.held_dq(), dq);
    // The same state stepped once more on the FRESH pair is the oracle for the held step.
    let mut oracle = st;
    let want = foc_step(&mut oracle, theta, fresh, 9_000, true, false);
    // The held step is handed a rail reading on A, the sort of sample the hold exists for.
    let got = foc_step(&mut st, theta, (32_767, fresh.1), 9_000, true, true);
    assert_eq!(
        got, want,
        "the held period commands what the fresh pair would"
    );
    assert_eq!(st.q_acc(), oracle.q_acc());
    assert_eq!(st.d_acc(), oracle.d_acc());
    assert_eq!(st.sums(), oracle.sums(), "the sums took the held pair");
    assert_eq!(st.iq_ref(), oracle.iq_ref());
    assert_eq!(st.held_dq(), dq, "a held period stores nothing");
    assert!(got.phases.iter().all(|p| matches!(p, PhaseCmd::Drive(_))));
    // A fresh period after the hold stores its own pair.
    let next = (400i16, 100i16);
    let cl = clarke(next.0, next.1);
    foc_step(&mut st, theta, next, 9_000, true, false);
    assert_eq!(st.held_dq(), park_forward(cl.alpha, cl.beta, theta));
    // A held period with `drive` clear is still the floated posture: zeroed accumulators and the
    // zero vector, and the held pair stands.
    let held = st.held_dq();
    let out = foc_step(&mut st, theta, (32_767, 32_767), 9_000, false, true);
    assert_eq!(out.phases, [PhaseCmd::Drive(MID_RAIL); 3]);
    assert_eq!((st.q_acc(), st.d_acc()), (0, 0));
    assert_eq!(st.held_dq(), held);
}

// --- The arm in closed loop: a plant model (`specs/foc-torque.md` section 9) -----------------

/// The phase resistance, ohms (`specs/foc-torque.md` section 5, the meter's figure of
/// 2026-10-09).
const PLANT_R: f64 = 0.24;
/// The phase inductance, henries (0.5 mH; not measured, the chop-physics figure).
const PLANT_L: f64 = 0.5e-3;
/// The bench supply, volts (the 2026-10-27 record's).
const PLANT_VBUS: f64 = 31.0;
/// The sense chain's scale on reading (a) of `specs/foc-torque.md` section 5, current counts per
/// amp: the staged 455 is about three times too high, and 155 is what puts the meter's resistance
/// on the measured line (`K_dc` 0.351 current counts per voltage count at 31 V).
const PLANT_CAL_A: f64 = 155.0;
/// Reading (b): the staged cal is right and the resistance in this frame is 0.71 ohm.
const PLANT_CAL_B: f64 = 455.0;
const PLANT_R_B: f64 = 0.71;
/// One PWM period, seconds.
const PLANT_T: f64 = 1.0 / 16_000.0;

/// A wye-connected three-phase R-L load with an optional back-EMF, driven by the arm's duties and
/// read by the arm's two sensed phases.
///
/// - The three duties are lowered to phase voltages: each leg's average is `duty / ARR * V_bus`,
///   and the winding sees each leg less the floating neutral (their mean), so a common offset (the
///   SVPWM's zero-sequence injection) drives nothing.
/// - `L di/dt = v - R i - e` per phase, forward Euler at the period (`T / tau` is 1/33).
/// - The ONE-PERIOD computation delay is modelled, because it is real: the ISR samples this
///   period's current and its duties take effect from the next period (the compares are
///   preloaded), so `step` integrates the period over the duties the PREVIOUS call returned.
/// - `sense_sign` +1 is the expectation of `specs/foc-torque.md` section 3 (positive is current
///   into the winding, the same sign as the voltage driving it); -1 is the inverted chain gate O2
///   exists to catch.
struct Plant {
    i: [f64; 3],
    emf: [f64; 3],
    applied: [u16; 3],
    sense_sign: f64,
    /// Phase resistance, ohms.
    r: f64,
    /// Current counts per amp.
    cal: f64,
}

impl Plant {
    /// The design's plant: reading (a) of section 5.
    fn new(sense_sign: f64) -> Self {
        Self::with(sense_sign, PLANT_R, PLANT_CAL_A)
    }

    /// The other reading of the measured line: the staged cal right, 0.71 ohm.
    fn reading_b(sense_sign: f64) -> Self {
        Self::with(sense_sign, PLANT_R_B, PLANT_CAL_B)
    }

    fn with(sense_sign: f64, r: f64, cal: f64) -> Self {
        Self {
            i: [0.0; 3],
            emf: [0.0; 3],
            applied: [MID_RAIL; 3],
            sense_sign,
            r,
            cal,
        }
    }

    /// The DC plant gain in the loop's units, current counts per voltage count: what the measured
    /// line's slope is the inverse of.
    fn k_dc(&self) -> f64 {
        volts_per_voltage_count() / self.r * self.cal
    }

    /// The two sensed phase currents, in counts, as `current_from_adc` would condition them.
    fn sense(&self) -> (i16, i16) {
        let count = |amps: f64| {
            (self.sense_sign * amps * self.cal)
                .round()
                .clamp(-32767.0, 32767.0) as i16
        };
        (count(self.i[0]), count(self.i[1]))
    }

    /// One period over the duties applied during it, then latch the arm's new duties for the next.
    fn step(&mut self, next: MotorOutput) {
        let v = self.applied.map(|d| d as f64 / ARR as f64 * PLANT_VBUS);
        let neutral = (v[0] + v[1] + v[2]) / 3.0;
        for ((i, v), e) in self.i.iter_mut().zip(v).zip(self.emf) {
            *i += PLANT_T / PLANT_L * (v - neutral - e - self.r * *i);
        }
        self.applied = next.phases.map(duty_of);
    }
}

/// What a closed-loop run measured, per period, in the arm's own counts: the `iq` and `id` it
/// read and the `vq` it commanded (after the circular limit, off its observation sum).
struct LoopTrace {
    iq: Vec<i32>,
    id: Vec<i32>,
    vq: Vec<i32>,
    q_acc: Vec<i32>,
}

/// Run the arm against the plant at a static rotor angle `theta` for `periods`, at a demand that
/// asks for `iq_ref` counts (the record's full scale is 28,500, so the demand IS the reference).
fn run_loop(theta: u16, iq_ref: i32, plant: &mut Plant, periods: usize) -> LoopTrace {
    let mut st = FocState::new(ref_offsets(), DutyOrder::DIRECT, 28_500);
    let mut trace = LoopTrace {
        iq: Vec::new(),
        id: Vec::new(),
        vq: Vec::new(),
        q_acc: Vec::new(),
    };
    for _ in 0..periods {
        let (ia, ib) = plant.sense();
        let cl = clarke(ia, ib);
        let (iq, id) = park_forward(cl.alpha, cl.beta, theta);
        let vq_before = st.sums().vq;
        let out = foc_step(&mut st, theta, (ia, ib), iq_ref, true, false);
        trace.iq.push(iq as i32);
        trace.id.push(id as i32);
        trace.vq.push(st.sums().vq.wrapping_sub(vq_before));
        trace.q_acc.push(st.q_acc());
        plant.step(out);
    }
    trace
}

/// One voltage count, in volts of phase-to-neutral amplitude, at the plant's bus: the SVPWM
/// arithmetic (alpha 32,767 is a phase-to-neutral peak of 0.577 `V_bus`; `specs/foc-torque.md`
/// section 5's unit table), read off the arithmetic itself rather than quoted.
fn volts_per_voltage_count() -> f64 {
    let s = svpwm(32767, 0);
    let d = [s.base, s.c1, s.c2].map(|d| d as f64);
    let mean = (d[0] + d[1] + d[2]) / 3.0;
    let peak = d.iter().map(|x| (x - mean).abs()).fold(0.0, f64::max);
    peak / ARR as f64 * PLANT_VBUS / 32767.0
}

/// **The q loop tracks a reference step at the start gains** (`specs/foc-torque.md` section 9,
/// re-pinned for round 2 on the measured plant): an `iq` step of 1,000 counts at each of the six
/// hall anchors, rotor static, with the sample-to-duty delay, on both readings of the measured
/// line (section 5): (a) the design's plant, 155 counts per amp on the meter's 0.24 ohm, where the
/// PI's zero cancels the plant's pole and the loop is the input filter's second-order pair at
/// 50 Hz, damping 0.89; (b) the staged 455 on 0.71 ohm, where the zero sits below the pole and the
/// loop crosses near 60 Hz with about 90 degrees of margin.
///
/// What holds, measured (printed with `--nocapture`): on (a) 98 % at period 113 (7 ms; the
/// second-order pair rises faster than a first-order loop of the same crossover, whose 98 % point
/// would be 13 ms) with 0.2 % overshoot; on (b) 98 % at period 212 with 0.4 %; on both `|id|` at
/// most 2 counts and `vq` within 1.2 % of the plant's `R * iq`. The assertions are those figures
/// with room (130 and 240 periods, 2 % overshoot, 2 % and 3 %). The 2026-10-26 pin (98 % within
/// 36 periods at a 300 Hz design) is superseded by section 5's round-2 bandwidth, not by a
/// tolerance.
#[test]
fn the_q_loop_tracks_a_step_at_the_start_gains() {
    let step = 1_000;
    let k_v = volts_per_voltage_count();
    for (reading, make, rise_max, overshoot_max) in [
        ("a", Plant::new as fn(f64) -> Plant, 130usize, 2i32),
        ("b", Plant::reading_b as fn(f64) -> Plant, 240, 2),
    ] {
        for code in 1..=6u8 {
            let theta = BASE_ANGLE[code as usize];
            let mut plant = make(1.0);
            let t = run_loop(theta, step, &mut plant, 1_000);
            let rise =
                t.iq.iter()
                    .position(|&iq| iq * 100 >= step * 98)
                    .expect("never reached 98 %");
            let at64 = t.iq[64];
            let peak = *t.iq.iter().max().unwrap();
            let id_max = t.id.iter().map(|x| x.abs()).max().unwrap();
            // Settled: the last 300 periods.
            let tail = &t.vq[700..];
            let vq_mean = tail.iter().sum::<i32>() as f64 / tail.len() as f64;
            let iq_tail = &t.iq[700..];
            let iq_mean = iq_tail.iter().sum::<i32>() as f64 / iq_tail.len() as f64;
            // R * iq in voltage counts: iq counts / cal = amps (Clarke is amplitude-invariant, and
            // the table's 32766 / 32768 is inside the tolerance), times R, over one voltage
            // count's volts.
            let vq_want = iq_mean / plant.cal * plant.r / k_v;
            std::println!(
                "reading ({reading}), K_dc {:.3}, code {code}: 98 % at period {rise}, iq[64] \
                 {at64}, peak {peak} ({:.1} % overshoot), |id| max {id_max}, settled iq \
                 {iq_mean:.1}, vq {vq_mean:.1} against R*iq {vq_want:.1} ({:+.2} %)",
                plant.k_dc(),
                (peak - step) as f64 * 100.0 / step as f64,
                (vq_mean / vq_want - 1.0) * 100.0
            );
            assert!(
                rise <= rise_max,
                "({reading}) code {code}: 98 % at period {rise}, past {rise_max}"
            );
            assert!(
                peak * 100 < step * (100 + overshoot_max),
                "({reading}) code {code}: overshoot to {peak} on a {step} step"
            );
            assert!(
                id_max * 100 <= step * 2,
                "({reading}) code {code}: id reached {id_max} on a {step} step"
            );
            assert!(
                (iq_mean - step as f64).abs() <= 2.0,
                "({reading}) code {code}: settled at {iq_mean}"
            );
            assert!(
                (vq_mean / vq_want - 1.0).abs() < 0.03,
                "({reading}) code {code}: vq settled at {vq_mean}, R * iq is {vq_want}"
            );
        }
    }
    // The design's plant is the measured line: its DC gain is O4's slope inverted, within 2 %.
    assert!((Plant::new(1.0).k_dc() / 0.351 - 1.0).abs() < 0.02);
    assert!((Plant::reading_b(1.0).k_dc() / 0.351 - 1.0).abs() < 0.02);
}

/// **The input filter** (`specs/foc-torque.md` section 5): `Lowpass` settles within one count of
/// a constant input from any state, reaches 63 % of a step at 16 periods at shift 4 (one time
/// constant) and half of it at about 11, shift 0 is the identity, a shift of 40 is defined (the
/// low five bits, so it is shift 8) and the `i16` extremes stay inside the state. And the claim
/// the design rests on: a white sequence of about 80 counts sd comes out at under a quarter of
/// that at shift 4 (the design figure is 0.18).
#[test]
fn the_input_filter_settles_steps_and_cuts_the_noise_as_designed() {
    let mut lp = Lowpass::new();
    for _ in 0..200 {
        lp.step(1_000, 4);
    }
    assert_eq!(lp.step(1_000, 4), 1_000);
    for _ in 0..200 {
        lp.step(-1_000, 4);
    }
    assert_eq!(lp.step(-1_000, 4), -1_000);
    // The step response at shift 4: 63 % at 16 periods, 50 % at about 11.
    let mut lp = Lowpass::new();
    let trace: Vec<i16> = (0..64).map(|_| lp.step(1_000, 4)).collect();
    assert!(
        (620..=650).contains(&trace[15]),
        "at 16 periods: {}",
        trace[15]
    );
    let half = trace.iter().position(|&y| y >= 500).unwrap() + 1;
    assert!((10..=12).contains(&half), "50 % at period {half}");
    // Shift 0 is the identity, from any state.
    let mut lp = Lowpass::new();
    lp.step(12_345, 4);
    assert_eq!(lp.step(-777, 0), -777);
    assert_eq!(lp.step(32_767, 0), 32_767);
    assert_eq!(lp.step(-32_768, 0), -32_768);
    // A poked shift of 40 is a shift of 8: defined, and slower.
    let mut lp = Lowpass::new();
    assert_eq!(lp.step(25_600, 40), 100);
    // The extremes, for thousands of periods: the state never leaves the input's range.
    let mut lp = Lowpass::new();
    for k in 0..5_000 {
        let x = if k % 2 == 0 { i16::MAX } else { i16::MIN };
        let y = lp.step(x, 1);
        assert!((-32_768..=32_767).contains(&(y as i32)));
    }
    // The noise: a fixed linear-congruential sequence, uniform over +-139 (sd 80), at shift 4
    // and at shift 0, the ratio of the output sds.
    let mut seed: u32 = 0x1234_5678;
    let mut noise = || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        ((seed >> 8) % 279) as i16 - 139
    };
    let samples: Vec<i16> = (0..20_000).map(|_| noise()).collect();
    let sd = |v: &[i16]| {
        let n = v.len() as f64;
        let mean = v.iter().map(|&x| x as f64).sum::<f64>() / n;
        (v.iter().map(|&x| (x as f64 - mean).powi(2)).sum::<f64>() / n).sqrt()
    };
    let raw_sd = sd(&samples);
    assert!((75.0..=85.0).contains(&raw_sd), "raw sd {raw_sd}");
    let mut lp = Lowpass::new();
    let filtered: Vec<i16> = samples.iter().map(|&x| lp.step(x, 4)).collect();
    let ratio = sd(&filtered[1_000..]) / raw_sd;
    std::println!(
        "filter at shift 4: noise sd {raw_sd:.1} -> {:.1} ({ratio:.3})",
        sd(&filtered[1_000..])
    );
    assert!((0.15..0.25).contains(&ratio), "noise ratio {ratio}");
}

/// **The loop at rest, under the sense noise, commands the voltage jitter the design says it
/// does** (`specs/foc-torque.md` section 5): the design's plant with a static rotor at a zero
/// reference, both sensed currents carrying the 80-count white sequence of the filter test on top
/// of the plant's own, and the per-period scatter of the commanded `vq` read off the trace. The
/// P term alone is about 27 counts (`kp` 1.87 on 14 counts); the I term's walk on the filtered
/// noise and the loop's own response bring the model to about 51, and the assertion is that
/// figure with room, under 65 counts. The same model at the 2026-10-26 gains without the filter
/// reads about 320 (the hiss), a ratio of 0.15. (That setting is not run here: the gains and the
/// shift are live statics every test thread shares; section 5 carries the model's figure.)
#[test]
fn the_loop_at_rest_under_the_sense_noise_commands_a_quiet_voltage() {
    let mut seed: u32 = 0x8765_4321;
    let mut noise = || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        ((seed >> 8) % 279) as i16 - 139
    };
    let theta = BASE_ANGLE[4];
    let mut plant = Plant::new(1.0);
    let mut st = FocState::new(ref_offsets(), DutyOrder::DIRECT, 28_500);
    let mut vq = Vec::new();
    for _ in 0..4_000 {
        let (ia, ib) = plant.sense();
        let noisy = (ia.saturating_add(noise()), ib.saturating_add(noise()));
        let before = st.sums().vq;
        let out = foc_step(&mut st, theta, noisy, 0, true, false);
        vq.push(st.sums().vq.wrapping_sub(before) as f64);
        plant.step(out);
    }
    let tail = &vq[1_000..];
    let mean = tail.iter().sum::<f64>() / tail.len() as f64;
    let sd = (tail.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / tail.len() as f64).sqrt();
    std::println!("vq at rest under 80-count sense noise: sd {sd:.1} counts (mean {mean:.1})");
    assert!(sd < 65.0, "vq scatter {sd}");
    assert!(mean.abs() < 20.0, "vq mean {mean}");
}

/// **The negative control: the same loop with the sense polarity inverted runs away** to the
/// circular limit (`specs/foc-torque.md` section 9; within 20 periods at the 2026-10-26 gains,
/// within 150 at round 2's). This is the hazard gate O2
/// exists for: an inverted chain reads the current it pushes as current the other way, and the
/// loop pushes harder. The circular limit and the PI clamps bound it in the model; on a board the
/// soft chop and the supply limit do.
#[test]
fn an_inverted_sense_polarity_saturates_the_q_voltage() {
    // The radius the circular limit holds the vector at: sqrt(CIRC_THRESH) is ~32,111, and the
    // table's gains land the limited vector a little inside it.
    let radius = (CIRC_THRESH as f64).sqrt();
    for code in 1..=6u8 {
        let theta = BASE_ANGLE[code as usize];
        let mut plant = Plant::new(-1.0);
        let t = run_loop(theta, 1_000, &mut plant, 400);
        let saturated =
            t.vq.iter()
                .position(|&vq| vq as f64 > radius * 0.97)
                .expect("the inverted loop never saturated");
        std::println!(
            "code {code}: inverted polarity, vq at the circular limit at period {saturated}"
        );
        // 20 periods at the 2026-10-26 gains; the round-2 loop is six times slower.
        assert!(
            saturated <= 150,
            "code {code}: vq reached the limit at period {saturated}"
        );
        // ...and it stays there, the current climbing away from the reference.
        assert!(t.vq[saturated..]
            .iter()
            .all(|&vq| vq as f64 > radius * 0.95));
        assert!(t.iq.last().unwrap().abs() > 5_000);
    }
}

/// **A reference the voltage ceiling cannot deliver clamps the integrator at its bound**
/// (`specs/foc-torque.md` section 5, anti-windup; gate O6's free wheel): a back-EMF along `theta`
/// above what the circular limit can oppose, which is the rotor-frame picture of a wheel spun past
/// the speed the bus can drive current into. The q accumulator rises to `int_max` and stays there,
/// never past it, `vq` sits at the limit, and `iq` falls under its reference (here it goes
/// negative: the back-EMF wins and drives current back).
#[test]
fn a_back_emf_above_the_voltage_ceiling_clamps_the_accumulator() {
    let k_v = volts_per_voltage_count();
    let int_max = FOC_GAINS_START.int_max;
    for code in 1..=6u8 {
        let theta = BASE_ANGLE[code as usize];
        let deg = theta as f64 * 360.0 / 65536.0;
        let mut plant = Plant::new(1.0);
        // 20 % above the full-scale vector's amplitude.
        let e = 1.2 * 32767.0 * k_v;
        plant.emf = [0.0, 1.0, 2.0].map(|k: f64| e * (deg - 120.0 * k).to_radians().cos());
        let t = run_loop(theta, 1_000, &mut plant, 2_000);
        assert!(
            t.q_acc.iter().all(|a| a.abs() <= int_max),
            "code {code}: the accumulator left its bound"
        );
        assert_eq!(
            *t.q_acc.last().unwrap(),
            int_max,
            "code {code}: clamped at int_max"
        );
        assert!(
            t.iq[1_500..].iter().all(|&iq| iq < 1_000),
            "code {code}: iq under its reference"
        );
        assert!(
            t.vq[1_500..].iter().all(|&vq| vq > 31_000),
            "code {code}: vq at the limit"
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Slice 4: the EFeru FOC fixture (qualitative response direction; same rules as slice 3).
//
// Provenance: the same harness/checkout as the COM fixture above (reference/efferu-oracle,
// EFeru @ a0751d5), FOC section generated 2026-07-04: z_ctrlTypSel=2 (FOC), voltage mode,
// diagnostics off, static rotor at their hall sum 2, ZERO phase currents, +-500 input,
// 400 settle steps. Observed running-model behavior, no tables or constants.
// ---------------------------------------------------------------------------------------------

/// (their_input, [DC_phaA, DC_phaB, DC_phaC]).
const EFERU_FOC_RESPONSE: [(i16, [i16; 3]); 2] = [(500, [-450, 449, 0]), (-500, [450, -450, 0])];

#[test]
fn efferu_foc_fixture_and_ours_mirror_the_response_with_input_sign() {
    // Shared semantic (qualitative response direction): flipping the drive input's sign mirrors
    // the phase response. Their fixture mirrors within the model's 1-LSB rounding asymmetry...
    let (pos, neg) = (EFERU_FOC_RESPONSE[0].1, EFERU_FOC_RESPONSE[1].1);
    for i in 0..3 {
        assert!(
            (pos[i] as i32 + neg[i] as i32).abs() <= 1,
            "phase {i}: {} vs {}",
            pos[i],
            neg[i]
        );
    }
    // ...and ours mirrors exactly at the same seam (the rotor-frame command through inverse
    // Park + SVPWM): negating (vq, vd) rotates the stator vector 180 deg, so the duties reflect
    // about the SVPWM center.
    let theta = BASE_ANGLE[2]; // a static rotor anchor, like the fixture's static hall
    let (vq, vd) = (7000i16, -300i16);
    let (a1, b1) = park_inverse(vq, vd, theta);
    let (a2, b2) = park_inverse(-vq, -vd, theta);
    assert_eq!(
        (a2, b2),
        (-a1, -b1),
        "inverse Park mirrors with the command sign"
    );
    let s1 = svpwm(a1, b1);
    let s2 = svpwm(a2, b2);
    for (d1, d2) in [(s1.base, s2.base), (s1.c1, s2.c1), (s1.c2, s2.c2)] {
        let r1 = d1 as i32 - SVPWM_CENTER;
        let r2 = d2 as i32 - SVPWM_CENTER;
        assert!(
            (r1 + r2).abs() <= 2,
            "duties must reflect about the center: {d1} vs {d2}"
        );
    }
}

// ---------------------------------------------------------------------------------------------
// MotorOutput::to_duties_enables (specs/motor-integration.md, "The per-period hot path" +
// Validation "Host"): the pure lowering to the two hardware register writes. Drive(n)->(n,true),
// Float->(0,false), covered over every six-step sector, the coast posture, and a three-drive case.
// ---------------------------------------------------------------------------------------------

#[test]
fn to_duties_enables_maps_drive_and_float() {
    // Drive carries its compare count and enables the channel; Float zeroes the duty and disables.
    let out = MotorOutput {
        phases: [PhaseCmd::Drive(1700), PhaseCmd::Drive(0), PhaseCmd::Float],
    };
    let (duties, enables) = out.to_duties_enables();
    assert_eq!(duties, [1700, 0, 0]);
    assert_eq!(enables, [true, true, false]);
    // The sink phase (Drive(0)) is a driven leg at compare 0, NOT a float: enable stays true.
    assert!(enables[1], "Drive(0) is a driven sink, not a float");
}

#[test]
fn to_duties_enables_coast_is_all_disabled() {
    // All-Float coast (six-step fault / zero demand): every channel disabled, every duty 0.
    let (duties, enables) = super::sixstep::COAST.to_duties_enables();
    assert_eq!(duties, [0, 0, 0]);
    assert_eq!(enables, [false, false, false]);
}

#[test]
fn to_duties_enables_every_sixstep_sector_is_one_float_two_driven() {
    // Every valid hall code yields exactly one disabled (floated) channel and two driven, and the
    // duties match the decoded pattern (source at the scaled duty, sink at 0).
    use super::sixstep::{sixstep_step, SixStep, SixStepState};
    use super::Direction;
    let st = SixStepState::new(SixStep::new(Direction::Forward, 0));
    let demand = 16_000i32;
    let src_duty = super::sixstep::demand_to_duty(demand);
    for code in [1u8, 2, 3, 4, 5, 6] {
        let out = sixstep_step(&st, code, demand);
        let (duties, enables) = out.to_duties_enables();
        let driven = enables.iter().filter(|e| **e).count();
        assert_eq!(driven, 2, "code {code}: exactly two driven phases");
        assert_eq!(
            enables.iter().filter(|e| !**e).count(),
            1,
            "code {code}: exactly one floated phase"
        );
        // Each disabled channel carries duty 0; the two driven carry the source duty and 0 (sink).
        for i in 0..3 {
            match out.phases[i] {
                PhaseCmd::Float => {
                    assert!(!enables[i]);
                    assert_eq!(duties[i], 0);
                }
                PhaseCmd::Drive(n) => {
                    assert!(enables[i]);
                    assert_eq!(duties[i], n);
                    assert!(n == 0 || n == src_duty, "driven duty is sink(0) or source");
                }
            }
        }
    }
}

#[test]
fn to_duties_enables_three_drive_case() {
    // The sine/FOC posture: all three phases driven about mid-rail, every channel enabled, every
    // duty carried verbatim.
    let out = MotorOutput {
        phases: [
            PhaseCmd::Drive(MID_RAIL + 400),
            PhaseCmd::Drive(MID_RAIL),
            PhaseCmd::Drive(MID_RAIL - 400),
        ],
    };
    let (duties, enables) = out.to_duties_enables();
    assert_eq!(duties, [MID_RAIL + 400, MID_RAIL, MID_RAIL - 400]);
    assert_eq!(enables, [true, true, true]);
    // Duties stay within the 0..=ARR compare scale.
    for d in duties {
        assert!(d <= ARR);
    }
}

// --- The rotor front-end accessor (the integration handoff's source) ---------------------------

/// `front()` exposes the SAME rotor state the step just produced: the published angle, the latched
/// speed, and the debounced hall code, which is what the integration layer publishes into the
/// handoff words + `CTRL_OBS` after each period. Driven over a real forward sector walk so the
/// values are the front-end's, not defaults.
#[test]
fn front_exposes_the_stepped_rotor_state() {
    let mut c = commutator(MethodState::SixStep(SixStepState::new(SixStep::new(
        Direction::Forward,
        0,
    ))));
    // Sector 1 (code 1 = hall A only): raw levels [1, 0, 0].
    for _ in 0..3 {
        c.step([1, 0, 0], (0, 0), 0, true, false);
    }
    assert_eq!(
        c.front().comm.prev_any_code,
        1,
        "the debounced code stepped"
    );
    assert_eq!(
        c.front().comm.angle,
        BASE_ANGLE[1],
        "the published angle is sector 1's base (stationary: no interpolation drift yet)"
    );
    assert_eq!(c.front().comm.invalid_dwell, 0, "a valid code clears dwell");
    assert!(!c.front().comm.hall_fault);

    // An INVALID code (all lines low = 0) raises the dwell counter and holds the angle: the
    // observation block's invalid-code count comes from here. Line A dropping back to 0 is a
    // second edge on the SAME line with no other line moving in between, which is precisely the
    // case the debounce holds off, so run its window out first.
    for _ in 0..=hall_debounce_periods(REF_PERIOD_HZ) {
        c.step([1, 0, 0], (0, 0), 0, true, false);
    }
    c.step([0, 0, 0], (0, 0), 0, true, false);
    assert_eq!(c.front().comm.prev_any_code, 0);
    assert!(c.front().comm.invalid_dwell >= 1, "invalid dwell counted");
    assert_eq!(
        c.front().comm.angle,
        BASE_ANGLE[1],
        "an invalid code does not rewrite the angle from the table"
    );
}

// ---------------------------------------------------------------------------------------------
// The hall debounce as a RATE LIMIT: the property the bench found missing.
// ---------------------------------------------------------------------------------------------

/// The reference period-ISR rate the recovered constants were measured at: the stock timer
/// contract (72 MHz / (2 x ARR), center-aligned) = 16 kHz.
const REF_PERIOD_HZ: u32 = 72_000_000 / (2 * ARR as u32);

/// The fleet hub's pole-pair count (`specs/board-model.md`: EFeru `n_polePairs` = 15). Written
/// down because every rpm figure in these tests converts through it.
const POLE_PAIRS: u32 = 15;

/// PWM periods per hall sector at a given MECHANICAL rpm: six sectors per electrical revolution,
/// `POLE_PAIRS` electrical revolutions per mechanical one.
fn periods_per_sector(rpm: u32) -> usize {
    (REF_PERIOD_HZ as usize * 60) / (rpm as usize * POLE_PAIRS as usize * 6)
}

/// Walk the forward sector sequence at `k` periods per sector for `warmup + revolutions`
/// electrical revolutions, and return how many periods AFTER the warm-up decoded a sector other
/// than the one being driven. A front end that tracks the rotor returns 0.
///
/// The warm-up matters for precision, not for leniency: spinning up from rest inserts one extra
/// edge (all-low to the first sector) that shortens the first same-line interval, so counting from
/// period zero measures the start transient on top of the steady-state property. Both are real,
/// and `no_hall_edge_is_dropped_at_working_speed` covers the from-rest case with no warm-up.
fn dropped_edges_at(fe: &mut RotorFrontEnd, k: usize, warmup: usize, revolutions: usize) -> usize {
    let fwd = [1u8, 3, 2, 6, 4, 5];
    let mut dropped = 0usize;
    for sector in 0..(6 * (warmup + revolutions)) {
        let want = fwd[sector % 6];
        for _ in 0..k {
            let code = fe.step(lines(want)).code;
            if sector >= 6 * warmup && code != want {
                dropped += 1;
            }
        }
    }
    dropped
}

/// THE regression property (bench, 2026-08-12): at a working road speed every commanded hall edge
/// must be accepted, so the decoded sector follows the rotor. When it does not, the vector stops
/// following the rotor, torque collapses and current climbs with duty -- which is exactly what the
/// bench saw above ~40 % throttle.
///
/// 1000 rpm is the top of the machine's working range (~31 km/h on a 6.5" wheel).
#[test]
fn no_hall_edge_is_dropped_at_working_speed() {
    let k = periods_per_sector(1000);
    assert!(
        k >= 2,
        "the sample rate must resolve a sector at all (k = {k})"
    );
    let mut fe = front_end();
    let dropped = dropped_edges_at(&mut fe, k, 0, 6);
    assert_eq!(
        dropped,
        0,
        "{dropped} of {} periods decoded a stale sector at 1000 rpm ({k} periods/sector): the \
         front end is dropping hall edges, so the vector is not following the rotor",
        k * 36
    );
}

/// The same property in steady state across the whole working speed range, so a fix that merely
/// moves the cap higher cannot be mistaken for one that removes it.
///
/// Note what the warm-up revolution does NOT buy: a per-line rate limiter fails this at every
/// speed in the list, including 200 rpm, whose steady-state same-line edge spacing (159 periods)
/// would clear a 150-period lockout on its own. Dropping an edge is not self-correcting. The
/// spin-up transient costs one edge, the stored levels stop matching the rotor, and the front end
/// never resynchronises, so a machine that was only ever going to run at 200 rpm is still broken.
#[test]
fn the_front_end_tracks_the_whole_working_speed_range() {
    for rpm in [200u32, 400, 600, 800, 1000, 1200] {
        let k = periods_per_sector(rpm);
        let mut fe = front_end();
        let dropped = dropped_edges_at(&mut fe, k, 1, 4);
        assert_eq!(
            dropped, 0,
            "edges dropped at {rpm} rpm ({k} periods/sector): the front end caps below {rpm} rpm"
        );
    }
}

/// The debounce is specified as a TIME, so the period count it lowers to has to move with the
/// period rate. This is the property that makes the constant safe to read: 150 alone means nothing
/// without 16 kHz beside it, and the recovered value is bit-exact only because the reference rate
/// is 16 kHz (`~/dev/Declassyfied/spec/commutation.md` 3).
#[test]
fn hall_debounce_is_a_time_not_a_period_count() {
    assert_eq!(REF_PERIOD_HZ, 16_000);
    assert_eq!(hall_debounce_periods(REF_PERIOD_HZ), 150, "0x96, recovered");

    // Double the ISR rate and the count doubles, holding the window at the same wall-clock time.
    assert_eq!(hall_debounce_periods(2 * REF_PERIOD_HZ), 300);
    assert_eq!(hall_debounce_periods(REF_PERIOD_HZ / 2), 75);

    // Across a wide range of plausible rates the realised window never falls below the specified
    // one (the count rounds up) and never overshoots it by more than one period.
    for period_hz in [2_000u32, 4_000, 8_000, 16_000, 20_000, 32_000, 48_000] {
        let realised = hall_debounce_window_us(period_hz);
        assert!(
            realised >= HALL_DEBOUNCE_US,
            "the window shrank to {realised} us at {period_hz} Hz"
        );
        assert!(
            realised - HALL_DEBOUNCE_US <= 1_000_000 / period_hz,
            "the window overshot by more than one period at {period_hz} Hz"
        );
    }
}

/// The ceiling the front end actually has, once the debounce is not a rate limiter: the sample
/// rate. Six sectors per electrical revolution, one hall read per period.
#[test]
fn the_tracking_ceiling_is_the_sample_rate_with_room_over_the_working_range() {
    assert_eq!(tracked_electrical_hz_ceiling(REF_PERIOD_HZ), 2_666);
    // 2,666 Hz electrical on a 15-pole-pair hub is 10,664 mechanical rpm.
    let rpm_ceiling = tracked_electrical_hz_ceiling(REF_PERIOD_HZ) * 60 / POLE_PAIRS;
    assert_eq!(rpm_ceiling, 10_664);
    // The floor the firmware asserts against is 1,000 rpm, the top of the working range.
    assert_eq!(MIN_TRACKED_ELECTRICAL_HZ * 60 / POLE_PAIRS, 1_000);
    assert!(tracked_electrical_hz_ceiling(REF_PERIOD_HZ) >= MIN_TRACKED_ELECTRICAL_HZ);
}

/// The stock eligibility rule (`~/dev/Declassyfied/spec/commutation.md` 5.1, the clean-room
/// `firmware/commutation.c:275`, and `FUN_080070b4` in the board20 decompile): a line is eligible
/// if EITHER its "recently changed" marker is clear OR its own lockout has drained. Any other
/// line's edge clears this line's marker, so during valid rotation -- where consecutive sectors
/// always move DIFFERENT lines -- the lockout never gates an edge. Dropping that disjunct turns a
/// bounce filter into a hard per-line rate limit.
#[test]
fn another_lines_edge_frees_a_locked_line() {
    let mut d = debounce();
    assert_eq!(d.step([1, 0, 0]), 0b001, "A's edge is accepted");
    assert_eq!(
        d.step([1, 1, 0]),
        0b011,
        "B's edge is accepted and clears A's marker"
    );
    assert!(
        d.lockout[0] > 0,
        "A's own lockout is still draining, which is the point of the test"
    );
    assert_eq!(
        d.step([0, 1, 0]),
        0b010,
        "A is eligible again: B's edge cleared A's marker, so A's lockout does not gate it"
    );
}

/// The other half of the same rule, so the fix cannot be "delete the lockout": while NO other line
/// has moved, a line that just changed stays deaf for the whole debounce window. That is the
/// bounce rejection the filter exists for, and it is what keeps a chattering line at standstill
/// from registering spurious edges.
#[test]
fn a_line_that_chatters_alone_is_rejected_for_the_whole_window() {
    let mut d = debounce();
    let reload = d.reload;
    assert_eq!(d.step([1, 0, 0]), 0b001, "A's edge is accepted");
    // A chatters back with no other line moving: rejected for the rest of the window.
    for period in 1..reload {
        assert_eq!(
            d.step([0, 0, 0]),
            0b001,
            "A's chatter was accepted at period {period} of a {reload}-period window"
        );
    }
    assert_eq!(
        d.step([0, 0, 0]),
        0b000,
        "once the window has drained the level change is accepted"
    );
}

// ---------------------------------------------------------------------------------------------
// The motor's alignment on the angle path (`specs/commutation.md`, "The alignment every method
// consumes"; measured 2026-10-20, `specs/bench-evidence/2026-10-20/sine-phase/RECORD.md`).
// ---------------------------------------------------------------------------------------------

/// The sine arm's own offset is the quarter-turn `sin`-versus-`cos` convention and nothing else:
/// a board's sine output is `sine_step` at the published angle plus that 90 deg plus the motor's
/// alignment, and the alignment is the only per-board term there is.
#[test]
fn the_sine_offset_is_the_quarter_turn_convention_only() {
    assert_eq!(super::SINE_PHASE_OFFSET_DEFAULT, ANGLE_90);
    assert_eq!(super::SINE_PHASE_OFFSET_DEFAULT, 0x4000);
    assert_eq!(Direction::Forward.angle_offset(), 0);
    let demand = 10_000;
    for offset in 0..6u8 {
        let align = SixStep::new(Direction::Forward, offset);
        for code in 1..=6u8 {
            let mut c = commutator_aligned(MethodState::Sine, Direction::Forward, offset);
            let out = c.step(lines(code), (0, 0), demand, true, false);
            let angle = c.front().comm.angle;
            assert_eq!(
                out.phases,
                sine::sine_step(
                    angle
                        .wrapping_add(angle_path_offset(align))
                        .wrapping_add(0x4000),
                    demand
                )
                .phases,
                "code {code}, align offset {offset}"
            );
        }
    }
}

/// A half-turn of the angle negates the lookup exactly, at every angle: the 180 deg the reverse
/// direction adds is a bit-exact inversion of the vector, not an approximation of one.
#[test]
fn a_half_turn_negates_the_lookup_exactly() {
    assert_eq!(Direction::Reverse.angle_offset(), 0x8000);
    for step in 0..1024u32 {
        let theta = (step * 64) as u16;
        let (s, c) = lookup_sincos(theta);
        assert_eq!(
            lookup_sincos(theta.wrapping_add(0x8000)),
            (-s, -c),
            "theta {theta:#06x}"
        );
    }
}

/// A reversed board drives the inverted vector on BOTH angle-path arms, the way six-step's
/// `Direction::Reverse` drives its mirror pattern: the sine duties mirror about the neutral
/// (reverse is forward with the demand's sign flipped, which is the silicon's "same current,
/// opposite speed sign"), and the FOC arm runs its two Park transforms half a turn round. Both
/// are pinned to the arm's own function at the shifted angle, so the half-turn is applied once,
/// in the dispatch, and not a second time inside an arm.
#[test]
fn a_reversed_board_drives_the_mirror_vector_on_the_angle_path() {
    let demand = 10_000;
    // Currents well off zero, so both FOC loops have a measurement to act on at every anchor.
    let currents = (2_400, -900);
    for code in 1..=6u8 {
        let raw = lines(code);

        // Sine.
        let mut fwd = commutator(MethodState::Sine);
        let mut rev = commutator_with(MethodState::Sine, Direction::Reverse);
        let f = fwd.step(raw, (0, 0), demand, true, false);
        let r = rev.step(raw, (0, 0), demand, true, false);
        let angle = fwd.front().comm.angle;
        assert_eq!(
            rev.front().comm.angle,
            angle,
            "the published angle is the rotor's, not the drive's"
        );
        assert_eq!(
            r.phases,
            sine::sine_step(
                angle
                    .wrapping_add(angle_path_offset(SixStep::new(Direction::Forward, 0)))
                    .wrapping_add(0x4000)
                    .wrapping_add(0x8000),
                demand
            )
            .phases,
            "code {code}: sine reverse is the carrier half a turn on"
        );
        assert_eq!(
            r.phases,
            fwd.step(raw, (0, 0), -demand, true, false).phases,
            "code {code}: sine reverse is forward with the demand negated"
        );
        for i in 0..3 {
            assert_eq!(
                duty_of(f.phases[i]) + duty_of(r.phases[i]),
                2 * MID_RAIL,
                "code {code} phase {i}: the reverse duty mirrors the forward one about the neutral"
            );
        }
        assert_ne!(
            r.phases, f.phases,
            "code {code}: the direction must move the sine drive"
        );

        // FOC. A current regulator at a ZERO reference is frame-invariant (the half-turn
        // negates both the measured currents and the loops' answer to them, so the same stator
        // vector comes out), so the direction shows where the torque command enters: the q
        // reference, which carries the demand's sign and NOT the frame's.
        let foc = || MethodState::Foc(foc_state());
        let mut fwd = commutator(foc());
        let mut rev = commutator_with(foc(), Direction::Reverse);
        let mut fe = front_end();
        let mut fresh = foc_state();
        for period in 0..4 {
            let f = fwd.step(raw, currents, demand, true, false);
            let r = rev.step(raw, currents, demand, true, false);
            let rotor = fe.step(raw);
            let half_turn = rotor
                .angle
                .wrapping_add(angle_path_offset(SixStep::new(Direction::Reverse, 0)));
            assert_eq!(
                r.phases,
                foc_step(&mut fresh, half_turn, currents, demand, true, false).phases,
                "code {code} period {period}: FOC reverse is foc_step half a turn on"
            );
            assert_ne!(
                r.phases, f.phases,
                "code {code} period {period}: the direction must move the FOC drive"
            );
        }
        // At a zero demand with no current the two directions both output the zero vector:
        // nothing to tell them apart, and nothing driven.
        let mut fwd = commutator(foc());
        let mut rev = commutator_with(foc(), Direction::Reverse);
        assert_eq!(
            fwd.step(raw, (0, 0), 0, true, false),
            rev.step(raw, (0, 0), 0, true, false)
        );
    }
}

/// The drive vector of a six-step output, in degrees: the source phase is +1, the sink -1, the
/// floating phase 0 (the decode's own weights, read back off the dispatch's output).
fn sixstep_vector_deg(out: MotorOutput) -> f64 {
    vector_angle_deg(out.phases.map(|p| match p {
        PhaseCmd::Drive(0) => -1.0,
        PhaseCmd::Drive(_) => 1.0,
        PhaseCmd::Float => 0.0,
    }))
}

/// The drive vector of a centered three-phase output, in degrees: each phase's swing about
/// mid-rail is its weight.
fn centered_vector_deg(out: MotorOutput) -> f64 {
    vector_angle_deg(out.phases.map(|p| duty_of(p) as f64 - MID_RAIL as f64))
}

/// **The same hall state and the same staged alignment drive the same vector under six-step and
/// under the angle path** (`specs/commutation.md`, "The alignment every method consumes"). This
/// is the property that makes `motor.method` safe to change: the rotor is where it is, and the
/// three arms must all drive against it the same way.
///
/// It is checked over every hall code, every align offset and both directions, on the FIRST step
/// of a fresh front end, where the published angle is exactly `BASE_ANGLE[code]` (asserted) so
/// the two outputs are compared at one rotor position with no interpolation between them. The
/// tolerance is a tenth of a degree, against the 60 degrees a single offset unit is worth.
///
/// It also DERIVES `DECODE_SECTOR_LEAD` rather than trusting it: at align offset 0 the decode's
/// vector leads the anchor by exactly two sectors, at every code.
#[test]
fn the_two_paths_drive_the_same_vector() {
    let demand = 10_000;
    for direction in [Direction::Forward, Direction::Reverse] {
        for offset in 0..6u8 {
            let align = SixStep::new(direction, offset);
            for code in [1u8, 3, 2, 6, 4, 5] {
                let raw = lines(code);
                let mut six = commutator_aligned(
                    MethodState::SixStep(SixStepState::new(align)),
                    direction,
                    offset,
                );
                let mut sin = commutator_aligned(MethodState::Sine, direction, offset);
                let six_out = six.step(raw, (0, 0), demand, true, false);
                let sine_out = sin.step(raw, (0, 0), demand, true, false);
                let anchor = BASE_ANGLE[code as usize];
                assert_eq!(
                    six.front().comm.angle,
                    anchor,
                    "code {code}: the first step snaps the angle to the anchor"
                );
                let delta =
                    (centered_vector_deg(sine_out) - sixstep_vector_deg(six_out)).rem_euclid(360.0);
                let delta = if delta > 180.0 { delta - 360.0 } else { delta };
                assert!(
                    delta.abs() < 0.1,
                    "direction {direction:?}, offset {offset}, code {code}: the angle path drives \
                     {delta} deg away from six-step"
                );
                // FOC's torque axis at the same aligned angle (`specs/foc-torque.md` section 3):
                // `(vq, vd) = (V, 0)` through inverse Park and SVPWM is the stator vector AT the
                // angle, so a positive q command drives the vector six-step and sine drive. The
                // tolerance is the lookup's index truncation (up to 63 of 65,536 units, 0.35 deg)
                // through two table reads.
                let aligned = anchor.wrapping_add(angle_path_offset(align));
                let (alpha, beta) = park_inverse(12_000, 0, aligned);
                let foc_out = MotorOutput {
                    phases: DutyOrder::DIRECT
                        .apply(svpwm(alpha, beta))
                        .map(PhaseCmd::Drive),
                };
                let delta =
                    (centered_vector_deg(foc_out) - sixstep_vector_deg(six_out)).rem_euclid(360.0);
                let delta = if delta > 180.0 { delta - 360.0 } else { delta };
                assert!(
                    delta.abs() < 0.6,
                    "direction {direction:?}, offset {offset}, code {code}: FOC's vq drives \
                     {delta} deg away from six-step"
                );
                // The constant the angle path's offset is built on, derived from the two tables.
                if direction == Direction::Forward && offset == 0 {
                    let anchor_deg = anchor as f64 * 360.0 / 65536.0;
                    let lead = (sixstep_vector_deg(six_out) - anchor_deg).rem_euclid(360.0);
                    assert!(
                        (lead - super::DECODE_SECTOR_LEAD as f64 * 60.0).abs() < 0.1,
                        "code {code}: the decode leads the anchor by {lead} deg"
                    );
                }
            }
        }
    }
}

/// The FOC arm indexes the same aligned angle, for every staged offset: the dispatch's output is
/// `foc_step` at the published angle plus `angle_path_offset`, so the align offset turns FOC's
/// two Park transforms with the drive it commands (a frame that did not turn with it would
/// regulate the measured current against a command in another frame).
#[test]
fn the_align_offset_turns_the_foc_frame_too() {
    let currents = (2_400, -900);
    let demand = 10_000;
    for direction in [Direction::Forward, Direction::Reverse] {
        for offset in 0..6u8 {
            let align = SixStep::new(direction, offset);
            for code in [1u8, 3, 2, 6, 4, 5] {
                let raw = lines(code);
                let mut c = commutator_aligned(MethodState::Foc(foc_state()), direction, offset);
                let mut fe = front_end();
                let mut st = foc_state();
                let got = c.step(raw, currents, demand, true, false);
                let rotor = fe.step(raw);
                let aligned = rotor.angle.wrapping_add(angle_path_offset(align));
                assert_eq!(
                    got.phases,
                    foc_step(&mut st, aligned, currents, demand, true, false).phases,
                    "direction {direction:?}, offset {offset}, code {code}"
                );
            }
        }
    }
}

/// `angle_path_offset` is the decode's own index composition, read in angle: the direction's
/// half-turn is exactly the decode's `+3` sectors, and the whole offset is `(lead + align + half)`
/// sectors. Rounding each multiple on its own keeps the two forms bit-identical.
#[test]
fn the_angle_path_offset_is_the_decodes_composition() {
    assert_eq!(super::sector_angle(3), Direction::Reverse.angle_offset());
    assert_eq!(super::sector_angle(0), Direction::Forward.angle_offset());
    assert_eq!(super::sector_angle(6), super::sector_angle(0));
    for offset in 0..6u8 {
        for (direction, half) in [(Direction::Forward, 0u8), (Direction::Reverse, 3)] {
            assert_eq!(
                angle_path_offset(SixStep::new(direction, offset)),
                super::sector_angle(super::DECODE_SECTOR_LEAD + offset + half),
                "direction {direction:?}, offset {offset}"
            );
        }
        // One offset unit is one sector of the published angle, in the direction the decode
        // rotates the state assignment.
        let step = angle_path_offset(SixStep::new(Direction::Forward, (offset + 1) % 6))
            .wrapping_sub(angle_path_offset(SixStep::new(Direction::Forward, offset)));
        assert!(
            step == 10922 || step == 10923,
            "offset {offset}: a unit moved the angle by {step}, not a sector"
        );
    }
}
