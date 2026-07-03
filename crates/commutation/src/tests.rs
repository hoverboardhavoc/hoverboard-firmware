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
