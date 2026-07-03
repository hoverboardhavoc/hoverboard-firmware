//! The FOC math layer, slice-1 subset: the shared fixed-point primitives.
//!
//! Recovered bit-exact from the archived implementation of the stock contract
//! (`specs/commutation.md`, Provenance): the angle constants, the stock `MAC`/`RND` rounding
//! forms, and the section-7 sine/cosine table + quadrant-folded lookup. The remaining FOC blocks
//! (conditioning + cal, Clarke/Park, q-PI, d-ramp, circular limit, SVPWM, `foc_step`) arrive in
//! slice 4; the shared hall front-end in slice 2.
//!
//! Q15 here means a RAW `i16` scaled +/-1.0 = +/-32767 (see the crate doc for why there is no
//! typed view).

// ============================================================================================
// Angle representation (spec "The shared hall front-end": 16-bit wrapping, 65536/electrical rev)
// ============================================================================================

/// 60 deg in 16-bit angle units (65536 / 6 = 10922.67, truncated). One sector.
pub const SECTOR_ANGLE: u16 = 0x2AAA; // 10922
/// 90 deg in angle units.
pub const ANGLE_90: u16 = 0x4000;

// ============================================================================================
// The stock round-and-saturate helpers (the FOC MAC / RND forms)
// ============================================================================================

/// Q15 round-and-shift WITHOUT saturation (the stock `RND`, used by inverse Park and the
/// circular-limit scaling): `(acc + ((acc >> 31) >> 17)) >> 15`, then truncate to the low 16 bits
/// (so an over-range result WRAPS mod 2^16, a deliberate part of the contract).
#[inline]
pub fn rnd_q15(acc: i32) -> i16 {
    // `(acc >> 31)` is 0 (non-negative) or -1 (negative); `>> 17` as a LOGICAL shift on the u32
    // view yields the bias 2^15 - 1 for negatives and 0 otherwise (round-half-away-from-zero).
    let bias = (((acc >> 31) as u32) >> 17) as i32;
    let shifted = acc.wrapping_add(bias) >> 15;
    shifted as i16 // truncate to 16 bits: wraps on overflow (no saturate)
}

/// Q15 round-and-saturate (the stock `MAC`): round as [`rnd_q15`] then saturate to
/// [-32767, +32767]. The -32768 sentinel is replaced with -32767 (reserved value).
#[inline]
pub fn mac_q15(acc: i32) -> i16 {
    let bias = (((acc >> 31) as u32) >> 17) as i32;
    let shifted = acc.wrapping_add(bias) >> 15;
    sat16(shifted)
}

/// Saturate an i32 to the symmetric signed-16 range [-32767, +32767] (the -32768 sentinel maps to
/// -32767).
#[inline]
pub fn sat16(x: i32) -> i16 {
    // The recovered form is two cases (underflow -> -32767, and the exact -32768 sentinel ->
    // -32767); they merge to one branch with identical semantics (clippy: if_same_then_else).
    if x > 32767 {
        32767
    } else if x <= -32768 {
        -32767
    } else {
        x as i16
    }
}

// ============================================================================================
// The sine/cosine table and quadrant-folded lookup (stock section 7)
// ============================================================================================

/// Quarter-wave sine table: 256 entries, `round(32767 * sin((i/256)*(pi/2)))` for i = 0..255.
/// Recovered, bit-exact (table[0]=0, table[1]=201, table[127]=23027, table[128]=23170,
/// table[255]=32766); the host test re-derives every entry against f64::sin.
pub static SIN_QUARTER: [i16; 256] = SIN_QUARTER_LITERAL;

/// `lookup_sincos(theta)` -> `(sin, cos)`, each Q15, for the 16-bit angle convention.
///
/// Quadrant select `quad = ((theta + 0x8000) >> 6) & 0x300`; in-quadrant index
/// `i = (theta & 0x3FFF) >> 6`, complement `j = 0xFF - i`; per-quadrant sign/reflection per the
/// recovered table.
#[inline]
pub fn lookup_sincos(theta: u16) -> (i16, i16) {
    let t = theta as u32;
    let quad = (t.wrapping_add(0x8000) >> 6) & 0x300;
    let i = ((t & 0x3FFF) >> 6) as usize; // 0..255
    let j = 0xFF - i;
    let ti = SIN_QUARTER[i];
    let tj = SIN_QUARTER[j];
    match quad {
        0x000 => (-ti, -tj),
        0x100 => (-tj, ti),
        0x200 => (ti, tj),
        0x300 => (tj, -ti),
        _ => unreachable!(),
    }
}

// The precomputed quarter-wave sine literal (round(32767 * sin((i/256)*pi/2)), i=0..255).
include!("sin_table.rs");
