//! The six-step (trapezoidal, hall-driven block commutation) arm, the DEFAULT method.
//!
//! Six-step needs NO current sensing: a hall code plus a voltage (duty) demand is enough to turn
//! the motor. It is the low-risk first-motion path, selectable before phase-current sensing is
//! validated on a board (see todo/commutation-methods.md).
//!
//! Per control period:
//!   1. Read + debounce halls and decode the 6-state code to a sector (the SHARED
//!      [`crate::foc::RotorFrontEnd`], same hall numbering / angle anchors as FOC).
//!   2. Index a commutation table by the hall code to a per-phase drive sign `{-1, 0, +1}` (one
//!      phase positive, one negative, one nominally idle).
//!   3. Scale by the voltage demand (a duty amplitude from the throttle/speed command, NOT a
//!      current loop) to three duties on the 0..ARR scale.
//!   4. Apply a per-motor sector-to-phase sign permutation (board mounting / phase-order
//!      differences), defaulting to identity.
//!
//! No current loop, no injected ADC. The table and the sector decode are anchored to this project's
//! own [`crate::foc::BASE_ANGLE`] hall numbering, not copied from another firmware; the per-motor
//! permutation absorbs wiring differences.
//!
//! # Floating-phase limitation (mid-rail, never floated)
//!
//! Classic six-step puts the idle phase into high-Z (both its FETs off). The current `runtime-hal`
//! [`runtime_hal::hotpath::PwmHandle::set_duties`] writes only the three compare registers and never
//! touches the channel-enable bits, so all three complementary pairs stay driven. There is no
//! per-channel high-Z primitive (filed as a HAL feature request). So the shippable form here is
//! "third phase driven to MID-RAIL (50% duty = ARR/2), never floated": the idle phase sits at the
//! half-period so it sources/sinks no average voltage relative to the centered neutral, which is
//! adequate to spin the wheel and matches what the reference firmwares ship. The `+1` phase rises
//! above mid-rail by the demand, the `-1` phase drops below it by the demand, the `0` phase holds at
//! mid-rail.

use crate::foc::BASE_ANGLE;
use crate::MotorOutput;

/// The PWM period (ARR) on the 0..ARR duty scale (TIMER0 center-aligned, ARR 2250).
pub const ARR: u16 = 2250;
/// Mid-rail duty: half the period (1125). The idle phase holds here; the active phases swing about
/// it by the scaled demand. A zero demand leaves all three phases at mid-rail (safe / centered).
pub const MID_RAIL: u16 = ARR / 2; // 1125

/// The per-hall-code drive-sign table: `SIXSTEP_SIGNS[code]` is `[signA, signB, signC]`, each in
/// `{-1, 0, +1}` (one phase high, one low, one idle). Index by the debounced hall code 1..6; index
/// 0 and 7 (invalid codes) are all-zero so an invalid hall holds all phases at mid-rail.
///
/// The pattern is anchored to this project's [`crate::foc::BASE_ANGLE`]. The forward commutation
/// sequence (from [`crate::foc`]'s `FWD_NEXT`) is 1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1, and the rotor
/// electrical angle advances 60 degrees each step. The drive vector is assigned to LEAD the rotor by
/// a constant 60 degrees and advance exactly 60 degrees per commutation step (a proper six-step
/// rotation), derived directly from the `BASE_ANGLE` anchors (not from another firmware's hall
/// numbering):
///
/// ```text
///   forward sector :   1       3       2       6       4       5
///   rotor angle    : 210deg  270deg  330deg   30deg   90deg  150deg
///   drive angle    : 270deg  330deg   30deg   90deg  150deg  210deg   (constant +60deg lead)
///   (A, B, C) signs: 0,-,+   +,-,0   +,0,-   0,+,-   -,+,0   -,0,+
/// ```
///
/// Indexed by hall code 1..6 (index 0 and 7 are invalid -> all-idle / all-mid-rail), the table is:
pub static SIXSTEP_SIGNS: [[i8; 3]; 8] = [
    [0, 0, 0],   // 0: invalid  -> all mid-rail
    [0, -1, 1],  // 1: (A0, B-, C+)
    [1, 0, -1],  // 2: (A+, B0, C-)
    [1, -1, 0],  // 3: (A+, B-, C0)
    [-1, 1, 0],  // 4: (A-, B+, C0)
    [-1, 0, 1],  // 5: (A-, B0, C+)
    [0, 1, -1],  // 6: (A0, B+, C-)
    [0, 0, 0],   // 7: invalid  -> all mid-rail
];

/// A per-motor sector-to-phase sign permutation (board mounting / phase-order differences). It maps
/// the table's `[signA, signB, signC]` to the actual `[CHx, CHy, CHz]` order for this motor. The
/// default is identity. A board whose phases are wired in a different order, or whose hall ordering
/// is reversed, overrides this so the same hall code drives the correct physical phases.
///
/// `order` is a permutation of `[0, 1, 2]` (which table phase feeds output channel i); `invert`
/// negates all three signs (reverses drive direction) when the board's phase sense is flipped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhasePermutation {
    /// `order[i]` selects which table phase (0=A, 1=B, 2=C) drives output channel i.
    pub order: [u8; 3],
    /// When true, negate every sign (overall drive-direction flip).
    pub invert: bool,
}

impl PhasePermutation {
    /// The identity permutation: output channel i is table phase i, no inversion. The default.
    pub const IDENTITY: PhasePermutation = PhasePermutation {
        order: [0, 1, 2],
        invert: false,
    };

    /// Apply the permutation to a table sign triple, returning the per-output-channel signs.
    #[inline]
    pub fn apply(&self, signs: [i8; 3]) -> [i8; 3] {
        let s = if self.invert {
            [-signs[0], -signs[1], -signs[2]]
        } else {
            signs
        };
        [
            s[self.order[0] as usize],
            s[self.order[1] as usize],
            s[self.order[2] as usize],
        ]
    }
}

impl Default for PhasePermutation {
    fn default() -> Self {
        PhasePermutation::IDENTITY
    }
}

/// The per-motor six-step state. There is no current loop and no interpolation state of its own; all
/// rotor information comes from the shared front-end, so the only state is the per-motor phase
/// permutation.
#[derive(Clone, Copy, Debug, Default)]
pub struct SixStepState {
    /// The per-motor sector-to-phase sign permutation (defaults to identity).
    pub perm: PhasePermutation,
}

impl SixStepState {
    /// A fresh six-step state with the identity permutation.
    pub fn new() -> Self {
        Self {
            perm: PhasePermutation::IDENTITY,
        }
    }
}

/// Clamp the (signed) drive demand to the duty amplitude available around mid-rail. The demand is a
/// voltage/throttle amplitude on roughly the same +-32767 scale the FOC frame uses; it is scaled to
/// at most `MID_RAIL` so the `+1`/`-1` phases never exceed `0..ARR`. Negative demand reverses the
/// drive (swaps which phases lead), preserved as a signed amplitude here and absorbed into the
/// signs by the caller's sign convention.
#[inline]
pub fn demand_to_amplitude(demand: i32) -> u16 {
    // Scale the |demand| (Q15-ish full scale 32767) to the [0, MID_RAIL] duty swing. Saturate at
    // MID_RAIL so the active phases stay inside 0..ARR.
    let mag = demand.unsigned_abs();
    let scaled = (mag as u64 * MID_RAIL as u64) / 32767u64;
    if scaled > MID_RAIL as u64 {
        MID_RAIL
    } else {
        scaled as u16
    }
}

/// One six-step control step. `code` is the debounced hall code (0..7) from the shared front-end;
/// `demand` is the signed voltage/throttle demand from the outer loop. Returns the three duties on
/// the 0..ARR scale (the idle phase at mid-rail, the `+`/`-` phases swung about it by the scaled
/// demand). A zero demand or an invalid hall code leaves all three phases at mid-rail (safe,
/// centered, no net drive). The third phase is held at mid-rail, never floated (see module docs).
#[inline]
pub fn sixstep_step(state: &SixStepState, code: u8, demand: i32) -> MotorOutput {
    let signs = state.perm.apply(SIXSTEP_SIGNS[(code & 0x7) as usize]);
    // A negative demand reverses drive: flip the sign convention so a reverse command swings the
    // opposite way. The amplitude is the magnitude; the direction rides on the demand's sign.
    let dir: i32 = if demand < 0 { -1 } else { 1 };
    let amp = demand_to_amplitude(demand) as i32;

    let mut duties = [MID_RAIL; 3];
    for (i, &sgn) in signs.iter().enumerate() {
        let off = (sgn as i32) * dir * amp;
        let d = MID_RAIL as i32 + off;
        // Clamp into 0..ARR defensively (amp <= MID_RAIL keeps this in range, but the clamp makes
        // the invariant explicit and survives a future larger amplitude scale).
        duties[i] = d.clamp(0, ARR as i32) as u16;
    }
    MotorOutput { duties }
}

/// True for a hall code that drives the motor (1..6). Codes 0 and 7 are invalid (all sensors low /
/// all high) and produce the all-mid-rail safe output.
#[inline]
pub fn is_valid_code(code: u8) -> bool {
    (1..=6).contains(&code)
}

/// The electrical-angle anchor for a hall code (re-exported convenience over [`BASE_ANGLE`] so a
/// caller can relate a six-step sector to its angle without reaching into [`crate::foc`]).
#[inline]
pub fn code_base_angle(code: u8) -> u16 {
    BASE_ANGLE[(code & 0x7) as usize]
}
