//! The FOC math layer: the shared fixed-point primitives, the shared hall front-end, and the
//! torque-mode FOC arm.
//!
//! The primitives and the transforms are recovered bit-exact from the archived implementation of
//! the stock contract (`specs/commutation.md`, Provenance): the angle constants, the stock
//! `MAC`/`RND` rounding forms, the section-7 sine/cosine table + quadrant-folded lookup, the
//! current conditioning and its cal gate, Clarke, both Park transforms, the circular limit and
//! SVPWM. The arm that runs them is NOT the stock one: [`foc_step`] closes two current loops, the
//! q axis on a reference the demand word scales and the d axis on zero, so the demand word is a
//! current and therefore a torque (`specs/foc-torque.md`; the stock arm's open-loop d-axis ramp and
//! zero-reference q-PI were deleted, section 6 there).
//!
//! Q15 here means a RAW `i16` scaled +/-1.0 = +/-32767 (see the crate doc for why there is no
//! typed view).

use base::lane::Lane;
use base::pi::{pi_step, PiGains};
use base::TORQUE_FULL_SCALE;
use core::sync::atomic::{AtomicI32, Ordering};

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

// ============================================================================================
// Section 8: phase-current zero-offset calibration and per-period current
// ============================================================================================

/// Lower bound of the offset acceptance window (inclusive).
pub const CAL_WINDOW_LO: u16 = 0x7531;
/// Span of the offset acceptance window: offset must satisfy `(offset - 0x7531) < 0x1193`.
pub const CAL_WINDOW_SPAN: u16 = 0x1193;
/// Upper bound of the offset acceptance window (exclusive).
pub const CAL_WINDOW_HI: u16 = CAL_WINDOW_LO + CAL_WINDOW_SPAN; // 0x86C4

/// Accumulate one 16-conversion offset calibration for a single channel. Per conversion the
/// reference adds `sample >> 3`; with 16 samples this yields a 2x sum of the average. Returns the
/// 16-bit accumulated offset (wrapping).
#[inline]
pub fn calibrate_offset(samples: &[u16; 16]) -> u16 {
    let mut acc: u16 = 0;
    for &s in samples.iter() {
        acc = acc.wrapping_add(s >> 3);
    }
    acc
}

/// Range-check an accumulated offset: `(offset - 0x7531) < 0x1193` (unsigned), i.e. offset in
/// `[0x7531, 0x86C4)`.
#[inline]
pub fn offset_in_window(offset: u16) -> bool {
    offset.wrapping_sub(CAL_WINDOW_LO) < CAL_WINDOW_SPAN
}

/// The per-period phase current consumed by the Clarke transform: `current = offset - 2*sample`,
/// saturated to [-0x7FFF, +0x7FFF] (the -0x8000 sentinel maps to -0x7FFF). The ADC delivers
/// left-aligned samples, so `offset` is 2x the zero-current reading and the live sample is doubled.
#[inline]
pub fn current_from_adc(offset: u16, sample: u16) -> i16 {
    let v = offset as i32 - 2 * sample as i32;
    sat16(v)
}

/// Clarke constant: Q15 ~= 1/sqrt(3) = 0.5773.
pub const CLARKE_A: i32 = 0x49E6; // 18918
/// Clarke constant: Q15 ~= 2/sqrt(3) = 1.1547.
pub const CLARKE_B: i32 = 0x93CC; // 37836

/// The stator-frame vector from the Clarke transform.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clarke {
    /// alpha = iA unchanged (the raw phase sample).
    pub alpha: i16,
    /// beta = MAC(-(iA*0x49E6 + iB*0x93CC)).
    pub beta: i16,
}

/// Section 6 step 2a: Clarke transform of the two offset-corrected phase currents.
/// `alpha = iA`; `beta = MAC( -( iA*0x49E6 + iB*0x93CC ) )`. Only beta is saturated.
#[inline]
pub fn clarke(i_a: i16, i_b: i16) -> Clarke {
    let acc = -(i_a as i32 * CLARKE_A + i_b as i32 * CLARKE_B);
    Clarke {
        alpha: i_a,
        beta: mac_q15(acc),
    }
}

// ============================================================================================
// Section 6 step 2b / step 6: forward and inverse Park
//
// The frame (`specs/foc-torque.md` section 3): the slot ALONG `theta` is the torque axis, `iq` /
// `vq`, and the other slot is `id` / `vd`, positive 90 degrees BEHIND `theta` (where the textbook
// d axis sits, lagging q; `park_forward_reads_iq_along_theta_and_id_a_quarter_turn_behind`, and
// `vd` drives the same direction). The stock code called these the other
// way round ("d" along theta), about 90 degrees from the textbook convention, and drove its torque
// on the slot it called d; the arithmetic is the stock's, bit for bit, and only the names moved.
// Why the along-theta slot is torque: the stock Clarke's beta is the negative of the textbook
// beta, so a command on that slot with the other at zero is a stator vector at `theta` exactly
// (`the_two_paths_drive_the_same_vector`); and the 2026-10-20 sine sweep found the current
// minimum where this convention predicts, within +-5.6 degrees (`specs/bench-evidence/2026-10-20/
// sine-phase/RECORD.md`): at no load the stator voltage is the back-EMF, which lies on the
// textbook q axis, so `theta` is the q axis.
// ============================================================================================

/// Forward Park: rotate the stator vector `(alpha, beta)` into the rotor frame at `theta`.
/// Returns `(iq, id)`: `iq = MAC(alpha*cos - beta*sin)`, the current along `theta` (the torque
/// axis), and `id = MAC(alpha*sin + beta*cos)`, the current along the direction 90 degrees behind
/// it.
#[inline]
pub fn park_forward(alpha: i16, beta: i16, theta: u16) -> (i16, i16) {
    let (s, c) = lookup_sincos(theta);
    let iq = mac_q15(alpha as i32 * c as i32 - beta as i32 * s as i32);
    let id = mac_q15(alpha as i32 * s as i32 + beta as i32 * c as i32);
    (iq, id)
}

/// Inverse Park: rotate the rotor-frame command `(vq, vd)` back into the stator frame at `theta`,
/// in the order [`park_forward`] returns its pair. Returns `(alpha, beta)` where
/// `alpha = RND(vq*cos + vd*sin)`, `beta = RND(vd*cos - vq*sin)`. The RND form truncates to 16
/// bits (WRAPS on overflow, no saturate; defined behavior).
#[inline]
pub fn park_inverse(vq: i16, vd: i16, theta: u16) -> (i16, i16) {
    let (s, c) = lookup_sincos(theta);
    let alpha = rnd_q15(vq as i32 * c as i32 + vd as i32 * s as i32);
    let beta = rnd_q15(vd as i32 * c as i32 - vq as i32 * s as i32);
    (alpha, beta)
}

// ============================================================================================
// Section 6 step 5 / 6.2: circular magnitude limit
// ============================================================================================

/// Squared-magnitude threshold for the circular limit (sqrt ~= 32111, ~0.980 of Q15 FS).
pub const CIRC_THRESH: u32 = 0x3D75_9621; // 1_031_116_321

/// The 67-entry Q15 circular-limit gain table, indexed by `((sq>>24) - 0x3D) & 0xFF`.
pub static CIRC_GAIN: [i16; 67] = [
    32494, 32360, 32096, 31839, 31587, 31342, 31102, 30868, 30639, 30415, //
    30196, 29981, 29771, 29565, 29464, 29265, 29069, 28878, 28690, 28506, //
    28325, 28148, 27974, 27803, 27635, 27470, 27309, 27229, 27071, 26916, //
    26764, 26614, 26467, 26322, 26180, 26039, 25901, 25766, 25632, 25500, //
    25435, 25307, 25180, 25055, 24932, 24811, 24692, 24574, 24458, 24343, //
    24230, 24119, 24009, 23901, 23848, 23741, 23637, 23533, 23431, 23331, //
    23231, 23133, 23036, 22941, 22846, 22753, 22661,
];

/// Section 6 step 5: circular magnitude limit on the rotor-frame command `(vq, vd)`.
/// `sq = vq^2 + vd^2`; if `sq <= THRESH` pass through, else scale both components by the table
/// gain (round-and-shift, no saturate). Symmetric in its two arguments.
#[inline]
pub fn circular_limit(vq: i16, vd: i16) -> (i16, i16) {
    let sq = (vq as i32 * vq as i32 + vd as i32 * vd as i32) as u32;
    if sq <= CIRC_THRESH {
        return (vq, vd);
    }
    let idx = (((sq >> 24) as i32 - 0x3D) & 0xFF) as usize;
    // The table covers exactly the 67 over-threshold bins; idx is in range for any sq > THRESH that
    // fits in 32 bits (max vq^2+vd^2 = 2*32767^2 = 0x7FFE0002, >>24 = 0x7F, idx <= 0x42 = 66).
    let g = CIRC_GAIN[idx.min(66)] as i32;
    // Scale: vq' = rnd_q15(g*vq), vd' = rnd_q15(g*vd), the section-6.2 round-and-shift form.
    (rnd_q15(g * vq as i32), rnd_q15(g * vd as i32))
}

// ============================================================================================
// Section 9: SVPWM (sector selection and three duty cycles)
// ============================================================================================

/// SVPWM beta scale.
pub const SVPWM_BETA: i32 = 9000;
/// SVPWM sqrt(3)*alpha scale (15588/9000 = sqrt(3)).
pub const SVPWM_ALPHA: i32 = 0x3CE4; // 15588
/// Half-period centering constant (0x465 = 1125, half of ARR 2250).
pub const SVPWM_CENTER: i32 = 0x465;

/// Rounded arithmetic right-shift by 17 (toward zero): `(x + ((u32)(x>>31) >> 15)) >> 17`.
#[inline]
pub fn rsh17(x: i32) -> i32 {
    let bias = (((x >> 31) as u32) >> 15) as i32;
    x.wrapping_add(bias) >> 17
}

/// Rounded arithmetic right-shift by 18 (toward zero): `(x + ((u32)(x>>31) >> 14)) >> 18`.
#[inline]
pub fn rsh18(x: i32) -> i32 {
    let bias = (((x >> 31) as u32) >> 14) as i32;
    x.wrapping_add(bias) >> 18
}

/// The result of SVPWM: the chosen sector and the three computed compare numbers (base, c1, c2),
/// each masked to 16 bits. Their (CH1, CH2, CH3) permutation per sector is a remaining
/// phase-ordering degree of freedom confirmed on the bench (section 16).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Svpwm {
    pub sector: u8,
    pub base: u16,
    pub c1: u16,
    pub c2: u16,
}

/// Section 9.1: sector selection from the stator command `(alpha, beta)`.
#[inline]
pub fn svpwm_sector(alpha: i16, beta: i16) -> u8 {
    let bterm = beta as i32 * SVPWM_BETA; // 9000*beta
    let aterm = alpha as i32 * SVPWM_ALPHA; // 15588*alpha
    let p = (-bterm + aterm) / 2;
    let q = (-bterm - aterm) / 2;
    // "beta term <= 0" means (9000*beta == 0) || (-9000*beta < 0).
    let beta_term_le0 = (bterm == 0) || (-bterm < 0);
    if p < 0 && q < 0 {
        5
    } else if p < 0 {
        // q >= 0
        if beta_term_le0 {
            4
        } else {
            3
        }
    } else if q < 0 {
        // p >= 0
        if beta_term_le0 {
            6
        } else {
            1
        }
    } else {
        2
    }
}

/// Section 9.1 + 9.2: sector selection and the three per-sector compare numbers.
#[inline]
pub fn svpwm(alpha: i16, beta: i16) -> Svpwm {
    let sector = svpwm_sector(alpha, beta);
    let b = -(beta as i32 * SVPWM_BETA); // B = -9000*beta
    let aterm = alpha as i32 * SVPWM_ALPHA;
    let p = (b + aterm) / 2;
    let q = (b - aterm) / 2;

    let (base, c1, c2) = match sector {
        1 | 4 => {
            let base = rsh18((b - q) + SVPWM_BETA) + SVPWM_CENTER;
            let c1 = base + rsh17(q);
            let c2 = c1 - rsh17(b);
            (base, c1, c2)
        }
        2 | 5 => {
            let base = rsh18((p - q) + SVPWM_BETA) + SVPWM_CENTER;
            let c1 = base + rsh17(q);
            let c2 = base - rsh17(p);
            (base, c1, c2)
        }
        // sectors 3, 6: note 9000*beta = -B
        _ => {
            let base = rsh18(-b + SVPWM_BETA + p) + SVPWM_CENTER;
            let c2 = base - rsh17(p);
            let c1 = rsh17(b) + c2;
            (base, c1, c2)
        }
    };
    Svpwm {
        sector,
        base: (base & 0xFFFF) as u16,
        c1: (c1 & 0xFFFF) as u16,
        c2: (c2 & 0xFFFF) as u16,
    }
}

// ============================================================================================
// Section 5: hall acquisition, debounce, commutation, interpolation, speed
// ============================================================================================

/// The per-line hall debounce window, as a TIME.
///
/// The recovered constant is a PERIOD COUNT, 150 (0x96), read out of the stock commutation
/// struct's `.data` initialiser at RAM `0x20000280` (`~/dev/BalanceAgain/findings/
/// commutation_constants.md`, both offroad images). A period count is meaningless without the rate
/// it is decremented at, and the stock rate is documented three independent ways: the decrement
/// lives inside the hall-read function itself (`FUN_080070b4` in the board20 decompile, which both
/// reloads and decrements), that function is called once per PWM period from the 16 kHz ADC_CMP
/// ISR, and `~/dev/Declassyfied/spec/commutation.md` 3 states "the loop rate equals this PWM
/// frequency", 16.0 kHz. So the recovered 150 periods IS 9,375 us, and this constant is that same
/// window written in the unit that survives a rate change: at the reference 16 kHz
/// [`hall_debounce_periods`] returns exactly the recovered 150, and at any other period rate it
/// returns the count that preserves the TIME rather than silently rescaling the window.
///
/// What the window is for: the filter rejects a line that toggles again before ANY other line has
/// moved, which is what bounce looks like and what a real commutation step never looks like (see
/// [`HallDebounce::step`]). It is therefore not a speed limit, and 9.375 ms is generous against a
/// hall latch's tens-of-microseconds chatter at no cost in tracked speed.
pub const HALL_DEBOUNCE_US: u32 = 9_375;

/// The debounce window ([`HALL_DEBOUNCE_US`]) in PWM periods at a given period-ISR rate: the value
/// a line's lockout counter is reloaded with, since the lockout is decremented once per period.
/// Rounded UP so the window is never shorter than specified. Saturating at `i16::MAX` because the
/// lockout field is an `i16`; the firmware's `HALL_DEBOUNCE_WINDOW_SURVIVES_THE_PERIOD_RATE`
/// compile-time assertion checks the round-trip, so a saturated value cannot ship silently.
pub const fn hall_debounce_periods(period_hz: u32) -> i16 {
    let periods = (HALL_DEBOUNCE_US as u64 * period_hz as u64).div_ceil(1_000_000);
    if periods > i16::MAX as u64 {
        i16::MAX
    } else {
        periods as i16
    }
}

/// The window [`hall_debounce_periods`] actually yields at `period_hz`, back in microseconds. The
/// round-trip the firmware asserts against [`HALL_DEBOUNCE_US`].
pub const fn hall_debounce_window_us(period_hz: u32) -> u32 {
    if period_hz == 0 {
        return 0;
    }
    (hall_debounce_periods(period_hz) as u64 * 1_000_000 / period_hz as u64) as u32
}

/// The highest ELECTRICAL frequency this front end can track at a given period-ISR rate.
///
/// The bound is the sample rate, not the debounce: the halls are read once per period and there
/// are six sectors per electrical revolution, so above `period_hz / 6` a sector can pass between
/// two samples and go unseen. The debounce does not bind (see [`HallDebounce::step`]), which is
/// the whole point of the stock eligibility rule.
///
/// Mechanical rpm = `60 * electrical_hz / pole_pairs`. At the reference 16 kHz that is 2,666 Hz
/// electrical, or **10,666 rpm** on an ASSUMED 15 pole pairs, roughly ten times the ~1,000 rpm top
/// of the machine's working range. 15 is what the fleet's 6.5 inch hubs are believed to carry and
/// has been counted on none of them; a board's real count is recorded in `store::MOTOR_POLE_PAIRS`,
/// which defaults to unset. A hub with MORE pole pairs reaches this ceiling at a proportionally
/// lower mechanical rpm, so the headroom above shrinks in the same ratio.
pub const fn tracked_electrical_hz_ceiling(period_hz: u32) -> u32 {
    period_hz / 6
}

/// The electrical frequency the front end MUST be able to track, as a floor for
/// [`tracked_electrical_hz_ceiling`]. The requirement was SIZED from an assumed 15 pole pairs, on
/// which 250 Hz is 1,000 mechanical rpm, ~31 km/h on a 6.5" wheel: the top of the machine's
/// working range. A board's real count is recorded in `store::MOTOR_POLE_PAIRS` and no board
/// carries one yet, and a hub with more pole pairs puts that same 250 Hz at a proportionally lower
/// rpm. The constant stays compiled and fixed regardless: it is a build-time floor on what the
/// front end must track, not a per-board conversion, and a sample-rate bound is not something a
/// staged pole-pair count can move. Asserted at compile time by the firmware so a future change to
/// the PWM period or the core clock that would re-cap the motor fails the build rather than a
/// bench session.
pub const MIN_TRACKED_ELECTRICAL_HZ: u32 = 250;

/// The 6-state base electrical-angle table. Index by hall code 1..6 (index 0 unused).
/// These are the bit-exact recovered anchors, bench-confirmed against live stock.
// In RAM, not flash: this is a 16 kHz hot-path table and the GD32F1x0 charges a flash DATA
// read above its 32 KiB zero-wait window (crates/firmware/memory.x). RAM is a fixed 1 cycle
// on both families and costs 58 B of `.data` for the five tables together.
#[cfg_attr(target_arch = "arm", link_section = ".data")]
pub static BASE_ANGLE: [u16; 8] = [
    0,      // 0: invalid
    0x9554, // 1
    0xEAAB, // 2
    0xBFFF, // 3
    0x4000, // 4
    0x6AAA, // 5
    0x1556, // 6
    0,      // 7: invalid
];

/// Forward-next neighbor (dir = +1) per hall code (index 0/7 unused).
// In RAM, not flash: this is a 16 kHz hot-path table and the GD32F1x0 charges a flash DATA
// read above its 32 KiB zero-wait window (crates/firmware/memory.x). RAM is a fixed 1 cycle
// on both families and costs 58 B of `.data` for the five tables together.
#[cfg_attr(target_arch = "arm", link_section = ".data")]
static FWD_NEXT: [u8; 8] = [0, 3, 6, 2, 5, 1, 4, 0];
/// Reverse-next neighbor (dir = -1) per hall code (index 0/7 unused).
// In RAM, not flash: this is a 16 kHz hot-path table and the GD32F1x0 charges a flash DATA
// read above its 32 KiB zero-wait window (crates/firmware/memory.x). RAM is a fixed 1 cycle
// on both families and costs 58 B of `.data` for the five tables together.
#[cfg_attr(target_arch = "arm", link_section = ".data")]
static REV_NEXT: [u8; 8] = [0, 5, 3, 1, 6, 4, 2, 0];

/// Per-line hall debounce state.
// Deviation from the archived code (named): the derived `Default` is dropped. It constructed a
// reload of 0 (debounce disabled), contradicting `new()`'s reference value; nothing consumed it,
// and a misleading constructor is a trap, not API. `Default` is not reimplemented either: the
// reload is derived from the period-ISR rate, and there is no default rate to derive it from.
#[derive(Clone, Copy, Debug)]
pub struct HallDebounce {
    /// Stored (debounced) level per line (0/1).
    pub level: [u8; 3],
    /// Per-line lockout countdown (signed, decremented each period while > 0).
    pub lockout: [i16; 3],
    /// Per-line "recently changed" marker.
    pub changed: [bool; 3],
    /// The lockout reload value, derived from the period-ISR rate by
    /// [`hall_debounce_periods`] so it expresses [`HALL_DEBOUNCE_US`] at whatever rate this
    /// board actually runs.
    pub reload: i16,
}

impl HallDebounce {
    /// New debounce state whose lockout reload expresses [`HALL_DEBOUNCE_US`] at `period_hz`, the
    /// rate at which [`HallDebounce::step`] will be called (the PWM period rate).
    pub fn new(period_hz: u32) -> Self {
        Self {
            level: [0; 3],
            lockout: [0; 3],
            changed: [false; 3],
            reload: hall_debounce_periods(period_hz),
        }
    }

    /// Section 5.1: feed one period of raw hall levels (each 0/1) and return the debounced 3-bit
    /// code `A | (B<<1) | (C<<2)`.
    #[allow(clippy::needless_range_loop)] // recovered loop shape: parallel arrays indexed by line
    pub fn step(&mut self, raw: [u8; 3]) -> u8 {
        for line in 0..3 {
            // The stock eligibility rule, a DISJUNCTION: a line may register an edge if EITHER it
            // is not the line that most recently changed OR its own lockout has drained. Because
            // an accepted edge clears the other two lines' markers, and because consecutive
            // sectors of a valid commutation sequence always move DIFFERENT lines, the lockout
            // never gates an edge during rotation. What it does gate is a line toggling again
            // before any other line has moved, which is what bounce looks like.
            //
            // `~/dev/Declassyfied/spec/commutation.md` 5.1 ("eligible ... only if EITHER it is not
            // the currently-locked line OR its lockout countdown has reached 0"), the clean-room
            // `~/dev/Declassyfied/firmware/commutation.c:275`, and `FUN_080070b4` in
            // `~/dev/BalanceAgain/analysis/boards/board20_master_GD32F130/decompiled.c:7829` all
            // state it the same way.
            //
            // This crate previously tested `self.lockout[line] == 0` alone. That drops the
            // disjunct and converts a bounce filter into a hard per-line rate limit of one edge
            // per `reload` periods, which at the recovered 150 periods / 16 kHz caps the tracked
            // electrical period at 18.75 ms: ~53 Hz electrical, ~210 rpm on a 15-pole-pair hub.
            // Bench, 2026-08-12: the motor stopped accelerating at ~40 % throttle with current
            // pinned at the supply limit and `invalid_dwell` at zero throughout, which is exactly
            // what silently dropped hall edges look like from outside.
            let eligible = !self.changed[line] || self.lockout[line] == 0;
            if eligible && raw[line] != self.level[line] {
                self.level[line] = raw[line];
                self.lockout[line] = self.reload;
                // Set this line's marker, clear the other two (only one hall changes at a time).
                self.changed = [false; 3];
                self.changed[line] = true;
            }
        }
        // After the edge tests, decrement any nonzero lockout countdown by 1 (clamped at 0).
        for line in 0..3 {
            if self.lockout[line] > 0 {
                self.lockout[line] -= 1;
            }
        }
        self.level[0] | (self.level[1] << 1) | (self.level[2] << 2)
    }
}

/// Hall-fault threshold: fault when `dwell * 250 > 16000` (i.e. dwell > 64 consecutive invalid).
pub const HALL_FAULT_DWELL_MUL: u32 = 250;
pub const HALL_FAULT_DWELL_LIMIT: u32 = 16000;

/// Inter-edge interval blend weights (section 5.3): `(prev*15 + new*85)/100`.
pub const BLEND_PREV: u32 = 0x0F; // 15
pub const BLEND_NEW: u32 = 0x55; // 85
/// Speed measurement window length (periods).
pub const SPEED_WINDOW: u16 = 0x140; // 320
/// Edge-rate scratch constant (section 5.4), taken as the positive integer 46081.
pub const EDGE_RATE_CONST: i32 = 0xB401; // 46081
/// Interpolation step words (section 5.4).
pub const STEP_NEG: u16 = 0xEAAB; // = -0x1555 ~= -30 deg
pub const STEP_POS: u16 = 0x1555; // = +30 deg
/// 60 deg as the magnitude-gate constant for the interpolation sub-accumulator (10922.0 as i32).
pub const ACC_GATE: i32 = 10922;
/// The 200-period interpolation gate.
pub const INTERP_GATE: u32 = 200;
/// The period-based wheel rate's numerator (`specs/control.md` (o)): `RATE_Q_UNIT * SPEED_WINDOW`,
/// so that an interval of exactly one window between two same-direction edges reads one count,
/// which is `RATE_Q_UNIT`. Dividing it by an interval in periods gives the rate in 1/4096 of an
/// edge per window, the unit the outer speed loop's filter already carries.
pub const RATE_Q_UNIT: i32 = 4096;
pub const RATE_Q_NUM: i32 = RATE_Q_UNIT * SPEED_WINDOW as i32; // 1,310,720

/// The commutation + interpolation + speed state (section 5), per motor.
/// (The archived hand-written all-zeros `Default` impl is the derived one; derived here.)
#[derive(Clone, Copy, Debug, Default)]
pub struct Commutation {
    /// The published rotor electrical angle (16-bit wrapping).
    pub angle: u16,
    /// Previous valid hall code (for direction).
    pub prev_code: u8,
    /// Previous assembled code (any change triggers the interval capture / interpolation reset).
    pub prev_any_code: u8,
    /// Stored direction (+1 / -1; 0 before the first valid neighbor step).
    pub dir: i32,
    /// Signed edge accumulator (incremented on +1, decremented on -1).
    pub edge_acc: i32,
    /// ABSOLUTE edge count since this struct was created: every counted neighbour transition adds
    /// 1, whichever way it went, so a reversing wheel accumulates like a forward one. It is the
    /// odometer's raw material (`specs/odometer.md`), and it is a separate word from
    /// [`Commutation::edge_acc`] because that one is a RATE: it is latched into
    /// [`Commutation::speed`] and cleared every window, and a window's net sum is not the sum of
    /// its edges' magnitudes whenever the wheel reversed inside it.
    ///
    /// WRAPPING, and the consumer folds it as deltas, so the wrap costs nothing. At a hub's
    /// working edge rate it is about 24 days of continuous rotation away.
    pub edges_total: u32,
    /// SIGNED edge position since this struct was created (`specs/state-feedback.md` section 3):
    /// every counted neighbour transition adds its direction, +1 or -1, in the counter's own sense
    /// (the side mirror is the consumer's), and nothing ever clears it. It is the third word
    /// beside [`Commutation::edge_acc`], which is a rate cleared every window, and
    /// [`Commutation::edges_total`], which adds 1 whichever way the wheel turned: neither is a
    /// position.
    ///
    /// WRAPPING, and the consumer folds it as deltas, as it does `edges_total`.
    pub pos_edges: i32,
    /// Latched raw speed (signed edge count per 320-period window).
    pub speed: i32,
    /// Speed measurement window counter.
    pub window: u16,
    /// The period-based wheel rate (`specs/control.md` (o)): `rate_dir * RATE_Q_NUM / interval`,
    /// signed in the counter's own sense, in 1/4096 of an edge per window. Set on the edge that
    /// completes a pair of SAME-direction edges from the periods between them; decays as `1/t`
    /// between edges once the time since the last edge exceeds that interval; reads 0 from the
    /// edge that reverses direction until the next same-direction pair, so a wheel rocking
    /// between two hall codes reads exactly 0 instead of a large speed of alternating sign.
    pub rate_q: i32,
    /// The direction (+1 / -1) of the last counted edge, 0 before the first. The reversal test
    /// for [`Self::rate_q`]; separate from [`Self::dir`], which the angle interpolation owns.
    pub rate_dir: i32,
    /// The interval, in periods, the current [`Self::rate_q`] was computed from. The decay
    /// between edges starts once `since_change` passes it.
    pub rate_interval: u32,
    /// Periods since the last hall-code change (sticks at 0xFFFF); doubles as the interval counter
    /// read as `new_interval` on the next change and as the 5.4 interpolation gate.
    pub since_change: u32,
    /// Previously captured inter-edge interval.
    pub prev_interval: u16,
    /// Blended inter-edge interval.
    pub interval_blend: u16,
    /// Per-period angle increment (`dir * 0x2AAA / interval_blend`).
    pub increment: i32,
    /// Interpolation sub-accumulator.
    pub acc: i32,
    /// Step/lead word (held across periods).
    pub step: u16,
    /// In-window flag (counter < 200 since last change).
    pub in_window: bool,
    /// Invalid-code dwell counter.
    pub invalid_dwell: u32,
    /// Hall-fault flag (raised when dwell > 64).
    pub hall_fault: bool,
}

impl Commutation {
    pub fn new() -> Self {
        Self::default()
    }

    /// Section 5.2-5.4: one period of commutation. `code` is the debounced 3-bit hall code (0..7).
    /// Updates direction, speed window, the interval blend, the interpolation terms, and rewrites
    /// the published angle. Returns the published rotor angle.
    pub fn step(&mut self, code: u8) -> u16 {
        let valid = (1..=6).contains(&code);

        // The periods-since-change counter increments every period (sticks at 0xFFFF). With one
        // period per code it reads 1 at the next change (incremented once since the prior reset),
        // which is the spec's `new_interval` value for the one-period-per-code check.
        if self.since_change < 0xFFFF {
            self.since_change += 1;
        }

        // --- Section 5.2: base-angle write (valid codes only, FIRST each period) ---
        if valid {
            self.angle = BASE_ANGLE[code as usize];
            self.invalid_dwell = 0;
        } else {
            // Invalid: do not rewrite the angle from the table; increment the dwell counter.
            self.invalid_dwell = self.invalid_dwell.saturating_add(1);
            if self.invalid_dwell * HALL_FAULT_DWELL_MUL > HALL_FAULT_DWELL_LIMIT {
                self.hall_fault = true;
            }
        }

        // --- Section 5.3: direction (valid neighbor transitions only) ---
        let code_changed = code != self.prev_any_code;
        if valid && self.prev_code >= 1 && self.prev_code <= 6 && code != self.prev_code {
            if FWD_NEXT[self.prev_code as usize] == code {
                self.dir = 1;
            } else if REV_NEXT[self.prev_code as usize] == code {
                self.dir = -1;
            }
            // Any other transition (a skipped state) is treated as noise / not counted.
            if FWD_NEXT[self.prev_code as usize] == code {
                self.edge_acc += 1;
                self.edges_total = self.edges_total.wrapping_add(1);
                self.pos_edges = self.pos_edges.wrapping_add(1);
                self.count_rate_edge(1);
            } else if REV_NEXT[self.prev_code as usize] == code {
                self.edge_acc -= 1;
                self.edges_total = self.edges_total.wrapping_add(1);
                self.pos_edges = self.pos_edges.wrapping_sub(1);
                self.count_rate_edge(-1);
            }
        }
        if valid {
            self.prev_code = code;
        }

        // The period-based rate's decay between edges (`specs/control.md` (o)): once the time
        // since the last edge exceeds the interval the estimate came from, the estimate is
        // `RATE_Q_NUM / since_change`, so a stopping wheel reads a rate falling as `1/t` instead
        // of holding its last reading. Skipped while `since_change <= rate_interval` (steady
        // rotation: the estimate is unchanged) and while the estimate is already 0 (after a
        // reversal, or before the first pair), and once `since_change` has stuck at 0xFFFF (the
        // value would not change), so the divide runs only on a coast-down. On the period that
        // carries an edge the block above has just set `rate_interval` to this `since_change`,
        // so the decay is skipped and the fresh estimate stands.
        if self.rate_q != 0 && self.since_change > self.rate_interval && self.since_change < 0xFFFF
        {
            self.rate_q = self.rate_dir * RATE_Q_NUM / self.since_change as i32;
        }

        // --- Section 5.3: speed measurement window ---
        self.window = self.window.wrapping_add(1);
        if self.window > SPEED_WINDOW {
            self.window = 0;
            self.speed = self.edge_acc;
            self.edge_acc = 0;
        }

        // --- Section 5.3 / 5.4: on every hall-code change, capture interval + reset interp ---
        if code_changed {
            // `new_interval` is the periods-since-change counter read BEFORE it is cleared.
            let new_interval = (self.since_change & 0xFFFF) as u16;
            // interval_blend = (prev*15 + new*85)/100
            let blend =
                (self.prev_interval as u32 * BLEND_PREV + new_interval as u32 * BLEND_NEW) / 100;
            self.interval_blend = blend as u16;
            self.prev_interval = new_interval;

            // Per-period increment: dir * (0x2AAA / interval_blend), 0 on a zero divisor.
            self.increment = if self.interval_blend == 0 {
                0
            } else {
                self.dir * (SECTOR_ANGLE as i32 / self.interval_blend as i32)
            };
            // (Edge-rate scratch term is computed but unused by the per-period angle math.)
            let _edge_rate: i16 = if new_interval == 0 {
                0
            } else {
                (self.dir * EDGE_RATE_CONST / new_interval as i32) as i16
            };

            // The periods-since-change counter and the sub-accumulator are both cleared to 0.
            self.since_change = 0;
            self.acc = 0;
        }
        self.prev_any_code = code;

        // --- Section 5.4: every period, after the base write ---
        if self.since_change < INTERP_GATE {
            self.in_window = true;
            // Step/lead word selection (sign pairing is the reverse of the naive guess).
            if self.increment >= 3 {
                self.step = STEP_NEG;
            } else if self.increment <= -3 {
                self.step = STEP_POS;
            }
            // -2..=+2: hold the previous step value.

            // Sub-accumulator magnitude gate: only add while |acc| < 10922 (tests BEFORE adding).
            if iabs32(self.acc) < ACC_GATE {
                self.acc = self.acc.wrapping_add(self.increment);
            }

            // Angle update: angle = angle + acc + step (modular 16-bit).
            self.angle = self
                .angle
                .wrapping_add(self.acc as u16)
                .wrapping_add(self.step);
        } else {
            self.step = 0;
            self.in_window = false;
            // Neither the accumulator nor the angle is advanced this period.
        }

        self.angle
    }
}

impl Commutation {
    /// One counted neighbour edge in direction `dir` (+1 / -1) for the period-based rate
    /// (`specs/control.md` (o)). `since_change` still holds the periods since the previous
    /// hall-code change (the edge block that clears it runs after this). An edge in the same
    /// direction as the last one completes a pair and publishes `RATE_Q_NUM / interval`; one that
    /// differs is the first edge of a new direction, publishes 0 and arms the next.
    #[inline]
    fn count_rate_edge(&mut self, dir: i32) {
        if dir == self.rate_dir {
            let interval = self.since_change.max(1);
            self.rate_interval = interval;
            self.rate_q = dir * RATE_Q_NUM / interval as i32;
        } else {
            self.rate_dir = dir;
            self.rate_interval = 0;
            self.rate_q = 0;
        }
    }
}

#[inline]
fn iabs32(x: i32) -> i32 {
    if x < 0 {
        -x
    } else {
        x
    }
}

// ============================================================================================
// The shared hall/sector/angle front-end (hoisted so all commutation methods reuse it)
// ============================================================================================

/// The shared rotor front-end: hall debounce ([`HallDebounce`]) plus the 6-state commutation /
/// inter-edge angle interpolation / speed estimator ([`Commutation`]). Hoisted out of [`foc_step`]
/// so six-step, sinusoidal, and FOC all reuse the SAME hall+angle path (no duplicated hall numbering
/// or angle math). Stepping it is byte-for-byte the original FOC sequence: `hall.step(raw)` then
/// `comm.step(code)`, so the FOC arm stays bit-preserved.
#[derive(Clone, Copy, Debug)]
pub struct RotorFrontEnd {
    /// Per-line hall debounce.
    pub hall: HallDebounce,
    /// The 6-state commutation + interpolation + speed estimator.
    pub comm: Commutation,
}

/// The published rotor snapshot from one front-end step: the debounced hall code, the interpolated
/// electrical angle, the latched speed, the in-interpolation-window flag, and the hall-fault flag.
/// The angle is what the sine / FOC modulation indexes; the code is what the six-step table indexes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct RotorState {
    /// The debounced 3-bit hall code (0..7; 1..6 valid).
    pub code: u8,
    /// The interpolated 16-bit electrical angle (65536 / rev).
    pub angle: u16,
    /// The latched signed speed (edge count per window).
    pub speed: i32,
    /// True while inside the 200-period interpolation window (rotor recently moved).
    pub in_window: bool,
    /// True once a persistent invalid-hall fault has latched.
    pub hall_fault: bool,
}

impl RotorFrontEnd {
    /// A fresh front-end with a cleared commutation estimator and a hall debounce sized for
    /// `period_hz`, the rate at which [`RotorFrontEnd::step`] will be called.
    pub fn new(period_hz: u32) -> Self {
        Self {
            hall: HallDebounce::new(period_hz),
            comm: Commutation::new(),
        }
    }

    /// One period of the shared front-end: debounce the raw halls, then commutate / interpolate.
    /// This is the exact sequence [`foc_step`] used inline, so behavior is unchanged. Returns the
    /// rotor snapshot every method consumes.
    #[inline]
    pub fn step(&mut self, raw_hall: [u8; 3]) -> RotorState {
        let code = self.hall.step(raw_hall);
        let angle = self.comm.step(code);
        RotorState {
            code,
            angle,
            speed: self.comm.speed,
            in_window: self.comm.in_window,
            hall_fault: self.comm.hall_fault,
        }
    }
}

// ============================================================================================
// Per-period FOC orchestration (the math the dispatch calls)
// ============================================================================================

/// The calibrated per-motor phase-current zero offsets, the FOC arm's STRUCTURAL cal gate
/// (spec "Current-sense conditioning and the FOC capability gate"): the fields are private and
/// the only constructor is [`PhaseOffsets::try_new`], which refuses an offset outside the
/// `[0x7531, 0x86C4)` window. A [`FocState`] needs a `PhaseOffsets`, so out-of-window offsets
/// cannot yield a runnable FOC, the refuses-run rule made unrepresentable rather than checked at
/// call sites. (Deviation from the archive, named: `MotorParams { pub offset_a, offset_b }` with
/// a `Default` of the reference measured offsets is replaced by this gated newtype; the reference
/// pair lives on as test data.)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhaseOffsets {
    a: u16,
    b: u16,
}

impl PhaseOffsets {
    /// Gate a calibrated offset pair: `Some` only if BOTH offsets are inside the acceptance
    /// window (`offset_in_window`); `None` refuses the calibration (and with it, FOC this boot).
    pub fn try_new(offset_a: u16, offset_b: u16) -> Option<Self> {
        if offset_in_window(offset_a) && offset_in_window(offset_b) {
            Some(Self {
                a: offset_a,
                b: offset_b,
            })
        } else {
            None
        }
    }

    /// Phase-A zero-current offset (in-window by construction).
    #[inline]
    pub fn a(&self) -> u16 {
        self.a
    }

    /// Phase-B zero-current offset (in-window by construction).
    #[inline]
    pub fn b(&self) -> u16 {
        self.b
    }
}

/// The per-motor `(base, c1, c2) -> (CH0, CH1, CH2)` SVPWM output permutation: the one stock
/// phase-ordering degree of freedom, per-motor config resolved on the bench (silicon queue),
/// defaulting to the direct order. (Deviation from the archive, named: the hot-path `DutyOrder`
/// fn pointer becomes data; `map[i]` selects which of (base, c1, c2) drives channel i.)
///
/// The selector is a [`Lane`], not a `u8`: a `u8` map admits a 4, so indexing `(base, c1, c2)`
/// with it made the compiler emit `panic_bounds_check` inside the 16 kHz `period_isr`, where
/// `panic-halt`'s spin leaves the bridge energized at the last duties with `MOE` set and nothing
/// able to take them down (`specs/panic-free.md`, requirement 2). With three-valued lanes the
/// permutation cannot express an out-of-range selector, so the check has nothing to prove. The
/// bench-resolved permutation stays per-motor DATA; when it arrives from the board model or the
/// store it is validated once, at construction, through [`Lane::from_index`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DutyOrder {
    map: [Lane; 3],
}

impl DutyOrder {
    /// The reference direct order: (CH0, CH1, CH2) = (base, c1, c2).
    pub const DIRECT: DutyOrder = DutyOrder {
        map: Lane::IDENTITY,
    };

    /// Apply the permutation to an SVPWM record, yielding the three channel compare counts.
    #[inline]
    pub fn apply(&self, s: Svpwm) -> [u16; 3] {
        let bcc = [s.base, s.c1, s.c2];
        self.map.map(|lane| lane.pick(&bcc))
    }
}

// ============================================================================================
// The two current PIs' gains (`specs/foc-torque.md` section 5)
// ============================================================================================

/// The start gains of the two current PIs, designed by pole-zero cancellation at a 300 Hz
/// crossover against the plant `specs/foc-torque.md` section 5 tabulates (`R` 0.24 ohm, `L`
/// 0.5 mH, so `tau = L / R` = 2.1 ms or 33 periods, and a DC gain of 1.0 current count per voltage
/// count at 29.6 V and 455 counts per amp).
///
/// - `kp` 4000 / 1024 = 3.91 voltage counts per current count of error: `omega_c * tau / K_dc`.
/// - `ki` 119 / 1024 = 0.116 per period: `kp * T_s / tau`, which puts the PI's zero on the plant's
///   pole (`Ti = tau`).
/// - `out_max` 32,767: the circular limit bounds the vector; this clamp bounds each axis.
/// - `int_max` 32,767 x 1024: an accumulator that cannot command past the output clamp.
///
/// Overflow, at the largest error two `i16` currents can make (65,534): `e * kp` is 2.6e8 and
/// `e * ki` 7.8e6, both inside `i32`, and the accumulator is bounded at 3.4e7. These are the bench's
/// starting point, not confirmed values: gate O5 of that spec measures the margin and the bake
/// folds the confirmed set into constants.
pub const FOC_GAINS_START: PiGains = PiGains {
    kp: 4000,
    kp_divisor: 1024,
    ki: 119,
    ki_divisor: 1024,
    out_max: 32767,
    int_max: 32767 * 1024,
};

/// The live bench knob over the current PIs' gains: [`PiGains`]' six words as `AtomicI32`, in the
/// same order (`repr(C)`), so a bench session retunes a RUNNING board with `hoverboard-cli write
/// <addr> <hex>` at the address `nm` gives for [`FOC_GAINS`], one word at a time.
///
/// Not store fields: the bench tunes them once per motor class, not per ride, and an arm does not
/// reset them (the `SINE_PHASE_OFFSET` bench knob's pattern, `specs/bench-evidence/2026-10-20/
/// sine-phase/RECORD.md`). The bake rule applies: a confirmed set becomes [`FOC_GAINS_START`] and
/// this static becomes plain constants, returning its 24 B of RAM and the six loads.
///
/// A word written mid-period can pair a new `kp` with an old `kp_divisor` for one period; that is
/// one period of a different gain, and [`pi_step`] is defined for every value either word can hold
/// (`base::pi`, "Defined for every input"), so a half-written set cannot panic the ISR.
#[repr(C)]
#[derive(Debug)]
pub struct FocGains {
    // One word per `PiGains` field, same name, same meaning.
    pub kp: AtomicI32,
    pub kp_divisor: AtomicI32,
    pub ki: AtomicI32,
    pub ki_divisor: AtomicI32,
    pub out_max: AtomicI32,
    pub int_max: AtomicI32,
}

impl FocGains {
    /// The knob initialised to a gain set.
    pub const fn new(g: PiGains) -> Self {
        Self {
            kp: AtomicI32::new(g.kp),
            kp_divisor: AtomicI32::new(g.kp_divisor),
            ki: AtomicI32::new(g.ki),
            ki_divisor: AtomicI32::new(g.ki_divisor),
            out_max: AtomicI32::new(g.out_max),
            int_max: AtomicI32::new(g.int_max),
        }
    }

    /// The gains in force this period: six `Relaxed` loads (about 12 cycles), read once per FOC
    /// period and shared by both axes.
    #[inline]
    pub fn load(&self) -> PiGains {
        PiGains {
            kp: self.kp.load(Ordering::Relaxed),
            kp_divisor: self.kp_divisor.load(Ordering::Relaxed),
            ki: self.ki.load(Ordering::Relaxed),
            ki_divisor: self.ki_divisor.load(Ordering::Relaxed),
            out_max: self.out_max.load(Ordering::Relaxed),
            int_max: self.int_max.load(Ordering::Relaxed),
        }
    }
}

/// The current PIs' gains, as the FOC arm reads them every period. Unmangled so the bench finds it
/// by name in the ELF; see [`FocGains`].
#[no_mangle]
pub static FOC_GAINS: FocGains = FocGains::new(FOC_GAINS_START);

// ============================================================================================
// The torque-mode arm (`specs/foc-torque.md` sections 2 to 5)
// ============================================================================================

/// The observation window the FOC sums are closed over, as a shift: 64 periods, one nominal
/// control tick. [`FocState::take_window`]'s mean is `sum >> FOC_WINDOW_SHIFT`; the period ISR
/// closes the window in the branch that closes the current limit's, and asserts its own window
/// length against this at build time (`firmware::motor::PERIODS_PER_TICK_SHIFT`), so the two
/// windows and the two divisors cannot come apart.
pub const FOC_WINDOW_SHIFT: u32 = 6;

/// The per-period observation sums the FOC arm keeps (`specs/foc-torque.md` section 7): measured
/// `iq` and `id`, and the `vq` commanded after the circular limit, one add each per period, closed
/// and restarted by [`FocState::take_window`].
///
/// A closed 64-period window of `i16` values cannot overflow an `i32` (64 x 32,768). The adds wrap
/// all the same, so a caller that never closes a window (a host test running the loop for
/// thousands of periods) is defined past `i32::MAX` rather than relying on the ISR's cadence
/// (`specs/panic-free.md`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FocSums {
    /// Sum of the measured current along `theta`, the torque axis.
    pub iq: i32,
    /// Sum of the measured current on the other axis (positive 90 degrees behind `theta`).
    pub id: i32,
    /// Sum of the q-axis voltage commanded, after the circular limit.
    pub vq: i32,
}

/// One closed observation window of the FOC arm ([`FocState::take_window`]): the three means and
/// the reference of the period that closed it, each an `i16` in the unit of its loop (current
/// counts for `iq`, `id` and `iq_ref`, voltage counts for `vq`). The period ISR publishes them
/// as `CTRL_OBS` words 48 (`id | iq << 16`) and 49 (`iq_ref | vq << 16`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FocWindow {
    /// Mean measured current along `theta`, the torque axis.
    pub iq: i16,
    /// Mean measured current on the other axis; the d loop's health is this reading zero.
    pub id: i16,
    /// Mean q-axis voltage commanded, after the circular limit: near 32,000 with `iq` under its
    /// reference is the voltage ceiling.
    pub vq: i16,
    /// The `iq` reference of the window's last period (0 if that period floated).
    pub iq_ref: i16,
}

/// The per-motor FOC records: the two current-loop accumulators, the reference's full scale, the
/// per-motor SVPWM output order, and the observation sums with the latched reference.
///
/// Constructible ONLY with a [`PhaseOffsets`] (the structural cal gate), which it keeps nothing
/// from: the currents arrive already conditioned, because the period ISR computes
/// [`current_from_adc`] on both phases every period for the current limit, and computing the same
/// pair again here was the stock arm's second copy. The shared rotor front-end is NOT here either
/// (the `Commutator` owns it and [`foc_step`] is handed the aligned angle).
///
/// Fresh on every build, which is the method-switch reset (`specs/commutation.md`, "The mode
/// model"): both accumulators, the sums and the latch start at zero.
#[derive(Clone, Copy, Debug)]
pub struct FocState {
    /// The q-axis PI's accumulator (the torque axis).
    q_acc: i32,
    /// The d-axis PI's accumulator.
    d_acc: i32,
    /// The `iq` reference a full-scale demand word asks for, in current counts
    /// ([`iq_reference`]).
    full_scale: i16,
    duty_order: DutyOrder,
    sums: FocSums,
    /// The `iq` reference of the last period (0 on a floated one).
    iq_ref: i16,
    /// The rotor-frame pair `(iq, id)` of the last period with both sensed phases fresh: what the
    /// PIs are fed on a held period (`specs/foc-torque.md` section 14).
    held_dq: (i16, i16),
}

impl FocState {
    /// Fresh FOC records for a motor whose calibration `_cal` was accepted, its SVPWM output
    /// order (pass [`DutyOrder::DIRECT`] until the bench resolves a board's permutation), and the
    /// reference's full scale in current counts. The integration layer derives `full_scale` from
    /// the staged current limit less the board's noise floor (`firmware::motor::rederive`), so it
    /// is non-negative there; a negative value would only invert the reference's sign.
    pub fn new(_cal: PhaseOffsets, duty_order: DutyOrder, full_scale: i16) -> Self {
        Self {
            q_acc: 0,
            d_acc: 0,
            full_scale,
            duty_order,
            sums: FocSums::default(),
            iq_ref: 0,
            held_dq: (0, 0),
        }
    }

    /// The rotor-frame pair of the last period with both sensed phases fresh.
    #[inline]
    pub fn held_dq(&self) -> (i16, i16) {
        self.held_dq
    }

    /// The q-axis PI's accumulator.
    #[inline]
    pub fn q_acc(&self) -> i32 {
        self.q_acc
    }

    /// The d-axis PI's accumulator.
    #[inline]
    pub fn d_acc(&self) -> i32 {
        self.d_acc
    }

    /// The reference's full scale, in current counts.
    #[inline]
    pub fn full_scale(&self) -> i16 {
        self.full_scale
    }

    /// The `iq` reference of the last period (0 on a floated one).
    #[inline]
    pub fn iq_ref(&self) -> i16 {
        self.iq_ref
    }

    /// The observation sums of the open window.
    #[inline]
    pub fn sums(&self) -> FocSums {
        self.sums
    }

    /// Close the observation window: the means of the three sums (`sum >> FOC_WINDOW_SHIFT`, an
    /// arithmetic shift, so a mean rounds toward minus infinity by at most one count) and the
    /// latched reference, with the sums restarting for the next window (`specs/foc-torque.md`
    /// section 7). The latch is left alone: it is the last period's reference, not a sum.
    ///
    /// The period ISR calls this once every 64 periods, in the branch that closes the current
    /// limit's window, so the casts cannot truncate: the mean of 64 `i16` values is an `i16`. A
    /// record built mid-window (the arm-time install) reads its first window short by the periods
    /// before it, one 4 ms window at an arm.
    #[inline]
    pub fn take_window(&mut self) -> FocWindow {
        let w = FocWindow {
            iq: (self.sums.iq >> FOC_WINDOW_SHIFT) as i16,
            id: (self.sums.id >> FOC_WINDOW_SHIFT) as i16,
            vq: (self.sums.vq >> FOC_WINDOW_SHIFT) as i16,
            iq_ref: self.iq_ref,
        };
        self.sums = FocSums::default();
        w
    }
}

/// The q-axis current reference for a demand word: `clamp(demand, -28500, 28500) * full_scale /
/// 28500`, rounding toward zero, so a full-scale word asks for exactly `full_scale` counts and the
/// reference carries the demand's sign (`specs/foc-torque.md` sections 3 and 4).
///
/// Per period rather than per demand write (section 2: one multiply and a divide by a constant;
/// deriving it once per write is the first cut if the cycle budget needs one). Cannot overflow:
/// 28,500 x 32,768 is under 2^30, and the result's magnitude is at most `|full_scale|`.
#[inline]
pub fn iq_reference(demand: i32, full_scale: i16) -> i16 {
    // `clamp` is safe here, unlike over the RAM-resident PI bounds: both ends are constants, so
    // its `min <= max` assert folds away at compile time (`specs/panic-free.md`, requirement 1).
    let demand = demand.clamp(-TORQUE_FULL_SCALE, TORQUE_FULL_SCALE);
    (demand * full_scale as i32 / TORQUE_FULL_SCALE) as i16
}

/// One FOC period (`specs/foc-torque.md` section 2, steps 5 to 12) at the aligned angle `theta`
/// (the front end's published angle plus the motor's `angle_path_offset`), from the two
/// conditioned phase currents `currents = (ia, ib)` ([`current_from_adc`] of each, after the
/// period ISR's sample hold).
///
/// Clarke and forward Park, unless **`hold`** is set: then either sensed phase is a held sample
/// (`firmware::motor::SampleHold`, section 14 there), the pair handed in is not this period's
/// current, and the PIs are fed the rotor-frame pair of the last period with both phases fresh
/// instead, the frame in which the quantity is constant at steady speed. An unheld period stores
/// its pair for that. Then the drive gate:
///
/// - **`drive`**: the q PI regulates `iq` to [`iq_reference`] of the demand and the d PI regulates
///   `id` to zero, on one shared set of gains ([`FOC_GAINS`]) and one accumulator each. No
///   decoupling feed-forward and no field weakening (section 2 says why neither is needed at the
///   rover's speeds).
/// - **not `drive`** (the period ISR will float every phase this period, `coast_posture`): both
///   accumulators are zeroed and the command is the zero vector. A disarmed board then cannot wind
///   an integrator on measurement noise and apply it on its first armed period, and that first
///   period starts from zero voltage (bumpless).
///
/// Then the circular limit, inverse Park, SVPWM and the per-motor duty order onto the three
/// channels, all driven (FOC never floats a phase itself; the coast is the ISR's, on the enables).
/// A zero demand with `drive` is zero CURRENT, driven: the loop cancels the back-EMF and the wheel
/// turns freely, unlike sine's mid-rail short.
///
/// Last, the observation sums take this period's `iq`, `id` and limited `vq`, and the reference is
/// latched; both run on a floated period too, so a disarmed board still reports its measured
/// currents (section 7).
pub fn foc_step(
    st: &mut FocState,
    theta: u16,
    currents: (i16, i16),
    demand: i32,
    drive: bool,
    hold: bool,
) -> crate::MotorOutput {
    let (iq, id) = if hold {
        st.held_dq
    } else {
        let cl = clarke(currents.0, currents.1);
        let dq = park_forward(cl.alpha, cl.beta, theta);
        st.held_dq = dq;
        dq
    };

    let (iq_ref, vq, vd) = if drive {
        let gains = FOC_GAINS.load();
        let iq_ref = iq_reference(demand, st.full_scale);
        let vq = pi_step(iq_ref as i32, iq as i32, &gains, &mut st.q_acc);
        let vd = pi_step(0, id as i32, &gains, &mut st.d_acc);
        (iq_ref, vq, vd)
    } else {
        st.q_acc = 0;
        st.d_acc = 0;
        (0, 0, 0)
    };

    let (vq, vd) = circular_limit(vq, vd);
    let (alpha, beta) = park_inverse(vq, vd, theta);
    let duties = st.duty_order.apply(svpwm(alpha, beta));

    st.sums.iq = st.sums.iq.wrapping_add(iq as i32);
    st.sums.id = st.sums.id.wrapping_add(id as i32);
    st.sums.vq = st.sums.vq.wrapping_add(vq as i32);
    st.iq_ref = iq_ref;

    crate::MotorOutput {
        phases: duties.map(crate::PhaseCmd::Drive),
    }
}

// The precomputed quarter-wave sine literal (round(32767 * sin((i/256)*pi/2)), i=0..255).
include!("sin_table.rs");
