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

use base::pi::{pi_step, PiRecord};

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
// ============================================================================================

/// Forward Park: rotate the stator vector `(alpha, beta)` into the rotor frame at `theta`.
/// Returns `(d, q)` where `d = MAC(alpha*cos - beta*sin)`, `q = MAC(alpha*sin + beta*cos)`.
#[inline]
pub fn park_forward(alpha: i16, beta: i16, theta: u16) -> (i16, i16) {
    let (s, c) = lookup_sincos(theta);
    let d = mac_q15(alpha as i32 * c as i32 - beta as i32 * s as i32);
    let q = mac_q15(alpha as i32 * s as i32 + beta as i32 * c as i32);
    (d, q)
}

/// Inverse Park: rotate the rotor-frame command `(d, q)` back into the stator frame at `theta`.
/// Returns `(alpha, beta)` where `alpha = RND(d*cos + q*sin)`, `beta = RND(q*cos - d*sin)`. The
/// RND form truncates to 16 bits (WRAPS on overflow, no saturate; defined behavior).
#[inline]
pub fn park_inverse(d: i16, q: i16, theta: u16) -> (i16, i16) {
    let (s, c) = lookup_sincos(theta);
    let alpha = rnd_q15(d as i32 * c as i32 + q as i32 * s as i32);
    let beta = rnd_q15(q as i32 * c as i32 - d as i32 * s as i32);
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

/// Section 6 step 5: circular magnitude limit. `sq = d^2 + q^2`; if `sq <= THRESH` pass through,
/// else scale both components by the table gain (round-and-shift, no saturate).
#[inline]
pub fn circular_limit(d: i16, q: i16) -> (i16, i16) {
    let sq = (d as i32 * d as i32 + q as i32 * q as i32) as u32;
    if sq <= CIRC_THRESH {
        return (d, q);
    }
    let idx = (((sq >> 24) as i32 - 0x3D) & 0xFF) as usize;
    // The table covers exactly the 67 over-threshold bins; idx is in range for any sq > THRESH that
    // fits in 32 bits (max d^2+q^2 = 2*32767^2 = 0x7FFE0002, >>24 = 0x7F, idx <= 0x42 = 66).
    let g = CIRC_GAIN[idx.min(66)] as i32;
    // Scale: d' = rnd_q15(g*d), q' = rnd_q15(g*q), the section-6.2 round-and-shift form.
    (rnd_q15(g * d as i32), rnd_q15(g * q as i32))
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

/// Hall debounce reload value (number of PWM periods a line is held after an edge). CONFIG; the
/// reference value is 150 (~9.4 ms at 16 kHz).
pub const HALL_DEBOUNCE_RELOAD: i16 = 150; // 0x96

/// The 6-state base electrical-angle table. Index by hall code 1..6 (index 0 unused).
/// These are the bit-exact recovered anchors, bench-confirmed against live stock.
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
static FWD_NEXT: [u8; 8] = [0, 3, 6, 2, 5, 1, 4, 0];
/// Reverse-next neighbor (dir = -1) per hall code (index 0/7 unused).
static REV_NEXT: [u8; 8] = [0, 5, 3, 1, 6, 4, 2, 0];

/// Per-line hall debounce state.
// Deviation from the archived code (named): the derived `Default` is dropped. It constructed a
// reload of 0 (debounce disabled), contradicting `new()`'s reference 150; nothing consumed it,
// and a misleading constructor is a trap, not API.
#[derive(Clone, Copy, Debug)]
pub struct HallDebounce {
    /// Stored (debounced) level per line (0/1).
    pub level: [u8; 3],
    /// Per-line lockout countdown (signed, decremented each period while > 0).
    pub lockout: [i16; 3],
    /// Per-line "recently changed" marker.
    pub changed: [bool; 3],
    /// Configurable reload value (defaults to 150).
    pub reload: i16,
}

impl Default for HallDebounce {
    fn default() -> Self {
        Self::new()
    }
}

impl HallDebounce {
    /// New debounce state with the reference reload value.
    pub fn new() -> Self {
        Self {
            level: [0; 3],
            lockout: [0; 3],
            changed: [false; 3],
            reload: HALL_DEBOUNCE_RELOAD,
        }
    }

    /// Section 5.1: feed one period of raw hall levels (each 0/1) and return the debounced 3-bit
    /// code `A | (B<<1) | (C<<2)`.
    #[allow(clippy::needless_range_loop)] // recovered loop shape: parallel arrays indexed by line
    pub fn step(&mut self, raw: [u8; 3]) -> u8 {
        for line in 0..3 {
            // Eligible if not currently locked (lockout reached 0).
            let eligible = self.lockout[line] == 0;
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
    /// Latched raw speed (signed edge count per 320-period window).
    pub speed: i32,
    /// Speed measurement window counter.
    pub window: u16,
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
            } else if REV_NEXT[self.prev_code as usize] == code {
                self.edge_acc -= 1;
            }
        }
        if valid {
            self.prev_code = code;
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
    /// A fresh front-end with the reference hall reload and a cleared commutation estimator.
    pub fn new() -> Self {
        Self {
            hall: HallDebounce::new(),
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

impl Default for RotorFrontEnd {
    fn default() -> Self {
        Self::new()
    }
}
// ============================================================================================
// Section 6 step 4 / 6.1: d-axis open-loop drive ramp
// ============================================================================================

/// d-axis ramp threshold (CONFIG; reference 800).
pub const RAMP_THRESH: i32 = 800; // 0x320
/// d-axis ramp relax-branch step (CONFIG; reference 0). STEP = 0 holds the command in the relax
/// branch rather than ramping it.
pub const RAMP_STEP: i32 = 0; // 0
/// Fixed slew step.
pub const RAMP_SLEW: i32 = 0x20;
/// Counter cap.
pub const RAMP_COUNTER_CAP: i32 = 0xFA;

/// The d-axis drive-command ramp state.
#[derive(Clone, Copy, Debug, Default)]
pub struct DRamp {
    /// The held d-axis command `s`.
    pub s: i32,
    /// The persistent counter (cap 0xFA).
    pub counter: i32,
}

impl DRamp {
    /// Section 6 step 4: one period of the open-loop rate-limited d-axis ramp. `demand` is the
    /// signed drive demand `D`. Returns the new d-axis command.
    pub fn step(&mut self, demand: i32) -> i32 {
        if demand / 1000 <= RAMP_THRESH {
            // Demand at/below threshold: relax by the held step.
            self.counter = 0x20;
            self.s += RAMP_STEP;
        } else {
            // Demand above threshold: slew by the counter.
            if self.counter < RAMP_COUNTER_CAP {
                self.counter += 1;
            }
            if self.counter < self.s {
                self.s -= self.counter;
            } else if self.s > 0 {
                self.s -= RAMP_SLEW;
            } else if -self.counter < self.s {
                self.s += RAMP_SLEW;
            } else {
                self.s += self.counter;
            }
        }
        self.s
    }
}

/// The recovered stock inner-current-loop PI record (the archived `control::helpers` `seed()`,
/// relocated here per the slice-1 decision: `base::pi` stays generic and the stock constants live
/// with their consumer). `0xF0002000` as signed 32-bit is `-268427264`; the integral-clamp fields
/// are seeded inverted relative to their names and clamped BY VALUE (see `base::pi`).
pub const fn foc_pi_record() -> PiRecord {
    const INT_LOW: i64 = 0xF000_2000u32 as i32 as i64; // -268427264 (negative; LOW bound)
    PiRecord {
        kp: 100,
        kp_divisor: 0x400,  // 1024
        ki: 0x32,           // 50
        ki_divisor: 0x2000, // 8192
        out_min: -32767,
        out_max: 0x7FFF,
        int_max: INT_LOW,
        int_min: -INT_LOW,
        accumulator: 0,
    }
}

// ============================================================================================
// Section 6 step 3 + the stall-aware anti-windup (the open-gaps fix)
// ============================================================================================

/// The q-axis current PI with stall-aware anti-windup.
///
/// The q-axis PI regulates the measured q-axis current to a reference of ZERO. The stock structure
/// is `base::pi`'s [`PiRecord`] / [`pi_step`] seeded by [`foc_pi_record`] (Kp=100, P_div=0x400,
/// Ki=50, I_div=0x2000, out clamp +-32767, integ clamp +-0x0FFFE000).
///
/// The open-gaps bench finding (2026-06-15): on a STALLED rotor commanded but not rotating, a small
/// residual `q_meas` bias cannot be nulled (the rotor does not move), so the integrator winds to its
/// clamp and the output pegs to ~1 A. Stock never sits there because while balancing it makes
/// continuous micro-movements so `q_meas` averages to zero. The fix: NO zero-torque deadband (stock
/// has none); add stall-aware q-PI anti-windup that HOLDS or BLEEDS the q integrator when commanded
/// but not rotating, so a stalled rotor cannot peg the output.
#[derive(Clone, Copy, Debug)]
pub struct QAxisPi {
    /// The shared inner-current-loop PI record (seeded with the reference gains).
    pub pi: PiRecord,
}

/// Fraction (numerator/256) by which the held q integrator is bled each stalled period. A value of
/// 255 holds (bleeds 1/256 per period, a slow leak that prevents windup without a hard freeze); the
/// effect is the integrator cannot ratchet up on a stalled rotor. This is the anti-windup leak.
pub const STALL_BLEED_NUM: i64 = 255;
pub const STALL_BLEED_DEN: i64 = 256;

impl Default for QAxisPi {
    fn default() -> Self {
        Self::new()
    }
}

impl QAxisPi {
    /// Construct with the recovered seed gains ([`foc_pi_record`]).
    pub fn new() -> Self {
        Self {
            pi: foc_pi_record(),
        }
    }

    /// Section 6 step 3: run the q-axis PI at reference 0 with stall-aware anti-windup.
    ///
    /// `q_measured` is the forward-Park q-axis current. `rotating` is true when the rotor is turning
    /// (hall edges seen recently / nonzero speed); `commanded` is true when a nonzero drive demand
    /// is present. When commanded but NOT rotating (the stalled case), the integrator is bled toward
    /// zero and prevented from ratcheting: the standard `pi_step` runs, then if the update grew the
    /// integrator magnitude it is reverted and a leak applied instead. This bounds the q integrator
    /// so a stalled rotor cannot peg the output. Returns the commanded q-axis voltage (i16).
    pub fn step(&mut self, q_measured: i32, rotating: bool, commanded: bool) -> i16 {
        let stalled = commanded && !rotating;

        if stalled {
            // Snapshot the integrator before the standard PI update.
            let before = self.pi.accumulator;
            let out = pi_step(0, q_measured, &mut self.pi);
            let after = self.pi.accumulator;
            // Stall-aware anti-windup: do not let the integrator grow in magnitude while stalled,
            // and bleed it toward zero so a residual q bias cannot wind it to the clamp.
            let grew = after.unsigned_abs() > before.unsigned_abs();
            if grew {
                // Revert the windup step: hold at the pre-update value, then bleed.
                self.pi.accumulator = before;
            }
            // Bleed the held integrator toward zero (the leak that keeps a stalled rotor bounded).
            self.pi.accumulator = self.pi.accumulator * STALL_BLEED_NUM / STALL_BLEED_DEN;
            // Recompute the output from the bled integrator so the returned voltage reflects it.
            // out = I / I_div + (e * Kp) / P_div, e = 0 - q_measured (matches pi_step's step 3/4).
            let e = -q_measured;
            let i_term = self.pi.accumulator / self.pi.ki_divisor as i64;
            let p_term = (e * self.pi.kp / self.pi.kp_divisor) as i64;
            let raw = i_term + p_term;
            let clamped = if raw < self.pi.out_min as i64 {
                self.pi.out_min as i64
            } else if raw > self.pi.out_max as i64 {
                self.pi.out_max as i64
            } else {
                raw
            };
            let _ = out; // the reverted/bled result supersedes the raw pi_step output
            clamped as i16
        } else {
            // Normal operation (rotating, or not commanded): the stock PI runs unmodified.
            pi_step(0, q_measured, &mut self.pi)
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DutyOrder {
    map: [u8; 3],
}

impl DutyOrder {
    /// The reference direct order: (CH0, CH1, CH2) = (base, c1, c2).
    pub const DIRECT: DutyOrder = DutyOrder { map: [0, 1, 2] };

    /// Apply the permutation to an SVPWM record, yielding the three channel compare counts.
    #[inline]
    pub fn apply(&self, s: Svpwm) -> [u16; 3] {
        let bcc = [s.base, s.c1, s.c2];
        [
            bcc[self.map[0] as usize],
            bcc[self.map[1] as usize],
            bcc[self.map[2] as usize],
        ]
    }
}

/// The per-motor FOC records: the cal-gated offsets, the q-PI (+ stall anti-windup), the d-axis
/// demand ramp, and the per-motor SVPWM output order. Constructible ONLY with a [`PhaseOffsets`]
/// (the structural cal gate). The shared rotor front-end is NOT here (deviation from the archive,
/// named: the archived `FocState` owned the front-end; the slice-3 `Commutator` owns it now, and
/// [`foc_step`] consumes the front-end's [`RotorState`] instead of stepping it, the identical
/// computation order with the step performed one layer up, exactly once per period).
#[derive(Clone, Copy, Debug)]
pub struct FocState {
    offsets: PhaseOffsets,
    q_pi: QAxisPi,
    d_ramp: DRamp,
    duty_order: DutyOrder,
}

impl FocState {
    /// Fresh FOC records from a cal-gated offset pair and the per-motor SVPWM output order
    /// (pass [`DutyOrder::DIRECT`] until the bench resolves a board's permutation).
    pub fn new(offsets: PhaseOffsets, duty_order: DutyOrder) -> Self {
        Self {
            offsets,
            q_pi: QAxisPi::new(),
            d_ramp: DRamp::default(),
            duty_order,
        }
    }
}

/// One FOC period over the shared front-end's rotor snapshot (the recovered computation order,
/// bit-preserved): offset-correct the two phase samples, Clarke, forward Park, the q-axis PI at
/// reference 0 with stall-aware anti-windup, the d-axis demand ramp (the demand enters HERE, not
/// the PI), the circular magnitude limit, inverse Park, SVPWM, and the per-motor duty order onto
/// the three channels (all driven; FOC never floats a phase).
///
/// (Deviations from the archived `foc_step`, named: the front-end step moved to the dispatch and
/// `rotor` arrives as data; the `FocOutput { svpwm, angle, speed, hall_fault }` wrapper is gone,
/// its telemetry fields were re-exports of front-end state the dispatch now owns, and the duty
/// order is applied here instead of in the retired hot layer.)
pub fn foc_step(
    st: &mut FocState,
    rotor: RotorState,
    sample_a: u16,
    sample_b: u16,
    demand: i32,
) -> crate::MotorOutput {
    let theta = rotor.angle;

    // Offset-corrected phase currents.
    let i_a = current_from_adc(st.offsets.a(), sample_a);
    let i_b = current_from_adc(st.offsets.b(), sample_b);

    // Clarke + forward Park.
    let cl = clarke(i_a, i_b);
    let (_d_meas, q_meas) = park_forward(cl.alpha, cl.beta, theta);

    // q-axis PI (reference 0) with stall-aware anti-windup.
    let rotating = rotor.speed != 0 || rotor.in_window;
    let commanded = demand != 0;
    let q_cmd = st.q_pi.step(q_meas as i32, rotating, commanded);

    // d-axis open-loop drive ramp (the demand enters here, NOT through the PI).
    let d_cmd = st.d_ramp.step(demand);

    // Circular magnitude limit on the (d, q) command pair. The q command is the PI output; in
    // this firmware's frame the d-slot command carries the drive torque.
    let (d_lim, q_lim) = circular_limit(sat16(d_cmd), q_cmd);

    // Inverse Park back to the stator frame, then SVPWM, then the per-motor channel order.
    let (alpha, beta) = park_inverse(d_lim, q_lim, theta);
    let duties = st.duty_order.apply(svpwm(alpha, beta));
    crate::MotorOutput {
        phases: duties.map(crate::PhaseCmd::Drive),
    }
}

// The precomputed quarter-wave sine literal (round(32767 * sin((i/256)*pi/2)), i=0..255).
include!("sin_table.rs");
