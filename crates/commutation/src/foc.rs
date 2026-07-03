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
// The precomputed quarter-wave sine literal (round(32767 * sin((i/256)*pi/2)), i=0..255).
include!("sin_table.rs");
