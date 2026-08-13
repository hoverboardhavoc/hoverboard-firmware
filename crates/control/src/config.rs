//! Config / tuning surface. The algorithm STRUCTURE is fixed (in the other modules); the gain
//! profiles and per-board limits live here as tunable inputs, with the reference
//! constants as defaults (Section 0, Section 6).
//!
//! The genuinely-fractional coefficients the spec marks "(float in original)" are Q-format at
//! their use sites (`base::fixed`, per `specs/control.md` section (f)) and flagged there. The
//! pure-integer divisors (`/10000`, `/100`, `*3900/scale`) are NOT Q types; they stay integer
//! divides (truncate toward zero).

/// A balance-PID gain triple `{kp, bk, pr}` (the live gain fields @0x58/@0x5c/@0x60). Section 6 /
/// Section 7.2.1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GainTriple {
    /// @0x58: proportional pitch gain (`kp`).
    pub kp: i32,
    /// @0x5c: battery-normalization / rate coefficient (`bk`).
    pub bk: i32,
    /// @0x60: derivative rate word (`pr`).
    pub pr: i32,
}

impl GainTriple {
    pub const fn new(kp: i32, bk: i32, pr: i32) -> Self {
        Self { kp, bk, pr }
    }
}

/// The standby seed set {50, 20, 0} (no-rider, the ALT(2) seed). Section 6 / Section 7.
pub const STANDBY_SET: GainTriple = GainTriple::new(0x32, 0x14, 0); // {50, 20, 0}

/// The RUN / Profile-A set {6000, 2000, 40} (rider-present). Section 6 / Section 13.
pub const RUN_PROFILE_A: GainTriple = GainTriple::new(6000, 2000, 40);

/// Profile B (low-authority) {3000, 1000, 30}. Section 6.
pub const PROFILE_B: GainTriple = GainTriple::new(3000, 1000, 30);

/// The IDLE->ARMING engage seed for the orientation != 0 path: {1000, 300, 0}. Section 7.2.
pub const ARMING_SEED_ORIENT_NZ: GainTriple = GainTriple::new(1000, 300, 0);

/// A selectable gain profile (Section 6). `coeff1/2/3` are copied into the live gain fields by
/// the FSM on engage/promote transitions. The base coefficient @0x48 (0.4, the same float in
/// both profiles) is not carried here; it enters at its use site as a flagged-fractional
/// constant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GainProfile {
    /// @0x4c
    pub coeff1: i32,
    /// @0x50
    pub coeff2: i32,
    /// @0x54
    pub coeff3: i32,
}

impl GainProfile {
    /// The profile a [`GainTriple`] describes. The triple is the storage form (the shadow's, the
    /// FSM's); the profile is the selection form.
    pub const fn of(t: GainTriple) -> Self {
        Self {
            coeff1: t.kp,
            coeff2: t.bk,
            coeff3: t.pr,
        }
    }
    /// Profile A (rider present, flag != 0) at its DEFAULT gains: 6000 / 2000 / 40. The live
    /// profile comes from the [`GainShadow`]; this is the untuned board's.
    pub const fn profile_a() -> Self {
        Self::of(RUN_PROFILE_A)
    }
    /// Profile B (no rider, flag == 0) at its default gains: 3000 / 1000 / 30.
    pub const fn profile_b() -> Self {
        Self::of(PROFILE_B)
    }
    /// As a `GainTriple` (the (coeff1, coeff2, coeff3) the FSM copies on a full promote).
    pub const fn as_triple(&self) -> GainTriple {
        GainTriple::new(self.coeff1, self.coeff2, self.coeff3)
    }
}

/// Section 6: rider-gated profile select. `flag != 0` (pad/rider present) selects Profile A,
/// `flag == 0` selects Profile B, each read from the live [`GainShadow`]. The base coefficient
/// (0.4) is the same in both, so it is not returned here.
pub fn select_profile(flag: bool, gains: &GainShadow) -> GainProfile {
    GainProfile::of(if flag { gains.a() } else { gains.b() })
}

// ----- The live gain shadow (`specs/rider-ui.md` section 4, prerequisite P1) -----

/// How many gains a profile carries; the index range of `CONTROL_GAIN_A` / `CONTROL_GAIN_B`.
pub const GAINS_PER_PROFILE: usize = 3;

/// The seam-enforced range of each gain index, `(min, max)` inclusive: index 0 `kp`, 1 `bk`,
/// 2 `pr`. Derived, not measured (`specs/rider-ui.md` section 9): the stock value x3, with the
/// PID's own [`pid::OUTPUT_CLAMP`] and the engagement envelope bounding what any of it can do to
/// the wheel. Both profiles share the table, and it is the ONLY range validation these fields
/// get: the store validates type only, so every writer (boot, the tune lane) goes through
/// [`GainShadow`].
pub const GAIN_RANGE: [(i16, i16); GAINS_PER_PROFILE] = [(0, 20000), (0, 10000), (0, 1000)];

/// Why a [`GainShadow`] write was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TuneError {
    /// The `(field_id, index)` names no live-tunable gain: not one of the two profile ids, or an
    /// index past [`GAINS_PER_PROFILE`]. The tune lane's whole allowlist, in one place.
    UnknownKey,
    /// The value is outside this index's [`GAIN_RANGE`].
    OutOfRange,
}

/// The live balance-PID gains: profile A and profile B, in RAM, as the running loop sees them.
///
/// **The write policy** (`specs/rider-ui.md` D3, the Betaflight model): a tune write changes the
/// running gains immediately and touches NO flash, so it is exempt from the armed refusal by
/// construction rather than by an exception in the armed check; an explicit Save is a plain
/// disarmed `CONFIG_WRITE` through the ordinary store path, and a reboot without one reverts,
/// because boot rebuilds this from the store.
///
/// **`live` vs `stored`.** `live` is what the loop runs. `stored` is the last flash value this
/// board read, kept so a later persist can be told apart from a live tune: [`Self::reconcile`]
/// moves a gain into `live` only when the FLASH value under it changed, so writing an unrelated
/// field can never quietly revert a gain the rider tuned. Both are clamped into [`GAIN_RANGE`] on
/// the way in, so a hand-poked out-of-range flash value cannot enter the loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GainShadow {
    live: [[i16; GAINS_PER_PROFILE]; 2],
    stored: [[i16; GAINS_PER_PROFILE]; 2],
}

impl GainShadow {
    /// The shadow of a board whose store carries `flash` (profile A's triple then profile B's, as
    /// the two `CONTROL_GAIN_*` fields read out index by index). Out-of-range values CLAMP: this
    /// is boot, and a board with a bad stored gain still has to run.
    pub const fn of_stored(flash: [[i16; GAINS_PER_PROFILE]; 2]) -> Self {
        let c = clamp_triple(flash);
        Self { live: c, stored: c }
    }

    /// Profile A's live triple.
    pub const fn a(&self) -> GainTriple {
        triple(self.live[0])
    }
    /// Profile B's live triple.
    pub const fn b(&self) -> GainTriple {
        triple(self.live[1])
    }

    /// The live value of one gain, or `None` if the key names no gain (the `TUNE_READ` half of the
    /// allowlist).
    pub fn get(&self, field_id: u8, index: u8) -> Option<i16> {
        let (p, i) = slot(field_id, index)?;
        Some(self.live[p][i])
    }

    /// The last-read FLASH value of one gain (what a Save would have to match), or `None` as
    /// [`Self::get`].
    pub fn stored(&self, field_id: u8, index: u8) -> Option<i16> {
        let (p, i) = slot(field_id, index)?;
        Some(self.stored[p][i])
    }

    /// Set one gain LIVE (the `TUNE_WRITE` half): the allowlist and the range are both enforced
    /// here, and an out-of-range value is REFUSED rather than clamped, so the writer learns its
    /// number was not taken. No flash is touched.
    pub fn set(&mut self, field_id: u8, index: u8, value: i16) -> Result<(), TuneError> {
        let (p, i) = slot(field_id, index).ok_or(TuneError::UnknownKey)?;
        let (lo, hi) = GAIN_RANGE[i];
        if value < lo || value > hi {
            return Err(TuneError::OutOfRange);
        }
        self.live[p][i] = value;
        Ok(())
    }

    /// Take a fresh read of the store's gains (the same shape [`Self::of_stored`] takes) and move
    /// into `live` exactly those whose FLASH value changed since the last read, clamped.
    ///
    /// This is what makes a Save converge without a reboot while leaving a live tune alone: after
    /// a `CONFIG_WRITE` the firmware re-reads and calls this, so a gain whose flash value moved
    /// follows it, and a gain whose flash value did not keeps whatever the tune lane put there.
    pub fn reconcile(&mut self, flash: [[i16; GAINS_PER_PROFILE]; 2]) {
        let c = clamp_triple(flash);
        for (p, profile) in c.iter().enumerate() {
            for (i, fresh) in profile.iter().enumerate() {
                if *fresh != self.stored[p][i] {
                    self.stored[p][i] = *fresh;
                    self.live[p][i] = *fresh;
                }
            }
        }
    }
}

impl Default for GainShadow {
    /// The untuned board: the compiled stock profiles, which are also the store's defaults.
    fn default() -> Self {
        Self::of_stored([
            [
                RUN_PROFILE_A.kp as i16,
                RUN_PROFILE_A.bk as i16,
                RUN_PROFILE_A.pr as i16,
            ],
            [
                PROFILE_B.kp as i16,
                PROFILE_B.bk as i16,
                PROFILE_B.pr as i16,
            ],
        ])
    }
}

/// The profile slot and gain index a `(field_id, index)` names, or `None` if it names no gain.
/// The tune lane's allowlist lives here: exactly the two profile ids, exactly three indices.
const fn slot(field_id: u8, index: u8) -> Option<(usize, usize)> {
    let p = match field_id {
        GAIN_FIELD_A => 0,
        GAIN_FIELD_B => 1,
        _ => return None,
    };
    if (index as usize) >= GAINS_PER_PROFILE {
        return None;
    }
    Some((p, index as usize))
}

/// The `store::CONTROL_GAIN_A` field id. Declared here as well because `control` does not depend
/// on `store` (the cascade is pure math); `store`'s own tests pin the two together, along with the
/// defaults, so neither can drift.
pub const GAIN_FIELD_A: u8 = 0x71;
/// The `store::CONTROL_GAIN_B` field id; see [`GAIN_FIELD_A`].
pub const GAIN_FIELD_B: u8 = 0x72;

/// Clamp both profiles' triples into [`GAIN_RANGE`].
const fn clamp_triple(mut t: [[i16; GAINS_PER_PROFILE]; 2]) -> [[i16; GAINS_PER_PROFILE]; 2] {
    let mut p = 0;
    while p < 2 {
        let mut i = 0;
        while i < GAINS_PER_PROFILE {
            let (lo, hi) = GAIN_RANGE[i];
            if t[p][i] < lo {
                t[p][i] = lo;
            } else if t[p][i] > hi {
                t[p][i] = hi;
            }
            i += 1;
        }
        p += 1;
    }
    t
}

/// One stored triple as the cascade's [`GainTriple`] (the i16 store width widened to the loop's).
const fn triple(t: [i16; GAINS_PER_PROFILE]) -> GainTriple {
    GainTriple::new(t[0] as i32, t[1] as i32, t[2] as i32)
}

// ----- Fixed constants used across the cascade (the contract values). -----

/// Pitch shaping (Section 4).
pub mod shaping {
    /// Differential-to-lean gain (0x12 = 18).
    pub const SPEED_TO_LEAN_GAIN: i32 = 0x12;
    /// Center lean offset (0xDAC = 3500).
    pub const CENTER_LEAN_OFFSET: i32 = 0xDAC;
    /// Absolute shaped-target clamp (+-7000).
    pub const ABS_CLAMP: i32 = 7000;
    /// Per-tick slew limit (+-0xFA = +-250).
    pub const SLEW_LIMIT: i32 = 0xFA;
}

/// Balance PID (Section 3.2).
pub mod pid {
    /// Battery-normalization divisor for term 1 (10000.0). Pure-integer divide.
    pub const BATT_DIVISOR: i32 = 10000;
    /// Proportional divisor for term 2 (100.0). Pure-integer divide.
    pub const PROP_DIVISOR: i32 = 100;
    /// Derivative divisor (100.0). Pure-integer divide.
    pub const DERIV_DIVISOR: i32 = 100;
    /// Derivative-term symmetric clamp (+-30473 = +-0x7709). Two-sided.
    pub const DERIV_CLAMP: i32 = 30473;
    /// Fixed raw numerator (0xF3C = 3900).
    pub const RAW_NUMERATOR: i32 = 3900;
    /// Raw output clamp (+-28500 = +-0x6F54).
    pub const OUTPUT_CLAMP: i32 = 28500;
    /// Pitch-scale hysteresis threshold (0xDAC = 3500).
    pub const SCALE_THRESHOLD: i32 = 0xDAC;
    /// Secondary scale low (0x320 = 800).
    pub const SECONDARY_SCALE_LOW: i32 = 0x320;
    /// Secondary scale high (0x640 = 1600).
    pub const SECONDARY_SCALE_HIGH: i32 = 0x640;
}

/// Speed/steer loop (Section 5, per the slice-4 re-cut) + the setpoint helper (Section 5.1).
/// The fractional Q coefficients (0.4/0.6, 0.9996, 1.2) stay flagged INLINE at their use sites
/// per the slice-1/2 precedent; the integer-valued contract constants live here.
pub mod speed {
    /// Direction/brake band threshold: the +-30.0f raw-bit compares (`0x41F00000` /
    /// `0xC1F00000`) of the original, integer-valued.
    pub const DIRECTION_BAND: i32 = 30;
    /// The integrator deadband divisor: `thr = W / 5` (unsigned divide of the unsigned window
    /// halfword).
    pub const DEADBAND_DIVISOR: u16 = 5;
    /// Section 5.1 saturation threshold (+-0x8000): at or beyond it the setpoint saturates.
    pub const SETPOINT_THRESHOLD: i32 = 0x8000;
    /// Section 5.1 saturation VALUE (+0x7FFF; the negative side is -0x7FFF = the 0x8001
    /// halfword, NEVER -0x8000).
    pub const SETPOINT_SAT: i16 = 0x7FFF;
}

/// Envelope / state machine (Section 7).
pub mod envelope {
    /// Envelope cap (0x6F54 = 28500).
    pub const CAP: i32 = 0x6F54;
    /// Engage ramp rate (+200/tick).
    pub const RAMP_UP: i32 = 200;
    /// Idle decay rate (-1000/tick magnitude).
    pub const DECAY: i32 = 1000;
}

/// Throttle-mode conditioning (spec (b)): EFeru's adopted defaults, each in ITS fixdt shape
/// (spec (f)); provenance reference/efferu-hoverboard @ a0751d589fd43d8975eda3683fac21a44bbfe8fa.
/// The two frame-adapter constants are the PORT's own seams (spec (b)'s frames around the
/// EFeru +-1000 command domain), not EFeru values.
pub mod throttle {
    /// Rate-limiter step, fixdt(1,16,4): 480 = 30.0 units/tick (config.h:190 DEFAULT_RATE).
    pub const RATE: i16 = 480;
    /// Low-pass coefficient, fixdt(0,16,16): 6553 = 0.1 (the spec (b) citation).
    pub const FILTER_COEF: u16 = 6553;
    /// Mixer speed coefficient, fixdt(1,16,14): 16384 = 1.0 (config.h:192 DEFAULT_*).
    pub const SPEED_COEFFICIENT: i16 = 16384;
    /// Mixer steer coefficient, fixdt(1,16,14): 8192 = 0.5 (config.h:193 DEFAULT_*).
    pub const STEER_COEFFICIENT: i16 = 8192;
    /// The EFeru command-domain limit (INPUT_MAX, util.c:271; INPUT_MIN is its negation).
    pub const CMD_LIMIT: i16 = 1000;
    /// Frame-in adapter: the +-32767 input frame's full scale (mapped to +-CMD_LIMIT exactly
    /// at the rails; the port's seam, not EFeru's).
    pub const FRAME_IN_MAX: i32 = 32767;
    /// Frame-out adapter numerator/denominator: +-1000 -> +-28500 (57/2; exact at the rails;
    /// the port's seam, not EFeru's).
    pub const REF_SCALE_NUM: i32 = 57;
    /// See [`REF_SCALE_NUM`].
    pub const REF_SCALE_DEN: i32 = 2;
}

/// Engagement machine (Section 7.2, per the slice-5 re-cut): the integer contract values. The
/// per-path float constants (the 0.4f/3.0f quadruple writes, the 100.0f upright scale) stay
/// flagged INLINE at their use sites per the slice-1/2 precedent.
pub mod fsm {
    /// Upright window limit, orient == 0 branch: engage requires `|f2iz(ref*100)| <= 0x9C3`
    /// (2499); the binary's `>` comparison guards the skip (the corrected window).
    pub const UPRIGHT_LIMIT: i32 = 0x9C3;
    /// Upright window limit, orient != 0 branch (0x1D4B = 7499).
    pub const UPRIGHT_LIMIT_ALT: i32 = 0x1D4B;
    /// Engage gating threshold: the shared gating/pickup halfword must exceed 500.
    pub const GATING_THRESHOLD: i16 = 500;
    /// RUN pickup counter trip (> 0x14 = 20 ticks, ~80 ms).
    pub const PICKUP_TRIP: i32 = 0x14;
    /// The pickup counter's saturating cap.
    pub const PICKUP_CAP: i32 = 100;
    /// RUN wind-down debounce trip (> 10 ticks, ~40 ms).
    pub const WINDDOWN_TRIP: i32 = 10;
    /// Sub-2 promotion debounce trip (> 5 ticks).
    pub const PROMOTE_TRIP: i32 = 5;
    /// The shared saturating cap of the promotion and wind-down debounce counters (0x8ACE).
    pub const DEBOUNCE_CAP: i32 = 0x8ACE;
}
