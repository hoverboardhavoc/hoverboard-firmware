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

/// A selectable gain profile (Section 6). The FSM seeds the live gain fields from `coeff1/2` on
/// engage and wind-down, and ramps them toward `coeff1/2/3` on every RUN pass ([`ramp`]). The
/// base coefficient @0x48 (0.4, the same float in both profiles) is not carried here; it enters
/// at its use site as a flagged-fractional constant.
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
    /// As a `GainTriple`: the (coeff1, coeff2, coeff3) the FSM ramps the live triple toward in
    /// RUN.
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

/// The inclusive lower bound of every gain index, both profiles (`specs/rider-ui.md` section 4):
/// the seam's range is `GAIN_MIN..=max[i]`.
pub const GAIN_MIN: i16 = 0;

/// The default inclusive upper bound of each gain index (`0 = kp`, `1 = bk`, `2 = pr`), both
/// profiles: the `store::CONTROL_GAIN_MAX` (0x74) defaults, which `store`'s tests pin to this.
/// The stock value x3, derived before there was a rover plant to check it against, so the owner
/// moves the maxima at runtime through that field (`specs/rider-ui.md` section 4, "Ranges"); the
/// PID's own [`pid::OUTPUT_CLAMP`] and the engagement envelope bound what any of it can do to the
/// wheel. The range a [`GainShadow`] enforces is the ONLY range validation the gain fields get:
/// the store validates type only, so every writer (boot, the tune lane) goes through it.
pub const DEFAULT_GAIN_MAX: [i16; GAINS_PER_PROFILE] = [20000, 10000, 1000];

/// Why a [`GainShadow`] write was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TuneError {
    /// The `(field_id, index)` names no live-tunable gain: not one of the two profile ids, or an
    /// index past [`GAINS_PER_PROFILE`]. The tune lane's whole allowlist, in one place.
    UnknownKey,
    /// The value is outside this index's `GAIN_MIN..=max` (the shadow's maxima).
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
/// field can never quietly revert a gain the rider tuned. Both are clamped into
/// `GAIN_MIN..=max[i]` on the way in, so a hand-poked out-of-range flash value cannot enter the
/// loop.
///
/// **`max`** is the per-index inclusive upper bound the shadow was built with (the boot-read
/// `CONTROL_GAIN_MAX`, each taken as `max(0, value)`), and every range check here reads it. It is
/// RE-READ at the next arm ([`GainShadow::re_apply_max`]; `specs/integration.md`, "When a stored
/// value takes effect: the arm-time re-read"), so a written maximum applies from the next arm
/// rather than the next power-cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GainShadow {
    live: [[i16; GAINS_PER_PROFILE]; 2],
    stored: [[i16; GAINS_PER_PROFILE]; 2],
    max: [i16; GAINS_PER_PROFILE],
}

impl GainShadow {
    /// The shadow of a board whose store carries `flash` (profile A's triple then profile B's, as
    /// the two `CONTROL_GAIN_*` fields read out index by index), bounded by `max` (the three
    /// `CONTROL_GAIN_MAX` indices as read; a negative maximum reads as 0, since it would otherwise
    /// make the range empty). Out-of-range values CLAMP: this is boot, and a board with a bad
    /// stored gain still has to run.
    pub const fn of_stored(
        flash: [[i16; GAINS_PER_PROFILE]; 2],
        max: [i16; GAINS_PER_PROFILE],
    ) -> Self {
        let max = floor_max(max);
        let c = clamp_triple(flash, &max);
        Self {
            live: c,
            stored: c,
            max,
        }
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
        if value < GAIN_MIN || value > self.max[i] {
            return Err(TuneError::OutOfRange);
        }
        self.live[p][i] = value;
        Ok(())
    }

    /// The ARM-TIME maxima re-apply (`specs/integration.md`, "When a stored value takes effect: the
    /// arm-time re-read"): take a fresh read of `CONTROL_GAIN_MAX` and install it as the shadow's
    /// range, under [`of_stored`](Self::of_stored)'s own non-negative floor.
    ///
    /// Both `live` and `stored` are re-clamped into the NEW range by the same helper `of_stored`
    /// uses: a maximum that shrank must not leave a live gain above it (the shadow's range is the
    /// only range validation the gain fields get, so a value outside it would otherwise reach the
    /// loop), and a maximum that grew simply admits values the tune lane was refusing.
    ///
    /// It moves no gain VALUE of its own: the two `CONTROL_GAIN_*` fields are not in the value row,
    /// they are live through the tune lane and converge on a disarmed save through
    /// [`Self::reconcile`].
    pub fn re_apply_max(&mut self, max: [i16; GAINS_PER_PROFILE]) {
        let max = floor_max(max);
        self.max = max;
        self.live = clamp_triple(self.live, &max);
        self.stored = clamp_triple(self.stored, &max);
    }

    /// Take a fresh read of the store's gains (the same shape [`Self::of_stored`] takes) and move
    /// into `live` exactly those whose FLASH value changed since the last read, clamped.
    ///
    /// This is what makes a Save converge without a reboot while leaving a live tune alone: after
    /// a `CONFIG_WRITE` the firmware re-reads and calls this, so a gain whose flash value moved
    /// follows it, and a gain whose flash value did not keeps whatever the tune lane put there.
    pub fn reconcile(&mut self, flash: [[i16; GAINS_PER_PROFILE]; 2]) {
        let c = clamp_triple(flash, &self.max);
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
    /// The untuned board: the compiled stock profiles and [`DEFAULT_GAIN_MAX`], which are also
    /// the store's defaults.
    fn default() -> Self {
        Self::of_stored(
            [
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
            ],
            DEFAULT_GAIN_MAX,
        )
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

/// A stored `CONTROL_GAIN_MAX` read as a usable range top: each index taken as `max(0, value)`,
/// since a negative maximum would otherwise make the range empty. The single owner of that floor,
/// shared by [`GainShadow::of_stored`] and [`GainShadow::re_apply_max`].
const fn floor_max(mut max: [i16; GAINS_PER_PROFILE]) -> [i16; GAINS_PER_PROFILE] {
    let mut i = 0;
    while i < GAINS_PER_PROFILE {
        if max[i] < GAIN_MIN {
            max[i] = GAIN_MIN;
        }
        i += 1;
    }
    max
}

/// Clamp both profiles' triples into `GAIN_MIN..=max[i]` (`max` already non-negative).
const fn clamp_triple(
    mut t: [[i16; GAINS_PER_PROFILE]; 2],
    max: &[i16; GAINS_PER_PROFILE],
) -> [[i16; GAINS_PER_PROFILE]; 2] {
    let mut p = 0;
    while p < 2 {
        let mut i = 0;
        while i < GAINS_PER_PROFILE {
            if t[p][i] < GAIN_MIN {
                t[p][i] = GAIN_MIN;
            } else if t[p][i] > max[i] {
                t[p][i] = max[i];
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
    /// The unit of the speed loop's blend input, hence of `pp`: CENTIDEGREES of fused pitch, 100
    /// per degree (`specs/control.md` (j): stock's mixer blends the `euler[0] * 100.0f - trim`
    /// cell, settled from the board-20 image). Named so the proportional path and (h)'s drive-lean
    /// term convert through the same constant; a host test pins it equal to the blend carry a
    /// 1.00 degree pitch word settles to.
    pub const PP_PER_DEGREE: i32 = 100;
}

pub mod ramp {
    //! The live gain RAMP (`specs/rider-ui.md` section 4, "Cross-thread and mid-loop discipline"):
    //! on every RUN pass the engagement machine steps each live gain TOWARD the shadow by at most a
    //! cap, and the cap is derived from the property the spec states: **the per-tick gain step is
    //! capped so that the resulting torque delta cannot exceed
    //! [`shaping::SLEW_LIMIT`](super::shaping::SLEW_LIMIT) at the worst-case input the envelope
    //! admits.**
    //!
    //! # The derivation
    //!
    //! The PID output is `((bv*bk)/10000 + (pp*kp)/100 + pr*kd/100 - off) * 3900 / scale`
    //! (`crate::pid`), linear in each gain, so a per-tick step `dkp`, `dbk`, `dpr` moves the output
    //! by
    //!
    //! ```text
    //! (dkp*|pp|/100 + dbk*|bv|/10000 + dpr*|kd|/100) * 3900 / scale   (exact, before truncation)
    //! ```
    //!
    //! The worst-case inputs, and why each is taken where it is:
    //!
    //! - **`|pp|` <= [`PP_BOUND`] = 2499 centidegrees**, the orient==0 upright window
    //!   ([`fsm::UPRIGHT_LIMIT`]). `pp` is `blend + trim + acc`: the blend is the pitch word, which
    //!   the window bounds at engage; the trim word has no producer (`block.trim` is 0) and the
    //!   leaky integrator is gated off in this image (`SpeedInputs::gate` is `false`, so `acc`
    //!   stays 0). A trim or integrator producer adds its own bound here (the integrator's leak
    //!   caps `|acc|` at 1.2 / 0.0004 = 3000). The window is an ENGAGE gate, not a RUN gate: a
    //!   machine falling past 25 degrees in RUN exceeds it, and the kp share below grows in
    //!   proportion (that is the limitation this choice accepts).
    //! - **`|bv|` <= [`BV_BOUND`] = 87,266**, the gyro full scale (+-500 deg/s, the IMU's
    //!   `GYRO_CONFIG`) in the rate word's unit, rad/s x 10000 (`specs/control.md` (j)). A hard
    //!   bound: the IMU clamps the bias-corrected count to +-32767, which decodes to 87,263 (pinned
    //!   against the real decode in the orchestrator tests).
    //! - **`|kd|` is unbounded**: a `Fix` with no producer (`block.kd` is `Fix::ZERO`) and no
    //!   staged range. So the `pr` cap is taken from the LIVE `kd` each pass (its integer ceiling,
    //!   so the bound is conservative): with `kd == 0` the `pr*kd` term is zero whatever `pr`
    //!   holds, and `pr` steps straight to its target; a future `kd` producer gets the bound with
    //!   no change here.
    //! - **`scale` is LIVE**: the engage gate admits any nonzero battery word (the clamp floor is 1
    //!   centivolt), there is no low-battery engage floor yet (`specs/sensing-and-safety.md`, open
    //!   question), and the rover's pack sags under torque, so a constant cap would rest on a floor
    //!   the code does not enforce. The cap is linear in `scale`, computed per pass from the word
    //!   the PID divides by.
    //!
    //! The [`SLEW_LIMIT`](super::shaping::SLEW_LIMIT) of 250 is split into fixed per-gain shares,
    //! [`KP_SHARE`] 180 + [`BK_SHARE`] 45 + [`PR_SHARE`] 10 = 235, so all three gains stepping in
    //! the same pass stay inside it, and each cap is
    //! `floor(share * divisor * scale / (3900 * bound))`. The 15 counts left over carry the
    //! integer truncations: `t78` and `t7c` each truncate once (a step can move each by at most
    //! one count more than its exact value), the `* 3900 / scale` divide truncates once more, and
    //! the 0.99/0.01 reference IIR adds one: `235 + 2*3900/scale + 2`, which is <= 250 for every
    //! `scale` >= 600 (6 V, a word no board runs on).
    //! The shares favour `kp` because its worst-case torque reach is the largest (its full range at
    //! `|pp|` = 2499 is about 5.7x `bk`'s at full gyro scale).
    //!
    //! At a battery word of 2400 (the LEV50-8's 24 V safe floor) the caps are `kp` 4, `bk` 3 per
    //! pass, and `pr` 6 at `kd` = 100; at 3300 (full) 6 and 4. From the standby seed `{50, 20, 0}`
    //! to profile A `{6000, 2000, 40}` takes 1,488 passes (5.95 s) at 2400; from profile A to the
    //! top of every range `{20000, 10000, 1000}` takes 3,500 passes (14.0 s) at 2400 and 2,334
    //! (9.3 s) at 3300.
    //!
    //! # Convergence
    //!
    //! Every cap is floored at 1 ([`ramp_toward`]), so a live gain that differs from its target
    //! moves at least one count per RUN pass and reaches any in-range target in at most `range`
    //! passes (20,000 for `kp`, the slowest). The floor overrides the derived cap only where the
    //! cap computes below 1: `kp` at `scale` < 542, `bk` at `scale` < 757, `pr` at `|kd|` > 0.256 x
    //! `scale`. Below 757 the floored `bk` step overruns its share (56.7 counts at 600 against
    //! 45), and the bound then rests on the SUM: with `kp` and `bk` each stepping 1 and `pr` inside
    //! its share, the worst-case delta is `3900 * (24.99 + 8.7266 + 2) / scale + 10 + 2` (244 at
    //! 600), which exceeds 250 only below a battery word of 586 (5.86 V, a word no board runs on).
    //! The `pr` floor is a `kd` 600 at 24 V, 6x the value the spec's own measurement simulates.
    //!
    //! # Trade: constant worst-case inputs vs the live inputs
    //!
    //! Only `scale` and `kd` are live. A cap computed from the live `|pp|` and `|bv|` would
    //! converge much faster near upright (a pass where `pp` is 100 admits about 25x the `kp` step),
    //! but the gain written on this pass is consumed by the NEXT pass's PID, against inputs that
    //! can move a long way in one tick (`bv` is a raw rate, and `pp` crosses zero while balancing),
    //! so a live-input cap is a bound on the wrong tick. `scale` (a filtered battery word) and `kd`
    //! (a coefficient) do not move that way.

    use super::{fsm, pid};

    /// The worst-case `|pp|` the cap is derived at, centidegrees: the orient==0 upright window (see
    /// the module doc for what is and is not inside it).
    pub const PP_BOUND: i32 = fsm::UPRIGHT_LIMIT;
    /// The worst-case `|bv|`, rad/s x 10000: the nominal +-500 deg/s gyro full scale (8.72665
    /// rad/s), at or above what the real decode of a clamped count gives.
    pub const BV_BOUND: i32 = 87_266;
    /// The torque-count share of [`super::shaping::SLEW_LIMIT`] one pass's `kp` step may use.
    pub const KP_SHARE: i32 = 180;
    /// The share `bk`'s step may use.
    pub const BK_SHARE: i32 = 45;
    /// The share `pr`'s step may use.
    pub const PR_SHARE: i32 = 10;

    const fn gcd(a: u32, b: u32) -> u32 {
        if b == 0 {
            a
        } else {
            gcd(b, a % b)
        }
    }

    /// `cap = floor(scale * NUM / DEN)` with the fraction `share * divisor / (3900 * bound)`
    /// reduced at compile time, so the per-pass arithmetic is one u32 multiply and one divide by
    /// a constant (no 64-bit division on the 250 Hz path).
    const fn reduced(share: i32, divisor: i32, bound: i32) -> (u32, u32) {
        let num = (share * divisor) as u32;
        let den = (pid::RAW_NUMERATOR * bound) as u32;
        let g = gcd(num, den);
        (num / g, den / g)
    }

    const KP_FRAC: (u32, u32) = reduced(KP_SHARE, pid::PROP_DIVISOR, PP_BOUND);
    const BK_FRAC: (u32, u32) = reduced(BK_SHARE, pid::BATT_DIVISOR, BV_BOUND);
    // `i16::MAX * NUM` must fit the u32 multiply.
    const _: () = assert!((KP_FRAC.0 as u64) * (i16::MAX as u64) <= u32::MAX as u64);
    const _: () = assert!((BK_FRAC.0 as u64) * (i16::MAX as u64) <= u32::MAX as u64);
    const _: () = assert!(
        KP_SHARE + BK_SHARE + PR_SHARE < super::shaping::SLEW_LIMIT,
        "the shares must leave room for the truncations"
    );

    /// The per-pass step cap of each gain at this pass's PID divisor word `scale` (centivolts,
    /// 0 = UNKNOWN) and derivative coefficient `kd`, BEFORE the floor of 1 [`ramp_toward`]
    /// applies. `pr`'s cap is `u32::MAX` when `kd` is zero (the term it multiplies is zero).
    pub fn caps(scale: i16, kd: base::fixed::Fix) -> [u32; 3] {
        let s = if scale > 0 { scale as u32 } else { 0 };
        let kp = s * KP_FRAC.0 / KP_FRAC.1;
        let bk = s * BK_FRAC.0 / BK_FRAC.1;
        // ceil(|kd|) from the Q32.32 bits. `|bits|` <= 2^63, so `mag >> 32` <= 2^31 and the
        // ceiling is at most 2^31 + 1, which fits a u32 without saturation.
        let mag = kd.to_bits().unsigned_abs();
        let kd_ceil = ((mag >> 32) + u64::from(mag as u32 != 0)) as u32;
        let pr = (s * (PR_SHARE * pid::DERIV_DIVISOR) as u32 / pid::RAW_NUMERATOR as u32)
            .checked_div(kd_ceil)
            .unwrap_or(u32::MAX);
        [kp, bk, pr]
    }

    /// One RUN pass of the ramp: each of `live`'s gains steps toward `target`'s by at most its
    /// [`caps`] entry, floored at 1 so a differing gain always moves (the convergence guarantee in
    /// the module doc), and never past the target.
    pub fn ramp_toward(
        live: super::GainTriple,
        target: super::GainTriple,
        scale: i16,
        kd: base::fixed::Fix,
    ) -> super::GainTriple {
        let [kp, bk, pr] = caps(scale, kd);
        super::GainTriple::new(
            step(live.kp, target.kp, kp),
            step(live.bk, target.bk, bk),
            step(live.pr, target.pr, pr),
        )
    }

    /// `live` moved toward `target` by at most `max(cap, 1)`.
    fn step(live: i32, target: i32, cap: u32) -> i32 {
        let c = if cap == 0 {
            1
        } else if cap > i32::MAX as u32 {
            i32::MAX
        } else {
            cap as i32
        };
        // Both operands are gain words (i16 store range widened), so the difference cannot
        // overflow.
        live + crate::helpers::clamp(target - live, -c, c)
    }
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
