//! Config / tuning surface. The algorithm STRUCTURE is fixed (in the other modules); the gains
//! and per-board limits live here as tunable inputs, with the reference constants as defaults
//! (Section 0, Section 6).
//!
//! The genuinely-fractional coefficients the spec marks "(float in original)" are Q-format at
//! their use sites (`base::fixed`, per `specs/control.md` section (f)) and flagged there. The
//! pure-integer divisors (`/100`, `*3900/scale`) are NOT Q types; they stay integer divides
//! (truncate toward zero).

/// The engagement machine's live gain triple `{kp, bk, pr}` (the stock live gain fields
/// @0x58/@0x5c/@0x60, Section 6 / Section 7.2.1).
///
/// **Nothing reads it** (`specs/state-feedback.md` section 5). It was the recovered balance PID's
/// gain input; the PID went with the state-feedback switch, and the engagement machine, which is
/// unchanged, still seeds and ramps it on its own transitions. The balance producer hands the
/// machine a constant profile ([`STANDBY_SET`]) so those seeds and the ramp keep running on dead
/// words. Removing the machinery is a flash lever recorded in `specs/decision-flash-budget.md`'s
/// terms, not part of that work.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GainTriple {
    /// @0x58: proportional pitch gain (`kp`).
    pub kp: i32,
    /// @0x5c: battery-normalization / rate coefficient (`bk`).
    pub bk: i32,
    /// @0x60: the third-term word (`pr`).
    pub pr: i32,
}

impl GainTriple {
    pub const fn new(kp: i32, bk: i32, pr: i32) -> Self {
        Self { kp, bk, pr }
    }
}

/// The standby seed set {50, 20, 0} (no-rider, the ALT(2) seed). Section 6 / Section 7. Also the
/// constant profile every producer hands the engagement machine (see [`GainTriple`]).
pub const STANDBY_SET: GainTriple = GainTriple::new(0x32, 0x14, 0); // {50, 20, 0}

/// The IDLE->ARMING engage seed for the orientation != 0 path: {1000, 300, 0}. Section 7.2.
pub const ARMING_SEED_ORIENT_NZ: GainTriple = GainTriple::new(1000, 300, 0);

/// A gain profile as the engagement machine takes it (Section 6): it seeds the live triple from
/// `coeff1/2` on engage and wind-down, and ramps it toward `coeff1/2/3` on every RUN pass
/// ([`ramp`]). Every producer passes `GainProfile::of(STANDBY_SET)` (see [`GainTriple`]).
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
    /// The profile a [`GainTriple`] describes.
    pub const fn of(t: GainTriple) -> Self {
        Self {
            coeff1: t.kp,
            coeff2: t.bk,
            coeff3: t.pr,
        }
    }
    /// As a `GainTriple`: the (coeff1, coeff2, coeff3) the FSM ramps the live triple toward in
    /// RUN.
    pub const fn as_triple(&self) -> GainTriple {
        GainTriple::new(self.coeff1, self.coeff2, self.coeff3)
    }
}

// ----- The live gain shadow (`specs/state-feedback.md` section 6) -----

/// The inclusive lower bound of every gain index (`specs/state-feedback.md` section 6): the seam's
/// range is `GAIN_MIN..=max[i]`.
pub const GAIN_MIN: i16 = 0;

/// The default inclusive upper bound of each state-feedback gain (`0 = gp`, `1 = gr`, `2 = gx`,
/// `3 = gv`): the `store::CONTROL_GAIN_MAX` (0x7A) defaults, which `store`'s tests pin to this.
/// Four to ten times above the section-7 start gains `[3491, 4002, 197, 926]`, so the owner has
/// room to tune without first moving the maxima, which are a field because the LQR's torque scale
/// (assumption A1) is uncertain by a factor near one. The output clamp and the engagement envelope
/// bound what any of it can do to the wheel. The range a [`GainShadow`] enforces is the ONLY range
/// validation the gain field gets: the store validates type only, so every writer (boot, the tune
/// lane) goes through it.
pub const DEFAULT_GAIN_MAX: [i16; sf::GAINS] = [20000, 20000, 2000, 10000];

/// Why a [`GainShadow`] write was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TuneError {
    /// The `(field_id, index)` names no live-tunable word: not [`SF_GAINS_FIELD`] index 0..3 or
    /// [`REFERENCE_FIELD`] index 0..1. The tune lane's whole allowlist, in one place.
    UnknownKey,
    /// The value is outside this index's range (the shadow's minima and maxima).
    OutOfRange,
}

/// How many words the live shadow carries: the four gains, then the two reference words.
pub const TUNED: usize = sf::GAINS + sf::REFERENCE_WORDS;

/// The `store::CONTROL_SF_GAINS` field id. Declared here as well because `control` does not depend
/// on `store` (the law is pure math); `store`'s own tests pin the two together, along with the
/// defaults, so neither can drift.
pub const SF_GAINS_FIELD: u8 = 0x79;
/// The `store::CONTROL_REFERENCE` field id; see [`SF_GAINS_FIELD`].
pub const REFERENCE_FIELD: u8 = 0x7B;

/// The shadow index of [`REFERENCE_FIELD`]'s index 0.
const REFERENCE_AT: usize = sf::GAINS;

/// The inclusive lower bound of each shadow word: [`GAIN_MIN`] for the four gains, the compiled
/// [`sf::REFERENCE_MIN`] for the reference row.
const MINIMA: [i16; TUNED] = [
    GAIN_MIN,
    GAIN_MIN,
    GAIN_MIN,
    GAIN_MIN,
    sf::REFERENCE_MIN[0],
    sf::REFERENCE_MIN[1],
];

/// The live tunable words in RAM, as the running law sees them: the gain row (`CONTROL_SF_GAINS`,
/// 0x79) and the reference row (`CONTROL_REFERENCE`, 0x7B), flat in that order.
///
/// **The write policy** (`specs/rider-ui.md` D3, the Betaflight model): a tune write changes the
/// running value immediately and touches NO flash, so it is exempt from the armed refusal by
/// construction rather than by an exception in the armed check; an explicit Save is a plain
/// disarmed `CONFIG_WRITE` through the ordinary store path, and a reboot without one reverts,
/// because boot rebuilds this from the store. In RUN the law does not multiply the shadow's gains
/// directly: it ramps its live copy toward them under fixed per-pass caps
/// ([`sf::RAMP_CAPS`]).
///
/// **`live` vs `stored`.** `live` is what the law reads. `stored` is the last flash value this
/// board read, kept so a later persist can be told apart from a live tune: [`Self::reconcile`]
/// moves a word into `live` only when the FLASH value under it changed, so writing an unrelated
/// field can never quietly revert a value the rider tuned. Both are clamped into range on the way
/// in, so a hand-poked out-of-range flash value cannot enter the law.
///
/// **`max`** is the per-word inclusive upper bound: the boot- and arm-read `CONTROL_GAIN_MAX`
/// (0x7A, each taken as `max(0, value)`) for the gain row, the compiled [`sf::REFERENCE_MAX`] for
/// the reference row. Every range check here reads it, and the gain half is RE-READ at the next
/// arm ([`GainShadow::re_apply_max`]; `specs/integration.md`, "When a stored value takes effect:
/// the arm-time re-read").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GainShadow {
    live: [i16; TUNED],
    stored: [i16; TUNED],
    max: [i16; TUNED],
}

impl GainShadow {
    /// The shadow of a board whose store carries `flash` (`CONTROL_SF_GAINS` indices 0..3, then
    /// `CONTROL_REFERENCE` indices 0..1), with the gain row bounded by `gain_max` (the four
    /// `CONTROL_GAIN_MAX` indices as read; a negative maximum reads as 0, since it would otherwise
    /// make the range empty). Out-of-range values CLAMP: this is boot, and a board with a bad
    /// stored value still has to run.
    pub const fn of_stored(flash: [i16; TUNED], gain_max: [i16; sf::GAINS]) -> Self {
        let max = maxima(gain_max);
        let c = clamp_all(flash, &max);
        Self {
            live: c,
            stored: c,
            max,
        }
    }

    /// The live gain row `[gp, gr, gx, gv]`: the targets the law's live gains ramp toward in RUN.
    pub const fn gains(&self) -> [i16; sf::GAINS] {
        [self.live[0], self.live[1], self.live[2], self.live[3]]
    }

    /// The live reference row `[speed_max, speed_slew]`.
    pub const fn reference(&self) -> [i16; sf::REFERENCE_WORDS] {
        [self.live[REFERENCE_AT], self.live[REFERENCE_AT + 1]]
    }

    /// The live value of one word, or `None` if the key names no tunable word (the `TUNE_READ`
    /// half of the allowlist).
    pub fn get(&self, field_id: u8, index: u8) -> Option<i16> {
        Some(self.live[slot(field_id, index)?])
    }

    /// The last-read FLASH value of one word (what a Save would have to match), or `None` as
    /// [`Self::get`].
    pub fn stored(&self, field_id: u8, index: u8) -> Option<i16> {
        Some(self.stored[slot(field_id, index)?])
    }

    /// Set one word LIVE (the `TUNE_WRITE` half): the allowlist and the range are both enforced
    /// here, and an out-of-range value is REFUSED rather than clamped, so the writer learns its
    /// number was not taken. No flash is touched.
    pub fn set(&mut self, field_id: u8, index: u8, value: i16) -> Result<(), TuneError> {
        let i = slot(field_id, index).ok_or(TuneError::UnknownKey)?;
        if value < MINIMA[i] || value > self.max[i] {
            return Err(TuneError::OutOfRange);
        }
        self.live[i] = value;
        Ok(())
    }

    /// The ARM-TIME maxima re-apply (`specs/integration.md`, "When a stored value takes effect: the
    /// arm-time re-read"): take a fresh read of `CONTROL_GAIN_MAX` and install it as the gain row's
    /// range, under [`of_stored`](Self::of_stored)'s own non-negative floor.
    ///
    /// **`live` is re-clamped; `stored` is deliberately NOT.** A maximum that shrank must not leave
    /// a live gain above it, because this range is the only range validation the gain field gets.
    /// But `stored` is a record of what FLASH held at the last read, and this call learns nothing
    /// new about flash; clamping it would make [`Self::reconcile`], which decides "the flash value
    /// changed" by comparing a freshly clamped read against `stored`, see a flash change that never
    /// happened and write it into `live` over a value the rider had tuned.
    ///
    /// A residual: `stored` is still a CLAMPED record in the other two writers, so a gain whose
    /// flash value sits above a maximum that later GROWS can be moved by a later `reconcile`.
    /// Closing it changes what the `stored()` readback means, which is the tune lane's model in
    /// `specs/rider-ui.md` section 4, so it is a spec-level decision and not this seam's to take.
    ///
    /// The reference row is not in it: its maxima are compiled, so there is nothing to re-read.
    pub fn re_apply_max(&mut self, gain_max: [i16; sf::GAINS]) {
        self.max = maxima(gain_max);
        self.live = clamp_all(self.live, &self.max);
    }

    /// Take a fresh read of the store's words (the shape [`Self::of_stored`] takes) and move into
    /// `live` exactly those whose FLASH value changed since the last read, clamped.
    ///
    /// This is what makes a Save converge without a reboot while leaving a live tune alone: after
    /// a `CONFIG_WRITE` the firmware re-reads and calls this, so a word whose flash value moved
    /// follows it, and a word whose flash value did not keeps whatever the tune lane put there.
    pub fn reconcile(&mut self, flash: [i16; TUNED]) {
        let c = clamp_all(flash, &self.max);
        for (i, fresh) in c.iter().enumerate() {
            if *fresh != self.stored[i] {
                self.stored[i] = *fresh;
                self.live[i] = *fresh;
            }
        }
    }
}

impl Default for GainShadow {
    /// The unstaged board: zero gains, [`sf::REFERENCE_DEFAULT`] and [`DEFAULT_GAIN_MAX`], which
    /// are also the store's defaults. Inert: zero gains give a zero word.
    fn default() -> Self {
        let r = sf::REFERENCE_DEFAULT;
        Self::of_stored([0, 0, 0, 0, r[0], r[1]], DEFAULT_GAIN_MAX)
    }
}

/// The shadow word a `(field_id, index)` names, or `None` if it names none. The tune lane's
/// allowlist lives here: exactly [`SF_GAINS_FIELD`] indices 0..3 and [`REFERENCE_FIELD`] indices
/// 0..1.
const fn slot(field_id: u8, index: u8) -> Option<usize> {
    let i = index as usize;
    match field_id {
        SF_GAINS_FIELD if i < sf::GAINS => Some(i),
        REFERENCE_FIELD if i < sf::REFERENCE_WORDS => Some(REFERENCE_AT + i),
        _ => None,
    }
}

/// The per-word maxima a stored `CONTROL_GAIN_MAX` makes: the gain row takes it with each index
/// floored at [`GAIN_MIN`], the reference row takes the compiled [`sf::REFERENCE_MAX`]. The single
/// owner of both, shared by [`GainShadow::of_stored`] and [`GainShadow::re_apply_max`].
const fn maxima(gain_max: [i16; sf::GAINS]) -> [i16; TUNED] {
    let mut m = [
        gain_max[0],
        gain_max[1],
        gain_max[2],
        gain_max[3],
        sf::REFERENCE_MAX[0],
        sf::REFERENCE_MAX[1],
    ];
    let mut i = 0;
    while i < sf::GAINS {
        if m[i] < GAIN_MIN {
            m[i] = GAIN_MIN;
        }
        i += 1;
    }
    m
}

/// Clamp every word into `MINIMA[i]..=max[i]`. A maximum below its minimum (a gain maximum is
/// floored at 0, so only a reference word could, and its maxima are compiled above) is not
/// reachable.
#[inline(never)]
const fn clamp_all(mut t: [i16; TUNED], max: &[i16; TUNED]) -> [i16; TUNED] {
    let mut i = 0;
    while i < TUNED {
        if t[i] < MINIMA[i] {
            t[i] = MINIMA[i];
        } else if t[i] > max[i] {
            t[i] = max[i];
        }
        i += 1;
    }
    t
}

// ----- Fixed constants used across the cascade (the contract values). -----

pub mod ramp {
    //! The engagement machine's gain RAMP (`specs/rider-ui.md` section 4, "Cross-thread and
    //! mid-loop discipline"): on every RUN pass the machine steps each word of its live triple
    //! TOWARD the profile it was handed by at most a cap.
    //!
    //! **The triple it ramps is no longer read by anything** (`specs/state-feedback.md` section 5):
    //! the PID that multiplied it went with the state-feedback switch, every producer hands the
    //! machine the constant [`STANDBY_SET`](super::STANDBY_SET) profile, and the law ramps its own
    //! four gains under fixed caps ([`super::sf::RAMP_CAPS`]). The machine is unchanged, so this
    //! ramp still runs; the derivation below is the retired PID's and is the record of why the
    //! caps are what they are. [`PROP_DIVISOR`], [`BATT_DIVISOR`] and [`DERIV_DIVISOR`] are that
    //! PID's term divisors, kept here as the derivation's inputs.
    //!
    //! The cap was derived from the property the spec states: **the per-tick gain step is capped
    //! so that the resulting torque delta cannot exceed [`SLEW_LIMIT`] at the worst-case input the
    //! envelope admits.**
    //!
    //! # The derivation
    //!
    //! The PID output is `((bv*bk)/10000 + (pp*kp)/100 + pr*kd/100 - off) * 3900 / scale` (the
    //! retired `pid.rs`), linear in each gain, so a per-tick step `dkp`, `dbk`, `dpr` moves the
    //! output by
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
    //!   proportion (that is the limitation this choice accepts). Since the tilt cutout
    //!   ([`fsm::TILT_LIMIT`], `specs/control.md` (l)) the excess is bounded at 4500/2499 = 1.8x
    //!   for the one tick the cutout fires on. Re-deriving the caps at 4500 is not taken: it
    //!   divides every ramp's speed by 1.8 (standby seed to profile A, 5.95 s to 10.7 s at a 24 V
    //!   word) for a case that lasts a tick.
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
    //! The [`SLEW_LIMIT`] of 250 is split into fixed per-gain shares,
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

    use super::fsm;
    use super::sf::RAW_NUMERATOR;

    /// The per-pass torque-delta bound, word counts (+-0xFA = +-250): the retired stock shaper's
    /// per-tick slew limit (Section 4), which the gain ramp's caps were derived against and which
    /// the law's own ramp caps are checked against (`specs/state-feedback.md` section 6).
    pub const SLEW_LIMIT: i32 = 0xFA;
    /// The retired PID's proportional divisor (100.0), the `kp` term's.
    pub const PROP_DIVISOR: i32 = 100;
    /// The retired PID's battery-normalization divisor (10000.0), the `bk` term's.
    pub const BATT_DIVISOR: i32 = 10000;
    /// The retired PID's derivative divisor (100.0), the `pr * kd` term's.
    pub const DERIV_DIVISOR: i32 = 100;

    /// The worst-case `|pp|` the cap is derived at, centidegrees: the orient==0 upright window (see
    /// the module doc for what is and is not inside it).
    pub const PP_BOUND: i32 = fsm::UPRIGHT_LIMIT;
    /// The worst-case `|bv|`, rad/s x 10000: the nominal +-500 deg/s gyro full scale (8.72665
    /// rad/s), at or above what the real decode of a clamped count gives.
    pub const BV_BOUND: i32 = 87_266;
    /// The torque-count share of [`SLEW_LIMIT`] one pass's `kp` step may use.
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
        let den = (RAW_NUMERATOR * bound) as u32;
        let g = gcd(num, den);
        (num / g, den / g)
    }

    const KP_FRAC: (u32, u32) = reduced(KP_SHARE, PROP_DIVISOR, PP_BOUND);
    const BK_FRAC: (u32, u32) = reduced(BK_SHARE, BATT_DIVISOR, BV_BOUND);
    // `i16::MAX * NUM` must fit the u32 multiply.
    const _: () = assert!((KP_FRAC.0 as u64) * (i16::MAX as u64) <= u32::MAX as u64);
    const _: () = assert!((BK_FRAC.0 as u64) * (i16::MAX as u64) <= u32::MAX as u64);
    const _: () = assert!(
        KP_SHARE + BK_SHARE + PR_SHARE < SLEW_LIMIT,
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
        let pr = (s * (PR_SHARE * DERIV_DIVISOR) as u32 / RAW_NUMERATOR as u32)
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
    /// The RUN tilt cutout, centidegrees of fused pitch or roll magnitude (`specs/control.md`
    /// (l)): past this the machine is falling and [`tilt_cutout`](crate::fsm::tilt_cutout) says
    /// so. OURS, not the binary's, which has no run-time angle gate at all.
    ///
    /// Wider than [`UPRIGHT_LIMIT`] on purpose, and 45 degrees rather than some other wider
    /// number: a wheeled inverted pendulum holds an angle only while its base accelerates at
    /// `g * tan(theta)`, which is 1.0 g here against the ~0.8 g a tyre has before it slides, so a
    /// machine this far over cannot be recovered by any wheel it carries. The stock still/level
    /// classifier's own "lying on its side" flag is 0.75 g of lateral accel, 48.6 degrees, just
    /// above it; the pickup path's inverted-deck edge is at 90.
    pub const TILT_LIMIT: i32 = 4500;
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

/// The four-term state-feedback law (`specs/state-feedback.md`, `crate::sf`). OURS: every constant
/// is a unit choice or a derived bound, and carries its derivation.
pub mod sf {
    /// Index order of the four gains (`store::CONTROL_SF_GAINS`, 0x79): pitch, pitch rate,
    /// position, speed.
    pub const GAINS: usize = 4;

    /// The divisor of each term (section 2's units table): `gp` per degree of a centidegree word,
    /// `gr` per 0.1 rad/s of a rad/s x 10000 word, `gx` per edge of a 1/[`POS_Q`]-edge error,
    /// `gv` per sixteenth of a count of a 1/[`Q`]-count error.
    pub const TERM_DIV: [i64; GAINS] = [100, 1000, POS_Q as i64, 256];

    /// The speed sub-unit: the filter, the reference and the speed error are in 1/4096 of a
    /// wheel-speed count, the unit the period ISR's rate word arrives in (`specs/control.md` (o)).
    pub const Q: i32 = 4096;

    /// The position sub-unit, 1/20480 of an edge: a count is 0.2 edges per 64-period tick, so a
    /// reference speed of `v_ref` [`Q`]-units moves the reference `v_ref` of these per tick and
    /// the fold adds no multiply on the reference side (section 4).
    pub const POS_Q: i32 = 20480;

    /// The fold's `dX2` is twice the mean wheel motion in edges, so one of its half-edges is
    /// `POS_Q / 2` position sub-units.
    pub const HALF_EDGE: i32 = POS_Q / 2;

    /// The wheel-speed filter's one-pole divisor, `v_filt += (v - v_filt) / FILT_DIV`: 4 ticks,
    /// 16 ms, which removes the ripple at the hall-edge rate (the rate updates once per edge and
    /// the spacing is irregular by a few percent) against a 200 ms pendulum.
    pub const FILT_DIV: i32 = 4;

    /// The stick divisor: `target = value * speed_max / STICK_DIV` takes the `+-32767` frame word
    /// and `speed_max` in sixteenths of a count to [`Q`] units (`4096 / 16 / 32767` to within the
    /// truncation; `32767 * 32767` fits `i32`).
    pub const STICK_DIV: i32 = 128;

    /// The speed error's symmetric clamp, 32 counts (9 m/s) in [`Q`] units: outside anything a
    /// machine reaches, it bounds the `gv` product (section 2's overflow bounds).
    pub const E_V_CLAMP: i32 = 32 * Q;

    /// The live gain ramp's per-pass step cap toward the shadow, per gain (section 6): at the
    /// engage window's pitch, 147 deg/s of rate, a 45-edge position error and one count of
    /// speed error, at a battery word of 2400 or more, one pass moves the word by at most 222
    /// counts plus truncations, inside the 250-count [`super::ramp::SLEW_LIMIT`]. Each is at least
    /// 1, so every target is reached.
    pub const RAMP_CAPS: [i16; GAINS] = [2, 1, 1, 1];

    /// The fixed numerator of the output stage's battery normalization (0xF3C = 3900): the word is
    /// `numer * 3900 / scale`, a duty, so torque holds across the pack's range to first order
    /// (section 2). The recovered PID's constant, which the law kept when the PID went.
    pub const RAW_NUMERATOR: i32 = 0xF3C;

    /// How many words the reference row carries (`store::CONTROL_REFERENCE`, 0x7B): `0 =
    /// speed_max`, `1 = speed_slew`.
    pub const REFERENCE_WORDS: usize = 2;

    /// The reference row's compiled inclusive lower bounds: `speed_max` 0 (the stick does nothing
    /// and the machine holds position), `speed_slew` 1 (a staged `speed_max` is never dead behind
    /// a zero rate).
    pub const REFERENCE_MIN: [i16; REFERENCE_WORDS] = [0, 1];

    /// The reference row's compiled inclusive upper bounds (section 6): `speed_max` 512
    /// sixteenths, 32 counts (9.2 m/s) at full stick, which puts the full-stick target exactly on
    /// [`E_V_CLAMP`] (`32767 * 512 / 128` = 131,068 of 131,072), so no reachable target is past the
    /// speed error's clamp; `speed_slew` 4096, one whole count per tick. Compiled rather than a
    /// field because these bound the reference's arithmetic, not the machine's authority, which
    /// the gains and the output clamp bound.
    pub const REFERENCE_MAX: [i16; REFERENCE_WORDS] = [512, 4096];

    /// The reference row's defaults, which are also `store::CONTROL_REFERENCE`'s (the store's
    /// tests pin the two): `speed_max` 0 keeps the stick dead until staged; `speed_slew` 41 is
    /// 0.01 count per tick, 0.72 m/s^2.
    pub const REFERENCE_DEFAULT: [i16; REFERENCE_WORDS] = [0, 41];
}
