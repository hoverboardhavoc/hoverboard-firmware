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
    /// Profile A (rider present, flag != 0): 6000 / 2000 / 40.
    pub const fn profile_a() -> Self {
        Self {
            coeff1: 6000,
            coeff2: 2000,
            coeff3: 0x28, // 40
        }
    }
    /// Profile B (no rider, flag == 0): 3000 / 1000 / 30.
    pub const fn profile_b() -> Self {
        Self {
            coeff1: 3000,
            coeff2: 1000,
            coeff3: 0x1E, // 30
        }
    }
    /// As a `GainTriple` (the (coeff1, coeff2, coeff3) the FSM copies on a full promote).
    pub const fn as_triple(&self) -> GainTriple {
        GainTriple::new(self.coeff1, self.coeff2, self.coeff3)
    }
}

/// Section 6: rider-gated profile select. `flag != 0` (pad/rider present) selects Profile A,
/// `flag == 0` selects Profile B. The base coefficient (0.4) is the same in both, so it is not
/// returned here.
pub fn select_profile(flag: bool) -> GainProfile {
    if flag {
        GainProfile::profile_a()
    } else {
        GainProfile::profile_b()
    }
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
