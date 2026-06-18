//! Config / tuning surface. The algorithm STRUCTURE is fixed (in the other modules); the gain
//! profiles and per-board limits live here as tunable inputs, with the reference
//! constants as defaults (Section 0, Section 6).
//!
//! The genuinely-fractional coefficients the spec marks "(float in original)" are reproduced as
//! `fixed` Q-format reciprocals/weights here and flagged. The pure-integer divisors (`/10000`,
//! `/100`, `*3900/scale`) are NOT Q types; they stay integer divides (truncate toward zero).

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
/// the FSM on engage/promote transitions; `base_coeff` is the 0.4 battery coefficient (same in
/// both profiles).
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
        Self { coeff1: 6000, coeff2: 2000, coeff3: 0x28 } // 0x28 = 40
    }
    /// Profile B (no rider, flag == 0): 3000 / 1000 / 30.
    pub const fn profile_b() -> Self {
        Self { coeff1: 3000, coeff2: 1000, coeff3: 0x1E } // 0x1E = 30
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

/// Envelope / state machine (Section 7).
pub mod envelope {
    /// Envelope cap (0x6F54 = 28500).
    pub const CAP: i32 = 0x6F54;
    /// Engage ramp rate (+200/tick).
    pub const RAMP_UP: i32 = 200;
    /// Idle decay rate (-1000/tick magnitude).
    pub const DECAY: i32 = 1000;
}
