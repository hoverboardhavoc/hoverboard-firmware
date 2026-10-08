//! The balance-mode drive input (`specs/control.md` (h)): the value component of the effective
//! drive command becomes a bounded, slewed commanded lean in CENTIDEGREES of equilibrium pitch,
//! converted into the shaper's `off` units through the live `kp`.
//!
//! The bound and the rate are stated in physical units (centidegrees), never in `off` units,
//! because `off` is scaled by `kp` and `kp` is tuned over an order of magnitude: at equilibrium
//! with zero rate the PID balances `pp*kp/100 = off`, so `off = kp * L / 100` (with `L` in `pp`'s
//! unit) shifts the equilibrium pitch by exactly `L` for any `kp`.
//!
//! The configuration is [`DriveLean`] (the `CONTROL_DRIVE_LEAN` store field, 0x73, boot-read and
//! clamped at its one seam, [`DriveLean::new`]); the slewed lean is the `drive_lean` carry of
//! [`ShapingState`](crate::ShapingState); [`drive_off`] is the conversion. The term enters the
//! shaper as [`ShapingInputs::drive_off`](crate::ShapingInputs::drive_off).

use crate::config::speed::PP_PER_DEGREE;
use crate::helpers::clamp_sym;

/// The `lean_max` seam ceiling, centidegrees (15 degrees): inside the rover's 27.9 degree torque
/// run-out with disturbance margin, and inside the 24.99 degree engage window.
pub const LEAN_MAX_CEIL: i16 = 1500;
/// The `lean_slew` seam floor, centidegrees per tick: a configured `lean_max` is never silently
/// dead behind a zero rate.
pub const LEAN_SLEW_MIN: i16 = 1;
/// The `lean_slew` seam ceiling, centidegrees per tick (250 degrees/s at 250 Hz).
pub const LEAN_SLEW_MAX: i16 = 100;

/// The full-stick magnitude of `DRIVE_CMD.value`: `lean_cmd = value * lean_max / 32767`.
const FULL_STICK: i32 = 32767;

/// The validated drive-lean configuration (`store::CONTROL_DRIVE_LEAN`, index 0 `lean_max`,
/// index 1 `lean_slew`). Read at boot; a `CONFIG_WRITE` applies at the next boot. Deliberately NOT
/// on the live tune lane: the bound is a protection parameter, set disarmed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriveLean {
    lean_max: i16,
    lean_slew: i16,
}

impl DriveLean {
    /// Build from the two stored words, each clamped into its seam range (`lean_max` 0..1500
    /// centidegrees, `lean_slew` 1..100 centidegrees per tick), so a hand-poked out-of-range flash
    /// value cannot reach the loop. `lean_max = 0` is the disabled state: the drive value is then
    /// discarded exactly as it was before this input existed.
    pub const fn new(lean_max: i16, lean_slew: i16) -> Self {
        DriveLean {
            lean_max: if lean_max < 0 {
                0
            } else if lean_max > LEAN_MAX_CEIL {
                LEAN_MAX_CEIL
            } else {
                lean_max
            },
            lean_slew: if lean_slew < LEAN_SLEW_MIN {
                LEAN_SLEW_MIN
            } else if lean_slew > LEAN_SLEW_MAX {
                LEAN_SLEW_MAX
            } else {
                lean_slew
            },
        }
    }

    /// The clamped full-stick lean, centidegrees.
    pub const fn lean_max(&self) -> i16 {
        self.lean_max
    }

    /// The clamped per-tick slew, centidegrees.
    pub const fn lean_slew(&self) -> i16 {
        self.lean_slew
    }

    /// The commanded lean for a drive `value`: `value * lean_max / 32767`, centidegrees,
    /// truncating toward zero (symmetric), `+-lean_max` at the rails.
    pub fn lean_cmd(&self, value: i16) -> i32 {
        (value as i32 * self.lean_max as i32) / FULL_STICK
    }

    /// One tick: move the slewed lean `lean` (the [`ShapingState`](crate::ShapingState) carry)
    /// toward [`Self::lean_cmd`] of `value` by at most `lean_slew`, and return it. A stale link
    /// arrives here as `value = 0`, so the slew IS the decay ramp.
    pub fn step(&self, value: i16, lean: &mut i32) -> i32 {
        let delta = self.lean_cmd(value) - *lean;
        *lean += clamp_sym(delta, self.lean_slew as i32);
        *lean
    }
}

impl Default for DriveLean {
    /// The unstaged board: `lean_max` 0 (disabled), `lean_slew` 4 (10 degrees/s), which are also
    /// the store's defaults.
    fn default() -> Self {
        Self::new(0, 4)
    }
}

/// The lean, centidegrees, as the shaper's `off` term through the live `kp`:
/// `-(kp * lean * PP_PER_DEGREE) / 10_000`, the product in i64, ONE truncation toward zero,
/// narrowed to i32 (in range: `kp` <= 20000 and `|lean|` <= 1500 give at most 300,000).
///
/// The leading minus is the sign convention: lean-forward is NEGATIVE pitch (`specs/attitude.md`),
/// and a positive drive value commands lean-forward, so a positive lean moves the equilibrium to a
/// negative pitch.
pub fn drive_off(kp: i32, lean: i32) -> i32 {
    (-(kp as i64 * lean as i64 * PP_PER_DEGREE as i64) / 10_000) as i32
}
