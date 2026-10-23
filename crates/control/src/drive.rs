//! The balance-mode drive input (`specs/control.md` (h)): the value component of the effective
//! drive command becomes a bounded, slewed commanded lean in CENTIDEGREES of equilibrium pitch,
//! converted into the shaper's `off` units through the live `kp`.
//!
//! The bound and the rate are stated in physical units (centidegrees), never in `off` units,
//! because `off` is scaled by `kp` and `kp` is tuned over an order of magnitude: at equilibrium
//! with zero rate the PID balances `pp*kp/100 = off`, so `off = kp * L / 100` (with `L` in `pp`'s
//! unit) shifts the equilibrium pitch by exactly `L` for any `kp`.
//!
//! The configuration is [`DriveLean`] (the `CONTROL_DRIVE_LEAN` store field, 0x73, read at boot and
//! again at every arm, clamped at its one seam, [`DriveLean::new`]); the slewed lean is the `drive_lean` carry of
//! [`ShapingState`](crate::ShapingState); [`drive_off`] is the conversion. The term enters the
//! shaper as [`ShapingInputs::drive_off`](crate::ShapingInputs::drive_off), added after the stock
//! slew (step 5) and outside its latch, so the stock `off`-unit clamp and slew bound the steer path
//! only and this module's centidegree bound and rate are the term's only limits.

use crate::config::speed::PP_PER_DEGREE;
use crate::helpers::{clamp_sym, iabs};

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
/// index 1 `lean_slew`). Read at boot and RE-READ at every arm, so a `CONFIG_WRITE` applies at the
/// next arm (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read"; this
/// field is in that decision's value row). Deliberately NOT on the live tune lane: the bound is a
/// protection parameter, set disarmed.
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

// -------------------------------------------------------------------------------------------
// The rigid frame's steering split (`specs/rover-one-imu.md` section 5)
// -------------------------------------------------------------------------------------------

/// The full-scale stock-native torque word (`specs/control.md` (a)): the magnitude the output
/// stage caps a wheel demand at, and therefore the sum a wheel pair has to fit inside. Also the
/// `turn_max` seam ceiling, because the saturation rule below is what protects the balance
/// authority; there is no smaller number that this bound could be derived from.
pub const TORQUE_FULL_SCALE: i32 = 28_500;
/// The `turn_slew` seam floor, torque counts per tick: a configured `turn_max` is never silently
/// dead behind a zero rate ([`LEAN_SLEW_MIN`]'s reason).
pub const TURN_SLEW_MIN: i16 = 1;
/// The `turn_slew` seam ceiling, torque counts per tick: 1000 crosses the whole word in 29 ticks
/// (114 ms).
pub const TURN_SLEW_MAX: i16 = 1000;

/// The validated turn configuration (`store::CONTROL_DRIVE_TURN`, index 0 `turn_max` in torque
/// counts at full stick, index 1 `turn_slew` in counts per tick), the [`DriveLean`] shape exactly:
/// read at boot, re-read at every arm, clamped at this one seam, and deliberately NOT on the live
/// tune lane (a protection parameter, set disarmed).
///
/// `turn_slew` exists because a step in differential torque is a yaw impulse, and on a rigid frame
/// a yaw impulse reaches the pitch loop as a disturbance through the steel. The lean has the same
/// bound for the same reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriveTurn {
    turn_max: i16,
    turn_slew: i16,
}

impl DriveTurn {
    /// Build from the two stored words, each clamped into its seam range (`turn_max`
    /// 0..[`TORQUE_FULL_SCALE`], `turn_slew` [`TURN_SLEW_MIN`]..[`TURN_SLEW_MAX`]), so a
    /// hand-poked flash value cannot reach the loop. `turn_max = 0` is the disabled state and the
    /// store's default: the machine drives straight exactly as it did before the field existed.
    pub const fn new(turn_max: i16, turn_slew: i16) -> Self {
        DriveTurn {
            turn_max: if turn_max < 0 {
                0
            } else if turn_max as i32 > TORQUE_FULL_SCALE {
                TORQUE_FULL_SCALE as i16
            } else {
                turn_max
            },
            turn_slew: if turn_slew < TURN_SLEW_MIN {
                TURN_SLEW_MIN
            } else if turn_slew > TURN_SLEW_MAX {
                TURN_SLEW_MAX
            } else {
                turn_slew
            },
        }
    }

    /// The clamped full-stick differential torque, counts.
    pub const fn turn_max(&self) -> i16 {
        self.turn_max
    }

    /// The clamped per-tick slew, counts.
    pub const fn turn_slew(&self) -> i16 {
        self.turn_slew
    }

    /// The commanded differential torque for a drive `steer`: `steer * turn_max / 32767`, counts,
    /// truncating toward zero (symmetric), `+-turn_max` at the rails. [`DriveLean::lean_cmd`]'s
    /// arithmetic on the other half of the same frame word.
    pub fn turn_cmd(&self, steer: i16) -> i32 {
        (steer as i32 * self.turn_max as i32) / FULL_STICK
    }

    /// One tick: move the slewed turn `turn` (the dispatch's carry) toward [`Self::turn_cmd`] of
    /// `steer` by at most `turn_slew`, and return it. A stale link arrives here as `steer = 0`, so
    /// the slew IS the decay ramp, exactly as [`DriveLean::step`]'s is.
    pub fn step(&self, steer: i16, turn: &mut i32) -> i32 {
        let delta = self.turn_cmd(steer) - *turn;
        *turn += clamp_sym(delta, self.turn_slew as i32);
        *turn
    }
}

impl Default for DriveTurn {
    /// The unstaged board: `turn_max` 0 (disabled), `turn_slew` 24 (a 3000-count turn in 0.5 s),
    /// which are also the store's defaults.
    fn default() -> Self {
        Self::new(0, 24)
    }
}

/// The steering split (`specs/rover-one-imu.md` section 5): the machine's common balance torque
/// `t` and the slewed differential `turn` become this board's own wheel demand and its peer's, in
/// that order. `side_right` is the leader's own wall (`store::CONTROL_SIDE`, the `role_right`
/// board fact): positive `steer` turns RIGHT, so the right wheel gets `t - turn` and the left gets
/// `t + turn`.
///
/// # The saturation rule, and why it inverts the app's
///
/// **When the pair does not fit, the TURN shrinks and the common torque is kept.** The app's
/// throttle-mode mixer (`DriveMix`, `specs/rider-ui.md` 3.2) does the opposite on purpose: it
/// brings the pair into range with a shared OFFSET so the two demands still differ by exactly the
/// steer, because a steering command that quietly straightens out at full deflection is the
/// dangerous surprise and a machine that runs slower than asked is not. That is correct for a
/// SPEED demand and exactly inverted for a BALANCE torque. A shared offset changes `left + right`,
/// which IS the fore-aft torque on the body, which IS what holds the machine up: giving up turn
/// authority makes a balancing rover go straighter than asked, and giving up common torque makes
/// it fall. Because the app ships the other rule, a reader coming from `DriveMix` will assume the
/// wrong one.
///
/// So the turn is clamped to `TORQUE_FULL_SCALE - |t|`, which gives the three properties that
/// matter: `left + right == 2t` exactly, in every case including saturation; neither wheel can
/// clip; and at full balance authority the turn is zero, which is right, because a machine
/// spending all its torque on not falling has none to spare for yawing.
///
/// The headroom is floored at 0 so the clamp's limit is never negative, which also makes the
/// `i16` narrowings below facts rather than luck: `|t +- turn| <= |t| + (28500 - |t|) = 28500`.
pub fn wheel_split(t: i16, turn: i32, side_right: bool) -> (i16, i16) {
    let headroom = TORQUE_FULL_SCALE - iabs(t as i32);
    let turn = clamp_sym(turn, if headroom < 0 { 0 } else { headroom });
    let right = (t as i32 - turn) as i16;
    let left = (t as i32 + turn) as i16;
    if side_right {
        (right, left)
    } else {
        (left, right)
    }
}
