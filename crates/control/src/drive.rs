//! The rigid frame's turn input and steering split (`specs/rover-one-imu.md` section 5): the
//! steer component of the effective drive command becomes a bounded, slewed DIFFERENTIAL torque
//! that the split adds to one wheel and takes from the other around the machine's common balance
//! torque.
//!
//! The value component's old consumer, the commanded lean (`specs/control.md` (h), `DriveLean`
//! and `drive_off`), went with the PID cascade: in balance mode the stick now commands a speed
//! and a position through the state-feedback reference (`specs/state-feedback.md` section 4).

use crate::helpers::{clamp_sym, iabs};

/// The full-stick magnitude of `DRIVE_CMD.steer`: `turn = steer * turn_max / 32767`.
const FULL_STICK: i32 = 32767;

// -------------------------------------------------------------------------------------------
// The rigid frame's steering split (`specs/rover-one-imu.md` section 5)
// -------------------------------------------------------------------------------------------

/// The full-scale stock-native torque word (`specs/control.md` (a)): the magnitude the output
/// stage caps a wheel demand at, and therefore the sum a wheel pair has to fit inside. Also the
/// `turn_max` seam ceiling, because the saturation rule below is what protects the balance
/// authority; there is no smaller number that this bound could be derived from.
pub const TORQUE_FULL_SCALE: i32 = 28_500;
/// The `turn_slew` seam floor, torque counts per tick: a configured `turn_max` is never silently
/// dead behind a zero rate.
pub const TURN_SLEW_MIN: i16 = 1;
/// The `turn_slew` seam ceiling, torque counts per tick: 1000 crosses the whole word in 29 ticks
/// (114 ms).
pub const TURN_SLEW_MAX: i16 = 1000;

/// The validated turn configuration (`store::CONTROL_DRIVE_TURN`, index 0 `turn_max` in torque
/// counts at full stick, index 1 `turn_slew` in counts per tick): read at boot, re-read at every
/// arm, clamped at this one seam, and deliberately NOT on the live tune lane (a protection
/// parameter, set disarmed).
///
/// `turn_slew` exists because a step in differential torque is a yaw impulse, and on a rigid frame
/// a yaw impulse reaches the pitch loop as a disturbance through the steel.
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
    /// truncating toward zero (symmetric), `+-turn_max` at the rails.
    pub fn turn_cmd(&self, steer: i16) -> i32 {
        (steer as i32 * self.turn_max as i32) / FULL_STICK
    }

    /// One tick: move the slewed turn `turn` (the dispatch's carry) toward [`Self::turn_cmd`] of
    /// `steer` by at most `turn_slew`, and return it. A stale link arrives here as `steer = 0`, so
    /// the slew IS the decay ramp.
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
/// board fact): positive `steer` turns RIGHT, so the right wheel gets `t + turn` and the left gets
/// `t - turn`.
///
/// # Why the right wheel gets the PLUS
///
/// **In this word, NEGATIVE is forward** (`specs/control.md` (a), "Which sign is forward"). The
/// demand is the stock torque word: `specs/attitude.md` publishes a forward lean as NEGATIVE pitch
/// and the balance law's pitch term follows its sign (`crate::sf`), so the torque that catches a
/// forward fall is negative, and the fleet's `motor.direction` is staged to make that negative
/// torque drive the wheel forward. Measured twice on silicon: the bench master on 2026-08-01
/// (`specs/silicon-queue.md`, "CATCH DIRECTION CORRECT") and the rover's right wheel alone on
/// 2026-10-23 (`specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`), where +7,809 of throttle
/// demand drove it backward.
///
/// Turning right means the right wheel runs LESS FORWARD than the left, which in a
/// backward-positive word is the MORE POSITIVE demand. Written the other way round (the right
/// wheel on `t - turn`, as it was until 2026-10-23) a positive steer yaws the machine left, which
/// is what the rover did on two surfaces.
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
    let right = (t as i32 + turn) as i16;
    let left = (t as i32 - turn) as i16;
    if side_right {
        (right, left)
    } else {
        (left, right)
    }
}
