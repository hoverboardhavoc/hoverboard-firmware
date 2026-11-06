//! The rigid frame's steering split (`specs/rover-one-imu.md` section 5): the DIFFERENTIAL torque
//! the heading hold forms (`specs/heading-hold.md`, `crate::sf`) is added to one wheel and taken
//! from the other around the machine's common balance torque. The open-loop turn that used to feed
//! it (`DriveTurn`, the stick's steer as a slewed torque) went with the heading hold.
//!
//! The value component's old consumer, the commanded lean (`specs/control.md` (h), `DriveLean`
//! and `drive_off`), went with the PID cascade: in balance mode the stick now commands a speed
//! and a position through the state-feedback reference (`specs/state-feedback.md` section 4).

use crate::helpers::{clamp_sym, iabs};
// The torque word's full scale: the magnitude the output stage caps a wheel demand at, and
// therefore the sum a wheel pair has to fit inside. Owned by `base` since the FOC arm scales its
// current reference by it too (`specs/foc-torque.md` section 6).
use base::TORQUE_FULL_SCALE;

// -------------------------------------------------------------------------------------------
// The rigid frame's steering split (`specs/rover-one-imu.md` section 5)
// -------------------------------------------------------------------------------------------

/// The steering split (`specs/rover-one-imu.md` section 5): the machine's common balance torque
/// `t` and the differential `turn` become this board's own wheel demand and its peer's, in
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
/// throttle-mode mixer (`DriveMix`, `specs/remote-ui.md` 3.2) does the opposite on purpose: it
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
