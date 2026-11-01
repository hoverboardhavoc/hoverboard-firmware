//! Arm and disarm on the arm band (`specs/handheld-remote.md` section 4.5), and what drops the arm
//! level by itself (section 4.7). The arm level is held locally, latching, in RAM only: every boot
//! starts disarmed and nothing re-arms by itself.
//!
//! A touch belongs to a zone by its touch-down point alone: a `Down` with `y0 >= ARM_ZONE_Y` is an
//! arm-zone touch, any other a throttle touch. While disarmed, an arm-zone touch
//! held for [`ARM_HOLD_MS`] to [`ARM_HOLD_MAX_MS`] with `yf` in the zone the whole time arms on the
//! sample that confirms its release. While armed, an arm-zone touch disarms on its `Down`.

use crate::touch::{Lift, SUB};

/// The arm zone's top edge: every panel point with `y` at or past it, the full width, px.
pub const ARM_ZONE_Y: i32 = 250;
/// The shortest hold that arms, from the touch's `Down` to the sample that confirms its release, ms.
pub const ARM_HOLD_MS: u32 = 1000;
/// The longest hold that arms (a longer one is a resting palm), ms.
pub const ARM_HOLD_MAX_MS: u32 = 5000;
/// The hold's progress fill moves in steps of this, so it redraws at most this often, ms
/// (section 5.3).
pub const FILL_STEP_MS: u32 = 50;
/// With no motor controller in the inbox for this long while connected, the arm drops (the app's
/// `BOARDS_OFF_MS`), ms.
pub const BOARDS_OFF_MS: u32 = 1000;

/// Which control a touch works, decided by its touch-down point.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Zone {
    /// `y0` in the arm zone: the arm band.
    Arm,
    /// `y0` above the arm zone: the throttle.
    Throttle,
}

impl Zone {
    /// The zone of a point `y`, in sixteenths of a pixel.
    pub fn of(y: i32) -> Zone {
        if y >= ARM_ZONE_Y * SUB {
            Zone::Arm
        } else {
            Zone::Throttle
        }
    }
}

/// One touch as the arm rule follows it, from its `Down` until its release is confirmed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Held {
    zone: Zone,
    /// When it went `Down`, ms.
    at: u32,
    /// `yf` has left the arm zone at some sample.
    left: bool,
    /// Its `Down` disarmed the machine, so its release does not arm.
    disarmed: bool,
}

/// What the arm band shows of a hold in progress (section 5.3).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HoldShow {
    /// Held this long, ms, in [`FILL_STEP_MS`] steps, under [`ARM_HOLD_MS`].
    Progress(u16),
    /// Held [`ARM_HOLD_MS`] or more: a release now arms.
    Release,
}

/// Why an arm is refused, in the statement line's order (section 5.4).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Refused {
    /// The link is not connected, or the walk or the ride facts have not finished.
    NotReady,
    /// The motor controllers store different frames (the app's `MachineArm.FramesDiffer`).
    FramesDiffer,
    /// A drive refusal holds (section 4.6).
    Drive(crate::drive::Refusal),
    /// No motor controller is in the inbox.
    NoBoard,
    /// A board the arm would arm reports an arm-blocking fault bit.
    Blocked,
}

/// What the arm rule reads at the moment of a release.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Conditions {
    /// Connected, walked and the ride facts read.
    pub ready: bool,
    pub frames_differ: bool,
    pub drive: Option<crate::drive::Refusal>,
    /// A motor controller is in the inbox, and the drive has a board to arm.
    pub board: bool,
    /// A board the arm would arm reports `fault & ARM_BLOCKING_BITS != 0` in its own word.
    pub blocked: bool,
}

impl Conditions {
    /// The first refusal that holds, or `None` when the arm may go ahead.
    pub fn refusal(&self) -> Option<Refused> {
        if !self.ready {
            Some(Refused::NotReady)
        } else if self.frames_differ {
            Some(Refused::FramesDiffer)
        } else if let Some(d) = self.drive {
            Some(Refused::Drive(d))
        } else if !self.board {
            Some(Refused::NoBoard)
        } else if self.blocked {
            Some(Refused::Blocked)
        } else {
            None
        }
    }
}

/// Why the arm level dropped without an arm-zone touch (section 4.7).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dropped {
    /// The link went away.
    Link,
    /// No motor controller in the inbox for [`BOARDS_OFF_MS`].
    BoardsOff,
    /// A board the arm arms reported an arm-blocking bit: the bit and the board, kept for the
    /// statement until the next arm (the app's `faultDisarm`).
    Fault { bit: u16, board: u8 },
}

/// The arm level and the bookkeeping of the touch that works it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Arm {
    armed: bool,
    /// The lift state of the last sample.
    lift: Lift,
    /// The touch now down, from its `Down`.
    touch: Option<Held>,
    /// The last fault disarm's cause, until the next arm.
    pub fault_disarm: Option<Dropped>,
    /// The last refused arm, until the next arm-zone touch.
    pub refused: Option<Refused>,
}

impl Default for Arm {
    fn default() -> Self {
        Arm {
            armed: false,
            lift: Lift::Idle,
            touch: None,
            fault_disarm: None,
            refused: None,
        }
    }
}

/// What a touch did to the arm.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Action {
    Armed,
    Disarmed,
    Refused(Refused),
}

impl Arm {
    /// Whether the arm level is held.
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Follow the touch through one sample: `lift` is the lift machine's state after the sample
    /// taken at `now` (ms). `cond` is read only for an arm.
    pub fn on_touch(
        &mut self,
        lift: Lift,
        now: u32,
        cond: impl FnOnce() -> Conditions,
    ) -> Option<Action> {
        let prev = core::mem::replace(&mut self.lift, lift);
        match (lift.points(), &mut self.touch) {
            (Some((y0, yf)), None) => {
                let zone = Zone::of(y0);
                let mut t = Held {
                    zone,
                    at: now,
                    left: Zone::of(yf) != Zone::Arm,
                    disarmed: false,
                };
                let mut action = None;
                if zone == Zone::Arm {
                    self.refused = None;
                    if self.armed {
                        self.armed = false;
                        t.disarmed = true;
                        action = Some(Action::Disarmed);
                    }
                }
                self.touch = Some(t);
                action
            }
            (Some((_, yf)), Some(t)) => {
                if Zone::of(yf) != Zone::Arm {
                    t.left = true;
                }
                None
            }
            (None, Some(_)) => {
                let t = self.touch.take()?;
                // The release is confirmed on the sample that leaves `Lifting` (a new press may
                // start on that same sample).
                if !matches!(prev, Lift::Lifting { .. }) {
                    return None;
                }
                self.release(t, now, cond)
            }
            (None, None) => None,
        }
    }

    fn release(&mut self, t: Held, now: u32, cond: impl FnOnce() -> Conditions) -> Option<Action> {
        let held = now.wrapping_sub(t.at);
        if t.zone != Zone::Arm
            || t.left
            || t.disarmed
            || self.armed
            || !(ARM_HOLD_MS..=ARM_HOLD_MAX_MS).contains(&held)
        {
            return None;
        }
        match cond().refusal() {
            Some(r) => {
                self.refused = Some(r);
                Some(Action::Refused(r))
            }
            None => {
                self.armed = true;
                self.fault_disarm = None;
                Some(Action::Armed)
            }
        }
    }

    /// The hold in progress at `now` (ms), while disarmed: an arm-zone touch that has stayed in the
    /// zone, did not disarm, and has not run past [`ARM_HOLD_MAX_MS`]. Whether the arm would be
    /// refused is the caller's to judge.
    pub fn hold(&self, now: u32) -> Option<HoldShow> {
        if self.armed {
            return None;
        }
        let t = self.touch?;
        if t.zone != Zone::Arm || t.left || t.disarmed {
            return None;
        }
        let held = now.wrapping_sub(t.at);
        if held > ARM_HOLD_MAX_MS {
            None
        } else if held >= ARM_HOLD_MS {
            Some(HoldShow::Release)
        } else {
            Some(HoldShow::Progress(
                (held / FILL_STEP_MS * FILL_STEP_MS) as u16,
            ))
        }
    }

    /// Drop the arm level for `why` (never refused). Returns whether it was held.
    pub fn drop_for(&mut self, why: Dropped) -> bool {
        let was = self.armed;
        self.armed = false;
        if was {
            if let Dropped::Fault { .. } = why {
                self.fault_disarm = Some(why);
            }
        }
        was
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drive::Refusal;

    /// A point in the arm zone and one above it, in sixteenths of a pixel.
    const IN: i32 = 270 * SUB;
    const ABOVE: i32 = 120 * SUB;

    fn ok() -> Conditions {
        Conditions {
            ready: true,
            frames_differ: false,
            drive: None,
            board: true,
            blocked: false,
        }
    }

    fn down(y0: i32, yf: i32) -> Lift {
        Lift::Down { y0, yf }
    }

    fn lifting(y0: i32, yf: i32, since: u32) -> Lift {
        Lift::Lifting {
            y0,
            yf,
            since,
            resume: 0,
        }
    }

    /// A touch at `y`, `Down` at `at`, held until its release is confirmed at `at + held`
    /// (`Lifting` for the last 40 ms). Every action it caused.
    fn touch(
        a: &mut Arm,
        y: i32,
        at: u32,
        held: u32,
        cond: Conditions,
    ) -> (Option<Action>, Option<Action>) {
        let first = a.on_touch(down(y, y), at, || cond);
        let up = at + held - 40;
        assert_eq!(a.on_touch(lifting(y, y, up), up, || cond), None);
        let last = a.on_touch(Lift::Idle, at + held, || cond);
        (first, last)
    }

    fn armed() -> Arm {
        let mut a = Arm::default();
        touch(&mut a, IN, 0, 1100, ok());
        assert!(a.armed());
        a
    }

    #[test]
    fn the_hold_window() {
        for (held, arms) in [(999, false), (1000, true), (5000, true), (5001, false)] {
            let mut a = Arm::default();
            let (first, last) = touch(&mut a, IN, 100, held, ok());
            assert_eq!(first, None, "{held} ms");
            assert_eq!(a.armed(), arms, "{held} ms");
            assert_eq!(last, arms.then_some(Action::Armed), "{held} ms");
        }
    }

    #[test]
    fn each_refusal_alone_refuses() {
        type Case = (fn(&mut Conditions), Refused);
        let cases: [Case; 5] = [
            (|c| c.ready = false, Refused::NotReady),
            (|c| c.frames_differ = true, Refused::FramesDiffer),
            (
                |c| c.drive = Some(Refusal::NoLeader),
                Refused::Drive(Refusal::NoLeader),
            ),
            (|c| c.board = false, Refused::NoBoard),
            (|c| c.blocked = true, Refused::Blocked),
        ];
        for (set, want) in cases {
            let mut c = ok();
            set(&mut c);
            assert_eq!(c.refusal(), Some(want));
            let mut a = Arm::default();
            let (_, last) = touch(&mut a, IN, 0, 1100, c);
            assert_eq!(last, Some(Action::Refused(want)));
            assert!(!a.armed());
            assert_eq!(a.refused, Some(want));
            // The next arm-zone touch clears it.
            a.on_touch(down(IN, IN), 2000, ok);
            assert_eq!(a.refused, None);
        }
        assert_eq!(ok().refusal(), None);
    }

    #[test]
    fn disarm_is_never_refused_and_the_touch_that_disarms_does_not_arm() {
        let bad = Conditions {
            ready: false,
            blocked: true,
            ..ok()
        };
        let mut a = armed();
        // Disarms on `Down`, before any hold, whatever the conditions.
        assert_eq!(
            a.on_touch(down(IN, IN), 5000, || bad),
            Some(Action::Disarmed)
        );
        assert!(!a.armed());
        assert_eq!(a.hold(6500), None, "no hold shown for it");
        a.on_touch(lifting(IN, IN, 6500), 6500, ok);
        assert_eq!(a.on_touch(Lift::Idle, 6540, ok), None);
        assert!(!a.armed());
        // A new hold arms again.
        touch(&mut a, IN, 7000, 1100, ok());
        assert!(a.armed());
    }

    #[test]
    fn a_touch_that_leaves_the_zone_during_the_hold_does_not_arm() {
        let mut a = Arm::default();
        a.on_touch(down(IN, IN), 0, ok);
        a.on_touch(down(IN, 240 * SUB), 300, ok);
        assert_eq!(a.hold(300), None);
        a.on_touch(down(IN, IN), 400, ok);
        a.on_touch(lifting(IN, IN, 1500), 1500, ok);
        assert_eq!(a.on_touch(Lift::Idle, 1540, ok), None);
        assert!(!a.armed());
    }

    #[test]
    fn a_throttle_touch_never_arms_or_disarms() {
        // Disarmed: a throttle touch dragged down into the zone and held does nothing.
        let mut a = Arm::default();
        assert_eq!(a.on_touch(down(ABOVE, ABOVE), 0, ok), None);
        a.on_touch(down(ABOVE, 280 * SUB), 200, ok);
        assert_eq!(a.hold(1500), None);
        a.on_touch(lifting(ABOVE, 280 * SUB, 1500), 1500, ok);
        assert_eq!(a.on_touch(Lift::Idle, 1540, ok), None);
        assert!(!a.armed());
        // Armed: the same drag stays a throttle drag and does not disarm.
        let mut a = armed();
        assert_eq!(a.on_touch(down(ABOVE, ABOVE), 3000, ok), None);
        assert_eq!(a.on_touch(down(ABOVE, 300 * SUB), 3200, ok), None);
        assert!(a.armed());
        a.on_touch(lifting(ABOVE, 300 * SUB, 3300), 3300, ok);
        assert_eq!(a.on_touch(Lift::Idle, 3340, ok), None);
        assert!(a.armed());
    }

    #[test]
    fn arming_happens_on_the_idle_transition_not_before() {
        let mut a = Arm::default();
        let mut read = false;
        a.on_touch(down(IN, IN), 0, || {
            read = true;
            ok()
        });
        a.on_touch(down(IN, IN), 1500, ok);
        assert!(!a.armed(), "held past ARM_HOLD, still down");
        assert_eq!(a.hold(1500), Some(HoldShow::Release));
        a.on_touch(lifting(IN, IN, 1600), 1600, ok);
        a.on_touch(lifting(IN, IN, 1600), 1620, ok);
        assert!(!a.armed(), "lifting, not yet confirmed");
        assert!(!read, "the conditions are read only at the release");
        assert_eq!(a.on_touch(Lift::Idle, 1640, ok), Some(Action::Armed));
        assert!(a.armed());
        assert_eq!(a.hold(1640), None);
    }

    #[test]
    fn a_new_press_on_the_confirming_sample_still_arms() {
        let mut a = Arm::default();
        a.on_touch(down(IN, IN), 0, ok);
        a.on_touch(lifting(IN, IN, 1100), 1100, ok);
        let pressing = Lift::Pressing {
            n: 1,
            sum: IN,
            last: IN,
        };
        assert_eq!(a.on_touch(pressing, 1140, ok), Some(Action::Armed));
    }

    #[test]
    fn a_lifting_dropout_during_the_hold_that_resumes_keeps_the_hold() {
        let mut a = Arm::default();
        a.on_touch(down(IN, IN), 0, ok);
        a.on_touch(lifting(IN, IN, 300), 300, ok);
        assert_eq!(a.hold(320), Some(HoldShow::Progress(300)));
        a.on_touch(down(IN, IN), 320, ok);
        assert_eq!(a.hold(980), Some(HoldShow::Progress(950)));
        a.on_touch(lifting(IN, IN, 1000), 1000, ok);
        assert_eq!(a.on_touch(Lift::Idle, 1040, ok), Some(Action::Armed));
    }

    #[test]
    fn the_hold_shows_progress_in_steps_then_release_then_nothing_past_the_maximum() {
        let mut a = Arm::default();
        a.on_touch(down(IN, IN), 10, ok);
        assert_eq!(a.hold(10), Some(HoldShow::Progress(0)));
        assert_eq!(a.hold(59), Some(HoldShow::Progress(0)));
        assert_eq!(a.hold(60), Some(HoldShow::Progress(50)));
        assert_eq!(a.hold(1009), Some(HoldShow::Progress(950)));
        assert_eq!(a.hold(1010), Some(HoldShow::Release));
        assert_eq!(a.hold(5010), Some(HoldShow::Release));
        assert_eq!(a.hold(5011), None, "a resting palm");
    }

    #[test]
    fn a_fault_disarm_keeps_its_cause_until_the_next_arm() {
        let mut a = armed();
        let why = Dropped::Fault {
            bit: linkctl::EV_LATCH_A,
            board: 2,
        };
        assert!(a.drop_for(why));
        assert!(!a.armed());
        assert_eq!(a.fault_disarm, Some(why));
        // A short tap keeps it; an arm clears it.
        touch(&mut a, IN, 3000, 200, ok());
        assert_eq!(a.fault_disarm, Some(why));
        touch(&mut a, IN, 4000, 1100, ok());
        assert_eq!(a.fault_disarm, None);
        // A link drop keeps no cause.
        assert!(a.drop_for(Dropped::Link));
        assert_eq!(a.fault_disarm, None);
        assert!(!a.drop_for(Dropped::Link), "already disarmed");
    }
}
