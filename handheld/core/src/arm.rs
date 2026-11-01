//! Arm and disarm on the BOOT button (`specs/handheld-remote.md` section 4.5), and what drops the
//! arm level by itself (section 4.7). The arm level is held locally, latching, in RAM only: every
//! boot starts disarmed and nothing re-arms by itself.

/// IO0 low for this long is a press; high for this long is a release, ms.
pub const DEBOUNCE_MS: u32 = 20;
/// The shortest press that arms, ms.
pub const PRESS_MIN_MS: u32 = 50;
/// The longest press that arms; a low held past it is a button fault, ms.
pub const PRESS_MAX_MS: u32 = 1500;
/// With no motor controller in the inbox for this long while connected, the arm drops (the app's
/// `BOARDS_OFF_MS`), ms.
pub const BOARDS_OFF_MS: u32 = 1000;

/// What the debounced button did on one sample.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Edge {
    /// A press started (IO0 low for [`DEBOUNCE_MS`]).
    Press,
    /// The press ended after `held` ms, measured edge to edge.
    Release { held: u32 },
    /// The low has lasted past [`PRESS_MAX_MS`]: a button fault until IO0 is high again.
    Fault,
}

/// The BOOT button's debouncer, polled every 2 ms.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Button {
    /// The debounced level: pressed.
    pressed: bool,
    /// When the raw level last changed, and to what (low = true).
    raw: Option<(bool, u32)>,
    /// When the press that is held started (its low edge).
    since: u32,
    /// The held press has run past [`PRESS_MAX_MS`].
    fault: bool,
}

impl Button {
    /// Whether a button fault is in force (IO0 low past [`PRESS_MAX_MS`], until it is high again).
    pub fn fault(&self) -> bool {
        self.fault
    }

    /// Feed IO0's level at `now` (ms; `low` is pressed).
    pub fn poll(&mut self, low: bool, now: u32) -> Option<Edge> {
        let changed_at = match self.raw {
            Some((level, at)) if level == low => at,
            _ => {
                self.raw = Some((low, now));
                now
            }
        };
        let stable = now.wrapping_sub(changed_at) >= DEBOUNCE_MS;
        if stable && low != self.pressed {
            self.pressed = low;
            if low {
                self.since = changed_at;
                return Some(Edge::Press);
            }
            let held = changed_at.wrapping_sub(self.since);
            let was_fault = self.fault;
            self.fault = false;
            return (!was_fault).then_some(Edge::Release { held });
        }
        if self.pressed && low && !self.fault && now.wrapping_sub(self.since) > PRESS_MAX_MS {
            self.fault = true;
            return Some(Edge::Fault);
        }
        None
    }
}

/// Why an arm is refused, in the statement line's order (section 5.4).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Refused {
    /// A button fault is in force ("BOOT held.").
    ButtonFault,
    /// The touch is not Idle ("Release the screen to arm.").
    FingerDown,
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
    pub button_fault: bool,
    pub finger_down: bool,
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
        if self.button_fault {
            Some(Refused::ButtonFault)
        } else if self.finger_down {
            Some(Refused::FingerDown)
        } else if !self.ready {
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

/// Why the arm level dropped without a BOOT press (section 4.7).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dropped {
    /// The link went away.
    Link,
    /// No motor controller in the inbox for [`BOARDS_OFF_MS`].
    BoardsOff,
    /// A board the arm arms reported an arm-blocking bit: the bit and the board, kept for the
    /// statement until the next BOOT press (the app's `faultDisarm`).
    Fault { bit: u16, board: u8 },
}

/// The arm level and the press bookkeeping.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Arm {
    armed: bool,
    /// The press now held disarmed the machine, so its release does not arm.
    press_disarmed: bool,
    /// The last fault disarm's cause, until the next BOOT press.
    pub fault_disarm: Option<Dropped>,
    /// The last refused arm, until the next BOOT press.
    pub refused: Option<Refused>,
}

/// What a BOOT edge did.
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

    /// Apply a button edge, reading `cond` for an arm.
    pub fn on_edge(&mut self, e: Edge, cond: impl FnOnce() -> Conditions) -> Option<Action> {
        match e {
            Edge::Press => {
                self.fault_disarm = None;
                self.refused = None;
                if self.armed {
                    self.armed = false;
                    self.press_disarmed = true;
                    return Some(Action::Disarmed);
                }
                self.press_disarmed = false;
                None
            }
            Edge::Fault => None,
            Edge::Release { held } => {
                let disarmed = core::mem::take(&mut self.press_disarmed);
                if disarmed || self.armed || !(PRESS_MIN_MS..=PRESS_MAX_MS).contains(&held) {
                    return None;
                }
                match cond().refusal() {
                    Some(r) => {
                        self.refused = Some(r);
                        Some(Action::Refused(r))
                    }
                    None => {
                        self.armed = true;
                        Some(Action::Armed)
                    }
                }
            }
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

    /// Hold IO0 at `low` from `from` for `ms`, 2 ms a poll; the edges seen.
    fn hold(b: &mut Button, low: bool, from: u32, ms: u32) -> ([Option<Edge>; 2], u32) {
        let mut seen = [None, None];
        let mut k = 0;
        let mut t = from;
        while t < from + ms {
            if let Some(e) = b.poll(low, t) {
                if k < 2 {
                    seen[k] = Some(e);
                }
                k += 1;
            }
            t += 2;
        }
        (seen, t)
    }

    fn press(ms: u32) -> (Button, [Option<Edge>; 2], [Option<Edge>; 2]) {
        let mut b = Button::default();
        let (_, t) = hold(&mut b, false, 0, 100);
        let (down, t) = hold(&mut b, true, t, ms);
        let (up, _) = hold(&mut b, false, t, 100);
        (b, down, up)
    }

    #[test]
    fn the_press_length_window() {
        let (_, d, u) = press(48);
        assert_eq!(d[0], Some(Edge::Press));
        assert_eq!(u[0], Some(Edge::Release { held: 48 }));
        let (_, _, u) = press(50);
        assert_eq!(u[0], Some(Edge::Release { held: 50 }));
        let (_, _, u) = press(1500);
        assert_eq!(u[0], Some(Edge::Release { held: 1500 }));
        // Past 1.5 s: a fault, and no release edge.
        let (b, d, u) = press(1504);
        assert_eq!(d, [Some(Edge::Press), Some(Edge::Fault)]);
        assert_eq!(u[0], None);
        assert!(!b.fault(), "the fault ends when IO0 is high again");
    }

    #[test]
    fn a_bounce_shorter_than_the_debounce_is_not_a_press() {
        let mut b = Button::default();
        let (_, t) = hold(&mut b, false, 0, 100);
        let (e, t) = hold(&mut b, true, t, 18);
        assert_eq!(e[0], None);
        let (e, _) = hold(&mut b, false, t, 100);
        assert_eq!(e[0], None);
    }

    fn ok() -> Conditions {
        Conditions {
            button_fault: false,
            finger_down: false,
            ready: true,
            frames_differ: false,
            drive: None,
            board: true,
            blocked: false,
        }
    }

    #[test]
    fn the_window_arms_and_refuses_by_length() {
        for (held, arms) in [(49, false), (50, true), (1500, true), (1501, false)] {
            let mut a = Arm::default();
            a.on_edge(Edge::Press, ok);
            let r = a.on_edge(Edge::Release { held }, ok);
            assert_eq!(a.armed(), arms, "{held} ms");
            assert_eq!(r.is_some(), arms);
        }
    }

    #[test]
    fn each_refusal_alone_refuses() {
        type Case = (fn(&mut Conditions), Refused);
        let cases: [Case; 7] = [
            (|c| c.button_fault = true, Refused::ButtonFault),
            (|c| c.finger_down = true, Refused::FingerDown),
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
            let mut a = Arm::default();
            a.on_edge(Edge::Press, || c);
            assert_eq!(
                a.on_edge(Edge::Release { held: 100 }, || c),
                Some(Action::Refused(want))
            );
            assert!(!a.armed());
            assert_eq!(a.refused, Some(want));
        }
    }

    #[test]
    fn a_press_that_disarms_does_not_arm_and_disarm_is_never_refused() {
        let mut a = Arm::default();
        a.on_edge(Edge::Press, ok);
        a.on_edge(Edge::Release { held: 100 }, ok);
        assert!(a.armed());
        let bad = || Conditions {
            finger_down: true,
            blocked: true,
            ..ok()
        };
        assert_eq!(a.on_edge(Edge::Press, bad), Some(Action::Disarmed));
        assert!(!a.armed());
        assert_eq!(a.on_edge(Edge::Release { held: 100 }, ok), None);
        assert!(!a.armed());
    }

    #[test]
    fn a_touch_in_any_non_idle_state_at_release_refuses() {
        use crate::touch::Lift;
        for lift in [
            Lift::Pressing {
                n: 1,
                sum: 0,
                last: 0,
            },
            Lift::Down { y0: 0, yf: 0 },
            Lift::Lifting {
                y0: 0,
                yf: 0,
                since: 0,
                resume: 0,
            },
        ] {
            let c = Conditions {
                finger_down: lift.finger_down(),
                ..ok()
            };
            let mut a = Arm::default();
            a.on_edge(Edge::Press, || c);
            a.on_edge(Edge::Release { held: 100 }, || c);
            assert!(!a.armed(), "{lift:?}");
        }
    }

    #[test]
    fn a_fault_disarm_keeps_its_cause_until_the_next_press() {
        let mut a = Arm::default();
        a.on_edge(Edge::Press, ok);
        a.on_edge(Edge::Release { held: 100 }, ok);
        let why = Dropped::Fault {
            bit: linkctl::EV_LATCH_A,
            board: 2,
        };
        assert!(a.drop_for(why));
        assert!(!a.armed());
        assert_eq!(a.fault_disarm, Some(why));
        a.on_edge(Edge::Press, ok);
        assert_eq!(a.fault_disarm, None);
        // A link drop keeps no cause.
        a.on_edge(Edge::Release { held: 100 }, ok);
        assert!(a.drop_for(Dropped::Link));
        assert_eq!(a.fault_disarm, None);
        assert!(!a.drop_for(Dropped::Link), "already disarmed");
    }
}
