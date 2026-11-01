//! Which boards a drive mode drives (`specs/handheld-remote.md` section 4.6): the app's
//! `DriveInForce` (`apps/remote/.../model/Nodes.kt`) and `DriveFrame` (`model/DriveFrame.kt`)
//! with the steer axis fixed at 0, plus the handheld's own refusal of SINGLE with two motor
//! controllers.

use heapless::Vec;

/// The most motor controllers a drive addresses.
pub const MAX_BOARDS: usize = 2;

/// `machine.drive_mode`'s vocabulary (`shared/store/src/field.rs`, `MACHINE_DRIVE_MODE`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DriveMode {
    Single = 0,
    Bound = 1,
    Differential = 2,
    Rigid = 3,
}

impl DriveMode {
    /// The mode a stored byte names.
    pub fn of(byte: u8) -> Option<DriveMode> {
        match byte {
            0 => Some(DriveMode::Single),
            1 => Some(DriveMode::Bound),
            2 => Some(DriveMode::Differential),
            3 => Some(DriveMode::Rigid),
            _ => None,
        }
    }

    /// Whether the mode drives every motor controller found, and so needs exactly two.
    pub fn drives_both(self) -> bool {
        self != DriveMode::Single
    }
}

/// Why a drive is refused (the app's `DriveRefusal`, and [`Refusal::SingleWithTwo`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Refusal {
    /// A node's class, or a motor controller's stored mode, has not been read.
    Unread,
    /// The motor controllers store different modes.
    Disagree,
    /// The stored byte is not a known mode.
    UnknownMode,
    /// The stored mode drives two motor controllers and `found` were found.
    NotTwo { mode: DriveMode, found: u8 },
    /// RIGID, and not exactly one motor controller reports itself the leader.
    NoLeader,
    /// DIFFERENTIAL, and not exactly one motor controller is set as the right wheel.
    NoRightWheel,
    /// SINGLE with two motor controllers: the app drives the board picked on Tune or Setup, and
    /// the handheld has no picker.
    SingleWithTwo,
}

impl DriveMode {
    /// The mode's name as the app prints it (`DriveMode.name`).
    pub fn name(self) -> &'static str {
        match self {
            DriveMode::Single => "SINGLE",
            DriveMode::Bound => "BOUND",
            DriveMode::Differential => "DIFFERENTIAL",
            DriveMode::Rigid => "RIGID",
        }
    }
}

impl Refusal {
    /// The Ride statement (`specs/remote-ui.md` 3.1c's short forms; `apps/remote/app/src/main/res/
    /// values/strings.xml`, `drive_refusal_*`), and the handheld's own for
    /// [`Refusal::SingleWithTwo`].
    pub fn write(self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        match self {
            Refusal::Unread => out.write_str("Reading the drive mode from every board."),
            Refusal::Disagree => {
                out.write_str("Cannot arm: the boards store different drive modes.")
            }
            Refusal::UnknownMode => {
                out.write_str("Cannot arm: the boards store an unknown drive mode.")
            }
            Refusal::NotTwo { mode, found } => write!(
                out,
                "Cannot arm: {} needs two boards, {} found.",
                mode.name(),
                found
            ),
            Refusal::NoLeader => {
                out.write_str("Cannot arm: no single board leads the rigid frame.")
            }
            Refusal::NoRightWheel => {
                out.write_str("Cannot arm: no single board is the right wheel.")
            }
            Refusal::SingleWithTwo => {
                out.write_str("Cannot arm: SINGLE needs a board picked on the phone.")
            }
        }
    }
}

/// One motor controller as the drive rule reads it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Board {
    /// Its L3 address.
    pub addr: u8,
    /// Its stored `machine.drive_mode`, `None` while unread.
    pub mode: Option<u8>,
    /// Its stored `control.side`, `None` while unread.
    pub side: Option<u8>,
    /// Whether its own `CYCLIC_STATE` claims the rigid frame's lead (`flags` bit 1).
    pub leader: bool,
}

/// The boards a tick addresses: `drive` get a `DRIVE_CMD`, `arm` get the `INPUTS` arm level.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Targets {
    pub drive: Vec<u8, MAX_BOARDS>,
    pub arm: Vec<u8, MAX_BOARDS>,
}

/// `control.side`'s right wheel (`shared/store/src/field.rs`, `CONTROL_SIDE`: 0 left, 1 right).
pub const SIDE_RIGHT: u8 = 1;

/// The targets of the drive in force on a machine whose motor controllers are `mcs` (sorted by
/// address), or why it is refused. `class_unread` is whether any node's class is still unread.
/// No motor controller and nothing unread is `Ok` with no targets (nothing to drive and nothing to
/// say: "No motor controller found yet." comes from the arm rule).
pub fn targets(mcs: &[Board], class_unread: bool) -> Result<Targets, Refusal> {
    let mut t = Targets::default();
    if class_unread || mcs.iter().any(|b| b.mode.is_none()) {
        return Err(Refusal::Unread);
    }
    let Some(first) = mcs.first() else {
        return Ok(t);
    };
    let byte = first.mode.unwrap_or(0);
    if mcs.iter().any(|b| b.mode != Some(byte)) {
        return Err(Refusal::Disagree);
    }
    let mode = DriveMode::of(byte).ok_or(Refusal::UnknownMode)?;
    if !mode.drives_both() {
        if mcs.len() != 1 {
            return Err(Refusal::SingleWithTwo);
        }
        let _ = t.drive.push(first.addr);
        let _ = t.arm.push(first.addr);
        return Ok(t);
    }
    if mcs.len() != 2 {
        return Err(Refusal::NotTwo {
            mode,
            found: mcs.len() as u8,
        });
    }
    match mode {
        DriveMode::Rigid => {
            let mut leaders = mcs.iter().filter(|b| b.leader);
            let (Some(lead), None) = (leaders.next(), leaders.next()) else {
                return Err(Refusal::NoLeader);
            };
            let _ = t.drive.push(lead.addr);
        }
        DriveMode::Differential => {
            let right = mcs.iter().filter(|b| b.side == Some(SIDE_RIGHT)).count();
            if right != 1 || mcs.iter().any(|b| b.side.is_none()) {
                return Err(Refusal::NoRightWheel);
            }
            // The app's mix with s = 0 is left = right = v: the same value to both.
            t.drive = mcs.iter().map(|b| b.addr).collect();
        }
        _ => t.drive = mcs.iter().map(|b| b.addr).collect(),
    }
    t.arm = mcs.iter().map(|b| b.addr).collect();
    Ok(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn b(addr: u8, mode: u8) -> Board {
        Board {
            addr,
            mode: Some(mode),
            side: Some(0),
            leader: false,
        }
    }

    fn v(a: &[u8]) -> Vec<u8, MAX_BOARDS> {
        Vec::from_slice(a).unwrap()
    }

    #[test]
    fn the_section_4_6_table() {
        // SINGLE: the one motor controller.
        let t = targets(&[b(2, 0)], false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[2]), v(&[2])));
        // BOUND: both, both.
        let t = targets(&[b(2, 1), b(3, 1)], false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[2, 3]), v(&[2, 3])));
        // DIFFERENTIAL: both, the same value; both armed.
        let mut r = b(3, 2);
        r.side = Some(SIDE_RIGHT);
        let t = targets(&[b(2, 2), r], false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[2, 3]), v(&[2, 3])));
        // RIGID: the leader only; both armed.
        let mut lead = b(3, 3);
        lead.leader = true;
        let t = targets(&[b(2, 3), lead], false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[3]), v(&[2, 3])));
    }

    #[test]
    fn each_refusal() {
        assert_eq!(targets(&[b(2, 1)], true), Err(Refusal::Unread));
        let mut u = b(3, 1);
        u.mode = None;
        assert_eq!(targets(&[b(2, 1), u], false), Err(Refusal::Unread));
        assert_eq!(targets(&[b(2, 1), b(3, 3)], false), Err(Refusal::Disagree));
        assert_eq!(targets(&[b(2, 9)], false), Err(Refusal::UnknownMode));
        assert_eq!(
            targets(&[b(2, 1)], false),
            Err(Refusal::NotTwo {
                mode: DriveMode::Bound,
                found: 1
            })
        );
        assert_eq!(targets(&[b(2, 3), b(3, 3)], false), Err(Refusal::NoLeader));
        let (mut l1, mut l2) = (b(2, 3), b(3, 3));
        l1.leader = true;
        l2.leader = true;
        assert_eq!(targets(&[l1, l2], false), Err(Refusal::NoLeader));
        assert_eq!(
            targets(&[b(2, 2), b(3, 2)], false),
            Err(Refusal::NoRightWheel)
        );
        let mut s = b(3, 2);
        s.side = None;
        assert_eq!(targets(&[b(2, 2), s], false), Err(Refusal::NoRightWheel));
        assert_eq!(
            targets(&[b(2, 0), b(3, 0)], false),
            Err(Refusal::SingleWithTwo)
        );
        assert_eq!(targets(&[], false), Ok(Targets::default()));
    }

    fn text(r: Refusal) -> heapless::String<80> {
        let mut s = heapless::String::new();
        r.write(&mut s).unwrap();
        s
    }

    #[test]
    fn the_statements_are_the_apps() {
        assert_eq!(
            text(Refusal::Unread),
            "Reading the drive mode from every board."
        );
        assert_eq!(
            text(Refusal::NotTwo {
                mode: DriveMode::Rigid,
                found: 1
            }),
            "Cannot arm: RIGID needs two boards, 1 found."
        );
        assert_eq!(
            text(Refusal::NoRightWheel),
            "Cannot arm: no single board is the right wheel."
        );
        assert_eq!(
            text(Refusal::SingleWithTwo),
            "Cannot arm: SINGLE needs a board picked on the phone."
        );
        for r in [Refusal::Disagree, Refusal::UnknownMode, Refusal::NoLeader] {
            assert!(text(r).starts_with("Cannot arm: "));
        }
    }
}
