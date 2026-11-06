//! Which boards a drive mode drives (`specs/handheld-remote.md` section 4.6): the app's
//! `DriveInForce` (`apps/remote/.../model/Nodes.kt`) and `DriveFrame` (`model/DriveFrame.kt`). The
//! drive mode is derived, not stored (`specs/machine-settings.md` section 7): RIGID is
//! `machine.mode` balance with `machine.frame` rigid, SINGLE is one motor controller, and two or
//! more that are not rigid are DIFFERENTIAL with the turn axis and BOUND without. The turn is spent
//! only in DIFFERENTIAL, as each wheel's share of the mix (section 4.8); `steer` on the wire is 0 in
//! every mode.

use heapless::Vec;

/// The most motor controllers a drive addresses.
pub const MAX_BOARDS: usize = 2;

/// The drive modes (the app's `DriveMode`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DriveMode {
    Single,
    Bound,
    Differential,
    Rigid,
}

/// `machine.mode`'s balance byte (`shared/store/src/field.rs`, `CONTROL_MODE`: 0 throttle, 1
/// balance; the firmware runs every other byte as throttle).
pub const MODE_BALANCE: u8 = 1;
/// `machine.frame`'s rigid byte (`shared/store/src/field.rs`, `CONTROL_FRAME`: 0 pivoting, 1
/// rigid; the firmware runs every other byte as pivoting).
pub const FRAME_RIGID: u8 = 1;

impl DriveMode {
    /// Whether the mode drives every motor controller found, and so needs exactly two.
    pub fn drives_both(self) -> bool {
        self != DriveMode::Single
    }
}

/// Why a drive is refused (the app's `DriveRefusal`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Refusal {
    /// A node's class, or a motor controller's `machine.mode` or `machine.frame`, has not been read.
    Unread,
    /// The motor controllers hold different `machine.mode` or `machine.frame` copies.
    Disagree,
    /// The derived mode drives two motor controllers and `found` were found.
    NotTwo { mode: DriveMode, found: u8 },
    /// RIGID, and not exactly one motor controller reports itself the leader.
    NoLeader,
    /// DIFFERENTIAL, and not exactly one motor controller is set as the right wheel.
    NoRightWheel,
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
    /// values/strings.xml`, `drive_refusal_*`).
    pub fn write(self, out: &mut impl core::fmt::Write) -> core::fmt::Result {
        match self {
            Refusal::Unread => out.write_str("Reading the control mode and frame."),
            Refusal::Disagree => {
                out.write_str("Cannot arm: the boards store different control modes or frames.")
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
        }
    }
}

/// One motor controller as the drive rule reads it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Board {
    /// Its L3 address.
    pub addr: u8,
    /// Its stored `machine.mode`, `None` while unread.
    pub mode: Option<u8>,
    /// Its stored `machine.frame`, `None` while unread.
    pub frame: Option<u8>,
    /// Its stored `control.side`, `None` while unread.
    pub side: Option<u8>,
    /// Whether its own `CYCLIC_STATE` claims the rigid frame's lead (`flags` bit 1).
    pub leader: bool,
}

/// The boards a tick addresses: `drive` get a `DRIVE_CMD`, `arm` get the `INPUTS` arm level. In
/// DIFFERENTIAL, `right` is the board whose `control.side` is right: it gets the mix's right
/// wheel, the other board the left wheel.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Targets {
    pub drive: Vec<u8, MAX_BOARDS>,
    pub arm: Vec<u8, MAX_BOARDS>,
    pub right: Option<u8>,
}

impl Targets {
    /// Whether the turn is spent (DIFFERENTIAL).
    pub fn steers(&self) -> bool {
        self.right.is_some()
    }

    /// Each driven board's demand for a forward demand `value` and a turn `turn`: in DIFFERENTIAL
    /// its wheel of the mix (section 4.8), otherwise `value` with the turn ignored.
    pub fn demands(&self, value: i16, turn: i16) -> Vec<(u8, i16), MAX_BOARDS> {
        let pair = crate::mix::of(i32::from(value), i32::from(turn), crate::mix::FULL_SCALE);
        self.drive
            .iter()
            .map(|&addr| {
                let d = match self.right {
                    None => value,
                    // The mix holds each wheel inside +-32767.
                    Some(r) if r == addr => pair.right as i16,
                    Some(_) => pair.left as i16,
                };
                (addr, d)
            })
            .collect()
    }
}

/// `side`'s right wheel (motor 0's, `shared/store/src/field.rs`, `CONTROL_SIDE`: 0 left, 1 right).
pub const SIDE_RIGHT: u8 = 1;

/// Whether the handheld uses the turn axis on a machine of two or more motor controllers that is
/// not rigid: DIFFERENTIAL with it, BOUND without. The app keeps this as a Ride-screen setting
/// (`LinkSettings.turnAxis`, off until set); the handheld has no such setting and takes the app's
/// default.
pub const TURN_AXIS: bool = false;

/// The mode a machine whose motor controllers are `mcs` derives to, or why it cannot be trusted
/// (the app's `DriveInForce.derive`). `None` with no refusal on a machine with no motor
/// controller.
fn derive(
    mcs: &[Board],
    class_unread: bool,
    turn_axis: bool,
) -> Result<Option<DriveMode>, Refusal> {
    if class_unread || mcs.iter().any(|b| b.mode.is_none() || b.frame.is_none()) {
        return Err(Refusal::Unread);
    }
    // Effective values: every mode byte but balance runs as throttle, every frame byte but rigid
    // as pivoting.
    let balance = |b: &Board| b.mode == Some(MODE_BALANCE);
    let rigid = |b: &Board| b.frame == Some(FRAME_RIGID);
    let Some(first) = mcs.first() else {
        return Ok(None);
    };
    if mcs
        .iter()
        .any(|b| balance(b) != balance(first) || rigid(b) != rigid(first))
    {
        return Err(Refusal::Disagree);
    }
    Ok(Some(if balance(first) && rigid(first) {
        DriveMode::Rigid
    } else if mcs.len() == 1 {
        DriveMode::Single
    } else if turn_axis {
        DriveMode::Differential
    } else {
        DriveMode::Bound
    }))
}

/// The mode the machine whose motor controllers are `mcs` derives to, when every copy it is
/// derived from is read and agrees.
pub fn mode(mcs: &[Board], class_unread: bool, turn_axis: bool) -> Option<DriveMode> {
    derive(mcs, class_unread, turn_axis).ok().flatten()
}

/// The targets of the drive in force on a machine whose motor controllers are `mcs` (sorted by
/// address), or why it is refused. `class_unread` is whether any node's class is still unread;
/// `turn_axis` chooses DIFFERENTIAL over BOUND ([`TURN_AXIS`]). No motor controller and nothing
/// unread is `Ok` with no targets (nothing to drive and nothing to say: "No motor controller found
/// yet." comes from the arm rule).
pub fn targets(mcs: &[Board], class_unread: bool, turn_axis: bool) -> Result<Targets, Refusal> {
    let mut t = Targets::default();
    let Some(mode) = derive(mcs, class_unread, turn_axis)? else {
        return Ok(t);
    };
    if !mode.drives_both() {
        let _ = t.drive.push(mcs[0].addr);
        let _ = t.arm.push(mcs[0].addr);
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
            let mut rights = mcs.iter().filter(|b| b.side == Some(SIDE_RIGHT));
            let (Some(right), None) = (rights.next(), rights.next()) else {
                return Err(Refusal::NoRightWheel);
            };
            if mcs.iter().any(|b| b.side.is_none()) {
                return Err(Refusal::NoRightWheel);
            }
            t.right = Some(right.addr);
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

    /// A motor controller at `addr` whose copies derive to `mode` on a machine of two (SINGLE on
    /// one): `machine.mode` and `machine.frame` both balance and rigid for RIGID, both throttle and
    /// pivoting otherwise.
    fn b(addr: u8, mode: DriveMode) -> Board {
        let rigid = mode == DriveMode::Rigid;
        Board {
            addr,
            mode: Some(if rigid { MODE_BALANCE } else { 0 }),
            frame: Some(if rigid { FRAME_RIGID } else { 0 }),
            side: Some(0),
            leader: false,
        }
    }

    use DriveMode::{Bound, Differential, Rigid, Single};

    fn v(a: &[u8]) -> Vec<u8, MAX_BOARDS> {
        Vec::from_slice(a).unwrap()
    }

    #[test]
    fn the_section_4_6_table() {
        // SINGLE: the one motor controller.
        let t = targets(&[b(2, Single)], false, false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[2]), v(&[2])));
        // BOUND: both, both.
        let t = targets(&[b(2, Bound), b(3, Bound)], false, false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[2, 3]), v(&[2, 3])));
        // DIFFERENTIAL: both, the right wheel named; both armed.
        let mut r = b(3, Differential);
        r.side = Some(SIDE_RIGHT);
        let t = targets(&[b(2, Differential), r], false, true).unwrap();
        assert_eq!(t.right, Some(3));
        assert_eq!((t.drive, t.arm), (v(&[2, 3]), v(&[2, 3])));
        // RIGID: the leader only; both armed.
        let mut lead = b(3, Rigid);
        lead.leader = true;
        let t = targets(&[b(2, Rigid), lead], false, false).unwrap();
        assert_eq!((t.drive, t.arm), (v(&[3]), v(&[2, 3])));
        assert_eq!(t.right, None, "only DIFFERENTIAL names a right wheel");
    }

    #[test]
    fn the_mode_is_derived_from_the_mode_and_the_frame() {
        let m = |mode_byte: u8, frame: u8, n: usize, turn: bool| {
            let one = Board {
                addr: 2,
                mode: Some(mode_byte),
                frame: Some(frame),
                side: Some(0),
                leader: false,
            };
            mode(&[one, Board { addr: 3, ..one }][..n], false, turn)
        };
        assert_eq!(m(MODE_BALANCE, FRAME_RIGID, 2, false), Some(Rigid));
        assert_eq!(m(MODE_BALANCE, FRAME_RIGID, 1, true), Some(Rigid));
        // Balance on pivoting platforms, and throttle on a rigid body, are not RIGID.
        assert_eq!(m(MODE_BALANCE, 0, 2, false), Some(Bound));
        assert_eq!(m(0, FRAME_RIGID, 2, true), Some(Differential));
        assert_eq!(m(0, 0, 1, true), Some(Single));
        // An unknown byte runs as throttle and pivoting.
        assert_eq!(m(9, 9, 2, false), Some(Bound));
        assert_eq!(mode(&[], false, false), None);
    }

    #[test]
    fn the_right_wheels_demand_goes_to_the_board_whose_side_is_right() {
        // The right wheel at the lower address, so the order of `drive` does not decide it.
        let mut r = b(2, Differential);
        r.side = Some(SIDE_RIGHT);
        let t = targets(&[r, b(3, Differential)], false, true).unwrap();
        assert_eq!(t.right, Some(2));
        // Positive turn: the left wheel faster (`DriveMix`, a turn to the right).
        let d = t.demands(4000, 2000);
        assert_eq!(d.as_slice(), &[(2, 3000), (3, 5000)]);
        // No turn: both the same.
        assert_eq!(t.demands(4000, 0).as_slice(), &[(2, 4000), (3, 4000)]);
        // Full forward and full right: the offset keeps the difference.
        assert_eq!(t.demands(32767, 32767).as_slice(), &[(2, 1), (3, 32767)]);
    }

    #[test]
    fn bound_and_rigid_ignore_the_turn() {
        let t = targets(&[b(2, Bound), b(3, Bound)], false, false).unwrap();
        assert_eq!(t.demands(4000, 20_000).as_slice(), &[(2, 4000), (3, 4000)]);
        let mut lead = b(3, Rigid);
        lead.leader = true;
        let t = targets(&[b(2, Rigid), lead], false, true).unwrap();
        assert_eq!(t.demands(-500, -9000).as_slice(), &[(3, -500)]);
        let t = targets(&[b(2, Single)], false, true).unwrap();
        assert_eq!(t.demands(700, 30_000).as_slice(), &[(2, 700)]);
    }

    #[test]
    fn each_refusal() {
        assert_eq!(targets(&[b(2, Single)], true, false), Err(Refusal::Unread));
        let (mut u, mut f) = (b(3, Bound), b(3, Bound));
        u.mode = None;
        f.frame = None;
        assert_eq!(
            targets(&[b(2, Bound), u], false, false),
            Err(Refusal::Unread)
        );
        assert_eq!(
            targets(&[b(2, Bound), f], false, false),
            Err(Refusal::Unread)
        );
        assert_eq!(
            targets(&[b(2, Bound), b(3, Rigid)], false, false),
            Err(Refusal::Disagree)
        );
        // Balance on one board and throttle on the other, both pivoting.
        let mut bal = b(3, Bound);
        bal.mode = Some(MODE_BALANCE);
        assert_eq!(
            targets(&[b(2, Bound), bal], false, false),
            Err(Refusal::Disagree)
        );
        let mut lone = b(2, Rigid);
        lone.leader = true;
        assert_eq!(
            targets(&[lone], false, false),
            Err(Refusal::NotTwo {
                mode: Rigid,
                found: 1
            })
        );
        assert_eq!(
            targets(&[b(2, Rigid), b(3, Rigid)], false, false),
            Err(Refusal::NoLeader)
        );
        let (mut l1, mut l2) = (b(2, Rigid), b(3, Rigid));
        l1.leader = true;
        l2.leader = true;
        assert_eq!(targets(&[l1, l2], false, false), Err(Refusal::NoLeader));
        assert_eq!(
            targets(&[b(2, Differential), b(3, Differential)], false, true),
            Err(Refusal::NoRightWheel)
        );
        let mut s = b(3, Differential);
        s.side = None;
        assert_eq!(
            targets(&[b(2, Differential), s], false, true),
            Err(Refusal::NoRightWheel)
        );
        assert_eq!(targets(&[], false, false), Ok(Targets::default()));
    }

    fn text(r: Refusal) -> heapless::String<80> {
        let mut s = heapless::String::new();
        r.write(&mut s).unwrap();
        s
    }

    #[test]
    fn the_statements_are_the_apps() {
        assert_eq!(text(Refusal::Unread), "Reading the control mode and frame.");
        assert_eq!(
            text(Refusal::Disagree),
            "Cannot arm: the boards store different control modes or frames."
        );
        assert_eq!(
            text(Refusal::NotTwo {
                mode: Rigid,
                found: 1
            }),
            "Cannot arm: RIGID needs two boards, 1 found."
        );
        assert_eq!(
            text(Refusal::NoRightWheel),
            "Cannot arm: no single board is the right wheel."
        );
        assert!(text(Refusal::NoLeader).starts_with("Cannot arm: "));
    }
}
