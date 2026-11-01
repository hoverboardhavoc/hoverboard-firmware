//! What the handheld sends while it can drive (`specs/handheld-remote.md` section 3.6): the app's
//! `CommandPump` (`apps/remote/.../ble/CommandPump.kt`, `LinkConfig.kt`) in behaviour, with the
//! release send added, and the latest-wins outbox the link drains.

use heapless::Vec;
use linkctl::{DriveCmd, DriveKind, Inputs, OP_DRIVE_CMD, OP_INPUTS};
use net::pdu::Pdu;
use net::walk::{PduBuf, MAX_PDU};

use crate::drive::{Targets, MAX_BOARDS};

/// The send period, ms (the app's `LinkConfig.SEND_INTERVAL_MS`).
pub const TICK_MS: u32 = 50;
/// Ticks an `INPUTS` change is sent on (the app's `INPUTS_CHANGE_REPEATS`).
pub const INPUTS_CHANGE_REPEATS: u8 = 3;
/// Ticks between `INPUTS` keepalives (the app's `INPUTS_KEEPALIVE_TICKS`, 500 ms).
pub const INPUTS_KEEPALIVE_TICKS: u8 = 10;
/// The boards' control tick, ms: the 250 Hz loop the `linkctl` timeouts count in
/// (`shared/linkctl/src/lib.rs`, `DRIVE_TIMEOUT_TICKS` is 200 ms at 50 ticks).
pub const CONTROL_TICK_MS: u32 = 4;

// Pinned against the boards' timeouts (section 4.7; the app's
// `theArmMirrorWindowOutlastsTheAppsKeepalive`, `RustSourceDriftTest.kt`): the send period is
// under half the drive timeout, and three keepalive periods fit the arm mirror's window.
const _: () = assert!(2 * TICK_MS < linkctl::DRIVE_TIMEOUT_TICKS * CONTROL_TICK_MS);
const _: () = assert!(
    3 * INPUTS_KEEPALIVE_TICKS as u32 * TICK_MS <= linkctl::INPUTS_TIMEOUT_TICKS * CONTROL_TICK_MS
);

/// What the rider asks for on this tick: the arm level, and each driven board's demand (only read
/// while armed; a board not listed is demanded zero).
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Command {
    pub armed: bool,
    pub demands: Vec<(u8, i16), MAX_BOARDS>,
}

impl Command {
    /// The command while armed with every board in `demands` at its demand.
    pub fn armed(demands: Vec<(u8, i16), MAX_BOARDS>) -> Command {
        Command {
            armed: true,
            demands,
        }
    }

    /// The demand for board `dst`.
    pub fn demand_of(&self, dst: u8) -> i16 {
        self.demands
            .iter()
            .find(|(d, _)| *d == dst)
            .map_or(0, |(_, v)| *v)
    }

    /// The `DRIVE_CMD` this command sends to `dst`: Throttle with its demand while armed (zero
    /// included), Neutral 0 while disarmed. Steer is always 0: the turn is in the per-board
    /// demands (section 4.8), and the firmware's own steer mixer stays inert.
    pub fn drive(&self, dst: u8) -> DriveCmd {
        if self.armed {
            DriveCmd {
                kind: DriveKind::Throttle,
                value: self.demand_of(dst),
                steer: 0,
            }
        } else {
            DriveCmd {
                kind: DriveKind::Neutral,
                value: 0,
                steer: 0,
            }
        }
    }

    /// The `INPUTS` this command sends: the arm level on buttons bit 0, `rider` 0.
    pub fn inputs(&self) -> Inputs {
        Inputs {
            buttons: if self.armed { Inputs::BUTTON_POWER } else { 0 },
            rider: 0,
        }
    }

    /// Whether going from `self` to `next` is a release: any board's demand to zero, or a disarm.
    fn released_by(&self, next: &Command) -> bool {
        self.armed
            && (!next.armed
                || self
                    .demands
                    .iter()
                    .any(|&(dst, v)| v != 0 && next.demand_of(dst) == 0))
    }
}

/// What one poll of the pump did.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tick {
    /// Nothing was due.
    Idle,
    /// A tick on the 50 ms cadence.
    Cadence,
    /// A release send: a board's demand went to zero, or a disarm, sent at once.
    Release,
}

impl Tick {
    /// Whether frames went into the outbox.
    pub fn sent(self) -> bool {
        self != Tick::Idle
    }
}

/// The frames due, at most one of each kind per board: a newer one replaces an unsent older one,
/// so a stalled link never delivers a backlog of old demands.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Outbox {
    pub drive: Vec<(u8, DriveCmd), MAX_BOARDS>,
    pub inputs: Vec<(u8, Inputs), MAX_BOARDS>,
}

fn put<T>(v: &mut Vec<(u8, T), MAX_BOARDS>, dst: u8, item: T) {
    if let Some(slot) = v.iter_mut().find(|(d, _)| *d == dst) {
        slot.1 = item;
    } else {
        let _ = v.push((dst, item));
    }
}

impl Outbox {
    /// Whether nothing is due.
    pub fn is_empty(&self) -> bool {
        self.drive.is_empty() && self.inputs.is_empty()
    }

    /// Take every frame due as PDUs from `src`, `DRIVE_CMD` before `INPUTS` (a disarm's Neutral
    /// and its arm level 0 go out in the same write).
    pub fn take(&mut self, src: u8) -> Vec<PduBuf, { 2 * MAX_BOARDS }> {
        let mut out = Vec::new();
        for (dst, d) in self.drive.iter() {
            let mut p = [0u8; DriveCmd::LEN];
            let n = d.encode(&mut p);
            if let Some(b) = encode(OP_DRIVE_CMD, src, *dst, &p[..n]) {
                let _ = out.push(b);
            }
        }
        for (dst, i) in self.inputs.iter() {
            let mut p = [0u8; Inputs::LEN];
            let n = i.encode(&mut p);
            if let Some(b) = encode(OP_INPUTS, src, *dst, &p[..n]) {
                let _ = out.push(b);
            }
        }
        self.drive.clear();
        self.inputs.clear();
        out
    }
}

/// One control PDU.
fn encode(op: u8, src: u8, dst: u8, payload: &[u8]) -> Option<PduBuf> {
    let mut tmp = [0u8; MAX_PDU];
    let n = Pdu::new(op, src, dst, payload)
        .ok()?
        .encode(&mut tmp)
        .ok()?;
    Vec::from_slice(&tmp[..n]).ok()
}

/// One armed board's `INPUTS` bookkeeping.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Book {
    addr: u8,
    level: bool,
    repeats_left: u8,
    since: u8,
}

/// The pump's schedule.
#[derive(Clone, Debug, Default)]
pub struct Pump {
    next: Option<u32>,
    last: Command,
    books: Vec<Book, MAX_BOARDS>,
}

impl Pump {
    /// Forget everything: a new connection starts a new schedule, disarmed.
    pub fn reset(&mut self) {
        *self = Pump::default();
    }

    /// Run the schedule at `now` (ms) for `cmd` to `targets`, putting what is due in `out`: a tick
    /// on the 50 ms cadence, or at once on a release (any board's demand going to zero, or a
    /// disarm), from which the cadence continues.
    pub fn poll(&mut self, now: u32, cmd: Command, targets: &Targets, out: &mut Outbox) -> Tick {
        let release = self.last.released_by(&cmd);
        let due = self.next.is_none_or(|t| now.wrapping_sub(t) as i32 >= 0);
        if !(due || release) {
            self.last = cmd;
            return Tick::Idle;
        }
        self.next = Some(now.wrapping_add(TICK_MS));
        for &dst in targets.drive.iter() {
            put(&mut out.drive, dst, cmd.drive(dst));
        }
        self.books.retain(|b| targets.arm.contains(&b.addr));
        for &dst in targets.arm.iter() {
            let book = match self.books.iter_mut().position(|b| b.addr == dst) {
                Some(i) => &mut self.books[i],
                None => {
                    let _ = self.books.push(Book {
                        addr: dst,
                        level: !cmd.armed,
                        repeats_left: 0,
                        since: 0,
                    });
                    let i = self.books.len() - 1;
                    &mut self.books[i]
                }
            };
            if book.level != cmd.armed {
                book.level = cmd.armed;
                book.repeats_left = INPUTS_CHANGE_REPEATS;
            }
            let send = if book.repeats_left > 0 {
                book.repeats_left -= 1;
                true
            } else {
                book.since += 1;
                book.since >= INPUTS_KEEPALIVE_TICKS
            };
            if send {
                book.since = 0;
                put(&mut out.inputs, dst, cmd.inputs());
            }
        }
        self.last = cmd;
        if release {
            Tick::Release
        } else {
            Tick::Cadence
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec as StdVec;

    fn both() -> Targets {
        Targets {
            drive: Vec::from_slice(&[2, 3]).unwrap(),
            arm: Vec::from_slice(&[2, 3]).unwrap(),
            right: None,
        }
    }

    /// Armed, both boards at `v`.
    fn at(v: i16) -> Command {
        Command::armed(Vec::from_slice(&[(2, v), (3, v)]).unwrap())
    }

    /// Run the pump every 2 ms over `ms` from `start` with `cmd`; the times a tick went out and
    /// the outboxes they filled (taken at once, as a link that keeps up does).
    fn run(p: &mut Pump, start: u32, ms: u32, cmd: Command, t: &Targets) -> StdVec<(u32, Outbox)> {
        let mut ticks = StdVec::new();
        let mut now = start;
        while now < start + ms {
            let mut o = Outbox::default();
            if p.poll(now, cmd.clone(), t, &mut o).sent() {
                ticks.push((now, o));
            }
            now += 2;
        }
        ticks
    }

    #[test]
    fn the_cadence_is_fifty_ms() {
        let mut p = Pump::default();
        let ticks = run(&mut p, 0, 500, Command::default(), &both());
        let at: StdVec<u32> = ticks.iter().map(|(t, _)| *t).collect();
        assert_eq!(at, [0, 50, 100, 150, 200, 250, 300, 350, 400, 450]);
    }

    #[test]
    fn disarmed_frames_are_neutral() {
        let mut p = Pump::default();
        let ticks = run(&mut p, 0, 100, Command::default(), &both());
        for (_, o) in &ticks {
            for (_, d) in &o.drive {
                assert_eq!((d.kind, d.value, d.steer), (DriveKind::Neutral, 0, 0));
            }
            assert_eq!(o.drive.len(), 2);
        }
        let armed = at(0);
        assert_eq!(armed.drive(2).kind, DriveKind::Throttle);
        assert_eq!(armed.inputs().buttons, Inputs::BUTTON_POWER);
        assert_eq!(armed.inputs().rider, 0);
    }

    #[test]
    fn inputs_go_three_times_on_a_change_then_every_tenth_tick() {
        let mut p = Pump::default();
        let ticks = run(&mut p, 0, 2000, at(0), &both());
        let with: StdVec<usize> = ticks
            .iter()
            .enumerate()
            .filter(|(_, (_, o))| !o.inputs.is_empty())
            .map(|(i, _)| i)
            .collect();
        assert_eq!(with, [0, 1, 2, 12, 22, 32]);
        // Both armed boards each time.
        assert_eq!(ticks[0].1.inputs.len(), 2);
    }

    #[test]
    fn the_release_send_goes_out_at_once_and_the_cadence_resumes_from_it() {
        let mut p = Pump::default();
        let t = both();
        run(&mut p, 0, 120, at(9000), &t); // ticks at 0, 50, 100; next due at 150
        let mut o = Outbox::default();
        let release = at(0);
        assert_eq!(
            p.poll(121, release.clone(), &t, &mut o),
            Tick::Release,
            "the release goes at once"
        );
        assert_eq!(o.drive[0].1.value, 0);
        let ticks = run(&mut p, 122, 120, release, &t);
        let at: StdVec<u32> = ticks.iter().map(|(t, _)| *t).collect();
        assert_eq!(at, [172, 222]);
        // A disarm is a release too, and carries the arm level 0 in the same tick.
        let mut o = Outbox::default();
        assert_eq!(p.poll(223, Command::default(), &t, &mut o), Tick::Release);
        assert_eq!(o.drive[0].1.kind, DriveKind::Neutral);
        assert_eq!(o.inputs.len(), 2);
        assert_eq!(o.inputs[0].1.buttons, 0);
    }

    #[test]
    fn the_outbox_holds_one_pending_frame_per_board() {
        let mut p = Pump::default();
        let t = both();
        let mut o = Outbox::default();
        p.poll(0, at(1000), &t, &mut o);
        p.poll(50, at(2000), &t, &mut o);
        p.poll(100, at(3000), &t, &mut o);
        assert_eq!(o.drive.len(), 2);
        assert!(o.drive.iter().all(|(_, d)| d.value == 3000));
        let pdus = o.take(0x80);
        assert!(o.is_empty());
        let first = Pdu::decode(&pdus[0]).unwrap();
        assert_eq!(
            (first.opcode, first.src, first.dst),
            (OP_DRIVE_CMD, 0x80, 2)
        );
        assert_eq!(DriveCmd::decode(first.payload).unwrap().value, 3000);
        let last = Pdu::decode(pdus.last().unwrap()).unwrap();
        assert_eq!(last.opcode, OP_INPUTS);
    }

    #[test]
    fn rigid_drives_the_leader_and_arms_both() {
        let mut p = Pump::default();
        let t = Targets {
            drive: Vec::from_slice(&[3]).unwrap(),
            arm: Vec::from_slice(&[2, 3]).unwrap(),
            right: None,
        };
        let mut o = Outbox::default();
        p.poll(
            0,
            Command::armed(Vec::from_slice(&[(3, 5)]).unwrap()),
            &t,
            &mut o,
        );
        assert_eq!(o.drive.len(), 1);
        assert_eq!(o.drive[0].0, 3);
        assert_eq!(o.inputs.len(), 2);
    }

    #[test]
    fn any_boards_demand_going_to_zero_is_a_release_send() {
        let mut p = Pump::default();
        let t = both();
        let mut o = Outbox::default();
        let two = |a, b| Command::armed(Vec::from_slice(&[(2, a), (3, b)]).unwrap());
        assert_eq!(p.poll(0, two(9000, 7000), &t, &mut o), Tick::Cadence);
        // One wheel's share of the mix goes to zero, the other does not: a release, at once.
        assert_eq!(p.poll(10, two(9000, 0), &t, &mut o), Tick::Release);
        assert_eq!((o.drive[0].1.value, o.drive[1].1.value), (9000, 0));
        // A change between nonzero demands waits for the cadence.
        assert_eq!(p.poll(20, two(8000, 100), &t, &mut o), Tick::Idle);
        assert_eq!(p.poll(60, two(8000, 100), &t, &mut o), Tick::Cadence);
        // Both to zero (the touch lifted): a release.
        assert_eq!(p.poll(61, two(0, 0), &t, &mut o), Tick::Release);
        // A demand that only changes in between ticks is not lost for the release check: it is
        // the last command polled that counts.
        assert_eq!(p.poll(62, two(500, 500), &t, &mut o), Tick::Idle);
        assert_eq!(p.poll(63, two(500, 0), &t, &mut o), Tick::Release);
    }

    #[test]
    fn steer_on_the_wire_is_zero_in_every_mode() {
        let mut p = Pump::default();
        let t = both();
        let mut o = Outbox::default();
        p.poll(
            0,
            Command::armed(Vec::from_slice(&[(2, -32767), (3, 32767)]).unwrap()),
            &t,
            &mut o,
        );
        assert!(o.drive.iter().all(|(_, d)| d.steer == 0));
        assert_eq!(o.drive[0].1.value, -32767);
        let mut o = Outbox::default();
        p.poll(100, Command::default(), &t, &mut o);
        assert!(o.drive.iter().all(|(_, d)| d.steer == 0));
    }
}
