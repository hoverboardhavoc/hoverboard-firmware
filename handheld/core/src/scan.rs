//! The Connect screen (`specs/handheld-remote.md` section 5.8): the scan list, the advertised name
//! in an advertisement, and the tap that picks a row.
//!
//! Rows are one per address in first-heard order, at most [`ROWS`]. A row not heard for
//! [`SILENT_MS`] goes and leaves its slot empty, and the next new address heard takes the first
//! empty slot, so no row moves under a finger. A later advertisement never erases a name an earlier
//! one carried. The RSSI shown changes at most every [`RSSI_MS`].

use heapless::String;

use crate::pick::LABEL_MAX;
use crate::touch::{Lift, Point};

/// Rows on the screen.
pub const ROWS: usize = 8;
/// A row not heard for this long goes, ms.
pub const SILENT_MS: u32 = 10_000;
/// The shortest time between two RSSI figures a row shows, ms (twice a second).
pub const RSSI_MS: u32 = 500;
/// The rows' area and height (section 5.8): y 24..288, eight rows of 33 px.
pub const ROWS_Y: i32 = 24;
pub const ROW_H: i32 = 33;

/// One advertising address.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Row {
    /// Most significant octet first.
    pub addr: [u8; 6],
    pub name: Option<String<LABEL_MAX>>,
    /// The RSSI shown, dBm.
    pub rssi: i8,
    rssi_at: u32,
    heard: u32,
}

/// The scan list.
#[derive(Clone, Debug, Default)]
pub struct ScanList {
    rows: [Option<Row>; ROWS],
}

impl ScanList {
    /// The slots, top to bottom.
    pub fn rows(&self) -> &[Option<Row>; ROWS] {
        &self.rows
    }

    /// An advertisement from `addr` at `now` (ms), with the name it carried, if any.
    pub fn heard(&mut self, addr: [u8; 6], name: Option<&str>, rssi: i8, now: u32) {
        let name = name.map(|n| crate::pick::Pick::new(addr, n).label);
        if let Some(r) = self.rows.iter_mut().flatten().find(|r| r.addr == addr) {
            r.heard = now;
            if name.is_some() {
                r.name = name;
            }
            if now.wrapping_sub(r.rssi_at) >= RSSI_MS {
                r.rssi = rssi;
                r.rssi_at = now;
            }
            return;
        }
        if let Some(slot) = self.rows.iter_mut().find(|s| s.is_none()) {
            *slot = Some(Row {
                addr,
                name,
                rssi,
                rssi_at: now,
                heard: now,
            });
        }
    }

    /// Drop the rows not heard for [`SILENT_MS`] at `now`.
    pub fn expire(&mut self, now: u32) {
        for s in self.rows.iter_mut() {
            if s.as_ref()
                .is_some_and(|r| now.wrapping_sub(r.heard) >= SILENT_MS)
            {
                *s = None;
            }
        }
    }
}

/// The advertised name in an advertisement's data: the complete local name (AD type 0x09), else
/// the shortened one (0x08), as bytes.
pub fn adv_name(data: &[u8]) -> Option<&[u8]> {
    let mut short = None;
    let mut i = 0;
    while i < data.len() {
        let len = usize::from(data[i]);
        if len == 0 || i + 1 + len > data.len() {
            break;
        }
        let (ty, body) = (data[i + 1], &data[i + 2..i + 1 + len]);
        match ty {
            0x09 => return Some(body),
            0x08 => short = Some(body),
            _ => {}
        }
        i += 1 + len;
    }
    short
}

/// The row a point (sixteenths of a pixel) is on, if any.
pub fn row_at(p: Point) -> Option<usize> {
    let y = p.y / crate::touch::SUB - ROWS_Y;
    (0..ROW_H * ROWS as i32)
        .contains(&y)
        .then_some((y / ROW_H) as usize)
}

/// A tap (section 5.8): a touch whose touch-down point and every point after it, to its confirmed
/// release, fall on the same target. `T` names a target (a row, the Cancel button).
#[derive(Clone, Copy, Debug)]
pub struct Tap<T> {
    prev: Option<Lift>,
    /// The touch now down: its target, and whether it has stayed on it.
    on: Option<(Option<T>, bool)>,
}

impl<T> Default for Tap<T> {
    fn default() -> Self {
        Tap {
            prev: None,
            on: None,
        }
    }
}

impl<T: Copy + PartialEq> Tap<T> {
    /// Follow the lift machine's state after a sample; `target` names what a point is on. Returns
    /// the target tapped, on the sample that confirms the release (the one that leaves
    /// `Lifting`).
    pub fn step(&mut self, lift: Lift, target: impl Fn(Point) -> Option<T>) -> Option<T> {
        let prev = self.prev.replace(lift);
        match (lift.points(), &mut self.on) {
            (Some((p0, pf)), None) => {
                let t = target(p0);
                self.on = Some((t, t.is_some() && target(pf) == t));
                None
            }
            (Some((_, pf)), Some((t, stayed))) => {
                if target(pf) != *t {
                    *stayed = false;
                }
                None
            }
            (None, Some(_)) => {
                let (t, stayed) = self.on.take()?;
                let released = matches!(prev, Some(Lift::Lifting { .. }));
                (released && stayed).then_some(t).flatten()
            }
            (None, None) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::touch::SUB;

    fn a(n: u8) -> [u8; 6] {
        [0xe0, 0, 0, 0, 0, n]
    }

    fn addrs(l: &ScanList) -> [Option<u8>; ROWS] {
        l.rows().clone().map(|r| r.map(|r| r.addr[5]))
    }

    #[test]
    fn rows_by_first_heard_at_most_eight() {
        let mut l = ScanList::default();
        for n in 1..=10 {
            l.heard(a(n), None, -60, u32::from(n));
        }
        l.heard(a(3), Some("x"), -50, 20);
        assert_eq!(addrs(&l), [1, 2, 3, 4, 5, 6, 7, 8].map(Some));
    }

    #[test]
    fn a_row_silent_ten_seconds_goes_and_its_place_is_reused() {
        let mut l = ScanList::default();
        l.heard(a(1), None, -60, 0);
        l.heard(a(2), None, -60, 0);
        l.heard(a(3), None, -60, 0);
        l.heard(a(1), None, -60, 5000);
        l.heard(a(3), None, -60, 5000);
        l.expire(9999);
        assert_eq!(addrs(&l)[..3], [Some(1), Some(2), Some(3)]);
        l.expire(10_000);
        assert_eq!(addrs(&l)[..3], [Some(1), None, Some(3)]);
        l.heard(a(9), None, -70, 10_100);
        assert_eq!(addrs(&l)[..4], [Some(1), Some(9), Some(3), None]);
    }

    #[test]
    fn a_later_nameless_advertisement_keeps_the_earlier_name() {
        let mut l = ScanList::default();
        l.heard(a(1), Some("bridge"), -60, 0);
        l.heard(a(1), None, -61, 100);
        assert_eq!(
            l.rows()[0].as_ref().unwrap().name.as_deref(),
            Some("bridge")
        );
        l.heard(a(1), Some("renamed"), -61, 200);
        assert_eq!(
            l.rows()[0].as_ref().unwrap().name.as_deref(),
            Some("renamed")
        );
        // A name heard later fills a row first heard without one.
        l.heard(a(2), None, -60, 0);
        l.heard(a(2), Some("late"), -60, 300);
        assert_eq!(l.rows()[1].as_ref().unwrap().name.as_deref(), Some("late"));
    }

    #[test]
    fn the_rssi_shown_changes_at_most_twice_a_second() {
        let mut l = ScanList::default();
        l.heard(a(1), None, -60, 0);
        l.heard(a(1), None, -40, 499);
        assert_eq!(l.rows()[0].as_ref().unwrap().rssi, -60);
        l.heard(a(1), None, -41, 500);
        assert_eq!(l.rows()[0].as_ref().unwrap().rssi, -41);
    }

    #[test]
    fn the_name_comes_from_the_complete_or_the_shortened_local_name() {
        // Flags, then the complete local name.
        let adv = [2, 0x01, 0x06, 6, 0x09, b'h', b'o', b'v', b'e', b'r'];
        assert_eq!(adv_name(&adv), Some(&b"hover"[..]));
        let short = [2, 0x01, 0x06, 3, 0x08, b'h', b'o'];
        assert_eq!(adv_name(&short), Some(&b"ho"[..]));
        assert_eq!(adv_name(&[2, 0x01, 0x06]), None);
        assert_eq!(adv_name(&[9, 0x09, b'x']), None, "a truncated structure");
    }

    fn at(y: i32) -> Point {
        Point::new(100 * SUB, y * SUB)
    }

    fn down(p0: Point, pf: Point) -> Lift {
        Lift::Down { p0, pf }
    }

    fn lifting(p0: Point, pf: Point) -> Lift {
        Lift::Lifting {
            p0,
            pf,
            since: 0,
            resume: 0,
        }
    }

    #[test]
    fn rows_are_33_px_from_y_24() {
        assert_eq!(row_at(at(23)), None);
        assert_eq!(row_at(at(24)), Some(0));
        assert_eq!(row_at(at(56)), Some(0));
        assert_eq!(row_at(at(57)), Some(1));
        assert_eq!(row_at(at(287)), Some(7));
        assert_eq!(row_at(at(288)), None);
    }

    #[test]
    fn a_tap_picks_the_row_its_down_and_release_share() {
        let mut t: Tap<usize> = Tap::default();
        let p = at(70);
        assert_eq!(t.step(down(p, p), row_at), None);
        assert_eq!(t.step(down(p, at(72)), row_at), None);
        assert_eq!(t.step(lifting(p, at(72)), row_at), None);
        assert_eq!(t.step(Lift::Idle, row_at), Some(1));
    }

    #[test]
    fn a_drag_across_rows_picks_nothing() {
        let mut t: Tap<usize> = Tap::default();
        let p = at(70);
        t.step(down(p, p), row_at);
        t.step(down(p, at(100)), row_at);
        // Back on its own row before the release: still nothing.
        t.step(down(p, at(70)), row_at);
        t.step(lifting(p, at(70)), row_at);
        assert_eq!(t.step(Lift::Idle, row_at), None);
    }

    #[test]
    fn a_touch_that_never_went_down_or_lands_off_the_rows_picks_nothing() {
        let mut t: Tap<usize> = Tap::default();
        let o = Point::default();
        t.step(
            Lift::Pressing {
                n: 1,
                sum: o,
                last: o,
            },
            row_at,
        );
        assert_eq!(t.step(Lift::Idle, row_at), None);
        let p = at(10);
        t.step(down(p, p), row_at);
        t.step(lifting(p, p), row_at);
        assert_eq!(t.step(Lift::Idle, row_at), None);
    }
}
