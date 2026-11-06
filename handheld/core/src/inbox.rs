//! What each board has said (`specs/handheld-remote.md` section 3.5, "Telemetry"): every
//! `CYCLIC_STATE` filed by its L3 source, a source dropped [`QUIET_MS`] after its last frame (the
//! app's `TelemetryInbox`, `apps/remote/.../model/TelemetryInbox.kt`). The caller owns the clock.

use heapless::Vec;
use linkctl::CyclicState;

/// A source leaves the inbox this long after its last frame (the app's `QUIET_MS`), ms.
pub const QUIET_MS: u32 = 1200;
/// The most sources held.
pub const MAX_SOURCES: usize = 8;

/// One source's latest frame and when it was heard.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Heard {
    pub src: u8,
    pub state: CyclicState,
    pub at: u32,
}

/// The inbox, and a count of frames received for the debug line's rate.
#[derive(Clone, Debug, Default)]
pub struct Inbox {
    heard: Vec<Heard, MAX_SOURCES>,
    /// `CYCLIC_STATE` frames received since the last [`Inbox::take_count`].
    count: u32,
}

impl Inbox {
    /// File `state` from `src`, heard at `now`, latest wins.
    pub fn cyclic(&mut self, src: u8, state: CyclicState, now: u32) {
        self.count = self.count.wrapping_add(1);
        match self.heard.iter_mut().find(|h| h.src == src) {
            Some(h) => {
                h.state = state;
                h.at = now;
            }
            None => {
                let _ = self.heard.push(Heard {
                    src,
                    state,
                    at: now,
                });
            }
        }
    }

    /// Drop every source not heard from for [`QUIET_MS`].
    pub fn expire(&mut self, now: u32) {
        self.heard.retain(|h| now.wrapping_sub(h.at) < QUIET_MS);
    }

    /// Forget everything (the link went away).
    pub fn clear(&mut self) {
        self.heard.clear();
    }

    /// The latest frame from `src`, if it is in the inbox.
    pub fn get(&self, src: u8) -> Option<&CyclicState> {
        self.heard.iter().find(|h| h.src == src).map(|h| &h.state)
    }

    /// Every source in the inbox.
    pub fn sources(&self) -> impl Iterator<Item = &Heard> {
        self.heard.iter()
    }

    /// The frames received since the last call, and reset the count.
    pub fn take_count(&mut self) -> u32 {
        core::mem::take(&mut self.count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn st(pitch: i16) -> CyclicState {
        let mut b = [0u8; CyclicState::ENCODED_LEN];
        b[0..2].copy_from_slice(&pitch.to_le_bytes());
        CyclicState::decode(&b).unwrap()
    }

    #[test]
    fn frames_are_filed_by_source_latest_wins() {
        let mut i = Inbox::default();
        i.cyclic(2, st(10), 0);
        i.cyclic(3, st(20), 0);
        i.cyclic(2, st(11), 100);
        assert_eq!(i.get(2).unwrap().pitch, 11);
        assert_eq!(i.get(3).unwrap().pitch, 20);
        assert_eq!(i.sources().count(), 2);
        assert_eq!(i.take_count(), 3);
        assert_eq!(i.take_count(), 0);
    }

    #[test]
    fn a_source_leaves_quiet_ms_after_its_last_frame() {
        let mut i = Inbox::default();
        i.cyclic(2, st(0), 0);
        i.cyclic(3, st(0), 500);
        i.expire(QUIET_MS - 1);
        assert!(i.get(2).is_some());
        i.expire(QUIET_MS);
        assert!(i.get(2).is_none());
        assert!(i.get(3).is_some());
        i.clear();
        assert!(i.get(3).is_none());
    }
}
