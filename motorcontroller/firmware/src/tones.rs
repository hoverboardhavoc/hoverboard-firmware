//! The tone player and the battery alarm levels (`specs/tones.md` sections 1 and 4): pure logic,
//! run once per served 250 Hz tick from the control task, host-tested here.
//!
//! The player picks the highest-priority event whose level is on and whose tune has a playable
//! note, plays that tune's notes in order and loops it while the level holds, and hands back the
//! tone's period in period-ISR periods (`P`, [`period`]) for `motor::TONE_PERIOD`, 0 for silence.
//! The period ISR makes the waveform from that word.
//!
//! A tune is read through [`Tunes`] at each note start and only that note is kept: the firmware's
//! reader borrows flash through `Store::get_bytes`, and a config write can compact the page under
//! a slice held across ticks, so none is.

// The caller is the target-only control task in `main.rs`; the host build compiles this module for
// its tests alone.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use store::tune::{count, note_at, Note};

/// The events that sound a tune, highest priority first (`specs/tones.md` section 4.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// `INPUTS.buttons` bit1 held: `machine.tone.horn`.
    Horn,
    /// The battery word below `machine.battery.floor`: `machine.tone.battery_critical`.
    BatteryCritical,
    /// The battery word below `machine.battery.warn`: `machine.tone.battery_low`.
    BatteryLow,
}

/// Every event in priority order.
const EVENTS: [Event; 3] = [Event::Horn, Event::BatteryCritical, Event::BatteryLow];

/// The three event levels, one per [`Event`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Levels {
    pub horn: bool,
    pub battery_critical: bool,
    pub battery_low: bool,
}

impl Levels {
    /// The levels as bits, bit `k` for [`EVENTS`]`[k]`: one byte to compare and test.
    const fn bits(self) -> u8 {
        self.horn as u8 | (self.battery_critical as u8) << 1 | (self.battery_low as u8) << 2
    }
}

/// The reader of the three tunes: hands `f` the current bytes of `e`'s tune, which it may only
/// look at inside the call.
pub trait Tunes {
    fn with<R>(&self, e: Event, f: impl FnOnce(&[u8]) -> R) -> R;
}

/// The control tick, in milliseconds (250 Hz).
pub const TICK_MS: u32 = 4;

/// The shortest tone period, in ISR periods: a 50% square wave needs a high and a low period, so
/// the highest tone is half the ISR rate.
pub const P_MIN: u32 = 2;
/// The longest tone period, in ISR periods.
pub const P_MAX: u32 = 65_535;

/// The tone period for `hz` at an ISR rate of `rate_hz`: `round(rate / hz)` clamped to
/// [`P_MIN`]`..=`[`P_MAX`], or 0 (silence) for a rest.
pub const fn period(hz: u16, rate_hz: u32) -> u32 {
    if hz == 0 {
        return 0;
    }
    let hz = hz as u32;
    let p = (rate_hz + hz / 2) / hz;
    if p < P_MIN {
        P_MIN
    } else if p > P_MAX {
        P_MAX
    } else {
        p
    }
}

/// The first playable note of `tune` at or after note `from`, wrapping, with its index; `None` when
/// the tune has none (empty, or every `dur` 0). At most one pass over the notes.
fn next_playable(tune: &[u8], from: usize) -> Option<(usize, Note)> {
    let n = count(tune);
    if n == 0 {
        return None;
    }
    (0..n).map(|k| (from + k) % n).find_map(|i| {
        note_at(tune, i)
            .filter(|note| note.playable())
            .map(|note| (i, note))
    })
}

/// The note the player is sounding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Playing {
    event: Event,
    /// The note's index in its tune.
    index: u8,
    /// The note's pitch, 0 = a rest.
    hz: u16,
    /// The note's duration, ms.
    dur_ms: u16,
    /// Time spent in the note, ms.
    elapsed_ms: u32,
}

/// The tone player (`specs/tones.md` section 4.2).
pub struct Player {
    /// The period ISR's rate, Hz (`R`): the motor module's `period_isr_hz`.
    rate_hz: u32,
    playing: Option<Playing>,
    /// The levels of the previous tick ([`Levels::bits`]), so a change selects again at once.
    levels: u8,
}

impl Player {
    /// A silent player for an ISR running at `rate_hz`.
    pub const fn new(rate_hz: u32) -> Self {
        Player {
            rate_hz,
            playing: None,
            levels: 0,
        }
    }

    /// One tick: `dt_ms` elapsed since the last, `levels` now. Returns the tone period `P` (0 =
    /// silence).
    ///
    /// The selection runs when a level changes and when the sounding note ends; in between it
    /// reads nothing. A higher event that comes on preempts at once; when it goes off, the lower
    /// event starts its tune from the first note. An event whose tune has no playable note does not
    /// take over: a muted horn leaves the battery alarm playing.
    pub fn tick(&mut self, levels: Levels, dt_ms: u32, tunes: &impl Tunes) -> u32 {
        if let Some(p) = self.playing.as_mut() {
            p.elapsed_ms = p.elapsed_ms.saturating_add(dt_ms);
        }
        let note_over = self
            .playing
            .is_some_and(|p| p.elapsed_ms >= u32::from(p.dur_ms));
        let levels = levels.bits();
        if levels != self.levels || note_over {
            self.levels = levels;
            self.select(levels, note_over, tunes);
        }
        self.playing.map_or(0, |p| period(p.hz, self.rate_hz))
    }

    /// Choose what plays from here: the highest event that is on and has a playable note. The one
    /// already playing keeps its place mid-note, or moves to its next playable note when its note
    /// is over (carrying the overshoot, so a looping tune keeps its period).
    fn select(&mut self, levels: u8, note_over: bool, tunes: &impl Tunes) {
        for (k, &e) in EVENTS.iter().enumerate() {
            if levels & 1 << k == 0 {
                continue;
            }
            let current = self.playing.filter(|p| p.event == e);
            let (from, carry) = match current {
                Some(_) if !note_over => return,
                Some(p) => (usize::from(p.index) + 1, p.elapsed_ms - u32::from(p.dur_ms)),
                None => (0, 0),
            };
            if let Some((i, note)) = tunes.with(e, |t| next_playable(t, from)) {
                self.playing = Some(Playing {
                    event: e,
                    index: i as u8,
                    hz: note.hz,
                    dur_ms: note.ms() as u16,
                    elapsed_ms: carry,
                });
                return;
            }
        }
        self.playing = None;
    }
}

/// How long the battery condition must hold before its level comes on, ms (`specs/tones.md`
/// section 4.4).
pub const ALARM_ON_MS: u32 = 2_000;
/// How long the condition must stay clear before its level goes off, ms.
pub const ALARM_OFF_MS: u32 = 10_000;

/// One battery level with its hysteresis in time: on after its condition has held for
/// [`ALARM_ON_MS`] continuously, off after it has been clear for [`ALARM_OFF_MS`] continuously.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Filtered {
    on: bool,
    /// How long the condition has disagreed with `on`, ms.
    against_ms: u32,
}

impl Filtered {
    /// One tick of the level whose condition is the battery word `word` known (non-zero) and below
    /// `threshold`, with `threshold <= 0` = none. An UNKNOWN word turns the level off at once and
    /// starts its filter over.
    ///
    /// `#[inline(never)]`, with the condition and the UNKNOWN rule inside: one body for the two
    /// levels, each called with three arguments (flash, `specs/tones.md` section 6).
    #[inline(never)]
    fn step(&mut self, word: i16, threshold: i16, dt_ms: u32) -> bool {
        if word == 0 {
            *self = Filtered {
                on: false,
                against_ms: 0,
            };
            return false;
        }
        let cond = threshold > 0 && word < threshold;
        if cond == self.on {
            self.against_ms = 0;
        } else {
            self.against_ms = self.against_ms.saturating_add(dt_ms);
            let needed = if self.on { ALARM_OFF_MS } else { ALARM_ON_MS };
            if self.against_ms >= needed {
                self.on = cond;
                self.against_ms = 0;
            }
        }
        self.on
    }
}

/// The two battery levels (`specs/tones.md` section 4.4), from the block's effective battery word.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BatteryAlarm {
    critical: Filtered,
    low: Filtered,
}

impl BatteryAlarm {
    pub const fn new() -> Self {
        BatteryAlarm {
            critical: Filtered {
                on: false,
                against_ms: 0,
            },
            low: Filtered {
                on: false,
                against_ms: 0,
            },
        }
    }

    /// One tick: the battery word `word` (centivolts, 0 = UNKNOWN), the floor and the warning
    /// threshold (`<= 0` = none), `dt_ms` elapsed. Returns `(critical, low)`.
    ///
    /// An UNKNOWN word sounds nothing: both levels go off at once and their filters start over.
    pub fn step(&mut self, word: i16, floor: i16, warn: i16, dt_ms: u32) -> (bool, bool) {
        (
            self.critical.step(word, floor, dt_ms),
            self.low.step(word, warn, dt_ms),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::vec::Vec;

    const R: u32 = 16_000;

    /// Three tunes as bytes, counting the reads.
    struct Fake {
        horn: Vec<u8>,
        critical: Vec<u8>,
        low: Vec<u8>,
        reads: Cell<u32>,
    }

    fn tune(notes: &[(u16, u8)]) -> Vec<u8> {
        notes
            .iter()
            .flat_map(|&(hz, dur)| Note { hz, dur }.encode())
            .collect()
    }

    impl Fake {
        fn new(horn: &[(u16, u8)], critical: &[(u16, u8)], low: &[(u16, u8)]) -> Self {
            Fake {
                horn: tune(horn),
                critical: tune(critical),
                low: tune(low),
                reads: Cell::new(0),
            }
        }
    }

    impl Tunes for Fake {
        fn with<R>(&self, e: Event, f: impl FnOnce(&[u8]) -> R) -> R {
            self.reads.set(self.reads.get() + 1);
            f(match e {
                Event::Horn => &self.horn,
                Event::BatteryCritical => &self.critical,
                Event::BatteryLow => &self.low,
            })
        }
    }

    fn lv(horn: bool, critical: bool, low: bool) -> Levels {
        Levels {
            horn,
            battery_critical: critical,
            battery_low: low,
        }
    }

    /// The pitch each tick plays, over `ticks` ticks at `levels`.
    fn run(p: &mut Player, t: &Fake, levels: Levels, ticks: usize) -> Vec<u32> {
        (0..ticks).map(|_| p.tick(levels, TICK_MS, t)).collect()
    }

    #[test]
    fn pitch_rounds_and_clamps() {
        assert_eq!(period(0, R), 0);
        assert_eq!(period(2000, R), 8);
        assert_eq!(period(1000, R), 16);
        assert_eq!(period(500, R), 32);
        // 16000 / 3000 = 5.33 rounds to 5; 16000 / 2900 = 5.52 rounds to 6.
        assert_eq!(period(3000, R), 5);
        assert_eq!(period(2900, R), 6);
        // The clamp: above 8 kHz is P = 2, below the 16-bit period is 65535.
        assert_eq!(period(8000, R), 2);
        assert_eq!(period(12_000, R), 2);
        assert_eq!(period(1, 16_000_000), 65_535);
    }

    #[test]
    fn silence_with_no_level() {
        let t = Fake::new(&[(2000, 50)], &[(2000, 15)], &[(2000, 15)]);
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, Levels::default(), 10), [0; 10]);
        assert_eq!(t.reads.get(), 0, "nothing is read while silent");
    }

    #[test]
    fn a_tune_plays_in_order_and_loops() {
        // 2000 Hz for 20 ms, rest 10 ms, 1000 Hz 10 ms, then again.
        let t = Fake::new(&[(2000, 2), (0, 1), (1000, 1)], &[], &[]);
        let mut p = Player::new(R);
        let got = run(&mut p, &t, lv(true, false, false), 12);
        // Note 1 ends at the first tick whose elapsed time reaches 20 ms (tick 5 after the start,
        // 20 ms); the rest at 10 ms needs three ticks (12 ms, overshoot 2 ms carried), so the
        // 1000 Hz note starts with 2 ms and ends after two ticks (10 ms), and the tune loops.
        assert_eq!(got, [8, 8, 8, 8, 8, 0, 0, 0, 16, 16, 8, 8]);
    }

    #[test]
    fn a_10_ms_note_ends_at_the_third_4_ms_tick() {
        let t = Fake::new(&[(2000, 1), (1000, 1)], &[], &[]);
        let mut p = Player::new(R);
        let got = run(&mut p, &t, lv(true, false, false), 4);
        // Starts at tick 0 (0 ms), 4 and 8 ms still short of 10, ends at 12 ms.
        assert_eq!(got, [8, 8, 8, 16]);
    }

    #[test]
    fn a_higher_event_preempts_at_once_and_the_lower_restarts_from_its_first_note() {
        let t = Fake::new(&[(2000, 50)], &[], &[(500, 1), (1000, 1), (0, 100)]);
        let mut p = Player::new(R);
        // The low alarm plays its first note, then its second.
        assert_eq!(run(&mut p, &t, lv(false, false, true), 4), [32, 32, 32, 16]);
        // The horn preempts on the tick its level comes on.
        assert_eq!(run(&mut p, &t, lv(true, false, true), 2), [8, 8]);
        // When the horn ends, the low alarm starts over at its first note.
        assert_eq!(run(&mut p, &t, lv(false, false, true), 1), [32]);
    }

    #[test]
    fn critical_preempts_low_and_horn_preempts_critical() {
        let t = Fake::new(&[(2000, 50)], &[(1000, 50)], &[(500, 50)]);
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, lv(false, false, true), 1), [32]);
        assert_eq!(run(&mut p, &t, lv(false, true, true), 1), [16]);
        assert_eq!(run(&mut p, &t, lv(true, true, true), 1), [8]);
        // A lower level coming on under the horn does not interrupt its note.
        let mut q = Player::new(R);
        assert_eq!(run(&mut q, &t, lv(true, false, false), 1), [8]);
        let reads = t.reads.get();
        assert_eq!(run(&mut q, &t, lv(true, false, true), 1), [8]);
        assert_eq!(
            t.reads.get(),
            reads,
            "the horn keeps its note; nothing re-read"
        );
    }

    #[test]
    fn a_muted_event_does_not_preempt() {
        let t = Fake::new(&[], &[], &[(500, 1), (1000, 1)]);
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, lv(false, false, true), 3), [32, 32, 32]);
        // The empty horn comes on: the alarm carries on with its tune, not restarted.
        assert_eq!(run(&mut p, &t, lv(true, false, true), 2), [16, 16]);
    }

    #[test]
    fn a_tune_with_no_playable_note_is_silent_and_reads_once() {
        let t = Fake::new(&[(2000, 0), (1000, 0)], &[], &[]);
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, lv(true, false, false), 5), [0; 5]);
        // One read when the level came on; no busy loop, no read per tick.
        assert_eq!(t.reads.get(), 1);
        // dur-0 notes in a playable tune are skipped.
        let t = Fake::new(&[(2000, 0), (1000, 1), (500, 0)], &[], &[]);
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, lv(true, false, false), 4), [16, 16, 16, 16]);
    }

    #[test]
    fn a_trailing_partial_note_is_ignored() {
        let mut horn = tune(&[(1000, 1)]);
        horn.extend_from_slice(&[0xD0, 0x07]);
        let t = Fake {
            horn,
            critical: Vec::new(),
            low: Vec::new(),
            reads: Cell::new(0),
        };
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, lv(true, false, false), 6), [16; 6]);
    }

    #[test]
    fn the_level_going_off_silences_at_once() {
        let t = Fake::new(&[(2000, 50)], &[], &[]);
        let mut p = Player::new(R);
        assert_eq!(run(&mut p, &t, lv(true, false, false), 2), [8, 8]);
        assert_eq!(run(&mut p, &t, Levels::default(), 1), [0]);
    }

    #[test]
    fn the_tune_is_reread_at_each_note_start() {
        // A one-note tune that loops reads the store once per note, not once per tick.
        let t = Fake::new(&[(2000, 5)], &[], &[]);
        let mut p = Player::new(R);
        run(&mut p, &t, lv(true, false, false), 100);
        // 400 ms of 50 ms notes, the first read at the start: 1 + 7 note ends (the 8th ends at
        // 400 ms, on the tick after the 100th).
        assert_eq!(t.reads.get(), 8);
    }

    #[test]
    fn battery_levels_come_on_after_2_s_and_go_off_after_10_s() {
        let mut a = BatteryAlarm::new();
        // floor 2400, warn 3300; word 3000 is low but not critical.
        let ticks_on = ALARM_ON_MS / TICK_MS;
        for _ in 0..ticks_on - 1 {
            assert_eq!(a.step(3000, 2400, 3300, TICK_MS), (false, false));
        }
        assert_eq!(a.step(3000, 2400, 3300, TICK_MS), (false, true));
        // A sag that ends before 2 s does not start the critical level, and a clear spell of
        // less than 10 s does not end the low one.
        for _ in 0..ticks_on - 1 {
            assert_eq!(a.step(2300, 2400, 3300, TICK_MS), (false, true));
        }
        for _ in 0..ALARM_OFF_MS / TICK_MS - 1 {
            assert_eq!(a.step(3400, 2400, 3300, TICK_MS), (false, true));
        }
        assert_eq!(a.step(3400, 2400, 3300, TICK_MS), (false, false));
        // Critical on after 2 s below the floor; low comes on beside it.
        let mut on = (false, false);
        for _ in 0..ticks_on {
            on = a.step(2300, 2400, 3300, TICK_MS);
        }
        assert_eq!(on, (true, true));
    }

    #[test]
    fn unknown_and_disabled_thresholds_sound_nothing() {
        let mut a = BatteryAlarm::new();
        for _ in 0..1_000 {
            assert_eq!(a.step(0, 2400, 3300, TICK_MS), (false, false), "UNKNOWN");
        }
        for _ in 0..1_000 {
            assert_eq!(
                a.step(100, 0, 0, TICK_MS),
                (false, false),
                "no floor, no warning"
            );
            assert_eq!(
                a.step(100, -1, -5, TICK_MS),
                (false, false),
                "negative is none"
            );
        }
        // An UNKNOWN word ends a level at once.
        let mut a = BatteryAlarm::new();
        for _ in 0..ALARM_ON_MS / TICK_MS {
            a.step(3000, 2400, 3300, TICK_MS);
        }
        assert_eq!(a.step(3000, 2400, 3300, TICK_MS), (false, true));
        assert_eq!(a.step(0, 2400, 3300, TICK_MS), (false, false));
    }
}
