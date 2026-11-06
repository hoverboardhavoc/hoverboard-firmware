//! The battery gauge (`specs/battery-gauge.md`): the board's LEDs show the tones battery alarm's
//! levels. Pure logic, run once per served 250 Hz tick from the control task, host-tested here.
//!
//! The gauge has no thresholds of its own: its states are the alarm's two filtered levels
//! (`tones::BatteryAlarm`, `specs/tones.md` section 4.4) and whether the battery word is known.
//! [`Indicators`] holds the alarm and its two readers, the tone player and the gauge, and steps
//! the alarm once per tick so both readers see the same levels.

// The caller is the target-only control task in `main.rs`; the host build compiles this module for
// its tests alone.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use crate::tones::{BatteryAlarm, Levels, Player, Tunes};

/// The LED bits, in the order of `layout::Layout::leds`.
pub const GREEN: u8 = 1 << 0;
pub const ORANGE: u8 = 1 << 1;
pub const RED: u8 = 1 << 2;

/// The Low state's slow blink, ms (section 2).
pub const SLOW_PERIOD_MS: u32 = 1_000;
pub const SLOW_ON_MS: u32 = 500;
/// The Critical state's fast blink, ms.
pub const FAST_PERIOD_MS: u32 = 250;
pub const FAST_ON_MS: u32 = 125;

/// The gauge's four states (section 2), each the index of its [`PATTERNS`] row.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum State {
    /// The battery word is UNKNOWN: every LED off.
    Dark,
    /// Known, neither level on: green, steady.
    Ok,
    /// The low level on, critical off: orange, slow blink.
    Low,
    /// The critical level on: red, fast blink.
    Critical,
}

/// One state's pattern: its colour's LED bit, the blink period and the on half, ms. A steady state
/// has a 1 ms period, which keeps it at 0 ms into the blink, and is on for its whole period; Dark
/// is never on.
struct Pattern {
    colour: u8,
    period_ms: u16,
    on_ms: u16,
}

/// The patterns by [`State`].
const PATTERNS: [Pattern; 4] = [
    Pattern {
        colour: 0,
        period_ms: 1,
        on_ms: 0,
    },
    Pattern {
        colour: GREEN,
        period_ms: 1,
        on_ms: 1,
    },
    Pattern {
        colour: ORANGE,
        period_ms: SLOW_PERIOD_MS as u16,
        on_ms: SLOW_ON_MS as u16,
    },
    Pattern {
        colour: RED,
        period_ms: FAST_PERIOD_MS as u16,
        on_ms: FAST_ON_MS as u16,
    },
];

/// The gauge: the state, the milliseconds into its blink, and the LEDs it last lit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gauge {
    state: State,
    into_ms: u16,
    lit: u8,
}

impl Gauge {
    /// A dark gauge, every LED off: the level the boot drives the pins to.
    const fn new() -> Self {
        Gauge {
            state: State::Dark,
            into_ms: 0,
            lit: 0,
        }
    }

    /// One step: the alarm's two levels, whether the battery word is known, the LEDs stored (the
    /// [`GREEN`], [`ORANGE`] and [`RED`] bits) and `dt_ms` elapsed since the last step. Returns the
    /// LEDs lit now and the LEDs whose level changed since the last step, as the same bits.
    ///
    /// The state lights its own colour's LED when that LED is stored and otherwise the first
    /// stored LED in the order green, orange, red (section 3). A blink starts in its on half when
    /// the state changes, and a half ends at the first step whose time into the blink reaches it.
    pub fn step(
        &mut self,
        critical: bool,
        low: bool,
        known: bool,
        stored: u8,
        dt_ms: u32,
    ) -> (u8, u8) {
        let state = if !known {
            State::Dark
        } else if critical {
            State::Critical
        } else if low {
            State::Low
        } else {
            State::Ok
        };
        let p = &PATTERNS[state as usize];
        if state != self.state {
            self.state = state;
            self.into_ms = 0;
        } else {
            let into = u32::from(self.into_ms).saturating_add(dt_ms);
            self.into_ms = (into % u32::from(p.period_ms)) as u16;
        }
        let led = if stored & p.colour != 0 {
            p.colour
        } else {
            stored.isolate_lowest_one()
        };
        let lit = if self.into_ms < p.on_ms { led } else { 0 };
        let changed = lit ^ self.lit;
        self.lit = lit;
        (lit, changed)
    }
}

/// What one tick reads: the block's effective battery word (centivolts, 0 = UNKNOWN), the floor in
/// force and the boot-read warning threshold (`<= 0` = none), the horn from the `INPUTS` mirror,
/// and the LEDs stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickIn {
    pub word: i16,
    pub floor: i16,
    pub warn: i16,
    pub horn: bool,
    pub stored: u8,
}

/// What one tick drives: the tone period for `motor::TONE_PERIOD`, and the LEDs lit and changed
/// ([`Gauge::step`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickOut {
    pub period: u32,
    pub lit: u8,
    pub changed: u8,
}

/// The battery alarm and its two readers (section 4, "Where it runs"): the alarm is stepped once
/// per tick, and the player and the gauge read that step's levels.
pub struct Indicators {
    alarm: BatteryAlarm,
    player: Player,
    gauge: Gauge,
}

impl Indicators {
    /// Silent and dark, for a period ISR running at `rate_hz`.
    pub const fn new(rate_hz: u32) -> Self {
        Indicators {
            alarm: BatteryAlarm::new(),
            player: Player::new(rate_hz),
            gauge: Gauge::new(),
        }
    }

    /// One served tick, `dt_ms` elapsed since the last.
    pub fn tick(&mut self, input: &TickIn, dt_ms: u32, tunes: &impl Tunes) -> TickOut {
        let (battery_critical, battery_low) =
            self.alarm.step(input.word, input.floor, input.warn, dt_ms);
        let levels = Levels {
            horn: input.horn,
            battery_critical,
            battery_low,
        };
        let period = self.player.tick(levels, dt_ms, tunes);
        let (lit, changed) = self.gauge.step(
            battery_critical,
            battery_low,
            input.word != 0,
            input.stored,
            dt_ms,
        );
        TickOut {
            period,
            lit,
            changed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tones::{Event, ALARM_OFF_MS, ALARM_ON_MS, TICK_MS};
    use std::vec::Vec;
    use store::tune::Note;

    const ALL: u8 = GREEN | ORANGE | RED;

    /// The LEDs lit at each of `ticks` 4 ms steps in one state.
    fn run(g: &mut Gauge, critical: bool, low: bool, stored: u8, ticks: usize) -> Vec<u8> {
        (0..ticks)
            .map(|_| g.step(critical, low, true, stored, TICK_MS).0)
            .collect()
    }

    /// The first `n` levels of a blink lighting `led`, whose on half is `on` ticks of `period`.
    fn blink(led: u8, on: usize, period: usize, n: usize) -> Vec<u8> {
        (0..n)
            .map(|k| if k % period < on { led } else { 0 })
            .collect()
    }

    #[test]
    fn three_leds_each_state_its_own_colour() {
        let mut g = Gauge::new();
        assert_eq!(run(&mut g, false, false, ALL, 300), [GREEN; 300]);
        // Low: 1,000 ms period, 500 ms on, at 4 ms ticks 125 on and 125 off.
        let mut g = Gauge::new();
        assert_eq!(
            run(&mut g, false, true, ALL, 750),
            blink(ORANGE, 125, 250, 750)
        );
        let mut g = Gauge::new();
        // Critical with low on beside it: red, fast.
        assert_eq!(run(&mut g, true, true, ALL, 300)[..2], [RED, RED]);
    }

    #[test]
    fn the_fast_half_ends_at_the_first_tick_that_reaches_125_ms() {
        // 125 ms is not a multiple of 4 ms: on at 0, 4, .. 124 ms (32 ticks), off from 128 ms. The
        // period ends at 252 ms, the first tick at or past 250 ms, and the overshoot is carried, so
        // the next on half starts 2 ms in and the blink keeps its 250 ms period on average.
        let mut g = Gauge::new();
        let got = run(&mut g, true, false, ALL, 200);
        assert_eq!(got[..32], [RED; 32]);
        assert_eq!(got[32..63], [0; 31], "off from 128 ms to 248 ms");
        assert_eq!(got[63], RED, "252 ms wraps to 2 ms, the on half");
        // Every half after is 31 or 32 ticks, as the 2 ms remainder moves the edges.
        let mut runs = Vec::new();
        let mut k = 0;
        while k < got.len() {
            let l = got[k];
            let start = k;
            while k < got.len() && got[k] == l {
                k += 1;
            }
            if k < got.len() {
                runs.push(k - start);
            }
        }
        assert!(runs.iter().all(|&n| n == 31 || n == 32), "{runs:?}");
    }

    #[test]
    fn a_single_led_shows_every_state_by_its_pattern() {
        for led in [GREEN, ORANGE, RED] {
            let mut g = Gauge::new();
            assert_eq!(run(&mut g, false, false, led, 300), vec![led; 300], "Ok");
            let mut g = Gauge::new();
            assert_eq!(
                run(&mut g, false, true, led, 500),
                blink(led, 125, 250, 500),
                "Low"
            );
            let mut g = Gauge::new();
            assert_eq!(
                run(&mut g, true, false, led, 32),
                vec![led; 32],
                "Critical on"
            );
            assert_eq!(
                run(&mut g, true, false, led, 31),
                vec![0; 31],
                "Critical off"
            );
        }
    }

    #[test]
    fn green_with_red_lights_green_for_low() {
        let stored = GREEN | RED;
        let mut g = Gauge::new();
        assert_eq!(run(&mut g, false, false, stored, 1), [GREEN]);
        assert_eq!(
            run(&mut g, false, true, stored, 250),
            blink(GREEN, 125, 250, 250)
        );
        assert_eq!(run(&mut g, true, true, stored, 32), [RED; 32]);
        // Orange with red: Ok takes the first stored, orange.
        let mut g = Gauge::new();
        assert_eq!(run(&mut g, false, false, ORANGE | RED, 1), [ORANGE]);
    }

    #[test]
    fn no_led_stored_lights_nothing() {
        let mut g = Gauge::new();
        for (c, l) in [(false, false), (false, true), (true, true)] {
            let got: Vec<(u8, u8)> = (0..300).map(|_| g.step(c, l, true, 0, TICK_MS)).collect();
            assert!(got.iter().all(|&o| o == (0, 0)));
        }
    }

    #[test]
    fn a_state_change_starts_the_blink_in_its_on_half() {
        let mut g = Gauge::new();
        // Into the off half of the slow blink, then critical comes on: red at once.
        let low = run(&mut g, false, true, ALL, 130);
        assert_eq!(low[129], 0);
        assert_eq!(g.step(true, true, true, ALL, TICK_MS), (RED, RED));
        // Into the fast blink's off half, then back to Low: orange at once.
        run(&mut g, true, true, ALL, 40);
        assert_eq!(g.step(false, true, true, ALL, TICK_MS), (ORANGE, ORANGE));
        // Back to Ok: orange off and green on in the same step.
        assert_eq!(
            g.step(false, false, true, ALL, TICK_MS),
            (GREEN, ORANGE | GREEN)
        );
    }

    #[test]
    fn only_a_level_that_changes_is_reported() {
        let mut g = Gauge::new();
        assert_eq!(g.step(false, false, true, ALL, TICK_MS), (GREEN, GREEN));
        for _ in 0..100 {
            assert_eq!(g.step(false, false, true, ALL, TICK_MS), (GREEN, 0));
        }
        // The slow blink reports each edge once: on at the change, off 500 ms later.
        let changes: Vec<usize> = (0..500)
            .filter(|_| g.step(false, true, true, ALL, TICK_MS).1 != 0)
            .collect();
        assert_eq!(changes.len(), 4, "on, off, on, off over 2 s");
    }

    #[test]
    fn an_unknown_word_is_dark_with_a_level_still_on() {
        let mut g = Gauge::new();
        assert_eq!(run(&mut g, true, true, ALL, 1), [RED]);
        assert_eq!(g.step(true, true, false, ALL, TICK_MS), (0, RED));
        for _ in 0..500 {
            assert_eq!(g.step(true, true, false, ALL, TICK_MS), (0, 0));
            assert_eq!(g.step(false, false, false, ALL, TICK_MS), (0, 0));
        }
    }

    /// The three tunes, the battery alarm's one note each.
    struct Tunes3;

    impl Tunes for Tunes3 {
        fn with<R>(&self, e: Event, f: impl FnOnce(&[u8]) -> R) -> R {
            let note = |hz| Note { hz, dur: 10 }.encode();
            match e {
                Event::Horn => f(&[]),
                Event::BatteryCritical => f(&note(2000)),
                Event::BatteryLow => f(&note(1000)),
            }
        }
    }

    #[test]
    fn the_player_and_the_gauge_see_the_same_levels_on_the_same_tick() {
        const R: u32 = 16_000;
        let mut ind = Indicators::new(R);
        // floor 2400, warn 3300.
        let input = |word| TickIn {
            word,
            floor: 2400,
            warn: 3300,
            horn: false,
            stored: ALL,
        };
        let first = ind.tick(&input(3400), TICK_MS, &Tunes3);
        assert_eq!((first.period, first.lit), (0, GREEN));
        // 3000 is low: both readers change on the tick the filter turns the level on, 2 s in.
        let ticks_on = (ALARM_ON_MS / TICK_MS) as usize;
        let low: Vec<TickOut> = (0..ticks_on)
            .map(|_| ind.tick(&input(3000), TICK_MS, &Tunes3))
            .collect();
        let tone = low.iter().position(|o| o.period != 0);
        let lamp = low.iter().position(|o| o.lit == ORANGE);
        assert_eq!(tone, Some(ticks_on - 1));
        assert_eq!(tone, lamp);
        assert_eq!(low[ticks_on - 1].period, 16, "the low tune, 1000 Hz");
        // Below the floor: critical, on the same tick for both.
        let crit: Vec<TickOut> = (0..ticks_on)
            .map(|_| ind.tick(&input(2300), TICK_MS, &Tunes3))
            .collect();
        let tone = crit.iter().position(|o| o.period == 8);
        let lamp = crit.iter().position(|o| o.lit == RED);
        assert_eq!(tone, Some(ticks_on - 1));
        assert_eq!(tone, lamp);
        // Restored: both levels off together after 10 s clear, the gauge green and the player
        // silent on the same tick.
        let ticks_off = (ALARM_OFF_MS / TICK_MS) as usize;
        let back: Vec<TickOut> = (0..ticks_off)
            .map(|_| ind.tick(&input(3400), TICK_MS, &Tunes3))
            .collect();
        let tone = back.iter().position(|o| o.period == 0);
        let lamp = back
            .iter()
            .position(|o| o.lit == GREEN && o.changed & GREEN != 0);
        assert_eq!(tone, Some(ticks_off - 1));
        assert_eq!(tone, lamp);
        // An UNKNOWN word: dark and silent at once.
        let unknown = ind.tick(&input(0), TICK_MS, &Tunes3);
        assert_eq!((unknown.period, unknown.lit), (0, 0));
    }
}
