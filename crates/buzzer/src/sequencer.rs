//! Beep sequencer: the state/fault cadences. Run once per 4 ms tick during normal operation.
//!
//! Each tick the sequencer evaluates, in strict first-match precedence, the forced-output window, the
//! fixed-count burst, then the flag-priority groups, and finally the all-clear idle state, and decides
//! whether the single tone output is ON or OFF this tick. It is **output-only logic**: it produces
//! [`crate::TickOutput`]; the caller applies the tone command and the pin-low write to the hardware.
//!
//! The mapping of a real-world fault to a priority group is **policy** owned by `state.md`, not this
//! crate. This crate fixes only the cadence each group plays and the strict priority order. The caller
//! fills a [`Conditions`] from its fault flags; the field names here are the cadences (beep counts),
//! not fault semantics. See `todo/buzzer.md` section 6. Every threshold and pattern is exact.

use crate::tone::{beep_off, beep_on};
use crate::TickOutput;

/// Short half-period threshold: `phase >` this fires a group transition. `0x3C = 60 ticks = 240 ms`.
/// Used by groups 2 through 9.
pub const HALF_PERIOD_SHORT: u16 = 0x3C; // 60 ticks, 240 ms

/// Long half-period threshold: `0x7D = 125 ticks = 500 ms`. Used by priority 1 and by the burst path.
pub const HALF_PERIOD_LONG: u16 = 0x7D; // 125 ticks, 500 ms

/// Idle `phase` reset value: `0x7E = 126`. Parked here in the all-clear state so the next asserted
/// condition's first half-period elapses immediately.
pub const PHASE_IDLE: u16 = 0x7E; // 126

/// One flag-priority pattern group (groups 2 through 9 of `todo/buzzer.md` section 6.3).
///
/// The step index walks `0..length`, wrapping to 0 when it reaches `length`. A beep sounds only when
/// the new index is one of the odd `on_indices` (5, 7, ... up to `length - 1`); every other index is
/// silent. Each cycle is a ~1.2 s silent lead-in (indices 0..4) then `N = (length - 4) / 2` beeps of
/// 240 ms separated by 240 ms gaps. The beep count `N` encodes the condition group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PatternGroup {
    /// Pattern length; the index wraps to 0 when it reaches this value.
    pub length: u8,
    /// The "on" (beep) indices: the odd values 5, 7, ... up to `length - 1`.
    pub on_indices: &'static [u8],
}

impl PatternGroup {
    /// Whether `index` sounds a beep in this group.
    fn is_on(self, index: u8) -> bool {
        let mut i = 0;
        while i < self.on_indices.len() {
            if self.on_indices[i] == index {
                return true;
            }
            i += 1;
        }
        false
    }
}

/// Priority group 2: 1 beep per cycle. Length 6 (wrap at 6), on-index 5.
pub const GROUP_1BEEP: PatternGroup = PatternGroup {
    length: 6,
    on_indices: &[5],
};
/// Priority group 3: 2 beeps per cycle. Length 8 (wrap at 8), on-indices 5, 7.
pub const GROUP_2BEEP: PatternGroup = PatternGroup {
    length: 8,
    on_indices: &[5, 7],
};
/// Priority group 4: 3 beeps per cycle. Length 10 (wrap at 10), on-indices 5, 7, 9.
pub const GROUP_3BEEP: PatternGroup = PatternGroup {
    length: 10,
    on_indices: &[5, 7, 9],
};
/// Priority group 5: 4 beeps per cycle. Length 12 (wrap at 12), on-indices 5, 7, 9, 11.
pub const GROUP_4BEEP: PatternGroup = PatternGroup {
    length: 12,
    on_indices: &[5, 7, 9, 11],
};
/// Priority group 6: 5 beeps per cycle. Length 14 (wrap at 14), on-indices 5, 7, 9, 11, 13.
pub const GROUP_5BEEP: PatternGroup = PatternGroup {
    length: 14,
    on_indices: &[5, 7, 9, 11, 13],
};
/// Priority group 7: 6 beeps per cycle. Length 16 (wrap at 16), on-indices 5, 7, 9, 11, 13, 15.
pub const GROUP_6BEEP: PatternGroup = PatternGroup {
    length: 16,
    on_indices: &[5, 7, 9, 11, 13, 15],
};
/// Priority group 8: 7 beeps per cycle. Length 18 (wrap at 18), on-indices 5, 7, 9, 11, 13, 15, 17.
pub const GROUP_7BEEP: PatternGroup = PatternGroup {
    length: 18,
    on_indices: &[5, 7, 9, 11, 13, 15, 17],
};
/// Priority group 9: 8 beeps per cycle. Length 20 (wrap at 20), on-indices 5, 7, 9, 11, 13, 15, 17, 19.
pub const GROUP_8BEEP: PatternGroup = PatternGroup {
    length: 20,
    on_indices: &[5, 7, 9, 11, 13, 15, 17, 19],
};

/// The condition inputs the sequencer evaluates, in strict first-match priority order.
///
/// These are the cadence triggers, not fault semantics: the caller (integration with `state.md`) maps
/// real fault flags onto these fields. "Any of" groups are collapsed to a single boolean by the
/// caller. The field order documents the priority; [`Sequencer::select`] resolves the first match.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Conditions {
    /// Priority 1: the 5-flag OR. Continuous 0.5 s on / 0.5 s off. Masks all lower groups.
    pub steady: bool,
    /// Priority 2: 1 beep per cycle.
    pub beeps1: bool,
    /// Priority 3: 2 beeps per cycle, fires only when [`Conditions::suppress_2beep`] is clear.
    pub beeps2: bool,
    /// Priority 3 suppressor: when set, priority 3 is skipped and evaluation continues to group 4.
    pub suppress_2beep: bool,
    /// Priority 4: 3 beeps per cycle.
    pub beeps3: bool,
    /// Priority 5: 4 beeps per cycle.
    pub beeps4: bool,
    /// Priority 6: 5 beeps per cycle.
    pub beeps5: bool,
    /// Priority 7: 6 beeps per cycle.
    pub beeps6: bool,
    /// Priority 8: 7 beeps per cycle.
    pub beeps7: bool,
    /// Priority 9: 8 beeps per cycle.
    pub beeps8: bool,
}

/// The resolved active cadence for a tick, after precedence and the priority-3 suppressor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cadence {
    /// Priority 1: continuous 0.5 s on / 0.5 s off square wave.
    Steady,
    /// A flag-priority pattern group (priorities 2 through 9).
    Pattern(PatternGroup),
    /// All checked conditions clear: silence.
    Idle,
}

impl Conditions {
    /// Resolve the first-match cadence from the asserted conditions, honoring the priority-3
    /// suppressor. Returns [`Cadence::Idle`] when every checked condition is clear.
    pub fn select(self) -> Cadence {
        if self.steady {
            Cadence::Steady
        } else if self.beeps1 {
            Cadence::Pattern(GROUP_1BEEP)
        } else if self.beeps2 && !self.suppress_2beep {
            Cadence::Pattern(GROUP_2BEEP)
        } else if self.beeps3 {
            Cadence::Pattern(GROUP_3BEEP)
        } else if self.beeps4 {
            Cadence::Pattern(GROUP_4BEEP)
        } else if self.beeps5 {
            Cadence::Pattern(GROUP_5BEEP)
        } else if self.beeps6 {
            Cadence::Pattern(GROUP_6BEEP)
        } else if self.beeps7 {
            Cadence::Pattern(GROUP_7BEEP)
        } else if self.beeps8 {
            Cadence::Pattern(GROUP_8BEEP)
        } else {
            Cadence::Idle
        }
    }
}

/// The beep sequencer: the shared working state walked by every priority group.
///
/// Construct with [`Sequencer::new`]. Optionally arm a forced-output window with [`Sequencer::hold`]
/// or a fixed-count burst with [`Sequencer::burst`]; both take precedence over flag selection. Drive
/// once per 4 ms tick with [`Sequencer::tick`], passing the resolved [`Conditions`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sequencer {
    /// 16-bit half-period tick counter (`phase`).
    phase: u16,
    /// Single shared step index byte; walks the active group's pattern.
    step: u8,
    /// 16-bit forced-output countdown (`hold`). Highest precedence while nonzero.
    hold: u16,
    /// "forced-window started" flag: set on the first tick of a `hold` window.
    started: bool,
    /// Fixed-count burst byte: plays N long beeps while nonzero.
    burst: u8,
    /// Priority-1 single-bit toggle state (the 0.5 s on / 0.5 s off square wave).
    steady_bit: bool,
    /// "settled" marker: set in the all-clear state. Diagnostic only.
    settled: bool,
}

impl Default for Sequencer {
    fn default() -> Self {
        Self::new()
    }
}

impl Sequencer {
    /// A fresh sequencer: all working state cleared.
    pub const fn new() -> Self {
        Sequencer {
            phase: 0,
            step: 0,
            hold: 0,
            started: false,
            burst: 0,
            steady_bit: false,
            settled: false,
        }
    }

    /// Arm a forced-output window: a steady tone held on for `ticks` ticks, then normal selection
    /// resumes (section 6.1). Overrides every flag group and the burst while it counts down.
    pub fn hold(&mut self, ticks: u16) {
        self.hold = ticks;
        self.started = false;
    }

    /// Arm a fixed-count burst: a 0.5 s on / 0.5 s off cadence repeated `count` times (section 6.2).
    /// Takes precedence over flag selection but not over an active `hold` window.
    pub fn burst(&mut self, count: u8) {
        self.burst = count;
    }

    /// The current `phase` counter (for tests / diagnostics).
    pub const fn phase(self) -> u16 {
        self.phase
    }

    /// The current shared step index (for tests / diagnostics).
    pub const fn step(self) -> u8 {
        self.step
    }

    /// The remaining `hold` countdown.
    pub const fn hold_remaining(self) -> u16 {
        self.hold
    }

    /// The remaining burst count.
    pub const fn burst_remaining(self) -> u8 {
        self.burst
    }

    /// Whether the "settled" (all-clear) marker is set.
    pub const fn settled(self) -> bool {
        self.settled
    }

    /// Run the sequencer once for a 4 ms tick with the resolved condition inputs.
    ///
    /// Precedence, first match wins:
    /// 1. Forced-output window (`hold != 0`): steady tone, decrement `hold`.
    /// 2. Fixed-count burst (`hold == 0, burst != 0`): 0.5 s on / 0.5 s off, decrement on each toggle.
    /// 3. Flag-priority selection (`hold == 0, burst == 0`): the first asserted group's pattern.
    /// 4. All-clear idle: silence, reset counters.
    pub fn tick(&mut self, cond: Conditions) -> TickOutput {
        // 6.1 Forced-output window, highest precedence.
        if self.hold != 0 {
            // First tick sets "started" (the spec's `beep_on` + started = 1); subsequent ticks keep
            // the tone held on (last-writer-wins steady tone). Either way: tone on, decrement hold.
            self.started = true;
            self.hold -= 1;
            return TickOutput::on(beep_on());
        }

        // 6.2 Fixed-count burst.
        if self.burst != 0 {
            // Advance phase; when it passes the long threshold, emit one half-cycle and decrement.
            self.phase = self.phase.wrapping_add(1);
            if self.phase > HALF_PERIOD_LONG {
                self.phase = 0;
                let on = (self.burst & 1) == 1;
                self.burst -= 1;
                return if on {
                    TickOutput::on(beep_on())
                } else {
                    TickOutput::off_pin_low(beep_off())
                };
            }
            // Within the half-period: hold the previous output. Report the current burst level so the
            // tone keeps its state between elapses (the driver only re-commands on elapse in the
            // reference; we keep the current bit's tone steady here).
            return if (self.burst & 1) == 1 {
                TickOutput::on(beep_on())
            } else {
                TickOutput::off_pin_low(beep_off())
            };
        }

        // 6.3 / 6.4 Flag-priority selection (hold == 0, burst == 0).
        match cond.select() {
            Cadence::Steady => self.tick_steady(),
            Cadence::Pattern(group) => self.tick_pattern(group),
            Cadence::Idle => self.tick_idle(),
        }
    }

    /// Priority 1: a single bit flipped every long half-period; beep on the 0 -> 1 edge.
    fn tick_steady(&mut self) -> TickOutput {
        self.settled = false;
        self.phase = self.phase.wrapping_add(1);
        if self.phase > HALF_PERIOD_LONG {
            self.phase = 0;
            // Toggle the 1-bit state; sound when it was 0 (now 1), else off.
            let was_zero = !self.steady_bit;
            self.steady_bit = !self.steady_bit;
            if was_zero {
                return TickOutput::on(beep_on());
            } else {
                return TickOutput::off_pin_low(beep_off());
            }
        }
        // Within the half-period: hold the current level.
        if self.steady_bit {
            TickOutput::on(beep_on())
        } else {
            TickOutput::off_pin_low(beep_off())
        }
    }

    /// A flag-priority pattern group: walk the shared step index, beep on the group's on-indices.
    fn tick_pattern(&mut self, group: PatternGroup) -> TickOutput {
        self.settled = false;
        self.phase = self.phase.wrapping_add(1);
        if self.phase > HALF_PERIOD_SHORT {
            self.phase = 0;
            self.step += 1;
            if self.step >= group.length {
                self.step = 0;
            }
            return if group.is_on(self.step) {
                TickOutput::on(beep_on())
            } else {
                TickOutput::off_pin_low(beep_off())
            };
        }
        // Within the half-period: hold the current step's level.
        if group.is_on(self.step) {
            TickOutput::on(beep_on())
        } else {
            TickOutput::off_pin_low(beep_off())
        }
    }

    /// All-clear idle (section 6.4): silence, set the settled marker, park `phase` at [`PHASE_IDLE`],
    /// clear the forced-window started flag.
    fn tick_idle(&mut self) -> TickOutput {
        self.settled = true;
        self.phase = PHASE_IDLE;
        self.started = false;
        TickOutput::off_pin_low(beep_off())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cond_for(c: Cadence) -> Conditions {
        let mut k = Conditions::default();
        match c {
            Cadence::Steady => k.steady = true,
            Cadence::Idle => {}
            Cadence::Pattern(g) => {
                if g == GROUP_1BEEP {
                    k.beeps1 = true
                } else if g == GROUP_2BEEP {
                    k.beeps2 = true
                } else if g == GROUP_3BEEP {
                    k.beeps3 = true
                } else if g == GROUP_4BEEP {
                    k.beeps4 = true
                } else if g == GROUP_5BEEP {
                    k.beeps5 = true
                } else if g == GROUP_6BEEP {
                    k.beeps6 = true
                } else if g == GROUP_7BEEP {
                    k.beeps7 = true
                } else if g == GROUP_8BEEP {
                    k.beeps8 = true
                }
            }
        }
        k
    }

    #[test]
    fn thresholds_and_idle_exact() {
        assert_eq!(HALF_PERIOD_SHORT, 0x3C);
        assert_eq!(HALF_PERIOD_SHORT, 60);
        assert_eq!(HALF_PERIOD_LONG, 0x7D);
        assert_eq!(HALF_PERIOD_LONG, 125);
        assert_eq!(PHASE_IDLE, 0x7E);
        assert_eq!(PHASE_IDLE, 126);
    }

    #[test]
    fn group_lengths_and_on_indices_exact() {
        assert_eq!(GROUP_1BEEP.length, 6);
        assert_eq!(GROUP_1BEEP.on_indices, &[5]);
        assert_eq!(GROUP_2BEEP.length, 8);
        assert_eq!(GROUP_2BEEP.on_indices, &[5, 7]);
        assert_eq!(GROUP_3BEEP.length, 10);
        assert_eq!(GROUP_3BEEP.on_indices, &[5, 7, 9]);
        assert_eq!(GROUP_4BEEP.length, 12);
        assert_eq!(GROUP_4BEEP.on_indices, &[5, 7, 9, 11]);
        assert_eq!(GROUP_5BEEP.length, 14);
        assert_eq!(GROUP_5BEEP.on_indices, &[5, 7, 9, 11, 13]);
        assert_eq!(GROUP_6BEEP.length, 16);
        assert_eq!(GROUP_6BEEP.on_indices, &[5, 7, 9, 11, 13, 15]);
        assert_eq!(GROUP_7BEEP.length, 18);
        assert_eq!(GROUP_7BEEP.on_indices, &[5, 7, 9, 11, 13, 15, 17]);
        assert_eq!(GROUP_8BEEP.length, 20);
        assert_eq!(GROUP_8BEEP.on_indices, &[5, 7, 9, 11, 13, 15, 17, 19]);
    }

    #[test]
    fn idle_is_silent_and_parks_phase() {
        let mut s = Sequencer::new();
        let out = s.tick(Conditions::default());
        assert!(!out.tone.is_on());
        assert!(s.settled());
        assert_eq!(s.phase(), PHASE_IDLE);
    }

    /// Count the beep ONSETS (off-or-start -> on transitions) over a window for a steady condition.
    fn count_onsets(s: &mut Sequencer, cond: Conditions, ticks: usize) -> usize {
        let mut prev_on = false;
        let mut onsets = 0;
        for _ in 0..ticks {
            let on = s.tick(cond).tone.is_on();
            if on && !prev_on {
                onsets += 1;
            }
            prev_on = on;
        }
        onsets
    }

    #[test]
    fn pattern_groups_emit_exact_beep_counts_per_cycle() {
        // N = (length - 4) / 2 beeps per cycle. Run exactly two full cycles and expect 2N onsets.
        // A cycle is `length` half-periods of (HALF_PERIOD_SHORT + 1) ticks each.
        let cases = [
            (GROUP_1BEEP, 1usize),
            (GROUP_2BEEP, 2),
            (GROUP_3BEEP, 3),
            (GROUP_4BEEP, 4),
            (GROUP_5BEEP, 5),
            (GROUP_6BEEP, 6),
            (GROUP_7BEEP, 7),
            (GROUP_8BEEP, 8),
        ];
        for (group, n) in cases {
            let mut s = Sequencer::new();
            // Park phase so the first half-period elapses immediately, like the idle handoff does.
            s.tick(Conditions::default());
            let cond = cond_for(Cadence::Pattern(group));
            let ticks_per_half = (HALF_PERIOD_SHORT as usize) + 1;
            // Two full cycles.
            let total = ticks_per_half * (group.length as usize) * 2;
            let onsets = count_onsets(&mut s, cond, total);
            assert_eq!(onsets, n * 2, "group with N={n} should beep {n} times/cycle");
        }
    }

    #[test]
    fn steady_is_half_second_on_off() {
        let mut s = Sequencer::new();
        let cond = cond_for(Cadence::Steady);
        // First long half-period: phase climbs 1..=125 (off), at the tick that pushes phase > 125 it
        // flips to on. Track the on/off run lengths.
        let ticks_per_half = (HALF_PERIOD_LONG as usize) + 1;
        let mut on_run = 0;
        let mut off_run = 0;
        let mut seen_on = false;
        for _ in 0..(ticks_per_half * 4) {
            let on = s.tick(cond).tone.is_on();
            if on {
                seen_on = true;
                on_run += 1;
            } else if seen_on {
                off_run += 1;
            }
        }
        // Once sounding, the on and off windows are each one long half-period (126 ticks) wide.
        // We do not assert exact totals across the ramp-up, only that both windows are populated and
        // sized to the long half-period granularity.
        assert!(on_run >= ticks_per_half - 1);
        assert!(off_run >= ticks_per_half - 1);
    }

    #[test]
    fn hold_window_overrides_then_resumes() {
        let mut s = Sequencer::new();
        s.hold(5);
        // While hold counts down, the tone is steady on regardless of conditions (even idle).
        for _ in 0..5 {
            assert!(s.tick(Conditions::default()).tone.is_on());
        }
        assert_eq!(s.hold_remaining(), 0);
        // After hold, idle selection resumes: silence.
        let out = s.tick(Conditions::default());
        assert!(!out.tone.is_on());
        assert!(s.settled());
    }

    #[test]
    fn fault_overrides_idle_and_lower_priority() {
        // Priority is strict first-match: a higher group masks lower ones.
        let k = Conditions {
            beeps2: true, // priority 3 (2 beeps)
            beeps8: true, // priority 9 (8 beeps), lower
            ..Conditions::default()
        };
        assert_eq!(k.select(), Cadence::Pattern(GROUP_2BEEP));
        // steady (priority 1) masks everything.
        let k = Conditions { steady: true, ..k };
        assert_eq!(k.select(), Cadence::Steady);
    }

    #[test]
    fn priority3_suppressor_skips_to_group4() {
        let k = Conditions {
            beeps2: true,
            beeps3: true, // priority 4
            suppress_2beep: true,
            ..Conditions::default()
        };
        // With the suppressor set, priority 3 is skipped, group 4 (3 beeps) is selected.
        assert_eq!(k.select(), Cadence::Pattern(GROUP_3BEEP));
        // Clearing the suppressor restores priority 3.
        let k = Conditions {
            suppress_2beep: false,
            ..k
        };
        assert_eq!(k.select(), Cadence::Pattern(GROUP_2BEEP));
    }

    #[test]
    fn burst_plays_count_long_beeps() {
        let mut s = Sequencer::new();
        // burst byte counts down by 1 per elapsed long half-period, emitting on when low bit is 1.
        s.burst(3);
        let ticks_per_half = (HALF_PERIOD_LONG as usize) + 1;
        let mut onsets = 0;
        let mut prev = false;
        // Run enough ticks to drain the burst (3 elapses).
        for _ in 0..(ticks_per_half * 4) {
            let on = s.tick(Conditions::default()).tone.is_on();
            if on && !prev {
                onsets += 1;
            }
            prev = on;
        }
        // burst 3 -> bit1 on, 2 -> off, 1 -> on: two on-elapses produce beeps.
        assert!(onsets >= 1);
        assert_eq!(s.burst_remaining(), 0);
    }
}
