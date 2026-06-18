//! Startup chime: a one-shot power-up confirmation, run once per 4 ms tick, gated by the external
//! "chime armed" flag.
//!
//! Net audible effect: a single continuous ~304 ms beep (the standard tone) at power-up, exactly once
//! per arm. Re-arming requires the armed flag to go clear then set again. See `todo/buzzer.md`
//! section 5. All thresholds are exact.

use crate::tone::{beep_off, beep_on};
use crate::TickOutput;

/// Chime tick-counter threshold: at/above this the chime is finished (`0x4C = 76 ticks = 304 ms`).
pub const CHIME_THRESHOLD: u16 = 0x4C; // 76 ticks, 304 ms

/// Value the tick counter is clamped to once the chime finishes (`0x4B = 75`).
pub const CHIME_CLAMP: u16 = 0x4B; // 75

/// Saturation ceiling for the tick counter while the chime runs (never wraps past `0xFFFE`).
pub const CHIME_TICK_MAX: u16 = 0xFFFE;

/// The startup-chime state machine (one shared field set, see `todo/buzzer.md` section 5).
///
/// Construct cleared with [`Chime::new`]. Drive once per 4 ms tick with [`Chime::tick`], passing the
/// external "chime armed" flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Chime {
    /// Tick counter (unsigned 16-bit, saturating). Reset value 0.
    counter: u16,
    /// Tone-on sub-flag: true while the chime tone is sounding. Reset value false.
    tone_on: bool,
}

impl Default for Chime {
    fn default() -> Self {
        Self::new()
    }
}

impl Chime {
    /// A fresh, idle chime: counter 0, tone-on cleared.
    pub const fn new() -> Self {
        Chime {
            counter: 0,
            tone_on: false,
        }
    }

    /// The current tick counter (for tests / diagnostics).
    pub const fn counter(self) -> u16 {
        self.counter
    }

    /// Whether the chime tone is currently sounding.
    pub const fn is_sounding(self) -> bool {
        self.tone_on
    }

    /// Run the chime once for a 4 ms tick. `armed` is the external "chime armed" flag.
    ///
    /// - `armed == false` (idle / reset state): tone off, pin held low, counter = 0, tone-on cleared.
    /// - `armed == true`: if the counter is below [`CHIME_TICK_MAX`], increment it (saturating). Then
    ///   while the counter is `< CHIME_THRESHOLD` start/hold the tone; at/above it, clamp the counter
    ///   to [`CHIME_CLAMP`], turn the tone off, and finish. The chime will not sound again until the
    ///   armed flag goes clear then set again.
    pub fn tick(&mut self, armed: bool) -> TickOutput {
        if !armed {
            // Idle / reset state: tone off, pin idle low, counters cleared.
            self.counter = 0;
            self.tone_on = false;
            return TickOutput::off_pin_low(beep_off());
        }

        // Armed: advance the saturating tick counter.
        if self.counter < CHIME_TICK_MAX {
            self.counter += 1;
        }

        if self.counter < CHIME_THRESHOLD {
            // Within the ~304 ms window: turn the tone on once (the spec's "if tone-on is 0, set the
            // tone") and hold it. Either way the tone stays sounding (last-writer-wins).
            self.tone_on = true;
            TickOutput::on(beep_on())
        } else {
            // Finished: clamp the counter, turn the tone off, drive the pin low.
            self.counter = CHIME_CLAMP;
            self.tone_on = false;
            TickOutput::off_pin_low(beep_off())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PinAction;

    #[test]
    fn thresholds_exact() {
        assert_eq!(CHIME_THRESHOLD, 0x4C);
        assert_eq!(CHIME_THRESHOLD, 76);
        assert_eq!(CHIME_CLAMP, 0x4B);
        assert_eq!(CHIME_CLAMP, 75);
        assert_eq!(CHIME_TICK_MAX, 0xFFFE);
    }

    #[test]
    fn idle_when_unarmed_is_silent() {
        let mut c = Chime::new();
        for _ in 0..10 {
            let out = c.tick(false);
            assert!(!out.tone.is_on());
            assert_eq!(out.pin, PinAction::DriveLow);
            assert_eq!(c.counter(), 0);
        }
    }

    #[test]
    fn single_continuous_beep_for_76_ticks() {
        let mut c = Chime::new();
        // Ticks 1..=75 (counter 1..=75): tone on the whole time.
        for t in 1..=75u16 {
            let out = c.tick(true);
            assert!(out.tone.is_on(), "tick {t} should sound");
            assert_eq!(c.counter(), t);
        }
        // Tick 76: counter reaches CHIME_THRESHOLD (76), tone turns off, counter clamps to 75.
        let out = c.tick(true);
        assert!(!out.tone.is_on(), "tone must turn off at threshold");
        assert_eq!(out.pin, PinAction::DriveLow);
        assert_eq!(c.counter(), CHIME_CLAMP);
    }

    #[test]
    fn plays_once_then_stays_silent_while_held_armed() {
        let mut c = Chime::new();
        // Run well past the chime length.
        let mut on_ticks = 0;
        for _ in 0..200 {
            if c.tick(true).tone.is_on() {
                on_ticks += 1;
            }
        }
        // Exactly the 75 in-window ticks sounded; never re-sounds while held armed.
        assert_eq!(on_ticks, 75);
        // Counter stays clamped.
        assert_eq!(c.counter(), CHIME_CLAMP);
    }

    #[test]
    fn re_arm_requires_clear_then_set() {
        let mut c = Chime::new();
        for _ in 0..200 {
            c.tick(true);
        }
        // Disarm: resets the counter.
        let out = c.tick(false);
        assert!(!out.tone.is_on());
        assert_eq!(c.counter(), 0);
        // Re-arm: the chime plays again.
        let mut on_ticks = 0;
        for _ in 0..200 {
            if c.tick(true).tone.is_on() {
                on_ticks += 1;
            }
        }
        assert_eq!(on_ticks, 75);
    }
}
