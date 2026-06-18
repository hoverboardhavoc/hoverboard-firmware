//! Buzzer / tone subsystem: pure output logic for the universal hoverboard firmware.
//!
//! A single piezo/buzzer element, driven by one logical output toggled from a timer update interrupt,
//! produces audible feedback: a power-up confirmation chime, an idle/heartbeat reminder, and a family
//! of fault/warning beep cadences. This crate is **output-only logic**: per tick it decides whether
//! the one tone output should be ON or OFF, and exposes the tone timer's reload/prescaler constants.
//! It never touches a register. The actual timer-PWM/GPIO toggling is the firmware edge (the caller
//! applies the on/off plus the reload constants to `runtime-hal`'s timer trait, and drives the buzzer
//! pin low on the off paths).
//!
//! Three logically independent producers share the single tone output, with last-writer-wins:
//!
//! - [`tone`]: the tone driver, the lowest layer. Frequency request -> [`tone::ToneCmd`].
//! - [`chime::Chime`]: the one-shot startup chime, run once per 4 ms tick, gated by a "chime armed"
//!   flag.
//! - [`sequencer::Sequencer`]: the periodic beep sequencer that realizes the state/fault cadences.
//!
//! Only one producer is active at a time in practice (the chime during init, the sequencer during
//! ready/run); they are not arbitrated inside this crate.
//!
//! The split the universal-firmware architecture requires (see `spec/core.md`):
//!
//! - **FIXED** here: the tone-driver constants ([`tone::TONE_HZ`] = 5000, [`tone::PSC`] = 0x23,
//!   [`tone::CAR`] = 0x190, [`tone::FREQ_REF`] = 2_000_000, compare value 0), the
//!   GPIO-toggle-in-update-ISR model, every cadence threshold and pattern, and the 250 Hz / 4 ms
//!   timebase.
//! - **CONFIG / board data** (not here): the concrete buzzer pin and tone timer instance, resolved
//!   through the `McuDescriptor` by `runtime-hal`.
//! - **POLICY** (owned by `state.md`, not here): which real fault maps to which priority group. This
//!   crate fixes the cadence each group plays and the priority order; the caller fills
//!   [`sequencer::Conditions`] from its fault flags.
//!
//! Uses the reference constants from the reference stock firmware; the constants and
//! semantics are preserved exactly. See `spec/core.md` and `todo/buzzer.md`.
//!
//! `no_std`; host tests in `#[cfg(test)]` modules link `std` via the host target.

#![no_std]

pub mod chime;
pub mod sequencer;
pub mod tone;

pub use chime::Chime;
pub use sequencer::{Cadence, Conditions, PatternGroup, Sequencer};
pub use tone::{beep_off, beep_on, car_for, tone, ToneCmd, CAR, COMPARE_VALUE, FREQ_REF, PSC, TONE_HZ};

/// The scheduler tick rate this subsystem runs at, Hz (250 Hz). Both sequenced producers (startup
/// chime, beep sequencer) are driven once per tick.
pub const TICK_HZ: u32 = 250;

/// Tick period in milliseconds (4 ms at 250 Hz). All cadence durations are expressed in these ticks.
pub const TICK_MS: u32 = 1000 / TICK_HZ;

/// What the firmware edge should do with the buzzer **pin** this tick, independent of the tone-timer
/// command.
///
/// On the "off" paths the producers force the pin low (the redundant pin-low write that guarantees the
/// pin idles low whichever half-cycle the update ISR stopped on). On the "on" paths the pin is left to
/// the update ISR's software toggle, so the crate makes no pin demand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinAction {
    /// Leave the pin to the update ISR's software toggle (tone sounding).
    LetIsrToggle,
    /// Drive the buzzer pin low now (tone off / idle).
    DriveLow,
}

/// The per-tick decision a producer hands back to the firmware edge: the tone-timer command plus the
/// buzzer-pin action.
///
/// The caller applies `tone` to `runtime-hal`'s timer trait (programming PSC/CAR and the update
/// interrupt / counter-enable per [`ToneCmd`]) and applies `pin` to the buzzer's `embedded-hal`
/// digital output. The crate decides ON vs OFF; the hardware effect is the caller's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickOutput {
    /// The tone-timer command (on with reload values, or off).
    pub tone: ToneCmd,
    /// What to do with the buzzer pin this tick.
    pub pin: PinAction,
}

impl TickOutput {
    /// Tone sounding: program/run the timer, leave the pin to the ISR toggle.
    pub(crate) const fn on(cmd: ToneCmd) -> Self {
        TickOutput {
            tone: cmd,
            pin: PinAction::LetIsrToggle,
        }
    }

    /// Tone off with the explicit pin-low write (the `beep_off` wrapper effect).
    pub(crate) const fn off_pin_low(cmd: ToneCmd) -> Self {
        TickOutput {
            tone: cmd,
            pin: PinAction::DriveLow,
        }
    }

    /// Whether the tone is sounding this tick.
    pub const fn is_on(self) -> bool {
        self.tone.is_on()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timebase_exact() {
        assert_eq!(TICK_HZ, 250);
        assert_eq!(TICK_MS, 4);
    }

    #[test]
    fn tone_params_exposed_exactly() {
        assert_eq!(TONE_HZ, 5000);
        assert_eq!(PSC, 0x23);
        assert_eq!(CAR, 0x190);
        assert_eq!(FREQ_REF, 2_000_000);
        assert_eq!(FREQ_REF, 0x001E8480);
        assert_eq!(COMPARE_VALUE, 0);
        assert_eq!(car_for(TONE_HZ), CAR);
    }

    #[test]
    fn on_output_lets_isr_toggle_off_drives_low() {
        let on = TickOutput::on(beep_on());
        assert!(on.is_on());
        assert_eq!(on.pin, PinAction::LetIsrToggle);
        let off = TickOutput::off_pin_low(beep_off());
        assert!(!off.is_on());
        assert_eq!(off.pin, PinAction::DriveLow);
    }
}
