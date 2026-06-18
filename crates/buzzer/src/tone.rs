//! Tone driver: the lowest layer. Computes, per "on"/"off" call, the timer reload values and the
//! update-interrupt / counter-enable state the firmware edge must apply.
//!
//! This crate is **output-only logic**: it never touches a register. The reference register offsets
//! and the GPIO-toggle-in-update-ISR model are recorded in `todo/buzzer.md` as provenance; here we
//! reduce them to the portable facts the caller applies to `runtime-hal`'s timer trait: the prescaler
//! [`PSC`], the auto-reload [`car_for`], whether the update interrupt is enabled, and whether the
//! counter runs. The audible 50% square wave is produced in software by the tone timer's update ISR
//! (toggling the buzzer pin); that ISR is the firmware edge, not this crate.
//!
//! Every numeric constant here is preserved exactly from the spec. See `todo/buzzer.md` sections 4.

/// The single tone request value used everywhere in this subsystem, in Hz (the default tone).
///
/// Every "beep on" event, the startup chime, and every sequencer pattern use this same request. It
/// selects [`CAR`] with [`PSC`], giving a pin frequency of `f_TIM / (2 * 36 * 401)`. The driver is
/// otherwise programmable (see [`car_for`]) but no other value is used.
pub const TONE_HZ: u32 = 5000;

/// Prescaler value (`PSC`), written on every "on" call. The counter ticks once per `PSC + 1 = 36`
/// timer-input-clock cycles. Fixed regardless of the requested frequency or `f_TIM`.
pub const PSC: u16 = 0x23; // 35

/// Auto-reload (`CAR`) for the [`TONE_HZ`] request: `(FREQ_REF / 5000) & 0xFFFF = 400`.
///
/// Equal to `car_for(TONE_HZ)`. Exposed as a named constant because it is the load-bearing value the
/// firmware edge writes for the standard tone.
pub const CAR: u16 = 0x190; // 400

/// Frequency reference constant divided by the requested frequency to compute the auto-reload.
///
/// This is a firmware constant, **not** by itself the timer's input clock. With the fixed [`PSC`] one
/// timer update event occurs every `(PSC + 1) * (CAR + 1) = 36 * (CAR + 1)` timer-input-clock cycles.
pub const FREQ_REF: u32 = 2_000_000; // 0x001E8480

/// The compare-class value the driver programs: **0**, never `period / 2`.
///
/// The reference timer is not a PWM generator; no channel-control or channel-compare register is ever
/// written. This constant records that a reimplementation must not write a 50%-duty compare anywhere.
pub const COMPARE_VALUE: u16 = 0;

/// Compute the auto-reload (`CAR`) for a frequency request: `(FREQ_REF / frequency) & 0xFFFF`.
///
/// Integer division of the reference constant by the request, low 16 bits kept. For [`TONE_HZ`] this
/// is [`CAR`] (400 / 0x190). A request of 0 means "off" and has no reload (see [`ToneCmd::Off`]); this
/// helper is only meaningful for `frequency > 0`.
pub const fn car_for(frequency: u32) -> u16 {
    (FREQ_REF / frequency) as u16
}

/// The driver's decision for one "on"/"off" call: what the firmware edge applies to the tone timer.
///
/// This captures only the portable, MCU-agnostic facts. The reference register write order (RCU
/// enable, CTL0 mode bits, CAR, PSC, SWEVG latch, INTF clear, DMAINTEN, CNT, CTL0 run) is provenance;
/// `runtime-hal`'s timer trait realizes the equivalent effect from these fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToneCmd {
    /// Tone on (`frequency > 0`): program the timer and run it.
    On {
        /// Prescaler to write ([`PSC`], every call).
        psc: u16,
        /// Auto-reload to write (`car_for(frequency)`).
        car: u16,
        /// The compare-class value, always [`COMPARE_VALUE`] (0). No PWM compare is written.
        compare: u16,
        /// Update interrupt enabled (true): the update ISR toggles the buzzer pin.
        update_irq: bool,
        /// Counter running (true): the timer counts and generates update events.
        run: bool,
    },
    /// Tone off (`frequency == 0`): disable the update interrupt, zero and stop the counter. No
    /// PSC/CAR/mode write occurs in this path, and the driver itself does not touch the buzzer pin
    /// (the `beep_off` wrapper adds the explicit pin-low write).
    Off {
        /// Update interrupt enabled: always false (disabled).
        update_irq: bool,
        /// Counter running: always false (stopped).
        run: bool,
    },
}

impl ToneCmd {
    /// Whether this command sounds the tone (the counter runs and the update ISR toggles the pin).
    pub const fn is_on(self) -> bool {
        matches!(self, ToneCmd::On { .. })
    }
}

/// The tone driver: convert a frequency request into the [`ToneCmd`] the firmware edge applies.
///
/// `frequency > 0` -> [`ToneCmd::On`] (program PSC/CAR, enable update IRQ, run). `frequency == 0` ->
/// [`ToneCmd::Off`] (disable update IRQ, zero and stop the counter). This mirrors the per-call effect
/// in `todo/buzzer.md` sections 4.1 / 4.2 reduced to portable fields.
pub const fn tone(frequency: u32) -> ToneCmd {
    if frequency > 0 {
        ToneCmd::On {
            psc: PSC,
            car: car_for(frequency),
            compare: COMPARE_VALUE,
            update_irq: true,
            run: true,
        }
    } else {
        ToneCmd::Off {
            update_irq: false,
            run: false,
        }
    }
}

/// `beep_on`: request the standard tone ([`TONE_HZ`]). Nothing else.
pub const fn beep_on() -> ToneCmd {
    tone(TONE_HZ)
}

/// `beep_off`: request tone off (`tone(0)`). The caller additionally drives the buzzer pin low (the
/// redundant pin-low write that guarantees the pin idles low whichever half-cycle the ISR stopped on);
/// see [`PinAction`] on the producer outputs.
pub const fn beep_off() -> ToneCmd {
    tone(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_exact() {
        assert_eq!(TONE_HZ, 5000);
        assert_eq!(PSC, 0x23);
        assert_eq!(PSC, 35);
        assert_eq!(CAR, 0x190);
        assert_eq!(CAR, 400);
        assert_eq!(FREQ_REF, 2_000_000);
        assert_eq!(FREQ_REF, 0x001E8480);
        assert_eq!(COMPARE_VALUE, 0);
    }

    #[test]
    fn car_for_5000_is_400() {
        assert_eq!(car_for(TONE_HZ), 0x190);
        assert_eq!(car_for(5000), 400);
        assert_eq!(CAR, car_for(TONE_HZ));
    }

    #[test]
    fn on_call_programs_psc_car_and_runs() {
        let cmd = beep_on();
        assert_eq!(
            cmd,
            ToneCmd::On {
                psc: 0x23,
                car: 0x190,
                compare: 0,
                update_irq: true,
                run: true,
            }
        );
        assert!(cmd.is_on());
    }

    #[test]
    fn off_call_disables_irq_and_stops() {
        let cmd = beep_off();
        assert_eq!(
            cmd,
            ToneCmd::Off {
                update_irq: false,
                run: false,
            }
        );
        assert!(!cmd.is_on());
    }

    #[test]
    fn no_pwm_compare_is_ever_period_over_two() {
        // The compare-class value is 0, never period/2 (CAR/2 = 200).
        if let ToneCmd::On { compare, car, .. } = beep_on() {
            assert_eq!(compare, 0);
            assert_ne!(compare, car / 2);
        } else {
            panic!("beep_on must be ToneCmd::On");
        }
    }
}
