//! The phase bars (`specs/remote-ui.md` 3.1 and 3.6), ported from the app's
//! `model/CurrentReading.kt` and `model/RideReadings.kt`.

use heapless::String;
use linkctl::CyclicState;

use super::write_tenths;

/// `CYCLIC_STATE.mode` bytes in which the period interrupt runs, so the window words are live:
/// INIT, READY and RUN (`motorcontroller/state/src/mode.rs`, `Mode`, discriminants 1 to 3; the app's
/// `CurrentReading.kt`, `MEASURING_MODES`). OFF (0), SHUTDOWN (4) and any unnamed byte are not.
/// The mode enum lives in a GD32 crate the handheld does not link, so the bytes are stated here.
const MEASURING_MODES: core::ops::RangeInclusive<u8> = 1..=3;

/// The clamp the firmware applies to `motors.0.current.limit` at boot, mA (the app's
/// `Fields.CURRENT_LIMIT_MA`, `shared/protocol-kotlin`; `specs/remote-ui.md` 3.4). The Rust store
/// crate does not carry the range, so it is stated here.
const CURRENT_LIMIT_MA: core::ops::RangeInclusive<u32> = 1_000..=40_000;

/// What one board's phase bar says (`CurrentReading.kt`, `CurrentReading`; `specs/remote-ui.md`
/// 3.6). The app's `Amps` also carries the derived DC-link figure, which shows only on Setup.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CurrentReading {
    /// Nothing has arrived from this board, or it has left the inbox.
    Silent,
    /// The motor path is not up, so the window words are stale.
    MotorsOff,
    /// The window closed on a period that drove nothing (`duty_on` 0).
    Coasting,
    /// `motors.0.current.cal` is unread or 0, so counts cannot become amps.
    Unscaled,
    /// The PHASE current, amps: the window mean over the board's counts per amp.
    Amps(f64),
}

/// What a board's `state` (`None` while silent) and its `motors.0.current.cal`
/// (`store::MOTOR_CURRENT_CAL`, `None` while unread) come to (`CurrentReading.kt`,
/// `currentReading`; `specs/remote-ui.md` 3.6): `CyclicObs::phase_mean` over the calibration.
pub fn current_reading(state: Option<&CyclicState>, counts_per_amp: Option<u16>) -> CurrentReading {
    let Some(s) = state else {
        return CurrentReading::Silent;
    };
    if !MEASURING_MODES.contains(&s.mode) {
        return CurrentReading::MotorsOff;
    }
    if s.obs.duty_on == 0 {
        return CurrentReading::Coasting;
    }
    match counts_per_amp {
        Some(cal) if cal > 0 => CurrentReading::Amps(s.obs.phase_mean as f64 / cal as f64),
        _ => CurrentReading::Unscaled,
    }
}

/// The phase bar's figure (`ui/screens/RideInstruments.kt`, the phase row's `figure`;
/// `specs/remote-ui.md` 3.1, "What a figure shows with no value"): "6.8 A" (the magnitude, to a
/// tenth, rounded half away from zero as Java's `%.1f` does), "0.0 A" coasting, "off" with the
/// motors down, "-- A" silent or unscaled.
pub fn phase_text(reading: &CurrentReading) -> String<12> {
    let mut out = String::new();
    match reading {
        CurrentReading::Amps(a) => {
            // |amps| <= 32767 counts at 1 count per amp: at most 327670 tenths, eight characters.
            write_tenths(&mut out, libm::round(libm::fabs(*a) * 10.0) as i32);
            let _ = out.push_str(" A");
        }
        CurrentReading::Coasting => {
            let _ = out.push_str("0.0 A");
        }
        CurrentReading::MotorsOff => {
            let _ = out.push_str("off");
        }
        CurrentReading::Silent | CurrentReading::Unscaled => {
            let _ = out.push_str("-- A");
        }
    }
    out
}

/// A phase bar's fill, `0.0..=1.0` (`RideReadings.kt`, `phaseFill`; `specs/remote-ui.md` 3.1): the
/// figure's magnitude over the board's own `motors.0.current.limit` (mA, `None` while unread),
/// clamped to 1,000..40,000 mA, and clamped to full. `None` (no fill) without a figure or while the
/// limit is unread.
pub fn phase_fill(reading: &CurrentReading, limit_ma: Option<u32>) -> Option<f64> {
    let CurrentReading::Amps(amps) = reading else {
        return None;
    };
    let limit = limit_ma?.clamp(*CURRENT_LIMIT_MA.start(), *CURRENT_LIMIT_MA.end());
    Some((libm::fabs(*amps) * 1000.0 / limit as f64).clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::readings::test_support::state;

    /// `CurrentReadingTest.kt`, `cal`: 455 counts per amp.
    const CAL: Option<u16> = Some(455);
    /// `CyclicObs.DUTY_FULL` (`LinkCtl.kt`): 2250, the commutation timer's ARR.
    const DUTY_FULL: u16 = 2250;
    const OFF: u8 = 0;
    const INIT: u8 = 1;
    const READY: u8 = 2;
    const RUN: u8 = 3;
    const SHUTDOWN: u8 = 4;
    const UNNAMED: u8 = 9;

    /// `CurrentReadingTest.kt`, `telemetry`.
    fn telemetry(mean: i16, duty: u16, peak: i16, mode: u8) -> CyclicState {
        let mut s = CyclicState {
            battery: 2900,
            mode,
            ..state()
        };
        s.obs.phase_mean = mean;
        s.obs.phase_peak = peak;
        s.obs.duty_on = duty;
        s
    }

    fn amps(r: CurrentReading) -> f64 {
        match r {
            CurrentReading::Amps(a) => a,
            other => panic!("not amps: {other:?}"),
        }
    }

    /// `CurrentReadingTest.kt`, `the phase current is the window mean over the board's counts per amp`.
    #[test]
    fn the_phase_current_is_the_mean_over_cal() {
        let r = current_reading(Some(&telemetry(910, DUTY_FULL, 910, RUN)), CAL);
        assert!((amps(r) - 2.0).abs() < 0.001);
    }

    /// `CurrentReadingTest.kt`, `the peak is not what the figures are derived from`.
    #[test]
    fn the_peak_is_not_used() {
        let r = current_reading(Some(&telemetry(455, DUTY_FULL, 4550, RUN)), CAL);
        assert!((amps(r) - 1.0).abs() < 0.001);
    }

    /// `CurrentReadingTest.kt`, `a board nothing has arrived from has no reading`.
    #[test]
    fn a_silent_board_has_no_reading() {
        assert_eq!(current_reading(None, CAL), CurrentReading::Silent);
    }

    /// `CurrentReadingTest.kt`, `a board whose motors are down shows no figure, however large its
    /// last window was`.
    #[test]
    fn motors_down_show_no_figure() {
        for mode in [OFF, SHUTDOWN, UNNAMED] {
            let s = telemetry(9000, DUTY_FULL, 9000, mode);
            assert_eq!(
                current_reading(Some(&s), CAL),
                CurrentReading::MotorsOff,
                "mode {mode}"
            );
        }
    }

    /// `CurrentReadingTest.kt`, `the figures appear as soon as the motor path is up`.
    #[test]
    fn figures_appear_once_the_motor_path_is_up() {
        for mode in [INIT, READY, RUN] {
            let s = telemetry(455, DUTY_FULL, 455, mode);
            assert!(matches!(
                current_reading(Some(&s), CAL),
                CurrentReading::Amps(_)
            ));
        }
    }

    /// `CurrentReadingTest.kt`, `a coasting period shows no figure rather than its noise`.
    #[test]
    fn a_coasting_period_shows_no_figure() {
        let s = telemetry(2000, 0, 2000, RUN);
        assert_eq!(current_reading(Some(&s), CAL), CurrentReading::Coasting);
    }

    /// `CurrentReadingTest.kt`, `a board whose calibration this session has not read shows no figure`.
    #[test]
    fn an_unread_calibration_shows_no_figure() {
        let s = telemetry(455, DUTY_FULL, 455, RUN);
        assert_eq!(current_reading(Some(&s), None), CurrentReading::Unscaled);
        assert_eq!(current_reading(Some(&s), Some(0)), CurrentReading::Unscaled);
    }

    /// `RideReadingsTest.kt`, `the phase fill is the figure over the clamped limit`.
    #[test]
    fn the_phase_fill_is_over_the_clamped_limit() {
        let six = CurrentReading::Amps(6.0);
        assert!((phase_fill(&six, Some(15_000)).unwrap() - 0.4).abs() < 1e-6);
        assert!((phase_fill(&CurrentReading::Amps(0.5), Some(0)).unwrap() - 0.5).abs() < 1e-6);
        assert!(
            (phase_fill(&CurrentReading::Amps(20.0), Some(50_000)).unwrap() - 0.5).abs() < 1e-6,
            "50 A clamps to 40 A"
        );
        assert_eq!(
            phase_fill(&CurrentReading::Amps(60.0), Some(15_000)),
            Some(1.0),
            "clamped to full"
        );
        assert_eq!(
            phase_fill(&six, None),
            None,
            "no fill with the limit unread"
        );
        assert_eq!(phase_fill(&CurrentReading::Coasting, Some(15_000)), None);
    }

    /// `specs/remote-ui.md` 3.1, "What a figure shows with no value" (the phase bar rows) and
    /// `specs/handheld-remote.md` 5.2's "6.8 A". No Kotlin test covers the text (the app's is in
    /// `RideInstruments.kt`); the amps are `CurrentReadingTest.kt`'s 910 counts at 455 per amp.
    #[test]
    fn the_phase_text_and_its_no_value_forms() {
        let two = current_reading(Some(&telemetry(910, DUTY_FULL, 910, RUN)), CAL);
        assert_eq!(phase_text(&two), "2.0 A");
        assert_eq!(phase_text(&CurrentReading::Amps(6.8)), "6.8 A");
        assert_eq!(phase_text(&CurrentReading::Amps(-0.25)), "0.3 A");
        assert_eq!(phase_text(&CurrentReading::Coasting), "0.0 A");
        assert_eq!(phase_text(&CurrentReading::MotorsOff), "off");
        assert_eq!(phase_text(&CurrentReading::Silent), "-- A");
        assert_eq!(phase_text(&CurrentReading::Unscaled), "-- A");
    }
}
