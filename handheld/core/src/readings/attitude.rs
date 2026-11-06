//! The attitude corners' text (`specs/handheld-remote.md` 5.2, `specs/remote-ui.md` 3.1), ported
//! from the app's `model/RideReadings.kt` (`angleText`) and `ui/screens/RideInstruments.kt`
//! (`AttitudeCorner`).

use heapless::String;
use linkctl::CyclicState;

use super::write_tenths;

/// An attitude figure in `centidegrees` to 0.1 degree, rounded half away from zero, as "2.3"; a
/// value that rounds to zero prints "0.0", never "-0.0" (`RideReadings.kt`, `angleText`;
/// `specs/remote-ui.md` 3.1, the attitude corners).
pub fn angle_text(centidegrees: i16) -> String<8> {
    let c = centidegrees as i32;
    let tenths = (c.abs() + 5) / 10;
    let mut out = String::new();
    // An i16 in centidegrees is at most 327.7 degrees: "-327.7" fits eight bytes.
    write_tenths(&mut out, if c < 0 { -tenths } else { tenths });
    out
}

/// What one attitude corner shows (`RideInstruments.kt`, `AttitudeCorner`; `specs/remote-ui.md`
/// 3.1, "What a figure shows with no value"): the board's own pitch and roll, the ring alone with
/// dashes while it is silent, or the ring alone and "no IMU" while its `imu.model` reads 0. No
/// IMU wins over silent, as the app orders the two.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Attitude {
    /// `imu.model` reads 0: no IMU fitted.
    NoImu,
    /// The board is silent.
    Silent,
    /// The board's `CYCLIC_STATE.pitch` and `roll`, centidegrees.
    Angles {
        /// Pitch, centidegrees.
        pitch: i16,
        /// Roll, centidegrees.
        roll: i16,
    },
}

impl Attitude {
    /// The corner for a board's `state` (`None` while silent) and its `imu.model`
    /// (`store::IMU_MODEL`, `None` while unread, which counts as fitted).
    pub fn of(state: Option<&CyclicState>, imu_model: Option<u8>) -> Attitude {
        match (imu_model, state) {
            (Some(0), _) => Attitude::NoImu,
            (_, None) => Attitude::Silent,
            (_, Some(s)) => Attitude::Angles {
                pitch: s.pitch,
                roll: s.roll,
            },
        }
    }

    /// The corner's two text lines as the handheld prints them (`specs/handheld-remote.md` 5.2):
    /// "p 2.3°" and "r -0.8°"; "p --" and "r --" silent; "no IMU" and an empty second line.
    pub fn lines(&self) -> (String<12>, String<12>) {
        let mut pitch = String::new();
        let mut roll = String::new();
        // Every line is at most "p -327.7°", ten bytes.
        match self {
            Attitude::NoImu => {
                let _ = pitch.push_str("no IMU");
            }
            Attitude::Silent => {
                let _ = pitch.push_str("p --");
                let _ = roll.push_str("r --");
            }
            Attitude::Angles { pitch: p, roll: r } => {
                let _ = pitch.push_str("p ");
                let _ = pitch.push_str(&angle_text(*p));
                let _ = pitch.push('°');
                let _ = roll.push_str("r ");
                let _ = roll.push_str(&angle_text(*r));
                let _ = roll.push('°');
            }
        }
        (pitch, roll)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::readings::test_support::state;

    /// `RideReadingsTest.kt`, `angle text is to a tenth and never minus zero`.
    #[test]
    fn angle_text_is_to_a_tenth_and_never_minus_zero() {
        assert_eq!(angle_text(-4), "0.0");
        assert_eq!(angle_text(-5), "-0.1");
        assert_eq!(angle_text(225), "2.3");
        assert_eq!(angle_text(0), "0.0");
        assert_eq!(angle_text(-1_234), "-12.3");
    }

    /// `specs/handheld-remote.md` 5.2 (the attitude slots) and `specs/remote-ui.md` 3.1's no-value
    /// table; no Kotlin test covers the corner (the app's is `RideInstruments.kt`,
    /// `AttitudeCorner`). The angles are `RideReadingsTest.kt`'s 225 and the spec's -0.8.
    #[test]
    fn the_corner_lines() {
        let s = CyclicState {
            pitch: 225,
            roll: -80,
            ..state()
        };
        let a = Attitude::of(Some(&s), Some(1));
        assert_eq!(
            a,
            Attitude::Angles {
                pitch: 225,
                roll: -80
            }
        );
        let (p, r) = a.lines();
        assert_eq!((p.as_str(), r.as_str()), ("p 2.3°", "r -0.8°"));
        assert_eq!(Attitude::of(Some(&s), None), a, "unread counts as fitted");

        let (p, r) = Attitude::of(None, Some(1)).lines();
        assert_eq!((p.as_str(), r.as_str()), ("p --", "r --"));

        assert_eq!(Attitude::of(None, Some(0)), Attitude::NoImu, "no IMU wins");
        let (p, r) = Attitude::of(Some(&s), Some(0)).lines();
        assert_eq!((p.as_str(), r.as_str()), ("no IMU", ""));

        let (p, _) = Attitude::of(
            Some(&CyclicState {
                pitch: i16::MIN,
                ..s
            }),
            None,
        )
        .lines();
        assert_eq!(p, "p -327.7°");
    }
}
