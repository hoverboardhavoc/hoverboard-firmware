//! The machine's speed from `CYCLIC_STATE.wheel_rate` (`specs/remote-ui.md` 3.1a), ported from
//! the app's `model/SpeedReading.kt`.

use core::f64::consts::PI;

use heapless::{String, Vec};

use super::{round_half_up, write_tenths, SIDE_RIGHT};

/// What one board brings to the machine's speed (`SpeedReading.kt`, `SpeedFacts`): its wheel rate
/// and the three stored facts that turn it into a road speed. Each fact is `None` while unread and
/// 0 while unset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpeedFacts {
    /// The board's L3 address: whom a missing fact is missing on.
    pub address: u8,
    /// `CYCLIC_STATE.wheel_rate`: Q12 of one hall edge per 320-period window, in the board's own
    /// counter sense.
    pub wheel_rate_q12: i32,
    /// `motors.0.pole_pairs` (`store::MOTOR_POLE_PAIRS`).
    pub pole_pairs: Option<u8>,
    /// `motors.0.wheel_diameter_mm` (`store::WHEEL_DIAMETER_MM`).
    pub diameter_mm: Option<u16>,
    /// `control.side` (`store::CONTROL_SIDE`); [`SIDE_RIGHT`] mirrors the rate.
    pub side: Option<u8>,
}

/// One of the three facts a speed needs (`SpeedReading.kt`, `SpeedReading.POLE_PAIRS`,
/// `WHEEL_DIAMETER` and `SIDE`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpeedFact {
    /// `motors.0.pole_pairs`.
    PolePairs,
    /// `motors.0.wheel_diameter_mm`.
    WheelDiameter,
    /// `control.side`.
    Side,
}

/// What the speed says (`SpeedReading.kt`, `SpeedReading`; `specs/remote-ui.md` 3.1a).
#[derive(Debug, Clone, PartialEq)]
pub enum SpeedReading {
    /// No motor controller is reporting, so there is no rate to convert.
    Silent,
    /// A rate and no speed, because a fact is missing on a reporting board: `unread` are facts not
    /// read from a board yet, `unset` facts a board holds as 0, each named once, in the order the
    /// boards and their facts were met.
    NoSpeed {
        /// Facts not read yet.
        unread: Vec<SpeedFact, 3>,
        /// Facts held as 0.
        unset: Vec<SpeedFact, 3>,
    },
    /// The machine's forward speed.
    Kmh(Kmh),
}

/// A forward speed in km/h, negative backwards (`SpeedReading.kt`, `SpeedReading.Kmh`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Kmh(pub f64);

/// A printed speed (`SpeedReading.kt`, `SpeedReading.Shown`), signed: to a tenth (the figure in
/// tenths, so 9.9 is `Tenths(99)`), or whole km/h.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shown {
    /// A figure to one decimal, in tenths of a km/h.
    Tenths(i32),
    /// A whole number of km/h.
    Whole(i32),
}

impl Kmh {
    /// The figure as Ride prints it (`SpeedReading.kt`, `Kmh.shown`; `specs/remote-ui.md` 3.1a):
    /// tenths below 10 km/h and whole km/h at and above, decided on the value rounded to a tenth,
    /// so 9.96 is 10 and not 10.0. A reading that rounds to zero is `Tenths(0)`, never minus zero.
    /// Rounds half up, as `roundToInt` does.
    pub fn shown(self) -> Shown {
        let tenths = round_half_up(self.0 * TENTHS);
        if tenths.abs() < WHOLE_FROM_TENTHS {
            Shown::Tenths(tenths)
        } else {
            Shown::Whole(round_half_up(self.0))
        }
    }

    /// Whether "R" follows the unit: the mean forward speed is below zero
    /// (`ui/screens/RideInstruments.kt`, `Speed`, `speed.kmh < 0`; `specs/remote-ui.md` 3.1, "Never
    /// negative"). Decided on the unrounded speed, as the app does.
    pub fn reverse(self) -> bool {
        self.0 < 0.0
    }
}

/// One board's forward speed, km/h, from complete facts (`SpeedReading.kt`, `kmh`;
/// `specs/remote-ui.md` 3.1a): `wheel_rate / 4096 * 50 / (6 * pole_pairs) * pi * diameter * 0.0036`.
fn board_kmh(rate_q12: i32, pole_pairs: u8, diameter_mm: u16, right: bool) -> f64 {
    let forward = if right {
        -(rate_q12 as f64)
    } else {
        rate_q12 as f64
    };
    let edges_per_second = forward / Q12 * WINDOWS_PER_SECOND;
    let revs_per_second = edges_per_second / (EDGES_PER_ELECTRICAL_REV * pole_pairs as f64);
    revs_per_second * PI * diameter_mm as f64 * MM_PER_S_TO_KMH
}

/// The machine's speed (`SpeedReading.kt`, `machineSpeed`; `specs/remote-ui.md` 3.1a): the MEAN
/// of the reporting boards' speeds, each turned into the machine's forward sense by its own
/// `control.side`. A speed only when every reporting board has all three facts; otherwise
/// [`SpeedReading::NoSpeed`] naming what is missing, never a one-wheel figure.
pub fn machine_speed(boards: &[SpeedFacts]) -> SpeedReading {
    if boards.is_empty() {
        return SpeedReading::Silent;
    }
    let mut unread: Vec<SpeedFact, 3> = Vec::new();
    let mut unset: Vec<SpeedFact, 3> = Vec::new();
    fn add(list: &mut Vec<SpeedFact, 3>, fact: SpeedFact) {
        if !list.contains(&fact) {
            // Three facts and room for three: a push of a fact not yet held cannot fail.
            let _ = list.push(fact);
        }
    }
    let mut sum = 0.0;
    let mut complete = true;
    for b in boards {
        match b.pole_pairs {
            None => add(&mut unread, SpeedFact::PolePairs),
            Some(0) => add(&mut unset, SpeedFact::PolePairs),
            Some(_) => {}
        }
        match b.diameter_mm {
            None => add(&mut unread, SpeedFact::WheelDiameter),
            Some(0) => add(&mut unset, SpeedFact::WheelDiameter),
            Some(_) => {}
        }
        if b.side.is_none() {
            add(&mut unread, SpeedFact::Side);
        }
        match (b.pole_pairs, b.diameter_mm, b.side) {
            (Some(pp), Some(d), Some(side)) if pp > 0 && d > 0 => {
                sum += board_kmh(b.wheel_rate_q12, pp, d, side == SIDE_RIGHT);
            }
            _ => complete = false,
        }
    }
    if complete {
        SpeedReading::Kmh(Kmh(sum / boards.len() as f64))
    } else {
        SpeedReading::NoSpeed { unread, unset }
    }
}

/// A speed's magnitude as Ride prints it, never a minus sign (`SpeedReading.kt`, `speedText`;
/// `specs/remote-ui.md` 3.1): "7.4", "12", "0.0".
pub fn speed_text(shown: Shown) -> String<8> {
    let mut out = String::new();
    match shown {
        Shown::Tenths(t) => write_tenths(&mut out, t.abs()),
        Shown::Whole(w) => {
            use core::fmt::Write;
            // An i32's magnitude is at most ten digits: it fits.
            let _ = write!(out, "{}", w.unsigned_abs());
        }
    }
    out
}

/// `wheel_rate`'s Q12 unit: 4096 is one hall edge per window.
const Q12: f64 = 4096.0;
/// 320-period windows per second at the 16 kHz PWM.
const WINDOWS_PER_SECOND: f64 = 50.0;
/// Hall edges per electrical revolution: the six-state commutation sequence.
const EDGES_PER_ELECTRICAL_REV: f64 = 6.0;
/// Millimetres per second to km/h.
const MM_PER_S_TO_KMH: f64 = 0.0036;
const TENTHS: f64 = 10.0;
const WHOLE_FROM_TENTHS: i32 = 100;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::readings::SIDE_LEFT;

    const Q12_ONE: i32 = 4096;
    /// `SpeedReadingTest.kt`, `ONE_EDGE_KMH`: 50 / 90 turns a second of a 165 mm wheel, in km/h.
    const ONE_EDGE_KMH: f64 = 50.0 / 90.0 * PI * 165.0 * 0.0036;
    const TOLERANCE: f64 = 1e-9;

    /// `SpeedReadingTest.kt`, `board`: the rover's wheel, 15 pole pairs and 165 mm.
    fn board(rate: i32) -> SpeedFacts {
        SpeedFacts {
            address: 0x01,
            wheel_rate_q12: rate,
            pole_pairs: Some(15),
            diameter_mm: Some(165),
            side: Some(SIDE_LEFT),
        }
    }

    fn kmh(reading: SpeedReading) -> f64 {
        match reading {
            SpeedReading::Kmh(k) => k.0,
            other => panic!("not a speed: {other:?}"),
        }
    }

    fn facts(list: &[SpeedFact]) -> Vec<SpeedFact, 3> {
        Vec::from_slice(list).unwrap()
    }

    /// `SpeedReadingTest.kt`, `one edge per window on the rover wheel is about one km per hour`.
    #[test]
    fn one_edge_per_window_is_about_one_kmh() {
        assert!((kmh(machine_speed(&[board(Q12_ONE)])) - ONE_EDGE_KMH).abs() < TOLERANCE);
        assert!((kmh(machine_speed(&[board(-Q12_ONE)])) + ONE_EDGE_KMH).abs() < TOLERANCE);
    }

    /// `SpeedReadingTest.kt`, `a right wheel is folded into the forward sense`.
    #[test]
    fn a_right_wheel_is_folded_forward() {
        let right = SpeedFacts {
            side: Some(SIDE_RIGHT),
            ..board(-Q12_ONE)
        };
        assert!((kmh(machine_speed(&[right])) - ONE_EDGE_KMH).abs() < TOLERANCE);
    }

    /// `SpeedReadingTest.kt`, `the machine speed is the mean of the boards in the forward sense`.
    #[test]
    fn the_machine_speed_is_the_forward_mean() {
        let right = |rate| SpeedFacts {
            side: Some(SIDE_RIGHT),
            ..board(rate)
        };
        let straight = machine_speed(&[board(Q12_ONE), right(-3 * Q12_ONE)]);
        assert!((kmh(straight) - 2.0 * ONE_EDGE_KMH).abs() < TOLERANCE);
        let spin = machine_speed(&[board(Q12_ONE), right(Q12_ONE)]);
        assert!(kmh(spin).abs() < TOLERANCE);
    }

    /// `SpeedReadingTest.kt`, `an unset geometry field gives no speed and names the field`.
    #[test]
    fn an_unset_field_names_the_field() {
        let no_pp = SpeedFacts {
            pole_pairs: Some(0),
            ..board(Q12_ONE)
        };
        assert_eq!(
            machine_speed(&[no_pp]),
            SpeedReading::NoSpeed {
                unread: facts(&[]),
                unset: facts(&[SpeedFact::PolePairs])
            }
        );
        let no_d = SpeedFacts {
            side: Some(SIDE_RIGHT),
            diameter_mm: Some(0),
            ..board(Q12_ONE)
        };
        assert_eq!(
            machine_speed(&[board(Q12_ONE), no_d]),
            SpeedReading::NoSpeed {
                unread: facts(&[]),
                unset: facts(&[SpeedFact::WheelDiameter])
            }
        );
    }

    /// `SpeedReadingTest.kt`, `an unread field gives no speed and is named as unread`.
    #[test]
    fn an_unread_field_is_named_unread() {
        let b = SpeedFacts {
            side: None,
            diameter_mm: None,
            ..board(Q12_ONE)
        };
        assert_eq!(
            machine_speed(&[b]),
            SpeedReading::NoSpeed {
                unread: facts(&[SpeedFact::WheelDiameter, SpeedFact::Side]),
                unset: facts(&[])
            }
        );
    }

    /// `SpeedReadingTest.kt`, `no reporting board is silence`.
    #[test]
    fn no_reporting_board_is_silence() {
        assert_eq!(machine_speed(&[]), SpeedReading::Silent);
    }

    /// `SpeedReadingTest.kt`, `tenths below ten km per hour and whole from ten`.
    #[test]
    fn tenths_below_ten_and_whole_from_ten() {
        assert_eq!(Kmh(ONE_EDGE_KMH).shown(), Shown::Tenths(10));
        assert_eq!(Kmh(-3.26).shown(), Shown::Tenths(-33));
        assert_eq!(Kmh(9.94).shown(), Shown::Tenths(99));
        assert_eq!(Kmh(9.96).shown(), Shown::Whole(10));
        assert_eq!(Kmh(-12.4).shown(), Shown::Whole(-12));
        assert_eq!(Kmh(-0.02).shown(), Shown::Tenths(0), "never -0.0");
    }

    /// `RideReadingsTest.kt`, `the speed prints its magnitude`.
    #[test]
    fn the_speed_prints_its_magnitude() {
        assert_eq!(speed_text(Shown::Tenths(-74)), "7.4");
        assert_eq!(speed_text(Shown::Whole(-12)), "12");
        assert_eq!(speed_text(Shown::Tenths(0)), "0.0");
    }

    /// `specs/remote-ui.md` 3.1, "Never negative": "R" while the mean forward speed is below zero.
    /// No Kotlin test covers it (the app's rule is in `RideInstruments.kt`, `Speed`); the numbers
    /// are `SpeedReadingTest.kt`'s.
    #[test]
    fn reverse_shows_while_the_speed_is_below_zero() {
        assert!(Kmh(-12.4).reverse());
        assert!(!Kmh(ONE_EDGE_KMH).reverse());
        assert!(!Kmh(0.0).reverse());
    }
}
