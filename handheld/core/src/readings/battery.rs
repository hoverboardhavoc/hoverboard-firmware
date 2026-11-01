//! The battery ring and pack line (`specs/remote-ui.md` 3.1, 3.6b and 3.6d), ported from the app's
//! `model/BatteryCurve.kt` and `model/RideReadings.kt`.
//!
//! Voltages stay in centivolts, the battery word's unit (`CyclicState::battery`), so a table's
//! endpoints and the floor comparison are exact.

use linkctl::CyclicState;

/// `board.battery.chemistry` (`store::BOARD_BATTERY_CHEMISTRY`) `0`: Li-ion NMC, the default. The
/// store crate names the bytes only in its doc comment and host-only name table, so they are stated
/// here; the app's mirror is `Fields.BatteryChemistry`.
pub const LI_ION: u8 = 0;
/// `board.battery.chemistry` `1`: LEV50 ([`LI_ION`] for the source).
pub const LEV50: u8 = 1;
/// `board.battery.chemistry` `2`: lithium titanate ([`LI_ION`] for the source).
pub const LTO: u8 = 2;

/// Each chemistry's resting cell voltage at 0, 10, ... 100 %, centivolts, indexed by the chemistry
/// byte (`BatteryCurve.kt`, `TABLES`; `specs/remote-ui.md` 3.6d). Typical datasheet shapes, not
/// measurements of these packs.
const TABLES: [[u16; 11]; 3] = [
    [300, 345, 355, 362, 368, 374, 380, 388, 397, 407, 420],
    [275, 350, 358, 363, 368, 373, 379, 386, 394, 402, 410],
    [170, 215, 222, 226, 229, 232, 236, 240, 245, 252, 270],
];

/// Charge at or below this fraction is low (`BatteryCurve.kt`, `LOW_FRACTION`).
pub const LOW_FRACTION: f64 = 0.15;

/// Remaining charge in `0.0..=1.0` for `centivolts` across `cells` cells of `chemistry`, read
/// linearly between the table's points and clamped at its ends, or `None` for a chemistry with no
/// table (`BatteryCurve.kt`, `BatteryCurve.fraction`; `specs/remote-ui.md` 3.6d). `cells` is not 0.
pub fn fraction(centivolts: u16, cells: u8, chemistry: u8) -> Option<f64> {
    let table = TABLES.get(chemistry as usize)?;
    let per_cell = centivolts as f64 / cells as f64;
    if per_cell <= table[0] as f64 {
        return Some(0.0);
    }
    if per_cell >= table[10] as f64 {
        return Some(1.0);
    }
    let upper = table.iter().position(|&t| t as f64 > per_cell)?;
    let lo = table[upper - 1] as f64;
    let step = (per_cell - lo) / (table[upper] as f64 - lo);
    Some(((upper - 1) as f64 + step) / (table.len() - 1) as f64)
}

/// Why a measured voltage has no state of charge (`BatteryCurve.kt`, `BatteryReading.Unjudged.Why`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unjudged {
    /// `board.battery.cells` is unread or 0.
    NoCells,
    /// `board.battery.chemistry` has not been read yet.
    NoChemistry,
    /// The chemistry is a byte with no table.
    UnknownChemistry,
}

/// A measured voltage against the stated pack (`BatteryCurve.kt`, `BatteryReading.Charge`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Charge {
    /// The pack voltage, centivolts.
    pub centivolts: u16,
    /// How full, `0.0..=1.0`.
    pub fraction: f64,
    /// Whether `fraction` is [`LOW_FRACTION`] or less.
    pub low: bool,
}

impl Charge {
    /// The fraction as a whole percent, rounded half up (`BatteryCurve.kt`, `Charge.percent`).
    pub fn percent(&self) -> u8 {
        libm::floor(self.fraction * 100.0 + 0.5) as u8
    }
}

/// What one board's battery display says (`BatteryCurve.kt`, `BatteryReading`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BatteryReading {
    /// The board is reporting no pack voltage: nothing arrived, or it sent `battery = 0`, the
    /// firmware's UNKNOWN (`TelemetryUi.kt`, `batteryVolts`).
    NoReading,
    /// A measured voltage, centivolts, with no pack to judge it against, for the reason given.
    Unjudged(u16, Unjudged),
    /// A measured voltage against the stated pack.
    Charge(Charge),
}

impl BatteryReading {
    /// The measured pack voltage, centivolts, or `None` with no reading: what the pack line
    /// prints beside the percent, or alone while unjudged (`specs/remote-ui.md` 3.1).
    pub fn centivolts(&self) -> Option<u16> {
        match self {
            BatteryReading::NoReading => None,
            BatteryReading::Unjudged(cv, _) => Some(*cv),
            BatteryReading::Charge(c) => Some(c.centivolts),
        }
    }
}

/// What a board's `state` (`None` while it is silent), its `board.battery.cells` and its
/// `board.battery.chemistry` (each `None` while unread) come to (`BatteryCurve.kt`,
/// `batteryReading`, with `TelemetryUi.kt`'s `batteryVolts`; `MainViewModel.kt`, `batteryOf`;
/// `specs/remote-ui.md` 3.6b). A cell count of 0 or an unread or unknown chemistry leaves the
/// voltage unjudged, never judged against a guessed pack.
pub fn battery_reading(
    state: Option<&CyclicState>,
    cells: Option<u8>,
    chemistry: Option<u8>,
) -> BatteryReading {
    let Some(cv) = state.map(|s| s.battery).filter(|&b| b != 0) else {
        return BatteryReading::NoReading;
    };
    let Some(cells) = cells.filter(|&c| c > 0) else {
        return BatteryReading::Unjudged(cv, Unjudged::NoCells);
    };
    let Some(chemistry) = chemistry else {
        return BatteryReading::Unjudged(cv, Unjudged::NoChemistry);
    };
    match fraction(cv, cells, chemistry) {
        None => BatteryReading::Unjudged(cv, Unjudged::UnknownChemistry),
        Some(f) => BatteryReading::Charge(Charge {
            centivolts: cv,
            fraction: f,
            low: f <= LOW_FRACTION,
        }),
    }
}

/// The board whose reading the ring shows, and that reading (`RideReadings.kt`, `ringReading`;
/// `specs/remote-ui.md` 3.1, "Whose reading"): of the `reporting` motor controllers (any order),
/// the one whose reading is a [`Charge`] with the lowest percent, the lower address on a tie;
/// with no `Charge`, the lowest-addressed reporting board's reading; `(None, NoReading)` while
/// none reports.
pub fn ring_reading(
    reporting: &[u8],
    battery_of: impl Fn(u8) -> BatteryReading,
) -> (Option<u8>, BatteryReading) {
    let mut best: Option<(u8, Charge)> = None;
    for &a in reporting {
        if let BatteryReading::Charge(c) = battery_of(a) {
            let better = match best {
                None => true,
                Some((ba, bc)) => (c.percent(), a) < (bc.percent(), ba),
            };
            if better {
                best = Some((a, c));
            }
        }
    }
    if let Some((a, c)) = best {
        return (Some(a), BatteryReading::Charge(c));
    }
    match reporting.iter().copied().min() {
        None => (None, BatteryReading::NoReading),
        Some(first) => (Some(first), battery_of(first)),
    }
}

/// Whether the ring and the pack line read caution (`RideReadings.kt`, `batteryLow`;
/// `specs/remote-ui.md` 3.1, "Low"): the reading's own [`Charge::low`], or its voltage at or below
/// the same board's `board.battery.floor` (`store::BOARD_BATTERY_FLOOR`, centivolts) while that
/// floor is above 0. A floor of 0 or unread adds nothing.
pub fn battery_low(reading: &BatteryReading, floor_centivolts: Option<i16>) -> bool {
    if let BatteryReading::Charge(c) = reading {
        if c.low {
            return true;
        }
    }
    let Some(cv) = reading.centivolts() else {
        return false;
    };
    match floor_centivolts {
        Some(floor) if floor > 0 => cv as i32 <= floor as i32,
        _ => false,
    }
}

/// How many of the ring's ten segments `percent` lights, `ceil(percent / 10)`, so 0 lights none
/// (`RideReadings.kt`, `litSegments`; `specs/remote-ui.md` 3.1).
pub fn lit_segments(percent: u8) -> u8 {
    let p = percent.min(100);
    p.div_ceil(10)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::readings::test_support::state;

    /// `BatteryCurveTest.kt`, `rover`: eight cells.
    const ROVER: u8 = 8;
    /// `BatteryCurveTest.kt`, `stock`: ten cells.
    const STOCK: u8 = 10;

    fn near(a: f64, b: f64) -> bool {
        (a - b).abs() < 0.0001
    }

    fn at(cv: u16) -> linkctl::CyclicState {
        linkctl::CyclicState {
            battery: cv,
            ..state()
        }
    }

    /// `BatteryCurveTest.kt`, `each table's endpoints are its cell's times the cell count` (the
    /// `fraction` half; `packCentivolts` is not ported).
    #[test]
    fn each_tables_endpoints() {
        assert!(near(fraction(4200, STOCK, LI_ION).unwrap(), 1.0));
        assert!(near(fraction(3000, STOCK, LI_ION).unwrap(), 0.0));
        assert!(near(fraction(3280, ROVER, LEV50).unwrap(), 1.0));
        assert!(near(fraction(2200, ROVER, LEV50).unwrap(), 0.0));
        assert!(near(fraction(2160, ROVER, LTO).unwrap(), 1.0));
        assert!(near(fraction(1360, ROVER, LTO).unwrap(), 0.0));
    }

    /// `BatteryCurveTest.kt`, `a point of the table reads as its own percent`.
    #[test]
    fn a_point_reads_as_its_percent() {
        assert!(near(fraction(2984, ROVER, LEV50).unwrap(), 0.5));
    }

    /// `BatteryCurveTest.kt`, `between two points the read is linear`.
    #[test]
    fn between_two_points_is_linear() {
        assert!(near(fraction(3710, STOCK, LI_ION).unwrap(), 0.45));
        assert!(near(fraction(1844, ROVER, LTO).unwrap(), 0.45));
    }

    /// `BatteryCurveTest.kt`, `the same voltage reads differently on two packs`.
    #[test]
    fn the_same_voltage_on_two_packs() {
        assert!(near(fraction(3000, ROVER, LI_ION).unwrap(), 0.51666));
        assert!(near(fraction(3000, STOCK, LI_ION).unwrap(), 0.0));
    }

    /// `BatteryCurveTest.kt`, `voltages outside the table clamp`.
    #[test]
    fn voltages_outside_clamp() {
        assert!(near(fraction(3500, ROVER, LI_ION).unwrap(), 1.0));
        assert!(near(fraction(2000, ROVER, LI_ION).unwrap(), 0.0));
        assert!(near(fraction(3000, ROVER, LTO).unwrap(), 1.0));
        assert!(near(fraction(1000, ROVER, LTO).unwrap(), 0.0));
    }

    /// `BatteryCurveTest.kt`, `a chemistry the app has no table for has no fraction and no endpoints`.
    #[test]
    fn an_unknown_chemistry_has_no_fraction() {
        assert_eq!(fraction(3000, ROVER, 3), None);
    }

    /// `BatteryCurveTest.kt`, `a board reporting no pack voltage has no reading`.
    #[test]
    fn no_pack_voltage_is_no_reading() {
        assert_eq!(
            battery_reading(Some(&at(0)), Some(ROVER), Some(LI_ION)),
            BatteryReading::NoReading
        );
        assert_eq!(
            battery_reading(None, Some(ROVER), Some(LI_ION)),
            BatteryReading::NoReading
        );
    }

    /// `BatteryCurveTest.kt`, `an unstated pack yields the volts and no charge`.
    #[test]
    fn an_unstated_pack_is_unjudged() {
        let s = at(2900);
        let want = BatteryReading::Unjudged(2900, Unjudged::NoCells);
        assert_eq!(battery_reading(Some(&s), None, Some(LI_ION)), want);
        assert_eq!(battery_reading(Some(&s), Some(0), Some(LI_ION)), want);
    }

    /// `BatteryCurveTest.kt`, `an unknown chemistry yields the volts and no charge`.
    #[test]
    fn an_unknown_chemistry_is_unjudged() {
        assert_eq!(
            battery_reading(Some(&at(2900)), Some(ROVER), Some(3)),
            BatteryReading::Unjudged(2900, Unjudged::UnknownChemistry)
        );
    }

    /// `BatteryCurveTest.kt`, `an unread chemistry yields the volts and no charge, as unread`.
    #[test]
    fn an_unread_chemistry_is_unjudged_as_unread() {
        assert_eq!(
            battery_reading(Some(&at(2900)), Some(ROVER), None),
            BatteryReading::Unjudged(2900, Unjudged::NoChemistry)
        );
    }

    /// `BatteryCurveTest.kt`, `a stated pack yields the volts, the fraction and the percent`.
    #[test]
    fn a_stated_pack_yields_the_charge() {
        let BatteryReading::Charge(c) = battery_reading(Some(&at(2984)), Some(ROVER), Some(LEV50))
        else {
            panic!("not a charge");
        };
        assert_eq!(c.centivolts, 2984);
        assert_eq!(
            battery_reading(Some(&at(2984)), Some(ROVER), Some(LEV50)).centivolts(),
            Some(2984)
        );
        assert!(near(c.fraction, 0.5));
        assert_eq!(c.percent(), 50);
    }

    /// `BatteryCurveTest.kt`, `low is the bottom fraction of the charge, whichever pack it is`.
    #[test]
    fn low_is_the_bottom_fraction() {
        let low = |cv, chem| match battery_reading(Some(&at(cv)), Some(ROVER), Some(chem)) {
            BatteryReading::Charge(c) => c.low,
            other => panic!("not a charge: {other:?}"),
        };
        assert!(low(2800, LI_ION));
        assert!(!low(2802, LI_ION));
        assert!(low(2830, LEV50));
        assert!(!low(2834, LEV50));
    }

    /// `RideReadingsTest.kt`, `lit segments are the percent over ten, rounded up`.
    #[test]
    fn lit_segments_round_up() {
        assert_eq!(lit_segments(0), 0);
        assert_eq!(lit_segments(1), 1);
        assert_eq!(lit_segments(10), 1);
        assert_eq!(lit_segments(11), 2);
        assert_eq!(lit_segments(100), 10);
    }

    /// `RideReadingsTest.kt`, `charge`.
    fn charge(percent: u8, centivolts: u16) -> BatteryReading {
        BatteryReading::Charge(Charge {
            centivolts,
            fraction: percent as f64 / 100.0,
            low: percent <= 15,
        })
    }

    fn lookup(table: &[(u8, BatteryReading)]) -> impl Fn(u8) -> BatteryReading + '_ {
        move |a| table.iter().find(|(x, _)| *x == a).unwrap().1
    }

    /// `RideReadingsTest.kt`, `the ring takes the lowest charge, else the lowest-addressed reading`.
    #[test]
    fn the_ring_takes_the_lowest_charge() {
        let readings = [(0x01, charge(72, 3700)), (0x02, charge(68, 3700))];
        assert_eq!(
            ring_reading(&[0x01, 0x02], lookup(&readings)),
            (Some(0x02), charge(68, 3700))
        );

        let tie = [(0x02, charge(50, 3700)), (0x01, charge(50, 3700))];
        assert_eq!(ring_reading(&[0x02, 0x01], lookup(&tie)).0, Some(0x01));

        let one = [(0x01, BatteryReading::NoReading), (0x02, charge(90, 3700))];
        assert_eq!(
            ring_reading(&[0x01, 0x02], lookup(&one)),
            (Some(0x02), charge(90, 3700)),
            "a Charge beats no reading"
        );

        let unjudged = BatteryReading::Unjudged(3000, Unjudged::NoCells);
        let none = [(0x02, BatteryReading::NoReading), (0x01, unjudged)];
        assert_eq!(ring_reading(&[0x02, 0x01], lookup(&none)).0, Some(0x01));

        assert_eq!(
            ring_reading(&[], |_| BatteryReading::NoReading),
            (None, BatteryReading::NoReading)
        );
    }

    /// `RideReadingsTest.kt`, `low is the bottom 15 percent or at or under the floor`.
    #[test]
    fn low_is_the_bottom_15_percent_or_the_floor() {
        assert!(battery_low(&charge(15, 3700), None));
        assert!(!battery_low(&charge(16, 3700), None));

        assert!(battery_low(&charge(60, 3300), Some(3300)), "at the floor");
        assert!(
            !battery_low(&charge(60, 3301), Some(3300)),
            "a centivolt above it"
        );
        assert!(
            !battery_low(&charge(60, 100), Some(0)),
            "a floor of 0 adds nothing"
        );
        assert!(
            !battery_low(&charge(60, 100), None),
            "an unread floor adds nothing"
        );
        assert!(!battery_low(&BatteryReading::NoReading, Some(3300)));
    }
}
