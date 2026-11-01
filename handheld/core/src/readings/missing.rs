//! The settings the Ride figures lack, in plain words (`specs/remote-ui.md` 3.1, "Missing
//! settings, in plain words"), ported from the app's `model/MissingSettings.kt`. The handheld has no
//! Setup, so a statement carries no "Set it ›" link (`specs/handheld-remote.md` 5.4).

use heapless::Vec;

use super::battery::{BatteryReading, Unjudged};
use super::current::CurrentReading;
use super::speed::{SpeedFacts, SpeedReading};

/// A setting a Ride figure needs and lacks, or has not read yet (`MissingSettings.kt`,
/// `MissingSetting`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissingSetting {
    /// A reporting board holds `motors.0.pole_pairs` or `motors.0.wheel_diameter_mm` as 0.
    WheelSize,
    /// No speed fact is unset, and one is not read yet.
    WheelReading,
    /// The ring's reading is `Unjudged::NoCells`.
    Cells,
    /// The ring's reading is `Unjudged::NoChemistry`.
    ChemistryReading,
    /// The ring's reading is `Unjudged::UnknownChemistry`.
    ChemistryUnknown,
    /// A slot board's `motors.0.current.cal` reads 0.
    CurrentCal,
    /// A slot board's `motors.0.current.cal` is not read yet.
    CurrentCalReading,
}

impl MissingSetting {
    /// The statement (`specs/remote-ui.md` 3.1, the "Missing settings" table; the app's
    /// `strings.xml`, `ride_need_*` and `ride_reading_*`), without the "Set it ›" suffix.
    pub fn text(self) -> &'static str {
        match self {
            MissingSetting::WheelSize => "Speed needs the wheel size.",
            MissingSetting::WheelReading => "Reading the wheel settings for speed.",
            MissingSetting::Cells => "Battery % needs the cell count.",
            MissingSetting::ChemistryReading => "Reading the pack chemistry for battery %.",
            MissingSetting::ChemistryUnknown => "Battery % needs a chemistry this app knows.",
            MissingSetting::CurrentCal => {
                "Phase current needs the current calibration, set by the host config tool."
            }
            MissingSetting::CurrentCalReading => {
                "Reading the current calibration for phase current."
            }
        }
    }
}

/// One missing setting and the board it is missing on (`MissingSettings.kt`, `Missing`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Missing {
    /// What is missing.
    pub setting: MissingSetting,
    /// The board it is missing on, where one is named.
    pub board: Option<u8>,
}

/// Every missing setting the Ride figures show, in the statements' order: speed, battery, phase
/// (`MissingSettings.kt`, `missingSettings`; `specs/remote-ui.md` 3.1). The handheld prints the
/// first (`specs/handheld-remote.md` 5.4).
///
/// - the speed: `speed` lacking a fact; an unset one wins over one not read yet, and names the
///   first of `speed_facts`' boards (address order) holding pole pairs or diameter as 0;
/// - the battery: the ring's `battery` reading, unjudged, on `ring_board`;
/// - the phase current: the first of `slot_boards` (left then right) whose reading is
///   [`CurrentReading::Unscaled`], by its `counts_per_amp` (`None` while unread, 0 when unset).
pub fn missing_settings(
    speed: &SpeedReading,
    speed_facts: &[SpeedFacts],
    ring_board: Option<u8>,
    battery: &BatteryReading,
    slot_boards: &[(u8, CurrentReading)],
    counts_per_amp: impl Fn(u8) -> Option<u16>,
) -> Vec<Missing, 3> {
    let mut out = Vec::new();
    // At most one entry per figure, three figures: no push below can fail.
    if let SpeedReading::NoSpeed { unread, unset } = speed {
        if !unset.is_empty() {
            let board = speed_facts
                .iter()
                .filter(|f| f.pole_pairs == Some(0) || f.diameter_mm == Some(0))
                .map(|f| f.address)
                .min();
            let _ = out.push(Missing {
                setting: MissingSetting::WheelSize,
                board,
            });
        } else if !unread.is_empty() {
            let _ = out.push(Missing {
                setting: MissingSetting::WheelReading,
                board: None,
            });
        }
    }
    if let BatteryReading::Unjudged(_, why) = battery {
        let setting = match why {
            Unjudged::NoCells => MissingSetting::Cells,
            Unjudged::NoChemistry => MissingSetting::ChemistryReading,
            Unjudged::UnknownChemistry => MissingSetting::ChemistryUnknown,
        };
        let _ = out.push(Missing {
            setting,
            board: ring_board,
        });
    }
    if let Some((board, _)) = slot_boards
        .iter()
        .find(|(_, r)| *r == CurrentReading::Unscaled)
    {
        let setting = match counts_per_amp(*board) {
            None => MissingSetting::CurrentCalReading,
            Some(_) => MissingSetting::CurrentCal,
        };
        let _ = out.push(Missing {
            setting,
            board: Some(*board),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::readings::battery::Charge;
    use crate::readings::speed::machine_speed;
    use crate::readings::SIDE_LEFT;

    /// `RideReadingsTest.kt`, `facts`.
    fn facts(address: u8, pole_pairs: Option<u8>, diameter_mm: Option<u16>) -> SpeedFacts {
        SpeedFacts {
            address,
            wheel_rate_q12: 0,
            pole_pairs,
            diameter_mm,
            side: Some(SIDE_LEFT),
        }
    }

    const CHARGE_50: BatteryReading = BatteryReading::Charge(Charge {
        centivolts: 3700,
        fraction: 0.5,
        low: false,
    });

    /// `RideReadingsTest.kt`, `missing settings name the setting and the board`, `missing`.
    fn missing(
        speed: &[SpeedFacts],
        battery: BatteryReading,
        phase: &[(u8, CurrentReading)],
        cal: &[(u8, u16)],
    ) -> Vec<Missing, 3> {
        missing_settings(
            &machine_speed(speed),
            speed,
            Some(0x01),
            &battery,
            phase,
            |a| cal.iter().find(|(x, _)| *x == a).map(|(_, c)| *c),
        )
    }

    fn one(setting: MissingSetting, board: Option<u8>) -> Vec<Missing, 3> {
        Vec::from_slice(&[Missing { setting, board }]).unwrap()
    }

    /// `RideReadingsTest.kt`, `missing settings name the setting and the board`.
    #[test]
    fn missing_settings_name_the_setting_and_the_board() {
        let unset = [
            facts(0x01, Some(15), Some(165)),
            facts(0x02, Some(0), Some(165)),
        ];
        assert_eq!(
            missing(&unset, CHARGE_50, &[], &[]),
            one(MissingSetting::WheelSize, Some(0x02))
        );
        // An unset fact wins over an unread one.
        assert_eq!(
            missing(
                &[facts(0x01, None, Some(165)), facts(0x02, Some(15), Some(0))],
                CHARGE_50,
                &[],
                &[]
            ),
            one(MissingSetting::WheelSize, Some(0x02))
        );
        assert_eq!(
            missing(&[facts(0x01, None, Some(165))], CHARGE_50, &[], &[]),
            one(MissingSetting::WheelReading, None)
        );
        let whole = [facts(0x01, Some(15), Some(165))];
        assert!(missing(&whole, CHARGE_50, &[], &[]).is_empty());

        let why = |w| BatteryReading::Unjudged(3780, w);
        assert_eq!(
            missing(&whole, why(Unjudged::NoCells), &[], &[]),
            one(MissingSetting::Cells, Some(0x01))
        );
        assert_eq!(
            missing(&whole, why(Unjudged::NoChemistry), &[], &[]),
            one(MissingSetting::ChemistryReading, Some(0x01))
        );
        assert_eq!(
            missing(&whole, why(Unjudged::UnknownChemistry), &[], &[]),
            one(MissingSetting::ChemistryUnknown, Some(0x01))
        );

        let phase = [
            (0x01, CurrentReading::MotorsOff),
            (0x02, CurrentReading::Unscaled),
        ];
        assert_eq!(
            missing(&whole, CHARGE_50, &phase, &[]),
            one(MissingSetting::CurrentCalReading, Some(0x02))
        );
        assert_eq!(
            missing(&whole, CHARGE_50, &phase, &[(0x02, 0)]),
            one(MissingSetting::CurrentCal, Some(0x02))
        );

        // All at once, in order.
        let all: Vec<MissingSetting, 3> = missing(&unset, why(Unjudged::NoCells), &phase, &[])
            .iter()
            .map(|m| m.setting)
            .collect();
        assert_eq!(
            all.as_slice(),
            &[
                MissingSetting::WheelSize,
                MissingSetting::Cells,
                MissingSetting::CurrentCalReading
            ]
        );
    }

    /// `specs/remote-ui.md` 3.1, the "Missing settings" table's statements, without "Set it ›"
    /// (`specs/handheld-remote.md` 5.4); the app's strings carry no test of their own.
    #[test]
    fn the_statements_are_the_tables_words() {
        assert_eq!(
            MissingSetting::WheelSize.text(),
            "Speed needs the wheel size."
        );
        assert_eq!(
            MissingSetting::WheelReading.text(),
            "Reading the wheel settings for speed."
        );
        assert_eq!(
            MissingSetting::Cells.text(),
            "Battery % needs the cell count."
        );
        assert_eq!(
            MissingSetting::ChemistryReading.text(),
            "Reading the pack chemistry for battery %."
        );
        assert_eq!(
            MissingSetting::ChemistryUnknown.text(),
            "Battery % needs a chemistry this app knows."
        );
        assert_eq!(
            MissingSetting::CurrentCal.text(),
            "Phase current needs the current calibration, set by the host config tool."
        );
        assert_eq!(
            MissingSetting::CurrentCalReading.text(),
            "Reading the current calibration for phase current."
        );
    }
}
