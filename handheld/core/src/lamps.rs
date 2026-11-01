//! The fault lamp grid (`specs/handheld-remote.md` 5.6, `specs/remote-ui.md` 3.1b), ported from
//! the app's `model/FaultLamp.kt` and the lamp rules of `MainViewModel.kt` (`lampWords`,
//! `gridShows`, `onFaultWords`' latch hold) and `ui/screens/FaultGrid.kt` (`lit`, `litOn`). No
//! sheets: the handheld has no touch target for one.
//!
//! A board's lamp word is its `CYCLIC_STATE.fault` masked to [`LAMP_BITS`], with any latch bit the
//! handheld holds for it ([`LatchHold::lamp_word`]). The functions below take the reporting boards'
//! lamp words as `(address, word)` pairs.

use heapless::Vec;
use linkctl::{
    ARM_BLOCKING_BITS, EV_COMMS_LOSS, EV_CONFIG, EV_IMU_LOSS, EV_LATCH_A, EV_LATCH_B, EV_MOTOR_ARM,
    EV_MOTOR_CAL, EV_MOTOR_HALL, EV_MOTOR_PERIOD, EV_STOP_ALL,
};

/// One lamp of the grid, in the grid's order (`FaultLamp.kt`, `FaultLamp`; `specs/remote-ui.md`
/// 3.1b, the lamp table): one per bit of [`ARM_BLOCKING_BITS`], and CONFIG last.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lamp {
    /// Bit 0, `EV_COMMS_LOSS`.
    Link,
    /// Bit 1, `EV_STOP_ALL`.
    Tilt,
    /// Bit 2, `EV_IMU_LOSS`.
    Imu,
    /// Bit 4, `EV_LATCH_A`.
    Latch,
    /// Bit 5, `EV_LATCH_B`.
    Latch2,
    /// Bit 8, `EV_MOTOR_HALL`.
    Hall,
    /// Bit 9, `EV_MOTOR_CAL`.
    Cal,
    /// Bit 10, `EV_MOTOR_PERIOD`.
    Period,
    /// Bit 11, `EV_MOTOR_ARM`.
    Arm,
    /// Bit 12, `EV_CONFIG`: blocks nothing.
    Config,
}

/// Every lamp, in the grid's order (`FaultLamp.kt`, `FaultLamp.entries`).
pub const LAMPS: [Lamp; 10] = [
    Lamp::Link,
    Lamp::Tilt,
    Lamp::Imu,
    Lamp::Latch,
    Lamp::Latch2,
    Lamp::Hall,
    Lamp::Cal,
    Lamp::Period,
    Lamp::Arm,
    Lamp::Config,
];

/// The two latch bits the handheld holds lit until the next BOOT press (`FaultLamp.kt`,
/// `LATCH_BITS`): the board clears a latch about 8 ms after it trips.
pub const LATCH_BITS: u16 = EV_LATCH_A | EV_LATCH_B;

/// Every bit a lamp draws: the arm-blocking ones and CONFIG (`FaultLamp.kt`, `LAMP_BITS`).
pub const LAMP_BITS: u16 = ARM_BLOCKING_BITS | EV_CONFIG;

/// The grid's column count on the handheld (`specs/handheld-remote.md` 5.6: five columns of 41 px,
/// two rows), so ten lamps fit in two rows. The app's four-or-five rule (`lampColumns`) is a phone
/// layout and is not ported.
pub const GRID_COLUMNS: usize = 5;

impl Lamp {
    /// The lamp's bit mask in the fault word (`FaultLamp.kt`, `bit`).
    pub const fn bit(self) -> u16 {
        match self {
            Lamp::Link => EV_COMMS_LOSS,
            Lamp::Tilt => EV_STOP_ALL,
            Lamp::Imu => EV_IMU_LOSS,
            Lamp::Latch => EV_LATCH_A,
            Lamp::Latch2 => EV_LATCH_B,
            Lamp::Hall => EV_MOTOR_HALL,
            Lamp::Cal => EV_MOTOR_CAL,
            Lamp::Period => EV_MOTOR_PERIOD,
            Lamp::Arm => EV_MOTOR_ARM,
            Lamp::Config => EV_CONFIG,
        }
    }

    /// The name the lamp carries in the grid (`FaultLamp.kt`, `label`).
    pub const fn label(self) -> &'static str {
        match self {
            Lamp::Link => "LINK",
            Lamp::Tilt => "TILT",
            Lamp::Imu => "IMU",
            Lamp::Latch => "LATCH",
            Lamp::Latch2 => "LATCH 2",
            Lamp::Hall => "HALL",
            Lamp::Cal => "CAL",
            Lamp::Period => "PERIOD",
            Lamp::Arm => "ARM",
            Lamp::Config => "CONFIG",
        }
    }

    /// The lamp's title, its name in a statement ("Disarmed: <title>, <board>.") (`FaultLamp.kt`,
    /// `title`; `strings.xml`, `lamp_*_title`; `specs/remote-ui.md` 3.1b, the lamp table).
    pub const fn title(self) -> &'static str {
        match self {
            Lamp::Link => "Board link lost",
            Lamp::Tilt => "Tilted past 45°",
            Lamp::Imu => "IMU not answering",
            Lamp::Latch => "Motor fault latched",
            Lamp::Latch2 => "Second motor fault latched",
            Lamp::Hall => "Hall sensor fault",
            Lamp::Cal => "Current calibration refused",
            Lamp::Period => "Motor timer stalled",
            Lamp::Arm => "Arm aborted",
            Lamp::Config => "Configuration incomplete",
        }
    }

    /// Whether the handheld holds this lamp lit after its bit clears (`FaultLamp.kt`, `held`).
    pub const fn held(self) -> bool {
        self.bit() & LATCH_BITS != 0
    }

    /// Whether this lamp's bit stops a board arming: every lamp but CONFIG (`FaultLamp.kt`,
    /// `blocksArm`).
    pub const fn blocks_arm(self) -> bool {
        self.bit() & ARM_BLOCKING_BITS != 0
    }

    /// The lamp for one bit mask of the fault word, or `None` for a bit that is not a lamp
    /// (`FaultLamp.kt`, `FaultLamp.of`).
    pub fn of(bit: u16) -> Option<Lamp> {
        LAMPS.into_iter().find(|l| l.bit() == bit)
    }

    /// The first lamp, in the grid's order, whose bit is set in `word`: the cause a fault disarm
    /// keeps for its statement (`MainViewModel.kt`, `faultCause`, `FaultLamp.lit(word).firstOrNull()`;
    /// `specs/handheld-remote.md` 4.7).
    pub fn first_lit(word: u16) -> Option<Lamp> {
        LAMPS.into_iter().find(|l| word & l.bit() != 0)
    }
}

/// What decides which lamps apply on one motor controller (`FaultLamp.kt`, `LampFacts`): its
/// address and its stored `imu.model` (`store::IMU_MODEL`), `None` while unread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LampFacts {
    /// The board's L3 address.
    pub address: u8,
    /// `imu.model`, `None` while unread.
    pub imu_model: Option<u8>,
}

impl LampFacts {
    /// Whether the board has an IMU fitted; unread counts as fitted, so an unread fact never hides
    /// a lamp (`FaultLamp.kt`, `LampFacts.imuFitted`).
    pub fn imu_fitted(&self) -> bool {
        self.imu_model != Some(0)
    }
}

/// Whether `lamp` applies on the machine whose motor controllers are `facts` (`FaultLamp.kt`,
/// `applies`; `specs/remote-ui.md` 3.1b, "Only applicable lamps are shown").
fn applies(lamp: Lamp, facts: &[LampFacts]) -> bool {
    match lamp {
        Lamp::Link => facts.len() == 2,
        Lamp::Tilt | Lamp::Imu => facts.iter().any(LampFacts::imu_fitted),
        Lamp::Latch2 | Lamp::Config => false,
        Lamp::Latch | Lamp::Hall | Lamp::Cal | Lamp::Period | Lamp::Arm => !facts.is_empty(),
    }
}

/// The lamps the grid shows, in the grid's order (`FaultLamp.kt`, `shownLamps`;
/// `specs/remote-ui.md` 3.1b): each lamp that applies on at least one motor controller in `facts`
/// (every one the walk found, so their count is what LINK reads), and each whose bit is set in any
/// reporting board's lamp word in `words`. A set bit is never hidden.
///
/// - LINK applies when the walk found two motor controllers.
/// - TILT and IMU apply when a board's IMU is fitted ([`LampFacts::imu_fitted`]).
/// - LATCH 2 and CONFIG never apply: they show only while their bit is set.
/// - LATCH, HALL, CAL, PERIOD and ARM apply on every motor controller.
pub fn shown_lamps(facts: &[LampFacts], words: &[(u8, u16)]) -> Vec<Lamp, 10> {
    let set = words.iter().fold(0, |acc, (_, w)| acc | w);
    LAMPS
        .into_iter()
        .filter(|&l| set & l.bit() != 0 || applies(l, facts))
        .collect()
}

/// Whether `lamp` is drawn lit: its bit is set on any reporting board, or the lamp test is on
/// (`FaultGrid.kt`, `Lamp`'s `on` and `UiState.lit`; `specs/remote-ui.md` 3.1b, the lamp states).
pub fn lamp_on(lamp: Lamp, words: &[(u8, u16)], lamp_test: bool) -> bool {
    lamp_test || words.iter().any(|(_, w)| w & lamp.bit() != 0)
}

/// Whether the board at `address`'s mark inside `lamp` is drawn lit: its own word has the bit, or
/// the lamp test is on (`FaultGrid.kt`, `litOn` and the mark rule).
pub fn mark_on(lamp: Lamp, address: u8, words: &[(u8, u16)], lamp_test: bool) -> bool {
    lamp_test
        || words
            .iter()
            .any(|&(a, w)| a == address && w & lamp.bit() != 0)
}

/// Whether the grid is drawn (`MainViewModel.kt`, `gridShows`; `specs/handheld-remote.md` 5.6): the
/// lamp test is on, or a reporting board's lamp word has an arm-blocking bit, which covers a LATCH
/// hold that is the only lit lamp. CONFIG alone does not draw it.
pub fn grid_shows(words: &[(u8, u16)], lamp_test: bool) -> bool {
    lamp_test || words.iter().any(|(_, w)| w & ARM_BLOCKING_BITS != 0)
}

/// The grid cell, `(row, column)`, of the lamp at `index` in [`shown_lamps`]' list
/// (`specs/handheld-remote.md` 5.6: [`GRID_COLUMNS`] columns, filled row by row as the app's
/// `FaultGrid` chunks its list).
pub fn grid_cell(index: usize) -> (usize, usize) {
    (index / GRID_COLUMNS, index % GRID_COLUMNS)
}

/// The LATCH and LATCH 2 hold (`MainViewModel.kt`, `onFaultWords` and `LocalState.latchHeld`;
/// `specs/remote-ui.md` 3.1b): a latch bit seen once stays lit, per board and per bit, until the
/// next BOOT press. It survives the grid hiding and showing; a disconnect drops it by starting a
/// new [`LatchHold::new`]. Held bits are not a bit any board reports, so they refuse no arm
/// (`specs/handheld-remote.md` 4.5).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LatchHold {
    /// Held latch bits by L3 address: every address a `u8` can name, so no board is ever left out.
    held: [u16; 256],
}

impl Default for LatchHold {
    fn default() -> Self {
        Self::new()
    }
}

impl LatchHold {
    /// Nothing held: what a connection starts with.
    pub const fn new() -> LatchHold {
        LatchHold { held: [0; 256] }
    }

    /// Folds one frame's fault word from the board at `address` in: its latch bits join the hold.
    pub fn observe(&mut self, address: u8, fault: u16) {
        self.held[address as usize] |= fault & LATCH_BITS;
    }

    /// The latch bits held for the board at `address`.
    pub fn held(&self, address: u8) -> u16 {
        self.held[address as usize]
    }

    /// The board's lamp word as the grid draws it (`MainViewModel.kt`, `lampWords`): its live
    /// `fault` masked to [`LAMP_BITS`], with its held latch bits.
    pub fn lamp_word(&self, address: u8, fault: u16) -> u16 {
        (fault & LAMP_BITS) | self.held(address)
    }

    /// Drops every hold: the BOOT press that ends a hold also arms (`specs/remote-ui.md` 3.1b, the
    /// app's ARM tap).
    pub fn clear_on_boot_press(&mut self) {
        self.held = [0; 256];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one(imu: Option<u8>) -> [LampFacts; 1] {
        [LampFacts {
            address: 0x01,
            imu_model: imu,
        }]
    }

    fn two(imu: Option<u8>) -> [LampFacts; 2] {
        [
            LampFacts {
                address: 0x01,
                imu_model: imu,
            },
            LampFacts {
                address: 0x02,
                imu_model: imu,
            },
        ]
    }

    fn zero_words(facts: &[LampFacts]) -> Vec<(u8, u16), 2> {
        facts.iter().map(|f| (f.address, 0)).collect()
    }

    /// `FaultLampTest.kt`, `each lamp applies where the firmware can light it`.
    #[test]
    fn each_lamp_applies_where_the_firmware_can_light_it() {
        for count in [1, 2] {
            for imu in [Some(0), Some(1), None] {
                let (a, b) = (one(imu), two(imu));
                let facts: &[LampFacts] = if count == 1 { &a } else { &b };
                let shown = shown_lamps(facts, &zero_words(facts));
                assert_eq!(count == 2, shown.contains(&Lamp::Link), "LINK on {count}");
                assert_eq!(imu != Some(0), shown.contains(&Lamp::Tilt), "TILT {imu:?}");
                assert_eq!(imu != Some(0), shown.contains(&Lamp::Imu), "IMU {imu:?}");
                for lamp in [Lamp::Latch, Lamp::Hall, Lamp::Cal, Lamp::Period, Lamp::Arm] {
                    assert!(shown.contains(&lamp), "{lamp:?} on every motor controller");
                }
                assert!(!shown.contains(&Lamp::Latch2), "LATCH 2 never applies");
                assert!(!shown.contains(&Lamp::Config), "CONFIG never applies");
            }
        }
    }

    /// `FaultLampTest.kt`, `an imu on one board shows the imu lamps for the machine`.
    #[test]
    fn an_imu_on_one_board_shows_the_imu_lamps() {
        let facts = [
            LampFacts {
                address: 0x01,
                imu_model: Some(1),
            },
            LampFacts {
                address: 0x02,
                imu_model: Some(0),
            },
        ];
        let shown = shown_lamps(&facts, &[]);
        assert!(shown.contains(&Lamp::Tilt) && shown.contains(&Lamp::Imu));
    }

    /// `FaultLampTest.kt`, `latch 2 is absent until its bit is set`.
    #[test]
    fn latch_2_is_absent_until_its_bit_is_set() {
        assert!(!shown_lamps(&two(Some(1)), &[(0x01, 0), (0x02, 0)]).contains(&Lamp::Latch2));
        assert!(
            shown_lamps(&two(Some(1)), &[(0x01, 0), (0x02, EV_LATCH_B)]).contains(&Lamp::Latch2)
        );
    }

    /// `FaultLampTest.kt`, `config is absent until its bit is set, and blocks nothing`.
    #[test]
    fn config_is_absent_until_set_and_blocks_nothing() {
        assert!(!shown_lamps(&two(Some(1)), &[(0x01, 0), (0x02, 0)]).contains(&Lamp::Config));
        assert!(shown_lamps(&two(Some(1)), &[(0x01, 0), (0x02, EV_CONFIG)]).contains(&Lamp::Config));
        assert!(!Lamp::Config.blocks_arm());
        let blocking: Vec<Lamp, 10> = LAMPS.into_iter().filter(|l| l.blocks_arm()).collect();
        assert_eq!(blocking.as_slice(), &LAMPS[..9]);
    }

    /// `FaultLampTest.kt`, `a set bit is never hidden`.
    #[test]
    fn a_set_bit_is_never_hidden() {
        for lamp in LAMPS {
            assert!(
                shown_lamps(&one(Some(0)), &[(0x01, lamp.bit())]).contains(&lamp),
                "{lamp:?} set on a board it does not apply on"
            );
        }
        assert!(shown_lamps(&[], &[(0x05, EV_MOTOR_HALL)]).contains(&Lamp::Hall));
    }

    /// `FaultLampTest.kt`, `the lamps keep the grid's order`.
    #[test]
    fn the_lamps_keep_the_grids_order() {
        let want: Vec<Lamp, 10> = LAMPS
            .into_iter()
            .filter(|&l| l != Lamp::Latch2 && l != Lamp::Config)
            .collect();
        assert_eq!(shown_lamps(&two(Some(1)), &[]), want);
    }

    /// `FaultLampTest.kt`, `four columns, or five when nine or ten lamps are shown` (its
    /// ten-lamps-in-two-rows half; the handheld's grid is always five columns,
    /// `specs/handheld-remote.md` 5.6).
    #[test]
    fn ten_lamps_fit_two_rows_of_five() {
        let all = shown_lamps(&two(Some(1)), &[(0x01, EV_LATCH_B), (0x02, EV_CONFIG)]);
        assert_eq!(all.len(), 10);
        assert_eq!(grid_cell(0), (0, 0));
        assert_eq!(grid_cell(4), (0, 4));
        assert_eq!(grid_cell(5), (1, 0));
        assert_eq!(grid_cell(all.len() - 1), (1, 4));
    }

    /// `FaultLampTest.kt`, `every lamp bit is exactly one lamp with its sheet` (the bit mapping;
    /// the sheet strings and codes are not ported, the handheld has no sheets). The titles are
    /// `specs/remote-ui.md` 3.1b's table.
    #[test]
    fn every_lamp_bit_is_exactly_one_lamp() {
        assert_eq!(ARM_BLOCKING_BITS, 0x0F37);
        assert_eq!(LAMP_BITS, ARM_BLOCKING_BITS | EV_CONFIG);
        let bits: Vec<u32, 16> = (0..16).filter(|n| LAMP_BITS & (1 << n) != 0).collect();
        assert_eq!(bits.len(), LAMPS.len());
        for n in bits {
            let lamp = Lamp::of(1 << n).expect("a lamp for every lamp bit");
            assert_eq!(lamp.bit().trailing_zeros(), n);
            assert_eq!(LAMPS.iter().filter(|l| l.bit() == 1 << n).count(), 1);
        }
        assert_eq!(LAMPS.iter().fold(0, |acc, l| acc | l.bit()), LAMP_BITS);
        for n in [3, 6, 7, 13, 14, 15] {
            assert_eq!(Lamp::of(1 << n), None, "bit {n} is not a lamp");
        }
        assert_eq!(Lamp::Latch2.label(), "LATCH 2");
        assert_eq!(Lamp::Hall.title(), "Hall sensor fault");
        assert_eq!(Lamp::Tilt.title(), "Tilted past 45°");
        for (i, a) in LAMPS.iter().enumerate() {
            for b in &LAMPS[i + 1..] {
                assert_ne!(a.label(), b.label());
                assert_ne!(a.title(), b.title());
            }
        }
    }

    /// `FaultLampTest.kt`, `only the latches are held`.
    #[test]
    fn only_the_latches_are_held() {
        let held: Vec<Lamp, 10> = LAMPS.into_iter().filter(|l| l.held()).collect();
        assert_eq!(held.as_slice(), &[Lamp::Latch, Lamp::Latch2]);
    }

    /// `specs/remote-ui.md` 3.1b, the LATCH hold and "While a held lamp is the only lit lamp, the
    /// grid shows"; the app tests it through the view model (`MainViewModel.kt`, `onFaultWords`),
    /// with no model test to copy. The bits are `FaultLampTest.kt`'s.
    #[test]
    fn a_latch_is_held_until_the_next_boot_press() {
        let mut hold = LatchHold::new();
        hold.observe(0x01, EV_LATCH_A | EV_MOTOR_HALL);
        assert_eq!(hold.held(0x01), EV_LATCH_A, "only latch bits are held");
        assert_eq!(hold.held(0x02), 0, "per board");
        // The board clears its latch on the next frame: the lamp stays lit, and the grid shows.
        let words = [
            (0x01, hold.lamp_word(0x01, 0)),
            (0x02, hold.lamp_word(0x02, 0)),
        ];
        assert_eq!(words[0].1, EV_LATCH_A);
        assert!(lamp_on(Lamp::Latch, &words, false));
        assert!(mark_on(Lamp::Latch, 0x01, &words, false));
        assert!(!mark_on(Lamp::Latch, 0x02, &words, false));
        assert!(grid_shows(&words, false));
        hold.observe(0x02, EV_LATCH_B);
        assert_eq!(hold.held(0x02), EV_LATCH_B);
        hold.clear_on_boot_press();
        assert_eq!(hold, LatchHold::default());
        assert_eq!(
            hold.lamp_word(0x01, EV_CONFIG | (1 << 6)),
            EV_CONFIG,
            "live bits masked"
        );
    }

    /// `specs/remote-ui.md` 3.1b, "When the grid shows" and the lamp states (`MainViewModel.kt`,
    /// `gridShows`; `FaultGrid.kt`, `Lamp`); no model test to copy.
    #[test]
    fn when_the_grid_shows_and_what_is_lit() {
        assert!(!grid_shows(&[(0x01, 0)], false));
        assert!(!grid_shows(&[(0x01, EV_CONFIG)], false), "CONFIG alone");
        assert!(grid_shows(&[(0x01, EV_CONFIG)], true), "the lamp test");
        assert!(grid_shows(&[(0x02, EV_MOTOR_HALL)], false));
        let words = [(0x01, 0), (0x02, EV_MOTOR_HALL)];
        assert!(!lamp_on(Lamp::Cal, &words, false));
        assert!(
            lamp_on(Lamp::Cal, &words, true),
            "every lamp lit in the test"
        );
        assert!(
            mark_on(Lamp::Cal, 0x01, &words, true),
            "every mark lit in the test"
        );
        assert_eq!(
            Lamp::first_lit(EV_MOTOR_ARM | EV_STOP_ALL),
            Some(Lamp::Tilt)
        );
        assert_eq!(Lamp::first_lit(1 << 6), None);
    }
}
