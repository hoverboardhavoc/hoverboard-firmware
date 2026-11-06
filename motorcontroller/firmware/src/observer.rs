//! The flux observer's loop-side half (`specs/flux-observer.md`, "Design (S1)" and "Fields"): the
//! live copy of the two per-motor store fields the tune lane writes, the pre-scaling of them into
//! the ISR's parameters, and the bus scale the ISR applies to each period's battery count.
//!
//! The observer itself, its verdict and its table are `commutation::observer`'s and run in the
//! period ISR (`motor::hw::period_isr`). Nothing here runs there: every derivation is in the loop
//! pass, and the result is installed whole (`motor::hw::install_observer`).

// The loop that calls this is target-only; the host build reaches it through the tests.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use crate::motor;
use commutation::observer::{BusScale, ObserverParams, LEAK_SHIFT_MAX, LEAK_SHIFT_MIN, RATIO_ONE};
use control::TuneError;
use store::{OBSERVER_MODEL, OBSERVER_SHADOW};

/// `motors.<i>.observer.model`'s words per motor.
pub const MODEL_WORDS: usize = OBSERVER_MODEL.len();
/// `motors.<i>.observer.shadow`'s words per motor.
pub const SHADOW_WORDS: usize = OBSERVER_SHADOW.len();
/// One motor's words: the model's, then the shadow's.
pub const WORDS: usize = MODEL_WORDS + SHADOW_WORDS;

/// One motor's words as stored or live, the model's then the shadow's.
pub type Words = [i16; WORDS];

/// The words a store holds for every motor ([`read`]'s shape).
pub type Stored = [Words; motor::COUNT];

/// The range of each word, the tune lane's seam: a `TUNE_WRITE` outside it is refused, and a
/// stored value outside it is clamped on the way in. `None` is a word the tune lane does not take
/// (`flux_uvs` and the spare, which nothing reads in S1).
const RANGE: [Option<(i16, i16)>; WORDS] = [
    // model: r_mohm, l_uh, flux_uvs, spare
    Some((0, i16::MAX)),
    Some((0, i16::MAX)),
    None,
    None,
    // shadow: band, spread, min_windows, ratio, offset, leak_shift, dt_comp
    Some((0, i16::MAX)),
    Some((0, i16::MAX)),
    Some((1, i16::MAX)),
    Some((0, RATIO_ONE)),
    Some((i16::MIN, i16::MAX)),
    Some((LEAK_SHIFT_MIN, LEAK_SHIFT_MAX)),
    Some((0, 1)),
];

/// The stored words through [`RANGE`]: a word with a range is clamped into it, one without is kept.
fn clamped(w: Words) -> Words {
    let mut out = w;
    for (v, r) in out.iter_mut().zip(RANGE) {
        if let Some((lo, hi)) = r {
            *v = (*v).clamp(lo, hi);
        }
    }
    out
}

/// The word a tune key `(field_id, index)` names: the motor and the word's place in [`Words`], or
/// `None` for a key that names no word of a motor this image has. The index is `motor * n +
/// element` (`store::IndexedField::at_motor`).
fn slot(field_id: u8, index: u8) -> Option<(usize, usize)> {
    let (n, base) = if field_id == OBSERVER_MODEL.id() {
        (MODEL_WORDS, 0)
    } else if field_id == OBSERVER_SHADOW.id() {
        (SHADOW_WORDS, MODEL_WORDS)
    } else {
        return None;
    };
    let m = index as usize / n;
    (m < motor::COUNT).then_some((m, base + index as usize % n))
}

/// Read every motor's two fields from the store, unclamped (the shadow clamps). Out of line: the
/// boot and the persist path share one copy.
#[inline(never)]
pub fn read<F: store::Flash>(s: &store::Store<F>) -> Stored {
    core::array::from_fn(|m| {
        let m = m as u8;
        let mut w = [0i16; WORDS];
        for (e, v) in w.iter_mut().enumerate().take(MODEL_WORDS) {
            *v = s.get(OBSERVER_MODEL.at_motor(m, e as u8));
        }
        for e in 0..SHADOW_WORDS {
            w[MODEL_WORDS + e] = s.get(OBSERVER_SHADOW.at_motor(m, e as u8));
        }
        w
    })
}

/// The observer's live words and what the ISR's parameters are derived from beside them: the
/// `control::GainShadow` pattern for a per-motor field pair (`specs/remote-ui.md` section 4, the
/// live tune lane: a tune write changes the running value and touches no flash; a persisted write
/// re-reads the store, and moves only the words whose flash value changed).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObserverLoop {
    live: Stored,
    stored: Stored,
    /// Each motor's counts per amp (`motors.<i>.current.cal`), from the last arm-row read.
    current_cal: [u16; motor::COUNT],
    /// Each motor's dead time in gate-timer ticks (`motor::dead_time_ticks` of its boot-read
    /// `motors.<i>.gate.dead_time`).
    dead_ticks: [u32; motor::COUNT],
    /// The bus scale of this board's own battery sense (`battery_sense.cal`, boot-read), `None`
    /// on a board that does not sense ([`ObserverLoop::bus_scale`]).
    local: Option<BusScale>,
}

impl ObserverLoop {
    /// The loop's copy of a board whose store holds `flash`, before the arm row has given it the
    /// calibration (the registered default until then) and the bring-up its dead times.
    /// `local` is the board's battery-sense calibration (slope, offset), `None` on a board that
    /// does not sense.
    pub fn of_stored(flash: Stored, local: Option<(i16, i16)>) -> Self {
        let c = flash.map(clamped);
        ObserverLoop {
            live: c,
            stored: c,
            current_cal: [store::MOTOR_CURRENT_CAL.default(); motor::COUNT],
            dead_ticks: [0; motor::COUNT],
            local: local.map(|(s, o)| BusScale::local(s, o)),
        }
    }

    /// The bus scale the loop publishes for the ISR (`motor::OBS_BUS`, `specs/flux-observer.md`,
    /// "Voltage sensing"): a board that senses its own battery, once its local word has primed,
    /// the period's own count through its calibration; a board that does not, the effective
    /// battery word, which is the master's over the link and 0 while that is stale or unknown,
    /// which suspends the observer.
    pub fn bus_scale(&self, local_word: i16, effective_word: i16) -> BusScale {
        match self.local {
            Some(s) if local_word > 0 => s,
            Some(_) => BusScale::NONE,
            None => BusScale::word(effective_word),
        }
    }

    /// A tune key's live value (`TUNE_READ`), or `None` for a key the observer does not own.
    pub fn get(&self, field_id: u8, index: u8) -> Option<i16> {
        let (m, w) = slot(field_id, index)?;
        RANGE[w].map(|_| self.live[m][w])
    }

    /// Whether the observer owns `field_id` on the tune lane.
    pub fn owns(field_id: u8) -> bool {
        field_id == OBSERVER_MODEL.id() || field_id == OBSERVER_SHADOW.id()
    }

    /// Set one word live (`TUNE_WRITE`): refused outside its range or for a word the lane does not
    /// take. The caller installs the parameters after.
    pub fn set(&mut self, field_id: u8, index: u8, value: i16) -> Result<(), TuneError> {
        let (m, w) = slot(field_id, index).ok_or(TuneError::UnknownKey)?;
        let (lo, hi) = RANGE[w].ok_or(TuneError::UnknownKey)?;
        if value < lo || value > hi {
            return Err(TuneError::OutOfRange);
        }
        self.live[m][w] = value;
        Ok(())
    }

    /// A fresh store read after a persisted write: each word whose FLASH value moved since the last
    /// read follows it, and a word the tune lane holds live keeps its value otherwise. Returns
    /// whether any live word changed.
    pub fn reconcile(&mut self, flash: Stored) -> bool {
        let mut changed = false;
        for (m, words) in flash.map(clamped).iter().enumerate() {
            for (w, fresh) in words.iter().enumerate() {
                if *fresh != self.stored[m][w] {
                    self.stored[m][w] = *fresh;
                    changed |= self.live[m][w] != *fresh;
                    self.live[m][w] = *fresh;
                }
            }
        }
        changed
    }

    /// The arm row's calibrations and the bring-up's dead-time codes (`None` for a motor with no
    /// runtime, which keeps what it had).
    pub fn set_motor_facts(
        &mut self,
        current_cal: [u16; motor::COUNT],
        dead_time: [Option<u8>; motor::COUNT],
    ) {
        self.current_cal = current_cal;
        for (t, d) in self.dead_ticks.iter_mut().zip(dead_time) {
            if let Some(d) = d {
                *t = motor::dead_time_ticks(d);
            }
        }
    }

    /// The ISR's parameters for every motor, from the live words.
    pub fn params(&self) -> [ObserverParams; motor::COUNT] {
        core::array::from_fn(|m| {
            let w = &self.live[m];
            let mut model = [0i16; MODEL_WORDS];
            model.copy_from_slice(&w[..MODEL_WORDS]);
            let mut shadow = [0i16; SHADOW_WORDS];
            shadow.copy_from_slice(&w[MODEL_WORDS..]);
            ObserverParams::derive(model, shadow, self.current_cal[m], self.dead_ticks[m])
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn defaults() -> Words {
        let mut w = [0i16; WORDS];
        w[..MODEL_WORDS].copy_from_slice(&OBSERVER_MODEL.defaults());
        w[MODEL_WORDS..].copy_from_slice(&OBSERVER_SHADOW.defaults());
        w
    }

    /// The tune keys: per motor and element, the model's two tunable words and all seven of the
    /// shadow's; the model's last two, another field, and a motor this image lacks are unknown.
    #[test]
    fn the_tune_keys_are_the_spec_words_per_motor() {
        let mut o = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        let model = OBSERVER_MODEL.id();
        let shadow = OBSERVER_SHADOW.id();
        assert_eq!(o.get(model, 0), Some(240));
        assert_eq!(o.get(model, 1), Some(500));
        assert_eq!(o.get(model, 2), None);
        assert_eq!(o.get(model, 3), None);
        assert_eq!(o.get(shadow, 5), Some(10));
        assert_eq!(o.get(0x27, 0), None);
        assert_eq!(o.set(model, 2, 1), Err(TuneError::UnknownKey));
        assert_eq!(o.set(0x27, 0, 1), Err(TuneError::UnknownKey));
        assert_eq!(
            o.set(shadow, (motor::COUNT * SHADOW_WORDS) as u8, 1),
            Err(TuneError::UnknownKey)
        );
        // Motor m's element e is index m * n + e.
        let last = (motor::COUNT - 1) as u8;
        assert_eq!(o.set(shadow, last * 7 + 4, -300), Ok(()));
        assert_eq!(o.get(shadow, last * 7 + 4), Some(-300));
        assert!(ObserverLoop::owns(model) && ObserverLoop::owns(shadow));
        assert!(!ObserverLoop::owns(0x27));
    }

    /// Each word's range: refused outside on the lane, clamped from the store.
    #[test]
    fn the_ranges_refuse_on_the_lane_and_clamp_from_the_store() {
        let mut o = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        let shadow = OBSERVER_SHADOW.id();
        for (e, bad) in [(5u8, 5i16), (5, 15), (6, 2), (3, 257), (2, 0), (0, -1)] {
            assert_eq!(
                o.set(shadow, e, bad),
                Err(TuneError::OutOfRange),
                "{e} {bad}"
            );
        }
        assert_eq!(
            o.set(OBSERVER_MODEL.id(), 0, -1),
            Err(TuneError::OutOfRange)
        );
        for (e, good) in [(5u8, 6i16), (5, 14), (6, 0), (3, 256), (2, 1)] {
            assert_eq!(o.set(shadow, e, good), Ok(()), "{e} {good}");
        }
        let mut w = defaults();
        w[MODEL_WORDS + 5] = 40;
        w[MODEL_WORDS + 3] = 1000;
        w[0] = -7;
        let o = ObserverLoop::of_stored([w; motor::COUNT], None);
        assert_eq!(o.get(shadow, 5), Some(14));
        assert_eq!(o.get(shadow, 3), Some(256));
        assert_eq!(o.get(OBSERVER_MODEL.id(), 0), Some(0));
    }

    /// A persisted write moves the words whose flash value changed and leaves a live tune of the
    /// others alone.
    #[test]
    fn a_persist_moves_only_the_words_flash_moved() {
        let mut o = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        let shadow = OBSERVER_SHADOW.id();
        o.set(shadow, 0, 1000).unwrap();
        let mut f = [defaults(); motor::COUNT];
        f[0][MODEL_WORDS + 1] = 900;
        assert!(o.reconcile(f));
        assert_eq!(o.get(shadow, 0), Some(1000), "the live tune survives");
        assert_eq!(
            o.get(shadow, 1),
            Some(900),
            "the persisted word follows flash"
        );
        assert!(!o.reconcile(f), "nothing moved the second time");
    }

    /// The parameters come out of the live words through the commutation crate's derivation, at
    /// the arm row's calibration and the bring-up's dead time.
    #[test]
    fn the_parameters_are_the_live_words_derived() {
        let mut o = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        o.set_motor_facts([160; motor::COUNT], [Some(28); motor::COUNT]);
        o.set(OBSERVER_SHADOW.id(), 5, 12).unwrap();
        let p = o.params()[0];
        let want = ObserverParams::derive(
            [240, 500, 0, 0],
            [2730, 1820, 64, 243, 0, 12, 1],
            160,
            motor::dead_time_ticks(28),
        );
        assert_eq!(p, want);
        assert_eq!(motor::dead_time_ticks(28), 56);
    }

    /// The store read: an empty store reads the registered defaults for every motor, and a motor's
    /// stored element lands at its place in [`Words`] and nowhere else.
    #[test]
    fn the_read_takes_each_motors_elements_by_key() {
        let mut flash = crate::test_flash::TestFlash::erased();
        let mut s = store::Store::mount(&mut flash).unwrap();
        assert_eq!(read(&s), [defaults(); motor::COUNT]);
        let last = (motor::COUNT - 1) as u8;
        s.set(OBSERVER_SHADOW.at_motor(last, 4), -1234).unwrap();
        s.set(OBSERVER_MODEL.at_motor(last, 1), 777).unwrap();
        let r = read(&s);
        let mut want = defaults();
        want[MODEL_WORDS + 4] = -1234;
        want[1] = 777;
        assert_eq!(r[last as usize], want);
    }

    /// The DTG decode over its four ranges (`specs/commutation.md`).
    #[test]
    fn the_dead_time_code_decodes_into_timer_ticks() {
        let div = board::DEAD_TIME_DTS_DIV;
        assert_eq!(motor::dead_time_ticks(0), 0);
        assert_eq!(motor::dead_time_ticks(127), 127 * div);
        assert_eq!(motor::dead_time_ticks(0x80), 128 * div);
        assert_eq!(motor::dead_time_ticks(0xBF), (64 + 63) * 2 * div);
        assert_eq!(motor::dead_time_ticks(0xC0), 32 * 8 * div);
        assert_eq!(motor::dead_time_ticks(0xDF), 63 * 8 * div);
        assert_eq!(motor::dead_time_ticks(0xE0), 32 * 16 * div);
        assert_eq!(motor::dead_time_ticks(0xFF), 63 * 16 * div);
    }

    /// The bus scale's source rule: a sensing board's own count once primed, nothing before; a
    /// board that borrows, the effective word, nothing while it is unknown.
    #[test]
    fn the_bus_scale_takes_the_local_count_or_the_borrowed_word() {
        let sensing = ObserverLoop::of_stored([defaults(); motor::COUNT], Some((25_200, -5)));
        assert_eq!(sensing.bus_scale(3100, 0), BusScale::local(25_200, -5));
        assert_eq!(sensing.bus_scale(0, 3100), BusScale::NONE);
        let borrowing = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        assert_eq!(borrowing.bus_scale(0, 3100), BusScale::word(3100));
        assert_eq!(borrowing.bus_scale(0, 0), BusScale::NONE);
    }
}
