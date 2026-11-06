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
use commutation::observer::{
    self as obs, BusScale, ObserverParams, BINS, LEAK_SHIFT_MAX, LEAK_SHIFT_MIN, RATIO_ONE,
};
use commutation::Direction;
use control::TuneError;
use linkctl::FocTrace;
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
/// (`flux_uvs`, which selects the observer's form and is written with `board set`, and the
/// spare).
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

/// `m` as an index into the per-motor arrays: itself when this image has the motor, else motor 0.
fn motor_index(m: usize) -> usize {
    if m < motor::COUNT {
        m
    } else {
        0
    }
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
    /// Each motor's drive direction (`motors.<i>.direction`, from the last arm-row read): the
    /// observer's quarter turn (`commutation::observer::ObserverParams::quarter`).
    direction: [Direction; motor::COUNT],
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
            direction: [Direction::Forward; motor::COUNT],
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

    /// The arm row's calibrations and directions, and the bring-up's dead-time codes (`None` for
    /// a motor with no runtime, which keeps what it had).
    pub fn set_motor_facts(
        &mut self,
        current_cal: [u16; motor::COUNT],
        dead_time: [Option<u8>; motor::COUNT],
        direction: [Direction; motor::COUNT],
    ) {
        self.current_cal = current_cal;
        self.direction = direction;
        for (t, d) in self.dead_ticks.iter_mut().zip(dead_time) {
            if let Some(d) = d {
                *t = motor::dead_time_ticks(d);
            }
        }
    }

    /// Motor `m`'s `min_windows`, the bin-measured threshold the `FOC_TRACE` bytes are taken at.
    pub fn min_windows(&self, m: usize) -> u16 {
        self.live[motor_index(m)][MODEL_WORDS + obs::SHADOW_MIN_WINDOWS].max(1) as u16
    }

    /// The ISR's parameters for every motor, from the live words.
    pub fn params(&self) -> [ObserverParams; motor::COUNT] {
        core::array::from_fn(|m| {
            let w = &self.live[m];
            let mut model = [0i16; MODEL_WORDS];
            model.copy_from_slice(&w[..MODEL_WORDS]);
            let mut shadow = [0i16; SHADOW_WORDS];
            shadow.copy_from_slice(&w[MODEL_WORDS..]);
            ObserverParams::derive(
                model,
                shadow,
                self.current_cal[m],
                self.dead_ticks[m],
                self.direction[m],
            )
        })
    }
}

/// The words of one motor a `FOC_TRACE` frame is packed from, as the period ISR published them
/// (`specs/link-control.md`, "`FOC_TRACE` (0x15, 46 B)"): `CTRL_OBS` words 48 to 50 and 52 to 54
/// of that motor, its speed, fault and state words, and the period counter.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TraceWords {
    /// [`motor::OBS_FOC_DQ`]: `id` low, `iq` high.
    pub dq: u32,
    /// [`motor::OBS_FOC_REF`]: `iq_ref` low, `vq` high.
    pub ref_vq: u32,
    /// [`motor::OBS_FOC_HOLD`].
    pub hold: u32,
    /// [`motor::OBS_FOC_ANGLES`].
    pub angles: u32,
    /// [`motor::OBS_FOC_TRACK`].
    pub track: u32,
    /// [`motor::OBS_FOC_FLUX`].
    pub flux: u32,
    /// [`motor::SPEED`].
    pub speed: i32,
    /// [`motor::FAULT`].
    pub fault: u32,
    /// [`motor::OBS_STATE`].
    pub state: u32,
    /// [`motor::PERIODS`].
    pub periods: u32,
}

/// Motor `m`'s words, loaded.
pub fn trace_words(m: usize) -> TraceWords {
    use core::sync::atomic::Ordering::Relaxed;
    let m = motor_index(m);
    TraceWords {
        dq: motor::OBS_FOC_DQ[m].load(Relaxed),
        ref_vq: motor::OBS_FOC_REF[m].load(Relaxed),
        hold: motor::OBS_FOC_HOLD[m].load(Relaxed),
        angles: motor::OBS_FOC_ANGLES[m].load(Relaxed),
        track: motor::OBS_FOC_TRACK[m].load(Relaxed),
        flux: motor::OBS_FOC_FLUX[m].load(Relaxed),
        speed: motor::SPEED[m].load(Relaxed),
        fault: motor::FAULT[m].load(Relaxed),
        state: motor::OBS_STATE[m].load(Relaxed),
        periods: motor::PERIODS[m].load(Relaxed),
    }
}

/// The window counter the trace's cadence runs on: the 64-period closes so far.
pub fn trace_window(periods: u32) -> u16 {
    (periods >> motor::PERIODS_PER_TICK_SHIFT) as u16
}

/// One `FOC_TRACE` frame for `motor` from its words and its tracking table's bins (`bins[k]`,
/// seen low and tracked high), the bytes taken at `min_windows`.
pub fn pack_trace(w: &TraceWords, bins: &[u32; BINS], min_windows: u16, motor: u8) -> FocTrace {
    let pair = |x: u32| (x as u16 as i16, (x >> 16) as u16 as i16);
    let (id, iq) = pair(w.dq);
    let (iq_ref, vq) = pair(w.ref_vq);
    let (err_mean, err_spread) = pair(w.track);
    let hi = (w.flux >> 16) as u16;
    let flag = |on: bool, bit: u8| if on { bit } else { 0 };
    FocTrace {
        window: trace_window(w.periods),
        id,
        iq,
        iq_ref,
        vq,
        hall_angle: w.angles as u16,
        obs_angle: (w.angles >> 16) as u16,
        err_mean,
        err_spread,
        flux_mag: w.flux as u16,
        speed: motor::wheel_speed_word(w.speed),
        held_a: w.hold as u8,
        held_b: (w.hold >> 8) as u8,
        coasted: (w.hold >> 16) as u8,
        cause: (w.hold >> 24) as u8,
        flags: flag(
            w.fault & motor::FAULT_HALL_DECODED != 0,
            FocTrace::FLAG_HALL_FAULT,
        ) | flag(hi & obs::TRACK_TRACKED != 0, FocTrace::FLAG_TRACKED)
            | flag(hi & obs::TRACK_SOURCE != 0, FocTrace::FLAG_OBSERVER_SOURCE)
            | flag(hi & obs::TRACK_IN_WINDOW != 0, FocTrace::FLAG_IN_WINDOW),
        method: (w.state >> 16) as u8,
        lock_bin: hi as u8,
        motor,
        track: bins.map(|b| obs::track_byte(b, min_windows)),
    }
}

/// Motor `m`'s frame, from what the period ISR published.
pub fn trace_frame(m: usize, min_windows: u16) -> FocTrace {
    let table = &motor::OBS_TRACK_TABLE[motor_index(m)];
    let bins = core::array::from_fn(|k| table.word(k));
    pack_trace(&trace_words(m), &bins, min_windows, m as u8)
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

    /// The source button's proof (`specs/flux-observer.md`, "S4, first slice", "The proof that it
    /// is the observer driving"): `motors.<i>.observer.shadow[4]`, the offset, takes a tune-lane
    /// write and reaches the parameters the ISR's source angle subtracts, on each motor's own key.
    #[test]
    fn the_offset_is_a_live_word_the_source_angle_reads() {
        let mut o = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        for m in 0..motor::COUNT {
            let key = OBSERVER_SHADOW.at_motor(m as u8, 4).key().index;
            o.set(OBSERVER_SHADOW.id(), key, 5_461).unwrap();
            assert_eq!(o.params()[m].offset, 5_461, "motor {m}");
            o.set(OBSERVER_SHADOW.id(), key, 16_384).unwrap();
            assert_eq!(o.params()[m].offset, 16_384);
            o.set(OBSERVER_SHADOW.id(), key, 0).unwrap();
            assert_eq!(o.params()[m].offset, 0);
        }
    }

    /// The parameters come out of the live words through the commutation crate's derivation, at
    /// the arm row's calibration and the bring-up's dead time.
    #[test]
    fn the_parameters_are_the_live_words_derived() {
        let mut o = ObserverLoop::of_stored([defaults(); motor::COUNT], None);
        o.set_motor_facts(
            [160; motor::COUNT],
            [Some(28); motor::COUNT],
            [Direction::Reverse; motor::COUNT],
        );
        o.set(OBSERVER_SHADOW.id(), 5, 12).unwrap();
        let p = o.params()[0];
        let want = ObserverParams::derive(
            [240, 500, 0, 0],
            [2730, 1820, 64, 243, 0, 12, 1],
            160,
            motor::dead_time_ticks(28),
            Direction::Reverse,
        );
        assert_eq!(p, want);
        assert_eq!(p.quarter, commutation::foc::ANGLE_90);
        o.set_motor_facts(
            [160; motor::COUNT],
            [None; motor::COUNT],
            [Direction::Forward; motor::COUNT],
        );
        assert_eq!(
            o.params()[0].quarter,
            0u16.wrapping_sub(commutation::foc::ANGLE_90)
        );
        assert_eq!(motor::dead_time_ticks(28), 56);
        assert_eq!(o.min_windows(0), 64);
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

    /// A frame packs the published words field for field (`specs/link-control.md`, "`FOC_TRACE`"),
    /// the flags from the fault word and word 54's high half, and the bins at `min_windows`.
    #[test]
    fn a_trace_frame_packs_the_published_words() {
        let w = TraceWords {
            dq: motor::pack_i16_pair(-2, 300),
            ref_vq: motor::pack_i16_pair(0x0102, -31_999),
            hold: motor::pack_foc_hold([18, 27], 5, 0x2A << 24),
            angles: 0xA0B0_ABCD,
            track: motor::pack_i16_pair(-273, 182),
            flux: (((4 | obs::TRACK_TRACKED | obs::TRACK_IN_WINDOW) as u32) << 16) | 21_000,
            speed: -7,
            fault: motor::FAULT_HALL | motor::FAULT_HALL_DECODED,
            state: motor::pack_obs_state(3, [true; 3], 2, 0),
            periods: 0x1234 << 6 | 17,
        };
        let mut bins = [0u32; BINS];
        bins[4] = 100 | 50 << 16;
        bins[5] = 63 | 63 << 16;
        let f = pack_trace(&w, &bins, 64, 1);
        assert_eq!(
            (f.window, f.id, f.iq, f.iq_ref, f.vq),
            (0x1234, -2, 300, 0x0102, -31_999)
        );
        assert_eq!(
            (
                f.hall_angle,
                f.obs_angle,
                f.err_mean,
                f.err_spread,
                f.flux_mag
            ),
            (0xABCD, 0xA0B0, -273, 182, 21_000)
        );
        assert_eq!(
            (f.speed, f.held_a, f.held_b, f.coasted, f.cause),
            (-7, 18, 27, 5, 0x2A)
        );
        assert_eq!(
            f.flags,
            FocTrace::FLAG_HALL_FAULT | FocTrace::FLAG_TRACKED | FocTrace::FLAG_IN_WINDOW
        );
        assert_eq!((f.method, f.lock_bin, f.motor), (2, 4, 1));
        assert_eq!(f.track[4], 125);
        assert_eq!(f.track[5], 255, "63 windows is not measured at 64");
        assert_eq!(f.track[0], 255);
        // A speed past i16 saturates, the word 24 narrowing.
        let f = pack_trace(
            &TraceWords {
                speed: 1 << 20,
                ..w
            },
            &bins,
            64,
            0,
        );
        assert_eq!(f.speed, i16::MAX);
    }

    /// The frame's `observer_source` column follows word 54's bit 9, and its `hall_fault` column
    /// is the front end's decoded fault, which the latch masks while the observer drives
    /// (`specs/flux-observer.md`, "S4, first slice").
    #[test]
    fn the_frame_says_which_angle_drove_and_the_decoded_hall_fault() {
        let w = TraceWords {
            dq: 0,
            ref_vq: 0,
            hold: 0,
            angles: 0,
            track: 0,
            flux: (((obs::LOCK_NONE as u16 | obs::TRACK_SOURCE) as u32) << 16) | 21_000,
            speed: 0,
            fault: motor::hall_fault_bits(true, true),
            state: motor::pack_obs_state(0, [true; 3], 2, 0),
            periods: 0,
        };
        let bins = [0u32; BINS];
        let f = pack_trace(&w, &bins, 64, 0);
        assert_eq!(
            f.flags,
            FocTrace::FLAG_OBSERVER_SOURCE | FocTrace::FLAG_HALL_FAULT,
            "the source drove and the halls are out, with the latch clear"
        );
        assert_eq!(motor::hall_fault_bits(true, true) & motor::FAULT_HALL, 0);
        let f = pack_trace(
            &TraceWords {
                flux: 21_000,
                fault: 0,
                ..w
            },
            &bins,
            64,
            0,
        );
        assert_eq!(f.flags, 0, "the halls drove, no fault");
    }
}
