//! The flux observer in shadow (`specs/flux-observer.md`, "Design (S1)"): a leaky voltage-model
//! flux integrator that runs on every FOC period beside the hall angle and drives nothing, the
//! per-window verdict that says whether it agreed with the halls, and the per-speed-bin table that
//! says from what speed it did.
//!
//! Everything here is pure: the period ISR (`firmware::motor`) hands in the applied voltage vector
//! ([`crate::foc::FocApplied`]), the conditioned current pair, the bus scale and the front end's
//! facts, and publishes what [`FluxObserver::close`] returns. The parameters arrive pre-scaled
//! ([`ObserverParams::derive`], run in the loop pass, never in the ISR), so the per-period work is
//! multiplies, shifts and one [`atan2_u16`].
//!
//! # Units
//!
//! - Voltage: the applied vector is in voltage counts, one count `V_bus / (sqrt 3 * 32767)`; the
//!   observer converts to millivolts with `kv`, `v_mV = v * kv >> 16`, `kv = V_bus_mV * 65536 /`
//!   [`KV_DEN`] ([`bus_kv`]).
//! - Flux: `i32` at 1/16 microvolt-second per LSB, so one millivolt applied for one 62.5 us period
//!   is one LSB, and the per-period increment is `v_mV - R i` in millivolts. The magnitude and the
//!   angle are taken in microvolt-seconds (`>> 4`).
//! - Angle: `u16`, 65536 per electrical revolution, the front end's unit.
//!
//! # The frame
//!
//! The crate's Clarke and Park work in the stock frame, whose beta is the negative of the textbook
//! beta (`foc.rs`, the frame note), and `theta` is the textbook angle of the q axis: a `vq` alone
//! drives the stator vector at `theta`. The observer integrates in the stock frame, reads the flux
//! angle in the textbook frame (`atan2(-beta, alpha)`), and moves a quarter turn to the q axis,
//! because the permanent-magnet flux lies on d. Which way is the motor's direction
//! ([`ObserverParams::quarter`], the bench finding of 2026-11-03, `specs/flux-observer.md`, "The
//! frame"): on a `Reverse` motor the flux is a quarter turn behind `theta`, on a `Forward` one a
//! quarter turn ahead, because the reverse sequence mirrors the front end's angle against the
//! stator frame. So with the halls and the alignment right, the observer's angle equals `theta`
//! plus the leak's phase lead.

use crate::foc::{atan2_u16, clarke, isqrt_u64, ANGLE_90, SPEED_WINDOW};
use crate::{Direction, ARR};
use core::sync::atomic::{AtomicU32, Ordering};

/// The tracking table's speed bins: one per edge of the front end's latched speed (edges per
/// 320-period window), the last holding 15 and above (`specs/flux-observer.md`, "Per speed bin").
pub const BINS: usize = 16;

/// The lock bin's "none" value.
pub const LOCK_NONE: u8 = 255;

/// The full-scale voltage count times `sqrt 3`, the denominator of [`bus_kv`]'s scale: one voltage
/// count is `V_bus / 56754` (`specs/foc-torque.md` section 5, `0.577 V_bus / 32767`).
pub const KV_DEN: i64 = 56_754;

/// The largest `kv` (65,535, a 56.7 V bus): `v * kv` must fit an `i32` for every `i16` `v`.
pub const KV_MAX: i32 = 65_535;

/// The leak shift's range (`motors.<i>.observer.shadow` index 5).
pub const LEAK_SHIFT_MIN: i16 = 6;
/// See [`LEAK_SHIFT_MIN`].
pub const LEAK_SHIFT_MAX: i16 = 14;

/// The ratio's unit: `ratio` passes a bin tracked in `ratio / 256` of its windows.
pub const RATIO_ONE: i16 = 256;

/// The window's length the observer's mean assumes when every period was integrated, the FOC
/// window (`crate::foc::FOC_WINDOW_SHIFT`).
pub const WINDOW: u8 = 1 << crate::foc::FOC_WINDOW_SHIFT;

// ============================================================================================
// The bus scale
// ============================================================================================

/// How the period ISR turns this period's battery count into `kv`: `kv = (raw12 * slope >> 8) +
/// 2 * offset`, clamped to `0..=`[`KV_MAX`], and 0 suspends the observer. Packed into one `u32` so
/// the 250 Hz loop publishes both halves in one store (`slope` low, `offset` high).
///
/// - A board that senses its own battery (`battery_sense.pin` stored, the master) publishes its
///   calibration's affine form, so the bus voltage is the period's own conversion, not the 64 ms
///   filtered word (`specs/flux-observer.md`, "Voltage sensing").
/// - A board that takes the peer's word (the slave) publishes `slope` 0 and the word as the
///   offset, refreshed per tick, and 0 while the word is stale or unknown, which suspends the
///   observer rather than integrating a frozen voltage.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BusScale {
    /// `kv` per 12-bit count, Q8.
    pub slope: u16,
    /// Half of `kv` at a count of zero.
    pub offset: i16,
}

impl BusScale {
    /// The suspended scale: `kv` 0 whatever the count.
    pub const NONE: BusScale = BusScale {
        slope: 0,
        offset: 0,
    };

    /// The scale of a local sense whose calibration is `centivolts = raw12 * cal_slope / 10_000 +
    /// cal_offset` (`battery_sense.cal`, microvolts per count and centivolts).
    pub fn local(cal_slope: i16, cal_offset: i16) -> BusScale {
        // kv per centivolt is 10 * 65536 / KV_DEN = 11.5474; per count that times cal_slope /
        // 10_000, in Q8: 0.295613 per microvolt of slope. The offset's half: 5.77369 per
        // centivolt. Both in 32-bit arithmetic over the whole `i16` range.
        let s = cal_slope.max(0) as u32 * 29_561 / 100_000;
        let o = cal_offset as i32 * 57_737 / 10_000;
        BusScale {
            slope: s as u16,
            offset: o.clamp(i16::MIN as i32, i16::MAX as i32) as i16,
        }
    }

    /// The scale of a battery word taken from elsewhere (centivolts, 0 = unknown, which suspends).
    pub fn word(centivolts: i16) -> BusScale {
        let kv = centivolts.max(0) as u32 * 115_474 / 10_000;
        BusScale {
            slope: 0,
            offset: (kv.min(KV_MAX as u32) / 2) as i16,
        }
    }

    /// The packed word ([`BusScale::unpack`]'s inverse).
    pub const fn pack(self) -> u32 {
        self.slope as u32 | ((self.offset as u16 as u32) << 16)
    }

    /// The scale a packed word carries.
    pub const fn unpack(w: u32) -> BusScale {
        BusScale {
            slope: w as u16,
            offset: (w >> 16) as u16 as i16,
        }
    }
}

/// `kv` for this period: the packed [`BusScale`] `w` applied to the 12-bit battery count `raw12`,
/// clamped to `0..=`[`KV_MAX`]. 0 suspends the observer.
#[inline(always)]
pub fn bus_kv(raw12: u32, w: u32) -> i32 {
    let s = BusScale::unpack(w);
    let kv = (((raw12 & 0xFFF) * s.slope as u32) >> 8) as i32 + 2 * s.offset as i32;
    kv.clamp(0, KV_MAX)
}

// ============================================================================================
// The parameters
// ============================================================================================

/// The observer's per-period constants, pre-scaled in the loop pass ([`ObserverParams::derive`])
/// and installed into the ISR's record whole.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObserverParams {
    /// `R / current_cal` in millivolts per current count, Q16: the resistive drop is
    /// `i * r_q >> 16`.
    pub r_q: i32,
    /// `16 L / current_cal` in flux LSB per current count, Q16: the inductive flux is
    /// `i * l_q >> 16`.
    pub l_q: i32,
    /// The dead time over the period against the voltage scale, Q16: the per-phase drop is
    /// `kv * dt_q >> 16` millivolts.
    pub dt_q: i32,
    /// The leak, `psi -= psi >> leak_shift` a period.
    pub leak_shift: u8,
    /// Whether the dead-time compensation runs.
    pub dt_comp: bool,
    /// The tracked window's largest `|mean|`, angle units.
    pub band: u16,
    /// The tracked window's largest spread, angle units.
    pub spread: u16,
    /// Subtracted from every period's error (the halls' frame against the observer's).
    pub offset: u16,
    /// A bin is measured at this many windows seen.
    pub min_windows: u16,
    /// A measured bin passes at `tracked * 256 >= ratio * seen`.
    pub ratio: u16,
    /// The leak's phase lead at each bin's speed, angle units (bin 0 none).
    pub lead: [u16; BINS],
    /// From the flux angle to `theta`: a quarter turn forward on a `Reverse` motor (the flux is
    /// behind q), a quarter turn back on a `Forward` one (the flux is ahead), the frame note.
    pub quarter: u16,
}

/// `motors.<i>.observer.model`'s indices.
pub const MODEL_R_MOHM: usize = 0;
/// See [`MODEL_R_MOHM`].
pub const MODEL_L_UH: usize = 1;
/// `motors.<i>.observer.shadow`'s indices.
pub const SHADOW_BAND: usize = 0;
/// See [`SHADOW_BAND`].
pub const SHADOW_SPREAD: usize = 1;
/// See [`SHADOW_BAND`].
pub const SHADOW_MIN_WINDOWS: usize = 2;
/// See [`SHADOW_BAND`].
pub const SHADOW_RATIO: usize = 3;
/// See [`SHADOW_BAND`].
pub const SHADOW_OFFSET: usize = 4;
/// See [`SHADOW_BAND`].
pub const SHADOW_LEAK_SHIFT: usize = 5;
/// See [`SHADOW_BAND`].
pub const SHADOW_DT_COMP: usize = 6;

impl ObserverParams {
    /// Pre-scale the stored words for the ISR: `model` and `shadow` are the two fields' words for
    /// one motor (`motors.<i>.observer.model`, `.shadow`), `current_cal` the motor's counts per amp
    /// (`motors.<i>.current.cal`), `dead_ticks` its dead time in timer ticks, `direction` the
    /// motor's drive direction (`motors.<i>.direction`, the arm's decode), which sets
    /// [`ObserverParams::quarter`].
    ///
    /// Every word is clamped into its range here, so a hand-poked flash value cannot reach the
    /// ISR: negative `R` and `L` read as 0, the leak shift as 6..=14, the ratio as 0..=256, a
    /// minimum window count as at least 1, a calibration as at least 1.
    pub fn derive(
        model: [i16; 4],
        shadow: [i16; 7],
        current_cal: u16,
        dead_ticks: u32,
        direction: Direction,
    ) -> ObserverParams {
        // 32-bit arithmetic throughout: an `i16` times 65,536 fits a `u32`, and so does the
        // voltage scale times the longest dead time a DTG code encodes.
        let cal = current_cal.max(1) as u32;
        let leak = shadow[SHADOW_LEAK_SHIFT].clamp(LEAK_SHIFT_MIN, LEAK_SHIFT_MAX) as u32;
        let r_q = model[MODEL_R_MOHM].max(0) as u32 * 65_536 / cal;
        let l_q = (model[MODEL_L_UH].max(0) as u32 * 65_536 / cal).saturating_mul(16);
        let dt_q = KV_DEN as u32 * dead_ticks.min(u16::MAX as u32) / (2 * ARR as u32);
        ObserverParams {
            r_q: r_q.min(i32::MAX as u32) as i32,
            l_q: l_q.min(i32::MAX as u32) as i32,
            dt_q: dt_q as i32,
            leak_shift: leak as u8,
            dt_comp: shadow[SHADOW_DT_COMP] != 0,
            band: shadow[SHADOW_BAND].max(0) as u16,
            spread: shadow[SHADOW_SPREAD].max(0) as u16,
            offset: shadow[SHADOW_OFFSET] as u16,
            min_windows: shadow[SHADOW_MIN_WINDOWS].max(1) as u16,
            ratio: shadow[SHADOW_RATIO].clamp(0, RATIO_ONE) as u16,
            lead: lead_table(leak),
            quarter: match direction {
                Direction::Reverse => ANGLE_90,
                Direction::Forward => 0u16.wrapping_sub(ANGLE_90),
            },
        }
    }
}

/// The leak's phase lead at each bin's speed, `atan(f_c / f)` in angle units: the leak is a
/// high-pass at `f_pwm / (2 pi 2^shift)` and bin `k` is `k` edges a 320-period window, `k f_pwm /
/// 1920` Hz electrical, so `f_c / f = 1920 / (2 pi 2^shift k)` whatever the period rate. Bin 0
/// (rest) has no entry.
pub fn lead_table(leak_shift: u32) -> [u16; BINS] {
    // 6 edges a revolution times the window, over 2 pi, scaled by 1000.
    const Y: i32 = (6 * SPEED_WINDOW as i64 * 1_000_000_000 / 6_283_185) as i32;
    let mut t = [0u16; BINS];
    // A trip count the optimiser cannot see, so the fifteen `atan2` calls stay one loop rather
    // than unrolling into the two-motor image's last few hundred bytes of flash; `get_mut` keeps
    // the loop free of a bounds panic all the same. Cold: run on a parameter change only.
    let n = core::hint::black_box(BINS);
    let shift = leak_shift.min(LEAK_SHIFT_MAX as u32);
    for k in 1..n {
        if let Some(e) = t.get_mut(k) {
            *e = atan2_u16(Y, ((1000 * k as u32) << shift) as i32);
        }
    }
    t
}

// ============================================================================================
// The observer and its window
// ============================================================================================

/// One period's inputs.
#[derive(Clone, Copy, Debug)]
pub struct ObserverInput {
    /// The voltage vector the LAST period computed (`FocApplied` before this period's step),
    /// voltage counts, stock frame. Its duties took effect at the end of the last period, so it
    /// covers the second half of the interval since the last sample, and the observer keeps the
    /// one before it for the first half.
    pub v: (i16, i16),
    /// The conditioned phase currents `(ia, ib)` the arm was handed: the pair after the sample
    /// hold (`specs/flux-observer.md`, "Which currents on a held period").
    pub i: (i16, i16),
    /// The hall angle the angle path aligned this period ([`crate::Commutator::theta`]): the
    /// angle the inverse Park used, unless the observer was the source.
    pub theta: u16,
    /// This period's bus scale ([`bus_kv`]); 0 suspends.
    pub kv: i32,
    /// The front end's latched speed, edges per 320-period window.
    pub speed: i32,
    /// The front end's latched hall fault.
    pub hall_fault: bool,
    /// Whether the bridge drives this period. A floated period's voltage is the motor's, which
    /// nothing measures, so it suspends the integration.
    pub drive: bool,
}

/// A closed window ([`FluxObserver::close`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ObserverWindow {
    /// `theta` of the window's last integrated period.
    pub theta: u16,
    /// The observer's angle on that period.
    pub angle: u16,
    /// The error's mean, observer minus hall less the offset and the bin's lead, angle units.
    pub mean: i16,
    /// The largest excursion from the mean on either side, saturated.
    pub spread: i16,
    /// The flux magnitude's RMS, microvolt-seconds, saturated.
    pub flux: u16,
    /// Every period of the window was integrated and none had a hall fault: the verdict applies.
    pub judged: bool,
    /// Judged, and the mean and the spread inside their bands.
    pub tracked: bool,
}

/// The observer's per-motor state: the flux, and the open window's sums
/// (`specs/flux-observer.md`, "Cost": the flux 8 B, the sums 16 B).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FluxObserver {
    psi_a: i32,
    psi_b: i32,
    /// The vector handed in last period: with this period's, the two halves of the interval
    /// between the last sample and this one.
    v_prev: (i16, i16),
    angle: u16,
    /// The lead the last step subtracted from the error (the bin's leak lead, signed by the
    /// direction of rotation), kept so the driving angle carries the same correction as the
    /// verdict: the raw integrator leads the flux by `atan(f_c / f)`, 18 degrees at the ceiling
    /// and 44 at bin 5, and a wheel driven from the raw angle ran at the wrong speed and
    /// oscillated (bench 2026-11-04, S4 test 1).
    lead: u16,
    theta: u16,
    sum: i32,
    max: i16,
    min: i16,
    mag2: u64,
    /// Periods integrated this window.
    n: u8,
    /// A period of this window carried a hall fault.
    fault: bool,
}

impl Default for FluxObserver {
    fn default() -> Self {
        Self::new()
    }
}

impl FluxObserver {
    /// A zero flux and an empty window.
    pub const fn new() -> Self {
        FluxObserver {
            psi_a: 0,
            psi_b: 0,
            v_prev: (0, 0),
            angle: 0,
            lead: 0,
            theta: 0,
            sum: 0,
            max: i16::MIN,
            min: i16::MAX,
            mag2: 0,
            n: 0,
            fault: false,
        }
    }

    /// The angle the FOC arm takes as its source while the rider holds the observer button
    /// (`specs/flux-observer.md`, "S4, first slice"): the angle the last [`FluxObserver::step`]
    /// computed (the last one integrated while suspended), in `theta`'s frame with the motor's
    /// quarter turn applied (word 52's high half), less `p.offset`, the word that places the halls'
    /// frame against the observer's, so the angle that drives is the one the verdict judges
    /// against `theta`. At the default offset of 0 it is word 52's high half.
    ///
    /// Read before this period's step, it is the integrator's angle at the START of the period,
    /// from the samples up to the last one: it lags the rotor by one period of rotation (2.25
    /// electrical degrees at 100 Hz electrical), where the hall front end's angle is this period's
    /// interpolation. No second `atan2`.
    #[inline(always)]
    pub fn source_angle(&self, p: &ObserverParams) -> u16 {
        self.angle.wrapping_sub(p.offset).wrapping_sub(self.lead)
    }

    /// The flux, 1/16 microvolt-second, stock frame.
    pub fn psi(&self) -> (i32, i32) {
        (self.psi_a, self.psi_b)
    }

    /// One period. Suspended (nothing integrated, nothing counted) when the bus scale is 0 or the
    /// bridge is floated; the window then closes short and is not judged. Returns the observer's
    /// angle (the last one while suspended).
    ///
    /// In the period ISR's zero-wait placement, and out of line, so the two-motor image (the
    /// GD32F103RC, whose flashed span sits near its ceiling) carries one copy for its two motors.
    #[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
    #[inline(never)]
    pub fn step(&mut self, p: &ObserverParams, x: &ObserverInput) -> u16 {
        // The voltage over the interval since the last sample: the sample is at the top of the
        // centre-aligned period and a written duty takes effect at the next update, at the period's
        // end, so the interval's first half ran the vector computed two periods ago and its
        // second half the one computed last period (`x.v`).
        let v = (
            (self.v_prev.0 as i32 + x.v.0 as i32) >> 1,
            (self.v_prev.1 as i32 + x.v.1 as i32) >> 1,
        );
        self.v_prev = x.v;
        if x.kv <= 0 || !x.drive {
            return self.angle;
        }
        // The applied vector in millivolts.
        let mut va = (v.0 * x.kv) >> 16;
        let mut vb = (v.1 * x.kv) >> 16;
        let (ia, ib) = x.i;
        if p.dt_comp {
            // Each phase is short of its command by the dead-time drop with its current's sign;
            // the Clarke of the three signed drops (amplitude-invariant, as the crate's), in the
            // stock frame (beta negated).
            let vdt = (x.kv * p.dt_q) >> 16;
            let ic = -(ia as i32) - ib as i32;
            let (sa, sb, sc) = ((ia as i32).signum(), (ib as i32).signum(), ic.signum());
            va -= (vdt * (2 * sa - sb - sc) * 21_845) >> 16;
            vb += (vdt * (sb - sc) * 37_837) >> 16;
        }
        let i = clarke(ia, ib);
        let shift = p.leak_shift as u32;
        let ra = ((i.alpha as i64 * p.r_q as i64) >> 16) as i32;
        let rb = ((i.beta as i64 * p.r_q as i64) >> 16) as i32;
        self.psi_a = self
            .psi_a
            .wrapping_add(va.wrapping_sub(ra))
            .wrapping_sub(self.psi_a.wrapping_shr(shift));
        self.psi_b = self
            .psi_b
            .wrapping_add(vb.wrapping_sub(rb))
            .wrapping_sub(self.psi_b.wrapping_shr(shift));
        let la = ((i.alpha as i64 * p.l_q as i64) >> 16) as i32;
        let lb = ((i.beta as i64 * p.l_q as i64) >> 16) as i32;
        let pm_a = self.psi_a.wrapping_sub(la) >> 4;
        let pm_b = self.psi_b.wrapping_sub(lb) >> 4;
        // Textbook beta is the stock one negated; from the flux (d) to q is the motor's quarter
        // turn (the frame note).
        let angle = atan2_u16(pm_b.wrapping_neg(), pm_a).wrapping_add(p.quarter);
        self.mag2 = self
            .mag2
            .wrapping_add((pm_a as i64 * pm_a as i64) as u64)
            .wrapping_add((pm_b as i64 * pm_b as i64) as u64);
        // The error, less the offset and the bin's lead in the direction of rotation.
        let lead = p.lead[bin(x.speed)];
        let lead = if x.speed < 0 {
            0u16.wrapping_sub(lead)
        } else {
            lead
        };
        self.lead = lead;
        let e = angle
            .wrapping_sub(x.theta)
            .wrapping_sub(p.offset)
            .wrapping_sub(lead) as i16;
        self.sum = self.sum.wrapping_add(e as i32);
        self.max = self.max.max(e);
        self.min = self.min.min(e);
        self.n = self.n.saturating_add(1);
        self.fault |= x.hall_fault;
        self.angle = angle;
        self.theta = x.theta;
        angle
    }

    /// Close the window: its mean, spread and RMS flux magnitude, and the verdict against `p`'s
    /// bands; the sums restart. A window the observer did not run in reads all zero; one it did not
    /// integrate in full (a suspended or a floated period, or a method switch) carries no verdict
    /// and reads zero but for its two angles (`specs/flux-observer.md`, "Voltage sensing": no verdict for those windows), so the
    /// means are the section 7 shift and no window divides.
    ///
    /// Runs in the period ISR once per window, so it is placed with the ISR, and kept out of line
    /// so a two-motor image carries one copy.
    #[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
    #[inline(never)]
    pub fn close(&mut self, p: &ObserverParams) -> ObserverWindow {
        let w = if self.n == 0 {
            // Not run this window (another method, or every period suspended): all zero, as the
            // FOC words are in another method.
            ObserverWindow::default()
        } else if self.n != WINDOW {
            ObserverWindow {
                theta: self.theta,
                angle: self.angle,
                ..ObserverWindow::default()
            }
        } else {
            let mean = self.sum >> crate::foc::FOC_WINDOW_SHIFT;
            let spread = (self.max as i32 - mean).max(mean - self.min as i32);
            let spread = spread.min(i16::MAX as i32) as i16;
            let flux = isqrt_u64(self.mag2 >> crate::foc::FOC_WINDOW_SHIFT).min(u16::MAX as u32);
            let judged = !self.fault;
            let mean = mean as i16;
            ObserverWindow {
                theta: self.theta,
                angle: self.angle,
                mean,
                spread,
                flux: flux as u16,
                judged,
                tracked: judged
                    && mean.unsigned_abs() <= p.band
                    && spread.unsigned_abs() <= p.spread,
            }
        };
        self.sum = 0;
        self.max = i16::MIN;
        self.min = i16::MAX;
        self.mag2 = 0;
        self.n = 0;
        self.fault = false;
        w
    }
}

/// The speed bin of a latched speed: its magnitude, the last bin holding 15 and above.
#[inline(always)]
pub fn bin(speed: i32) -> usize {
    (speed.unsigned_abs() as usize).min(BINS - 1)
}

// ============================================================================================
// The tracking table
// ============================================================================================

/// The per-speed-bin table (`specs/flux-observer.md`, "Per speed bin"): each bin a `u32`, windows
/// seen in the low half and windows tracked in the high half, both saturating. Atomics because the
/// period ISR is its sole writer and the loop pass reads it for `FOC_TRACE`; a clear is the ISR's
/// too, on a request the loop raises.
pub struct TrackTable {
    bins: [AtomicU32; BINS],
}

impl Default for TrackTable {
    fn default() -> Self {
        Self::new()
    }
}

impl TrackTable {
    /// An empty table.
    pub const fn new() -> Self {
        TrackTable {
            bins: [const { AtomicU32::new(0) }; BINS],
        }
    }

    /// Bin `k`'s word.
    #[inline]
    pub fn word(&self, k: usize) -> u32 {
        self.bins[k % BINS].load(Ordering::Relaxed)
    }

    /// Credit one judged window to bin `k`: seen, and tracked if it was.
    #[inline]
    pub fn credit(&self, k: usize, tracked: bool) {
        let b = &self.bins[k % BINS];
        b.store(
            credit(b.load(Ordering::Relaxed), tracked),
            Ordering::Relaxed,
        );
    }

    /// Clear every bin.
    pub fn clear(&self) {
        for b in &self.bins {
            b.store(0, Ordering::Relaxed);
        }
    }

    /// The lock bin over the table ([`lock_bin`]). Run in the period ISR at every window close,
    /// so placed with it.
    #[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
    #[inline(never)]
    pub fn lock_bin(&self, min_windows: u16, ratio: u16) -> u8 {
        lock_bin(|k| self.word(k), min_windows, ratio)
    }
}

/// A bin word with one more window seen, and tracked if `tracked`, each saturating at `u16::MAX`.
#[inline]
pub fn credit(w: u32, tracked: bool) -> u32 {
    let seen = (w as u16).saturating_add(1);
    let t = ((w >> 16) as u16).saturating_add(tracked as u16);
    seen as u32 | (t as u32) << 16
}

/// Windows seen in a bin word.
#[inline]
pub const fn seen(w: u32) -> u16 {
    w as u16
}

/// Windows tracked in a bin word.
#[inline]
pub const fn tracked(w: u32) -> u16 {
    (w >> 16) as u16
}

/// A bin is measured at `min_windows` windows seen.
#[inline]
pub fn measured(w: u32, min_windows: u16) -> bool {
    seen(w) >= min_windows
}

/// A bin passes when measured and tracked in at least `ratio / 256` of its windows.
#[inline]
pub fn passing(w: u32, min_windows: u16, ratio: u16) -> bool {
    measured(w, min_windows) && tracked(w) as u32 * 256 >= ratio as u32 * seen(w) as u32
}

/// The lock bin (`specs/flux-observer.md`, "The lock bin"): the lowest bin `k` that passes and
/// above which every measured bin passes, [`LOCK_NONE`] when none does. Bin 0 (rest) never passes,
/// and an unmeasured bin neither passes nor blocks. `word(k)` is bin `k`'s word.
pub fn lock_bin(word: impl Fn(usize) -> u32, min_windows: u16, ratio: u16) -> u8 {
    let mut lock = LOCK_NONE;
    for k in (1..BINS).rev() {
        let w = word(k);
        if !measured(w, min_windows) {
            continue;
        }
        if !passing(w, min_windows, ratio) {
            break;
        }
        lock = k as u8;
    }
    lock
}

/// A bin's `FOC_TRACE` byte: tracked over seen in 0.4 percent steps (0..=250), or 255 while the
/// bin is not measured (`specs/link-control.md`, `FOC_TRACE` offset 30).
pub fn track_byte(w: u32, min_windows: u16) -> u8 {
    if !measured(w, min_windows) || seen(w) == 0 {
        return 255;
    }
    (tracked(w) as u32 * 250 / seen(w) as u32) as u8
}

// ============================================================================================
// The published words (`CTRL_OBS` 52 to 54)
// ============================================================================================

/// Word 54's high half, bit 8: this window tracked.
pub const TRACK_TRACKED: u16 = 1 << 8;
/// Word 54's high half, bit 9: the observer is the angle source (the window's last period drove
/// from it).
pub const TRACK_SOURCE: u16 = 1 << 9;
/// Word 54's high half, bit 10: the front end was in its interpolation window at the close.
pub const TRACK_IN_WINDOW: u16 = 1 << 10;

/// Word 52: `theta` low, the observer's angle high.
#[inline]
pub fn pack_angles(w: &ObserverWindow) -> u32 {
    w.theta as u32 | (w.angle as u32) << 16
}

/// Word 53: the error's mean low, its spread high.
#[inline]
pub fn pack_track(w: &ObserverWindow) -> u32 {
    w.mean as u16 as u32 | (w.spread as u16 as u32) << 16
}

/// Word 54: the RMS flux low; the lock bin, the tracked bit, the source bit (`source`: the
/// window's last period drove from the observer) and the in-window bit high.
#[inline]
pub fn pack_flux(w: &ObserverWindow, lock: u8, source: bool, in_window: bool) -> u32 {
    let hi = lock as u16
        | if w.tracked { TRACK_TRACKED } else { 0 }
        | if source { TRACK_SOURCE } else { 0 }
        | if in_window { TRACK_IN_WINDOW } else { 0 };
    w.flux as u32 | (hi as u32) << 16
}

#[cfg(test)]
mod tests;
