//! The plan-gated motor bring-up and the 16 kHz period ISR (`specs/motor-integration.md`, slice 3:
//! "Firmware bring-up, disarmed").
//!
//! This is the motor half of the one universal image. It is **disarmed by construction**: the
//! arming gate (the sole MOE writer, `runtime_hal::timer::arming`) is never named anywhere in this
//! MODULE, so no code path here can energize a bridge. The timer is configured, the counter runs
//! and the compare units toggle, the injected ADC converts and its end-of-conversion ISR steps the
//! commutator, but with MOE clear nothing reaches the gate drivers.
//!
//! Slice 5 added arming to the image without weakening that: the gate lives in `crate::arm`, which
//! holds it and nothing else, and the bring-up hands that module the configured timer rather than a
//! gate derived here. The property a host test now enforces over the crate's source is
//! CONFINEMENT (the arming surface is named in `arm.rs` alone, and MOE is set in exactly one place)
//! where slice 3's test enforced ABSENCE. Everything below it stays a disarmed layer: it can stop
//! the bridge, through the demand word and the channel enables, and it cannot start one.
//!
//! Split, like the rest of this crate: everything that is pure arithmetic or ordering lives here on
//! ALL targets (so the host test run reaches it), and the hardware half is `target_os = "none"`.
//!
//! # What runs where
//!
//! - **The period ISR (16 kHz, highest priority)**: reads the injected currents, reads the halls,
//!   steps `crates/commutation`, applies duties + channel enables, re-arms the ADC trigger,
//!   publishes the handoff words. Nothing else runs there: no allocation, no blocking, no link
//!   work, no flash.
//! - **The 250 Hz control task**: writes the demand word and reads back angle / speed / period
//!   liveness (`crate::firmware`'s `control_task_cb`).
//!
//! The two contexts share only the atomic words below, one writer each, `Relaxed`
//! (`specs/motor-integration.md`, "The 250 Hz to 16 kHz handoff": each word is independently
//! meaningful and no reader derives a conjunction across two of them).
//!
//! # Single motor, deliberately
//!
//! The model is `N = 2`-ready but motor 1 is not built: no 12-FET board is in the validation loop,
//! so a second set of handoff words and a second bring-up would be code nothing exercises. The
//! words below are motor 0's.

// On a HOST target the whole `firmware` module (every consumer of these items) is compiled out, so
// the pure half below reads as dead code there; CI clippys the host surface with
// `--all-targets -D warnings`. Same reason the `probe_window` / `link_drain` helpers carry an
// allow: their one caller is the target-only service loop.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

// -------------------------------------------------------------------------------------------
// The handoff words (motor 0). One writer each; `Relaxed` throughout.
// -------------------------------------------------------------------------------------------

/// The signed drive demand: the +-28500 stock-native torque word, verbatim (`specs/control.md`'s
/// demand-scale contract: no rescaling anywhere). Written by the 250 Hz control task, read by the
/// period ISR.
pub static DEMAND: AtomicI32 = AtomicI32::new(0);
/// Demand write sequence: bumped by the 250 Hz task on every demand write, so the ISR can tell a
/// fresh write from a repeated one WITHOUT the writer having to touch a second data word (a demand
/// that happens to repeat its value is still a fresh write). The ISR's freshness guard counts
/// periods since this last changed.
pub static DEMAND_SEQ: AtomicU32 = AtomicU32::new(0);
/// The wrapping electrical angle (u16 in a u32 cell). Written by the period ISR.
pub static ANGLE: AtomicU32 = AtomicU32::new(0);
/// The signed edge count per 320-period window, raw (the pole-pair fold to speed units is the
/// Phase-D consumer's). Written by the period ISR.
pub static SPEED: AtomicI32 = AtomicI32::new(0);
/// The free-running period counter: the liveness signal. Written by the period ISR.
pub static PERIODS: AtomicU32 = AtomicU32::new(0);
/// Motor fault bits ([`FAULT_HALL`], [`FAULT_DUTY_RANGE`], [`FAULT_DEMAND_STALE`]). Written by the
/// period ISR (sticky within a boot; the fault PRODUCERS that drive shutdown are slice 4).
pub static FAULT: AtomicU32 = AtomicU32::new(0);
/// The invalid-hall dwell count from the commutator's front end. Written by the period ISR.
pub static INVALID_DWELL: AtomicU32 = AtomicU32::new(0);
/// Observation, packed: `hall_code | enables << 8 | method << 16 | flags << 24`. Written by the
/// period ISR (the flags' `configured` / `current_sense` / `cal-accepted` bits are set once at
/// bring-up).
pub static OBS_STATE: AtomicU32 = AtomicU32::new(0);
/// The MEASURED quiet-bridge phase-current offsets, packed `offset_a | offset_b << 16`
/// (`specs/motor-integration.md`, bring-up step 9 and silicon stage 3). Written ONCE, by the
/// calibration step of the bring-up, before the period vector is unmasked; zero on a boot whose
/// conversions never completed. The calibration runs on every brought-up motor, whatever method
/// is requested: the current limit needs a trusted zero on both sensed phases.
///
/// Each half is in the ACCUMULATED offset unit ([`cal_offsets`]): the sum of 16 conversions each
/// shifted right by 3, which is 2x the zero-current register value. A bench read compares it against
/// the acceptance window and the stock healthy band in those units, NOT against a raw sample.
///
/// The measured pair is published whether or not [`commutation::foc::PhaseOffsets`] accepted it,
/// so a bench read sees WHERE an out-of-window board actually sits rather than only that it was
/// refused. Acceptance is the [`OBS_CAL_ACCEPTED`] flag; refusal is [`FAULT_INIT_CAL`]; a zero
/// word with the refusal set means no conversion completed.
pub static OBS_CAL: AtomicU32 = AtomicU32::new(0);
/// The last applied duties, packed `d0 | d1 << 16`. Written by the period ISR.
pub static OBS_DUTY01: AtomicU32 = AtomicU32::new(0);
/// The last applied duty 2 plus the electrical angle, packed `d2 | angle << 16`. Written by the
/// period ISR.
pub static OBS_DUTY2_ANGLE: AtomicU32 = AtomicU32::new(0);
/// Whether the TIMER0 counter is turning, and therefore whether the period ISR should be firing.
/// Set by the bring-up's `StartCounter` and by [`crate::arm`]'s, cleared by the shutdown sequence's
/// counter stop; written only on the boot / 250 Hz thread, never by the ISR.
///
/// It is the PREMISE of the period-liveness supervisor ([`PeriodHealth::update`]): a counter this
/// layer deliberately stopped is not a wedged vector, and reading it as one would turn every clean
/// shutdown into a boot-long fault.
pub static COUNTER_RUNNING: AtomicBool = AtomicBool::new(false);

/// Latched invalid-hall fault (the commutator's dwell fault: > 64 consecutive invalid codes).
pub const FAULT_HALL: u32 = 1 << 0;
/// A duty the HAL refused as out of range (the compare would never match): the outputs were left
/// as they were, per the write-order rule.
pub const FAULT_DUTY_RANGE: u32 = 1 << 1;
/// The demand-freshness guard fired (the 250 Hz task stopped writing; all phases forced to float).
pub const FAULT_DEMAND_STALE: u32 = 1 << 2;
/// The init-failure fault: the offset calibration was REFUSED (`specs/motor-integration.md`,
/// bring-up step 9), so there is no trusted zero-current reference and the current limit cannot
/// be enforced; [`motor_fault_level`] keeps such a board from arming. Set ONCE by the bring-up, before the
/// period vector is unmasked, and carried forward by the ISR (which seeds its own accumulator from
/// it, so the ISR stays the sole writer of [`FAULT`] after the unmask).
pub const FAULT_INIT_CAL: u32 = 1 << 3;

/// `OBS_STATE` flag: the motor was brought up (the plan carried a motor and every step succeeded).
pub const OBS_CONFIGURED: u32 = 1 << 0;
/// `OBS_STATE` flag: the injected (phase-current) path is live, so the period vector is the
/// injected end-of-conversion interrupt.
pub const OBS_CURRENT_SENSE: u32 = 1 << 1;
/// `OBS_STATE` flag: the period ISR advanced `PERIODS` by at least [`PERIODS_PER_TICK_MIN`] over the
/// last 250 Hz tick (the period-liveness observation; the fault producer is slice 4).
pub const OBS_PERIOD_LIVE: u32 = 1 << 2;
/// `OBS_STATE` flag: the demand-freshness override is currently forcing all phases to float.
pub const OBS_COASTING: u32 = 1 << 3;
/// `OBS_STATE` flag: no motor was brought up because the plan carries none ([`MotorSkip::Absent`]).
pub const OBS_SKIP_ABSENT: u32 = 1 << 4;
/// `OBS_STATE` flag: no motor was brought up because it has no phase-current group
/// ([`MotorSkip::NoCurrentSense`]).
pub const OBS_SKIP_NO_SENSE: u32 = 1 << 5;
/// `OBS_STATE` flag: a bring-up step failed ([`MotorSkip::StepFailed`]); the failing step's index
/// into [`BRING_UP_STEPS`] rides in the byte the method occupies when configured.
pub const OBS_SKIP_STEP_FAILED: u32 = 1 << 6;
/// `OBS_STATE` flag: the offset calibration ran and [`commutation::foc::PhaseOffsets`] ACCEPTED
/// the measured pair (both offsets inside its window). Clear on a refused one, which
/// [`FAULT_INIT_CAL`] also records.
pub const OBS_CAL_ACCEPTED: u32 = 1 << 7;

/// Read the bring-up's `configured` fact back out of a packed [`OBS_STATE`] word. The packing is
/// this module's model, so the unpacking is too: the 250 Hz task asks here rather than open-coding
/// the shift (`specs/motor-integration.md`, "Observation").
#[inline]
pub fn obs_configured(state: u32) -> bool {
    state & (OBS_CONFIGURED << 24) != 0
}

// -------------------------------------------------------------------------------------------
// The two guards (pure; `specs/motor-integration.md`, "Two guards the handoff needs")
// -------------------------------------------------------------------------------------------

/// Periods without a fresh demand write after which the ISR applies the all-float output override
/// (16 ms at 16 kHz). A SAFETY parameter, not a tuning detail: the 250 Hz task nominally writes
/// demand every 4 ms and, measured, no worse than ~4.4 ms/tick under peer load, so 16 ms tolerates
/// three to four missed writes without false-tripping on normal slip, while sitting at roughly 1/30
/// of the 500 ms IWDG window (the bridge is silenced by this guard long before the IWDG's
/// reset-based cover would fire).
pub const DEMAND_STALE_PERIODS: u32 = 256;

/// The demand-freshness guard: has the demand word gone stale? The override does NOT fire at
/// exactly [`DEMAND_STALE_PERIODS`] periods and DOES fire at one more.
#[inline]
pub fn demand_stale(periods_since_write: u32) -> bool {
    periods_since_write > DEMAND_STALE_PERIODS
}

/// Periods the ISR runs per 250 Hz tick when healthy: 16000 / 250 = 64.
pub const PERIODS_PER_TICK_NOMINAL: u32 = 64;
/// The period-liveness floor per tick: half the nominal, so ordinary tick jitter (a control run that
/// lands early or late against the free-running 16 kHz ISR) cannot read as a stopped ISR, while a
/// wedged vector / stopped counter / stalled trigger reads as one immediately.
pub const PERIODS_PER_TICK_MIN: u32 = PERIODS_PER_TICK_NOMINAL / 2;

/// The period-liveness guard: did the period ISR advance far enough over one 250 Hz tick? A
/// shortfall means the ISR has stopped while the bridge may still hold its last duties (a `fault_a`
/// producer in slice 4; observed as [`OBS_PERIOD_LIVE`] here).
#[inline]
pub fn periods_live(delta_periods: u32) -> bool {
    delta_periods >= PERIODS_PER_TICK_MIN
}

/// Consecutive 250 Hz ticks of period-ISR shortfall before the liveness LOSS level asserts (20 ms).
/// The spec's producer is a SUSTAINED shortfall, not a single tick's: one tick can land short from
/// ordinary scheduling jitter against a free-running 16 kHz counter, while a wedged vector, a
/// stopped counter or a stalled trigger never recovers, so five in a row separates them.
pub const PERIOD_LOSS_THRESHOLD: u16 = 5;
/// Consecutive live ticks that clear the loss level again. Asymmetric with
/// [`PERIOD_LOSS_THRESHOLD`] on purpose (the shape `orchestrator::ImuHealth` already uses for the
/// IMU-loss level): the fault asserts quickly and releases only on a proven-clean stream.
pub const PERIOD_RECOVER_THRESHOLD: u16 = 25;

/// The period-liveness fault level: the hysteresis latch over [`periods_live`], mirroring the
/// IMU-loss supervisor's shape. Absence (no motor brought up) is NOT loss and never touches it, so
/// a board with no motor never faults on a counter that was always going to stay at zero.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PeriodHealth {
    fail_streak: u16,
    ok_streak: u16,
    loss: bool,
}

impl PeriodHealth {
    /// A healthy start: no shortfall, no loss.
    pub const fn new() -> Self {
        PeriodHealth {
            fail_streak: 0,
            ok_streak: 0,
            loss: false,
        }
    }

    /// Fold one 250 Hz tick's liveness observation in. `running` false = the counter that the ISR
    /// rides is not turning, because the motor was never brought up OR because a shutdown stopped it
    /// (reset to healthy, never loss).
    ///
    /// **The premise is a RUNNING counter, not merely a configured motor** (slice 5): the shutdown
    /// sequence's last step stops the counter, so between a disarm and the next arm the period ISR
    /// is correctly silent. Gating on `configured` alone would read that deliberate silence as a
    /// wedged vector, assert the loss level into `fault_a`, and hold the vehicle out of RUN for the
    /// rest of the boot: a fault produced by the shutdown that was supposed to end cleanly.
    pub fn update(&mut self, running: bool, live: bool) {
        if !running {
            *self = PeriodHealth::new();
            return;
        }
        if live {
            self.fail_streak = 0;
            self.ok_streak = self.ok_streak.saturating_add(1);
            if self.loss && self.ok_streak >= PERIOD_RECOVER_THRESHOLD {
                self.loss = false;
            }
        } else {
            self.ok_streak = 0;
            self.fail_streak = self.fail_streak.saturating_add(1);
            if self.fail_streak >= PERIOD_LOSS_THRESHOLD {
                self.loss = true;
            }
        }
    }

    /// The period-liveness loss level (feeds `fault_a`).
    pub const fn loss(&self) -> bool {
        self.loss
    }
}

/// The motor's contribution to `fault_a` (`specs/motor-integration.md`, "the motor-side fault
/// producers"): the hall dwell fault, period-liveness loss, a refused calibration, and slice 5's
/// refused arm. All four drive SHUTDOWN and therefore disarm.
///
/// The other two [`FAULT`] bits are deliberately NOT producers here: [`FAULT_DEMAND_STALE`] is
/// self-mitigating (the ISR has already floated every phase by the time it is set, so escalating
/// to SHUTDOWN adds no silencing), and [`FAULT_DUTY_RANGE`] records a refused write whose outputs
/// were left untouched by the write-order rule. Both stay observable in the motor block.
///
/// Gated on `configured`: a board with no motor brought up publishes a zero fault word and a
/// never-advancing period counter, and neither is a fault.
///
/// `arm_refused` is slice 5's fourth producer: an arm attempt that could not confirm the period ISR
/// was live (`crate::arm`). It is a producer because the alternative posture is the dangerous one --
/// a vehicle sitting in RUN believing it is driving, with a bridge that was never energized and a
/// commutator that was never stepped. Making it a fault turns that into a shutdown.
///
/// The three [`FAULT`]-word producers are read from the WORD; the other two levels arrive as levels.
/// Nothing downstream ever reads the word again: this function is the level's single owner, and the
/// arming gate consumes only what it returns.
#[inline]
pub fn motor_fault_level(
    configured: bool,
    faults: u32,
    period_loss: bool,
    arm_refused: bool,
) -> bool {
    configured && (period_loss || arm_refused || (faults & (FAULT_HALL | FAULT_INIT_CAL)) != 0)
}

// -------------------------------------------------------------------------------------------
// The current limit (pure; `specs/motor-integration.md`, "The current limit")
// -------------------------------------------------------------------------------------------

/// The current-sense calibration's seam range, stock current counts per amp of phase current.
///
/// The count is the unit [`commutation::foc::current_from_adc`] defines and the FOC arm consumes
/// (`offset - 2*sample` over the left-aligned injected word, so one 12-bit ADC LSB of deviation is
/// 16 counts and the sensor's full scale is +-32767 counts); nothing is rescaled, the limit is
/// converted INTO it. The scale itself is per-board data, read at boot from
/// `store::MOTOR_CURRENT_CAL` (0x67) into `board::MotorPlan` and carried into [`limit_counts`]:
/// it is a property of the shunt and amplifier chain fitted, not a constant
/// (`specs/motor-integration.md`, "The current-sense calibration").
///
/// The seam clamps it HERE, because the store validates type only (the
/// `orchestrator::battery::VbattCal::new` precedent). The upper bound is forced by the `i16` the
/// limit comparison holds: `CURRENT_LIMIT_CEILING_MA * cal / 1000 <= i16::MAX` gives 819. The
/// lower bound keeps a zero or a typo from making every milliamp limit saturate the comparator.
pub const CURRENT_CAL_MIN: u16 = 100;
/// The calibration's upper seam bound; see [`CURRENT_CAL_MIN`].
pub const CURRENT_CAL_MAX: u16 = 819;

/// The converted limit's FLOOR, in counts rather than milliamps (2,000): a mis-staged tiny value
/// would otherwise turn the bridge into a permanent chop, which reads as "no drive" with nothing to
/// say why.
///
/// A count, not a milliamp, because what it protects against is the sense chain's NOISE, which is a
/// count-domain fact: with a per-board scale a milliamp floor would mean a different count on every
/// board. A milliamp request that converts to less than this is clamped UP to it.
///
/// **2,100, from the HIGH-WATER of five rest-floor reads on 2026-10-09, not the lowest.** Gate 1 of
/// the five sessions that day read maxima of 1,444 / 1,863 / 1,910 / 1,941 / **2,097** counts
/// (medians ~1,050), and an earlier 2,000 here was set from one of the low reads. It was too low:
/// in the 09:23 session's armed STILL soak, demand and duty 0 throughout, the limiter chopped in
/// 2 of 94 samples against a 2,000-count limit, which is the exact condition this floor exists to
/// prevent. The bench tool's own fallback floor already used 2,100 for the same reason.
///
/// Per-board in truth, and queued as such (`specs/store-field-audit.md`, the 2026-10-09 sweep,
/// Tier 1 item 1): this is one board's noise measured in the units of a field that is now per-board,
/// so the number belongs beside `motor.current_cal`, not here.
pub const MIN_LIMIT_COUNTS: i16 = 2_100;

/// The staged limit's ceiling (40 A): keeps the comparison inside the sensor's full scale.
pub const CURRENT_LIMIT_CEILING_MA: u32 = 40_000;

// Where the 819 comes from, so the clamp cannot drift from its derivation: the ceiling must convert
// to a count the i16 comparison can hold at the TOP of the calibration's seam range. The runtime
// clamp in `limit_counts` is what holds the property for any stored value; this pins the bound it
// clamps to.
const _: () = assert!(
    CURRENT_LIMIT_CEILING_MA * CURRENT_CAL_MAX as u32 / 1000 <= i16::MAX as u32,
    "the current-limit ceiling no longer fits the sensor's count range"
);

/// Consecutive over-limit periods that trip the hard over-current fault (one nominal control
/// tick, the hall dwell fault's constant class): the chop is not containing the current.
pub const OVER_CURRENT_TRIP_PERIODS: u32 = 64;

/// The over-current trip count: the period ISR increments it on each trip and is its sole
/// writer; the 250 Hz task raises the motor-0 fault latch when it sees it change.
pub static OVER_CURRENT_TRIPS: AtomicU32 = AtomicU32::new(0);
/// Observation, packed by [`pack_motor_current`]: the last completed 64-period window's peak
/// phase-current magnitude, the periods the soft limit floated in it, and the trip count's low
/// byte. Written by the period ISR at each window boundary (`CTRL_OBS` word 31).
pub static OBS_CURRENT: AtomicU32 = AtomicU32::new(0);
/// The battery-sense count (`specs/sensing-and-safety.md`, "The battery word", acquisition): the
/// injected group's THIRD rank, reduced once to the 12-bit right-aligned count (`sample >> 3`, the
/// left-aligned datum's unit, bring-up step 9's contract). Written by the period ISR every period,
/// one store and no arithmetic; read by the 250 Hz task, which converts and filters it
/// (`orchestrator::battery`). Stays 0 on a board whose group has no battery rank or never
/// converts, which the reader takes as "no conversion yet" (the word stays UNKNOWN).
pub static VBATT_RAW: AtomicU32 = AtomicU32::new(0);

/// The injected group's channel list, in rank order (`specs/motor-integration.md` bring-up step
/// 5): the two phase-current ranks from the plan (rank 0 = phase A, rank 1 = phase B, the ranks
/// the current limit reads), plus the battery rank as rank 2 when the plan carries `board.vbatt`.
/// Two slots or three, never the stock four: the aux consumer does not exist here.
pub fn injected_ranks(phase: [u8; 2], vbatt: Option<u8>) -> heapless::Vec<u8, 4> {
    let mut ranks = heapless::Vec::new();
    // Cannot overflow: at most three pushes into a capacity-4 vector.
    for ch in phase.into_iter().chain(vbatt) {
        let _ = ranks.push(ch);
    }
    ranks
}

/// Convert the staged `MOTOR_CURRENT_LIMIT` (milliamps) into the soft limit in stock current
/// counts, once, at bring-up: `min(ma, 40 A) * clamp(cal, 100, 819) / 1000`, floored at
/// [`MIN_LIMIT_COUNTS`].
///
/// `cal` is this board's own counts per amp (`store::MOTOR_CURRENT_CAL`, carried on
/// `board::MotorPlan`), and this is the boot seam that clamps it: the store validates type only, so
/// a hand-poked out-of-range flash value cannot reach the comparison.
#[inline]
pub fn limit_counts(ma: u32, cal: u16) -> i16 {
    let cal = cal.clamp(CURRENT_CAL_MIN, CURRENT_CAL_MAX) as u32;
    // Cannot exceed i16::MAX: the const assert above pins the ceiling-times-CURRENT_CAL_MAX
    // product, and both factors are clamped to it here.
    let counts = (ma.min(CURRENT_LIMIT_CEILING_MA) * cal / 1000) as i16;
    counts.max(MIN_LIMIT_COUNTS)
}

/// The hard trip's magnitude: twice the soft limit, saturated to the sensor's full scale. A
/// working chop cannot be holding a current this far over its limit (one period of float moves a
/// few amps); a shorted phase or a wrong hall table into a locked rotor can.
#[inline]
pub const fn hard_trip_counts(limit_counts: i16) -> i16 {
    let h = 2 * limit_counts as i32;
    if h > i16::MAX as i32 {
        i16::MAX
    } else {
        h as i16
    }
}

/// The phase-current magnitude this period, from the two sensed phases: the third is the
/// Kirchhoff remainder `-(ia + ib)`, saturated the stock way, and the magnitude is the largest of
/// the three absolute values (saturating, so it is always in `0..=32767`).
#[inline(always)]
pub fn phase_magnitude(ia: i16, ib: i16) -> i16 {
    let ic = commutation::foc::sat16(-(ia as i32 + ib as i32));
    ia.saturating_abs()
        .max(ib.saturating_abs())
        .max(ic.saturating_abs())
}

/// Pack `CTRL_OBS` word 31: `peak` (i16 counts) in bits 0..15, `chopped` in 16..23, and the trip
/// count's low byte (wrapping, for display; the full count is [`OVER_CURRENT_TRIPS`]) in 24..31.
#[inline(always)]
pub fn pack_motor_current(peak: i16, chopped: u8, trips: u32) -> u32 {
    (peak as u16 as u32) | ((chopped as u32) << 16) | ((trips & 0xFF) << 24)
}

/// One period's verdict from [`CurrentLimit::step`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CurrentVerdict {
    /// Over the soft limit: float every phase THIS period (the channel enables, never MOE).
    pub chop: bool,
    /// The hard trip fired this period: the ISR publishes the new trip count.
    pub trip: bool,
}

/// The per-period current limit: the soft-limit decision, the hard trip, and the observation
/// window. Lives in the period ISR's record, built once at bring-up from the converted limit.
///
/// A trip counts once per over-limit EPISODE: both trip conditions stay true for every further
/// over-limit period, so the run is re-armed only by a period under the limit (the same period
/// that resets the consecutive count).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CurrentLimit {
    limit: i16,
    hard_trip: i16,
    /// Consecutive over-limit periods.
    over_run: u32,
    /// This over-limit episode has already tripped.
    tripped: bool,
    /// Trips so far this boot.
    trips: u32,
    /// The running maximum magnitude of the open window.
    peak: i16,
    /// Periods the soft limit floated in the open window.
    chopped: u8,
}

impl CurrentLimit {
    /// A limit of `limit_counts` (see [`limit_counts`]), no history.
    pub const fn new(limit_counts: i16) -> Self {
        CurrentLimit {
            limit: limit_counts,
            hard_trip: hard_trip_counts(limit_counts),
            over_run: 0,
            tripped: false,
            trips: 0,
            peak: 0,
            chopped: 0,
        }
    }

    /// Re-derive the LIMIT at an arm, keeping the boot-cumulative observation
    /// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read").
    ///
    /// This type holds two kinds of thing, and the arm-time re-read moves exactly one of them:
    ///
    /// - **configuration**: `limit` and `hard_trip`, which ARE the value row (the stored milliamp
    ///   limit converted through [`limit_counts`] against the stored per-board calibration). They
    ///   are replaced here, the hard trip through [`hard_trip_counts`] so that arithmetic keeps its
    ///   single owner exactly as in [`CurrentLimit::new`].
    /// - **observation**: `trips`, `peak` and `chopped`, which are NOT. `trips` is documented
    ///   "trips so far this boot", it is published through [`OVER_CURRENT_TRIPS`] and into
    ///   `CTRL_OBS` word 31 by [`pack_motor_current`], and that block's counters are
    ///   boot-cumulative by contract. A re-arm that restarted the count would make a published
    ///   counter step BACKWARDS, which the first thing to exercise it would see: the current-limit
    ///   bench gate (`specs/silicon-queue.md`, gate 5) ends by re-arming after a trip and watching
    ///   `trips` and the latch. So they survive.
    ///
    /// The EPISODE state (`over_run`, `tripped`) is reset, which is neither of those: it is the
    /// in-flight judgement of one over-limit run against the limit that has just been replaced, so
    /// carrying it would suppress or mis-date the first trip under the new limit. Resetting it is
    /// free of consequence here because the bridge is disarmed for the whole arm sequence (MOE is
    /// its last step), so no episode can be genuinely in flight.
    pub fn reconfigure(&mut self, limit_counts: i16) {
        self.limit = limit_counts;
        self.hard_trip = hard_trip_counts(limit_counts);
        self.over_run = 0;
        self.tripped = false;
    }

    /// Fold one period's magnitude in. The chop is `mag > limit` (not `>=`), on this period's
    /// magnitude, unfiltered. The trip is `mag >= hard_trip_counts` or the
    /// [`OVER_CURRENT_TRIP_PERIODS`]th consecutive over-limit period.
    #[inline(always)]
    pub fn step(&mut self, mag: i16) -> CurrentVerdict {
        self.peak = self.peak.max(mag);
        if mag <= self.limit {
            self.over_run = 0;
            self.tripped = false;
            return CurrentVerdict {
                chop: false,
                trip: false,
            };
        }
        self.chopped = self.chopped.saturating_add(1);
        self.over_run = self.over_run.saturating_add(1);
        let trip =
            !self.tripped && (mag >= self.hard_trip || self.over_run >= OVER_CURRENT_TRIP_PERIODS);
        if trip {
            self.tripped = true;
            self.trips = self.trips.wrapping_add(1);
        }
        CurrentVerdict { chop: true, trip }
    }

    /// Trips so far this boot.
    #[inline(always)]
    pub fn trips(&self) -> u32 {
        self.trips
    }

    /// Close the window: the packed observation word for it, and the running maximum and chop
    /// count restart for the next one.
    #[inline(always)]
    pub fn take_window(&mut self) -> u32 {
        let w = pack_motor_current(self.peak, self.chopped, self.trips);
        self.peak = 0;
        self.chopped = 0;
        w
    }
}

// -------------------------------------------------------------------------------------------
// The bring-up step list (pure; `specs/motor-integration.md`, "Bring-up")
// -------------------------------------------------------------------------------------------

/// How many times the bring-up re-asserts the injected group's external-trigger enable while
/// waiting for the group to start (`specs/motor-integration.md`, the maintained-trigger rule).
///
/// **Why a re-assert loop exists at all**: a once-only ETEIC loses a bring-up race on silicon (the
/// F103 master converted on 4/4 clean PORs of one image and 0/2 of another built from the same
/// source at a different link layout, from a byte-identical programmed register set), and the stock
/// firmware never runs a statically-armed ETEIC: it re-asserts the bit every PWM period. This is
/// the half of that mechanism the ISR cannot provide, since a group that never converts never
/// enters the ISR to be re-asserted from it.
pub const TRIGGER_START_ATTEMPTS: u32 = 8;

/// Poll iterations spent waiting for the injected end-of-conversion flag after each re-assert. At
/// 72 MHz a volatile-read poll iteration is a few cycles, so this covers tens of 62.5 us periods
/// per attempt: a healthy group sets EOIC on the FIRST period and exits immediately, and only a
/// board where the trigger is genuinely dead pays the full
/// `TRIGGER_START_ATTEMPTS x TRIGGER_START_SPINS` (a few tens of milliseconds, well inside the
/// 500 ms IWDG window), after which it is recorded as a failed step rather than left running with a
/// dead trigger.
pub const TRIGGER_START_SPINS: u32 = 20_000;

/// Timer-triggered conversions accumulated per channel by the offset calibration
/// (`specs/motor-integration.md`, bring-up step 9: "the 16-conversion offset calibration per
/// channel on a quiet bridge"). At 16 kHz the whole measurement costs 1 ms of boot. It is the
/// array length [`commutation::foc::calibrate_offset`] takes, so the count and the accumulation
/// cannot drift apart: changing it here stops compiling rather than silently rescaling the offset.
pub const CAL_SAMPLES: usize = 16;

/// Fold a pair of [`CAL_SAMPLES`]-conversion sample runs into the measured offset pair.
///
/// The arithmetic itself is NOT here: each channel goes through
/// [`commutation::foc::calibrate_offset`], the single owner of the offset's unit, which sums
/// `sample >> 3` over the 16 conversions. The injected sample is left-aligned `<< 3` (RM0008
/// Figure 31), so that accumulation is 2x the zero-current register value, which is exactly the unit
/// [`commutation::foc::PhaseOffsets`]'s acceptance window and `current_from_adc`'s
/// `offset - 2*sample` are expressed in. This function only pairs the two channels, so the pairing
/// is host-testable without an ADC.
#[inline]
pub fn cal_offsets(samples_a: &[u16; CAL_SAMPLES], samples_b: &[u16; CAL_SAMPLES]) -> (u16, u16) {
    (
        commutation::foc::calibrate_offset(samples_a),
        commutation::foc::calibrate_offset(samples_b),
    )
}

/// What the offset calibration did this boot. It runs on every brought-up motor (every one has
/// current sense by construction, see [`MotorSkip::NoCurrentSense`]), because the current limit
/// reads both sensed phases against their measured zeros whatever method runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalOutcome {
    /// The calibration ran and [`commutation::foc::PhaseOffsets`] ACCEPTED the measured pair.
    Accepted,
    /// The calibration ran and was REFUSED: either an offset outside `PhaseOffsets`'s acceptance
    /// window, or no conversion completed inside the poll budget. Both mean the same thing to the
    /// policy (there is no trustworthy zero-current reference), and the measured pair, or its
    /// absence, separates them in [`OBS_CAL`].
    Refused,
}

/// The init-failure fault bits a calibration outcome raises (`specs/motor-integration.md`,
/// bring-up step 9 + `sensing-and-safety.md` delta (b)): a REFUSED calibration falls back to
/// six-step AND raises [`FAULT_INIT_CAL`], which [`motor_fault_level`] feeds into `fault_a`. This
/// is `commutation.md`'s validator rule; the mode machine stays method-agnostic.
///
/// The fallback half of that rule is currently subsumed by [`running_method`]'s built-arm clamp
/// (six-step is the only arm built, so every boot already runs six-step). The fault bit is
/// therefore the outcome's whole observable effect, and it is the half that matters: a board with
/// no trustworthy zero cannot enforce the current limit, and a board that cannot enforce the limit
/// does not drive. Since the calibration became unconditional this applies to six-step boards
/// too.
///
/// The `Foc`-against-`current_sense = 0` arm of the spec's policy is NOT expressed here: such a
/// motor is not brought up at all ([`MotorSkip::NoCurrentSense`]), because the no-current-sense
/// period vector has no registered handler, so there is no six-step to fall back TO. It is
/// recorded distinguishably in the observation block instead, and the fallback lands with the
/// no-current-sense period path that gives it something to fall back to.
#[inline]
pub fn init_fault_bits(cal: CalOutcome) -> u32 {
    match cal {
        CalOutcome::Refused => FAULT_INIT_CAL,
        CalOutcome::Accepted => 0,
    }
}

/// One step of the `InitAction` bring-up sequence. The vocabulary deliberately cannot express
/// arming: there is no MOE step, and the enactor below holds no arming gate, so the ordered list IS
/// the proof that the bring-up never energizes the bridge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BringUpStep {
    /// Enable TIMER0's peripheral clock. Load-bearing FIRST: an unclocked GD32 advanced timer
    /// silently ignores register writes and reads back zero (bench-proven 2026-07-26).
    EnableTimerClock,
    /// Program the time base + the three complementary channel pairs + dead-time, MOE off.
    ConfigureTimer,
    /// Program CH3 as the ADC-trigger compare (2249, PWM mode 1) and TRGO = update.
    ConfigureTriggerChannel,
    /// Route the six gate pins to the advanced-timer alternate function. AFTER the timer is
    /// configured, so the pins take the configured idle levels the instant they leave input mode.
    RouteGatePins,
    /// Enable the ADC peripheral clock (and its prescaler).
    EnableAdcClock,
    /// Put the two phase-current pins, and the battery-sense pin when the plan carries one, in
    /// analog mode (the digital input buffer would otherwise clamp the sample).
    ConfigurePhasePins,
    /// Program + calibrate the injected group: the two phase-current ranks plus the battery rank
    /// when the plan carries `board.vbatt` ([`injected_ranks`]), 7.5-cycle sampling,
    /// left-aligned, TIMER0 CH3 trigger, scan mode, EOIC interrupt enabled.
    ConfigureInjectedGroup,
    /// Start the counter. Safe while disarmed: outputs do not reach the pins until MOE is set.
    StartCounter,
    /// Confirm the timer-triggered injected group has actually STARTED, re-asserting its
    /// external-trigger enable until it does (the maintained-trigger rule; see
    /// [`TRIGGER_START_ATTEMPTS`]). AFTER `StartCounter`, load-bearing: the trigger event only
    /// exists once the counter runs, so a confirm placed earlier could never see a conversion.
    ConfirmTriggerStart,
    /// Measure the quiet-bridge phase-current zero offsets ([`CAL_SAMPLES`] timer-triggered
    /// conversions per channel) and gate them through [`commutation::foc::PhaseOffsets`], on
    /// every brought-up motor (the current limit's zero reference). AFTER `ConfirmTriggerStart`, load-bearing: the measurement
    /// reads conversions the confirm has just proven are happening, so a board with a dead trigger
    /// never reaches a calibration that could only time out. Its own step (rather than folded into
    /// `SelectMethodAndInstall`, the spec's step 9) so a refusal is attributable in the
    /// observation block and the ordering is testable as data.
    ///
    /// The bridge is quiet by construction here: MOE is never written in this crate, so no phase
    /// can be driven while the offsets are measured.
    CalibratePhaseOffsets,
    /// Read `MOTOR_METHOD`, build the commutator records, install the runtime the ISR reads.
    SelectMethodAndInstall,
    /// Register the period handler on the HAL's control-handler seam.
    RegisterPeriodHandler,
    /// Unmask the period vector in the NVIC. **LAST**, a deliberate deviation from the spec's
    /// step-7-then-8 order: the ISR clears the injected end-of-conversion flag through the handle
    /// that `SelectMethodAndInstall` publishes, and EOIC is a level source, so a vector unmasked
    /// before the runtime exists would re-enter forever instead of being a benign no-op.
    EnablePeriodVector,
}

/// The bring-up sequence, in order. One list: a motor without phase-current sense is not brought up
/// at all in this slice (the no-current-sense period vector is the TIMER0 update interrupt, and the
/// HAL wires no registered handler to it), so there is no second ordering to express.
pub const BRING_UP_STEPS: [BringUpStep; 13] = [
    BringUpStep::EnableTimerClock,
    BringUpStep::ConfigureTimer,
    BringUpStep::ConfigureTriggerChannel,
    BringUpStep::RouteGatePins,
    BringUpStep::EnableAdcClock,
    BringUpStep::ConfigurePhasePins,
    BringUpStep::ConfigureInjectedGroup,
    BringUpStep::StartCounter,
    BringUpStep::ConfirmTriggerStart,
    BringUpStep::CalibratePhaseOffsets,
    BringUpStep::SelectMethodAndInstall,
    BringUpStep::RegisterPeriodHandler,
    BringUpStep::EnablePeriodVector,
];

/// Why a motor was not brought up. Recorded, never fatal: a board with no motor (or with one this
/// slice cannot drive) boots exactly as it did before, link + control only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorSkip {
    /// The plan carries no gate set or no hall set: the motor is absent, a valid board state.
    Absent,
    /// The plan carries no phase-current group. The period vector for such a motor is the TIMER0
    /// update interrupt, which no registered-handler seam covers yet; bringing the timer up with no
    /// period ISR would leave a running counter nothing steps, so the motor is left alone.
    NoCurrentSense,
    /// A bring-up step failed on the hardware (a base that did not resolve, a refused config, a
    /// calibration timeout). Carries the step that failed.
    StepFailed(BringUpStep),
}

// -------------------------------------------------------------------------------------------
// Method selection (pure)
// -------------------------------------------------------------------------------------------

/// Decode the `MOTOR_METHOD` store byte into the REQUESTED commutation method. A policy input,
/// not a plan field: `commutation.md`'s "switching applies only while disarmed" rule means it is
/// re-read at every bring-up. [`running_method`] decides what actually runs.
#[inline]
pub fn requested_method(method_byte: u8) -> commutation::CommutationMethod {
    commutation::CommutationMethod::from_u8(method_byte)
}

/// The commutation method the bring-up will actually run: the BUILT-ARM CLAMP.
///
/// **Six-step only, still.** Sine is slice 6 and FOC is slice 7, and neither has a bench gate
/// before then: `Sine`'s open-loop modulation is signed off against a scope on a spinning wheel,
/// and `Foc`'s per-mode records (`commutation::foc::FocState`) stay uninhabited until its own
/// slice, the calibration this slice adds notwithstanding (measuring the offsets is not the same
/// as running the current loop that consumes them). So every requested method runs six-step and
/// READS BACK as six-step in the observation block, rather than silently claiming a method it is
/// not running.
///
/// Every variant is named rather than caught by a wildcard, so adding a method to the crate is a
/// compile error here and building an arm is an edit here.
///
/// Keeping the unbuilt arms out is also what keeps them out of the IMAGE: the dispatch is a match
/// on the records, so a method never selected is a method LTO drops.
#[inline]
pub fn running_method(requested: commutation::CommutationMethod) -> commutation::CommutationMethod {
    use commutation::CommutationMethod as M;
    match requested {
        M::SixStep | M::Sine | M::Foc => M::SixStep,
    }
}

// -------------------------------------------------------------------------------------------
// The arm-time re-derivation (pure; `specs/integration.md`, "When a stored value takes effect:
// the arm-time re-read")
// -------------------------------------------------------------------------------------------

/// The bring-up-row facts the arm-time rebuild REUSES rather than re-reads.
///
/// Both are `board::MotorPlan` fields, derived once at boot from the staged layout
/// (`board::plumbing::read_fields` -> `board::validate`), and both stay in the PERIPHERAL row of the
/// spec's table: the direction and the align offset are hall-to-phase wiring facts, not values a
/// rider tunes, and the plan they live on is validated as a whole against the detected silicon. The
/// arm path therefore carries them forward from the runtime the bring-up built instead of reading
/// the store again, which is what makes an arm-time re-read of `motor.method` a RAM rebuild and
/// nothing more.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BootFixed {
    /// Drive direction: `false` = Forward, `true` = Reverse (`board::MotorPlan::direction`).
    pub direction: bool,
    /// Six-step align offset, 0..5 (`board::MotorPlan::align_offset`).
    pub align_offset: u8,
}

/// The six-step records for a motor, from the two boot-fixed decode facts.
///
/// ONE construction shape, shared by the bring-up's [`BringUpStep::SelectMethodAndInstall`] and the
/// arm-time [`rederive`] below, so a re-armed board cannot get records built differently from the
/// ones the boot installed. Six-step only, because [`running_method`] clamps every requested method
/// to six-step and the other two arms are not built; building an arm is an edit THERE and here
/// together.
#[inline]
pub fn six_step_records(boot: BootFixed) -> commutation::MethodState {
    commutation::MethodState::SixStep(commutation::sixstep::SixStepState::new(
        commutation::sixstep::SixStep::new(
            if boot.direction {
                commutation::sixstep::Direction::Reverse
            } else {
                commutation::sixstep::Direction::Forward
            },
            boot.align_offset,
        ),
    ))
}

/// EXACTLY what an arm-time re-read installs into the period ISR's record, and nothing else.
///
/// The absences are the content: no offsets (the measured quiet-bridge zeros are carried through
/// untouched, see the install), no `base_flags`, no fault word, no handle. A field here is a field
/// the arm path writes.
#[derive(Clone, Copy, Debug)]
pub struct Rederived {
    /// The method byte the ISR publishes: [`running_method`] of [`requested_method`] of the stored
    /// byte, so a clamped request reads back as what actually runs.
    pub method: u8,
    /// The soft limit in stock current counts, re-derived through [`limit_counts`] from the stored
    /// milliamp limit and the stored per-board calibration.
    ///
    /// A bare count rather than a built [`CurrentLimit`], deliberately: the ISR's record mixes this
    /// configuration with a boot-cumulative trip count, and a built record here could only carry a
    /// zeroed one into the install. Carrying the count means the installed value set structurally
    /// CANNOT restart an observation ([`CurrentLimit::reconfigure`] is what applies it).
    pub limit_counts: i16,
    /// Fresh per-mode records for the running method ([`six_step_records`]), installed through
    /// `commutation::Commutator::switch_method` so the shared rotor front end survives.
    pub records: commutation::MethodState,
}

/// Re-derive the motor's VALUE row from freshly read store values plus the boot-fixed decode facts.
///
/// Pure: it validates and converts, and it touches no peripheral and no static. The three inputs
/// are the three value-row motor fields (`MOTOR_METHOD`, `MOTOR_CURRENT_LIMIT`,
/// `MOTOR_CURRENT_CAL`); everything else the ISR's record holds comes from `boot` or is left alone.
///
/// The conversions are the boot path's own, by the same owners: [`requested_method`] +
/// [`running_method`] for the method, [`limit_counts`] + [`CurrentLimit::new`] for the limit, and
/// [`six_step_records`] for the records. Nothing is re-implemented here.
///
/// # Why re-reading the method is safe for the injected ADC group (the spec's first edge)
///
/// Re-reading `motor.method` rebuilds [`commutation::MethodState`], which is RAM, and NOTHING in
/// the injected group depends on the method choice. The group's rank list comes from
/// [`injected_ranks`], which takes the plan's phase-current channels and the battery channel and
/// has no method argument at all, and programming it is a BRING-UP step
/// ([`BringUpStep::ConfigureInjectedGroup`]) driven by `plan.phase_current` and `plan.vbatt` alone.
/// Six-step and sine share that group, and [`running_method`] clamps every requested byte to
/// six-step today, so the records this installs are six-step whatever was stored. A method that
/// wanted different ranks would have to change `injected_ranks`, which would make the group a
/// peripheral re-configuration and put the field back in the bring-up row; it does not.
pub fn rederive(
    method_byte: u8,
    current_limit_ma: u32,
    current_cal: u16,
    boot: BootFixed,
) -> Rederived {
    Rederived {
        method: running_method(requested_method(method_byte)).to_u8(),
        limit_counts: limit_counts(current_limit_ma, current_cal),
        records: six_step_records(boot),
    }
}

/// Record a motor that was NOT brought up into the observation word, so a bench read distinguishes
/// "no motor configured" from "configured but this slice cannot drive it" from "a step failed, and
/// which one" -- all of which look alike as a silent absence otherwise. Every arm leaves
/// [`OBS_CONFIGURED`] clear.
/// `#[inline(always)]` with its ONE call site (the boot path): a single packed store. Left to the
/// inliner it stayed out of line, as a 86 B function, once the boot/loop split stopped `main` being
/// one whole-program body; inlined it costs the boot frame nothing.
#[inline(always)]
pub fn record_skip(skip: MotorSkip) {
    let (flag, step_index) = match skip {
        MotorSkip::Absent => (OBS_SKIP_ABSENT, 0),
        MotorSkip::NoCurrentSense => (OBS_SKIP_NO_SENSE, 0),
        MotorSkip::StepFailed(step) => (
            OBS_SKIP_STEP_FAILED,
            BRING_UP_STEPS
                .iter()
                .position(|s| *s == step)
                .unwrap_or(0xFF) as u8,
        ),
    };
    OBS_STATE.store(
        pack_obs_state(0, [false; 3], step_index, flag),
        Ordering::Relaxed,
    );
}

/// Pack the observation word the bench reads out of `CTRL_OBS`.
#[inline]
pub fn pack_obs_state(hall_code: u8, enables: [bool; 3], method: u8, flags: u32) -> u32 {
    let en = (enables[0] as u32) | ((enables[1] as u32) << 1) | ((enables[2] as u32) << 2);
    (hall_code as u32) | (en << 8) | ((method as u32) << 16) | ((flags & 0xFF) << 24)
}

// -------------------------------------------------------------------------------------------
// The hardware half.
// -------------------------------------------------------------------------------------------

#[cfg(target_os = "none")]
pub use hw::{bring_up, period_isr_hz};

#[cfg(target_os = "none")]
pub mod hw {
    use super::*;
    use board::{AdcInput, MotorPlan};
    use commutation::foc::PhaseOffsets;
    use commutation::Commutator;
    use core::ptr::addr_of_mut;
    use heapless::Vec;
    use runtime_hal::config::{
        AdcClockDiv, BreakConfig, ClockDiv, InjectedAdcConfig, InjectedChannel, OcMode, PwmAlign,
        PwmChannelConfig, PwmConfig, TimerTriggerLink, TrgoSource,
    };
    use runtime_hal::{
        clock, irq, Chip, InjectedAdcController, InjectedHandle, InputGroup, PeriphLabel,
        PwmHandle, PwmTimer, TriggeredAdc,
    };

    /// The PWM period: `commutation::ARR`, the crate that owns the constant (the stock 16 kHz
    /// centre-aligned contract at the fleet's 72 MHz timer clock).
    const PERIOD: u16 = commutation::ARR;
    /// PSC = 0: the timer runs from the timer clock undivided. Named here rather than written into
    /// `timer_config` as a literal, because [`period_isr_hz`] derives the ISR rate from it and the
    /// two must not be able to disagree.
    const PRESCALER: u16 = 0;

    /// The rate the period ISR runs at, derived from the timer configuration this module owns
    /// rather than written down as "16 kHz". Centre-aligned, so the counter crosses `PERIOD`
    /// twice per period: `timer_clock / ((PRESCALER + 1) * 2 * PERIOD)`.
    ///
    /// It is a real number the rest of the image needs, not a comment: the commutation front end's
    /// hall debounce window is a TIME, and the period count that expresses it can only be derived
    /// from this rate (`commutation::foc::HALL_DEBOUNCE_US`). At the fleet's 72 MHz timer clock
    /// this is 72e6 / (1 * 2 * 2250) = 16,000.
    pub const fn period_isr_hz(timer_clock_hz: u32) -> u32 {
        timer_clock_hz / ((PRESCALER as u32 + 1) * 2 * PERIOD as u32)
    }
    /// The ADC-trigger compare: 50 timer counts below the period, so the CH3 compare reference
    /// (the trigger, see `timer_config`'s `trgo_src`) rises 694 ns before the end-of-up-count
    /// low-current point and stays high across it. Re-armed every period.
    ///
    /// **The 50-count offset is a trigger-WIDTH requirement, not a sampling-instant choice** (F103
    /// master, 2026-07-26). The reference's own 2249 puts the compare one count below the period,
    /// making the trigger level 1 timer tick (13.9 ns) wide against an ADC clocked at APB2/6 =
    /// 12 MHz (83 ns); the injected group did not start at all. Swept on silicon within ONE boot,
    /// with the group otherwise configured exactly as it ships: 2249 (1 tick) produced no
    /// conversion, 2245 (5 ticks, 69 ns) converted, and every wider value converted.
    ///
    /// That sweep does not settle a hard threshold. Timer and ADC run from the same 72 MHz clock in
    /// a fixed 6:1 ratio, so a narrow pulse sits at a FIXED phase of the ADC clock within a boot,
    /// and the phase, set at PLL lock, can decide catch-or-miss per boot. One boot's sweep cannot
    /// separate "too narrow at any phase" from "wrong phase this boot", and per-boot phase is the
    /// model that also accounts for the diagnostic image's 4/4 conversions, its 0/3 re-run and the
    /// pristine image's 0/2. 50 counts (694 ns, over eight ADC clocks) is safe under either model,
    /// which is why it is the value here rather than a value just above 5 ticks. The F1x0's latch
    /// behavior is unmeasured; it converts at this width.
    ///
    /// 2200 is 1.1% of the period early, with the 7.5-cycle sample aperture still spanning the
    /// low-current point. Slices 4 and 6 (offset calibration, FOC) re-check the instant with
    /// current flowing.
    const TRIGGER_COMPARE: u16 = PERIOD - 50;
    /// CH3 output enable (`plan-hotpath-readiness.md` delta 4). SET for golden parity rather than
    /// to make the trigger work: it puts CHCTL2 at the stock golden's 0x1DDD rather than 0x0DDD.
    /// Nothing hands over to it. The trigger is the CH3 compare REFERENCE carried on TRGO (see
    /// `TRIGGER_COMPARE` and `timer_config`), which is taken before the output-enable gating, so
    /// the disarmed and armed configurations share one trigger path and stage 4 changes no trigger
    /// field.
    ///
    /// **Its deciding-bit role is superseded (silicon, 2026-07-26).** An earlier reading made this
    /// bit the reason the injected group did or did not convert, from a run with the CH3 link and
    /// CH3EN CLEAR. The session's own control refutes it: with the CH3 link selected and CH3EN SET,
    /// the group still converted on neither family, while TIMER0's CH3IF latched in INTF
    /// throughout. The settled finding is that the trigger LINK (the CTL1 bit-12 selection)
    /// decides, not the channel enable. The CH3 channel LINK (ETSIC = 001) rides the channel
    /// output, which MOE gates, so it produces nothing on a disarmed bridge; this path triggers
    /// from TRGO carrying the compare reference instead. See `specs/motor-integration.md`,
    /// bring-up step 3's mode-dependent-trigger paragraph.
    ///
    /// TIMER0_CH3's pin (PA11) is never routed to its alternate function here, so enabling the
    /// channel drives nothing off-chip.
    const TRIGGER_CH_ENABLE: bool = true;
    /// The injected sample-time code for 7.5 ADC cycles (`ADC_SAMPLETIME_7POINT5`), the stock
    /// inserted-group value on every phase-current channel.
    const SAMPLE_TIME_7P5: u8 = 1;

    /// Everything the period ISR touches, built once at bring-up. The ISR is its SOLE accessor
    /// after installation (the main thread installs it before the vector is unmasked and never
    /// looks at it again), which is what makes the `static mut` sound.
    struct MotorRuntime {
        pwm: PwmHandle,
        injected: InjectedHandle,
        halls: InputGroup,
        commutator: Commutator,
        /// The bring-up-row decode facts this motor was configured with ([`BootFixed`]): written
        /// once by the bring-up and never written again, so the arm-time re-derivation can rebuild
        /// the commutator records without re-reading the board plan.
        boot: BootFixed,
        /// The bring-up's flag bits (configured / current-sense), ORed into every published state.
        base_flags: u32,
        method: u8,
        /// Free-running period count (the ISR's own copy; published into [`PERIODS`]).
        periods: u32,
        /// Periods since the demand word was last freshly written.
        since_demand: u32,
        /// The last observed [`DEMAND_SEQ`].
        last_seq: u32,
        /// The fault bits accumulated so far (published into [`FAULT`]).
        faults: u32,
        /// The measured quiet-bridge zero offsets `(a, b)` the phase currents are read against,
        /// in [`cal_offsets`]'s accumulated unit.
        offsets: (u16, u16),
        /// The per-period current limit (soft chop, hard trip, observation window), built at
        /// bring-up from the boot-read `MOTOR_CURRENT_LIMIT` and re-derived at every arm through
        /// [`CurrentLimit::reconfigure`], which replaces the limit and keeps the observation
        /// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read").
        current: CurrentLimit,
    }

    /// The ISR's state. Written by the bring-up before the period vector is unmasked, and
    /// read/written by the period ISR afterwards.
    ///
    /// **The invariant, as of the arm-time re-read** (`specs/integration.md`, "When a stored value
    /// takes effect: the arm-time re-read"): the period ISR is the sole accessor WHENEVER THE PERIOD
    /// VECTOR CAN FIRE, and the 250 Hz arm path mutates the value set only where it cannot, i.e.
    /// inside [`install_rederived`]'s `cortex_m::interrupt::free`. It is not enough to arm while the
    /// counter is stopped: on the FIRST arm of a boot the counter is still running from the bring-up
    /// (nothing stopped it), so the ISR may be mid-period at that moment.
    ///
    /// In `.uninit` (cortex-m-rt's NOLOAD section), so the `None` below is NOT loaded at reset:
    /// left in `.data`, the static's whole init image (all zero but the niche byte that spells
    /// `None`) sat in flash. [`bring_up`] writes `None` on entry, and the only reader, the period
    /// ISR, can only be registered by `bring_up` after that, so no read ever sees reset garbage.
    #[link_section = ".uninit.MOTOR"]
    static mut MOTOR: Option<MotorRuntime> = None;

    /// The configured timer, as the 250 Hz thread's own handle on it. Written once by the bring-up
    /// on the boot thread; read afterwards only by [`start_counter`] / [`stop_counter`] /
    /// [`float_all_channels`], which run on the 250 Hz control task and nowhere else.
    ///
    /// **Two writers of `CHCTL2` exist by construction here, and the ordering is what makes that
    /// safe.** The period ISR writes the channel enables every period through its own
    /// [`PwmHandle`]; [`float_all_channels`] writes them from the shutdown sequence. They can only
    /// overlap inside one shutdown, where MOE has ALREADY been cleared (the sequence's first step),
    /// so neither write can reach a gate driver, and the step after floats is the counter stop that
    /// ends the ISR. The float is the posture, not the silencing act; the silencing act is MOE.
    static mut TIMER: Option<PwmTimer> = None;

    /// Start the counter (the arm sequence's first step). Idempotent: on the first arm of a boot
    /// the bring-up already started it.
    pub fn start_counter() {
        // SAFETY: read-only access to a static written once on the boot thread, from the 250 Hz
        // task, which is the only reader.
        if let Some(t) = unsafe { (*addr_of_mut!(TIMER)).as_ref() } {
            t.enable_counter();
            COUNTER_RUNNING.store(true, Ordering::Relaxed);
        }
    }

    /// Stop the counter (the shutdown sequence's last step). With MOE already clear this changes
    /// nothing electrically; what it does is end the period ISR, which is why the liveness
    /// supervisor's premise is cleared with it rather than after it.
    pub fn stop_counter() {
        // SAFETY: as `start_counter`.
        if let Some(t) = unsafe { (*addr_of_mut!(TIMER)).as_ref() } {
            COUNTER_RUNNING.store(false, Ordering::Relaxed);
            t.disable_counter();
        }
    }

    /// The bring-up-row decode facts this motor was configured with, or `None` on a board with no
    /// runtime. The arm path hands them to [`rederive`] so an arm-time rebuild of the commutator
    /// records uses the SAME direction and align offset the boot installed.
    ///
    /// Read under `cortex_m::interrupt::free` for the reason stated on [`MOTOR`]: the field itself
    /// is written once by the bring-up and never again, but forming a reference to the runtime at
    /// all has to exclude the period ISR. A handful of cycles, once per arm.
    pub fn boot_fixed() -> Option<BootFixed> {
        cortex_m::interrupt::free(|_| {
            // SAFETY: the period vector cannot fire inside this section, so the 250 Hz thread is
            // the only accessor of the runtime for its duration (the `MOTOR` invariant).
            unsafe { (*addr_of_mut!(MOTOR)).as_ref() }.map(|m| m.boot)
        })
    }

    /// Install an arm-time re-derivation into the period ISR's record: the method byte, the current
    /// limit, and the per-mode records through `Commutator::switch_method`. `false` on a board with
    /// no runtime (which is unarmable anyway, so the arm is refused).
    ///
    /// **Those three and nothing else.** It does not touch `offsets`, `base_flags`, `faults`,
    /// [`FAULT`] or [`OBS_CAL`], and [`Rederived`] carries no field that could. The limit goes in
    /// through [`CurrentLimit::reconfigure`] rather than as a built record, so the boot-cumulative
    /// trip count the ISR publishes survives an arm (see that seam for the split). The records go
    /// through `switch_method` precisely because that seam replaces the per-mode records and
    /// deliberately leaves the SHARED rotor front end alone, so the angle, the latched speed and
    /// the hall debounce history stay continuous across an arm.
    ///
    /// # Why the phase-offset calibration is NOT redone (the spec's second edge)
    ///
    /// An arm-time method change does not need the quiet-bridge offsets re-measured, so the arm is
    /// not refused on that account and the existing measured pair is carried through untouched. The
    /// offsets are the zero-current reading of the two sensed phases and their ADC path: a property
    /// of the SENSE CHAIN and of a current-free bridge, not of the commutation method. That is why
    /// [`BringUpStep::CalibratePhaseOffsets`] is unconditional and its own comment says the limit
    /// reads both sensed phases against these zeros whatever method runs.
    ///
    /// The consequence is the reason to state it rather than leave it implied: re-measuring at arm
    /// would be a PERIPHERAL measurement (16 timer-triggered conversions) in the value row, it would
    /// mean a quiet bridge the arm path cannot guarantee, and it could newly raise
    /// [`FAULT_INIT_CAL`] and so refuse an arm for a reason unrelated to the value that was written.
    ///
    /// # The critical section
    ///
    /// `cortex_m::interrupt::free`, because the period ISR is otherwise the sole accessor of this
    /// record and it may be MID-PERIOD here: on the first arm of a boot the counter has been
    /// running since the bring-up started it. The section is a few hundred cycles (a method byte,
    /// the limit seam's four field writes, and the records swap) against a 4,500-cycle period, so
    /// no conversion is lost:
    /// the injected end-of-conversion flag is a level source, so a period whose entry is delayed
    /// inside the section is served the moment it ends.
    pub fn install_rederived(r: &Rederived) -> bool {
        cortex_m::interrupt::free(|_| {
            // SAFETY: the period vector cannot fire inside this section, so the 250 Hz thread is
            // the only accessor of the runtime for its duration (the `MOTOR` invariant).
            match unsafe { (*addr_of_mut!(MOTOR)).as_mut() } {
                Some(m) => {
                    m.method = r.method;
                    m.current.reconfigure(r.limit_counts);
                    m.commutator.switch_method(r.records);
                    true
                }
                None => false,
            }
        })
    }

    /// Float every phase (the shutdown sequence's explicit coast posture). Not inferable from a
    /// zero demand across methods, so it is applied directly.
    pub fn float_all_channels() {
        // SAFETY: as `start_counter`.
        if let Some(t) = unsafe { (*addr_of_mut!(TIMER)).as_ref() } {
            t.handle().set_channel_outputs([false; 3]);
        }
    }

    /// What a successful bring-up hands back. One field, because there is exactly one thing the
    /// caller does with it (the method actually running and the injected channels programmed are
    /// published in the observation block, not returned: a field nothing reads is a field that
    /// cannot be wrong).
    #[derive(Clone, Copy, Debug)]
    pub struct MotorRuntimeSummary {
        /// The configured timer, handed over so [`crate::arm`] can derive its arming gate from it.
        /// This module never derives one: the gate is that module's alone, which is what keeps this
        /// one disarmed by construction (a host test scans the source for it).
        pub timer: PwmTimer,
    }

    /// Bring one motor up from its validated plan, disarmed (`BRING_UP_STEPS`, in order).
    ///
    /// Returns the summary on success, or the step that stopped it. Nothing here writes MOE: the
    /// arming gate is not built, not held, and not reachable from this module. The summary carries
    /// the configured timer so `crate::arm` can build one, which is the only route by which this
    /// board becomes armable at all.
    /// `period_hz` is the rate this bring-up's period ISR will run at ([`period_isr_hz`] of the
    /// configured timer clock). The commutator's hall debounce window is derived from it.
    /// `current_limit_ma` is the boot-read `MOTOR_CURRENT_LIMIT`, converted here by
    /// [`limit_counts`] against the plan's own `current_cal` into the ISR's record. A `CONFIG_WRITE`
    /// of either applies at the next ARM, which re-derives both through the same owners and
    /// installs the result (`specs/integration.md`, "When a stored value takes effect: the arm-time
    /// re-read"); this boot read is the first value the board runs on, not the only one.
    /// `vbatt` is the plan's battery-sense input: `Some` adds the battery rank to the injected
    /// group (rank 2, after the two phase ranks) and puts its pin in analog mode beside them.
    pub fn bring_up(
        chip: &Chip,
        plan: &MotorPlan,
        vbatt: Option<AdcInput>,
        method_byte: u8,
        period_hz: u32,
        current_limit_ma: u32,
    ) -> Result<MotorRuntimeSummary, MotorSkip> {
        // `MOTOR` lives in `.uninit` (see the static): give it the `None` its initializer names
        // before anything that could register its reader. `write`, not `=`, so the reset garbage
        // is never dropped.
        // SAFETY: the boot thread, before the period vector is registered or unmasked; no
        // reference formed.
        unsafe { addr_of_mut!(MOTOR).write(None) };
        // Step 1 of the spec's list, before the ordered steps: refuse an absent layout. A motor
        // with no gate set or no hall set is absent, which is a valid board state, not a fault.
        let (gates, halls) = match (plan.gates, plan.halls) {
            (Some(g), Some(h)) => (g, h),
            _ => return Err(MotorSkip::Absent),
        };
        let phase = plan.phase_current.ok_or(MotorSkip::NoCurrentSense)?;
        // The policy input, read once. What actually runs is `running_method`'s clamp, applied at
        // `SelectMethodAndInstall`.
        let requested = requested_method(method_byte);

        // The one configured timer object, built by `ConfigureTimer` and reused by every later
        // step that touches the timer (the trigger channel, the counter start, the per-cycle
        // handle): configuring it once is the bring-up, not a step that repeats.
        let mut timer: Option<PwmTimer> = None;
        let mut injected = None;
        // The calibration's outcome and the offsets the ISR reads currents against, set by
        // `CalibratePhaseOffsets`.
        let mut cal: Option<(CalOutcome, (u16, u16))> = None;

        for step in BRING_UP_STEPS {
            let failed = |s: BringUpStep| MotorSkip::StepFailed(s);
            match step {
                BringUpStep::EnableTimerClock => {
                    let rcu = chip.rcu_base().map_err(|_| failed(step))?;
                    clock::enable_timer(rcu, chip.clock(), PeriphLabel::Timer0)
                        .map_err(|_| failed(step))?;
                }
                BringUpStep::ConfigureTimer => {
                    timer = Some(
                        PwmTimer::configure(chip, &timer_config(&gates))
                            .map_err(|_| failed(step))?,
                    );
                }
                BringUpStep::ConfigureTriggerChannel => {
                    // The trigger channel + TRGO are a separate step on the SAME configured timer
                    // (the HAL keeps them out of `configure` so the channel golden stays
                    // CH3-untouched).
                    timer.ok_or(failed(step))?.configure_trigger(
                        TRIGGER_COMPARE,
                        OcMode::Pwm1,
                        TRIGGER_CH_ENABLE,
                        // The same TRGO source `timer_config` carries, for the same reason: the
                        // CH3 compare reference, which the F10x ADC latches where it misses the
                        // update event's one-cycle pulse.
                        TrgoSource::Ch3Compare,
                    );
                }
                BringUpStep::RouteGatePins => {
                    for pin in gates.hi.iter().chain(gates.lo.iter()) {
                        chip.route_advanced_pwm_pin(pin.packed())
                            .map_err(|_| failed(step))?;
                    }
                }
                BringUpStep::EnableAdcClock => {
                    let rcu = chip.rcu_base().map_err(|_| failed(step))?;
                    clock::enable_adc(rcu, chip.clock(), PeriphLabel::Adc0)
                        .map_err(|_| failed(step))?;
                }
                BringUpStep::ConfigurePhasePins => {
                    // The battery-sense pin rides with the phase pins: every analog input the
                    // injected group converts is put in analog mode by this one step.
                    for pin in phase.pins.iter().chain(vbatt.as_ref().map(|v| &v.pin)) {
                        let port = match pin.port() {
                            0 => PeriphLabel::Gpioa,
                            1 => PeriphLabel::Gpiob,
                            2 => PeriphLabel::Gpioc,
                            _ => return Err(failed(step)),
                        };
                        chip.analog_pin(port, pin.pin()).map_err(|_| failed(step))?;
                    }
                }
                BringUpStep::ConfigureInjectedGroup => {
                    let handle = InjectedAdcController::new()
                        .configure(
                            chip,
                            &injected_config(&injected_ranks(
                                phase.channels,
                                vbatt.map(|v| v.channel),
                            )),
                        )
                        .map_err(|_| failed(step))?;
                    injected = Some(handle);
                }
                BringUpStep::StartCounter => {
                    let t = timer.ok_or(failed(step))?;
                    t.enable_counter();
                    COUNTER_RUNNING.store(true, Ordering::Relaxed);
                    // SAFETY: the one write, on the boot thread, before the scheduler exists and
                    // therefore before any 250 Hz reader does.
                    unsafe { *addr_of_mut!(TIMER) = Some(t) };
                }
                BringUpStep::ConfirmTriggerStart => {
                    let inj = injected.ok_or(failed(step))?;
                    if !confirm_trigger_start(&inj) {
                        return Err(failed(step));
                    }
                }
                BringUpStep::CalibratePhaseOffsets => {
                    // Unconditional (`specs/motor-integration.md`, "The current limit"): the limit
                    // reads both sensed phases against these zeros whatever method runs. A
                    // refusal does NOT fail the step: the motor still comes up, on six-step,
                    // carrying the init-failure fault, which keeps it from arming.
                    let inj = injected.ok_or(failed(step))?;
                    cal = Some(match measure_offsets(&inj) {
                        Some((a, b)) => {
                            OBS_CAL.store((a as u32) | ((b as u32) << 16), Ordering::Relaxed);
                            // The acceptance window's single owner is the commutation crate's
                            // gated newtype, so the check is ITS constructor, never a copy of
                            // its constants here.
                            let outcome = if PhaseOffsets::try_new(a, b).is_some() {
                                CalOutcome::Accepted
                            } else {
                                CalOutcome::Refused
                            };
                            (outcome, (a, b))
                        }
                        // No conversion inside the poll budget: no trustworthy zero-current
                        // reference, and OBS_CAL stays zero to say the measurement itself never
                        // happened.
                        None => (CalOutcome::Refused, (0, 0)),
                    });
                }
                BringUpStep::SelectMethodAndInstall => {
                    let (pwm, inj, (cal, offsets)) = match (timer, injected, cal) {
                        (Some(t), Some(i), Some(c)) => (t.handle(), i, c),
                        _ => return Err(failed(step)),
                    };
                    let group = chip
                        .input_group([halls.a.packed(), halls.b.packed(), halls.c.packed()])
                        .map_err(|_| failed(step))?;
                    let method = running_method(requested);
                    // The two boot-fixed decode facts, kept on the runtime so the arm-time
                    // re-derivation rebuilds the records from the SAME direction and offset rather
                    // than re-reading the plan (`specs/integration.md`, the arm-time re-read).
                    let boot = BootFixed {
                        direction: plan.direction,
                        align_offset: plan.align_offset,
                    };
                    // One construction shape for the records, shared with `motor::rederive`.
                    let records = six_step_records(boot);
                    let base_flags = OBS_CONFIGURED
                        | OBS_CURRENT_SENSE
                        | if cal == CalOutcome::Accepted {
                            OBS_CAL_ACCEPTED
                        } else {
                            0
                        };
                    // The init-failure fault bits, published here and SEEDED into the ISR's own
                    // accumulator: the ISR stores its accumulator over `FAULT` every period, so an
                    // init fault the ISR did not know about would be erased on the first entry.
                    // Seeding it keeps the ISR the sole writer of `FAULT` after the unmask while
                    // still carrying a fault raised before it.
                    let init_faults = init_fault_bits(cal);
                    FAULT.store(init_faults, Ordering::Relaxed);
                    // SAFETY: the one write, on the boot thread, BEFORE the period vector is
                    // unmasked (the next-but-one step), so no ISR can observe it half-built.
                    unsafe {
                        *addr_of_mut!(MOTOR) = Some(MotorRuntime {
                            pwm,
                            injected: inj,
                            halls: group,
                            commutator: Commutator::new(records, period_hz),
                            boot,
                            base_flags,
                            method: method.to_u8(),
                            periods: 0,
                            since_demand: 0,
                            last_seq: DEMAND_SEQ.load(Ordering::Relaxed),
                            faults: init_faults,
                            offsets,
                            current: CurrentLimit::new(limit_counts(
                                current_limit_ma,
                                plan.current_cal,
                            )),
                        });
                    }
                    OBS_STATE.store(
                        pack_obs_state(0, [false; 3], method.to_u8(), base_flags),
                        Ordering::Relaxed,
                    );
                }
                BringUpStep::RegisterPeriodHandler => irq::register_control_handler(period_isr),
                BringUpStep::EnablePeriodVector => {
                    let (period_irq, _, _) = irq::motor_era_irqs(chip.irq());
                    irq::unmask_irq(period_irq);
                }
            }
        }

        Ok(MotorRuntimeSummary {
            timer: timer.ok_or(MotorSkip::StepFailed(BringUpStep::ConfigureTimer))?,
        })
    }

    /// Re-assert the injected group's external-trigger enable until the group actually converts.
    /// True once the injected end-of-conversion flag sets, i.e. once a timer-triggered conversion
    /// has completed; false if it never does within
    /// [`TRIGGER_START_ATTEMPTS`] x [`TRIGGER_START_SPINS`].
    ///
    /// The counter is already running when this runs, so the trigger event is firing; what is in
    /// question is only whether the injected group responds to it. Each attempt clears any stale
    /// EOIC (so the flag observed is a NEW conversion, not the routine one the ADCON re-assert in
    /// the HAL's injected bring-up leaves behind) and then re-asserts ETEIC, exactly the pair the
    /// stock firmware performs every period.
    ///
    /// A successful confirm deliberately leaves EOIC SET: the period vector is unmasked two steps
    /// later, so the flag simply makes the first ISR entry immediate, and that ISR clears it as its
    /// first act.
    fn confirm_trigger_start(injected: &InjectedHandle) -> bool {
        (0..TRIGGER_START_ATTEMPTS).any(|_| await_conversion(injected))
    }

    /// Wait for ONE fresh timer-triggered injected conversion: clear the end-of-conversion flag,
    /// re-assert the external-trigger enable, then poll the flag back up. The single owner of that
    /// sequence, shared by the bring-up confirm and the offset calibration, because it is the same
    /// question both ask ("has a conversion completed since I looked?") and the same stock pair
    /// the period ISR performs.
    ///
    /// Leaves the flag SET on success, which is what the callers want: the confirm hands it to the
    /// next calibration read, and the last calibration read hands it to the first ISR entry.
    fn await_conversion(injected: &InjectedHandle) -> bool {
        injected.clear_eoic();
        injected.reassert_trigger_enable();
        let mut spins = TRIGGER_START_SPINS;
        while spins > 0 {
            if injected.eoic() {
                return true;
            }
            spins -= 1;
        }
        false
    }

    /// Measure the quiet-bridge phase-current zero offsets: [`CAL_SAMPLES`] timer-triggered
    /// conversions per channel, accumulated by [`cal_offsets`] into the offset unit the acceptance
    /// window is expressed in. `None` if any conversion failed to complete inside the poll budget.
    ///
    /// The bridge is quiet by construction, not by convention: MOE is never written in this crate,
    /// so no phase can be driven while this runs. The two ranks are sampled from the SAME
    /// conversion, so both offsets come from one instant of the same period.
    fn measure_offsets(injected: &InjectedHandle) -> Option<(u16, u16)> {
        let mut run_a = [0u16; CAL_SAMPLES];
        let mut run_b = [0u16; CAL_SAMPLES];
        for (sa, sb) in run_a.iter_mut().zip(run_b.iter_mut()) {
            if !await_conversion(injected) {
                return None;
            }
            let s = injected.read_injected();
            *sa = s[0];
            *sb = s[1];
        }
        Some(cal_offsets(&run_a, &run_b))
    }

    /// The reconciled timer configuration (`specs/motor-integration.md` bring-up step 3 =
    /// `plan-hotpath-readiness.md` section 1's reference column, so the step list and the stage-2
    /// golden agree by construction).
    fn timer_config(gates: &board::GateSet) -> PwmConfig {
        let ch = |i: usize| PwmChannelConfig {
            high: gates.hi[i].packed(),
            low: gates.lo[i].packed(),
            // The complementary (low) side is inverted, active-low: the reference's polarity.
            polarity: true,
            // Idle levels: N-side only high (the golden's CTL1 0x2A20), so a disarmed bridge idles
            // safe on the low side. NOT both sides high (0x3F20), which fails the golden diff.
            idle_high: false,
            idle_high_n: true,
        };
        PwmConfig {
            timer: PeriphLabel::Timer0,
            channels: [ch(0), ch(1), ch(2)],
            period: PERIOD,
            // The timer clock undivided, which is what makes 72 MHz / (2 x 2250) = 16 kHz.
            // `period_isr_hz` derives that rate from these same two constants.
            prescaler: PRESCALER,
            // Per-board data, never a crate constant (`specs/commutation.md`).
            dead_time: gates.dead_time,
            // Stock parity: the hardware break input is deliberately disabled (over-current
            // protection is software's, out of scope here; the bench current limit is the cap).
            brk: BreakConfig {
                enabled: false,
                level: false,
            },
            trigger_compare: TRIGGER_COMPARE,
            // Delta 1: centre-aligned mode 1 (CAM = 01).
            align: PwmAlign::Center1,
            // Delta 2: auto-reload shadow OFF (CAR is constant after bring-up; the golden diffs it).
            arse: false,
            // Delta 3: the trigger channel's compare-match lands at the end-of-up-count point.
            trigger_oc_mode: OcMode::Pwm1,
            // Delta 4: resolved on silicon by G-EOC.
            trigger_ch_enable: TRIGGER_CH_ENABLE,
            // Delta 5, load-bearing: with centre-aligned counting CREP = 1 is ONE update event per
            // full up+down period; CREP = 0 doubles the update/TRGO rate to 32 kHz and the whole
            // 4500-cycle budget is wrong.
            crep: 1,
            ckdiv: ClockDiv::Div2,
            // TRGO carries the CH3 COMPARE REFERENCE (MMC = 111, O3CPRE), not the update event.
            // Measured on the F103 master (2026-07-26): the update event's TRGO is a single
            // timer-clock pulse (13.9 ns at 72 MHz) and this family's ADC, clocked at APB2/6 =
            // 12 MHz, does not latch it -- the injected group never starts, from a configuration
            // whose every register reads correct. Proof, on one boot, in this order: a
            // software-started injected conversion converts both ranks fine; TRGO on update
            // produces nothing over 300 ms with fresh ETEIC edges, a fresh ADC wake and a full
            // re-calibration; switching MMC to a LEVEL source triggers the group immediately.
            // O3CPRE is the CH3 compare's internal reference, so with `trigger_compare` = 2249 in
            // PWM1 mode it rises exactly at the end-of-up-count sampling instant, once per
            // centre-aligned period. It is taken BEFORE the output-enable gating, which is why it
            // works while disarmed where the CH3 channel link (ADC ETSIC = 001, what stock uses)
            // produces nothing: that one rides the channel OUTPUT, which MOE gates.
            trgo_src: TrgoSource::Ch3Compare,
        }
    }

    /// The injected-group configuration over [`injected_ranks`]: two slots, or three with the
    /// battery rank, never the stock four (the aux consumer does not exist here, so populating it
    /// would model samples nothing reads). 7.5 cycles on every rank.
    fn injected_config(ranks: &[u8]) -> InjectedAdcConfig {
        let mut chans: Vec<InjectedChannel, 4> = Vec::new();
        for &channel in ranks.iter() {
            // Cannot overflow: `injected_ranks` yields at most three into a capacity-4 vector.
            let _ = chans.push(InjectedChannel {
                channel,
                sample_time: SAMPLE_TIME_7P5,
            });
        }
        InjectedAdcConfig {
            adc: PeriphLabel::Adc0,
            channels: chans,
            left_aligned: true,
            trigger_timer: PeriphLabel::Timer0,
            // **TRGO, not CH3, and this is a silicon finding (2026-07-26).** G-EOC ran the
            // readiness plan's prescribed order on the F103 master with every other link
            // verifiably right (the ADC converts: a software-started inserted conversion filled
            // both IDATA ranks and drove one full period through this ISR) and the CH3-compare
            // link produced NO conversion in either CH3EN state, while TRGO produced them at the
            // PWM rate immediately. The stock firmware corroborates the mechanism: its recovered
            // arm-time op SWITCHES ADC_CTL1's trigger select to CH3 (`|= 0x1000`) and its disarm
            // op switches it back (`&= ~0x1000`), i.e. stock also runs TRGO while DISARMED. The
            // reading that fits both: the CH3-sourced trigger rides the channel's OUTPUT, which
            // MOE gates, so it cannot exist on a disarmed bridge. Stage 4 (first energize) is
            // where the CH3 link becomes available and where the sampling instant moves to the
            // end-of-up-count point; until then TRGO's update event is the trigger, and with
            // CREP = 1 that is exactly one conversion per full centre-aligned period (measured
            // 16.4 kHz, not the 32 kHz CREP = 0 would give).
            trigger_link: TimerTriggerLink::Trgo,
            clock_div: AdcClockDiv::Div6,
        }
    }

    /// The period ISR (16 kHz, the highest-priority code in the image). Registered on the HAL's
    /// control-handler seam, which the RAM vector table's ADC injected-EOC slot routes to.
    ///
    /// The body is the spec's hot path verbatim, and the write order is load-bearing: range-check
    /// and write the compares FIRST, then gate the outputs, so a rejected duty cannot change which
    /// phases drive.
    /// Linked into `.hotcode`, at the bottom of flash: on the GD32F1x0 only the first 32 KiB of
    /// flash is zero-wait, and this ISR measured 17,166 cycles per period above that line against
    /// **596 below it** (F103, same binary: 600). `crates/firmware/memory.x` owns the placement and
    /// asserts the window at link time.
    #[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
    extern "C" fn period_isr() {
        // SAFETY: the ISR is the sole accessor of MOTOR once the vector is unmasked; the bring-up's
        // single write happens strictly before that unmask, on the boot thread.
        let Some(m) = (unsafe { (*addr_of_mut!(MOTOR)).as_mut() }) else {
            return;
        };
        // FIRST: clear the injected end-of-conversion flag. It is a level source into the NVIC and
        // reading IDATAx does not clear it, so this is what makes the vector fire once per period
        // instead of re-entering forever.
        m.injected.clear_eoic();
        // THEN re-assert the injected group's external-trigger enable, stock parity: stock's
        // per-period op is exactly this pair (clear EOIC, `ADC_CTL1 |= 0x8000`), which is what
        // keeps the enable a maintained fact rather than a one-shot arming for as long as the ISR
        // runs. The bring-up's start-confirm is the other half, and it is the half that fixes the
        // POR race, since a group that never converts never reaches this line.
        m.injected.reassert_trigger_enable();

        let samples = m.injected.read_injected();
        // The battery rank's count (rank 2), reduced once to 12 bits and handed to the 250 Hz task.
        // Not in this period's decision; the ISR owns it only because it owns the ADC. With no
        // battery rank configured IDATA2 is never written and reads its reset 0.
        VBATT_RAW.store((samples[2] >> 3) as u32, Ordering::Relaxed);
        // The phase-current magnitude, this period, from the same two samples (the stock unit,
        // against the bring-up's measured zeros), and the current limit's verdict on it.
        let mag = phase_magnitude(
            commutation::foc::current_from_adc(m.offsets.0, samples[0]),
            commutation::foc::current_from_adc(m.offsets.1, samples[1]),
        );
        let current = m.current.step(mag);
        if current.trip {
            OVER_CURRENT_TRIPS.store(m.current.trips(), Ordering::Relaxed);
        }
        let code = m.halls.read();
        let levels = [code & 1, (code >> 1) & 1, (code >> 2) & 1];

        // Demand freshness: count periods since the 250 Hz task last wrote.
        let seq = DEMAND_SEQ.load(Ordering::Relaxed);
        if seq != m.last_seq {
            m.last_seq = seq;
            m.since_demand = 0;
        } else {
            m.since_demand = m.since_demand.saturating_add(1);
        }
        let demand = DEMAND.load(Ordering::Relaxed);

        let out = m.commutator.step(levels, (samples[0], samples[1]), demand);
        let (duties, enables) = out.to_duties_enables();

        let stale = demand_stale(m.since_demand);
        let applied_enables = if stale || current.chop {
            // The explicit coast posture: all phases float regardless of method. It cannot be
            // inferred from a zero demand (only six-step coasts at zero demand; sine holds
            // mid-rail and FOC holds ~1125 on every phase), so the guard applies it directly, and
            // it applies it BEFORE any duty write so silencing never depends on one.
            //
            // Two guards own it. The demand-freshness guard (a stalled 250 Hz task), and the soft
            // current limit, for this period only (the next re-evaluates from its own sample).
            // Both act on the channel enables and never on MOE: the bridge stays armed and every
            // phase floats, the current decaying through the body diodes.
            m.pwm.set_channel_outputs([false; 3]);
            if stale {
                m.faults |= FAULT_DEMAND_STALE;
            }
            [false; 3]
        } else if m.pwm.set_duties(duties).is_ok() {
            m.pwm.set_channel_outputs(enables);
            enables
        } else {
            // Refused duty: leave the outputs exactly as they were (the write-order rule).
            m.faults |= FAULT_DUTY_RANGE;
            enables
        };
        // Re-arm the ADC trigger compare so the next period samples at the same instant.
        let _ = m.pwm.rearm_trigger(TRIGGER_COMPARE);

        // Publish the handoff + observation words (this ISR is the sole writer of each).
        let comm = &m.commutator.front().comm;
        if comm.hall_fault {
            m.faults |= FAULT_HALL;
        }
        m.periods = m.periods.wrapping_add(1);
        PERIODS.store(m.periods, Ordering::Relaxed);
        // The current observation: one whole 64-period window per publish, so a 250 Hz reader
        // never sees a half-built peak.
        if m.periods % PERIODS_PER_TICK_NOMINAL == 0 {
            OBS_CURRENT.store(m.current.take_window(), Ordering::Relaxed);
        }
        ANGLE.store(comm.angle as u32, Ordering::Relaxed);
        SPEED.store(comm.speed, Ordering::Relaxed);
        INVALID_DWELL.store(comm.invalid_dwell, Ordering::Relaxed);
        FAULT.store(m.faults, Ordering::Relaxed);
        let flags = m.base_flags | if stale { OBS_COASTING } else { 0 };
        OBS_STATE.store(
            pack_obs_state(comm.prev_any_code, applied_enables, m.method, flags),
            Ordering::Relaxed,
        );
        OBS_DUTY01.store(
            (duties[0] as u32) | ((duties[1] as u32) << 16),
            Ordering::Relaxed,
        );
        OBS_DUTY2_ANGLE.store(
            (duties[2] as u32) | ((comm.angle as u32) << 16),
            Ordering::Relaxed,
        );
    }
}

// -------------------------------------------------------------------------------------------
// Host tests (the pure surfaces + the structural disarmed-by-construction check).
// -------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// This board's counts per amp, as every test below converts against it: the registered
    /// default of `store::MOTOR_CURRENT_CAL` (0x67), which is where the scale now lives. Taken
    /// from the handle, so a changed default shows up in these tests rather than drifting past them.
    const CAL: u16 = store::MOTOR_CURRENT_CAL.default();

    /// The injected group's rank list (bring-up step 5): the two-slot group where `board.vbatt` is
    /// absent (the current-limit slice's expectation, unchanged), and the three-slot group where it
    /// is present, the battery channel as rank 2 behind the two phase ranks the current limit
    /// reads. The bench pairs' channels: F103 master PB0/PA0 = 8/0 with PA4 = 4; F130 slave
    /// PB0/PB1 = 8/9.
    #[test]
    fn injected_ranks_two_slots_or_three_with_the_battery() {
        assert_eq!(injected_ranks([8, 9], None).as_slice(), &[8, 9]);
        assert_eq!(injected_ranks([8, 0], Some(4)).as_slice(), &[8, 0, 4]);
        // The phase ranks keep their positions either way.
        let three = injected_ranks([8, 0], Some(4));
        assert_eq!(&three[..2], injected_ranks([8, 0], None).as_slice());
    }

    /// The freshness guard's boundary, exactly as the spec states it: it does NOT fire at 256
    /// periods and DOES fire at 257.
    #[test]
    fn demand_freshness_boundary() {
        assert!(!demand_stale(0));
        assert!(!demand_stale(DEMAND_STALE_PERIODS - 1));
        assert!(!demand_stale(DEMAND_STALE_PERIODS), "not at 256");
        assert!(demand_stale(DEMAND_STALE_PERIODS + 1), "fires at 257");
        assert!(demand_stale(u32::MAX));
    }

    /// 256 periods at 16 kHz is 16 ms: three to four missed 4 ms writes, and ~1/30 of the 500 ms
    /// IWDG window, so the guard silences the bridge long before a watchdog reset would.
    #[test]
    fn demand_stale_window_is_16ms_at_16khz() {
        let ms = (DEMAND_STALE_PERIODS * 1000) / (PERIODS_PER_TICK_NOMINAL * 250);
        assert_eq!(ms, 16);
        assert!(ms * 30 <= 500, "well inside the IWDG window");
    }

    /// The liveness guard over synthetic counters: a stopped ISR (or a badly short advance) is not
    /// live; the nominal 64/tick and anything at or above half of it is.
    #[test]
    fn period_liveness_over_synthetic_counters() {
        assert!(!periods_live(0), "a stopped ISR");
        assert!(!periods_live(PERIODS_PER_TICK_MIN - 1));
        assert!(periods_live(PERIODS_PER_TICK_MIN));
        assert!(periods_live(PERIODS_PER_TICK_NOMINAL));
        assert!(periods_live(PERIODS_PER_TICK_NOMINAL * 2), "a late tick");
    }

    /// The bring-up ordering invariants the spec marks load-bearing, over the step list as data.
    #[test]
    fn bring_up_step_order() {
        let idx = |s: BringUpStep| BRING_UP_STEPS.iter().position(|x| *x == s).unwrap();
        // The clock before ANY timer register write (an unclocked advanced timer swallows writes).
        assert!(idx(BringUpStep::EnableTimerClock) < idx(BringUpStep::ConfigureTimer));
        assert!(idx(BringUpStep::EnableTimerClock) < idx(BringUpStep::ConfigureTriggerChannel));
        // The timer configured (idle levels, dead-time, off-states) BEFORE the gate pins leave
        // input mode, so they take the configured idle levels immediately.
        assert!(idx(BringUpStep::ConfigureTimer) < idx(BringUpStep::RouteGatePins));
        // The ADC clock before the injected group's registers, and the analog pin mode with it.
        assert!(idx(BringUpStep::EnableAdcClock) < idx(BringUpStep::ConfigureInjectedGroup));
        assert!(idx(BringUpStep::ConfigurePhasePins) < idx(BringUpStep::ConfigureInjectedGroup));
        // The calibration reads conversions, so it runs AFTER the confirm has proven they are
        // happening, and before the record construction its outcome feeds.
        assert!(idx(BringUpStep::ConfirmTriggerStart) < idx(BringUpStep::CalibratePhaseOffsets));
        assert!(idx(BringUpStep::CalibratePhaseOffsets) < idx(BringUpStep::SelectMethodAndInstall));
        // The injected group programmed AND the counter running before the trigger start-confirm:
        // the confirm re-asserts ETEIC and waits for a conversion, and the trigger event only
        // exists once the counter runs, so a confirm ordered any earlier could never see one.
        assert!(idx(BringUpStep::ConfigureInjectedGroup) < idx(BringUpStep::ConfirmTriggerStart));
        assert!(idx(BringUpStep::StartCounter) < idx(BringUpStep::ConfirmTriggerStart));
        // ...and the confirm before the vector is unmasked, so a board whose trigger never starts
        // is recorded as a failed step instead of arriving at an ISR that will never be entered.
        assert!(idx(BringUpStep::ConfirmTriggerStart) < idx(BringUpStep::EnablePeriodVector));
        // The runtime installed before the handler is registered, and the vector unmasked LAST:
        // the ISR clears a LEVEL flag through the installed handle, so an earlier unmask would
        // re-enter forever.
        assert!(idx(BringUpStep::SelectMethodAndInstall) < idx(BringUpStep::RegisterPeriodHandler));
        assert_eq!(
            *BRING_UP_STEPS.last().unwrap(),
            BringUpStep::EnablePeriodVector
        );
        // Every step appears exactly once.
        for s in BRING_UP_STEPS {
            assert_eq!(BRING_UP_STEPS.iter().filter(|x| **x == s).count(), 1);
        }
    }

    /// The trigger start-confirm's budget: each attempt waits many PWM periods (so a slow start is
    /// caught rather than raced past), and the worst case, paid only by a board whose trigger never
    /// starts, stays far inside the 500 ms IWDG window.
    #[test]
    fn trigger_start_confirm_budget() {
        // A volatile-read poll iteration costs at least ~3 cycles and, on the slower family's
        // flash fetch, no more than ~16. 4500 cycles is one 16 kHz period at 72 MHz.
        let periods_per_attempt = (TRIGGER_START_SPINS as u64 * 3) / 4500;
        assert!(
            periods_per_attempt >= 10,
            "each attempt must span several periods, spans {periods_per_attempt}"
        );
        let worst_ms =
            (TRIGGER_START_ATTEMPTS as u64 * TRIGGER_START_SPINS as u64 * 16 * 1000) / 72_000_000;
        assert!(
            worst_ms < 100,
            "a dead trigger costs {worst_ms} ms, and the IWDG window is 500 ms"
        );
    }

    /// MOE is never set during init, structurally: the step vocabulary has no arming step, so no
    /// ordering of it can energize the bridge. Asserted over the whole vocabulary (a new variant
    /// that armed would have to be added here).
    #[test]
    fn no_bring_up_step_can_arm() {
        for s in BRING_UP_STEPS {
            assert!(
                matches!(
                    s,
                    BringUpStep::EnableTimerClock
                        | BringUpStep::ConfigureTimer
                        | BringUpStep::ConfigureTriggerChannel
                        | BringUpStep::RouteGatePins
                        | BringUpStep::EnableAdcClock
                        | BringUpStep::ConfigurePhasePins
                        | BringUpStep::ConfigureInjectedGroup
                        | BringUpStep::StartCounter
                        | BringUpStep::ConfirmTriggerStart
                        | BringUpStep::CalibratePhaseOffsets
                        | BringUpStep::SelectMethodAndInstall
                        | BringUpStep::RegisterPeriodHandler
                        | BringUpStep::EnablePeriodVector
                ),
                "an unexpected bring-up step exists: {s:?}"
            );
        }
    }

    /// Method selection in this slice: six-step for every byte. Sine (slice 6) and FOC (slice 7)
    /// are not built, so a board configured for either runs six-step and READS BACK as six-step in
    /// the observation block, rather than silently claiming a method it is not running.
    #[test]
    fn method_selection_is_six_step_until_its_own_slice() {
        use commutation::CommutationMethod as M;
        for byte in [0u8, 1, 2, 99] {
            assert_eq!(
                running_method(requested_method(byte)),
                M::SixStep,
                "byte {byte}: sine is slice 6 and FOC is slice 7"
            );
        }
    }

    /// A board's boot-fixed decode facts, as a bring-up would have built them.
    const BOOT: BootFixed = BootFixed {
        direction: false,
        align_offset: 2,
    };

    /// **The current limit is recomputed at arm, through `limit_counts`.** Both inputs move it, and
    /// both clamp ends are the conversion's own (the milliamp ceiling, the calibration seam range,
    /// and the count floor).
    #[test]
    fn the_current_limit_is_reconverted_at_arm() {
        // The re-derivation carries the COUNT (see `Rederived::limit_counts`), and the count is
        // `limit_counts`'s, so a changed calibration moves it...
        let at =
            |ma: u32, cal: u16| rederive(0, ma, cal, BOOT).limit_counts == limit_counts(ma, cal);
        assert_ne!(
            limit_counts(20_000, 200),
            limit_counts(20_000, 400),
            "the per-board calibration is part of the conversion"
        );
        assert!(at(20_000, 200) && at(20_000, 400));
        // ...and so does a changed milliamp limit.
        assert_ne!(limit_counts(10_000, 455), limit_counts(20_000, 455));
        assert!(at(10_000, 455) && at(20_000, 455));
        // The clamp ends, through the same owner: a tiny request floors at MIN_LIMIT_COUNTS, a
        // calibration outside the seam clamps into it, and the milliamp ceiling saturates.
        assert_eq!(rederive(0, 1, 455, BOOT).limit_counts, MIN_LIMIT_COUNTS);
        assert_eq!(
            rederive(0, 20_000, 0, BOOT).limit_counts,
            limit_counts(20_000, CURRENT_CAL_MIN)
        );
        assert_eq!(
            rederive(0, 20_000, u16::MAX, BOOT).limit_counts,
            limit_counts(20_000, CURRENT_CAL_MAX)
        );
        assert_eq!(
            rederive(0, u32::MAX, 455, BOOT).limit_counts,
            limit_counts(CURRENT_LIMIT_CEILING_MA, 455)
        );
        // And the count the re-derivation carries IS the limit in force once installed: a record
        // reconfigured to it behaves exactly as one built with it (the install's own seam below).
        for (ma, cal) in [(10_000u32, 455u16), (20_000, 200), (1, 455)] {
            let counts = rederive(0, ma, cal, BOOT).limit_counts;
            let mut reconfigured = CurrentLimit::new(MIN_LIMIT_COUNTS);
            reconfigured.reconfigure(counts);
            let fresh = CurrentLimit::new(counts);
            assert_eq!(reconfigured, fresh, "ma {ma}, cal {cal}");
        }
    }

    /// **The arm-time limit seam replaces the CONFIGURATION and keeps the boot-cumulative
    /// OBSERVATION** (`specs/integration.md`, the arm-time re-read; the split is on
    /// [`CurrentLimit::reconfigure`]). `trips` is published through [`OVER_CURRENT_TRIPS`] and into
    /// `CTRL_OBS` word 31, whose counters are boot-cumulative, so a re-arm must not make it step
    /// back.
    #[test]
    fn a_reconfigured_limit_keeps_its_trip_count_and_takes_the_new_limit() {
        let lim = 4_000i16;
        let mut c = CurrentLimit::new(lim);
        // A hard trip, so there is a count and a window to preserve.
        assert!(c.step(hard_trip_counts(lim)).trip);
        assert_eq!(c.trips(), 1);
        // Re-arm with a different limit.
        let fresh = 8_000i16;
        c.reconfigure(fresh);
        assert_eq!(
            c.trips(),
            1,
            "the boot-cumulative trip count survives the arm"
        );
        // The NEW limit is the one in force, and so is the new hard trip, both observable through
        // `step`: a magnitude between the old limit and the new one no longer chops, and the new
        // hard trip is the one that trips.
        assert!(!c.step(fresh).chop, "at the new limit: drives");
        assert!(
            c.step(fresh + 1).chop,
            "one count over the new limit: floats"
        );
        let v = c.step(hard_trip_counts(fresh));
        assert!(v.trip, "the new hard trip is in force");
        assert_eq!(c.trips(), 2, "and it counts ON TOP of the old count");
        // The window's running maximum survived too (it is the same class of observation): the
        // peak is the largest magnitude seen since the last window close, across the re-arm.
        let packed = c.take_window();
        assert_eq!(
            packed,
            pack_motor_current(hard_trip_counts(fresh), 3, 2),
            "the open window carried across the reconfigure"
        );
    }

    /// **The over-current RE-LATCH survives an arm**, which is the reason the configuration /
    /// observation split exists at all (`specs/motor-integration.md`, "The hard trip": "a condition
    /// that persists through the OFF dwell (a real short) re-latches on the first period after the
    /// next arm").
    ///
    /// The latch is driven by a CHANGE of the published count, not by its value: the ISR writes
    /// [`OVER_CURRENT_TRIPS`] only on a trip, and the 250 Hz task raises motor 0's latch when that
    /// word differs from the count it last saw (`main.rs`, `if trips != shell.last_trips`). So a
    /// re-arm that restarted the count would republish 1 against a `last_trips` already holding 1,
    /// the change would never happen, and the real short would energize the bridge with no latch.
    /// Here the count goes 1 -> 2 across the arm, so the edge the latch consumes is there.
    ///
    /// Both halves of the seam are load-bearing in this test: `trips` surviving is what makes the
    /// second trip a CHANGE rather than a repeat, and the episode reset is what lets the first
    /// period after the arm trip at all.
    #[test]
    fn the_over_current_relatch_survives_an_arm() {
        let lim = 4_000i16;
        let mut c = CurrentLimit::new(lim);
        // The run before the arm: a short trips once, and the ISR publishes 1.
        assert!(c.step(hard_trip_counts(lim)).trip);
        let published_before = c.trips();
        assert_eq!(published_before, 1);
        // The arm: the value row is re-derived and installed (same limit here, since the operator
        // changed something else), and the bridge was disarmed for the whole sequence.
        c.reconfigure(lim);
        // The first period after the arm, with the short still there.
        let v = c.step(hard_trip_counts(lim));
        assert!(v.trip, "the first period after the arm re-trips");
        assert_ne!(
            c.trips(),
            published_before,
            "the published count must CHANGE, or the 250 Hz task raises no latch"
        );
        assert_eq!(c.trips(), 2, "and it counts on, never restarting");
    }

    /// The EPISODE state is reset by the seam, so a genuine new over-limit episode still counts
    /// after an arm. Without the reset, `tripped` from the pre-arm episode would suppress the first
    /// trip under the new limit.
    #[test]
    fn a_reconfigure_mid_episode_does_not_suppress_the_next_trip() {
        let lim = 4_000i16;
        let mut c = CurrentLimit::new(lim);
        // Mid-episode: over the limit and already tripped, with no clean period to re-arm it.
        assert!(c.step(hard_trip_counts(lim)).trip);
        assert!(
            !c.step(hard_trip_counts(lim)).trip,
            "same episode, one trip"
        );
        assert_eq!(c.trips(), 1);
        c.reconfigure(lim);
        // The same magnitude now counts as a NEW episode's trip.
        assert!(
            c.step(hard_trip_counts(lim)).trip,
            "the episode in flight was judged against the replaced limit"
        );
        assert_eq!(c.trips(), 2);
    }

    /// **A method byte rebuilds the records, and the rebuild carries the BOOT-FIXED decode facts.**
    /// For every byte: the published method is the built-arm clamp of the decoded request, and the
    /// rebuilt records behave exactly as a freshly constructed state for the boot's direction and
    /// align offset, over every hall code and both demand signs.
    #[test]
    fn a_method_byte_rebuilds_the_records_from_the_boot_facts() {
        use commutation::sixstep::{sixstep_step, Direction, SixStep, SixStepState};
        use commutation::MethodState;
        for direction in [false, true] {
            for align_offset in 0..6u8 {
                let boot = BootFixed {
                    direction,
                    align_offset,
                };
                let fresh = SixStepState::new(SixStep::new(
                    if direction {
                        Direction::Reverse
                    } else {
                        Direction::Forward
                    },
                    align_offset,
                ));
                for byte in 0..=u8::MAX {
                    let r = rederive(byte, 10_000, 455, boot);
                    assert_eq!(
                        r.method,
                        running_method(requested_method(byte)).to_u8(),
                        "byte {byte} must read back as what runs, not what was asked"
                    );
                    let MethodState::SixStep(st) = r.records else {
                        panic!("the only built arm is six-step");
                    };
                    assert_eq!(r.records.method(), running_method(requested_method(byte)));
                    assert_eq!(st, fresh, "fresh records for the boot-fixed decode");
                    for code in 0..8u8 {
                        for demand in [-28_500i32, -1, 0, 1, 28_500] {
                            assert_eq!(
                                sixstep_step(&st, code, demand),
                                sixstep_step(&fresh, code, demand),
                                "code {code}, demand {demand}"
                            );
                        }
                    }
                }
            }
        }
    }

    /// The REQUESTED method is decoded faithfully even though the running method is clamped, so the
    /// clamp in [`running_method`] is a visible POLICY act rather than a parse that lost the
    /// request. That is also what lets the observation block report six-step instead of claiming a
    /// method the board is not running.
    ///
    /// It decides nothing else: the offset calibration
    /// ([`BringUpStep::CalibratePhaseOffsets`]) is unconditional, and the decoded request reaches
    /// nothing in the bring-up but [`BringUpStep::SelectMethodAndInstall`] (the installed records,
    /// `MotorRuntime.method`, and the published observation byte).
    #[test]
    fn requested_method_decodes_the_store_byte() {
        use commutation::CommutationMethod as M;
        assert_eq!(requested_method(0), M::SixStep);
        assert_eq!(requested_method(1), M::Sine);
        assert_eq!(requested_method(2), M::Foc);
        assert_eq!(
            requested_method(99),
            M::SixStep,
            "unknown bytes are six-step"
        );
    }

    /// The calibration ACCUMULATION, not a mean: [`CAL_SAMPLES`] conversions per channel summed as
    /// `sample >> 3`, which is 2x the zero-current sample as the register presents it.
    /// The pairing is this crate's; the arithmetic is `commutation::foc::calibrate_offset`'s, and
    /// this test pins that the two channels are not swapped and the unit is not rescaled here.
    #[test]
    fn cal_offsets_accumulates_through_the_commutation_owner() {
        use commutation::foc::calibrate_offset;

        // A constant stream of one register value: 16 x (sample >> 3) = 2 x that value.
        let stream = |reg: u16| [reg; CAL_SAMPLES];
        assert_eq!(
            cal_offsets(&stream(0x3FD8), &stream(0x3ED0)),
            (0x7FB0, 0x7DA0)
        );
        // The bench's own measurement (`04-master-cal-4por.log`): register means 0x3F00 / 0x3F90
        // land mid-window once accumulated, which is the unit the acceptance gate reads.
        let (a, b) = cal_offsets(&stream(0x3F00), &stream(0x3F90));
        assert_eq!((a, b), (0x7E00, 0x7F20));
        assert!(commutation::foc::PhaseOffsets::try_new(a, b).is_some());
        // It is NOT the mean of the raw registers: that is half the accumulated offset, and it
        // falls outside the window (the slice-4 bench failure, in one assertion).
        assert!(commutation::foc::PhaseOffsets::try_new(0x3F00, 0x3F90).is_none());
        // Delegation, channel order included: each half is exactly the owner's accumulation.
        let a_run = stream(0x3F00);
        let b_run = stream(0x3F90);
        assert_eq!(
            cal_offsets(&a_run, &b_run),
            (calibrate_offset(&a_run), calibrate_offset(&b_run))
        );
        assert_eq!(cal_offsets(&[0; CAL_SAMPLES], &[0; CAL_SAMPLES]), (0, 0));
    }

    /// The fallback policy's live half (`specs/motor-integration.md`, bring-up step 9): a REFUSED
    /// calibration raises the init-failure fault; an accepted one and a boot that ran none do not.
    /// The fallback to six-step itself is the built-arm clamp above, asserted here alongside so
    /// the pair reads as one policy.
    #[test]
    fn refused_cal_falls_back_to_six_step_and_raises_the_init_fault() {
        use commutation::CommutationMethod as M;
        assert_eq!(init_fault_bits(CalOutcome::Refused), FAULT_INIT_CAL);
        assert_eq!(init_fault_bits(CalOutcome::Accepted), 0);
        // ...and whatever the outcome, what runs is six-step.
        assert_eq!(running_method(requested_method(2)), M::SixStep);
        // The init fault is a fault_a producer on a configured motor.
        assert!(motor_fault_level(true, FAULT_INIT_CAL, false, false));
    }

    /// The acceptance window is the commutation crate's, not a copy: the bench's stage-3 band and
    /// the boundary both answer through `PhaseOffsets::try_new`, which is what the bring-up calls.
    #[test]
    fn the_cal_window_owner_is_the_commutation_crate() {
        use commutation::foc::PhaseOffsets;
        // The stock-healthy sanity band the silicon gate reads against.
        assert!(PhaseOffsets::try_new(0x7DAE, 0x7FB8).is_some());
        // The acceptance window's edges: lo inclusive, hi exclusive.
        assert!(PhaseOffsets::try_new(0x7531, 0x86C3).is_some());
        assert!(PhaseOffsets::try_new(0x7530, 0x7FB8).is_none());
        assert!(PhaseOffsets::try_new(0x7FB8, 0x86C4).is_none());
        // A quiet-bridge measurement that came back railed or dead is refused.
        assert!(PhaseOffsets::try_new(0, 0).is_none());
        assert!(PhaseOffsets::try_new(0xFFFF, 0xFFFF).is_none());
    }

    /// `Foc` against a motor with no phase-current group does not run FOC and is recorded: the
    /// motor is not brought up at all, so there is no six-step to fall back to and no arming can
    /// follow. The spec's `current_sense = 0` fallback arm, as this slice expresses it.
    #[test]
    fn foc_against_no_current_sense_is_a_recorded_skip_not_a_running_motor() {
        // The word `record_skip` publishes for this reason (asserted against the live static in
        // `skip_reasons_are_distinguishable_in_the_observation`; built purely here so the two
        // tests do not race on the shared observation word).
        let w = pack_obs_state(0, [false; 3], 0, OBS_SKIP_NO_SENSE);
        assert!(!obs_configured(w), "nothing was brought up");
        assert_eq!((w >> 24) & 0xFF, OBS_SKIP_NO_SENSE);
        // ...and with nothing configured, no motor fault is produced from its dead counters, so
        // the FOC request neither runs FOC nor faults the board: it is recorded and ignored.
        assert!(!motor_fault_level(false, 0, true, false));
        assert_eq!(requested_method(2), commutation::CommutationMethod::Foc);
    }

    /// The period-liveness fault producer over synthetic counters: a SUSTAINED shortfall asserts,
    /// a single short tick does not, absence never does, and recovery needs a proven-clean stream.
    #[test]
    fn period_liveness_fault_needs_a_sustained_shortfall() {
        let mut h = PeriodHealth::new();
        assert!(!h.loss());
        // One short tick is jitter, not a fault.
        h.update(true, false);
        assert!(!h.loss(), "one short tick is not the fault");
        // A live tick resets the streak.
        h.update(true, true);
        for _ in 0..(PERIOD_LOSS_THRESHOLD - 1) {
            h.update(true, false);
        }
        assert!(!h.loss(), "one short of the threshold");
        h.update(true, false);
        assert!(h.loss(), "the threshold asserts");
        // Recovery is asymmetric: a single good tick does not release it.
        h.update(true, true);
        assert!(h.loss(), "one good tick does not release the fault");
        for _ in 0..PERIOD_RECOVER_THRESHOLD {
            h.update(true, true);
        }
        assert!(!h.loss(), "a proven-clean stream releases it");
        // Absence is never loss: an unconfigured motor's counter never advances, and that is
        // exactly the pre-motor boot posture, which must not fault.
        let mut absent = PeriodHealth::new();
        for _ in 0..1000 {
            absent.update(false, false);
        }
        assert!(!absent.loss());
    }

    /// Which fault bits reach `fault_a`, and which deliberately do not.
    #[test]
    fn the_motor_fault_producers_are_exactly_the_four_named() {
        // The three producers.
        assert!(
            motor_fault_level(true, FAULT_HALL, false, false),
            "hall dwell"
        );
        assert!(
            motor_fault_level(true, FAULT_INIT_CAL, false, false),
            "refused cal"
        );
        assert!(
            motor_fault_level(true, 0, true, false),
            "period-liveness loss"
        );
        assert!(motor_fault_level(true, 0, false, true), "a refused arm");
        // Not producers: the freshness guard has already floated every phase itself, and a refused
        // duty left the outputs untouched. Both stay observable in the motor block.
        assert!(!motor_fault_level(true, FAULT_DEMAND_STALE, false, false));
        assert!(!motor_fault_level(true, FAULT_DUTY_RANGE, false, false));
        // Nothing configured: nothing produced, whatever the words say.
        assert!(!motor_fault_level(
            false,
            FAULT_HALL | FAULT_INIT_CAL,
            true,
            false
        ));
    }

    /// The cal-accepted flag rides bit 7 of the observation's flag byte, so it does not collide
    /// with any skip reason and survives the pack/unpack round trip.
    #[test]
    fn cal_accepted_flag_packs_clear_of_the_skip_reasons() {
        let flags = OBS_CONFIGURED | OBS_CURRENT_SENSE | OBS_CAL_ACCEPTED;
        let w = pack_obs_state(5, [true, true, false], 0, flags);
        assert_eq!((w >> 24) & 0xFF, flags);
        assert!(obs_configured(w));
        assert_eq!(
            OBS_CAL_ACCEPTED
                & (OBS_SKIP_ABSENT | OBS_SKIP_NO_SENSE | OBS_SKIP_STEP_FAILED | OBS_COASTING),
            0,
            "the flag byte's bits stay disjoint"
        );
        // Bit 7 is the last bit the flag byte has: the packing masks flags to 0xFF, so a ninth
        // flag would be silently dropped rather than shifted into the method byte.
        assert_eq!(pack_obs_state(0, [false; 3], 0, 1 << 8) >> 24, 0);
    }

    /// A skipped motor is recorded distinguishably: which reason, and for a failed step, which
    /// step. `configured` stays clear in every arm.
    #[test]
    fn skip_reasons_are_distinguishable_in_the_observation() {
        record_skip(MotorSkip::Absent);
        let w = OBS_STATE.load(Ordering::Relaxed);
        assert_eq!(
            (w >> 24) & 0xFF,
            OBS_SKIP_ABSENT >> 4 << 4 >> 4 << 4,
            "absent"
        );
        assert_eq!(w & OBS_CONFIGURED, 0);

        record_skip(MotorSkip::NoCurrentSense);
        let w = OBS_STATE.load(Ordering::Relaxed);
        assert_eq!((w >> 24) & 0xFF, OBS_SKIP_NO_SENSE >> 4 << 4 >> 4 << 4);

        record_skip(MotorSkip::StepFailed(BringUpStep::ConfigureInjectedGroup));
        let w = OBS_STATE.load(Ordering::Relaxed);
        assert_eq!((w >> 24) & 0xFF, OBS_SKIP_STEP_FAILED);
        assert_eq!(
            (w >> 16) & 0xFF,
            BRING_UP_STEPS
                .iter()
                .position(|s| *s == BringUpStep::ConfigureInjectedGroup)
                .unwrap() as u32,
            "the failing step's index rides in the method byte"
        );
    }

    /// The calibration is unconditional: the outcome has no "not requested" arm left, so every
    /// brought-up motor either has trusted zeros or carries the init fault. The exhaustive match is
    /// the assertion (a reintroduced variant stops this compiling).
    #[test]
    fn the_calibration_outcome_has_no_not_requested_arm() {
        for c in [CalOutcome::Accepted, CalOutcome::Refused] {
            let fault = match c {
                CalOutcome::Accepted => 0,
                CalOutcome::Refused => FAULT_INIT_CAL,
            };
            assert_eq!(init_fault_bits(c), fault);
        }
    }

    /// The boot conversion at its clamps: the 40 A milliamp ceiling, the COUNT-domain floor, and
    /// the scale as the per-board value it now is.
    #[test]
    fn limit_counts_at_the_clamps() {
        assert_eq!(
            CAL, 455,
            "0x67's registered default, the 2026-10-09 bench figure"
        );
        assert_eq!(
            limit_counts(10_000, CAL),
            4_550,
            "the registered limit, 10 A"
        );
        assert_eq!(
            limit_counts(15_000, CAL),
            6_825,
            "the walk tool's round-trip value, 15 A"
        );
        // The floor is a count, so a milliamp request worth less than it is clamped UP: at this
        // scale 4,615 mA converts to 2,099 counts, one under the floor.
        assert_eq!(limit_counts(0, CAL), MIN_LIMIT_COUNTS, "the floor");
        assert_eq!(limit_counts(4_615, CAL), MIN_LIMIT_COUNTS);
        assert_eq!(
            limit_counts(4_700, CAL),
            2_138,
            "clear of the floor, converts straight through"
        );
        // The ceiling stays a MILLIAMP ceiling.
        assert_eq!(limit_counts(40_000, CAL), 18_200);
        assert_eq!(
            limit_counts(40_001, CAL),
            18_200,
            "over the ceiling reads as the ceiling"
        );
        assert_eq!(limit_counts(u32::MAX, CAL), 18_200);
        // The same request against two other boards' scales: the conversion is the board's, not
        // the firmware's. 800 is what the compiled constant used to assert for every board.
        assert_eq!(limit_counts(10_000, 800), 8_000);
        assert_eq!(limit_counts(10_000, 300), 3_000);
    }

    /// The calibration's own seam (`CURRENT_CAL_MIN`..`CURRENT_CAL_MAX`), clamped at the boot
    /// conversion because the store validates type only. The upper bound is what keeps the 40 A
    /// ceiling's product inside the `i16` the limit comparison holds; without the clamp, a
    /// hand-poked 0x67 would wrap it negative and the chop would fire on every period.
    #[test]
    fn the_calibration_is_clamped_into_its_seam_range() {
        // At the top of the seam with the 40 A ceiling: the widest product the comparison can see.
        assert_eq!(limit_counts(40_000, CURRENT_CAL_MAX), 32_760);
        assert!(limit_counts(40_000, CURRENT_CAL_MAX) > 0, "inside the i16");
        // Above the seam, the clamp holds that same bound.
        assert_eq!(limit_counts(40_000, CURRENT_CAL_MAX + 1), 32_760);
        assert_eq!(
            limit_counts(40_000, 1_000),
            32_760,
            "a plausible typo, still bounded"
        );
        assert_eq!(limit_counts(40_000, u16::MAX), 32_760);
        // Below it: an unset or typoed 0x67 cannot make every limit saturate the comparator.
        assert_eq!(
            limit_counts(40_000, 0),
            4_000,
            "clamped up to CURRENT_CAL_MIN"
        );
        assert_eq!(limit_counts(40_000, CURRENT_CAL_MIN - 1), 4_000);
        assert_eq!(limit_counts(40_000, CURRENT_CAL_MIN), 4_000);
        // And the hard trip over the clamped limit saturates rather than wrapping.
        assert_eq!(
            hard_trip_counts(limit_counts(40_000, CURRENT_CAL_MAX)),
            32_767
        );
    }

    /// The hard trip is twice the soft limit, saturating at the sensor's full scale.
    #[test]
    fn hard_trip_counts_saturates_at_full_scale() {
        assert_eq!(hard_trip_counts(800), 1_600);
        assert_eq!(hard_trip_counts(8_000), 16_000);
        assert_eq!(hard_trip_counts(16_383), 32_766);
        assert_eq!(hard_trip_counts(16_384), 32_767, "saturates");
        assert_eq!(
            hard_trip_counts(limit_counts(40_000, CAL)),
            32_767,
            "the 40 A limit's 2x saturates at this board's scale too"
        );
        assert_eq!(hard_trip_counts(i16::MAX), 32_767);
    }

    /// The magnitude fold: the third phase is the Kirchhoff remainder of the two sensed ones, with
    /// its sign, saturated the stock way, and the magnitude is the largest of the three.
    #[test]
    fn phase_magnitude_folds_the_unsensed_phase() {
        // The third phase dominates: ic = -(100 + 200) = -300.
        assert_eq!(phase_magnitude(100, 200), 300);
        assert_eq!(phase_magnitude(-100, -50), 150, "ic = +150");
        // A sensed phase dominates.
        assert_eq!(phase_magnitude(-900, 400), 900);
        assert_eq!(phase_magnitude(0, 0), 0);
        // Saturation: the remainder of two full-scale phases saturates to +-32767.
        assert_eq!(phase_magnitude(32_767, 32_767), 32_767);
        assert_eq!(phase_magnitude(-32_767, -32_767), 32_767);
        assert_eq!(phase_magnitude(20_000, 20_000), 32_767, "ic saturates");
        assert_eq!(phase_magnitude(i16::MIN, 0), 32_767, "never negative");
        // Six-step: one phase floats. A or B floating: the magnitude is the other's.
        assert_eq!(phase_magnitude(0, -500), 500, "A floating");
        assert_eq!(phase_magnitude(700, 0), 700, "B floating");
        // C floating: ia = -ib, so the magnitude is |ia| = |ib|.
        assert_eq!(phase_magnitude(400, -400), 400, "C floating");
        assert_eq!(phase_magnitude(-400, 400), 400, "C floating, reversed");
    }

    /// The unit, end to end through the stock reader: one 12-bit ADC LSB of deviation from the
    /// measured zero is 16 counts, and the bench's zero reads zero current.
    #[test]
    fn the_magnitude_is_in_stock_current_counts() {
        use commutation::foc::current_from_adc;
        // The bench's accumulated offset 0x7E00 is 2x the zero-current register value 0x3F00.
        let zero = 0x3F00u16;
        let offset = 0x7E00u16;
        assert_eq!(current_from_adc(offset, zero), 0);
        // The left-aligned register carries the 12-bit value << 3, so one LSB is 8 register units.
        assert_eq!(current_from_adc(offset, zero - 8), 16);
        assert_eq!(current_from_adc(offset, zero + 8), -16);
        // The unit, not this board's scale: 50 ADC LSB (EFeru's `A2BIT_CONV`, the figure the old
        // compiled `COUNTS_PER_AMP = 800` was built from) is 800 counts in it.
        let one_amp = current_from_adc(offset, zero - 50 * 8);
        assert_eq!(one_amp, 800);
        assert_eq!(
            phase_magnitude(one_amp, current_from_adc(offset, zero)),
            800
        );
    }

    /// The chop decision is `>`, not `>=`, on this period's magnitude alone.
    #[test]
    fn the_chop_is_strictly_over_the_limit() {
        let lim = limit_counts(10_000, CAL);
        let mut c = CurrentLimit::new(lim);
        assert!(!c.step(lim).chop, "AT the limit drives");
        assert!(c.step(lim + 1).chop, "one count over floats");
        assert!(
            !c.step(lim).chop,
            "the next period re-evaluates from its own sample"
        );
        assert!(!c.step(0).chop);
    }

    /// The hard trip's magnitude condition: at `hard_trip_counts` on the first period, and not one
    /// count under it.
    #[test]
    fn the_trip_fires_at_the_hard_magnitude() {
        let lim = limit_counts(2_000, CAL);
        let hard = hard_trip_counts(lim);
        let mut c = CurrentLimit::new(lim);
        let v = c.step(hard - 1);
        assert!(
            v.chop && !v.trip,
            "over the limit, under the hard trip: chop only"
        );
        assert_eq!(c.trips(), 0);
        let mut c = CurrentLimit::new(lim);
        let v = c.step(hard);
        assert!(
            v.chop && v.trip,
            "at the hard magnitude: trip on the first period"
        );
        assert_eq!(c.trips(), 1);
    }

    /// The hard trip's run condition: 63 consecutive over-limit periods do not trip, the 64th
    /// does, and one clean period resets the run.
    #[test]
    fn the_trip_fires_at_64_consecutive_over_limit_periods() {
        let lim = limit_counts(2_000, CAL);
        let over = lim + 1;
        let mut c = CurrentLimit::new(lim);
        for _ in 0..OVER_CURRENT_TRIP_PERIODS - 1 {
            assert!(!c.step(over).trip);
        }
        assert_eq!(c.trips(), 0, "63 consecutive do not trip");
        assert!(c.step(over).trip, "the 64th does");
        assert_eq!(c.trips(), 1);

        // One clean period resets the run.
        let mut c = CurrentLimit::new(lim);
        for _ in 0..OVER_CURRENT_TRIP_PERIODS - 1 {
            c.step(over);
        }
        assert!(!c.step(lim).trip, "a period at the limit is clean");
        for _ in 0..OVER_CURRENT_TRIP_PERIODS - 1 {
            assert!(!c.step(over).trip, "the run restarted");
        }
        assert!(c.step(over).trip);
    }

    /// A trip counts once per over-limit episode: the conditions stay true for every further
    /// over-limit period, and only a clean period re-arms the trip.
    #[test]
    fn a_trip_counts_once_per_over_limit_episode() {
        let lim = limit_counts(2_000, CAL);
        let hard = hard_trip_counts(lim);
        let mut c = CurrentLimit::new(lim);
        assert!(c.step(hard).trip);
        for _ in 0..200 {
            let v = c.step(hard);
            assert!(v.chop, "every over-limit period still floats");
            assert!(!v.trip, "the same episode does not trip again");
        }
        assert_eq!(c.trips(), 1);
        c.step(0);
        assert!(c.step(hard).trip, "a clean period re-arms it");
        assert_eq!(c.trips(), 2);
    }

    /// The observation window: the peak is the window's maximum magnitude, `chopped` its count of
    /// floated periods, and closing the window publishes both and restarts them.
    #[test]
    fn the_window_peak_and_its_restart() {
        let lim = limit_counts(1_000, CAL);
        let mut c = CurrentLimit::new(lim);
        for mag in [10, 500, lim + 5, 30, lim + 1] {
            c.step(mag);
        }
        let w = c.take_window();
        assert_eq!(w & 0xFFFF, (lim + 5) as u32, "peak");
        assert_eq!((w >> 16) & 0xFF, 2, "two periods floated");
        assert_eq!(w >> 24, 0, "no trip");
        // Restarted: the next window sees only its own periods.
        c.step(40);
        let w = c.take_window();
        assert_eq!(w & 0xFFFF, 40);
        assert_eq!((w >> 16) & 0xFF, 0);
        // A whole window floated: `chopped` reads 64, and the trip it caused shows in the top byte.
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(lim + 1);
        }
        let w = c.take_window();
        assert_eq!((w >> 16) & 0xFF, 64);
        assert_eq!(w >> 24, 1, "the run tripped once");
    }

    /// The packed word's layout (`CTRL_OBS` word 31): peak in 0..15, chopped in 16..23, the trip
    /// count's low byte (wrapping) in 24..31.
    #[test]
    fn motor_current_packing() {
        let w = pack_motor_current(32_000, 64, 3);
        assert_eq!(w & 0xFFFF, 32_000);
        assert_eq!((w >> 16) & 0xFF, 64);
        assert_eq!(w >> 24, 3);
        assert_eq!(
            pack_motor_current(0, 0, 0x1FF) >> 24,
            0xFF,
            "trips wrap in the byte"
        );
        assert_eq!(pack_motor_current(0, 0, 0x100) >> 24, 0);
    }

    /// The hard trip is NOT a motor-side fault level producer: it reaches `fault_a` only through
    /// motor 0's latch (the orchestrator's `raise_over_current` seam, whose host test asserts the
    /// level stays clear through a trip). Here, over the fold function: the ISR records a trip in
    /// `OVER_CURRENT_TRIPS` / `OBS_CURRENT` and sets no `FAULT` bit, and the fold reads exactly the
    /// hall and calibration bits of that word, so no bit a trip could set is folded.
    #[test]
    fn a_trip_does_not_reach_the_motor_fault_level() {
        let mut c = CurrentLimit::new(limit_counts(1_000, CAL));
        assert!(c.step(i16::MAX).trip);
        let producers = FAULT_HALL | FAULT_INIT_CAL;
        for bit in 0..32 {
            let w = 1u32 << bit;
            assert_eq!(
                motor_fault_level(true, w, false, false),
                w & producers != 0,
                "fault bit {bit}"
            );
        }
        assert!(!motor_fault_level(true, 0, false, false));
    }

    /// The packed observation word round-trips the fields the bench reads back over SWD.
    #[test]
    fn obs_state_packing() {
        let w = pack_obs_state(6, [true, false, true], 1, OBS_CONFIGURED | OBS_PERIOD_LIVE);
        assert_eq!(w & 0xFF, 6, "hall code");
        assert_eq!((w >> 8) & 0xFF, 0b101, "enables");
        assert_eq!((w >> 16) & 0xFF, 1, "method");
        assert_eq!((w >> 24) & 0xFF, 0b101, "flags");
    }
}
