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
//!   steps `motorcontroller/commutation`, applies duties + channel enables, re-arms the ADC trigger,
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
use orchestrator::{EV_MOTOR_ARM, EV_MOTOR_CAL, EV_MOTOR_HALL, EV_MOTOR_PERIOD};

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
/// The period-based wheel rate (`specs/control.md` (o)): the commutator's `rate_q`, the
/// reciprocal of the periods between two same-direction hall edges in 1/4096 of a count (one
/// count = one edge per 320-period window, [`SPEED`]'s unit), decaying as `1/t` between edges
/// and 0 across a reversal. The outer speed loop's measurement; copied into the block beside the
/// count, published as `CTRL_OBS` word 39. Written by the period ISR.
pub static WHEEL_RATE_Q12: AtomicI32 = AtomicI32::new(0);
/// The wheel's SIGNED edge position (`specs/state-feedback.md` section 3): the commutator's
/// `pos_edges`, +1 or -1 per counted hall edge in the counter's own sense, never cleared, wrapping.
/// Copied into the block's `wheel_pos` by the 250 Hz shell beside [`WHEEL_RATE_Q12`]; the cyclic carries
/// its low 16 bits to the peer. Written by the period ISR.
pub static POS_EDGES: AtomicI32 = AtomicI32::new(0);
/// The free-running period counter: the liveness signal. Written by the period ISR.
pub static PERIODS: AtomicU32 = AtomicU32::new(0);
/// This boot's ABSOLUTE hall-edge count, the odometer's raw material (`specs/odometer.md`): the
/// commutator's own `edges_total`, which adds 1 on every counted edge whichever way the wheel
/// turned. Written by the period ISR, in the once-per-64-periods branch that already closes the
/// current window, so the per-period path gains nothing for it.
///
/// It counts THIS BOOT and it wraps. The total a board keeps is this folded as deltas onto the
/// value its store holds, which is [`crate::odometer::Odometer`]'s job and not this cell's.
pub static EDGES: AtomicU32 = AtomicU32::new(0);
/// Motor fault bits ([`FAULT_HALL`], [`FAULT_DUTY_RANGE`], [`FAULT_DEMAND_STALE`],
/// [`FAULT_INIT_CAL`], [`FAULT_METHOD`]). Written by the period ISR (sticky within a boot), seeded
/// by the bring-up with the two bits it can raise before the ISR exists.
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
/// Each half is in the ACCUMULATED offset unit ([`cal_offsets`]): 16 times the mean of the
/// conversions each shifted right by 3, which is 2x the zero-current register value (round 4
/// measures blocks of [`CAL_SAMPLES`] and publishes the mean of the first two consecutive blocks
/// that agree, or the last block of a refused run, `specs/foc-torque.md` section 15). A bench read
/// compares it against the acceptance window
/// and the stock healthy band in those units, NOT against a raw sample.
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
/// Set by the bring-up's `StartCounter`, on the boot thread, and never written again: the counter
/// is the hall-edge measurement's clock, owned by the bring-up, and NOTHING in this image stops it
/// (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed"). The ISR never
/// writes it.
///
/// It is the PREMISE of the period-liveness supervisor ([`PeriodHealth::update`]): a board whose
/// counter never started is silent for a reason that is not a wedged vector, and reading it as one
/// would fault a board that was only ever waiting.
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
/// The method-demotion fault: the stored `MOTOR_METHOD` named a method this board cannot run, so
/// the bring-up fell back to six-step (`specs/commutation.md`, "Current-sense conditioning and the
/// FOC capability gate": a `Foc` request on a board without phase-current sense, or with a refused
/// offset calibration, falls back and raises a fault flag rather than silently driving). Raised by
/// [`method_fault_bits`], set ONCE by the bring-up beside [`FAULT_INIT_CAL`] and carried forward by
/// the ISR.
///
/// Deliberately NOT a [`motor_fault_level`] producer. The board is running six-step, which is what
/// a six-step-configured board runs, so demoting and then refusing to drive would make a FOC
/// request a brick. It is the READBACK: the same shape as the control dispatch demoting Balance to
/// Throttle and raising the mode fault on a board with no IMU (`motorcontroller/orchestrator`,
/// `EV_MODE_FAULT`). The refused-cal half of the demotion keeps its own teeth, because
/// [`FAULT_INIT_CAL`] IS a producer, so that board does not arm at all.
pub const FAULT_METHOD: u32 = 1 << 4;

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

/// Whether a packed [`OBS_STATE`] word says the motor's demand word is a CURRENT: a configured
/// motor whose published method byte is `CommutationMethod::Foc` (`specs/foc-torque.md` section
/// 4). The 250 Hz task hands this to the balance law as `BlockWords::torque_is_current`, so the
/// law stops normalizing its word by the battery exactly when the period ISR runs the current
/// loop.
///
/// The byte is the RUNNING method (the ISR republishes it every period from its own record, so it
/// follows the capability demotion and the arm-time install), which is the one the word's meaning
/// depends on. Gated on `configured` because the same byte carries a failed bring-up step's index
/// ([`record_skip`]), and an index of 2 is not a method.
#[inline]
pub fn obs_torque_is_current(state: u32) -> bool {
    obs_configured(state) && (state >> 16) as u8 == commutation::CommutationMethod::Foc.to_u8()
}

/// The control block's wheel-speed word, from this module's [`SPEED`] cell: the signed edge count
/// per 320-period window in the ISR's own unit, with NO rescaling (`specs/link-control.md`,
/// "`wheel_speed` carries the raw edge count"). The pole-pair fold to a speed unit stays the
/// Phase-D consumer's, as [`SPEED`] says, and the block row is deliberately not where it happens.
///
/// SATURATING, not a cast, and that is a safety property rather than tidiness: the cell is an
/// `i32`, the block row an `i16`, and a wrapping narrowing lands on exactly 0 every 65,536 edges.
/// Zero is the one value the step-off latch's health rule reads as "the wheel is not moving"
/// (`motorcontroller/state/src/fault.rs`, `FaultLatch::is_healthy` tests `b_motion != 0`), so a wrap could
/// report a turning wheel as stationary. A clamp can only ever land on the rails, which are
/// nonzero. The saturation is unreachable on the real plant (a 20 ms window of hall edges is tens,
/// not tens of thousands); it is here so the narrowing is a stated fact rather than luck, the way
/// the shell's own `i16` narrowings in `motorcontroller/orchestrator/src/dispatch.rs` are.
///
/// Plain `i16` bounds, NOT `commutation::foc::sat16`: that one saturates to the symmetric
/// [-32767, +32767] because the FOC domain reserves the `-0x8000` sentinel, and the block row
/// reserves nothing, so borrowing it would exclude a legal value for a reason that does not apply.
///
/// A stationary wheel reads EXACTLY 0: the hall code does not change, so the commutator counts no
/// edges, the cell holds 0, and 0 clamps to 0. No floor, no offset, nothing added on the way.
#[inline]
pub fn wheel_speed_word(speed: i32) -> i16 {
    speed.clamp(i16::MIN as i32, i16::MAX as i32) as i16
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

/// The output posture for one period: all three phases FLOAT instead of being driven. Three
/// producers, ORed because each is sufficient on its own, and the ISR evaluates it once per
/// period.
///
/// - **Not `armed`**: the bridge is not energized, so the ISR has no business driving channel
///   enables into it. This is the producer that keeps the shutdown's `FloatChannels` posture TRUE
///   rather than true for one period: the shutdown no longer stops the counter
///   (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed"), so the ISR runs
///   on across it, measuring hall edges for the OFF interlock, and it holds the bridge floated
///   the whole time it does. It also covers the boot-to-first-arm stretch, where the counter has
///   always run and the ISR used to drive enables at a bridge MOE alone was holding quiet.
/// - **`stale`**: the demand-freshness guard, i.e. the 250 Hz task stopped writing.
/// - **`chop`**: the soft current limit, for this period only (the next re-evaluates from its own
///   sample).
#[inline]
pub fn coast_posture(armed: bool, stale: bool, chop: bool) -> bool {
    !armed || stale || chop
}

/// Periods the ISR runs per 250 Hz tick when healthy: 16000 / 250 = 64.
pub const PERIODS_PER_TICK_NOMINAL: u32 = 64;
/// [`PERIODS_PER_TICK_NOMINAL`] as a SHIFT, which is what the observation window's mean divides
/// by ([`CurrentLimit::take_window`]). It exists so the divisor and the window length cannot come
/// apart: a window of 64 periods whose mean shifted by 5 would read double, and the assert below
/// is what makes that a build failure rather than a telemetry bug.
pub const PERIODS_PER_TICK_SHIFT: u32 = 6;
const _: () = assert!(1 << PERIODS_PER_TICK_SHIFT == PERIODS_PER_TICK_NOMINAL);
// The FOC arm's means shift by their own constant (the commutation crate cannot see this one), and
// the ISR closes its window in the same branch as the current limit's, so the two must agree.
const _: () = assert!(commutation::foc::FOC_WINDOW_SHIFT == PERIODS_PER_TICK_SHIFT);
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
    /// rides is not turning, which now means only one thing: the motor was never brought up
    /// (reset to healthy, never loss).
    ///
    /// **The premise is a RUNNING counter, not merely a configured motor** (slice 5). It was the
    /// shutdown's deliberate silence that this guarded against; the shutdown no longer stops the
    /// counter (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed"), so the
    /// supervisor now watches an ISR that runs for the whole of a powered life, armed or not. The
    /// premise stays stated rather than assumed: the fault this produces must never be one this
    /// layer manufactured by stopping the thing it then measures.
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
/// `arm_refused` is slice 5's fourth producer: an arm attempt that aborted before the MOE step
/// (`crate::arm`, which holds the two refusals this level sees as one bool). It is a producer
/// because the alternative posture is the dangerous one -- a vehicle sitting in RUN believing it is
/// driving, with a bridge that was never energized and a commutator that was never stepped. Making
/// it a fault turns that into a shutdown.
///
/// The two refusing steps differ in STICKINESS, which this level is deliberately blind to
/// (`specs/integration.md`, "A refused re-read refuses the ARM, not the boot"): a failed
/// `ConfirmPeriodsLive` is sticky for the boot like the ISR's own fault bits, because it measures
/// something an OFF dwell does not heal, while a failed `ReReadValues` releases at the end of the
/// engage that met it (an OFF pass with the power request dropped), because it is a verdict on a
/// stored number that heals the instant a correct one is written. Both
/// reach this level while HELD, so either one shuts the board down; how long each is held belongs
/// to the producer, exactly as it does for the hall dwell fault and the liveness hysteresis.
///
/// The two [`FAULT`]-word producers are read from the WORD; the other two levels arrive as levels.
/// Nothing downstream ever reads the word again: this function is the level's single owner, and the
/// arming gate consumes only what it returns.
///
/// It returns the CAUSES, one `orchestrator::events::EV_MOTOR_*` bit each
/// (`specs/link-control.md`, "`fault` widens to u16"): [`EV_MOTOR_HALL`] for [`FAULT_HALL`],
/// [`EV_MOTOR_CAL`] for [`FAULT_INIT_CAL`], [`EV_MOTOR_PERIOD`] for `period_loss` and
/// [`EV_MOTOR_ARM`] for `arm_refused`. A consumer that needs the old level reads the union
/// (`!= 0`); the cyclic `fault` word carries the four apart.
#[inline]
pub fn motor_fault_level(
    configured: bool,
    faults: u32,
    period_loss: bool,
    arm_refused: bool,
) -> u16 {
    if !configured {
        return 0;
    }
    let bit = |set: bool, ev: u16| if set { ev } else { 0 };
    bit(faults & FAULT_HALL != 0, EV_MOTOR_HALL)
        | bit(faults & FAULT_INIT_CAL != 0, EV_MOTOR_CAL)
        | bit(period_loss, EV_MOTOR_PERIOD)
        | bit(arm_refused, EV_MOTOR_ARM)
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
/// `orchestrator::battery::VbattCal::new` precedent), and the band is a PLAUSIBILITY one on a board
/// fact, the same class as the battery divider's slope clamp. It is no longer an arithmetic bound:
/// 819 was once derived from a 40 A milliamp ceiling fitting the `i16` (`40_000 * 819 / 1000 <=
/// i16::MAX`), and that ceiling is gone, because [`MAX_LIMIT_COUNTS`] bounds the CONVERTED count
/// instead and nothing the conversion computes can leave the `i16` whatever the stored scale is.
///
/// What the band still buys, which the count window cannot: it bounds how far a typoed scale can
/// move the ENFORCED limit away from the LABELLED one. The count ceiling bounds the limit in
/// absolute terms; the ratio is the seam's. Without the upper bound a stored 8,190 would convert a
/// deliberately conservative 5 A staging to the widest limit the window admits (16,383 counts,
/// 36 A at this chain's measured scale) with nothing said; with it, the factor a wrong scale can
/// introduce is at most `819 / true`, and the two scales any source has claimed for this hardware
/// class both sit inside the band (455 measured here, EFeru's 800). The lower bound keeps a zero or
/// a typo from collapsing every staged limit onto the noise floor, which reads as "no drive".
pub const CURRENT_CAL_MIN: u16 = 100;
/// The calibration's upper seam bound; see [`CURRENT_CAL_MIN`].
pub const CURRENT_CAL_MAX: u16 = 819;

/// The lowest noise floor a board may claim, in counts (1,000): the bottom of the plausibility band
/// the boot seam clamps `store::MOTOR_NOISE_FLOOR` (0x6B) into, whose top is [`MAX_LIMIT_COUNTS`].
///
/// **The floor itself is per-board data and no longer a constant.** It is the rest noise of the
/// shunt and amplifier chain fitted, measured in the same counts as `motor.current_cal`, and it was
/// a compiled constant of 2,100, taken from the high-water of five rest-floor reads on ONE board in
/// ONE session (1,444 / 1,863 / 1,910 / 1,941 / 2,097 counts, medians around 1,050). That spread is
/// the argument. One number for every chain is too low for the noisy end, where it chops on the
/// chain's own noise (this constant was 2,000 until an armed STILL soak on 2026-10-09 chopped in 2
/// of 94 samples with demand and duty 0 throughout), and too high for the quiet end, where it
/// silently raises the minimum enforceable current. 2,100 is now the FIELD's default, so a board
/// with no record enforces exactly what the image enforced before
/// (`specs/store-field-audit.md`, the 2026-10-09 sweep, Tier 1 item 1).
///
/// **Why 1,000 is the bottom.** A claim below it is under the quietest rest reading any bench board
/// has produced (1,054 median, 1,444 worst of the five), so it is a typo rather than a measurement,
/// and the consequence of accepting it is a board that trips at rest. The band is a PLAUSIBILITY
/// one at the seam rather than a validated type, the shape `CURRENT_CAL_MIN` and the dead-time floor
/// both use: the store validates type only, so [`noise_floor_counts`] is where a hand-poked flash
/// value stops.
pub const NOISE_FLOOR_MIN: i16 = 1_000;

/// The converted limit's CEILING, in counts (16,383): half the sensor's full scale, the largest
/// soft limit whose hard trip ([`hard_trip_counts`], twice the limit) is still expressible in the
/// `i16` the comparison holds.
///
/// A count for the same reason as the noise floor (see [`NOISE_FLOOR_MIN`]), and one step further:
/// this bound is a property of the COMPARISON, so it is not any board's number at all. It replaces a 40,000 mA
/// ceiling, which is a count only once a scale is assumed, and which was the wrong half of the
/// constraint: it checked that the SOFT limit fits the `i16` and never that the hard trip does. At
/// 800 counts per amp a 40 A request converted to 32,000 counts, [`hard_trip_counts`] saturated at
/// 32,767, and the 2x margin the trip's whole justification rests on degenerated to 1.02x with no
/// diagnostic (`specs/store-field-audit.md`, the 2026-10-09 board-specifics sweep, Tier 1 item 2).
///
/// Clamping the converted count makes the 2x hold for every limit the window admits, and the real
/// ceiling then follows the board's own scale, which is the point: 36.0 A at the measured 455
/// counts per amp, 20.5 A at EFeru's 800, 20.0 A at the top of the calibration seam.
///
/// It is also the TOP of the noise floor's plausibility band, which is what keeps the 2x true now
/// the floor is per-board: a floor clamped to at most this bound leaves every limit
/// [`limit_counts`] can produce inside `0..=MAX_LIMIT_COUNTS` whatever a board claims. At the very
/// top the window is one value wide and the limit has no range left, which is the diagnostic a
/// board staging a floor there has asked for.
pub const MAX_LIMIT_COUNTS: i16 = i16::MAX / 2;

// What MAX_LIMIT_COUNTS is FOR, pinned here so it cannot drift from the function that depends on
// it: the hard trip of the largest admissible limit is still an i16, so `hard_trip_counts`'
// saturation is unreachable for every value `limit_counts` can produce. The runtime clamp in
// `limit_counts` is what holds the property for any stored request and scale; this pins the bound
// it clamps to.
const _: () = assert!(
    2 * MAX_LIMIT_COUNTS as i32 <= i16::MAX as i32,
    "the hard trip of the largest admissible limit no longer fits the comparison's i16"
);
// The noise floor's band, which with a per-board floor is what the 2x assert above rests on: a
// clamped floor never exceeds the ceiling, so `limit_counts`' clamp is well-formed (`i16::clamp`
// panics on an inverted pair) and its result is bounded by `MAX_LIMIT_COUNTS` for every value a
// board can stage.
const _: () = assert!(
    NOISE_FLOOR_MIN < MAX_LIMIT_COUNTS,
    "the noise floor's plausibility band is empty: its bottom has reached the limit ceiling"
);

/// The largest staged request [`limit_counts`] multiplies: an OVERFLOW guard for its 32-bit
/// arithmetic, not a policy ceiling, and derived from the window rather than from an assumed scale.
/// Capping the request here cannot change what any board enforces, because even at the lowest scale
/// the calibration seam admits this many milliamps already converts above the window's top: both
/// properties are pinned below, so neither can drift if a bound moves.
const LIMIT_REQUEST_CAP_MA: u32 = (MAX_LIMIT_COUNTS as u32 + 1) * 1_000 / CURRENT_CAL_MIN as u32;
const _: () = assert!(
    LIMIT_REQUEST_CAP_MA
        .checked_mul(CURRENT_CAL_MAX as u32)
        .is_some(),
    "the capped request times the widest scale no longer fits the conversion's u32"
);
const _: () = assert!(
    LIMIT_REQUEST_CAP_MA * CURRENT_CAL_MIN as u32 / 1_000 > MAX_LIMIT_COUNTS as u32,
    "the request cap is low enough to move an enforced limit, so it is a ceiling and not a guard"
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
/// Observation, packed by [`pack_motor_mean_duty`]: the MEAN magnitude of the same window
/// [`OBS_CURRENT`] closes, and the on-duty of the period that closed it
/// (`specs/link-control.md`, the `CYCLIC_STATE` layout, offsets 13 and 15).
///
/// Written by the period ISR in the SAME window-boundary branch as [`OBS_CURRENT`], from the same
/// `take_window` call, so a reader's peak and mean always describe one window of 64 periods. That
/// is the whole point of the mean: the peak is a maximum over ADC samples and reads high near the
/// noise floor, so a calibration cross-check compares the mean, and comparing two different
/// windows would reintroduce the noise the mean is there to remove.
///
/// Not folded into [`OBS_CURRENT`]: that word's 32 bits are spoken for (peak, chop count, trip
/// byte), and its layout is `CTRL_OBS` word 31, which is append-only.
pub static OBS_MEAN_DUTY: AtomicU32 = AtomicU32::new(0);
/// Observation, packed by [`pack_i16_pair`]: the window's MEAN conditioned phase currents, `ia`
/// low half and `ib` high half, in current counts (`specs/foc-torque.md` section 7, `CTRL_OBS`
/// word 47). From the same [`CurrentLimit::take_window`] close as [`OBS_CURRENT`], in every method:
/// the sensed pair the FOC arm's frame is built from, so a bench host can run Clarke and Park over
/// it at a held rotor's angle whatever method drives (gate O2 does, in sine). Written by the
/// period ISR at each window boundary.
pub static OBS_FOC_PHASE: AtomicU32 = AtomicU32::new(0);
/// Observation, packed by [`pack_i16_pair`]: the FOC arm's window means of the measured currents,
/// `id` low half and `iq` high half ([`commutation::foc::FocState::take_window`]; `CTRL_OBS` word
/// 48). Written by the period ISR at each window boundary; 0 while the method is not FOC.
pub static OBS_FOC_DQ: AtomicU32 = AtomicU32::new(0);
/// Observation, packed by [`pack_i16_pair`]: the FOC arm's `iq` reference of the period that
/// closed the window, low half, and the window's mean `vq` after the circular limit, high half
/// (`CTRL_OBS` word 49). Written by the period ISR at each window boundary; 0 while the method is
/// not FOC.
pub static OBS_FOC_REF: AtomicU32 = AtomicU32::new(0);
/// Observation, packed by [`pack_foc_hold`]: the hold word (`specs/foc-torque.md` section 7, round
/// 3, `CTRL_OBS` word 50): the periods of the closing window on which each sensed phase's sample
/// was held ([`SampleHold`]), the periods the bridge was floated ([`coast_posture`]), and the
/// cause byte naming which of the float's three conditions the window saw, whether a period had
/// both sensed phases held, and the offset calibration's block-pair count and verdict for this
/// boot. Written by the period ISR at each window boundary, in every method.
pub static OBS_FOC_HOLD: AtomicU32 = AtomicU32::new(0);
/// The battery-sense count (`specs/sensing-and-safety.md`, "The battery word", acquisition): the
/// injected group's THIRD rank, reduced once to the 12-bit right-aligned count (`sample >> 3`, the
/// left-aligned datum's unit, bring-up step 9's contract). Written by the period ISR every period,
/// one store and no arithmetic; read by the 250 Hz task, which converts and filters it
/// (`orchestrator::battery`). Stays 0 on a board whose group has no battery rank or never
/// converts, which the reader takes as "no conversion yet" (the word stays UNKNOWN).
pub static VBATT_RAW: AtomicU32 = AtomicU32::new(0);

/// The injected group's channel list, in rank order (`specs/motor-integration.md` bring-up step
/// 5): the two phase-current ranks from the plan (rank 0 = phase A, rank 1 = phase B, the ranks
/// the current limit reads), plus the battery rank as rank 2 when the plan carries `board.battery.pin`.
/// Two slots or three, never the stock four: the aux consumer does not exist here.
pub fn injected_ranks(phase: [u8; 2], vbatt: Option<u8>) -> heapless::Vec<u8, 4> {
    let mut ranks = heapless::Vec::new();
    // Cannot overflow: at most three pushes into a capacity-4 vector.
    for ch in phase.into_iter().chain(vbatt) {
        let _ = ranks.push(ch);
    }
    ranks
}

/// What this board's staged noise floor (`store::MOTOR_NOISE_FLOOR`, 0x6B) means to the limit: the
/// stored word clamped into [`NOISE_FLOOR_MIN`]`..=`[`MAX_LIMIT_COUNTS`], the plausibility band.
///
/// The ONE owner of that band, so [`limit_counts`] and anything that reports what the board will do
/// read the same number. The store validates type only, so this is the seam where a hand-poked or
/// typoed flash value stops, the `CURRENT_CAL_MIN` shape and the `orchestrator::battery::VbattCal`
/// precedent. Clamped rather than refused, for the reason [`limit_counts`] gives: a boot seam has
/// nowhere to refuse a value TO, and a board that refused to enforce any limit would be worse than
/// one enforcing a plausible floor.
///
/// The result is positive and at most [`MAX_LIMIT_COUNTS`], which is what [`limit_counts`]' clamp
/// needs of it (see the const asserts beside the bounds).
#[inline]
pub fn noise_floor_counts(stored: u16) -> i16 {
    stored.clamp(NOISE_FLOOR_MIN as u16, MAX_LIMIT_COUNTS as u16) as i16
}

/// Convert the staged `MOTOR_CURRENT_LIMIT` (milliamps) into the soft limit in stock current
/// counts, once, at bring-up: `ma * clamp(cal, 100, 819) / 1000`, clamped into
/// [`noise_floor_counts`]`(noise_floor)..=`[`MAX_LIMIT_COUNTS`].
///
/// **Both bounds are COUNTS, the domain the comparison works in**, and the clamp is on the RESULT,
/// not on the request: the request is milliamps, what a milliamp buys is the board's scale, and a
/// bound in milliamps is therefore a bound at one assumed scale (see [`MAX_LIMIT_COUNTS`] for what
/// that cost). A request the board's scale cannot express is clamped DOWN to the ceiling, exactly
/// as a request under the sense chain's noise is clamped UP to the floor: an out-of-range staged
/// value is bounded here, never refused, and there is nowhere at a boot seam to refuse it to.
///
/// **Two of the three inputs are this board's own measured facts, not constants.** `cal` is its
/// counts per amp (`store::MOTOR_CURRENT_CAL`, carried on `board::MotorPlan`) and `noise_floor` is
/// its sense chain's rest noise (`store::MOTOR_NOISE_FLOOR`, carried on [`BootFixed`]); this is the
/// boot seam that clamps both, because the store validates type only, so a hand-poked out-of-range
/// flash value cannot reach the comparison.
///
/// **The floor moves, the ceiling does not, and the hard trip stays twice the limit.** A board may
/// raise its own minimum enforceable current; it cannot widen the window past
/// [`MAX_LIMIT_COUNTS`], because its floor is clamped to at most that bound. So every limit this
/// can return is still in `0..=MAX_LIMIT_COUNTS` and [`hard_trip_counts`]' doubling is still exact
/// at every calibration and every claimed floor.
#[inline]
pub fn limit_counts(ma: u32, cal: u16, noise_floor: u16) -> i16 {
    let cal = cal.clamp(CURRENT_CAL_MIN, CURRENT_CAL_MAX) as u32;
    // The request is bounded only so the 32-bit multiply cannot overflow; see
    // `LIMIT_REQUEST_CAP_MA` for why it is not a ceiling in disguise.
    let counts = ma.min(LIMIT_REQUEST_CAP_MA) * cal / 1_000;
    // The window, both ends visible in one expression. The cast cannot truncate: the clamped floor
    // is positive and at most MAX_LIMIT_COUNTS, so the clamped value is in 0..=MAX_LIMIT_COUNTS.
    counts.clamp(
        noise_floor_counts(noise_floor) as u32,
        MAX_LIMIT_COUNTS as u32,
    ) as i16
}

/// The FOC arm's current-reference full scale, in counts: what a full-scale torque word (+-28,500,
/// `base::TORQUE_FULL_SCALE`) asks the q-axis loop for (`specs/foc-torque.md` section 4). The soft
/// limit [`limit_counts`] produced, less this board's noise floor ([`noise_floor_counts`]),
/// floored at zero.
///
/// **Derived, not a field**, so its three inputs (`motor.current_limit` 0x20, `motor.current_cal`
/// 0x67, `motor.noise_floor` 0x6B) each keep their one owner: a separate scale field would need a
/// validation rule against the limit, because a reference above the chop threshold makes the chop
/// and the loop fight, and one field cannot disagree with itself. EFeru's torque mode makes its
/// current limit the reference ceiling the same way.
///
/// **Why the floor comes off.** The soft chop acts on this period's phase MAGNITUDE against the
/// limit, unfiltered, and the loop regulates the vector's amplitude, whose phase peaks touch the
/// magnitude six times per electrical turn; a reference AT the limit would chop on measurement
/// noise at full demand, and one floated period moves the current by amps, which the loop then
/// chases. The noise floor is the board's own figure for exactly that band, so `limit - floor` is
/// the highest reference at which the chop stays silent and remains the backstop it is meant to be.
///
/// At the fleet defaults (10,000 mA, 455 counts per amp, floor 2,100) that is 4,550 - 2,100 = 2,450
/// counts, 5.4 A of phase-current amplitude. The limit is never below the clamped floor, so this
/// reaches zero exactly when the limit sits AT the floor, which is the board saying it cannot hold
/// any current above its noise.
#[inline]
pub fn full_scale_counts(limit_counts: i16, noise_floor: u16) -> i16 {
    // In i32, so a pair outside the window `limit_counts` produces still subtracts without wrapping;
    // the result is in `0..=MAX_LIMIT_COUNTS` for every pair it can produce.
    (limit_counts as i32 - noise_floor_counts(noise_floor) as i32)
        .max(0)
        .min(i16::MAX as i32) as i16
}

/// The hard trip's magnitude: twice the soft limit, saturated to the sensor's full scale. A
/// working chop cannot be holding a current this far over its limit (one period of float moves a
/// few amps); a shorted phase or a wrong hall table into a locked rotor can.
///
/// **The saturation is unreachable for every limit [`limit_counts`] produces**, which is what
/// [`MAX_LIMIT_COUNTS`] exists to guarantee and what the const assert beside it pins: the 2x is
/// exact across the whole admissible window, including at its top. It stays as the guard for an
/// argument this `pub fn`'s `i16` admits and the conversion cannot hand it.
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

/// Pack [`OBS_MEAN_DUTY`]: the window's `mean` magnitude (i16 counts) in bits 0..15 and `duty_on`
/// in 16..31. Two 16-bit words in one 32-bit cell so the pair is published by a single relaxed
/// store and a reader cannot catch one half of it.
#[inline(always)]
pub fn pack_motor_mean_duty(mean: i16, duty_on: u16) -> u32 {
    (mean as u16 as u32) | ((duty_on as u32) << 16)
}

/// The window's mean magnitude, as [`pack_motor_mean_duty`] packed it.
#[inline]
pub fn unpack_mean(w: u32) -> i16 {
    w as u16 as i16
}

/// The on-duty, as [`pack_motor_mean_duty`] packed it.
#[inline]
pub fn unpack_duty_on(w: u32) -> u16 {
    (w >> 16) as u16
}

/// Pack two `i16` into one 32-bit cell, `lo` in bits 0..15 and `hi` in 16..31, so a pair is
/// published by a single relaxed store and a reader cannot catch one half of it: the FOC
/// observation words ([`OBS_FOC_PHASE`], [`OBS_FOC_DQ`], [`OBS_FOC_REF`]).
#[inline(always)]
pub fn pack_i16_pair(lo: i16, hi: i16) -> u32 {
    (lo as u16 as u32) | ((hi as u16 as u32) << 16)
}

/// The peak magnitude [`pack_motor_current`] packed, so the payload builder reads the word
/// through the inverse of the pack rather than re-deriving its layout at the consumer.
#[inline]
pub fn unpack_peak(w: u32) -> i16 {
    w as u16 as i16
}

/// The ON-DUTY of one period: the largest duty actually APPLIED to the bridge, `0..ARR`.
///
/// It is the duty of the conducting leg. In six-step one phase pair conducts, the source leg at
/// the commanded duty and the sink at compare 0, so the maximum over the enabled channels IS the
/// on-duty, and the consumer's `(mean / cal) * (duty_on / ARR)` is the DC-link current a bench
/// PSU displays (`specs/rider-ui.md` 3.6).
///
/// A period whose channels are all floated reads 0, which is the physical truth rather than a
/// sentinel: a coasting period (a stale demand, or the soft current limit chopping this one) has
/// no conducting leg at all, so no duty was applied however the duty registers are still loaded.
/// This is why it takes the APPLIED enables and not the commutator's own output: the two differ
/// exactly when a guard silenced the bridge, which is the case a derived link current must not
/// read as conduction.
/// Written as three gated values rather than an indexed loop because it is inlined into the
/// 16 kHz ISR, in `.hotcode`: the loop form made the compiler spill both arrays to the stack to
/// index them, which is six stores and three iterations where this is a handful of conditional
/// moves.
#[inline(always)]
pub fn on_duty(duties: [u16; 3], enables: [bool; 3]) -> u16 {
    let applied = |i: usize| if enables[i] { duties[i] } else { 0 };
    applied(0).max(applied(1)).max(applied(2))
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
    /// The magnitude SUM of the open window, for its mean ([`CurrentLimit::take_window`]). It
    /// rides in the window the peak already closes rather than opening a second one, because the
    /// two words are only comparable if they describe the same 64 periods.
    ///
    /// A `u32` cannot overflow a window: 64 periods at [`phase_magnitude`]'s 32,767 ceiling is
    /// 2,097,088. It is also why the per-period cost is a bare add, with no saturation and no
    /// width check in the 16 kHz path.
    sum: u32,
    /// The SIGNED sums of the two conditioned phase currents over the open window, for their means
    /// ([`OBS_FOC_PHASE`], `specs/foc-torque.md` section 7). Here and not in the FOC records
    /// because they are read in every method and close with the same window: one owner closes
    /// the whole 64 periods. An `i32` holds a window of `i16` (64 x 32,768).
    sum_a: i32,
    sum_b: i32,
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
            sum: 0,
            sum_a: 0,
            sum_b: 0,
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
    /// - **observation**: `trips`, `peak`, the three sums and `chopped`, which are NOT. What makes
    ///   `trips` boot-cumulative is its own field doc ("trips so far this boot") and
    ///   [`OVER_CURRENT_TRIPS`]'s (the same words, with the ISR its sole writer), published into
    ///   `CTRL_OBS` word 31 by [`pack_motor_current`]; the current-limit bench gate
    ///   (`specs/silicon-queue.md`, gate 5) then reads it across exactly this event, ending by
    ///   re-arming after a trip and watching `trips` and the latch. A re-arm that restarted the
    ///   count would make a published counter step BACKWARDS under the one procedure that looks.
    ///   So they survive. (The spec's "append-only, offset-preserving" rule governs field OFFSETS
    ///   in that block, not counter semantics, so it is not the authority here.)
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

    /// Fold one period's two conditioned phase currents in. The magnitude is
    /// [`phase_magnitude`]`(ia, ib)`, folded here so the window has one owner. The chop is
    /// `mag > limit` (not `>=`), on this period's magnitude, unfiltered. The trip is
    /// `mag >= hard_trip_counts` or the [`OVER_CURRENT_TRIP_PERIODS`]th consecutive over-limit
    /// period.
    #[inline(always)]
    pub fn step(&mut self, ia: i16, ib: i16) -> CurrentVerdict {
        let mag = phase_magnitude(ia, ib);
        self.peak = self.peak.max(mag);
        // The window's mean accumulators: the whole per-period cost of the means, three adds,
        // before any branch, so every period of the window is in them including the ones that
        // chop. `mag` is a magnitude (`phase_magnitude` saturates to `0..=32767`), which is what
        // makes the widening cast exact and the sum monotonic.
        self.sum += mag as u32;
        self.sum_a += ia as i32;
        self.sum_b += ib as i32;
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

    /// Close the window: the packed [`OBS_CURRENT`] word for it, the window's mean magnitude, and
    /// the window's mean phase currents `(ia, ib)`, with the running maximum, the sums and the
    /// chop count restarting for the next one.
    ///
    /// ONE call closes the whole window, which is a correctness property rather than convenience:
    /// the peak and the mean are compared against each other by the consumer
    /// (`specs/rider-ui.md` 3.6), so they must come from the same 64 periods, and two separate
    /// closers could be called in different passes or one of them forgotten.
    ///
    /// The mean is a SHIFT, not a divide: the ISR closes the window every
    /// [`PERIODS_PER_TICK_NOMINAL`] periods and that count is a power of two
    /// ([`PERIODS_PER_TICK_SHIFT`], asserted against it at build time). The casts cannot
    /// truncate: the mean of magnitudes each `<= 32,767` is itself `<= 32,767`, and the mean of
    /// `i16` currents is an `i16`. The signed means are arithmetic shifts, so they round toward
    /// minus infinity, by at most one count.
    #[inline(always)]
    pub fn take_window(&mut self) -> (u32, i16, (i16, i16)) {
        let w = pack_motor_current(self.peak, self.chopped, self.trips);
        let mean = (self.sum >> PERIODS_PER_TICK_SHIFT) as i16;
        let phase = (
            (self.sum_a >> PERIODS_PER_TICK_SHIFT) as i16,
            (self.sum_b >> PERIODS_PER_TICK_SHIFT) as i16,
        );
        self.peak = 0;
        self.sum = 0;
        self.sum_a = 0;
        self.sum_b = 0;
        self.chopped = 0;
        (w, mean, phase)
    }
}

// -------------------------------------------------------------------------------------------
// The sampling instant, and the sample hold at the voltage ceiling (`specs/foc-torque.md`,
// section 14)
// -------------------------------------------------------------------------------------------

/// The ADC-trigger compare: 50 timer counts below the period (`commutation::ARR`, which `hw`'s
/// `PERIOD` is), so the CH3 compare reference
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
pub const TRIGGER_COMPARE: u16 = commutation::ARR - 50;

/// The ADC's clock divider off the APB2 clock (`ADC_CLOCK_DIV`, the `AdcClockDiv::Div6` the
/// injected group is configured with in `hw::injected_config`, derived from this number there):
/// one ADC clock is this many timer ticks, since the timer runs from the same 72 MHz.
pub const ADC_CLOCK_DIV: u32 = 6;

/// The ADC's latency from the external trigger to the start of the first rank's sampling, in ADC
/// clocks: about two on this family (the datasheet's trigger-conversion latency for an injected
/// group). 12 timer ticks, 0.17 us.
pub const ADC_TRIGGER_LATENCY_CLOCKS: u32 = 2;

/// The settle time a shunt amplifier is allowed between its leg's low side turning on and its
/// rank's sampling aperture opening, nanoseconds (`specs/foc-torque.md` section 14). Not
/// measured: the record's clean run at a 26,000 clamp had 2.0 us of it and no trips over 4.5 s at
/// speed, and this is that figure with half again as margin. Gate N2 sweeps the knob it sets.
pub const SHUNT_SETTLE_NS: u32 = 3_000;

/// The highest compare count at which a sensed phase's low-side shunt sample is taken as valid,
/// for a board whose dead time is `dead_time_dtg` (`motor.dead_time`, 0x64, in DTG codes of
/// `board::DEAD_TIME_DTS_DIV` timer ticks each). Above it the phase is held
/// ([`SampleHold::arm`]).
///
/// The derivation (`specs/foc-torque.md` section 14): rank 0's aperture opens at
/// [`TRIGGER_COMPARE`] plus the ADC's trigger latency, on the up count; a phase at compare `c` has
/// its low side turn on at `c` plus the dead time, and needs [`SHUNT_SETTLE_NS`] after that before
/// the aperture, so `c <= TRIGGER_COMPARE + latency - settle - dead_time`. Rank 1 opens 120 ticks
/// later and its own bound is looser; the stricter serves both. At the fleet's staged 28 this is
/// 2200 + 12 - 216 - 56 = **1,940**.
pub const fn hold_compare_max(dead_time_dtg: u8) -> u16 {
    let ticks_per_us = board::GATE_TIMER_HZ / 1_000_000;
    let opens = TRIGGER_COMPARE as i32 + (ADC_TRIGGER_LATENCY_CLOCKS * ADC_CLOCK_DIV) as i32;
    let settle = (SHUNT_SETTLE_NS * ticks_per_us / 1_000) as i32;
    let dead = dead_time_dtg as i32 * board::DEAD_TIME_DTS_DIV as i32;
    (opens - settle - dead) as u16
}

// The cast above is exact for every DTG code: the margin is wider than the longest dead time the
// byte can encode, so the derivation never goes negative (and it falls with the dead time, so the
// longest code is the lowest value).
const _: () = assert!(
    TRIGGER_COMPARE as u32 + ADC_TRIGGER_LATENCY_CLOCKS * ADC_CLOCK_DIV
        > SHUNT_SETTLE_NS * (board::GATE_TIMER_HZ / 1_000_000) / 1_000
            + u8::MAX as u32 * board::DEAD_TIME_DTS_DIV
);

/// The compare threshold the period ISR holds a sensed phase above, as a live bench knob
/// (`specs/foc-torque.md` section 14, the `FOC_GAINS` pattern): one `AtomicU32`, read with a
/// `Relaxed` load every period, written on a running board with `hoverboard-cli write <addr>
/// <hex>` at the address `arm-none-eabi-nm` gives for it. The bring-up stores
/// [`hold_compare_max`] of the staged dead time into it, so the board's own derivation is what
/// runs until a session pokes it, and an arm does not reset it. Until the bring-up it holds
/// `commutation::ARR`, which no compare exceeds: nothing held. The bake rule applies once gate
/// N2 confirms a value.
#[no_mangle]
pub static FOC_HOLD_COMPARE: AtomicU32 = AtomicU32::new(commutation::ARR as u32);

/// How many consecutive periods a sensed phase may be held before its raw sample is taken again
/// (`specs/foc-torque.md` section 14, "A hold that does not end"): 1,024 periods, 64 ms, 23
/// electrical turns at the record's no-load speed. A wheel that turns at all keeps its hold (the
/// bands rotate through the phases long before this); a rotor that is static with its duty at
/// the top of the count is a loop that cannot reach its reference, where the chop must not judge
/// a stale pair indefinitely, so the raw sample returns and trips if it is the transient.
pub const HOLD_MAX_PERIODS: u16 = 1024;

/// The sample hold (`specs/foc-torque.md` section 14): the period ISR's substitution of a sensed
/// phase's conditioned current by its last fresh value while that phase's leg is driven too close
/// to the top of the count for its low-side shunt to be sampled validly. The flags are decided
/// from the duties the ISR writes for the COMING period ([`SampleHold::arm`]) and consumed on
/// that period ([`SampleHold::apply`]), before the current limit and the arm see the pair, so the
/// chop, the trip and the window all judge the pair after the hold and never the transient
/// sample. Indexed as the injected ranks are: 0 is `motor.phase_a`, the leg CH0 drives; 1 is
/// `motor.phase_b`, the leg CH1 drives (the mapping contract of `specs/foc-torque.md` section 3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleHold {
    /// The last pair taken fresh, per phase.
    last: [i16; 2],
    /// Which phases the coming period holds.
    flagged: [bool; 2],
    /// Consecutive held periods, per phase, against [`HOLD_MAX_PERIODS`].
    run: [u16; 2],
    /// Periods of the open observation window on which each phase was held, for word 50
    /// ([`SampleHold::take_window`]).
    held: [u8; 2],
    /// A period of the open window held both sensed phases.
    both: bool,
}

impl Default for SampleHold {
    fn default() -> Self {
        Self::new()
    }
}

impl SampleHold {
    /// No phase flagged, no history.
    pub const fn new() -> Self {
        SampleHold {
            last: [0; 2],
            flagged: [false; 2],
            run: [0; 2],
            held: [0; 2],
            both: false,
        }
    }

    /// Close the observation window's share of the hold word: how many periods each phase was
    /// held and whether any period held both, with the counts restarting
    /// (`specs/foc-torque.md` section 7, word 50).
    #[inline(always)]
    pub fn take_window(&mut self) -> ([u8; 2], bool) {
        let w = (self.held, self.both);
        self.held = [0; 2];
        self.both = false;
        w
    }

    /// This period's pair after the hold, and whether either phase was held. A flagged phase whose
    /// run is under the cap takes its last fresh value and counts the period; any other phase
    /// takes its fresh sample, stores it and clears its run.
    #[inline(always)]
    pub fn apply(&mut self, ia: i16, ib: i16) -> ((i16, i16), bool) {
        let raw = [ia, ib];
        let mut out = raw;
        let mut any = false;
        let mut i = 0;
        while i < 2 {
            if self.flagged[i] && self.run[i] < HOLD_MAX_PERIODS {
                self.run[i] += 1;
                // A plain add: the window closes every 64 periods, so the count is at most 64.
                self.held[i] = self.held[i].wrapping_add(1);
                out[i] = self.last[i];
                // Both held: `any` is already set by phase 0 when phase 1 is held too.
                self.both |= any;
                any = true;
            } else {
                self.run[i] = 0;
                self.last[i] = raw[i];
            }
            i += 1;
        }
        ((out[0], out[1]), any)
    }

    /// Flag, for the coming period, each sensed phase whose compare in `duties` (the three
    /// channel compares the ISR is about to write, in channel order) is above `threshold`
    /// ([`FOC_HOLD_COMPARE`]).
    #[inline(always)]
    pub fn arm(&mut self, duties: [u16; 3], threshold: u16) {
        self.flagged = [duties[0] > threshold, duties[1] > threshold];
    }
}

/// The hold word's cause byte, as bits of the packed word (`specs/foc-torque.md` section 7, word
/// 50, bits 24 to 31): the bridge was disarmed on a floated period of the window.
pub const HOLD_CAUSE_DISARMED: u32 = 1 << 24;
/// A floated period of the window was the demand-freshness guard's ([`demand_stale`]).
pub const HOLD_CAUSE_STALE: u32 = 1 << 25;
/// A floated period of the window was the soft current limit's chop.
pub const HOLD_CAUSE_CHOP: u32 = 1 << 26;
/// A period of the window held both sensed phases (nothing fresh for the arm that period).
pub const HOLD_CAUSE_BOTH_HELD: u32 = 1 << 27;
/// The offset calibration's block-pair count for this boot, bits 28 and 29 (section 15: 1 to 3,
/// 0 before the calibration ran).
pub const HOLD_CAL_PAIRS_SHIFT: u32 = 28;
/// The two bits of the block-pair count, in place.
pub const HOLD_CAL_PAIRS_MASK: u32 = 0b11 << HOLD_CAL_PAIRS_SHIFT;
/// The offset calibration was refused because its block pairs never agreed (section 15).
pub const HOLD_CAL_DISAGREED: u32 = 1 << 30;

/// The cause bits of one floated period, from the three inputs [`coast_posture`] took.
#[inline(always)]
pub fn coast_causes(armed: bool, stale: bool, chop: bool) -> u32 {
    (if armed { 0 } else { HOLD_CAUSE_DISARMED })
        | (if stale { HOLD_CAUSE_STALE } else { 0 })
        | (if chop { HOLD_CAUSE_CHOP } else { 0 })
}

/// Pack the hold word (`CTRL_OBS` word 50): `held[0]` (phase a's held periods) in bits 0..7,
/// `held[1]` in 8..15, the floated periods in 16..23, and `causes`' high byte (the `HOLD_CAUSE_*`
/// and `HOLD_CAL_*` bits, which already sit at bits 24 to 31) in 24..31.
#[inline(always)]
pub fn pack_foc_hold(held: [u8; 2], coasted: u8, causes: u32) -> u32 {
    held[0] as u32 | (held[1] as u32) << 8 | (coasted as u32) << 16 | (causes & 0xFF00_0000)
}

/// The hold word's four fields back out of the packed word: `(held_a, held_b, coasted, causes)`,
/// the causes as the word's high byte in place (so the `HOLD_CAUSE_*` bits test directly). The
/// tests' inverse of the pack; the image has no reader of the word (the bench does).
#[cfg(test)]
pub fn unpack_foc_hold(w: u32) -> (u8, u8, u8, u32) {
    (w as u8, (w >> 8) as u8, (w >> 16) as u8, w & 0xFF00_0000)
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
/// `TRIGGER_START_ATTEMPTS x TRIGGER_START_SPINS` (a few tens of milliseconds; the free watchdog
/// is not running yet, `main.rs` starts it after the motor bring-up), after which it is recorded
/// as a failed step rather than left running with a dead trigger.
pub const TRIGGER_START_SPINS: u32 = 20_000;

/// Timer-triggered conversions per channel in one offset-calibration BLOCK
/// (`specs/foc-torque.md` section 15, round 4; `specs/motor-integration.md` bring-up step 9):
/// 4,096, 256 ms at 16 kHz, the count [`commutation::foc::calibrate_offset`]'s shift is derived
/// from, so the count and the accumulation cannot drift apart. Two consecutive blocks that agree
/// make a calibration ([`cal_verdict`]).
pub const CAL_SAMPLES: usize = commutation::foc::CAL_BLOCK;

/// Timer-triggered conversions awaited and discarded after the trigger start-confirm, before the
/// first block is counted (section 15, the settle): 512 periods, 32 ms once per boot, so an
/// amplifier output, its bias or the ADC's reference still moving in the first milliseconds is
/// not what the first block averages (a first block that settles disagrees with the second and
/// costs a third, 256 ms). The watchdog is brought up after the motor.
pub const CAL_SETTLE_CONVERSIONS: u32 = 512;

/// How far apart, per channel, two consecutive blocks' offsets may be and still AGREE, in the
/// offset's own counts (section 15): 192, 12 ADC LSB, 6 counts of the bench's readout unit (word
/// 25 over 32). The rover slave's zero wanders about +-5 of those counts (160 here) over seconds;
/// two 256 ms blocks straddle at most the part of a swing that fits 512 ms, and a block's noise
/// is under a count (9 LSB sd per sample over the square root of 4,096). A board whose
/// consecutive blocks differ by more than 12 LSB three times running is truly off.
pub const CAL_AGREE_COUNTS: u16 = 192;

/// Block pairs judged before the calibration is REFUSED for disagreement (section 15): 3. Pair k
/// is blocks k and k + 1, so a refusal has measured [`CAL_BLOCKS`] blocks.
pub const CAL_ATTEMPTS: u8 = 3;

/// Blocks measured at most: one more than the pairs judged, since consecutive blocks overlap as
/// pairs.
pub const CAL_BLOCKS: usize = CAL_ATTEMPTS as usize + 1;

/// Two blocks' offsets agree when each channel's two values are within [`CAL_AGREE_COUNTS`].
#[inline]
pub fn cal_agree(a: (u16, u16), b: (u16, u16)) -> bool {
    let within = |x: u16, y: u16| (x as i32 - y as i32).unsigned_abs() <= CAL_AGREE_COUNTS as u32;
    within(a.0, b.0) && within(a.1, b.1)
}

/// The mean of two agreeing blocks, per channel: the offset the calibration accepts.
#[inline]
pub fn cal_mean(a: (u16, u16), b: (u16, u16)) -> (u16, u16) {
    (
        ((a.0 as u32 + b.0 as u32) / 2) as u16,
        ((a.1 as u32 + b.1 as u32) / 2) as u16,
    )
}

/// One block of the calibration: the two channels' offsets.
pub type CalBlock = (u16, u16);

/// What the blocks measured so far decide ([`cal_verdict`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalVerdict {
    /// A pair of consecutive blocks agreed: its mean, and how many pairs it took (1 to
    /// [`CAL_ATTEMPTS`]; pair k is blocks k and k + 1).
    Agreed((u16, u16), u8),
    /// No pair has agreed yet and another block may be measured.
    Again,
    /// [`CAL_ATTEMPTS`] pairs judged, none agreed: refused, with the last block measured (the
    /// pair word 25 publishes, so a refused board reports where it sat).
    Refused((u16, u16)),
}

/// The attempt rule over the blocks measured so far, in order: the first pair of consecutive
/// blocks that agrees is the calibration, and [`CAL_ATTEMPTS`] disagreeing pairs ([`CAL_BLOCKS`]
/// blocks) refuse it (section 15). The hardware loop (`measure_offsets`) calls this after each
/// block so it can stop at the first agreement; the host tests run it over sequences.
pub fn cal_verdict(blocks: &[CalBlock]) -> CalVerdict {
    for (k, w) in blocks.windows(2).enumerate() {
        if cal_agree(w[0], w[1]) {
            return CalVerdict::Agreed(cal_mean(w[0], w[1]), k as u8 + 1);
        }
    }
    match blocks.last() {
        Some(&last) if blocks.len() >= CAL_BLOCKS => CalVerdict::Refused(last),
        _ => CalVerdict::Again,
    }
}

/// The hold word's calibration bits for a verdict (`HOLD_CAL_PAIRS_SHIFT`, `HOLD_CAL_DISAGREED`;
/// word 50's cause byte): the pair count, and the disagreement flag on a refusal. `Again` is not
/// an outcome and carries no bits.
#[inline]
pub fn cal_hold_bits(v: CalVerdict) -> u32 {
    match v {
        CalVerdict::Agreed(_, pairs) => (pairs as u32) << HOLD_CAL_PAIRS_SHIFT,
        CalVerdict::Refused(_) => {
            ((CAL_ATTEMPTS as u32) << HOLD_CAL_PAIRS_SHIFT) | HOLD_CAL_DISAGREED
        }
        CalVerdict::Again => 0,
    }
}

/// The boot time the calibration can cost at the period rate, in periods: the settle plus every
/// block up to a refusal. 16,896 periods, 1,056 ms at 16 kHz (544 ms when the first pair agrees),
/// spent in the bring-up BEFORE `main.rs` starts the free watchdog, so no timeout is in play;
/// pinned under 1,100 ms here and by the tests so the next change to a constant is priced.
pub const CAL_MAX_PERIODS: u32 = CAL_SETTLE_CONVERSIONS + CAL_SAMPLES as u32 * CAL_BLOCKS as u32;
// Under 1,100 ms of boot at the nominal period rate (section 15), pinned where the constants live.
const _: () = assert!(CAL_MAX_PERIODS * 1_000 / (PERIODS_PER_TICK_NOMINAL * 250) < 1_100);

/// Fold a block's two running sums ([`CAL_SAMPLES`] terms per channel) into the measured offset
/// pair.
///
/// The arithmetic itself is NOT here: each channel's sum is formed term by term through
/// [`commutation::foc::cal_accumulate`] and finished by [`commutation::foc::calibrate_offset`],
/// the single owner of the offset's unit, which shifts the sum of `sample >> 3` back to the
/// sixteen's unit. The injected sample is left-aligned `<< 3` (RM0008 Figure 31), so that
/// accumulation is 2x the zero-current register value, which is exactly the unit
/// [`commutation::foc::PhaseOffsets`]'s acceptance window and `current_from_adc`'s
/// `offset - 2*sample` are expressed in. This function only pairs the two channels, so the
/// pairing is host-testable without an ADC.
#[inline]
pub fn cal_offsets(sum_a: u32, sum_b: u32) -> CalBlock {
    (
        commutation::foc::calibrate_offset(sum_a),
        commutation::foc::calibrate_offset(sum_b),
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

/// The calibration's OUTCOME, read off its one product: the pair the acceptance window returned.
/// `Some` IS an accepted calibration (`commutation::foc::PhaseOffsets::try_new` is the only way to
/// get one), so the verdict is never taken twice from the raw samples.
#[inline]
pub fn cal_outcome(accepted: Option<commutation::foc::PhaseOffsets>) -> CalOutcome {
    match accepted {
        Some(_) => CalOutcome::Accepted,
        None => CalOutcome::Refused,
    }
}

/// The init-failure fault bits a calibration outcome raises (`specs/motor-integration.md`,
/// bring-up step 9 + `sensing-and-safety.md` delta (b)): a REFUSED calibration falls back to
/// six-step AND raises [`FAULT_INIT_CAL`], which [`motor_fault_level`] feeds into `fault_a`. This
/// is `commutation.md`'s validator rule; the mode machine stays method-agnostic.
///
/// The FALLBACK half of that rule is [`running_method`]'s capability gate, and these are two bits
/// saying two things. A refused calibration leaves [`BootFixed::accepted_offsets`] empty, so this
/// bit says "no trustworthy zero on either sensed phase, and a board that cannot enforce the
/// current limit does not drive" (it is a [`motor_fault_level`] producer), while [`FAULT_METHOD`]
/// beside it says "the method running is not the method stored". Since the calibration became
/// unconditional, the first applies to six-step boards too.
///
/// The `Foc`-against-`current_sense = 0` arm of the same policy never reaches this function: a
/// motor with no phase-current group is not brought up at all ([`MotorSkip::NoCurrentSense`]),
/// because the no-current-sense period vector has no registered handler, so there is no running
/// motor to fault. It is recorded distinguishably in the observation block instead, and the gate
/// covers it structurally: no bring-up means no calibration, so no accepted offsets, so no FOC.
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
    /// Enable the gate set's advanced-timer peripheral clock (`GateSet::timer`: TIMER0, or TIMER7
    /// on a second motor). Load-bearing FIRST: an unclocked GD32 advanced timer
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
    /// when the plan carries `board.battery.pin` ([`injected_ranks`]), 7.5-cycle sampling,
    /// left-aligned, the gate timer's CH3 trigger, scan mode, EOIC interrupt enabled.
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

/// The commutation method that will actually run: the CAPABILITY GATE.
///
/// All three arms are selectable. Six-step and sine are open-loop, so a request for either is
/// honoured on any brought-up motor. FOC is honoured only where the board has phase current to
/// close a loop ON, and that whole fact is [`BootFixed::accepted_offsets`]: it can only be `Some`
/// on a board that declares `motor.current_sense` (0x66), wired the two phase pins the declaration
/// is validated against (`board::MotorPlan::phase_current`), and whose quiet-bridge calibration
/// was ACCEPTED by `commutation::foc::PhaseOffsets`. A `Foc` request without it is demoted to
/// six-step and raises [`FAULT_METHOD`] ([`method_fault_bits`]), the way a board with no IMU is
/// demoted out of Balance rather than balancing on no attitude (`specs/commutation.md`,
/// "Current-sense conditioning and the FOC capability gate").
///
/// A verdict, not a second policy: it reads the method off [`method_records`], which is the one
/// place the choice is made, so what runs and what is published cannot drift apart.
///
/// Every variant is named rather than caught by a wildcard, so adding a method to the crate is a
/// compile error in [`method_records`].
///
/// **SELECTABLE IS NOT TUNED, and that lands on FOC alone.** FOC closes its q-axis loop on phase
/// current, which is scaled by `motor.current_cal` (0x67), and this fleet's 455 counts per amp is
/// an UPPER BOUND taken from a window maximum over ADC noise rather than a fit: the two two-point
/// slopes available disagree (81 and 306 counts per amp) and the sweep that would settle it has not
/// run (`specs/motor-integration.md`, "The current-sense calibration"). An over-stated scale
/// under-states the measured current, so the current loop is mistuned by whatever that factor
/// turns out to be, and it will present as a control problem rather than a calibration one.
/// Six-step and sine are open-loop and do not care. Until the sweep runs, selecting FOC is a bench
/// exercise.
///
/// The arm itself is the torque-mode current loop of `specs/foc-torque.md` (the recovered stock
/// arm, whose d-axis ramp held its command at zero for every demand this firmware can produce, is
/// deleted), and it has not yet run on silicon: that spec's bench gates O2 and O3 measure the sense
/// polarity and the phase mapping, and close the loop at a zero reference, before any torque is
/// asked of it.
///
/// (The claim this comment used to carry, that "a method never selected is a method LTO drops", was
/// FALSE. Folding both arms of `Commutator::step` to a constant shrank the release image by
/// 2,136 B, so the sine and FOC arms were in the image all along, inlined into `period_isr` with no
/// symbols of their own. LTO cannot drop them: the `MethodState` discriminant lives in a
/// `static mut`, so this verdict changes a value and not a type. Measured 2026-10-10,
/// `specs/decision-flash-budget.md`.)
#[inline]
pub fn running_method(
    requested: commutation::CommutationMethod,
    boot: BootFixed,
) -> commutation::CommutationMethod {
    // The decode and the full scale select no arm: each is carried INSIDE one arm's records, never
    // matched on, so records built for any value answer the same verdict
    // (`the_decode_selects_no_arm` pins it over every byte). The Forward, offset-0 decode and the
    // zero scale here are that fact, not defaults.
    method_records(
        requested,
        boot,
        commutation::sixstep::SixStep::new(commutation::Direction::Forward, 0),
        0,
    )
    .method()
}

/// The demotion fault bits a method selection raises: [`FAULT_METHOD`] when what RUNS is not what
/// the store asked for, nothing when it is. It asks [`running_method`] rather than taking the
/// running method as an argument, so a caller cannot report a demotion that did not happen or miss
/// one that did.
#[inline]
pub fn method_fault_bits(requested: commutation::CommutationMethod, boot: BootFixed) -> u32 {
    if running_method(requested, boot) == requested {
        0
    } else {
        FAULT_METHOD
    }
}

// -------------------------------------------------------------------------------------------
// The arm-time re-derivation (pure; `specs/integration.md`, "When a stored value takes effect:
// the arm-time re-read")
// -------------------------------------------------------------------------------------------

/// The bring-up-row facts the arm-time rebuild REUSES rather than re-reads.
///
/// The six-step decode facts used to ride here, the drive direction and the align offset, and no
/// longer do: `motor.direction` and `motor.align_offset` are value-row fields since 2026-10-20
/// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read"), re-read at
/// every arm and handed to the rebuild as one decode ([`rederive`]). The row's test is whether a
/// field touches a peripheral register, and neither does: a wheel that turns the wrong way is a
/// mounting fact discovered the first time the machine moves, and sweeping the offset 0 through 5
/// under a boot-read cost six reboots.
///
/// The noise floor stays, in a different row: it is read straight from the store at boot
/// (`specs/integration.md`, "When a stored value takes effect": the third group, values the loop
/// consumes that stay boot-read), and carrying it here is what makes the arm-time re-derivation
/// convert against the SAME floor the bring-up installed. The alternative was a half state:
/// `motor.current_limit` and `motor.current_cal` are re-read at every arm, so a floor re-read
/// beside them would be a third input to one expression taking effect at a different moment, and a
/// floor NOT carried at all would leave the arm path inventing one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BootFixed {
    /// This board's staged phase-current noise floor as STORED (`store::MOTOR_NOISE_FLOOR`, 0x6B,
    /// read at the brought-up motor's index). Raw, because [`noise_floor_counts`] owns the band and
    /// a second clamp here would be a second place for it to drift from.
    pub noise_floor: u16,
    /// This motor's quiet-bridge phase-current zeros, GATED: `Some` exactly when the board senses
    /// phase current and [`BringUpStep::CalibratePhaseOffsets`] was ACCEPTED, `None` when it was
    /// refused.
    ///
    /// It is the FOC capability gate's one input ([`running_method`]) and at the same time the one
    /// value `commutation::foc::FocState::new` takes, which is why the capability is this field and
    /// not a `bool` beside a raw pair: a FOC record cannot be built out of a capability claim, only
    /// out of accepted offsets. The raw measured pair stays separate (the ISR's own `offsets`,
    /// published in [`OBS_CAL`]), because the current limit reads both sensed phases against
    /// whatever zeros were measured, accepted or not.
    ///
    /// It rides [`BootFixed`] for the noise floor's reason and one more: the arm-time re-derivation
    /// must NOT re-measure the offsets (see `hw::install_rederived`, "Why the phase-offset
    /// calibration is NOT redone"), so the accepted pair has to be carried forward rather than
    /// re-taken. A board with no phase-current group never builds a `BootFixed` at all
    /// ([`MotorSkip::NoCurrentSense`]), which is how `current_sense = 0` reaches this gate.
    pub accepted_offsets: Option<commutation::foc::PhaseOffsets>,
}

/// The drive direction as the commutation crate names it: `reversed` is `board::MotorPlan::
/// direction` at boot and the decoded `motor.direction` byte at every arm
/// ([`requested_direction`]). It enters the six-step decode ([`six_step_records`]), and that
/// decode is what the dispatch installs (`Commutator::new` at boot, `Commutator::switch_method`
/// at every arm), so sine and FOC turn a reversed board's wheel the way its six-step does
/// (`specs/commutation.md`, "The alignment every method consumes").
#[inline]
pub fn drive_direction(reversed: bool) -> commutation::Direction {
    if reversed {
        commutation::Direction::Reverse
    } else {
        commutation::Direction::Forward
    }
}

/// Decode the `MOTOR_DIRECTION` (0x62) store byte at the arm path: `0 = Forward`, nonzero =
/// `Reverse`, the field's own rule (`store::MOTOR_DIRECTION`), the same one `board::validate`
/// applies to the boot plan's copy. A value-row input, re-read at every arm ([`rederive`]).
#[inline]
pub fn requested_direction(direction_byte: u8) -> commutation::Direction {
    drive_direction(direction_byte != 0)
}

/// The six-step records for a motor: the decode, which is the two re-read facts
/// (`motor.direction`, `motor.align_offset`) in the commutation crate's own type.
///
/// The six-step arm's one construction shape, reached from [`method_records`] both for a six-step
/// request and for every request the capability gate demotes onto it.
#[inline]
pub fn six_step_records(decode: commutation::sixstep::SixStep) -> commutation::MethodState {
    commutation::MethodState::SixStep(commutation::sixstep::SixStepState::new(decode))
}

/// The per-mode records for what the requested method will actually run.
///
/// THE one place the method choice is made. One construction shape for all three arms, shared by
/// the bring-up's [`BringUpStep::SelectMethodAndInstall`] and the arm-time [`rederive`] below, so a
/// re-armed board cannot get records built differently from the ones the boot installed, and
/// `commutation::MethodState::method()` of the result IS [`running_method`]'s verdict.
///
/// - **Six-step**: the decode config ([`six_step_records`]), `decode`, which is the only one of
///   the three arms' inputs that is not on `boot`, because its two facts (`motor.direction`,
///   `motor.align_offset`) are the ones re-read at every arm; the records are built FOR it, and
///   the caller installs the decode itself beside them (`Commutator::new`,
///   `Commutator::switch_method`), which is where the angle path reads the same two facts.
/// - **Sine**: no per-mode records at all. It reads the shared rotor front end's interpolated angle
///   and nothing else, so there is no state to build and none to reset.
/// - **FOC**: the accepted offsets as the capability's proof, the reference's full scale
///   `full_scale` ([`full_scale_counts`]; ignored by the other two arms), two fresh current-loop
///   accumulators, and the per-motor SVPWM output order. The order is `DutyOrder::DIRECT` until a
///   bench session resolves this board's phase permutation (`specs/foc-torque.md` gate O2;
///   per-board data in the board model when a board exhibits one, not a stored field). A `Foc`
///   request with no accepted offsets lands on six-step, which is the
///   capability gate's fallback and not a silent one ([`method_fault_bits`] raises
///   [`FAULT_METHOD`]). Selecting FOC is a bench exercise until the current-sense sweep runs: see
///   [`running_method`]'s "selectable is not tuned".
///
/// The records are FRESH on every build, which is the spec's method-switch reset made structural:
/// the two current-loop accumulators carry nothing across a method change or an arm, while
/// `Commutator::switch_method` deliberately leaves the shared rotor front end alone.
///
/// The match is on the REQUEST, not on a verdict, so every arm is reachable and adding a method to
/// the crate is a compile error here.
///
/// `#[inline(never)]`, MEASURED (2026-10-10, `cargo image`): its two call sites are the bring-up and
/// the arm-time re-derivation, and inlined it emitted the FOC record's seed (the 9-word stock PI
/// record, the ramp and the duty order) twice, for **212 B** of the enabling cost. Both call sites
/// are cold, one per boot and one per arm, so a call is the right trade; and keeping the build out
/// of line also keeps the optimizer from sinking it into `hw::install_rederived`'s masked region,
/// which runs with interrupts off while the 16 kHz ISR waits.
#[inline(never)]
pub fn method_records(
    requested: commutation::CommutationMethod,
    boot: BootFixed,
    decode: commutation::sixstep::SixStep,
    full_scale: i16,
) -> commutation::MethodState {
    use commutation::CommutationMethod as M;
    match (requested, boot.accepted_offsets) {
        (M::Sine, _) => commutation::MethodState::Sine,
        (M::Foc, Some(offsets)) => commutation::MethodState::Foc(commutation::foc::FocState::new(
            offsets,
            commutation::foc::DutyOrder::DIRECT,
            full_scale,
        )),
        // Six-step as asked, and the demoted `Foc` that has no current to close a loop on.
        (M::SixStep, _) | (M::Foc, None) => six_step_records(decode),
    }
}

/// EXACTLY what an arm-time re-read installs into the period ISR's record, and nothing else.
///
/// The absences are the content: no offsets (the ISR's measured quiet-bridge zeros are carried
/// through untouched, see the install; a FOC record built here embeds the pair the BOOT
/// calibration accepted, off `boot`, never a new measurement), no `base_flags`, no fault word, no
/// handle. A field here is a field the arm path writes.
#[derive(Clone, Copy, Debug)]
pub struct Rederived {
    /// The method byte the ISR publishes, read off [`Rederived::records`] rather than decided
    /// again, so a demoted request reads back as what actually runs.
    pub method: u8,
    /// The soft limit in stock current counts, re-derived through [`limit_counts`] from the stored
    /// milliamp limit, the stored per-board calibration and the boot-read per-board noise floor.
    ///
    /// A bare count rather than a built [`CurrentLimit`], deliberately: the ISR's record mixes this
    /// configuration with a boot-cumulative trip count, and a built record here could only carry a
    /// zeroed one into the install. Carrying the count means the installed value set structurally
    /// CANNOT restart an observation ([`CurrentLimit::reconfigure`] is what applies it).
    pub limit_counts: i16,
    /// Fresh per-mode records for the running method ([`method_records`]), installed through
    /// `commutation::Commutator::switch_method` so the shared rotor front end survives. A FOC
    /// record carries the arm's current-reference full scale, [`full_scale_counts`] of
    /// [`Rederived::limit_counts`] and the boot-read floor, so the scale is installed with the
    /// records, in the same masked write; it has no field of its own here, because nothing
    /// installs it separately.
    pub records: commutation::MethodState,
    /// The hall-to-phase alignment the records were built for: the re-read decode itself, over
    /// the `motor.direction` and `motor.align_offset` bytes ([`requested_direction`] plus
    /// `SixStep::new`). Installed beside the records through the same seam, so the six-step
    /// decode's state index and the angle path's offset (`commutation::angle_path_offset`) come
    /// from one read and cannot be aligned differently. Measured 2026-10-20
    /// (`specs/bench-evidence/2026-10-20/sine-phase/RECORD.md`, the direction gate): as a boot
    /// fact, writing the field and arming changed nothing and only a reset reversed the wheel.
    pub decode: commutation::sixstep::SixStep,
}

/// Re-derive the motor's VALUE row from freshly read store values plus the boot-fixed decode facts.
///
/// Pure: it validates and converts, and it touches no peripheral and no static. The five inputs
/// are the five value-row motor fields (`MOTOR_METHOD`, `MOTOR_DIRECTION`, `MOTOR_ALIGN_OFFSET`,
/// `MOTOR_CURRENT_LIMIT`, `MOTOR_CURRENT_CAL`); everything else the ISR's record holds comes from
/// `boot` or is left alone. So the value row stays twelve fields wide: `motor.noise_floor` is read
/// once, at boot.
///
/// The conversions are the boot path's own, by the same owners: [`requested_method`] +
/// [`requested_direction`] + `commutation::sixstep::SixStep::new` (the offset taken mod 6 there,
/// the decode's own rule) + [`method_records`] for the method and its records (one call, so the
/// published byte and the installed arm are the same decision, and the records and the direction
/// installed beside them are one read), [`limit_counts`] + [`CurrentLimit::new`] for the limit, and
/// [`full_scale_counts`] of that same limit for the FOC arm's reference scale, which the FOC records
/// are built with. Nothing is re-implemented here.
///
/// The limit's third input, the noise floor, comes off `boot` rather than out of the value row: it
/// is boot-read ([`BootFixed::noise_floor`] carries why), so an arm converts against the floor the
/// bring-up installed.
///
/// # Why re-reading the method is safe for the injected ADC group (the spec's first edge)
///
/// Re-reading `motor.method` rebuilds [`commutation::MethodState`], which is RAM, and NOTHING in
/// the injected group depends on the method choice. The group's rank list comes from
/// [`injected_ranks`], which takes the plan's phase-current channels and the battery channel and
/// has no method argument at all, and programming it is a BRING-UP step
/// ([`BringUpStep::ConfigureInjectedGroup`]) driven by `plan.phase_current` and `plan.vbatt` alone.
/// All three arms share that one group: the calibration is unconditional and the current limit
/// reads both sensed phases every period whatever runs, so an arm that switches to FOC starts
/// CONSUMING samples the ISR was already taking rather than asking for different ones. A method
/// that wanted different ranks would have to change [`injected_ranks`], which would make the group
/// a peripheral re-configuration and put the field back in the bring-up row; none of the three
/// does.
///
/// # Why an arm-time `Foc` cannot run on an uncalibrated board
///
/// The capability gate reads `boot.accepted_offsets`, which the BRING-UP filled and nothing since
/// has touched, so an arm can select FOC only on a board whose boot calibration was accepted. The
/// case that would need the fault bit raised here, a `Foc` write arriving on a board whose
/// calibration was refused, cannot reach a running motor: [`FAULT_INIT_CAL`] is a
/// [`motor_fault_level`] producer, so that board does not arm. That is why [`Rederived`] still
/// carries no fault word; the demotion is visible in the method byte it installs.
pub fn rederive(
    method_byte: u8,
    direction_byte: u8,
    align_offset_byte: u8,
    current_limit_ma: u32,
    current_cal: u16,
    boot: BootFixed,
) -> Rederived {
    let decode =
        commutation::sixstep::SixStep::new(requested_direction(direction_byte), align_offset_byte);
    let limit = limit_counts(current_limit_ma, current_cal, boot.noise_floor);
    let full_scale = full_scale_counts(limit, boot.noise_floor);
    let records = method_records(requested_method(method_byte), boot, decode, full_scale);
    Rederived {
        method: records.method().to_u8(),
        limit_counts: limit,
        records,
        decode,
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

    /// The dead-time generator's prescale (CKDIV), as the timer's enum: the UNIT of every stored
    /// `motor.dead_time`, since a DTG code is a time only through it.
    ///
    /// DERIVED from `board::DEAD_TIME_DTS_DIV`, the one owner of that unit, rather than written
    /// here as `ClockDiv::Div2` beside a sentence in the validator saying the same thing. The
    /// validator converts the fleet's 500 ns reference through the same number to get its floor,
    /// so a divider change moves the floor and the boot refuses a staged code that has become too
    /// short, instead of running it as a shorter dead time than it was staged for.
    /// The injected group's ADC clock divider as the HAL's enum, DERIVED from [`ADC_CLOCK_DIV`]
    /// (the one owner of the number, which the sample hold's threshold is derived from too) so
    /// the two cannot disagree. A divider the ADC does not offer is a compile error.
    const ADC_CLOCK_DIV_ENUM: AdcClockDiv = match ADC_CLOCK_DIV {
        6 => AdcClockDiv::Div6,
        _ => panic!("ADC_CLOCK_DIV is not a divider this bring-up programs"),
    };

    const CKDIV: ClockDiv = match board::DEAD_TIME_DTS_DIV {
        1 => ClockDiv::Div1,
        2 => ClockDiv::Div2,
        4 => ClockDiv::Div4,
        // The timer offers no other divider, so a `DEAD_TIME_DTS_DIV` outside this set is a
        // configuration that cannot be programmed: a compile error, not a runtime path.
        _ => panic!("DEAD_TIME_DTS_DIV is not a CKDIV the timer can be programmed with"),
    };

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
        /// The bring-up-row facts this motor was configured with ([`BootFixed`]): written once by
        /// the bring-up and never written again, so the arm-time re-derivation converts and builds
        /// against the floor and the zeros the boot installed.
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
        /// The sample hold (`specs/foc-torque.md` section 14): the flags armed from the duties
        /// this ISR wrote last period, the last fresh pair, the per-phase run, the window's counts.
        hold: SampleHold,
        /// Periods of the open window the bridge was floated ([`coast_posture`]), for word 50.
        coasted: u8,
        /// The cause bits the open window's floated periods raised ([`coast_causes`]), plus the
        /// boot's calibration bits (`HOLD_CAL_*`), which are set once and ORed in at every close.
        causes: u32,
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
    /// on the boot thread; read afterwards only by [`float_all_channels`], which runs on the
    /// 250 Hz control task and nowhere else.
    ///
    /// **Two writers of `CHCTL2` exist by construction here, and the ordering is what makes that
    /// safe.** The period ISR writes the channel enables every period through its own
    /// [`PwmHandle`]; [`float_all_channels`] writes them from the shutdown sequence. They can only
    /// overlap inside one shutdown, where MOE has ALREADY been cleared (the sequence's first step),
    /// so neither write can reach a gate driver, and the ISR itself holds the all-float posture
    /// from the next period on, because the bridge is disarmed ([`coast_posture`]). The float is
    /// the posture, not the silencing act; the silencing act is MOE.
    static mut TIMER: Option<PwmTimer> = None;

    /// The bring-up-row facts this motor was configured with, or `None` on a board with no
    /// runtime. The arm path hands them to [`rederive`] so an arm-time rebuild converts the limit
    /// against the SAME noise floor and builds FOC records on the SAME accepted zeros the boot
    /// installed.
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
    /// limit, and the per-mode records with the drive direction through
    /// `Commutator::switch_method`. `false` on a board with no runtime (which is unarmable anyway,
    /// so the arm is refused).
    ///
    /// **Those three fields and nothing else.** It does not touch `offsets`, `base_flags`,
    /// `faults`, [`FAULT`] or [`OBS_CAL`], and [`Rederived`] carries no field that could. The limit
    /// goes in through [`CurrentLimit::reconfigure`] rather than as a built record, so the
    /// boot-cumulative trip count the ISR publishes survives an arm (see that seam for the split).
    /// The records and the direction go through `switch_method` precisely because that seam
    /// replaces the per-mode records and the angle path's half-turn and deliberately leaves the
    /// SHARED rotor front end alone, so the angle, the latched speed and the hall debounce history
    /// stay continuous across an arm, a direction change included: the front-end measures the
    /// rotor, and the direction is a fact about the drive, not the rotor.
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
    /// running since the bring-up started it.
    ///
    /// **MEASURED on the built image rather than estimated** (re-measured 2026-10-23 when the
    /// align offset joined the angle path's installed offset, `cargo image`; this function inlines
    /// into `firmware::re_read_arm_values`, its `cpsid` at 0x0800_adf6, as the SECOND of that
    /// function's two masked regions; the first is [`boot_fixed`]'s read): the masked region is
    /// **75 instructions in 200 bytes between the `cpsid` and the interrupt-restore test, with no
    /// loop and no call**, up from 62 in 176 on 2026-10-20 when the direction joined it, from 57
    /// in 158 at `4cbdc27`, and from 53 in 148 when measured on 2026-10-10 as sine and FOC became
    /// selectable. The 2026-10-23 growth is `commutation::angle_path_offset`, which the optimizer
    /// sank in whole: the decode's `% 6` and the lead's add on the sector count, the shift and the
    /// `umull` divide by six that turn sectors into an angle, the direction's conditional
    /// `-0x8000`, and the halfword store into `Commutator`'s `angle_offset`. It is straight-line
    /// arithmetic on values already in registers, which is why 13 instructions cost 24 bytes.
    /// The earlier growth was the record copy: `commutation::MethodState` is as large as its biggest variant, so it is
    /// FOC-sized (56 B, three `ldmia`/`stmia` pairs) rather than six-step-sized, which is also why
    /// no later method can grow this window without growing that enum. The convention goes with
    /// the number, because an earlier figure of 46 in 132 does not reproduce on any image: the
    /// count is the BODY, from the instruction after the `cpsid` to the one before the restore
    /// test's `ldr`. It executes from flash ABOVE the F1x0's 32 KiB zero-wait line by
    /// construction (the re-read is deliberately not in `.hotcode`), where this crate's own ISR
    /// measurement prices a fetch at ~8.8 cycles per word rather than 1, so the honest figure is a
    /// few hundred cycles, order 300-500, against the 4,500-cycle period. No conversion is lost
    /// either way: the injected end-of-conversion flag is a level source, so a period whose entry
    /// is delayed inside the section is served the moment it ends.
    ///
    /// **The caveat matters more than the number: the window's CONTENTS are compiler-determined,
    /// not source-determined.** The optimizer SANK pure arithmetic into it that
    /// [`crate::arm::rederive`] computes as plain values beforehand. Visible in the disassembly of
    /// the masked region: [`limit_counts`]' calibration clamp (the 100 and 819 compares), its
    /// overflow cap (the 163,840 compare), the 32x16 multiply and the magic-number divide by 1,000,
    /// its count-window clamp (the staged floor's own band and the [`MAX_LIMIT_COUNTS`] compares),
    /// [`hard_trip_counts`]' doubling as a bare shift (the window makes the saturating branch
    /// unreachable and the optimizer drops it), and even the align-offset `% 6` the decode and
    /// `angle_path_offset` each ask for, folded together into one conditional correction of the
    /// sector count. Nothing in the source asks for that, and nothing in the source prevents it.
    ///
    /// Two consequences. A method arm whose records are larger had to RE-MEASURE this window rather
    /// than assume it was still tens of instructions, which is what the figure above is: FOC's
    /// `FocState` is the largest variant and the measurement was retaken for it. And trying to
    /// outwit the sinking (an opaque barrier, a pre-materialized local) is NOT the fix: a
    /// structural claim nobody can verify per build is worse than a measurement with its date on
    /// it. What does hold independently of codegen is that the region is branch-free and
    /// call-free, so it is bounded by its instruction count however the optimizer arranges it.
    pub fn install_rederived(r: &Rederived) -> bool {
        cortex_m::interrupt::free(|_| {
            // SAFETY: the period vector cannot fire inside this section, so the 250 Hz thread is
            // the only accessor of the runtime for its duration (the `MOTOR` invariant).
            match unsafe { (*addr_of_mut!(MOTOR)).as_mut() } {
                Some(m) => {
                    m.method = r.method;
                    m.current.reconfigure(r.limit_counts);
                    m.commutator.switch_method(r.records, r.decode);
                    true
                }
                None => false,
            }
        })
    }

    /// Float every phase (the shutdown sequence's explicit coast posture). Not inferable from a
    /// zero demand across methods, so it is applied directly.
    pub fn float_all_channels() {
        // SAFETY: read-only access to a static written once on the boot thread, from the 250 Hz
        // task, which is the only reader.
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
    /// `noise_floor` is the boot-read `MOTOR_NOISE_FLOOR` at this motor's index, the third input to
    /// that conversion and the only one of the three that is BOOT-read rather than re-read at every
    /// arm: it rides [`BootFixed`] from here so both paths convert against one floor.
    /// `vbatt` is the plan's battery-sense input: `Some` adds the battery rank to the injected
    /// group (rank 2, after the two phase ranks) and puts its pin in analog mode beside them.
    pub fn bring_up(
        chip: &Chip,
        plan: &MotorPlan,
        vbatt: Option<AdcInput>,
        method_byte: u8,
        period_hz: u32,
        current_limit_ma: u32,
        noise_floor: u16,
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
        // The advanced timer this motor's gate set belongs to, taken from the VALIDATED model
        // rather than written as a literal: `board::Capabilities::gate_set` derived the index from
        // the six gate pins, and the 12-FET's second motor is TIMER7. One resolution, one owner
        // ([`gate_timer_label`]), consumed by the clock enable, the timer config and the
        // injected-group trigger alike. Attributed to `EnableTimerClock` because that is the first
        // step that needs it. An index with no label stops the bring-up here; a TIMER7 gate set
        // gets its label and stops later, at `ConfigureInjectedGroup`, where the ADC's trigger mux
        // refuses anything but TIMER0 (see [`gate_timer_label`]). Either way the motor fails loudly
        // instead of driving TIMER0's registers under another timer's name.
        let timer_label = gate_timer_label(gates.timer)
            .ok_or(MotorSkip::StepFailed(BringUpStep::EnableTimerClock))?;
        // The policy input, read once. What actually runs is the capability gate's verdict,
        // applied at `SelectMethodAndInstall`.
        let requested = requested_method(method_byte);

        // The one configured timer object, built by `ConfigureTimer` and reused by every later
        // step that touches the timer (the trigger channel, the counter start, the per-cycle
        // handle): configuring it once is the bring-up, not a step that repeats.
        let mut timer: Option<PwmTimer> = None;
        let mut injected = None;
        // The calibration's two products, set by `CalibratePhaseOffsets`: the GATED pair (the FOC
        // capability, and the value `FocState::new` takes) and the RAW measured pair the ISR reads
        // currents against whether or not it was accepted.
        let mut cal: Option<(Option<PhaseOffsets>, (u16, u16))> = None;
        // The calibration's bits of the hold word (section 7), set by `CalibratePhaseOffsets`.
        let mut cal_bits = 0u32;

        for step in BRING_UP_STEPS {
            let failed = |s: BringUpStep| MotorSkip::StepFailed(s);
            match step {
                BringUpStep::EnableTimerClock => {
                    let rcu = chip.rcu_base().map_err(|_| failed(step))?;
                    clock::enable_timer(rcu, chip.clock(), timer_label)
                        .map_err(|_| failed(step))?;
                }
                BringUpStep::ConfigureTimer => {
                    timer = Some(
                        PwmTimer::configure(chip, &timer_config(&gates, timer_label))
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
                            &injected_config(
                                &injected_ranks(phase.channels, vbatt.map(|v| v.channel)),
                                timer_label,
                            ),
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
                        Some(verdict) => {
                            cal_bits = cal_hold_bits(verdict);
                            // The pair published and the pair the ISR reads currents against: the
                            // agreed mean, or the last block of a refused run (so a refused board
                            // reports where it sat); only an agreed pair is offered to the gate.
                            let (agreed, (a, b)) = match verdict {
                                CalVerdict::Agreed(mean, _) => (Some(mean), mean),
                                CalVerdict::Refused(last) => (None, last),
                                // Not an outcome `measure_offsets` returns; read as a refusal.
                                CalVerdict::Again => (None, (0, 0)),
                            };
                            OBS_CAL.store((a as u32) | ((b as u32) << 16), Ordering::Relaxed);
                            // The acceptance window's single owner is the commutation crate's
                            // gated newtype, so the check is ITS constructor, never a copy of its
                            // constants here, and what is carried on is the constructor's OUTPUT:
                            // the accepted pair a FOC record can be built from, or nothing.
                            (
                                agreed.and_then(|(a, b)| PhaseOffsets::try_new(a, b)),
                                (a, b),
                            )
                        }
                        // No conversion inside the poll budget: no trustworthy zero-current
                        // reference, and OBS_CAL stays zero to say the measurement itself never
                        // happened.
                        None => (None, (0, 0)),
                    });
                }
                BringUpStep::SelectMethodAndInstall => {
                    let (pwm, inj, (accepted, offsets)) = match (timer, injected, cal) {
                        (Some(t), Some(i), Some(c)) => (t.handle(), i, c),
                        _ => return Err(failed(step)),
                    };
                    let group = chip
                        .input_group([halls.a.packed(), halls.b.packed(), halls.c.packed()])
                        .map_err(|_| failed(step))?;
                    // The boot-fixed facts, kept on the runtime so the arm-time re-derivation
                    // rebuilds the records from the SAME floor and accepted zeros rather than
                    // re-measuring the bridge (`specs/integration.md`, the arm-time re-read).
                    let boot = BootFixed {
                        noise_floor,
                        accepted_offsets: accepted,
                    };
                    // The plan's direction and align offset are the boot's first values of
                    // `motor.direction` and `motor.align_offset`, read by `board::plumbing` at the
                    // same moment as the method byte; every arm re-reads both fields and installs
                    // them again (`rederive`).
                    let decode = commutation::sixstep::SixStep::new(
                        drive_direction(plan.direction),
                        plan.align_offset,
                    );
                    // The boot's soft limit, and the FOC reference scale derived from it, by the
                    // same owners the arm-time `rederive` uses.
                    let limit = limit_counts(current_limit_ma, plan.current_cal, boot.noise_floor);
                    // One construction shape for the records, shared with `motor::rederive`, and
                    // the method that runs is the records' own: a demoted request cannot publish
                    // the method it asked for.
                    let records = method_records(
                        requested,
                        boot,
                        decode,
                        full_scale_counts(limit, boot.noise_floor),
                    );
                    let method = records.method();
                    let cal = cal_outcome(accepted);
                    let base_flags = OBS_CONFIGURED
                        | OBS_CURRENT_SENSE
                        | if cal == CalOutcome::Accepted {
                            OBS_CAL_ACCEPTED
                        } else {
                            0
                        };
                    // The init-failure and method-demotion fault bits, published here and SEEDED
                    // into the ISR's own accumulator: the ISR stores its accumulator over `FAULT` every period, so an
                    // init fault the ISR did not know about would be erased on the first entry.
                    // Seeding it keeps the ISR the sole writer of `FAULT` after the unmask while
                    // still carrying a fault raised before it.
                    let init_faults = init_fault_bits(cal) | method_fault_bits(requested, boot);
                    FAULT.store(init_faults, Ordering::Relaxed);
                    // SAFETY: the one write, on the boot thread, BEFORE the period vector is
                    // unmasked (the next-but-one step), so no ISR can observe it half-built.
                    unsafe {
                        *addr_of_mut!(MOTOR) = Some(MotorRuntime {
                            pwm,
                            injected: inj,
                            halls: group,
                            commutator: Commutator::new(records, period_hz, decode),
                            boot,
                            base_flags,
                            method: method.to_u8(),
                            periods: 0,
                            since_demand: 0,
                            last_seq: DEMAND_SEQ.load(Ordering::Relaxed),
                            faults: init_faults,
                            offsets,
                            current: CurrentLimit::new(limit),
                            hold: SampleHold::new(),
                            coasted: 0,
                            causes: cal_bits,
                        });
                    }
                    // The sample hold's threshold for THIS board's dead time
                    // (`specs/foc-torque.md` section 14): stored once, here, so the derivation
                    // runs against the staged `motor.dead_time` the timer was just configured
                    // with, and the knob keeps whatever a bench session writes over it after.
                    FOC_HOLD_COMPARE
                        .store(hold_compare_max(gates.dead_time) as u32, Ordering::Relaxed);
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

    /// One block of the offset calibration: [`CAL_SAMPLES`] timer-triggered conversions per
    /// channel, each added to the channel's running sum as it is read
    /// ([`commutation::foc::cal_accumulate`]) and the two sums folded by [`cal_offsets`] into the
    /// offset unit. `None` if any conversion failed to complete inside the poll budget. The two
    /// ranks are read from the SAME conversion, so both channels come from one instant of each
    /// period. Two words of the bring-up's frame, where round 3's arrays were 128 B.
    fn measure_block(injected: &InjectedHandle) -> Option<CalBlock> {
        let (mut sum_a, mut sum_b) = (0u32, 0u32);
        for _ in 0..CAL_SAMPLES {
            if !await_conversion(injected) {
                return None;
            }
            let s = injected.read_injected();
            sum_a = commutation::foc::cal_accumulate(sum_a, s[0]);
            sum_b = commutation::foc::cal_accumulate(sum_b, s[1]);
        }
        Some(cal_offsets(sum_a, sum_b))
    }

    /// Measure the quiet-bridge phase-current zero offsets (`specs/foc-torque.md` section 15):
    /// [`CAL_SETTLE_CONVERSIONS`] conversions discarded, then blocks until two consecutive ones
    /// agree or [`CAL_BLOCKS`] have not ([`cal_verdict`], the one rule, applied after each block
    /// so the loop stops at the first agreement). `None` if any conversion failed to complete
    /// inside the poll budget; otherwise the verdict, never `Again`.
    ///
    /// The bridge is quiet by construction, not by convention: MOE is never written in this crate,
    /// so no phase can be driven while this runs.
    fn measure_offsets(injected: &InjectedHandle) -> Option<CalVerdict> {
        for _ in 0..CAL_SETTLE_CONVERSIONS {
            if !await_conversion(injected) {
                return None;
            }
        }
        let mut blocks: [CalBlock; CAL_BLOCKS] = [(0, 0); CAL_BLOCKS];
        for n in 0..CAL_BLOCKS {
            blocks[n] = measure_block(injected)?;
            match cal_verdict(&blocks[..=n]) {
                CalVerdict::Again => {}
                verdict => return Some(verdict),
            }
        }
        // Unreachable by the rule (the last block decides), kept as the refusal it would be.
        Some(CalVerdict::Refused(blocks[CAL_BLOCKS - 1]))
    }

    /// The advanced-timer label behind a validated [`board::GateSet::timer`] index: the ONE place
    /// that index becomes a HAL label (index 0 = TIMER0, 1 = TIMER7/TIM8, the numbering
    /// `board::Capabilities::gate_set` returns). `None` for anything else, which is a model
    /// mismatch rather than a board state: the fleet's parts carry at most two advanced timers.
    ///
    /// A `Timer7` gate set resolves its label here and then FAILS the bring-up at
    /// `ConfigureInjectedGroup`, and that is the honest outcome rather than a gap in this match:
    /// the HAL has the label, and it is the ADC's injected trigger mux that refuses it
    /// (`runtime_hal::adc::etsic_for_link` returns `None` for any trigger timer but TIMER0,
    /// because ADC0's injected external-trigger selector has no TIMER7 TRGO slot on these parts).
    /// So this change makes the second motor fail LOUDLY where it used to silently drive TIMER0's
    /// registers, which is the point; it does not make a second motor work. The 12-FET's second
    /// bridge needs an injected-trigger path resolved on silicon (its own ADC instance, or a
    /// timer-link the mux does offer), and that is not done.
    fn gate_timer_label(timer: u8) -> Option<PeriphLabel> {
        match timer {
            0 => Some(PeriphLabel::Timer0),
            1 => Some(PeriphLabel::Timer7),
            _ => None,
        }
    }

    /// The reconciled timer configuration (`specs/motor-integration.md` bring-up step 3 =
    /// `plan-hotpath-readiness.md` section 1's reference column, so the step list and the stage-2
    /// golden agree by construction). `timer` is the label [`gate_timer_label`] derived from this
    /// motor's validated gate set, never a literal.
    fn timer_config(gates: &board::GateSet, timer: PeriphLabel) -> PwmConfig {
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
            timer,
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
            // The dead-time generator's prescale, which is what gives `motor.dead_time` its unit
            // (`CKDIV`, derived from `board::DEAD_TIME_DTS_DIV`).
            ckdiv: CKDIV,
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
    fn injected_config(ranks: &[u8], trigger_timer: PeriphLabel) -> InjectedAdcConfig {
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
            trigger_timer,
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
            clock_div: ADC_CLOCK_DIV_ENUM,
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
    /// **596 below it** (F103, same binary: 600). `motorcontroller/firmware/memory.x` owns the placement and
    /// asserts the window at link time.
    #[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
    extern "C" fn period_isr() {
        // SAFETY: the `MOTOR` invariant (see the static), which is NOT "the ISR is the only
        // accessor after the unmask" any more: this ISR is the only accessor WHENEVER THE PERIOD
        // VECTOR CAN FIRE, and the 250 Hz arm path writes the record's value set only where it
        // cannot, inside `install_rederived`'s `cortex_m::interrupt::free`. So the accesses that
        // exist are the bring-up's write before the unmask, this ISR, and that masked install, and
        // no two of them can be live at once. A later reader deciding whether that critical section
        // is necessary should start from the install's own doc comment, not from this line.
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
        // The two conditioned phase currents, this period, from the same two samples (the stock
        // unit, against the bring-up's measured zeros): the current limit's input, and the FOC
        // arm's, which takes this pair rather than computing it a second time. Where FOC runs the
        // measured zeros ARE the accepted pair (the calibration gates the pair it measured).
        let ia = commutation::foc::current_from_adc(m.offsets.0, samples[0]);
        let ib = commutation::foc::current_from_adc(m.offsets.1, samples[1]);
        // The sample hold (`specs/foc-torque.md` section 14): a phase flagged from the duties this
        // ISR wrote last period keeps its last fresh value, so neither the limit below nor the
        // arm sees the transient a shunt reads when its leg's low side is barely on.
        let ((ia, ib), hold) = m.hold.apply(ia, ib);
        // The current limit's verdict on this period's phase-current magnitude (folded inside, with
        // the window's sums).
        let current = m.current.step(ia, ib);
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
        let stale = demand_stale(m.since_demand);
        // The posture is decided BEFORE the step, because the FOC arm needs it: all three inputs
        // are known by now, and on a period whose output will float the arm zeroes its two
        // accumulators instead of integrating measurement noise into a voltage it would apply on
        // the first armed period (`specs/foc-torque.md` section 2). A code motion only: the bridge
        // writes below keep their order (duties, then enables).
        let armed = crate::arm::hw::armed();
        let coast = coast_posture(armed, stale, current.chop);
        // Word 50's count of floated periods (at most 64 a window, a plain add) and which of the
        // three conditions floated them (zero bits on a driven period), branch-free.
        m.coasted = m.coasted.wrapping_add(coast as u8);
        m.causes |= coast_causes(armed, stale, current.chop);

        let out = m.commutator.step(levels, (ia, ib), demand, !coast, hold);
        let (duties, enables) = out.to_duties_enables();
        // Flag, for the coming period, the sensed phases these duties drive above the threshold
        // (section 14, step 13 of the chain): decided on the duties as computed, since a coasted
        // period floats every leg and its sample is not a shunt reading either way.
        m.hold
            .arm(duties, FOC_HOLD_COMPARE.load(Ordering::Relaxed) as u16);

        let applied_enables = if coast {
            // The explicit coast posture: all phases float regardless of method. It cannot be
            // inferred from a zero demand (only six-step coasts at zero demand; sine holds
            // mid-rail and FOC holds ~1125 on every phase), so the guard applies it directly, and
            // it applies it BEFORE any duty write so silencing never depends on one.
            //
            // Three producers own it ([`coast_posture`]). A disarmed bridge, which is the posture
            // the shutdown left and this ISR now has to maintain, since the shutdown leaves the
            // counter running so the hall-edge measurement stays live. The demand-freshness guard
            // (a stalled 250 Hz task). And the soft current limit, for this period only. None of
            // them touches MOE: where the bridge IS armed, it stays armed and every phase floats,
            // the current decaying through the body diodes.
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
        m.periods = m.periods.wrapping_add(1);
        PERIODS.store(m.periods, Ordering::Relaxed);
        // The current observation: one whole 64-period window per publish, so a 250 Hz reader
        // never sees a half-built peak. The mean and the on-duty ride in the same branch, from
        // the same close, so the three words a consumer compares describe one window and the
        // period that ended it (`specs/link-control.md`, the `CYCLIC_STATE` layout).
        if m.periods % PERIODS_PER_TICK_NOMINAL == 0 {
            // The odometer's edge count rides in this branch for the same reason the current
            // window does: once per 64 periods is 250 Hz, far faster than the service loop that
            // folds it, and the per-period path pays nothing (`specs/odometer.md`).
            EDGES.store(m.commutator.front().comm.edges_total, Ordering::Relaxed);
            let (current, mean, (mean_a, mean_b)) = m.current.take_window();
            OBS_CURRENT.store(current, Ordering::Relaxed);
            OBS_MEAN_DUTY.store(
                pack_motor_mean_duty(mean, on_duty(duties, applied_enables)),
                Ordering::Relaxed,
            );
            // The FOC observation (`specs/foc-torque.md` section 7): the phase means in every
            // method, from the close above; the arm's own window closed here too, and zeros where
            // the method keeps none, so a non-FOC board's words 48 and 49 read 0.
            OBS_FOC_PHASE.store(pack_i16_pair(mean_a, mean_b), Ordering::Relaxed);
            let foc = m.commutator.take_foc_window().unwrap_or_default();
            OBS_FOC_DQ.store(pack_i16_pair(foc.id, foc.iq), Ordering::Relaxed);
            OBS_FOC_REF.store(pack_i16_pair(foc.iq_ref, foc.vq), Ordering::Relaxed);
            // The hold word (section 7, word 50): the window's held and floated counts with the
            // causes, closed and restarted here; the calibration bits ride along every close.
            let (held, both) = m.hold.take_window();
            let causes = m.causes | if both { HOLD_CAUSE_BOTH_HELD } else { 0 };
            OBS_FOC_HOLD.store(pack_foc_hold(held, m.coasted, causes), Ordering::Relaxed);
            m.coasted = 0;
            m.causes &= HOLD_CAL_PAIRS_MASK | HOLD_CAL_DISAGREED;
        }
        // The front end's words, borrowed after the window branch: closing the FOC window takes
        // the commutator mutably.
        let comm = &m.commutator.front().comm;
        if comm.hall_fault {
            m.faults |= FAULT_HALL;
        }
        ANGLE.store(comm.angle as u32, Ordering::Relaxed);
        SPEED.store(comm.speed, Ordering::Relaxed);
        WHEEL_RATE_Q12.store(comm.rate_q, Ordering::Relaxed);
        POS_EDGES.store(comm.pos_edges, Ordering::Relaxed);
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

    /// This board's staged noise floor, the same way: the registered default of
    /// `store::MOTOR_NOISE_FLOOR` (0x6B), which IS the constant the field replaced, so every figure
    /// quoted below is the one the image enforced when the floor was compiled in.
    const FLOOR: u16 = store::MOTOR_NOISE_FLOOR.default();

    /// The injected group's rank list (bring-up step 5): the two-slot group where `board.battery.pin` is
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

    /// The coast posture: a disarmed bridge floats every period, whatever the other two producers
    /// say, and an armed one floats only on the two guards. The disarmed arm is what keeps the
    /// shutdown's float from being undone by the next period, now that the shutdown leaves the
    /// counter running so the hall-edge measurement stays live
    /// (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed").
    #[test]
    fn a_disarmed_bridge_coasts_every_period() {
        for stale in [false, true] {
            for chop in [false, true] {
                assert!(coast_posture(false, stale, chop), "disarmed, never driven");
            }
        }
        assert!(
            !coast_posture(true, false, false),
            "armed and healthy drives"
        );
        assert!(coast_posture(true, true, false), "the freshness guard");
        assert!(coast_posture(true, false, true), "the soft current limit");
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

    /// A STATIONARY WHEEL READS EXACTLY 0 on the control block's row, with no floor, no offset and
    /// nothing added on the way: the step-off latch's health rule tests `b_motion != 0`
    /// (`motorcontroller/state/src/fault.rs`, `FaultLatch::is_healthy`), so any nonzero the conversion
    /// invented would report a parked vehicle as rolling for as long as it sat there.
    #[test]
    fn a_stationary_wheel_reads_exactly_zero() {
        assert_eq!(wheel_speed_word(0), 0);
        // And the first edge in either direction is not swallowed by a deadband that is not there.
        assert_eq!(wheel_speed_word(1), 1);
        assert_eq!(wheel_speed_word(-1), -1);
    }

    /// The word carries the cell's own value, unrescaled, across the range an `i16` holds
    /// (`specs/link-control.md`, "`wheel_speed` carries the raw edge count").
    #[test]
    fn the_word_is_the_edge_count_unrescaled() {
        for s in [
            7,
            -7,
            300,
            -300,
            32766,
            -32767,
            i16::MAX as i32,
            i16::MIN as i32,
        ] {
            assert_eq!(wheel_speed_word(s) as i32, s, "edge count {s} rescaled");
        }
    }

    /// SATURATION, where a cast would WRAP, and the case that matters is the wrap onto 0: a plain
    /// `as i16` turns 65,536 edges into exactly 0, the one value the step-off latch reads as "not
    /// moving". The clamp lands on a rail, which is nonzero, for every input.
    #[test]
    fn the_word_saturates_where_a_cast_would_wrap_onto_zero() {
        assert_eq!(
            65_536_i32 as i16, 0,
            "the wrap this conversion must not have"
        );
        assert_eq!(wheel_speed_word(65_536), i16::MAX);
        assert_eq!(wheel_speed_word(-65_536), i16::MIN);
        assert_eq!(wheel_speed_word(i16::MAX as i32 + 1), i16::MAX);
        assert_eq!(wheel_speed_word(i16::MIN as i32 - 1), i16::MIN);
        assert_eq!(wheel_speed_word(i32::MAX), i16::MAX);
        assert_eq!(wheel_speed_word(i32::MIN), i16::MIN);
        // Every multiple of 65,536 is a cast's wrap onto 0; none of them is the word's.
        for k in 1..=8_i32 {
            let edges = k * 65_536;
            assert_eq!(edges as i16, 0, "a cast wraps {edges} onto 0");
            assert_ne!(wheel_speed_word(edges), 0);
            assert_ne!(wheel_speed_word(-edges), 0);
        }
    }

    /// The two consumers of the one cell read it the same way: the OFF-inhibit level
    /// (`crate::arm::off_inhibit`) and the block row's zero test can never disagree about whether
    /// the wheel is turning, because the narrowing preserves zero-ness in both directions. The
    /// 250 Hz shell derives both from a single load for the same reason.
    ///
    /// Taken on a LIVE measurement, which is the only case where the two are claims about the same
    /// thing: a shortfall makes the inhibit unconditional while the block row keeps reporting the
    /// last closed window, and that divergence is the point of it rather than a disagreement.
    #[test]
    fn the_off_inhibit_level_and_the_block_rows_zero_test_agree() {
        for s in [
            0,
            1,
            -1,
            9,
            -9,
            i16::MAX as i32,
            i16::MIN as i32,
            65_536,
            -65_536,
            i32::MAX,
            i32::MIN,
        ] {
            assert_eq!(
                crate::arm::off_inhibit(true, true, s),
                wheel_speed_word(s) != 0,
                "the inhibit level and the block row disagree at {s}"
            );
        }
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

    /// **Each method requested yields its OWN records**, on a board that can run all three: the
    /// store byte selects the arm, and the arm's records are the ones that arm needs. An unknown
    /// byte is six-step, by `CommutationMethod::from_u8`.
    #[test]
    fn each_method_requested_yields_its_own_records() {
        use commutation::CommutationMethod as M;
        let boot = boot_accepted();
        for (byte, want) in [
            (0u8, M::SixStep),
            (1, M::Sine),
            (2, M::Foc),
            (99, M::SixStep),
        ] {
            let r = method_records(requested_method(byte), boot, DECODE, FULL_SCALE);
            assert_eq!(
                running_method(requested_method(byte), boot),
                want,
                "byte {byte}"
            );
            assert_eq!(
                r.method(),
                want,
                "byte {byte}: the records are that method's"
            );
            assert_eq!(
                method_fault_bits(requested_method(byte), boot),
                0,
                "byte {byte}: nothing was demoted, so nothing faults"
            );
            // The records are the arm's own shape, not a six-step record wearing a label.
            match (want, r) {
                (M::SixStep, commutation::MethodState::SixStep(_)) => {}
                (M::Sine, commutation::MethodState::Sine) => {}
                (M::Foc, commutation::MethodState::Foc(_)) => {}
                (w, got) => panic!("byte {byte}: wanted {w:?} records, got {got:?}"),
            }
        }
    }

    /// **The selected arm is the arm the ISR steps**, observed through the one dispatch the period
    /// ISR uses (`commutation::Commutator::step`) rather than through the record's type alone. The
    /// three postures at ZERO demand are distinct by design and that is what identifies the live
    /// arm: six-step coasts (all phases float), sine holds every phase at mid-rail, and FOC drives
    /// all three (it never floats one). A board whose records were six-step wearing another
    /// method's label would fail this.
    #[test]
    fn the_selected_arm_is_the_one_the_dispatch_steps() {
        use commutation::{Commutator, PhaseCmd};
        let boot = boot_accepted();
        // A valid hall code, so six-step has a pattern to decode rather than a sensor fault.
        let halls = [1u8, 0, 0];
        let step = |byte: u8| {
            let mut c = Commutator::new(
                method_records(requested_method(byte), boot, DECODE, FULL_SCALE),
                16_000,
                DECODE,
            );
            c.step(halls, (0, 0), 0, true, false).phases
        };
        assert_eq!(
            step(0),
            [PhaseCmd::Float; 3],
            "six-step coasts at zero demand"
        );
        assert_eq!(
            step(1),
            [PhaseCmd::Drive(commutation::MID_RAIL); 3],
            "sine holds mid-rail at zero demand"
        );
        assert!(
            step(2).iter().all(|p| matches!(p, PhaseCmd::Drive(_))),
            "FOC drives all three phases, always"
        );
    }

    /// **The records are FRESH on every build**, which is the spec's method-switch reset
    /// (`specs/commutation.md`, "The mode model": the per-mode records are replaced wholesale, the
    /// shared front end is not). Asserted where it can carry: FOC's two current-loop accumulators
    /// and its observation sums are the only per-mode state in the image that integrates, so a
    /// stepped record must not be what a rebuild returns.
    #[test]
    fn a_rebuild_resets_the_per_method_records() {
        use commutation::foc::foc_step;
        use commutation::CommutationMethod as M;
        let boot = boot_accepted();
        let fresh = || match method_records(M::Foc, boot, DECODE, FULL_SCALE) {
            commutation::MethodState::Foc(st) => st,
            other => panic!("the FOC request builds FOC records, got {other:?}"),
        };
        // Two builds agree...
        let (mut a, b) = (fresh(), fresh());
        assert_eq!(std::format!("{a:?}"), std::format!("{b:?}"));
        // ...and a stepped record does not, so the rebuild is a reset and not a reuse.
        for _ in 0..8 {
            foc_step(&mut a, 0x2000, (300, -200), 20_000, true, false);
        }
        assert_ne!(a.q_acc(), 0, "the q loop integrated");
        assert_ne!(
            std::format!("{a:?}"),
            std::format!("{:?}", fresh()),
            "a carried current-loop accumulator would survive a method change"
        );
    }

    /// **The decode selects no arm.** [`running_method`] reads its verdict off records built for
    /// the Forward, offset-0 decode; this pins that records built for the other direction and
    /// every other offset carry the same verdict over every store byte and both capability states,
    /// so that literal is a fact about the match and not a hidden default.
    #[test]
    fn the_decode_selects_no_arm() {
        use commutation::sixstep::SixStep;
        use commutation::Direction;
        for boot in [BOOT, boot_accepted()] {
            for offset in 0..6u8 {
                for byte in 0..=u8::MAX {
                    let requested = requested_method(byte);
                    assert_eq!(
                        method_records(
                            requested,
                            boot,
                            SixStep::new(Direction::Reverse, offset),
                            FULL_SCALE,
                        )
                        .method(),
                        running_method(requested, boot),
                        "byte {byte}, offset {offset}"
                    );
                }
            }
        }
    }

    /// **The direction and align-offset bytes are live in the re-derivation, for every arm**
    /// (`specs/integration.md`, "When a stored value takes effect": both are in the value row).
    /// Direction byte 0 is Forward and every other byte is Reverse (the field's rule); the offset
    /// byte is taken as `SixStep::new` takes it (mod 6, so 7 is 1). The six-step records are built
    /// for the pair, and the direction carried beside the records, which the angle path installs,
    /// is the same decode's, on a board that runs all three arms.
    #[test]
    fn the_decode_bytes_are_rederived_for_every_arm() {
        use commutation::sixstep::SixStep;
        use commutation::{Direction, MethodState};
        let boot = boot_accepted();
        for method_byte in [0u8, 1, 2] {
            for (direction_byte, want) in [
                (0u8, Direction::Forward),
                (1, Direction::Reverse),
                (0xFF, Direction::Reverse),
            ] {
                for align_offset_byte in [0u8, 3, 5, 7] {
                    let r = rederive(
                        method_byte,
                        direction_byte,
                        align_offset_byte,
                        10_000,
                        CAL,
                        boot,
                    );
                    assert_eq!(
                        r.decode.direction(),
                        want,
                        "method {method_byte}, direction byte {direction_byte}"
                    );
                    if let MethodState::SixStep(st) = r.records {
                        assert_eq!(
                            st.decode,
                            SixStep::new(want, align_offset_byte),
                            "the six-step decode is built for the same direction and offset \
                             (offset byte {align_offset_byte})"
                        );
                    }
                }
            }
        }
    }

    /// **The verdict and the records cannot disagree**, over every store byte and both capability
    /// states: [`running_method`] is defined as the records' own method, so the byte the ISR
    /// publishes is the arm the ISR steps.
    #[test]
    fn the_published_method_is_the_one_that_runs() {
        for boot in [BOOT, boot_accepted()] {
            for byte in 0..=u8::MAX {
                let requested = requested_method(byte);
                let records = method_records(requested, boot, DECODE, FULL_SCALE);
                assert_eq!(
                    records.method(),
                    running_method(requested, boot),
                    "byte {byte}"
                );
                // ...and the re-derivation publishes that same method, not the request.
                let r = rederive(byte, 0, 2, 10_000, CAL, boot);
                assert_eq!(r.method, records.method().to_u8(), "byte {byte}");
                assert_eq!(r.records.method(), records.method(), "byte {byte}");
                // The observation block carries it in the method byte the bench reads.
                let w = pack_obs_state(0, [false; 3], r.method, OBS_CONFIGURED);
                assert_eq!((w >> 16) & 0xFF, r.method as u32, "byte {byte}");
            }
        }
    }

    /// **FOC is gated on the board declaring phase-current sense** (`specs/commutation.md`,
    /// "Current-sense conditioning and the FOC capability gate"). With no accepted zeros, which is
    /// what a board with `motor.current_sense = 0` and a board with a refused calibration both
    /// present as, a stored `Foc` runs SIX-STEP and raises the demotion fault rather than closing a
    /// current loop on no current. The open-loop arms are unaffected: they need no current sense,
    /// so neither is demoted and neither faults.
    #[test]
    fn foc_without_accepted_offsets_falls_back_with_the_method_fault() {
        use commutation::CommutationMethod as M;
        assert_eq!(running_method(M::Foc, BOOT), M::SixStep);
        assert_eq!(method_fault_bits(M::Foc, BOOT), FAULT_METHOD);
        let r = method_records(M::Foc, BOOT, DECODE, FULL_SCALE);
        assert_eq!(r.method(), M::SixStep);
        let (commutation::MethodState::SixStep(st), commutation::MethodState::SixStep(fresh)) =
            (r, six_step_records(DECODE))
        else {
            panic!("the demotion lands on six-step's own records");
        };
        assert_eq!(st, fresh, "demoted, not half-built");
        // The open-loop arms are not gated on current sense.
        for m in [M::SixStep, M::Sine] {
            assert_eq!(running_method(m, BOOT), m);
            assert_eq!(method_fault_bits(m, BOOT), 0);
        }
        // With the zeros accepted, the same request is honoured.
        assert_eq!(running_method(M::Foc, boot_accepted()), M::Foc);
        assert_eq!(method_fault_bits(M::Foc, boot_accepted()), 0);
        // The demotion is a READBACK, not a shutdown: six-step is what a six-step board runs.
        assert_eq!(motor_fault_level(true, FAULT_METHOD, false, false), 0);
    }

    /// A quiet-bridge offset pair inside `commutation::foc::PhaseOffsets`' acceptance window: the
    /// bench's own stage-3 measurement. Not a `const`, because the window's gate is a function.
    fn accepted() -> commutation::foc::PhaseOffsets {
        commutation::foc::PhaseOffsets::try_new(0x7E00, 0x7F20).expect("in window")
    }

    /// A board's boot-fixed facts, as a bring-up with an ACCEPTED calibration would have built
    /// them: current sense present and trusted zeros, so every method is available.
    fn boot_accepted() -> BootFixed {
        BootFixed {
            accepted_offsets: Some(accepted()),
            ..BOOT
        }
    }

    /// A board's boot-fixed facts with NO accepted zeros: either the calibration was refused, or
    /// the board senses no phase current at all and never reached one. FOC is unavailable.
    const BOOT: BootFixed = BootFixed {
        noise_floor: FLOOR,
        accepted_offsets: None,
    };

    /// A six-step decode as an arm would have re-read it: Forward, align offset 2.
    const DECODE: commutation::sixstep::SixStep =
        commutation::sixstep::SixStep::new(commutation::Direction::Forward, 2);

    /// The FOC reference full scale the fleet defaults derive (10,000 mA at 455 counts per amp is a
    /// 4,550-count limit, less the 2,100 floor), for the tests that only need a FOC record built.
    /// `the_foc_full_scale_is_the_limit_less_the_floor` pins the arithmetic.
    const FULL_SCALE: i16 = 2_450;

    /// **The FOC reference's full scale is the staged limit less the board's noise floor**
    /// (`specs/foc-torque.md` section 4), re-derived at every arm beside the limit from the same
    /// three inputs, and the FOC records are built with it.
    #[test]
    fn the_foc_full_scale_is_the_limit_less_the_floor() {
        use commutation::MethodState;
        // The scale the FOC records an arm-time re-derivation built carry.
        let scale = |r: Rederived| match r.records {
            MethodState::Foc(st) => st.full_scale(),
            other => panic!("an accepted board runs the FOC it asked for, got {other:?}"),
        };
        // The fleet defaults: 4,550 - 2,100 = 2,450 counts, 5.4 A at 455 counts per amp.
        let r = rederive(2, 0, 2, 10_000, CAL, boot_accepted());
        assert_eq!(r.limit_counts, 4_550);
        assert_eq!(full_scale_counts(r.limit_counts, FLOOR), 2_450);
        assert_eq!(scale(r), 2_450, "the FOC records carry it");
        assert_eq!(scale(r), FULL_SCALE);
        // Each of the three inputs moves it, through its one owner: the limit (17,800 mA is the
        // spec's 13.2 A reference), the calibration, and the boot-read floor.
        assert_eq!(
            scale(rederive(2, 0, 2, 17_800, CAL, boot_accepted())),
            5_999
        );
        assert_eq!(
            scale(rederive(2, 0, 2, 10_000, 800, boot_accepted())),
            8_000 - 2_100
        );
        let quiet = BootFixed {
            noise_floor: 1_500,
            ..boot_accepted()
        };
        assert_eq!(scale(rederive(2, 0, 2, 10_000, CAL, quiet)), 3_050);
        // Floored at zero when the limit sits AT the floor (a request under the floor is clamped up
        // to it): the board can hold no current above its noise, so the reference is zero.
        let r = rederive(2, 0, 2, 1, CAL, boot_accepted());
        assert_eq!(r.limit_counts, noise_floor_counts(FLOOR));
        assert_eq!(scale(r), 0);
        // The function itself, at the window's ends and past them (a pair `limit_counts` cannot
        // produce still floors rather than wrapping).
        assert_eq!(
            full_scale_counts(MAX_LIMIT_COUNTS, FLOOR),
            MAX_LIMIT_COUNTS - 2_100
        );
        assert_eq!(full_scale_counts(0, FLOOR), 0);
        assert_eq!(full_scale_counts(i16::MIN, u16::MAX), 0);
        // The demotion rule and its fault are unchanged: no accepted zeros, a FOC request runs
        // six-step and raises FAULT_METHOD.
        let r = rederive(2, 0, 2, 10_000, CAL, BOOT);
        assert_eq!(r.method, commutation::CommutationMethod::SixStep.to_u8());
        assert!(matches!(r.records, MethodState::SixStep(_)));
        assert_eq!(
            method_fault_bits(commutation::CommutationMethod::Foc, BOOT),
            FAULT_METHOD
        );
    }

    /// **The current limit is recomputed at arm, through `limit_counts`.** Its two value-row inputs
    /// move it, and every clamp end is the conversion's own (the calibration seam range, and the
    /// count window's floor and ceiling).
    ///
    /// The window's floor comes off `boot`, not out of the value row, so this also pins that an arm
    /// converts against the floor the BRING-UP read: a board whose runtime carries one floor cannot
    /// have its minimum enforceable current moved by an arm.
    #[test]
    fn the_current_limit_is_reconverted_at_arm() {
        // The re-derivation carries the COUNT (see `Rederived::limit_counts`), and the count is
        // `limit_counts`'s, so a changed calibration moves it...
        let at = |ma: u32, cal: u16| {
            rederive(0, 0, 2, ma, cal, BOOT).limit_counts == limit_counts(ma, cal, FLOOR)
        };
        assert_ne!(
            limit_counts(20_000, 200, FLOOR),
            limit_counts(20_000, 400, FLOOR),
            "the per-board calibration is part of the conversion"
        );
        assert!(at(20_000, 200) && at(20_000, 400));
        // ...and so does a changed milliamp limit.
        assert_ne!(
            limit_counts(10_000, 455, FLOOR),
            limit_counts(20_000, 455, FLOOR)
        );
        assert!(at(10_000, 455) && at(20_000, 455));
        // The clamp ends, through the same owner: a tiny request floors at this board's own floor, a
        // calibration outside the seam clamps into it, and an unbounded request ceilings at
        // MAX_LIMIT_COUNTS.
        assert_eq!(
            rederive(0, 0, 2, 1, 455, BOOT).limit_counts,
            noise_floor_counts(FLOOR)
        );
        assert_eq!(
            rederive(0, 0, 2, 20_000, 0, BOOT).limit_counts,
            limit_counts(20_000, CURRENT_CAL_MIN, FLOOR)
        );
        assert_eq!(
            rederive(0, 0, 2, 20_000, u16::MAX, BOOT).limit_counts,
            limit_counts(20_000, CURRENT_CAL_MAX, FLOOR)
        );
        assert_eq!(
            rederive(0, 0, 2, u32::MAX, 455, BOOT).limit_counts,
            MAX_LIMIT_COUNTS
        );
        // The floor the arm uses is the runtime's, not the registered default: a board brought up
        // with a quieter chain floors lower at every arm, and a noisier one floors higher.
        for floor in [NOISE_FLOOR_MIN as u16, FLOOR, 4_000] {
            let boot = BootFixed {
                noise_floor: floor,
                ..BOOT
            };
            assert_eq!(
                rederive(0, 0, 2, 1, 455, boot).limit_counts,
                noise_floor_counts(floor),
                "floor {floor}"
            );
        }
        // And the count the re-derivation carries IS the limit in force once installed: a record
        // reconfigured to it behaves exactly as one built with it (the install's own seam below).
        for (ma, cal) in [(10_000u32, 455u16), (20_000, 200), (1, 455)] {
            let counts = rederive(0, 0, 2, ma, cal, BOOT).limit_counts;
            let mut reconfigured = CurrentLimit::new(noise_floor_counts(FLOOR));
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
        assert!(c.step(hard_trip_counts(lim), 0).trip);
        assert_eq!(c.trips(), 1);
        // Re-arm with a different limit.
        let fresh = 8_000i16;
        c.reconfigure(fresh);
        assert_eq!(
            c.trips(),
            1,
            "the boot-cumulative trip count survives the arm"
        );
        // The open window's running maximum and chop count survived too (the same class of
        // observation). Pinned with a LOWER magnitude after the arm, deliberately: a magnitude at
        // or above the old peak would re-establish it by itself, and the assertion would then hold
        // even against a reconfigure that had zeroed it.
        assert!(!c.step(100, 0).chop, "100 counts is under the new limit");
        assert_eq!(
            c.take_window().0,
            pack_motor_current(hard_trip_counts(lim), 1, 1),
            "the window's peak and chop count carried across the reconfigure"
        );
        // The NEW limit is the one in force, and so is the new hard trip, both observable through
        // `step`: a magnitude at the new limit no longer chops, and the new hard trip is the one
        // that trips.
        assert!(!c.step(fresh, 0).chop, "at the new limit: drives");
        assert!(
            c.step(fresh + 1, 0).chop,
            "one count over the new limit: floats"
        );
        let v = c.step(hard_trip_counts(fresh), 0);
        assert!(v.trip, "the new hard trip is in force");
        assert_eq!(c.trips(), 2, "and it counts ON TOP of the old count");
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
        assert!(c.step(hard_trip_counts(lim), 0).trip);
        let published_before = c.trips();
        assert_eq!(published_before, 1);
        // The arm: the value row is re-derived and installed (same limit here, since the operator
        // changed something else), and the bridge was disarmed for the whole sequence.
        c.reconfigure(lim);
        // The first period after the arm, with the short still there.
        let v = c.step(hard_trip_counts(lim), 0);
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
        assert!(c.step(hard_trip_counts(lim), 0).trip);
        assert!(
            !c.step(hard_trip_counts(lim), 0).trip,
            "same episode, one trip"
        );
        assert_eq!(c.trips(), 1);
        c.reconfigure(lim);
        // The same magnitude now counts as a NEW episode's trip.
        assert!(
            c.step(hard_trip_counts(lim), 0).trip,
            "the episode in flight was judged against the replaced limit"
        );
        assert_eq!(c.trips(), 2);
    }

    /// **A method byte rebuilds the records, and the rebuild carries the re-read decode.** For
    /// every six-step byte: the published method is the decoded request as the capability gate
    /// honours it, and the rebuilt records behave exactly as a freshly constructed state for the
    /// direction and align-offset bytes' decode, over every hall code and both demand signs. The
    /// direction the re-derivation carries beside the records is the one they were built for.
    #[test]
    fn a_method_byte_rebuilds_the_records_from_the_re_read_decode() {
        use commutation::sixstep::{sixstep_step, SixStep, SixStepState};
        use commutation::Direction;
        use commutation::MethodState;
        for (direction_byte, direction) in [(0u8, Direction::Forward), (1, Direction::Reverse)] {
            for align_offset in 0..6u8 {
                let fresh = SixStepState::new(SixStep::new(direction, align_offset));
                // The six-step bytes: 0, and every unknown byte `from_u8` folds onto it. 1 and 2
                // select their own arms, which `each_method_requested_yields_its_own_records`
                // covers; here the decode facts are the subject.
                for byte in (0..=u8::MAX).filter(|b| !matches!(b, 1 | 2)) {
                    let r = rederive(byte, direction_byte, align_offset, 10_000, 455, BOOT);
                    assert_eq!(r.decode.direction(), direction, "byte {byte}");
                    assert_eq!(
                        r.method,
                        running_method(requested_method(byte), BOOT).to_u8(),
                        "byte {byte} must read back as what runs, not what was asked"
                    );
                    let MethodState::SixStep(st) = r.records else {
                        panic!("byte {byte} is a six-step byte");
                    };
                    assert_eq!(
                        r.records.method(),
                        running_method(requested_method(byte), BOOT)
                    );
                    assert_eq!(st, fresh, "fresh records for the re-read decode");
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

    /// The REQUESTED method is decoded faithfully whether or not the capability gate honours it, so
    /// a demotion in [`running_method`] is a visible POLICY act rather than a parse that lost the
    /// request. That is also what lets the observation block report what runs instead of claiming a
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
    /// `sample >> 3` into a running sum as `measure_block` forms it, which is 2x the zero-current
    /// sample as the register presents it. The pairing is this crate's; the arithmetic is
    /// `commutation::foc::calibrate_offset`'s, and this test pins that the two channels are not
    /// swapped and the unit is not rescaled here.
    #[test]
    fn cal_offsets_accumulates_through_the_commutation_owner() {
        use commutation::foc::{cal_accumulate, calibrate_offset};

        // A constant stream of one register value, folded as the hardware loop folds it:
        // 16 x (sample >> 3) = 2 x that value, whatever the block length.
        let stream = |reg: u16| (0..CAL_SAMPLES).fold(0u32, |acc, _| cal_accumulate(acc, reg));
        assert_eq!(
            cal_offsets(stream(0x3FD8), stream(0x3ED0)),
            (0x7FB0, 0x7DA0)
        );
        // The bench's own measurement (`04-master-cal-4por.log`): register means 0x3F00 / 0x3F90
        // land mid-window once accumulated, which is the unit the acceptance gate reads.
        let (a, b) = cal_offsets(stream(0x3F00), stream(0x3F90));
        assert_eq!((a, b), (0x7E00, 0x7F20));
        assert!(commutation::foc::PhaseOffsets::try_new(a, b).is_some());
        // It is NOT the mean of the raw registers: that is half the accumulated offset, and it
        // falls outside the window (the slice-4 bench failure, in one assertion).
        assert!(commutation::foc::PhaseOffsets::try_new(0x3F00, 0x3F90).is_none());
        // Delegation, channel order included: each half is exactly the owner's accumulation.
        assert_eq!(
            cal_offsets(stream(0x3F00), stream(0x3F90)),
            (
                calibrate_offset(stream(0x3F00)),
                calibrate_offset(stream(0x3F90))
            )
        );
        assert_eq!(cal_offsets(0, 0), (0, 0));
    }

    /// **Two consecutive blocks that agree make a calibration; three pairs that do not refuse
    /// it** (`specs/foc-torque.md` section 15, round 4): `cal_agree` at the tolerance's edge both
    /// ways, the tolerance against the measured wander, `cal_mean` of two blocks, the fold over
    /// sequences of blocks (an agreeing first pair accepted with one counted, a disagreeing pair
    /// then an agreeing one with two, a third pair agreeing with three, three disagreeing pairs
    /// over four blocks refused with the last block, fewer blocks asking for another), the hold
    /// word's bits for each verdict, an agreeing mean outside the window still refused by the
    /// gate, and the boot budget of the whole thing under 1,100 ms at 16 kHz.
    #[test]
    fn two_agreeing_blocks_calibrate_and_three_disagreeing_pairs_refuse() {
        let mid = (0x7FB0u16, 0x7DA0u16);
        let edge = |d: i32| ((mid.0 as i32 + d) as u16, mid.1);
        assert!(cal_agree(mid, mid));
        assert!(cal_agree(mid, edge(CAL_AGREE_COUNTS as i32)));
        assert!(cal_agree(mid, edge(-(CAL_AGREE_COUNTS as i32))));
        assert!(!cal_agree(mid, edge(CAL_AGREE_COUNTS as i32 + 1)));
        assert!(!cal_agree(mid, edge(-(CAL_AGREE_COUNTS as i32) - 1)));
        // Either channel alone can break the agreement.
        assert!(!cal_agree(mid, (mid.0, mid.1 + CAL_AGREE_COUNTS + 1)));
        // The tolerance is 6 counts of the bench's readout unit (32 offset counts each): the rover
        // slave's +-5 count wander (160) fits, and round 3's 128 did not.
        assert_eq!(CAL_AGREE_COUNTS, 6 * 32);
        assert!(cal_agree(mid, edge(160)));
        assert!(cal_agree(mid, edge(-160)));
        assert_eq!(cal_mean((100, 200), (104, 203)), (102, 201));
        assert_eq!(cal_mean(mid, mid), mid);
        // The rover slave's round-3 error, as one block 300 off: the pair still disagrees.
        let off = (mid.0 + 300, mid.1);
        assert!(!cal_agree(mid, off));
        // The fold over consecutive blocks: pair k is blocks k and k + 1.
        let near = edge(40);
        assert_eq!(cal_verdict(&[]), CalVerdict::Again);
        assert_eq!(cal_verdict(&[mid]), CalVerdict::Again);
        assert_eq!(
            cal_verdict(&[mid, near]),
            CalVerdict::Agreed(cal_mean(mid, near), 1)
        );
        assert_eq!(cal_verdict(&[mid, off]), CalVerdict::Again);
        assert_eq!(
            cal_verdict(&[mid, off, mid]),
            CalVerdict::Again,
            "two disagreeing pairs ask for a fourth block"
        );
        // A disagreeing pair then an agreeing one: the agreeing pair's mean, two counted.
        let off_near = (off.0 + 40, off.1);
        assert_eq!(
            cal_verdict(&[mid, off, off_near]),
            CalVerdict::Agreed(cal_mean(off, off_near), 2)
        );
        assert_eq!(
            cal_verdict(&[mid, off, mid, near]),
            CalVerdict::Agreed(cal_mean(mid, near), 3)
        );
        assert_eq!(cal_verdict(&[mid, off, mid, off]), CalVerdict::Refused(off));
        // The first agreeing pair decides, even when a later one would too.
        assert_eq!(
            cal_verdict(&[mid, near, off, off_near]),
            CalVerdict::Agreed(cal_mean(mid, near), 1)
        );
        assert_eq!(CAL_ATTEMPTS, 3);
        assert_eq!(CAL_BLOCKS, 4);
        // The hold word's bits.
        assert_eq!(
            cal_hold_bits(cal_verdict(&[mid, near])),
            1 << HOLD_CAL_PAIRS_SHIFT
        );
        assert_eq!(
            cal_hold_bits(cal_verdict(&[mid, off, off_near])),
            2 << HOLD_CAL_PAIRS_SHIFT
        );
        assert_eq!(
            cal_hold_bits(cal_verdict(&[mid, off, mid, off])),
            (3 << HOLD_CAL_PAIRS_SHIFT) | HOLD_CAL_DISAGREED
        );
        assert_eq!(cal_hold_bits(CalVerdict::Again), 0);
        // An agreeing pair whose mean sits outside the window is still refused by the gate.
        let low = (0x7000u16, mid.1);
        if let CalVerdict::Agreed((a, b), _) = cal_verdict(&[low, low]) {
            assert!(commutation::foc::PhaseOffsets::try_new(a, b).is_none());
        } else {
            panic!("an agreeing pair");
        }
        // The boot budget: the settle plus every block up to a refusal, 1,056 ms, 544 when the
        // first pair agrees (the under-1,100-ms pin is the compile-time assertion beside
        // `CAL_MAX_PERIODS`); spent before the free watchdog starts.
        assert_eq!(CAL_SAMPLES, 4096);
        assert_eq!(CAL_MAX_PERIODS, 512 + 4096 * 4);
        assert_eq!(
            CAL_MAX_PERIODS * 1_000 / (PERIODS_PER_TICK_NOMINAL * 250),
            1_056
        );
        assert_eq!(
            (CAL_SETTLE_CONVERSIONS + 2 * CAL_SAMPLES as u32) * 1_000
                / (PERIODS_PER_TICK_NOMINAL * 250),
            544
        );
    }

    /// **A refused calibration still blocks FOC** (`specs/motor-integration.md`, bring-up step 9):
    /// the refusal raises the init-failure fault AND leaves the capability gate with no accepted
    /// zeros, so a stored `Foc` runs six-step. Both halves of one policy, with the outcome read off
    /// the acceptance window's own product so the two cannot be decided differently.
    #[test]
    fn refused_cal_falls_back_to_six_step_and_raises_the_init_fault() {
        use commutation::CommutationMethod as M;
        assert_eq!(cal_outcome(None), CalOutcome::Refused);
        assert_eq!(cal_outcome(Some(accepted())), CalOutcome::Accepted);
        assert_eq!(init_fault_bits(CalOutcome::Refused), FAULT_INIT_CAL);
        assert_eq!(init_fault_bits(CalOutcome::Accepted), 0);
        // A refused calibration is a board with no accepted zeros, which is the gate's input.
        let refused = BootFixed {
            accepted_offsets: None,
            ..BOOT
        };
        assert_eq!(running_method(M::Foc, refused), M::SixStep);
        assert_eq!(
            init_fault_bits(cal_outcome(refused.accepted_offsets))
                | method_fault_bits(M::Foc, refused),
            FAULT_INIT_CAL | FAULT_METHOD,
            "the bring-up seeds both bits: no trusted zero, and a demoted method"
        );
        // The init fault is a fault_a producer on a configured motor, so such a board does not
        // arm, which is also why an arm-time `Foc` cannot reach a running motor uncalibrated.
        assert_eq!(
            motor_fault_level(true, FAULT_INIT_CAL, false, false),
            EV_MOTOR_CAL
        );
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
        assert_eq!(motor_fault_level(false, 0, true, false), 0);
        assert_eq!(requested_method(2), commutation::CommutationMethod::Foc);
        // The gate covers it a second way, structurally: no bring-up means no calibration, so no
        // accepted zeros, so a `Foc` request has nothing to build a current loop on.
        assert_eq!(BOOT.accepted_offsets, None);
        assert_eq!(
            running_method(commutation::CommutationMethod::Foc, BOOT),
            commutation::CommutationMethod::SixStep
        );
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
        // The four producers, each on its own cause bit.
        assert_eq!(
            motor_fault_level(true, FAULT_HALL, false, false),
            EV_MOTOR_HALL,
            "hall dwell"
        );
        assert_eq!(
            motor_fault_level(true, FAULT_INIT_CAL, false, false),
            EV_MOTOR_CAL,
            "refused cal"
        );
        assert_eq!(
            motor_fault_level(true, 0, true, false),
            EV_MOTOR_PERIOD,
            "period-liveness loss"
        );
        assert_eq!(
            motor_fault_level(true, 0, false, true),
            EV_MOTOR_ARM,
            "a refused arm"
        );
        // Several at once are reported together, not collapsed.
        assert_eq!(
            motor_fault_level(true, FAULT_HALL | FAULT_INIT_CAL, true, true),
            orchestrator::EV_MOTOR_BITS
        );
        // Not producers: the freshness guard has already floated every phase itself, and a refused
        // duty left the outputs untouched. Both stay observable in the motor block.
        assert_eq!(motor_fault_level(true, FAULT_DEMAND_STALE, false, false), 0);
        assert_eq!(motor_fault_level(true, FAULT_DUTY_RANGE, false, false), 0);
        // Nothing configured: nothing produced, whatever the words say.
        assert_eq!(
            motor_fault_level(false, FAULT_HALL | FAULT_INIT_CAL, true, true),
            0
        );
    }

    /// The balance law's torque-mode flag is the published method byte of a CONFIGURED motor:
    /// set for FOC only, with every flag the ISR or the 250 Hz task can add, and clear for a
    /// failed bring-up whose step index happens to be the FOC discriminant.
    #[test]
    fn the_torque_mode_flag_is_a_configured_motor_running_foc() {
        let foc = commutation::CommutationMethod::Foc.to_u8();
        let live = OBS_CONFIGURED | OBS_CURRENT_SENSE | OBS_CAL_ACCEPTED;
        for flags in [OBS_CONFIGURED, live, live | OBS_COASTING | OBS_PERIOD_LIVE] {
            for (method, want) in [(0u8, false), (1, false), (foc, true), (3, false)] {
                let w = pack_obs_state(5, [true, true, true], method, flags);
                assert_eq!(obs_torque_is_current(w), want, "{method} {flags:#x}");
            }
        }
        // The step index rides the method byte on a skip.
        let failed = pack_obs_state(0, [false; 3], foc, OBS_SKIP_STEP_FAILED);
        assert!(!obs_torque_is_current(failed));
        assert!(!obs_torque_is_current(0), "before the bring-up publishes");
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

    /// The boot conversion at its clamps, both of them COUNTS: the noise floor, the ceiling that
    /// keeps the hard trip's 2x expressible, and the scale as the per-board value it now is.
    #[test]
    fn limit_counts_at_the_clamps() {
        assert_eq!(
            CAL, 455,
            "0x67's registered default, the 2026-10-09 bench figure"
        );
        assert_eq!(
            limit_counts(10_000, CAL, FLOOR),
            4_550,
            "the registered limit, 10 A"
        );
        assert_eq!(
            limit_counts(15_000, CAL, FLOOR),
            6_825,
            "the walk tool's round-trip value, 15 A"
        );
        // The floor is a count, so a milliamp request worth less than it is clamped UP: at this
        // scale 4,615 mA converts to 2,099 counts, one under the default floor of 2,100.
        assert_eq!(
            FLOOR, 2_100,
            "0x6B's registered default, the constant it replaced"
        );
        assert_eq!(limit_counts(0, CAL, FLOOR), 2_100, "the floor");
        assert_eq!(limit_counts(4_615, CAL, FLOOR), 2_100);
        assert_eq!(
            limit_counts(4_700, CAL, FLOOR),
            2_138,
            "clear of the floor, converts straight through"
        );
        // The ceiling is a COUNT, so where it bites in milliamps is this board's scale: 16,383
        // counts is 36.0 A at 455 counts per amp, and a request above that clamps DOWN to it.
        assert_eq!(
            limit_counts(36_006, CAL, FLOOR),
            16_382,
            "one count under the ceiling"
        );
        assert_eq!(
            limit_counts(36_007, CAL, FLOOR),
            MAX_LIMIT_COUNTS,
            "exactly the ceiling"
        );
        assert_eq!(
            limit_counts(36_008, CAL, FLOOR),
            MAX_LIMIT_COUNTS,
            "over the ceiling reads as the ceiling"
        );
        assert_eq!(limit_counts(40_000, CAL, FLOOR), MAX_LIMIT_COUNTS);
        assert_eq!(
            limit_counts(u32::MAX, CAL, FLOOR),
            MAX_LIMIT_COUNTS,
            "no overflow"
        );
        // The same ceiling in the units the owner reads it in, at the three scales the sweep
        // quoted: 36.0 A here, 20.5 A at EFeru's 800, 20.0 A at the top of the calibration seam.
        for (cal, deci_amps) in [(CAL, 360u32), (800, 205), (CURRENT_CAL_MAX, 200)] {
            let amps_x10 = (MAX_LIMIT_COUNTS as u32 * 10 + cal as u32 / 2) / cal as u32;
            assert_eq!(amps_x10, deci_amps, "the real ceiling at cal {cal}");
            assert_eq!(limit_counts(u32::MAX, cal, FLOOR), MAX_LIMIT_COUNTS);
        }
        // The same request against two other boards' scales: the conversion is the board's, not
        // the firmware's. 800 is what the compiled constant used to assert for every board.
        assert_eq!(limit_counts(10_000, 800, FLOOR), 8_000);
        assert_eq!(limit_counts(10_000, 300, FLOOR), 3_000);
    }

    /// The calibration's own seam (`CURRENT_CAL_MIN`..`CURRENT_CAL_MAX`), clamped at the boot
    /// conversion because the store validates type only. The count window bounds the limit in
    /// absolute terms; this band is what bounds the RATIO between the labelled limit and the
    /// enforced one, which is the only thing a wrong scale can move once the result is clamped.
    #[test]
    fn the_calibration_is_clamped_into_its_seam_range() {
        // Above the seam, at a request the count ceiling does not reach, so the seam is what is
        // visible: a scale an order out would otherwise convert this 5 A staging to the widest
        // limit the window admits (16,383 counts, 36 A at the measured scale), silently.
        assert_eq!(limit_counts(5_000, CURRENT_CAL_MAX, FLOOR), 4_095);
        assert_eq!(
            limit_counts(5_000, 8_190, FLOOR),
            4_095,
            "a scale an order out"
        );
        assert_eq!(limit_counts(5_000, CURRENT_CAL_MAX + 1, FLOOR), 4_095);
        assert_eq!(
            limit_counts(5_000, 1_000, FLOOR),
            4_095,
            "a plausible typo, still bounded"
        );
        assert_eq!(limit_counts(5_000, u16::MAX, FLOOR), 4_095);
        // Below it: an unset or typoed 0x67 cannot collapse every staged limit onto the floor.
        assert_eq!(
            limit_counts(40_000, 0, FLOOR),
            4_000,
            "clamped up to CURRENT_CAL_MIN"
        );
        assert_eq!(limit_counts(40_000, CURRENT_CAL_MIN - 1, FLOOR), 4_000);
        assert_eq!(limit_counts(40_000, CURRENT_CAL_MIN, FLOOR), 4_000);
        // And at the top of the seam the conversion is still inside the window, with the hard
        // trip's 2x intact: that is the count ceiling's job, not the seam's.
        let top = limit_counts(u32::MAX, CURRENT_CAL_MAX, FLOOR);
        assert_eq!(top, MAX_LIMIT_COUNTS);
        assert_eq!(hard_trip_counts(top), 2 * top);
    }

    // --- The sample hold (`specs/foc-torque.md` section 14) ---------------------------------

    /// The staged limit every hold test below judges against: the fleet defaults, 4,550 counts,
    /// a hard trip at 9,100.
    fn limit() -> CurrentLimit {
        CurrentLimit::new(limit_counts(10_000, CAL, FLOOR))
    }

    #[test]
    fn the_hold_threshold_is_derived_from_the_sampling_instant() {
        // The offroad pair's staged dead time (28 DTG codes of two ticks): 2200 + 12 - 216 - 56.
        assert_eq!(hold_compare_max(28), 1_940);
        // Spelled from the constants, so a moved trigger or a changed allowance moves it.
        let opens = TRIGGER_COMPARE as u32 + ADC_TRIGGER_LATENCY_CLOCKS * ADC_CLOCK_DIV;
        let settle = SHUNT_SETTLE_NS * (board::GATE_TIMER_HZ / 1_000_000) / 1_000;
        assert_eq!(opens, 2_212);
        assert_eq!(settle, 216);
        assert_eq!(
            hold_compare_max(28) as u32,
            opens - settle - 28 * board::DEAD_TIME_DTS_DIV
        );
        // Two counts per DTG code: the 6-FET board's 25 sits six higher.
        assert_eq!(hold_compare_max(25), 1_946);
        assert_eq!(hold_compare_max(0), 1_996);
        // Until the bring-up derives it the knob holds the period itself, which no compare exceeds.
        assert_eq!(
            FOC_HOLD_COMPARE.load(Ordering::Relaxed),
            commutation::ARR as u32
        );
    }

    #[test]
    fn the_flags_come_from_the_previous_calls_duties_and_lag_them_by_one_call() {
        let mut h = SampleHold::new();
        // Nothing armed: both fresh, and both stored.
        assert_eq!(h.apply(300, -200), ((300, -200), false));
        // Phase A's leg above the threshold for the coming period, B's not.
        h.arm([1_941, 1_940, 2_250], 1_940);
        // ...so the NEXT sample of A is replaced by its last fresh value; B is taken as read.
        assert_eq!(h.apply(32_767, -210), ((300, -210), true));
        // The flags stand until re-armed: still held, still the same last value.
        assert_eq!(h.apply(-32_767, -220), ((300, -220), true));
        // Re-armed below the threshold: the next sample is fresh again, and becomes the value.
        h.arm([1_125, 1_125, 1_125], 1_940);
        assert_eq!(h.apply(310, -230), ((310, -230), false));
        // Both sensed legs above: both held; the third leg is not sensed and never flags.
        h.arm([2_000, 2_000, 0], 1_940);
        assert_eq!(h.apply(1, 2), ((310, -230), true));
        h.arm([0, 0, 2_250], 1_940);
        assert_eq!(h.apply(1, 2), ((1, 2), false));
    }

    #[test]
    fn a_held_phases_transient_reaches_neither_the_chop_nor_the_trip_nor_the_peak() {
        let mut h = SampleHold::new();
        let mut lim = limit();
        // A clean period at 300 counts, stored.
        let ((ia, ib), _) = h.apply(300, -150);
        assert!(!lim.step(ia, ib).chop);
        // Phase A's leg driven above the threshold next period; its shunt then reads a rail.
        h.arm([2_100, 900, 400], 1_940);
        let ((ia, ib), held) = h.apply(32_767, -150);
        assert!(held);
        let v = lim.step(ia, ib);
        assert_eq!((v.chop, v.trip), (false, false));
        // The window saw the held value, not the transient.
        let (word, _, (mean_a, _)) = lim.take_window();
        assert_eq!(unpack_peak(word), 300);
        assert_eq!(mean_a, (300 * 2) >> PERIODS_PER_TICK_SHIFT);
        // The same sample unflagged trips at once (32,767 is past the 9,100 hard trip).
        let mut h = SampleHold::new();
        let mut lim = limit();
        let ((ia, ib), held) = h.apply(32_767, -150);
        assert!(!held);
        let v = lim.step(ia, ib);
        assert_eq!((v.chop, v.trip), (true, true));
    }

    #[test]
    fn a_hold_is_released_on_the_1025th_period_and_the_fresh_sample_rearms_it() {
        let mut h = SampleHold::new();
        assert_eq!(h.apply(500, 0), ((500, 0), false));
        h.arm([2_200, 0, 0], 1_940);
        for k in 0..HOLD_MAX_PERIODS {
            assert_eq!(h.apply(-32_767, 0), ((500, 0), true), "period {k}");
        }
        // The cap: the raw sample is taken, stored, and judged by whoever reads it.
        assert_eq!(h.apply(-32_767, 0), ((-32_767, 0), false));
        // The flag still stands, so the next period holds again, on the sample just stored.
        assert_eq!(h.apply(123, 0), ((-32_767, 0), true));
        // A phase that is not flagged is never counted: B's run stays clear throughout.
        h.arm([0, 0, 0], 1_940);
        assert_eq!(h.apply(7, 9), ((7, 9), false));
    }

    /// **The hold word packs the window's counts and causes and the counts restart at the close**
    /// (`specs/foc-torque.md` section 7, word 50): the per-phase held counts come from
    /// `SampleHold`'s own applications, a period holding both phases raises its bit, the three
    /// float conditions raise theirs, and the calibration bits survive a close where the window's
    /// own bits do not.
    #[test]
    fn the_hold_word_packs_the_windows_counts_and_causes() {
        let mut h = SampleHold::new();
        h.arm([2_000, 0, 0], 1_940);
        for _ in 0..5 {
            h.apply(1, 2);
        }
        h.arm([2_000, 2_000, 0], 1_940);
        for _ in 0..3 {
            h.apply(1, 2);
        }
        h.arm([0, 2_000, 0], 1_940);
        h.apply(1, 2);
        let (held, both) = h.take_window();
        assert_eq!((held, both), ([8, 4], true));
        assert_eq!(
            h.take_window(),
            ([0, 0], false),
            "the close restarts the counts"
        );
        // A window with a lone phase held never raises the both bit.
        h.arm([2_000, 0, 0], 1_940);
        h.apply(1, 2);
        assert_eq!(h.take_window(), ([1, 0], false));
        // The cause bits, one per input of `coast_posture`, none for a driven period's inputs.
        assert_eq!(coast_causes(true, false, false), 0);
        assert_eq!(coast_causes(false, false, false), HOLD_CAUSE_DISARMED);
        assert_eq!(coast_causes(true, true, false), HOLD_CAUSE_STALE);
        assert_eq!(coast_causes(true, false, true), HOLD_CAUSE_CHOP);
        assert_eq!(
            coast_causes(false, true, true),
            HOLD_CAUSE_DISARMED | HOLD_CAUSE_STALE | HOLD_CAUSE_CHOP
        );
        // The pack, and its inverse.
        let causes = HOLD_CAUSE_STALE | HOLD_CAUSE_BOTH_HELD | (2 << HOLD_CAL_PAIRS_SHIFT);
        let w = pack_foc_hold([18, 27], 5, causes);
        assert_eq!(w, 0x2A05_1B12);
        assert_eq!(unpack_foc_hold(w), (18, 27, 5, causes));
        // Nothing below bit 24 in `causes` leaks into the counts, and a full window packs.
        assert_eq!(pack_foc_hold([64, 64], 64, 0x00FF_FFFF), 0x0040_4040);
        assert_eq!(
            pack_foc_hold([0, 0], 0, u32::MAX) >> 24,
            0xFF,
            "the whole cause byte is carried"
        );
        // The bits sit where section 7 says.
        assert_eq!(HOLD_CAUSE_DISARMED, 1 << 24);
        assert_eq!(HOLD_CAUSE_STALE, 1 << 25);
        assert_eq!(HOLD_CAUSE_CHOP, 1 << 26);
        assert_eq!(HOLD_CAUSE_BOTH_HELD, 1 << 27);
        assert_eq!(HOLD_CAL_PAIRS_MASK, 0x3000_0000);
        assert_eq!(HOLD_CAL_DISAGREED, 1 << 30);
    }

    /// The SVPWM bands the threshold draws, from the crate's own arithmetic (`specs/foc-torque.md`
    /// section 14, "How much, and how stale"): at the circular limit's radius the compare reaches
    /// 2,227 at the sector middles, where that leg's low side never conducts at the staged dead
    /// time, each phase is above 1,940 for about 36 % of the electrical turn and both sensed phases
    /// at once for about 2.6 %; at a magnitude of 23,000 nothing is above it; at the record's
    /// clean clamp of 26,000 each phase is above it for about 26 % and never both.
    #[test]
    fn the_svpwm_bands_above_the_threshold_at_the_radius() {
        use commutation::foc::{circular_limit, park_inverse, svpwm, CIRC_THRESH};
        let thr = hold_compare_max(28);
        let radius = (CIRC_THRESH as f64).sqrt() as i16; // 32,111
        let sweep = |mag: i16| {
            let (mut max, mut above, mut both, mut n) = (0u16, [0u32; 3], 0u32, 0u32);
            for t in (0..=u16::MAX).step_by(16) {
                let (vq, vd) = circular_limit(mag, 0);
                let (a, b) = park_inverse(vq, vd, t);
                let s = svpwm(a, b);
                let d = [s.base, s.c1, s.c2];
                max = max.max(d[0]).max(d[1]).max(d[2]);
                for i in 0..3 {
                    above[i] += (d[i] > thr) as u32;
                }
                both += (d[0] > thr && d[1] > thr) as u32;
                n += 1;
            }
            (
                max,
                above.map(|a| a as f64 / n as f64),
                both as f64 / n as f64,
            )
        };
        let (max, above, both) = sweep(radius);
        std::println!("radius: max compare {max}, above {above:?}, both {both}");
        assert_eq!(max, 2_227);
        // The low-side window of that leg, ticks: 2 (2250 - c) minus the 56-tick dead time.
        let dead_ticks = 28 * board::DEAD_TIME_DTS_DIV as i32;
        assert!(2 * (commutation::ARR as i32 - max as i32) - dead_ticks < 0);
        for frac in above {
            assert!((0.34..0.38).contains(&frac), "band fraction {frac}");
        }
        assert!((0.015..0.04).contains(&both), "both sensed phases {both}");
        let (max, above, _) = sweep(23_000);
        assert!(max <= thr, "max compare {max} at 23,000");
        assert_eq!(above, [0.0; 3]);
        // The record's clean clamp, 26,000: each phase above for about 94 degrees, never both.
        let (max, above, both) = sweep(26_000);
        std::println!("26,000: max compare {max}, above {above:?}, both {both}");
        assert_eq!(max, 2_018);
        for frac in above {
            assert!((0.25..0.28).contains(&frac), "band fraction {frac}");
        }
        assert_eq!(both, 0.0);
    }

    /// **The hard trip is twice the soft limit for every limit the window admits, exactly, and the
    /// top of the window is where that is tightest.**
    ///
    /// This test used to assert the opposite, and that is the defect it now pins: it read
    /// `hard_trip_counts(limit_counts(40_000, CAL)) == 32_767`, "the 40 A limit's 2x saturates at
    /// this board's scale too". A saturated hard trip is not a hard trip: at 800 counts per amp the
    /// 40 A the milliamp ceiling permitted converted to 32,000 counts and the trip sat 1.02x above
    /// the limit it is supposed to stand twice clear of, with nothing said anywhere
    /// (`specs/store-field-audit.md`, the 2026-10-09 sweep, Tier 1 item 2).
    #[test]
    fn the_hard_trip_is_twice_every_limit_the_window_admits() {
        // Every count any board's window can admit, the top included, and the bottom is now the
        // band's bottom rather than one board's measured floor: a quieter chain enforces limits
        // below the old constant, and the 2x has to be exact there too. No saturation anywhere.
        for limit in NOISE_FLOOR_MIN..=MAX_LIMIT_COUNTS {
            assert_eq!(hard_trip_counts(limit), 2 * limit, "limit {limit}");
        }
        // The tightest point, stated as its own figure: the ceiling's 2x is one count inside the
        // comparison's i16, which is what MAX_LIMIT_COUNTS is chosen to make true.
        assert_eq!(MAX_LIMIT_COUNTS, 16_383);
        assert_eq!(hard_trip_counts(MAX_LIMIT_COUNTS), 32_766);
        assert!(hard_trip_counts(MAX_LIMIT_COUNTS) < i16::MAX);
        // The case the old assertion was built on, now exact rather than degenerate.
        let was_degenerate = hard_trip_counts(limit_counts(40_000, CAL, FLOOR));
        assert_eq!(was_degenerate, 32_766);
        assert_eq!(was_degenerate, 2 * limit_counts(40_000, CAL, FLOOR));
        // The saturation survives as the guard on an argument this `pub fn`'s i16 admits and
        // `limit_counts` cannot produce.
        assert_eq!(
            hard_trip_counts(MAX_LIMIT_COUNTS + 1),
            i16::MAX,
            "saturates"
        );
        assert_eq!(hard_trip_counts(i16::MAX), i16::MAX);
    }

    /// The property the degenerate assertion hid, walked rather than sampled: across the whole
    /// staged-milliamp range at five scales AND at seven staged noise floors, the converted limit
    /// stays inside the count window and its hard trip is exactly twice it, the ceiling included.
    /// With a milliamp ceiling and no count ceiling this fails at every scale at or above 410 counts
    /// per amp, the measured 455 among them.
    ///
    /// **The floors are what keep this meaningful now the window's bottom is per-board.** The walk
    /// would prove nothing against one hardcoded floor, so it runs the band's bottom, its top, the
    /// registered default, the spread the bench actually measured on 2026-10-09, and two values
    /// OUTSIDE the band in each direction (0 and `u16::MAX`), which is what a hand-poked flash word
    /// can be. The window's bottom moves with the floor and its top never does, so the 2x is exact
    /// at every combination: that relation is the thing a per-board floor must not be able to break.
    #[test]
    fn the_2x_holds_across_the_range_at_every_scale_and_every_staged_floor() {
        for cal in [CURRENT_CAL_MIN, 300, CAL, 800, CURRENT_CAL_MAX] {
            for staged in [
                0u16,
                NOISE_FLOOR_MIN as u16,
                1_444,
                FLOOR,
                4_000,
                16_383,
                u16::MAX,
            ] {
                let floor = noise_floor_counts(staged);
                let mut reached_ceiling = false;
                // The staged field is a u32 of milliamps: 0 to 200 A in 10 mA steps, then the
                // extremes. Stepped coarsely at the floors that are only there for their clamp.
                let step = if staged == FLOOR { 10 } else { 1_000 };
                let steps = (0..=200_000).step_by(step);
                for ma in steps.chain([u32::MAX / 2, u32::MAX - 1, u32::MAX]) {
                    let limit = limit_counts(ma, cal, staged);
                    assert!(
                        (floor..=MAX_LIMIT_COUNTS).contains(&limit),
                        "cal {cal}, floor {staged}, {ma} mA converted outside the window: {limit}"
                    );
                    assert_eq!(
                        hard_trip_counts(limit),
                        2 * limit,
                        "cal {cal}, floor {staged}, {ma} mA: the hard trip is not twice the limit"
                    );
                    reached_ceiling |= limit == MAX_LIMIT_COUNTS;
                }
                assert!(
                    reached_ceiling,
                    "cal {cal}, floor {staged}: the sweep never reached the ceiling"
                );
            }
        }
    }

    /// The noise floor's plausibility band: the seam clamps the stored word into
    /// `NOISE_FLOOR_MIN..=MAX_LIMIT_COUNTS`, and that is the ONLY bound on it, because the store
    /// validates type only.
    ///
    /// Both ends are reachable states of a real board, not hypotheticals: 0 is an unwritten record
    /// read by a tool that forgot the default, and a 16-bit word poked over the mailbox can hold
    /// anything. A board claiming a floor under the quietest rest reading the bench has produced
    /// would chop on its own noise; one claiming a floor past the ceiling would have no window left.
    #[test]
    fn the_noise_floor_is_clamped_into_its_plausibility_band() {
        assert_eq!(NOISE_FLOOR_MIN, 1_000);
        // Inside the band, including both ends: carried straight through.
        for staged in [NOISE_FLOOR_MIN as u16, 1_444, 2_097, FLOOR, 16_383] {
            assert_eq!(noise_floor_counts(staged), staged as i16, "staged {staged}");
        }
        // Below it: a typo rather than a measurement, clamped UP to the band's bottom.
        for staged in [0u16, 1, 999] {
            assert_eq!(
                noise_floor_counts(staged),
                NOISE_FLOOR_MIN,
                "staged {staged}"
            );
        }
        // Above it: clamped DOWN to the limit ceiling, which is what keeps `limit_counts`' clamp
        // well-formed and its result inside the i16 the hard trip's 2x needs.
        for staged in [16_384u16, 32_768, u16::MAX] {
            assert_eq!(
                noise_floor_counts(staged),
                MAX_LIMIT_COUNTS,
                "staged {staged}"
            );
        }
        // The registered default is inside the band, so a board with no record needs no clamping.
        assert_eq!(noise_floor_counts(FLOOR), FLOOR as i16);
        // At the band's top the window is one value wide: every request converts to the ceiling.
        for ma in [0u32, 1, 10_000, u32::MAX] {
            assert_eq!(limit_counts(ma, CAL, u16::MAX), MAX_LIMIT_COUNTS, "{ma} mA");
        }
        // The floor is a MINIMUM on the enforced limit, which is the whole hazard: at the measured
        // scale the default floor is 4.6 A, so a staged 4 A is enforced at 4.6 A with nothing said,
        // and the board that read its own quieter chain (1,444 counts, the lowest of the five on
        // 2026-10-09) enforces the 4 A it was asked for.
        assert_eq!(limit_counts(4_000, CAL, FLOOR), 2_100, "4 A becomes 4.6 A");
        assert_eq!(limit_counts(4_000, CAL, 1_444), 1_820, "4 A stays 4 A");
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
        let lim = limit_counts(10_000, CAL, FLOOR);
        let mut c = CurrentLimit::new(lim);
        assert!(!c.step(lim, 0).chop, "AT the limit drives");
        assert!(c.step(lim + 1, 0).chop, "one count over floats");
        assert!(
            !c.step(lim, 0).chop,
            "the next period re-evaluates from its own sample"
        );
        assert!(!c.step(0, 0).chop);
    }

    /// The hard trip's magnitude condition: at `hard_trip_counts` on the first period, and not one
    /// count under it.
    #[test]
    fn the_trip_fires_at_the_hard_magnitude() {
        let lim = limit_counts(2_000, CAL, FLOOR);
        let hard = hard_trip_counts(lim);
        let mut c = CurrentLimit::new(lim);
        let v = c.step(hard - 1, 0);
        assert!(
            v.chop && !v.trip,
            "over the limit, under the hard trip: chop only"
        );
        assert_eq!(c.trips(), 0);
        let mut c = CurrentLimit::new(lim);
        let v = c.step(hard, 0);
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
        let lim = limit_counts(2_000, CAL, FLOOR);
        let over = lim + 1;
        let mut c = CurrentLimit::new(lim);
        for _ in 0..OVER_CURRENT_TRIP_PERIODS - 1 {
            assert!(!c.step(over, 0).trip);
        }
        assert_eq!(c.trips(), 0, "63 consecutive do not trip");
        assert!(c.step(over, 0).trip, "the 64th does");
        assert_eq!(c.trips(), 1);

        // One clean period resets the run.
        let mut c = CurrentLimit::new(lim);
        for _ in 0..OVER_CURRENT_TRIP_PERIODS - 1 {
            c.step(over, 0);
        }
        assert!(!c.step(lim, 0).trip, "a period at the limit is clean");
        for _ in 0..OVER_CURRENT_TRIP_PERIODS - 1 {
            assert!(!c.step(over, 0).trip, "the run restarted");
        }
        assert!(c.step(over, 0).trip);
    }

    /// A trip counts once per over-limit episode: the conditions stay true for every further
    /// over-limit period, and only a clean period re-arms the trip.
    #[test]
    fn a_trip_counts_once_per_over_limit_episode() {
        let lim = limit_counts(2_000, CAL, FLOOR);
        let hard = hard_trip_counts(lim);
        let mut c = CurrentLimit::new(lim);
        assert!(c.step(hard, 0).trip);
        for _ in 0..200 {
            let v = c.step(hard, 0);
            assert!(v.chop, "every over-limit period still floats");
            assert!(!v.trip, "the same episode does not trip again");
        }
        assert_eq!(c.trips(), 1);
        c.step(0, 0);
        assert!(c.step(hard, 0).trip, "a clean period re-arms it");
        assert_eq!(c.trips(), 2);
    }

    /// The observation window: the peak is the window's maximum magnitude, `chopped` its count of
    /// floated periods, and closing the window publishes both and restarts them.
    #[test]
    fn the_window_peak_and_its_restart() {
        let lim = limit_counts(1_000, CAL, FLOOR);
        let mut c = CurrentLimit::new(lim);
        for mag in [10, 500, lim + 5, 30, lim + 1] {
            c.step(mag, 0);
        }
        let (w, _, _) = c.take_window();
        assert_eq!(w & 0xFFFF, (lim + 5) as u32, "peak");
        assert_eq!((w >> 16) & 0xFF, 2, "two periods floated");
        assert_eq!(w >> 24, 0, "no trip");
        // Restarted: the next window sees only its own periods.
        c.step(40, 0);
        let (w, _, _) = c.take_window();
        assert_eq!(w & 0xFFFF, 40);
        assert_eq!((w >> 16) & 0xFF, 0);
        // A whole window floated: `chopped` reads 64, and the trip it caused shows in the top byte.
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(lim + 1, 0);
        }
        let (w, _, _) = c.take_window();
        assert_eq!((w >> 16) & 0xFF, 64);
        assert_eq!(w >> 24, 1, "the run tripped once");
    }

    /// The window's MEAN, which is the word a calibration cross-check compares
    /// (`specs/rider-ui.md` 3.6): the sum of the window's magnitudes over the window's length,
    /// and it comes out of the SAME close as the peak.
    #[test]
    fn the_window_mean_is_the_average_of_its_periods() {
        let lim = limit_counts(1_000, CAL, FLOOR);
        let mut c = CurrentLimit::new(lim);
        // A full window of one constant magnitude: the mean IS that magnitude, which is the
        // property a wrong shift breaks first (a shift of 5 would read 2x, of 7 half).
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(300, 0);
        }
        let (w, mean, _) = c.take_window();
        assert_eq!(mean, 300, "a constant window's mean is that constant");
        assert_eq!(w & 0xFFFF, 300, "and it is the peak's own window");

        // A window with one spike: the PEAK reads the spike, the mean reads the average, and the
        // difference between the two is the whole reason the mean is on the wire.
        c.step(6_400, 0);
        for _ in 1..PERIODS_PER_TICK_NOMINAL {
            c.step(0, 0);
        }
        let (w, mean, _) = c.take_window();
        assert_eq!(w & 0xFFFF, 6_400, "the peak is the spike");
        assert_eq!(mean, 100, "the mean is 6400 / 64");

        // Restarted with the peak: a closed window leaves nothing behind.
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(8, 0);
        }
        let (_, mean, _) = c.take_window();
        assert_eq!(mean, 8, "the sum restarted with the window");
    }

    /// The mean cannot overflow its accumulator or its narrowing, at the one input that could do
    /// either: a whole window at the magnitude ceiling.
    #[test]
    fn a_window_at_the_magnitude_ceiling_does_not_overflow_the_mean() {
        let mut c = CurrentLimit::new(MAX_LIMIT_COUNTS);
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(i16::MAX, 0);
        }
        let (_, mean, _) = c.take_window();
        assert_eq!(mean, i16::MAX, "64 x 32,767 fits a u32 and the mean an i16");
    }

    /// Both current words come from ONE call, so a consumer comparing them is always comparing
    /// one window: there is no way to take the peak of one window and the mean of another out of
    /// this type, and a window closed twice yields nothing the second time.
    #[test]
    fn one_call_closes_the_whole_window() {
        let mut c = CurrentLimit::new(limit_counts(1_000, CAL, FLOOR));
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(640, 0);
        }
        let (w, mean, _) = c.take_window();
        assert_eq!(w & 0xFFFF, 640);
        assert_eq!(mean, 640);
        let (w, mean, _) = c.take_window();
        assert_eq!(w & 0xFFFF, 0, "the peak restarted");
        assert_eq!(mean, 0, "and so did the sum");
    }

    /// **The step folds the magnitude of the PAIR** (`specs/foc-torque.md` section 7): handing
    /// `step` the two sensed currents chops and trips exactly as handing it their
    /// [`phase_magnitude`] did, including on the unsensed phase's Kirchhoff remainder, and the
    /// window's peak is that magnitude.
    #[test]
    fn the_step_judges_the_magnitude_of_the_pair() {
        let lim = limit_counts(10_000, CAL, FLOOR);
        let half = lim / 2 + 1;
        for (ia, ib) in [
            (0i16, 0i16),
            (lim, 0),
            (lim + 1, 0),
            (0, -(lim + 1)),
            (half, half), // ic = -(lim + 2): only the unsensed phase is over
            (-half, -half),
            (half, -half),
            (hard_trip_counts(lim), -3),
            (i16::MIN, i16::MIN),
        ] {
            let mut by_pair = CurrentLimit::new(lim);
            let mut by_mag = CurrentLimit::new(lim);
            let mag = phase_magnitude(ia, ib);
            assert_eq!(by_pair.step(ia, ib), by_mag.step(mag, 0), "({ia}, {ib})");
            assert_eq!(by_pair.take_window().0 & 0xFFFF, mag as u16 as u32);
        }
        let mut c = CurrentLimit::new(lim);
        assert!(c.step(half, half).chop, "the third phase chops");
    }

    /// **`take_window` returns the two phase means** beside the peak and the magnitude mean, from
    /// the same close (`CTRL_OBS` word 47): each the signed sum of its phase over the window
    /// shifted by its length, rounding toward minus infinity, and restarting with the window.
    #[test]
    fn the_window_carries_the_two_phase_means() {
        let lim = limit_counts(10_000, CAL, FLOOR);
        let mut c = CurrentLimit::new(lim);
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(300, -120);
        }
        let (w, mean, phase) = c.take_window();
        assert_eq!(phase, (300, -120), "a constant window reads its constant");
        assert_eq!(
            (w & 0xFFFF, mean),
            (300, 300),
            "the same window's peak and mean"
        );

        // A moving window: the means are the sums over 64, and a negative remainder rounds down.
        let (mut sa, mut sb) = (0i32, 0i32);
        for k in 0..PERIODS_PER_TICK_NOMINAL as i16 {
            let (ia, ib) = (k * 9 - 250, 77 - k * 5);
            sa += ia as i32;
            sb += ib as i32;
            c.step(ia, ib);
        }
        let (_, _, phase) = c.take_window();
        assert_eq!(phase, ((sa >> 6) as i16, (sb >> 6) as i16));
        c.step(-1, 1);
        assert_eq!(c.take_window().2, (-1, 0), "toward minus infinity");
        assert_eq!(
            c.take_window().2,
            (0, 0),
            "the sums restarted with the window"
        );

        // The ceiling on both phases at once, opposite signs (so the third reads zero).
        for _ in 0..PERIODS_PER_TICK_NOMINAL {
            c.step(-32_767, 32_767);
        }
        assert_eq!(c.take_window().2, (-32_767, 32_767));
    }

    /// The FOC observation words' packing: two `i16` halves, low then high, each recovered with
    /// its sign by the reader's decode (`tools/swdobs.py`, `s16` of each half), and neither
    /// bleeding into the other.
    #[test]
    fn i16_pair_packing_round_trips() {
        for (lo, hi) in [
            (0i16, 0i16),
            (300, -120),
            (-1, 0),
            (0, -1),
            (i16::MIN, i16::MAX),
        ] {
            let w = pack_i16_pair(lo, hi);
            assert_eq!((w as u16 as i16, (w >> 16) as u16 as i16), (lo, hi));
        }
        assert_eq!(pack_i16_pair(-1, 0), 0x0000_FFFF);
        assert_eq!(pack_i16_pair(0, -1), 0xFFFF_0000);
        assert_eq!(pack_i16_pair(2_450, -7), 0xFFF9_0992);
    }

    /// The mean/duty word's layout and its unpackers, which are each other's inverse: the
    /// consumer reads the window through the inverse of the ISR's pack rather than re-deriving
    /// the bit positions.
    #[test]
    fn mean_duty_packing_round_trips() {
        for (mean, duty) in [
            (0i16, 0u16),
            (300, 1_125),
            (-3, 2_250),
            (i16::MAX, u16::MAX),
        ] {
            let w = pack_motor_mean_duty(mean, duty);
            assert_eq!(unpack_mean(w), mean, "mean {mean}");
            assert_eq!(unpack_duty_on(w), duty, "duty {duty}");
        }
        // The halves do not bleed into each other: a full-scale duty leaves the mean alone.
        assert_eq!(unpack_mean(pack_motor_mean_duty(7, u16::MAX)), 7);
        assert_eq!(unpack_duty_on(pack_motor_mean_duty(-1, 0)), 0);
        // And the peak's unpacker is the inverse of its own pack, sign and all.
        for peak in [0i16, 1, 32_767, -1, i16::MIN] {
            assert_eq!(unpack_peak(pack_motor_current(peak, 64, 3)), peak);
        }
    }

    /// The ON-DUTY of a period: the conducting leg's duty, and 0 for a period that coasted.
    #[test]
    fn the_on_duty_is_the_conducting_legs_duty() {
        // Six-step: one leg at the commanded duty, the sink leg driven at 0, the third floating.
        assert_eq!(on_duty([1_700, 0, 0], [true, true, false]), 1_700);
        assert_eq!(on_duty([0, 0, 1_700], [false, true, true]), 1_700);
        // A coasting period: every channel floated, so nothing conducted, whatever the duty
        // registers still hold. This is the soft current limit's own period, and the case a
        // derived link current must not read as conduction.
        assert_eq!(on_duty([1_700, 1_700, 1_700], [false; 3]), 0);
        // A gated channel's duty is not the on-duty even when it is the largest.
        assert_eq!(on_duty([2_000, 500, 0], [false, true, true]), 500);
        // Sine / FOC drive all three: the largest applied compare is the on-duty.
        assert_eq!(on_duty([1_125, 1_600, 700], [true; 3]), 1_600);
        assert_eq!(on_duty([0, 0, 0], [true; 3]), 0, "all legs at zero");
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
        let mut c = CurrentLimit::new(limit_counts(1_000, CAL, FLOOR));
        assert!(c.step(i16::MAX, 0).trip);
        let producers = FAULT_HALL | FAULT_INIT_CAL;
        for bit in 0..32 {
            let w = 1u32 << bit;
            assert_eq!(
                motor_fault_level(true, w, false, false) != 0,
                w & producers != 0,
                "fault bit {bit}"
            );
        }
        assert_eq!(motor_fault_level(true, 0, false, false), 0);
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
