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
/// calibration step of the bring-up, before the period vector is unmasked; zero on a boot that ran
/// no calibration (six-step requested) or whose conversions never completed.
///
/// Each half is in the ACCUMULATED offset unit ([`cal_offsets`]): the sum of 16 conversions each
/// shifted right by 3, which is 2x the zero-current register value. A bench read compares it against
/// the acceptance window and the stock healthy band in those units, NOT against a raw sample.
///
/// The measured pair is published whether or not [`commutation::foc::PhaseOffsets`] accepted it,
/// so a bench read sees WHERE an out-of-window board actually sits rather than only that it was
/// refused. Acceptance is the [`OBS_CAL_ACCEPTED`] flag; refusal is [`FAULT_INIT_CAL`]; neither
/// set with a zero word means no calibration was requested.
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
/// The init-failure fault: the FOC offset calibration was REFUSED, so the bring-up fell back to
/// six-step (`specs/motor-integration.md`, bring-up step 9). Set ONCE by the bring-up, before the
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
/// the measured pair (both offsets inside its window). Clear on a boot that ran no calibration and
/// on a refused one; [`FAULT_INIT_CAL`] separates those two.
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

/// What the offset calibration did this boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalOutcome {
    /// `MOTOR_METHOD` did not request FOC, so no calibration ran. Six-step and sine consume no
    /// current offsets, so measuring them would model a fact nothing reads.
    NotRequested,
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
/// therefore the outcome's whole observable effect this slice, and it is the half that matters:
/// it is what stops a board whose current sense cannot be trusted from being allowed to arm.
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
        CalOutcome::NotRequested | CalOutcome::Accepted => 0,
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
    /// Put the two phase-current pins in analog mode (the digital input buffer would otherwise
    /// clamp the sample).
    ConfigurePhasePins,
    /// Program + calibrate the injected group: the two phase-current ranks, 7.5-cycle sampling,
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
    /// conversions per channel) and gate them through [`commutation::foc::PhaseOffsets`], when
    /// `MOTOR_METHOD` requests FOC. AFTER `ConfirmTriggerStart`, load-bearing: the measurement
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
/// re-read at every bring-up. Its one live consumer is the decision to CALIBRATE (only FOC
/// consumes phase-current offsets); [`running_method`] decides what actually runs.
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
    use board::MotorPlan;
    use commutation::foc::PhaseOffsets;
    use commutation::{CommutationMethod, Commutator, MethodState};
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
    }

    /// The ISR's state. Written once by the bring-up before the period vector is unmasked, and
    /// read/written only by the period ISR afterwards.
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
    pub fn bring_up(
        chip: &Chip,
        plan: &MotorPlan,
        method_byte: u8,
        period_hz: u32,
    ) -> Result<MotorRuntimeSummary, MotorSkip> {
        // Step 1 of the spec's list, before the ordered steps: refuse an absent layout. A motor
        // with no gate set or no hall set is absent, which is a valid board state, not a fault.
        let (gates, halls) = match (plan.gates, plan.halls) {
            (Some(g), Some(h)) => (g, h),
            _ => return Err(MotorSkip::Absent),
        };
        let phase = plan.phase_current.ok_or(MotorSkip::NoCurrentSense)?;
        // The policy input, read once: it decides only whether the offset calibration runs. What
        // actually runs is `running_method`'s clamp, applied at `SelectMethodAndInstall`.
        let requested = requested_method(method_byte);

        // The one configured timer object, built by `ConfigureTimer` and reused by every later
        // step that touches the timer (the trigger channel, the counter start, the per-cycle
        // handle): configuring it once is the bring-up, not a step that repeats.
        let mut timer: Option<PwmTimer> = None;
        let mut injected = None;
        let mut cal = CalOutcome::NotRequested;

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
                    for pin in phase.pins.iter() {
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
                        .configure(chip, &injected_config(&phase.channels))
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
                    // Only FOC consumes phase-current offsets, so only FOC pays for measuring
                    // them (`specs/motor-integration.md`, bring-up step 9). A refusal does NOT
                    // fail the step: the motor still comes up, on six-step, carrying the
                    // init-failure fault -- which is the spec's fallback, not a dead bring-up.
                    if requested == CommutationMethod::Foc {
                        let inj = injected.ok_or(failed(step))?;
                        cal = match measure_offsets(&inj) {
                            Some((a, b)) => {
                                OBS_CAL.store((a as u32) | ((b as u32) << 16), Ordering::Relaxed);
                                // The acceptance window's single owner is the commutation crate's
                                // gated newtype, so the check is ITS constructor, never a copy of
                                // its constants here.
                                if PhaseOffsets::try_new(a, b).is_some() {
                                    CalOutcome::Accepted
                                } else {
                                    CalOutcome::Refused
                                }
                            }
                            // No conversion inside the poll budget: no trustworthy zero-current
                            // reference, and OBS_CAL stays zero to say the measurement itself
                            // never happened.
                            None => CalOutcome::Refused,
                        };
                    }
                }
                BringUpStep::SelectMethodAndInstall => {
                    let (pwm, inj) = match (timer, injected) {
                        (Some(t), Some(i)) => (t.handle(), i),
                        _ => return Err(failed(step)),
                    };
                    let group = chip
                        .input_group([halls.a.packed(), halls.b.packed(), halls.c.packed()])
                        .map_err(|_| failed(step))?;
                    let method = running_method(requested);
                    let records = MethodState::SixStep(commutation::sixstep::SixStepState::new(
                        commutation::sixstep::SixStep::new(
                            if plan.direction {
                                commutation::sixstep::Direction::Reverse
                            } else {
                                commutation::sixstep::Direction::Forward
                            },
                            plan.align_offset,
                        ),
                    ));
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
                            base_flags,
                            method: method.to_u8(),
                            periods: 0,
                            since_demand: 0,
                            last_seq: DEMAND_SEQ.load(Ordering::Relaxed),
                            faults: init_faults,
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

    /// The injected-group configuration: TWO slots, deliberately, not the stock four (the aux and
    /// battery consumers do not exist here, so populating them would model samples nothing reads).
    fn injected_config(channels: &[u8; 2]) -> InjectedAdcConfig {
        let mut chans: Vec<InjectedChannel, 4> = Vec::new();
        for &channel in channels.iter() {
            // Cannot overflow: two pushes into a capacity-4 vector.
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
        let applied_enables = if stale {
            // The explicit coast posture: all phases float regardless of method. It cannot be
            // inferred from a zero demand (only six-step coasts at zero demand; sine holds
            // mid-rail and FOC holds ~1125 on every phase), so the guard applies it directly, and
            // it applies it BEFORE any duty write so silencing never depends on one.
            m.pwm.set_channel_outputs([false; 3]);
            m.faults |= FAULT_DEMAND_STALE;
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

    /// The REQUESTED method is decoded faithfully even though the running method is clamped: it is
    /// what decides whether the offset calibration runs, so a board asking for FOC must be
    /// distinguishable from one asking for six-step.
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
        assert_eq!(init_fault_bits(CalOutcome::NotRequested), 0);
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
