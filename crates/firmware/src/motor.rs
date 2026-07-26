//! The plan-gated motor bring-up and the 16 kHz period ISR (`specs/motor-integration.md`, slice 3:
//! "Firmware bring-up, disarmed").
//!
//! This is the motor half of the one universal image. It is **disarmed by construction**: the
//! arming gate (the sole MOE writer, `runtime_hal::timer::arming`) is never named anywhere in this
//! crate, so no code path here can energize a bridge. The timer is configured, the counter runs and
//! the compare units toggle, the injected ADC converts and its end-of-conversion ISR steps the
//! commutator, but with MOE clear nothing reaches the gate drivers. Arming is slice 5.
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

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

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
/// period ISR (the flags' `configured` / `current_sense` bits are set once at bring-up).
pub static OBS_STATE: AtomicU32 = AtomicU32::new(0);
/// The last applied duties, packed `d0 | d1 << 16`. Written by the period ISR.
pub static OBS_DUTY01: AtomicU32 = AtomicU32::new(0);
/// The last applied duty 2 plus the electrical angle, packed `d2 | angle << 16`. Written by the
/// period ISR.
pub static OBS_DUTY2_ANGLE: AtomicU32 = AtomicU32::new(0);

/// Latched invalid-hall fault (the commutator's dwell fault: > 64 consecutive invalid codes).
pub const FAULT_HALL: u32 = 1 << 0;
/// A duty the HAL refused as out of range (the compare would never match): the outputs were left
/// as they were, per the write-order rule.
pub const FAULT_DUTY_RANGE: u32 = 1 << 1;
/// The demand-freshness guard fired (the 250 Hz task stopped writing; all phases forced to float).
pub const FAULT_DEMAND_STALE: u32 = 1 << 2;

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
pub const BRING_UP_STEPS: [BringUpStep; 12] = [
    BringUpStep::EnableTimerClock,
    BringUpStep::ConfigureTimer,
    BringUpStep::ConfigureTriggerChannel,
    BringUpStep::RouteGatePins,
    BringUpStep::EnableAdcClock,
    BringUpStep::ConfigurePhasePins,
    BringUpStep::ConfigureInjectedGroup,
    BringUpStep::StartCounter,
    BringUpStep::ConfirmTriggerStart,
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

/// The commutation method this bring-up will actually run, from the `MOTOR_METHOD` byte.
///
/// **Six-step only in this slice.** Sine is slice 6 and FOC is slice 7, and neither has a bench
/// gate before then: `Sine`'s open-loop modulation is signed off against a scope on a spinning
/// wheel, and `Foc`'s per-mode records (`commutation::foc::FocState`) are uninhabited until its own
/// slice, with the offset calibration that would populate them in slice 4. So any other byte falls
/// back to six-step here (recorded: the running method is published in the observation block, so a
/// board configured for sine reads back as six-step rather than silently claiming sine). Slice 4
/// adds the rest of the policy -- a refused calibration, and `Foc` against a motor with no current
/// sense -- with the init-failure fault producer that reports it.
///
/// Keeping the unbuilt arms out is also what keeps them out of the IMAGE: the dispatch is a match
/// on the records, so a method never selected is a method LTO drops.
#[inline]
pub fn method_for(_method_byte: u8) -> commutation::CommutationMethod {
    commutation::CommutationMethod::SixStep
}

/// Record a motor that was NOT brought up into the observation word, so a bench read distinguishes
/// "no motor configured" from "configured but this slice cannot drive it" from "a step failed, and
/// which one" -- all of which look alike as a silent absence otherwise. Every arm leaves
/// [`OBS_CONFIGURED`] clear.
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
pub use hw::bring_up;

#[cfg(target_os = "none")]
mod hw {
    use super::*;
    use board::MotorPlan;
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

    /// What a bring-up did, for the boot-time record.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct MotorRuntimeSummary {
        /// The method actually running.
        pub method: CommutationMethod,
        /// The injected channels programmed, in rank order.
        pub channels: [u8; 2],
    }

    /// Bring one motor up from its validated plan, disarmed (`BRING_UP_STEPS`, in order).
    ///
    /// Returns the summary on success, or the step that stopped it. Nothing here writes MOE: the
    /// arming gate is not built, not held, and not reachable from this crate.
    pub fn bring_up(
        chip: &Chip,
        plan: &MotorPlan,
        method_byte: u8,
    ) -> Result<MotorRuntimeSummary, MotorSkip> {
        // Step 1 of the spec's list, before the ordered steps: refuse an absent layout. A motor
        // with no gate set or no hall set is absent, which is a valid board state, not a fault.
        let (gates, halls) = match (plan.gates, plan.halls) {
            (Some(g), Some(h)) => (g, h),
            _ => return Err(MotorSkip::Absent),
        };
        let phase = plan.phase_current.ok_or(MotorSkip::NoCurrentSense)?;

        // The one configured timer object, built by `ConfigureTimer` and reused by every later
        // step that touches the timer (the trigger channel, the counter start, the per-cycle
        // handle): configuring it once is the bring-up, not a step that repeats.
        let mut timer: Option<PwmTimer> = None;
        let mut injected = None;
        let mut method = CommutationMethod::SixStep;

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
                BringUpStep::StartCounter => timer.ok_or(failed(step))?.enable_counter(),
                BringUpStep::ConfirmTriggerStart => {
                    let inj = injected.ok_or(failed(step))?;
                    if !confirm_trigger_start(&inj) {
                        return Err(failed(step));
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
                    method = method_for(method_byte);
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
                    let base_flags = OBS_CONFIGURED | OBS_CURRENT_SENSE;
                    // SAFETY: the one write, on the boot thread, BEFORE the period vector is
                    // unmasked (the next-but-one step), so no ISR can observe it half-built.
                    unsafe {
                        *addr_of_mut!(MOTOR) = Some(MotorRuntime {
                            pwm,
                            injected: inj,
                            halls: group,
                            commutator: Commutator::new(records),
                            base_flags,
                            method: method.to_u8(),
                            periods: 0,
                            since_demand: 0,
                            last_seq: DEMAND_SEQ.load(Ordering::Relaxed),
                            faults: 0,
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
            method,
            channels: phase.channels,
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
        for _ in 0..TRIGGER_START_ATTEMPTS {
            injected.clear_eoic();
            injected.reassert_trigger_enable();
            let mut spins = TRIGGER_START_SPINS;
            while spins > 0 {
                if injected.eoic() {
                    return true;
                }
                spins -= 1;
            }
        }
        false
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
            // PSC = 0: the timer runs from the timer clock undivided, which is what makes
            // 72 MHz / (2 x 2250) = 16 kHz. Named, not assumed.
            prescaler: 0,
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
                        | BringUpStep::SelectMethodAndInstall
                        | BringUpStep::RegisterPeriodHandler
                        | BringUpStep::EnablePeriodVector
                ),
                "an unexpected bring-up step exists: {s:?}"
            );
        }
    }

    /// **Disarmed by construction** (`specs/motor-integration.md` slice 3: "the arm call absent
    /// from the tree entirely, not merely unreachable"). The arming gate type and its methods are
    /// not named anywhere in this crate's source, so there is no code path, reachable or not, that
    /// can write MOE. Comment lines are stripped first, so the prose above may discuss arming
    /// while the CODE may not touch it.
    #[test]
    fn the_arm_call_is_absent_from_the_whole_crate() {
        let sources = [
            ("main.rs", include_str!("main.rs")),
            ("motor.rs", include_str!("motor.rs")),
        ];
        for (name, src) in sources {
            let code: std::string::String = src
                .lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .collect::<std::vec::Vec<_>>()
                .join("\n");
            // The tokens are assembled from pieces so this test's own source does not contain
            // them (it scans itself, being part of the crate).
            for token in [
                concat!("Arm", "Gate"),
                concat!("arm_", "gate"),
                concat!(".arm", "()"),
                concat!("CC", "HP"),
                concat!("set_", "moe"),
            ] {
                assert!(
                    !code.contains(token),
                    "{name} names `{token}`: the arming surface must be absent from this crate \
                     until slice 5"
                );
            }
        }
    }

    /// Method selection in this slice: six-step for every byte. Sine (slice 6) and FOC (slice 7)
    /// are not built, so a board configured for either runs six-step and READS BACK as six-step in
    /// the observation block, rather than silently claiming a method it is not running.
    #[test]
    fn method_selection_is_six_step_until_its_own_slice() {
        use commutation::CommutationMethod as M;
        assert_eq!(method_for(0), M::SixStep);
        assert_eq!(method_for(1), M::SixStep, "sine is slice 6");
        assert_eq!(method_for(2), M::SixStep, "FOC is slice 7");
        assert_eq!(method_for(99), M::SixStep, "an unknown byte too");
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
