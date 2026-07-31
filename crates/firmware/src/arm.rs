//! Arming: the one place in the image that can energize a bridge (`specs/motor-integration.md`,
//! slice 5, "MOE enactment").
//!
//! `crates/firmware/src/motor.rs` is disarmed by construction and stays that way: it configures the
//! timer, runs the counter and steps the commutator, and it never names the arming gate. This file
//! is the complement. It holds the [`runtime_hal::ArmGate`] for the configured motor and it holds
//! nothing else, so **the whole energize decision is one file, one static, one `arm()` call**, which
//! is bring-up step 11's "keep the energize point one visible boundary in one place" made
//! structural. A host test in this module enforces it over the crate's source.
//!
//! # What decides
//!
//! The mode machine, and only the mode machine (`specs/sensing-and-safety.md`: the `MoeGate`
//! invariant is set on the INIT pass, cleared on every SHUTDOWN pass and fault path, never enabled
//! across an OFF dwell). This layer never decides to arm; it enacts [`ControlOutput::moe`], and it
//! adds two refusals of its own, in the arm direction only:
//!
//! - a motor that was never brought up cannot be armed (there is no runtime to drive it), and
//! - the motor-side fault LEVEL vetoes arming.
//!
//! **The level, never the raw `FAULT` word.** [`decide`] takes a `bool` and cannot express reading
//! the word, deliberately: `motor::FAULT` carries bits that are not fault producers
//! (`FAULT_DEMAND_STALE` is self-mitigating, the ISR has already floated every phase by the time it
//! is set; `FAULT_DUTY_RANGE` records a refused write that changed no output), and both are
//! boot-sticky. A gate consuming the word would let a single stale-demand period at boot block
//! arming for the rest of the boot, on a board whose bridge is perfectly healthy. The producers that
//! DO belong are folded by [`motor::motor_fault_level`], which is the level's single owner.
//!
//! # The two sequences
//!
//! [`ARM_STEPS`] and [`SHUTDOWN_STEPS`] are ordered lists, the [`motor::BRING_UP_STEPS`] pattern, so
//! the orderings the spec marks load-bearing are testable as data without hardware. The shutdown
//! list is the inverse of the arm list with MOE FIRST (`specs/motor-integration.md`: "Clearing MOE
//! before anything else preserves the layered-disable ordering ... torque zeroing and MOE clear are
//! two independent silencing paths, never collapsed").

// The hardware half is target-only; on the host only the pure surfaces below compile, and their
// consumers (the target service loop) do not, so the host build reads them as dead code.
#![cfg_attr(not(target_os = "none"), allow(dead_code))]

// -------------------------------------------------------------------------------------------
// The ordered sequences (pure)
// -------------------------------------------------------------------------------------------

/// One step of the arm sequence. Exactly one step can set MOE, and it is LAST.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmStep {
    /// Start the timer counter. Idempotent on the first arm of a boot (the bring-up already
    /// started it); load-bearing on a re-arm, where [`ShutdownStep::StopCounter`] stopped it.
    StartCounter,
    /// Confirm the period ISR is actually being served, by watching [`motor::PERIODS`] advance.
    /// **Load-bearing, and the reason arming is not simply a register write**: MOE is the only
    /// thing standing between a stopped commutator and a bridge holding whatever the compare units
    /// last held, so the bridge is energized only after the thing that will step it is proven
    /// alive. It watches the ISR's own liveness word rather than the injected end-of-conversion
    /// flag the BRING-UP confirm polls, because here the vector is already unmasked: the live ISR
    /// clears that flag itself, so a poll of it would race the ISR and could refuse a healthy arm.
    ConfirmPeriodsLive,
    /// Zero the demand word, so the compare values the bridge energizes into are the ones the ISR
    /// derived from a zero demand. The mode machine already guarantees this (torque is produced
    /// only in RUN, and MOE rises at INIT, two passes earlier), but the energize point should not
    /// depend on a property of a different layer.
    ZeroDemand,
    /// Set MOE. **The one energize act in the image**, and the last step, so every precondition
    /// above it has already passed.
    SetMoe,
}

/// The arm sequence, in order.
pub const ARM_STEPS: [ArmStep; 4] = [
    ArmStep::StartCounter,
    ArmStep::ConfirmPeriodsLive,
    ArmStep::ZeroDemand,
    ArmStep::SetMoe,
];

/// One step of the shutdown sequence (`ShutdownAction`), the inverse of the arm sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShutdownStep {
    /// Clear MOE. **FIRST**, unconditionally: it is the act that actually silences the bridge, and
    /// nothing below it may be able to fail before it runs.
    Disarm,
    /// Zero the demand word: the second, independent silencing path, never collapsed into the
    /// first.
    ZeroDemand,
    /// Float every phase (`set_channel_outputs([false; 3])`), the explicit coast posture. Not
    /// inferable from the zero demand above: only six-step coasts to all-float at zero demand.
    FloatChannels,
    /// Stop the counter. LAST: with MOE already clear it changes nothing electrically, and it is
    /// what leaves the timer in the state a later arm's [`ArmStep::StartCounter`] restores.
    StopCounter,
}

/// The shutdown sequence, in order.
pub const SHUTDOWN_STEPS: [ShutdownStep; 4] = [
    ShutdownStep::Disarm,
    ShutdownStep::ZeroDemand,
    ShutdownStep::FloatChannels,
    ShutdownStep::StopCounter,
];

/// What this tick's inputs call for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmDecision {
    /// Nothing to enact: the hardware already matches the mode machine.
    Idle,
    /// Run [`ARM_STEPS`].
    Arm,
    /// Run [`SHUTDOWN_STEPS`].
    Shutdown,
}

/// The arm gate: the whole decision, as a total function of four levels.
///
/// - `moe_allowed`: the mode machine's per-motor allowance, the sole ARMING authority.
/// - `armed`: whether this layer has already set MOE (its own record, so the decision is an edge).
/// - `brought_up`: the motor has a runtime (`motor::OBS_CONFIGURED`). A board with no motor can
///   never be armed, so a plan-less or failed bring-up cannot reach the bridge.
/// - `fault_level`: the motor-side fault LEVEL ([`motor::motor_fault_level`]), never the raw
///   `FAULT` word (see the module docs).
///
/// The asymmetry is deliberate and is the safety property: `brought_up` and `fault_level` can only
/// REFUSE an arm, never force or hold one, while `!moe_allowed` and `fault_level` both DEMAND a
/// shutdown. A fault that appears while armed therefore shuts the bridge down here even in the
/// impossible case where the mode machine still allows MOE, and a fault that appears while
/// disarmed can never be the reason the bridge stays energized.
#[inline]
pub fn decide(moe_allowed: bool, armed: bool, brought_up: bool, fault_level: bool) -> ArmDecision {
    if armed {
        if moe_allowed && !fault_level {
            ArmDecision::Idle
        } else {
            ArmDecision::Shutdown
        }
    } else if moe_allowed && brought_up && !fault_level {
        ArmDecision::Arm
    } else {
        ArmDecision::Idle
    }
}

/// The OFF-inhibit producer (`specs/motor-integration.md`, "MOE enactment": `off_inhibit` gets its
/// real producer at last, and it reads `SPEED[i]`).
///
/// True while the wheel is turning. `speed` is the period ISR's raw signed hall-edge count over its
/// 320-period (20 ms) window, so ANY net edge in the window is motion and the threshold is 1: the
/// predicate holds the machine in OFF, i.e. it refuses to ENGAGE a vehicle whose wheel is already
/// moving, and being conservative there costs a rider one more button press while being permissive
/// costs an engage under a rolling wheel. A stationary wheel reads exactly 0 (hall codes do not
/// change, so the commutator counts no edges), and a bouncing hall line at rest contributes
/// alternating signs that net toward 0 rather than accumulating.
///
/// It cannot block a shutdown: the mode machine consults `off_inhibit` in OFF only.
#[inline]
pub fn off_inhibit_from_speed(speed: i32) -> bool {
    speed != 0
}

/// The [`ArmStep::ConfirmPeriodsLive`] spin budget, in poll iterations. One 16 kHz period is 62.5 us
/// (4500 cycles at 72 MHz), so a healthy motor satisfies the confirm within ~1100 iterations of a
/// ~4-cycle atomic-load loop; the budget is ~25x that, and it is paid in full only by a board whose
/// period ISR has stopped, once, on the tick that tried to arm.
pub const ARM_CONFIRM_SPINS: u32 = 30_000;

// -------------------------------------------------------------------------------------------
// The hardware half
// -------------------------------------------------------------------------------------------

#[cfg(target_os = "none")]
pub mod hw {
    use super::{
        decide, ArmDecision, ArmStep, ShutdownStep, ARM_CONFIRM_SPINS, ARM_STEPS, SHUTDOWN_STEPS,
    };
    use crate::motor;
    use core::ptr::addr_of_mut;
    use core::sync::atomic::{AtomicBool, Ordering};
    use runtime_hal::ArmGate;

    /// The configured motor's arming gate, installed once at boot from the bring-up's timer and
    /// read only by the 250 Hz control task afterwards. `None` on a board whose motor was not
    /// brought up, which is what makes such a board unarmable rather than merely unarmed.
    ///
    /// It is the ONLY handle in this crate that can write `CCHP`: the HAL builds it as a
    /// deliberately separate object from the per-cycle `PwmHandle` the period ISR drives, and this
    /// module is the only holder of it.
    static mut GATE: Option<ArmGate> = None;

    /// Whether this layer has set MOE. Its own record, so [`decide`] sees an EDGE rather than
    /// re-running a sequence every tick. Written only on the 250 Hz thread.
    static ARMED: AtomicBool = AtomicBool::new(false);

    /// An arm attempt that could not confirm the period ISR was live, sticky for the boot. It feeds
    /// the motor-side fault level, so a board that failed to arm SHUTS DOWN loudly instead of
    /// sitting in RUN with a silent, unarmed bridge.
    static ARM_REFUSED: AtomicBool = AtomicBool::new(false);

    /// Install the arming gate for a brought-up motor. Called once, on the boot thread, from the
    /// bring-up's summary. Installing it does not arm anything: MOE is untouched here, and the
    /// mode machine is in OFF for the whole of boot.
    pub fn install(timer: &runtime_hal::PwmTimer) {
        // SAFETY: the one write, on the boot thread, before the scheduler exists and therefore
        // before any control task can read it.
        unsafe { *addr_of_mut!(GATE) = Some(timer.arm_gate()) };
    }

    /// Whether MOE is currently set by this layer.
    #[inline]
    pub fn armed() -> bool {
        ARMED.load(Ordering::Relaxed)
    }

    /// Whether an arm attempt has been refused this boot (a level into
    /// [`motor::motor_fault_level`]).
    #[inline]
    pub fn refused() -> bool {
        ARM_REFUSED.load(Ordering::Relaxed)
    }

    /// Enact this tick's arming decision. Called by the 250 Hz control task AFTER the demand word
    /// is published, so a shutdown's zeroed demand is the last word written this tick rather than
    /// one the same tick overwrites.
    pub fn enact(moe_allowed: bool, brought_up: bool, fault_level: bool) {
        match decide(moe_allowed, armed(), brought_up, fault_level) {
            ArmDecision::Idle => {}
            ArmDecision::Arm => run_arm(),
            ArmDecision::Shutdown => run_shutdown(),
        }
    }

    /// [`ARM_STEPS`], in order. A refused confirm aborts BEFORE the MOE step and runs the shutdown
    /// sequence instead, so the failure path leaves the bridge in the disarmed, counter-stopped
    /// posture rather than half-way through an arm.
    fn run_arm() {
        for step in ARM_STEPS {
            match step {
                ArmStep::StartCounter => motor::hw::start_counter(),
                ArmStep::ConfirmPeriodsLive => {
                    if !confirm_periods_live() {
                        ARM_REFUSED.store(true, Ordering::Relaxed);
                        run_shutdown();
                        return;
                    }
                }
                ArmStep::ZeroDemand => zero_demand(),
                ArmStep::SetMoe => {
                    // SAFETY: read-only access to a static written once on the boot thread; the
                    // 250 Hz control task is the only reader.
                    if let Some(g) = unsafe { (*addr_of_mut!(GATE)).as_ref() } {
                        g.arm();
                        ARMED.store(true, Ordering::Relaxed);
                    }
                }
            }
        }
    }

    /// [`SHUTDOWN_STEPS`], in order, MOE first.
    fn run_shutdown() {
        for step in SHUTDOWN_STEPS {
            match step {
                ShutdownStep::Disarm => {
                    // SAFETY: as `run_arm`.
                    if let Some(g) = unsafe { (*addr_of_mut!(GATE)).as_ref() } {
                        g.disarm();
                    }
                    ARMED.store(false, Ordering::Relaxed);
                }
                ShutdownStep::ZeroDemand => zero_demand(),
                ShutdownStep::FloatChannels => motor::hw::float_all_channels(),
                ShutdownStep::StopCounter => motor::hw::stop_counter(),
            }
        }
    }

    /// Zero the demand word and bump its sequence, so the ISR sees a FRESH write of zero rather
    /// than an unchanged word its freshness guard would keep ageing.
    fn zero_demand() {
        motor::DEMAND.store(0, Ordering::Relaxed);
        motor::DEMAND_SEQ.fetch_add(1, Ordering::Relaxed);
    }

    /// Watch [`motor::PERIODS`] advance, bounded by [`ARM_CONFIRM_SPINS`].
    fn confirm_periods_live() -> bool {
        let start = motor::PERIODS.load(Ordering::Relaxed);
        let mut spins = ARM_CONFIRM_SPINS;
        while spins > 0 {
            if motor::PERIODS.load(Ordering::Relaxed) != start {
                return true;
            }
            spins -= 1;
        }
        false
    }
}

// -------------------------------------------------------------------------------------------
// Host tests
// -------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::{
        motor_fault_level, FAULT_DEMAND_STALE, FAULT_DUTY_RANGE, FAULT_HALL, FAULT_INIT_CAL,
    };

    /// The mode machine is the sole arming authority: with its allowance withdrawn, nothing this
    /// layer knows can arm.
    #[test]
    fn nothing_arms_without_the_mode_machines_allowance() {
        for brought_up in [false, true] {
            for fault in [false, true] {
                assert_eq!(
                    decide(false, false, brought_up, fault),
                    ArmDecision::Idle,
                    "moe_allowed false must never arm"
                );
            }
        }
    }

    /// A motor that was never brought up cannot be armed, however willing the mode machine is.
    #[test]
    fn an_unconfigured_motor_cannot_be_armed() {
        assert_eq!(decide(true, false, false, false), ArmDecision::Idle);
    }

    /// The fault level vetoes an arm and forces a shutdown, in both directions.
    #[test]
    fn the_fault_level_refuses_an_arm_and_demands_a_shutdown() {
        assert_eq!(decide(true, false, true, true), ArmDecision::Idle, "no arm");
        assert_eq!(
            decide(true, true, true, true),
            ArmDecision::Shutdown,
            "a fault while armed shuts down even with MOE still allowed"
        );
    }

    /// The happy path and its idle steady state: one arm on the rising edge, nothing after.
    #[test]
    fn arming_is_an_edge_not_a_repeated_write() {
        assert_eq!(decide(true, false, true, false), ArmDecision::Arm);
        assert_eq!(decide(true, true, true, false), ArmDecision::Idle);
    }

    /// The falling edge: the allowance withdrawn is a shutdown, and a disarmed system with no
    /// allowance is idle (never a second shutdown).
    #[test]
    fn withdrawing_the_allowance_shuts_down_once() {
        assert_eq!(decide(false, true, true, false), ArmDecision::Shutdown);
        assert_eq!(decide(false, false, true, false), ArmDecision::Idle);
    }

    /// **The standing rule: the gate consumes the LEVEL, never the raw `FAULT` word.** The two
    /// non-producer bits are exactly the ones that would otherwise block arming for a boot: a
    /// single stale-demand period (which the ISR has already silenced by floating every phase) and
    /// a refused duty write (which changed no output). Driven through the level's single owner, so
    /// this test breaks if a producer is ever quietly added or removed.
    #[test]
    fn the_non_producer_fault_bits_never_block_arming() {
        for word in [
            FAULT_DEMAND_STALE,
            FAULT_DUTY_RANGE,
            FAULT_DEMAND_STALE | FAULT_DUTY_RANGE,
        ] {
            let level = motor_fault_level(true, word, false, false);
            assert!(!level, "word {word:#x} is not a fault producer");
            assert_eq!(
                decide(true, false, true, level),
                ArmDecision::Arm,
                "word {word:#x} must not block arming"
            );
        }
        // ...and the producers do block it, through the same level.
        for word in [FAULT_HALL, FAULT_INIT_CAL] {
            let level = motor_fault_level(true, word, false, false);
            assert!(level);
            assert_eq!(decide(true, false, true, level), ArmDecision::Idle);
        }
        // As does a refused arm, so a board that failed to arm cannot silently retry forever.
        assert!(motor_fault_level(true, 0, false, true));
        assert_eq!(
            decide(true, false, true, motor_fault_level(true, 0, false, true)),
            ArmDecision::Idle
        );
    }

    /// The arm ordering: MOE is LAST, after the liveness confirm, and appears exactly once.
    #[test]
    fn arm_step_order() {
        let idx = |s: ArmStep| ARM_STEPS.iter().position(|x| *x == s).unwrap();
        assert_eq!(*ARM_STEPS.last().unwrap(), ArmStep::SetMoe);
        assert!(idx(ArmStep::StartCounter) < idx(ArmStep::ConfirmPeriodsLive));
        assert!(idx(ArmStep::ConfirmPeriodsLive) < idx(ArmStep::SetMoe));
        assert!(idx(ArmStep::ZeroDemand) < idx(ArmStep::SetMoe));
        assert_eq!(
            ARM_STEPS.iter().filter(|s| **s == ArmStep::SetMoe).count(),
            1,
            "one energize act, not two"
        );
    }

    /// **`disarm-before-shutdown-steps`** (`specs/motor-integration.md`, Validation): MOE is
    /// cleared FIRST, before the demand zeroing, the float and the counter stop, so the silencing
    /// act never waits on a step that could fail.
    #[test]
    fn shutdown_step_order_clears_moe_first() {
        assert_eq!(*SHUTDOWN_STEPS.first().unwrap(), ShutdownStep::Disarm);
        let idx = |s: ShutdownStep| SHUTDOWN_STEPS.iter().position(|x| *x == s).unwrap();
        assert!(idx(ShutdownStep::Disarm) < idx(ShutdownStep::ZeroDemand));
        assert!(idx(ShutdownStep::ZeroDemand) < idx(ShutdownStep::FloatChannels));
        assert!(idx(ShutdownStep::FloatChannels) < idx(ShutdownStep::StopCounter));
        for s in SHUTDOWN_STEPS {
            assert_eq!(SHUTDOWN_STEPS.iter().filter(|x| **x == s).count(), 1);
        }
    }

    /// The shutdown list is the arm list inverted: every arm step has its undo, and the two
    /// independent silencing paths (MOE and the demand word) are both present and separate.
    #[test]
    fn the_shutdown_list_inverts_the_arm_list() {
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::Disarm)); // undoes SetMoe
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::StopCounter)); // undoes StartCounter
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::ZeroDemand)); // and ArmStep::ZeroDemand
        assert_eq!(ARM_STEPS.len(), SHUTDOWN_STEPS.len());
    }

    /// The liveness confirm's budget: a healthy motor satisfies it in well under a period, and a
    /// dead one costs one tick, far inside the 500 ms IWDG window.
    #[test]
    fn arm_confirm_budget() {
        // A period is 4500 cycles at 72 MHz; a poll iteration is at least ~4 cycles.
        let healthy_iters = 4500 / 4;
        assert!(
            ARM_CONFIRM_SPINS > healthy_iters * 20,
            "the budget must be many periods wide, is {ARM_CONFIRM_SPINS}"
        );
        // Worst case, on the slower family's flash fetch (~16 cycles per iteration).
        let worst_ms = (ARM_CONFIRM_SPINS as u64 * 16 * 1000) / 72_000_000;
        assert!(worst_ms < 10, "a dead ISR costs {worst_ms} ms to refuse");
    }

    /// OFF-inhibit from the raw speed word: any net edge in the window is motion.
    #[test]
    fn off_inhibit_follows_wheel_motion() {
        assert!(!off_inhibit_from_speed(0), "a stationary wheel");
        assert!(off_inhibit_from_speed(1));
        assert!(off_inhibit_from_speed(-1), "either direction");
        assert!(off_inhibit_from_speed(i32::MIN));
        assert!(off_inhibit_from_speed(i32::MAX));
    }

    /// **The arming surface is confined to this file** (`specs/motor-integration.md`, slice 5;
    /// the successor to slice 3's "the arm call absent from the tree entirely"). Slice 3 could
    /// assert absence because nothing armed; slice 5 arms, so the property that replaces absence is
    /// CONFINEMENT: the arming gate is named in `arm.rs` and nowhere else in the crate, and the one
    /// call that sets MOE occurs exactly once. That is bring-up step 11's "one visible boundary in
    /// one place", enforced by a test rather than by review.
    ///
    /// Comment lines are stripped first, so prose may discuss arming while code may not. The tokens
    /// are assembled from pieces so this test's own source does not contain them (it scans itself).
    #[test]
    fn the_arming_surface_is_confined_to_this_file() {
        let strip = |src: &str| -> std::string::String {
            src.lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .collect::<std::vec::Vec<_>>()
                .join("\n")
        };
        let gate_type = concat!("Arm", "Gate");
        let arm_call = concat!(".arm", "()");
        let disarm_call = concat!(".dis", "arm()");

        // Every OTHER file in the crate: the arming surface is absent, exactly as it was at
        // slice 3. `motor.rs` in particular stays disarmed by construction.
        for (name, src) in [
            ("main.rs", include_str!("main.rs")),
            ("motor.rs", include_str!("motor.rs")),
        ] {
            let code = strip(src);
            for token in [gate_type, arm_call, disarm_call, concat!("CC", "HP")] {
                assert!(
                    !code.contains(token),
                    "{name} names `{token}`: the arming surface belongs to arm.rs alone"
                );
            }
        }

        // This file: the gate is here, and MOE is set in exactly ONE place.
        let code = strip(include_str!("arm.rs"));
        assert!(
            code.contains(gate_type),
            "arm.rs must be the file that holds the arming gate"
        );
        assert_eq!(
            code.matches(arm_call).count(),
            1,
            "exactly one call sets MOE in the whole crate"
        );
        assert_eq!(
            code.matches(disarm_call).count(),
            1,
            "exactly one call clears MOE in the whole crate"
        );
        // The raw CCHP register is the HAL's; this crate reaches MOE only through the gate.
        assert!(!code.contains(concat!("CC", "HP")));
    }
}
