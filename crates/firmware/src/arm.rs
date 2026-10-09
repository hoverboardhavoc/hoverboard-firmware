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

use crate::motor;

// -------------------------------------------------------------------------------------------
// The ordered sequences (pure)
// -------------------------------------------------------------------------------------------

/// One step of the arm sequence. Exactly one step can set MOE, and it is LAST.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmStep {
    /// Re-read the VALUE-ONLY store fields and install them, all or nothing
    /// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read"). FIRST,
    /// before the counter even starts, for three reasons that are each load-bearing:
    ///
    /// - it runs with the bridge DISARMED, because [`ArmStep::SetMoe`] is the last step, so nothing
    ///   it installs can reach a gate driver before every other precondition has passed;
    /// - it runs with the config-write path IDLE, because R4 (the armed config-write gate,
    ///   `specs/integration.md`) refuses every `CONFIG_WRITE` while armed, so no writer can be
    ///   half-way through the fields being read;
    /// - it is ALL OR NOTHING ([`rederive`] derives and validates before anything is written), so a
    ///   refusal applies nothing.
    ///
    /// A refusal takes the EXISTING refusal route rather than arming on a stale or half-applied
    /// value: `ARM_REFUSED` and the shutdown sequence, exactly as a failed
    /// [`ArmStep::ConfirmPeriodsLive`].
    ReReadValues,
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
pub const ARM_STEPS: [ArmStep; 5] = [
    ArmStep::ReReadValues,
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

// -------------------------------------------------------------------------------------------
// The arm-time re-read (pure; `specs/integration.md`, "When a stored value takes effect: the
// arm-time re-read")
// -------------------------------------------------------------------------------------------

/// The VALUE ROW, as read from flash: the nine fields a disarmed `CONFIG_WRITE` can change that
/// take effect at the next ARM rather than at the next boot (`specs/integration.md`, the decision's
/// table).
///
/// The row is "values the loop consumes": RAM state or arithmetic, touching no peripheral register.
/// What is deliberately ABSENT is the other row, "peripheral configuration": the pin assignments
/// and their alternate functions, the timer, the injected ADC ranks, and `motor.dead_time`.
/// Changing any of those means re-configuring hardware the 16 kHz period ISR is running on, so they
/// stay at bring-up, by the owner's agreement; `motor.dead_time` in particular is a live timer
/// register (DTG) and re-applying it would mean poking a running timer for no pressing gain.
///
/// Three more fields are absent for their own reasons, not because they are peripheral:
/// `CONTROL_MODE` is not in the value row (a mode switch has its own disarmed seam,
/// `control::ControlDispatch::switch_mode`), `IMU_GYRO_BIAS` is carried through from the IMU's
/// installed config rather than re-read (see [`rederive`]), and the two `CONTROL_GAIN_*` fields are
/// already live through the tune lane (`specs/rider-ui.md` section 4, with `reconcile_gains`
/// handling their persist path). Only their MAXIMA are here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArmValues {
    /// `store::MOTOR_METHOD` (0x21).
    pub method_byte: u8,
    /// `store::MOTOR_CURRENT_LIMIT` (0x20), milliamps.
    pub current_limit_ma: u32,
    /// `store::MOTOR_CURRENT_CAL` (0x67) at motor 0, stock current counts per amp.
    pub current_cal: u16,
    /// `store::IMU_AXIS_SIGN` (0x65), indices 0..5 = `[ax, ay, az, gx, gy, gz]`.
    pub imu_sign: [i32; 6],
    /// `store::IMU_AXIS_ROLE` (0x68), indices 0..1 = `[UP, PITCH_RATE]`.
    pub imu_roles: [u8; 2],
    /// `store::CONTROL_RIDER_REQUIRED` (0x23), raw: the owning type decodes it.
    pub rider_required_byte: u8,
    /// `store::CONTROL_BATTERY_FLOOR` (0x24), centivolts, `<= 0` = no floor.
    pub battery_floor: i16,
    /// `store::CONTROL_DRIVE_LEAN` (0x73), indices 0..1 = `[lean_max, lean_slew]`, centidegrees.
    pub drive_lean: [i16; 2],
    /// `store::CONTROL_GAIN_MAX` (0x74), indices 0..2 = the per-gain inclusive upper bounds.
    pub gain_max: [i16; 3],
}

/// Read the whole value row, the ONE read site.
///
/// Generic over the flash seam so the host tests drive it against a real mounted `Store`. Every
/// read is infallible (`Store::get` returns the registered default for a missing record), so this
/// cannot fail and a virgin board re-derives exactly the defaults the boot path used.
///
/// The firmware brings up motor 0, so `MOTOR_CURRENT_CAL` is read at index 0, matching
/// `board::plumbing`'s own per-motor `.at(m)` fill.
pub fn read_arm_values<F: store::Flash>(s: &store::Store<F>) -> ArmValues {
    let mut imu_sign = [0i32; 6];
    for (i, v) in imu_sign.iter_mut().enumerate() {
        *v = s.get(store::IMU_AXIS_SIGN.at(i as u8));
    }
    ArmValues {
        method_byte: s.get(store::MOTOR_METHOD),
        current_limit_ma: s.get(store::MOTOR_CURRENT_LIMIT),
        current_cal: s.get(store::MOTOR_CURRENT_CAL.at(0)),
        imu_sign,
        imu_roles: [
            s.get(store::IMU_AXIS_ROLE.at(0)),
            s.get(store::IMU_AXIS_ROLE.at(1)),
        ],
        rider_required_byte: s.get(store::CONTROL_RIDER_REQUIRED),
        battery_floor: s.get(store::CONTROL_BATTERY_FLOOR),
        drive_lean: [
            s.get(store::CONTROL_DRIVE_LEAN.at(0)),
            s.get(store::CONTROL_DRIVE_LEAN.at(1)),
        ],
        gain_max: [
            s.get(store::CONTROL_GAIN_MAX.at(0)),
            s.get(store::CONTROL_GAIN_MAX.at(1)),
            s.get(store::CONTROL_GAIN_MAX.at(2)),
        ],
    }
}

/// Everything an arm-time re-read installs, derived and validated, ready to apply.
#[derive(Clone, Copy, Debug)]
pub struct Rederived {
    /// The period ISR's value set (method, current limit, fresh records).
    pub motor: motor::Rederived,
    /// The IMU's re-staged axis frame, or `None` on a board with no IMU (nothing to install).
    pub imu: Option<imu::Config>,
    /// `CONTROL_RIDER_REQUIRED` raw, for `control::ControlDispatch` to decode by its own rule.
    pub rider_required_byte: u8,
    /// `CONTROL_BATTERY_FLOOR` as written (the dispatch takes the word, `<= 0` = no floor).
    pub battery_floor: i16,
    /// The drive-lean bound and rate, through `control::DriveLean::new`'s seam clamp.
    pub drive_lean: control::DriveLean,
    /// The gain maxima as read; `control::GainShadow` applies its own non-negative floor.
    pub gain_max: [i16; 3],
}

/// Derive and VALIDATE the whole value row before anything is written. `None` means the arm must be
/// REFUSED.
///
/// **The all-or-nothing shape is load-bearing.** Everything is derived and validated here, in RAM,
/// against nothing the loop can see; only a `Some` is applied, and it is applied as a whole. So a
/// refusal applies NOTHING and no half-applied value set can reach the loop: there is no state in
/// which the current limit came from the new flash values while the axis frame came from the old
/// ones.
///
/// `imu_bias` is the IMU's INSTALLED gyro bias, carried through rather than re-read, because
/// `IMU_GYRO_BIAS` is not in the value row: `Some(bias)` on a board whose IMU was brought up (the
/// boot-read bias, from `imu::Imu::config()`), `None` on a board with no IMU. With `Some`, a frame
/// `imu::Config::staged` refuses makes the WHOLE re-derivation `None`, because a wrong axis role
/// means balancing about the wrong axis and nothing in the loop can tell. With `None` the `imu`
/// field is `None` and that is NOT a refusal: a throttle-only board must still arm exactly as it
/// does today.
pub fn rederive(
    values: &ArmValues,
    boot: motor::BootFixed,
    imu_bias: Option<[i32; 3]>,
) -> Option<Rederived> {
    // The one validating step: an IMU frame that is not a proper rotation with distinct roles is
    // refused here, before anything is installed, exactly as the boot bring-up refuses it.
    let imu = match imu_bias {
        Some(bias) => Some(imu::Config::staged(values.imu_sign, bias, values.imu_roles).ok()?),
        None => None,
    };
    Some(Rederived {
        motor: motor::rederive(
            values.method_byte,
            values.current_limit_ma,
            values.current_cal,
            boot,
        ),
        imu,
        rider_required_byte: values.rider_required_byte,
        battery_floor: values.battery_floor,
        // The seam clamp, applied by the type that owns it (`lean_max` 0..1500, `lean_slew` 1..100).
        drive_lean: control::DriveLean::new(values.drive_lean[0], values.drive_lean[1]),
        gain_max: values.gain_max,
    })
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
    ///
    /// `rederive` is the arm-time re-read's whole apply ([`ArmStep::ReReadValues`]): it returns
    /// whether every applicable value was installed, and it is called only on the ARM path. A
    /// closure rather than a value because it reaches the caller's shell and the store static, and
    /// `FnMut` because [`run_arm`] calls it from the step loop.
    pub fn enact(
        moe_allowed: bool,
        brought_up: bool,
        fault_level: bool,
        mut rederive: impl FnMut() -> bool,
    ) {
        match decide(moe_allowed, armed(), brought_up, fault_level) {
            ArmDecision::Idle => {}
            ArmDecision::Arm => run_arm(&mut rederive),
            ArmDecision::Shutdown => run_shutdown(),
        }
    }

    /// [`ARM_STEPS`], in order. A refused step aborts BEFORE the MOE step and runs the shutdown
    /// sequence instead, so the failure path leaves the bridge in the disarmed, counter-stopped
    /// posture rather than half-way through an arm.
    ///
    /// Two steps can refuse, and both take the same route: the arm-time re-read
    /// ([`ArmStep::ReReadValues`]) and the liveness confirm ([`ArmStep::ConfirmPeriodsLive`]).
    /// `ARM_REFUSED` is STICKY for the boot and feeds [`motor::motor_fault_level`], so either
    /// refusal shuts the board down loudly and does not retry until the next boot: a board that
    /// could not install the values it was told to run on does not quietly run on the old ones.
    fn run_arm(rederive: &mut impl FnMut() -> bool) {
        for step in ARM_STEPS {
            match step {
                ArmStep::ReReadValues => {
                    if !rederive() {
                        ARM_REFUSED.store(true, Ordering::Relaxed);
                        run_shutdown();
                        return;
                    }
                }
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

    /// The arm ordering: the value re-read is FIRST, MOE is LAST, after the liveness confirm, and
    /// appears exactly once.
    #[test]
    fn arm_step_order() {
        let idx = |s: ArmStep| ARM_STEPS.iter().position(|x| *x == s).unwrap();
        // FIRST, and therefore before the energize act: the re-read runs on a disarmed bridge
        // (`specs/integration.md`, the arm-time re-read).
        assert_eq!(*ARM_STEPS.first().unwrap(), ArmStep::ReReadValues);
        assert!(idx(ArmStep::ReReadValues) < idx(ArmStep::SetMoe));
        assert!(idx(ArmStep::ReReadValues) < idx(ArmStep::StartCounter));
        assert_eq!(
            ARM_STEPS
                .iter()
                .filter(|s| **s == ArmStep::ReReadValues)
                .count(),
            1,
            "one re-read, not two"
        );
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

    /// The shutdown list inverts the arm list: every arm step that changes HARDWARE state has its
    /// undo, and the two independent silencing paths (MOE and the demand word) are both present and
    /// separate.
    ///
    /// The two lists are no longer the same LENGTH, and that is the honest shape rather than a gap:
    /// [`ArmStep::ReReadValues`] installs VALUES (a method byte, a current limit, RAM records) and
    /// has no undo, because there is no previous value to restore to and nothing it wrote can
    /// energize anything. The shutdown's job is to silence a bridge; the step list it inverts is the
    /// one that can make a bridge live. So the property asserted is a one-way one: every
    /// hardware-touching arm step has an undo, and the shutdown list carries exactly the four.
    #[test]
    fn the_shutdown_list_inverts_the_arm_list() {
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::Disarm)); // undoes SetMoe
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::StopCounter)); // undoes StartCounter
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::ZeroDemand)); // and ArmStep::ZeroDemand
                                                                     // The arm steps that touch hardware, each with its undo above; `ReReadValues` is the one
                                                                     // that does not and the one with no undo.
        let hardware: std::vec::Vec<ArmStep> = ARM_STEPS
            .into_iter()
            .filter(|s| *s != ArmStep::ReReadValues)
            .collect();
        assert_eq!(
            hardware.len(),
            SHUTDOWN_STEPS.len(),
            "every hardware-touching arm step has its undo"
        );
        assert_eq!(ARM_STEPS.len(), SHUTDOWN_STEPS.len() + 1);
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

    // -----------------------------------------------------------------------------------------
    // The arm-time re-read (`specs/integration.md`, "When a stored value takes effect: the
    // arm-time re-read")
    // -----------------------------------------------------------------------------------------

    /// A mounted store over the crate's in-RAM host flash, with `f` applied to it.
    fn with_store(f: impl FnOnce(&mut store::Store<crate::test_flash::TestFlash>)) -> ArmValues {
        let mut flash = crate::test_flash::TestFlash::erased();
        let mut s = store::Store::mount(&mut flash).unwrap();
        f(&mut s);
        read_arm_values(&s)
    }

    /// A valid staged frame: the compiled reference map's signs with the identity roles, which
    /// `imu::Config::staged` accepts (both triples are proper rotations).
    const GOOD_SIGN: [i32; 6] = [-1, 1, -1, -1, 1, -1];
    const GOOD_ROLES: [u8; 2] = [3, 2];

    /// A board's boot-fixed decode facts, as the bring-up would have built them.
    const BOOT: motor::BootFixed = motor::BootFixed {
        direction: false,
        align_offset: 2,
    };

    /// **A value written while disarmed is picked up by the re-read.** Every one of the nine
    /// value-row fields, written to a real mounted store and read back through the one read site.
    #[test]
    fn the_value_row_is_read_from_flash() {
        let v = with_store(|s| {
            s.set(store::MOTOR_METHOD, 2).unwrap();
            s.set(store::MOTOR_CURRENT_LIMIT, 7_500).unwrap();
            s.set(store::MOTOR_CURRENT_CAL.at(0), 300).unwrap();
            for (i, sign) in [1i32, -1, 1, 1, -1, 1].into_iter().enumerate() {
                s.set(store::IMU_AXIS_SIGN.at(i as u8), sign).unwrap();
            }
            s.set(store::IMU_AXIS_ROLE.at(0), 1).unwrap();
            s.set(store::IMU_AXIS_ROLE.at(1), 3).unwrap();
            s.set(store::CONTROL_RIDER_REQUIRED, 0).unwrap();
            s.set(store::CONTROL_BATTERY_FLOOR, 2_900).unwrap();
            s.set(store::CONTROL_DRIVE_LEAN.at(0), 900).unwrap();
            s.set(store::CONTROL_DRIVE_LEAN.at(1), 7).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(0), 1_234).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(1), 567).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(2), 89).unwrap();
        });
        assert_eq!(
            v,
            ArmValues {
                method_byte: 2,
                current_limit_ma: 7_500,
                current_cal: 300,
                imu_sign: [1, -1, 1, 1, -1, 1],
                imu_roles: [1, 3],
                rider_required_byte: 0,
                battery_floor: 2_900,
                drive_lean: [900, 7],
                gain_max: [1_234, 567, 89],
            }
        );
    }

    /// **The value row cannot silently grow.** A BRING-UP-ROW field written to the same store does
    /// not appear in the re-read: `motor.dead_time` (a live timer register), a pin assignment, and
    /// the two decode facts the arm path takes from `BootFixed` instead. If one of these is ever
    /// added to the row, this test fails and the addition has to be a deliberate edit of the
    /// spec's table.
    #[test]
    fn a_bring_up_row_field_is_not_in_the_value_row() {
        let untouched = with_store(|_| {});
        let v = with_store(|s| {
            s.set(store::MOTOR_DEAD_TIME, 0x1C).unwrap();
            s.set(store::MOTOR_HALL_A, 0x2D).unwrap();
            s.set(store::MOTOR_DIRECTION, 1).unwrap();
            s.set(store::MOTOR_ALIGN_OFFSET, 5).unwrap();
        });
        assert_eq!(
            v, untouched,
            "a bring-up-row write must not change what the arm path re-reads"
        );
    }

    /// **Fail closed on a refused IMU frame.** A staged frame `imu::Config::staged` refuses makes
    /// the WHOLE re-derivation `None` (so the arm is refused and nothing is applied), while the
    /// same values with a valid frame re-derive.
    #[test]
    fn a_refused_imu_frame_refuses_the_whole_rederivation() {
        let values = |sign: [i32; 6], roles: [u8; 2]| ArmValues {
            method_byte: 0,
            current_limit_ma: 10_000,
            current_cal: 455,
            imu_sign: sign,
            imu_roles: roles,
            rider_required_byte: 1,
            battery_floor: 2_400,
            drive_lean: [0, 4],
            gain_max: [20_000, 10_000, 1_000],
        };
        let bias = Some([48, 13, -88]);
        // A reflection (an odd number of negated axes) and a role pair naming one chip axis twice:
        // both are frames `staged` refuses, and both refuse the arm.
        for (sign, roles) in [
            ([1, 1, -1, -1, 1, -1], GOOD_ROLES),
            (GOOD_SIGN, [2, 2]),
            (GOOD_SIGN, [0, 9]),
        ] {
            assert!(
                imu::Config::staged(sign, [0; 3], roles).is_err(),
                "the fixture must be a frame `staged` refuses"
            );
            assert!(
                rederive(&values(sign, roles), BOOT, bias).is_none(),
                "a refused frame applies nothing"
            );
        }
        // The same values with a good frame re-derive, and the carried-through bias is installed.
        let good = rederive(&values(GOOD_SIGN, GOOD_ROLES), BOOT, bias).expect("a valid frame");
        assert_eq!(
            good.imu.expect("an IMU board stages a config").gyro_bias,
            [48, 13, -88]
        );
    }

    /// **A board with no IMU still re-derives.** `imu_bias: None` is "this board has no IMU", not a
    /// refusal: the throttle-only board arms exactly as it does today, with no config to install,
    /// and even a frame that would be refused cannot stop it (nothing reads those fields).
    #[test]
    fn a_board_with_no_imu_still_rederives() {
        let values = ArmValues {
            method_byte: 1,
            current_limit_ma: 12_000,
            current_cal: 455,
            // Deliberately a reflection: with no IMU it is never staged, so it cannot refuse.
            imu_sign: [1, 1, -1, -1, 1, -1],
            imu_roles: [2, 2],
            rider_required_byte: 0,
            battery_floor: 0,
            drive_lean: [1_500, 100],
            gain_max: [20_000, 10_000, 1_000],
        };
        let r = rederive(&values, BOOT, None).expect("no IMU is not a refusal");
        assert!(r.imu.is_none(), "nothing to install");
        // The rest of the row is still derived, through its own owners.
        assert_eq!(
            r.motor.method,
            commutation::CommutationMethod::SixStep.to_u8()
        );
        assert_eq!(r.rider_required_byte, 0);
        assert_eq!(r.battery_floor, 0);
        assert_eq!(r.drive_lean, control::DriveLean::new(1_500, 100));
        assert_eq!(r.gain_max, [20_000, 10_000, 1_000]);
    }

    /// The seam clamps ride on the owning types rather than on this layer: an out-of-range
    /// `CONTROL_DRIVE_LEAN` pair is clamped by `DriveLean::new`, which is the only place that
    /// range lives.
    #[test]
    fn the_drive_lean_goes_through_its_seam_clamp() {
        let mut values = ArmValues {
            method_byte: 0,
            current_limit_ma: 10_000,
            current_cal: 455,
            imu_sign: GOOD_SIGN,
            imu_roles: GOOD_ROLES,
            rider_required_byte: 1,
            battery_floor: 2_400,
            drive_lean: [i16::MAX, 0],
            gain_max: [20_000, 10_000, 1_000],
        };
        let r = rederive(&values, BOOT, None).unwrap();
        assert_eq!(r.drive_lean.lean_max(), control::drive::LEAN_MAX_CEIL);
        assert_eq!(r.drive_lean.lean_slew(), control::drive::LEAN_SLEW_MIN);
        values.drive_lean = [-1, i16::MAX];
        let r = rederive(&values, BOOT, None).unwrap();
        assert_eq!(r.drive_lean.lean_max(), 0, "negative is the disabled state");
        assert_eq!(r.drive_lean.lean_slew(), control::drive::LEAN_SLEW_MAX);
    }

    /// **The arm-time install cannot disturb the bring-up's measured phase-current offsets**
    /// (`specs/integration.md`, the arm-time re-read's second edge: an arm-time method change does
    /// not need the quiet-bridge calibration redone, so the measured pair is carried through).
    ///
    /// Half of that property is already structural and needs no test: [`motor::Rederived`] carries
    /// no offsets field, so the install has nothing to write them FROM. The other half is that the
    /// install does not reach past its argument into the runtime's other fields, and the honest
    /// check for that is a source scan of the install's body, in the spirit of the confinement test
    /// below: it must not name `offsets`, `base_flags`, `faults`, `FAULT` or `OBS_CAL`. The field
    /// names are assembled from pieces so this test's own source does not contain them.
    #[test]
    fn the_arm_time_install_does_not_touch_the_measured_offsets() {
        let src = include_str!("motor.rs");
        let fname = concat!("install_", "rederived");
        let at = src
            .find(&std::format!("pub fn {fname}"))
            .expect("the install must exist in motor.rs");
        let body = &src[at..];
        let end = body.find("\n    }\n").expect("the install's body ends");
        let body: std::string::String = body[..end]
            .lines()
            .filter(|l| !l.trim_start().starts_with("//"))
            .collect::<std::vec::Vec<_>>()
            .join("\n");
        for token in [
            concat!("off", "sets"),
            concat!("base_", "flags"),
            concat!("fau", "lts"),
            concat!("FAU", "LT"),
            concat!("OBS_", "CAL"),
            // A BUILT limit record would carry a zeroed trip count into the ISR, and that count is
            // boot-cumulative (`motor::CurrentLimit::reconfigure` holds the split). The install
            // must go through the seam, so the constructor must not appear here.
            concat!("CurrentLimit::", "new"),
        ] {
            assert!(
                !body.contains(token),
                "the arm-time install names `{token}`: it installs the value row and nothing else"
            );
        }
        // ...and it does install the three it is for, each through its owning seam.
        for token in [
            "method",
            "current",
            concat!("re", "configure"),
            concat!("switch_", "method"),
        ] {
            assert!(body.contains(token), "the install must write `{token}`");
        }
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
