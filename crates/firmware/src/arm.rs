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
    /// - it runs with the config-write path IDLE, and the guarantee for that is the EXECUTION
    ///   MODEL, not R4: the link drains that reach `store.set_value` (service-loop step 2) and the
    ///   task dispatch that reaches this step (step 6) run on the SAME thread, so a write cannot be
    ///   in flight while this reads. R4 (the armed config-write gate, `specs/integration.md`) is a
    ///   different statement, about writes once the board IS armed, and it is not what this step
    ///   rests on: the responder's armed flag is sampled at step 4, before the dispatch that arms
    ///   at step 6, so R4's refusal begins a pass later than MOE does. That window is
    ///   pre-existing and belongs to `crates/net`, not here;
    /// - it is ALL OR NOTHING ([`rederive`] derives and validates before anything is written), so a
    ///   refusal applies nothing.
    ///
    /// A refusal refuses the ARM rather than arming on a stale or half-applied value: it is held
    /// as a level into [`motor::motor_fault_level`] and runs the shutdown sequence, so the mode
    /// machine shuts the board down. It is RETRYABLE, which is where it parts company with
    /// [`ArmStep::ConfirmPeriodsLive`] (`specs/integration.md`, "A refused re-read refuses the ARM,
    /// not the boot"): this step's verdict is on a STORED NUMBER, which heals the instant a correct
    /// one is written, so the refusal is released once the engage that met it ends (an OFF pass
    /// with the power request dropped) and the NEXT engage re-reads. The confirm's verdict is a
    /// MEASUREMENT of a wedged vector, which no write heals, so it stays sticky for the boot.
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

/// Derive and VALIDATE the whole value row before anything is written. `Err` means the arm must be
/// REFUSED, and carries WHICH of the three frame checks refused it, because that is the one cause
/// of a refused arm an operator can act on: it reaches `CTRL_OBS` word 33's cause byte through the
/// caller's re-read seam (`specs/integration.md`, "The arm refusals"), named exactly as the boot
/// path names the same frame in `BOARD_OBS`.
///
/// **The all-or-nothing shape is load-bearing.** Everything is derived and validated here, in RAM,
/// against nothing the loop can see; only an `Ok` is applied, and it is applied as a whole. So a
/// refusal applies NOTHING and no half-applied value set can reach the loop: there is no state in
/// which the current limit came from the new flash values while the axis frame came from the old
/// ones.
///
/// `imu_bias` is the IMU's INSTALLED gyro bias, carried through rather than re-read, because
/// `IMU_GYRO_BIAS` is not in the value row: `Some(bias)` on a board whose IMU was brought up (the
/// boot-read bias, from `imu::Imu::config()`), `None` on a board with no IMU. With `Some`, a frame
/// `imu::Config::staged` refuses makes the WHOLE re-derivation an `Err`, because a wrong axis role
/// means balancing about the wrong axis and nothing in the loop can tell. With `None` the `imu`
/// field is `None` and that is NOT a refusal: a throttle-only board must still arm exactly as it
/// does today.
pub fn rederive(
    values: &ArmValues,
    boot: motor::BootFixed,
    imu_bias: Option<[i32; 3]>,
) -> Result<Rederived, imu::FrameError> {
    // The one validating step: an IMU frame that is not a proper rotation with distinct roles is
    // refused here, before anything is installed, exactly as the boot bring-up refuses it.
    let imu = match imu_bias {
        Some(bias) => Some(imu::Config::staged(
            values.imu_sign,
            bias,
            values.imu_roles,
        )?),
        None => None,
    };
    Ok(Rederived {
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
// The refusal observation (pure; `specs/integration.md`, "The arm refusals (word 33, permanent)")
// -------------------------------------------------------------------------------------------

/// Byte lane of the [`ArmStep::ReReadValues`] refusal count in the published word.
const RE_READ_COUNT: u32 = 0;
/// Byte lane of the [`ArmStep::ConfirmPeriodsLive`] refusal count.
const CONFIRM_COUNT: u32 = 8;
/// Byte lane of the last refused IMU frame's cause byte.
const FRAME_CAUSE: u32 = 16;

/// One lane's saturating increment.
const fn bump(word: u32, lane: u32) -> u32 {
    let n = ((word >> lane) & 0xFF) as u8;
    (word & !(0xFF << lane)) | ((n.saturating_add(1) as u32) << lane)
}

/// Fold one [`ArmStep::ReReadValues`] refusal into the published refusal word (`CTRL_OBS` word 33),
/// with `cause` the refused IMU frame's cause byte (`0` = the refusal had no frame cause).
///
/// The counts SATURATE rather than wrap, because the question a bench read asks of them is "once,
/// or every attempt": a count that wrapped to 0 answers it wrongly, and no count above 255 says
/// anything the 255 did not. They are per-boot and nothing clears them, which is the whole reason
/// the word exists: the LEVEL this refusal feeds is released once the engage that met it ends, so a
/// board refused once and left disarmed would otherwise be indistinguishable from one that was
/// never engaged.
///
/// `cause == 0` leaves the cause byte as it stands, rather than blanking it. The byte names the
/// last refused FRAME, not the last refusal, and the three other ways the re-read can refuse (no
/// store mounted, no motor runtime, an install the runtime rejected) all describe a board
/// [`decide`] would not have armed anyway, so blanking the one informative byte on their account
/// would lose the only cause a reader can act on. The counts still step, so a refusal with no
/// frame cause is visible as a count that moved while the cause byte did not.
pub const fn note_re_read_refusal(word: u32, cause: u8) -> u32 {
    let w = bump(word, RE_READ_COUNT);
    if cause == 0 {
        w
    } else {
        (w & !(0xFF << FRAME_CAUSE)) | ((cause as u32) << FRAME_CAUSE)
    }
}

/// Fold one [`ArmStep::ConfirmPeriodsLive`] refusal into the published refusal word. It carries no
/// cause byte: the step's whole verdict is that [`motor::PERIODS`] did not advance.
///
/// Kept beside the re-read's count even though that refusal is boot-sticky and therefore readable
/// as a level, because the level says only THAT it is held, never how many attempts it ate.
pub const fn note_confirm_refusal(word: u32) -> u32 {
    bump(word, CONFIRM_COUNT)
}

/// The ONE mapping from a frame `imu::Config::staged` refused onto the field it names and the
/// refused triple's first `IMU_AXIS_SIGN` index: a bad role pair names `IMU_AXIS_ROLE`, a mirrored
/// accel triple names `IMU_AXIS_SIGN` index 0, a mirrored gyro triple the same field at index 3
/// (`specs/imu.md`, `IMU_AXIS_ROLE`, "Validation").
///
/// Two observers read it and they must not name the same frame differently: the BOOT refusal's
/// `BOARD_OBS` record (`main.rs`'s `imu_frame_refusal`, which builds the `BoardError` from this
/// pair) and the ARM refusal's `CTRL_OBS` cause byte ([`imu_frame_cause_byte`]). It lives in this
/// module's PURE half, with the arm-time consumer, so the agreement is host-testable: the boot path
/// is inside the target-only `mod firmware` and nothing on the host could otherwise pin it.
pub const fn imu_frame_fault(e: imu::FrameError) -> (board::BoardField, u8) {
    match e {
        imu::FrameError::Roles => (board::BoardField::ImuAxisRole, 0),
        imu::FrameError::Accel => (board::BoardField::ImuAxisSign, 0),
        imu::FrameError::Gyro => (board::BoardField::ImuAxisSign, 3),
    }
}

/// The `CTRL_OBS` word-33 cause byte for a frame the arm-time re-read refused
/// (`specs/integration.md`, "The arm refusals"), built from [`imu_frame_fault`] so it cannot name a
/// different field than the boot record does for the same frame.
///
/// Bit 7 marks a cause PRESENT, because the accel refusal's field and index are both encoded as
/// zero and a reader must be able to tell it from "no frame refusal this boot". Bit 2 says the
/// refusal names `IMU_AXIS_ROLE` rather than `IMU_AXIS_SIGN`. Bits 0..1 carry the refused triple's
/// first `IMU_AXIS_SIGN` index exactly as `BOARD_OBS`'s `detail` does (0 = accel, 3 = gyro), so the
/// two records decode by the same rule. `tools/swdobs.py` mirrors it.
pub const fn imu_frame_cause_byte(e: imu::FrameError) -> u8 {
    let (field, first) = imu_frame_fault(e);
    let names_role = matches!(field, board::BoardField::ImuAxisRole);
    0x80 | ((names_role as u8) << 2) | first
}

// -------------------------------------------------------------------------------------------
// The hardware half
// -------------------------------------------------------------------------------------------

#[cfg(target_os = "none")]
pub mod hw {
    use super::{
        decide, note_confirm_refusal, note_re_read_refusal, ArmDecision, ArmStep, ShutdownStep,
        ARM_CONFIRM_SPINS, ARM_STEPS, SHUTDOWN_STEPS,
    };
    use crate::motor;
    use core::ptr::addr_of_mut;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
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

    /// A refused [`ArmStep::ConfirmPeriodsLive`]: the period ISR could not be shown to be running,
    /// so the thing that would step the commutator is not proven alive. **Sticky for the boot**,
    /// with no clear path anywhere in this module, and deliberately so
    /// (`specs/motor-integration.md`, the motor-side fault producers): it is a MEASUREMENT of
    /// something an OFF dwell does not heal, and retrying it means repeatedly arming a bridge
    /// whose commutator may be dead. A power cycle is the recovery, as it is for the hall dwell
    /// fault and the refused calibration.
    static CONFIRM_REFUSED: AtomicBool = AtomicBool::new(false);

    /// A refused [`ArmStep::ReReadValues`]: the stored value row could not be derived and installed
    /// (a refused IMU axis frame, or no runtime to install into). **Held, then released on an OFF
    /// pass whose POWER REQUEST HAS DROPPED** (`specs/integration.md`, "A refused re-read refuses
    /// the ARM, not the boot"), which is the one way it differs from `CONFIRM_REFUSED`: it is a
    /// verdict on a STORED NUMBER, and a corrected number must take effect at the next engage
    /// rather than at the next boot.
    ///
    /// **Held rather than cleared where it is set.** The motor-side level is folded before
    /// `control_task` consumes it, so the refusal this flag records is read a tick LATER, and that
    /// tick is the one that carries the fault to the mode machine, clears `MoeGate` and runs the
    /// shutdown pass. Clearing on the refusing tick would leave `MoeGate` set with [`decide`] still
    /// returning [`ArmDecision::Arm`], and the board would re-read flash and run the whole shutdown
    /// sequence every 4 ms.
    ///
    /// **And the request half is what makes the retry one per ENGAGE.** The mode machine's power
    /// request is a LEVEL, re-seeded from the input every tick, and the OFF gate is taken on the
    /// first tick it is on with no fault asserted, so an OFF-pass release alone would retry every
    /// 20 ms while a rider leaned on the button: five ticks to the cycle, the enact records
    /// stepping with it, this boot's refusal count saturating in about five seconds, and R4 (the
    /// armed config-write refusal, `specs/integration.md`) answering `CFG_ARMED` on three ticks in
    /// five, which would refuse the corrective write that is the only way out. Keyed on
    /// `ControlOutput::off_request_clear` instead, a held request parks the board in OFF with the
    /// refusal held, which is exactly the posture in which a config write is accepted.
    static RE_READ_REFUSED: AtomicBool = AtomicBool::new(false);

    /// The refusal observation (`CTRL_OBS` word 33), maintained in the published packing by
    /// [`super::note_re_read_refusal`] / [`super::note_confirm_refusal`]: a saturating per-boot
    /// count of each step's refusals, plus the last refused IMU frame's cause byte.
    ///
    /// Nothing clears it, which is the point: `RE_READ_REFUSED` is released when the engage that
    /// met it ends, so without a count a board refused for a bad axis frame and then left disarmed
    /// reads exactly like a board that was never engaged. One writer (the 250 Hz thread, inside
    /// [`run_arm`]).
    static REFUSAL_OBS: AtomicU32 = AtomicU32::new(0);

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

    /// Whether an arm refusal is currently HELD (the level into [`motor::motor_fault_level`]).
    ///
    /// The OR of the two refusals, because the fold takes ONE bool and is the level's single owner:
    /// the two statics differ in how long they are held, never in what they mean downstream, so
    /// either one shuts the board down rather than leaving it unarmed in RUN.
    #[inline]
    pub fn refused() -> bool {
        CONFIRM_REFUSED.load(Ordering::Relaxed) || RE_READ_REFUSED.load(Ordering::Relaxed)
    }

    /// The refusal observation word, for the `CTRL_OBS` publish (word 33).
    #[inline]
    pub fn refusal_obs() -> u32 {
        REFUSAL_OBS.load(Ordering::Relaxed)
    }

    /// Enact this tick's arming decision. Called by the 250 Hz control task AFTER the demand word
    /// is published, so a shutdown's zeroed demand is the last word written this tick rather than
    /// one the same tick overwrites.
    ///
    /// `re_read` is the arm-time re-read's whole apply ([`ArmStep::ReReadValues`]): `Ok(())` once
    /// every applicable value is installed, `Err(cause)` on a refusal, carrying the observation's
    /// frame-cause byte (`0` = the refusal was not a refused axis frame). It is called only on the
    /// ARM path. A closure rather than a value because it reaches the caller's shell and the store
    /// static, and `FnMut` because [`run_arm`] calls it from the step loop.
    ///
    /// Named `re_read` rather than `rederive` deliberately: this module's `use super::*` brings
    /// [`crate::arm::rederive`] into scope, and the caller's closure also calls
    /// [`motor::rederive`], so a parameter by that name would shadow one of the two functions it is
    /// built from and leave a reader of [`run_arm`] unable to tell which is being called.
    ///
    /// `off_request_clear` is `ControlOutput::off_request_clear`, the orchestrator's published
    /// end-of-engage seam: this pass resolved to OFF and the power request is clear. It is TAKEN
    /// rather than recomputed here, because both halves are the orchestrator's own facts (the mode
    /// it just resolved and the request it folds from the button and the `INPUTS` mirror), and this
    /// layer sees only MOE. The fault latches, the `stop_all` latch and the engagement machine
    /// clear on the OFF half of the same pass; `RE_READ_REFUSED` additionally waits for the
    /// request, for the reason given on that flag.
    pub fn enact(
        moe_allowed: bool,
        brought_up: bool,
        fault_level: bool,
        off_request_clear: bool,
        mut re_read: impl FnMut() -> Result<(), u8>,
    ) {
        match decide(moe_allowed, armed(), brought_up, fault_level) {
            ArmDecision::Idle => {}
            ArmDecision::Arm => run_arm(&mut re_read),
            ArmDecision::Shutdown => run_shutdown(),
        }
        // The end-of-engage release. AFTER the decision, which cannot be affected by it either way:
        // the pass resolved to OFF with the request clear, so the decision above is a shutdown or
        // nothing, and the level the decision consumed was folded before `control_task` ran. A
        // refusal set by `run_arm` above cannot be released here on its own tick: an arm runs
        // only while MOE is allowed, which the machine grants from INIT, never on a pass that
        // resolves to OFF with no request standing.
        if off_request_clear {
            RE_READ_REFUSED.store(false, Ordering::Relaxed);
        }
    }

    /// [`ARM_STEPS`], in order. A refused step aborts BEFORE the MOE step and runs the shutdown
    /// sequence instead, so the failure path leaves the bridge in the disarmed, counter-stopped
    /// posture rather than half-way through an arm.
    ///
    /// Two steps can refuse, and both are held as the level [`motor::motor_fault_level`] folds, so
    /// either shuts the board down loudly rather than leaving it in RUN with a silent, unarmed
    /// bridge: a board that could not install the values it was told to run on does not quietly run
    /// on the old ones. What differs is how long each is held, and the two flags say why:
    /// `CONFIRM_REFUSED` is sticky for the boot, `RE_READ_REFUSED` releases at the end of the
    /// engage that met it.
    ///
    /// Both also step their count in `REFUSAL_OBS`, which nothing clears, so the refusal that does
    /// release is still visible to a bench read afterwards.
    fn run_arm(re_read: &mut impl FnMut() -> Result<(), u8>) {
        for step in ARM_STEPS {
            match step {
                ArmStep::ReReadValues => {
                    if let Err(cause) = re_read() {
                        RE_READ_REFUSED.store(true, Ordering::Relaxed);
                        REFUSAL_OBS.store(
                            note_re_read_refusal(REFUSAL_OBS.load(Ordering::Relaxed), cause),
                            Ordering::Relaxed,
                        );
                        run_shutdown();
                        return;
                    }
                }
                ArmStep::StartCounter => motor::hw::start_counter(),
                ArmStep::ConfirmPeriodsLive => {
                    if !confirm_periods_live() {
                        CONFIRM_REFUSED.store(true, Ordering::Relaxed);
                        REFUSAL_OBS.store(
                            note_confirm_refusal(REFUSAL_OBS.load(Ordering::Relaxed)),
                            Ordering::Relaxed,
                        );
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
        // As does a refused arm while it is held, so a board that failed to arm cannot sit in RUN
        // with an unarmed bridge, nor retry inside the same engage.
        assert!(motor_fault_level(true, 0, false, true));
        assert_eq!(
            decide(true, false, true, motor_fault_level(true, 0, false, true)),
            ArmDecision::Idle
        );
    }

    /// **The refusal is a LEVEL, so holding it refuses the arm and releasing it permits one**
    /// (`specs/integration.md`, "A refused re-read refuses the ARM, not the boot"): the retry the
    /// OFF-pass clear buys is an arm ATTEMPT, which means the fold and the gate have to answer
    /// differently either side of that clear, through the level's single owner.
    ///
    /// Both refusals are one `bool` here because `motor_fault_level` takes one and the arm layer
    /// hands it the OR of its two statics: what differs between them is how long each is held, not
    /// what the level means downstream.
    #[test]
    fn a_held_refusal_refuses_the_arm_and_a_released_one_permits_the_next() {
        let held = motor_fault_level(true, 0, false, true);
        assert!(held, "either refusal reaches the level while held");
        assert_eq!(
            decide(true, false, true, held),
            ArmDecision::Idle,
            "no arm attempt while the refusal is held"
        );
        // And the shutdown the mode machine runs on the tick that reads the held level is what
        // takes the pass to OFF, where the re-read refusal is released.
        assert_eq!(decide(true, true, true, held), ArmDecision::Shutdown);
        let released = motor_fault_level(true, 0, false, false);
        assert!(!released);
        assert_eq!(
            decide(true, false, true, released),
            ArmDecision::Arm,
            "the next engage after the release re-reads rather than waiting for a reboot"
        );
    }

    /// The published refusal word (`CTRL_OBS` word 33): the two counts are independent byte lanes,
    /// they saturate rather than wrap, and the cause byte rides along without disturbing either.
    #[test]
    fn the_refusal_word_counts_each_step_in_its_own_lane() {
        let w = note_re_read_refusal(0, 0x80);
        assert_eq!(
            w, 0x0080_0001,
            "count 1 in the low lane, the cause in the third"
        );
        let w = note_confirm_refusal(w);
        assert_eq!(w, 0x0080_0101, "the confirm lane steps, nothing else moves");
        let w = note_re_read_refusal(w, 0x84);
        assert_eq!(w, 0x0084_0102, "the newer cause replaces the older one");
        // A refusal with no frame cause steps the count and KEEPS the last frame's cause: the byte
        // names the last refused FRAME, and the count is what says a refusal happened.
        let w = note_re_read_refusal(w, 0);
        assert_eq!(w, 0x0084_0103);
        // Saturation, per lane, with the other lanes untouched.
        let mut re_read = 0x0084_01FEu32;
        for _ in 0..3 {
            re_read = note_re_read_refusal(re_read, 0);
        }
        assert_eq!(
            re_read, 0x0084_01FF,
            "the count saturates, it does not wrap to 0"
        );
        let mut confirm = 0x0000_FE00u32;
        for _ in 0..3 {
            confirm = note_confirm_refusal(confirm);
        }
        assert_eq!(confirm, 0x0000_FF00);
        // The reserved top byte stays zero through every fold.
        assert_eq!(re_read >> 24, 0);
        assert_eq!(confirm >> 24, 0);
    }

    /// **The cause byte, pinned to its three values** (`specs/integration.md`, "The arm refusals";
    /// `tools/swdobs.py`'s `decode_arm_refusals` decodes exactly these). The byte is the only part
    /// of a refused arm an operator can act on, and it is read by a tool outside this build, so the
    /// three numbers are the contract rather than the function's shape.
    ///
    /// It also pins the agreement the "one shared mapping" claim rests on: [`imu_frame_fault`] is
    /// what `main.rs`'s `imu_frame_refusal` builds the boot `BOARD_OBS` record from, so a frame
    /// that refuses the ARM names the same field, and the same `IMU_AXIS_SIGN` index, as the same
    /// frame refusing the BOOT. That function is inside the target-only `mod firmware` and cannot
    /// be tested here; the pair it consumes can.
    #[test]
    fn the_frame_cause_byte_names_each_check_distinctly() {
        use imu::FrameError::{Accel, Gyro, Roles};
        // Bit 7 = present, bit 2 = names IMU_AXIS_ROLE, bits 0..1 = the triple's first index.
        assert_eq!(imu_frame_cause_byte(Accel), 0x80);
        assert_eq!(imu_frame_cause_byte(Gyro), 0x83);
        assert_eq!(imu_frame_cause_byte(Roles), 0x84);
        // Distinct, and never 0: a zero byte is "no frame refusal this boot" in the published word,
        // and the accel case (field and index both zero) is the one that would otherwise collide.
        for (a, b) in [(Accel, Gyro), (Accel, Roles), (Gyro, Roles)] {
            assert_ne!(imu_frame_cause_byte(a), imu_frame_cause_byte(b));
        }
        for e in [Accel, Gyro, Roles] {
            assert_ne!(imu_frame_cause_byte(e), 0);
            // The byte survives the fold into the word and comes back out of the cause lane.
            assert_eq!(
                (note_re_read_refusal(0, imu_frame_cause_byte(e)) >> 16) & 0xFF,
                imu_frame_cause_byte(e) as u32
            );
        }
        // The pair the boot record is built from, which is what makes the two records agree.
        assert_eq!(
            imu_frame_fault(Roles),
            (board::BoardField::ImuAxisRole, 0),
            "a bad role pair names IMU_AXIS_ROLE"
        );
        assert_eq!(imu_frame_fault(Accel), (board::BoardField::ImuAxisSign, 0));
        assert_eq!(
            imu_frame_fault(Gyro),
            (board::BoardField::ImuAxisSign, 3),
            "the gyro triple starts at IMU_AXIS_SIGN index 3"
        );
    }

    /// **The re-read refusal is written in exactly two places and released in exactly one**, which
    /// is inside the end-of-engage guard (`specs/integration.md`: the motor-side level is folded
    /// before `control_task` consumes it, so a refusal cleared where it is set would leave
    /// `MoeGate` set with [`decide`] still returning [`ArmDecision::Arm`] and the board would
    /// re-read flash and run the whole shutdown sequence every 4 ms; and the guard carries the
    /// request half, without which the retry is every 20 ms rather than once per engage).
    ///
    /// The flags are target-only statics, so what is pinnable on the host is the SHAPE of the
    /// hardware half, in the spirit of the two source scans below. It enumerates EVERY use of each
    /// flag rather than searching for the one spelling a release happens to be written in today: a
    /// `swap(false, ..)`, a `fetch_and(..)` or a `store(off_request_clear, ..)` would all be
    /// releases a substring check for `.store(false` would miss. And it slices the guard's own
    /// block rather than asking whether the function contains both the guard and the release, which
    /// an empty guard followed by an unconditional release would satisfy.
    ///
    /// Comment lines are stripped first, so prose may discuss a clear that code may not, and the
    /// tokens are assembled from pieces so this test's own source does not contain them (it scans
    /// itself).
    #[test]
    fn the_re_read_refusal_is_released_once_inside_the_end_of_engage_guard() {
        let code: std::string::String = include_str!("arm.rs")
            .lines()
            .filter(|l| !l.trim_start().starts_with("//"))
            .collect::<std::vec::Vec<_>>()
            .join("\n");
        let re_read = concat!("RE_READ_", "REFUSED");
        let confirm = concat!("CONFIRM_", "REFUSED");

        // Every mention of each flag, with what follows it, so an unexpected accessor is a failure
        // rather than something the scan simply does not look for.
        let uses = |flag: &str| -> std::vec::Vec<std::string::String> {
            code.match_indices(flag)
                .filter(|(at, _)| !code[..*at].ends_with("static "))
                .map(|(at, _)| {
                    let tail = &code[at + flag.len()..];
                    tail.chars().take(14).collect::<std::string::String>()
                })
                .collect()
        };
        for (flag, releases) in [(re_read, 1usize), (confirm, 0)] {
            let mut loads = 0;
            let mut latches = 0;
            let mut clears = 0;
            for use_ in uses(flag) {
                if use_.starts_with(".load(") {
                    loads += 1;
                } else if use_.starts_with(".store(true") {
                    latches += 1;
                } else if use_.starts_with(".store(false") {
                    clears += 1;
                } else {
                    panic!(
                        "{flag} is used as `{use_}`: only a load, one `.store(true` and (for the \
                            retryable refusal) one `.store(false` are the sanctioned accesses"
                    );
                }
            }
            assert!(loads >= 1, "{flag} must be read into the fault level");
            assert_eq!(latches, 1, "{flag} is latched in exactly one place");
            assert_eq!(clears, releases, "{flag}'s release count");
        }

        // The one release sits INSIDE the guard's block, not merely in the same function: slice
        // from the guard to the brace that closes it.
        let guard = concat!("if off_", "request_clear {");
        let at = code
            .find(guard)
            .expect("the end-of-engage guard must exist");
        let body = &code[at..];
        let end = body
            .find("\n        }")
            .expect("the guard's block must close");
        assert!(
            body[..end].contains(&std::format!("{re_read}.store(false")),
            "the release must be inside the end-of-engage guard"
        );
        // And the arm sequence, which SETS the refusal, releases nothing.
        let arm_at = code
            .find(&std::format!("fn {}(", concat!("run_", "arm")))
            .expect("run_arm must exist");
        let arm_body = &code[arm_at..];
        let arm_end = arm_body
            .find("\n    }\n")
            .expect("run_arm's block must close");
        assert!(
            !arm_body[..arm_end].contains(".store(false"),
            "the arm sequence must not release a refusal on the tick that sets one"
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

    /// **The value row cannot silently grow.** Every registered field OUTSIDE the row is written to
    /// the same store, and the re-read comes back unchanged.
    ///
    /// Driven off `store::REGISTRY` rather than a handful of samples, so it covers what it claims
    /// and keeps covering it: a field added to the registry is in this test the moment it exists,
    /// and a field moved INTO the value row has to be moved in [`ROW_IDS`] below too, which is a
    /// deliberate edit beside the spec's table (`specs/integration.md`, "When a stored value takes
    /// effect: the arm-time re-read"). The row it excludes is the nine the decision names; what it
    /// therefore perturbs includes `motor.dead_time` (a live timer register), every pin assignment,
    /// the timer-side and injected-group fields, the two decode facts the arm path takes from
    /// `BootFixed`, `CONTROL_MODE`, `IMU_GYRO_BIAS` and the two `CONTROL_GAIN_*` values.
    #[test]
    fn a_bring_up_row_field_is_not_in_the_value_row() {
        let untouched = with_store(|_| {});
        let mut perturbed = 0usize;
        let v = with_store(|s| {
            for def in store::REGISTRY.iter() {
                if ROW_IDS.contains(&def.field_id) {
                    continue;
                }
                let key = store::Key {
                    field_id: def.field_id,
                    index: def.index,
                };
                // A value that differs from this field's default, so an absent write and a
                // perturbed one cannot look alike.
                let fresh = match def.default {
                    store::Value::U8(x) => store::Value::U8(x.wrapping_add(1)),
                    store::Value::U16(x) => store::Value::U16(x.wrapping_add(1)),
                    store::Value::U32(x) => store::Value::U32(x.wrapping_add(1)),
                    store::Value::U64(x) => store::Value::U64(x.wrapping_add(1)),
                    store::Value::I16(x) => store::Value::I16(x.wrapping_add(1)),
                    store::Value::I32(x) => store::Value::I32(x.wrapping_add(1)),
                    store::Value::I64(x) => store::Value::I64(x.wrapping_add(1)),
                    store::Value::Bool(x) => store::Value::Bool(!x),
                    store::Value::Str(_) => store::Value::Str(b"not-the-default"),
                    store::Value::Bytes(_) => store::Value::Bytes(&[0xA5, 0x5A]),
                };
                s.set_value(key, fresh).expect("the write must land");
                perturbed += 1;
            }
        });
        // The registry is the point of the test, so a parse or a filter that silently covered
        // nothing would be worse than no test.
        assert!(
            perturbed >= store::REGISTRY_LEN - 16,
            "only {perturbed} of {} registry entries were perturbed",
            store::REGISTRY_LEN
        );
        assert_eq!(
            v, untouched,
            "a write outside the value row must not change what the arm path re-reads"
        );
    }

    /// The nine field ids of the spec's value row, as [`read_arm_values`] reads them. The one place
    /// the row is written down as data, for the test above.
    const ROW_IDS: [u8; 9] = [
        0x21, // MOTOR_METHOD
        0x20, // MOTOR_CURRENT_LIMIT
        0x67, // MOTOR_CURRENT_CAL
        0x65, // IMU_AXIS_SIGN
        0x68, // IMU_AXIS_ROLE
        0x23, // CONTROL_RIDER_REQUIRED
        0x24, // CONTROL_BATTERY_FLOOR
        0x73, // CONTROL_DRIVE_LEAN
        0x74, // CONTROL_GAIN_MAX
    ];

    /// [`ROW_IDS`] is the row the reader actually reads: every id there comes from the store handle
    /// of the field the re-read names, so a renumbered field cannot leave the list stale.
    #[test]
    fn the_row_id_list_matches_the_fields_the_re_read_reads() {
        assert_eq!(
            ROW_IDS,
            [
                store::MOTOR_METHOD.id(),
                store::MOTOR_CURRENT_LIMIT.id(),
                store::MOTOR_CURRENT_CAL.id(),
                store::IMU_AXIS_SIGN.id(),
                store::IMU_AXIS_ROLE.id(),
                store::CONTROL_RIDER_REQUIRED.id(),
                store::CONTROL_BATTERY_FLOOR.id(),
                store::CONTROL_DRIVE_LEAN.at(0).id(),
                store::CONTROL_GAIN_MAX.at(0).id(),
            ]
        );
    }

    /// **Fail closed on a refused IMU frame.** A staged frame `imu::Config::staged` refuses makes
    /// the WHOLE re-derivation an `Err` (so the arm is refused and nothing is applied), while the
    /// same values with a valid frame re-derive.
    ///
    /// The `Err` carries the frame check that refused it, VERBATIM from `staged` rather than
    /// re-derived here, because that error is what becomes `CTRL_OBS` word 33's cause byte: a
    /// re-derivation that reported its own guess at the reason would let the published cause name a
    /// different field than the one that actually refused.
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
            let refused = imu::Config::staged(sign, [0; 3], roles)
                .expect_err("the fixture must be a frame `staged` refuses");
            assert_eq!(
                rederive(&values(sign, roles), BOOT, bias).err(),
                Some(refused),
                "a refused frame applies nothing, and reports the check that refused it"
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
    /// install reaches NO other field of the ISR's runtime record, and the honest check for that is
    /// a source scan of the install's body, in the spirit of the confinement test below.
    ///
    /// The forbidden names are read out of `MotorRuntime`'s own declaration rather than listed
    /// here, because a list of names is not the property: scanning for the three the install wants
    /// would pass an `m.periods = 0` or an `m.pwm` poke. Every field of the runtime except the three
    /// the install is FOR must be absent from its body, so a field added to the runtime is covered
    /// the moment it exists.
    #[test]
    fn the_arm_time_install_touches_only_the_value_row_of_the_isr_record() {
        let src = include_str!("motor.rs");
        // The install's body, comment lines stripped (so prose may discuss a field that code may
        // not name).
        let strip = |text: &str| -> std::string::String {
            text.lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .collect::<std::vec::Vec<_>>()
                .join("\n")
        };
        let block = |head: &str| -> std::string::String {
            let at = src
                .find(head)
                .unwrap_or_else(|| panic!("{head} must exist in motor.rs"));
            let rest = &src[at..];
            let end = rest
                .find("\n    }\n")
                .unwrap_or_else(|| panic!("{head}'s block must end"));
            strip(&rest[..end])
        };
        let body = block(&std::format!("pub fn {}", concat!("install_", "rederived")));

        // The ISR record's fields, parsed from the struct: `name: Type,` lines, comments dropped.
        let runtime = block(&std::format!("struct {} {{", concat!("Motor", "Runtime")));
        let fields: std::vec::Vec<&str> = runtime
            .lines()
            .filter_map(|l| l.trim().strip_suffix(','))
            .filter_map(|l| l.split_once(':'))
            .map(|(name, _)| name.trim())
            .filter(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'))
            .collect();
        assert!(
            fields.len() >= 10,
            "the runtime's fields did not parse: {fields:?}"
        );
        // The three the install is for, and therefore the only three it may name.
        let installed = ["method", "current", "commutator"];
        for f in installed {
            assert!(fields.contains(&f), "`{f}` is no longer a runtime field");
            assert!(body.contains(f), "the install must write `{f}`");
        }
        for f in &fields {
            if installed.contains(f) {
                continue;
            }
            assert!(
                !body.contains(f),
                "the arm-time install names the runtime's `{f}`: it installs the value row and \
                 nothing else (the measured phase offsets in particular are carried through)"
            );
        }
        // The two statics the bring-up's calibration publishes through, and the limit record's
        // CONSTRUCTOR: a built record would carry a zeroed trip count into the ISR, and that count
        // is boot-cumulative, so the install must go through
        // `motor::CurrentLimit::reconfigure` instead (that seam holds the split).
        for token in [
            concat!("FAU", "LT"),
            concat!("OBS_", "CAL"),
            concat!("CurrentLimit::", "new"),
        ] {
            assert!(
                !body.contains(token),
                "the arm-time install names `{token}`: it installs the value row and nothing else"
            );
        }
        // ...and each of the three goes in through its owning seam.
        for token in [concat!("re", "configure"), concat!("switch_", "method")] {
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
