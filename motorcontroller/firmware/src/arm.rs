//! Arming: the one place in the image that can energize a bridge (`specs/motor-integration.md`,
//! slice 5, "MOE enactment").
//!
//! `motorcontroller/firmware/src/motor.rs` is disarmed by construction and stays that way: it configures the
//! timer, runs the counter and steps the commutator, and it never names the arming gate. This file
//! is the complement. It holds the [`hal::ArmGate`] for the configured motor and it holds
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
    /// Re-read the arm row from the store and install it (`specs/config-read-once.md` section 5:
    /// the arm-row function, which the boot calls too). FIRST, before the counter is even
    /// confirmed, and it cannot refuse: the row is checked by the CLI and the app before it is
    /// written, so this step installs what is stored. It runs with the bridge DISARMED, because
    /// [`ArmStep::SetMoe`] is the last step, and with the config-write path IDLE, because the link
    /// drains that reach `store.set_value` and the task dispatch that reaches this step run on the
    /// same thread.
    ReReadValues,
    /// Confirm the period ISR is actually being served, by watching [`motor::PERIODS`] advance.
    /// **Load-bearing, and the reason arming is not simply a register write**: MOE is the only
    /// thing standing between a stopped commutator and a bridge holding whatever the compare units
    /// last held, so the bridge is energized only after the thing that will step it is proven
    /// alive. It watches the ISR's own liveness word rather than the injected end-of-conversion
    /// flag the BRING-UP confirm polls, because here the vector is already unmasked: the live ISR
    /// clears that flag itself, so a poll of it would race the ISR and could refuse a healthy arm.
    ///
    /// The counter it watches is the BRING-UP's: started there, stopped nowhere, because the
    /// hall-edge measurement the OFF interlock reads has to stay live while the board is disarmed
    /// (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed"). So this step
    /// starts nothing, it only proves: the same liveness proof, taken immediately before the one
    /// energize act, on the first arm of a boot and on every later one. A counter that somehow is
    /// not turning refuses the arm here rather than being restarted into it.
    ConfirmPeriodsLive,
    /// Zero the demand word, so the duties the first armed period writes are derived from a zero
    /// demand. The mode machine already guarantees this (torque is produced only in RUN, and MOE
    /// rises at INIT, two passes earlier), but the energize point should not depend on a property
    /// of a different layer. MOE rises onto a FLOATED bridge (the ISR holds all three phases
    /// floated while it is disarmed, [`motor::coast_posture`]) and the first period after it writes
    /// the duties before it enables a channel, the write-order rule, so a compare value left over
    /// from the previous run is never driven.
    ZeroDemand,
    /// Set MOE. **The one energize act in the image**, and the last step, so every precondition
    /// above it has already passed.
    SetMoe,
}

/// The arm sequence, in order.
pub const ARM_STEPS: [ArmStep; 4] = [
    ArmStep::ReReadValues,
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
    /// LAST, and it is the posture the board then keeps: the period ISR holds all-float for as
    /// long as the bridge is disarmed ([`motor::coast_posture`]), so this step states the posture
    /// at the instant of the shutdown and the ISR maintains it afterwards.
    FloatChannels,
}

/// The shutdown sequence, in order.
///
/// **The counter is not in it**, and that is the fix for a measured defect rather than an
/// omission (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed";
/// `specs/bench-evidence/2026-10-20/sine-phase/RECORD.md`). Stopping it ended the period ISR,
/// which is the only writer of [`motor::SPEED`], so a run that ended with the wheel still turning
/// froze that word nonzero; [`off_inhibit`] then held the mode machine in OFF for the rest of the
/// boot and the board refused every later arm with no fault and no indication. The counter is the
/// hall-edge measurement's clock, started once by the bring-up and never stopped, so the interlock
/// reads a wheel that is turning NOW. What the shutdown owns is the energizing path, which is MOE,
/// the demand word and the channel enables.
///
/// Keeping the clock running is what makes the interlock answerable; it is not what makes the
/// answer safe when the clock stops anyway. A vector this sequence does not own can still wedge,
/// and [`off_inhibit`] reads the liveness observation beside the speed word for exactly that case.
pub const SHUTDOWN_STEPS: [ShutdownStep; 3] = [
    ShutdownStep::Disarm,
    ShutdownStep::ZeroDemand,
    ShutdownStep::FloatChannels,
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
/// - `fault_level`: the motor-side fault LEVEL, the union of [`motor::motor_fault_level`]'s causes, never the raw
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
/// True while the wheel is turning, and true while nothing can say whether it is. It cannot block a
/// shutdown: the mode machine consults `off_inhibit` in OFF only.
///
/// **`speed`** is the period ISR's raw signed hall-edge count over its 320-period (20 ms) window,
/// so ANY net edge in the window is motion and the threshold is 1: the predicate holds the machine
/// in OFF, i.e. it refuses to ENGAGE a vehicle whose wheel is already moving, and being conservative
/// there costs a rider one more button press while being permissive costs an engage under a rolling
/// wheel. A stationary wheel reads exactly 0 (hall codes do not change, so the commutator counts no
/// edges), and a bouncing hall line at rest contributes alternating signs that net toward 0 rather
/// than accumulating.
///
/// **`period_live`** is this tick's period-liveness observation ([`motor::periods_live`]), and it is
/// what makes that word a MEASUREMENT rather than a remembered one (`specs/motor-integration.md`,
/// "The OFF interlock must read a LIVE speed"). The ISR is the sole writer of [`motor::SPEED`], and
/// a word it is not writing says what the wheel was doing when the last window closed, which is a
/// different claim and one nothing retracts. Both ways of being wrong are reachable from there: a
/// wheel that has since stopped reads as moving, which deadlocks the board in OFF, and a wheel that
/// has since started rolling reads as stationary, which would let the bridge energize into it. The
/// second is the hazard, so a shortfall inhibits outright and the speed word is not consulted at
/// all. Unknown is not stopped.
///
/// The observation and not the hysteresis latch, deliberately: `PeriodHealth` waits five ticks
/// because a FAULT must not be manufactured out of scheduling jitter, while this predicate only has
/// to refuse to trust one sample, and refusing a sample costs one 4 ms tick of not arming. A board
/// held here is not held silently either, because a shortfall that lasts is also what raises the
/// liveness level in [`motor::motor_fault_level`].
///
/// **`motor_running`** is the premise both of the above rest on, `motor::obs_configured` AND
/// [`motor::COUNTER_RUNNING`], the same pair [`motor::PeriodHealth::update`] takes and for the same
/// reason: a board that brought no motor up has no wheel to be rolling and a period counter that was
/// never going to advance, so it must not be inhibited by a liveness it could never have had. Its
/// [`motor::SPEED`] has no writer and sits at the cell's initial 0.
#[inline]
pub fn off_inhibit(motor_running: bool, period_live: bool, speed: i32) -> bool {
    motor_running && (!period_live || speed != 0)
}

// -------------------------------------------------------------------------------------------
// The arm-time re-read (pure; `specs/integration.md`, "When a stored value takes effect: the
// arm-time re-read")
// -------------------------------------------------------------------------------------------

/// The VALUE ROW, as read from flash: the fields a disarmed `CONFIG_WRITE` can change that
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
/// Two more fields are absent for their own reasons, not because they are peripheral:
/// `IMU_GYRO_BIAS` is carried through from the IMU's installed config rather than re-read (see
/// [`rederive`]), and `CONTROL_SF_GAINS` and `CONTROL_REFERENCE` are already live through the
/// tune lane (`specs/state-feedback.md` section 6, with `reconcile_gains` handling their persist
/// path). Only the gains' MAXIMA are here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArmValues {
    /// `store::MOTOR_METHOD` (0x23, `machine.method`).
    pub method_byte: u8,
    /// `store::MOTOR_DIRECTION` (0x4F) at motor 0, raw: `motor::requested_direction` decodes it.
    /// In the row since 2026-10-20, when the bench showed the boot-read it replaced: writing the
    /// field and arming changed nothing, writing it and resetting reversed the wheel
    /// (`specs/bench-evidence/2026-10-20/sine-phase/RECORD.md`, the direction gate).
    pub direction_byte: u8,
    /// `store::MOTOR_ALIGN_OFFSET` (0x51) at motor 0, raw: `commutation::sixstep::SixStep::new`
    /// takes it mod 6. In the row with the direction (the row's test is whether a field touches a
    /// peripheral register, and the offset enters one argument along from the direction in the
    /// same decode); as a boot-read, sweeping it cost a reboot per value.
    pub align_offset_byte: u8,
    /// `store::CONTROL_MODE` (0x20, `machine.mode`) raw: the owning type decodes it and judges it against the
    /// board's IMU fact, which this layer does not hold.
    pub control_mode_byte: u8,
    /// `store::CONTROL_FRAME` (0x21, `machine.frame`) raw: resolved beside the mode byte by the same owner
    /// (`specs/rover-one-imu.md` section 5: in the row so staging a frame costs a re-arm, not a
    /// power cycle).
    pub frame_byte: u8,
    /// `store::CONTROL_SIDE` (0x50, `side`) at motor 0, raw: which wheel of a rigid machine this
    /// board's motor drives, the
    /// owner of the control block's `role_right` word (`specs/rover-one-imu.md` section 5). In
    /// the row beside the frame, and for the same reason: it is a machine-type setting a phone
    /// stages, and staging it should cost a re-arm rather than a power cycle.
    pub side_byte: u8,
    /// `store::MOTOR_CURRENT_LIMIT` (0x4E) at motor 0, milliamps.
    pub current_limit_ma: u32,
    /// `store::MOTOR_CURRENT_CAL` (0x4C) at motor 0, stock current counts per amp.
    pub current_cal: u16,
    /// `store::IMU_AXIS_SIGN` (0x15), indices 0..2 = `[x, y, z]`, applied to the accelerometer
    /// and the gyro alike (`imu::Config::staged`).
    pub imu_sign: [i32; 3],
    /// `store::IMU_AXIS_ROLE` (0x16), indices 0..1 = `[UP, PITCH_RATE]`.
    pub imu_roles: [u8; 2],
    /// `store::CONTROL_RIDER_REQUIRED` (0x22), raw: the owning type decodes it.
    pub rider_required_byte: u8,
    /// `store::BOARD_BATTERY_FLOOR` (0x26), centivolts, `<= 0` = no floor.
    pub battery_floor: i16,
    /// `store::CONTROL_GAIN_MAX` (0x28), indices 0..3 = the per-gain inclusive upper bounds of
    /// `[gp, gr, gx, gv]`.
    pub gain_max: [i16; 4],
    /// `store::CONTROL_DRIVE_TURN` (0x2B), indices 0..1 = `[turn_max, turn_slew]`, torque counts.
    pub drive_turn: [i16; 2],
    /// `store::CONTROL_POS_CLAMP` (0x2A), edges, raw: the owning seam (`control::sf::pos_clamp`)
    /// clamps it (`specs/state-feedback.md` section 4).
    pub pos_clamp: i16,
}

/// Read the whole value row, the ONE read site.
///
/// Generic over the flash seam so the host tests drive it against a real mounted `Store`. Every
/// read is infallible (`Store::get` returns the registered default for a missing record), so this
/// cannot fail and a virgin board re-derives exactly the defaults the boot path used.
///
/// The firmware brings up motor 0, so `MOTOR_DIRECTION`, `MOTOR_ALIGN_OFFSET` and
/// `MOTOR_CURRENT_CAL` are read at index 0, matching `layout::read_motor`'s own per-motor `.at(m)`
/// fill.
/// Every key [`read_arm_values`] resolves, as ONE row: exactly the registered keys whose effect is
/// arm (`store::Effect::Arm`, `specs/machine-settings.md` section 5), which a test holds to the
/// registry, in registry order. A per-motor field is read at motor 0, the one motor a board drives.
///
/// The row exists because of what a per-field read costs. `Store::get` walks the whole active
/// page for each field (append-only, newest wins, so it cannot stop at the first match), and read
/// a field at a time this row walked the page twenty-one times per arm, CRC-checking every record
/// on every walk. Measured on the offroad master 2026-10-22 at a 67%-full page: 23-36 ms of
/// blocked main loop per arm, which overran both RX rings and tripped the peer-staleness
/// supervision on both boards about once a second (`specs/integration.md`, "What the row costs").
///
/// **Adding a field here costs one key and one match arm, not one page walk.** That is the point
/// of the row, and it is why the list is written out rather than assembled somewhere clever: the
/// next person moving a value into the arm-time read should see exactly what it contains.
pub const ARM_ROW: [store::Key; 22] = [
    store::IMU_AXIS_SIGN.at(0).key(),
    store::IMU_AXIS_SIGN.at(1).key(),
    store::IMU_AXIS_SIGN.at(2).key(),
    store::IMU_AXIS_ROLE.at(0).key(),
    store::IMU_AXIS_ROLE.at(1).key(),
    store::CONTROL_MODE.key(),
    store::CONTROL_FRAME.key(),
    store::CONTROL_RIDER_REQUIRED.key(),
    store::MOTOR_METHOD.key(),
    store::BOARD_BATTERY_FLOOR.key(),
    store::CONTROL_GAIN_MAX.at(0).key(),
    store::CONTROL_GAIN_MAX.at(1).key(),
    store::CONTROL_GAIN_MAX.at(2).key(),
    store::CONTROL_GAIN_MAX.at(3).key(),
    store::CONTROL_POS_CLAMP.key(),
    store::CONTROL_DRIVE_TURN.at(0).key(),
    store::CONTROL_DRIVE_TURN.at(1).key(),
    store::MOTOR_CURRENT_CAL.at(0).key(),
    store::MOTOR_CURRENT_LIMIT.at(0).key(),
    store::MOTOR_DIRECTION.at(0).key(),
    store::CONTROL_SIDE.at(0).key(),
    store::MOTOR_ALIGN_OFFSET.at(0).key(),
];

pub fn read_arm_values<F: store::Flash>(s: &store::Store<F>) -> ArmValues {
    // Start from the registered defaults and let the pass overwrite what the store carries: a
    // field with no record then reads exactly what `Store::get` would have returned for it.
    let mut v = ArmValues {
        method_byte: store::MOTOR_METHOD.default(),
        direction_byte: store::MOTOR_DIRECTION.at(0).default(),
        align_offset_byte: store::MOTOR_ALIGN_OFFSET.at(0).default(),
        control_mode_byte: store::CONTROL_MODE.default(),
        frame_byte: store::CONTROL_FRAME.default(),
        side_byte: store::CONTROL_SIDE.at(0).default(),
        current_limit_ma: store::MOTOR_CURRENT_LIMIT.at(0).default(),
        current_cal: store::MOTOR_CURRENT_CAL.at(0).default(),
        imu_sign: [
            store::IMU_AXIS_SIGN.at(0).default(),
            store::IMU_AXIS_SIGN.at(1).default(),
            store::IMU_AXIS_SIGN.at(2).default(),
        ],
        imu_roles: [
            store::IMU_AXIS_ROLE.at(0).default(),
            store::IMU_AXIS_ROLE.at(1).default(),
        ],
        rider_required_byte: store::CONTROL_RIDER_REQUIRED.default(),
        battery_floor: store::BOARD_BATTERY_FLOOR.default(),
        gain_max: [
            store::CONTROL_GAIN_MAX.at(0).default(),
            store::CONTROL_GAIN_MAX.at(1).default(),
            store::CONTROL_GAIN_MAX.at(2).default(),
            store::CONTROL_GAIN_MAX.at(3).default(),
        ],
        drive_turn: [
            store::CONTROL_DRIVE_TURN.at(0).default(),
            store::CONTROL_DRIVE_TURN.at(1).default(),
        ],
        pos_clamp: store::CONTROL_POS_CLAMP.default(),
    };
    // One pass. Records arrive oldest first, so a later one simply overwrites: newest wins, the
    // same rule `Store::get` applies, reached by assignment instead of by a second walk. Each arm
    // matches on the SAME handle it decodes with, so a key and its destination cannot drift apart.
    s.for_each_in(&ARM_ROW, |k, tag, b| {
        if k == store::MOTOR_METHOD.key() {
            v.method_byte = store::scalar_or_default(store::MOTOR_METHOD, tag, b);
        } else if k == store::MOTOR_DIRECTION.at(0).key() {
            v.direction_byte = store::scalar_or_default(store::MOTOR_DIRECTION.at(0), tag, b);
        } else if k == store::MOTOR_ALIGN_OFFSET.at(0).key() {
            v.align_offset_byte = store::scalar_or_default(store::MOTOR_ALIGN_OFFSET.at(0), tag, b);
        } else if k == store::CONTROL_MODE.key() {
            v.control_mode_byte = store::scalar_or_default(store::CONTROL_MODE, tag, b);
        } else if k == store::CONTROL_FRAME.key() {
            v.frame_byte = store::scalar_or_default(store::CONTROL_FRAME, tag, b);
        } else if k == store::CONTROL_SIDE.at(0).key() {
            v.side_byte = store::scalar_or_default(store::CONTROL_SIDE.at(0), tag, b);
        } else if k == store::MOTOR_CURRENT_LIMIT.at(0).key() {
            v.current_limit_ma = store::scalar_or_default(store::MOTOR_CURRENT_LIMIT.at(0), tag, b);
        } else if k == store::MOTOR_CURRENT_CAL.at(0).key() {
            v.current_cal = store::scalar_or_default(store::MOTOR_CURRENT_CAL.at(0), tag, b);
        } else if k == store::CONTROL_RIDER_REQUIRED.key() {
            v.rider_required_byte = store::scalar_or_default(store::CONTROL_RIDER_REQUIRED, tag, b);
        } else if k == store::BOARD_BATTERY_FLOOR.key() {
            v.battery_floor = store::scalar_or_default(store::BOARD_BATTERY_FLOOR, tag, b);
        } else if k == store::CONTROL_POS_CLAMP.key() {
            v.pos_clamp = store::scalar_or_default(store::CONTROL_POS_CLAMP, tag, b);
        } else if k.field_id == store::IMU_AXIS_SIGN.id() {
            if let Some(slot) = v.imu_sign.get_mut(k.index as usize) {
                *slot = store::scalar_or_default(store::IMU_AXIS_SIGN.at(k.index), tag, b);
            }
        } else if k.field_id == store::IMU_AXIS_ROLE.id() {
            if let Some(slot) = v.imu_roles.get_mut(k.index as usize) {
                *slot = store::scalar_or_default(store::IMU_AXIS_ROLE.at(k.index), tag, b);
            }
        } else if k.field_id == store::CONTROL_GAIN_MAX.id() {
            if let Some(slot) = v.gain_max.get_mut(k.index as usize) {
                *slot = store::scalar_or_default(store::CONTROL_GAIN_MAX.at(k.index), tag, b);
            }
        } else if k.field_id == store::CONTROL_DRIVE_TURN.id() {
            if let Some(slot) = v.drive_turn.get_mut(k.index as usize) {
                *slot = store::scalar_or_default(store::CONTROL_DRIVE_TURN.at(k.index), tag, b);
            }
        }
    });
    v
}

/// Everything the arm row installs, converted and ready to apply.
#[derive(Clone, Copy, Debug)]
pub struct Rederived {
    /// The period ISR's value set (method, current limit, fresh records, the method fault).
    pub motor: motor::Rederived,
    /// The IMU's axis frame, or `None` on a board with no IMU (nothing to install).
    pub imu: Option<imu::Config>,
    /// `CONTROL_MODE` raw, for the orchestrator's control section to decode, judge against the
    /// board's IMU fact and install (`orchestrator::re_apply_control_values`): a mode that cannot
    /// run is DEMOTED with the mode fault, which is a runtime fact, not a configuration check.
    pub control_mode_byte: u8,
    /// `CONTROL_FRAME` raw, resolved with the mode byte by the same owner.
    pub frame_byte: u8,
    /// `CONTROL_SIDE` raw, decoded by the control block's owner into `role_right`.
    pub side_byte: u8,
    /// `CONTROL_RIDER_REQUIRED` raw, for `control::ControlDispatch` to decode by its own rule.
    pub rider_required_byte: u8,
    /// `BOARD_BATTERY_FLOOR` as written (the dispatch takes the word, `<= 0` = no floor).
    pub battery_floor: i16,
    /// The turn bound and rate, through `control::DriveTurn::new`'s seam clamp.
    pub drive_turn: control::DriveTurn,
    /// The gain maxima as read; `control::GainShadow` applies its own non-negative floor.
    pub gain_max: [i16; 4],
    /// `CONTROL_POS_CLAMP` as read; the control section's owner clamps it through
    /// `control::sf::pos_clamp`.
    pub pos_clamp: i16,
}

/// Convert the arm row into what is installed: the motor's records and limit through
/// [`motor::rederive`], the IMU frame through `imu::Config::staged`'s unset rule. Nothing is
/// refused (`specs/config-read-once.md` section 5): the CLI and the app check the row before they
/// write it.
///
/// `imu_bias` is the IMU's INSTALLED gyro bias, carried through because `IMU_GYRO_BIAS` is a boot
/// calibration and not in the row: `Some(bias)` on a board whose IMU was brought up, `None` on a
/// board with no IMU, whose `imu` is then `None`.
pub fn rederive(
    values: &ArmValues,
    boot: motor::BootFixed,
    imu_bias: Option<[i32; 3]>,
) -> Rederived {
    Rederived {
        motor: motor::rederive(
            values.method_byte,
            values.direction_byte,
            values.align_offset_byte,
            values.current_limit_ma,
            values.current_cal,
            boot,
        ),
        imu: imu_bias.map(|bias| imu::Config::staged(values.imu_sign, bias, values.imu_roles)),
        control_mode_byte: values.control_mode_byte,
        frame_byte: values.frame_byte,
        side_byte: values.side_byte,
        rider_required_byte: values.rider_required_byte,
        battery_floor: values.battery_floor,
        // The seam clamp, applied by the type that owns it (`turn_max` 0..28500, `turn_slew`
        // 1..1000).
        drive_turn: control::DriveTurn::new(values.drive_turn[0], values.drive_turn[1]),
        gain_max: values.gain_max,
        pos_clamp: values.pos_clamp,
    }
}

/// The [`ArmStep::ConfirmPeriodsLive`] spin budget, in poll iterations. One 16 kHz period is 62.5 us
/// (4500 cycles at 72 MHz), so a healthy motor satisfies the confirm within ~1100 iterations of a
/// ~4-cycle atomic-load loop; the budget is ~25x that, and it is paid in full only by a board whose
/// period ISR has stopped, once, on the tick that tried to arm.
pub const ARM_CONFIRM_SPINS: u32 = 30_000;

// -------------------------------------------------------------------------------------------
// The refusal observation (pure; `specs/integration.md`, "The arm refusals (word 33, permanent)")
// -------------------------------------------------------------------------------------------

/// Byte lane of the [`ArmStep::ConfirmPeriodsLive`] refusal count in the published word. Lane 0
/// counted the re-read's refusals and lane 16 the refused IMU frame's cause, until the re-read
/// stopped refusing (`specs/config-read-once.md` section 5); both read 0.
const CONFIRM_COUNT: u32 = 8;

/// Fold one [`ArmStep::ConfirmPeriodsLive`] refusal into the published refusal word (`CTRL_OBS`
/// word 33): a saturating per-boot count, because the question a bench read asks of it is "once,
/// or every attempt", and a count that wrapped to 0 answers it wrongly. The refusal itself is
/// boot-sticky and readable as a level; the level says only THAT it is held, never how many
/// attempts it ate.
pub const fn note_confirm_refusal(word: u32) -> u32 {
    let n = ((word >> CONFIRM_COUNT) & 0xFF) as u8;
    (word & !(0xFF << CONFIRM_COUNT)) | ((n.saturating_add(1) as u32) << CONFIRM_COUNT)
}

// -------------------------------------------------------------------------------------------
// The hardware half
// -------------------------------------------------------------------------------------------

#[cfg(target_os = "none")]
pub mod hw {
    use super::{
        decide, note_confirm_refusal, ArmDecision, ArmStep, ShutdownStep, ARM_CONFIRM_SPINS,
        ARM_STEPS, SHUTDOWN_STEPS,
    };
    use crate::motor;
    use core::ptr::addr_of_mut;
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use hal::ArmGate;

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

    /// The refusal observation (`CTRL_OBS` word 33), maintained in the published packing by
    /// [`super::note_confirm_refusal`]: a saturating per-boot count of the confirm's refusals. One
    /// writer (the 250 Hz thread, inside [`run_arm`]).
    static REFUSAL_OBS: AtomicU32 = AtomicU32::new(0);

    /// Install the arming gate for a brought-up motor. Called once, on the boot thread, from the
    /// bring-up's summary. Installing it does not arm anything: MOE is untouched here, and the
    /// mode machine is in OFF for the whole of boot.
    pub fn install(timer: &hal::PwmTimer) {
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
    #[inline]
    pub fn refused() -> bool {
        CONFIRM_REFUSED.load(Ordering::Relaxed)
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
    /// `re_read` is the arm-row function ([`ArmStep::ReReadValues`]), called only on the ARM path:
    /// a closure because it reaches the caller's shell and the store static.
    pub fn enact(
        moe_allowed: bool,
        brought_up: bool,
        fault_level: bool,
        mut re_read: impl FnMut(),
    ) {
        match decide(moe_allowed, armed(), brought_up, fault_level) {
            ArmDecision::Idle => {}
            ArmDecision::Arm => run_arm(&mut re_read),
            ArmDecision::Shutdown => run_shutdown(),
        }
    }

    /// [`ARM_STEPS`], in order. A refused confirm aborts BEFORE the MOE step and runs the shutdown
    /// sequence instead, so the failure path leaves the bridge in the disarmed, all-float posture
    /// rather than half-way through an arm. It is held as the level [`motor::motor_fault_level`]
    /// folds, sticky for the boot, and steps its count in `REFUSAL_OBS`.
    fn run_arm(re_read: &mut impl FnMut()) {
        for step in ARM_STEPS {
            match step {
                ArmStep::ReReadValues => re_read(),
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
            let level = motor_fault_level(true, word, false, false) != 0;
            assert!(!level, "word {word:#x} is not a fault producer");
            assert_eq!(
                decide(true, false, true, level),
                ArmDecision::Arm,
                "word {word:#x} must not block arming"
            );
        }
        // ...and the producers do block it, through the same level.
        for word in [FAULT_HALL, FAULT_INIT_CAL] {
            let level = motor_fault_level(true, word, false, false) != 0;
            assert!(level);
            assert_eq!(decide(true, false, true, level), ArmDecision::Idle);
        }
        // As does a refused arm while it is held, so a board that failed to arm cannot sit in RUN
        // with an unarmed bridge, nor retry inside the same engage.
        assert_ne!(motor_fault_level(true, 0, false, true), 0);
        assert_eq!(
            decide(
                true,
                false,
                true,
                motor_fault_level(true, 0, false, true) != 0
            ),
            ArmDecision::Idle
        );
    }

    /// **The refusal is a LEVEL, so holding it refuses the arm and releasing it permits one**
    /// (`specs/integration.md`, "A refused re-read refuses the ARM, not the boot"): the retry the
    /// OFF-pass clear buys is an arm ATTEMPT, which means the fold and the gate have to answer
    /// differently either side of that clear, through the level's single owner.
    ///
    /// Both refusals are one `bool` here because `motor_fault_level` takes one and the arm layer
    /// hands it the OR of its two statics (one cause bit, `EV_MOTOR_ARM`): what differs between
    /// them is how long each is held, not what the level means downstream.
    #[test]
    fn a_held_refusal_refuses_the_arm_and_a_released_one_permits_the_next() {
        let held = motor_fault_level(true, 0, false, true) != 0;
        assert!(held, "either refusal reaches the level while held");
        assert_eq!(
            decide(true, false, true, held),
            ArmDecision::Idle,
            "no arm attempt while the refusal is held"
        );
        // And the shutdown the mode machine runs on the tick that reads the held level is what
        // takes the pass to OFF, where the re-read refusal is released.
        assert_eq!(decide(true, true, true, held), ArmDecision::Shutdown);
        let released = motor_fault_level(true, 0, false, false) != 0;
        assert!(!released);
        assert_eq!(
            decide(true, false, true, released),
            ArmDecision::Arm,
            "the next engage after the release re-reads rather than waiting for a reboot"
        );
    }

    /// The published refusal word (`CTRL_OBS` word 33): the confirm's count in its lane, saturating
    /// rather than wrapping, every other lane zero.
    #[test]
    fn the_refusal_word_counts_the_confirm_in_its_lane() {
        assert_eq!(note_confirm_refusal(0), 0x0000_0100);
        let mut w = 0x0000_FE00u32;
        for _ in 0..3 {
            w = note_confirm_refusal(w);
        }
        assert_eq!(w, 0x0000_FF00, "the count saturates, it does not wrap to 0");
    }

    /// **The confirm refusal is latched in one place and never released**: it is a measurement of
    /// a wedged vector, which no write heals, so it is boot-sticky. The flag is a target-only
    /// static, so what is pinnable on the host is the shape of the hardware half: every use of it
    /// is a load or the one latch. Comment lines are stripped first, and the token is assembled so
    /// this test's own source does not contain it.
    #[test]
    fn the_confirm_refusal_is_latched_once_and_never_released() {
        let code: std::string::String = include_str!("arm.rs")
            .lines()
            .filter(|l| !l.trim_start().starts_with("//"))
            .collect::<std::vec::Vec<_>>()
            .join("\n");
        let flag = concat!("CONFIRM_", "REFUSED");
        let (mut loads, mut latches) = (0, 0);
        for (at, _) in code.match_indices(flag) {
            if code[..at].ends_with("static ") {
                continue;
            }
            let tail: std::string::String = code[at + flag.len()..].chars().take(12).collect();
            if tail.starts_with(".load(") {
                loads += 1;
            } else if tail.starts_with(".store(true") {
                latches += 1;
            } else {
                panic!("{flag} is used as `{tail}`: only a load and one latch are sanctioned");
            }
        }
        assert!(loads >= 1, "{flag} must be read into the fault level");
        assert_eq!(latches, 1, "{flag} is latched in exactly one place");
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
        assert!(idx(ArmStep::ReReadValues) < idx(ArmStep::ConfirmPeriodsLive));
        assert_eq!(
            ARM_STEPS
                .iter()
                .filter(|s| **s == ArmStep::ReReadValues)
                .count(),
            1,
            "one re-read, not two"
        );
        assert_eq!(*ARM_STEPS.last().unwrap(), ArmStep::SetMoe);
        assert!(idx(ArmStep::ConfirmPeriodsLive) < idx(ArmStep::SetMoe));
        assert!(idx(ArmStep::ZeroDemand) < idx(ArmStep::SetMoe));
        assert_eq!(
            ARM_STEPS.iter().filter(|s| **s == ArmStep::SetMoe).count(),
            1,
            "one energize act, not two"
        );
    }

    /// **`disarm-before-shutdown-steps`** (`specs/motor-integration.md`, Validation): MOE is
    /// cleared FIRST, before the demand zeroing and the float, so the silencing act never waits on
    /// a step that could fail.
    #[test]
    fn shutdown_step_order_clears_moe_first() {
        assert_eq!(*SHUTDOWN_STEPS.first().unwrap(), ShutdownStep::Disarm);
        let idx = |s: ShutdownStep| SHUTDOWN_STEPS.iter().position(|x| *x == s).unwrap();
        assert!(idx(ShutdownStep::Disarm) < idx(ShutdownStep::ZeroDemand));
        assert!(idx(ShutdownStep::ZeroDemand) < idx(ShutdownStep::FloatChannels));
        for s in SHUTDOWN_STEPS {
            assert_eq!(SHUTDOWN_STEPS.iter().filter(|x| **x == s).count(), 1);
        }
    }

    /// The shutdown list inverts the arm list: every arm step that can make the bridge LIVE has
    /// its undo, and the two independent silencing paths (MOE and the demand word) are both
    /// present and separate.
    ///
    /// The two lists are not the same LENGTH, and that is the honest shape rather than a gap. Two
    /// arm steps have no undo and need none: [`ArmStep::ReReadValues`] installs VALUES (a method
    /// byte, a current limit, RAM records), with no previous value to restore to and nothing it
    /// wrote that can energize anything, and [`ArmStep::ConfirmPeriodsLive`] only MEASURES. One
    /// shutdown step has no arm counterpart: [`ShutdownStep::FloatChannels`] states a posture the
    /// period ISR then holds, and no arm step ever set it.
    #[test]
    fn the_shutdown_list_inverts_the_arm_list() {
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::Disarm)); // undoes SetMoe
        assert!(SHUTDOWN_STEPS.contains(&ShutdownStep::ZeroDemand)); // and ArmStep::ZeroDemand
                                                                     // The arm steps that change what the bridge does, each with its undo above.
        let energizing: std::vec::Vec<ArmStep> = ARM_STEPS
            .into_iter()
            .filter(|s| !matches!(s, ArmStep::ReReadValues | ArmStep::ConfirmPeriodsLive))
            .collect();
        assert_eq!(
            energizing,
            [ArmStep::ZeroDemand, ArmStep::SetMoe],
            "the energizing steps, and only those"
        );
        assert_eq!(
            SHUTDOWN_STEPS.len(),
            energizing.len() + 1,
            "every energizing arm step has its undo, plus the float posture"
        );
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

    /// OFF-inhibit from the raw speed word, with the period ISR running: any net edge in the
    /// window is motion.
    #[test]
    fn off_inhibit_follows_wheel_motion() {
        assert!(!off_inhibit(true, true, 0), "a stationary wheel");
        assert!(off_inhibit(true, true, 1));
        assert!(off_inhibit(true, true, -1), "either direction");
        assert!(off_inhibit(true, true, i32::MIN));
        assert!(off_inhibit(true, true, i32::MAX));
    }

    /// **A speed nothing is measuring inhibits, whatever it last said**
    /// (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed"). The hazard the
    /// zero case covers is the one the deadlock case hides: a board parked at `SPEED` 0 whose
    /// period vector then wedges would read "stationary" for a wheel that has since been pushed
    /// into motion, and arming energizes the bridge into it.
    #[test]
    fn a_stopped_period_counter_inhibits_whatever_the_speed_word_says() {
        assert!(off_inhibit(true, false, 0), "a stale zero is not a stop");
        assert!(off_inhibit(true, false, -17), "the reported frozen word");
        assert!(off_inhibit(true, false, i32::MAX));
    }

    /// A board that brought no motor up is never inhibited: its counter was never going to advance,
    /// so a liveness shortfall there is the expected state and not an unknown speed
    /// ([`motor::PeriodHealth::update`] takes the same premise and resets to healthy on it).
    #[test]
    fn a_board_with_no_motor_is_never_inhibited() {
        assert!(!off_inhibit(false, false, 0));
        assert!(
            !off_inhibit(false, false, -17),
            "no writer, so the cell cannot hold this; the premise decides regardless"
        );
    }

    /// **The transition, through the real mode machine**, in both directions across a vector that
    /// stops and comes back.
    ///
    /// Forward: the wheel is rolling when the ISR stops, so the word freezes nonzero and the
    /// machine is held. That hold is right while the wheel rolls and becomes wrong the moment it
    /// stops, which nothing on the board can see; the board recovers on its own once the
    /// measurement does, with no dwell timer and no reboot, because the first window the resumed
    /// ISR closes latches the wheel's actual 0.
    ///
    /// Backward, and this is the leg the old unconditional read got wrong: the board is parked at
    /// `SPEED` 0 when the ISR stops, the machine is then pushed, and the word still says 0. Reading
    /// it would let the bridge energize into a rolling wheel.
    #[test]
    fn the_machine_leaves_off_when_the_measurement_comes_back_and_not_before() {
        use state::{Mode, ModeInputs, ModeMachine};

        fn press(off_inhibit: bool) -> Mode {
            let mut m: ModeMachine<1> = ModeMachine::new();
            m.tick(&ModeInputs {
                power_request: true,
                off_inhibit,
                ..ModeInputs::default()
            })
            .mode
        }

        // Rolling, measured: held, correctly.
        assert_eq!(press(off_inhibit(true, true, -17)), Mode::Off);
        // The vector wedges. The word is frozen at -17 and the wheel coasts to rest behind it.
        assert_eq!(
            press(off_inhibit(true, false, -17)),
            Mode::Off,
            "a frozen word must not be read as a measurement"
        );
        // The ISR resumes. Its first closed window counts the edges of a wheel at rest: 0.
        assert_eq!(press(off_inhibit(true, true, 0)), Mode::Init);

        // The other direction, from a board parked at rest.
        assert_eq!(press(off_inhibit(true, true, 0)), Mode::Init, "at rest");
        // The vector wedges here instead, and the machine is pushed. The word is the only thing
        // that could report that, and its writer is the ISR that is not running.
        assert_eq!(
            press(off_inhibit(true, false, 0)),
            Mode::Off,
            "a stale zero would arm the bridge into a wheel that is rolling"
        );
        // It comes back, and now the word is a measurement of a rolling wheel.
        assert_eq!(press(off_inhibit(true, true, 6)), Mode::Off);
        // Pushed no longer, measured: the machine arms.
        assert_eq!(press(off_inhibit(true, true, 0)), Mode::Init);
    }

    /// **A hold on an unmeasurable speed is never a silent one.** The same shortfall that makes the
    /// speed unknown is what raises the period-liveness level once it is sustained
    /// ([`motor::PeriodHealth`], five consecutive ticks), so a board held here publishes a gating
    /// motor fault within 20 ms rather than sitting in OFF with nothing asserted. That is the
    /// difference between this hold and the defect it replaces, which had no fault, no refusal
    /// count and no indication.
    #[test]
    fn an_unmeasurable_speed_is_held_with_a_fault_not_in_silence() {
        let mut health = motor::PeriodHealth::new();
        let level = |h: &motor::PeriodHealth| motor_fault_level(true, 0, h.loss(), false) != 0;
        for tick in 1..motor::PERIOD_LOSS_THRESHOLD {
            health.update(true, false);
            assert!(off_inhibit(true, false, 0), "inhibited from the first tick");
            assert!(
                !level(&health),
                "tick {tick} is still inside the jitter band"
            );
        }
        health.update(true, false);
        assert!(
            level(&health),
            "a sustained shortfall gates arming on its own, with an indication"
        );
    }

    /// **A run that ends with the wheel still turning does not wedge the board in OFF**
    /// (`specs/motor-integration.md`, "The OFF interlock must read a LIVE speed"; the defect
    /// measured on the bench master 2026-10-20,
    /// `specs/bench-evidence/2026-10-20/sine-phase/RECORD.md`, finding 3: the board armed 8 times,
    /// ended one run at speed -4, and refused every arm after it until a reset).
    ///
    /// The model is the real chain, and the one fact it takes from the image rather than from a
    /// copy of it is [`SHUTDOWN_STEPS`]: the period ISR is the sole writer of `motor::SPEED`, it
    /// runs only while the counter runs, and the shutdown sequence is the only thing in the image
    /// that could stop that counter. So the model applies the sequence as data, and a step that is
    /// not one of the posture steps is a counter stop. With the measurement stopped the word keeps
    /// the value the last window latched; with it running the wheel's own stop reaches it.
    #[test]
    fn a_run_that_ends_at_speed_does_not_wedge_the_machine_in_off() {
        use state::{Mode, ModeInputs, ModeMachine};

        // The run ends with the wheel still turning: the ISR's last 20 ms window latched -4.
        let mut speed = -4i32;
        let mut measuring = true;
        for step in SHUTDOWN_STEPS {
            measuring &= matches!(
                step,
                ShutdownStep::Disarm | ShutdownStep::ZeroDemand | ShutdownStep::FloatChannels
            );
        }

        // The wheel coasts to rest while the board sits disarmed. Each 20 ms window the ISR
        // latches the edges it counted, if it is still being served.
        for window_edges in [-2, -1, 0, 0] {
            if measuring {
                speed = window_edges;
            }
        }

        // The next power press, with no fault standing.
        let mut m: ModeMachine<1> = ModeMachine::new();
        let inputs = ModeInputs {
            power_request: true,
            off_inhibit: off_inhibit(true, measuring, speed),
            ..ModeInputs::default()
        };
        assert_eq!(
            m.tick(&inputs).mode,
            Mode::Init,
            "a shutdown taken at speed {speed} is still holding the machine in OFF"
        );
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
    const GOOD_SIGN: [i32; 3] = [-1, 1, -1];
    const GOOD_ROLES: [u8; 2] = [3, 2];

    /// A board's boot-fixed facts, as the bring-up would have built them: the boot-read noise
    /// floor at its registered default (`store::MOTOR_NOISE_FLOOR`, the floor the
    /// limit re-derivation converts against), and the quiet-bridge zeros the boot calibration
    /// accepted (the FOC capability; `None` here, so this board's `Foc` requests are demoted, which
    /// `crate::motor`'s own tests cover).
    const BOOT: motor::BootFixed = motor::BootFixed {
        noise_floor: store::MOTOR_NOISE_FLOOR.default(),
        accepted_offsets: None,
    };

    /// **A value written while disarmed is picked up by the re-read.** Every one of the
    /// value-row fields, written to a real mounted store and read back through the one read site.
    #[test]
    fn the_value_row_is_read_from_flash() {
        let v = with_store(|s| {
            s.set(store::MOTOR_METHOD, 2).unwrap();
            s.set(store::MOTOR_DIRECTION.at(0), 1).unwrap();
            s.set(store::MOTOR_ALIGN_OFFSET.at(0), 4).unwrap();
            s.set(store::CONTROL_MODE, 1).unwrap();
            s.set(store::CONTROL_FRAME, 1).unwrap();
            s.set(store::CONTROL_SIDE.at(0), 1).unwrap();
            s.set(store::MOTOR_CURRENT_LIMIT.at(0), 7_500).unwrap();
            s.set(store::MOTOR_CURRENT_CAL.at(0), 300).unwrap();
            for (i, sign) in [1i32, -1, 1].into_iter().enumerate() {
                s.set(store::IMU_AXIS_SIGN.at(i as u8), sign).unwrap();
            }
            s.set(store::IMU_AXIS_ROLE.at(0), 1).unwrap();
            s.set(store::IMU_AXIS_ROLE.at(1), 3).unwrap();
            s.set(store::CONTROL_RIDER_REQUIRED, 0).unwrap();
            s.set(store::BOARD_BATTERY_FLOOR, 2_900).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(0), 1_234).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(1), 567).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(2), 89).unwrap();
            s.set(store::CONTROL_GAIN_MAX.at(3), 12).unwrap();
            s.set(store::CONTROL_DRIVE_TURN.at(0), 2_500).unwrap();
            s.set(store::CONTROL_DRIVE_TURN.at(1), 33).unwrap();
            s.set(store::CONTROL_POS_CLAMP, 60).unwrap();
        });
        assert_eq!(
            v,
            ArmValues {
                method_byte: 2,
                direction_byte: 1,
                align_offset_byte: 4,
                control_mode_byte: 1,
                frame_byte: 1,
                side_byte: 1,
                current_limit_ma: 7_500,
                current_cal: 300,
                imu_sign: [1, -1, 1],
                imu_roles: [1, 3],
                rider_required_byte: 0,
                battery_floor: 2_900,
                gain_max: [1_234, 567, 89, 12],
                drive_turn: [2_500, 33],
                pos_clamp: 60,
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
    /// effect: the arm-time re-read"). What it perturbs includes `motor.dead_time` (a live timer
    /// register), every pin assignment, the timer-side and injected-group fields,
    /// `motor.noise_floor` (the one boot-read fact the arm path takes from `BootFixed`),
    /// `IMU_GYRO_BIAS` and the two live-lane fields `CONTROL_SF_GAINS` and `CONTROL_REFERENCE`.
    #[test]
    fn a_bring_up_row_field_is_not_in_the_value_row() {
        let untouched = with_store(|_| {});
        let mut perturbed = 0usize;
        let mut skipped = 0usize;
        let v = with_store(|s| {
            for row in store::REGISTRY.iter() {
                let def = row.def();
                if ROW_IDS.contains(&def.field_id) {
                    skipped += 1;
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
                    store::Value::Pin(_) => store::Value::Pin([2, 3]),
                    store::Value::PinAf(_) => store::Value::PinAf([2, 3, 4]),
                    store::Value::PinAdc(_) => store::Value::PinAdc([2, 3, 4]),
                };
                s.set_value(key, fresh).expect("the write must land");
                perturbed += 1;
            }
        });
        // The registry is the point of the test, so a parse or a filter that silently covered
        // nothing would be worse than no test.
        assert_eq!(
            perturbed + skipped,
            store::REGISTRY_LEN,
            "every registry entry is perturbed or in the value row"
        );
        assert!(
            perturbed > skipped,
            "only {perturbed} of {} registry entries were perturbed",
            store::REGISTRY_LEN
        );
        assert_eq!(
            v, untouched,
            "a write outside the value row must not change what the arm path re-reads"
        );
    }

    /// The field ids of the arm row, as [`read_arm_values`] reads them.
    const ROW_IDS: [u8; 15] = [
        0x15, // IMU_AXIS_SIGN
        0x16, // IMU_AXIS_ROLE
        0x20, // CONTROL_MODE
        0x21, // CONTROL_FRAME
        0x22, // CONTROL_RIDER_REQUIRED
        0x23, // MOTOR_METHOD
        0x26, // BOARD_BATTERY_FLOOR
        0x28, // CONTROL_GAIN_MAX
        0x2A, // CONTROL_POS_CLAMP
        0x2B, // CONTROL_DRIVE_TURN
        0x4C, // MOTOR_CURRENT_CAL
        0x4E, // MOTOR_CURRENT_LIMIT
        0x4F, // MOTOR_DIRECTION
        0x50, // CONTROL_SIDE
        0x51, // MOTOR_ALIGN_OFFSET
    ];

    /// `specs/machine-settings.md` section 9, test 1: [`ARM_ROW`] is exactly the registered keys
    /// whose effect is arm, so the row cannot drift from the registry's attribute, and [`ROW_IDS`]
    /// is its ids.
    #[test]
    fn the_arm_row_is_the_registrys_arm_effect_keys() {
        let arm: std::vec::Vec<store::Key> = store::REGISTRY
            .iter()
            .filter(|r| r.effect == store::Effect::Arm)
            .map(|r| r.key())
            .collect();
        assert_eq!(ARM_ROW.as_slice(), arm.as_slice());
        let mut ids: std::vec::Vec<u8> = ARM_ROW.iter().map(|k| k.field_id).collect();
        ids.dedup();
        assert_eq!(ids, ROW_IDS);
    }

    /// `specs/machine-settings.md` section 9, test 4, end to end over a real store: a persisted
    /// write of an arm-effect field sets `FLAG_REARM` until the next arm, a boot-effect field sets
    /// `FLAG_REBOOT` until boot, and a live field sets neither (its write moves only the machine
    /// digest). The firmware's one persist consumer recomputes `Digests::of` exactly as here.
    #[test]
    fn a_persisted_write_sets_the_flag_of_its_effect_class() {
        use linkctl::CyclicState as C;
        use orchestrator::{Digests, Machine, N_LINKS};
        let mut flash = crate::test_flash::TestFlash::erased();
        let mut s = store::Store::mount(&mut flash).unwrap();
        let mut m = Machine::new(Digests::of(&s), [false; N_LINKS]);
        let machine0 = m.digest();
        // Live: the tune lane's field. Neither flag, the machine digest moves.
        s.set(store::CONTROL_SF_GAINS.at(0), 1_000).unwrap();
        m.persisted(Digests::of(&s));
        assert_eq!(m.flags(), 0);
        assert_ne!(m.digest(), machine0);
        // Arm: the turn bound. FLAG_REARM until the arm-time re-read installs it.
        s.set(store::CONTROL_DRIVE_TURN.at(0), 2_000).unwrap();
        m.persisted(Digests::of(&s));
        assert_eq!(m.flags(), C::FLAG_REARM);
        m.armed();
        assert_eq!(m.flags(), 0);
        // A per-motor arm field at motor 0, the one the row reads.
        s.set(store::CONTROL_SIDE.at(0), 1).unwrap();
        m.persisted(Digests::of(&s));
        assert_eq!(m.flags(), C::FLAG_REARM);
        m.armed();
        // Boot: a pin. FLAG_REBOOT, which an arm does not clear.
        s.set_value(store::BOARD_SELF_HOLD.key(), store::Value::Pin([2, 0]))
            .unwrap();
        m.persisted(Digests::of(&s));
        assert_eq!(m.flags(), C::FLAG_REBOOT);
        m.armed();
        assert_eq!(m.flags(), C::FLAG_REBOOT);
        // The next boot runs what the store holds: both clear.
        let m = Machine::new(Digests::of(&s), [false; N_LINKS]);
        assert_eq!(m.flags(), 0);
        // A board-owned write (the walk's address) waits for nothing: the walk adopts it at once.
        let mut m = m;
        s.set(store::NODE_ADDRESS, 3).unwrap();
        m.persisted(Digests::of(&s));
        assert_eq!(m.flags(), 0);
    }

    /// **The stored frame is installed as stored** (`specs/config-read-once.md` section 5): a frame
    /// the CLI would refuse is still converted and installed, because the firmware checks nothing,
    /// and the carried-through bias rides with it.
    #[test]
    fn the_stored_imu_frame_is_installed_as_stored() {
        let values = |sign: [i32; 3], roles: [u8; 2]| ArmValues {
            method_byte: 0,
            direction_byte: 0,
            align_offset_byte: 2,
            control_mode_byte: 1,
            frame_byte: 0,
            side_byte: 0,
            current_limit_ma: 10_000,
            current_cal: 455,
            imu_sign: sign,
            imu_roles: roles,
            rider_required_byte: 1,
            battery_floor: 2_400,
            gain_max: [20_000, 20_000, 2_000, 10_000],
            drive_turn: [0, 24],
            pos_clamp: 45,
        };
        let bias = Some([48, 13, -88]);
        for (sign, roles) in [([1, 1, -1], GOOD_ROLES), (GOOD_SIGN, GOOD_ROLES)] {
            let r = rederive(&values(sign, roles), BOOT, bias);
            let cfg = r.imu.expect("an IMU board stages a config");
            assert_eq!(cfg.gyro_bias, [48, 13, -88]);
            assert_eq!(cfg.sign[..3], sign);
            assert_eq!(cfg.roles, roles);
        }
    }

    /// **A board with no IMU still re-derives.** `imu_bias: None` is "this board has no IMU", not a
    /// refusal: the throttle-only board arms exactly as it does today, with no config to install,
    /// and even a frame that would be refused cannot stop it (nothing reads those fields).
    #[test]
    fn a_board_with_no_imu_still_rederives() {
        let values = ArmValues {
            method_byte: 1,
            direction_byte: 0,
            align_offset_byte: 2,
            // Balance, on a board with no IMU: the byte travels UNDECODED, because a mode that
            // cannot run is demoted with the mode fault by the type that owns the IMU fact
            // (`ControlDispatch::re_apply_values`), exactly as at boot. It is not a refusal, so it
            // is not this function's business.
            control_mode_byte: 1,
            frame_byte: 0,
            side_byte: 0,
            current_limit_ma: 12_000,
            current_cal: 455,
            // Deliberately a reflection: with no IMU it is never staged, so it cannot refuse.
            imu_sign: [1, 1, -1],
            imu_roles: [2, 2],
            rider_required_byte: 0,
            battery_floor: 0,
            gain_max: [20_000, 20_000, 2_000, 10_000],
            drive_turn: [0, 24],
            pos_clamp: 900,
        };
        let r = rederive(&values, BOOT, None);
        assert!(r.imu.is_none(), "nothing to install");
        assert_eq!(
            r.control_mode_byte, 1,
            "the mode byte is carried, not judged"
        );
        // The rest of the row is still derived, through its own owners. The stored byte is 1, and
        // sine is open-loop, so it is honoured as asked: no current sense is needed to run it, and
        // this board has none accepted (`BOOT`).
        assert_eq!(r.motor.method, commutation::CommutationMethod::Sine.to_u8());
        assert_eq!(
            r.motor.records.method(),
            commutation::CommutationMethod::Sine
        );
        assert_eq!(r.rider_required_byte, 0);
        assert_eq!(r.battery_floor, 0);
        assert_eq!(r.gain_max, [20_000, 20_000, 2_000, 10_000]);
        // Carried raw: the control section's owner clamps it (`control::sf::pos_clamp`), the
        // `side_byte` pattern.
        assert_eq!(r.pos_clamp, 900);
    }

    /// The seam clamps ride on the owning types rather than on this layer: an out-of-range
    /// `CONTROL_DRIVE_TURN` pair is clamped by `DriveTurn::new`, which is the only place that range
    /// lives.
    #[test]
    fn the_drive_turn_goes_through_its_seam_clamp() {
        let mut values = ArmValues {
            method_byte: 0,
            direction_byte: 0,
            align_offset_byte: 2,
            control_mode_byte: 0,
            frame_byte: 0,
            side_byte: 0,
            current_limit_ma: 10_000,
            current_cal: 455,
            imu_sign: GOOD_SIGN,
            imu_roles: GOOD_ROLES,
            rider_required_byte: 1,
            battery_floor: 2_400,
            gain_max: [20_000, 20_000, 2_000, 10_000],
            drive_turn: [i16::MAX, 0],
            pos_clamp: 45,
        };
        let r = rederive(&values, BOOT, None);
        assert_eq!(r.drive_turn.turn_max() as i32, base::TORQUE_FULL_SCALE);
        assert_eq!(r.drive_turn.turn_slew(), control::drive::TURN_SLEW_MIN);
        values.drive_turn = [-1, i16::MAX];
        let r = rederive(&values, BOOT, None);
        assert_eq!(r.drive_turn.turn_max(), 0, "negative is the disabled state");
        assert_eq!(r.drive_turn.turn_slew(), control::drive::TURN_SLEW_MAX);
    }

    /// **A direction written while disarmed is live at the next arm, without a reboot, on both
    /// arms** (`specs/integration.md`, "When a stored value takes effect": `motor.direction` joined
    /// the value row on 2026-10-20, after the bench measured the boot-read it replaces: writing
    /// 0x62 and arming changed nothing, writing it and resetting reversed the wheel). One mounted
    /// store, one unchanged `BOOT`, two re-reads through the one read site: the first re-derives
    /// the Forward the board booted with; after a disarmed write of 1 the second re-derives
    /// Reverse, on the six-step decode and on the direction the angle path installs
    /// (`Commutator::switch_method`), and a sine request carries it the same way. Nothing on the
    /// motor side besides the direction moved between the two.
    #[test]
    fn a_direction_written_while_disarmed_is_live_at_the_next_arm() {
        use commutation::{Direction, MethodState};
        let mut flash = crate::test_flash::TestFlash::erased();
        let mut s = store::Store::mount(&mut flash).unwrap();
        s.set(store::MOTOR_METHOD, 0).unwrap();
        // The boot's first value: the registered default, Forward.
        let first = rederive(&read_arm_values(&s), BOOT, None);
        assert_eq!(first.motor.decode.direction(), Direction::Forward);
        let MethodState::SixStep(st) = first.motor.records else {
            panic!("six-step records");
        };
        assert_eq!(st.decode.direction(), Direction::Forward);
        // The disarmed write (R4 is what keeps it disarmed; here there is no bridge at all), then
        // the next arm's re-read.
        s.set(store::MOTOR_DIRECTION.at(0), 1).unwrap();
        let next = rederive(&read_arm_values(&s), BOOT, None);
        assert_eq!(
            next.motor.decode.direction(),
            Direction::Reverse,
            "the angle path's install"
        );
        let MethodState::SixStep(st) = next.motor.records else {
            panic!("six-step records");
        };
        assert_eq!(
            st.decode.direction(),
            Direction::Reverse,
            "the six-step decode"
        );
        assert_eq!(next.motor.method, first.motor.method);
        assert_eq!(next.motor.limit_counts, first.motor.limit_counts);
        // The same write reaches a sine board's angle path, where the records carry nothing.
        s.set(store::MOTOR_METHOD, 1).unwrap();
        let sine = rederive(&read_arm_values(&s), BOOT, None);
        assert!(matches!(sine.motor.records, MethodState::Sine));
        assert_eq!(sine.motor.decode.direction(), Direction::Reverse);
        // And written back to 0, the next arm is Forward again: no reset in between.
        s.set(store::MOTOR_DIRECTION.at(0), 0).unwrap();
        assert_eq!(
            rederive(&read_arm_values(&s), BOOT, None)
                .motor
                .decode
                .direction(),
            Direction::Forward
        );
    }

    /// **An align offset written while disarmed is live at the next arm, without a reboot**
    /// (`specs/integration.md`, "When a stored value takes effect": `motor.align_offset` joined
    /// the value row with the direction on 2026-10-20, because it enters the same decode one
    /// argument along and touches no peripheral register; as a boot-read, a sweep of 0..5 cost six
    /// reboots). The sweep itself, through one mounted store and one unchanged `BOOT`: each
    /// value written while disarmed is the decode the next arm installs, equal to a fresh
    /// `SixStep::new` for it, with the direction, method and limit unmoved throughout. The decode
    /// is what BOTH paths install since 2026-10-23 (`commutation::angle_path_offset` resolves the
    /// angle path's offset from this same value), so the sweep moves sine and FOC with six-step.
    #[test]
    fn an_align_offset_written_while_disarmed_is_live_at_the_next_arm() {
        use commutation::sixstep::SixStep;
        use commutation::{Direction, MethodState};
        let mut flash = crate::test_flash::TestFlash::erased();
        let mut s = store::Store::mount(&mut flash).unwrap();
        let first = rederive(&read_arm_values(&s), BOOT, None);
        for offset in [1u8, 5, 0, 3] {
            s.set(store::MOTOR_ALIGN_OFFSET.at(0), offset).unwrap();
            let next = rederive(&read_arm_values(&s), BOOT, None);
            let MethodState::SixStep(st) = next.motor.records else {
                panic!("six-step records");
            };
            assert_eq!(
                st.decode,
                SixStep::new(Direction::Forward, offset),
                "offset {offset}"
            );
            assert_eq!(
                next.motor.decode.direction(),
                first.motor.decode.direction()
            );
            assert_eq!(next.motor.method, first.motor.method);
            assert_eq!(next.motor.limit_counts, first.motor.limit_counts);
        }
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
        // The four the install is for, and therefore the only four it may name; of `faults` only
        // the method fault bit (checked below).
        let installed = ["method", "current", "commutator", "faults"];
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
        assert!(
            body.contains(concat!("(m.faults & !FAULT_", "METHOD) | r.method_fault")),
            "the install touches the method fault bit of `faults` and no other"
        );
        for token in [
            concat!("FAU", "LT.store"),
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

    /// **The arm-row function is the only reader of the arm row** (`specs/config-read-once.md`
    /// section 9): no other source file of the image reads an `ARM_ROW` field through the store, so
    /// the boot cannot grow an inline copy of a row read again. The handles' names are held to
    /// [`ARM_ROW`]'s ids, so a field added to the row without its name here fails.
    #[test]
    fn the_arm_row_function_is_the_only_reader_of_the_arm_row() {
        let handles: [(&str, u8); 15] = [
            ("IMU_AXIS_SIGN", store::IMU_AXIS_SIGN.id()),
            ("IMU_AXIS_ROLE", store::IMU_AXIS_ROLE.id()),
            ("CONTROL_MODE", store::CONTROL_MODE.id()),
            ("CONTROL_FRAME", store::CONTROL_FRAME.id()),
            ("CONTROL_RIDER_REQUIRED", store::CONTROL_RIDER_REQUIRED.id()),
            ("MOTOR_METHOD", store::MOTOR_METHOD.id()),
            ("BOARD_BATTERY_FLOOR", store::BOARD_BATTERY_FLOOR.id()),
            ("CONTROL_GAIN_MAX", store::CONTROL_GAIN_MAX.id()),
            ("CONTROL_POS_CLAMP", store::CONTROL_POS_CLAMP.id()),
            ("CONTROL_DRIVE_TURN", store::CONTROL_DRIVE_TURN.id()),
            ("MOTOR_CURRENT_CAL", store::MOTOR_CURRENT_CAL.id()),
            ("MOTOR_CURRENT_LIMIT", store::MOTOR_CURRENT_LIMIT.id()),
            ("MOTOR_DIRECTION", store::MOTOR_DIRECTION.id()),
            ("CONTROL_SIDE", store::CONTROL_SIDE.id()),
            ("MOTOR_ALIGN_OFFSET", store::MOTOR_ALIGN_OFFSET.id()),
        ];
        let mut ids: std::vec::Vec<u8> = ARM_ROW.iter().map(|k| k.field_id).collect();
        ids.dedup();
        let named: std::vec::Vec<u8> = handles.iter().map(|(_, id)| *id).collect();
        assert_eq!(named, ids, "the names here are the arm row's, in its order");
        let code = |src: &str| -> std::string::String {
            src.lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .collect::<std::vec::Vec<_>>()
                .join("\n")
        };
        for (file, src) in [
            ("main.rs", include_str!("main.rs")),
            ("motor.rs", include_str!("motor.rs")),
            ("layout.rs", include_str!("layout.rs")),
            ("odometer.rs", include_str!("odometer.rs")),
        ] {
            let c = code(src);
            for (name, _) in &handles {
                assert!(
                    !c.contains(&std::format!("get(store::{name}"))
                        && !c.contains(&std::format!("get({name}")),
                    "{file} reads {name}: the arm row is read by the arm-row function alone"
                );
            }
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
