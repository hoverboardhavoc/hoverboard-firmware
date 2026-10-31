//! Pipeline steps 7 + 8 (`specs/integration.md`): the control dispatch over the shared
//! engagement shell, and the cyclic TX build.
//!
//! Step 7 realizes `specs/control.md`'s assembly at the orchestrator, with the rigid frame's two
//! producers beside it (`specs/rover-one-imu.md`: the LEADER is the balance assembly whose output
//! also goes on the wire, the FOLLOWER applies the leader's word through the shared shell, and
//! both gate their wheel on the peer cyclic's freshness). The orchestrator owns the balance
//! producer record (`SfState`, the (g) note) alongside the `FsmState`, and wires the block words
//! into the crate-owned math. Per tick, in block order: the turn input (rigid leader), the
//! four-term state-feedback law (`specs/state-feedback.md`), then the engagement FSM (the sole
//! writer of the torque setpoint). Throttle mode runs the EFeru conditioner off the
//! effective drive command (neutral once stale: the `link-control.md` decay, applied at the
//! reference producer's input so the conditioning's own rate limit IS the decay ramp) and feeds
//! the same shell with the balance-only upright/rider/step-off gates parameterized off.
//!
//! Step 8 builds `CYCLIC_STATE` from the block words, stock-native, no rescaling at the link
//! boundary; emission is gated on holding an assigned address (the caller passes the address
//! fact, which lives in `net`).
//!
//! What the torque word MEANS follows the motor's running method: a duty on six-step and sine, a
//! q-axis current reference on FOC (`specs/foc-torque.md`). The one place this layer acts on the
//! difference is the balance law's battery normalization ([`law_scale`], fed
//! [`BlockWords::torque_is_current`]); every other path carries the word unchanged.

use crate::{events, LinkInbox, OrchestratorState};
use base::fixed::Fix;
use control::{
    clamp, fsm_step, iabs, sf_step, wheel_split, ControlDispatch, DriveTurn, FsmInputs, FsmState,
    GainProfile, GainShadow, GatingFilter, Producer, SfConfig, SfInputs, SfState, SubState,
    ThrottleConfig, STANDBY_SET,
};
use linkctl::{CyclicObs, CyclicState, DriveKind};

/// The pitch-RATE axis of the calibrated rad/s gyro vector (`imu::Sample::gyro`, sign map
/// applied) feeding the block's rate word (@0x9c, `specs/control.md` (j)): body Y (pitch is rotation about Y in the x-forward reference mount; the archive orchestrator's
/// wiring). A BODY-frame index: board mounts that differ, including a board on edge, are mapped
/// into body order upstream by the IMU frame (`imu::Config`'s sign map and axis roles, applied in
/// `imu::Imu::decode`, `specs/imu.md` `IMU_AXIS_ROLE`), so this constant never changes per board.
pub const PITCH_RATE_AXIS: usize = 1;

/// The UP axis of the sign-applied accel frame feeding the block's gating/pickup row: body Z
/// (level reads +Z gravity in the x-forward reference mount, `specs/attitude.md`). A BODY-frame
/// index, exactly as the pitch-rate axis is: the IMU frame (sign map + axis roles) maps every
/// mount into body order upstream; the stock firmware's own `-raw_az` IS that sign map for its
/// mount.
pub const UP_AXIS: usize = 2;

/// The step-7/8 control section of the orchestrator state: the mode dispatch plus the balance
/// producer records the orchestrator owns (`specs/control.md` mode.rs note), and the RAM control
/// block's input words whose producers are out of scope this round (`control.md` (e): each word
/// lives here, its canonical row, defaulted benign; tests and later producers write it).
pub struct ControlCtl {
    /// The mode dispatch (boot seam: `CONTROL_MODE` + `CONTROL_FRAME` bytes + `imu_configured` +
    /// `CONTROL_RIDER_REQUIRED` byte + `BOARD_BATTERY_FLOOR` word; the four stored ones are
    /// re-read at every arm through [`re_apply_control_values`]).
    pub dispatch: ControlDispatch,
    /// The throttle conditioning constants (EFeru defaults; a tunable surface).
    pub throttle_cfg: ThrottleConfig,
    /// The state-feedback law's carries (`specs/state-feedback.md` sections 3 and 4, balance
    /// producer record): the filtered wheel speed, the slewed speed reference, the position error,
    /// the live gains, this pass's four terms and the two position latches.
    pub sf: SfState,
    /// The engagement machine: the torque setpoint's sole writer.
    pub fsm: FsmState,
    /// The live tunable words (`specs/state-feedback.md` section 6): the RAM shadow of
    /// `CONTROL_SF_GAINS` and `CONTROL_REFERENCE`, built from the store at boot and writable live
    /// by the tune lane. Read once per balance pass into the law's configuration; never written
    /// from the control pass, so the tune lane's writer and this reader never contend for it.
    pub gains: GainShadow,
    /// The law's position clamp, edges: the seam-clamped `CONTROL_POS_CLAMP`
    /// (`control::sf::pos_clamp`), read at boot and re-read at every arm (`specs/integration.md`,
    /// "When a stored value takes effect: the arm-time re-read"). 0 disables the position term.
    pub pos_clamp: i16,
    /// The rigid frame's turn input bound and rate (`specs/rover-one-imu.md` section 5): the
    /// seam-clamped `CONTROL_DRIVE_TURN`, read at boot and re-read at every arm beside
    /// [`Self::pos_clamp`], the same discipline and the same reason. `turn_max` 0 (the default)
    /// keeps the machine driving straight.
    pub drive_turn: DriveTurn,
    /// The SLEWED turn, stock torque counts: the differential the split applies to the leader's
    /// common torque this pass. Written by [`balance_step`] on a leader (a stale drive command
    /// arrives as `steer = 0`, so the slew is the decay ramp) and read by
    /// [`control_dispatch_step`]'s split. Reset with the engagement machine
    /// ([`reset_engagement`]), which is the OFF pass and an arm that changed the producer.
    pub turn: i32,
    /// The gating/pickup row's conditioning carry (the recovered stock producer,
    /// `control::gating`). Stepped by the attitude step, not by the dispatch: it conditions an
    /// IMU channel, so it lives with the IMU tick and is NOT reset by a control-mode change,
    /// exactly as the attitude filter is not. Nor does it need to be: the attitude step conditions
    /// it every tick in either mode, so it cannot go stale across a change.
    pub gating: GatingFilter,
    /// The PRE-ENVELOPE torque view: the reference the active mode arm fed the engagement machine
    /// on the last pass, before the machine's gating and soft-start envelope act on it. Sole
    /// writer: [`run_shell`], which every mode arm goes through, so this is mode-agnostic (the
    /// state-feedback word in Balance, the EFeru conditioner's frame-out word in Throttle) and has
    /// exactly one write site.
    ///
    /// It exists because [`FsmState::torque_setpoint`](control::FsmState::torque_setpoint) is the
    /// machine's OUTPUT and is zero whenever the machine is disengaged, which is every disarmed
    /// tick. This word is live regardless of sub-state, so a disarmed board tilted by hand shows
    /// what the controller WOULD command: the tilt torque-shadow (`specs/control.md`,
    /// "Observation").
    pub pre_env_ref: i16,
    /// The demand this board applies to its OWN wheel this pass: the engagement machine's torque
    /// setpoint through the rigid freshness gate (`specs/rover-one-imu.md` section 6: a rigid
    /// board applies torque only while its peer's cyclic is fresher than
    /// [`RIGID_STALE_TICKS`](crate::RIGID_STALE_TICKS); on a pivoting board it IS the setpoint).
    /// Sole writer: [`control_dispatch_step`]. Readers: the firmware's demand publish (through
    /// the OBS snapshot), `CTRL_OBS`, and the cyclic's `peer_demand` on a leader, so what the
    /// leader commands its peer is exactly what it applies itself and a one-way link failure
    /// stops both wheels rather than one.
    pub wheel_demand: i16,
    /// The demand this board commands its PEER to apply this pass: the other half of the
    /// steering split on a rigid leader, through the same freshness gate as
    /// [`Self::wheel_demand`], and 0 from every other producer. Sole writer:
    /// [`control_dispatch_step`]. Sole reader: the cyclic builder's `peer_demand` word.
    ///
    /// Two fields and not one with a sign convention, because they are two different numbers the
    /// moment a turn is commanded: the leader keeps `T + turn` or `T - turn` by its own side and
    /// sends the other.
    pub peer_demand: i16,
}

impl ControlCtl {
    #[allow(clippy::too_many_arguments)]
    fn new(
        control_mode_byte: u8,
        frame_byte: u8,
        imu_configured: bool,
        gains: GainShadow,
        pos_clamp: i16,
        drive_turn: DriveTurn,
        rider_required_byte: u8,
        battery_floor: i16,
    ) -> Self {
        ControlCtl {
            dispatch: ControlDispatch::new(
                control_mode_byte,
                frame_byte,
                imu_configured,
                rider_required_byte,
                battery_floor,
            ),
            throttle_cfg: ThrottleConfig::default(),
            sf: SfState::default(),
            fsm: FsmState::default(),
            gains,
            pos_clamp: control::sf::pos_clamp(pos_clamp),
            drive_turn,
            turn: 0,
            gating: GatingFilter::default(),
            pre_env_ref: 0,
            wheel_demand: 0,
            peer_demand: 0,
        }
    }
}

/// The RAM control block's words as data (`specs/control.md` (e); the physical cells + volatile
/// access are the motor era's). Writers annotated per row; the rows whose producers are out of
/// scope this round hold documented placeholders and are settable (the block is their canonical
/// home, not a test hack).
pub struct BlockWords {
    /// Attitude pitch word (stock CB+0x3a), centidegrees. Writer: the attitude step (step 2).
    pub pitch_word: i16,
    /// Attitude roll word (stock CB+0x3e), centidegrees. Writer: the attitude step.
    pub roll_word: i16,
    /// The pitch-rate word (@0x9c, the stock PID `bv` / FSM `ref_9c` cell; the law's `pr`):
    /// rad/s x 10000, truncated toward zero, on [`PITCH_RATE_AXIS`] of `imu::Sample::gyro` (stock
    /// `f2iz(gy * 0.000266316114f * 10000.0f)`, 2.663 per raw count at +-500 deg/s full scale;
    /// `specs/control.md` (j)). Writer: the attitude step.
    pub pitch_rate: i32,
    /// Per-motor local wheel-speed word (stock CB+0x34): the period ISR's SIGNED EDGE COUNT per
    /// 320-period window, in the ISR's own unit with NO rescaling (`specs/link-control.md`,
    /// "`wheel_speed` carries the raw edge count"). Not the stock's speed unit, which would need
    /// this board's pole-pair count (`store::MOTOR_POLE_PAIRS`, registered and defaulting to
    /// unset, so no board carries one yet) and the stock's x376 fold. The stock-derived
    /// coefficients the readers below carry are therefore gains to tune against the plant rather
    /// than absolute inheritances.
    ///
    /// Writer: the firmware's 250 Hz shell, which narrows the period ISR's speed cell saturatingly
    /// into motor 0's row before each pass (`motor::wheel_speed_word`); the ISR owns the cell and
    /// this crate never names it, so the orchestrator stays host-testable. Motor 1 has no producer
    /// in the image and its row holds 0.
    ///
    /// Readers: the engagement blend's `ref_34`, the step-off latch's `b_motion`
    /// ([`crate::control_task`]) and the cyclic payload (`cyclic_state`).
    pub wheel_speed: [i16; crate::N_MOTORS],
    /// This board's period-based wheel rate (`specs/control.md` (o)): the period ISR's
    /// `Commutation::rate_q`, the reciprocal of the periods between two same-direction hall
    /// edges in 1/4096 of a count (one count = one edge per 320-period window, `wheel_speed`'s
    /// unit), decaying as `1/t` between edges and 0 across a reversal. Same sign sense as
    /// `wheel_speed`, the counter's own. Sent unscaled as `CyclicState::wheel_rate`.
    ///
    /// Writer: the firmware's 250 Hz shell, from `motor::WHEEL_RATE_Q12`, beside `wheel_speed`. Reader:
    /// the state-feedback law's speed term ([`control::sf_step`]), through the side mirror
    /// (`control::wheel_forward`), and the cyclic builder, for the remote app's speed readout. The
    /// recovered paths keep the count.
    pub wheel_rate_q12: i32,
    /// This board's SIGNED wheel position (`specs/state-feedback.md` section 3): the period ISR's
    /// `Commutation::pos_edges`, hall edges in the counter's own sense (the side mirror,
    /// `control::wheel_forward`, is the reader's), never cleared, wrapping, so a reader folds
    /// deltas.
    ///
    /// Writer: the firmware's 250 Hz shell, from `motor::POS_EDGES`, beside `wheel_rate_q12`.
    /// Readers: the state-feedback law's position fold ([`control::sf_step`]), the cyclic builder
    /// (its low 16 bits are `CyclicState::wheel_pos`) and the observation's own-wheel position.
    pub wheel_pos: i32,
    /// The board's EFFECTIVE battery word, centivolts, **0 = UNKNOWN** (stock CB+0x20;
    /// `specs/sensing-and-safety.md`, "The battery word"). Writer: the 250 Hz tick's source rule
    /// ([`crate::battery::battery_source`]: the local filtered sense where the plan carries
    /// `board.battery.pin`, else a fresh peer's nonzero cyclic word, else 0). Readers: the balance law's
    /// `scale` while its word is a duty ([`law_scale`]), the balance engage gate in every method,
    /// and the cyclic/telemetry payload.
    pub battery: i16,
    /// The FSM's shared gating/pickup halfword: the CONDITIONED UP-AXIS ACCEL COUNT (stock
    /// `0x20000204`, +-4 g at 8192 counts per g, so the machine's `> 500` engage edge is 0.061 g
    /// of gravity on the deck's up-axis and its `< 0` pickup edge is an inverted deck;
    /// `control::gating`). Writer: the attitude step (step 2), through [`ControlCtl::gating`].
    /// It holds across a missed IMU sample exactly as the attitude words do.
    ///
    /// This row is the BALANCE mode's gate input. Throttle mode does not read it: the up-axis
    /// gate is an orientation gate, and throttle mode runs on boards with no IMU at all, so it
    /// feeds the shell its own engagement intent (see [`throttle_step`]) the same way it
    /// parameterizes off the upright window and the pad gate.
    pub gating_field: i16,
    /// The orientation/role field (0x26): board fact, selects the FSM's alternate path. False =
    /// the standard orient==0 family.
    pub orientation_nz: bool,
    /// The left/right role flag: which wheel of the machine this board drives. Writer: the
    /// `store::CONTROL_SIDE` byte, at boot and at every arm ([`re_apply_control_values`]), which
    /// is this word's owner (`specs/rover-one-imu.md` section 5). Readers: the rigid LEADER's
    /// steering split ([`control_dispatch_step`]), the throttle reference's left/right pick
    /// ([`throttle_step`]), inert at the `steer = 0` the app sends outside the rigid drive mode,
    /// and the state-feedback law's side mirror ([`control::sf_step`]).
    pub role_right: bool,
    /// This board's torque word is a CURRENT: the motor is running `MOTOR_METHOD` 2, whose demand
    /// is a q-axis current reference (`specs/foc-torque.md` section 4). Writer: the firmware's
    /// 250 Hz shell, from the method byte the period ISR publishes (`motor::OBS_STATE`, the byte
    /// `CTRL_OBS` word 20 carries), so it is the method the motor RUNS, after the capability
    /// demotion, not the one staged. Reader: the balance law's divisor ([`law_scale`]). Nothing
    /// else: the battery floor gate keeps reading the measured [`Self::battery`].
    ///
    /// The leader's own method only. Its word reaches a rigid follower as `peer_demand` and means
    /// a current there only if the follower runs FOC too, which is a staging rule (both boards of a
    /// machine stage the same method), read back on the bench and not enforced here.
    pub torque_is_current: bool,
}

impl BlockWords {
    fn new(role_right: bool) -> Self {
        BlockWords {
            pitch_word: 0,
            roll_word: 0,
            pitch_rate: 0,
            wheel_speed: [0; crate::N_MOTORS],
            wheel_rate_q12: 0,
            wheel_pos: 0,
            battery: 0,
            gating_field: 0,
            orientation_nz: false,
            role_right,
            torque_is_current: false,
        }
    }
}

/// The balance law's output divisor (`control::SfInputs::scale`; `specs/foc-torque.md` section 4).
/// The law divides its numerator by the battery word (`numer * 3900 / scale`) because on six-step
/// and sine the word is a duty, and a duty's torque falls with the rail. A current reference does
/// not, so while the word is a current the divisor is the nominal pack
/// ([`control::config::sf::NOMINAL_SCALE`]) whatever the battery reads, including UNKNOWN: the
/// UNKNOWN word's refusal is the engage gate's ([`control::ControlDispatch::battery_ok`], fed the
/// measured word in both cases), so the terms stay live disarmed on a board with no battery source,
/// and a word lost mid-RUN no longer zeroes the torque of a machine that is already balancing.
#[inline]
pub(crate) const fn law_scale(battery: i16, torque_is_current: bool) -> i16 {
    if torque_is_current {
        control::config::sf::NOMINAL_SCALE
    } else {
        battery
    }
}

/// Build the control section (the [`OrchestratorState`] constructor's delegate).
#[allow(clippy::too_many_arguments)]
pub(crate) fn new_ctl(
    control_mode_byte: u8,
    frame_byte: u8,
    side_byte: u8,
    imu_configured: bool,
    gains: GainShadow,
    pos_clamp: i16,
    drive_turn: DriveTurn,
    rider_required_byte: u8,
    battery_floor: i16,
) -> (ControlCtl, BlockWords) {
    (
        ControlCtl::new(
            control_mode_byte,
            frame_byte,
            imu_configured,
            gains,
            pos_clamp,
            drive_turn,
            rider_required_byte,
            battery_floor,
        ),
        BlockWords::new(side_is_right(side_byte)),
    )
}

/// Decode the `store::CONTROL_SIDE` byte into [`BlockWords::role_right`]: `1 = right`, every
/// other byte = left, the `control::Frame::from_u8` rule (an unknown value reads as the registered
/// default). Not a harmless default on a right wheel: the state-feedback law reads its own and its
/// peer's wheel through this side, so an unset right wheel reverses the law's position and speed
/// terms, which is what gate 1 of `specs/state-feedback.md` reads disarmed.
///
/// One rule, two callers: the boot seam above and [`re_apply_control_values`].
const fn side_is_right(byte: u8) -> bool {
    byte == 1
}

/// The effective drive command (`specs/link-control.md`, "Supervision" + `DRIVE_CMD`): the
/// latest command's live words when fresh and of `Throttle` kind, else neutral (staleness is a
/// reference-zeroing, not a fault; the conditioning's rate limit turns the snap into the decay
/// ramp).
fn effective_drive(inbox: &LinkInbox) -> (i16, i16) {
    match inbox.drive() {
        Some(d) if d.kind == DriveKind::Throttle && !inbox.drive_stale() => (d.value, d.steer),
        _ => (0, 0),
    }
}

/// The folded rider level: local pads OR the remote `INPUTS` mirror OR the peer cyclic's rider
/// flag (the consumption fold; the cyclic TX reports LOCAL pads only, so mirrored levels never
/// feed back on themselves).
pub(crate) fn rider_level(state: &OrchestratorState) -> bool {
    state.rider_present
        || state.inbox.remote_rider_present()
        || state
            .inbox
            .peer()
            .map(|p| p.rider_present())
            .unwrap_or(false)
}

/// The EFFECTIVE rider level (`specs/control.md` (i), the rider requirement): the folded
/// [`rider_level`] on a board that requires a rider (`CONTROL_RIDER_REQUIRED`, id 0x23, default
/// 1 = required), else true, because a machine configured to carry no rider is not a machine
/// whose rider is missing.
///
/// ONE substitution, both consumers in the dispatch: [`balance_step`]'s engage conjunction and its
/// step-off producer, so the two cannot disagree about whether someone is aboard.
pub(crate) fn effective_rider(state: &OrchestratorState) -> bool {
    !state.ctl.dispatch.rider_required() || rider_level(state)
}

/// Step 7: one control-dispatch pass. `run` is this tick's mode-machine outcome (`Mode::Run`),
/// the engagement machine's power-enable nest (`control.md` (c): it nests inside RUN). Returns
/// the WHEEL demand (also left in [`ControlCtl::wheel_demand`]): the engagement machine's torque
/// setpoint (the block's sole-writer row, `state.ctl.fsm.torque_setpoint`) through the rigid
/// freshness gate.
///
/// **The one rule that covers most of the rigid frame's failure behaviour**
/// (`specs/rover-one-imu.md` section 6): a rigid board, leader or follower, applies torque only
/// while its peer's cyclic is fresher than [`RIGID_STALE_TICKS`](crate::RIGID_STALE_TICKS). Past
/// that age the follower has no command and the leader has no partner, and in both cases the
/// correct wheel demand is zero. A reference-zeroing and not a fault: the engagement machine keeps
/// its sub-state and the next fresh frame restores torque, so a single late pass costs a tick of
/// authority and not the machine; the 100 ms `comms_loss` disarm is the separate, slower clock.
pub(crate) fn control_dispatch_step(state: &mut OrchestratorState, run: bool) -> i16 {
    let producer = state.ctl.dispatch.mode();
    let setpoint = match producer {
        Producer::Balance | Producer::Leader => balance_step(state, run),
        Producer::Throttle => throttle_step(state, run),
        Producer::Follower => follower_step(state, run),
    };
    // The steering split (`specs/rover-one-imu.md` section 5), on the LEADER and nowhere else:
    // its setpoint is the MACHINE's common torque, and the slewed turn makes two wheel demands of
    // it on the same scale. Every other producer drives its own wheel with its setpoint and
    // commands its peer nothing.
    let (own, peer) = if producer == Producer::Leader {
        wheel_split(setpoint, state.ctl.turn, state.block.role_right)
    } else {
        (setpoint, 0)
    };
    // The freshness gate, applied AFTER the split and to BOTH numbers. Order matters: gating the
    // common torque first would leave the turn to be split out of a zero, which is two equal and
    // opposite NONZERO wheel demands, so a quiet link would spin the machine instead of stopping
    // it.
    let rigid = matches!(producer, Producer::Leader | Producer::Follower);
    let stale = rigid && state.inbox.rigid_peer().is_none();
    state.ctl.wheel_demand = if stale { 0 } else { own };
    state.ctl.peer_demand = if stale { 0 } else { peer };
    state.ctl.wheel_demand
}

/// The LIVE mode-fault level (`events::EV_MODE_FAULT`): the validation seam's demotion, OR the
/// two-leader staging error (`specs/rover-one-imu.md` section 2): a rigid leader whose fresh peer
/// cyclic also claims the leader flag. One function, read by the pipeline's level fold and by the
/// OBS snapshot, so the counter, the cyclic `fault` word and `CTRL_OBS` cannot disagree about it.
/// Not a Fault A, deliberately: the refusal is the cleared engagement gate in [`balance_step`],
/// which keeps both boards from engaging while this level says why.
pub(crate) fn mode_fault_level(state: &OrchestratorState) -> bool {
    state.ctl.dispatch.mode_fault()
        || (state.ctl.dispatch.mode() == Producer::Leader
            && state.inbox.rigid_peer().is_some_and(|p| p.leader()))
}

/// The `peer_down` level (`specs/rover-one-imu.md` section 6, "One wheel faults and the other does
/// not"): **in rigid mode only**, a fresh peer cyclic whose own level mask carries a Fault-A
/// producer. Folded into this board's `fault_a` beside `comms_loss`, so a one-wheel fault stops
/// both wheels.
///
/// The organising fact it serves: on a rigid two-wheel balancer a wheel that stops while the other
/// drives is worse than both stopping. One wheel delivers half the fore-aft authority and a yaw
/// moment the loop has no sensor for, and a disarmed wheel FREEWHEELS (there is no brake in this
/// firmware), so a one-sided stop turns a fall into a fall plus a spin.
///
/// The three conditions, each load-bearing:
///
/// - **Rigid only.** Two pivoting platforms are two machines; a peer's fault is not this board's
///   business, and the stock behaviour must not change under it.
/// - **[`LinkInbox::rigid_peer`](crate::LinkInbox::rigid_peer)**, so the word is read through the
///   freshness window every other rigid consumer reads it through. Past it the level releases and
///   the 100 ms `comms_loss` disarm is the producer that takes over.
/// - **[`events::FAULT_A_BITS`]**, the same group the local aggregate uses, so
///   `EV_POWER_REQUEST` (which an armed board holds for the whole arm) and `EV_MODE_FAULT`
///   are excluded by construction rather than by a comment. A released power request is not a
///   fault: `specs/rover-one-imu.md` names the two other mechanisms that cover it.
///
/// Self-clearing, with no arming deadlock: a powered, unarmed, healthy peer reports 0, and a
/// faulted peer's producers clear on its own OFF pass, after which its next cyclic reports 0 and
/// this level releases. What makes that true rather than hoped for is
/// [`events::Levels::own_mask`], the byte the peer actually sends.
pub(crate) fn peer_down_level(state: &OrchestratorState) -> bool {
    matches!(
        state.ctl.dispatch.mode(),
        Producer::Leader | Producer::Follower
    ) && state
        .inbox
        .rigid_peer()
        .is_some_and(|p| p.fault & events::FAULT_A_BITS != 0)
}

/// Run the shared engagement shell, recording the reference the mode arm fed it before the
/// machine's gating and soft-start envelope act on it ([`ControlCtl::pre_env_ref`], the
/// tilt torque-shadow's observable). Both mode arms call this, so the record is mode-agnostic and
/// has one writer.
///
/// The narrowing is a stated fact rather than luck, the [`throttle_step`] `demand_gate`
/// precedent: both arms bound the reference to +-28500 by construction (the balance law clamps
/// its word there; the throttle frame-out scale caps it), so the clamp cannot bind.
///
/// # Placement
///
/// Linked into `.hotcode` (`motorcontroller/firmware/memory.x`, "Hot-path flash placement"): on the
/// GD32F1x0 only the first 32 KiB of flash is zero-wait, and this is the 250 Hz control tick's
/// engagement machine. Before this function existed, the shell body was placed by the
/// `*(.text.*fsm_step*)` NAME ANCHOR in memory.x; introducing a wrapper inlined `fsm_step` into it
/// and renamed the emitted symbol, so the anchor silently stopped matching and the whole body
/// (980 B) landed above the boundary with the link still green. That is the exact failure mode
/// memory.x warns about, so this uses the mechanism it prefers instead: the attribute travels with
/// the item and cannot go stale on a rename. `#[inline(never)]` is required alongside it, because
/// `link_section` names where the emitted symbol lands and does not stop LLVM from inlining the
/// body back into a caller that lives elsewhere.
#[cfg_attr(target_arch = "arm", inline(never))]
#[cfg_attr(target_arch = "arm", link_section = ".hotcode")]
fn run_shell(ctl: &mut ControlCtl, inp: &FsmInputs) -> i16 {
    ctl.pre_env_ref = clamp(inp.smoothed_ref, i16::MIN as i32, i16::MAX as i32) as i16;
    // The machine is unchanged and still seeds and ramps a gain triple; nothing reads that triple
    // since the PID went, so every producer hands it the one constant profile
    // (`specs/state-feedback.md` section 5).
    fsm_step(inp, &GainProfile::of(STANDBY_SET), &mut ctl.fsm)
}

/// The balance assembly: the turn input, the four-term state-feedback law
/// (`specs/state-feedback.md`), then the engagement FSM. Runs the pivoting producer and the rigid
/// LEADER alike; the leader differs in one input, its engagement gate (below), and in what happens
/// to its output afterwards ([`control_dispatch_step`]'s freshness gate and the cyclic's
/// `peer_demand`).
fn balance_step(state: &mut OrchestratorState, run: bool) -> i16 {
    let leader = state.ctl.dispatch.mode() == Producer::Leader;
    let peer_wheel = state.inbox.peer().map(|p| p.wheel_speed).unwrap_or(0);
    // The EFFECTIVE rider level ([`effective_rider`], `specs/control.md` (i)), read once and fed
    // to both consumers below, so the engage conjunction and the step-off producer cannot
    // disagree about whether someone is aboard: waived, the conjunction drops the term and the
    // wind-down is held clear.
    let rider = effective_rider(state);
    let (drive_value, drive_steer) = effective_drive(&state.inbox);

    // The rigid frame's turn input (`specs/rover-one-imu.md` section 5), on the LEADER only: the
    // `DRIVE_CMD.steer` word as a bounded, slewed DIFFERENTIAL torque, stepped here where the
    // drive command is read and consumed by the split after the output stage. A stale link
    // arrives as `steer = 0`, so the slew is the decay ramp.
    if leader {
        state.ctl.drive_turn.step(drive_steer, &mut state.ctl.turn);
    }

    // The law (`specs/state-feedback.md` section 2), every tick, armed or not: the terms are live
    // outside RUN (the gains are then the shadow's), which is what makes the sign gate a disarmed
    // read, and the reference cells are zeroed on every tick the machine is not in RUN (section
    // 4). The stick is the effective drive value, so a stale link is target speed 0 and the slew
    // is the stop ramp. The peer's position is the inbox's unwrap, which moves only on a tick
    // that accepted a frame.
    let [speed_max, speed_slew] = state.ctl.gains.reference();
    let cfg = SfConfig {
        gains: state.ctl.gains.gains(),
        speed_max,
        speed_slew,
        pos_clamp: state.ctl.pos_clamp,
    };
    let inp = SfInputs {
        pitch_word: state.block.pitch_word,
        pitch_rate: state.block.pitch_rate,
        wheel_rate_q12: state.block.wheel_rate_q12,
        wheel_pos: state.block.wheel_pos,
        peer_pos: state.inbox.peer_wheel_pos().unwrap_or(0),
        role_right: state.block.role_right,
        value: drive_value,
        // The effective battery word (0 = UNKNOWN makes the word 0) while the word is a duty; the
        // nominal pack while it is a current.
        scale: law_scale(state.block.battery, state.block.torque_is_current),
        in_run: run && state.ctl.fsm.sub_state == SubState::Run,
    };
    let word = sf_step(&cfg, &inp, &mut state.ctl.sf);

    // The rigid LEADER's partner gate (`specs/rover-one-imu.md` sections 2 and 6): the leader
    // engages only while a fresh peer cyclic says the peer is the rigid FOLLOWER and not a second
    // leader. Both halves are staging errors a cleared gate turns into a machine that will not
    // arm: a peer running pivoting (no follower flag) would leave this wheel balancing alone, and
    // two leaders are two balance loops on one welded frame. The gate is the ENGAGE edge, so this
    // is a refusal to start, which is the cheap place to be told; a pivoting board's gate is its
    // own up-axis row, untouched.
    let partner_ok = !leader
        || state
            .inbox
            .rigid_peer()
            .is_some_and(|p| p.follower() && !p.leader());
    // The engagement FSM: the shared shell with the balance gates live.
    let fsm_in = FsmInputs {
        orientation_nz: state.block.orientation_nz,
        // The FSM's upright window takes the pitch in DEGREES (it scales x100 inside).
        upright_ref: fix_from_out(state.attitude.pitch_deg),
        pre_gate_clear: true, // the pre-gate byte's producer IS recovered (the still/level classifier's side-lying flag) and deliberately not built; see control.md
        smoothed_ref: word as i32,
        gating_field: if partner_ok {
            state.block.gating_field
        } else {
            0
        },
        rider_present: rider,
        // An UNKNOWN battery word blocks engage (the law would run at zero output), and so does a
        // word under the low-battery floor (`BOARD_BATTERY_FLOOR`); engage only, never a
        // disengage. The MEASURED word in every method: in torque mode the law no longer divides
        // by it, but an empty pack is still an empty pack.
        battery_ok: state.ctl.dispatch.battery_ok(state.block.battery),
        // The latched fault aggregate; the peer's lockdown flag enters the immediate-stop
        // inputs (`link-control.md`: a gating fault into the engagement machine, level).
        over_current: state.latches[0].is_latched() || state.inbox.peer_lockdown(),
        stall: false,
        comms_loss: state.inbox.comms_loss(),
        // The run-time tilt cutout's latch (`specs/control.md` (l)): past the limit on pitch or
        // roll the machine is falling, and this forces the engagement machine to IDLE on the tick
        // it asserts. The same latch is a Fault A producer one layer up, which is what disarms the
        // board and, through `peer_down`, stops a rigid follower's wheel with it.
        tilt: state.tilt_latch,
        enable_bytes_clear: true, // aggregate enable bytes: producers unrecovered, permitting
        power_enable: run,
        stop_byte: state.inbox.peer_lockdown(),
        // Step-off: the rider leaving is the wind-down producer; held clear (false) when the
        // requirement is waived, because `rider` is then true.
        winddown_enables_clear: !rider,
        promote_condition: false, // sub-2 promotion byte: producer unrecovered
        ref_9c: state.block.pitch_rate,
        ref_34: state.block.wheel_speed[0],
        ref_36: peer_wheel,
        feedback_fb: 0, // measured feedback: the motor era's producer
        // The dead triple's ramp (`control::config::ramp`): the battery word, and no `kd`.
        pid_scale: state.block.battery,
        pid_kd: Fix::ZERO,
    };
    run_shell(&mut state.ctl, &fsm_in)
}

/// The throttle assembly: the EFeru conditioner off the effective drive command, feeding the
/// SAME engagement shell with the balance-only gates parameterized off (zero upright reference,
/// rider/step-off gates inert: pads are balance's rider detection; `control.md` (b)/(c)).
fn throttle_step(state: &mut OrchestratorState, run: bool) -> i16 {
    let (speed_cmd, steer_cmd) = effective_drive(&state.inbox);
    let cfg = state.ctl.throttle_cfg;
    let out = state
        .ctl
        .dispatch
        .throttle_reference(&cfg, speed_cmd, steer_cmd);
    let reference = if state.block.role_right {
        out.ref_right
    } else {
        out.ref_left
    };

    // The mode's own gate input, on the +-28500 word the output stage envelopes: the demand's
    // MAGNITUDE. The shell's fixed `> 500` therefore reads as a 1.75 % demand deadband
    // (`DRIVE_CMD.value` ~590 of +-32767; 1000 -> 855, 2000 -> 1738), and because it is a
    // magnitude, both directions engage and the RUN pickup path stays inert in throttle mode
    // exactly as the step-off path does. The clamp cannot bind (the frame-out scale caps
    // |reference| at 28500); it is here so the i16 narrowing is a stated fact, not luck.
    let demand_gate = clamp(iabs(reference), 0, i16::MAX as i32) as i16;
    run_gated_shell(state, run, reference, demand_gate)
}

/// The rigid FOLLOWER's engagement intent on the shell's fixed `> 500` gate edge
/// (`specs/rover-one-imu.md` section 2): a positive constant comfortably above the edge, asserted
/// while the leader is fresh and in mode RUN. A constant and not the demand's magnitude, because
/// at the moment of engagement an upright, still rover needs almost no torque, so a deadband on
/// the demand would hold the follower disengaged exactly when the leader starts balancing.
/// Positive and constant, it also leaves the RUN pickup path (`< 0` for more than 20 ticks)
/// inert, as throttle mode's magnitude does.
pub const FOLLOWER_GATE: i16 = 1000;

/// The rigid FOLLOWER (`specs/rover-one-imu.md` section 2): no attitude, no IMU health, no
/// balance law.
/// Its whole control job is to take the leader's word off the fresh peer cyclic and pass it
/// through the shared engagement shell, with the balance-only gates parameterized off exactly as
/// throttle mode parameterizes them.
///
/// The word is applied only under the leader flag, so a peer's zero is a zero and not a command;
/// the engagement intent comes from the leader's MODE byte and not from the demand's magnitude
/// ([`FOLLOWER_GATE`]). Both read the FRESH peer only
/// ([`LinkInbox::rigid_peer`](crate::LinkInbox::rigid_peer)), so a stale leader is a zero
/// reference here as well as a zeroed wheel in [`control_dispatch_step`]: the pre-envelope shadow
/// a bench reads says what this board is being told right now. The follower's own soft-start
/// envelope still runs; the leader's word is already enveloped, so on boards that reach RUN
/// within a few ticks of each other the second clamp is a no-op (section 9 records the open
/// measurement).
fn follower_step(state: &mut OrchestratorState, run: bool) -> i16 {
    let leader = state.inbox.rigid_peer().filter(|p| p.leader());
    let demand = leader.map(|p| p.peer_demand).unwrap_or(0);
    let gate = match leader {
        Some(p) if p.mode == state::Mode::Run.as_byte() => FOLLOWER_GATE,
        _ => 0,
    };
    run_gated_shell(state, run, demand as i32, gate)
}

/// The shared engagement shell with the balance-only gates parameterized off: the shape the
/// throttle producer and the rigid follower have in common (`control.md` (b)/(c): one engagement
/// machine, N reference producers). `reference` is the producer's word on the +-28500 scale the
/// output stage envelopes; `gate` is the producer's own engagement intent on the shell's `> 500`
/// edge. Zero upright reference (the window always passes), rider present (the pad gate is
/// balance's), battery ok (neither producer divides by the battery), no step-off.
fn run_gated_shell(state: &mut OrchestratorState, run: bool, reference: i32, gate: i16) -> i16 {
    let fsm_in = FsmInputs {
        orientation_nz: state.block.orientation_nz,
        upright_ref: Fix::ZERO, // the upright window parameterized off (always passes)
        pre_gate_clear: true,
        smoothed_ref: reference,
        gating_field: gate,
        rider_present: true, // the pad gate is balance-only
        battery_ok: true,    // neither producer divides by the battery
        over_current: state.latches[0].is_latched() || state.inbox.peer_lockdown(),
        stall: false,
        comms_loss: state.inbox.comms_loss(),
        // The tilt latch, as the balance arm passes it. A board with no configured IMU never sets
        // it (the follower, a padless throttle kart), so this is live only on a board that has an
        // attitude to judge: a throttle board that tips over stops the same way a balancing one
        // does.
        tilt: state.tilt_latch,
        enable_bytes_clear: true,
        power_enable: run,
        stop_byte: state.inbox.peer_lockdown(),
        winddown_enables_clear: false, // no step-off in these modes
        promote_condition: false,
        ref_9c: state.block.pitch_rate,
        ref_34: state.block.wheel_speed[0],
        ref_36: state.inbox.peer().map(|p| p.wheel_speed).unwrap_or(0),
        feedback_fb: 0,
        // The ramp still runs in RUN (the machine is shared) on its dead triple, against the same
        // words the balance arm passes.
        pid_scale: state.block.battery,
        pid_kd: Fix::ZERO,
    };
    run_shell(&mut state.ctl, &fsm_in)
}

/// Reset the engagement machine to its disengaged state (`specs/motor-integration.md`,
/// prerequisite 4). Called on the OFF pass, the one edge every disengage path crosses: the mode
/// machine's SHUTDOWN arm always resolves to OFF, and RUN is reachable only back through
/// OFF -> INIT -> READY, so a machine reset here cannot be re-entered engaged.
///
/// [`re_apply_control_values`] calls it as its second caller, on an arm whose re-read changed the
/// active mode: the machine is mode-agnostic but its sub-state and soft-start envelope are not
/// meaningful across a change of which producer feeds it.
///
/// Why it is needed: the FSM is a faithful rebuild of the stock binary, whose only abort inputs
/// are comms-loss / stop / over-current. A `fault_a` producer with no stock input of its own
/// (`imu_loss`, the motor-side fault level, `fault_b`) therefore drives the mode machine to
/// SHUTDOWN while leaving `sub_state` at RUN and the soft-start envelope at its cap;
/// re-entering RUN would resume at full authority with no ramp. Resetting here rather than
/// widening the FSM's abort inputs keeps the machine binary-faithful and keeps the disengage act
/// on the orchestrator's existing OFF edge, beside the latch clear.
pub(crate) fn reset_engagement(ctl: &mut ControlCtl) {
    ctl.fsm = FsmState::default();
    // The slewed turn is the engagement machine's companion carry (`specs/rover-one-imu.md`
    // section 5): a machine that comes back engaged
    // must not resume mid-turn, and the slew's whole purpose is that the differential arrives as
    // a ramp rather than a yaw impulse.
    ctl.turn = 0;
}

/// The control section's ARM-TIME value apply (`specs/integration.md`, "When a stored value takes
/// effect: the arm-time re-read"): install the value-row fields the dispatch owns, and, where the
/// re-read changed the resolved PRODUCER (a mode or a frame), replace the balance producer record
/// this layer owns (`SfState`, the `specs/control.md` mode.rs note: replaced wholesale, the
/// `switch_method` discipline) and reset the engagement machine with it.
///
/// `CONTROL_MODE` is in the value row, so a mode change applies at the next arm like every other
/// value-shaped field, and this is the only path that moves it after boot. The write itself is
/// gated by R4 (the armed config-write refusal), one layer up, where the board is actually armed.
///
/// `imu_configured` is read from the state rather than taken, because it is a BOARD fact fixed at
/// bring-up: the validation seam must judge a fresh mode byte against the same IMU fact the boot
/// seam judged the stored one against.
/// The argument list is the value row's control half one for one, the boot seam's precedent; a
/// struct for the lint would be a type with this one caller and the firmware's one call site.
#[allow(clippy::too_many_arguments)]
pub fn re_apply_control_values(
    state: &mut OrchestratorState,
    control_mode_byte: u8,
    frame_byte: u8,
    side_byte: u8,
    rider_required_byte: u8,
    battery_floor: i16,
    pos_clamp: i16,
) {
    // The board's own side, this word's owner (`specs/rover-one-imu.md` section 5). Not a
    // producer change: which wheel this board is under does not reset anything.
    state.block.role_right = side_is_right(side_byte);
    // The law's position clamp, through its seam (`specs/state-feedback.md` section 4). In the
    // value row and not the tune lane, so it moves at an arm and never under a moving machine.
    state.ctl.pos_clamp = control::sf::pos_clamp(pos_clamp);
    let mode_changed = state.ctl.dispatch.re_apply_values(
        control_mode_byte,
        frame_byte,
        state.imu_configured,
        rider_required_byte,
        battery_floor,
    );
    if mode_changed {
        state.ctl.sf = SfState::default();
        reset_engagement(&mut state.ctl);
    }
}

/// Step 8: build this board's `CYCLIC_STATE` from the block words, stock-native, NO rescaling at
/// the link boundary (`specs/link-control.md`, "Payload shapes"). Returns `None` while the board
/// holds no assigned address (pre-assignment there is no peer contract; the caller passes the
/// address fact, which lives in `net`). The rider flag reports the LOCAL pads only (mirrored
/// levels must not feed back); the `fault` word is this pass's gating-producer level mask; the
/// lockdown flag has no local producer (the master-shutdown origin is motor-era work) and reports
/// clear.
pub fn cyclic_tx(state: &OrchestratorState, addressed: bool) -> Option<CyclicState> {
    if !addressed {
        return None;
    }
    // Emission rate split (round-4 defect C; `specs/link-control.md`, "Addressing and emission"):
    // emit every [`CYCLIC_EMIT_TICKS`] ticks (125 Hz nominal). The blocking polled TX costs ~1 ms
    // per frame even at 460800, which the 4 ms tick cannot afford every tick alongside the
    // control callback; at 125 Hz the receiver's 25-tick supervision window still holds 12x
    // margin.
    //
    // The decision itself is the pipeline's, on the elapsed-tick clock it keeps
    // (`OrchestratorState::cyclic_since_tx`); this reads the flag because the firmware calls it
    // with `&OrchestratorState`. It used to be `control_ticks.is_multiple_of(2)`, which is an
    // invocation parity and not a rate: measured on the offroad pair 2026-10-22, a 294-run
    // catch-up burst fired 147 frames back to back into a peer whose RX ring holds 38 ms of this
    // stream.
    if !state.cyclic_due {
        return None;
    }
    Some(cyclic_state(state))
}

/// Ticks between `CYCLIC_STATE` emissions on the inter-board port: 250 Hz / 2 = **125 Hz**, the
/// rate round-4 defect C settled. Named here beside [`BLE_CYCLIC_DIVISOR`], its twin, so the two
/// cadences read as one thing in one place; the clock they are counted against is
/// [`OrchestratorState::cyclic_since_tx`].
pub const CYCLIC_EMIT_TICKS: u32 = 2;

/// Build the `CYCLIC_STATE` payload from the block words. One builder, so the peer port and the BLE
/// port publish the same board state and can only differ in rate.
///
/// The appended block ([`CyclicObs`]) is part of every emission, as it is part of every payload
/// the codec accepts: the three current words are the window the firmware folded in before this
/// pass, and the two identity bytes are the boot's own constants.
///
/// `fault` is this board's OWN producer levels as the pipeline folded them this pass
/// (`events::Levels::own_mask`, held by [`OrchestratorState::own_levels`]), so a peer reads which
/// producers are asserted RIGHT NOW rather than a code it could miss between emissions
/// (`specs/rover-one-imu.md` section 4). Own levels and not the gating mask: `peer_down` is
/// derived from this byte on the other board, and echoing it back would latch both boards into a
/// fault neither could clear ([`events::Levels::own_mask`] carries the reasoning).
///
/// The rigid frame's words: a LEADER sets the leader flag and puts its peer's half of the
/// steering split ([`ControlCtl::peer_demand`]) on the wire, through the same freshness gate as
/// the half it applies itself, so a leader that cannot hear its follower commands it nothing and
/// a one-way link failure stops both wheels rather than one; a FOLLOWER sets the follower flag
/// and commands nothing; a pivoting or throttle board sets neither and commands nothing. The
/// field has one writer, [`control_dispatch_step`], which is why no role test is needed here.
fn cyclic_state(state: &OrchestratorState) -> CyclicState {
    let mut flags = 0u8;
    if state.rider_present {
        flags |= CyclicState::FLAG_RIDER;
    }
    let producer = state.ctl.dispatch.mode();
    match producer {
        Producer::Leader => flags |= CyclicState::FLAG_LEADER,
        Producer::Follower => flags |= CyclicState::FLAG_FOLLOWER,
        Producer::Throttle | Producer::Balance => {}
    }
    CyclicState {
        pitch: state.block.pitch_word,
        roll: state.block.roll_word,
        wheel_speed: state.block.wheel_speed[0],
        battery: state.block.battery as u16,
        mode: state.mode.mode_byte(),
        fault: state.own_levels,
        flags,
        peer_demand: state.ctl.peer_demand,
        // The low 16 bits, unmirrored: the leader unwraps it by delta and mirrors it by the
        // sender's side (`LinkInbox::peer_wheel_pos`).
        wheel_pos: state.block.wheel_pos as i16,
        obs: CyclicObs {
            phase_peak: state.phase.peak,
            phase_mean: state.phase.mean,
            duty_on: state.phase.duty_on,
            boot_tag: state.identity.boot_tag,
            chip: state.identity.chip,
            odometer: state.odometer,
        },
        // Unscaled, in the block's own Q12 (`specs/link-control.md`, `CYCLIC_STATE`).
        wheel_rate: state.block.wheel_rate_q12,
    }
}

/// Control runs between BLE cyclic emissions: 250 Hz / 50 = **5 Hz**.
///
/// The controller sample rides the SWD mailbox port and the bridge re-publishes it on its BLE
/// link (`specs/hoverboard-bridge.md`). Since 2026-10-30 (`wheel_rate`, then the `u16` `fault`)
/// the payload is 32 B (16 B committed + the 12 B `CyclicObs` block + the `i32` rate), an L3 PDU
/// of `3 + 32 = 35` B:
///
/// - Both carriers' L2 frame capacity is 128 (`swd_mailbox::FRAME_CAPACITY`, the bridge's
///   `router::FRAME`), so the PDU is ONE fragment, `1 + 35 = 36` B of L2 frame and 40 B of
///   stream with SOF, len and CRC. Own sample plus the 2.5 Hz peer forward put at most 80 B in
///   the mailbox's 256 B outbound ring between two host polls.
/// - On the bridge's BLE link the 40 B stream is two 20-byte notifications (20 + 20), as the
///   27 B payload's 35 B stream was. The phone reassembles from the byte stream, so no frame has
///   to fit one notification.
///
/// The arithmetic that set 5 Hz while the board drove its own CC2541 at 9600 baud (a 16 B frame,
/// two fragments, 40 B at 27 B of payload) no longer applies; 5 Hz stays because the app renders
/// rider-facing state, not a control input.
pub const BLE_CYCLIC_DIVISOR: u32 = 50;

/// The BLE port's decimated `CYCLIC_STATE`: the SAME payload [`cyclic_tx`] builds for the peer,
/// emitted at [`BLE_CYCLIC_DIVISOR`] (5 Hz) instead of 125 Hz.
///
/// Separate from [`cyclic_tx`] rather than a second consumer of its output because the two ports
/// have unrelated budgets: the inter-board UART is 460800 baud and point-to-point, the BLE module is
/// 9600 baud and shared with the command stream. Gated on `addressed` for the same reason
/// [`cyclic_tx`] is: an unassigned board has no `src` worth publishing.
pub fn ble_cyclic_tx(state: &OrchestratorState, addressed: bool) -> Option<CyclicState> {
    if !addressed {
        return None;
    }
    // On the elapsed-tick clock, for the reason [`cyclic_tx`] gives.
    if !state.ble_cyclic_due {
        return None;
    }
    Some(cyclic_state(state))
}

/// Ticks between forwards of the PEER's `CYCLIC_STATE` to the BLE port: 250 Hz / 100 =
/// **2.5 Hz**, half the board's own [`BLE_CYCLIC_DIVISOR`] rate (`specs/link-control.md`, "The
/// peer's state on the BLE port: the forward").
///
/// Half, because the second board is rendered and not ridden from: its battery, fault code, mode
/// and part change on a scale of seconds or never, and its attitude is display-only. The budget is
/// the reason it is not simply the same rate. A sample is 40 B on this wire (two fragments, 41.7 ms
/// at 9600 8N1), so the own stream is 200 B/s and this forward adds 100 B/s: 300 B/s, 31% of the
/// outbound direction. On the shared-budget model the specs have used so far (both directions
/// against one 960 B/s), BOUND throttle mode's command stream to two boards (568 B/s) puts the port
/// at 90% with the forward at 2.5 Hz and would put it at 101% at 5 Hz, past the port. The
/// odometer's four payload bytes took those from 84% and 93% to 89% and 99%, and `wheel_pos`'s two
/// took them the rest of the way.
///
/// The clock is [`OrchestratorState::ble_peer_since_tx`], ticks elapsed and not calls made, so a
/// catch-up burst forwards once. The counter starts at zero: the first forward is 400 ms after
/// construction, since no peer is known at boot and there is no announce-yourself case.
pub const BLE_PEER_CYCLIC_TICKS: u32 = 100;

/// A peer's state to forward, with the address it arrived under: the two halves of the PDU the
/// firmware builds, `[OP_CYCLIC_STATE][src][0x00][state]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PeerCyclic {
    /// The L3 address of the board that stated this, which is the `src` the forward carries so
    /// the phone attributes the state to the peer and not to this board.
    pub src: u8,
    /// The peer's last accepted state, as the inbox holds it.
    pub state: CyclicState,
}

/// The BLE port's forward of the PEER's `CYCLIC_STATE` (`specs/link-control.md`, "The peer's state
/// on the BLE port: the forward"): the inbox's last accepted peer state under the peer's own `src`,
/// every [`BLE_PEER_CYCLIC_TICKS`], and only while that state is fresh
/// ([`LinkInbox::fresh_peer`](crate::LinkInbox::fresh_peer), the complement of `comms_loss`).
///
/// Gated on `addressed` as the board's own emissions are: an unassigned board is silent on the
/// BLE port no matter what it has heard. The state is re-encoded by the firmware through the same
/// codec the peer encoded it with, which is a bijection on the 32 B layout, so the bytes that
/// reach the phone are the bytes the peer put on the inter-board wire.
pub fn ble_peer_tx(state: &OrchestratorState, addressed: bool) -> Option<PeerCyclic> {
    if !addressed || !state.ble_peer_due {
        return None;
    }
    state
        .inbox
        .fresh_peer()
        .map(|(src, s)| PeerCyclic { src, state: s })
}

/// Centidegrees (i16, saturating) from a degree-valued `Out`: the stock-native attitude word
/// scale (the FSM's own x100.0f upright scaling of the degree-valued reference is the in-crate
/// precedent for the degree<->centidegree relation).
pub(crate) fn out_to_centi(deg: base::fixed::Out) -> i16 {
    let centi = (deg.to_bits() as i64 * 100) >> 16;
    centi.clamp(i16::MIN as i64, i16::MAX as i64) as i16
}

/// A degree-valued `Out` widened to `Fix` (lossless: I16F16 -> I32F32).
pub(crate) fn fix_from_out(deg: base::fixed::Out) -> Fix {
    Fix::from_bits((deg.to_bits() as i64) << 16)
}
