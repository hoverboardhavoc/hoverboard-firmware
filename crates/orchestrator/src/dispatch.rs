//! Pipeline steps 7 + 8 (`specs/integration.md`): the control dispatch over the shared
//! engagement shell, and the cyclic TX build.
//!
//! Step 7 realizes `specs/control.md`'s assembly at the orchestrator: the orchestrator owns the
//! balance producer records (`ShapingState` / `IirCarry` / `SpeedState`, the (g) note) alongside
//! the `FsmState`, and wires the block words into the crate-owned math. Per tick, in block
//! order: the speed loop (the `pp` producer), the shaper (the commanded lean), the balance PID
//! (consuming the FSM's live gain triple from the previous transition), then the engagement FSM
//! (the sole writer of the torque setpoint). Throttle mode runs the EFeru conditioner off the
//! effective drive command (neutral once stale: the `link-control.md` decay, applied at the
//! reference producer's input so the conditioning's own rate limit IS the decay ramp) and feeds
//! the same shell with the balance-only upright/rider/step-off gates parameterized off.
//!
//! Step 8 builds `CYCLIC_STATE` from the block words, stock-native, no rescaling at the link
//! boundary; emission is gated on holding an assigned address (the caller passes the address
//! fact, which lives in `net`).
//!
//! The cascade->FOC drive path is out of scope entirely (`specs/control.md` (a): the d-ramp
//! source contradiction; no consumer exists pre-motor). The torque word's only consumers this
//! round are OBS and the cyclic build.

use crate::{LinkInbox, OrchestratorState};
use base::fixed::Fix;
use control::{
    balance_pid, fsm_step, select_profile, shape_pitch_target, speed_loop, ControlDispatch,
    ControlMode, FsmInputs, FsmState, IirCarry, PidInputs, ShapingInputs, ShapingState,
    SpeedInputs, SpeedState, SubState, ThrottleConfig,
};
use linkctl::{CyclicState, DriveKind};

/// The pitch-RATE axis of the sign-applied gyro frame feeding the block's rate word (@0x9c):
/// body Y (pitch is rotation about Y in the x-forward reference mount; the archive orchestrator's
/// wiring). Board mounts that differ recalibrate through the attitude sign maps upstream.
pub const PITCH_RATE_AXIS: usize = 1;

/// The battery placeholder word, centivolts: the fleet-nominal 36 V pack, above the PID's 3500
/// hysteresis knee (`specs/integration.md`, Out-scope: the block word carries a placeholder
/// until the inputs/sensing producer is built; it must be nonzero, it is the PID's divisor).
pub const BATTERY_PLACEHOLDER_CENTIVOLT: i16 = 3600;

/// The step-7/8 control section of the orchestrator state: the mode dispatch plus the balance
/// producer records the orchestrator owns (`specs/control.md` mode.rs note), and the RAM control
/// block's input words whose producers are out of scope this round (`control.md` (e): each word
/// lives here, its canonical row, defaulted benign; tests and later producers write it).
pub struct ControlCtl {
    /// The mode dispatch (boot seam: `CONTROL_MODE` byte + `imu_configured`).
    pub dispatch: ControlDispatch,
    /// The throttle conditioning constants (EFeru defaults; a tunable surface).
    pub throttle_cfg: ThrottleConfig,
    /// The shaper's persistent state (balance producer record).
    pub shaping: ShapingState,
    /// The PID reference-IIR carry (balance producer record).
    pub iir: IirCarry,
    /// The speed/steer loop state (balance producer record).
    pub speed: SpeedState,
    /// The engagement machine: the torque setpoint's sole writer.
    pub fsm: FsmState,
}

impl ControlCtl {
    fn new(control_mode_byte: u8, imu_configured: bool) -> Self {
        ControlCtl {
            dispatch: ControlDispatch::new(control_mode_byte, imu_configured),
            throttle_cfg: ThrottleConfig::default(),
            shaping: ShapingState::default(),
            iir: IirCarry::default(),
            speed: SpeedState::default(),
            fsm: FsmState::default(),
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
    /// The pitch-rate word (@0x9c, the PID `bv` / FSM `ref_9c` cell): the sign-applied gyro
    /// counts on [`PITCH_RATE_AXIS`]. Writer: the attitude step.
    pub pitch_rate: i32,
    /// Per-motor local wheel-speed word (stock CB+0x34). Writer: the commutation ISR (motor
    /// era); placeholder 0 pre-motor.
    pub wheel_speed: [i16; crate::N_MOTORS],
    /// The filtered local battery word, centivolts. Producer: the sensing task (not built;
    /// VBATT is master-only anyway); placeholder [`BATTERY_PLACEHOLDER_CENTIVOLT`]. A peer
    /// cyclic's battery word takes precedence as the PID scale (`link-control.md`: the scale
    /// input on boards without VBATT sense).
    pub battery: i16,
    /// The FSM's shared gating/pickup halfword (engage requires `> 500`). Producer: the stock
    /// producer is unrecovered (inputs work, `control.md` (g)); 0 = engagement gated closed.
    pub gating_field: i16,
    /// The speed-loop trim word (`control.md` (d) input assembly). Producer out of scope; 0.
    pub trim: i16,
    /// The PID derivative coefficient `kd` (@0xb4, Q). Producer: bench tuning (unrecovered);
    /// 0 per the archive orchestrator's precedent (no derivative authority until tuned).
    pub kd: Fix,
    /// The orientation/role field (0x26): board fact, selects the FSM's alternate path. False =
    /// the standard orient==0 family.
    pub orientation_nz: bool,
    /// The left/right role flag (the shaper's steer sign). Board fact; false.
    pub role_right: bool,
}

impl BlockWords {
    fn new() -> Self {
        BlockWords {
            pitch_word: 0,
            roll_word: 0,
            pitch_rate: 0,
            wheel_speed: [0; crate::N_MOTORS],
            battery: BATTERY_PLACEHOLDER_CENTIVOLT,
            gating_field: 0,
            trim: 0,
            kd: Fix::ZERO,
            orientation_nz: false,
            role_right: false,
        }
    }
}

/// Build the control section (the [`OrchestratorState`] constructor's delegate).
pub(crate) fn new_ctl(control_mode_byte: u8, imu_configured: bool) -> (ControlCtl, BlockWords) {
    (
        ControlCtl::new(control_mode_byte, imu_configured),
        BlockWords::new(),
    )
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

/// The PID scale word: the peer cyclic's filtered battery when a peer exists (the boards-without-
/// VBATT-sense rule), else the local block word (the placeholder until the sensing producer).
fn battery_scale(state: &OrchestratorState) -> i16 {
    state
        .inbox
        .peer()
        .map(|p| p.battery as i16)
        .unwrap_or(state.block.battery)
}

/// Step 7: one control-dispatch pass. `run` is this tick's mode-machine outcome (`Mode::Run`),
/// the engagement machine's power-enable nest (`control.md` (c): it nests inside RUN). Returns
/// the torque setpoint word (also left in the block's sole-writer row,
/// `state.ctl.fsm.torque_setpoint`).
pub(crate) fn control_dispatch_step(state: &mut OrchestratorState, run: bool) -> i16 {
    match state.ctl.dispatch.mode() {
        ControlMode::Balance => balance_step(state, run),
        ControlMode::Throttle => throttle_step(state, run),
    }
}

/// The balance assembly (`specs/control.md` (c)/(d), block order): speed loop -> shaper -> PID
/// -> engagement FSM.
fn balance_step(state: &mut OrchestratorState, run: bool) -> i16 {
    let peer = state.inbox.peer();
    let peer_wheel = peer.map(|p| p.wheel_speed).unwrap_or(0);
    let rider = rider_level(state);
    let (_, drive_steer) = effective_drive(&state.inbox);
    let pitch_fix = fix_from_out(state.attitude.pitch_deg);

    // The speed loop: the `pp` (correction) producer, every tick. The gate byte / window / dir
    // parameters have unrecovered producers (`control.md` open questions): gate off keeps the
    // integrator on its decay-only path; the s1/s2 mapping (local, peer) is the provisional
    // reading pinned on the bench when the loop first drives hardware.
    let s_in = SpeedInputs {
        blend_input: pitch_fix,
        trim: state.block.trim,
        gate: false,
        s1: state.block.wheel_speed[0],
        s2: peer_wheel,
        window: 0,
        wheel_a: state.block.wheel_speed[0],
        wheel_b: peer_wheel,
        dir_step: Fix::ZERO,
        dir_out_pos: Fix::ZERO,
        dir_out_neg: Fix::ZERO,
        run_active: run && state.ctl.fsm.sub_state == SubState::Run,
    };
    speed_loop(&s_in, &mut state.ctl.speed);

    // The shaper: the commanded lean. roll_b is the peer's roll mirror (`link-control.md`'s
    // roll_b consumer); no peer degrades local-only (pass roll_a, the ShapingInputs contract).
    let sh_in = ShapingInputs {
        roll_a: state.block.roll_word,
        roll_b: peer.map(|p| p.roll).unwrap_or(state.block.roll_word),
        steer: drive_steer,
        role_right: state.block.role_right,
    };
    let off = shape_pitch_target(&sh_in, &mut state.ctl.shaping);

    // The balance PID, consuming the FSM's live gain triple (written on the PREVIOUS
    // transition: "every setup takes effect on the next PID tick").
    let gains = state.ctl.fsm.gains;
    let pid_in = PidInputs {
        bv: state.block.pitch_rate,
        bk: gains.bk,
        pp: state.ctl.speed.correction,
        kp: gains.kp,
        pr: gains.pr,
        kd: state.block.kd,
        off,
        scale: battery_scale(state),
    };
    let pid_out = balance_pid(&pid_in, &mut state.ctl.iir);

    // The engagement FSM: the shared shell with the balance gates live.
    let fsm_in = FsmInputs {
        orientation_nz: state.block.orientation_nz,
        upright_ref: pitch_fix,
        pre_gate_clear: true, // the master pre-gate byte's producer is unrecovered (inputs work)
        smoothed_ref: pid_out.smoothed_ref as i32,
        gating_field: state.block.gating_field,
        rider_present: rider,
        // The latched fault aggregate; the peer's lockdown flag enters the immediate-stop
        // inputs (`link-control.md`: a gating fault into the engagement machine, level).
        over_current: state.latches[0].is_latched() || state.inbox.peer_lockdown(),
        stall: false,
        comms_loss: state.inbox.comms_loss(),
        tilt: false,
        enable_bytes_clear: true, // aggregate enable bytes: producers unrecovered, permitting
        power_enable: run,
        stop_byte: state.inbox.peer_lockdown(),
        winddown_enables_clear: !rider, // step-off: the rider leaving is the wind-down producer
        promote_condition: false,       // sub-2 promotion byte: producer unrecovered
        ref_9c: state.block.pitch_rate,
        ref_34: state.block.wheel_speed[0],
        ref_36: peer_wheel,
        feedback_fb: 0, // measured feedback: the motor era's producer
    };
    let profile = select_profile(rider);
    fsm_step(&fsm_in, &profile, &mut state.ctl.fsm)
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

    let fsm_in = FsmInputs {
        orientation_nz: state.block.orientation_nz,
        upright_ref: Fix::ZERO, // the upright window parameterized off (always passes)
        pre_gate_clear: true,
        smoothed_ref: reference,
        gating_field: state.block.gating_field,
        rider_present: true, // the pad gate is balance-only
        over_current: state.latches[0].is_latched() || state.inbox.peer_lockdown(),
        stall: false,
        comms_loss: state.inbox.comms_loss(),
        tilt: false,
        enable_bytes_clear: true,
        power_enable: run,
        stop_byte: state.inbox.peer_lockdown(),
        winddown_enables_clear: false, // no step-off in throttle mode
        promote_condition: false,
        ref_9c: state.block.pitch_rate,
        ref_34: state.block.wheel_speed[0],
        ref_36: state.inbox.peer().map(|p| p.wheel_speed).unwrap_or(0),
        feedback_fb: 0,
    };
    let profile = select_profile(rider_level(state));
    fsm_step(&fsm_in, &profile, &mut state.ctl.fsm)
}

/// The disarmed-only control-mode switch (`specs/control.md` (b): a mode change is a config
/// write, applied while disarmed only). Wraps `ControlDispatch::switch_mode` with the arm fact
/// (`any_moe_allowed`, the system's arm definition) and, on apply, resets the balance producer
/// records the orchestrator owns (the mode.rs note: replaced wholesale, the `switch_method`
/// discipline; the FSM/block state resets with them). Returns whether the switch applied.
pub fn switch_control_mode(state: &mut OrchestratorState, requested: u8) -> bool {
    let disarmed = !state.mode.any_moe_allowed();
    let applied = state
        .ctl
        .dispatch
        .switch_mode(requested, state.imu_configured, disarmed);
    if applied {
        state.ctl.shaping = ShapingState::default();
        state.ctl.iir = IirCarry::default();
        state.ctl.speed = SpeedState::default();
        state.ctl.fsm = FsmState::default();
    }
    applied
}

/// Step 8: build this board's `CYCLIC_STATE` from the block words, stock-native, NO rescaling at
/// the link boundary (`specs/link-control.md`, "Payload shapes"). Returns `None` while the board
/// holds no assigned address (pre-assignment there is no peer contract; the caller passes the
/// address fact, which lives in `net`). The rider flag reports the LOCAL pads only (mirrored
/// levels must not feed back); the fault-code byte and the lockdown flag have no local producers
/// this round (the trip/code producers and the master-shutdown origin are motor-era work) and
/// report healthy/clear.
pub fn cyclic_tx(state: &OrchestratorState, addressed: bool) -> Option<CyclicState> {
    if !addressed {
        return None;
    }
    let mut flags = 0u8;
    if state.rider_present {
        flags |= CyclicState::FLAG_RIDER;
    }
    Some(CyclicState {
        pitch: state.block.pitch_word,
        roll: state.block.roll_word,
        wheel_speed: state.block.wheel_speed[0],
        battery: state.block.battery as u16,
        mode: state.mode.mode_byte(),
        fault: 0,
        flags,
    })
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
