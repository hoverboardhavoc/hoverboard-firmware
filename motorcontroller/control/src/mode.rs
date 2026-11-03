//! The control-mode model and dispatch (`specs/control.md` (b)): the runtime-selected
//! reference producer over ONE engagement shell. `CONTROL_MODE` and `CONTROL_FRAME` are
//! registered store fields (the `MOTOR_METHOD` precedent; registered in `shared/store` with this
//! consumer); the validation seam resolves the two bytes, the board's `CONTROL_LEADER` flag and
//! its IMU fact into the one [`Producer`] that runs (`specs/rigid-leader.md` section 5.1, the
//! resolution table), with
//! fallback + fault where the request cannot be honoured, the commutation
//! Foc-without-current-sense precedent. The engagement machine stays MODE-AGNOSTIC: this module
//! never forks it; per-mode gating is data in `FsmInputs` (throttle mode and the rigid follower
//! parameterize the balance-only gates off by feeding a zero upright reference).

use crate::throttle::{throttle_tick, ThrottleConfig, ThrottleOutput, ThrottleState};

/// The runtime control mode (`CONTROL_MODE`'s value vocabulary): `0 = Throttle` (the default:
/// works on every board, no IMU required; balancing is an opt-in), `1 = Balance`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    /// EFeru-parity input conditioning; never touches the IMU.
    Throttle = 0,
    /// The state-feedback balance law (`specs/state-feedback.md`), consuming attitude.
    Balance = 1,
}

impl ControlMode {
    /// Decode the registered field byte. Unknown values fall back to [`ControlMode::Throttle`]
    /// (the mode that works on every board; the fail-safe default, spec (b)).
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => ControlMode::Balance,
            _ => ControlMode::Throttle,
        }
    }
}

/// The machine's frame (`CONTROL_FRAME`'s value vocabulary, `specs/control.md` (k)):
/// `0 = Pivoting` platforms (the default: each board balances the platform it is under on its
/// own IMU, the stock hoverboard), `1 = Rigid` body (one balance loop for the machine, both
/// wheels taking its torque, `specs/rover-one-imu.md`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame {
    /// Two platforms, two loops: every board runs its own producer.
    Pivoting = 0,
    /// One welded body, one loop: the board whose `CONTROL_LEADER` is set leads, the other follows.
    Rigid = 1,
}

impl Frame {
    /// Decode the registered field byte. Unknown values fall back to [`Frame::Pivoting`], which
    /// is byte-identical to the image before the field existed (the fail-safe default).
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Frame::Rigid,
            _ => Frame::Pivoting,
        }
    }
}

/// The RESOLVED reference producer: what the mode-selection seam made of the stored bytes and the
/// board's IMU fact (`specs/rigid-leader.md` section 5.1, the resolution table). The
/// discriminants are what `CTRL_OBS` publishes as the control-mode byte, so a bench read tells a
/// leader from a follower.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Producer {
    /// The EFeru-parity conditioner off the drive command; never touches the IMU.
    Throttle = 0,
    /// The balance law on this board's own IMU, for this board's own platform.
    Balance = 1,
    /// The balance law as the rigid frame's LEADER: its torque drives this wheel AND goes on the
    /// wire as `CYCLIC_STATE.peer_demand`, under the leader flag.
    Leader = 2,
    /// The rigid frame's FOLLOWER: no attitude, no balance law; it applies the leader's word
    /// through the shared engagement shell.
    Follower = 3,
}

/// The validation seam's outcome: the producer in force, plus the fault flag (raised when the
/// request was demoted).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModeSelection {
    /// The producer actually in force.
    pub active: Producer,
    /// True when the request cannot be honoured: Balance requested on a pivoting board without a
    /// configured IMU (demoted to Throttle), or a flagged rigid leader without one (it resolves to
    /// the leader and its engagement gate is held closed, `specs/rigid-leader.md` section 5.2).
    pub fault: bool,
}

/// The mode-selection validation seam (spec (b), the commutation Foc precedent; the table is
/// `specs/rigid-leader.md` section 5.1):
///
/// | `machine.mode` | `machine.frame` | `control.leader` | IMU configured | producer |
/// |---|---|---|---|---|
/// | Throttle | either | either | either | [`Producer::Throttle`] |
/// | Balance | Pivoting | either | yes | [`Producer::Balance`] |
/// | Balance | Pivoting | either | no | [`Producer::Throttle`], **mode fault** |
/// | Balance | Rigid | set | yes | [`Producer::Leader`] |
/// | Balance | Rigid | set | no | [`Producer::Leader`], **mode fault** (the gate held closed) |
/// | Balance | Rigid | clear | either | [`Producer::Follower`] |
///
/// Pivoting Balance requires a configured IMU (stored, and answering its probe); selecting it
/// without one falls back to Throttle AND raises the fault flag, exactly as `MOTOR_METHOD = Foc`
/// without current sense falls back to SixStep. On a rigid frame the leader flag picks the role and
/// the IMU does not: a flagged board leads, and without its IMU it raises the fault, which the
/// orchestrator's partner gate reads to keep it from engaging; an unflagged board follows, whatever
/// IMU it has. `leader_byte` is read only in the two rigid balance rows; any nonzero byte is set.
/// `imu_configured` is a parameterized input; its real producer is the integration layer's board
/// plan (the board-model validator's IMU group).
pub fn select_mode(
    requested: u8,
    frame_byte: u8,
    leader_byte: u8,
    imu_configured: bool,
) -> ModeSelection {
    let mut fault = false;
    let active = match (ControlMode::from_u8(requested), Frame::from_u8(frame_byte)) {
        (ControlMode::Throttle, _) => Producer::Throttle,
        (ControlMode::Balance, Frame::Pivoting) if imu_configured => Producer::Balance,
        (ControlMode::Balance, Frame::Pivoting) => {
            fault = true;
            Producer::Throttle
        }
        (ControlMode::Balance, Frame::Rigid) if leader_byte != 0 => {
            fault = !imu_configured;
            Producer::Leader
        }
        (ControlMode::Balance, Frame::Rigid) => Producer::Follower,
    };
    ModeSelection { active, fault }
}

/// The mode dispatch: the resolved producer, the demotion fault, and the throttle producer's records,
/// one per motor the board drives (`M`, the firmware's motor count, `specs/twelve-fet.md` section
/// 4).
/// The balance producer's record (`SfState`) lives with the orchestrator that runs the law
/// (spec (g): integration); resetting THEM when the active mode
/// changes is that layer's duty, which is why [`ControlDispatch::re_apply_values`] reports the
/// change rather than keeping it to itself.
#[derive(Clone, Copy, Debug)]
pub struct ControlDispatch<const M: usize> {
    mode: Producer,
    mode_fault: bool,
    /// Whether balance mode requires a rider (`CONTROL_RIDER_REQUIRED`, spec (i)). Set by the boot
    /// seam and RE-READ at every arm ([`ControlDispatch::re_apply_values`];
    /// `specs/integration.md`, "When a stored value takes effect: the arm-time re-read"), so a
    /// value written while disarmed takes effect at the next arm rather than the next boot. Those
    /// two sites are the only writers.
    rider_required: bool,
    /// The low-battery floor in centivolts (`BOARD_BATTERY_FLOOR`,
    /// `specs/sensing-and-safety.md`, "The low-battery floor"); `<= 0` = no floor. Set by the boot
    /// seam beside the rider decision and re-read at every arm with it, the same discipline.
    battery_floor: i16,
    /// The throttle producer's conditioning records, one per motor, each fed its own motor's drive
    /// command (replaced wholesale when an arm-time re-apply changes the active mode, the
    /// `switch_method` reset discipline).
    pub throttle: [ThrottleState; M],
}

/// Decode the `CONTROL_RIDER_REQUIRED` byte (spec (i)): `0` waives the rider requirement, any other
/// value keeps it, so the field's default `1` and a corrupt byte both read as required.
///
/// One rule, two callers: [`ControlDispatch::new`] at boot and
/// [`ControlDispatch::re_apply_values`] at every arm.
#[inline]
const fn rider_required_from(byte: u8) -> bool {
    byte != 0
}

impl<const M: usize> ControlDispatch<M> {
    /// The boot seam: resolve the registered `CONTROL_MODE`, `CONTROL_FRAME` and `CONTROL_LEADER`
    /// bytes against the board's IMU fact ([`select_mode`]), decode the `CONTROL_RIDER_REQUIRED` byte (spec (i): `0`
    /// waives the rider requirement, any other value keeps it, so the default `1` and a corrupt
    /// byte both read as required), take the `BOARD_BATTERY_FLOOR` word as written (no clamp
    /// beyond the type: `<= 0` is no floor, a floor above any reachable word refuses every balance
    /// engage), with fresh producer records.
    pub fn new(
        control_mode_byte: u8,
        frame_byte: u8,
        leader_byte: u8,
        imu_configured: bool,
        rider_required_byte: u8,
        battery_floor: i16,
    ) -> Self {
        let sel = select_mode(control_mode_byte, frame_byte, leader_byte, imu_configured);
        Self {
            mode: sel.active,
            mode_fault: sel.fault,
            rider_required: rider_required_from(rider_required_byte),
            battery_floor,
            throttle: [ThrottleState::default(); M],
        }
    }

    /// The ARM-TIME value re-apply (`specs/integration.md`, "When a stored value takes effect: the
    /// arm-time re-read"): take a fresh read of the five value-row fields this type owns, decode
    /// them by the same rules the boot seam uses, and install them.
    ///
    /// Returns whether the resolved PRODUCER changed (a frame change moving Balance to Leader
    /// counts, `specs/rover-one-imu.md` section 5, and so does a moved `CONTROL_LEADER` flag). That is the caller's signal to replace the balance
    /// producer record, which lives with the orchestrator that runs the law (spec (g)); the
    /// throttle records this type owns are replaced here on the same condition, and on that
    /// condition ONLY, because every arm runs this call and an arm that re-read the same mode byte
    /// must leave the conditioning carries exactly as the last pass left them.
    ///
    /// The validation seam re-runs unconditionally, so a demotion raises the fault and a corrected
    /// byte clears it exactly as at boot. It has to: a Balance request on a board with no IMU and a
    /// Throttle request both run Throttle, and only one of the two is a demotion, so the fault does
    /// not follow the mode change (nor does a flagged leader's, which keeps the producer).
    ///
    /// **There is no armed-request refusal here, and the reason is the CALL SITE.** The arm path
    /// runs this as its FIRST step, on the pass the mode machine grants the MOE allowance
    /// (`arm::ArmStep::ReReadValues`), so a refusal keyed on that allowance would refuse every arm
    /// rather than guard anything. What keeps a mode change away from an energized bridge is R4
    /// (the armed config-write gate, `specs/integration.md`), which is where the refusal belongs:
    /// the byte this reads can only have reached flash while disarmed, and `ArmStep::SetMoe` is the
    /// LAST arm step, so nothing installed here reaches a gate driver before every other
    /// precondition has passed.
    pub fn re_apply_values(
        &mut self,
        control_mode_byte: u8,
        frame_byte: u8,
        leader_byte: u8,
        imu_configured: bool,
        rider_required_byte: u8,
        battery_floor: i16,
    ) -> bool {
        let sel = select_mode(control_mode_byte, frame_byte, leader_byte, imu_configured);
        let changed = sel.active != self.mode;
        self.mode = sel.active;
        self.mode_fault = sel.fault;
        if changed {
            self.throttle = [ThrottleState::default(); M];
        }
        self.rider_required = rider_required_from(rider_required_byte);
        self.battery_floor = battery_floor;
        changed
    }

    /// The producer in force.
    pub fn mode(&self) -> Producer {
        self.mode
    }

    /// True when the requested mode was demoted at the validation seam.
    pub fn mode_fault(&self) -> bool {
        self.mode_fault
    }

    /// Whether balance mode requires a rider (spec (i)). False waives the term: the balance arm
    /// substitutes a present rider for all three of the level's consumers (the engage
    /// conjunction, the step-off wind-down producer, the profile select).
    pub fn rider_required(&self) -> bool {
        self.rider_required
    }

    /// The low-battery floor in force, centivolts (`<= 0` = none): the battery-critical alarm's
    /// threshold (`specs/tones.md` section 4.4) beside the engage gate's.
    pub fn battery_floor(&self) -> i16 {
        self.battery_floor
    }

    /// The balance engage's battery term (`FsmInputs::battery_ok`): the effective battery word
    /// `battery` (centivolts, 0 = UNKNOWN) is known AND at or above the floor, or the floor is
    /// `<= 0` (none). Read only by the engage conjunction, so a word falling under the floor
    /// mid-run never ends the run.
    pub fn battery_ok(&self, battery: i16) -> bool {
        battery != 0 && (self.battery_floor <= 0 || battery >= self.battery_floor)
    }

    /// One throttle-producer tick for motor `motor` (meaningful in [`ControlMode::Throttle`]; the
    /// balance mode's reference is the state-feedback law's word, which the orchestrator runs), on
    /// that motor's own conditioning records. The caller feeds the chosen side's `ref_*` into the
    /// engagement machine. `motor` is below `M`: the caller iterates its motors.
    pub fn throttle_reference(
        &mut self,
        motor: usize,
        cfg: &ThrottleConfig,
        speed_in: i16,
        steer_in: i16,
    ) -> ThrottleOutput {
        throttle_tick(cfg, speed_in, steer_in, &mut self.throttle[motor])
    }
}
