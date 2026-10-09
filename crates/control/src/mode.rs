//! The control-mode model and dispatch (`specs/control.md` (b)): the runtime-selected
//! reference producer over ONE engagement shell. `CONTROL_MODE` is a registered store field
//! (the `MOTOR_METHOD` precedent; registered in `crates/store` with this consumer); the
//! validation seam enforces the one composition constraint (Balance requires a configured
//! IMU) with fallback + fault, the commutation Foc-without-current-sense precedent. The
//! engagement machine stays MODE-AGNOSTIC: this module never forks it; per-mode gating is
//! data in `FsmInputs` (throttle mode parameterizes the balance-only gates off by feeding a
//! zero upright reference).

use crate::throttle::{throttle_tick, ThrottleConfig, ThrottleOutput, ThrottleState};

/// The runtime control mode (`CONTROL_MODE`'s value vocabulary): `0 = Throttle` (the default:
/// works on every board, no IMU required; balancing is an opt-in), `1 = Balance`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    /// EFeru-parity input conditioning; never touches the IMU.
    Throttle = 0,
    /// The recovered stock cascade (shaping -> PID -> smoothed reference), consuming attitude.
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

/// The validation seam's outcome: the ACTIVE mode after the composition constraint, plus the
/// fault flag (raised when the request was demoted).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModeSelection {
    /// The mode actually in force.
    pub active: ControlMode,
    /// True when the requested mode was demoted (Balance requested without a configured IMU).
    pub fault: bool,
}

/// The mode-selection validation seam (spec (b), the commutation Foc precedent): Balance
/// requires a configured IMU (`imu.model != 0`); selecting it without one falls back to
/// Throttle AND raises the fault flag, exactly as `MOTOR_METHOD = Foc` without current sense
/// falls back to SixStep. `imu_configured` is a parameterized input; its real producer is the
/// integration layer's board plan (the board-model validator's IMU group).
pub fn select_mode(requested: u8, imu_configured: bool) -> ModeSelection {
    match ControlMode::from_u8(requested) {
        ControlMode::Balance if !imu_configured => ModeSelection {
            active: ControlMode::Throttle,
            fault: true,
        },
        m => ModeSelection {
            active: m,
            fault: false,
        },
    }
}

/// The mode dispatch: the active mode, the demotion fault, and the throttle producer's records.
/// The balance producer's records (`ShapingState` / `IirCarry` / `SpeedState`) live with the
/// orchestrator that runs the cascade (spec (g): integration); resetting THEM when the active mode
/// changes is that layer's duty, which is why [`ControlDispatch::re_apply_values`] reports the
/// change rather than keeping it to itself.
#[derive(Clone, Copy, Debug)]
pub struct ControlDispatch {
    mode: ControlMode,
    mode_fault: bool,
    /// Whether balance mode requires a rider (`CONTROL_RIDER_REQUIRED`, spec (i)). Set by the boot
    /// seam and RE-READ at every arm ([`ControlDispatch::re_apply_values`];
    /// `specs/integration.md`, "When a stored value takes effect: the arm-time re-read"), so a
    /// value written while disarmed takes effect at the next arm rather than the next boot. Those
    /// two sites are the only writers.
    rider_required: bool,
    /// The low-battery floor in centivolts (`CONTROL_BATTERY_FLOOR`,
    /// `specs/sensing-and-safety.md`, "The low-battery floor"); `<= 0` = no floor. Set by the boot
    /// seam beside the rider decision and re-read at every arm with it, the same discipline.
    battery_floor: i16,
    /// The throttle producer's conditioning records (replaced wholesale when an arm-time re-apply
    /// changes the active mode, the `switch_method` reset discipline).
    pub throttle: ThrottleState,
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

impl ControlDispatch {
    /// The boot seam: decode + validate the registered `CONTROL_MODE` byte against the board's
    /// IMU fact, decode the `CONTROL_RIDER_REQUIRED` byte (spec (i): `0` waives the rider
    /// requirement, any other value keeps it, so the default `1` and a corrupt byte both read as
    /// required), take the `CONTROL_BATTERY_FLOOR` word as written (no clamp beyond the type:
    /// `<= 0` is no floor, a floor above any reachable word refuses every balance engage), with
    /// fresh producer records.
    pub fn new(
        control_mode_byte: u8,
        imu_configured: bool,
        rider_required_byte: u8,
        battery_floor: i16,
    ) -> Self {
        let sel = select_mode(control_mode_byte, imu_configured);
        Self {
            mode: sel.active,
            mode_fault: sel.fault,
            rider_required: rider_required_from(rider_required_byte),
            battery_floor,
            throttle: ThrottleState::default(),
        }
    }

    /// The ARM-TIME value re-apply (`specs/integration.md`, "When a stored value takes effect: the
    /// arm-time re-read"): take a fresh read of the three value-row fields this type owns, decode
    /// them by the same rules the boot seam uses, and install them.
    ///
    /// Returns whether the ACTIVE mode changed. That is the caller's signal to replace the balance
    /// producer records, which live with the orchestrator that runs the cascade (spec (g)); the
    /// throttle records this type owns are replaced here on the same condition, and on that
    /// condition ONLY, because every arm runs this call and an arm that re-read the same mode byte
    /// must leave the conditioning carries exactly as the last pass left them.
    ///
    /// The validation seam re-runs unconditionally, so a demotion raises the fault and a corrected
    /// byte clears it exactly as at boot. It has to: a Balance request on a board with no IMU and a
    /// Throttle request both run Throttle, and only one of the two is a demotion, so the fault does
    /// not follow the mode change.
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
        imu_configured: bool,
        rider_required_byte: u8,
        battery_floor: i16,
    ) -> bool {
        let sel = select_mode(control_mode_byte, imu_configured);
        let changed = sel.active != self.mode;
        self.mode = sel.active;
        self.mode_fault = sel.fault;
        if changed {
            self.throttle = ThrottleState::default();
        }
        self.rider_required = rider_required_from(rider_required_byte);
        self.battery_floor = battery_floor;
        changed
    }

    /// The mode in force.
    pub fn mode(&self) -> ControlMode {
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

    /// The balance engage's battery term (`FsmInputs::battery_ok`): the effective battery word
    /// `battery` (centivolts, 0 = UNKNOWN) is known AND at or above the floor, or the floor is
    /// `<= 0` (none). Read only by the engage conjunction, so a word falling under the floor
    /// mid-run never ends the run.
    pub fn battery_ok(&self, battery: i16) -> bool {
        battery != 0 && (self.battery_floor <= 0 || battery >= self.battery_floor)
    }

    /// One throttle-producer tick (meaningful in [`ControlMode::Throttle`]; the balance mode's
    /// reference is the PID's smoothed output, produced by the cascade the orchestrator runs).
    /// The caller feeds the chosen side's `ref_*` into the engagement machine's mirror.
    pub fn throttle_reference(
        &mut self,
        cfg: &ThrottleConfig,
        speed_in: i16,
        steer_in: i16,
    ) -> ThrottleOutput {
        throttle_tick(cfg, speed_in, steer_in, &mut self.throttle)
    }
}
