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
/// orchestrator that runs the cascade (spec (g): integration); resetting THEM on a mode switch
/// is that layer's duty under the same disarmed-only rule, exactly as the commutation
/// integration layer owns `switch_method`'s disarmed gate.
#[derive(Clone, Copy, Debug)]
pub struct ControlDispatch {
    mode: ControlMode,
    mode_fault: bool,
    /// The throttle producer's conditioning records (replaced wholesale on a mode switch, the
    /// `switch_method` reset discipline).
    pub throttle: ThrottleState,
}

impl ControlDispatch {
    /// The boot seam: decode + validate the registered `CONTROL_MODE` byte against the board's
    /// IMU fact, with fresh producer records.
    pub fn new(control_mode_byte: u8, imu_configured: bool) -> Self {
        let sel = select_mode(control_mode_byte, imu_configured);
        Self {
            mode: sel.active,
            mode_fault: sel.fault,
            throttle: ThrottleState::default(),
        }
    }

    /// The mode in force.
    pub fn mode(&self) -> ControlMode {
        self.mode
    }

    /// True when the requested mode was demoted at the validation seam.
    pub fn mode_fault(&self) -> bool {
        self.mode_fault
    }

    /// The mode-switch seam (spec (b): mode changes apply while DISARMED only, the
    /// `MOTOR_METHOD` rule; a mode change is a config write). Returns whether the switch
    /// applied. On apply, the producer records are REPLACED wholesale with fresh ones (the
    /// commutation `switch_method` reset discipline) and the validation seam re-runs (a
    /// demotion on switch raises the fault exactly as at boot). Armed requests are refused
    /// without touching anything.
    pub fn switch_mode(&mut self, requested: u8, imu_configured: bool, disarmed: bool) -> bool {
        if !disarmed {
            return false;
        }
        let sel = select_mode(requested, imu_configured);
        self.mode = sel.active;
        self.mode_fault = sel.fault;
        self.throttle = ThrottleState::default();
        true
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
