//! The master-output-enable (MOE) arming gate (state.md §4, §8).
//!
//! This crate is the sole software writer of MOE in normal operation. It DECIDES whether the bridge
//! hardware gate is allowed to be active; the firmware ENACTS the decision against runtime-hal's
//! `ComplementaryPwm` MOE. The decision is safety-owned arming, never opened by the control loop.
//!
//! The MOE-ordering invariant (FIXED): MOE is set exactly once, on the INIT pass, and cleared on
//! every SHUTDOWN pass and every fault path. It must never be left enabled across an OFF dwell. This
//! matches the arming-layer expectation in safety.md (Disarmed / Fault-latched hold MOE clear) and
//! the hardware break, which clears the same MOE by independent means.
//!
//! There is one [`MoeGate`] per motor (one per advanced timer). The N-motor mapping (state.md §9)
//! runs the INIT bring-up and the SHUTDOWN safe-down per motor; a fault on any motor trips the
//! single shared mode machine, which safes all motors.

/// The per-motor MOE arming-gate decision. Holds whether MOE is currently allowed (armed). The
/// firmware reads [`MoeGate::allowed`] each tick and drives the hardware MOE to match: allowed ->
/// MOE active, not allowed -> MOE clear.
///
/// The gate is driven only by the mode machine's lifecycle events, never directly by the control
/// loop:
/// - [`MoeGate::set_at_init`] on the single INIT pass (the only place MOE is opened).
/// - [`MoeGate::clear`] on every SHUTDOWN pass and every fault path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MoeGate {
    /// True when the bridge gate is allowed active (armed). False holds the bridge hardware-off.
    allowed: bool,
}

impl MoeGate {
    /// A fresh gate with MOE not allowed (clear). The reset-settled resting state; the bridge is
    /// disabled until an INIT pass opens it.
    pub const fn new() -> Self {
        MoeGate { allowed: false }
    }

    /// The decision the firmware applies: `true` = MOE may be active, `false` = MOE must be clear.
    pub const fn allowed(&self) -> bool {
        self.allowed
    }

    /// Open the gate. Called ONLY from the INIT pass (state.md §4.2 step 4), and only here in the
    /// normal path. This is the safety-owned arming write; the control loop never calls it.
    pub fn set_at_init(&mut self) {
        self.allowed = true;
    }

    /// Clear the gate (force the bridge hardware-off decision). Called on every SHUTDOWN pass
    /// (§4.3 steps 1-2) and on every fault path. Idempotent: clearing an already-clear gate is a
    /// no-op, so a combined fault-and-power-off SHUTDOWN is safe.
    pub fn clear(&mut self) {
        self.allowed = false;
    }
}

impl Default for MoeGate {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_gate_is_clear() {
        let g = MoeGate::new();
        assert!(!g.allowed(), "reset-settled resting state has MOE clear");
    }

    #[test]
    fn set_at_init_opens_gate() {
        let mut g = MoeGate::new();
        g.set_at_init();
        assert!(g.allowed());
    }

    #[test]
    fn clear_forces_gate_off_and_is_idempotent() {
        let mut g = MoeGate::new();
        g.set_at_init();
        g.clear();
        assert!(!g.allowed());
        g.clear(); // idempotent
        assert!(!g.allowed());
    }
}
