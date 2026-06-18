//! System mode/fault state layer.
//!
//! This crate is the mode/fault-state layer from `todo/state.md`: the top-level vehicle mode graph
//! (OFF -> INIT -> READY -> RUN -> SHUTDOWN), the fixed fault codes, the per-motor fault-latch unit,
//! and the safety-owned MOE arming-gate decision. It is the integration point where every command
//! input and every safety condition converges, and it is the sole software owner of the
//! master-output-enable (MOE) gate decision.
//!
//! It is pure logic with no hardware dependency. It consumes already-debounced / decoded inputs
//! (power request, fault flags) and emits the mode byte plus the MOE-allowed decision per motor. The
//! firmware applies the decision to runtime-hal's `ComplementaryPwm` MOE; this crate never touches
//! hardware.
//!
//! Layering against `todo/safety.md`: safety.md owns the ARMING state machine (Unconfigured /
//! Disarmed / Commissioning / Armed / Fault-latched) and consumes the mode byte and fault flags this
//! layer publishes. The arming layer's `Armed` is a precondition: it feeds the OFF-only inhibit
//! here, so a Disarmed or still-Commissioning board holds OFF -> INIT. A self-test failure or a
//! hardware-break trip in safety.md maps onto a fault here (Fault A/B), tripping RUN -> SHUTDOWN.
//! Where the two could appear to disagree, safety.md's policy invariants win.
//!
//! Modules:
//! - [`mode`]: the mode state machine ([`mode::ModeMachine`]), the mode byte ([`mode::Mode`]), and
//!   the per-tick INIT / SHUTDOWN side-effect signals.
//! - [`fault`]: the fixed fault codes and the per-motor count-to-limit fault latch
//!   ([`fault::FaultLatch`]).
//! - [`moe`]: the per-motor MOE arming-gate decision ([`moe::MoeGate`]).
//!
//! `no_std`; host tests in `#[cfg(test)]` modules link `std` via the host target.

#![no_std]

pub mod fault;
pub mod mode;
pub mod moe;

// Public re-exports for the common types.
pub use fault::{
    FaultLatch, CODE_OVERCURRENT, CODE_STALL, COUNT_CAP, LATCH_THRESHOLD, RUN_SUBSTATE,
};
pub use mode::{InitAction, Mode, ModeInputs, ModeMachine, ShutdownAction, TickOutcome};
pub use moe::MoeGate;
