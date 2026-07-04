//! System mode/fault state layer (`specs/sensing-and-safety.md`).
//!
//! This crate is the Layer-6 mode/fault-state layer: the top-level vehicle mode graph
//! (OFF -> INIT -> READY -> RUN -> SHUTDOWN), the fixed fault codes, the per-motor fault-latch
//! unit, and the safety-owned MOE arming-gate decision. It is the integration point where every
//! command input and every safety condition converges, and it is the sole software owner of the
//! master-output-enable (MOE) gate decision.
//!
//! It is pure logic with no hardware dependency. It consumes already-debounced / decoded inputs
//! (power request, fault flags) and emits the mode byte plus the MOE-allowed decision per motor
//! as data; the integration layer enacts the decision against the hardware MOE (this crate never
//! touches hardware). Recovered from the archived implementation (`archive/accumulated-build`,
//! commit `74b7773`) per the spec's provenance section.
//!
//! Layering, per the spec's decisions:
//! - **This machine owns arming**: the arm predicate IS [`mode::ModeMachine::any_moe_allowed`];
//!   `armed == any_moe_allowed()` is the system's arm definition. The config-apply gate that
//!   consumes it (the spec's R4) is future integration work; the predicate itself ships here.
//! - **INIT enactment is method/capability-aware at the integration layer** (the commutation cal
//!   gate + `MOTOR_METHOD` decide what the bring-up constructs); the machine itself stays
//!   method-agnostic and only signals `InitAction`.
//! - **Command sources are parameterized data** ([`mode::ModeInputs`]); producers (buttons,
//!   remote, app, link supervision) arrive with the layers that exercise them.
//! - The 250 Hz cadence comes from the `scheduler` crate's dispatch, which the firmware main
//!   loop interleaves with its free-running link servicing (the spec's coexistence contract).
//!
//! Modules:
//! - [`mode`]: the mode state machine ([`mode::ModeMachine`]), the mode byte ([`mode::Mode`]),
//!   and the per-tick INIT / SHUTDOWN side-effect signals.
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
