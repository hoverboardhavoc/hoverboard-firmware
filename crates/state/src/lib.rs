//! System mode/fault state layer (`specs/sensing-and-safety.md`).
//!
//! This crate is the Layer-6 mode/fault-state layer: the fixed fault codes and the per-motor
//! fault-latch unit ([`fault`], this slice), and, arriving with the next slice per the spec's
//! implementation slicing, the top-level mode graph (OFF -> INIT -> READY -> RUN -> SHUTDOWN)
//! and the safety-owned MOE arming-gate decision.
//!
//! Pure logic with no hardware dependency: it consumes already-debounced / decoded inputs (fault
//! flags, power request) and emits decisions as data. Recovered from the archived implementation
//! (`archive/accumulated-build`, commit `74b7773`).
//!
//! `no_std`; host tests in `#[cfg(test)]` modules link `std` via the host target.

#![no_std]

pub mod fault;

// Public re-exports for the common types.
pub use fault::{
    FaultLatch, CODE_OVERCURRENT, CODE_STALL, COUNT_CAP, LATCH_THRESHOLD, RUN_SUBSTATE,
};
