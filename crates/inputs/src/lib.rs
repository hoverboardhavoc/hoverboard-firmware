//! Hoverboard input conditioning: discrete-line debounce, combo/edge derivation, the rider-present
//! foot-pad field, and the analog throttle filter. A pure producer of shared state, owning no
//! actuator and no hardware. The caller samples the GPIO line levels, the pad levels, and the raw
//! ADC throttle word; this crate turns them into debounced flags, combo flags, the 2-bit pad field,
//! and the scaled + IIR-filtered throttle.
//!
//! Per [todo/inputs.md](../../../todo/inputs.md), this crate fixes the BEHAVIOR and the exact
//! reference CONSTANTS. Every concrete pin assignment, polarity, combo-pair membership,
//! and the throttle ADC channel is board-definition config the caller resolves; here the count of
//! debounced lines and the combo memberships are parameters, and the machine is replicated per line.
//!
//! Two rates, no shared mutable state between them (independently testable):
//! - [`debounce`] / [`combo`] / [`pad`]  run at 16 ms (every 4th scheduler tick);
//! - [`throttle`]                         runs at 4 ms (every scheduler tick).
//!
//! No-FPU: the debounce/combo/pad logic is pure integer/boolean. The throttle IIR is genuinely
//! fractional (Ka = 0.0003, Kb = 0.9997, tau ~13.3 s); software float is banned from the hot path,
//! so its carry is reproduced in Q-format (`fixed::I32F32`) and validated host-side against an f64
//! reference. The throttle runs at 4 ms, not the PWM-rate hot path, but the same Q discipline holds.
//!
//! The reference constants are preserved exactly.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod combo;
pub mod debounce;
pub mod pad;
pub mod throttle;

// Common re-exports.
pub use combo::{combined_button, ComboPair, ComboSet, ComboState};
pub use debounce::{DebounceLine, DebouncePhase, LineBank, MAX_LINES};
pub use pad::{PadBank, PadField, PAD_A_BIT, PAD_B_BIT};
pub use throttle::{
    scaled_throttle, ThrottleFilter, KA, KB, OUTPUT_BIAS, SCALE_NUM, SCALE_SHIFT,
};

#[cfg(test)]
mod tests;
