//! Hall-sensored FOC commutation inner loop for the universal hoverboard firmware.
//!
//! This is the per-motor inner loop of the control cascade (see todo/commutation.md). It runs once
//! per PWM period (~16 kHz) from the injected-ADC end-of-conversion ISR. Per cycle, per motor: read
//! the two phase currents + the three halls, decode commutation and interpolate the rotor angle,
//! run the Clarke/Park FOC current loop, apply the circular magnitude limit, inverse-Park, run
//! SVPWM, write the three duties, and re-arm the ADC-trigger compare.
//!
//! Two layers:
//! - [`foc`] the pure, host-testable, no-FPU fixed-point FOC math (the bulk). Every numeric
//!   constant is the bit-exact reference value. Q15 (+/-1.0 = +/-32767) for the FOC
//!   vector math, plain integer counts elsewhere; angle is a 16-bit wrapping integer (65536/rev).
//! - [`hot`] the thin hot-path wiring layer (compile-checked against runtime-hal, not host-run): a
//!   per-PWM-cycle `control_step` that reads the injected ADC + halls, calls [`foc`], writes the
//!   three duties via `ComplementaryPwm`, and re-arms the ADC trigger. MOE (the bridge-arm gate) is
//!   owned by the safety layer; commutation NEVER opens MOE, it only writes duties.
//!
//! The reference constants are preserved exactly. Comments avoid the
//! em-dash per the project writing style.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod foc;
pub mod hot;

// Re-export the FOC math surface the firmware (and the hot-path layer) call.
pub use foc::{
    q15, Clarke, FocState, MotorOutput, MotorParams, Q15, current_from_adc, lookup_sincos,
};

#[cfg(test)]
mod tests;
