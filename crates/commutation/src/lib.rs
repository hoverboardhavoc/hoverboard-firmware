//! Three-mode motor commutation math (`specs/commutation.md`), slice 1: the primitives.
//!
//! Pure, host-testable, no-FPU fixed-point math for the runtime-selectable commutation methods
//! (six-step / sinusoidal / FOC). **HAL-free by contract**: every hardware fact enters as a
//! function argument and leaves as data; nothing here (in any slice) configures a pin, timer,
//! PWM output, or ADC, and the output vocabulary cannot express MOE (arming is the safety
//! layer's, by construction).
//!
//! This slice carries the shared primitives: the duty scale ([`ARR`]/[`MID_RAIL`]), the angle
//! constants, the stock fixed-point rounding forms ([`foc::rnd_q15`]/[`foc::mac_q15`]/
//! [`foc::sat16`]), the 256-entry quarter-wave sine table + [`foc::lookup_sincos`], and the
//! per-phase output vocabulary ([`PhaseCmd`]/[`MotorOutput`]). Recovered from the archived
//! implementation (`archive/accumulated-build`, commit `74b7773`) per the spec's provenance
//! section; every numeric constant is the bit-exact recovered value.
//!
//! Q-format note (the spec's resolved open question): the crate's Q15 quantities are RAW `i16`
//! (+/-1.0 = +/-32767) throughout. The stock ops need exact wrap/saturate semantics the `fixed`
//! types do not express (e.g. [`foc::rnd_q15`] deliberately WRAPS mod 2^16), so a typed
//! `base::fixed::Q15` view would be inspection-only sugar with no consumer; it is dropped.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod foc;

// ================================================================================================
// The duty scale (single owner; specs/commutation.md "Output representation")
// ================================================================================================

/// The PWM period (auto-reload) on the duty scale: the stock timer contract (TIMER0
/// center-aligned, PSC 0, ARR 2250 = 16 kHz at the fleet's 72 MHz PLL clock). A crate constant
/// because the FOC SVPWM constants are baked to it; integration brings every board to this
/// contract.
pub const ARR: u16 = 2250;

/// Mid-rail duty: half the period (1125, the SVPWM centering constant 0x465). The centered
/// modulation neutral: sine/FOC phases swing about it; a mid-rail phase sources/sinks no average
/// voltage relative to the centered neutral.
pub const MID_RAIL: u16 = ARR / 2; // 1125

// ================================================================================================
// The per-phase output vocabulary (specs/commutation.md "Output representation")
// ================================================================================================

/// One phase's per-period command. `Drive` is a compare count on the `0..=ARR` scale; `Float` is
/// true high-Z (both FETs of the leg off; the integration layer maps it to the HAL's
/// channel-disable). The modes genuinely differ in posture: sine/FOC drive all three phases,
/// six-step floats its idle phase, and all-`Float` is the coast posture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseCmd {
    /// Drive the leg's complementary pair at this compare count (0..=ARR).
    Drive(u16),
    /// Float the leg (true high-Z, both outputs disabled).
    Float,
}

/// The method-agnostic per-period, per-motor output: three per-phase commands. MOE / arming is
/// deliberately not expressible here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorOutput {
    /// The per-phase commands for CH0/CH1/CH2.
    pub phases: [PhaseCmd; 3],
}

#[cfg(test)]
mod tests;
