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
pub mod sine;
pub mod sixstep;

// Re-export the FOC math surface the firmware (and the hot-path layer) call.
pub use foc::{
    q15, Clarke, FocOutput, FocState, MotorParams, Q15, RotorFrontEnd, RotorState, current_from_adc,
    foc_step, lookup_sincos,
};
pub use sixstep::{PhasePermutation, SixStepState};

// ================================================================================================
// The runtime-selectable commutation-method abstraction (todo/commutation-methods.md)
// ================================================================================================

/// The per-motor commutation method, chosen at RUNTIME (not a compile-time feature, so one binary
/// carries all three). `SixStep` is the default so a board can spin its wheels before phase-current
/// sensing is validated. `repr(u8)` with explicit discriminants so it maps to the (future) link
/// opcode 0x33 / board-config field byte (this task does not implement those, only the enum).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommutationMethod {
    /// Trapezoidal hall-driven block commutation. NO current sensing. The default first-motion path.
    SixStep = 0,
    /// Open-loop sinusoidal modulation off the interpolated angle. NO current sensing.
    Sine = 1,
    /// Hall-sensored field-oriented control (the existing closed-loop path). Consumes the injected
    /// ADC currents; selected only once current sensing is trusted on the board.
    Foc = 2,
}

impl Default for CommutationMethod {
    /// The default method is six-step (the no-current-sensing first-motion path).
    fn default() -> Self {
        CommutationMethod::SixStep
    }
}

impl CommutationMethod {
    /// The raw discriminant byte (for the config field / link opcode payload).
    #[inline]
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parse a method from its discriminant byte; unknown bytes fall back to the default (`SixStep`)
    /// so an unconfigured or out-of-range value defaults to the no-current-sensing path. Mirrors the
    /// link `Opcode` from_u8 pattern.
    #[inline]
    pub const fn from_u8(b: u8) -> Self {
        match b {
            1 => CommutationMethod::Sine,
            2 => CommutationMethod::Foc,
            _ => CommutationMethod::SixStep,
        }
    }
}

/// The method-agnostic per-period output: the three final timer-count duties on the 0..ARR scale.
/// Carrying `[u16; 3]` (rather than alpha/beta or an SVPWM record) is what keeps `hot.rs` agnostic:
/// every arm produces final compare counts, written straight through `ComplementaryPwm::set_duties`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct MotorOutput {
    /// The three phase duties (CH0/CH1/CH2 compare values) on the 0..ARR scale.
    pub duties: [u16; 3],
}

/// The per-method persistent state, plus the SHARED rotor front-end (inside [`FocState`]). All three
/// arms step the same hall+commutation front-end (so switching method keeps the angle/speed history
/// continuous); six-step adds its phase permutation, FOC keeps its PI / ramp records. The FOC arm
/// also needs the injected-ADC samples, passed per call (not stored here).
#[derive(Clone, Copy, Debug)]
pub struct MethodState {
    /// The FOC state, which owns the shared [`RotorFrontEnd`] (hall debounce + commutation /
    /// interpolation) plus the q-PI and d-ramp records. Six-step and sine use only its front-end.
    pub foc: FocState,
    /// The six-step phase permutation (per-motor wiring override; default identity).
    pub sixstep: SixStepState,
}

impl MethodState {
    /// A fresh method state from the per-motor FOC params (default permutation, cleared records).
    pub fn new(params: MotorParams) -> Self {
        Self {
            foc: FocState::new(params),
            sixstep: SixStepState::new(),
        }
    }
}

/// The single dispatch entry point that replaces the direct `foc_step` call in the hot path. It
/// steps the SHARED front-end (hall debounce + sector decode + angle interpolation) and `match`es to
/// one of the three arms, returning the method-agnostic [`MotorOutput`] (three final duties).
///
/// - `raw_hall` is the three raw hall levels (0/1) for this period.
/// - `samples` are the two raw injected-ADC phase-current samples `(sample_a, sample_b)`; they are
///   consumed ONLY by the [`CommutationMethod::Foc`] arm (six-step / sine ignore them, so a board
///   without current sensing passes zeros).
/// - `demand` is the signed outer-loop drive demand.
///
/// The `Foc` arm wraps the existing [`foc_step`] VERBATIM (the FOC path stays bit-preserved: it
/// steps the front-end and runs the same Clarke/Park/PI/SVPWM math); its rich SVPWM output is mapped
/// to three duties via `duty_order` (the per-motor (base, c1, c2) -> (CH0, CH1, CH2) permutation,
/// the same one the FOC path always used). Six-step and sine step the front-end directly and produce
/// duties from the hall code / interpolated angle.
#[inline]
pub fn commutate_step(
    method: CommutationMethod,
    raw_hall: [u8; 3],
    samples: (u16, u16),
    demand: i32,
    state: &mut MethodState,
    duty_order: crate::hot::DutyOrder,
) -> MotorOutput {
    match method {
        CommutationMethod::SixStep => {
            // Step the shared front-end for the debounced hall code, then the block table.
            let rotor = state.foc.front.step(raw_hall);
            sixstep::sixstep_step(&state.sixstep, rotor.code, demand)
        }
        CommutationMethod::Sine => {
            // Step the shared front-end for the interpolated angle, then sinusoidal modulation.
            let rotor = state.foc.front.step(raw_hall);
            sine::sine_step(rotor.angle, demand)
        }
        CommutationMethod::Foc => {
            // The existing closed-loop path, verbatim (bit-preserved): foc_step steps the SAME
            // front-end and runs the full FOC math, including the injected-ADC currents.
            let out = foc_step(&mut state.foc, raw_hall, samples.0, samples.1, demand);
            MotorOutput {
                duties: duty_order(out.svpwm),
            }
        }
    }
}

#[cfg(test)]
mod tests;
