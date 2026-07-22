//! Three-mode motor commutation math (`specs/commutation.md`), slice 1: the primitives.
//!
//! Pure, host-testable, no-FPU fixed-point math for the runtime-selectable commutation methods
//! (six-step / sinusoidal / FOC). **HAL-free by contract**: every hardware fact enters as a
//! function argument and leaves as data; nothing here (in any slice) configures a pin, timer,
//! PWM output, or ADC, and the output vocabulary cannot express MOE (arming is the safety
//! layer's, by construction).
//!
//! Layout: [`foc`] carries the shared primitives (the stock fixed-point rounding forms, the sine
//! table + lookup), the shared hall front-end ([`foc::RotorFrontEnd`]) every mode steps once per
//! period, and the recovered FOC blocks; [`sixstep`] and [`sine`] are the open-loop arms; the
//! dispatch is [`Commutator`] over [`MethodState`]. Recovered from the archived implementation
//! (`archive/accumulated-build`, commit `74b7773`) per the spec's provenance section, except the
//! six-step arm, whose contract is the silicon-proven example choreography; every numeric
//! constant is the bit-exact recovered value.
//!
//! Q-format note (the spec's resolved open question): the crate's Q15 quantities are RAW `i16`
//! (+/-1.0 = +/-32767) throughout. The stock ops need exact wrap/saturate semantics the `fixed`
//! types do not express (e.g. [`foc::rnd_q15`] deliberately WRAPS mod 2^16), so a typed
//! `base::fixed::Q15` view would be inspection-only sugar with no consumer; it is dropped.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod foc;
pub mod sine;
pub mod sixstep;

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

impl MotorOutput {
    /// Lower the per-phase commands to the two hardware register writes the integration hot path
    /// applies (`specs/motor-integration.md`, "The per-period hot path"): the compare counts and
    /// the per-channel output enables. `Drive(n) -> (n, true)` (drive the leg's complementary pair
    /// at compare `n`); `Float -> (0, false)` (channel disabled = true high-Z, the compare value
    /// unused so 0). Pure and HAL-free; the integration layer feeds the returned duties to
    /// `set_duties` (range-checked first) and the enables to `set_channel_outputs` in that order.
    #[inline]
    pub fn to_duties_enables(&self) -> ([u16; 3], [bool; 3]) {
        let mut duties = [0u16; 3];
        let mut enables = [false; 3];
        for (i, phase) in self.phases.iter().enumerate() {
            match phase {
                PhaseCmd::Drive(n) => {
                    duties[i] = *n;
                    enables[i] = true;
                }
                PhaseCmd::Float => {
                    duties[i] = 0;
                    enables[i] = false;
                }
            }
        }
        (duties, enables)
    }
}

// ================================================================================================
// The runtime-selectable mode model (specs/commutation.md "The mode model")
// ================================================================================================

/// The per-motor commutation method, chosen at RUNTIME (one binary carries all three).
/// `repr(u8)`: the discriminant IS the `MOTOR_METHOD` store field value (id 0x21, default 0).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum CommutationMethod {
    /// Trapezoidal hall-driven block commutation. NO current sensing. The default first-motion
    /// path (and the `MOTOR_METHOD` field's default 0).
    #[default]
    SixStep = 0,
    /// Open-loop sinusoidal modulation off the interpolated angle. NO current sensing.
    Sine = 1,
    /// Hall-sensored field-oriented control (closed-loop). Consumes the injected-ADC currents;
    /// selectable only where current sensing exists and the offset cal passed (the integration
    /// validator's rule).
    Foc = 2,
}

impl CommutationMethod {
    /// The raw discriminant byte (the `MOTOR_METHOD` field / link payload value).
    #[inline]
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parse a method from its discriminant byte; unknown bytes fall back to `SixStep` so an
    /// unconfigured or out-of-range value selects the no-current-sensing path.
    #[inline]
    pub const fn from_u8(b: u8) -> Self {
        match b {
            1 => CommutationMethod::Sine,
            2 => CommutationMethod::Foc,
            _ => CommutationMethod::SixStep,
        }
    }
}

/// The per-mode records, one variant per method, so the selected method and its records cannot
/// disagree. Sine is stateless beyond the shared front-end; six-step carries its decode config;
/// FOC carries [`foc::FocState`] (uninhabited until slice 4, so a `Foc` value cannot exist yet
/// and the dispatch's FOC arm is statically unreachable).
#[derive(Clone, Copy, Debug)]
pub enum MethodState {
    /// Six-step records (the decode config: direction + bench-swept align offset).
    SixStep(sixstep::SixStepState),
    /// Sine has no per-mode records.
    Sine,
    /// FOC records (slice 4; the type is uninhabited until then).
    Foc(foc::FocState),
}

impl MethodState {
    /// The method these records belong to.
    #[inline]
    pub fn method(&self) -> CommutationMethod {
        match self {
            MethodState::SixStep(_) => CommutationMethod::SixStep,
            MethodState::Sine => CommutationMethod::Sine,
            MethodState::Foc(_) => CommutationMethod::Foc,
        }
    }
}

/// The per-motor dispatch: the SHARED rotor front-end (which survives a method switch) plus the
/// per-mode records (which do not). The two are one struct so the reset seam is structural:
/// [`Commutator::switch_method`] replaces the records wholesale with caller-built fresh ones and
/// cannot touch the front-end.
#[derive(Clone, Copy, Debug)]
pub struct Commutator {
    front: foc::RotorFrontEnd,
    method: MethodState,
}

impl Commutator {
    /// A fresh commutator: a cleared front-end plus the initial per-mode records.
    pub fn new(method: MethodState) -> Self {
        Self {
            front: foc::RotorFrontEnd::new(),
            method,
        }
    }

    /// The active method.
    #[inline]
    pub fn method(&self) -> CommutationMethod {
        self.method.method()
    }

    /// The method-switch reset seam (spec "The mode model"): the per-mode records are REPLACED
    /// with the caller's fresh ones; the shared front-end is NOT touched (angle/speed history
    /// stays continuous, mode-independent truth about the rotor). Switching happens only while
    /// disarmed (the integration layer's rule; a method change is a config write).
    pub fn switch_method(&mut self, fresh: MethodState) {
        self.method = fresh;
    }

    /// One per-PWM-period step: step the shared front-end once, then dispatch to the active arm.
    ///
    /// - `raw_hall`: the three raw hall levels (0/1) this period.
    /// - `samples`: the two raw injected-ADC phase-current samples `(a, b)`; consumed ONLY by the
    ///   FOC arm (slice 4). Six-step / sine ignore them (a board without current sensing passes
    ///   zeros).
    /// - `demand`: the signed drive demand (the single 250 Hz hand-off word).
    pub fn step(&mut self, raw_hall: [u8; 3], samples: (u16, u16), demand: i32) -> MotorOutput {
        let rotor = self.front.step(raw_hall);
        match &mut self.method {
            MethodState::SixStep(st) => sixstep::sixstep_step(st, rotor.code, demand),
            MethodState::Sine => sine::sine_step(rotor.angle, demand),
            MethodState::Foc(st) => foc::foc_step(st, rotor, samples.0, samples.1, demand),
        }
    }
}

#[cfg(test)]
mod tests;
