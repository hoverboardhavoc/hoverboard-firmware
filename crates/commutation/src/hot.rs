//! The hot-path wiring layer (compile-checked against runtime-hal, NOT host-run).
//!
//! A per-PWM-cycle `control_step` that reads the injected ADC (two phase currents) + the halls,
//! calls the [`crate::foc`] math, writes the three duties via `ComplementaryPwm`'s `PwmHandle`, and
//! re-arms the ADC-trigger compare. This is the body the firmware registers via
//! `register_control_handler` (runtime-hal `irq`) to run in the injected-EOC ISR at the PWM rate.
//!
//! MOE (main-output-enable) is the ARMING gate owned by crates/state / the safety layer
//! (runtime-hal's `arming::ArmGate`, resolved from the same timer base). Commutation NEVER opens
//! MOE; it only writes duties. The per-cycle [`PwmHandle`] deliberately holds no MOE accessor, so
//! this layer cannot arm the bridge (a SAFETY invariant enforced by runtime-hal).
//!
//! Generalized to N motors: [`MotorRig`] bundles one motor's resolved per-cycle handles + its FOC
//! state. The single-motor reference values are the reference instantiation. For N motors, the
//! firmware holds an array of [`MotorRig`] and steps each in turn.

use runtime_hal::hotpath::hall::HallReader;
use runtime_hal::hotpath::{InjectedHandle, PwmHandle};

use crate::foc::{foc_step, FocState, MotorParams};

/// The ADC-trigger compare value re-armed each period (section 9.3): 0x8C9 = 2249, one below ARR,
/// at the end of the up-count low-current point so injected sampling stays synchronized.
pub const ADC_TRIGGER_COMPARE: u16 = 0x8C9; // 2249

/// Which injected-rank slots carry the two phase currents (section 8): slot 0 = phase A, slot 1 =
/// phase B (slots 2/3 are the auxiliary + battery-voltage senses the loop does not consume).
pub const PHASE_A_SLOT: usize = 0;
pub const PHASE_B_SLOT: usize = 1;

/// Which (CH1, CH2, CH3) the three SVPWM compare numbers (base, c1, c2) drive, per sector. This is
/// the one remaining phase-ordering degree of freedom (section 16), confirmed on the bench against
/// commanded vs produced field. The reference instantiation uses a direct (base, c1, c2) order; the
/// permutation is carried per motor so a board with different phase wiring overrides it.
pub type DutyOrder = fn(crate::foc::Svpwm) -> [u16; 3];

/// The reference (base, c1, c2) -> (CH1, CH2, CH3) order (identity). Overridden per motor when the
/// bench confirms a different phase wiring.
pub fn duty_order_reference(s: crate::foc::Svpwm) -> [u16; 3] {
    [s.base, s.c1, s.c2]
}

/// One motor's resolved hot-path rig: the per-cycle handles (PWM duties + trigger, injected ADC,
/// halls) plus the FOC state and the per-sector duty permutation. `PwmHandle` / `InjectedHandle` /
/// `HallReader` are all `Copy`, resolve-once runtime-hal handles (no descriptor lookup per call).
pub struct MotorRig {
    /// The complementary-PWM per-cycle handle (writes the three duties + re-arms the trigger). Holds
    /// NO MOE accessor: this rig cannot arm the bridge.
    pub pwm: PwmHandle,
    /// The injected-ADC per-cycle handle (reads the phase currents, left-aligned, raw).
    pub adc: InjectedHandle,
    /// The three-hall GPIO reader (packs the raw 3-bit code).
    pub halls: HallReader,
    /// The per-motor FOC math state.
    pub foc: FocState,
    /// The per-sector (base, c1, c2) -> (CH1, CH2, CH3) permutation.
    pub duty_order: DutyOrder,
    /// The drive-disable flag (section 9.3): when set, write all three compares = 0 (per-cycle
    /// soft-off) instead of the computed values. Distinct from the hardware MOE gate.
    pub drive_disable: bool,
}

impl MotorRig {
    /// Build a motor rig from its resolved runtime-hal handles and per-motor params. The handles are
    /// produced once at bring-up by `ComplementaryPwm::configure` / `TriggeredAdc::configure` /
    /// `HallReader::resolve` against the board definition; this constructor just bundles them.
    pub fn new(
        pwm: PwmHandle,
        adc: InjectedHandle,
        halls: HallReader,
        params: MotorParams,
    ) -> Self {
        Self {
            pwm,
            adc,
            halls,
            foc: FocState::new(params),
            duty_order: duty_order_reference,
            drive_disable: false,
        }
    }

    /// One PWM-period control step for this motor (the injected-EOC ISR body, per motor).
    ///
    /// Reads the injected phase currents + the halls, runs the FOC math, writes the three duties via
    /// the PWM handle, and re-arms the ADC-trigger compare. `demand` is the signed drive demand the
    /// outer 250 Hz loop wrote to shared RAM. Returns the FOC output (the published angle/speed and
    /// the duties) for the caller to publish to shared RAM. MOE is untouched (cannot arm).
    #[inline]
    pub fn control_step(&mut self, demand: i32) -> crate::foc::MotorOutput {
        // Read the injected phase-current samples (left-aligned, raw) and the halls.
        let inj = self.adc.read_injected();
        let sample_a = inj[PHASE_A_SLOT];
        let sample_b = inj[PHASE_B_SLOT];
        let code = self.halls.read();
        let raw = [code & 1, (code >> 1) & 1, (code >> 2) & 1];

        // The FOC math.
        let out = foc_step(&mut self.foc, raw, sample_a, sample_b, demand);

        // SVPWM duties -> (CH1, CH2, CH3), or the per-cycle soft-off (all-zero) when drive-disabled.
        let duties = if self.drive_disable {
            [0u16, 0u16, 0u16]
        } else {
            (self.duty_order)(out.svpwm)
        };

        // Write the three duties (MOE untouched: cannot energize a disarmed bridge). A duty above
        // the period is rejected by the handle; clamp to the period defensively so a transient never
        // errors the ISR (the circular limiter keeps SVPWM in the linear region in normal use).
        let period = self.pwm.period();
        let clamped = [
            duties[0].min(period),
            duties[1].min(period),
            duties[2].min(period),
        ];
        let _ = self.pwm.set_duties(clamped);

        // Re-arm the ADC-trigger compare so the next injected sample stays at the low-current point.
        let _ = self.pwm.rearm_trigger(ADC_TRIGGER_COMPARE.min(period));

        out
    }
}

/// Register the control-loop handler with runtime-hal's `irq` so the injected-EOC ISR (the ADC
/// vector) calls it at the PWM rate. The firmware supplies a `'static extern "C" fn()` that walks
/// its `MotorRig` array and calls [`MotorRig::control_step`] for each motor with that motor's
/// current demand. Re-exported so the firmware registers through this crate.
///
/// This is a thin pass-through to `runtime_hal::irq::register_control_handler`; the handler itself
/// (which owns the `'static` motor state) lives in the firmware binary, not here, because it needs
/// the concrete motor topology and the shared-RAM demand/publish words.
#[inline]
pub fn register_control_loop(handler: extern "C" fn()) {
    runtime_hal::irq::register_control_handler(handler);
}

// The hot-path wiring layer is normally only COMPILE-checked against runtime-hal (it touches real
// MMIO). Under runtime-hal's `mock` feature (enabled by this crate's dev-dependency) the handles are
// backed by a host array, so a host test can drive `control_step` end-to-end and assert the duty +
// trigger register writes land, proving the wiring composes with the real handle methods.
// Gated on `cfg(test)` only: this crate's dev-dependency pulls runtime-hal with its `mock` feature,
// so the host-array register backend (`reg::mock`) is active during `cargo test` and these run; the
// `#[cfg(test)]` module is never part of a real firmware build (where runtime-hal has no mock).
#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::MotorParams;
    use runtime_hal::descriptor::GpioPath;
    use runtime_hal::reg::{mock, Reg32};

    // TIMER0 advanced-timer base (both families); compare-register offsets from runtime-hal hotpath.
    const TIMER0_BASE: u32 = 0x4001_2C00;
    const CH0CV: u32 = 0x34;
    const CH1CV: u32 = 0x38;
    const CH2CV: u32 = 0x3C;
    const CH3CV: u32 = 0x40; // the ADC-trigger compare
    const ADC0_BASE: u32 = 0x4001_2400;
    const PERIOD: u16 = 2250;

    #[test]
    fn control_step_writes_duties_and_rearms_trigger() {
        let _g = mock::lock();
        mock::reset();

        // Resolve-once handles against the mock register space (the same constructors the runtime-hal
        // configure() bodies call). HallReader resolves three lines on GPIOA at the APB path.
        let pwm = PwmHandle::new(TIMER0_BASE, PERIOD);
        let adc = InjectedHandle::new(ADC0_BASE, 4);
        let halls = HallReader::resolve(
            GpioPath::ApbCrlCrh,
            [(0x4001_0800, 13), (0x4001_0800, 1), (0x4001_0800, 14)],
        );
        let mut rig = MotorRig::new(pwm, adc, halls, MotorParams::default());

        // One control step with zero demand (the zero vector); the FOC math runs and the duties land.
        let out = rig.control_step(0);

        // The three phase duties were written to CH0CV/CH1CV/CH2CV, on the 0..2250 scale.
        let d0 = Reg32::new(TIMER0_BASE, CH0CV).read() as u16;
        let d1 = Reg32::new(TIMER0_BASE, CH1CV).read() as u16;
        let d2 = Reg32::new(TIMER0_BASE, CH2CV).read() as u16;
        assert!(d0 <= PERIOD && d1 <= PERIOD && d2 <= PERIOD);
        // The reference duty order is (base, c1, c2) -> (CH1, CH2, CH3).
        assert_eq!(d0, out.svpwm.base.min(PERIOD));
        assert_eq!(d1, out.svpwm.c1.min(PERIOD));
        assert_eq!(d2, out.svpwm.c2.min(PERIOD));

        // The ADC-trigger compare was re-armed to 0x8C9 = 2249.
        let trig = Reg32::new(TIMER0_BASE, CH3CV).read() as u16;
        assert_eq!(trig, ADC_TRIGGER_COMPARE);
    }

    #[test]
    fn drive_disable_writes_zero_duties() {
        let _g = mock::lock();
        mock::reset();

        let pwm = PwmHandle::new(TIMER0_BASE, PERIOD);
        let adc = InjectedHandle::new(ADC0_BASE, 4);
        let halls = HallReader::resolve(GpioPath::ApbCrlCrh, [(0x4001_0800, 13); 3]);
        let mut rig = MotorRig::new(pwm, adc, halls, MotorParams::default());
        rig.drive_disable = true;

        rig.control_step(5000);

        // Per-cycle soft-off: all three compares = 0 (bridge held off via 0 % duty).
        assert_eq!(Reg32::new(TIMER0_BASE, CH0CV).read(), 0);
        assert_eq!(Reg32::new(TIMER0_BASE, CH1CV).read(), 0);
        assert_eq!(Reg32::new(TIMER0_BASE, CH2CV).read(), 0);
        // The trigger is still re-armed (sampling stays synchronized even when soft-off).
        assert_eq!(
            Reg32::new(TIMER0_BASE, CH3CV).read() as u16,
            ADC_TRIGGER_COMPARE
        );
    }
}
