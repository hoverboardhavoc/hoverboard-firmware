//! The top-level mode state machine (state.md §3, §4).
//!
//! Driven once per 4 ms tick by the 250 Hz cooperative scheduler. One pass evaluates the current
//! mode byte and performs at most one transition plus, for INIT and SHUTDOWN, the associated
//! one-shot side effects. It is a plain switch on the mode byte; an unrecognized value is a no-op
//! (do not crash, do not enable the bridge).
//!
//! The mode byte is the SOLE output of the state machine to the rest of the system. The machine
//! does not call indicators, beeps, the balance machine, or peers; they read the mode byte and the
//! fault flags. This machine is also the sole software owner of the MOE gate (see `crate::moe`).
//!
//! Fault A, Fault B, and the OFF-only inhibit are LEVEL-SENSITIVE inputs sampled fresh on every
//! tick (state.md §3.4). The machine does not latch, edge-detect, or remember them; any stickiness
//! lives in the producer (e.g. the `crate::fault::FaultLatch`). The power-request flag is likewise
//! level-sensitive, except the SHUTDOWN pass additionally clears it so a momentary off request
//! resolves cleanly to OFF.

use crate::moe::MoeGate;

/// The system mode byte. Numeric values are FIXED and part of the observable contract (telemetry,
/// indicators, peers). Repr is `u8` so the byte maps verbatim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// MCU/scheduler/comms/indication alive; motor path not brought up; bridge disabled (MOE
    /// clear). Waiting for a power request with no fault asserted. Reset-settled resting state.
    Off = 0,
    /// One-shot power-on enable: bring up the motor path and set MOE (§4.2).
    Init = 1,
    /// Transitional; promotes to RUN on the next tick. Powered and indicating; torque still zero.
    Ready = 2,
    /// Powered and balancing-capable. The balance machine owns torque engagement within RUN.
    Run = 3,
    /// One-shot safe-down: clear MOE, zero/force outputs to safe, power down indicators, clear the
    /// power-request flag, then drop to OFF (§4.3).
    Shutdown = 4,
}

impl Mode {
    /// The mode byte. The sole output of the machine.
    pub const fn as_byte(self) -> u8 {
        self as u8
    }
}

/// The level-sensitive inputs sampled by the mode machine on a tick (state.md §3.3, §3.4). Every
/// field is read-only to the machine and may be driven to 0 or 1 by the producers on any tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ModeInputs {
    /// Single global on/off intent: `true` = on requested, `false` = off requested. Level-sensitive.
    /// On an originating node it comes from rider inputs; on a mirroring node it is copied over the
    /// link. The SHUTDOWN pass clears it (see [`ModeMachine::tick`]).
    pub power_request: bool,
    /// Fault A: acts at OFF (blocks OFF -> INIT) and during RUN (forces RUN -> SHUTDOWN). Sampled
    /// as a level; aggregate of the node's sticky fault producers (e.g. over-current / stall
    /// latches, link-loss).
    pub fault_a: bool,
    /// Fault B: same gating role as Fault A. Two flags so distinct producer groups map cleanly.
    pub fault_b: bool,
    /// OFF-only inhibit: blocks OFF -> INIT only; NOT consulted in RUN. The boot-not-ready /
    /// startup-and-calibration condition, and the arming-not-yet-`Armed` condition from safety.md
    /// (a Disarmed or still-Commissioning board holds OFF -> INIT).
    pub off_inhibit: bool,
}

/// The result of one INIT pass: the per-motor bring-up that the firmware must enact (state.md
/// §4.2). The mode machine sets the MOE gate(s) and signals that the firmware should run the
/// hardware bring-up. The current-offset zero-calibration range check is the firmware's; if it
/// fails the firmware raises a fault (init-failure) so the very next RUN gate trips to SHUTDOWN, or
/// refuses the READY promotion. This crate does not perform the hardware steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InitAction {
    /// True on the tick the INIT bring-up runs. The firmware performs §4.2 steps 1-6 per motor
    /// (aux timer clear, link DMA re-arm, ADC sequence, set MOE, complementary outputs + dead-time,
    /// enable IRQs) and the offset range check.
    pub run_bringup: bool,
}

/// The result of one SHUTDOWN pass: the per-motor safe-down the firmware must enact (state.md
/// §4.3). After this the bridge is off by two independent means (MOE cleared here, and the firmware
/// forcing the timer output off) and the duties are zeroed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShutdownAction {
    /// True on the tick the safe-down runs. The firmware performs §4.3 steps 1-5 per motor (force
    /// main output off, clear MOE, zero SVPWM and run one update, force timer outputs to safe,
    /// power down indicators). This crate has already cleared the MOE gate decision.
    pub run_safedown: bool,
}

/// The per-tick outcome the firmware consumes: the (possibly advanced) mode byte plus the one-shot
/// INIT / SHUTDOWN actions for this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickOutcome {
    /// The mode byte AFTER this tick's transition. The sole output.
    pub mode: Mode,
    /// Set when this tick was the INIT pass (run the bring-up).
    pub init: Option<InitAction>,
    /// Set when this tick was the SHUTDOWN pass (run the safe-down).
    pub shutdown: Option<ShutdownAction>,
}

/// The mode state machine with `N` per-motor MOE gates (state.md §9: one MOE per motor / advanced
/// timer, one shared mode byte). `N = 1` for F130C8 / F103C8; `N = 2` for the RCT6 (TIMER0 +
/// TIMER7). The fault-latch units (`crate::fault::FaultLatch`) are owned alongside this by the
/// firmware, one per motor; their latches feed Fault A/B into [`ModeInputs`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeMachine<const N: usize> {
    mode: Mode,
    moe: [MoeGate; N],
    /// Local copy of the power-request flag the machine owns clearing of on SHUTDOWN. The firmware
    /// seeds it each tick from [`ModeInputs::power_request`] via [`ModeMachine::tick`], and reads it
    /// back via [`ModeMachine::power_request`] after a SHUTDOWN pass cleared it.
    power_request: bool,
}

impl<const N: usize> ModeMachine<N> {
    /// A fresh machine in OFF with all MOE gates clear and no power request. The reset-settled
    /// resting state; after a watchdog reset the boot path re-enters here and the mode byte starts
    /// at OFF.
    pub const fn new() -> Self {
        ModeMachine {
            mode: Mode::Off,
            moe: [MoeGate::new(); N],
            power_request: false,
        }
    }

    /// The current mode byte enum. The sole output of the machine.
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// The current mode byte value (0..=4).
    pub const fn mode_byte(&self) -> u8 {
        self.mode.as_byte()
    }

    /// The MOE-allowed decision for motor `i`: `true` = MOE may be active, `false` = MOE must be
    /// clear. The firmware applies this to runtime-hal's `ComplementaryPwm` MOE. Out-of-range `i`
    /// returns `false` (defensive: never report a non-existent motor as armed).
    pub fn moe_allowed(&self, i: usize) -> bool {
        self.moe.get(i).map(|g| g.allowed()).unwrap_or(false)
    }

    /// True if any motor's MOE gate is allowed active. Convenience for the firmware's "is the bridge
    /// armed at all" check.
    pub fn any_moe_allowed(&self) -> bool {
        self.moe.iter().any(|g| g.allowed())
    }

    /// The machine's owned power-request flag, after any SHUTDOWN-pass clear. The firmware should
    /// fold this back into the shared power-request source so a momentary off request resolves to
    /// OFF (state.md §3.4, §4.3 step 6).
    pub const fn power_request(&self) -> bool {
        self.power_request
    }

    /// One 4 ms pass of the mode machine (state.md §3.2). Evaluates `inputs`, performs at most one
    /// transition, runs the one-shot INIT / SHUTDOWN side effects on the MOE gate(s), and returns
    /// the [`TickOutcome`]. The `inputs.power_request` level seeds the machine's owned copy each
    /// tick BEFORE evaluation, so a held level is honored and the SHUTDOWN clear takes effect.
    ///
    /// Gate assignment (exact, §3.2):
    /// - OFF -> INIT requires ALL of: power_request on, Fault A clear, Fault B clear, OFF-inhibit
    ///   clear. Any one asserted holds OFF.
    /// - INIT -> READY: unconditional (after running the bring-up + setting MOE).
    /// - READY -> RUN: unconditional.
    /// - RUN -> SHUTDOWN fires on ANY of: Fault A asserted, Fault B asserted, power_request off.
    ///   (The OFF-only inhibit is NOT consulted in RUN.)
    /// - SHUTDOWN -> OFF: unconditional (after the safe-down + clearing MOE + clearing the request).
    pub fn tick(&mut self, inputs: &ModeInputs) -> TickOutcome {
        // The power-request flag is a level: refresh the owned copy from the input each tick. The
        // SHUTDOWN pass below may then clear it; the firmware reads it back via power_request().
        self.power_request = inputs.power_request;

        let mut init = None;
        let mut shutdown = None;

        match self.mode {
            Mode::Off => {
                // §3.2: housekeeping convert runs first (firmware-side current-sense poll). Then the
                // OFF gate. Any of Fault A, Fault B, OFF-inhibit, or a non-on request holds OFF.
                let blocked = inputs.fault_a || inputs.fault_b || inputs.off_inhibit;
                if !blocked && self.power_request {
                    self.mode = Mode::Init;
                }
            }
            Mode::Init => {
                // One-shot power-on enable (§4.2). Set MOE here, and only here, per motor. The
                // firmware runs the hardware bring-up and the current-offset range check.
                for g in self.moe.iter_mut() {
                    g.set_at_init();
                }
                init = Some(InitAction { run_bringup: true });
                self.mode = Mode::Ready;
            }
            Mode::Ready => {
                // Pure one-tick pass-through.
                self.mode = Mode::Run;
            }
            Mode::Run => {
                // §3.2: leave RUN on any of Fault A, Fault B, or power-request off. OFF-inhibit is
                // NOT consulted in RUN.
                if inputs.fault_a || inputs.fault_b || !self.power_request {
                    self.mode = Mode::Shutdown;
                }
            }
            Mode::Shutdown => {
                // One-shot safe-down (§4.3). Clear MOE per motor, then drop to OFF and clear the
                // power-request flag. Idempotent whether entered by fault or by an off request.
                for g in self.moe.iter_mut() {
                    g.clear();
                }
                shutdown = Some(ShutdownAction { run_safedown: true });
                self.mode = Mode::Off;
                self.power_request = false;
            }
        }

        TickOutcome {
            mode: self.mode,
            init,
            shutdown,
        }
    }
}

impl<const N: usize> Default for ModeMachine<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req_on() -> ModeInputs {
        ModeInputs {
            power_request: true,
            ..Default::default()
        }
    }

    #[test]
    fn mode_byte_values_are_fixed() {
        assert_eq!(Mode::Off.as_byte(), 0);
        assert_eq!(Mode::Init.as_byte(), 1);
        assert_eq!(Mode::Ready.as_byte(), 2);
        assert_eq!(Mode::Run.as_byte(), 3);
        assert_eq!(Mode::Shutdown.as_byte(), 4);
    }

    #[test]
    fn clean_power_on_off_init_ready_run_in_three_ticks() {
        let mut m: ModeMachine<1> = ModeMachine::new();
        assert_eq!(m.mode(), Mode::Off);
        assert!(!m.moe_allowed(0), "OFF holds MOE clear");

        // Tick 1: OFF -> INIT (request on, all clear). The INIT pass (bring-up + MOE set) runs on
        // the next tick, when the mode byte is current at INIT.
        let o = m.tick(&req_on());
        assert_eq!(o.mode, Mode::Init);
        assert!(o.init.is_none(), "bring-up runs on the INIT pass, not the OFF->INIT tick");
        assert!(!m.moe_allowed(0), "MOE not yet set on the OFF->INIT tick");

        // Tick 2: INIT pass runs the bring-up and sets MOE, then INIT -> READY.
        let o = m.tick(&req_on());
        assert_eq!(o.mode, Mode::Ready);
        assert!(o.init.is_some(), "INIT bring-up runs on the INIT pass");
        assert!(o.init.unwrap().run_bringup);
        assert!(m.moe_allowed(0), "MOE set on the INIT pass");

        // Tick 3: READY -> RUN (unconditional). RUN reached on the third tick.
        let o = m.tick(&req_on());
        assert_eq!(o.mode, Mode::Run);
        assert!(m.moe_allowed(0), "MOE still allowed in RUN");
    }

    fn drive_to_run(m: &mut ModeMachine<1>) {
        m.tick(&req_on()); // INIT
        m.tick(&req_on()); // READY
        let o = m.tick(&req_on()); // RUN
        assert_eq!(o.mode, Mode::Run);
    }

    #[test]
    fn off_init_blocked_by_each_gate_input() {
        // Fault A holds OFF.
        let mut m: ModeMachine<1> = ModeMachine::new();
        let mut inp = req_on();
        inp.fault_a = true;
        assert_eq!(m.tick(&inp).mode, Mode::Off);
        assert!(!m.moe_allowed(0));

        // Fault B holds OFF.
        let mut m: ModeMachine<1> = ModeMachine::new();
        let mut inp = req_on();
        inp.fault_b = true;
        assert_eq!(m.tick(&inp).mode, Mode::Off);

        // OFF-only inhibit holds OFF (boot-not-ready / not-yet-Armed).
        let mut m: ModeMachine<1> = ModeMachine::new();
        let mut inp = req_on();
        inp.off_inhibit = true;
        assert_eq!(m.tick(&inp).mode, Mode::Off);

        // Power request off holds OFF.
        let mut m: ModeMachine<1> = ModeMachine::new();
        assert_eq!(m.tick(&ModeInputs::default()).mode, Mode::Off);
    }

    #[test]
    fn run_to_shutdown_on_fault_a_clears_moe_and_request() {
        let mut m: ModeMachine<1> = ModeMachine::new();
        drive_to_run(&mut m);

        // Fault A asserts in RUN: RUN -> SHUTDOWN. The safe-down (MOE clear) runs on the SHUTDOWN
        // pass, the next tick.
        let mut inp = req_on();
        inp.fault_a = true;
        let o = m.tick(&inp);
        assert_eq!(o.mode, Mode::Shutdown);
        assert!(o.shutdown.is_none(), "safe-down runs on the SHUTDOWN pass, not the RUN->SHUTDOWN tick");

        // SHUTDOWN pass: safe-down runs (MOE cleared), SHUTDOWN -> OFF, power-request cleared.
        let o = m.tick(&inp);
        assert_eq!(o.mode, Mode::Off);
        assert!(o.shutdown.is_some(), "safe-down runs");
        assert!(!m.moe_allowed(0), "MOE cleared on the SHUTDOWN pass");
        assert!(!m.power_request(), "power-request cleared on SHUTDOWN");
    }

    #[test]
    fn run_to_shutdown_on_power_off_routes_through_shutdown() {
        let mut m: ModeMachine<1> = ModeMachine::new();
        drive_to_run(&mut m);

        // Power request goes off mid-RUN. Must route through SHUTDOWN (full safe-down), not a direct
        // jump to OFF.
        let o = m.tick(&ModeInputs::default());
        assert_eq!(o.mode, Mode::Shutdown);
        // MOE is cleared on the SHUTDOWN pass (the next tick), not the RUN->SHUTDOWN tick.
        let o = m.tick(&ModeInputs::default());
        assert_eq!(o.mode, Mode::Off);
        assert!(!m.moe_allowed(0), "MOE cleared on the SHUTDOWN pass");
    }

    #[test]
    fn off_inhibit_not_consulted_in_run() {
        let mut m: ModeMachine<1> = ModeMachine::new();
        drive_to_run(&mut m);
        // OFF-only inhibit asserted in RUN must NOT cause SHUTDOWN.
        let mut inp = req_on();
        inp.off_inhibit = true;
        let o = m.tick(&inp);
        assert_eq!(o.mode, Mode::Run, "OFF-inhibit is not a runtime trip");
    }

    #[test]
    fn fault_raised_then_lowered_re_enters_init() {
        // Fault asserts in RUN -> SHUTDOWN -> OFF (request cleared by SHUTDOWN). Rider requests power
        // again while fault still asserted -> held OFF. Producer lowers fault -> next tick OFF -> INIT.
        let mut m: ModeMachine<1> = ModeMachine::new();
        drive_to_run(&mut m);

        let mut faulted = req_on();
        faulted.fault_a = true;
        assert_eq!(m.tick(&faulted).mode, Mode::Shutdown);
        assert_eq!(m.tick(&faulted).mode, Mode::Off);

        // Rider requests power again, fault still asserted -> held OFF.
        let mut req_with_fault = req_on();
        req_with_fault.fault_a = true;
        assert_eq!(m.tick(&req_with_fault).mode, Mode::Off);
        assert_eq!(m.tick(&req_with_fault).mode, Mode::Off);

        // Producer lowers the fault while request still held -> OFF -> INIT next tick.
        let o = m.tick(&req_on());
        assert_eq!(o.mode, Mode::Init, "re-enters INIT once all gates read clear");
        // The new INIT pass (next tick) re-enables MOE.
        m.tick(&req_on());
        assert!(m.moe_allowed(0), "MOE re-enabled on the new INIT pass");
    }

    #[test]
    fn fault_and_power_off_both_true_single_shutdown_pass() {
        let mut m: ModeMachine<1> = ModeMachine::new();
        drive_to_run(&mut m);
        // Power off AND a fault both true.
        let inp = ModeInputs {
            power_request: false,
            fault_a: true,
            ..Default::default()
        };
        let o = m.tick(&inp);
        assert_eq!(o.mode, Mode::Shutdown);
        // Single safe-down, idempotent.
        let o = m.tick(&inp);
        assert_eq!(o.mode, Mode::Off);
    }

    #[test]
    fn re_power_after_non_sticky_stop_re_runs_init() {
        let mut m: ModeMachine<1> = ModeMachine::new();
        drive_to_run(&mut m);
        // Stop only because power went off (no fault).
        m.tick(&ModeInputs::default()); // SHUTDOWN
        assert_eq!(m.tick(&ModeInputs::default()).mode, Mode::Off);
        assert!(!m.moe_allowed(0), "MOE not left enabled across the OFF dwell");
        // Fresh on request re-runs INIT cleanly; bring-up repeated, MOE re-enabled on the INIT pass.
        let o = m.tick(&req_on()); // OFF -> INIT
        assert_eq!(o.mode, Mode::Init);
        assert!(!m.moe_allowed(0), "not yet set on the OFF->INIT tick");
        let o = m.tick(&req_on()); // INIT pass: bring-up + MOE, -> READY
        assert!(o.init.unwrap().run_bringup, "bring-up repeated every power-on");
        assert!(m.moe_allowed(0));
    }

    #[test]
    fn moe_never_enabled_across_off_dwell() {
        // MOE is clear in OFF before any INIT, and clear again after SHUTDOWN -> OFF.
        let mut m: ModeMachine<1> = ModeMachine::new();
        assert!(!m.moe_allowed(0));
        // Hold OFF for several ticks with no request.
        for _ in 0..5 {
            assert_eq!(m.tick(&ModeInputs::default()).mode, Mode::Off);
            assert!(!m.moe_allowed(0));
        }
    }

    #[test]
    fn n_motor_all_moe_gates_set_at_init_and_cleared_on_shutdown() {
        let mut m: ModeMachine<2> = ModeMachine::new();
        let inp = req_on();
        m.tick(&inp); // OFF -> INIT
        m.tick(&inp); // INIT pass (sets both MOE) -> READY
        assert!(m.moe_allowed(0) && m.moe_allowed(1), "both motors MOE set on the INIT pass");
        m.tick(&inp); // READY -> RUN
        // Fault on one motor (via aggregated Fault A) safes ALL motors.
        let mut f = req_on();
        f.fault_a = true;
        m.tick(&f); // RUN -> SHUTDOWN
        m.tick(&f); // SHUTDOWN pass clears both MOE -> OFF
        assert!(!m.moe_allowed(0) && !m.moe_allowed(1), "all motors safed");
    }

    #[test]
    fn moe_allowed_out_of_range_is_false() {
        let m: ModeMachine<1> = ModeMachine::new();
        assert!(!m.moe_allowed(5), "non-existent motor never reported armed");
    }

    #[test]
    fn integrates_with_fault_latch_as_fault_source() {
        use crate::fault::{FaultLatch, CODE_OVERCURRENT};
        // The fault-latch unit's sticky latch feeds Fault A. Demonstrates the level-sensitive
        // sampling: the mode machine follows the latch.
        let mut latch = FaultLatch::new();
        latch.running_enable = 1;
        let mut m: ModeMachine<1> = ModeMachine::new();

        // Reach RUN with the latch healthy.
        latch.a_substate = 3;
        latch.b_motion = 100;
        for _ in 0..3 {
            latch.tick();
            let inp = ModeInputs {
                power_request: true,
                fault_a: latch.is_latched(),
                ..Default::default()
            };
            m.tick(&inp);
        }
        assert_eq!(m.mode(), Mode::Run);

        // Over-current arrives: latch fires, Fault A asserts, RUN -> SHUTDOWN.
        latch.fault_code = CODE_OVERCURRENT;
        latch.tick();
        assert!(latch.is_latched());
        let inp = ModeInputs {
            power_request: true,
            fault_a: latch.is_latched(),
            ..Default::default()
        };
        let o = m.tick(&inp); // RUN -> SHUTDOWN
        assert_eq!(o.mode, Mode::Shutdown);
        let o = m.tick(&inp); // SHUTDOWN pass clears MOE -> OFF
        assert_eq!(o.mode, Mode::Off);
        assert!(!m.moe_allowed(0), "fault clears MOE on the SHUTDOWN pass");
    }
}
