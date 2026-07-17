//! The orchestrator core (`specs/integration.md`): the pure heart of the integrated firmware.
//!
//! Realizes the spec's "Execution model" state shell, "The 250 Hz pipeline" steps 1-6 and 9, and
//! "The input task", as plain functions over [`OrchestratorState`] plus already-sampled inputs.
//! No peripheral access, no statics, no scheduler: the firmware shell (slice 7) owns the
//! `Scheduler` static, the SysTick ISR, hardware sampling, and the `CTRL_OBS` RAM block; this
//! crate owns everything between the sampled inputs and the decided outputs, which is what makes
//! the pipeline host-testable end to end.
//!
//! - [`control_task`]: one 250 Hz pass. IMU sample in (or the zero sample; a failing read does
//!   not fault this round, it reads zero and is observable), Mahony attitude, the link-inbox
//!   snapshot + staleness ages (`specs/link-control.md`, "Supervision"), the per-motor fault
//!   latches, input assembly + the mode machine (`off_inhibit = false` this round), and the R3
//!   enactment seam: the `TickOutcome` init/shutdown records and per-motor `moe_allowed` leave as
//!   DATA in [`ControlOutput`]; nothing hardware happens here. Pipeline steps 7 (control
//!   dispatch) and 8 (cyclic TX) are slice 6: their inputs (the inbox words, the drive-staleness
//!   predicate, rider/lockdown levels, the attitude output) are typed and ready, and no stand-in
//!   output is fabricated for them.
//! - [`input_task`]: one 16 ms pass. The power-button debounce ([`inputs::LineBank`], active-low,
//!   two-call press / one-call release), the foot pads ([`inputs::PadBank`]) into the rider
//!   level, and the [`inputs::ThrottleFilter`] over the remote `INPUTS` throttle word (no local
//!   ADC this round). `power_request` = debounced button OR the `INPUTS` mirror bit (level
//!   semantics, both producers equivalent).
//! - [`LinkInbox`]: the latest-wins words the firmware loop's delivered-PDU hand-back routes in
//!   (`linkctl::Payload`), plus the supervision state: the cyclic/drive age counters, the
//!   `comms_loss` level (never-seen-a-peer exempt: single-board operation is legitimate), and
//!   the `stop_all` latch with its OFF-dwell clear.
//!
//! `crates/control` / `crates/state` stay link-agnostic (`specs/link-control.md`, "Crate
//! placement"): `linkctl` types stop at this crate's inbox; the mode machine consumes plain
//! levels.
//!
//! `no_std`; host tests in `tests.rs` link `std` via the host target.

#![no_std]

#[cfg(test)]
extern crate std;

use base::fixed::Fix;
use linkctl::{CyclicState, DriveCmd, Payload, CYCLIC_TIMEOUT_TICKS, DRIVE_TIMEOUT_TICKS};
use state::{FaultLatch, InitAction, ModeInputs, ModeMachine, ShutdownAction};

/// The per-motor breadth of the orchestrator state: the control block's dual-motor shape
/// (`specs/control.md` (e); one MOE gate + one fault latch per advanced timer). Single-motor
/// boards simply never enact motor 1's records (the enact seam carries both; the hardware layer
/// applies the ones its plan configures).
pub const N_MOTORS: usize = 2;

// ---------------------------------------------------------------------------------------------
// The link inbox: latest-wins words + supervision (`specs/link-control.md`).
// ---------------------------------------------------------------------------------------------

/// The latest-wins link words and their supervision state. The firmware loop routes each
/// delivered control-block PDU through `linkctl::decode` and hands the payload to
/// [`LinkInbox::accept`]; the 250 Hz pipeline bumps the ages ([`LinkInbox::tick_ages`], step 3)
/// and consumes the levels. All four families are best-effort / latest-wins: no queue, one slot
/// per family.
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkInbox {
    /// The latest accepted peer `CYCLIC_STATE` (`None` = never seen a peer since boot).
    peer: Option<CyclicState>,
    /// Ticks since the last accepted cyclic (saturating; meaningless while `peer` is `None`).
    cyclic_age: u32,
    /// The latest accepted `DRIVE_CMD` (`None` = never received).
    drive: Option<DriveCmd>,
    /// Ticks since the last accepted drive command.
    drive_age: u32,
    /// The latest accepted remote `INPUTS` mirror (`None` = never received).
    remote: Option<linkctl::Inputs>,
    /// The `FAULT action = STOP_ALL` latch: set on receipt, feeds `ModeInputs.fault_a`, cleared
    /// only by the mode machine passing through OFF (the OFF-dwell clear, applied by
    /// [`control_task`]).
    stop_all: bool,
}

impl LinkInbox {
    /// An empty inbox: no peer, no drive, no mirror, latch clear.
    pub const fn new() -> Self {
        LinkInbox {
            peer: None,
            cyclic_age: 0,
            drive: None,
            drive_age: 0,
            remote: None,
            stop_all: false,
        }
    }

    /// Accept one decoded control-block payload (latest-wins). `CYCLIC_STATE` and `DRIVE_CMD`
    /// reset their staleness ages; a `FAULT` with `action = STOP_ALL` sets the latch (a
    /// notify-only `FAULT` carries no consumer this round: the level lives in
    /// `CYCLIC_STATE.fault`, and the controller-facing path is deferred to telemetry).
    pub fn accept(&mut self, payload: Payload) {
        match payload {
            Payload::CyclicState(c) => {
                self.peer = Some(c);
                self.cyclic_age = 0;
            }
            Payload::DriveCmd(d) => {
                self.drive = Some(d);
                self.drive_age = 0;
            }
            Payload::Inputs(i) => self.remote = Some(i),
            Payload::Fault(f) => {
                if f.stop_all() {
                    self.stop_all = true;
                }
            }
        }
    }

    /// One 250 Hz age bump (pipeline step 3). Ages only advance once the family has been seen:
    /// a board that never saw a peer does not accrue toward `comms_loss` (the never-seen
    /// exemption).
    fn tick_ages(&mut self) {
        if self.peer.is_some() {
            self.cyclic_age = self.cyclic_age.saturating_add(1);
        }
        if self.drive.is_some() {
            self.drive_age = self.drive_age.saturating_add(1);
        }
    }

    /// The peer-staleness level (`specs/link-control.md`, "Supervision"): asserted while the age
    /// of the last accepted cyclic exceeds [`CYCLIC_TIMEOUT_TICKS`]. Level-sensitive (fresh
    /// cyclic clears it); never asserted before the first cyclic (single-board operation is
    /// legitimate).
    pub fn comms_loss(&self) -> bool {
        self.peer.is_some() && self.cyclic_age > CYCLIC_TIMEOUT_TICKS
    }

    /// The drive-staleness predicate: true when no `DRIVE_CMD` is fresh within
    /// [`DRIVE_TIMEOUT_TICKS`] (or none was ever received). The slice-6 throttle-reference
    /// producer decays to neutral on this (a reference-zeroing, not a fault).
    pub fn drive_stale(&self) -> bool {
        match self.drive {
            None => true,
            Some(_) => self.drive_age > DRIVE_TIMEOUT_TICKS,
        }
    }

    /// The latest drive command, `None` when never received. Staleness is the caller's to apply
    /// via [`LinkInbox::drive_stale`].
    pub fn drive(&self) -> Option<DriveCmd> {
        self.drive
    }

    /// The latest peer cyclic words, `None` when never seen.
    pub fn peer(&self) -> Option<CyclicState> {
        self.peer
    }

    /// The peer lockdown level (`CYCLIC_STATE.flags` bit 7, latest-wins): the slice-6 engagement
    /// machine treats it as a gating fault (sub-state forced to 0, torque 0) while asserted.
    pub fn peer_lockdown(&self) -> bool {
        self.peer.map(|p| p.lockdown()).unwrap_or(false)
    }

    /// The remote power-request mirror bit (`INPUTS.buttons` bit 0, level).
    pub fn remote_power_request(&self) -> bool {
        self.remote.map(|i| i.power_request()).unwrap_or(false)
    }

    /// The remote rider-present mirror bit (`INPUTS.rider` bit 0, level).
    pub fn remote_rider_present(&self) -> bool {
        self.remote.map(|i| i.rider_present()).unwrap_or(false)
    }

    /// The latest remote throttle word, `None` when no `INPUTS` mirror was ever received (the
    /// input task only steps the throttle filter once a word exists, so the one-shot IIR
    /// baseline captures a real sample, not a fabricated zero).
    pub fn remote_throttle(&self) -> Option<i16> {
        self.remote.map(|i| i.throttle)
    }

    /// The `stop_all` latch level (feeds `fault_a`).
    pub fn stop_all(&self) -> bool {
        self.stop_all
    }

    /// The cyclic age in ticks (OBS).
    pub fn cyclic_age(&self) -> u32 {
        self.cyclic_age
    }

    /// The drive age in ticks (OBS).
    pub fn drive_age(&self) -> u32 {
        self.drive_age
    }
}

// ---------------------------------------------------------------------------------------------
// The orchestrator state (the archive's `CtrlState` pattern, spec "Execution model").
// ---------------------------------------------------------------------------------------------

/// Everything the task bodies own between passes. The firmware places ONE of these in its static
/// orchestrator state (main-thread only: the loop and the dispatch callbacks run in the same
/// context, so there is no concurrent access this round); host tests just hold one on the stack.
pub struct OrchestratorState {
    /// The Mahony attitude filter (pipeline step 2).
    pub mahony: attitude::Mahony,
    /// The latest attitude output (pitch/roll degrees + quaternion).
    pub attitude: attitude::Output,
    /// The boot outcome: plan-present AND probe-ok (`specs/integration.md`, boot delta step 1).
    /// Fixed at construction; every failure is fail-soft (link-only-plus-throttle boot).
    pub imu_configured: bool,
    /// Tracks per-tick read success: a configured IMU whose read failed this tick reads zero and
    /// is observable here (no fault this round).
    pub imu_live: bool,
    /// The link inbox + supervision.
    pub inbox: LinkInbox,
    /// The per-motor fault latches (pipeline step 4). Present but with quiescent trip inputs
    /// this round: `running_enable` stays 0 until the slice-6 engagement sub-state tie
    /// (`a_substate`) and the motor-era trip producers exist.
    pub latches: [FaultLatch; N_MOTORS],
    /// The mode machine (pipeline step 5): the sole MOE owner.
    pub mode: ModeMachine<N_MOTORS>,
    /// The power-button debouncer (input task): one active-low line.
    pub button: inputs::LineBank,
    /// The debounced power-button level (the whole hold).
    pub button_pressed: bool,
    /// The foot-pad debouncers (input task).
    pub pads: inputs::PadBank,
    /// The merged 2-bit pad field (bit 0 = pad A, bit 1 = pad B; the gain-schedule selector).
    pub pad_field: u8,
    /// The local rider-present level: both pads asserted.
    pub rider_present: bool,
    /// The throttle IIR over the remote `INPUTS` word.
    pub throttle: inputs::ThrottleFilter,
    /// The latest filtered throttle output (+200 rest bias; meaningful once a mirror word
    /// arrived).
    pub throttle_filtered: i16,
    /// 250 Hz pipeline pass count (OBS).
    pub control_ticks: u32,
    /// 16 ms input-task pass count (OBS).
    pub input_ticks: u32,
    /// Count of INIT bring-up records emitted through the enact seam (OBS "last enact actions").
    pub enact_inits: u32,
    /// Count of SHUTDOWN safe-down records emitted through the enact seam (OBS).
    pub enact_shutdowns: u32,
}

impl OrchestratorState {
    /// A fresh orchestrator: mode OFF, empty inbox, idle inputs, identity attitude.
    /// `imu_configured` comes from the boot path (plan-present AND probe-ok); `attitude_cfg` is
    /// the per-board attitude calibration (the reference defaults on an uncalibrated board).
    pub fn new(imu_configured: bool, attitude_cfg: attitude::Config) -> Self {
        OrchestratorState {
            mahony: attitude::Mahony::new(attitude_cfg),
            attitude: attitude::Output::default(),
            imu_configured,
            imu_live: false,
            inbox: LinkInbox::new(),
            latches: [FaultLatch::new(); N_MOTORS],
            mode: ModeMachine::new(),
            button: inputs::LineBank::new(1),
            button_pressed: false,
            pads: inputs::PadBank::new(),
            pad_field: 0,
            rider_present: false,
            throttle: inputs::ThrottleFilter::new(),
            throttle_filtered: 0,
            control_ticks: 0,
            input_ticks: 0,
            enact_inits: 0,
            enact_shutdowns: 0,
        }
    }

    /// The assembled power-request level: debounced local button OR the remote `INPUTS` mirror
    /// bit (`specs/integration.md`, "The input task"; level semantics, either producer holds it).
    pub fn power_request(&self) -> bool {
        self.button_pressed || self.inbox.remote_power_request()
    }

    /// The OBS snapshot (pipeline step 9 as data): the firmware copies these into the `CTRL_OBS`
    /// RAM block each pass; host tests read them directly.
    pub fn obs(&self) -> Obs {
        let mut moe_bits = 0u8;
        for i in 0..N_MOTORS {
            if self.mode.moe_allowed(i) {
                moe_bits |= 1 << i;
            }
        }
        Obs {
            control_ticks: self.control_ticks,
            input_ticks: self.input_ticks,
            mode_byte: self.mode.mode_byte(),
            moe_bits,
            pitch_milli_deg: out_to_milli(self.attitude.pitch_deg),
            imu_configured: self.imu_configured,
            imu_live: self.imu_live,
            comms_loss: self.inbox.comms_loss(),
            cyclic_age: self.inbox.cyclic_age(),
            drive_age: self.inbox.drive_age(),
            enact_inits: self.enact_inits,
            enact_shutdowns: self.enact_shutdowns,
        }
    }
}

/// The OBS delta (pipeline step 9 as data): everything the firmware's `CTRL_OBS` RAM block
/// publishes per pass (`specs/integration.md`, "Observation"). The magic/boot-count header and
/// the RAM placement are the firmware shell's (slice 7); this is the pipeline-owned payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Obs {
    /// 250 Hz pipeline passes since boot.
    pub control_ticks: u32,
    /// 16 ms input-task passes since boot.
    pub input_ticks: u32,
    /// The mode byte.
    pub mode_byte: u8,
    /// Per-motor MOE-allowed decisions, bit i = motor i.
    pub moe_bits: u8,
    /// The latest attitude pitch, millidegrees.
    pub pitch_milli_deg: i32,
    /// The boot outcome (plan-present AND probe-ok).
    pub imu_configured: bool,
    /// This tick's read success.
    pub imu_live: bool,
    /// The peer-staleness level.
    pub comms_loss: bool,
    /// Ticks since the last accepted peer cyclic.
    pub cyclic_age: u32,
    /// Ticks since the last accepted drive command.
    pub drive_age: u32,
    /// INIT bring-up records emitted through the enact seam ("last enact actions").
    pub enact_inits: u32,
    /// SHUTDOWN safe-down records emitted through the enact seam.
    pub enact_shutdowns: u32,
}

/// Millidegrees from a degree-valued `Out` (I16F16) without overflowing the Q type
/// (`deg * 1000` exceeds I16F16's integer range past ~32 degrees, so the scale runs on the raw
/// bits in i64).
fn out_to_milli(deg: base::fixed::Out) -> i32 {
    ((deg.to_bits() as i64 * 1000) >> 16) as i32
}

// ---------------------------------------------------------------------------------------------
// The 250 Hz control task (pipeline steps 1-6 + 9).
// ---------------------------------------------------------------------------------------------

/// One 250 Hz pass's decisions, leaving as data (the R3 seam: `specs/integration.md` step 6).
/// The firmware records these into the control block + `CTRL_OBS`; the motor era replaces the
/// recording with the hardware MOE gate and the method-aware step lists AT THIS SEAM, nothing
/// else moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlOutput {
    /// The mode byte after this tick.
    pub mode_byte: u8,
    /// The per-motor MOE-allowed decisions.
    pub moe: [bool; N_MOTORS],
    /// Set on the tick the INIT bring-up must run (the recorded enactment).
    pub init: Option<InitAction>,
    /// Set on the tick the SHUTDOWN safe-down must run (the recorded enactment).
    pub shutdown: Option<ShutdownAction>,
    /// The comms-loss level this tick (OBS + the slice-6 `FsmInputs.comms_loss` feed).
    pub comms_loss: bool,
}

/// One 250 Hz control pass over already-sampled inputs. `sample` is the IMU read's product
/// (`None` = not configured or the read failed; the pipeline substitutes the zero sample and
/// records `imu_live`).
///
/// Steps (spec order): 1 sample in, 2 attitude, 3 inbox ages + snapshot, 4 fault latches,
/// 5 input assembly + mode machine (`off_inhibit = false` this round: its real producer needs
/// wheel speed, motor era), 6 the enactment record. Steps 7-8 (control dispatch, cyclic TX) are
/// slice 6. Step 9's OBS counters update here; the snapshot is [`OrchestratorState::obs`].
pub fn control_task(state: &mut OrchestratorState, sample: Option<&imu::Sample>) -> ControlOutput {
    // Step 1: the sample (or the zero sample). Live = configured and read ok this tick.
    state.imu_live = state.imu_configured && sample.is_some();
    static ZERO: imu::Sample = imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [0; 3],
        temp_centi_degc: 0,
        still: false,
    };
    let s = sample.unwrap_or(&ZERO);

    // Step 2: attitude. Gyro in rad/s, accel as sign-applied counts (direction only): the
    // attitude.md wiring, identical to the imu-bench gate image.
    let accel = [
        Fix::from_num(s.accel_raw[0]),
        Fix::from_num(s.accel_raw[1]),
        Fix::from_num(s.accel_raw[2]),
    ];
    state.attitude = state.mahony.update(s.gyro, accel);

    // Step 3: the link inbox snapshot: bump the staleness ages, then read the levels.
    state.inbox.tick_ages();
    let comms_loss = state.inbox.comms_loss();

    // Step 4: the fault latches (one per motor). Their trip inputs are quiescent this round
    // (`running_enable` = 0 until the slice-6 sub-state tie), so the tick is inert but the unit
    // is present and ordered exactly as the spec's pipeline has it.
    for latch in state.latches.iter_mut() {
        latch.tick();
    }

    // Step 5: input assembly + the mode machine. comms_loss and stop_all fold into Fault A (the
    // stock mapping) alongside motor 0's latch; motor 1's latch is the Fault B producer group.
    let mode_inputs = ModeInputs {
        power_request: state.power_request(),
        fault_a: state.latches[0].is_latched() || comms_loss || state.inbox.stop_all(),
        fault_b: state.latches[1].is_latched(),
        off_inhibit: false,
    };
    let outcome = state.mode.tick(&mode_inputs);

    // The stop_all OFF-dwell clear (`specs/link-control.md`, FAULT): the latch holds through
    // SHUTDOWN and releases once the machine has ticked in (or into) OFF, so it forces at least
    // one full OFF pass; re-entry then follows the mode machine's own gates.
    if outcome.mode == state::Mode::Off {
        self_clear_stop_all(&mut state.inbox);
    }

    // Step 6: the enactment seam, as records (R3 stubbed at its seam).
    if outcome.init.is_some() {
        state.enact_inits = state.enact_inits.wrapping_add(1);
    }
    if outcome.shutdown.is_some() {
        state.enact_shutdowns = state.enact_shutdowns.wrapping_add(1);
    }

    // Step 9 (counters half; the snapshot is `obs()`).
    state.control_ticks = state.control_ticks.wrapping_add(1);

    let mut moe = [false; N_MOTORS];
    for (i, m) in moe.iter_mut().enumerate() {
        *m = state.mode.moe_allowed(i);
    }
    ControlOutput {
        mode_byte: outcome.mode.as_byte(),
        moe,
        init: outcome.init,
        shutdown: outcome.shutdown,
        comms_loss,
    }
}

/// The one sanctioned `stop_all` clear (kept as a named edge so the latch discipline is visible
/// at its single call site).
fn self_clear_stop_all(inbox: &mut LinkInbox) {
    inbox.stop_all = false;
}

// ---------------------------------------------------------------------------------------------
// The 16 ms input task.
// ---------------------------------------------------------------------------------------------

/// The already-sampled 16 ms input levels the firmware supplies. Unconfigured pins sample as
/// their idle level (`false`): an absent button is never asserted, an absent pad never carries a
/// foot (the plan decides what gets sampled; this crate sees only levels).
#[derive(Clone, Copy, Debug, Default)]
pub struct InputSample {
    /// The power button, active-low already mapped: `true` = the line reads asserted.
    pub button_asserted: bool,
    /// Pad A, active-high: `true` = pin high (foot on).
    pub pad_a_high: bool,
    /// Pad B, active-high.
    pub pad_b_high: bool,
}

/// One 16 ms input pass (`specs/integration.md`, "The input task"): debounce the button, run the
/// pads into the rider level, and step the throttle IIR over the latest remote `INPUTS` word
/// (only once one exists, so the filter's one-shot baseline captures a real sample). The
/// products land in [`OrchestratorState`] (`button_pressed`, `pad_field`, `rider_present`,
/// `throttle_filtered`) where the 250 Hz pipeline and the slice-6 consumers read them.
pub fn input_task(state: &mut OrchestratorState, sample: &InputSample) {
    state
        .button
        .update(if sample.button_asserted { 0b1 } else { 0 });
    state.button_pressed = state.button.pressed(0);

    state.pad_field = state.pads.update(sample.pad_a_high, sample.pad_b_high);
    state.rider_present = state.pad_field == (inputs::PAD_A_BIT | inputs::PAD_B_BIT);

    // The throttle word arrives over the link as a raw ADC word carried in the i16 payload
    // field; reinterpret to the filter's unsigned domain.
    if let Some(word) = state.inbox.remote_throttle() {
        state.throttle_filtered = state.throttle.step(word as u16);
    }

    state.input_ticks = state.input_ticks.wrapping_add(1);
}

#[cfg(test)]
mod tests;
