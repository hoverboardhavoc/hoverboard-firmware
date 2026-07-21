//! The orchestrator core (`specs/integration.md`): the pure heart of the integrated firmware.
//!
//! Realizes the spec's "Execution model" state shell, "The 250 Hz pipeline" steps 1-6 and 9, and
//! "The input task", as plain functions over [`OrchestratorState`] plus already-sampled inputs.
//! No peripheral access, no statics, no scheduler: the firmware shell (slice 7) owns the
//! `Scheduler` static, the SysTick ISR, hardware sampling, and the `CTRL_OBS` RAM block; this
//! crate owns everything between the sampled inputs and the decided outputs, which is what makes
//! the pipeline host-testable end to end.
//!
//! - [`control_task`]: one 250 Hz pass. IMU sample in (`None` = absent or a failed read); on a
//!   missing sample the attitude filter HOLDS (skips the update) rather than integrating the zero
//!   sample, and a configured IMU's read health is tracked ([`ImuHealth`]): `IMU_LOSS_THRESHOLD`
//!   consecutive failed reads assert `imu_loss` into `fault_a` (a failing read must not silently
//!   freeze attitude under live torque, `specs/sensing-and-safety.md`, "IMU-loss supervision";
//!   sanity-audit P0-1) and open the retry breaker; Mahony attitude, the link-inbox
//!   snapshot + staleness ages (`specs/link-control.md`, "Supervision"), the per-motor fault
//!   latches, input assembly + the mode machine (`off_inhibit = false` this round), and the R3
//!   enactment seam: the `TickOutcome` init/shutdown records and per-motor `moe_allowed` leave as
//!   DATA in [`ControlOutput`]; nothing hardware happens here. Step 7 (the [`dispatch`] module)
//!   runs the control dispatch over the shared engagement shell: Balance = the `control.md`
//!   assembly (speed loop, shaper, PID, FSM) with the orchestrator owning the producer records;
//!   Throttle = the EFeru conditioner off the effective (staleness-decayed) drive command. The
//!   torque setpoint word is the block's sole-writer row, OBS/cyclic consumers only this round.
//!   Step 8 ([`cyclic_tx`]) builds `CYCLIC_STATE` from the block words, gated on an assigned
//!   address.
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

pub mod dispatch;

use base::fixed::Fix;
use dispatch::{new_ctl, out_to_centi, BlockWords, ControlCtl, PITCH_RATE_AXIS};
use linkctl::{CyclicState, DriveCmd, Payload, CYCLIC_TIMEOUT_TICKS, DRIVE_TIMEOUT_TICKS};
use state::{FaultLatch, InitAction, ModeInputs, ModeMachine, ShutdownAction};

pub use dispatch::{cyclic_tx, switch_control_mode};

/// The per-motor breadth of the orchestrator state: the control block's dual-motor shape
/// (`specs/control.md` (e); one MOE gate + one fault latch per advanced timer). Single-motor
/// boards simply never enact motor 1's records (the enact seam carries both; the hardware layer
/// applies the ones its plan configures).
pub const N_MOTORS: usize = 2;

// ---------------------------------------------------------------------------------------------
// IMU-loss supervision (`specs/sensing-and-safety.md`, "IMU-loss supervision"; sanity-audit
// P0-1). A configured IMU whose burst read fails must become a FAULT, not a silent
// zero-substitution: the balance PID keeps producing torque on a frozen attitude otherwise, the
// one fail-dangerous path on the way to motors.
// ---------------------------------------------------------------------------------------------

/// Consecutive failed reads that assert the IMU-loss fault (and open the retry breaker). 5 ticks
/// = 20 ms at 250 Hz: long enough to ride out an isolated glitch (the bench clone's noise floor
/// still ACKs; a genuine loss NACKs continuously), short enough that the mode machine disengages
/// well before a balance loop on frozen attitude diverges (~tens of ms). The worst-case cost
/// before the breaker opens is bounded to this many blocking reads.
pub const IMU_LOSS_THRESHOLD: u16 = 5;

/// Consecutive good reads that clear the IMU-loss fault once asserted (hysteresis). 25 ticks =
/// 100 ms of an unbroken clean stream: a single lucky read must not re-arm a balancing vehicle on
/// a flaky sensor, so recovery demands a sustained stream, not one sample. Re-engagement then
/// still costs a full OFF -> INIT -> READY -> RUN arming cycle (the mode machine's own gates).
pub const IMU_RECOVER_THRESHOLD: u16 = 25;

/// The retry-breaker probe cadence, in control ticks: once the breaker is open the firmware
/// attempts the blocking read only every this-many ticks. 250 ticks = 1 s. Rationale: the board
/// is already disengaged (SHUTDOWN/OFF) once the loss fault fires, so recovery latency is not
/// safety-critical and re-engagement needs a deliberate rider action anyway; a 1 s interval
/// bounds the worst-case stuck-bus read (~10-28 ms of polled I2C `wait_flag`) to ~3% CPU instead
/// of collapsing the 250 Hz loop by retrying it every tick.
pub const IMU_PROBE_CADENCE: u32 = 250;

/// A configured IMU's read health: the consecutive-failure / consecutive-success streaks and the
/// latched loss level. Absence (no IMU configured) is NOT loss and never touches this. The loss
/// level is a hysteresis latch (asserts at [`IMU_LOSS_THRESHOLD`] fails, clears at
/// [`IMU_RECOVER_THRESHOLD`] successes); the retry breaker tracks the failure streak directly (so
/// one good probe read closes the breaker and lets the recovery stream run at full rate, while
/// the fault stays asserted until the stream is proven clean).
#[derive(Clone, Copy, Debug)]
pub struct ImuHealth {
    fail_streak: u16,
    ok_streak: u16,
    loss: bool,
}

impl ImuHealth {
    /// A healthy start: no failures, no loss.
    pub const fn new() -> Self {
        ImuHealth {
            fail_streak: 0,
            ok_streak: 0,
            loss: false,
        }
    }

    /// Fold one control tick's outcome in. `configured` false = absence (reset to healthy, never
    /// loss); otherwise `read_ok` advances the failure or success streak and moves the hysteresis
    /// latch.
    fn update(&mut self, configured: bool, read_ok: bool) {
        if !configured {
            *self = ImuHealth::new();
            return;
        }
        if read_ok {
            self.fail_streak = 0;
            self.ok_streak = self.ok_streak.saturating_add(1);
            if self.loss && self.ok_streak >= IMU_RECOVER_THRESHOLD {
                self.loss = false;
            }
        } else {
            self.ok_streak = 0;
            self.fail_streak = self.fail_streak.saturating_add(1);
            if self.fail_streak >= IMU_LOSS_THRESHOLD {
                self.loss = true;
            }
        }
    }

    /// The IMU-loss fault level (feeds `fault_a`): asserted after [`IMU_LOSS_THRESHOLD`] fails,
    /// held until [`IMU_RECOVER_THRESHOLD`] consecutive successes.
    pub const fn loss(&self) -> bool {
        self.loss
    }

    /// Whether the retry breaker is open (the failure streak has reached the threshold): the
    /// firmware backs the blocking read off to the probe cadence while this holds.
    pub const fn backoff(&self) -> bool {
        self.fail_streak >= IMU_LOSS_THRESHOLD
    }

    /// The current consecutive-failure streak (OBS/diagnostics).
    pub const fn fail_streak(&self) -> u16 {
        self.fail_streak
    }
}

impl Default for ImuHealth {
    fn default() -> Self {
        Self::new()
    }
}

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
    /// Tracks per-tick read success: `imu_configured AND read ok this tick`. False on a failed
    /// read (the staleness signal); the filter holds and [`imu_health`](Self::imu_health) decides
    /// when the miss stream becomes the loss fault.
    pub imu_live: bool,
    /// The configured IMU's read health (`specs/sensing-and-safety.md`, "IMU-loss supervision"):
    /// the miss/hit streaks + the loss latch that feeds `fault_a` and drives the retry breaker.
    pub imu_health: ImuHealth,
    /// The link inbox + supervision.
    pub inbox: LinkInbox,
    /// The per-motor fault latches (pipeline step 4). The a_substate tie is live: each pass
    /// feeds the engagement machine's sub-state (previous tick's, the spec's step order) as
    /// `a_substate`, the per-motor wheel word as `b_motion`, and `running_enable` follows the
    /// mode machine's RUN (the latch task runs on a running system; its 10-minute idle
    /// count-to-latch is the stock idle timeout). The fast trip producers (over-current codes,
    /// ext_trip) stay quiescent until the motor era. Cleared whole on any pass whose resulting
    /// mode is OFF (the power-cycle analog; the latch contract's downstream co-writer).
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
    /// The step-7 control section: the mode dispatch + the balance producer records
    /// ([`dispatch::ControlCtl`]).
    pub ctl: ControlCtl,
    /// The RAM control block's words as data ([`dispatch::BlockWords`]; `specs/control.md` (e)).
    pub block: BlockWords,
}

impl OrchestratorState {
    /// A fresh orchestrator: mode OFF, empty inbox, idle inputs, identity attitude, and the
    /// control dispatch through its boot seam (`ControlDispatch::new(CONTROL_MODE byte,
    /// imu_configured)`: Balance demotes to Throttle with the mode fault when the IMU is
    /// absent). `imu_configured` comes from the boot path (plan-present AND probe-ok);
    /// `attitude_cfg` is the per-board attitude calibration (the reference defaults on an
    /// uncalibrated board).
    pub fn new(
        control_mode_byte: u8,
        imu_configured: bool,
        attitude_cfg: attitude::Config,
    ) -> Self {
        let (ctl, block) = new_ctl(control_mode_byte, imu_configured);
        OrchestratorState {
            mahony: attitude::Mahony::new(attitude_cfg),
            attitude: attitude::Output::default(),
            imu_configured,
            imu_live: false,
            imu_health: ImuHealth::new(),
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
            ctl,
            block,
        }
    }

    /// The assembled power-request level: debounced local button OR the remote `INPUTS` mirror
    /// bit (`specs/integration.md`, "The input task"; level semantics, either producer holds it).
    pub fn power_request(&self) -> bool {
        self.button_pressed || self.inbox.remote_power_request()
    }

    /// Whether the firmware should attempt the blocking IMU read on this control tick (the retry
    /// breaker, `specs/integration.md` pipeline step 1). True normally (read every tick); once the
    /// breaker is open ([`IMU_LOSS_THRESHOLD`] consecutive fails) only every [`IMU_PROBE_CADENCE`]
    /// ticks, so a stuck-bus read (~10-28 ms of polled I2C) is bounded to a probe cadence instead
    /// of burned every 4 ms tick. A non-configured board never opens the breaker (absence is not
    /// loss), so every tick reads (and yields `None` for want of a bus, unchanged).
    pub fn imu_read_due(&self, tick: u32) -> bool {
        !self.imu_health.backoff() || tick.is_multiple_of(IMU_PROBE_CADENCE)
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
            imu_loss: self.imu_health.loss(),
            comms_loss: self.inbox.comms_loss(),
            cyclic_age: self.inbox.cyclic_age(),
            drive_age: self.inbox.drive_age(),
            enact_inits: self.enact_inits,
            enact_shutdowns: self.enact_shutdowns,
            torque_setpoint: self.ctl.fsm.torque_setpoint,
            sub_state: self.ctl.fsm.sub_state as u8,
            control_mode: self.ctl.dispatch.mode() as u8,
            mode_fault: self.ctl.dispatch.mode_fault(),
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
    /// The IMU-loss fault level (`IMU_LOSS_THRESHOLD` consecutive failed reads on a configured
    /// IMU; feeds `fault_a`). Distinct from `imu_live`: a single miss drops `imu_live` but does
    /// not fault.
    pub imu_loss: bool,
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
    /// The torque setpoint word (the block's sole-writer row; OBS/cyclic consumers only).
    pub torque_setpoint: i16,
    /// The engagement machine's sub-state byte (the `a_substate` the latches consume).
    pub sub_state: u8,
    /// The active control mode byte (0 = Throttle, 1 = Balance).
    pub control_mode: u8,
    /// True when the requested mode was demoted at the validation seam (Balance without an IMU).
    pub mode_fault: bool,
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
    /// The comms-loss level this tick (OBS + the FSM's immediate-stop feed).
    pub comms_loss: bool,
    /// The IMU-loss fault level this tick (folded into `fault_a`; OBS).
    pub imu_loss: bool,
    /// The torque setpoint word this tick (step 7's output; the sole-writer row's value).
    pub torque_setpoint: i16,
    /// The engagement sub-state byte after this tick (0 forces a zero setpoint).
    pub sub_state: u8,
}

/// One 250 Hz control pass over already-sampled inputs. `sample` is the IMU read's product
/// (`None` = not configured or the read failed; on `None` the attitude filter HOLDS, and for a
/// configured IMU the miss advances [`ImuHealth`] toward the loss fault).
///
/// Steps, in spec order: sample in (step 1); attitude plus the block's attitude/rate words
/// (step 2); inbox ages + snapshot (step 3); the fault latches with the a_substate tie
/// (step 4); input assembly + the mode machine, `off_inhibit = false` this round since its real
/// producer needs wheel speed (step 5); the enactment record (step 6); the control dispatch,
/// [`dispatch`] (step 7). The cyclic TX (step 8) is [`cyclic_tx`], called by the firmware with
/// the address fact. The OBS counters (step 9) update here; the snapshot is
/// [`OrchestratorState::obs`].
pub fn control_task(
    state: &mut OrchestratorState,
    sample: Option<&imu::Sample>,
    dt_ticks: u32,
) -> ControlOutput {
    // Step 1: IMU health + the sample. Live = configured AND read ok this tick. A configured
    // board tracks its read health: IMU_LOSS_THRESHOLD consecutive fails assert `imu_loss` (a
    // failing read must NOT silently freeze attitude under live torque, sanity-audit P0-1) and
    // open the retry breaker; absence (not configured) is never loss.
    state.imu_live = state.imu_configured && sample.is_some();
    state
        .imu_health
        .update(state.imu_configured, sample.is_some());
    let imu_loss = state.imu_health.loss();

    // Step 2: attitude. On a real sample: gyro in rad/s, accel as sign-applied counts (direction
    // only, the attitude.md wiring), integrated dt-honest (round-4 defect B: over the ticks that
    // ACTUALLY elapsed, not an assumed 4 ms). On a MISSING sample (absence OR a failed read) the
    // filter HOLDS: it skips the update rather than integrating the zero sample (which froze
    // pitch/roll at their last values while the mode machine kept producing torque). The last-good
    // attitude and block words hold; `imu_live`/`imu_loss` make the staleness visible.
    if let Some(s) = sample {
        let accel = [
            Fix::from_num(s.accel_raw[0]),
            Fix::from_num(s.accel_raw[1]),
            Fix::from_num(s.accel_raw[2]),
        ];
        state.attitude = state.mahony.update_dt(s.gyro, accel, dt_ticks);
        // The block's attitude words (stock-native centidegrees) and the pitch-rate word (@0x9c,
        // the sign-applied gyro counts on the pitch axis).
        state.block.pitch_word = out_to_centi(state.attitude.pitch_deg);
        state.block.roll_word = out_to_centi(state.attitude.roll_deg);
        state.block.pitch_rate = s.gyro_raw[PITCH_RATE_AXIS] as i32;
    }

    // Step 3: the link inbox snapshot: bump the staleness ages, then read the levels.
    state.inbox.tick_ages();
    let comms_loss = state.inbox.comms_loss();

    // Step 4: the fault latches (one per motor), with the a_substate tie live: the engagement
    // machine's sub-state (previous tick's value: the latches run before the dispatch in the
    // spec's step order) as `a_substate`, the per-motor wheel word as `b_motion`, and
    // `running_enable` following RUN (the latch task runs on a running system; in RUN its
    // HEALTHY predicate is meaningful, and idle-in-RUN counts toward the stock 10-minute
    // latch). The fast trip inputs (codes, ext_trip) stay quiescent until the motor era.
    let a_substate = state.ctl.fsm.sub_state as i8;
    let run_enable = (state.mode.mode() == state::Mode::Run) as u8;
    for (i, latch) in state.latches.iter_mut().enumerate() {
        latch.running_enable = run_enable;
        latch.a_substate = a_substate;
        latch.b_motion = state.block.wheel_speed[i];
        latch.tick();
    }

    // Step 5: input assembly + the mode machine. comms_loss, stop_all, and imu_loss fold into
    // Fault A (the stock mapping: general/sensing faults) alongside motor 0's latch; motor 1's
    // latch is the Fault B producer group.
    let mode_inputs = ModeInputs {
        power_request: state.power_request(),
        fault_a: state.latches[0].is_latched() || comms_loss || state.inbox.stop_all() || imu_loss,
        fault_b: state.latches[1].is_latched(),
        off_inhibit: false,
    };
    let outcome = state.mode.tick(&mode_inputs);

    // The stop_all OFF-dwell clear (`specs/link-control.md`, FAULT): the latch holds through
    // SHUTDOWN and releases once the machine has ticked in (or into) OFF, so it forces at least
    // one full OFF pass; re-entry then follows the mode machine's own gates.
    //
    // The fault-latch clear discipline rides the same edge (`specs/integration.md` step 4, the
    // slice-6 audit fold): the latches are one-way and stock cleared them by cutting its own
    // power at OFF (the SELF_HOLD release); a pass whose resulting mode is OFF is this
    // integration's power-cycle analog, and the orchestrator is the "downstream co-writer" the
    // latch contract names. Without it a tripped latch (e.g. the 10-minute idle-in-RUN timeout)
    // would block OFF -> INIT forever.
    if outcome.mode == state::Mode::Off {
        self_clear_stop_all(&mut state.inbox);
        for latch in state.latches.iter_mut() {
            *latch = FaultLatch::new();
        }
    }

    // Step 6: the enactment seam, as records (R3 stubbed at its seam).
    if outcome.init.is_some() {
        state.enact_inits = state.enact_inits.wrapping_add(1);
    }
    if outcome.shutdown.is_some() {
        state.enact_shutdowns = state.enact_shutdowns.wrapping_add(1);
    }

    // Step 7: the control dispatch (Balance = the cascade assembly, Throttle = the EFeru
    // conditioner off the effective drive command), producing the torque setpoint word into the
    // block's sole-writer row.
    let torque_setpoint = dispatch::control_dispatch_step(state, outcome.mode == state::Mode::Run);

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
        imu_loss,
        torque_setpoint,
        sub_state: state.ctl.fsm.sub_state as u8,
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
