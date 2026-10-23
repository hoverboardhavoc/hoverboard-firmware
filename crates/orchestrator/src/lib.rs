// SPDX-License-Identifier: GPL-3.0-or-later
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
//!   latches, input assembly + the mode machine (`off_inhibit` from the firmware's
//!   wheel-motion level), and the R3
//!   enactment seam: the `TickOutcome` init/shutdown records and per-motor `moe_allowed` leave as
//!   DATA in [`ControlOutput`]; nothing hardware happens here. Step 7 (the [`dispatch`] module)
//!   runs the control dispatch over the shared engagement shell: Balance = the `control.md`
//!   assembly (speed loop, shaper, PID, FSM) with the orchestrator owning the producer records;
//!   Throttle = the EFeru conditioner off the effective (staleness-decayed) drive command. The
//!   torque setpoint word is the block's sole-writer row, OBS/cyclic consumers only this round.
//!   Step 8 ([`cyclic_tx`]) builds `CYCLIC_STATE` from the block words, gated on an assigned
//!   address. After step 3 the [`battery`] source rule writes the block's effective battery word
//!   (local filtered sense, else a fresh peer's word, else 0 = UNKNOWN).
//! - [`input_task`]: one 16 ms pass. The power-button debounce ([`inputs::LineBank`], active-low,
//!   two-call press / one-call release) and the foot pads ([`inputs::PadBank`]) into the rider
//!   level. `power_request` = debounced button OR the `INPUTS` mirror bit (level
//!   semantics), with one asymmetry between the two producers: the button's level is held by a
//!   finger and a pin, the mirror's only by a controller that keeps TALKING inside
//!   `INPUTS_TIMEOUT_TICKS`.
//! - [`LinkInbox`]: the latest-wins words the firmware loop's delivered-PDU hand-back routes in
//!   (`linkctl::Payload`), plus the supervision state: the cyclic/drive/mirror age counters, the
//!   `comms_loss` level (never-seen-a-peer exempt: single-board operation is legitimate), and
//!   the `stop_all` latch with its OFF-dwell clear. Every aged family de-asserts on staleness,
//!   the remote `INPUTS` mirror included: a controller holds a level by STAYING PRESENT (any
//!   `INPUTS` or `DRIVE_CMD` from the node that stated it, [`LinkInbox::refreshes_mirror`]), so
//!   the one release path a departed controller cannot take is taken for it here.
//!
//! `crates/control` / `crates/state` stay link-agnostic (`specs/link-control.md`, "Crate
//! placement"): `linkctl` types stop at this crate's inbox; the mode machine consumes plain
//! levels.
//!
//! `no_std`; host tests in `tests.rs` link `std` via the host target.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod battery;
pub mod dispatch;
pub mod events;

use base::fixed::Fix;
use control::helpers::q_to_int_d2iz;
use dispatch::{new_ctl, out_to_centi, BlockWords, ControlCtl, PITCH_RATE_AXIS, UP_AXIS};
use events::FaultEvents;
use linkctl::{
    ChipTag, CyclicState, DriveCmd, Payload, CYCLIC_TIMEOUT_TICKS, DRIVE_TIMEOUT_TICKS,
    INPUTS_TIMEOUT_TICKS,
};
use state::{FaultLatch, InitAction, ModeInputs, ModeMachine, ShutdownAction};

pub use dispatch::{
    ble_cyclic_tx, ble_peer_tx, cyclic_tx, re_apply_control_values, PeerCyclic, BLE_CYCLIC_DIVISOR,
    BLE_PEER_CYCLIC_TICKS, CYCLIC_EMIT_TICKS,
};
pub use events::{
    EV_COMMS_LOSS, EV_IMU_LOSS, EV_LATCH_A, EV_LATCH_B, EV_MODE_FAULT, EV_MOTOR_FAULT,
    EV_POWER_REQUEST, EV_STOP_ALL, FAULT_A_BITS, N_EVENT_PRODUCERS,
};

/// The per-motor breadth of the orchestrator state: the control block's dual-motor shape
/// (`specs/control.md` (e); one MOE gate + one fault latch per advanced timer). Single-motor
/// boards simply never enact motor 1's records (the enact seam carries both; the hardware layer
/// applies the ones its plan configures).
pub const N_MOTORS: usize = 2;

/// The breadth of the input task's debounce bank: one active-low line, the power button
/// (`specs/integration.md`, "The input task"). The bank's size IS its line count
/// ([`inputs::LineBank`]), so this constant is where that count is decided, once, rather than a
/// number carried alongside an array that cannot vouch for it. The foot pads are a separate bank
/// ([`inputs::PadBank`]) with its own fixed shape.
pub const N_BUTTON_LINES: usize = 1;

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
/// bounds the cost of a failing read to well under 1% CPU instead of collapsing the 250 Hz loop by
/// retrying it every tick. The cadence rations that cost; the per-pass bound on it belongs to the
/// HAL's I2C poll budget, which is what keeps any single pass inside the motor's demand-stale
/// coast (the firmware const-asserts the two against each other).
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
    /// The L3 source address the latest peer cyclic arrived under (`None` = never seen a peer).
    /// Recorded beside the state because the BLE forward re-publishes that state under the
    /// address of the board that stated it (`specs/link-control.md`, "The peer's state on the
    /// BLE port: the forward"), and the inbox is the one owner of what the peer last said.
    peer_src: Option<u8>,
    /// Ticks since the last accepted cyclic (saturating; meaningless while `peer` is `None`).
    cyclic_age: u32,
    /// The largest value [`LinkInbox::cyclic_age`] has reached since boot: the PEAK LATCH, an
    /// instrument and not an input ([`LinkInbox::cyclic_age_peak`]).
    cyclic_age_peak: u32,
    /// The latest accepted `DRIVE_CMD` (`None` = never received).
    drive: Option<DriveCmd>,
    /// Ticks since the last accepted drive command.
    drive_age: u32,
    /// The latest accepted remote `INPUTS` mirror (`None` = never received). CONTENT: written by
    /// `INPUTS` and by nothing else, so no other payload can put a level in here that its sender
    /// never stated.
    remote: Option<linkctl::Inputs>,
    /// The L3 source address of the node that wrote [`LinkInbox::remote`] (`None` = no mirror
    /// yet). The mirror's OWNER: whose continued presence keeps it fresh, per
    /// [`LinkInbox::refreshes_mirror`].
    remote_src: Option<u8>,
    /// Ticks since the mirror's owner was last heard from: LIVENESS, not content. Reset by any
    /// payload that counts as the owner still being here, which is a wider set than the payload
    /// that wrote the content. Past [`INPUTS_TIMEOUT_TICKS`] the mirror stops being a source
    /// ([`LinkInbox::remote_stale`]) and stops taking a liveness refresh at all: expiry is a
    /// one-way door, and only fresh content reopens it (see [`LinkInbox::refreshes_mirror`]).
    remote_age: u32,
    /// The `FAULT action = STOP_ALL` latch: set on receipt, feeds `ModeInputs.fault_a`, cleared
    /// only by the mode machine passing through OFF (the OFF-dwell clear, applied by
    /// [`control_task`]).
    stop_all: bool,
}

/// The rigid frame's freshness window on the peer cyclic, in served ticks
/// (`specs/rover-one-imu.md` section 6): a rigid board applies torque only while its peer's
/// cyclic is at most this old.
///
/// **5 ticks = 20 ms, MEASURED** (`specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`): the
/// maximum age observed on either board over a ten-minute run on the offroad GD32F130 pair, which
/// was 3 ticks, plus one 8 ms emission period, which is section 8's rule. It replaces a derived
/// 4, and the measurement has two limits the number inherits. The poll interval was about 7.7 ms
/// against a 4 ms tick, so what the run rules out is a POPULATION of excursions to 4, not a
/// single rare one. And the run was disarmed: the hazard that would stretch the window, the IMU's
/// I2C burst delaying the cyclic transmit, was live, but the motor ISR's load was not, so an
/// energised re-run is a gate of its own (section 8).
///
/// Deliberately a separate, faster clock than `linkctl::CYCLIC_TIMEOUT_TICKS` (25, the disarm):
/// crossing this one costs a tick of authority and is restored by the next frame; crossing that
/// one is a fault.
pub const RIGID_STALE_TICKS: u32 = 5;

impl LinkInbox {
    /// An empty inbox: no peer, no drive, no mirror, latch clear.
    pub const fn new() -> Self {
        LinkInbox {
            peer: None,
            peer_src: None,
            cyclic_age: 0,
            cyclic_age_peak: 0,
            drive: None,
            drive_age: 0,
            remote: None,
            remote_src: None,
            remote_age: 0,
            stop_all: false,
        }
    }

    /// Does a payload from `src` count as evidence that the mirror's owner is still here
    /// (`specs/link-control.md`, "Supervision": controller liveness)?
    ///
    /// Three independent conditions, and each is the point.
    ///
    /// **The payload has to be a controller OPERATING this board on demand**, which is `INPUTS`
    /// and `DRIVE_CMD` and nothing else. A live LINK is not a live controller:
    ///
    /// - `CYCLIC_STATE` is an unconditional heartbeat that any powered, addressed board emits
    ///   every 8 ms whether or not anyone is riding. Counting it would let a peer board hold its
    ///   neighbour armed for as long as it has power, which is the failure the timeout exists to
    ///   prevent, restated.
    /// - `FAULT` is an edge notification, and the meaning of the one action it carries is STOP.
    ///   The payload that says stop must not extend the window that keeps the machine armed.
    /// - The walk and config families (`NODE_HELLO`, `PROBE_PORTS`, `ASSIGN`, `CONFIG_READ` /
    ///   `CONFIG_WRITE`) never reach this inbox at all: `net::Responder` answers them and the
    ///   firmware's hand-back only routes the `0x10..0x2F` control block. They would be wrong
    ///   here even if they did arrive, because they are a CONFIGURATOR talking to a board, not a
    ///   rider holding one armed, and the board already REFUSES `ASSIGN` and `CONFIG_WRITE`
    ///   outright while armed. Traffic the board declines to act on cannot be its evidence that
    ///   someone is aboard.
    ///
    /// **And it has to come from the node that owns the mirror.** Otherwise a second controller
    /// idling on the same link holds a rider's arm alive on their behalf, and the question the
    /// timeout answers decays from "has MY controller gone away" to "is anyone at all talking".
    /// The `src` byte is unauthenticated and this is not access control: it is SCOPING, and it
    /// consumes the identity `net` establishes (guest addresses are granted precisely so nodes
    /// are distinguishable) rather than re-deriving one here.
    ///
    /// **And the mirror has to still be alive.** Liveness EXTENDS a window; it may not reopen one
    /// that has closed. Past the timeout the board has already released the level, and a payload
    /// arriving then says the controller is talking again, never that the level it stated 1.6 s
    /// ago still stands. Without this an expired mirror walks a DISARMED board back to RUN on a
    /// single demand frame with no fresh `INPUTS` anywhere in the sequence, which is the spurious
    /// ARM the content/liveness split exists to prevent, reached by the other door. An expired
    /// mirror is revivable only by `INPUTS`, which rewrites the content in the same breath, so
    /// what a board re-arms on is always a level its controller stated just now.
    fn refreshes_mirror(&self, src: u8, payload: &Payload) -> bool {
        matches!(payload, Payload::Inputs(_) | Payload::DriveCmd(_))
            && self.remote_src == Some(src)
            && !self.remote_stale()
    }

    /// Accept one decoded control-block payload from L3 node `src` (latest-wins). `CYCLIC_STATE`,
    /// `DRIVE_CMD` and `INPUTS` reset their own staleness ages; a `FAULT` with
    /// `action = STOP_ALL` sets the latch (a notify-only `FAULT` carries no consumer this round:
    /// the level lives in `CYCLIC_STATE.fault`, and the controller-facing path is deferred to
    /// telemetry).
    ///
    /// The mirror is written in two separable halves, which is what keeps a demand from
    /// impersonating a request:
    ///
    /// - Its CONTENT (`remote`, and the owner `remote_src` that comes with it) is written by
    ///   `INPUTS` alone. A `DRIVE_CMD` cannot create a mirror, cannot set a level in one, and
    ///   cannot change who owns one.
    /// - Its LIVENESS (`remote_age`) is refreshed by any payload passing
    ///   [`LinkInbox::refreshes_mirror`], so a rider whose 20 Hz demand stream is arriving stays
    ///   armed on the strength of the demand stream, not on whether the far slower keepalive that
    ///   last stated the level happened to survive the hop.
    pub fn accept(&mut self, src: u8, payload: Payload) {
        if self.refreshes_mirror(src, &payload) {
            self.remote_age = 0;
        }
        match payload {
            Payload::CyclicState(c) => {
                self.peer = Some(c);
                self.peer_src = Some(src);
                self.cyclic_age = 0;
            }
            Payload::DriveCmd(d) => {
                self.drive = Some(d);
                self.drive_age = 0;
            }
            Payload::Inputs(i) => {
                self.remote = Some(i);
                self.remote_src = Some(src);
                self.remote_age = 0;
            }
            Payload::Fault(f) => {
                if f.stop_all() {
                    self.stop_all = true;
                }
            }
        }
    }

    /// One age bump (pipeline step 3), for a run that SERVED A NEW TICK. Ages only advance once
    /// the family has been seen: a board that never saw a peer does not accrue toward
    /// `comms_loss` (the never-seen exemption).
    ///
    /// **`served` is what makes these ages a clock** (`specs/link-control.md`, "What a tick of the
    /// ages is"). They used to advance once per `control_task` CALL, which is not the same
    /// quantity: the scheduler holds missed slots in its `runcount` and `dispatch()` drains them
    /// one per main-loop pass, so after an absence this pipeline runs many times inside one
    /// SysTick tick. Measured on the offroad pair 2026-10-22: bursts of up to 294 runs, with the
    /// control task sustaining 300-650 runs/s against a peer emitting 125 frames/s. The ages then
    /// crossed `CYCLIC_TIMEOUT_TICKS` in 40-80 ms of wall time against a demonstrably live link,
    /// which asserted `comms_loss`, which is a `fault_a` producer, which shut a RUNNING board down
    /// about once per second on both boards at once.
    ///
    /// So the unit is **one count per SysTick tick in which this board actually serviced its
    /// links**, and the two degenerate cases are deliberate:
    ///
    /// - A catch-up run (`served = false`, no tick elapsed since the previous run) advances
    ///   nothing. No time passed, and the peer cannot have sent anything in no time.
    /// - A run that follows an absence of many ticks advances by ONE, not by the ticks missed.
    ///   The board could not have heard the peer while it was away, so its own absence is not
    ///   evidence that the PEER has gone, which is the only question this age exists to answer.
    ///   A board that is away that long has its own producers for it (period liveness, the RX
    ///   ring's lap counter); borrowing the peer's is both a mislabel and contagious, because the
    ///   emission cadence rides the same clock and a stalled board's silence trips its peer.
    ///
    /// The cost of the forgiveness is bounded and stated: a board missing a fraction `f` of its
    /// slots detects a genuinely dead peer in `CYCLIC_TIMEOUT_TICKS / (1 - f)` ticks of wall time
    /// rather than `CYCLIC_TIMEOUT_TICKS`. At the 8.7% measured on the offroad master that is
    /// 110 ms against the nominal 100 ms.
    fn tick_ages(&mut self, served: bool) {
        if !served {
            return;
        }
        if self.peer.is_some() {
            self.cyclic_age = self.cyclic_age.saturating_add(1);
            // The peak latch, taken here because here is where the age moves: every value any
            // consumer can read is a value this line has already seen.
            if self.cyclic_age > self.cyclic_age_peak {
                self.cyclic_age_peak = self.cyclic_age;
            }
        }
        if self.drive.is_some() {
            self.drive_age = self.drive_age.saturating_add(1);
        }
        if self.remote.is_some() {
            self.remote_age = self.remote_age.saturating_add(1);
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

    /// The peer cyclic AS A RIGID-FRAME INPUT: the latest words while they are at most
    /// [`RIGID_STALE_TICKS`] old, `None` once older or never seen. Every rigid consumer reads the
    /// peer through here (the follower's word and gate, the leader's partner gate, the two-leader
    /// level, the wheel gate on both), so staleness is applied at the read and cannot be forgotten
    /// by a new consumer, the [`LinkInbox::fresh_remote`] discipline.
    pub fn rigid_peer(&self) -> Option<&CyclicState> {
        self.peer
            .as_ref()
            .filter(|_| self.cyclic_age <= RIGID_STALE_TICKS)
    }

    /// The address the latest peer cyclic arrived under, `None` when never seen.
    pub fn peer_src(&self) -> Option<u8> {
        self.peer_src
    }

    /// The peer's last state and the address it arrived under, while that state is FRESH: a
    /// peer has been seen and its cyclic age is inside [`CYCLIC_TIMEOUT_TICKS`], the complement of
    /// [`comms_loss`](Self::comms_loss). This is the BLE forward's source (`specs/link-control.md`,
    /// "The peer's state on the BLE port: the forward"): past the window the forward stops rather
    /// than repeating a reading this board has itself stopped trusting. A longer window than
    /// [`rigid_peer`](Self::rigid_peer)'s, because this one feeds a display and that one feeds a
    /// wheel.
    pub fn fresh_peer(&self) -> Option<(u8, CyclicState)> {
        match (self.peer, self.peer_src) {
            (Some(p), Some(src)) if self.cyclic_age <= CYCLIC_TIMEOUT_TICKS => Some((src, p)),
            _ => None,
        }
    }

    /// The peer lockdown level (`CYCLIC_STATE.flags` bit 7, latest-wins): the slice-6 engagement
    /// machine treats it as a gating fault (sub-state forced to 0, torque 0) while asserted.
    pub fn peer_lockdown(&self) -> bool {
        self.peer.map(|p| p.lockdown()).unwrap_or(false)
    }

    /// The mirror-staleness predicate (`specs/link-control.md`, "Supervision"): true when the
    /// mirror's owner has not been heard from within [`INPUTS_TIMEOUT_TICKS`] (or no mirror was
    /// ever received). Exactly the shape of [`LinkInbox::drive_stale`]; the difference is what it
    /// is allowed to do, which is nothing at all: a stale mirror is not a source.
    ///
    /// "Heard from" is [`LinkInbox::refreshes_mirror`], NOT "sent an `INPUTS`". The window is
    /// supposed to answer whether the controller has gone away, and answering "did the keepalive
    /// specifically arrive" instead put a rider's arm on the app's slowest frame: at a 500 ms
    /// keepalive against 1.5 s, two consecutive losses survived with no margin at all, while
    /// twenty demand frames a second were arriving and being enacted.
    pub fn remote_stale(&self) -> bool {
        match self.remote {
            None => true,
            Some(_) => self.remote_age > INPUTS_TIMEOUT_TICKS,
        }
    }

    /// The mirror AS A SOURCE: the latest `INPUTS` while it is fresh, `None` once it is stale.
    ///
    /// Every mirror read goes through here, which is the point. The failure this closes was one
    /// slot that never expired, so a controller's last word asserted forever: dropping the link
    /// while armed left the board in RUN with the motor enables set, indefinitely, and no release
    /// path the controller could take, because it was already gone. Staleness therefore has to be
    /// applied at the READ, where it cannot be forgotten by a new consumer, rather than at each
    /// call site.
    fn fresh_remote(&self) -> Option<linkctl::Inputs> {
        if self.remote_stale() {
            None
        } else {
            self.remote
        }
    }

    /// The remote power-request mirror bit (`INPUTS.buttons` bit 0, level). A stale mirror reads
    /// RELEASED: an assertion nobody is still making must not hold a machine armed.
    pub fn remote_power_request(&self) -> bool {
        self.fresh_remote()
            .map(|i| i.power_request())
            .unwrap_or(false)
    }

    /// The remote rider-present mirror bit (`INPUTS.rider` bit 0, level). A stale mirror reads
    /// NO RIDER, for the same reason the power bit does: it is a permissive level (it folds in by
    /// OR with the local pads, `dispatch::rider_level`), so holding it true on a dead link would
    /// let a departed controller keep asserting that someone is aboard.
    pub fn remote_rider_present(&self) -> bool {
        self.fresh_remote()
            .map(|i| i.rider_present())
            .unwrap_or(false)
    }

    /// The `stop_all` latch level (feeds `fault_a`).
    pub fn stop_all(&self) -> bool {
        self.stop_all
    }

    /// The cyclic age in ticks (OBS).
    pub fn cyclic_age(&self) -> u32 {
        self.cyclic_age
    }

    /// The PEAK cyclic age in ticks since boot (OBS, `CTRL_OBS` word 36): the largest value
    /// [`LinkInbox::cyclic_age`] has ever held on this board, latched in
    /// [`LinkInbox::tick_ages`] as the age is advanced.
    ///
    /// **It exists because [`RIGID_STALE_TICKS`] is set from a worst case and the worst case was
    /// being sampled.** The bench read the live age out of `CTRL_OBS` over SWD at about 130 Hz
    /// against a 250 Hz age, so roughly half of all ticks were never looked at and whether a rare
    /// excursion was caught was a coin flip. Three ten-minute runs on 2026-10-23 disagreed about
    /// the maximum, one energised run reporting 11 ticks and two later ones capping at 4
    /// (`specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`), and no amount of sampling
    /// harder settles that. A latch taken where the age is maintained sees every tick by
    /// construction, so one read at the end of a soak run is the whole answer.
    ///
    /// **Since boot, and nothing clears it.** A soak run is a single boot, the number is
    /// monotonic, so a read at any moment is the true maximum so far and the reader resets it by
    /// resetting the board. A clear path would need a writer, and a writer would need a consumer
    /// nothing here has.
    ///
    /// Only the peer cyclic gets one. The other ages in this inbox (`drive_age`, `remote_age`)
    /// are supervision windows with no number being measured against them, and an instrument with
    /// no reader is cost without an answer.
    pub fn cyclic_age_peak(&self) -> u32 {
        self.cyclic_age_peak
    }

    /// The drive age in ticks (OBS).
    pub fn drive_age(&self) -> u32 {
        self.drive_age
    }

    /// Ticks since the mirror's owner was last heard from (meaningless before the first mirror).
    pub fn remote_age(&self) -> u32 {
        self.remote_age
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
    pub button: inputs::LineBank<N_BUTTON_LINES>,
    /// The debounced power-button level (the whole hold).
    pub button_pressed: bool,
    /// The foot-pad debouncers (input task).
    pub pads: inputs::PadBank,
    /// The merged 2-bit pad field (bit 0 = pad A, bit 1 = pad B; the gain-schedule selector).
    pub pad_field: u8,
    /// The local rider-present level: both pads asserted.
    pub rider_present: bool,
    /// 250 Hz pipeline pass count (OBS).
    ///
    /// An INVOCATION count, not a clock: read against the SysTick count (`CTRL_OBS` words 2 and
    /// 4) it is the instrument that shows how many slots this board missed, which is why it is
    /// counted this way and why nothing that measures time may be derived from it. The two
    /// emission cadences used to be, and are not any more ([`Self::cyclic_since_tx`]).
    pub control_ticks: u32,
    /// Ticks elapsed since the last `CYCLIC_STATE` emission to the peer port, the inter-board
    /// cadence's clock (`specs/link-control.md`, "Addressing and emission").
    ///
    /// It replaces `control_ticks.is_multiple_of(2)`. An invocation parity is not a rate when the
    /// scheduler drains missed slots one per main-loop pass: a 294-run catch-up burst fired 147
    /// emissions back to back, ~4 kB into the peer's 128-byte RX ring, which is the other half of
    /// how one board's absence trips its neighbour (measured on the offroad pair 2026-10-22: 75
    /// ring laps on the SLAVE against 24 absences of its own). Counting ticks bounds the burst to
    /// one frame however many runs it holds.
    pub cyclic_since_tx: u32,
    /// Whether this pass is an emitting one for the peer port, decided by
    /// [`cyclic_since_tx`](Self::cyclic_since_tx) reaching [`CYCLIC_EMIT_TICKS`] and read by
    /// [`cyclic_tx`], which the firmware calls with `&self` after the pipeline has run.
    pub cyclic_due: bool,
    /// Ticks elapsed since the last `CYCLIC_STATE` emission to the BLE port, the 5 Hz cadence's
    /// clock. [`cyclic_since_tx`](Self::cyclic_since_tx)'s twin, for the reason stated there and
    /// against [`BLE_CYCLIC_DIVISOR`].
    pub ble_cyclic_since_tx: u32,
    /// Whether this pass is an emitting one for the BLE port, read by [`ble_cyclic_tx`].
    pub ble_cyclic_due: bool,
    /// Ticks elapsed since the last forward of the peer's `CYCLIC_STATE` to the BLE port, the
    /// 2.5 Hz forward's clock against [`BLE_PEER_CYCLIC_TICKS`] (`specs/link-control.md`, "The
    /// peer's state on the BLE port: the forward"). Counted in ELAPSED TICKS (`dt_ticks`), not
    /// pipeline calls, so a catch-up burst of runs inside one tick forwards once.
    pub ble_peer_since_tx: u32,
    /// Whether this pass is a forwarding one, decided by [`ble_peer_since_tx`](Self::ble_peer_since_tx)
    /// reaching [`BLE_PEER_CYCLIC_TICKS`] and read by [`ble_peer_tx`], which the firmware calls with
    /// `&self` after the pipeline has run.
    pub ble_peer_due: bool,
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
    /// The motor-side fault level (`specs/motor-integration.md`, "the motor-side fault
    /// producers"): the hall dwell fault, period-liveness loss, and a refused offset calibration,
    /// folded by the firmware into one level and written here BEFORE each pass. It joins
    /// `fault_a`, so it drives SHUTDOWN and therefore disarm.
    ///
    /// A level, not a latch, at this boundary: the producers own their own stickiness (the ISR's
    /// fault bits are boot-sticky, the liveness level has its own hysteresis), and duplicating
    /// that here would make two places responsible for one fact. A board with no motor brought up
    /// leaves it false, so the pre-motor boot posture is byte-for-byte what it was.
    pub motor_fault: bool,
    /// The OFF-inhibit level (`specs/motor-integration.md`, "MOE enactment": `off_inhibit` gets its
    /// real producer at slice 5). True while the wheel is turning, derived by the firmware from the
    /// period ISR's `SPEED` word and written here BEFORE each pass, the [`Self::motor_fault`]
    /// pattern.
    ///
    /// It holds the machine in OFF: a vehicle whose wheel is already moving does not engage. It is
    /// deliberately NOT consulted in RUN (`state::ModeMachine`'s own rule), so it can never block a
    /// shutdown, and a wheel spun by hand can never stop a fault from shutting the bridge down. A
    /// board with no motor brought up leaves it false, so the pre-motor boot posture is unchanged.
    pub motor_moving: bool,
    /// The local battery sense (`specs/sensing-and-safety.md`, "The battery word", source rule
    /// 1): `Some` exactly when the plan carries `board.vbatt`, with the boot-read calibration and
    /// the 250 Hz filter. `None` = this board does not sense; its word comes from its peer or is
    /// UNKNOWN.
    pub vbatt: Option<battery::LocalSense>,
    /// The latest battery-sense count (12-bit, right-aligned; 0 = no conversion yet), copied by
    /// the firmware from the period ISR's `VBATT_RAW` word and written here BEFORE each pass, the
    /// [`Self::motor_fault`] pattern. Read only where [`Self::vbatt`] is `Some`.
    pub vbatt_raw: u16,
    /// Per-producer transition counters for the gating levels ([`events::FaultEvents`], the O1
    /// attribution instrument). Stepped once per control pass from the SAME levels the mode
    /// machine's inputs are assembled from, so a producer that asserts and releases inside one
    /// tick is still attributed.
    pub events: FaultEvents,
    /// This board's OWN gating-producer levels as of the last pass
    /// ([`events::Levels::own_mask`]): the `CYCLIC_STATE.fault` byte
    /// (`specs/rover-one-imu.md` section 4). Sole writer: the pipeline's level fold; sole reader:
    /// the cyclic builder. It is a word of its own rather than
    /// [`FaultEvents::levels`](events::FaultEvents::levels), because that is the GATING mask,
    /// which carries `peer_down` on `EV_COMMS_LOSS`'s bit, and a `peer_down` echoed back onto the
    /// wire latches both boards (`events::Levels::own_mask`).
    pub own_levels: u8,
    /// The RUN tilt cutout's LATCH (`specs/control.md` (l)): set on any tick the fused attitude
    /// says the machine is past `control::config::fsm::TILT_LIMIT` on either tilt axis, held, and
    /// released only on an OFF pass whose POWER REQUEST HAS DROPPED
    /// ([`ControlOutput::off_request_clear`]).
    ///
    /// Two consumers, the `over_current` pair: [`events::Levels::tilt`], so the board disarms and
    /// a rigid peer's `peer_down` stops its wheel too, and `FsmInputs::tilt` at both dispatch call
    /// sites, so the engagement machine drops to IDLE on the tick it asserts rather than one tick
    /// later with the OFF pass.
    ///
    /// **Why the release is the end of the ENGAGE and not the OFF pass** (where the `stop_all`
    /// latch and the fault latches release, and where `reset_engagement` runs): the mode machine's
    /// power request is a level re-seeded from the input every tick, and the rover's comes from
    /// the app's `INPUTS` mirror, which a fallen machine's owner is still holding. An OFF-pass
    /// release would therefore re-arm the machine within three ticks of it settling back under the
    /// limit, which is a machine righting itself and driving off unbidden. `arm::RE_READ_REFUSED`
    /// is held on the same seam for the same class of reason.
    ///
    /// It is not sticky beyond that: a machine still lying down re-asserts the level on the next
    /// tick, so the latch cannot be cleared out from under a fallen board.
    pub tilt_latch: bool,
    /// The last current window the period ISR closed ([`PhaseWindow`]), copied by the firmware
    /// from `motor::OBS_CURRENT` / `motor::OBS_MEAN_DUTY` before each pass, the
    /// [`Self::vbatt_raw`] pattern.
    ///
    /// Telemetry only: no control consumer reads it (the limiter acts in the 16 kHz ISR, which
    /// owns the words this copy comes from). A board with no motor brought up leaves it zeroed,
    /// and zeroed is the only reading there is: a board carrying no current. There is no
    /// "did not say" on the wire either, because every accepted `CYCLIC_STATE` carries the block
    /// ([`linkctl::CyclicState::obs`]).
    pub phase: PhaseWindow,
    /// The per-boot identity this board publishes ([`BoardIdentity`]): fixed at construction, so
    /// the firmware reads the boot counter and the detected part ONCE rather than every tick.
    pub identity: BoardIdentity,
}

/// The last completed 64-period current window, as the period ISR published it
/// (`specs/link-control.md`, the `CYCLIC_STATE` layout, offsets 11..17). One record, because the
/// three words are only meaningful together: the peak and the mean describe the SAME window, and
/// the duty is what was applied in the period that closed it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PhaseWindow {
    /// The window's peak phase-current magnitude, stock current counts.
    pub peak: i16,
    /// The window's mean magnitude, same counts. A calibration cross-check compares this one: a
    /// maximum over ADC samples reads high near the noise floor (`specs/rider-ui.md` 3.6).
    pub mean: i16,
    /// The on-duty applied in the period that closed the window, `0..ARR`; 0 for a period that
    /// coasted.
    pub duty_on: u16,
}

/// What a board says about ITSELF for the life of a boot (`specs/link-control.md`, the
/// `CYCLIC_STATE` layout, offsets 17 and 18): the boot counter's low byte, which tells a
/// controller that a board rebooted under it instead of asking an operator to confirm a power
/// cycle, and the part the detect probe identified, which tells the controller which capability
/// table predicts this board's layout verdict.
///
/// Both are constants of the boot, so they are constructor inputs and never tick work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardIdentity {
    /// The boot counter's LOW BYTE (`boot_count & 0xFF`, the `CTRL_OBS` ordinal). Enough to tell
    /// a board that rebooted from one that did not; a wrap needs exactly 256 boots between two
    /// observations.
    pub boot_tag: u8,
    /// The part `detect_chip` identified at boot.
    pub chip: ChipTag,
}

impl OrchestratorState {
    /// A fresh orchestrator: mode OFF, empty inbox, idle inputs, identity attitude, and the
    /// control dispatch through its boot seam (`ControlDispatch::new(CONTROL_MODE byte,
    /// CONTROL_FRAME byte, imu_configured, CONTROL_RIDER_REQUIRED byte, CONTROL_BATTERY_FLOOR)`:
    /// the resolution table of `specs/rover-one-imu.md` section 2, where pivoting Balance
    /// demotes to Throttle with the mode fault when the IMU is absent and rigid Balance leads
    /// with one and follows without). `imu_configured` comes from the boot path (plan-present AND probe-ok);
    /// `attitude_cfg` is the per-board attitude calibration (the reference defaults on an
    /// uncalibrated board). `vbatt_cal` is the battery-sense calibration where the plan carries
    /// `board.vbatt` (`None` = the board does not sense). `drive_lean` is the balance-mode drive
    /// input's seam-clamped bound and rate (`specs/control.md` (h); the default is disabled).
    /// `rider_required_byte` is the `CONTROL_RIDER_REQUIRED` byte (`specs/control.md` (i); the
    /// default 1 keeps the balance rider gate, 0 waives it), decoded by the same boot seam.
    /// `battery_floor` is the `CONTROL_BATTERY_FLOOR` word (`specs/sensing-and-safety.md`, "The
    /// low-battery floor": centivolts, `<= 0` = none), carried into the same dispatch.
    /// `identity` is what this board says about itself for the life of the boot
    /// ([`BoardIdentity`]: the boot tag and the detected part), a constructor input because both
    /// halves are fixed before the first tick and neither is ever recomputed.
    ///
    /// The argument list is the boot reads one for one (the firmware's `init_shell` precedent);
    /// bundling them into a struct for the lint would be a type with this one constructor.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        control_mode_byte: u8,
        frame_byte: u8,
        side_byte: u8,
        imu_configured: bool,
        attitude_cfg: attitude::Config,
        gains: control::GainShadow,
        vbatt_cal: Option<battery::VbattCal>,
        drive_lean: control::DriveLean,
        drive_turn: control::DriveTurn,
        rider_required_byte: u8,
        battery_floor: i16,
        identity: BoardIdentity,
    ) -> Self {
        let (ctl, block) = new_ctl(
            control_mode_byte,
            frame_byte,
            side_byte,
            imu_configured,
            gains,
            drive_lean,
            drive_turn,
            rider_required_byte,
            battery_floor,
        );
        OrchestratorState {
            mahony: attitude::Mahony::new(attitude_cfg),
            attitude: attitude::Output::default(),
            imu_configured,
            imu_live: false,
            imu_health: ImuHealth::new(),
            inbox: LinkInbox::new(),
            latches: [FaultLatch::new(); N_MOTORS],
            mode: ModeMachine::new(),
            button: inputs::LineBank::new(),
            button_pressed: false,
            pads: inputs::PadBank::new(),
            pad_field: 0,
            rider_present: false,
            control_ticks: 0,
            // Both cadences start DUE and both clocks start at zero, so a board emits on the
            // first read after construction and then on its own interval. That is the behaviour
            // the invocation-parity gate had at `control_ticks == 0`, kept deliberately: the
            // first emission is how a peer learns this board exists.
            cyclic_since_tx: 0,
            cyclic_due: true,
            ble_cyclic_since_tx: 0,
            ble_cyclic_due: true,
            // Starts at zero and is first due BLE_PEER_CYCLIC_TICKS after construction: unlike the
            // board's own emission there is no announce-yourself case, since no peer is known at
            // boot.
            ble_peer_since_tx: 0,
            ble_peer_due: false,
            input_ticks: 0,
            enact_inits: 0,
            enact_shutdowns: 0,
            ctl,
            block,
            motor_fault: false,
            motor_moving: false,
            vbatt: vbatt_cal.map(battery::LocalSense::new),
            vbatt_raw: 0,
            events: FaultEvents::default(),
            own_levels: 0,
            tilt_latch: false,
            phase: PhaseWindow::default(),
            identity,
        }
    }

    /// The assembled power-request level: debounced local button OR the remote `INPUTS` mirror
    /// bit (`specs/integration.md`, "The input task"; level semantics, either producer holds it).
    ///
    /// A board armed from its own button is unaffected by anything the link does: the mirror's
    /// staleness only withdraws the mirror's own half of this OR.
    pub fn power_request(&self) -> bool {
        self.button_pressed || self.inbox.remote_power_request()
    }

    /// Whether the firmware should attempt the blocking IMU read on this control tick (the retry
    /// breaker, `specs/integration.md` pipeline step 1). True normally (read every tick); once the
    /// breaker is open ([`IMU_LOSS_THRESHOLD`] consecutive fails) only every [`IMU_PROBE_CADENCE`]
    /// ticks, so the cost of a failing read is paid at the probe cadence instead of every 4 ms
    /// tick. Note what this does NOT do: the first [`IMU_LOSS_THRESHOLD`] passes after a bus goes
    /// bad all pay in full, before the breaker opens, so the per-pass cost has to be survivable on
    /// its own and the HAL's poll budget is what makes it so. A non-configured board never opens the breaker (absence is not
    /// loss), so every tick reads (and yields `None` for want of a bus, unchanged).
    pub fn imu_read_due(&self, tick: u32) -> bool {
        !self.imu_health.backoff() || tick.is_multiple_of(IMU_PROBE_CADENCE)
    }

    /// The over-current trip seam (`specs/motor-integration.md`, "The hard trip"): write the
    /// one-shot `CODE_OVERCURRENT` into motor `motor`'s fault-latch mailbox. The latch's own tick
    /// (pipeline step 4) consumes it on the next pass and latches one-way, which reaches the FSM as
    /// `over_current` and the mode machine as `latch_a`; it clears where every latch clears, on a
    /// pass whose resulting mode is OFF.
    ///
    /// The firmware calls it when the period ISR's trip count changes; it is a seam so the host
    /// drives the whole chain without the ISR. It is deliberately NOT the motor-side fault level
    /// ([`OrchestratorState::motor_fault`]): the trip reaches `fault_a` through the latch, and a
    /// second route would give one fact two owners.
    pub fn raise_over_current(&mut self, motor: usize) {
        self.latches[motor].fault_code = state::CODE_OVERCURRENT;
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
            roll_milli_deg: out_to_milli(self.attitude.roll_deg),
            imu_configured: self.imu_configured,
            imu_live: self.imu_live,
            imu_loss: self.imu_health.loss(),
            comms_loss: self.inbox.comms_loss(),
            cyclic_age: self.inbox.cyclic_age(),
            cyclic_age_peak: self.inbox.cyclic_age_peak(),
            drive_age: self.inbox.drive_age(),
            enact_inits: self.enact_inits,
            enact_shutdowns: self.enact_shutdowns,
            torque_setpoint: self.ctl.wheel_demand,
            sub_state: self.ctl.fsm.sub_state as u8,
            control_mode: self.ctl.dispatch.mode() as u8,
            mode_fault: dispatch::mode_fault_level(self),
            motor_fault: self.motor_fault,
            gating_field: self.block.gating_field,
            pre_env_torque: self.ctl.pre_env_ref,
            event_levels: self.events.levels(),
            event_counts: self.events.counts(),
            battery: self.block.battery,
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
    /// The latest attitude roll, millidegrees. Conditioned exactly as `pitch_milli_deg` (the same
    /// `out_to_milli` scale off the held Mahony output), so the hold-not-zeros semantics apply to
    /// roll identically: on a missing sample the filter holds and this reports the last-good roll
    /// rather than a zero (`specs/integration.md`, pipeline step 2). Published into `CTRL_OBS` for
    /// the hand-tilt roll-sign session; the ZYX roll sign itself is still an open question
    /// (`specs/attitude.md`, "Open questions"), so the tool reports the observed sign.
    pub roll_milli_deg: i32,
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
    /// The largest value `cyclic_age` has reached since boot
    /// ([`LinkInbox::cyclic_age_peak`]): the worst case recorded rather than sampled.
    pub cyclic_age_peak: u32,
    /// Ticks since the last accepted drive command.
    pub drive_age: u32,
    /// INIT bring-up records emitted through the enact seam ("last enact actions").
    pub enact_inits: u32,
    /// SHUTDOWN safe-down records emitted through the enact seam.
    pub enact_shutdowns: u32,
    /// The WHEEL demand this pass ([`dispatch::ControlCtl::wheel_demand`]): the engagement
    /// machine's torque setpoint through the rigid freshness gate, which is what the firmware
    /// publishes to `motor::DEMAND`. On a pivoting board it is the sole-writer row itself.
    pub torque_setpoint: i16,
    /// The engagement machine's sub-state byte (the `a_substate` the latches consume).
    pub sub_state: u8,
    /// The resolved reference producer (`control::Producer`: 0 = Throttle, 1 = Balance, 2 = rigid
    /// Leader, 3 = rigid Follower).
    pub control_mode: u8,
    /// The live mode-fault level ([`dispatch::mode_fault_level`]): the validation seam's demotion
    /// (pivoting Balance without an IMU), or a rigid leader hearing a second leader.
    pub mode_fault: bool,
    /// The motor-side fault level this pass (hall dwell fault, period-liveness loss, or a refused
    /// offset calibration), as folded into `fault_a`. OBS only: the producers are the firmware's
    /// motor module and the level arrives through [`OrchestratorState::motor_fault`].
    pub motor_fault: bool,
    /// The engagement machine's gating/pickup row: the CONDITIONED UP-AXIS ACCEL COUNT
    /// (`control::gating`; +-4 g at 8192 counts per g, so the machine's `> 500` engage edge is
    /// 0.061 g of gravity on the deck's up-axis and its `< 0` pickup edge is an inverted deck).
    ///
    /// Published because it is the balance era's first gate and its first failure mode: a board
    /// whose accel sign map is wrong reads NEGATIVE here at level rest, which cannot engage and
    /// (worse) sits on the pickup edge. A level, right-way-up board must read large and positive,
    /// and that is a check a disarmed bench read can make before any wheel is free
    /// (`specs/silicon-queue.md`, the accel-sign gate). It holds across a missed IMU sample
    /// exactly as the attitude words do.
    pub gating_field: i16,
    /// The PRE-ENVELOPE torque view: the reference the active mode arm fed the shared engagement
    /// machine this tick, before the machine's gating and soft-start envelope act on it
    /// ([`dispatch::ControlCtl::pre_env_ref`]).
    ///
    /// Distinct from [`Self::torque_setpoint`], which is the machine's OUTPUT and is therefore
    /// zero whenever the machine is disengaged. That is exactly the disarmed case, so the
    /// published torque word alone cannot answer "what would the balance controller command if it
    /// were engaged?". This word can: it is live every tick regardless of sub-state, which is what
    /// makes the disarmed tilt torque-shadow possible.
    pub pre_env_torque: i16,
    /// The live gating-producer level mask (`events::EV_*`), the O1 attribution instrument's
    /// level half.
    pub event_levels: u8,
    /// The per-producer saturating transition counts (`events::EV_*` bit order), the O1
    /// attribution instrument's count half.
    pub event_counts: [u8; N_EVENT_PRODUCERS],
    /// The block's effective battery word, centivolts, 0 = UNKNOWN (`specs/sensing-and-safety.md`,
    /// "The battery word"): the PID's divisor and the balance engage gate's input, published so
    /// the bench gate on it is a plain read.
    pub battery: i16,
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
    /// The wheel demand this tick (step 7's output: the sole-writer row's value through the rigid
    /// freshness gate, [`dispatch::ControlCtl::wheel_demand`]).
    pub torque_setpoint: i16,
    /// The engagement sub-state byte after this tick (0 forces a zero setpoint).
    pub sub_state: u8,
    /// This pass resolved to OFF **and** the power request is clear: the once-per-ENGAGE seam, as
    /// distinct from the once-per-OFF-pass one.
    ///
    /// Published rather than left to a consumer to recompute, because both halves are this
    /// crate's: the resulting mode is already tested here for the latch and engagement clears, and
    /// the request is [`OrchestratorState::power_request`]'s fold of the button and the `INPUTS`
    /// mirror. A consumer with only [`ControlOutput::mode_byte`] could reconstruct the first half
    /// and not the second.
    ///
    /// **Why the request half is load-bearing.** The request is a LEVEL, re-seeded from the input
    /// every tick, and the OFF gate is taken on the first tick it is on with no fault
    /// (`state::ModeMachine::tick`, pinned by `fault_raised_then_lowered_re_enters_init`). So a
    /// consumer that releases a hold on every OFF pass releases it again every 20 ms for as long as
    /// a rider leans on the button: OFF, INIT, READY, RUN, SHUTDOWN and back, with the enact
    /// records stepping each time. With the request half, the release is one per engage. The arm
    /// layer's retryable re-read refusal is the first consumer (`specs/integration.md`, "A refused
    /// re-read refuses the ARM, not the boot").
    ///
    /// It is true on the SHUTDOWN pass whose resulting mode is OFF, not only on a later dwell
    /// pass, which is the same "resulting mode" rule the latch clears use; with the request already
    /// clear, nothing can re-enter on the next tick either way.
    pub off_request_clear: bool,
}

/// One 250 Hz control pass over already-sampled inputs. `sample` is the IMU read's product
/// (`None` = not configured or the read failed; on `None` the attitude filter HOLDS, and for a
/// configured IMU the miss advances [`ImuHealth`] toward the loss fault).
///
/// Steps, in spec order: sample in (step 1); attitude plus the block's attitude/rate words
/// (step 2); inbox ages + snapshot (step 3); the fault latches with the a_substate tie
/// (step 4); input assembly + the mode machine, with `off_inhibit` taken from
/// [`OrchestratorState::motor_moving`], the wheel-motion level the firmware derives from the
/// period ISR's speed word (step 5); the enactment record (step 6); the control dispatch,
/// [`dispatch`] (step 7). The cyclic TX (step 8) is [`cyclic_tx`], called by the firmware with
/// the address fact. The OBS counters (step 9) update here; the snapshot is
/// [`OrchestratorState::obs`].
///
/// **`dt_ticks` is the SysTick ticks that actually elapsed since the previous call, and ZERO is a
/// legal value** (`specs/link-control.md`, "What a tick of the ages is"). The scheduler holds
/// missed slots in its `runcount` and the main loop drains them one per pass, so after an absence
/// this runs several times inside one tick; those runs are one service opportunity, and
/// everything here that measures TIME (the attitude integration, the link ages, the two emission
/// cadences) is gated on `dt_ticks > 0` rather than on being called. Everything that counts WORK
/// (`control_ticks`, the mode machine, the dispatch) still runs on every call.
pub fn control_task(
    state: &mut OrchestratorState,
    sample: Option<&imu::Sample>,
    dt_ticks: u32,
) -> ControlOutput {
    // Did a SysTick tick elapse since the previous call? Every time-measuring step below asks
    // this, and only this, so one catch-up burst cannot age anything.
    let served = dt_ticks > 0;
    // Step 1: IMU health + the sample. Live = configured AND read ok this tick. A configured
    // board tracks its read health: IMU_LOSS_THRESHOLD consecutive fails assert `imu_loss` (a
    // failing read must NOT silently freeze attitude under live torque, sanity-audit P0-1) and
    // open the retry breaker; absence (not configured) is never loss.
    //
    // Gated on `served`, with the rest of the time-measuring steps: the IMU is sampled once per
    // tick, so a catch-up run carries no sample and has nothing to say about read health. Without
    // the gate a burst would feed the breaker hundreds of consecutive MISSES inside one tick and
    // assert `imu_loss`, which is a `fault_a` producer, so the burst would trade one spurious
    // shutdown producer for another. `imu_live` holds its value for the same reason it holds on a
    // missing sample: it reports THIS TICK's read, and the tick has not changed.
    if served {
        state.imu_live = state.imu_configured && sample.is_some();
        state
            .imu_health
            .update(state.imu_configured, sample.is_some());
    }
    let imu_loss = state.imu_health.loss();

    // Step 2: attitude. On a real sample: gyro in rad/s, accel as sign-applied counts (direction
    // only, the attitude.md wiring), integrated dt-honest (round-4 defect B: over the ticks that
    // ACTUALLY elapsed, not an assumed 4 ms). On a MISSING sample (absence OR a failed read) the
    // filter HOLDS: it skips the update rather than integrating the zero sample (which froze
    // pitch/roll at their last values while the mode machine kept producing torque). The last-good
    // attitude and block words hold; `imu_live`/`imu_loss` make the staleness visible.
    //
    // A catch-up run (`!served`) never integrates, whatever it was handed: `update_dt` is a time
    // integral and no time has passed. Unguarded, one measured 294-run burst would have advanced
    // the filter by 294 ticks of gyro inside a few milliseconds.
    if let Some(s) = sample.filter(|_| served) {
        let accel = [
            Fix::from_num(s.accel_raw[0]),
            Fix::from_num(s.accel_raw[1]),
            Fix::from_num(s.accel_raw[2]),
        ];
        state.attitude = state.mahony.update_dt(s.gyro, accel, dt_ticks);
        // The block's attitude words (stock-native centidegrees) and the pitch-rate word (@0x9c,
        // rad/s x 10000 on the pitch axis, truncated toward zero: stock's
        // `f2iz(gy * GYRO_SCALE * 10000.0f)`, `specs/control.md` (j)).
        state.block.pitch_word = out_to_centi(state.attitude.pitch_deg);
        state.block.roll_word = out_to_centi(state.attitude.roll_deg);
        state.block.pitch_rate =
            q_to_int_d2iz(s.gyro[PITCH_RATE_AXIS] * Fix::from_num(10000)) as i32;
        // The gating/pickup row (`control::gating`): the conditioned UP-AXIS accel count, the
        // engagement machine's orientation gate. The sample's accel counts arrive ALREADY
        // sign-applied by the IMU crate's config (`sign_clamp` in `imu::Sample`, the canonical
        // owner of the board's accel wiring), so they are used directly here: applying a sign
        // map again would cancel it on negative-sign axes (the audit-caught double-application).
        // Stepped here, with the IMU tick that produces it, and held (like the attitude words)
        // when a sample is missing.
        state.block.gating_field = state
            .ctl
            .gating
            .tick(Fix::from_num(s.accel_raw[UP_AXIS] as i32));
    }

    // Step 3: the link inbox snapshot: bump the staleness ages, then read the levels. The ages
    // count SERVED TICKS, not calls ([`LinkInbox::tick_ages`]).
    state.inbox.tick_ages(served);
    let comms_loss = state.inbox.comms_loss();

    // Step 3b: the three emission cadences, on the same clock and for the same reason. A rate
    // expressed as a parity of `control_ticks` is not a rate once the scheduler drains missed
    // slots one per main-loop pass; expressed in ticks elapsed, a burst of any length still emits
    // one frame (`specs/link-control.md`, "Addressing and emission"). The flags are read after
    // this pipeline returns, by `cyclic_tx` / `ble_cyclic_tx` / `ble_peer_tx`, which the firmware
    // calls with the address fact and an immutable borrow.
    state.cyclic_since_tx = state.cyclic_since_tx.saturating_add(dt_ticks);
    state.cyclic_due = state.cyclic_since_tx >= CYCLIC_EMIT_TICKS;
    if state.cyclic_due {
        state.cyclic_since_tx = 0;
    }
    state.ble_cyclic_since_tx = state.ble_cyclic_since_tx.saturating_add(dt_ticks);
    state.ble_cyclic_due = state.ble_cyclic_since_tx >= BLE_CYCLIC_DIVISOR;
    if state.ble_cyclic_due {
        state.ble_cyclic_since_tx = 0;
    }
    state.ble_peer_since_tx = state.ble_peer_since_tx.saturating_add(dt_ticks);
    state.ble_peer_due = state.ble_peer_since_tx >= BLE_PEER_CYCLIC_TICKS;
    if state.ble_peer_due {
        state.ble_peer_since_tx = 0;
    }

    // The battery word's source rule (`specs/sensing-and-safety.md`, "The battery word"), after
    // the inbox ages so a peer's word is taken only while its mirror is fresh: the local filtered
    // sense where the plan carries one, else a fresh peer's nonzero word, else 0 = UNKNOWN. The
    // block row is the EFFECTIVE word every consumer reads (PID scale, engage gate, cyclic).
    let local = state
        .vbatt
        .as_mut()
        .map(|sense| sense.step(state.vbatt_raw));
    state.block.battery = battery::battery_source(local, state.inbox.peer(), !comms_loss);

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

    // Step 5: input assembly + the mode machine. comms_loss, stop_all, imu_loss and the motor-side
    // fault level fold into Fault A (the stock mapping: general/sensing faults) alongside motor
    // 0's latch; motor 1's latch is the Fault B producer group.
    //
    // The individual levels are named once here and used TWICE: folded into the aggregates the
    // mode machine consumes, and folded into the O1 attribution mask (`events`). One read per
    // producer, so the counters cannot disagree with the decision they explain.
    let stop_all = state.inbox.stop_all();
    let latch_a = state.latches[0].is_latched();
    let latch_b = state.latches[1].is_latched();
    // The tilt cutout's producer (`specs/control.md` (l)): the fused attitude words say the
    // machine is past the limit on a tilt axis, on a board whose IMU is configured (the only
    // board on which those words mean anything; a follower or a padless throttle kart has no
    // level of its own and stops through its peer or not at all). One-way here: the release is
    // the end-of-engage seam below.
    state.tilt_latch |=
        state.imu_configured && control::tilt_cutout(state.block.pitch_word, state.block.roll_word);
    let levels = events::Levels {
        comms_loss,
        stop_all,
        imu_loss,
        motor_fault: state.motor_fault,
        latch_a,
        latch_b,
        mode_fault: dispatch::mode_fault_level(state),
        power_request: state.power_request(),
        // The tilt cutout's level, folded onto `EV_STOP_ALL`'s bit in BOTH masks: this board is
        // latched stopped, which is what that bit already means, and a rigid peer must read it
        // off the wire to stop its own wheel (`events::Levels::tilt`).
        tilt: state.tilt_latch,
        // The rigid frame's one-wheel-fault interlock (`specs/rover-one-imu.md` section 6). It
        // folds into `fault_a` on `EV_COMMS_LOSS`'s bit and is the one level NOT published on
        // this board's own wire byte below.
        peer_down: dispatch::peer_down_level(state),
    };
    state.events.tick(levels.mask());
    state.own_levels = levels.own_mask();
    let mode_inputs = ModeInputs {
        power_request: levels.power_request,
        fault_a: levels.fault_a(),
        fault_b: levels.fault_b(),
        // The OFF-inhibit producer, live at last (`specs/motor-integration.md`, slice 5): the
        // wheel-motion level the firmware derived from the period ISR's SPEED word. Pre-motor it
        // was a hardcoded `false` for want of wheel speed.
        off_inhibit: state.motor_moving,
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
    //
    // The engagement machine resets on the SAME edge (`specs/motor-integration.md`,
    // prerequisite 4, a stage-4 gate): the FSM's stock abort inputs do not include `imu_loss`,
    // `stop_all`, the motor-side fault level or `fault_b`, so those producers shut the mode
    // machine down while leaving the engagement sub-state at RUN with its envelope at the cap.
    // Without this, re-entry to RUN would resume at full authority, skipping the soft-start.
    let off_pass = outcome.mode == state::Mode::Off;
    if off_pass {
        self_clear_stop_all(&mut state.inbox);
        for latch in state.latches.iter_mut() {
            *latch = FaultLatch::new();
        }
        dispatch::reset_engagement(&mut state.ctl);
    }

    // The end-of-engage seam: this pass resolved to OFF and the power request is clear. The tilt
    // latch releases HERE and not on the OFF pass above, so a machine that fell cannot re-arm
    // while its owner is still holding the app's power control
    // ([`OrchestratorState::tilt_latch`]); `arm::RE_READ_REFUSED` is released on the same seam.
    let off_request_clear = off_pass && !levels.power_request;
    if off_request_clear {
        state.tilt_latch = false;
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
        // The once-per-engage seam, as the tilt latch's release took it above. `power_request` is
        // the level the machine was given this pass, so this says the pass ended in OFF with
        // nothing asking to come back on.
        off_request_clear,
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

/// One 16 ms input pass (`specs/integration.md`, "The input task"): debounce the button and run
/// the pads into the rider level. The products land in [`OrchestratorState`] (`button_pressed`,
/// `pad_field`, `rider_present`) where the 250 Hz pipeline reads them.
pub fn input_task(state: &mut OrchestratorState, sample: &InputSample) {
    state
        .button
        .update(if sample.button_asserted { 0b1 } else { 0 });
    state.button_pressed = state.button.pressed(0);

    state.pad_field = state.pads.update(sample.pad_a_high, sample.pad_b_high);
    state.rider_present = state.pad_field == (inputs::PAD_A_BIT | inputs::PAD_B_BIT);

    state.input_ticks = state.input_ticks.wrapping_add(1);
}

#[cfg(test)]
mod tests;
