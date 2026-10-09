// SPDX-License-Identifier: GPL-3.0-or-later
//! Link-control payload codec (`specs/link-control.md`).
//!
//! The four inter-board control payload families ([`CyclicState`], [`DriveCmd`], [`Inputs`],
//! [`Fault`]) that ride L3 PDUs in the reserved control opcode block `0x10..0x2F`, plus the
//! supervision timeout constants. An L7 payload here is the payload of one L3 PDU
//! (`[opcode][src][dst][payload...]`, one PDU per L2 packet); this crate owns only the payload
//! bytes. `crates/net` keeps forwarding the block by `dst` without interpreting it, and
//! `crates/control` / `crates/state` keep consuming plain words: these types never leak into
//! their APIs.
//!
//! Conventions, per the spec's "Envelope and conventions":
//! - All multi-byte fields are **little-endian** (the stock exchange's big-endian layout is not
//!   carried).
//! - **Committed-prefix decode** (the archive precedent, kept): a decoder reads its committed
//!   prefix and ignores trailing bytes (fields append, never reorder), so a future build can
//!   append fields and an old build still decodes. A payload shorter than the committed prefix
//!   is rejected; the delivery class is best-effort / latest-wins, so the caller drops the PDU
//!   and no error propagates ([`decode`] returns `None`). [`CyclicState`] is the first family to
//!   USE that rule in both directions: its committed prefix is eleven bytes and this build
//!   appends the eight-byte [`CyclicObs`] block, so during a staged rollout a peer on the older
//!   image still decodes this build's emission, and this build still decodes that peer's eleven
//!   bytes with [`CyclicState::obs`] `None`.
//! - All four families are best-effort / latest-wins: no seq, no ack, no retransmit. Loss is
//!   handled by the cyclic cadence plus the supervision timeouts below.
//!
//! Recovered shapes: the struct/codec pattern follows the archived payload set
//! (`archive/accumulated-build:crates/link/src/payload.rs`) where it still fits; the opcode
//! numbering is the re-allocation from the spec's "Opcode allocation" table (the archive's
//! numbering partially collided with the reserved telemetry block and is superseded).
//!
//! `no_std`; host tests in the `#[cfg(test)]` module link `std` via the host target.

#![no_std]

// --- Opcode allocation (the reserved control block 0x10..0x2F) ---------------------------------

/// `CYCLIC_STATE` opcode: board <-> board on the inter-board UART (every 2nd control run, 125 Hz)
/// and board -> controller on the BLE port (every 50th, 5 Hz; `specs/link-control.md`).
pub const OP_CYCLIC_STATE: u8 = 0x10;

/// `DRIVE_CMD` opcode: controller -> board, on demand (phone/host rate).
pub const OP_DRIVE_CMD: u8 = 0x11;

/// `INPUTS` opcode: controller/peer -> board, on demand.
pub const OP_INPUTS: u8 = 0x12;

/// `FAULT` opcode: board -> peer, on latch edge.
pub const OP_FAULT: u8 = 0x13;

// --- Supervision timeouts (the link-loss fault producer's constants) ---------------------------

/// Peer-staleness trip, in 250 Hz ticks (100 ms): while the age of the last accepted
/// `CYCLIC_STATE` exceeds this, the `comms_loss` level asserts (feeding `ModeInputs.fault_a` and
/// `FsmInputs.comms_loss`). Level-sensitive: fresh cyclic clears it. A board that has never seen
/// a peer does NOT assert `comms_loss` (single-board operation is legitimate).
pub const CYCLIC_TIMEOUT_TICKS: u32 = 25;

/// Drive-staleness decay, in 250 Hz ticks (200 ms): with no fresh `DRIVE_CMD`, the throttle
/// reference decays to neutral. A reference-zeroing, not a fault: a controller letting go is
/// normal.
pub const DRIVE_TIMEOUT_TICKS: u32 = 50;

/// Remote-`INPUTS`-mirror staleness, in 250 Hz ticks (1.5 s): while the mirror's owner has gone
/// this long unheard, the mirror stops being a source at all. Every level it carries reads as
/// released (`power_request` clear, `rider` clear), so a controller that goes away cannot leave a
/// board holding an assertion it can no longer withdraw
/// (`specs/link-control.md`, "Supervision").
///
/// UNHEARD, not un-refreshed: the age is reset by any `INPUTS` or `DRIVE_CMD` from the node that
/// stated the level, while that level is still inside this window
/// (`orchestrator::LinkInbox::refreshes_mirror` owns the rule and the exclusions; content still
/// comes from `INPUTS` alone, and once expired only content revives it). The question this
/// constant answers is whether the
/// controller has gone away, so a rider streaming twenty demand frames a second answers it, and
/// answering it from the keepalive alone rested their arm on the app's slowest frame while its
/// fastest one was arriving and being enacted.
///
/// Why it is a third number rather than a reuse of either constant above:
///
/// - `CYCLIC_TIMEOUT_TICKS` (100 ms) supervises a WIRED inter-board UART running a fixed 125 Hz
///   cadence, where 100 ms is twelve missed frames and can only mean the peer is gone.
/// - `DRIVE_TIMEOUT_TICKS` (200 ms) bounds how long a demand may outlive its sender. A demand is
///   safe to drop early: the wheels simply stop.
/// - This mirror arrives over the BLE hop, where neither the sender's cadence nor the delivery is
///   a fixed frame period. Two figures set the floor. The SLOWEST REFRESH anyone plans to send:
///   the committed rider app streams `DRIVE_CMD` every 50 ms while connected and re-states the
///   level every 500 ms (`LinkConfig`), but a keepalive-only holder is legitimate and
///   `swd-mailbox-inputs --hold` is one, so the floor is still set by a bare repeat cadence, not
///   by the demand stream. And the observed DELIVERY PAUSES: phone-side connection-parameter
///   churn produces gaps over 200 ms with nothing lost at all, because the link retransmits and a
///   delayed frame arrives LATE rather than missing. So 1.5 s spans two 550 ms keepalive periods
///   with a churn pause on top (one lost keepalive plus a stall of the kind already seen).
///   Shorter windows start tripping on ordinary events: under 1.1 s a single lost keepalive at
///   the planned cadence disarms, under 750 ms a keepalive merely DELAYED by an observed churn
///   pause does, and a disarm mid-ride is a fall on a balancing machine. A late frame is not
///   evidence that a controller is gone.
///
/// Widening what refreshes it does NOT lower it, for two reasons. The keepalive-only holder above
/// still exists, so the floor the derivation rests on is unmoved. And the safety asymmetry runs
/// the other way from the usual one: what a shorter window buys is releasing an arm sooner, while
/// the runaway it might be imagined to bound is already bounded at 200 ms by `DRIVE_TIMEOUT_TICKS`
/// below, which is unchanged. So the whole benefit of shortening is that a board nobody is
/// commanding sits armed-but-coasting for less time, and the whole cost of shortening is a
/// balancing machine dropping a rider who is still aboard. What the wider refresh rule DID buy is
/// a much stronger guarantee at the same number: reaching 1.5 s now takes a total blackout of a
/// 20 Hz stream, thirty consecutive frames, where before it took two missed keepalives.
///
/// That margin is DERIVED, not measured: no run has yet held a real board armed over BLE for
/// minutes to see whether 1.5 s ever trips on its own, and the delivered rate through the CC2541
/// has never been characterised as a rate (the failures seen are bimodal, full cadence or a
/// dropped session). The instrument that decides this number is the negative control in
/// `specs/silicon-queue.md`, "Mirror staleness": minutes of ordinary streaming with no spurious
/// disarm. If that fires, the answer is a longer timeout here (the ordering below is the only
/// hard constraint on raising it) or a faster keepalive in the controller, never a bypass at the
/// read. Lowering it is a retune this fix makes CONCEIVABLE and no measurement yet supports;
/// it needs its own evidence, from the same instrument, that no legitimate holder falls inside
/// the shorter window.
///
/// It is deliberately LONGER than `DRIVE_TIMEOUT_TICKS`, and that ordering is the safety
/// property, not a coincidence: the demand always decays to zero (200 ms) well before the bridge
/// releases (1.5 s), so link loss stops the machine first and disarms it second, never the
/// reverse. Any future re-tuning has to preserve the ordering.
pub const INPUTS_TIMEOUT_TICKS: u32 = 375;

// The ordering above, held by the compiler rather than by whoever edits the numbers next: link
// loss must zero the demand before it releases the bridge.
const _: () = assert!(INPUTS_TIMEOUT_TICKS > DRIVE_TIMEOUT_TICKS);

// --- Codec plumbing ----------------------------------------------------------------------------

/// Reasons a payload decode can fail. The delivery class is best-effort, so the caller's response
/// to any decode failure is to drop the PDU; no error propagates further.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Fewer bytes than the committed prefix requires.
    TooShort,
}

#[inline]
fn rd_i16(b: &[u8], off: usize) -> i16 {
    i16::from_le_bytes([b[off], b[off + 1]])
}

#[inline]
fn rd_u16(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}

// --- CYCLIC_STATE (11 B committed + an 8 B appended block) -----------------------------------

/// The part a board reports in [`CyclicObs::chip`]: what `detect_chip` identified at boot
/// (`specs/link-control.md`, the `CYCLIC_STATE` layout, offset 18). A controller needs it to
/// know which capability table predicts this board's layout verdict, and the board is the only
/// thing that knows: the three fleet parts differ in pin bonding, gate maps and ADC channels,
/// and nothing else on the wire distinguishes them.
///
/// The tags are OUR allocation, not a silicon id: the GD32 parts carry no readable part number,
/// so the firmware derives the tag from what the detect probe MEASURED (the family discriminator
/// plus the per-instance advanced-timer count, [`ChipTag::from_detected`]).
///
/// [`ChipTag::Unknown`] is the fail-safe for a byte this build does not allocate, exactly as
/// [`DriveKind::Neutral`] is for an unknown kind byte: a controller that cannot name the part
/// must say so rather than assume one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ChipTag {
    /// The part is not named: a byte this build does not allocate.
    Unknown = 0,
    /// GD32F103C8, LQFP48: the bench F103 master and the 6-FET split boards.
    F103C8 = 1,
    /// GD32F130C8, LQFP48: the bench F130 slave and the offroad pair.
    F130C8 = 2,
    /// GD32F103RC, LQFP64: the 12-FET dual-motor mainboard, two advanced timers.
    F103RC = 3,
}

impl ChipTag {
    /// The raw wire byte.
    #[inline]
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// The tag a wire byte names; anything unallocated is [`ChipTag::Unknown`] (fail-safe).
    #[inline]
    pub const fn from_u8(b: u8) -> ChipTag {
        match b {
            1 => ChipTag::F103C8,
            2 => ChipTag::F130C8,
            3 => ChipTag::F103RC,
            _ => ChipTag::Unknown,
        }
    }

    /// The fleet part these two MEASURED detection facts name: whether the family probe matched
    /// F10x (false = F1x0) and the per-instance advanced-timer count the probe counted
    /// (`runtime_hal::McuDescriptor::adv_timers`).
    ///
    /// Those two answer it for the whole fleet: the F1x0 family has one 48-pin member here, and
    /// the two F10x members differ by exactly the second advanced timer (the 12-FET's TIMER7
    /// drives its second motor's gates, which is also why the count is the fact a layout cares
    /// about). The flash density would separate them too, but the timer count is the capability
    /// the parts are told apart BY everywhere else in this tree.
    ///
    /// A combination the fleet has no member for is [`ChipTag::Unknown`] rather than the nearest
    /// part: a guessed part is a wrong capability table, and the layout editor would predict a
    /// verdict the board will not give.
    #[inline]
    pub const fn from_detected(f10x_family: bool, adv_timers: u8) -> ChipTag {
        match (f10x_family, adv_timers) {
            (true, 1) => ChipTag::F103C8,
            (true, 2) => ChipTag::F103RC,
            (false, 1) => ChipTag::F130C8,
            _ => ChipTag::Unknown,
        }
    }
}

/// The APPENDED observation block (`specs/link-control.md`, the `CYCLIC_STATE` layout, offsets
/// 11..19): the current window a controller displays and cross-checks the board's calibration
/// against, and the two per-boot constants that date the rest of the payload.
///
/// It is not part of the committed prefix, so [`CyclicState::obs`] is `None` for a peer running
/// an image from before it existed (the staged-rollout case). Absent is not zero: a zeroed
/// current reading is a board carrying no current, and a consumer that cannot tell the two apart
/// would display 0.0 A for a board that never said.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CyclicObs {
    /// The last completed 64-period window's PEAK phase-current magnitude, stock current counts,
    /// as the firmware's `OBS_CURRENT` word already packs it.
    pub phase_peak: i16,
    /// The SAME window's MEAN magnitude, same counts. The peak is a maximum over ADC samples and
    /// so reads high near the noise floor, which is why a calibration cross-check compares the
    /// mean (`specs/rider-ui.md` 3.6). Both words describe one window, so the two are comparable.
    pub phase_mean: i16,
    /// The last applied on-duty, `0..ARR`, so a consumer can derive the DC-link current a bench
    /// PSU displays as `(mean / cal) * (duty_on / ARR)`. 0 for a period that coasted: no phase
    /// conducted in it.
    pub duty_on: u16,
    /// The boot counter's LOW BYTE, which is enough to tell a board that rebooted from one that
    /// did not. The full `u32` is not carried: a wrap needs exactly 256 boots between two
    /// observations.
    pub boot_tag: u8,
    /// The part `detect_chip` identified at boot ([`ChipTag`]).
    pub chip: ChipTag,
}

impl CyclicObs {
    /// On-wire length of the appended block.
    ///
    /// EIGHT, not the six the spec's prose beside the table says: the table's own five rows are
    /// `i16 + i16 + u16 + u8 + u8`, and its last offset is 18, so the block ends at 19. The table
    /// is the layout (the offsets and the types are what a decoder has to agree with); the "six
    /// bytes" sentence is a miscount of it, and the BLE budget stated there is derived from the
    /// same miscount (see `BLE_CYCLIC_DIVISOR` in `crates/orchestrator/src/dispatch.rs` for the
    /// re-derived figure).
    pub const LEN: usize = 8;

    /// Encode into `out` (the block alone, as it sits after the committed prefix), returning the
    /// byte count ([`Self::LEN`]).
    fn encode(&self, out: &mut [u8]) -> usize {
        out[0..2].copy_from_slice(&self.phase_peak.to_le_bytes());
        out[2..4].copy_from_slice(&self.phase_mean.to_le_bytes());
        out[4..6].copy_from_slice(&self.duty_on.to_le_bytes());
        out[6] = self.boot_tag;
        out[7] = self.chip.to_u8();
        Self::LEN
    }

    /// Decode the block from `b`, which must hold at least [`Self::LEN`] bytes.
    fn decode(b: &[u8]) -> CyclicObs {
        CyclicObs {
            phase_peak: rd_i16(b, 0),
            phase_mean: rd_i16(b, 2),
            duty_on: rd_u16(b, 4),
            boot_tag: b[6],
            chip: ChipTag::from_u8(b[7]),
        }
    }
}

// The block's declared length against the sum of its fields' widths, so a field that changes
// type cannot leave the length behind: i16 + i16 + u16 + u8 + u8.
const _: () = assert!(CyclicObs::LEN == 2 + 2 + 2 + 1 + 1);

/// The per-tick peer state mirror. The words are the RAM control block's stock-native words
/// (`specs/control.md` section (e)); no rescaling happens at the link boundary in either
/// direction.
///
/// Port-directed emission (dst `0x00`, inter-board UART port only) and the no-peer degradation
/// are the emitter's contract (`specs/link-control.md`, "Addressing and emission"), not this
/// codec's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CyclicState {
    /// Attitude pitch (stock CB+0x3a word). Peer consumer: peer-attitude mirror (OBS/telemetry).
    pub pitch: i16,
    /// Attitude roll (stock CB+0x3e word). Peer consumer: the shaper's `roll_b` roll-mirror
    /// input.
    pub roll: i16,
    /// Local wheel-speed word (stock CB+0x34). Peer consumer: the engagement blend's `ref_36`
    /// (peer speed).
    pub wheel_speed: i16,
    /// The board's EFFECTIVE battery word, centivolts, 0 = UNKNOWN (stock CB+0x20): its own
    /// filtered sense, else the word it relays from its peer. Peer consumer: the PID `scale` input
    /// on boards without VBATT sense (`orchestrator::battery`'s source rule 2).
    pub battery: u16,
    /// The mode byte. Peer consumer: supervision/OBS.
    pub mode: u8,
    /// Latched fault code, 0 = healthy. Peer consumer: visibility/OBS.
    pub fault: u8,
    /// Flag bits: [`Self::FLAG_RIDER`] (bit0), [`Self::FLAG_LOCKDOWN`] (bit7).
    pub flags: u8,
    /// The appended observation block ([`CyclicObs`]), or `None` from a peer whose image predates
    /// it. Every emitter in this tree fills it; the `None` exists for the RECEIVE path, where a
    /// peer may be running an older image through a staged rollout.
    pub obs: Option<CyclicObs>,
}

impl CyclicState {
    /// On-wire length of the committed prefix: the eleven bytes every build has carried, which a
    /// decoder requires and which [`Self::decode`] rejects a payload shorter than.
    pub const LEN: usize = 11;

    /// On-wire length this build EMITS: the committed prefix plus the appended [`CyclicObs`]
    /// block. A buffer handed to [`Self::encode`] has to be this long, and the committed prefix
    /// is what a DECODER requires, so the two are separate numbers.
    pub const ENCODED_LEN: usize = 19;

    /// `flags` bit0: rider present. Peer consumer: rider mirror (profile select).
    pub const FLAG_RIDER: u8 = 1 << 0;

    /// `flags` bit7: lockdown, the stock master-shutdown semantic. The receiver treats it as a
    /// gating fault into the engagement machine (sub-state forced to 0, torque setpoint 0) for
    /// as long as it is asserted (level, latest-wins).
    pub const FLAG_LOCKDOWN: u8 = 1 << 7;

    /// Rider-present flag (bit0).
    pub const fn rider_present(&self) -> bool {
        self.flags & Self::FLAG_RIDER != 0
    }

    /// Lockdown flag (bit7): lockdown -> immediate disengage on the receiver.
    pub const fn lockdown(&self) -> bool {
        self.flags & Self::FLAG_LOCKDOWN != 0
    }

    /// Encode into `out`, returning the byte count: [`Self::ENCODED_LEN`] with an appended block,
    /// [`Self::LEN`] without one. The count is what the caller puts on the wire, which is why it
    /// is returned rather than assumed.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0..2].copy_from_slice(&self.pitch.to_le_bytes());
        out[2..4].copy_from_slice(&self.roll.to_le_bytes());
        out[4..6].copy_from_slice(&self.wheel_speed.to_le_bytes());
        out[6..8].copy_from_slice(&self.battery.to_le_bytes());
        out[8] = self.mode;
        out[9] = self.fault;
        out[10] = self.flags;
        match self.obs {
            Some(obs) => {
                debug_assert!(out.len() >= Self::ENCODED_LEN);
                Self::LEN + obs.encode(&mut out[Self::LEN..])
            }
            None => Self::LEN,
        }
    }

    /// Decode the committed prefix, plus the appended block when the payload carries all of it;
    /// ignore trailing bytes.
    ///
    /// **A payload of exactly the committed prefix decodes, with `obs: None`.** That is the
    /// staged-rollout case and it is not an error: a peer running an image from before the block
    /// existed emits eleven bytes, every committed field of which is still exactly where this
    /// build expects it (the append-only rule). Rejecting it would silence a working peer's
    /// pitch, roll, battery and lockdown flag over a telemetry block, which is the wrong trade in
    /// the direction that matters: the lockdown flag is a safety level.
    ///
    /// A payload between the two lengths carries a PARTIAL block, which is nothing a sender in
    /// this tree can produce (the encode is all-or-none) and not something to half-read, so it
    /// decodes as absent too.
    pub fn decode(b: &[u8]) -> Result<CyclicState, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        Ok(CyclicState {
            pitch: rd_i16(b, 0),
            roll: rd_i16(b, 2),
            wheel_speed: rd_i16(b, 4),
            battery: rd_u16(b, 6),
            mode: b[8],
            fault: b[9],
            flags: b[10],
            obs: (b.len() >= Self::ENCODED_LEN).then(|| CyclicObs::decode(&b[Self::LEN..])),
        })
    }
}

// The emitted length is the prefix plus the block, held by the compiler rather than by whoever
// edits either number next.
const _: () = assert!(CyclicState::ENCODED_LEN == CyclicState::LEN + CyclicObs::LEN);

// --- DRIVE_CMD (5 B): a controller's drive reference -------------------------------------------

/// The `DRIVE_CMD.kind` discriminant. An unknown kind byte decodes as [`DriveKind::Neutral`]
/// (fail-safe).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DriveKind {
    /// Reference zero; `value`/`steer` are not live.
    Neutral = 0,
    /// `value`/`steer` live.
    Throttle = 1,
}

/// A controller's drive reference. Consumer: the throttle-mode reference producer
/// (`specs/integration.md`). Consumed from any port via normal L3 delivery; the firmware never
/// originates it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveCmd {
    /// Command kind; see [`DriveKind`].
    pub kind: DriveKind,
    /// Speed demand, `ControlDispatch::throttle_reference` input scale.
    pub value: i16,
    /// Steer demand, same scale.
    pub steer: i16,
}

impl DriveCmd {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 5;

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.kind as u8;
        out[1..3].copy_from_slice(&self.value.to_le_bytes());
        out[3..5].copy_from_slice(&self.steer.to_le_bytes());
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes. An unknown `kind` byte decodes as
    /// [`DriveKind::Neutral`] (fail-safe); `value`/`steer` are carried through but not live
    /// under `Neutral`.
    pub fn decode(b: &[u8]) -> Result<DriveCmd, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        let kind = match b[0] {
            1 => DriveKind::Throttle,
            _ => DriveKind::Neutral,
        };
        Ok(DriveCmd {
            kind,
            value: rd_i16(b, 1),
            steer: rd_i16(b, 3),
        })
    }
}

// --- INPUTS (2 B): remote input mirror ---------------------------------------------------------

/// Remote input mirror. Consumer: the input-assembly step (`ModeInputs.power_request` is
/// level-sensitive and copied over the link on a mirroring node, `specs/sensing-and-safety.md`).
/// Consumed from any port via normal L3 delivery; the firmware never originates it.
/// A remote carries LEVELS, not demand: `buttons` and `rider` are what a controller asserts about
/// itself, and the demand it wants arrives as [`DriveCmd`]. A raw throttle word used to lead this
/// payload, mirroring a board's own throttle HARDWARE (an ADC word, the same family as the button
/// and the pads) into an IIR nothing read; it is deleted (`specs/todo.md` part 3). The one thing
/// that made the deletion awkward is recorded here because it is the payload convention: fields
/// APPEND and never reorder, and this one was FIRST, so removing it shifted `buttons` and `rider`
/// and the firmware and the Kotlin mirror had to move together or the drift gate fails the build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inputs {
    /// Button bits: [`Self::BUTTON_POWER`] (bit0).
    pub buttons: u8,
    /// Rider bits: [`Self::RIDER_PRESENT`] (bit0).
    pub rider: u8,
}

impl Inputs {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 2;

    /// `buttons` bit0: power request (level).
    pub const BUTTON_POWER: u8 = 1 << 0;

    /// `rider` bit0: rider present.
    pub const RIDER_PRESENT: u8 = 1 << 0;

    /// Power-request level (`buttons` bit0).
    pub const fn power_request(&self) -> bool {
        self.buttons & Self::BUTTON_POWER != 0
    }

    /// Rider-present level (`rider` bit0).
    pub const fn rider_present(&self) -> bool {
        self.rider & Self::RIDER_PRESENT != 0
    }

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.buttons;
        out[1] = self.rider;
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
    pub fn decode(b: &[u8]) -> Result<Inputs, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        Ok(Inputs {
            buttons: b[0],
            rider: b[1],
        })
    }
}

// --- FAULT (2 B): latch-edge notification ------------------------------------------------------

/// Latch-edge notification, emitted once per latch edge (not cyclic; the level lives in
/// `CyclicState.fault`). Receiving [`Self::ACTION_STOP_ALL`] sets a local `stop_all` latch that
/// feeds `ModeInputs.fault_a`; it clears only when the mode machine passes through the OFF dwell
/// (the receiver's contract, not this codec's).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fault {
    /// The latched fault code (`state::fault` codes).
    pub code: u8,
    /// Action byte: [`Self::ACTION_NOTIFY`] or [`Self::ACTION_STOP_ALL`].
    pub action: u8,
}

impl Fault {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 2;

    /// `action` 0: notify only.
    pub const ACTION_NOTIFY: u8 = 0;

    /// `action` 1: STOP_ALL.
    pub const ACTION_STOP_ALL: u8 = 1;

    /// True when the action byte is exactly [`Self::ACTION_STOP_ALL`].
    pub const fn stop_all(&self) -> bool {
        self.action == Self::ACTION_STOP_ALL
    }

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.code;
        out[1] = self.action;
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
    pub fn decode(b: &[u8]) -> Result<Fault, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        Ok(Fault {
            code: b[0],
            action: b[1],
        })
    }
}

// --- Dispatch ----------------------------------------------------------------------------------

/// A decoded control-block payload, tagged by family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Payload {
    /// [`OP_CYCLIC_STATE`].
    CyclicState(CyclicState),
    /// [`OP_DRIVE_CMD`].
    DriveCmd(DriveCmd),
    /// [`OP_INPUTS`].
    Inputs(Inputs),
    /// [`OP_FAULT`].
    Fault(Fault),
}

/// Decode a delivered control-block PDU payload by opcode (the firmware's routing entry for the
/// `0x10..0x2F` hand-back, `specs/integration.md`). Returns `None` for an opcode this crate does
/// not allocate or a payload shorter than the family's committed prefix: the delivery class is
/// best-effort, so the PDU is simply dropped and no error propagates.
pub fn decode(opcode: u8, payload: &[u8]) -> Option<Payload> {
    match opcode {
        OP_CYCLIC_STATE => CyclicState::decode(payload).ok().map(Payload::CyclicState),
        OP_DRIVE_CMD => DriveCmd::decode(payload).ok().map(Payload::DriveCmd),
        OP_INPUTS => Inputs::decode(payload).ok().map(Payload::Inputs),
        OP_FAULT => Fault::decode(payload).ok().map(Payload::Fault),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    // -- Allocation and constants (pinned to the spec's tables) ---------------------------------

    #[test]
    fn opcode_allocation_pinned() {
        assert_eq!(OP_CYCLIC_STATE, 0x10);
        assert_eq!(OP_DRIVE_CMD, 0x11);
        assert_eq!(OP_INPUTS, 0x12);
        assert_eq!(OP_FAULT, 0x13);
        // All four live in the reserved control block 0x10..0x2F.
        for op in [OP_CYCLIC_STATE, OP_DRIVE_CMD, OP_INPUTS, OP_FAULT] {
            assert!((0x10..0x2F).contains(&op));
        }
    }

    #[test]
    fn supervision_constants_pinned() {
        // 25 ticks = 100 ms at 250 Hz; 50 ticks = 200 ms; 375 ticks = 1.5 s.
        assert_eq!(CYCLIC_TIMEOUT_TICKS, 25);
        assert_eq!(DRIVE_TIMEOUT_TICKS, 50);
        assert_eq!(INPUTS_TIMEOUT_TICKS, 375);
    }

    #[test]
    fn committed_lengths_pinned() {
        assert_eq!(CyclicState::LEN, 11);
        assert_eq!(DriveCmd::LEN, 5);
        assert_eq!(Inputs::LEN, 2);
        assert_eq!(Fault::LEN, 2);
    }

    /// The appended block's length and the emitted length, which are NOT the committed prefix:
    /// the prefix is what a decoder requires of a sender, and these are what this build writes.
    #[test]
    fn appended_block_lengths_pinned() {
        assert_eq!(CyclicObs::LEN, 8, "i16 + i16 + u16 + u8 + u8");
        assert_eq!(CyclicState::ENCODED_LEN, 19);
        assert_eq!(CyclicState::ENCODED_LEN, CyclicState::LEN + CyclicObs::LEN);
    }

    // -- Wire layout (byte-exact, little-endian) ------------------------------------------------

    fn cyclic_sample() -> CyclicState {
        CyclicState {
            pitch: -2,       // 0xFFFE
            roll: 0x0102,    // LE 02 01
            wheel_speed: -1, // 0xFFFF
            battery: 0xA1B2, // LE B2 A1
            mode: 0x03,
            fault: 0x11,
            flags: CyclicState::FLAG_RIDER | CyclicState::FLAG_LOCKDOWN,
            obs: Some(CyclicObs {
                phase_peak: 0x0304, // LE 04 03
                phase_mean: -3,     // 0xFFFD
                duty_on: 0x08C1,    // LE C1 08
                boot_tag: 0x7B,
                chip: ChipTag::F130C8,
            }),
        }
    }

    #[test]
    fn cyclic_state_wire_layout_is_little_endian() {
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        assert_eq!(cyclic_sample().encode(&mut buf), CyclicState::ENCODED_LEN);
        assert_eq!(
            buf,
            [
                0xFE, 0xFF, // pitch -2
                0x02, 0x01, // roll 0x0102
                0xFF, 0xFF, // wheel_speed -1
                0xB2, 0xA1, // battery 0xA1B2
                0x03, // mode
                0x11, // fault
                0x81, // flags: bit0 | bit7
                // The appended block, from offset 11.
                0x04, 0x03, // phase_peak 0x0304
                0xFD, 0xFF, // phase_mean -3
                0xC1, 0x08, // duty_on 0x08C1
                0x7B, // boot_tag
                0x02, // chip: F130C8
            ]
        );
    }

    /// The committed prefix is byte-for-byte what it was BEFORE the block existed, which is the
    /// whole offset-preserving claim: the same eleven bytes, from a payload that now carries
    /// eight more.
    #[test]
    fn the_appended_block_moves_no_committed_byte() {
        let mut long = [0u8; CyclicState::ENCODED_LEN];
        let n = cyclic_sample().encode(&mut long);
        assert_eq!(n, CyclicState::ENCODED_LEN);

        let mut short = [0u8; CyclicState::ENCODED_LEN];
        let legacy = CyclicState {
            obs: None,
            ..cyclic_sample()
        };
        assert_eq!(legacy.encode(&mut short), CyclicState::LEN);
        assert_eq!(long[..CyclicState::LEN], short[..CyclicState::LEN]);
    }

    #[test]
    fn drive_cmd_wire_layout_is_little_endian() {
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: -300,   // 0xFED4
            steer: 0x1234, // LE 34 12
        };
        let mut buf = [0u8; DriveCmd::LEN];
        assert_eq!(cmd.encode(&mut buf), DriveCmd::LEN);
        assert_eq!(buf, [0x01, 0xD4, 0xFE, 0x34, 0x12]);
    }

    #[test]
    fn inputs_wire_layout_is_little_endian() {
        let inp = Inputs {
            buttons: Inputs::BUTTON_POWER,
            rider: Inputs::RIDER_PRESENT,
        };
        let mut buf = [0u8; Inputs::LEN];
        assert_eq!(inp.encode(&mut buf), Inputs::LEN);
        assert_eq!(buf, [0x01, 0x01]);
    }

    #[test]
    fn fault_wire_layout() {
        let f = Fault {
            code: 0x21,
            action: Fault::ACTION_STOP_ALL,
        };
        let mut buf = [0u8; Fault::LEN];
        assert_eq!(f.encode(&mut buf), Fault::LEN);
        assert_eq!(buf, [0x21, 0x01]);
    }

    // -- Round trips ----------------------------------------------------------------------------

    #[test]
    fn cyclic_state_round_trip() {
        let orig = cyclic_sample();
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        orig.encode(&mut buf);
        assert_eq!(CyclicState::decode(&buf), Ok(orig));
    }

    /// THE STAGED-ROLLOUT CASE: a peer running an image from before the appended block emits
    /// eleven bytes, and they decode, with the block absent. The committed fields all survive;
    /// the lockdown flag in particular is a safety level and must not be lost over a telemetry
    /// block.
    #[test]
    fn an_eleven_byte_peer_decodes_with_the_block_absent() {
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        let n = CyclicState {
            obs: None,
            ..cyclic_sample()
        }
        .encode(&mut buf);
        assert_eq!(n, CyclicState::LEN);

        let got = CyclicState::decode(&buf[..n]).expect("the committed prefix decodes");
        assert_eq!(got.obs, None, "the peer did not say");
        assert!(got.lockdown(), "the committed flags are still read");
        assert_eq!(got.battery, 0xA1B2);
        // Absent is not zero: the one thing a consumer must be able to tell apart.
        assert_ne!(
            got.obs,
            Some(CyclicObs {
                phase_peak: 0,
                phase_mean: 0,
                duty_on: 0,
                boot_tag: 0,
                chip: ChipTag::Unknown,
            })
        );
    }

    /// A payload between the two lengths carries a partial block, which no sender here produces
    /// (the encode is all-or-none) and which is read as absent rather than half-decoded.
    #[test]
    fn a_partial_appended_block_decodes_as_absent() {
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        cyclic_sample().encode(&mut buf);
        for len in CyclicState::LEN..CyclicState::ENCODED_LEN {
            let got = CyclicState::decode(&buf[..len]).expect("the prefix is whole");
            assert_eq!(got.obs, None, "{len} bytes is a partial block");
        }
    }

    /// The chip byte's mapping, both ways, including the fail-safe: an unallocated byte is
    /// `Unknown`, so a controller that meets a part this build does not know says so.
    #[test]
    fn chip_tag_maps_both_ways() {
        for (tag, byte) in [
            (ChipTag::Unknown, 0u8),
            (ChipTag::F103C8, 1),
            (ChipTag::F130C8, 2),
            (ChipTag::F103RC, 3),
        ] {
            assert_eq!(tag.to_u8(), byte);
            assert_eq!(ChipTag::from_u8(byte), tag);
        }
        for byte in [4u8, 0x7F, 0xFF] {
            assert_eq!(ChipTag::from_u8(byte), ChipTag::Unknown, "byte {byte:#04x}");
        }
    }

    /// The fleet's three parts from the two facts the detect probe MEASURES, and `Unknown` for a
    /// combination no fleet part has, rather than the nearest one.
    #[test]
    fn chip_tag_from_the_measured_detection_facts() {
        assert_eq!(ChipTag::from_detected(true, 1), ChipTag::F103C8);
        assert_eq!(ChipTag::from_detected(true, 2), ChipTag::F103RC);
        assert_eq!(ChipTag::from_detected(false, 1), ChipTag::F130C8);
        // No fleet member: an F1x0 with two advanced timers, or a count the probe cannot have
        // measured. A guessed part is a wrong capability table.
        assert_eq!(ChipTag::from_detected(false, 2), ChipTag::Unknown);
        assert_eq!(ChipTag::from_detected(true, 0), ChipTag::Unknown);
        assert_eq!(ChipTag::from_detected(false, 0), ChipTag::Unknown);
    }

    #[test]
    fn drive_cmd_round_trip_both_kinds() {
        for kind in [DriveKind::Neutral, DriveKind::Throttle] {
            let orig = DriveCmd {
                kind,
                value: -12345,
                steer: 6789,
            };
            let mut buf = [0u8; DriveCmd::LEN];
            orig.encode(&mut buf);
            assert_eq!(DriveCmd::decode(&buf), Ok(orig));
        }
    }

    #[test]
    fn inputs_round_trip() {
        let orig = Inputs {
            buttons: 0xFF,
            rider: 0x01,
        };
        let mut buf = [0u8; Inputs::LEN];
        orig.encode(&mut buf);
        assert_eq!(Inputs::decode(&buf), Ok(orig));
    }

    #[test]
    fn fault_round_trip() {
        let orig = Fault {
            code: 0x11,
            action: Fault::ACTION_NOTIFY,
        };
        let mut buf = [0u8; Fault::LEN];
        orig.encode(&mut buf);
        assert_eq!(Fault::decode(&buf), Ok(orig));
    }

    // -- Committed-prefix rule: trailing bytes ignored ------------------------------------------

    #[test]
    fn trailing_bytes_are_ignored_every_family() {
        // Encode each family into an oversized buffer with poisoned trailing bytes; the decode
        // must read only the committed prefix and match the original.
        let mut buf = [0xEEu8; 32];

        let cyc = cyclic_sample();
        cyc.encode(&mut buf);
        assert_eq!(CyclicState::decode(&buf), Ok(cyc));

        let mut buf = [0xEEu8; 32];
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: 5,
            steer: -5,
        };
        cmd.encode(&mut buf);
        assert_eq!(DriveCmd::decode(&buf), Ok(cmd));

        let mut buf = [0xEEu8; 32];
        let inp = Inputs {
            buttons: 0,
            rider: 1,
        };
        inp.encode(&mut buf);
        assert_eq!(Inputs::decode(&buf), Ok(inp));

        let mut buf = [0xEEu8; 32];
        let flt = Fault {
            code: 0x21,
            action: 0,
        };
        flt.encode(&mut buf);
        assert_eq!(Fault::decode(&buf), Ok(flt));
    }

    // -- Committed-prefix rule: short payloads rejected -----------------------------------------

    #[test]
    fn short_payloads_are_rejected() {
        let buf = [0u8; 16];
        // One byte short of each family's committed prefix, and empty.
        for len in [CyclicState::LEN - 1, 0] {
            assert_eq!(CyclicState::decode(&buf[..len]), Err(DecodeError::TooShort));
        }
        for len in [DriveCmd::LEN - 1, 0] {
            assert_eq!(DriveCmd::decode(&buf[..len]), Err(DecodeError::TooShort));
        }
        for len in [Inputs::LEN - 1, 0] {
            assert_eq!(Inputs::decode(&buf[..len]), Err(DecodeError::TooShort));
        }
        for len in [Fault::LEN - 1, 0] {
            assert_eq!(Fault::decode(&buf[..len]), Err(DecodeError::TooShort));
        }
    }

    // -- DRIVE_CMD fail-safe --------------------------------------------------------------------

    #[test]
    fn unknown_drive_kind_decodes_as_neutral() {
        for kind_byte in [2u8, 0x7F, 0xFF] {
            let buf = [kind_byte, 0xD4, 0xFE, 0x34, 0x12];
            let got = DriveCmd::decode(&buf).unwrap();
            assert_eq!(got.kind, DriveKind::Neutral, "kind byte {kind_byte:#04x}");
            // The words are carried through (not live under Neutral).
            assert_eq!(got.value, -300);
            assert_eq!(got.steer, 0x1234);
        }
        // The defined kind bytes map exactly.
        assert_eq!(
            DriveCmd::decode(&[0, 0, 0, 0, 0]).unwrap().kind,
            DriveKind::Neutral
        );
        assert_eq!(
            DriveCmd::decode(&[1, 0, 0, 0, 0]).unwrap().kind,
            DriveKind::Throttle
        );
    }

    // -- Flag-bit extraction --------------------------------------------------------------------

    #[test]
    fn cyclic_flag_bits_extract() {
        let mut c = cyclic_sample();
        c.flags = 0;
        assert!(!c.rider_present());
        assert!(!c.lockdown());
        c.flags = CyclicState::FLAG_RIDER;
        assert!(c.rider_present());
        assert!(!c.lockdown());
        c.flags = CyclicState::FLAG_LOCKDOWN;
        assert!(!c.rider_present());
        assert!(c.lockdown());
        // Foreign bits do not bleed into the defined flags.
        c.flags = !(CyclicState::FLAG_RIDER | CyclicState::FLAG_LOCKDOWN);
        assert!(!c.rider_present());
        assert!(!c.lockdown());
    }

    #[test]
    fn inputs_flag_bits_extract() {
        let mut i = Inputs {
            buttons: 0,
            rider: 0,
        };
        assert!(!i.power_request());
        assert!(!i.rider_present());
        i.buttons = Inputs::BUTTON_POWER;
        i.rider = Inputs::RIDER_PRESENT;
        assert!(i.power_request());
        assert!(i.rider_present());
        // Only bit0 is defined on each byte.
        i.buttons = 0xFE;
        i.rider = 0xFE;
        assert!(!i.power_request());
        assert!(!i.rider_present());
    }

    #[test]
    fn fault_action_extracts() {
        assert!(!Fault {
            code: 0x11,
            action: Fault::ACTION_NOTIFY
        }
        .stop_all());
        assert!(Fault {
            code: 0x11,
            action: Fault::ACTION_STOP_ALL
        }
        .stop_all());
        // Only the exact STOP_ALL byte triggers; an unknown action byte stays notify-only.
        assert!(!Fault {
            code: 0x11,
            action: 2
        }
        .stop_all());
    }

    // -- Dispatch (the 0x10..0x2F hand-back routing entry) --------------------------------------

    #[test]
    fn dispatch_routes_each_opcode() {
        let cyc = cyclic_sample();
        let mut buf = [0u8; 24];
        cyc.encode(&mut buf);
        assert_eq!(
            decode(OP_CYCLIC_STATE, &buf[..CyclicState::ENCODED_LEN]),
            Some(Payload::CyclicState(cyc))
        );

        let mut buf = [0u8; 16];
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: 1,
            steer: 2,
        };
        cmd.encode(&mut buf);
        assert_eq!(
            decode(OP_DRIVE_CMD, &buf[..DriveCmd::LEN]),
            Some(Payload::DriveCmd(cmd))
        );

        let inp = Inputs {
            buttons: 1,
            rider: 0,
        };
        inp.encode(&mut buf);
        assert_eq!(
            decode(OP_INPUTS, &buf[..Inputs::LEN]),
            Some(Payload::Inputs(inp))
        );

        let flt = Fault {
            code: 0x21,
            action: 1,
        };
        flt.encode(&mut buf);
        assert_eq!(
            decode(OP_FAULT, &buf[..Fault::LEN]),
            Some(Payload::Fault(flt))
        );
    }

    #[test]
    fn dispatch_drops_unallocated_and_short() {
        let buf = [0u8; 16];
        // Unallocated opcodes: elsewhere in the control block, the telemetry block, and outside.
        for op in [0x14u8, 0x2E, 0x40, 0x00, 0xFF] {
            assert_eq!(decode(op, &buf), None, "opcode {op:#04x}");
        }
        // Short payloads drop (no error propagates), per the best-effort class.
        assert_eq!(decode(OP_CYCLIC_STATE, &buf[..CyclicState::LEN - 1]), None);
        assert_eq!(decode(OP_DRIVE_CMD, &buf[..DriveCmd::LEN - 1]), None);
        assert_eq!(decode(OP_INPUTS, &buf[..Inputs::LEN - 1]), None);
        assert_eq!(decode(OP_FAULT, &buf[..Fault::LEN - 1]), None);
    }
}
