// SPDX-License-Identifier: GPL-3.0-or-later
//! Link-control payload codec (`specs/link-control.md`).
//!
//! The three inter-board control payload families ([`CyclicState`], [`DriveCmd`], [`Inputs`])
//! and the trace pair ([`FocTrace`], [`TraceSet`], with the board's [`TraceSubscription`]) that
//! ride L3 PDUs in the reserved control opcode block `0x10..0x2F`, plus the supervision timeout
//! constants. An L7 payload here is the payload of one L3 PDU
//! (`[opcode][src][dst][payload...]`, one PDU per L2 packet); this crate owns only the payload
//! bytes. `shared/net` keeps forwarding the block by `dst` without interpreting it, and
//! `motorcontroller/control` / `motorcontroller/state` keep consuming plain words: these types never leak into
//! their APIs.
//!
//! Conventions, per the spec's "Envelope and conventions":
//! - All multi-byte fields are **little-endian** (the stock exchange's big-endian layout is not
//!   carried).
//! - **Committed-prefix decode** (the archive precedent, kept) for [`DriveCmd`] and [`Inputs`]:
//!   a decoder reads its committed prefix and ignores trailing bytes (fields append,
//!   never reorder), so a future build can append fields and an old build still decodes. A
//!   payload shorter than the committed prefix is rejected; the delivery class is best-effort /
//!   latest-wins, so the caller drops the PDU and no error propagates ([`decode`] returns
//!   `None`).
//! - **[`CyclicState`] is decoded by EXACT LENGTH**: [`CyclicState::ENCODED_LEN`], 34 bytes (the
//!   sixteen committed bytes, the appended [`CyclicObs`] block, the `wheel_rate` and the `machine`
//!   digest) from a one-motor board, or [`CyclicState::TWO_MOTOR_LEN`], 46 bytes, from a two-motor
//!   board, which appends motor 1's words ([`CyclicMotor`]). Any other length is a
//!   protocol error. It once accepted eleven bytes with the block absent, for a peer on an image
//!   from before the block; nothing in this project owes an older image compatibility until a
//!   version is released, and boards and phones are updated together (owner policy, 2026-10-10).
//!   The other argument for the tolerance, that a truncated frame must not cost the lockdown
//!   flag, was wrong on the framing: L2 drops a frame whose CRC-16 fails and resyncs at the next
//!   SOF (`specs/l2.md`), so a truncated frame never reaches this decoder. An eleven-byte payload
//!   can only come from a peer that deliberately sent eleven, which is the compatibility case.
//! - All three families are best-effort / latest-wins: no seq, no ack, no retransmit. Loss is
//!   handled by the cyclic cadence plus the supervision timeouts below.
//!
//! Recovered shapes: the struct/codec pattern follows the archived payload set
//! (`archive/accumulated-build:shared/link/src/payload.rs`) where it still fits; the opcode
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

/// `FOC_TRACE` opcode: board -> controller, on a `TRACE_SET` subscription, every Nth 64-period
/// window close, on the mailbox port only (`specs/link-control.md`, "`FOC_TRACE` (0x15, 46 B)").
pub const OP_FOC_TRACE: u8 = 0x15;

/// `TRACE_SET` opcode: controller -> board, the trace subscription and its keepalive
/// (`specs/link-control.md`, "`TRACE_SET` (0x16, 2 B)").
pub const OP_TRACE_SET: u8 = 0x16;

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
///   the committed remote app streams `DRIVE_CMD` every 50 ms while connected and re-states the
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

/// The trace subscription's expiry, in 250 Hz ticks (1.5 s): a subscription lasts this long from
/// the last `TRACE_SET` with a non-zero divisor (`specs/link-control.md`, "`TRACE_SET`"), so a
/// killed tool, a phone that left its scope screen or a dropped link stops the stream by itself. A
/// subscriber renews at 2 Hz. The `INPUTS` figure, for the same reason (a level with an expiry),
/// and not a safety bound: the stream drives nothing.
pub const TRACE_TIMEOUT_TICKS: u32 = 375;

// --- Codec plumbing ----------------------------------------------------------------------------

/// Reasons a payload decode can fail. The delivery class is best-effort, so the caller's response
/// to any decode failure is to drop the PDU; no error propagates further.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Fewer bytes than the committed prefix requires ([`DriveCmd`], [`Inputs`]).
    TooShort,
    /// Not a length the payload has: [`CyclicState`] is decoded by exact length, so a payload of
    /// anything but [`CyclicState::ENCODED_LEN`] or [`CyclicState::TWO_MOTOR_LEN`] lands here,
    /// short or long.
    WrongLen,
}

#[inline]
fn rd_i16(b: &[u8], off: usize) -> i16 {
    i16::from_le_bytes([b[off], b[off + 1]])
}

#[inline]
fn rd_u16(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}

#[inline]
fn rd_u32(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

#[inline]
fn rd_i32(b: &[u8], off: usize) -> i32 {
    rd_u32(b, off) as i32
}

// --- CYCLIC_STATE (34 B: 16 B committed + a 12 B appended block + the i32 wheel_rate + the u16
// machine digest; + the 12 B motor-1 block = 46 B from a two-motor board) -------------------------

/// The part a board reports in [`CyclicObs::chip`]: the chip its image is built for
/// (`specs/link-control.md`, the `CYCLIC_STATE` layout, offset 23). A controller needs it to
/// know which capability table predicts this board's layout verdict, and the board is the only
/// thing that knows: the three fleet parts differ in pin bonding, gate maps and ADC channels,
/// and nothing else on the wire distinguishes them.
///
/// The tags are OUR allocation, not a silicon id: the GD32 parts carry no readable part number,
/// so each image reports the tag its chip's descriptor carries (`hal::descriptor::DESCRIPTOR`'s
/// `chip_tag`, specs/hal.md section 4.2).
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
}

/// The APPENDED observation block (`specs/link-control.md`, the `CYCLIC_STATE` layout, offsets
/// 16..28): the current window a controller displays and cross-checks the board's calibration
/// against, the two per-boot constants that date the rest of the payload, and this wheel's
/// odometer.
///
/// Every `CYCLIC_STATE` carries it ([`CyclicState::obs`] is not an option), so a zeroed window
/// has exactly one reading: a board carrying no current. A board with no current sense, or one
/// whose first window has not closed, reports zeros and still reports its identity bytes, which
/// are constants of its boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CyclicObs {
    /// The last completed 64-period window's PEAK phase-current magnitude, stock current counts,
    /// as the firmware's `OBS_CURRENT` word already packs it.
    pub phase_peak: i16,
    /// The SAME window's MEAN magnitude, same counts. The peak is a maximum over ADC samples and
    /// so reads high near the noise floor, which is why a calibration cross-check compares the
    /// mean (`specs/remote-ui.md` 3.6). Both words describe one window, so the two are comparable.
    pub phase_mean: i16,
    /// The last applied on-duty, `0..ARR`, so a consumer can derive the DC-link current a bench
    /// PSU displays as `(mean / cal) * (duty_on / ARR)`. 0 for a period that coasted: no phase
    /// conducted in it.
    pub duty_on: u16,
    /// The boot counter's LOW BYTE, which is enough to tell a board that rebooted from one that
    /// did not. The full `u32` is not carried: a wrap needs exactly 256 boots between two
    /// observations.
    pub boot_tag: u8,
    /// The part the board's image is built for ([`ChipTag`]).
    pub chip: ChipTag,
    /// This wheel's LIVE lifetime travel, in ABSOLUTE hall edges: the total the board's store
    /// last recorded plus every edge counted since it booted (`specs/odometer.md`). Reversing
    /// adds, so it never winds back.
    ///
    /// COUNTS, never a distance. Converting needs a pole-pair count and a wheel diameter
    /// (`store::MOTOR_POLE_PAIRS` and `store::WHEEL_DIAMETER_MM`), both registered and both
    /// defaulting to unset, so a consumer converts only for a board that carries both and
    /// otherwise displays counts and says that is what they are.
    pub odometer: u32,
}

impl CyclicObs {
    /// On-wire length of the appended block: `i16 + i16 + u16 + u8 + u8 + u32`, offsets 16 to 27
    /// of the payload.
    ///
    /// Additions go on the END, so every existing offset survives one, which is how the odometer
    /// took it from 8 to 12. The BLE budget is derived from this number rather than guessed at:
    /// see `BLE_CYCLIC_DIVISOR` in `motorcontroller/orchestrator/src/dispatch.rs`.
    pub const LEN: usize = 12;

    /// Encode into `out` (the block alone, as it sits after the committed prefix), returning the
    /// byte count ([`Self::LEN`]).
    fn encode(&self, out: &mut [u8]) -> usize {
        out[0..2].copy_from_slice(&self.phase_peak.to_le_bytes());
        out[2..4].copy_from_slice(&self.phase_mean.to_le_bytes());
        out[4..6].copy_from_slice(&self.duty_on.to_le_bytes());
        out[6] = self.boot_tag;
        out[7] = self.chip.to_u8();
        out[8..12].copy_from_slice(&self.odometer.to_le_bytes());
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
            odometer: rd_u32(b, 8),
        }
    }
}

// The block's declared length against the sum of its fields' widths, so a field that changes
// type cannot leave the length behind: i16 + i16 + u16 + u8 + u8 + u32.
const _: () = assert!(CyclicObs::LEN == 2 + 2 + 2 + 1 + 1 + 4);

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
    /// The mode byte. Peer consumer: the rigid follower's engagement intent (it engages while
    /// its leader reports RUN, `specs/rover-one-imu.md` section 2) and supervision/OBS.
    pub mode: u8,
    /// The board's OWN gating-producer LEVEL MASK this pass, `u16` LE at offset 9
    /// (`orchestrator::events::Levels::own_mask`, one `EV_*` bit per producer, the motor-side
    /// level as its four causes on bits 8 to 11; `specs/link-control.md`, `CYCLIC_STATE`;
    /// `specs/rover-one-imu.md` section 4), 0 = every producer
    /// clear. A level, not a code: the mode machine's SHUTDOWN is one pass long while this frame
    /// goes out every second pass, so a code naming one fault could be missed entirely, and what
    /// a peer needs to know is whether this board is healthy right now. The fault-A group is
    /// `fault & orchestrator::events::FAULT_A_BITS`; the mask also carries the mode fault and the
    /// power request, which are not faults and which that constant excludes by construction.
    pub fault: u16,
    /// Flag bits: [`Self::FLAG_RIDER`] (bit0), [`Self::FLAG_LEADER`] (bit1),
    /// [`Self::FLAG_FOLLOWER`] (bit2), [`Self::FLAG_REARM`] (bit3), [`Self::FLAG_REBOOT`] (bit4),
    /// [`Self::FLAG_PEER_SILENT`] (bit5), [`Self::FLAG_LOCKDOWN`] (bit7).
    pub flags: u8,
    /// The torque this board commands its peer to apply, on the same +-28500 stock-native scale
    /// as every other demand in the system (`specs/rover-one-imu.md` section 4). **0 unless this
    /// board is a rigid-mode leader**, which [`Self::FLAG_LEADER`] states: one meaning, one
    /// writer (the leader's control pass), one consumer (the follower's reference producer). The
    /// board's OWN applied wheel demand is not this word; that is `CTRL_OBS`'s.
    pub peer_demand: i16,
    /// The LOW 16 BITS of this board's signed wheel position (`specs/state-feedback.md`
    /// section 3): the period ISR's `pos_edges`, hall edges in the counter's own sense, NOT
    /// mirrored to forward (the receiver knows the sender's side and mirrors). Every board sends
    /// it. Peer consumer: a rigid LEADER, which unwraps it by `i16` delta per accepted frame into
    /// the machine's mean position; at most a few edges move between frames at 125 Hz, against a
    /// wrap of 65,536.
    pub wheel_pos: i16,
    /// The appended observation block ([`CyclicObs`]). Every emission carries it and every
    /// accepted payload has it, which is the guarantee the exact-length decode buys: a consumer
    /// reads the current window and the board's identity without a present/absent arm.
    pub obs: CyclicObs,
    /// This board's period-measured wheel rate, at offset 28 after the block
    /// (`specs/link-control.md`, `CYCLIC_STATE`): `motor::WHEEL_RATE_Q12` as the block carries
    /// it, unscaled, in Q12 of [`Self::wheel_speed`]'s unit (4096 = one hall edge per 320-period
    /// window), in the counter's own sense, decaying as `1/t` between edges and 0 across a
    /// reversal. Consumer: the remote app's speed readout; the peer decodes it and nothing on it
    /// reads it.
    pub wheel_rate: i32,
    /// This board's **machine digest** at offset 32 (`specs/machine-settings.md` section 4.1): a
    /// CRC-16 over the effective values of every machine field (`store::Store::machine_digest`).
    /// Consumer: the peer's arm gate, which refuses to arm while a configured link's peer reports
    /// a different one (`EV_MACHINE`); and a client, which tells the copies apart by it.
    pub machine: u16,
}

impl CyclicState {
    /// Length of the committed words, which is also the offset the appended [`CyclicObs`] block
    /// sits at (`specs/link-control.md` gives the layout in those parts). A length the wire never
    /// carries on its own: see [`Self::ENCODED_LEN`].
    pub const LEN: usize = 16;

    /// The offset of [`Self::wheel_rate`], the word after the [`CyclicObs`] block.
    pub const WHEEL_RATE_OFF: usize = Self::LEN + CyclicObs::LEN;

    /// The offset of [`Self::machine`], after [`Self::wheel_rate`].
    pub const MACHINE_OFF: usize = Self::WHEEL_RATE_OFF + 4;

    /// The on-wire length of a ONE-motor board's `CYCLIC_STATE` payload, and of every board's
    /// base: the committed words, the appended [`CyclicObs`] block, the `i32`
    /// [`Self::wheel_rate`] and the `u16` [`Self::machine`]. [`Self::encode`] writes exactly
    /// this, and a buffer handed to it has to be this long.
    pub const ENCODED_LEN: usize = 34;

    /// The offset of the motor-1 block ([`CyclicMotor`]) a two-motor board appends: the end of
    /// the base (`specs/link-control.md`, "The motor-1 block").
    pub const MOTOR1_OFF: usize = Self::ENCODED_LEN;

    /// The on-wire length of a TWO-motor board's payload: the base and motor 1's block.
    /// [`Self::decode`] accepts this and [`Self::ENCODED_LEN`], and nothing else.
    pub const TWO_MOTOR_LEN: usize = Self::MOTOR1_OFF + CyclicMotor::LEN;

    /// `flags` bit0: rider present. Peer consumer: rider mirror (profile select).
    pub const FLAG_RIDER: u8 = 1 << 0;

    /// `flags` bit1: this board is the rigid-frame LEADER (`control.frame = 1` with a configured
    /// IMU, `specs/rover-one-imu.md` section 2), so [`Self::peer_demand`] is a command and not a
    /// peer's zero. Peer consumers: the follower, which applies the word only under this flag,
    /// and a second leader, which reads it as the two-leader staging error.
    pub const FLAG_LEADER: u8 = 1 << 1;

    /// `flags` bit2: this board is the rigid-frame FOLLOWER (`control.frame = 1` with no
    /// configured IMU). Peer consumer: a leader, which feeds its engagement shell a cleared gate
    /// until a fresh peer cyclic carries this bit, so a pair whose frames disagree does not
    /// balance on one wheel (`specs/rover-one-imu.md` section 6, "A pair that disagrees").
    pub const FLAG_FOLLOWER: u8 = 1 << 2;

    /// `flags` bit3: an arm-effect field was written since this board last armed, so it does not
    /// yet run what it stores (`specs/machine-settings.md` section 5). Cleared when it arms or
    /// boots. Consumer: a client, which says "takes effect at the next arm".
    pub const FLAG_REARM: u8 = 1 << 3;

    /// `flags` bit4: a boot-effect field was written since this board booted (`specs/machine-settings.md`
    /// section 5). Cleared when it boots. Consumer: a client, which says "restart the board".
    pub const FLAG_REBOOT: u8 = 1 << 4;

    /// `flags` bit5: a configured link has heard no `CYCLIC_STATE` since this board booted
    /// (`specs/board-route.md` section 4), the first of [`Self::FAULT_MACHINE`]'s two causes. With
    /// `FAULT_MACHINE` set and this bit clear, a peer's digest differs. Consumer: a client, which
    /// says which (`hoverboard-cli machine nodes`, the app's MACHINE lamp).
    pub const FLAG_PEER_SILENT: u8 = 1 << 5;

    /// `fault` bit13, `EV_MACHINE`: the board refuses to arm because a configured link's peer was
    /// not heard or reports another [`Self::machine`] digest (`specs/machine-settings.md` section
    /// 4.2); [`Self::FLAG_PEER_SILENT`] says which. It blocks arming and nothing else. Consumer: a
    /// client, which marks the board (`hoverboard-cli machine nodes`).
    pub const FAULT_MACHINE: u16 = 1 << 13;

    /// `flags` bit7: lockdown, the stock master-shutdown semantic. The receiver treats it as a
    /// gating fault into the engagement machine (sub-state forced to 0, torque setpoint 0) for
    /// as long as it is asserted (level, latest-wins).
    pub const FLAG_LOCKDOWN: u8 = 1 << 7;

    /// Rider-present flag (bit0).
    pub const fn rider_present(&self) -> bool {
        self.flags & Self::FLAG_RIDER != 0
    }

    /// Rigid-frame leader flag (bit1).
    pub const fn leader(&self) -> bool {
        self.flags & Self::FLAG_LEADER != 0
    }

    /// Rigid-frame follower flag (bit2).
    pub const fn follower(&self) -> bool {
        self.flags & Self::FLAG_FOLLOWER != 0
    }

    /// Re-arm flag (bit3): an arm-effect field is written and not yet running.
    pub const fn rearm(&self) -> bool {
        self.flags & Self::FLAG_REARM != 0
    }

    /// Reboot flag (bit4): a boot-effect field is written and not yet running.
    pub const fn reboot(&self) -> bool {
        self.flags & Self::FLAG_REBOOT != 0
    }

    /// Peer-silent flag (bit5): a configured link has heard nothing since boot.
    pub const fn peer_silent(&self) -> bool {
        self.flags & Self::FLAG_PEER_SILENT != 0
    }

    /// The machine fault (`fault` bit13): the board refuses to arm until its peers agree.
    pub const fn machine_mismatch(&self) -> bool {
        self.fault & Self::FAULT_MACHINE != 0
    }

    /// Lockdown flag (bit7): lockdown -> immediate disengage on the receiver.
    pub const fn lockdown(&self) -> bool {
        self.flags & Self::FLAG_LOCKDOWN != 0
    }

    /// Encode into `out`, returning the byte count ([`Self::ENCODED_LEN`]), as the other three
    /// families' encoders return theirs.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::ENCODED_LEN);
        out[0..2].copy_from_slice(&self.pitch.to_le_bytes());
        out[2..4].copy_from_slice(&self.roll.to_le_bytes());
        out[4..6].copy_from_slice(&self.wheel_speed.to_le_bytes());
        out[6..8].copy_from_slice(&self.battery.to_le_bytes());
        out[8] = self.mode;
        out[9..11].copy_from_slice(&self.fault.to_le_bytes());
        out[11] = self.flags;
        out[12..14].copy_from_slice(&self.peer_demand.to_le_bytes());
        out[14..16].copy_from_slice(&self.wheel_pos.to_le_bytes());
        self.obs.encode(&mut out[Self::LEN..]);
        out[Self::WHEEL_RATE_OFF..Self::MACHINE_OFF]
            .copy_from_slice(&self.wheel_rate.to_le_bytes());
        out[Self::MACHINE_OFF..Self::ENCODED_LEN].copy_from_slice(&self.machine.to_le_bytes());
        Self::ENCODED_LEN
    }

    /// Decode the base of a payload of exactly [`Self::ENCODED_LEN`] or [`Self::TWO_MOTOR_LEN`];
    /// **any other length is [`DecodeError::WrongLen`]**, short or long. A two-motor payload's
    /// motor-1 block is not read: [`Self::decode_with_motor1`] reads it.
    ///
    /// The exception to the crate's committed-prefix rule, and the module doc carries the whole
    /// argument: a sender in this tree writes thirty-four bytes or forty-six, L2 never delivers a
    /// truncated frame, and no older image is owed compatibility, so every other length is a peer
    /// that is not speaking this protocol.
    ///
    /// A board decodes its peer with this and keeps the base: nothing on a board reads a peer's
    /// motor 1, and the forward re-encodes the base, so a forwarded two-motor frame leaves as 34 B
    /// (`specs/link-control.md`, "The motor-1 block").
    pub fn decode(b: &[u8]) -> Result<CyclicState, DecodeError> {
        if b.len() != Self::ENCODED_LEN && b.len() != Self::TWO_MOTOR_LEN {
            return Err(DecodeError::WrongLen);
        }
        Ok(Self::decode_base(b))
    }

    /// Decode a payload as [`Self::decode`] does, and its motor-1 block with it: `Some` for a
    /// [`Self::TWO_MOTOR_LEN`] payload, `None` for a one-motor board's [`Self::ENCODED_LEN`]. A
    /// controller's decode: the app's and the CLI's per-wheel figures read motor 1 from it.
    pub fn decode_with_motor1(b: &[u8]) -> Result<(CyclicState, Option<CyclicMotor>), DecodeError> {
        let base = Self::decode(b)?;
        let motor1 =
            (b.len() == Self::TWO_MOTOR_LEN).then(|| CyclicMotor::decode(&b[Self::MOTOR1_OFF..]));
        Ok((base, motor1))
    }

    /// The base words from `b`, which holds at least [`Self::ENCODED_LEN`] bytes.
    fn decode_base(b: &[u8]) -> CyclicState {
        CyclicState {
            pitch: rd_i16(b, 0),
            roll: rd_i16(b, 2),
            wheel_speed: rd_i16(b, 4),
            battery: rd_u16(b, 6),
            mode: b[8],
            fault: rd_u16(b, 9),
            flags: b[11],
            peer_demand: rd_i16(b, 12),
            wheel_pos: rd_i16(b, 14),
            obs: CyclicObs::decode(&b[Self::LEN..]),
            wheel_rate: rd_i32(b, Self::WHEEL_RATE_OFF),
            machine: rd_u16(b, Self::MACHINE_OFF),
        }
    }
}

// The emitted length is the prefix plus the block plus the i32 rate plus the u16 digest, held by
// the compiler rather than by whoever edits either number next.
const _: () = assert!(CyclicState::ENCODED_LEN == CyclicState::LEN + CyclicObs::LEN + 4 + 2);

/// The MOTOR-1 BLOCK a two-motor board appends to its `CYCLIC_STATE` at
/// [`CyclicState::MOTOR1_OFF`] (`specs/link-control.md`, "The motor-1 block"): motor 1's wheel
/// words, each defined as its motor-0 counterpart in the base is. The base's words stay motor 0's
/// and the board's.
///
/// No odometer: the firmware keeps one, motor 0's (`specs/odometer.md`, "No second motor").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CyclicMotor {
    /// Motor 1's window edge count, as [`CyclicState::wheel_speed`] (offset 34).
    pub wheel_speed: i16,
    /// Motor 1's window peak phase-current magnitude, counts, as [`CyclicObs::phase_peak`]
    /// (offset 36).
    pub phase_peak: i16,
    /// Motor 1's window mean magnitude, counts, as [`CyclicObs::phase_mean`] (offset 38).
    pub phase_mean: i16,
    /// Motor 1's last applied on-duty, as [`CyclicObs::duty_on`] (offset 40).
    pub duty_on: u16,
    /// Motor 1's period-measured rate, Q12 of `wheel_speed`'s unit, as
    /// [`CyclicState::wheel_rate`] (offset 42).
    pub wheel_rate: i32,
}

impl CyclicMotor {
    /// On-wire length of the block: `i16 + i16 + i16 + u16 + i32`, offsets 34 to 45.
    pub const LEN: usize = 12;

    /// Every word zero: a stopped wheel carrying no current. What an emitter's slot holds before
    /// its first pass.
    pub const ZERO: CyclicMotor = CyclicMotor {
        wheel_speed: 0,
        phase_peak: 0,
        phase_mean: 0,
        duty_on: 0,
        wheel_rate: 0,
    };

    /// Encode the block alone into `out`, as it sits after the base, returning the byte count
    /// ([`Self::LEN`]). The emitter writes it at [`CyclicState::MOTOR1_OFF`].
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0..2].copy_from_slice(&self.wheel_speed.to_le_bytes());
        out[2..4].copy_from_slice(&self.phase_peak.to_le_bytes());
        out[4..6].copy_from_slice(&self.phase_mean.to_le_bytes());
        out[6..8].copy_from_slice(&self.duty_on.to_le_bytes());
        out[8..12].copy_from_slice(&self.wheel_rate.to_le_bytes());
        Self::LEN
    }

    /// Decode the block from `b`, which must hold at least [`Self::LEN`] bytes.
    fn decode(b: &[u8]) -> CyclicMotor {
        CyclicMotor {
            wheel_speed: rd_i16(b, 0),
            phase_peak: rd_i16(b, 2),
            phase_mean: rd_i16(b, 4),
            duty_on: rd_u16(b, 6),
            wheel_rate: rd_i32(b, 8),
        }
    }
}

// The block's length against its fields' widths: i16 + i16 + i16 + u16 + i32.
const _: () = assert!(CyclicMotor::LEN == 2 + 2 + 2 + 2 + 4);

// --- DRIVE_CMD (6 B): a controller's drive reference -------------------------------------------

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

/// A controller's drive reference for one motor of the addressed node. Consumer: the
/// throttle-mode reference producer (`specs/integration.md`). Consumed from any port via normal
/// L3 delivery; the firmware never originates it. A two-motor board takes one per motor
/// (`specs/twelve-fet.md` section 6.1); a sender drives each wheel with its own frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriveCmd {
    /// Command kind; see [`DriveKind`].
    pub kind: DriveKind,
    /// Speed demand, `ControlDispatch::throttle_reference` input scale.
    pub value: i16,
    /// Steer demand, same scale.
    pub steer: i16,
    /// The motor on the addressed node, `0..` its motor count. A board drops a frame whose
    /// motor it does not drive.
    pub motor: u8,
}

impl DriveCmd {
    /// On-wire length of the committed prefix. A 5-byte payload (the format before `motor`) is
    /// [`DecodeError::TooShort`]: no compatibility path, and the dropped frame lets that motor's
    /// demand decay to neutral.
    pub const LEN: usize = 6;

    /// Offset of `motor` in the payload: the byte a latest-wins queue keys on beside the
    /// destination, so one wheel's frame never replaces another's.
    pub const MOTOR_OFF: usize = 5;

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.kind as u8;
        out[1..3].copy_from_slice(&self.value.to_le_bytes());
        out[3..5].copy_from_slice(&self.steer.to_le_bytes());
        out[Self::MOTOR_OFF] = self.motor;
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
            motor: b[Self::MOTOR_OFF],
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
/// and the pads) into an IIR nothing read; it is deleted
/// (`specs/link-control.md`, "`INPUTS` (2 B): remote input mirror"). The one thing
/// that made the deletion awkward is recorded here because it is the payload convention: fields
/// APPEND and never reorder, and this one was FIRST, so removing it shifted `buttons` and `rider`
/// and the firmware and the Kotlin mirror had to move together or the drift gate fails the build.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inputs {
    /// Button bits: [`Self::BUTTON_POWER`] (bit0), [`Self::BUTTON_HORN`] (bit1),
    /// [`Self::BUTTON_OBSERVER`] (bit2).
    pub buttons: u8,
    /// Rider bits: [`Self::RIDER_PRESENT`] (bit0).
    pub rider: u8,
}

impl Inputs {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 2;

    /// `buttons` bit0: power request (level).
    pub const BUTTON_POWER: u8 = 1 << 0;

    /// `buttons` bit1: the horn (level, `specs/tones.md` section 4.3): the board plays its horn
    /// tune while the mirror it consumes carries it.
    pub const BUTTON_HORN: u8 = 1 << 1;
    /// `buttons` bit2: observer source request (level): the FOC commutates from the flux
    /// observer's angle while it is held (`specs/flux-observer.md`, "S4, first slice").
    pub const BUTTON_OBSERVER: u8 = 1 << 2;

    /// `rider` bit0: rider present.
    pub const RIDER_PRESENT: u8 = 1 << 0;

    /// Power-request level (`buttons` bit0).
    pub const fn power_request(&self) -> bool {
        self.buttons & Self::BUTTON_POWER != 0
    }

    /// Horn level (`buttons` bit1).
    pub const fn horn(&self) -> bool {
        self.buttons & Self::BUTTON_HORN != 0
    }

    /// Observer-source level (`buttons` bit2).
    pub const fn observer_request(&self) -> bool {
        self.buttons & Self::BUTTON_OBSERVER != 0
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

// --- FOC_TRACE (46 B): the FOC loop and the flux observer's shadow, one window a frame -------------

/// One closed 64-period window of one motor's FOC loop and flux observer (`specs/link-control.md`,
/// "`FOC_TRACE` (0x15, 46 B)"; `specs/flux-observer.md`): the quantities the period ISR publishes
/// at the window close (`CTRL_OBS` words 48 to 50 and 52 to 54, that motor's) and the tracking
/// table. Built by the board's loop pass on a subscription ([`TraceSubscription`]); consumed by
/// `hoverboard-cli board trace` and the app's FOC scope screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FocTrace {
    /// The 64-period window counter at the close, wrapping.
    pub window: u16,
    /// Window mean `id`, current counts.
    pub id: i16,
    /// Window mean `iq`, current counts.
    pub iq: i16,
    /// The closing period's `iq` reference.
    pub iq_ref: i16,
    /// Window mean `vq` after the circular limit, voltage counts.
    pub vq: i16,
    /// `theta` at the close: the front end's angle plus the alignment, 65536 a revolution.
    pub hall_angle: u16,
    /// The observer's angle at the close, same units.
    pub obs_angle: u16,
    /// The window's angle-error mean, observer minus hall, angle units.
    pub err_mean: i16,
    /// The window's largest excursion from that mean, angle units.
    pub err_spread: i16,
    /// The window's RMS flux magnitude, microvolt-seconds.
    pub flux_mag: u16,
    /// The front end's latched signed speed, edges per 320-period window.
    pub speed: i16,
    /// Periods `phase_a` was held this window.
    pub held_a: u8,
    /// Periods `phase_b` was held.
    pub held_b: u8,
    /// Floated periods.
    pub coasted: u8,
    /// The hold cause byte.
    pub cause: u8,
    /// [`Self::FLAG_HALL_FAULT`], [`Self::FLAG_TRACKED`], [`Self::FLAG_OBSERVER_SOURCE`],
    /// [`Self::FLAG_IN_WINDOW`].
    pub flags: u8,
    /// The commutation method byte (2 FOC); the FOC and observer fields are 0 in another.
    pub method: u8,
    /// The lock bin, 1..=15, or 255 for none.
    pub lock_bin: u8,
    /// Which motor's fields these are.
    pub motor: u8,
    /// Per speed bin, windows tracked over windows seen in 0.4 percent steps (0..=250), or 255
    /// while the bin is not measured.
    pub track: [u8; 16],
}

impl FocTrace {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 46;

    /// `flags` bit 0: the front end's hall fault.
    pub const FLAG_HALL_FAULT: u8 = 1 << 0;
    /// `flags` bit 1: this window tracked.
    pub const FLAG_TRACKED: u8 = 1 << 1;
    /// `flags` bit 2: the observer is the angle source (the window's last period drove from it).
    pub const FLAG_OBSERVER_SOURCE: u8 = 1 << 2;
    /// `flags` bit 3: the front end was in its interpolation window at the close.
    pub const FLAG_IN_WINDOW: u8 = 1 << 3;

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        let words = [
            self.window,
            self.id as u16,
            self.iq as u16,
            self.iq_ref as u16,
            self.vq as u16,
            self.hall_angle,
            self.obs_angle,
            self.err_mean as u16,
            self.err_spread as u16,
            self.flux_mag,
            self.speed as u16,
        ];
        for (k, w) in words.iter().enumerate() {
            out[2 * k..2 * k + 2].copy_from_slice(&w.to_le_bytes());
        }
        out[22..30].copy_from_slice(&[
            self.held_a,
            self.held_b,
            self.coasted,
            self.cause,
            self.flags,
            self.method,
            self.lock_bin,
            self.motor,
        ]);
        out[30..46].copy_from_slice(&self.track);
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
    pub fn decode(b: &[u8]) -> Result<FocTrace, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        let mut track = [0u8; 16];
        track.copy_from_slice(&b[30..46]);
        Ok(FocTrace {
            window: rd_u16(b, 0),
            id: rd_i16(b, 2),
            iq: rd_i16(b, 4),
            iq_ref: rd_i16(b, 6),
            vq: rd_i16(b, 8),
            hall_angle: rd_u16(b, 10),
            obs_angle: rd_u16(b, 12),
            err_mean: rd_i16(b, 14),
            err_spread: rd_i16(b, 16),
            flux_mag: rd_u16(b, 18),
            speed: rd_i16(b, 20),
            held_a: b[22],
            held_b: b[23],
            coasted: b[24],
            cause: b[25],
            flags: b[26],
            method: b[27],
            lock_bin: b[28],
            motor: b[29],
            track,
        })
    }
}

// --- TRACE_SET (2 B): the trace subscription ---------------------------------------------------

/// The trace subscription and its keepalive (`specs/link-control.md`, "`TRACE_SET` (0x16, 2 B)").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceSet {
    /// Bits 0..6: emit `FOC_TRACE` every this many window closes, 0 = off; the board clamps 1..4
    /// to [`Self::DIVISOR_MIN`]. Bit 7 ([`Self::RESET`]): clear the selected motor's tracking
    /// table before applying.
    pub divisor: u8,
    /// The motor to trace.
    pub motor: u8,
}

impl TraceSet {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 2;

    /// `divisor` bit 7: clear the tracking table first, with any divisor, 0 included.
    pub const RESET: u8 = 1 << 7;
    /// `divisor` bits 0..6.
    pub const DIVISOR_MASK: u8 = 0x7F;
    /// The smallest divisor the board runs (50 Hz): a divisor of 1 to 4 reads as this.
    pub const DIVISOR_MIN: u8 = 5;

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0] = self.divisor;
        out[1] = self.motor;
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
    pub fn decode(b: &[u8]) -> Result<TraceSet, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        Ok(TraceSet {
            divisor: b[0],
            motor: b[1],
        })
    }

    /// Whether the reset bit is set.
    pub const fn reset(&self) -> bool {
        self.divisor & Self::RESET != 0
    }

    /// The divisor the board runs: 0 off, 1..4 clamped up to [`Self::DIVISOR_MIN`].
    pub const fn effective_divisor(&self) -> u8 {
        let d = self.divisor & Self::DIVISOR_MASK;
        if d != 0 && d < Self::DIVISOR_MIN {
            Self::DIVISOR_MIN
        } else {
            d
        }
    }
}

/// The board's one trace subscription (`specs/link-control.md`, "`TRACE_SET`"): a LEVEL with an
/// expiry, latest-wins across subscribers, and the emission cadence over the window counter. Pure:
/// the loop pass hands it the tick and the window, and acts on what it returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceSubscription {
    /// The effective divisor, 0 = off.
    divisor: u8,
    motor: u8,
    /// The tick of the last `TRACE_SET` with a non-zero divisor.
    renewed: u32,
    /// The window counter of the last emission, `None` before the first of this subscription.
    last: Option<u16>,
}

impl Default for TraceSubscription {
    fn default() -> Self {
        Self::new()
    }
}

impl TraceSubscription {
    /// No subscription.
    pub const fn new() -> Self {
        TraceSubscription {
            divisor: 0,
            motor: 0,
            renewed: 0,
            last: None,
        }
    }

    /// Apply a `TRACE_SET` heard at tick `now`; `motor_up` says whether this board brought its
    /// `motor` up. A motor it has not is refused with no change at all, the reset bit included.
    /// Returns the motor whose tracking table to clear when the reset bit was set.
    pub fn apply(&mut self, s: TraceSet, now: u32, motor_up: bool) -> Option<u8> {
        if !motor_up {
            return None;
        }
        let d = s.effective_divisor();
        if d == 0 {
            self.divisor = 0;
        } else {
            if !self.active(now) {
                self.last = None;
            }
            self.divisor = d;
            self.motor = s.motor;
            self.renewed = now;
        }
        s.reset().then_some(s.motor)
    }

    /// Whether a subscription is live at tick `now`: a non-zero divisor renewed less than
    /// [`TRACE_TIMEOUT_TICKS`] ago.
    pub fn active(&self, now: u32) -> bool {
        self.divisor != 0 && now.wrapping_sub(self.renewed) < TRACE_TIMEOUT_TICKS
    }

    /// The effective divisor, 0 = off.
    pub fn divisor(&self) -> u8 {
        self.divisor
    }

    /// The emission decision at tick `now` with the window counter at `window`: the motor to emit
    /// for when the subscription is live and `window` is the divisor or more past the last
    /// emission (or this subscription has emitted none), which it records; `None` otherwise.
    pub fn due(&mut self, window: u16, now: u32) -> Option<u8> {
        if !self.active(now) {
            return None;
        }
        match self.last {
            Some(l) if window.wrapping_sub(l) < self.divisor as u16 => None,
            _ => {
                self.last = Some(window);
                Some(self.motor)
            }
        }
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
    /// [`OP_TRACE_SET`].
    TraceSet(TraceSet),
}

/// Decode a delivered control-block PDU payload by opcode (the firmware's routing entry for the
/// `0x10..0x2F` hand-back, `specs/integration.md`). The families a board consumes: `FOC_TRACE` is
/// a board's to send and a host's to read ([`FocTrace::decode`]), so a board image carries no
/// decoder for it and it reads here as unallocated. Returns `None` for an opcode this crate does
/// not allocate, a payload shorter than an append-tolerant family's committed prefix, or a
/// `CYCLIC_STATE` payload that is neither [`CyclicState::ENCODED_LEN`] nor
/// [`CyclicState::TWO_MOTOR_LEN`]: the delivery class is best-effort, so the PDU is simply
/// dropped and no error propagates. A `CYCLIC_STATE` decodes to its base ([`CyclicState::decode`]).
pub fn decode(opcode: u8, payload: &[u8]) -> Option<Payload> {
    match opcode {
        OP_CYCLIC_STATE => CyclicState::decode(payload).ok().map(Payload::CyclicState),
        OP_DRIVE_CMD => DriveCmd::decode(payload).ok().map(Payload::DriveCmd),
        OP_INPUTS => Inputs::decode(payload).ok().map(Payload::Inputs),
        OP_TRACE_SET => TraceSet::decode(payload).ok().map(Payload::TraceSet),
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
        assert_eq!(OP_FOC_TRACE, 0x15);
        assert_eq!(OP_TRACE_SET, 0x16);
        // All five live in the reserved control block 0x10..0x2F.
        for op in [
            OP_CYCLIC_STATE,
            OP_DRIVE_CMD,
            OP_INPUTS,
            OP_FOC_TRACE,
            OP_TRACE_SET,
        ] {
            assert!((0x10..0x2F).contains(&op));
        }
    }

    #[test]
    fn supervision_constants_pinned() {
        // 25 ticks = 100 ms at 250 Hz; 50 ticks = 200 ms; 375 ticks = 1.5 s.
        assert_eq!(CYCLIC_TIMEOUT_TICKS, 25);
        assert_eq!(DRIVE_TIMEOUT_TICKS, 50);
        assert_eq!(INPUTS_TIMEOUT_TICKS, 375);
        assert_eq!(TRACE_TIMEOUT_TICKS, 375);
    }

    #[test]
    fn committed_lengths_pinned() {
        assert_eq!(CyclicState::LEN, 16);
        assert_eq!(DriveCmd::LEN, 6);
        assert_eq!(Inputs::LEN, 2);
        assert_eq!(FocTrace::LEN, 46);
        assert_eq!(TraceSet::LEN, 2);
    }

    /// The appended block's length and the `CYCLIC_STATE` payload's one on-wire length, against
    /// the committed words it is built from: 16 is where the block starts, 28 is where
    /// `wheel_rate` starts, 32 is where `machine` starts, 34 is what the wire carries and what a
    /// decoder requires.
    #[test]
    fn appended_block_lengths_pinned() {
        assert_eq!(CyclicObs::LEN, 12, "i16 + i16 + u16 + u8 + u8 + u32");
        assert_eq!(CyclicState::WHEEL_RATE_OFF, 28);
        assert_eq!(CyclicState::MACHINE_OFF, 32);
        assert_eq!(CyclicState::ENCODED_LEN, 34);
        assert_eq!(
            CyclicState::ENCODED_LEN,
            CyclicState::LEN + CyclicObs::LEN + 4 + 2
        );
    }

    // -- Wire layout (byte-exact, little-endian) ------------------------------------------------

    fn cyclic_sample() -> CyclicState {
        CyclicState {
            pitch: -2,       // 0xFFFE
            roll: 0x0102,    // LE 02 01
            wheel_speed: -1, // 0xFFFF
            battery: 0xA1B2, // LE B2 A1
            mode: 0x03,
            fault: 0x0911,
            flags: CyclicState::FLAG_RIDER | CyclicState::FLAG_LOCKDOWN,
            peer_demand: -28500, // 0x90AC
            wheel_pos: -0x1235,  // 0xEDCB
            obs: CyclicObs {
                phase_peak: 0x0304, // LE 04 03
                phase_mean: -3,     // 0xFFFD
                duty_on: 0x08C1,    // LE C1 08
                boot_tag: 0x7B,
                chip: ChipTag::F130C8,
                odometer: 0x0A0B_0C0D,
            },
            wheel_rate: -0x0102_0304, // 0xFEFDFCFC
            machine: 0xBEEF,          // LE EF BE
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
                0x11, 0x09, // fault 0x0911
                0x81, // flags: bit0 | bit7
                0xAC, 0x90, // peer_demand -28500
                0xCB, 0xED, // wheel_pos -0x1235
                // The appended block, from offset 16.
                0x04, 0x03, // phase_peak 0x0304
                0xFD, 0xFF, // phase_mean -3
                0xC1, 0x08, // duty_on 0x08C1
                0x7B, // boot_tag
                0x02, // chip: F130C8
                0x0D, 0x0C, 0x0B, 0x0A, // odometer 0x0A0B0C0D
                0xFC, 0xFC, 0xFD, 0xFE, // wheel_rate -0x01020304, from offset 28
                0xEF, 0xBE, // machine 0xBEEF, from offset 32
            ]
        );
    }

    #[test]
    fn drive_cmd_wire_layout_is_little_endian() {
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: -300,   // 0xFED4
            steer: 0x1234, // LE 34 12
            motor: 1,
        };
        let mut buf = [0u8; DriveCmd::LEN];
        assert_eq!(cmd.encode(&mut buf), DriveCmd::LEN);
        assert_eq!(buf, [0x01, 0xD4, 0xFE, 0x34, 0x12, 0x01]);
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
    fn the_horn_is_buttons_bit1() {
        assert_eq!(Inputs::BUTTON_HORN, 0x02);
        let inp = Inputs {
            buttons: Inputs::BUTTON_HORN,
            rider: 0,
        };
        let mut buf = [0u8; Inputs::LEN];
        inp.encode(&mut buf);
        assert_eq!(buf, [0x02, 0x00]);
        let back = Inputs::decode(&buf).unwrap();
        assert!(back.horn() && !back.power_request());
        let both = Inputs::decode(&[0x03, 0x00]).unwrap();
        assert!(both.horn() && both.power_request());
        assert!(!Inputs::decode(&[0x01, 0x00]).unwrap().horn());
    }

    // -- Round trips ----------------------------------------------------------------------------

    #[test]
    fn cyclic_state_round_trip() {
        let orig = cyclic_sample();
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        orig.encode(&mut buf);
        assert_eq!(CyclicState::decode(&buf), Ok(orig));
    }

    /// A payload short of [`CyclicState::ENCODED_LEN`] is REFUSED, the sixteen committed bytes
    /// included.
    ///
    /// Eleven decoded here until 2026-10-10, for a peer on an image from before the appended
    /// block. Nothing is owed an older image before a release, and the other argument for it
    /// (a truncated frame must not cost the lockdown flag) was wrong on the framing: a frame
    /// whose CRC-16 fails is dropped at L2 and the stream resyncs at the next SOF
    /// (`specs/l2.md`), so a truncated payload never arrives here. Eleven bytes, or any partial
    /// block above them, is a peer that is not speaking this protocol.
    #[test]
    fn a_cyclic_payload_short_of_the_whole_is_refused() {
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        assert_eq!(
            cyclic_sample().encode(&mut buf),
            CyclicState::ENCODED_LEN,
            "a sender writes the one length"
        );
        for len in 0..CyclicState::ENCODED_LEN {
            assert_eq!(
                CyclicState::decode(&buf[..len]),
                Err(DecodeError::WrongLen),
                "{len} bytes"
            );
        }
        assert!(
            CyclicState::decode(&buf).is_ok(),
            "and the whole payload decodes"
        );
    }

    /// An OVER-LONG payload is refused too, where the other three families would read their
    /// prefix and ignore the rest: this family's length is exact in both directions, so a
    /// thirty-one-byte reading of a thirty-two-byte payload is a disagreement about the layout,
    /// not a field this build has yet to learn.
    #[test]
    fn an_over_long_cyclic_payload_is_refused() {
        let mut buf = [0xEEu8; CyclicState::ENCODED_LEN + 4];
        cyclic_sample().encode(&mut buf);
        for len in CyclicState::ENCODED_LEN + 1..=CyclicState::ENCODED_LEN + 4 {
            assert_eq!(
                CyclicState::decode(&buf[..len]),
                Err(DecodeError::WrongLen),
                "{len} bytes"
            );
        }
    }

    // -- The motor-1 block (a two-motor board's 46 B payload) ------------------------------------

    fn motor1_sample() -> CyclicMotor {
        CyclicMotor {
            wheel_speed: -5,         // 0xFFFB
            phase_peak: 0x0506,      // LE 06 05
            phase_mean: 0x0102,      // LE 02 01
            duty_on: 0x0A0B,         // LE 0B 0A
            wheel_rate: 0x1122_3344, // LE 44 33 22 11
        }
    }

    /// A two-motor payload's encoding: the base exactly as a one-motor board writes it, then motor
    /// 1's block from offset 34.
    fn two_motor_wire() -> [u8; CyclicState::TWO_MOTOR_LEN] {
        let mut buf = [0u8; CyclicState::TWO_MOTOR_LEN];
        let n = cyclic_sample().encode(&mut buf);
        assert_eq!(n, CyclicState::MOTOR1_OFF);
        assert_eq!(motor1_sample().encode(&mut buf[n..]), CyclicMotor::LEN);
        buf
    }

    /// The block's offsets and the two-motor length (`specs/link-control.md`, "The motor-1 block":
    /// 12 B at offset 34, a 46 B payload).
    #[test]
    fn motor1_block_lengths_pinned() {
        assert_eq!(CyclicMotor::LEN, 12, "i16 + i16 + i16 + u16 + i32");
        assert_eq!(CyclicState::MOTOR1_OFF, 34);
        assert_eq!(CyclicState::TWO_MOTOR_LEN, 46);
    }

    #[test]
    fn motor1_block_wire_layout_is_little_endian() {
        let buf = two_motor_wire();
        let mut base = [0u8; CyclicState::ENCODED_LEN];
        cyclic_sample().encode(&mut base);
        assert_eq!(
            buf[..CyclicState::MOTOR1_OFF],
            base,
            "the base is unchanged"
        );
        assert_eq!(
            buf[CyclicState::MOTOR1_OFF..],
            [
                0xFB, 0xFF, // wheel_speed -5, offset 34
                0x06, 0x05, // phase_peak 0x0506, offset 36
                0x02, 0x01, // phase_mean 0x0102, offset 38
                0x0B, 0x0A, // duty_on 0x0A0B, offset 40
                0x44, 0x33, 0x22, 0x11, // wheel_rate 0x11223344, offset 42
            ]
        );
    }

    /// A 46 B payload decodes the base as a 34 B one would and the block as motor 1's; a 34 B
    /// payload has no motor 1.
    #[test]
    fn two_motor_round_trip_and_one_motor_has_no_motor1() {
        let buf = two_motor_wire();
        assert_eq!(
            CyclicState::decode_with_motor1(&buf),
            Ok((cyclic_sample(), Some(motor1_sample())))
        );
        assert_eq!(
            CyclicState::decode(&buf),
            Ok(cyclic_sample()),
            "the base alone"
        );
        assert_eq!(
            CyclicState::decode_with_motor1(&buf[..CyclicState::ENCODED_LEN]),
            Ok((cyclic_sample(), None))
        );
    }

    /// Every length but 34 and 46 is refused by both decoders, a truncated two-motor frame
    /// included (`specs/link-control.md`, "The motor-1 block": it fails as any short frame does).
    #[test]
    fn only_the_two_lengths_decode() {
        let mut buf = [0xEEu8; CyclicState::TWO_MOTOR_LEN + 4];
        buf[..CyclicState::TWO_MOTOR_LEN].copy_from_slice(&two_motor_wire());
        for len in 0..=buf.len() {
            let ok = len == CyclicState::ENCODED_LEN || len == CyclicState::TWO_MOTOR_LEN;
            assert_eq!(CyclicState::decode(&buf[..len]).is_ok(), ok, "{len} bytes");
            assert_eq!(
                CyclicState::decode_with_motor1(&buf[..len]).is_ok(),
                ok,
                "{len} bytes"
            );
            if !ok {
                assert_eq!(CyclicState::decode(&buf[..len]), Err(DecodeError::WrongLen));
            }
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

    #[test]
    fn drive_cmd_round_trip_both_kinds() {
        for kind in [DriveKind::Neutral, DriveKind::Throttle] {
            for motor in [0u8, 1, 0xFF] {
                let orig = DriveCmd {
                    kind,
                    value: -12345,
                    steer: 6789,
                    motor,
                };
                let mut buf = [0u8; DriveCmd::LEN];
                orig.encode(&mut buf);
                assert_eq!(DriveCmd::decode(&buf), Ok(orig));
            }
        }
    }

    /// A 5-byte payload is the format before `motor`: refused, with no fallback to motor 0
    /// (`specs/twelve-fet.md` section 6.1).
    #[test]
    fn a_five_byte_drive_cmd_is_too_short() {
        assert_eq!(
            DriveCmd::decode(&[0x01, 0xD4, 0xFE, 0x34, 0x12]),
            Err(DecodeError::TooShort)
        );
        assert_eq!(decode(OP_DRIVE_CMD, &[0x01, 0xD4, 0xFE, 0x34, 0x12]), None);
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

    // -- Committed-prefix rule: trailing bytes ignored ------------------------------------------

    /// The two append-tolerant families. `CyclicState` is deliberately absent: its length is
    /// exact, and `an_over_long_cyclic_payload_is_refused` is its half of this pair.
    #[test]
    fn trailing_bytes_are_ignored_every_append_tolerant_family() {
        // Encode each family into an oversized buffer with poisoned trailing bytes; the decode
        // must read only the committed prefix and match the original.
        let mut buf = [0xEEu8; 32];
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: 5,
            steer: -5,
            motor: 1,
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
    }

    // -- Committed-prefix rule: short payloads rejected -----------------------------------------

    #[test]
    fn short_payloads_are_rejected() {
        let buf = [0u8; 16];
        // One byte short of each family's committed prefix, and empty. `CyclicState` is in
        // `a_cyclic_payload_short_of_the_whole_is_refused`: it has no prefix to be one short of.
        for len in [DriveCmd::LEN - 1, 0] {
            assert_eq!(DriveCmd::decode(&buf[..len]), Err(DecodeError::TooShort));
        }
        for len in [Inputs::LEN - 1, 0] {
            assert_eq!(Inputs::decode(&buf[..len]), Err(DecodeError::TooShort));
        }
    }

    // -- DRIVE_CMD fail-safe --------------------------------------------------------------------

    #[test]
    fn unknown_drive_kind_decodes_as_neutral() {
        for kind_byte in [2u8, 0x7F, 0xFF] {
            let buf = [kind_byte, 0xD4, 0xFE, 0x34, 0x12, 0x00];
            let got = DriveCmd::decode(&buf).unwrap();
            assert_eq!(got.kind, DriveKind::Neutral, "kind byte {kind_byte:#04x}");
            // The words are carried through (not live under Neutral).
            assert_eq!(got.value, -300);
            assert_eq!(got.steer, 0x1234);
        }
        // The defined kind bytes map exactly.
        assert_eq!(
            DriveCmd::decode(&[0, 0, 0, 0, 0, 0]).unwrap().kind,
            DriveKind::Neutral
        );
        assert_eq!(
            DriveCmd::decode(&[1, 0, 0, 0, 0, 0]).unwrap().kind,
            DriveKind::Throttle
        );
    }

    // -- Flag-bit extraction --------------------------------------------------------------------

    #[test]
    fn cyclic_flag_bits_extract() {
        let mut c = cyclic_sample();
        c.flags = 0;
        assert!(!c.rider_present());
        assert!(!c.leader());
        assert!(!c.lockdown());
        c.flags = CyclicState::FLAG_RIDER;
        assert!(c.rider_present());
        assert!(!c.leader());
        assert!(!c.lockdown());
        c.flags = CyclicState::FLAG_LEADER;
        assert!(!c.rider_present());
        assert!(c.leader());
        assert!(!c.follower());
        assert!(!c.lockdown());
        c.flags = CyclicState::FLAG_FOLLOWER;
        assert!(!c.leader());
        assert!(c.follower());
        c.flags = CyclicState::FLAG_LOCKDOWN;
        assert!(!c.rider_present());
        assert!(!c.leader());
        assert!(!c.follower());
        assert!(c.lockdown());
        c.flags = CyclicState::FLAG_REARM;
        assert!(c.rearm() && !c.reboot() && !c.follower());
        c.flags = CyclicState::FLAG_REBOOT;
        assert!(!c.rearm() && c.reboot() && !c.lockdown());
        assert!(!c.peer_silent());
        c.flags = CyclicState::FLAG_PEER_SILENT;
        assert!(c.peer_silent() && !c.reboot() && !c.lockdown());
        assert_eq!(
            (
                CyclicState::FLAG_REARM,
                CyclicState::FLAG_REBOOT,
                CyclicState::FLAG_PEER_SILENT
            ),
            (0x08, 0x10, 0x20)
        );
        // Foreign bits do not bleed into the defined flags.
        c.flags = !(CyclicState::FLAG_RIDER
            | CyclicState::FLAG_LEADER
            | CyclicState::FLAG_FOLLOWER
            | CyclicState::FLAG_REARM
            | CyclicState::FLAG_REBOOT
            | CyclicState::FLAG_PEER_SILENT
            | CyclicState::FLAG_LOCKDOWN);
        assert!(!c.rider_present());
        assert!(!c.leader());
        assert!(!c.follower());
        assert!(!c.rearm());
        assert!(!c.reboot());
        assert!(!c.peer_silent());
        assert!(!c.lockdown());
    }

    #[test]
    fn inputs_flag_bits_extract() {
        let mut i = Inputs {
            buttons: 0,
            rider: 0,
        };
        assert!(!i.power_request());
        assert!(!i.observer_request());
        assert!(!i.rider_present());
        i.buttons = Inputs::BUTTON_POWER;
        i.rider = Inputs::RIDER_PRESENT;
        assert!(i.power_request());
        assert!(!i.observer_request(), "bit0 is not the observer");
        assert!(i.rider_present());
        i.buttons = Inputs::BUTTON_OBSERVER;
        assert!(i.observer_request());
        assert!(!i.power_request(), "bit2 is not the power request");
        assert!(!i.horn(), "bit2 is not the horn");
        i.buttons = Inputs::BUTTON_HORN;
        assert!(!i.observer_request(), "bit1 is not the observer");
        // Bits 0 to 2 are defined on `buttons`, bit0 on `rider`.
        i.buttons = 0xF8;
        i.rider = 0xFE;
        assert!(!i.power_request());
        assert!(!i.horn());
        assert!(!i.observer_request());
        assert!(!i.rider_present());
    }

    // -- Dispatch (the 0x10..0x2F hand-back routing entry) --------------------------------------

    #[test]
    fn dispatch_routes_each_opcode() {
        let cyc = cyclic_sample();
        let mut buf = [0u8; CyclicState::ENCODED_LEN];
        cyc.encode(&mut buf);
        assert_eq!(
            decode(OP_CYCLIC_STATE, &buf[..CyclicState::ENCODED_LEN]),
            Some(Payload::CyclicState(cyc))
        );
        // A two-motor board's payload routes to its base.
        assert_eq!(
            decode(OP_CYCLIC_STATE, &two_motor_wire()),
            Some(Payload::CyclicState(cyc))
        );

        let mut buf = [0u8; 16];
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: 1,
            steer: 2,
            motor: 1,
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
    }

    #[test]
    fn dispatch_drops_unallocated_and_short() {
        let buf = [0u8; 16];
        // Unallocated opcodes: elsewhere in the control block (0x13 among them, the retired
        // `FAULT` slot), the telemetry block, and outside.
        for op in [0x13u8, 0x14, 0x2E, 0x40, 0x00, 0xFF] {
            assert_eq!(decode(op, &buf), None, "opcode {op:#04x}");
        }
        // Short payloads drop (no error propagates), per the best-effort class. For
        // `CYCLIC_STATE` that is every length but its two, so the sixteen committed bytes, a
        // thirty-fifth byte and a truncated two-motor frame drop here as well.
        assert_eq!(decode(OP_CYCLIC_STATE, &buf[..CyclicState::LEN]), None);
        assert_eq!(decode(OP_CYCLIC_STATE, &buf[..CyclicState::LEN - 1]), None);
        assert_eq!(
            decode(OP_CYCLIC_STATE, &[0u8; CyclicState::ENCODED_LEN + 1]),
            None
        );
        assert_eq!(
            decode(OP_CYCLIC_STATE, &[0u8; CyclicState::TWO_MOTOR_LEN - 1]),
            None
        );
        assert_eq!(decode(OP_DRIVE_CMD, &buf[..DriveCmd::LEN - 1]), None);
        assert_eq!(decode(OP_INPUTS, &buf[..Inputs::LEN - 1]), None);
    }

    // -- FOC_TRACE and TRACE_SET (`specs/link-control.md`; `specs/flux-observer.md`) -----------

    fn trace_sample() -> FocTrace {
        FocTrace {
            window: 0x1234,
            id: -2,
            iq: 300,
            iq_ref: 0x0102,
            vq: -31_999,
            hall_angle: 0xABCD,
            obs_angle: 0xA0B0,
            err_mean: -273,
            err_spread: 182,
            flux_mag: 21_000,
            speed: -7,
            held_a: 18,
            held_b: 27,
            coasted: 5,
            cause: 0x2A,
            flags: FocTrace::FLAG_TRACKED | FocTrace::FLAG_IN_WINDOW,
            method: 2,
            lock_bin: 4,
            motor: 1,
            track: [
                255, 255, 255, 255, 250, 249, 0, 125, 1, 2, 3, 4, 5, 6, 7, 255,
            ],
        }
    }

    #[test]
    fn foc_trace_encodes_to_the_pinned_bytes_and_back() {
        let t = trace_sample();
        let mut buf = [0u8; FocTrace::LEN];
        assert_eq!(t.encode(&mut buf), FocTrace::LEN);
        assert_eq!(
            buf,
            [
                0x34, 0x12, // window
                0xFE, 0xFF, // id -2
                0x2C, 0x01, // iq 300
                0x02, 0x01, // iq_ref 0x0102
                0x01, 0x83, // vq -31999
                0xCD, 0xAB, // hall_angle
                0xB0, 0xA0, // obs_angle
                0xEF, 0xFE, // err_mean -273
                0xB6, 0x00, // err_spread 182
                0x08, 0x52, // flux_mag 21000
                0xF9, 0xFF, // speed -7
                18, 27, 5, 0x2A, // held_a, held_b, coasted, cause
                0x0A, 2, 4, 1, // flags, method, lock_bin, motor
                255, 255, 255, 255, 250, 249, 0, 125, 1, 2, 3, 4, 5, 6, 7,
                255, // track, from 30
            ]
        );
        assert_eq!(FocTrace::decode(&buf), Ok(t));
        // The board's dispatch does not take it: a board sends `FOC_TRACE`, never consumes it.
        assert_eq!(decode(OP_FOC_TRACE, &buf), None);
        // The committed prefix: a byte short is refused, a trailing byte is ignored.
        assert_eq!(
            FocTrace::decode(&buf[..FocTrace::LEN - 1]),
            Err(DecodeError::TooShort)
        );
        let mut long = [0u8; FocTrace::LEN + 1];
        long[..FocTrace::LEN].copy_from_slice(&buf);
        assert_eq!(FocTrace::decode(&long), Ok(t));
    }

    #[test]
    fn trace_set_encodes_to_the_pinned_bytes_and_back() {
        let s = TraceSet {
            divisor: TraceSet::RESET | 25,
            motor: 1,
        };
        let mut buf = [0u8; TraceSet::LEN];
        assert_eq!(s.encode(&mut buf), TraceSet::LEN);
        assert_eq!(buf, [0x99, 0x01]);
        assert_eq!(decode(OP_TRACE_SET, &buf), Some(Payload::TraceSet(s)));
        assert_eq!(decode(OP_TRACE_SET, &buf[..1]), None);
        assert!(s.reset());
        assert_eq!(s.effective_divisor(), 25);
    }

    /// A divisor of 1 to 4 reads as 5; 0 is off; above 4 is itself; the reset bit is not part of
    /// it.
    #[test]
    fn a_divisor_of_one_to_four_reads_as_five() {
        for d in 1..=4u8 {
            let s = TraceSet {
                divisor: d,
                motor: 0,
            };
            assert_eq!(s.effective_divisor(), 5, "{d}");
            let s = TraceSet {
                divisor: d | TraceSet::RESET,
                motor: 0,
            };
            assert_eq!(s.effective_divisor(), 5, "{d} with reset");
        }
        for (d, want) in [
            (0u8, 0u8),
            (5, 5),
            (6, 6),
            (25, 25),
            (0x7F, 0x7F),
            (0x80, 0),
        ] {
            assert_eq!(
                TraceSet {
                    divisor: d,
                    motor: 0
                }
                .effective_divisor(),
                want,
                "{d}"
            );
        }
    }

    /// The subscription lasts 1.5 s from the last `TRACE_SET` with a non-zero divisor, and a
    /// renewal inside it keeps it; a divisor of 0 ends it at once.
    #[test]
    fn the_subscription_expires_at_one_and_a_half_seconds_and_renews() {
        let mut sub = TraceSubscription::new();
        assert!(!sub.active(0));
        let on = TraceSet {
            divisor: 5,
            motor: 0,
        };
        assert_eq!(sub.apply(on, 1000, true), None);
        assert!(sub.active(1000));
        assert!(sub.active(1000 + TRACE_TIMEOUT_TICKS - 1));
        assert!(!sub.active(1000 + TRACE_TIMEOUT_TICKS));
        // Renewed at 2 Hz (125 ticks) it never lapses.
        let mut t = 1000;
        for _ in 0..20 {
            t += 125;
            sub.apply(on, t, true);
            assert!(sub.active(t + 124));
        }
        // Across the tick counter's wrap.
        sub.apply(on, u32::MAX - 10, true);
        assert!(sub.active(5));
        // Off at once.
        sub.apply(
            TraceSet {
                divisor: 0,
                motor: 0,
            },
            t + 1,
            true,
        );
        assert!(!sub.active(t + 1));
        assert_eq!(sub.divisor(), 0);
    }

    /// The reset bit applies with any divisor, 0 included, and names the motor in the frame; a
    /// motor the board did not bring up is refused with no change and no reset.
    #[test]
    fn the_reset_bit_applies_with_a_divisor_of_zero_and_a_motor_not_up_is_refused() {
        let mut sub = TraceSubscription::new();
        let reset_only = TraceSet {
            divisor: TraceSet::RESET,
            motor: 0,
        };
        assert_eq!(sub.apply(reset_only, 10, true), Some(0));
        assert!(
            !sub.active(10),
            "a reset with divisor 0 subscribes to nothing"
        );
        sub.apply(
            TraceSet {
                divisor: 10,
                motor: 0,
            },
            20,
            true,
        );
        assert_eq!(
            sub.apply(
                TraceSet {
                    divisor: TraceSet::RESET | 10,
                    motor: 1
                },
                30,
                false
            ),
            None,
            "motor 1 not up: refused, no reset"
        );
        assert!(sub.active(30));
        assert_eq!(sub.due(100, 30), Some(0), "still motor 0");
        assert_eq!(
            sub.apply(
                TraceSet {
                    divisor: TraceSet::RESET | 10,
                    motor: 1
                },
                40,
                true
            ),
            Some(1)
        );
        assert_eq!(
            sub.due(110, 40),
            Some(1),
            "the new motor from the next window"
        );
    }

    /// The emission cadence: the first window of a subscription, then every `divisor` windows,
    /// across the counter's wrap; nothing once expired.
    #[test]
    fn the_emission_is_every_divisor_windows() {
        let mut sub = TraceSubscription::new();
        sub.apply(
            TraceSet {
                divisor: 5,
                motor: 0,
            },
            0,
            true,
        );
        let emitted: std::vec::Vec<u16> = (0xFFF0u16..=0xFFFF)
            .chain(0..20)
            .filter(|w| sub.due(*w, 1).is_some())
            .collect();
        assert_eq!(emitted, [0xFFF0, 0xFFF5, 0xFFFA, 0xFFFF, 4, 9, 14, 19]);
        // The same window twice emits once.
        assert_eq!(sub.due(24, 1), Some(0));
        assert_eq!(sub.due(24, 1), None);
        assert_eq!(sub.due(100, TRACE_TIMEOUT_TICKS), None, "expired");
        // A fresh subscription after an expiry emits at its first window.
        sub.apply(
            TraceSet {
                divisor: 5,
                motor: 0,
            },
            1000,
            true,
        );
        assert_eq!(sub.due(25, 1000), Some(0));
    }
}
