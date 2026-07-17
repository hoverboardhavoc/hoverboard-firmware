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
//!   and no error propagates ([`decode`] returns `None`).
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

/// `CYCLIC_STATE` opcode: board <-> board, every tick (250 Hz), inter-board UART only.
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

// --- CYCLIC_STATE (11 B): the per-tick peer state mirror ---------------------------------------

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
    /// Filtered battery word (stock CB+0x20). Peer consumer: the PID `scale` input on boards
    /// without VBATT sense.
    pub battery: u16,
    /// The mode byte. Peer consumer: supervision/OBS.
    pub mode: u8,
    /// Latched fault code, 0 = healthy. Peer consumer: visibility/OBS.
    pub fault: u8,
    /// Flag bits: [`Self::FLAG_RIDER`] (bit0), [`Self::FLAG_LOCKDOWN`] (bit7).
    pub flags: u8,
}

impl CyclicState {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 11;

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

    /// Encode into `out`, returning the byte count ([`Self::LEN`]).
    pub fn encode(&self, out: &mut [u8]) -> usize {
        debug_assert!(out.len() >= Self::LEN);
        out[0..2].copy_from_slice(&self.pitch.to_le_bytes());
        out[2..4].copy_from_slice(&self.roll.to_le_bytes());
        out[4..6].copy_from_slice(&self.wheel_speed.to_le_bytes());
        out[6..8].copy_from_slice(&self.battery.to_le_bytes());
        out[8] = self.mode;
        out[9] = self.fault;
        out[10] = self.flags;
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
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
        })
    }
}

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

// --- INPUTS (4 B): remote input mirror ---------------------------------------------------------

/// Remote input mirror. Consumer: the input-assembly step (`ModeInputs.power_request` is
/// level-sensitive and copied over the link on a mirroring node, `specs/sensing-and-safety.md`).
/// Consumed from any port via normal L3 delivery; the firmware never originates it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inputs {
    /// Raw throttle word (feeds the inputs task's `ThrottleFilter`).
    pub throttle: i16,
    /// Button bits: [`Self::BUTTON_POWER`] (bit0).
    pub buttons: u8,
    /// Rider bits: [`Self::RIDER_PRESENT`] (bit0).
    pub rider: u8,
}

impl Inputs {
    /// On-wire length of the committed prefix.
    pub const LEN: usize = 4;

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
        out[0..2].copy_from_slice(&self.throttle.to_le_bytes());
        out[2] = self.buttons;
        out[3] = self.rider;
        Self::LEN
    }

    /// Decode the committed prefix; ignore trailing bytes.
    pub fn decode(b: &[u8]) -> Result<Inputs, DecodeError> {
        if b.len() < Self::LEN {
            return Err(DecodeError::TooShort);
        }
        Ok(Inputs {
            throttle: rd_i16(b, 0),
            buttons: b[2],
            rider: b[3],
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
        // 25 ticks = 100 ms at 250 Hz; 50 ticks = 200 ms.
        assert_eq!(CYCLIC_TIMEOUT_TICKS, 25);
        assert_eq!(DRIVE_TIMEOUT_TICKS, 50);
    }

    #[test]
    fn committed_lengths_pinned() {
        assert_eq!(CyclicState::LEN, 11);
        assert_eq!(DriveCmd::LEN, 5);
        assert_eq!(Inputs::LEN, 4);
        assert_eq!(Fault::LEN, 2);
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
        }
    }

    #[test]
    fn cyclic_state_wire_layout_is_little_endian() {
        let mut buf = [0u8; CyclicState::LEN];
        assert_eq!(cyclic_sample().encode(&mut buf), CyclicState::LEN);
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
            ]
        );
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
            throttle: 0x7FFF,
            buttons: Inputs::BUTTON_POWER,
            rider: Inputs::RIDER_PRESENT,
        };
        let mut buf = [0u8; Inputs::LEN];
        assert_eq!(inp.encode(&mut buf), Inputs::LEN);
        assert_eq!(buf, [0xFF, 0x7F, 0x01, 0x01]);
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
        let mut buf = [0u8; CyclicState::LEN];
        orig.encode(&mut buf);
        assert_eq!(CyclicState::decode(&buf), Ok(orig));
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
            throttle: -1,
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
            throttle: 100,
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
            throttle: 0,
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
        let mut buf = [0u8; 16];
        cyc.encode(&mut buf);
        assert_eq!(
            decode(OP_CYCLIC_STATE, &buf[..CyclicState::LEN]),
            Some(Payload::CyclicState(cyc))
        );

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
            throttle: 3,
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
