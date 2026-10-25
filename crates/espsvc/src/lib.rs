// SPDX-License-Identifier: GPL-3.0-or-later
//! The hoverboard bridge's commands (`specs/hoverboard-bridge.md`, "ESP commands").
//!
//! An ESP command is an ordinary L3 PDU (`[opcode][src][dst][payload]`, `crates/net`'s
//! [`Pdu`]) whose opcode is in the bridge range `0x80..=0x9F`. This crate owns the opcodes and the
//! payload bytes; the PDU header is `net`'s. All multi-byte fields are little-endian.
//!
//! | Opcode | Message | Payload |
//! | --- | --- | --- |
//! | `0x80` | [`Msg::InfoReq`] | none |
//! | `0x81` | [`Msg::Info`] | [`EspInfo`], 36 B |
//! | `0x82` | [`Msg::TargetIdReq`] | none |
//! | `0x83` | [`Msg::TargetId`] | [`TargetId`], 31 B |
//! | `0x85` | [`Msg::ClearProtection`] | none |
//! | `0x86` | [`Msg::FlashBegin`] | length `u32`, SHA-256 `[u8; 32]` |
//! | `0x87` | [`Msg::FlashChunk`] | offset `u32`, data (at most [`CHUNK`] B) |
//! | `0x88` | [`Msg::FlashAck`] | highest contiguous offset received `u32` |
//! | `0x89` | [`Msg::FlashStatus`] | stage `u8`, progress `u32`, status `u8` |
//! | `0x90` | [`Msg::TargetCtrl`] | [`Ctrl`] `u8` |
//! | `0x92` | [`Msg::MemRead`] | address `u32`, length `u16` (at most [`MEM_MAX`]) |
//! | `0x93` | [`Msg::MemData`] | address `u32`, data |
//! | `0x94` | [`Msg::MemWrite`] | address `u32`, data (word-aligned, whole words) |
//! | `0x98` | [`Msg::UpdateBegin`] | length `u32`, SHA-256 `[u8; 32]`, version `[u8; 3]` |
//! | `0x99` | [`Msg::UpdateChunk`] | offset `u32`, data (at most [`CHUNK`] B) |
//! | `0x9A` | [`Msg::UpdateAck`] | highest contiguous offset received `u32` |
//! | `0x9B` | [`Msg::UpdateConfirm`] | none |
//! | `0x9F` | [`Msg::Result`] | the opcode answered `u8`, [`Status`] `u8` |
//!
//! `ESP_UPDATE_*` are the bridge's own slot updates, with the bulk window of the board flash under
//! their own opcodes, so a bridge image can never be written to a board. The end of an update is a
//! `RESULT` answering `ESP_UPDATE_CHUNK`: `Ok` once the whole image is in the inactive slot, its
//! SHA-256 and its app descriptor are checked, and the slot is the next boot's; the bridge then
//! resets. The new image boots pending verification until `ESP_UPDATE_CONFIRM`.
#![no_std]

use net::pdu::{Pdu, PduError, HEADER_LEN};

/// `ESP_INFO` request.
pub const OP_INFO_REQ: u8 = 0x80;
/// `ESP_INFO` reply.
pub const OP_INFO: u8 = 0x81;
/// `TARGET_ID` request.
pub const OP_TARGET_ID_REQ: u8 = 0x82;
/// `TARGET_ID` reply.
pub const OP_TARGET_ID: u8 = 0x83;
/// `CLEAR_PROTECTION`.
pub const OP_CLEAR_PROTECTION: u8 = 0x85;
/// `FLASH_BEGIN`.
pub const OP_FLASH_BEGIN: u8 = 0x86;
/// `FLASH_CHUNK`.
pub const OP_FLASH_CHUNK: u8 = 0x87;
/// `FLASH_ACK`.
pub const OP_FLASH_ACK: u8 = 0x88;
/// `FLASH_STATUS`.
pub const OP_FLASH_STATUS: u8 = 0x89;
/// `TARGET_CTRL`.
pub const OP_TARGET_CTRL: u8 = 0x90;
/// `MEM_READ`.
pub const OP_MEM_READ: u8 = 0x92;
/// `MEM_DATA`.
pub const OP_MEM_DATA: u8 = 0x93;
/// `MEM_WRITE`.
pub const OP_MEM_WRITE: u8 = 0x94;
/// `ESP_UPDATE_BEGIN`.
pub const OP_UPDATE_BEGIN: u8 = 0x98;
/// `ESP_UPDATE_CHUNK`.
pub const OP_UPDATE_CHUNK: u8 = 0x99;
/// `ESP_UPDATE_ACK`.
pub const OP_UPDATE_ACK: u8 = 0x9A;
/// `ESP_UPDATE_CONFIRM`.
pub const OP_UPDATE_CONFIRM: u8 = 0x9B;
/// `RESULT`.
pub const OP_RESULT: u8 = 0x9F;

/// Is `op` in the bridge command range `0x80..=0x9F` (`specs/l3.md` records it)?
pub const fn is_esp_opcode(op: u8) -> bool {
    op >= 0x80 && op <= 0x9F
}

/// The largest PDU any link carries (`net::walk::MAX_PDU`).
pub const MAX_PDU: usize = net::walk::MAX_PDU;
/// Data bytes per `FLASH_CHUNK`: what is left of [`MAX_PDU`] after the header and the 4 B offset,
/// rounded down to a multiple of 16 (`specs/hoverboard-bridge.md`, "Bulk transfers").
pub const CHUNK: usize = 48;
const _: () = assert!(HEADER_LEN + 4 + CHUNK <= MAX_PDU);
/// Chunks in flight before the sender waits for a `FLASH_ACK`.
pub const WINDOW: usize = 8;
/// The most bytes one `MEM_READ` / `MEM_WRITE` moves: a multiple of 4 that fits a `MEM_DATA`.
pub const MEM_MAX: usize = 56;
const _: () = assert!(HEADER_LEN + 4 + MEM_MAX <= MAX_PDU);

/// A command's outcome, carried in `RESULT` and `FLASH_STATUS`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Status {
    /// Done.
    Ok = 0,
    /// The SWD engine is held by a flash or a protection clear.
    Busy = 1,
    /// A malformed or out-of-range request (a length, an alignment, an offset).
    BadRequest = 2,
    /// The target did not answer on SWD (connect failed, or a transfer ran out of retries).
    NoTarget = 3,
    /// Timer 0's `MOE` is still set after the disarm write: cut the rail.
    Armed = 4,
    /// Read protection is on (the protection byte is not `0xA5`): clear it first.
    Protected = 5,
    /// The write reaches the stored configuration's pages, which the bridge never writes.
    StoreRange = 6,
    /// The image is larger than the flash buffer, or the part is one this version does not flash.
    TooLarge = 7,
    /// The received image's SHA-256 does not match `FLASH_BEGIN`'s.
    ShaMismatch = 8,
    /// The flash controller reported `PGERR` or `WPERR`, or an erase did not leave `0xFFFF`.
    FlashError = 9,
    /// The read-back after programming differs from the buffer.
    VerifyMismatch = 10,
    /// A target operation did not finish in time (a halt, the flash `BUSY` bit, the reboot).
    Timeout = 11,
    /// Clearing protection did not take: the protection byte is not `0xA5`, or the USER byte
    /// changed.
    ClearFailed = 12,
    /// The update is not an ESP32-C3 app image, or its app descriptor's version is not the one
    /// `ESP_UPDATE_BEGIN` announced.
    BadImage = 13,
}

impl Status {
    /// Decode a wire byte.
    pub fn from_u8(b: u8) -> Option<Status> {
        use Status::*;
        Some(match b {
            0 => Ok,
            1 => Busy,
            2 => BadRequest,
            3 => NoTarget,
            4 => Armed,
            5 => Protected,
            6 => StoreRange,
            7 => TooLarge,
            8 => ShaMismatch,
            9 => FlashError,
            10 => VerifyMismatch,
            11 => Timeout,
            12 => ClearFailed,
            13 => BadImage,
            _ => return None,
        })
    }

    /// What it means, for a terminal.
    pub fn describe(self) -> &'static str {
        match self {
            Status::Ok => "ok",
            Status::Busy => "busy: a flash or a protection clear holds the SWD engine",
            Status::BadRequest => "bad request",
            Status::NoTarget => "no target: SWD did not answer",
            Status::Armed => "armed: MOE is still set after the disarm write; cut the rail",
            Status::Protected => "read protection is on; run clear-protection first",
            Status::StoreRange => "refused: the range reaches the stored configuration's pages",
            Status::TooLarge => {
                "too large for the flash buffer, or a part this version does not flash"
            }
            Status::ShaMismatch => "the received image's SHA-256 does not match",
            Status::FlashError => "the flash controller reported an error",
            Status::VerifyMismatch => "the read-back differs from the image",
            Status::Timeout => "the target did not finish in time",
            Status::ClearFailed => {
                "clearing protection did not take (protection byte or USER byte wrong)"
            }
            Status::BadImage => {
                "not an ESP32-C3 app image, or not the version announced at the start"
            }
        }
    }
}

/// A `TARGET_CTRL` action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Ctrl {
    /// Clear `MOE` and read it back.
    Disarm = 0,
    /// Disarm, set the debug hold, halt.
    Halt = 1,
    /// Resume a halted core.
    Resume = 2,
    /// `SYSRESETREQ`.
    Reset = 3,
    /// Disarm, set the debug hold, reset and halt at the reset vector.
    ResetHalt = 4,
}

impl Ctrl {
    /// Decode a wire byte.
    pub fn from_u8(b: u8) -> Option<Ctrl> {
        Some(match b {
            0 => Ctrl::Disarm,
            1 => Ctrl::Halt,
            2 => Ctrl::Resume,
            3 => Ctrl::Reset,
            4 => Ctrl::ResetHalt,
            _ => return None,
        })
    }
}

/// A `FLASH_STATUS` stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Stage {
    /// Chunks are arriving.
    Receiving = 0,
    /// Pages are being erased; progress counts bytes erased.
    Erasing = 1,
    /// Halfwords are being programmed; progress counts bytes programmed.
    Programming = 2,
    /// The range is being read back; progress counts bytes compared.
    Verifying = 3,
    /// The board was reset and its mailbox header reads `MBX1` again.
    Booted = 4,
    /// The flash stopped; the status byte says why.
    Failed = 5,
}

impl Stage {
    /// Decode a wire byte.
    pub fn from_u8(b: u8) -> Option<Stage> {
        Some(match b {
            0 => Stage::Receiving,
            1 => Stage::Erasing,
            2 => Stage::Programming,
            3 => Stage::Verifying,
            4 => Stage::Booted,
            5 => Stage::Failed,
            _ => return None,
        })
    }
}

/// The bridge's own state (`ESP_INFO`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EspInfo {
    /// Firmware version `[major, minor, patch]`.
    pub version: [u8; 3],
    /// The first 32 bits of the build's git commit (`0` when built outside git).
    pub commit: u32,
    /// The chip: [`CHIP_ESP32C3`].
    pub chip: u8,
    /// The SWD clock the engine measured at boot, in kHz.
    pub swd_khz: u16,
    /// Transfers retried on `WAIT`.
    pub waits: u32,
    /// Transfers retried on `FAULT`.
    pub faults: u32,
    /// Reads whose parity bit was wrong.
    pub parity_errors: u32,
    /// Line resets and reconnects after a transfer got no `ACK`.
    pub reconnects: u32,
    /// The mailbox port: [`MailboxState`] as a byte.
    pub mailbox: u8,
    /// The Wi-Fi station: [`WifiState`] as a byte.
    pub wifi: u8,
    /// The station's IPv4 address while [`WifiState::Up`], else zero.
    pub ip: [u8; 4],
    /// The app slot running: `0` for `ota_0`, `1` for `ota_1`, [`SLOT_NONE`] when it is not one
    /// of them.
    pub slot: u8,
    /// The running slot booted from an update and waits for `ESP_UPDATE_CONFIRM`; a reset before
    /// it returns to the other slot.
    pub pending: bool,
    /// The bridge's L3 node address, `0x00` while unassigned (`specs/hoverboard-bridge.md`, "The bridge as a
    /// node").
    pub node: u8,
    /// Whether the bridge's store mounted at boot. When it did not, the bridge runs unassigned with the
    /// default name, and nothing written to it survives a reset.
    pub store_mounted: bool,
}

/// `EspInfo::slot` when the running image is in neither app slot.
pub const SLOT_NONE: u8 = 0xFF;

/// The Wi-Fi station's state, in `ESP_INFO`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WifiState {
    /// Not associated with the network.
    Down = 0,
    /// Associated, with no address yet.
    Joined = 1,
    /// Associated and addressed: the console and the mDNS name are served.
    Up = 2,
}

impl WifiState {
    /// Decode a wire byte.
    pub fn from_u8(b: u8) -> Option<WifiState> {
        Some(match b {
            0 => WifiState::Down,
            1 => WifiState::Joined,
            2 => WifiState::Up,
            _ => return None,
        })
    }
}

/// `EspInfo::chip` for an ESP32-C3.
pub const CHIP_ESP32C3: u8 = 1;

/// The mailbox port's state, in `ESP_INFO`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MailboxState {
    /// Attached and acknowledged: frames flow.
    Up = 0,
    /// No target on SWD.
    NoTarget = 1,
    /// The header at the mailbox base is not `MBX1` (no firmware, or not ours, or not running).
    NoHeader = 2,
    /// The header's layout version differs from this build's.
    VersionMismatch = 3,
    /// The image's store registry fingerprint differs from this build's.
    RegistryMismatch = 4,
    /// Attached, waiting for the firmware's `epoch_ack`.
    Attaching = 5,
    /// Taken down while a flash or a protection clear holds the engine.
    Held = 6,
}

impl MailboxState {
    /// Decode a wire byte.
    pub fn from_u8(b: u8) -> Option<MailboxState> {
        use MailboxState::*;
        Some(match b {
            0 => Up,
            1 => NoTarget,
            2 => NoHeader,
            3 => VersionMismatch,
            4 => RegistryMismatch,
            5 => Attaching,
            6 => Held,
            _ => return None,
        })
    }
}

/// The target's identity (`TARGET_ID`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TargetId {
    /// The debug port ID (`0x1BA01477` on both parts).
    pub dpidr: u32,
    /// The debug MCU ID at `0xE004_2000` (does not tell the families apart).
    pub mcu_id: u32,
    /// `CPUID` at `0xE000_ED00`.
    pub cpuid: u32,
    /// [`FAMILY_F10X`] or [`FAMILY_F1X0`], told by where the UID is.
    pub family: u8,
    /// The 96-bit unique ID.
    pub uid: [u8; 12],
    /// The read-protection byte (`0xA5` = off).
    pub rdp: u8,
    /// The USER option byte.
    pub user: u8,
    /// The check of the last protection clear: [`CLEAR_NONE`], [`CLEAR_PASS`] or [`CLEAR_FAIL`].
    pub clear_check: u8,
    /// The flash size register, in KiB.
    pub flash_kib: u16,
    /// Timer 0's `MOE` is set.
    pub armed: bool,
}

/// `TargetId::family`: GD32F10x (UID at `0x1FFF_F7E8`).
pub const FAMILY_F10X: u8 = 0;
/// `TargetId::family`: GD32F1x0 (UID at `0x1FFF_F7AC`).
pub const FAMILY_F1X0: u8 = 1;
/// `TargetId::clear_check`: no protection clear recorded since the bridge booted.
pub const CLEAR_NONE: u8 = 0;
/// `TargetId::clear_check`: the protection byte is `0xA5` and the USER byte is the one recorded.
pub const CLEAR_PASS: u8 = 1;
/// `TargetId::clear_check`: the clear did not take.
pub const CLEAR_FAIL: u8 = 2;

/// One decoded ESP command or reply. Borrowed data points into the PDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Msg<'a> {
    /// `ESP_INFO` request.
    InfoReq,
    /// `ESP_INFO` reply.
    Info(EspInfo),
    /// `TARGET_ID` request.
    TargetIdReq,
    /// `TARGET_ID` reply.
    TargetId(TargetId),
    /// `CLEAR_PROTECTION`.
    ClearProtection,
    /// `FLASH_BEGIN`.
    FlashBegin {
        /// Image length in bytes.
        len: u32,
        /// SHA-256 of the image.
        sha256: [u8; 32],
    },
    /// `FLASH_CHUNK`.
    FlashChunk {
        /// Byte offset into the image, a multiple of [`CHUNK`].
        offset: u32,
        /// The bytes.
        data: &'a [u8],
    },
    /// `FLASH_ACK`: every byte below `contiguous` has arrived.
    FlashAck {
        /// Highest contiguous offset received.
        contiguous: u32,
    },
    /// `FLASH_STATUS`.
    FlashStatus {
        /// Where the flash is.
        stage: Stage,
        /// Bytes done in this stage.
        progress: u32,
        /// Why it failed, or `Ok`.
        status: Status,
    },
    /// `TARGET_CTRL`.
    TargetCtrl(Ctrl),
    /// `MEM_READ`.
    MemRead {
        /// Start address.
        addr: u32,
        /// Byte count, at most [`MEM_MAX`].
        len: u16,
    },
    /// `MEM_DATA`.
    MemData {
        /// Start address.
        addr: u32,
        /// The bytes.
        data: &'a [u8],
    },
    /// `MEM_WRITE`.
    MemWrite {
        /// Start address, word-aligned.
        addr: u32,
        /// The bytes, a whole number of words, at most [`MEM_MAX`].
        data: &'a [u8],
    },
    /// `ESP_UPDATE_BEGIN`.
    UpdateBegin {
        /// Image length in bytes.
        len: u32,
        /// SHA-256 of the image.
        sha256: [u8; 32],
        /// The version in the image's app descriptor, `[major, minor, patch]` ([`app_version`]).
        version: [u8; 3],
    },
    /// `ESP_UPDATE_CHUNK`.
    UpdateChunk {
        /// Byte offset into the image, a multiple of [`CHUNK`].
        offset: u32,
        /// The bytes.
        data: &'a [u8],
    },
    /// `ESP_UPDATE_ACK`: every byte below `contiguous` has arrived.
    UpdateAck {
        /// Highest contiguous offset received.
        contiguous: u32,
    },
    /// `ESP_UPDATE_CONFIRM`: mark the running slot valid.
    UpdateConfirm,
    /// `RESULT`.
    Result {
        /// The opcode answered.
        op: u8,
        /// The outcome.
        status: Status,
    },
}

/// Why a PDU did not decode as an ESP command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Not an opcode this version knows.
    UnknownOpcode(u8),
    /// The payload length or a field value is wrong for the opcode.
    Malformed(u8),
}

/// Why [`Msg::encode`] failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The data does not fit one PDU.
    TooLong,
    /// `out` is too small.
    Pdu(PduError),
}

fn le32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn le16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

/// A little cursor over a payload buffer.
struct W<'a> {
    buf: &'a mut [u8],
    n: usize,
}

impl W<'_> {
    fn put(&mut self, b: &[u8]) {
        self.buf[self.n..self.n + b.len()].copy_from_slice(b);
        self.n += b.len();
    }
}

const INFO_LEN: usize = 36;
const TARGET_ID_LEN: usize = 31;

impl<'a> Msg<'a> {
    /// This message's opcode.
    pub fn opcode(&self) -> u8 {
        match self {
            Msg::InfoReq => OP_INFO_REQ,
            Msg::Info(_) => OP_INFO,
            Msg::TargetIdReq => OP_TARGET_ID_REQ,
            Msg::TargetId(_) => OP_TARGET_ID,
            Msg::ClearProtection => OP_CLEAR_PROTECTION,
            Msg::FlashBegin { .. } => OP_FLASH_BEGIN,
            Msg::FlashChunk { .. } => OP_FLASH_CHUNK,
            Msg::FlashAck { .. } => OP_FLASH_ACK,
            Msg::FlashStatus { .. } => OP_FLASH_STATUS,
            Msg::TargetCtrl(_) => OP_TARGET_CTRL,
            Msg::MemRead { .. } => OP_MEM_READ,
            Msg::MemData { .. } => OP_MEM_DATA,
            Msg::MemWrite { .. } => OP_MEM_WRITE,
            Msg::UpdateBegin { .. } => OP_UPDATE_BEGIN,
            Msg::UpdateChunk { .. } => OP_UPDATE_CHUNK,
            Msg::UpdateAck { .. } => OP_UPDATE_ACK,
            Msg::UpdateConfirm => OP_UPDATE_CONFIRM,
            Msg::Result { .. } => OP_RESULT,
        }
    }

    /// Encode the whole PDU (`[opcode][src][dst][payload]`) into `out`, returning its length.
    pub fn encode(&self, src: u8, dst: u8, out: &mut [u8]) -> Result<usize, EncodeError> {
        let mut payload = [0u8; MAX_PDU - HEADER_LEN];
        let n = self.encode_payload(&mut payload)?;
        Pdu::new(self.opcode(), src, dst, &payload[..n])
            .and_then(|p| p.encode(out))
            .map_err(EncodeError::Pdu)
    }

    fn encode_payload(&self, buf: &mut [u8; MAX_PDU - HEADER_LEN]) -> Result<usize, EncodeError> {
        let mut w = W { buf, n: 0 };
        let room = MAX_PDU - HEADER_LEN;
        match *self {
            Msg::InfoReq | Msg::TargetIdReq | Msg::ClearProtection | Msg::UpdateConfirm => {}
            Msg::Info(i) => {
                w.put(&i.version);
                w.put(&i.commit.to_le_bytes());
                w.put(&[i.chip]);
                w.put(&i.swd_khz.to_le_bytes());
                w.put(&i.waits.to_le_bytes());
                w.put(&i.faults.to_le_bytes());
                w.put(&i.parity_errors.to_le_bytes());
                w.put(&i.reconnects.to_le_bytes());
                w.put(&[i.mailbox, i.wifi]);
                w.put(&i.ip);
                w.put(&[i.slot, i.pending as u8]);
                w.put(&[i.node, i.store_mounted as u8]);
            }
            Msg::TargetId(t) => {
                w.put(&t.dpidr.to_le_bytes());
                w.put(&t.mcu_id.to_le_bytes());
                w.put(&t.cpuid.to_le_bytes());
                w.put(&[t.family]);
                w.put(&t.uid);
                w.put(&[t.rdp, t.user, t.clear_check]);
                w.put(&t.flash_kib.to_le_bytes());
                w.put(&[t.armed as u8]);
            }
            Msg::FlashBegin { len, sha256 } => {
                w.put(&len.to_le_bytes());
                w.put(&sha256);
            }
            Msg::UpdateBegin {
                len,
                sha256,
                version,
            } => {
                w.put(&len.to_le_bytes());
                w.put(&sha256);
                w.put(&version);
            }
            Msg::FlashChunk { offset, data } | Msg::UpdateChunk { offset, data } => {
                if data.len() > CHUNK {
                    return Err(EncodeError::TooLong);
                }
                w.put(&offset.to_le_bytes());
                w.put(data);
            }
            Msg::FlashAck { contiguous } | Msg::UpdateAck { contiguous } => {
                w.put(&contiguous.to_le_bytes())
            }
            Msg::FlashStatus {
                stage,
                progress,
                status,
            } => {
                w.put(&[stage as u8]);
                w.put(&progress.to_le_bytes());
                w.put(&[status as u8]);
            }
            Msg::TargetCtrl(c) => w.put(&[c as u8]),
            Msg::MemRead { addr, len } => {
                w.put(&addr.to_le_bytes());
                w.put(&len.to_le_bytes());
            }
            Msg::MemData { addr, data } | Msg::MemWrite { addr, data } => {
                if data.len() > MEM_MAX || 4 + data.len() > room {
                    return Err(EncodeError::TooLong);
                }
                w.put(&addr.to_le_bytes());
                w.put(data);
            }
            Msg::Result { op, status } => w.put(&[op, status as u8]),
        }
        Ok(w.n)
    }

    /// Decode a PDU's opcode and payload. The header's `src`/`dst` are the caller's to route on.
    pub fn decode(pdu: &Pdu<'a>) -> Result<Msg<'a>, DecodeError> {
        let op = pdu.opcode;
        let p = pdu.payload;
        let bad = DecodeError::Malformed(op);
        let exact = |n: usize| if p.len() == n { Ok(()) } else { Err(bad) };
        Ok(match op {
            OP_INFO_REQ => {
                exact(0)?;
                Msg::InfoReq
            }
            OP_INFO => {
                exact(INFO_LEN)?;
                Msg::Info(EspInfo {
                    version: [p[0], p[1], p[2]],
                    commit: le32(&p[3..]),
                    chip: p[7],
                    swd_khz: le16(&p[8..]),
                    waits: le32(&p[10..]),
                    faults: le32(&p[14..]),
                    parity_errors: le32(&p[18..]),
                    reconnects: le32(&p[22..]),
                    mailbox: p[26],
                    wifi: p[27],
                    ip: [p[28], p[29], p[30], p[31]],
                    slot: p[32],
                    pending: p[33] != 0,
                    node: p[34],
                    store_mounted: p[35] != 0,
                })
            }
            OP_TARGET_ID_REQ => {
                exact(0)?;
                Msg::TargetIdReq
            }
            OP_TARGET_ID => {
                exact(TARGET_ID_LEN)?;
                let mut uid = [0u8; 12];
                uid.copy_from_slice(&p[13..25]);
                Msg::TargetId(TargetId {
                    dpidr: le32(p),
                    mcu_id: le32(&p[4..]),
                    cpuid: le32(&p[8..]),
                    family: p[12],
                    uid,
                    rdp: p[25],
                    user: p[26],
                    clear_check: p[27],
                    flash_kib: le16(&p[28..]),
                    armed: p[30] != 0,
                })
            }
            OP_CLEAR_PROTECTION => {
                exact(0)?;
                Msg::ClearProtection
            }
            OP_FLASH_BEGIN => {
                exact(36)?;
                let mut sha256 = [0u8; 32];
                sha256.copy_from_slice(&p[4..36]);
                Msg::FlashBegin {
                    len: le32(p),
                    sha256,
                }
            }
            OP_FLASH_CHUNK | OP_UPDATE_CHUNK => {
                if p.len() < 5 || p.len() > 4 + CHUNK {
                    return Err(bad);
                }
                let (offset, data) = (le32(p), &p[4..]);
                if op == OP_FLASH_CHUNK {
                    Msg::FlashChunk { offset, data }
                } else {
                    Msg::UpdateChunk { offset, data }
                }
            }
            OP_FLASH_ACK | OP_UPDATE_ACK => {
                exact(4)?;
                let contiguous = le32(p);
                if op == OP_FLASH_ACK {
                    Msg::FlashAck { contiguous }
                } else {
                    Msg::UpdateAck { contiguous }
                }
            }
            OP_UPDATE_BEGIN => {
                exact(39)?;
                let mut sha256 = [0u8; 32];
                sha256.copy_from_slice(&p[4..36]);
                Msg::UpdateBegin {
                    len: le32(p),
                    sha256,
                    version: [p[36], p[37], p[38]],
                }
            }
            OP_UPDATE_CONFIRM => {
                exact(0)?;
                Msg::UpdateConfirm
            }
            OP_FLASH_STATUS => {
                exact(6)?;
                Msg::FlashStatus {
                    stage: Stage::from_u8(p[0]).ok_or(bad)?,
                    progress: le32(&p[1..]),
                    status: Status::from_u8(p[5]).ok_or(bad)?,
                }
            }
            OP_TARGET_CTRL => {
                exact(1)?;
                Msg::TargetCtrl(Ctrl::from_u8(p[0]).ok_or(bad)?)
            }
            OP_MEM_READ => {
                exact(6)?;
                Msg::MemRead {
                    addr: le32(p),
                    len: le16(&p[4..]),
                }
            }
            OP_MEM_DATA | OP_MEM_WRITE => {
                if p.len() < 4 || p.len() > 4 + MEM_MAX {
                    return Err(bad);
                }
                let (addr, data) = (le32(p), &p[4..]);
                if op == OP_MEM_DATA {
                    Msg::MemData { addr, data }
                } else {
                    Msg::MemWrite { addr, data }
                }
            }
            OP_RESULT => {
                exact(2)?;
                Msg::Result {
                    op: p[0],
                    status: Status::from_u8(p[1]).ok_or(bad)?,
                }
            }
            other => return Err(DecodeError::UnknownOpcode(other)),
        })
    }
}

/// The receiving end of a bulk transfer (`specs/hoverboard-bridge.md`, "Bulk transfers"): chunks arrive at
/// [`CHUNK`]-aligned offsets in any order, duplicated or not, and the acknowledgement is the
/// highest offset below which every byte has arrived. `N` is the buffer capacity in bytes.
pub struct BulkRx<const N: usize> {
    buf: [u8; N],
    len: u32,
    /// One bit per chunk.
    have: [u8; BITMAP_BYTES],
    contiguous: u32,
}

/// The most chunks a [`BulkRx`] tracks: 98,304 B at [`CHUNK`] each, above any board image.
pub const MAX_CHUNKS: usize = 2048;
const BITMAP_BYTES: usize = MAX_CHUNKS / 8;

/// Why [`BulkRx`] refused a chunk or a start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BulkError {
    /// The transfer is longer than the buffer.
    TooLarge,
    /// The offset is not chunk-aligned, past the end, or the chunk is the wrong length.
    BadChunk,
}

impl<const N: usize> BulkRx<N> {
    /// An idle receiver.
    pub const fn new() -> Self {
        const { assert!(N <= MAX_CHUNKS * CHUNK) };
        BulkRx {
            buf: [0xFF; N],
            len: 0,
            have: [0; BITMAP_BYTES],
            contiguous: 0,
        }
    }

    /// Start a transfer of `len` bytes, dropping any previous one.
    pub fn begin(&mut self, len: u32) -> Result<(), BulkError> {
        if len as usize > N || len == 0 {
            return Err(BulkError::TooLarge);
        }
        self.len = len;
        self.contiguous = 0;
        let chunks = (len as usize).div_ceil(CHUNK);
        self.have[..chunks.div_ceil(8)].fill(0);
        self.buf[..len as usize].fill(0xFF);
        Ok(())
    }

    /// Take one chunk. Returns the new highest contiguous offset, which is what `FLASH_ACK`
    /// carries. A chunk already held is accepted again unchanged (a duplicate).
    pub fn chunk(&mut self, offset: u32, data: &[u8]) -> Result<u32, BulkError> {
        let off = offset as usize;
        if !off.is_multiple_of(CHUNK) || offset >= self.len {
            return Err(BulkError::BadChunk);
        }
        let want = CHUNK.min(self.len as usize - off);
        if data.len() != want {
            return Err(BulkError::BadChunk);
        }
        self.buf[off..off + want].copy_from_slice(data);
        let i = off / CHUNK;
        self.have[i / 8] |= 1 << (i % 8);
        while self.contiguous < self.len {
            let j = self.contiguous as usize / CHUNK;
            if self.have[j / 8] & (1 << (j % 8)) == 0 {
                break;
            }
            self.contiguous = (self.contiguous + CHUNK as u32).min(self.len);
        }
        Ok(self.contiguous)
    }

    /// Every byte has arrived.
    pub fn complete(&self) -> bool {
        self.len > 0 && self.contiguous == self.len
    }

    /// The transfer's length (0 when idle).
    pub fn len(&self) -> u32 {
        self.len
    }

    /// No transfer has been started.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The bytes received so far (the whole image once [`complete`](Self::complete)).
    pub fn data(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// Forget the transfer.
    pub fn reset(&mut self) {
        self.len = 0;
        self.contiguous = 0;
    }
}

impl<const N: usize> Default for BulkRx<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// The receiving end of a bulk transfer whose bytes go straight on as they arrive (the bridge's own
/// update, written into a flash slot far larger than RAM) rather than into a buffer. Only the next
/// bytes in order are taken; a duplicate, or a chunk past a gap, is dropped and answered with the
/// same acknowledgement, which is all the go-back-N [`BulkTx`] needs: it resends from there.
#[derive(Debug, Clone, Copy, Default)]
pub struct StreamRx {
    len: u32,
    contiguous: u32,
}

impl StreamRx {
    /// An idle receiver.
    pub const fn new() -> Self {
        StreamRx {
            len: 0,
            contiguous: 0,
        }
    }

    /// Start a transfer of `len` bytes, dropping any previous one.
    pub fn begin(&mut self, len: u32) -> Result<(), BulkError> {
        if len == 0 {
            return Err(BulkError::TooLarge);
        }
        self.len = len;
        self.contiguous = 0;
        Ok(())
    }

    /// Look at one chunk. `Ok(true)`: these are the next bytes, the caller takes them, and
    /// [`contiguous`](Self::contiguous) has moved past them. `Ok(false)`: a duplicate or a chunk
    /// past a gap, dropped. Either way the acknowledgement is [`contiguous`](Self::contiguous).
    pub fn chunk(&mut self, offset: u32, data: &[u8]) -> Result<bool, BulkError> {
        let off = offset as usize;
        if !off.is_multiple_of(CHUNK) || offset >= self.len {
            return Err(BulkError::BadChunk);
        }
        if data.len() != CHUNK.min(self.len as usize - off) {
            return Err(BulkError::BadChunk);
        }
        if offset != self.contiguous {
            return Ok(false);
        }
        self.contiguous += data.len() as u32;
        Ok(true)
    }

    /// Every byte below this offset has been taken.
    pub fn contiguous(&self) -> u32 {
        self.contiguous
    }

    /// Every byte has arrived.
    pub fn complete(&self) -> bool {
        self.len > 0 && self.contiguous == self.len
    }

    /// The transfer's length (0 when idle).
    pub fn len(&self) -> u32 {
        self.len
    }

    /// No transfer has been started.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Forget the transfer.
    pub fn reset(&mut self) {
        *self = StreamRx::new();
    }
}

/// Where an ESP-IDF app image keeps its app descriptor: after the 24 B image header and the first
/// segment's 8 B header (`esp_app_desc_t`, which `esp_app_desc!` places first in that segment).
pub const APP_DESC_OFFSET: usize = 0x20;
/// The image header's first byte.
const IMAGE_MAGIC: u8 = 0xE9;
/// The image header's chip id (bytes 12..14) for an ESP32-C3.
const CHIP_ID_ESP32C3: u16 = 5;
/// `esp_app_desc_t::magic_word`.
const APP_DESC_MAGIC: u32 = 0xABCD_5432;
/// `esp_app_desc_t::version`: 32 B of text after the magic, the secure version and 8 reserved
/// bytes.
const APP_DESC_VERSION: usize = APP_DESC_OFFSET + 16;
/// The bytes of an image [`app_version`] reads.
pub const APP_HEAD: usize = APP_DESC_VERSION + 32;

/// The version an ESP32-C3 app image (`espflash save-image` output) carries in its app descriptor,
/// `[major, minor, patch]` from its leading `major.minor.patch`. `None` when `image` does not start
/// with an ESP32-C3 image header and an app descriptor, or the version is not in that form. Reads
/// the first [`APP_HEAD`] bytes.
pub fn app_version(image: &[u8]) -> Option<[u8; 3]> {
    if image.len() < APP_HEAD
        || image[0] != IMAGE_MAGIC
        || le16(&image[12..]) != CHIP_ID_ESP32C3
        || le32(&image[APP_DESC_OFFSET..]) != APP_DESC_MAGIC
    {
        return None;
    }
    let text = &image[APP_DESC_VERSION..APP_HEAD];
    let text = &text[..text.iter().position(|&b| b == 0).unwrap_or(text.len())];
    let mut out = [0u8; 3];
    let mut parts = text.splitn(3, |&b| b == b'.');
    for (i, slot) in out.iter_mut().enumerate() {
        let part = parts.next()?;
        // The patch number may carry a suffix (`1.2.3-rc1`, `1.2.3+abc`).
        let digits = if i == 2 {
            &part[..part
                .iter()
                .position(|b| !b.is_ascii_digit())
                .unwrap_or(part.len())]
        } else {
            part
        };
        if digits.is_empty() || digits.len() > 3 || !digits.iter().all(u8::is_ascii_digit) {
            return None;
        }
        let n = digits
            .iter()
            .fold(0u16, |n, &d| n * 10 + u16::from(d - b'0'));
        *slot = u8::try_from(n).ok()?;
    }
    Some(out)
}

/// The sending end of a bulk transfer: which chunk goes next. Go-back-N on the acknowledgement:
/// at most [`WINDOW`] chunks past the acknowledged offset are out, and a stall resends from the
/// acknowledged offset.
#[derive(Debug, Clone, Copy)]
pub struct BulkTx {
    len: u32,
    acked: u32,
    next: u32,
}

impl BulkTx {
    /// A sender for `len` bytes.
    pub fn new(len: u32) -> Self {
        BulkTx {
            len,
            acked: 0,
            next: 0,
        }
    }

    /// The next chunk to send, as `(offset, length)`, or `None` while the window is full or
    /// everything is out.
    pub fn next_chunk(&mut self) -> Option<(u32, usize)> {
        let window_end = self.acked + (WINDOW * CHUNK) as u32;
        if self.next >= self.len || self.next >= window_end {
            return None;
        }
        let off = self.next;
        let n = CHUNK.min((self.len - off) as usize);
        self.next += n as u32;
        Some((off, n))
    }

    /// Record an acknowledgement (`FLASH_ACK`, `ESP_UPDATE_ACK`). One below one already seen (a
    /// late duplicate) changes nothing.
    pub fn ack(&mut self, contiguous: u32) {
        let c = contiguous.min(self.len);
        if c > self.acked {
            self.acked = c;
        }
        if self.next < self.acked {
            self.next = self.acked;
        }
    }

    /// Nothing arrived for a while: send again from the acknowledged offset.
    pub fn rewind(&mut self) {
        self.next = self.acked;
    }

    /// The receiver has every byte.
    pub fn done(&self) -> bool {
        self.acked >= self.len
    }

    /// Bytes acknowledged.
    pub fn acked(&self) -> u32 {
        self.acked
    }
}

#[cfg(test)]
mod tests;
