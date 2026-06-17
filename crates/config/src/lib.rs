//! Board-definition parse (Phase 5).
//!
//! Decodes the framed canonical-CBOR **board definition** blob into typed, bounded structs the node
//! runtime (a later phase) consumes. The board blob carries the wiring and tuning in *logical*
//! references only (no chip facts; those live in the `runtime-hal` MCU descriptor). This crate owns
//! the board-half format described in `todo/board-config.md` and `todo/board-config-tool.md`.
//!
//! # Frame layout (all little-endian)
//!
//! | offset | field     | type  | meaning                                                    |
//! |--------|-----------|-------|------------------------------------------------------------|
//! | 0      | `magic`   | `u32` | [`BOARD_MAGIC`] (ASCII `"HBRD"` as a little-endian `u32`)   |
//! | 4      | `version` | `u16` | [`VERSION`]; a different value is [`ConfigError::BadVersion`] |
//! | 6      | `length`  | `u16` | CBOR payload length in bytes                                |
//! | 8      | `crc`     | `u16` | CRC-16/MODBUS over the `length` payload bytes               |
//! | 10     | payload   | bytes | exactly `length` bytes of canonical CBOR                    |
//!
//! [`BOARD_MAGIC`] is distinct from runtime-hal's MCU-descriptor magic (`"RHLD"`), so the two flash
//! blobs cannot be confused for one another. The CRC is [`link::crc16::modbus`], byte-for-byte the
//! same CRC-16/MODBUS the link frame and the MCU descriptor use.
//!
//! # CBOR schema (string keys)
//!
//! The payload is a single CBOR map with string keys. The decoder is positional-agnostic and skips
//! unknown keys for forward compatibility. This mirrors runtime-hal's `parse.rs` string-key
//! map-decode style. Recognized top-level keys:
//!
//! - `"format"`: uint, the schema format byte.
//! - `"name"`: text, the board name.
//! - `"control"`: map `{ "mode": "articulated" | "rigid" }`.
//! - `"speed_sync"`: map `{ "peer": "link:<name>" }`.
//! - `"links"`: array of link maps (see [`LinkConfig`]).
//! - `"motors"`: array of motor maps (see [`MotorConfig`]).
//! - `"battery"`: map `{ "cells": uint }`.
//! - `"limits"`: map `{ "current_ma": uint, "speed_max": uint }`.
//!
//! `link:<name>` references in `speed_sync.peer`, `attitude_source`, and `drive_source` are resolved
//! to declared-link **indices** during the validation pass, so the returned structs carry indices,
//! not names.

#![no_std]

use heapless::{String, Vec};
use minicbor::data::Type;
use minicbor::Decoder;

use link::crc16;
use link::item::{DataItem, ItemSet};

// --- bounds -----------------------------------------------------------------------------------

/// Maximum number of links a board definition may declare.
pub const MAX_LINKS: usize = 4;
/// Maximum number of motors a board definition may declare.
pub const MAX_MOTORS: usize = 4;
/// Maximum number of fused attitude sources (link indices) a motor may name.
pub const MAX_FUSED: usize = 4;
/// Maximum length of a bounded name string (link name, usart label, board name, device name,
/// timer/phase/hall/current-sense labels).
pub const MAX_NAME: usize = 24;

/// A bounded name string used throughout the board definition.
pub type Name = String<MAX_NAME>;

/// Fixed frame header size in bytes (`magic` + `version` + `length` + `crc`).
pub const HEADER_LEN: usize = 10;

/// Board-definition frame magic: ASCII `"HBRD"` read as a little-endian `u32`. Distinct from
/// runtime-hal's MCU-descriptor magic (`"RHLD"`) so the two flash blobs are never confused.
pub const BOARD_MAGIC: u32 = u32::from_le_bytes(*b"HBRD");

/// Frame version this build decodes. A header `version` other than this is
/// [`ConfigError::BadVersion`].
pub const VERSION: u16 = 1;

// --- types ------------------------------------------------------------------------------------

/// Link arbitration setting (behavior comes from the produce/consume bindings, not this enum, but
/// the setting still gates who initiates on the wire).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Arbitration {
    Initiator,
    Follower,
    BusMaster,
    Async,
}

/// Control mode for the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Articulated,
    Rigid,
}

/// Where a motor's attitude (balance loop input) comes from. `link:<name>` references are resolved
/// to declared-link indices during the validation pass.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttitudeSource {
    /// The local IMU on this board.
    LocalImu,
    /// A single declared link, by resolved link index.
    Link(u8),
    /// A fusion of several declared links, by resolved link index.
    Fused(Vec<u8, MAX_FUSED>),
}

/// Where a motor's drive command comes from. `link:<name>` references are resolved to declared-link
/// indices during the validation pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveSource {
    /// The local balance loop on this board.
    LocalBalance,
    /// A declared link, by resolved link index (a follower motor).
    Link(u8),
}

/// One declared link.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LinkConfig {
    /// The link's logical name (used by `link:<name>` references, resolved away at parse time).
    pub name: Name,
    /// Logical USART label (GD 0-indexed, e.g. `"USART1"`).
    pub usart_label: Name,
    /// Target baud rate.
    pub baud: u32,
    /// This node's id on the link, `0..=0xFE`.
    pub node_id: u8,
    /// Arbitration setting.
    pub arbitration: Arbitration,
    /// Items this node produces onto the link.
    pub produce: ItemSet,
    /// Items this node consumes from the link.
    pub consume: ItemSet,
    /// For a BLE link, the advertised device name. Present iff this is a BLE link.
    pub device_name: Option<Name>,
}

/// One declared motor. Pin / channel resolution is runtime-hal's job, so the timer / phase / hall /
/// current-sense logical labels are carried as opaque bounded strings for now.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotorConfig {
    /// Logical timer label (e.g. `"TIMER0"`).
    pub timer_label: Name,
    /// Where the balance loop input comes from.
    pub attitude_source: AttitudeSource,
    /// Where the drive command comes from.
    pub drive_source: DriveSource,
    /// Phase-leg labels, opaque for now (`{hi, lo}` per leg flattened to `["a.hi","a.lo",...]`).
    pub phase_labels: Vec<Name, 6>,
    /// Hall pin labels, opaque for now.
    pub hall_labels: Vec<Name, 3>,
    /// Current-sense pin label, opaque for now.
    pub current_sense_label: Option<Name>,
}

/// Scaled-integer limits (no floats; canonical CBOR carries none).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Limits {
    /// Current limit in milliamps.
    pub current_ma: u32,
    /// Speed cap (scaled integer).
    pub speed_max: u32,
}

/// A decoded, validated board definition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardConfig {
    /// Schema format byte.
    pub format: u8,
    /// Board name.
    pub name: Name,
    /// Control mode.
    pub control_mode: ControlMode,
    /// `speed_sync.peer` resolved to a declared-link index, if present.
    pub speed_sync_peer: Option<u8>,
    /// Declared links.
    pub links: Vec<LinkConfig, MAX_LINKS>,
    /// Declared motors.
    pub motors: Vec<MotorConfig, MAX_MOTORS>,
    /// Battery cell count.
    pub battery_cells: u8,
    /// Limits.
    pub limits: Limits,
}

/// Errors from [`parse`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfigError {
    /// Frame magic did not match [`BOARD_MAGIC`] (a blank `0xFF` sector lands here first).
    BadMagic,
    /// Frame version did not match [`VERSION`].
    BadVersion,
    /// Frame length runs past the end of the blob.
    BadLength,
    /// CRC-16/MODBUS over the payload did not match the header CRC.
    CrcMismatch,
    /// The payload was not well-formed / decodable CBOR in the expected shape.
    Cbor,
    /// Two declared links share a name.
    DuplicateLinkName,
    /// A `link:<name>` reference named no declared link.
    UnresolvedLinkRef,
    /// A produce/consume entry named no known [`DataItem`].
    UnknownItem,
    /// A `node_id` was out of the `0..=0xFE` range.
    BadNodeId,
    /// More links / motors / fused sources than the bounds allow, or a name too long.
    TooMany,
    /// A link declares a `device_name` (a BLE link) but is not `arbitration: async`.
    BleNotAsync,
}

// --- public entry point -----------------------------------------------------------------------

/// Parse a framed board-definition blob into a validated [`BoardConfig`].
///
/// Steps, in order:
/// 1. frame: length / magic / version / payload bounds,
/// 2. CRC-16/MODBUS over the payload ([`link::crc16::modbus`]),
/// 3. CBOR decode of the string-keyed payload (unknown keys skipped),
/// 4. validation: duplicate link names, `link:<name>` resolution to indices, known produce/consume
///    items, `node_id` range, and the BLE-must-be-async rule.
pub fn parse(blob: &[u8]) -> Result<BoardConfig, ConfigError> {
    // 1. Frame.
    if blob.len() < HEADER_LEN {
        return Err(ConfigError::BadMagic);
    }
    let magic = u32::from_le_bytes([blob[0], blob[1], blob[2], blob[3]]);
    if magic != BOARD_MAGIC {
        return Err(ConfigError::BadMagic);
    }
    let version = u16::from_le_bytes([blob[4], blob[5]]);
    if version != VERSION {
        return Err(ConfigError::BadVersion);
    }
    let length = u16::from_le_bytes([blob[6], blob[7]]) as usize;
    let crc = u16::from_le_bytes([blob[8], blob[9]]);

    let payload_end = HEADER_LEN.checked_add(length).ok_or(ConfigError::BadLength)?;
    if blob.len() < payload_end {
        return Err(ConfigError::BadLength);
    }
    let payload = &blob[HEADER_LEN..payload_end];

    // 2. CRC.
    if crc16::modbus(payload) != crc {
        return Err(ConfigError::CrcMismatch);
    }

    // 3. CBOR decode into the raw (name-carrying) structs.
    let raw = decode_payload(payload)?;

    // 4. Validate and resolve names to indices.
    validate(raw)
}

// --- raw decode (names still as strings) ------------------------------------------------------

/// A link source reference carried through decode before resolution: either a fixed source or a
/// list of `link:<name>` strings to resolve to indices.
#[derive(Debug, Clone)]
enum RawAttitude {
    LocalImu,
    /// A single `link:<name>`.
    Link(Name),
    /// A `fused(<names>)`: each is a bare link name.
    Fused(Vec<Name, MAX_FUSED>),
}

#[derive(Debug, Clone)]
enum RawDrive {
    LocalBalance,
    Link(Name),
}

/// A motor as decoded, before source resolution.
struct RawMotor {
    timer_label: Name,
    attitude: RawAttitude,
    drive: RawDrive,
    phase_labels: Vec<Name, 6>,
    hall_labels: Vec<Name, 3>,
    current_sense_label: Option<Name>,
}

/// The decoded payload, links still carrying names and motors still carrying raw sources.
struct RawConfig {
    format: u8,
    name: Name,
    control_mode: ControlMode,
    speed_sync_peer_name: Option<Name>,
    links: Vec<LinkConfig, MAX_LINKS>,
    motors: Vec<RawMotor, MAX_MOTORS>,
    battery_cells: u8,
    limits: Limits,
}

fn decode_payload(payload: &[u8]) -> Result<RawConfig, ConfigError> {
    let mut d = Decoder::new(payload);

    let mut format: u8 = 0;
    let mut name: Name = Name::new();
    let mut control_mode = ControlMode::Articulated;
    let mut speed_sync_peer_name: Option<Name> = None;
    let mut links: Vec<LinkConfig, MAX_LINKS> = Vec::new();
    let mut motors: Vec<RawMotor, MAX_MOTORS> = Vec::new();
    let mut battery_cells: u8 = 0;
    let mut limits = Limits::default();

    let n = map_len(&mut d)?;
    for _ in 0..n {
        let key = d.str().map_err(cbor)?;
        match key {
            "format" => format = read_u8(&mut d)?,
            "name" => name = read_name(&mut d)?,
            "control" => control_mode = decode_control(&mut d)?,
            "speed_sync" => speed_sync_peer_name = decode_speed_sync(&mut d)?,
            "links" => links = decode_links(&mut d)?,
            "motors" => motors = decode_motors(&mut d)?,
            "battery" => battery_cells = decode_battery(&mut d)?,
            "limits" => limits = decode_limits(&mut d)?,
            // Unknown keys are skipped for forward compatibility (not errored).
            _ => d.skip().map_err(cbor)?,
        }
    }

    Ok(RawConfig {
        format,
        name,
        control_mode,
        speed_sync_peer_name,
        links,
        motors,
        battery_cells,
        limits,
    })
}

/// Decode `{ "mode": "articulated" | "rigid" }`, skipping unknown keys.
fn decode_control(d: &mut Decoder) -> Result<ControlMode, ConfigError> {
    let mut mode = ControlMode::Articulated;
    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "mode" => {
                mode = match d.str().map_err(cbor)? {
                    "articulated" => ControlMode::Articulated,
                    "rigid" => ControlMode::Rigid,
                    _ => return Err(ConfigError::Cbor),
                }
            }
            _ => d.skip().map_err(cbor)?,
        }
    }
    Ok(mode)
}

/// Decode `{ "peer": "link:<name>" }` into the raw peer name (the `link:` prefix stripped). Returns
/// `None` if no `peer` key was present.
fn decode_speed_sync(d: &mut Decoder) -> Result<Option<Name>, ConfigError> {
    let mut peer: Option<Name> = None;
    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "peer" => {
                let s = d.str().map_err(cbor)?;
                peer = Some(strip_link_ref(s)?);
            }
            _ => d.skip().map_err(cbor)?,
        }
    }
    Ok(peer)
}

/// Decode `{ "cells": uint }`, skipping unknown keys.
fn decode_battery(d: &mut Decoder) -> Result<u8, ConfigError> {
    let mut cells: u8 = 0;
    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "cells" => cells = read_u8(d)?,
            _ => d.skip().map_err(cbor)?,
        }
    }
    Ok(cells)
}

/// Decode `{ "current_ma": uint, "speed_max": uint }`, skipping unknown keys.
fn decode_limits(d: &mut Decoder) -> Result<Limits, ConfigError> {
    let mut limits = Limits::default();
    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "current_ma" => limits.current_ma = d.u32().map_err(cbor)?,
            "speed_max" => limits.speed_max = d.u32().map_err(cbor)?,
            _ => d.skip().map_err(cbor)?,
        }
    }
    Ok(limits)
}

/// Decode the `"links"` array into the bounded link vector.
fn decode_links(d: &mut Decoder) -> Result<Vec<LinkConfig, MAX_LINKS>, ConfigError> {
    let mut out: Vec<LinkConfig, MAX_LINKS> = Vec::new();
    let n = array_len(d)?;
    for _ in 0..n {
        let link = decode_one_link(d)?;
        out.push(link).map_err(|_| ConfigError::TooMany)?;
    }
    Ok(out)
}

/// Decode one link map.
fn decode_one_link(d: &mut Decoder) -> Result<LinkConfig, ConfigError> {
    let mut name: Name = Name::new();
    let mut usart_label: Name = Name::new();
    let mut baud: u32 = 0;
    let mut node_id: u8 = 0;
    let mut arbitration = Arbitration::Follower;
    let mut produce = ItemSet::empty();
    let mut consume = ItemSet::empty();
    let mut device_name: Option<Name> = None;

    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "name" => name = read_name(d)?,
            "usart" => usart_label = read_name(d)?,
            "baud" => baud = d.u32().map_err(cbor)?,
            "node_id" => node_id = read_u8(d)?,
            "arbitration" => arbitration = decode_arbitration(d)?,
            "produce" => produce = decode_item_list(d)?,
            "consume" => consume = decode_item_list(d)?,
            "device_name" => device_name = Some(read_name(d)?),
            _ => d.skip().map_err(cbor)?,
        }
    }

    Ok(LinkConfig {
        name,
        usart_label,
        baud,
        node_id,
        arbitration,
        produce,
        consume,
        device_name,
    })
}

/// Decode an `"arbitration"` string into the enum.
fn decode_arbitration(d: &mut Decoder) -> Result<Arbitration, ConfigError> {
    Ok(match d.str().map_err(cbor)? {
        "initiator" => Arbitration::Initiator,
        "follower" => Arbitration::Follower,
        "bus_master" => Arbitration::BusMaster,
        "async" => Arbitration::Async,
        _ => return Err(ConfigError::Cbor),
    })
}

/// Decode a `produce` / `consume` array of item-name strings into an [`ItemSet`]. An unrecognized
/// item name is [`ConfigError::UnknownItem`].
fn decode_item_list(d: &mut Decoder) -> Result<ItemSet, ConfigError> {
    let mut set = ItemSet::empty();
    let n = array_len(d)?;
    for _ in 0..n {
        let item = item_from_name(d.str().map_err(cbor)?)?;
        set.insert(item);
    }
    Ok(set)
}

/// Map a wire item-name string to a [`DataItem`].
///
/// The names are those in `board-config.md`. `"status"` maps to [`DataItem::Status`], the cyclic-state
/// status byte (distinct from `"fault"`).
fn item_from_name(s: &str) -> Result<DataItem, ConfigError> {
    Ok(match s {
        "attitude" => DataItem::Attitude,
        "wheel_speed" => DataItem::WheelSpeed,
        "drive_cmd" => DataItem::DriveCmd,
        "inputs" => DataItem::Inputs,
        "telemetry" => DataItem::Telemetry,
        "fault" => DataItem::Fault,
        "status" => DataItem::Status,
        _ => return Err(ConfigError::UnknownItem),
    })
}

/// Decode the `"motors"` array into raw motors (sources unresolved).
fn decode_motors(d: &mut Decoder) -> Result<Vec<RawMotor, MAX_MOTORS>, ConfigError> {
    let mut out: Vec<RawMotor, MAX_MOTORS> = Vec::new();
    let n = array_len(d)?;
    for _ in 0..n {
        let m = decode_one_motor(d)?;
        out.push(m).map_err(|_| ConfigError::TooMany)?;
    }
    Ok(out)
}

/// Decode one motor map into a [`RawMotor`].
fn decode_one_motor(d: &mut Decoder) -> Result<RawMotor, ConfigError> {
    let mut timer_label: Name = Name::new();
    let mut attitude = RawAttitude::LocalImu;
    let mut drive = RawDrive::LocalBalance;
    let mut phase_labels: Vec<Name, 6> = Vec::new();
    let mut hall_labels: Vec<Name, 3> = Vec::new();
    let mut current_sense_label: Option<Name> = None;

    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "timer" => timer_label = read_name(d)?,
            "attitude_source" => attitude = decode_attitude_source(d)?,
            "drive_source" => drive = decode_drive_source(d)?,
            "phases" => phase_labels = decode_phases(d)?,
            "halls" => hall_labels = decode_halls(d)?,
            "current_sense" => current_sense_label = Some(decode_current_sense(d)?),
            _ => d.skip().map_err(cbor)?,
        }
    }

    Ok(RawMotor {
        timer_label,
        attitude,
        drive,
        phase_labels,
        hall_labels,
        current_sense_label,
    })
}

/// Decode an `attitude_source`: either the string `"local_imu"`, a `"link:<name>"` string, or a
/// map `{ "fused": [<names>] }` (each entry a bare or `link:`-prefixed name).
fn decode_attitude_source(d: &mut Decoder) -> Result<RawAttitude, ConfigError> {
    match d.datatype().map_err(cbor)? {
        Type::String | Type::StringIndef => {
            let s = d.str().map_err(cbor)?;
            if s == "local_imu" {
                Ok(RawAttitude::LocalImu)
            } else {
                Ok(RawAttitude::Link(strip_link_ref(s)?))
            }
        }
        Type::Map | Type::MapIndef => {
            let mut names: Vec<Name, MAX_FUSED> = Vec::new();
            let n = map_len(d)?;
            for _ in 0..n {
                match d.str().map_err(cbor)? {
                    "fused" => {
                        let m = array_len(d)?;
                        for _ in 0..m {
                            let nm = strip_link_ref(d.str().map_err(cbor)?)?;
                            names.push(nm).map_err(|_| ConfigError::TooMany)?;
                        }
                    }
                    _ => d.skip().map_err(cbor)?,
                }
            }
            Ok(RawAttitude::Fused(names))
        }
        _ => Err(ConfigError::Cbor),
    }
}

/// Decode a `drive_source`: either `"local_balance"` or a `"link:<name>"` string.
fn decode_drive_source(d: &mut Decoder) -> Result<RawDrive, ConfigError> {
    let s = d.str().map_err(cbor)?;
    if s == "local_balance" {
        Ok(RawDrive::LocalBalance)
    } else {
        Ok(RawDrive::Link(strip_link_ref(s)?))
    }
}

/// Decode the `"phases"` map `{ "a": {hi,lo}, "b": {..}, "c": {..} }` into a flat label list
/// `["a.hi","a.lo","b.hi","b.lo","c.hi","c.lo"]` (opaque labels; pin resolution is runtime-hal's
/// job). Each leg's pins are carried verbatim.
fn decode_phases(d: &mut Decoder) -> Result<Vec<Name, 6>, ConfigError> {
    let mut out: Vec<Name, 6> = Vec::new();
    let n = map_len(d)?;
    for _ in 0..n {
        // The leg key ("a"/"b"/"c") is carried as a label prefix so the order is preserved.
        let _leg = d.str().map_err(cbor)?;
        // Each leg value is a map { "hi": pin, "lo": pin }.
        let m = map_len(d)?;
        for _ in 0..m {
            let _side = d.str().map_err(cbor)?; // "hi" / "lo"
            let pin = read_name(d)?;
            out.push(pin).map_err(|_| ConfigError::TooMany)?;
        }
    }
    Ok(out)
}

/// Decode the `"halls"` array of pin labels.
fn decode_halls(d: &mut Decoder) -> Result<Vec<Name, 3>, ConfigError> {
    let mut out: Vec<Name, 3> = Vec::new();
    let n = array_len(d)?;
    for _ in 0..n {
        let pin = read_name(d)?;
        out.push(pin).map_err(|_| ConfigError::TooMany)?;
    }
    Ok(out)
}

/// Decode the `"current_sense"` map `{ "pin": <pin>, "adc": <label>, "channel": uint }` into the
/// pin label (opaque; the adc/channel are skipped here, runtime-hal owns resolution).
fn decode_current_sense(d: &mut Decoder) -> Result<Name, ConfigError> {
    let mut pin: Name = Name::new();
    let n = map_len(d)?;
    for _ in 0..n {
        match d.str().map_err(cbor)? {
            "pin" => pin = read_name(d)?,
            _ => d.skip().map_err(cbor)?,
        }
    }
    Ok(pin)
}

// --- validation and name -> index resolution --------------------------------------------------

/// Validate the decoded config and resolve all `link:<name>` references to declared-link indices.
fn validate(raw: RawConfig) -> Result<BoardConfig, ConfigError> {
    // Duplicate link names.
    for i in 0..raw.links.len() {
        for j in (i + 1)..raw.links.len() {
            if raw.links[i].name == raw.links[j].name {
                return Err(ConfigError::DuplicateLinkName);
            }
        }
    }

    // node_id range and BLE-must-be-async.
    for link in raw.links.iter() {
        if link.node_id == 0xFF {
            return Err(ConfigError::BadNodeId);
        }
        if link.device_name.is_some() && link.arbitration != Arbitration::Async {
            return Err(ConfigError::BleNotAsync);
        }
    }

    // Resolve a link name to its index.
    let resolve = |name: &Name| -> Result<u8, ConfigError> {
        for (idx, link) in raw.links.iter().enumerate() {
            if &link.name == name {
                return Ok(idx as u8);
            }
        }
        Err(ConfigError::UnresolvedLinkRef)
    };

    // speed_sync.peer.
    let speed_sync_peer = match &raw.speed_sync_peer_name {
        Some(name) => Some(resolve(name)?),
        None => None,
    };

    // Per-motor sources.
    let mut motors: Vec<MotorConfig, MAX_MOTORS> = Vec::new();
    for m in raw.motors.iter() {
        let attitude_source = match &m.attitude {
            RawAttitude::LocalImu => AttitudeSource::LocalImu,
            RawAttitude::Link(name) => AttitudeSource::Link(resolve(name)?),
            RawAttitude::Fused(names) => {
                let mut idxs: Vec<u8, MAX_FUSED> = Vec::new();
                for name in names.iter() {
                    idxs.push(resolve(name)?).map_err(|_| ConfigError::TooMany)?;
                }
                AttitudeSource::Fused(idxs)
            }
        };
        let drive_source = match &m.drive {
            RawDrive::LocalBalance => DriveSource::LocalBalance,
            RawDrive::Link(name) => DriveSource::Link(resolve(name)?),
        };
        motors
            .push(MotorConfig {
                timer_label: m.timer_label.clone(),
                attitude_source,
                drive_source,
                phase_labels: m.phase_labels.clone(),
                hall_labels: m.hall_labels.clone(),
                current_sense_label: m.current_sense_label.clone(),
            })
            .map_err(|_| ConfigError::TooMany)?;
    }

    Ok(BoardConfig {
        format: raw.format,
        name: raw.name,
        control_mode: raw.control_mode,
        speed_sync_peer,
        links: raw.links,
        motors,
        battery_cells: raw.battery_cells,
        limits: raw.limits,
    })
}

// --- small decode helpers ---------------------------------------------------------------------

/// Map a minicbor decode error onto [`ConfigError::Cbor`].
#[inline]
fn cbor(_e: minicbor::decode::Error) -> ConfigError {
    ConfigError::Cbor
}

/// Read a definite-length map header, returning the pair count.
#[inline]
fn map_len(d: &mut Decoder) -> Result<u64, ConfigError> {
    d.map().map_err(cbor)?.ok_or(ConfigError::Cbor)
}

/// Read a definite-length array header, returning the element count.
#[inline]
fn array_len(d: &mut Decoder) -> Result<u64, ConfigError> {
    d.array().map_err(cbor)?.ok_or(ConfigError::Cbor)
}

/// Read a CBOR unsigned integer that must fit in a `u8`.
#[inline]
fn read_u8(d: &mut Decoder) -> Result<u8, ConfigError> {
    match d.datatype().map_err(cbor)? {
        Type::U8 | Type::U16 | Type::U32 | Type::U64 | Type::Int => d.u8().map_err(cbor),
        _ => Err(ConfigError::Cbor),
    }
}

/// Read a CBOR text string into a bounded [`Name`]. Too long is [`ConfigError::TooMany`].
#[inline]
fn read_name(d: &mut Decoder) -> Result<Name, ConfigError> {
    let s = d.str().map_err(cbor)?;
    Name::try_from(s).map_err(|_| ConfigError::TooMany)
}

/// Strip a `link:` prefix from a reference string, returning the bare link name. A reference without
/// the prefix is taken verbatim (so `"peer"` and `"link:peer"` both resolve to link `peer`).
#[inline]
fn strip_link_ref(s: &str) -> Result<Name, ConfigError> {
    let bare = s.strip_prefix("link:").unwrap_or(s);
    Name::try_from(bare).map_err(|_| ConfigError::TooMany)
}

#[cfg(test)]
mod tests;
