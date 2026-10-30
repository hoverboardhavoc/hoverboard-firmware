//! The controller-driven discovery + address-assignment walk (`specs/l3.md`, "Discovery and address
//! assignment").
//!
//! Two halves:
//! - [`Responder`] - the **board side**: a passive responder that answers `NODE_HELLO` with its
//!   identity, probes its own ports on `PROBE_PORTS` and reports `PORTS`, and takes an `ASSIGN`
//!   (persisting `node_address` to the `store`) or relays a directed `ASSIGN` out an egress port with
//!   `dst` rewritten to `0x00`. It never self-assigns and never reads a hardware id - identity is
//!   positional during discovery, the persisted address after.
//! - [`Controller`] - the **host side** (transient controller): holds the address map and drives the
//!   whole process, working outward to the leaves (first contact -> assign -> probe -> assign the
//!   frontier -> recurse).
//!
//! Both are HAL-free and link-agnostic: they consume and produce whole (L2-reassembled) PDUs by port
//! index. The firmware (and the Tier-1 harness) does the L2 `send`/`poll_recv` around them. The board
//! side wraps a [`Forwarder`] so source-learning + multi-hop forwarding happen on the same path.
//!
//! Two integration seams (`specs/integration.md`):
//! - **The delivered-PDU hand-back**: [`Responder::ingest`] returns a delivered-but-unhandled PDU
//!   ([`DeliveredPdu`]) to the caller by bounded copy instead of dropping it, so the firmware can
//!   route the reserved control block `0x10..0x2F` through its own codec. `net` stays
//!   payload-agnostic: it never interprets those opcodes.
//! - **The armed write gate** (R4): [`Responder::set_armed`] is sampled by the caller each loop
//!   pass from the mode machine's `any_moe_allowed()`. While armed, every net-owned flash-persist
//!   path refuses: the private `apply_write` (the single choke point between `CONFIG_WRITE`/
//!   `_MULTI` and `store.set_value`) rejects with [`CFG_ARMED`], and an `ASSIGN` to self is
//!   refused whole (no persist, no in-RAM adopt) with a [`STATUS_ERR`] ACK. Reads are unaffected;
//!   the relay half of `ASSIGN` stays ungated (no flash on the relay).

use heapless::Vec;
use store::{
    DynError, Flash, Key, Store, StoreError, Stored, Type, Value, DEVICE_NAME, ERASED_TAG,
    NODE_ADDRESS, NODE_CLASS, SECRET, WIFI_PASS, WIFI_SSID,
};

use crate::forward::Forwarder;
use crate::pdu::{self, Opcode, Pdu, GUEST_FIRST, GUEST_LAST};

/// The most local ports any board in the fleet has (a 12-FET gateway: BLE + two sideboard UARTs, plus
/// headroom). Sizes the per-port arrays and the probe state.
pub const MAX_PORTS: usize = 4;
/// The largest walk/CONFIG PDU. A `PORTS` reply for [`MAX_PORTS`] ports is `3 + 1 + 4*4 = 20` bytes;
/// `CONFIG_*` values are small. 64 leaves headroom and stays one L2 fragment on the mock link.
pub const MAX_PDU: usize = 64;
/// The most emissions one ingest can produce. A `PROBE_PORTS` emits one probe per port ([`MAX_PORTS`]);
/// a flood emits at most `MAX_PORTS - 1` (split-horizon); a reply/forward emits one. So `MAX_PORTS` is
/// the bound. (Kept exactly that, not larger, for the firmware's tight 8 KiB-image RAM budget.)
pub const MAX_EMIT: usize = MAX_PORTS;

/// A bounded encoded-PDU buffer.
pub type PduBuf = Vec<u8, MAX_PDU>;

/// One thing to put on the wire: the egress port and the encoded PDU.
#[derive(Clone)]
pub struct Emission {
    /// The local port to send it out.
    pub port: u8,
    /// The encoded PDU bytes.
    pub bytes: PduBuf,
}

/// The bounded set of emissions one step produces.
pub type Emits = Vec<Emission, MAX_EMIT>;

// ---- wire constants (`specs/l3.md`, the opcode table) ----

/// `NODE_HELLO` request `kind`: a transient controller making first contact (grant it a guest address).
pub const KIND_CONTROLLER: u8 = 0x01;
/// `NODE_HELLO` request `kind`: a board probing a neighbour on the controller's behalf (no grant).
pub const KIND_PROBE: u8 = 0x02;

// The guest range itself is NOT declared here. It is the address space's own fact and lives with the
// rest of the addressing model in [`crate::pdu`] ([`pdu::GUEST_FIRST`] / [`pdu::GUEST_LAST`], beside
// `is_controller`, which is the predicate that has to agree with it). This module is a consumer: it
// decides which guest address to grant NEXT, which is the grant protocol's business.

/// `PORTS` neighbour state: nothing wired to this port.
pub const NB_EMPTY: u8 = 0;
/// `PORTS` neighbour state: a board with no address yet (`node_id == 0x00`).
pub const NB_UNASSIGNED: u8 = 1;
/// `PORTS` neighbour state: a board (or guest) that already has an address (`neighbour_addr`).
pub const NB_ASSIGNED: u8 = 2;
/// The bits of a `PORTS` `neighbour_state` byte that carry the state (`specs/l3.md`, "Tier-1 wire
/// encoding"). Bit 7 is reserved for "member of the node's link group", which this firmware never
/// sets; every decoder masks with this so a reply that does set it still classifies the port.
pub const NB_STATE_MASK: u8 = 0x03;

/// `PORTS` port medium tag (the minimal default; `specs/l3.md` open question): a UART.
pub const PORT_UART: u8 = 0;
/// `PORTS` port medium tag: the BLE link.
pub const PORT_BLE: u8 = 1;
/// `PORTS` port medium tag: an SWD mailbox.
pub const PORT_SWD: u8 = 2;

/// `ASSIGN` `egress_port` meaning "the addressed board itself" (assign directly, not via a relay).
pub const EGRESS_SELF: u8 = 0xFF;

/// `ASSIGN_ACK` / `CONFIG_RESP` status: success.
pub const STATUS_OK: u8 = 0;
/// `ASSIGN_ACK` / `CONFIG_RESP` status: failure (e.g. a flash error).
pub const STATUS_ERR: u8 = 1;

/// The L3 protocol version a `NODE_HELLO` reply reports.
pub const PROTO_VER: u8 = 1;

/// `NODE_HELLO` reply `mcu`: the chip family, for presets and pin routing, never the node's class
/// (`specs/l3.md`, "Node class"). A GD32F10x board.
pub const MCU_F10X: u8 = 1;
/// `NODE_HELLO` reply `mcu`: a GD32F1x0 board.
pub const MCU_F1X0: u8 = 2;
/// `NODE_HELLO` reply `mcu`: the hoverboard bridge (`specs/hoverboard-bridge.md`, "Ports").
pub const MCU_ESP32C3: u8 = 3;

/// `node.class` of a bridge, the hoverboard bridge (`specs/l3.md`, "Node class"). A motor
/// controller's class is the registry default of `store::NODE_CLASS`, which is where the field's
/// model keeps it; the bridge's value has no registry row of its own, so it lives here, beside the
/// `Responder` that answers with it.
pub const CLASS_BRIDGE: u8 = 0x02;

/// The longest `device.name` a bridge takes: what fits its BLE advertising packet beside the flags
/// (31 B less 3 for the flags and 2 for the name's own header).
pub const BRIDGE_NAME_MAX: usize = 26;

/// A `device.name` a bridge can advertise over BLE and answer mDNS for: 1 to [`BRIDGE_NAME_MAX`]
/// bytes of ASCII letters, digits and `-`. A bridge's `CONFIG_WRITE` of any other name is refused
/// with [`CFG_BAD`], and a bridge whose store already holds one advertises its default name.
pub fn bridge_name_usable(name: &[u8]) -> bool {
    !name.is_empty()
        && name.len() <= BRIDGE_NAME_MAX
        && name.iter().all(|b| b.is_ascii_alphanumeric() || *b == b'-')
}

/// The longest `wifi.ssid` a bridge takes: what an 802.11 SSID holds.
pub const WIFI_SSID_MAX: usize = 32;
/// The shortest non-empty `wifi.pass` a bridge takes: a WPA2 passphrase's minimum.
pub const WIFI_PASS_MIN: usize = 8;
/// The longest `wifi.pass` a bridge takes: a WPA2 passphrase's maximum (a 64-digit hex PSK is not
/// accepted). One `CONFIG_WRITE` carries at most [`CONFIG_WRITE_VALUE_MAX`] value bytes, so the
/// longer ones cannot be written over a link; the bound matters to a store written by an image that
/// had no check.
pub const WIFI_PASS_MAX: usize = 63;

/// `CONFIG_WRITE`'s payload before the value: `[field_id, index, type_tag]`.
const CONFIG_WRITE_HEAD: usize = 3;
/// `CONFIG_RESP`'s payload before the value: `[field_id, index, status, type_tag]`.
const CONFIG_RESP_HEAD: usize = 4;
/// The most value bytes one `CONFIG_WRITE` carries: [`MAX_PDU`] less the PDU header and
/// `[field_id, index, type_tag]`.
pub const CONFIG_WRITE_VALUE_MAX: usize = MAX_PDU - pdu::HEADER_LEN - CONFIG_WRITE_HEAD;
/// The most value bytes one `CONFIG_RESP` carries: [`MAX_PDU`] less the PDU header and
/// `[field_id, index, status, type_tag]`. A write's echo and a read of the key carry the value, so
/// a longer value is stored but never answered; a secret's echo and read carry none
/// ([`CFG_SECRET`]), so it is bound by [`CONFIG_WRITE_VALUE_MAX`] alone.
pub const CONFIG_RESP_VALUE_MAX: usize = MAX_PDU - pdu::HEADER_LEN - CONFIG_RESP_HEAD;

/// Whether a bridge can use `value` for `key` (`specs/bridge-onboarding.md` section 2.1): a
/// `device.name` that passes [`bridge_name_usable`] (ASCII, so UTF-8), a `wifi.ssid` of at most
/// [`WIFI_SSID_MAX`] bytes, a `wifi.pass` that is empty or [`WIFI_PASS_MIN`] to [`WIFI_PASS_MAX`]
/// bytes, the two Wi-Fi fields UTF-8, which the radio's station configuration takes. Any other key
/// or value is the store's to judge. A bridge's `CONFIG_WRITE` of a value this refuses answers
/// [`CFG_BAD`]; a board stores any of them.
pub fn bridge_value_usable(key: Key, value: &Value) -> bool {
    let Value::Str(v) = value else {
        return true;
    };
    let text = || core::str::from_utf8(v).is_ok();
    if key == DEVICE_NAME.key() {
        bridge_name_usable(v)
    } else if key.field_id == WIFI_SSID.id() {
        v.len() <= WIFI_SSID_MAX && text()
    } else if key.field_id == WIFI_PASS.id() {
        (v.is_empty() || (WIFI_PASS_MIN..=WIFI_PASS_MAX).contains(&v.len())) && text()
    } else {
        true
    }
}

/// `CONFIG_RESP` status: success.
pub const CFG_OK: u8 = 0;
/// `CONFIG_RESP` status: a malformed request (unknown type tag, bad value bytes, truncated PDU), or
/// a value the node cannot use (a bridge's value that fails [`bridge_value_usable`]).
pub const CFG_BAD: u8 = 1;
/// `CONFIG_RESP` status: no field declares this `field_id`.
pub const CFG_UNKNOWN_KEY: u8 = 2;
/// `CONFIG_RESP` status: the value's type did not match the field's registered type.
pub const CFG_TYPE_MISMATCH: u8 = 3;
/// `CONFIG_RESP` status: the store write failed (flash full / error).
pub const CFG_STORE_ERR: u8 = 4;
/// `TUNE_WRITE`: set one live-tunable value in RAM and reply `CONFIG_RESP`
/// (`specs/rider-ui.md` section 4, the live tune lane). Payload is `CONFIG_WRITE`'s
/// `[field_id, index, type_tag, value...]`.
///
/// It is declared here, beside the `CFG_*` statuses it answers with, but it is NOT handled here:
/// the values it writes live in the control loop's RAM, not in the store, so the walk layer hands
/// the PDU back and the firmware applies it (the delivered-PDU hand-back). What that buys is the
/// whole point of the lane: a tune write touches NO flash, so it is exempt from the armed refusal
/// BY CONSTRUCTION rather than by an exception in [`Responder::set_armed`]'s check, and the
/// storage-layer rule "no flash program or erase while armed, no exceptions" stays literally true.
/// Persisting a tuned value is a separate, ordinary, disarmed `CONFIG_WRITE`.
pub const OP_TUNE_WRITE: u8 = 0x34;
/// `TUNE_READ`: read one live-tunable value back out of RAM, replying `CONFIG_RESP`. Payload is
/// `CONFIG_READ`'s `[field_id, index]`. See [`OP_TUNE_WRITE`] for why the lane exists and why the
/// walk layer declares but does not handle it.
pub const OP_TUNE_READ: u8 = 0x35;

/// `CONFIG_RESP` status: the write was rejected because the board is armed (some motor's MOE is
/// allowed; `specs/integration.md`, R4). Reads are unaffected; retry after disarm.
pub const CFG_ARMED: u8 = 5;

/// `CONFIG_RESP` status: the field is read-only (`node.class`; `specs/l3.md`, "Node class"). The
/// store refuses the write ([`DynError::ReadOnly`]) and nothing is persisted.
pub const CFG_READ_ONLY: u8 = 6;

/// `CONFIG_RESP` status: the region is genuinely out of space. The board compacted and the value
/// still does not fit, so the live set alone fills a clean page. Nothing is wrong with the flash
/// and retrying will not help; something has to go.
pub const CFG_NO_SPACE: u8 = 7;

/// `CONFIG_RESP` status: the board compacted but could not COMMIT the packed page, so it could not
/// reclaim the space (`StoreError::CommitFailed`). No configuration was lost, but nothing more can
/// be written until the cause is found.
///
/// It exists because the two states above used to be one byte. On 2026-10-23 a board whose flash
/// refused the compaction's commit answered the same [`CFG_STORE_ERR`] a board with a genuinely
/// full region would, so a firmware defect looked like a board that was simply out of room
/// (`specs/bench-evidence/2026-10-23/store-compact-commit/`).
pub const CFG_COMMIT_FAILED: u8 = 8;

/// `CONFIG_RESP` status: a successful `CONFIG_READ` of a key with no record (`specs/cli-config.md`
/// section 6.1). The value that follows is the field's default and nothing is stored; a key with a
/// record answers [`CFG_OK`]. A client treats both as a successful read.
pub const CFG_DEFAULT: u8 = 9;

/// `CONFIG_RESP` status: a `CONFIG_READ` of a stored secret (`store::SECRET`,
/// `specs/bridge-onboarding.md` section 2.2). The key holds a value and no value follows: no node
/// reads a secret back over a link. An unstored secret answers [`CFG_DEFAULT`] with its default.
pub const CFG_SECRET: u8 = 10;

/// Map a store [`DynError`] to a `CONFIG_RESP` status byte.
fn cfg_status(e: DynError) -> u8 {
    match e {
        DynError::UnknownKey => CFG_UNKNOWN_KEY,
        DynError::TypeMismatch => CFG_TYPE_MISMATCH,
        DynError::ReadOnly => CFG_READ_ONLY,
        DynError::Store(_) => CFG_STORE_ERR,
    }
}

/// Map a failed `store.compact()` to a `CONFIG_RESP` status byte.
fn compact_status(e: StoreError) -> u8 {
    match e {
        StoreError::CommitFailed => CFG_COMMIT_FAILED,
        _ => CFG_STORE_ERR,
    }
}

/// Append `[type_tag, value-bytes...]` of `v` to a `CONFIG_RESP` payload.
fn push_value(out: &mut PduBuf, v: &Value) {
    let _ = out.push(v.kind().tag());
    let mut tmp = [0u8; MAX_PDU];
    let n = v.encode(&mut tmp);
    let _ = out.extend_from_slice(&tmp[..n]);
}

/// What a decoded TUNE PDU asks for ([`decode_tune`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TuneOp {
    /// `TUNE_READ`: report the live value.
    Read,
    /// `TUNE_WRITE`: set the live value.
    Write(i16),
    /// The request named a key but could not be honoured on the wire's own terms (a type tag that
    /// is not `I16`, or a truncated value). Reply this `CFG_*` status and touch nothing.
    Refuse(u8),
}

/// The longest `CONFIG_RESP` payload a tune reply can take: `[field_id, index, status, type_tag]`
/// plus the two value bytes.
const TUNE_RESP_MAX: usize = 6;

/// Decode a `TUNE_WRITE` / `TUNE_READ` payload into the key it names and what it asks for.
///
/// `None` when the opcode is not a tune opcode, or the payload is too short to name a key at all:
/// there is then nothing to reply ABOUT, and an unanswerable frame is dropped exactly as any other
/// malformed one is. Everything the wire itself can judge is judged here (the value's type tag and
/// width); whether the KEY is tunable and whether the VALUE is in range are the live shadow's to
/// say, because it owns the allowlist and the ranges.
pub fn decode_tune(opcode: u8, payload: &[u8]) -> Option<(Key, TuneOp)> {
    if payload.len() < 2 {
        return None;
    }
    let key = Key {
        field_id: payload[0],
        index: payload[1],
    };
    match opcode {
        OP_TUNE_READ => Some((key, TuneOp::Read)),
        OP_TUNE_WRITE => {
            let op = match payload.get(2).map(|t| Type::from_tag(*t)) {
                None => TuneOp::Refuse(CFG_BAD),
                Some(None) => TuneOp::Refuse(CFG_BAD),
                Some(Some(Type::I16)) => match Value::decode(Type::I16, &payload[3..]) {
                    Some(Value::I16(v)) => TuneOp::Write(v),
                    _ => TuneOp::Refuse(CFG_BAD),
                },
                Some(Some(_)) => TuneOp::Refuse(CFG_TYPE_MISMATCH),
            };
            Some((key, op))
        }
        _ => None,
    }
}

/// Build the `CONFIG_RESP` payload for a tune reply into `buf`, returning its length: the echoed
/// key, the status, and, when the operation succeeded, the `I16` value now live. Emitted by
/// [`Responder::reply_tune`].
///
/// The same shape a `CONFIG_*` reply takes, deliberately: a controller decodes both with one
/// parser, and the only difference between the two lanes is which memory answered.
fn tune_resp(buf: &mut [u8; TUNE_RESP_MAX], key: Key, status: u8, value: Option<i16>) -> usize {
    buf[0] = key.field_id;
    buf[1] = key.index;
    buf[2] = status;
    match value {
        Some(v) => {
            buf[3] = Type::I16.tag();
            buf[4..6].copy_from_slice(&v.to_le_bytes());
            6
        }
        // No type tag on a refusal, the `CONFIG_RESP` error convention (`on_config_read`).
        None => {
            buf[3] = 0;
            4
        }
    }
}

/// Encode `pdu` and push it as an emission out `port` (best-effort; an over-long PDU is dropped).
fn emit(out: &mut Emits, port: u8, pdu: &Pdu) {
    let mut tmp = [0u8; MAX_PDU];
    if let Ok(n) = pdu.encode(&mut tmp) {
        let mut b: PduBuf = Vec::new();
        if b.extend_from_slice(&tmp[..n]).is_ok() {
            let _ = out.push(Emission { port, bytes: b });
        }
    }
}

/// A delivered-but-unhandled PDU handed back to the caller by [`Responder::ingest`] (the
/// delivered-PDU hand-back, `specs/integration.md`): a PDU the forwarder delivered to this node
/// whose opcode the walk layer does not handle (the reserved `0x10..0x2F` control block, the
/// telemetry block, or a stray controller-bound walk opcode). A bounded copy: the payload is at
/// most [`MAX_PDU`] bytes (in practice `MAX_PDU - 3`, the header's share of the frame). The
/// firmware routes the control block through `linkctl::decode` and drops everything else; `net`
/// never interprets any of it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeliveredPdu {
    /// The raw opcode byte.
    pub opcode: u8,
    /// The source node address.
    pub src: u8,
    /// The payload bytes, copied out of the delivered frame.
    pub payload: PduBuf,
}

impl DeliveredPdu {
    /// Copy `pdu`'s hand-back fields. `None` if the payload exceeds the [`MAX_PDU`] bound (cannot
    /// happen for a PDU that already fit a decoded frame; the drop is the best-effort class).
    fn from_pdu(pdu: &Pdu) -> Option<DeliveredPdu> {
        let mut payload: PduBuf = Vec::new();
        payload.extend_from_slice(pdu.payload).ok()?;
        Some(DeliveredPdu {
            opcode: pdu.opcode,
            src: pdu.src,
            payload,
        })
    }
}

/// The width of one node's guest slice (`specs/l3.md`, "Addressing").
const GUEST_SLICE: u8 = 8;

/// The guest slice a node at `addr` grants from, as `(first, len)`: the 8 addresses at
/// `0x80 + 8 * (addr & 0x0F)`, so two controllers attached at two different nodes hold different
/// guest addresses (`specs/l3.md`, "Addressing"). Slice 15 stops at [`GUEST_LAST`] (`0xF8..=0xFE`,
/// seven addresses), because `0xFF` is broadcast. An unassigned node (`0x00`) grants from slice 0.
/// Every address in a slice is inside [`GUEST_FIRST`]`..=`[`GUEST_LAST`].
fn guest_slice(addr: u8) -> (u8, u8) {
    let first = GUEST_FIRST + GUEST_SLICE * (addr & 0x0F);
    (first, (GUEST_LAST - first + 1).min(GUEST_SLICE))
}

/// One board's in-flight `PROBE_PORTS`: who to reply `PORTS` to, and the per-port neighbour states
/// gathered from the probe replies (initialised to `empty`; a reply upgrades a slot).
struct Probe {
    reply_to: u8,
    n_ports: u8,
    states: [(u8, u8); MAX_PORTS], // (neighbour_state, neighbour_addr)
}

/// The board-side walk responder. Wraps a [`Forwarder`] (so learning + forwarding run on the same
/// path) and adds the walk opcode handling. `no_std`, persists `node_address` through the `store`.
pub struct Responder {
    fwd: Forwarder,
    port_kinds: [u8; MAX_PORTS],
    fw_ver: u16,
    mcu: u8,
    /// What this node is, answered for a `CONFIG_READ` of `node.class` (`specs/l3.md`, "Node
    /// class"): the node's own fact, never stored, so the read-only registry row only refuses
    /// writes.
    class: u8,
    /// The node's own default `device.name`, answered for a `CONFIG_READ` of an unstored
    /// `device.name` in place of the registry's (`specs/cli-config.md` section 6.2): the bridge's
    /// MAC-based name, which it never writes to its store. `None` on a board, which answers the
    /// registry default.
    default_name: Option<&'static [u8]>,
    /// The next grant's offset inside this node's guest slice ([`guest_slice`]). The slice itself
    /// is derived from the live address at each grant, so an `ASSIGN` or a boot-time restore moves
    /// the allocator to the new slice with no further state.
    guest_slot: u8,
    probe: Option<Probe>,
    /// The R4 armed flag: while set, `apply_write` rejects with [`CFG_ARMED`] and `ASSIGN`-to-self
    /// refuses with [`STATUS_ERR`]. Sampled each loop
    /// pass by the caller from the mode machine's `any_moe_allowed()` via [`Responder::set_armed`].
    armed: bool,
    /// Set when a `CONFIG_WRITE` / `_MULTI` entry has PERSISTED since the caller last asked
    /// ([`Responder::take_config_persisted`]). A fact about this responder's own action, reported
    /// so a caller holding a RAM shadow of a persisted field can re-read it; the walk layer knows
    /// nothing about which fields those are.
    config_persisted: bool,
}

impl Responder {
    /// A fresh, unconfigured node (`addr = 0x00`) with `n_ports` ports of the given medium kinds,
    /// reporting `mcu` in `NODE_HELLO` and `class` for `node.class` (a board passes
    /// `store::NODE_CLASS`'s default, the bridge [`CLASS_BRIDGE`]).
    pub fn new(n_ports: u8, port_kinds: [u8; MAX_PORTS], mcu: u8, class: u8, fw_ver: u16) -> Self {
        // The per-port arrays (`port_kinds`, the probe `states`) are sized to MAX_PORTS; a board
        // declaring more would index out of bounds in `poll_probe`. The fleet max is 4, so this never
        // fires in practice - it guards a future part with more ports.
        debug_assert!(n_ports as usize <= MAX_PORTS, "n_ports exceeds MAX_PORTS");
        Responder {
            fwd: Forwarder::new(pdu::NO_ADDRESS, n_ports),
            port_kinds,
            fw_ver,
            mcu,
            class,
            default_name: None,
            guest_slot: 0,
            probe: None,
            armed: false,
            config_persisted: false,
        }
    }

    /// Set the R4 armed flag (`specs/integration.md`, "R4: the armed config-write gate"). The
    /// caller samples it each loop pass from `mode.any_moe_allowed()`; while armed, nothing
    /// persists: config writes (`CONFIG_WRITE` / `CONFIG_WRITE_MULTI`) are rejected with
    /// [`CFG_ARMED`], and an `ASSIGN` to self is refused with a [`STATUS_ERR`] ACK (no persist,
    /// no in-RAM adopt). Reads are unaffected.
    pub fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
    }

    /// Answer `name` for a read of an unstored `device.name`, with `CFG_DEFAULT`: a node whose
    /// default name is its own rather than the registry's (the bridge, `specs/hoverboard-bridge.md`,
    /// "Its store").
    pub fn set_default_name(&mut self, name: &'static [u8]) {
        self.default_name = Some(name);
    }

    /// What a `CONFIG_READ` of `key` answers: the value and the status, [`CFG_OK`] when a record
    /// holds the value, [`CFG_DEFAULT`] when none does, and [`CFG_SECRET`] for a stored secret, whose
    /// value no reply carries. `node.class` comes from the node, an unstored `device.name` from the
    /// node's own default when it has one, everything else from the store.
    fn read_value<'s, F: Flash>(
        &self,
        store: &'s Store<F>,
        key: Key,
    ) -> Result<(Value<'s>, u8), DynError> {
        if key == NODE_CLASS.key() {
            return Ok((Value::U8(self.class), CFG_OK));
        }
        let (value, stored) = store.get_value(key)?;
        Ok(match (stored, self.default_name) {
            (Stored::Yes, _) if SECRET.contains(&key.field_id) => (value, CFG_SECRET),
            (Stored::Yes, _) => (value, CFG_OK),
            (Stored::No, Some(name)) if key == DEVICE_NAME.key() => (Value::Str(name), CFG_DEFAULT),
            (Stored::No, _) => (value, CFG_DEFAULT),
        })
    }

    /// Whether a `CONFIG_WRITE` / `_MULTI` entry has persisted since this was last called, and
    /// clear it (`specs/rider-ui.md` section 4: the live gain shadow re-reads the store on this
    /// signal, so a Save converges without a reboot).
    ///
    /// A LEVEL rather than a key: `_MULTI` can persist several fields in one PDU, and a single-key
    /// slot would report only one of them. The caller re-reads the fields it shadows and takes the
    /// ones whose stored value actually moved, which is exact under any write pattern and cannot
    /// revert a live tune that no flash write touched.
    pub fn take_config_persisted(&mut self) -> bool {
        core::mem::take(&mut self.config_persisted)
    }

    /// The current R4 armed flag.
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// This board's current L3 address (`0x00` until assigned).
    pub fn addr(&self) -> u8 {
        self.fwd.addr()
    }

    /// The forwarder (for routing-table inspection in tests).
    pub fn forwarder(&self) -> &Forwarder {
        &self.fwd
    }

    /// Restore the persisted `node_address` from flash into the live address (a reboot: the board
    /// re-reads its address and resumes; `specs/l3.md`, "Recovery after a reboot").
    pub fn restore_addr<F: Flash>(&mut self, store: &Store<F>) {
        if let Ok((Value::U8(a), _)) = store.get_value(NODE_ADDRESS.key()) {
            self.fwd.set_addr(a);
        }
    }

    /// Is a `PROBE_PORTS` in flight (awaiting its probe replies / a tick to report)?
    pub fn probing(&self) -> bool {
        self.probe.is_some()
    }

    /// A reboot (`specs/l3.md`, "Recovery after a reboot"): the routing table is soft state, so it is
    /// **cleared** and re-learned from traffic; the `node_address` is **re-read from flash** (it
    /// persists). The board then resumes its cyclic peer traffic, which re-populates every table.
    pub fn reboot<F: Flash>(&mut self, store: &Store<F>) {
        self.fwd.clear_routes();
        self.restore_addr(store);
    }

    /// Originate a frame from this board (its resumed cyclic peer traffic, or a board-to-board control
    /// frame): `src =` this board's address, routed by the learned table (flood if `dst` is unknown).
    /// `opcode` is any valid opcode (an L7 `DRIVE` is forwarded best-effort, never interpreted here).
    pub fn originate(&self, opcode: u8, dst: u8, payload: &[u8], out: &mut Emits) {
        if let Ok(pdu) = Pdu::new(opcode, self.addr(), dst, payload) {
            self.fwd.originate(&pdu, &mut |port, f| emit(out, port, f));
        }
    }

    /// Ingest one received frame on `ingress`: learn + forward (multi-hop), and if it is for this node,
    /// handle the walk opcode. Emissions (forwarded frames + walk replies/probes) are appended to `out`.
    ///
    /// Returns the delivered-PDU hand-back (`specs/integration.md`): `Some` when the frame was
    /// delivered to this node but its opcode is not one the walk layer handles (a bounded copy;
    /// see [`DeliveredPdu`]). The firmware routes the `0x10..0x2F` control block through its own
    /// codec and drops the rest; a caller that ignores the return keeps the old drop behavior.
    pub fn ingest<F: Flash>(
        &mut self,
        ingress: u8,
        frame: &[u8],
        store: &mut Store<F>,
        out: &mut Emits,
    ) -> Option<DeliveredPdu> {
        let Ok(pdu) = Pdu::decode(frame) else {
            return None;
        };
        // Run the forwarder: forwarded copies emit straight into `out` (the caller's emissions;
        // an intermediate `Emits` here was a ~300 B stack duplicate the slice-7 budget audit
        // caught), while the delivered-to-self PDU is captured separately so the walk handling
        // does not nest inside the forwarder's borrow.
        let mut delivered: Option<PduBuf> = None;
        self.fwd.ingest(
            ingress,
            &pdu,
            &mut |d| {
                let mut b: PduBuf = Vec::new();
                let mut tmp = [0u8; MAX_PDU];
                if let Ok(n) = d.encode(&mut tmp) {
                    let _ = b.extend_from_slice(&tmp[..n]);
                    delivered = Some(b);
                }
            },
            &mut |port, f| emit(out, port, f),
        );
        if let Some(buf) = delivered {
            if let Ok(p) = Pdu::decode(&buf) {
                return self.handle_local(ingress, &p, store, out);
            }
        }
        None
    }

    /// Complete an in-flight probe (the firmware poll's "probe window elapsed" tick): emit the `PORTS`
    /// reply with the gathered per-port states, routed toward the controller.
    pub fn poll_probe(&mut self, out: &mut Emits) {
        let Some(probe) = self.probe.take() else {
            return;
        };
        let mut payload: PduBuf = Vec::new();
        let _ = payload.push(probe.n_ports);
        for p in 0..probe.n_ports {
            let (state, addr) = probe.states[p as usize];
            let _ = payload.push(p);
            let _ = payload.push(self.port_kinds[p as usize]);
            let _ = payload.push(state);
            let _ = payload.push(addr);
        }
        let r = Pdu::from_op(Opcode::Ports, self.addr(), probe.reply_to, &payload);
        self.fwd.originate(&r, &mut |port, f| emit(out, port, f));
    }

    fn handle_local<F: Flash>(
        &mut self,
        ingress: u8,
        pdu: &Pdu,
        store: &mut Store<F>,
        out: &mut Emits,
    ) -> Option<DeliveredPdu> {
        match Opcode::from_u8(pdu.opcode) {
            Some(Opcode::NodeHello) => self.on_hello(ingress, pdu, out),
            Some(Opcode::ProbePorts) => self.on_probe_ports(pdu, out),
            Some(Opcode::Assign) => self.on_assign(pdu, store, out),
            Some(Opcode::ConfigRead) => self.on_config_read(pdu, store, out),
            Some(Opcode::ConfigWrite) => self.on_config_write(pdu, store, out),
            Some(Opcode::ConfigWriteMulti) => self.on_config_write_multi(pdu, store, out),
            Some(Opcode::ConfigCompact) => self.on_config_compact(pdu, store, out),
            // Delivered but not handled here: an opcode L3 does not interpret (the reserved
            // control/telemetry blocks) or a stray controller-bound walk opcode (PORTS /
            // ASSIGN_ACK / CONFIG_RESP). Handed back to the caller by copy; the firmware decides
            // (it routes 0x10..0x2F, drops the rest).
            _ => return DeliveredPdu::from_pdu(pdu),
        }
        None
    }

    /// `CONFIG_READ [field_id, index]` -> `CONFIG_RESP [field_id, index, status, type_tag, value...]`,
    /// routed back toward the requester (the dynamic `store.get_value` path; `specs/l3.md`).
    fn on_config_read<F: Flash>(&mut self, pdu: &Pdu, store: &Store<F>, out: &mut Emits) {
        if pdu.payload.len() < 2 {
            return;
        }
        let key = Key {
            field_id: pdu.payload[0],
            index: pdu.payload[1],
        };
        let mut resp: PduBuf = Vec::new();
        let _ = resp.push(key.field_id);
        let _ = resp.push(key.index);
        match self.read_value(store, key) {
            Ok((v, status)) => {
                let _ = resp.push(status);
                if status == CFG_SECRET {
                    let _ = resp.push(0); // no type tag, no value
                } else {
                    push_value(&mut resp, &v);
                }
            }
            Err(e) => {
                let _ = resp.push(cfg_status(e));
                let _ = resp.push(0); // no type tag on an error
            }
        }
        self.reply_config(pdu.src, &resp, out);
    }

    /// `CONFIG_WRITE [field_id, index, type_tag, value...]` -> persist (`store.set_value`) and
    /// `CONFIG_RESP [field_id, index, status, type_tag, value...]` echoing the value a read answers
    /// now, none for a stored secret.
    fn on_config_write<F: Flash>(&mut self, pdu: &Pdu, store: &mut Store<F>, out: &mut Emits) {
        if pdu.payload.len() < CONFIG_WRITE_HEAD {
            return;
        }
        let key = Key {
            field_id: pdu.payload[0],
            index: pdu.payload[1],
        };
        let status = self.apply_write(
            store,
            key,
            pdu.payload[2],
            &pdu.payload[CONFIG_WRITE_HEAD..],
        );

        let mut resp: PduBuf = Vec::new();
        let _ = resp.push(key.field_id);
        let _ = resp.push(key.index);
        let _ = resp.push(status);
        match self.read_value(store, key) {
            Ok((v, read)) if status == CFG_OK && read != CFG_SECRET => push_value(&mut resp, &v),
            _ => {
                let _ = resp.push(0);
            }
        }
        self.reply_config(pdu.src, &resp, out);
    }

    /// `CONFIG_WRITE_MULTI [count, (field_id, index, type_tag, vlen, value[vlen])*]`: apply each entry
    /// (a whole board definition in one PDU), then a single `CONFIG_RESP [0, 0, status, 0]` ack.
    fn on_config_write_multi<F: Flash>(
        &mut self,
        pdu: &Pdu,
        store: &mut Store<F>,
        out: &mut Emits,
    ) {
        let p = pdu.payload;
        if p.is_empty() {
            return;
        }
        let count = p[0] as usize;
        let mut off = 1;
        let mut status = CFG_OK;
        for _ in 0..count {
            if off + 4 > p.len() {
                status = CFG_BAD;
                break;
            }
            let field_id = p[off];
            let index = p[off + 1];
            let type_tag = p[off + 2];
            let vlen = p[off + 3] as usize;
            off += 4;
            if off + vlen > p.len() {
                status = CFG_BAD;
                break;
            }
            let key = Key { field_id, index };
            let s = self.apply_write(store, key, type_tag, &p[off..off + vlen]);
            off += vlen;
            if s != CFG_OK {
                status = s; // first failure wins; keep applying the rest is not required
                break;
            }
        }
        let resp = [0u8, 0, status, 0];
        self.reply_config(pdu.src, &resp, out);
    }

    /// `CONFIG_COMPACT []` -> `store.compact()`, acked `CONFIG_RESP [0, 0, status, 0]` like the
    /// multi-write (`specs/cli-config.md` section 6.3), so superseded values and un-stored keys are
    /// erased from flash on request. It erases a page, so R4 refuses it while armed and touches no
    /// flash; disarmed it is the compaction a full-page write already runs, under the same watchdog
    /// budget (`specs/storage-layer.md`, "Compaction and the watchdog window").
    fn on_config_compact<F: Flash>(&mut self, pdu: &Pdu, store: &mut Store<F>, out: &mut Emits) {
        let status = if self.armed {
            CFG_ARMED
        } else {
            match store.compact() {
                Ok(()) => CFG_OK,
                Err(e) => compact_status(e),
            }
        };
        self.reply_config(pdu.src, &[0, 0, status, 0], out);
    }

    /// Decode + validate a value of `type_tag` and persist it to `key`, returning the `CONFIG_RESP`
    /// status (`OK` / `ARMED` / `UNKNOWN_KEY` / `TYPE_MISMATCH` / `BAD`; `BAD` also for a value a
    /// bridge cannot use, [`bridge_value_usable`]). The tombstone tag with no value bytes un-stores the key instead
    /// (`store.erase_value`, `specs/cli-config.md` section 6.2). The single choke point between
    /// `CONFIG_WRITE`/`_MULTI` and the store's writes, which is what makes the R4 gate airtight: the
    /// armed check runs first, before any decode, so an armed board persists nothing regardless of
    /// the request's shape, un-stores included.
    fn apply_write<F: Flash>(
        &mut self,
        store: &mut Store<F>,
        key: Key,
        type_tag: u8,
        bytes: &[u8],
    ) -> u8 {
        if self.armed {
            return CFG_ARMED;
        }
        // `None` un-stores.
        let value = if type_tag == ERASED_TAG && bytes.is_empty() {
            None
        } else {
            let Some(kind) = Type::from_tag(type_tag) else {
                return CFG_BAD;
            };
            let Some(value) = Value::decode(kind, bytes) else {
                return CFG_BAD;
            };
            if self.class == CLASS_BRIDGE && !bridge_value_usable(key, &value) {
                return CFG_BAD;
            }
            Some(value)
        };
        let persist = |store: &mut Store<F>| match value {
            Some(v) => store.set_value(key, v),
            None => store.erase_value(key),
        };
        match persist(store) {
            Ok(()) => {
                self.config_persisted = true;
                CFG_OK
            }
            // `Full` is not a failure, it is the store's documented "compact() then retry": the
            // value fits a clean page, just not the active page's remaining space. This and the
            // ASSIGN persist above are the store's two REMOTE writers (both carry the retry;
            // the audit corrected the original "only writer" census), and without it a board
            // simply stops accepting
            // config once its active page fills, which is what it did on the bench on 2026-07-31
            // (two of six staged fields written, the rest CFG_STORE_ERR).
            //
            // Compaction erases and programs a flash page, so it stalls the main loop: 60 ms for
            // a full 1 KiB page on a GD32F130, measured on silicon, against the firmware's 500 ms
            // free watchdog (`specs/storage-layer.md`, "Compaction and the watchdog window", which
            // owns that budget and the measurements behind it). That is safe HERE and only here:
            // the `armed` guard above has already refused the write if the board is armed, so a
            // compaction can never run under live torque. A second `Full` after compacting is a
            // genuine out-of-space and reports.
            Err(DynError::Store(StoreError::Full)) => match store.compact() {
                Ok(()) => match persist(store) {
                    Ok(()) => {
                        self.config_persisted = true;
                        CFG_OK
                    }
                    // A SECOND `Full`, on a page just packed down to its live set, is a genuinely
                    // out-of-space region and says so in its own byte.
                    Err(DynError::Store(StoreError::Full)) => CFG_NO_SPACE,
                    Err(e) => cfg_status(e),
                },
                Err(e) => compact_status(e),
            },
            Err(e) => cfg_status(e),
        }
    }

    /// Reply to a `TUNE_READ` / `TUNE_WRITE` the caller served out of its own RAM
    /// (`specs/rider-ui.md` section 4): the echoed key, the `CFG_*` status, and the live value on
    /// success. The reply is an ordinary `CONFIG_RESP` out the same emitter every other config
    /// reply uses, so the two lanes are one thing to route and one thing to parse; only the memory
    /// that answered differs.
    pub fn reply_tune(&self, dst: u8, key: Key, status: u8, value: Option<i16>, out: &mut Emits) {
        let mut buf = [0u8; TUNE_RESP_MAX];
        let n = tune_resp(&mut buf, key, status, value);
        self.reply_config(dst, &buf[..n], out);
    }

    /// Emit a `CONFIG_RESP` toward `dst` (the requester), routed by the learned table.
    fn reply_config(&self, dst: u8, payload: &[u8], out: &mut Emits) {
        let r = Pdu::from_op(Opcode::ConfigResp, self.addr(), dst, payload);
        self.fwd.originate(&r, &mut |port, f| emit(out, port, f));
    }

    fn on_hello(&mut self, ingress: u8, pdu: &Pdu, out: &mut Emits) {
        if pdu.payload.len() == 1 {
            // A request ("identify yourself"): reply identity out the ingress port (toward the asker).
            let kind = pdu.payload[0];
            let your_addr = if kind == KIND_CONTROLLER {
                let (first, len) = guest_slice(self.addr());
                // Wrap WITHIN the slice, which lies inside the guest range. An unguarded `+ 1`
                // would walk a grant into `0xFF` (broadcast), then `0x00` (no-address) and then
                // the BOARD range, where the forwarder's source-learning WOULD adopt it and a guest
                // could capture a node's route. One attach can burn several grants (a
                // retransmitted hello is a fresh grant), so same-boot reconnect churn reaches the
                // end of any finite counter. The `%` also covers a slot carried over from an
                // 8-address slice into the 7-address top one after an address change.
                let slot = self.guest_slot % len;
                self.guest_slot = (slot + 1) % len;
                first + slot
            } else {
                pdu::NO_ADDRESS
            };
            let node_id = self.addr();
            let fw = self.fw_ver.to_le_bytes();
            let reply = [node_id, PROTO_VER, fw[0], fw[1], self.mcu, your_addr];
            let r = Pdu::from_op(Opcode::NodeHello, node_id, pdu.src, &reply);
            emit(out, ingress, &r);
        } else if let Some(probe) = self.probe.as_mut() {
            // A reply to my own probe: classify the neighbour on `ingress` by its node_id.
            if (ingress as usize) < MAX_PORTS {
                let node_id = pdu.payload[0];
                let state = if node_id == pdu::NO_ADDRESS {
                    NB_UNASSIGNED
                } else {
                    NB_ASSIGNED
                };
                probe.states[ingress as usize] = (state, node_id);
            }
        }
    }

    fn on_probe_ports(&mut self, pdu: &Pdu, out: &mut Emits) {
        let n = self.fwd.n_ports();
        self.probe = Some(Probe {
            reply_to: pdu.src,
            n_ports: n,
            states: [(NB_EMPTY, 0u8); MAX_PORTS],
        });
        // Probe every port: a directed NODE_HELLO(kind=PROBE) to "the one peer" on each link.
        let req = [KIND_PROBE];
        for p in 0..n {
            let r = Pdu::from_op(Opcode::NodeHello, self.addr(), pdu::NO_ADDRESS, &req);
            emit(out, p, &r);
        }
    }

    fn on_assign<F: Flash>(&mut self, pdu: &Pdu, store: &mut Store<F>, out: &mut Emits) {
        if pdu.payload.len() < 2 {
            return;
        }
        let egress = pdu.payload[0];
        let new_addr = pdu.payload[1];
        if egress == EGRESS_SELF {
            // For this board: persist node_address, adopt it, and ACK back toward the controller.
            // Re-assigning the same address just re-persists and re-ACKs (idempotent).
            //
            // R4 covers this persist path too (`specs/integration.md`, "R4"): arming has no
            // assignment precondition, so a walk can reach an armed board, and the persist is a
            // flash program (a store append can trigger compaction, a page erase). While armed
            // the ASSIGN is refused whole: no persist, and no in-RAM adopt either (an
            // adopt-and-defer would half-state the ACK against a reboot). The ACK carries the
            // existing error status, so the controller sees a retryable refusal.
            let status = if self.armed {
                STATUS_ERR
            } else {
                match store.set_value(NODE_ADDRESS.key(), Value::U8(new_addr)) {
                    Ok(()) => {
                        self.fwd.set_addr(new_addr);
                        STATUS_OK
                    }
                    // Same contract as the CONFIG_WRITE arm below: `Full` means "compact() then
                    // retry" (ASSIGN is the store's OTHER remote writer; the audit caught this
                    // path missing the retry the config path got). The `armed` refusal above
                    // already guarantees compaction never runs under live torque.
                    Err(DynError::Store(StoreError::Full)) => match store.compact() {
                        Ok(()) => match store.set_value(NODE_ADDRESS.key(), Value::U8(new_addr)) {
                            Ok(()) => {
                                self.fwd.set_addr(new_addr);
                                STATUS_OK
                            }
                            Err(_) => STATUS_ERR,
                        },
                        // The ASSIGN ACK carries one OK/ERR bit and no room for a reason, so the
                        // commit failure cannot be named here; the `CONFIG_WRITE` path is where a
                        // board reports it (`CFG_COMMIT_FAILED`).
                        Err(_) => STATUS_ERR,
                    },
                    Err(_) => STATUS_ERR,
                }
            };
            let ack = [new_addr, status];
            let r = Pdu::from_op(Opcode::AssignAck, new_addr, pdu.src, &ack);
            self.fwd.originate(&r, &mut |port, f| emit(out, port, f));
        } else {
            // I am the relay: forward the ASSIGN out `egress` to the one peer, dst rewritten to 0x00 and
            // egress rewritten to SELF (the neighbour takes it as its own), src kept = the controller so
            // the neighbour's ACK routes back.
            let fwd_payload = [EGRESS_SELF, new_addr];
            let r = Pdu::from_op(Opcode::Assign, pdu.src, pdu::NO_ADDRESS, &fwd_payload);
            emit(out, egress, &r);
        }
    }
}

// ---------------------------------------------------------------------------------------------------
// The controller (host side): drives the walk, holds the address map.
// ---------------------------------------------------------------------------------------------------

/// The most nodes a Tier-1 walk addresses (the two worked topologies are <= 3 boards; headroom).
pub const MAX_NODES: usize = 8;
/// The pending-task queue bound.
pub const MAX_TASKS: usize = 16;

/// One step of the walk the controller still owes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Task {
    /// First contact on the attach link.
    Hello,
    /// Assign the board on the attach link directly (the gateway): `ASSIGN(egress=SELF)`.
    AssignGateway { new_addr: u8 },
    /// `PROBE_PORTS(addr)`.
    Probe { addr: u8 },
    /// Directed `ASSIGN(egress, new_addr)` to `relay` for its unassigned/colliding neighbour.
    AssignNeighbor { relay: u8, egress: u8, new_addr: u8 },
}

/// Where the host's own link lands, as the fleet itself reported it: the node whose `PORTS` reply
/// named the controller's guest address as an assigned neighbour, and the port it named it on.
///
/// This is the walk's evidence for "which board am I plugged into". It is not always present (a
/// board's probe window can elapse before the host answers its `NODE_HELLO` probe, in which case
/// that port reports `NB_EMPTY`), which is why it corroborates [`Controller::gateway_addr`] rather
/// than replacing it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct HostLink {
    /// The node that reported it: the board the host is attached to.
    pub node: u8,
    /// That board's local port number the host is on.
    pub port: u8,
    /// The port's medium tag ([`PORT_SWD`] for the mailbox, [`PORT_UART`], [`PORT_BLE`]).
    pub kind: u8,
}

/// One node the walk has placed: its address, its position (the relay it was reached through and
/// that relay's port; `(0, EGRESS_SELF)` for the node on the controller's own link), and whether
/// the address is one this controller handed out during this walk (`here`) rather than one the
/// node answered with already holding it. Only a node placed `here` is ever moved to resolve a
/// collision (`specs/l3.md`, "Discovery and address assignment", step 3).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Placed {
    addr: u8,
    relay: u8,
    egress: u8,
    here: bool,
}

/// The transient controller. Sequential request/response (the acknowledged delivery class): it holds
/// one outstanding request, advances on its reply, and works the queue to quiescence. All requests
/// leave on the single attach port; the gateway forwards them onward by `dst`.
pub struct Controller {
    guest_addr: u8,
    // The attached board answered first contact, so `guest_addr` is its grant.
    granted: bool,
    next_board: u8,
    // Every node this walk has placed: its positional identity, and whether this controller gave
    // it its address.
    assigned: Vec<Placed, MAX_NODES>,
    queue: Vec<Task, MAX_TASKS>,
    outstanding: Option<Task>,
    host_link: Option<HostLink>,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    /// A controller about to make first contact. Its provisional guest address is [`GUEST_FIRST`]
    /// (adopted from the gateway's grant on the first reply).
    pub fn new() -> Self {
        let mut queue: Vec<Task, MAX_TASKS> = Vec::new();
        let _ = queue.push(Task::Hello);
        Controller {
            guest_addr: GUEST_FIRST,
            granted: false,
            next_board: 0x01,
            assigned: Vec::new(),
            queue,
            outstanding: None,
            host_link: None,
        }
    }

    /// The controller's (guest) address.
    pub fn guest_addr(&self) -> u8 {
        self.guest_addr
    }

    /// The guest address the attached board granted in its `NODE_HELLO` reply, or `None` until that
    /// reply arrives. A guest that only attaches (the console's `watch` and `drive`) stops here and
    /// sends from this address.
    pub fn granted(&self) -> Option<u8> {
        self.granted.then_some(self.guest_addr)
    }

    /// The board addresses handed out this walk.
    pub fn assigned_addrs(&self) -> Vec<u8, MAX_NODES> {
        self.assigned.iter().map(|p| p.addr).collect()
    }

    /// **The attached node**: the board on the far end of the host's own link, by construction of
    /// the walk. First contact goes out the attach port and only the attached board can answer it,
    /// so the first node recorded is that board, whether it adopted a fresh address here
    /// ([`Task::AssignGateway`]) or answered already holding a persisted one.
    ///
    /// It is NOT "address 0x01", and it is not "the lowest address": a board that persisted `0x02`
    /// in a past session answers first contact as `0x02` while the freshly-allocated NEIGHBOUR gets
    /// `0x01` (allocation starts there). Addressing a bench command by a remembered literal is how
    /// the 2026-07-31 arm session sent its first power request to the SLAVE.
    pub fn gateway_addr(&self) -> Option<u8> {
        self.assigned.first().map(|p| p.addr)
    }

    /// The port-table evidence for the host's own link ([`HostLink`]), when a `PORTS` reply carried
    /// it. Corroborates [`Self::gateway_addr`] from the fleet's side.
    pub fn host_link(&self) -> Option<HostLink> {
        self.host_link
    }

    /// The walk is finished: nothing queued and nothing outstanding.
    pub fn is_complete(&self) -> bool {
        self.queue.is_empty() && self.outstanding.is_none()
    }

    /// The next request to send out the attach port, or `None` while a reply is still outstanding or
    /// the walk is complete.
    pub fn next_request(&mut self) -> Option<PduBuf> {
        if self.outstanding.is_some() {
            return None;
        }
        // Take from the front (BFS-ish: assign before descending keeps the walk outward).
        if self.queue.is_empty() {
            return None;
        }
        let task = self.queue.remove(0);
        let buf = self.build_request(&task);
        self.outstanding = Some(task);
        Some(buf)
    }

    /// Reply to a `NODE_HELLO` probe of the controller's own port (a board probing it on the walk's
    /// behalf): the controller is a node too, so it answers with its guest identity. Returns the reply
    /// PDU to send back out the attach port, or `None` if `frame` is not such a probe.
    pub fn reply_to_probe(&self, frame: &[u8]) -> Option<PduBuf> {
        let pdu = Pdu::decode(frame).ok()?;
        if pdu.known() == Some(Opcode::NodeHello) && pdu.payload.len() == 1 {
            // node_id = whatever address the gateway last granted us (provisionally GUEST_FIRST, and
            // adopted unchecked from the grant below), so the prober records "assigned(<that addr>)".
            let reply = [self.guest_addr, PROTO_VER, 0, 0, 0, pdu::NO_ADDRESS];
            let r = Pdu::from_op(Opcode::NodeHello, self.guest_addr, pdu.src, &reply);
            let mut tmp = [0u8; MAX_PDU];
            let n = r.encode(&mut tmp).ok()?;
            let mut b: PduBuf = Vec::new();
            b.extend_from_slice(&tmp[..n]).ok()?;
            return Some(b);
        }
        None
    }

    /// Feed a frame that may answer the outstanding request, advancing the walk. Returns whether
    /// the frame was that reply and was consumed.
    ///
    /// A reply is bound to the request it answers (`specs/l3.md`, "Discovery and address
    /// assignment"): a frame that does not fit the outstanding request ([`Self::answers`]) is not a
    /// reply, the request stays outstanding for its retransmit, and `false` tells the caller to
    /// route the frame on (its telemetry inbox). A duplicate or late `PORTS` read against the next
    /// probe is read against the wrong node's position and looks like a collision, which is how a
    /// rider renumbered an offroad slave to `0x03` and later the master to `0x05` on 2026-10-25.
    pub fn on_reply(&mut self, frame: &[u8]) -> bool {
        let Ok(pdu) = Pdu::decode(frame) else {
            return false;
        };
        let Some(task) = self.outstanding else {
            return false;
        };
        if !self.answers(task, &pdu) {
            return false;
        }
        self.outstanding = None;
        match task {
            Task::Hello => {
                let node_id = pdu.payload[0];
                let your_addr = pdu.payload[5];
                self.guest_addr = your_addr; // adopt the granted guest address
                self.granted = true;
                if node_id == pdu::NO_ADDRESS {
                    let na = self.alloc();
                    self.enqueue(Task::AssignGateway { new_addr: na });
                } else {
                    self.record(node_id, 0, EGRESS_SELF, false);
                    self.enqueue(Task::Probe { addr: node_id });
                }
            }
            // A non-OK status (e.g. the R4 armed refusal, STATUS_ERR) clears the outstanding task
            // but records nothing and probes nothing: the board did not adopt the address, so
            // acting on it would put a nobody-holds-it address in the map. Retry policy stays with
            // the host driving the walk (the unexpected-reply philosophy).
            //
            // A reassignment of a node already placed (`reassign`) moved its record when it was
            // decided, so `record` finds it and nothing is probed again.
            Task::AssignGateway { new_addr } => {
                if pdu.payload[1] == STATUS_OK && self.record(new_addr, 0, EGRESS_SELF, true) {
                    self.enqueue(Task::Probe { addr: new_addr });
                }
            }
            Task::AssignNeighbor {
                relay,
                egress,
                new_addr,
            } => {
                if pdu.payload[1] == STATUS_OK && self.record(new_addr, relay, egress, true) {
                    self.enqueue(Task::Probe { addr: new_addr });
                }
            }
            Task::Probe { addr } => self.on_ports(addr, &pdu),
        }
        true
    }

    /// Whether `pdu` is the reply to `task`, the outstanding request (`specs/l3.md`, "A reply is
    /// bound to the request it answers"). Every reply is addressed to this controller's guest
    /// address: for the first-contact `NODE_HELLO` that is the address the request was sent from,
    /// since the grant is adopted only from the reply that passes this check. A `PORTS` comes from
    /// the node probed, and an `ASSIGN_ACK` from the address assigned and names it in its payload.
    fn answers(&self, task: Task, pdu: &Pdu) -> bool {
        if pdu.dst != self.guest_addr {
            return false;
        }
        let p = pdu.payload;
        match (task, pdu.known()) {
            (Task::Hello, Some(Opcode::NodeHello)) => p.len() >= 6,
            (Task::Probe { addr }, Some(Opcode::Ports)) => pdu.src == addr,
            (
                Task::AssignGateway { new_addr } | Task::AssignNeighbor { new_addr, .. },
                Some(Opcode::AssignAck),
            ) => p.len() >= 2 && pdu.src == new_addr && p[0] == new_addr,
            _ => false,
        }
    }

    fn on_ports(&mut self, probed: u8, pdu: &Pdu) {
        if pdu.payload.is_empty() {
            return;
        }
        // The board we probed was reached through its parent (the relay we assigned it via). Its
        // upstream port reports that parent as an `assigned` neighbour - which is the link we came in
        // on, not a board to act on.
        let parent = self.parent_of(probed);
        let n = pdu.payload[0] as usize;
        let entry = |i: usize| -> Option<(u8, u8, u8, u8)> {
            let base = 1 + i * 4;
            let p = pdu.payload.get(base..base + 4)?;
            // Bits 0-1 only: bit 7 is the link-group flag, which says nothing about the state.
            Some((p[0], p[1], p[2] & NB_STATE_MASK, p[3]))
        };

        // The parent's address on more than one port: the upstream link and a neighbour that
        // already held the address the parent was given this walk. Which port is upstream cannot
        // be told from the report, so the parent moves (the rule below) and this node is probed
        // again once it has: the second report names the parent by its new address.
        if let Some(par) = parent {
            let seen = (0..n)
                .filter_map(entry)
                .filter(|&(_, _, state, naddr)| state == NB_ASSIGNED && naddr == par)
                .count();
            if seen > 1 && self.placed_here(par) {
                self.reassign(par);
                self.enqueue(Task::Probe { addr: probed });
                return;
            }
        }

        // The probed node's address for this report: it changes if the node itself is moved.
        let mut probed = probed;
        for (port, kind, state, naddr) in (0..n).map_while(entry) {
            if state == NB_ASSIGNED && naddr == self.guest_addr {
                // The board is telling us where WE are: this port is the host's own link, so this
                // is the attached node. Recorded once (the first reporter); a second claimant is a
                // topology the host cannot have, and the caller reconciles it against the gateway.
                if self.host_link.is_none() {
                    self.host_link = Some(HostLink {
                        node: probed,
                        port,
                        kind,
                    });
                }
            }
            match state {
                NB_UNASSIGNED => {
                    let na = self.alloc();
                    self.enqueue(Task::AssignNeighbor {
                        relay: probed,
                        egress: port,
                        new_addr: na,
                    });
                }
                NB_ASSIGNED => {
                    if pdu::is_controller(naddr) {
                        // The controller/guest itself, reached back through a probe: ignore.
                    } else if Some(naddr) == parent {
                        // The upstream link back to the parent we descended from: ignore.
                    } else if let Some(pos) = self.position_of(naddr) {
                        if pos == (probed, port) {
                            // Already known at this position - nothing to do (a re-walk).
                        } else if self.placed_here(naddr) {
                            // Two holders of one address: the node this walk assigned it to, and
                            // this neighbour, which answered already holding it. The one this
                            // walk assigned moves; the neighbour keeps its address and is
                            // descended into like any held one (`specs/l3.md`, step 3).
                            let moved = self.reassign(naddr);
                            if probed == naddr {
                                probed = moved;
                            }
                            self.record(naddr, probed, port, false);
                            self.enqueue(Task::Probe { addr: naddr });
                        } else {
                            // Two nodes that both answered holding it: this neighbour, the one
                            // reached second, is reassigned and re-persists.
                            let na = self.alloc();
                            self.enqueue(Task::AssignNeighbor {
                                relay: probed,
                                egress: port,
                                new_addr: na,
                            });
                        }
                    } else {
                        // A board carrying a stale address from a past session: adopt it as-is and
                        // descend (a valid, persisted board; reassigned only if it later collides).
                        self.record(naddr, probed, port, false);
                        self.enqueue(Task::Probe { addr: naddr });
                    }
                }
                _ => {} // empty
            }
        }
    }

    /// Whether `addr` was handed out by this controller during this walk.
    fn placed_here(&self, addr: u8) -> bool {
        self.assigned.iter().any(|p| p.addr == addr && p.here)
    }

    /// Move the node this walk placed at `old` to a fresh address, returning it. The move is
    /// decided now and the map follows it at once (the node's record, the records of nodes reached
    /// through it, the queued tasks that name it, the host link); its `ASSIGN` goes to the front of
    /// the queue, so nothing addressed to `old` is sent before the node has left it. The node at
    /// the controller's own link is assigned directly (`dst = 0x00`), any other through its relay.
    fn reassign(&mut self, old: u8) -> u8 {
        let na = self.alloc();
        let mut task = None;
        for p in self.assigned.iter_mut() {
            if p.addr == old {
                p.addr = na;
                task = Some(if p.egress == EGRESS_SELF {
                    Task::AssignGateway { new_addr: na }
                } else {
                    Task::AssignNeighbor {
                        relay: p.relay,
                        egress: p.egress,
                        new_addr: na,
                    }
                });
            }
        }
        for p in self.assigned.iter_mut() {
            if p.relay == old {
                p.relay = na;
            }
        }
        for t in self.queue.iter_mut() {
            match t {
                Task::Probe { addr } if *addr == old => *addr = na,
                Task::AssignNeighbor { relay, .. } if *relay == old => *relay = na,
                _ => {}
            }
        }
        if let Some(h) = self.host_link.as_mut() {
            if h.node == old {
                h.node = na;
            }
        }
        if let Some(t) = task {
            let _ = self.queue.insert(0, t);
        }
        na
    }

    /// The address of the relay through which `addr` was reached (its parent in the walk tree), or
    /// `None` for the gateway (reached on the controller's own link).
    fn parent_of(&self, addr: u8) -> Option<u8> {
        self.assigned
            .iter()
            .find(|p| p.addr == addr)
            .map(|p| p.relay)
            .filter(|&r| pdu::is_unicast(r))
    }

    fn build_request(&self, task: &Task) -> PduBuf {
        let (op, dst, payload): (Opcode, u8, Vec<u8, 4>) = match *task {
            Task::Hello => (Opcode::NodeHello, pdu::NO_ADDRESS, one(KIND_CONTROLLER)),
            Task::AssignGateway { new_addr } => {
                (Opcode::Assign, pdu::NO_ADDRESS, two(EGRESS_SELF, new_addr))
            }
            Task::Probe { addr } => (Opcode::ProbePorts, addr, Vec::new()),
            Task::AssignNeighbor {
                relay,
                egress,
                new_addr,
            } => (Opcode::Assign, relay, two(egress, new_addr)),
        };
        let r = Pdu::from_op(op, self.guest_addr, dst, &payload);
        let mut tmp = [0u8; MAX_PDU];
        let n = r.encode(&mut tmp).unwrap_or(0);
        let mut b: PduBuf = Vec::new();
        let _ = b.extend_from_slice(&tmp[..n]);
        b
    }

    /// Allocate the next free board address (`0x01..=0x7F`), skipping any already handed out.
    fn alloc(&mut self) -> u8 {
        loop {
            let a = self.next_board;
            self.next_board = self.next_board.wrapping_add(1);
            if pdu::is_board(a) && self.position_of(a).is_none() {
                return a;
            }
        }
    }

    /// Place `addr` at `(relay, egress)` unless it is already placed; whether it was new.
    fn record(&mut self, addr: u8, relay: u8, egress: u8, here: bool) -> bool {
        if self.position_of(addr).is_some() {
            return false;
        }
        self.assigned
            .push(Placed {
                addr,
                relay,
                egress,
                here,
            })
            .is_ok()
    }

    fn position_of(&self, addr: u8) -> Option<(u8, u8)> {
        self.assigned
            .iter()
            .find(|p| p.addr == addr)
            .map(|p| (p.relay, p.egress))
    }

    fn enqueue(&mut self, task: Task) {
        let _ = self.queue.push(task);
    }
}

/// A one-byte payload.
fn one(b: u8) -> Vec<u8, 4> {
    let mut v = Vec::new();
    let _ = v.push(b);
    v
}

/// A two-byte payload.
fn two(a: u8, b: u8) -> Vec<u8, 4> {
    let mut v = Vec::new();
    let _ = v.push(a);
    let _ = v.push(b);
    v
}

#[cfg(test)]
mod tests;
