//! The walk and the configuration commands (`specs/hoverboard-bridge.md`, "The terminal tool"): the whole
//! walk with `net`'s [`Controller`], then `node.class` and `device.name` read from every address;
//! `CONFIG_READ` and `CONFIG_WRITE` typed as the store's registry says; and `--node` resolved to
//! an address by number or by name.

use std::time::{Duration, Instant};

use net::pdu::{Opcode, Pdu, NO_ADDRESS};
use net::walk::{Controller, CLASS_BRIDGE};
use store::{Key, Type, Value, DEVICE_NAME, NODE_CLASS};

use crate::wire::Wire;

/// How long one walk or configuration reply may take. A `PROBE_PORTS` is answered after the
/// node's probe window (half a second on a board and on the bridge), and the walk's requests to the
/// far board cross the mailbox and the inter-board UART.
pub const REPLY_WAIT: Duration = Duration::from_secs(3);
/// How many times a walk or configuration request is sent before giving up.
const TRIES: u32 = 3;
/// The longest one wait for a packet inside the walk, so a resend is not late by much.
const POLL: Duration = Duration::from_millis(50);

/// The motor controller's `node.class`, the registry default.
pub const CLASS_MOTOR: u8 = NODE_CLASS.default();

/// A node the walk found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Found {
    /// Its address.
    pub addr: u8,
    /// Its `node.class`, if it answered.
    pub class: Option<u8>,
    /// Its `device.name`, if it answered.
    pub name: Option<String>,
}

/// The name of a `node.class` value.
pub fn class_name(class: Option<u8>) -> &'static str {
    match class {
        Some(CLASS_MOTOR) => "motor controller",
        Some(CLASS_BRIDGE) => "bridge",
        Some(_) => "unknown class",
        None => "no answer",
    }
}

/// The name of a `CONFIG_RESP` status byte (`crates/net/src/walk.rs`, the `CFG_*` statuses).
pub fn status_name(status: u8) -> String {
    use net::walk::*;
    match status {
        CFG_OK => "ok".into(),
        CFG_BAD => "bad request".into(),
        CFG_UNKNOWN_KEY => "unknown key".into(),
        CFG_TYPE_MISMATCH => "type mismatch".into(),
        CFG_STORE_ERR => "store error".into(),
        CFG_ARMED => "armed".into(),
        CFG_READ_ONLY => "read-only".into(),
        CFG_NO_SPACE => "no space".into(),
        CFG_COMMIT_FAILED => "commit failed".into(),
        s => format!("status {s}"),
    }
}

/// Run the whole walk over `wire`: first contact, then every node addressed and probed. Answers
/// the probes of the tool's own link as the walk goes. A request with no answer in
/// [`REPLY_WAIT`] is sent again, [`TRIES`] times in all.
pub fn walk<W: Wire>(wire: &mut W) -> Result<Controller, String> {
    walk_within(wire, REPLY_WAIT)
}

/// [`walk`] with the resend timeout `reply_wait`.
///
/// The timeout runs off the outstanding request's own age, so a request is resent when it is due
/// whether or not other packets arrived meanwhile (`specs/l3.md`, "A reply is bound to the request
/// it answers"): a board streams `CYCLIC_STATE` at 5 Hz, and a loop that resent only on a quiet
/// `recv` never resent a request whose reply was lost. A packet the controller does not take as
/// the outstanding request's reply (telemetry, a duplicate or late reply) is dropped here; the
/// walk runs before `watch` and `drive` start counting.
fn walk_within<W: Wire>(wire: &mut W, reply_wait: Duration) -> Result<Controller, String> {
    let mut c = Controller::new();
    let mut last: Option<(Vec<u8>, Instant, u32)> = None;
    while !c.is_complete() {
        if let Some(req) = c.next_request() {
            wire.send(&req);
            last = Some((req.to_vec(), Instant::now(), 1));
        }
        if let Some(pkt) = wire.recv(POLL) {
            if let Some(r) = c.reply_to_probe(&pkt) {
                wire.send(&r);
            } else if c.on_reply(&pkt) {
                last = None;
            }
        }
        if let Some((req, at, tries)) = last.as_mut() {
            if at.elapsed() > reply_wait {
                if *tries >= TRIES {
                    return Err(format!(
                        "the walk got no answer to {}",
                        describe(req).unwrap_or_default()
                    ));
                }
                wire.send(req);
                *at = Instant::now();
                *tries += 1;
            }
        }
    }
    Ok(c)
}

/// A request named for an error message: its opcode and destination.
fn describe(pdu: &[u8]) -> Option<String> {
    let p = Pdu::decode(pdu).ok()?;
    Some(format!("{:?} to {:#04x}", p.known()?, p.dst))
}

/// A `CONFIG_RESP`: its status, and the value's type tag and bytes when there is one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigResp {
    /// The `CFG_*` status.
    pub status: u8,
    /// The type tag, `0` with no value.
    pub tag: u8,
    /// The value bytes.
    pub bytes: Vec<u8>,
}

impl ConfigResp {
    /// The value, decoded.
    pub fn value(&self) -> Option<Value<'_>> {
        Value::decode(Type::from_tag(self.tag)?, &self.bytes)
    }
}

/// Send a `CONFIG_*` request for `key` from `src` to `dst` and wait for its `CONFIG_RESP`. A
/// request to `0x00` is answered by the node on the tool's own link, whatever its address.
fn config<W: Wire>(
    wire: &mut W,
    op: Opcode,
    src: u8,
    dst: u8,
    key: Key,
    value: &[u8],
) -> Result<ConfigResp, String> {
    let mut payload = vec![key.field_id, key.index];
    payload.extend_from_slice(value);
    let pdu = Pdu::from_op(op, src, dst, &payload);
    let mut buf = [0u8; net::walk::MAX_PDU];
    let n = pdu
        .encode(&mut buf)
        .map_err(|_| "the value does not fit one PDU".to_string())?;
    for _ in 0..TRIES {
        wire.send(&buf[..n]);
        let deadline = Instant::now() + REPLY_WAIT;
        while let Some(pkt) = wire.recv(deadline.saturating_duration_since(Instant::now())) {
            let Ok(p) = Pdu::decode(&pkt) else { continue };
            let from = dst == NO_ADDRESS || p.src == dst;
            if p.known() == Some(Opcode::ConfigResp)
                && p.dst == src
                && from
                && p.payload.len() >= 4
                && p.payload[..2] == [key.field_id, key.index]
            {
                return Ok(ConfigResp {
                    status: p.payload[2],
                    tag: p.payload[3],
                    bytes: p.payload[4..].to_vec(),
                });
            }
        }
    }
    Err(format!("no CONFIG_RESP from {dst:#04x}"))
}

/// `CONFIG_READ` of `key` at `dst`.
pub fn read<W: Wire>(wire: &mut W, src: u8, dst: u8, key: Key) -> Result<ConfigResp, String> {
    config(wire, Opcode::ConfigRead, src, dst, key, &[])
}

/// `CONFIG_WRITE` of a `kind` value, `bytes` as the wire carries it, to `key` at `dst`.
pub fn write<W: Wire>(
    wire: &mut W,
    src: u8,
    dst: u8,
    key: Key,
    kind: Type,
    bytes: &[u8],
) -> Result<ConfigResp, String> {
    let mut v = vec![kind.tag()];
    v.extend_from_slice(bytes);
    config(wire, Opcode::ConfigWrite, src, dst, key, &v)
}

/// Read `node.class` and `device.name` from every address the walk found, in address order.
pub fn survey<W: Wire>(wire: &mut W, walked: &Controller) -> Vec<Found> {
    let src = walked.guest_addr();
    let mut addrs = walked.assigned_addrs().to_vec();
    addrs.sort_unstable();
    addrs
        .into_iter()
        .map(|addr| {
            let class =
                read(wire, src, addr, NODE_CLASS.key())
                    .ok()
                    .and_then(|r| match r.value() {
                        Some(Value::U8(c)) => Some(c),
                        _ => None,
                    });
            let name =
                read(wire, src, addr, DEVICE_NAME.key())
                    .ok()
                    .and_then(|r| match r.value() {
                        Some(Value::Str(s)) => Some(String::from_utf8_lossy(s).into_owned()),
                        _ => None,
                    });
            Found { addr, class, name }
        })
        .collect()
}

/// A number: decimal, or hexadecimal with `0x`.
pub fn parse_int(s: &str) -> Option<i128> {
    let (neg, digits) = match s.strip_prefix('-') {
        Some(d) => (true, d),
        None => (false, s),
    };
    let v = match digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        Some(h) => i128::from_str_radix(h, 16).ok()?,
        None => digits.parse::<i128>().ok()?,
    };
    Some(if neg { -v } else { v })
}

/// The node `spec` names among `found`: an address, or a `device.name`.
pub fn resolve(spec: &str, found: &[Found]) -> Result<u8, String> {
    if let Some(a) = parse_int(spec) {
        return u8::try_from(a)
            .ok()
            .filter(|&a| net::pdu::is_board(a))
            .ok_or_else(|| format!("not a node address (0x01..0x7f): {spec}"));
    }
    let named: Vec<u8> = found
        .iter()
        .filter(|f| f.name.as_deref() == Some(spec))
        .map(|f| f.addr)
        .collect();
    match named[..] {
        [a] => Ok(a),
        [] => Err(format!("no node named {spec}")),
        _ => Err(format!("more than one node is named {spec}")),
    }
}

/// A field argument: `<id>` or `<id>.<index>`, each a number.
pub fn parse_key(s: &str) -> Result<Key, String> {
    let (id, index) = s.split_once('.').unwrap_or((s, "0"));
    let byte = |t: &str| {
        parse_int(t)
            .and_then(|v| u8::try_from(v).ok())
            .ok_or_else(|| format!("not a field: {s}"))
    };
    Ok(Key {
        field_id: byte(id)?,
        index: byte(index)?,
    })
}

/// `text` as a value of the registry's type for `key`, encoded as `CONFIG_WRITE` carries it.
pub fn parse_value(key: Key, text: &str) -> Result<(Type, Vec<u8>), String> {
    let def = store::lookup_key(key).ok_or_else(|| {
        format!(
            "no field {:#04x} index {} in the registry",
            key.field_id, key.index
        )
    })?;
    let bad = || format!("not a {:?}: {text}", def.kind);
    let int = |lo: i128, hi: i128| {
        parse_int(text)
            .filter(|v| (lo..=hi).contains(v))
            .ok_or_else(bad)
    };
    let bytes = match def.kind {
        Type::U8 => vec![int(0, u8::MAX.into())? as u8],
        Type::U16 => (int(0, u16::MAX.into())? as u16).to_le_bytes().to_vec(),
        Type::U32 => (int(0, u32::MAX.into())? as u32).to_le_bytes().to_vec(),
        Type::U64 => (int(0, u64::MAX.into())? as u64).to_le_bytes().to_vec(),
        Type::I16 => (int(i16::MIN.into(), i16::MAX.into())? as i16)
            .to_le_bytes()
            .to_vec(),
        Type::I32 => (int(i32::MIN.into(), i32::MAX.into())? as i32)
            .to_le_bytes()
            .to_vec(),
        Type::I64 => (int(i64::MIN.into(), i64::MAX.into())? as i64)
            .to_le_bytes()
            .to_vec(),
        Type::Bool => match text {
            "1" | "true" | "on" => vec![1],
            "0" | "false" | "off" => vec![0],
            _ => return Err(bad()),
        },
        Type::Str => text.as_bytes().to_vec(),
        Type::Blob => {
            let h = text.trim_start_matches("0x");
            if !h.len().is_multiple_of(2) {
                return Err(bad());
            }
            (0..h.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&h[i..i + 2], 16).map_err(|_| bad()))
                .collect::<Result<_, _>>()?
        }
    };
    Ok((def.kind, bytes))
}

/// A value as `config` prints it.
pub fn show(v: &Value) -> String {
    match *v {
        Value::U8(x) => format!("{x} ({x:#04x})"),
        Value::U16(x) => format!("{x} ({x:#06x})"),
        Value::U32(x) => format!("{x} ({x:#010x})"),
        Value::U64(x) => x.to_string(),
        Value::I16(x) => x.to_string(),
        Value::I32(x) => x.to_string(),
        Value::I64(x) => x.to_string(),
        Value::Bool(x) => x.to_string(),
        Value::Str(s) => format!("{:?}", String::from_utf8_lossy(s)),
        Value::Bytes(b) => b.iter().map(|x| format!("{x:02x}")).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use net::walk::{NB_ASSIGNED, PORT_UART, PROTO_VER};
    use std::collections::VecDeque;

    /// One board at `0x01` behind a link that streams telemetry without pause and loses the
    /// reply to the first `PROBE_PORTS`.
    struct Chatty {
        replies: VecDeque<Vec<u8>>,
        probes: u32,
    }

    fn encode(op: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = [0u8; net::walk::MAX_PDU];
        let n = Pdu::new(op, src, dst, payload)
            .unwrap()
            .encode(&mut buf)
            .unwrap();
        buf[..n].to_vec()
    }

    impl Wire for Chatty {
        fn send(&mut self, pdu: &[u8]) {
            let p = Pdu::decode(pdu).unwrap();
            match p.known() {
                Some(Opcode::NodeHello) => self.replies.push_back(encode(
                    Opcode::NodeHello.to_u8(),
                    0x01,
                    p.src,
                    &[0x01, PROTO_VER, 0, 0, 0x10, 0x88],
                )),
                Some(Opcode::ProbePorts) => {
                    self.probes += 1;
                    if self.probes > 1 {
                        self.replies.push_back(encode(
                            Opcode::Ports.to_u8(),
                            0x01,
                            p.src,
                            &[1, 0, PORT_UART, NB_ASSIGNED, 0x88],
                        ));
                    }
                }
                _ => {}
            }
        }
        fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
            Some(self.replies.pop_front().unwrap_or_else(|| {
                encode(linkctl::OP_CYCLIC_STATE, 0x01, net::pdu::BROADCAST, &[0; 8])
            }))
        }
        fn window(&self) -> usize {
            1
        }
    }

    #[test]
    fn a_lost_probe_reply_is_resent_while_telemetry_keeps_arriving() {
        let mut wire = Chatty {
            replies: VecDeque::new(),
            probes: 0,
        };
        let c = walk_within(&mut wire, Duration::from_millis(100)).expect("the walk completes");
        assert_eq!(wire.probes, 2, "the probe was resent once");
        assert_eq!(c.assigned_addrs().to_vec(), [0x01]);
        assert_eq!(c.guest_addr(), 0x88);
    }
}
