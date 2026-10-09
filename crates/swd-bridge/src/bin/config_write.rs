//! General config-field reader/writer over the SWD mailbox: stage a whole board-layout preset
//! (or any registered field) on a board, confirm each write with a read-back, or just read.
//!
//! Usage: `swd-mailbox-config <openocd-host:port> [--base HEX] [--dst attached|ADDR] [--tune]
//!         [--force-type TYPE] ARG ...`
//!
//! Each `ARG` is one of:
//! - `FIELD[:INDEX]=VALUE`: write, then read the field back and confirm it matches.
//! - `FIELD[:INDEX]`: read only.
//!
//! - `FIELD` is a `0x`-hex or decimal store field id (e.g. `0x48` = `imu.scl_pin`).
//! - `INDEX` (optional, default 0) is the per-motor / per-gain index (`0x71:1` = gain A `bk`).
//! - `VALUE` is parsed against the field's REGISTERED type (`crates/store`; a packed `port|pin`
//!   byte such as `0x16` = PB6 is a `u8` field). No board-model validation happens here: a
//!   duplicate pin or a gate-capable pin in a LED field is a well-typed write this tool passes
//!   through, so the FIRMWARE's boot validator (`crates/board`) is what judges the layout after a
//!   reboot.
//! - `--dst` picks the board: `attached` (the default) resolves the board this host's mailbox
//!   link lands on from the walk; an address reaches a board THROUGH it (the two-hop case).
//! - `--tune` sends `TUNE_WRITE` / `TUNE_READ` (`specs/rider-ui.md` section 4, the live lane:
//!   RAM only, no flash) instead of `CONFIG_WRITE` / `CONFIG_READ`.
//! - `--force-type TYPE` encodes every written value as that store type whatever the registry
//!   says, so the board's type check can be watched refusing it (`CFG_TYPE_MISMATCH`).
//!
//! Every reply is printed with its `CFG_*` status by name. A non-OK status or a read-back that
//! differs from what was written counts as a failure for the exit code, but every argument is
//! still attempted, because a bench gate wants to see each refusal, not stop at the first.
//!
//! Example (the silicon-queue section-6 VALID-layout case: the standard-family IMU on PB6/PB7
//! with the port freed in `LINK_SET`, then reboot to have the board validate it):
//!
//! ```text
//! swd-mailbox-config 127.0.0.1:6666 0x02=0x06 0x48=0x16 0x49=0x17 0x60=2
//! #   LINK_SET(0x02)=0b110 (inter-board+BLE live, PB6/PB7 port freed)
//! #   imu.scl_pin(0x48)=PB6  imu.sda_pin(0x49)=PB7  imu.model(0x60)=2
//! # then REBOOT the board (bench: power-cycle via the relay, or the L3 REBOOT opcode once it
//! # exists) and read BOARD_OBS: expect magic "BRDV", result 0 (OBS_OK). Result 11 with a
//! # working layout = IMU frame refused (IMU_AXIS_SIGN / IMU_AXIS_ROLE not a rotation).
//! ```
//!
//! A wrong-type value, an unknown field id, or a bad argument is rejected before any wire
//! traffic.

use std::process::ExitCode;
use std::time::Duration;

use net::walk::{CFG_ARMED, CFG_BAD, CFG_OK, CFG_STORE_ERR, CFG_TYPE_MISMATCH, CFG_UNKNOWN_KEY};
use store::{Key, Type, Value};
use swd_bridge::config::{
    parse_field_arg, parse_field_value, parse_key_arg, parse_type_name, parse_value_as,
};
use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::walk::{host_link_note, CfgResp, WalkDriver};
use swd_bridge::{HostMailbox, MAILBOX_BASE};

const USAGE: &str = "usage: swd-mailbox-config <host:port> [--base HEX] [--dst attached|ADDR] \
     [--tune] [--force-type TYPE] (FIELD[:INDEX]=VALUE | FIELD[:INDEX]) ...";

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("FAIL: {e}");
            ExitCode::FAILURE
        }
    }
}

/// A `CFG_*` status by name, for the printed reply.
fn status_name(s: u8) -> String {
    match s {
        CFG_OK => "CFG_OK".into(),
        CFG_BAD => "CFG_BAD".into(),
        CFG_UNKNOWN_KEY => "CFG_UNKNOWN_KEY".into(),
        CFG_TYPE_MISMATCH => "CFG_TYPE_MISMATCH".into(),
        CFG_STORE_ERR => "CFG_STORE_ERR".into(),
        CFG_ARMED => "CFG_ARMED".into(),
        other => format!("status {other}"),
    }
}

/// The decoded value a `CONFIG_RESP` carries, or `None` on a refusal (no type tag).
fn resp_value(r: &CfgResp) -> Option<Value<'_>> {
    let kind = Type::from_tag(r.type_tag)?;
    Value::decode(kind, &r.value)
}

/// One printable reply line: the status by name and the value if there is one.
///
/// A `Str` is rendered LOSSILY rather than as a byte array: the board carries a `STR` record as
/// bytes and makes no encoding claim about it (`specs/decision-flash-budget.md`, shrink round 2
/// item 4), but this line is read by a person, and `std` makes the lossy render free here. The host
/// is the right place to pay for it; the board is not.
fn describe(r: &CfgResp) -> String {
    match resp_value(r) {
        Some(Value::Str(b)) => format!(
            "{} value Str({:?})",
            status_name(r.status),
            String::from_utf8_lossy(b)
        ),
        Some(v) => format!("{} value {v:?}", status_name(r.status)),
        None => status_name(r.status),
    }
}

/// How `--dst` was given (mirrors `swd-mailbox-inputs` / `swd-mailbox-drive`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DstArg {
    Attached,
    Explicit(u8),
}

fn parse_u8(s: &str) -> Result<u8, String> {
    let r = s
        .strip_prefix("0x")
        .map(|h| u8::from_str_radix(h, 16))
        .unwrap_or_else(|| s.parse::<u8>());
    r.map_err(|_| format!("bad u8 value {s:?}"))
}

fn parse_dst(s: &str) -> Result<DstArg, String> {
    if s.eq_ignore_ascii_case("attached") {
        return Ok(DstArg::Attached);
    }
    parse_u8(s)
        .map(DstArg::Explicit)
        .map_err(|_| format!("bad --dst value {s:?} (want `attached` or an address like 0x02)"))
}

/// One parsed argument: a write (with the raw value text) or a read.
enum Op<'a> {
    Write { key: Key, raw: &'a str },
    Read { key: Key },
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let endpoint = args.next().ok_or(USAGE)?;

    // Parse + type-check EVERYTHING before any wire traffic, so a typo fails fast without
    // half-staging a layout.
    let mut base = MAILBOX_BASE;
    let mut dst = DstArg::Attached;
    let mut tune = false;
    let mut force_type: Option<Type> = None;
    let mut raw_args: Vec<String> = Vec::new();
    let rest: Vec<String> = args.collect();
    let mut it = rest.into_iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--base" => {
                let b = it.next().ok_or("--base needs a hex value")?;
                base = u32::from_str_radix(b.trim_start_matches("0x"), 16)
                    .map_err(|_| format!("bad --base {b:?}"))?;
            }
            "--dst" => dst = parse_dst(&it.next().ok_or("--dst needs a value")?)?,
            "--tune" => tune = true,
            "--force-type" => {
                let t = it.next().ok_or("--force-type needs a type name")?;
                force_type =
                    Some(parse_type_name(&t).ok_or_else(|| format!("bad --force-type {t:?}"))?);
            }
            other if other.starts_with("--") => {
                return Err(format!("unknown argument {other:?}\n{USAGE}"))
            }
            _ => raw_args.push(a),
        }
    }
    if raw_args.is_empty() {
        return Err(format!("no FIELD[:INDEX][=VALUE] arguments\n{USAGE}"));
    }
    // The value to send for a write: the registry's type unless --force-type overrides it.
    fn value_of<'r>(force: Option<Type>, key: Key, raw: &'r str) -> Result<Value<'r>, String> {
        match force {
            Some(kind) => parse_value_as(kind, key.field_id, raw).map_err(|e| e.to_string()),
            None => parse_field_value(key.field_id, raw).map_err(|e| e.to_string()),
        }
    }
    let mut ops: Vec<Op<'_>> = Vec::new();
    for a in &raw_args {
        if a.contains('=') {
            let (field_id, index, raw) = parse_field_arg(a).map_err(|e| e.to_string())?;
            let key = Key { field_id, index };
            // Pre-validate the value here too (fail before the wire).
            value_of(force_type, key, raw)?;
            ops.push(Op::Write { key, raw });
        } else {
            let (field_id, index) = parse_key_arg(a).map_err(|e| e.to_string())?;
            ops.push(Op::Read {
                key: Key { field_id, index },
            });
        }
    }

    // Attach the mailbox + walk once to learn the fleet's addresses.
    let mem = OpenOcdTcl::connect(&endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200)
        .map_err(|_| "firmware never wrote epoch_ack (no poll-site running?)".to_string())?;

    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;
    // The ATTACHED node, resolved from the walk (never a remembered literal; see the R4 incident
    // in `WalkDriver::attached_addr`), unless --dst names a board reached through it.
    let dst = match dst {
        DstArg::Attached => {
            let a = walk.attached_addr().map_err(|e| e.to_string())?;
            println!(
                "dst resolved: attached node 0x{a:02x}{}",
                host_link_note(walk.host_link())
            );
            a
        }
        DstArg::Explicit(a) => {
            match walk.attached_addr().ok() {
                Some(r) if r == a => {
                    println!("dst 0x{a:02x} (explicit; this IS the attached node)")
                }
                Some(r) => println!(
                    "dst 0x{a:02x} (explicit; NOTE the attached node is 0x{r:02x}, so this reaches \
                     a board THROUGH it)"
                ),
                None => println!("dst 0x{a:02x} (explicit; the walk resolved no attached node)"),
            }
            a
        }
    };
    let lane = if tune { "TUNE" } else { "CONFIG" };
    println!("{} {} op(s) on node 0x{dst:02x}", ops.len(), lane);

    let t = Duration::from_secs(10);
    let mut failures = 0;
    for op in &ops {
        match op {
            Op::Read { key } => {
                let r = if tune {
                    walk.tune_read(dst, *key, t)
                } else {
                    walk.config_read(dst, *key, t)
                }
                .map_err(|e| e.to_string())?;
                println!(
                    "  {lane}_READ {:#04x}:{} -> {}",
                    key.field_id,
                    key.index,
                    describe(&r)
                );
                if r.status != CFG_OK {
                    failures += 1;
                }
            }
            Op::Write { key, raw } => {
                let value = value_of(force_type, *key, raw)?;
                let w = if tune {
                    walk.tune_write(dst, *key, &value, t)
                } else {
                    walk.config_write(dst, *key, value, t)
                }
                .map_err(|e| e.to_string())?;
                println!(
                    "  {lane}_WRITE {:#04x}:{} = {raw} ({:?}) -> {}",
                    key.field_id,
                    key.index,
                    value.kind(),
                    describe(&w)
                );
                if w.status != CFG_OK {
                    failures += 1;
                    continue;
                }
                let r = if tune {
                    walk.tune_read(dst, *key, t)
                } else {
                    walk.config_read(dst, *key, t)
                }
                .map_err(|e| e.to_string())?;
                let got = resp_value(&r);
                let matches = r.status == CFG_OK && got.as_ref() == Some(&value);
                println!(
                    "  {lane}_READ  {:#04x}:{} -> {}{}",
                    key.field_id,
                    key.index,
                    describe(&r),
                    if matches {
                        "  (write -> read matches)"
                    } else {
                        "  MISMATCH"
                    }
                );
                if !matches {
                    failures += 1;
                }
            }
        }
    }

    if failures == 0 {
        if tune {
            println!("PASS: {} TUNE op(s) answered CFG_OK", ops.len());
        } else {
            println!(
                "PASS: {} CONFIG op(s) answered CFG_OK; a staged layout needs a REBOOT for the \
                 board's boot validator to judge it",
                ops.len()
            );
        }
        Ok(())
    } else {
        Err(format!(
            "{failures} op(s) did not answer CFG_OK (or read back differently)"
        ))
    }
}
