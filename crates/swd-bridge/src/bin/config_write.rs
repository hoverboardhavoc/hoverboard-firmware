//! General config-field writer over the SWD mailbox: stage a whole board-layout preset (or any
//! registered field) on the running master, then confirm each write with a `CONFIG_READ`.
//!
//! Usage: `swd-mailbox-config <openocd-host:port> [--base HEX] FIELD[:INDEX]=VALUE ...`
//!
//! - `FIELD` is a `0x`-hex or decimal store field id (e.g. `0x48` = `imu.scl_pin`).
//! - `INDEX` (optional, default 0) is the per-motor index (`motor.hall_a` on motor 1 = `0x4A:1`).
//! - `VALUE` is parsed against the field's REGISTERED type (`crates/store`; a packed `port|pin`
//!   byte such as `0x16` = PB6 is a `u8` field). No board-model validation happens here: a
//!   duplicate pin or a gate-capable pin in a LED field is a well-typed write this tool passes
//!   through, so the FIRMWARE's boot validator (`crates/board`) is what judges the layout after a
//!   reboot.
//!
//! Example (the silicon-queue section-6 VALID-layout case: the standard-family IMU on PB6/PB7
//! with the port freed in `LINK_SET`, then reboot to have the board validate it):
//!
//! ```text
//! swd-mailbox-config 127.0.0.1:6666 0x02=0x06 0x48=0x16 0x49=0x17 0x60=2
//! #   LINK_SET(0x02)=0b110 (inter-board+BLE live, PB6/PB7 port freed)
//! #   imu.scl_pin(0x48)=PB6  imu.sda_pin(0x49)=PB7  imu.model(0x60)=2
//! # then REBOOT the board (bench: power-cycle via the relay, or the L3 REBOOT opcode once it
//! # exists) and read BOARD_OBS: expect magic "BRDV", result 0 (OBS_OK).
//! ```
//!
//! The tool writes + confirms only; the reboot is the operator's next step (the validator runs at
//! boot). A wrong-type value, an unknown field id, or a bad argument is rejected before any wire
//! traffic.

use std::process::ExitCode;
use std::time::Duration;

use net::walk::CFG_OK;
use store::{Key, Type, Value};
use swd_bridge::config::{parse_field_arg, parse_field_value};
use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::walk::WalkDriver;
use swd_bridge::{HostMailbox, MAILBOX_BASE};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("FAIL: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let endpoint = args
        .next()
        .ok_or("usage: swd-mailbox-config <host:port> [--base HEX] FIELD[:INDEX]=VALUE ...")?;

    // Optional --base, then the field arguments. Parse+type-check EVERYTHING before any wire
    // traffic, so a typo fails fast without half-staging a layout.
    let mut base = MAILBOX_BASE;
    let mut raw_args: Vec<String> = Vec::new();
    let rest: Vec<String> = args.collect();
    let mut it = rest.into_iter();
    while let Some(a) = it.next() {
        if a == "--base" {
            let b = it.next().ok_or("--base needs a hex value")?;
            base = u32::from_str_radix(b.trim_start_matches("0x"), 16)
                .map_err(|_| format!("bad --base {b:?}"))?;
        } else {
            raw_args.push(a);
        }
    }
    if raw_args.is_empty() {
        return Err("no FIELD[:INDEX]=VALUE arguments".into());
    }
    // (field_id, index, value_str) for each; value_str borrows raw_args for the Str case.
    let mut fields: Vec<(u8, u8, &str)> = Vec::new();
    for a in &raw_args {
        let parsed = parse_field_arg(a).map_err(|e| e.to_string())?;
        // Pre-validate the value against the type here too (fail before the wire).
        parse_field_value(parsed.0, parsed.2).map_err(|e| e.to_string())?;
        fields.push(parsed);
    }

    // Attach the mailbox + walk once to learn the master's address.
    let mem = OpenOcdTcl::connect(&endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200)
        .map_err(|_| "firmware never wrote epoch_ack (no poll-site running?)".to_string())?;

    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;
    let master = walk
        .master_addr()
        .ok_or("walk assigned no address (is the firmware running?)")?;
    println!("staging {} field(s) on master 0x{master:02x}", fields.len());

    // Write each field, then read it back and confirm.
    let mut failures = 0;
    for (field_id, index, value_str) in fields {
        let key = Key { field_id, index };
        let value = parse_field_value(field_id, value_str).map_err(|e| e.to_string())?;

        let w = walk
            .config_write(master, key, value, Duration::from_secs(10))
            .map_err(|e| e.to_string())?;
        if w.status != CFG_OK {
            eprintln!(
                "  {field_id:#04x}:{index} = {value_str}: WRITE status {} != OK",
                w.status
            );
            failures += 1;
            continue;
        }
        let r = walk
            .config_read(master, key, Duration::from_secs(10))
            .map_err(|e| e.to_string())?;
        if r.status != CFG_OK {
            eprintln!(
                "  {field_id:#04x}:{index} = {value_str}: READ status {} != OK",
                r.status
            );
            failures += 1;
            continue;
        }
        // Confirm the readback equals what we wrote (re-parse for the comparison value).
        let want = parse_field_value(field_id, value_str).map_err(|e| e.to_string())?;
        let kind = Type::from_tag(r.type_tag).ok_or("CONFIG_RESP bad type tag")?;
        let got = Value::decode(kind, &r.value);
        if got.as_ref() == Some(&want) {
            println!("  {field_id:#04x}:{index} = {value_str}  OK (write -> read matches)");
        } else {
            eprintln!("  {field_id:#04x}:{index} = {value_str}: readback {got:?} != {want:?}");
            failures += 1;
        }
    }

    if failures == 0 {
        println!("PASS: staged {} field(s); REBOOT the board to have its boot validator judge the layout", raw_args.len());
        Ok(())
    } else {
        Err(format!("{failures} field(s) failed to stage"))
    }
}
