//! swd-mailbox Tier-2 round-trip (step 3): drive the L3 walk + `CONFIG_*` into the master over the
//! mailbox, **core running, no halt**, and independently confirm the persisted `node_address`.
//!
//! Usage: `swd-mailbox-walk <openocd-host:port> [base-hex] [config-value]`
//!
//! `config-value` (decimal, default 15000) is the `MOTOR_CURRENT_LIMIT` value round-tripped in
//! step 3: bench re-runs pass a FRESH value so persisted state from a prior run cannot fake the
//! round-trip (the bench-session runbook rule).
//!
//! 1. attach + the **epoch_ack handshake** end to end (the firmware now flushes + writes `epoch_ack`,
//!    the bridge waits for it);
//! 2. the walk: first-contact `NODE_HELLO` (identity + guest grant) -> `ASSIGN` (master persists its
//!    `node_address`) -> `PROBE_PORTS` (only the upstream mailbox port, no downstream neighbour yet);
//! 3. `CONFIG_WRITE` / `CONFIG_READ` of a field, round-tripped over the mailbox;
//! 4. independent confirmation: read the store region back over SWD, mount it, and check the persisted
//!    `node_address` equals the address the walk assigned.

use std::process::ExitCode;
use std::time::Duration;

use net::walk::CFG_OK;
use store::{Type, Value, MOTOR_CURRENT_LIMIT, NODE_ADDRESS};
use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::walk::{mount_store_image, ImageFlash, WalkDriver};
use swd_bridge::{HostMailbox, MemAp, MAILBOX_BASE};

// The F103 master's store region: the wired master's part parameters (64 KiB flash, 1 KiB pages)
// through the placement rule's one owner, `store::geometry` (= 0x0800_F800, the top two pages).
const F103_FLASH_SIZE: u32 = 64 * 1024;
const STORE_PAGE: usize = 1024;
const STORE_BASE: u32 = store::geometry::store_base(F103_FLASH_SIZE, STORE_PAGE as u32);
const STORE_LEN: usize = store::geometry::region_len(STORE_PAGE);

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let endpoint = match args.next() {
        Some(e) => e,
        None => {
            eprintln!("usage: swd-mailbox-walk <openocd-host:port> [base-hex]");
            return ExitCode::FAILURE;
        }
    };
    let base = args
        .next()
        .and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok())
        .unwrap_or(MAILBOX_BASE);
    let config_value = args
        .next()
        .and_then(|s| s.parse::<u32>().ok())
        .unwrap_or(15_000);

    match run(&endpoint, base, config_value) {
        Ok(()) => {
            println!(
                "PASS: swd-mailbox Tier-2 round-trip (walk + CONFIG + persisted node_address)"
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("FAIL: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(endpoint: &str, base: u32, config_value: u32) -> Result<(), String> {
    let mem = OpenOcdTcl::connect(endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;

    // 1. Attach + the epoch_ack handshake (the firmware's poll-site flushes + acks).
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200) // ~2 s budget
        .map_err(|_| {
            "firmware never wrote epoch_ack == epoch (no poll-site running?)".to_string()
        })?;
    println!(
        "attach: epoch -> {}, epoch_ack matched (handshake complete)",
        host.session_epoch()
    );

    // 2. The walk.
    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;
    let master = walk
        .master_addr()
        .ok_or_else(|| "walk assigned no address".to_string())?;
    println!(
        "walk complete: master assigned 0x{master:02x} (controller guest 0x{:02x})",
        walk.guest_addr()
    );

    // 3. CONFIG_WRITE / CONFIG_READ a field over the mailbox.
    let key = MOTOR_CURRENT_LIMIT.key();
    let w = walk
        .config_write(
            master,
            key,
            Value::U32(config_value),
            Duration::from_secs(10),
        )
        .map_err(|e| e.to_string())?;
    if w.status != CFG_OK {
        return Err(format!("CONFIG_WRITE status {} != OK", w.status));
    }
    let r = walk
        .config_read(master, key, Duration::from_secs(10))
        .map_err(|e| e.to_string())?;
    if r.status != CFG_OK {
        return Err(format!("CONFIG_READ status {} != OK", r.status));
    }
    let kind = Type::from_tag(r.type_tag).ok_or_else(|| "CONFIG_RESP bad type tag".to_string())?;
    match Value::decode(kind, &r.value) {
        Some(Value::U32(v)) if v == config_value => {
            println!("CONFIG round-trip: MOTOR_CURRENT_LIMIT write {config_value} -> read {v}")
        }
        other => {
            return Err(format!(
                "CONFIG_READ value {other:?} != U32({config_value})"
            ))
        }
    }

    // 4. Independent confirmation: read the store region back over SWD, mount it, read node_address.
    let mut image = vec![0u8; STORE_LEN];
    walk.mem()
        .read(STORE_BASE, &mut image)
        .map_err(|e| e.to_string())?;
    let persisted = mount_store_image(ImageFlash::new(STORE_PAGE, image), NODE_ADDRESS.key())
        .map_err(|e| e.to_string())?;
    match persisted {
        Value::U8(a) if a == master => {
            println!("flash readback: persisted node_address = 0x{a:02x} (matches the walk)")
        }
        other => {
            return Err(format!(
                "persisted node_address {other:?} != assigned 0x{master:02x}"
            ))
        }
    }

    Ok(())
}
