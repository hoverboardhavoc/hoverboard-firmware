//! l3 Tier 2 (the final tier): the host controller drives the discovery walk that reaches the SLAVE
//! THROUGH the master over the inter-board UART, then a two-hop `CONFIG` to the slave, and (after a
//! cold-cycle) confirms reboot recovery (`specs/l3.md`, "Tier 2"). All over the master's SWD mailbox.
//!
//! Usage:
//!   `l3-tier2 <openocd-host:port> [base-hex]`              full run: walk + two-hop slave CONFIG
//!   `l3-tier2 <openocd-host:port> --config-only [base-hex]` post-reboot: CONFIG the slave WITHOUT a
//!                                                           walk (the persisted addresses + a learned
//!                                                           route must make it work)
//!
//! Full run:
//!   1. attach + the epoch_ack handshake;
//!   2. the walk: master gets `0x01`, the slave is found on the inter-board port and gets `0x02`
//!      (a directed `ASSIGN` forwarded out the UART; the slave's `ASSIGN_ACK` source-routes back);
//!   3. a two-hop `CONFIG_WRITE`/`CONFIG_READ` to the SLAVE (`0x02`): the request floods/forwards to
//!      the slave over the UART, the slave persists + replies, and the reply source-routes back;
//!   4. a `CONFIG` to the master (`0x01`) + an independent master flash readback of `node_address`.
//!
//! `--config-only` (run AFTER a GPIO42 cold-cycle of both boards): SKIP the walk and `CONFIG` the
//! slave `0x02` (and master `0x01`) directly. Success proves both boards re-read their `node_address`
//! from flash on reboot and the master re-learned the route to the slave from the flooded request,
//! WITHOUT re-running discovery.

use std::process::ExitCode;
use std::time::Duration;

use net::walk::CFG_OK;
use store::{Type, Value, MOTOR_CURRENT_LIMIT, NODE_ADDRESS};
use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::walk::{mount_store_image, CfgResp, ImageFlash, WalkDriver};
use swd_bridge::{HostMailbox, MemAp, MAILBOX_BASE};

// The F103 master's store region: the wired master's part parameters (64 KiB flash, 1 KiB pages)
// through the placement rule's one owner, `store::geometry` (= 0x0800_F800, the top two pages).
const F103_FLASH_SIZE: u32 = 64 * 1024;
const STORE_PAGE: usize = 1024;
const STORE_BASE: u32 = store::geometry::store_base(F103_FLASH_SIZE, STORE_PAGE as u32);
const STORE_LEN: usize = store::geometry::region_len(STORE_PAGE);

const MASTER: u8 = 0x01;
const SLAVE: u8 = 0x02;
const CFG_TIMEOUT: Duration = Duration::from_secs(10);

fn main() -> ExitCode {
    let mut endpoint: Option<String> = None;
    let mut base = MAILBOX_BASE;
    let mut config_only = false;
    for arg in std::env::args().skip(1) {
        if arg == "--config-only" {
            config_only = true;
        } else if let Some(h) = arg.strip_prefix("0x") {
            base = u32::from_str_radix(h, 16).unwrap_or(MAILBOX_BASE);
        } else if endpoint.is_none() {
            endpoint = Some(arg);
        }
    }
    let Some(endpoint) = endpoint else {
        eprintln!("usage: l3-tier2 <openocd-host:port> [--config-only] [base-hex]");
        return ExitCode::FAILURE;
    };

    match run(&endpoint, base, config_only) {
        Ok(()) => {
            println!(
                "PASS: l3 Tier 2 ({})",
                if config_only {
                    "reboot recovery"
                } else {
                    "walk + two-hop CONFIG"
                }
            );
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("FAIL: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(endpoint: &str, base: u32, config_only: bool) -> Result<(), String> {
    let mem = OpenOcdTcl::connect(endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200).map_err(|_| {
        "firmware never wrote epoch_ack == epoch (no poll-site running?)".to_string()
    })?;
    println!(
        "attach: epoch -> {}, epoch_ack matched (handshake complete)",
        host.session_epoch()
    );

    let mut walk = WalkDriver::new(host);

    if !config_only {
        // The full walk: master 0x01, slave 0x02 (the two-hop ASSIGN over the inter-board UART).
        walk.run_walk(Duration::from_secs(30))
            .map_err(|e| e.to_string())?;
        let master = walk
            .master_addr()
            .ok_or_else(|| "walk assigned no address".to_string())?;
        if master != MASTER {
            return Err(format!(
                "master assigned 0x{master:02x}, expected 0x{MASTER:02x}"
            ));
        }
        println!(
            "walk complete: master 0x{master:02x} (controller guest 0x{:02x})",
            walk.guest_addr()
        );
    } else {
        println!(
            "config-only: skipping the walk (relying on persisted addresses + route re-learning)"
        );
    }

    // Two-hop CONFIG to the SLAVE (0x02): the request reaches the slave THROUGH the master over the
    // inter-board UART (a learned route, or a flood when the route is not yet learned), the slave
    // persists + replies, and the reply source-routes back to the host.
    config_roundtrip(&mut walk, SLAVE, 21_000)?;
    println!("two-hop CONFIG to slave 0x{SLAVE:02x}: write 21000 -> read 21000 (reached + replied through the master)");

    // And the master itself (one hop) for completeness.
    config_roundtrip(&mut walk, MASTER, 17_000)?;
    println!("CONFIG to master 0x{MASTER:02x}: write 17000 -> read 17000");

    if !config_only {
        // Independent confirmation of the MASTER's persisted node_address (read its flash over SWD).
        let mut image = vec![0u8; STORE_LEN];
        walk.mem()
            .read(STORE_BASE, &mut image)
            .map_err(|e| e.to_string())?;
        let persisted = mount_store_image(ImageFlash::new(STORE_PAGE, image), NODE_ADDRESS.key())
            .map_err(|e| e.to_string())?;
        match persisted {
            Value::U8(a) if a == MASTER => {
                println!("flash readback: master node_address = 0x{a:02x} (matches the walk)")
            }
            other => {
                return Err(format!(
                    "persisted master node_address {other:?} != assigned 0x{MASTER:02x}"
                ))
            }
        }
        // The slave's persistence is confirmed independently: a `--config-only` run AFTER a cold-cycle
        // reaches the slave as 0x02, which it can only be if it re-read 0x02 from its own flash.
    }

    Ok(())
}

/// `CONFIG_WRITE(dst, MOTOR_CURRENT_LIMIT, value)` then `CONFIG_READ` it back; verify the round-trip.
fn config_roundtrip<M: MemAp>(walk: &mut WalkDriver<M>, dst: u8, value: u32) -> Result<(), String> {
    let key = MOTOR_CURRENT_LIMIT.key();
    let w = walk
        .config_write(dst, key, Value::U32(value), CFG_TIMEOUT)
        .map_err(|e| format!("CONFIG_WRITE to 0x{dst:02x}: {e}"))?;
    check_ok(&w, dst, "WRITE")?;
    let r = walk
        .config_read(dst, key, CFG_TIMEOUT)
        .map_err(|e| format!("CONFIG_READ from 0x{dst:02x}: {e}"))?;
    check_ok(&r, dst, "READ")?;
    let kind = Type::from_tag(r.type_tag).ok_or_else(|| "CONFIG_RESP bad type tag".to_string())?;
    match Value::decode(kind, &r.value) {
        Some(Value::U32(v)) if v == value => Ok(()),
        other => Err(format!(
            "0x{dst:02x} CONFIG_READ value {other:?} != U32({value})"
        )),
    }
}

fn check_ok(resp: &CfgResp, dst: u8, what: &str) -> Result<(), String> {
    if resp.status != CFG_OK {
        return Err(format!(
            "0x{dst:02x} CONFIG_{what} status {} != OK",
            resp.status
        ));
    }
    Ok(())
}
