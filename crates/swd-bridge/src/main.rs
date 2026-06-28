//! Bench validation for the SWD mailbox bridge (Tier-2 step 2): drive the **transport** against the
//! step-1 firmware on the master over openocd's MEM-AP, core running.
//!
//! Usage: `swd-mailbox-bridge <openocd-host:port> [base-hex]`
//!
//! The step-1 firmware has no mailbox poll-site yet (that is step 3), so it does **not** ack the epoch
//! or echo. So this proves the transport alone: attach (validate magic, bump epoch, discard stale
//! outbound), then write bytes into `h2t` and confirm with a second MEM-AP read that `h2t_head`
//! advanced and the bytes landed; and read/write `t2h` symmetrically (the host plays stand-in producer
//! into `t2h`, the bridge consumes). The full L2-frame round-trip waits on step 3 (an echo in the loop).

use std::process::ExitCode;

use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::{HostMailbox, MemAp, MAILBOX_BASE};
use swd_mailbox::layout;
use swd_mailbox::{H2T_DATA_OFF, T2H_DATA_OFF};

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let endpoint = match args.next() {
        Some(e) => e,
        None => {
            eprintln!("usage: swd-mailbox-bridge <openocd-host:port> [base-hex]");
            return ExitCode::FAILURE;
        }
    };
    let base = match args.next() {
        Some(s) => u32::from_str_radix(s.trim_start_matches("0x"), 16).unwrap_or(MAILBOX_BASE),
        None => MAILBOX_BASE,
    };

    match run(&endpoint, base) {
        Ok(()) => {
            println!("PASS: bridge transport validated on silicon (core running)");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("FAIL: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(endpoint: &str, base: u32) -> Result<(), String> {
    let mem = OpenOcdTcl::connect(endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);

    // 1. Validate the header the firmware initialized at boot.
    let magic = host.magic().map_err(|e| e.to_string())?;
    let version = host.version().map_err(|e| e.to_string())?;
    println!(
        "header @ {base:#010x}: magic={magic:#010x} (\"MBX1\"={:#010x}) version={version}",
        swd_mailbox::MAGIC
    );
    host.validate().map_err(|e| e.to_string())?;

    // 2. Attach: bump epoch, discard stale outbound. (No ack wait: the step-1 firmware has no poll-site.)
    let epoch_before = host.epoch().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    let epoch_after = host.epoch().map_err(|e| e.to_string())?;
    println!(
        "attach: epoch {epoch_before} -> {epoch_after} (session {})",
        host.session_epoch()
    );
    if epoch_after != epoch_before.wrapping_add(1) {
        return Err(format!(
            "epoch did not bump: {epoch_before} -> {epoch_after}"
        ));
    }
    if host.t2h_used().map_err(|e| e.to_string())? != 0 {
        return Err("t2h not flushed on attach".into());
    }

    // 3. h2t: produce bytes, confirm with a SECOND MEM-AP read that h2t_head advanced and they landed.
    let pattern: [u8; 6] = [0x5A, 0xA5, 0xDE, 0xAD, 0xBE, 0xEF];
    let head_before = host.h2t_head().map_err(|e| e.to_string())?;
    let h2t_tail = host.h2t_tail().map_err(|e| e.to_string())?;
    let n = host.produce(&pattern).map_err(|e| e.to_string())?;
    if n != pattern.len() {
        return Err(format!("produce wrote {n} of {} bytes", pattern.len()));
    }
    // Second, independent read of the index and the ring slots.
    let head_after = host
        .mem()
        .read32(base + layout::H2T_HEAD as u32)
        .map_err(|e| e.to_string())?;
    if head_after != head_before.wrapping_add(n as u32) {
        return Err(format!(
            "h2t_head {head_before} -> {head_after}, expected +{n}"
        ));
    }
    let slot = h2t_tail & (256 - 1); // bytes were written at the producer head == tail (ring empty)
    let mut readback = [0u8; 6];
    host.mem()
        .read(base + H2T_DATA_OFF as u32 + slot, &mut readback)
        .map_err(|e| e.to_string())?;
    if readback != pattern {
        return Err(format!(
            "h2t readback {readback:02x?} != written {pattern:02x?}"
        ));
    }
    println!("h2t: produced {n} bytes, h2t_head {head_before} -> {head_after}, ring bytes match");

    // 4. t2h symmetric: the host plays stand-in producer (write bytes + advance t2h_head over MEM-AP),
    //    the bridge consumes them. Proves the consume path + MEM-AP write/read of t2h.
    let t2h_pattern: [u8; 4] = [0x11, 0x22, 0x33, 0x44];
    let t2h_head0 = host.t2h_head().map_err(|e| e.to_string())?;
    let t2h_slot = t2h_head0 & (256 - 1);
    {
        let mem = host.mem();
        mem.write(base + T2H_DATA_OFF as u32 + t2h_slot, &t2h_pattern)
            .map_err(|e| e.to_string())?; // data first
        mem.write32(
            base + layout::T2H_HEAD as u32,
            t2h_head0.wrapping_add(t2h_pattern.len() as u32),
        )
        .map_err(|e| e.to_string())?; // then the producer head
    }
    let mut dst = [0u8; 8];
    let k = host.consume(&mut dst).map_err(|e| e.to_string())?;
    if dst[..k] != t2h_pattern {
        return Err(format!(
            "t2h consume {:02x?} != written {t2h_pattern:02x?}",
            &dst[..k]
        ));
    }
    if host.t2h_used().map_err(|e| e.to_string())? != 0 {
        return Err("t2h_tail did not advance to drain".into());
    }
    println!(
        "t2h: stand-in producer wrote {} bytes, bridge consumed them, t2h drained",
        t2h_pattern.len()
    );

    Ok(())
}
