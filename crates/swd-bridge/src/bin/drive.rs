//! Command a **drive demand** on an addressed board over the SWD mailbox: the payload that
//! actually reaches the throttle-mode reference producer.
//!
//! Usage: `swd-mailbox-drive <openocd-host:port> [--base HEX] [--dst attached|ADDR]
//!         [--value N] [--steer N] [--hold SECONDS]`
//!
//! # Why this exists (and why `swd-mailbox-inputs --throttle` is not it)
//!
//! Two different words are called "throttle" on this link and only one of them drives anything:
//!
//! - `DRIVE_CMD` (`linkctl::OP_DRIVE_CMD` = 0x11) carries the demand the control task conditions
//!   into the reference the engagement machine envelopes. **This is the one that moves a wheel.**
//! - `INPUTS.throttle` (0x12) is the raw ADC-mirror word from a board's own throttle hardware. It
//!   is filtered into `throttle_filtered` and, today, nothing consumes it.
//!
//! The 2026-07-31 arm session tried to command its first motion with `--throttle` on the INPUTS
//! tool. Even with the engagement gate fixed, that word could not have moved anything.
//!
//! # Holding a demand
//!
//! `DRIVE_CMD` is best-effort / latest-wins and **decays**: with no fresh command for
//! `linkctl::DRIVE_TIMEOUT_TICKS` (50 ticks = 200 ms) the firmware zeroes the reference. So a
//! single send is a 200 ms blip, not a demand. This tool re-sends every 100 ms for `--hold`
//! seconds, then sends an explicit `Neutral` and exits.
//!
//! That decay is the safety property, not an inconvenience: kill this tool, unplug the host, lose
//! the link, and the demand is gone within 200 ms without anything having to notice. `--hold` is
//! bounded for the same reason; there is no "hold forever" mode.
//!
//! Arming is separate and is a LEVEL the firmware holds: `swd-mailbox-inputs --buttons 1` first,
//! this second.

use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};

use linkctl::{DriveCmd, DriveKind, OP_DRIVE_CMD};
use net::Pdu;
use swd_bridge::openocd::OpenOcdTcl;
use swd_bridge::walk::{host_link_note, WalkDriver};
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

const USAGE: &str = "usage: swd-mailbox-drive <host:port> [--base HEX] [--dst attached|ADDR] \
     [--value N] [--steer N] [--hold SECONDS]";

/// The re-send period. Half the firmware's 200 ms staleness window, so a dropped frame still
/// leaves one more send inside the window.
const RESEND: Duration = Duration::from_millis(100);

/// The longest `--hold` this tool accepts. A bench demand is a bounded act; a longer run is a
/// deliberate decision to re-issue the command, not a flag value.
const MAX_HOLD_SECS: u64 = 60;

/// Encode a `DRIVE_CMD` L3 PDU (opcode `0x11`) carrying `cmd`, from `src` to `dst`. The payload
/// bytes come from `linkctl::DriveCmd::encode` (the canonical owner); this only wraps them in the
/// L3 header.
fn encode_drive_pdu(src: u8, dst: u8, cmd: &DriveCmd) -> Vec<u8> {
    let mut payload = [0u8; DriveCmd::LEN];
    cmd.encode(&mut payload);
    let pdu = Pdu::new(OP_DRIVE_CMD, src, dst, &payload).expect("OP_DRIVE_CMD is a valid opcode");
    let mut buf = [0u8; net::pdu::HEADER_LEN + DriveCmd::LEN];
    let n = pdu
        .encode(&mut buf)
        .expect("buf fits L3 header + DRIVE_CMD payload");
    buf[..n].to_vec()
}

/// Parse a `0x`-hex or decimal `u8` (addresses).
fn parse_u8(s: &str) -> Result<u8, String> {
    let r = s
        .strip_prefix("0x")
        .map(|h| u8::from_str_radix(h, 16))
        .unwrap_or_else(|| s.parse::<u8>());
    r.map_err(|_| format!("bad u8 value {s:?}"))
}

/// Parse a decimal `i16` demand word (the +-32767 frame scale).
fn parse_i16(s: &str) -> Result<i16, String> {
    s.parse::<i16>().map_err(|_| format!("bad i16 value {s:?}"))
}

/// How `--dst` was given (mirrors `swd-mailbox-inputs`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DstArg {
    /// Resolve the attached node from the walk.
    Attached,
    /// A literal address.
    Explicit(u8),
}

fn parse_dst(s: &str) -> Result<DstArg, String> {
    if s.eq_ignore_ascii_case("attached") {
        return Ok(DstArg::Attached);
    }
    parse_u8(s)
        .map(DstArg::Explicit)
        .map_err(|_| format!("bad --dst value {s:?} (want `attached` or an address like 0x02)"))
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let endpoint = args.next().ok_or(USAGE)?;

    let mut base = MAILBOX_BASE;
    let mut dst = DstArg::Attached;
    let mut value: i16 = 0;
    let mut steer: i16 = 0;
    let mut hold_secs: u64 = 5;

    let mut it = args;
    while let Some(a) = it.next() {
        let mut val = || it.next().ok_or_else(|| format!("{a} needs a value"));
        match a.as_str() {
            "--base" => {
                let b = val()?;
                base = u32::from_str_radix(b.trim_start_matches("0x"), 16)
                    .map_err(|_| format!("bad --base {b:?}"))?;
            }
            "--dst" => dst = parse_dst(&val()?)?,
            "--value" => value = parse_i16(&val()?)?,
            "--steer" => steer = parse_i16(&val()?)?,
            "--hold" => {
                let h = val()?;
                hold_secs = h.parse::<u64>().map_err(|_| format!("bad --hold {h:?}"))?;
                if hold_secs == 0 || hold_secs > MAX_HOLD_SECS {
                    return Err(format!("--hold must be 1..={MAX_HOLD_SECS} seconds"));
                }
            }
            other => return Err(format!("unknown argument {other:?}\n{USAGE}")),
        }
    }

    // Attach the mailbox + walk to bring the L3 link up and learn the fleet's addresses.
    let mem = OpenOcdTcl::connect(&endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200)
        .map_err(|_| "firmware never wrote epoch_ack (no poll-site running?)".to_string())?;

    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;

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
                    "dst 0x{a:02x} (explicit; NOTE the attached node is 0x{r:02x}, so this drives \
                     a board reached THROUGH it)"
                ),
                None => println!("dst 0x{a:02x} (explicit; the walk resolved no attached node)"),
            }
            a
        }
    };
    let src = walk.guest_addr();

    let live = DriveCmd {
        kind: DriveKind::Throttle,
        value,
        steer,
    };
    let pdu = encode_drive_pdu(src, dst, &live);
    println!(
        "DRIVE 0x{src:02x}->0x{dst:02x}: value={value} steer={steer}, held for {hold_secs} s \
         (re-sent every {} ms; the firmware decays to neutral {} ms after the last one)",
        RESEND.as_millis(),
        linkctl::DRIVE_TIMEOUT_TICKS * 4,
    );
    println!("  PDU bytes: {pdu:02x?}");

    let deadline = Instant::now() + Duration::from_secs(hold_secs);
    let mut sends = 0u32;
    while Instant::now() < deadline {
        walk.send_pdu(&pdu).map_err(|e| e.to_string())?;
        sends += 1;
        sleep(RESEND);
    }

    // Release explicitly rather than leaning on the decay: the demand is zero before this process
    // exits, and the decay stays the backstop for the ways a tool does NOT get to exit cleanly.
    let neutral = DriveCmd {
        kind: DriveKind::Neutral,
        value: 0,
        steer: 0,
    };
    walk.send_pdu(&encode_drive_pdu(src, dst, &neutral))
        .map_err(|e| e.to_string())?;

    println!("sent {sends} DRIVE frames, then an explicit Neutral");
    println!("PASS: demand released (the reference is zero; the board stays ARMED until INPUTS clears it)");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_a_live_throttle_pdu() {
        // L3 header [op, src, dst] then the payload [kind, value_lo, value_hi, steer_lo, steer_hi].
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: 1000,
            steer: 0,
        };
        let pdu = encode_drive_pdu(0x80, 0x02, &cmd);
        assert_eq!(pdu, vec![0x11, 0x80, 0x02, 0x01, 0xe8, 0x03, 0x00, 0x00]);
    }

    #[test]
    fn encodes_the_neutral_release() {
        let cmd = DriveCmd {
            kind: DriveKind::Neutral,
            value: 0,
            steer: 0,
        };
        let pdu = encode_drive_pdu(0x80, 0x02, &cmd);
        assert_eq!(pdu, vec![0x11, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn encodes_a_negative_value_little_endian() {
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: -1000,
            steer: 500,
        };
        let pdu = encode_drive_pdu(0x81, 0x01, &cmd);
        assert_eq!(pdu, vec![0x11, 0x81, 0x01, 0x01, 0x18, 0xfc, 0xf4, 0x01]);
    }

    #[test]
    fn the_resend_period_stays_inside_the_firmware_decay_window() {
        // The tool's contract with the firmware: a held demand must never lapse between sends.
        // 50 ticks at 250 Hz = 200 ms; two re-sends fit inside it, so one lost frame is survivable.
        let window_ms = (linkctl::DRIVE_TIMEOUT_TICKS as u128) * 4;
        assert!(RESEND.as_millis() * 2 <= window_ms, "{window_ms} ms window");
    }

    #[test]
    fn parse_dst_accepts_the_word_and_an_address() {
        assert_eq!(parse_dst("attached"), Ok(DstArg::Attached));
        assert_eq!(parse_dst("ATTACHED"), Ok(DstArg::Attached));
        assert_eq!(parse_dst("0x02"), Ok(DstArg::Explicit(2)));
        assert!(parse_dst("0x1ff").is_err());
    }
}
