//! Command a **drive demand** on an addressed board over the SWD mailbox: the payload that
//! actually reaches the throttle-mode reference producer.
//!
//! Usage: `swd-mailbox-drive <openocd-host:port> [--base HEX] [--dst attached|ADDR]
//!         [--value N] [--steer N] [--hold SECONDS] [--period-ms N] [--trace]`
//!
//! # Why this exists (and why `swd-mailbox-inputs --throttle` is not it)
//!
//! Two different words are called "throttle" on this link and only one of them drives anything:
//!
//! - `DRIVE_CMD` (`linkctl::OP_DRIVE_CMD` = 0x11) carries the demand the control task conditions
//!   into the reference the engagement machine envelopes. **This is the one that moves a wheel.**
//! - `INPUTS` (0x12) carries only the levels a controller asserts about itself (power request,
//!   rider present). It used to lead with a raw ADC-mirror throttle word, which nothing consumed
//!   and which is now deleted (`specs/todo.md` part 3).
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
//! `--period-ms` sets the re-send period (default 100 ms, the hold's own cadence; 50 ms is the
//! rider app's 20 Hz pump). It is capped so two sends always fit inside the decay window, which
//! is what makes a held demand a demand. `--trace` prints one line per frame with the wall-clock
//! time (UNIX ns) just before and just after the mailbox write, so a second observer sampling the
//! destination's `drive_age` over SWD can put a latency on the relayed path
//! (`specs/todo.md` part 1, the two-hop `DRIVE_CMD` latency).
//!
//! Arming is separate, and it expires the same way this demand does, only slower: the `INPUTS`
//! mirror carrying `power_request` ages out after `linkctl::INPUTS_TIMEOUT_TICKS` (1.5 s), so the
//! arming command has to be HELD too. Run `swd-mailbox-inputs --buttons 1 --hold SECS` in another
//! terminal first, and this second, inside that hold.
//!
//! This tool's demand does NOT extend that hold, and should not be expected to. The firmware
//! refreshes the mirror on any `INPUTS` or `DRIVE_CMD` **from the node that stated the level**,
//! and each of these tools walks its own first contact and is granted its own guest address, so
//! to the board they are two different controllers. The rider app is one process holding one
//! address, which is the case the rule is for; the bench pair is deliberately two, and the
//! `inputs --hold` side is what keeps the arm alive on its own re-sends.

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
     [--value N] [--steer N] [--hold SECONDS] [--period-ms N] [--trace]";

/// The default re-send period. Half the firmware's 200 ms staleness window, so a dropped frame
/// still leaves one more send inside the window.
const RESEND: Duration = Duration::from_millis(100);

/// The firmware's demand decay window in milliseconds (`DRIVE_TIMEOUT_TICKS` at 250 Hz).
const DECAY_MS: u64 = linkctl::DRIVE_TIMEOUT_TICKS as u64 * 4;

/// The longest `--period-ms` this tool accepts: two sends must fit inside the decay window, the
/// same property the default satisfies, so a dropped frame still leaves one send in the window.
const MAX_PERIOD_MS: u64 = DECAY_MS / 2;

/// The shortest `--period-ms`: below this the SWD mailbox write itself takes the whole period.
const MIN_PERIOD_MS: u64 = 10;

/// Wall-clock time as UNIX nanoseconds, the clock a second host process can share.
fn unix_ns() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

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
    let mut period = RESEND;
    let mut trace = false;

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
            "--period-ms" => {
                let p = val()?;
                let ms = p
                    .parse::<u64>()
                    .map_err(|_| format!("bad --period-ms {p:?}"))?;
                if !(MIN_PERIOD_MS..=MAX_PERIOD_MS).contains(&ms) {
                    return Err(format!(
                        "--period-ms must be {MIN_PERIOD_MS}..={MAX_PERIOD_MS} (two sends inside \
                         the {DECAY_MS} ms decay window)"
                    ));
                }
                period = Duration::from_millis(ms);
            }
            "--trace" => trace = true,
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
         (re-sent every {} ms; the firmware decays to neutral {DECAY_MS} ms after the last one)",
        period.as_millis(),
    );
    println!("  PDU bytes: {pdu:02x?}");
    if trace {
        println!("  trace: send <n> <unix_ns before write> <unix_ns after write>");
    }

    let deadline = Instant::now() + Duration::from_secs(hold_secs);
    let mut sends = 0u32;
    while Instant::now() < deadline {
        let next = Instant::now() + period;
        let before = unix_ns();
        walk.send_pdu(&pdu).map_err(|e| e.to_string())?;
        let after = unix_ns();
        sends += 1;
        if trace {
            println!("send {sends} {before} {after}");
        }
        // Pace from the send's START so the mailbox write's own duration does not stretch the
        // period; a write slower than the period just runs the next send immediately.
        let now = Instant::now();
        if next > now {
            sleep(next - now);
        }
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
    println!(
        "PASS: demand released (the reference is zero; the board stays ARMED only while an INPUTS \
         hold keeps power_request fresh)"
    );
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
        // And the same bound caps what --period-ms may ask for.
        assert!((MAX_PERIOD_MS as u128) * 2 <= window_ms);
    }

    #[test]
    fn parse_dst_accepts_the_word_and_an_address() {
        assert_eq!(parse_dst("attached"), Ok(DstArg::Attached));
        assert_eq!(parse_dst("ATTACHED"), Ok(DstArg::Attached));
        assert_eq!(parse_dst("0x02"), Ok(DstArg::Explicit(2)));
        assert!(parse_dst("0x1ff").is_err());
    }
}
