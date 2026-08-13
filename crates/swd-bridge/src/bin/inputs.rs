//! Deliver an INPUTS L7 payload to an addressed board over the SWD mailbox, so a bench session can
//! drive the firmware's mode machine from the host.
//!
//! Usage: `swd-mailbox-inputs <openocd-host:port> [--base HEX] [--dst attached|ADDR]
//!         [--buttons BYTE] [--throttle N] [--rider BYTE] [--hold SECS]`
//!
//! The tool attaches the mailbox, runs the L3 walk (the same bring-up the config writer uses, so it
//! learns the fleet's addresses and confirms the firmware is live), then sends ONE INPUTS PDU
//! (`linkctl::OP_INPUTS` = 0x12) carrying the `linkctl::Inputs` payload (throttle i16, buttons u8,
//! rider u8, little-endian, encoded by `linkctl` -- the canonical owner of the bytes). The PDU is
//! delivered to `--dst`, `src` = the controller's guest address.
//!
//! # `--dst`: the attached node, resolved, never remembered
//!
//! The default (and `--dst attached` said out loud) is the board THIS host's mailbox link lands on,
//! resolved from the walk by `WalkDriver::attached_addr` and printed with the port-table evidence
//! behind it. An explicit `--dst 0xNN` still goes wherever it is told, which is what a deliberate
//! command to the far board wants.
//!
//! This tool ARMS BOARDS (`--buttons 1` asserts `power_request`), so its destination is a safety
//! fact, not a convenience. On 2026-07-31 the runbook carried a literal `--dst 0x01` from an era
//! when the master held that address; the master had since persisted `0x02`, `0x01` had been
//! allocated to the SLAVE, and the session's first power request armed the slave's bridge.
//!
//! **`--throttle` is NOT the word that moves a wheel.** `INPUTS.throttle` is the raw ADC-mirror
//! word from a board's own throttle hardware; it is filtered into `throttle_filtered` and nothing
//! consumes it today. The demand the control task conditions is `DRIVE_CMD` (0x11), which
//! `swd-mailbox-drive` sends. The 2026-07-31 arm session tried to command its first motion with
//! `--throttle` here; even with the engagement gate fixed, that word could not have moved
//! anything.
//!
//! # The mirror EXPIRES: a one-shot send arms for 1.5 s, not forever
//!
//! INPUTS is best-effort / latest-wins (`specs/link-control.md`): there is no reply, so the tool
//! encodes + sends and confirms the send. What the firmware does with it changed: the mirror now
//! ages, and past `linkctl::INPUTS_TIMEOUT_TICKS` (1.5 s) every level it carries reads as
//! released. A board armed by a single `--buttons 1` therefore disarms 1.5 s later, on its own.
//! That is the point of the timeout (a controller that goes away cannot leave a bridge live), and
//! it makes this tool a controller like any other: it holds a level by REPEATING it.
//!
//! `--hold SECS` does the repeating, re-sending the same payload every
//! [`RESEND_INTERVAL_MS`] over one already-attached link. A shell loop around the whole command
//! cannot substitute: each invocation re-attaches and re-runs the L3 walk, which takes longer than
//! the timeout it is trying to stay inside.
//!
//! Arming the armed CONFIG_WRITE gate (`--buttons 1` asserts power_request, walking the mode machine
//! OFF->INIT->READY->RUN so `any_moe_allowed` = armed); `--buttons 0` clears it immediately, and so
//! does Ctrl-C during a hold: the interrupt runs the same explicit all-clear a completed hold runs
//! (see [`INTERRUPTED`]). Only the exits this process does not get to handle (`kill -9`, a pulled
//! cable, a lost session) leave the release to the firmware's 1.5 s timeout, which is the backstop
//! that timeout exists to be:
//!
//! ```text
//! swd-mailbox-inputs 127.0.0.1:6666 --dst attached --buttons 1 --hold 300  # armed for 5 min
//! swd-mailbox-inputs 127.0.0.1:6666 --dst attached --buttons 1             # armed for 1.5 s
//! swd-mailbox-inputs 127.0.0.1:6666 --dst attached --buttons 0             # disarm now
//! ```

use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use linkctl::{Inputs, INPUTS_TIMEOUT_TICKS, OP_INPUTS};
use net::Pdu;
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

const USAGE: &str = "usage: swd-mailbox-inputs <host:port> [--base HEX] [--dst attached|ADDR] \
     [--buttons BYTE] [--throttle N] [--rider BYTE] [--hold SECS]";

/// The control-task period in ms (250 Hz), the unit `INPUTS_TIMEOUT_TICKS` counts in.
const CONTROL_TICK_MS: u64 = 4;

/// The re-send cadence while `--hold`ing: half the firmware's mirror timeout, derived from it
/// rather than written down again, so re-tuning the timeout cannot leave this behind. Half gives
/// the hold a whole missed send of margin before the board would release.
const RESEND_INTERVAL_MS: u64 = INPUTS_TIMEOUT_TICKS as u64 * CONTROL_TICK_MS / 2;

/// The number of re-sends that follow the first send over a `--hold SECS` request.
fn resends_for(hold_secs: u64) -> u64 {
    hold_secs.saturating_mul(1000) / RESEND_INTERVAL_MS
}

/// Set by the `SIGINT` handler, polled by the hold's sleep.
///
/// `--hold` keeps a board armed for exactly as long as this process keeps talking, so the ways it
/// STOPS are a safety fact, and Ctrl-C is the ordinary way an operator stops it: it means stop NOW.
/// With no handler the process dies mid-sleep and the release falls to the firmware's mirror
/// timeout ([`INPUTS_TIMEOUT_TICKS`], 1.5 s) - a real disarm, but a slower and less deliberate one
/// than the tool can do itself. The flag turns an interrupt into the SAME explicit all-clear a
/// completed hold sends. The timeout stays the backstop for the exits no handler can cover
/// (`kill -9`, a pulled cable, a lost session, this host dying).
static INTERRUPTED: AtomicBool = AtomicBool::new(false);

/// How often the hold's sleep looks at [`INTERRUPTED`]. `std::thread::sleep` cannot be woken, so
/// the interrupt is polled; 20 ms is imperceptible to an operator and free next to a 750 ms
/// re-send interval.
const INTERRUPT_POLL_MS: u64 = 20;

/// The `SIGINT` handler. Async-signal-safe by construction: an atomic store and `signal`, both on
/// POSIX's safe list, with no allocation, no I/O and no locks. The release itself runs on the main
/// thread, where sending a PDU over the mailbox is a normal thing to do.
extern "C" fn on_sigint(_sig: libc::c_int) {
    INTERRUPTED.store(true, Ordering::SeqCst);
    // Step aside so a SECOND Ctrl-C kills the process outright: the all-clear this interrupt
    // schedules is a write to openocd over TCP, and if that hangs the operator must still be able
    // to get out from the keyboard.
    // SAFETY: `signal` is async-signal-safe, and `SIG_DFL` is the disposition the process started
    // with.
    unsafe { libc::signal(libc::SIGINT, libc::SIG_DFL) };
}

/// Install [`on_sigint`], so Ctrl-C releases rather than abandons.
fn install_sigint_handler() {
    // SAFETY: `on_sigint` is a plain `extern "C"` fn with the `sighandler_t` signature, and does
    // only async-signal-safe work.
    // The two-step cast (fn item -> pointer -> integer) is what `sighandler_t` wants and what the
    // `function_casts_as_integer` lint requires; a direct fn-to-integer cast is denied.
    unsafe { libc::signal(libc::SIGINT, on_sigint as *const () as libc::sighandler_t) };
}

/// Sleep for `d` unless Ctrl-C arrives first. `true` = the whole interval elapsed, `false` = the
/// hold was interrupted and its caller should go release.
fn sleep_unless_interrupted(d: Duration) -> bool {
    let until = Instant::now() + d;
    loop {
        if INTERRUPTED.load(Ordering::SeqCst) {
            return false;
        }
        let left = until.saturating_duration_since(Instant::now());
        if left.is_zero() {
            return true;
        }
        std::thread::sleep(left.min(Duration::from_millis(INTERRUPT_POLL_MS)));
    }
}

/// The longest `--hold` this tool accepts (30 minutes). A bench session is a bounded act with a
/// person in front of it; the flag is a way to keep a board armed WHILE somebody watches it, not
/// a way to leave one armed. `swd-mailbox-drive` bounds its own hold for the same reason.
const MAX_HOLD_SECS: u64 = 1800;

/// How `--dst` was given. `Attached` (the default, and the literal `--dst attached`) resolves the
/// board the host is plugged into from the walk; `Explicit` goes where it is told.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DstArg {
    /// Resolve from the walk (`WalkDriver::attached_addr`).
    Attached,
    /// A literal address.
    Explicit(u8),
}

/// Parse a `--dst` value: the word `attached`, or an address (`0x02` / `2`).
fn parse_dst(s: &str) -> Result<DstArg, String> {
    if s.eq_ignore_ascii_case("attached") {
        return Ok(DstArg::Attached);
    }
    parse_u8(s)
        .map(DstArg::Explicit)
        .map_err(|_| format!("bad --dst value {s:?} (want `attached` or an address like 0x02)"))
}

/// Encode an INPUTS L3 PDU (opcode `0x12`) carrying `inputs`, from `src` to `dst`. The payload bytes
/// come from `linkctl::Inputs::encode` (the canonical owner); this only wraps them in the L3 header.
fn encode_inputs_pdu(src: u8, dst: u8, inputs: &Inputs) -> Vec<u8> {
    let mut payload = [0u8; Inputs::LEN];
    inputs.encode(&mut payload);
    // OP_INPUTS (0x12) is a fixed, valid opcode (not 0x00/0xFF), so both calls are infallible; the
    // buffer is sized exactly to the L3 header + the committed INPUTS prefix.
    let pdu = Pdu::new(OP_INPUTS, src, dst, &payload).expect("OP_INPUTS is a valid opcode");
    let mut buf = [0u8; net::pdu::HEADER_LEN + Inputs::LEN];
    let n = pdu
        .encode(&mut buf)
        .expect("buf fits L3 header + INPUTS payload");
    buf[..n].to_vec()
}

/// Parse a `0x`-hex or decimal `u8` (addresses, byte fields).
fn parse_u8(s: &str) -> Result<u8, String> {
    let r = s
        .strip_prefix("0x")
        .map(|h| u8::from_str_radix(h, 16))
        .unwrap_or_else(|| s.parse::<u8>());
    r.map_err(|_| format!("bad u8 value {s:?}"))
}

/// Parse a decimal (or `0x`-hex) `i16` for the throttle word.
fn parse_i16(s: &str) -> Result<i16, String> {
    if let Some(h) = s.strip_prefix("0x") {
        // A hex throttle is a raw bit pattern (e.g. 0xFFFF = -1).
        return u16::from_str_radix(h, 16)
            .map(|u| u as i16)
            .map_err(|_| format!("bad i16 value {s:?}"));
    }
    s.parse::<i16>().map_err(|_| format!("bad i16 value {s:?}"))
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let endpoint = args.next().ok_or(USAGE)?;

    let mut base = MAILBOX_BASE;
    let mut dst = DstArg::Attached;
    let mut buttons: u8 = 0;
    let mut throttle: i16 = 0;
    let mut rider: u8 = 0;
    let mut hold_secs: u64 = 0;

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
            "--buttons" => buttons = parse_u8(&val()?)?,
            "--throttle" => throttle = parse_i16(&val()?)?,
            "--rider" => rider = parse_u8(&val()?)?,
            "--hold" => {
                let h = val()?;
                hold_secs = h.parse::<u64>().map_err(|_| format!("bad --hold {h:?}"))?;
                if hold_secs > MAX_HOLD_SECS {
                    return Err(format!("--hold must be 0..={MAX_HOLD_SECS} seconds"));
                }
            }
            other => return Err(format!("unknown argument {other:?}\n{USAGE}")),
        }
    }

    // Attach the mailbox + walk to bring the L3 link up and learn the master's address.
    let mem = OpenOcdTcl::connect(&endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200)
        .map_err(|_| "firmware never wrote epoch_ack (no poll-site running?)".to_string())?;

    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;

    // Resolve the destination. `attached` derives it from the walk and SAYS what it derived, with
    // the port-table evidence: this tool can arm a bridge, so the operator must be able to check
    // the board it is about to talk to against the board in front of them.
    let dst = match dst {
        DstArg::Attached => {
            let a = walk.attached_addr().map_err(|e| e.to_string())?;
            println!(
                "dst resolved: attached node 0x{a:02x}{}",
                swd_bridge::walk::host_link_note(walk.host_link()),
            );
            a
        }
        DstArg::Explicit(a) => {
            let resolved = walk.attached_addr().ok();
            match resolved {
                Some(r) if r == a => {
                    println!("dst 0x{a:02x} (explicit; this IS the attached node)")
                }
                Some(r) => println!(
                    "dst 0x{a:02x} (explicit; NOTE the attached node is 0x{r:02x}, so this is a \
                     command to a board reached THROUGH it)"
                ),
                None => println!("dst 0x{a:02x} (explicit; the walk resolved no attached node)"),
            }
            a
        }
    };
    let src = walk.guest_addr();

    let inputs = Inputs {
        throttle,
        buttons,
        rider,
    };
    let pdu = encode_inputs_pdu(src, dst, &inputs);

    // Arm the interrupt path BEFORE the send that can arm a board, so there is no window in which
    // Ctrl-C kills an armed session without an all-clear. A one-shot send needs no handler: the
    // process is already over, and the mirror expires on its own.
    if hold_secs > 0 {
        install_sigint_handler();
    }
    walk.send_pdu(&pdu).map_err(|e| e.to_string())?;

    println!(
        "sent INPUTS 0x{src:02x}->0x{dst:02x}: throttle={throttle} buttons={buttons:#04x} \
         (power_request={}) rider={rider:#04x} (rider_present={})",
        inputs.power_request(),
        inputs.rider_present(),
    );
    println!("  PDU bytes: {pdu:02x?}");

    if hold_secs == 0 {
        println!(
            "PASS: INPUTS delivered (best-effort, no reply). The mirror EXPIRES after {} ms: \
             every level in it reads released from then on. Use --hold SECS to keep it asserted.",
            INPUTS_TIMEOUT_TICKS as u64 * CONTROL_TICK_MS
        );
        return Ok(());
    }

    // The hold: re-send the same payload over the attached link until the deadline, or until
    // Ctrl-C, which leaves by the same all-clear below. Stopping in any way at all is a release;
    // the timeout is what covers the stops this process cannot run code for.
    let started = Instant::now();
    let deadline = started + Duration::from_secs(hold_secs);
    println!(
        "holding for {hold_secs} s: re-sending every {RESEND_INTERVAL_MS} ms \
         ({} re-sends). Ctrl-C releases immediately (explicit all-clear); a killed process \
         releases {} ms later.",
        resends_for(hold_secs),
        INPUTS_TIMEOUT_TICKS as u64 * CONTROL_TICK_MS
    );
    let mut sends = 1u64;
    let mut interrupted = false;
    while Instant::now() < deadline {
        if !sleep_unless_interrupted(Duration::from_millis(RESEND_INTERVAL_MS)) {
            interrupted = true;
            break;
        }
        walk.send_pdu(&pdu).map_err(|e| e.to_string())?;
        sends += 1;
    }

    // Release explicitly rather than leaning on the timeout, the `swd-mailbox-drive` discipline:
    // every level is clear before this process exits, and the timeout stays the backstop for the
    // ways a tool does NOT get to exit cleanly.
    let clear = Inputs {
        throttle: 0,
        buttons: 0,
        rider: 0,
    };
    walk.send_pdu(&encode_inputs_pdu(src, dst, &clear))
        .map_err(|e| e.to_string())?;

    if interrupted {
        println!("sent {sends} INPUTS frames, then an explicit all-clear (Ctrl-C)");
        println!(
            "RELEASED: interrupted after {} s of a {hold_secs} s hold; every level released \
             (power_request clear, board disarmed)",
            started.elapsed().as_secs()
        );
    } else {
        println!("sent {sends} INPUTS frames, then an explicit all-clear");
        println!(
            "PASS: held {hold_secs} s; every level released (power_request clear, board disarmed)"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_hold_cadence_stays_inside_the_firmwares_mirror_timeout() {
        // The one property that makes --hold work: consecutive sends must be closer together than
        // the firmware's mirror timeout, or the board releases between them. Derived from
        // linkctl's constant, so re-tuning the timeout moves this with it; the test is here to
        // catch the day someone hardcodes the cadence instead.
        let timeout_ms = INPUTS_TIMEOUT_TICKS as u64 * CONTROL_TICK_MS;
        assert_eq!(timeout_ms, 1500, "375 ticks at 250 Hz");
        assert!(
            RESEND_INTERVAL_MS < timeout_ms,
            "a hold that re-sends slower than the timeout does not hold"
        );
        // A whole missed send of margin: even if one re-send is lost, the next still lands inside
        // the window.
        assert!(RESEND_INTERVAL_MS * 2 <= timeout_ms);

        // A hold covers the time asked for: the gap left after the last re-send is under one
        // interval, so it never expires early.
        for secs in [1, 5, 60, 300] {
            let n = resends_for(secs);
            assert!(
                (n + 1) * RESEND_INTERVAL_MS >= secs * 1000,
                "hold of {secs} s leaves a gap at the end"
            );
        }
        assert_eq!(resends_for(0), 0, "no --hold = the single send");
    }

    #[test]
    fn ctrl_c_releases_the_hold_rather_than_leaving_it_to_the_timeout() {
        // The property the handler exists for: SIGINT sets the flag the hold's sleep polls, so the
        // loop stops waiting and runs the same explicit all-clear a completed hold runs. Left to
        // the default disposition the process would die inside the sleep and the board would stay
        // armed for the rest of INPUTS_TIMEOUT_TICKS.
        install_sigint_handler();
        // SAFETY: `raise` delivers SIGINT to this thread with our handler installed, so it is
        // handled rather than fatal. This is the only test that touches INTERRUPTED, so a parallel
        // test cannot see the flag it sets.
        unsafe { libc::raise(libc::SIGINT) };
        assert!(
            INTERRUPTED.load(Ordering::SeqCst),
            "SIGINT must arm the release path"
        );

        let t0 = Instant::now();
        assert!(
            !sleep_unless_interrupted(Duration::from_secs(30)),
            "an interrupted hold must not sit out its re-send interval"
        );
        assert!(
            t0.elapsed() < Duration::from_secs(1),
            "the interrupt must be noticed within a poll interval, not at the end of the sleep"
        );

        // And the handler stepped aside as it fired: a SECOND Ctrl-C is the default disposition
        // again, so an all-clear that hangs on the network can still be killed from the keyboard.
        // SAFETY: setting the disposition to the value it should already hold, and reading back
        // the previous one.
        let prev = unsafe { libc::signal(libc::SIGINT, libc::SIG_DFL) };
        assert_eq!(
            prev,
            libc::SIG_DFL,
            "the handler must restore SIG_DFL when it fires"
        );
    }

    #[test]
    fn encodes_power_request_pdu_bytes() {
        // buttons bit0 = power_request; throttle/rider zero. L3 header [op, src, dst] then the
        // little-endian INPUTS payload [throttle_lo, throttle_hi, buttons, rider].
        let inp = Inputs {
            throttle: 0,
            buttons: Inputs::BUTTON_POWER,
            rider: 0,
        };
        let pdu = encode_inputs_pdu(0x80, 0x01, &inp);
        assert_eq!(pdu, vec![0x12, 0x80, 0x01, 0x00, 0x00, 0x01, 0x00]);
    }

    #[test]
    fn encodes_disarm_pdu_bytes() {
        // buttons = 0 clears power_request.
        let inp = Inputs {
            throttle: 0,
            buttons: 0,
            rider: 0,
        };
        let pdu = encode_inputs_pdu(0x80, 0x01, &inp);
        assert_eq!(pdu, vec![0x12, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn encodes_throttle_and_rider_little_endian() {
        // throttle -1 = 0xFFFF LE; rider bit0 set; buttons power+bit1.
        let inp = Inputs {
            throttle: -1,
            buttons: Inputs::BUTTON_POWER | 0x02,
            rider: Inputs::RIDER_PRESENT,
        };
        let pdu = encode_inputs_pdu(0x81, 0x02, &inp);
        assert_eq!(pdu, vec![0x12, 0x81, 0x02, 0xff, 0xff, 0x03, 0x01]);
    }

    #[test]
    fn parse_u8_hex_and_dec() {
        assert_eq!(parse_u8("0x01"), Ok(1));
        assert_eq!(parse_u8("255"), Ok(255));
        assert!(parse_u8("0x1ff").is_err());
    }

    #[test]
    fn parse_i16_signed_and_hex() {
        assert_eq!(parse_i16("-1"), Ok(-1));
        assert_eq!(parse_i16("0xffff"), Ok(-1));
        assert_eq!(parse_i16("300"), Ok(300));
    }
}
