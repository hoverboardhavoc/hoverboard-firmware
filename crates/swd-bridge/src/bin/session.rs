//! `swd-mailbox-session`: ONE owner of a mailbox session, holding an arm and a demand together.
//!
//! **Interim bench tool.** It exists to get the current limit's energised gates run with the owner
//! already at the bench; the formal version is briefed in
//! `specs/agent-runs/mailbox-session-owner-prompt.md` and should replace this.
//!
//! # Why it has to be one process
//!
//! A second tool starting mid-session cost two current-limit gates on 2026-10-08: the board
//! disarmed within 1.5 s of it attaching, every time. **Corrected 2026-10-09 after an audit read
//! the code rather than the spec:** three mechanisms are in play, and the first explanation given
//! (that `attach()` invalidates the earlier producer's session) was wrong.
//!
//! - `EpochWatch::poll` (`crates/swd-mailbox/src/lib.rs`) flushes the inbound ring ONCE on a
//!   changed `epoch` and tracks no producer identity, and the host's `produce` re-reads `h2t_head`
//!   on every send and never looks at the epoch. So the first producer's LATER frames are consumed
//!   normally. What the flush does kill is whatever was in flight at that instant.
//! - The arriving tool then runs the 30 s L3 walk, re-running `NODE_HELLO`/`ASSIGN` against the
//!   firmware while the first tool is mid-stream.
//! - And both processes are producers on one SPSC ring: each reads `h2t_head`, each writes at it,
//!   and the later commit overwrites the earlier advance (`specs/swd-mailbox.md`, "Ring
//!   discipline": exactly one side may write `head`).
//!
//! Any one of the three loses an `INPUTS` keepalive, and the mirror ages out at
//! `INPUTS_TIMEOUT_TICKS` (1.5 s). A lock would prevent the third and probably the second; it
//! cannot prevent the first, because the flush is a consequence of attaching at all. One session
//! OWNER removes all three, which is this: one attach, one producer, both payloads, and demand
//! changes arriving on STDIN rather than in a new process.
//!
//! Usage: `swd-mailbox-session <host:port> [--base HEX] [--dst attached|ADDR] [--buttons BYTE]
//!         [--rider BYTE] [--value N] [--steer N] [--hold SECS]`
//!
//! Commands, one per line on stdin, each acknowledged on stdout so a driving script can
//! synchronise:
//!
//! ```text
//! value N     hold throttle demand N (`ok value=N`)
//! neutral     hold DriveKind::Neutral, demand 0 (`ok neutral`)
//! quit        Neutral, then the INPUTS all-clear, then exit
//! ```
//!
//! Until the first `value`, no `DRIVE_CMD` is sent at all, so the arm-and-soak phase of a gate puts
//! exactly the traffic on the link that `swd-mailbox-inputs --hold` put there.
//!
//! # Ending
//!
//! EOF on stdin, Ctrl-C, `quit`, the `--hold` deadline and a FAILED SEND all end the session the
//! same way: an explicit Neutral FIRST, then the INPUTS all-clear. That order is the firmware's own,
//! which asserts `INPUTS_TIMEOUT_TICKS > DRIVE_TIMEOUT_TICKS`: release the demand while the board is
//! still armed, then drop the arm. The release is BEST EFFORT and every step of it is attempted even
//! when an earlier one failed, because a send failing mid-loop is exactly when the levels most need
//! putting down, and the all-clear is the step that disarms. The firmware's two timeouts remain the
//! backstop for the exits no handler can cover (`kill -9`, a pulled cable, this host dying), not a
//! substitute for releasing when this process is still able to act.

use std::io::BufRead;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::time::{Duration, Instant};

use linkctl::{
    DriveCmd, DriveKind, Inputs, DRIVE_TIMEOUT_TICKS, INPUTS_TIMEOUT_TICKS, OP_DRIVE_CMD, OP_INPUTS,
};
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

const USAGE: &str = "usage: swd-mailbox-session <host:port> [--base HEX] [--dst attached|ADDR] \
     [--buttons BYTE] [--rider BYTE] [--value N] [--steer N] [--hold SECS]";

/// The control-task period in ms (250 Hz), the unit both timeouts count in.
const CONTROL_TICK_MS: u64 = 4;

/// The firmware's INPUTS mirror timeout and DRIVE_CMD decay window, in ms.
const INPUTS_TIMEOUT_MS: u64 = INPUTS_TIMEOUT_TICKS as u64 * CONTROL_TICK_MS;
const DECAY_MS: u64 = DRIVE_TIMEOUT_TICKS as u64 * CONTROL_TICK_MS;

/// The INPUTS re-send cadence: comfortably inside the mirror timeout with a whole missed send of
/// margin, and slow enough that it costs the demand stream little link time.
const INPUTS_PERIOD_MS: u64 = INPUTS_TIMEOUT_MS * 7 / 15;

/// The DRIVE_CMD re-send cadence. Shorter than the drive tool's 100 ms on purpose: an INPUTS send
/// can land between two demand sends, and a mailbox write through the bench Pi costs 52 to 94 ms
/// (`specs/todo.md`, the two-hop latency result), so the pair has to fit inside the decay window.
const DRIVE_PERIOD_MS: u64 = 60;

/// How often the loop looks at stdin and [`INTERRUPTED`] between sends. It sits on the critical
/// path of the demand's decay window (see [`WORST_DEMAND_GAP_MS`]), and the work it gates is a
/// `try_recv` plus an atomic load, so it is short: the loop's cost is the mailbox writes.
const POLL_MS: u64 = 4;

/// The longest `--hold` this tool accepts (30 minutes), the `swd-mailbox-inputs` bound: a bench
/// session is a bounded act with a person in front of it.
const MAX_HOLD_SECS: u64 = 1800;

// The property that makes one process able to hold both payloads: each is re-sent faster than the
// window the firmware ages it out in, with a whole missed send of margin. These are relations
// between constants, so they are checked at COMPILE time and a violation fails the build.
const _: () = assert!(INPUTS_TIMEOUT_MS == 1500, "375 ticks at 250 Hz");
const _: () = assert!(DECAY_MS == 200, "50 ticks at 250 Hz");
const _: () = assert!(INPUTS_PERIOD_MS * 2 <= INPUTS_TIMEOUT_MS);
const _: () = assert!(DRIVE_PERIOD_MS * 2 <= DECAY_MS);
/// The measured worst-case cost of one mailbox write through the bench Pi (`specs/todo.md`, the
/// two-hop latency result: 52 to 94 ms).
const WORST_WRITE_MS: u64 = 94;

/// The worst receipt-to-receipt gap between two demand frames, which must stay inside [`DECAY_MS`].
///
/// Both payloads go over ONE openocd connection, so an INPUTS write can sit between two demand
/// writes, and `next_drive` is set from the instant BEFORE the write, which makes the schedule
/// start-to-start. Two branches, whichever is larger:
///
/// - period-bound: the deadline is missed by at most one poll, and an INPUTS write can still
///   intervene before the demand write begins, so `DRIVE_PERIOD_MS + POLL_MS + WORST_WRITE_MS`;
/// - write-bound: at 60 ms the period is already PAST when the previous write completes (one write
///   costs up to 94 ms), so the next demand write begins after the poll and the intervening INPUTS
///   write, `2 * WORST_WRITE_MS + POLL_MS`.
///
/// The write-bound branch is the real one at these constants, which is why lowering
/// `DRIVE_PERIOD_MS` buys nothing: the cadence is bounded by the write cost, not by the period.
/// Setting `next_drive` AFTER the write would make it receipt-relative and ADD the period to that
/// cost (60 + 4 + 2 * 94 = 252 ms), lapsing the demand outright. What does buy margin is the poll,
/// which is why it is 4 ms: 192 ms against a 200 ms decay. The earlier assertion claimed 164 ms,
/// counting one write where two are serialised.
const WORST_DEMAND_GAP_MS: u64 = {
    let period_bound = DRIVE_PERIOD_MS + POLL_MS + WORST_WRITE_MS;
    let write_bound = 2 * WORST_WRITE_MS + POLL_MS;
    if period_bound > write_bound {
        period_bound
    } else {
        write_bound
    }
};
const _: () = assert!(
    WORST_DEMAND_GAP_MS < DECAY_MS,
    "a demand frame would lapse when an INPUTS send intervenes"
);

/// Set by the `SIGINT` handler, polled by the loop. Ctrl-C means release now, by the same path a
/// completed session takes, rather than leaving the levels to the firmware's timeouts.
static INTERRUPTED: AtomicBool = AtomicBool::new(false);

/// The `SIGINT` handler: async-signal-safe (an atomic store and `signal`, no allocation, no I/O).
extern "C" fn on_sigint(_sig: libc::c_int) {
    INTERRUPTED.store(true, Ordering::SeqCst);
    // Step aside so a SECOND Ctrl-C kills the process outright: the release this schedules is a
    // write to openocd over TCP, and if that hangs the operator must still be able to get out.
    // SAFETY: `signal` is async-signal-safe and `SIG_DFL` is the disposition we started with.
    unsafe { libc::signal(libc::SIGINT, libc::SIG_DFL) };
}

fn install_sigint_handler() {
    // SAFETY: `on_sigint` has the `sighandler_t` signature and does only signal-safe work. The
    // two-step cast is what the `function_casts_as_integer` lint requires.
    unsafe { libc::signal(libc::SIGINT, on_sigint as *const () as libc::sighandler_t) };
}

/// One line from stdin, parsed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cmd {
    /// Hold this throttle demand.
    Value(i16),
    /// Hold Neutral (demand released, board still armed).
    Neutral,
    /// Release everything and exit.
    Quit,
}

/// Parse one stdin line. Blank lines and `#` comments are ignored (`None`); anything else that is
/// not a command is an error the caller prints and otherwise ignores, because a typo must not drop
/// an arm.
fn parse_cmd(line: &str) -> Result<Option<Cmd>, String> {
    let line = line.trim();
    if line.is_empty() || line.starts_with('#') {
        return Ok(None);
    }
    let mut it = line.split_whitespace();
    let word = it.next().unwrap_or_default();
    let rest: Vec<&str> = it.collect();
    match (word, rest.as_slice()) {
        ("value", [v]) => v
            .parse::<i16>()
            .map(|v| Some(Cmd::Value(v)))
            .map_err(|_| format!("bad value {v:?} (want an i16 on the +-32767 frame scale)")),
        ("neutral", []) => Ok(Some(Cmd::Neutral)),
        ("quit", []) => Ok(Some(Cmd::Quit)),
        _ => Err(format!(
            "unknown command {line:?} (want `value N`, `neutral` or `quit`)"
        )),
    }
}

/// Read stdin on its own thread, so the send loop never blocks on the operator or the script.
/// EOF closes the channel, which the loop reads as `quit`: the parent ended the session, or died.
fn spawn_stdin_reader() -> Receiver<String> {
    let (tx, rx) = mpsc::channel();
    std::thread::spawn(move || {
        for line in std::io::stdin().lock().lines() {
            match line {
                Ok(l) => {
                    if tx.send(l).is_err() {
                        return;
                    }
                }
                Err(_) => return,
            }
        }
    });
    rx
}

/// Encode an INPUTS L3 PDU (`OP_INPUTS`), payload bytes from `linkctl` (their canonical owner).
fn encode_inputs_pdu(src: u8, dst: u8, inputs: &Inputs) -> Vec<u8> {
    let mut payload = [0u8; Inputs::LEN];
    inputs.encode(&mut payload);
    let pdu = Pdu::new(OP_INPUTS, src, dst, &payload).expect("OP_INPUTS is a valid opcode");
    let mut buf = [0u8; net::pdu::HEADER_LEN + Inputs::LEN];
    let n = pdu
        .encode(&mut buf)
        .expect("buf fits L3 header + INPUTS payload");
    buf[..n].to_vec()
}

/// Encode a `DRIVE_CMD` L3 PDU (`OP_DRIVE_CMD`), payload bytes from `linkctl`.
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

/// Parse a `0x`-hex or decimal `u8` (addresses, byte fields).
fn parse_u8(s: &str) -> Result<u8, String> {
    let r = s
        .strip_prefix("0x")
        .map(|h| u8::from_str_radix(h, 16))
        .unwrap_or_else(|| s.parse::<u8>());
    r.map_err(|_| format!("bad u8 value {s:?}"))
}

fn parse_i16(s: &str) -> Result<i16, String> {
    s.parse::<i16>().map_err(|_| format!("bad i16 value {s:?}"))
}

/// How `--dst` was given (mirrors `swd-mailbox-inputs` and `swd-mailbox-drive`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DstArg {
    Attached,
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

/// The demand the session is holding: `None` until the first `value`, so the pre-demand phase puts
/// no `DRIVE_CMD` on the link at all.
fn drive_of(value: i16, steer: i16, neutral: bool) -> DriveCmd {
    DriveCmd {
        kind: if neutral {
            DriveKind::Neutral
        } else {
            DriveKind::Throttle
        },
        value: if neutral { 0 } else { value },
        steer: if neutral { 0 } else { steer },
    }
}

/// Put both levels down, best effort: the demand first (while the board is still armed), then the
/// INPUTS all-clear, which is the firmware's own order. EVERY step is attempted even when an
/// earlier one failed, because the all-clear is the step that disarms and a failed Neutral is no
/// reason to skip it. Returns the failures, in the order they happened, so the caller can say what
/// did not get through.
fn release(
    send: &mut dyn FnMut(&[u8]) -> Result<(), String>,
    src: u8,
    dst: u8,
    demanding: bool,
) -> Vec<String> {
    let mut errs = Vec::new();
    if demanding {
        if let Err(e) = send(&encode_drive_pdu(src, dst, &drive_of(0, 0, true))) {
            errs.push(format!("the Neutral did not go out: {e}"));
        }
    }
    let clear = Inputs {
        buttons: 0,
        rider: 0,
    };
    if let Err(e) = send(&encode_inputs_pdu(src, dst, &clear)) {
        errs.push(format!("the INPUTS all-clear did not go out: {e}"));
    }
    errs
}

/// What the process reports once [`release`] has run, and its exit status.
///
/// A clean release says so and the loop's own error (if the session ended on a failed send) is
/// still the process's failure. A release that did not get through is the worse case: the levels
/// are down only when the firmware's timeouts expire them, and the operator has to be told that in
/// those words rather than reading a bare send error.
fn outcome(ending: &str, send_err: Option<String>, release_errs: &[String]) -> Result<(), String> {
    if release_errs.is_empty() {
        println!(
            "RELEASED ({ending}): demand zero, every level released (power_request clear, board \
             disarmed)"
        );
        return match send_err {
            None => Ok(()),
            Some(e) => Err(format!("{e}; the release then went through")),
        };
    }
    let mut why: Vec<String> = send_err.into_iter().collect();
    why.extend(release_errs.iter().cloned());
    Err(format!(
        "NOT RELEASED ({ending}): {}. The firmware's timeouts are all that is left: the demand \
         decays {DECAY_MS} ms and the arm ages out {INPUTS_TIMEOUT_MS} ms after the last frame that \
         reached the board",
        why.join("; ")
    ))
}

fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let endpoint = args.next().ok_or(USAGE)?;

    let mut base = MAILBOX_BASE;
    let mut dst = DstArg::Attached;
    let mut buttons: u8 = 0;
    let mut rider: u8 = 0;
    let mut value: i16 = 0;
    let mut steer: i16 = 0;
    let mut hold_secs: u64 = 0;
    let mut started_with_value = false;

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
            "--rider" => rider = parse_u8(&val()?)?,
            "--value" => {
                value = parse_i16(&val()?)?;
                started_with_value = true;
            }
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
    if hold_secs == 0 {
        return Err(format!(
            "--hold SECS is required: this tool IS a hold (one session, held levels)\n{USAGE}"
        ));
    }

    // Attach the mailbox + walk: the one attach of this session.
    let mem = OpenOcdTcl::connect(&endpoint).map_err(|e| e.to_string())?;
    let mut host = HostMailbox::new(mem, base);
    host.validate().map_err(|e| e.to_string())?;
    host.attach().map_err(|e| e.to_string())?;
    host.wait_flush_ack(200)
        .map_err(|_| "firmware never wrote epoch_ack (no poll-site running?)".to_string())?;

    let mut walk = WalkDriver::new(host);
    walk.run_walk(Duration::from_secs(30))
        .map_err(|e| e.to_string())?;

    // Resolve the destination and SAY what was resolved: this tool arms boards, so the operator
    // must be able to check it against the board in front of them.
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
                    "dst 0x{a:02x} (explicit; NOTE the attached node is 0x{r:02x}, so this is a \
                     command to a board reached THROUGH it)"
                ),
                None => println!("dst 0x{a:02x} (explicit; the walk resolved no attached node)"),
            }
            a
        }
    };
    let src = walk.guest_addr();

    let inputs = Inputs { buttons, rider };
    let inputs_pdu = encode_inputs_pdu(src, dst, &inputs);

    // Arm the interrupt path BEFORE the first send that can arm a board.
    install_sigint_handler();
    let stdin_rx = spawn_stdin_reader();

    println!(
        "session 0x{src:02x}->0x{dst:02x}: buttons={buttons:#04x} (power_request={}) \
         rider={rider:#04x} (rider_present={}), INPUTS every {INPUTS_PERIOD_MS} ms, \
         DRIVE every {DRIVE_PERIOD_MS} ms once a demand is set (mirror timeout          {INPUTS_TIMEOUT_MS} ms, demand decay {DECAY_MS} ms), bounded at {hold_secs} s",
        inputs.power_request(),
        inputs.rider_present(),
    );
    println!("  commands on stdin: `value N`, `neutral`, `quit` (EOF and Ctrl-C release and exit)");

    let started = Instant::now();
    let deadline = started + Duration::from_secs(hold_secs);
    let mut next_inputs = Instant::now();
    let mut next_drive = Instant::now();
    let mut demand: Option<DriveCmd> = started_with_value.then(|| drive_of(value, steer, false));
    let mut inputs_sends = 0u64;
    let mut drive_sends = 0u64;
    let mut ending = "the hold elapsed";
    // A send that fails ends the session, but it returns through the release below, never past it.
    let mut send_err: Option<String> = None;

    if started_with_value {
        println!("ok value={value}");
    }

    loop {
        if INTERRUPTED.load(Ordering::SeqCst) {
            ending = "Ctrl-C";
            break;
        }
        if Instant::now() >= deadline {
            break;
        }

        // Apply every command waiting, in order: the last one wins, which is what a script stepping
        // a ladder means. A bad line is reported and skipped, never a reason to drop the arm.
        let mut quit = false;
        loop {
            match stdin_rx.try_recv() {
                Ok(line) => match parse_cmd(&line) {
                    Ok(None) => {}
                    Ok(Some(Cmd::Value(v))) => {
                        demand = Some(drive_of(v, steer, false));
                        next_drive = Instant::now();
                        println!("ok value={v}");
                    }
                    Ok(Some(Cmd::Neutral)) => {
                        demand = Some(drive_of(0, 0, true));
                        next_drive = Instant::now();
                        println!("ok neutral");
                    }
                    Ok(Some(Cmd::Quit)) => {
                        quit = true;
                    }
                    Err(e) => println!("ERR {e}"),
                },
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    quit = true;
                    ending = "stdin closed";
                    break;
                }
            }
        }
        if quit {
            if ending == "the hold elapsed" {
                ending = "quit";
            }
            break;
        }

        // INPUTS first when both are due: the arm is what the demand depends on.
        let now = Instant::now();
        if now >= next_inputs {
            if let Err(e) = walk.send_pdu(&inputs_pdu) {
                send_err = Some(format!("the INPUTS send failed: {e}"));
                ending = "a send failed";
                break;
            }
            inputs_sends += 1;
            next_inputs = now + Duration::from_millis(INPUTS_PERIOD_MS);
        }
        let now = Instant::now();
        if let Some(cmd) = demand {
            if now >= next_drive {
                if let Err(e) = walk.send_pdu(&encode_drive_pdu(src, dst, &cmd)) {
                    send_err = Some(format!("the DRIVE_CMD send failed: {e}"));
                    ending = "a send failed";
                    break;
                }
                drive_sends += 1;
                next_drive = now + Duration::from_millis(DRIVE_PERIOD_MS);
            }
        }
        std::thread::sleep(Duration::from_millis(POLL_MS));
    }

    // Release in the firmware's own order, on EVERY way out of that loop including a failed send.
    let release_errs = {
        let mut send = |bytes: &[u8]| walk.send_pdu(bytes).map_err(|e| e.to_string());
        release(&mut send, src, dst, demand.is_some())
    };

    println!(
        "sent {inputs_sends} INPUTS and {drive_sends} DRIVE frames over {:.1} s, then an explicit \
         Neutral and all-clear",
        started.elapsed().as_secs_f64()
    );
    outcome(ending, send_err, &release_errs)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_the_three_commands_and_ignores_noise() {
        assert_eq!(parse_cmd("value 3000"), Ok(Some(Cmd::Value(3000))));
        assert_eq!(parse_cmd("  value -1000  "), Ok(Some(Cmd::Value(-1000))));
        assert_eq!(parse_cmd("neutral"), Ok(Some(Cmd::Neutral)));
        assert_eq!(parse_cmd("quit"), Ok(Some(Cmd::Quit)));
        assert_eq!(parse_cmd(""), Ok(None));
        assert_eq!(parse_cmd("   "), Ok(None));
        assert_eq!(parse_cmd("# a comment"), Ok(None));
        // Errors are errors, not silently-zero demands.
        assert!(parse_cmd("value").is_err());
        assert!(parse_cmd("value 40000").is_err());
        assert!(parse_cmd("value 1 2").is_err());
        assert!(parse_cmd("neutral now").is_err());
        assert!(parse_cmd("drive 3000").is_err());
    }

    #[test]
    fn neutral_is_a_kind_not_a_zero_value() {
        // The firmware distinguishes them: Neutral is an explicit release, a Throttle of 0 is a
        // commanded zero. The release path must send the former.
        let n = drive_of(3000, 500, true);
        assert_eq!(n.kind, DriveKind::Neutral);
        assert_eq!((n.value, n.steer), (0, 0));
        let t = drive_of(3000, 500, false);
        assert_eq!(t.kind, DriveKind::Throttle);
        assert_eq!((t.value, t.steer), (3000, 500));
    }

    #[test]
    fn encodes_the_same_bytes_as_the_two_single_purpose_tools() {
        // The wire bytes are the contract this tool must not reinterpret: identical to
        // swd-mailbox-inputs and swd-mailbox-drive for the same payloads.
        let inp = Inputs {
            buttons: Inputs::BUTTON_POWER,
            rider: Inputs::RIDER_PRESENT,
        };
        assert_eq!(
            encode_inputs_pdu(0x80, 0x02, &inp),
            vec![0x12, 0x80, 0x02, 0x01, 0x01]
        );
        let cmd = DriveCmd {
            kind: DriveKind::Throttle,
            value: 1000,
            steer: 0,
        };
        assert_eq!(
            encode_drive_pdu(0x80, 0x02, &cmd),
            vec![0x11, 0x80, 0x02, 0x01, 0xe8, 0x03, 0x00, 0x00]
        );
        let neutral = drive_of(0, 0, true);
        assert_eq!(
            encode_drive_pdu(0x80, 0x02, &neutral),
            vec![0x11, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]
        );
    }

    /// A sender that records every PDU it was handed and fails the calls named in `fail_on`.
    fn recorder<'a>(
        fail_on: &'static [usize],
        sent: &'a mut Vec<Vec<u8>>,
    ) -> impl FnMut(&[u8]) -> Result<(), String> + 'a {
        let mut n = 0;
        move |bytes: &[u8]| {
            sent.push(bytes.to_vec());
            n += 1;
            if fail_on.contains(&(n - 1)) {
                Err(format!("write {n} refused"))
            } else {
                Ok(())
            }
        }
    }

    #[test]
    fn a_failed_neutral_does_not_skip_the_all_clear() {
        // The all-clear is the step that disarms, so it is attempted even when the Neutral before
        // it failed, and a release that fails outright still attempted both.
        let mut sent = Vec::new();
        let errs = release(&mut recorder(&[0], &mut sent), 0x80, 0x02, true);
        assert_eq!(sent.len(), 2, "both PDUs attempted");
        assert_eq!(sent[0], encode_drive_pdu(0x80, 0x02, &drive_of(0, 0, true)));
        assert_eq!(
            sent[1],
            encode_inputs_pdu(
                0x80,
                0x02,
                &Inputs {
                    buttons: 0,
                    rider: 0
                }
            )
        );
        assert_eq!(errs.len(), 1);
        assert!(errs[0].contains("Neutral"), "{errs:?}");

        let mut sent = Vec::new();
        let errs = release(&mut recorder(&[0, 1], &mut sent), 0x80, 0x02, true);
        assert_eq!(sent.len(), 2);
        assert_eq!(errs.len(), 2);

        // No demand was ever set: nothing to neutralise, and the all-clear still goes.
        let mut sent = Vec::new();
        let errs = release(&mut recorder(&[], &mut sent), 0x80, 0x02, false);
        assert_eq!(sent.len(), 1);
        assert!(errs.is_empty());
    }

    #[test]
    fn a_mid_loop_send_error_is_reported_through_the_release_not_instead_of_it() {
        // Clean loop, clean release: success.
        assert_eq!(outcome("quit", None, &[]), Ok(()));
        // A failed send ends the session, but the levels did go down, and the process still fails.
        let e = outcome(
            "a send failed",
            Some("the DRIVE_CMD send failed: EOF".into()),
            &[],
        )
        .expect_err("a failed send is still a failure");
        assert!(e.contains("DRIVE_CMD"), "{e}");
        assert!(e.contains("release then went through"), "{e}");
        // A release that did not get through says so, and names the timeouts that are left.
        let e = outcome(
            "Ctrl-C",
            None,
            &["the INPUTS all-clear did not go out: EOF".into()],
        )
        .expect_err("an unreleased arm is a failure");
        assert!(e.starts_with("NOT RELEASED (Ctrl-C)"), "{e}");
        assert!(e.contains("all-clear"), "{e}");
        assert!(e.contains(&INPUTS_TIMEOUT_MS.to_string()), "{e}");
    }

    #[test]
    fn parse_dst_accepts_the_word_and_an_address() {
        assert_eq!(parse_dst("attached"), Ok(DstArg::Attached));
        assert_eq!(parse_dst("0x02"), Ok(DstArg::Explicit(2)));
        assert!(parse_dst("0x1ff").is_err());
    }
}
