//! Host tests for the `ble` crate (`specs/ble.md`, "Build / test plan (host)").
//!
//! These run on the laptop with `cargo test -p ble`. They drive [`Module::bring_up`] against a stub
//! serial (a fake `Read + Write + ReadReady`) and assert the spec's host items:
//! - bring-up to data mode + passthrough (a stub that acks every command and echoes after data mode); the
//!   passthrough also proves the `MODE=DATA` ack is DRAINED, not leaked into the byte stream,
//! - `Error::Probe` on a silent stub (never replies `AT+OK\r\n`),
//! - the probe advances on the exact 7-byte `AT+OK\r\n` ack in the drained reply (a bare `OK` does NOT;
//!   trailing bytes after the ack are tolerated and drained),
//! - the data-mode gate: no `Pipe` (and so no passthrough) exists until transparent mode.

use super::*;
use embedded_io::{ErrorType, Read, ReadReady, Write};
use std::vec::Vec;

/// A no-op `DelayNs`: the host tests have no real timing, so every wait returns immediately. The pacing
/// only matters on silicon; here it lets the bring-up sequence run to completion instantly.
struct NoDelay;
impl DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

/// How a stub serial answers the probe (state 0), so we can exercise the exact-7-byte match.
#[derive(Clone, Copy)]
enum ProbeReply {
    /// Reply the exact 7 bytes `AT+OK\r\n` to each probe (advances).
    Ok,
    /// Reply nothing (a silent module -> `Error::Probe`).
    Silent,
    /// Reply a bare `OK\r\n` (4 bytes; must NOT advance -- no `AT+OK\r\n` in the stream).
    BareOk,
    /// Reply the ack plus a trailing byte `AT+OK\r\nX` (8 bytes); DOES advance -- the drain matches the
    /// 7-byte `AT+OK\r\n` and discards the trailing `X` (the tolerant case; the real module sends no tail).
    LongLine,
}

/// A stub serial. It watches the bytes written to it; when a complete `AT\r\n` probe is written it queues
/// the configured reply into the RX buffer. After data mode is reached (the `MODE=DATA` command was
/// written), it echoes everything subsequently written, so a `Pipe` round-trip can be tested.
///
/// `read_ready`/`read` drain the RX buffer; `write` records TX and drives the reply/echo behavior. This
/// is the spec's "fake `Read + Write` that replies `AT+OK\r\n` to the probe and echoes after".
struct StubSerial {
    reply: ProbeReply,
    tx: Vec<u8>,
    rx: Vec<u8>,
    in_data_mode: bool,
}

impl StubSerial {
    fn new(reply: ProbeReply) -> Self {
        Self {
            reply,
            tx: Vec::new(),
            rx: Vec::new(),
            in_data_mode: false,
        }
    }

    /// Queue the configured probe reply into the RX buffer (called when a full `AT\r\n` was written).
    fn queue_probe_reply(&mut self) {
        match self.reply {
            ProbeReply::Ok => self.rx.extend_from_slice(b"AT+OK\r\n"),
            ProbeReply::Silent => {}
            ProbeReply::BareOk => self.rx.extend_from_slice(b"OK\r\n"),
            ProbeReply::LongLine => self.rx.extend_from_slice(b"AT+OK\r\nX"),
        }
    }
}

impl ErrorType for StubSerial {
    type Error = core::convert::Infallible;
}

impl Read for StubSerial {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() || self.rx.is_empty() {
            return Ok(0);
        }
        let n = buf.len().min(self.rx.len());
        for (i, b) in self.rx.drain(..n).enumerate() {
            buf[i] = b;
        }
        Ok(n)
    }
}

impl ReadReady for StubSerial {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.rx.is_empty())
    }
}

impl Write for StubSerial {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.extend_from_slice(buf);

        // The module acks EVERY command line (one ending in `\r\n`) with `AT+OK\r\n`, EXCEPT the probe
        // `AT\r\n`, which gets the configured reply so the probe-match cases can be exercised. (Match on
        // the TX tail so the caller's per-call write granularity does not matter.) This models the real
        // module: `bring_up` must DRAIN each ack or the config does not take, and the `MODE=DATA` ack must
        // not leak into the transparent stream.
        if !self.in_data_mode && self.tx.ends_with(b"AT\r\n") {
            self.queue_probe_reply();
        } else if !self.in_data_mode && self.tx.ends_with(b"\r\n") {
            self.rx.extend_from_slice(b"AT+OK\r\n");
        }
        // Once MODE=DATA has been written, the module is a transparent bridge: echo afterwards. (Its own
        // AT+OK ack was queued just above and must be consumed by the MODE_DATA drain, not echoed.)
        if self
            .tx
            .windows(b"AT+MODE=DATA\r\n".len())
            .any(|w| w == b"AT+MODE=DATA\r\n")
        {
            self.in_data_mode = true;
        }

        // In data mode, every written byte is echoed back into RX (transparent loopback), except the
        // MODE=DATA command itself (the write that flips the mode is the command, not payload).
        if self.in_data_mode && buf != b"AT+MODE=DATA\r\n" {
            self.rx.extend_from_slice(buf);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Spec item: bring-up reaches data mode and returns `Ok(Pipe)`; from there `read`/`write` pass through.
#[test]
fn bring_up_reaches_data_mode_and_passes_through() {
    let stub = StubSerial::new(ProbeReply::Ok);
    let mut delay = NoDelay;
    let mut pipe = Module::new("bench-board")
        .con_interval(16)
        .adv_interval(32)
        .bring_up(stub, &mut delay)
        .expect("bring-up should reach data mode on an AT+OK stub");

    // Passthrough: write payload, the transparent stub echoes it, read it back.
    pipe.write_all(b"hello").unwrap();
    let mut buf = [0u8; 5];
    let n = pipe.read(&mut buf).unwrap();
    assert_eq!(n, 5);
    assert_eq!(&buf[..n], b"hello");
}

/// Spec item: the exact AT sequence is sent in order, with `SET=1` BEFORE `MODE=DATA`, and the interval
/// integers formatted into their commands (the `_` underscore is real).
#[test]
fn bring_up_sends_the_exact_sequence_in_order() {
    let stub = StubSerial::new(ProbeReply::Ok);
    let mut delay = NoDelay;
    let pipe = Module::new("name")
        .con_interval(16)
        .adv_interval(32)
        .bring_up(stub, &mut delay)
        .unwrap();
    let tx = pipe.into_inner().tx;

    // Locate each command in the TX stream and assert the order. The probe may repeat, so find the LAST
    // probe and require the rest after it.
    let s = |needle: &[u8]| tx.windows(needle.len()).position(|w| w == needle);
    let probe = s(b"AT\r\n").expect("probe present");
    let name = s(b"AT+NAME=name\r\n").expect("name present");
    let con = s(b"AT+CON_INTERVAL=16\r\n").expect("con_interval present (underscore real)");
    let adv = s(b"AT+ADV_INTERVAL=32\r\n").expect("adv_interval present (underscore real)");
    let set = s(b"AT+SET=1\r\n").expect("set present");
    let mode = s(b"AT+MODE=DATA\r\n").expect("mode=data present");

    assert!(
        probe < name && name < con && con < adv && adv < set && set < mode,
        "order must be probe < NAME < CON_INTERVAL < ADV_INTERVAL < SET=1 < MODE=DATA"
    );
}

/// Spec item: a stub that never replies `AT+OK\r\n` yields `Error::Probe` within the budget (no hang).
#[test]
fn silent_module_fails_probe() {
    let stub = StubSerial::new(ProbeReply::Silent);
    let mut delay = NoDelay;
    // `Pipe` is not Debug, so match on the Result rather than `unwrap_err`.
    assert!(matches!(
        Module::new("name").bring_up(stub, &mut delay),
        Err(Error::Probe)
    ));
}

/// Spec item: a bare `OK` does NOT match the exact 7-byte `AT+OK\r\n`, so bring-up fails `Error::Probe`.
#[test]
fn bare_ok_does_not_advance() {
    let stub = StubSerial::new(ProbeReply::BareOk);
    let mut delay = NoDelay;
    assert!(matches!(
        Module::new("name").bring_up(stub, &mut delay),
        Err(Error::Probe)
    ));
}

/// Spec item: a reply with the `AT+OK\r\n` ack plus trailing bytes (`AT+OK\r\nX`) advances -- the drain
/// matches the ack and discards the trailing byte (the real module sends no tail; this is the tolerant
/// case). Bring-up reaches data mode.
#[test]
fn longer_line_with_ok_advances() {
    let stub = StubSerial::new(ProbeReply::LongLine);
    let mut delay = NoDelay;
    assert!(Module::new("name").bring_up(stub, &mut delay).is_ok());
}

/// Spec item: the data-mode gate keeps the pipe inert until `DataMode`. Because a `Pipe` is ONLY returned
/// after the full sequence reaches state 6, no `Pipe` (and so no passthrough) can exist on a module that
/// never advances past the probe. We assert that the failure path returns no pipe at all.
#[test]
fn data_mode_gate_no_pipe_before_data_mode() {
    let stub = StubSerial::new(ProbeReply::Silent);
    let mut delay = NoDelay;
    let result = Module::new("name").bring_up(stub, &mut delay);
    // No Pipe exists pre-DataMode: the result is the error, never an Ok(Pipe).
    assert!(
        result.is_err(),
        "no Pipe must be handed out before transparent data mode"
    );
}

/// `drain_until_ok` drains every byte and reports whether the exact 7-byte `AT+OK\r\n` appeared in the
/// stream: a bare/short/empty reply never matches; the ack with leading or trailing junk DOES (the drain
/// slides a 7-byte window and tolerates surrounding bytes), which is the new, robust probe semantics.
#[test]
fn drain_matches_at_ok_in_the_stream() {
    fn saw_ok(reply: &[u8]) -> bool {
        // Drive the drain directly via a tiny RX-only stub seeded with `reply`.
        struct RxOnly {
            rx: Vec<u8>,
        }
        impl ErrorType for RxOnly {
            type Error = core::convert::Infallible;
        }
        impl Read for RxOnly {
            fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
                if buf.is_empty() || self.rx.is_empty() {
                    return Ok(0);
                }
                let n = buf.len().min(self.rx.len());
                for (i, b) in self.rx.drain(..n).enumerate() {
                    buf[i] = b;
                }
                Ok(n)
            }
        }
        impl ReadReady for RxOnly {
            fn read_ready(&mut self) -> Result<bool, Self::Error> {
                Ok(!self.rx.is_empty())
            }
        }
        let mut s = RxOnly { rx: reply.to_vec() };
        super::drain_until_ok(&mut s, &mut NoDelay, STEP_MS).unwrap()
    }

    assert!(saw_ok(b"AT+OK\r\n"), "exact ack must match");
    assert!(
        saw_ok(b"AT+OK\r\nX"),
        "ack + trailing byte matches (trailing drained)"
    );
    assert!(
        saw_ok(b"junkAT+OK\r\n"),
        "ack after leading junk matches (window slides)"
    );
    assert!(!saw_ok(b"OK\r\n"), "bare OK must not match");
    assert!(!saw_ok(b"AT+OK"), "incomplete ack must not match");
    assert!(!saw_ok(b""), "empty must not match");
}

/// Constants present and stable (`specs/ble.md`, "Constants present and stable"). Pins the exact AT bytes
/// (including the real `_` in the interval commands) and the discovery hints so they cannot drift.
#[test]
fn constants_are_stable() {
    assert_eq!(at::PROBE, b"AT\r\n");
    assert_eq!(at::OK_REPLY, b"AT+OK\r\n");
    assert_eq!(at::OK_REPLY.len(), 7);
    assert_eq!(at::CON_INTERVAL_PREFIX, b"AT+CON_INTERVAL="); // underscore 0x5f
    assert_eq!(at::ADV_INTERVAL_PREFIX, b"AT+ADV_INTERVAL="); // underscore 0x5f
    assert_eq!(at::SET, b"AT+SET=1\r\n");
    assert_eq!(at::MODE_DATA, b"AT+MODE=DATA\r\n");
    assert_eq!(at::BAUD, 9600);
    assert_eq!(gatt::SERVICE_HINT, 0xFFE0);
    assert_eq!(gatt::CHAR_HINT, 0xFFE1);
}
