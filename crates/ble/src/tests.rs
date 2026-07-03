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
use std::cell::Cell;
use std::rc::Rc;
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

// ----------------------------------------------------------------------------------------------
// Register-faithful fixtures (`specs/ble-rx.md`, Part A): a shared virtual clock + a 1-byte RX
// register that loses bytes to overrun when the reader is too slow, the way the real GD32 USART
// does. The `Vec`-backed `StubSerial` above cannot model that (bytes never expire there), which is
// exactly why the ack-draining bug passed host CI and only failed on silicon.
// ----------------------------------------------------------------------------------------------

/// One UART bit-time-pack: 10 bits (8N1 + start/stop) at 9600 baud, in nanoseconds (1e9 * 10 / 9600).
const BYTE_NS: u64 = 1_041_667;

/// The shared virtual clock: nanoseconds since an arbitrary origin, shared (`Rc`) between the mock
/// delay (which advances it) and the [`RegisterSerial`] (which schedules byte arrivals against it).
type Clock = Rc<Cell<u64>>;

/// A `DelayNs` over the shared virtual clock: every `delay_ns(n)` ADVANCES the clock by `n` (so the
/// provided `delay_us`/`delay_ms` advance it too). No real time; the clock only moves when the code
/// under test delays, so the tests are deterministic and instant.
struct MockDelay {
    clock: Clock,
}

impl MockDelay {
    fn new(clock: Clock) -> Self {
        Self { clock }
    }
}

impl DelayNs for MockDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.clock.set(self.clock.get() + ns as u64);
    }
}

/// A byte the module has scheduled onto the wire: it becomes readable once the clock reaches
/// `arrival` (the time its last bit has shifted into the 1-byte RX register).
struct ScheduledByte {
    arrival: u64,
    val: u8,
}

/// A serial that models a 1-byte RX register fed at 9600 8N1. Replies are scheduled byte-by-byte at
/// the baud rate, so a consumer that reads slower than `BYTE_NS` overruns the register and loses all
/// but the freshest byte, exactly like `runtime-hal`'s `try_read_byte` (clears ORE, returns the
/// freshest). A consumer polling faster than `BYTE_NS` (the crate's ~`POLL_US` drain) sees every byte
/// once, in order, and reconstructs the full `AT+OK\r\n` ack.
struct RegisterSerial {
    clock: Clock,
    reply: ProbeReply,
    tx: Vec<u8>,
    /// Module-scheduled RX bytes, in ascending `arrival` order (later commands schedule later).
    scheduled: Vec<ScheduledByte>,
    /// The arrival time of the most recently consumed byte; only bytes arriving after it are visible.
    last_consumed: u64,
    /// Count of bytes dropped to register overrun (a slow read lost them); the tests assert on this.
    overruns: usize,
    in_data_mode: bool,
}

impl RegisterSerial {
    fn new(clock: Clock, reply: ProbeReply) -> Self {
        Self {
            clock,
            reply,
            tx: Vec::new(),
            scheduled: Vec::new(),
            last_consumed: 0,
            overruns: 0,
            in_data_mode: false,
        }
    }

    /// How many bytes the register has dropped to overrun (a too-slow reader).
    fn overruns(&self) -> usize {
        self.overruns
    }

    /// Schedule `bytes` to arrive at 9600 8N1 starting from now: byte `i` arrives at
    /// `command_complete_time + (i + 1) * BYTE_NS` (the module replies after the command).
    fn schedule_reply(&mut self, bytes: &[u8]) {
        let now = self.clock.get();
        for (i, &val) in bytes.iter().enumerate() {
            self.scheduled.push(ScheduledByte {
                arrival: now + (i as u64 + 1) * BYTE_NS,
                val,
            });
        }
    }
}

impl ErrorType for RegisterSerial {
    type Error = core::convert::Infallible;
}

impl Read for RegisterSerial {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let now = self.clock.get();
        // Among arrived-but-unconsumed bytes (`last_consumed < arrival <= now`), the register holds
        // only the freshest; reading drops the rest as overruns and returns the freshest. `scheduled`
        // is ascending, so the last match is the freshest.
        let mut freshest: Option<(u64, u8)> = None;
        let mut count = 0usize;
        for s in &self.scheduled {
            if s.arrival > self.last_consumed && s.arrival <= now {
                count += 1;
                freshest = Some((s.arrival, s.val));
            }
        }
        match freshest {
            None => Ok(0),
            Some((arrival, val)) => {
                self.overruns += count - 1;
                self.last_consumed = arrival;
                buf[0] = val;
                Ok(1)
            }
        }
    }
}

impl ReadReady for RegisterSerial {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        let now = self.clock.get();
        Ok(self
            .scheduled
            .iter()
            .any(|s| s.arrival > self.last_consumed && s.arrival <= now))
    }
}

impl Write for RegisterSerial {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.extend_from_slice(buf);

        // Pre-data-mode, a full command line (ending `\r\n`) schedules the module's baud-paced reply:
        // the probe `AT\r\n` gets the configured reply; every other command gets `AT+OK\r\n`, and
        // `AT+SET=1` double-acks (two `AT+OK`), as on silicon.
        if !self.in_data_mode && self.tx.ends_with(b"AT\r\n") {
            match self.reply {
                ProbeReply::Ok => self.schedule_reply(b"AT+OK\r\n"),
                ProbeReply::Silent => {}
                ProbeReply::BareOk => self.schedule_reply(b"OK\r\n"),
                ProbeReply::LongLine => self.schedule_reply(b"AT+OK\r\nX"),
            }
        } else if !self.in_data_mode && self.tx.ends_with(b"\r\n") {
            if self.tx.ends_with(at::SET) {
                self.schedule_reply(b"AT+OK\r\nAT+OK\r\n");
            } else {
                self.schedule_reply(b"AT+OK\r\n");
            }
        }

        // After `MODE=DATA` the module is a transparent bridge: its own ack was scheduled just above
        // (and the drain must consume it), and from here written bytes are echoed back at the baud.
        if self
            .tx
            .windows(at::MODE_DATA.len())
            .any(|w| w == at::MODE_DATA)
        {
            self.in_data_mode = true;
        }
        if self.in_data_mode && buf != at::MODE_DATA {
            self.schedule_reply(buf);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Part A test 1: prompt drain succeeds through the register, and passthrough round-trips. Bring-up
/// over the baud-paced register returns `Ok(Pipe)` (the ~`POLL_US` poll captures each ack byte with
/// zero overrun loss), the TX stream carries the full AT sequence in order, and a payload written
/// after data mode echoes back through the same register.
#[test]
fn bring_up_through_register_drains_and_passes_through() {
    let clock: Clock = Rc::new(Cell::new(0));
    let serial = RegisterSerial::new(clock.clone(), ProbeReply::Ok);
    let mut delay = MockDelay::new(clock.clone());

    let mut pipe = Module::new("bench-board")
        .con_interval(16)
        .adv_interval(32)
        .bring_up(serial, &mut delay)
        .expect("bring-up should reach data mode through the baud-paced 1-byte register");

    // Passthrough: write a payload, then poll it back at the prompt cadence (faster than BYTE_NS) so
    // the baud-paced echo is read without overrun, reconstructing the whole payload.
    pipe.write_all(b"hello").unwrap();
    let mut got = Vec::new();
    for _ in 0..10_000 {
        let mut one = [0u8; 1];
        if pipe.read(&mut one).unwrap() == 1 {
            got.push(one[0]);
            if got.len() == b"hello".len() {
                break;
            }
        } else {
            delay.delay_us(POLL_US);
        }
    }
    assert_eq!(
        got, b"hello",
        "passthrough round-trips through the register"
    );

    // The TX stream carries the full sequence in order (probe < NAME < CON < ADV < SET=1 < MODE=DATA).
    let tx = pipe.into_inner().tx;
    let s = |needle: &[u8]| tx.windows(needle.len()).position(|w| w == needle);
    let probe = s(b"AT\r\n").expect("probe present");
    let name = s(b"AT+NAME=bench-board\r\n").expect("name present");
    let con = s(b"AT+CON_INTERVAL=16\r\n").expect("con_interval present");
    let adv = s(b"AT+ADV_INTERVAL=32\r\n").expect("adv_interval present");
    let set = s(b"AT+SET=1\r\n").expect("set present");
    let mode = s(b"AT+MODE=DATA\r\n").expect("mode=data present");
    assert!(
        probe < name && name < con && con < adv && adv < set && set < mode,
        "order must be probe < NAME < CON_INTERVAL < ADV_INTERVAL < SET=1 < MODE=DATA"
    );
}

/// Part A test 2: slow reading overruns the register (the pinned contract). Stage an `AT+OK\r\n`
/// reply, advance the clock past the WHOLE reply with a single `delay_ms` (the blind-delay-then-read
/// drain the original bug had), then read once: only the final byte survives and the overrun counter
/// shows the earlier six were dropped, so a blind delay could never have matched the 7-byte ack.
#[test]
fn slow_read_overruns_the_register() {
    let clock: Clock = Rc::new(Cell::new(0));
    let mut serial = RegisterSerial::new(clock.clone(), ProbeReply::Ok);
    // A full command line schedules the `AT+OK\r\n` ack, byte by byte at the baud.
    serial.write_all(b"AT\r\n").unwrap();

    // Blind delay past the entire 7-byte reply (7 * BYTE_NS ~ 7.3 ms), then a single read.
    MockDelay::new(clock.clone()).delay_ms(10);
    let mut buf = [0u8; 16];
    let n = serial.read(&mut buf).unwrap();

    assert_eq!(n, 1, "the 1-byte register holds only one byte");
    assert_eq!(
        buf[0], b'\n',
        "only the freshest byte (the ack's trailing LF) survives a slow read"
    );
    assert_eq!(
        serial.overruns(),
        b"AT+OK\r\n".len() - 1,
        "the other six ack bytes were lost to register overrun"
    );
}

/// Part A test 3: no spurious overrun at the prompt rate. Drive `drain_until_ok` against a staged
/// `AT+OK\r\n` at the real `POLL_US` cadence: it matches the ack and the overrun counter stays zero
/// (every baud-paced byte was pulled from the register before the next overwrote it).
#[test]
fn prompt_drain_matches_with_no_overrun() {
    let clock: Clock = Rc::new(Cell::new(0));
    let mut serial = RegisterSerial::new(clock.clone(), ProbeReply::Ok);
    serial.write_all(b"AT\r\n").unwrap();

    let mut delay = MockDelay::new(clock.clone());
    let matched = super::drain_until_ok(&mut serial, &mut delay, STEP_MS).unwrap();

    assert!(matched, "the prompt poll reconstructs the 7-byte AT+OK ack");
    assert_eq!(
        serial.overruns(),
        0,
        "polling faster than BYTE_NS loses no bytes to overrun"
    );
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

/// The BT-probe phase-1 detector: `probe` answers true for an `AT+OK` module, false for a silent port
/// (a peer board / the un-initialised IMU) and for a bare `OK` (not the exact `AT+OK\r\n`).
#[test]
fn probe_detects_a_module_and_rejects_silence_and_bare_ok() {
    let mut delay = NoDelay;
    let mut ok = StubSerial::new(ProbeReply::Ok);
    assert!(
        crate::probe(&mut ok, &mut delay, 3).unwrap(),
        "an AT+OK module is detected"
    );
    let mut silent = StubSerial::new(ProbeReply::Silent);
    assert!(
        !crate::probe(&mut silent, &mut delay, 3).unwrap(),
        "a silent port is not a module"
    );
    let mut bare = StubSerial::new(ProbeReply::BareOk);
    assert!(
        !crate::probe(&mut bare, &mut delay, 3).unwrap(),
        "a bare OK (not AT+OK) is not a match"
    );
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

// --- the data-mode gate type (specs/ble.md, "The data-mode gate type") -------------------------

/// `Pipe` implements `ReadReady` by delegation: what lets `link::SerialTransport` (ReadReady-gated)
/// ride the gate type directly, with readiness exactly the inner serial's.
#[test]
fn pipe_read_ready_delegates_to_the_inner_serial() {
    let mut stub = StubSerial::new(ProbeReply::Ok);
    stub.in_data_mode = true;
    let mut pipe = Pipe::assume_data_mode(stub);
    assert!(!pipe.read_ready().unwrap(), "empty RX: not ready");
    // Data-mode echo: a write queues the echo into RX, which readiness must reflect.
    pipe.write_all(b"z").unwrap();
    assert!(pipe.read_ready().unwrap(), "echoed byte pending: ready");
    let mut one = [0u8; 1];
    assert_eq!(pipe.read(&mut one).unwrap(), 1);
    assert_eq!(&one, b"z");
    assert!(!pipe.read_ready().unwrap(), "drained again");
}

/// The fallback-arm constructor: a serial whose data-mode state is established knowledge wraps into
/// a fully functional `Pipe` (transparent both ways) without any AT handshake bytes on the wire.
#[test]
fn assume_data_mode_wraps_without_a_handshake() {
    let mut stub = StubSerial::new(ProbeReply::Silent); // would fail a real probe
    stub.in_data_mode = true; // the "warm module still transparent" the l3.md link-set arm asserts
    let mut pipe = Pipe::assume_data_mode(stub);
    pipe.write_all(b"hello").unwrap();
    let mut buf = [0u8; 8];
    let n = pipe.read(&mut buf).unwrap();
    assert_eq!(
        &buf[..n],
        b"hello",
        "transparent echo through the gate type"
    );
    // No AT traffic was generated by construction: the TX log holds only the data bytes.
    assert_eq!(pipe.into_inner().tx, b"hello");
}
