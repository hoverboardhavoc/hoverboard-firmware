//! Host-testable transport logic over `embedded-io`.
//!
//! This module is generic over `embedded_io` traits so the transport LOGIC (non-blocking RX drain,
//! frame TX, BLE AT bring-up) is fully host-testable with an in-memory mock serial. The firmware
//! crate instantiates these generics with the concrete `runtime_hal::UsartSerial` (which implements
//! `embedded_io::{Read, Write, ReadReady}` with `Error = UsartError`); that concrete wiring is not
//! part of this crate.
//!
//! Two pieces live here:
//!
//! - [`LinkPort`]: a thin adapter that moves bytes between a serial and a [`StreamFramer`]. RX is a
//!   bounded, non-blocking drain (it must never block a cooperative scheduler task); TX encodes a
//!   frame and writes it whole. Line errors fold into a health counter, never a panic.
//! - [`BleBringup`]: the BLE module AT bring-up state machine (states 0..6) plus the transparent
//!   data-mode gate. While not in data mode the RX bytes feed only the state-0 line check, NOT the
//!   framer. After data mode the caller uses a `LinkPort` to feed the same framer.

use heapless::Vec;

use crate::frame::{self, DecodedFrame, FrameHeader, MAX_FRAME};

/// Maximum bytes drained from the serial in one `poll_rx` pass. Bounding the per-pass byte count
/// stops one task pass from starving the others in the cooperative scheduler: a continuously busy
/// line yields after this many bytes and resumes next pass.
pub const MAX_RX_PER_PASS: usize = 256;

/// Size of the stack buffer each `Read::read` call drains into. Small so it stays on the stack; the
/// drain loop fills it repeatedly until the line is empty or the per-pass cap is reached.
const RX_CHUNK: usize = 64;

/// Outcome of one `poll_rx` pass: how many bytes were drained this pass, whether the per-pass cap
/// was hit (so the caller knows more may be waiting), and how many line errors were seen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RxStatus {
    /// Total bytes read from the serial this pass.
    pub bytes: usize,
    /// True if the drain stopped because it reached [`MAX_RX_PER_PASS`] rather than because the
    /// line went empty. More bytes may be waiting for the next pass.
    pub capped: bool,
    /// Line errors observed during this pass (folded into the port's health counter too).
    pub errors: u32,
}

/// A transport adapter pairing a serial with the shared [`StreamFramer`] and a TX scratch buffer.
///
/// Generic over `S: embedded_io::Read + embedded_io::Write + embedded_io::ReadReady`, which the
/// concrete `runtime_hal::UsartSerial` satisfies.
pub struct LinkPort<S> {
    serial: S,
    framer: crate::framer::StreamFramer,
    tx_buf: Vec<u8, MAX_FRAME>,
    /// Cumulative line-error count (overrun / framing / parity). A health signal, not fatal.
    line_errors: u32,
}

impl<S> LinkPort<S>
where
    S: embedded_io::Read + embedded_io::Write + embedded_io::ReadReady,
{
    /// Wrap a serial in a fresh link port (empty framer, empty TX buffer, zero error count).
    pub fn new(serial: S) -> Self {
        Self {
            serial,
            framer: crate::framer::StreamFramer::new(),
            tx_buf: Vec::new(),
            line_errors: 0,
        }
    }

    /// Cumulative line errors seen on RX so far (health signal).
    pub fn line_errors(&self) -> u32 {
        self.line_errors
    }

    /// Borrow the underlying serial (e.g. to share it with a `BleBringup` before data mode).
    pub fn serial_mut(&mut self) -> &mut S {
        &mut self.serial
    }

    /// Reset the framer to the hunt state, dropping any partial frame.
    pub fn reset_framer(&mut self) {
        self.framer.reset();
    }

    /// Non-blocking RX drain. Loops while the serial reports `read_ready()`, reading available bytes
    /// into a stack buffer and feeding them to the framer, which calls `sink` once per complete
    /// CRC-valid frame.
    ///
    /// Properties:
    /// - Never blocks: it only reads after `read_ready()` reports true, and the bytes that follow a
    ///   ready signal are already available, so `Read::read` returns them without blocking. With no
    ///   bytes ready it returns immediately, dispatching nothing.
    /// - Bounded: at most [`MAX_RX_PER_PASS`] bytes per pass, so one pass cannot starve others.
    /// - Line errors (from `read_ready` or `read`) increment the health counter and end the pass;
    ///   they never panic.
    pub fn poll_rx(&mut self, sink: &mut impl FnMut(DecodedFrame)) -> RxStatus {
        let mut status = RxStatus::default();
        let mut chunk = [0u8; RX_CHUNK];

        loop {
            // Stop if we have hit the per-pass cap; flag it so the caller knows more may wait.
            if status.bytes >= MAX_RX_PER_PASS {
                status.capped = true;
                break;
            }

            // Only read when the line says a byte is ready; this is what keeps the drain
            // non-blocking. A line error here is counted and ends the pass.
            match self.serial.read_ready() {
                Ok(true) => {}
                Ok(false) => break,
                Err(_) => {
                    self.line_errors = self.line_errors.saturating_add(1);
                    status.errors = status.errors.saturating_add(1);
                    break;
                }
            }

            // Cap this read so the running total never exceeds MAX_RX_PER_PASS.
            let remaining = MAX_RX_PER_PASS - status.bytes;
            let want = remaining.min(RX_CHUNK);
            match self.serial.read(&mut chunk[..want]) {
                Ok(0) => break, // nothing actually came back; avoid spinning
                Ok(n) => {
                    status.bytes += n;
                    self.framer.feed(&chunk[..n], sink);
                }
                Err(_) => {
                    self.line_errors = self.line_errors.saturating_add(1);
                    status.errors = status.errors.saturating_add(1);
                    break;
                }
            }
        }

        status
    }

    /// Encode `hdr` + `payload` into the TX scratch buffer and write the whole frame to the serial.
    ///
    /// Returns the encoded frame length on success. Errors come from `frame::encode` (payload too
    /// long, buffer too small, which cannot happen for `tx_buf` sized at `MAX_FRAME`) or from the
    /// serial write, wrapped in [`SendError`].
    pub fn send_frame(&mut self, hdr: &FrameHeader, payload: &[u8]) -> Result<usize, SendError<S::Error>> {
        // tx_buf is fixed at MAX_FRAME capacity; resize to that so encode has a full buffer to
        // write into, then truncate to the actual encoded length for the write.
        self.tx_buf.clear();
        self.tx_buf
            .resize(MAX_FRAME, 0)
            .map_err(|_| SendError::Encode(frame::EncodeError::OutTooSmall))?;

        let n = frame::encode(hdr, payload, &mut self.tx_buf).map_err(SendError::Encode)?;
        self.serial
            .write_all(&self.tx_buf[..n])
            .map_err(SendError::Io)?;
        Ok(n)
    }
}

/// Why `send_frame` failed: either encoding the frame or the serial write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendError<E> {
    /// `frame::encode` rejected the header/payload (e.g. payload too long).
    Encode(frame::EncodeError),
    /// The underlying serial write failed (line error). Carries the serial's own error type.
    Io(E),
}

/// Default advertised BLE device name when the board definition does not override it. A sensible
/// default advertised name; a board/config field should override it.
pub const DEFAULT_DEVICE_NAME: &str = "Hoverboard";

/// The exact 7-byte probe reply that advances state 0.
const AT_OK: &[u8] = b"AT+OK\r\n";

/// Maximum device-name length the bring-up will embed in an `AT+NAME=` command. Bounds the AT TX
/// scratch buffer so it stays stack/`heapless`-sized.
pub const MAX_DEVICE_NAME: usize = 32;

/// Scratch buffer big enough for the longest AT command: `AT+NAME=` (8) + name + `\r\n` (2).
const AT_TX_CAP: usize = 8 + MAX_DEVICE_NAME + 2;

/// Line-match buffer: just long enough to recognise the 7-byte `AT+OK\r\n`. We only ever need to
/// confirm the most recent line is exactly that, so a tiny ring of the last few bytes suffices.
const AT_LINE_CAP: usize = AT_OK.len();

/// The BLE module AT bring-up state machine plus the transparent data-mode gate.
///
/// Generic over `S: embedded_io::Read + embedded_io::Write + embedded_io::ReadReady`. One `step`
/// advances at most one state; a slow (~248 ms) periodic task drives it so each AT command completes
/// and the reply arrives before the next step.
///
/// State table (matches ble-link.md):
///
/// | State | Action | Next |
/// |-------|--------|------|
/// | 0 | resend `AT\r\n`; advance only when the last RX line is exactly the 7 bytes `AT+OK\r\n` | 1 if matched, else 0 |
/// | 1 | send `AT+NAME=<device_name>\r\n` | 2 |
/// | 2 | send `AT+CONINTERVAL=16\r\n` | 3 |
/// | 3 | send `AT+ADVINTERVAL=32\r\n` | 4 |
/// | 4 | send `AT+SET=1\r\n` | 5 |
/// | 5 | send `AT+MODE=DATA\r\n` | 6 |
/// | 6 | terminal: set `data_mode = true`, do nothing | 6 |
///
/// While `!data_mode`, RX bytes feed only the state-0 line check (`pump_rx`); they are NOT fed to a
/// `StreamFramer`. After data mode the caller drives a [`LinkPort`] instead.
pub struct BleBringup<S> {
    serial: S,
    state: u8,
    data_mode: bool,
    /// Advertised device name, embedded into the `AT+NAME=` command at state 1.
    device_name: Vec<u8, MAX_DEVICE_NAME>,
    /// Rolling tail of recently received bytes, kept exactly `AT_OK.len()` long, so we can test
    /// whether the most recent line is `AT+OK\r\n`.
    line: Vec<u8, AT_LINE_CAP>,
}

impl<S> BleBringup<S>
where
    S: embedded_io::Read + embedded_io::Write + embedded_io::ReadReady,
{
    /// Bring-up with the default device name ([`DEFAULT_DEVICE_NAME`]).
    pub fn new(serial: S) -> Self {
        Self::with_device_name(serial, DEFAULT_DEVICE_NAME)
    }

    /// Bring-up advertising `device_name`. The name is truncated to [`MAX_DEVICE_NAME`] bytes.
    pub fn with_device_name(serial: S, device_name: &str) -> Self {
        let mut name: Vec<u8, MAX_DEVICE_NAME> = Vec::new();
        for &b in device_name.as_bytes().iter().take(MAX_DEVICE_NAME) {
            // Capacity is MAX_DEVICE_NAME and we take at most that many, so this cannot fail.
            let _ = name.push(b);
        }
        Self {
            serial,
            state: 0,
            data_mode: false,
            device_name: name,
            line: Vec::new(),
        }
    }

    /// True once the module has been switched into transparent data mode (state reached 6).
    pub fn is_data_mode(&self) -> bool {
        self.data_mode
    }

    /// Current state index (0..=6). Exposed for tests and supervision.
    pub fn state(&self) -> u8 {
        self.state
    }

    /// Borrow the underlying serial (e.g. to hand it to a `LinkPort` after data mode).
    pub fn serial_mut(&mut self) -> &mut S {
        &mut self.serial
    }

    /// Feed received bytes into the state-0 line check ONLY. This is the inert-framer path: while
    /// not in data mode the `AT+OK\r\n` reply is consumed here, never by a `StreamFramer`. Keeps a
    /// rolling tail of the last `AT_OK.len()` bytes so `step` can test the most recent line.
    ///
    /// A no-op once in data mode (the caller routes data-mode RX to the framer via `LinkPort`).
    pub fn pump_rx(&mut self, bytes: &[u8]) {
        if self.data_mode {
            return;
        }
        for &b in bytes {
            if self.line.is_full() {
                // Shift left by one to keep only the most recent AT_OK.len() bytes.
                self.line.remove(0);
            }
            // After the possible remove there is always room.
            let _ = self.line.push(b);
        }
    }

    /// True if the rolling line tail is exactly the 7 bytes `AT+OK\r\n`.
    fn line_is_at_ok(&self) -> bool {
        self.line.len() == AT_OK.len() && &self.line[..] == AT_OK
    }

    /// Advance the bring-up by at most one state.
    ///
    /// State 0 resends `AT\r\n` and stays until `pump_rx` has delivered an exact `AT+OK\r\n`; states
    /// 1..5 each send their AT command and advance unconditionally; state 6 sets `data_mode` and is
    /// terminal (sends nothing further). Returns the (possibly unchanged) state after the step.
    ///
    /// Serial-write errors are swallowed (the slow pacing retries naturally); they do not panic and
    /// do not advance state on their own.
    pub fn step(&mut self) -> u8 {
        match self.state {
            0 => {
                if self.line_is_at_ok() {
                    self.state = 1;
                } else {
                    let _ = self.serial.write_all(b"AT\r\n");
                }
            }
            1 => {
                let mut cmd: Vec<u8, AT_TX_CAP> = Vec::new();
                let _ = cmd.extend_from_slice(b"AT+NAME=");
                let _ = cmd.extend_from_slice(&self.device_name);
                let _ = cmd.extend_from_slice(b"\r\n");
                let _ = self.serial.write_all(&cmd);
                self.state = 2;
            }
            2 => {
                let _ = self.serial.write_all(b"AT+CONINTERVAL=16\r\n");
                self.state = 3;
            }
            3 => {
                let _ = self.serial.write_all(b"AT+ADVINTERVAL=32\r\n");
                self.state = 4;
            }
            4 => {
                let _ = self.serial.write_all(b"AT+SET=1\r\n");
                self.state = 5;
            }
            5 => {
                let _ = self.serial.write_all(b"AT+MODE=DATA\r\n");
                self.state = 6;
            }
            _ => {
                // State 6 (or any out-of-range value): terminal. Set the data-mode gate and go
                // quiet. From here the caller uses a LinkPort to feed the shared framer.
                self.state = 6;
                self.data_mode = true;
            }
        }
        self.state
    }
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{encode, FrameHeader, MAX_FRAME, PROTO_VER};
    use crate::opcode::Opcode;
    use core::cell::RefCell;
    use std::rc::Rc;

    /// In-memory bidirectional serial backing both ends of a transport test.
    ///
    /// The shared inner holds an RX queue (bytes the device "received", which `Read` drains) and a
    /// TX log (bytes the code under test "wrote", which `Write` appends). `Error = Infallible`
    /// because nothing in memory can fail; `embedded_io` provides `Error for Infallible`.
    #[derive(Default)]
    struct Inner {
        rx: std::collections::VecDeque<u8>,
        tx: Vec<u8, 1024>,
    }

    #[derive(Clone)]
    struct MockSerial {
        inner: Rc<RefCell<Inner>>,
    }

    impl MockSerial {
        fn new() -> Self {
            Self { inner: Rc::new(RefCell::new(Inner::default())) }
        }

        /// Queue bytes for the code under test to read.
        fn push_rx(&self, bytes: &[u8]) {
            let mut inner = self.inner.borrow_mut();
            for &b in bytes {
                inner.rx.push_back(b);
            }
        }

        /// Snapshot everything written so far.
        fn tx_log(&self) -> std::vec::Vec<u8> {
            self.inner.borrow().tx.iter().copied().collect()
        }

        /// Clear the TX log (between bring-up steps).
        fn clear_tx(&self) {
            self.inner.borrow_mut().tx.clear();
        }
    }

    impl embedded_io::ErrorType for MockSerial {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Read for MockSerial {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let mut inner = self.inner.borrow_mut();
            let mut n = 0;
            while n < buf.len() {
                match inner.rx.pop_front() {
                    Some(b) => {
                        buf[n] = b;
                        n += 1;
                    }
                    None => break,
                }
            }
            Ok(n)
        }
    }

    impl embedded_io::ReadReady for MockSerial {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.inner.borrow().rx.is_empty())
        }
    }

    impl embedded_io::Write for MockSerial {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let mut inner = self.inner.borrow_mut();
            for &b in buf {
                // Test buffer is generous; ignore the rare overflow rather than fail the trait.
                let _ = inner.tx.push(b);
            }
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    fn make_frame(opcode: Opcode, payload: &[u8], src: u8, dst: u8) -> ([u8; MAX_FRAME], usize) {
        let mut out = [0u8; MAX_FRAME];
        let hdr = FrameHeader { ver: PROTO_VER, opcode, src, dst, len: payload.len() as u8 };
        let n = encode(&hdr, payload, &mut out).unwrap();
        (out, n)
    }

    /// Count frames dispatched by a `poll_rx`, collecting their opcodes for assertions.
    fn collect_opcodes(port: &mut LinkPort<MockSerial>) -> std::vec::Vec<Opcode> {
        let mut ops = std::vec::Vec::new();
        port.poll_rx(&mut |f| ops.push(f.header.opcode));
        ops
    }

    #[test]
    fn poll_rx_frame_split_across_two_chunks() {
        let serial = MockSerial::new();
        let mut port = LinkPort::new(serial.clone());
        let (buf, n) = make_frame(Opcode::DriveCmd, &[1, 2, 3], 1, 2);

        // First chunk: only part of the frame is ready. Drain it; framer holds the partial frame.
        serial.push_rx(&buf[..4]);
        let ops = collect_opcodes(&mut port);
        assert_eq!(ops.len(), 0, "partial frame must not dispatch");

        // Second chunk: the rest arrives. Now exactly one frame dispatches.
        serial.push_rx(&buf[4..n]);
        let ops = collect_opcodes(&mut port);
        assert_eq!(ops, std::vec![Opcode::DriveCmd]);
    }

    #[test]
    fn poll_rx_two_coalesced_frames_dispatch_twice() {
        let serial = MockSerial::new();
        let mut port = LinkPort::new(serial.clone());
        let (a, an) = make_frame(Opcode::CyclicState, &[9], 1, 2);
        let (b, bn) = make_frame(Opcode::DriveCmd, &[7, 7], 3, 4);

        serial.push_rx(&a[..an]);
        serial.push_rx(&b[..bn]);
        let ops = collect_opcodes(&mut port);
        assert_eq!(ops, std::vec![Opcode::CyclicState, Opcode::DriveCmd]);
    }

    #[test]
    fn poll_rx_corrupted_frame_dispatches_zero() {
        let serial = MockSerial::new();
        let mut port = LinkPort::new(serial.clone());
        let (mut buf, n) = make_frame(Opcode::DriveCmd, &[1, 2, 3], 1, 2);
        buf[crate::frame::HEADER_LEN] ^= 0xFF; // corrupt payload, leave CRC

        serial.push_rx(&buf[..n]);
        let ops = collect_opcodes(&mut port);
        assert_eq!(ops.len(), 0, "corrupted frame must drop with no dispatch");
    }

    #[test]
    fn poll_rx_nonblocking_returns_immediately_when_empty() {
        let serial = MockSerial::new();
        let mut port = LinkPort::new(serial.clone());

        // Nothing queued: read_ready() is false on the first check, so poll_rx takes the
        // false-path immediately, reads nothing, dispatches nothing, and does not hang.
        let mut dispatched = 0;
        let status = port.poll_rx(&mut |_| dispatched += 1);
        assert_eq!(status.bytes, 0);
        assert!(!status.capped);
        assert_eq!(dispatched, 0);
    }

    #[test]
    fn poll_rx_bounded_per_pass() {
        let serial = MockSerial::new();
        let mut port = LinkPort::new(serial.clone());

        // Queue more raw bytes than the per-pass cap (garbage; we only care about the byte count).
        let flood = [0u8; MAX_RX_PER_PASS + 100];
        serial.push_rx(&flood);

        let status = port.poll_rx(&mut |_| {});
        assert_eq!(status.bytes, MAX_RX_PER_PASS, "one pass must stop at the cap");
        assert!(status.capped, "hitting the cap must be flagged");
    }

    #[test]
    fn send_frame_writes_whole_encoded_frame() {
        let serial = MockSerial::new();
        let mut port = LinkPort::new(serial.clone());
        let hdr = FrameHeader { ver: PROTO_VER, opcode: Opcode::Telemetry, src: 1, dst: 2, len: 0 };
        let n = port.send_frame(&hdr, &[0xAA, 0xBB]).unwrap();

        let log = serial.tx_log();
        assert_eq!(log.len(), n);
        // The written bytes must decode back to the same frame.
        let f = crate::frame::decode(&log).unwrap();
        assert_eq!(f.header.opcode, Opcode::Telemetry);
        assert_eq!(f.payload, &[0xAA, 0xBB]);
    }

    #[test]
    fn ble_bringup_full_sequence() {
        let serial = MockSerial::new();
        let mut ble = BleBringup::with_device_name(serial.clone(), "MY BOARD");
        assert!(!ble.is_data_mode());

        // State 0 with no RX: resends AT\r\n and stays at 0.
        serial.clear_tx();
        assert_eq!(ble.step(), 0);
        assert_eq!(serial.tx_log(), b"AT\r\n");

        // State 0 with garbage RX: still resends AT\r\n, still at 0.
        ble.pump_rx(b"junk\r\n");
        serial.clear_tx();
        assert_eq!(ble.step(), 0);
        assert_eq!(serial.tx_log(), b"AT\r\n");

        // Feed the exact 7-byte AT+OK\r\n: the next step advances 0 -> 1 (guard satisfied). State 0
        // sends nothing on the advancing step; the AT+NAME goes out on the state-1 step.
        ble.pump_rx(b"AT+OK\r\n");
        serial.clear_tx();
        assert_eq!(ble.step(), 1);
        assert_eq!(serial.tx_log().len(), 0, "the 0 -> 1 advance sends nothing");

        // State 1 emits AT+NAME with the configured custom name, advancing to 2.
        serial.clear_tx();
        assert_eq!(ble.step(), 2);
        assert_eq!(serial.tx_log(), b"AT+NAME=MY BOARD\r\n");

        // States 2..5 emit the remaining config commands in order, one per step.
        serial.clear_tx();
        assert_eq!(ble.step(), 3);
        assert_eq!(serial.tx_log(), b"AT+CONINTERVAL=16\r\n");

        serial.clear_tx();
        assert_eq!(ble.step(), 4);
        assert_eq!(serial.tx_log(), b"AT+ADVINTERVAL=32\r\n");

        serial.clear_tx();
        assert_eq!(ble.step(), 5);
        assert_eq!(serial.tx_log(), b"AT+SET=1\r\n");

        serial.clear_tx();
        assert_eq!(ble.step(), 6);
        assert_eq!(serial.tx_log(), b"AT+MODE=DATA\r\n");
        // The AT+MODE=DATA step moved to state 6 but data_mode is not yet set; the next (terminal)
        // step sets the gate.
        assert!(!ble.is_data_mode());

        // State 6: terminal, sets data_mode, goes quiet.
        serial.clear_tx();
        assert_eq!(ble.step(), 6);
        assert!(ble.is_data_mode());
        assert_eq!(serial.tx_log().len(), 0, "state 6 must be quiet");

        // Further steps stay terminal and quiet.
        serial.clear_tx();
        assert_eq!(ble.step(), 6);
        assert_eq!(serial.tx_log().len(), 0);
    }

    #[test]
    fn ble_state0_requires_exact_at_ok() {
        let serial = MockSerial::new();
        let mut ble = BleBringup::new(serial.clone());

        // A near-miss line (extra prefix that pushes AT+OK out of the 7-byte window is fine, but a
        // line that is not exactly AT+OK\r\n must not advance). Feed "XAT+OK\r" then a stray byte.
        ble.pump_rx(b"AT+OK\r"); // 6 bytes, not yet the trailing \n
        assert_eq!(ble.step(), 0, "incomplete reply must not advance");

        // Now complete it. Because pump_rx keeps only the last 7 bytes, feeding the final \n leaves
        // exactly AT+OK\r\n in the window.
        ble.pump_rx(b"\n");
        assert_eq!(ble.step(), 1, "exact AT+OK\\r\\n must advance");
    }

    #[test]
    fn ble_pump_rx_keeps_only_recent_tail() {
        let serial = MockSerial::new();
        let mut ble = BleBringup::new(serial.clone());
        // A long preamble followed by the exact reply must still match (rolling tail).
        ble.pump_rx(b"garbage garbage garbage AT+OK\r\n");
        assert_eq!(ble.step(), 1);
    }

    #[test]
    fn ble_default_device_name() {
        let serial = MockSerial::new();
        let mut ble = BleBringup::new(serial.clone());
        ble.pump_rx(b"AT+OK\r\n");
        ble.step(); // advance to state 1
        serial.clear_tx();
        ble.step(); // emit AT+NAME at state 1 -> 2
        assert_eq!(serial.tx_log(), b"AT+NAME=Hoverboard\r\n");
    }
}
