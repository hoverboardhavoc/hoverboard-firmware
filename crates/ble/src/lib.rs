//! BLE module crate (`ble`): a portable, `no_std`, HAL-free driver for the onboard AT-command BLE
//! module (the bench's CC2541-class transparent bridge).
//!
//! One definition of the module's behavior, shared by parties that otherwise share nothing (the main
//! firmware, the bootloader, and the codegen'd web configurator): the AT bring-up to transparent data
//! mode, the data-mode gate, and the module contract constants (GATT discovery hints, AT strings, the
//! baud). See `specs/ble.md`.
//!
//! HAL-free: the crate is generic over `embedded-io` (`Read + Write + ReadReady`) and `embedded-hal`
//! (`DelayNs`) traits, NOT over `runtime-hal`. [`Module::bring_up`] takes the caller's serial, owns the
//! AT command sequence and reply parsing, then returns a [`Pipe`] that passes bytes through. The
//! firmware (or the loopback test image) owns the serial; this crate stays HAL-free.
//!
//! Two types, one per phase:
//! - [`Module`] is the configurable device BEFORE data mode: settings only (name, intervals), no I/O.
//! - [`Pipe`] is the data-mode channel: a transparent UART over BLE that wraps the inner serial and
//!   implements the same `embedded-io` traits, so it is used exactly like a UART.
//!
//! The AT sequence and the exact-7-byte `AT+OK\r\n` probe match are grounded on the Ghidra decompile of
//! the stock F103 firmware (the order is `SET=1` then `MODE=DATA`); see `specs/ble.md` for provenance.

#![no_std]
// The host test harness needs std (formatting, Vec-backed stub serials); the crate itself is no_std.
#[cfg(test)]
extern crate std;

use embedded_hal::delay::DelayNs;
use embedded_io::{Read, ReadReady, Write};

/// GATT discovery hints. The module is a transparent bridge and the firmware does NO GATT itself: the
/// central discovers the characteristic at runtime (prefer this service/characteristic, else walk every
/// service for the first characteristic with both `WRITE_WITHOUT_RESPONSE`/`WRITE` and `NOTIFY`). These
/// are HINTS, not a pinned contract (`specs/ble.md`, "Module contract constants").
pub mod gatt {
    /// Preferred service UUID to look for first (16-bit). A discovery hint, not guaranteed.
    pub const SERVICE_HINT: u16 = 0xFFE0;
    /// Preferred characteristic UUID to look for first (16-bit). A discovery hint, not guaranteed.
    pub const CHAR_HINT: u16 = 0xFFE1;
}

/// The AT command contract: the exact bytes the bring-up sends and the exact probe reply it matches.
///
/// Grounded on the Ghidra decompile of the stock F103 firmware (`bluetooth.c` `bt_send_AT_*`); the
/// underscore in `CON_INTERVAL`/`ADV_INTERVAL` is real (`0x5f`). The interval commands here are the
/// prefixes; [`Module::bring_up`] formats the configured integer and the trailing `\r\n` after them.
pub mod at {
    /// The probe sent in state 0: `AT\r\n`.
    pub const PROBE: &[u8] = b"AT\r\n";
    /// The EXACT 7-byte reply that advances past the probe: `AT+OK\r\n`. A bare `OK` or a longer line
    /// must NOT advance (`specs/ble.md`, "Probe match").
    pub const OK_REPLY: &[u8] = b"AT+OK\r\n";
    /// `AT+NAME=` prefix; the configured name and `\r\n` follow.
    pub const NAME_PREFIX: &[u8] = b"AT+NAME=";
    /// `AT+CON_INTERVAL=` prefix (the underscore is `0x5f`); the configured integer and `\r\n` follow.
    pub const CON_INTERVAL_PREFIX: &[u8] = b"AT+CON_INTERVAL=";
    /// `AT+ADV_INTERVAL=` prefix (the underscore is `0x5f`); the configured integer and `\r\n` follow.
    pub const ADV_INTERVAL_PREFIX: &[u8] = b"AT+ADV_INTERVAL=";
    /// `AT+SET=1\r\n`. Sent BEFORE `MODE=DATA` (the stock state machine's case 0x04 -> 0x05).
    pub const SET: &[u8] = b"AT+SET=1\r\n";
    /// `AT+MODE=DATA\r\n`. The terminal command that switches the module to transparent data mode.
    pub const MODE_DATA: &[u8] = b"AT+MODE=DATA\r\n";
    /// The line terminator appended after the formatted interval/name values.
    pub const CRLF: &[u8] = b"\r\n";

    /// The module's fixed operating (data-mode) baud, 8N1. The caller builds its serial at this rate;
    /// [`super::Module`] carries no baud setter (reconfiguring via `AT+BAUD` is unbuilt, `specs/ble.md`).
    pub const BAUD: u32 = 9600;

    /// Default `CON_INTERVAL` decimal value.
    pub const DEFAULT_CON_INTERVAL: u16 = 16;
    /// Default `ADV_INTERVAL` decimal value.
    pub const DEFAULT_ADV_INTERVAL: u16 = 32;
}

/// Bring-up pacing: the per-step RX drain window. Each AT command's `AT+OK\r\n` ack is consumed within
/// this window (and it paces the next command), ~248 ms/step (`specs/ble.md`).
const STEP_MS: u32 = 248;

/// The short drain window after `MODE=DATA`: long enough to consume that command's `AT+OK` ack (so it
/// does not leak into the transparent byte stream), short enough not to swallow real data. After
/// `MODE=DATA` the module is a transparent bridge, so reading is stopped here (the data-mode gate).
const MODE_DRAIN_MS: u32 = 120;

/// RX poll granularity while draining: ~200 us, i.e. faster than the ~1 ms/byte wire rate at 9600 8N1, so
/// every ack byte is pulled from the 1-byte RX register before the next overruns it.
const POLL_US: u32 = 200;
/// Polls per millisecond at [`POLL_US`] (used to size a drain window in poll iterations).
const POLLS_PER_MS: u32 = 1000 / POLL_US;

/// How many `STEP_MS` ticks the probe (state 0) is retried before giving up with [`Error::Probe`]. The
/// probe is the only state that waits on a reply; a silent module fails here rather than hanging. The
/// bench probe locked `AT+OK` on the first reply, so a generous budget still fails fast on true silence.
const PROBE_RETRIES: u32 = 20;

/// Bring-up error. `Probe` means the exact 7-byte `AT+OK\r\n` was never seen within the budget (a silent
/// or wedged module); `Serial` wraps the inner serial's error.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// The probe reply `AT+OK\r\n` never arrived within the bring-up budget. The caller power-cycles the
    /// module and retries (re-entry from data mode is not a supported cold-boot path, `specs/ble.md`).
    Probe,
    /// The inner serial returned an error during the AT sequence.
    Serial(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Serial(e)
    }
}

/// The configurable BLE module: settings only, no I/O. Built before bring-up (the configuration API).
///
/// `name` is a config value (never a baked product name). `con_interval`/`adv_interval` default to
/// 16/32 and are formatted into the `AT+CON_INTERVAL=`/`AT+ADV_INTERVAL=` commands. The `SET=1` /
/// `MODE=DATA` order is internal (a fixed constant). There is no baud setter: the operating baud is the
/// fixed known constant ([`at::BAUD`]) and the caller builds its serial at that rate.
pub struct Module<'a> {
    name: &'a str,
    con_interval: u16,
    adv_interval: u16,
}

impl<'a> Module<'a> {
    /// A module advertised as `name` (a config value), with the default intervals (CON=16, ADV=32).
    pub fn new(name: &'a str) -> Self {
        Self {
            name,
            con_interval: at::DEFAULT_CON_INTERVAL,
            adv_interval: at::DEFAULT_ADV_INTERVAL,
        }
    }

    /// Set the connection interval formatted into `AT+CON_INTERVAL=<n>\r\n` (default 16).
    pub fn con_interval(mut self, n: u16) -> Self {
        self.con_interval = n;
        self
    }

    /// Set the advertising interval formatted into `AT+ADV_INTERVAL=<n>\r\n` (default 32).
    pub fn adv_interval(mut self, n: u16) -> Self {
        self.adv_interval = n;
        self
    }

    /// Apply the settings over `serial`, run AT to transparent data mode (`SET=1` then `MODE=DATA`), and
    /// return the [`Pipe`]. Blocking, boot-time.
    ///
    /// The state machine (`specs/ble.md`):
    /// - state 0: send `AT\r\n`, advance ONLY on the exact 7-byte `AT+OK\r\n` ack, else resend (up to the
    ///   probe budget; a silent module fails [`Error::Probe`] rather than hanging),
    /// - state 1: `AT+NAME=<name>\r\n`,
    /// - state 2: `AT+CON_INTERVAL=<n>\r\n`,
    /// - state 3: `AT+ADV_INTERVAL=<n>\r\n`,
    /// - state 4: `AT+SET=1\r\n`,
    /// - state 5: `AT+MODE=DATA\r\n`,
    /// - state 6: terminal -> the `Pipe` is returned (the data-mode gate: no `Pipe` exists before this).
    ///
    /// The module ACKs EVERY command with `AT+OK\r\n` (confirmed on silicon 2026-06-25). Each step must
    /// CONSUME its ack: [`drain_until_ok`] polls RX promptly (at ~[`POLL_US`], faster than the ~1 ms/byte
    /// wire rate, so the 7-byte ack is never lost to the 1-byte RX register's overrun) and discards through
    /// the step window. Not draining the acks leaves the config commands without effect (the name and
    /// intervals never take) even though the bytes are transmitted correctly -- the bug the original
    /// blind-`delay_ms` version had, proven against `runtime-hal`'s polled serial on the bench. After
    /// `MODE=DATA` the drain is kept short ([`MODE_DRAIN_MS`]) and reading then stops: the module is
    /// transparent from there, so further bytes are DATA, not acks, and must not be consumed.
    pub fn bring_up<S, D>(self, mut serial: S, delay: &mut D) -> Result<Pipe<S>, Error<S::Error>>
    where
        S: Read + Write + ReadReady,
        D: DelayNs,
    {
        // State 0: probe until the exact 7-byte AT+OK\r\n ack, draining PROMPTLY (no late single read that
        // would overrun the 1-byte RX register), else resend. Pre-DATA, RX bytes feed only the detector.
        let mut matched = false;
        for _ in 0..PROBE_RETRIES {
            serial.write_all(at::PROBE)?;
            serial.flush()?;
            if drain_until_ok(&mut serial, delay, STEP_MS)? {
                matched = true;
                break;
            }
        }
        if !matched {
            return Err(Error::Probe);
        }

        // States 1..4: send each command and CONSUME its AT+OK ack -- the draining is what makes the
        // config stick. A missing ack is not fatal here (drain_until_ok times out and we proceed), but the
        // draining (and pacing) is the point. The interval values are formatted in; SET=1 precedes MODE.
        serial.write_all(at::NAME_PREFIX)?;
        serial.write_all(self.name.as_bytes())?;
        serial.write_all(at::CRLF)?;
        serial.flush()?;
        drain_until_ok(&mut serial, delay, STEP_MS)?;

        write_interval(&mut serial, at::CON_INTERVAL_PREFIX, self.con_interval)?;
        serial.flush()?;
        drain_until_ok(&mut serial, delay, STEP_MS)?;

        write_interval(&mut serial, at::ADV_INTERVAL_PREFIX, self.adv_interval)?;
        serial.flush()?;
        drain_until_ok(&mut serial, delay, STEP_MS)?;

        serial.write_all(at::SET)?;
        serial.flush()?;
        // SET=1 double-acks (`AT+OK\r\nAT+OK\r\n` on silicon); the full-window drain clears both.
        drain_until_ok(&mut serial, delay, STEP_MS)?;

        // State 5: MODE=DATA. Consume ITS ack with a SHORT drain, then STOP -- the module is transparent
        // after this, so a longer read would swallow real data. The short drain also keeps the ack out of
        // the echoed byte stream.
        serial.write_all(at::MODE_DATA)?;
        serial.flush()?;
        drain_until_ok(&mut serial, delay, MODE_DRAIN_MS)?;

        // State 6: terminal. The module is now a transparent bridge; hand back the data-mode pipe.
        Ok(Pipe { serial })
    }
}

/// Quick liveness probe for the BT-probe bring-up phase (`specs/l3.md`, "Unconfigured bring-up"): send
/// up to `tries` `AT\r\n` and report whether the exact `AT+OK\r\n` answers, WITHOUT consuming the
/// serial or running the full [`Module::bring_up`]. The firmware probes each whitelisted USART with
/// this; only the one that answers is then handed to [`Module::bring_up`]. Cheap (a few short windows)
/// so a board with no module does not spend the full bring-up probe budget on every whitelisted port.
///
/// Nothing but a CC2541-class module answers `AT`, so a `true` is unambiguous. A peer board on an
/// inter-board UART, or the un-initialised IMU on USART0-remap, answers neither (it classifies
/// `empty`), so `probe` returns `false` and that port goes to the passive link-listen phase.
pub fn probe<S, D>(serial: &mut S, delay: &mut D, tries: u32) -> Result<bool, Error<S::Error>>
where
    S: Read + Write + ReadReady,
    D: DelayNs,
{
    for _ in 0..tries {
        serial.write_all(at::PROBE)?;
        serial.flush()?;
        if drain_until_ok(serial, delay, STEP_MS)? {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Poll RX promptly through a `budget_ms` window, discarding every byte, and report whether the exact
/// 7-byte `AT+OK\r\n` ack appeared anywhere in the drained stream.
///
/// The module acks every command with `AT+OK\r\n`; CONSUMING the ack is required for the config to take
/// (proven on silicon, `specs/ble.md`). Polling at ~[`POLL_US`] (faster than the ~1 ms/byte rate at 9600)
/// pulls each byte out of the 1-byte RX register before the next overruns it -- unlike a single delayed
/// read, which drops all but the last byte. A 7-byte sliding window matches the exact `AT+OK\r\n` (so a
/// bare `OK` still never matches), while draining the whole window also clears the `AT+SET=1` double-ack.
fn drain_until_ok<S, D>(
    serial: &mut S,
    delay: &mut D,
    budget_ms: u32,
) -> Result<bool, Error<S::Error>>
where
    S: Read + ReadReady,
    D: DelayNs,
{
    let mut window = [0u8; at::OK_REPLY.len()];
    let mut filled = 0usize;
    let mut saw_ok = false;
    let polls = budget_ms.saturating_mul(POLLS_PER_MS);
    for _ in 0..polls {
        if serial.read_ready()? {
            let mut one = [0u8; 1];
            if serial.read(&mut one)? == 1 {
                if filled < window.len() {
                    window[filled] = one[0];
                    filled += 1;
                } else {
                    window.rotate_left(1);
                    window[window.len() - 1] = one[0];
                }
                if filled == window.len() && window == *at::OK_REPLY {
                    saw_ok = true;
                }
            }
        } else {
            delay.delay_us(POLL_US);
        }
    }
    Ok(saw_ok)
}

/// Format `<prefix><n>\r\n` onto the serial (e.g. `AT+CON_INTERVAL=16\r\n`), no alloc.
fn write_interval<S>(serial: &mut S, prefix: &[u8], n: u16) -> Result<(), Error<S::Error>>
where
    S: Write,
{
    serial.write_all(prefix)?;
    // u16 is at most 5 decimal digits; format into a small stack buffer, no alloc.
    let mut digits = [0u8; 5];
    let mut i = digits.len();
    let mut v = n;
    if v == 0 {
        i -= 1;
        digits[i] = b'0';
    } else {
        while v > 0 {
            i -= 1;
            digits[i] = b'0' + (v % 10) as u8;
            v /= 10;
        }
    }
    serial.write_all(&digits[i..])?;
    serial.write_all(at::CRLF)?;
    Ok(())
}

/// The data-mode channel: a transparent UART over BLE that wraps the inner serial `S` and implements the
/// same `embedded-io` traits (`Read`/`Write`/`ReadReady`), so it is used exactly like a UART.
///
/// `Pipe` is the **data-mode gate type**, and it survives into the link layer (`specs/ble.md`, "The
/// data-mode gate type"): the firmware's BLE L2 link is built over the `Pipe`, never over the raw
/// serial, so a link can only exist on a serial that is KNOWN to be in transparent data mode. Exactly
/// two constructors establish that knowledge:
/// - [`Module::bring_up`] completing the AT handshake (state 6, the handshake arm), or
/// - [`Pipe::assume_data_mode`], where data mode is already established knowledge (the `l3.md`
///   configured-boot fallback: the persisted link-set identifies the port as the BLE module and the
///   patient AT probe answered nothing, i.e. a warm module still in data mode).
pub struct Pipe<S> {
    serial: S,
}

impl<S> Pipe<S> {
    /// The fallback-arm constructor: wrap a serial whose data-mode state is ALREADY established
    /// knowledge, without a handshake. This is not a bypass of the gate but its second honest arm
    /// (`specs/ble.md`): the caller asserts a fact it holds from elsewhere (the l3.md link-set + a
    /// patient AT probe that answered nothing = a warm module still transparent). A caller that
    /// cannot justify that assertion must run [`Module::bring_up`] instead.
    pub fn assume_data_mode(serial: S) -> Self {
        Pipe { serial }
    }

    /// Consume the pipe and return the inner serial. Introspection/teardown only (tests unwrap the
    /// stub serial): the DATA PATH keeps the `Pipe` (`specs/ble.md`, the gate type survives into the
    /// link layer; nothing unwraps it to build a link).
    pub fn into_inner(self) -> S {
        self.serial
    }
}

impl<S: embedded_io::ErrorType> embedded_io::ErrorType for Pipe<S> {
    type Error = S::Error;
}

impl<S: Read> Read for Pipe<S> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.serial.read(buf)
    }
}

impl<S: Write> Write for Pipe<S> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.serial.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.serial.flush()
    }
}

impl<S: ReadReady> ReadReady for Pipe<S> {
    /// Delegates to the inner serial: the pipe is transparent, readiness included. What lets
    /// `link::SerialTransport` (which is `ReadReady`-gated) ride the gate type directly.
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.serial.read_ready()
    }
}

#[cfg(test)]
mod tests;
