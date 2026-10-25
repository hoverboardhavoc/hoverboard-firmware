//! The SWD engine (`specs/hoverboard-bridge.md`, "SWD engine"): one module owns the pins and every transfer.
//!
//! A port of `Gigadump/gdf130backup/main/swd.c` (reimplemented, not linked), with its gaps fixed:
//!
//! - **Read parity is checked.** A debug port read with bad parity is retried on the spot (those
//!   reads are idempotent); a memory access port read with bad parity has already moved `TAR`, so
//!   the memory operation restarts from the last word that arrived intact.
//! - **ACK handling.** `WAIT` retries the same request (up to [`SwdConfig::wait_retries`]); `FAULT`
//!   writes `ABORT` = `0x1E` and the operation restarts; no ACK is a line fault, answered with a line
//!   reset and reconnect before the operation restarts. Restarts are bounded by a count without
//!   progress ([`SwdConfig::op_retries`]) and a deadline without progress
//!   ([`SwdConfig::deadline_us`]), so a dead line returns an error in bounded time.
//! - **Counters.** Every `WAIT`, `FAULT`, parity error and reconnect is counted ([`Counters`]).
//!
//! The pins are behind [`SwdPins`], so the host tests run this exact code, bit for bit, against a
//! simulated debug port (`crate::sim`).

use core::fmt;

/// The pin and time operations the engine needs. On the ESP32-C3 these are GPIO6 (SWCLK) and GPIO7
/// (SWDIO) through the GPIO set/clear registers; in the host tests, a simulated target.
pub trait SwdPins {
    /// Drive SWDIO from the host.
    fn swdio_out(&mut self);
    /// Release SWDIO (the target drives it).
    fn swdio_in(&mut self);
    /// Put `bit` on SWDIO (driven) and run one clock: SWCLK low, half a period, high, half a period.
    fn write_bit(&mut self, bit: bool);
    /// Run one clock and sample SWDIO in the low half: low, half a period, sample, high, half a
    /// period.
    fn read_bit(&mut self) -> bool;
    /// One clock with no data: a turnaround.
    fn clock(&mut self);
    /// Monotonic microseconds.
    fn now_us(&mut self) -> u64;
}

/// The retry budget.
#[derive(Debug, Clone, Copy)]
pub struct SwdConfig {
    /// `WAIT` answers in a row before a transfer gives up (`swd.c` uses 256).
    pub wait_retries: u32,
    /// Restarts of one memory operation without progress before it gives up.
    pub op_retries: u32,
    /// Time without progress before a memory operation gives up, in microseconds.
    pub deadline_us: u64,
}

impl Default for SwdConfig {
    fn default() -> Self {
        SwdConfig {
            wait_retries: 256,
            op_retries: 32,
            deadline_us: 250_000,
        }
    }
}

/// What the engine has seen since boot. `ESP_INFO` reports them.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    /// `WAIT` answers (each one retried).
    pub waits: u32,
    /// `FAULT` answers (each one answered with an `ABORT`).
    pub faults: u32,
    /// Reads whose parity bit was wrong.
    pub parity_errors: u32,
    /// Line resets and reconnects after a transfer got no ACK.
    pub reconnects: u32,
}

/// What one transfer saw when it did not complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XferError {
    /// `WAIT`, more times in a row than the budget allows.
    Wait,
    /// `FAULT`: a sticky error flag is set.
    Fault,
    /// No valid ACK: a line fault, or no target.
    NoAck,
    /// A read whose parity bit was wrong.
    Parity,
}

/// Why an SWD operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwdError {
    /// The restart count or the deadline ran out; the fault is what the last attempt saw.
    Exhausted(XferError),
    /// The debug power-up request was never acknowledged.
    PowerUp,
}

impl fmt::Display for SwdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SwdError::Exhausted(fault) => write!(f, "SWD retries exhausted (last: {fault:?})"),
            SwdError::PowerUp => write!(f, "SWD debug power-up not acknowledged"),
        }
    }
}

// Debug port and memory access port registers (ADIv5).
const DP_DPIDR: u8 = 0x0; // read
const DP_ABORT: u8 = 0x0; // write
const DP_CTRL_STAT: u8 = 0x4;
const DP_SELECT: u8 = 0x8;
const DP_RDBUFF: u8 = 0xC;
const AP_CSW: u8 = 0x0;
const AP_TAR: u8 = 0x4;
const AP_DRW: u8 = 0xC;

/// `ABORT` value that clears every sticky flag: `ORUNERRCLR | WDERRCLR | STKERRCLR | STKCMPCLR`.
pub const ABORT_CLEAR: u32 = 0x1E;
/// `CTRL/STAT` power-up requests.
const CSYSPWRUPREQ: u32 = 1 << 30;
const CDBGPWRUPREQ: u32 = 1 << 28;
const CSYSPWRUPACK: u32 = 1 << 31;
const CDBGPWRUPACK: u32 = 1 << 29;
/// `CTRL/STAT` sticky error and write data error.
const STICKYERR: u32 = 1 << 5;
const WDATAERR: u32 = 1 << 7;
/// `CSW`: 32-bit, single auto-increment.
pub const CSW_WORD: u32 = 0x2300_0052;
/// `CSW`: 16-bit, no increment.
pub const CSW_HALF: u32 = 0x2300_0041;
/// The memory access port auto-increment wraps inside this many bytes.
const TAR_WRAP: u32 = 0x400;

/// The 3-bit ACK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ack {
    Ok,
    Wait,
    Fault,
    Missing,
}

/// A request header: `APnDP`, `RnW` and the register address bits.
#[derive(Clone, Copy)]
struct Req {
    ap: bool,
    read: bool,
    reg: u8,
}

impl Req {
    fn bits(self) -> [bool; 8] {
        let a2 = self.reg & 0x4 != 0;
        let a3 = self.reg & 0x8 != 0;
        let parity = (self.ap as u8 + self.read as u8 + a2 as u8 + a3 as u8) & 1 == 1;
        [true, self.ap, self.read, a2, a3, parity, false, true]
    }
}

/// The retry state of one memory operation.
struct Retry {
    tries: u32,
    since: u64,
}

/// The SWD engine.
pub struct Swd<P: SwdPins> {
    pins: P,
    cfg: SwdConfig,
    counters: Counters,
    /// `SELECT` as last written, so a run of operations writes it once.
    select: Option<u32>,
    /// `CSW` as last written.
    csw: Option<u32>,
    /// The `DPIDR` read by the last successful connect.
    dpidr: Option<u32>,
}

impl<P: SwdPins> Swd<P> {
    /// An engine on `pins`, not yet connected.
    pub fn new(pins: P, cfg: SwdConfig) -> Self {
        Swd {
            pins,
            cfg,
            counters: Counters::default(),
            select: None,
            csw: None,
            dpidr: None,
        }
    }

    /// The counters since boot.
    pub fn counters(&self) -> Counters {
        self.counters
    }

    /// The pins (the simulated target, in tests).
    pub fn pins(&mut self) -> &mut P {
        &mut self.pins
    }

    /// The `DPIDR` of the last successful connect, if the engine is connected.
    pub fn dpidr(&self) -> Option<u32> {
        self.dpidr
    }

    /// Forget the connection (the target went away): the next [`connect`](Self::connect) starts
    /// from a line reset.
    pub fn forget(&mut self) {
        self.dpidr = None;
        self.select = None;
        self.csw = None;
    }

    /// Monotonic microseconds from the pins' clock.
    pub fn now_us(&mut self) -> u64 {
        self.pins.now_us()
    }

    /// Measure the clock: time `clocks` idle cycles (SWDIO driven low, which is idle at any point
    /// between transfers) and return kHz.
    pub fn measure_khz(&mut self, clocks: u32) -> u32 {
        self.pins.swdio_out();
        let t0 = self.pins.now_us();
        for _ in 0..clocks {
            self.pins.write_bit(false);
        }
        let us = self.pins.now_us().saturating_sub(t0).max(1);
        (clocks as u64 * 1000 / us) as u32
    }

    // ---- the wire ----

    /// One transfer on the wire, no retry: request, turnaround, ACK, data and parity, idle.
    /// Returns the ACK, the read data and whether its parity checked.
    fn raw(&mut self, req: Req, wdata: u32) -> (Ack, u32, bool) {
        let p = &mut self.pins;
        p.swdio_out();
        for b in req.bits() {
            p.write_bit(b);
        }
        p.swdio_in();
        p.clock(); // turnaround
        let a = p.read_bit() as u8 | (p.read_bit() as u8) << 1 | (p.read_bit() as u8) << 2;
        let ack = match a {
            0b001 => Ack::Ok,
            0b010 => Ack::Wait,
            0b100 => Ack::Fault,
            _ => Ack::Missing,
        };
        let mut rdata = 0;
        let mut parity_ok = true;
        if ack == Ack::Ok {
            if req.read {
                let mut ones = 0u32;
                for i in 0..32 {
                    if p.read_bit() {
                        rdata |= 1 << i;
                        ones += 1;
                    }
                }
                let parity = p.read_bit();
                parity_ok = parity == (ones & 1 == 1);
                p.clock(); // turnaround back to the host
                p.swdio_out();
            } else {
                p.clock(); // turnaround
                p.swdio_out();
                for i in 0..32 {
                    p.write_bit(wdata & (1 << i) != 0);
                }
                p.write_bit(wdata.count_ones() & 1 == 1);
            }
        } else {
            p.clock(); // turnaround after WAIT / FAULT / nothing
            p.swdio_out();
        }
        for _ in 0..8 {
            p.write_bit(false); // idle
        }
        (ack, rdata, parity_ok)
    }

    /// One transfer with the `WAIT` retry, and the parity retry for a debug port read (which is
    /// idempotent). Everything else is the caller's to recover.
    fn xfer(&mut self, req: Req, wdata: u32) -> Result<u32, XferError> {
        let mut waits = 0;
        loop {
            let (ack, rdata, parity_ok) = self.raw(req, wdata);
            match ack {
                Ack::Ok if req.read && !parity_ok => {
                    self.counters.parity_errors += 1;
                    if req.ap {
                        return Err(XferError::Parity);
                    }
                }
                Ack::Ok => return Ok(rdata),
                Ack::Wait => {
                    self.counters.waits += 1;
                }
                Ack::Fault => {
                    self.counters.faults += 1;
                    return Err(XferError::Fault);
                }
                Ack::Missing => return Err(XferError::NoAck),
            }
            waits += 1;
            if waits > self.cfg.wait_retries {
                return Err(if ack == Ack::Wait {
                    XferError::Wait
                } else {
                    XferError::Parity
                });
            }
        }
    }

    fn dp_read(&mut self, reg: u8) -> Result<u32, XferError> {
        self.xfer(
            Req {
                ap: false,
                read: true,
                reg,
            },
            0,
        )
    }

    fn dp_write(&mut self, reg: u8, v: u32) -> Result<(), XferError> {
        self.xfer(
            Req {
                ap: false,
                read: false,
                reg,
            },
            v,
        )
        .map(|_| ())
    }

    fn ap_read(&mut self, reg: u8) -> Result<u32, XferError> {
        self.xfer(
            Req {
                ap: true,
                read: true,
                reg,
            },
            0,
        )
    }

    fn ap_write(&mut self, reg: u8, v: u32) -> Result<(), XferError> {
        self.xfer(
            Req {
                ap: true,
                read: false,
                reg,
            },
            v,
        )
        .map(|_| ())
    }

    fn line_reset(&mut self) {
        self.pins.swdio_out();
        for _ in 0..56 {
            self.pins.write_bit(true);
        }
    }

    /// Line reset, JTAG-to-SWD `0xE79E`, line reset, idle, then the `DPIDR` read the debug port
    /// requires after a line reset.
    fn wake(&mut self) -> Result<u32, XferError> {
        self.select = None;
        self.csw = None;
        self.line_reset();
        let mut seq: u16 = 0xE79E;
        for _ in 0..16 {
            self.pins.write_bit(seq & 1 != 0);
            seq >>= 1;
        }
        self.line_reset();
        for _ in 0..4 {
            self.pins.write_bit(false);
        }
        self.dp_read(DP_DPIDR)
    }

    /// The recovery for one fault (`ABORT` for a `FAULT`, a reconnect for a missing ACK), then the
    /// budget check. `Ok` means try the operation again. A recovery that itself fails is the next
    /// fault to recover from, under the same budget.
    fn recover(&mut self, mut fault: XferError, r: &mut Retry) -> Result<(), SwdError> {
        loop {
            let res = match fault {
                XferError::Fault => self.dp_write(DP_ABORT, ABORT_CLEAR),
                XferError::NoAck => {
                    self.counters.reconnects += 1;
                    self.wake()
                        .and_then(|_| self.dp_write(DP_ABORT, ABORT_CLEAR))
                }
                XferError::Wait | XferError::Parity => Ok(()),
            };
            r.tries += 1;
            let late = self.pins.now_us().saturating_sub(r.since) > self.cfg.deadline_us;
            if r.tries > self.cfg.op_retries || late {
                return Err(SwdError::Exhausted(res.err().unwrap_or(fault)));
            }
            match res {
                Ok(()) => return Ok(()),
                Err(f) => fault = f,
            }
        }
    }

    fn retry_state(&mut self) -> Retry {
        Retry {
            tries: 0,
            since: self.pins.now_us(),
        }
    }

    /// The operation made progress: reset its budget.
    fn progressed(&mut self, r: &mut Retry) {
        r.tries = 0;
        r.since = self.pins.now_us();
    }

    fn select0(&mut self) -> Result<(), XferError> {
        if self.select != Some(0) {
            self.dp_write(DP_SELECT, 0)?;
            self.select = Some(0);
        }
        Ok(())
    }

    fn set_csw(&mut self, csw: u32) -> Result<(), XferError> {
        self.select0()?;
        if self.csw != Some(csw) {
            self.ap_write(AP_CSW, csw)?;
            self.csw = Some(csw);
        }
        Ok(())
    }

    /// After a run of posted writes: the last one completed without a sticky error.
    fn writes_landed(&mut self) -> Result<(), XferError> {
        let st = self.dp_read(DP_CTRL_STAT)?;
        if st & (STICKYERR | WDATAERR) != 0 {
            return Err(XferError::Fault);
        }
        Ok(())
    }

    // ---- connect ----

    /// Connect (`specs/hoverboard-bridge.md`, "Connect"): line reset, JTAG-to-SWD, line reset, idle, read
    /// `DPIDR`, clear sticky errors, `SELECT` = 0, request debug and system power-up and poll for
    /// both acknowledgements. Returns `DPIDR`.
    pub fn connect(&mut self) -> Result<u32, SwdError> {
        self.dpidr = None;
        let mut r = self.retry_state();
        let id = loop {
            match self.wake() {
                Ok(id) => break id,
                // The wake is the recovery for a missing ACK; count it as one.
                Err(XferError::NoAck) => {
                    self.counters.reconnects += 1;
                    r.tries += 1;
                    let late = self.pins.now_us().saturating_sub(r.since) > self.cfg.deadline_us;
                    if r.tries > self.cfg.op_retries || late {
                        return Err(SwdError::Exhausted(XferError::NoAck));
                    }
                }
                Err(f) => self.recover(f, &mut r)?,
            }
        };
        let mut r = self.retry_state();
        loop {
            match self.power_up() {
                Ok(true) => break,
                Ok(false) => return Err(SwdError::PowerUp),
                Err(f) => self.recover(f, &mut r)?,
            }
        }
        self.dpidr = Some(id);
        Ok(id)
    }

    fn power_up(&mut self) -> Result<bool, XferError> {
        self.dp_write(DP_ABORT, ABORT_CLEAR)?;
        self.select = None;
        self.select0()?;
        self.dp_write(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ)?;
        for _ in 0..100 {
            let st = self.dp_read(DP_CTRL_STAT)?;
            if st & (CDBGPWRUPACK | CSYSPWRUPACK) == CDBGPWRUPACK | CSYSPWRUPACK {
                return Ok(true);
            }
        }
        Ok(false)
    }

    // ---- memory access ----

    /// Read `out.len()` words from `addr` (word-aligned). Posted reads: each result arrives with
    /// the next access and the last through `RDBUFF`; `TAR` is rewritten at every 1 KB boundary.
    pub fn read_words(&mut self, addr: u32, out: &mut [u32]) -> Result<(), SwdError> {
        let mut done = 0;
        let mut r = self.retry_state();
        while done < out.len() {
            let before = done;
            let res = self.read_segment(addr, out, &mut done);
            if done > before {
                self.progressed(&mut r);
            }
            if let Err(f) = res {
                self.recover(f, &mut r)?;
            }
        }
        Ok(())
    }

    fn read_segment(
        &mut self,
        addr: u32,
        out: &mut [u32],
        done: &mut usize,
    ) -> Result<(), XferError> {
        let start = *done;
        let a = addr.wrapping_add(4 * start as u32);
        let room = ((TAR_WRAP - (a & (TAR_WRAP - 1))) / 4) as usize;
        let end = out.len().min(start + room);
        self.set_csw(CSW_WORD)?;
        self.ap_write(AP_TAR, a)?;
        self.ap_read(AP_DRW)?; // starts the first read; returns a stale value
        for (k, slot) in out.iter_mut().enumerate().take(end - 1).skip(start) {
            *slot = self.ap_read(AP_DRW)?;
            *done = k + 1;
        }
        out[end - 1] = self.dp_read(DP_RDBUFF)?;
        *done = end;
        Ok(())
    }

    /// Write `data` as words from `addr` (word-aligned), `TAR` rewritten at every 1 KB boundary.
    pub fn write_words(&mut self, addr: u32, data: &[u32]) -> Result<(), SwdError> {
        self.write_run(addr, data.len(), CSW_WORD, 4, |k| data[k])
    }

    /// Write `data` as halfwords from `addr` (halfword-aligned), each through its own `TAR` (the
    /// halfword `CSW` does not increment): the flash controller's programming access.
    pub fn write_halfwords(&mut self, addr: u32, data: &[u16]) -> Result<(), SwdError> {
        self.write_run(addr, data.len(), CSW_HALF, 2, |k| {
            let a = addr + 2 * k as u32;
            let v = data[k] as u32;
            if a & 2 != 0 {
                v << 16
            } else {
                v
            }
        })
    }

    /// The shared write loop. A write's `OK` confirms the one before it (the access port takes a
    /// new write only once the last has completed, and a bus error would make this one `FAULT`),
    /// so a restart goes back one word, not to the start. The last write is confirmed by a
    /// `CTRL/STAT` read with no sticky error.
    fn write_run(
        &mut self,
        addr: u32,
        n: usize,
        csw: u32,
        step: u32,
        word: impl Fn(usize) -> u32,
    ) -> Result<(), SwdError> {
        let mut done = 0;
        let mut r = self.retry_state();
        while done < n {
            let before = done;
            let res = self.write_segment(addr, n, csw, step, &word, &mut done);
            if done > before {
                self.progressed(&mut r);
            }
            if let Err(f) = res {
                self.recover(f, &mut r)?;
            }
        }
        Ok(())
    }

    fn write_segment(
        &mut self,
        addr: u32,
        n: usize,
        csw: u32,
        step: u32,
        word: &impl Fn(usize) -> u32,
        done: &mut usize,
    ) -> Result<(), XferError> {
        let start = *done;
        let a = addr.wrapping_add(step * start as u32);
        let end = if csw == CSW_WORD {
            let room = ((TAR_WRAP - (a & (TAR_WRAP - 1))) / step) as usize;
            n.min(start + room)
        } else {
            n
        };
        self.set_csw(csw)?;
        if csw == CSW_WORD {
            self.ap_write(AP_TAR, a)?;
        }
        for k in start..end {
            if csw != CSW_WORD {
                self.ap_write(AP_TAR, addr.wrapping_add(step * k as u32))?;
            }
            self.ap_write(AP_DRW, word(k))?;
            *done = k; // this write's OK confirmed the one before it
        }
        self.writes_landed()?;
        *done = end;
        Ok(())
    }

    /// Read one word.
    pub fn read_word(&mut self, addr: u32) -> Result<u32, SwdError> {
        let mut v = [0u32];
        self.read_words(addr, &mut v)?;
        Ok(v[0])
    }

    /// Write one word.
    pub fn write_word(&mut self, addr: u32, v: u32) -> Result<(), SwdError> {
        self.write_words(addr, &[v])
    }

    /// Read bytes from any address: the covering words, sliced.
    pub fn read_bytes(&mut self, addr: u32, out: &mut [u8]) -> Result<(), SwdError> {
        let mut a = addr;
        let mut o = 0;
        while o < out.len() {
            let base = a & !3;
            let skip = (a - base) as usize;
            let mut words = [0u32; 16];
            let nw = (skip + out.len() - o).div_ceil(4).min(words.len());
            self.read_words(base, &mut words[..nw])?;
            let mut bytes = [0u8; 64];
            for (i, w) in words[..nw].iter().enumerate() {
                bytes[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
            }
            let take = (4 * nw - skip).min(out.len() - o);
            out[o..o + take].copy_from_slice(&bytes[skip..skip + take]);
            o += take;
            a += take as u32;
        }
        Ok(())
    }

    /// Write bytes to any address: whole words in the middle, and a read-modify-write of the word
    /// at an unaligned edge (the mailbox ring writes bytes at any index; the bytes it rewrites
    /// unchanged beside them are ones the other end only reads).
    pub fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), SwdError> {
        let mut a = addr;
        let mut i = 0;
        while i < data.len() {
            let base = a & !3;
            let off = (a - base) as usize;
            if off != 0 || data.len() - i < 4 {
                let take = (4 - off).min(data.len() - i);
                let mut w = self.read_word(base)?.to_le_bytes();
                w[off..off + take].copy_from_slice(&data[i..i + take]);
                self.write_word(base, u32::from_le_bytes(w))?;
                i += take;
                a += take as u32;
            } else {
                let mut words = [0u32; 16];
                let nw = ((data.len() - i) / 4).min(words.len());
                for (k, w) in words[..nw].iter_mut().enumerate() {
                    let s = i + 4 * k;
                    *w = u32::from_le_bytes([data[s], data[s + 1], data[s + 2], data[s + 3]]);
                }
                self.write_words(base, &words[..nw])?;
                i += 4 * nw;
                a += 4 * nw as u32;
            }
        }
        Ok(())
    }
}

/// Word access to the target's memory: the part of the engine's API the RAM loader's driver and
/// the core register access use (`crate::loader`, `crate::target`), so the same driver runs on
/// the engine and, in the emulator tests, straight onto an emulated core's memory.
pub trait MemPort {
    /// What an access fails with.
    type Error;
    /// Read one word.
    fn read_word(&mut self, addr: u32) -> Result<u32, Self::Error>;
    /// Write one word.
    fn write_word(&mut self, addr: u32, v: u32) -> Result<(), Self::Error>;
    /// Write `data` as words from `addr` (word-aligned).
    fn write_words(&mut self, addr: u32, data: &[u32]) -> Result<(), Self::Error>;
}

impl<P: SwdPins> MemPort for Swd<P> {
    type Error = SwdError;
    fn read_word(&mut self, addr: u32) -> Result<u32, SwdError> {
        Swd::read_word(self, addr)
    }
    fn write_word(&mut self, addr: u32, v: u32) -> Result<(), SwdError> {
        Swd::write_word(self, addr, v)
    }
    fn write_words(&mut self, addr: u32, data: &[u32]) -> Result<(), SwdError> {
        Swd::write_words(self, addr, data)
    }
}
