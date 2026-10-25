//! A simulated target for the host tests (`specs/esp-tool.md`, "Verification"): a bit-level SWD
//! debug port and memory access port that injects `WAIT`, `FAULT`, missing ACKs and read-parity
//! errors at set rates, in front of a GD32 model with a flash controller that has write-once
//! halfwords, erase to `0xFFFF`, `PGERR` on a re-program and the F1x0 closed-halfword rule.
//!
//! The debug port follows the wire exactly as [`SwdPins`] presents it, and panics on a protocol
//! violation by the engine (a write while the target drives the line, a missing turnaround, a bad
//! write parity), so a test that passes has also checked the engine's bit sequence.

extern crate std;

use std::vec;
use std::vec::Vec;

use swd_mailbox::Mailbox;

use crate::swd::{Counters, SwdPins};

/// A memory bus behind the access port. `size` is 2 or 4; a halfword travels in the low 16 bits.
pub trait Bus {
    /// Read; `None` is a bus error.
    fn read(&mut self, addr: u32, size: u32) -> Option<u32>;
    /// Write; `false` is a bus error.
    fn write(&mut self, addr: u32, size: u32, val: u32) -> bool;
}

/// Fault injection rates, in parts per thousand of requests.
#[derive(Debug, Clone, Copy, Default)]
pub struct Inject {
    /// `WAIT` (on requests a debug port may answer `WAIT`).
    pub wait: u32,
    /// `FAULT` (sets the sticky error, on requests a debug port may answer `FAULT`).
    pub fault: u32,
    /// No ACK: the target ignores the request and waits for a line reset.
    pub noack: u32,
    /// A read whose parity bit arrives flipped.
    pub parity: u32,
    /// No target at all: nothing ever answers.
    pub dead: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Resp {
    Ok(u32),
    Wait,
    Fault,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Idle between transfers (the host drives).
    Hunt,
    /// Collecting a request.
    Request { bits: u8, n: u8 },
    /// The request is decoded; waiting for the host to release SWDIO.
    Release { req: u8, resp: Resp },
    /// Released; waiting for the turnaround clock.
    Turn { req: u8, resp: Resp },
    /// The host is reading the ACK.
    Ack { req: u8, resp: Resp, n: u8 },
    /// The host is reading 32 data bits and parity.
    ReadData { val: u32, parity: bool, n: u8 },
    /// After read data: turnaround clock, then the host drives.
    ReadTurn { clocked: bool },
    /// After a write's ACK: turnaround clock, then the host drives.
    WriteTurn { req: u8, clocked: bool },
    /// The host writes 32 data bits and parity.
    WriteData { req: u8, acc: u64, n: u8 },
    /// After `WAIT`/`FAULT`: turnaround clock, then the host drives.
    NackTurn { clocked: bool },
    /// No response until a line reset.
    Lost,
}

/// The simulated debug port and the board behind it. Implements [`SwdPins`].
pub struct SimPins<B: Bus> {
    /// The board.
    pub bus: B,
    /// What to inject.
    pub inject: Inject,
    /// What was injected, in the engine's counter terms (a missing ACK is one reconnect).
    pub injected: Counters,
    rng: u64,
    t_us: u64,
    host_drives: bool,
    ones: u32,
    since_reset: u32,
    jtag: u16,
    selected: bool,
    needs_idcode: bool,
    phase: Phase,
    // Debug port.
    ctrl: u32,
    select: u32,
    sticky: bool,
    rdbuff: u32,
    // Memory access port.
    csw: u32,
    tar: u32,
    /// Requests the port has decoded.
    pub requests: u64,
}

const DPIDR: u32 = 0x1BA0_1477;

impl<B: Bus> SimPins<B> {
    /// A port in front of `bus`, in JTAG mode until the engine selects SWD.
    pub fn new(bus: B, inject: Inject, seed: u64) -> Self {
        SimPins {
            bus,
            inject,
            injected: Counters::default(),
            rng: seed,
            t_us: 0,
            host_drives: true,
            ones: 0,
            since_reset: 0,
            jtag: 0,
            selected: false,
            needs_idcode: true,
            phase: Phase::Hunt,
            ctrl: 0,
            select: 0,
            sticky: false,
            rdbuff: 0,
            csw: 0,
            tar: 0,
            requests: 0,
        }
    }

    fn roll(&mut self, per_mille: u32) -> bool {
        self.rng = self
            .rng
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.rng >> 33) % 1000) < per_mille as u64
    }

    fn tick(&mut self) {
        self.t_us += 1;
    }

    /// Advance the simulated clock (a delay the engine's caller takes).
    pub fn sleep_us(&mut self, us: u64) {
        self.t_us += us;
    }

    fn line_bit(&mut self, b: bool) {
        if b {
            self.ones += 1;
            if self.ones == 50 {
                // Line reset: the debug port is in reset state and wants a DPIDR read first.
                self.phase = Phase::Hunt;
                self.needs_idcode = true;
                self.since_reset = 0;
                self.jtag = 0;
                return;
            }
        } else {
            self.ones = 0;
        }
        if self.ones < 50 && self.since_reset < 16 {
            // The 16 bits after a line reset: JTAG-to-SWD selects the SWD protocol.
            self.jtag |= (b as u16) << self.since_reset;
            self.since_reset += 1;
            if self.since_reset == 16 && self.jtag == 0xE79E {
                self.selected = true;
            }
        }
    }

    fn decode(&mut self, req: u8) -> Phase {
        // bits: 0 start, 1 APnDP, 2 RnW, 3 A2, 4 A3, 5 parity, 6 stop, 7 park
        let ap = req & 2 != 0;
        let read = req & 4 != 0;
        let a = (req >> 3) & 3;
        let parity = (req >> 5) & 1;
        let ok = req & 1 == 1
            && ((req >> 1 & 1) + (req >> 2 & 1) + (req >> 3 & 1) + (req >> 4 & 1)) & 1 == parity
            && req & 0x40 == 0
            && req & 0x80 != 0;
        if !ok || !self.selected || self.inject.dead {
            return Phase::Lost;
        }
        let is_idcode = !ap && read && a == 0;
        if self.needs_idcode && !is_idcode {
            return Phase::Lost;
        }
        self.requests += 1;
        if self.roll(self.inject.noack) {
            self.injected.reconnects += 1;
            return Phase::Lost;
        }
        // The requests a debug port answers even with a sticky error, and never with WAIT.
        let exempt = is_idcode || (!ap && read && a == 1) || (!ap && !read && a == 0);
        let resp = if !exempt && self.roll(self.inject.wait) {
            self.injected.waits += 1;
            Resp::Wait
        } else if !exempt && self.roll(self.inject.fault) {
            self.injected.faults += 1;
            self.sticky = true;
            Resp::Fault
        } else if !exempt && self.sticky {
            Resp::Fault
        } else if read {
            Resp::Ok(self.do_read(ap, a))
        } else {
            Resp::Ok(0)
        };
        Phase::Release { req, resp }
    }

    fn do_read(&mut self, ap: bool, a: u8) -> u32 {
        if !ap {
            return match a {
                0 => {
                    self.needs_idcode = false;
                    DPIDR
                }
                1 => {
                    let acks = (self.ctrl & (1 << 28)) << 1 | (self.ctrl & (1 << 30)) << 1;
                    self.ctrl | acks | if self.sticky { 1 << 5 } else { 0 }
                }
                _ => self.rdbuff,
            };
        }
        let out = self.rdbuff;
        self.rdbuff = match a {
            0 => self.csw,
            1 => self.tar,
            3 => {
                let v = match self.bus.read(self.tar & !3, 4) {
                    Some(v) => v,
                    None => {
                        self.sticky = true;
                        0
                    }
                };
                self.advance_tar(4);
                v
            }
            _ => 0,
        };
        out
    }

    fn advance_tar(&mut self, size: u32) {
        if (self.csw >> 4) & 3 == 1 {
            self.tar = (self.tar & !0x3FF) | (self.tar.wrapping_add(size) & 0x3FF);
        }
    }

    fn do_write(&mut self, req: u8, val: u32) {
        let ap = req & 2 != 0;
        let a = (req >> 3) & 3;
        if !ap {
            match a {
                0 => {
                    if val & 0x04 != 0 {
                        self.sticky = false;
                    }
                }
                1 => self.ctrl = val & 0x5000_0000,
                2 => self.select = val,
                _ => {}
            }
            return;
        }
        assert_eq!(
            self.select, 0,
            "engine accessed the AP with SELECT {:#x}",
            self.select
        );
        match a {
            0 => self.csw = val,
            1 => self.tar = val,
            3 => {
                let ok = match self.csw & 7 {
                    2 => self.bus.write(self.tar & !3, 4, val),
                    1 => {
                        let lane = (self.tar & 2) * 8;
                        self.bus.write(self.tar & !1, 2, (val >> lane) & 0xFFFF)
                    }
                    s => panic!("engine used CSW size {s}"),
                };
                if !ok {
                    self.sticky = true;
                }
                let size = if self.csw & 7 == 2 { 4 } else { 2 };
                self.advance_tar(size);
            }
            _ => {}
        }
    }
}

impl<B: Bus> SwdPins for SimPins<B> {
    fn swdio_out(&mut self) {
        self.host_drives = true;
        self.phase = match self.phase {
            Phase::ReadTurn { clocked: true } | Phase::NackTurn { clocked: true } => Phase::Hunt,
            Phase::WriteTurn { req, clocked: true } => Phase::WriteData { req, acc: 0, n: 0 },
            Phase::ReadTurn { clocked: false } | Phase::NackTurn { clocked: false } => {
                panic!("host took SWDIO back without a turnaround clock")
            }
            Phase::WriteTurn { clocked: false, .. } => {
                panic!("host drove write data without a turnaround clock")
            }
            Phase::Release { .. } | Phase::Turn { .. } | Phase::Ack { .. } => {
                panic!("host drove SWDIO during the ACK phase ({:?})", self.phase)
            }
            Phase::ReadData { .. } => panic!("host drove SWDIO during read data"),
            p => p,
        };
    }

    fn swdio_in(&mut self) {
        self.host_drives = false;
        self.phase = match self.phase {
            Phase::Release { req, resp } => Phase::Turn { req, resp },
            Phase::Lost => Phase::Lost,
            p => panic!("host released SWDIO outside a request ({p:?})"),
        };
    }

    fn write_bit(&mut self, b: bool) {
        self.tick();
        assert!(self.host_drives, "host wrote with SWDIO released");
        self.line_bit(b);
        if self.ones >= 50 {
            return;
        }
        self.phase = match self.phase {
            Phase::Hunt if b => Phase::Request { bits: 1, n: 1 },
            Phase::Hunt => Phase::Hunt,
            Phase::Request { bits, n } => {
                let bits = bits | (b as u8) << n;
                if n + 1 == 8 {
                    self.decode(bits)
                } else {
                    Phase::Request { bits, n: n + 1 }
                }
            }
            Phase::WriteData { req, acc, n } => {
                let acc = acc | (b as u64) << n;
                if n + 1 == 33 {
                    let val = acc as u32;
                    let parity = (acc >> 32) & 1 == 1;
                    assert_eq!(parity, val.count_ones() & 1 == 1, "bad write parity");
                    self.do_write(req, val);
                    Phase::Hunt
                } else {
                    Phase::WriteData { req, acc, n: n + 1 }
                }
            }
            Phase::Lost => Phase::Lost,
            p => panic!("host wrote a bit in {p:?}"),
        };
    }

    fn read_bit(&mut self) -> bool {
        self.tick();
        assert!(!self.host_drives, "host read with SWDIO driven");
        let (bit, next) = match self.phase {
            Phase::Ack { req, resp, n } => {
                let ack = match resp {
                    Resp::Ok(_) => 0b001,
                    Resp::Wait => 0b010,
                    Resp::Fault => 0b100,
                };
                let bit = (ack >> n) & 1 == 1;
                let next = if n + 1 < 3 {
                    Phase::Ack {
                        req,
                        resp,
                        n: n + 1,
                    }
                } else {
                    match resp {
                        Resp::Ok(v) if req & 4 != 0 => {
                            let flip = self.roll(self.inject.parity);
                            if flip {
                                self.injected.parity_errors += 1;
                            }
                            Phase::ReadData {
                                val: v,
                                parity: (v.count_ones() & 1 == 1) ^ flip,
                                n: 0,
                            }
                        }
                        Resp::Ok(_) => Phase::WriteTurn {
                            req,
                            clocked: false,
                        },
                        _ => Phase::NackTurn { clocked: false },
                    }
                };
                (bit, next)
            }
            Phase::ReadData { val, parity, n } => {
                if n < 32 {
                    (
                        (val >> n) & 1 == 1,
                        Phase::ReadData {
                            val,
                            parity,
                            n: n + 1,
                        },
                    )
                } else {
                    (parity, Phase::ReadTurn { clocked: false })
                }
            }
            Phase::Lost => (true, Phase::Lost), // the pull-up
            p => panic!("host read a bit in {p:?}"),
        };
        self.phase = next;
        bit
    }

    fn clock(&mut self) {
        self.tick();
        self.phase = match self.phase {
            Phase::Turn { req, resp } => Phase::Ack { req, resp, n: 0 },
            Phase::ReadTurn { clocked: false } => Phase::ReadTurn { clocked: true },
            Phase::WriteTurn {
                req,
                clocked: false,
            } => Phase::WriteTurn { req, clocked: true },
            Phase::NackTurn { clocked: false } => Phase::NackTurn { clocked: true },
            Phase::Lost => Phase::Lost,
            p => panic!("unexpected turnaround clock in {p:?}"),
        };
    }

    fn now_us(&mut self) -> u64 {
        self.t_us
    }
}

/// Which GD32 the model is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    /// GD32F10x: UID at `0x1FFF_F7E8`; programming order free.
    F10x,
    /// GD32F1x0: UID at `0x1FFF_F7AC`; the closed-halfword rule.
    F1x0,
}

const FLASH_BASE: u32 = 0x0800_0000;
const FLASH_LEN: u32 = 64 * 1024;
const RAM_BASE: u32 = 0x2000_0000;
const RAM_LEN: u32 = 8 * 1024;
const OB_BASE: u32 = 0x1FFF_F800;
const FMC: u32 = 0x4002_2000;
const CCHP: u32 = 0x4001_2C44;
const MOE: u32 = 1 << 15;

/// The FMC status bits.
pub const STAT_BUSY: u32 = 1;
/// `PGERR`.
pub const STAT_PGERR: u32 = 1 << 2;
/// `WPERR`.
pub const STAT_WPERR: u32 = 1 << 4;
const STAT_ENDF: u32 = 1 << 5;
const CTL_PG: u32 = 1;
const CTL_PER: u32 = 1 << 1;
const CTL_MER: u32 = 1 << 2;
const CTL_OBPG: u32 = 1 << 4;
const CTL_OBER: u32 = 1 << 5;
const CTL_START: u32 = 1 << 6;
const CTL_LK: u32 = 1 << 7;
const CTL_OBWEN: u32 = 1 << 9;

/// A GD32 mainboard MCU as seen over the memory access port.
pub struct Gd32 {
    /// The family.
    pub family: Family,
    /// Main flash, as halfwords.
    pub flash: Vec<u16>,
    /// Halfwords the F1x0 rule has closed until the next erase: programming the low halfword of a
    /// word closes the high halfword of the word below, within a page and never across one
    /// (measured on a GD32F130C8, `runtime_hal::fmc`, "A high halfword is closed by its successor").
    closed: Vec<bool>,
    /// RAM, word-aligned so the firmware's `Mailbox` can sit on it.
    ram: Vec<u32>,
    /// The 16 B option byte block as 8 halfwords (value in the low byte, complement in the high).
    pub option: [u16; 8],
    /// The protection byte as loaded at the last power-on or reset.
    loaded_spc: u8,
    /// The 96-bit unique ID.
    pub uid: [u8; 12],
    // Flash controller.
    locked: bool,
    key_stage: u8,
    ob_key_stage: u8,
    ctl: u32,
    stat: u32,
    addr: u32,
    busy_polls: u32,
    // Core and debug.
    debugen: bool,
    /// The core is halted.
    pub halted: bool,
    demcr: u32,
    /// `DBG_CTL0`.
    pub dbg_ctl0: u32,
    /// Timer 0 `CCHP`.
    pub cchp: u32,
    /// `MOE` cannot be cleared (a board whose disarm write does not take).
    pub moe_stuck: bool,
    /// System resets since the model was built.
    pub resets: u32,
    /// The flash size register, KiB.
    pub flash_kib: u16,
    /// Bus writes into the store's pages (the last 2 KB), which the tool must never make.
    pub store_writes: u32,
}

impl Gd32 {
    /// An unprotected part with erased flash, powered on.
    pub fn new(family: Family) -> Self {
        let mut g = Gd32 {
            family,
            flash: vec![0xFFFF; (FLASH_LEN / 2) as usize],
            closed: vec![false; (FLASH_LEN / 2) as usize],
            ram: vec![0; (RAM_LEN / 4) as usize],
            option: [
                0x5AA5, 0x00FF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
            ],
            loaded_spc: 0xA5,
            uid: [0x8A, 0xDC, 0x39, 0x32, 1, 2, 3, 4, 5, 6, 7, 8],
            locked: true,
            key_stage: 0,
            ob_key_stage: 0,
            ctl: CTL_LK,
            stat: 0,
            addr: 0,
            busy_polls: 0,
            debugen: false,
            halted: false,
            demcr: 0,
            dbg_ctl0: 0,
            cchp: 0,
            moe_stuck: false,
            resets: 0,
            flash_kib: 64,
            store_writes: 0,
        };
        g.power_cycle();
        g
    }

    /// Fill flash with `image` from the base (a part that already holds firmware).
    pub fn load(&mut self, image: &[u8]) {
        for (i, c) in image.chunks(2).enumerate() {
            self.flash[i] = c[0] as u16 | (*c.get(1).unwrap_or(&0xFF) as u16) << 8;
        }
    }

    /// Main flash as bytes.
    pub fn flash_bytes(&self) -> Vec<u8> {
        self.flash.iter().flat_map(|h| h.to_le_bytes()).collect()
    }

    /// Set the protection byte as a stock board ships it (read protection on, level 1).
    pub fn protect(&mut self) {
        self.option[0] = 0xFF00;
        self.power_cycle();
    }

    /// The protection byte as the option block holds it.
    pub fn spc(&self) -> u8 {
        self.option[0] as u8
    }

    /// The USER byte as the option block holds it.
    pub fn user(&self) -> u8 {
        self.option[1] as u8
    }

    /// The firmware end of the mailbox, on this model's RAM.
    pub fn mailbox(&mut self) -> Mailbox {
        // SAFETY: the RAM vector is never resized, so the pointer outlives every handle the test
        // holds while the model lives.
        unsafe { Mailbox::from_raw(self.ram.as_mut_ptr() as *mut u8) }
    }

    /// Power off and on: the option bytes load, the debug registers clear, the core boots.
    pub fn power_cycle(&mut self) {
        self.dbg_ctl0 = 0;
        self.debugen = false;
        self.demcr = 0;
        self.reset();
        self.resets -= 1;
    }

    fn protected(&self) -> bool {
        self.loaded_spc != 0xA5
    }

    fn reset(&mut self) {
        self.resets += 1;
        let spc = self.option[0];
        self.loaded_spc = if (spc >> 8) as u8 == !(spc as u8) {
            spc as u8
        } else {
            0xFF
        };
        self.locked = true;
        self.ctl = CTL_LK;
        self.stat = 0;
        self.busy_polls = 0;
        self.cchp = 0;
        self.halted = self.debugen && self.demcr & 1 != 0;
        if !self.halted && self.flash[0] != 0xFFFF {
            // The image boots and initialises its mailbox header.
            self.mailbox().init_header();
        }
    }

    fn flash_op_done(&mut self) {
        self.busy_polls = 2;
        self.stat |= STAT_ENDF;
    }

    fn ctl_write(&mut self, val: u32) {
        if self.locked {
            return;
        }
        if val & CTL_LK != 0 {
            self.locked = true;
            self.ctl = CTL_LK;
            return;
        }
        let obwen = self.ctl & CTL_OBWEN != 0 && val & CTL_OBWEN != 0;
        let mut v = val & !(CTL_OBWEN | CTL_LK);
        if !obwen {
            v &= !(CTL_OBPG | CTL_OBER);
        }
        self.ctl = v | if obwen { CTL_OBWEN } else { 0 };
        if v & CTL_START != 0 {
            if v & CTL_PER != 0 {
                let page = (self.addr.wrapping_sub(FLASH_BASE) / 1024) as usize;
                if page < (FLASH_LEN / 1024) as usize {
                    if page >= 62 {
                        self.store_writes += 1;
                    }
                    let r = page * 512..page * 512 + 512;
                    self.flash[r.clone()].fill(0xFFFF);
                    self.closed[r].fill(false);
                }
            } else if v & CTL_MER != 0 {
                self.flash.fill(0xFFFF);
                self.closed.fill(false);
            } else if v & CTL_OBER != 0 {
                if self.protected() {
                    self.flash.fill(0xFFFF);
                    self.closed.fill(false);
                }
                self.option = [0xFFFF; 8];
            }
            self.ctl &= !CTL_START;
            self.flash_op_done();
        }
    }

    fn program(&mut self, addr: u32, val: u16) -> bool {
        if self.locked || self.ctl & CTL_PG == 0 {
            self.stat |= STAT_PGERR;
            return true;
        }
        if self.protected() {
            self.stat |= STAT_WPERR;
            return true;
        }
        let i = ((addr - FLASH_BASE) / 2) as usize;
        if i >= 31 * 1024 {
            self.store_writes += 1;
        }
        if (self.flash[i] != 0xFFFF || self.closed[i]) && val != 0 {
            self.stat |= STAT_PGERR;
        } else {
            self.flash[i] &= val;
            if self.family == Family::F1x0 && addr.is_multiple_of(4) && !addr.is_multiple_of(1024) {
                self.closed[i - 1] = true;
            }
        }
        self.flash_op_done();
        true
    }

    fn uid_base(&self) -> u32 {
        match self.family {
            Family::F10x => 0x1FFF_F7E8,
            Family::F1x0 => 0x1FFF_F7AC,
        }
    }
}

impl Bus for Gd32 {
    fn read(&mut self, addr: u32, size: u32) -> Option<u32> {
        assert_eq!(size, 4, "the engine reads words");
        let uid = self.uid_base();
        Some(match addr {
            a if (FLASH_BASE..FLASH_BASE + FLASH_LEN).contains(&a) => {
                if self.protected() {
                    return None;
                }
                let i = ((a - FLASH_BASE) / 2) as usize;
                self.flash[i] as u32 | (self.flash[i + 1] as u32) << 16
            }
            a if (RAM_BASE..RAM_BASE + RAM_LEN).contains(&a) => {
                let i = ((a - RAM_BASE) / 4) as usize;
                // SAFETY: `i` is in bounds; volatile and through the buffer pointer (no slice
                // reference) because the firmware's `Mailbox` writes the same words through a
                // raw pointer from the same buffer.
                unsafe { self.ram.as_mut_ptr().add(i).read_volatile() }
            }
            a if (OB_BASE..OB_BASE + 16).contains(&a) => {
                let i = ((a - OB_BASE) / 2) as usize;
                self.option[i] as u32 | (self.option[i + 1] as u32) << 16
            }
            a if (uid..uid + 12).contains(&a) => {
                let o = (a - uid) as usize;
                u32::from_le_bytes([
                    self.uid[o],
                    self.uid[o + 1],
                    self.uid[o + 2],
                    self.uid[o + 3],
                ])
            }
            0x1FFF_F7E0 => self.flash_kib as u32 | 8 << 16,
            0x1FFF_F7E8..=0x1FFF_F7F3 => 0xFFFF_FFFF, // F1x0: no UID here
            0x4002_200C => {
                let busy = if self.busy_polls > 0 {
                    self.busy_polls -= 1;
                    STAT_BUSY
                } else {
                    0
                };
                self.stat | busy
            }
            0x4002_2010 => self.ctl | if self.locked { CTL_LK } else { 0 },
            0x4002_2014 => self.addr,
            CCHP => self.cchp,
            0xE004_2000 => 0x1303_0410,
            0xE004_2004 => self.dbg_ctl0,
            0xE000_ED00 => 0x412F_C231,
            0xE000_EDF0 => {
                (self.halted as u32) << 17
                    | 1 << 16
                    | (self.halted as u32) << 1
                    | self.debugen as u32
            }
            0xE000_EDFC => self.demcr,
            _ => return None,
        })
    }

    fn write(&mut self, addr: u32, size: u32, val: u32) -> bool {
        match (addr, size) {
            (a, 2) if (FLASH_BASE..FLASH_BASE + FLASH_LEN).contains(&a) => {
                self.program(a, val as u16)
            }
            (a, 2) if (OB_BASE..OB_BASE + 16).contains(&a) => {
                if self.locked || self.ctl & CTL_OBPG == 0 || self.ctl & CTL_OBWEN == 0 {
                    self.stat |= STAT_PGERR;
                    return true;
                }
                let i = ((a - OB_BASE) / 2) as usize;
                if self.option[i] != 0xFFFF {
                    self.stat |= STAT_PGERR;
                } else {
                    self.option[i] = val as u16;
                }
                self.flash_op_done();
                true
            }
            (a, 4) if (RAM_BASE..RAM_BASE + RAM_LEN).contains(&a) => {
                let i = ((a - RAM_BASE) / 4) as usize;
                // SAFETY: as in `read`.
                unsafe { self.ram.as_mut_ptr().add(i).write_volatile(val) };
                true
            }
            (0x4002_2004, 4) => {
                match (self.key_stage, val) {
                    (0, 0x4567_0123) => self.key_stage = 1,
                    (1, 0xCDEF_89AB) => {
                        self.key_stage = 0;
                        self.locked = false;
                        self.ctl &= !CTL_LK;
                    }
                    _ => {
                        self.key_stage = 0;
                        self.locked = true;
                        return false;
                    }
                }
                true
            }
            (0x4002_2008, 4) => {
                match (self.ob_key_stage, val) {
                    (0, 0x4567_0123) => self.ob_key_stage = 1,
                    (1, 0xCDEF_89AB) if !self.locked => {
                        self.ob_key_stage = 0;
                        self.ctl |= CTL_OBWEN;
                    }
                    _ => self.ob_key_stage = 0,
                }
                true
            }
            (0x4002_200C, 4) => {
                self.stat &= !(val & (STAT_PGERR | STAT_WPERR | STAT_ENDF));
                true
            }
            (0x4002_2010, 4) => {
                self.ctl_write(val);
                true
            }
            (0x4002_2014, 4) => {
                self.addr = val;
                true
            }
            (CCHP, 4) => {
                self.cchp = if self.moe_stuck {
                    val | (self.cchp & MOE)
                } else {
                    val
                };
                true
            }
            (0xE004_2004, 4) => {
                self.dbg_ctl0 = val;
                true
            }
            (0xE000_EDF0, 4) => {
                if val >> 16 == 0xA05F {
                    self.debugen = val & 1 != 0;
                    self.halted = self.debugen && val & 2 != 0;
                }
                true
            }
            (0xE000_ED0C, 4) => {
                if val == 0x05FA_0004 {
                    self.reset();
                }
                true
            }
            (0xE000_EDFC, 4) => {
                self.demcr = val;
                true
            }
            _ => false,
        }
    }
}

/// A connected engine on a fresh model, for tests.
pub fn board(family: Family, inject: Inject, seed: u64) -> crate::swd::Swd<SimPins<Gd32>> {
    crate::swd::Swd::new(
        SimPins::new(Gd32::new(family), inject, seed),
        crate::swd::SwdConfig::default(),
    )
}

/// `FMC` registers, for tests that inspect the model.
pub const FMC_BASE: u32 = FMC;

/// The ESP itself, as [`crate::svc::Platform`] asks for it.
pub struct SimPlatform {
    /// What [`crate::svc::Platform::wifi`] answers.
    pub wifi: (espsvc::WifiState, [u8; 4]),
}

impl SimPlatform {
    /// An ESP with no Wi-Fi.
    pub fn new() -> Self {
        SimPlatform {
            wifi: (espsvc::WifiState::Down, [0; 4]),
        }
    }
}

impl Default for SimPlatform {
    fn default() -> Self {
        Self::new()
    }
}

impl crate::svc::Platform for SimPlatform {
    fn wifi(&self) -> (espsvc::WifiState, [u8; 4]) {
        self.wifi
    }
}
