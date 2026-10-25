//! The RAM loader (`specs/hoverboard-bridge.md`, "Flashing", "The RAM loader"): a small programmer in
//! the target's SRAM that the bridge feeds through a ring, so the SWD traffic is one word write per
//! 4 bytes instead of a write and a status poll per halfword.
//!
//! This module owns the layout (the job, the simulated board and the tests read it here; the loader
//! source `loader/f1x.S` carries the two addresses it needs as literals, which a test checks) and
//! the bridge's side of the protocol, over a [`MemPort`] so the emulator tests drive the real
//! loader bytes with this same code.
//!
//! The protocol: the bridge writes image words into the ring and advances `wp` once per
//! [`CHUNK`]; the loader programs halfwords from `rp`, publishes `rp` after each, and spins while
//! `wp == rp`. The bridge reads `rp` only when the ring would otherwise be full. Once every chunk is
//! in, the bridge waits for `rp` to catch up with `wp`, writes `wp = 0`, and waits for the `bkpt`
//! halt; the loader leaves `FMC_STAT` in `r0` and the next flash address in `r2`. An error stops
//! the loader at the failing halfword (`r2`) and sets `rp` to 0.

use crate::swd::MemPort;
use crate::target::{self, reg, PortError};

/// The loader's Thumb code, assembled from `loader/f1x.S` (the command is at its top).
pub const CODE: &[u8] = include_bytes!("../../loader/f1x.bin");
/// Where the code runs: the mailbox region, free while the board is halted for a flash.
pub const BASE: u32 = 0x2000_0000;
/// The room for the code.
pub const CODE_MAX: u32 = 128;
/// The bridge's write pointer into the ring (an address in it); 0 ends the run.
pub const WP: u32 = BASE + CODE_MAX;
/// The loader's read pointer into the ring; 0 after an error.
pub const RP: u32 = WP + 4;
/// The ring's first byte.
pub const RING_START: u32 = 0x2000_0400;
/// The ring's size.
pub const RING_LEN: u32 = 4096;
/// One past the ring's last byte.
pub const RING_END: u32 = RING_START + RING_LEN;
/// What the bridge writes between two advances of `wp`.
pub const CHUNK: u32 = 1024;
/// The loader's stack top: the top of the 8 KiB the smallest part has, so one loader runs on
/// every part.
pub const STACK_TOP: u32 = 0x2000_2000;

const _: () = assert!(CODE.len() < CODE_MAX as usize);
const _: () = assert!(RING_LEN.is_multiple_of(CHUNK) && RING_END <= STACK_TOP);

/// Write the loader's code into SRAM. The core must be halted.
pub fn load<M: MemPort>(m: &mut M) -> Result<(), M::Error> {
    let mut words = [0u32; (CODE_MAX / 4) as usize];
    let n = CODE.len().div_ceil(4);
    for (k, w) in words[..n].iter_mut().enumerate() {
        let mut b = [0u8; 4];
        for (j, x) in b.iter_mut().enumerate() {
            *x = *CODE.get(4 * k + j).unwrap_or(&0);
        }
        *w = u32::from_le_bytes(b);
    }
    m.write_words(BASE, &words[..n])
}

/// How a run ended: what the loader left in `r0` and `r2`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Outcome {
    /// `FMC_STAT`.
    pub stat: u32,
    /// The next flash address, or on an error the failing halfword's.
    pub addr: u32,
}

/// What one [`Run::poll`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Poll {
    /// A chunk went into the ring; the bytes fed so far.
    Fed(u32),
    /// The loader moved (`rp` advanced, or the run reached its next phase).
    Moved,
    /// Nothing moved since the last poll.
    Waiting,
    /// The loader halted.
    Done(Outcome),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Feed,
    Drain,
    Halt,
    Done(Outcome),
}

/// One loader run: `len` bytes programmed from flash address `addr`, ascending.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Run {
    addr: u32,
    len: u32,
    sent: u32,
    /// `wp` as last written.
    wp: u32,
    /// `rp` as last read.
    rp: u32,
    phase: Phase,
}

impl Run {
    /// Start a run on a halted core with the loader's code in place: the control words, the
    /// register arguments (`r0` ring start, `r1` ring end, `r2` flash address, `r3` halfword
    /// count), the stack, the entry point, then resume.
    pub fn start<M: MemPort>(m: &mut M, addr: u32, len: u32) -> Result<Run, PortError<M::Error>> {
        m.write_word(WP, RING_START)?;
        m.write_word(RP, RING_START)?;
        target::write_core_reg(m, reg::R0, RING_START)?;
        target::write_core_reg(m, reg::R1, RING_END)?;
        target::write_core_reg(m, reg::R2, addr)?;
        target::write_core_reg(m, reg::R3, len.div_ceil(2))?;
        target::write_core_reg(m, reg::SP, STACK_TOP)?;
        target::write_core_reg(m, reg::PC, BASE | 1)?;
        target::write_core_reg(m, reg::XPSR, reg::XPSR_T)?;
        target::resume(m)?;
        Ok(Run {
            addr,
            len,
            sent: 0,
            wp: RING_START,
            rp: RING_START,
            phase: Phase::Feed,
        })
    }

    /// The flash address after the run's last halfword: where `r2` ends on success.
    pub fn end(&self) -> u32 {
        self.addr + 2 * self.len.div_ceil(2)
    }

    /// The bytes fed into the ring so far.
    pub fn sent(&self) -> u32 {
        self.sent
    }

    /// Bytes in the ring the loader has not consumed, by the last `rp` read.
    fn used(&self) -> u32 {
        (self.wp + RING_LEN - self.rp) % RING_LEN
    }

    /// Read `rp`. Returns whether it moved; an `rp` of 0 is the loader's error exit.
    fn read_rp<M: MemPort>(&mut self, m: &mut M) -> Result<bool, M::Error> {
        let rp = m.read_word(RP)?;
        if rp == 0 {
            self.phase = Phase::Halt;
            return Ok(true);
        }
        let moved = rp != self.rp;
        self.rp = rp;
        Ok(moved)
    }

    /// Do the next bounded piece of the run: write one chunk (reading `rp` first only when the
    /// ring would be full), or read `rp` once while draining, or check for the halt once.
    /// `data` is the run's bytes (`len` of them).
    pub fn poll<M: MemPort>(
        &mut self,
        m: &mut M,
        data: &[u8],
    ) -> Result<Poll, PortError<M::Error>> {
        match self.phase {
            Phase::Feed => {
                let n = CHUNK.min(self.len - self.sent);
                let adv = n.next_multiple_of(2);
                if self.used() + adv >= RING_LEN {
                    let moved = self.read_rp(m)?;
                    if self.phase != Phase::Feed || self.used() + adv >= RING_LEN {
                        return Ok(if moved { Poll::Moved } else { Poll::Waiting });
                    }
                }
                let mut words = [0u32; (CHUNK / 4) as usize];
                let nw = n.div_ceil(4) as usize;
                let at = self.sent as usize;
                for (k, w) in words[..nw].iter_mut().enumerate() {
                    let mut b = [0xFFu8; 4];
                    for (j, x) in b.iter_mut().enumerate() {
                        let o = 4 * k + j;
                        if o < n as usize {
                            *x = data[at + o];
                        }
                    }
                    *w = u32::from_le_bytes(b);
                }
                m.write_words(self.wp, &words[..nw])?;
                let wp = self.wp + adv;
                self.wp = if wp == RING_END { RING_START } else { wp };
                m.write_word(WP, self.wp)?;
                self.sent += n;
                if self.sent == self.len {
                    self.phase = Phase::Drain;
                }
                Ok(Poll::Fed(self.sent))
            }
            Phase::Drain => {
                let moved = self.read_rp(m)?;
                if self.phase == Phase::Drain && self.rp == self.wp {
                    m.write_word(WP, 0)?;
                    self.phase = Phase::Halt;
                    return Ok(Poll::Moved);
                }
                Ok(if moved { Poll::Moved } else { Poll::Waiting })
            }
            Phase::Halt => {
                if !target::halted(m)? {
                    return Ok(Poll::Waiting);
                }
                let o = Outcome {
                    stat: target::read_core_reg(m, reg::R0)?,
                    addr: target::read_core_reg(m, reg::R2)?,
                };
                self.phase = Phase::Done(o);
                Ok(Poll::Done(o))
            }
            Phase::Done(o) => Ok(Poll::Done(o)),
        }
    }
}
