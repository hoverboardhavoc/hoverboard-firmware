//! Flashing (`specs/hoverboard-bridge.md`, "Flashing"): erase and program only the pages an image covers,
//! never the store's last 2 KB, so the stored configuration survives; and clearing read protection
//! with the USER-byte check.
//!
//! The sequence, as [`FlashJob`] runs it one page per [`FlashJob::step`] so the caller can keep the
//! radio serviced between pages:
//!
//! 1. Connect, identify, refuse a protected or unsupported part, disarm, debug hold, halt.
//! 2. Unlock the flash controller.
//! 3. Erase each 1 KB page the image covers, page 0 last.
//! 4. Program halfwords in ascending address order, pages 1 and up first and page 0 last, re-arming
//!    `PG` per page. Ascending order is required on the F1x0 (programming the halfword at a
//!    4-byte-aligned address closes the halfword below it until the next erase); page 0 last means
//!    a flash cut short leaves an erased vector table and a chip reachable over SWD for a retry.
//! 5. After every halfword, poll `BUSY`, then check `PGERR` and `WPERR` and stop on either.
//! 6. Lock, read the whole range back and compare.
//! 7. Clear the mailbox magic, reset, release, and report booted once the header reads `MBX1`.

use espsvc::Status;

use crate::swd::{Swd, SwdError, SwdPins};
use crate::target::{self, TargetError, RDP_OFF};

/// Main flash base.
pub const FLASH_BASE: u32 = 0x0800_0000;
/// The erase page.
pub const PAGE: u32 = 1024;
/// The store's first byte on a 64 KiB part (`specs/decision-flash-budget.md`): the last 2 KB.
pub const STORE_START: u32 = 0x0800_F800;
/// The largest image: everything below the store (63,488 B, the firmware's link limit).
pub const MAX_IMAGE: usize = (STORE_START - FLASH_BASE) as usize;
/// The only flash size this version flashes. The 12-FET's store address is an open question
/// (`specs/hoverboard-bridge.md`, "Open questions"), so a part reporting more than 64 KiB is refused rather
/// than guessed at; a smaller part would put the store elsewhere and is refused too.
pub const SUPPORTED_KIB: u16 = 64;

const FMC_KEY: u32 = 0x4002_2004;
const FMC_OBKEY: u32 = 0x4002_2008;
const FMC_STAT: u32 = 0x4002_200C;
const FMC_CTL: u32 = 0x4002_2010;
const FMC_ADDR: u32 = 0x4002_2014;
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;
const STAT_BUSY: u32 = 1;
const STAT_PGERR: u32 = 1 << 2;
const STAT_WPERR: u32 = 1 << 4;
const STAT_ENDF: u32 = 1 << 5;
const CTL_PG: u32 = 1;
const CTL_PER: u32 = 1 << 1;
const CTL_OBPG: u32 = 1 << 4;
const CTL_OBER: u32 = 1 << 5;
const CTL_START: u32 = 1 << 6;
const CTL_LK: u32 = 1 << 7;
const CTL_OBWEN: u32 = 1 << 9;
/// `STAT` reads before `BUSY` counts as stuck. A halfword programs in about 37.5 us and a page
/// erases in tens of ms; one `STAT` read takes about 0.15 ms at 1 MHz.
const BUSY_POLLS: u32 = 2000;
/// How long the new image gets to write its mailbox header after the reset.
const BOOT_TIMEOUT_US: u64 = 3_000_000;

/// Why a flash or a protection clear stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    /// A board operation failed (SWD, still armed, no halt).
    Target(TargetError),
    /// The protection byte is not `0xA5`.
    Protected,
    /// The part's flash size is not [`SUPPORTED_KIB`].
    UnsupportedPart(u16),
    /// The image reaches the store's pages, or is empty.
    StoreRange,
    /// `BUSY` did not clear.
    Busy,
    /// The flash controller did not unlock (or did not enable option byte writes).
    Unlock,
    /// An erase reported an error (`stat` is `FMC_STAT`).
    Erase {
        /// The page address.
        addr: u32,
        /// `FMC_STAT`.
        stat: u32,
    },
    /// A halfword program reported an error.
    Program {
        /// The halfword address.
        addr: u32,
        /// `FMC_STAT`.
        stat: u32,
    },
    /// The read-back differs.
    Verify {
        /// The first differing byte's address.
        addr: u32,
    },
    /// The new image did not write its mailbox header in time.
    Boot,
}

impl From<SwdError> for FlashError {
    fn from(e: SwdError) -> Self {
        FlashError::Target(TargetError::Swd(e))
    }
}

impl From<TargetError> for FlashError {
    fn from(e: TargetError) -> Self {
        FlashError::Target(e)
    }
}

impl FlashError {
    /// The wire status.
    pub fn status(self) -> Status {
        match self {
            FlashError::Target(TargetError::Armed) => Status::Armed,
            FlashError::Target(TargetError::Swd(_)) => Status::NoTarget,
            FlashError::Target(TargetError::Timeout) => Status::Timeout,
            FlashError::Protected => Status::Protected,
            FlashError::UnsupportedPart(_) => Status::TooLarge,
            FlashError::StoreRange => Status::StoreRange,
            FlashError::Busy | FlashError::Boot => Status::Timeout,
            FlashError::Unlock | FlashError::Erase { .. } | FlashError::Program { .. } => {
                Status::FlashError
            }
            FlashError::Verify { .. } => Status::VerifyMismatch,
        }
    }
}

/// Refuse a range that is empty or reaches the store's pages.
pub fn check_range(len: usize) -> Result<(), FlashError> {
    if len == 0 || len > MAX_IMAGE {
        return Err(FlashError::StoreRange);
    }
    Ok(())
}

/// Connect, identify, and refuse a part this version must not flash.
pub fn check_part<P: SwdPins>(swd: &mut Swd<P>) -> Result<espsvc::TargetId, FlashError> {
    let id = target::identify(swd)?;
    if id.rdp != RDP_OFF {
        return Err(FlashError::Protected);
    }
    if id.flash_kib != SUPPORTED_KIB {
        return Err(FlashError::UnsupportedPart(id.flash_kib));
    }
    Ok(id)
}

fn wait_idle<P: SwdPins>(swd: &mut Swd<P>) -> Result<u32, FlashError> {
    for _ in 0..BUSY_POLLS {
        let st = swd.read_word(FMC_STAT)?;
        if st & STAT_BUSY == 0 {
            return Ok(st);
        }
    }
    Err(FlashError::Busy)
}

fn clear_stat<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), FlashError> {
    Ok(swd.write_word(FMC_STAT, STAT_PGERR | STAT_WPERR | STAT_ENDF)?)
}

pub(crate) fn unlock<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), FlashError> {
    if swd.read_word(FMC_CTL)? & CTL_LK != 0 {
        swd.write_word(FMC_KEY, KEY1)?;
        swd.write_word(FMC_KEY, KEY2)?;
    }
    if swd.read_word(FMC_CTL)? & CTL_LK != 0 {
        return Err(FlashError::Unlock);
    }
    Ok(())
}

fn lock<P: SwdPins>(swd: &mut Swd<P>) -> Result<(), FlashError> {
    Ok(swd.write_word(FMC_CTL, CTL_LK)?)
}

/// Erase one page.
pub fn erase_page<P: SwdPins>(swd: &mut Swd<P>, addr: u32) -> Result<(), FlashError> {
    if addr >= STORE_START {
        return Err(FlashError::StoreRange);
    }
    wait_idle(swd)?;
    clear_stat(swd)?;
    swd.write_word(FMC_CTL, CTL_PER)?;
    swd.write_word(FMC_ADDR, addr)?;
    swd.write_word(FMC_CTL, CTL_PER | CTL_START)?;
    let st = wait_idle(swd)?;
    swd.write_word(FMC_CTL, 0)?;
    if st & (STAT_PGERR | STAT_WPERR) != 0 {
        return Err(FlashError::Erase { addr, stat: st });
    }
    Ok(())
}

/// The halfword at `addr` of an image based at [`FLASH_BASE`] (an odd tail byte pairs with `0xFF`).
fn halfword(image: &[u8], addr: u32) -> u16 {
    let o = (addr - FLASH_BASE) as usize;
    image[o] as u16 | (*image.get(o + 1).unwrap_or(&0xFF) as u16) << 8
}

/// Program `image`'s halfwords in `[addr, end)` in ascending order. `PG` is re-armed for the run.
/// Erased-value halfwords (`0xFFFF`) are left as the erase left them. Stops on `PGERR` or `WPERR`,
/// with one allowance: a `PGERR` on a halfword that reads back as the value written is a
/// re-program by the engine's own restart (the target took the write but its ACK was lost on the
/// line), not a failed program, and the run continues.
pub fn program_range<P: SwdPins>(
    swd: &mut Swd<P>,
    image: &[u8],
    addr: u32,
    end: u32,
) -> Result<(), FlashError> {
    if end > STORE_START {
        return Err(FlashError::StoreRange);
    }
    wait_idle(swd)?;
    clear_stat(swd)?;
    swd.write_word(FMC_CTL, 0)?;
    swd.write_word(FMC_CTL, CTL_PG)?;
    let mut a = addr;
    let result = loop {
        if a >= end {
            break Ok(());
        }
        let v = halfword(image, a);
        if v != 0xFFFF {
            if let Err(e) = program_one(swd, a, v) {
                break Err(e);
            }
        }
        a += 2;
    };
    swd.write_word(FMC_CTL, 0)?;
    result
}

fn program_one<P: SwdPins>(swd: &mut Swd<P>, a: u32, v: u16) -> Result<(), FlashError> {
    swd.write_halfwords(a, &[v])?;
    let st = wait_idle(swd)?;
    if st & (STAT_PGERR | STAT_WPERR) == 0 {
        return Ok(());
    }
    clear_stat(swd)?;
    if st & STAT_WPERR == 0 {
        let w = swd.read_word(a & !3)?;
        let got = if a & 2 != 0 { w >> 16 } else { w } as u16;
        if got == v {
            return Ok(());
        }
    }
    Err(FlashError::Program { addr: a, stat: st })
}

/// What one [`FlashJob::step`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Still going: the stage and the bytes done in it.
    Progress(espsvc::Stage, u32),
    /// Finished: booted, or the error.
    Finished(Result<(), FlashError>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Prepare,
    Erase(u32),
    Program(u32),
    Verify(u32),
    Boot(u64),
    Done,
}

/// One board flash, run a page at a time.
pub struct FlashJob {
    len: u32,
    pages: u32,
    step: Step,
}

impl FlashJob {
    /// A job for an image of `len` bytes; refuses a range that reaches the store.
    pub fn new(len: usize) -> Result<Self, FlashError> {
        check_range(len)?;
        Ok(FlashJob {
            len: len as u32,
            pages: (len as u32).div_ceil(PAGE),
            step: Step::Prepare,
        })
    }

    /// The `i`-th page in the job's order: 1, 2, .. last, then 0.
    fn page(&self, i: u32) -> u32 {
        if i + 1 < self.pages {
            i + 1
        } else {
            0
        }
    }

    /// Run the next piece of the job. `image` is the whole buffer (`len` bytes).
    pub fn step<P: SwdPins>(&mut self, swd: &mut Swd<P>, image: &[u8]) -> Event {
        match self.run(swd, image) {
            Ok(ev) => ev,
            Err(e) => {
                self.step = Step::Done;
                // Leave the controller locked; the core stays halted with page 0 erased or intact.
                let _ = lock(swd);
                Event::Finished(Err(e))
            }
        }
    }

    fn run<P: SwdPins>(&mut self, swd: &mut Swd<P>, image: &[u8]) -> Result<Event, FlashError> {
        use espsvc::Stage;
        Ok(match self.step {
            Step::Prepare => {
                check_part(swd)?;
                target::halt(swd)?;
                unlock(swd)?;
                self.step = Step::Erase(0);
                Event::Progress(Stage::Erasing, 0)
            }
            Step::Erase(i) => {
                erase_page(swd, FLASH_BASE + self.page(i) * PAGE)?;
                let done = i + 1;
                self.step = if done == self.pages {
                    Step::Program(0)
                } else {
                    Step::Erase(done)
                };
                let bytes = (done * PAGE).min(self.len);
                if done == self.pages {
                    Event::Progress(Stage::Programming, 0)
                } else {
                    Event::Progress(Stage::Erasing, bytes)
                }
            }
            Step::Program(i) => {
                let start = FLASH_BASE + self.page(i) * PAGE;
                let end = (start + PAGE).min(FLASH_BASE + self.len);
                program_range(swd, image, start, end)?;
                let done = i + 1;
                if done == self.pages {
                    lock(swd)?;
                    self.step = Step::Verify(0);
                    Event::Progress(Stage::Verifying, 0)
                } else {
                    self.step = Step::Program(done);
                    Event::Progress(Stage::Programming, (done * PAGE).min(self.len))
                }
            }
            Step::Verify(off) => {
                let n = (PAGE).min(self.len - off) as usize;
                let mut words = [0u32; (PAGE / 4) as usize];
                let nw = n.div_ceil(4);
                swd.read_words(FLASH_BASE + off, &mut words[..nw])?;
                for (k, w) in words[..nw].iter().enumerate() {
                    for (j, b) in w.to_le_bytes().iter().enumerate() {
                        let o = off as usize + 4 * k + j;
                        if o < self.len as usize && image[o] != *b {
                            return Err(FlashError::Verify {
                                addr: FLASH_BASE + o as u32,
                            });
                        }
                    }
                }
                let next = off + n as u32;
                if next < self.len {
                    self.step = Step::Verify(next);
                    Event::Progress(Stage::Verifying, next)
                } else {
                    // The old image's header is still in RAM: clear the magic so `MBX1` means
                    // the new one wrote it. Reset while halted, then release: whether `C_HALT`
                    // survives the reset or not, the core starts from the reset vector.
                    swd.write_word(swd_mailbox::MAILBOX_BASE, 0)?;
                    target::reset(swd)?;
                    target::release(swd)?;
                    self.step = Step::Boot(swd.now_us());
                    Event::Progress(Stage::Verifying, self.len)
                }
            }
            Step::Boot(t0) => {
                if swd.read_word(swd_mailbox::MAILBOX_BASE)? == swd_mailbox::MAGIC {
                    self.step = Step::Done;
                    Event::Finished(Ok(()))
                } else if swd.now_us().saturating_sub(t0) > BOOT_TIMEOUT_US {
                    return Err(FlashError::Boot);
                } else {
                    Event::Progress(Stage::Verifying, self.len)
                }
            }
            Step::Done => Event::Finished(Ok(())),
        })
    }
}

/// Clear read protection (`specs/hoverboard-bridge.md`, "First install on a read-protected board"). This
/// mass-erases the chip. Records the USER byte (and the two DATA bytes) first, erases the option
/// bytes, writes the protection byte `0xA5` and writes USER and DATA back, each halfword with its
/// complement (the F1x0 does not generate it). Returns the recorded USER byte; the check that it
/// took is the next identify after the board is power-cycled by hand.
///
/// `OBWEN` is preserved in every `CTL` write. It is set by the `OBKEY` sequence and only ever
/// cleared by software, so the `CTL = OBER` write `gdf130backup` makes clears it and the erase does
/// not happen, which is how that clear could report success without taking effect.
pub fn clear_protection<P: SwdPins>(swd: &mut Swd<P>) -> Result<u8, FlashError> {
    let id = target::identify(swd)?;
    let user = id.user;
    let data = swd.read_word(target::OPTION_BYTES + 4)?;
    let (data0, data1) = (data as u8, (data >> 16) as u8);
    target::halt(swd)?;
    unlock(swd)?;
    swd.write_word(FMC_OBKEY, KEY1)?;
    swd.write_word(FMC_OBKEY, KEY2)?;
    if swd.read_word(FMC_CTL)? & CTL_OBWEN == 0 {
        lock(swd)?;
        return Err(FlashError::Unlock);
    }
    let result = (|| {
        wait_idle(swd)?;
        clear_stat(swd)?;
        swd.write_word(FMC_CTL, CTL_OBWEN | CTL_OBER)?;
        swd.write_word(FMC_CTL, CTL_OBWEN | CTL_OBER | CTL_START)?;
        let st = wait_idle(swd)?;
        if st & (STAT_PGERR | STAT_WPERR) != 0 {
            return Err(FlashError::Erase {
                addr: target::OPTION_BYTES,
                stat: st,
            });
        }
        swd.write_word(FMC_CTL, CTL_OBWEN | CTL_OBPG)?;
        program_option(swd, 0, RDP_OFF)?;
        program_option(swd, 2, user)?;
        if data0 != 0xFF {
            program_option(swd, 4, data0)?;
        }
        if data1 != 0xFF {
            program_option(swd, 6, data1)?;
        }
        Ok(user)
    })();
    lock(swd)?;
    result
}

fn program_option<P: SwdPins>(swd: &mut Swd<P>, offset: u32, b: u8) -> Result<(), FlashError> {
    let addr = target::OPTION_BYTES + offset;
    swd.write_halfwords(addr, &[((!b as u16) << 8) | b as u16])?;
    let st = wait_idle(swd)?;
    if st & (STAT_PGERR | STAT_WPERR) != 0 {
        clear_stat(swd)?;
        return Err(FlashError::Program { addr, stat: st });
    }
    Ok(())
}

/// The check of a protection clear after the power cycle: the protection byte is `0xA5` and the
/// USER byte is the one recorded before the clear.
pub fn clear_took(id: &espsvc::TargetId, recorded_user: u8) -> bool {
    id.rdp == RDP_OFF && id.user == recorded_user
}
