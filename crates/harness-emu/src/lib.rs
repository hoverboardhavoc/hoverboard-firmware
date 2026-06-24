//! Host-only Unicorn runner for the test harness (tier-2, the CI executor).
//!
//! This is the executor that earns the whole harness spec: the EXACT thumbv7m machine code that
//! ships on the chip runs under a CPU + memory emulator on the host, and reaches its verdict the
//! same RAM-channel way the silicon does. Unicorn maps memory at the real GD32 addresses (flash @
//! `0x0800_0000`, RAM @ `0x2000_0000`), loads the subject image, sets SP/PC from the vector table,
//! delivers the command word into mapped RAM, runs, and reads the result struct back from mapped
//! RAM, mirroring what the bench driver does over SWD.
//!
//! Unicorn is CPU + memory only: no NVIC, SysTick, `wfi` wakeup, or async exception entry. The
//! smoke subject (and the first real consumers) are straight-line / polled, so none of that is
//! needed here. The subject never halts (it busy-spins), so this runner does NOT wait for a `bkpt`:
//! it hooks the subject's write of the result `magic` word and stops emulation there, with an
//! instruction-count cap as a hang backstop.
//!
//! `unicorn-engine` is GPL-2.0 and links a native lib (built from source via cmake). That is fine
//! for host-only CI tooling; it is NEVER linked into firmware. It is an optional dependency behind
//! the `emu` feature, so a default thumbv7m build never pulls it into the graph.
//!
//! ## How the runner gets the image
//!
//! The host test builds `harness-smoke` for `thumbv7m-none-eabi` (release) and parses the resulting
//! ELF with the `object` crate, loading each `PT_LOAD` segment at its physical address. Parsing the
//! ELF (rather than objcopy-ing to a flat `.bin`) keeps the toolchain to plain cargo + a crate (no
//! `arm-none-eabi-objcopy` / `llvm-objcopy` dependency in CI), and the cortex-m-rt image is a single
//! contiguous flash segment anyway. The vector table's first two words (SP, reset PC) are then read
//! straight out of mapped flash, exactly as the silicon's Cortex-M loads them at reset.

#![cfg(feature = "emu")]

use harness_abi::{SmokeObs, CMD_ADDR, RESULT_ADDR, SMOKE_MAGIC, VERDICT_PASS};
use object::{Object, ObjectSegment};
use std::cell::Cell;
use std::rc::Rc;
use unicorn_engine::{
    unicorn_const::{Arch, HookType, Mode, Prot},
    RegisterARM, Unicorn,
};

/// GD32 memory map (the C8 sizes, valid for every fleet part the one image links for).
const FLASH_BASE: u64 = 0x0800_0000;
const FLASH_LEN: u64 = 64 * 1024;
const RAM_BASE: u64 = 0x2000_0000;
const RAM_LEN: u64 = 8 * 1024; // full 8 KiB mapped; the subject's memory.x just shrinks the LINKED region

/// Instruction-count cap: a generous backstop so a hung/looping subject (one that never writes
/// magic) cannot spin forever. The smoke subject reaches its magic write in well under this.
const INSN_CAP: usize = 1_000_000;

/// Outcome of one command run under the emulator: the decoded result struct plus whether the magic
/// hook actually fired (false = the subject never completed within the instruction cap).
#[derive(Debug, Clone, Copy)]
pub struct SmokeRun {
    pub magic: u32,
    pub verdict: u32,
    pub echo: u32,
    pub completed: bool,
}

impl SmokeRun {
    /// The same pass condition the bench driver applies: the run completed (magic landed) and the
    /// subject's self-check passed. `echo` is kept for the caller's failure message.
    pub fn passed(&self) -> bool {
        self.completed && self.magic == SMOKE_MAGIC && self.verdict == VERDICT_PASS
    }
}

/// Read a little-endian `u32` from mapped emulator memory.
fn read_u32(uc: &Unicorn<'_, Rc<Cell<bool>>>, addr: u64) -> u32 {
    let mut buf = [0u8; 4];
    uc.mem_read(addr, &mut buf).expect("mem_read");
    u32::from_le_bytes(buf)
}

/// Map the GD32 memory, load the subject ELF's loadable segments into flash, and set SP/PC from the
/// vector table. Returns the configured emulator and its shared "magic was written" flag.
fn make_emu(elf: &[u8]) -> (Unicorn<'static, Rc<Cell<bool>>>, Rc<Cell<bool>>) {
    let magic_seen = Rc::new(Cell::new(false));
    let mut uc = Unicorn::new_with_data(
        Arch::ARM,
        // Cortex-M3 executes Thumb-2. `Mode::THUMB` is what runs the image here: `Mode::MCLASS` was
        // tried (it is the "real" M-profile model) but this unicorn-engine 2.1.5 build rejects every
        // Thumb instruction under MCLASS with INSN_INVALID (confirmed against a trivial `movs`), so
        // MCLASS is unusable. `Mode::THUMB` decodes the Thumb-2 image correctly. The cost is no
        // M-profile exception/NVIC machinery, which this rig does not use anyway (CPU + memory only,
        // straight-line/polled subjects), and the verdict is the RAM result struct, not any
        // M-profile state. The one requirement THUMB imposes is that `emu_start`'s begin address
        // (and any resume PC) carry the Thumb bit, or Unicorn decodes the first block as ARM (see
        // `run_one`).
        Mode::THUMB,
        magic_seen.clone(),
    )
    .expect("unicorn init");

    uc.mem_map(FLASH_BASE, FLASH_LEN, Prot::ALL)
        .expect("map flash");
    uc.mem_map(RAM_BASE, RAM_LEN, Prot::ALL).expect("map ram");

    // Load each PT_LOAD segment at its physical address (flash). A cortex-m-rt release image is one
    // contiguous flash segment; loading by address is robust if that ever changes.
    let obj = object::File::parse(elf).expect("parse subject ELF");
    for seg in obj.segments() {
        let addr = seg.address();
        let data = seg.data().expect("segment data");
        if data.is_empty() {
            continue;
        }
        uc.mem_write(addr, data).expect("load segment into flash");
    }

    reset_core(&mut uc);

    // Hook the subject's write of the result magic word (offset 0 of SmokeObs at RESULT_ADDR) and
    // stop emulation when it lands. SmokeObs writes echo + verdict BEFORE magic, so by the time this
    // fires they are already committed in mapped RAM and the read-back is complete.
    let flag = magic_seen.clone();
    uc.add_mem_hook(
        HookType::MEM_WRITE,
        RESULT_ADDR as u64,
        RESULT_ADDR as u64 + 3, // the 4-byte magic word
        move |uc, _mem_type, _addr, _size, _value| {
            flag.set(true);
            uc.emu_stop().expect("emu_stop from magic hook");
            true
        },
    )
    .expect("add magic write hook");

    (uc, magic_seen)
}

/// Set SP and PC from the vector table at the start of flash (`[0x0800_0000]` = initial SP,
/// `[0x0800_0004]` = reset PC), exactly as the Cortex-M core does at reset. The reset vector already
/// carries the Thumb bit (bit 0) in the table; PC keeps it (Unicorn runs Thumb when the address is
/// odd). This is also the cold re-entry used between the two commands, mirroring the bench driver's
/// `reset` between phases.
fn reset_core(uc: &mut Unicorn<'_, Rc<Cell<bool>>>) {
    let sp = read_u32(uc, FLASH_BASE);
    let pc = read_u32(uc, FLASH_BASE + 4); // reset vector, Thumb bit set
    uc.reg_write(RegisterARM::SP, sp as u64).expect("set SP");
    uc.reg_write(RegisterARM::PC, pc as u64).expect("set PC");
}

/// Run the subject image once for one command word: write the command to `CMD_ADDR`, run from the
/// reset PC until the magic-write hook stops it (or the instruction cap trips), then decode
/// `SmokeObs` from mapped RAM. The caller resets the core between commands (cold re-entry).
fn run_one(
    uc: &mut Unicorn<'_, Rc<Cell<bool>>>,
    magic_seen: &Rc<Cell<bool>>,
    cmd: u32,
) -> SmokeRun {
    magic_seen.set(false);
    // Clear the magic word so a stale value from a previous run cannot read as completion.
    uc.mem_write(RESULT_ADDR as u64, &0u32.to_le_bytes())
        .expect("clear magic");
    // Deliver the command word, exactly as the bench driver writes it over SWD before resuming.
    uc.mem_write(CMD_ADDR as u64, &cmd.to_le_bytes())
        .expect("write command word");

    // Begin at the reset PC with the Thumb bit SET, or Unicorn decodes the first block as ARM and
    // faults on the first Thumb instruction (INSN_INVALID). until = 0 (no address stop); the magic
    // hook stops us. count = INSN_CAP backstops a hang. A clean stop from the hook returns Ok; a
    // stop from the count cap also returns Ok with the hook never having fired, which `completed`
    // below distinguishes.
    let pc = uc.reg_read(RegisterARM::PC).expect("read PC") | 1;
    let _ = uc.emu_start(pc, 0, 0, INSN_CAP);

    SmokeRun {
        magic: read_u32(uc, RESULT_ADDR as u64),
        verdict: read_u32(uc, RESULT_ADDR as u64 + 4),
        echo: read_u32(uc, RESULT_ADDR as u64 + 8),
        completed: magic_seen.get(),
    }
}

const _: () = assert!(std::mem::size_of::<SmokeObs>() == 12);

/// Run the smoke subject ELF under Unicorn for each command word, returning one [`SmokeRun`] per
/// command. The caller (the test) asserts every run `passed()`. Each command is a cold re-entry
/// (SP/PC reset from the vector table), so the second command does not depend on the first's state.
pub fn run_smoke(elf: &[u8], commands: &[u32]) -> Vec<SmokeRun> {
    let (mut uc, magic_seen) = make_emu(elf);
    let mut out = Vec::with_capacity(commands.len());
    for (i, &cmd) in commands.iter().enumerate() {
        if i > 0 {
            reset_core(&mut uc); // cold re-entry for the next command
        }
        out.push(run_one(&mut uc, &magic_seen, cmd));
    }
    out
}
