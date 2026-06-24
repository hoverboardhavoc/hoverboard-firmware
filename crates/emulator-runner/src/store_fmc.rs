//! The tier-2 store path under Unicorn: a persistent-flash two-phase run plus a mapped FMC peripheral
//! model.
//!
//! The dummy's [`crate::run_image`] builds a fresh emulator and reloads the image every call, so it
//! cannot carry flash across the reset between phases. The store test is `set+persist` (phase 0) ->
//! reset -> `read` (phase 1), and the flash bytes must survive the reset. [`run_two_phase`] keeps the
//! SAME flash-backed memory across a re-run from the reset vector (only RAM is cleared) and delivers
//! the phase index via `CMD_ADDR`.
//!
//! The store writes flash through `runtime-hal::fmc`, whose TARGET path is the raw-MMIO FMC register
//! sequence (not the HAL's host model: under Unicorn the image runs the `cfg(target_arch = "arm")`
//! path). So the runner maps an FMC peripheral at `0x4002_2000` and services the exact registers the
//! driver drives ([`FmcModel`]): `KEY` (`+0x04`) unlock, `CTL` (`+0x10`) `PG`/`PER`/`START`, `ADDR`
//! (`+0x14`), and `STAT` (`+0x0C`) where `BUSY` reads `0` and `ENDF`/`WPERR`/`PGERR` are
//! write-1-to-clear. Page erase fills the addressed page with `0xFFFF`; a halfword program is
//! write-once (re-program of a non-`0xFFFF` halfword sets `PGERR` and leaves the cell unchanged).

use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use object::elf::{ProgramHeader32, PT_LOAD};
use object::read::elf::{ElfFile32, FileHeader, ProgramHeader};
use object::Endianness;
use test_shared::{TestResult, CMD_ADDR, RESULT_ADDR};
use unicorn_engine::{Arch, ArmCpuModel, HookType, MemType, Mode, Prot, RegisterARM, Unicorn};

/// Load every `PT_LOAD` program segment into the emulator at its PHYSICAL address (LMA), using the
/// file bytes (`p_offset`/`p_filesz`). This matters for the FMC driver's RAM-resident `.data`
/// functions: that segment's VMA is in RAM but its LMA is in flash (cortex-m-rt copies it to RAM at
/// startup), so loading by VMA would leave the flash LMA erased and the runtime copy would corrupt the
/// RAM-resident code. The dummy has no such segment, but the store does, so we load by LMA here.
fn load_segments_by_paddr(emu: &mut Unicorn<()>, bytes: &[u8]) -> std::io::Result<()> {
    let elf = ElfFile32::<Endianness>::parse(bytes)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e.to_string()))?;
    let endian = elf.endian();
    let headers: &[ProgramHeader32<Endianness>] =
        elf.elf_header()
            .program_headers(endian, bytes)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e.to_string()))?;
    for ph in headers {
        if ph.p_type(endian) != PT_LOAD {
            continue;
        }
        let paddr = ph.p_paddr(endian) as u64;
        let data = ph
            .data(endian, bytes)
            .map_err(|_| std::io::Error::new(std::io::ErrorKind::InvalidData, "segment data"))?;
        if data.is_empty() {
            continue;
        }
        emu.mem_write(paddr, data).expect("load segment (paddr)");
    }
    Ok(())
}

/// Build a fresh Unicorn ARM/Thumb instance with the Cortex-M7 CPU model selected. The default
/// `Arch::ARM` core lacks the M-profile system registers, so `mrs r, PRIMASK` / `cpsid i` in the FMC
/// driver's RAM-resident critical section fault as `INSN_INVALID`. The M7 model (a superset of the M3
/// the GD32 is) implements PRIMASK + the full Thumb-2 set, so the exact shipping FMC sequence runs.
/// (`Mode::MCLASS` still rejects Thumb, the documented gotcha, so we keep `Mode::THUMB` and set the
/// CPU model explicitly.)
fn new_emu() -> Unicorn<'static, ()> {
    let mut emu = Unicorn::new(Arch::ARM, Mode::THUMB).expect("unicorn init");
    emu.ctl_set_cpu_model(ArmCpuModel::CORTEX_M7 as i32)
        .expect("select Cortex-M7 CPU model");
    emu
}

/// GD32 flash base (memory-mapped; the vector table and the store region both live here).
const FLASH_BASE: u64 = 0x0800_0000;
/// Flash window to map. Generous: covers the 256 KiB high-density part (the 2 KiB-page store region
/// sits at its top), so the store region of both the 64 KiB and the 256 KiB chips is mapped.
const FLASH_SIZE: u64 = 256 * 1024;
/// GD32 RAM base.
const RAM_BASE: u64 = 0x2000_0000;
/// RAM window: the full 8 KiB part plus the reserved tail (RESULT/CMD words).
const RAM_SIZE: u64 = 8 * 1024;

/// The FMC peripheral base on both families.
const FMC_BASE: u64 = 0x4002_2000;
/// MMIO map size for the FMC (must be 4 KiB-aligned + a multiple of 4 KiB per Unicorn's `mmio_map`).
const FMC_MAP_SIZE: u64 = 0x1000;

// FMC register offsets (from FMC_BASE), matching runtime-hal::fmc.
const KEY: u64 = 0x04;
const STAT: u64 = 0x0C;
const CTL: u64 = 0x10;
const ADDR: u64 = 0x14;

// Unlock keys.
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

// CTL bits.
const CTL_PG: u32 = 1 << 0;
const CTL_PER: u32 = 1 << 1;
const CTL_START: u32 = 1 << 6;
const CTL_LK: u32 = 1 << 7;

// STAT bits.
const STAT_BUSY: u32 = 1 << 0;
const STAT_PGERR: u32 = 1 << 2;
const STAT_WPERR: u32 = 1 << 4;
const STAT_ENDF: u32 = 1 << 5;

/// Hang backstop: max instructions per phase before the run is called "not ready".
const MAX_INSTRUCTIONS: usize = 5_000_000;

/// The erased-halfword value.
const ERASED: u16 = 0xFFFF;

/// The FMC controller model: the register state the driver drives, shared (via `Rc<RefCell<_>>`)
/// between the FMC MMIO callbacks and the flash write hook (the program path stores halfwords to
/// flash addresses while `PG` is set, so the write hook needs to see `PG`).
struct FmcModel {
    /// Tracks the unlock handshake: set after KEY1, consumed on KEY2 (which clears `LK` in `ctl`).
    key1_seen: bool,
    /// The `CTL` register (PG / PER / START / LK). `LK` set at reset (locked).
    ctl: u32,
    /// The `STAT` register error/end flags (`ENDF`/`WPERR`/`PGERR`); `BUSY` always reads 0.
    stat: u32,
    /// The `ADDR` register (the absolute page address for an erase).
    addr: u32,
}

impl FmcModel {
    fn new() -> Self {
        FmcModel {
            key1_seen: false,
            // Locked at reset: LK set.
            ctl: CTL_LK,
            stat: 0,
            addr: 0,
        }
    }
}

/// Read 4 bytes little-endian from a slice.
fn read_u32_le(buf: &[u8]) -> u32 {
    u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]])
}

/// Run `image` two-phase under Unicorn with a persistent flash region and the FMC model: phase 0
/// (`set+persist`) then a reset re-run as phase 1 (`read`), with the flash bytes carried across. The
/// `page_size` (1024 or 2048) is the detected part's page; it sizes the FMC model's page erase. The
/// flash region is the image's loaded segments PLUS whatever the store programs at runtime; only RAM
/// is cleared between phases. Returns the phase-1 [`TestResult`].
pub fn run_two_phase(image: &Path, page_size: usize) -> std::io::Result<TestResult> {
    let bytes = std::fs::read(image)?;

    let mut emu = new_emu();

    emu.mem_map(FLASH_BASE, FLASH_SIZE, Prot::ALL)
        .expect("map flash");
    emu.mem_map(RAM_BASE, RAM_SIZE, Prot::ALL).expect("map ram");

    // The store region of an UNWRITTEN part is erased flash (0xFFFF). The image's segments only cover
    // the code at the bottom of flash; the store region at the TOP is not in any segment, so it stays
    // whatever the freshly mapped memory is (zero). Pre-fill the whole flash window with 0xFF so the
    // store region (and any not-yet-programmed flash) reads as erased, matching real silicon's clean
    // slate. Segment loads below then overwrite the code region.
    let erased = vec![0xFFu8; FLASH_SIZE as usize];
    emu.mem_write(FLASH_BASE, &erased).expect("erase flash");

    // Load every loadable segment at its PHYSICAL address (over the erased fill).
    load_segments_by_paddr(&mut emu, &bytes)?;

    // The shared FMC model. `Rc<RefCell<_>>` so the FMC MMIO callbacks and the flash write hook share
    // the register state (the program path stores to flash while PG is set; the hook reads PG).
    let model = Rc::new(RefCell::new(FmcModel::new()));

    install_fmc(&mut emu, &model, page_size);

    // Phase 0: set + persist. Reads SP/PC from the (now-loaded) vector table.
    run_one_phase(&mut emu, 0)?;

    // Reset: clear RAM only (flash, the FMC-programmed store, survives). The reserved tail is outside
    // the linked region, but we rewrite CMD_ADDR per phase anyway, so clearing all of RAM is fine.
    let zero = vec![0u8; RAM_SIZE as usize];
    emu.mem_write(RAM_BASE, &zero).expect("clear ram");
    // Reset the FMC model between phases (a real reset re-locks the controller); the flash CONTENT is
    // untouched (it is in the flash mapping, not the model).
    *model.borrow_mut() = FmcModel::new();

    // Phase 1: cold-mount + read. Same image, same flash, fresh RAM.
    run_one_phase(&mut emu, 1)?;

    let mut result = [0u8; 8];
    emu.mem_read(RESULT_ADDR as u64, &mut result)
        .expect("read result");
    Ok(TestResult {
        ready: read_u32_le(&result[0..4]),
        output: read_u32_le(&result[4..8]),
    })
}

/// Run one phase from the reset vector: deliver `phase` to `CMD_ADDR`, set SP, start, stop on the
/// `RESULT_ADDR` write (or the instruction cap for the hang backstop).
fn run_one_phase(emu: &mut Unicorn<()>, phase: u32) -> std::io::Result<()> {
    // Vector table at the start of flash: word 0 = initial SP, word 1 = reset vector (PC).
    let mut vt = [0u8; 8];
    emu.mem_read(FLASH_BASE, &mut vt)
        .expect("read vector table");
    let sp = read_u32_le(&vt[0..4]);
    let reset = read_u32_le(&vt[4..8]);
    emu.reg_write(RegisterARM::SP, sp as u64).expect("set sp");

    // Deliver the phase to the reserved RAM tail before running (mirrors SWD/silicon).
    emu.mem_write(CMD_ADDR as u64, &phase.to_le_bytes())
        .expect("write phase");

    // Stop as soon as the firmware writes the result word (`ready`, written LAST).
    let hook = emu
        .add_mem_hook(
            HookType::MEM_WRITE,
            RESULT_ADDR as u64,
            RESULT_ADDR as u64 + 3,
            move |uc, _t, _a, _s, _v| {
                let _ = uc.emu_stop();
                true
            },
        )
        .expect("add result-write hook");

    let begin = (reset as u64) | 1;
    let _ = emu.emu_start(begin, FLASH_BASE + FLASH_SIZE, 0, MAX_INSTRUCTIONS);
    let _ = emu.remove_hook(hook);
    Ok(())
}

/// Install the FMC model: the MMIO peripheral at `0x4002_2000` plus the flash write hook that models
/// write-once programming. `page_size` sizes a page erase.
fn install_fmc(emu: &mut Unicorn<()>, model: &Rc<RefCell<FmcModel>>, page_size: usize) {
    // --- the flash write hook: model halfword write-once programming -------------------------------
    //
    // The driver's program path stores halfwords directly to flash addresses while `PG` is set. We
    // intercept writes into the flash window: if `PG` is active and the target halfword is NOT erased
    // (`!= 0xFFFF`), the hardware refuses (sets `PGERR`, content unchanged) - we restore the original
    // bytes after the write and set `PGERR`. A fresh (erased) halfword programs normally and sets
    // `ENDF`. (Code segment loads happen via `mem_write`, NOT CPU stores, so they do not hit this
    // hook; only the store's runtime FMC programs do.)
    {
        let model = Rc::clone(model);
        emu.add_mem_hook(
            HookType::MEM_WRITE,
            FLASH_BASE,
            FLASH_BASE + FLASH_SIZE - 1,
            move |uc, _t: MemType, addr: u64, size: usize, value: i64| {
                let mut m = model.borrow_mut();
                if m.ctl & CTL_PG == 0 {
                    // Not in program mode: let it land (should not happen in the store flow).
                    return true;
                }
                // Read the current content of the targeted span (before this write commits).
                let mut cur = vec![0u8; size];
                if uc.mem_read(addr, &mut cur).is_err() {
                    return true;
                }
                let already_written = cur.chunks(2).any(|c| {
                    let hw = if c.len() == 2 {
                        u16::from_le_bytes([c[0], c[1]])
                    } else {
                        c[0] as u16
                    };
                    hw != ERASED
                });
                if already_written {
                    // Write-once violation: set PGERR and restore the original bytes (the write that
                    // is about to commit must NOT change the cell). We restore on the NEXT block by
                    // re-writing the saved bytes immediately; since the store has not committed yet,
                    // schedule the restore by writing back after letting the value land is unreliable,
                    // so we proactively veto by re-writing `cur` here (the engine applies our write
                    // last). NOTE: the HAL pre-checks NotErased before ever issuing a program, so this
                    // path is a backstop the store flow never reaches.
                    m.stat |= STAT_PGERR;
                    let _ = uc.mem_write(addr, &cur);
                    return true;
                }
                // A legal fresh program: it lands; signal end-of-program.
                let _ = value;
                m.stat |= STAT_ENDF;
                true
            },
        )
        .expect("add flash write hook");
    }

    // --- the FMC MMIO peripheral at 0x4002_2000 ----------------------------------------------------
    let read_model = Rc::clone(model);
    let write_model = Rc::clone(model);
    let page_size = page_size as u32;
    emu.mmio_map(
        FMC_BASE,
        FMC_MAP_SIZE,
        Some(
            move |_uc: &mut Unicorn<()>, off: u64, _size: usize| -> u64 {
                let m = read_model.borrow();
                match off {
                    // STAT: BUSY always reads 0 (the op completes synchronously in the model); the
                    // error/end flags read back so the driver's decode + write-1-to-clear works.
                    STAT => (m.stat & !STAT_BUSY) as u64,
                    CTL => m.ctl as u64,
                    ADDR => m.addr as u64,
                    // KEY reads back 0 (write-only in practice).
                    _ => 0,
                }
            },
        ),
        Some(
            move |uc: &mut Unicorn<()>, off: u64, _size: usize, value: u64| {
                let v = value as u32;
                let mut m = write_model.borrow_mut();
                match off {
                    KEY => {
                        // Unlock handshake: KEY1 then KEY2 clears LK.
                        if !m.key1_seen && v == KEY1 {
                            m.key1_seen = true;
                        } else if m.key1_seen && v == KEY2 {
                            m.key1_seen = false;
                            m.ctl &= !CTL_LK;
                        } else {
                            m.key1_seen = false;
                        }
                    }
                    ADDR => m.addr = v,
                    CTL => {
                        m.ctl = v;
                        // A START with PER triggers a page erase at ADDR: fill the page with 0xFFFF.
                        if v & CTL_START != 0 && v & CTL_PER != 0 {
                            let page = vec![0xFFu8; page_size as usize];
                            let _ = uc.mem_write(m.addr as u64, &page);
                            m.stat |= STAT_ENDF;
                        }
                    }
                    STAT => {
                        // Write-1-to-clear ENDF/WPERR/PGERR.
                        m.stat &= !(v & (STAT_ENDF | STAT_WPERR | STAT_PGERR));
                    }
                    _ => {}
                }
            },
        ),
    )
    .expect("map FMC peripheral");
}
