//! The store's Tier-2 executor: a PERSISTENT-flash, two-phase Unicorn emulator with an FMC board
//! model, plus a host crafted-region load path.
//!
//! The dummy's single-shot `run_image` rebuilds the emulator and reloads the image every call, so it
//! cannot carry flash across the reset between the store's phases (`set+persist` -> reset -> `read`).
//! [`StoreEmu`] keeps ONE Unicorn instance alive: each [`StoreEmu::run_phase`] clears RAM only and
//! re-runs from the reset vector, so the flash-backed memory (the store region) survives. The host
//! drives the whole scenario x phase matrix over `CMD_ADDR` against a single `store-test` image.
//!
//! Three additions over the dummy runner, all here:
//! 1. a **persistent-flash two-phase run** ([`StoreEmu::run_phase`]);
//! 2. a mapped **FMC peripheral model** ([`crate::fmc_model`]) at `0x4002_2000`, servicing the exact
//!    registers `runtime-hal::fmc` drives, backing the flash region with the mapped memory so the
//!    model and the real driver provably agree;
//! 3. a host **"load crafted region image"** path ([`StoreEmu::load_region`]) for the planted
//!    torn-write / `Full` / compaction scenarios.
//!
//! Flash is mapped READ+EXEC (not writable), so a halfword program store FAULTS to a write-protect
//! mem-hook the FMC model services: it applies write-once (a re-program of a non-`0xFFFF` cell sets
//! `PGERR` and leaves the cell unchanged) and lets a fresh program land. Page erase fills `0xFFFF`.

use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use object::elf::PT_LOAD;
use object::read::elf::{ElfFile32, ProgramHeader};
use object::LittleEndian;
use test_shared::{TestResult, CMD_ADDR, RESULT_ADDR, RESULT_BUF_LEN};
use unicorn_engine::{Arch, ArmCpuModel, HookType, MemType, Mode, Prot, RegisterARM, Unicorn};

use crate::fmc_model::{CtlEffect, FmcModel, ProgramOutcome, FMC_BASE, FMC_WINDOW};

/// GD32 flash base.
const FLASH_BASE: u64 = 0x0800_0000;
/// Flash window to map: 256 KiB, covering the chip2k store region at the top of the 256 KiB extent
/// (`0x0803_F000`) as well as the low-flash code. A 4 KiB-multiple, the `mem_protect` unit.
const FLASH_SIZE: u64 = 256 * 1024;
/// GD32 RAM base.
const RAM_BASE: u64 = 0x2000_0000;
/// RAM window to map (8 KiB part + the reserved tail words).
const RAM_SIZE: u64 = 8 * 1024;

/// Hang backstop: max instructions per phase before the run is called "not ready". The store's
/// mount/scan/append over the FMC model is more work than the dummy's handful of instructions, so
/// this is sized generously while still bounding a hung/faulted image.
const MAX_INSTRUCTIONS: usize = 50_000_000;

/// Flash mapped non-writable so a program store faults into the write-protect hook the FMC model
/// services.
fn flash_ro() -> Prot {
    Prot::READ | Prot::EXEC
}

/// Shared mutable glue state between the FMC MMIO callbacks and the flash write-protect hook.
struct Shared {
    fmc: FmcModel,
}

/// The store Tier-2 executor: one persistent Unicorn instance with the FMC model and the store-region
/// geometry, re-run from the reset vector per phase.
pub struct StoreEmu {
    emu: Unicorn<'static, ()>,
    shared: Rc<RefCell<Shared>>,
    /// Absolute base of the store region (the top two pages of the modelled flash extent).
    store_base: u64,
    /// Region length in bytes (`2 * page_size`).
    region_len: usize,
}

impl StoreEmu {
    /// Build a persistent emulator for a part with `page_size` (1024 or 2048) and a `flash_size`
    /// (the chip's flash EXTENT in bytes: 64 KiB for chip1k, 256 KiB for chip2k) and load `image`
    /// once. The store region is the top two pages of that EXTENT (NOT of the mapped window):
    /// `store_base = (FLASH_BASE + flash_size) - 2 * page_size`, matching the on-target `FmcFlash`
    /// geometry for the corresponding `Chip` (chip1k -> `0x0800_F800`; chip2k -> `0x0803_F000`).
    ///
    /// Unicorn maps memory zero-initialized, but erased NOR flash reads `0xFFFF`, and the store's
    /// virgin-region logic + the FMC model's write-once both depend on that, so the whole mapped flash
    /// is filled with `0xFF` before the image's segments are loaded over the low-flash code.
    pub fn new(image: &Path, page_size: u32, flash_size: u32) -> std::io::Result<Self> {
        let bytes = std::fs::read(image)?;
        let elf = ElfFile32::<LittleEndian>::parse(&*bytes)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e.to_string()))?;

        let shared = Rc::new(RefCell::new(Shared {
            fmc: FmcModel::new(page_size),
        }));

        let mut emu = Unicorn::new(Arch::ARM, Mode::THUMB).expect("unicorn init");
        // Use a Cortex-M3 CPU model: the store's FMC critical section runs `mrs r, PRIMASK` /
        // `cpsid i` (M-profile-only system instructions), which the default A-profile core rejects as
        // INSN_INVALID. (The dummy needs none of these, so its runner leaves the default core.)
        emu.ctl_set_cpu_model(ArmCpuModel::CORTEX_M3 as i32)
            .expect("set cortex-m3 cpu model");

        // Flash mapped READ+EXEC (not writable); RAM mapped ALL.
        emu.mem_map(FLASH_BASE, FLASH_SIZE, flash_ro())
            .expect("map flash");
        emu.mem_map(RAM_BASE, RAM_SIZE, Prot::ALL).expect("map ram");

        // Initialize the whole flash to the erased state (0xFF), since Unicorn maps it zeroed. Use the
        // host backdoor (flash is mapped read-only).
        let erased = vec![0xFFu8; FLASH_SIZE as usize];
        emu.mem_write(FLASH_BASE, &erased).expect("erase flash");

        // Load every PT_LOAD segment at its PHYSICAL address (p_paddr / LMA), NOT its virtual address.
        // This is load-bearing for the store image: `.data` (the RAM-resident FMC critical section)
        // has VMA 0x2000_0000 but LMA in flash, and cortex-m-rt's Reset copies it from the LMA to RAM
        // at startup. Loading by VMA would put `.data` straight into RAM where the RAM-clear wipes it
        // and the Reset copy reads erased flash, so the FMC code never makes it to RAM (the dummy has
        // no `.data`, so its VMA-based loader sufficed). The release ELF has no symbol table; the
        // program headers carry the bytes + their physical addresses.
        let endian = elf.endian();
        let data_ref: &[u8] = &bytes;
        for ph in elf.elf_program_headers() {
            if ph.p_type(endian) != PT_LOAD {
                continue;
            }
            let paddr = ph.p_paddr(endian) as u64;
            let seg_data = ph.data(endian, data_ref).map_err(|_| {
                std::io::Error::new(std::io::ErrorKind::InvalidData, "segment data")
            })?;
            if seg_data.is_empty() {
                continue;
            }
            emu.mem_write(paddr, seg_data).expect("load segment at LMA");
        }

        // The FMC MMIO model at 0x4002_2000: a register read serves the FMC registers (BUSY reads 0,
        // plus the sticky flags); a register write services KEY/STAT/ADDR and, on CTL=PER|START,
        // performs the page erase (fill 0xFFFF via the host backdoor, which bypasses the read-only
        // flash protection).
        let read_shared = shared.clone();
        let write_shared = shared.clone();
        emu.mmio_map(
            FMC_BASE,
            FMC_WINDOW,
            Some(
                move |_uc: &mut Unicorn<'_, ()>, off: u64, _size: usize| -> u64 {
                    read_shared.borrow().fmc.read_reg(off) as u64
                },
            ),
            Some(
                move |uc: &mut Unicorn<'_, ()>, off: u64, _size: usize, value: u64| {
                    let (eff, ps) = {
                        let mut s = write_shared.borrow_mut();
                        let eff = s.fmc.write_reg(off, value as u32);
                        (eff, s.fmc.page_size() as usize)
                    };
                    if let CtlEffect::ErasePage(base) = eff {
                        let blank = vec![0xFFu8; ps];
                        uc.mem_write(base as u64, &blank).expect("erase fill");
                    }
                },
            ),
        )
        .expect("map FMC model");

        // The flash write-protect hook: a halfword program store (`strh` to a flash address while PG is
        // set) faults here because flash is mapped read-only. The FMC model decides write-once; we
        // PERFORM the write ourselves through the host backdoor (`mem_write`, which bypasses the
        // protection) and return `true` so Unicorn skips the original faulting store. A fresh cell is
        // written; a re-program of a non-`0xFFFF` cell is skipped (PGERR set, cell unchanged, exactly
        // like silicon). A store outside a program sequence is a genuine fault (return false).
        let hook_shared = shared.clone();
        emu.add_mem_hook(
            HookType::MEM_WRITE_PROT,
            FLASH_BASE,
            FLASH_BASE + FLASH_SIZE - 1,
            move |uc: &mut Unicorn<'_, ()>,
                  _mem: MemType,
                  addr: u64,
                  size: usize,
                  val: i64|
                  -> bool {
                let mut cur = [0u8; 2];
                uc.mem_read(addr, &mut cur).expect("read cell");
                let cur_hw = u16::from_le_bytes(cur);
                let outcome = hook_shared.borrow_mut().fmc.apply_program_halfword(cur_hw);
                match outcome {
                    ProgramOutcome::Write => {
                        // Fresh cell: apply the program write via the backdoor (size is the store
                        // width, normally 2; mask the value to that width).
                        let bytes = (val as u64).to_le_bytes();
                        let n = size.min(8);
                        uc.mem_write(addr, &bytes[..n]).expect("program write");
                        true
                    }
                    // Re-program: skip the write, PGERR was set, cell stays unchanged (write-once).
                    ProgramOutcome::Rejected => true,
                    // A write outside a program sequence is a genuine fault.
                    ProgramOutcome::NotProgramming => false,
                }
            },
        )
        .expect("add flash write-protect hook");

        // The store region is the top two pages of the chip's flash EXTENT (matching FmcFlash).
        // The placement rule's one owner (store::geometry): the top two pages of the extent.
        let store_base = store::geometry::store_base(flash_size, page_size) as u64;
        Ok(Self {
            emu,
            shared,
            store_base,
            region_len: 2 * page_size as usize,
        })
    }

    /// Load a hand-built store-region byte image into the device's flash-backed memory before a read
    /// phase (the planted torn-write / `Full` / compaction scenarios). `image.len()` must equal the
    /// region length (`2 * page_size`). Writes go through the host backdoor (flash is mapped R-only).
    pub fn load_region(&mut self, image: &[u8]) {
        assert_eq!(
            image.len(),
            self.region_len,
            "crafted region image must be exactly two pages ({} bytes)",
            self.region_len
        );
        // `mem_write` is the host backdoor; it bypasses the read-only flash protection.
        self.emu
            .mem_write(self.store_base, image)
            .expect("load crafted region");
    }

    /// Run one phase: clear RAM, deliver `cmd = (scenario << 16) | phase` to `CMD_ADDR`, run from the
    /// reset vector keeping the persistent flash, and read the published [`TestResult`] back. `ready`
    /// not reaching `RESULT_READY` (a hang/fault) leaves `ready` at 0, which the caller treats as a
    /// caught failure (the `store-hang` / negative-control cases).
    pub fn run_phase(&mut self, cmd: u32) -> TestResult {
        // Clear RAM only (flash persists across the "reboot"). This zeroes the reserved tail too, so we
        // re-deliver CMD_ADDR after.
        let zeros = vec![0u8; RAM_SIZE as usize];
        self.emu.mem_write(RAM_BASE, &zeros).expect("clear ram");

        // Reset the FMC controller state between phases (a clean reboot of the controller model: LK
        // set, no sticky flags). The flash bytes persist across the reboot, which is the whole point.
        {
            let ps = self.shared.borrow().fmc.page_size();
            self.shared.borrow_mut().fmc = FmcModel::new(ps);
        }

        // Vector table at flash base: word 0 = initial SP, word 1 = reset vector (PC, Thumb bit set).
        let mut vt = [0u8; 8];
        self.emu
            .mem_read(FLASH_BASE, &mut vt)
            .expect("read vector table");
        let sp = u32::from_le_bytes([vt[0], vt[1], vt[2], vt[3]]);
        let reset = u32::from_le_bytes([vt[4], vt[5], vt[6], vt[7]]);
        self.emu
            .reg_write(RegisterARM::SP, sp as u64)
            .expect("set sp");

        // Deliver the host cmd to the reserved RAM tail (survives the device-side reset / .bss clear).
        self.emu
            .mem_write(CMD_ADDR as u64, &cmd.to_le_bytes())
            .expect("write cmd");

        // Stop as soon as the firmware writes `ready` (RESULT_ADDR, written LAST); the value has landed
        // by the time we read it back. A hung image never triggers this and hits the instruction cap.
        let hook_id = self
            .emu
            .add_mem_hook(
                HookType::MEM_WRITE,
                RESULT_ADDR as u64,
                RESULT_ADDR as u64 + 3,
                move |uc: &mut Unicorn<'_, ()>, _m, _a, _s, _v| -> bool {
                    let _ = uc.emu_stop();
                    true
                },
            )
            .expect("add ready-write hook");

        let begin = (reset as u64) | 1;
        // The instruction cap is the hang backstop: an image that never publishes hits it and returns
        // with `ready` unwritten (0), which the caller reads as a caught failure.
        let _ = self
            .emu
            .emu_start(begin, FLASH_BASE + FLASH_SIZE, 0, MAX_INSTRUCTIONS);
        let _ = self.emu.remove_hook(hook_id);

        // Read the result back (ready @ +0, output @ +4, len @ +8, buf @ +10).
        let mut head = [0u8; 10];
        self.emu
            .mem_read(RESULT_ADDR as u64, &mut head)
            .expect("read result head");
        let len = u16::from_le_bytes([head[8], head[9]]);
        let mut buf = [0u8; RESULT_BUF_LEN];
        self.emu
            .mem_read(RESULT_ADDR as u64 + 10, &mut buf)
            .expect("read result buf");
        TestResult {
            ready: u32::from_le_bytes([head[0], head[1], head[2], head[3]]),
            output: u32::from_le_bytes([head[4], head[5], head[6], head[7]]),
            len,
            buf,
        }
    }
}
