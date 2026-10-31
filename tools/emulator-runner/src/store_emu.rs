//! The store's Tier-2 executor: a PERSISTENT-flash, two-phase Unicorn emulator with an FMC board
//! model, plus a host crafted-region load path.
//!
//! The dummy's single-shot `run_image` rebuilds the emulator and reloads the image every call, so it
//! cannot carry flash across the reset between the store's phases (`set+persist` -> reset -> `read`).
//! [`StoreEmu`] keeps ONE Unicorn instance alive: each [`StoreEmu::run_phase`] repaints RAM only and
//! re-runs from the reset vector, so the flash-backed memory (the store region) survives. The host
//! drives the whole scenario x phase matrix over `CMD_ADDR` against a single `store-test` image.
//!
//! Three additions over the dummy runner, all here:
//! 1. a **persistent-flash two-phase run** ([`StoreEmu::run_phase`]);
//! 2. a mapped **FMC peripheral model** ([`crate::fmc_model`]) at `0x4002_2000`, servicing the exact
//!    registers `runtime-hal::fmc` drives, backing the flash region with the mapped memory so the
//!    model and the real driver provably agree;
//! 3. a host **"load crafted region image"** path ([`StoreEmu::load_region`]) for the planted
//!    torn-write / `Full` / compaction scenarios;
//! 4. a **measured stack high-water** ([`StoreEmu::stack_depth`]) over the painted RAM, which is
//!    what the two stack gates in `tests/store_scenarios.rs` are built from.
//!
//! **What the stack high-water covers, and what it does not.** It measures the `store-test` image,
//! not `motorcontroller/firmware`'s, so it is not a bound on the shipped image's deepest chain; that number
//! comes from the shipped ELF's `.stack_sizes` and from the board's own boot paint
//! (`specs/firmware.md`, "RAM/stack budget"). What transfers is the store code the two images share,
//! and only where a gate states a DIFFERENCE between two scenarios run on this one image: the
//! registry `lookup`'s marginal cost (`dynamic_config_write_costs_no_extra_stack_chip1k`) and the
//! compaction chain's marginal staging (`compaction_costs_no_record_sized_stack_chip1k`). A stack
//! regression in either of those is a CI failure here rather than a bench discovery. Anything else
//! on the firmware's chain, `service_loop`, `ingest`, the resolvers, the ISRs, is not in this image
//! and is not gated here.
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
use unicorn_engine::{Arch, ArmCpuModel, HookType, Mode, Prot, RegisterARM, Unicorn};

use crate::fmc_board::{attach, FLASH_BASE};
use crate::fmc_model::FmcModel;

/// Flash window to map: 256 KiB, covering the chip2k store region at the top of the 256 KiB extent
/// (`0x0803_F000`) as well as the low-flash code. A 4 KiB-multiple, the `mem_protect` unit.
const FLASH_SIZE: u64 = 256 * 1024;
/// GD32 RAM base.
const RAM_BASE: u64 = 0x2000_0000;
/// RAM window to map (8 KiB part + the reserved tail words).
const RAM_SIZE: u64 = 8 * 1024;

/// The word RAM is filled with before each phase, so an untouched stack word is distinguishable from
/// a written one ([`StoreEmu::stack_depth`]).
///
/// This is the emulator's version of the firmware's own boot stack paint (`motorcontroller/firmware`'s
/// `paint_stack`), and it measures the same quantity the bench reads out of `CTRL_OBS`. Doing it here
/// costs the image nothing and needs no hardware: the emulator owns RAM before the reset vector runs,
/// so the paint cannot disturb the run and no unsound write-below-the-stack-pointer is involved.
///
/// `0xA5` bytes rather than zero because zero is what `.bss` init writes and what a cleared local
/// looks like: a distinctive pattern is what makes "never written" a decidable question.
const RAM_PAINT: u8 = 0xA5;

/// Hang backstop: max instructions per phase before the run is called "not ready". The store's
/// mount/scan/append over the FMC model is more work than the dummy's handful of instructions, so
/// this is sized generously while still bounding a hung/faulted image.
const MAX_INSTRUCTIONS: usize = 50_000_000;

/// The store Tier-2 executor: one persistent Unicorn instance with the FMC model and the store-region
/// geometry, re-run from the reset vector per phase.
pub struct StoreEmu {
    emu: Unicorn<'static, ()>,
    fmc: Rc<RefCell<FmcModel>>,
    /// Absolute base of the store region (the top two pages of the modelled flash extent).
    store_base: u64,
    /// Region length in bytes (`2 * page_size`).
    region_len: usize,
    /// The lowest RAM address the stack may reach before it collides with static data: the end of the
    /// image's highest RAM `PT_LOAD` segment (`p_vaddr + p_memsz`, which covers `.bss`, not just the
    /// `.data` bytes in the file). The floor of the [`stack_depth`](StoreEmu::stack_depth) scan.
    ram_static_end: u64,
    /// The initial stack pointer (vector table word 0) = the top of the stack region.
    stack_top: u64,
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

        let mut emu = Unicorn::new(Arch::ARM, Mode::THUMB).expect("unicorn init");
        // Use a Cortex-M3 CPU model: the store's FMC critical section runs `mrs r, PRIMASK` /
        // `cpsid i` (M-profile-only system instructions), which the default A-profile core rejects as
        // INSN_INVALID. (The dummy needs none of these, so its runner leaves the default core.)
        emu.ctl_set_cpu_model(ArmCpuModel::CORTEX_M3 as i32)
            .expect("set cortex-m3 cpu model");

        // Flash erased and behind the FMC model (`crate::fmc_board`); RAM mapped ALL.
        let fmc = attach(&mut emu, FLASH_SIZE, page_size);
        emu.mem_map(RAM_BASE, RAM_SIZE, Prot::ALL).expect("map ram");

        // Load every PT_LOAD segment at its PHYSICAL address (p_paddr / LMA), NOT its virtual address.
        // This is load-bearing for the store image: `.data` (the RAM-resident FMC critical section)
        // has VMA 0x2000_0000 but LMA in flash, and cortex-m-rt's Reset copies it from the LMA to RAM
        // at startup. Loading by VMA would put `.data` straight into RAM where the RAM-clear wipes it
        // and the Reset copy reads erased flash, so the FMC code never makes it to RAM (the dummy has
        // no `.data`, so its VMA-based loader sufficed). The release ELF has no symbol table; the
        // program headers carry the bytes + their physical addresses.
        let endian = elf.endian();
        let data_ref: &[u8] = &bytes;
        // The stack-scan floor, gathered on the same pass: the highest RAM address any loadable
        // segment RESERVES. `p_memsz` (not `p_filesz`) is the load-bearing part, since `.bss` has no
        // bytes in the file but does occupy RAM below the stack.
        let mut ram_static_end = RAM_BASE;
        for ph in elf.elf_program_headers() {
            if ph.p_type(endian) != PT_LOAD {
                continue;
            }
            let paddr = ph.p_paddr(endian) as u64;
            let vaddr = ph.p_vaddr(endian) as u64;
            let memsz = ph.p_memsz(endian) as u64;
            if (RAM_BASE..RAM_BASE + RAM_SIZE).contains(&vaddr) {
                ram_static_end = ram_static_end.max((vaddr + memsz).next_multiple_of(4));
            }
            let seg_data = ph.data(endian, data_ref).map_err(|_| {
                std::io::Error::new(std::io::ErrorKind::InvalidData, "segment data")
            })?;
            if seg_data.is_empty() {
                continue;
            }
            emu.mem_write(paddr, seg_data).expect("load segment at LMA");
        }

        // The initial stack pointer is vector table word 0; the stack grows down from it.
        let mut vt0 = [0u8; 4];
        emu.mem_read(FLASH_BASE, &mut vt0).expect("read initial SP");
        let stack_top = u32::from_le_bytes(vt0) as u64;

        // The store region is the top two pages of the chip's flash EXTENT (matching FmcFlash).
        // The placement rule's one owner (store::geometry): the top two pages of the extent.
        let store_base = store::geometry::store_base(flash_size, page_size) as u64;
        Ok(Self {
            emu,
            fmc,
            store_base,
            region_len: 2 * page_size as usize,
            ram_static_end,
            stack_top,
        })
    }

    /// Bytes of stack the LAST [`run_phase`](StoreEmu::run_phase) actually used, measured.
    ///
    /// The phase starts with RAM filled with [`RAM_PAINT`], so a stack word the run never touched
    /// still reads as paint. The deepest point reached is therefore the LOWEST address in
    /// `[ram_static_end, stack_top)` that no longer reads as paint, and the depth is
    /// `stack_top - that address`. Scanning UPWARD from the static floor (rather than down from the
    /// stack pointer) is the conservative direction: a frame that allocated slots it never wrote
    /// still counts, because the scan stops at the first disturbed word below everything.
    ///
    /// It is a direct measurement rather than an estimate from frame sizes, and the image it measures
    /// is `store-test`, the one running here. It says nothing about `motorcontroller/firmware`'s own depth: a
    /// figure from this is usable against another scenario's figure from this, on the same image, and
    /// a bare absolute carries over to nothing. It is also the COMPLEMENT of what
    /// `tools/stack-margin.sh` reports off a board, on a third image again: this is DEPTH BELOW the
    /// stack top, that is FREE PAINT ABOVE `_stack_end`, and the two sum to the region only within one
    /// image. Do not compare the numbers across the three.
    pub fn stack_depth(&mut self) -> u64 {
        let len = (self.stack_top - self.ram_static_end) as usize;
        let mut ram = vec![0u8; len];
        self.emu
            .mem_read(self.ram_static_end, &mut ram)
            .expect("read stack region");
        for (i, w) in ram.as_chunks::<4>().0.iter().enumerate() {
            if *w != [RAM_PAINT; 4] {
                let low_water = self.ram_static_end + (i * 4) as u64;
                return self.stack_top - low_water;
            }
        }
        0
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

    /// Read the device's whole store region back out of the flash-backed memory
    /// (`2 * page_size` bytes, the same span [`load_region`](StoreEmu::load_region) plants).
    ///
    /// What a test does with it is check the device's own on-flash bookkeeping, which is the only
    /// way the host can tell a run that COMPACTED from a run that did not: compaction is the one
    /// operation that commits a page header on the spare side.
    pub fn read_region(&mut self) -> Vec<u8> {
        let mut out = vec![0u8; self.region_len];
        self.emu
            .mem_read(self.store_base, &mut out)
            .expect("read store region");
        out
    }

    /// Run one phase: clear RAM, deliver `cmd = (scenario << 16) | phase` to `CMD_ADDR`, run from the
    /// reset vector keeping the persistent flash, and read the published [`TestResult`] back. `ready`
    /// not reaching `RESULT_READY` (a hang/fault) leaves `ready` at 0, which the caller treats as a
    /// caught failure (the `store-hang` / negative-control cases).
    pub fn run_phase(&mut self, cmd: u32) -> TestResult {
        // PAINT RAM (flash persists across the "reboot"), so the stack region carries a known pattern
        // and [`stack_depth`](StoreEmu::stack_depth) can tell a touched word from an untouched one.
        // The image initialises everything it relies on (cortex-m-rt's Reset copies `.data` and zeroes
        // `.bss`), so paint rather than zero changes nothing about the run.
        let paint = vec![RAM_PAINT; RAM_SIZE as usize];
        self.emu.mem_write(RAM_BASE, &paint).expect("paint ram");
        // The reserved tail is NOT part of the measurement and must start clear: `ready` staying 0 is
        // how a hang / fault is caught, and painted bytes there would read as a published result.
        let tail_base = RESULT_ADDR as u64;
        let zeros = vec![0u8; (RAM_BASE + RAM_SIZE - tail_base) as usize];
        self.emu
            .mem_write(tail_base, &zeros)
            .expect("clear reserved tail");

        // Reset the FMC controller state between phases (a clean reboot of the controller model: LK
        // set, no sticky flags). The flash bytes persist across the reboot, which is the whole point.
        {
            let ps = self.fmc.borrow().page_size();
            *self.fmc.borrow_mut() = FmcModel::new(ps);
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
