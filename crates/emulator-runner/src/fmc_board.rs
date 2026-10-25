//! The GD32's main flash behind the FMC model, mapped into a Unicorn instance: flash erased and
//! mapped READ+EXEC (not writable), so a halfword program store faults into a write-protect hook
//! the FMC model services, and the FMC registers as MMIO onto the same model. The store's executor
//! ([`crate::store_emu`]) and the hoverboard bridge's RAM loader test (`tests/bridge_loader.rs`)
//! both build their flash with [`attach`].

use std::cell::RefCell;
use std::rc::Rc;

use unicorn_engine::{HookType, MemType, Prot, Unicorn};

use crate::fmc_model::{CtlEffect, FmcModel, ProgramOutcome, FMC_BASE, FMC_WINDOW};

/// GD32 flash base.
pub const FLASH_BASE: u64 = 0x0800_0000;

/// Map `flash_size` bytes of erased flash at [`FLASH_BASE`] and the FMC at `0x4002_2000`, both
/// served by one [`FmcModel`] for a part with `page_size` pages, which is returned so the host can
/// reach the registers and reset the controller.
///
/// Unicorn maps memory zero-initialized, but erased NOR flash reads `0xFFFF`, and the FMC model's
/// write-once depends on that, so the whole mapped flash is filled with `0xFF` first.
pub fn attach(
    emu: &mut Unicorn<'static, ()>,
    flash_size: u64,
    page_size: u32,
) -> Rc<RefCell<FmcModel>> {
    let fmc = Rc::new(RefCell::new(FmcModel::new(page_size)));
    emu.mem_map(FLASH_BASE, flash_size, Prot::READ | Prot::EXEC)
        .expect("map flash");
    // The host backdoor bypasses the read-only protection.
    emu.mem_write(FLASH_BASE, &vec![0xFFu8; flash_size as usize])
        .expect("erase flash");

    // The FMC MMIO model at 0x4002_2000: a register read serves the FMC registers (BUSY reads 0,
    // plus the sticky flags); a register write services KEY/STAT/ADDR and, on CTL=PER|START,
    // performs the page erase (fill 0xFFFF via the host backdoor, which bypasses the read-only
    // flash protection).
    let read_fmc = fmc.clone();
    let write_fmc = fmc.clone();
    emu.mmio_map(
        FMC_BASE,
        FMC_WINDOW,
        Some(
            move |_uc: &mut Unicorn<'_, ()>, off: u64, _size: usize| -> u64 {
                read_fmc.borrow().read_reg(off) as u64
            },
        ),
        Some(
            move |uc: &mut Unicorn<'_, ()>, off: u64, _size: usize, value: u64| {
                let (eff, ps) = {
                    let mut m = write_fmc.borrow_mut();
                    let eff = m.write_reg(off, value as u32);
                    (eff, m.page_size() as usize)
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
    let hook_fmc = fmc.clone();
    emu.add_mem_hook(
        HookType::MEM_WRITE_PROT,
        FLASH_BASE,
        FLASH_BASE + flash_size - 1,
        move |uc: &mut Unicorn<'_, ()>, _mem: MemType, addr: u64, size: usize, val: i64| -> bool {
            let mut cur = [0u8; 2];
            uc.mem_read(addr, &mut cur).expect("read cell");
            let cur_hw = u16::from_le_bytes(cur);
            // The closed-halfword rule needs the cell ABOVE the target too (see
            // `FmcModel::apply_program_halfword`). A target in the very last halfword of flash
            // has no successor to read, so treat it as erased (open).
            let mut succ = [0xFFu8; 2];
            if addr + 2 < FLASH_BASE + flash_size {
                uc.mem_read(addr + 2, &mut succ)
                    .expect("read successor cell");
            }
            let succ_hw = u16::from_le_bytes(succ);
            let outcome =
                hook_fmc
                    .borrow_mut()
                    .apply_program_halfword(addr as u32, cur_hw, succ_hw);
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

    fmc
}
