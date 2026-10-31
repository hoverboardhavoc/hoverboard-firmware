//! The hoverboard bridge's RAM loader (`bridge/loader/f1x.bin`) run under Unicorn,
//! driven by the bridge's own driver (`hoverboard_bridge_core::loader`) through a memory port onto
//! the emulator: the loader's bytes in RAM at `0x2000_0000`, the core registers set through
//! `DCRSR`/`DCRDR`, the ring fed and read as over SWD, the halt from its `bkpt`, and the result
//! read back from the FMC model's flash.
//!
//! The port interleaves the two sides as the hardware does: every access the driver makes lets the
//! core run a slice of instructions while it is not halted.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use emulator_runner::fmc_board::{attach, FLASH_BASE};
use emulator_runner::fmc_model::{FmcModel, FMC_BASE, FMC_WINDOW};
use hoverboard_bridge_core::loader::{self, Outcome, Poll, Run};
use hoverboard_bridge_core::swd::MemPort;
use hoverboard_bridge_core::target::{DCRDR, DCRSR, DHCSR};
use unicorn_engine::{Arch, ArmCpuModel, Mode, Prot, RegisterARM, Unicorn};

const FLASH_SIZE: u64 = 64 * 1024;
const RAM_BASE: u64 = 0x2000_0000;
const RAM_SIZE: u64 = 8 * 1024;
const PAGE: u32 = 1024;
/// Instructions the core runs per access while it runs: a few halfwords' worth, so the driver
/// sees the ring fill and drain rather than a loader that keeps up instantly.
const SLICE: usize = 40;

/// A Cortex-M3 under Unicorn with the GD32's flash and FMC, seen through its debug registers.
struct EmuPort {
    emu: Unicorn<'static, ()>,
    fmc: Rc<RefCell<FmcModel>>,
    halted: bool,
    /// Set by the `bkpt` hook.
    bkpt: Rc<Cell<bool>>,
    dcrdr: u32,
    /// The next instruction, while running.
    pc: u64,
}

fn core_reg(sel: u32) -> RegisterARM {
    match sel {
        0 => RegisterARM::R0,
        1 => RegisterARM::R1,
        2 => RegisterARM::R2,
        3 => RegisterARM::R3,
        13 => RegisterARM::SP,
        15 => RegisterARM::PC,
        16 => RegisterARM::XPSR,
        _ => panic!("the driver selected core register {sel}"),
    }
}

impl EmuPort {
    /// A halted core, flash erased, the FMC unlocked as the bridge leaves it before a run.
    fn new() -> Self {
        let mut emu = Unicorn::new(Arch::ARM, Mode::THUMB).expect("unicorn init");
        emu.ctl_set_cpu_model(ArmCpuModel::CORTEX_M3 as i32)
            .expect("cortex-m3");
        let fmc = attach(&mut emu, FLASH_SIZE, PAGE);
        emu.mem_map(RAM_BASE, RAM_SIZE, Prot::ALL).expect("map ram");
        let bkpt = Rc::new(Cell::new(false));
        let hit = bkpt.clone();
        emu.add_intr_hook(move |uc, _intno| {
            hit.set(true);
            let _ = uc.emu_stop();
        })
        .expect("bkpt hook");
        let mut p = EmuPort {
            emu,
            fmc,
            halted: true,
            bkpt,
            dcrdr: 0,
            pc: 0,
        };
        p.write_word(FMC_BASE as u32 + 4, 0x4567_0123).unwrap();
        p.write_word(FMC_BASE as u32 + 4, 0xCDEF_89AB).unwrap();
        p
    }

    /// Let the core run a slice, if it is running.
    fn tick(&mut self) {
        if self.halted {
            return;
        }
        let r = self.emu.emu_start(self.pc | 1, u64::MAX, 0, SLICE);
        if self.bkpt.get() {
            self.bkpt.set(false);
            self.halted = true;
            return;
        }
        r.expect("the loader runs without faulting");
        self.pc = self.emu.reg_read(RegisterARM::PC).unwrap();
    }

    fn flash(&self, addr: u32, len: usize) -> Vec<u8> {
        let mut v = vec![0u8; len];
        self.emu.mem_read(addr as u64, &mut v).unwrap();
        v
    }

    fn fmc_reg(addr: u32) -> Option<u64> {
        let a = addr as u64;
        (FMC_BASE..FMC_BASE + FMC_WINDOW)
            .contains(&a)
            .then(|| a - FMC_BASE)
    }
}

impl MemPort for EmuPort {
    type Error = String;

    fn read_word(&mut self, addr: u32) -> Result<u32, String> {
        let v = if addr == DHCSR {
            (self.halted as u32) << 17 | 1 << 16 | (self.halted as u32) << 1 | 1
        } else if addr == DCRDR {
            self.dcrdr
        } else if let Some(off) = Self::fmc_reg(addr) {
            self.fmc.borrow().read_reg(off)
        } else {
            let mut b = [0u8; 4];
            self.emu
                .mem_read(addr as u64, &mut b)
                .map_err(|e| format!("read {addr:#x}: {e:?}"))?;
            u32::from_le_bytes(b)
        };
        self.tick();
        Ok(v)
    }

    fn write_word(&mut self, addr: u32, v: u32) -> Result<(), String> {
        if addr == DHCSR {
            assert_eq!(v >> 16, 0xA05F, "DBGKEY");
            if v & 2 != 0 {
                self.halted = true;
            } else if self.halted {
                let xpsr = self.emu.reg_read(RegisterARM::XPSR).unwrap();
                assert_ne!(xpsr & 1 << 24, 0, "resumed without the Thumb bit");
                self.pc = self.emu.reg_read(RegisterARM::PC).unwrap() & !1;
                self.halted = false;
            }
        } else if addr == DCRSR {
            assert!(self.halted, "a core register transfer on a running core");
            let r = core_reg(v & 0x1F);
            if v & 1 << 16 != 0 {
                self.emu.reg_write(r, self.dcrdr as u64).unwrap();
            } else {
                self.dcrdr = self.emu.reg_read(r).unwrap() as u32;
            }
        } else if addr == DCRDR {
            self.dcrdr = v;
        } else if let Some(off) = Self::fmc_reg(addr) {
            self.fmc.borrow_mut().write_reg(off, v);
        } else if (RAM_BASE..RAM_BASE + RAM_SIZE).contains(&(addr as u64)) {
            self.emu.mem_write(addr as u64, &v.to_le_bytes()).unwrap();
        } else {
            return Err(format!("the driver wrote {addr:#x}"));
        }
        self.tick();
        Ok(())
    }

    fn write_words(&mut self, addr: u32, data: &[u32]) -> Result<(), String> {
        for (k, w) in data.iter().enumerate() {
            self.write_word(addr + 4 * k as u32, *w)?;
        }
        Ok(())
    }
}

/// `FMC_STAT`'s error bits: `PGERR`, `WPERR`.
const STAT_ERR: u32 = 1 << 2 | 1 << 4;

/// One run of `data` at `addr`, driven to the halt: the run and what the loader left in `r0`, `r2`.
fn run(p: &mut EmuPort, addr: u32, data: &[u8]) -> (Run, Outcome) {
    let mut run = Run::start(p, addr, data.len() as u32).expect("start");
    for _ in 0..1_000_000 {
        if let Poll::Done(o) = run.poll(p, data).expect("poll") {
            return (run, o);
        }
    }
    panic!("the loader never halted");
}

fn image(len: usize) -> Vec<u8> {
    let mut v: Vec<u8> = (0..len as u32).map(|i| (i * 7 + (i >> 8)) as u8).collect();
    v[200..210].fill(0xFF); // erased-value halfwords, which the loader leaves alone
    v
}

#[test]
fn the_loader_programs_an_image_across_three_pages_and_it_reads_back() {
    let mut p = EmuPort::new();
    loader::load(&mut p).unwrap();
    let base = FLASH_BASE as u32 + PAGE;
    // Three pages and part of a fourth, then a run that goes round the 4 KiB ring.
    for (addr, len) in [(base, 3 * 1024 + 301), (base + 4 * PAGE, 9 * 1024 + 2)] {
        let img = image(len);
        let (r, o) = run(&mut p, addr, &img);
        assert_eq!(o.stat & STAT_ERR, 0, "{o:?}");
        assert_eq!(o.addr, r.end(), "r2 is the address after the last halfword");
        let mut want = img.clone();
        if len % 2 == 1 {
            want.push(0xFF);
        }
        assert_eq!(p.flash(addr, want.len()), want);
    }
    assert!(p.halted);
    assert!(!p.fmc.borrow().programming(), "the loader cleared PG");
}

#[test]
fn a_reprogram_under_the_loader_ends_in_pgerr_at_the_failing_halfword() {
    let mut p = EmuPort::new();
    loader::load(&mut p).unwrap();
    let base = FLASH_BASE as u32 + PAGE;
    // A high halfword: one already programmed at a low halfword would instead close the halfword
    // below it, and the run would stop there (the F1x0 rule the FMC model carries).
    let bad = base + 2 * PAGE + 0x42;
    let (r, o) = run(&mut p, bad, &[0x34, 0x12]);
    assert_eq!((o.stat & STAT_ERR, o.addr), (0, r.end()));
    let img = image(3 * 1024);
    let (_, o) = run(&mut p, base, &img);
    assert_eq!(o.stat & STAT_ERR, 1 << 2, "PGERR: {o:?}");
    assert_eq!(o.addr, bad, "r2 is the failing halfword");
    assert_eq!(
        p.read_word(loader::RP).unwrap(),
        0,
        "rp = 0 on the error exit"
    );
    let o = (bad - base) as usize;
    assert_eq!(p.flash(base, o), img[..o], "everything below it programmed");
    assert_eq!(
        p.flash(bad, 2),
        [0x34, 0x12],
        "the cell kept its first value"
    );
    assert!(
        p.flash(bad + 2, 64).iter().all(|&b| b == 0xFF),
        "nothing above it"
    );
}
