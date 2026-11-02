// SPDX-License-Identifier: GPL-3.0-or-later
//! Run one entry function under Unicorn and record its peripheral accesses.
//!
//! The image's loadable segments are written at their run addresses (so initialised data is in RAM
//! as if the startup code had copied it; no startup code runs), the family's reset seeds and the
//! vector's overrides are written into the peripheral space, and the CPU enters the entry function
//! with LR pointing at an unmapped-code sentinel. The run ends when the function returns there.
//!
//! Only the peripheral windows are hooked (APB/AHB at `0x4000_0000..=0x5FFF_FFFF` and the Cortex-M
//! private bus at `0xE000_0000..=0xE00F_FFFF`), never flash or RAM. Each load or store there is
//! recorded with its width, and a load from an address with a scripted response sees that value.

use std::collections::HashMap;

use object::elf::PT_LOAD;
use object::read::elf::{ElfFile32, ProgramHeader};
use object::{LittleEndian, Object, ObjectSymbol, SymbolKind};
use unicorn_engine::{Arch, ArmCpuModel, HookType, MemType, Mode, Prot, RegisterARM, Unicorn};

use crate::target::Target;
use crate::trace::{Access, Op};
use crate::vector::Response;

/// The code and data windows the image may occupy.
const FLASH: (u64, u64) = (0x0800_0000, 512 * 1024);
const RAM: (u64, u64) = (0x2000_0000, 64 * 1024);
/// The peripheral windows mapped as plain memory (reads see what was last written or seeded).
const MMIO_MAPS: [(u64, u64); 4] = [
    (0x4000_0000, 0x3_0000),
    (0x4800_0000, 0x1_0000),
    (0xE000_0000, 0x10_0000),
    // The system-memory page with the factory words (the unique ID, the flash size, the MM32's HSI
    // trim at 0x1FFF_F7FA): read by the clock and identity code, seeded by a family table, never
    // recorded (not a peripheral).
    (0x1FFF_F000, 0x1000),
];
/// The windows whose accesses are recorded.
const HOOKED: [(u64, u64); 2] = [(0x4000_0000, 0x5FFF_FFFF), (0xE000_0000, 0xE00F_FFFF)];
/// The return address the entry function is called with. The run stops when the PC reaches it.
const SENTINEL: u64 = 0x1000_0000;
/// The instruction budget for one run: far above any vector's straight-line work, so reaching it
/// means a poll that never saw its flag.
pub const STEP_CAP: usize = 2_000_000;

/// The code to run: loadable segments and named functions.
#[derive(Debug, Clone)]
pub struct Image {
    /// `(run address, bytes)`; any zero fill beyond the file bytes is included.
    pub segments: Vec<(u32, Vec<u8>)>,
    /// Function symbols: `(start, size, name)`, the Thumb bit cleared.
    pub functions: Vec<(u32, u32, String)>,
}

impl Image {
    /// Read an ELF32 little-endian image.
    pub fn from_elf(bytes: &[u8]) -> Result<Image, String> {
        let elf = ElfFile32::<LittleEndian>::parse(bytes).map_err(|e| e.to_string())?;
        let endian = LittleEndian;
        let mut segments = Vec::new();
        for ph in elf.elf_program_headers() {
            if ph.p_type(endian) != PT_LOAD || ph.p_memsz(endian) == 0 {
                continue;
            }
            let mut data = ph
                .data(endian, bytes)
                .map_err(|_| "bad segment".to_string())?
                .to_vec();
            data.resize(ph.p_memsz(endian) as usize, 0);
            segments.push((ph.p_vaddr(endian), data));
        }
        let functions = elf
            .symbols()
            .filter(|s| s.kind() == SymbolKind::Text)
            .filter_map(|s| {
                let name = s.name().ok()?.to_string();
                Some(((s.address() as u32) & !1, s.size() as u32, name))
            })
            .collect();
        Ok(Image {
            segments,
            functions,
        })
    }

    /// The address of function `name`.
    pub fn function(&self, name: &str) -> Result<u32, String> {
        self.functions
            .iter()
            .find(|(_, _, n)| n == name)
            .map(|(a, _, _)| *a)
            .ok_or_else(|| format!("no function {name} in the image"))
    }

    /// The function containing `pc`, for a diagnostic.
    pub fn function_at(&self, pc: u32) -> Option<&str> {
        self.functions
            .iter()
            .find(|(a, s, _)| pc >= *a && pc < a + (*s).max(1))
            .map(|(_, _, n)| n.as_str())
    }
}

/// How a run ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Exit {
    /// The entry function returned.
    Returned,
    /// The instruction budget ran out at this PC (a poll that never exits).
    StepCap(u32),
    /// The emulator stopped on an error at this PC.
    Fault(u32, String),
}

/// A run's accesses and how it ended.
#[derive(Debug, Clone)]
pub struct Run {
    /// The peripheral accesses, in order.
    pub accesses: Vec<Access>,
    /// How the run ended.
    pub exit: Exit,
}

impl Run {
    /// An error naming where the run stopped, unless it returned.
    pub fn check_returned(&self, image: &Image) -> Result<(), String> {
        let at = |pc: u32| match image.function_at(pc) {
            Some(f) => format!("pc 0x{pc:08X} in {f}"),
            None => format!("pc 0x{pc:08X}"),
        };
        match &self.exit {
            Exit::Returned => Ok(()),
            Exit::StepCap(pc) => Err(format!(
                "did not return within {STEP_CAP} instructions ({}): a poll whose flag no read \
                 response sets, or a panic",
                at(*pc)
            )),
            Exit::Fault(pc, e) => Err(format!("emulation stopped: {e} ({})", at(*pc))),
        }
    }
}

enum Script {
    Value(u32),
    Sequence(Vec<u32>, usize),
    Or(u32),
}

struct State {
    accesses: Vec<Access>,
    scripts: HashMap<u32, Script>,
    reads_zero: HashMap<u32, u32>,
}

fn mask(size: usize) -> u32 {
    if size >= 4 {
        u32::MAX
    } else {
        (1u32 << (size * 8)) - 1
    }
}

/// Run `entry` of `image` with the family's reset seeds, the vector's overrides and its scripted
/// reads.
pub fn run(
    image: &Image,
    entry: &str,
    target: &Target,
    responses: &[(u32, Response)],
    overrides: &[(u32, u32)],
) -> Result<Run, String> {
    let entry_addr = image.function(entry)?;
    let scripts = responses
        .iter()
        .map(|(addr, r)| {
            let s = match r {
                Response::Value(v) => Script::Value(*v),
                Response::Sequence(v) => Script::Sequence(v.clone(), 0),
                Response::Or(m) => Script::Or(*m),
            };
            (*addr, s)
        })
        .collect();
    let reads_zero = target
        .blocks
        .iter()
        .flat_map(|b| {
            b.write_only
                .keys()
                .chain(b.self_clearing.keys())
                .map(|off| (b.base + off, target.reads_zero(b.base + off)))
        })
        .collect();
    let state = State {
        accesses: Vec::new(),
        scripts,
        reads_zero,
    };
    fn e(what: &'static str) -> impl Fn(unicorn_engine::uc_error) -> String {
        move |err| format!("unicorn: {what}: {err:?}")
    }

    let mut uc = Unicorn::new_with_data(Arch::ARM, Mode::THUMB, state).map_err(e("open"))?;
    // The M-profile core: the default A-profile one refuses the system instructions (mrs/cpsid) the
    // Rust side's critical sections use. A Cortex-M0 family runs the M0 model, so a Thumb-2
    // instruction in either image faults instead of running.
    let model = if target.family.cortex_m0() {
        ArmCpuModel::CORTEX_M0
    } else {
        ArmCpuModel::CORTEX_M3
    };
    uc.ctl_set_cpu_model(model as i32).map_err(e("cpu model"))?;
    for (base, size) in [FLASH, RAM, (SENTINEL, 0x1000)]
        .into_iter()
        .chain(MMIO_MAPS)
    {
        uc.mem_map(base, size, Prot::ALL).map_err(e("map"))?;
    }

    for (addr, data) in &image.segments {
        uc.mem_write(u64::from(*addr), data).map_err(|err| {
            format!("segment at 0x{addr:08X} is outside the mapped windows: {err:?}")
        })?;
    }
    for block in &target.blocks {
        for (off, value) in &block.reset {
            uc.mem_write(u64::from(block.base + off), &value.to_le_bytes())
                .map_err(e("seed"))?;
        }
    }
    for (addr, value) in overrides {
        uc.mem_write(u64::from(*addr), &value.to_le_bytes())
            .map_err(e("override"))?;
    }

    for (lo, hi) in HOOKED {
        uc.add_mem_hook(
            HookType::MEM_WRITE,
            lo,
            hi,
            |uc: &mut Unicorn<State>, _: MemType, addr, size, value| {
                uc.get_data_mut().accesses.push(Access {
                    op: Op::Write,
                    size: size as u8,
                    addr: addr as u32,
                    value: (value as u32) & mask(size),
                });
                true
            },
        )
        .map_err(e("write hook"))?;
        uc.add_mem_hook(
            HookType::MEM_READ,
            lo,
            hi,
            |uc: &mut Unicorn<State>, _: MemType, addr, size, _| {
                let a = addr as u32;
                let mut word = [0u8; 4];
                let _ = uc.mem_read(addr, &mut word);
                let mut current = u32::from_le_bytes(word);
                let data = uc.get_data_mut();
                let reads_zero = data.reads_zero.get(&a).copied();
                let scripted = match data.scripts.get_mut(&a) {
                    None => None,
                    Some(Script::Value(v)) => Some(*v),
                    Some(Script::Or(m)) => Some(current | *m),
                    Some(Script::Sequence(values, next)) => {
                        let v = values[(*next).min(values.len() - 1)];
                        *next += 1;
                        Some(v)
                    }
                };
                // The hook runs before the load, so the value written here is what it reads. An OR
                // response puts its bits in memory, where a following read-modify-write writes them
                // back, as it would write back a hardware status bit.
                if let Some(m) = reads_zero {
                    current &= !m;
                }
                let value = match scripted {
                    Some(v) => v,
                    None => current,
                };
                if scripted.is_some() || reads_zero.is_some() {
                    let _ = uc.mem_write(addr, &value.to_le_bytes());
                }
                uc.get_data_mut().accesses.push(Access {
                    op: Op::Read,
                    size: size as u8,
                    addr: a,
                    value: value & mask(size),
                });
                true
            },
        )
        .map_err(e("read hook"))?;
    }

    uc.reg_write(RegisterARM::SP, RAM.0 + RAM.1)
        .map_err(e("sp"))?;
    uc.reg_write(RegisterARM::LR, SENTINEL | 1)
        .map_err(e("lr"))?;
    let result = uc.emu_start(u64::from(entry_addr) | 1, SENTINEL, 0, STEP_CAP);
    let pc = uc.reg_read(RegisterARM::PC).map_err(e("pc"))? as u32;
    let exit = match result {
        Ok(()) if u64::from(pc) == SENTINEL => Exit::Returned,
        Ok(()) => Exit::StepCap(pc),
        Err(err) => Exit::Fault(pc, format!("{err:?}")),
    };
    let accesses = std::mem::take(&mut uc.get_data_mut().accesses);
    Ok(Run { accesses, exit })
}

#[cfg(test)]
mod tests {
    //! The emulator on hand-encoded Thumb code, so the recorder is checked without a compiler.

    use super::*;
    use crate::target::Family;

    fn target() -> Target {
        Target::parse(
            Family::Gd32f1x0,
            "[blocks.RCU_BASE]\nbase = 0x4002_1000\nsize = 0x400\nreset = { \"0x14\" = 0x14 }\n\
             write_only = { \"0x18\" = 0xF000 }\nself_clearing = { \"0x18\" = 0x0F00 }\n",
        )
        .unwrap()
    }

    /// `MOVW`/`MOVT rd, imm16` (Thumb-2 T3 / T1).
    fn mov16(top: bool, rd: u16, imm: u16) -> [u16; 2] {
        let op = if top { 0xF2C0 } else { 0xF240 };
        let i = (imm >> 11) & 1;
        let imm4 = imm >> 12;
        let imm3 = (imm >> 8) & 7;
        let imm8 = imm & 0xFF;
        [op | (i << 10) | imm4, (imm3 << 12) | (rd << 8) | imm8]
    }

    fn load32(rd: u16, value: u32) -> Vec<u16> {
        let mut v = mov16(false, rd, value as u16).to_vec();
        v.extend(mov16(true, rd, (value >> 16) as u16));
        v
    }

    fn image(code: &[u16]) -> Image {
        let bytes = code.iter().flat_map(|h| h.to_le_bytes()).collect();
        Image {
            segments: vec![(0x0800_0000, bytes)],
            functions: vec![(0x0800_0000, (code.len() * 2) as u32, "entry".into())],
        }
    }

    const BX_LR: u16 = 0x4770;

    #[test]
    fn widths_values_and_scripted_reads_are_recorded() {
        let mut code = load32(0, 0x4002_1000);
        code.extend(load32(1, 0xDEAD_BEEF));
        code.extend([
            0x6181, // str  r1, [r0, #0x18]
            0x8381, // strh r1, [r0, #0x1C]
            0x7401, // strb r1, [r0, #0x10]
            0x6802, // ldr  r2, [r0]        (an OR response over the reset seed)
            0x6942, // ldr  r2, [r0, #0x14] (the seed)
            0x6982, // ldr  r2, [r0, #0x18] (bits 15:8, write-only and self-clearing, read as 0)
            BX_LR,
        ]);
        let t = target();
        let run = run(
            &image(&code),
            "entry",
            &t,
            &[(0x4002_1000, Response::Or(0x2))],
            &[(0x4002_1000, 0x1)],
        )
        .unwrap();
        assert_eq!(run.exit, Exit::Returned);
        let w = |size, addr, value| Access {
            op: Op::Write,
            size,
            addr,
            value,
        };
        let r = |addr, value| Access {
            op: Op::Read,
            size: 4,
            addr,
            value,
        };
        assert_eq!(
            run.accesses,
            vec![
                w(4, 0x4002_1018, 0xDEAD_BEEF),
                w(2, 0x4002_101C, 0xBEEF),
                w(1, 0x4002_1010, 0xEF),
                r(0x4002_1000, 0x3),
                r(0x4002_1014, 0x14),
                r(0x4002_1018, 0xDEAD_00EF),
            ]
        );
    }

    /// `loop: ldr r2, [r0]; cmp r2, #3; bne loop; bx lr`.
    fn poll_loop() -> Vec<u16> {
        let mut code = load32(0, 0x4002_1000);
        code.extend([0x6802, 0x2A03, 0xD1FC, BX_LR]);
        code
    }

    #[test]
    fn a_sequence_response_ends_a_poll() {
        let t = target();
        let seq = Response::Sequence(vec![0, 1, 3]);
        let run = run(
            &image(&poll_loop()),
            "entry",
            &t,
            &[(0x4002_1000, seq)],
            &[],
        )
        .unwrap();
        assert_eq!(run.exit, Exit::Returned);
        let values: Vec<u32> = run.accesses.iter().map(|a| a.value).collect();
        assert_eq!(values, vec![0, 1, 3]);
    }

    #[test]
    fn a_poll_nothing_answers_hits_the_step_cap() {
        let t = target();
        let img = image(&poll_loop());
        let run = run(&img, "entry", &t, &[], &[]).unwrap();
        assert!(matches!(run.exit, Exit::StepCap(_)), "{:?}", run.exit);
        let err = run.check_returned(&img).unwrap_err();
        assert!(err.contains("in entry"), "{err}");
    }
}
