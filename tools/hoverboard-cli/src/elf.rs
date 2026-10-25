//! The checks `tools/flash.sh` runs on an image before it is flashed, and the ELF-to-binary
//! conversion (`specs/hoverboard-bridge.md`, "From the terminal tool").
//!
//! - **The `wfi` scan.** A bare `wfi` with `DBG_CTL0 = 0` locks GD32 SWD re-attach. The Thumb code
//!   regions (the `$t` mapping symbols) are decoded instruction by instruction, so data in the text
//!   section cannot pass for one and a 32-bit instruction's second half is never read as the first.
//! - **The code floor.** `.text` plus `.hotcode` of at least [`CODE_FLOOR`] bytes (an LTO-gutted
//!   image links and flashes and does nothing).
//! - **The required symbols.** `IMAGE_GUARD_REQ_SYMS` from `tools/image-guard-syms.sh`, the one copy
//!   CI and `flash.sh` read, matched against `nm`-style lines as `grep -E` matches them there.

use object::elf::{SHF_ALLOC, SHF_EXECINSTR, SHF_WRITE};
use object::{Object, ObjectSection, ObjectSymbol, SectionFlags, SectionKind};
use regex::Regex;

/// The release code floor (`flash.sh`'s integrated profile, CI's guard).
pub const CODE_FLOOR: u64 = 40_000;
/// Where the image is linked (there is no bootloader).
pub const LINK_BASE: u64 = 0x0800_0000;

const GUARD_SYMS: &str = include_str!("../../image-guard-syms.sh");

/// The required-symbol patterns, as `tools/image-guard-syms.sh` lists them.
pub fn required_patterns() -> Vec<String> {
    GUARD_SYMS
        .lines()
        .find_map(|l| l.strip_prefix("IMAGE_GUARD_REQ_SYMS='"))
        .and_then(|l| l.strip_suffix('\''))
        .map(|l| l.split(',').map(str::to_string).collect())
        .unwrap_or_default()
}

/// Why an image is refused.
#[derive(Debug, PartialEq, Eq)]
pub enum ElfError {
    /// Not an ELF this tool reads.
    Parse(String),
    /// A `wfi` at this address.
    Wfi(u64),
    /// `.text` + `.hotcode` below the floor.
    CodeFloor(u64),
    /// Required symbols missing.
    MissingSymbols(Vec<String>),
    /// No loadable bytes, or bytes outside flash.
    Layout(String),
}

impl std::fmt::Display for ElfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ElfError::Parse(e) => write!(f, "not a readable ELF: {e}"),
            ElfError::Wfi(a) => write!(
                f,
                "REFUSED: the image contains a 'wfi' instruction at {a:#010x}; a bare wfi with \
                 DBG_CTL0=0 locks GD32 SWD re-attach"
            ),
            ElfError::CodeFloor(n) => write!(
                f,
                "REFUSED: release code (.text + .hotcode) is {n} B, below the {CODE_FLOOR} B floor \
                 (an LTO-gutted image?)"
            ),
            ElfError::MissingSymbols(m) => write!(
                f,
                "REFUSED: required symbol(s) absent from the image (LTO-gutted?): {}",
                m.join(" ")
            ),
            ElfError::Layout(e) => write!(f, "REFUSED: {e}"),
        }
    }
}

/// Run the three checks.
pub fn check(elf: &[u8]) -> Result<(), ElfError> {
    let obj = object::File::parse(elf).map_err(|e| ElfError::Parse(e.to_string()))?;
    if let Some(a) = find_wfi(&obj) {
        return Err(ElfError::Wfi(a));
    }
    let code: u64 = obj
        .sections()
        .filter(|s| matches!(s.name(), Ok(".text") | Ok(".hotcode")))
        .map(|s| s.size())
        .sum();
    if code < CODE_FLOOR {
        return Err(ElfError::CodeFloor(code));
    }
    let lines = nm_lines(&obj);
    let missing: Vec<String> = required_patterns()
        .into_iter()
        .filter(|p| {
            let re = Regex::new(p).expect("a guard pattern is a valid regex");
            !lines.iter().any(|l| re.is_match(l))
        })
        .collect();
    if !missing.is_empty() {
        return Err(ElfError::MissingSymbols(missing));
    }
    Ok(())
}

/// `nm`-style lines (`addr type name`) for the symbol table, typed as `nm` types them for these
/// images: `T` code, `B` zero-initialised data, `D` data, `R` read-only data; lower case when local.
fn nm_lines(obj: &object::File) -> Vec<String> {
    obj.symbols()
        .filter_map(|s| {
            let name = s.name().ok()?;
            if name.is_empty() || name.starts_with('$') {
                return None;
            }
            let sec = obj.section_by_index(s.section_index()?).ok()?;
            let flags = match sec.flags() {
                SectionFlags::Elf { sh_flags } => sh_flags,
                _ => 0,
            };
            let mut t = if flags & u64::from(SHF_EXECINSTR) != 0 {
                'T'
            } else if sec.kind() == SectionKind::UninitializedData {
                'B'
            } else if flags & u64::from(SHF_WRITE) != 0 {
                'D'
            } else if flags & u64::from(SHF_ALLOC) != 0 {
                'R'
            } else {
                return None;
            };
            if !s.is_global() {
                t = t.to_ascii_lowercase();
            }
            Some(format!("{:08x} {t} {name}", s.address()))
        })
        .collect()
}

/// The first `wfi` (16-bit `0xBF30`, or 32-bit `0xF3AF 0x8003`) in a Thumb code region.
fn find_wfi(obj: &object::File) -> Option<u64> {
    for sec in obj.sections() {
        let SectionFlags::Elf { sh_flags } = sec.flags() else {
            continue;
        };
        if sh_flags & u64::from(SHF_EXECINSTR) == 0 {
            continue;
        }
        let Ok(data) = sec.data() else { continue };
        let (start, end) = (sec.address(), sec.address() + sec.size());
        // Mapping symbols mark where code ($t / $a) and data ($d) begin inside the section.
        let mut marks: Vec<(u64, bool)> = obj
            .symbols()
            .filter(|s| s.section_index() == Some(sec.index()))
            .filter_map(|s| {
                let n = s.name().ok()?;
                let code = n == "$t" || n.starts_with("$t.");
                let dat = n == "$d" || n.starts_with("$d.") || n == "$a" || n.starts_with("$a.");
                (code || dat).then_some((s.address(), code))
            })
            .collect();
        if marks.is_empty() {
            marks.push((start, true)); // no mapping symbols: all of it is code
        }
        marks.sort();
        for (i, &(from, code)) in marks.iter().enumerate() {
            if !code {
                continue;
            }
            let to = marks.get(i + 1).map_or(end, |m| m.0);
            let mut a = from;
            while a + 2 <= to {
                let o = (a - start) as usize;
                let hw = u16::from_le_bytes([data[o], data[o + 1]]);
                if hw >> 11 >= 0b11101 {
                    // The first half of a 32-bit instruction.
                    if a + 4 <= to {
                        let hw2 = u16::from_le_bytes([data[o + 2], data[o + 3]]);
                        if hw == 0xF3AF && hw2 == 0x8003 {
                            return Some(a);
                        }
                    }
                    a += 4;
                } else {
                    if hw == 0xBF30 {
                        return Some(a);
                    }
                    a += 2;
                }
            }
        }
    }
    None
}

/// The flash image: every loadable segment's file bytes at its load (physical) address, from
/// [`LINK_BASE`], gaps filled with `0xFF` (what `objcopy -O binary` produces for these images).
pub fn to_bin(elf: &[u8]) -> Result<Vec<u8>, ElfError> {
    use object::elf::{FileHeader32, PT_LOAD};
    use object::read::elf::{FileHeader, ProgramHeader};
    use object::LittleEndian;
    let hdr =
        FileHeader32::<LittleEndian>::parse(elf).map_err(|e| ElfError::Parse(e.to_string()))?;
    let e = hdr.endian().map_err(|e| ElfError::Parse(e.to_string()))?;
    let phs = hdr
        .program_headers(e, elf)
        .map_err(|e| ElfError::Parse(e.to_string()))?;
    let mut out: Vec<u8> = Vec::new();
    for ph in phs {
        if ph.p_type(e) != PT_LOAD || ph.p_filesz(e) == 0 {
            continue;
        }
        let data = ph
            .data(e, elf)
            .map_err(|_| ElfError::Parse("a segment's bytes are outside the file".into()))?;
        let addr = u64::from(ph.p_paddr(e));
        if addr < LINK_BASE || addr - LINK_BASE + data.len() as u64 > 0x10_0000 {
            return Err(ElfError::Layout(format!(
                "a loadable segment at {addr:#x} is outside flash"
            )));
        }
        let off = (addr - LINK_BASE) as usize;
        if out.len() < off + data.len() {
            out.resize(off + data.len(), 0xFF);
        }
        out[off..off + data.len()].copy_from_slice(data);
    }
    if out.is_empty() {
        return Err(ElfError::Layout("no loadable bytes".into()));
    }
    Ok(out)
}
