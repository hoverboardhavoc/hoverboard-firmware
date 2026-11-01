//! The checks `tools/flash.sh` runs on an image before it is flashed, the ELF-to-binary conversion
//! (`specs/hoverboard-bridge.md`, "From the terminal tool"), and the debug strip a bundle stores its
//! ELFs with (`specs/firmware-bundle.md` section 3).
//!
//! - **The `wfi` scan.** A bare `wfi` with `DBG_CTL0 = 0` locks GD32 SWD re-attach. The Thumb code
//!   regions (the `$t` mapping symbols) are decoded instruction by instruction, so data in the text
//!   section cannot pass for one and a 32-bit instruction's second half is never read as the first.
//! - **The image's guards** ([`Guards`]), from `tools/image-guard-syms.sh`, keyed by image name, the
//!   one copy CI and `flash.sh` read; a bundle carries each image's copy in its manifest:
//!   - the code floor: `.text` plus `.hotcode` of at least `code_floor` bytes (an LTO-gutted image
//!     links and flashes and does nothing);
//!   - the required symbols, matched against `nm`-style lines as `grep -E` matches them there;
//!   - the hot-window symbols: the first `nm` line (in `nm`'s name order) each matches must sit
//!     below [`HOT_LIMIT`].
//! - **The layout check** ([`to_bin`]): loadable bytes, all inside flash.
//!
//! An ELF given directly gets the `wfi` scan and the layout check only ([`check_wfi`], [`to_bin`]):
//! it does not say which image it is, so there are no guards to apply.

use object::elf::{SHF_ALLOC, SHF_EXECINSTR, SHF_WRITE};
use object::{Object, ObjectSection, ObjectSymbol, SectionFlags, SectionKind};
use regex::Regex;
use serde::{Deserialize, Serialize};

/// Where the image is linked (there is no bootloader).
pub const LINK_BASE: u64 = 0x0800_0000;
/// The F1x0 zero-wait flash boundary the hot-window symbols must sit below.
pub const HOT_LIMIT: u64 = 0x0800_8000;

/// An image's release guards (`tools/image-guard-syms.sh`, and a bundle manifest's
/// `images[].guards`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Guards {
    /// `.text` + `.hotcode` must be at least this many bytes.
    pub code_floor: u64,
    /// `grep -E` patterns that must each match an `nm` line.
    pub required_symbols: Vec<String>,
    /// `grep -E` patterns whose first `nm` match must sit below [`HOT_LIMIT`]; absent for an image
    /// with no hot-window gate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hot_symbols: Option<Vec<String>>,
}

/// The guards `tools/image-guard-syms.sh` (its text in `file`) gives `image`: the `<image>)` arm of
/// its `image_guards` case, up to the arm's `;;`.
pub fn guards_for(file: &str, image: &str) -> Result<Guards, String> {
    let head = format!("{image})");
    let mut lines = file.lines().map(str::trim);
    if !lines.any(|l| l == head) {
        return Err(format!(
            "tools/image-guard-syms.sh has no guards for image {image}"
        ));
    }
    let (mut floor, mut req, mut hot) = (None, None, None);
    for l in lines {
        if l == ";;" {
            break;
        }
        let Some((key, value)) = l
            .strip_prefix("IMAGE_GUARD_")
            .and_then(|r| r.split_once('='))
        else {
            continue;
        };
        let value = value.trim_end_matches(";;").trim_end();
        let value = value
            .strip_prefix('\'')
            .and_then(|v| v.strip_suffix('\''))
            .unwrap_or(value);
        let list = || -> Vec<String> {
            value
                .split(',')
                .filter(|p| !p.is_empty())
                .map(str::to_string)
                .collect()
        };
        match key {
            "FLOOR" => {
                floor = Some(value.parse::<u64>().map_err(|_| {
                    format!("tools/image-guard-syms.sh: {image}: IMAGE_GUARD_FLOOR {value:?} is not a number")
                })?)
            }
            "REQ_SYMS" => req = Some(list()),
            "HOT_SYMS" => hot = Some(list()),
            _ => {
                return Err(format!(
                    "tools/image-guard-syms.sh: {image}: unknown key IMAGE_GUARD_{key}"
                ))
            }
        }
    }
    let missing = |k: &str| format!("tools/image-guard-syms.sh: {image}: no IMAGE_GUARD_{k}");
    Ok(Guards {
        code_floor: floor.ok_or_else(|| missing("FLOOR"))?,
        required_symbols: req.ok_or_else(|| missing("REQ_SYMS"))?,
        hot_symbols: hot.filter(|h| !h.is_empty()),
    })
}

/// Why an image is refused.
#[derive(Debug, PartialEq, Eq)]
pub enum ElfError {
    /// Not an ELF this tool reads.
    Parse(String),
    /// A `wfi` at this address.
    Wfi(u64),
    /// `.text` + `.hotcode` (the first) below the floor (the second).
    CodeFloor(u64, u64),
    /// Required symbols missing.
    MissingSymbols(Vec<String>),
    /// Hot-window patterns that match no symbol.
    HotMissing(Vec<String>),
    /// Hot-window patterns whose first match sits at or above [`HOT_LIMIT`], with that address.
    HotCold(Vec<(String, u64)>),
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
            ElfError::CodeFloor(n, floor) => write!(
                f,
                "REFUSED: release code (.text + .hotcode) is {n} B, below the {floor} B floor \
                 (an LTO-gutted image?)"
            ),
            ElfError::MissingSymbols(m) => write!(
                f,
                "REFUSED: required symbol(s) absent from the image (LTO-gutted?): {}",
                m.join(" ")
            ),
            ElfError::HotMissing(m) => write!(
                f,
                "REFUSED: hot-window symbol(s) no longer resolve (inlined or renamed?): {}",
                m.join(" ")
            ),
            ElfError::HotCold(c) => {
                let c: Vec<String> = c.iter().map(|(p, a)| format!("{p}@{a:#010x}")).collect();
                write!(
                    f,
                    "REFUSED: hot-path symbol(s) above the {HOT_LIMIT:#010x} zero-wait boundary: {}",
                    c.join(" ")
                )
            }
            ElfError::Layout(e) => write!(f, "REFUSED: {e}"),
        }
    }
}

fn parse(elf: &[u8]) -> Result<object::File<'_>, ElfError> {
    object::File::parse(elf).map_err(|e| ElfError::Parse(e.to_string()))
}

fn compile(p: &str) -> Result<Regex, ElfError> {
    Regex::new(p).map_err(|e| ElfError::Parse(format!("guard pattern {p:?}: {e}")))
}

/// The `wfi` scan alone: what an ELF given directly gets, with [`to_bin`]'s layout check.
pub fn check_wfi(elf: &[u8]) -> Result<(), ElfError> {
    match find_wfi(&parse(elf)?) {
        Some(a) => Err(ElfError::Wfi(a)),
        None => Ok(()),
    }
}

/// The `wfi` scan and the image's guards.
pub fn check(elf: &[u8], guards: &Guards) -> Result<(), ElfError> {
    let obj = parse(elf)?;
    if let Some(a) = find_wfi(&obj) {
        return Err(ElfError::Wfi(a));
    }
    let code: u64 = obj
        .sections()
        .filter(|s| matches!(s.name(), Ok(".text") | Ok(".hotcode")))
        .map(|s| s.size())
        .sum();
    if code < guards.code_floor {
        return Err(ElfError::CodeFloor(code, guards.code_floor));
    }
    let lines = nm_lines(&obj);
    let mut missing = Vec::new();
    for p in &guards.required_symbols {
        let re = compile(p)?;
        if !lines.iter().any(|(_, l)| re.is_match(l)) {
            missing.push(p.clone());
        }
    }
    if !missing.is_empty() {
        return Err(ElfError::MissingSymbols(missing));
    }
    let (mut gone, mut cold) = (Vec::new(), Vec::new());
    for p in guards.hot_symbols.iter().flatten() {
        let re = compile(p)?;
        match lines.iter().find(|(_, l)| re.is_match(l)) {
            None => gone.push(p.clone()),
            Some(&(a, _)) if a >= HOT_LIMIT => cold.push((p.clone(), a)),
            Some(_) => {}
        }
    }
    if !gone.is_empty() {
        return Err(ElfError::HotMissing(gone));
    }
    if !cold.is_empty() {
        return Err(ElfError::HotCold(cold));
    }
    Ok(())
}

/// The ELF without its debug sections (what `objcopy --strip-debug` removes: every `.debug_*`
/// section), the symbol table kept, because the guards read symbols.
pub fn strip_debug(elf: &[u8]) -> Result<Vec<u8>, ElfError> {
    let mut b =
        object::build::elf::Builder::read(elf).map_err(|e| ElfError::Parse(e.to_string()))?;
    for s in b.sections.iter_mut() {
        if s.name.starts_with(b".debug_") {
            s.delete = true;
        }
    }
    let mut out = Vec::new();
    b.write(&mut out)
        .map_err(|e| ElfError::Parse(format!("could not rewrite the ELF: {e}")))?;
    Ok(out)
}

/// `nm`-style lines (`addr type name`) for the symbol table, with each address, in `nm`'s default
/// order (by name), typed as `nm` types them for these images: `T` code, `B` zero-initialised data,
/// `D` data, `R` read-only data; lower case when local.
fn nm_lines(obj: &object::File) -> Vec<(u64, String)> {
    let mut v: Vec<(u64, String, String)> = obj
        .symbols()
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
            Some((
                s.address(),
                name.to_string(),
                format!("{:08x} {t} {name}", s.address()),
            ))
        })
        .collect();
    v.sort_by(|a, b| a.1.cmp(&b.1));
    v.into_iter().map(|(a, _, l)| (a, l)).collect()
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
