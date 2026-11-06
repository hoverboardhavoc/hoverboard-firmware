//! What more than one test file builds on. Each test binary uses part of it.
#![allow(dead_code)]

use hoverboard_cli::elf::{self, Guards};

/// `tools/image-guard-syms.sh`, the one copy of the images' guards.
pub const GUARD_FILE: &str = include_str!("../../../image-guard-syms.sh");
/// Where the images are linked.
pub const BASE: u32 = 0x0800_0000;

/// The GD32F130C8 mainboard image's guards, as the packer reads them.
pub fn release_guards() -> Guards {
    elf::guards_for(GUARD_FILE, "mainboard-gd32f130c8").unwrap()
}

/// Guard patterns as names an image would carry: text symbols with the patterns' text (anchors
/// removed), and the `B` ones, which go in .bss. A name on both lists is given once.
pub fn names_for(patterns: &[String]) -> (Vec<String>, Vec<String>) {
    let mut text = Vec::new();
    let mut bss = Vec::new();
    for p in patterns {
        let name = p.trim_end_matches('$').to_string();
        let (list, n) = if let Some(b) = name.strip_prefix("B ") {
            (&mut bss, b.to_string())
        } else if let Some(t) = name.strip_prefix("T ") {
            (&mut text, t.to_string())
        } else {
            (&mut text, format!("_ZN{name}17h0123456789abcdefE"))
        };
        if !list.contains(&n) {
            list.push(n);
        }
    }
    (text, bss)
}

/// Every name `g` asks for, required and hot.
pub fn names_of(g: &Guards) -> (Vec<String>, Vec<String>) {
    let mut all = g.required_symbols.clone();
    all.extend(g.hot_symbols.iter().flatten().cloned());
    names_for(&all)
}

/// The GD32F130C8 mainboard image's guard names (required and hot).
pub fn all_required() -> (Vec<String>, Vec<String>) {
    names_of(&release_guards())
}

/// An executable ARM ELF as the linker would leave it.
pub struct Fixture {
    /// The `.text` bytes at [`BASE`], one loadable segment.
    pub text: Vec<u8>,
    /// The first `thumb` bytes are a `$t` code region, the rest `$d` data the `wfi` scan skips.
    pub thumb: usize,
    /// Global functions, each at an offset into `.text`.
    pub text_syms: Vec<(String, u32)>,
    /// Global objects in a 64 B `.bss` at `0x2000_0000`.
    pub bss_syms: Vec<String>,
    /// A `.debug_info` section, which a strip removes.
    pub debug: bool,
    /// `.text`'s declared alignment. The section sits at a 4-aligned file offset, so 8 gives the
    /// offset-below-alignment layout cortex-m 0.7.10 images have.
    pub text_align: u64,
}

impl Fixture {
    /// `code` padded to `floor` with zeros, all of it data to the `wfi` scan, every name in `names`
    /// at the flash base.
    pub fn new(code: &[u8], floor: u64, names: (Vec<String>, Vec<String>)) -> Fixture {
        let mut text = code.to_vec();
        text.resize(text.len().max(floor as usize), 0);
        Fixture {
            text,
            thumb: 0,
            text_syms: names.0.into_iter().map(|n| (n, 0)).collect(),
            bss_syms: names.1,
            debug: false,
            text_align: 4,
        }
    }

    pub fn build(&self) -> Vec<u8> {
        use object::elf::*;
        use object::write::elf::{FileHeader, ProgramHeader, SectionHeader, Sym, Writer};
        let text = &self.text;
        let debug_bytes = b"not really DWARF, but a .debug_ section all the same";

        let mut out = Vec::new();
        let mut w = Writer::new(object::Endianness::Little, false, &mut out);
        w.reserve_file_header();
        w.reserve_program_headers(1);
        let text_off = w.reserve(text.len(), 4);
        let debug_off = if self.debug {
            w.reserve(debug_bytes.len(), 1)
        } else {
            0
        };
        let text_name = w.add_section_name(b".text");
        let bss_name = w.add_section_name(b".bss");
        let debug_name = self.debug.then(|| w.add_section_name(b".debug_info"));
        w.reserve_null_section_index();
        let text_idx = w.reserve_section_index();
        let bss_idx = w.reserve_section_index();
        if self.debug {
            w.reserve_section_index();
        }
        let t_mark = w.add_string(b"$t");
        let d_mark = w.add_string(b"$d");
        let t_names: Vec<_> = self
            .text_syms
            .iter()
            .map(|(n, a)| (w.add_string(n.as_bytes()), *a))
            .collect();
        let b_names: Vec<_> = self
            .bss_syms
            .iter()
            .map(|n| w.add_string(n.as_bytes()))
            .collect();
        w.reserve_null_symbol_index();
        let locals = if self.thumb > 0 { 2 } else { 1 };
        for _ in 0..locals {
            w.reserve_symbol_index(Some(text_idx));
        }
        for _ in &t_names {
            w.reserve_symbol_index(Some(text_idx));
        }
        for _ in &b_names {
            w.reserve_symbol_index(Some(bss_idx));
        }
        w.reserve_symtab_section_index();
        w.reserve_strtab_section_index();
        w.reserve_shstrtab_section_index();
        w.reserve_symtab();
        w.reserve_strtab();
        w.reserve_shstrtab();
        w.reserve_section_headers();

        w.write_file_header(&FileHeader {
            os_abi: 0,
            abi_version: 0,
            e_type: ET_EXEC,
            e_machine: EM_ARM,
            e_entry: u64::from(BASE),
            e_flags: 0,
        })
        .unwrap();
        w.write_align_program_headers();
        w.write_program_header(&ProgramHeader {
            p_type: PT_LOAD,
            p_flags: PF_R | PF_X,
            p_offset: text_off as u64,
            p_vaddr: u64::from(BASE),
            p_paddr: u64::from(BASE),
            p_filesz: text.len() as u64,
            p_memsz: text.len() as u64,
            p_align: 4,
        });
        w.write_align(4);
        w.write(text);
        if self.debug {
            w.write(debug_bytes);
        }
        w.write_null_symbol();
        let sym = |name, section, st_info, st_value| Sym {
            name: Some(name),
            section: Some(section),
            st_info,
            st_other: 0,
            st_shndx: 0,
            st_value,
            st_size: 0,
        };
        let local = (STB_LOCAL << 4) | STT_NOTYPE;
        if self.thumb > 0 {
            w.write_symbol(&sym(t_mark, text_idx, local, u64::from(BASE)));
        }
        let data_at = BASE + self.thumb as u32;
        w.write_symbol(&sym(d_mark, text_idx, local, u64::from(data_at)));
        for (n, a) in t_names {
            let f = (STB_GLOBAL << 4) | STT_FUNC;
            w.write_symbol(&sym(n, text_idx, f, u64::from(BASE + a)));
        }
        for n in b_names {
            let o = (STB_GLOBAL << 4) | STT_OBJECT;
            w.write_symbol(&sym(n, bss_idx, o, 0x2000_0000));
        }
        w.write_strtab();
        w.write_shstrtab();
        w.write_null_section_header();
        w.write_section_header(&SectionHeader {
            name: Some(text_name),
            sh_type: SHT_PROGBITS,
            sh_flags: u64::from(SHF_ALLOC | SHF_EXECINSTR),
            sh_addr: u64::from(BASE),
            sh_offset: text_off as u64,
            sh_size: text.len() as u64,
            sh_link: 0,
            sh_info: 0,
            sh_addralign: self.text_align,
            sh_entsize: 0,
        });
        w.write_section_header(&SectionHeader {
            name: Some(bss_name),
            sh_type: SHT_NOBITS,
            sh_flags: u64::from(SHF_ALLOC | SHF_WRITE),
            sh_addr: 0x2000_0000,
            sh_offset: 0,
            sh_size: 64,
            sh_link: 0,
            sh_info: 0,
            sh_addralign: 4,
            sh_entsize: 0,
        });
        if let Some(name) = debug_name {
            w.write_section_header(&SectionHeader {
                name: Some(name),
                sh_type: SHT_PROGBITS,
                sh_flags: 0,
                sh_addr: 0,
                sh_offset: debug_off as u64,
                sh_size: debug_bytes.len() as u64,
                sh_link: 0,
                sh_info: 0,
                sh_addralign: 1,
                sh_entsize: 0,
            });
        }
        w.write_symtab_section_header(1 + locals as u32);
        w.write_strtab_section_header();
        w.write_shstrtab_section_header();
        out
    }
}

/// An ELF that passes the GD32F130C8 mainboard image's guards: `code` (padded to the code floor, all of it
/// data to the `wfi` scan) at the flash base, and every guard name.
pub fn flashable_elf(code: &[u8]) -> Vec<u8> {
    let g = release_guards();
    Fixture::new(code, g.code_floor, names_of(&g)).build()
}
