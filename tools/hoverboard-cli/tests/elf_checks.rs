//! The image checks `flash.sh` runs, as `hoverboard-cli flash` runs them, on built ELF fixtures.

mod common;

use common::all_required;
use hoverboard_cli::elf::{self, ElfError, CODE_FLOOR};
use object::write::{Object, StandardSection, Symbol, SymbolSection};
use object::{Architecture, BinaryFormat, Endianness, SymbolFlags, SymbolKind, SymbolScope};

/// An ARM ELF with a `.text` of `code` (a `$t` region, then a `$d` literal pool of `pool`) and a
/// global symbol for each name in `syms`.
fn fixture(code: &[u8], pool: &[u8], syms: &[&str], bss: &[&str]) -> Vec<u8> {
    let mut o = Object::new(BinaryFormat::Elf, Architecture::Arm, Endianness::Little);
    let text = o.section_id(StandardSection::Text);
    let mut body = code.to_vec();
    body.extend_from_slice(pool);
    // Pad to the code floor so the floor check does not fire on these fixtures.
    let pad = (CODE_FLOOR as usize).saturating_sub(body.len());
    body.extend(std::iter::repeat_n(0u8, pad));
    o.append_section_data(text, &body, 4);
    fn sym(
        o: &mut Object,
        name: &str,
        value: u64,
        section: object::write::SectionId,
        scope: SymbolScope,
    ) {
        let kind = if name.starts_with('$') {
            SymbolKind::Label
        } else if section == o.section_id(StandardSection::Text) {
            SymbolKind::Text
        } else {
            SymbolKind::Data
        };
        o.add_symbol(Symbol {
            name: name.as_bytes().to_vec(),
            value,
            size: 0,
            kind,
            scope,
            weak: false,
            section: SymbolSection::Section(section),
            flags: SymbolFlags::None,
        });
    }
    sym(&mut o, "$t", 0, text, SymbolScope::Compilation);
    sym(
        &mut o,
        "$d",
        code.len() as u64,
        text,
        SymbolScope::Compilation,
    );
    sym(
        &mut o,
        "$t",
        (code.len() + pool.len()) as u64,
        text,
        SymbolScope::Compilation,
    );
    for s in syms {
        sym(&mut o, s, 0, text, SymbolScope::Linkage);
    }
    let bss_id = o.section_id(StandardSection::UninitializedData);
    o.append_section_bss(bss_id, 64, 4);
    for s in bss {
        sym(&mut o, s, 0, bss_id, SymbolScope::Linkage);
    }
    o.write().unwrap()
}

fn refs(v: &[String]) -> Vec<&str> {
    v.iter().map(String::as_str).collect()
}

#[test]
fn the_guard_list_is_read_from_the_one_copy() {
    let p = elf::required_patterns();
    assert!(p.len() >= 19, "{p:?}");
    assert!(p.contains(&"T main$".to_string()));
    assert!(p.contains(&"B CTRL_OBS$".to_string()));
}

#[test]
fn a_clean_image_passes() {
    let (t, b) = all_required();
    // movs; bl (32-bit, its second half 0xBF30-like bytes must not be read as a wfi); nop
    let code = [0x00, 0x20, 0xFF, 0xF7, 0x30, 0xBF, 0x00, 0xBF];
    let pool = [0x30, 0xBF, 0x30, 0xBF]; // a literal that looks like wfi, in a $d region
    elf::check(&fixture(&code, &pool, &refs(&t), &refs(&b))).unwrap();
}

#[test]
fn a_wfi_in_code_is_refused_in_both_encodings() {
    let (t, b) = all_required();
    let narrow = [0x00, 0x20, 0x30, 0xBF];
    assert_eq!(
        elf::check(&fixture(&narrow, &[], &refs(&t), &refs(&b))),
        Err(ElfError::Wfi(2))
    );
    let wide = [0x00, 0xBF, 0xAF, 0xF3, 0x03, 0x80];
    assert_eq!(
        elf::check(&fixture(&wide, &[], &refs(&t), &refs(&b))),
        Err(ElfError::Wfi(2))
    );
}

#[test]
fn a_missing_required_symbol_is_named() {
    let (mut t, b) = all_required();
    t.retain(|s| !s.contains("5probe3run"));
    match elf::check(&fixture(&[0, 0xBF], &[], &refs(&t), &refs(&b))) {
        Err(ElfError::MissingSymbols(m)) => assert_eq!(m, vec!["5probe3run".to_string()]),
        other => panic!("{other:?}"),
    }
    // A `B` symbol in .text is not the .bss symbol the guard asks for.
    let (t, _) = all_required();
    assert!(matches!(
        elf::check(&fixture(&[0, 0xBF], &[], &refs(&t), &[])),
        Err(ElfError::MissingSymbols(_))
    ));
}

/// A minimal executable ELF32 with two PT_LOAD segments: .text at 0x0800_0000 and .data whose
/// load address (0x0800_0010) differs from its run address (0x2000_0000).
fn exec_fixture() -> Vec<u8> {
    let mut f = vec![0u8; 0x100];
    f[..4].copy_from_slice(b"\x7fELF");
    f[4] = 1; // ELFCLASS32
    f[5] = 1; // little-endian
    f[6] = 1;
    f[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
    f[18..20].copy_from_slice(&40u16.to_le_bytes()); // EM_ARM
    f[20..24].copy_from_slice(&1u32.to_le_bytes());
    f[28..32].copy_from_slice(&52u32.to_le_bytes()); // e_phoff
    f[40..42].copy_from_slice(&52u16.to_le_bytes()); // e_ehsize
    f[42..44].copy_from_slice(&32u16.to_le_bytes()); // e_phentsize
    f[44..46].copy_from_slice(&2u16.to_le_bytes()); // e_phnum
    let ph = |f: &mut Vec<u8>, at: usize, off: u32, vaddr: u32, paddr: u32, len: u32| {
        let w = [1, off, vaddr, paddr, len, len, 5, 4];
        for (i, v) in w.iter().enumerate() {
            f[at + 4 * i..at + 4 * i + 4].copy_from_slice(&v.to_le_bytes());
        }
    };
    ph(&mut f, 52, 0xC0, 0x0800_0000, 0x0800_0000, 8);
    ph(&mut f, 84, 0xD0, 0x2000_0000, 0x0800_0010, 4);
    f[0xC0..0xC8].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
    f[0xD0..0xD4].copy_from_slice(&[9, 10, 11, 12]);
    f
}

#[test]
fn the_binary_places_each_segment_at_its_load_address() {
    let bin = elf::to_bin(&exec_fixture()).unwrap();
    let mut want = vec![1, 2, 3, 4, 5, 6, 7, 8];
    want.extend([0xFF; 8]);
    want.extend([9, 10, 11, 12]);
    assert_eq!(bin, want);
}
