//! The image checks `flash.sh` runs, as `hoverboard-cli flash` runs them, on built ELF fixtures:
//! the guards read from `tools/image-guard-syms.sh` by image name, the `wfi` scan an ELF given
//! directly gets alone, and the debug strip a bundle stores its ELFs with.

mod common;

use common::{all_required, release_guards, Fixture, GUARD_FILE};
use hoverboard_cli::elf::{self, ElfError, Guards, HOT_LIMIT};
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
    let pad = (release_guards().code_floor as usize).saturating_sub(body.len());
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

fn check(elf: &[u8]) -> Result<(), ElfError> {
    elf::check(elf, &release_guards())
}

#[test]
fn the_guards_are_read_from_the_one_copy_by_image_name() {
    let g = release_guards();
    assert_eq!(g.code_floor, 40_000);
    assert!(g.required_symbols.len() >= 13, "{g:?}");
    assert!(g.required_symbols.contains(&"T main$".to_string()));
    assert!(g.required_symbols.contains(&"B CTRL_OBS$".to_string()));
    assert!(g
        .required_symbols
        .contains(&"T DMA_Channel3_4$".to_string()));
    let hot = g.hot_symbols.unwrap();
    assert!(hot.contains(&"4base5fixed3div".to_string()), "{hot:?}");
    // The other chip's image has its own entry, with its own vector names.
    let f103 = elf::guards_for(GUARD_FILE, "mainboard-gd32f103c8").unwrap();
    assert!(f103
        .required_symbols
        .contains(&"T DMA0_Channel5$".to_string()));
    assert!(f103.hot_symbols.unwrap().contains(&"T ADC0_1$".to_string()));
    assert!(elf::guards_for(GUARD_FILE, "mainboard-gd32")
        .unwrap_err()
        .contains("no guards for image mainboard-gd32"));
}

#[test]
fn an_image_with_no_hot_window_has_no_hot_symbols() {
    let file = "image_guards() {
  case \"$1\" in
    small)
      # a comment
      IMAGE_GUARD_FLOOR=3000
      IMAGE_GUARD_REQ_SYMS='T main$,5probe3run'
      IMAGE_GUARD_HOT_SYMS=''
      ;;
    big)
      IMAGE_GUARD_FLOOR=40000
      IMAGE_GUARD_REQ_SYMS='x'
      IMAGE_GUARD_HOT_SYMS='y,z'
      ;;
  esac
}";
    assert_eq!(
        elf::guards_for(file, "small").unwrap(),
        Guards {
            code_floor: 3000,
            required_symbols: vec!["T main$".into(), "5probe3run".into()],
            hot_symbols: None,
        }
    );
    assert_eq!(
        elf::guards_for(file, "big").unwrap().hot_symbols,
        Some(vec!["y".into(), "z".into()])
    );
    let no_floor = file.replace("IMAGE_GUARD_FLOOR=3000", "");
    assert!(elf::guards_for(&no_floor, "small").is_err());
}

#[test]
fn a_clean_image_passes() {
    let (t, b) = all_required();
    // movs; bl (32-bit, its second half 0xBF30-like bytes must not be read as a wfi); nop
    let code = [0x00, 0x20, 0xFF, 0xF7, 0x30, 0xBF, 0x00, 0xBF];
    let pool = [0x30, 0xBF, 0x30, 0xBF]; // a literal that looks like wfi, in a $d region
    check(&fixture(&code, &pool, &refs(&t), &refs(&b))).unwrap();
}

#[test]
fn a_wfi_in_code_is_refused_in_both_encodings() {
    let (t, b) = all_required();
    let narrow = [0x00, 0x20, 0x30, 0xBF];
    assert_eq!(
        check(&fixture(&narrow, &[], &refs(&t), &refs(&b))),
        Err(ElfError::Wfi(2))
    );
    let wide = [0x00, 0xBF, 0xAF, 0xF3, 0x03, 0x80];
    assert_eq!(
        check(&fixture(&wide, &[], &refs(&t), &refs(&b))),
        Err(ElfError::Wfi(2))
    );
}

#[test]
fn a_missing_required_symbol_is_named() {
    let (mut t, b) = all_required();
    t.retain(|s| !s.contains("5motor7PERIODS"));
    match check(&fixture(&[0, 0xBF], &[], &refs(&t), &refs(&b))) {
        Err(ElfError::MissingSymbols(m)) => assert_eq!(m, vec!["5motor7PERIODS".to_string()]),
        other => panic!("{other:?}"),
    }
    // A `B` symbol in .text is not the .bss symbol the guard asks for.
    let (t, _) = all_required();
    assert!(matches!(
        check(&fixture(&[0, 0xBF], &[], &refs(&t), &[])),
        Err(ElfError::MissingSymbols(_))
    ));
}

#[test]
fn a_hot_symbol_above_the_boundary_or_gone_is_refused() {
    let g = release_guards();
    let (mut t, b) = common::names_of(&g);
    let at = |t: &[String]| -> Vec<(String, u32)> {
        t.iter()
            .map(|n| {
                let off = if n.contains("4base5crc16") { 0x9000 } else { 0 };
                (n.clone(), off)
            })
            .collect()
    };
    let mut f = Fixture::new(&[], g.code_floor, (t.clone(), b.clone()));
    elf::check(&f.build(), &g).unwrap();
    f.text_syms = at(&t);
    match elf::check(&f.build(), &g) {
        Err(ElfError::HotCold(c)) => {
            assert_eq!(c, vec![("4base5crc16".to_string(), HOT_LIMIT + 0x1000)])
        }
        other => panic!("{other:?}"),
    }
    t.retain(|n| !n.contains("6record12is_committed"));
    let f = Fixture::new(&[], g.code_floor, (t, b));
    assert_eq!(
        elf::check(&f.build(), &g),
        Err(ElfError::HotMissing(vec!["6record12is_committed".into()]))
    );
}

#[test]
fn an_elf_given_directly_gets_the_wfi_scan_alone() {
    // Below the floor and with no guard symbols: the guards would refuse it, the scan does not.
    let mut f = Fixture::new(&[0x00, 0x20, 0x00, 0xBF], 0, (vec!["main".into()], vec![]));
    f.thumb = 4;
    let elf = f.build();
    elf::check_wfi(&elf).unwrap();
    assert!(matches!(check(&elf), Err(ElfError::CodeFloor(4, 40_000))));
    f.text = vec![0x00, 0x20, 0x30, 0xBF];
    assert_eq!(elf::check_wfi(&f.build()), Err(ElfError::Wfi(0x0800_0002)));
}

#[test]
fn a_size_measurement_image_is_never_turned_into_a_flash_binary() {
    // Every guard passes, so only the marker refuses it: on the path `board flash` takes (the wfi
    // scan, then the binary) and on the bundle's (the guards, then the binary).
    let g = release_guards();
    let (mut text, bss) = common::names_of(&g);
    text.push(elf::NOT_FLASHABLE.to_string());
    let elf = Fixture::new(&[], g.code_floor, (text, bss)).build();
    elf::check_wfi(&elf).unwrap();
    elf::check(&elf, &g).unwrap();
    assert_eq!(elf::to_bin(&elf), Err(ElfError::NotFlashable));
}

#[test]
fn the_strip_removes_the_debug_sections_and_keeps_the_symbols_and_the_flashed_bytes() {
    use object::{Object, ObjectSection, ObjectSymbol};
    let g = release_guards();
    let mut f = Fixture::new(&[1, 2, 3, 4], g.code_floor, common::names_of(&g));
    f.debug = true;
    let full = f.build();
    let stripped = elf::strip_debug(&full).unwrap();
    let names = |b: &[u8]| -> (Vec<String>, Vec<String>) {
        let o = object::File::parse(b).unwrap();
        let s = o
            .sections()
            .map(|s| s.name().unwrap().to_string())
            .collect();
        let y = o.symbols().map(|s| s.name().unwrap().to_string()).collect();
        (s, y)
    };
    let (full_secs, full_syms) = names(&full);
    let (secs, syms) = names(&stripped);
    assert!(full_secs.contains(&".debug_info".to_string()));
    assert!(!secs.iter().any(|s| s.starts_with(".debug_")), "{secs:?}");
    assert!(secs.contains(&".symtab".to_string()));
    assert_eq!(syms, full_syms);
    assert_eq!(elf::to_bin(&stripped).unwrap(), elf::to_bin(&full).unwrap());
    elf::check(&stripped, &g).unwrap();
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

#[test]
fn the_strip_takes_a_section_whose_offset_is_less_aligned_than_its_address() {
    use object::{Object, ObjectSection};
    // cortex-m 0.7.10 images declare `.text` 8-aligned at a file offset that is only 4-aligned.
    let g = release_guards();
    let mut f = Fixture::new(&[1, 2, 3, 4], g.code_floor, common::names_of(&g));
    f.debug = true;
    f.text_align = 8;
    let full = f.build();
    let text_offset = |b: &[u8]| {
        let o = object::File::parse(b).unwrap();
        let t = o.section_by_name(".text").unwrap();
        (t.file_range().unwrap().0, t.data().unwrap().to_vec())
    };
    assert_ne!(
        text_offset(&full).0 % 8,
        0,
        "the fixture places .text off its alignment"
    );
    let stripped = elf::strip_debug(&full).unwrap();
    assert_eq!(text_offset(&stripped), text_offset(&full));
    assert_eq!(elf::to_bin(&stripped), elf::to_bin(&full));
}
