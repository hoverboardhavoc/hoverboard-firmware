//! The firmware bundle (`specs/firmware-bundle.md` section 6, host tests): the manifest and the
//! name form, the selection against a `TargetId`, the SHA-256 check, bundle-or-ELF detection, the
//! guards taken from the manifest, and the packer.

mod common;

use common::{names_of, release_guards, Fixture};
use espsvc::{TargetId, FAMILY_F10X, FAMILY_F1X0};
use hoverboard_cli::bundle::{
    self, check_name, check_table, file_kind, image_table, is_bundle, pack, BoardTarget, Bundle,
    BundleError, EspChip, Family, FileKind, Image, ImageKind, ImageTable, Manifest, PackEntry,
    SelectError, Target, TreeEntry,
};
use hoverboard_cli::elf::{self, ElfError, Guards};
use sha2::{Digest, Sha256};

/// The image tables, from the crate that builds the images.
const FIRMWARE_TOML: &str = include_str!("../../../motorcontroller/firmware/Cargo.toml");

fn chip(family: u8, flash_kib: u16) -> TargetId {
    TargetId {
        family,
        flash_kib,
        rdp: 0xA5,
        ..TargetId::default()
    }
}

fn t(family: Family, flash_kib: u16) -> BoardTarget {
    BoardTarget { family, flash_kib }
}

const F10X_64: BoardTarget = BoardTarget {
    family: Family::Gd32f10x,
    flash_kib: 64,
};
const F1X0_64: BoardTarget = BoardTarget {
    family: Family::Gd32f1x0,
    flash_kib: 64,
};

/// Board targets as the manifest lists them.
fn boards(t: &[BoardTarget]) -> Vec<Target> {
    t.iter().copied().map(Target::Board).collect()
}

/// The bridge target of the ESP32-C3.
const C3: Target = Target::Bridge(bundle::BridgeTarget {
    chip: EspChip::Esp32c3,
});

fn sha(b: &[u8]) -> String {
    Sha256::digest(b)
        .iter()
        .map(|x| format!("{x:02x}"))
        .collect()
}

/// An ELF that passes the GD32F130C8 mainboard image's guards.
fn release_elf() -> Vec<u8> {
    common::flashable_elf(&[0x11, 0x22, 0x33, 0x44])
}

/// A small image's guards and an ELF that passes them and fails the mainboard image's.
fn small() -> (Guards, Vec<u8>) {
    let g = Guards {
        code_floor: 2000,
        required_symbols: vec!["T main$".into(), "cortex_m_rt_main".into()],
        hot_symbols: None,
    };
    let elf = Fixture::new(&[0x55; 8], g.code_floor, names_of(&g)).build();
    (g, elf)
}

fn image(name: &str, targets: &[BoardTarget], guards: &Guards, elf: &[u8]) -> Image {
    Image {
        name: name.into(),
        kind: ImageKind::Board,
        file: format!("{name}.elf"),
        targets: boards(targets),
        guards: Some(guards.clone()),
        sha256: sha(elf),
    }
}

/// A bridge image of `kind` for the ESP32-C3, holding `file`.
fn bridge_image(name: &str, kind: ImageKind, file: &[u8]) -> Image {
    Image {
        name: name.into(),
        kind,
        file: format!("{name}.bin"),
        targets: vec![C3],
        guards: None,
        sha256: sha(file),
    }
}

/// An ESP32-C3 app image of `len` bytes whose app descriptor carries `version`, as
/// `espflash save-image` writes one.
fn app_image(len: usize, version: &str) -> Vec<u8> {
    let mut img: Vec<u8> = (0..len as u32)
        .map(|i| (i.wrapping_mul(2_654_435_761) >> 11) as u8)
        .collect();
    img[0] = 0xE9;
    img[12..14].copy_from_slice(&5u16.to_le_bytes());
    img[0x20..0x24].copy_from_slice(&0xABCD_5432u32.to_le_bytes());
    img[0x30..0x50].fill(0);
    img[0x30..0x30 + version.len()].copy_from_slice(version.as_bytes());
    img
}

/// A merged install image: the whole 4 MB, starting with the ESP image magic (the bootloader's
/// header, which carries no app descriptor).
fn install_image() -> Vec<u8> {
    let mut img = vec![0xFF; hoverboard_cli::install::IMAGE_LEN];
    img[..64].fill(0x11);
    img[0] = 0xE9;
    img
}

/// A bundle of `images`, each with its ELF, written without the packer's checks.
fn bundle_of(images: Vec<(Image, Vec<u8>)>) -> Vec<u8> {
    let (imgs, elfs): (Vec<Image>, Vec<Vec<u8>>) = images.into_iter().unzip();
    let m = Manifest {
        commit: "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd".into(),
        built: "2026-11-01T17:03:58Z".into(),
        images: imgs,
    };
    let refs: Vec<&[u8]> = elfs.iter().map(Vec::as_slice).collect();
    bundle::write(&m, &refs)
}

/// The manifest of the spec's example, with `edit` applied to its text.
fn manifest_text(edit: impl Fn(String) -> String) -> String {
    let s = r#"{
  "commit": "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd",
  "built": "2026-11-01T17:03:58Z",
  "images": [
    {
      "name": "mainboard-gd32f130c8",
      "kind": "board",
      "file": "mainboard-gd32f130c8.elf",
      "targets": [
        { "family": "gd32f1x0", "flash_kib": 64 }
      ],
      "guards": {
        "code_floor": 40000,
        "required_symbols": ["T main$"],
        "hot_symbols": ["9run_shell"]
      },
      "sha256": "eaafb89defe9379f9e80af90e1af1af6ed35ef341d885a5fcd195c03c102cbb0"
    }
  ]
}"#;
    edit(s.to_string())
}

#[test]
fn the_manifest_parses() {
    let m = Manifest::parse(&manifest_text(|s| s)).unwrap();
    assert_eq!(m.commit, "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd");
    assert_eq!(m.built, "2026-11-01T17:03:58Z");
    let i = &m.images[0];
    assert_eq!(i.name, "mainboard-gd32f130c8");
    assert_eq!(i.file, "mainboard-gd32f130c8.elf");
    assert_eq!(i.kind, ImageKind::Board);
    assert_eq!(i.targets, boards(&[F1X0_64]));
    let g = i.guards.as_ref().unwrap();
    assert_eq!(g.code_floor, 40_000);
    assert_eq!(g.hot_symbols, Some(vec!["9run_shell".to_string()]));
    // An image with no hot-window gate leaves the key out.
    let m = Manifest::parse(&manifest_text(|s| {
        s.replace(",\n        \"hot_symbols\": [\"9run_shell\"]", "")
    }))
    .unwrap();
    assert_eq!(m.images[0].guards.as_ref().unwrap().hot_symbols, None);
}

#[test]
fn a_malformed_manifest_is_refused() {
    let edits: &[(&str, &str)] = &[
        ("\"commit\"", "\"version\": 1, \"commit\""), // an unknown key
        ("\"kind\": \"board\",", ""),                 // a missing key
        ("\"board\"", "\"controller\""),              // an unknown kind
        ("\"mainboard-gd32f130c8\",", "\"controller-gd32f130c8\","), // a name of no role
        ("\"mainboard-gd32f130c8\",", "\"mainboard_gd32f130c8\","), // not <role>-<chip>
        ("\"gd32f1x0\"", "\"gd32f30x\""),             // an unknown family
        ("\"flash_kib\": 64 }\n", "\"flash_kib\": -1 }\n"), // not a size
        ("eaafb89d", "eaafb89"),                      // not 64 hex digits
        ("\"images\": [", "\"images\": [], \"x\": ["), // no images, and an unknown key
        ("{", "["),                                   // not JSON
    ];
    for (from, to) in edits {
        let text = manifest_text(|s| s.replacen(from, to, 1));
        assert!(
            matches!(Manifest::parse(&text), Err(BundleError::Manifest(_))),
            "{from} -> {to} was accepted"
        );
    }
    let no_images = r#"{"commit": "a", "built": "b", "images": []}"#;
    assert!(Manifest::parse(no_images).is_err());
    // Two images of one name.
    let one = manifest_text(|s| s);
    let m: serde_json::Value = serde_json::from_str(&one).unwrap();
    let img = m["images"][0].clone();
    let mut twice = m.clone();
    let mut other = img.clone();
    other["file"] = "other.elf".into();
    twice["images"] = serde_json::Value::Array(vec![img, other]);
    match Manifest::parse(&twice.to_string()) {
        Err(BundleError::Manifest(e)) => assert!(e.contains("given twice"), "{e}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_bundle_is_told_from_an_elf_by_its_content() {
    let elf = release_elf();
    let z = bundle_of(vec![(
        image("mainboard-gd32f130c8", &[F1X0_64], &release_guards(), &elf),
        elf.clone(),
    )]);
    assert_eq!(file_kind(&z), Ok(FileKind::Bundle));
    assert_eq!(file_kind(&elf), Ok(FileKind::Elf));
    assert!(file_kind(b"\xe9\x03\x02\x20 an ESP image").is_err());
    assert!(file_kind(b"").is_err());
}

#[test]
fn a_sha256_mismatch_is_refused_before_the_elf_is_parsed() {
    // Not an ELF at all: a parse would fail, so a refusal naming the SHA-256 came first.
    let junk = vec![0xA5u8; 64];
    let mut img = image("mainboard-gd32f130c8", &[F1X0_64], &release_guards(), &junk);
    img.sha256 = sha(b"what the packer stored");
    let z = bundle_of(vec![(img, junk)]);
    assert_eq!(
        Bundle::open(&z).unwrap_err(),
        BundleError::Sha256("mainboard-gd32f130c8.elf".into())
    );
}

#[test]
fn a_file_the_manifest_lists_must_be_in_the_archive() {
    let elf = release_elf();
    let img = image("mainboard-a", &[F1X0_64], &release_guards(), &elf);
    let m = Manifest {
        commit: "c".into(),
        built: "b".into(),
        images: vec![img.clone()],
    };
    // An archive holding the manifest and a differently named file.
    let mut w = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
    let o = zip::write::SimpleFileOptions::default();
    use std::io::Write;
    w.start_file("manifest.json", o).unwrap();
    w.write_all(serde_json::to_string(&m).unwrap().as_bytes())
        .unwrap();
    w.start_file("b.elf", o).unwrap();
    w.write_all(&elf).unwrap();
    let z = w.finish().unwrap().into_inner();
    assert_eq!(
        Bundle::open(&z).unwrap_err(),
        BundleError::Missing("mainboard-a.elf".into())
    );
    assert!(matches!(
        Bundle::open(b"PK\x03\x04 truncated"),
        Err(BundleError::Archive(_))
    ));
}

/// The GD32F103C8 mainboard image's guards and an ELF that passes them.
fn f103() -> (Guards, Vec<u8>) {
    let g = elf::guards_for(common::GUARD_FILE, "mainboard-gd32f103c8").unwrap();
    let elf = Fixture::new(&[0x66; 8], g.code_floor, names_of(&g)).build();
    (g, elf)
}

/// A bundle of four images: the two 64 KiB mainboard images, one per family, and two F130 32 KiB
/// images (a mainboard and a sideboard), so the 32 KiB chip has several matches.
fn fleet() -> Bundle {
    let elf = release_elf();
    let (sg, selfm) = small();
    let (g103, elf103) = f103();
    let f130_32 = t(Family::Gd32f1x0, 32);
    Bundle::open(&bundle_of(vec![
        (
            image("mainboard-gd32f130c8", &[F1X0_64], &release_guards(), &elf),
            elf.clone(),
        ),
        (
            image("mainboard-gd32f130c6", &[f130_32], &sg, &selfm),
            selfm.clone(),
        ),
        (
            image("sideboard-gd32f130c6", &[f130_32], &sg, &selfm),
            selfm,
        ),
        (
            image("mainboard-gd32f103c8", &[F10X_64], &g103, &elf103),
            elf103.clone(),
        ),
    ]))
    .unwrap()
}

#[test]
fn selection_takes_the_one_image_for_the_chip() {
    let b = fleet();
    for (family, target, name) in [
        (FAMILY_F10X, F10X_64, "mainboard-gd32f103c8"),
        (FAMILY_F1X0, F1X0_64, "mainboard-gd32f130c8"),
    ] {
        let (img, matched) = b.select(&chip(family, 64), None).unwrap();
        assert_eq!(img.name, name);
        assert_eq!(matched, target);
        // --image naming the match is the same choice.
        let (img, _) = b.select(&chip(family, 64), Some(name)).unwrap();
        assert_eq!(img.name, name);
    }
    let line = bundle::choice_line(&b.manifest, &b.manifest.images[0], F1X0_64);
    assert_eq!(
        line,
        "image mainboard-gd32f130c8 for the chip (gd32f1x0 64 KiB), from the bundle built from \
         commit c0eb25917cd904d5d9ba9307f56cb8251cbb61dd"
    );
}

#[test]
fn selection_with_no_match_names_the_chip_and_every_image() {
    let b = fleet();
    let e = b.select(&chip(FAMILY_F10X, 128), None).unwrap_err();
    assert_eq!(
        e,
        SelectError::NoMatch {
            chip: t(Family::Gd32f10x, 128),
            images: vec![
                ("mainboard-gd32f130c8".into(), boards(&[F1X0_64])),
                (
                    "mainboard-gd32f130c6".into(),
                    boards(&[t(Family::Gd32f1x0, 32)])
                ),
                (
                    "sideboard-gd32f130c6".into(),
                    boards(&[t(Family::Gd32f1x0, 32)])
                ),
                ("mainboard-gd32f103c8".into(), boards(&[F10X_64])),
            ],
        }
    );
    let text = e.to_string();
    assert!(text.contains("(gd32f10x 128 KiB)"), "{text}");
    assert!(
        text.contains("mainboard-gd32f103c8: gd32f10x 64 KiB"),
        "{text}"
    );
    assert_eq!(
        b.select(&chip(7, 64), None).unwrap_err(),
        SelectError::UnknownFamily(7)
    );
}

#[test]
fn several_matches_need_image_and_list_them() {
    let b = fleet();
    let c = chip(FAMILY_F1X0, 32);
    let e = b.select(&c, None).unwrap_err();
    assert_eq!(
        e,
        SelectError::Several {
            chip: t(Family::Gd32f1x0, 32),
            images: vec!["mainboard-gd32f130c6".into(), "sideboard-gd32f130c6".into(),],
        }
    );
    assert!(e.to_string().contains("\n  sideboard-gd32f130c6"), "{e}");
}

#[test]
fn image_cannot_override_the_match() {
    let b = fleet();
    assert_eq!(
        b.select(&chip(FAMILY_F1X0, 64), Some("sideboard-gd32f130c6"))
            .unwrap_err(),
        SelectError::NotForChip {
            name: "sideboard-gd32f130c6".into(),
            chip: F1X0_64,
            targets: boards(&[t(Family::Gd32f1x0, 32)]),
        }
    );
    assert_eq!(
        b.select(&chip(FAMILY_F1X0, 64), Some("mainboard-gd32f103rc"))
            .unwrap_err(),
        SelectError::NotInBundle("mainboard-gd32f103rc".into())
    );
}

#[test]
fn a_match_the_bridge_would_refuse_is_refused_naming_the_part() {
    // --image resolves the several matches, and the 32 KiB F130 is in the bridge's part table.
    let b = fleet();
    let (img, _) = b
        .select(&chip(FAMILY_F1X0, 32), Some("mainboard-gd32f130c6"))
        .unwrap();
    assert_eq!(img.name, "mainboard-gd32f130c6");
    // The 256 KiB F103 is in no row: an image for it matches, and is refused.
    let (g103, elf103) = f103();
    let rc = t(Family::Gd32f10x, 256);
    let b = Bundle::open(&bundle_of(vec![(
        image("mainboard-gd32f103rc", &[rc], &g103, &elf103),
        elf103.clone(),
    )]))
    .unwrap();
    let e = b.select(&chip(FAMILY_F10X, 256), None).unwrap_err();
    assert_eq!(
        e,
        SelectError::BridgeRefuses {
            name: "mainboard-gd32f103rc".into(),
            chip: rc,
        }
    );
    assert!(e.to_string().contains("gd32f10x 256 KiB part"), "{e}");
}

#[test]
fn the_mm32_is_named_mm32spin05_and_an_unsupported_part_is_refused_with_its_ids() {
    let (sg, selfm) = small();
    let mm32 = t(Family::Mm32spin05, 32);
    let b = Bundle::open(&bundle_of(vec![(
        image("mainboard-mm32spin05", &[mm32], &sg, &selfm),
        selfm.clone(),
    )]))
    .unwrap();
    let (img, matched) = b
        .select(&chip(espsvc::FAMILY_MM32SPIN05, 32), None)
        .unwrap();
    assert_eq!((img.name.as_str(), matched), ("mainboard-mm32spin05", mm32));
    assert_eq!(matched.to_string(), "mm32spin05 32 KiB");
    let id = TargetId {
        family: espsvc::FAMILY_UNSUPPORTED,
        cpuid: 0x410F_C241,
        mcu_id: 0,
        ..TargetId::default()
    };
    let e = b.select(&id, None).unwrap_err();
    assert_eq!(
        e,
        SelectError::UnsupportedPart {
            cpuid: 0x410F_C241,
            mcu_id: 0
        }
    );
    assert!(e.to_string().contains("cpuid 0x410fc241"), "{e}");
}

#[test]
fn the_guards_come_from_the_manifest() {
    let b = fleet();
    let main = &b.manifest.images[0];
    let tiny = &b.manifest.images[1];
    // Each passes its own guards.
    assert_eq!(b.prepare(main).unwrap(), elf::to_bin(b.file(main)).unwrap());
    b.prepare(tiny).unwrap();
    // The small image fails the mainboard image's guards (its floor), and the mainboard image
    // fails the small one's (a required symbol it does not carry).
    assert!(matches!(
        elf::check(b.file(tiny), main.guards.as_ref().unwrap()),
        Err(ElfError::CodeFloor(_, 40_000))
    ));
    assert!(matches!(
        elf::check(b.file(main), tiny.guards.as_ref().unwrap()),
        Err(ElfError::MissingSymbols(m)) if m == vec!["cortex_m_rt_main".to_string()]
    ));
    // Each family's mainboard image carries its own chip's vectors: the F103 image fails the F130
    // image's guards on the DMA vector of the link's receive ring.
    let f103 = &b.manifest.images[3];
    b.prepare(f103).unwrap();
    assert!(matches!(
        elf::check(b.file(f103), main.guards.as_ref().unwrap()),
        Err(ElfError::MissingSymbols(m)) if m == vec!["T DMA_Channel3_4$".to_string()]
    ));
    // The same ELF under another image's guards in the manifest is refused by prepare.
    let (sg, _) = small();
    let elf = b.file(main).to_vec();
    let swapped = Bundle::open(&bundle_of(vec![(
        image("mainboard-gd32f130c8", &[F1X0_64], &sg, &elf),
        elf.clone(),
    )]))
    .unwrap();
    assert!(matches!(
        swapped.prepare(&swapped.manifest.images[0]),
        Err(ElfError::MissingSymbols(_))
    ));
}

#[test]
fn the_image_tables_are_in_their_cargo_toml() {
    for (name, target) in [
        ("mainboard-gd32f103c8", F10X_64),
        ("mainboard-gd32f130c8", F1X0_64),
    ] {
        let chip = name.strip_prefix("mainboard-").unwrap();
        assert_eq!(
            image_table(FIRMWARE_TOML, name).unwrap(),
            ImageTable {
                kind: ImageKind::Board,
                features: vec![chip.to_string()],
                targets: vec![Target::Board(target)],
            }
        );
    }
    assert!(image_table(FIRMWARE_TOML, "mainboard-gd32f103rc")
        .unwrap_err()
        .contains("no [package.metadata.hoverboard-image.mainboard-gd32f103rc]"));
    let bad = "[package.metadata.hoverboard-image.x]\nkind = \"board\"\nfeatures = []\n\
               targets = [{ family = \"gd32f10x\", flash_kib = 64, ram_kib = 20 }]\n";
    assert!(image_table(bad, "x").is_err());
    // The name's chip is checked against the table's features (section 3.2).
    let table = |name: &str, features: &str| {
        format!(
            "[package.metadata.hoverboard-image.{name}]\nkind = \"board\"\nfeatures = \
             {features}\ntargets = [{{ family = \"gd32f1x0\", flash_kib = 64 }}]\n"
        )
    };
    let ok = table("mainboard-gd32f130c8", r#"["gd32f130c8"]"#);
    assert!(image_table(&ok, "mainboard-gd32f130c8").is_ok());
    let wrong = table("mainboard-gd32f103c8", r#"["gd32f130c8"]"#);
    let e = image_table(&wrong, "mainboard-gd32f103c8").unwrap_err();
    assert!(e.contains("one chip feature"), "{e}");
    let none = table("mainboard-gd32f130c8", "[]");
    assert!(image_table(&none, "mainboard-gd32f130c8").is_err());
}

#[test]
fn the_packer_round_trips() {
    let g = release_guards();
    let mut f = Fixture::new(&[0x11, 0x22, 0x33, 0x44], g.code_floor, names_of(&g));
    f.debug = true;
    let built = f.build();
    let table = image_table(FIRMWARE_TOML, "mainboard-gd32f130c8").unwrap();
    let z = pack(
        &[PackEntry {
            name: "mainboard-gd32f130c8",
            table,
            guards: Some(g.clone()),
            file: &built,
        }],
        "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd",
        "2026-11-01T17:03:58Z",
    )
    .unwrap();
    let b = Bundle::open(&z).unwrap();
    let stored = elf::strip_debug(&built).unwrap();
    assert_eq!(
        b.manifest,
        Manifest {
            commit: "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd".into(),
            built: "2026-11-01T17:03:58Z".into(),
            images: vec![Image {
                name: "mainboard-gd32f130c8".into(),
                kind: ImageKind::Board,
                file: "mainboard-gd32f130c8.elf".into(),
                targets: boards(&[F1X0_64]),
                guards: Some(g),
                sha256: sha(&stored),
            }],
        }
    );
    let img = &b.manifest.images[0];
    assert_eq!(b.file(img), &stored[..]);
    assert!(b.file(img).len() < built.len());
    assert_eq!(b.prepare(img).unwrap(), elf::to_bin(&built).unwrap());
    let info = b.describe();
    assert!(info.starts_with("commit  c0eb2591"), "{info}");
    assert!(
        info.contains("image   mainboard-gd32f130c8 (board): gd32f1x0 64 KiB; 40000 B flashed"),
        "{info}"
    );
}

#[test]
fn the_packer_refuses_an_elf_that_fails_its_guards() {
    let g = release_guards();
    let table = || image_table(FIRMWARE_TOML, "mainboard-gd32f130c8").unwrap();
    let entry = |file: &'static [u8]| PackEntry {
        name: "mainboard-gd32f130c8",
        table: table(),
        guards: Some(release_guards()),
        file,
    };
    // A required symbol gone.
    let (mut text, bss) = names_of(&g);
    text.retain(|n| !n.contains("5motor2hw10period_isr"));
    let gutted: &'static [u8] = Fixture::new(&[], g.code_floor, (text, bss)).build().leak();
    let e = pack(&[entry(gutted)], "c", "b").unwrap_err();
    assert!(
        e.contains("mainboard-gd32f130c8: REFUSED: required symbol"),
        "{e}"
    );
    // A wfi in code.
    let mut f = Fixture::new(&[0x00, 0x20, 0x30, 0xBF], g.code_floor, names_of(&g));
    f.thumb = 4;
    let sleeper: &'static [u8] = f.build().leak();
    let e = pack(&[entry(sleeper)], "c", "b").unwrap_err();
    assert!(e.contains("'wfi'"), "{e}");
    // The small image's ELF under the mainboard image's name and guards.
    let (_, small_elf) = small();
    let e = pack(&[entry(small_elf.leak())], "c", "b").unwrap_err();
    assert!(e.contains("floor"), "{e}");
    // One name twice.
    let ok: &'static [u8] = release_elf().leak();
    let e = pack(&[entry(ok), entry(ok)], "c", "b").unwrap_err();
    assert!(e.contains("given twice"), "{e}");
}

#[test]
fn a_pack_argument_names_crate_image_and_elf() {
    assert_eq!(
        TreeEntry::parse("motorcontroller/firmware:mainboard-gd32f130c8=target/x/firmware")
            .unwrap(),
        TreeEntry {
            crate_dir: "motorcontroller/firmware".into(),
            name: "mainboard-gd32f130c8".into(),
            file: "target/x/firmware".into(),
        }
    );
    // A drive letter's colon stays in the crate directory.
    let e = TreeEntry::parse(r"C:\fw\firmware:mainboard-gd32f130c8=C:\fw\out").unwrap();
    assert_eq!(e.crate_dir, std::path::PathBuf::from(r"C:\fw\firmware"));
    assert_eq!(e.name, "mainboard-gd32f130c8");
    assert!(TreeEntry::parse("motorcontroller/firmware=x").is_err());
    assert!(TreeEntry::parse("dir:Main=x").is_err());
}

#[test]
fn image_names_are_role_then_chip() {
    for ok in [
        "mainboard-gd32f103c8",
        "mainboard-gd32f130c8",
        "sideboard-gd32f130c6",
        "mainboard-gd32f103rc",
    ] {
        assert!(check_name(ok, ImageKind::Board).is_ok(), "{ok}");
    }
    for bad in [
        "gd32f103c8-mainboard",
        "mainboard",
        "mainboard-",
        "controller-gd32f103c8",
        "mainboard-GD32F103C8",
        "mainboard-gd32f103c8-x",
    ] {
        assert!(check_name(bad, ImageKind::Board).is_err(), "{bad}");
    }
    assert_eq!(
        check_name("mainboard-gd32f103c8", ImageKind::Board),
        Ok("gd32f103c8")
    );
    // Against the table: the name's chip is the table's one chip feature.
    let f = |v: &[&str]| ImageTable {
        kind: ImageKind::Board,
        features: v.iter().map(|s| s.to_string()).collect(),
        targets: boards(&[F10X_64]),
    };
    let name = "mainboard-gd32f103c8";
    assert!(check_table(name, &f(&["gd32f103c8"])).is_ok());
    assert!(check_table(name, &f(&["gd32f130c8"])).is_err());
    assert!(check_table(name, &f(&[])).is_err());
    assert!(check_table(name, &f(&["gd32f103c8", "imu"])).is_ok());
    assert!(check_table(name, &f(&["gd32f103c8", "gd32f130c8"])).is_err());
    // A board image's targets are board targets.
    let mut c3 = f(&["gd32f103c8"]);
    c3.targets = vec![C3];
    assert!(check_table(name, &c3)
        .unwrap_err()
        .contains("not a board target"));
}

#[test]
fn the_bundle_is_deflated() {
    let elf = release_elf();
    let z = bundle_of(vec![(
        image("mainboard-gd32f130c8", &[F1X0_64], &release_guards(), &elf),
        elf.clone(),
    )]);
    let mut a = zip::ZipArchive::new(std::io::Cursor::new(&z)).unwrap();
    for i in 0..a.len() {
        let f = a.by_index(i).unwrap();
        assert_eq!(
            f.compression(),
            zip::CompressionMethod::Deflated,
            "{:?}",
            f.name()
        );
    }
    assert!(Bundle::open(&z).is_ok());
}

#[test]
fn the_build_time_is_utc() {
    assert_eq!(bundle::utc(0), "1970-01-01T00:00:00Z");
    assert_eq!(bundle::utc(1_793_552_638), "2026-11-01T17:03:58Z");
    assert_eq!(bundle::utc(951_782_400), "2000-02-29T00:00:00Z");
}

/// The bridge crate's image tables.
const BRIDGE_TOML: &str = include_str!("../../../bridge/Cargo.toml");

/// A bundle of the GD32F130C8 mainboard image and the bridge's two, packed from the crates' own
/// tables.
fn with_bridge(app: &[u8], install: &[u8]) -> Result<Vec<u8>, String> {
    let elf = release_elf();
    pack(
        &[
            PackEntry {
                name: "mainboard-gd32f130c8",
                table: image_table(FIRMWARE_TOML, "mainboard-gd32f130c8").unwrap(),
                guards: Some(release_guards()),
                file: &elf,
            },
            PackEntry {
                name: "bridge-esp32c3",
                table: image_table(BRIDGE_TOML, "bridge-esp32c3").unwrap(),
                guards: None,
                file: app,
            },
            PackEntry {
                name: "bridge-esp32c3-install",
                table: image_table(BRIDGE_TOML, "bridge-esp32c3-install").unwrap(),
                guards: None,
                file: install,
            },
        ],
        "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd",
        "2026-11-01T17:03:58Z",
    )
}

#[test]
fn the_bridge_tables_are_in_the_bridge_crate() {
    for (name, kind) in [
        ("bridge-esp32c3", ImageKind::Bridge),
        ("bridge-esp32c3-install", ImageKind::BridgeInstall),
    ] {
        assert_eq!(
            image_table(BRIDGE_TOML, name).unwrap(),
            ImageTable {
                kind,
                features: Vec::new(),
                targets: vec![C3],
            }
        );
    }
}

#[test]
fn the_packer_round_trips_the_bridge_images() {
    let app = app_image(9_001, "0.2.0");
    let inst = install_image();
    let z = with_bridge(&app, &inst).unwrap();
    let b = Bundle::open(&z).unwrap();
    let m = &b.manifest;
    assert_eq!(
        m.images[1],
        Image {
            name: "bridge-esp32c3".into(),
            kind: ImageKind::Bridge,
            file: "bridge-esp32c3.bin".into(),
            targets: vec![C3],
            guards: None,
            sha256: sha(&app),
        }
    );
    assert_eq!(m.images[2].kind, ImageKind::BridgeInstall);
    assert_eq!(m.images[2].file, "bridge-esp32c3-install.bin");
    // Stored as built, and checked as its install command checks it.
    assert_eq!(b.file(&m.images[1]), &app[..]);
    assert_eq!(
        b.bridge_file(&m.images[1]).unwrap(),
        (&app[..], Some([0, 2, 0]))
    );
    assert_eq!(b.bridge_file(&m.images[2]).unwrap(), (&inst[..], None));
    // The manifest carries no guards for a bridge image, and the kind's name.
    let mut a = zip::ZipArchive::new(std::io::Cursor::new(&z)).unwrap();
    let mut text = String::new();
    std::io::Read::read_to_string(&mut a.by_name("manifest.json").unwrap(), &mut text).unwrap();
    let json: serde_json::Value = serde_json::from_str(&text).unwrap();
    assert!(json["images"][0].get("guards").is_some());
    assert!(json["images"][1].get("guards").is_none());
    assert_eq!(json["images"][2]["kind"], "bridge-install");
    assert_eq!(json["images"][2]["targets"][0]["chip"], "esp32c3");
    // The mostly erased install image deflates to a fraction of its size.
    assert!(z.len() < inst.len() / 100, "{} B", z.len());
    let info = b.describe();
    assert!(
        info.contains("image   bridge-esp32c3 (bridge): esp32c3; 9001 B\n"),
        "{info}"
    );
    assert!(
        info.contains("image   bridge-esp32c3-install (bridge-install): esp32c3; 4194304 B\n"),
        "{info}"
    );
    assert_eq!(b.commit_word(), Some(0xc0eb_2591));
}

#[test]
fn the_packer_refuses_a_bridge_image_its_install_would_refuse() {
    let app = app_image(9_001, "0.2.0");
    let inst = install_image();
    // The two files swapped: each fails the other's check.
    let e = with_bridge(&inst, &inst).unwrap_err();
    assert!(
        e.contains("bridge-esp32c3: not an ESP32-C3 app image"),
        "{e}"
    );
    let e = with_bridge(&app, &app).unwrap_err();
    assert!(
        e.contains("bridge-esp32c3-install: not a merged install image: 9001 B"),
        "{e}"
    );
    // A bridge image has no guards.
    let e = pack(
        &[PackEntry {
            name: "bridge-esp32c3",
            table: image_table(BRIDGE_TOML, "bridge-esp32c3").unwrap(),
            guards: Some(release_guards()),
            file: &app,
        }],
        "c",
        "b",
    )
    .unwrap_err();
    assert!(e.contains("a bridge image has no guards"), "{e}");
}

#[test]
fn bridge_image_names_are_bridge_then_chip() {
    assert_eq!(
        check_name("bridge-esp32c3", ImageKind::Bridge),
        Ok("esp32c3")
    );
    assert_eq!(
        check_name("bridge-esp32c3-install", ImageKind::BridgeInstall),
        Ok("esp32c3")
    );
    for (bad, kind) in [
        ("bridge-esp32c3-install", ImageKind::Bridge),
        ("bridge-esp32c3", ImageKind::BridgeInstall),
        ("mainboard-gd32f130c8", ImageKind::Bridge),
        ("bridge-gd32f130c8", ImageKind::Board),
        ("esp32c3-bridge", ImageKind::Bridge),
        ("bridge-", ImageKind::Bridge),
    ] {
        assert!(check_name(bad, kind).is_err(), "{bad} as {kind:?}");
    }
    // Against the table: the name's chip is every target's chip, and a target is a bridge one.
    let table = ImageTable {
        kind: ImageKind::Bridge,
        features: Vec::new(),
        targets: vec![C3],
    };
    assert!(check_table("bridge-esp32c3", &table).is_ok());
    let e = check_table("bridge-esp32s3", &table).unwrap_err();
    assert!(
        e.contains("names the chip esp32s3, but it lists the target esp32c3"),
        "{e}"
    );
    let board = ImageTable {
        targets: boards(&[F1X0_64]),
        ..table
    };
    let e = check_table("bridge-esp32c3", &board).unwrap_err();
    assert!(e.contains("not a bridge target"), "{e}");
}

/// A manifest with the spec's two bridge entries and one board image, with `edit` applied.
fn bridge_manifest(edit: impl Fn(String) -> String) -> String {
    let h = "ab".repeat(32);
    let s = format!(
        r#"{{
  "commit": "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd",
  "built": "2026-11-01T17:03:58Z",
  "images": [
    {{ "name": "mainboard-gd32f130c8", "kind": "board", "file": "m.elf",
       "targets": [ {{ "family": "gd32f1x0", "flash_kib": 64 }} ],
       "guards": {{ "code_floor": 40000, "required_symbols": ["T main$"] }},
       "sha256": "{h}" }},
    {{ "name": "bridge-esp32c3", "kind": "bridge", "file": "bridge-esp32c3.bin",
       "targets": [ {{ "chip": "esp32c3" }} ], "sha256": "{h}" }},
    {{ "name": "bridge-esp32c3-install", "kind": "bridge-install",
       "file": "bridge-esp32c3-install.bin", "targets": [ {{ "chip": "esp32c3" }} ],
       "sha256": "{h}" }}
  ]
}}"#
    );
    edit(s)
}

#[test]
fn a_manifest_s_bridge_entries_are_checked() {
    let m = Manifest::parse(&bridge_manifest(|s| s)).unwrap();
    assert_eq!(m.images[1].kind, ImageKind::Bridge);
    assert_eq!(m.images[1].targets, vec![C3]);
    assert_eq!(m.images[2].kind, ImageKind::BridgeInstall);
    assert_eq!(m.images[2].guards, None);
    let edits: &[(&str, &str)] = &[
        // A bridge image with guards.
        (
            r#""targets": [ { "chip": "esp32c3" } ], "sha256""#,
            r#""targets": [ { "chip": "esp32c3" } ], "guards": { "code_floor": 1, "required_symbols": [] }, "sha256""#,
        ),
        // A board image without them.
        (
            r#""guards": { "code_floor": 40000, "required_symbols": ["T main$"] },"#,
            "",
        ),
        // A board target on a bridge image, and a bridge target on a board image.
        (
            r#"{ "chip": "esp32c3" }"#,
            r#"{ "family": "gd32f1x0", "flash_kib": 64 }"#,
        ),
        (
            r#"{ "family": "gd32f1x0", "flash_kib": 64 }"#,
            r#"{ "chip": "esp32c3" }"#,
        ),
        // An unknown chip, and a target with both sorts of field.
        (r#""chip": "esp32c3""#, r#""chip": "esp32s3""#),
        (
            r#"{ "chip": "esp32c3" }"#,
            r#"{ "chip": "esp32c3", "flash_kib": 64 }"#,
        ),
        // The install image named as the app image, and the install kind on the app image.
        (
            r#""name": "bridge-esp32c3-install""#,
            r#""name": "bridge-esp32c3x""#,
        ),
        (r#""kind": "bridge","#, r#""kind": "bridge-install","#),
    ];
    for (from, to) in edits {
        let text = bridge_manifest(|s| s.replacen(from, to, 1));
        assert_ne!(
            text,
            bridge_manifest(|s| s),
            "{from} is not in the manifest"
        );
        assert!(
            matches!(Manifest::parse(&text), Err(BundleError::Manifest(_))),
            "{from} -> {to} was accepted"
        );
    }
}

#[test]
fn a_bridge_image_is_chosen_by_the_esp_chip() {
    use espsvc::CHIP_ESP32C3;
    let b =
        Bundle::open(&with_bridge(&app_image(9_001, "0.2.0"), &install_image()).unwrap()).unwrap();
    let pick = |kind, chip| b.select_bridge(kind, chip).map(|i| i.name.as_str());
    assert_eq!(
        pick(ImageKind::Bridge, Some(CHIP_ESP32C3)),
        Ok("bridge-esp32c3")
    );
    assert_eq!(
        pick(ImageKind::BridgeInstall, Some(CHIP_ESP32C3)),
        Ok("bridge-esp32c3-install")
    );
    // An ESP_INFO that does not decode: the one image of the kind.
    assert_eq!(pick(ImageKind::Bridge, None), Ok("bridge-esp32c3"));
    assert_eq!(
        pick(ImageKind::Bridge, Some(9)),
        Err(SelectError::UnknownChip(9))
    );
    // The board selection is not confused by the bridge images.
    let (img, _) = b.select(&chip(FAMILY_F1X0, 64), None).unwrap();
    assert_eq!(img.name, "mainboard-gd32f130c8");
    assert_eq!(
        b.select(&chip(FAMILY_F1X0, 64), Some("bridge-esp32c3")),
        Err(SelectError::NotInBundle("bridge-esp32c3".into()))
    );
    // A bundle of board images only.
    let boards_only = fleet();
    let e = boards_only
        .select_bridge(ImageKind::Bridge, Some(CHIP_ESP32C3))
        .unwrap_err();
    assert_eq!(
        e,
        SelectError::NoBridgeImage {
            kind: ImageKind::Bridge,
            chip: Some(EspChip::Esp32c3),
            images: Vec::new(),
        }
    );
    assert_eq!(
        e.to_string(),
        "REFUSED: no bridge image in the bundle is for the chip (esp32c3)"
    );
    assert_eq!(
        boards_only
            .select_bridge(ImageKind::BridgeInstall, None)
            .unwrap_err()
            .to_string(),
        "REFUSED: the bundle has no bridge-install image"
    );
}

#[test]
fn a_bridge_image_is_told_from_a_bundle_by_its_content() {
    let app = app_image(9_001, "0.2.0");
    let inst = install_image();
    let z = with_bridge(&app, &inst).unwrap();
    assert!(is_bundle(&z));
    assert!(!is_bundle(&app));
    assert!(!is_bundle(&inst));
    // Both bridge images start with the ESP image magic; the install check tells them apart.
    assert_eq!(app[0], inst[0]);
    assert_eq!(
        bundle::check_bridge_file(ImageKind::Bridge, &app),
        Ok(Some([0, 2, 0]))
    );
    assert!(bundle::check_bridge_file(ImageKind::Bridge, &inst).is_err());
    assert_eq!(
        bundle::check_bridge_file(ImageKind::BridgeInstall, &inst),
        Ok(None)
    );
    assert!(bundle::check_bridge_file(ImageKind::BridgeInstall, &app).is_err());
    // A damaged bridge image in a bundle written without the packer is refused when it is used.
    let junk = vec![0x5Au8; 300];
    let raw = Bundle::open(&bundle_of(vec![(
        bridge_image("bridge-esp32c3", ImageKind::Bridge, &junk),
        junk.clone(),
    )]))
    .unwrap();
    let e = raw.bridge_file(&raw.manifest.images[0]).unwrap_err();
    assert!(
        e.contains("bridge-esp32c3: not an ESP32-C3 app image"),
        "{e}"
    );
}
