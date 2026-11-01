//! The firmware bundle (`specs/firmware-bundle.md` section 6, host tests): the manifest and the
//! name form, the selection against a `TargetId`, the SHA-256 check, bundle-or-ELF detection, the
//! guards taken from the manifest, and the packer.

mod common;

use common::{names_of, release_guards, Fixture};
use espsvc::{TargetId, FAMILY_F10X, FAMILY_F1X0};
use hoverboard_cli::bundle::{
    self, check_name, file_kind, image_table, pack, Bundle, BundleError, Family, FileKind, Image,
    ImageKind, ImageTable, Manifest, PackEntry, SelectError, Target, TreeEntry,
};
use hoverboard_cli::elf::{self, ElfError, Guards};
use sha2::{Digest, Sha256};

/// The universal image's table, from the crate that builds it.
const FIRMWARE_TOML: &str = include_str!("../../../motorcontroller/firmware/Cargo.toml");

fn chip(family: u8, flash_kib: u16) -> TargetId {
    TargetId {
        family,
        flash_kib,
        rdp: 0xA5,
        ..TargetId::default()
    }
}

fn t(family: Family, flash_kib: u16) -> Target {
    Target { family, flash_kib }
}

const F10X_64: Target = Target {
    family: Family::Gd32f10x,
    flash_kib: 64,
};
const F1X0_64: Target = Target {
    family: Family::Gd32f1x0,
    flash_kib: 64,
};

fn sha(b: &[u8]) -> String {
    Sha256::digest(b)
        .iter()
        .map(|x| format!("{x:02x}"))
        .collect()
}

/// An ELF that passes the universal image's guards.
fn release_elf() -> Vec<u8> {
    common::flashable_elf(&[0x11, 0x22, 0x33, 0x44])
}

/// A small image's guards and an ELF that passes them and fails the universal image's.
fn small() -> (Guards, Vec<u8>) {
    let g = Guards {
        code_floor: 2000,
        required_symbols: vec!["T main$".into(), "cortex_m_rt_main".into()],
        hot_symbols: None,
    };
    let elf = Fixture::new(&[0x55; 8], g.code_floor, names_of(&g)).build();
    (g, elf)
}

fn image(name: &str, targets: &[Target], guards: &Guards, elf: &[u8]) -> Image {
    Image {
        name: name.into(),
        kind: ImageKind::Board,
        file: format!("{name}.elf"),
        targets: targets.to_vec(),
        guards: guards.clone(),
        sha256: sha(elf),
    }
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
      "name": "mainboard-gd32",
      "kind": "board",
      "file": "mainboard-gd32.elf",
      "targets": [
        { "family": "gd32f10x", "flash_kib": 64 },
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
    assert_eq!(i.name, "mainboard-gd32");
    assert_eq!(i.file, "mainboard-gd32.elf");
    assert_eq!(i.kind, ImageKind::Board);
    assert_eq!(i.targets, vec![F10X_64, F1X0_64]);
    assert_eq!(i.guards.code_floor, 40_000);
    assert_eq!(i.guards.hot_symbols, Some(vec!["9run_shell".to_string()]));
    // An image with no hot-window gate leaves the key out.
    let m = Manifest::parse(&manifest_text(|s| {
        s.replace(",\n        \"hot_symbols\": [\"9run_shell\"]", "")
    }))
    .unwrap();
    assert_eq!(m.images[0].guards.hot_symbols, None);
}

#[test]
fn a_malformed_manifest_is_refused() {
    let edits: &[(&str, &str)] = &[
        ("\"commit\"", "\"version\": 1, \"commit\""), // an unknown key
        ("\"kind\": \"board\",", ""),                 // a missing key
        ("\"board\"", "\"controller\""),              // an unknown kind
        ("\"mainboard-gd32\",", "\"controller-gd32\","), // a name of no role
        ("\"mainboard-gd32\",", "\"mainboard_gd32\","), // not <role>-<chip>
        ("\"gd32f10x\"", "\"gd32f30x\""),             // an unknown family
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
        image("mainboard-gd32", &[F1X0_64], &release_guards(), &elf),
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
    let mut img = image("mainboard-gd32", &[F1X0_64], &release_guards(), &junk);
    img.sha256 = sha(b"what the packer stored");
    let z = bundle_of(vec![(img, junk)]);
    assert_eq!(
        Bundle::open(&z).unwrap_err(),
        BundleError::Sha256("mainboard-gd32.elf".into())
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

/// A bundle of three images: the universal one, and two F130 32 KiB images (a mainboard and a
/// sideboard), so the 32 KiB chip has several matches.
fn three() -> Bundle {
    let elf = release_elf();
    let (sg, selfm) = small();
    let f130_32 = t(Family::Gd32f1x0, 32);
    Bundle::open(&bundle_of(vec![
        (
            image(
                "mainboard-gd32",
                &[F10X_64, F1X0_64],
                &release_guards(),
                &elf,
            ),
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
    ]))
    .unwrap()
}

#[test]
fn selection_takes_the_one_image_for_the_chip() {
    let b = three();
    for (family, target) in [(FAMILY_F10X, F10X_64), (FAMILY_F1X0, F1X0_64)] {
        let (img, matched) = b.select(&chip(family, 64), None).unwrap();
        assert_eq!(img.name, "mainboard-gd32");
        assert_eq!(matched, target);
        // --image naming the match is the same choice.
        let (img, _) = b.select(&chip(family, 64), Some("mainboard-gd32")).unwrap();
        assert_eq!(img.name, "mainboard-gd32");
    }
    let line = bundle::choice_line(&b.manifest, &b.manifest.images[0], F1X0_64);
    assert_eq!(
        line,
        "image mainboard-gd32 for the chip (gd32f1x0 64 KiB), from the bundle built from commit \
         c0eb25917cd904d5d9ba9307f56cb8251cbb61dd"
    );
}

#[test]
fn selection_with_no_match_names_the_chip_and_every_image() {
    let b = three();
    let e = b.select(&chip(FAMILY_F10X, 128), None).unwrap_err();
    assert_eq!(
        e,
        SelectError::NoMatch {
            chip: t(Family::Gd32f10x, 128),
            images: vec![
                ("mainboard-gd32".into(), vec![F10X_64, F1X0_64]),
                ("mainboard-gd32f130c6".into(), vec![t(Family::Gd32f1x0, 32)]),
                ("sideboard-gd32f130c6".into(), vec![t(Family::Gd32f1x0, 32)]),
            ],
        }
    );
    let text = e.to_string();
    assert!(text.contains("(gd32f10x 128 KiB)"), "{text}");
    assert!(
        text.contains("mainboard-gd32: gd32f10x 64 KiB, gd32f1x0 64 KiB"),
        "{text}"
    );
    assert_eq!(
        b.select(&chip(7, 64), None).unwrap_err(),
        SelectError::UnknownFamily(7)
    );
}

#[test]
fn several_matches_need_image_and_list_them() {
    let b = three();
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
    let b = three();
    assert_eq!(
        b.select(&chip(FAMILY_F1X0, 64), Some("sideboard-gd32f130c6"))
            .unwrap_err(),
        SelectError::NotForChip {
            name: "sideboard-gd32f130c6".into(),
            chip: F1X0_64,
            targets: vec![t(Family::Gd32f1x0, 32)],
        }
    );
    assert_eq!(
        b.select(&chip(FAMILY_F1X0, 64), Some("mainboard-gd32f103c8"))
            .unwrap_err(),
        SelectError::NotInBundle("mainboard-gd32f103c8".into())
    );
}

#[test]
fn a_match_the_bridge_would_refuse_is_refused_naming_the_part() {
    let b = three();
    // --image resolves the several matches; the 32 KiB part is still one the bridge refuses.
    let e = b
        .select(&chip(FAMILY_F1X0, 32), Some("mainboard-gd32f130c6"))
        .unwrap_err();
    assert_eq!(
        e,
        SelectError::BridgeRefuses {
            name: "mainboard-gd32f130c6".into(),
            chip: t(Family::Gd32f1x0, 32),
        }
    );
    assert!(e.to_string().contains("gd32f1x0 32 KiB part"), "{e}");
}

#[test]
fn the_guards_come_from_the_manifest() {
    let b = three();
    let main = &b.manifest.images[0];
    let tiny = &b.manifest.images[1];
    // Each passes its own guards.
    assert_eq!(b.prepare(main).unwrap(), elf::to_bin(b.elf(main)).unwrap());
    b.prepare(tiny).unwrap();
    // The small image fails the universal image's guards (its floor), and the universal image
    // fails the small one's (a required symbol it does not carry).
    assert!(matches!(
        elf::check(b.elf(tiny), &main.guards),
        Err(ElfError::CodeFloor(_, 40_000))
    ));
    assert!(matches!(
        elf::check(b.elf(main), &tiny.guards),
        Err(ElfError::MissingSymbols(m)) if m == vec!["cortex_m_rt_main".to_string()]
    ));
    // The same ELF under another image's guards in the manifest is refused by prepare.
    let (sg, _) = small();
    let elf = b.elf(main).to_vec();
    let swapped = Bundle::open(&bundle_of(vec![(
        image("mainboard-gd32", &[F1X0_64], &sg, &elf),
        elf.clone(),
    )]))
    .unwrap();
    assert!(matches!(
        swapped.prepare(&swapped.manifest.images[0]),
        Err(ElfError::MissingSymbols(_))
    ));
}

#[test]
fn the_universal_image_table_is_in_its_cargo_toml() {
    let tbl = image_table(FIRMWARE_TOML, "mainboard-gd32").unwrap();
    assert_eq!(
        tbl,
        ImageTable {
            kind: ImageKind::Board,
            features: vec![],
            targets: vec![F10X_64, F1X0_64],
        }
    );
    assert!(image_table(FIRMWARE_TOML, "mainboard-gd32f103c8")
        .unwrap_err()
        .contains("no [package.metadata.hoverboard-image.mainboard-gd32f103c8]"));
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
    let table = image_table(FIRMWARE_TOML, "mainboard-gd32").unwrap();
    let z = pack(
        &[PackEntry {
            name: "mainboard-gd32",
            table,
            guards: g.clone(),
            elf: &built,
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
                name: "mainboard-gd32".into(),
                kind: ImageKind::Board,
                file: "mainboard-gd32.elf".into(),
                targets: vec![F10X_64, F1X0_64],
                guards: g,
                sha256: sha(&stored),
            }],
        }
    );
    let img = &b.manifest.images[0];
    assert_eq!(b.elf(img), &stored[..]);
    assert!(b.elf(img).len() < built.len());
    assert_eq!(b.prepare(img).unwrap(), elf::to_bin(&built).unwrap());
    let info = b.describe();
    assert!(info.starts_with("commit  c0eb2591"), "{info}");
    assert!(
        info.contains(
            "image   mainboard-gd32 (board): gd32f10x 64 KiB, gd32f1x0 64 KiB; 40000 B flashed"
        ),
        "{info}"
    );
}

#[test]
fn the_packer_refuses_an_elf_that_fails_its_guards() {
    let g = release_guards();
    let table = || image_table(FIRMWARE_TOML, "mainboard-gd32").unwrap();
    let entry = |elf: &'static [u8]| PackEntry {
        name: "mainboard-gd32",
        table: table(),
        guards: release_guards(),
        elf,
    };
    // A required symbol gone.
    let (mut text, bss) = names_of(&g);
    text.retain(|n| !n.contains("5probe3run"));
    let gutted: &'static [u8] = Fixture::new(&[], g.code_floor, (text, bss)).build().leak();
    let e = pack(&[entry(gutted)], "c", "b").unwrap_err();
    assert!(
        e.contains("mainboard-gd32: REFUSED: required symbol"),
        "{e}"
    );
    // A wfi in code.
    let mut f = Fixture::new(&[0x00, 0x20, 0x30, 0xBF], g.code_floor, names_of(&g));
    f.thumb = 4;
    let sleeper: &'static [u8] = f.build().leak();
    let e = pack(&[entry(sleeper)], "c", "b").unwrap_err();
    assert!(e.contains("'wfi'"), "{e}");
    // The small image's ELF under the universal image's name and guards.
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
        TreeEntry::parse("motorcontroller/firmware:mainboard-gd32=target/x/firmware").unwrap(),
        TreeEntry {
            crate_dir: "motorcontroller/firmware".into(),
            name: "mainboard-gd32".into(),
            elf: "target/x/firmware".into(),
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
        "mainboard-gd32",
    ] {
        assert_eq!(check_name(ok, None), Ok(()), "{ok}");
    }
    for bad in [
        "gd32f103c8-mainboard",
        "mainboard",
        "mainboard-",
        "controller-gd32f103c8",
        "mainboard-GD32F103C8",
        "mainboard-gd32f103c8-x",
    ] {
        assert!(check_name(bad, None).is_err(), "{bad}");
    }
    // Against the table: the one chip feature, or none for the universal image.
    let f = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    assert!(check_name("mainboard-gd32f103c8", Some(&f(&["gd32f103c8"]))).is_ok());
    assert!(check_name("mainboard-gd32f103c8", Some(&f(&["gd32f130c8"]))).is_err());
    assert!(check_name("mainboard-gd32f103c8", Some(&f(&[]))).is_err());
    assert!(check_name("mainboard-gd32", Some(&f(&[]))).is_ok());
    assert!(check_name("mainboard-gd32", Some(&f(&["gd32f103c8"]))).is_err());
}

#[test]
fn the_bundle_is_deflated() {
    let elf = release_elf();
    let z = bundle_of(vec![(
        image("mainboard-gd32", &[F1X0_64], &release_guards(), &elf),
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
