//! The firmware bundle (`specs/firmware-bundle.md`): every motor-controller image a build makes, in
//! one zip archive (`.hbfw`) with a `manifest.json`, so `board update` reads the chip on the board
//! and flashes the image built for it.
//!
//! - [`file_kind`] tells a bundle from an ELF by the first bytes, not the extension.
//! - [`Bundle::open`] reads the archive and the manifest and checks each ELF's SHA-256 before
//!   anything parses it.
//! - [`Bundle::select`] picks the image whose target equals the family and flash size `TARGET_ID`
//!   reports, takes `--image` among several, and refuses a match the bridge would refuse
//!   (`SUPPORTED_KIB`, section 3.4).
//! - [`Bundle::prepare`] runs the image's guards from the manifest and converts it to the binary.
//! - [`pack`] builds a bundle from ELFs and the facts [`image_table`] and [`elf::guards_for`] read,
//!   refusing an ELF `board update` would refuse; [`pack_tree`] reads those from a checkout.
//! - [`check_name`] holds the `<role>-<chip>` name form (section 3.2): on a manifest's names, and
//!   against an image's own table when it is packed.

use std::io::{Cursor, Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;

use espsvc::{TargetId, FAMILY_F10X, FAMILY_F1X0};
use hoverboard_bridge_core::flash::SUPPORTED_KIB;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::elf::{self, ElfError, Guards};

/// The manifest's path inside the archive.
pub const MANIFEST: &str = "manifest.json";

/// A chip family, by the CLI's name for the `TargetId::family` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Family {
    /// [`FAMILY_F10X`].
    #[serde(rename = "gd32f10x")]
    Gd32f10x,
    /// [`FAMILY_F1X0`].
    #[serde(rename = "gd32f1x0")]
    Gd32f1x0,
}

impl Family {
    /// The family `TargetId::family` reports, if it is one the CLI names.
    pub fn of(code: u8) -> Option<Family> {
        match code {
            FAMILY_F10X => Some(Family::Gd32f10x),
            FAMILY_F1X0 => Some(Family::Gd32f1x0),
            _ => None,
        }
    }

    /// The manifest's name for it.
    pub fn name(self) -> &'static str {
        match self {
            Family::Gd32f10x => "gd32f10x",
            Family::Gd32f1x0 => "gd32f1x0",
        }
    }
}

/// What an image is installed as (section 3.1, `images[].kind`). Board images are the ones a bundle
/// carries so far; the bridge kinds join with the bridge images.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ImageKind {
    /// Flashed over SWD by `board update`.
    Board,
}

impl ImageKind {
    pub fn name(self) -> &'static str {
        match self {
            ImageKind::Board => "board",
        }
    }
}

/// The roles a board image's name may start with (section 3.2): what the board is, which the chip
/// cannot report.
const ROLES: [&str; 2] = ["mainboard", "sideboard"];

/// Check `name` against the `<role>-<chip>` form (section 3.2). With `features` (the image's own
/// table, when it is packed), the chip must be its one chip feature, so the table stays the one
/// owner of the chip.
pub fn check_name(name: &str, features: Option<&[String]>) -> Result<(), String> {
    let bad = || {
        format!(
            "image name {name} is not <role>-<chip> (role {}; chip a hal chip feature, \
             lower-case letters and digits)",
            ROLES.join(" or ")
        )
    };
    let (role, chip) = name.split_once('-').ok_or_else(bad)?;
    if !ROLES.contains(&role)
        || chip.is_empty()
        || !chip
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit())
    {
        return Err(bad());
    }
    if let Some(features) = features {
        let is_chip = |f: &&String| f.starts_with("gd32") || f.starts_with("mm32");
        let chips: Vec<&String> = features.iter().filter(is_chip).collect();
        if !(chips.len() == 1 && chips[0] == chip) {
            return Err(format!(
                "image name {name} names the chip {chip}, but its table builds it with the \
                 features {features:?}: the chip in the name is the one chip feature there"
            ));
        }
    }
    Ok(())
}

/// A chip an image runs on. A chip matches when both fields equal what `TARGET_ID` reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Target {
    pub family: Family,
    pub flash_kib: u16,
}

impl std::fmt::Display for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} {} KiB", self.family.name(), self.flash_kib)
    }
}

fn targets_text(t: &[Target]) -> String {
    t.iter()
        .map(Target::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

/// One image in the manifest.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Image {
    pub name: String,
    pub kind: ImageKind,
    pub file: String,
    pub targets: Vec<Target>,
    pub guards: Guards,
    pub sha256: String,
}

/// `manifest.json`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub commit: String,
    pub built: String,
    pub images: Vec<Image>,
}

impl Manifest {
    /// Parse and check a manifest: at least one image, unique names of the `<role>-<chip>` form
    /// and unique files, every image with a target and a SHA-256 of 64 hex digits.
    pub fn parse(text: &str) -> Result<Manifest, BundleError> {
        let m: Manifest =
            serde_json::from_str(text).map_err(|e| BundleError::Manifest(e.to_string()))?;
        let bad = |e: String| Err(BundleError::Manifest(e));
        if m.images.is_empty() {
            return bad("no images".into());
        }
        for (i, img) in m.images.iter().enumerate() {
            if let Err(e) = check_name(&img.name, None) {
                return bad(e);
            }
            if m.images[..i].iter().any(|o| o.name == img.name) {
                return bad(format!("image name {} given twice", img.name));
            }
            if m.images[..i].iter().any(|o| o.file == img.file) {
                return bad(format!("file {} given twice", img.file));
            }
            if img.targets.is_empty() {
                return bad(format!("image {} has no targets", img.name));
            }
            if img.sha256.len() != 64 || !img.sha256.bytes().all(|b| b.is_ascii_hexdigit()) {
                return bad(format!("image {}: sha256 is not 64 hex digits", img.name));
            }
        }
        Ok(m)
    }
}

/// What a file given to `board flash` or `board update` is, by its first bytes (section 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileKind {
    /// A zip archive: a firmware bundle.
    Bundle,
    /// An ELF.
    Elf,
}

/// Tell a bundle (the zip local-header signature) from an ELF (its magic).
pub fn file_kind(bytes: &[u8]) -> Result<FileKind, String> {
    if bytes.starts_with(b"PK\x03\x04") {
        Ok(FileKind::Bundle)
    } else if bytes.starts_with(b"\x7fELF") {
        Ok(FileKind::Elf)
    } else {
        Err("neither a firmware bundle (a zip archive) nor an ELF".into())
    }
}

/// Why a bundle is refused when it is opened.
#[derive(Debug, PartialEq, Eq)]
pub enum BundleError {
    /// Not a zip archive this tool reads.
    Archive(String),
    /// The manifest is absent or does not parse.
    Manifest(String),
    /// A file the manifest lists is not in the archive.
    Missing(String),
    /// A file's SHA-256 is not the manifest's.
    Sha256(String),
}

impl std::fmt::Display for BundleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BundleError::Archive(e) => write!(f, "not a readable firmware bundle: {e}"),
            BundleError::Manifest(e) => write!(f, "REFUSED: the bundle's {MANIFEST}: {e}"),
            BundleError::Missing(n) => {
                write!(f, "REFUSED: {n} is in the manifest but not in the bundle")
            }
            BundleError::Sha256(n) => write!(
                f,
                "REFUSED: {n}'s SHA-256 is not the one the manifest gives (a damaged bundle?)"
            ),
        }
    }
}

/// Why no image is chosen for the chip.
#[derive(Debug, PartialEq, Eq)]
pub enum SelectError {
    /// `TARGET_ID` reports a family code the CLI has no name for.
    UnknownFamily(u8),
    /// No image has a target equal to the chip; every image, with its targets.
    NoMatch {
        chip: Target,
        images: Vec<(String, Vec<Target>)>,
    },
    /// Several images match and `--image` named none; each by name (which says its role).
    Several { chip: Target, images: Vec<String> },
    /// `--image` names no image in the bundle.
    NotInBundle(String),
    /// `--image` names an image that does not match the chip.
    NotForChip {
        name: String,
        chip: Target,
        targets: Vec<Target>,
    },
    /// The image matches, but the bridge does not flash the part.
    BridgeRefuses { name: String, chip: Target },
}

impl std::fmt::Display for SelectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SelectError::UnknownFamily(c) => {
                write!(
                    f,
                    "REFUSED: TARGET_ID reports chip family {c}, which no image names"
                )
            }
            SelectError::NoMatch { chip, images } => {
                write!(
                    f,
                    "REFUSED: no image in the bundle is for the chip ({chip}); it has:"
                )?;
                for (n, t) in images {
                    write!(f, "\n  {n}: {}", targets_text(t))?;
                }
                Ok(())
            }
            SelectError::Several { chip, images } => {
                write!(
                    f,
                    "REFUSED: {} images are for the chip ({chip}); name one with --image:",
                    images.len()
                )?;
                for n in images {
                    write!(f, "\n  {n}")?;
                }
                Ok(())
            }
            SelectError::NotInBundle(n) => write!(f, "REFUSED: the bundle has no image {n}"),
            SelectError::NotForChip {
                name,
                chip,
                targets,
            } => write!(
                f,
                "REFUSED: image {name} is for {}, not the chip ({chip}); --image chooses among \
                 the images that match, it does not override the match",
                targets_text(targets)
            ),
            SelectError::BridgeRefuses { name, chip } => write!(
                f,
                "REFUSED: image {name} matches the chip ({chip}), but the bridge flashes only \
                 {SUPPORTED_KIB} KiB parts and would refuse a {chip} part"
            ),
        }
    }
}

/// An opened bundle: the manifest, and each image's ELF as stored, SHA-256 checked.
#[derive(Debug)]
pub struct Bundle {
    pub manifest: Manifest,
    elfs: Vec<Vec<u8>>,
}

impl Bundle {
    /// Read the archive and the manifest, and check every listed ELF's SHA-256.
    pub fn open(bytes: &[u8]) -> Result<Bundle, BundleError> {
        let mut zip = zip::ZipArchive::new(Cursor::new(bytes))
            .map_err(|e| BundleError::Archive(e.to_string()))?;
        let mut read = |name: &str| -> Result<Option<Vec<u8>>, BundleError> {
            let mut f = match zip.by_name(name) {
                Ok(f) => f,
                Err(zip::result::ZipError::FileNotFound) => return Ok(None),
                Err(e) => return Err(BundleError::Archive(format!("{name}: {e}"))),
            };
            let mut v = Vec::new();
            f.read_to_end(&mut v)
                .map_err(|e| BundleError::Archive(format!("{name}: {e}")))?;
            Ok(Some(v))
        };
        let text = read(MANIFEST)?.ok_or_else(|| BundleError::Manifest("absent".into()))?;
        let text =
            String::from_utf8(text).map_err(|_| BundleError::Manifest("not UTF-8 text".into()))?;
        let manifest = Manifest::parse(&text)?;
        let mut elfs = Vec::new();
        for img in &manifest.images {
            let elf = read(&img.file)?.ok_or_else(|| BundleError::Missing(img.file.clone()))?;
            if hex(&Sha256::digest(&elf)) != img.sha256.to_ascii_lowercase() {
                return Err(BundleError::Sha256(img.file.clone()));
            }
            elfs.push(elf);
        }
        Ok(Bundle { manifest, elfs })
    }

    /// The ELF of `image`, as stored.
    pub fn elf(&self, image: &Image) -> &[u8] {
        let i = self
            .manifest
            .images
            .iter()
            .position(|m| m.name == image.name)
            .expect("an image of this bundle");
        &self.elfs[i]
    }

    /// The image for the chip `id` reports, and the target it matched; `want` is `--image`.
    pub fn select(
        &self,
        id: &TargetId,
        want: Option<&str>,
    ) -> Result<(&Image, Target), SelectError> {
        let family = Family::of(id.family).ok_or(SelectError::UnknownFamily(id.family))?;
        let chip = Target {
            family,
            flash_kib: id.flash_kib,
        };
        let images = &self.manifest.images;
        let matches = |i: &&Image| i.targets.contains(&chip);
        let chosen = match want {
            Some(w) => {
                let img = images
                    .iter()
                    .find(|i| i.name == w)
                    .ok_or_else(|| SelectError::NotInBundle(w.to_string()))?;
                if !matches(&img) {
                    return Err(SelectError::NotForChip {
                        name: img.name.clone(),
                        chip,
                        targets: img.targets.clone(),
                    });
                }
                img
            }
            None => {
                let m: Vec<&Image> = images.iter().filter(matches).collect();
                match m[..] {
                    [one] => one,
                    [] => {
                        return Err(SelectError::NoMatch {
                            chip,
                            images: images
                                .iter()
                                .map(|i| (i.name.clone(), i.targets.clone()))
                                .collect(),
                        })
                    }
                    _ => {
                        return Err(SelectError::Several {
                            chip,
                            images: m.iter().map(|i| i.name.clone()).collect(),
                        })
                    }
                }
            }
        };
        if chip.flash_kib != SUPPORTED_KIB {
            return Err(SelectError::BridgeRefuses {
                name: chosen.name.clone(),
                chip,
            });
        }
        Ok((chosen, chip))
    }

    /// Run `image`'s guards from the manifest on its ELF, then convert it to the flash binary.
    pub fn prepare(&self, image: &Image) -> Result<Vec<u8>, ElfError> {
        let elf = self.elf(image);
        elf::check(elf, &image.guards)?;
        elf::to_bin(elf)
    }

    /// What `firmware info` prints: the commit, the build time, and per image its name, kind,
    /// targets and the size of the binary it flashes.
    pub fn describe(&self) -> String {
        let m = &self.manifest;
        let mut s = format!("commit  {}\nbuilt   {}\n", m.commit, m.built);
        for img in &m.images {
            let size = match elf::to_bin(self.elf(img)) {
                Ok(b) => format!("{} B flashed", b.len()),
                Err(e) => e.to_string(),
            };
            s.push_str(&format!(
                "image   {} ({}): {}; {size}\n",
                img.name,
                img.kind.name(),
                targets_text(&img.targets)
            ));
        }
        s
    }
}

/// The line `board update` prints for the image it chose.
pub fn choice_line(m: &Manifest, img: &Image, chip: Target) -> String {
    format!(
        "image {} for the chip ({chip}), from the bundle built from commit {}",
        img.name, m.commit
    )
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Write a bundle: the manifest, then each image's ELF (`elfs[i]` for `manifest.images[i]`),
/// deflated (section 3). [`pack`] is what checks them; this writes what it is given.
pub fn write(manifest: &Manifest, elfs: &[&[u8]]) -> Vec<u8> {
    assert_eq!(manifest.images.len(), elfs.len());
    let mut w = zip::ZipWriter::new(Cursor::new(Vec::new()));
    let opts = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated)
        .unix_permissions(0o644);
    let text = serde_json::to_string_pretty(manifest).expect("a manifest serialises");
    w.start_file(MANIFEST, opts).expect("in-memory zip");
    w.write_all(text.as_bytes()).expect("in-memory zip");
    w.write_all(b"\n").expect("in-memory zip");
    for (img, elf) in manifest.images.iter().zip(elfs) {
        w.start_file(img.file.as_str(), opts)
            .expect("in-memory zip");
        w.write_all(elf).expect("in-memory zip");
    }
    w.finish().expect("in-memory zip").into_inner()
}

/// An image's table in its crate's `Cargo.toml`, `[package.metadata.hoverboard-image.<name>]`.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImageTable {
    pub kind: ImageKind,
    /// The cargo features that build the image. The build reads them; the bundle does not carry
    /// them.
    pub features: Vec<String>,
    pub targets: Vec<Target>,
}

/// The table `cargo_toml` (a `Cargo.toml`'s text) gives `name`, with the name's form checked
/// against it (section 3.2).
pub fn image_table(cargo_toml: &str, name: &str) -> Result<ImageTable, String> {
    let doc: toml::Table = toml::from_str(cargo_toml).map_err(|e| format!("Cargo.toml: {e}"))?;
    let v = doc
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("hoverboard-image"))
        .and_then(|h| h.get(name))
        .ok_or_else(|| format!("Cargo.toml has no [package.metadata.hoverboard-image.{name}]"))?;
    let t: ImageTable = v
        .clone()
        .try_into()
        .map_err(|e| format!("[package.metadata.hoverboard-image.{name}]: {e}"))?;
    if t.targets.is_empty() {
        return Err(format!(
            "[package.metadata.hoverboard-image.{name}] has no targets"
        ));
    }
    check_name(name, Some(&t.features))?;
    Ok(t)
}

/// One image to pack.
pub struct PackEntry<'a> {
    pub name: &'a str,
    pub table: ImageTable,
    pub guards: Guards,
    /// The ELF as built (with its debug sections).
    pub elf: &'a [u8],
}

/// Build a bundle: strip each ELF's debug sections, refuse one whose flashed bytes the strip
/// changed or that fails its guards or the layout check, and write the archive and the manifest.
pub fn pack(entries: &[PackEntry], commit: &str, built: &str) -> Result<Vec<u8>, String> {
    if entries.is_empty() {
        return Err("firmware pack needs at least one image".into());
    }
    let mut images = Vec::new();
    let mut elfs = Vec::new();
    for (i, e) in entries.iter().enumerate() {
        if entries[..i].iter().any(|o| o.name == e.name) {
            return Err(format!("image {} given twice", e.name));
        }
        let err = |x: ElfError| format!("{}: {x}", e.name);
        let stripped = elf::strip_debug(e.elf).map_err(err)?;
        if elf::to_bin(&stripped).map_err(err)? != elf::to_bin(e.elf).map_err(err)? {
            return Err(format!(
                "{}: removing the debug sections changed the flashed bytes",
                e.name
            ));
        }
        elf::check(&stripped, &e.guards).map_err(err)?;
        check_name(e.name, Some(&e.table.features)).map_err(|x| format!("{}: {x}", e.name))?;
        images.push(Image {
            name: e.name.to_string(),
            kind: e.table.kind,
            file: format!("{}.elf", e.name),
            targets: e.table.targets.clone(),
            guards: e.guards.clone(),
            sha256: hex(&Sha256::digest(&stripped)),
        });
        elfs.push(stripped);
    }
    let manifest = Manifest {
        commit: commit.to_string(),
        built: built.to_string(),
        images,
    };
    let refs: Vec<&[u8]> = elfs.iter().map(Vec::as_slice).collect();
    Ok(write(&manifest, &refs))
}

/// One `firmware pack` argument, `<crate-dir>:<image-name>=<elf>`.
#[derive(Debug, PartialEq, Eq)]
pub struct TreeEntry {
    pub crate_dir: PathBuf,
    pub name: String,
    pub elf: PathBuf,
}

impl TreeEntry {
    /// Parse `<crate-dir>:<image-name>=<elf>`. The image name is lower-case letters, digits and
    /// hyphens, and it is the last `:<name>=` in the argument, so a crate directory may hold a `:`.
    pub fn parse(arg: &str) -> Result<TreeEntry, String> {
        let re = regex::Regex::new(r"^(.+):([a-z0-9][a-z0-9-]*)=(.+)$").expect("a valid regex");
        let c = re
            .captures(arg)
            .ok_or_else(|| format!("{arg}: not <crate-dir>:<image-name>=<elf>"))?;
        Ok(TreeEntry {
            crate_dir: PathBuf::from(&c[1]),
            name: c[2].to_string(),
            elf: PathBuf::from(&c[3]),
        })
    }
}

fn git(dir: &Path, args: &[&str]) -> Result<String, String> {
    let o = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(args)
        .output()
        .map_err(|e| format!("git: {e}"))?;
    if !o.status.success() {
        return Err(format!(
            "git {} in {}: {}",
            args.join(" "),
            dir.display(),
            String::from_utf8_lossy(&o.stderr).trim()
        ));
    }
    Ok(String::from_utf8_lossy(&o.stdout).trim().to_string())
}

/// `firmware pack` from a checkout: each entry's table from its crate's `Cargo.toml`, its guards
/// from the checkout's `tools/image-guard-syms.sh`, the commit from `git rev-parse HEAD`. A tree
/// with changes to tracked files is refused unless `dirty`, and then the commit is suffixed
/// `-dirty`; untracked files do not count. Every crate must be in the same checkout.
pub fn pack_tree(entries: &[TreeEntry], dirty: bool) -> Result<Vec<u8>, String> {
    let first = entries
        .first()
        .ok_or("firmware pack needs at least one <crate-dir>:<image-name>=<elf>")?;
    let root = PathBuf::from(git(&first.crate_dir, &["rev-parse", "--show-toplevel"])?);
    for e in entries {
        let r = PathBuf::from(git(&e.crate_dir, &["rev-parse", "--show-toplevel"])?);
        if r != root {
            return Err(format!(
                "{} is in {}, not {}: a bundle is built from one checkout",
                e.crate_dir.display(),
                r.display(),
                root.display()
            ));
        }
    }
    let mut commit = git(&root, &["rev-parse", "HEAD"])?;
    if !git(&root, &["status", "--porcelain", "--untracked-files=no"])?.is_empty() {
        if !dirty {
            return Err(format!(
                "{} has uncommitted changes; commit them, or give --dirty",
                root.display()
            ));
        }
        commit.push_str("-dirty");
    }
    let guard_path = root.join("tools/image-guard-syms.sh");
    let guard_file = std::fs::read_to_string(&guard_path)
        .map_err(|e| format!("{}: {e}", guard_path.display()))?;
    let mut loaded = Vec::new();
    for e in entries {
        let toml_path = e.crate_dir.join("Cargo.toml");
        let text = std::fs::read_to_string(&toml_path)
            .map_err(|x| format!("{}: {x}", toml_path.display()))?;
        let table =
            image_table(&text, &e.name).map_err(|x| format!("{}: {x}", toml_path.display()))?;
        let guards = elf::guards_for(&guard_file, &e.name)?;
        let elf = std::fs::read(&e.elf).map_err(|x| format!("{}: {x}", e.elf.display()))?;
        loaded.push((table, guards, elf));
    }
    let packs: Vec<PackEntry> = entries
        .iter()
        .zip(&loaded)
        .map(|(e, (table, guards, elf))| PackEntry {
            name: &e.name,
            table: table.clone(),
            guards: guards.clone(),
            elf,
        })
        .collect();
    pack(&packs, &commit, &utc_now())
}

/// The current UTC time as `YYYY-MM-DDTHH:MM:SSZ`.
pub fn utc_now() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    utc(secs)
}

/// `secs` since the Unix epoch as `YYYY-MM-DDTHH:MM:SSZ` (days to a civil date by Howard Hinnant's
/// `civil_from_days`).
pub fn utc(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}
