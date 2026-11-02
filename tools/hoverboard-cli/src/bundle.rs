//! The firmware bundle (`specs/firmware-bundle.md`): every image a build makes, the board images
//! and the bridge's two, in one zip archive (`.hbfw`) with a `manifest.json`, so `board update`,
//! `bridge update`, `bridge install` and `machine update` read what each device is and install the
//! image built for it.
//!
//! - [`is_bundle`] tells a bundle from a single file by the zip signature, not the extension;
//!   [`file_kind`] tells a bundle from an ELF.
//! - [`Bundle::open`] reads the archive and the manifest and checks each file's SHA-256 before
//!   anything parses it.
//! - [`Bundle::select`] picks the board image whose target equals the family and flash size
//!   `TARGET_ID` reports, takes `--image` among several, and refuses a match the bridge would
//!   refuse (a part in no row of the bridge's part table, `hoverboard_bridge_core::part`,
//!   section 3.4). [`Bundle::select_bridge`] picks a bridge image of a
//!   kind by the chip `ESP_INFO` reports (section 4.2).
//! - [`Bundle::prepare`] runs a board image's guards from the manifest and converts it to the
//!   binary; [`Bundle::bridge_file`] runs a bridge image's install check (section 3.3).
//! - [`pack`] builds a bundle from images and the facts [`image_table`] and [`elf::guards_for`]
//!   read, refusing an image its install command would refuse; [`pack_tree`] reads those from a
//!   checkout.
//! - [`check_name`] holds the name form (section 3.2) on a manifest's names, and [`check_table`]
//!   holds it against an image's own table when it is packed.

use std::io::{Cursor, Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;

use espsvc::{
    TargetId, CHIP_ESP32C3, FAMILY_F10X, FAMILY_F1X0, FAMILY_MM32SPIN05, FAMILY_UNSUPPORTED,
};
use hoverboard_bridge_core::part;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::elf::{self, ElfError, Guards};
use crate::install;

/// The manifest's path inside the archive.
pub const MANIFEST: &str = "manifest.json";

/// A board chip family, by the CLI's name for the `TargetId::family` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Family {
    /// [`FAMILY_F10X`].
    #[serde(rename = "gd32f10x")]
    Gd32f10x,
    /// [`FAMILY_F1X0`].
    #[serde(rename = "gd32f1x0")]
    Gd32f1x0,
    /// [`FAMILY_MM32SPIN05`].
    #[serde(rename = "mm32spin05")]
    Mm32spin05,
}

impl Family {
    /// The family `TargetId::family` reports, if it is one the CLI names.
    pub fn of(code: u8) -> Option<Family> {
        match code {
            FAMILY_F10X => Some(Family::Gd32f10x),
            FAMILY_F1X0 => Some(Family::Gd32f1x0),
            FAMILY_MM32SPIN05 => Some(Family::Mm32spin05),
            _ => None,
        }
    }

    /// The manifest's name for it.
    pub fn name(self) -> &'static str {
        match self {
            Family::Gd32f10x => "gd32f10x",
            Family::Gd32f1x0 => "gd32f1x0",
            Family::Mm32spin05 => "mm32spin05",
        }
    }
}

/// A bridge chip, by the CLI's name for the `EspInfo::chip` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EspChip {
    /// [`CHIP_ESP32C3`].
    #[serde(rename = "esp32c3")]
    Esp32c3,
}

impl EspChip {
    /// The chip `EspInfo::chip` reports, if it is one the CLI names.
    pub fn of(code: u8) -> Option<EspChip> {
        match code {
            CHIP_ESP32C3 => Some(EspChip::Esp32c3),
            _ => None,
        }
    }

    /// The manifest's name for it.
    pub fn name(self) -> &'static str {
        match self {
            EspChip::Esp32c3 => "esp32c3",
        }
    }
}

/// What an image is installed as (section 3.1, `images[].kind`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ImageKind {
    /// An ELF, flashed over SWD by `board update`.
    Board,
    /// An app image, for `bridge update`.
    Bridge,
    /// A merged install image, for `bridge install`.
    BridgeInstall,
}

impl ImageKind {
    pub fn name(self) -> &'static str {
        match self {
            ImageKind::Board => "board",
            ImageKind::Bridge => "bridge",
            ImageKind::BridgeInstall => "bridge-install",
        }
    }

    /// The extension of its file in the archive.
    fn extension(self) -> &'static str {
        match self {
            ImageKind::Board => "elf",
            ImageKind::Bridge | ImageKind::BridgeInstall => "bin",
        }
    }
}

/// The roles a board image's name may start with (section 3.2): what the board is, which the chip
/// cannot report.
const BOARD_ROLES: [&str; 2] = ["mainboard", "sideboard"];
/// A bridge image's role.
const BRIDGE_ROLE: &str = "bridge";
/// What an install image's name ends with.
const INSTALL_SUFFIX: &str = "-install";

/// Check `name` against the form of its `kind` (section 3.2): `<role>-<chip>` with a board role
/// for a board image, `bridge-<chip>` for an app image, `bridge-<chip>-install` for an install
/// image. The chip is lower-case letters and digits. Returns the chip.
pub fn check_name(name: &str, kind: ImageKind) -> Result<&str, String> {
    let bad = || {
        let form = match kind {
            ImageKind::Board => format!("<role>-<chip> (role {})", BOARD_ROLES.join(" or ")),
            ImageKind::Bridge => format!("{BRIDGE_ROLE}-<chip>"),
            ImageKind::BridgeInstall => format!("{BRIDGE_ROLE}-<chip>{INSTALL_SUFFIX}"),
        };
        format!(
            "image name {name} is not {form}, as a {} image is named (the chip lower-case letters \
             and digits)",
            kind.name()
        )
    };
    let stem = match kind {
        ImageKind::BridgeInstall => name.strip_suffix(INSTALL_SUFFIX).ok_or_else(bad)?,
        _ => name,
    };
    let (role, chip) = stem.split_once('-').ok_or_else(bad)?;
    let role_ok = match kind {
        ImageKind::Board => BOARD_ROLES.contains(&role),
        ImageKind::Bridge | ImageKind::BridgeInstall => role == BRIDGE_ROLE,
    };
    if !role_ok
        || chip.is_empty()
        || !chip
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit())
    {
        return Err(bad());
    }
    Ok(chip)
}

/// The targets a bridge image's name says it is for: every one a bridge target of the name's chip.
fn check_bridge_targets(name: &str, chip: &str, targets: &[Target]) -> Result<(), String> {
    for t in targets {
        match t {
            Target::Bridge(b) if b.chip.name() == chip => {}
            _ => {
                return Err(format!(
                    "image {name} names the chip {chip}, but it lists the target {t}: a bridge \
                     image's targets are its name's chip"
                ))
            }
        }
    }
    Ok(())
}

/// Check `name` against its own `table` (section 3.2), so the table stays the one owner of the
/// chip: a board image's chip must be its one chip feature, a bridge image's the chip of every
/// target. The targets must be of the kind's sort.
pub fn check_table(name: &str, table: &ImageTable) -> Result<(), String> {
    let chip = check_name(name, table.kind)?;
    check_targets(name, table.kind, &table.targets)?;
    match table.kind {
        ImageKind::Board => {
            let is_chip = |f: &&String| f.starts_with("gd32") || f.starts_with("mm32");
            let chips: Vec<&String> = table.features.iter().filter(is_chip).collect();
            if !(chips.len() == 1 && chips[0] == chip) {
                return Err(format!(
                    "image name {name} names the chip {chip}, but its table builds it with the \
                     features {:?}: the chip in the name is the one chip feature there",
                    table.features
                ));
            }
            Ok(())
        }
        ImageKind::Bridge | ImageKind::BridgeInstall => {
            check_bridge_targets(name, chip, &table.targets)
        }
    }
}

/// At least one target, each of the sort `kind` installs on: a board target for a board image, a
/// bridge target for a bridge image.
fn check_targets(name: &str, kind: ImageKind, targets: &[Target]) -> Result<(), String> {
    if targets.is_empty() {
        return Err(format!("image {name} has no targets"));
    }
    let board = kind == ImageKind::Board;
    if let Some(t) = targets
        .iter()
        .find(|t| matches!(t, Target::Board(_)) != board)
    {
        return Err(format!(
            "image {name} is a {} image, and {t} is not a {} target",
            kind.name(),
            if board { "board" } else { "bridge" }
        ));
    }
    Ok(())
}

/// A board chip a board image runs on. It matches when both fields equal what `TARGET_ID`
/// reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardTarget {
    pub family: Family,
    pub flash_kib: u16,
}

impl std::fmt::Display for BoardTarget {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} {} KiB", self.family.name(), self.flash_kib)
    }
}

/// The chip a bridge image runs on. It matches `ESP_INFO`'s chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BridgeTarget {
    pub chip: EspChip,
}

/// What an image runs on (section 3.1, `images[].targets`): a board target or a bridge target,
/// told apart by their fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Target {
    Board(BoardTarget),
    Bridge(BridgeTarget),
}

impl std::fmt::Display for Target {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Target::Board(t) => t.fmt(f),
            Target::Bridge(t) => f.write_str(t.chip.name()),
        }
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
    /// A board image's release guards; a bridge image has none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub guards: Option<Guards>,
    pub sha256: String,
}

impl Image {
    /// Whether it is for the board chip `chip`.
    fn is_for(&self, chip: BoardTarget) -> bool {
        self.targets.contains(&Target::Board(chip))
    }
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
    /// Parse and check a manifest: at least one image, unique names of their kind's form and
    /// unique files, every image with targets of its kind's sort (a bridge image's of its name's
    /// chip), guards on a board image and none on a bridge image, and a SHA-256 of 64 hex digits.
    pub fn parse(text: &str) -> Result<Manifest, BundleError> {
        let m: Manifest =
            serde_json::from_str(text).map_err(|e| BundleError::Manifest(e.to_string()))?;
        let bad = |e: String| Err(BundleError::Manifest(e));
        if m.images.is_empty() {
            return bad("no images".into());
        }
        for (i, img) in m.images.iter().enumerate() {
            let chip = match check_name(&img.name, img.kind) {
                Ok(c) => c,
                Err(e) => return bad(e),
            };
            if m.images[..i].iter().any(|o| o.name == img.name) {
                return bad(format!("image name {} given twice", img.name));
            }
            if m.images[..i].iter().any(|o| o.file == img.file) {
                return bad(format!("file {} given twice", img.file));
            }
            if let Err(e) = check_targets(&img.name, img.kind, &img.targets) {
                return bad(e);
            }
            let board = img.kind == ImageKind::Board;
            if !board {
                if let Err(e) = check_bridge_targets(&img.name, chip, &img.targets) {
                    return bad(e);
                }
            }
            if img.guards.is_some() != board {
                return bad(format!(
                    "image {} is a {} image: guards are {}",
                    img.name,
                    img.kind.name(),
                    if board {
                        "needed on a board image"
                    } else {
                        "for board images only"
                    }
                ));
            }
            if img.sha256.len() != 64 || !img.sha256.bytes().all(|b| b.is_ascii_hexdigit()) {
                return bad(format!("image {}: sha256 is not 64 hex digits", img.name));
            }
        }
        Ok(m)
    }
}

/// Whether `bytes` start with the zip local-header signature: a firmware bundle, whatever the
/// file is called (section 4).
pub fn is_bundle(bytes: &[u8]) -> bool {
    bytes.starts_with(b"PK\x03\x04")
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
    if is_bundle(bytes) {
        Ok(FileKind::Bundle)
    } else if bytes.starts_with(b"\x7fELF") {
        Ok(FileKind::Elf)
    } else {
        Err("neither a firmware bundle (a zip archive) nor an ELF".into())
    }
}

/// The install check of a bridge file of `kind` (section 3.3), as the command that installs it
/// runs it on a single file: an app image's version for `bridge update`, a whole merged image for
/// `bridge install`. The app image's version.
pub fn check_bridge_file(kind: ImageKind, bytes: &[u8]) -> Result<Option<[u8; 3]>, String> {
    match kind {
        ImageKind::Bridge => espsvc::app_version(bytes).map(Some).ok_or_else(|| {
            "not an ESP32-C3 app image (make one with espflash save-image)".to_string()
        }),
        ImageKind::BridgeInstall => install::check_image(bytes).map(|()| None),
        ImageKind::Board => unreachable!("a board image is an ELF"),
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
    /// `TARGET_ID` reports a part the bridge does not support ([`FAMILY_UNSUPPORTED`]), with what
    /// it read.
    UnsupportedPart { cpuid: u32, mcu_id: u32 },
    /// No board image has a target equal to the chip; every board image, with its targets.
    NoMatch {
        chip: BoardTarget,
        images: Vec<(String, Vec<Target>)>,
    },
    /// Several board images match and `--image` named none; each by name (which says its role).
    Several {
        chip: BoardTarget,
        images: Vec<String>,
    },
    /// `--image` names no board image in the bundle.
    NotInBundle(String),
    /// `--image` names an image that does not match the chip.
    NotForChip {
        name: String,
        chip: BoardTarget,
        targets: Vec<Target>,
    },
    /// The image matches, but the bridge does not flash the part.
    BridgeRefuses { name: String, chip: BoardTarget },
    /// `ESP_INFO` reports a chip code the CLI has no name for.
    UnknownChip(u8),
    /// No image of the kind is for the bridge chip (`None`: the chip is unknown, and the bundle
    /// has no image of the kind); every image of the kind, with its targets.
    NoBridgeImage {
        kind: ImageKind,
        chip: Option<EspChip>,
        images: Vec<(String, Vec<Target>)>,
    },
    /// The chip is unknown and the bundle has several images of the kind, by name.
    SeveralBridgeImages {
        kind: ImageKind,
        images: Vec<String>,
    },
}

impl std::fmt::Display for SelectError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let list = |f: &mut std::fmt::Formatter<'_>, images: &[(String, Vec<Target>)]| {
            for (n, t) in images {
                write!(f, "\n  {n}: {}", targets_text(t))?;
            }
            Ok(())
        };
        match self {
            SelectError::UnsupportedPart { cpuid, mcu_id } => write!(
                f,
                "REFUSED: a part the bridge does not support (cpuid {cpuid:#010x}, mcu id \
                 {mcu_id:#010x})"
            ),
            SelectError::UnknownFamily(c) => {
                write!(
                    f,
                    "REFUSED: TARGET_ID reports chip family {c}, which no image names"
                )
            }
            SelectError::NoMatch { chip, images } => {
                write!(
                    f,
                    "REFUSED: no board image in the bundle is for the chip ({chip}); it has:"
                )?;
                list(f, images)
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
            SelectError::NotInBundle(n) => {
                write!(f, "REFUSED: the bundle has no board image {n}")
            }
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
                "REFUSED: image {name} matches the chip ({chip}), but the bridge's part table \
                 has no row for it and the bridge would refuse a {chip} part"
            ),
            SelectError::UnknownChip(c) => {
                write!(
                    f,
                    "REFUSED: ESP_INFO reports chip {c}, which no image names"
                )
            }
            SelectError::NoBridgeImage { kind, chip, images } => {
                match chip {
                    Some(c) => write!(
                        f,
                        "REFUSED: no {} image in the bundle is for the chip ({})",
                        kind.name(),
                        c.name()
                    )?,
                    None => write!(f, "REFUSED: the bundle has no {} image", kind.name())?,
                }
                if images.is_empty() {
                    return Ok(());
                }
                write!(f, "; it has:")?;
                list(f, images)
            }
            SelectError::SeveralBridgeImages { kind, images } => write!(
                f,
                "REFUSED: the bridge's chip is unknown (its ESP_INFO did not decode), and the \
                 bundle has {} {} images: {}",
                images.len(),
                kind.name(),
                images.join(", ")
            ),
        }
    }
}

/// An opened bundle: the manifest, and each image's file as stored, SHA-256 checked.
#[derive(Debug)]
pub struct Bundle {
    pub manifest: Manifest,
    files: Vec<Vec<u8>>,
}

impl Bundle {
    /// Read the archive and the manifest, and check every listed file's SHA-256.
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
        let mut files = Vec::new();
        for img in &manifest.images {
            let file = read(&img.file)?.ok_or_else(|| BundleError::Missing(img.file.clone()))?;
            if hex(&Sha256::digest(&file)) != img.sha256.to_ascii_lowercase() {
                return Err(BundleError::Sha256(img.file.clone()));
            }
            files.push(file);
        }
        Ok(Bundle { manifest, files })
    }

    /// The file of `image`, as stored.
    pub fn file(&self, image: &Image) -> &[u8] {
        let i = self
            .manifest
            .images
            .iter()
            .position(|m| m.name == image.name)
            .expect("an image of this bundle");
        &self.files[i]
    }

    /// The board image for the chip `id` reports, and the target it matched; `want` is `--image`.
    pub fn select(
        &self,
        id: &TargetId,
        want: Option<&str>,
    ) -> Result<(&Image, BoardTarget), SelectError> {
        if id.family == FAMILY_UNSUPPORTED {
            return Err(SelectError::UnsupportedPart {
                cpuid: id.cpuid,
                mcu_id: id.mcu_id,
            });
        }
        let family = Family::of(id.family).ok_or(SelectError::UnknownFamily(id.family))?;
        let chip = BoardTarget {
            family,
            flash_kib: id.flash_kib,
        };
        let boards: Vec<&Image> = self
            .manifest
            .images
            .iter()
            .filter(|i| i.kind == ImageKind::Board)
            .collect();
        let chosen = match want {
            Some(w) => {
                let img = *boards
                    .iter()
                    .find(|i| i.name == w)
                    .ok_or_else(|| SelectError::NotInBundle(w.to_string()))?;
                if !img.is_for(chip) {
                    return Err(SelectError::NotForChip {
                        name: img.name.clone(),
                        chip,
                        targets: img.targets.clone(),
                    });
                }
                img
            }
            None => {
                let m: Vec<&Image> = boards.iter().copied().filter(|i| i.is_for(chip)).collect();
                match m[..] {
                    [one] => one,
                    [] => {
                        return Err(SelectError::NoMatch {
                            chip,
                            images: listed(&boards),
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
        if part::lookup(id.family, id.flash_kib).is_none() {
            return Err(SelectError::BridgeRefuses {
                name: chosen.name.clone(),
                chip,
            });
        }
        Ok((chosen, chip))
    }

    /// The image of `kind` (`bridge` or `bridge-install`) for the bridge chip `chip` (the code
    /// `ESP_INFO` reports), section 4.2 step 2. With no chip (an `ESP_INFO` that does not decode),
    /// the one image of the kind, or a refusal when there are several.
    pub fn select_bridge(&self, kind: ImageKind, chip: Option<u8>) -> Result<&Image, SelectError> {
        let chip = chip
            .map(|c| EspChip::of(c).ok_or(SelectError::UnknownChip(c)))
            .transpose()?;
        let of_kind: Vec<&Image> = self
            .manifest
            .images
            .iter()
            .filter(|i| i.kind == kind)
            .collect();
        let m: Vec<&Image> = match chip {
            Some(c) => of_kind
                .iter()
                .copied()
                .filter(|i| {
                    i.targets
                        .contains(&Target::Bridge(BridgeTarget { chip: c }))
                })
                .collect(),
            None => of_kind.clone(),
        };
        match m[..] {
            [one] => Ok(one),
            [] => Err(SelectError::NoBridgeImage {
                kind,
                chip,
                images: listed(&of_kind),
            }),
            _ => Err(SelectError::SeveralBridgeImages {
                kind,
                images: m.iter().map(|i| i.name.clone()).collect(),
            }),
        }
    }

    /// Run a board `image`'s guards from the manifest on its ELF, then convert it to the flash
    /// binary.
    pub fn prepare(&self, image: &Image) -> Result<Vec<u8>, ElfError> {
        let elf = self.file(image);
        let guards = image.guards.as_ref().expect("a board image has guards");
        elf::check(elf, guards)?;
        elf::to_bin(elf)
    }

    /// A bridge `image`'s file after its install check (section 3.3), and the app image's
    /// version.
    pub fn bridge_file(&self, image: &Image) -> Result<(&[u8], Option<[u8; 3]>), String> {
        let file = self.file(image);
        let version =
            check_bridge_file(image.kind, file).map_err(|e| format!("{}: {e}", image.name))?;
        Ok((file, version))
    }

    /// What `firmware info` prints: the commit, the build time, and per image its name, kind,
    /// targets and size: the binary it flashes for a board image, the file for a bridge image.
    pub fn describe(&self) -> String {
        let m = &self.manifest;
        let mut s = format!("commit  {}\nbuilt   {}\n", m.commit, m.built);
        for img in &m.images {
            let file = self.file(img);
            let size = match img.kind {
                ImageKind::Board => match elf::to_bin(file) {
                    Ok(b) => format!("{} B flashed", b.len()),
                    Err(e) => e.to_string(),
                },
                ImageKind::Bridge | ImageKind::BridgeInstall => format!("{} B", file.len()),
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

    /// The first 32 bits of the bundle's commit, as `ESP_INFO` reports a bridge's: the first eight
    /// hex digits.
    pub fn commit_word(&self) -> Option<u32> {
        let c = self.manifest.commit.get(..8)?;
        u32::from_str_radix(c, 16).ok()
    }
}

/// Each image of `images` with its targets, for a refusal.
fn listed(images: &[&Image]) -> Vec<(String, Vec<Target>)> {
    images
        .iter()
        .map(|i| (i.name.clone(), i.targets.clone()))
        .collect()
}

/// The line `board update` prints for the image it chose.
pub fn choice_line(m: &Manifest, img: &Image, chip: BoardTarget) -> String {
    format!(
        "image {} for the chip ({chip}), from the bundle built from commit {}",
        img.name, m.commit
    )
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// Write a bundle: the manifest, then each image's file (`files[i]` for `manifest.images[i]`),
/// deflated (section 3). [`pack`] is what checks them; this writes what it is given.
pub fn write(manifest: &Manifest, files: &[&[u8]]) -> Vec<u8> {
    assert_eq!(manifest.images.len(), files.len());
    let mut w = zip::ZipWriter::new(Cursor::new(Vec::new()));
    let opts = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated)
        .unix_permissions(0o644);
    let text = serde_json::to_string_pretty(manifest).expect("a manifest serialises");
    w.start_file(MANIFEST, opts).expect("in-memory zip");
    w.write_all(text.as_bytes()).expect("in-memory zip");
    w.write_all(b"\n").expect("in-memory zip");
    for (img, file) in manifest.images.iter().zip(files) {
        w.start_file(img.file.as_str(), opts)
            .expect("in-memory zip");
        w.write_all(file).expect("in-memory zip");
    }
    w.finish().expect("in-memory zip").into_inner()
}

/// An image's table in its crate's `Cargo.toml`, `[package.metadata.hoverboard-image.<name>]`.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImageTable {
    pub kind: ImageKind,
    /// The cargo features that build the image. The build reads them; the bundle does not carry
    /// them. A bridge image's are empty (section 3.3).
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
    check_table(name, &t)?;
    Ok(t)
}

/// One image to pack.
pub struct PackEntry<'a> {
    pub name: &'a str,
    pub table: ImageTable,
    /// A board image's guards; `None` for a bridge image.
    pub guards: Option<Guards>,
    /// The file as built: a board ELF with its debug sections, a bridge image as `espflash`
    /// wrote it.
    pub file: &'a [u8],
}

/// Build a bundle. A board ELF has its debug sections stripped, and is refused when the strip
/// changed its flashed bytes or when it fails its guards or the layout check; a bridge image is
/// refused when it fails its install check, and is stored as it is. Then the archive and the
/// manifest are written.
pub fn pack(entries: &[PackEntry], commit: &str, built: &str) -> Result<Vec<u8>, String> {
    if entries.is_empty() {
        return Err("firmware pack needs at least one image".into());
    }
    let mut images = Vec::new();
    let mut files = Vec::new();
    for (i, e) in entries.iter().enumerate() {
        if entries[..i].iter().any(|o| o.name == e.name) {
            return Err(format!("image {} given twice", e.name));
        }
        check_table(e.name, &e.table).map_err(|x| format!("{}: {x}", e.name))?;
        let kind = e.table.kind;
        let stored = match kind {
            ImageKind::Board => {
                let guards = e
                    .guards
                    .as_ref()
                    .ok_or_else(|| format!("{}: a board image needs its guards", e.name))?;
                let err = |x: ElfError| format!("{}: {x}", e.name);
                let stripped = elf::strip_debug(e.file).map_err(err)?;
                if elf::to_bin(&stripped).map_err(err)? != elf::to_bin(e.file).map_err(err)? {
                    return Err(format!(
                        "{}: removing the debug sections changed the flashed bytes",
                        e.name
                    ));
                }
                elf::check(&stripped, guards).map_err(err)?;
                stripped
            }
            ImageKind::Bridge | ImageKind::BridgeInstall => {
                if e.guards.is_some() {
                    return Err(format!("{}: a bridge image has no guards", e.name));
                }
                check_bridge_file(kind, e.file).map_err(|x| format!("{}: {x}", e.name))?;
                e.file.to_vec()
            }
        };
        images.push(Image {
            name: e.name.to_string(),
            kind,
            file: format!("{}.{}", e.name, kind.extension()),
            targets: e.table.targets.clone(),
            guards: e.guards.clone(),
            sha256: hex(&Sha256::digest(&stored)),
        });
        files.push(stored);
    }
    let manifest = Manifest {
        commit: commit.to_string(),
        built: built.to_string(),
        images,
    };
    let refs: Vec<&[u8]> = files.iter().map(Vec::as_slice).collect();
    Ok(write(&manifest, &refs))
}

/// One `firmware pack` argument, `<crate-dir>:<image-name>=<file>`.
#[derive(Debug, PartialEq, Eq)]
pub struct TreeEntry {
    pub crate_dir: PathBuf,
    pub name: String,
    pub file: PathBuf,
}

impl TreeEntry {
    /// Parse `<crate-dir>:<image-name>=<file>`. The image name is lower-case letters, digits and
    /// hyphens, and it is the last `:<name>=` in the argument, so a crate directory may hold a `:`.
    pub fn parse(arg: &str) -> Result<TreeEntry, String> {
        let re = regex::Regex::new(r"^(.+):([a-z0-9][a-z0-9-]*)=(.+)$").expect("a valid regex");
        let c = re
            .captures(arg)
            .ok_or_else(|| format!("{arg}: not <crate-dir>:<image-name>=<file>"))?;
        Ok(TreeEntry {
            crate_dir: PathBuf::from(&c[1]),
            name: c[2].to_string(),
            file: PathBuf::from(&c[3]),
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

/// `firmware pack` from a checkout: each entry's table from its crate's `Cargo.toml`, a board
/// image's guards from the checkout's `tools/image-guard-syms.sh`, the commit from
/// `git rev-parse HEAD`. A tree with changes to tracked files is refused unless `dirty`, and then
/// the commit is suffixed `-dirty`; untracked files do not count. Every crate must be in the same
/// checkout.
pub fn pack_tree(entries: &[TreeEntry], dirty: bool) -> Result<Vec<u8>, String> {
    let first = entries
        .first()
        .ok_or("firmware pack needs at least one <crate-dir>:<image-name>=<file>")?;
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
        let guards = match table.kind {
            ImageKind::Board => Some(elf::guards_for(&guard_file, &e.name)?),
            ImageKind::Bridge | ImageKind::BridgeInstall => None,
        };
        let file = std::fs::read(&e.file).map_err(|x| format!("{}: {x}", e.file.display()))?;
        loaded.push((table, guards, file));
    }
    let packs: Vec<PackEntry> = entries
        .iter()
        .zip(&loaded)
        .map(|(e, (table, guards, file))| PackEntry {
            name: &e.name,
            table: table.clone(),
            guards: guards.clone(),
            file,
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
