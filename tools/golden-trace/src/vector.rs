// SPDX-License-Identifier: GPL-3.0-or-later
//! Vector files: one operation per file, `vectors/<area>/<name>.toml`, with its vendor trace beside
//! it as `<name>.vendor.trace`.
//!
//! ```toml
//! # Why the mode, and why any narrowing (comments are the reasoning; the harness reads the keys).
//! description = """What the operation is."""
//! family = "gd32f10x"              # gd32f10x, gd32f1x0 or mm32spin05; the HAL chips follow from it
//! chips = ["gd32f103c8"]           # optional: a subset of the family's chips, with the reason
//! mode = "final_state"             # register_writes | final_state | with_polling
//! assert_only = ["<GPIOB_BASE>+0x00"]   # or ignore = [...]; at most one of the two
//!
//! [read_responses]                 # what a load from a status register returns
//! "<RCU_BASE>+0x00" = { or = 0x0200_0002 }   # the register's value with these bits set
//! "<SPI0_BASE>+0x08" = [0x02, 0x01]          # each read the next value; the last one sticks
//! "<ADC0_BASE>+0x4C" = 0x5D2                 # every read this value
//!
//! [reset_overrides]                # replaces the family table's reset seed for this run
//! "<RCU_BASE>+0x04" = 0
//!
//! [vendor]
//! body = '''C statements against the family's vendor library'''
//! prelude = '''optional C at file scope, e.g. #include "system_mm32spin0x_q.c"'''
//!
//! [hal]
//! body = '''Rust: a module body defining `pub fn body()` against the `hal` crate'''
//! # or, while the HAL module it needs does not exist yet:
//! pending = "the USART driver"
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::compare::{Mode, Scope};
use crate::target::{Family, Target};

/// A scripted read response.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Response {
    /// Every read returns this value.
    Value(u32),
    /// Each read returns the next value; the last one repeats.
    Sequence(Vec<u32>),
    /// Each read returns the register's current value with these bits set (the value is not
    /// stored, so the register keeps what software wrote).
    Or(u32),
}

/// The HAL side of a vector.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HalSide {
    /// A module body defining `pub fn body()`.
    Body(String),
    /// Not written yet: the HAL module it needs, named.
    Pending(String),
}

/// One vector.
#[derive(Debug, Clone)]
pub struct Vector {
    /// `<area>/<name>`, e.g. `clock/enable_adc0_f10x`.
    pub id: String,
    /// The vector file.
    pub path: PathBuf,
    /// The vendor family.
    pub family: Family,
    /// The HAL chips the vector runs on: the family's, or the subset the file names.
    pub chips: Vec<&'static str>,
    /// The comparison mode.
    pub mode: Mode,
    /// What the operation is.
    pub description: String,
    /// Scripted reads, by the address expression in the file.
    pub read_responses: BTreeMap<String, Response>,
    /// Reset seeds replaced for this run.
    pub reset_overrides: BTreeMap<String, u32>,
    /// `assert_only` entries, if any.
    pub assert_only: Vec<String>,
    /// `ignore` entries, if any.
    pub ignore: Vec<String>,
    /// The vendor body (C).
    pub vendor_body: String,
    /// C at file scope before the vendor body's function, if any: an `#include` of a vendor
    /// source whose static functions the body calls (the MM32's clock tree is a static function of
    /// LibSamples' `system_mm32spin0x_q.c`).
    pub vendor_prelude: Option<String>,
    /// The HAL side.
    pub hal: HalSide,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawVector {
    description: String,
    family: String,
    #[serde(default)]
    chips: Vec<String>,
    mode: String,
    #[serde(default)]
    assert_only: Vec<String>,
    #[serde(default)]
    ignore: Vec<String>,
    #[serde(default)]
    read_responses: BTreeMap<String, RawResponse>,
    #[serde(default)]
    reset_overrides: BTreeMap<String, u32>,
    vendor: RawVendor,
    hal: RawHal,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum RawResponse {
    Value(u32),
    Sequence(Vec<u32>),
    Or { or: u32 },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawVendor {
    body: String,
    prelude: Option<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawHal {
    body: Option<String>,
    pending: Option<String>,
}

impl Vector {
    /// Parse the vector file at `path`, whose id is `id`.
    pub fn parse(id: &str, path: &Path, text: &str) -> Result<Vector, String> {
        let at = |e: String| format!("{}: {e}", path.display());
        let raw: RawVector = toml::from_str(text).map_err(|e| at(e.to_string()))?;
        if !raw.assert_only.is_empty() && !raw.ignore.is_empty() {
            return Err(at("assert_only and ignore are exclusive".into()));
        }
        let mut read_responses = BTreeMap::new();
        for (k, v) in raw.read_responses {
            let r = match v {
                RawResponse::Value(x) => Response::Value(x),
                RawResponse::Sequence(s) if s.is_empty() => {
                    return Err(at(format!("read_responses {k}: an empty sequence")))
                }
                RawResponse::Sequence(s) => Response::Sequence(s),
                RawResponse::Or { or } => Response::Or(or),
            };
            read_responses.insert(k, r);
        }
        let hal = match (raw.hal.body, raw.hal.pending) {
            (Some(b), None) => HalSide::Body(b),
            (None, Some(p)) => HalSide::Pending(p),
            _ => return Err(at("[hal] needs exactly one of body and pending".into())),
        };
        let family = Family::from_name(&raw.family).map_err(at)?;
        let chips = if raw.chips.is_empty() {
            family.chips().to_vec()
        } else {
            raw.chips
                .iter()
                .map(|c| {
                    family
                        .chips()
                        .iter()
                        .find(|f| **f == c.as_str())
                        .copied()
                        .ok_or_else(|| at(format!("{c} is not a {} chip", family.name())))
                })
                .collect::<Result<_, _>>()?
        };
        Ok(Vector {
            id: id.to_string(),
            path: path.to_path_buf(),
            family,
            chips,
            mode: Mode::from_name(&raw.mode).map_err(at)?,
            description: raw.description,
            read_responses,
            reset_overrides: raw.reset_overrides,
            assert_only: raw.assert_only,
            ignore: raw.ignore,
            vendor_body: raw.vendor.body,
            vendor_prelude: raw.vendor.prelude,
            hal,
        })
    }

    /// The vendor trace file beside the vector.
    pub fn vendor_trace_path(&self) -> PathBuf {
        self.path.with_extension("vendor.trace")
    }

    /// The comparison scope, resolved against the family's table.
    pub fn scope(&self, target: &Target) -> Result<Scope, String> {
        let ranges = |list: &[String]| -> Result<Vec<(u32, u32)>, String> {
            list.iter()
                .map(|e| target.range(e).map_err(|err| format!("{}: {err}", self.id)))
                .collect()
        };
        Ok(if !self.assert_only.is_empty() {
            Scope::AssertOnly(ranges(&self.assert_only)?)
        } else if !self.ignore.is_empty() {
            Scope::Ignore(ranges(&self.ignore)?)
        } else {
            Scope::All
        })
    }

    /// The scripted reads by address.
    pub fn responses(&self, target: &Target) -> Result<Vec<(u32, Response)>, String> {
        self.read_responses
            .iter()
            .map(|(k, v)| {
                let addr = target
                    .resolve(k)
                    .map_err(|e| format!("{}: read_responses: {e}", self.id))?;
                Ok((addr, v.clone()))
            })
            .collect()
    }

    /// The reset overrides by address.
    pub fn overrides(&self, target: &Target) -> Result<Vec<(u32, u32)>, String> {
        self.reset_overrides
            .iter()
            .map(|(k, v)| {
                let addr = target
                    .resolve(k)
                    .map_err(|e| format!("{}: reset_overrides: {e}", self.id))?;
                Ok((addr, *v))
            })
            .collect()
    }
}

/// Every vector under `dir` (`<area>/<name>.toml`), sorted by id.
pub fn load_all(dir: &Path) -> Result<Vec<Vector>, String> {
    let mut out = Vec::new();
    let areas = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    for area in areas {
        let area = area.map_err(|e| e.to_string())?.path();
        if !area.is_dir() {
            continue;
        }
        let area_name = area
            .file_name()
            .and_then(|n| n.to_str())
            .ok_or("bad area name")?
            .to_string();
        for f in std::fs::read_dir(&area).map_err(|e| e.to_string())? {
            let path = f.map_err(|e| e.to_string())?.path();
            if path.extension().and_then(|e| e.to_str()) != Some("toml") {
                continue;
            }
            let stem = path
                .file_stem()
                .and_then(|n| n.to_str())
                .ok_or("bad vector name")?;
            let id = format!("{area_name}/{stem}");
            let text =
                std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            out.push(Vector::parse(&id, &path, &text)?);
        }
    }
    out.sort_by(|a, b| a.id.cmp(&b.id));
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MINIMAL: &str = r#"
description = "x"
family = "gd32f1x0"
mode = "with_polling"
assert_only = ["<RCU_BASE>+0x00"]
[read_responses]
"<RCU_BASE>+0x00" = { or = 0x2 }
"<RCU_BASE>+0x04" = [1, 2]
"<RCU_BASE>+0x08" = 7
[vendor]
body = "RCU_CTL0 |= 1U;"
[hal]
pending = "nothing"
"#;

    #[test]
    fn every_response_form_parses() {
        let v = Vector::parse("t/x", Path::new("t/x.toml"), MINIMAL).unwrap();
        assert_eq!(v.family, Family::Gd32f1x0);
        assert_eq!(v.chips, vec!["gd32f130c8"]);
        assert_eq!(v.mode, Mode::WithPolling);
        assert_eq!(v.read_responses["<RCU_BASE>+0x00"], Response::Or(2));
        assert_eq!(
            v.read_responses["<RCU_BASE>+0x04"],
            Response::Sequence(vec![1, 2])
        );
        assert_eq!(v.read_responses["<RCU_BASE>+0x08"], Response::Value(7));
        assert_eq!(v.hal, HalSide::Pending("nothing".into()));
        assert_eq!(v.vendor_trace_path(), Path::new("t/x.vendor.trace"));
    }

    #[test]
    fn contradictory_files_are_refused() {
        let both = MINIMAL.replace(
            "assert_only = [\"<RCU_BASE>+0x00\"]",
            "assert_only = [\"<RCU_BASE>\"]\nignore = [\"<RCU_BASE>\"]",
        );
        assert!(Vector::parse("t/x", Path::new("t/x.toml"), &both).is_err());
        let neither = MINIMAL.replace("pending = \"nothing\"", "");
        assert!(Vector::parse("t/x", Path::new("t/x.toml"), &neither).is_err());
        let typo = MINIMAL.replace("mode =", "mod =");
        assert!(Vector::parse("t/x", Path::new("t/x.toml"), &typo).is_err());
        let foreign = MINIMAL.replace("mode =", "chips = [\"gd32f103c8\"]\nmode =");
        assert!(Vector::parse("t/x", Path::new("t/x.toml"), &foreign).is_err());
    }

    /// Every shipped vector parses, resolves against its family's table, and has its vendor trace
    /// checked in beside it.
    #[test]
    fn the_shipped_vectors_are_complete() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let vectors = load_all(&dir.join("vectors")).unwrap();
        assert!(!vectors.is_empty());
        for v in &vectors {
            let t = Target::load(dir, v.family).unwrap();
            v.scope(&t).unwrap();
            v.responses(&t).unwrap();
            v.overrides(&t).unwrap();
            let trace = std::fs::read_to_string(v.vendor_trace_path())
                .unwrap_or_else(|e| panic!("{}: no vendor trace: {e}", v.id));
            crate::trace::parse(&trace, &t).unwrap_or_else(|e| panic!("{}: {e}", v.id));
        }
    }
}
