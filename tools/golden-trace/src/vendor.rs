// SPDX-License-Identifier: GPL-3.0-or-later
//! The vendor side: a vector's C body compiled against its family's vendor library with
//! arm-none-eabi-gcc, and the trace file it renders to.
//!
//! The GD32 families take GigaDevice's standard peripheral libraries (`third_party/gd32-spl/`,
//! committed); the MM32SPIN05 takes MindMotion's LibSamples V1.13, which may not be redistributed,
//! so `tools/fetch-vendor.sh` downloads and verifies it into the gitignored
//! `third_party/mm32-libsamples/`, and this builder refuses an MM32 vector without that verified
//! copy (specs/hal.md, section 8.1).
//!
//! The body is wrapped in `gt_entry`, compiled with the family's defines, and linked with every
//! library source of the family (`--gc-sections` keeps what the body calls). Nothing else runs:
//! no startup code, no `SystemInit`, so the trace is the body's accesses alone.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::target::Family;

/// The Arm GNU Toolchain release the checked-in vendor traces are generated with. A regeneration
/// with another compiler is refused, so a trace difference is never a compiler difference.
pub const PINNED_GCC: &str = "Arm GNU Toolchain 15.2.Rel1";

/// The compile flags of the Cortex-M3 GD32 families. `-std=gnu11` because the SPL typedefs
/// `bool`; `-ffunction-sections` so the link keeps only what the body reaches.
pub const CFLAGS: &[&str] = &[
    "-Os",
    "-mcpu=cortex-m3",
    "-mthumb",
    "-std=gnu11",
    "-ffreestanding",
    "-fno-common",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall",
    "-Wno-unused-function",
];

/// The compile flags of the Cortex-M0 MM32SPIN05: [`CFLAGS`] for the M0 core.
pub const CFLAGS_M0: &[&str] = &[
    "-Os",
    "-mcpu=cortex-m0",
    "-mthumb",
    "-std=gnu11",
    "-ffreestanding",
    "-fno-common",
    "-ffunction-sections",
    "-fdata-sections",
    "-Wall",
    "-Wno-unused-function",
];

/// The entry symbol the wrapper defines.
pub const ENTRY: &str = "gt_entry";

/// The SPL's commit, as `third_party/gd32-spl/SOURCE` records it.
pub const SPL_COMMIT: &str = "4ead0d8a549820330ebb5e03907fb8bb2e6ac5b5";

/// The SHA-256 of `LibSamples_MM32SPIN0x_q_V1.13.zip`, the one `tools/fetch-vendor.sh` checks and
/// writes to `third_party/mm32-libsamples/SHA256` after a verified unpack.
pub const LIBSAMPLES_SHA256: &str =
    "eb294c20129a474daf6295e03f42667f14c4826b1639c2afbf3bad3c2e4a8cbd";

/// How one family's library is built.
struct Layout {
    /// The library root, relative to the repository.
    root: &'static str,
    /// The source directory, relative to `root`.
    sources: &'static str,
    /// The defines the library needs.
    defines: &'static [&'static str],
    /// Include directories, relative to `root`.
    includes: &'static [&'static str],
    /// Whether the harness's own `vendor/<family>/` header directory goes first (the SPL expects
    /// the application to supply its libopt header).
    harness_includes: bool,
    /// The header the generated body includes.
    header: &'static str,
    /// The compile flags.
    cflags: &'static [&'static str],
}

fn layout(family: Family) -> Layout {
    match family {
        // The medium-density define (the GD32F103C8); the high-density GD32F103RC shares every
        // register model these vectors exercise.
        Family::Gd32f10x => Layout {
            root: "third_party/gd32-spl",
            sources: "GD32F10x/GD32F10x_standard_peripheral/Source",
            defines: &["GD32F10X_MD", "USE_STDPERIPH_DRIVER"],
            includes: &[
                "GD32F10x/GD32F10x_standard_peripheral/Include",
                "GD32F10x/CMSIS/GD/GD32F10x/Include",
                "GD32F10x/CMSIS",
            ],
            harness_includes: true,
            header: "gd32f10x.h",
            cflags: CFLAGS,
        },
        // The GD32F1x0 CMSIS directory ships core_cm3.h without the core_cmInstr.h and
        // core_cmFunc.h it includes; the GD32F10x CMSIS directory has them, from the same ARM CMSIS
        // release (V3.30, the core_cm3.h files are byte-identical).
        Family::Gd32f1x0 => Layout {
            root: "third_party/gd32-spl",
            sources: "GD32F1x0/GD32F1x0_standard_peripheral/Source",
            defines: &["GD32F130_150", "USE_STDPERIPH_DRIVER"],
            includes: &[
                "GD32F1x0/GD32F1x0_standard_peripheral/Include",
                "GD32F1x0/CMSIS/GD/GD32F1x0/Include",
                "GD32F1x0/CMSIS",
                "GD32F10x/CMSIS",
            ],
            harness_includes: true,
            header: "gd32f1x0.h",
            cflags: CFLAGS,
        },
        // LibSamples needs no define: `mm32_reg.h` is the one part's header. `hal_conf.h` pulls in
        // every driver header; the CMSIS core headers are the ones the package ships (the KEIL and
        // IAR copies are identical; `reg_common.h` includes `core_cm0.h`).
        Family::Mm32spin05 => Layout {
            root: "third_party/mm32-libsamples/LibSamples_MM32SPIN0x_q_V1.13/Device",
            sources: "MM32SPIN0x_q/HAL_Lib/Src",
            defines: &[],
            // `Source/` for a prelude's `#include "system_mm32spin0x_q.c"` (not compiled into the
            // library: the harness runs no SystemInit).
            includes: &[
                "MM32SPIN0x_q/HAL_Lib/Inc",
                "MM32SPIN0x_q/Include",
                "MM32SPIN0x_q/Source",
                "CMSIS/KEIL_Core",
            ],
            harness_includes: false,
            header: "hal_conf.h",
            cflags: CFLAGS_M0,
        },
    }
}

/// Refuse an MM32 run unless `tools/fetch-vendor.sh` left a verified LibSamples copy: its stamp
/// file holds the pinned hash. A GD family needs nothing (its library is committed).
pub fn require_verified(repo: &Path, family: Family) -> Result<(), String> {
    if family != Family::Mm32spin05 {
        return Ok(());
    }
    let stamp = repo.join("third_party/mm32-libsamples/SHA256");
    let tree = repo.join(layout(family).root);
    match std::fs::read_to_string(&stamp) {
        Ok(text) if text.trim() == LIBSAMPLES_SHA256 && tree.is_dir() => Ok(()),
        _ => Err(format!(
            "the MM32SPIN05 vectors need MindMotion's LibSamples V1.13 verified at \
             third_party/mm32-libsamples/ (SHA-256 {LIBSAMPLES_SHA256}); run tools/fetch-vendor.sh"
        )),
    }
}

/// The vendor builder for one run: the compiler, the source tree and a build directory.
pub struct Builder {
    gcc: String,
    repo: PathBuf,
    assets: PathBuf,
    build: PathBuf,
    /// The SPL objects compiled so far, per family.
    objects: RefCell<BTreeMap<Family, Vec<PathBuf>>>,
    /// The compiler's version line.
    pub gcc_version: String,
}

fn run(cmd: &mut Command) -> Result<(), String> {
    let out = cmd
        .output()
        .map_err(|e| format!("{:?}: {e}", cmd.get_program()))?;
    if !out.status.success() {
        return Err(format!(
            "{:?} failed:\n{}{}",
            cmd.get_program(),
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    Ok(())
}

impl Builder {
    /// A builder over `repo` (the repository root), building into `build`. Refuses a compiler other
    /// than [`PINNED_GCC`].
    pub fn new(repo: &Path, harness: &Path, build: &Path) -> Result<Builder, String> {
        let gcc = "arm-none-eabi-gcc".to_string();
        let out = Command::new(&gcc)
            .arg("--version")
            .output()
            .map_err(|e| format!("{gcc}: {e} (the vendor traces need {PINNED_GCC})"))?;
        let version = String::from_utf8_lossy(&out.stdout)
            .lines()
            .next()
            .unwrap_or("")
            .trim()
            .to_string();
        if !version.contains(&format!("({PINNED_GCC} ")) {
            return Err(format!(
                "{gcc} is {version:?}; the vendor traces are generated with {PINNED_GCC}"
            ));
        }
        std::fs::create_dir_all(build).map_err(|e| e.to_string())?;
        Ok(Builder {
            gcc,
            repo: repo.to_path_buf(),
            assets: harness.join("vendor"),
            build: build.to_path_buf(),
            objects: RefCell::new(BTreeMap::new()),
            gcc_version: version,
        })
    }

    fn compile_args(&self, family: Family) -> Vec<String> {
        let l = layout(family);
        let root = self.repo.join(l.root);
        let mut args: Vec<String> = l.cflags.iter().map(|s| s.to_string()).collect();
        for d in l.defines {
            args.push(format!("-D{d}"));
        }
        // The harness's libopt header first: the SPL expects the application to supply it.
        if l.harness_includes {
            args.push(format!("-I{}", self.assets.join(family.name()).display()));
        }
        for inc in l.includes {
            args.push(format!("-I{}", root.join(inc).display()));
        }
        args
    }

    /// The family's library sources compiled to objects, once per builder (so a run never links
    /// objects an earlier run compiled with other flags).
    fn spl_objects(&self, family: Family) -> Result<Vec<PathBuf>, String> {
        if let Some(objects) = self.objects.borrow().get(&family) {
            return Ok(objects.clone());
        }
        require_verified(&self.repo, family)?;
        let l = layout(family);
        let src = self.repo.join(l.root).join(l.sources);
        let dir = self.build.join(format!("spl-{}", family.name()));
        let mut sources: Vec<PathBuf> = std::fs::read_dir(&src)
            .map_err(|e| format!("{}: {e}", src.display()))?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().and_then(|x| x.to_str()) == Some("c"))
            .collect();
        sources.sort();
        let objects: Vec<PathBuf> = sources
            .iter()
            .map(|s| dir.join(s.file_name().unwrap()).with_extension("o"))
            .collect();
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        for (s, o) in sources.iter().zip(&objects) {
            run(Command::new(&self.gcc)
                .args(self.compile_args(family))
                .arg("-c")
                .arg(s)
                .arg("-o")
                .arg(o))?;
        }
        self.objects.borrow_mut().insert(family, objects.clone());
        Ok(objects)
    }

    /// Build a vector body (and its file-scope `prelude`, if any) into an ELF, returning its
    /// bytes.
    pub fn build(
        &self,
        id: &str,
        family: Family,
        prelude: Option<&str>,
        body: &str,
    ) -> Result<Vec<u8>, String> {
        let objects = self.spl_objects(family)?;
        let name = id.replace('/', "__");
        let c = self.build.join(format!("{name}.c"));
        let elf = self.build.join(format!("{name}.elf"));
        let header = layout(family).header;
        let prelude = match prelude {
            Some(p) => format!("{p}\n"),
            None => String::new(),
        };
        let source = format!(
            "/* Generated by golden-trace from vectors/{id}.toml. */\n#include \"{header}\"\n\n\
             {prelude}__attribute__((used, noinline, section(\".text.{ENTRY}\")))\nvoid {ENTRY}(void)\n{{\n{body}}}\n"
        );
        std::fs::write(&c, source).map_err(|e| e.to_string())?;
        run(Command::new(&self.gcc)
            .args(self.compile_args(family))
            .arg("-nostdlib")
            .arg("-T")
            .arg(self.assets.join("link.ld"))
            .arg(&c)
            .args(&objects)
            .arg("-lgcc")
            .arg("-Wl,--gc-sections")
            .arg("-o")
            .arg(&elf))
        .map_err(|e| format!("{id}: {e}"))?;
        std::fs::read(&elf).map_err(|e| e.to_string())
    }

    /// The header lines a vendor trace file starts with.
    pub fn header(&self, id: &str, family: Family) -> String {
        let l = layout(family);
        let defines: Vec<String> = l.defines.iter().map(|d| format!("-D{d}")).collect();
        let oracle = match family {
            Family::Gd32f10x | Family::Gd32f1x0 => format!(
                "{}/{} (CommunityGD32Cores/GD32Firmware {SPL_COMMIT})",
                l.root,
                l.sources.split('/').next().unwrap_or("")
            ),
            Family::Mm32spin05 => format!(
                "MindMotion LibSamples_MM32SPIN0x_q V1.13 (zip SHA-256 {LIBSAMPLES_SHA256})"
            ),
        };
        let defines = if defines.is_empty() {
            "(none)".to_string()
        } else {
            defines.join(" ")
        };
        format!(
            "# Vendor trace of vectors/{id}.toml. Generated by `golden-trace regen`; do not edit.\n\
             # oracle:   {oracle}\n\
             # defines:  {defines}\n\
             # compiler: {gcc}\n\
             # cflags:   {cflags}\n",
            gcc = self.gcc_version,
            cflags = l.cflags.join(" "),
        )
    }
}
