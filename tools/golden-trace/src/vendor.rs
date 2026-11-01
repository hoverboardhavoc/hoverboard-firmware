// SPDX-License-Identifier: GPL-3.0-or-later
//! The vendor side: a vector's C body compiled against GigaDevice's standard peripheral library
//! (`third_party/gd32-spl/`) with arm-none-eabi-gcc, and the trace file it renders to.
//!
//! The body is wrapped in `gt_entry`, compiled with the family's device define, and linked with
//! every SPL source of the family (`--gc-sections` keeps what the body calls). Nothing else runs:
//! no startup code, no `SystemInit`, so the trace is the body's accesses alone.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::target::Family;

/// The Arm GNU Toolchain release the checked-in vendor traces are generated with. A regeneration
/// with another compiler is refused, so a trace difference is never a compiler difference.
pub const PINNED_GCC: &str = "Arm GNU Toolchain 15.2.Rel1";

/// The compile flags. `-std=gnu11` because the SPL typedefs `bool`; `-ffunction-sections` so the
/// link keeps only what the body reaches.
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

/// The entry symbol the wrapper defines.
pub const ENTRY: &str = "gt_entry";

/// The SPL's commit, as `third_party/gd32-spl/SOURCE` records it.
pub const SPL_COMMIT: &str = "4ead0d8a549820330ebb5e03907fb8bb2e6ac5b5";

/// How one family's SPL is built.
struct Layout {
    /// The family directory under `third_party/gd32-spl/`.
    dir: &'static str,
    /// The device define the family header requires.
    define: &'static str,
    /// Include directories, relative to `third_party/gd32-spl/`.
    includes: &'static [&'static str],
}

fn layout(family: Family) -> Layout {
    match family {
        // The medium-density define (the GD32F103C8); the high-density GD32F103RC shares every
        // register model these vectors exercise.
        Family::Gd32f10x => Layout {
            dir: "GD32F10x",
            define: "GD32F10X_MD",
            includes: &[
                "GD32F10x/GD32F10x_standard_peripheral/Include",
                "GD32F10x/CMSIS/GD/GD32F10x/Include",
                "GD32F10x/CMSIS",
            ],
        },
        // The GD32F1x0 CMSIS directory ships core_cm3.h without the core_cmInstr.h and
        // core_cmFunc.h it includes; the GD32F10x CMSIS directory has them, from the same ARM CMSIS
        // release (V3.30, the core_cm3.h files are byte-identical).
        Family::Gd32f1x0 => Layout {
            dir: "GD32F1x0",
            define: "GD32F130_150",
            includes: &[
                "GD32F1x0/GD32F1x0_standard_peripheral/Include",
                "GD32F1x0/CMSIS/GD/GD32F1x0/Include",
                "GD32F1x0/CMSIS",
                "GD32F10x/CMSIS",
            ],
        },
    }
}

/// The vendor builder for one run: the compiler, the source tree and a build directory.
pub struct Builder {
    gcc: String,
    spl: PathBuf,
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
            spl: repo.join("third_party/gd32-spl"),
            assets: harness.join("vendor"),
            build: build.to_path_buf(),
            objects: RefCell::new(BTreeMap::new()),
            gcc_version: version,
        })
    }

    fn compile_args(&self, family: Family) -> Vec<String> {
        let l = layout(family);
        let mut args: Vec<String> = CFLAGS.iter().map(|s| s.to_string()).collect();
        args.push(format!("-D{}", l.define));
        args.push("-DUSE_STDPERIPH_DRIVER".into());
        // The harness's libopt header first: the SPL expects the application to supply it.
        args.push(format!("-I{}", self.assets.join(family.name()).display()));
        for inc in l.includes {
            args.push(format!("-I{}", self.spl.join(inc).display()));
        }
        args
    }

    /// The family's SPL sources compiled to objects, once per builder (so a run never links objects
    /// an earlier run compiled with other flags).
    fn spl_objects(&self, family: Family) -> Result<Vec<PathBuf>, String> {
        if let Some(objects) = self.objects.borrow().get(&family) {
            return Ok(objects.clone());
        }
        let l = layout(family);
        let src = self
            .spl
            .join(l.dir)
            .join(format!("{}_standard_peripheral/Source", l.dir));
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

    /// Build a vector body into an ELF, returning its bytes.
    pub fn build(&self, id: &str, family: Family, body: &str) -> Result<Vec<u8>, String> {
        let objects = self.spl_objects(family)?;
        let name = id.replace('/', "__");
        let c = self.build.join(format!("{name}.c"));
        let elf = self.build.join(format!("{name}.elf"));
        let header = format!("{}.h", family.name());
        let source = format!(
            "/* Generated by golden-trace from vectors/{id}.toml. */\n#include \"{header}\"\n\n\
             __attribute__((used, noinline, section(\".text.{ENTRY}\")))\nvoid {ENTRY}(void)\n{{\n{body}}}\n"
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
        format!(
            "# Vendor trace of vectors/{id}.toml. Generated by `golden-trace regen`; do not edit.\n\
             # oracle:   third_party/gd32-spl/{dir} (CommunityGD32Cores/GD32Firmware {SPL_COMMIT})\n\
             # defines:  -D{define} -DUSE_STDPERIPH_DRIVER\n\
             # compiler: {gcc}\n\
             # cflags:   {cflags}\n",
            dir = l.dir,
            define = l.define,
            gcc = self.gcc_version,
            cflags = CFLAGS.join(" "),
        )
    }
}
