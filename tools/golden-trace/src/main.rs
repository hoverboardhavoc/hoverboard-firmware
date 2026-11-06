// SPDX-License-Identifier: GPL-3.0-or-later
//! `golden-trace`: the golden register-trace harness's command line (run from `tools/`).
//!
//! ```text
//! cargo run -p golden-trace -- check [FILTER]          every HAL trace against its vendor trace
//! cargo run -p golden-trace -- regen [--check] [FILTER] rebuild the vendor traces (--check: compare)
//! cargo run -p golden-trace -- trace VECTOR (--vendor | --chip CHIP)   print one fresh trace
//! ```
//!
//! `FILTER` keeps the vectors whose id (`<area>/<name>`) contains it. `check` needs the pinned
//! nightly with the thumbv7m target (and thumbv6m for the MM32SPIN05); `regen` and
//! `trace --vendor` also need arm-none-eabi-gcc from the Arm GNU Toolchain 15.2.Rel1. Any command
//! over an MM32 vector needs the verified LibSamples copy `tools/fetch-vendor.sh` leaves.

use std::collections::BTreeMap;
use std::path::Path;
use std::process::ExitCode;

use golden_trace::emu::{self, Image};
use golden_trace::target::{Family, Target};
use golden_trace::vector::{self, HalSide, Vector};
use golden_trace::{compare, hal, trace, vendor};

fn usage() -> ExitCode {
    eprintln!(
        "usage: golden-trace check [FILTER]\n       golden-trace regen [--check] [FILTER]\n       \
         golden-trace trace VECTOR (--vendor | --chip CHIP)"
    );
    ExitCode::from(2)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    let result = match args.as_slice() {
        ["check"] => check(""),
        ["check", filter] => check(filter),
        ["regen"] => regen(false, ""),
        ["regen", "--check"] => regen(true, ""),
        ["regen", "--check", filter] => regen(true, filter),
        ["regen", filter] => regen(false, filter),
        ["trace", id, "--vendor"] => show(id, None),
        ["trace", id, "--chip", chip] => show(id, Some(chip)),
        _ => return usage(),
    };
    match result {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("golden-trace: {e}");
            ExitCode::FAILURE
        }
    }
}

struct Context {
    vectors: Vec<Vector>,
    targets: BTreeMap<Family, Target>,
}

fn context(filter: &str) -> Result<Context, String> {
    let dir = golden_trace::harness_dir();
    let vectors: Vec<Vector> = vector::load_all(&dir.join("vectors"))?
        .into_iter()
        .filter(|v| v.id.contains(filter))
        .collect();
    if vectors.is_empty() {
        return Err(format!("no vector matches {filter:?}"));
    }
    let mut targets = BTreeMap::new();
    for f in Family::ALL {
        targets.insert(f, Target::load(&dir, f)?);
    }
    // Every command over an MM32 vector, its HAL-only `check` included, runs on a verified
    // LibSamples copy (specs/hal.md, section 8.1).
    let repo = golden_trace::repo_dir();
    for f in Family::ALL {
        if vectors.iter().any(|v| v.family == f) {
            vendor::require_verified(&repo, f)?;
        }
    }
    Ok(Context { vectors, targets })
}

/// One emulated run of `entry`, refused unless it returned.
fn trace_of(
    image: &Image,
    entry: &str,
    v: &Vector,
    target: &Target,
) -> Result<Vec<trace::Access>, String> {
    let run = emu::run(
        image,
        entry,
        target,
        &v.responses(target)?,
        &v.overrides(target)?,
    )?;
    run.check_returned(image)?;
    Ok(run.accesses)
}

/// The HAL images for every chip the vectors run on.
fn hal_images(ctx: &Context) -> Result<BTreeMap<&'static str, Image>, String> {
    let repo = golden_trace::repo_dir();
    let build = golden_trace::build_dir();
    let mut images = BTreeMap::new();
    for f in Family::ALL {
        let of_family: Vec<&Vector> = ctx.vectors.iter().filter(|v| v.family == f).collect();
        if !of_family.iter().any(|v| matches!(v.hal, HalSide::Body(_))) {
            continue;
        }
        for chip in f.chips() {
            let on_chip: Vec<&Vector> = of_family
                .iter()
                .copied()
                .filter(|v| v.chips.contains(chip))
                .collect();
            eprintln!("building the HAL image for {chip}");
            let elf = hal::build(&repo, &build, f, chip, &on_chip)?;
            let bytes = std::fs::read(&elf).map_err(|e| format!("{}: {e}", elf.display()))?;
            images.insert(*chip, Image::from_elf(&bytes)?);
        }
    }
    Ok(images)
}

fn check(filter: &str) -> Result<bool, String> {
    let ctx = context(filter)?;
    let images = hal_images(&ctx)?;
    let (mut pass, mut pending, mut fail) = (0, 0, 0);
    for v in &ctx.vectors {
        let target = &ctx.targets[&v.family];
        let path = v.vendor_trace_path();
        let vendor = match std::fs::read_to_string(&path)
            .map_err(|e| format!("{}: {e}", path.display()))
            .and_then(|t| trace::parse(&t, target))
        {
            Ok(t) => t,
            Err(e) => {
                println!("FAIL     {}  (vendor trace: {e})", v.id);
                fail += 1;
                continue;
            }
        };
        if let HalSide::Pending(what) = &v.hal {
            println!("pending  {}  (needs {what})", v.id);
            pending += 1;
            continue;
        }
        let scope = v.scope(target)?;
        let counts = v.write_counts(target)?;
        for chip in &v.chips {
            let image = &images[chip];
            let outcome = trace_of(image, &hal::entry(&v.id), v, target).map(|h| {
                let mut o =
                    compare::compare(v.mode, &scope.narrow(&h), &scope.narrow(&vendor), target);
                let miscounted = compare::write_counts(&h, &counts, target);
                o.matched &= miscounted.is_empty();
                o.diff.extend(miscounted);
                o
            });
            match outcome {
                Ok(o) if o.matched => {
                    println!("pass     {}  {chip}", v.id);
                    pass += 1;
                }
                Ok(o) => {
                    println!("FAIL     {}  {chip}  ({:?}):", v.id, v.mode);
                    for line in o.diff {
                        println!("           {line}");
                    }
                    fail += 1;
                }
                Err(e) => {
                    println!("FAIL     {}  {chip}  ({e})", v.id);
                    fail += 1;
                }
            }
        }
    }
    println!("golden-trace check: {pass} pass, {pending} pending, {fail} fail");
    Ok(fail == 0)
}

/// The vendor trace file text for `v`.
fn vendor_text(b: &vendor::Builder, v: &Vector, target: &Target) -> Result<String, String> {
    let elf = b.build(&v.id, v.family, v.vendor_prelude.as_deref(), &v.vendor_body)?;
    let image = Image::from_elf(&elf)?;
    let accesses =
        trace_of(&image, vendor::ENTRY, v, target).map_err(|e| format!("{}: {e}", v.id))?;
    Ok(format!(
        "{}{}",
        b.header(&v.id, v.family),
        trace::render(&accesses, target)
    ))
}

fn builder() -> Result<vendor::Builder, String> {
    vendor::Builder::new(
        &golden_trace::repo_dir(),
        &golden_trace::harness_dir(),
        &golden_trace::build_dir().join("vendor"),
    )
}

fn regen(check_only: bool, filter: &str) -> Result<bool, String> {
    let ctx = context(filter)?;
    let b = builder()?;
    let mut differ = 0;
    for v in &ctx.vectors {
        let text = vendor_text(&b, v, &ctx.targets[&v.family])?;
        let path = v.vendor_trace_path();
        let current = std::fs::read_to_string(&path).ok();
        if check_only {
            if current.as_deref() == Some(text.as_str()) {
                println!("same     {}", v.id);
            } else {
                println!("DIFFERS  {}  (rebuilt from the pinned source):", v.id);
                print_text_diff(current.as_deref().unwrap_or(""), &text);
                differ += 1;
            }
        } else if current.as_deref() != Some(text.as_str()) {
            std::fs::write(&path, &text).map_err(|e| format!("{}: {e}", path.display()))?;
            println!("wrote    {}", rel(&path));
        }
    }
    if check_only {
        println!(
            "golden-trace regen --check: {} same, {differ} differ",
            ctx.vectors.len() - differ
        );
    }
    Ok(differ == 0)
}

fn rel(path: &Path) -> String {
    let dir = golden_trace::harness_dir();
    path.strip_prefix(&dir)
        .map(|p| p.display().to_string())
        .unwrap_or_else(|_| path.display().to_string())
}

/// The lines that differ between the checked-in and the rebuilt text.
fn print_text_diff(old: &str, new: &str) {
    let (o, n): (Vec<&str>, Vec<&str>) = (old.lines().collect(), new.lines().collect());
    for i in 0..o.len().max(n.len()) {
        let (a, b) = (o.get(i), n.get(i));
        if a != b {
            if let Some(a) = a {
                println!("           checked in line {}: {a}", i + 1);
            }
            if let Some(b) = b {
                println!("           rebuilt    line {}: {b}", i + 1);
            }
        }
    }
}

fn show(id: &str, chip: Option<&str>) -> Result<bool, String> {
    let ctx = context(id)?;
    let v = ctx
        .vectors
        .iter()
        .find(|v| v.id == id)
        .ok_or_else(|| format!("no vector {id}"))?;
    let target = &ctx.targets[&v.family];
    match chip {
        None => print!("{}", vendor_text(&builder()?, v, target)?),
        Some(chip) => {
            if !v.chips.contains(&chip) {
                return Err(format!("{id} does not run on {chip}"));
            }
            let elf = hal::build(
                &golden_trace::repo_dir(),
                &golden_trace::build_dir(),
                v.family,
                chip,
                &[v],
            )?;
            let bytes = std::fs::read(&elf).map_err(|e| e.to_string())?;
            let accesses = trace_of(&Image::from_elf(&bytes)?, &hal::entry(&v.id), v, target)?;
            print!("{}", trace::render(&accesses, target));
        }
    }
    Ok(true)
}
