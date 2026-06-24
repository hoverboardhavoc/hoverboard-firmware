//! The tier-2 Unicorn proof: build the real `harness-smoke` thumbv7m image and run it under the
//! emulator for both command words, asserting the subject's self-checked verdict comes back PASS
//! the same RAM-channel way the silicon would. This is the host-side proof of the whole pipeline.
//!
//! Gated on the `emu` feature (which pulls `unicorn-engine`), so it is only built/run with
//! `cargo test -p harness-emu --features emu`. CI runs it that way; a default thumbv7m build never
//! sees it.

#![cfg(feature = "emu")]

use std::path::PathBuf;
use std::process::Command;

/// The two command words the spec pins. `0xDEAD_0000` and `0x0000_BEEF` exercise distinct echoes,
/// and a correct echo for each proves the subject actually read the delivered word (the XOR in
/// `smoke` would not match a constant).
const COMMANDS: [u32; 2] = [0xDEAD_0000, 0x0000_BEEF];

/// Build `harness-smoke` for thumbv7m-none-eabi (release) and return the path to its ELF. The image
/// is built here (not a checked-in artifact) so the test always runs the current source.
fn build_subject_elf() -> PathBuf {
    // The harness-emu crate dir is CARGO_MANIFEST_DIR; the workspace root is two levels up
    // (crates/harness-emu -> crates -> repo root).
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_root = manifest_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root")
        .to_path_buf();

    // A separate target dir for the cross build so it never clashes with the host test's target dir
    // (a host `cargo test` and this nested thumbv7m build must not fight over the same lock/files).
    // Place it UNDER target/ so the repo's `**/target/` gitignore covers it (no stray artifact).
    let target_dir = manifest_dir.join("target/subject");

    let status = Command::new(env!("CARGO"))
        .current_dir(&workspace_root)
        .args([
            "build",
            "-p",
            "harness-smoke",
            "--release",
            "--target",
            "thumbv7m-none-eabi",
        ])
        .env("CARGO_TARGET_DIR", &target_dir)
        .status()
        .expect("spawn cargo to build harness-smoke");
    assert!(
        status.success(),
        "building harness-smoke for thumbv7m failed"
    );

    let elf = target_dir.join("thumbv7m-none-eabi/release/harness-smoke");
    assert!(elf.exists(), "subject ELF not found at {}", elf.display());
    elf
}

#[test]
fn smoke_subject_passes_under_unicorn() {
    let elf_path = build_subject_elf();
    let elf = std::fs::read(&elf_path).expect("read subject ELF");

    let runs = harness_emu::run_smoke(&elf, &COMMANDS);
    assert_eq!(runs.len(), COMMANDS.len());

    for (cmd, run) in COMMANDS.iter().zip(runs.iter()) {
        assert!(
            run.passed(),
            "smoke run for cmd {:#010x} did not pass: completed={} magic={:#010x} verdict={} echo={:#010x} (expected echo {:#010x})",
            cmd,
            run.completed,
            run.magic,
            run.verdict,
            run.echo,
            harness_abi::smoke(*cmd),
        );
    }
}
