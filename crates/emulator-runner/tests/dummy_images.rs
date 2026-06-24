//! Tier-2 CI gate: run the real thumbv7m dummy images under Unicorn and assert the rig both reports
//! a correct image as PASS and catches a bad/incomplete one.
//!
//! These tests need the three built ELFs. emulator-runner is excluded from the workspace (it links
//! Unicorn's native lib, which cannot cross-compile to the chip), so we build the dummy-test images
//! via the workspace and read them from the known target path. The release ELF has no symbol table;
//! the runner reads/writes the fixed RESULT_ADDR/CMD_ADDR directly, so no symbol resolution is needed.

use std::path::PathBuf;
use std::process::Command;

use emulator_runner::produced_correct_output;

/// The same input the spec's Level-1 tests use. The transform makes a correct output impossible
/// unless the image actually received this value.
const INPUT: u32 = 0xDEAD_0000;

/// Workspace root: two levels up from this crate's manifest dir (`crates/emulator-runner`).
fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

/// Build the three dummy images (release thumbv7m) once and return the directory holding them.
fn dummy_release_dir() -> PathBuf {
    let root = workspace_root();
    let status = Command::new(env!("CARGO"))
        .current_dir(&root)
        .args([
            "build",
            "--release",
            "-p",
            "dummy-test",
            "--target",
            "thumbv7m-none-eabi",
        ])
        .status()
        .expect("spawn cargo to build dummy-test");
    assert!(status.success(), "building dummy-test images failed");
    root.join("target/thumbv7m-none-eabi/release")
}

fn image(name: &str) -> PathBuf {
    dummy_release_dir().join(name)
}

#[test]
fn pass_image_is_correct() {
    assert!(produced_correct_output(&image("dummy-pass"), INPUT));
}

#[test]
fn fail_image_is_caught() {
    // The fail image forgets the transform, so the rig must see the wrong output: GREEN here means
    // produced_correct_output is false.
    assert!(!produced_correct_output(&image("dummy-fail"), INPUT));
}

#[test]
fn hang_image_is_caught() {
    // The hang image never publishes, so `ready` never sets and the instruction cap fires: GREEN
    // here means produced_correct_output is false.
    assert!(!produced_correct_output(&image("dummy-hang"), INPUT));
}
