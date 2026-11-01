//! Tier-2 CI gate: run the real thumbv7m dummy images under Unicorn and assert the rig both reports
//! a correct image as PASS and catches a bad/incomplete one.
//!
//! These tests need the three built ELFs, once per chip. emulator-runner is a tools/ workspace member (it links
//! Unicorn's native lib, which cannot cross-compile to the chip), so we build the dummy-test images
//! in the motorcontroller workspace and read them from its known target path. The release ELF has no symbol table;
//! the runner reads/writes the fixed RESULT_ADDR/CMD_ADDR directly, so no symbol resolution is needed.

use std::path::PathBuf;
use std::process::Command;
use std::sync::OnceLock;

use emulator_runner::produced_correct_output;

/// The same input the spec's Level-1 tests use. The transform makes a correct output impossible
/// unless the image actually received this value.
const INPUT: u32 = 0xDEAD_0000;

/// The motorcontroller workspace, which builds the GD32 images: `../../motorcontroller` from this
/// crate's manifest dir (`tools/emulator-runner`).
fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("motorcontroller")
}

/// The chips the images are built for (specs/hal.md section 8.3): each build links that chip's
/// flash vector table from `hal`.
const CHIPS: [&str; 3] = ["gd32f103c8", "gd32f130c8", "gd32f103rc"];

/// Build the three dummy images (release thumbv7m) for `chip` and copy them into a per-chip
/// directory, since every chip's build writes the same three names.
fn build_dummy_images(chip: &str) -> PathBuf {
    let root = workspace_root();
    let status = Command::new(env!("CARGO"))
        .current_dir(&root)
        .args([
            "build",
            "--release",
            "-p",
            "dummy-test",
            "--features",
            chip,
            "--target",
            "thumbv7m-none-eabi",
        ])
        .status()
        .expect("spawn cargo to build dummy-test");
    assert!(
        status.success(),
        "building dummy-test images for {chip} failed"
    );
    let release = root.join("target/thumbv7m-none-eabi/release");
    let dir = release.join(format!("dummy-{chip}"));
    std::fs::create_dir_all(&dir).expect("per-chip image directory");
    for name in ["dummy-pass", "dummy-fail", "dummy-hang"] {
        std::fs::copy(release.join(name), dir.join(name)).expect("copy a dummy image aside");
    }
    dir
}

/// The per-chip image directories, built once (the tests run on parallel threads).
fn images() -> &'static [PathBuf] {
    static DIRS: OnceLock<Vec<PathBuf>> = OnceLock::new();
    DIRS.get_or_init(|| CHIPS.iter().map(|c| build_dummy_images(c)).collect())
}

#[test]
fn pass_image_is_correct() {
    for dir in images() {
        assert!(
            produced_correct_output(&dir.join("dummy-pass"), INPUT),
            "{}",
            dir.display()
        );
    }
}

#[test]
fn fail_image_is_caught() {
    // The fail image forgets the transform, so the rig must see the wrong output: GREEN here means
    // produced_correct_output is false.
    for dir in images() {
        assert!(
            !produced_correct_output(&dir.join("dummy-fail"), INPUT),
            "{}",
            dir.display()
        );
    }
}

#[test]
fn hang_image_is_caught() {
    // The hang image never publishes, so `ready` never sets and the instruction cap fires: GREEN
    // here means produced_correct_output is false.
    for dir in images() {
        assert!(
            !produced_correct_output(&dir.join("dummy-hang"), INPUT),
            "{}",
            dir.display()
        );
    }
}
