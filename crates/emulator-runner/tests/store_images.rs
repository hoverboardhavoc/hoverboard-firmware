//! Tier-2 CI gate for the store: run the real thumbv7m `store-test` images under Unicorn against the
//! FMC board model, two-phase (set+persist -> reset -> read) with a persistent flash region, and
//! assert the rig both reports a persisting image as PASS and catches a no-persist one. Run at BOTH
//! 1 KiB and 2 KiB page sizes (the 2 KiB path via the synthesized F103RC-style chip).
//!
//! emulator-runner is excluded from the workspace (it links Unicorn's native lib), so we build the
//! store-test images via cargo and read them from the target path. The 1 KiB and 2 KiB variants use
//! different chip features, so each builds into its OWN target dir (distinct `CARGO_TARGET_DIR`) to
//! avoid clobbering the other's binaries.

use std::path::PathBuf;
use std::process::Command;

use emulator_runner::store_persists;

/// Workspace root: two levels up from this crate's manifest dir (`crates/emulator-runner`).
fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

/// Build the two store-test images (release thumbv7m) for a page-size variant into a dedicated target
/// dir and return that dir's release path. `variant_dir` keeps the 1 KiB and 2 KiB builds apart (they
/// produce identically-named binaries with different chips). `features`/`no_default` select the chip.
fn build_variant(variant_dir: &str, no_default: bool, features: &str) -> PathBuf {
    let root = workspace_root();
    let target_dir = root.join("target").join(variant_dir);
    let mut cmd = Command::new(env!("CARGO"));
    cmd.current_dir(&root)
        .env("CARGO_TARGET_DIR", &target_dir)
        .args([
            "build",
            "--release",
            "-p",
            "store-test",
            "--target",
            "thumbv7m-none-eabi",
        ]);
    if no_default {
        cmd.arg("--no-default-features");
    }
    if !features.is_empty() {
        cmd.args(["--features", features]);
    }
    let status = cmd.status().expect("spawn cargo to build store-test");
    assert!(
        status.success(),
        "building store-test ({variant_dir}) failed"
    );
    target_dir.join("thumbv7m-none-eabi/release")
}

/// The 1 KiB-page images (default `f103` chip, 64 KiB / 1 KiB pages).
fn dir_1k() -> PathBuf {
    build_variant("store-test-1k", false, "f103")
}

/// The 2 KiB-page images (synthesized F103RC-style 256 KiB / 2 KiB chip via `detect-internals`).
fn dir_2k() -> PathBuf {
    build_variant("store-test-2k", true, "chip2k")
}

#[test]
fn persist_survives_reboot_1k() {
    let img = dir_1k().join("store-pass");
    assert!(
        store_persists(&img, 1024),
        "1 KiB: the persisted value must survive the reboot (read == T_VAL)"
    );
}

#[test]
fn no_persist_is_caught_1k() {
    let img = dir_1k().join("store-fail-no-persist");
    assert!(
        !store_persists(&img, 1024),
        "1 KiB: the no-persist image must be caught (read != T_VAL)"
    );
}

#[test]
fn persist_survives_reboot_2k() {
    let img = dir_2k().join("store-pass");
    assert!(
        store_persists(&img, 2048),
        "2 KiB: the persisted value must survive the reboot (read == T_VAL)"
    );
}

#[test]
fn no_persist_is_caught_2k() {
    let img = dir_2k().join("store-fail-no-persist");
    assert!(
        !store_persists(&img, 2048),
        "2 KiB: the no-persist image must be caught (read != T_VAL)"
    );
}
