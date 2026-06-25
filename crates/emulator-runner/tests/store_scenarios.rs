//! Tier-2 CI gate for the STORE: the real thumbv7m `store-test` image running under Unicorn with the
//! FMC board model, persistent flash across the two-phase reset, and host-planted crafted regions.
//!
//! Like the dummy tests, these build the firmware image via the workspace and read it from the target
//! path. `store-test` is built TWICE: `--features chip1k` (two 1 KiB pages) and
//! `--no-default-features --features chip2k` (two 2 KiB pages); the two binaries share a name, so each
//! is copied aside after its build. The host plants crafted regions with the store's OWN codec
//! (`store::record::encode`), so a planted record is byte-identical to what the firmware writes.
//!
//! The judgments live on the host (the same non-circular shape as the dummy): the host knows the
//! value it planted / expects (`T_VAL`, `T_STR_VAL`, `T_BLOB_VAL`) and asserts the device's read-back.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

use emulator_runner::store_emu::StoreEmu;
use store::scenarios;
use store::{
    COMPACT, FULL, PERSIST, TORN_HEADER, TORN_PAYLOAD, T_BLOB_VAL, T_STR_VAL, T_VAL, VAR_VALUE,
};
use test_shared::RESULT_READY;

// The chip flash EXTENTS the emulator's store region geometry derives from (matching FmcFlash): the
// chip1k 1 KiB-page part is the 64 KiB F103 (store at 0x0800_F800), the chip2k 2 KiB-page part is the
// 256 KiB high-density F10x (store at 0x0803_F000).
const FLASH_1K: u32 = 64 * 1024;
const FLASH_2K: u32 = 256 * 1024;

/// Pack a `(scenario, phase)` into the host `cmd` the device reads from `CMD_ADDR`.
fn cmd(scenario: u32, phase: u32) -> u32 {
    (scenario << 16) | (phase & 0xFFFF)
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

/// Build `store-test` with the given feature args and copy the ELF to a stable per-feature path so
/// the chip1k and chip2k binaries (same crate, same name) do not clobber each other.
fn build_store_test(feature_args: &[&str], tag: &str) -> PathBuf {
    let root = workspace_root();
    let mut args = vec![
        "build",
        "--release",
        "-p",
        "store-test",
        "--target",
        "thumbv7m-none-eabi",
    ];
    args.extend_from_slice(feature_args);
    let status = Command::new(env!("CARGO"))
        .current_dir(&root)
        .args(&args)
        .status()
        .expect("spawn cargo to build store-test");
    assert!(status.success(), "building store-test ({tag}) failed");
    let built = root.join("target/thumbv7m-none-eabi/release/store-test");
    let dst = root.join(format!(
        "target/thumbv7m-none-eabi/release/store-test-{tag}"
    ));
    std::fs::copy(&built, &dst).expect("copy store-test image aside");
    dst
}

fn chip1k_image() -> &'static Path {
    static P: OnceLock<PathBuf> = OnceLock::new();
    P.get_or_init(|| build_store_test(&["--features", "chip1k"], "chip1k"))
}

fn chip2k_image() -> &'static Path {
    static P: OnceLock<PathBuf> = OnceLock::new();
    P.get_or_init(|| build_store_test(&["--no-default-features", "--features", "chip2k"], "chip2k"))
}

/// Build a crafted region for a planted scenario into a fresh `Vec`, via the store's SHARED builders
/// (`store::scenarios`), so the planted region is byte-identical to the one the hardware runner plants
/// AND to what the firmware itself writes (both tiers call the same codec-backed builders).
fn planted_region(scenario: u32, page_size: usize) -> Vec<u8> {
    let mut buf = vec![0u8; 2 * page_size];
    assert!(
        scenarios::build_planted_region(scenario, &mut buf, page_size),
        "scenario {scenario} is not host-planted"
    );
    buf
}

// =====================================================================================
// Persist-survives-reboot (both page sizes) + the no-write negative control.
// =====================================================================================

fn run_persist(image: &Path, page_size: u32, flash_size: u32) {
    let mut emu = StoreEmu::new(image, page_size, flash_size).expect("build emu");
    // Phase 0: set + persist (the append IS the persist). Phase 1: reboot + read.
    let _ = emu.run_phase(cmd(PERSIST, 0));
    let r = emu.run_phase(cmd(PERSIST, 1));
    assert_eq!(r.ready, RESULT_READY, "phase 1 did not publish");
    assert_eq!(
        r.output, T_VAL,
        "persisted value did not survive the reboot"
    );
}

#[test]
fn persist_survives_reboot_chip1k() {
    run_persist(chip1k_image(), 1024, FLASH_1K);
}

#[test]
fn persist_survives_reboot_chip2k() {
    run_persist(chip2k_image(), 2048, FLASH_2K);
}

#[test]
fn no_write_negative_control_chip1k() {
    // The store analog of dummy-fail: send ONLY the read phase, never the set. The read returns the
    // default, not T_VAL, so the rig CATCHES a vacuous pass.
    let mut emu = StoreEmu::new(chip1k_image(), 1024, FLASH_1K).expect("build emu");
    let r = emu.run_phase(cmd(PERSIST, 1));
    assert_eq!(r.ready, RESULT_READY, "read phase should still publish");
    assert_ne!(r.output, T_VAL, "no write happened, yet T_VAL was read");
}

// =====================================================================================
// Variable-value round trip (device-written), via TestResult.buf/len.
// =====================================================================================

#[test]
fn variable_value_round_trip_chip1k() {
    let mut emu = StoreEmu::new(chip1k_image(), 1024, FLASH_1K).expect("build emu");
    // Phase 0: set_str(DEVICE_NAME) + set_bytes(T_BLOB).
    let _ = emu.run_phase(cmd(VAR_VALUE, 0));
    // Phase 1: get_str(DEVICE_NAME) -> buf == T_STR_VAL.
    let r = emu.run_phase(cmd(VAR_VALUE, 1));
    assert_eq!(r.ready, RESULT_READY);
    assert_eq!(
        &r.buf[..r.len as usize],
        T_STR_VAL.as_bytes(),
        "STR round-trip mismatch"
    );
    // Phase 2: get_bytes -> buf == T_BLOB_VAL.
    let r = emu.run_phase(cmd(VAR_VALUE, 2));
    assert_eq!(r.ready, RESULT_READY);
    assert_eq!(
        &r.buf[..r.len as usize],
        T_BLOB_VAL,
        "BLOB round-trip mismatch"
    );
}

// =====================================================================================
// Host-planted crafted-region scenarios (the device only cold-mounts + reads recovery).
// =====================================================================================

#[test]
fn compaction_preserves_keys_chip1k() {
    // Plant more than one record (known T_KEY + a couple of unknown keys), with an OLDER T_KEY value
    // superseded by T_VAL, so the latest-per-key survivor the device reads is T_VAL.
    let ps = 1024usize;
    let image = planted_region(COMPACT, ps);

    let mut emu = StoreEmu::new(chip1k_image(), ps as u32, FLASH_1K).expect("build emu");
    emu.load_region(&image);
    let r = emu.run_phase(cmd(COMPACT, 1));
    assert_eq!(r.ready, RESULT_READY);
    assert_eq!(r.output, T_VAL, "compaction lost the latest T_KEY");
}

#[test]
fn torn_payload_recovery_chip1k() {
    // A good T_KEY=T_VAL record, then a half-written PAYLOAD of a newer value (hdr_crc good, val_crc
    // corrupt). Mount skips the torn record; the last good value (T_VAL) reads. No erase.
    let ps = 1024usize;
    let image = planted_region(TORN_PAYLOAD, ps);

    let mut emu = StoreEmu::new(chip1k_image(), ps as u32, FLASH_1K).expect("build emu");
    emu.load_region(&image);
    let r = emu.run_phase(cmd(TORN_PAYLOAD, 1));
    assert_eq!(r.ready, RESULT_READY);
    assert_eq!(
        r.output, T_VAL,
        "torn-payload recovery did not read the last good value"
    );
}

#[test]
fn torn_header_auto_compaction_chip1k() {
    // A good T_KEY=T_VAL record, then a torn HEADER (hdr_crc bad). Mount auto-compacts; the survivor
    // reads back and the frontier is clean.
    let ps = 1024usize;
    let image = planted_region(TORN_HEADER, ps);

    let mut emu = StoreEmu::new(chip1k_image(), ps as u32, FLASH_1K).expect("build emu");
    emu.load_region(&image);
    let r = emu.run_phase(cmd(TORN_HEADER, 1));
    assert_eq!(r.ready, RESULT_READY);
    assert_eq!(
        r.output, T_VAL,
        "torn-header auto-compaction lost the survivor"
    );
}

#[test]
fn full_then_compact_then_retry_chip1k() {
    // Plant a near-full active page of a large unknown blob, with NO T_KEY record yet. Phase 0: the
    // device set(T_KEY) returns Full -> compact() -> retry succeeds. Phase 1: read back T_VAL.
    let ps = 1024usize;
    let image = planted_region(FULL, ps);

    let mut emu = StoreEmu::new(chip1k_image(), ps as u32, FLASH_1K).expect("build emu");
    emu.load_region(&image);
    // Phase 0: set -> Full -> compact -> retry (all on-device).
    let r0 = emu.run_phase(cmd(FULL, 0));
    assert_eq!(r0.ready, RESULT_READY, "FULL phase 0 did not complete");
    // Phase 1: read back the value the retry wrote.
    let r1 = emu.run_phase(cmd(FULL, 1));
    assert_eq!(r1.ready, RESULT_READY);
    assert_eq!(
        r1.output, T_VAL,
        "Full->compact->retry did not persist T_VAL"
    );
}

// =====================================================================================
// The deliberately-broken hang image (the store analog of dummy-hang): caught by the cap.
// =====================================================================================

#[test]
fn hang_image_is_caught_chip1k() {
    let root = workspace_root();
    // The hang binary is built alongside store-test by the same -p store-test build.
    let _ = chip1k_image(); // ensure the crate is built
    let hang = root.join("target/thumbv7m-none-eabi/release/store-hang");
    let mut emu = StoreEmu::new(&hang, 1024, FLASH_1K).expect("build emu");
    let r = emu.run_phase(cmd(PERSIST, 1));
    // It never publishes, so `ready` never reaches RESULT_READY: the cap fires and the rig catches it.
    assert_ne!(
        r.ready, RESULT_READY,
        "hang image should never publish ready"
    );
}
