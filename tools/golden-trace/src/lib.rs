// SPDX-License-Identifier: GPL-3.0-or-later
//! The golden register-trace harness (specs/hal.md, section 8.1).
//!
//! Each vector (`vectors/<area>/<name>.toml`) is one operation written twice: as C against
//! GigaDevice's standard peripheral library (`third_party/gd32-spl/`) and as Rust against the HAL
//! (`motorcontroller/hal`). Both run under Unicorn with the peripheral space hooked; the recorded
//! accesses are compared in the vector's mode ([`compare`]).
//!
//! The vendor traces are generated from the pinned sources with the pinned compiler and checked in
//! beside each vector (`<name>.vendor.trace`). `golden-trace check` compares a fresh HAL trace
//! against every checked-in vendor trace, for every chip of the vector's family;
//! `golden-trace regen --check` rebuilds every vendor trace and fails unless it reproduces the
//! checked-in file byte for byte.

pub mod compare;
pub mod emu;
pub mod hal;
pub mod target;
pub mod trace;
pub mod vector;
pub mod vendor;

use std::path::PathBuf;

/// `tools/golden-trace`.
pub fn harness_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// The repository root.
pub fn repo_dir() -> PathBuf {
    let dir = harness_dir().join("../..");
    dir.canonicalize().unwrap_or(dir)
}

/// Where builds go: `tools/target/golden-trace`.
pub fn build_dir() -> PathBuf {
    harness_dir().join("../target/golden-trace")
}
