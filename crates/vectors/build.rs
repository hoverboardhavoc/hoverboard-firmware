//! Emits the `device.x` that cortex-m-rt's `device` feature makes its `link.x` `INCLUDE`, and puts
//! its directory on the linker search path so the final binary's link finds it.
//!
//! `device.x` is where an svd2rust device crate would put `PROVIDED(SOME_IRQ = DefaultHandler)`
//! weak aliases for named interrupt handlers. This workspace names no interrupt handlers through
//! the flash table (runtime-hal builds a RAM table and flips `VTOR`; see `src/lib.rs`), so the file
//! is deliberately empty -- it exists only to satisfy the `INCLUDE`.
//!
//! Cargo accumulates every transitive dependency's `rustc-link-search` onto the final binary's link
//! line (the same mechanism that lets binaries find cortex-m-rt's own `link.x`), so every crate
//! that depends on `vectors` gets this path without restating it.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Host builds never use link.x, so there is nothing to satisfy there.
    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() != Ok("arm") {
        return;
    }

    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let mut f = File::create(out.join("device.x")).expect("create device.x");
    f.write_all(
        b"/* Intentionally empty. See the `vectors` crate: the flash table's IRQ slots all point\n\
          at DefaultHandler, and real dispatch happens through runtime-hal's RAM table after the\n\
          VTOR flip, so there are no named handlers to PROVIDE weak aliases for. */\n",
    )
    .expect("write device.x");

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
}
