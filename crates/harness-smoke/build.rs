//! Put `memory.x` on the linker search path so cortex-m-rt's `link.x` can include it. Only the
//! bare-metal (thumbv7m) build links a `memory.x`; a host build never reaches the linker script,
//! so copying it unconditionally is harmless (the search-path entry is simply unused on host).

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).expect("copy memory.x");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");
}
