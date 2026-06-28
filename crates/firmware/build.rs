//! Put this crate's `memory.x` on the linker search path so cortex-m-rt's `link.x` can `INCLUDE` it,
//! and relink when it changes. (Same pattern as dummy-test / store-test.)
//!
//! The SWD mailbox uses a reserved-region carve (memory.x starts RAM above the mailbox base), NOT a
//! `.mailbox` linker section: an `INSERT AFTER .bss` section fragment drags cortex-m-rt's `__ebss` down
//! to the mailbox region's end (`0x2000_0230`), collapsing it BELOW `__sbss` (`0x2000_0400`).
//! cortex-m-rt 0.7.5's `.bss` init is an equality-terminated loop (`cmp __ebss,__sbss; beq done;
//! stm r0!; b`), so with `__sbss > __ebss` it never terminates and stores **upward off the end of RAM**
//! -> an imprecise bus fault before `main` (confirmed on the F103 master). So there is no extra linker
//! script here - just memory.x, the reserved-region idiom.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
