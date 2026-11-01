use std::env;
use std::fs;
use std::path::PathBuf;

/// The bottom of SRAM the SWD mailbox carves out (memory.x); the linked RAM starts above it.
const MAILBOX_CARVE: u32 = 0x240;

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    // hal publishes its chip's flash, page and SRAM sizes as `links` metadata (hal/build.rs), which
    // reaches this script as DEP_HAL_*. The host build has no hal dependency and links no image, so
    // it writes no layout.
    let (Ok(flash_kib), Ok(page_bytes), Ok(ram_kib)) = (
        env::var("DEP_HAL_FLASH_KIB"),
        env::var("DEP_HAL_PAGE_BYTES"),
        env::var("DEP_HAL_RAM_KIB"),
    ) else {
        return;
    };
    let n = |v: &str| -> u32 { v.parse().expect("hal chip metadata is a number") };
    let flash = n(&flash_kib) * 1024 - 2 * n(&page_bytes);
    let ram = n(&ram_kib) * 1024 - MAILBOX_CARVE;
    let memory = include_str!("memory.x")
        .replace("@FLASH_LENGTH@", &format!("{flash:#X}"))
        .replace("@RAM_LENGTH@", &format!("{ram:#X}"));
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), memory).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}
