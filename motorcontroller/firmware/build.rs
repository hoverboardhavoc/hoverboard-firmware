use std::env;
use std::fs;
use std::path::PathBuf;

/// The bottom of SRAM the SWD mailbox carves out (memory.x); the linked RAM starts above it.
const MAILBOX_CARVE: u32 = 0x240;
/// The `oversize-link` build's regions (Cargo.toml): larger than any chip's, so an image over its
/// chip's budget still links and its size can be measured.
const OVERSIZE_FLASH: u32 = 256 * 1024;
const OVERSIZE_RAM: u32 = 64 * 1024 - MAILBOX_CARVE;

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=not-flashable.x");
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
    // The budgets are the chip's in every build; only the linked regions differ.
    let oversize = env::var_os("CARGO_FEATURE_OVERSIZE_LINK").is_some();
    let (link_flash, link_ram, marker) = if oversize {
        (
            OVERSIZE_FLASH,
            OVERSIZE_RAM,
            include_str!("not-flashable.x"),
        )
    } else {
        (flash, ram, "")
    };
    let memory = include_str!("memory.x")
        .replace("@FLASH_LENGTH@", &format!("{link_flash:#X}"))
        .replace("@RAM_LENGTH@", &format!("{link_ram:#X}"))
        .replace("@FLASH_BUDGET@", &format!("{flash:#X}"))
        .replace("@RAM_BUDGET@", &format!("{ram:#X}"))
        .replace("@NOT_FLASHABLE@", marker);
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), memory).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}
