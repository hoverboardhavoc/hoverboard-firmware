use std::env;
use std::fs;
use std::path::PathBuf;

/// The bottom of SRAM the SWD mailbox carves out (memory.x); the linked RAM starts above it.
const MAILBOX_CARVE: u32 = 0x240;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    // `universal_image`: the build with no chip feature, the universal image on runtime-hal. A cfg
    // set here rather than a `universal` cargo feature, because a feature would change this crate's
    // metadata hash and with it the universal image's function order (Cargo.toml explains).
    println!("cargo:rustc-check-cfg=cfg(universal_image)");
    let chip_feature = ["CARGO_FEATURE_GD32F103C8", "CARGO_FEATURE_GD32F130C8"]
        .iter()
        .any(|f| env::var_os(f).is_some());
    if !chip_feature {
        println!("cargo:rustc-cfg=universal_image");
    }
    // A per-chip image: hal publishes its chip's flash, page and SRAM sizes as `links` metadata
    // (hal/build.rs), which reaches this script as DEP_HAL_*. The universal image does not depend on
    // hal and links memory.x as it is.
    let chip = (
        env::var("DEP_HAL_FLASH_KIB"),
        env::var("DEP_HAL_PAGE_BYTES"),
        env::var("DEP_HAL_RAM_KIB"),
    );
    let memory = match chip {
        (Ok(flash_kib), Ok(page_bytes), Ok(ram_kib)) => {
            let n = |v: &str| -> u32 { v.parse().expect("hal chip metadata is a number") };
            let flash = n(&flash_kib) * 1024 - 2 * n(&page_bytes);
            let ram = n(&ram_kib) * 1024 - MAILBOX_CARVE;
            include_str!("memory-chip.x")
                .replace("@FLASH_LENGTH@", &format!("{flash:#X}"))
                .replace("@RAM_LENGTH@", &format!("{ram:#X}"))
        }
        _ => include_str!("memory.x").to_string(),
    };
    fs::write(out.join("memory.x"), memory).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-chip.x");
    println!("cargo:rerun-if-changed=build.rs");
}
