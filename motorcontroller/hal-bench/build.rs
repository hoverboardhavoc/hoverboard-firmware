//! The validators' linker regions, from the chip facts `hal` publishes (DEP_HAL_*): flash less the
//! store's two pages (the bridge refuses an image that reaches them), and SRAM above the SWD
//! mailbox carve, as the firmware's per-chip images link.
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let n = |k: &str| -> u32 {
        env::var(k)
            .map(|v| v.parse().expect("hal chip metadata is a number"))
            .unwrap_or(0)
    };
    let (flash_kib, page, ram_kib) = (
        n("DEP_HAL_FLASH_KIB"),
        n("DEP_HAL_PAGE_BYTES"),
        n("DEP_HAL_RAM_KIB"),
    );
    if flash_kib != 0 {
        let flash = flash_kib * 1024 - 2 * page;
        let ram = ram_kib * 1024 - 0x240;
        fs::write(
            out.join("memory.x"),
            format!(
                "MEMORY\n{{\n  FLASH : ORIGIN = 0x08000000, LENGTH = {flash:#X}\n  \
                 RAM : ORIGIN = 0x20000240, LENGTH = {ram:#X}\n}}\n"
            ),
        )
        .unwrap();
        println!("cargo:rustc-link-search={}", out.display());
    }
    println!("cargo:rerun-if-changed=build.rs");
}
