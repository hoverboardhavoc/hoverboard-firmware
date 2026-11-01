//! The linker script (`specs/handheld-remote.md` section 6.1).

fn main() {
    if std::env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("xtensa") {
        println!("cargo:rustc-link-arg=-Tlinkall.x");
    }
}
