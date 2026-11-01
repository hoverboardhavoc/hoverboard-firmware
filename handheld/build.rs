//! The pinned bridge (`specs/handheld-remote.md` section 3.2) and the linker script.
//!
//! `HANDHELD_BRIDGE` holds the bridge's BLE address as six colon-separated hex octets, most
//! significant first (`e0:6e:f1:12:34:56`). It becomes `BRIDGE: Option<[u8; 6]>` in
//! `$OUT_DIR/bridge.rs`, in the same order; a value that does not parse fails the build. Unset, the
//! constant is `None` and the image boots to "No bridge set."

use std::env;
use std::fs;
use std::path::Path;

fn parse(s: &str) -> Result<[u8; 6], String> {
    let parts: Vec<&str> = s.trim().split(':').collect();
    if parts.len() != 6 {
        return Err(format!("{s:?} is not six colon-separated octets"));
    }
    let mut a = [0u8; 6];
    for (o, p) in a.iter_mut().zip(&parts) {
        if p.len() != 2 {
            return Err(format!("{p:?} in {s:?} is not two hex digits"));
        }
        *o = u8::from_str_radix(p, 16).map_err(|_| format!("{p:?} in {s:?} is not hex"))?;
    }
    Ok(a)
}

fn main() {
    println!("cargo:rerun-if-env-changed=HANDHELD_BRIDGE");
    let value = match env::var("HANDHELD_BRIDGE") {
        Ok(s) if !s.trim().is_empty() => match parse(&s) {
            Ok(a) => format!("Some({a:?})"),
            Err(e) => panic!("HANDHELD_BRIDGE: {e}"),
        },
        _ => "None".to_string(),
    };
    let out = Path::new(&env::var("OUT_DIR").unwrap()).join("bridge.rs");
    fs::write(
        out,
        format!(
            "/// The pinned bridge's BLE address, most significant octet first.\n\
             pub const BRIDGE: Option<[u8; 6]> = {value};\n"
        ),
    )
    .unwrap();

    if env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("xtensa") {
        println!("cargo:rustc-link-arg=-Tlinkall.x");
    }
}
