//! The CLI built without the `ble` feature, as the bench Pi builds it (`specs/bridge-onboarding.md`
//! section 4.2): a BLE link, given with `--device` or by giving no link, and `scan` are refused,
//! naming the feature. Run with `cargo test --no-default-features`; the default build compiles none
//! of it.
#![cfg(not(feature = "ble"))]

use std::process::Command;

#[test]
fn without_ble_the_ble_link_and_scan_are_refused_naming_the_feature() {
    for args in [
        &["--device", "hb-bridge-ab12", "info"][..],
        &["info"],
        &["nodes"],
        &["scan"],
    ] {
        let o = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
            .args(args)
            .output()
            .unwrap();
        let err = String::from_utf8_lossy(&o.stderr);
        assert!(!o.status.success(), "{args:?}");
        assert!(
            err.contains("built without the ble feature"),
            "{args:?}: {err}"
        );
    }
}
