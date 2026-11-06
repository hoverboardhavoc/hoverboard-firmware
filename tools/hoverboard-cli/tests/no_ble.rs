//! The CLI built without the `ble` feature, as the bench Pi builds it (`specs/bridge-onboarding.md`
//! section 4.2): a BLE link, given with `--device` or by giving no link, and `ble scan` are
//! refused, naming the feature; `usb scan` is not (`specs/cli-discovery.md` section 1). Run with `cargo test --no-default-features`; the default build compiles none
//! of it.
#![cfg(not(feature = "ble"))]

use std::process::Command;

#[test]
fn without_ble_the_ble_link_and_scan_are_refused_naming_the_feature() {
    for args in [
        &["--device", "hb-bridge-ab12", "bridge", "info", "0x03"][..],
        &["bridge", "info", "0x03"],
        &["machine", "nodes"],
        &["ble", "scan"],
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

/// `usb scan` reaches its own refusal of a link option, not the feature's: the build without `ble`
/// carries it. (Run without a link it would open the host's C3 ports.)
#[test]
fn without_ble_usb_scan_is_there() {
    let o = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args(["--serial", "/dev/null", "usb", "scan"])
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&o.stderr);
    assert!(!o.status.success());
    assert!(err.contains("usb scan opens every port itself"), "{err}");
}
