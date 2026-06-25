//! Host tool: emit a crafted store-region image for a planted bench scenario.
//!
//! The host-planted Tier-3 scenarios (COMPACT / TORN_PAYLOAD / TORN_HEADER / FULL) need a crafted
//! store-region byte image written into the device's flash before the read phase. This dumps that
//! image (length `2 * page_size`) using the store's OWN codec (`store::scenarios`), so the bytes are
//! byte-identical to what the firmware writes and to what the emulator plants.
//!
//! Usage: `cargo run --example craft_region --features test-fields -- <scenario> <page_size> <out>`
//! where `<scenario>` is one of compact|torn_payload|torn_header|full and `<page_size>` is 1024 or
//! 2048. Writes the raw region bytes to `<out>` (flash with `flash write_image <out> <store_base>
//! bin`).

use std::fs;

use store::scenarios;
use store::{COMPACT, FULL, TORN_HEADER, TORN_PAYLOAD};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 4 {
        eprintln!("usage: craft_region <compact|torn_payload|torn_header|full> <page_size> <out>");
        std::process::exit(2);
    }
    let scenario = match args[1].as_str() {
        "compact" => COMPACT,
        "torn_payload" => TORN_PAYLOAD,
        "torn_header" => TORN_HEADER,
        "full" => FULL,
        other => {
            eprintln!("unknown scenario {other}");
            std::process::exit(2);
        }
    };
    let page_size: usize = args[2].parse().expect("page_size must be an integer");
    let out = &args[3];

    let mut buf = vec![0u8; 2 * page_size];
    assert!(
        scenarios::build_planted_region(scenario, &mut buf, page_size),
        "scenario {} is not host-planted",
        args[1]
    );
    fs::write(out, &buf).expect("write region image");
    eprintln!(
        "wrote {} bytes ({} page_size {}) to {}",
        buf.len(),
        args[1],
        page_size,
        out
    );
}
