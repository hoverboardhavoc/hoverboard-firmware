// SPDX-License-Identifier: GPL-3.0-or-later
//! Flash one dummy image to a real GD32 over SWD, run it, and assert the same outcome the emulator
//! does. Manual, run on the Pi. Compiles for the host; the human runs it on hardware.
//!
//! It takes the chip, the probe selector, and the image path as arguments (nothing hardcoded), and
//! judges the result with the shared `output == dummy(input)` check. It takes no bench lock: the
//! operator takes the resource of the board behind `--probe` first with
//! `~/dev/hoverboard-bench/bench-lock.sh`, and `probes` for a `VID:PID` selector with no serial,
//! which is a scan.

use std::path::PathBuf;
use std::time::Duration;

use clap::Parser;
use hardware_runner::{run_image, RunError};
use test_shared::dummy;

/// Run a dummy firmware image on real GD32 silicon over SWD and assert its output.
#[derive(Parser, Debug)]
#[command(about, long_about = None)]
struct Args {
    /// The ELF image to flash (e.g. a dummy-pass / dummy-fail / dummy-hang release binary).
    image: PathBuf,

    /// probe-rs target/chip name. A GD32 definition with CPUTAPID 0 (the GD32 reports a different DAP
    /// ID than the STM32 part probe-rs assumes).
    #[arg(long)]
    chip: String,

    /// probe-rs probe selector: `VID:PID` or `VID:PID:serial`. Identify probes by full serial, not
    /// USB port (the master F103C8 dapdirect; the slave F130C8 on an older clone).
    #[arg(long)]
    probe: String,

    /// The input word the host delivers and recomputes the expected output from.
    #[arg(long, default_value_t = 0xDEAD_0000, value_parser = parse_u32)]
    input: u32,

    /// Poll timeout in milliseconds. A hung/faulted subject must time out (the ST-Link clones cannot
    /// drive NRST, so we never block forever).
    #[arg(long, default_value_t = 2000)]
    timeout_ms: u64,
}

fn parse_u32(s: &str) -> Result<u32, String> {
    let s = s.trim();
    let parsed = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16)
    } else {
        s.parse::<u32>()
    };
    parsed.map_err(|e| format!("invalid u32 {s:?}: {e}"))
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();

    let result = run_image(
        &args.image,
        &args.chip,
        &args.probe,
        args.input,
        Duration::from_millis(args.timeout_ms),
    );

    match result {
        Ok(r) => {
            let expected = dummy(args.input);
            if r.output == expected {
                println!(
                    "PASS: input={:#010x} output={:#010x} (expected {:#010x})",
                    args.input, r.output, expected
                );
                std::process::ExitCode::SUCCESS
            } else {
                eprintln!(
                    "FAIL: wrong output for input {:#010x}: got {:#010x}, expected {:#010x}",
                    args.input, r.output, expected
                );
                std::process::ExitCode::FAILURE
            }
        }
        Err(RunError::Timeout) => {
            eprintln!("FAIL: subject hung (no result within timeout)");
            std::process::ExitCode::FAILURE
        }
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}
