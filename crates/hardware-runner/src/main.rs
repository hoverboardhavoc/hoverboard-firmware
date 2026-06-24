//! Flash one dummy image to a real GD32 over SWD, run it, and assert the same outcome the emulator
//! does. Manual, run on the Pi, bench-locked. Compiles for the host; the human runs it on hardware.
//!
//! It takes the chip, the probe selector, and the image path as arguments (nothing hardcoded), holds
//! `tools/bench-lock.sh` around the hardware op (one shared physical bench), and judges the result
//! with the shared `output == dummy(input)` check.

use std::path::PathBuf;
use std::process::Command;
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

    /// Distinct bench-lock owner for this agent (e.g. claude-main, claude-2). Held around the run.
    #[arg(long, default_value = "hardware-runner")]
    lock_owner: String,
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

/// Path to `tools/bench-lock.sh`, found relative to this crate's manifest dir at build time.
fn bench_lock_script() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|root| root.join("tools/bench-lock.sh"))
        .expect("locate tools/bench-lock.sh")
}

/// Acquire the bench lock (one shared physical bench). Returns Ok only if we got it.
fn bench_lock_acquire(owner: &str) -> Result<(), String> {
    let status = Command::new(bench_lock_script())
        .args(["acquire", owner, "hardware-runner silicon run"])
        .status()
        .map_err(|e| format!("spawn bench-lock acquire: {e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err("bench is locked by another owner".to_string())
    }
}

/// Release the bench lock. Best-effort: a failure to release is logged, not fatal.
fn bench_lock_release(owner: &str) {
    let _ = Command::new(bench_lock_script())
        .args(["release", owner])
        .status();
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();

    // Hold tools/bench-lock.sh around the hardware op (acquire before, release after).
    if let Err(e) = bench_lock_acquire(&args.lock_owner) {
        eprintln!("could not acquire bench lock: {e}");
        return std::process::ExitCode::FAILURE;
    }

    let result = run_image(
        &args.image,
        &args.chip,
        &args.probe,
        args.input,
        Duration::from_millis(args.timeout_ms),
    );

    bench_lock_release(&args.lock_owner);

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
