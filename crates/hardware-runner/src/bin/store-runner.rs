//! Flash a `store-test` image to a real GD32 over SWD, run it two-phase across a real reset (the
//! cross-reboot), and assert the persisted value survived. Manual, run on the Pi, bench-locked.
//! Compiles for the host; the human runs it on hardware.
//!
//! It takes the chip, the probe selector, and the image path as arguments (nothing hardcoded), holds
//! `tools/bench-lock.sh` around the hardware op (one shared physical bench), and judges the result
//! with the shared `output == store::T_VAL` check (the store's analog of the dummy assertion).
//!
//! The pass image (`store-pass`) reads back `T_VAL` after the reboot; the negative control
//! (`store-fail-no-persist`) reads the default, which this driver reports as a FAIL (caught).

use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use clap::Parser;
use hardware_runner::store_silicon::run_two_phase;
use hardware_runner::RunError;

/// Run a `store-test` image on real GD32 silicon over SWD, two-phase, and assert persistence.
#[derive(Parser, Debug)]
#[command(about, long_about = None)]
struct Args {
    /// The ELF image to flash (a store-pass / store-fail-no-persist release binary).
    image: PathBuf,

    /// probe-rs target/chip name. A GD32 definition with CPUTAPID 0 (the GD32 reports a different DAP
    /// ID than the STM32 part probe-rs assumes).
    #[arg(long)]
    chip: String,

    /// probe-rs probe selector: `VID:PID` or `VID:PID:serial`. Identify probes by full serial, not
    /// USB port (the master F103C8 dapdirect ST-Link; the slave F130C8 dap42 CMSIS-DAP).
    #[arg(long)]
    probe: String,

    /// Per-phase poll timeout in milliseconds. A hung/faulted subject must time out (the ST-Link
    /// clones cannot drive NRST, so we never block forever).
    #[arg(long, default_value_t = 2000)]
    timeout_ms: u64,

    /// Distinct bench-lock owner for this agent (e.g. claude-main, claude-2). Held around the run.
    #[arg(long, default_value = "store-runner")]
    lock_owner: String,
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
        .args(["acquire", owner, "store-runner silicon run"])
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

    if let Err(e) = bench_lock_acquire(&args.lock_owner) {
        eprintln!("could not acquire bench lock: {e}");
        return std::process::ExitCode::FAILURE;
    }

    let result = run_two_phase(
        &args.image,
        &args.chip,
        &args.probe,
        Duration::from_millis(args.timeout_ms),
    );

    bench_lock_release(&args.lock_owner);

    match result {
        Ok(output) => {
            let expected = store::T_VAL;
            if output == expected {
                println!(
                    "PASS: persisted value survived the reboot: output={output:#010x} (== T_VAL)"
                );
                std::process::ExitCode::SUCCESS
            } else {
                eprintln!(
                    "FAIL: read-back {output:#010x} != T_VAL {expected:#010x} (value did not persist)"
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
