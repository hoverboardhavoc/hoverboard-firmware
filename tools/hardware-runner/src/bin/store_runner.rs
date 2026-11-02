//! Drive the store's Tier-3 silicon scenarios on a real 1 KiB GD32 bench part over SWD, host-judging
//! each against the store's own value consts. The silicon counterpart of the emulator's
//! `store_scenarios` matrix (same `(scenario, phase)` ids, same crafted regions).
//!
//! Manual, run on the Pi. It takes the chip, probe selector, and `store-test` image path as arguments
//! (nothing hardcoded), flashes the image ONCE, then runs the requested scenario's phases (pre-erasing the region for a clean slate or
//! planting a crafted region as the scenario needs), and judges the device's read-back on the host.
//!
//! The 12-FET 2 KiB part is a DEFERRED bench task (OpenOCD + ESP32 elaphureLink, which probe-rs has no
//! backend for); this binary drives only the 1 KiB-page parts (`--features gd32f103c8` or `gd32f130c8`). The 2 KiB-page logic is
//! CI-covered by the emulated GD32F103RC path.
//!
//! It takes no bench lock: the operator takes the resource of the board behind `--probe` first with
//! `~/dev/hoverboard-bench/bench-lock.sh`, and `probes` for a `VID:PID` selector with no serial,
//! which is a scan.

use std::path::PathBuf;
use std::time::Duration;

use clap::{Parser, ValueEnum};
use hardware_runner::{build_planted_region, Part, RunError, StoreDriver};
use store::{
    COMPACT, FULL, PERSIST, TORN_HEADER, TORN_PAYLOAD, T_BLOB_VAL, T_STR_VAL, T_VAL, VAR_VALUE,
};
use test_shared::{TestResult, RESULT_READY};

/// The store scenarios this driver can run on a 1 KiB silicon part (the host-driven matrix mirror).
#[derive(Copy, Clone, Debug, ValueEnum)]
enum Scenario {
    /// Persist-survives-reboot: phase 0 set + persist, phase 1 cold-mount + read == T_VAL.
    Persist,
    /// The no-write negative control: ONLY the read phase; the read returns the default, not T_VAL.
    NoWrite,
    /// Variable-value round trip (device-written): set_str/set_bytes, then read each back.
    VarValue,
    /// Compaction preserves keys: host plants a multi-record region; latest-per-key T_VAL survives.
    Compact,
    /// Torn-payload recovery: host plants a half-written payload; the last good value (T_VAL) reads.
    TornPayload,
    /// Torn-header auto-compaction: host plants a torn header; the survivor (T_VAL) reads back.
    TornHeader,
    /// Full -> compact -> retry: host plants a near-full page; the device set returns Full, compacts,
    /// retries; phase 1 reads back T_VAL.
    Full,
}

/// Run the store Tier-3 scenarios on real GD32 silicon over SWD and assert the read-back.
#[derive(Parser, Debug)]
#[command(about, long_about = None)]
struct Args {
    /// The `store-test` ELF image to flash (built `--features gd32f103c8` or `gd32f130c8` for the 1 KiB bench parts).
    image: PathBuf,

    /// The scenario to run.
    #[arg(long, value_enum)]
    scenario: Scenario,

    /// probe-rs target/chip name. A GD32 definition with CPUTAPID 0.
    #[arg(long)]
    chip: String,

    /// probe-rs probe selector: `VID:PID` or `VID:PID:serial` (the master F103 via its ST-Link, the
    /// slave F130 via the dap42 CMSIS-DAP; identify by full serial, not USB port).
    #[arg(long)]
    probe: String,

    /// Poll timeout in milliseconds. A hung/faulted subject must time out (the ST-Link clones cannot
    /// drive NRST, so we never block forever).
    #[arg(long, default_value_t = 4000)]
    timeout_ms: u64,
}

fn cmd(scenario: u32, phase: u32) -> u32 {
    (scenario << 16) | (phase & 0xFFFF)
}

/// The whole scenario, judged on the host. `Ok(())` is a pass; `Err(msg)` is a caught failure.
fn run_scenario(driver: &mut StoreDriver, scenario: Scenario) -> Result<(), String> {
    match scenario {
        Scenario::Persist => {
            // Clean slate, then set + persist (phase 0), reboot + read (phase 1) == T_VAL.
            erase(driver)?;
            let _ = phase(driver, cmd(PERSIST, 0))?;
            let r = phase(driver, cmd(PERSIST, 1))?;
            expect_ready(&r)?;
            expect_scalar(&r, T_VAL, "persisted value did not survive the reboot")
        }
        Scenario::NoWrite => {
            // The store analog of dummy-fail: ONLY the read phase, never the set. The read returns the
            // default, not T_VAL, so a vacuous pass is caught (here a wrong T_VAL would be the failure).
            erase(driver)?;
            let r = phase(driver, cmd(PERSIST, 1))?;
            expect_ready(&r)?;
            if r.output == T_VAL {
                Err("no write happened, yet T_VAL was read (vacuous pass)".to_string())
            } else {
                Ok(())
            }
        }
        Scenario::VarValue => {
            // Device-written variable values: phase 0 set_str + set_bytes; phase 1 get_text; phase 2
            // get_bytes. The host compares the buf bytes byte-identically.
            erase(driver)?;
            let _ = phase(driver, cmd(VAR_VALUE, 0))?;
            let r = phase(driver, cmd(VAR_VALUE, 1))?;
            expect_ready(&r)?;
            expect_bytes(&r, T_STR_VAL.as_bytes(), "STR round-trip mismatch")?;
            let r = phase(driver, cmd(VAR_VALUE, 2))?;
            expect_ready(&r)?;
            expect_bytes(&r, T_BLOB_VAL, "BLOB round-trip mismatch")
        }
        Scenario::Compact => planted_read(driver, COMPACT, "compaction lost the latest T_KEY"),
        Scenario::TornPayload => planted_read(
            driver,
            TORN_PAYLOAD,
            "torn-payload recovery did not read the last good value",
        ),
        Scenario::TornHeader => planted_read(
            driver,
            TORN_HEADER,
            "torn-header auto-compaction lost the survivor",
        ),
        Scenario::Full => {
            // Host plants a near-full active page; the device set(T_KEY) returns Full -> compact ->
            // retry (phase 0, all on-device); phase 1 reads back T_VAL.
            plant(driver, FULL)?;
            let r0 = phase(driver, cmd(FULL, 0))?;
            expect_ready(&r0)?;
            let r1 = phase(driver, cmd(FULL, 1))?;
            expect_ready(&r1)?;
            expect_scalar(&r1, T_VAL, "Full->compact->retry did not persist T_VAL")
        }
    }
}

/// A host-planted recovery scenario whose device step is just a cold-mount + read of the survivor.
fn planted_read(driver: &mut StoreDriver, scenario: u32, lost_msg: &str) -> Result<(), String> {
    plant(driver, scenario)?;
    let r = phase(driver, cmd(scenario, 1))?;
    expect_ready(&r)?;
    expect_scalar(&r, T_VAL, lost_msg)
}

/// Pre-erase the store region for a clean slate.
fn erase(driver: &mut StoreDriver) -> Result<(), String> {
    driver
        .erase_region()
        .map_err(|e| format!("region erase: {e}"))
}

/// Build the crafted region via the SHARED store builders and plant it (byte-identical to the
/// emulator's planted region).
fn plant(driver: &mut StoreDriver, scenario: u32) -> Result<(), String> {
    let image = build_planted_region(scenario, Part::CHIP1K)
        .ok_or_else(|| format!("scenario {scenario} is not host-planted"))?;
    driver
        .load_region(&image)
        .map_err(|e| format!("plant crafted region: {e}"))
}

/// Run one phase, mapping a probe error / timeout into a caught failure string.
fn phase(driver: &mut StoreDriver, c: u32) -> Result<TestResult, String> {
    match driver.run_phase(c) {
        Ok(r) => Ok(r),
        Err(RunError::Timeout) => Err("subject hung (no result within timeout)".to_string()),
        Err(e) => Err(format!("probe error: {e}")),
    }
}

fn expect_ready(r: &TestResult) -> Result<(), String> {
    if r.ready == RESULT_READY {
        Ok(())
    } else {
        Err("phase did not publish ready".to_string())
    }
}

fn expect_scalar(r: &TestResult, expected: u32, msg: &str) -> Result<(), String> {
    if r.output == expected {
        Ok(())
    } else {
        Err(format!(
            "{msg}: got {:#010x}, expected {expected:#010x}",
            r.output
        ))
    }
}

fn expect_bytes(r: &TestResult, expected: &[u8], msg: &str) -> Result<(), String> {
    let got = &r.buf[..r.len as usize];
    if got == expected {
        Ok(())
    } else {
        Err(format!("{msg}: got {got:02x?}, expected {expected:02x?}"))
    }
}

fn main() -> std::process::ExitCode {
    let args = Args::parse();

    // The 1 KiB parts are the only silicon this driver runs (the 12-FET 2 KiB part is a deferred bench task
    // over OpenOCD + ESP32 elaphureLink, which probe-rs has no backend for).
    let result = (|| -> Result<(), String> {
        let mut driver = StoreDriver::attach(
            &args.image,
            &args.chip,
            &args.probe,
            Part::CHIP1K,
            Duration::from_millis(args.timeout_ms),
        )
        .map_err(|e| format!("attach/flash: {e}"))?;
        run_scenario(&mut driver, args.scenario)
    })();

    match result {
        Ok(()) => {
            println!("PASS: scenario {:?}", args.scenario);
            std::process::ExitCode::SUCCESS
        }
        Err(msg) => {
            eprintln!("FAIL: scenario {:?}: {msg}", args.scenario);
            std::process::ExitCode::FAILURE
        }
    }
}
