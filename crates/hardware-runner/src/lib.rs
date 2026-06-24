//! Run a dummy firmware image on real GD32 silicon over SWD and read the result.
//!
//! This is the authoritative bench oracle: the same image and the same RAM result/command channel
//! the emulator uses, but on a real part. It flashes the image, writes the host input to `CMD_ADDR`,
//! resets, polls `RESULT_ADDR` (the `ready` word) over SWD until it reads `RESULT_READY` or a
//! wall-clock timeout expires, reads the [`TestResult`] back, and the caller judges it with the same
//! `output == dummy(input)` check the emulator uses.
//!
//! The subject busy-spins after publishing (never `wfi`): a `wfi` park can lock SWD re-attach on the
//! GD32F130, and the ST-Link clones do not drive NRST, so a hung subject must be recoverable by a
//! plain poll timeout rather than waiting forever.
//!
//! The probe-rs SWD path is behind the `probe` feature (it links the native probe stack). Without it
//! the crate is an empty host library.

#![cfg(feature = "probe")]

use std::path::Path;
use std::time::{Duration, Instant};

use probe_rs::flashing::{download_file, Format};
use probe_rs::probe::list::Lister;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::{MemoryInterface, Permissions};
use test_shared::{dummy, TestResult, CMD_ADDR, RESULT_ADDR, RESULT_READY};

/// How the silicon run can fail before it even reaches a judgment.
#[derive(Debug)]
pub enum RunError {
    /// Opening the probe, flashing, resetting, or an SWD transfer failed.
    Probe(String),
    /// The subject never published `RESULT_READY` within the timeout (it hung or faulted). This is
    /// the `dummy-hang` outcome; the caller treats it as a caught failure.
    Timeout,
}

impl std::fmt::Display for RunError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RunError::Probe(s) => write!(f, "probe/SWD error: {s}"),
            RunError::Timeout => write!(f, "subject did not publish a result within the timeout"),
        }
    }
}

impl std::error::Error for RunError {}

/// Flash `image` to `chip` via the probe selected by `probe_selector`, deliver `input`, reset, and
/// poll for the result over SWD until `timeout` elapses.
///
/// `chip` is a probe-rs target name (e.g. a GD32F1x0 part); `probe_selector` is a probe-rs selector
/// string (`VID:PID` or `VID:PID:serial`). Both come from the caller (CLI), nothing is hardcoded.
///
/// Bench note: the GD32 reports a different DAP ID than the STM32 target probe-rs assumes, so a chip
/// definition with `cpu_tap_id`/CPUTAPID 0 must be selected via `chip`. Hold `tools/bench-lock.sh`
/// around this call (one shared physical bench); see the binary.
pub fn run_image(
    image: &Path,
    chip: &str,
    probe_selector: &str,
    input: u32,
    timeout: Duration,
) -> Result<TestResult, RunError> {
    let selector = DebugProbeSelector::try_from(probe_selector)
        .map_err(|e| RunError::Probe(format!("parse probe selector {probe_selector}: {e}")))?;
    let lister = Lister::new();
    let probe = lister
        .open(selector)
        .map_err(|e| RunError::Probe(format!("open probe {probe_selector}: {e}")))?;

    let mut session = probe
        .attach(chip, Permissions::default())
        .map_err(|e| RunError::Probe(format!("attach to {chip}: {e}")))?;

    // Flash the image (erase + program). This places the vector table and code in flash.
    download_file(&mut session, image, Format::Elf)
        .map_err(|e| RunError::Probe(format!("flash {}: {e}", image.display())))?;

    {
        let mut core = session
            .core(0)
            .map_err(|e| RunError::Probe(format!("select core: {e}")))?;

        // Deliver the host input to the reserved RAM tail. It is outside the linked RAM region, so it
        // survives the reset and the startup .bss clear; the device reads it after reset.
        core.write_word_32(CMD_ADDR as u64, input)
            .map_err(|e| RunError::Probe(format!("write CMD_ADDR: {e}")))?;

        // Reset and let it run from the vector table.
        core.reset()
            .map_err(|e| RunError::Probe(format!("reset: {e}")))?;
    }

    // Poll `ready` over SWD with a wall-clock timeout (the ST-Link clones cannot drive NRST, so a hung
    // subject must time out rather than block forever).
    let deadline = Instant::now() + timeout;
    loop {
        let mut core = session
            .core(0)
            .map_err(|e| RunError::Probe(format!("select core: {e}")))?;
        let ready = core
            .read_word_32(RESULT_ADDR as u64)
            .map_err(|e| RunError::Probe(format!("read RESULT_ADDR: {e}")))?;
        if ready == RESULT_READY {
            let output = core
                .read_word_32(RESULT_ADDR as u64 + 4)
                .map_err(|e| RunError::Probe(format!("read output: {e}")))?;
            return Ok(TestResult { ready, output });
        }
        if Instant::now() >= deadline {
            return Err(RunError::Timeout);
        }
        drop(core);
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// The host-side judgment, identical to the emulator's: run the image on silicon and decide whether
/// it produced the correct output. The expected value comes from the shared [`dummy`]. A timeout
/// (the subject hung) or a wrong output returns false.
pub fn produced_correct_output(
    image: &Path,
    chip: &str,
    probe_selector: &str,
    input: u32,
    timeout: Duration,
) -> bool {
    match run_image(image, chip, probe_selector, input, timeout) {
        Ok(r) => r.ready == RESULT_READY && r.output == dummy(input),
        Err(_) => false,
    }
}
