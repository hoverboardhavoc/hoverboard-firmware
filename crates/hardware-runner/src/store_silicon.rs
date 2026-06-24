//! The tier-3 silicon driver for the store: flash a `store-test` image, run it two-phase across a
//! real reset (the cross-reboot), and assert the persisted value survived.
//!
//! The store test is `set+persist` (phase 0) -> RESET -> `read` (phase 1), and the flash bytes must
//! survive the reset. On silicon the reset IS a real chip reset, so the flash persists naturally (only
//! RAM is cleared by `cortex-m-rt` startup). The driver:
//!
//! 1. flashes the image once (which also erases the store region, a clean slate);
//! 2. delivers phase 0 to `CMD_ADDR`, resets, waits for the run to publish (set + persist);
//! 3. delivers phase 1 to `CMD_ADDR`, resets again (the cross-reboot), waits for the published read;
//! 4. asserts `output == store::T_VAL` (the host recomputes the expected value, non-circular).
//!
//! `chip` / `probe_selector` / `image` come from the caller (CLI), nothing hardcoded. The subject
//! busy-spins after publishing (never `wfi`), so a hung/faulted subject times out rather than locking
//! SWD re-attach on the NRST-less clones.

use std::path::Path;
use std::time::{Duration, Instant};

use probe_rs::flashing::{download_file, Format};
use probe_rs::probe::list::Lister;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::{MemoryInterface, Permissions, Session};
use test_shared::{CMD_ADDR, RESULT_ADDR, RESULT_READY};

use crate::RunError;

/// Deliver `phase` to `CMD_ADDR`, reset, and poll `RESULT_ADDR` (the `ready` word) over SWD until it
/// reads `RESULT_READY` or `timeout` expires. Returns the published `output` word. A timeout means the
/// subject hung or faulted before publishing.
fn run_phase(session: &mut Session, phase: u32, timeout: Duration) -> Result<u32, RunError> {
    {
        let mut core = session
            .core(0)
            .map_err(|e| RunError::Probe(format!("select core: {e}")))?;
        // Deliver the phase to the reserved RAM tail (outside the linked region, so it survives reset).
        core.write_word_32(CMD_ADDR as u64, phase)
            .map_err(|e| RunError::Probe(format!("write CMD_ADDR: {e}")))?;
        // Reset and let it run from the vector table (the cross-reboot between phases).
        core.reset()
            .map_err(|e| RunError::Probe(format!("reset: {e}")))?;
    }

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
            return Ok(output);
        }
        if Instant::now() >= deadline {
            return Err(RunError::Timeout);
        }
        drop(core);
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Flash `image` to `chip` via `probe_selector`, then run it two-phase across a real reset and return
/// the phase-1 read-back (`output`). The caller asserts `output == store::T_VAL`. Hold
/// `tools/bench-lock.sh` around this call (one shared physical bench); see the binary.
pub fn run_two_phase(
    image: &Path,
    chip: &str,
    probe_selector: &str,
    timeout: Duration,
) -> Result<u32, RunError> {
    let selector = DebugProbeSelector::try_from(probe_selector)
        .map_err(|e| RunError::Probe(format!("parse probe selector {probe_selector}: {e}")))?;
    let lister = Lister::new();
    let probe = lister
        .open(selector)
        .map_err(|e| RunError::Probe(format!("open probe {probe_selector}: {e}")))?;
    let mut session = probe
        .attach(chip, Permissions::default())
        .map_err(|e| RunError::Probe(format!("attach to {chip}: {e}")))?;

    // Flash the image (erase + program). This places the vector table + code AND erases the store
    // region to a clean slate.
    download_file(&mut session, image, Format::Elf)
        .map_err(|e| RunError::Probe(format!("flash {}: {e}", image.display())))?;

    // Phase 0: set + persist (the published output is 0, ignored).
    let _ = run_phase(&mut session, 0, timeout)?;
    // Phase 1: cold-mount + read across the reset. The read-back is the persisted value.
    run_phase(&mut session, 1, timeout)
}
