//! The store's Tier-3 silicon driver: a two-phase, persistent-flash probe-rs runner that matches the
//! emulator's host-driven `(scenario, phase)` matrix on a real GD32.
//!
//! The dummy's single-phase [`run_image`](crate::run_image) flashes, delivers one input, resets, and
//! polls once. The store needs more, all of it here:
//!
//! 1. **Two phases over one flashed image.** Flash the `store-test` image ONCE, run phase 0 (write
//!    `(scenario, phase=0)` to `CMD_ADDR`, reset, poll `RESULT_ADDR`), then reset again WITHOUT
//!    re-flashing and run phase 1, reading back the extended [`TestResult`] (incl. `len` / `buf` for
//!    the variable-value scenarios). Flash content (the store region) persists across the phase-1
//!    reset: the image is not re-downloaded and the store region is not erased between phases. This is
//!    the silicon analog of the emulator's [`StoreEmu::run_phase`].
//! 2. **Store-region pre-erase.** Before a clean-slate scenario, erase the store region (the top two
//!    pages) over the probe so the device cold-mounts a virgin region. The region is derived the same
//!    way the store does: `store_base = (FLASH_BASE + flash_size) - 2 * page_size`. `page_size` /
//!    `flash_size` are driver INPUTS (per the selected part), not hardcoded.
//! 3. **Crafted-region load.** For the planted torn-write / `Full` / compaction scenarios, write a
//!    codec-built store-region byte image into flash at `store_base` before the read phase. The bytes
//!    come from the SHARED [`store::scenarios`] builders, the exact same code the emulator runner
//!    plants, so mock / Unicorn / silicon are byte-identical (DoD #5).
//!
//! The two 1 KiB bench parts (F103 master via ST-Link, F130 slave via dap42 CMSIS-DAP) run the
//! `--features chip1k` image over this driver. The 12-FET 2 KiB part is a DEFERRED bench task (see
//! [`Chip2kDeferred`]).
//!
//! Bench note: hold `tools/bench-lock.sh` around any call here (one shared physical bench); the binary
//! does. The subject busy-spins after publishing (never `wfi`), so a hung subject is caught by the poll
//! timeout (the ST-Link clones cannot drive NRST).

use std::path::Path;
use std::time::{Duration, Instant};

use probe_rs::flashing::{download_file, DownloadOptions, Format};
use probe_rs::probe::list::Lister;
use probe_rs::probe::DebugProbeSelector;
use probe_rs::{MemoryInterface, Permissions, Session};
use test_shared::{TestResult, CMD_ADDR, RESULT_ADDR, RESULT_BUF_LEN, RESULT_READY};

use crate::RunError;

/// GD32 flash base. The store region is the top two pages of the part's flash EXTENT.
const FLASH_BASE: u64 = 0x0800_0000;

/// The geometry of the store region for the selected part, derived exactly as the on-target
/// `FmcFlash` does (`store_base = (FLASH_BASE + flash_size) - 2 * page_size`). `page_size` and
/// `flash_size` are part-specific driver inputs, NOT hardcoded: the bench parts pick them on the CLI,
/// consistent with how the `store-test` image picks its `Chip` by build feature (`chip1k` / `chip2k`).
///
/// chip1k: `page_size = 1024`, `flash_size = 64 KiB` -> `store_base = 0x0800_F800`.
/// chip2k: `page_size = 2048`, `flash_size = 256 KiB` -> `store_base = 0x0803_F000` (deferred silicon).
#[derive(Clone, Copy, Debug)]
pub struct Part {
    /// The detected page size in bytes (1024 or 2048).
    pub page_size: usize,
    /// The flash EXTENT in bytes (64 KiB for chip1k, 256 KiB for chip2k).
    pub flash_size: u32,
}

impl Part {
    /// The 1 KiB-page bench parts (F103 master, F130 slave): 64 KiB flash, 1 KiB pages.
    pub const CHIP1K: Part = Part {
        page_size: 1024,
        flash_size: 64 * 1024,
    };

    /// The 2 KiB-page 12-FET part (GD32F103RC): 256 KiB flash, 2 KiB pages. Its SILICON run is a
    /// DEFERRED bench task (see [`Chip2kDeferred`]); this constant only documents the geometry and lets
    /// the host-side region builders be exercised for it. The 2 KiB-page LOGIC is already CI-covered by
    /// the emulated `chip2k` path (Tier 2).
    pub const CHIP2K: Part = Part {
        page_size: 2048,
        flash_size: 256 * 1024,
    };

    /// The absolute base of the store region (`(FLASH_BASE + flash_size) - 2 * page_size`), the same
    /// derivation the store's `FmcFlash` uses, so the driver plants/erases exactly where the device
    /// cold-mounts.
    pub fn store_base(&self) -> u64 {
        (FLASH_BASE + self.flash_size as u64) - 2 * self.page_size as u64
    }

    /// The store region length in bytes (`2 * page_size`).
    pub fn region_len(&self) -> usize {
        2 * self.page_size
    }
}

/// The 12-FET 2 KiB silicon oracle is a DEFERRED bench task, NOT implemented here.
///
/// Per the storage spec (Tier 3), the 2 KiB-page part (GD32F103RC, 256 KiB) would be driven over
/// **OpenOCD + the ESP32 elaphureLink** WiFi-SWD bridge, for which `probe-rs` has NO backend
/// (elaphureLink is not a probe-rs transport), and which is gated on bench hardware. The 2 KiB-page
/// *logic* is already CI-covered by the emulated `chip2k` path (Tier 2, `emulator-runner`), so only the
/// on-silicon oracle is deferred, not the coverage.
///
/// This is a deliberate, documented gap, NOT a broken stub: the host-side region geometry for the part
/// is available via [`Part::CHIP2K`] (so the crafted-region builders can be exercised for it), but the
/// silicon transport is intentionally absent. When a probe-rs elaphureLink backend (or a different
/// 2 KiB-page part on a probe-rs-supported transport) exists, wire it through [`StoreDriver`] with
/// [`Part::CHIP2K`]; nothing else in this driver assumes 1 KiB.
//
// DEFERRED: 12-FET (GD32F103RC, 2 KiB) silicon over OpenOCD + ESP32 elaphureLink. probe-rs has no
// elaphureLink backend; gated on bench hardware. The 2 KiB logic is CI-covered by the emulated chip2k
// path. See specs/storage-layer.md Tier 3.
#[derive(Debug)]
#[non_exhaustive]
pub struct Chip2kDeferred;

/// A two-phase silicon driver over one attached probe session, flashed once with the `store-test`
/// image. Phases run by resetting WITHOUT re-flashing, so the store region in flash persists.
pub struct StoreDriver {
    session: Session,
    part: Part,
    timeout: Duration,
}

impl StoreDriver {
    /// Attach to `chip` via `probe_selector`, flash the `store-test` `image` ONCE, and prepare to run
    /// phases against `part`'s store geometry. The image stays resident across every later phase reset.
    ///
    /// `chip` is a probe-rs target name (a GD32 definition with CPUTAPID 0); `probe_selector` is a
    /// probe-rs selector (`VID:PID` or `VID:PID:serial`). Both come from the caller (CLI), as in
    /// [`run_image`](crate::run_image). `part` is [`Part::CHIP1K`] for the bench parts (chip2k silicon
    /// is deferred, see [`Chip2kDeferred`]).
    pub fn attach(
        image: &Path,
        chip: &str,
        probe_selector: &str,
        part: Part,
        timeout: Duration,
    ) -> Result<Self, RunError> {
        let selector = DebugProbeSelector::try_from(probe_selector)
            .map_err(|e| RunError::Probe(format!("parse probe selector {probe_selector}: {e}")))?;
        let lister = Lister::new();
        let probe = lister
            .open(selector)
            .map_err(|e| RunError::Probe(format!("open probe {probe_selector}: {e}")))?;
        let mut session = probe
            .attach(chip, Permissions::default())
            .map_err(|e| RunError::Probe(format!("attach to {chip}: {e}")))?;

        // Flash the image ONCE (erase + program). Every later phase resets WITHOUT re-flashing, so the
        // store region survives across phases (the whole point of the two-phase run).
        download_file(&mut session, image, Format::Elf)
            .map_err(|e| RunError::Probe(format!("flash {}: {e}", image.display())))?;

        Ok(Self {
            session,
            part,
            timeout,
        })
    }

    /// Pre-erase the store region (the top two pages) so the device cold-mounts a virgin region. Writes
    /// `region_len` bytes of `0xFF` at `store_base` via the flash loader, which erases the affected
    /// sectors and programs the erased pattern, leaving the region all-`0xFFFF` (the erased NOR state
    /// the store's virgin-region logic expects). The flashed `store-test` code (low flash) is untouched
    /// (the region is above the app slot).
    pub fn erase_region(&mut self) -> Result<(), RunError> {
        let blank = vec![0xFFu8; self.part.region_len()];
        self.load_flash(self.part.store_base(), &blank)
    }

    /// Plant a crafted store-region byte image into flash at `store_base` (the host-planted scenarios).
    /// `image.len()` must equal `region_len`. The device then cold-mounts this region and reports
    /// recovery. The bytes MUST come from the shared [`store::scenarios`] builders so they are
    /// byte-identical to the emulator's planted region (use [`build_planted_region`]).
    pub fn load_region(&mut self, image: &[u8]) -> Result<(), RunError> {
        if image.len() != self.part.region_len() {
            return Err(RunError::Probe(format!(
                "crafted region image must be exactly two pages ({} bytes), got {}",
                self.part.region_len(),
                image.len()
            )));
        }
        self.load_flash(self.part.store_base(), image)
    }

    /// Program `bytes` to flash at absolute `addr` via a `FlashLoader` (erase the affected sectors +
    /// program). Used for both the region pre-erase and the crafted-region load.
    fn load_flash(&mut self, addr: u64, bytes: &[u8]) -> Result<(), RunError> {
        let mut loader = self.session.target().flash_loader();
        loader
            .add_data(addr, bytes)
            .map_err(|e| RunError::Probe(format!("stage flash data at {addr:#010x}: {e}")))?;
        loader
            .commit(&mut self.session, DownloadOptions::default())
            .map_err(|e| RunError::Probe(format!("commit flash data at {addr:#010x}: {e}")))?;
        Ok(())
    }

    /// Run one phase: deliver `cmd = (scenario << 16) | phase` to `CMD_ADDR`, reset WITHOUT
    /// re-flashing (flash persists), poll `RESULT_ADDR` until `RESULT_READY` or the timeout, and read
    /// the extended [`TestResult`] back (ready @ +0, output @ +4, len @ +8, buf @ +10). A timeout (a
    /// hung/faulted subject) returns [`RunError::Timeout`], the caught-failure outcome.
    pub fn run_phase(&mut self, cmd: u32) -> Result<TestResult, RunError> {
        {
            let mut core = self
                .session
                .core(0)
                .map_err(|e| RunError::Probe(format!("select core: {e}")))?;
            // Deliver the host cmd to the reserved RAM tail (outside the linked RAM region, so it
            // survives the reset and the startup .bss clear; the device reads it after reset).
            core.write_word_32(CMD_ADDR as u64, cmd)
                .map_err(|e| RunError::Probe(format!("write CMD_ADDR: {e}")))?;
            // Reset and run from the vector table. The image is NOT re-flashed, so the store region in
            // flash persists across this reset (the two-phase invariant).
            core.reset()
                .map_err(|e| RunError::Probe(format!("reset: {e}")))?;
        }

        let deadline = Instant::now() + self.timeout;
        loop {
            let mut core = self
                .session
                .core(0)
                .map_err(|e| RunError::Probe(format!("select core: {e}")))?;
            let ready = core
                .read_word_32(RESULT_ADDR as u64)
                .map_err(|e| RunError::Probe(format!("read RESULT_ADDR: {e}")))?;
            if ready == RESULT_READY {
                return read_result(&mut core, ready);
            }
            if Instant::now() >= deadline {
                return Err(RunError::Timeout);
            }
            drop(core);
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}

/// Read the full extended [`TestResult`] back over SWD by byte offset: `output` (+4), `len` (+8), and
/// `buf` (+10), the variable-value channel the store's `get_str` / `get_bytes` cases publish.
fn read_result(core: &mut probe_rs::Core<'_>, ready: u32) -> Result<TestResult, RunError> {
    let output = core
        .read_word_32(RESULT_ADDR as u64 + 4)
        .map_err(|e| RunError::Probe(format!("read output: {e}")))?;
    // len is a u16 at +8; read the two bytes and the buf bytes at +10 in one block.
    let mut tail = [0u8; 2 + RESULT_BUF_LEN];
    core.read_8(RESULT_ADDR as u64 + 8, &mut tail)
        .map_err(|e| RunError::Probe(format!("read len/buf: {e}")))?;
    let len = u16::from_le_bytes([tail[0], tail[1]]);
    let mut buf = [0u8; RESULT_BUF_LEN];
    buf.copy_from_slice(&tail[2..]);
    Ok(TestResult {
        ready,
        output,
        len,
        buf,
    })
}

/// Build the crafted store-region image for a host-planted `scenario` (COMPACT / TORN_* / FULL) into a
/// fresh `Vec`, via the SHARED [`store::scenarios`] builders, so the planted region is byte-identical
/// to the one the emulator runner plants AND to what the firmware itself writes. Returns `None` for a
/// device-driven scenario (PERSIST / VAR_VALUE) that plants nothing.
///
/// This is the silicon side of DoD #5: the very same code (`store::scenarios::build_planted_region`)
/// drives both tiers, so byte-identity is by construction, not convention.
pub fn build_planted_region(scenario: u32, part: Part) -> Option<Vec<u8>> {
    let mut buf = vec![0u8; part.region_len()];
    if store::scenarios::build_planted_region(scenario, &mut buf, part.page_size) {
        Some(buf)
    } else {
        None
    }
}
