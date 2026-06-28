//! The universal firmware binary: ONE image that detects which GD32 it is on at boot and runs
//! everywhere (F103 master, F130 slave, 12-FET). There is no per-part build, the binary detects its
//! silicon at runtime and adapts (specs/firmware.md).
//!
//! It is thin: a `main` that wires together libraries it does not own (`store` + its `FmcFlash`, and
//! `runtime-hal`'s `detect_chip`). MVP scope is **boot safe -> detect -> mount the store -> the bare
//! housekeeping loop**; the link and the control layers fill in later (roadmap.md L6-L9).
//!
//! Boot sequence (specs/firmware.md, "Boot sequence"):
//!   1. cortex-m-rt reset (SP/PC set, .data/.bss initialized) -- before `main`.
//!   2. Boot safe: nothing that could drive a motor is touched. The MVP has no motor code at all;
//!      the gate is implicit (we never arm a bridge), but the rule exists first by design.
//!   3. `detect_chip()` -- fail loud if detection fails: the firmware cannot run without knowing its
//!      silicon, so a failed detect panics (panic-halt) rather than guessing a register layout.
//!   4. `Store::mount(FmcFlash::new(&chip))` -- the store replays its log; absent keys read defaults.
//!   5. The bare housekeeping loop.
//!
//! On a host target (where it cannot link as a cortex-m image, nor the target-gated HAL) it degrades
//! to an empty `main`, so a host `cargo build`/`cargo test` over the workspace stays green; the real
//! image is only ever built for the chip. (Same degrade pattern as store-test / dummy-test.)

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;
    use runtime_hal::detect_chip;
    use store::{FmcFlash, Store};
    use swd_mailbox::{Mailbox, MAILBOX_BASE};

    #[entry]
    fn main() -> ! {
        // Boot safe: nothing that could drive a motor is touched (no motor code in the MVP).

        // Initialize the SWD mailbox header FIRST, before any bridge could attach: write
        // magic/version/offsets/caps and zero the indices + epoch/epoch_ack. The mailbox occupies a
        // fixed RESERVED region [MAILBOX_BASE, +REGION_LEN) at the bottom of SRAM (memory.x starts the
        // linked RAM above it), so it is indeterminate at reset and the linker never touches it; without
        // this init the bridge's magic check (Attach step 2) reads garbage. SAFETY: the reserved region
        // is REGION_LEN bytes at the fixed base, owned only here; accessed only through volatile
        // reads/writes via the handle.
        let mailbox = unsafe { Mailbox::from_raw(MAILBOX_BASE as *mut u8) };
        mailbox.init_header();

        // Detect the silicon. Fail loud: the firmware cannot run without knowing its part, so a
        // failed detect panics (panic-halt) rather than guessing a register layout.
        let chip = detect_chip().unwrap();

        // Mount the store at the detected top-of-flash. FmcFlash derives the absolute store region
        // from the Chip; mount replays the log (absent keys read defaults).
        let mut flash = FmcFlash::new(&chip);
        let _store = Store::mount(&mut flash).unwrap();

        // The bare housekeeping main loop: empty in the MVP (no link, no control tick to service).
        // Structured so the future 250 Hz control ISR can preempt it. Busy-spin, NEVER wfi: a wfi
        // park with no DBG_CTL0 debug-hold bits locks SWD re-attach on the GD32F130 (see the spec).
        loop {
            nop(); // housekeeping slot: empty in the MVP, preemptible by the future control ISR
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
