//! dummy-hang: spins immediately and never publishes a result, so `ready` never reaches
//! RESULT_READY. The host's bound (the emulator's instruction cap, the silicon driver's wall-clock
//! timeout) must fire and the rig must report this as a caught failure.
//!
//! See dummy-pass.rs for why a host build degrades to an empty `main`.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;

    #[entry]
    fn main() -> ! {
        // Never writes RESULT_ADDR. Busy-spin, NEVER wfi (see the spec).
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
