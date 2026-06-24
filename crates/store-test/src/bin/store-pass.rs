//! store-pass: the correct store image. Reads the phase from `CMD_ADDR`, runs `store::run_phase` over
//! the real `FmcFlash` (the HAL `Fmc` adapter), and publishes the read-back as `TestResult.output`
//! with `ready` written LAST.
//!
//! phase 0 = set T_KEY + persist (returns 0); phase 1 = cold-mount + read T_KEY (returns the value).
//! The host writes the phase before each run and resets between phases (the flash survives, only RAM
//! is cleared), so phase 1's cold mount reads the value provably from flash, not RAM.
//!
//! On a host target it degrades to an empty `main` so a bare host `cargo build`/`cargo test` over the
//! workspace stays green; the real image is only ever built for the chip.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;
    use store::run_phase;
    use store_test::{publish, read_phase, selected_chip, FmcFlash};

    #[entry]
    fn main() -> ! {
        let phase = read_phase();
        let chip = selected_chip();
        let mut flash = FmcFlash::new(&chip);
        let output = run_phase(&mut flash, phase);
        publish(output);
        // Busy-spin, NEVER wfi: a wfi park can lock SWD re-attach on the GD32F130 (see the spec).
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
