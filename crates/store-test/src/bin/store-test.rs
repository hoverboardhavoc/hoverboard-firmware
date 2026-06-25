//! store-test: the store firmware image. Reads the host-packed `cmd` from `CMD_ADDR`, runs the store
//! `run` / `run_var` step over the real `FmcFlash` (built from the build-feature-selected `Chip`),
//! publishes a `TestResult`, and busy-spins.
//!
//! ONE image drives the whole scenario x phase matrix: the host writes `cmd = (scenario << 16) |
//! phase` and resets between phases (the store cold-mounts each time, so a survivor is provably from
//! flash). The variable-value scenario publishes `buf`/`len`; every other scenario publishes the
//! scalar `output`.
//!
//! On a host target (where it cannot link as a cortex-m image, nor the target-gated HAL) it degrades
//! to an empty `main`, so a bare host `cargo build`/`cargo test` over the workspace stays green; the
//! real image is only ever built for the chip.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;
    use store::{FmcFlash, VAR_VALUE};
    use store_test::{publish_scalar, publish_var, read_cmd, selected_chip};
    use test_shared::RESULT_BUF_LEN;

    #[entry]
    fn main() -> ! {
        let cmd = read_cmd();
        let scenario = cmd >> 16;

        // Build the FmcFlash over the compile-time-selected Chip (NEVER runtime detect_chip()).
        let chip = selected_chip();
        let mut flash = FmcFlash::new(&chip);

        if scenario == VAR_VALUE {
            // The variable-value scenario carries a multi-byte read-back in TestResult.buf/len.
            let mut buf = [0u8; RESULT_BUF_LEN];
            let n = store::run_var(&mut flash, cmd, &mut buf);
            publish_var(&buf[..n]);
        } else {
            let output = store::run(&mut flash, cmd);
            publish_scalar(output);
        }

        // Busy-spin, NEVER wfi: a wfi park can lock SWD re-attach on the GD32F130 (see the spec).
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
