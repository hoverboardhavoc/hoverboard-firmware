//! dummy-pass: the correct image. Reads the input, applies the transform, publishes dummy(input).
//!
//! This is a thumbv7m firmware binary. On a host target (where it cannot link as a cortex-m image)
//! it degrades to an empty `main` so a bare host `cargo build`/`cargo test` over the workspace stays
//! green; the real image is only ever built for the chip.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use dummy_test::{publish, read_input};
    use panic_halt as _;
    use test_shared::dummy;

    #[entry]
    fn main() -> ! {
        let input = read_input();
        publish(dummy(input));
        // Busy-spin, NEVER wfi: a wfi park can lock SWD re-attach on the GD32F130 (see the spec).
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
