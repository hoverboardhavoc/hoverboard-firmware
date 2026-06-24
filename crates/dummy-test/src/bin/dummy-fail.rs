//! dummy-fail: a planted bug. Forgets the transform and publishes the raw input as the output, so
//! the host's `output == dummy(input)` check sees the wrong value and the rig must catch it.
//!
//! See dummy-pass.rs for why a host build degrades to an empty `main`.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use dummy_test::{publish, read_input};
    use panic_halt as _;

    #[entry]
    fn main() -> ! {
        let input = read_input();
        publish(input); // the bug: should be dummy(input)
                        // Busy-spin, NEVER wfi (see the spec).
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
