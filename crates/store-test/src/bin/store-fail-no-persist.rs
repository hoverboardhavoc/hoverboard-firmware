//! store-fail-no-persist: the negative control. In phase 0 it sets T_KEY LIVE but SKIPS persist, so
//! after the reset (cold mount) phase 1 reads the registry default, not T_VAL. The host's
//! `output == T_VAL` check then fails and the rig CATCHES it, proving the test verifies flash
//! persistence, not a RAM round-trip (the store's analog of dummy-fail).
//!
//! See store-pass.rs for why a host build degrades to an empty `main`.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use cortex_m::asm::nop;
    use cortex_m_rt::entry;
    use panic_halt as _;
    use store::{Store, Value, T_KEY};
    use store_test::{publish, read_phase, selected_chip, FmcFlash};

    #[entry]
    fn main() -> ! {
        let phase = read_phase();
        let chip = selected_chip();
        let mut flash = FmcFlash::new(&chip);
        // Cold mount (the "reboot"): rebuild the RAM table from flash.
        let mut store = Store::mount(&mut flash);
        let output = match phase {
            // The planted bug: set live but SKIP persist. Nothing reaches flash.
            0 => {
                store.set_live(T_KEY, Value::U32(store::T_VAL));
                0
            }
            // Read after the reboot: the default (0), NOT T_VAL, because phase 0 never persisted.
            _ => store.get(T_KEY).u32(),
        };
        publish(output);
        // Busy-spin, NEVER wfi (see the spec).
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
