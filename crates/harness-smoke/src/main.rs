//! The harness rig's own self-test subject: the simplest possible test image, touching no
//! peripheral, so a green run proves the whole pipeline (build -> command delivery -> run ->
//! result read-back -> verdict) before any real consumer test is trusted.
//!
//! On boot it reads the command word the harness wrote to [`harness_abi::CMD_ADDR`], computes
//! `echo = smoke(cmd)`, self-checks with the shared `smoke_ok`, and publishes [`SmokeObs`] to
//! [`harness_abi::RESULT_ADDR`] with `magic` written LAST. The XOR in `smoke` means a correct
//! echo proves the subject actually read the delivered word, not a constant. It then busy-spins
//! (NOT `wfi`: a `wfi` park with no DBGMCU debug-low-power bits locks SWD re-attach on the
//! GD32F130, an unrecoverable brick on the NRST-less clones; see `~/.claude/CLAUDE.md`).
//!
//! The command/result words live in a reserved RAM tail carved by `memory.x` (RAM shrunk to end
//! at the tail). The subject does NOT use a `#[link_section]` static at RAM origin: that collides
//! with cortex-m-rt's startup `.bss` clear (observed to bus-fault on silicon). It reads/writes the
//! fixed addresses directly with volatile accesses.
//!
//! One image, two executors: the same `.bin`/`.elf` runs under Unicorn in CI (`harness-emu`) and
//! on real GD32 silicon over SWD on the bench (`bench-runner`), reaching its verdict the same way.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

// --- bare-metal subject (thumbv7m) -------------------------------------------------------------
//
// The whole bare-metal body is gated on cfg(target_os = "none") so the workspace host build can
// still compile this crate (the host stub below provides an empty `main`); this is the established
// pattern that keeps a host `cargo test` of the workspace green without a cortex-m-rt host target.
#[cfg(target_os = "none")]
mod bare {
    use cortex_m_rt::entry;
    use harness_abi::{
        smoke, smoke_ok, SmokeObs, CMD_ADDR, RESULT_ADDR, SMOKE_MAGIC, VERDICT_FAIL, VERDICT_PASS,
    };
    use panic_halt as _;

    #[entry]
    fn main() -> ! {
        // Read the command word the harness delivered to the fixed tail address. Volatile so the
        // optimiser cannot assume a value for memory it cannot see the harness write.
        // SAFETY: CMD_ADDR is in the reserved RAM tail (carved by memory.x); a plain aligned load.
        let cmd: u32 = unsafe { core::ptr::read_volatile(CMD_ADDR as *const u32) };

        let echo = smoke(cmd);
        let verdict = if smoke_ok(cmd, echo) {
            VERDICT_PASS
        } else {
            VERDICT_FAIL
        };

        // Publish the result struct, writing magic LAST so a reader that sees SMOKE_MAGIC at
        // offset 0 knows echo + verdict are already committed. Field-addressed volatile stores so
        // the optimiser cannot reorder magic ahead of the other two or elide them.
        // SAFETY: RESULT_ADDR is the start of the reserved tail; this path is the only writer,
        // reads are external (the harness, by SWD or mapped RAM). #[repr(C)] fixes the offsets.
        let p = RESULT_ADDR as *mut SmokeObs;
        unsafe {
            core::ptr::addr_of_mut!((*p).echo).write_volatile(echo);
            core::ptr::addr_of_mut!((*p).verdict).write_volatile(verdict);
            core::ptr::addr_of_mut!((*p).magic).write_volatile(SMOKE_MAGIC); // LAST: run complete
        }

        // Busy-spin, NOT wfi (see the module doc): nop keeps the debug port live so the bench can
        // re-attach. No bkpt: the emulator hooks the magic write to stop, the bench polls magic.
        loop {
            cortex_m::asm::nop();
        }
    }
}

// --- host stub ---------------------------------------------------------------------------------
//
// On a host target this crate is built only so the workspace `cargo test`/`cargo build` for the
// host compiles every member. The real subject is bare-metal-only; here it is an empty `main` and
// pulls in no bare-metal dep.
#[cfg(not(target_os = "none"))]
fn main() {}
