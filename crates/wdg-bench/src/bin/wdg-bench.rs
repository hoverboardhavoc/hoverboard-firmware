//! Watchdog reload-contract on-silicon validator (`specs/silicon-queue.md` section 5, "Watchdog
//! reload contract"; the R2 placement of `specs/sensing-and-safety.md`).
//!
//! One universal image (either bench family). It arms `runtime-hal`'s `FreeWatchdog` and runs the
//! R2-shaped main loop:
//!
//! - **(a) link-servicing stand-in**: a calibrated busy block (~300 us at 72 MHz), standing in for
//!   `service_links()`;
//! - **(b) scheduler dispatch**: the `scheduler` crate at its spec'd 250 Hz SysTick tick (LOAD
//!   from `scheduler::systick_load`, priority 0xF0, CTRL = 7; the SysTick ISR calls `tick()`, the
//!   main loop calls `dispatch()` interrupt-free), with one trivial registered task;
//! - **(c) watchdog feed AFTER dispatch**, never inside the link section (the R2 placement under
//!   test).
//!
//! Two SWD-pokable stall knobs live in `WDG_BENCH_OBS` (volatile-read every loop pass):
//!
//! - `stall_full = 1`: hang the whole loop (no link servicing, no dispatch, no feed);
//! - `stall_dispatch_only = 1`: keep servicing the link stand-in (loop + link counters keep
//!   growing) but skip dispatch + feed.
//!
//! Proof protocol (the bench runner drives it over SWD):
//!
//! 1. Healthy loop >= 60 s: `loop_count`/`dispatch_count`/`tick_count` grow, `was_fwdgt_reset` = 0.
//! 2. Poke `stall_full = 1`: the part watchdog-resets within the timeout; after reboot
//!    `was_fwdgt_reset` reads 1 (the knob itself is cleared by the startup .data init, so the
//!    rebooted loop runs healthy again).
//! 3. Poke `stall_dispatch_only = 1`: also resets (while `link_count` was still growing), proving
//!    the feed sits after dispatch, not inside link servicing.
//!
//! The reset cause is read at boot via `runtime-hal`'s `clock::was_fwdgt_reset` and then cleared
//! (`clock::clear_reset_flags`), so each boot's `was_fwdgt_reset` field reflects only the reset
//! that produced it. `FreeWatchdog::freeze_on_debug_halt()` is set at bring-up so a debugger that
//! halts the core mid-session is not reset out from under the probe (no effect when not attached).
//!
//! Read/poke over SWD: `nm` the `WDG_BENCH_OBS` symbol, field offsets documented on
//! [`firmware::Obs`]. Failure states busy-spin with a distinct `err` code (all before the watchdog
//! is armed, so an error halt does not reset-loop), NEVER `wfi` (GD32 SWD-lockout rule). No GPIO
//! pin is configured at all (nothing near PA9/PA10 or any FET gate).

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
mod firmware {
    use core::ptr::addr_of_mut;

    use cortex_m::asm::nop;
    use cortex_m::peripheral::scb::SystemHandler;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::{entry, exception};
    use panic_halt as _;

    use runtime_hal::clock::{self, ClockConfig};
    use runtime_hal::{detect_chip, FreeWatchdog, WdgTimeout};
    use scheduler::{systick_load, Scheduler};
    use test_shared::obs_store;

    // --- parameters -----------------------------------------------------------------------------

    /// The production 72 MHz tree (IRC8M -> PLL): the SysTick 250 Hz binding is validated at the
    /// shipping clock. The watchdog itself runs off the independent IRC40K/LSI regardless.
    const CLOCK: ClockConfig = ClockConfig::REFERENCE_72M_IRC8M;

    /// Watchdog timeout: 800 ms nominal (mid "~500 ms - 1 s"). `WdgTimeout::resolve` maps it to
    /// prescaler /8, reload 3999 on the 40 kHz LSI nominal (rounded up, never shorter; the real
    /// LSI is loosely trimmed, so the silicon period is approximate but bounded well under the
    /// 60 s healthy-run gate and well over one loop pass).
    const WDG_TIMEOUT_MS: u32 = 800;

    /// Link-servicing stand-in cost: ~300 us at 72 MHz (`cortex_m::asm::delay` counts cycles).
    const LINK_STANDIN_CYCLES: u32 = 300 * 72;

    /// The trivial registered task's reload: 1 = runs every 250 Hz tick, so `task_runs` tracks
    /// `tick_count` while dispatch is healthy and freezes the moment dispatch stalls.
    const TASK_RELOAD: u16 = 1;

    // --- the SWD-readable / pokable block --------------------------------------------------------

    /// The one SWD-readable observation + command block. `#[repr(C)]`, every field offset from the
    /// `WDG_BENCH_OBS` symbol base pinned by the compile-time asserts below. The two stall knobs
    /// are volatile-read every main-loop pass, so the bench runner pokes them over SWD while the
    /// loop runs; a watchdog reset clears the whole block (startup .data init), which is the
    /// protocol's re-arm.
    ///
    /// | offset | field | meaning |
    /// |---|---|---|
    /// | `0x00` | `magic` | `0x5744_4742` ("WDGB") once the block is live (set at loop entry, or together with a fatal `err`) |
    /// | `0x04` | `err` | 0 = ok; nonzero = fatal bring-up error, busy-spinning (codes on the field) |
    /// | `0x05` | `was_fwdgt_reset` | 1 = THIS boot was caused by a free-watchdog reset (read before the flags were cleared) |
    /// | `0x06` | `stall_full` | POKE 1 to hang the whole loop (expect a watchdog reset) |
    /// | `0x07` | `stall_dispatch_only` | POKE 1 to keep the link stand-in running but skip dispatch + feed (expect a watchdog reset) |
    /// | `0x08` | `tick_count` | 250 Hz SysTick ISR passes (grows even while the main loop is stalled) |
    /// | `0x0C` | `loop_count` | main-loop passes |
    /// | `0x10` | `dispatch_count` | dispatch + feed passes (freezes under `stall_dispatch_only`) |
    /// | `0x14` | `task_runs` | runs of the registered reload-1 task (via dispatch; tracks `tick_count` while healthy) |
    /// | `0x18` | `link_count` | link stand-in passes (keeps growing under `stall_dispatch_only`) |
    /// | `0x1C` | `timeout_ms` | the armed watchdog timeout in ms (constant, for the record) |
    ///
    /// Total size `0x20` (32) bytes.
    #[repr(C)]
    pub struct Obs {
        /// `0x5744_4742` ("WDGB").
        magic: u32,
        /// Fatal bring-up error, busy-spinning (watchdog NOT armed on any of these, so no reset
        /// loop): 0 ok, 1 detect_chip failed, 2 no RCU base, 3 clock tree failed, 4 scheduler
        /// register failed, 5 SysTick LOAD does not fit 24 bits, 6 watchdog start failed.
        err: u8,
        /// This boot was a free-watchdog reset.
        was_fwdgt_reset: u8,
        /// Stall knob: hang everything.
        stall_full: u8,
        /// Stall knob: link stand-in keeps running, dispatch + feed skipped.
        stall_dispatch_only: u8,
        /// SysTick ISR passes.
        tick_count: u32,
        /// Main-loop passes.
        loop_count: u32,
        /// Dispatch + feed passes.
        dispatch_count: u32,
        /// Registered-task runs.
        task_runs: u32,
        /// Link stand-in passes.
        link_count: u32,
        /// Armed watchdog timeout, ms.
        timeout_ms: u32,
    }

    // Pin every offset the bench runner depends on (the doc table above).
    const _: () = {
        assert!(core::mem::offset_of!(Obs, magic) == 0x00);
        assert!(core::mem::offset_of!(Obs, err) == 0x04);
        assert!(core::mem::offset_of!(Obs, was_fwdgt_reset) == 0x05);
        assert!(core::mem::offset_of!(Obs, stall_full) == 0x06);
        assert!(core::mem::offset_of!(Obs, stall_dispatch_only) == 0x07);
        assert!(core::mem::offset_of!(Obs, tick_count) == 0x08);
        assert!(core::mem::offset_of!(Obs, loop_count) == 0x0C);
        assert!(core::mem::offset_of!(Obs, dispatch_count) == 0x10);
        assert!(core::mem::offset_of!(Obs, task_runs) == 0x14);
        assert!(core::mem::offset_of!(Obs, link_count) == 0x18);
        assert!(core::mem::offset_of!(Obs, timeout_ms) == 0x1C);
        assert!(core::mem::size_of::<Obs>() == 0x20);
    };

    const MAGIC: u32 = 0x5744_4742; // "WDGB"

    /// The single SWD-readable/pokable instance (read/poke by `nm WDG_BENCH_OBS`).
    #[no_mangle]
    static mut WDG_BENCH_OBS: Obs = Obs {
        magic: 0,
        err: 0,
        was_fwdgt_reset: 0,
        stall_full: 0,
        stall_dispatch_only: 0,
        tick_count: 0,
        loop_count: 0,
        dispatch_count: 0,
        task_runs: 0,
        link_count: 0,
        timeout_ms: 0,
    };

    /// Volatile load of one FIELD of the obs block (the read half of `test_shared::obs_store!`,
    /// local to this bench: the knobs are poked externally over SWD, so every read must be a real
    /// volatile load, never cached or elided). Raw pointers end to end, no reference to the
    /// `static mut` is formed.
    macro_rules! obs_load {
        ($obs:path, $field:ident) => {{
            // SAFETY: obs-block discipline (this firmware + external SWD are the only accessors;
            // raw-pointer volatile read).
            unsafe {
                let p = core::ptr::addr_of!($obs);
                core::ptr::addr_of!((*p).$field).read_volatile()
            }
        }};
    }

    // --- the scheduler (ISR ticks it, main dispatches it) ----------------------------------------

    /// The 250 Hz scheduler. Touched from exactly two places: the SysTick ISR (`tick()`) and the
    /// main loop's interrupt-free `dispatch()`, so ISR and thread mutation never interleave.
    static mut SCHEDULER: Scheduler = Scheduler::new();

    /// The trivial registered task: bump `task_runs`. Runs via dispatch only, so it freezes as
    /// soon as dispatch stalls, while `tick_count` (ISR-side) keeps growing.
    fn tick_task() {
        let n = obs_load!(WDG_BENCH_OBS, task_runs).wrapping_add(1);
        obs_store!(WDG_BENCH_OBS, task_runs, n);
    }

    /// The 250 Hz SysTick ISR: advance the scheduler one tick and bump the ISR heartbeat.
    #[exception]
    fn SysTick() {
        // SAFETY: single core; the only other SCHEDULER access is the main loop's dispatch, which
        // runs inside cortex_m::interrupt::free, so this ISR never preempts it mid-mutation.
        unsafe { (*addr_of_mut!(SCHEDULER)).tick() };
        let n = obs_load!(WDG_BENCH_OBS, tick_count).wrapping_add(1);
        obs_store!(WDG_BENCH_OBS, tick_count, n);
    }

    // --- entry ----------------------------------------------------------------------------------

    #[entry]
    fn main() -> ! {
        let mut cp = cortex_m::Peripherals::take().unwrap();

        let chip = match detect_chip() {
            Ok(c) => c,
            Err(_) => fail(1),
        };

        // Reset cause FIRST (read before clearing), so the protocol's after-reboot check works:
        // a watchdog reset lands here with FWDGTRSTF set.
        let rcu = match chip.rcu_base() {
            Ok(b) => b,
            Err(_) => fail(2),
        };
        let wdg_reset = clock::was_fwdgt_reset(rcu);
        clock::clear_reset_flags(rcu);
        obs_store!(WDG_BENCH_OBS, was_fwdgt_reset, wdg_reset as u8);

        // The production 72 MHz tree (the SysTick LOAD below derives from it).
        if clock::configure_tree(&chip, &CLOCK).is_err() {
            fail(3);
        }

        // Scheduler: one trivial reload-1 task, registered before the SysTick interrupt exists
        // (no tick can interleave with the registration).
        // SAFETY: interrupts not yet enabled for SysTick; sole accessor here.
        if unsafe { (*addr_of_mut!(SCHEDULER)).register(tick_task, TASK_RELOAD) }.is_err() {
            fail(4);
        }

        // SysTick per the scheduler contract (sensing-and-safety "the SysTick edge"): LOAD from
        // scheduler::systick_load (24-bit checked), VAL = 0, priority 0xF0 (lowest), CTRL = 7
        // (ENABLE | TICKINT | CLKSOURCE = core clock).
        let load = match systick_load(CLOCK.sysclk_hz) {
            Some(l) => l,
            None => fail(5),
        };
        // SAFETY: priority write before the interrupt is enabled; 0xF0 = lowest, per the contract.
        unsafe { cp.SCB.set_priority(SystemHandler::SysTick, 0xF0) };
        cp.SYST.set_clock_source(SystClkSource::Core);
        cp.SYST.set_reload(load);
        cp.SYST.clear_current();
        cp.SYST.enable_interrupt();
        cp.SYST.enable_counter();

        // Watchdog LAST (every fail() above halts un-armed, so an error never reset-loops).
        // freeze_on_debug_halt keeps a halting debugger from being reset out from under the probe;
        // it has no effect while the core runs, so the stall protocol is untouched.
        FreeWatchdog::freeze_on_debug_halt();
        let mut wdg = match FreeWatchdog::start(&chip, WdgTimeout::from_millis(WDG_TIMEOUT_MS)) {
            Ok(w) => w,
            Err(_) => fail(6),
        };
        obs_store!(WDG_BENCH_OBS, timeout_ms, WDG_TIMEOUT_MS);
        obs_store!(WDG_BENCH_OBS, magic, MAGIC);

        // --- the R2 main loop: (a) link stand-in, (b) dispatch, (c) feed AFTER dispatch ---------
        loop {
            let lc = obs_load!(WDG_BENCH_OBS, loop_count).wrapping_add(1);
            obs_store!(WDG_BENCH_OBS, loop_count, lc);

            // Stall knob 1 (volatile-read every pass): hang EVERYTHING. The watchdog must reset
            // the part; the reboot clears the knob (startup .data init).
            if obs_load!(WDG_BENCH_OBS, stall_full) != 0 {
                loop {
                    nop();
                }
            }

            // (a) Link-servicing stand-in: a calibrated ~300 us busy block.
            cortex_m::asm::delay(LINK_STANDIN_CYCLES);
            let nk = obs_load!(WDG_BENCH_OBS, link_count).wrapping_add(1);
            obs_store!(WDG_BENCH_OBS, link_count, nk);

            // Stall knob 2 (volatile-read every pass): keep the link section alive, skip
            // dispatch + feed. The watchdog must still reset, proving the feed's R2 placement.
            if obs_load!(WDG_BENCH_OBS, stall_dispatch_only) != 0 {
                continue;
            }

            // (b) Scheduler dispatch, interrupt-free so the SysTick tick never interleaves with
            // a dispatch pass over the same table.
            cortex_m::interrupt::free(|_| {
                // SAFETY: the SysTick ISR is masked for the duration; sole accessor.
                unsafe { (*addr_of_mut!(SCHEDULER)).dispatch() };
            });

            // (c) The watchdog feed, AFTER dispatch (R2), never inside the link section.
            wdg.feed();
            let dc = obs_load!(WDG_BENCH_OBS, dispatch_count).wrapping_add(1);
            obs_store!(WDG_BENCH_OBS, dispatch_count, dc);
        }
    }

    /// Record a fatal bring-up error (with the magic, so the reader trusts the block) and
    /// busy-spin forever. Only reachable BEFORE the watchdog is armed. NEVER `wfi` (GD32
    /// SWD-lockout rule).
    fn fail(code: u8) -> ! {
        obs_store!(WDG_BENCH_OBS, err, code);
        obs_store!(WDG_BENCH_OBS, magic, MAGIC);
        loop {
            nop();
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
