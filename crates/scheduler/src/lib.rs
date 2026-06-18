//! 250 Hz cooperative outer-task scheduler.
//!
//! A fixed 20-slot task table whose tick handler advances per-slot down-counters and marks tasks due,
//! plus a main-loop dispatch pass that runs every due task in ascending slot order. This is the
//! **outer** loop only (control orchestrator, balance state machine, debouncers, BLE bring-up, rider
//! detection, ...); the motor inner FOC loop runs in the per-PWM-period ADC ISR, not here.
//!
//! Pure logic: no register access, no chip facts, no ownership of SysTick. The firmware's real SysTick
//! ISR calls [`Scheduler::tick`]; the main loop calls [`Scheduler::dispatch`]. The crate is reused
//! verbatim across every target (F130C8, F103C8, F103RCT6) and is host-testable.
//!
//! The reference constants and semantics below are preserved exactly. See `spec/core.md` and
//! `todo/scheduler.md`.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target.

#![no_std]

/// Number of task slots in the fixed table, indices `0..NSLOT`.
///
/// Also the out-of-range sentinel returned by [`Scheduler::sched_register`] when the table is full
/// (callers test success by `index < NSLOT`).
pub const NSLOT: usize = 20;

/// Fixed scheduler tick rate, Hz. Identical on every target. The SysTick reload value is derived from
/// the running system clock to hit this rate; the table and dispatch behavior do not vary by part.
pub const TICK_HZ: u32 = 250;

/// Tick period in milliseconds (4 ms at 250 Hz).
///
/// Reload-to-period mapping: a task with `reload = R` has consecutive due events exactly `R` ticks
/// apart, so its period is `R * TICK_MS` ms. Hence:
///
/// | reload `R` | period |
/// |---|---|
/// | 1 | 4 ms |
/// | **4** | **16 ms** |
/// | 15 | 60 ms |
/// | 62 | 248 ms |
/// | 250 | 1000 ms |
///
/// `reload = 0` is the **one-shot** marker (no reschedule; removed after it runs).
pub const TICK_MS: u32 = 1000 / TICK_HZ;

/// Status / error-code byte values (`sched_status`).
///
/// Init clears this to [`SchedError::None`]. It is written only on the two error conditions below and
/// is never read inside the unit; it exists for external diagnostics. The exact code values are
/// preserved from the original (full = 1, unregister-empty = 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SchedError {
    /// `0`: cleared by init; no error.
    None = 0,
    /// `1`: registration failed because the table is full.
    Full = 1,
    /// `2`: unregister targeted an already-empty slot.
    UnregisterEmpty = 2,
}

/// A registered slot index (`0..NSLOT`). Returned by the typed [`Scheduler::register`] API.
pub type SlotId = usize;

/// A task callback: a bare function pointer taking no arguments and returning nothing, matching the
/// original's `callback()` call site. `None` is the free/empty marker (the original's `callback == 0`
/// free test); callers must never register a "zero" / absent callback.
pub type Callback = fn();

/// One task-table slot. Mirrors the original 12-byte record's four fields and their widths.
///
/// Free test everywhere is `callback.is_none()` (the original's `callback == 0`).
#[derive(Clone, Copy)]
struct Slot {
    /// The function to run. `None` means the slot is free/empty.
    callback: Option<Callback>,
    /// Unsigned 16-bit down-counter: ticks until the next due event.
    counter: u16,
    /// Unsigned 16-bit reload period in ticks. `reload == 0` marks a one-shot.
    reload: u16,
    /// Unsigned 8-bit count of pending (unconsumed) due events. `runcount > 0` means due.
    /// The tick handler increments it; dispatch decrements it. Not overflow-guarded.
    runcount: u8,
}

impl Slot {
    /// A fully-cleared (free) slot, as written by init and by clear/unregister.
    const EMPTY: Slot = Slot {
        callback: None,
        counter: 0,
        reload: 0,
        runcount: 0,
    };
}

/// The 250 Hz cooperative scheduler: the 20-slot task table plus the status byte.
///
/// Construct with [`Scheduler::new`] (already cleared, status `None`). On hardware, call
/// [`Scheduler::systick_init`] to program SysTick; on host or when SysTick is owned elsewhere, the
/// table is fully usable without it.
pub struct Scheduler {
    table: [Slot; NSLOT],
    status: SchedError,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler {
    /// A fresh scheduler: all 20 slots cleared, `sched_status = 0` (`None`).
    pub const fn new() -> Self {
        Scheduler {
            table: [Slot::EMPTY; NSLOT],
            status: SchedError::None,
        }
    }

    /// The diagnostic status byte (`sched_status`). Set by init to `None`, then written only on the
    /// full and unregister-empty error conditions. Never read inside the unit; exposed for tests and
    /// external diagnostics.
    pub fn status(&self) -> SchedError {
        self.status
    }

    /// Clear the table and the status byte, matching `systick_init` steps 1 and 2 (the host-relevant,
    /// MCU-agnostic part). SysTick programming itself is target hardware and lives in the firmware's
    /// SysTick edge, not in this pure-logic crate; see [`systick_load`] for the reload computation and
    /// its 24-bit fatal-error check.
    ///
    /// Effects, in order:
    /// 1. Clear all 20 slots (`callback=0, counter=0, reload=0, runcount=0` each).
    /// 2. Set `sched_status = 0`.
    pub fn systick_init(&mut self) {
        // Step 1: clear all 20 slots. (The original transiently sets the unregister-empty error as a
        // side effect of clearing already-empty slots; step 2 below overwrites it, so we just clear.)
        self.table = [Slot::EMPTY; NSLOT];
        // Step 2: clear the status byte.
        self.status = SchedError::None;
    }

    /// Low-level registration, mirroring the original `sched_register(callback, initial_counter,
    /// reload) -> index` exactly.
    ///
    /// 1. Scan ascending for the lowest-index free slot (first `callback == 0`).
    /// 2. If found (`index < NSLOT`): write the fields, `runcount = 0`, return the index.
    /// 3. If full: set `sched_status = 1` (`Full`), return `NSLOT` (the out-of-range sentinel), modify
    ///    nothing. Callers test success by `index < NSLOT`.
    pub fn sched_register(&mut self, callback: Callback, initial_counter: u16, reload: u16) -> usize {
        for index in 0..NSLOT {
            if self.table[index].callback.is_none() {
                self.table[index] = Slot {
                    callback: Some(callback),
                    counter: initial_counter,
                    reload,
                    runcount: 0,
                };
                return index;
            }
        }
        // No free slot: the table is full.
        self.status = SchedError::Full;
        NSLOT
    }

    /// Typed registration wrapper. Registers a periodic task with `initial_counter = reload` (first
    /// due event on tick `reload + 1`, then every `reload` ticks).
    ///
    /// Returns the [`SlotId`] on success, or [`SchedError::Full`] when the table is full.
    pub fn register(&mut self, callback: Callback, reload: u16) -> Result<SlotId, SchedError> {
        let index = self.sched_register(callback, reload, reload);
        if index < NSLOT {
            Ok(index)
        } else {
            Err(SchedError::Full)
        }
    }

    /// Typed one-shot registration: runs once after `delay` ticks, then is removed by dispatch.
    ///
    /// Encoded as the original's one-shot marker `reload == 0` with `initial_counter = delay`, so the
    /// task fires on tick `delay + 1` and dispatch removes the slot after the single run.
    ///
    /// Returns the [`SlotId`] on success, or [`SchedError::Full`] when the table is full.
    pub fn register_oneshot(&mut self, callback: Callback, delay: u16) -> Result<SlotId, SchedError> {
        let index = self.sched_register(callback, delay, 0);
        if index < NSLOT {
            Ok(index)
        } else {
            Err(SchedError::Full)
        }
    }

    /// Low-level unregister, mirroring the original `sched_unregister(index) -> was_already_empty`.
    ///
    /// 1. Read the slot's callback; `was_already_empty = (callback == 0)`.
    /// 2. If `was_already_empty`: set `sched_status = 2` (`UnregisterEmpty`).
    /// 3. Unconditionally clear the slot (full clear).
    /// 4. Return `was_already_empty`. The slot is always left fully cleared.
    pub fn sched_unregister(&mut self, index: SlotId) -> bool {
        let was_already_empty = self.table[index].callback.is_none();
        if was_already_empty {
            self.status = SchedError::UnregisterEmpty;
        }
        // Unconditionally clear (the original clears even an already-empty slot).
        self.table[index] = Slot::EMPTY;
        was_already_empty
    }

    /// Typed unregister wrapper. Clears the slot and returns `Ok(())` if it held a task, or
    /// `Err(SchedError::UnregisterEmpty)` if the slot was already empty (the status byte is set to
    /// `UnregisterEmpty` in that case, and the slot is still cleared).
    pub fn unregister(&mut self, slot: SlotId) -> Result<(), SchedError> {
        if self.sched_unregister(slot) {
            Err(SchedError::UnregisterEmpty)
        } else {
            Ok(())
        }
    }

    /// The 250 Hz SysTick action: "advance one tick". This is what the firmware's real SysTick ISR
    /// calls; the crate does **not** own SysTick. Takes nothing, returns nothing.
    ///
    /// For each slot `0..NSLOT` ascending:
    /// 1. If empty (`callback == 0`): skip entirely, touch no field.
    /// 2. Else if `counter != 0`: decrement `counter` by 1 (not yet due).
    /// 3. Else (`counter == 0`, expired):
    ///    a. Increment `runcount` by 1 (now due; pending due events accumulate).
    ///    b. If `reload != 0`: reload `counter = reload - 1` (reschedule for the next period).
    ///    c. If `reload == 0` (one-shot): leave `counter = 0` (do not reschedule).
    ///
    /// `runcount` is u8 and not overflow-guarded; in normal operation dispatch drains it far more
    /// often than once per 256 ticks. A one-shot left at `counter == 0` has `runcount` incremented on
    /// every later tick until dispatch removes the slot; that accumulated runcount is discarded with
    /// the slot, so the callback still runs exactly once.
    pub fn tick(&mut self) {
        for index in 0..NSLOT {
            let slot = &mut self.table[index];
            // Step 1: empty slots are completely untouched.
            if slot.callback.is_none() {
                continue;
            }
            if slot.counter != 0 {
                // Step 2: not yet due.
                slot.counter -= 1;
            } else {
                // Step 3: expired -> mark due.
                slot.runcount = slot.runcount.wrapping_add(1);
                if slot.reload != 0 {
                    // Step 3b: reschedule for the next period.
                    slot.counter = slot.reload - 1;
                }
                // Step 3c: one-shot (reload == 0) leaves counter = 0.
            }
        }
    }

    /// The main-loop action: run all due tasks in ascending slot-index order. Called from the main
    /// loop; takes nothing.
    ///
    /// For each slot `0..NSLOT` ascending:
    /// 1. If `runcount != 0` (due):
    ///    a. Call `callback()` (no arguments).
    ///    b. Decrement `runcount` by 1, consuming exactly **one** pending due event, not all of them.
    ///    c. If `reload == 0` (one-shot): unregister this slot (full clear).
    /// 2. After all 20 slots, the post-dispatch hook (an empty no-op in the original) would run; it is
    ///    omitted here.
    ///
    /// Each pass runs each due task at most once, so a task that accumulated `runcount = k` takes `k`
    /// passes to drain (unless it is a one-shot, removed on the first pass). A callback that
    /// registers/unregisters mid-pass mutates the table and the loop continues over the changed table;
    /// a freshly registered task has `runcount = 0`, so it does not run until a later tick.
    pub fn dispatch(&mut self) {
        for index in 0..NSLOT {
            // Re-read each iteration: a callback may have mutated the table.
            if self.table[index].runcount != 0 {
                // Step 1a: run the callback. (Pull the fn pointer out before calling so the borrow on
                // `self.table` is released and the callback could re-enter scheduler ops if it held a
                // reference; here it is a bare fn() so no aliasing concern, but this keeps it clean.)
                if let Some(callback) = self.table[index].callback {
                    callback();
                }
                // Step 1b: consume exactly one pending due event.
                self.table[index].runcount -= 1;
                // Step 1c: one-shot -> remove after running.
                if self.table[index].reload == 0 {
                    self.sched_unregister(index);
                }
            }
        }
        // Step 2: post-dispatch hook (empty no-op) omitted.
    }
}

/// Compute the SysTick `LOAD` (reload) value for the fixed 250 Hz tick from the running system clock,
/// matching `systick_init` step 3: `LOAD = floor(sysclk / TICK_HZ) - 1`.
///
/// Returns `Err(())` if the result does not fit in SysTick's 24-bit reload (`>= 0x0100_0000`); the
/// original treats this as a fatal configuration error and hangs forever rather than silently
/// truncating. The caller (the firmware's SysTick edge) then programs `VAL = 0`, exception priority
/// `0xF0`, and `CTRL = 7` (ENABLE | TICKINT | CLKSOURCE). That register access is hardware and lives
/// outside this pure-logic crate; only this clock-derived computation is provided here.
pub fn systick_load(sysclk_hz: u32) -> Result<u32, ()> {
    let load = sysclk_hz / TICK_HZ - 1;
    // SysTick LOAD is a 24-bit field; values >= 2^24 do not fit.
    if load >= 0x0100_0000 {
        Err(())
    } else {
        Ok(load)
    }
}

// Host tests link std via the host target; a no_std crate must pull it in explicitly.
#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU32, Ordering};
    use std::sync::{Mutex, MutexGuard};
    use std::vec;
    use std::vec::Vec;

    // The observable counters below are process-global statics (the callback is a bare `fn()`, so it
    // can only reach statics). Tests run in parallel, so serialize every test that touches them behind
    // one lock to keep the shared counters race-free. Each test resets the counters under the guard.
    static GUARD: Mutex<()> = Mutex::new(());

    fn lock_counters() -> MutexGuard<'static, ()> {
        // Ignore poisoning: a panicking test still leaves the counters resettable by the next one.
        let g = GUARD.lock().unwrap_or_else(|e| e.into_inner());
        reset_counters();
        g
    }

    // Observable task callbacks: each increments a distinct static counter the test reads. Because the
    // crate's `Callback` is a bare `fn()`, the counters must be statics the fn can reach.
    static C0: AtomicU32 = AtomicU32::new(0);
    static C1: AtomicU32 = AtomicU32::new(0);
    static C2: AtomicU32 = AtomicU32::new(0);
    static C3: AtomicU32 = AtomicU32::new(0);

    fn task0() {
        C0.fetch_add(1, Ordering::Relaxed);
    }
    fn task1() {
        C1.fetch_add(1, Ordering::Relaxed);
    }
    fn task2() {
        C2.fetch_add(1, Ordering::Relaxed);
    }
    fn task3() {
        C3.fetch_add(1, Ordering::Relaxed);
    }

    fn reset_counters() {
        C0.store(0, Ordering::Relaxed);
        C1.store(0, Ordering::Relaxed);
        C2.store(0, Ordering::Relaxed);
        C3.store(0, Ordering::Relaxed);
    }

    // Drive N ticks then a single dispatch pass, the normal main-loop cadence.
    fn run_ticks_then_dispatch(sched: &mut Scheduler, n: usize) {
        for _ in 0..n {
            sched.tick();
            sched.dispatch();
        }
    }

    #[test]
    fn new_is_empty_and_clean() {
        let sched = Scheduler::new();
        assert_eq!(sched.status(), SchedError::None);
        // No slot is occupied: registering should land at index 0.
        let mut sched = sched;
        assert_eq!(sched.register(task0, 4).unwrap(), 0);
    }

    #[test]
    fn systick_init_clears_table_and_status() {
        let mut sched = Scheduler::new();
        // Fill a slot and force an error status.
        sched.register(task0, 4).unwrap();
        let _ = sched.unregister(5); // empty slot -> status UnregisterEmpty
        assert_eq!(sched.status(), SchedError::UnregisterEmpty);

        sched.systick_init();
        assert_eq!(sched.status(), SchedError::None);
        // Table cleared: first registration lands at 0 again.
        assert_eq!(sched.register(task0, 4).unwrap(), 0);
    }

    #[test]
    fn reload_constants_match_spec() {
        // 250 Hz, 4 ms tick, reload 4 == 16 ms.
        assert_eq!(TICK_HZ, 250);
        assert_eq!(TICK_MS, 4);
        assert_eq!(NSLOT, 20);
        // reload -> period mapping documented on TICK_MS.
        assert_eq!(4 * TICK_MS, 16); // reload 4 -> 16 ms
        assert_eq!(15 * TICK_MS, 60); // reload 15 -> 60 ms
        assert_eq!(62 * TICK_MS, 248); // reload 62 -> 248 ms
        assert_eq!(250 * TICK_MS, 1000); // reload 250 -> 1000 ms
    }

    #[test]
    fn reload4_task_fires_every_fourth_tick() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // register() sets initial_counter = reload = 4, so first due on tick 5, then every 4 ticks.
        sched.register(task0, 4).unwrap();

        // First 4 ticks: counter 4 -> 0, not yet due.
        for _ in 0..4 {
            sched.tick();
            sched.dispatch();
        }
        assert_eq!(C0.load(Ordering::Relaxed), 0);

        // 5th tick: due, fires once.
        sched.tick();
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 1);

        // Next 8 ticks: two more firings (every 4th).
        run_ticks_then_dispatch(&mut sched, 8);
        assert_eq!(C0.load(Ordering::Relaxed), 3);

        // Total after 5 + 8 = 13 ticks: firings at ticks 5, 9, 13 -> 3.
        // After 12 more ticks (25 total): ticks 17, 21, 25 -> 6.
        run_ticks_then_dispatch(&mut sched, 12);
        assert_eq!(C0.load(Ordering::Relaxed), 6);
    }

    #[test]
    fn reload1_task_fires_every_tick() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // initial_counter = reload = 1: first due on tick 2, then every tick.
        sched.register(task0, 1).unwrap();
        run_ticks_then_dispatch(&mut sched, 10);
        // Due on ticks 2..10 inclusive = 9 firings.
        assert_eq!(C0.load(Ordering::Relaxed), 9);
    }

    #[test]
    fn dispatch_runs_in_ascending_slot_order() {
        // Record the order callbacks run within a single dispatch pass.
        use std::sync::Mutex;
        static ORDER: Mutex<Vec<u8>> = Mutex::new(Vec::new());
        fn a() {
            ORDER.lock().unwrap().push(0);
        }
        fn b() {
            ORDER.lock().unwrap().push(1);
        }
        fn c() {
            ORDER.lock().unwrap().push(2);
        }

        ORDER.lock().unwrap().clear();
        let mut sched = Scheduler::new();
        // Register in a deliberately scrambled call order, but they take ascending slots 0,1,2.
        assert_eq!(sched.register(a, 1).unwrap(), 0);
        assert_eq!(sched.register(b, 1).unwrap(), 1);
        assert_eq!(sched.register(c, 1).unwrap(), 2);

        // One tick brings each counter 1 -> 0; second tick marks all due.
        sched.tick(); // counters 1 -> 0
        sched.tick(); // due, runcount = 1 each, counter reloaded to 0
        sched.dispatch();

        let order = ORDER.lock().unwrap().clone();
        assert_eq!(order, vec![0, 1, 2], "dispatch must run ascending slot order");
    }

    #[test]
    fn oneshot_runs_once_then_removed() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // One-shot with delay 2: due on tick 3, runs once, slot removed.
        let id = sched.register_oneshot(task0, 2).unwrap();
        assert_eq!(id, 0);

        // Ticks 1,2: counter 2 -> 0.
        run_ticks_then_dispatch(&mut sched, 2);
        assert_eq!(C0.load(Ordering::Relaxed), 0);

        // Tick 3: due, fires once, slot removed by dispatch.
        sched.tick();
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 1);

        // Many more ticks: never fires again (slot is empty).
        run_ticks_then_dispatch(&mut sched, 100);
        assert_eq!(C0.load(Ordering::Relaxed), 1);

        // The freed slot 0 is reused by the next registration.
        assert_eq!(sched.register(task1, 4).unwrap(), 0);
    }

    #[test]
    fn register_when_full_returns_full_error() {
        let mut sched = Scheduler::new();
        // Fill all 20 slots.
        for i in 0..NSLOT {
            assert_eq!(sched.register(task0, 4).unwrap(), i);
        }
        assert_eq!(sched.status(), SchedError::None);

        // 21st registration: full.
        let err = sched.register(task1, 4).unwrap_err();
        assert_eq!(err, SchedError::Full);
        assert_eq!(sched.status(), SchedError::Full);

        // The low-level form returns the NSLOT sentinel and modifies nothing.
        assert_eq!(sched.sched_register(task1, 4, 4), NSLOT);
    }

    #[test]
    fn unregister_empty_returns_error_and_sets_status() {
        let mut sched = Scheduler::new();
        // Unregister a never-registered slot.
        let err = sched.unregister(7).unwrap_err();
        assert_eq!(err, SchedError::UnregisterEmpty);
        assert_eq!(sched.status(), SchedError::UnregisterEmpty);

        // Low-level form returns was_already_empty = true.
        assert!(sched.sched_unregister(7));
    }

    #[test]
    fn unregister_occupied_succeeds() {
        let mut sched = Scheduler::new();
        let id = sched.register(task0, 4).unwrap();
        assert!(sched.unregister(id).is_ok());
        // Now empty: re-unregister errors.
        assert_eq!(sched.unregister(id).unwrap_err(), SchedError::UnregisterEmpty);
    }

    #[test]
    fn tick_decrements_marks_due_and_reloads() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // initial_counter = reload = 3 via sched_register for precise control.
        let id = sched.sched_register(task0, 3, 3);
        assert_eq!(id, 0);

        // Inspect internal state through behavior: 3 ticks bring counter to 0 with no due event.
        sched.tick(); // counter 3 -> 2
        sched.tick(); // counter 2 -> 1
        sched.tick(); // counter 1 -> 0
        // Not due yet (runcount still 0): dispatch runs nothing.
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 0);

        // 4th tick: counter == 0 -> increment runcount, reload counter = reload - 1 = 2.
        sched.tick();
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 1);

        // After reload, next due is 3 ticks later (counter went 2 -> 1 -> 0 -> due).
        run_ticks_then_dispatch(&mut sched, 3);
        assert_eq!(C0.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn dispatch_consumes_one_due_event_per_pass() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // reload 1, initial_counter 0: due on tick 1, then every tick.
        let id = sched.sched_register(task0, 0, 1);
        assert_eq!(id, 0);

        // Tick three times WITHOUT dispatching: runcount accumulates.
        sched.tick(); // counter 0 -> due, runcount 1, counter = reload-1 = 0
        sched.tick(); // due, runcount 2
        sched.tick(); // due, runcount 3

        // Each dispatch consumes exactly one pending due event.
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 1);
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 2);
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 3);
        // Drained: further dispatch does nothing until the next tick.
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn empty_slots_are_skipped_and_untouched() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // Register at slot 0 and slot 2, leaving slot 1 empty.
        assert_eq!(sched.sched_register(task0, 1, 1), 0);
        // Manually open a gap: register two then unregister the middle.
        assert_eq!(sched.sched_register(task1, 1, 1), 1);
        assert_eq!(sched.sched_register(task2, 1, 1), 2);
        let _ = sched.unregister(1);

        // Tick twice (due on tick 2), dispatch: only slots 0 and 2 fire, slot 1 stays empty.
        sched.tick();
        sched.tick();
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 1);
        assert_eq!(C1.load(Ordering::Relaxed), 0); // empty slot never ran
        assert_eq!(C2.load(Ordering::Relaxed), 1);

        // The freed slot 1 is the lowest free index, so the next register reclaims it.
        assert_eq!(sched.register(task3, 4).unwrap(), 1);
    }

    #[test]
    fn independent_initial_counter_and_reload() {
        let _g = lock_counters();
        let mut sched = Scheduler::new();
        // First delay 10, steady period 2: first due on tick 11, then every 2 ticks.
        let id = sched.sched_register(task0, 10, 2);
        assert_eq!(id, 0);

        run_ticks_then_dispatch(&mut sched, 10);
        assert_eq!(C0.load(Ordering::Relaxed), 0); // not due until tick 11

        sched.tick();
        sched.dispatch();
        assert_eq!(C0.load(Ordering::Relaxed), 1); // tick 11 due

        // Then every 2 ticks: ticks 13, 15, 17, 19, 21 ...
        run_ticks_then_dispatch(&mut sched, 10);
        assert_eq!(C0.load(Ordering::Relaxed), 6); // 1 + 5 more
    }

    #[test]
    fn systick_load_computation_and_24bit_check() {
        // Nominal: 72 MHz / 250 - 1 = 287999, fits in 24 bits.
        assert_eq!(systick_load(72_000_000).unwrap(), 72_000_000 / 250 - 1);
        // 8 MHz HSI: 8_000_000 / 250 - 1 = 31999.
        assert_eq!(systick_load(8_000_000).unwrap(), 31999);
        // A clock so high the reload overflows 24 bits is a fatal config error.
        // 0x0100_0000 ticks per 4 ms => sysclk = (2^24 + 1) * 250.
        let too_fast = (0x0100_0000u32 + 1) * TICK_HZ;
        assert!(systick_load(too_fast).is_err());
    }
}
