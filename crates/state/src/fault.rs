//! Fault codes and the per-motor fault-latch unit (state.md §5).
//!
//! Fault detection is distributed; latching and response are centralized here. The over-current /
//! stall / consistency latch task runs once per 4 ms tick (250 Hz), one instance per motor. It does
//! two independent jobs each tick, in order: consume the FAST fault inputs (an external trip flag
//! and a one-shot fault-code byte), then run the SLOW persistent-inconsistency counter. The whole
//! evaluation is gated by the per-motor `running_enable` flag.
//!
//! The latch is one-way: this unit only ever writes `fault_latch = 1`, never clears it. Nothing
//! here clears `ext_trip` either. The sticky behavior belongs to this producer; the mode machine
//! (see `crate::mode`) keeps sampling the flag as a level every tick. The normal clearing path is a
//! power cycle / off->on, performed by a downstream co-writer, not by this unit.

/// Fast over-current fault code. Latches `fault_latch` immediately when seen in the code mailbox.
/// Part of the observable contract (telemetry, BLE status, beep pattern); preserved verbatim.
pub const CODE_OVERCURRENT: u8 = 0x11;

/// Fast stall code. Consumed (zeroed) when seen but does NOT latch directly; the stall path latches
/// via the slow count-to-limit counter below.
pub const CODE_STALL: u8 = 0x21;

/// Counter value at which the slow fault latches; the counter clamps here. 150000 ticks at 4 ms is
/// about 10 minutes. (`0x000249F0`.)
pub const LATCH_THRESHOLD: u32 = 150_000;

/// Increment guard: the counter is not incremented at or above this value, so it never wraps.
/// (`0x000F4240`.)
pub const COUNT_CAP: u32 = 1_000_000;

/// The `A` (balance sub-state) value meaning "cleanly running".
pub const RUN_SUBSTATE: i8 = 3;

/// The per-motor fault-latch unit. One instance per motor (one per advanced timer). All fields are
/// logical names; the board/runtime layer binds them to storage. Construct with [`FaultLatch::new`]
/// and call [`FaultLatch::tick`] once per 4 ms scheduler pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultLatch {
    /// Gate: the unit is inert when 0 (does not consume the code, does not reset or increment the
    /// counter, does not touch the latch). Tested `!= 0`. The link/role-enable byte.
    pub running_enable: u8,
    /// External trip input (e.g. hardware break / comparator latch). Level-sensitive: re-asserts
    /// the latch every tick while set. NOT cleared by this unit. Tested `!= 0`.
    pub ext_trip: u8,
    /// One-shot code mailbox written by the current path; this unit consumes (zeroes) it. The
    /// producer must rewrite it to signal again. Tested against the fixed codes.
    pub fault_code: u8,
    /// Balance sub-state, sign-extended byte. 0 = idle, 1 = soft-start, 2 = wind-down, 3 = run.
    pub a_substate: i8,
    /// Wheel motion/speed word, sign-extended halfword.
    pub b_motion: i16,
    /// Persistent-inconsistency tick counter. Clamps at `LATCH_THRESHOLD`, increment suppressed at
    /// `COUNT_CAP`.
    pub fault_counter: u32,
    /// Latched fault flag; this unit only ever writes 1, never clears it.
    pub fault_latch: u8,
}

impl FaultLatch {
    /// A fresh, disabled latch unit: gate off, no trip, no code, counter and latch clear. The board
    /// layer sets `running_enable` once the motor's role enables it.
    pub const fn new() -> Self {
        FaultLatch {
            running_enable: 0,
            ext_trip: 0,
            fault_code: 0,
            a_substate: 0,
            b_motion: 0,
            fault_counter: 0,
            fault_latch: 0,
        }
    }

    /// True once the latch has fired. The mode machine samples this (via Fault A/B) as a level.
    pub const fn is_latched(&self) -> bool {
        self.fault_latch != 0
    }

    /// The HEALTHY predicate (state.md §5.2 step 3), evaluated on the sign-extended `A` and `B`:
    ///
    /// ```text
    /// (A != 0 || B != 0) && (A == 3 || B == 0)
    /// ```
    ///
    /// Healthy means at least one of the two is nonzero AND either the sub-state is RUN (3) or the
    /// wheel is not moving. UNHEALTHY (counting) is the negation:
    /// `(A == 0 && B == 0) || (A != 3 && B != 0)`.
    pub const fn is_healthy(&self) -> bool {
        let a = self.a_substate;
        let b = self.b_motion;
        (a != 0 || b != 0) && (a == RUN_SUBSTATE || b == 0)
    }

    /// One 4 ms pass of the latch task. Exact order per state.md §5.2:
    ///
    /// - Step 0, gate: if `running_enable == 0`, do nothing and return (code not consumed, counter
    ///   frozen not reset, latch untouched).
    /// - Step 1, external trip: if `ext_trip != 0`, set `fault_latch = 1` (does not clear ext_trip).
    /// - Step 2, consume the fault code: examine then clear `fault_code`. `0x11` latches; `0x21` and
    ///   any other value (including 0) do not. `fault_code` is zeroed in all cases.
    /// - Step 3, consistency test: if HEALTHY, set `fault_counter = 0` and return (step 4 skipped).
    /// - Step 4, count and latch (UNHEALTHY only): increment the counter below `COUNT_CAP`, then if
    ///   it is at or above `LATCH_THRESHOLD`, clamp it to `LATCH_THRESHOLD` and set `fault_latch = 1`.
    pub fn tick(&mut self) {
        // Step 0: gate. Completely inert when disabled.
        if self.running_enable == 0 {
            return;
        }

        // Step 1: external trip. Level-sensitive, re-asserts every tick while set.
        if self.ext_trip != 0 {
            self.fault_latch = 1;
        }

        // Step 2: consume the one-shot code mailbox. Exactly one case applies; zeroed in all three.
        let code = self.fault_code;
        self.fault_code = 0;
        if code == CODE_OVERCURRENT {
            // Over-current: immediate latch.
            self.fault_latch = 1;
        }
        // CODE_STALL and any other value (including 0): consumed, no direct latch.

        // Step 3: consistency test. HEALTHY excuses the tick and zeroes the counter.
        if self.is_healthy() {
            self.fault_counter = 0;
            return;
        }

        // Step 4: count and latch (UNHEALTHY only).
        if self.fault_counter < COUNT_CAP {
            self.fault_counter += 1;
        }
        if self.fault_counter >= LATCH_THRESHOLD {
            self.fault_counter = LATCH_THRESHOLD;
            self.fault_latch = 1;
        }
    }
}

impl Default for FaultLatch {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enabled() -> FaultLatch {
        let mut f = FaultLatch::new();
        f.running_enable = 1;
        f
    }

    #[test]
    fn fixed_constants_preserved() {
        assert_eq!(CODE_OVERCURRENT, 0x11);
        assert_eq!(CODE_STALL, 0x21);
        assert_eq!(LATCH_THRESHOLD, 150_000);
        assert_eq!(LATCH_THRESHOLD, 0x0002_49F0);
        assert_eq!(COUNT_CAP, 1_000_000);
        assert_eq!(COUNT_CAP, 0x000F_4240);
        assert_eq!(RUN_SUBSTATE, 3);
    }

    #[test]
    fn gate_off_is_completely_inert() {
        let mut f = FaultLatch::new(); // running_enable == 0
        f.ext_trip = 1;
        f.fault_code = CODE_OVERCURRENT;
        f.fault_counter = 1234;
        f.a_substate = 0;
        f.b_motion = 0; // would be UNHEALTHY if enabled
        f.tick();
        // Code not consumed, counter frozen (not reset), latch untouched.
        assert_eq!(f.fault_code, CODE_OVERCURRENT);
        assert_eq!(f.fault_counter, 1234);
        assert_eq!(f.fault_latch, 0);
        assert_eq!(f.ext_trip, 1);
    }

    #[test]
    fn overcurrent_code_latches_immediately() {
        let mut f = enabled();
        f.fault_code = CODE_OVERCURRENT;
        // Healthy motion so the slow path cannot be the cause.
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 100;
        f.tick();
        assert_eq!(f.fault_latch, 1);
        assert_eq!(f.fault_code, 0, "code is consumed");
    }

    #[test]
    fn stall_code_is_consumed_but_does_not_latch_directly() {
        let mut f = enabled();
        f.fault_code = CODE_STALL;
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 100;
        f.tick();
        assert_eq!(f.fault_code, 0, "stall code is consumed");
        assert_eq!(f.fault_latch, 0, "stall code does not latch directly");
    }

    #[test]
    fn unknown_code_is_consumed_no_latch() {
        let mut f = enabled();
        f.fault_code = 0x55;
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 100;
        f.tick();
        assert_eq!(f.fault_code, 0);
        assert_eq!(f.fault_latch, 0);
    }

    #[test]
    fn ext_trip_latches_and_is_level_sensitive() {
        let mut f = enabled();
        f.ext_trip = 1;
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 1;
        f.tick();
        assert_eq!(f.fault_latch, 1);
        assert_eq!(f.ext_trip, 1, "ext_trip is not cleared by the unit");
        // Re-asserts every tick while set.
        f.fault_latch = 0; // pretend a co-writer cleared it
        f.tick();
        assert_eq!(f.fault_latch, 1, "re-asserted while ext_trip stays high");
    }

    #[test]
    fn healthy_predicate_table() {
        let mut f = enabled();
        // idle, stationary: A==0,B==0 -> UNHEALTHY (both zero).
        f.a_substate = 0;
        f.b_motion = 0;
        assert!(!f.is_healthy());
        // running cleanly: A==3 -> HEALTHY regardless of motion.
        f.a_substate = 3;
        f.b_motion = 500;
        assert!(f.is_healthy());
        // not run substate but moving: A==1,B!=0 -> UNHEALTHY (commanded-not-turning style).
        f.a_substate = 1;
        f.b_motion = 500;
        assert!(!f.is_healthy());
        // not run substate, not moving: A==1,B==0 -> HEALTHY.
        f.a_substate = 1;
        f.b_motion = 0;
        assert!(f.is_healthy());
        // idle but moving (coasting): A==0,B!=0 -> UNHEALTHY.
        f.a_substate = 0;
        f.b_motion = 500;
        assert!(!f.is_healthy());
    }

    #[test]
    fn b_min_halfword_counts_as_nonzero() {
        let mut f = enabled();
        // 0x8000 as i16 is the most-negative value: nonzero, so it counts in the != 0 test.
        f.a_substate = 0;
        f.b_motion = i16::MIN; // 0x8000
        assert!(!f.is_healthy(), "B = 0x8000 is nonzero so A==0,B!=0 is UNHEALTHY");
    }

    #[test]
    fn slow_counter_latches_at_exactly_threshold() {
        let mut f = enabled();
        // Make every tick UNHEALTHY: idle + stationary (A==0,B==0).
        f.a_substate = 0;
        f.b_motion = 0;
        for _ in 0..(LATCH_THRESHOLD - 1) {
            f.tick();
        }
        assert_eq!(f.fault_counter, LATCH_THRESHOLD - 1);
        assert_eq!(f.fault_latch, 0, "not yet latched at threshold - 1");
        f.tick();
        assert_eq!(f.fault_counter, LATCH_THRESHOLD);
        assert_eq!(f.fault_latch, 1, "latches at exactly the threshold tick");
    }

    #[test]
    fn preset_149999_latches_and_clamps_same_tick() {
        let mut f = enabled();
        f.a_substate = 0;
        f.b_motion = 0;
        f.fault_counter = 149_999;
        f.tick();
        assert_eq!(f.fault_counter, LATCH_THRESHOLD); // 150000, clamped
        assert_eq!(f.fault_latch, 1);
    }

    #[test]
    fn counter_clamps_at_threshold_in_steady_state() {
        let mut f = enabled();
        f.a_substate = 0;
        f.b_motion = 0;
        f.fault_counter = 250_000; // preset in [150000, 999999]
        f.tick();
        // Increments to 250001 then clamps back to 150000.
        assert_eq!(f.fault_counter, LATCH_THRESHOLD);
        assert_eq!(f.fault_latch, 1);
        // Stays clamped on further unhealthy ticks.
        f.tick();
        assert_eq!(f.fault_counter, LATCH_THRESHOLD);
    }

    #[test]
    fn count_cap_suppresses_increment_but_clamp_still_latches() {
        let mut f = enabled();
        f.a_substate = 0;
        f.b_motion = 0;
        f.fault_counter = COUNT_CAP; // >= cap: increment suppressed
        f.tick();
        // No increment (cap), but >= threshold so clamp forces 150000 and latches.
        assert_eq!(f.fault_counter, LATCH_THRESHOLD);
        assert_eq!(f.fault_latch, 1);
    }

    #[test]
    fn count_cap_prevents_wrap() {
        let mut f = enabled();
        f.a_substate = 0;
        f.b_motion = 0;
        f.fault_counter = COUNT_CAP - 1;
        f.tick(); // increments to exactly COUNT_CAP
        assert_eq!(f.fault_counter, LATCH_THRESHOLD); // then clamped back down
        // Even forcing the raw counter to the cap, it never increments past it.
        f.fault_counter = COUNT_CAP;
        f.tick();
        assert_eq!(f.fault_counter, LATCH_THRESHOLD);
    }

    #[test]
    fn healthy_tick_at_threshold_zeroes_counter_no_latch_write() {
        let mut f = enabled();
        // Counter already at/above threshold, but this tick is HEALTHY.
        f.fault_counter = 200_000;
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 500;
        f.fault_latch = 0;
        f.tick();
        assert_eq!(f.fault_counter, 0, "HEALTHY zeroes the counter");
        assert_eq!(f.fault_latch, 0, "no latch write on the HEALTHY path");
    }

    #[test]
    fn one_way_latch_does_not_auto_clear() {
        let mut f = enabled();
        f.fault_code = CODE_OVERCURRENT;
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 1;
        f.tick();
        assert_eq!(f.fault_latch, 1);
        // Subsequent perfectly healthy ticks must not clear the latch.
        f.a_substate = RUN_SUBSTATE;
        f.b_motion = 100;
        for _ in 0..10 {
            f.tick();
        }
        assert_eq!(f.fault_latch, 1, "latch is set-only; never auto-clears");
    }

    #[test]
    fn three_steps_can_each_latch_same_tick_idempotent() {
        let mut f = enabled();
        f.ext_trip = 1; // step 1
        f.fault_code = CODE_OVERCURRENT; // step 2
        f.a_substate = 0;
        f.b_motion = 0; // step 4 (with preset)
        f.fault_counter = 149_999;
        f.tick();
        assert_eq!(f.fault_latch, 1, "idempotent: all three paths set the same flag");
        assert_eq!(f.fault_counter, LATCH_THRESHOLD);
    }
}
