// SPDX-License-Identifier: GPL-3.0-or-later
//! The I2C master: bring-up, polled transfers and the `embedded-hal` 1.0 `i2c::I2c` trait.
//!
//! Two register models (`hal_i2c`), one per build (specs/hal.md, section 3.1):
//!
//! - `gd` (GD32F10x and GD32F1x0, `i2c/gd.rs`): the event-based block (`gd32f10x_i2c.h` and
//!   `gd32f1x0_i2c.h` agree on every offset and bit used here).
//! - `mm32` (MM32SPIN05, `i2c/mm32.rs`): a DesignWare-style block with command and data FIFOs.
//!
//! Both models carry an [`I2c`] with the same surface; this file holds what they share: the bus
//! clear, the time budget and the bus modes. The instance is a base from the descriptor; the bus
//! clock is APB1 from the [`ClockConfig`](crate::clock::ClockConfig).
//!
//! # The time budget
//!
//! Every poll draws from one countdown per bus acquisition (a fresh START to its STOP, including a
//! repeated START), sized from the core clock so its bound is in microseconds:
//! [`worst_case_block_us`]. The firmware asserts in a `const` that twice that bound fits inside the
//! motor's demand-stale coast, because the IMU read is the control task's one blocking call and the
//! commutation ISR floats the phases when the control task goes quiet past the coast.

#[cfg(hal_i2c = "gd")]
#[path = "i2c/gd.rs"]
mod model;
#[cfg(hal_i2c = "mm32")]
#[path = "i2c/mm32.rs"]
mod model;

pub use model::I2c;

use crate::gpio::{self, AfMode};

/// SCL pulses a bus clear issues at most: a stuck byte's eight data clocks and its ACK clock (NXP
/// UM10204, section 3.1.16, "Bus clear").
const BUS_CLEAR_MAX_PULSES: u32 = 9;

/// Core cycles one poll iteration costs, as an upper bound.
///
/// The divisor that turns the microsecond budget into an iteration count, so the upper end of the
/// range is the safe one: an overstated cost understates the count and the real block ends early.
/// `runtime-hal` disassembled the loop in the release image at 21 core cycles (an APB1 load of
/// `STAT0`, the flag tests, the budget load and store, the back-branch), plus an APB1 bridge stall
/// and flash wait states that the listing cannot count. With the coast assertion the firmware makes,
/// the real loop could cost up to about 1.8 times the disassembled figure before the guarantee
/// fails.
pub const POLL_CYCLES: u32 = 32;

/// Microseconds one bus acquisition may spend polling.
///
/// Above a healthy 14-byte IMU burst (about 0.4 ms at 400 kHz, 1.7 ms at 100 kHz) by 15 and 3.5
/// times; below the firmware's 16 ms coast by enough that twice [`worst_case_block_us`] fits. A
/// longer transfer on a slow bus is split into several acquisitions rather than raising this,
/// because raising it spends the motor's coast.
pub const TXN_BUDGET_US: u32 = 6_000;

/// Poll iterations in [`TXN_BUDGET_US`] at `core_hz`, at least 1.
pub const fn poll_budget(core_hz: u32) -> u32 {
    let n = (core_hz / 1_000_000) * TXN_BUDGET_US / POLL_CYCLES;
    if n == 0 {
        1
    } else {
        n
    }
}

/// A tenth of [`poll_budget`], at least 1: the cap on the bus-idle wait before a fresh START (a
/// healthy bus is idle within a byte time of the last STOP), and the budget of the recovery that
/// runs after a failure, which has to work when the acquisition's budget is spent.
pub const fn idle_budget(core_hz: u32) -> u32 {
    let n = poll_budget(core_hz) / 10;
    if n == 0 {
        1
    } else {
        n
    }
}

/// The most microseconds one failing bus acquisition can block its caller, at core clock `core_hz`.
///
/// Every poll in an acquisition, including the bus-idle wait and the recovery that wait may run,
/// draws from the one [`poll_budget`]; the recovery after a failed transfer has its own
/// [`idle_budget`]. So the bound is `(poll_budget + idle_budget) * POLL_CYCLES` core cycles, which at
/// 72 MHz is `(13_500 + 1_350) * 32 / 72` = 6_600 us. A sub-MHz clock reports `u32::MAX` rather than
/// divide by zero.
pub const fn worst_case_block_us(core_hz: u32) -> u32 {
    let polls = poll_budget(core_hz) + idle_budget(core_hz);
    match (polls * POLL_CYCLES).checked_div(core_hz / 1_000_000) {
        Some(us) => us,
        None => u32::MAX,
    }
}

/// The fast-mode duty cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastDuty {
    /// Tlow / Thigh = 2 (`DTCY` clear, the vendor's `I2C_DTCY_2`).
    Two,
    /// Tlow / Thigh = 16 / 9 (`DTCY` set, `I2C_DTCY_16_9`).
    SixteenNine,
}

/// The bus mode: the SCL frequency and, in fast mode, the duty cycle. As in the vendor library, the
/// frequency picks the mode: standard up to 100 kHz, fast above it, up to 400 kHz.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cMode {
    /// SCL frequency, Hz.
    pub speed_hz: u32,
    /// The fast-mode duty cycle; not consulted at 100 kHz and below.
    pub duty: FastDuty,
}

impl I2cMode {
    /// A standard-mode bus at `speed_hz` (at most 100 kHz).
    pub const fn standard(speed_hz: u32) -> Self {
        Self {
            speed_hz,
            duty: FastDuty::Two,
        }
    }

    /// A fast-mode bus at `speed_hz` (above 100 kHz, at most 400 kHz) with duty cycle `duty`.
    pub const fn fast(speed_hz: u32, duty: FastDuty) -> Self {
        Self { speed_hz, duty }
    }
}

/// Clock a stuck slave off the bus through pin closures, and return the SCL pulses issued.
///
/// `drive_scl(true)` and `drive_sda(true)` release the open-drain line; `false` pulls it low. With
/// both lines released, an SDA that reads high is a clean bus and nothing more happens. Otherwise
/// SCL is pulsed until the slave releases SDA, at most [`BUS_CLEAR_MAX_PULSES`] times, and a STOP
/// (SDA rising while SCL is high) returns the slave to idle.
fn bus_clear(
    mut read_sda: impl FnMut() -> bool,
    mut drive_scl: impl FnMut(bool),
    mut drive_sda: impl FnMut(bool),
    mut settle: impl FnMut(),
) -> u32 {
    drive_scl(true);
    drive_sda(true);
    settle();
    if read_sda() {
        return 0;
    }
    let mut pulses = 0;
    while pulses < BUS_CLEAR_MAX_PULSES && !read_sda() {
        drive_scl(false);
        settle();
        drive_scl(true);
        settle();
        pulses += 1;
    }
    drive_sda(false);
    settle();
    drive_sda(true);
    settle();
    pulses
}

/// The port base, pin number and alternate function of SCL and SDA.
#[derive(Debug, Clone, Copy)]
struct BusPins {
    scl_base: u32,
    scl: u8,
    scl_af: u8,
    sda_base: u32,
    sda: u8,
    sda_af: u8,
}

impl BusPins {
    /// Both pins as AF open-drain with the pull-up: the block owns the bus from here.
    fn claim_af(&self) {
        gpio::configure_af(
            self.scl_base,
            self.scl,
            AfMode::OpenDrainPullUp,
            self.scl_af,
        );
        gpio::configure_af(
            self.sda_base,
            self.sda,
            AfMode::OpenDrainPullUp,
            self.sda_af,
        );
    }
}

/// The bus clear on SCL and SDA as open-drain outputs, with a settle of `settle_cycles` (a 100 kHz
/// half period) between edges. A clean bus costs one SDA read. Returns whether SDA reads high
/// (released) at the end.
///
/// Both output latches go high BEFORE the pins become outputs: `configure_output_od` does not
/// preset the latch, and after a warm reset it reads 0, so a pin made an output with its latch
/// low drives the line low at once and the slave sees an SCL edge and a STOP-like SDA edge before
/// the clean-bus check (runtime-hal 71c39e3, "bus clear without the entry glitch"). With the
/// latches high the pins come up released and the first edge on the bus is the sequence's own.
fn clear_bus(pins: &BusPins, settle_cycles: u32) -> bool {
    let BusPins {
        scl_base,
        scl,
        sda_base,
        sda,
        ..
    } = *pins;
    let at = move |line: Line| match line {
        Line::Scl => (scl_base, scl),
        Line::Sda => (sda_base, sda),
    };
    clear_bus_with(
        |line, high| {
            let (base, pin) = at(line);
            gpio::set_pin(base, pin, high)
        },
        |line| {
            let (base, pin) = at(line);
            gpio::configure_output_od(base, pin, true)
        },
        || gpio::read_pin(sda_base, sda),
        || settle(settle_cycles),
    )
}

/// One of the two bus lines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Line {
    Scl,
    Sda,
}

/// [`clear_bus`] over closures, so the host tests see the order of the pin operations: `latch`
/// sets a line's output latch (`true` = released), `make_output` makes it an open-drain output.
fn clear_bus_with(
    mut latch: impl FnMut(Line, bool),
    mut make_output: impl FnMut(Line),
    mut read_sda: impl FnMut() -> bool,
    mut settle: impl FnMut(),
) -> bool {
    latch(Line::Scl, true);
    latch(Line::Sda, true);
    make_output(Line::Scl);
    make_output(Line::Sda);
    let latch = core::cell::RefCell::new(latch);
    bus_clear(
        &mut read_sda,
        |high| (latch.borrow_mut())(Line::Scl, high),
        |high| (latch.borrow_mut())(Line::Sda, high),
        &mut settle,
    );
    read_sda()
}

/// Busy-wait between two bus-clear edges (the host has no bus to time).
#[inline(always)]
fn settle(cycles: u32) {
    #[cfg(target_arch = "arm")]
    cortex_m::asm::delay(cycles);
    #[cfg(not(target_arch = "arm"))]
    let _ = cycles;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_block_bound_at_72mhz() {
        assert_eq!(poll_budget(72_000_000), 13_500);
        assert_eq!(idle_budget(72_000_000), 1_350);
        assert_eq!(worst_case_block_us(72_000_000), 6_600);
        // The firmware's assertion: twice the bound inside the 16 ms coast
        // (256 periods * 2 * ARR 2250 / 72 counts per us).
        assert!(worst_case_block_us(72_000_000) * 2 < 256 * 2 * 2250 / 72);
    }

    #[test]
    fn the_block_bound_is_the_budget_in_time_at_any_clock() {
        for mhz in [8u32, 24, 48, 72, 108] {
            let us = worst_case_block_us(mhz * 1_000_000);
            // The budget plus a tenth, give or take the integer division.
            let want = TXN_BUDGET_US + TXN_BUDGET_US / 10;
            assert!(us <= want && us + 2 >= want, "{mhz} MHz: {us} us");
        }
    }

    #[test]
    fn budgets_never_reach_zero() {
        assert_eq!(poll_budget(0), 1);
        assert_eq!(idle_budget(1_000_000), 18);
        assert_eq!(idle_budget(0), 1);
        assert_eq!(worst_case_block_us(999_999), u32::MAX);
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Op {
        Latch(Line, bool),
        Output(Line),
    }

    /// The pin operations `clear_bus_with` makes on a bus whose SDA reads `sda`, and its result.
    fn clear_ops(sda: bool) -> (Vec<Op>, bool) {
        let ops = core::cell::RefCell::new(Vec::new());
        let released = clear_bus_with(
            |l, h| ops.borrow_mut().push(Op::Latch(l, h)),
            |l| ops.borrow_mut().push(Op::Output(l)),
            || sda,
            || {},
        );
        (ops.into_inner(), released)
    }

    #[test]
    fn the_latches_go_high_before_the_pins_become_outputs() {
        // runtime-hal 71c39e3: a pin made an output with its latch at the warm-reset 0 drives the
        // line low at once, an SCL edge and a STOP-like SDA edge before the clean-bus check.
        let (ops, released) = clear_ops(true);
        assert!(released);
        assert_eq!(
            ops[..4],
            [
                Op::Latch(Line::Scl, true),
                Op::Latch(Line::Sda, true),
                Op::Output(Line::Scl),
                Op::Output(Line::Sda),
            ]
        );
        // A clean bus: after that only the sequence's own release of both lines, no low edge.
        assert!(ops[4..].iter().all(|op| matches!(op, Op::Latch(_, true))));
    }

    #[test]
    fn a_held_bus_reports_not_released_after_the_pulses() {
        let (ops, released) = clear_ops(false);
        assert!(!released);
        let scl_lows = ops
            .iter()
            .filter(|op| **op == Op::Latch(Line::Scl, false))
            .count();
        assert_eq!(scl_lows as u32, BUS_CLEAR_MAX_PULSES);
        // Never an output before both latches are high.
        let first_output = ops.iter().position(|op| matches!(op, Op::Output(_)));
        assert_eq!(first_output, Some(2));
    }

    /// A simulated bus: SDA held low by a slave until `release_after` SCL pulses (never if `None`).
    struct Bus {
        release_after: Option<u32>,
        scl_falls: u32,
        sda: Vec<bool>,
    }

    fn run(release_after: Option<u32>) -> (u32, Bus) {
        let bus = core::cell::RefCell::new(Bus {
            release_after,
            scl_falls: 0,
            sda: Vec::new(),
        });
        let n = bus_clear(
            || {
                let b = bus.borrow();
                b.release_after.is_some_and(|r| b.scl_falls >= r)
            },
            |high| {
                if !high {
                    bus.borrow_mut().scl_falls += 1;
                }
            },
            |high| bus.borrow_mut().sda.push(high),
            || {},
        );
        (n, bus.into_inner())
    }

    #[test]
    fn a_clean_bus_gets_no_pulses_and_no_stop() {
        let (n, bus) = run(Some(0));
        assert_eq!(n, 0);
        assert_eq!(bus.scl_falls, 0);
        assert_eq!(bus.sda, [true], "SDA released once, never pulled low");
    }

    #[test]
    fn a_stuck_bus_gets_nine_pulses_and_a_stop() {
        let (n, bus) = run(None);
        assert_eq!(n, BUS_CLEAR_MAX_PULSES);
        assert_eq!(bus.scl_falls, BUS_CLEAR_MAX_PULSES);
        assert_eq!(bus.sda, [true, false, true], "release, then the STOP edge");
    }

    #[test]
    fn pulsing_stops_when_the_slave_lets_go() {
        for release in 1..BUS_CLEAR_MAX_PULSES {
            let (n, bus) = run(Some(release));
            assert_eq!(n, release);
            assert_eq!(bus.scl_falls, release);
            assert_eq!(bus.sda, [true, false, true]);
        }
    }
}
