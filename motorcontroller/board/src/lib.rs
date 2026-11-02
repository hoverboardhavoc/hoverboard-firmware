// SPDX-License-Identifier: GPL-3.0-or-later
//! The board's pin vocabulary and its boot status record (`specs/configured-links.md`).
//!
//! The firmware applies the stored configuration and checks none of it
//! (`specs/config-read-once.md`); the CLI and the app check it before it is written. What this
//! crate owns is what the firmware and the host tools share: the dead-time constants
//! ([`DEAD_TIME_MIN_DTG`], the CLI's floor) and the [`status::StatusRecord`] a peripheral that does
//! not come up adds an entry to.
//!
//! `no_std`; host tests in `#[cfg(test)]` link `std` via the host target.

#![no_std]

pub mod status;

/// The gate timer's clock, hertz. TIMER0 is on APB2 and the image's tree runs APB2 undivided, so
/// the timer clock is the sysclk; `motorcontroller/firmware/src/main.rs` asserts both the prescaler and
/// this number against the clock tree it configures, which is what makes the value here a fact
/// about the running image rather than a copy of one.
pub const GATE_TIMER_HZ: u32 = 72_000_000;

/// The dead-time generator's prescale off [`GATE_TIMER_HZ`] (the timer's CKDIV field), which is
/// what gives a stored `motor.dead_time` a UNIT: one DTG tick is `DEAD_TIME_DTS_DIV /
/// GATE_TIMER_HZ`, 27.8 ns here.
///
/// **The unit has one owner, which is this constant.** `motorcontroller/firmware/src/motor.rs` derives the
/// timer's CKDIV enum from this number rather than writing `ClockDiv::Div2` beside it, and
/// [`DEAD_TIME_MIN_DTG`] below converts the fleet's nanosecond reference through it. So a change
/// to the divider or to the clock moves the floor with it, and the CLI refuses to write a code that
/// is too short under the new unit. The field stays a DTG code because the unit is a property of
/// this image, identical on every board it boots, and not board data.
pub const DEAD_TIME_DTS_DIV: u32 = 2;

/// The shortest dead time any board in the fleet is measured running, nanoseconds: the 12-FET
/// dump's DTCFG 32 at its own 64 MHz undivided generator (`specs/commutation.md`, the dead-time
/// contracts). The fleet's other three contracts are longer: 694 ns (DTG 25, silicon-proven on
/// the 6-FET split board), the offroad stock dump's 778 ns (DTCFG 0x1C = 28), and EFeru's 750 ns
/// reference figure. This is the reference [`DEAD_TIME_MIN_DTG`] is derived from.
pub const DEAD_TIME_MIN_NS: u32 = 500;

/// The dead-time FLOOR a CONFIGURED gate group must meet, in `motor.dead_time` DTG ticks
/// (`specs/board-model.md`, section 2: the rule is a floor, not a nonzero test). The CLI and the
/// app check it before writing (`specs/config-read-once.md` section 4); the firmware programs the
/// stored dead time as it is.
///
/// [`DEAD_TIME_MIN_NS`] through one tick of the generator, which is 18 at this clock and this
/// divider. Derived rather than written as 18, because the number is only a dead TIME while the
/// unit behind it holds, and the unit is two constants away.
///
/// The BDTR `DTG` encoding counts single ticks over 0..=127; above that it switches to coarser
/// steps with an offset, every one of which is LONGER than 128 ticks, so a floor derived in the
/// linear range holds for the whole byte and the rule needs no ceiling. The assertion below keeps
/// the derivation inside that range.
///
/// 18 refuses the dangerous range while admitting every value the fleet is known to run (25, 28
/// and 32), and sits below EFeru's 750 ns (27 ticks here). Raising it to 25, the bench boards'
/// own value, is a judgement for the owner; lowering it needs a measurement rather than an
/// argument.
///
/// A dead time of 1 tick, 28 ns, is roughly 25x too short, which is shoot-through, both FETs of a
/// leg conducting through the transition.
///
/// Zero is NOT measured against this floor: it is the correct value for a board with no motor,
/// and it is legal with the gate group unset. The floor applies only where the gates are claimed.
pub const DEAD_TIME_MIN_DTG: u8 =
    (DEAD_TIME_MIN_NS * (GATE_TIMER_HZ / 1_000_000) / (1_000 * DEAD_TIME_DTS_DIV)) as u8;

// The floor stays in the DTG encoding's linear range, which is what lets the rule be a plain
// `<` on the byte with no ceiling: outside it the cast would also truncate.
const _: () =
    assert!(DEAD_TIME_MIN_NS * (GATE_TIMER_HZ / 1_000_000) / (1_000 * DEAD_TIME_DTS_DIV) <= 127);

#[cfg(test)]
mod tests;
