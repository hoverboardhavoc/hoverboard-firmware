// SPDX-License-Identifier: GPL-3.0-or-later
//! The comparison of a HAL trace against its vendor trace (specs/hal.md, section 8.1).
//!
//! Three modes, all width-strict (an access is its op, width, address and value, so a 32-bit write
//! never equals two 16-bit writes over the same bytes):
//!
//! - `register_writes`: the ordered list of writes; reads are not compared.
//! - `final_state`: the last write to each address (its width and value, less the table's
//!   self-clearing command bits); order and intermediate writes are not compared.
//! - `with_polling`: the ordered list of reads and writes, so a poll the HAL drops or adds fails.
//!
//! A vector narrows the comparison to the accesses inside its `assert_only` ranges, or outside its
//! `ignore` ranges, before either mode runs.

use std::collections::BTreeMap;

use crate::target::Target;
use crate::trace::{Access, Op};

/// A comparison mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Ordered writes.
    RegisterWrites,
    /// The final value per address.
    FinalState,
    /// Ordered reads and writes.
    WithPolling,
}

impl Mode {
    /// The mode named in a vector file.
    pub fn from_name(name: &str) -> Result<Mode, String> {
        match name {
            "register_writes" => Ok(Mode::RegisterWrites),
            "final_state" => Ok(Mode::FinalState),
            "with_polling" => Ok(Mode::WithPolling),
            other => Err(format!(
                "unknown mode {other:?} (register_writes, final_state or with_polling)"
            )),
        }
    }
}

/// Which accesses a comparison covers, as inclusive address ranges.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Scope {
    /// Every access.
    All,
    /// Only the accesses inside these ranges.
    AssertOnly(Vec<(u32, u32)>),
    /// Every access outside these ranges.
    Ignore(Vec<(u32, u32)>),
}

impl Scope {
    /// Whether `a` is compared.
    pub fn covers(&self, a: &Access) -> bool {
        let inside =
            |ranges: &[(u32, u32)]| ranges.iter().any(|&(lo, hi)| a.addr >= lo && a.addr <= hi);
        match self {
            Scope::All => true,
            Scope::AssertOnly(r) => inside(r),
            Scope::Ignore(r) => !inside(r),
        }
    }

    /// The accesses this scope compares, in order.
    pub fn narrow(&self, accesses: &[Access]) -> Vec<Access> {
        accesses
            .iter()
            .copied()
            .filter(|a| self.covers(a))
            .collect()
    }
}

/// The result of one comparison.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Outcome {
    /// Whether the traces agree.
    pub matched: bool,
    /// The differences, one per line, empty on a match.
    pub diff: Vec<String>,
}

/// Compare `hal` against `vendor` (both already narrowed) in `mode`.
pub fn compare(mode: Mode, hal: &[Access], vendor: &[Access], target: &Target) -> Outcome {
    match mode {
        Mode::RegisterWrites => {
            let w = |t: &[Access]| -> Vec<Access> {
                t.iter().copied().filter(|a| a.op == Op::Write).collect()
            };
            ordered(&w(hal), &w(vendor), target)
        }
        Mode::WithPolling => ordered(hal, vendor, target),
        Mode::FinalState => final_state(hal, vendor, target),
    }
}

fn final_state(hal: &[Access], vendor: &[Access], target: &Target) -> Outcome {
    let last = |t: &[Access]| -> BTreeMap<u32, Access> {
        t.iter()
            .filter(|a| a.op == Op::Write)
            .map(|a| {
                let state = Access {
                    value: a.value & !target.self_clearing(a.addr),
                    ..*a
                };
                (a.addr, state)
            })
            .collect()
    };
    let (h, v) = (last(hal), last(vendor));
    let mut diff = Vec::new();
    let addrs: std::collections::BTreeSet<u32> = h.keys().chain(v.keys()).copied().collect();
    for addr in addrs {
        match (v.get(&addr), h.get(&addr)) {
            (Some(va), Some(ha)) if va == ha => {}
            (Some(va), Some(ha)) => diff.push(format!(
                "final {}: vendor {} / hal {}",
                target.symbolise(addr),
                va.render(target),
                ha.render(target)
            )),
            (Some(va), None) => diff.push(format!(
                "final {}: vendor {} / hal never writes it",
                target.symbolise(addr),
                va.render(target)
            )),
            (None, Some(ha)) => diff.push(format!(
                "final {}: vendor never writes it / hal {}",
                target.symbolise(addr),
                ha.render(target)
            )),
            (None, None) => {}
        }
    }
    Outcome {
        matched: diff.is_empty(),
        diff,
    }
}

/// An ordered comparison, reported as a line diff over the longest common subsequence: `-` is an
/// access only the vendor makes, `+` one only the HAL makes, and unchanged accesses are shown as
/// context.
fn ordered(hal: &[Access], vendor: &[Access], target: &Target) -> Outcome {
    if hal == vendor {
        return Outcome {
            matched: true,
            diff: Vec::new(),
        };
    }
    let (n, m) = (vendor.len(), hal.len());
    // lcs[i][j]: the common-subsequence length of vendor[i..] and hal[j..].
    let mut lcs = vec![vec![0u32; m + 1]; n + 1];
    for i in (0..n).rev() {
        for j in (0..m).rev() {
            lcs[i][j] = if vendor[i] == hal[j] {
                lcs[i + 1][j + 1] + 1
            } else {
                lcs[i + 1][j].max(lcs[i][j + 1])
            };
        }
    }
    let mut diff = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < n || j < m {
        if i < n && j < m && vendor[i] == hal[j] {
            diff.push(format!("  {}", vendor[i].render(target)));
            i += 1;
            j += 1;
        } else if j < m && (i == n || lcs[i][j + 1] >= lcs[i + 1][j]) {
            diff.push(format!("+ {}   (hal only)", hal[j].render(target)));
            j += 1;
        } else {
            diff.push(format!("- {}   (vendor only)", vendor[i].render(target)));
            i += 1;
        }
    }
    Outcome {
        matched: false,
        diff,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::target::Family;

    fn target() -> Target {
        Target::parse(
            Family::Gd32f10x,
            "[blocks.RCU_BASE]\nbase = 0x4002_1000\nsize = 0x400\n\
             [blocks.GPIOA_BASE]\nbase = 0x4001_0800\nsize = 0x400\n",
        )
        .unwrap()
    }

    fn w(size: u8, addr: u32, value: u32) -> Access {
        Access {
            op: Op::Write,
            size,
            addr,
            value,
        }
    }

    fn r(addr: u32, value: u32) -> Access {
        Access {
            op: Op::Read,
            size: 4,
            addr,
            value,
        }
    }

    /// A clock enable followed by a polled pin write: a read-modify-write, a poll, a write.
    fn base() -> Vec<Access> {
        vec![
            r(0x4002_1018, 0),
            w(4, 0x4002_1018, 0x4),
            r(0x4002_1000, 0x2),
            r(0x4001_0800, 0x4444_4444),
            w(4, 0x4001_0800, 0x4444_4B44),
        ]
    }

    #[test]
    fn identical_traces_match_in_every_mode() {
        let t = target();
        for mode in [Mode::RegisterWrites, Mode::FinalState, Mode::WithPolling] {
            assert!(compare(mode, &base(), &base(), &t).matched, "{mode:?}");
        }
    }

    #[test]
    fn a_32_bit_write_never_equals_two_16_bit_writes() {
        let t = target();
        let split = vec![
            r(0x4002_1018, 0),
            w(4, 0x4002_1018, 0x4),
            r(0x4002_1000, 0x2),
            r(0x4001_0800, 0x4444_4444),
            w(2, 0x4001_0800, 0x4B44),
            w(2, 0x4001_0802, 0x4444),
        ];
        for mode in [Mode::RegisterWrites, Mode::FinalState, Mode::WithPolling] {
            assert!(!compare(mode, &split, &base(), &t).matched, "{mode:?}");
        }
    }

    #[test]
    fn a_wrong_value_a_wrong_offset_and_a_missing_write_fail() {
        let t = target();
        let mut value = base();
        value[4].value = 0x4444_4F44;
        let mut offset = base();
        offset[4].addr = 0x4001_0804;
        let mut missing = base();
        missing.remove(1);
        for planted in [value, offset, missing] {
            for mode in [Mode::RegisterWrites, Mode::FinalState, Mode::WithPolling] {
                assert!(!compare(mode, &planted, &base(), &t).matched, "{mode:?}");
            }
        }
    }

    #[test]
    fn a_dropped_poll_fails_only_with_polling() {
        let t = target();
        let mut dropped = base();
        dropped.remove(2);
        assert!(!compare(Mode::WithPolling, &dropped, &base(), &t).matched);
        assert!(compare(Mode::RegisterWrites, &dropped, &base(), &t).matched);
        assert!(compare(Mode::FinalState, &dropped, &base(), &t).matched);
        let out = compare(Mode::WithPolling, &dropped, &base(), &t);
        assert!(out
            .diff
            .iter()
            .any(|l| l.starts_with("- R4 <RCU_BASE>+0x00")));
    }

    #[test]
    fn final_state_ignores_order_and_intermediate_writes() {
        let t = target();
        let restyled = vec![
            w(4, 0x4001_0800, 0x4444_4044),
            w(4, 0x4001_0800, 0x4444_4B44),
            w(4, 0x4002_1018, 0x4),
        ];
        assert!(compare(Mode::FinalState, &restyled, &base(), &t).matched);
        assert!(!compare(Mode::RegisterWrites, &restyled, &base(), &t).matched);
    }

    #[test]
    fn a_final_state_leaves_out_self_clearing_bits() {
        let t = Target::parse(
            Family::Gd32f10x,
            "[blocks.ADC0_BASE]\nbase = 0x4001_2400\nsize = 0x400\n\
             self_clearing = { \"0x08\" = 0x4 }\n",
        )
        .unwrap();
        let command_last = vec![w(4, 0x4001_2408, 0x9801), w(4, 0x4001_2408, 0x9805)];
        let state_last = vec![w(4, 0x4001_2408, 0x9805), w(4, 0x4001_2408, 0x9801)];
        assert!(compare(Mode::FinalState, &state_last, &command_last, &t).matched);
        let other = vec![w(4, 0x4001_2408, 0x9800)];
        assert!(!compare(Mode::FinalState, &other, &command_last, &t).matched);
    }

    #[test]
    fn scopes_narrow_by_address_and_cover_whole_words() {
        let t = target();
        let only_gpio = Scope::AssertOnly(vec![t.range("<GPIOA_BASE>+0x00").unwrap()]);
        assert_eq!(only_gpio.narrow(&base()).len(), 2);
        let no_rcu = Scope::Ignore(vec![t.range("<RCU_BASE>").unwrap()]);
        assert_eq!(no_rcu.narrow(&base()), only_gpio.narrow(&base()));
        // A half-word write at +0x02 is inside the register named +0x00.
        assert!(only_gpio.covers(&w(2, 0x4001_0802, 0)));
        assert!(!only_gpio.covers(&w(4, 0x4001_0804, 0)));
    }
}
