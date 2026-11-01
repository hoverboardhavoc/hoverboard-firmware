// SPDX-License-Identifier: GPL-3.0-or-later
//! The vendor families, their HAL chips, and each family's peripheral block table
//! (`targets/<family>.toml`): the names a trace prints addresses with, the register names for its
//! comments, and the reset values the emulator seeds.

use std::collections::BTreeMap;
use std::path::Path;

use serde::Deserialize;

/// A GigaDevice standard-peripheral-library family: one vendor oracle, one or more HAL chips.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Family {
    /// GD32F10x (the F103 mainboards).
    Gd32f10x,
    /// GD32F1x0 (the F130 boards).
    Gd32f1x0,
}

impl Family {
    /// Every family, in report order.
    pub const ALL: [Family; 2] = [Family::Gd32f10x, Family::Gd32f1x0];

    /// The family's name, as vector files and `targets/` spell it.
    pub fn name(self) -> &'static str {
        match self {
            Family::Gd32f10x => "gd32f10x",
            Family::Gd32f1x0 => "gd32f1x0",
        }
    }

    /// The family named `name`.
    pub fn from_name(name: &str) -> Result<Family, String> {
        Family::ALL
            .into_iter()
            .find(|f| f.name() == name)
            .ok_or_else(|| format!("unknown family {name:?} (gd32f10x or gd32f1x0)"))
    }

    /// The HAL chip features a vector of this family runs against. Both F103 parts share every
    /// GD32F10x register model (motorcontroller/hal/build.rs) and differ only in the descriptor, so
    /// a `_f10x` vector runs on both, which also checks each descriptor's bases.
    pub fn chips(self) -> &'static [&'static str] {
        match self {
            Family::Gd32f10x => &["gd32f103c8", "gd32f103rc"],
            Family::Gd32f1x0 => &["gd32f130c8"],
        }
    }
}

/// One peripheral block.
#[derive(Debug, Clone)]
pub struct Block {
    /// The symbol a trace prints, `RCU_BASE` in `<RCU_BASE>+0x18`.
    pub name: String,
    /// The block's base address.
    pub base: u32,
    /// The block's size in bytes.
    pub size: u32,
    /// Register names by offset, for trace comments.
    pub registers: BTreeMap<u32, String>,
    /// Reset values by offset, seeded into the emulator before a run.
    pub reset: BTreeMap<u32, u32>,
    /// Write-only bits by offset: a load returns them as 0, as the silicon's undefined read of them
    /// can, so a read-modify-write that would write back what it read shows in the trace.
    pub write_only: BTreeMap<u32, u32>,
}

/// A family's block table.
#[derive(Debug, Clone)]
pub struct Target {
    /// The family.
    pub family: Family,
    /// The blocks, sorted by base and non-overlapping.
    pub blocks: Vec<Block>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawTarget {
    blocks: BTreeMap<String, RawBlock>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawBlock {
    base: u32,
    size: u32,
    #[serde(default)]
    registers: BTreeMap<String, String>,
    #[serde(default)]
    reset: BTreeMap<String, u32>,
    #[serde(default)]
    write_only: BTreeMap<String, u32>,
}

/// Parse a number written `0x..` (hex) or in decimal, with optional `_` separators.
pub fn parse_u32(text: &str) -> Result<u32, String> {
    let t = text.trim().replace('_', "");
    let parsed = match t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16),
        None => t.parse::<u32>(),
    };
    parsed.map_err(|_| format!("not a number: {text:?}"))
}

impl Target {
    /// Load `targets/<family>.toml` from the harness directory.
    pub fn load(harness_dir: &Path, family: Family) -> Result<Target, String> {
        let path = harness_dir
            .join("targets")
            .join(format!("{}.toml", family.name()));
        let text =
            std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        Target::parse(family, &text).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// Parse a block table.
    pub fn parse(family: Family, text: &str) -> Result<Target, String> {
        let raw: RawTarget = toml::from_str(text).map_err(|e| e.to_string())?;
        let mut blocks = Vec::new();
        for (name, b) in raw.blocks {
            let offsets = |keys: &mut dyn Iterator<Item = &String>| -> Result<(), String> {
                for k in keys {
                    let off = parse_u32(k)?;
                    if off >= b.size {
                        return Err(format!("{name}: offset {k} is outside the block"));
                    }
                }
                Ok(())
            };
            offsets(&mut b.registers.keys())?;
            offsets(&mut b.reset.keys())?;
            offsets(&mut b.write_only.keys())?;
            let registers = b
                .registers
                .iter()
                .map(|(k, v)| Ok((parse_u32(k)?, v.clone())))
                .collect::<Result<_, String>>()?;
            let reset = b
                .reset
                .iter()
                .map(|(k, v)| Ok((parse_u32(k)?, *v)))
                .collect::<Result<_, String>>()?;
            let write_only = b
                .write_only
                .iter()
                .map(|(k, v)| Ok((parse_u32(k)?, *v)))
                .collect::<Result<_, String>>()?;
            blocks.push(Block {
                name,
                base: b.base,
                size: b.size,
                registers,
                reset,
                write_only,
            });
        }
        blocks.sort_by_key(|b| b.base);
        for pair in blocks.windows(2) {
            if u64::from(pair[0].base) + u64::from(pair[0].size) > u64::from(pair[1].base) {
                return Err(format!("{} overlaps {}", pair[0].name, pair[1].name));
            }
        }
        Ok(Target { family, blocks })
    }

    /// The block holding `addr`.
    pub fn block_of(&self, addr: u32) -> Option<&Block> {
        self.blocks
            .iter()
            .find(|b| addr >= b.base && u64::from(addr) < u64::from(b.base) + u64::from(b.size))
    }

    /// The block named `name`.
    pub fn block_named(&self, name: &str) -> Result<&Block, String> {
        self.blocks
            .iter()
            .find(|b| b.name == name)
            .ok_or_else(|| format!("no block {name} in the {} table", self.family.name()))
    }

    /// `addr` as `<BLOCK>+0xNN`, or as a bare address outside every block.
    pub fn symbolise(&self, addr: u32) -> String {
        match self.block_of(addr) {
            Some(b) => format!("<{}>+0x{:02X}", b.name, addr - b.base),
            None => format!("0x{addr:08X}"),
        }
    }

    /// The register name at `addr`, if the table names it.
    pub fn register_name(&self, addr: u32) -> Option<&str> {
        let b = self.block_of(addr)?;
        b.registers.get(&(addr - b.base)).map(String::as_str)
    }

    /// Split `<BLOCK>` or `<BLOCK>+rest` into the block and the text after the `+`.
    fn split<'t>(&self, expr: &'t str) -> Result<(&Block, Option<&'t str>), String> {
        let e = expr.trim();
        let inner = e
            .strip_prefix('<')
            .ok_or_else(|| format!("{expr:?}: expected <BLOCK> or <BLOCK>+0xNN"))?;
        let (name, rest) = inner
            .split_once('>')
            .ok_or_else(|| format!("{expr:?}: missing '>'"))?;
        let block = self.block_named(name)?;
        let rest = rest.trim();
        if rest.is_empty() {
            return Ok((block, None));
        }
        let off = rest
            .strip_prefix('+')
            .ok_or_else(|| format!("{expr:?}: expected '+' after the block"))?;
        Ok((block, Some(off)))
    }

    /// The address `<BLOCK>+0xNN` (or a bare number) names.
    pub fn resolve(&self, expr: &str) -> Result<u32, String> {
        if !expr.trim().starts_with('<') {
            return parse_u32(expr);
        }
        let (block, off) = self.split(expr)?;
        let off = match off {
            Some(o) => parse_u32(o)?,
            None => 0,
        };
        if off >= block.size {
            return Err(format!("{expr:?}: offset outside {}", block.name));
        }
        Ok(block.base + off)
    }

    /// The inclusive address range a filter entry covers. `<BLOCK>` is the whole block;
    /// `<BLOCK>+0xNN` the 32-bit register at that offset (all four of its bytes, so a narrower
    /// access inside it stays in scope); `<BLOCK>+0xLO..0xHI` the registers from `LO` through `HI`.
    pub fn range(&self, expr: &str) -> Result<(u32, u32), String> {
        let (block, off) = self.split(expr)?;
        let (lo, hi) = match off {
            None => (0, block.size - 1),
            Some(o) => match o.split_once("..") {
                Some((lo, hi)) => (parse_u32(lo)?, parse_u32(hi)? + 3),
                None => {
                    let at = parse_u32(o)?;
                    (at, at + 3)
                }
            },
        };
        if lo > hi || hi >= block.size {
            return Err(format!("{expr:?}: range outside {}", block.name));
        }
        Ok((block.base + lo, block.base + hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
[blocks.RCU_BASE]
base = 0x4002_1000
size = 0x400
registers = { "0x18" = "APB2EN" }
reset = { "0x04" = 0x10 }

[blocks.GPIOA_BASE]
base = 0x4001_0800
size = 0x400
"#;

    fn sample() -> Target {
        Target::parse(Family::Gd32f10x, SAMPLE).unwrap()
    }

    #[test]
    fn symbols_round_trip() {
        let t = sample();
        assert_eq!(t.symbolise(0x4002_1018), "<RCU_BASE>+0x18");
        assert_eq!(t.resolve("<RCU_BASE>+0x18").unwrap(), 0x4002_1018);
        assert_eq!(t.resolve("<GPIOA_BASE>").unwrap(), 0x4001_0800);
        assert_eq!(t.register_name(0x4002_1018), Some("APB2EN"));
        assert_eq!(t.symbolise(0x5000_0000), "0x50000000");
        assert_eq!(t.blocks[1].reset.get(&4), Some(&0x10));
    }

    #[test]
    fn a_register_range_covers_every_byte_of_its_words() {
        let t = sample();
        assert_eq!(
            t.range("<RCU_BASE>+0x18").unwrap(),
            (0x4002_1018, 0x4002_101B)
        );
        assert_eq!(
            t.range("<RCU_BASE>+0x00..0x0C").unwrap(),
            (0x4002_1000, 0x4002_100F)
        );
        assert_eq!(t.range("<RCU_BASE>").unwrap(), (0x4002_1000, 0x4002_13FF));
    }

    #[test]
    fn bad_expressions_are_refused() {
        let t = sample();
        assert!(t.resolve("<NOPE>+0x00").is_err());
        assert!(t.resolve("<RCU_BASE>+0x400").is_err());
        assert!(t.range("<RCU_BASE>+0x3FE").is_err());
        assert!(t.resolve("RCU_BASE+0x00").is_err());
    }

    #[test]
    fn overlapping_blocks_are_refused() {
        let text = "[blocks.A]\nbase = 0x100\nsize = 0x100\n[blocks.B]\nbase = 0x1FC\nsize = 0x4\n";
        assert!(Target::parse(Family::Gd32f1x0, text).is_err());
    }

    #[test]
    fn the_shipped_tables_load() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        for family in Family::ALL {
            let t = Target::load(dir, family).unwrap();
            assert!(t.block_named("RCU_BASE").is_ok(), "{family:?}");
        }
    }
}
