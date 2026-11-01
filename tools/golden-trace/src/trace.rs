// SPDX-License-Identifier: GPL-3.0-or-later
//! A register trace: the ordered peripheral accesses of one run, and its text form.
//!
//! One line per access: `W4 <RCU_BASE>+0x18 0x00000004   # APB2EN`. The op is `R` or `W`, the digit
//! the access width in bytes, then the symbolic address and the value at that width. `#` starts a
//! comment; a line that is only a comment is a header line.

use crate::target::{parse_u32, Target};

/// Read or write.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Op {
    /// A load.
    Read,
    /// A store.
    Write,
}

/// One peripheral access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Access {
    /// Load or store.
    pub op: Op,
    /// Width in bytes: 1, 2 or 4.
    pub size: u8,
    /// The address.
    pub addr: u32,
    /// The value loaded or stored, at the access width.
    pub value: u32,
}

impl Access {
    /// The line for this access, without its comment.
    pub fn render(&self, target: &Target) -> String {
        let op = match self.op {
            Op::Read => 'R',
            Op::Write => 'W',
        };
        format!(
            "{op}{} {} 0x{:0width$X}",
            self.size,
            target.symbolise(self.addr),
            self.value,
            width = usize::from(self.size) * 2
        )
    }
}

/// The trace body: one line per access, with the register name as a comment where the table has
/// one.
pub fn render(accesses: &[Access], target: &Target) -> String {
    let mut out = String::new();
    for a in accesses {
        let line = a.render(target);
        match target.register_name(a.addr) {
            Some(name) => out.push_str(&format!("{line:<34} # {name}\n")),
            None => {
                out.push_str(&line);
                out.push('\n');
            }
        }
    }
    out
}

/// Parse a trace file's accesses (header and comments skipped).
pub fn parse(text: &str, target: &Target) -> Result<Vec<Access>, String> {
    let mut out = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let at = |e: String| format!("line {}: {e}", n + 1);
        let mut parts = line.split_whitespace();
        let (Some(opw), Some(addr), Some(value), None) =
            (parts.next(), parts.next(), parts.next(), parts.next())
        else {
            return Err(at(format!(
                "expected `<op><width> <address> <value>`: {raw:?}"
            )));
        };
        let op = match opw.as_bytes().first() {
            Some(b'R') => Op::Read,
            Some(b'W') => Op::Write,
            _ => return Err(at(format!("bad op {opw:?}"))),
        };
        let size: u8 = opw[1..]
            .parse()
            .map_err(|_| at(format!("bad width {opw:?}")))?;
        if ![1, 2, 4].contains(&size) {
            return Err(at(format!("bad width {opw:?}")));
        }
        out.push(Access {
            op,
            size,
            addr: target.resolve(addr).map_err(at)?,
            value: parse_u32(value).map_err(at)?,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::target::Family;

    fn target() -> Target {
        Target::parse(
            Family::Gd32f1x0,
            "[blocks.RCU_BASE]\nbase = 0x4002_1000\nsize = 0x400\nregisters = { \"0x14\" = \"AHBEN\" }\n",
        )
        .unwrap()
    }

    #[test]
    fn render_and_parse_round_trip() {
        let t = target();
        let accesses = vec![
            Access {
                op: Op::Read,
                size: 4,
                addr: 0x4002_1014,
                value: 0x14,
            },
            Access {
                op: Op::Write,
                size: 2,
                addr: 0x4002_1016,
                value: 0xBEEF,
            },
            Access {
                op: Op::Write,
                size: 1,
                addr: 0x5000_0000,
                value: 0x7,
            },
        ];
        let text = format!("# a header line\n{}", render(&accesses, &t));
        assert!(text.contains("R4 <RCU_BASE>+0x14 0x00000014"));
        assert!(text.contains("# AHBEN"));
        assert!(text.contains("W2 <RCU_BASE>+0x16 0xBEEF"));
        assert!(text.contains("W1 0x50000000 0x07"));
        assert_eq!(parse(&text, &t).unwrap(), accesses);
    }

    #[test]
    fn malformed_lines_are_refused() {
        let t = target();
        assert!(parse("X4 <RCU_BASE>+0x00 0x0\n", &t).is_err());
        assert!(parse("W3 <RCU_BASE>+0x00 0x0\n", &t).is_err());
        assert!(parse("W4 <RCU_BASE>+0x00\n", &t).is_err());
        assert!(parse("W4 <NOPE>+0x00 0x0\n", &t).is_err());
    }
}
