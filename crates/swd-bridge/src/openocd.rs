//! The silicon [`MemAp`] backend: openocd's TCL RPC over TCP.
//!
//! openocd holds the SWD session and exposes a TCL RPC (default port 6666, commands terminated by
//! `0x1a`). Its `read_memory`/`write_memory` commands do **background AHB-AP** access - they read and
//! write target RAM **while the core runs**, no halt (the same path the bench's `LINK_OBS`/`mdw` reads
//! use). Start openocd on the probe host, e.g.:
//!
//! ```text
//! openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c 'bindto 0.0.0.0' -c init
//! ```
//!
//! and point [`OpenOcdTcl::connect`] at `host:6666`.

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;

use crate::{BridgeError, MemAp};

/// The TCL RPC command terminator.
const SEP: u8 = 0x1a;

/// A [`MemAp`] over an openocd TCL RPC connection.
pub struct OpenOcdTcl {
    writer: TcpStream,
    reader: BufReader<TcpStream>,
}

impl OpenOcdTcl {
    /// Connect to a running openocd's TCL RPC at `addr` (e.g. `"192.168.0.108:6666"`).
    pub fn connect(addr: &str) -> Result<Self, BridgeError> {
        let s = TcpStream::connect(addr)
            .map_err(|e| BridgeError::MemAp(format!("connect {addr}: {e}")))?;
        let reader = BufReader::new(s.try_clone().map_err(io)?);
        Ok(OpenOcdTcl { writer: s, reader })
    }

    /// Send one TCL command and return its captured output (without the `0x1a` terminator).
    fn rpc(&mut self, cmd: &str) -> Result<String, BridgeError> {
        self.writer.write_all(cmd.as_bytes()).map_err(io)?;
        self.writer.write_all(&[SEP]).map_err(io)?;
        self.writer.flush().map_err(io)?;
        let mut buf = Vec::new();
        let n = self.reader.read_until(SEP, &mut buf).map_err(io)?;
        if n == 0 {
            return Err(BridgeError::MemAp("openocd closed the connection".into()));
        }
        if buf.last() == Some(&SEP) {
            buf.pop();
        }
        Ok(String::from_utf8_lossy(&buf).trim().to_string())
    }

    /// `read_memory addr width count` -> a vec of values (each token may be `0x..` hex or decimal).
    fn read_memory(
        &mut self,
        addr: u32,
        width: u32,
        count: usize,
    ) -> Result<Vec<u32>, BridgeError> {
        let out = self.rpc(&format!("read_memory 0x{addr:08x} {width} {count}"))?;
        let mut vals = Vec::with_capacity(count);
        for tok in out.split_whitespace() {
            vals.push(parse_word(tok)?);
        }
        if vals.len() != count {
            return Err(BridgeError::MemAp(format!(
                "read_memory 0x{addr:08x} {width} {count}: expected {count} values, got {} ({out:?})",
                vals.len()
            )));
        }
        Ok(vals)
    }

    /// `write_memory addr width {decimal values...}`.
    fn write_memory(&mut self, addr: u32, width: u32, vals: &[u32]) -> Result<(), BridgeError> {
        let mut cmd = format!("write_memory 0x{addr:08x} {width} {{");
        for (i, v) in vals.iter().enumerate() {
            if i > 0 {
                cmd.push(' ');
            }
            cmd.push_str(&v.to_string());
        }
        cmd.push('}');
        let out = self.rpc(&cmd)?;
        if !out.is_empty() {
            // write_memory is normally silent; a non-empty reply is an error string.
            return Err(BridgeError::MemAp(format!("write_memory: {out}")));
        }
        Ok(())
    }
}

impl MemAp for OpenOcdTcl {
    fn read32(&mut self, addr: u32) -> Result<u32, BridgeError> {
        Ok(self.read_memory(addr, 32, 1)?[0])
    }
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), BridgeError> {
        self.write_memory(addr, 32, &[val])
    }
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), BridgeError> {
        if out.is_empty() {
            return Ok(());
        }
        let words = self.read_memory(addr, 8, out.len())?;
        for (b, w) in out.iter_mut().zip(words) {
            *b = w as u8;
        }
        Ok(())
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), BridgeError> {
        if data.is_empty() {
            return Ok(());
        }
        let vals: Vec<u32> = data.iter().map(|&b| b as u32).collect();
        self.write_memory(addr, 8, &vals)
    }
}

fn io(e: std::io::Error) -> BridgeError {
    BridgeError::MemAp(e.to_string())
}

/// Parse a TCL value token: `0x..` hex or plain decimal.
fn parse_word(tok: &str) -> Result<u32, BridgeError> {
    let parsed = if let Some(hex) = tok.strip_prefix("0x").or_else(|| tok.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16)
    } else {
        tok.parse::<u32>()
    };
    parsed.map_err(|_| BridgeError::MemAp(format!("unparseable openocd value {tok:?}")))
}
