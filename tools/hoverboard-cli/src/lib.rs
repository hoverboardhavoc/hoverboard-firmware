//! `hoverboard-cli`: the terminal tool for the hoverboard bridge (`specs/hoverboard-bridge.md`, "The terminal
//! tool"). The library holds what the tests exercise without a radio: the ELF checks, the framing,
//! the command client, the serial and TCP links, the walk and the configuration commands, and the
//! L3 guest that `watch` and `drive` use.
//! `main.rs` adds the BLE link (btleplug) and the command line.

pub mod client;
pub mod elf;
pub mod guest;
pub mod nodes;
pub mod transport;
pub mod wire;
