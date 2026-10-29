//! `hoverboard-cli`: the terminal tool for the hoverboard bridge (`specs/hoverboard-bridge.md`, "The terminal
//! tool"). The library holds what the tests exercise without a radio: the ELF checks, the framing,
//! the command client, the serial and TCP links, the walk and the configuration requests, a board's
//! store as a file and the name table's rules, and the L3 guest that `watch` and `drive` use.
//! `main.rs` adds the BLE link (btleplug) and the command line.

pub mod client;
pub mod config;
pub mod elf;
pub mod guest;
pub mod install;
pub mod names;
pub mod nodes;
pub mod transport;
pub mod wire;
