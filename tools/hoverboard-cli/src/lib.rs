//! `hoverboard-cli`: the terminal tool for the hoverboard bridge (`specs/hoverboard-bridge.md`, "The terminal
//! tool"). The library holds what the tests exercise without a radio: the ELF checks, the framing,
//! the command client, the serial and TCP links, the walk and the configuration requests, a board's
//! store as a file and the name table's rules, `erase-store`'s lines and guard, and the L3 guest
//! that `watch` and `drive` use.
//! `main.rs` adds the command line, and `ble.rs` the BLE link (btleplug, the `ble` feature).

pub mod client;
pub mod config;
pub mod elf;
pub mod erase;
pub mod guest;
pub mod install;
pub mod names;
pub mod nodes;
pub mod status;
pub mod transport;
pub mod wire;
