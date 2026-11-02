//! `hoverboard-cli`: the terminal tool for the hoverboard bridge (`specs/hoverboard-bridge.md`, "The terminal
//! tool"). The library holds what the tests exercise without a radio: the ELF checks, the firmware
//! bundle (`specs/firmware-bundle.md`), the framing,
//! the command client, the serial and TCP links, the walk and the configuration requests, a board's
//! store as a file and the name table's rules, `board erase`'s lines and guard, the L3 guest that
//! `machine watch` and `machine drive` use, `machine update`'s plan and order, the command line, the targets, the route of an SWD
//! command, and the machine commands' rules.
//! `main.rs` adds the command line, and `ble.rs` the BLE link (btleplug, the `ble` feature).

pub mod bundle;
pub mod cli;
pub mod client;
pub mod config;
pub mod elf;
pub mod erase;
pub mod fleet;
pub mod guest;
pub mod install;
pub mod machine;
pub mod names;
pub mod nodes;
pub mod route;
pub mod status;
pub mod target;
pub mod transport;
pub mod wire;
