//! The hardware-free half of the handheld remote (`specs/handheld-remote.md`, section 6.1).
//! Everything here builds for the host and is tested there; the binary crate (`handheld`) holds only
//! what touches the hardware or the radio.
#![no_std]

pub mod wire;
