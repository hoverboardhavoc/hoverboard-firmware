//! Firmware-side bindings of the generic link layer to runtime-hal's USART (Phase 2 on-target glue).
//!
//! The `link` crate is generic over `embedded_io::{Read, Write, ReadReady}` so its transport logic
//! (the non-blocking RX drain in [`link::transport::LinkPort`], the BLE AT bring-up in
//! [`link::transport::BleBringup`]) is host-testable with a mock serial. This module instantiates
//! those generics with the REAL `runtime_hal::UsartSerial`, which proves the concrete serial
//! satisfies the `embedded_io` bounds, and provides the firmware-side glue functions that wire a
//! brought-up USART to the framer and route decoded frames into the node runtime.
//!
//! These are intended to run from the 250 Hz cooperative scheduler on target: the inter-board
//! cyclic task drives [`poll_link`] each tick (a bounded, non-blocking RX drain), and a slow task
//! steps [`ble_bringup_port`]'s AT machine. None of this is called from `main` yet, which stays the
//! blinky bring-up; the scheduler and the board-descriptor resolution wiring (mapping a logical link
//! label to a brought-up `UsartSerial`) are the next on-hardware step. This module exists now as a
//! compile / link check: the firmware crate building with `BoardLinkPort = LinkPort<UsartSerial>`
//! is the proof that the generic transport instantiates with the concrete runtime-hal serial.

#![allow(dead_code)]

use runtime_hal::descriptor::{ClockPath, ClockProfile};
use runtime_hal::usart::{Usart, UsartBus};
use runtime_hal::UsartSerial;

use link::transport::{BleBringup, LinkPort};

use node::{NodeRuntime, NodeState};

use heapless::Vec;

/// Maximum frames collected from one [`LinkPort::poll_rx`] pass before routing them into the node.
/// The drain is already bounded to `link::transport::MAX_RX_PER_PASS` bytes per pass, so the number
/// of complete frames in one pass is small; this caps the scratch buffer so it stays stack-sized.
const MAX_FRAMES_PER_PASS: usize = 8;

/// The concrete inter-board link port for this firmware: the generic [`LinkPort`] transport adapter
/// instantiated with the real `runtime_hal::UsartSerial`.
///
/// The crate compiling with this alias is the central thing Phase 2 verifies: it PROVES
/// `UsartSerial` implements `embedded_io::{Read, Write, ReadReady}`, the bounds `LinkPort` requires.
pub type BoardLinkPort = LinkPort<UsartSerial>;

/// The concrete BLE bring-up state machine for this firmware: [`BleBringup`] instantiated with the
/// real `runtime_hal::UsartSerial`. Proves the BLE adapter also satisfies the `embedded_io` bounds
/// with the concrete serial.
pub type BoardBleBringup = BleBringup<UsartSerial>;

/// Bring up a USART through runtime-hal and wrap it as an inter-board [`BoardLinkPort`].
///
/// `base` is the resolved USART data base (from the MCU descriptor's `AddrTable`); `path` selects
/// the register model (F10x / F1x0); `profile` carries the clock tree the baud divisor is derived
/// from; `bus` is the APB bus the USART sits on (so the input clock is read from the right APB
/// prescaler); `baud` is the target rate. This is the `Usart::bring_up` signature as runtime-hal
/// defines it (`bring_up(base, path, profile, bus, baud)`), wrapped with `UsartSerial::new`, then
/// handed to `LinkPort::new`.
///
/// On target the caller resolves `base` / `path` / `profile` / `bus` / `baud` from the board
/// descriptor for a logical link label; that descriptor resolution is the next wiring step and is
/// not done here.
pub fn bring_up_link(
    base: u32,
    path: ClockPath,
    profile: &ClockProfile,
    bus: UsartBus,
    baud: u32,
) -> BoardLinkPort {
    let usart = Usart::bring_up(base, path, profile, bus, baud);
    let serial = UsartSerial::new(usart);
    LinkPort::new(serial)
}

/// Bring up a USART through runtime-hal and wrap it as a [`BoardBleBringup`] AT bring-up machine
/// (default device name). Same descriptor-resolution note as [`bring_up_link`].
pub fn ble_bringup_port(
    base: u32,
    path: ClockPath,
    profile: &ClockProfile,
    bus: UsartBus,
    baud: u32,
) -> BoardBleBringup {
    let usart = Usart::bring_up(base, path, profile, bus, baud);
    let serial = UsartSerial::new(usart);
    BleBringup::new(serial)
}

/// Drain the inter-board link's RX once (non-blocking) and route every decoded, CRC-valid frame
/// into the node runtime on `link_index`.
///
/// `LinkPort::poll_rx` calls the sink closure once per decoded frame. Routing via
/// `node.route_inbound(...)` needs `&mut node` and `&mut state`, but the closure is also passed to
/// `poll_rx` while `port` is borrowed; rather than entangle those borrows, the closure only copies
/// each frame's bytes into a small `heapless::Vec`, and routing happens after `poll_rx` returns.
/// This sidesteps the closure-borrow conflict and keeps the drain bounded.
///
/// `DecodedFrame` borrows the framer's internal buffer (it carries a payload slice), so it cannot
/// outlive the `poll_rx` call; the closure re-encodes each frame into an owned `MAX_FRAME` buffer
/// (header + payload + CRC) and decodes it again after the pass to route it.
pub fn poll_link(
    port: &mut BoardLinkPort,
    link_index: usize,
    node: &mut NodeRuntime,
    state: &mut NodeState,
) {
    // Collect the raw bytes of each complete frame this pass. We cannot hold the borrowed
    // DecodedFrame past poll_rx, so snapshot each frame's wire bytes into an owned buffer.
    let mut frames: Vec<Vec<u8, { link::frame::MAX_FRAME }>, MAX_FRAMES_PER_PASS> = Vec::new();

    port.poll_rx(&mut |f| {
        // Re-encode the decoded frame back to its wire form so it can be decoded again after the
        // borrow ends. encode writes header + payload + CRC and returns the length.
        let mut buf: Vec<u8, { link::frame::MAX_FRAME }> = Vec::new();
        if buf.resize(link::frame::MAX_FRAME, 0).is_err() {
            return;
        }
        match link::frame::encode(&f.header, f.payload, &mut buf) {
            Ok(n) => {
                buf.truncate(n);
                // Drop silently if the per-pass frame cap is reached (bounded scratch).
                let _ = frames.push(buf);
            }
            Err(_) => {}
        }
    });

    // Route the collected frames now that the port borrow has ended.
    for buf in frames.iter() {
        if let Ok(frame) = link::frame::decode(buf) {
            node.route_inbound(link_index, &frame, state);
            node.note_frame(link_index, state);
        }
    }
}

/// Parse a board-definition blob and build the node runtime from it.
///
/// `config::parse` decodes the framed CBOR blob into a [`config::BoardConfig`]; on success
/// `NodeRuntime::from_config` builds the routing + supervision runtime. The companion
/// [`NodeState`] is built separately via `NodeRuntime::new_state`.
pub fn load_board_config(blob: &[u8]) -> Result<NodeRuntime, config::ConfigError> {
    let cfg = config::parse(blob)?;
    Ok(NodeRuntime::from_config(&cfg))
}
