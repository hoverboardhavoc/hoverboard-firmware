//! The host **controller** that drives the L3 walk + `CONFIG_*` into the master over the mailbox.
//!
//! It wraps an attached [`HostMailbox`](crate::HostMailbox) as a [`BridgeSerial`](crate::BridgeSerial)
//! -> `link::SerialTransport` -> `link::Link` (the bridge end of the L2 link), and runs `net`'s
//! `Controller` over it: sequential request/response with retransmit, handling the master's
//! `NODE_HELLO` probes of the controller inline. After the walk, `CONFIG_WRITE`/`CONFIG_READ` round-trip
//! a field, and [`mount_store_image`] confirms the persisted `node_address` by mounting the store over
//! a flash image read back over SWD - an independent check that does not trust the wire.

use std::time::{Duration, Instant};

use link::{Link, SerialTransport};
use net::walk::PduBuf;
use net::{Controller, Opcode, Pdu};
use store::{Store, Value};
use swd_mailbox::FRAME_CAPACITY;

use crate::{BridgeError, BridgeSerial, HostMailbox, MemAp};

/// Reconcile the walk's two independent answers to "which board am I plugged into" into the one
/// address a bench command may default to (see [`WalkDriver::attached_addr`] for the incident that
/// motivates it).
///
/// `gateway` is the node that answered first contact out the attach port, which is the attached
/// board by construction. `host_link` is the fleet's own account of where the host sits, present
/// only when a `PORTS` reply carried it. They agree or the resolution is refused; a missing
/// gateway is refused too. Pure, so every branch is exercisable off-bench.
pub fn resolve_attached(
    gateway: Option<u8>,
    host_link: Option<net::walk::HostLink>,
) -> Result<u8, String> {
    let gateway = gateway
        .ok_or_else(|| "walk resolved no attached node (is the firmware running?)".to_string())?;
    if let Some(hl) = host_link {
        if hl.node != gateway {
            return Err(format!(
                "attached node is ambiguous: first contact answered from 0x{gateway:02x}, but the \
                 port table puts the host's link on 0x{:02x} (port {}, kind {}). Refusing to \
                 guess; pass --dst explicitly.",
                hl.node, hl.port, hl.kind
            ));
        }
    }
    Ok(gateway)
}

/// The one phrasing of the port-table corroboration, for every tool that resolves the attached node
/// (`WalkDriver::attached_addr`): a bench operator has to be able to see WHY an address resolved,
/// and whether the fleet itself confirmed it or the resolution rests on first contact alone.
pub fn host_link_note(link: Option<net::walk::HostLink>) -> String {
    match link {
        Some(hl) => {
            let medium = match hl.kind {
                net::walk::PORT_UART => "UART",
                net::walk::PORT_BLE => "BLE",
                net::walk::PORT_SWD => "SWD mailbox",
                _ => "unknown medium",
            };
            format!(
                " (port table: 0x{:02x} reports the host on its port {}, kind {} = {})",
                hl.node, hl.port, hl.kind, medium
            )
        }
        None => " (first contact; no port-table corroboration this walk)".to_string(),
    }
}

/// Decode and print one PDU on the wire (bench walk tracing): `dir op src->dst [payload]`.
fn trace_pdu(dir: &str, frame: &[u8]) {
    match Pdu::decode(frame) {
        Ok(p) => {
            let op = p
                .known()
                .map(|o| format!("{o:?}"))
                .unwrap_or_else(|| format!("op{:#04x}", p.opcode));
            eprintln!(
                "  {dir} {op} 0x{:02x}->0x{:02x} {:02x?}",
                p.src, p.dst, p.payload
            );
        }
        Err(_) => eprintln!("  {dir} <undecodable {} B> {frame:02x?}", frame.len()),
    }
}

/// A parsed `CONFIG_RESP` payload (`[field_id, index, status, type_tag, value...]`).
#[derive(Debug, Clone)]
pub struct CfgResp {
    /// The field id echoed back.
    pub field_id: u8,
    /// The index echoed back.
    pub index: u8,
    /// `0` = OK; see `net::walk::CFG_*`.
    pub status: u8,
    /// The value's storage type tag.
    pub type_tag: u8,
    /// The value bytes (decode with `store::Value::decode(Type::from_tag(type_tag), &value)`).
    pub value: Vec<u8>,
}

/// The host walk driver over the mailbox L2 link.
pub struct WalkDriver<M: MemAp> {
    link: Link<SerialTransport<BridgeSerial<M>>>,
    controller: Controller,
}

impl<M: MemAp> WalkDriver<M> {
    /// Wrap an **already-attached** [`HostMailbox`] (epoch bumped + ack received) as the bridge L2 link
    /// and a fresh controller.
    pub fn new(host: HostMailbox<M>) -> Self {
        let serial = BridgeSerial::new(host);
        let link = Link::new(SerialTransport::new(serial, FRAME_CAPACITY));
        WalkDriver {
            link,
            controller: Controller::new(),
        }
    }

    /// Reach the underlying MEM-AP (e.g. to read the store region back over SWD).
    pub fn mem(&mut self) -> &mut M {
        self.link.transport_mut().serial_mut().mailbox().mem()
    }

    /// **The attached node**: the address of the board this host's mailbox link lands on, which is
    /// the only board a bench command may be aimed at by default.
    ///
    /// Resolved from the walk, never from a remembered literal: the gateway (the node that answered
    /// first contact out the attach port) is the attached board by construction, and the port table
    /// corroborates it whenever a board reported the controller's own address as a neighbour. A
    /// port table that names a DIFFERENT node is a contradiction and is refused rather than guessed
    /// at.
    ///
    /// The 2026-07-31 arm session is the reason this exists: `specs/arm-session.md` hardcoded
    /// `--dst 0x01` from an era when the master held that address. The master had since persisted
    /// `0x02`, so `0x01` was allocated to the SLAVE, and the session's first power request armed the
    /// slave's bridge instead of the master's.
    pub fn attached_addr(&self) -> Result<u8, BridgeError> {
        resolve_attached(self.controller.gateway_addr(), self.controller.host_link())
            .map_err(BridgeError::MemAp)
    }

    /// The port-table evidence for the host's own link, when the walk saw it (a board's probe
    /// window can elapse before the host answers, in which case the port reports empty and this is
    /// `None`). Printed beside the resolved address so a bench operator sees WHY it resolved.
    pub fn host_link(&self) -> Option<net::walk::HostLink> {
        self.controller.host_link()
    }

    /// The controller's adopted guest address.
    pub fn guest_addr(&self) -> u8 {
        self.controller.guest_addr()
    }

    fn recv(&mut self) -> Option<Vec<u8>> {
        let mut buf = [0u8; 256];
        self.link.poll_recv(&mut buf).map(|f| f.to_vec())
    }

    fn send(&mut self, pdu: &[u8]) -> Result<(), BridgeError> {
        self.link
            .send(pdu)
            .map_err(|_| BridgeError::MemAp("L2 send: packet too large".into()))
    }

    /// Send one already-encoded L3 PDU over the mailbox L2 link, best-effort (no reply awaited). Used
    /// by the best-effort / latest-wins control families (`specs/link-control.md`): the INPUTS sender
    /// hands its encoded PDU here after the walk has brought the link up.
    pub fn send_pdu(&mut self, pdu: &[u8]) -> Result<(), BridgeError> {
        self.send(pdu)
    }

    /// Drive the walk to completion (`NODE_HELLO` -> `ASSIGN` -> `PROBE_PORTS` -> ...), retransmitting
    /// an unanswered request and replying to the master's probes of the controller.
    pub fn run_walk(&mut self, overall: Duration) -> Result<(), BridgeError> {
        let deadline = Instant::now() + overall;
        let retx = Duration::from_secs(3);
        let mut last_req: Option<PduBuf> = None;
        let mut req_at = Instant::now();
        while !self.controller.is_complete() {
            if Instant::now() > deadline {
                return Err(BridgeError::MemAp("walk timed out".into()));
            }
            if let Some(req) = self.controller.next_request() {
                trace_pdu("->", &req);
                self.send(&req)?;
                last_req = Some(req);
                req_at = Instant::now();
            }
            if let Some(frame) = self.recv() {
                trace_pdu("<-", &frame);
                if let Ok(p) = Pdu::decode(&frame) {
                    if p.known() == Some(Opcode::Ports) && !p.payload.is_empty() {
                        let n = p.payload[0] as usize;
                        eprint!("  PORTS from 0x{:02x}: {n} port(s)", p.src);
                        for i in 0..n {
                            let b = 1 + i * 4;
                            if b + 3 < p.payload.len() {
                                let state = match p.payload[b + 2] {
                                    0 => "empty",
                                    1 => "unassigned",
                                    2 => "assigned",
                                    _ => "?",
                                };
                                eprint!(
                                    " [port {} kind {} {} 0x{:02x}]",
                                    p.payload[b],
                                    p.payload[b + 1],
                                    state,
                                    p.payload[b + 3]
                                );
                            }
                        }
                        eprintln!();
                    }
                }
                if let Some(reply) = self.controller.reply_to_probe(&frame) {
                    self.send(&reply)?;
                } else {
                    self.controller.on_reply(&frame);
                }
            } else if Instant::now() - req_at > retx {
                if let Some(req) = last_req.clone() {
                    self.send(&req)?;
                    req_at = Instant::now();
                }
            }
        }
        Ok(())
    }

    /// `CONFIG_WRITE(dst, key, value)` -> the response.
    pub fn config_write(
        &mut self,
        dst: u8,
        key: store::Key,
        value: Value,
        timeout: Duration,
    ) -> Result<CfgResp, BridgeError> {
        let payload = crate::config::encode_config_write(key, &value);
        self.config_request(Opcode::ConfigWrite, dst, &payload, timeout)
    }

    /// `CONFIG_READ(dst, key)` -> the response.
    pub fn config_read(
        &mut self,
        dst: u8,
        key: store::Key,
        timeout: Duration,
    ) -> Result<CfgResp, BridgeError> {
        self.config_request(Opcode::ConfigRead, dst, &[key.field_id, key.index], timeout)
    }

    fn config_request(
        &mut self,
        op: Opcode,
        dst: u8,
        payload: &[u8],
        timeout: Duration,
    ) -> Result<CfgResp, BridgeError> {
        let src = self.controller.guest_addr();
        let pdu = Pdu::from_op(op, src, dst, payload);
        let mut buf = [0u8; 128];
        let n = pdu
            .encode(&mut buf)
            .map_err(|e| BridgeError::MemAp(format!("encode CONFIG: {e:?}")))?;

        let deadline = Instant::now() + timeout;
        let retx = Duration::from_secs(3);
        self.send(&buf[..n])?;
        let mut req_at = Instant::now();
        loop {
            if Instant::now() > deadline {
                return Err(BridgeError::MemAp("CONFIG timed out".into()));
            }
            if let Some(frame) = self.recv() {
                if let Ok(rp) = Pdu::decode(&frame) {
                    match rp.known() {
                        // The master probing the controller mid-exchange: answer it.
                        Some(Opcode::NodeHello) if rp.payload.len() == 1 => {
                            if let Some(reply) = self.controller.reply_to_probe(&frame) {
                                self.send(&reply)?;
                            }
                        }
                        Some(Opcode::ConfigResp) if rp.payload.len() >= 3 => {
                            return Ok(CfgResp {
                                field_id: rp.payload[0],
                                index: rp.payload[1],
                                status: rp.payload[2],
                                type_tag: *rp.payload.get(3).unwrap_or(&0),
                                value: rp.payload.get(4..).unwrap_or(&[]).to_vec(),
                            });
                        }
                        _ => {} // ignore anything else
                    }
                }
            } else if Instant::now() - req_at > retx {
                self.send(&buf[..n])?;
                req_at = Instant::now();
            }
        }
    }
}

/// A read-only `store::Flash` over a flash image read back over SWD - so the host can mount the
/// master's store and read a field back **independently of the wire** (`erase`/`program` are never
/// reached by `mount` on a clean log, nor by `get_value`).
pub struct ImageFlash {
    page_size: usize,
    bytes: Vec<u8>,
}

impl ImageFlash {
    /// An image of `bytes` with `page_size`-byte pages (the store region is two pages).
    pub fn new(page_size: usize, bytes: Vec<u8>) -> Self {
        ImageFlash { page_size, bytes }
    }
}

impl store::Flash for ImageFlash {
    fn page_size(&self) -> usize {
        self.page_size
    }
    fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }
    fn erase_page(&mut self, _page: usize) -> Result<(), base::error::FlashError> {
        Err(base::error::FlashError::Locked) // read-only image; never reached for a clean store
    }
    fn program(&mut self, _off: usize, _bytes: &[u8]) -> Result<(), base::error::FlashError> {
        Err(base::error::FlashError::Locked)
    }
}

/// Mount the store over a read-back image and read one field's `Value` (an independent confirmation of
/// a persisted field, e.g. `node_address`).
///
/// The image and the `Store` are leaked so the returned `Value` (which borrows the image for
/// `Str`/`Bytes`; a scalar like `node_address` is owned anyway) is genuinely `'static`. The CLI runs
/// once and exits, so the leak is harmless.
pub fn mount_store_image(
    image: ImageFlash,
    key: store::Key,
) -> Result<Value<'static>, BridgeError> {
    let image: &'static mut ImageFlash = Box::leak(Box::new(image));
    let store: &'static Store<'static, ImageFlash> = Box::leak(Box::new(
        Store::mount(image).map_err(|e| BridgeError::MemAp(format!("store mount: {e:?}")))?,
    ));
    store
        .get_value(key)
        .map_err(|e| BridgeError::MemAp(format!("store get_value: {e:?}")))
}
