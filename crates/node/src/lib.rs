//! Node / binding runtime (Phase 3) + link supervision and fault propagation (Phase 4).
//!
//! Turns a decoded [`config::BoardConfig`] into a runtime routing + supervision layer. Behavior is
//! read from the bindings (per-link `produce` / `consume` [`ItemSet`]s and per-motor
//! `attitude_source` / `drive_source` / `speed_sync_peer`), never from a role enum.
//!
//! # Routing ([`NodeRuntime::route_inbound`])
//!
//! Each decoded frame arrives tagged with the link index it came in on. The frame's data item is
//! checked against that link's `consume` set first: an item outside `consume` is ignored after the
//! CRC pass (no fault). An item in `consume` is routed by the binding model:
//!
//! - `CyclicState` carries attitude (`pitch` / `roll`), wheel speed, and a status byte, each a
//!   separately bound item. Attitude lands in the [`MotorSink::attitude_in`] (or the fusion buffer)
//!   of any motor whose `attitude_source` names this link; the wheel speed lands in
//!   [`NodeState::peer_wheel_speed`] when `speed_sync_peer` names this link; the status lands in
//!   [`NodeState::peer_status`].
//! - `DriveCmd` lands in the [`MotorSink::drive_setpoint`] of any motor whose `drive_source` names
//!   this link (a rigid follower; a direct motor command). A balancing motor
//!   (`drive_source = LocalBalance`) does not take a `DriveCmd` as a torque bypass.
//! - `Inputs` lands in [`NodeState::remote_setpoint`] (the speed/steer setpoint entry point a
//!   balancing node uses, the same entry the local rider lean uses).
//! - `Fault` feeds the fault subsystem (Phase 4): `action = stop-all` requests an immediate safe
//!   transition.
//!
//! # Supervision (Phase 4)
//!
//! Per link a receive-timeout counter is incremented by [`NodeRuntime::tick_no_frame`] (called by
//! the cyclic task each 4 ms when no good frame arrived) and cleared by [`NodeRuntime::note_frame`]
//! on any good frame. Crossing [`RX_TIMEOUT_TICKS`] latches a recoverable per-link comms-loss flag,
//! which drives the `running_enable` of any motor fed by that link to false; when frames resume the
//! flag lowers and `running_enable` recovers. A latched local fault yields a single broadcast
//! [`Fault`] per latch edge from [`NodeRuntime::take_outbound_fault`].

#![no_std]

use heapless::Vec;

use config::{AttitudeSource, BoardConfig, DriveSource, MAX_FUSED, MAX_LINKS, MAX_MOTORS};
use link::frame::{DecodedFrame, BROADCAST};
use link::item::{DataItem, ItemSet};
use link::opcode::Opcode;
use link::payload::{CyclicState, DriveCmd, Fault, Inputs, NodeHello};

/// `Fault.action` value requesting an immediate stop-all safe transition on all consuming nodes.
pub const FAULT_ACTION_STOP_ALL: u8 = 1;

/// Receive-timeout threshold in cyclic ticks (a small multiple of the 4 ms cyclic period). Crossing
/// this latches the per-link comms-loss flag.
pub const RX_TIMEOUT_TICKS: u16 = 5;

/// Protocol version this node speaks (mirrors `link::frame::PROTO_VER`).
const PROTO_VER: u8 = link::frame::PROTO_VER;

// --- inbound attitude ---------------------------------------------------------------------------

/// An attitude sample (the `pitch` / `roll` pair from a `CyclicState`), as written into a motor's
/// balance-loop input by the router.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CyclicAttitude {
    pub pitch: i16,
    pub roll: i16,
}

// --- per-motor sink -----------------------------------------------------------------------------

/// The per-motor control sinks the router writes into. The future control loop reads these; the
/// node runtime owns `running_enable` (Phase 4) and the inbound attitude / drive setpoint (Phase 3).
#[derive(Debug, Clone, Default)]
pub struct MotorSink {
    /// Attitude routed into this motor's balance loop, when `attitude_source` is a single `Link`.
    pub attitude_in: Option<CyclicAttitude>,
    /// Per-source fusion buffer, when `attitude_source` is `Fused`. Indexed positionally by the
    /// order the fused link indices appear in the motor's `attitude_source`. The fusion law itself
    /// is control-layer work; the router only deposits the per-source samples here.
    pub fusion_buf: Vec<Option<CyclicAttitude>, MAX_FUSED>,
    /// A direct drive command routed in when `drive_source` is a `Link` (a rigid follower).
    pub drive_setpoint: Option<DriveCmd>,
    /// Whether this motor is permitted to run. The supervisor drives this false when a link feeding
    /// the motor is lost, and true again when the link recovers.
    pub running_enable: bool,
}

impl MotorSink {
    fn new(fused_len: usize) -> MotorSink {
        let mut fusion_buf: Vec<Option<CyclicAttitude>, MAX_FUSED> = Vec::new();
        for _ in 0..fused_len {
            // Bounded by MAX_FUSED in the config; cannot overflow.
            let _ = fusion_buf.push(None);
        }
        MotorSink {
            attitude_in: None,
            fusion_buf,
            drive_setpoint: None,
            // A motor starts disabled until a link feeding it (if any) is healthy. A motor with no
            // link source (purely local) starts enabled, see NodeState::new.
            running_enable: false,
        }
    }
}

// --- node-wide state / sinks --------------------------------------------------------------------

/// The state the routing writes into and the supervisor drives, and that the future control loop
/// reads. Built alongside the [`NodeRuntime`] so its `motors` length and per-motor fusion buffers
/// match the config.
#[derive(Debug, Clone, Default)]
pub struct NodeState {
    /// Per-motor sinks, one per declared motor (same order as the config).
    pub motors: Vec<MotorSink, MAX_MOTORS>,
    /// The peer's wheel speed, routed in when `speed_sync_peer` names the receiving link.
    pub peer_wheel_speed: Option<i16>,
    /// The peer's cyclic status byte, routed in alongside attitude / wheel speed.
    pub peer_status: Option<u8>,
    /// A remote movement setpoint (from a consumed `Inputs`): the speed/steer entry point a
    /// balancing node uses, the same entry the local rider lean uses.
    pub remote_setpoint: Option<Inputs>,
    /// Set true when any link latches comms-loss (mirrors the per-link flags; a convenience the
    /// mode machine can sample as the link-loss fault flag).
    pub comms_loss: bool,
    /// Set true when a received `Fault { action = stop-all }` was consumed: request an immediate
    /// safe transition without waiting for an ack.
    pub stop_all_requested: bool,
    /// The most recent received fault code (non-stop-all faults also land here for visibility).
    pub received_fault: Option<u8>,
}

impl NodeState {
    fn new(rt: &NodeRuntime) -> NodeState {
        let mut motors: Vec<MotorSink, MAX_MOTORS> = Vec::new();
        for m in rt.motors.iter() {
            let fused_len = match &m.attitude_source {
                AttitudeSource::Fused(v) => v.len(),
                _ => 0,
            };
            let mut sink = MotorSink::new(fused_len);
            // A motor with no link feeding it has nothing to lose, so it is enabled from the start.
            if !m.depends_on_link() {
                sink.running_enable = true;
            }
            // Bounded by MAX_MOTORS; cannot overflow.
            let _ = motors.push(sink);
        }
        NodeState {
            motors,
            peer_wheel_speed: None,
            peer_status: None,
            remote_setpoint: None,
            comms_loss: false,
            stop_all_requested: false,
            received_fault: None,
        }
    }
}

// --- per-motor binding --------------------------------------------------------------------------

/// A motor's resolved sources, copied out of the config so routing does not re-borrow it.
#[derive(Debug, Clone)]
struct MotorBinding {
    attitude_source: AttitudeSource,
    drive_source: DriveSource,
}

impl MotorBinding {
    /// True if this motor's attitude or drive comes from a link (so a lost link should disable it).
    fn depends_on_link(&self) -> bool {
        matches!(self.drive_source, DriveSource::Link(_))
            || matches!(
                self.attitude_source,
                AttitudeSource::Link(_) | AttitudeSource::Fused(_)
            )
    }

    /// True if a lost link at `link_index` should disable this motor (feeds its attitude or drive).
    fn fed_by(&self, link_index: u8) -> bool {
        if let DriveSource::Link(i) = self.drive_source {
            if i == link_index {
                return true;
            }
        }
        match &self.attitude_source {
            AttitudeSource::Link(i) => *i == link_index,
            AttitudeSource::Fused(v) => v.iter().any(|&i| i == link_index),
            AttitudeSource::LocalImu => false,
        }
    }
}

// --- per-link binding ---------------------------------------------------------------------------

/// A link's resolved bindings, copied out of the config.
#[derive(Debug, Clone)]
struct LinkBinding {
    /// This node's id on the link.
    node_id: u8,
    /// Items this node produces onto the link.
    produce: ItemSet,
    /// Items this node consumes from the link.
    consume: ItemSet,
}

impl LinkBinding {
    /// The `NodeHello.caps` value for this link: the union of produce and consume.
    fn caps(&self) -> ItemSet {
        self.produce.union(self.consume)
    }
}

// --- supervision per link -----------------------------------------------------------------------

/// Per-link receive-timeout supervisor.
#[derive(Debug, Clone, Default)]
struct LinkSupervisor {
    /// Ticks since the last good frame.
    rx_timeout: u16,
    /// Latched comms-loss flag (recoverable: lowered when frames resume).
    comms_loss: bool,
}

// --- the runtime --------------------------------------------------------------------------------

/// The runtime routing + supervision layer built from a [`config::BoardConfig`].
pub struct NodeRuntime {
    /// This node's id (taken from the first declared link's `node_id`, the node's own id on the
    /// inter-board link). Used as the `src` of produced frames.
    this_node_id: u8,
    links: Vec<LinkBinding, MAX_LINKS>,
    motors: Vec<MotorBinding, MAX_MOTORS>,
    /// `speed_sync.peer` resolved to a declared-link index, if present.
    speed_sync_peer: Option<u8>,
    /// Per-link supervisors, indexed the same as `links`.
    supervisors: Vec<LinkSupervisor, MAX_LINKS>,
    /// A locally latched fault pending broadcast. Set by [`NodeRuntime::latch_local_fault`]; cleared
    /// (taken) by [`NodeRuntime::take_outbound_fault`] so it yields once per latch edge.
    pending_outbound_fault: Option<Fault>,
}

impl NodeRuntime {
    /// Build a runtime from a board definition. The companion [`NodeState`] is built via
    /// [`NodeRuntime::new_state`].
    pub fn from_config(cfg: &BoardConfig) -> NodeRuntime {
        let mut links: Vec<LinkBinding, MAX_LINKS> = Vec::new();
        let mut supervisors: Vec<LinkSupervisor, MAX_LINKS> = Vec::new();
        for l in cfg.links.iter() {
            let _ = links.push(LinkBinding {
                node_id: l.node_id,
                produce: l.produce,
                consume: l.consume,
            });
            let _ = supervisors.push(LinkSupervisor::default());
        }

        let mut motors: Vec<MotorBinding, MAX_MOTORS> = Vec::new();
        for m in cfg.motors.iter() {
            let _ = motors.push(MotorBinding {
                attitude_source: m.attitude_source.clone(),
                drive_source: m.drive_source,
            });
        }

        // This node's id: the first link's node_id (its id on the inter-board link). A board with
        // no links has no link identity; default to 0.
        let this_node_id = cfg.links.first().map(|l| l.node_id).unwrap_or(0);

        NodeRuntime {
            this_node_id,
            links,
            motors,
            speed_sync_peer: cfg.speed_sync_peer,
            supervisors,
            pending_outbound_fault: None,
        }
    }

    /// Build the companion [`NodeState`] whose `motors` length and fusion buffers match this
    /// runtime.
    pub fn new_state(&self) -> NodeState {
        NodeState::new(self)
    }

    /// This node's id (the `src` of produced frames).
    pub fn this_node_id(&self) -> u8 {
        self.this_node_id
    }

    /// The number of declared links.
    pub fn link_count(&self) -> usize {
        self.links.len()
    }

    // --- caps / hello ---------------------------------------------------------------------------

    /// The `NodeHello.caps` `ItemSet` for a link: the union of its produce and consume sets.
    pub fn caps(&self, link_index: usize) -> ItemSet {
        self.links
            .get(link_index)
            .map(|l| l.caps())
            .unwrap_or_else(ItemSet::empty)
    }

    /// Build a `NodeHello` for a link carrying that link's caps.
    pub fn node_hello(&self, link_index: usize) -> NodeHello {
        let caps = self.caps(link_index);
        NodeHello {
            node_id: self.this_node_id,
            role: 0,
            motor_count: self.motors.len() as u8,
            proto_ver: PROTO_VER,
            fw_ver: 0,
            caps: caps.bits(),
        }
    }

    // --- inbound routing (Phase 3) --------------------------------------------------------------

    /// Route a decoded frame received on `link_index` into `state`, per the binding model. An item
    /// outside this link's `consume` set is ignored after the CRC pass (no fault, no sink change).
    pub fn route_inbound(
        &mut self,
        link_index: usize,
        frame: &DecodedFrame,
        state: &mut NodeState,
    ) {
        let consume = match self.links.get(link_index) {
            Some(l) => l.consume,
            None => return,
        };
        let link_idx_u8 = link_index as u8;

        match frame.header.opcode {
            Opcode::CyclicState => {
                let cs = match CyclicState::decode(frame.payload) {
                    Ok(cs) => cs,
                    Err(_) => return,
                };
                // Attitude: an independently bound item.
                if consume.contains(DataItem::Attitude) {
                    self.route_attitude(link_idx_u8, cs.pitch, cs.roll, state);
                }
                // Wheel speed: routed to the speed-sync blend when this link is the named peer.
                if consume.contains(DataItem::WheelSpeed)
                    && self.speed_sync_peer == Some(link_idx_u8)
                {
                    state.peer_wheel_speed = Some(cs.wheel_speed);
                }
                // Status byte.
                if consume.contains(DataItem::Status) {
                    state.peer_status = Some(cs.status);
                }
            }
            Opcode::DriveCmd => {
                if !consume.contains(DataItem::DriveCmd) {
                    return;
                }
                let dc = match DriveCmd::decode(frame.payload) {
                    Ok(dc) => dc,
                    Err(_) => return,
                };
                // A direct motor command for any follower motor whose drive_source names this link.
                for (i, m) in self.motors.iter().enumerate() {
                    if let DriveSource::Link(src) = m.drive_source {
                        if src == link_idx_u8 {
                            if let Some(sink) = state.motors.get_mut(i) {
                                sink.drive_setpoint = Some(dc);
                            }
                        }
                    }
                }
            }
            Opcode::Inputs => {
                if !consume.contains(DataItem::Inputs) {
                    return;
                }
                if let Ok(inp) = Inputs::decode(frame.payload) {
                    // A movement setpoint into the speed/steer entry point (not a torque bypass).
                    state.remote_setpoint = Some(inp);
                }
            }
            Opcode::Fault => {
                if !consume.contains(DataItem::Fault) {
                    return;
                }
                if let Ok(f) = Fault::decode(frame.payload) {
                    state.received_fault = Some(f.fault_code);
                    if f.action == FAULT_ACTION_STOP_ALL {
                        // Immediate safe transition request, no ack.
                        state.stop_all_requested = true;
                    }
                }
            }
            _ => {
                // Any other opcode (NodeHello, Telemetry, Config*, Event, Unknown) is not routed to
                // a control sink here; ignored after CRC.
            }
        }
    }

    /// Deposit an attitude sample into every motor whose `attitude_source` names `link_index`.
    fn route_attitude(&self, link_index: u8, pitch: i16, roll: i16, state: &mut NodeState) {
        let att = CyclicAttitude { pitch, roll };
        for (i, m) in self.motors.iter().enumerate() {
            let sink = match state.motors.get_mut(i) {
                Some(s) => s,
                None => continue,
            };
            match &m.attitude_source {
                AttitudeSource::Link(src) if *src == link_index => {
                    sink.attitude_in = Some(att);
                }
                AttitudeSource::Fused(v) => {
                    // Deposit into the positional slot matching this link in the fused list.
                    for (slot, &src) in v.iter().enumerate() {
                        if src == link_index {
                            if let Some(cell) = sink.fusion_buf.get_mut(slot) {
                                *cell = Some(att);
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // --- outbound production (Phase 3) ----------------------------------------------------------

    /// Build the cyclic frame this link produces this tick from local attitude / wheel speed /
    /// status, if the link's `produce` set asks for any of those items. Returns the opcode, the
    /// payload, and the destination (`BROADCAST` for the point-to-point pair, since the cyclic frame
    /// is not addressed to a specific peer id here). Returns `None` when the link produces none of
    /// the cyclic items.
    ///
    /// `src` of the eventual frame is `this_node_id`; the caller stamps the header.
    pub fn build_cyclic(
        &self,
        link_index: usize,
        local: &LocalCyclic,
    ) -> Option<(Opcode, CyclicState, u8)> {
        let link = self.links.get(link_index)?;
        let produces_cyclic = link.produce.contains(DataItem::Attitude)
            || link.produce.contains(DataItem::WheelSpeed)
            || link.produce.contains(DataItem::Status);
        if !produces_cyclic {
            return None;
        }
        let cs = CyclicState {
            pitch: if link.produce.contains(DataItem::Attitude) {
                local.pitch
            } else {
                0
            },
            roll: if link.produce.contains(DataItem::Attitude) {
                local.roll
            } else {
                0
            },
            wheel_speed: if link.produce.contains(DataItem::WheelSpeed) {
                local.wheel_speed
            } else {
                0
            },
            status: if link.produce.contains(DataItem::Status) {
                local.status
            } else {
                0
            },
        };
        Some((Opcode::CyclicState, cs, BROADCAST))
    }

    /// The source id to stamp on produced frames (`this_node_id`).
    pub fn produce_src(&self) -> u8 {
        self.this_node_id
    }

    /// The peer node id on a link (its declared `node_id`), or `None` for an out-of-range index.
    pub fn link_node_id(&self, link_index: usize) -> Option<u8> {
        self.links.get(link_index).map(|l| l.node_id)
    }

    // --- supervision (Phase 4) ------------------------------------------------------------------

    /// Note a good frame on `link_index`: clear the receive-timeout counter and, if the comms-loss
    /// flag was latched, lower it and re-enable any motor fed only by recovered links.
    pub fn note_frame(&mut self, link_index: usize, state: &mut NodeState) {
        if let Some(sup) = self.supervisors.get_mut(link_index) {
            sup.rx_timeout = 0;
            sup.comms_loss = false;
        } else {
            return;
        }
        // A good frame on a healthy link enables any motor fed by it (and clears the node-wide
        // comms-loss flag if no other link is lost). Always refresh so the first frame on a
        // link-fed motor enables it.
        self.refresh_enables(state);
    }

    /// Advance the receive-timeout for `link_index` by one cyclic tick with no good frame. Crossing
    /// [`RX_TIMEOUT_TICKS`] latches comms-loss and disables any motor fed by the link.
    pub fn tick_no_frame(&mut self, link_index: usize, state: &mut NodeState) {
        let crossed = {
            let sup = match self.supervisors.get_mut(link_index) {
                Some(s) => s,
                None => return,
            };
            if sup.comms_loss {
                // Already latched; keep the counter saturated.
                sup.rx_timeout = sup.rx_timeout.saturating_add(1);
                false
            } else {
                sup.rx_timeout = sup.rx_timeout.saturating_add(1);
                if sup.rx_timeout >= RX_TIMEOUT_TICKS {
                    sup.comms_loss = true;
                    true
                } else {
                    false
                }
            }
        };
        if crossed {
            self.refresh_enables(state);
        }
    }

    /// True if `link_index` has latched comms-loss.
    pub fn comms_loss(&self, link_index: usize) -> bool {
        self.supervisors
            .get(link_index)
            .map(|s| s.comms_loss)
            .unwrap_or(false)
    }

    /// True if any link has latched comms-loss.
    pub fn any_comms_loss(&self) -> bool {
        self.supervisors.iter().any(|s| s.comms_loss)
    }

    /// Recompute every motor's `running_enable` and the node-wide `comms_loss` flag from the
    /// current per-link comms-loss state. A motor fed by a lost link is disabled; a motor whose
    /// feeding links are all healthy (or that has no link source) is enabled.
    fn refresh_enables(&self, state: &mut NodeState) {
        let any = self.any_comms_loss();
        state.comms_loss = any;
        for (i, m) in self.motors.iter().enumerate() {
            let sink = match state.motors.get_mut(i) {
                Some(s) => s,
                None => continue,
            };
            if !m.depends_on_link() {
                sink.running_enable = true;
                continue;
            }
            // Disabled iff any link feeding this motor is in comms-loss.
            let mut lost = false;
            for (li, sup) in self.supervisors.iter().enumerate() {
                if sup.comms_loss && m.fed_by(li as u8) {
                    lost = true;
                    break;
                }
            }
            sink.running_enable = !lost;
        }
    }

    /// Latch a local fault to broadcast on producing links. Yields once per latch edge via
    /// [`NodeRuntime::take_outbound_fault`]. Calling this while a fault is already pending replaces
    /// the pending value (still one broadcast per call edge).
    pub fn latch_local_fault(&mut self, fault: Fault) {
        self.pending_outbound_fault = Some(fault);
    }

    /// Take the pending broadcast `Fault`, if any. Returns it once per latch edge (subsequent calls
    /// return `None` until another fault is latched). The caller stamps `dst = BROADCAST` and
    /// `src = this_node_id` and emits it on producing links.
    pub fn take_outbound_fault(&mut self) -> Option<Fault> {
        self.pending_outbound_fault.take()
    }

    /// True if `link_index` produces `item` (so a broadcast fault should go out on it).
    pub fn link_produces(&self, link_index: usize, item: DataItem) -> bool {
        self.links
            .get(link_index)
            .map(|l| l.produce.contains(item))
            .unwrap_or(false)
    }
}

/// Local cyclic values a board offers for production (from its own IMU / wheel sensor / status).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LocalCyclic {
    pub pitch: i16,
    pub roll: i16,
    pub wheel_speed: i16,
    pub status: u8,
}

#[cfg(test)]
mod tests;
