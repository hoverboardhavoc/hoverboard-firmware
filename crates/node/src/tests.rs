//! Host tests for Phase 3 (routing) + Phase 4 (supervision) + the Phase 6 loopback.
//!
//! Configs are built by encoding the same string-keyed CBOR the config crate tests use, framing it
//! with the board magic/version/length + CRC-16/MODBUS, and running it through `config::parse` (the
//! real path). Tests link `std` via the host target; the library is `no_std`.

extern crate std;
use std::vec::Vec as StdVec;

use minicbor::Encoder;

use super::*;
use config::{parse, BoardConfig};
use link::frame::{encode, DecodedFrame, FrameHeader, BROADCAST, MAX_FRAME, PROTO_VER};
use link::item::DataItem;
use link::opcode::Opcode;
use link::payload::{CyclicState, DriveCmd, Fault, Inputs};
use link::StreamFramer;

// --- config-blob helpers (mirror crates/config/src/tests.rs) -----------------------------------

const BOARD_MAGIC: u32 = u32::from_le_bytes(*b"HBRD");
const BOARD_VERSION: u16 = 1;

fn encode_payload<F>(f: F) -> StdVec<u8>
where
    F: FnOnce(
        &mut Encoder<&mut [u8]>,
    ) -> Result<(), minicbor::encode::Error<minicbor::encode::write::EndOfSlice>>,
{
    let mut buf = [0u8; 2048];
    let total = buf.len();
    let remaining = {
        let mut enc = Encoder::new(&mut buf[..]);
        f(&mut enc).expect("encode");
        enc.into_writer().len()
    };
    let written = total - remaining;
    buf[..written].to_vec()
}

fn frame_blob(payload: &[u8]) -> StdVec<u8> {
    let mut out = StdVec::new();
    out.extend_from_slice(&BOARD_MAGIC.to_le_bytes());
    out.extend_from_slice(&BOARD_VERSION.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    out.extend_from_slice(&link::crc16::modbus(payload).to_le_bytes());
    out.extend_from_slice(payload);
    out
}

/// An articulated half: one motor, attitude_source local_imu, drive_source local_balance, one link
/// "peer" producing and consuming [attitude, wheel_speed, status], speed_sync.peer = peer.
/// `node_id` and the peer's id are parameters so the two halves of a pair get distinct ids.
fn articulated_cfg(node_id: u8) -> BoardConfig {
    let payload = encode_payload(|e| {
        e.map(6)?;
        e.str("format")?.u8(1)?;
        e.str("name")?.str("articulated-half")?;
        e.str("control")?.map(1)?.str("mode")?.str("articulated")?;
        e.str("speed_sync")?.map(1)?.str("peer")?.str("link:peer")?;
        e.str("links")?.array(1)?;
        {
            e.map(7)?;
            e.str("name")?.str("peer")?;
            e.str("usart")?.str("USART1")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(node_id)?;
            e.str("arbitration")?.str("initiator")?;
            e.str("produce")?
                .array(3)?
                .str("attitude")?
                .str("wheel_speed")?
                .str("status")?;
            e.str("consume")?
                .array(3)?
                .str("attitude")?
                .str("wheel_speed")?
                .str("status")?;
        }
        e.str("motors")?.array(1)?;
        {
            e.map(3)?;
            e.str("timer")?.str("TIMER0")?;
            e.str("attitude_source")?.str("local_imu")?;
            e.str("drive_source")?.str("local_balance")?;
        }
        Ok(())
    });
    parse(&frame_blob(&payload)).expect("parse articulated")
}

/// A mainboard: 2 motors, attitude_source link:sideA / link:sideB, drive_source link:sideA /
/// link:sideB (rigid followers), two links sideA/sideB consuming [attitude, drive_cmd].
fn mainboard_cfg() -> BoardConfig {
    let payload = encode_payload(|e| {
        e.map(5)?;
        e.str("format")?.u8(1)?;
        e.str("name")?.str("mainboard")?;
        e.str("control")?.map(1)?.str("mode")?.str("rigid")?;
        e.str("links")?.array(2)?;
        for (nm, node) in [("sideA", 2u8), ("sideB", 3u8)] {
            e.map(6)?;
            e.str("name")?.str(nm)?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(node)?;
            e.str("arbitration")?.str("bus_master")?;
            e.str("consume")?
                .array(2)?
                .str("attitude")?
                .str("drive_cmd")?;
        }
        e.str("motors")?.array(2)?;
        for src in ["sideA", "sideB"] {
            e.map(3)?;
            e.str("timer")?.str("TIMER0")?;
            let link_ref = std::format!("link:{src}");
            e.str("attitude_source")?.str(&link_ref)?;
            e.str("drive_source")?.str(&link_ref)?;
        }
        Ok(())
    });
    parse(&frame_blob(&payload)).expect("parse mainboard")
}

/// A sideboard sensor node: 0 motors, one link "up" producing [attitude].
fn sideboard_cfg() -> BoardConfig {
    let payload = encode_payload(|e| {
        e.map(3)?;
        e.str("name")?.str("sideboard")?;
        e.str("control")?.map(1)?.str("mode")?.str("articulated")?;
        e.str("links")?.array(1)?;
        {
            e.map(6)?;
            e.str("name")?.str("up")?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(2)?;
            e.str("arbitration")?.str("follower")?;
            e.str("produce")?.array(1)?.str("attitude")?;
        }
        Ok(())
    });
    parse(&frame_blob(&payload)).expect("parse sideboard")
}

// --- frame helpers ------------------------------------------------------------------------------

/// Encode a frame into a buffer and return the decoded view borrowing that buffer.
fn make_frame(opcode: Opcode, payload: &[u8], src: u8, dst: u8) -> ([u8; MAX_FRAME], usize) {
    let mut out = [0u8; MAX_FRAME];
    let hdr = FrameHeader { ver: PROTO_VER, opcode, src, dst, len: payload.len() as u8 };
    let n = encode(&hdr, payload, &mut out).unwrap();
    (out, n)
}

fn decode_frame(buf: &[u8]) -> DecodedFrame<'_> {
    link::frame::decode(buf).unwrap()
}

fn item_set(items: &[DataItem]) -> ItemSet {
    let mut s = ItemSet::empty();
    for &i in items {
        s.insert(i);
    }
    s
}

// --- Phase 3: caps / node_hello ----------------------------------------------------------------

#[test]
fn caps_equal_produce_consume_union() {
    let cfg = articulated_cfg(1);
    let rt = NodeRuntime::from_config(&cfg);
    let want = item_set(&[DataItem::Attitude, DataItem::WheelSpeed, DataItem::Status]);
    assert_eq!(rt.caps(0), want);

    let hello = rt.node_hello(0);
    assert_eq!(hello.caps, want.bits());
    assert_eq!(hello.node_id, 1);
    assert_eq!(hello.motor_count, 1);
    assert_eq!(hello.proto_ver, PROTO_VER);
}

#[test]
fn caps_union_when_produce_and_consume_differ() {
    // Mainboard sideA consumes [attitude, drive_cmd], produces nothing -> caps is just consume.
    let cfg = mainboard_cfg();
    let rt = NodeRuntime::from_config(&cfg);
    let want = item_set(&[DataItem::Attitude, DataItem::DriveCmd]);
    assert_eq!(rt.caps(0), want);
}

// --- Phase 3: routing --------------------------------------------------------------------------

#[test]
fn cyclic_attitude_and_wheel_speed_route_to_sinks() {
    let cfg = articulated_cfg(1);
    let mut rt = NodeRuntime::from_config(&cfg);
    // Force the motor's attitude_source onto the peer link so attitude routes to it (the synthetic
    // articulated half is local_imu; rebind via a mainboard-style config would be cleaner, but the
    // task asks to assert attitude lands in the expected motor's sink, so use the mainboard below).
    // Here assert wheel_speed + status route node-wide regardless of motor binding.
    let mut state = rt.new_state();

    let cs = CyclicState { pitch: 111, roll: -22, wheel_speed: 1234, status: 0b1010 };
    let mut buf = [0u8; CyclicState::LEN];
    cs.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::CyclicState, &buf, 2, 1);
    let f = decode_frame(&fb[..n]);

    rt.route_inbound(0, &f, &mut state);
    assert_eq!(state.peer_wheel_speed, Some(1234));
    assert_eq!(state.peer_status, Some(0b1010));
    // local_imu motor takes no link attitude.
    assert_eq!(state.motors[0].attitude_in, None);
}

#[test]
fn cyclic_attitude_routes_to_link_bound_motor() {
    let cfg = mainboard_cfg();
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    // Motor 0 attitude_source = link:sideA (index 0). Feed a CyclicState on link 0.
    let cs = CyclicState { pitch: 500, roll: -300, wheel_speed: 7, status: 1 };
    let mut buf = [0u8; CyclicState::LEN];
    cs.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::CyclicState, &buf, 2, 0);
    let f = decode_frame(&fb[..n]);

    rt.route_inbound(0, &f, &mut state);
    assert_eq!(
        state.motors[0].attitude_in,
        Some(CyclicAttitude { pitch: 500, roll: -300 })
    );
    // Motor 1 (sideB) untouched by a sideA frame.
    assert_eq!(state.motors[1].attitude_in, None);
}

#[test]
fn drive_cmd_routes_to_follower_motor() {
    let cfg = mainboard_cfg();
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    let dc = DriveCmd { kind: 1, value: -250, steer: 40 };
    let mut buf = [0u8; DriveCmd::LEN];
    dc.encode(&mut buf);
    // Send on link 1 (sideB) -> motor 1's drive_source = Link(1).
    let (fb, n) = make_frame(Opcode::DriveCmd, &buf, 3, 0);
    let f = decode_frame(&fb[..n]);

    rt.route_inbound(1, &f, &mut state);
    assert_eq!(state.motors[1].drive_setpoint, Some(dc));
    assert_eq!(state.motors[0].drive_setpoint, None);
}

#[test]
fn item_outside_consume_is_ignored() {
    // Sideboard "up" link produces [attitude] and consumes nothing. A CyclicState arriving on it
    // must change no sink.
    let cfg = sideboard_cfg();
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    let cs = CyclicState { pitch: 9, roll: 9, wheel_speed: 9, status: 9 };
    let mut buf = [0u8; CyclicState::LEN];
    cs.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::CyclicState, &buf, 1, 2);
    let f = decode_frame(&fb[..n]);

    rt.route_inbound(0, &f, &mut state);
    assert_eq!(state.peer_wheel_speed, None);
    assert_eq!(state.peer_status, None);
}

#[test]
fn drive_cmd_to_balancing_motor_is_not_a_torque_bypass() {
    // The articulated half's motor is drive_source = local_balance. A DriveCmd on its link must NOT
    // set drive_setpoint (a balancing motor does not take DriveCmd as a direct command). The link
    // does not even consume drive_cmd here, so it is doubly ignored.
    let cfg = articulated_cfg(1);
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    let dc = DriveCmd { kind: 1, value: 999, steer: 0 };
    let mut buf = [0u8; DriveCmd::LEN];
    dc.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::DriveCmd, &buf, 2, 1);
    let f = decode_frame(&fb[..n]);

    rt.route_inbound(0, &f, &mut state);
    assert_eq!(state.motors[0].drive_setpoint, None);
}

#[test]
fn inputs_route_to_remote_setpoint() {
    // A board that consumes inputs routes it to the remote_setpoint sink.
    let payload = encode_payload(|e| {
        e.map(3)?;
        e.str("name")?.str("inputs-consumer")?;
        e.str("links")?.array(1)?;
        {
            e.map(5)?;
            e.str("name")?.str("app")?;
            e.str("usart")?.str("USART2")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(9)?;
            e.str("consume")?.array(1)?.str("inputs")?;
        }
        e.str("motors")?.array(1)?;
        {
            e.map(3)?;
            e.str("timer")?.str("TIMER0")?;
            e.str("attitude_source")?.str("local_imu")?;
            e.str("drive_source")?.str("local_balance")?;
        }
        Ok(())
    });
    let cfg = parse(&frame_blob(&payload)).expect("parse inputs-consumer");
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    let inp = Inputs { throttle: -700, buttons: 0x03, rider: 1 };
    let mut buf = [0u8; Inputs::LEN];
    inp.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::Inputs, &buf, 9, 0);
    let f = decode_frame(&fb[..n]);

    rt.route_inbound(0, &f, &mut state);
    assert_eq!(state.remote_setpoint, Some(inp));
}

#[test]
fn fused_attitude_routes_to_positional_buffer() {
    // A motor fused over [sideA, sideB]: a frame on sideA fills slot 0, sideB fills slot 1.
    let payload = encode_payload(|e| {
        e.map(3)?;
        e.str("name")?.str("fused-board")?;
        e.str("links")?.array(2)?;
        for (nm, node) in [("sideA", 2u8), ("sideB", 3u8)] {
            e.map(5)?;
            e.str("name")?.str(nm)?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(node)?;
            e.str("consume")?.array(1)?.str("attitude")?;
        }
        e.str("motors")?.array(1)?;
        {
            e.map(2)?;
            e.str("timer")?.str("TIMER0")?;
            e.str("attitude_source")?
                .map(1)?
                .str("fused")?
                .array(2)?
                .str("link:sideA")?
                .str("link:sideB")?;
        }
        Ok(())
    });
    let cfg = parse(&frame_blob(&payload)).expect("parse fused");
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    let csa = CyclicState { pitch: 10, roll: 20, wheel_speed: 0, status: 0 };
    let mut buf = [0u8; CyclicState::LEN];
    csa.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::CyclicState, &buf, 2, 0);
    rt.route_inbound(0, &decode_frame(&fb[..n]), &mut state);

    let csb = CyclicState { pitch: 30, roll: 40, wheel_speed: 0, status: 0 };
    csb.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::CyclicState, &buf, 3, 0);
    rt.route_inbound(1, &decode_frame(&fb[..n]), &mut state);

    assert_eq!(state.motors[0].fusion_buf[0], Some(CyclicAttitude { pitch: 10, roll: 20 }));
    assert_eq!(state.motors[0].fusion_buf[1], Some(CyclicAttitude { pitch: 30, roll: 40 }));
    // Fused does not populate the single-source attitude_in slot.
    assert_eq!(state.motors[0].attitude_in, None);
}

// --- Phase 3: production -----------------------------------------------------------------------

#[test]
fn build_cyclic_emits_local_values() {
    let cfg = articulated_cfg(1);
    let rt = NodeRuntime::from_config(&cfg);
    let local = LocalCyclic { pitch: -45, roll: 12, wheel_speed: 600, status: 0b0101 };
    let (op, cs, dst) = rt.build_cyclic(0, &local).expect("produces cyclic");
    assert_eq!(op, Opcode::CyclicState);
    assert_eq!(cs.pitch, -45);
    assert_eq!(cs.wheel_speed, 600);
    assert_eq!(cs.status, 0b0101);
    assert_eq!(dst, BROADCAST);
    assert_eq!(rt.produce_src(), 1);
}

#[test]
fn build_cyclic_none_when_link_produces_nothing_cyclic() {
    // Mainboard sideA produces nothing -> no cyclic frame.
    let cfg = mainboard_cfg();
    let rt = NodeRuntime::from_config(&cfg);
    assert!(rt.build_cyclic(0, &LocalCyclic::default()).is_none());
}

// --- Phase 4: supervision ----------------------------------------------------------------------

#[test]
fn comms_loss_latches_and_disables_fed_motor() {
    let cfg = mainboard_cfg();
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    // Motor 0 is fed by link 0 (sideA). A link-fed motor starts disabled until a frame arrives.
    assert!(!state.motors[0].running_enable);
    // A good frame enables it.
    rt.note_frame(0, &mut state);
    assert!(state.motors[0].running_enable);

    // Withhold frames on link 0 past the threshold.
    for _ in 0..RX_TIMEOUT_TICKS {
        rt.tick_no_frame(0, &mut state);
    }
    assert!(rt.comms_loss(0));
    assert!(state.comms_loss);
    assert!(!state.motors[0].running_enable);
    // Motor 1 (sideB) is unaffected: its link never timed out (but it was never noted, so still
    // disabled by default). Note it then confirm it stays enabled.
    rt.note_frame(1, &mut state);
    assert!(state.motors[1].running_enable);

    // Resume frames on link 0: the flag lowers and the motor re-enables.
    rt.note_frame(0, &mut state);
    assert!(!rt.comms_loss(0));
    assert!(!state.comms_loss);
    assert!(state.motors[0].running_enable);
}

#[test]
fn comms_loss_does_not_latch_before_threshold() {
    let cfg = mainboard_cfg();
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();
    rt.note_frame(0, &mut state);

    for _ in 0..(RX_TIMEOUT_TICKS - 1) {
        rt.tick_no_frame(0, &mut state);
    }
    assert!(!rt.comms_loss(0));
    assert!(state.motors[0].running_enable);
}

#[test]
fn received_stop_all_fault_requests_safe_transition() {
    // A board consuming [fault].
    let payload = encode_payload(|e| {
        e.map(2)?;
        e.str("name")?.str("fault-consumer")?;
        e.str("links")?.array(1)?;
        {
            e.map(5)?;
            e.str("name")?.str("peer")?;
            e.str("usart")?.str("USART1")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(1)?;
            e.str("consume")?.array(1)?.str("fault")?;
        }
        Ok(())
    });
    let cfg = parse(&frame_blob(&payload)).expect("parse fault-consumer");
    let mut rt = NodeRuntime::from_config(&cfg);
    let mut state = rt.new_state();

    let f = Fault { fault_code: 7, action: FAULT_ACTION_STOP_ALL };
    let mut buf = [0u8; Fault::LEN];
    f.encode(&mut buf);
    let (fb, n) = make_frame(Opcode::Fault, &buf, 2, 1);
    rt.route_inbound(0, &decode_frame(&fb[..n]), &mut state);

    assert!(state.stop_all_requested);
    assert_eq!(state.received_fault, Some(7));
}

#[test]
fn local_fault_yields_one_outbound_broadcast() {
    let cfg = articulated_cfg(1);
    let mut rt = NodeRuntime::from_config(&cfg);

    assert!(rt.take_outbound_fault().is_none());
    rt.latch_local_fault(Fault { fault_code: 3, action: FAULT_ACTION_STOP_ALL });
    let out = rt.take_outbound_fault().expect("one fault");
    assert_eq!(out.fault_code, 3);
    // Only once per latch edge.
    assert!(rt.take_outbound_fault().is_none());
    // The producing peer link should carry the fault broadcast (it produces attitude etc.); the
    // node would stamp dst = BROADCAST. Confirm the link is a producing link.
    assert!(rt.link_produces(0, DataItem::Attitude));
}

// --- Phase 6: two-instance loopback ------------------------------------------------------------

/// One side of the loopback: a runtime, its state, its inbound framer, and the local values it
/// produces.
struct Side {
    rt: NodeRuntime,
    state: NodeState,
    framer: StreamFramer,
    local: LocalCyclic,
}

impl Side {
    fn new(cfg: &BoardConfig, local: LocalCyclic) -> Side {
        let rt = NodeRuntime::from_config(cfg);
        let state = rt.new_state();
        Side { rt, state, framer: StreamFramer::new(), local }
    }

    /// Produce this side's cyclic frame for link 0 into a wire buffer; returns the bytes.
    fn produce_bytes(&self) -> ([u8; MAX_FRAME], usize) {
        let (op, cs, dst) = self.rt.build_cyclic(0, &self.local).expect("cyclic");
        let mut pbuf = [0u8; CyclicState::LEN];
        let plen = cs.encode(&mut pbuf);
        let hdr = FrameHeader {
            ver: PROTO_VER,
            opcode: op,
            src: self.rt.produce_src(),
            dst,
            len: plen as u8,
        };
        let mut out = [0u8; MAX_FRAME];
        let n = encode(&hdr, &pbuf[..plen], &mut out).unwrap();
        (out, n)
    }

    /// Feed received bytes into the framer; route every complete frame on link 0 and note it.
    fn receive_bytes(&mut self, bytes: &[u8]) {
        // Collect decoded frames first (the framer borrows; route after).
        let mut frames: StdVec<([u8; MAX_FRAME], usize)> = StdVec::new();
        self.framer.feed(bytes, &mut |f| {
            // Re-encode into an owned buffer so we can route after the borrow ends.
            let mut out = [0u8; MAX_FRAME];
            let n = encode(&f.header, f.payload, &mut out).unwrap();
            frames.push((out, n));
        });
        for (buf, n) in frames.iter() {
            let f = link::frame::decode(&buf[..*n]).unwrap();
            self.rt.route_inbound(0, &f, &mut self.state);
            self.rt.note_frame(0, &mut self.state);
        }
    }
}

#[test]
fn loopback_articulated_pair_exchanges_cyclic() {
    // Two halves. Each consumes [attitude, wheel_speed, status] but its motor is local_imu, so peer
    // attitude lands node-wide in peer_status / peer_wheel_speed (attitude_in stays None for a
    // local_imu motor, which is correct). Assert wheel_speed crosses each tick.
    let cfg_a = articulated_cfg(1);
    let cfg_b = articulated_cfg(2);
    let mut a = Side::new(&cfg_a, LocalCyclic { pitch: 100, roll: 0, wheel_speed: 111, status: 1 });
    let mut b = Side::new(&cfg_b, LocalCyclic { pitch: -50, roll: 0, wheel_speed: 222, status: 2 });

    for _ in 0..6 {
        let (abuf, an) = a.produce_bytes();
        let (bbuf, bn) = b.produce_bytes();
        // Cross-connect: A's frame to B, B's frame to A.
        b.receive_bytes(&abuf[..an]);
        a.receive_bytes(&bbuf[..bn]);
    }

    // Each side sees the other's produced wheel speed and status.
    assert_eq!(a.state.peer_wheel_speed, Some(222));
    assert_eq!(a.state.peer_status, Some(2));
    assert_eq!(b.state.peer_wheel_speed, Some(111));
    assert_eq!(b.state.peer_status, Some(1));
}

#[test]
fn loopback_withholding_frames_latches_comms_loss() {
    let cfg_a = articulated_cfg(1);
    let cfg_b = articulated_cfg(2);
    let mut a = Side::new(&cfg_a, LocalCyclic { pitch: 0, roll: 0, wheel_speed: 1, status: 0 });
    let mut b = Side::new(&cfg_b, LocalCyclic { pitch: 0, roll: 0, wheel_speed: 2, status: 0 });

    // A few healthy ticks.
    for _ in 0..3 {
        let (abuf, an) = a.produce_bytes();
        b.receive_bytes(&abuf[..an]);
        let (bbuf, bn) = b.produce_bytes();
        a.receive_bytes(&bbuf[..bn]);
    }
    assert!(!b.rt.comms_loss(0));

    // Now A stops producing. B sees no frames and ticks its timeout.
    for _ in 0..RX_TIMEOUT_TICKS {
        b.rt.tick_no_frame(0, &mut b.state);
    }
    assert!(b.rt.comms_loss(0));
    assert!(b.state.comms_loss);
}
