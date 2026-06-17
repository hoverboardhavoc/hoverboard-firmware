//! Host tests: build a board blob in-test (minicbor-encode the string-keyed payload, then frame it
//! with magic/version/length + CRC via `link::crc16::modbus`), round-trip through `parse`, and
//! assert the decoded structs match. Negative tests cover every frame and validation failure.
//!
//! Tests link `std` via the host target; the library itself is `no_std`.

extern crate std;
use std::vec::Vec as StdVec;

use minicbor::Encoder;

use super::*;
use link::item::DataItem;

// --- encoding helpers -------------------------------------------------------------------------

/// Encode a closure's CBOR into a fresh buffer and return the written bytes. minicbor's `Vec`/`std`
/// Write impls are off (default-features = false), so we encode into a fixed `&mut [u8]` and slice
/// to the consumed length via the remaining-writer length.
fn encode_payload<F>(f: F) -> StdVec<u8>
where
    F: FnOnce(&mut Encoder<&mut [u8]>) -> Result<(), minicbor::encode::Error<minicbor::encode::write::EndOfSlice>>,
{
    let mut buf = [0u8; 1024];
    let total = buf.len();
    let remaining = {
        let mut enc = Encoder::new(&mut buf[..]);
        f(&mut enc).expect("encode");
        // `&mut [u8]` Write advances the slice; the leftover length tells us how much was written.
        enc.into_writer().len()
    };
    let written = total - remaining;
    buf[..written].to_vec()
}

/// Frame a CBOR payload with the board magic/version/length + CRC-16/MODBUS, producing a blob
/// `parse` accepts.
fn frame(payload: &[u8]) -> StdVec<u8> {
    let mut out = StdVec::new();
    out.extend_from_slice(&BOARD_MAGIC.to_le_bytes());
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    out.extend_from_slice(&crc16::modbus(payload).to_le_bytes());
    out.extend_from_slice(payload);
    out
}

// --- board (1): reference articulated half ---------------------------------------------------
//
// 1 motor, attitude_source local_imu, drive_source local_balance, one link "peer" producing and
// consuming [attitude, wheel_speed, status], speed_sync.peer = peer. Mirrors the worked example in
// board-config.md.

fn board_payload() -> StdVec<u8> {
    encode_payload(|e| {
        e.map(8)?;
        e.str("format")?.u8(1)?;
        e.str("name")?.str("example-master-2-2-20")?;
        e.str("control")?.map(1)?.str("mode")?.str("articulated")?;
        e.str("speed_sync")?.map(1)?.str("peer")?.str("link:peer")?;
        // links
        e.str("links")?.array(1)?;
        {
            e.map(7)?;
            e.str("name")?.str("peer")?;
            e.str("usart")?.str("USART1")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(1)?;
            e.str("arbitration")?.str("initiator")?;
            e.str("produce")?.array(3)?
                .str("attitude")?.str("wheel_speed")?.str("status")?;
            e.str("consume")?.array(3)?
                .str("attitude")?.str("wheel_speed")?.str("status")?;
        }
        // battery / limits
        e.str("battery")?.map(1)?.str("cells")?.u8(10)?;
        e.str("limits")?.map(2)?
            .str("current_ma")?.u32(8000)?
            .str("speed_max")?.u32(1000)?;
        // motors
        e.str("motors")?.array(1)?;
        {
            e.map(6)?;
            e.str("timer")?.str("TIMER0")?;
            e.str("attitude_source")?.str("local_imu")?;
            e.str("drive_source")?.str("local_balance")?;
            e.str("phases")?.map(3)?;
            e.str("a")?.map(2)?.str("hi")?.str("PA8")?.str("lo")?.str("PB13")?;
            e.str("b")?.map(2)?.str("hi")?.str("PA9")?.str("lo")?.str("PB14")?;
            e.str("c")?.map(2)?.str("hi")?.str("PA10")?.str("lo")?.str("PB15")?;
            e.str("halls")?.array(3)?.str("PA4")?.str("PA5")?.str("PA6")?;
            e.str("current_sense")?.map(3)?
                .str("pin")?.str("PB1")?
                .str("adc")?.str("ADC0")?
                .str("channel")?.u8(9)?;
        }
        Ok(())
    })
}

#[test]
fn board_round_trip() {
    let payload = board_payload();
    let blob = frame(&payload);
    let cfg = parse(&blob).expect("parse reference");

    assert_eq!(cfg.format, 1);
    assert_eq!(cfg.name.as_str(), "example-master-2-2-20");
    assert_eq!(cfg.control_mode, ControlMode::Articulated);
    assert_eq!(cfg.battery_cells, 10);
    assert_eq!(cfg.limits.current_ma, 8000);
    assert_eq!(cfg.limits.speed_max, 1000);

    // one link "peer" at index 0
    assert_eq!(cfg.links.len(), 1);
    let peer = &cfg.links[0];
    assert_eq!(peer.name.as_str(), "peer");
    assert_eq!(peer.usart_label.as_str(), "USART1");
    assert_eq!(peer.baud, 115200);
    assert_eq!(peer.node_id, 1);
    assert_eq!(peer.arbitration, Arbitration::Initiator);
    assert!(peer.device_name.is_none());

    // produce/consume = [attitude, wheel_speed, status]
    let mut want = ItemSet::empty();
    want.insert(DataItem::Attitude);
    want.insert(DataItem::WheelSpeed);
    want.insert(DataItem::Status);
    assert_eq!(peer.produce, want);
    assert_eq!(peer.consume, want);

    // speed_sync.peer resolves to link index 0
    assert_eq!(cfg.speed_sync_peer, Some(0));

    // one motor, local sources
    assert_eq!(cfg.motors.len(), 1);
    let m = &cfg.motors[0];
    assert_eq!(m.timer_label.as_str(), "TIMER0");
    assert_eq!(m.attitude_source, AttitudeSource::LocalImu);
    assert_eq!(m.drive_source, DriveSource::LocalBalance);
    // phases flattened to 6 pin labels in a/b/c hi/lo order
    assert_eq!(m.phase_labels.len(), 6);
    assert_eq!(m.phase_labels[0].as_str(), "PA8");
    assert_eq!(m.phase_labels[1].as_str(), "PB13");
    assert_eq!(m.phase_labels[5].as_str(), "PB15");
    assert_eq!(m.hall_labels.len(), 3);
    assert_eq!(m.hall_labels[0].as_str(), "PA4");
    assert_eq!(m.current_sense_label.as_deref(), Some("PB1"));
}

#[test]
fn board_crc_matches_helper() {
    // The framed blob's CRC bytes equal link::crc16::modbus(payload), byte-for-byte.
    let payload = board_payload();
    let blob = frame(&payload);
    let crc_in_frame = u16::from_le_bytes([blob[8], blob[9]]);
    assert_eq!(crc_in_frame, crc16::modbus(&payload));
}

// --- board (2): 12-FET mainboard + attitude sideboard -----------------------------------------
//
// Mainboard: 2 motors, attitude_source link:sideA / link:sideB, two links consuming [attitude].
// Sideboard: 0 motors, one link producing [attitude].

fn mainboard_payload() -> StdVec<u8> {
    encode_payload(|e| {
        e.map(6)?;
        e.str("format")?.u8(1)?;
        e.str("name")?.str("12fet-mainboard")?;
        e.str("control")?.map(1)?.str("mode")?.str("rigid")?;
        // two links sideA, sideB, both consuming [attitude]
        e.str("links")?.array(2)?;
        for (nm, node) in [("sideA", 2u8), ("sideB", 3u8)] {
            e.map(6)?;
            e.str("name")?.str(nm)?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(node)?;
            e.str("arbitration")?.str("bus_master")?;
            e.str("consume")?.array(1)?.str("attitude")?;
        }
        // two motors, each driven by local balance, attitude from the matching side link
        e.str("motors")?.array(2)?;
        for src in ["link:sideA", "link:sideB"] {
            e.map(3)?;
            e.str("timer")?.str("TIMER0")?;
            e.str("attitude_source")?.str(src)?;
            e.str("drive_source")?.str("local_balance")?;
        }
        e.str("battery")?.map(1)?.str("cells")?.u8(10)?;
        Ok(())
    })
}

fn sideboard_payload() -> StdVec<u8> {
    encode_payload(|e| {
        e.map(4)?;
        e.str("format")?.u8(1)?;
        e.str("name")?.str("attitude-sideboard")?;
        e.str("control")?.map(1)?.str("mode")?.str("articulated")?;
        // one link producing [attitude], no motors
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
    })
}

#[test]
fn mainboard_round_trip() {
    let blob = frame(&mainboard_payload());
    let cfg = parse(&blob).expect("parse mainboard");

    assert_eq!(cfg.name.as_str(), "12fet-mainboard");
    assert_eq!(cfg.control_mode, ControlMode::Rigid);
    assert_eq!(cfg.links.len(), 2);
    assert_eq!(cfg.links[0].name.as_str(), "sideA");
    assert_eq!(cfg.links[1].name.as_str(), "sideB");

    let mut attitude_only = ItemSet::empty();
    attitude_only.insert(DataItem::Attitude);
    assert_eq!(cfg.links[0].consume, attitude_only);
    assert_eq!(cfg.links[1].consume, attitude_only);

    // motor sources resolved to the matching link indices (sideA = 0, sideB = 1)
    assert_eq!(cfg.motors.len(), 2);
    assert_eq!(cfg.motors[0].attitude_source, AttitudeSource::Link(0));
    assert_eq!(cfg.motors[1].attitude_source, AttitudeSource::Link(1));
    assert_eq!(cfg.motors[0].drive_source, DriveSource::LocalBalance);
}

#[test]
fn sideboard_round_trip() {
    let blob = frame(&sideboard_payload());
    let cfg = parse(&blob).expect("parse sideboard");

    assert_eq!(cfg.name.as_str(), "attitude-sideboard");
    assert_eq!(cfg.motors.len(), 0);
    assert_eq!(cfg.links.len(), 1);
    let mut attitude_only = ItemSet::empty();
    attitude_only.insert(DataItem::Attitude);
    assert_eq!(cfg.links[0].produce, attitude_only);
    assert_eq!(cfg.links[0].name.as_str(), "up");
}

// --- fused attitude source --------------------------------------------------------------------

#[test]
fn fused_attitude_resolves_to_indices() {
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
            e.str("attitude_source")?.map(1)?
                .str("fused")?.array(2)?.str("link:sideA")?.str("link:sideB")?;
        }
        Ok(())
    });
    let cfg = parse(&frame(&payload)).expect("parse fused");
    match &cfg.motors[0].attitude_source {
        AttitudeSource::Fused(idxs) => {
            assert_eq!(idxs.as_slice(), &[0u8, 1u8]);
        }
        other => panic!("expected Fused, got {other:?}"),
    }
}

#[test]
fn unknown_top_level_key_is_skipped() {
    // A forward-compat key the firmware does not know must be ignored, not errored.
    let payload = encode_payload(|e| {
        e.map(3)?;
        e.str("name")?.str("fwd-compat")?;
        e.str("future_thing")?.array(2)?.u8(1)?.str("ignored")?;
        e.str("battery")?.map(1)?.str("cells")?.u8(7)?;
        Ok(())
    });
    let cfg = parse(&frame(&payload)).expect("parse with unknown key");
    assert_eq!(cfg.name.as_str(), "fwd-compat");
    assert_eq!(cfg.battery_cells, 7);
}

// --- BLE link (must be async) -----------------------------------------------------------------

fn ble_link_payload(arbitration: &str) -> StdVec<u8> {
    encode_payload(|e| {
        e.map(2)?;
        e.str("name")?.str("ble-board")?;
        e.str("links")?.array(1)?;
        {
            e.map(7)?;
            e.str("name")?.str("app")?;
            e.str("usart")?.str("USART2")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(9)?;
            e.str("arbitration")?.str(arbitration)?;
            e.str("device_name")?.str("Hoverboard")?;
            e.str("produce")?.array(1)?.str("telemetry")?;
        }
        Ok(())
    })
}

#[test]
fn ble_async_link_ok() {
    let cfg = parse(&frame(&ble_link_payload("async"))).expect("parse ble async");
    let app = &cfg.links[0];
    assert_eq!(app.arbitration, Arbitration::Async);
    assert_eq!(app.device_name.as_deref(), Some("Hoverboard"));
}

#[test]
fn ble_non_async_link_rejected() {
    let err = parse(&frame(&ble_link_payload("initiator"))).unwrap_err();
    assert_eq!(err, ConfigError::BleNotAsync);
}

// --- negative frame tests ---------------------------------------------------------------------

#[test]
fn bad_magic_rejected() {
    let mut blob = frame(&board_payload());
    blob[0] ^= 0xFF;
    assert_eq!(parse(&blob).unwrap_err(), ConfigError::BadMagic);
}

#[test]
fn bad_version_rejected() {
    let mut blob = frame(&board_payload());
    // version is at offset 4..6
    blob[4] = 0x99;
    assert_eq!(parse(&blob).unwrap_err(), ConfigError::BadVersion);
}

#[test]
fn truncated_length_rejected() {
    let payload = board_payload();
    let mut blob = frame(&payload);
    // Claim a longer payload than is actually present.
    let bigger = (payload.len() as u16) + 10;
    blob[6..8].copy_from_slice(&bigger.to_le_bytes());
    assert_eq!(parse(&blob).unwrap_err(), ConfigError::BadLength);
}

#[test]
fn flipped_crc_byte_rejected() {
    let mut blob = frame(&board_payload());
    // CRC is at offset 8..10.
    blob[8] ^= 0x01;
    assert_eq!(parse(&blob).unwrap_err(), ConfigError::CrcMismatch);
}

#[test]
fn blank_sector_rejected() {
    // An all-0xFF sector fails magic first.
    let blob = [0xFFu8; 64];
    assert_eq!(parse(&blob).unwrap_err(), ConfigError::BadMagic);
}

// --- negative validation tests ----------------------------------------------------------------

#[test]
fn unresolved_link_ref_rejected() {
    // attitude_source names "nope", which is not a declared link.
    let payload = encode_payload(|e| {
        e.map(3)?;
        e.str("name")?.str("bad-ref")?;
        e.str("links")?.array(1)?;
        {
            e.map(4)?;
            e.str("name")?.str("sideA")?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(2)?;
        }
        e.str("motors")?.array(1)?;
        {
            e.map(2)?;
            e.str("timer")?.str("TIMER0")?;
            e.str("attitude_source")?.str("link:nope")?;
        }
        Ok(())
    });
    assert_eq!(parse(&frame(&payload)).unwrap_err(), ConfigError::UnresolvedLinkRef);
}

#[test]
fn duplicate_link_name_rejected() {
    let payload = encode_payload(|e| {
        e.map(2)?;
        e.str("name")?.str("dup")?;
        e.str("links")?.array(2)?;
        for node in [2u8, 3u8] {
            e.map(4)?;
            e.str("name")?.str("peer")?; // same name twice
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(node)?;
        }
        Ok(())
    });
    assert_eq!(parse(&frame(&payload)).unwrap_err(), ConfigError::DuplicateLinkName);
}

#[test]
fn out_of_range_node_id_rejected() {
    let payload = encode_payload(|e| {
        e.map(2)?;
        e.str("name")?.str("bad-node")?;
        e.str("links")?.array(1)?;
        {
            e.map(4)?;
            e.str("name")?.str("peer")?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(0xFF)?; // out of 0..=0xFE
        }
        Ok(())
    });
    assert_eq!(parse(&frame(&payload)).unwrap_err(), ConfigError::BadNodeId);
}

#[test]
fn unknown_item_rejected() {
    let payload = encode_payload(|e| {
        e.map(2)?;
        e.str("name")?.str("bad-item")?;
        e.str("links")?.array(1)?;
        {
            e.map(5)?;
            e.str("name")?.str("peer")?;
            e.str("usart")?.str("USART0")?;
            e.str("baud")?.u32(115200)?;
            e.str("node_id")?.u8(2)?;
            e.str("produce")?.array(1)?.str("not_a_real_item")?;
        }
        Ok(())
    });
    assert_eq!(parse(&frame(&payload)).unwrap_err(), ConfigError::UnknownItem);
}
