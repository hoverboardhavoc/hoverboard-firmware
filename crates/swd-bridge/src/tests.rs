//! Host unit tests over [`MockMemAp`], driving the bridge end (this crate) AND the firmware end
//! (`swd_mailbox`'s pointer `Mailbox`/`MailboxSerial`) over ONE shared buffer - so the SPSC mailbox is
//! exercised end to end on the host, no silicon. The bench (silicon) check is the CLI in `main.rs`.

use super::*;
use link::{Link, SerialTransport};
use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, FRAME_CAPACITY, REGION_LEN};

/// A 4-byte-aligned shared backing for one mailbox region. The firmware `Mailbox` (pointer) and the
/// bridge `MockMemAp` (base 0) both address it.
struct Shared {
    backing: std::boxed::Box<[u32]>,
}
impl Shared {
    fn new() -> Self {
        Shared {
            backing: std::vec![0xDEAD_BEEFu32; REGION_LEN.div_ceil(4)].into_boxed_slice(),
        }
    }
    fn ptr(&mut self) -> *mut u8 {
        self.backing.as_mut_ptr() as *mut u8
    }
    fn firmware(&mut self) -> Mailbox {
        // SAFETY: the backing outlives every handle (Shared owns it for the test).
        unsafe { Mailbox::from_raw(self.ptr()) }
    }
    fn bridge(&mut self) -> HostMailbox<MockMemAp> {
        // SAFETY: as above; base 0 so addr == offset into the shared backing.
        let mem = unsafe { MockMemAp::new(self.ptr(), REGION_LEN) };
        HostMailbox::new(mem, 0)
    }
}

#[test]
fn attach_validates_bumps_epoch_and_discards_stale_outbound() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    // A stale outbound left by a previous session: the firmware produced into t2h.
    let mut fw_serial = MailboxSerial::firmware(fw);
    fw_serial.write(&[1, 2, 3, 4]).unwrap();

    let mut host = sh.bridge();
    assert_eq!(host.epoch().unwrap(), 0);
    host.attach().unwrap();
    assert_eq!(host.epoch().unwrap(), 1); // bumped
    assert_eq!(host.session_epoch(), 1);
    assert_eq!(host.t2h_used().unwrap(), 0); // stale outbound discarded (t2h_tail := t2h_head)
}

#[test]
fn attach_rejects_an_uninitialized_header() {
    let mut sh = Shared::new();
    // No init_header: magic is the 0xDEADBEEF fill.
    let mut host = sh.bridge();
    match host.attach() {
        Err(BridgeError::Invalid { .. }) => {}
        other => panic!("expected Invalid, got {other:?}"),
    }
}

#[test]
fn flush_ack_is_epoch_ack_based() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut watch = EpochWatch::new(fw);

    let mut host = sh.bridge();
    host.attach().unwrap();
    assert!(!host.flush_acked().unwrap()); // epoch_ack (0) != session_epoch (1)

    // Firmware services the epoch change: flush + (framer reset) + ack.
    assert!(watch.poll());
    watch.ack();
    assert!(host.flush_acked().unwrap()); // epoch_ack == epoch now
}

#[test]
fn bridge_produce_is_drained_by_the_firmware_consumer() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();

    let head0 = host.h2t_head().unwrap();
    let n = host.produce(&[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
    assert_eq!(n, 4);
    assert_eq!(host.h2t_head().unwrap(), head0 + 4); // committed
    assert_eq!(host.h2t_used().unwrap(), 4);

    // The firmware (pointer side) drains the same ring.
    let mut fw_serial = MailboxSerial::firmware(fw);
    let mut got = [0u8; 8];
    let k = fw_serial.read(&mut got).unwrap();
    assert_eq!(&got[..k], &[0xDE, 0xAD, 0xBE, 0xEF]);
    assert_eq!(host.h2t_used().unwrap(), 0); // firmware advanced h2t_tail
}

#[test]
fn firmware_produce_is_drained_by_the_bridge_consumer() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();

    // The firmware produces into t2h; the bridge consumes it.
    let mut fw_serial = MailboxSerial::firmware(fw);
    fw_serial.write(&[0x11, 0x22, 0x33]).unwrap();
    let mut dst = [0u8; 8];
    let k = host.consume(&mut dst).unwrap();
    assert_eq!(&dst[..k], &[0x11, 0x22, 0x33]);
    assert_eq!(host.t2h_used().unwrap(), 0);
}

#[test]
fn produce_wraps_the_ring_correctly() {
    // Drive h2t_head/tail near the cap boundary, then a straddling produce, and confirm the firmware
    // reads the bytes back in order.
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();
    let mut fw_serial = MailboxSerial::firmware(fw);

    // Move both indices to 250 (near RING_CAP=256) by producing+draining 250 bytes.
    let filler = std::vec![0u8; 250];
    host.produce(&filler).unwrap();
    let mut sink = [0u8; 250];
    let mut got = 0;
    while got < 250 {
        got += fw_serial.read(&mut sink[got..]).unwrap();
    }
    assert_eq!(host.h2t_head().unwrap(), 250);

    // Now a 12-byte produce straddles 250..256 then wraps to 0..6.
    let payload: Vec<u8> = (100..112u8).collect();
    assert_eq!(host.produce(&payload).unwrap(), 12);
    let mut out = [0u8; 12];
    let mut k = 0;
    while k < 12 {
        k += fw_serial.read(&mut out[k..]).unwrap();
    }
    assert_eq!(&out[..], &payload[..]);
}

#[test]
fn l2_frame_round_trips_bridge_to_firmware_over_serialtransport() {
    // The full transport: link::SerialTransport over the bridge serial <-> over the firmware serial,
    // one shared SPSC mailbox. A whole L2 frame round-trips both directions.
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();

    let mut fw_link: Link<SerialTransport<MailboxSerial>> = Link::new(SerialTransport::new(
        MailboxSerial::firmware(fw),
        FRAME_CAPACITY,
    ));
    let mut watch = EpochWatch::new(fw);

    // Attach + the epoch handshake (firmware flushes h2t, resets its framer, acks).
    let mut host = sh.bridge();
    host.attach().unwrap();
    assert!(watch.poll());
    fw_link.transport_mut().reset();
    watch.ack();
    assert!(host.flush_acked().unwrap());

    let mut bridge_link: Link<SerialTransport<BridgeSerial<MockMemAp>>> = Link::new(
        SerialTransport::new(BridgeSerial::new(host), FRAME_CAPACITY),
    );

    // bridge -> firmware
    let req = [0x01u8, 0x80, 0x00, 0xDE, 0xAD];
    bridge_link.send(&req).expect("bridge send");
    let mut out = [0u8; 512];
    assert_eq!(fw_link.poll_recv(&mut out), Some(&req[..]));

    // firmware -> bridge
    let resp = [0x07u8, 0x01, 0x80, 0x00];
    fw_link.send(&resp).expect("firmware send");
    let mut out2 = [0u8; 512];
    assert_eq!(bridge_link.poll_recv(&mut out2), Some(&resp[..]));
}

// ---------------------------------------------------------------------------------------------
// General config-field write path (config.rs): the registry-sourced typed-value parser, the
// shared payload encoder, and an end-to-end write -> read -> board::validate over a mock
// responder + store (proving the tool can stage BOTH a valid and an invalid layout for the
// firmware's boot validator to judge). The registry is the ONLY field-source (no second table).
// ---------------------------------------------------------------------------------------------

mod config_tests {
    use crate::config::{encode_config_write, parse_field_arg, parse_field_value, FieldArgError};
    use base::error::FlashError;
    use board::plumbing::{read_fields, reserved_set, AllowlistPort};
    use board::{validate, BoardErrorKind, BoardField, Capabilities, Pin};
    use net::walk::{Emits, CFG_OK, MAX_PDU};
    use net::{Opcode, Pdu, Responder};
    use store::{
        Flash, Store, Type, Value, IMU_MODEL, IMU_SCL_PIN, IMU_SDA_PIN, LED_GREEN, LED_RED,
        LINK_SET, MOTOR_CURRENT_LIMIT, MOTOR_HALL_A, NODE_ADDRESS,
    };

    // --- the registry-sourced parser -------------------------------------------------------

    #[test]
    fn parse_scalar_and_packed_pin_by_registry_type() {
        // A scalar (u32 tunable): decimal.
        let v = parse_field_value(MOTOR_CURRENT_LIMIT.id(), "22750").unwrap();
        assert_eq!(v, Value::U32(22750));
        assert_eq!(v.kind(), Type::U32, "the type came from the registry");
        // A packed port|pin byte (u8 board-layout field): hex round-trips exactly (0x16 = PB6).
        let p = parse_field_value(IMU_SCL_PIN.id(), "0x16").unwrap();
        assert_eq!(p, Value::U8(0x16));
        // ...and decimal for the same u8 field.
        assert_eq!(
            parse_field_value(IMU_SCL_PIN.id(), "22").unwrap(),
            Value::U8(22)
        );
    }

    #[test]
    fn parse_is_type_checked_against_the_registry() {
        // A non-numeric into a u8 field is rejected (honoring the field's registered type), NOT
        // silently accepted or coerced.
        let e = parse_field_value(IMU_SCL_PIN.id(), "PB6").unwrap_err();
        assert!(matches!(e, FieldArgError::BadValue { .. }), "{e:?}");
        // Out of range for the type is rejected too (256 into a u8).
        let e = parse_field_value(IMU_SCL_PIN.id(), "256").unwrap_err();
        assert!(matches!(e, FieldArgError::BadValue { .. }), "{e:?}");
        // An unknown field id is rejected by the registry lookup (no second table to drift).
        assert_eq!(
            parse_field_value(0x77, "1").unwrap_err(),
            FieldArgError::UnknownField(0x77)
        );
    }

    #[test]
    fn registry_is_the_single_field_source() {
        // Every scalar/bool field the registry declares parses a type-appropriate value whose kind
        // matches the registry's kind exactly -- the parser reads the type from `store::lookup`,
        // never a duplicate table here.
        for def in store::registry() {
            let raw = match def.kind {
                Type::U8 | Type::U16 | Type::U32 | Type::U64 => "1",
                Type::I16 | Type::I32 | Type::I64 => "-1",
                Type::Bool => "true",
                Type::Str => "x",
                Type::Blob => continue, // not writable via this CLI (asserted below)
            };
            let v = parse_field_value(def.field_id, raw).unwrap();
            assert_eq!(v.kind(), def.kind, "field {:#04x}", def.field_id);
        }
        // A Blob field is explicitly unsupported (no board-layout field is a blob).
        let e = parse_field_value(store::SOME_BLOB.id(), "00").unwrap_err();
        assert!(matches!(e, FieldArgError::UnsupportedType { .. }), "{e:?}");
    }

    #[test]
    fn field_arg_splits_field_index_value() {
        // FIELD=VALUE (index defaults to 0).
        assert_eq!(parse_field_arg("0x48=0x16").unwrap(), (0x48, 0, "0x16"));
        // FIELD:INDEX=VALUE (a per-motor field on motor 1).
        assert_eq!(
            parse_field_arg(&format!("{:#04x}:1=0x2A", MOTOR_HALL_A.id())).unwrap(),
            (MOTOR_HALL_A.id(), 1, "0x2A")
        );
        // Decimal field id works too.
        assert_eq!(parse_field_arg("2=6").unwrap(), (0x02, 0, "6"));
        // A missing '=' or an unknown field id fails at the arg layer.
        assert!(matches!(
            parse_field_arg("0x48").unwrap_err(),
            FieldArgError::BadArg { .. }
        ));
        assert_eq!(
            parse_field_arg("0x77=1").unwrap_err(),
            FieldArgError::UnknownField(0x77)
        );
    }

    #[test]
    fn encode_config_write_layout() {
        // The wire payload is [field_id, index, type_tag, value_le...]; a u8 pin is one value byte.
        let key = IMU_SCL_PIN.at(0).key();
        let p = encode_config_write(key, &Value::U8(0x16));
        assert_eq!(p, vec![IMU_SCL_PIN.id(), 0, Type::U8.tag(), 0x16]);
        // A u32 tunable encodes little-endian after the 3 header bytes.
        let p = encode_config_write(MOTOR_CURRENT_LIMIT.key(), &Value::U32(22750));
        assert_eq!(&p[..3], &[MOTOR_CURRENT_LIMIT.id(), 0, Type::U32.tag()]);
        assert_eq!(&p[3..], &22750u32.to_le_bytes());
    }

    // --- end-to-end: config-write into a live responder+store, read back, then validate ----

    /// A minimal in-RAM [`Flash`] for a board's store (the net walk-tests pattern; the store's own
    /// `MockFlash` is crate-internal).
    struct TestFlash {
        page_size: usize,
        bytes: std::vec::Vec<u8>,
    }
    impl TestFlash {
        fn erased() -> Self {
            TestFlash {
                page_size: 1024,
                bytes: std::vec![0xFFu8; 2 * 1024],
            }
        }
    }
    impl Flash for TestFlash {
        fn page_size(&self) -> usize {
            self.page_size
        }
        fn as_bytes(&self) -> &[u8] {
            &self.bytes
        }
        fn erase_page(&mut self, page: usize) -> Result<(), FlashError> {
            let (s, e) = (
                page * self.page_size,
                page * self.page_size + self.page_size,
            );
            self.bytes
                .get_mut(s..e)
                .ok_or(FlashError::OutOfBounds)?
                .fill(0xFF);
            Ok(())
        }
        fn program(&mut self, off: usize, data: &[u8]) -> Result<(), FlashError> {
            if !off.is_multiple_of(2) || !data.len().is_multiple_of(2) {
                return Err(FlashError::Misaligned);
            }
            let dst = self
                .bytes
                .get_mut(off..off + data.len())
                .ok_or(FlashError::OutOfBounds)?;
            for (d, &b) in dst.iter_mut().zip(data) {
                if *d != 0xFF && b != *d {
                    return Err(FlashError::ProgramFailed);
                }
                *d = b;
            }
            Ok(())
        }
    }

    /// A permissive mock chip for the staged benign + standard-family-IMU layout: every pin exists,
    /// nothing staged is gate-capable, PA4 is the vbatt ADC channel, PB6/PB7 is I2C0. (Enough for
    /// the blank fleet defaults + the IMU group to validate; no motor group is staged, so
    /// `gate_set` is never reached.)
    struct MockChip;
    impl Capabilities for MockChip {
        fn pin_exists(&self, _pin: Pin) -> bool {
            true
        }
        fn gate_capable(&self, _pin: Pin) -> bool {
            false
        }
        fn gate_set(&self, _hi: [Pin; 3], _lo: [Pin; 3]) -> Option<u8> {
            None
        }
        fn adc_channel(&self, pin: Pin) -> Option<u8> {
            (pin.packed() == 0x04).then_some(4) // PA4 = vbatt = channel 4
        }
        fn i2c_pair(&self, scl: Pin, sda: Pin) -> Option<u8> {
            ((scl.packed(), sda.packed()) == (0x16, 0x17)).then_some(0) // PB6/PB7 = I2C0
        }
    }

    /// The firmware's compiled safe-USART allowlist (specs/l3.md): PA2/PA3 (bit 1), PB10/PB11
    /// (bit 2), PB6/PB7 (bit 3). With LINK_SET = 0b110 the PB6/PB7 port (bit 3, clear) is FREED so
    /// the IMU can claim it. The boot self-hold assert pin (PB12).
    const ALLOWLIST: &[AllowlistPort] = &[
        AllowlistPort {
            link_set_bit: 1,
            pins: [0x02, 0x03],
        },
        AllowlistPort {
            link_set_bit: 2,
            pins: [0x1A, 0x1B],
        },
        AllowlistPort {
            link_set_bit: 3,
            pins: [0x16, 0x17],
        },
    ];
    const BOOT_SELF_HOLD: Option<u8> = Some(0x1C);

    /// A single board (responder + store) the config path drives; preassigned an address so it
    /// processes CONFIG_* addressed to it (the walk assigns this on silicon).
    struct BoardNode {
        resp: Responder,
        flash: TestFlash,
        addr: u8,
    }
    impl BoardNode {
        fn booted(addr: u8) -> Self {
            let mut flash = TestFlash::erased();
            {
                let mut s = Store::mount(&mut flash).unwrap();
                s.set_value(NODE_ADDRESS.key(), Value::U8(addr)).unwrap();
            }
            let mut resp = Responder::new(1, [0u8; 4], /*mcu*/ 2, /*fw*/ 0x0001);
            {
                let s = Store::mount(&mut flash).unwrap();
                resp.restore_addr(&s);
            }
            BoardNode { resp, flash, addr }
        }

        /// Ingest one CONFIG_* PDU (controller 0x80 -> this board) and return the CONFIG_RESP
        /// payload the responder emitted: `[field_id, index, status, type_tag, value...]`.
        fn config(&mut self, op: Opcode, payload: &[u8]) -> std::vec::Vec<u8> {
            let pdu = Pdu::from_op(op, 0x80, self.addr, payload);
            let mut buf = [0u8; MAX_PDU];
            let n = pdu.encode(&mut buf).unwrap();
            let mut store = Store::mount(&mut self.flash).unwrap();
            let mut emits = Emits::new();
            self.resp.ingest(0, &buf[..n], &mut store, &mut emits);
            let e = emits.iter().find(|e| {
                Pdu::decode(&e.bytes)
                    .map(|p| p.known() == Some(Opcode::ConfigResp))
                    .unwrap_or(false)
            });
            Pdu::decode(&e.expect("a CONFIG_RESP emission").bytes)
                .unwrap()
                .payload
                .to_vec()
        }

        /// Stage one field through the tool's parse + encode + the CONFIG_WRITE wire path, then
        /// read it back; assert the write status is OK and the readback equals what was written.
        fn stage(&mut self, field_id: u8, index: u8, raw: &str) {
            let value = parse_field_value(field_id, raw).unwrap();
            let key = store::Key { field_id, index };
            let w = self.config(Opcode::ConfigWrite, &encode_config_write(key, &value));
            assert_eq!(w[2], CFG_OK, "write {field_id:#04x} status");
            let r = self.config(Opcode::ConfigRead, &[field_id, index]);
            assert_eq!(r[2], CFG_OK, "read {field_id:#04x} status");
            let kind = Type::from_tag(r[3]).unwrap();
            assert_eq!(
                Value::decode(kind, &r[4..]),
                Some(value),
                "readback {field_id:#04x}"
            );
        }
    }

    #[test]
    fn stages_a_valid_layout_the_firmware_validator_accepts() {
        // The silicon-queue section-6 VALID case: free the PB6/PB7 port in LINK_SET, then write the
        // standard-family IMU group. Each write is confirmed by a read-back over the wire path.
        let mut b = BoardNode::booted(0x01);
        b.stage(LINK_SET.id(), 0, "0x06"); // bits 1+2 live; PB6/PB7 (bit 3) freed
        b.stage(IMU_SCL_PIN.id(), 0, "0x16"); // PB6
        b.stage(IMU_SDA_PIN.id(), 0, "0x17"); // PB7
        b.stage(IMU_MODEL.id(), 0, "2");

        // Now run the SAME validator the firmware runs at boot over the staged store.
        let mut flash = b.flash;
        let s = Store::mount(&mut flash).unwrap();
        let link_set: u8 = s.get(LINK_SET);
        assert_eq!(link_set, 0x06);
        let reserved = reserved_set(ALLOWLIST, link_set);
        let plan = validate(
            &read_fields(&s),
            &MockChip,
            reserved.as_slice(),
            BOOT_SELF_HOLD,
        )
        .expect("the staged valid layout must validate");
        let imu = plan.imu.expect("IMU group present");
        assert_eq!(
            (imu.scl.packed(), imu.sda.packed(), imu.model, imu.bus),
            (0x16, 0x17, 2, 0)
        );
    }

    #[test]
    fn stages_an_invalid_layout_the_firmware_validator_rejects() {
        // The section-6 INVALID case: a DUPLICATE pin. led.red is written to led.green's default
        // pin (PB3 = 0x13), so two fields claim PB3 -- a well-typed write this tool passes through
        // (no client-side board-model check), which the firmware's boot validator then rejects.
        let mut b = BoardNode::booted(0x01);
        assert_eq!(LED_GREEN.default(), 0x13); // led.green defaults to PB3
        b.stage(LED_RED.id(), 0, "0x13"); // led.red := PB3 too -> a duplicate

        let mut flash = b.flash;
        let s = Store::mount(&mut flash).unwrap();
        let reserved = reserved_set(ALLOWLIST, s.get(LINK_SET));
        let err = validate(
            &read_fields(&s),
            &MockChip,
            reserved.as_slice(),
            BOOT_SELF_HOLD,
        )
        .expect_err("the duplicate pin must be rejected");
        assert_eq!(err.field.field, BoardField::LedRed);
        match err.kind {
            BoardErrorKind::DuplicatePin(p) => assert_eq!(p.packed(), 0x13),
            other => panic!("expected DuplicatePin(PB3), got {other:?}"),
        }
    }
}
