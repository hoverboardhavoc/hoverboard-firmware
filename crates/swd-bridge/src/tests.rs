//! Host unit tests over [`MockMemAp`], driving the bridge end (this crate) AND the firmware end
//! (`swd_mailbox`'s pointer `Mailbox`/`MailboxSerial`) over ONE shared buffer - so the SPSC mailbox is
//! exercised end to end on the host, no silicon. The bench (silicon) check is the CLI in `main.rs`.

use super::*;
use crate::walk::WalkDriver;
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
    let mut host = sh.bridge();

    // A PREVIOUS session, and the outbound it left behind. The session is what makes the stale
    // bytes reachable at all: an unattached firmware drops its emissions at the source rather than
    // fill a ring nobody reads (`specs/swd-mailbox.md`, "Backpressure", requirement 5), and
    // `init_header` zeroes `epoch` on every boot, so stale outbound presupposes an attach.
    assert_eq!(host.epoch().unwrap(), 0);
    host.attach().unwrap();
    assert_eq!(host.epoch().unwrap(), 1); // bumped
    let mut fw_serial = MailboxSerial::firmware(fw);
    fw_serial.write(&[1, 2, 3, 4]).unwrap();
    assert_eq!(host.t2h_used().unwrap(), 4);

    // This session's attach: another bump, and the previous session's outbound discarded.
    host.attach().unwrap();
    assert_eq!(host.epoch().unwrap(), 2);
    assert_eq!(host.session_epoch(), 2);
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
// Backpressure at the HOST end of the ring (`specs/swd-mailbox.md`, "Backpressure", requirement
// 6). A wedged or halted core stops advancing `h2t_tail`, so the bridge's outbound ring fills. The
// board is not at risk; the bench session is, and what it needs is the cause, not a panic.
//
// A wedged board is exercised here by simply never running a firmware consumer over the shared
// mailbox, which is what a halted core looks like from the bridge: `h2t_head` advances, `h2t_tail`
// never does.
// ---------------------------------------------------------------------------------------------

/// One L3 PDU of the size the tools actually send (a 4-byte opcode/src/dst + kind), which is 9
/// bytes once the frag-hdr and the SOF / len / CRC-16 stream header are on it.
const TOOL_PDU: [u8; 4] = [0x01, 0x80, 0x00, 0x02];

#[test]
fn a_full_h2t_is_refused_whole_and_latched_rather_than_written_in_part() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();

    // 250 of 256 bytes produced and nothing draining them: 6 bytes free, less than a 9-byte frame.
    host.produce(&std::vec![0x5Au8; 250]).unwrap();
    assert_eq!(host.h2t_used().unwrap(), 250);
    let mut serial = BridgeSerial::new(host);
    let head_before = serial.mailbox().h2t_head().unwrap();

    match serial.write(&[0xA5u8; 9]) {
        Err(BridgeError::NotDraining { used, needed }) => {
            assert_eq!((used, needed), (250, 9));
        }
        other => panic!("expected NotDraining, got {other:?}"),
    }
    assert_eq!(
        serial.mailbox().h2t_head().unwrap(),
        head_before,
        "all-or-nothing: head never advanced"
    );
    assert_eq!(
        serial.mailbox().h2t_used().unwrap(),
        250,
        "nothing produced"
    );

    // The latch hands the cause over exactly once (the caller takes it after each send).
    assert!(matches!(
        serial.take_write_error(),
        Some(BridgeError::NotDraining { .. })
    ));
    assert!(serial.take_write_error().is_none());
}

/// A [`MemAp`] that lies ONCE about `h2t_tail`, on request: the armed read (the write's free check)
/// sees a ring the firmware has drained, every later one (`produce`'s own re-read of the same two
/// words) sees the truth.
///
/// That is indistinguishable from a second host process producing into `h2t` between the check and
/// the commit, which is the single-producer premise the whole free check rests on. Nothing enforces
/// that premise (`todo.md` section 4) and this bench violated it on 2026-10-08, so the never-`Ok(0)`
/// property is not allowed to depend on it. The firmware endpoint checks its own `produce` count the
/// same way; its pointer `Mailbox` reads RAM directly and has no seam to inject this interleaving
/// into, so this is where the shape is pinned.
struct TailLiesOnce {
    inner: MockMemAp,
    armed: bool,
}

impl TailLiesOnce {
    fn arm(&mut self) {
        self.armed = true;
    }
}

impl MemAp for TailLiesOnce {
    fn read32(&mut self, addr: u32) -> Result<u32, BridgeError> {
        // The test's mailbox base is 0, so an address IS a header offset.
        if self.armed && addr == layout::H2T_TAIL as u32 {
            self.armed = false;
            return self.inner.read32(layout::H2T_HEAD as u32); // "all drained"
        }
        self.inner.read32(addr)
    }
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), BridgeError> {
        self.inner.write32(addr, val)
    }
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), BridgeError> {
        self.inner.read(addr, out)
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), BridgeError> {
        self.inner.write(addr, data)
    }
}

#[test]
fn a_short_produce_is_an_error_rather_than_an_ok_zero() {
    // The structural half of the contract: the free check can be wrong, so the count `produce`
    // returns is checked rather than assumed. With the check lied to once, `produce` places a 6-byte
    // prefix of a 9-byte frame and reports 6, and that must surface as the refusal.
    //
    // Here the refusal is the only outcome: the double lies once, so the retry `write_all` would
    // make after an `Ok(6)` meets a truthful free check and gets `NotDraining { used: 256,
    // needed: 3 }`. The mechanism the check exists for is the general one, where a producer keeps
    // taking the space: the retry's `write` then places nothing, and that `Ok(0)` is the panic.
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    // SAFETY: the backing outlives every handle (Shared owns it for the test); base 0 so addr ==
    // offset, as `Shared::bridge` does.
    let inner = unsafe { MockMemAp::new(sh.ptr(), REGION_LEN) };
    let mut host = HostMailbox::new(
        TailLiesOnce {
            inner,
            armed: false,
        },
        0,
    );
    host.attach().unwrap();
    host.produce(&std::vec![0x5Au8; 250]).unwrap(); // truthfully 6 bytes free
    let mut serial = BridgeSerial::new(host);

    serial.mailbox().mem().arm(); // the free check now sees a whole empty ring
    match serial.write(&[0xA5u8; 9]) {
        Err(BridgeError::NotDraining { used, needed }) => {
            // The POST-commit occupancy, so it matches the `h2t_used` asserted below and what an
            // operator's `mdw` of the header would read: `produce` stops only when free runs out, so
            // a refusal on the count means the ring is full.
            assert_eq!((used, needed), (256, 9));
        }
        other => panic!("expected NotDraining, got {other:?}"),
    }
    // The worst case is now a short frame in the ring, a malformed prefix the firmware's framer
    // resyncs past, instead of a panic.
    assert_eq!(serial.mailbox().h2t_used().unwrap(), 256);
    assert!(matches!(
        serial.take_write_error(),
        Some(BridgeError::NotDraining { .. })
    ));
}

#[test]
fn a_wedged_board_is_reported_by_the_walk_driver_instead_of_panicking() {
    // The behaviour requirement 6 exists for. Before this, `BridgeSerial` answered `Ok(0)` and
    // `write_all` panicked with "write() returned Ok(0)" - the least useful failure available
    // mid-session. Now the send fails with the cause, which every tool prints through its own
    // `FAIL: {e}` exit path. This test completing at all is the no-panic half.
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();
    host.produce(&std::vec![0x5Au8; 250]).unwrap(); // the wedged board's undrained inbound ring

    let mut walk = WalkDriver::new(host);
    let err = walk
        .send_pdu(&TOOL_PDU)
        .expect_err("a full h2t must surface, not be swallowed as a dropped frame");
    // The whole operator-facing line, pinned: it is what requirement 6 delivers, and it has to
    // name the cause and the numbers without a disassembler or a second tool.
    assert_eq!(
        err.to_string(),
        "the board is not draining its mailbox: 250 of 256 bytes unread in h2t, no room for this \
         9-byte frame (a halted or wedged core, or a firmware with no mailbox poll-site)"
    );
}

#[test]
fn a_drained_h2t_never_refuses() {
    // The other direction: the refusal is backlog, not a leak. With the firmware consuming each
    // frame, a hundred tool PDUs go out over the same driver and none is refused.
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();

    let mut fw_link: Link<SerialTransport<MailboxSerial>> = Link::new(SerialTransport::new(
        MailboxSerial::firmware(fw),
        FRAME_CAPACITY,
    ));
    let mut walk = WalkDriver::new(host);
    let mut out = [0u8; 512];
    for _ in 0..100 {
        walk.send_pdu(&TOOL_PDU).expect("send");
        assert_eq!(fw_link.poll_recv(&mut out), Some(&TOOL_PDU[..]));
    }
}

// ---------------------------------------------------------------------------------------------
// General config-field write path (config.rs): the registry-sourced typed-value parser, the
// shared payload encoder, and an end-to-end write -> read -> board::validate over a mock
// responder + store (proving the tool can stage BOTH a valid and an invalid layout for the
// firmware's boot validator to judge). The registry is the ONLY field-source (no second table).
// ---------------------------------------------------------------------------------------------

mod config_tests {
    use crate::config::{
        encode_config_write, parse_field_arg, parse_field_value, parse_key_arg, parse_type_name,
        parse_value_as, FieldArgError,
    };
    use base::error::FlashError;
    use board::plumbing::{read_fields, reserved_set, AllowlistPort};
    use board::{validate, BoardErrorKind, BoardField, Capabilities, Pin};
    use net::walk::{Emits, CFG_OK, MAX_PDU};
    use net::{Opcode, Pdu, Responder};
    use store::{
        Flash, Store, Type, Value, ATTITUDE_LEVEL_TRIM, IMU_MODEL, IMU_SCL_PIN, IMU_SDA_PIN,
        LED_GREEN, LED_RED, LINK_SET, MOTOR_CURRENT_LIMIT, MOTOR_HALL_A, NODE_ADDRESS,
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
        for def in &store::REGISTRY {
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
    fn key_arg_is_the_read_form_and_refuses_a_value() {
        // FIELD alone (index 0) and FIELD:INDEX, same id/index rules as the write form.
        assert_eq!(parse_key_arg("0x71").unwrap(), (0x71, 0));
        assert_eq!(parse_key_arg("0x71:2").unwrap(), (0x71, 2));
        assert_eq!(parse_key_arg("2").unwrap(), (0x02, 0));
        // A value on a read is a confusion worth refusing, not silently splitting.
        assert!(matches!(
            parse_key_arg("0x71=9000").unwrap_err(),
            FieldArgError::BadArg { .. }
        ));
        assert_eq!(
            parse_key_arg("0x77").unwrap_err(),
            FieldArgError::UnknownField(0x77)
        );
    }

    #[test]
    fn a_forced_type_overrides_the_registry_for_the_mismatch_probe() {
        // 0x71 is an I16 gain; the type-mismatch probe sends it as a U32 on purpose.
        assert_eq!(parse_type_name("U32"), Some(Type::U32));
        assert_eq!(parse_type_name("i16"), Some(Type::I16));
        assert_eq!(parse_type_name("float"), None);
        assert_eq!(
            parse_value_as(Type::U32, 0x71, "9000").unwrap(),
            Value::U32(9000)
        );
        assert_eq!(parse_field_value(0x71, "9000").unwrap(), Value::I16(9000));
        // The forced type still range-checks against ITS width.
        assert!(parse_value_as(Type::U8, 0x71, "9000").is_err());
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
    /// nothing staged is gate-capable, PA4 is the vbatt ADC channel if a layout stages it (the blank
    /// defaults no longer do), PB6/PB7 is I2C0. (Enough for the blank fleet defaults + the IMU group
    /// to validate; no motor group is staged, so
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

    /// The firmware's compiled safe-USART allowlist (specs/l3.md): PA2/PA3 (bit 1, the inter-board
    /// link on `net` slot 1), and the two BLE wirings sharing `net` slot 2 - PB10/PB11 (bit 2, the
    /// standard family) and PB6/PB7 (bit 3, the classywalk offroad family). This fixture models a
    /// STANDARD-family board, so the PB6/PB7 wiring is the one this silicon cannot route; with
    /// LINK_SET = 0b110 that port is clear as well, and either reason alone FREES PB6/PB7 so the
    /// IMU can claim it. The boot self-hold assert pin (PB12).
    const ALLOWLIST: &[AllowlistPort] = &[
        AllowlistPort {
            link_set_bit: 1,
            net_port: 1,
            pins: [0x02, 0x03],
            routable: true,
        },
        AllowlistPort {
            link_set_bit: 2,
            net_port: 2,
            pins: [0x1A, 0x1B],
            routable: true,
        },
        AllowlistPort {
            link_set_bit: 3,
            net_port: 2,
            pins: [0x16, 0x17],
            routable: false,
        },
    ];

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
        let plan = validate(&read_fields(&s), &MockChip, reserved.as_slice())
            .plan
            .expect("the staged valid layout must validate");
        let imu = plan.imu.expect("IMU group present");
        assert_eq!(
            (imu.scl.packed(), imu.sda.packed(), imu.model, imu.bus),
            (0x16, 0x17, 2, 0)
        );
    }

    #[test]
    fn stages_the_level_trim_the_attitude_filter_consumes() {
        // The per-board level trim, staged over the wire path and then run through the SAME
        // consumer the firmware builds at boot. The unit and the sign are the load-bearing part:
        // the field is centidegrees, subtracted, so a board reading +3.05 deg while level stages
        // 305 and publishes zero.
        use attitude::{Config, Mahony, Output};
        use base::fixed::Fix;

        let mut b = BoardNode::booted(0x01);
        b.stage(ATTITUDE_LEVEL_TRIM.id(), 0, "305"); // the recovered stock master value
        b.stage(ATTITUDE_LEVEL_TRIM.id(), 1, "-266"); // the slave's, on the roll index

        let mut flash = b.flash;
        let s = Store::mount(&mut flash).unwrap();
        let staged = [
            s.get(ATTITUDE_LEVEL_TRIM.at(0)),
            s.get(ATTITUDE_LEVEL_TRIM.at(1)),
        ];
        assert_eq!(
            staged,
            [305, -266],
            "indexed per axis, signed, centidegrees"
        );

        // A settled level board: the untrimmed filter publishes some standing offset, the trimmed
        // one publishes that offset less the staged trim, on each channel independently.
        let settled = |cfg: Config| {
            let mut m = Mahony::new(cfg);
            let accel = [Fix::ZERO, Fix::from_num(600), Fix::from_num(16000)];
            let mut out = Output::default();
            for _ in 0..500 {
                out = m.update([Fix::ZERO; 3], accel);
            }
            (out.pitch_deg.to_num::<f64>(), out.roll_deg.to_num::<f64>())
        };
        let (p_plain, r_plain) = settled(Config::default());
        let (p_staged, r_staged) = settled(Config::staged(staged));
        assert!(
            (p_plain - p_staged - 3.05).abs() < 1e-3,
            "pitch: {p_plain} -> {p_staged}"
        );
        assert!(
            (r_plain - r_staged + 2.66).abs() < 1e-3,
            "roll: {r_plain} -> {r_staged}"
        );

        // An unstaged board reads the registry default and is untrimmed, exactly as before the
        // field existed.
        let mut fresh = BoardNode::booted(0x02).flash;
        let s = Store::mount(&mut fresh).unwrap();
        assert_eq!(
            [
                s.get(ATTITUDE_LEVEL_TRIM.at(0)),
                s.get(ATTITUDE_LEVEL_TRIM.at(1))
            ],
            [0, 0]
        );
        assert_eq!(settled(Config::staged([0, 0])), (p_plain, r_plain));
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
        let err = validate(&read_fields(&s), &MockChip, reserved.as_slice())
            .plan
            .expect_err("the duplicate pin must be rejected");
        assert_eq!(err.field.field, BoardField::LedRed);
        match err.kind {
            BoardErrorKind::DuplicatePin(p) => assert_eq!(p.packed(), 0x13),
            other => panic!("expected DuplicatePin(PB3), got {other:?}"),
        }
    }
}

/// The attached-node resolution (`walk::resolve_attached`), the R4 fix from the 2026-07-31 arm
/// session: a bench command's destination is DERIVED, and every branch of the derivation is
/// checked here rather than at a live bench with a bridge that can arm.
#[cfg(test)]
mod attached_node {
    use crate::walk::{host_link_note, resolve_attached};
    use net::walk::{HostLink, PORT_SWD, PORT_UART};

    #[test]
    fn resolves_the_gateway_when_the_port_table_agrees() {
        // The arm-session topology: the attached master persisted 0x02, the slave holds 0x01.
        let hl = HostLink {
            node: 0x02,
            port: 0,
            kind: PORT_SWD,
        };
        assert_eq!(resolve_attached(Some(0x02), Some(hl)), Ok(0x02));
    }

    #[test]
    fn resolves_the_gateway_with_no_port_table_corroboration() {
        // A board's probe window can elapse before the host answers, so the mailbox port reports
        // empty and no host-link evidence exists. First contact still settles it.
        assert_eq!(resolve_attached(Some(0x02), None), Ok(0x02));
    }

    #[test]
    fn refuses_when_the_two_answers_disagree() {
        // Two accounts of where the host sits cannot both be true, and a tool that can ARM a
        // bridge does not get to pick one. Refuse, and say both.
        let hl = HostLink {
            node: 0x01,
            port: 1,
            kind: PORT_UART,
        };
        let err = resolve_attached(Some(0x02), Some(hl)).expect_err("ambiguous");
        assert!(err.contains("0x02"), "{err}");
        assert!(err.contains("0x01"), "{err}");
        assert!(
            err.contains("--dst"),
            "the operator is told the way out: {err}"
        );
    }

    #[test]
    fn refuses_when_the_walk_addressed_nobody() {
        assert!(resolve_attached(None, None).is_err());
    }

    #[test]
    fn the_note_names_the_medium_and_says_when_there_is_no_evidence() {
        let note = host_link_note(Some(HostLink {
            node: 0x02,
            port: 0,
            kind: PORT_SWD,
        }));
        assert!(note.contains("0x02"), "{note}");
        assert!(note.contains("SWD mailbox"), "{note}");
        assert!(host_link_note(None).contains("no port-table corroboration"));
    }
}
