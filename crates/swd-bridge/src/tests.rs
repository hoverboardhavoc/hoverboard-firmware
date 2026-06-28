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

    let mut fw_link: Link<_> = Link::new(SerialTransport::new(
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

    let mut bridge_link: Link<_> = Link::new(SerialTransport::new(
        BridgeSerial::new(host),
        FRAME_CAPACITY,
    ));

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
