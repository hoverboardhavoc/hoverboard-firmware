//! The bridge end (this crate) and the firmware end (`swd_mailbox`'s pointer `Mailbox` /
//! `MailboxSerial` / `EpochWatch`) over ONE shared buffer, so the SPSC mailbox is exercised end to
//! end with no silicon. `crates/swd-bridge` keeps its own suite over the same code through its
//! re-exports; this one pins the `no_std` crate on its own, with a typed backend error and the
//! injected delay.

extern crate std;

use std::boxed::Box;
use std::vec;
use std::vec::Vec;

use super::*;
use link::{Link, SerialTransport};
use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, FRAME_CAPACITY, REGION_LEN};

/// A typed backend error, as the bridge's SWD engine has (no `String` in a `no_std` crate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct OutOfRange {
    addr: u32,
    len: usize,
}

impl fmt::Display for OutOfRange {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "access {:#x}+{} past the mock region",
            self.addr, self.len
        )
    }
}

/// An in-RAM [`MemAp`] over the shared backing, base 0, so an address is an offset into it.
struct RamAp {
    ptr: *mut u8,
    len: usize,
}

impl RamAp {
    fn slice(&mut self, addr: u32, n: usize) -> Result<&mut [u8], OutOfRange> {
        let start = addr as usize;
        if start + n > self.len {
            return Err(OutOfRange { addr, len: n });
        }
        // SAFETY: bounds checked; the backing outlives the accessor (Shared owns it for the test).
        Ok(unsafe { core::slice::from_raw_parts_mut(self.ptr.add(start), n) })
    }
}

impl MemAp for RamAp {
    type Error = OutOfRange;
    fn read32(&mut self, addr: u32) -> Result<u32, OutOfRange> {
        let s = self.slice(addr, 4)?;
        Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn write32(&mut self, addr: u32, val: u32) -> Result<(), OutOfRange> {
        self.slice(addr, 4)?.copy_from_slice(&val.to_le_bytes());
        Ok(())
    }
    fn read(&mut self, addr: u32, out: &mut [u8]) -> Result<(), OutOfRange> {
        let n = out.len();
        out.copy_from_slice(self.slice(addr, n)?);
        Ok(())
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), OutOfRange> {
        self.slice(addr, data.len())?.copy_from_slice(data);
        Ok(())
    }
}

/// A 4-byte-aligned backing both ends address.
struct Shared {
    backing: Box<[u32]>,
}

impl Shared {
    fn new() -> Self {
        Shared {
            backing: vec![0xDEAD_BEEFu32; REGION_LEN.div_ceil(4)].into_boxed_slice(),
        }
    }
    fn ptr(&mut self) -> *mut u8 {
        self.backing.as_mut_ptr() as *mut u8
    }
    fn firmware(&mut self) -> Mailbox {
        // SAFETY: the backing outlives every handle.
        unsafe { Mailbox::from_raw(self.ptr()) }
    }
    fn bridge(&mut self) -> HostMailbox<RamAp> {
        HostMailbox::new(
            RamAp {
                ptr: self.ptr(),
                len: REGION_LEN,
            },
            0,
        )
    }
}

#[test]
fn an_uninitialized_header_is_refused_without_a_write() {
    let mut sh = Shared::new();
    let mut host = sh.bridge();
    match host.attach() {
        Err(BridgeError::Invalid { magic, .. }) => assert_eq!(magic, 0xDEAD_BEEF),
        other => panic!("expected Invalid, got {other:?}"),
    }
    assert_eq!(host.epoch().unwrap(), 0xDEAD_BEEF, "nothing written");
}

#[test]
fn the_flush_ack_waits_through_the_injected_delay_until_the_firmware_acks() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut watch = EpochWatch::new(fw);
    let mut host = sh.bridge();
    host.attach().unwrap();
    assert!(!host.flush_acked().unwrap());

    // The firmware observes the epoch on the third pause: the delay is the caller's, and it is
    // where the other end gets to run.
    let mut pauses = 0;
    host.wait_flush_ack(10, || {
        pauses += 1;
        if pauses == 3 && watch.poll() {
            watch.ack();
        }
    })
    .unwrap();
    assert_eq!(pauses, 3);

    // A firmware with no poll-site never acks, and the budget is a count of pauses.
    host.attach().unwrap();
    let mut n = 0;
    assert_eq!(
        host.wait_flush_ack(5, || n += 1),
        Err(BridgeError::AckTimeout)
    );
    assert_eq!(n, 5);
}

#[test]
fn a_stream_frame_round_trips_both_ways_and_across_the_ring_wrap() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut watch = EpochWatch::new(fw);
    let mut host = sh.bridge();
    host.attach().unwrap();
    assert!(watch.poll());
    watch.ack();
    host.wait_flush_ack(1, || {}).unwrap();

    let mut bridge: Link<SerialTransport<BridgeSerial<RamAp>>> = Link::new(SerialTransport::new(
        BridgeSerial::new(host),
        FRAME_CAPACITY,
    ));
    let mut board: Link<SerialTransport<MailboxSerial>> = Link::new(SerialTransport::new(
        MailboxSerial::firmware(fw),
        FRAME_CAPACITY,
    ));

    // Enough traffic to wrap both 256 B rings several times.
    for i in 0..40u8 {
        let packet: Vec<u8> = (0..(20 + i)).map(|b| b ^ i).collect();
        bridge.send(&packet).unwrap();
        let mut out = [0u8; 256];
        let got = board.poll_recv(&mut out).expect("board receives");
        assert_eq!(got, &packet[..]);

        let reply: Vec<u8> = packet.iter().rev().copied().collect();
        board.send(&reply).unwrap();
        let got = bridge.poll_recv(&mut out).expect("bridge receives");
        assert_eq!(got, &reply[..]);
    }
    assert!(bridge
        .transport_mut()
        .serial_mut()
        .take_write_error()
        .is_none());
}

#[test]
fn a_full_inbound_ring_refuses_the_whole_frame_and_latches_the_error() {
    let mut sh = Shared::new();
    sh.firmware().init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();
    // Leave 6 bytes free; nothing drains (a halted board).
    assert_eq!(host.produce(&[0x11; 250]).unwrap(), 250);
    let mut serial = BridgeSerial::new(host);
    match serial.write(&[0x5A; 9]) {
        Err(BridgeError::NotDraining { used, needed }) => {
            assert_eq!((used, needed), (250, 9));
        }
        other => panic!("expected NotDraining, got {other:?}"),
    }
    assert_eq!(serial.mailbox().h2t_used().unwrap(), 250, "nothing placed");
    assert!(matches!(
        serial.take_write_error(),
        Some(BridgeError::NotDraining { .. })
    ));
    assert!(serial.take_write_error().is_none(), "the latch empties");
}

#[test]
fn a_second_attach_and_a_second_producer_are_both_reported() {
    let mut sh = Shared::new();
    sh.firmware().init_header();
    let mut first = sh.bridge();
    first.attach().unwrap();
    let mut second = sh.bridge();
    second.attach().unwrap();
    let mut serial = BridgeSerial::new(first);
    assert_eq!(
        serial.write(&[1, 2, 3]),
        Err(BridgeError::SessionTaken { ours: 1, found: 2 })
    );

    // A writer that never attached moves h2t_head under a live session.
    let mut owner = sh.bridge();
    owner.attach().unwrap();
    let mut poker = sh.bridge();
    poker.produce(&[9, 9]).unwrap();
    let head = owner.h2t_head().unwrap();
    assert_eq!(
        owner.produce(&[1]),
        Err(BridgeError::SecondProducer {
            expected: head - 2,
            found: head
        })
    );
}

#[test]
fn a_detached_bridge_attaches_again_after_the_board_reinitialises() {
    let mut sh = Shared::new();
    let fw = sh.firmware();
    fw.init_header();
    let mut host = sh.bridge();
    host.attach().unwrap();
    assert!(host.attached());
    // The board resets: init_header zeroes the epoch, which is not ours any more.
    fw.init_header();
    assert_eq!(
        host.check_session(),
        Err(BridgeError::SessionTaken { ours: 1, found: 0 })
    );
    host.detach();
    assert!(!host.attached());
    host.attach().unwrap();
    assert_eq!(host.session_epoch(), 1);
    host.check_session().unwrap();
}

#[test]
fn a_backend_error_maps_and_displays() {
    let e: BridgeError<OutOfRange> = BridgeError::MemAp(OutOfRange { addr: 0x10, len: 4 });
    let s: BridgeError<u8> = e.clone().map_mem(|_| 7);
    assert_eq!(s, BridgeError::MemAp(7));
    let text = std::format!("{e}");
    assert_eq!(text, "MEM-AP error: access 0x10+4 past the mock region");
}
