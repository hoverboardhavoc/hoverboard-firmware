//! Tier-1 host tests (`specs/swd-mailbox.md`, "Tier 1 - host"): the ring logic against a mock RAM
//! buffer (SPSC head/tail, wrap at `cap`, `used`/`free`, producer-writes-then-commits ordering), the
//! `MailboxSerial` carrying `l2.md` frames end to end, and the epoch flush dropping a planted stale
//! partial then a fresh frame round-tripping. HAL-free, no silicon.

use super::*;
use link::{Link, SerialTransport, SOF};
use std::boxed::Box;
use std::vec::Vec;

/// 4-byte-aligned mock RAM for one mailbox region. The `Box<[u32]>` backing outlives the returned
/// [`Mailbox`] handle (the test keeps it). `u32` backing guarantees the 4-byte alignment the header
/// words need.
struct MockRam {
    backing: Box<[u32]>,
}

impl MockRam {
    fn new() -> Self {
        let words = REGION_LEN.div_ceil(4);
        // Non-zero fill so a test that forgets `init_header` cannot accidentally pass against zeroed
        // RAM: a mailbox region is indeterminate at reset, never zeroed.
        MockRam {
            backing: std::vec![0xDEAD_BEEFu32; words].into_boxed_slice(),
        }
    }

    fn mailbox(&mut self) -> Mailbox {
        // SAFETY: `backing` is 4-byte aligned, at least REGION_LEN bytes, and outlives every handle
        // (the MockRam owns it for the rest of the test).
        unsafe { Mailbox::from_raw(self.backing.as_mut_ptr() as *mut u8) }
    }
}

// ---------------------------------------------------------------------------------------------------
// Ring logic: SPSC head/tail, wrap, used/free, producer-commit ordering.
// ---------------------------------------------------------------------------------------------------

#[test]
fn init_header_sets_abi_and_zeros_indices() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    assert_eq!(mb.magic(), MAGIC);
    assert_eq!(mb.magic(), u32::from_le_bytes(*b"MBX1"));
    assert_eq!(mb.version(), VERSION);
    assert!(mb.is_valid());
    assert_eq!(mb.epoch(), 0);
    for r in [H2T, T2H] {
        assert_eq!(mb.head(r), 0);
        assert_eq!(mb.tail(r), 0);
        assert_eq!(mb.used(r), 0);
        assert_eq!(mb.free(r), RING_CAP);
    }
}

#[test]
fn produce_then_consume_round_trips_bytes() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    let src: Vec<u8> = (0..50u8).collect();
    let n = mb.produce(H2T, &src, Commit::Compiler);
    assert_eq!(n, 50);
    assert_eq!(mb.used(H2T), 50);
    assert_eq!(mb.free(H2T), RING_CAP - 50);
    assert_eq!(mb.head(H2T), 50);
    assert_eq!(mb.tail(H2T), 0);

    let mut dst = [0u8; 64];
    let got = mb.consume(H2T, &mut dst, Commit::Compiler);
    assert_eq!(got, 50);
    assert_eq!(&dst[..50], &src[..]);
    assert_eq!(mb.used(H2T), 0);
    assert_eq!(mb.free(H2T), RING_CAP);
    assert_eq!(mb.tail(H2T), 50);
}

#[test]
fn producer_writes_payload_before_committing_head() {
    // The ordering the SPSC discipline + DMB guarantee: a reader that sees the new `head` finds the
    // bytes under it. `produce` writes every payload byte, then the barrier, then the single `head`
    // store - so when `head` reaches H+N the data at slots [H, H+N) is already there.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    let src = [0x11u8, 0x22, 0x33, 0x44];
    let head_before = mb.head(H2T);
    let n = mb.produce(H2T, &src, Commit::Compiler);
    assert_eq!(n, 4);
    // head advanced by exactly N (the lone commit), tail untouched (the producer never writes tail).
    assert_eq!(mb.head(H2T), head_before + 4);
    assert_eq!(mb.tail(H2T), 0);
    // Every committed slot under the new head already holds its payload byte.
    for (i, &b) in src.iter().enumerate() {
        assert_eq!(mb.data_byte(H2T, head_before + i as u32), b);
    }
}

#[test]
fn used_free_track_across_produce_and_consume() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    assert_eq!(mb.free(H2T), RING_CAP);
    mb.produce(H2T, &[0u8; 100], Commit::Compiler);
    assert_eq!(mb.used(H2T), 100);
    assert_eq!(mb.free(H2T), 156);
    let mut dst = [0u8; 40];
    mb.consume(H2T, &mut dst, Commit::Compiler);
    assert_eq!(mb.used(H2T), 60);
    assert_eq!(mb.free(H2T), 196);
}

#[test]
fn produce_is_bounded_by_free_space() {
    // The producer never overwrites unconsumed data: it writes at most `free` bytes and reports a
    // short (or zero) count. With nothing consumed, the ring fills to exactly `cap` and then refuses.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    let big = std::vec![0xA5u8; (RING_CAP as usize) + 10];
    let n = mb.produce(H2T, &big, Commit::Compiler);
    assert_eq!(n, RING_CAP as usize); // filled exactly cap (free-running counters use the whole ring)
    assert_eq!(mb.used(H2T), RING_CAP);
    assert_eq!(mb.free(H2T), 0);
    // Full ring: a further produce writes nothing.
    assert_eq!(mb.produce(H2T, &[1, 2, 3], Commit::Compiler), 0);
    assert_eq!(mb.used(H2T), RING_CAP);
}

#[test]
fn consume_is_bounded_by_used() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    mb.produce(H2T, &[1, 2, 3], Commit::Compiler);
    let mut dst = [0u8; 32];
    assert_eq!(mb.consume(H2T, &mut dst, Commit::Compiler), 3);
    assert_eq!(&dst[..3], &[1, 2, 3]);
    // Nothing left: a further consume reads nothing.
    assert_eq!(mb.consume(H2T, &mut dst, Commit::Compiler), 0);
}

#[test]
fn data_wraps_at_cap_with_free_running_counters() {
    // Drive head/tail far past `cap` so the slot index wraps the buffer repeatedly; the bytes must
    // still round-trip, proving `slot = index & (cap - 1)` and the wrapping `u32` arithmetic.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    let cap = RING_CAP as usize;
    let mut expected_total: u64 = 0;
    let mut next: u8 = 0;
    // 10 full ring-sized passes: each fills the ring, then drains it, advancing the free-running
    // counters to 10*cap (well past a single wrap) while slots cycle the 256-byte buffer.
    for _ in 0..10 {
        let chunk: Vec<u8> = (0..cap)
            .map(|_| {
                let v = next;
                next = next.wrapping_add(1);
                v
            })
            .collect();
        assert_eq!(mb.produce(T2H, &chunk, Commit::Compiler), cap);
        let mut dst = std::vec![0u8; cap];
        assert_eq!(mb.consume(T2H, &mut dst, Commit::Compiler), cap);
        assert_eq!(dst, chunk);
        expected_total += cap as u64;
    }
    // The free-running counters advanced past several wraps and stayed equal (ring empty).
    assert_eq!(mb.head(T2H) as u64, expected_total % (1u64 << 32));
    assert_eq!(mb.head(T2H), mb.tail(T2H));
    assert_eq!(mb.used(T2H), 0);
}

#[test]
fn partial_wrap_preserves_byte_order() {
    // Offset the ring so a single produce straddles the cap boundary, then read it back in order.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    // Advance head/tail to near the end of the buffer (240 of 256) without wrapping yet.
    mb.produce(H2T, &[0u8; 240], Commit::Compiler);
    let mut sink = [0u8; 240];
    mb.consume(H2T, &mut sink, Commit::Compiler);
    assert_eq!(mb.head(H2T), 240);

    // Now produce 32 bytes: slots 240..256 then wrap to 0..16.
    let payload: Vec<u8> = (100..132u8).collect();
    assert_eq!(mb.produce(H2T, &payload, Commit::Compiler), 32);
    // The straddling bytes are at the expected wrapped slots.
    assert_eq!(mb.data_byte(H2T, 248), payload[8]); // slot 248
    assert_eq!(mb.data_byte(H2T, 256), payload[16]); // slot 0 after wrap
    let mut dst = [0u8; 32];
    assert_eq!(mb.consume(H2T, &mut dst, Commit::Compiler), 32);
    assert_eq!(&dst[..], &payload[..]);
}

// ---------------------------------------------------------------------------------------------------
// MailboxSerial carrying l2.md frames end to end (firmware <-> bridge over the rings).
// ---------------------------------------------------------------------------------------------------

const RECV_BUF: usize = 512;

fn firmware_link(mb: Mailbox) -> Link<SerialTransport<MailboxSerial>> {
    Link::new(SerialTransport::new(
        MailboxSerial::firmware(mb),
        FRAME_CAPACITY,
    ))
}
fn bridge_link(mb: Mailbox) -> Link<SerialTransport<MailboxSerial>> {
    Link::new(SerialTransport::new(
        MailboxSerial::bridge(mb),
        FRAME_CAPACITY,
    ))
}

#[test]
fn l2_frame_round_trips_both_directions() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    let mut fw = firmware_link(mb);
    let mut br = bridge_link(mb);

    // bridge -> firmware (a discovery-sized L3 PDU: opcode/src/dst + payload).
    let req = [0x01u8, 0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
    br.send(&req).expect("bridge send");
    let mut out = [0u8; RECV_BUF];
    assert_eq!(fw.poll_recv(&mut out), Some(&req[..]));

    // firmware -> bridge (the reply rides the t2h ring).
    let resp = [0x07u8, 0x01, 0x80, 0x02, 0x00];
    fw.send(&resp).expect("firmware send");
    let mut out2 = [0u8; RECV_BUF];
    assert_eq!(br.poll_recv(&mut out2), Some(&resp[..]));
}

#[test]
fn many_small_frames_reuse_the_rings() {
    // Sustained config-rate traffic: many small packets in sequence, each drained before the next,
    // so the free-running counters advance well past `cap` and the rings are reused cleanly.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();

    let mut fw = firmware_link(mb);
    let mut br = bridge_link(mb);
    let mut out = [0u8; RECV_BUF];

    for i in 0u8..40 {
        let pkt = [0x30u8, 0x80, 0x01, i, i.wrapping_mul(3)];
        br.send(&pkt).expect("send");
        assert_eq!(fw.poll_recv(&mut out), Some(&pkt[..]));
    }
    // Counters advanced far past one wrap (40 frames * ~9 bytes each > cap).
    assert!(mb.head(H2T) > RING_CAP);
    assert_eq!(mb.used(H2T), 0);
}

#[test]
fn no_frame_ready_yields_none() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    let mut fw = firmware_link(mb);
    let mut out = [0u8; RECV_BUF];
    assert_eq!(fw.poll_recv(&mut out), None);
}

#[test]
fn whole_frame_fits_the_ring_at_once() {
    // The frame_capacity-below-ring invariant: a maximal mailbox L2 frame, once stream-encoded, fits
    // the 256-byte ring in one shot (no partial-write backpressure for the cooperative producer).
    assert!(FRAME_CAPACITY + STREAM_OVERHEAD <= RING_CAP as usize);

    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    let mut fw = firmware_link(mb);
    let mut br = bridge_link(mb);

    // A packet that fragments to exactly one maximal frame (chunk = frame_capacity - 1).
    let payload: Vec<u8> = (0..(FRAME_CAPACITY as u16 - 1)).map(|i| i as u8).collect();
    br.send(&payload).expect("send max single frame");
    assert!(mb.used(H2T) <= RING_CAP); // the whole encoded frame sat in the ring at once
    let mut out = [0u8; RECV_BUF];
    assert_eq!(fw.poll_recv(&mut out), Some(&payload[..]));
}

// ---------------------------------------------------------------------------------------------------
// Epoch flush: a bumped epoch drops a planted stale partial and resets the framer.
// ---------------------------------------------------------------------------------------------------

/// A stale partial frame a previous bridge left behind: a `SOF` declaring a long (len = 200) frame
/// with only a few payload bytes, so it never completes on its own and, left in the framer, would
/// absorb the next session's frame as payload.
const STALE_PARTIAL: [u8; 5] = [SOF, 200, 0x00, 0x11, 0x22];

#[test]
fn without_flush_a_stale_framer_partial_swallows_the_fresh_frame() {
    // The hazard the epoch flush exists to prevent (control case, no flush): a stale long-frame header
    // already pulled into the framer makes the framer absorb the fresh frame's bytes as payload, so
    // nothing decodes.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    let mut fw = firmware_link(mb);

    mb.produce(H2T, &STALE_PARTIAL, Commit::Compiler);
    let mut out = [0u8; RECV_BUF];
    assert_eq!(fw.poll_recv(&mut out), None); // stale pulled into the framer, no frame yet

    // A fresh, perfectly valid frame - but the framer is mid-200-byte window, so it is swallowed.
    let mut br = bridge_link(mb);
    br.send(&[0x01u8, 0xAA, 0xBB]).expect("send");
    assert_eq!(fw.poll_recv(&mut out), None); // swallowed: the flush is load-bearing
}

#[test]
fn epoch_flush_drops_stale_ring_bytes_and_acks() {
    // Stale partial still sitting in the ring (not yet consumed). Attach bumps epoch; the bridge
    // cannot drain h2t (it is the producer there), so it waits for the firmware flush ack.
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    let mut fw = firmware_link(mb);
    let mut watch = EpochWatch::new(mb);

    mb.produce(H2T, &STALE_PARTIAL, Commit::Compiler);
    assert_eq!(mb.used(H2T), STALE_PARTIAL.len() as u32);

    let bridge = Bridge::attach(mb).expect("valid header");
    assert!(!bridge.flush_acked()); // stale bytes still in h2t -> not acked

    // Firmware poll sees the new epoch: flush the inbound ring + reset the framer.
    assert!(watch.poll());
    fw.transport_mut().reset();
    assert!(bridge.flush_acked()); // h2t drained to empty
    assert_eq!(mb.used(H2T), 0);

    // A fresh frame from the new session round-trips cleanly.
    let mut br = bridge_link(bridge.mailbox());
    let payload = [0x01u8, 0xAA, 0xBB];
    br.send(&payload).expect("send");
    let mut out = [0u8; RECV_BUF];
    assert_eq!(fw.poll_recv(&mut out), Some(&payload[..]));
}

#[test]
fn epoch_flush_resets_framer_partial_then_fresh_frame_round_trips() {
    // Stale partial already pulled into the framer; the epoch reset clears it so the fresh frame
    // decodes. This is the framer half of the flush (the ring half is the test above).
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    let mut fw = firmware_link(mb);
    let mut watch = EpochWatch::new(mb);

    mb.produce(H2T, &STALE_PARTIAL, Commit::Compiler);
    let mut out = [0u8; RECV_BUF];
    assert_eq!(fw.poll_recv(&mut out), None); // stale now buffered in the framer

    let bridge = Bridge::attach(mb).expect("valid header");
    assert!(watch.poll()); // new epoch -> flush ring (already empty) ...
    fw.transport_mut().reset(); // ... and reset the framer (drops the partial)
    assert!(bridge.flush_acked());

    let mut br = bridge_link(bridge.mailbox());
    let payload = [0x01u8, 0xCC, 0xDD];
    br.send(&payload).expect("send");
    assert_eq!(fw.poll_recv(&mut out), Some(&payload[..]));
}

#[test]
fn attach_rejects_an_invalid_header_without_writing() {
    // An uninitialized region (no init_header): magic/version garbage -> attach declines and writes
    // nothing (the bridge must not clobber a block that is not our running firmware).
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    // deliberately NOT init_header
    assert!(!mb.is_valid());
    let before_epoch = mb.epoch();
    assert!(matches!(Bridge::attach(mb), Err(AttachError::Invalid)));
    assert_eq!(mb.epoch(), before_epoch); // unchanged: no write on an invalid header
}

#[test]
fn bridge_attach_bumps_epoch_and_discards_stale_outbound() {
    let mut ram = MockRam::new();
    let mb = ram.mailbox();
    mb.init_header();
    // Stale outbound left by a previous session: the firmware produced into t2h.
    mb.produce(T2H, &[1, 2, 3, 4], Commit::Hardware);
    assert_eq!(mb.used(T2H), 4);

    let e0 = mb.epoch();
    let _bridge = Bridge::attach(mb).expect("valid");
    assert_eq!(mb.epoch(), e0 + 1); // a new session
    assert_eq!(mb.used(T2H), 0); // bridge discarded stale outbound (its own t2h_tail := t2h_head)
}
