//! The L2 service tying fragmentation/reassembly to one transport.
//!
//! Per `specs/l2.md` ("The service L2 offers L3"): a best-effort, atomic, one-hop packet datagram
//! service. [`Link`] is generic over a [`Transport`] that carries opaque L2 frames
//! (`[ frag-hdr ][ chunk ]`) on its wire; the same fragmentation/reassembly logic runs over every
//! transport instance (every shipped link is the SOF/len/CRC byte stream, `specs/l2.md` "one
//! framing, every link"), each with its own per-frame capacity.

use heapless::Vec;

use crate::frag::{FragHdr, MAX_FRAGMENTS, MAX_PID};
use crate::reasm::{fragment, FragError, Reassembler, MAX_PACKET};

/// Largest L2 frame (`frag-hdr` + chunk) any link emits: the format ceiling (a one-byte stream-frame
/// `len` caps the inner frame at 255). The shipped capacities are far smaller (`specs/l2.md`,
/// "Transport instances": 128/96/16).
pub const MAX_L2_FRAME: usize = 255;

/// One per-link transport that carries opaque L2 frames (`[ frag-hdr ][ chunk ]`). The shipped
/// [`SerialTransport`](crate::serial::SerialTransport) wraps each frame in the SOF/len/CRC stream
/// frame on every link; the Tier-1 tests also drive a datagram-style mock that sends the frame
/// as one transaction as-is. L2 never sees the difference.
pub trait Transport {
    /// The largest L2 frame, in bytes, this link puts in one frame: `frag-hdr` + chunk. The usable
    /// chunk is `frame_capacity() - 1`.
    fn frame_capacity(&self) -> usize;

    /// Put one L2 frame (`l2.len() <= frame_capacity()`) on the wire.
    fn send_l2_frame(&mut self, l2: &[u8]);

    /// Encode one L2 frame into `out` EXACTLY as [`send_l2_frame`](Transport::send_l2_frame) would
    /// put it on the wire, returning the encoded length, or `None` if `out` is too small or the
    /// frame is unencodable. Nothing is written to the wire.
    ///
    /// The seam a caller needs when the wire is too slow to block on: [`send_l2_frame`] is a
    /// blocking polled write, which costs ~20 ms for a 19-byte frame on the 9600-baud BLE module
    /// (`specs/ble.md`), far past the 4 ms control budget. Such a caller takes the bytes here and
    /// meters them out itself. The framing choice stays the transport's: the caller never
    /// reconstructs SOF/len/CRC for itself.
    fn encode_l2_frame(&self, l2: &[u8], out: &mut [u8]) -> Option<usize>;

    /// Pull the next received L2 frame into `out`, returning its length, or `None` if none is ready.
    fn recv_l2_frame(&mut self, out: &mut [u8]) -> Option<usize>;
}

/// One fragment's wire bytes, as [`Link::stage_fragment`] handed them back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Staged {
    /// How many bytes of the caller's `out` buffer the frame occupies.
    pub len: usize,
    /// More fragments of this packet follow: the caller owes the wire the rest of the set, in
    /// index order, before it puts anything else out.
    pub more: bool,
}

/// Reason [`Link::send`] can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendError {
    /// The packet is larger than this link can carry (more than [`MAX_FRAGMENTS`] fragments).
    PacketTooLarge,
}

/// L2 over one transport: fragments outgoing packets, reassembles incoming ones.
///
/// `N` is the reassembly buffer size - the largest packet this link reassembles. It defaults to
/// [`MAX_PACKET`] (a maximal 16-fragment UART packet, ~4 KB), but a small-MTU, single-fragment carrier
/// (the SWD mailbox: <=64-byte L3/config PDUs) sets a small `N` to keep the `Link` off a tight stack /
/// out of a tight RAM budget. `N` does not affect the send path or `mtu_hint`.
///
/// `F` is the on-stack frame scratch size used by [`Link::send`] and [`Link::poll_recv`]. It defaults
/// to [`MAX_L2_FRAME`] (the protocol ceiling), but a link whose transport advertises a smaller
/// `frame_capacity()` should set `F` to that capacity: the scratch buffers live on the deepest
/// drain/send stack chains, and on the 8 KiB-RAM parts sizing them to the protocol ceiling wastes
/// ~180 B of stack per call site over what the carrier can ever emit. `F` must be at least the
/// transport's `frame_capacity()` (asserted in [`Link::new`]).
pub struct Link<T, const N: usize = MAX_PACKET, const F: usize = MAX_L2_FRAME> {
    transport: T,
    /// The `PID` assigned to the next outgoing packet (increments per packet, wraps 0..7).
    tx_pid: u8,
    reasm: Reassembler<N>,
}

impl<T: Transport, const N: usize, const F: usize> Link<T, N, F> {
    /// Wrap a transport in an L2 link.
    pub fn new(transport: T) -> Link<T, N, F> {
        // The frame scratch must hold any frame this transport can emit; with constant capacities
        // (every shipped link) this folds to a compile-time check.
        assert!(F >= transport.frame_capacity());
        Link {
            transport,
            tx_pid: 0,
            reasm: Reassembler::new(),
        }
    }

    /// Borrow the underlying transport (for tests/inspection).
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Borrow the underlying transport mutably. The SWD mailbox uses this to reset the byte-stream
    /// framer on an epoch change (`specs/swd-mailbox.md`); the UART path uses it for re-init.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// The largest packet this link will carry: [`MAX_FRAGMENTS`] x usable-chunk. L3 can keep its
    /// packets within this where it cares (`specs/l2.md`, `mtu_hint`).
    pub fn mtu_hint(&self) -> usize {
        MAX_FRAGMENTS * (self.transport.frame_capacity() - 1)
    }

    /// Deliver one opaque packet to the peer. L2 fragments internally to the link's frame capacity;
    /// the caller never sees the MTU.
    pub fn send(&mut self, packet: &[u8]) -> Result<(), SendError> {
        let chunk_cap = self.transport.frame_capacity() - 1;
        let pid = self.tx_pid;
        let transport = &mut self.transport;
        fragment(packet, chunk_cap, pid, |hdr: FragHdr, chunk: &[u8]| {
            let mut frame: Vec<u8, F> = Vec::new();
            // Capacities are sized so these never overflow: chunk_cap <= frame_capacity - 1 <=
            // F - 1 (asserted in `Link::new`), leaving room for the frag-hdr byte.
            let _ = frame.push(hdr.encode());
            let _ = frame.extend_from_slice(chunk);
            transport.send_l2_frame(&frame);
        })
        .map_err(|e| match e {
            FragError::PacketTooLarge | FragError::ZeroChunkCap => SendError::PacketTooLarge,
        })?;
        self.tx_pid = (self.tx_pid + 1) & MAX_PID;
        Ok(())
    }

    /// Encode fragment `index` of `packet` into `out` as the complete wire bytes of ONE frame,
    /// without touching the wire, exactly as [`send`](Link::send) would have written that fragment.
    /// Returns the encoded length and whether more fragments follow, or `None` if `index` is past
    /// the packet's last fragment, the packet needs more than [`MAX_FRAGMENTS`], or `out` is too
    /// small.
    ///
    /// The non-blocking counterpart of [`send`](Link::send), for a link whose blocking write costs
    /// more than the caller's time budget (the 9600-baud BLE module: ~20 ms a frame against a 4 ms
    /// control tick). The caller meters the returned bytes onto the wire over many passes, then
    /// comes back for the next fragment once the wire has taken this one. A packet of any size
    /// [`send`](Link::send) can carry goes out this way, one frame at a time.
    ///
    /// **The caller owes the set the wire, in order.** Fragments 0..n of one packet share a PID,
    /// which is consumed when the LAST fragment is staged, so the whole set must be staged in index
    /// order with nothing else staged or sent on this link in between. Two rules follow, both the
    /// caller's to keep (`specs/link-control.md`, "Addressing and emission"):
    /// - a frame that has begun going out may not be abandoned or interrupted: the receiver's framer
    ///   reads a frame by its length byte, so anything written between two metered bytes is consumed
    ///   as this frame's body and CRC-fails it;
    /// - another packet's fragments may not be interleaved with this set's: a `FRAG_IDX` 0 arriving
    ///   mid-reassembly discards the set in progress (`crate::reasm`, atomic-or-discard).
    ///
    /// Abandoning a set part-way is safe for the LINK (the next packet's `FRAG_IDX` 0 restarts
    /// reassembly, and the PID it reuses is the one nothing completed under), but the abandoned
    /// packet is lost.
    pub fn stage_fragment(
        &mut self,
        packet: &[u8],
        index: usize,
        out: &mut [u8],
    ) -> Option<Staged> {
        let chunk_cap = self.transport.frame_capacity() - 1;
        // Reuse the fragmenter rather than re-deriving MORE/FRAG_IDX here, so the header convention
        // keeps ONE owner (`crate::frag` via `crate::reasm::fragment`): this picks the wanted
        // fragment out of the same sequence `send` would have emitted.
        let mut frame: Vec<u8, F> = Vec::new();
        let mut staged = None;
        let mut i = 0usize;
        fragment(
            packet,
            chunk_cap,
            self.tx_pid,
            |hdr: FragHdr, chunk: &[u8]| {
                if i == index {
                    // Capacities are sized so these never overflow: chunk_cap <= frame_capacity - 1
                    // <= F - 1 (asserted in `Link::new`), leaving room for the frag-hdr byte.
                    let _ = frame.push(hdr.encode());
                    let _ = frame.extend_from_slice(chunk);
                    staged = Some(hdr.more);
                }
                i += 1;
            },
        )
        .ok()?;
        let more = staged?;
        let n = self.transport.encode_l2_frame(&frame, out)?;
        if !more {
            self.tx_pid = (self.tx_pid + 1) & MAX_PID;
        }
        Some(Staged { len: n, more })
    }

    /// Return the next fully reassembled packet into `out`, or `None`. Non-blocking: it drains the
    /// transport's ready frames and feeds them through reassembly, returning the first completed
    /// packet (the reassembled bytes are copied into `out`).
    pub fn poll_recv<'a>(&mut self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let mut frame_buf = [0u8; F];
        while let Some(n) = self.transport.recv_l2_frame(&mut frame_buf) {
            if n == 0 {
                continue; // a frame with no frag-hdr cannot exist; ignore defensively
            }
            let hdr = frame_buf[0];
            let chunk = &frame_buf[1..n];
            if let Some(pkt) = self.reasm.push(hdr, chunk) {
                let len = pkt.len();
                out[..len].copy_from_slice(pkt);
                return Some(&out[..len]);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framer::{encode as encode_stream_frame, StreamFramer, MAX_STREAM_FRAME};
    use std::collections::VecDeque;
    use std::vec::Vec as StdVec;

    /// A mock 20-byte datagram link (the BLE instance): each L2 frame rides one "transaction" as-is,
    /// no SOF/len/CRC. A loopback wire feeds sends straight back to receives. It records the largest
    /// frame it ever emitted so a test can assert the BLE rule "never emit a frame > 20 B".
    struct MockDatagramLink {
        capacity: usize,
        wire: VecDeque<StdVec<u8>>,
        max_emitted: usize,
    }

    impl MockDatagramLink {
        fn new(capacity: usize) -> Self {
            MockDatagramLink {
                capacity,
                wire: VecDeque::new(),
                max_emitted: 0,
            }
        }
    }

    impl Transport for MockDatagramLink {
        fn frame_capacity(&self) -> usize {
            self.capacity
        }
        fn send_l2_frame(&mut self, l2: &[u8]) {
            // The hard invariant for the BLE link: a frame must fit one ATT transaction.
            assert!(
                l2.len() <= self.capacity,
                "datagram frame {} > capacity {}",
                l2.len(),
                self.capacity
            );
            self.max_emitted = self.max_emitted.max(l2.len());
            self.wire.push_back(l2.to_vec());
        }
        fn encode_l2_frame(&self, l2: &[u8], out: &mut [u8]) -> Option<usize> {
            // A datagram carrier puts the frame on the wire as-is, so that is what it hands back.
            if out.len() < l2.len() {
                return None;
            }
            out[..l2.len()].copy_from_slice(l2);
            Some(l2.len())
        }
        fn recv_l2_frame(&mut self, out: &mut [u8]) -> Option<usize> {
            let frame = self.wire.pop_front()?;
            out[..frame.len()].copy_from_slice(&frame);
            Some(frame.len())
        }
    }

    /// A mock byte-stream link (the inter-board UART instance): each L2 frame is wrapped in
    /// SOF/len/CRC onto a byte wire, and the receive side runs the real [`StreamFramer`] over the
    /// wire to recover frames. Loopback: sent bytes feed straight back.
    struct MockByteStreamLink {
        capacity: usize,
        wire: VecDeque<u8>,
        framer: StreamFramer,
        rx_frames: VecDeque<StdVec<u8>>,
    }

    impl MockByteStreamLink {
        fn new(capacity: usize) -> Self {
            MockByteStreamLink {
                capacity,
                wire: VecDeque::new(),
                framer: StreamFramer::new(),
                rx_frames: VecDeque::new(),
            }
        }
    }

    impl Transport for MockByteStreamLink {
        fn frame_capacity(&self) -> usize {
            self.capacity
        }
        fn send_l2_frame(&mut self, l2: &[u8]) {
            let mut out = [0u8; MAX_STREAM_FRAME];
            let n = encode_stream_frame(l2, &mut out).expect("encode stream frame");
            self.wire.extend(&out[..n]);
        }
        fn encode_l2_frame(&self, l2: &[u8], out: &mut [u8]) -> Option<usize> {
            encode_stream_frame(l2, out).ok()
        }
        fn recv_l2_frame(&mut self, out: &mut [u8]) -> Option<usize> {
            if self.rx_frames.is_empty() && !self.wire.is_empty() {
                // Drain the wire through the framer, queueing any whole frames it emits.
                let bytes: StdVec<u8> = self.wire.drain(..).collect();
                let rx = &mut self.rx_frames;
                self.framer.feed(&bytes, &mut |f| rx.push_back(f.to_vec()));
            }
            let frame = self.rx_frames.pop_front()?;
            out[..frame.len()].copy_from_slice(&frame);
            Some(frame.len())
        }
    }

    // Send `packet` and read back the next reassembled packet, asserting it round-trips.
    fn assert_round_trip<T: Transport>(link: &mut Link<T>, packet: &[u8]) {
        link.send(packet).expect("send");
        let mut out = [0u8; MAX_PACKET_TEST];
        let got = link.poll_recv(&mut out).expect("a packet");
        assert_eq!(got, packet);
    }

    const MAX_PACKET_TEST: usize = 16 * 254;

    #[test]
    fn datagram_single_frame_round_trip() {
        let mut link: Link<_> = Link::new(MockDatagramLink::new(20));
        assert_round_trip(&mut link, &[1, 2, 3, 4, 5]);
        // One fragment, one byte of overhead: a 5-byte packet -> a 6-byte frame.
        assert_eq!(link.transport().max_emitted, 6);
    }

    #[test]
    fn datagram_multi_fragment_round_trip() {
        let mut link: Link<_> = Link::new(MockDatagramLink::new(20));
        // 50 bytes over a 19-byte usable chunk -> 3 BLE transactions.
        let packet: StdVec<u8> = (0..50u8).collect();
        assert_round_trip(&mut link, &packet);
    }

    /// Stage every fragment of `packet` in order, concatenating the wire bytes, as the metering
    /// caller puts them out.
    fn stage_all<T: Transport, const F: usize>(
        link: &mut Link<T, MAX_PACKET, F>,
        packet: &[u8],
    ) -> StdVec<u8> {
        let mut wire = StdVec::new();
        let mut out = [0u8; MAX_L2_FRAME];
        let mut i = 0;
        loop {
            let s = link
                .stage_fragment(packet, i, &mut out)
                .expect("fragment stages");
            wire.extend_from_slice(&out[..s.len]);
            if !s.more {
                return wire;
            }
            i += 1;
            assert!(i < MAX_FRAGMENTS, "fragment sequence never ended");
        }
    }

    #[test]
    fn stage_returns_exactly_the_bytes_send_would_have_written() {
        // The staged-TX seam must be byte-identical to the blocking path: same frag-hdr, same
        // SOF/len/CRC framing, same PID sequence. Anything else and a metered frame would decode
        // differently from a sent one.
        let packet: StdVec<u8> = (0..14u8).collect();
        let mut sent = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        sent.send(&packet).expect("send fits one fragment");
        let wire: StdVec<u8> = sent.transport().wire.iter().copied().collect();

        let mut staged = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let mut out = [0u8; 32];
        let s = staged
            .stage_fragment(&packet, 0, &mut out)
            .expect("stages one frame");
        assert!(!s.more, "a 14 B packet is one fragment on a 15 B chunk");
        assert_eq!(&out[..s.len], &wire[..], "staged bytes == sent bytes");
    }

    #[test]
    fn a_multi_fragment_packet_stages_exactly_the_bytes_send_would_have_written() {
        // The P1 case: a packet OVER the single-fragment chunk (a 16 B PORTS reply on the BLE
        // link's 15 B chunk, and a 64 B PDU above it) still goes out, and byte-for-byte as the
        // blocking path would have written it, fragment headers and PID included.
        for len in [16usize, 19, 64] {
            let packet: StdVec<u8> = (0..len as u8).collect();
            let mut sent = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
            sent.send(&packet).expect("send");
            let wire: StdVec<u8> = sent.transport().wire.iter().copied().collect();

            let mut staged = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
            assert_eq!(stage_all(&mut staged, &packet), wire, "{len} B packet");
        }
    }

    #[test]
    fn staged_fragments_reassemble_into_the_packet() {
        // And the receiver's own view: the staged bytes fed back through the framer and the
        // reassembler yield the packet whole.
        let packet: StdVec<u8> = (0..64u8).collect();
        let mut link = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let wire = stage_all(&mut link, &packet);
        link.transport_mut().wire.extend(&wire);
        let mut out = [0u8; MAX_PACKET_TEST];
        assert_eq!(link.poll_recv(&mut out), Some(&packet[..]));
    }

    #[test]
    fn staging_consumes_one_pid_per_packet_not_per_fragment() {
        // The set shares a PID (the reassembler tears a set whose PID changes mid-way), and the
        // next packet gets the next PID. Read the PIDs back out of the frag-hdr bytes: the wire
        // frame is SOF, len, frag-hdr, ...
        let mut link = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let first = stage_all(&mut link, &(0..40u8).collect::<StdVec<u8>>());
        let second = stage_all(&mut link, &(0..40u8).collect::<StdVec<u8>>());
        let pids = |wire: &[u8]| -> StdVec<u8> {
            let mut pids = StdVec::new();
            let mut i = 0;
            while i < wire.len() {
                let body = wire[i + 1] as usize;
                pids.push(FragHdr::decode(wire[i + 2]).pid);
                i += 2 + body + 2; // SOF + len + body + CRC16
            }
            pids
        };
        assert_eq!(pids(&first), StdVec::from([0, 0, 0]), "one PID for the set");
        assert_eq!(pids(&second), StdVec::from([1, 1, 1]), "the next packet's");
    }

    #[test]
    fn staging_past_the_last_fragment_is_refused() {
        // The caller's loop terminator: `more == false` on the last fragment, and asking for one
        // past it returns None rather than an empty frame.
        let mut link = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let mut out = [0u8; 32];
        let s = link
            .stage_fragment(&[0u8; 16], 1, &mut out)
            .expect("frag 1");
        assert!(!s.more, "16 B is two fragments on a 15 B chunk");
        assert!(
            link.stage_fragment(&[0u8; 16], 2, &mut out).is_none(),
            "there is no third fragment"
        );
    }

    #[test]
    fn stage_of_a_cyclic_state_pdu_is_nineteen_bytes_on_the_ble_wire() {
        // The arithmetic the 5 Hz BLE rate is derived from (orchestrator::BLE_CYCLIC_DIVISOR):
        // an 11-byte CYCLIC_STATE payload in a 3-byte-header L3 PDU is 14 B; the BLE link's frame
        // capacity is 16, so the usable chunk is 15 and the packet is ONE fragment; the wire frame
        // is SOF + len + (frag-hdr + 14) + CRC = 19 B. 19 B at 9600 8N1 is 19.8 ms, which is why
        // the emission is metered rather than sent. It also fits one 20-byte ATT notification, so
        // the module never re-chunks it. If this number moves, the rate must be re-derived.
        const BLE_FRAME_CAP: usize = 16;
        let pdu = [0u8; 3 + 11];
        let mut link =
            Link::<_, MAX_PACKET, BLE_FRAME_CAP>::new(MockByteStreamLink::new(BLE_FRAME_CAP));
        let mut out = [0u8; 32];
        let s = link.stage_fragment(&pdu, 0, &mut out).expect("stages");
        assert!(!s.more, "one fragment: 14 B against a 15 B chunk");
        assert_eq!(s.len, 19, "CYCLIC_STATE is 19 B on the BLE wire");
        assert!(s.len <= 20, "fits one ATT notification without re-chunking");
    }

    #[test]
    fn stage_puts_nothing_on_the_wire() {
        // Staging is not sending: the transport must be untouched until the caller meters it out.
        let mut link = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let mut out = [0u8; 32];
        link.stage_fragment(&[1, 2, 3], 0, &mut out)
            .expect("stages");
        assert!(link.transport().wire.is_empty(), "nothing written");
        // The same for a fragment in the middle of a set.
        link.stage_fragment(&[0u8; 40], 1, &mut out)
            .expect("stages");
        assert!(link.transport().wire.is_empty(), "nothing written");
    }

    #[test]
    fn byte_stream_single_fragment_round_trip() {
        // Realistic UART capacity: a small packet rides one fragment (MORE=0), no fragmentation.
        let mut link = Link::new(MockByteStreamLink::new(255));
        assert_round_trip(&mut link, &[0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn byte_stream_multi_fragment_round_trip() {
        // A small byte-stream capacity forces the same fragmentation logic to split over the stream
        // link too, exercising the parameterization (identical logic, different MTU/transport).
        let mut link = Link::new(MockByteStreamLink::new(10));
        let packet: StdVec<u8> = (0..40u8).collect();
        assert_round_trip(&mut link, &packet);
    }

    #[test]
    fn ble_instance_never_emits_a_frame_over_20_bytes() {
        // The headline parameterization assertion (specs/l2.md, Tier 1): drive the BLE instance with
        // a packet that must fragment, and confirm no emitted frame ever exceeds 20 B.
        let mut link: Link<_> = Link::new(MockDatagramLink::new(20));
        let packet: StdVec<u8> = (0..(16 * 19)).map(|i| i as u8).collect(); // the max: 304 B
        link.send(&packet).expect("send");
        assert!(
            link.transport().max_emitted <= 20,
            "emitted {}",
            link.transport().max_emitted
        );
        // And it still reassembles.
        let mut out = [0u8; MAX_PACKET_TEST];
        assert_eq!(link.poll_recv(&mut out), Some(&packet[..]));
    }

    #[test]
    fn mtu_hint_reflects_capacity() {
        let ble: Link<_> = Link::new(MockDatagramLink::new(20));
        assert_eq!(ble.mtu_hint(), 16 * 19); // 304
        let uart: Link<_> = Link::new(MockByteStreamLink::new(255));
        assert_eq!(uart.mtu_hint(), 16 * 254); // 4064
    }

    #[test]
    fn pid_increments_and_wraps_across_packets() {
        // Nine single-frame packets: PID should run 0..7 then wrap to 0, each delivered cleanly.
        let mut link: Link<_> = Link::new(MockDatagramLink::new(20));
        for i in 0u8..9 {
            assert_round_trip(&mut link, &[i, i.wrapping_add(1)]);
        }
    }

    #[test]
    fn oversize_packet_rejected() {
        let mut link: Link<_> = Link::new(MockDatagramLink::new(20));
        let packet: StdVec<u8> = (0..(16 * 19 + 1)).map(|i| i as u8).collect();
        assert_eq!(link.send(&packet), Err(SendError::PacketTooLarge));
    }

    #[test]
    fn poll_recv_empty_when_no_frames() {
        let mut link: Link<_> = Link::new(MockDatagramLink::new(20));
        let mut out = [0u8; 64];
        assert_eq!(link.poll_recv(&mut out), None);
    }
}
