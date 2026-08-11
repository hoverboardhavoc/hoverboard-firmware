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

    /// Encode `packet` into `out` as the complete wire bytes of ONE frame, without touching the
    /// wire, and consume a PID exactly as [`send`](Link::send) would. Returns the encoded length,
    /// or `None` if the packet does not fit a single fragment or `out` is too small.
    ///
    /// The non-blocking counterpart of [`send`](Link::send), for a link whose blocking write costs
    /// more than the caller's time budget (the 9600-baud BLE module: ~20 ms a frame against a 4 ms
    /// control tick). The caller meters the returned bytes onto the wire over many passes. Single
    /// fragment only, deliberately: every packet this seam carries (an 11-byte `CYCLIC_STATE`
    /// payload in a 14-byte PDU, against a 15-byte usable chunk) is single-fragment by
    /// construction, so the caller never has to keep a multi-frame sequence contiguous.
    ///
    /// **What it does NOT do is keep the wire to itself.** These bytes are one frame, and the
    /// receiver's framer reads a frame by its length byte, so anything the caller writes between
    /// two metered bytes (including a plain [`send`](Link::send) on the same link) is read as this
    /// frame's body and CRC-fails it. Metering makes the caller the wire's scheduler: it owns
    /// finishing a staged frame before it puts anything else out (`specs/link-control.md`,
    /// "Addressing and emission").
    pub fn stage(&mut self, packet: &[u8], out: &mut [u8]) -> Option<usize> {
        let chunk_cap = self.transport.frame_capacity() - 1;
        if packet.len() > chunk_cap {
            return None; // would fragment
        }
        // Reuse the fragmenter so the single-fragment header convention keeps ONE owner (`reasm`);
        // the guard above means it emits exactly one fragment.
        let mut frame: Vec<u8, F> = Vec::new();
        fragment(
            packet,
            chunk_cap,
            self.tx_pid,
            |hdr: FragHdr, chunk: &[u8]| {
                let _ = frame.push(hdr.encode());
                let _ = frame.extend_from_slice(chunk);
            },
        )
        .ok()?;
        let n = self.transport.encode_l2_frame(&frame, out)?;
        self.tx_pid = (self.tx_pid + 1) & MAX_PID;
        Some(n)
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
        let n = staged.stage(&packet, &mut out).expect("stages one frame");
        assert_eq!(&out[..n], &wire[..], "staged bytes == sent bytes");
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
        let n = link.stage(&pdu, &mut out).expect("single fragment");
        assert_eq!(n, 19, "CYCLIC_STATE is 19 B on the BLE wire");
        assert!(n <= 20, "fits one ATT notification without re-chunking");
    }

    #[test]
    fn stage_refuses_a_packet_that_would_fragment() {
        // Single fragment only: a metered multi-fragment packet would interleave with anything
        // else the link sends. The refusal is explicit, not a silent truncation.
        let mut link = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let mut out = [0u8; 64];
        assert!(link.stage(&[0u8; 15], &mut out).is_some(), "15 B fits");
        assert!(
            link.stage(&[0u8; 16], &mut out).is_none(),
            "16 B would fragment"
        );
    }

    #[test]
    fn stage_puts_nothing_on_the_wire() {
        // Staging is not sending: the transport must be untouched until the caller meters it out.
        let mut link = Link::<_, MAX_PACKET, 16>::new(MockByteStreamLink::new(16));
        let mut out = [0u8; 32];
        link.stage(&[1, 2, 3], &mut out).expect("stages");
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
