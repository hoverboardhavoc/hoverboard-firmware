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
pub struct Link<T, const N: usize = MAX_PACKET> {
    transport: T,
    /// The `PID` assigned to the next outgoing packet (increments per packet, wraps 0..7).
    tx_pid: u8,
    reasm: Reassembler<N>,
}

impl<T: Transport, const N: usize> Link<T, N> {
    /// Wrap a transport in an L2 link.
    pub fn new(transport: T) -> Link<T, N> {
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
            let mut frame: Vec<u8, MAX_L2_FRAME> = Vec::new();
            // Capacities are sized so these never overflow: chunk_cap <= frame_capacity - 1 <=
            // MAX_L2_FRAME - 1, leaving room for the frag-hdr byte.
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

    /// Return the next fully reassembled packet into `out`, or `None`. Non-blocking: it drains the
    /// transport's ready frames and feeds them through reassembly, returning the first completed
    /// packet (the reassembled bytes are copied into `out`).
    pub fn poll_recv<'a>(&mut self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let mut frame_buf = [0u8; MAX_L2_FRAME];
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
