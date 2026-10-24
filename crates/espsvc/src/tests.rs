extern crate std;

use std::vec::Vec;

use super::*;

fn round_trip(m: Msg) {
    let mut buf = [0u8; MAX_PDU];
    let n = m.encode(0x80, 0x00, &mut buf).unwrap();
    assert!(n <= MAX_PDU);
    let pdu = Pdu::decode(&buf[..n]).unwrap();
    assert_eq!((pdu.src, pdu.dst), (0x80, 0x00));
    assert!(is_esp_opcode(pdu.opcode));
    assert_eq!(Msg::decode(&pdu).unwrap(), m);
}

#[test]
fn every_message_round_trips_inside_one_pdu() {
    let data = [0xA5u8; CHUNK];
    let mem = [0x3Cu8; MEM_MAX];
    for m in [
        Msg::InfoReq,
        Msg::Info(EspInfo {
            version: [0, 1, 0],
            commit: 0xd31c_5bb0,
            chip: CHIP_ESP32C3,
            swd_khz: 1000,
            waits: 1,
            faults: 2,
            parity_errors: 3,
            reconnects: 4,
            mailbox: MailboxState::Up as u8,
        }),
        Msg::TargetIdReq,
        Msg::TargetId(TargetId {
            dpidr: 0x1BA0_1477,
            mcu_id: 0x1303_0410,
            cpuid: 0x412F_C231,
            family: FAMILY_F1X0,
            uid: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12],
            rdp: 0xA5,
            user: 0xFF,
            clear_check: CLEAR_PASS,
            flash_kib: 64,
            armed: true,
        }),
        Msg::ClearProtection,
        Msg::FlashBegin {
            len: 58_000,
            sha256: [7; 32],
        },
        Msg::FlashChunk {
            offset: 48 * 7,
            data: &data,
        },
        Msg::FlashChunk {
            offset: 0,
            data: &data[..5],
        },
        Msg::FlashAck { contiguous: 480 },
        Msg::FlashStatus {
            stage: Stage::Programming,
            progress: 1024,
            status: Status::Ok,
        },
        Msg::TargetCtrl(Ctrl::ResetHalt),
        Msg::MemRead {
            addr: 0x2000_0000,
            len: 56,
        },
        Msg::MemData {
            addr: 0x2000_0000,
            data: &mem,
        },
        Msg::MemWrite {
            addr: 0x2000_0100,
            data: &mem[..8],
        },
        Msg::Result {
            op: OP_TARGET_CTRL,
            status: Status::Armed,
        },
    ] {
        round_trip(m);
    }
}

#[test]
fn malformed_payloads_and_unknown_opcodes_are_refused() {
    let pdu = Pdu::new(OP_FLASH_ACK, 1, 2, &[1, 2, 3]).unwrap();
    assert_eq!(Msg::decode(&pdu), Err(DecodeError::Malformed(OP_FLASH_ACK)));
    let pdu = Pdu::new(OP_TARGET_CTRL, 1, 2, &[9]).unwrap();
    assert_eq!(
        Msg::decode(&pdu),
        Err(DecodeError::Malformed(OP_TARGET_CTRL))
    );
    let pdu = Pdu::new(0x98, 1, 2, &[]).unwrap();
    assert_eq!(Msg::decode(&pdu), Err(DecodeError::UnknownOpcode(0x98)));
    let big = [0u8; CHUNK + 1];
    assert_eq!(
        Msg::FlashChunk {
            offset: 0,
            data: &big
        }
        .encode(0, 0, &mut [0u8; 128]),
        Err(EncodeError::TooLong)
    );
}

/// A deterministic generator, so a failing schedule reproduces.
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 33) as u32
    }
}

#[test]
fn a_transfer_with_drops_duplicates_and_reordering_arrives_whole() {
    let image: Vec<u8> = (0..10_007u32).map(|i| (i * 31 + 7) as u8).collect();
    for seed in 1..20u64 {
        let mut rng = Lcg(seed);
        let mut rx = BulkRx::<16_384>::new();
        rx.begin(image.len() as u32).unwrap();
        let mut tx = BulkTx::new(image.len() as u32);
        let mut wire: Vec<(u32, usize)> = Vec::new();
        let mut acks: Vec<u32> = Vec::new();
        let mut idle = 0;
        while !tx.done() {
            while let Some(c) = tx.next_chunk() {
                match rng.next() % 10 {
                    0 => {}                               // dropped
                    1 => wire.extend_from_slice(&[c, c]), // duplicated
                    _ => wire.push(c),
                }
            }
            if wire.len() > 1 {
                let i = rng.next() as usize % wire.len();
                let j = rng.next() as usize % wire.len();
                wire.swap(i, j); // reordered
            }
            let mut progressed = false;
            for (off, n) in wire.drain(..) {
                let a = rx
                    .chunk(off, &image[off as usize..off as usize + n])
                    .unwrap();
                if !rng.next().is_multiple_of(8) {
                    acks.push(a); // an ack can be lost too
                }
            }
            for a in acks.drain(..) {
                let before = tx.acked();
                tx.ack(a);
                progressed |= tx.acked() > before;
            }
            if !progressed {
                idle += 1;
                tx.rewind();
            }
            assert!(idle < 10_000, "seed {seed} stalled");
        }
        assert!(rx.complete());
        assert_eq!(rx.data(), &image[..]);
    }
}

#[test]
fn the_receiver_refuses_misaligned_short_and_oversized_chunks() {
    let mut rx = BulkRx::<1024>::new();
    assert_eq!(rx.begin(2048), Err(BulkError::TooLarge));
    rx.begin(100).unwrap();
    assert_eq!(rx.chunk(1, &[0; 48]), Err(BulkError::BadChunk));
    assert_eq!(rx.chunk(48, &[0; 47]), Err(BulkError::BadChunk));
    assert_eq!(rx.chunk(96, &[0; 4]), Ok(0), "the tail chunk is 4 B");
    assert_eq!(rx.chunk(144, &[0; 4]), Err(BulkError::BadChunk));
    assert_eq!(rx.chunk(0, &[1; 48]), Ok(48));
    assert_eq!(rx.chunk(48, &[2; 48]), Ok(100));
    assert!(rx.complete());
}
