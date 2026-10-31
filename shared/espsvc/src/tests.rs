extern crate std;

use std::vec;
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
            wifi: WifiState::Up as u8,
            ip: [192, 168, 0, 57],
            slot: 1,
            pending: true,
            node: 0x03,
            store_mounted: true,
            gdb: true,
            moe_clear_us: 137,
            bad_read: Some(BadRead {
                ack: 0b001,
                data: 0x1BA0_1476,
            }),
            no_ack: Some(0b111),
        }),
        Msg::Info(EspInfo {
            moe_clear_us: MOE_CLEAR_NONE,
            bad_read: None,
            no_ack: None,
            ..EspInfo::default()
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
        Msg::UpdateBegin {
            len: 403_744,
            sha256: [9; 32],
            version: [0, 1, 0],
        },
        Msg::UpdateChunk {
            offset: 48 * 8000,
            data: &data,
        },
        Msg::UpdateChunk {
            offset: 0,
            data: &data[..1],
        },
        Msg::UpdateAck {
            contiguous: 403_744,
        },
        Msg::UpdateConfirm,
        Msg::Result {
            op: OP_UPDATE_CHUNK,
            status: Status::BadImage,
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
    let pdu = Pdu::new(0x9C, 1, 2, &[]).unwrap();
    assert_eq!(Msg::decode(&pdu), Err(DecodeError::UnknownOpcode(0x9C)));
    let pdu = Pdu::new(OP_UPDATE_BEGIN, 1, 2, &[0; 36]).unwrap();
    assert_eq!(
        Msg::decode(&pdu),
        Err(DecodeError::Malformed(OP_UPDATE_BEGIN)),
        "a FLASH_BEGIN-sized payload is not an update begin"
    );
    let pdu = Pdu::new(OP_UPDATE_CONFIRM, 1, 2, &[0]).unwrap();
    assert_eq!(
        Msg::decode(&pdu),
        Err(DecodeError::Malformed(OP_UPDATE_CONFIRM))
    );
    let pdu = Pdu::new(OP_UPDATE_CHUNK, 1, 2, &[0; 4]).unwrap();
    assert_eq!(
        Msg::decode(&pdu),
        Err(DecodeError::Malformed(OP_UPDATE_CHUNK))
    );
    let big = [0u8; CHUNK + 1];
    assert_eq!(
        Msg::FlashChunk {
            offset: 0,
            data: &big
        }
        .encode(0, 0, &mut [0u8; 128]),
        Err(EncodeError::TooLong)
    );
    assert_eq!(
        Msg::UpdateChunk {
            offset: 0,
            data: &big
        }
        .encode(0, 0, &mut [0u8; 128]),
        Err(EncodeError::TooLong)
    );
}

#[test]
fn update_and_flash_opcodes_never_decode_as_each_other() {
    let data = [1u8; 8];
    let mut buf = [0u8; MAX_PDU];
    let n = Msg::UpdateChunk {
        offset: 0,
        data: &data,
    }
    .encode(0x80, 0, &mut buf)
    .unwrap();
    let m = Msg::decode(&Pdu::decode(&buf[..n]).unwrap()).unwrap();
    assert!(matches!(m, Msg::UpdateChunk { .. }));
    let n = Msg::FlashAck { contiguous: 48 }
        .encode(0, 0x80, &mut buf)
        .unwrap();
    let m = Msg::decode(&Pdu::decode(&buf[..n]).unwrap()).unwrap();
    assert_eq!(m, Msg::FlashAck { contiguous: 48 });
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
    for (seed, window) in (1..20u64).flat_map(|s| [(s, WINDOW_BLE), (s, WINDOW_STREAM)]) {
        let mut rng = Lcg(seed);
        let mut rx = BulkRx::<16_384>::new();
        rx.begin(image.len() as u32).unwrap();
        let mut tx = BulkTx::new(image.len() as u32, window);
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
                rx.chunk(off, &image[off as usize..off as usize + n])
                    .unwrap();
                if let Some(a) = rx.take_ack() {
                    if !rng.next().is_multiple_of(8) {
                        acks.push(a); // an ack can be lost too
                    }
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
fn erases_and_writes_at_any_offset_grow_the_transfer_with_erased_gaps() {
    let mut rx = BulkRx::<4096>::new();
    rx.erase(0, 2048).unwrap();
    rx.place(1024, &[1, 2, 3]).unwrap();
    rx.place(3000, &[9]).unwrap();
    assert_eq!(rx.len(), 3001);
    let d = rx.data();
    assert!(d[..1024].iter().all(|&b| b == 0xFF));
    assert_eq!(&d[1024..1027], &[1, 2, 3]);
    assert!(
        d[1027..3000].iter().all(|&b| b == 0xFF),
        "the gap reads erased"
    );
    assert_eq!(d[3000], 9);
    assert_eq!(rx.place(4095, &[0, 0]), Err(BulkError::TooLarge));
    rx.reset();
    rx.place(4, &[7]).unwrap();
    assert_eq!(
        rx.data(),
        &[0xFF, 0xFF, 0xFF, 0xFF, 7],
        "a reset leaves nothing behind"
    );
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

/// Run one transfer through `tx` and a lossy, duplicating, reordering wire into `rx`, returning the
/// bytes `rx` took, in the order it took them.
fn stream_transfer(seed: u64, image: &[u8], window: usize) -> Vec<u8> {
    let mut rng = Lcg(seed);
    let mut rx = StreamRx::new();
    rx.begin(image.len() as u32).unwrap();
    let mut tx = BulkTx::new(image.len() as u32, window);
    let mut taken = Vec::new();
    let mut wire: Vec<(u32, usize)> = Vec::new();
    let mut idle = 0;
    while !tx.done() {
        while let Some(c) = tx.next_chunk() {
            match rng.next() % 10 {
                0 => {}
                1 => wire.extend_from_slice(&[c, c]),
                _ => wire.push(c),
            }
        }
        if wire.len() > 1 {
            let i = rng.next() as usize % wire.len();
            let j = rng.next() as usize % wire.len();
            wire.swap(i, j);
        }
        let mut progressed = false;
        for (off, n) in wire.drain(..) {
            let data = &image[off as usize..off as usize + n];
            if rx.chunk(off, data).unwrap() {
                taken.extend_from_slice(data);
            }
            if let Some(a) = rx.take_ack() {
                if !rng.next().is_multiple_of(8) {
                    let before = tx.acked();
                    tx.ack(a);
                    progressed |= tx.acked() > before;
                }
            }
        }
        if !progressed {
            idle += 1;
            tx.rewind();
        }
        assert!(idle < 100_000, "seed {seed} stalled");
    }
    assert!(rx.complete());
    taken
}

#[test]
fn the_stream_receiver_takes_every_byte_once_in_order_through_drops_duplicates_and_reordering() {
    // Longer than any BulkRx: the bridge's own image is hundreds of KB.
    let image: Vec<u8> = (0..120_011u32).map(|i| (i * 131 + 3) as u8).collect();
    for seed in 1..8u64 {
        for window in [WINDOW_BLE, WINDOW_STREAM] {
            assert_eq!(
                stream_transfer(seed, &image, window),
                image,
                "seed {seed}, window {window}"
            );
        }
    }
}

#[test]
fn both_receivers_acknowledge_after_every_eight_chunks_and_at_the_end() {
    // 20 chunks and a 4 B tail: acknowledgements after chunks 8 and 16 and the last one.
    let len = 20 * CHUNK as u32 + 4;
    let chunk = |i: u32| (i * CHUNK as u32, if i == 20 { 4 } else { CHUNK });
    let mut bulk = BulkRx::<2048>::new();
    bulk.begin(len).unwrap();
    let mut stream = StreamRx::new();
    stream.begin(len).unwrap();
    let (mut b_acks, mut s_acks) = (Vec::new(), Vec::new());
    for i in 0..=20 {
        let (off, n) = chunk(i);
        bulk.chunk(off, &[0; CHUNK][..n]).unwrap();
        stream.chunk(off, &[0; CHUNK][..n]).unwrap();
        b_acks.extend(bulk.take_ack().map(|a| (i + 1, a)));
        s_acks.extend(stream.take_ack().map(|a| (i + 1, a)));
    }
    let want = [(8, 8 * CHUNK as u32), (16, 16 * CHUNK as u32), (21, len)];
    assert_eq!(b_acks, want);
    assert_eq!(s_acks, want);
    // Duplicates and chunks past a gap count too, and the acknowledgement stays below the gap.
    stream.begin(len).unwrap();
    stream.chunk(0, &[0; CHUNK]).unwrap();
    for _ in 0..7 {
        stream.chunk(2 * CHUNK as u32, &[0; CHUNK]).unwrap();
    }
    assert_eq!(stream.take_ack(), Some(CHUNK as u32));
    assert_eq!(stream.take_ack(), None, "nothing new since");
}

#[test]
fn the_stream_receiver_refuses_misaligned_short_and_out_of_range_chunks() {
    let mut rx = StreamRx::new();
    assert_eq!(rx.begin(0), Err(BulkError::TooLarge));
    rx.begin(100).unwrap();
    assert_eq!(rx.chunk(1, &[0; 48]), Err(BulkError::BadChunk));
    assert_eq!(rx.chunk(48, &[0; 47]), Err(BulkError::BadChunk));
    assert_eq!(rx.chunk(144, &[0; 4]), Err(BulkError::BadChunk));
    assert_eq!(rx.chunk(96, &[0; 4]), Ok(false), "past a gap: dropped");
    assert_eq!(rx.contiguous(), 0);
    assert_eq!(rx.chunk(0, &[1; 48]), Ok(true));
    assert_eq!(rx.chunk(0, &[1; 48]), Ok(false), "a duplicate");
    assert_eq!(rx.chunk(48, &[2; 48]), Ok(true));
    assert_eq!(rx.chunk(96, &[3; 4]), Ok(true));
    assert!(rx.complete());
    assert_eq!(rx.contiguous(), 100);
    rx.reset();
    assert!(rx.is_empty());
}

/// The head of an ESP32-C3 app image as `espflash save-image` writes it, with `version` in the app
/// descriptor.
fn app_head(version: &[u8]) -> Vec<u8> {
    let mut h = vec![0u8; APP_HEAD + 16];
    h[0] = 0xE9;
    h[12..14].copy_from_slice(&5u16.to_le_bytes());
    h[APP_DESC_OFFSET..APP_DESC_OFFSET + 4].copy_from_slice(&0xABCD_5432u32.to_le_bytes());
    h[APP_DESC_OFFSET + 16..APP_DESC_OFFSET + 16 + version.len()].copy_from_slice(version);
    h
}

#[test]
fn the_app_version_is_read_from_an_esp32c3_image_and_nothing_else() {
    assert_eq!(app_version(&app_head(b"0.1.0")), Some([0, 1, 0]));
    assert_eq!(app_version(&app_head(b"12.0.255-rc1")), Some([12, 0, 255]));
    assert_eq!(app_version(&app_head(b"1.2.3+41acfc6")), Some([1, 2, 3]));
    for bad in [&b"1.2"[..], b"1.2.", b"1..3", b"256.0.0", b"a.b.c", b""] {
        assert_eq!(app_version(&app_head(bad)), None, "{bad:?}");
    }
    let mut h = app_head(b"0.1.0");
    h[0] = 0xE8;
    assert_eq!(app_version(&h), None, "not an image header");
    let mut h = app_head(b"0.1.0");
    h[12] = 0; // chip id 0: an ESP32 image
    assert_eq!(app_version(&h), None, "another chip");
    let mut h = app_head(b"0.1.0");
    h[APP_DESC_OFFSET] ^= 1;
    assert_eq!(app_version(&h), None, "no app descriptor");
    assert_eq!(app_version(&app_head(b"0.1.0")[..APP_HEAD - 1]), None);
    // A board image starts with its stack pointer, not a bridge image header.
    let board = [0x00u8, 0x20, 0x00, 0x20, 0x01, 0x01, 0x00, 0x08];
    assert_eq!(app_version(&[&board[..], &[0; 120]].concat()), None);
}
