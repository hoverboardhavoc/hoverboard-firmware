//! Host tests (`specs/esp-tool.md`, "Verification"): the engine, the board operations, flashing,
//! the mailbox session and the routing between the controller links, against the bit-level simulated debug port and the
//! GD32 model in [`crate::sim`].

extern crate std;

use std::vec;
use std::vec::Vec;

use espsvc::{Msg, Status, FAMILY_F10X, FAMILY_F1X0};
use link::{Link, SerialTransport, StreamFramer};
use net::pdu::Pdu;
use sha2::{Digest, Sha256};
use swd_mailbox::{EpochWatch, Mailbox, MailboxSerial, FRAME_CAPACITY};

use crate::flash::{self, FlashError, FlashJob, FLASH_BASE, PAGE, STORE_START};
use crate::mailbox::MailboxPort;
use crate::router::{Port, Router, NOTIFY_MAX};
use crate::sim::{board, Family, Gd32, Inject, SimPins};
use crate::svc::{Build, ImageBuffer, Service};
use crate::swd::{Counters, Swd, SwdError, XferError};
use crate::target::{self, TargetError};

type SimSwd = Swd<SimPins<Gd32>>;

fn model(s: &mut SimSwd) -> &mut Gd32 {
    &mut s.pins().bus
}

/// The engine's counters must account for every fault the port injected, one for one.
fn assert_counters_match(s: &mut SimSwd) {
    let injected = s.pins().injected;
    assert_eq!(s.counters(), injected);
}

// ---- the SWD engine ----

#[test]
fn a_clean_line_connects_and_identifies_both_families() {
    for (fam, code, uid0) in [
        (Family::F10x, FAMILY_F10X, 0x8A),
        (Family::F1x0, FAMILY_F1X0, 0x8A),
    ] {
        let mut s = board(fam, Inject::default(), 1);
        let khz = s.measure_khz(10_000);
        assert_eq!(khz, 1000, "the simulated clock is 1 us per bit");
        assert_eq!(s.connect(), Ok(0x1BA0_1477));
        let id = target::identify(&mut s).unwrap();
        assert_eq!(id.family, code);
        assert_eq!(id.mcu_id, 0x1303_0410);
        assert_eq!(id.cpuid, 0x412F_C231);
        assert_eq!(id.uid[0], uid0);
        assert_eq!((id.rdp, id.user, id.flash_kib), (0xA5, 0xFF, 64));
        assert!(!id.armed);
        assert_eq!(s.counters(), Counters::default());
    }
}

#[test]
fn every_transfer_returns_the_right_data_or_an_error_and_the_counters_match() {
    for seed in 1..6u64 {
        let inject = Inject {
            wait: 40,
            fault: 15,
            noack: 8,
            parity: 15,
            dead: false,
        };
        let mut s = board(Family::F1x0, inject, seed);
        s.connect().unwrap();
        // Spans the 1 KB auto-increment wrap at 0x2000_0400 twice.
        let base = 0x2000_0300;
        for round in 0..8u32 {
            let data: Vec<u32> = (0..400u32)
                .map(|i| i.wrapping_mul(0x9E37_79B9) ^ round)
                .collect();
            s.write_words(base, &data).unwrap();
            let mut back = vec![0u32; data.len()];
            s.read_words(base, &mut back).unwrap();
            assert_eq!(back, data, "seed {seed} round {round}");
        }
        let c = s.counters();
        assert!(c.waits > 0 && c.faults > 0 && c.parity_errors > 0 && c.reconnects > 0);
        assert_counters_match(&mut s);
    }
}

#[test]
fn unaligned_byte_runs_read_and_write_through_words() {
    let mut s = board(Family::F10x, Inject::default(), 3);
    s.connect().unwrap();
    s.write_words(0x2000_0100, &[0x1111_1111; 8]).unwrap();
    let bytes: Vec<u8> = (1..=13).collect();
    s.write_bytes(0x2000_0103, &bytes).unwrap();
    let mut back = [0u8; 17];
    s.read_bytes(0x2000_0102, &mut back).unwrap();
    assert_eq!(back[0], 0x11);
    assert_eq!(&back[1..14], &bytes[..]);
    assert_eq!(&back[14..], &[0x11; 3]);
}

#[test]
fn a_dead_line_fails_within_the_deadline() {
    let mut s = board(
        Family::F1x0,
        Inject {
            dead: true,
            ..Inject::default()
        },
        1,
    );
    assert_eq!(s.connect(), Err(SwdError::Exhausted(XferError::NoAck)));
    // With an unbounded count, the deadline alone ends it.
    let mut s = Swd::new(
        SimPins::new(
            Gd32::new(Family::F1x0),
            Inject {
                dead: true,
                ..Inject::default()
            },
            1,
        ),
        crate::swd::SwdConfig {
            op_retries: u32::MAX,
            ..Default::default()
        },
    );
    let t0 = s.now_us();
    assert!(s.connect().is_err());
    let took = s.now_us() - t0;
    assert!(took > 250_000 && took < 251_000, "took {took} us");
}

#[test]
fn a_line_that_dies_mid_session_errors_out_of_a_read() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    s.connect().unwrap();
    s.pins().inject.dead = true;
    let mut v = [0u32; 4];
    assert_eq!(
        s.read_words(0x2000_0000, &mut v),
        Err(SwdError::Exhausted(XferError::NoAck))
    );
    s.pins().inject.dead = false;
    s.forget();
    target::ensure_connected(&mut s).unwrap();
    s.read_words(0x2000_0000, &mut v).unwrap();
}

// ---- board operations ----

#[test]
fn a_halt_disarms_first_and_sets_the_debug_hold() {
    let mut s = board(Family::F10x, Inject::default(), 1);
    s.connect().unwrap();
    model(&mut s).cchp = 1 << 15 | 0x1C;
    assert!(target::armed(&mut s).unwrap());
    target::halt(&mut s).unwrap();
    let g = model(&mut s);
    assert!(g.halted);
    assert_eq!(g.cchp, 0x1C, "MOE cleared, the dead time left alone");
    assert_eq!(g.dbg_ctl0 & target::DEBUG_HOLD, target::DEBUG_HOLD);
    target::resume(&mut s).unwrap();
    assert!(!model(&mut s).halted);
}

#[test]
fn a_board_whose_disarm_does_not_take_is_not_halted() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    s.connect().unwrap();
    let g = model(&mut s);
    g.cchp = 1 << 15;
    g.moe_stuck = true;
    assert_eq!(target::halt(&mut s), Err(TargetError::Armed));
    assert_eq!(target::reset_halt(&mut s), Err(TargetError::Armed));
    assert!(!model(&mut s).halted);
}

#[test]
fn reset_and_halt_stops_at_the_reset_vector_and_release_runs() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    model(&mut s).load(&[0x00, 0x20, 0x00, 0x20, 0x09, 0x01, 0x00, 0x08]);
    s.connect().unwrap();
    let resets = model(&mut s).resets;
    target::reset_halt(&mut s).unwrap();
    assert!(model(&mut s).halted);
    assert_eq!(model(&mut s).resets, resets + 1);
    target::release(&mut s).unwrap();
    assert!(!model(&mut s).halted);
    target::reset(&mut s).unwrap();
    assert!(!model(&mut s).halted, "no vector catch left behind");
}

// ---- flashing ----

/// A test image: a vector table that looks real, then a pattern, `len` bytes.
fn image(len: usize) -> Vec<u8> {
    let mut v: Vec<u8> = (0..len as u32).map(|i| (i * 7 + (i >> 8)) as u8).collect();
    v[..8].copy_from_slice(&[0x00, 0x20, 0x00, 0x20, 0x09, 0x01, 0x00, 0x08]);
    // Some erased-value halfwords, which the programmer skips.
    if len > 110 {
        v[100..110].fill(0xFF);
    }
    v
}

fn run_job(s: &mut SimSwd, img: &[u8]) -> Result<(), FlashError> {
    let mut job = FlashJob::new(img.len()).unwrap();
    for _ in 0..10_000 {
        if let flash::Event::Finished(r) = job.step(s, img) {
            return r;
        }
    }
    panic!("the job never finished");
}

#[test]
fn an_image_across_pages_programs_verifies_and_boots_leaving_the_store_alone() {
    for fam in [Family::F1x0, Family::F10x] {
        let mut s = board(fam, Inject::default(), 1);
        let store: Vec<u16> = (0..1024u32).map(|i| i as u16 ^ 0x5A5A).collect();
        let g = model(&mut s);
        g.flash[31 * 1024..].copy_from_slice(&store);
        g.load(&[0xAB; 4]); // an old image, so the board boots and has a header
        g.power_cycle();
        assert_eq!(g.mailbox().magic(), swd_mailbox::MAGIC);
        g.cchp = 1 << 15;

        let img = image(3 * 1024 + 517); // four pages, the last partial
        run_job(&mut s, &img).unwrap();
        let g = model(&mut s);
        assert_eq!(&g.flash_bytes()[..img.len()], &img[..]);
        assert_eq!(
            &g.flash[31 * 1024..],
            &store[..],
            "the store pages are untouched"
        );
        assert_eq!(g.store_writes, 0);
        assert!(!g.halted, "released and running");
        assert_eq!(
            g.mailbox().magic(),
            swd_mailbox::MAGIC,
            "the new image wrote its header"
        );
        assert_eq!(g.cchp & 1 << 15, 0, "disarmed before the halt");
    }
}

#[test]
fn flashing_through_a_noisy_line_still_verifies() {
    let mut s = board(
        Family::F1x0,
        Inject {
            wait: 20,
            fault: 5,
            noack: 3,
            parity: 10,
            dead: false,
        },
        7,
    );
    let img = image(2 * 1024 + 100);
    run_job(&mut s, &img).unwrap();
    assert_eq!(&model(&mut s).flash_bytes()[..img.len()], &img[..]);
    assert_counters_match(&mut s);
}

#[test]
fn a_descending_program_fails_on_the_f1x0_and_not_on_the_f10x() {
    for (fam, fails) in [(Family::F1x0, true), (Family::F10x, false)] {
        let mut s = board(fam, Inject::default(), 1);
        s.connect().unwrap();
        target::halt(&mut s).unwrap();
        flash::unlock(&mut s).unwrap();
        let page = FLASH_BASE + PAGE;
        flash::erase_page(&mut s, page).unwrap();
        let img = vec![0x12u8; 2 * PAGE as usize];
        let mut result = Ok(());
        for a in (page..page + 8).step_by(2).rev() {
            result = flash::program_range(&mut s, &img, a, a + 2);
            if result.is_err() {
                break;
            }
        }
        if fails {
            assert!(
                matches!(result, Err(FlashError::Program { addr, .. }) if addr == page + 2),
                "{result:?}"
            );
        } else {
            result.unwrap();
        }
    }
}

#[test]
fn a_write_that_reaches_the_store_is_refused_before_the_board_is_touched() {
    assert!(FlashJob::new(flash::MAX_IMAGE).is_ok());
    assert!(matches!(
        FlashJob::new(flash::MAX_IMAGE + 1),
        Err(FlashError::StoreRange)
    ));
    let mut s = board(Family::F1x0, Inject::default(), 1);
    s.connect().unwrap();
    assert_eq!(
        flash::erase_page(&mut s, STORE_START),
        Err(FlashError::StoreRange)
    );
    let img = vec![0u8; 64 * 1024];
    assert_eq!(
        flash::program_range(&mut s, &img, STORE_START - 2, STORE_START + 2),
        Err(FlashError::StoreRange)
    );
    assert_eq!(model(&mut s).store_writes, 0);
}

#[test]
fn a_flash_cut_short_leaves_page_0_erased() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    model(&mut s).load(&image(4096)); // something already there
    let img = image(3 * 1024);
    let mut job = FlashJob::new(img.len()).unwrap();
    // Prepare, three erases, then two of the three page programs: power is lost here.
    for _ in 0..6 {
        assert!(matches!(job.step(&mut s, &img), flash::Event::Progress(..)));
    }
    let f = model(&mut s).flash_bytes();
    assert!(
        f[..1024].iter().all(|&b| b == 0xFF),
        "the vector table is erased"
    );
    assert_eq!(&f[1024..3072], &img[1024..3072]);
}

#[test]
fn a_protected_or_oversized_part_is_refused() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    model(&mut s).protect();
    assert_eq!(run_job(&mut s, &image(2048)), Err(FlashError::Protected));
    assert!(!model(&mut s).halted);

    let mut s = board(Family::F10x, Inject::default(), 1);
    model(&mut s).flash_kib = 256;
    assert_eq!(
        run_job(&mut s, &image(2048)),
        Err(FlashError::UnsupportedPart(256))
    );
    assert_eq!(FlashError::UnsupportedPart(256).status(), Status::TooLarge);
}

#[test]
fn clearing_protection_keeps_the_user_byte_and_takes_after_a_power_cycle() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    let g = model(&mut s);
    g.load(&image(4096));
    g.option[1] = 0x857A; // USER 0x7A (hardware watchdog off, the rest as shipped)
    g.option[2] = 0xED12; // DATA0 0x12
    g.protect();
    let before = target::identify(&mut s).unwrap();
    assert_ne!(before.rdp, 0xA5);
    let user = flash::clear_protection(&mut s).unwrap();
    assert_eq!(user, 0x7A);
    model(&mut s).power_cycle();
    s.forget();
    let after = target::identify(&mut s).unwrap();
    assert!(flash::clear_took(&after, user));
    let g = model(&mut s);
    assert_eq!(g.option[2], 0xED12, "DATA0 written back");
    assert!(
        g.flash.iter().all(|&h| h == 0xFFFF),
        "the clear mass-erased the chip"
    );
    // Then an image flashes.
    run_job(&mut s, &image(1500)).unwrap();
}

// ---- the mailbox session ----

/// The board's end of the mailbox, as the firmware runs it.
struct Firmware {
    watch: EpochWatch,
    link: Link<SerialTransport<MailboxSerial>>,
}

impl Firmware {
    fn new(mb: Mailbox) -> Self {
        Firmware {
            watch: EpochWatch::new(mb),
            link: Link::new(SerialTransport::new(
                MailboxSerial::firmware(mb),
                FRAME_CAPACITY,
            )),
        }
    }
    /// One service-loop pass: the epoch flush, then every whole packet.
    fn poll(&mut self) -> Vec<Vec<u8>> {
        if self.watch.poll() {
            self.link.transport_mut().reset();
            self.watch.ack();
        }
        let mut got = Vec::new();
        let mut buf = [0u8; 256];
        while let Some(p) = self.link.poll_recv(&mut buf) {
            got.push(p.to_vec());
        }
        got
    }
}

fn booted(family: Family, inject: Inject) -> (MailboxPort<SimPins<Gd32>>, Firmware) {
    let mut s = board(family, inject, 11);
    let g = model(&mut s);
    g.load(&image(64));
    g.power_cycle();
    let fw = Firmware::new(g.mailbox());
    (MailboxPort::new(s), fw)
}

fn attach(port: &mut MailboxPort<SimPins<Gd32>>, fw: &mut Firmware) {
    port.poll(|_| {});
    assert_eq!(port.state(), espsvc::MailboxState::Attaching);
    fw.poll();
    port.poll(|_| {});
    assert_eq!(port.state(), espsvc::MailboxState::Up);
}

#[test]
fn the_port_attaches_and_carries_frames_both_ways() {
    let (mut port, mut fw) = booted(Family::F10x, Inject::default());
    attach(&mut port, &mut fw);
    let pkt = [0x30u8, 0x80, 0x01, 0x10, 0x00];
    port.send(&pkt);
    port.poll(|_| {});
    assert_eq!(fw.poll(), vec![pkt.to_vec()]);
    fw.link.send(&[0x32, 0x01, 0x80, 0x10, 0, 0, 1, 7]).unwrap();
    let mut got = Vec::new();
    port.poll(|p| got.push(p.to_vec()));
    assert_eq!(got, vec![vec![0x32, 0x01, 0x80, 0x10, 0, 0, 1, 7]]);
}

#[test]
fn a_full_ring_keeps_frames_for_the_next_poll_and_drive_frames_are_latest_wins() {
    let (mut port, mut fw) = booted(Family::F1x0, Inject::default());
    attach(&mut port, &mut fw);
    // The board stops draining: fill h2t with config frames until the queue holds some back.
    let mut sent = 0u32;
    while port.queued() < 3 {
        port.send(&[
            0x30, 0x80, 0x01, sent as u8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ]);
        port.poll(|_| {});
        sent += 1;
        assert!(sent < 40, "the ring never filled");
    }
    let held = port.queued();
    // Three drive frames to one node collapse into one queued frame, the newest.
    for speed in 1..=3u8 {
        port.send(&[linkctl::OP_DRIVE_CMD, 0x80, 0x01, speed]);
    }
    assert_eq!(port.queued(), held + 1);
    // The board drains: everything still queued arrives, the drive frame once with speed 3.
    let mut all = Vec::new();
    for _ in 0..50 {
        all.extend(fw.poll());
        port.poll(|_| {});
    }
    let drives: Vec<_> = all
        .iter()
        .filter(|p| p[0] == linkctl::OP_DRIVE_CMD)
        .collect();
    assert_eq!(drives, vec![&vec![linkctl::OP_DRIVE_CMD, 0x80, 0x01, 3]]);
    assert_eq!(port.queued(), 0);
    assert_eq!(
        all.len() as u32,
        sent + 1,
        "every config frame and one drive frame"
    );
    assert_eq!(port.dropped, 0);
}

#[test]
fn the_port_reopens_after_a_board_reset() {
    let (mut port, mut fw) = booted(Family::F1x0, Inject::default());
    attach(&mut port, &mut fw);
    target::reset(port.swd()).unwrap(); // init_header: epoch back to 0
    let mut fw = Firmware::new(port.swd().pins().bus.mailbox());
    port.swd().pins().sleep_us(200_000);
    port.poll(|_| {});
    port.poll(|_| {});
    assert_eq!(port.state(), espsvc::MailboxState::Attaching);
    fw.poll();
    port.poll(|_| {});
    assert_eq!(port.state(), espsvc::MailboxState::Up);
    port.send(&[0x30, 0x80, 0x01, 1]);
    port.poll(|_| {});
    assert_eq!(fw.poll(), vec![vec![0x30, 0x80, 0x01, 1]]);
}

#[test]
fn a_header_from_another_build_leaves_the_port_down() {
    let (mut port, _fw) = booted(Family::F1x0, Inject::default());
    use crate::sim::Bus;
    assert!(port.swd().pins().bus.write(0x2000_0004, 4, 1)); // the header's version word
    port.poll(|_| {});
    assert_eq!(port.state(), espsvc::MailboxState::VersionMismatch);
}

// ---- the controller links ----

/// How many bytes a port hands its driver at a time: a BLE notification, a USB packet.
fn out_size(port: Port) -> usize {
    match port {
        Port::Ble => NOTIFY_MAX,
        Port::Usb => 64,
    }
}

/// The controller's end of `port`: a stream framer over what the router sends out of it.
fn received(router: &mut Router, port: Port) -> Vec<Vec<u8>> {
    let mut framer: StreamFramer = StreamFramer::new();
    let mut out = Vec::new();
    let mut n = [0u8; 64];
    let size = out_size(port);
    loop {
        let k = router.next_out(port, &mut n[..size]);
        if k == 0 {
            break;
        }
        framer.feed(&n[..k], &mut |f| out.push(f[1..].to_vec())); // drop the frag-hdr
    }
    out
}

fn stream(packets: &[Vec<u8>]) -> Vec<u8> {
    let mut s = Vec::new();
    for p in packets {
        let mut l2 = vec![0u8]; // frag-hdr: single fragment, pid 0
        l2.extend_from_slice(p);
        let mut buf = [0u8; 300];
        let n = link::encode_stream_frame(&l2, &mut buf).unwrap();
        s.extend_from_slice(&buf[..n]);
    }
    s
}

fn service() -> Service<'static> {
    let rx: &mut ImageBuffer = std::boxed::Box::leak(std::boxed::Box::new(ImageBuffer::new()));
    Service::new(
        Build {
            version: [0, 1, 0],
            commit: 0,
            swd_khz: 1000,
        },
        rx,
    )
}

fn encode(m: &Msg, src: u8) -> Vec<u8> {
    let mut buf = [0u8; espsvc::MAX_PDU];
    let n = m.encode(src, 0x00, &mut buf).unwrap();
    buf[..n].to_vec()
}

#[test]
fn a_stream_split_anywhere_reaches_the_mailbox_whole_and_esp_commands_never_do() {
    for port_under_test in [Port::Ble, Port::Usb] {
        let (mut port, mut fw) = booted(Family::F10x, Inject::default());
        attach(&mut port, &mut fw);
        let mut svc = service();
        let mut router = Router::new();
        if port_under_test == Port::Ble {
            router.connect(Port::Ble); // the USB console attaches by its first bytes
        }

        let info = encode(&Msg::InfoReq, 0x80);
        let board_pkts: Vec<Vec<u8>> = (0..6u8)
            .map(|i| {
                let mut p = vec![0x30, 0x80, 0x01];
                p.extend((0..(i * 9 + 1)).map(|b| b ^ i));
                p
            })
            .collect();
        let mut all = board_pkts.clone();
        all.insert(2, info.clone());
        all.insert(5, info.clone());
        let bytes = stream(&all);

        // Split at every pattern from single bytes to coalesced runs, as the CC2541, a USB host
        // and a TCP relay do.
        for split in [1usize, 2, 3, 7, 18, 20, 64, 1000] {
            for chunk in bytes.chunks(split) {
                router.on_bytes(port_under_test, chunk);
                router.pump(&mut port, &mut svc);
            }
            for _ in 0..4 {
                router.pump(&mut port, &mut svc);
            }
            assert_eq!(fw.poll(), board_pkts, "{port_under_test:?} split {split}");
            let replies = received(&mut router, port_under_test);
            assert_eq!(
                replies.len(),
                2,
                "two ESP_INFO answers, {port_under_test:?} split {split}"
            );
            for r in replies {
                let pdu = Pdu::decode(&r).unwrap();
                assert_eq!((pdu.src, pdu.dst), (0x00, 0x80));
                assert!(matches!(Msg::decode(&pdu), Ok(Msg::Info(_))));
            }
        }
    }
}

#[test]
fn with_both_links_attached_replies_go_back_where_the_request_came_from() {
    let (mut port, mut fw) = booted(Family::F1x0, Inject::default());
    attach(&mut port, &mut fw);
    let mut svc = service();
    let mut router = Router::new();
    router.connect(Port::Ble);
    router.on_bytes(Port::Usb, &[]);
    assert!(router.attached(Port::Usb));

    // An ESP_INFO on each link is answered on that link alone.
    router.on_bytes(Port::Ble, &stream(&[encode(&Msg::InfoReq, 0x80)]));
    router.on_bytes(Port::Usb, &stream(&[encode(&Msg::TargetIdReq, 0x81)]));
    router.pump(&mut port, &mut svc);
    let ble = received(&mut router, Port::Ble);
    let usb = received(&mut router, Port::Usb);
    assert_eq!(ble.len(), 1);
    assert!(matches!(
        Msg::decode(&Pdu::decode(&ble[0]).unwrap()),
        Ok(Msg::Info(_))
    ));
    assert_eq!(usb.len(), 1);
    assert!(matches!(
        Msg::decode(&Pdu::decode(&usb[0]).unwrap()),
        Ok(Msg::TargetId(_))
    ));

    // A board packet goes to both controllers.
    fw.link.send(&[0x32, 0x01, 0x80, 0x10, 0, 0, 1, 7]).unwrap();
    router.pump(&mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Ble).len(), 1);
    assert_eq!(received(&mut router, Port::Usb).len(), 1);

    // A flash begun over USB reports there to the end; the BLE central hears none of it.
    let img = image(3000);
    let sha: [u8; 32] = Sha256::digest(&img).into();
    let begin = Msg::FlashBegin {
        len: img.len() as u32,
        sha256: sha,
    };
    router.on_bytes(Port::Usb, &stream(&[encode(&begin, 0x81)]));
    router.pump(&mut port, &mut svc);
    let chunks: Vec<Vec<u8>> = img
        .chunks(espsvc::CHUNK)
        .enumerate()
        .map(|(i, d)| {
            encode(
                &Msg::FlashChunk {
                    offset: (i * espsvc::CHUNK) as u32,
                    data: d,
                },
                0x81,
            )
        })
        .collect();
    let mut usb = received(&mut router, Port::Usb);
    for c in chunks.chunks(4) {
        router.on_bytes(Port::Usb, &stream(c));
        router.pump(&mut port, &mut svc);
        usb.extend(received(&mut router, Port::Usb));
    }
    for _ in 0..10_000 {
        router.pump(&mut port, &mut svc);
        usb.extend(received(&mut router, Port::Usb));
        if !svc.busy() {
            break;
        }
    }
    assert!(received(&mut router, Port::Ble).is_empty());
    let last = Pdu::decode(usb.last().unwrap()).unwrap();
    assert!(matches!(
        Msg::decode(&last),
        Ok(Msg::FlashStatus {
            stage: espsvc::Stage::Booted,
            status: Status::Ok,
            ..
        })
    ));
    assert!(usb.iter().all(|p| Pdu::decode(p).unwrap().dst == 0x81));

    // The USB controller goes away: board packets still reach the BLE central.
    router.disconnect(Port::Usb);
    let mut fw = Firmware::new(port.swd().pins().bus.mailbox());
    for _ in 0..20 {
        port.swd().pins().sleep_us(20_000);
        router.pump(&mut port, &mut svc);
        fw.poll();
    }
    fw.link.send(&[0x32, 0x01, 0x80, 0x10, 0, 0, 1, 8]).unwrap();
    router.pump(&mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Ble).len(), 1);
    assert!(received(&mut router, Port::Usb).is_empty());
}

#[test]
fn the_usb_console_answers_again_after_its_host_closes_and_reopens_the_port() {
    let (mut port, mut fw) = booted(Family::F10x, Inject::default());
    attach(&mut port, &mut fw);
    let mut svc = service();
    let mut router = Router::new();
    let info = stream(&[encode(&Msg::InfoReq, 0x80)]);
    let answered = |router: &mut Router| {
        received(router, Port::Usb)
            .iter()
            .filter(|p| matches!(Msg::decode(&Pdu::decode(p).unwrap()), Ok(Msg::Info(_))))
            .count()
    };

    router.on_bytes(Port::Usb, &info);
    router.pump(&mut port, &mut svc);
    assert_eq!(answered(&mut router), 1);

    // The host stops reading (the firmware's write wait runs out): the port detaches, and the
    // next host's first bytes attach it again on a clean stream.
    router.disconnect(Port::Usb);
    assert!(!router.attached(Port::Usb));
    router.on_bytes(Port::Usb, &info);
    router.pump(&mut port, &mut svc);
    assert_eq!(answered(&mut router), 1);

    // A host that goes away mid-frame leaves a frame start whose length runs past every request a
    // new host would send; the console stays attached, since nothing was waiting to be written.
    let cut = &info[..info.len() - 3];
    let mut stuck = vec![link::SOF, 120];
    stuck.extend_from_slice(cut);
    router.on_bytes(Port::Usb, &stuck);
    for _ in 0..3 {
        router.on_bytes(Port::Usb, &info);
        router.pump(&mut port, &mut svc);
    }
    assert_eq!(answered(&mut router), 0, "swallowed by the stale frame");
    // The firmware drops the partial frame after a quiet gap; the next request is answered.
    router.drop_partial(Port::Usb);
    router.on_bytes(Port::Usb, &info);
    router.pump(&mut port, &mut svc);
    assert_eq!(answered(&mut router), 1);
}
