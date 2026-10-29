//! Host tests (`specs/hoverboard-bridge.md`, "Verification"): the engine, the board operations, flashing,
//! the mailbox session and the routing between the controller links, against the bit-level simulated debug port and the
//! GD32 model in [`crate::sim`].

extern crate std;

use std::vec;
use std::vec::Vec;

use espsvc::{Msg, Status, FAMILY_F10X, FAMILY_F1X0};
use link::StreamFramer;
use net::pdu::{Opcode, Pdu, NO_ADDRESS};
use sha2::{Digest, Sha256};

use crate::flash::{self, Finish, FlashError, FlashJob, FLASH_BASE, PAGE, STORE_START};
use crate::loader::{self, Poll};
use crate::mailbox::MailboxPort;
use crate::node::Node;
use crate::router::{Port, Router, NOTIFY_MAX};
use crate::sim::{board, Family, Firmware, Gd32, Inject, Machine, RamFlash, SimPins, SimPlatform};
use crate::svc::{Build, ImageBuffer, Service};
use crate::swd::{Counters, Swd, SwdError, XferError};
use crate::target::{self, TargetError};
use crate::update::Update;

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
fn a_dead_line_keeps_the_connect_trying_for_500_ms_and_records_the_ack_bits() {
    let mut s = board(
        Family::F1x0,
        Inject {
            dead: true,
            ..Inject::default()
        },
        1,
    );
    let t0 = s.now_us();
    assert_eq!(s.connect(), Err(SwdError::Exhausted(XferError::NoAck)));
    let took = s.now_us() - t0;
    assert!(took > 500_000 && took < 501_000, "took {took} us");
    assert!(
        s.pins().switches > 1_000,
        "the wake was repeated through the window"
    );
    assert_eq!(s.counters().reconnects, 1, "one connect, one reconnect");
    let d = s.diag();
    assert_eq!(d.no_ack, Some(0b111), "the pull-up reads all ones");
    assert_eq!(d.bad_read, None);
}

#[test]
fn a_transfer_with_no_ack_leaves_swdio_released_for_34_clocks() {
    // A dead line: every request gets no ACK.
    let mut s = board(
        Family::F1x0,
        Inject {
            dead: true,
            ..Inject::default()
        },
        1,
    );
    let _ = s.connect();
    assert!(s.pins().noacks > 1_000);
    assert_eq!(s.pins().noack_release, Some(34));

    // An injected missing ACK in the middle of a session, on every kind of request.
    let mut s = board(
        Family::F1x0,
        Inject {
            noack: 100,
            ..Inject::default()
        },
        7,
    );
    s.connect().unwrap();
    let data: Vec<u32> = (0..300u32).collect();
    s.write_words(0x2000_0000, &data).unwrap();
    let mut back = vec![0u32; data.len()];
    s.read_words(0x2000_0000, &mut back).unwrap();
    assert_eq!(back, data);
    assert!(s.pins().noacks > 0);
    assert_eq!(s.pins().noack_release, Some(34));
    assert_counters_match(&mut s);
}

#[test]
fn a_parity_failure_records_the_ack_bits_and_the_data_word() {
    let mut s = board(Family::F10x, Inject::default(), 1);
    s.connect().unwrap();
    assert_eq!(s.diag(), crate::swd::LineDiag::default());

    // Every read's parity flipped: the connect retries for its window and fails, and the last bad
    // read is the DPIDR as it came back.
    s.pins().inject.parity = 1000;
    let t0 = s.now_us();
    assert_eq!(s.connect(), Err(SwdError::Exhausted(XferError::Parity)));
    // A wake that sees bad parity re-reads DPIDR up to the WAIT budget (257 reads, about 14 ms)
    // before the window is checked.
    let took = s.now_us() - t0;
    assert!(took > 500_000 && took < 520_000, "took {took} us");
    let d = s.diag();
    assert_eq!(
        d.bad_read,
        Some(crate::swd::BadRead {
            ack: 0b001,
            data: 0x1BA0_1477
        })
    );
    assert_eq!(d.no_ack, None);
    assert_counters_match(&mut s);

    // A clean line again connects, and the record stays until the next bad read.
    s.pins().inject.parity = 0;
    s.connect().unwrap();
    assert_eq!(s.diag(), d);
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
    let mut job = FlashJob::new(img.len(), Finish::Boot).unwrap();
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
        assert_eq!(g.loader_runs, 2, "programmed through the loader");
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

/// One loader run of `data` at `addr`, driven to its halt (the loader's code already in SRAM).
fn loader_run(s: &mut SimSwd, addr: u32, data: &[u8]) -> Result<(), FlashError> {
    let mut run = flash::start_run(s, addr, data.len() as u32)?;
    for _ in 0..100_000 {
        if let Poll::Done(o) = run.poll(s, data)? {
            return flash::run_result(&run, o);
        }
    }
    panic!("the loader never halted");
}

/// A halted, unlocked board with `pages` pages from page 1 erased and the loader in SRAM.
fn loader_board(fam: Family, pages: u32) -> SimSwd {
    let mut s = board(fam, Inject::default(), 1);
    s.connect().unwrap();
    target::halt(&mut s).unwrap();
    flash::unlock(&mut s).unwrap();
    for p in 1..=pages {
        flash::erase_page(&mut s, FLASH_BASE + p * PAGE).unwrap();
    }
    loader::load(&mut s).unwrap();
    s
}

#[test]
fn a_descending_program_fails_on_the_f1x0_and_not_on_the_f10x() {
    for (fam, fails) in [(Family::F1x0, true), (Family::F10x, false)] {
        let mut s = loader_board(fam, 1);
        let page = FLASH_BASE + PAGE;
        let mut result = Ok(());
        for a in (page..page + 8).step_by(2).rev() {
            result = loader_run(&mut s, a, &[0x12, 0x12]);
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
fn a_loader_run_that_hits_pgerr_reports_the_failing_address_after_the_ring_wraps() {
    for pace in [u32::MAX, 3] {
        let mut s = loader_board(Family::F1x0, 7);
        model(&mut s).loader_pace = pace;
        let base = FLASH_BASE + PAGE;
        // A halfword already programmed 5 KiB into the run: past the 4 KiB ring's first wrap.
        let bad = base + 5 * 1024 + 6;
        loader_run(&mut s, bad, &[0x34, 0x12]).unwrap();
        let img = image(6 * 1024);
        let r = loader_run(&mut s, base, &img);
        assert!(
            matches!(r, Err(FlashError::Program { addr, stat }) if addr == bad && stat & crate::sim::STAT_PGERR != 0),
            "{r:?}"
        );
        assert_eq!(
            s.read_word(loader::RP).unwrap(),
            0,
            "rp = 0 on the error exit"
        );
        let f = model(&mut s).flash_bytes();
        let o = (base - FLASH_BASE) as usize;
        assert_eq!(
            &f[o..o + 5 * 1024 + 6],
            &img[..5 * 1024 + 6],
            "everything below it programmed"
        );
        assert_eq!(
            &f[o + 5 * 1024 + 8..o + 6 * 1024],
            &[0xFF; 1024 - 8][..],
            "nothing above it"
        );
    }
}

#[test]
fn a_slow_loader_fills_the_ring_which_wraps_and_the_image_still_verifies() {
    for fam in [Family::F1x0, Family::F10x] {
        let mut s = board(fam, Inject::default(), 1);
        model(&mut s).loader_pace = 2;
        let img = image(13 * 1024 + 300); // three trips round the ring, a short last chunk
        run_job(&mut s, &img).unwrap();
        let g = model(&mut s);
        assert_eq!(&g.flash_bytes()[..img.len()], &img[..]);
        assert_eq!(g.loader_runs, 2, "pages 1 and up, then page 0");
    }
}

#[test]
fn images_shorter_than_a_chunk_or_a_page_or_of_odd_length_program_whole() {
    for len in [9, 700, 1024, 1025, 4 * 1024 + 2, 5 * 1024 + 37] {
        let mut s = board(Family::F1x0, Inject::default(), 1);
        let img = image(len);
        run_job(&mut s, &img).unwrap();
        let g = model(&mut s);
        let f = g.flash_bytes();
        assert_eq!(&f[..len], &img[..], "{len}");
        assert!(
            f[len..len.next_multiple_of(1024)]
                .iter()
                .all(|&b| b == 0xFF),
            "{len}"
        );
        assert_eq!(g.loader_runs, if len > 1024 { 2 } else { 1 }, "{len}");
    }
}

#[test]
fn a_write_that_reaches_the_store_is_refused_before_the_board_is_touched() {
    assert!(FlashJob::new(flash::MAX_IMAGE, Finish::Boot).is_ok());
    assert!(matches!(
        FlashJob::new(flash::MAX_IMAGE + 1, Finish::Boot),
        Err(FlashError::StoreRange)
    ));
    let mut s = board(Family::F1x0, Inject::default(), 1);
    s.connect().unwrap();
    assert_eq!(
        flash::erase_page(&mut s, STORE_START),
        Err(FlashError::StoreRange)
    );
    assert!(matches!(
        flash::start_run(&mut s, STORE_START - 2, 4),
        Err(FlashError::StoreRange)
    ));
    assert_eq!(model(&mut s).store_writes, 0);
    assert_eq!(model(&mut s).loader_runs, 0);
}

#[test]
fn a_flash_cut_short_leaves_page_0_erased() {
    let mut s = board(Family::F1x0, Inject::default(), 1);
    model(&mut s).load(&image(4096)); // something already there
    let img = image(3 * 1024);
    let mut job = FlashJob::new(img.len(), Finish::Boot).unwrap();
    // Step until pages 1 and 2 are in the ring and before page 0's run: power is lost here.
    loop {
        match job.step(&mut s, &img) {
            flash::Event::Progress(espsvc::Stage::Programming, n) if n >= 2048 => break,
            flash::Event::Finished(r) => panic!("finished early: {r:?}"),
            _ => {}
        }
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

// ---- the RAM loader ----

#[test]
fn the_loader_fits_and_carries_the_layout_this_crate_owns() {
    assert!(loader::CODE.len() < loader::CODE_MAX as usize);
    let words: Vec<u32> = loader::CODE
        .chunks(4)
        .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect();
    assert!(words.contains(&loader::WP), "the control block address");
    assert!(
        words.contains(&crate::sim::FMC_BASE),
        "the flash controller"
    );
    assert_eq!(
        loader::RP,
        loader::WP + 4,
        "rp follows wp, as the loader addresses it"
    );
}

/// The checked-in `f1x.bin` is what `f1x.S` assembles to, with the command at the top of the
/// source. Skipped (with a note) when the assembler is not on the path.
#[test]
fn the_checked_in_loader_bytes_are_what_the_source_assembles_to() {
    use std::process::Command;
    let app = concat!(env!("CARGO_MANIFEST_DIR"), "/..");
    if Command::new("arm-none-eabi-as")
        .arg("--version")
        .output()
        .is_err()
    {
        std::println!("skipped: arm-none-eabi-as is not on the path");
        return;
    }
    let dir = std::format!("{app}/target/loader-check");
    std::fs::create_dir_all(&dir).unwrap();
    let obj = std::format!("{dir}/f1x.o");
    let bin = std::format!("{dir}/f1x.bin");
    let ok = Command::new("arm-none-eabi-as")
        .args(["-mcpu=cortex-m3", "-mthumb", "-o", &obj])
        .arg(std::format!("{app}/loader/f1x.S"))
        .status()
        .unwrap()
        .success();
    assert!(ok, "the loader source assembles");
    let ok = Command::new("arm-none-eabi-objcopy")
        .args(["-O", "binary", &obj, &bin])
        .status()
        .unwrap()
        .success();
    assert!(ok, "objcopy");
    assert_eq!(std::fs::read(&bin).unwrap(), loader::CODE);
}

// ---- the mailbox session ----

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
        Port::Usb | Port::Tcp => 64,
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

/// The bridge's node over a fresh store: unassigned, as a new bridge boots.
fn esp_node() -> Node<'static, RamFlash> {
    let flash = std::boxed::Box::leak(std::boxed::Box::new(RamFlash::erased(4096)));
    Node::new(store::Store::mount(flash).unwrap(), true, 0x0001)
}

fn service() -> Service<'static, SimPlatform> {
    let rx: &mut ImageBuffer = std::boxed::Box::leak(std::boxed::Box::new(ImageBuffer::new()));
    let platform = SimPlatform::new();
    Service::new(
        Build {
            version: [0, 1, 0],
            commit: 0,
            swd_khz: 1000,
        },
        rx,
        std::boxed::Box::leak(std::boxed::Box::new(Update::new())),
        platform,
    )
}

/// A `CYCLIC_STATE` from `src` to the one peer, as the board emits it on its mailbox port. The node
/// never reads the payload, so any bytes serve.
fn cyclic_state(src: u8) -> Vec<u8> {
    let mut p = vec![linkctl::OP_CYCLIC_STATE, src, 0x00];
    p.extend(0..linkctl::CyclicState::ENCODED_LEN as u8);
    p
}

fn encode(m: &Msg, src: u8) -> Vec<u8> {
    let mut buf = [0u8; espsvc::MAX_PDU];
    let n = m.encode(src, 0x00, &mut buf).unwrap();
    buf[..n].to_vec()
}

#[test]
fn a_stream_split_anywhere_reaches_the_mailbox_whole_and_esp_commands_never_do() {
    for port_under_test in [Port::Ble, Port::Usb, Port::Tcp] {
        let (mut port, mut fw) = booted(Family::F10x, Inject::default());
        attach(&mut port, &mut fw);
        let mut svc = service();
        let mut router = Router::new();
        let mut node = esp_node();
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
                node.pump(&mut router, &mut port, &mut svc);
            }
            for _ in 0..4 {
                node.pump(&mut router, &mut port, &mut svc);
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
    let mut node = esp_node();
    router.connect(Port::Ble);
    router.on_bytes(Port::Usb, &[]);
    assert!(router.attached(Port::Usb));

    // An ESP_INFO on each link is answered on that link alone.
    router.on_bytes(Port::Ble, &stream(&[encode(&Msg::InfoReq, 0x80)]));
    router.on_bytes(Port::Usb, &stream(&[encode(&Msg::TargetIdReq, 0x81)]));
    node.pump(&mut router, &mut port, &mut svc);
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

    // The board's telemetry for its one peer goes to both controllers; a reply for one guest goes
    // to that guest's link alone, by the table.
    fw.link.send(&cyclic_state(0x01)).unwrap();
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Ble).len(), 1);
    assert_eq!(received(&mut router, Port::Usb).len(), 1);
    fw.link.send(&[0x32, 0x01, 0x80, 0x10, 0, 0, 1, 7]).unwrap();
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Ble).len(), 1);
    assert!(received(&mut router, Port::Usb).is_empty());

    // A flash begun over USB reports there to the end; the BLE central hears none of it.
    let img = image(3000);
    let sha: [u8; 32] = Sha256::digest(&img).into();
    let begin = Msg::FlashBegin {
        len: img.len() as u32,
        sha256: sha,
    };
    router.on_bytes(Port::Usb, &stream(&[encode(&begin, 0x81)]));
    node.pump(&mut router, &mut port, &mut svc);
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
        node.pump(&mut router, &mut port, &mut svc);
        usb.extend(received(&mut router, Port::Usb));
    }
    for _ in 0..10_000 {
        node.pump(&mut router, &mut port, &mut svc);
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
        node.pump(&mut router, &mut port, &mut svc);
        fw.poll();
    }
    fw.link.send(&cyclic_state(0x01)).unwrap();
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Ble).len(), 1);
    assert!(received(&mut router, Port::Usb).is_empty());
}

#[test]
fn the_usb_console_answers_again_after_its_host_closes_and_reopens_the_port() {
    let (mut port, mut fw) = booted(Family::F10x, Inject::default());
    attach(&mut port, &mut fw);
    let mut svc = service();
    let mut router = Router::new();
    let mut node = esp_node();
    let info = stream(&[encode(&Msg::InfoReq, 0x80)]);
    let answered = |router: &mut Router| {
        received(router, Port::Usb)
            .iter()
            .filter(|p| matches!(Msg::decode(&Pdu::decode(p).unwrap()), Ok(Msg::Info(_))))
            .count()
    };

    router.on_bytes(Port::Usb, &info);
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(answered(&mut router), 1);

    // The host stops reading (the firmware's write wait runs out): the port detaches, and the
    // next host's first bytes attach it again on a clean stream.
    router.disconnect(Port::Usb);
    assert!(!router.attached(Port::Usb));
    router.on_bytes(Port::Usb, &info);
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(answered(&mut router), 1);

    // A host that goes away mid-frame leaves a frame start whose length runs past every request a
    // new host would send; the console stays attached, since nothing was waiting to be written.
    let cut = &info[..info.len() - 3];
    let mut stuck = vec![link::SOF, 120];
    stuck.extend_from_slice(cut);
    router.on_bytes(Port::Usb, &stuck);
    for _ in 0..3 {
        router.on_bytes(Port::Usb, &info);
        node.pump(&mut router, &mut port, &mut svc);
    }
    assert_eq!(answered(&mut router), 0, "swallowed by the stale frame");
    // The firmware drops the partial frame after a quiet gap; the next request is answered.
    router.drop_partial(Port::Usb);
    router.on_bytes(Port::Usb, &info);
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(answered(&mut router), 1);
}

#[test]
fn esp_info_reports_the_wifi_station() {
    let (mut port, mut fw) = booted(Family::F10x, Inject::default());
    attach(&mut port, &mut fw);
    let rx: &mut ImageBuffer = std::boxed::Box::leak(std::boxed::Box::new(ImageBuffer::new()));
    let mut platform = SimPlatform::new();
    platform.wifi = (espsvc::WifiState::Up, [192, 168, 0, 57]);
    let build = Build {
        version: [0, 1, 0],
        commit: 0,
        swd_khz: 1000,
    };
    let mut svc = Service::new(
        build,
        rx,
        std::boxed::Box::leak(std::boxed::Box::new(Update::new())),
        platform,
    );
    let mut router = Router::new();
    let mut node = esp_node();
    router.on_bytes(Port::Tcp, &stream(&[encode(&Msg::InfoReq, 0x80)]));
    node.pump(&mut router, &mut port, &mut svc);
    let r = received(&mut router, Port::Tcp);
    let Ok(Msg::Info(i)) = Msg::decode(&Pdu::decode(&r[0]).unwrap()) else {
        panic!("no ESP_INFO");
    };
    assert_eq!(
        (i.wifi, i.ip),
        (espsvc::WifiState::Up as u8, [192, 168, 0, 57])
    );
}

// ---- the bridge's own update ----

/// An ESP32-C3 app image of `len` bytes whose app descriptor carries `version`.
fn esp_image(len: usize, version: &str, seed: u32) -> Vec<u8> {
    let mut img: Vec<u8> = (0..len as u32)
        .map(|i| (i.wrapping_mul(2_654_435_761).wrapping_add(seed) >> 13) as u8)
        .collect();
    img[0] = 0xE9;
    img[12..14].copy_from_slice(&5u16.to_le_bytes());
    img[0x20..0x24].copy_from_slice(&0xABCD_5432u32.to_le_bytes());
    img[0x30..0x50].fill(0);
    img[0x30..0x30 + version.len()].copy_from_slice(version.as_bytes());
    img
}

/// The bridge's service loop, with an update sent through it over the TCP console.
struct UpdateRig {
    port: MailboxPort<SimPins<Gd32>>,
    svc: Service<'static, SimPlatform>,
    router: Router,
    node: Node<'static, RamFlash>,
}

impl UpdateRig {
    fn new(platform: SimPlatform) -> Self {
        let (mut port, mut fw) = booted(Family::F10x, Inject::default());
        attach(&mut port, &mut fw);
        let rx: &mut ImageBuffer = std::boxed::Box::leak(std::boxed::Box::new(ImageBuffer::new()));
        let build = Build {
            version: [0, 1, 0],
            commit: 0,
            swd_khz: 1000,
        };
        let mut router = Router::new();
        router.connect(Port::Tcp);
        UpdateRig {
            node: esp_node(),
            port,
            svc: Service::new(
                build,
                rx,
                std::boxed::Box::leak(std::boxed::Box::new(Update::new())),
                platform,
            ),
            router,
        }
    }

    /// Send `msgs`, return the decoded replies' PDUs. The bytes go in as the firmware's service
    /// loop moves them, a TCP read at a time while the port has room for one, a pass each.
    fn exchange(&mut self, msgs: &[Vec<u8>]) -> Vec<Vec<u8>> {
        let bytes = stream(msgs);
        let mut rest = &bytes[..];
        let mut out = Vec::new();
        loop {
            while !rest.is_empty() && self.router.room(Port::Tcp) >= 256 {
                let n = rest.len().min(256);
                self.router.on_bytes(Port::Tcp, &rest[..n]);
                rest = &rest[n..];
            }
            self.node
                .pump(&mut self.router, &mut self.port, &mut self.svc);
            out.extend(received(&mut self.router, Port::Tcp));
            if rest.is_empty() {
                break;
            }
        }
        assert_eq!(self.router.rx_overflow, 0);
        out
    }

    fn result(&mut self, m: &Msg) -> Status {
        let r = self.exchange(&[encode(m, 0x80)]);
        match Msg::decode(&Pdu::decode(r.last().unwrap()).unwrap()) {
            Ok(Msg::Result { status, .. }) => status,
            other => panic!("{other:?}"),
        }
    }

    fn info(&mut self) -> espsvc::EspInfo {
        let r = self.exchange(&[encode(&Msg::InfoReq, 0x80)]);
        match Msg::decode(&Pdu::decode(&r[0]).unwrap()) {
            Ok(Msg::Info(i)) => i,
            other => panic!("{other:?}"),
        }
    }

    /// `ESP_UPDATE_BEGIN` and the chunks with the console's window, chunks dropped, duplicated
    /// and reordered by `seed`'s schedule; the final `RESULT` for `ESP_UPDATE_CHUNK`.
    fn update(&mut self, img: &[u8], version: [u8; 3], seed: u64) -> Status {
        let sha256: [u8; 32] = Sha256::digest(img).into();
        let begin = Msg::UpdateBegin {
            len: img.len() as u32,
            sha256,
            version,
        };
        let s = self.result(&begin);
        if s != Status::Ok {
            return s;
        }
        let mut rng = seed;
        let mut next = || {
            rng = rng
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (rng >> 33) as u32
        };
        let mut tx = espsvc::BulkTx::new(img.len() as u32, espsvc::WINDOW_STREAM);
        for _ in 0..100_000 {
            let mut wire = Vec::new();
            while let Some((off, n)) = tx.next_chunk() {
                let m = encode(
                    &Msg::UpdateChunk {
                        offset: off,
                        data: &img[off as usize..off as usize + n],
                    },
                    0x80,
                );
                match next() % 10 {
                    0 => {}
                    1 => wire.extend([m.clone(), m]),
                    _ => wire.push(m),
                }
            }
            if wire.len() > 1 {
                let i = next() as usize % wire.len();
                let j = next() as usize % wire.len();
                wire.swap(i, j);
            }
            let before = tx.acked();
            for r in self.exchange(&wire) {
                match Msg::decode(&Pdu::decode(&r).unwrap()).unwrap() {
                    Msg::UpdateAck { contiguous } => tx.ack(contiguous),
                    Msg::Result { op, status } => {
                        assert_eq!(op, espsvc::OP_UPDATE_CHUNK);
                        return status;
                    }
                    other => panic!("{other:?}"),
                }
            }
            if tx.acked() == before {
                tx.rewind();
            }
        }
        panic!("the update stalled");
    }
}

#[test]
fn an_update_lands_in_the_other_slot_and_boots_pending_until_confirmed() {
    let img = esp_image(10_003, "0.2.0", 1);
    let mut rig = UpdateRig::new(SimPlatform::with_slots(16 * 1024));
    let i = rig.info();
    assert_eq!(
        (i.slot, i.pending),
        (0, false),
        "a first install runs ota_0"
    );

    assert_eq!(rig.update(&img, [0, 2, 0], 7), Status::Ok);
    assert!(rig.svc.reboot_requested());
    let p = rig.svc.platform();
    assert_eq!(&p.slots[1][..img.len()], &img[..]);
    assert!(p.slots[1][img.len()..].iter().all(|&b| b == 0xFF));
    assert!(
        p.slots[0].iter().all(|&b| b == 0xFF),
        "the running slot untouched"
    );
    // A duplicate of the last chunk after the end is acknowledged and changes nothing.
    let tail = img.len() / espsvc::CHUNK * espsvc::CHUNK;
    let dup = encode(
        &Msg::UpdateChunk {
            offset: tail as u32,
            data: &img[tail..],
        },
        0x80,
    );
    let r = rig.exchange(&[dup]);
    assert_eq!(
        Msg::decode(&Pdu::decode(&r[0]).unwrap()),
        Ok(Msg::UpdateAck {
            contiguous: img.len() as u32
        })
    );

    // The reset: the new slot boots pending verification; a confirm makes it valid.
    let mut p = core::mem::take(rig.svc.platform());
    p.reset();
    let mut rig = UpdateRig::new(p);
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (1, true));
    assert_eq!(rig.result(&Msg::UpdateConfirm), Status::Ok);
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (1, false));
    rig.svc.platform().reset();
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (1, false), "confirmed: it stays");
    // The next update goes to ota_0.
    let img2 = esp_image(5_000, "0.3.0", 2);
    assert_eq!(rig.update(&img2, [0, 3, 0], 8), Status::Ok);
    assert_eq!(&rig.svc.platform().slots[0][..img2.len()], &img2[..]);
}

#[test]
fn an_unconfirmed_update_returns_to_the_previous_slot_at_the_next_reset() {
    let img = esp_image(9_000, "0.2.0", 3);
    let mut rig = UpdateRig::new(SimPlatform::with_slots(16 * 1024));
    assert_eq!(rig.update(&img, [0, 2, 0], 9), Status::Ok);
    rig.svc.platform().reset();
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (1, true));
    // No confirm: the next reset runs ota_0 again.
    rig.svc.platform().reset();
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (0, false));
    // An update from there writes ota_1 again, and boots it.
    assert_eq!(rig.update(&img, [0, 2, 0], 10), Status::Ok);
    rig.svc.platform().reset();
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (1, true));
}

#[test]
fn a_wrong_image_is_never_activated() {
    let img = esp_image(6_000, "0.2.0", 4);
    let platform = SimPlatform::with_slots(8 * 1024);
    let mut rig = UpdateRig::new(platform);

    // Larger than a slot: refused at the start.
    let big = esp_image(8 * 1024 + 1, "0.2.0", 4);
    assert_eq!(rig.update(&big, [0, 2, 0], 1), Status::TooLarge);
    // A version other than the one announced.
    assert_eq!(rig.update(&img, [0, 2, 1], 2), Status::BadImage);
    // Not an ESP32-C3 app image: a board image.
    let board: Vec<u8> = image(6_000);
    assert_eq!(rig.update(&board, [0, 2, 0], 3), Status::BadImage);
    // A SHA-256 that is not the image's.
    let s = rig.result(&Msg::UpdateBegin {
        len: img.len() as u32,
        sha256: [0; 32],
        version: [0, 2, 0],
    });
    assert_eq!(s, Status::Ok);
    let mut last = None;
    for (i, c) in img.chunks(espsvc::CHUNK).enumerate() {
        let m = Msg::UpdateChunk {
            offset: (i * espsvc::CHUNK) as u32,
            data: c,
        };
        for r in rig.exchange(&[encode(&m, 0x80)]) {
            last = Some(Msg::decode(&Pdu::decode(&r).unwrap()).unwrap().opcode());
            if let Ok(Msg::Result { status, .. }) = Msg::decode(&Pdu::decode(&r).unwrap()) {
                assert_eq!(status, Status::ShaMismatch);
            }
        }
    }
    assert_eq!(last, Some(espsvc::OP_RESULT));
    // A late chunk of the refused update gets no answer.
    let m = Msg::UpdateChunk {
        offset: 0,
        data: &img[..espsvc::CHUNK],
    };
    assert!(rig.exchange(&[encode(&m, 0x80)]).is_empty());

    assert!(!rig.svc.reboot_requested());
    let p = rig.svc.platform();
    assert_eq!(p.selected, None, "nothing activated");
    p.reset();
    let i = rig.info();
    assert_eq!((i.slot, i.pending), (0, false));
}

/// For each chunk sent alone, in order, the acknowledgement it drew (`None` for none).
fn acks_per_chunk(rig: &mut UpdateRig, chunks: &[Vec<u8>], ack_op: u8) -> Vec<Option<u32>> {
    chunks
        .iter()
        .map(|c| {
            let r = rig.exchange(core::slice::from_ref(c));
            let acks: Vec<u32> = r
                .iter()
                .filter(|p| p[0] == ack_op)
                .map(|p| match Msg::decode(&Pdu::decode(p).unwrap()).unwrap() {
                    Msg::FlashAck { contiguous } | Msg::UpdateAck { contiguous } => contiguous,
                    other => panic!("{other:?}"),
                })
                .collect();
            assert!(acks.len() <= 1, "{acks:?}");
            acks.first().copied()
        })
        .collect()
}

#[test]
fn the_receiver_acknowledges_after_every_eight_chunks_and_at_the_end() {
    // 20 chunks and a 13 B tail, in order: acknowledgements after chunks 8, 16 and 21.
    let n = 20 * espsvc::CHUNK + 13;
    let want: Vec<Option<u32>> = (1..=21u32)
        .map(|i| match i {
            8 | 16 => Some(i * espsvc::CHUNK as u32),
            21 => Some(n as u32),
            _ => None,
        })
        .collect();

    // The board flash.
    let img = image(n);
    let mut rig = UpdateRig::new(SimPlatform::new());
    let sha256: [u8; 32] = Sha256::digest(&img).into();
    let begin = Msg::FlashBegin {
        len: n as u32,
        sha256,
    };
    assert_eq!(rig.result(&begin), Status::Ok);
    let chunks: Vec<Vec<u8>> = img
        .chunks(espsvc::CHUNK)
        .enumerate()
        .map(|(i, d)| {
            let offset = (i * espsvc::CHUNK) as u32;
            encode(&Msg::FlashChunk { offset, data: d }, 0x80)
        })
        .collect();
    assert_eq!(
        acks_per_chunk(&mut rig, &chunks, espsvc::OP_FLASH_ACK),
        want
    );

    // The bridge's own update.
    let img = esp_image(n, "0.2.0", 5);
    let mut rig = UpdateRig::new(SimPlatform::with_slots(16 * 1024));
    let sha256: [u8; 32] = Sha256::digest(&img).into();
    let begin = Msg::UpdateBegin {
        len: n as u32,
        sha256,
        version: [0, 2, 0],
    };
    assert_eq!(rig.result(&begin), Status::Ok);
    let chunks: Vec<Vec<u8>> = img
        .chunks(espsvc::CHUNK)
        .enumerate()
        .map(|(i, d)| {
            let offset = (i * espsvc::CHUNK) as u32;
            encode(&Msg::UpdateChunk { offset, data: d }, 0x80)
        })
        .collect();
    assert_eq!(
        acks_per_chunk(&mut rig, &chunks, espsvc::OP_UPDATE_ACK),
        want
    );
    assert!(rig.svc.reboot_requested());
}

#[test]
fn an_update_erases_the_next_sector_ahead_in_a_quiet_pass_and_every_sector_before_writing_it() {
    let img = esp_image(3 * 4096 + 1000, "0.2.0", 6);
    let sha256: [u8; 32] = Sha256::digest(&img).into();
    let chunk = |i: usize| {
        let o = i * espsvc::CHUNK;
        let d = &img[o..(o + espsvc::CHUNK).min(img.len())];
        encode(
            &Msg::UpdateChunk {
                offset: o as u32,
                data: d,
            },
            0x80,
        )
    };
    let chunks = img.len().div_ceil(espsvc::CHUNK);
    for quiet_passes in [true, false] {
        // A slot holding an older image: a sector written without its erase keeps its zeros.
        let mut platform = SimPlatform::with_slots(16 * 1024);
        platform.slots[1].fill(0x00);
        let mut rig = UpdateRig::new(platform);
        let begin = Msg::UpdateBegin {
            len: img.len() as u32,
            sha256,
            version: [0, 2, 0],
        };
        assert_eq!(rig.result(&begin), Status::Ok);
        rig.exchange(&(0..10).map(chunk).collect::<Vec<_>>());
        assert!(
            rig.svc.platform().slots[1][..2 * 4096]
                .iter()
                .all(|&b| b == 0),
            "nothing erased while chunks arrive"
        );
        if quiet_passes {
            // Passes with no controller packet: the sector being filled and the next one are
            // erased, one per pass, and no further.
            for _ in 0..4 {
                rig.exchange(&[]);
            }
            let slot = &rig.svc.platform().slots[1];
            assert!(slot[..2 * 4096].iter().all(|&b| b == 0xFF));
            assert!(slot[2 * 4096..].iter().all(|&b| b == 0));
        }
        let rest: Vec<Vec<u8>> = (10..chunks).map(chunk).collect();
        let r = rig.exchange(&rest);
        let last = Msg::decode(&Pdu::decode(r.last().unwrap()).unwrap()).unwrap();
        assert_eq!(
            last,
            Msg::Result {
                op: espsvc::OP_UPDATE_CHUNK,
                status: Status::Ok
            },
            "quiet passes {quiet_passes}"
        );
        let slot = &rig.svc.platform().slots[1];
        assert_eq!(&slot[..img.len()], &img[..]);
        assert!(slot[img.len()..4 * 4096].iter().all(|&b| b == 0xFF));
        assert!(slot[4 * 4096..].iter().all(|&b| b == 0), "past the image");
    }
}

#[test]
fn both_console_carriers_are_served_at_once_and_replies_go_back_where_they_came_from() {
    let (mut port, mut fw) = booted(Family::F1x0, Inject::default());
    attach(&mut port, &mut fw);
    let mut svc = service();
    let mut router = Router::new();
    let mut node = esp_node();
    // A USB host that never reads still counts: nothing detects it leaving.
    router.on_bytes(Port::Usb, &stream(&[encode(&Msg::InfoReq, 0x80)]));
    router.connect(Port::Tcp);
    router.on_bytes(Port::Tcp, &stream(&[encode(&Msg::TargetIdReq, 0x81)]));
    node.pump(&mut router, &mut port, &mut svc);
    let usb = received(&mut router, Port::Usb);
    let tcp = received(&mut router, Port::Tcp);
    assert_eq!(usb.len(), 1);
    assert!(matches!(
        Msg::decode(&Pdu::decode(&usb[0]).unwrap()),
        Ok(Msg::Info(_))
    ));
    assert_eq!(tcp.len(), 1);
    assert!(matches!(
        Msg::decode(&Pdu::decode(&tcp[0]).unwrap()),
        Ok(Msg::TargetId(_))
    ));

    // A board packet goes out on both carriers.
    fw.link.send(&[0x32, 0x01, 0x80, 0x10, 0, 0, 1, 7]).unwrap();
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Usb).len(), 1);
    assert_eq!(received(&mut router, Port::Tcp).len(), 1);

    // The TCP client hangs up; the next one starts on a clean stream, even after a partial frame.
    let info = stream(&[encode(&Msg::InfoReq, 0x82)]);
    router.on_bytes(Port::Tcp, &info[..info.len() - 2]);
    router.disconnect(Port::Tcp);
    router.connect(Port::Tcp);
    router.on_bytes(Port::Tcp, &info);
    node.pump(&mut router, &mut port, &mut svc);
    assert_eq!(received(&mut router, Port::Tcp).len(), 1);
    assert!(received(&mut router, Port::Usb).is_empty());
}

// ---- the bridge as a node ----

/// A controller on one of the bridge's links: the `net` walk controller, answering the bridge's probes of
/// its link, with the replies that are not the walk's kept aside.
struct Guest {
    link: Port,
    ctrl: net::walk::Controller,
    /// `CONFIG_RESP`s, `PORTS` outside a walk, ESP replies, and anything else addressed to it.
    inbox: Vec<Vec<u8>>,
    /// Frames for the one peer (`dst = 0x00`) that reached it: the board's telemetry.
    heard: Vec<Vec<u8>>,
}

impl Guest {
    fn on(link: Port) -> Self {
        Guest {
            link,
            ctrl: net::walk::Controller::new(),
            inbox: Vec::new(),
            heard: Vec::new(),
        }
    }
}

/// The simulated machine ([`Machine`]) with controllers on the bridge's links, a console guest first.
struct Rig<'f> {
    m: Machine<'f>,
    guests: Vec<Guest>,
}

impl<'f> core::ops::Deref for Rig<'f> {
    type Target = Machine<'f>;
    fn deref(&self) -> &Machine<'f> {
        &self.m
    }
}

impl<'f> core::ops::DerefMut for Rig<'f> {
    fn deref_mut(&mut self) -> &mut Machine<'f> {
        &mut self.m
    }
}

impl<'f> Rig<'f> {
    /// A fresh bridge over `flash` in front of a master holding 0x01 and a slave holding 0x02, with a
    /// controller on the USB console.
    fn new(flash: &'f mut RamFlash) -> Self {
        Rig {
            m: Machine::new(flash, service()),
            guests: vec![Guest::on(Port::Usb)],
        }
    }

    fn send(&mut self, link: Port, pdu: &[u8]) {
        self.m.router.on_bytes(link, &stream(&[pdu.to_vec()]));
    }

    /// One pass of the machine, then each guest takes what reached it; whether anything moved.
    fn step(&mut self) -> bool {
        let mut moved = self.m.step();
        for g in self.guests.iter_mut() {
            let link = g.link;
            for f in received(&mut self.m.router, link) {
                moved = true;
                let p = Pdu::decode(&f).unwrap();
                if let Some(r) = g.ctrl.reply_to_probe(&f) {
                    self.m.router.on_bytes(link, &stream(&[r.to_vec()]));
                } else if p.dst == NO_ADDRESS {
                    g.heard.push(f);
                } else if !g.ctrl.is_complete()
                    && matches!(
                        p.known(),
                        Some(Opcode::NodeHello | Opcode::AssignAck | Opcode::Ports)
                    )
                {
                    g.ctrl.on_reply(&f);
                } else {
                    g.inbox.push(f);
                }
            }
        }
        moved
    }

    /// Run until nothing moves, letting each probe window run out once its answers are in.
    fn settle(&mut self) {
        for _ in 0..1000 {
            if self.step() || self.m.tick() {
                continue;
            }
            return;
        }
        panic!("the machine did not settle");
    }
}

impl Rig<'_> {
    /// Guest `g` walks the machine to completion.
    fn walk(&mut self, g: usize) {
        for _ in 0..200 {
            if self.guests[g].ctrl.is_complete() {
                return;
            }
            if let Some(req) = self.guests[g].ctrl.next_request() {
                let link = self.guests[g].link;
                self.send(link, &req);
            }
            self.settle();
        }
        panic!("the walk did not complete");
    }

    /// Send `opcode` from guest `g` to `dst` and settle; what reached the guest meanwhile.
    fn ask(&mut self, g: usize, opcode: u8, dst: u8, payload: &[u8]) -> Vec<Vec<u8>> {
        let src = self.guests[g].ctrl.guest_addr();
        let link = self.guests[g].link;
        self.send(link, &frame(opcode, src, dst, payload));
        self.settle();
        std::mem::take(&mut self.guests[g].inbox)
    }

    /// `CONFIG_READ` of `key` from `dst` by the console guest: the `CONFIG_RESP` payload.
    fn config_read(&mut self, dst: u8, key: store::Key) -> Vec<u8> {
        let r = self.ask(
            0,
            Opcode::ConfigRead.to_u8(),
            dst,
            &[key.field_id, key.index],
        );
        let p = r
            .iter()
            .map(|f| Pdu::decode(f).unwrap())
            .find(|p| p.known() == Some(Opcode::ConfigResp) && p.src == dst)
            .expect("a CONFIG_RESP");
        p.payload.to_vec()
    }

    fn esp_info(&mut self, g: usize, dst: u8) -> espsvc::EspInfo {
        let src = self.guests[g].ctrl.guest_addr();
        let r = self.ask(g, espsvc::OP_INFO_REQ, dst, &[]);
        r.iter()
            .map(|f| Pdu::decode(f).unwrap())
            .find_map(|p| match Msg::decode(&p) {
                Ok(Msg::Info(i)) if p.dst == src => Some(i),
                _ => None,
            })
            .expect("an ESP_INFO")
    }
}

fn frame(opcode: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
    let mut buf = [0u8; 300];
    let n = Pdu::new(opcode, src, dst, payload)
        .unwrap()
        .encode(&mut buf)
        .unwrap();
    buf[..n].to_vec()
}

fn esp_addr(flash: &mut RamFlash) -> Option<u8> {
    let s = store::Store::mount(flash).unwrap();
    let mut a = None;
    s.for_each_in(&[store::NODE_ADDRESS.key()], |_, _, v| {
        a = v.first().copied()
    });
    a
}

#[test]
fn a_console_controller_walks_the_esp_and_both_boards_and_the_esp_ends_at_0x03() {
    let mut flash = RamFlash::erased(4096);
    {
        let mut m = Rig::new(&mut flash);
        m.walk(0);

        // `specs/hoverboard-bridge.md`, "A fresh bridge in front of an addressed machine".
        assert_eq!(m.node.addr(), 0x03);
        assert_eq!(m.master.resp.addr(), 0x01);
        assert_eq!(m.slave.resp.addr(), 0x02);
        let mut found = m.guests[0].ctrl.assigned_addrs().to_vec();
        found.sort_unstable();
        assert_eq!(found, [0x01, 0x02, 0x03]);
        assert_eq!(m.guests[0].ctrl.gateway_addr(), Some(0x03));

        // `node.class`: 2 from the bridge, 1 from each board; a write of it is refused by the bridge.
        for (addr, class) in [(0x03, net::walk::CLASS_BRIDGE), (0x01, 1), (0x02, 1)] {
            let r = m.config_read(addr, store::NODE_CLASS.key());
            assert_eq!(
                r[2..],
                [net::walk::CFG_OK, store::Type::U8.tag(), class],
                "{addr:#04x}"
            );
        }
        let w = m.ask(
            0,
            Opcode::ConfigWrite.to_u8(),
            0x03,
            &[store::NODE_CLASS.id(), 0, store::Type::U8.tag(), 1],
        );
        assert_eq!(
            Pdu::decode(&w[0]).unwrap().payload[2],
            net::walk::CFG_READ_ONLY
        );

        // The bridge answers the master's probe of its mailbox port: the master reports it there.
        let ports = m.ask(0, Opcode::ProbePorts.to_u8(), 0x01, &[]);
        let p = Pdu::decode(&ports[0]).unwrap();
        assert_eq!(p.known(), Some(Opcode::Ports));
        assert_eq!(
            p.payload[1..5],
            [0, net::walk::PORT_SWD, net::walk::NB_ASSIGNED, 0x03]
        );

        // ESP_INFO, by the bridge's address, reports the address and the store.
        let i = m.esp_info(0, 0x03);
        assert_eq!((i.node, i.store_mounted), (0x03, true));
    }
    // The address is in the bridge's own store, and a reboot comes back with it.
    assert_eq!(esp_addr(&mut flash), Some(0x03));
    let node = Node::new(store::Store::mount(&mut flash).unwrap(), true, 0x0001);
    assert_eq!(node.addr(), 0x03);
}

#[test]
fn guests_on_ble_and_on_the_console_hold_different_addresses_and_both_hear_the_board() {
    let mut flash = RamFlash::erased(4096);
    let mut m = Rig::new(&mut flash);
    m.walk(0);
    m.router.connect(Port::Ble);
    m.router.connect(Port::Tcp);
    m.guests.push(Guest::on(Port::Ble));
    m.guests.push(Guest::on(Port::Tcp));

    // Both attach at the bridge, now 0x03: each is granted an address from the bridge's slice, a
    // different one each. The console guest attached while the bridge was unassigned (slice 0).
    let mut granted = Vec::new();
    for g in 0..3 {
        if g > 0 {
            let hello = m.guests[g].ctrl.next_request().unwrap();
            let link = m.guests[g].link;
            m.send(link, &hello);
            m.settle();
        }
        granted.push(m.guests[g].ctrl.granted().expect("an answered hello"));
    }
    let slice = 0x80 + 8 * 0x03;
    assert_eq!(granted[0], 0x80);
    assert!(
        granted[1..].iter().all(|a| (slice..slice + 8).contains(a)),
        "{granted:02x?}"
    );
    assert_ne!(granted[1], granted[2]);

    // The board's telemetry for its one peer (its own sample and the slave's forward) reaches
    // BLE, USB and TCP with `src` unchanged.
    for src in [0x01, 0x02] {
        m.fw.link.send(&cyclic_state(src)).unwrap();
    }
    m.settle();
    for g in &m.guests {
        let srcs: Vec<u8> = g.heard.iter().map(|f| f[1]).collect();
        assert_eq!(srcs, [0x01, 0x02], "{:?}", g.link);
        assert!(g
            .heard
            .iter()
            .all(|f| f[0] == linkctl::OP_CYCLIC_STATE && f[2] == 0x00));
    }

    // A reply for one guest goes to that guest alone: a CONFIG_RESP from the slave to the BLE
    // guest reaches BLE and neither console carrier.
    let ble = granted[1];
    let r = m.ask(
        1,
        Opcode::ConfigRead.to_u8(),
        0x02,
        &[store::NODE_CLASS.id(), 0],
    );
    assert_eq!(Pdu::decode(&r[0]).unwrap().dst, ble);
    assert!(m.guests[0].inbox.is_empty() && m.guests[2].inbox.is_empty());
}

#[test]
fn an_esp_command_to_the_esps_address_is_answered_from_either_side() {
    let mut flash = RamFlash::erased(4096);
    let mut m = Rig::new(&mut flash);
    m.walk(0);
    let me = m.node.addr();

    // From the console, by address and by the one-peer address.
    assert_eq!(m.esp_info(0, me).node, me);
    assert_eq!(m.esp_info(0, 0x00).node, me);

    // From the board side: a controller on another bridge of the machine, reached through the boards.
    // The reply goes back into the mailbox, from the bridge's address, to that controller.
    let answers = |m: &Rig| {
        m.into_mailbox
            .iter()
            .map(|f| Pdu::decode(f).unwrap())
            .filter(|p| p.opcode == espsvc::OP_INFO && (p.src, p.dst) == (me, 0xA8))
            .count()
    };
    m.into_mailbox.clear();
    m.fw.link
        .send(&frame(espsvc::OP_INFO_REQ, 0xA8, me, &[]))
        .unwrap();
    m.settle();
    assert_eq!(answers(&m), 1);

    // The same command to the one-peer address from the mailbox is not the bridge's to answer.
    m.into_mailbox.clear();
    m.fw.link
        .send(&frame(espsvc::OP_INFO_REQ, 0xA8, 0x00, &[]))
        .unwrap();
    m.settle();
    assert_eq!(answers(&m), 0);
    assert!(m.into_mailbox.is_empty());
}

#[test]
fn a_drive_frame_from_a_console_guest_floods_to_the_board_then_follows_the_learned_route() {
    // No walk: a guest attaches and drives at once, as `hoverboard-cli drive` does.
    let mut flash = RamFlash::erased(4096);
    let mut m = Rig::new(&mut flash);
    m.router.connect(Port::Ble);
    m.guests.push(Guest::on(Port::Ble));
    let hello = m.guests[0].ctrl.next_request().unwrap();
    m.send(Port::Usb, &hello);
    m.settle();
    let me = m.guests[0].ctrl.granted().unwrap();
    let drive = |m: &mut Rig| {
        m.send(
            Port::Usb,
            &frame(linkctl::OP_DRIVE_CMD, me, 0x01, &[1, 0x10, 0x00, 0, 0]),
        );
        m.settle();
    };

    // Unknown route: the flood reaches the master, and the BLE link too.
    assert_eq!(m.node.responder().forwarder().port_toward(0x01), None);
    drive(&mut m);
    assert_eq!(m.master.got, [(linkctl::OP_DRIVE_CMD, me)]);
    let flooded = std::mem::take(&mut m.guests[1].inbox);
    assert_eq!(flooded.len(), 1);
    assert_eq!(flooded[0][0], linkctl::OP_DRIVE_CMD);

    // The board's telemetry teaches the route; the next frame goes to the mailbox alone.
    m.fw.link.send(&cyclic_state(0x01)).unwrap();
    m.settle();
    assert_eq!(
        m.node.responder().forwarder().port_toward(0x01),
        Some(crate::node::L3_MAILBOX)
    );
    m.master.got.clear();
    drive(&mut m);
    assert_eq!(m.master.got, [(linkctl::OP_DRIVE_CMD, me)]);
    assert!(m.guests[1].inbox.is_empty());
}

#[test]
fn the_esp_keeps_its_name_and_address_in_its_store_and_reports_a_store_that_did_not_mount() {
    let mut flash = RamFlash::erased(4096);
    {
        let mut m = Rig::new(&mut flash);
        m.walk(0);
        let name = b"rover-esp";
        let mut w = vec![store::DEVICE_NAME.id(), 0, store::Type::Str.tag()];
        w.extend_from_slice(name);
        let r = m.ask(0, Opcode::ConfigWrite.to_u8(), 0x03, &w);
        assert_eq!(Pdu::decode(&r[0]).unwrap().payload[2], net::walk::CFG_OK);
        let r = m.config_read(0x03, store::DEVICE_NAME.key());
        assert_eq!(&r[4..], name);
        assert_eq!(m.node.store().get_text(store::DEVICE_NAME), name);
    }
    let s = store::Store::mount(&mut flash).unwrap();
    assert_eq!(s.get_text(store::DEVICE_NAME), b"rover-esp");

    // A store that did not mount: the firmware hands the node an empty one, and says so.
    let mut empty = RamFlash::erased(4096);
    let (mut port, mut fw) = booted(Family::F1x0, Inject::default());
    attach(&mut port, &mut fw);
    let mut node = Node::new(store::Store::mount(&mut empty).unwrap(), false, 0x0001);
    let mut svc = service();
    let mut router = Router::new();
    router.on_bytes(Port::Usb, &stream(&[encode(&Msg::InfoReq, 0x80)]));
    node.pump(&mut router, &mut port, &mut svc);
    let r = received(&mut router, Port::Usb);
    let Ok(Msg::Info(i)) = Msg::decode(&Pdu::decode(&r[0]).unwrap()) else {
        panic!("no ESP_INFO");
    };
    assert_eq!((i.node, i.store_mounted), (0x00, false));
}

// ---- the GDB server ----

use crate::gdb::{self, Gdb};
use crate::svc::Identity;
use std::string::{String, ToString};

type SimGdb<'b> = Gdb<'b, 'static, SimPins<Gd32>, SimPlatform>;

fn sim<'a>(g: &'a mut SimGdb<'_>) -> &'a mut Gd32 {
    &mut g.parts().0.swd().pins().bus
}

/// One packet as GDB frames it: `$data#checksum`.
fn packet(data: &[u8]) -> Vec<u8> {
    let sum = data.iter().fold(0u8, |a, b| a.wrapping_add(*b));
    let mut p = vec![b'$'];
    p.extend_from_slice(data);
    p.extend_from_slice(std::format!("#{sum:02x}").as_bytes());
    p
}

/// Binary data as GDB escapes it in `X` and `vFlashWrite`.
fn escape(data: &[u8]) -> Vec<u8> {
    let mut v = Vec::new();
    for &b in data {
        if matches!(b, b'#' | b'$' | b'}' | b'*') {
            v.extend([b'}', b ^ 0x20]);
        } else {
            v.push(b);
        }
    }
    v
}

/// One service pass for the session: the flash job, then the server.
fn pass(g: &mut SimGdb<'_>) {
    let (port, svc) = g.parts();
    svc.poll(port, &mut |_, _| {});
    g.poll();
}

/// The packets the server sent, `+` acks skipped, run-length encoding expanded.
fn replies(g: &mut SimGdb<'_>) -> Vec<Vec<u8>> {
    let mut raw = Vec::new();
    let mut buf = [0u8; 512];
    loop {
        let n = g.take_out(&mut buf);
        if n == 0 {
            break;
        }
        raw.extend_from_slice(&buf[..n]);
    }
    let mut out = Vec::new();
    let mut i = 0;
    while i < raw.len() {
        if raw[i] != b'$' {
            i += 1;
            continue;
        }
        let end = i + raw[i..].iter().position(|&b| b == b'#').unwrap();
        let mut body = Vec::new();
        for &b in &raw[i + 1..end] {
            body.push(b);
        }
        // Expand `x*n` runs.
        let mut exp = Vec::new();
        let mut k = 0;
        while k < body.len() {
            if body[k] == b'*' {
                let prev = *exp.last().unwrap();
                for _ in 0..(body[k + 1] - 29) {
                    exp.push(prev);
                }
                k += 2;
            } else {
                exp.push(body[k]);
                k += 1;
            }
        }
        out.push(exp);
        i = end + 3;
    }
    out
}

/// Send one packet and run passes until a reply that is not console output arrives (or `passes`
/// run out); every packet received, console output included.
fn ask_raw(g: &mut SimGdb<'_>, data: &[u8], passes: usize) -> Vec<Vec<u8>> {
    let mut bytes = packet(data);
    let mut got = Vec::new();
    for _ in 0..passes {
        let n = g.push(&bytes);
        bytes.drain(..n);
        pass(g);
        got.extend(replies(g));
        let last_is_answer = got
            .last()
            .is_some_and(|p: &Vec<u8>| !(p.first() == Some(&b'O') && p.len() > 1 && p != b"OK"));
        if bytes.is_empty() && last_is_answer {
            break;
        }
    }
    got
}

/// Send one packet; the answer.
fn ask(g: &mut SimGdb<'_>, data: &str) -> String {
    let r = ask_raw(g, data.as_bytes(), 50);
    String::from_utf8(r.last().cloned().unwrap_or_default()).unwrap()
}

/// A `monitor` command; the console text it printed.
fn monitor(g: &mut SimGdb<'_>, cmd: &str) -> String {
    let hex: String = cmd.bytes().map(|b| std::format!("{b:02x}")).collect();
    let r = ask_raw(g, std::format!("qRcmd,{hex}").as_bytes(), 50);
    assert_eq!(r.last().map(|v| v.as_slice()), Some(&b"OK"[..]));
    let mut text = String::new();
    for p in &r[..r.len() - 1] {
        let h = &p[1..];
        for k in (0..h.len()).step_by(2) {
            let s = std::str::from_utf8(&h[k..k + 2]).unwrap();
            text.push(u8::from_str_radix(s, 16).unwrap() as char);
        }
    }
    text
}

fn hex_le(v: u32) -> String {
    v.to_le_bytes()
        .iter()
        .map(|b| std::format!("{b:02x}"))
        .collect()
}

/// A running F1x0 with its mailbox attached, a service, and a GDB client connected that has
/// negotiated no-ack mode as GDB does over TCP.
fn gdb_board() -> (
    MailboxPort<SimPins<Gd32>>,
    Firmware,
    Service<'static, SimPlatform>,
) {
    let (port, fw) = crate::sim::running_board();
    (port, fw, service())
}

fn open(g: &mut SimGdb<'_>) {
    g.open();
    assert_eq!(ask(g, "QStartNoAckMode"), "OK");
    let sup = ask(g, "qSupported:swbreak+;hwbreak+;xmlRegisters=arm");
    assert!(sup.contains("PacketSize=1000"), "{sup}");
    assert!(sup.contains("qXfer:features:read+"), "{sup}");
    assert!(sup.contains("qXfer:memory-map:read+"), "{sup}");
}

#[test]
fn an_attach_halts_through_disarm_pauses_the_mailbox_and_a_detach_resumes() {
    let (mut port, mut fw, mut svc) = gdb_board();
    assert_eq!(port.state(), espsvc::MailboxState::Up);
    port.swd().pins().bus.cchp = 1 << 15;
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    let b = sim(&mut g);
    assert!(b.halted);
    assert_eq!(b.cchp & 1 << 15, 0, "disarmed before the halt");
    assert!(b.fpb_enabled);
    assert!(g.parts().1.gdb_status().connected && g.parts().1.gdb_status().halted);

    // Halted: the board's frames wait in its ring, and the bridge reads none.
    fw.link.send(&[0x32, 0x01, 0x80, 0x10]).unwrap();
    let mut got = Vec::new();
    g.parts().0.poll(|p| got.push(p.to_vec()));
    assert!(got.is_empty(), "the mailbox is left alone while halted");

    // Resumed: the same session carries the frame on.
    assert!(ask_raw(&mut g, b"c", 3).is_empty(), "no stop yet");
    assert!(!sim(&mut g).halted);
    g.parts().0.poll(|p| got.push(p.to_vec()));
    assert_eq!(got, vec![vec![0x32, 0x01, 0x80, 0x10]]);
    assert_eq!(g.parts().0.state(), espsvc::MailboxState::Up);

    // Interrupt and leave: the comparators are cleared and the core runs again.
    g.push(&[0x03]);
    pass(&mut g);
    assert_eq!(replies(&mut g).last().unwrap()[..3], *b"S02");
    assert!(sim(&mut g).halted);
    assert_eq!(ask(&mut g, "Z1,8000100,2"), "OK");
    assert_ne!(sim(&mut g).fp_comp[0], 0);
    g.close();
    let b = sim(&mut g);
    assert!(!b.halted, "a detach resumes");
    assert_eq!(b.fp_comp[0], 0, "and clears the comparators");
    assert!(!g.parts().1.gdb_status().connected);
}

#[test]
fn a_detach_and_a_kill_are_answered_before_the_session_ends() {
    // `specs/hoverboard-bridge.md`, "Debugging", "One client": the reply goes out before the
    // firmware is told to close the connection.
    for (cmd, want) in [("D", Some("OK")), ("k", None)] {
        let (mut port, _fw, mut svc) = gdb_board();
        let mut g = Gdb::new(&mut port, &mut svc);
        open(&mut g);
        let n = g.push(&packet(cmd.as_bytes()));
        assert_eq!(n, packet(cmd.as_bytes()).len());
        pass(&mut g);
        if want.is_some() {
            assert!(
                !g.take_ended(),
                "{cmd}: not ended while the reply waits to be taken"
            );
        }
        let got = replies(&mut g);
        assert_eq!(
            got.last()
                .map(|p| String::from_utf8(p.clone()).unwrap())
                .as_deref(),
            want,
            "{cmd}"
        );
        assert!(g.take_ended(), "{cmd}: ended once any reply is out");
        assert!(!g.take_ended(), "reading it clears it");
        assert!(!sim(&mut g).halted, "{cmd}: the core runs again");
        assert!(!g.parts().1.gdb_status().connected);
    }
}

#[test]
fn registers_read_and_write_round_trip_through_dcrsr() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    let b = sim(&mut g);
    for (i, r) in b.regs.iter_mut().enumerate() {
        *r = 0x1000_0000 + i as u32;
    }
    b.regs[20] = 0x0201_FF01; // CONTROL 2, FAULTMASK 1, BASEPRI 0xFF, PRIMASK 1
    let g_reply = ask(&mut g, "g");
    let mut want = String::new();
    for i in 0..19u32 {
        want += &hex_le(0x1000_0000 + i);
    }
    want += "01ff0102";
    assert_eq!(g_reply, want);

    // `G` writes them all back changed; `p` and `P` one at a time.
    let mut all = String::new();
    for i in 0..19u32 {
        all += &hex_le(0x2000_0000 + i);
    }
    all += "00100203";
    assert_eq!(ask(&mut g, &std::format!("G{all}")), "OK");
    let b = sim(&mut g);
    assert_eq!(b.regs[0], 0x2000_0000);
    assert_eq!(b.regs[15], 0x2000_000F);
    assert_eq!(b.regs[18], 0x2000_0012);
    assert_eq!(b.regs[20], 0x0302_1000);
    assert_eq!(ask(&mut g, "pf"), hex_le(0x2000_000F));
    assert_eq!(
        ask(&mut g, &std::format!("Pf={}", hex_le(0x0800_0201))),
        "OK"
    );
    assert_eq!(sim(&mut g).regs[15], 0x0800_0201);
    assert_eq!(ask(&mut g, "p13"), "00", "primask");
    assert_eq!(ask(&mut g, "P13=01"), "OK");
    assert_eq!(ask(&mut g, "P16=03"), "OK");
    assert_eq!(
        sim(&mut g).regs[20],
        0x0302_1001,
        "primask and control, the rest kept"
    );
}

#[test]
fn memory_reads_and_writes_word_byte_and_binary_and_while_running() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    assert_eq!(ask(&mut g, "M20001001,3:a1b2c3"), "OK");
    assert_eq!(ask(&mut g, "m20001000,5"), "00a1b2c300");
    let mut x = b"X20001100,4:".to_vec();
    x.extend(escape(&[0x23, 0x24, 0x7D, 0x2A]));
    assert_eq!(
        String::from_utf8(ask_raw(&mut g, &x, 50).pop().unwrap()).unwrap(),
        "OK"
    );
    assert_eq!(ask(&mut g, "m20001100,4"), "23247d2a");
    // The flash reads too, as GDB's `x` and `compare-sections` do.
    sim(&mut g).load(&[0x00, 0x20, 0x00, 0x20, 0x09, 0x01, 0x00, 0x08]);
    assert_eq!(ask(&mut g, "m8000004,4"), "09010008");

    // The core running: memory access still answers.
    assert!(ask_raw(&mut g, b"c", 3).is_empty());
    assert!(!sim(&mut g).halted);
    assert_eq!(ask(&mut g, "m20001001,2"), "a1b2");
}

#[test]
fn a_breakpoint_hit_on_an_armed_board_clears_moe_records_the_time_and_stops_with_the_reason() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    assert_eq!(ask(&mut g, "Z1,8000100,2"), "OK");
    assert_eq!(ask(&mut g, "Z0,8000202,2"), "OK");
    assert_eq!(
        ask(&mut g, "Z1,20000000,2"),
        "E16",
        "SRAM is outside the patch unit"
    );
    let b = sim(&mut g);
    assert_eq!(b.fp_comp[0], 1 << 30 | 0x0800_0100 | 1);
    assert_eq!(
        b.fp_comp[1],
        2 << 30 | 0x0800_0200 | 1,
        "the upper halfword"
    );

    assert!(ask_raw(&mut g, b"c", 3).is_empty());
    let b = sim(&mut g);
    b.cchp = 1 << 15 | 0x1C; // the firmware armed
    assert!(!b.arrive(0x0800_0104), "no comparator there");
    assert!(b.arrive(0x0800_0100));
    pass(&mut g);
    let stop = String::from_utf8(replies(&mut g).pop().unwrap()).unwrap();
    assert!(
        stop.starts_with("T05") && stop.contains("hwbreak"),
        "{stop}"
    );
    assert_eq!(
        sim(&mut g).cchp,
        0x1C,
        "MOE cleared at the halt, the dead time kept"
    );
    let us = g.parts().1.gdb_status().moe_clear_us;
    assert!(us > 0 && us < 500, "{us} us from the poll to the clear");
    assert!(monitor(&mut g, "counters").contains(&std::format!("halt to MOE clear: {us} us")));

    // ESP_INFO carries both.
    let mut info = None;
    let req = encode(&Msg::InfoReq, 0x80);
    let (port, svc) = g.parts();
    let me = Identity {
        addr: 0x03,
        store_mounted: true,
    };
    svc.handle(&Pdu::decode(&req).unwrap(), me, port, &mut |_, m| {
        if let Msg::Info(i) = m {
            info = Some(*i);
        }
    });
    let info = info.unwrap();
    assert!(info.gdb);
    assert_eq!(info.moe_clear_us, us);

    // The software breakpoint reports as one.
    assert!(ask_raw(&mut g, b"c", 3).is_empty());
    assert!(sim(&mut g).arrive(0x0800_0202));
    pass(&mut g);
    let stop = String::from_utf8(replies(&mut g).pop().unwrap()).unwrap();
    assert!(stop.contains("swbreak"), "{stop}");
    assert_eq!(ask(&mut g, "z0,8000202,2"), "OK");
    assert_eq!(sim(&mut g).fp_comp[1], 0);
}

#[test]
fn a_watchpoint_stops_with_its_address() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    assert_eq!(ask(&mut g, "Z2,20000400,4"), "OK");
    assert_eq!(
        ask(&mut g, "Z2,20000401,4"),
        "E16",
        "not aligned to its size"
    );
    let b = sim(&mut g);
    assert_eq!(b.dwt[0], [0x2000_0400, 2, 0b0110]);
    assert_ne!(b.dwt_trcena(), 0, "the DWT powered");
    assert!(ask_raw(&mut g, b"c", 3).is_empty());
    assert!(
        !sim(&mut g).touch(0x2000_0400, false),
        "a read does not trip a write watch"
    );
    assert!(sim(&mut g).touch(0x2000_0402, true));
    pass(&mut g);
    let stop = String::from_utf8(replies(&mut g).pop().unwrap()).unwrap();
    assert!(stop.contains("watch:20000400"), "{stop}");
}

#[test]
fn a_step_advances_the_pc_one_instruction_with_interrupts_masked_then_unmasked() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    sim(&mut g).regs[15] = 0x0800_0130;
    sim(&mut g).cchp = 1 << 15;
    let r = ask_raw(&mut g, b"vCont;s", 5);
    let stop = String::from_utf8(r.last().unwrap().clone()).unwrap();
    assert!(stop.starts_with("S05"), "{stop}");
    let b = sim(&mut g);
    assert_eq!((b.regs[15], b.steps), (0x0800_0132, 1));
    assert!(b.maskints, "masked for the step");
    assert_eq!(b.cchp, 0, "a step goes through disarm");
    assert_eq!(ask(&mut g, "pf"), hex_le(0x0800_0132));
    assert!(ask_raw(&mut g, b"c", 3).is_empty());
    assert!(!sim(&mut g).maskints, "unmasked before the resume");
}

#[test]
fn an_interrupt_halts_through_disarm() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    assert!(ask_raw(&mut g, b"c", 3).is_empty());
    sim(&mut g).cchp = 1 << 15;
    g.push(&[0x03]);
    pass(&mut g);
    let stop = String::from_utf8(replies(&mut g).pop().unwrap()).unwrap();
    assert!(stop.starts_with("S02"), "{stop}");
    let b = sim(&mut g);
    assert!(b.halted);
    assert_eq!(b.cchp, 0);
    assert_eq!(
        g.parts().1.gdb_status().moe_clear_us,
        espsvc::MOE_CLEAR_NONE,
        "the halt's own disarm, not a halt found by the poll"
    );
}

/// `load`: erase the blocks, write the image in packets, finish; the reply to `vFlashDone`.
fn gdb_load(g: &mut SimGdb<'_>, img: &[u8]) -> String {
    let blocks = img.len().div_ceil(1024) * 1024;
    assert_eq!(
        ask(g, &std::format!("vFlashErase:08000000,{blocks:x}")),
        "OK"
    );
    for (k, c) in img.chunks(1000).enumerate() {
        let mut p = std::format!("vFlashWrite:{:x}:", 0x0800_0000 + 1000 * k).into_bytes();
        p.extend(escape(c));
        let r = ask_raw(g, &p, 50);
        assert_eq!(r.last().map(|v| v.as_slice()), Some(&b"OK"[..]));
    }
    // Nothing touched the board yet.
    let r = ask_raw(g, b"vFlashDone", 10_000);
    String::from_utf8(r.last().cloned().unwrap()).unwrap()
}

#[test]
fn a_load_of_three_pages_ends_in_flash_halted_at_the_reset_vector() {
    let (mut port, _fw, mut svc) = gdb_board();
    let store: Vec<u16> = (0..1024u32).map(|i| i as u16 ^ 0x5A5A).collect();
    port.swd().pins().bus.flash[31 * 1024..].copy_from_slice(&store);
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    let img = image(3 * 1024);
    assert_eq!(gdb_load(&mut g, &img), "OK");
    let b = sim(&mut g);
    assert_eq!(&b.flash_bytes()[..img.len()], &img[..]);
    assert_eq!(
        &b.flash[31 * 1024..],
        &store[..],
        "the store pages are untouched"
    );
    assert_eq!(b.store_writes, 0);
    assert_eq!(b.loader_runs, 2);
    assert!(
        b.halted,
        "held at the reset vector for `monitor reset` or `continue`"
    );
    assert_eq!(b.regs[15], 0x0800_0108, "the image's reset handler");
    assert_eq!(ask(&mut g, "pf"), hex_le(0x0800_0108));
    assert!(monitor(&mut g, "reset").contains("the core runs"));
    assert!(!sim(&mut g).halted);
}

#[test]
fn a_bad_verify_fails_vflashdone_and_the_store_is_refused() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    assert_eq!(
        ask(&mut g, "vFlashErase:0800f800,400"),
        "E05",
        "the store's pages"
    );
    assert_eq!(
        ask(&mut g, "vFlashErase:0800f400,800"),
        "E05",
        "reaching into them"
    );
    sim(&mut g).bad_cell = Some(0x0800_0805);
    let reply = gdb_load(&mut g, &image(3 * 1024));
    assert_eq!(
        reply,
        std::format!("E{:02x}", Status::VerifyMismatch as u8),
        "the verify mismatch fails vFlashDone"
    );
}

#[test]
fn a_terminal_flash_is_refused_while_gdb_holds_the_core_halted() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    let begin = encode(
        &Msg::FlashBegin {
            len: 1024,
            sha256: [0; 32],
        },
        0x80,
    );
    let mut status = None;
    let (port, svc) = g.parts();
    let me = Identity {
        addr: 0,
        store_mounted: true,
    };
    svc.handle(&Pdu::decode(&begin).unwrap(), me, port, &mut |_, m| {
        if let Msg::Result { status: s, .. } = m {
            status = Some(*s);
        }
    });
    assert_eq!(status, Some(Status::Busy));
}

fn xfer(g: &mut SimGdb<'_>, object: &str, annex: &str) -> String {
    let mut text = String::new();
    loop {
        let r = ask(
            g,
            &std::format!("qXfer:{object}:read:{annex}:{:x},400", text.len()),
        );
        text += &r[1..];
        if r.starts_with('l') {
            return text;
        }
    }
}

#[test]
fn the_memory_map_and_the_target_description_parse_with_the_right_ranges_and_registers() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    let map = xfer(&mut g, "memory-map", "");
    let doc = roxmltree::Document::parse(&map).unwrap();
    let num = |s: &str| u64::from_str_radix(s.trim_start_matches("0x"), 16).unwrap();
    let regions: Vec<(String, u64, u64)> = doc
        .descendants()
        .filter(|n| n.has_tag_name("memory"))
        .map(|n| {
            (
                n.attribute("type").unwrap().to_string(),
                num(n.attribute("start").unwrap()),
                num(n.attribute("length").unwrap()),
            )
        })
        .collect();
    assert!(regions.contains(&(
        "flash".into(),
        0x0800_0000,
        (STORE_START - FLASH_BASE) as u64
    )));
    assert!(regions.contains(&("ram".into(), 0x2000_0000, 0xE000_0000)));
    let flash = doc
        .descendants()
        .find(|n| n.attribute("type") == Some("flash"))
        .unwrap();
    let block = flash
        .children()
        .find(|n| n.has_tag_name("property"))
        .unwrap();
    assert_eq!(block.attribute("name"), Some("blocksize"));
    assert_eq!(num(block.text().unwrap()), PAGE as u64);
    // The regions tile the address space without overlapping.
    let mut sorted = regions.clone();
    sorted.sort_by_key(|r| r.1);
    let mut at = 0u64;
    for (_, start, len) in &sorted {
        assert_eq!(*start, at);
        at += len;
    }
    assert_eq!(at, 1 << 32);

    let xml = xfer(&mut g, "features", "target.xml");
    let doc = roxmltree::Document::parse(&xml).unwrap();
    let regs: Vec<(&str, u32)> = doc
        .descendants()
        .filter(|n| n.has_tag_name("reg"))
        .map(|n| {
            (
                n.attribute("name").unwrap(),
                n.attribute("bitsize").unwrap().parse().unwrap(),
            )
        })
        .collect();
    let names: Vec<&str> = regs.iter().map(|r| r.0).collect();
    assert_eq!(
        names,
        [
            "r0",
            "r1",
            "r2",
            "r3",
            "r4",
            "r5",
            "r6",
            "r7",
            "r8",
            "r9",
            "r10",
            "r11",
            "r12",
            "sp",
            "lr",
            "pc",
            "xpsr",
            "msp",
            "psp",
            "primask",
            "basepri",
            "faultmask",
            "control"
        ]
    );
    let bytes: u32 = regs.iter().map(|r| r.1 / 8).sum();
    assert_eq!(bytes, 80, "what `g` carries");
    assert_eq!(xml, gdb::TARGET_XML);
}

#[test]
fn monitor_id_armed_and_disarm_answer() {
    let (mut port, _fw, mut svc) = gdb_board();
    let mut g = Gdb::new(&mut port, &mut svc);
    open(&mut g);
    let id = monitor(&mut g, "id");
    assert!(id.contains("dpidr   0x1ba01477"), "{id}");
    assert!(id.contains("family  GD32F1x0"), "{id}");
    assert!(id.contains("flash   64 KiB"), "{id}");
    assert_eq!(monitor(&mut g, "armed"), "disarmed: MOE clear\n");
    sim(&mut g).cchp = 1 << 15;
    assert_eq!(monitor(&mut g, "armed"), "armed: MOE set\n");
    assert_eq!(monitor(&mut g, "disarm"), "disarmed: MOE clear\n");
    assert!(monitor(&mut g, "protection").contains("read protection off (0xa5)"));
    assert!(monitor(&mut g, "reset halt").contains("halted at the reset vector"));
    assert!(sim(&mut g).halted);
    assert!(monitor(&mut g, "frobnicate").contains("monitor commands:"));
}

// ---- no target (`specs/hoverboard-bridge.md`, "No target") ----

/// The connect retry interval with no target answering.
const RETRY_US: u64 = 500_000;
/// The firmware's yield between service passes (`main.rs`).
const YIELD_US: u64 = 200;

/// A bridge's service loop with a console guest on USB, on a board whose line can go dead and come
/// back. The board's firmware runs its end of the mailbox on every pass.
struct NoTargetRig {
    port: MailboxPort<SimPins<Gd32>>,
    fw: Firmware,
    svc: Service<'static, SimPlatform>,
    router: Router,
    node: Node<'static, RamFlash>,
    guest: Guest,
    /// Every frame that reached the guest.
    usb: Vec<Vec<u8>>,
    /// The simulated time of each pass that tried the SWD connect (the engine woke the line).
    attempts: Vec<u64>,
    /// When the last pass began.
    pass_at: u64,
}

impl NoTargetRig {
    fn new(dead: bool) -> Self {
        let (port, fw) = booted(
            Family::F1x0,
            Inject {
                dead,
                ..Inject::default()
            },
        );
        NoTargetRig {
            port,
            fw,
            svc: service(),
            router: Router::new(),
            node: esp_node(),
            guest: Guest::on(Port::Usb),
            usb: Vec::new(),
            attempts: Vec::new(),
            pass_at: 0,
        }
    }

    fn now(&mut self) -> u64 {
        self.port.swd().now_us()
    }

    fn set_dead(&mut self, dead: bool) {
        self.port.swd().pins().inject.dead = dead;
    }

    fn reconnects(&mut self) -> u32 {
        self.port.swd().counters().reconnects
    }

    /// One service-loop pass as the firmware runs it (the node's pump, then the yield), the board's
    /// firmware pass, and the guest's turn at what reached it.
    fn pass(&mut self) {
        let t = self.now();
        self.pass_at = t;
        let woke = self.port.swd().pins().switches;
        self.node
            .pump(&mut self.router, &mut self.port, &mut self.svc);
        if self.port.swd().pins().switches != woke {
            self.attempts.push(t);
        }
        self.port.swd().pins().sleep_us(YIELD_US);
        self.fw.poll();
        for f in received(&mut self.router, Port::Usb) {
            if let Some(r) = self.guest.ctrl.reply_to_probe(&f) {
                self.router.on_bytes(Port::Usb, &stream(&[r.to_vec()]));
            } else if !self.guest.ctrl.is_complete() {
                self.guest.ctrl.on_reply(&f);
            } else {
                self.guest.inbox.push(f.clone());
            }
            self.usb.push(f);
        }
    }

    /// Passes until the simulated clock reaches `t`.
    fn run_until(&mut self, t: u64) {
        while self.now() < t {
            self.pass();
        }
    }

    /// Passes until `done`, for at most `limit_us` of simulated time; whether it came.
    fn run_while(&mut self, limit_us: u64, mut done: impl FnMut(&mut Self) -> bool) -> bool {
        let end = self.now() + limit_us;
        while self.now() < end {
            if done(self) {
                return true;
            }
            self.pass();
        }
        done(self)
    }

    /// The guest walks what it can reach through the USB console.
    fn walk(&mut self) {
        let ok = self.run_while(5_000_000, |r| {
            if let Some(req) = r.guest.ctrl.next_request() {
                r.router.on_bytes(Port::Usb, &stream(&[req.to_vec()]));
            }
            r.guest.ctrl.is_complete()
        });
        assert!(ok, "the walk did not complete");
    }

    /// `ESP_INFO` asked over the TCP console.
    fn esp_info(&mut self) -> espsvc::EspInfo {
        self.router
            .on_bytes(Port::Tcp, &stream(&[encode(&Msg::InfoReq, 0x81)]));
        self.pass();
        let r = received(&mut self.router, Port::Tcp);
        r.iter()
            .find_map(|f| match Msg::decode(&Pdu::decode(f).unwrap()) {
                Ok(Msg::Info(i)) => Some(i),
                _ => None,
            })
            .expect("an ESP_INFO")
    }
}

/// The gaps between consecutive connect attempts are each one retry interval, to within a pass.
fn assert_one_interval_apart(attempts: &[u64]) {
    for w in attempts.windows(2) {
        let gap = w[1] - w[0];
        assert!(
            gap > RETRY_US && gap <= RETRY_US + 2 * YIELD_US,
            "attempts {gap} us apart: {attempts:?}"
        );
    }
}

#[test]
fn with_no_target_the_bridge_tries_the_connect_once_per_interval_and_serves_its_consoles() {
    let mut r = NoTargetRig::new(true);
    r.run_until(1_000_000);

    // The console's first contact is granted, and the guest's walk finds the bridge alone: its
    // mailbox port reads empty.
    r.walk();
    assert!(r.guest.ctrl.granted().is_some());
    let me = r.node.addr();
    assert_ne!(me, NO_ADDRESS);
    assert_eq!(r.guest.ctrl.assigned_addrs().to_vec(), [me]);
    let ports = r
        .usb
        .iter()
        .map(|f| Pdu::decode(f).unwrap())
        .find(|p| p.known() == Some(Opcode::Ports) && p.src == me)
        .expect("the bridge's PORTS");
    assert_eq!(
        ports.payload[5..7],
        [1, net::walk::PORT_SWD],
        "port 1 is the mailbox"
    );
    assert_eq!(
        ports.payload[7] & net::walk::NB_STATE_MASK,
        net::walk::NB_EMPTY,
        "nothing on the mailbox port"
    );

    // ESP_INFO, on the other console, reports no target and counts one reconnect per attempt.
    r.run_until(5_000_000);
    let attempts = r.attempts.len() as u32;
    let i = r.esp_info();
    assert_eq!(i.mailbox, espsvc::MailboxState::NoTarget as u8);
    assert_eq!(i.reconnects, attempts);

    // One attempt per interval from the first pass on: 5 s holds 10 of them, 500 ms apart.
    let t = r.now();
    assert_eq!(r.attempts[0], 0, "the first pass tries at once");
    assert_eq!(attempts as u64, t / RETRY_US, "{:?}", r.attempts);
    assert_one_interval_apart(&r.attempts);
    assert_eq!(r.port.swd().pins().requests, 0, "nothing ever answered");
}

#[test]
fn a_board_that_appears_is_found_within_one_interval_and_the_mailbox_opens() {
    let mut r = NoTargetRig::new(true);
    r.run_until(1_700_000);
    assert_eq!(r.port.state(), espsvc::MailboxState::NoTarget);
    let before = r.attempts.len();

    // The board answers from 1.7 s on; the next attempt finds it and the mailbox goes up.
    let t = r.now();
    r.set_dead(false);
    let up = r.run_while(RETRY_US + 2 * YIELD_US, |r| {
        r.port.state() == espsvc::MailboxState::Up
    });
    assert!(up, "the mailbox did not open within one interval");
    assert_eq!(r.attempts.len(), before + 1, "found by the first attempt");
    assert!(r.attempts[before] - t <= RETRY_US);

    // It carries frames, and the line stays connected: no more attempts.
    r.fw.link.send(&cyclic_state(0x01)).unwrap();
    r.router.on_bytes(Port::Usb, &[]);
    r.run_until(t + 3_000_000);
    assert_eq!(r.port.state(), espsvc::MailboxState::Up);
    assert!(r.usb.iter().any(|f| f == &cyclic_state(0x01)));
    assert_eq!(r.attempts.len(), before + 1);
}

#[test]
fn a_board_that_goes_away_mid_session_is_retried_once_per_interval_and_found_again() {
    let mut r = NoTargetRig::new(false);
    r.run_until(200_000);
    assert_eq!(r.port.state(), espsvc::MailboxState::Up);
    assert_eq!(r.attempts.len(), 1, "the boot connect");

    // The board loses power: the header check finds it gone and the port closes.
    r.set_dead(true);
    let lost = r.run_while(200_000, |r| {
        r.port.state() == espsvc::MailboxState::NoTarget
    });
    assert!(lost);
    let lost_at = r.pass_at;
    let first = r.attempts.len();
    let counted = r.reconnects();

    // From then on, one connect attempt per interval, each counted once.
    r.run_until(lost_at + 10 * (RETRY_US + 2 * YIELD_US));
    let since = r.attempts[first..].to_vec();
    assert_eq!(since.len(), 10, "{since:?}");
    let gap = since[0] - lost_at;
    assert!(gap > RETRY_US && gap <= RETRY_US + 2 * YIELD_US, "{gap} us");
    assert_one_interval_apart(&since);
    assert_eq!(r.reconnects() - counted, since.len() as u32);
    assert_eq!(r.esp_info().mailbox, espsvc::MailboxState::NoTarget as u8);

    // Powered again: found by the next attempt, and the session reopens.
    let t = r.now();
    r.set_dead(false);
    let up = r.run_while(RETRY_US + 2 * YIELD_US, |r| {
        r.port.state() == espsvc::MailboxState::Up
    });
    assert!(up, "the mailbox did not reopen within one interval");
    assert!(r.attempts.last().unwrap() - t <= RETRY_US);
}

#[test]
fn a_commanded_connect_tries_for_500_ms_and_the_background_retry_makes_one_short_attempt() {
    // The engine: one wake for the background retry, the whole window for a command.
    let mut s = board(
        Family::F1x0,
        Inject {
            dead: true,
            ..Inject::default()
        },
        1,
    );
    let t0 = s.now_us();
    assert_eq!(
        s.connect_background(),
        Err(SwdError::Exhausted(XferError::NoAck))
    );
    assert!(s.now_us() - t0 < 1_000, "took {} us", s.now_us() - t0);
    assert_eq!(s.pins().switches, 1, "one wake");
    assert_eq!(s.counters().reconnects, 1);
    let t0 = s.now_us();
    assert_eq!(s.connect(), Err(SwdError::Exhausted(XferError::NoAck)));
    let took = s.now_us() - t0;
    assert!((500_000..501_000).contains(&took), "took {took} us");
    assert_eq!(s.counters().reconnects, 2);

    // The service loop: with no target, no pass spends more than a wake on the line, and a
    // TARGET_ID asked over the TCP console spends the window.
    let mut r = NoTargetRig::new(true);
    let mut longest = 0;
    while r.now() < 2_000_000 {
        let t = r.now();
        r.pass();
        longest = longest.max(r.now() - t);
    }
    assert_eq!(r.attempts.len(), 4);
    assert!(longest < 1_000, "a pass took {longest} us");
    r.router
        .on_bytes(Port::Tcp, &stream(&[encode(&Msg::TargetIdReq, 0x81)]));
    let t = r.now();
    r.pass();
    let took = r.now() - t;
    assert!(
        (500_000..520_000).contains(&took),
        "TARGET_ID took {took} us"
    );
    let reply = received(&mut r.router, Port::Tcp);
    assert!(reply.iter().any(|f| matches!(
        Msg::decode(&Pdu::decode(f).unwrap()),
        Ok(Msg::Result { .. })
    )));
}

#[test]
fn the_esp_answers_a_default_read_un_stores_its_name_and_compacts() {
    // `specs/cli-config.md` sections 6.1 to 6.3, on the bridge's own `Responder` and store.
    use crate::node::{advertised_name, BRIDGE_NAME_MAX};
    let mut flash = RamFlash::erased(4096);
    {
        let mut m = Rig::new(&mut flash);
        m.walk(0);
        // `link_set` is not stored on the bridge: a successful read of the default.
        let r = m.config_read(0x03, store::LINK_SET.key());
        assert_eq!(r[2..], [net::walk::CFG_DEFAULT, store::Type::U8.tag(), 0]);

        let mut w = vec![store::DEVICE_NAME.id(), 0, store::Type::Str.tag()];
        w.extend_from_slice(b"rover-esp");
        let r = m.ask(0, Opcode::ConfigWrite.to_u8(), 0x03, &w);
        assert_eq!(Pdu::decode(&r[0]).unwrap().payload[2], net::walk::CFG_OK);
        assert_eq!(
            m.config_read(0x03, store::DEVICE_NAME.key())[2],
            net::walk::CFG_OK
        );

        // Un-store it: the tombstone tag and no value.
        let unstore = [store::DEVICE_NAME.id(), 0, store::ERASED_TAG];
        let r = m.ask(0, Opcode::ConfigWrite.to_u8(), 0x03, &unstore);
        assert_eq!(Pdu::decode(&r[0]).unwrap().payload[2], net::walk::CFG_OK);
        let r = m.config_read(0x03, store::DEVICE_NAME.key());
        assert_eq!(r[2], net::walk::CFG_DEFAULT);

        // Compact: the superseded and un-stored records leave the bridge's store.
        let r = m.ask(0, Opcode::ConfigCompact.to_u8(), 0x03, &[]);
        assert_eq!(
            Pdu::decode(&r[0]).unwrap().payload,
            [0, 0, net::walk::CFG_OK, 0]
        );
    }
    let mut s = store::Store::mount(&mut flash).unwrap();
    let mut names = 0;
    s.for_each_in(&[store::DEVICE_NAME.key()], |_, _, _| names += 1);
    assert_eq!(names, 0, "no record of the name is left");
    // The next boot advertises the default name again and writes it into the store.
    let mut buf = [0; BRIDGE_NAME_MAX];
    let n = advertised_name(&mut s, [0, 0, 0, 0, 0xAB, 0x12], &mut buf);
    assert_eq!(&buf[..n], b"hb-bridge-ab12");
    assert_eq!(s.get_text(store::DEVICE_NAME), b"hb-bridge-ab12");
}

#[test]
fn the_boot_name_falls_back_for_a_stored_name_it_cannot_advertise() {
    use crate::node::{advertised_name, BRIDGE_NAME_MAX};
    let mac = [0, 0, 0, 0, 0xAB, 0x12];
    let boot = |flash: &mut RamFlash| {
        let mut s = store::Store::mount(flash).unwrap();
        let mut buf = [0; BRIDGE_NAME_MAX];
        let n = advertised_name(&mut s, mac, &mut buf);
        (buf[..n].to_vec(), s.get_text(store::DEVICE_NAME).to_vec())
    };

    // Nothing written: the default, and the store now holds it.
    let mut flash = RamFlash::erased(4096);
    assert_eq!(
        boot(&mut flash),
        (b"hb-bridge-ab12".to_vec(), b"hb-bridge-ab12".to_vec())
    );

    // A usable name is advertised.
    store::Store::mount(&mut flash)
        .unwrap()
        .set_str(store::DEVICE_NAME, "rover-esp")
        .unwrap();
    assert_eq!(
        boot(&mut flash),
        (b"rover-esp".to_vec(), b"rover-esp".to_vec())
    );

    // A store written before the refusal existed: the default is advertised, the store keeps it.
    store::Store::mount(&mut flash)
        .unwrap()
        .set_str(store::DEVICE_NAME, "Offroad master")
        .unwrap();
    assert_eq!(
        boot(&mut flash),
        (b"hb-bridge-ab12".to_vec(), b"Offroad master".to_vec())
    );
}
