//! `hoverboard-cli`'s client against the ESP's command service over an in-memory link
//! (`specs/esp-tool.md`, "Verification", ESP commands): the ESP side is the real router, mailbox
//! port and service on the simulated GD32, and the link drops, duplicates and reorders chunks.

use std::collections::VecDeque;
use std::time::Duration;

use esp_tool_core::ble::{Router, NOTIFY_MAX};
use esp_tool_core::mailbox::MailboxPort;
use esp_tool_core::sim::{board, Family, Gd32, Inject, SimPins};
use esp_tool_core::svc::{Build, ImageBuffer, Service};
use espsvc::{Ctrl, Stage, Status, OP_FLASH_CHUNK};
use hoverboard_cli::client::{Client, ClientError, Progress};
use hoverboard_cli::wire::Wire;
use link::StreamFramer;
use sha2::{Digest, Sha256};

/// The ESP, as the firmware's service loop runs it.
struct Esp {
    router: Router,
    port: MailboxPort<SimPins<Gd32>>,
    svc: Service<'static>,
}

impl Esp {
    fn new(family: Family) -> Self {
        let rx: &'static mut ImageBuffer = Box::leak(Box::new(ImageBuffer::new()));
        let mut router = Router::new();
        router.connect();
        Esp {
            router,
            port: MailboxPort::new(board(family, Inject::default(), 5)),
            svc: Service::new(
                Build {
                    version: [0, 1, 0],
                    commit: 0xd31c5bb0,
                    swd_khz: 1000,
                },
                rx,
            ),
        }
    }
    fn model(&mut self) -> &mut Gd32 {
        &mut self.port.swd().pins().bus
    }
}

/// A deterministic generator.
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

/// The in-memory link: PDUs framed onto the ESP's BLE byte stream, notifications framed back.
/// Chunks are dropped, duplicated and reordered on the way in; acknowledgements are dropped on the
/// way out.
struct TestWire {
    esp: Esp,
    rng: Lcg,
    faults: bool,
    pending: Vec<Vec<u8>>,
    inbox: VecDeque<Vec<u8>>,
    framer: StreamFramer,
}

impl TestWire {
    fn new(esp: Esp, seed: u64, faults: bool) -> Self {
        TestWire {
            esp,
            rng: Lcg(seed),
            faults,
            pending: Vec::new(),
            inbox: VecDeque::new(),
            framer: StreamFramer::new(),
        }
    }

    /// Hand the ESP everything sent, in a shuffled order, split at random points, and run it.
    fn deliver(&mut self) {
        if self.faults && self.pending.len() > 1 {
            for _ in 0..self.pending.len() / 2 {
                let i = self.rng.next() as usize % self.pending.len();
                let j = self.rng.next() as usize % self.pending.len();
                self.pending.swap(i, j);
            }
        }
        let mut bytes = Vec::new();
        for p in self.pending.drain(..) {
            let mut l2 = vec![0u8]; // a single-fragment frag-hdr
            l2.extend_from_slice(&p);
            let mut buf = [0u8; 300];
            let n = link::encode_stream_frame(&l2, &mut buf).unwrap();
            bytes.extend_from_slice(&buf[..n]);
        }
        let mut rest = &bytes[..];
        while !rest.is_empty() {
            let k = (1 + self.rng.next() as usize % 64).min(rest.len());
            self.esp.router.on_write(&rest[..k]);
            rest = &rest[k..];
            self.pump();
        }
        self.pump();
    }

    fn pump(&mut self) {
        let e = &mut self.esp;
        e.router.pump(&mut e.port, &mut e.svc);
        let mut n = [0u8; NOTIFY_MAX];
        loop {
            let k = e.router.next_notification(&mut n);
            if k == 0 {
                break;
            }
            let inbox = &mut self.inbox;
            self.framer
                .feed(&n[..k], &mut |f| inbox.push_back(f[1..].to_vec()));
        }
    }
}

impl Wire for TestWire {
    fn send(&mut self, pdu: &[u8]) {
        if self.faults && pdu[0] == OP_FLASH_CHUNK {
            match self.rng.next() % 10 {
                0 => return,                          // dropped
                1 => self.pending.push(pdu.to_vec()), // duplicated
                _ => {}
            }
        }
        self.pending.push(pdu.to_vec());
    }

    fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
        self.deliver();
        loop {
            let p = self.inbox.pop_front()?;
            if self.faults && p[0] == espsvc::OP_FLASH_ACK && self.rng.next().is_multiple_of(8) {
                continue; // an acknowledgement lost on the way back
            }
            return Some(p);
        }
    }
}

fn image(len: usize) -> Vec<u8> {
    let mut v: Vec<u8> = (0..len as u32).map(|i| (i * 13 + (i >> 7)) as u8).collect();
    v[..8].copy_from_slice(&[0x00, 0x20, 0x00, 0x20, 0x09, 0x01, 0x00, 0x08]);
    v
}

#[test]
fn a_flash_with_dropped_duplicated_and_reordered_chunks_reaches_the_same_sha256() {
    for (seed, family) in [(1, Family::F1x0), (2, Family::F10x), (3, Family::F1x0)] {
        let mut esp = Esp::new(family);
        let store: Vec<u16> = (0..1024u16).map(|i| i ^ 0xA55A).collect();
        esp.model().flash[31 * 1024..].copy_from_slice(&store);
        let img = image(4 * 1024 + 333);
        let mut c = Client::new(TestWire::new(esp, seed, true));
        let mut stages = Vec::new();
        c.flash(&img, |p: Progress| {
            if stages.last() != Some(&p.stage) {
                stages.push(p.stage);
            }
        })
        .unwrap();
        assert_eq!(
            stages,
            [
                Stage::Receiving,
                Stage::Erasing,
                Stage::Programming,
                Stage::Verifying,
                Stage::Booted
            ]
        );
        let g = c.wire().esp.model();
        let flashed = g.flash_bytes();
        assert_eq!(
            Sha256::digest(&flashed[..img.len()]),
            Sha256::digest(&img),
            "seed {seed}"
        );
        assert_eq!(&g.flash[31 * 1024..], &store[..]);
        assert_eq!(g.mailbox().magic(), swd_mailbox::MAGIC, "booted");
    }
}

#[test]
fn info_id_memory_and_target_control_round_trip() {
    let mut c = Client::new(TestWire::new(Esp::new(Family::F10x), 9, false));
    let info = c.info().unwrap();
    assert_eq!((info.version, info.swd_khz), ([0, 1, 0], 1000));
    let id = c.target_id().unwrap();
    assert_eq!(
        (id.dpidr, id.family, id.rdp),
        (0x1BA0_1477, espsvc::FAMILY_F10X, 0xA5)
    );

    let data: Vec<u8> = (0..120).collect();
    c.write(0x2000_0400, &data).unwrap();
    assert_eq!(c.read(0x2000_0402, 117).unwrap(), &data[2..119]);

    c.wire().esp.model().cchp = 1 << 15;
    c.ctrl(Ctrl::Halt).unwrap();
    assert!(c.wire().esp.model().halted);
    assert_eq!(c.wire().esp.model().cchp, 0);
    c.ctrl(Ctrl::Resume).unwrap();
    assert!(!c.wire().esp.model().halted);

    c.wire().esp.model().moe_stuck = true;
    c.wire().esp.model().cchp = 1 << 15;
    assert_eq!(c.ctrl(Ctrl::Halt), Err(ClientError::Status(Status::Armed)));
}

#[test]
fn a_protected_board_is_refused_at_begin_and_cleared_by_clear_protection() {
    let mut esp = Esp::new(Family::F1x0);
    esp.model().protect();
    let mut c = Client::new(TestWire::new(esp, 4, false));
    assert_eq!(
        c.flash(&image(2000), |_| {}),
        Err(ClientError::Status(Status::Protected))
    );
    let user = c.target_id().unwrap().user;
    c.clear_protection().unwrap();
    c.wire().esp.model().power_cycle();
    c.wire().esp.port.swd().forget();
    let id = c.target_id().unwrap();
    assert_eq!(
        (id.rdp, id.user, id.clear_check),
        (0xA5, user, espsvc::CLEAR_PASS)
    );
    c.flash(&image(2000), |_| {}).unwrap();
}
