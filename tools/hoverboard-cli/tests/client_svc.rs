//! `hoverboard-cli`'s client against the bridge's command service over an in-memory link
//! (`specs/hoverboard-bridge.md`, "Verification", ESP commands): the bridge side is the real links, node,
//! mailbox port and service on the simulated GD32, and the link drops, duplicates and reorders chunks. The
//! same bridge also answers on its console through a local TCP socket, as the firmware's Wi-Fi console
//! serves it, reached by the `--tcp` link, and the built binary runs its bulk commands (a
//! multi-chunk read, a flash from an ELF) over that link. The BLE-shaped link keeps
//! [`espsvc::WINDOW_BLE`] chunks in flight and the TCP link [`espsvc::WINDOW_STREAM`].

mod common;

use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::net::TcpListener;
use std::process::{Command, Stdio};
use std::time::Duration;
use std::time::Instant;

use espsvc::{Ctrl, Msg, Stage, Status, OP_FLASH_CHUNK};
use hoverboard_bridge_core::mailbox::MailboxPort;
use hoverboard_bridge_core::node::Node;
use hoverboard_bridge_core::router::{Port, Router, NOTIFY_MAX};
use hoverboard_bridge_core::sim::{board, Family, Gd32, Inject, RamFlash, SimPins, SimPlatform};
use hoverboard_bridge_core::svc::{Build, ImageBuffer, Service};
use hoverboard_bridge_core::update::Update;
use hoverboard_cli::client::{sample_line, Client, ClientError, Progress, StageTimes};
use hoverboard_cli::transport::Tcp;
use hoverboard_cli::wire::{ByteLink, Framed, Wire};
use link::StreamFramer;
use net::pdu::Pdu;
use sha2::{Digest, Sha256};

/// The address the simulated bridge holds from a past walk.
const BRIDGE_ADDR: u8 = 0x03;

/// The bridge, as the firmware's service loop runs it.
struct Esp {
    router: Router,
    node: Node<'static, RamFlash>,
    port: MailboxPort<SimPins<Gd32>>,
    svc: Service<'static, SimPlatform>,
}

impl Esp {
    fn new(family: Family) -> Self {
        Self::with_platform(family, SimPlatform::new())
    }

    fn with_platform(family: Family, platform: SimPlatform) -> Self {
        let rx: &'static mut ImageBuffer = Box::leak(Box::new(ImageBuffer::new()));
        let mut router = Router::new();
        router.connect(Port::Ble);
        let flash: &'static mut RamFlash = Box::leak(Box::new(RamFlash::erased(4096)));
        let mut keys = store::Store::mount(flash).unwrap();
        // The address a past walk gave the bridge, which `bridge update` names it by.
        keys.set_value(store::NODE_ADDRESS.key(), store::Value::U8(BRIDGE_ADDR))
            .unwrap();
        Esp {
            router,
            node: Node::new(
                keys,
                true,
                0x0001,
                hoverboard_bridge_core::sim::SIM_MAC,
                &hoverboard_bridge_core::sim::SIM_NAME,
            ),
            port: MailboxPort::new(board(family, Inject::default(), 5)),
            svc: Service::new(
                Build {
                    version: [0, 1, 0],
                    commit: 0xd31c5bb0,
                    swd_khz: 1000,
                },
                rx,
                Box::leak(Box::new(Update::new())),
                platform,
            ),
        }
    }

    /// The bridge resets: the bootloader picks a slot from otadata, and everything else starts over.
    fn reset(mut self) -> Self {
        let mut platform = std::mem::take(self.svc.platform());
        platform.reset();
        Esp::with_platform(Family::F1x0, platform)
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

/// The in-memory link: PDUs framed onto the bridge's BLE byte stream, notifications framed back.
/// Chunks are dropped, duplicated and reordered on the way in; acknowledgements are dropped on the
/// way out.
struct TestWire {
    esp: Esp,
    rng: Lcg,
    faults: bool,
    window: usize,
    pending: Vec<Vec<u8>>,
    inbox: VecDeque<Vec<u8>>,
    framer: StreamFramer,
}

impl TestWire {
    fn new(esp: Esp, seed: u64, faults: bool) -> Self {
        Self::with_window(esp, seed, faults, espsvc::WINDOW_BLE)
    }

    fn with_window(esp: Esp, seed: u64, faults: bool, window: usize) -> Self {
        TestWire {
            esp,
            rng: Lcg(seed),
            faults,
            window,
            pending: Vec::new(),
            inbox: VecDeque::new(),
            framer: StreamFramer::new(),
        }
    }

    /// Hand the bridge everything sent, in a shuffled order, split at random points, and run it.
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
            self.esp.router.on_bytes(Port::Ble, &rest[..k]);
            rest = &rest[k..];
            self.pump();
        }
        self.pump();
        assert_eq!(self.esp.router.rx_overflow, 0);
    }

    fn pump(&mut self) {
        let e = &mut self.esp;
        e.node.pump(&mut e.router, &mut e.port, &mut e.svc);
        let mut n = [0u8; NOTIFY_MAX];
        loop {
            let k = e.router.next_out(Port::Ble, &mut n);
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
        // The service loop runs on between replies: a flash step with nothing new to report
        // (the RAM loader still draining the ring) sends none.
        for _ in 0..10_000 {
            while let Some(p) = self.inbox.pop_front() {
                if self.faults && p[0] == espsvc::OP_FLASH_ACK && self.rng.next().is_multiple_of(8)
                {
                    continue; // an acknowledgement lost on the way back
                }
                return Some(p);
            }
            self.pump();
        }
        None
    }

    fn window(&self) -> usize {
        self.window
    }
}

fn image(len: usize) -> Vec<u8> {
    let mut v: Vec<u8> = (0..len as u32).map(|i| (i * 13 + (i >> 7)) as u8).collect();
    v[..8].copy_from_slice(&[0x00, 0x20, 0x00, 0x20, 0x09, 0x01, 0x00, 0x08]);
    v
}

#[test]
fn a_flash_with_dropped_duplicated_and_reordered_chunks_reaches_the_same_sha256() {
    let runs = [(1, Family::F1x0), (2, Family::F10x), (3, Family::F1x0)]
        .into_iter()
        .flat_map(|r| [(r, espsvc::WINDOW_BLE), (r, espsvc::WINDOW_STREAM)]);
    for ((seed, family), window) in runs {
        let mut esp = Esp::new(family);
        let store: Vec<u16> = (0..1024u16).map(|i| i ^ 0xA55A).collect();
        esp.model().flash[31 * 1024..].copy_from_slice(&store);
        let img = image(4 * 1024 + 333);
        let mut c = Client::new(TestWire::with_window(esp, seed, true, window));
        let mut stages = Vec::new();
        let mut times = StageTimes::new();
        let t0 = Instant::now();
        c.flash(&img, |p: Progress| {
            times.on(p, t0.elapsed());
            if stages.last() != Some(&p.stage) {
                stages.push(p.stage);
            }
        })
        .unwrap();
        let names: Vec<&str> = times.durations().iter().map(|d| d.0).collect();
        assert_eq!(names, ["transfer", "erase", "program", "verify", "boot"]);
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
            "seed {seed}, window {window}"
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

/// A bridge that answers each `MEM_READ` with a counter's bytes, `delay` after it was asked.
struct SlowReads {
    delay: Duration,
    pending: VecDeque<Vec<u8>>,
    reads: u32,
}

impl Wire for SlowReads {
    fn send(&mut self, pdu: &[u8]) {
        let p = Pdu::decode(pdu).unwrap();
        if let Ok(Msg::MemRead { addr, len }) = Msg::decode(&p) {
            self.reads += 1;
            let data = vec![self.reads as u8; len as usize];
            let mut buf = [0u8; espsvc::MAX_PDU];
            let n = Msg::MemData { addr, data: &data }
                .encode(p.dst, p.src, &mut buf)
                .unwrap();
            self.pending.push_back(buf[..n].to_vec());
        }
    }
    fn recv(&mut self, _timeout: Duration) -> Option<Vec<u8>> {
        let r = self.pending.pop_front()?;
        std::thread::sleep(self.delay);
        Some(r)
    }
    fn window(&self) -> usize {
        1
    }
}

/// `specs/board-route.md` section 6, test 5: `board read --hz` on one client reads every slot
/// while the bridge keeps up, and a slow bridge yields fewer samples, each a real read, never one
/// made up for a slot it missed. The line is the time, to the millisecond, and the bytes.
#[test]
fn a_streaming_read_takes_the_slots_the_bridge_keeps_up_with() {
    let run = |delay: Duration, len: usize| {
        let mut c = Client::new(SlowReads {
            delay,
            pending: VecDeque::new(),
            reads: 0,
        });
        let t0 = Instant::now();
        let mut got: Vec<(Duration, Vec<u8>)> = Vec::new();
        c.read_every(
            0x2000_0920,
            len,
            Duration::from_millis(50),
            || t0.elapsed() >= Duration::from_millis(500),
            |at, data| got.push((at, data.to_vec())),
        )
        .unwrap();
        (got, c.wire().reads)
    };
    // Fast: at most one sample per 50 ms slot, each one read. How many slots a run gets depends
    // on how far the host's sleeps overshoot (a CI runner gave 5 in 500 ms), so the count is
    // bounded above only; what is exact is that no slot holds two samples.
    let (fast, reads) = run(Duration::ZERO, 4);
    assert!((1..=11).contains(&fast.len()), "{} samples", fast.len());
    assert_eq!(reads as usize, fast.len());
    for (i, (at, data)) in fast.iter().enumerate() {
        assert!(*at >= Duration::from_millis(50) * i as u32, "{i}: {at:?}");
        assert_eq!(data, &vec![i as u8 + 1; 4]);
    }
    for w in fast.windows(2) {
        assert!(
            w[1].0.as_millis() / 50 > w[0].0.as_millis() / 50,
            "{fast:?}"
        );
    }
    // Slow: each read takes 120 ms, so it misses two slots in three; the samples are the reads,
    // each in the first slot after the last one ended.
    let (slow, reads) = run(Duration::from_millis(120), 4);
    assert!((1..=5).contains(&slow.len()), "{} samples", slow.len());
    assert_eq!(reads as usize, slow.len());
    for w in slow.windows(2) {
        assert!(w[1].0 >= w[0].0 + Duration::from_millis(120), "{slow:?}");
        assert_eq!(w[1].1[0], w[0].1[0] + 1, "no sample is made up");
    }
    // A range over MEM_MAX is the MEM_READs it needs, in one sample.
    let (wide, reads) = run(Duration::ZERO, espsvc::MEM_MAX + 4);
    assert_eq!(reads as usize, 2 * wide.len());
    assert_eq!(wide[0].1.len(), espsvc::MEM_MAX + 4);

    assert_eq!(
        sample_line(Duration::from_millis(1234), &[0x4d, 0x42, 0x00, 0xff]),
        "1.234 4d4200ff"
    );
    assert_eq!(sample_line(Duration::ZERO, &[]), "0.000 ");
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

/// The bridge's console on TCP, as the firmware's Wi-Fi console serves it: each client attaches the
/// console, whatever arrives on the socket goes into its stream and whatever it sends goes back
/// out, for `connections` clients one after the other, each until it hangs up. Returns the bridge for
/// its board's state.
fn serve_console_over_tcp(listener: TcpListener, mut esp: Esp, connections: usize) -> Esp {
    for _ in 0..connections {
        let (mut sock, _) = listener.accept().unwrap();
        esp.router.connect(Port::Tcp);
        sock.set_read_timeout(Some(Duration::from_millis(1)))
            .unwrap();
        let mut buf = [0u8; 256];
        let mut out = [0u8; 64];
        loop {
            // As the firmware's service loop: a read only while the port has room for it.
            if esp.router.room(Port::Tcp) >= buf.len() {
                match sock.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        esp.router.on_bytes(Port::Tcp, &buf[..n]);
                    }
                    Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                    Err(_) => break,
                }
            }
            esp.node.pump(&mut esp.router, &mut esp.port, &mut esp.svc);
            loop {
                let k = esp.router.next_out(Port::Tcp, &mut out);
                if k == 0 {
                    break;
                }
                if sock.write_all(&out[..k]).is_err() {
                    break;
                }
            }
        }
        esp.router.disconnect(Port::Tcp);
        assert_eq!(esp.router.rx_overflow, 0);
    }
    esp
}

#[test]
fn the_tcp_link_reaches_the_console_with_the_same_commands() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let mut esp = Esp::new(Family::F1x0);
    let store: Vec<u16> = (0..1024u16).map(|i| i ^ 0x5AA5).collect();
    esp.model().flash[31 * 1024..].copy_from_slice(&store);
    let server = std::thread::spawn(move || serve_console_over_tcp(listener, esp, 1));

    let mut c = Client::new(Framed::new(Tcp::connect(&addr).unwrap()));
    assert_eq!(c.info().unwrap().swd_khz, 1000);
    assert_eq!(c.target_id().unwrap().family, espsvc::FAMILY_F1X0);
    let data: Vec<u8> = (0..64).collect();
    c.write(0x2000_0400, &data).unwrap();
    assert_eq!(c.read(0x2000_0400, 64).unwrap(), data);
    let store_bytes: Vec<u8> = store.iter().flat_map(|h| h.to_le_bytes()).collect();
    assert_eq!(c.read(0x0800_F800, 2048).unwrap(), store_bytes);
    let img = image(3 * 1024 + 77);
    c.flash(&img, |_| {}).unwrap();
    drop(c);

    let mut esp = server.join().unwrap();
    let g = esp.model();
    assert_eq!(
        Sha256::digest(&g.flash_bytes()[..img.len()]),
        Sha256::digest(&img)
    );
    assert_eq!(&g.flash[31 * 1024..], &store[..]);
    assert_eq!(g.mailbox().magic(), swd_mailbox::MAGIC, "booted");
}

/// The TCP link with the client's view of a bulk transfer recorded: every chunk written and every
/// acknowledgement read, decoded from the stream framing.
struct Spy {
    tcp: Tcp,
    out: StreamFramer,
    back: StreamFramer,
    /// The end of the furthest chunk sent.
    sent_end: u32,
    /// The highest acknowledgement heard, and how many came.
    acked: u32,
    acks: usize,
    /// The most chunks sent and not acknowledged at any write.
    most_in_flight: u32,
}

impl Spy {
    fn new(tcp: Tcp) -> Self {
        Spy {
            tcp,
            out: StreamFramer::new(),
            back: StreamFramer::new(),
            sent_end: 0,
            acked: 0,
            acks: 0,
            most_in_flight: 0,
        }
    }
}

impl ByteLink for Spy {
    fn write(&mut self, bytes: &[u8]) {
        let mut ends = Vec::new();
        self.out.feed(bytes, &mut |f| {
            if let Ok(Msg::FlashChunk { offset, data }) =
                Msg::decode(&Pdu::decode(&f[1..]).unwrap())
            {
                ends.push(offset + data.len() as u32);
            }
        });
        for e in ends {
            self.sent_end = self.sent_end.max(e);
        }
        let in_flight = (self.sent_end - self.acked).div_ceil(espsvc::CHUNK as u32);
        self.most_in_flight = self.most_in_flight.max(in_flight);
        self.tcp.write(bytes);
    }

    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        let got = self.tcp.read(timeout);
        let mut acks = Vec::new();
        self.back.feed(&got, &mut |f| {
            if let Ok(Msg::FlashAck { contiguous }) = Msg::decode(&Pdu::decode(&f[1..]).unwrap()) {
                acks.push(contiguous);
            }
        });
        for a in acks {
            self.acked = self.acked.max(a);
            self.acks += 1;
        }
        got
    }

    fn window(&self) -> usize {
        self.tcp.window()
    }
}

#[test]
fn a_flash_over_tcp_keeps_64_chunks_in_flight_and_reaches_the_same_sha256() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let mut esp = Esp::new(Family::F1x0);
    let store: Vec<u16> = (0..1024u16).map(|i| i ^ 0x3CC3).collect();
    esp.model().flash[31 * 1024..].copy_from_slice(&store);
    let server = std::thread::spawn(move || serve_console_over_tcp(listener, esp, 1));

    let img = image(30 * 1024 + 5);
    let mut c = Client::new(Framed::new(Spy::new(Tcp::connect(&addr).unwrap())));
    c.flash(&img, |_| {}).unwrap();
    let spy = c.wire().inner();
    assert_eq!(spy.most_in_flight, espsvc::WINDOW_STREAM as u32);
    assert_eq!(spy.acked, img.len() as u32);
    let chunks = img.len().div_ceil(espsvc::CHUNK);
    assert_eq!(
        spy.acks,
        chunks.div_ceil(espsvc::ACK_EVERY as usize),
        "one acknowledgement per 8 chunks and one at the end"
    );
    drop(c);

    let mut esp = server.join().unwrap();
    let g = esp.model();
    assert_eq!(
        Sha256::digest(&g.flash_bytes()[..img.len()]),
        Sha256::digest(&img)
    );
    assert_eq!(&g.flash[31 * 1024..], &store[..]);
    assert_eq!(g.mailbox().magic(), swd_mailbox::MAGIC, "booted");
}

/// Run the built binary, with no terminal on its input; a run that outlives `limit` is killed and fails the test (a command that
/// took the BLE path on a host with no radio waits there).
fn cli(args: &[&str], limit: Duration) -> (bool, String, String) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let t0 = Instant::now();
    while child.try_wait().unwrap().is_none() {
        if t0.elapsed() > limit {
            let _ = child.kill();
            panic!("hoverboard-cli {args:?} still running after {limit:?}");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let o = child.wait_with_output().unwrap();
    (
        o.status.success(),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

/// The master's GD32 once a [`serve_machine_over_tcp`] run is over: its flash as halfwords and as
/// bytes, and whether its mailbox header is written (it booted).
struct Master {
    flash: Vec<u16>,
    bytes: Vec<u8>,
    booted: bool,
}

/// The whole simulated machine (`hoverboard_bridge_core::sim::Machine`: the bridge, the master on
/// its mailbox, the slave on the master's UART) on the bridge's TCP console, for `connections`
/// clients one after the other, with the master's store pages holding `store` first. A board's SWD
/// commands go through the bridge wired to it, which the walk finds.
fn serve_machine_over_tcp(listener: TcpListener, store: Vec<u16>, connections: usize) -> Master {
    serve_machine_with(listener, store, connections, |_| {})
}

/// What [`serve_machine_with`] does to the master's GD32 before serving.
type Prep = fn(&mut Gd32);

/// [`serve_machine_over_tcp`], with `prep` applied to the master's GD32 first (its family, its
/// flash size).
fn serve_machine_with(
    listener: TcpListener,
    store: Vec<u16>,
    connections: usize,
    prep: Prep,
) -> Master {
    use hoverboard_bridge_core::sim::Machine;
    let flash: &'static mut RamFlash = Box::leak(Box::new(RamFlash::erased(4096)));
    let svc = Service::new(
        Build {
            version: [0, 1, 0],
            commit: 0,
            swd_khz: 1000,
        },
        Box::leak(Box::new(ImageBuffer::new())),
        Box::leak(Box::new(Update::new())),
        SimPlatform::new(),
    );
    let mut m = Machine::new(flash, svc);
    m.port.swd().pins().bus.flash[31 * 1024..].copy_from_slice(&store);
    prep(&mut m.port.swd().pins().bus);
    for _ in 0..connections {
        let (mut sock, _) = listener.accept().unwrap();
        m.router.connect(Port::Tcp);
        sock.set_read_timeout(Some(Duration::from_millis(1)))
            .unwrap();
        let mut buf = [0u8; 256];
        let mut out = [0u8; 64];
        loop {
            if m.router.room(Port::Tcp) >= buf.len() {
                match sock.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => m.router.on_bytes(Port::Tcp, &buf[..n]),
                    Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                    Err(_) => break,
                }
            }
            // While a job runs the board is halted and its firmware does not run: the bridge's
            // service loop alone, as on the bench.
            let moved = if m.svc.busy() {
                m.node.pump(&mut m.router, &mut m.port, &mut m.svc);
                true
            } else {
                m.step()
            };
            loop {
                let k = m.router.next_out(Port::Tcp, &mut out);
                if k == 0 {
                    break;
                }
                let _ = sock.write_all(&out[..k]);
            }
            if !moved {
                m.tick();
            }
        }
        m.router.disconnect(Port::Tcp);
    }
    let g = &mut m.port.swd().pins().bus;
    Master {
        flash: g.flash.clone(),
        bytes: g.flash_bytes(),
        booted: g.mailbox().magic() == swd_mailbox::MAGIC,
    }
}

#[test]
fn the_binary_runs_a_multi_chunk_read_and_a_flash_through_the_boards_bridge() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let store: Vec<u16> = (0..1024u16).map(|i| i.wrapping_mul(0x9E37)).collect();
    let pages = store.clone();
    let server = std::thread::spawn(move || serve_machine_over_tcp(listener, pages, 2));
    let limit = Duration::from_secs(60);

    let (ok, out, err) = cli(
        &[
            "--tcp",
            &addr,
            "board",
            "read",
            "0x01",
            "0x0800F800",
            "2048",
        ],
        limit,
    );
    assert!(ok, "{err}");
    assert!(err.contains(&format!("connected to {addr}")), "{err}");
    let read: Vec<u8> = out
        .lines()
        .flat_map(|l| l.split_once(": ").unwrap().1.split(' '))
        .map(|h| u8::from_str_radix(h, 16).unwrap())
        .collect();
    let want: Vec<u8> = store.iter().flat_map(|h| h.to_le_bytes()).collect();
    assert_eq!(read, want);

    let img = image(5 * 1024 + 11);
    let elf = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("flash-over-tcp.elf");
    std::fs::write(&elf, common::flashable_elf(&img)).unwrap();
    let tcp = format!("--tcp={addr}");
    let (ok, out, err) = cli(
        &[&tcp, "board", "flash", "0x01", elf.to_str().unwrap()],
        limit,
    );
    assert!(ok, "{err}");
    assert!(out.starts_with("booted"), "{out}");
    let stages = out.lines().find(|l| l.starts_with("stages: ")).expect(&out);
    for name in ["transfer", "erase", "program", "verify", "boot"] {
        assert!(stages.contains(&format!("{name} ")), "{stages}");
    }

    let g = server.join().unwrap();
    let n = (common::release_guards().code_floor as usize).max(img.len());
    assert_eq!(&g.bytes[..img.len()], &img[..]);
    assert!(g.bytes[img.len()..n].iter().all(|&b| b == 0));
    assert_eq!(&g.flash[31 * 1024..], &store[..]);
    assert!(g.booted, "booted");
}

/// A bundle packed from `elf` as the universal image, with `targets` in place of its table's when
/// given.
fn bundle_file(
    name: &str,
    elf: &[u8],
    targets: Option<Vec<hoverboard_cli::bundle::Target>>,
) -> String {
    use hoverboard_cli::bundle::{image_table, pack, PackEntry};
    let mut table = image_table(
        include_str!("../../../motorcontroller/firmware/Cargo.toml"),
        "mainboard-gd32",
    )
    .unwrap();
    if let Some(t) = targets {
        table.targets = t;
    }
    let z = pack(
        &[PackEntry {
            name: "mainboard-gd32",
            table,
            guards: common::release_guards(),
            elf,
        }],
        "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd",
        "2026-11-01T17:03:58Z",
    )
    .unwrap();
    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    std::fs::write(&path, z).unwrap();
    path.to_str().unwrap().to_string()
}

#[test]
fn the_binary_flashes_the_image_a_bundle_holds_for_the_chip() {
    let img = image(5 * 1024 + 11);
    // Named .elf: the content says it is a bundle.
    let path = bundle_file("bundle-named-elf.elf", &common::flashable_elf(&img), None);
    let f10x: Prep = |g| g.family = Family::F10x;
    let f1x0: Prep = |g| g.family = Family::F1x0;
    for (prep, name) in [(f10x, "gd32f10x"), (f1x0, "gd32f1x0")] {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let server =
            std::thread::spawn(move || serve_machine_with(listener, vec![0xFFFF; 1024], 2, prep));
        let tcp = format!("--tcp={addr}");
        let limit = Duration::from_secs(60);

        // No terminal and no --yes: chosen and checked, then refused before anything is streamed.
        let (ok, out, err) = cli(&[&tcp, "board", "flash", "0x01", &path], limit);
        assert!(!ok, "{out}");
        assert!(err.contains("give --yes"), "{err}");
        let line = format!(
            "image mainboard-gd32 (mainboard) for the chip ({name} 64 KiB), from the bundle built \
             from commit c0eb25917cd904d5d9ba9307f56cb8251cbb61dd"
        );
        assert!(out.contains(&line), "{out}");

        let (ok, out, err) = cli(&[&tcp, "board", "flash", "0x01", &path, "--yes"], limit);
        assert!(ok, "{err}");
        assert!(err.contains("SHA-256 checked"), "{err}");
        assert!(out.starts_with(&line), "{out}");
        assert!(out.contains("\nbooted"), "{out}");

        let g = server.join().unwrap();
        assert_eq!(&g.bytes[..img.len()], &img[..]);
        assert!(g.booted, "booted");
    }
}

#[test]
fn a_bundle_match_the_bridge_would_refuse_streams_nothing() {
    use hoverboard_cli::bundle::{Family as F, Target};
    let img = image(1024);
    let path = bundle_file(
        "bundle-32k.hbfw",
        &common::flashable_elf(&img),
        Some(vec![Target {
            family: F::Gd32f1x0,
            flash_kib: 32,
        }]),
    );
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let prep: Prep = |g| {
        g.family = Family::F1x0;
        g.flash_kib = 32;
    };
    let server =
        std::thread::spawn(move || serve_machine_with(listener, vec![0xFFFF; 1024], 2, prep));
    let tcp = format!("--tcp={addr}");
    let limit = Duration::from_secs(30);
    let (ok, _, err) = cli(&[&tcp, "board", "flash", "0x01", &path, "--yes"], limit);
    assert!(!ok);
    assert!(err.contains("would refuse a gd32f1x0 32 KiB part"), "{err}");
    // The chip is not the image's: refused naming both.
    let other = bundle_file("bundle-64k.hbfw", &common::flashable_elf(&img), None);
    let (ok, _, err) = cli(&[&tcp, "board", "flash", "0x01", &other, "--yes"], limit);
    assert!(!ok);
    assert!(
        err.contains("no image in the bundle is for the chip (gd32f1x0 32 KiB)"),
        "{err}"
    );
    let g = server.join().unwrap();
    // Nothing streamed: the master still runs the image the simulated machine boots.
    assert!(g.booted);
    assert_ne!(&g.bytes[..img.len()], &img[..]);
}

#[test]
fn firmware_info_prints_the_manifest_and_reads_no_board() {
    let limit = Duration::from_secs(20);
    let elf = common::flashable_elf(&image(64));
    let path = bundle_file("info.hbfw", &elf, None);
    let (ok, out, err) = cli(&["firmware", "info", &path], limit);
    assert!(ok, "{err}");
    assert_eq!(
        out,
        "commit  c0eb25917cd904d5d9ba9307f56cb8251cbb61dd\nbuilt   2026-11-01T17:03:58Z\n\
         image   mainboard-gd32 (mainboard): gd32f10x 64 KiB, gd32f1x0 64 KiB; 40000 B flashed\n"
    );
    let elf_path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("info.elf");
    std::fs::write(&elf_path, &elf).unwrap();
    let (ok, _, err) = cli(&["firmware", "info", elf_path.to_str().unwrap()], limit);
    assert!(!ok && err.contains("not a firmware bundle"), "{err}");
    // --image belongs to flash, and an ELF has no images to choose from.
    let (ok, _, err) = cli(&["firmware", "info", &path, "--image", "x"], limit);
    assert!(!ok && err.contains("--image"), "{err}");
    let (ok, _, err) = cli(
        &[
            "--tcp=127.0.0.1:1",
            "board",
            "flash",
            "0x01",
            elf_path.to_str().unwrap(),
            "--image",
            "x",
        ],
        limit,
    );
    assert!(!ok && err.contains("is an ELF"), "{err}");
}

#[test]
fn a_link_option_never_falls_back_to_ble() {
    let limit = Duration::from_secs(20);
    // Nothing listens on port 1: the TCP link fails at once, with no BLE scan behind it.
    let (ok, _, err) = cli(
        &[
            "--tcp",
            "127.0.0.1:1",
            "board",
            "read",
            "0x01",
            "0x0800F800",
            "2048",
        ],
        limit,
    );
    assert!(!ok && err.contains("127.0.0.1:1"), "{err}");
    let (ok, _, err) = cli(&["--tcp=127.0.0.1:1", "bridge", "info", "0x03"], limit);
    assert!(!ok && err.contains("127.0.0.1:1"), "{err}");
    let (ok, _, err) = cli(&["--tpc", "127.0.0.1:1", "bridge", "info", "0x03"], limit);
    assert!(!ok && err.contains("unknown option --tpc"), "{err}");
}

/// The console on TCP with the bridge resetting as the firmware does once an update is in place: the
/// client's connection drops and the next one reaches the bridge after its reset. Serves until
/// `connections` clients have come and gone.
fn serve_console_resetting(listener: TcpListener, esp: Esp, connections: usize) -> Esp {
    serve_console_resetting_info(listener, esp, connections, 0)
}

/// [`serve_console_resetting`] for a bridge built with another `ESP_INFO` layout: every `ESP_INFO`
/// payload it sends is `info_delta` bytes longer (zero bytes appended) or shorter.
fn serve_console_resetting_info(
    listener: TcpListener,
    mut esp: Esp,
    connections: usize,
    info_delta: isize,
) -> Esp {
    for _ in 0..connections {
        let mut framer: StreamFramer = StreamFramer::new();
        let (mut sock, _) = listener.accept().unwrap();
        esp.router.connect(Port::Tcp);
        sock.set_read_timeout(Some(Duration::from_millis(1)))
            .unwrap();
        let mut buf = [0u8; 256];
        let mut out = [0u8; 64];
        loop {
            if esp.router.room(Port::Tcp) >= buf.len() {
                match sock.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => {
                        esp.router.on_bytes(Port::Tcp, &buf[..n]);
                    }
                    Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                    Err(_) => break,
                }
            }
            esp.node.pump(&mut esp.router, &mut esp.port, &mut esp.svc);
            loop {
                let k = esp.router.next_out(Port::Tcp, &mut out);
                if k == 0 {
                    break;
                }
                if info_delta == 0 {
                    let _ = sock.write_all(&out[..k]);
                    continue;
                }
                let mut frames = Vec::new();
                framer.feed(&out[..k], &mut |f| frames.push(f.to_vec()));
                for mut f in frames {
                    // A frame holding a whole packet (MORE clear, fragment index 0): the frag-hdr,
                    // then the PDU's opcode.
                    let hdr = link::FragHdr::decode(f[0]);
                    if !hdr.more && hdr.frag_idx == 0 && f[1] == espsvc::OP_INFO {
                        let len = f.len().checked_add_signed(info_delta).unwrap();
                        f.resize(len, 0);
                    }
                    let mut buf = [0u8; 300];
                    let n = link::encode_stream_frame(&f, &mut buf).unwrap();
                    let _ = sock.write_all(&buf[..n]);
                }
            }
            if esp.svc.reboot_requested() {
                break;
            }
        }
        if esp.svc.reboot_requested() {
            drop(sock);
            esp = esp.reset();
        } else {
            esp.router.disconnect(Port::Tcp);
        }
    }
    esp
}

/// An ESP32-C3 app image of `len` bytes whose app descriptor carries `version`.
fn esp_image(len: usize, version: &str) -> Vec<u8> {
    let mut img: Vec<u8> = (0..len as u32)
        .map(|i| (i.wrapping_mul(2_654_435_761) >> 11) as u8)
        .collect();
    img[0] = 0xE9;
    img[12..14].copy_from_slice(&5u16.to_le_bytes());
    img[0x20..0x24].copy_from_slice(&0xABCD_5432u32.to_le_bytes());
    img[0x30..0x50].fill(0);
    img[0x30..0x30 + version.len()].copy_from_slice(version.as_bytes());
    img
}

#[test]
fn update_esp_over_tcp_confirms_the_new_slot_or_leaves_it_to_roll_back() {
    use hoverboard_bridge_core::sim::OtaState;
    let img = esp_image(9_001, "0.2.0");
    let bin = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("esp-update.bin");
    std::fs::write(&bin, &img).unwrap();
    let limit = Duration::from_secs(60);

    // Confirmed: the bridge runs ota_1 and keeps it.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let esp = Esp::with_platform(Family::F1x0, SimPlatform::with_slots(16 * 1024));
    let server = std::thread::spawn(move || serve_console_resetting(listener, esp, 2));
    let (ok, out, err) = cli(
        &[
            "--tcp",
            &addr,
            "bridge",
            "update",
            "0x03",
            bin.to_str().unwrap(),
        ],
        limit,
    );
    assert!(ok, "{err}");
    assert!(
        err.contains("running ota_0; sending 9001 B, version 0.2.0"),
        "{err}"
    );
    assert!(out.contains("slot        ota_1\n"), "{out}");
    let mut esp = server.join().unwrap();
    let p = esp.svc.platform();
    assert_eq!(&p.slots[1][..img.len()], &img[..]);
    assert_eq!(p.selected, Some((1, OtaState::Valid)));
    p.reset();
    assert_eq!(p.booted, 1, "a confirmed slot stays");

    // Not confirmed: the bridge runs ota_1 pending, and its next reset returns to ota_0.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let esp = Esp::with_platform(Family::F1x0, SimPlatform::with_slots(16 * 1024));
    let server = std::thread::spawn(move || serve_console_resetting(listener, esp, 2));
    let tcp = format!("--tcp={addr}");
    let (ok, out, err) = cli(
        &[
            &tcp,
            "bridge",
            "update",
            "0x03",
            bin.to_str().unwrap(),
            "--no-confirm",
        ],
        limit,
    );
    assert!(ok, "{err}");
    assert!(
        out.contains("slot        ota_1, pending verification"),
        "{out}"
    );
    assert!(
        out.contains("not confirmed: the bridge's next reset returns to ota_0"),
        "{out}"
    );
    let mut esp = server.join().unwrap();
    let p = esp.svc.platform();
    p.reset();
    assert_eq!(p.booted, 0, "rolled back");

    // A bridge whose `ESP_INFO` has another layout, longer or shorter than this CLI's: the update
    // goes on without it and confirms.
    for delta in [5, -5] {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let esp = Esp::with_platform(Family::F1x0, SimPlatform::with_slots(16 * 1024));
        let server =
            std::thread::spawn(move || serve_console_resetting_info(listener, esp, 2, delta));
        let (ok, out, err) = cli(
            &[
                "--tcp",
                &addr,
                "bridge",
                "update",
                "0x03",
                bin.to_str().unwrap(),
            ],
            limit,
        );
        assert!(ok, "{delta}: {err}");
        assert_eq!(
            err.matches("bridge's info has another layout; continuing")
                .count(),
            2,
            "{delta}: before the update and after the confirm: {err}"
        );
        assert!(err.contains("sending 9001 B, version 0.2.0"), "{err}");
        assert!(err.contains("confirmed"), "{err}");
        assert!(!out.contains("slot "), "{delta}: no info printed: {out}");
        let mut esp = server.join().unwrap();
        let p = esp.svc.platform();
        assert_eq!(&p.slots[1][..img.len()], &img[..]);
        assert_eq!(p.selected, Some((1, OtaState::Valid)), "{delta}");
    }

    // A file that is not a bridge image costs no connection.
    let (ok, _, err) = cli(
        &[
            "--tcp",
            "127.0.0.1:1",
            "bridge",
            "update",
            "0x03",
            "Cargo.toml",
        ],
        limit,
    );
    assert!(!ok && err.contains("not an ESP32-C3 app image"), "{err}");
}
