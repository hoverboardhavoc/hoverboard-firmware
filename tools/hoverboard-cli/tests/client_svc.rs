//! `hoverboard-cli`'s client against the bridge's command service over an in-memory link
//! (`specs/hoverboard-bridge.md`, "Verification", ESP commands): the bridge side is the real links, node,
//! mailbox port and service on the simulated GD32, and the link drops, duplicates and reorders chunks. The
//! same bridge also answers on its console through a local TCP socket, as the firmware's Wi-Fi console
//! serves it, reached by the `--tcp` link, and the built binary runs its bulk commands (a
//! multi-chunk read, a flash from an ELF) over that link.

mod common;

use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::net::TcpListener;
use std::process::{Command, Stdio};
use std::time::Duration;
use std::time::Instant;

use espsvc::{Ctrl, Stage, Status, OP_FLASH_CHUNK};
use hoverboard_bridge_core::mailbox::MailboxPort;
use hoverboard_bridge_core::node::Node;
use hoverboard_bridge_core::router::{Port, Router, NOTIFY_MAX};
use hoverboard_bridge_core::sim::{board, Family, Gd32, Inject, RamFlash, SimPins, SimPlatform};
use hoverboard_bridge_core::svc::{Build, ImageBuffer, Service};
use hoverboard_bridge_core::update::Update;
use hoverboard_cli::client::{Client, ClientError, Progress, StageTimes};
use hoverboard_cli::transport::Tcp;
use hoverboard_cli::wire::{Framed, Wire};
use link::StreamFramer;
use sha2::{Digest, Sha256};

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
        Esp {
            router,
            node: Node::new(store::Store::mount(flash).unwrap(), true, 0x0001),
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
            match sock.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    esp.router.on_bytes(Port::Tcp, &buf[..n]);
                }
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                Err(_) => break,
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

/// An executable ARM ELF as the linker would leave it for `flash`: one loadable `.text` at the
/// flash base holding `code` (padded to the code floor, marked `$d` so the `wfi` scan reads none of
/// it as instructions), a `.bss`, and the guard's required symbols.
fn flashable_elf(code: &[u8]) -> Vec<u8> {
    use object::elf::*;
    use object::write::elf::{FileHeader, ProgramHeader, SectionHeader, Sym, Writer};
    let base = 0x0800_0000u32;
    let mut text = code.to_vec();
    text.resize(text.len().max(hoverboard_cli::elf::CODE_FLOOR as usize), 0);
    let (t_syms, b_syms) = common::all_required();

    let mut out = Vec::new();
    let mut w = Writer::new(object::Endianness::Little, false, &mut out);
    w.reserve_file_header();
    w.reserve_program_headers(1);
    let text_off = w.reserve(text.len(), 4);
    let text_name = w.add_section_name(b".text");
    let bss_name = w.add_section_name(b".bss");
    w.reserve_null_section_index();
    let text_idx = w.reserve_section_index();
    let bss_idx = w.reserve_section_index();
    let d_name = w.add_string(b"$d");
    let t_names: Vec<_> = t_syms.iter().map(|n| w.add_string(n.as_bytes())).collect();
    let b_names: Vec<_> = b_syms.iter().map(|n| w.add_string(n.as_bytes())).collect();
    w.reserve_null_symbol_index();
    w.reserve_symbol_index(Some(text_idx)); // $d, the one local
    for _ in &t_names {
        w.reserve_symbol_index(Some(text_idx));
    }
    for _ in &b_names {
        w.reserve_symbol_index(Some(bss_idx));
    }
    w.reserve_symtab_section_index();
    w.reserve_strtab_section_index();
    w.reserve_shstrtab_section_index();
    w.reserve_symtab();
    w.reserve_strtab();
    w.reserve_shstrtab();
    w.reserve_section_headers();

    w.write_file_header(&FileHeader {
        os_abi: 0,
        abi_version: 0,
        e_type: ET_EXEC,
        e_machine: EM_ARM,
        e_entry: u64::from(base),
        e_flags: 0,
    })
    .unwrap();
    w.write_align_program_headers();
    w.write_program_header(&ProgramHeader {
        p_type: PT_LOAD,
        p_flags: PF_R | PF_X,
        p_offset: text_off as u64,
        p_vaddr: u64::from(base),
        p_paddr: u64::from(base),
        p_filesz: text.len() as u64,
        p_memsz: text.len() as u64,
        p_align: 4,
    });
    w.write_align(4);
    w.write(&text);
    w.write_null_symbol();
    let sym = |name, section, st_info, st_value| Sym {
        name: Some(name),
        section: Some(section),
        st_info,
        st_other: 0,
        st_shndx: 0,
        st_value,
        st_size: 0,
    };
    w.write_symbol(&sym(
        d_name,
        text_idx,
        (STB_LOCAL << 4) | STT_NOTYPE,
        u64::from(base),
    ));
    for n in t_names {
        w.write_symbol(&sym(
            n,
            text_idx,
            (STB_GLOBAL << 4) | STT_FUNC,
            u64::from(base),
        ));
    }
    for n in b_names {
        w.write_symbol(&sym(
            n,
            bss_idx,
            (STB_GLOBAL << 4) | STT_OBJECT,
            0x2000_0000,
        ));
    }
    w.write_strtab();
    w.write_shstrtab();
    w.write_null_section_header();
    w.write_section_header(&SectionHeader {
        name: Some(text_name),
        sh_type: SHT_PROGBITS,
        sh_flags: u64::from(SHF_ALLOC | SHF_EXECINSTR),
        sh_addr: u64::from(base),
        sh_offset: text_off as u64,
        sh_size: text.len() as u64,
        sh_link: 0,
        sh_info: 0,
        sh_addralign: 4,
        sh_entsize: 0,
    });
    w.write_section_header(&SectionHeader {
        name: Some(bss_name),
        sh_type: SHT_NOBITS,
        sh_flags: u64::from(SHF_ALLOC | SHF_WRITE),
        sh_addr: 0x2000_0000,
        sh_offset: 0,
        sh_size: 64,
        sh_link: 0,
        sh_info: 0,
        sh_addralign: 4,
        sh_entsize: 0,
    });
    w.write_symtab_section_header(2);
    w.write_strtab_section_header();
    w.write_shstrtab_section_header();
    out
}

/// Run the built binary; a run that outlives `limit` is killed and fails the test (a command that
/// took the BLE path on a host with no radio waits there).
fn cli(args: &[&str], limit: Duration) -> (bool, String, String) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args(args)
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

#[test]
fn the_binary_runs_a_multi_chunk_read_and_a_flash_over_tcp() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap().to_string();
    let mut esp = Esp::new(Family::F1x0);
    let store: Vec<u16> = (0..1024u16).map(|i| i.wrapping_mul(0x9E37)).collect();
    esp.model().flash[31 * 1024..].copy_from_slice(&store);
    let server = std::thread::spawn(move || serve_console_over_tcp(listener, esp, 2));
    let limit = Duration::from_secs(60);

    let (ok, out, err) = cli(&["--tcp", &addr, "read", "0x0800F800", "2048"], limit);
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
    std::fs::write(&elf, flashable_elf(&img)).unwrap();
    let tcp = format!("--tcp={addr}");
    let (ok, out, err) = cli(&[&tcp, "flash", elf.to_str().unwrap()], limit);
    assert!(ok, "{err}");
    assert!(out.starts_with("booted"), "{out}");
    let stages = out.lines().find(|l| l.starts_with("stages: ")).expect(&out);
    for name in ["transfer", "erase", "program", "verify", "boot"] {
        assert!(stages.contains(&format!("{name} ")), "{stages}");
    }

    let mut esp = server.join().unwrap();
    let g = esp.model();
    let flashed = g.flash_bytes();
    let n = (hoverboard_cli::elf::CODE_FLOOR as usize).max(img.len());
    assert_eq!(&flashed[..img.len()], &img[..]);
    assert!(flashed[img.len()..n].iter().all(|&b| b == 0));
    assert_eq!(&g.flash[31 * 1024..], &store[..]);
    assert_eq!(g.mailbox().magic(), swd_mailbox::MAGIC, "booted");
}

#[test]
fn a_link_option_never_falls_back_to_ble() {
    let limit = Duration::from_secs(20);
    // Nothing listens on port 1: the TCP link fails at once, with no BLE scan behind it.
    let (ok, _, err) = cli(
        &["--tcp", "127.0.0.1:1", "read", "0x0800F800", "2048"],
        limit,
    );
    assert!(!ok && err.contains("127.0.0.1:1"), "{err}");
    let (ok, _, err) = cli(&["--tcp=127.0.0.1:1", "info"], limit);
    assert!(!ok && err.contains("127.0.0.1:1"), "{err}");
    let (ok, _, err) = cli(&["--tpc", "127.0.0.1:1", "info"], limit);
    assert!(!ok && err.contains("unknown option --tpc"), "{err}");
}

/// The console on TCP with the bridge resetting as the firmware does once an update is in place: the
/// client's connection drops and the next one reaches the bridge after its reset. Serves until
/// `connections` clients have come and gone.
fn serve_console_resetting(listener: TcpListener, mut esp: Esp, connections: usize) -> Esp {
    for _ in 0..connections {
        let (mut sock, _) = listener.accept().unwrap();
        esp.router.connect(Port::Tcp);
        sock.set_read_timeout(Some(Duration::from_millis(1)))
            .unwrap();
        let mut buf = [0u8; 256];
        let mut out = [0u8; 64];
        loop {
            match sock.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    esp.router.on_bytes(Port::Tcp, &buf[..n]);
                }
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                Err(_) => break,
            }
            esp.node.pump(&mut esp.router, &mut esp.port, &mut esp.svc);
            loop {
                let k = esp.router.next_out(Port::Tcp, &mut out);
                if k == 0 {
                    break;
                }
                let _ = sock.write_all(&out[..k]);
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
        &["--tcp", &addr, "update-esp", bin.to_str().unwrap()],
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
        &[&tcp, "update-esp", bin.to_str().unwrap(), "--no-confirm"],
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

    // A file that is not a bridge image costs no connection.
    let (ok, _, err) = cli(&["--tcp", "127.0.0.1:1", "update-esp", "Cargo.toml"], limit);
    assert!(!ok && err.contains("not an ESP32-C3 app image"), "{err}");
}
