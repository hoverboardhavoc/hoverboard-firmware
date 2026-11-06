//! A simulated machine of two pairs: two bridges, each with the real links, node, mailbox port and
//! command service of `hoverboard_bridge_core`, each in front of a simulated GD32 whose firmware
//! end of the mailbox and walk responder answer as a board does, the two boards linked by UART.
//! Each bridge serves its console on its own local TCP port, so a command takes the real path over
//! a link to one of them. The tests run `board update` over the master's bridge.

use std::io::{ErrorKind, Read};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use hoverboard_bridge_core::mailbox::MailboxPort;
use hoverboard_bridge_core::node::{default_name, Node, PROBE_WINDOW_US};
use hoverboard_bridge_core::router::{Port, Router};
use hoverboard_bridge_core::sim::{
    running_board, Family, Firmware, Gd32, RamFlash, SimBoard, SimPins, SimPlatform,
};
use hoverboard_bridge_core::svc::{Build, ImageBuffer, Service};
use hoverboard_bridge_core::update::Update;
use hoverboard_cli::bundle::{image_table, pack, PackEntry};
use hoverboard_cli::elf;

use super::*;

#[path = "../tests/common/mod.rs"]
mod common;

/// The commit both simulated bridges were built from.
const OLD: u32 = 0x50a4_6c91;

/// One bridge and the board on its mailbox.
struct Side {
    router: Router,
    node: Node<'static, RamFlash>,
    port: MailboxPort<SimPins<Gd32>>,
    svc: Service<'static, SimPlatform>,
    fw: Firmware,
    board: SimBoard,
    /// Frames the other board sent on the UART, not yet taken in.
    uart_in: Vec<Vec<u8>>,
    sock: Option<TcpStream>,
}

fn service(platform: SimPlatform) -> Service<'static, SimPlatform> {
    Service::new(
        Build {
            version: [0, 1, 0],
            commit: OLD,
            swd_khz: 1000,
        },
        Box::leak(Box::new(ImageBuffer::new())),
        Box::leak(Box::new(Update::new())),
        platform,
    )
}

impl Side {
    /// Bridge `bridge` named `name` (its MAC's last byte `mac`) in front of board `board` of
    /// `family`, whose SWD `uid` is the one its walk responder reports.
    fn new(name: &'static str, mac: u8, bridge: u8, board: u8, family: Family) -> Self {
        let flash: &'static mut RamFlash = Box::leak(Box::new(RamFlash::erased(4096)));
        let mut keys = store::Store::mount(flash).unwrap();
        keys.set_value(store::NODE_ADDRESS.key(), store::Value::U8(bridge))
            .unwrap();
        keys.set_value(store::DEVICE_NAME.key(), store::Value::Str(name.as_bytes()))
            .unwrap();
        let mac = [0, 0, 0, 0, 0xAB, mac];
        let default: &'static [u8; 14] = Box::leak(Box::new(default_name(mac)));
        let (mut port, fw) = running_board();
        let g = &mut port.swd().pins().bus;
        g.uid = [board; 12];
        g.family = family;
        Side {
            router: Router::new(),
            node: Node::new(keys, true, 0x0001, mac, default),
            port,
            svc: service(SimPlatform::with_slots(16 * 1024)),
            fw,
            board: SimBoard::holding(board),
            uart_in: Vec::new(),
            sock: None,
        }
    }

    fn gd32(&mut self) -> &mut Gd32 {
        &mut self.port.swd().pins().bus
    }

    /// Route the board's emissions: port 0 into its mailbox, port 1 onto the UART.
    fn route(&mut self, emits: &net::walk::Emits, uart_out: &mut Vec<Vec<u8>>) {
        for e in emits {
            match e.port {
                0 => {
                    let _ = self.fw.link.send(&e.bytes);
                }
                1 => uart_out.push(e.bytes.to_vec()),
                _ => {}
            }
        }
    }

    /// One pass of the bridge's service loop and the board's firmware; whether anything moved.
    /// While a job runs the board is halted and only the bridge runs.
    fn step(&mut self, uart_out: &mut Vec<Vec<u8>>) -> bool {
        self.node
            .pump(&mut self.router, &mut self.port, &mut self.svc);
        if self.svc.busy() {
            return true;
        }
        let mut moved = self.port.queued() > 0;
        for f in self.fw.poll() {
            moved = true;
            let e = self.board.ingest(0, &f);
            self.route(&e, uart_out);
        }
        for f in std::mem::take(&mut self.uart_in) {
            moved = true;
            let e = self.board.ingest(1, &f);
            self.route(&e, uart_out);
        }
        moved
    }

    /// Nothing moved: close a probe window in flight, or let the clock run while the mailbox
    /// waits on the board. Whether there was one.
    fn tick(&mut self, uart_out: &mut Vec<Vec<u8>>) -> bool {
        if self.board.resp.probing() {
            let mut e = net::walk::Emits::new();
            self.board.resp.poll_probe(&mut e);
            self.route(&e, uart_out);
            return true;
        }
        if self.node.responder().probing() {
            self.port.swd().pins().sleep_us(PROBE_WINDOW_US + 1);
            return true;
        }
        if self.port.state() != espsvc::MailboxState::Up {
            self.port.swd().pins().sleep_us(100_000);
            return true;
        }
        false
    }

    /// Take a waiting client when none is connected, and move bytes both ways.
    fn serve(&mut self, listener: &TcpListener) -> bool {
        let mut moved = false;
        if self.sock.is_none() {
            if let Ok((s, _)) = listener.accept() {
                s.set_read_timeout(Some(Duration::from_millis(1))).unwrap();
                s.set_nonblocking(false).unwrap();
                self.router.connect(Port::Tcp);
                self.sock = Some(s);
            }
        }
        let Some(sock) = self.sock.as_mut() else {
            return false;
        };
        let mut buf = [0u8; 256];
        let mut closed = false;
        if self.router.room(Port::Tcp) >= buf.len() {
            match sock.read(&mut buf) {
                Ok(0) => closed = true,
                Ok(n) => {
                    self.router.on_bytes(Port::Tcp, &buf[..n]);
                    moved = true;
                }
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                Err(_) => closed = true,
            }
        }
        let mut out = [0u8; 64];
        loop {
            let k = self.router.next_out(Port::Tcp, &mut out);
            if k == 0 {
                break;
            }
            let _ = sock.write_all(&out[..k]);
            moved = true;
        }
        if closed {
            self.sock = None;
            self.router.disconnect(Port::Tcp);
        }
        moved
    }
}

/// What the test reads of a side once the run is over (a side holds raw mailbox pointers and stays
/// on the server's thread).
#[derive(Debug)]
struct After {
    flash: Vec<u8>,
    booted: bool,
}

impl Side {
    fn after(mut self) -> After {
        let g = self.gd32();
        let (flash, booted) = (g.flash_bytes(), g.mailbox().magic() == swd_mailbox::MAGIC);
        After { flash, booted }
    }
}

/// The machine: side 0 the slave's bridge and board, side 1 the master's, serving until `stop`.
fn serve(sides: &mut [Side; 2], listeners: &[TcpListener; 2], stop: &AtomicBool) {
    for l in listeners {
        l.set_nonblocking(true).unwrap();
    }
    while !stop.load(Ordering::Relaxed) {
        let mut moved = false;
        for i in 0..2 {
            moved |= sides[i].serve(&listeners[i]);
            let mut uart = Vec::new();
            moved |= sides[i].step(&mut uart);
            sides[1 - i].uart_in.extend(uart);
        }
        if !moved {
            let mut ticked = false;
            for i in 0..2 {
                let mut uart = Vec::new();
                ticked |= sides[i].tick(&mut uart);
                sides[1 - i].uart_in.extend(uart);
            }
            if !ticked {
                std::thread::sleep(Duration::from_micros(200));
            }
        }
    }
}

/// An ESP32-C3 app image whose app descriptor carries `0.2.0`.
fn app_image() -> Vec<u8> {
    let mut img: Vec<u8> = (0..9_001u32)
        .map(|i| (i.wrapping_mul(2_654_435_761) >> 11) as u8)
        .collect();
    img[0] = 0xE9;
    img[12..14].copy_from_slice(&5u16.to_le_bytes());
    img[0x20..0x24].copy_from_slice(&0xABCD_5432u32.to_le_bytes());
    img[0x30..0x50].fill(0);
    img[0x30..0x35].copy_from_slice(b"0.2.0");
    img
}

/// A bundle of both mainboard images and the bridge's app image, written to a file.
fn bundle_file(app: &[u8], tag: &str) -> (String, Vec<u8>, Vec<u8>) {
    let firmware = include_str!("../../../motorcontroller/firmware/Cargo.toml");
    let bridge = include_str!("../../../bridge/Cargo.toml");
    let f130 = common::flashable_elf(&[0x11, 0x22, 0x33, 0x44]);
    let g103 = elf::guards_for(common::GUARD_FILE, "mainboard-gd32f103c8").unwrap();
    let f103 = common::Fixture::new(&[0x66; 8], g103.code_floor, common::names_of(&g103)).build();
    let z = pack(
        &[
            PackEntry {
                name: "mainboard-gd32f103c8",
                table: image_table(firmware, "mainboard-gd32f103c8").unwrap(),
                guards: Some(g103),
                file: &f103,
            },
            PackEntry {
                name: "mainboard-gd32f130c8",
                table: image_table(firmware, "mainboard-gd32f130c8").unwrap(),
                guards: Some(common::release_guards()),
                file: &f130,
            },
            PackEntry {
                name: "bridge-esp32c3",
                table: image_table(bridge, "bridge-esp32c3").unwrap(),
                guards: None,
                file: app,
            },
        ],
        "c0eb25917cd904d5d9ba9307f56cb8251cbb61dd",
        "2026-11-02T08:00:00Z",
    )
    .unwrap();
    let path = std::env::temp_dir().join(format!("{tag}-{}.hbfw", std::process::id()));
    std::fs::write(&path, z).unwrap();
    (
        path.to_str().unwrap().to_string(),
        elf::to_bin(&f103).unwrap(),
        elf::to_bin(&f130).unwrap(),
    )
}

/// Run `command` over the master's bridge of the rover's layout: the slave board 0x01 on bridge
/// 0x02, the master board 0x03 on bridge 0x04, the boards linked by UART; the master a GD32F103,
/// the slave a GD32F130. The command's outcome and the two sides after it.
fn on_master(
    command: impl FnOnce(&Link) -> Result<(), String>,
) -> (Result<(), String>, [After; 2]) {
    let listeners = [
        TcpListener::bind("127.0.0.1:0").unwrap(),
        TcpListener::bind("127.0.0.1:0").unwrap(),
    ];
    let port = listeners[1].local_addr().unwrap().to_string();
    let stop = Arc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let server = std::thread::spawn(move || {
        let mut sides = [
            Side::new("rover-slave", 0x01, 0x02, 0x01, Family::F1x0),
            Side::new("rover-master", 0x02, 0x04, 0x03, Family::F10x),
        ];
        serve(&mut sides, &listeners, &flag);
        sides.map(Side::after)
    });
    let r = command(&Link::Tcp(port));
    stop.store(true, Ordering::Relaxed);
    (r, server.join().unwrap())
}

#[test]
fn board_update_flashes_the_image_for_the_boards_chip_through_its_bridge() {
    let (path, f103_bin, _) = bundle_file(&app_image(), "board-update-sim");
    let op = Swd::Update {
        bundle: path.clone(),
        image: None,
        yes: true,
    };
    let (r, sides) = on_master(|link| board_swd(link, "03030303", &op));
    let _ = std::fs::remove_file(&path);
    assert_eq!(r, Ok(()));
    let m = &sides[1];
    assert_eq!(
        &m.flash[..f103_bin.len()],
        &f103_bin[..],
        "the GD32F103 image"
    );
    assert!(m.booted, "its board booted");
}
