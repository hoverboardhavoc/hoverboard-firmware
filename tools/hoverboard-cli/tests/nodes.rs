//! The walk and the configuration commands of the built binary (`specs/esp-tool.md`, "The terminal
//! tool"), over the `--tcp` link to a whole simulated machine: the ESP's real links, node, mailbox
//! port and service, a master's `net` responder on the other end of the mailbox, and its slave on
//! the master's UART (`esp_tool_core::sim::Machine`). The ESP starts unassigned; the boards hold
//! `0x01` and `0x02`.

use std::io::{ErrorKind, Read, Write};
use std::net::TcpListener;
use std::process::Command;
use std::time::Duration;

use esp_tool_core::router::Port;
use esp_tool_core::sim::{Machine, RamFlash, SimPlatform};
use esp_tool_core::svc::{Build, ImageBuffer, Service};
use esp_tool_core::update::Update;

fn machine() -> Machine<'static> {
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
    Machine::new(flash, svc)
}

/// What the machine holds once the clients are done: the ESP's address and stored name, and the
/// `DRIVE_CMD`s each board took.
struct After {
    esp: u8,
    name: Vec<u8>,
    drives: [usize; 2],
}

/// Serve the ESP's TCP console for `connections` clients one after the other, running the whole
/// machine between reads, and letting the probe windows close whenever nothing moves.
fn serve(listener: TcpListener, connections: usize) -> After {
    let mut m = machine();
    for _ in 0..connections {
        let (mut sock, _) = listener.accept().unwrap();
        m.router.connect(Port::Tcp);
        sock.set_read_timeout(Some(Duration::from_millis(1)))
            .unwrap();
        let mut buf = [0u8; 256];
        let mut out = [0u8; 64];
        loop {
            match sock.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => m.router.on_bytes(Port::Tcp, &buf[..n]),
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                Err(_) => break,
            }
            let moved = m.step();
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
    let drives = |got: &[(u8, u8)]| {
        got.iter()
            .filter(|&&(op, _)| op == linkctl::OP_DRIVE_CMD)
            .count()
    };
    After {
        esp: m.node.addr(),
        name: m.node.store().get_text(store::DEVICE_NAME).to_vec(),
        drives: [drives(&m.master.got), drives(&m.slave.got)],
    }
}

fn cli(args: &[&str]) -> (bool, String, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args(args)
        .output()
        .unwrap();
    (
        o.status.success(),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

#[test]
fn nodes_name_config_and_esp_commands_by_node() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 9));

    // The walk: the fresh ESP ends at 0x03 and the boards keep theirs.
    let (ok, out, err) = cli(&[&tcp, "nodes"]);
    assert!(ok, "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert_eq!(rows.len(), 4, "{out}");
    assert!(rows[1].starts_with("0x01     motor controller"), "{out}");
    assert!(rows[2].starts_with("0x02     motor controller"), "{out}");
    assert!(rows[3].starts_with("0x03     bridge"), "{out}");

    // A name, then the node by that name.
    let (ok, out, err) = cli(&[&tcp, "name", "0x03", "rover-esp"]);
    assert!(ok, "{err}");
    assert_eq!(out.trim(), "device.name of 0x03 (Hoverboard): ok");
    let (ok, out, err) = cli(&[&tcp, "config", "rover-esp", "0x10"]);
    assert!(ok, "{err}");
    assert_eq!(out.trim(), "0x10.0 at 0x03 (rover-esp): ok \"rover-esp\"");

    // A typed write the board refuses, printed by name, and the command fails.
    let (ok, out, _) = cli(&[&tcp, "config", "0x01", "0x03", "2"]);
    assert!(!ok);
    assert!(out.contains(": read-only"), "{out}");

    // An ESP command to the ESP on the link, and by name through the walk.
    let (ok, out, err) = cli(&[&tcp, "info"]);
    assert!(ok, "{err}");
    assert!(out.contains("node        0x03"), "{out}");
    assert!(out.contains("store       mounted"), "{out}");
    let (ok, out, err) = cli(&[&tcp, "--node", "rover-esp", "info"]);
    assert!(ok, "{err}");
    assert!(out.contains("node        0x03"), "{out}");

    // A motor controller is not an ESP.
    let (ok, _, err) = cli(&[&tcp, "--node=0x01", "info"]);
    assert!(!ok);
    assert!(err.contains("motor controller, not a bridge"), "{err}");

    // `drive` with no nodes drives the motor controllers the walk found.
    let (ok, out, err) = cli(&[&tcp, "drive", "300", "1"]);
    assert!(ok, "{err}");
    assert!(out.contains("neutral sent, power 0 sent"), "{out}");

    // `watch` with nodes given only attaches.
    let (ok, _, err) = cli(&[&tcp, "watch", "1", "0x01"]);
    assert!(ok, "{err}");

    let after = server.join().unwrap();
    assert_eq!(after.esp, 0x03);
    assert_eq!(after.name, b"rover-esp");
    for d in after.drives {
        assert!(d >= 20, "{d} drive frames");
    }
}

/// A board's own end of a link, its BLE module's place: one `net` responder on port 0, no ESP.
struct Sock(std::net::TcpStream);

impl hoverboard_cli::wire::ByteLink for Sock {
    fn write(&mut self, bytes: &[u8]) {
        let _ = self.0.write_all(bytes);
    }
    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        let _ = self
            .0
            .set_read_timeout(Some(timeout.max(Duration::from_millis(1))));
        let mut buf = [0u8; 512];
        let n = self.0.read(&mut buf).unwrap_or(0);
        buf[..n].to_vec()
    }
}

#[test]
fn over_a_boards_own_link_an_esp_command_finds_no_bridge() {
    use hoverboard_cli::wire::{Framed, Wire};
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || {
        let (sock, _) = listener.accept().unwrap();
        let mut w = Framed::new(Sock(sock));
        let mut b = esp_tool_core::sim::SimBoard::holding(0x01);
        let t0 = std::time::Instant::now();
        while t0.elapsed() < Duration::from_secs(20) {
            match w.recv(Duration::from_millis(5)) {
                Some(f) => {
                    for e in &b.ingest(0, &f) {
                        if e.port == 0 {
                            w.send(&e.bytes);
                        }
                    }
                }
                None if b.resp.probing() => {
                    let mut e = net::walk::Emits::new();
                    b.resp.poll_probe(&mut e);
                    for x in &e {
                        if x.port == 0 {
                            w.send(&x.bytes);
                        }
                    }
                }
                None => {}
            }
        }
    });
    let (ok, _, err) = cli(&[&tcp, "info"]);
    assert!(!ok);
    assert!(err.contains("no bridge on this machine"), "{err}");
    drop(server);
}
