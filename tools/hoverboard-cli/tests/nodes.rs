//! The walk and the configuration commands of the built binary (`specs/hoverboard-bridge.md`, "The terminal
//! tool"), over the `--tcp` link to a whole simulated machine: the bridge's real links, node, mailbox
//! port and service, a master's `net` responder on the other end of the mailbox, and its slave on
//! the master's UART (`hoverboard_bridge_core::sim::Machine`). The bridge starts unassigned; the boards hold
//! `0x01` and `0x02`.

use std::io::{ErrorKind, Read, Write};
use std::net::TcpListener;
use std::process::Command;
use std::time::Duration;

use hoverboard_bridge_core::router::Port;
use hoverboard_bridge_core::sim::{Machine, RamFlash, SimPlatform};
use hoverboard_bridge_core::svc::{Build, ImageBuffer, Service};
use hoverboard_bridge_core::update::Update;

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

/// What the machine holds once the clients are done: the bridge's address and stored name, the
/// `DRIVE_CMD`s each board took, and the opcodes each client connection sent.
struct After {
    esp: u8,
    name: Vec<u8>,
    drives: [usize; 2],
    ops: Vec<Vec<u8>>,
}

/// The bytes a client sent, framed back into packets for [`After::ops`].
#[derive(Default)]
struct Tap(Vec<u8>);

impl hoverboard_cli::wire::ByteLink for Tap {
    fn write(&mut self, _bytes: &[u8]) {}
    fn read(&mut self, _timeout: Duration) -> Vec<u8> {
        std::mem::take(&mut self.0)
    }
    fn window(&self) -> usize {
        1
    }
}

/// Serve the bridge's TCP console for `connections` clients one after the other, running the whole
/// machine between reads, and letting the probe windows close whenever nothing moves.
fn serve(listener: TcpListener, connections: usize) -> After {
    use hoverboard_cli::wire::Wire;
    let mut m = machine();
    let mut ops = Vec::new();
    for _ in 0..connections {
        let mut tap = hoverboard_cli::wire::Framed::new(Tap::default());
        let mut sent = Vec::new();
        let (mut sock, _) = listener.accept().unwrap();
        m.router.connect(Port::Tcp);
        sock.set_read_timeout(Some(Duration::from_millis(1)))
            .unwrap();
        let mut buf = [0u8; 256];
        let mut out = [0u8; 64];
        loop {
            match sock.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    m.router.on_bytes(Port::Tcp, &buf[..n]);
                    tap.inner().0.extend_from_slice(&buf[..n]);
                    while let Some(p) = tap.recv(Duration::ZERO) {
                        sent.push(p[0]);
                    }
                }
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
        ops.push(sent);
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
        ops,
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
    let server = std::thread::spawn(move || serve(listener, 10));

    // The walk: the fresh bridge ends at 0x03 and the boards keep theirs.
    let (ok, out, err) = cli(&[&tcp, "nodes"]);
    assert!(ok, "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert_eq!(rows.len(), 4, "{out}");
    assert!(rows[1].starts_with("0x01     motor controller"), "{out}");
    assert!(rows[2].starts_with("0x02     motor controller"), "{out}");
    assert!(rows[3].starts_with("0x03     bridge"), "{out}");

    // A name the bridge cannot advertise is refused with the rule, and the old one stays.
    let (ok, out, err) = cli(&[&tcp, "name", "0x03", "Offroad master"]);
    assert!(!ok);
    let rule = "name must be 1 to 26 letters, digits or '-'";
    assert_eq!(
        out.trim(),
        format!("device.name of 0x03 (hb-bridge-ab12): {rule}")
    );
    assert!(err.contains(&format!("refused: {rule}")), "{err}");

    // A name, then the node by that name.
    let (ok, out, err) = cli(&[&tcp, "name", "0x03", "rover-esp"]);
    assert!(ok, "{err}");
    assert_eq!(out.trim(), "device.name of 0x03 (hb-bridge-ab12): ok");
    let (ok, out, err) = cli(&[&tcp, "config", "rover-esp", "0x10"]);
    assert!(ok, "{err}");
    assert_eq!(out.trim(), "rover-esp: device.name = \"rover-esp\"");

    // A key the board's image does not know: forced past the name table, refused by the board.
    let (ok, _, err) = cli(&[&tcp, "config", "0x01", "0x99", "5", "--force"]);
    assert!(!ok);
    assert!(
        err.contains("warning: 0x99 is not in the name table"),
        "{err}"
    );
    assert!(
        err.contains("0x99.0: the board does not know this key"),
        "{err}"
    );

    // An ESP command to the bridge on the link, and by name through the walk.
    let (ok, out, err) = cli(&[&tcp, "info"]);
    assert!(ok, "{err}");
    assert!(out.contains("node        0x03"), "{out}");
    assert!(out.contains("store       mounted"), "{out}");
    let (ok, out, err) = cli(&[&tcp, "--node", "rover-esp", "info"]);
    assert!(ok, "{err}");
    assert!(out.contains("node        0x03"), "{out}");

    // A motor controller is not a bridge.
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

/// A scratch file for one test, under cargo's per-test temporary directory.
fn scratch(name: &str) -> String {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("cli-config");
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name).to_string_lossy().into_owned()
}

/// The exit code of a run of the binary.
fn cli_code(args: &[&str]) -> (Option<i32>, String, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args(args)
        .output()
        .unwrap();
    (
        o.status.code(),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

#[test]
fn a_boards_store_as_a_file_through_the_binary() {
    // The refusals that need no board come first: they open no link.
    let (ok, _, err) = cli(&[
        "--tcp=127.0.0.1:9",
        "config",
        "rover-left",
        "control.mod",
        "balance",
    ]);
    assert!(!ok);
    assert_eq!(
        err.trim(),
        "error: unknown field control.mod (did you mean control.mode?)"
    );
    let (ok, _, err) = cli(&[
        "--tcp=127.0.0.1:9",
        "config",
        "x",
        "control.mode.1",
        "balance",
    ]);
    assert!(!ok);
    assert_eq!(
        err.trim(),
        "error: control.mode has one index (--force writes index 1 anyway)"
    );
    let missing = scratch("missing.json");
    let (code, _, err) = cli_code(&["--tcp=127.0.0.1:9", "diff", "x", &missing]);
    assert_eq!(code, Some(2), "{err}");
    // diff's 1 means changes alone: a usage error is 2 like any other, wherever it stands.
    for args in [
        &["--tcp=127.0.0.1:9", "diff", "x", "f.json", "--bogus"][..],
        &["--bogus", "--tcp=127.0.0.1:9", "diff", "x", "f.json"],
        &["--tcp=127.0.0.1:9", "diff", "--yes", "x", "f.json"],
        &["--tcp=127.0.0.1:9", "diff"],
        &[
            "--tcp=127.0.0.1:9",
            "--tcp=127.0.0.1:9",
            "diff",
            "x",
            "f.json",
        ],
    ] {
        let (code, _, err) = cli_code(args);
        assert_eq!(code, Some(2), "{args:?}: {err}");
        assert!(err.starts_with("error: "), "{err}");
    }
    let (code, _, err) = cli_code(&["--bogus", "--tcp=127.0.0.1:9", "apply", "x", "f.json"]);
    assert_eq!(
        (code, err.trim()),
        (Some(1), "error: unknown option --bogus")
    );
    // A flag before the command word still makes it a diff: a link that cannot be opened is 2.
    let empty = scratch("empty.json");
    std::fs::write(&empty, "{}").unwrap();
    for args in [
        &["--tcp", "127.0.0.1:9", "--yes", "diff", "x", &empty][..],
        &["--replace", "--tcp=127.0.0.1:9", "diff", "x", &empty],
        &["--tcp=127.0.0.1:9", "--replace", "diff", "x", &empty],
    ] {
        let (code, _, err) = cli_code(args);
        assert_eq!(code, Some(2), "{args:?}: {err}");
    }
    // An unknown command word is a usage error before any link is opened.
    let idle = TcpListener::bind("127.0.0.1:0").unwrap();
    let at = format!("--tcp={}", idle.local_addr().unwrap());
    for args in [
        &[at.as_str(), "bogus"][..],
        &["--yes", at.as_str(), "bogus", "x"],
    ] {
        let (code, _, err) = cli_code(args);
        assert_eq!(code, Some(1), "{args:?}: {err}");
        assert!(err.starts_with("error: usage: "), "{err}");
    }
    idle.set_nonblocking(true).unwrap();
    assert_eq!(
        idle.accept().map(|_| ()).unwrap_err().kind(),
        ErrorKind::WouldBlock,
        "an unknown command connected"
    );

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 14));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };

    // One field at a time, by name or by number.
    let (code, out, err) = run(&["config", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out.trim(),
        // The line starts with the name the board had when the command found it.
        "Hoverboard: device.name: \"Hoverboard\" (default) -> \"rover-left\""
    );
    let (code, out, err) = run(&["config", "rover-left", "control.mode", "balance"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out.trim(),
        "rover-left: control.mode: throttle (default) -> balance"
    );
    let (code, out, err) = run(&["config", "0x01", "0x4A.1", "PB5"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: motors[1].hall.a: none (default) -> PB5\npin fields take effect at the next boot\n"
    );
    let (code, _, err) = run(&["config", "rover-lft", "control.mode", "balance"]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("no node named rover-lft (nodes: rover-left, Hoverboard, hb-bridge-ab12)"),
        "{err}"
    );

    // Dump, then a diff against the dump reports nothing.
    let file = scratch("rover-left.json");
    let (code, _, err) = run(&["dump", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert!(
        err.contains("rover-left: 3 stored keys written to"),
        "{err}"
    );
    let dumped: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
    assert_eq!(
        dumped,
        serde_json::json!({
            "device": { "name": "rover-left" },
            "control": { "mode": "balance" },
            "motors": [{}, { "hall": { "a": "PB5" } }],
        })
    );
    let (code, out, err) = run(&["diff", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: 0 changed, 0 pinned, 3 unchanged");

    // A stray key: kept by a merge, un-stored by a replace.
    let (code, _, err) = run(&["config", "rover-left", "control.frame", "rigid"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, out, err) = run(&["diff", "--replace", "rover-left", &file]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "rover-left: 0 changed, 0 pinned, 3 unchanged, 1 un-stored\n\
         rover-left: control.frame: rigid -> pivoting (default)\n"
    );
    // Without a terminal, replace refuses rather than asking; with --yes it un-stores.
    let (code, _, err) = run(&["replace", "rover-left", &file]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("rover-left: replace would un-store 1 keys; give --yes"),
        "{err}"
    );
    let (code, out, err) = run(&["replace", "rover-left", &file, "--yes"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: 0 changed, 0 pinned, 3 unchanged, 1 un-stored\n\
         rover-left: control.frame: rigid -> pivoting (default)\n"
    );
    let (code, out, err) = run(&["compact", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: compact: ok");
    let (code, out, err) = run(&["diff", "--replace", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: 0 changed, 0 pinned, 3 unchanged");
    let (code, out, err) = run(&["config", "rover-left", "control.frame"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: control.frame = pivoting (default)");

    // apply merges: a file naming one changed key writes it and keeps the rest.
    let machine = scratch("machine.json");
    std::fs::write(&machine, r#"{ "control": { "mode": "throttle" } }"#).unwrap();
    let (code, out, err) = run(&["apply", "rover-left", &machine]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: 1 changed, 0 pinned, 0 unchanged, 2 kept\n\
         rover-left: control.mode: balance -> throttle\n\
         rover-left: kept: device.name = \"rover-left\" (stored, not in the file)\n\
         rover-left: kept: motors[1].hall.a = PB5 (stored, not in the file)\n"
    );
    server.join().unwrap();
}

/// `specs/bridge-onboarding.md` section 2.2 through the binary: `diff` of a file carrying the
/// password counts it as changed every time, prints it as `(set) -> (set)`, exits 1, and the value
/// is in no output.
#[test]
fn a_secret_through_the_binary_is_never_printed_and_diff_always_counts_it() {
    const PASS: &str = "hunter2hunter2";
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 3));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };
    let creds = scratch("wifi.json");
    std::fs::write(
        &creds,
        format!(r#"{{ "wifi": {{ "ssid": "bench", "pass": "{PASS}" }} }}"#),
    )
    .unwrap();

    let (code, out, err) = run(&["apply", "0x01", &creds]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "Hoverboard: 2 changed, 0 pinned, 0 unchanged\n\
         Hoverboard: wifi.ssid: \"\" (default) -> \"bench\"\n\
         Hoverboard: wifi.pass: (default) -> (set)\n"
    );
    assert!(!out.contains(PASS) && !err.contains(PASS), "{out}{err}");

    let (code, out, err) = run(&["diff", "0x01", &creds]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "Hoverboard: 1 changed, 0 pinned, 1 unchanged\n\
         Hoverboard: wifi.pass: (set) -> (set)\n"
    );
    assert!(!out.contains(PASS) && !err.contains(PASS), "{out}{err}");

    let (code, out, err) = run(&["config", "0x01", "wifi.pass"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "Hoverboard: wifi.pass = (set)");
    server.join().unwrap();
}

/// `specs/bridge-onboarding.md` section 3: a value longer than one `CONFIG_WRITE` carries is
/// refused by the file check, before `apply` opens a link and before `install-esp` touches the port.
#[test]
fn a_value_one_pdu_cannot_carry_is_refused_before_any_link_or_install() {
    let max = net::walk::CONFIG_WRITE_VALUE_MAX;
    let long = "p".repeat(max + 1);
    let creds = scratch("long-pass.json");
    std::fs::write(
        &creds,
        format!(r#"{{ "wifi": {{ "ssid": "bench", "pass": "{long}" }} }}"#),
    )
    .unwrap();
    let refusal = format!(
        "error: {creds}: wifi.pass: {} bytes, more than the {max} one CONFIG_WRITE carries",
        max + 1
    );

    let idle = TcpListener::bind("127.0.0.1:0").unwrap();
    let at = format!("--tcp={}", idle.local_addr().unwrap());
    let (code, out, err) = cli_code(&[&at, "apply", "0x01", &creds]);
    assert_eq!(code, Some(1));
    assert_eq!(err.trim(), refusal);
    assert!(!out.contains(&long) && !err.contains(&long));
    idle.set_nonblocking(true).unwrap();
    assert_eq!(
        idle.accept().map(|_| ()).unwrap_err().kind(),
        ErrorKind::WouldBlock,
        "apply connected"
    );

    // install-esp checks the files before the guard opens the port: the port named here does not
    // exist, so a run that reached it would fail on the port instead.
    let image = scratch("install.bin");
    let mut bytes = vec![0xFFu8; 4 * 1024 * 1024];
    bytes[0] = 0xE9;
    std::fs::write(&image, bytes).unwrap();
    let port = scratch("no-such-port");
    let (code, out, err) = cli_code(&["--serial", &port, "install-esp", &image, &creds, "--yes"]);
    assert_eq!(code, Some(1));
    assert_eq!(err.trim(), refusal);
    assert!(out.is_empty(), "{out}");
}

#[test]
fn an_address_needs_no_walk_and_a_name_does() {
    // `specs/cli-config.md` section 4.1: an address is reached by attaching and reading its
    // `device.name`, the slave included, which the bridge reaches through the master by flooding
    // until the reply teaches the route; a name needs the walk.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 6));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };
    let (code, out, err) = run(&["config", "0x02", "control.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "Hoverboard: control.mode = throttle (default)");
    let (code, _, err) = run(&["config", "0x02", "device.name", "rover-right"]);
    assert_eq!(code, Some(0), "{err}");
    // The prefix is the name the read returns.
    let (code, out, err) = run(&["config", "0x02", "control.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: control.mode = throttle (default)");
    let (code, out, err) = run(&["config", "rover-right", "control.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: control.mode = throttle (default)");
    // An address nothing answers: an error naming it, and no walk to fall back on.
    let (code, _, err) = run(&["config", "0x05", "control.mode"]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains(
            "no answer from 0x05; `hoverboard-cli nodes` lists the nodes this link reaches"
        ),
        "{err}"
    );
    let (code, out, err) = run(&["compact", "0x02"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: compact: ok");

    let after = server.join().unwrap();
    let probe = net::pdu::Opcode::ProbePorts.to_u8();
    let hello = net::pdu::Opcode::NodeHello.to_u8();
    for (i, sent) in after.ops.iter().enumerate() {
        assert_eq!(sent.first(), Some(&hello), "connection {i} attaches first");
        assert_eq!(
            sent.contains(&probe),
            i == 3,
            "connection {i} sent {sent:02x?}: only the one naming a node walks"
        );
    }
}

#[test]
fn fields_json_is_the_committed_name_table() {
    // `tools/robo-presets.py`'s test checks its names against this file, so the file must be what
    // the binary prints: regenerate it with `hoverboard-cli fields --json > fields.json`.
    let (ok, out, err) = cli(&["fields", "--json"]);
    assert!(ok, "{err}");
    // A checkout that turned the file's line ends into CRLF still holds the same table.
    let committed = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/fields.json"))
        .unwrap_or_default()
        .replace("\r\n", "\n");
    assert!(
        out == committed,
        "tools/hoverboard-cli/fields.json is not `hoverboard-cli fields --json`'s output"
    );
}

/// A board's own end of a link, its BLE module's place: one `net` responder on port 0, no bridge.
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
    fn window(&self) -> usize {
        espsvc::WINDOW_STREAM
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
        let mut b = hoverboard_bridge_core::sim::SimBoard::holding(0x01);
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
