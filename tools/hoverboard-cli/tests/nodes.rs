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
/// `DRIVE_CMD`s each board took, the opcodes each client connection sent, and the master's store
/// pages on its flash (the simulated GD32's, which `STORE_ERASE` reaches).
struct After {
    esp: u8,
    name: Vec<u8>,
    drives: [usize; 2],
    ops: Vec<Vec<u8>>,
    store_pages: Vec<u16>,
}

/// The store pages' first halfword in the simulated GD32's flash.
fn store_half() -> usize {
    use hoverboard_bridge_core::part::{FLASH_BASE, GD32F130C8};
    ((GD32F130C8.store_start() - FLASH_BASE) / 2) as usize
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
    serve_with(listener, connections, |_| {})
}

/// [`serve`], with the machine set up by `setup` first.
fn serve_with(
    listener: TcpListener,
    connections: usize,
    setup: impl FnOnce(&mut Machine<'static>),
) -> After {
    serve_heard(listener, connections, setup, Vec::new())
}

/// [`serve_with`], with the master putting each of `beat` (a `CYCLIC_STATE` PDU, its own or one
/// it forwards from the slave) on its mailbox every 100 ms, which the bridge re-publishes on its
/// links as the firmware's frames are.
fn serve_heard(
    listener: TcpListener,
    connections: usize,
    setup: impl FnOnce(&mut Machine<'static>),
    beat: Vec<Vec<u8>>,
) -> After {
    use hoverboard_cli::wire::Wire;
    let mut m = machine();
    setup(&mut m);
    let mut next_beat = std::time::Instant::now();
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
            if !beat.is_empty() && std::time::Instant::now() >= next_beat {
                for f in &beat {
                    let _ = m.fw.link.send(f);
                }
                next_beat += Duration::from_millis(100);
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
        store_pages: m.port.swd().pins().bus.flash[store_half()..].to_vec(),
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
fn the_noun_commands_by_target() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 10));

    // The walk: the fresh bridge ends at 0x03 and the boards keep theirs.
    let (ok, out, err) = cli(&[&tcp, "machine", "nodes"]);
    assert!(ok, "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert_eq!(rows.len(), 4, "{out}");
    assert_eq!(
        rows[0],
        "address  class             uid                       name"
    );
    // Each node's `IDENTITY`: a board's factory unique ID, the bridge's MAC and six zero bytes.
    // The simulated boards send no CYCLIC_STATE, so each is marked silent.
    assert_eq!(
        rows[1],
        "0x01     motor controller  010101010101010101010101  Hoverboard  ! silent"
    );
    assert_eq!(
        rows[2],
        "0x02     motor controller  020202020202020202020202  Hoverboard  ! silent"
    );
    assert_eq!(
        rows[3],
        "0x03     bridge            00000000ab12000000000000  hb-bridge-ab12"
    );

    // A name the bridge cannot advertise is refused with the rule, and the old one stays.
    let (ok, out, err) = cli(&[
        &tcp,
        "bridge",
        "set",
        "0x03",
        "device.name",
        "Offroad master",
    ]);
    assert!(!ok);
    assert!(out.is_empty(), "{out}");
    let rule = "name must be 1 to 26 letters, digits or '-'";
    assert!(
        err.contains(&format!("hb-bridge-ab12: device.name: refused: {rule}")),
        "{err}"
    );

    // A name, then the node by that name.
    let (ok, out, err) = cli(&[&tcp, "bridge", "set", "0x03", "device.name", "rover-esp"]);
    assert!(ok, "{err}");
    assert_eq!(
        out,
        "hb-bridge-ab12: device.name: \"hb-bridge-ab12\" (default) -> \"rover-esp\"\n\
         1 takes effect at the next boot\n"
    );
    let (ok, out, err) = cli(&[&tcp, "bridge", "get", "rover-esp", "0x02"]);
    assert!(ok, "{err}");
    assert_eq!(out.trim(), "rover-esp: device.name = \"rover-esp\"");

    // A key the board's image does not know: forced past the name table, refused by the board.
    let (ok, _, err) = cli(&[&tcp, "board", "set", "0x01", "0x99", "5", "--force"]);
    assert!(!ok);
    assert!(
        err.contains("warning: 0x99 is not in the name table"),
        "{err}"
    );
    assert!(
        err.contains("0x99.0: the board does not know this key"),
        "{err}"
    );

    // A bridge command to the bridge on the link by its address, and by name through the walk.
    let (ok, out, err) = cli(&[&tcp, "bridge", "info", "0x03"]);
    assert!(ok, "{err}");
    assert!(out.contains("node        0x03"), "{out}");
    assert!(out.contains("store       mounted"), "{out}");
    let (ok, out, err) = cli(&[&tcp, "bridge", "info", "rover-esp"]);
    assert!(ok, "{err}");
    assert!(out.contains("node        0x03"), "{out}");

    // A motor controller is not a bridge.
    let (ok, _, err) = cli(&[&tcp, "bridge", "info", "0x01"]);
    assert!(!ok);
    assert!(
        err.contains("0x01 (0101...) is a motor controller, not a bridge: use the board commands"),
        "{err}"
    );

    // `machine drive` with no boards drives the motor controllers the walk found.
    let (ok, out, err) = cli(&[&tcp, "machine", "drive", "300", "1"]);
    assert!(ok, "{err}");
    assert!(out.contains("neutral sent, power 0 sent"), "{out}");

    // `machine watch` with boards given by address only attaches.
    let (ok, _, err) = cli(&[&tcp, "machine", "watch", "1", "0x01"]);
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
        "board",
        "set",
        "rover-left",
        "machine.mod",
        "balance",
    ]);
    assert!(!ok);
    assert_eq!(
        err.trim(),
        "error: unknown field machine.mod (did you mean machine.mode?)"
    );
    let (ok, _, err) = cli(&[
        "--tcp=127.0.0.1:9",
        "board",
        "set",
        "x",
        "machine.mode.1",
        "balance",
    ]);
    assert!(!ok);
    assert_eq!(
        err.trim(),
        "error: machine.mode has one index (--force writes index 1 anyway)"
    );
    let missing = scratch("missing.json");
    let (code, _, err) = cli_code(&["--tcp=127.0.0.1:9", "board", "diff", "x", &missing]);
    assert_eq!(code, Some(2), "{err}");
    // diff's 1 means changes alone: a usage error is 2 like any other, wherever it stands.
    for args in [
        &[
            "--tcp=127.0.0.1:9",
            "board",
            "diff",
            "x",
            "f.json",
            "--bogus",
        ][..],
        &[
            "--bogus",
            "--tcp=127.0.0.1:9",
            "board",
            "diff",
            "x",
            "f.json",
        ],
        &["--tcp=127.0.0.1:9", "board", "diff", "--yes", "x", "f.json"],
        &["--tcp=127.0.0.1:9", "board", "diff"],
        &["--tcp=127.0.0.1:9", "machine", "diff", "--yes"],
        &[
            "--tcp=127.0.0.1:9",
            "--tcp=127.0.0.1:9",
            "board",
            "diff",
            "x",
            "f.json",
        ],
    ] {
        let (code, _, err) = cli_code(args);
        assert_eq!(code, Some(2), "{args:?}: {err}");
        assert!(err.starts_with("error: "), "{err}");
    }
    let (code, _, err) = cli_code(&[
        "--bogus",
        "--tcp=127.0.0.1:9",
        "board",
        "apply",
        "x",
        "f.json",
    ]);
    assert_eq!(
        (code, err.trim()),
        (Some(1), "error: unknown option --bogus")
    );
    // A flag before the command word still makes it a diff: a link that cannot be opened is 2.
    let empty = scratch("empty.json");
    std::fs::write(&empty, "{}").unwrap();
    for args in [
        &[
            "--tcp",
            "127.0.0.1:9",
            "--force",
            "board",
            "diff",
            "x",
            &empty,
        ][..],
        &[
            "--replace",
            "--tcp=127.0.0.1:9",
            "board",
            "diff",
            "x",
            &empty,
        ],
        &[
            "--tcp=127.0.0.1:9",
            "--replace",
            "board",
            "diff",
            "x",
            &empty,
        ],
    ] {
        let (code, _, err) = cli_code(args);
        assert_eq!(code, Some(2), "{args:?}: {err}");
    }
    // An unknown command, or an old flat one, is a usage error before any link is opened.
    let idle = TcpListener::bind("127.0.0.1:0").unwrap();
    let at = format!("--tcp={}", idle.local_addr().unwrap());
    for args in [
        &[at.as_str(), "bogus"][..],
        &["--yes", at.as_str(), "bogus", "x"],
        &[at.as_str(), "nodes"],
        &[at.as_str(), "config", "0x01", "machine.mode"],
        &[at.as_str(), "board", "nodes"],
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
    let server = std::thread::spawn(move || serve(listener, 15));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };

    // One field at a time, by name or by number.
    let (code, out, err) = run(&["board", "set", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        // The line starts with the name the board had when the command found it.
        "Hoverboard: device.name: \"Hoverboard\" (default) -> \"rover-left\"\n\
         1 takes effect at the next boot\n"
    );
    let (code, out, err) = run(&["board", "set", "rover-left", "machine.mode", "balance"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: machine.mode: throttle (default) -> balance\n\
         1 takes effect at the next arm\n\
         machine fields changed on rover-left only: the boards now differ (\"machine diff\")\n"
    );
    let (code, out, err) = run(&["board", "set", "0x01", "0x40.1", "PB5"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: motors[1].hall.a: none (default) -> PB5\n1 takes effect at the next boot\n"
    );
    let (code, _, err) = run(&["board", "set", "rover-lft", "machine.mode", "balance"]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("no node named rover-lft (nodes: rover-left, Hoverboard, hb-bridge-ab12)"),
        "{err}"
    );

    // Dump, then a diff against the dump reports nothing.
    let file = scratch("rover-left.json");
    let (code, _, err) = run(&["board", "dump", "rover-left", &file]);
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
            "machine": { "mode": "balance" },
            "motors": [{}, { "hall": { "a": "PB5" } }],
        })
    );
    let (code, out, err) = run(&["board", "diff", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: 0 changed, 0 pinned, 3 unchanged");

    // A stray key: kept by a merge, un-stored by a replace.
    let (code, _, err) = run(&["board", "set", "rover-left", "machine.frame", "rigid"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, out, err) = run(&["board", "diff", "--replace", "rover-left", &file]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "rover-left: 0 changed, 0 pinned, 3 unchanged, 1 un-stored\n\
         rover-left: machine.frame: rigid -> pivoting (default)\n\
         1 takes effect at the next arm\n"
    );
    // Without a terminal, replace refuses rather than asking; with --yes it un-stores.
    let (code, _, err) = run(&["board", "replace", "rover-left", &file]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("rover-left: replace would un-store 1 keys; give --yes"),
        "{err}"
    );
    let (code, out, err) = run(&["board", "replace", "rover-left", &file, "--yes"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: 0 changed, 0 pinned, 3 unchanged, 1 un-stored\n\
         rover-left: machine.frame: rigid -> pivoting (default)\n\
         1 takes effect at the next arm\n\
         machine fields changed on rover-left only: the boards now differ (\"machine diff\")\n"
    );
    let (code, out, err) = run(&["board", "compact", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: compact: ok");
    let (code, out, err) = run(&["board", "diff", "--replace", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: 0 changed, 0 pinned, 3 unchanged");
    let (code, out, err) = run(&["board", "get", "rover-left", "machine.frame"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: machine.frame = pivoting (default)");

    // apply merges: a file naming one changed key writes it and keeps the rest.
    let machine = scratch("machine.json");
    std::fs::write(&machine, r#"{ "machine": { "mode": "throttle" } }"#).unwrap();
    let (code, out, err) = run(&["board", "apply", "rover-left", &machine]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: 1 changed, 0 pinned, 0 unchanged, 2 kept\n\
         rover-left: machine.mode: balance -> throttle\n\
         rover-left: kept: device.name = \"rover-left\" (stored, not in the file)\n\
         rover-left: kept: motors[1].hall.a = PB5 (stored, not in the file)\n\
         1 takes effect at the next arm\n\
         machine fields changed on rover-left only: the boards now differ (\"machine diff\")\n"
    );
    // A pin ends with where to see what the board made of it.
    let links = scratch("links.json");
    std::fs::write(&links, r#"{ "links": [{ "tx": "PA2", "rx": "PA3" }] }"#).unwrap();
    let (code, out, err) = run(&["board", "apply", "rover-left", &links]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: 2 changed, 0 pinned, 0 unchanged, 3 kept\n\
         rover-left: links[0].tx: none (default) -> PA2\n\
         rover-left: links[0].rx: none (default) -> PA3\n\
         rover-left: kept: device.name = \"rover-left\" (stored, not in the file)\n\
         rover-left: kept: machine.mode = throttle (stored, not in the file)\n\
         rover-left: kept: motors[1].hall.a = PB5 (stored, not in the file)\n\
         2 take effect at the next boot\n\
         check the boot report with \"board status\" after the next boot\n"
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

    let (code, out, err) = run(&["board", "apply", "0x01", &creds]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "Hoverboard: 2 changed, 0 pinned, 0 unchanged\n\
         Hoverboard: wifi.ssid: \"\" (default) -> \"bench\"\n\
         Hoverboard: wifi.pass: (default) -> (set)\n\
         2 take effect at the next boot\n"
    );
    assert!(!out.contains(PASS) && !err.contains(PASS), "{out}{err}");

    let (code, out, err) = run(&["board", "diff", "0x01", &creds]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "Hoverboard: 1 changed, 0 pinned, 1 unchanged\n\
         Hoverboard: wifi.pass: (set) -> (set)\n\
         1 takes effect at the next boot\n"
    );
    assert!(!out.contains(PASS) && !err.contains(PASS), "{out}{err}");

    let (code, out, err) = run(&["board", "get", "0x01", "wifi.pass"]);
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
    let (code, out, err) = cli_code(&[&at, "board", "apply", "0x01", &creds]);
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
    let (code, out, err) = cli_code(&[
        "--serial", &port, "bridge", "install", &image, &creds, "--yes",
    ]);
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
    let (code, out, err) = run(&["board", "get", "0x02", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "Hoverboard: machine.mode = throttle (default)");
    let (code, _, err) = run(&["board", "set", "0x02", "device.name", "rover-right"]);
    assert_eq!(code, Some(0), "{err}");
    // The prefix is the name the read returns.
    let (code, out, err) = run(&["board", "get", "0x02", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: machine.mode = throttle (default)");
    let (code, out, err) = run(&["board", "get", "rover-right", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: machine.mode = throttle (default)");
    // An address nothing answers: an error naming it, and no walk to fall back on.
    let (code, _, err) = run(&["board", "get", "0x05", "machine.mode"]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains(
            "no answer from 0x05; `hoverboard-cli machine nodes` lists the nodes this link \
             reaches"
        ),
        "{err}"
    );
    let (code, out, err) = run(&["board", "compact", "0x02"]);
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
    // the binary prints: regenerate it with `hoverboard-cli board fields --json > fields.json`.
    let (ok, out, err) = cli(&["board", "fields", "--json"]);
    assert!(ok, "{err}");
    // A checkout that turned the file's line ends into CRLF still holds the same table.
    let committed = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/fields.json"))
        .unwrap_or_default()
        .replace("\r\n", "\n");
    assert!(
        out == committed,
        "tools/hoverboard-cli/fields.json is not `hoverboard-cli board fields --json`'s output"
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
    let (ok, _, err) = cli(&[&tcp, "bridge", "info", "0x01"]);
    assert!(!ok);
    assert!(err.contains("motor controller, not a bridge"), "{err}");
    drop(server);
}

/// `status` and the `nodes` marker over a simulated machine whose master reports one entry of each
/// reason in its boot report (`STATUS_READ`, `specs/configured-links.md` section 5) and whose
/// slave reports none.
#[test]
fn status_reads_the_boot_report_from_each_motor_controller() {
    use board::status::Reason::*;
    const LINK: u8 = store::LINK_TX.id();
    static ENTRIES: [u8; 28] = [
        store::MOTOR_HALL_A.id(),
        0,
        NotAPin as u8,
        0x47, // hall: a byte that is no pin
        LINK,
        0,
        Incomplete as u8,
        store::LINK_RX.id(), // link 0: rx unset
        LINK,
        1,
        NotRouted as u8,
        0x16, // link 1: no USART
        store::LED_GREEN.id(),
        0,
        SwdPin as u8,
        0x0D, // leds.green on SWDIO
        store::MOTOR_GATE_HI_A.id(),
        0,
        DeadTime as u8,
        10, // gate: dead time 10
        store::IMU_SCL_PIN.id(),
        0,
        NotUp as u8,
        0, // imu: no probe answer
        store::IMU_AXIS_SIGN.id(),
        0,
        ImuFrame as u8,
        0, // imu: the sign triple refused
    ];
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || {
        serve_with(listener, 3, |m| {
            m.master.resp.set_status(&ENTRIES);
        })
    });

    let (ok, out, err) = cli(&[&tcp, "board", "status"]);
    assert!(ok, "{err}");
    let lines: Vec<&str> = out.lines().collect();
    assert_eq!(
        lines,
        [
            "Hoverboard: motors[0] hall: stored byte 0x47 is not a pin",
            "Hoverboard: links[0]: incomplete, links[0].rx is not set",
            "Hoverboard: links[1]: no USART routes this pair on this chip",
            "Hoverboard: leds.green: PA13 is an SWD pin",
            "Hoverboard: motors[0] gate: dead time 10 is below the floor of 18",
            "Hoverboard: imu: the IMU did not answer its probe",
            "Hoverboard: imu: imu.axis_sign is not a rotation",
            "Hoverboard: ok",
        ],
        "{out}"
    );

    let (ok, out, err) = cli(&[&tcp, "board", "status", "0x02"]);
    assert!(ok, "{err}");
    assert_eq!(out, "Hoverboard: ok\n");

    let (ok, out, err) = cli(&[&tcp, "machine", "nodes"]);
    assert!(ok, "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert!(
        rows[1].starts_with("0x01")
            && rows[1].ends_with("! silent, 7 status entries (\"board status\")"),
        "{out}"
    );
    assert!(rows[2].ends_with("Hoverboard  ! silent"), "{out}");
    server.join().unwrap();
}

/// `specs/machine-settings.md` section 6.1: the walk asks every address it found for its
/// `IDENTITY` and keeps it with the node.
#[test]
fn the_walk_records_each_nodes_identity() {
    use hoverboard_cli::nodes::{self, Identity};
    use net::walk::{CLASS_BRIDGE, CLASS_MOTOR, MCU_ESP32C3, MCU_F1X0};
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let at = listener.local_addr().unwrap();
    let server = std::thread::spawn(move || serve(listener, 1));
    let found = {
        let sock = std::net::TcpStream::connect(at).unwrap();
        let mut wire = hoverboard_cli::wire::Framed::new(Sock(sock));
        let walked = nodes::walk(&mut wire).unwrap();
        nodes::survey(&mut wire, &walked)
    };
    server.join().unwrap();
    let board = |uid: u8| Identity {
        class: CLASS_MOTOR,
        mcu: MCU_F1X0,
        fw_ver: 0x0001,
        uid: [uid; 12],
    };
    let got: Vec<(u8, Option<Identity>)> = found.iter().map(|f| (f.addr, f.identity)).collect();
    assert_eq!(
        got,
        [
            (0x01, Some(board(0x01))),
            (0x02, Some(board(0x02))),
            (
                0x03,
                Some(Identity {
                    class: CLASS_BRIDGE,
                    mcu: MCU_ESP32C3,
                    fw_ver: 0x0001,
                    uid: [0, 0, 0, 0, 0xAB, 0x12, 0, 0, 0, 0, 0, 0],
                })
            ),
        ]
    );
    assert_eq!(found[2].class(), Some(CLASS_BRIDGE));
}

#[test]
fn erase_store_prints_what_a_dump_leaves_out_refuses_without_a_terminal_and_erases_with_yes() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    // The master's flash holds records in its store pages.
    let server = std::thread::spawn(move || {
        serve_with(listener, 5, |m| {
            let at = store_half();
            for h in &mut m.port.swd().pins().bus.flash[at..at + 16] {
                *h = 0x1234;
            }
        })
    });
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };

    let (code, _, err) = run(&["board", "set", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, _, err) = run(&[
        "board",
        "set",
        "0x01",
        "motors.0.odometer",
        "123456",
        "--force",
    ]);
    assert_eq!(code, Some(0), "{err}");

    // Usage: a board and --yes, and no link opened for it.
    let (code, _, err) = cli_code(&["--tcp=127.0.0.1:9", "board", "erase", "0x01", "now"]);
    assert_eq!(code, Some(1));
    assert!(err.starts_with("error: usage: "), "{err}");

    // Without a terminal (the test's stdin is not one) and without --yes: the lines, then the
    // refusal, and no STORE_ERASE sent. The board's own bridge is the one on the link.
    let restore = "  write it back with: hoverboard-cli board set rover-left motors.0.odometer \
                   123456 --force";
    let lines = |out: &str| out.lines().map(str::to_string).collect::<Vec<_>>();
    let (code, out, err) = run(&["board", "erase", "0x01"]);
    assert_eq!(code, Some(1), "{out}{err}");
    assert_eq!(
        lines(&out),
        [
            "rover-left: node.address = 1",
            "rover-left: motors[0].odometer = 123456",
            restore,
            "rover-left: motors[1].odometer = 0 (default)",
        ],
        "{out}"
    );
    assert!(
        err.contains("rover-left: board erase erases every stored setting; give --yes"),
        "{err}"
    );

    // By the board's name too; with --yes it erases, waits for the mailbox, and gives the board
    // its old address back through the bridge's mailbox port.
    let (code, out, err) = run(&["board", "erase", "rover-left", "--yes"]);
    assert_eq!(code, Some(0), "{out}{err}");
    assert_eq!(lines(&out)[2], restore, "{out}");
    assert_eq!(
        lines(&out)[4..],
        ["erased", "node.address 1 restored"],
        "{out}"
    );

    // The slave has no bridge on its mailbox: refused before anything is erased.
    let (code, _, err) = run(&["board", "erase", "0x02", "--yes"]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("no bridge is wired to 0x02 (Hoverboard, 0202...)"),
        "{err}"
    );

    let after = server.join().unwrap();
    let erase = espsvc::OP_STORE_ERASE;
    let sent: Vec<bool> = after.ops.iter().map(|o| o.contains(&erase)).collect();
    assert_eq!(
        sent,
        [false, false, false, true, false],
        "{:02x?}",
        after.ops
    );
    // The erasing connection's last request is the ASSIGN that restores the address.
    let assign = net::pdu::Opcode::Assign.to_u8();
    assert_eq!(after.ops[3].last(), Some(&assign), "{:02x?}", after.ops[3]);
    assert!(
        after.store_pages.iter().all(|&h| h == 0xFFFF),
        "the master's store pages are erased"
    );
}

#[test]
fn an_swd_command_to_an_address_on_the_connected_bridges_mailbox_skips_the_walk() {
    // `specs/machine-settings.md` section 6.3: the tool attaches, sends the bridge one
    // `PROBE_PORTS`, and when its mailbox neighbour is the address sends the command straight
    // through it. Another address, a name and a `uid` prefix walk.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 7));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };

    // The walk gives the fresh bridge its address, 0x03.
    let (code, _, err) = run(&["machine", "nodes"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, _, err) = run(&["board", "target-id", "0x01"]);
    assert_eq!(code, Some(0), "{err}");
    // The slave is not on the bridge's mailbox: walked, and refused as before.
    let (code, _, err) = run(&["board", "target-id", "0x02"]);
    assert_eq!(code, Some(1));
    assert!(err.contains("no bridge is wired to 0x02"), "{err}");
    let (code, _, err) = run(&["board", "set", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, _, err) = run(&["board", "target-id", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, _, err) = run(&["board", "target-id", "01010101"]);
    assert_eq!(code, Some(0), "{err}");
    // board erase through the same check: the address's own bridge, with no walk.
    let (code, out, err) = run(&["board", "erase", "0x01", "--yes"]);
    assert_eq!(code, Some(0), "{out}{err}");
    assert!(out.starts_with("rover-left: node.address = 1"), "{out}");
    assert!(out.ends_with("erased\nnode.address 1 restored\n"), "{out}");

    let after = server.join().unwrap();
    let probe = net::pdu::Opcode::ProbePorts.to_u8();
    let identify = net::pdu::Opcode::Identify.to_u8();
    let probes: Vec<usize> = after
        .ops
        .iter()
        .map(|o| o.iter().filter(|&&op| op == probe).count())
        .collect();
    // The straight connections send one `PROBE_PORTS`; the walked ones probe the bridge and both
    // boards, the first walk once more as the fresh bridge moves to 0x03, and the `board set` by
    // address sends none.
    assert_eq!(probes, [4, 1, 3, 0, 3, 3, 1], "{:02x?}", after.ops);
    // The straight one asks the attached node alone whether it is a bridge, after the probe
    // (`specs/board-route.md` section 5).
    let hello = net::pdu::Opcode::NodeHello.to_u8();
    let at = |ops: &[u8], op| ops.iter().position(|&o| o == op);
    assert!(
        at(&after.ops[1], probe) < at(&after.ops[1], identify),
        "{:02x?}",
        after.ops[1]
    );
    assert_eq!(after.ops[1].iter().filter(|&&op| op == identify).count(), 1);
    // The uid prefix the bridge's SWD target does not report: one TARGET_ID, then the walk.
    assert_eq!(
        after.ops[5][..4],
        [hello, identify, espsvc::OP_TARGET_ID_REQ, probe],
        "{:02x?}",
        after.ops[5]
    );
    assert_eq!(
        after.ops[6].iter().filter(|&&op| op == identify).count(),
        2,
        "erase asks the bridge and the board alone: {:02x?}",
        after.ops[6]
    );
    assert!(after.ops[6].contains(&espsvc::OP_STORE_ERASE));
}

/// `specs/board-route.md` sections 1 and 2 through the binary: a `uid` prefix of the target the
/// connected bridge reads over SWD goes straight through it (no walk, no `PROBE_PORTS`), and
/// `board read --hz` routes once and streams its samples on that one connection.
#[test]
fn a_uid_prefix_goes_straight_and_a_read_streams_on_one_connection() {
    use std::io::BufRead;
    // The simulated GD32's unique ID, which `TARGET_ID` reads.
    const SWD_UID: &str = "8adc39320102030405060708";
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 2));

    let (code, out, err) = cli_code(&[&tcp, "board", "target-id", "8ADC"]);
    assert_eq!(code, Some(0), "{err}");
    assert!(out.contains(&format!("uid         {SWD_UID}")), "{out}");

    let mut child = Command::new(env!("CARGO_BIN_EXE_hoverboard-cli"))
        .args([
            &tcp,
            "board",
            "read",
            "8adc",
            "0x20000000",
            "16",
            "--hz",
            "20",
        ])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::null())
        .spawn()
        .unwrap();
    let stdout = child.stdout.take().unwrap();
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for l in std::io::BufReader::new(stdout)
            .lines()
            .map_while(Result::ok)
        {
            let _ = tx.send(l);
        }
    });
    let t0 = std::time::Instant::now();
    let mut lines = Vec::new();
    while t0.elapsed() < Duration::from_millis(1500) {
        if let Ok(l) = rx.recv_timeout(Duration::from_millis(50)) {
            lines.push(l);
        }
    }
    child.kill().unwrap();
    child.wait().unwrap();

    // Each line: the seconds since the start, to the millisecond, then the 16 bytes; at most one
    // per 50 ms slot, none made up.
    assert!((3..=31).contains(&lines.len()), "{lines:?}");
    let mut last = -1.0;
    for l in &lines {
        let (t, hex) = l.split_once(' ').unwrap();
        assert_eq!(t.split_once('.').map(|(_, ms)| ms.len()), Some(3), "{l}");
        let t: f64 = t.parse().unwrap();
        assert!(t > last, "{lines:?}");
        last = t;
        assert_eq!(hex.len(), 32, "{l}");
        assert!(hex.bytes().all(|b| b.is_ascii_hexdigit()), "{l}");
    }

    let after = server.join().unwrap();
    let hello = net::pdu::Opcode::NodeHello.to_u8();
    let identify = net::pdu::Opcode::Identify.to_u8();
    let id = espsvc::OP_TARGET_ID_REQ;
    assert_eq!(
        after.ops[0],
        [hello, identify, id, id],
        "{:02x?}",
        after.ops[0]
    );
    let stream = &after.ops[1];
    assert_eq!(stream[..3], [hello, identify, id], "{stream:02x?}");
    let reads = stream
        .iter()
        .filter(|&&op| op == espsvc::OP_MEM_READ)
        .count();
    assert_eq!(
        stream.len(),
        3 + reads,
        "one route, then reads: {stream:02x?}"
    );
    // The kill can land between a read and its line.
    assert!(
        reads == lines.len() || reads == lines.len() + 1,
        "{reads} reads, {} lines",
        lines.len()
    );
}

/// A `CYCLIC_STATE` PDU from board `src` with `fault`, `flags` and the machine digest `machine`,
/// every other word zero, as a board sends it to `0x00` on its mailbox.
fn cyclic(src: u8, fault: u16, flags: u8, machine: u16) -> Vec<u8> {
    use linkctl::{ChipTag, CyclicObs, CyclicState};
    let s = CyclicState {
        pitch: 0,
        roll: 0,
        wheel_speed: 0,
        battery: 0,
        mode: 0,
        fault,
        flags,
        peer_demand: 0,
        wheel_pos: 0,
        obs: CyclicObs {
            phase_peak: 0,
            phase_mean: 0,
            duty_on: 0,
            boot_tag: 0,
            chip: ChipTag::Unknown,
            odometer: 0,
        },
        wheel_rate: 0,
        machine,
    };
    let mut b = [0u8; CyclicState::ENCODED_LEN];
    let n = s.encode(&mut b);
    let mut buf = [0u8; 64];
    let k = net::pdu::Pdu::new(linkctl::OP_CYCLIC_STATE, src, 0x00, &b[..n])
        .unwrap()
        .encode(&mut buf)
        .unwrap();
    buf[..k].to_vec()
}

/// `specs/machine-settings.md` section 6.3: `machine nodes` marks each board from the frames it
/// hears after the walk, `EV_MACHINE` by the cause `FLAG_PEER_SILENT` names (`specs/board-route.md`
/// section 4), and `machine apply` ends with whether the reported digests agree.
#[test]
fn machine_nodes_marks_each_board_from_its_frames() {
    use linkctl::CyclicState as C;
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let beat = vec![
        cyclic(0x01, C::FAULT_MACHINE, C::FLAG_REARM, 0x1234),
        cyclic(
            0x02,
            C::FAULT_MACHINE,
            C::FLAG_REBOOT | C::FLAG_PEER_SILENT,
            0x5678,
        ),
    ];
    let server = std::thread::spawn(move || serve_heard(listener, 2, |_| {}, beat));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };
    let (code, out, err) = run(&["machine", "nodes"]);
    assert_eq!(code, Some(0), "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert_eq!(
        rows[1],
        "0x01     motor controller  010101010101010101010101  Hoverboard  ! machine differs \
         (\"machine diff\"), written, runs at the next arm"
    );
    assert_eq!(
        rows[2],
        "0x02     motor controller  020202020202020202020202  Hoverboard  ! a link peer not \
         heard, written, runs at the next boot"
    );
    assert!(rows[3].ends_with("hb-bridge-ab12"), "{out}");

    let file = scratch("heard.json");
    std::fs::write(&file, r#"{ "machine": { "mode": "balance" } }"#).unwrap();
    let (code, out, err) = run(&["machine", "apply", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out.lines().last(),
        Some("machine digests differ: Hoverboard 0x1234, Hoverboard 0x5678 (\"machine diff\")"),
        "{out}"
    );
    server.join().unwrap();
}

/// The `machine` commands over the simulated machine: a field to every board, the copies that
/// differ, the machine file out and back, and the files of the wrong kind refused.
#[test]
fn the_machine_commands_through_the_binary() {
    let left = "010101010101010101010101";
    let right = "020202020202020202020202";
    // The refusals that need no machine open no link.
    let idle = "--tcp=127.0.0.1:9";
    let (code, _, err) = cli_code(&[idle, "machine", "get", "leds.green"]);
    assert_eq!(code, Some(1));
    assert!(err.contains("leds.green is not a machine field"), "{err}");
    let machine_file = scratch("machine-kind.json");
    std::fs::write(&machine_file, r#"{ "boards": {} }"#).unwrap();
    let (code, _, err) = cli_code(&[idle, "board", "apply", "0x01", &machine_file]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("a machine file (it has \"boards\"): give it to the machine commands"),
        "{err}"
    );
    let board_file = scratch("board-kind.json");
    std::fs::write(&board_file, r#"{ "leds": { "green": "PB3" } }"#).unwrap();
    let (code, _, err) = cli_code(&[idle, "machine", "apply", &board_file]);
    assert_eq!(code, Some(1));
    assert!(
        err.contains("is a board file (it has \"leds\"): give it to the board commands"),
        "{err}"
    );

    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 15));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };
    let (code, _, err) = run(&["board", "set", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, _, err) = run(&["board", "set", "0x02", "device.name", "rover-right"]);
    assert_eq!(code, Some(0), "{err}");

    // A machine field to every motor controller, read back as one value.
    let (code, out, err) = run(&["machine", "set", "machine.mode", "balance"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: machine.mode: throttle (default) -> balance\n\
         rover-right: machine.mode: throttle (default) -> balance\n\
         2 take effect at the next arm\n"
    );
    let (code, out, err) = run(&["machine", "get", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out, "machine.mode = balance\n");

    // One board's copy changed through `board`: the boards differ, and `machine diff` says how.
    let (code, out, err) = run(&["board", "set", "rover-left", "machine.frame", "rigid"]);
    assert_eq!(code, Some(0), "{err}");
    assert!(
        out.ends_with(
            "machine fields changed on rover-left only: the boards now differ (\"machine diff\")\n"
        ),
        "{out}"
    );
    let (code, out, err) = run(&["machine", "diff"]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "machine.frame: rover-left rigid, rover-right pivoting (default)\n"
    );
    let (code, out, err) = run(&["machine", "get", "machine.frame"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: machine.frame = rigid\nrover-right: machine.frame = pivoting (default)\n"
    );

    // The dump refuses copies that differ unless told which to write.
    let file = scratch("machine.json");
    let (code, _, err) = run(&["machine", "dump", &file]);
    assert_eq!(code, Some(1));
    assert!(err.contains("give --from <board>"), "{err}");
    assert!(
        err.contains("machine.frame: rover-left rigid, rover-right pivoting (default)"),
        "{err}"
    );
    let (code, _, err) = run(&["machine", "dump", &file, "--from", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    let dumped: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
    assert_eq!(
        dumped,
        serde_json::json!({
            "machine": { "mode": "balance", "frame": "rigid" },
            "boards": {
                left: { "device": { "name": "rover-left" } },
                right: { "device": { "name": "rover-right" } },
            },
        })
    );

    // Applied back, every board takes the machine section and its own entry; the digests are
    // not heard from the simulated boards.
    let (code, out, err) = run(&["machine", "apply", &file]);
    assert_eq!(code, Some(0), "{err}");
    let lines: Vec<&str> = out.lines().collect();
    assert!(
        lines.contains(&"rover-left: 0 changed, 0 pinned, 3 unchanged"),
        "{out}"
    );
    assert!(
        lines.contains(&"rover-right: machine.frame: pivoting (default) -> rigid"),
        "{out}"
    );
    assert_eq!(
        lines.last(),
        Some(&"machine digests: nothing heard from rover-left, rover-right")
    );
    let (code, out, err) = run(&["machine", "diff"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out, "the boards' machine settings agree\n");

    // A replaced controller: the file's right-hand entry names a uid not on the machine and the
    // right board has no entry. Without a terminal it is refused, naming the --map that answers;
    // with that --map the entry goes to the board.
    let gone = "8adc39323335381255343333";
    let swapped = scratch("swapped.json");
    std::fs::write(
        &swapped,
        serde_json::json!({
            "boards": {
                left: { "device": { "name": "rover-left" } },
                gone: { "device": { "name": "rover-new" } },
            },
        })
        .to_string(),
    )
    .unwrap();
    let (code, out, err) = run(&["machine", "apply", &swapped]);
    assert_eq!(code, Some(1));
    assert!(out.is_empty(), "nothing written: {out}");
    assert!(
        err.contains(&format!(
            "8adc... is not on this machine; write its entry to 0x02 (rover-right, 0202...): give \
             --map {gone}=0x02 to write it without a terminal"
        )),
        "{err}"
    );
    let map = format!("{gone}=rover-right");
    let (code, out, err) = run(&["machine", "apply", &swapped, "--map", &map]);
    assert_eq!(code, Some(0), "{err}");
    assert!(
        out.contains("rover-right: device.name: \"rover-right\" -> \"rover-new\""),
        "{out}"
    );

    // An entry no board reports and no board to offer it to: skipped and listed, the rest
    // written, exit 1.
    let extra = scratch("extra.json");
    std::fs::write(
        &extra,
        serde_json::json!({
            "machine": { "turn": [12000, 600] },
            "boards": {
                left: {}, right: {}, gone: { "device": { "name": "elsewhere" } },
            },
        })
        .to_string(),
    )
    .unwrap();
    let (code, out, err) = run(&["machine", "apply", &extra]);
    assert_eq!(code, Some(1), "{err}");
    assert!(
        out.contains(&format!(
            "{gone}: no board of this machine reports this uid; skipped"
        )),
        "{out}"
    );
    assert!(
        out.contains("rover-new: machine.turn[0]: 0 (default) -> 12000"),
        "{out}"
    );

    // A board the file has no entry for, and no machine section to give it: left alone.
    let one = scratch("one.json");
    std::fs::write(
        &one,
        serde_json::json!({ "boards": { left: { "device": { "name": "rover-left" } } } })
            .to_string(),
    )
    .unwrap();
    let (code, out, err) = run(&["machine", "apply", &one]);
    assert_eq!(code, Some(0), "{err}");
    assert!(
        out.contains(&format!("rover-new: no entry in {one}; left alone")),
        "{out}"
    );
    assert!(!out.contains("rover-new: 0 changed"), "{out}");
    server.join().unwrap();
}
