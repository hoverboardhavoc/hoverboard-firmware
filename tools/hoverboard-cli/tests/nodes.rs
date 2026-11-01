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
    use hoverboard_bridge_core::flash::{FLASH_BASE, STORE_START};
    ((STORE_START - FLASH_BASE) / 2) as usize
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
    use hoverboard_cli::wire::Wire;
    let mut m = machine();
    setup(&mut m);
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
fn nodes_name_config_and_esp_commands_by_node() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp = format!("--tcp={}", listener.local_addr().unwrap());
    let server = std::thread::spawn(move || serve(listener, 10));

    // The walk: the fresh bridge ends at 0x03 and the boards keep theirs.
    let (ok, out, err) = cli(&[&tcp, "nodes"]);
    assert!(ok, "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert_eq!(rows.len(), 4, "{out}");
    assert_eq!(
        rows[0],
        "address  class             uid                       name"
    );
    // Each node's `IDENTITY`: a board's factory unique ID, the bridge's MAC and six zero bytes.
    assert_eq!(
        rows[1],
        "0x01     motor controller  010101010101010101010101  Hoverboard"
    );
    assert_eq!(
        rows[2],
        "0x02     motor controller  020202020202020202020202  Hoverboard"
    );
    assert_eq!(
        rows[3],
        "0x03     bridge            00000000ab12000000000000  hb-bridge-ab12"
    );

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
    let (ok, out, err) = cli(&[&tcp, "config", "rover-esp", "0x02"]);
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
        "config",
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
    let server = std::thread::spawn(move || serve(listener, 15));
    let run = |args: &[&str]| {
        let mut a = vec![tcp.as_str()];
        a.extend_from_slice(args);
        cli_code(&a)
    };

    // One field at a time, by name or by number.
    let (code, out, err) = run(&["config", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        // The line starts with the name the board had when the command found it.
        "Hoverboard: device.name: \"Hoverboard\" (default) -> \"rover-left\"\n\
         1 takes effect at the next boot\n"
    );
    let (code, out, err) = run(&["config", "rover-left", "machine.mode", "balance"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: machine.mode: throttle (default) -> balance\n\
         1 takes effect at the next arm\n"
    );
    let (code, out, err) = run(&["config", "0x01", "0x40.1", "PB5"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: motors[1].hall.a: none (default) -> PB5\n1 takes effect at the next boot\n"
    );
    let (code, _, err) = run(&["config", "rover-lft", "machine.mode", "balance"]);
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
            "machine": { "mode": "balance" },
            "motors": [{}, { "hall": { "a": "PB5" } }],
        })
    );
    let (code, out, err) = run(&["diff", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: 0 changed, 0 pinned, 3 unchanged");

    // A stray key: kept by a merge, un-stored by a replace.
    let (code, _, err) = run(&["config", "rover-left", "machine.frame", "rigid"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, out, err) = run(&["diff", "--replace", "rover-left", &file]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "rover-left: 0 changed, 0 pinned, 3 unchanged, 1 un-stored\n\
         rover-left: machine.frame: rigid -> pivoting (default)\n\
         1 takes effect at the next arm\n"
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
         rover-left: machine.frame: rigid -> pivoting (default)\n\
         1 takes effect at the next arm\n"
    );
    let (code, out, err) = run(&["compact", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: compact: ok");
    let (code, out, err) = run(&["diff", "--replace", "rover-left", &file]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: 0 changed, 0 pinned, 3 unchanged");
    let (code, out, err) = run(&["config", "rover-left", "machine.frame"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-left: machine.frame = pivoting (default)");

    // apply merges: a file naming one changed key writes it and keeps the rest.
    let machine = scratch("machine.json");
    std::fs::write(&machine, r#"{ "machine": { "mode": "throttle" } }"#).unwrap();
    let (code, out, err) = run(&["apply", "rover-left", &machine]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(
        out,
        "rover-left: 1 changed, 0 pinned, 0 unchanged, 2 kept\n\
         rover-left: machine.mode: balance -> throttle\n\
         rover-left: kept: device.name = \"rover-left\" (stored, not in the file)\n\
         rover-left: kept: motors[1].hall.a = PB5 (stored, not in the file)\n\
         1 takes effect at the next arm\n"
    );
    // A pin ends with where to see what the board made of it.
    let links = scratch("links.json");
    std::fs::write(&links, r#"{ "links": [{ "tx": "PA2", "rx": "PA3" }] }"#).unwrap();
    let (code, out, err) = run(&["apply", "rover-left", &links]);
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
         check the boot report with \"status\" after the next boot\n"
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
         Hoverboard: wifi.pass: (default) -> (set)\n\
         2 take effect at the next boot\n"
    );
    assert!(!out.contains(PASS) && !err.contains(PASS), "{out}{err}");

    let (code, out, err) = run(&["diff", "0x01", &creds]);
    assert_eq!(code, Some(1), "{err}");
    assert_eq!(
        out,
        "Hoverboard: 1 changed, 0 pinned, 1 unchanged\n\
         Hoverboard: wifi.pass: (set) -> (set)\n\
         1 takes effect at the next boot\n"
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
    let (code, out, err) = run(&["config", "0x02", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "Hoverboard: machine.mode = throttle (default)");
    let (code, _, err) = run(&["config", "0x02", "device.name", "rover-right"]);
    assert_eq!(code, Some(0), "{err}");
    // The prefix is the name the read returns.
    let (code, out, err) = run(&["config", "0x02", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: machine.mode = throttle (default)");
    let (code, out, err) = run(&["config", "rover-right", "machine.mode"]);
    assert_eq!(code, Some(0), "{err}");
    assert_eq!(out.trim(), "rover-right: machine.mode = throttle (default)");
    // An address nothing answers: an error naming it, and no walk to fall back on.
    let (code, _, err) = run(&["config", "0x05", "machine.mode"]);
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

    let (ok, out, err) = cli(&[&tcp, "status"]);
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

    let (ok, out, err) = cli(&[&tcp, "status", "0x02"]);
    assert!(ok, "{err}");
    assert_eq!(out, "Hoverboard: ok\n");

    let (ok, out, err) = cli(&[&tcp, "nodes"]);
    assert!(ok, "{err}");
    let rows: Vec<&str> = out.lines().collect();
    assert!(
        rows[1].starts_with("0x01") && rows[1].ends_with("! 7 status entries, see \"status\""),
        "{out}"
    );
    assert!(!rows[2].contains('!'), "{out}");
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

    let (code, _, err) = run(&["config", "0x01", "device.name", "rover-left"]);
    assert_eq!(code, Some(0), "{err}");
    let (code, _, err) = run(&["config", "0x01", "motors.0.odometer", "123456", "--force"]);
    assert_eq!(code, Some(0), "{err}");

    // Usage: nothing but --yes, and no link opened for it.
    let (code, _, err) = cli_code(&["--tcp=127.0.0.1:9", "erase-store", "now"]);
    assert_eq!(
        (code, err.trim()),
        (Some(1), "error: erase-store takes [--yes]")
    );

    // Without a terminal (the test's stdin is not one) and without --yes: the lines, then the
    // refusal, and no STORE_ERASE sent. The board is the one behind the bridge on the link.
    let restore = "  write it back with: hoverboard-cli config rover-left motors.0.odometer \
                   123456 --force";
    let lines = |out: &str| out.lines().map(str::to_string).collect::<Vec<_>>();
    let (code, out, err) = run(&["erase-store"]);
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
        err.contains("rover-left: erase-store erases every stored setting; give --yes"),
        "{err}"
    );

    // By the bridge's name too; with --yes it erases, waits for the mailbox, and gives the board
    // its old address back through the bridge's mailbox port.
    let (code, out, err) = run(&["--node", "hb-bridge-ab12", "erase-store", "--yes"]);
    assert_eq!(code, Some(0), "{out}{err}");
    assert_eq!(lines(&out)[2], restore, "{out}");
    assert_eq!(
        lines(&out)[4..],
        ["erased", "node.address 1 restored"],
        "{out}"
    );

    // A motor controller is not a bridge.
    let (code, _, err) = run(&["--node", "0x02", "erase-store", "--yes"]);
    assert_eq!(code, Some(1));
    assert!(err.contains("motor controller, not a bridge"), "{err}");

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
