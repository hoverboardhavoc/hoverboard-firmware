//! `hoverboard-cli`: talk to a hoverboard bridge over BLE, its USB console, or that console through
//! a TCP relay. BLE is the `ble` cargo feature, on by default; a build without it (the bench Pi's)
//! refuses `--device` and `scan`.
//!
//! ```text
//! hoverboard-cli scan [--secs N]
//! hoverboard-cli [LINK] nodes
//! hoverboard-cli [LINK] name <node> <text>
//! hoverboard-cli [LINK] config <node> <path> [<value>] [--force]
//! hoverboard-cli [LINK] dump <node> [<file>]
//! hoverboard-cli [LINK] diff [--replace] <node> <file>... [--force]
//! hoverboard-cli [LINK] apply <node> <file>... [--force]
//! hoverboard-cli [LINK] replace <node> <file>... [--yes] [--force]
//! hoverboard-cli [LINK] compact <node>
//! hoverboard-cli fields --json
//! hoverboard-cli [LINK] [--node NODE] info | id | clear-protection
//! hoverboard-cli [LINK] [--node NODE] disarm | halt | resume | reset | reset-halt
//! hoverboard-cli [LINK] [--node NODE] read <addr> <len> | write <addr> <hex>
//! hoverboard-cli [LINK] [--node NODE] flash <bundle|elf> [--image NAME] [--yes]
//! hoverboard-cli firmware info <bundle>
//! hoverboard-cli firmware pack <out.hbfw> <crate-dir>:<image-name>=<elf>... [--dirty]
//! hoverboard-cli [LINK] [--node NODE] erase-store [--yes]
//! hoverboard-cli [LINK] [--node NODE] update-esp <bin> [--no-confirm]
//! hoverboard-cli [--serial PORT] install-esp <image.bin> [<file>...] [--yes]
//! hoverboard-cli [LINK] watch <secs> [<node>...]
//! hoverboard-cli [LINK] drive <value> <secs> [<node>...]
//!
//! LINK: --device NAME|ADDRESS   BLE (the default; with no --device, the first advertiser whose
//!                               name starts with `hb-bridge-`)
//!       --serial DEVICE         the ESP's USB console (/dev/ttyACM0, /dev/cu.usbmodem*)
//!       --tcp HOST:PORT         the bridge's Wi-Fi console (<name>.local:5333), or the USB
//!                               console through a serial-to-TCP relay
//! ```
//!
//! A node is an address (`0x03`) or a `device.name`. An address is reached directly: the tool
//! attaches and reads the node's `device.name`, with no walk. A name needs the walk, which reads
//! every node's name and resolves it. `nodes` walks the machine with `net`'s controller and lists
//! every node's address, class and name; `name` writes the node's `device.name` and prints the
//! status by name.
//!
//! `config`, `dump`, `diff`, `apply`, `replace` and `compact` treat a board's store as a file
//! (`specs/cli-config.md`): a field goes by its name in the store's name table (`motors.1.hall.a`,
//! `control.mode`) or by its raw `<id>.<index>`, and a value takes the file's forms (`PC13`,
//! `balance`, `null` to un-store). Every line about a board starts with its `device.name`. `diff`
//! exits 0 with no changes, 1 with changes, 2 on an error. `fields --json` prints the name table.
//! A secret (`wifi.pass`) is never printed: it shows as `(set)`, `dump` leaves it out and says so,
//! and a file that gives one always writes it (`specs/bridge-onboarding.md` section 2.2).
//!
//! An ESP command goes to the bridge the link reaches (`0x00`, the one peer) unless `--node` names
//! another, which is reached through the boards by its address. Over a board's own BLE module the
//! node on the link is not a bridge, and the command is refused.
//!
//! `scan` is BLE only. `watch` and `drive` attach as an L3 guest with `NODE_HELLO` and send from
//! the granted address. With no nodes given they walk first, and take the motor controllers the
//! walk found: `watch` counts only those nodes' `CYCLIC_STATE`, `drive` drives them and counts
//! every source.
//!
//! `flash` takes a firmware bundle or an ELF, told apart by the file's first bytes
//! (`specs/firmware-bundle.md` section 4). From a bundle it checks each ELF's SHA-256, reads
//! `TARGET_ID`, and takes the image whose target equals the chip's family and flash size; with
//! several, `--image` names one of them, and it cannot name one that does not match. A match the
//! bridge would refuse (a part that is not 64 KiB) is refused before anything is streamed. It runs
//! that image's guards from the manifest, prints the choice and asks, unless `--yes`; without a
//! terminal it refuses unless `--yes` is given. An ELF given directly gets the `wfi` scan and the
//! layout check only: it does not say which chip it is for or which guards apply.
//!
//! `firmware info` prints a bundle's manifest and reads no board. `firmware pack` builds one: each
//! entry's role and targets from its crate's `[package.metadata.hoverboard-image.<name>]`, its
//! guards from `tools/image-guard-syms.sh`, the commit from `git rev-parse HEAD`; a tree with
//! changes to tracked files is refused unless `--dirty`, which suffixes the commit `-dirty`.
//!
//! `update-esp` sends a bridge image (`espflash save-image` output) into the bridge's other app slot,
//! waits for the bridge to reset into it, reconnects over the same link, and confirms it; with
//! `--no-confirm` it stops after the reconnect, and the bridge's next reset returns to the previous
//! slot. It needs no `ESP_INFO` it can decode: one from a bridge built for another layout is
//! skipped with a note.
//!
//! `erase-store` erases the store of the board behind the bridge (`STORE_ERASE`,
//! `specs/hoverboard-bridge.md`, "Erasing the store"). It walks to find that board (the bridge's one
//! walked neighbour), prints what a `dump` leaves out and the erase loses (`node.address`, each
//! motor's `odometer`, with the `config` line that writes a stored odometer back), asks, and prints
//! `erased` once the bridge reports success and its mailbox is up again. It then gives the board
//! its old `node.address` back with a directed `ASSIGN` out the bridge's mailbox port, as the walk
//! assigns a frontier node, and prints `node.address <n> restored`; a board whose address was not
//! read or was unset is left unassigned, and an `ASSIGN` with no `ASSIGN_ACK` fails the command
//! with the `config` line that restores the address by hand. Without a terminal it refuses unless
//! `--yes` is given.
//!
//! `install-esp` makes a SuperMini a bridge over its own USB port (`specs/bridge-onboarding.md`
//! section 3): it asks before erasing a bridge that answers there, writes the merged 4 MB install
//! image through the `espflash` library, waits for the first boot, applies the files given (a new
//! bridge's `device.name`, `wifi.ssid` and `wifi.pass`), and with an SSID among them resets the
//! bridge and waits for it to join.

use std::collections::BTreeMap;
use std::io::{BufRead, IsTerminal, Write as _};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use espsvc::{Ctrl, FAMILY_F1X0};
use net::pdu::NO_ADDRESS;

use hoverboard_cli::bundle::{self, Bundle, Kind, TreeEntry};
use hoverboard_cli::client::{Client, ClientError, Progress, StageTimes};
use hoverboard_cli::config::{self, Board};
use hoverboard_cli::elf;
use hoverboard_cli::erase;
use hoverboard_cli::guest::{self, Drive, Tally, STOP_REPEATS};
use hoverboard_cli::install::{self, Answered, Given, Guard};
use hoverboard_cli::names;
use hoverboard_cli::nodes::{self, class_name, Found, CLASS_MOTOR};
use hoverboard_cli::transport::{Serial, Tcp};
use hoverboard_cli::wire::{ByteLink, Framed, Wire};

#[cfg(feature = "ble")]
mod ble;

/// What a build without the `ble` cargo feature (the bench Pi's) says for a BLE link or `scan`
/// (`specs/bridge-onboarding.md` section 4.2).
#[cfg(not(feature = "ble"))]
const NO_BLE: &str = "this hoverboard-cli is built without the ble feature: no BLE link \
                      (--device, or no link given) and no scan; reach the bridge with --serial \
                      or --tcp";

/// The BLE link to the advertiser `device` names, or the first `hb-bridge-` one.
#[cfg(feature = "ble")]
fn ble_link(device: Option<&str>) -> Result<Box<dyn ByteLink>, String> {
    Ok(Box::new(ble::connect(device)?))
}

/// The BLE link: refused, naming the feature.
#[cfg(not(feature = "ble"))]
fn ble_link(_device: Option<&str>) -> Result<Box<dyn ByteLink>, String> {
    Err(NO_BLE.into())
}

/// `scan`'s advertisers, as (name, address).
#[cfg(feature = "ble")]
fn scan_names(secs: u64) -> Result<Vec<(String, String)>, String> {
    ble::scan_names(secs)
}

/// `scan`: refused, naming the feature.
#[cfg(not(feature = "ble"))]
fn scan_names(_secs: u64) -> Result<Vec<(String, String)>, String> {
    Err(NO_BLE.into())
}

fn parse_u32(s: &str) -> Result<u32, String> {
    let r = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u32::from_str_radix(h, 16),
        None => s.parse(),
    };
    r.map_err(|_| format!("not a number: {s}"))
}

fn parse_secs(s: Option<&&str>, need: &str) -> Result<u64, String> {
    let s = s.ok_or(need)?;
    s.parse()
        .map_err(|_| format!("not a number of seconds: {s}"))
}

fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let s = s.trim_start_matches("0x");
    if !s.len().is_multiple_of(2) {
        return Err("hex needs an even number of digits".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|_| format!("not hex: {s}")))
        .collect()
}

fn hexdump(addr: u32, data: &[u8]) {
    for (i, row) in data.chunks(16).enumerate() {
        let hex: Vec<String> = row.iter().map(|b| format!("{b:02x}")).collect();
        println!("{:08x}: {}", addr + 16 * i as u32, hex.join(" "));
    }
}

fn print_id(id: &espsvc::TargetId) {
    let family = if id.family == FAMILY_F1X0 {
        "GD32F1x0"
    } else {
        "GD32F10x"
    };
    let uid: Vec<String> = id.uid.iter().map(|b| format!("{b:02x}")).collect();
    println!("dpidr       {:#010x}", id.dpidr);
    println!("mcu id      {:#010x}", id.mcu_id);
    println!("cpuid       {:#010x}", id.cpuid);
    println!("family      {family}");
    println!("uid         {}", uid.join(""));
    println!(
        "protection  {:#04x} ({})",
        id.rdp,
        if id.rdp == 0xA5 { "off" } else { "ON" }
    );
    println!("user        {:#04x}", id.user);
    println!(
        "clear check {}",
        match id.clear_check {
            espsvc::CLEAR_PASS => "pass",
            espsvc::CLEAR_FAIL => "FAIL",
            _ => "none since the bridge booted",
        }
    );
    println!("flash       {} KiB", id.flash_kib);
    println!("armed       {}", if id.armed { "YES" } else { "no" });
}

fn print_info(i: &espsvc::EspInfo) {
    let state = espsvc::MailboxState::from_u8(i.mailbox)
        .map_or_else(|| format!("{}", i.mailbox), |s| format!("{s:?}"));
    println!(
        "version     {}.{}.{}",
        i.version[0], i.version[1], i.version[2]
    );
    println!("commit      {:08x}", i.commit);
    println!(
        "chip        {}",
        if i.chip == espsvc::CHIP_ESP32C3 {
            "ESP32-C3"
        } else {
            "?"
        }
    );
    println!("swd clock   {} kHz", i.swd_khz);
    println!(
        "counters    wait {} fault {} parity {} reconnect {}",
        i.waits, i.faults, i.parity_errors, i.reconnects
    );
    let bad_read = i.bad_read.map_or_else(
        || "none".to_string(),
        |b| format!("ack {:#05b} data {:#010x}", b.ack, b.data),
    );
    let no_ack = i
        .no_ack
        .map_or_else(|| "none".to_string(), |a| format!("{a:#05b}"));
    println!("last bad read: {bad_read}; no ack: {no_ack}");
    println!("mailbox     {state}");
    println!("wifi        {}", install::wifi_text(i));
    println!("slot        {}", slot_name(i));
    match i.node {
        NO_ADDRESS => println!("node        unassigned"),
        a => println!("node        {a:#04x}"),
    }
    println!(
        "store       {}",
        if i.store_mounted {
            "mounted"
        } else {
            "NOT mounted: unassigned, default name, nothing kept across a reset"
        }
    );
    println!(
        "gdb         {}",
        if i.gdb { "connected" } else { "not connected" }
    );
    match i.moe_clear_us {
        espsvc::MOE_CLEAR_NONE => println!("moe clear   none yet"),
        us => println!("moe clear   {us} us from the halt seen to MOE cleared"),
    }
}

use install::slot_name;

/// How long the bridge may take to reset into an update and answer again (Wi-Fi rejoin and DHCP
/// included).
const UPDATE_REBOOT: Duration = Duration::from_secs(120);
/// The pause before each reconnect attempt.
const RECONNECT_GAP: Duration = Duration::from_secs(2);

/// A client over whichever link was opened.
type EspClient = Client<Framed<Box<dyn ByteLink>>>;

/// Open `via` again until the bridge answers the attach, within [`UPDATE_REBOOT`].
fn reconnect(via: &Via, node: Option<&str>) -> Result<EspClient, String> {
    let t0 = Instant::now();
    let mut why = String::from("no answer");
    while t0.elapsed() < UPDATE_REBOOT {
        std::thread::sleep(RECONNECT_GAP);
        match esp_client(via, node) {
            Ok(c) => return Ok(c),
            Err(e) => why = e,
        }
    }
    Err(format!(
        "the bridge did not answer within {} s of the update ({why})",
        UPDATE_REBOOT.as_secs()
    ))
}

/// The note `update-esp` prints for an `ESP_INFO` it cannot decode.
const OTHER_LAYOUT: &str = "bridge's info has another layout; continuing";

/// `ESP_INFO` for `update-esp`: `None`, with [`OTHER_LAYOUT`] printed, when it does not decode.
fn info_or_note(c: &mut EspClient) -> Result<Option<espsvc::EspInfo>, String> {
    let i = c.info_if_decodes().map_err(|e| e.to_string())?;
    if i.is_none() {
        eprintln!("{OTHER_LAYOUT}");
    }
    Ok(i)
}

/// `update-esp`: send the image, reconnect after the reset, and confirm it unless `no_confirm`.
/// It depends on no reply whose layout can change: the attach, `ESP_UPDATE_BEGIN`'s answer and
/// `ESP_UPDATE_CONFIRM`'s `RESULT`; `ESP_INFO` is printed, and the new slot checked against it,
/// only where it decodes (`specs/hoverboard-bridge.md`, "Installing and updating the bridge").
fn update_esp(via: &Via, node: Option<&str>, path: &str, no_confirm: bool) -> Result<(), String> {
    let image = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let version = espsvc::app_version(&image).ok_or_else(|| {
        format!("{path}: not an ESP32-C3 app image (make one with espflash save-image)")
    })?;
    let e = |e: ClientError| e.to_string();
    let mut c = esp_client(via, node)?;
    let before = info_or_note(&mut c)?;
    let running = before
        .as_ref()
        .map_or_else(String::new, |b| format!("running {}; ", slot_name(b)));
    eprintln!(
        "{running}sending {} B, version {}.{}.{}",
        image.len(),
        version[0],
        version[1],
        version[2]
    );
    let t0 = Instant::now();
    let total = image.len();
    c.update_esp(&image, version, |done| {
        eprint!("\r{done}/{total} ({:.1} s)   ", t0.elapsed().as_secs_f32())
    })
    .map_err(e)?;
    eprintln!();
    eprintln!(
        "written and checked in {:.1} s; the bridge is resetting into it",
        t0.elapsed().as_secs_f32()
    );
    drop(c);

    let mut c = reconnect(via, node)?;
    if no_confirm {
        let after = info_or_note(&mut c)?;
        if let (Some(b), Some(a)) = (&before, &after) {
            if a.slot == b.slot || !a.pending {
                return Err(format!(
                    "the bridge did not boot the new image: it runs {}",
                    slot_name(a)
                ));
            }
        }
        if let Some(a) = &after {
            print_info(a);
        }
        println!(
            "not confirmed: the bridge's next reset returns to {}",
            before
                .as_ref()
                .map_or_else(|| "the previous slot".to_string(), slot_name)
        );
        return Ok(());
    }
    c.confirm().map_err(e)?;
    let after = info_or_note(&mut c)?;
    if let (Some(b), Some(a)) = (&before, &after) {
        if a.slot == b.slot {
            return Err(format!(
                "the bridge did not boot the new image: it runs {}",
                slot_name(a)
            ));
        }
    }
    eprintln!("confirmed");
    if let Some(a) = &after {
        print_info(a);
    }
    Ok(())
}

/// A bridge that answered on a port: the client to it, the guest address it granted, its
/// `device.name`, and its `ESP_INFO` when that decodes.
type OnPort = (EspClient, u8, String, Option<espsvc::EspInfo>);

/// A bridge on `port` that answers an attach within `wait`; `None` when nothing answers (a blank
/// chip, other firmware, or a port that is not there yet).
fn bridge_on(port: &str, wait: Duration) -> Option<OnPort> {
    let link: Box<dyn ByteLink> = Box::new(Serial::open(port).ok()?);
    let mut wire = Framed::new(link);
    let src = guest::attach(&mut wire, 1, wait)?;
    let name = match nodes::read(&mut wire, src, NO_ADDRESS, store::DEVICE_NAME.key())
        .ok()?
        .value()
    {
        Some(store::Value::Str(s)) => String::from_utf8_lossy(s).into_owned(),
        _ => "the bridge".to_string(),
    };
    let mut c = Client::addressed(wire, src, NO_ADDRESS);
    let info = c.info_if_decodes().ok().flatten();
    Some((c, src, name, info))
}

/// Wait up to `within` for the bridge on `port` to answer with an `ESP_INFO` that `done` accepts,
/// reopening the port while the chip resets: the answer, accepted or not when time ran out while
/// connected; `None` when nothing answered.
fn wait_for_bridge(
    port: &str,
    within: Duration,
    done: impl Fn(&espsvc::EspInfo) -> bool,
) -> Option<(EspClient, u8, String, espsvc::EspInfo)> {
    let t0 = Instant::now();
    while t0.elapsed() < within {
        let Some((mut c, src, name, Some(mut info))) = bridge_on(port, HELLO_WAIT) else {
            std::thread::sleep(Duration::from_millis(500));
            continue;
        };
        loop {
            if done(&info) || t0.elapsed() >= within {
                return Some((c, src, name, info));
            }
            std::thread::sleep(RECONNECT_GAP);
            match c.info() {
                Ok(i) => info = i,
                Err(_) => break,
            }
        }
    }
    None
}

/// `install-esp` (`specs/bridge-onboarding.md` section 3.2): check the image and the files, choose
/// the port, ask before erasing a bridge that answers there, write the image, wait for the first
/// boot, apply the files, and when they set an SSID, reset the bridge and wait for it to join.
fn install_esp(given: Given, image_path: &str, files: &[&str], yes: bool) -> Result<(), String> {
    let (port, found) = install::choose_port(given, &install::host_ports())?;
    let image = std::fs::read(image_path).map_err(|e| format!("{image_path}: {e}"))?;
    install::check_image(&image).map_err(|e| format!("{image_path}: {e}"))?;
    let files = if files.is_empty() {
        None
    } else {
        Some(load_files(files, false)?)
    };
    if found {
        eprintln!("using {port}, the one ESP32-C3 USB port");
    }

    // 1. The guard.
    let answered = bridge_on(&port, HELLO_WAIT).map(|(_, _, name, info)| Answered {
        name,
        addr: info.map(|i| i.node),
        commit: info.map(|i| i.commit),
    });
    if let Some(a) = &answered {
        println!("bridge on {port}: {}", a.describe());
    }
    match install::guard(answered.as_ref(), yes, std::io::stdin().is_terminal()) {
        Guard::Proceed => {}
        Guard::Refuse(why) => return Err(why),
        Guard::Ask(q) => {
            if !confirm(&q) {
                return Err("nothing installed".into());
            }
        }
    }

    // 2. The install, ending in a hard reset into the new image.
    let t0 = Instant::now();
    install::write_image(&port, &image)?;
    eprintln!(
        "written and verified in {:.1} s; the chip is resetting into it",
        t0.elapsed().as_secs_f32()
    );

    // 3. The first boot.
    let (mut c, src, name, info) = wait_for_bridge(&port, install::BOOT_WAIT, |_| true)
        .ok_or_else(|| {
            format!(
                "the bridge did not answer on {port} within {} s of the install",
                install::BOOT_WAIT.as_secs()
            )
        })?;
    println!(
        "booted: {name}, slot {}, wifi {}",
        slot_name(&info),
        install::wifi_text(&info)
    );

    // 4. The files.
    let Some(files) = files else {
        return Ok(());
    };
    let mut b = Board {
        wire: c.wire(),
        src,
        dst: NO_ADDRESS,
        name,
    };
    let (reads, unknown) = config::read_board(&mut b, &config::plan_keys(&files))?;
    let plan = config::plan(&files, &reads, &unknown, false).map_err(|p| {
        let msgs: Vec<String> = p.into_iter().map(|p| p.msg).collect();
        msgs.join("\nerror: ")
    })?;
    let after = config::execute(&mut b, &plan).map_err(|r| {
        for l in config::written_lines(&b.name, &plan, &reads, &r) {
            println!("{l}");
        }
        r.msg
    })?;
    for l in config::report(&b.name, &plan, &reads, &after) {
        println!("{l}");
    }
    drop(c);
    let ssid = files.entries.iter().find_map(|e| {
        (e.key == store::WIFI_SSID.key())
            .then(|| e.value.as_str())
            .flatten()
            .filter(|s| !s.is_empty())
    });
    let Some(ssid) = ssid else {
        return Ok(());
    };
    install::reset(&port)?;
    eprintln!("reset; waiting for it to join {ssid}");
    let up = |i: &espsvc::EspInfo| i.wifi == espsvc::WifiState::Up as u8;
    match wait_for_bridge(&port, install::JOIN_WAIT, up) {
        Some((_, _, name, info)) if up(&info) => {
            let [a, b, c, d] = info.ip;
            println!("joined: {name}, {ssid}, {a}.{b}.{c}.{d}");
            Ok(())
        }
        _ => Err(format!("did not join {ssid}")),
    }
}

fn confirm(prompt: &str) -> bool {
    print!("{prompt} [y/N] ");
    let _ = std::io::stdout().flush();
    let mut line = String::new();
    let _ = std::io::stdin().lock().read_line(&mut line);
    line.trim().eq_ignore_ascii_case("y")
}

/// How many `NODE_HELLO`s an attach sends, and how long it waits for each answer.
const HELLO_TRIES: u32 = 3;
const HELLO_WAIT: Duration = Duration::from_secs(1);
/// The gap between rounds of `drive`'s stop frames.
const STOP_GAP: Duration = Duration::from_millis(50);
/// The longest one wait for a packet, so a Ctrl-C is seen within it.
const POLL: Duration = Duration::from_millis(50);
/// The interval between `watch` lines.
const LINE_EVERY: Duration = Duration::from_secs(1);

/// Attach to the node on the link as a guest and return the granted address.
fn attach(wire: &mut impl Wire) -> Result<u8, String> {
    let a = guest::attach(wire, HELLO_TRIES, HELLO_WAIT)
        .ok_or("no NODE_HELLO answer from the node on this link")?;
    eprintln!("attached as guest {a:#04x}");
    Ok(a)
}

/// Walk the machine, then read every node's class and name; the guest address and what was found.
fn walk(wire: &mut impl Wire) -> Result<(u8, Vec<Found>), String> {
    let c = nodes::walk(wire)?;
    let found = nodes::survey(wire, &c);
    eprintln!(
        "walked {} nodes as guest {:#04x}",
        found.len(),
        c.guest_addr()
    );
    Ok((c.guest_addr(), found))
}

/// A node for messages: its address and, when it has one, its name.
fn label(f: &Found) -> String {
    match &f.name {
        Some(n) => format!("{:#04x} ({n})", f.addr),
        None => format!("{:#04x}", f.addr),
    }
}

/// Reach the node `spec` names (`specs/cli-config.md` section 4.1): an address by attaching and
/// reading its `device.name`, with no walk (a frame to an address the forwarders have not learned
/// is flooded until the reply teaches the route); a name by walking and resolving it. The node's
/// class is read only for a name (the walk reads it); [`class_of`] reads it for an address.
fn reach(wire: &mut impl Wire, spec: &str) -> Result<(u8, Found), String> {
    if nodes::parse_int(spec).is_none() {
        return walk_to(wire, spec);
    }
    let addr = nodes::resolve(spec, &[])?;
    let src = attach(wire)?;
    let r = nodes::read(wire, src, addr, store::DEVICE_NAME.key()).map_err(|_| {
        format!(
            "no answer from {addr:#04x}; `hoverboard-cli nodes` lists the nodes this link reaches"
        )
    })?;
    let name = match r.value() {
        Some(store::Value::Str(s)) => Some(String::from_utf8_lossy(s).into_owned()),
        _ => None,
    };
    Ok((
        src,
        Found {
            addr,
            class: None,
            name,
        },
    ))
}

/// `f`'s `node.class`, read from the node when reaching it did not.
fn class_of(wire: &mut impl Wire, src: u8, f: &Found) -> Option<u8> {
    f.class.or_else(|| {
        match nodes::read(wire, src, f.addr, store::NODE_CLASS.key())
            .ok()?
            .value()
        {
            Some(store::Value::U8(c)) => Some(c),
            _ => None,
        }
    })
}

/// Walk and resolve `spec` to a node the walk found.
fn walk_to(wire: &mut impl Wire, spec: &str) -> Result<(u8, Found), String> {
    let (src, found) = walk(wire)?;
    let a = nodes::resolve(spec, &found)?;
    let f = found
        .into_iter()
        .find(|f| f.addr == a)
        .ok_or_else(|| format!("the walk found no node {a:#04x}"))?;
    Ok((src, f))
}

/// The name lines about a node start with: its `device.name`, or its address when it has none.
fn node_name(f: &Found) -> String {
    f.name.clone().unwrap_or_else(|| format!("{:#04x}", f.addr))
}

/// The files of `diff`, `apply` and `replace`, read and checked before any link is opened; the
/// problems `--force` lets through are printed as warnings.
fn load_files(paths: &[&str], force: bool) -> Result<config::Files, String> {
    if paths.is_empty() {
        return Err("give at least one file".into());
    }
    let texts = paths
        .iter()
        .map(|p| {
            std::fs::read_to_string(p)
                .map(|t| (p.to_string(), t))
                .map_err(|e| format!("{p}: {e}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let files = config::read_files(&texts);
    for w in config::warnings(&files.problems, force) {
        eprintln!("{w}");
    }
    config::stop_on(&files.problems, force)?;
    Ok(files)
}

/// `diff`, `apply` and `replace` (`specs/cli-config.md` section 4): check the files, read the board,
/// plan, and unless `dry`, write. Returns whether anything is (or would be) written.
fn file_command(
    via: &Via,
    spec: &str,
    paths: &[&str],
    replace: bool,
    dry: bool,
    flags: &Flags,
) -> Result<bool, String> {
    let files = load_files(paths, flags.force)?;
    let mut wire = Framed::new(via.open()?);
    let (src, f) = reach(&mut wire, spec)?;
    let mut b = Board {
        wire: &mut wire,
        src,
        dst: f.addr,
        name: node_name(&f),
    };
    let (reads, unknown) = config::read_board(&mut b, &config::plan_keys(&files))?;
    let plan = config::plan(&files, &reads, &unknown, replace).map_err(|p| {
        let msgs: Vec<String> = p.into_iter().map(|p| p.msg).collect();
        msgs.join("\nerror: ")
    })?;
    let none = BTreeMap::new();
    if dry {
        for l in config::report(&b.name, &plan, &reads, &none) {
            println!("{l}");
        }
        return Ok(plan.changes());
    }
    let n = plan.unstores();
    if replace && n > 0 && !flags.yes {
        if !std::io::stdin().is_terminal() {
            return Err(format!(
                "{}: replace would un-store {n} keys; give --yes to do it without a terminal",
                b.name
            ));
        }
        for l in config::report(&b.name, &plan, &reads, &none) {
            println!("{l}");
        }
        if !confirm(&format!("Un-store {n} keys on {}?", b.name)) {
            return Err("nothing written".into());
        }
    }
    let after = config::execute(&mut b, &plan).map_err(|r| {
        for l in config::written_lines(&b.name, &plan, &reads, &r) {
            println!("{l}");
        }
        r.msg
    })?;
    for l in config::report(&b.name, &plan, &reads, &after) {
        println!("{l}");
    }
    // A pin field is read only at boot; what the board made of it is its `board.status`.
    if plan
        .steps
        .iter()
        .any(|s| names::row_of(s.key).is_some_and(|r| r.kind == store::Kind::Pin))
    {
        println!("{}", hoverboard_cli::status::PIN_HINT);
    }
    Ok(plan.changes())
}

/// The flags of the configuration commands.
struct Flags {
    /// `--force`: the checks it may let through become warnings.
    force: bool,
    /// `--yes`: `replace` un-stores without asking.
    yes: bool,
}

/// A node's `board.status` value, or `None` when it does not answer one.
fn board_status(wire: &mut impl Wire, src: u8, addr: u8) -> Option<Vec<u8>> {
    let r = nodes::read(wire, src, addr, store::BOARD_STATUS.key()).ok()?;
    if r.status != net::walk::CFG_OK {
        return None;
    }
    hoverboard_cli::status::entries_of(r.value())
}

/// The motor controllers the walk found, for `watch` and `drive` with no nodes given.
fn motor_controllers(wire: &mut impl Wire) -> Result<(u8, Vec<u8>), String> {
    let (src, found) = walk(wire)?;
    let motors: Vec<u8> = found
        .iter()
        .filter(|f| f.class == Some(CLASS_MOTOR))
        .map(|f| f.addr)
        .collect();
    if motors.is_empty() {
        return Err("the walk found no motor controller".into());
    }
    Ok((src, motors))
}

/// The client for an ESP command: to the bridge `node` names, through the boards, or with none given
/// to the node on the link, which has to be a bridge.
fn esp_client(via: &Via, node: Option<&str>) -> Result<EspClient, String> {
    let mut wire = Framed::new(via.open()?);
    let (src, dst) = match node {
        Some(spec) => {
            let (src, mut f) = reach(&mut wire, spec)?;
            f.class = class_of(&mut wire, src, &f);
            if f.class != Some(net::walk::CLASS_BRIDGE) {
                return Err(format!(
                    "{} is a {}, not a bridge",
                    label(&f),
                    class_name(f.class)
                ));
            }
            (src, f.addr)
        }
        None => {
            let src = attach(&mut wire)?;
            let here = nodes::read(&mut wire, src, NO_ADDRESS, store::NODE_CLASS.key())?;
            if here.value() != Some(store::Value::U8(net::walk::CLASS_BRIDGE)) {
                let (_, found) = walk(&mut wire)?;
                let bridges: Vec<String> = found
                    .iter()
                    .filter(|f| f.class == Some(net::walk::CLASS_BRIDGE))
                    .map(label)
                    .collect();
                return Err(if bridges.is_empty() {
                    "no bridge on this machine".to_string()
                } else {
                    format!(
                        "the node on this link is not a bridge; give --node with one of: {}",
                        bridges.join(", ")
                    )
                });
            }
            (src, NO_ADDRESS)
        }
    };
    Ok(Client::addressed(wire, src, dst))
}

/// `erase-store`: walk, find the board behind the bridge (`node` names the bridge, or the one on
/// the link), print what the erase loses that a `dump` leaves out, ask, send `STORE_ERASE`, wait
/// for the board's mailbox, and give the board its old address back out the bridge's mailbox port.
fn erase_store(via: &Via, node: Option<&str>, yes: bool) -> Result<(), String> {
    let mut wire = Framed::new(via.open()?);
    let walked = nodes::walk(&mut wire)?;
    let found = nodes::survey(&mut wire, &walked);
    let src = walked.guest_addr();
    let (bridge, dst) = match node {
        Some(spec) => {
            let a = nodes::resolve(spec, &found)?;
            (a, a)
        }
        None => (
            walked
                .gateway_addr()
                .ok_or("the walk found no node on this link")?,
            NO_ADDRESS,
        ),
    };
    let at = |a: u8| found.iter().find(|f| f.addr == a);
    let b = at(bridge).ok_or_else(|| format!("the walk found no node {bridge:#04x}"))?;
    if b.class != Some(net::walk::CLASS_BRIDGE) {
        return Err(format!(
            "{} is a {}, not a bridge; give --node with a bridge",
            label(b),
            class_name(b.class)
        ));
    }
    let board = walked
        .neighbours(bridge)
        .into_iter()
        .filter_map(at)
        .find(|f| f.class == Some(CLASS_MOTOR))
        .ok_or_else(|| format!("the walk found no board behind {}", label(b)))?;
    let name = node_name(board);
    let mailbox = walked
        .port_of_kind(bridge, net::walk::PORT_SWD)
        .ok_or_else(|| format!("{} reported no mailbox port", label(b)))?;
    let mut on = Board {
        wire: &mut wire,
        src,
        dst: board.addr,
        name: name.clone(),
    };
    let (reads, _) = config::read_board(&mut on, &config::board_owned_keys())?;
    for l in erase::lines(&name, &reads) {
        println!("{l}");
    }
    match erase::guard(&name, yes, std::io::stdin().is_terminal()) {
        Guard::Proceed => {}
        Guard::Refuse(why) => return Err(why),
        Guard::Ask(q) => {
            if !confirm(&q) {
                return Err("nothing erased".into());
            }
        }
    }
    let mut c = Client::addressed(wire, src, dst);
    let e = |e: ClientError| e.to_string();
    c.store_erase().map_err(e)?;
    let state = c.wait_mailbox(erase::MAILBOX_WAIT).map_err(e)?;
    if state != espsvc::MailboxState::Up as u8 {
        let state = espsvc::MailboxState::from_u8(state)
            .map_or_else(|| format!("{state}"), |s| format!("{s:?}"));
        return Err(format!(
            "{name}: the store is erased, but the board's mailbox was not up within {} s ({state})",
            erase::MAILBOX_WAIT.as_secs()
        ));
    }
    println!("erased");
    let line = erase::restore(c.wire(), src, bridge, mailbox, &name, &reads)?;
    println!("{line}");
    Ok(())
}

/// Board addresses from the command line.
fn board_list(args: &[&str]) -> Result<Vec<u8>, String> {
    args.iter()
        .map(|s| {
            parse_u32(s).and_then(|a| match u8::try_from(a) {
                Ok(a) if net::pdu::is_board(a) => Ok(a),
                _ => Err(format!("not a board address (0x01..0x7f): {s}")),
            })
        })
        .collect()
}

/// Listen for `secs`, or until `stop` is set, counting `CYCLIC_STATE` into `tally` and printing
/// its line once a second; send `drive`'s frames as they fall due. Returns the tally and the run's
/// length.
fn session(
    wire: &mut impl Wire,
    mut tally: Tally,
    secs: u64,
    mut drive: Option<&mut Drive>,
    stop: &AtomicBool,
) -> (Tally, Duration) {
    let end = Duration::from_secs(secs);
    let mut next_line = LINE_EVERY;
    let t0 = Instant::now();
    loop {
        let at = t0.elapsed();
        if at >= end || stop.load(Ordering::Relaxed) {
            return (tally, at.min(end));
        }
        let mut next = next_line.min(end);
        if let Some(d) = drive.as_deref_mut() {
            for f in d.due(at) {
                wire.send(&f);
            }
            next = next.min(d.next_due());
        }
        let wait = next.saturating_sub(t0.elapsed()).min(POLL);
        if let Some(pkt) = wire.recv(wait) {
            tally.feed(&pkt, t0.elapsed());
        }
        if t0.elapsed() >= next_line && next_line <= end {
            println!("{}", tally.line(next_line));
            next_line += LINE_EVERY;
        }
    }
}

/// A `CONFIG_RESP` status as the exit outcome: anything but ok fails the command.
fn config_outcome(status: u8) -> Result<(), String> {
    if status == net::walk::CFG_OK {
        Ok(())
    } else {
        Err(format!("refused: {}", nodes::status_name(status)))
    }
}

fn print_summary(tally: &Tally, elapsed: Duration) {
    for l in tally.summary(elapsed) {
        println!("{l}");
    }
}

/// Which link to the bridge.
enum Via<'a> {
    /// BLE, to this advertiser or the first `hb-bridge-` one.
    Ble(Option<&'a str>),
    /// The USB console at this device.
    Serial(&'a str),
    /// The USB console through the relay at this `host:port`.
    Tcp(&'a str),
}

impl Via<'_> {
    fn open(&self) -> Result<Box<dyn ByteLink>, String> {
        Ok(match *self {
            Via::Ble(device) => ble_link(device)?,
            Via::Serial(device) => {
                let s = Serial::open(device)?;
                eprintln!("opened {device}");
                Box::new(s)
            }
            Via::Tcp(addr) => {
                let t = Tcp::connect(addr)?;
                eprintln!("connected to {addr}");
                Box::new(t)
            }
        })
    }
}

/// Every command word; any other is a usage error before a link is opened.
const COMMANDS: &[&str] = &[
    "scan",
    "nodes",
    "status",
    "name",
    "config",
    "dump",
    "diff",
    "apply",
    "replace",
    "compact",
    "fields",
    "info",
    "id",
    "flash",
    "firmware",
    "update-esp",
    "install-esp",
    "clear-protection",
    "erase-store",
    "disarm",
    "halt",
    "resume",
    "reset",
    "reset-halt",
    "read",
    "write",
    "watch",
    "drive",
];

/// Run the command line. `diff` exits 2 on every error, usage errors included, so its 1 means
/// only "there are changes" (`specs/cli-config.md` section 4.2).
fn run(args: &[String]) -> Result<u8, String> {
    let mut cmd = None;
    match dispatch(args, &mut cmd) {
        Err(e) if cmd.as_deref() == Some("diff") => {
            eprintln!("error: {e}");
            Ok(2)
        }
        r => r,
    }
}

/// The command line's work; `cmd` is set to the command as soon as it is known, before any error
/// in the options is returned.
fn dispatch(args: &[String], cmd_out: &mut Option<String>) -> Result<u8, String> {
    let mut via = None;
    // The first error in the options, returned once the command is known.
    let mut early: Option<String> = None;
    let mut node: Option<&str> = None;
    let mut image_flag: Option<&str> = None;
    let mut rest: Vec<&str> = Vec::new();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        // `--opt value` or `--opt=value`.
        let (opt, inline) = match a.split_once('=') {
            Some((o, v)) if a.starts_with("--") => (o, Some(v)),
            _ => (a.as_str(), None),
        };
        let make: fn(&str) -> Via = match opt {
            "--device" => |d| Via::Ble(Some(d)),
            "--serial" => |d| Via::Serial(d),
            "--tcp" => |d| Via::Tcp(d),
            "--secs" | "--no-confirm" | "--force" | "--yes" | "--replace" | "--json"
            | "--dirty" => {
                rest.push(a); // the commands' own options
                continue;
            }
            "--node" | "--image" => {
                let slot = if opt == "--node" {
                    &mut node
                } else {
                    &mut image_flag
                };
                match inline.or_else(|| it.next().map(String::as_str)) {
                    Some(v) if slot.replace(v).is_some() => {
                        early.get_or_insert(format!("give {opt} once"));
                    }
                    Some(_) => {}
                    None => {
                        early.get_or_insert(format!("{opt} needs a value"));
                    }
                }
                continue;
            }
            o if o.starts_with("--") => {
                early.get_or_insert(format!("unknown option {o}"));
                continue;
            }
            _ => {
                rest.push(a);
                continue;
            }
        };
        match inline.or_else(|| it.next().map(String::as_str)) {
            Some(v) if via.replace(make(v)).is_some() => {
                early.get_or_insert("give one of --device, --serial, --tcp".into());
            }
            Some(_) => {}
            None => {
                early.get_or_insert(format!("{opt} needs a value"));
            }
        }
    }
    // The command word is the first argument that is neither a flag nor `--secs`'s value, wherever
    // the flags stand; it moves to the front, so the positional arguments follow it.
    let mut word = None;
    let mut i = 0;
    while i < rest.len() {
        match rest[i] {
            "--secs" => i += 2,
            a if a.starts_with("--") => i += 1,
            _ => {
                word = Some(i);
                break;
            }
        }
    }
    if let Some(i) = word {
        let w = rest.remove(i);
        rest.insert(0, w);
    }
    *cmd_out = word.map(|_| rest[0].to_string());
    if let Some(e) = early {
        return Err(e);
    }
    let given = match &via {
        None => Given::Nothing,
        Some(Via::Serial(p)) => Given::Serial(p),
        Some(Via::Tcp(_)) => Given::Tcp,
        Some(Via::Ble(_)) => Given::Device,
    };
    let via = via.unwrap_or(Via::Ble(None));
    let usage = "usage: hoverboard-cli [--device NAME|ADDR | --serial DEVICE | --tcp HOST:PORT] [--node NODE] scan|nodes|status [<node>...]|name <node> <text>|config <node> <path> [<value>] [--force]|dump <node> [<file>]|diff [--replace] <node> <file>...|apply <node> <file>...|replace <node> <file>... [--yes]|compact <node>|fields --json|info|id|flash <bundle|elf> [--image NAME] [--yes]|firmware info <bundle>|firmware pack <out.hbfw> <crate-dir>:<image-name>=<elf>... [--dirty]|update-esp <bin> [--no-confirm]|install-esp <image.bin> [<file>...] [--yes]|clear-protection|erase-store [--yes]|disarm|halt|resume|reset|reset-halt|read <addr> <len>|write <addr> <hex>|watch <secs> [<node>...]|drive <value> <secs> [<node>...]";
    let cmd = match word {
        Some(_) if COMMANDS.contains(&rest[0]) => rest[0],
        _ => return Err(usage.into()),
    };
    let e = |e: ClientError| e.to_string();

    // The configuration commands' flags, wherever they stand; what remains is positional.
    let flags = Flags {
        force: rest.contains(&"--force"),
        yes: rest.contains(&"--yes"),
    };
    let replace_flag = rest.contains(&"--replace");
    let json_flag = rest.contains(&"--json");
    if [
        "config", "dump", "diff", "apply", "replace", "compact", "fields",
    ]
    .contains(&cmd)
    {
        rest.retain(|a| !["--force", "--yes", "--replace", "--json"].contains(a));
        let allowed = match cmd {
            "config" | "apply" => !flags.yes && !replace_flag && !json_flag,
            "diff" => !flags.yes && !json_flag,
            "replace" => !replace_flag && !json_flag,
            "fields" => json_flag && !flags.force && !flags.yes && !replace_flag,
            _ => !flags.force && !flags.yes && !replace_flag && !json_flag,
        };
        if !allowed {
            return Err(usage.into());
        }
    }

    if cmd == "fields" {
        if rest.len() != 1 {
            return Err("fields takes --json".into());
        }
        let text = serde_json::to_string_pretty(&names::table_json()).map_err(|e| e.to_string())?;
        println!("{text}");
        return Ok(0);
    }

    if cmd == "diff" {
        let [_, spec, ref paths @ ..] = rest[..] else {
            return Err("diff needs [--replace] <node> <file>...".into());
        };
        return file_command(&via, spec, paths, replace_flag, true, &flags).map(u8::from);
    }

    if cmd == "apply" || cmd == "replace" {
        let [_, spec, ref paths @ ..] = rest[..] else {
            return Err(format!("{cmd} needs <node> <file>..."));
        };
        file_command(&via, spec, paths, cmd == "replace", false, &flags)?;
        return Ok(0);
    }

    if cmd == "dump" {
        let (spec, out) = match rest[..] {
            [_, spec] => (spec, None),
            [_, spec, out] => (spec, Some(out)),
            _ => return Err("dump needs <node> [<file>]".into()),
        };
        let mut wire = Framed::new(via.open()?);
        let (src, f) = reach(&mut wire, spec)?;
        let mut b = Board {
            wire: &mut wire,
            src,
            dst: f.addr,
            name: node_name(&f),
        };
        let (reads, unknown) = config::read_board(&mut b, &config::dump_keys())?;
        for k in unknown {
            eprintln!(
                "warning: {}: the board does not know {}",
                b.name,
                names::path_of(k)
            );
        }
        for l in config::left_out(&b.name, &reads) {
            eprintln!("{l}");
        }
        let stored = reads
            .values()
            .filter(|r| r.status == net::walk::CFG_OK)
            .count();
        let text = serde_json::to_string_pretty(&config::to_file(&reads))
            .map_err(|e| e.to_string())?
            + "\n";
        match out {
            Some(p) => {
                std::fs::write(p, text).map_err(|e| format!("{p}: {e}"))?;
                eprintln!("{}: {stored} stored keys written to {p}", b.name);
            }
            None => print!("{text}"),
        }
        return Ok(0);
    }

    if cmd == "compact" {
        let [_, spec] = rest[..] else {
            return Err("compact needs <node>".into());
        };
        let mut wire = Framed::new(via.open()?);
        let (src, f) = reach(&mut wire, spec)?;
        let status = nodes::compact(&mut wire, src, f.addr)?;
        println!("{}: compact: {}", node_name(&f), nodes::status_name(status));
        config_outcome(status)?;
        return Ok(0);
    }

    if cmd == "scan" {
        if !matches!(via, Via::Ble(_)) {
            return Err("scan is BLE only".into());
        }
        let secs = match rest.get(1) {
            Some(&"--secs") => rest
                .get(2)
                .ok_or("--secs needs a number")?
                .parse()
                .map_err(|_| "bad --secs")?,
            _ => 5,
        };
        for (name, addr) in scan_names(secs)? {
            println!("{addr}  {name}");
        }
        return Ok(0);
    }

    if cmd == "nodes" {
        if rest.len() > 1 {
            return Err(usage.into());
        }
        let mut wire = Framed::new(via.open()?);
        let (src, found) = walk(&mut wire)?;
        println!("address  class             name");
        for f in &found {
            // A motor controller whose boot status has entries is marked (`board.status`).
            let n = match f.class {
                Some(CLASS_MOTOR) => board_status(&mut wire, src, f.addr)
                    .map(|v| v.len() / board::status::ENTRY_LEN)
                    .unwrap_or(0),
                _ => 0,
            };
            let mark = if n > 0 {
                format!("  ! {n} status entries, see \"status\"")
            } else {
                String::new()
            };
            println!(
                "{:#04x}     {:<16}  {}{mark}",
                f.addr,
                class_name(f.class),
                f.name.as_deref().unwrap_or("-")
            );
        }
        return Ok(0);
    }

    if cmd == "status" {
        let mut wire = Framed::new(via.open()?);
        let (src, found) = walk(&mut wire)?;
        let targets: Vec<&Found> = if rest.len() > 1 {
            rest[1..]
                .iter()
                .map(|spec| {
                    let a = nodes::resolve(spec, &found)?;
                    found
                        .iter()
                        .find(|f| f.addr == a)
                        .ok_or_else(|| format!("the walk found no node {a:#04x}"))
                })
                .collect::<Result<_, String>>()?
        } else {
            found
                .iter()
                .filter(|f| f.class == Some(CLASS_MOTOR))
                .collect()
        };
        if targets.is_empty() {
            return Err("the walk found no motor controller".into());
        }
        for f in targets {
            let name = node_name(f);
            let Some(value) = board_status(&mut wire, src, f.addr) else {
                println!("{name}: no board.status answer");
                continue;
            };
            let mut link_pins = |i: u8| {
                let mut pin =
                    |k: store::Key| match nodes::read(&mut wire, src, f.addr, k).ok()?.value() {
                        Some(store::Value::U8(p)) => Some(p),
                        _ => None,
                    };
                Some((
                    pin(store::LINK_TX.at(i).key())?,
                    pin(store::LINK_RX.at(i).key())?,
                ))
            };
            let pins: Vec<Option<(u8, u8)>> = (0..store::LINKS).map(&mut link_pins).collect();
            for l in hoverboard_cli::status::lines(&name, &value, |i| {
                pins.get(i as usize).copied().flatten()
            }) {
                println!("{l}");
            }
        }
        return Ok(0);
    }

    if cmd == "name" {
        let [_, spec, text] = rest[..] else {
            return Err("name needs <node> <text>".into());
        };
        let mut wire = Framed::new(via.open()?);
        let (src, mut f) = reach(&mut wire, spec)?;
        f.class = class_of(&mut wire, src, &f);
        let key = store::DEVICE_NAME.key();
        let r = nodes::write(
            &mut wire,
            src,
            f.addr,
            key,
            store::Type::Str,
            text.as_bytes(),
        )?;
        let shown = nodes::name_status(f.class, r.status);
        println!("device.name of {}: {shown}", label(&f));
        return if r.status == net::walk::CFG_OK {
            Ok(0)
        } else {
            Err(format!("refused: {shown}"))
        };
    }

    if cmd == "config" {
        let (spec, path, value) = match rest[..] {
            [_, spec, path] => (spec, path, None),
            [_, spec, path, value] => (spec, path, Some(value)),
            _ => return Err("config needs <node> <path> [<value>]".into()),
        };
        let (key, _) = names::parse_path(path).map_err(|p| p.msg)?;
        if value.is_some() {
            let checks = names::write_checks(key);
            for w in config::warnings(&checks, flags.force) {
                eprintln!("{w}");
            }
            config::stop_on(&checks, flags.force)?;
        }
        let mut wire = Framed::new(via.open()?);
        let (src, f) = reach(&mut wire, spec)?;
        let mut b = Board {
            wire: &mut wire,
            src,
            dst: f.addr,
            name: node_name(&f),
        };
        match value {
            None => println!("{}", config::config_read(&mut b, key)?),
            Some(text) => {
                let w = config::config_write(&mut b, key, text, flags.force)?;
                for l in w.warnings {
                    eprintln!("{l}");
                }
                for l in w.lines {
                    println!("{l}");
                }
            }
        }
        return Ok(0);
    }

    if cmd == "watch" {
        let secs = parse_secs(rest.get(1), "watch needs <secs> [<node>...]")?;
        let given = board_list(&rest[2.min(rest.len())..])?;
        let mut wire = Framed::new(via.open()?);
        let watched = if given.is_empty() {
            motor_controllers(&mut wire)?.1
        } else {
            attach(&mut wire)?;
            given
        };
        let (tally, elapsed) = session(
            &mut wire,
            Tally::only(&watched),
            secs,
            None,
            &AtomicBool::new(false),
        );
        print_summary(&tally, elapsed);
        return Ok(0);
    }

    if cmd == "drive" {
        let need = "drive needs <value> <secs> [<node>...]";
        let value: i16 = rest
            .get(1)
            .ok_or(need)?
            .parse()
            .map_err(|_| format!("not a drive value (-32768..32767): {}", rest[1]))?;
        let secs = parse_secs(rest.get(2), need)?;
        let given = board_list(&rest[3.min(rest.len())..])?;
        let stop = Arc::new(AtomicBool::new(false));
        let flag = stop.clone();
        ctrlc::set_handler(move || flag.store(true, Ordering::Relaxed))
            .map_err(|e| e.to_string())?;
        let mut wire = Framed::new(via.open()?);
        let (src, nodes) = if given.is_empty() {
            motor_controllers(&mut wire)?
        } else {
            (attach(&mut wire)?, given)
        };
        let mut drive = Drive::new(src, nodes, value);
        let (tally, elapsed) = session(&mut wire, Tally::new(), secs, Some(&mut drive), &stop);
        for _ in 0..STOP_REPEATS {
            for f in drive.stop() {
                wire.send(&f);
            }
            std::thread::sleep(STOP_GAP);
        }
        if stop.load(Ordering::Relaxed) {
            eprintln!("interrupted");
        }
        println!("neutral sent, power 0 sent");
        print_summary(&tally, elapsed);
        return Ok(0);
    }

    if image_flag.is_some() && cmd != "flash" {
        return Err("--image chooses the image `flash` takes from a bundle".into());
    }
    if rest.contains(&"--dirty") && cmd != "firmware" {
        return Err(usage.into());
    }

    if cmd == "firmware" {
        if node.is_some() || !matches!(given, Given::Nothing) {
            return Err(
                "firmware reads and writes bundle files; it takes no link and no --node".into(),
            );
        }
        let dirty = rest.contains(&"--dirty");
        rest.retain(|a| *a != "--dirty");
        return match rest[1..] {
            ["info", path] if !dirty => {
                let b = open_bundle(path)?;
                print!("{}", b.describe());
                Ok(0)
            }
            ["pack", out, ref entries @ ..] if !entries.is_empty() => {
                let entries = entries
                    .iter()
                    .map(|a| TreeEntry::parse(a))
                    .collect::<Result<Vec<_>, _>>()?;
                let z = bundle::pack_tree(&entries, dirty)?;
                std::fs::write(out, &z).map_err(|e| format!("{out}: {e}"))?;
                let b = Bundle::open(&z).map_err(|e| e.to_string())?;
                println!("wrote {out} ({} B)", z.len());
                print!("{}", b.describe());
                Ok(0)
            }
            _ => Err(usage.into()),
        };
    }

    if cmd == "install-esp" {
        let yes = rest.contains(&"--yes");
        rest.retain(|a| *a != "--yes");
        let need = "install-esp needs <image.bin> [<file>...] [--yes]";
        let [_, image, ref files @ ..] = rest[..] else {
            return Err(need.into());
        };
        if rest.iter().any(|a| a.starts_with("--")) {
            return Err(need.into());
        }
        if node.is_some() {
            return Err("install-esp takes no --node: it installs the bridge on the port".into());
        }
        return install_esp(given, image, files, yes).map(|()| 0);
    }

    if cmd == "erase-store" {
        let yes = rest.contains(&"--yes");
        rest.retain(|a| *a != "--yes");
        if rest.len() != 1 {
            return Err("erase-store takes [--yes]".into());
        }
        return erase_store(&via, node, yes).map(|()| 0);
    }

    if cmd == "update-esp" {
        let need = "update-esp needs <bin> [--no-confirm]";
        let path = *rest.get(1).ok_or(need)?;
        let no_confirm = match &rest[2..] {
            [] => false,
            ["--no-confirm"] => true,
            _ => return Err(need.into()),
        };
        return update_esp(&via, node, path, no_confirm).map(|()| 0);
    }

    // Read the file before the link, so a refused image or a damaged bundle costs no connection.
    let flash_yes = cmd == "flash" && rest.contains(&"--yes");
    let flash_file = if cmd == "flash" {
        rest.retain(|a| *a != "--yes");
        let ["flash", path] = rest[..] else {
            return Err("flash needs <bundle|elf> [--image NAME] [--yes]".into());
        };
        let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
        Some(
            match bundle::kind(&bytes).map_err(|e| format!("{path}: {e}"))? {
                Kind::Bundle => {
                    let b = Bundle::open(&bytes).map_err(|e| e.to_string())?;
                    eprintln!(
                        "bundle built from commit {} at {}, {} image(s), SHA-256 checked",
                        b.manifest.commit,
                        b.manifest.built,
                        b.manifest.images.len()
                    );
                    FlashFile::Bundle(b)
                }
                Kind::Elf => {
                    if image_flag.is_some() {
                        return Err(format!(
                            "{path} is an ELF; --image chooses an image in a bundle"
                        ));
                    }
                    elf::check_wfi(&bytes).map_err(|e| e.to_string())?;
                    let bin = elf::to_bin(&bytes).map_err(|e| e.to_string())?;
                    eprintln!(
                    "ELF given directly: no wfi, layout checked; the image guards run on a bundle"
                );
                    FlashFile::Bin(bin)
                }
            },
        )
    } else {
        None
    };

    let mut c = esp_client(&via, node)?;
    match cmd {
        "info" => print_info(&c.info().map_err(e)?),
        "id" => print_id(&c.target_id().map_err(e)?),
        "clear-protection" => {
            let id = c.target_id().map_err(e)?;
            print_id(&id);
            if !confirm(
                "Clearing protection MASS-ERASES the chip: the stock firmware is lost unless it \
                 was backed up with gdf130backup. Continue?",
            ) {
                return Err("not cleared".into());
            }
            c.clear_protection().map_err(e)?;
            println!("cleared; USER byte recorded as {:#04x}", id.user);
            print!("Power-cycle the board by hand, then press Enter. ");
            let _ = std::io::stdout().flush();
            let _ = std::io::stdin().lock().read_line(&mut String::new());
            let after = c.target_id().map_err(e)?;
            print_id(&after);
            if after.rdp != 0xA5 || after.user != id.user {
                return Err(format!(
                    "the clear did not take: protection {:#04x}, USER {:#04x} (was {:#04x})",
                    after.rdp, after.user, id.user
                ));
            }
            println!("protection off, USER byte unchanged");
        }
        "disarm" => c.ctrl(Ctrl::Disarm).map_err(e)?,
        "halt" => c.ctrl(Ctrl::Halt).map_err(e)?,
        "resume" => c.ctrl(Ctrl::Resume).map_err(e)?,
        "reset" => c.ctrl(Ctrl::Reset).map_err(e)?,
        "reset-halt" => c.ctrl(Ctrl::ResetHalt).map_err(e)?,
        "read" => {
            let addr = parse_u32(rest.get(1).ok_or("read needs <addr> <len>")?)?;
            let len = parse_u32(rest.get(2).ok_or("read needs <addr> <len>")?)? as usize;
            hexdump(addr, &c.read(addr, len).map_err(e)?);
        }
        "write" => {
            let addr = parse_u32(rest.get(1).ok_or("write needs <addr> <hex>")?)?;
            let data = parse_hex(rest.get(2).ok_or("write needs <addr> <hex>")?)?;
            if addr % 4 != 0 || data.len() % 4 != 0 {
                return Err("write takes a word-aligned address and whole words".into());
            }
            c.write(addr, &data).map_err(e)?;
        }
        "flash" => {
            let image = match flash_file.expect("read above") {
                FlashFile::Bin(bin) => bin,
                FlashFile::Bundle(b) => {
                    let id = c.target_id().map_err(e)?;
                    let (img, chip) = b.select(&id, image_flag).map_err(|e| e.to_string())?;
                    println!("{}", bundle::choice_line(&b.manifest, img, chip));
                    let bin = b.prepare(img).map_err(|e| e.to_string())?;
                    eprintln!("{}: no wfi, its guards and the layout checked", img.name);
                    if !flash_yes {
                        if !std::io::stdin().is_terminal() {
                            return Err("give --yes to flash without a terminal".into());
                        }
                        if !confirm(&format!("Flash {}?", img.name)) {
                            return Err("not flashed".into());
                        }
                    }
                    bin
                }
            };
            let t0 = std::time::Instant::now();
            let mut stage = None;
            let mut times = StageTimes::new();
            c.flash(&image, |p: Progress| {
                times.on(p, t0.elapsed());
                if stage.is_some() && stage != Some(p.stage) {
                    eprintln!();
                }
                stage = Some(p.stage);
                eprint!(
                    "\r{:?} {}/{} ({:.1} s)   ",
                    p.stage,
                    p.done,
                    p.total,
                    t0.elapsed().as_secs_f32()
                );
            })
            .map_err(e)?;
            eprintln!();
            println!(
                "booted ({} B in {:.1} s)",
                image.len(),
                t0.elapsed().as_secs_f32()
            );
            let stages: Vec<String> = times
                .durations()
                .iter()
                .map(|(name, d)| format!("{name} {:.2} s", d.as_secs_f32()))
                .collect();
            println!("stages: {}", stages.join(", "));
        }
        _ => unreachable!("{cmd} is in COMMANDS and handled above"),
    }
    Ok(0)
}

/// What `flash` was given, read before the link is opened.
enum FlashFile {
    /// An ELF given directly, converted.
    Bin(Vec<u8>),
    /// A bundle, opened and checked; the image is chosen once `TARGET_ID` is read.
    Bundle(Bundle),
}

/// Read and open a bundle, refusing a file that is not one.
fn open_bundle(path: &str) -> Result<Bundle, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if bundle::kind(&bytes) != Ok(Kind::Bundle) {
        return Err(format!("{path}: not a firmware bundle (a zip archive)"));
    }
    Bundle::open(&bytes).map_err(|e| e.to_string())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => ExitCode::from(code),
        Err(msg) => {
            eprintln!("error: {msg}");
            ExitCode::FAILURE
        }
    }
}
