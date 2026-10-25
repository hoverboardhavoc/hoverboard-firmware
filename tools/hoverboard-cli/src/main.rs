//! `hoverboard-cli`: talk to a hoverboard bridge over BLE, its USB console, or that console through
//! a TCP relay.
//!
//! ```text
//! hoverboard-cli scan [--secs N]
//! hoverboard-cli [LINK] nodes
//! hoverboard-cli [LINK] name <node> <text>
//! hoverboard-cli [LINK] config <node> <field>[.<index>] [<value>]
//! hoverboard-cli [LINK] [--node NODE] info | id | clear-protection
//! hoverboard-cli [LINK] [--node NODE] disarm | halt | resume | reset | reset-halt
//! hoverboard-cli [LINK] [--node NODE] read <addr> <len> | write <addr> <hex>
//! hoverboard-cli [LINK] [--node NODE] flash <elf>
//! hoverboard-cli [LINK] [--node NODE] update-esp <bin> [--no-confirm]
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
//! A node is an address (`0x03`) or a `device.name`. `nodes` walks the machine with `net`'s
//! controller and lists every node's address, class and name; `name` and `config` walk, then write
//! or read the node's key, typed as the store's registry says, and print the status by name.
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
//! `update-esp` sends a bridge image (`espflash save-image` output) into the bridge's other app slot,
//! waits for the bridge to reset into it, reconnects over the same link, and confirms it; with
//! `--no-confirm` it stops after the reconnect, and the bridge's next reset returns to the previous
//! slot.

use std::io::{BufRead, Write as _};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

use btleplug::api::{
    Central, CentralState, CharPropFlags, Characteristic, Manager as _, Peripheral as _,
    ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use espsvc::{Ctrl, FAMILY_F1X0};
use futures::StreamExt;
use net::pdu::NO_ADDRESS;
use tokio::runtime::Runtime;

use hoverboard_cli::client::{Client, ClientError, Progress};
use hoverboard_cli::elf;
use hoverboard_cli::guest::{self, Drive, Tally, STOP_REPEATS};
use hoverboard_cli::nodes::{self, class_name, Found, CLASS_MOTOR};
use hoverboard_cli::transport::{Serial, Tcp};
use hoverboard_cli::wire::{ByteLink, Framed, Wire};

/// The CC2541's notification and write size, which the bridge keeps.
const BLE_CHUNK: usize = 20;

struct Ble {
    rt: Runtime,
    periph: Peripheral,
    write_char: Characteristic,
    rx: mpsc::Receiver<Vec<u8>>,
}

impl ByteLink for Ble {
    fn write(&mut self, bytes: &[u8]) {
        for c in bytes.chunks(BLE_CHUNK) {
            let _ = self.rt.block_on(self.periph.write(
                &self.write_char,
                c,
                WriteType::WithoutResponse,
            ));
        }
    }

    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        let mut out = match self.rx.recv_timeout(timeout) {
            Ok(v) => v,
            Err(_) => return Vec::new(),
        };
        while let Ok(v) = self.rx.try_recv() {
            out.extend(v);
        }
        out
    }
}

/// How long the Bluetooth stack may take to report an adapter and its state.
const ADAPTER_WAIT: Duration = Duration::from_secs(5);
/// How long finding, connecting to and subscribing to a bridge may take.
const CONNECT_WAIT: Duration = Duration::from_secs(30);

const NO_BLUETOOTH: &str =
    "no powered-on Bluetooth adapter on this host; reach the ESP's USB console with --serial or --tcp";

/// The first adapter, if it is powered on. A host with no Bluetooth either lists none or never
/// answers (CoreBluetooth on a machine without the hardware), so both are bounded.
async fn adapter() -> Result<Adapter, String> {
    let found = tokio::time::timeout(ADAPTER_WAIT, async {
        let manager = Manager::new().await.map_err(|e| e.to_string())?;
        let a = manager
            .adapters()
            .await
            .map_err(|e| e.to_string())?
            .into_iter()
            .next()
            .ok_or(NO_BLUETOOTH)?;
        match a.adapter_state().await.map_err(|e| e.to_string())? {
            CentralState::PoweredOn => Ok(a),
            _ => Err(NO_BLUETOOTH.to_string()),
        }
    })
    .await;
    found.unwrap_or_else(|_| Err(NO_BLUETOOTH.to_string()))
}

/// Advertisers seen in `secs`: (name, address, peripheral).
async fn scan(secs: u64) -> Result<Vec<(String, String, Peripheral)>, String> {
    let a = adapter().await?;
    a.start_scan(ScanFilter::default())
        .await
        .map_err(|e| e.to_string())?;
    tokio::time::sleep(Duration::from_secs(secs)).await;
    let _ = a.stop_scan().await;
    let mut out = Vec::new();
    for p in a.peripherals().await.map_err(|e| e.to_string())? {
        if let Ok(Some(props)) = p.properties().await {
            if let Some(name) = props.local_name {
                out.push((name, p.address().to_string(), p));
            }
        }
    }
    out.sort_by(|x, y| x.0.cmp(&y.0));
    Ok(out)
}

fn connect(device: Option<&str>) -> Result<Ble, String> {
    let rt = Runtime::new().map_err(|e| e.to_string())?;
    let (periph, write_char, rx) = rt.block_on(async {
        tokio::time::timeout(CONNECT_WAIT, async {
            let found = scan(4).await?;
            let (name, _, periph) = found
                .into_iter()
                .find(|(n, a, _)| match device {
                    Some(d) => n == d || a.eq_ignore_ascii_case(d),
                    None => n.starts_with("hb-bridge-"),
                })
                .ok_or_else(|| match device {
                    Some(d) => format!("no advertiser named or at {d}"),
                    None => "no hb-bridge- advertiser found".to_string(),
                })?;
            periph.connect().await.map_err(|e| e.to_string())?;
            periph
                .discover_services()
                .await
                .map_err(|e| e.to_string())?;
            // By property, not UUID, as the app's transport does.
            let chars = periph.characteristics();
            let write_char = chars
                .iter()
                .find(|c| c.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE))
                .cloned()
                .ok_or("no write-without-response characteristic")?;
            let notify_char = chars
                .iter()
                .find(|c| c.properties.contains(CharPropFlags::NOTIFY))
                .cloned()
                .ok_or("no notify characteristic")?;
            periph
                .subscribe(&notify_char)
                .await
                .map_err(|e| e.to_string())?;
            let mut stream = periph.notifications().await.map_err(|e| e.to_string())?;
            let (tx, rx) = mpsc::channel();
            tokio::spawn(async move {
                while let Some(n) = stream.next().await {
                    if tx.send(n.value).is_err() {
                        break;
                    }
                }
            });
            eprintln!("connected to {name}");
            Ok::<_, String>((periph, write_char, rx))
        })
        .await
        .unwrap_or_else(|_| {
            Err(format!(
                "no BLE connection within {} s",
                CONNECT_WAIT.as_secs()
            ))
        })
    })?;
    Ok(Ble {
        rt,
        periph,
        write_char,
        rx,
    })
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
    println!("mailbox     {state}");
    let [a, b, c, d] = i.ip;
    match espsvc::WifiState::from_u8(i.wifi) {
        Some(espsvc::WifiState::Up) => println!("wifi        up, {a}.{b}.{c}.{d}"),
        Some(espsvc::WifiState::Joined) => println!("wifi        joined, no address yet"),
        Some(espsvc::WifiState::Down) => println!("wifi        not joined"),
        None => println!("wifi        {}", i.wifi),
    }
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
}

fn slot_name(i: &espsvc::EspInfo) -> String {
    let slot = match i.slot {
        espsvc::SLOT_NONE => "neither app slot".to_string(),
        n => format!("ota_{n}"),
    };
    if i.pending {
        format!("{slot}, pending verification")
    } else {
        slot
    }
}

/// How long the bridge may take to reset into an update and answer again (Wi-Fi rejoin and DHCP
/// included).
const UPDATE_REBOOT: Duration = Duration::from_secs(120);
/// The pause before each reconnect attempt.
const RECONNECT_GAP: Duration = Duration::from_secs(2);

/// A client over whichever link was opened.
type EspClient = Client<Framed<Box<dyn ByteLink>>>;

/// Open `via` again until the bridge answers `ESP_INFO`, within [`UPDATE_REBOOT`].
fn reconnect(via: &Via, node: Option<&str>) -> Result<(EspClient, espsvc::EspInfo), String> {
    let t0 = Instant::now();
    let mut why = String::from("no answer");
    while t0.elapsed() < UPDATE_REBOOT {
        std::thread::sleep(RECONNECT_GAP);
        match esp_client(via, node) {
            Ok(mut c) => match c.info() {
                Ok(i) => return Ok((c, i)),
                Err(e) => why = e.to_string(),
            },
            Err(e) => why = e,
        }
    }
    Err(format!(
        "the bridge did not answer within {} s of the update ({why})",
        UPDATE_REBOOT.as_secs()
    ))
}

/// `update-esp`: send the image, reconnect after the reset, check the new slot runs pending, and
/// confirm it unless `no_confirm`.
fn update_esp(via: &Via, node: Option<&str>, path: &str, no_confirm: bool) -> Result<(), String> {
    let image = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let version = espsvc::app_version(&image).ok_or_else(|| {
        format!("{path}: not an ESP32-C3 app image (make one with espflash save-image)")
    })?;
    let e = |e: ClientError| e.to_string();
    let mut c = esp_client(via, node)?;
    let before = c.info().map_err(e)?;
    eprintln!(
        "running {}; sending {} B, version {}.{}.{}",
        slot_name(&before),
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

    let (mut c, after) = reconnect(via, node)?;
    if after.slot == before.slot || !after.pending {
        return Err(format!(
            "the bridge did not boot the new image: it runs {}",
            slot_name(&after)
        ));
    }
    if no_confirm {
        print_info(&after);
        println!(
            "not confirmed: the bridge's next reset returns to {}",
            slot_name(&before)
        );
        return Ok(());
    }
    c.confirm().map_err(e)?;
    print_info(&c.info().map_err(e)?);
    Ok(())
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
            let (src, f) = walk_to(&mut wire, spec)?;
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
            Via::Ble(device) => Box::new(connect(device)?),
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

fn run(args: &[String]) -> Result<(), String> {
    let mut via = None;
    let mut node: Option<&str> = None;
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
            "--secs" | "--no-confirm" => {
                rest.push(a); // scan's and update-esp's own options
                continue;
            }
            "--node" => {
                let v = match inline {
                    Some(v) => v,
                    None => it.next().ok_or("--node needs a value")?,
                };
                if node.replace(v).is_some() {
                    return Err("give --node once".into());
                }
                continue;
            }
            o if o.starts_with("--") => return Err(format!("unknown option {o}")),
            _ => {
                rest.push(a);
                continue;
            }
        };
        let v = match inline {
            Some(v) => v,
            None => it.next().ok_or_else(|| format!("{opt} needs a value"))?,
        };
        if via.replace(make(v)).is_some() {
            return Err("give one of --device, --serial, --tcp".into());
        }
    }
    let via = via.unwrap_or(Via::Ble(None));
    let usage = "usage: hoverboard-cli [--device NAME|ADDR | --serial DEVICE | --tcp HOST:PORT] [--node NODE] scan|nodes|name <node> <text>|config <node> <field>[.<index>] [<value>]|info|id|flash <elf>|update-esp <bin> [--no-confirm]|clear-protection|disarm|halt|resume|reset|reset-halt|read <addr> <len>|write <addr> <hex>|watch <secs> [<node>...]|drive <value> <secs> [<node>...]";
    let cmd = *rest.first().ok_or(usage)?;
    let e = |e: ClientError| e.to_string();

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
        let rt = Runtime::new().map_err(|e| e.to_string())?;
        for (name, addr, _) in rt.block_on(scan(secs))? {
            println!("{addr}  {name}");
        }
        return Ok(());
    }

    if cmd == "nodes" {
        if rest.len() > 1 {
            return Err(usage.into());
        }
        let mut wire = Framed::new(via.open()?);
        let (_, found) = walk(&mut wire)?;
        println!("address  class             name");
        for f in &found {
            println!(
                "{:#04x}     {:<16}  {}",
                f.addr,
                class_name(f.class),
                f.name.as_deref().unwrap_or("-")
            );
        }
        return Ok(());
    }

    if cmd == "name" {
        let [_, spec, text] = rest[..] else {
            return Err("name needs <node> <text>".into());
        };
        let mut wire = Framed::new(via.open()?);
        let (src, f) = walk_to(&mut wire, spec)?;
        let key = store::DEVICE_NAME.key();
        let r = nodes::write(
            &mut wire,
            src,
            f.addr,
            key,
            store::Type::Str,
            text.as_bytes(),
        )?;
        println!(
            "device.name of {}: {}",
            label(&f),
            nodes::status_name(r.status)
        );
        return config_outcome(r.status);
    }

    if cmd == "config" {
        let (spec, field, value) = match rest[..] {
            [_, spec, field] => (spec, field, None),
            [_, spec, field, value] => (spec, field, Some(value)),
            _ => return Err("config needs <node> <field>[.<index>] [<value>]".into()),
        };
        let key = nodes::parse_key(field)?;
        let typed = value.map(|v| nodes::parse_value(key, v)).transpose()?;
        let mut wire = Framed::new(via.open()?);
        let (src, f) = walk_to(&mut wire, spec)?;
        let r = match typed {
            None => nodes::read(&mut wire, src, f.addr, key)?,
            Some((kind, bytes)) => nodes::write(&mut wire, src, f.addr, key, kind, &bytes)?,
        };
        let shown = r.value().map(|v| nodes::show(&v)).unwrap_or_default();
        println!(
            "{:#04x}.{} at {}: {} {shown}",
            key.field_id,
            key.index,
            label(&f),
            nodes::status_name(r.status)
        );
        return config_outcome(r.status);
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
        return Ok(());
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
        return Ok(());
    }

    if cmd == "update-esp" {
        let need = "update-esp needs <bin> [--no-confirm]";
        let path = *rest.get(1).ok_or(need)?;
        let no_confirm = match &rest[2..] {
            [] => false,
            ["--no-confirm"] => true,
            _ => return Err(need.into()),
        };
        return update_esp(&via, node, path, no_confirm);
    }

    // Read the ELF before the link, so a refused image costs no connection.
    let image = if cmd == "flash" {
        let path = rest.get(1).ok_or("flash needs an ELF path")?;
        let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
        elf::check(&bytes).map_err(|e| e.to_string())?;
        eprintln!("ELF checks passed (no wfi, code floor, required symbols)");
        Some(elf::to_bin(&bytes).map_err(|e| e.to_string())?)
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
            let image = image.expect("read above");
            let t0 = std::time::Instant::now();
            let mut stage = None;
            c.flash(&image, |p: Progress| {
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
        }
        _ => return Err(usage.into()),
    }
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(msg) => {
            eprintln!("hoverboard-cli: {msg}");
            ExitCode::FAILURE
        }
    }
}
