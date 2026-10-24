//! `hoverboard-cli`: talk to an ESP32 board tool over BLE, its USB console, or that console through
//! a TCP relay.
//!
//! ```text
//! hoverboard-cli scan [--secs N]
//! hoverboard-cli [LINK] info | id | clear-protection
//! hoverboard-cli [LINK] disarm | halt | resume | reset | reset-halt
//! hoverboard-cli [LINK] read <addr> <len> | write <addr> <hex>
//! hoverboard-cli [LINK] flash <elf>
//!
//! LINK: --device NAME|ADDRESS   BLE (the default; with no --device, the first advertiser whose
//!                               name starts with `hb-esp-`)
//!       --serial DEVICE         the ESP's USB console (/dev/ttyACM0, /dev/cu.usbmodem*)
//!       --tcp HOST:PORT         the USB console through a serial-to-TCP relay
//! ```
//!
//! `scan` is BLE only.

use std::io::{BufRead, Write as _};
use std::process::ExitCode;
use std::sync::mpsc;
use std::time::Duration;

use btleplug::api::{
    Central, CentralState, CharPropFlags, Characteristic, Manager as _, Peripheral as _,
    ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use espsvc::{Ctrl, FAMILY_F1X0};
use futures::StreamExt;
use tokio::runtime::Runtime;

use hoverboard_cli::client::{Client, ClientError, Progress};
use hoverboard_cli::elf;
use hoverboard_cli::transport::{Serial, Tcp};
use hoverboard_cli::wire::{ByteLink, Framed};

/// The CC2541's notification and write size, which the ESP keeps.
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
/// How long finding, connecting to and subscribing to an ESP may take.
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
                    None => n.starts_with("hb-esp-"),
                })
                .ok_or_else(|| match device {
                    Some(d) => format!("no advertiser named or at {d}"),
                    None => "no hb-esp- advertiser found".to_string(),
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
            _ => "none since the ESP booted",
        }
    );
    println!("flash       {} KiB", id.flash_kib);
    println!("armed       {}", if id.armed { "YES" } else { "no" });
}

fn confirm(prompt: &str) -> bool {
    print!("{prompt} [y/N] ");
    let _ = std::io::stdout().flush();
    let mut line = String::new();
    let _ = std::io::stdin().lock().read_line(&mut line);
    line.trim().eq_ignore_ascii_case("y")
}

/// Which link to the ESP.
enum Via<'a> {
    /// BLE, to this advertiser or the first `hb-esp-` one.
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
            "--secs" => {
                rest.push(a); // scan's own option
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
    let usage = "usage: hoverboard-cli [--device NAME|ADDR | --serial DEVICE | --tcp HOST:PORT] scan|info|id|flash <elf>|clear-protection|disarm|halt|resume|reset|reset-halt|read <addr> <len>|write <addr> <hex>";
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

    let mut c = Client::new(Framed::new(via.open()?));
    match cmd {
        "info" => {
            let i = c.info().map_err(e)?;
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
        }
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
