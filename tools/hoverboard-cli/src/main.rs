//! `hoverboard-cli`: talk to a hoverboard bridge over BLE, its USB console, or that console through
//! a TCP relay. BLE is the `ble` cargo feature, on by default; a build without it (the bench Pi's)
//! refuses `--device` and `scan`.
//!
//! Every command starts with the noun it acts on (`specs/machine-settings.md` section 6.3): `board`
//! (one motor controller), `machine` (every motor controller) or `bridge`; the link options come
//! before it, and `scan` stands outside. `hoverboard-cli` with no command prints the whole grammar
//! ([`cli::USAGE`]).
//!
//! ```text
//! LINK: --device NAME|ADDRESS   BLE (the default; with no --device, the first advertiser whose
//!                               name starts with `hb-bridge-`)
//!       --serial DEVICE         the ESP's USB console (/dev/ttyACM0, /dev/cu.usbmodem*)
//!       --tcp HOST:PORT         the bridge's Wi-Fi console (<name>.local:5333), or the USB
//!                               console through a serial-to-TCP relay
//! ```
//!
//! A board or bridge is named by a target ([`target`]): an address (`0x03`), which needs no walk,
//! else an exact `device.name`, else a prefix of its factory unique ID (`8adc`), which need the walk.
//!
//! `board get`, `set`, `dump`, `apply`, `replace` and `diff` (and the same under `bridge`) treat a
//! node's store as a file (`specs/cli-config.md`): a field goes by its name in the store's name
//! table (`motors.1.hall.a`, `machine.mode`) or by its raw `<id>.<index>`, and a value takes the
//! file's forms (`PC13`, `balance`, `null` to un-store). Every line about a board starts with its
//! `device.name`. `diff` exits 0 with no changes, 1 with changes, 2 on an error. A secret
//! (`wifi.pass`) is never printed (`specs/bridge-onboarding.md` section 2.2). A machine field
//! written through `board` goes to that board alone, and the command says the boards now differ.
//!
//! The `board` commands that use SWD (`erase`, `flash`, `clear-protection`, the target controls,
//! `read`, `write`, `target-id`) go through the bridge wired to the board ([`route`]): straight
//! through the connected bridge when the `uid` prefix names the target it reads over SWD, or the
//! address is on its mailbox, and found by the walk otherwise. `board erase` goes only through the
//! board's own bridge, the one this link reaches, and `board halt` never over a route through the
//! board it halts. `board read --hz` routes once and reads at that rate until interrupted.
//!
//! The `machine` commands act on every motor controller the walk finds: `nodes` lists the walk and
//! marks each board from its `CYCLIC_STATE` and boot report; `get` and `set` a machine field;
//! `dump`, `apply`, `replace` and `diff` a machine file, whose `boards` entries go to the board
//! reporting each `uid`; `watch` and `drive` attach as an L3 guest.
//!
//! `board update` takes a firmware bundle and `board flash` an ELF; each refuses the other's file,
//! told apart by its first bytes (`specs/firmware-bundle.md` section 4). From a bundle,
//! `board update` checks each ELF's SHA-256, reads the board's `TARGET_ID`, and takes the image
//! whose target equals the chip's family and flash size; with several, `--image` names one of them,
//! and it cannot name one that does not match. A match the bridge would refuse (a part that is not
//! 64 KiB, section 3.4) is refused before anything is streamed. It prints the choice, runs that
//! image's guards from the manifest and asks, unless `--yes`; without a terminal it refuses unless
//! `--yes` is given. `board flash`'s ELF gets the `wfi` scan and the layout check only: it does not
//! say which chip it is for or which guards apply.
//!
//! `firmware info` prints a bundle's manifest and reads no board. `firmware pack` builds one: each
//! entry's role and targets from its crate's `[package.metadata.hoverboard-image.<name>]`, its
//! guards from `tools/image-guard-syms.sh`, the commit from `git rev-parse HEAD`; a tree with
//! changes to tracked files is refused unless `--dirty`, which suffixes the commit `-dirty`.
//!
//! `bridge update` sends a bridge image into the bridge's other app slot, reconnects and confirms
//! it; it needs no walk and no `IDENTIFY` for an address, so it reaches a bridge on an older image.
//! From a bundle it takes the `bridge` image for the chip `ESP_INFO` reports and asks first, unless
//! `--yes`; a single app image is sent with no question. `bridge install` makes a SuperMini a bridge
//! over its own USB port (`specs/bridge-onboarding.md` section 3), from a bundle's
//! `bridge-install` image or a merged install image.
//!
//! `machine update` (`specs/firmware-bundle.md` section 4.3) walks the machine over the given link,
//! pairs each board with the bridge wired to it, reads every `ESP_INFO` and `TARGET_ID` and chooses
//! every image before it asks once. Then it takes one pair at a time over that bridge's own link
//! (`<name>.local:5333`, else BLE by name; the given link for its own bridge, which goes last):
//! the bridge's update, a reconnect after its reboot, the board's update routed by its `uid`, and
//! its boot report. It stops at the first failure, prints what was done, failed and not started,
//! and reports each bridge's commit and the machine digests over the given link.

use std::collections::BTreeMap;
use std::io::{BufRead, IsTerminal, Write as _};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use espsvc::{Ctrl, Status};
use net::pdu::NO_ADDRESS;
use net::walk::{Controller, CLASS_BRIDGE};

use hoverboard_cli::bundle::{self, Bundle, FileKind, ImageKind, TreeEntry};
use hoverboard_cli::checks;
use hoverboard_cli::cli::{self, Command, Link, Mode, Noun, Swd};
use hoverboard_cli::client::{sample_line, Client, ClientError, Progress, StageTimes};
use hoverboard_cli::config::{self, Board};
use hoverboard_cli::elf;
use hoverboard_cli::erase;
use hoverboard_cli::fleet;
use hoverboard_cli::guest::{self, Drive, Tally, Wheel, WheelTarget, STOP_REPEATS};
use hoverboard_cli::install::{self, Answered, Given, Guard};
use hoverboard_cli::machine::{self, MachineFile};
use hoverboard_cli::names;
use hoverboard_cli::nodes::{self, class_name, Found, Walked, CLASS_MOTOR};
use hoverboard_cli::route::{self, Purpose, Route, Via};
use hoverboard_cli::target::{self, check_class, class_of, label, node_name, Target};
use hoverboard_cli::transport::{Serial, Tcp};
use hoverboard_cli::wire::{ByteLink, Framed, Wire};

#[cfg(feature = "ble")]
mod ble;

#[cfg(test)]
mod machine_update_sim;

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

fn hexdump(addr: u32, data: &[u8]) {
    for (i, row) in data.chunks(16).enumerate() {
        let hex: Vec<String> = row.iter().map(|b| format!("{b:02x}")).collect();
        println!("{:08x}: {}", addr + 16 * i as u32, hex.join(" "));
    }
}

/// The family `TARGET_ID` reports, for a terminal.
fn family_name(family: u8) -> &'static str {
    match family {
        espsvc::FAMILY_F10X => "GD32F10x",
        espsvc::FAMILY_F1X0 => "GD32F1x0",
        espsvc::FAMILY_MM32SPIN05 => "MM32SPIN05",
        espsvc::FAMILY_UNSUPPORTED => "unsupported",
        _ => "unknown",
    }
}

/// A bridge refusal as text; an unsupported part's carries the `cpuid` and `mcu_id` it reports
/// (`specs/bridge-chip-support.md`, section 3).
fn refusal(c: &mut EspClient, e: ClientError) -> String {
    let unsupported = matches!(
        e,
        ClientError::Status(Status::UnsupportedPart)
            | ClientError::Flash(_, Status::UnsupportedPart)
    );
    match c.target_id() {
        Ok(id) if unsupported => {
            format!("{e} (cpuid {:#010x}, mcu id {:#010x})", id.cpuid, id.mcu_id)
        }
        _ => e.to_string(),
    }
}

/// A bridge refusal of `board update`, `board flash`, `board clear-protection` or `board
/// target-id` as text: [`refusal`]'s, with the recover hint at its end when the bridge reads no SWD
/// target (`specs/bridge-nrst.md` section 3).
fn identify_refusal(c: &mut EspClient, e: ClientError) -> String {
    match e {
        ClientError::Status(Status::NoTarget) | ClientError::Flash(_, Status::NoTarget) => {
            route::identify_refusal(&e)
        }
        e => refusal(c, e),
    }
}

fn print_id(id: &espsvc::TargetId) {
    let family = family_name(id.family);
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
/// How long `machine nodes` and the end of `machine apply` listen for `CYCLIC_STATE`: every motor
/// controller's frames arrive on the attached port at 5 Hz or faster.
const LISTEN: Duration = Duration::from_secs(1);

/// A client over whichever link was opened.
type EspClient = Client<Framed<Box<dyn ByteLink>>>;

/// Open `link`.
fn open(link: &Link) -> Result<Box<dyn ByteLink>, String> {
    Ok(match link {
        Link::Ble(device) => ble_link(device.as_deref())?,
        Link::Serial(device) => {
            let s = Serial::open(device)?;
            eprintln!("opened {device}");
            Box::new(s)
        }
        Link::Tcp(addr) => {
            let t = Tcp::connect(addr)?;
            eprintln!("connected to {addr}");
            Box::new(t)
        }
    })
}

/// A framed wire over `link`.
fn wire_on(link: &Link) -> Result<Framed<Box<dyn ByteLink>>, String> {
    Ok(Framed::new(open(link)?))
}

/// Open `link` again until the bridge `spec` names answers the attach, within [`UPDATE_REBOOT`].
fn reconnect(link: &Link, spec: &str) -> Result<EspClient, String> {
    let t0 = Instant::now();
    let mut why = String::from("no answer");
    while t0.elapsed() < UPDATE_REBOOT {
        std::thread::sleep(RECONNECT_GAP);
        match bridge_client(link, spec, false) {
            Ok(c) => return Ok(c),
            Err(e) => why = e,
        }
    }
    Err(format!(
        "the bridge did not answer within {} s of the update ({why})",
        UPDATE_REBOOT.as_secs()
    ))
}

/// The note `bridge update` prints for an `ESP_INFO` it cannot decode.
const OTHER_LAYOUT: &str = "bridge's info has another layout; continuing";

/// `ESP_INFO` for `bridge update`: `None`, with [`OTHER_LAYOUT`] printed, when it does not decode.
fn info_or_note(c: &mut EspClient) -> Result<Option<espsvc::EspInfo>, String> {
    let i = c.info_if_decodes().map_err(|e| e.to_string())?;
    if i.is_none() {
        eprintln!("{OTHER_LAYOUT}");
    }
    Ok(i)
}

/// `bridge update` (`specs/firmware-bundle.md` section 4.2): from a bundle, the `bridge` image for
/// the chip `ESP_INFO` reports, after a question unless `yes`; from a single app image, that image
/// with no question. The file is read and checked before the link is opened.
fn bridge_update(
    link: &Link,
    spec: &str,
    path: &str,
    yes: bool,
    no_confirm: bool,
) -> Result<(), String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if !bundle::is_bundle(&bytes) {
        let version = bundle::check_bridge_file(ImageKind::Bridge, &bytes)
            .map_err(|e| format!("{path}: {e}"))?
            .expect("an app image has a version");
        let mut c = bridge_client(link, spec, false)?;
        let before = info_or_note(&mut c)?;
        return send_bridge_image(c, link, spec, &bytes, version, before, no_confirm);
    }
    let b = open_checked_bundle(path, &bytes)?;
    let mut c = bridge_client(link, spec, false)?;
    let before = info_or_note(&mut c)?;
    let img = b
        .select_bridge(ImageKind::Bridge, before.map(|i| i.chip))
        .map_err(|e| e.to_string())?;
    let (file, version) = b.bridge_file(img)?;
    let version = version.expect("an app image has a version");
    if before.is_none() {
        eprintln!("the bridge's chip was not checked: its ESP_INFO did not decode, and the bundle has one bridge image");
    }
    let now = before.map_or("commit unknown".to_string(), |i| {
        format!("commit {:08x}", i.commit)
    });
    println!(
        "image {} for the bridge ({now}), from the bundle built from commit {}",
        img.name, b.manifest.commit
    );
    ask(
        yes,
        &format!("Update the bridge to {}?", img.name),
        "not updated",
    )?;
    send_bridge_image(c, link, spec, file, version, before, no_confirm)
}

/// The question a bundle's install asks (`specs/firmware-bundle.md` section 4): none with `yes`; a
/// refusal without a terminal; otherwise `prompt`, refused with `no` unless answered yes.
fn ask(yes: bool, prompt: &str, no: &str) -> Result<(), String> {
    if yes {
        return Ok(());
    }
    if !terminal() {
        return Err("give --yes to install without a terminal".into());
    }
    if confirm(prompt) {
        Ok(())
    } else {
        Err(no.into())
    }
}

/// Send `image` (an app image of `version`) to the bridge `c` reaches, reconnect after the reset,
/// and confirm it unless `no_confirm`. It depends on no reply whose layout can change: the attach,
/// `ESP_UPDATE_BEGIN`'s answer and `ESP_UPDATE_CONFIRM`'s `RESULT`; `ESP_INFO` (`before`, and
/// after) is printed, and the new slot checked against it, only where it decodes
/// (`specs/hoverboard-bridge.md`, "Installing and updating the bridge").
fn send_bridge_image(
    mut c: EspClient,
    link: &Link,
    spec: &str,
    image: &[u8],
    version: [u8; 3],
    before: Option<espsvc::EspInfo>,
    no_confirm: bool,
) -> Result<(), String> {
    let e = |e: ClientError| e.to_string();
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
    c.update_esp(image, version, |done| {
        eprint!("\r{done}/{total} ({:.1} s)   ", t0.elapsed().as_secs_f32())
    })
    .map_err(e)?;
    eprintln!();
    eprintln!(
        "written and checked in {:.1} s; the bridge is resetting into it",
        t0.elapsed().as_secs_f32()
    );
    drop(c);

    let mut c = reconnect(link, spec)?;
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
    let src = guest::attach(&mut wire, 1, wait)?.guest;
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

/// `bridge install`'s image: from a bundle, its `bridge-install` image for the ESP32-C3, the only
/// chip the install writes (`specs/firmware-bundle.md` section 4); otherwise the file, checked as a
/// merged install image.
fn install_image(path: &str) -> Result<Vec<u8>, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if !bundle::is_bundle(&bytes) {
        install::check_image(&bytes).map_err(|e| format!("{path}: {e}"))?;
        return Ok(bytes);
    }
    let b = open_checked_bundle(path, &bytes)?;
    let img = b
        .select_bridge(ImageKind::BridgeInstall, Some(espsvc::CHIP_ESP32C3))
        .map_err(|e| e.to_string())?;
    let (file, _) = b.bridge_file(img)?;
    println!(
        "image {} from the bundle built from commit {}",
        img.name, b.manifest.commit
    );
    Ok(file.to_vec())
}

/// `bridge install` (`specs/bridge-onboarding.md` section 3.2): check the image and the files,
/// choose the port, ask before erasing a bridge that answers there, write the image, wait for the
/// first boot, apply the files, and when they set an SSID, reset the bridge and wait for it to join.
fn bridge_install(
    given: Given,
    image_path: &str,
    files: &[String],
    yes: bool,
) -> Result<(), String> {
    let image = install_image(image_path)?;
    let (port, found) = install::choose_port(given, &install::host_ports())?;
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
        class: CLASS_BRIDGE,
    };
    let (reads, unknown) = config::read_board(&mut b, &config::plan_keys(&files))?;
    let plan = config::plan(&files, &reads, &unknown, false).map_err(joined)?;
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

/// Whether stdin is a terminal, which a question needs.
fn terminal() -> bool {
    std::io::stdin().is_terminal()
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

/// Attach to the node on the link as a guest.
fn attach(wire: &mut impl Wire) -> Result<guest::Attached, String> {
    let a = guest::attach(wire, HELLO_TRIES, HELLO_WAIT)
        .ok_or("no NODE_HELLO answer from the node on this link")?;
    eprintln!("attached as guest {:#04x}", a.guest);
    Ok(a)
}

/// Walk the machine, then ask every node's `IDENTITY` and read its name.
fn walk(wire: &mut impl Wire) -> Result<Walked, String> {
    let started = nodes::start(wire)?;
    walk_on(wire, started)
}

/// [`walk`] on from the start `nodes::start` made.
fn walk_on(wire: &mut impl Wire, started: Controller) -> Result<Walked, String> {
    let w = nodes::finish(wire, started)?;
    eprintln!("walked {} nodes as guest {:#04x}", w.found.len(), w.src);
    Ok(w)
}

/// The line `no answer from <addr>` ends with.
fn no_answer(addr: u8) -> String {
    format!("no answer from {addr:#04x}; `hoverboard-cli machine nodes` lists the nodes this link reaches")
}

/// Reach the node `spec` names for a `board` or `bridge` command (section 6.3, "A target"): an
/// address by attaching and asking its `IDENTITY` and `device.name`, with no walk (a frame to an
/// address the forwarders have not learned is flooded until the reply teaches the route); a name or
/// a `uid` prefix by walking and resolving it. A node of the other class is refused.
fn reach(wire: &mut impl Wire, spec: &str, noun: Noun) -> Result<(u8, Found), String> {
    let (src, f) = match target::parse(spec)? {
        Target::Address(addr) => {
            let src = attach(wire)?.guest;
            (src, found_at(wire, src, addr)?)
        }
        Target::Lookup(_) => {
            let w = walk(wire)?;
            let a = target::resolve(spec, &w.found)?;
            (w.src, target::at(&w.found, a)?.clone())
        }
    };
    check_class(&f, noun)?;
    Ok((src, f))
}

/// The node at `addr` as its `IDENTITY` and `device.name` give it, asked from `src`.
fn found_at(wire: &mut impl Wire, src: u8, addr: u8) -> Result<Found, String> {
    let identity = nodes::identify(wire, src, addr).map_err(|_| no_answer(addr))?;
    let name = nodes::read(wire, src, addr, store::DEVICE_NAME.key())
        .ok()
        .and_then(|r| match r.value() {
            Some(store::Value::Str(s)) => Some(String::from_utf8_lossy(s).into_owned()),
            _ => None,
        });
    Ok(Found {
        addr,
        identity: Some(identity),
        name,
    })
}

/// The client for a bridge command to the bridge `spec` names. An address needs no walk: the
/// tool attaches and sends to `0x00` when the address is the attached node's own, else to the
/// address. With `check`, the node has to answer `IDENTIFY` as a bridge; `bridge update` skips
/// that, so it reaches a bridge on an image older than `IDENTIFY` (`specs/machine-settings.md`
/// section 8, step 4), and relies on the update's own replies.
fn bridge_client(link: &Link, spec: &str, check: bool) -> Result<EspClient, String> {
    let mut wire = wire_on(link)?;
    let (src, dst) = match target::parse(spec)? {
        Target::Address(a) => {
            let at = attach(&mut wire)?;
            let dst = if at.node == Some(a) { NO_ADDRESS } else { a };
            if check {
                let id = nodes::identify(&mut wire, at.guest, dst).map_err(|_| no_answer(a))?;
                let f = Found {
                    addr: a,
                    identity: Some(id),
                    name: None,
                };
                check_class(&f, Noun::Bridge)?;
            }
            (at.guest, dst)
        }
        Target::Lookup(_) => {
            let w = walk(&mut wire)?;
            let a = target::resolve(spec, &w.found)?;
            check_class(target::at(&w.found, a)?, Noun::Bridge)?;
            let dst = if w.topology.gateway == Some(a) {
                NO_ADDRESS
            } else {
                a
            };
            (w.src, dst)
        }
    };
    Ok(Client::addressed(wire, src, dst))
}

/// Every problem of a refused plan, one per line.
fn joined(p: Vec<names::Problem>) -> String {
    let msgs: Vec<String> = p.into_iter().map(|p| p.msg).collect();
    msgs.join("\nerror: ")
}

/// Print the problems `force` lets through as warnings, and stop on the others.
fn check_problems(problems: &[names::Problem], force: bool) -> Result<(), String> {
    for w in config::warnings(problems, force) {
        eprintln!("{w}");
    }
    config::stop_on(problems, force)
}

/// The board files of `board` and `bridge` commands, read and checked before any link is opened;
/// the problems `--force` lets through are printed as warnings. A machine file is refused.
fn load_files(paths: &[String], force: bool) -> Result<config::Files, String> {
    let texts = paths
        .iter()
        .map(|p| {
            std::fs::read_to_string(p)
                .map(|t| (p.to_string(), t))
                .map_err(|e| format!("{p}: {e}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let files = config::read_files(&texts);
    check_problems(&files.problems, force)?;
    Ok(files)
}

/// The machine file of the `machine` commands, read and checked before any link is opened. A
/// board file is refused.
fn load_machine(path: &str, force: bool) -> Result<MachineFile, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let file = machine::read_machine_file(path, &text)?;
    check_problems(&file.problems(), force)?;
    Ok(file)
}

/// Whether `key` is a machine field.
fn is_machine(key: store::Key) -> bool {
    names::row_of(key).and_then(|r| r.scope()) == Some(store::Scope::Machine)
}

/// The line a `board` command ends with when it changed a machine field on one board.
fn machine_only(name: &str) -> String {
    format!("machine fields changed on {name} only: the boards now differ (\"machine diff\")")
}

/// The line after a write that changed a pin.
fn pin_hint(plan: &config::Plan) {
    if config::changed_keys(plan)
        .any(|k| names::row_of(k).is_some_and(|r| r.kind == store::Kind::Pin))
    {
        println!("{}", hoverboard_cli::status::PIN_HINT);
    }
}

/// `board`/`bridge` `diff`, `apply` and `replace` (`specs/cli-config.md` section 4): check the
/// files, read the node, plan, and unless a diff, write. Returns whether anything is (or would be)
/// written.
fn files_command(
    link: &Link,
    noun: Noun,
    spec: &str,
    paths: &[String],
    mode: Mode,
    force: bool,
    yes: bool,
) -> Result<bool, String> {
    let files = load_files(paths, force)?;
    let mut wire = wire_on(link)?;
    let (src, f) = reach(&mut wire, spec, noun)?;
    let mut b = Board {
        wire: &mut wire,
        src,
        dst: f.addr,
        name: node_name(&f),
        class: class_of(noun),
    };
    let (reads, unknown) = config::read_board(&mut b, &config::plan_keys(&files))?;
    let replace = matches!(mode, Mode::Replace | Mode::Diff { replace: true });
    let plan = config::plan(&files, &reads, &unknown, replace).map_err(joined)?;
    let none = BTreeMap::new();
    if noun == Noun::Board && !matches!(mode, Mode::Diff { .. }) {
        check_board(&mut *b.wire, f.addr, &files, &reads, &plan)?;
        if plan_changes_leader(&plan) {
            let w = walk(&mut *b.wire)?;
            check_leader_on(&mut *b.wire, &w, &[(f.addr, &plan)])?;
        }
    }
    if let Mode::Diff { .. } = mode {
        for l in config::report(&b.name, &plan, &reads, &none) {
            println!("{l}");
        }
        return Ok(plan.changes());
    }
    let n = plan.unstores();
    if replace && n > 0 && !yes {
        if !terminal() {
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
    pin_hint(&plan);
    if config::changed_keys(&plan).any(is_machine) {
        println!("{}", machine_only(&b.name));
    }
    Ok(plan.changes())
}

/// A node's boot report entries (`STATUS_READ`), or `None` when it does not answer.
fn board_status(wire: &mut impl Wire, src: u8, addr: u8) -> Option<Vec<u8>> {
    nodes::status_read(wire, src, addr).ok()
}

/// The motor controllers among `found`.
fn motors(found: &[Found]) -> Vec<&Found> {
    found
        .iter()
        .filter(|f| f.class() == Some(CLASS_MOTOR))
        .collect()
}

/// The boards `targets` name, among a walk's; every motor controller when none is given.
fn boards_of<'a>(w: &'a Walked, targets: &[String]) -> Result<Vec<&'a Found>, String> {
    let out: Vec<&Found> = if targets.is_empty() {
        motors(&w.found)
    } else {
        targets
            .iter()
            .map(|t| target::at(&w.found, target::resolve(t, &w.found)?))
            .collect::<Result<_, String>>()?
    };
    if out.is_empty() {
        return Err("the walk found no motor controller".into());
    }
    Ok(out)
}

/// The boards `machine watch` and `drive` act on: with only addresses given, no walk.
fn guest_boards(wire: &mut impl Wire, targets: &[String]) -> Result<(u8, Vec<u8>), String> {
    let addrs: Option<Vec<u8>> = targets
        .iter()
        .map(|t| match target::parse(t) {
            Ok(Target::Address(a)) => Some(a),
            _ => None,
        })
        .collect();
    match addrs {
        Some(a) if !a.is_empty() => Ok((attach(wire)?.guest, a)),
        _ => {
            let w = walk(wire)?;
            let b = boards_of(&w, targets)?.iter().map(|f| f.addr).collect();
            Ok((w.src, b))
        }
    }
}

/// The wheels `machine drive`'s targets name (`specs/twelve-fet.md` section 6.3): each board's
/// motor count read from it first, then a bare board as all its motors and `<board>:<motor>` as
/// one; every wheel of every motor controller when none is given.
fn drive_wheels(wire: &mut impl Wire, targets: &[String]) -> Result<(u8, Vec<Wheel>), String> {
    let wanted = targets
        .iter()
        .map(|t| WheelTarget::parse(t))
        .collect::<Result<Vec<_>, _>>()?;
    let boards: Vec<String> = wanted.iter().map(|w| w.board.to_string()).collect();
    let (src, addrs) = guest_boards(wire, &boards)?;
    // `guest_boards` answers one address per target, in order, or every motor controller.
    let picked: Vec<(u8, Option<u8>)> = if wanted.is_empty() {
        addrs.iter().map(|&a| (a, None)).collect()
    } else {
        addrs
            .iter()
            .zip(&wanted)
            .map(|(&a, w)| (a, w.motor))
            .collect()
    };
    let mut counts = BTreeMap::new();
    for &(a, _) in &picked {
        if let std::collections::btree_map::Entry::Vacant(e) = counts.entry(a) {
            e.insert(guest::motor_count(wire, src, a).map_err(|_| no_answer(a))?);
        }
    }
    let wheels = guest::wheels(&picked, |a| counts[&a])?;
    let listed: Vec<String> = wheels
        .iter()
        .map(|w| format!("{:#04x}:{}", w.node, w.motor))
        .collect();
    eprintln!("driving {}", listed.join(" "));
    Ok((src, wheels))
}

/// The bridge's TCP console port (`specs/hoverboard-bridge.md`, "The Wi-Fi console").
const CONSOLE_PORT: u16 = 5333;
/// How long `machine update` waits for a flashed board's boot report.
const STATUS_WAIT: Duration = Duration::from_secs(10);
/// How long `machine update`'s report walks again for a node the plan walked.
const REPORT_WAIT: Duration = Duration::from_secs(20);

/// Whether the node that answers an attach on `link` holds `addr`.
fn reaches(link: &Link, addr: u8) -> Result<(), String> {
    let mut wire = wire_on(link)?;
    let at = attach(&mut wire)?;
    match at.node {
        Some(a) if a == addr => Ok(()),
        Some(a) => Err(format!("the node there holds {a:#04x}, not {addr:#04x}")),
        None => Err(format!("the node there holds no address, not {addr:#04x}")),
    }
}

/// The direct link to the bridge `name` at `addr` (`specs/firmware-bundle.md` section 4.3, step 2):
/// its TCP console at `<name>.local:5333`, else, with the `ble` feature, BLE by that name.
fn bridge_link(name: &str, addr: u8) -> Result<Link, String> {
    let tcp = Link::Tcp(format!("{name}.local:{CONSOLE_PORT}"));
    let tried = match reaches(&tcp, addr) {
        Ok(()) => return Ok(tcp),
        Err(e) => format!("TCP {name}.local:{CONSOLE_PORT}: {e}"),
    };
    #[cfg(feature = "ble")]
    {
        let ble = Link::Ble(Some(name.to_string()));
        match reaches(&ble, addr) {
            Ok(()) => Ok(ble),
            Err(e) => Err(format!("{tried}; BLE {name}: {e}")),
        }
    }
    #[cfg(not(feature = "ble"))]
    Err(format!("{tried}; no BLE in this build"))
}

/// How `machine update` opens a bridge's own link from its `device.name` and address:
/// [`bridge_link`], or the simulated machine's ports in the tests.
type ByName<'a> = &'a dyn Fn(&str, u8) -> Result<Link, String>;

/// `machine update`'s device steps on the real links.
struct Machine<'a> {
    given: &'a Link,
    by_name: ByName<'a>,
    bundle: &'a Bundle,
    plan: &'a fleet::Plan,
}

impl fleet::Steps for Machine<'_> {
    type Link = Link;

    fn open(&mut self, pair: &fleet::Pair) -> Result<Link, String> {
        let b = &pair.bridge;
        println!("== {}", pair.label());
        if pair.given {
            reaches(self.given, b.addr)?;
            return Ok(self.given.clone());
        }
        let link = (self.by_name)(&b.name, b.addr)?;
        eprintln!("{}: reached on its own link", b.name);
        Ok(link)
    }

    fn bridge(&mut self, link: &Link, pair: &fleet::Pair) -> Result<(), String> {
        let spec = format!("{:#04x}", pair.bridge.addr);
        let mut c = bridge_client(link, &spec, false)?;
        let before = info_or_note(&mut c)?;
        let img = self
            .bundle
            .manifest
            .images
            .iter()
            .find(|i| i.name == pair.bridge.image)
            .expect("the plan's image");
        let (file, version) = self.bundle.bridge_file(img)?;
        let version = version.expect("an app image has a version");
        send_bridge_image(c, link, &spec, file, version, before, false)
    }

    fn rejoin(&mut self, pair: &fleet::Pair) -> Result<Link, String> {
        let b = &pair.bridge;
        let t0 = Instant::now();
        loop {
            let link = if pair.given {
                reaches(self.given, b.addr).map(|()| self.given.clone())
            } else {
                (self.by_name)(&b.name, b.addr)
            };
            match link {
                Ok(l) => return Ok(l),
                Err(e) if t0.elapsed() >= UPDATE_REBOOT => {
                    return Err(format!(
                        "{} did not answer within {} s of its update ({e})",
                        b.name,
                        UPDATE_REBOOT.as_secs()
                    ))
                }
                Err(_) => std::thread::sleep(RECONNECT_GAP),
            }
        }
    }

    fn board(&mut self, link: &Link, board: &fleet::PlanBoard) -> Result<(), String> {
        let uid = nodes::uid_hex(&board.uid);
        let (mut c, r) = swd_route(link, &uid, Purpose::Other)?;
        if !matches!(r.via, Via::Uid) {
            return Err(format!(
                "{} is not the SWD target of the bridge on this link",
                board.name
            ));
        }
        let id = c.target_id().map_err(|e| e.to_string())?;
        let (img, chip) = self.bundle.select(&id, None).map_err(|e| e.to_string())?;
        if img.name != board.image || chip != board.chip {
            return Err(format!(
                "{}: TARGET_ID now matches {} ({chip}), not the plan's {} ({})",
                board.name, img.name, board.image, board.chip
            ));
        }
        println!("{}", bundle::choice_line(&self.bundle.manifest, img, chip));
        flash_board(&mut c, &self.plan.bins[&board.image], refusal)
    }

    fn status(&mut self, link: &Link, board: &fleet::PlanBoard) -> Result<(), String> {
        let t0 = Instant::now();
        loop {
            let read = wire_on(link).and_then(|mut wire| {
                let src = attach(&mut wire)?.guest;
                board_status(&mut wire, src, board.addr)
                    .ok_or_else(|| format!("{}: no STATUS answer", board.name))
            });
            match read {
                Ok(v) => {
                    let lines = hoverboard_cli::status::lines(&board.name, &v, |_| None);
                    for l in &lines {
                        println!("{l}");
                    }
                    return if v.is_empty() {
                        Ok(())
                    } else {
                        Err(lines.join("; "))
                    };
                }
                Err(e) if t0.elapsed() >= STATUS_WAIT => return Err(e),
                Err(_) => std::thread::sleep(RECONNECT_GAP),
            }
        }
    }
}

/// `machine update` (`specs/firmware-bundle.md` section 4.3): open and check the bundle, walk and
/// plan over the given link, ask once, take each pair over its bridge's own link with the given
/// link's pair last, stop at the first failure, and report over the given link. Exits 1 unless
/// every pair is done.
fn machine_update(link: &Link, path: &str, yes: bool) -> Result<u8, String> {
    machine_update_with(link, path, yes, &bridge_link)
}

/// [`machine_update`], with each bridge's own link opened by `by_name`.
fn machine_update_with(link: &Link, path: &str, yes: bool, by_name: ByName) -> Result<u8, String> {
    let b = read_bundle(path)?;
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let src = w.src;
    let gateway = w.topology.gateway;
    let dst = |a: u8| if gateway == Some(a) { NO_ADDRESS } else { a };
    let plan = {
        let wire = std::cell::RefCell::new(&mut wire);
        fleet::plan(
            &w,
            &b,
            |bridge| {
                let mut w = wire.borrow_mut();
                Client::addressed(&mut **w, src, dst(bridge))
                    .info_if_decodes()
                    .map_err(|e| e.to_string())
            },
            |bridge, _board| {
                let mut w = wire.borrow_mut();
                Client::addressed(&mut **w, src, dst(bridge))
                    .target_id()
                    .map_err(|e| e.to_string())
            },
        )?
    };
    drop(wire);
    for l in fleet::plan_lines(&plan) {
        println!("{l}");
    }
    ask(
        yes,
        "Update every bridge and board above?",
        "nothing updated",
    )?;
    let mut steps = Machine {
        given: link,
        by_name,
        bundle: &b,
        plan: &plan,
    };
    let outcomes = fleet::run(&plan, &mut steps);
    println!("== outcome");
    for l in fleet::outcome_lines(&plan, &outcomes) {
        println!("{l}");
    }
    println!("== report");
    if let Err(e) = machine_report(link, &w.found, &plan, &outcomes) {
        println!("report: {e}");
    }
    Ok(u8::from(!fleet::all_done(&outcomes)))
}

/// `machine update`'s report over the given link (section 4.3, step 5): each bridge's commit from
/// `ESP_INFO`, each board's image, and whether the boards' machine digests agree.
fn machine_report(
    link: &Link,
    planned: &[Found],
    plan: &fleet::Plan,
    outcomes: &[fleet::Outcome],
) -> Result<(), String> {
    // A board that has just booted may not answer the first walk, nor the boards beyond it: walk
    // again until every node the plan walked is found, or REPORT_WAIT has passed.
    let t0 = Instant::now();
    let (mut wire, w) = loop {
        let mut wire = wire_on(link)?;
        let w = walk(&mut wire)?;
        let all = planned
            .iter()
            .all(|p| w.found.iter().any(|f| f.addr == p.addr));
        if all || t0.elapsed() >= REPORT_WAIT {
            break (wire, w);
        }
        std::thread::sleep(RECONNECT_GAP);
    };
    let missing: Vec<String> = planned
        .iter()
        .filter(|p| !w.found.iter().any(|f| f.addr == p.addr))
        .map(node_name)
        .collect();
    if !missing.is_empty() {
        println!(
            "not found by the walk within {} s: {}",
            REPORT_WAIT.as_secs(),
            missing.join(", ")
        );
    }
    let src = w.src;
    let gateway = w.topology.gateway;
    for f in w.found.iter().filter(|f| f.class() == Some(CLASS_BRIDGE)) {
        let dst = if gateway == Some(f.addr) {
            NO_ADDRESS
        } else {
            f.addr
        };
        let commit = match Client::addressed(&mut wire, src, dst).info_if_decodes() {
            Ok(Some(i)) => format!("commit {:08x}", i.commit),
            Ok(None) => "ESP_INFO has another layout".into(),
            Err(e) => format!("no ESP_INFO ({e})"),
        };
        println!("bridge {}: {commit}", node_name(f));
    }
    for (pair, o) in plan.pairs.iter().zip(outcomes) {
        if let Some(d) = &pair.board {
            let state = if *o == fleet::Outcome::Done {
                format!("runs {}", d.image)
            } else {
                "not updated".into()
            };
            println!("board  {}: {state}", d.name);
        }
    }
    let heard = listen(&mut wire);
    let digests: Vec<(String, Option<u16>)> = motors(&w.found)
        .into_iter()
        .map(|f| (node_name(f), heard.last(f.addr).map(|s| s.machine)))
        .collect();
    println!("{}", machine::digest_line(&digests));
    Ok(())
}

/// Listen for `CYCLIC_STATE` for [`LISTEN`].
fn listen(wire: &mut impl Wire) -> Tally {
    let mut tally = Tally::new();
    let t0 = Instant::now();
    while t0.elapsed() < LISTEN {
        if let Some(pkt) = wire.recv(LISTEN.saturating_sub(t0.elapsed()).min(POLL)) {
            tally.feed(&pkt, t0.elapsed());
        }
    }
    tally
}

/// The client to the bridge wired to the board `spec` names, for an SWD command for `purpose`, and
/// the route it took ([`route::swd`]): by the `uid` the attached bridge reads over SWD, by the
/// attached bridge's mailbox for an address, or by the walk.
fn swd_route(link: &Link, spec: &str, purpose: Purpose) -> Result<(EspClient, Route), String> {
    let mut wire = wire_on(link)?;
    let r = route::swd(&mut wire, spec, purpose)?;
    match &r.via {
        Via::Uid => eprintln!(
            "{spec} is the SWD target of the bridge this link reaches, guest {:#04x}",
            r.src
        ),
        Via::Mailbox { board, bridge, .. } => eprintln!(
            "{board:#04x} is on the mailbox of bridge {bridge:#04x}, guest {:#04x}",
            r.src
        ),
        Via::Walk { walked, .. } => eprintln!(
            "walked {} nodes as guest {:#04x}",
            walked.found.len(),
            walked.src
        ),
    }
    Ok((Client::addressed(wire, r.src, r.dst), r))
}

/// The chip of the board at `addr`, as `TARGET_ID` through the bridge wired to it reports it, over
/// the open `wire` (`specs/config-read-once.md` section 4: the pin checks are made against it).
fn chip_on(wire: &mut impl Wire, addr: u8) -> Result<hal::pincap::Chip, String> {
    let r = route::swd(wire, &format!("{addr:#04x}"), Purpose::Other)?;
    let id = Client::addressed(&mut *wire, r.src, r.dst)
        .target_id()
        .map_err(|e| e.to_string())?;
    hoverboard_cli::checks::chip_of(id.family, id.flash_kib).ok_or_else(|| {
        format!(
            "{addr:#04x}: TARGET_ID reports {} with {} KiB of flash, a part no image is built for",
            family_name(id.family),
            id.flash_kib
        )
    })
}

/// The board checks of a plan (`specs/config-read-once.md` section 4), with the chip read when the
/// plan changes a key the pin checks read or the files carry a pin, whose `af` form is checked
/// whether or not it changes.
fn check_board(
    wire: &mut impl Wire,
    addr: u8,
    files: &config::Files,
    reads: &config::Reads,
    plan: &config::Plan,
) -> Result<(), String> {
    let chip = if plan
        .steps
        .iter()
        .any(|s| hoverboard_cli::checks::needs_chip(s.key))
        || hoverboard_cli::checks::first_pin(&files.entries).is_some()
    {
        Some(chip_on(wire, addr)?)
    } else {
        None
    };
    config::check_plan(chip, files, reads, plan)
}

/// The leader check across the machine (`specs/rigid-leader.md` section 6.1): every motor
/// controller the walk `w` found, its `machine.mode`, `machine.frame` and `control.leader` read as
/// stored and, for a board `changed` names, with its plan's writes of those keys applied. A board
/// whose three keys cannot be read refuses the write; `--force` overrides nothing here.
fn check_leader_on(
    wire: &mut impl Wire,
    w: &Walked,
    changed: &[(u8, &config::Plan)],
) -> Result<(), String> {
    let mut states = Vec::new();
    for f in motors(&w.found) {
        let unread = || {
            format!(
                "{}: {:#04x} did not answer; the leader check needs every board",
                names::path_of(store::CONTROL_LEADER.key()),
                f.addr
            )
        };
        let mut b = Board {
            wire: &mut *wire,
            src: w.src,
            dst: f.addr,
            name: node_name(f),
            class: CLASS_MOTOR,
        };
        let (reads, unknown) =
            config::read_board(&mut b, &checks::LEADER_KEYS).map_err(|_| unread())?;
        if !unknown.is_empty() {
            return Err(unread());
        }
        let mut plan = config::Plan::default();
        if let Some((_, p)) = changed.iter().find(|(a, _)| *a == f.addr) {
            plan.steps = p
                .steps
                .iter()
                .filter(|s| checks::changes_leader(s.key))
                .cloned()
                .collect();
        }
        states.push((f.addr, config::state_after(&reads, &plan)));
    }
    let boards: Vec<(u8, &checks::State)> = states.iter().map(|(a, s)| (*a, s)).collect();
    match checks::check_leader(&boards) {
        Some(r) => Err(r.msg),
        None => Ok(()),
    }
}

/// Whether `plan` writes a key the leader check reads.
fn plan_changes_leader(plan: &config::Plan) -> bool {
    plan.steps.iter().any(|s| checks::changes_leader(s.key))
}

/// Whether a node's chip routes by the AFIO remap (the GD32F10x), from its `IDENTITY` family: its
/// pins carry no `af` in a file (`specs/config-read-once.md` section 3).
fn remaps(f: &Found) -> bool {
    f.identity.is_some_and(|i| i.mcu == net::walk::MCU_F10X)
}

/// `board erase` (`specs/hoverboard-bridge.md`, "Erasing the store"): print what the erase loses
/// that a dump leaves out, ask, send `STORE_ERASE` through the board's own bridge, wait for the
/// board's mailbox, and give the board its old address back out the bridge's mailbox port.
fn board_erase(link: &Link, spec: &str, yes: bool) -> Result<(), String> {
    let (mut c, r) = swd_route(link, spec, Purpose::Erase)?;
    let src = r.src;
    let (board, bridge, mailbox) = match r.via {
        Via::Mailbox {
            board,
            bridge,
            port,
        } => {
            let f = found_at(c.wire(), src, board)?;
            check_class(&f, Noun::Board)?;
            (f, bridge, port)
        }
        Via::Walk {
            walked,
            board,
            bridge,
            port,
        } => (target::at(&walked.found, board)?.clone(), bridge, port),
        Via::Uid => unreachable!("board erase never routes by uid"),
    };
    let name = node_name(&board);
    let mut on = Board {
        wire: c.wire(),
        src,
        dst: board.addr,
        name: name.clone(),
        class: CLASS_MOTOR,
    };
    let (reads, _) = config::read_board(&mut on, &config::board_owned_keys())?;
    for l in erase::lines(&name, &reads) {
        println!("{l}");
    }
    match erase::guard(&name, yes, terminal()) {
        Guard::Proceed => {}
        Guard::Refuse(why) => return Err(why),
        Guard::Ask(q) => {
            if !confirm(&q) {
                return Err("nothing erased".into());
            }
        }
    }
    let e = |e: ClientError| e.to_string();
    c.store_erase().map_err(|x| refusal(&mut c, x))?;
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

/// A board's SWD command through the bridge wired to it.
fn board_swd(link: &Link, spec: &str, op: &Swd) -> Result<(), String> {
    // Read the file before the link, so a refused image or a damaged bundle costs no connection.
    let file = match op {
        Swd::Flash { file } => Some(FlashFile::Bin(read_elf(file)?)),
        Swd::Update { bundle, .. } => Some(FlashFile::Bundle(read_bundle(bundle)?)),
        _ => None,
    };
    let purpose = match op {
        Swd::Ctrl(Ctrl::Halt) => Purpose::Halt,
        Swd::TargetId | Swd::ClearProtection | Swd::Flash { .. } | Swd::Update { .. } => {
            Purpose::Identify
        }
        _ => Purpose::Other,
    };
    let (mut c, _) = swd_route(link, spec, purpose)?;
    let e = |e: ClientError| e.to_string();
    let no_target = |e: ClientError| route::identify_refusal(&e);
    match op {
        Swd::TargetId => print_id(&c.target_id().map_err(no_target)?),
        Swd::ClearProtection => {
            let id = c.target_id().map_err(no_target)?;
            print_id(&id);
            if id.family == espsvc::FAMILY_UNSUPPORTED {
                return Err(format!(
                    "REFUSED: {} (cpuid {:#010x}, mcu id {:#010x})",
                    Status::UnsupportedPart.describe(),
                    id.cpuid,
                    id.mcu_id
                ));
            }
            if !confirm(
                "Clearing protection MASS-ERASES the chip: the stock firmware is lost unless it \
                 was backed up with gdf130backup. Continue?",
            ) {
                return Err("not cleared".into());
            }
            c.clear_protection().map_err(|x| refusal(&mut c, x))?;
            if id.family == espsvc::FAMILY_MM32SPIN05 {
                // The bridge's clear ends in a system reset, which reloads the MM32's option
                // bytes; it has no USER byte to compare.
                let after = c.target_id().map_err(e)?;
                print_id(&after);
                if after.rdp != 0xA5 {
                    return Err("the clear did not take: RDPRT is still set".into());
                }
                println!("protection off");
                return Ok(());
            }
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
        Swd::Ctrl(ctrl) => c.ctrl(*ctrl).map_err(e)?,
        Swd::Read {
            addr,
            len,
            every: None,
        } => hexdump(*addr, &c.read(*addr, *len as usize).map_err(e)?),
        Swd::Read {
            addr,
            len,
            every: Some(every),
        } => {
            let stop = Arc::new(AtomicBool::new(false));
            let flag = stop.clone();
            ctrlc::set_handler(move || flag.store(true, Ordering::Relaxed))
                .map_err(|e| e.to_string())?;
            c.read_every(
                *addr,
                *len as usize,
                *every,
                || stop.load(Ordering::Relaxed),
                |at, data| println!("{}", sample_line(at, data)),
            )
            .map_err(e)?;
        }
        Swd::Write { addr, data } => c.write(*addr, data).map_err(e)?,
        Swd::Flash { .. } | Swd::Update { .. } => {
            let (want, yes) = match op {
                Swd::Update { image, yes, .. } => (image.as_deref(), *yes),
                _ => (None, false),
            };
            let image = match file.expect("read above") {
                FlashFile::Bin(bin) => bin,
                FlashFile::Bundle(b) => {
                    let id = c.target_id().map_err(no_target)?;
                    let (img, chip) = b.select(&id, want).map_err(|e| e.to_string())?;
                    println!("{}", bundle::choice_line(&b.manifest, img, chip));
                    let bin = b.prepare(img).map_err(|e| e.to_string())?;
                    eprintln!("{}: no wfi, its guards and the layout checked", img.name);
                    if !yes {
                        if !terminal() {
                            return Err("give --yes to flash without a terminal".into());
                        }
                        if !confirm(&format!("Flash {}?", img.name)) {
                            return Err("not flashed".into());
                        }
                    }
                    bin
                }
            };
            flash_board(&mut c, &image, identify_refusal)?;
        }
    }
    Ok(())
}

/// `board recover` (`specs/bridge-nrst.md` section 3): the bridge this link reaches holds its board
/// in reset, attaches and halts it at the reset vector; print the uid and chip it read. The board
/// stays halted for the next command.
fn board_recover(link: &Link) -> Result<(), String> {
    let mut wire = wire_on(link)?;
    let r = route::recover(&mut wire)?;
    println!(
        "{}: held in reset, attached, halted at the reset vector",
        r.bridge
    );
    println!(
        "SWD target {} ({} {} KiB)",
        nodes::uid_hex(&r.id.uid),
        family_name(r.id.family).to_lowercase(),
        r.id.flash_kib
    );
    Ok(())
}

/// Flash `image` through the bridge `c` reaches and wait for `booted`, printing the progress and
/// each stage's time.
fn flash_board(
    c: &mut EspClient,
    image: &[u8],
    refuse: fn(&mut EspClient, ClientError) -> String,
) -> Result<(), String> {
    let t0 = Instant::now();
    let mut stage = None;
    let mut times = StageTimes::new();
    c.flash(image, |p: Progress| {
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
    .map_err(|x| refuse(c, x))?;
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
    Ok(())
}

/// What `board flash` or `board update` was given, read before the link is opened.
enum FlashFile {
    /// `board flash`'s ELF, converted.
    Bin(Vec<u8>),
    /// `board update`'s bundle, opened and checked; the image is chosen once `TARGET_ID` is read.
    Bundle(Bundle),
}

/// `board flash`'s file: an ELF, with the `wfi` scan and the layout check only
/// (`specs/firmware-bundle.md` section 4.1, last paragraph). A bundle is refused and pointed at
/// `board update`; anything else is checked as an ELF and refused by its magic.
fn read_elf(path: &str) -> Result<Vec<u8>, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if bundle::file_kind(&bytes) == Ok(FileKind::Bundle) {
        return Err(format!(
            "{path} is a firmware bundle: `board update` installs from a bundle, `board flash` \
             takes an ELF"
        ));
    }
    if bundle::file_kind(&bytes) != Ok(FileKind::Elf) {
        return Err(format!("{path}: not an ELF"));
    }
    elf::check_wfi(&bytes).map_err(|e| e.to_string())?;
    let bin = elf::to_bin(&bytes).map_err(|e| e.to_string())?;
    eprintln!("ELF given directly: no wfi, layout checked; the image guards run on a bundle");
    Ok(bin)
}

/// `board update`'s file: a bundle, opened and SHA-256 checked. An ELF is refused and pointed at
/// `board flash`.
fn read_bundle(path: &str) -> Result<Bundle, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    match bundle::file_kind(&bytes) {
        Ok(FileKind::Bundle) => {}
        Ok(FileKind::Elf) => {
            return Err(format!(
                "{path} is an ELF: `board flash` takes an ELF, `board update` a firmware bundle"
            ))
        }
        Err(e) => return Err(format!("{path}: {e}")),
    }
    open_checked_bundle(path, &bytes)
}

/// Open the bundle `bytes` read from `path`, every file's SHA-256 checked, and say so.
fn open_checked_bundle(path: &str, bytes: &[u8]) -> Result<Bundle, String> {
    let b = Bundle::open(bytes).map_err(|e| format!("{path}: {e}"))?;
    eprintln!(
        "bundle built from commit {} at {}, {} image(s), SHA-256 checked",
        b.manifest.commit,
        b.manifest.built,
        b.manifest.images.len()
    );
    Ok(b)
}

/// Read and open a bundle, refusing a file that is not one.
fn open_bundle(path: &str) -> Result<Bundle, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if bundle::file_kind(&bytes) != Ok(FileKind::Bundle) {
        return Err(format!("{path}: not a firmware bundle (a zip archive)"));
    }
    Bundle::open(&bytes).map_err(|e| e.to_string())
}

/// `firmware info` and `firmware pack`: bundle files, no link.
fn firmware_cmd(given: &Given, command: &Command) -> Result<(), String> {
    if !matches!(given, Given::Nothing) {
        return Err("firmware reads and writes bundle files; it takes no link".into());
    }
    match command {
        Command::FirmwareInfo { bundle } => print!("{}", open_bundle(bundle)?.describe()),
        Command::FirmwarePack {
            out,
            entries,
            dirty,
        } => {
            let entries = entries
                .iter()
                .map(|a| TreeEntry::parse(a))
                .collect::<Result<Vec<_>, _>>()?;
            let z = bundle::pack_tree(&entries, *dirty)?;
            std::fs::write(out, &z).map_err(|e| format!("{out}: {e}"))?;
            let b = Bundle::open(&z).map_err(|e| e.to_string())?;
            println!("wrote {out} ({} B)", z.len());
            print!("{}", b.describe());
        }
        _ => unreachable!("only the firmware commands"),
    }
    Ok(())
}

/// `board status`: the boot report of each board given, or of every motor controller.
fn board_status_cmd(link: &Link, targets: &[String]) -> Result<(), String> {
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let src = w.src;
    for f in boards_of(&w, targets)? {
        let name = node_name(f);
        let Some(value) = board_status(&mut wire, src, f.addr) else {
            println!("{name}: no STATUS answer");
            continue;
        };
        let mut link_pins = |i: u8| {
            let mut pin = |k: store::Key| match nodes::read(&mut wire, src, f.addr, k).ok()?.value()
            {
                Some(store::Value::PinAf([port, pin, _])) if port != 0xFF => {
                    Some(names::pin_text(port, pin))
                }
                _ => None,
            };
            Some((
                pin(store::LINK_TX.at(i).key())?,
                pin(store::LINK_RX.at(i).key())?,
            ))
        };
        let pins: Vec<Option<(String, String)>> = (0..store::LINKS).map(&mut link_pins).collect();
        for l in hoverboard_cli::status::lines(&name, &value, |i| {
            pins.get(i as usize).cloned().flatten()
        }) {
            println!("{l}");
        }
    }
    Ok(())
}

/// `machine nodes`: the walk, with each node's address, class, `uid` and name, and each motor
/// controller marked from the `CYCLIC_STATE` heard over [`LISTEN`] and its boot report.
fn machine_nodes(link: &Link) -> Result<(), String> {
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let entries: Vec<usize> = w
        .found
        .iter()
        .map(|f| match f.class() {
            Some(CLASS_MOTOR) => board_status(&mut wire, w.src, f.addr)
                .map_or(0, |v| v.len() / board::status::ENTRY_LEN),
            _ => 0,
        })
        .collect();
    let heard = listen(&mut wire);
    println!("address  class             uid                       name");
    for (f, n) in w.found.iter().zip(entries) {
        let marks = machine::marks(f.class(), heard.last(f.addr), n);
        let mark = if marks.is_empty() {
            String::new()
        } else {
            format!("  ! {}", marks.join(", "))
        };
        let uid = f.identity.map_or("-".into(), |i| nodes::uid_hex(&i.uid));
        println!(
            "{:#04x}     {:<16}  {uid:<24}  {}{mark}",
            f.addr,
            class_name(f.class()),
            f.name.as_deref().unwrap_or("-")
        );
    }
    Ok(())
}

/// Each motor controller's reads of `keys`, with its name.
fn read_each(
    wire: &mut impl Wire,
    w: &Walked,
    keys: &[store::Key],
) -> Result<Vec<(Found, config::Reads)>, String> {
    let mut out = Vec::new();
    for f in motors(&w.found) {
        let mut b = Board {
            wire: &mut *wire,
            src: w.src,
            dst: f.addr,
            name: node_name(f),
            class: CLASS_MOTOR,
        };
        let (reads, _) = config::read_board(&mut b, keys)?;
        out.push((f.clone(), reads));
    }
    if out.is_empty() {
        return Err("the walk found no motor controller".into());
    }
    Ok(out)
}

/// `machine get`: one line when every copy holds the same value, else one per board.
fn machine_get(link: &Link, path: &str) -> Result<(), String> {
    let key = machine::machine_key(path)?;
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let copies: Vec<(String, nodes::ConfigResp)> = read_each(&mut wire, &w, &[key])?
        .into_iter()
        .filter_map(|(f, mut r)| Some((node_name(&f), r.remove(&key)?)))
        .collect();
    for l in machine::get_lines(key, &copies) {
        println!("{l}");
    }
    Ok(())
}

/// `machine set`: the field to every motor controller.
fn machine_set(link: &Link, path: &str, value: &str, force: bool) -> Result<(), String> {
    let key = machine::machine_key(path)?;
    let checks = names::write_checks(key);
    check_problems(&checks, force)?;
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    // The machine as the whole command leaves it, judged before the first board is written.
    if checks::changes_leader(key) {
        let plan = config::set_plan(key, value)?;
        let each: Vec<(u8, &config::Plan)> =
            motors(&w.found).iter().map(|f| (f.addr, &plan)).collect();
        check_leader_on(&mut wire, &w, &each)?;
    }
    let mut changed = Vec::new();
    for f in motors(&w.found) {
        let mut b = Board {
            wire: &mut wire,
            src: w.src,
            dst: f.addr,
            name: node_name(f),
            class: CLASS_MOTOR,
        };
        let written = config::config_write(&mut b, key, value, force, None)?;
        for l in &written.warnings {
            eprintln!("{l}");
        }
        println!("{}", written.line);
        changed.extend(written.changed);
    }
    if let Some(l) = config::effect_line(changed) {
        println!("{l}");
    }
    Ok(())
}

/// The motor controllers of a walk as matching sees them: each with a `uid` it may match by.
fn candidates(w: &Walked) -> Vec<machine::Board> {
    motors(&w.found)
        .into_iter()
        .map(|f| machine::Board {
            addr: f.addr,
            label: label(f),
            uid: target::usable_uid(f, &w.found),
        })
        .collect()
}

/// `machine dump`: every motor controller's file, filed under its `uid`, with the machine section
/// once; refused when the copies differ, unless `from` names the copy to write.
fn machine_dump(link: &Link, out: &str, from: Option<&str>) -> Result<(), String> {
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let each = read_each(&mut wire, &w, &config::dump_keys())?;
    let mut boards = Vec::new();
    for (f, reads) in &each {
        let uid = target::usable_uid(f, &w.found).ok_or_else(|| {
            format!(
                "{} reports no uid a machine file can file it under",
                label(f)
            )
        })?;
        for l in config::left_out(&node_name(f), reads) {
            eprintln!("{l}");
        }
        boards.push((uid, reads.clone(), remaps(f)));
    }
    let copies: Vec<(String, config::Reads)> = each
        .iter()
        .map(|(f, r)| (node_name(f), r.clone()))
        .collect();
    let differ = machine::differing(&copies);
    let from = match from {
        Some(spec) => {
            let a = target::resolve(spec, &w.found)?;
            each.iter()
                .position(|(f, _)| f.addr == a)
                .ok_or_else(|| format!("{spec} is not a motor controller of this machine"))?
        }
        None if differ.is_empty() => 0,
        None => {
            return Err(format!(
                "the boards' machine settings differ; give --from <board> to write one copy:\n{}",
                differ.join("\n")
            ))
        }
    };
    let text = serde_json::to_string_pretty(&machine::dump_file(&boards, from))
        .map_err(|e| e.to_string())?
        + "\n";
    std::fs::write(out, text).map_err(|e| format!("{out}: {e}"))?;
    eprintln!("{} boards written to {out}", boards.len());
    Ok(())
}

/// `machine diff` with no file: the machine fields whose copies differ, by board. Whether any do.
fn machine_diff_copies(link: &Link) -> Result<bool, String> {
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let copies: Vec<(String, config::Reads)> = read_each(&mut wire, &w, &machine::machine_keys())?
        .into_iter()
        .map(|(f, r)| (node_name(&f), r))
        .collect();
    let differ = machine::differing(&copies);
    if differ.is_empty() {
        println!("the boards' machine settings agree");
    }
    for l in &differ {
        println!("{l}");
    }
    Ok(!differ.is_empty())
}

/// `machine apply`, `replace` and `diff` with a file (section 6.3): match each entry to its board,
/// give every motor controller the machine section and each matched board its entry, report, and
/// after a write say whether the digests agree. The exit code: 1 when an entry was skipped or a
/// diff has changes.
fn machine_files(
    link: &Link,
    path: &str,
    mode: Mode,
    force: bool,
    yes: bool,
    maps: &[(String, String)],
) -> Result<u8, String> {
    let file = load_machine(path, force)?;
    let uids: Vec<machine::Uid> = file.entries.iter().map(|e| e.uid).collect();
    let map_entries = maps
        .iter()
        .map(|(u, _)| machine::map_entry(&uids, u))
        .collect::<Result<Vec<usize>, _>>()?;
    let mut wire = wire_on(link)?;
    let w = walk(&mut wire)?;
    let map_boards = maps
        .iter()
        .map(|(_, t)| target::resolve(t, &w.found))
        .collect::<Result<Vec<u8>, _>>()?;
    let pairs: Vec<(usize, u8)> = map_entries.into_iter().zip(map_boards).collect();
    let boards = candidates(&w);
    let matching = machine::match_entries(&uids, &boards, &pairs)?
        .decide(|q| terminal().then(|| confirm(&format!("{q}?"))))?;
    let replace_all = matches!(mode, Mode::Replace | Mode::Diff { replace: true });
    let diff = matches!(mode, Mode::Diff { .. });

    // Plan every board before writing any.
    let mut plans = Vec::new();
    for f in motors(&w.found) {
        let entry = matching.entry_of(f.addr);
        if entry.is_none() && file.machine.entries.is_empty() {
            println!("{}: no entry in {path}; left alone", node_name(f));
            continue;
        }
        let files = file.board_files(entry);
        let mut b = Board {
            wire: &mut wire,
            src: w.src,
            dst: f.addr,
            name: node_name(f),
            class: CLASS_MOTOR,
        };
        let (reads, unknown) = config::read_board(&mut b, &config::plan_keys(&files))?;
        // A board with no entry gets the machine section merged, never replaced, and its report
        // names only the machine fields it keeps.
        let replace = replace_all && entry.is_some();
        let mut plan = config::plan(&files, &reads, &unknown, replace).map_err(joined)?;
        if !diff {
            check_board(&mut *b.wire, f.addr, &files, &reads, &plan)
                .map_err(|e| format!("{}: {e}", node_name(f)))?;
        }
        if entry.is_none() {
            plan.kept.retain(|(k, _)| is_machine(*k));
        }
        plans.push((f.clone(), reads, plan, entry.is_some()));
    }
    if !diff && plans.iter().any(|p| plan_changes_leader(&p.2)) {
        let each: Vec<(u8, &config::Plan)> = plans.iter().map(|p| (p.0.addr, &p.2)).collect();
        check_leader_on(&mut wire, &w, &each)?;
    }
    let none = BTreeMap::new();
    let mut changes = false;
    let unstores: usize = plans.iter().map(|p| p.2.unstores()).sum();
    if !diff && unstores > 0 && !yes {
        if !terminal() {
            return Err(format!(
                "replace would un-store {unstores} keys; give --yes to do it without a terminal"
            ));
        }
        for (f, reads, plan, _) in &plans {
            for l in config::report(&node_name(f), plan, reads, &none) {
                println!("{l}");
            }
        }
        if !confirm(&format!("Un-store {unstores} keys?")) {
            return Err("nothing written".into());
        }
    }
    for (f, reads, plan, listed) in &plans {
        let name = node_name(f);
        if !listed {
            println!("{name}: no entry in {path}: the machine section only");
        }
        if diff {
            for l in config::report(&name, plan, reads, &none) {
                println!("{l}");
            }
            changes |= plan.changes();
            continue;
        }
        let mut b = Board {
            wire: &mut wire,
            src: w.src,
            dst: f.addr,
            name: name.clone(),
            class: CLASS_MOTOR,
        };
        let after = config::execute(&mut b, plan).map_err(|r| {
            for l in config::written_lines(&name, plan, reads, &r) {
                println!("{l}");
            }
            r.msg
        })?;
        for l in config::report(&name, plan, reads, &after) {
            println!("{l}");
        }
        pin_hint(plan);
    }
    for &i in &matching.skipped {
        println!(
            "{}: no board of this machine reports this uid; skipped",
            nodes::uid_hex(&uids[i])
        );
    }
    if !diff {
        let heard = listen(&mut wire);
        let digests: Vec<(String, Option<u16>)> = motors(&w.found)
            .into_iter()
            .map(|f| (node_name(f), heard.last(f.addr).map(|s| s.machine)))
            .collect();
        println!("{}", machine::digest_line(&digests));
    }
    Ok(u8::from(changes || !matching.skipped.is_empty()))
}

/// Listen for `secs`, or until `stop` is set, counting `CYCLIC_STATE` into `tally` and printing
/// its line once a second, or with `json` every counted frame as a JSON line
/// ([`Tally::frame_json`]); send `drive`'s frames as they fall due. Returns the tally and the
/// run's length.
fn session(
    wire: &mut impl Wire,
    mut tally: Tally,
    secs: u64,
    mut drive: Option<&mut Drive>,
    stop: &AtomicBool,
    json: bool,
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
            let at = t0.elapsed();
            if let Some(src) = tally.feed_src(&pkt, at) {
                if json {
                    if let Some(line) = tally.frame_json(src, at) {
                        println!("{line}");
                    }
                }
            }
        }
        if !json && t0.elapsed() >= next_line && next_line <= end {
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

/// Run the command line. A `diff` exits 2 on every error, usage errors included, so its 1 means
/// only "there are changes" (`specs/cli-config.md` section 4.2).
fn run(args: &[String]) -> Result<u8, String> {
    let (diff, inv) = cli::parse(args);
    match inv.and_then(execute) {
        Err(e) if diff => {
            eprintln!("error: {e}");
            Ok(2)
        }
        r => r,
    }
}

/// Carry out one command.
fn execute(inv: cli::Invocation) -> Result<u8, String> {
    let given = match &inv.link {
        None => Given::Nothing,
        Some(Link::Serial(p)) => Given::Serial(p),
        Some(Link::Tcp(_)) => Given::Tcp,
        Some(Link::Ble(_)) => Given::Device,
    };
    let link = inv.link.clone().unwrap_or(Link::Ble(None));
    let link = &link;
    match inv.command {
        Command::Scan { secs } => {
            if !matches!(link, Link::Ble(_)) {
                return Err("scan is BLE only".into());
            }
            for (name, addr) in scan_names(secs)? {
                println!("{addr}  {name}");
            }
        }
        Command::Fields => {
            let text =
                serde_json::to_string_pretty(&names::table_json()).map_err(|e| e.to_string())?;
            println!("{text}");
        }
        Command::Get { noun, target, path } => {
            let (key, _) = names::parse_path(&path).map_err(|p| p.msg)?;
            let mut wire = wire_on(link)?;
            let (src, f) = reach(&mut wire, &target, noun)?;
            let mut b = Board {
                wire: &mut wire,
                src,
                dst: f.addr,
                name: node_name(&f),
                class: class_of(noun),
            };
            println!("{}", config::config_read(&mut b, key)?);
        }
        Command::Set {
            noun,
            target,
            path,
            value,
            force,
        } => {
            let (key, _) = names::parse_path(&path).map_err(|p| p.msg)?;
            check_problems(&names::write_checks(key), force)?;
            let mut wire = wire_on(link)?;
            let (src, f) = reach(&mut wire, &target, noun)?;
            if noun == Noun::Board && checks::changes_leader(key) {
                let plan = config::set_plan(key, &value)?;
                let w = walk(&mut wire)?;
                check_leader_on(&mut wire, &w, &[(f.addr, &plan)])?;
            }
            let mut b = Board {
                wire: &mut wire,
                src,
                dst: f.addr,
                name: node_name(&f),
                class: class_of(noun),
            };
            let chip = if noun == Noun::Board && hoverboard_cli::checks::needs_chip(key) {
                Some(chip_on(&mut *b.wire, f.addr)?)
            } else {
                None
            };
            let w = config::config_write(&mut b, key, &value, force, chip)?;
            for l in &w.warnings {
                eprintln!("{l}");
            }
            for l in w.lines() {
                println!("{l}");
            }
            if w.changed.is_some_and(is_machine) {
                println!("{}", machine_only(&b.name));
            }
        }
        Command::Dump { noun, target, out } => {
            let mut wire = wire_on(link)?;
            let (src, f) = reach(&mut wire, &target, noun)?;
            let mut b = Board {
                wire: &mut wire,
                src,
                dst: f.addr,
                name: node_name(&f),
                class: class_of(noun),
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
            let text = serde_json::to_string_pretty(&config::to_file(&reads, remaps(&f)))
                .map_err(|e| e.to_string())?
                + "\n";
            match out {
                Some(p) => {
                    std::fs::write(&p, text).map_err(|e| format!("{p}: {e}"))?;
                    eprintln!("{}: {stored} stored keys written to {p}", b.name);
                }
                None => print!("{text}"),
            }
        }
        Command::Files {
            noun,
            target,
            files,
            mode,
            force,
            yes,
        } => {
            let changes = files_command(link, noun, &target, &files, mode, force, yes)?;
            if let Mode::Diff { .. } = mode {
                return Ok(u8::from(changes));
            }
        }
        Command::Compact { target } => {
            let mut wire = wire_on(link)?;
            let (src, f) = reach(&mut wire, &target, Noun::Board)?;
            let status = nodes::compact(&mut wire, src, f.addr)?;
            println!("{}: compact: {}", node_name(&f), nodes::status_name(status));
            config_outcome(status)?;
        }
        Command::Status { targets } => board_status_cmd(link, &targets)?,
        Command::Erase { target, yes } => board_erase(link, &target, yes)?,
        Command::Swd { target, op } => board_swd(link, &target, &op)?,
        Command::Recover => board_recover(link)?,
        Command::Nodes => machine_nodes(link)?,
        Command::MachineGet { path } => machine_get(link, &path)?,
        Command::MachineSet { path, value, force } => machine_set(link, &path, &value, force)?,
        Command::MachineDump { out, from } => machine_dump(link, &out, from.as_deref())?,
        Command::MachineFiles {
            file: None, mode, ..
        } => {
            debug_assert!(matches!(mode, Mode::Diff { replace: false }));
            return machine_diff_copies(link).map(u8::from);
        }
        Command::MachineFiles {
            file: Some(path),
            mode,
            force,
            yes,
            maps,
        } => return machine_files(link, &path, mode, force, yes, &maps),
        Command::Watch {
            secs,
            targets,
            json,
        } => {
            let mut wire = wire_on(link)?;
            let (_, watched) = guest_boards(&mut wire, &targets)?;
            let (tally, elapsed) = session(
                &mut wire,
                Tally::only(&watched),
                secs,
                None,
                &AtomicBool::new(false),
                json,
            );
            if json {
                // stdout carries the frames alone, so a consumer reads it line by line as JSON.
                for l in tally.summary(elapsed) {
                    eprintln!("{l}");
                }
            } else {
                print_summary(&tally, elapsed);
            }
        }
        Command::Drive {
            value,
            secs,
            targets,
        } => {
            let stop = Arc::new(AtomicBool::new(false));
            let flag = stop.clone();
            ctrlc::set_handler(move || flag.store(true, Ordering::Relaxed))
                .map_err(|e| e.to_string())?;
            let mut wire = wire_on(link)?;
            let (src, wheels) = drive_wheels(&mut wire, &targets)?;
            let mut drive = Drive::new(src, wheels, value);
            let (tally, elapsed) = session(
                &mut wire,
                Tally::new(),
                secs,
                Some(&mut drive),
                &stop,
                false,
            );
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
        }
        Command::BridgeInfo { target } => {
            let mut c = bridge_client(link, &target, true)?;
            print_info(&c.info().map_err(|e| e.to_string())?);
        }
        Command::BridgeUpdate {
            target,
            file,
            yes,
            no_confirm,
        } => bridge_update(link, &target, &file, yes, no_confirm)?,
        Command::MachineUpdate { bundle, yes } => return machine_update(link, &bundle, yes),
        Command::BridgeInstall { image, files, yes } => bridge_install(given, &image, &files, yes)?,
        c @ (Command::FirmwareInfo { .. } | Command::FirmwarePack { .. }) => {
            firmware_cmd(&given, &c)?
        }
    }
    Ok(0)
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
