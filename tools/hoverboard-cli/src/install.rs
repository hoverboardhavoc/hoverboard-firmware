//! `install-esp` (`specs/bridge-onboarding.md` section 3): a blank or reinstalled SuperMini made a
//! bridge over its own USB port. The port is chosen here, the image checked, the guard decided, and
//! the image written through the `espflash` library, which resets the C3 into its ROM loader with
//! the USB-Serial-JTAG's control lines, loads Espressif's flasher stub and writes the image at
//! `0x0`. `main.rs` runs the command on these pieces.

use std::time::{Duration, Instant};

use espflash::connection::{Connection, ResetAfterOperation, ResetBeforeOperation};
use espflash::flasher::Flasher;
use espflash::target::{Chip, ProgressCallbacks};
use espsvc::{EspInfo, WifiState, SLOT_NONE};
use serialport::{FlowControl, SerialPortType, UsbPortInfo};

/// The C3's USB-Serial-JTAG, as its USB vendor and product ids.
pub const C3_USB: (u16, u16) = (0x303a, 0x1001);
/// The merged install image's size: the SuperMini's whole 4 MB, so writing it also erases `keys`
/// and `otadata`.
pub const IMAGE_LEN: usize = 4 * 1024 * 1024;
/// The first byte of an ESP image, the bootloader's header at `0x0`.
pub const ESP_IMAGE_MAGIC: u8 = 0xE9;

/// A serial port as the host lists it: its name and, for a USB port, its vendor and product ids.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// `/dev/cu.usbmodem101`, `/dev/ttyACM0`, `COM5`.
    pub name: String,
    /// The USB vendor and product ids, for a USB port.
    pub usb: Option<(u16, u16)>,
}

/// The link the command line names, as `install-esp` sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Given<'a> {
    /// No link: the port is looked for.
    Nothing,
    /// `--serial PORT`.
    Serial(&'a str),
    /// `--tcp`.
    Tcp,
    /// `--device`.
    Device,
}

/// Why the install needs the port itself, for the refusals of the other links.
const NEEDS_USB: &str = "install-esp needs the C3's USB port (--serial PORT): it resets the chip \
                         into its ROM loader with the port's control lines, which";

/// The serial ports the host lists, with their USB ids.
pub fn host_ports() -> Vec<PortInfo> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| PortInfo {
            name: p.port_name,
            usb: match p.port_type {
                SerialPortType::UsbPort(u) => Some((u.vid, u.pid)),
                _ => None,
            },
        })
        .collect()
}

/// The port to install through, from the link `given` and the host's `ports`, and whether it was
/// found rather than named (`specs/bridge-onboarding.md` section 3.1). `--serial` names it;
/// without it, the one port with the C3's USB ids ([`C3_USB`]) is used, and none or several is a
/// refusal listing them. `--tcp` and `--device` are refused: neither carries the control lines.
/// macOS lists each USB serial device twice, as `/dev/cu.*` and `/dev/tty.*`; the `tty` twin of a
/// listed `cu` port is the same device and is not counted.
pub fn choose_port(given: Given, ports: &[PortInfo]) -> Result<(String, bool), String> {
    match given {
        Given::Serial(p) => return Ok((p.to_string(), false)),
        Given::Tcp => return Err(format!("{NEEDS_USB} --tcp does not carry")),
        Given::Device => return Err(format!("{NEEDS_USB} BLE does not carry")),
        Given::Nothing => {}
    }
    let twin = |name: &str| {
        name.strip_prefix("/dev/tty.")
            .is_some_and(|rest| ports.iter().any(|p| p.name == format!("/dev/cu.{rest}")))
    };
    let c3: Vec<&str> = ports
        .iter()
        .filter(|p| p.usb == Some(C3_USB) && !twin(&p.name))
        .map(|p| p.name.as_str())
        .collect();
    let listed = || {
        let all: Vec<String> = ports
            .iter()
            .filter(|p| !twin(&p.name))
            .map(|p| match p.usb {
                Some((v, d)) => format!("{} ({v:04x}:{d:04x})", p.name),
                None => p.name.clone(),
            })
            .collect();
        if all.is_empty() {
            "none".to_string()
        } else {
            all.join(", ")
        }
    };
    let (v, d) = C3_USB;
    match c3[..] {
        [one] => Ok((one.to_string(), true)),
        [] => Err(format!(
            "no ESP32-C3 USB port ({v:04x}:{d:04x}) found; give --serial PORT (ports: {})",
            listed()
        )),
        _ => Err(format!(
            "{} ESP32-C3 USB ports ({v:04x}:{d:04x}): {}; give --serial PORT",
            c3.len(),
            c3.join(", ")
        )),
    }
}

/// Refuse a file that is not a merged install image (`specs/bridge-onboarding.md` section 3.1):
/// [`IMAGE_LEN`] bytes starting with [`ESP_IMAGE_MAGIC`].
pub fn check_image(bytes: &[u8]) -> Result<(), String> {
    if bytes.len() != IMAGE_LEN {
        return Err(format!(
            "not a merged install image: {} B, not {IMAGE_LEN} B (make one with espflash \
             save-image --merge --flash-size 4mb, apps/hoverboard-bridge/README.md)",
            bytes.len()
        ));
    }
    if bytes[0] != ESP_IMAGE_MAGIC {
        return Err(format!(
            "not a merged install image: it starts with {:#04x}, not the ESP image magic {:#04x}",
            bytes[0], ESP_IMAGE_MAGIC
        ));
    }
    Ok(())
}

/// A bridge that answered on the port before the install.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Answered {
    /// Its `device.name`.
    pub name: String,
    /// Its node address, from `ESP_INFO`, when that decoded.
    pub addr: Option<u8>,
    /// Its image's commit, from `ESP_INFO`, when that decoded.
    pub commit: Option<u32>,
}

impl Answered {
    /// The line naming it: `<name>, address 0x03, commit 1a2b3c4d`.
    pub fn describe(&self) -> String {
        let addr = match self.addr {
            Some(0) => "unassigned".to_string(),
            Some(a) => format!("address {a:#04x}"),
            None => "address unknown".to_string(),
        };
        let commit = self.commit.map_or_else(
            || "commit unknown".to_string(),
            |c| format!("commit {c:08x}"),
        );
        format!("{}, {addr}, {commit}", self.name)
    }
}

/// What the guard does before the install (`specs/bridge-onboarding.md` section 3.2, step 1).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Guard {
    /// Install without asking.
    Proceed,
    /// Ask this question, and install only on a yes.
    Ask(String),
    /// Refuse, for this reason.
    Refuse(String),
}

/// The guard's decision: nothing answering (a blank chip, or other firmware) installs; a bridge
/// that answered is asked about, unless `yes`; without a `terminal` on stdin, a bridge that
/// answered is refused unless `yes`, as `replace` refuses.
pub fn guard(answered: Option<&Answered>, yes: bool, terminal: bool) -> Guard {
    match answered {
        None => Guard::Proceed,
        Some(_) if yes => Guard::Proceed,
        Some(a) if !terminal => Guard::Refuse(format!(
            "{} answered on this port: give --yes to install over it without a terminal",
            a.name
        )),
        Some(a) => Guard::Ask(format!(
            "Erase {}'s name, address and Wi-Fi credentials and install?",
            a.name
        )),
    }
}

/// The app slot `ESP_INFO` reports, as `info` prints it.
pub fn slot_name(i: &EspInfo) -> String {
    let slot = match i.slot {
        SLOT_NONE => "neither app slot".to_string(),
        n => format!("ota_{n}"),
    };
    if i.pending {
        format!("{slot}, pending verification")
    } else {
        slot
    }
}

/// The Wi-Fi state `ESP_INFO` reports, as `info` prints it.
pub fn wifi_text(i: &EspInfo) -> String {
    let [a, b, c, d] = i.ip;
    match WifiState::from_u8(i.wifi) {
        Some(WifiState::Up) => format!("up, {a}.{b}.{c}.{d}"),
        Some(WifiState::Joined) => "joined, no address yet".into(),
        Some(WifiState::Down) => "not joined".into(),
        Some(WifiState::NotConfigured) => "not configured".into(),
        None => format!("{}", i.wifi),
    }
}

/// `espflash`'s progress, printed in the form `flash` prints its own: the stage, the blocks done
/// and the time.
struct Progress {
    t0: Instant,
    total: usize,
}

impl ProgressCallbacks for Progress {
    fn init(&mut self, _addr: u32, total: usize) {
        self.total = total;
    }

    fn update(&mut self, current: usize) {
        eprint!(
            "\rProgramming {current}/{} ({:.1} s)   ",
            self.total,
            self.t0.elapsed().as_secs_f32()
        );
    }

    fn verifying(&mut self) {
        eprint!("\nVerifying ({:.1} s)   ", self.t0.elapsed().as_secs_f32());
    }

    fn finish(&mut self, _skipped: bool) {
        eprintln!();
    }
}

/// The C3's USB port opened for `espflash`, with the USB-Serial-JTAG's ids, which choose its reset
/// sequences.
fn connection(port: &str, before: ResetBeforeOperation) -> Result<Connection, String> {
    let serial = serialport::new(port, 115_200)
        .flow_control(FlowControl::None)
        .open_native()
        .map_err(|e| format!("{port}: {e}"))?;
    let (vid, pid) = C3_USB;
    let info = UsbPortInfo {
        vid,
        pid,
        serial_number: None,
        manufacturer: None,
        product: None,
    };
    Ok(Connection::new(
        serial,
        info,
        ResetAfterOperation::HardReset,
        before,
        115_200,
    ))
}

/// Write `image` at `0x0` through `port` (`specs/bridge-onboarding.md` section 3.2, step 2):
/// reset into the ROM loader through the USB-Serial-JTAG, load the flasher stub, write and verify,
/// then hard-reset the chip into the new image. Progress goes to stderr.
pub fn write_image(port: &str, image: &[u8]) -> Result<(), String> {
    let conn = connection(port, ResetBeforeOperation::UsbReset)?;
    let mut flasher = Flasher::connect(conn, true, true, false, Some(Chip::Esp32c3), None)
        .map_err(|e| format!("{port}: no ESP32-C3 ROM loader: {e}"))?;
    let mut progress = Progress {
        t0: Instant::now(),
        total: 0,
    };
    flasher
        .write_bin_to_flash(0, image, &mut progress)
        .map_err(|e| format!("{port}: writing the image: {e}"))
}

/// Hard-reset the chip through `port`'s control lines, as `espflash` does after a write.
pub fn reset(port: &str) -> Result<(), String> {
    connection(port, ResetBeforeOperation::NoReset)?
        .reset()
        .map_err(|e| format!("{port}: reset: {e}"))
}

/// How long the first boot may take to answer an attach (section 3.2, step 3).
pub const BOOT_WAIT: Duration = Duration::from_secs(15);
/// How long the bridge may take to join the network after its reset (section 3.2, step 4).
pub const JOIN_WAIT: Duration = Duration::from_secs(30);

#[cfg(test)]
mod tests {
    use super::*;

    fn port(name: &str, usb: Option<(u16, u16)>) -> PortInfo {
        PortInfo {
            name: name.into(),
            usb,
        }
    }

    #[test]
    fn the_port_is_the_one_c3_named_or_found_and_the_other_links_are_refused() {
        let ftdi = port("/dev/ttyUSB0", Some((0x0403, 0x6001)));
        let c3 = port("/dev/ttyACM0", Some(C3_USB));
        let bt = port("/dev/ttyS0", None);

        // One match: used, and said to be found.
        assert_eq!(
            choose_port(Given::Nothing, &[ftdi.clone(), c3.clone(), bt.clone()]),
            Ok(("/dev/ttyACM0".to_string(), true))
        );
        // macOS lists the device twice; the tty twin is the same port.
        let mac = [
            port("/dev/cu.usbmodem101", Some(C3_USB)),
            port("/dev/tty.usbmodem101", Some(C3_USB)),
        ];
        assert_eq!(
            choose_port(Given::Nothing, &mac),
            Ok(("/dev/cu.usbmodem101".to_string(), true))
        );
        // None: refused, listing what there is.
        assert_eq!(
            choose_port(Given::Nothing, &[ftdi.clone(), bt.clone()]).unwrap_err(),
            "no ESP32-C3 USB port (303a:1001) found; give --serial PORT (ports: /dev/ttyUSB0 \
             (0403:6001), /dev/ttyS0)"
        );
        assert!(choose_port(Given::Nothing, &[])
            .unwrap_err()
            .ends_with("(ports: none)"));
        // Several: refused, naming them.
        let two = [c3.clone(), port("/dev/ttyACM1", Some(C3_USB))];
        assert_eq!(
            choose_port(Given::Nothing, &two).unwrap_err(),
            "2 ESP32-C3 USB ports (303a:1001): /dev/ttyACM0, /dev/ttyACM1; give --serial PORT"
        );
        // --serial names it, whatever is listed.
        assert_eq!(
            choose_port(Given::Serial("COM5"), &two),
            Ok(("COM5".to_string(), false))
        );
        // --tcp and --device carry no control lines.
        let tcp = choose_port(Given::Tcp, std::slice::from_ref(&c3)).unwrap_err();
        assert!(tcp.starts_with("install-esp needs the C3's USB port (--serial PORT)"));
        assert!(tcp.ends_with("--tcp does not carry"), "{tcp}");
        assert!(choose_port(Given::Device, &[c3])
            .unwrap_err()
            .ends_with("BLE does not carry"));
    }

    #[test]
    fn only_a_whole_4_mb_esp_image_is_installed() {
        let mut image = vec![0xFF; IMAGE_LEN];
        image[0] = ESP_IMAGE_MAGIC;
        assert_eq!(check_image(&image), Ok(()));
        image[0] = 0xFF;
        assert!(check_image(&image)
            .unwrap_err()
            .contains("starts with 0xff, not the ESP image magic 0xe9"));
        let mut app = vec![0xFF; 1_200_000];
        app[0] = ESP_IMAGE_MAGIC;
        assert!(check_image(&app)
            .unwrap_err()
            .starts_with("not a merged install image: 1200000 B, not 4194304 B"));
        assert!(check_image(&[]).is_err());
    }

    #[test]
    fn the_guard_asks_about_a_bridge_and_needs_yes_without_a_terminal() {
        let found = Answered {
            name: "offroad-slave".into(),
            addr: Some(0x03),
            commit: Some(0x1a2b_3c4d),
        };
        assert_eq!(
            found.describe(),
            "offroad-slave, address 0x03, commit 1a2b3c4d"
        );
        // Nothing answered: install, terminal or not.
        assert_eq!(guard(None, false, true), Guard::Proceed);
        assert_eq!(guard(None, false, false), Guard::Proceed);
        // A bridge answered: ask at a terminal, unless --yes.
        assert_eq!(
            guard(Some(&found), false, true),
            Guard::Ask(
                "Erase offroad-slave's name, address and Wi-Fi credentials and install?".into()
            )
        );
        assert_eq!(guard(Some(&found), true, true), Guard::Proceed);
        // Without a terminal: refused, unless --yes.
        assert_eq!(
            guard(Some(&found), false, false),
            Guard::Refuse(
                "offroad-slave answered on this port: give --yes to install over it without a \
                 terminal"
                    .into()
            )
        );
        assert_eq!(guard(Some(&found), true, false), Guard::Proceed);
        let unknown = Answered {
            addr: None,
            commit: None,
            ..found
        };
        assert_eq!(
            unknown.describe(),
            "offroad-slave, address unknown, commit unknown"
        );
    }
}
