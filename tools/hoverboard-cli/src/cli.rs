//! The command line (`specs/machine-settings.md` section 6.3): the link options, then the noun the
//! command acts on (`board`, `machine` or `bridge`) and its verb; `scan` stands outside, since it
//! runs before there is a machine. [`parse`] reads it into a [`Command`] before any link is opened.

use std::time::Duration;

use espsvc::Ctrl;

/// Which link to the bridge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Link {
    /// BLE, to this advertiser or the first `hb-bridge-` one.
    Ble(Option<String>),
    /// The USB console at this device.
    Serial(String),
    /// The USB console through the relay at this `host:port`.
    Tcp(String),
}

/// What a command acts on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Noun {
    /// One motor controller.
    Board,
    /// Every motor controller of the machine.
    Machine,
    /// One bridge.
    Bridge,
}

impl Noun {
    /// The noun as the command line spells it.
    pub fn word(self) -> &'static str {
        match self {
            Noun::Board => "board",
            Noun::Machine => "machine",
            Noun::Bridge => "bridge",
        }
    }
}

/// What a file command does with the files (`specs/cli-config.md` section 4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Merge: write what differs, un-store nothing.
    Apply,
    /// Make the board equal the files.
    Replace,
    /// Print what `apply`, or with `replace` set `replace`, would change.
    Diff {
        /// Against `replace` rather than `apply`.
        replace: bool,
    },
}

/// A command that goes to the bridge wired to a board, over SWD.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Swd {
    /// `TARGET_ID`.
    TargetId,
    /// `CLEAR_PROTECTION`, asked about first.
    ClearProtection,
    /// `TARGET_CTRL`.
    Ctrl(Ctrl),
    /// `MEM_READ` of `len` bytes at `addr`: once, or with `every` (`--hz`) again at that interval
    /// until interrupted (`specs/board-route.md` section 2).
    Read {
        /// The address.
        addr: u32,
        /// How many bytes.
        len: u32,
        /// The interval `--hz` gives.
        every: Option<Duration>,
    },
    /// `MEM_WRITE` of whole words at a word-aligned `addr`.
    Write {
        /// The address.
        addr: u32,
        /// The bytes.
        data: Vec<u8>,
    },
    /// `board flash`: the ELF at `file`, a development build, flashed as it is
    /// (`specs/firmware-bundle.md` section 4.1, last paragraph).
    Flash {
        /// An ELF; a bundle is refused (it takes `board update`).
        file: String,
    },
    /// `board update`: the image a firmware bundle has for the board's chip
    /// (`specs/firmware-bundle.md` section 4.1).
    Update {
        /// A bundle; an ELF is refused (it takes `board flash`).
        bundle: String,
        /// `--image`: the image to take when several in the bundle match the chip.
        image: Option<String>,
        /// `--yes`: flash without asking.
        yes: bool,
    },
}

/// One command, its arguments read and checked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// `scan`: the BLE advertisers.
    Scan {
        /// How long to listen.
        secs: u64,
    },
    /// `board fields --json`: the name table.
    Fields,
    /// `board get` / `bridge get`: one field.
    Get {
        /// `Board` or `Bridge`.
        noun: Noun,
        /// The node.
        target: String,
        /// The field's path.
        path: String,
    },
    /// `board set` / `bridge set`: write one field.
    Set {
        /// `Board` or `Bridge`.
        noun: Noun,
        /// The node.
        target: String,
        /// The field's path.
        path: String,
        /// The value, in the file's forms.
        value: String,
        /// `--force`.
        force: bool,
    },
    /// `board dump` / `bridge dump`: the node's file.
    Dump {
        /// `Board` or `Bridge`.
        noun: Noun,
        /// The node.
        target: String,
        /// Where to write it; stdout without.
        out: Option<String>,
    },
    /// `apply`, `replace` and `diff` under `board` or `bridge`.
    Files {
        /// `Board` or `Bridge`.
        noun: Noun,
        /// The node.
        target: String,
        /// The files, merged in order.
        files: Vec<String>,
        /// What to do with them.
        mode: Mode,
        /// `--force`.
        force: bool,
        /// `--yes`.
        yes: bool,
    },
    /// `board compact`.
    Compact {
        /// The board.
        target: String,
    },
    /// `board status`: the boot report of each board given, or every motor controller.
    Status {
        /// The boards; every motor controller when empty.
        targets: Vec<String>,
    },
    /// `board erase`: `STORE_ERASE`, then the address restore.
    Erase {
        /// The board.
        target: String,
        /// `--yes`.
        yes: bool,
    },
    /// A board command that goes to its bridge over SWD.
    Swd {
        /// The board.
        target: String,
        /// What to do.
        op: Swd,
    },
    /// `board recover`: hold the board on the attached bridge's SWD header in reset, attach and halt
    /// it at its reset vector (`specs/bridge-nrst.md` section 3). It names no board: one whose
    /// firmware blocks SWD answers neither a walk nor a `uid` read.
    Recover,
    /// `machine nodes`: the walk, with each board's flags.
    Nodes,
    /// `machine get`: a machine field from every motor controller.
    MachineGet {
        /// The field's path.
        path: String,
    },
    /// `machine set`: a machine field to every motor controller.
    MachineSet {
        /// The field's path.
        path: String,
        /// The value.
        value: String,
        /// `--force`.
        force: bool,
    },
    /// `machine dump`: the machine file.
    MachineDump {
        /// Where to write it.
        out: String,
        /// The board whose machine copy to write when the copies differ.
        from: Option<String>,
    },
    /// `machine apply`, `replace` and `diff`; `diff` with no file lists the machine fields whose
    /// copies differ.
    MachineFiles {
        /// The machine file; `None` only for `diff`.
        file: Option<String>,
        /// What to do with it.
        mode: Mode,
        /// `--force`.
        force: bool,
        /// `--yes`.
        yes: bool,
        /// `--map <uid>=<target>`, each pairing an entry with a board.
        maps: Vec<(String, String)>,
    },
    /// `machine watch`.
    Watch {
        /// How long.
        secs: u64,
        /// The boards; every motor controller when empty.
        targets: Vec<String>,
        /// `--json`: one JSON line per frame in place of the once-a-second line.
        json: bool,
    },
    /// `machine drive` (`specs/twelve-fet.md` section 6.3).
    Drive {
        /// The throttle value.
        value: i16,
        /// How long.
        secs: u64,
        /// The wheels: `<board>` for all its motors, `<board>:<motor>` for one; every wheel of
        /// every motor controller when empty.
        targets: Vec<String>,
    },
    /// `bridge info`: `ESP_INFO`.
    BridgeInfo {
        /// The bridge.
        target: String,
    },
    /// `bridge update`: a new bridge image over the link, from a bundle or an app image.
    BridgeUpdate {
        /// The bridge.
        target: String,
        /// The bundle or the app image.
        file: String,
        /// `--yes`: no question for a bundle's image.
        yes: bool,
        /// `--no-confirm`.
        no_confirm: bool,
    },
    /// `firmware info`: a bundle's manifest; no link.
    FirmwareInfo {
        /// The bundle.
        bundle: String,
    },
    /// `firmware pack`: build a bundle; no link.
    FirmwarePack {
        /// Where to write it.
        out: String,
        /// Each `<crate-dir>:<image-name>=<elf>`.
        entries: Vec<String>,
        /// `--dirty`: pack from a tree with uncommitted changes.
        dirty: bool,
    },
    /// `bridge install`: the first install over USB.
    BridgeInstall {
        /// A bundle, or the merged install image.
        image: String,
        /// Files to apply after the first boot.
        files: Vec<String>,
        /// `--yes`.
        yes: bool,
    },
}

/// A command line: the link, when one is given, and the command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Invocation {
    /// The link option.
    pub link: Option<Link>,
    /// The command.
    pub command: Command,
}

/// The usage text every usage error is.
pub const USAGE: &str =
    "usage: hoverboard-cli [--device NAME|ADDR | --serial DEVICE | --tcp HOST:PORT] <command>
  scan [--secs N]
  board get <board> <path> | set <board> <path> <value> [--force] | dump <board> [<file>]
  board apply <board> <file>... [--force] | replace <board> <file>... [--yes] [--force]
  board diff [--replace] <board> <file>... [--force] | status [<board>...] | compact <board>
  board fields --json | erase <board> [--yes] | clear-protection <board>
  board update <board> <bundle> [--image NAME] [--yes] | flash <board> <elf>
  board disarm|halt|resume|reset|reset-halt <board> | target-id <board> | recover
  board read <board> <addr> <len> [--hz N] | write <board> <addr> <hex>
  machine nodes | get <path> | set <path> <value> [--force] | dump <file> [--from <board>]
  machine apply <file> [--map <uid>=<board>]... [--force]
  machine replace <file> [--yes] [--map <uid>=<board>]... [--force]
  machine diff [--replace] [<file>] [--map <uid>=<board>]... [--force]
  machine watch <secs> [<board>...] [--json] | drive <value> <secs> [<board>[:<motor>]...]
  bridge info <bridge> | update <bridge> <bundle|bin> [--yes] [--no-confirm]
  bridge install <bundle|image> [<file>...] [--yes]
  bridge get|set|dump|apply|replace|diff, as under board
  firmware info <bundle> | pack <out.hbfw> <crate-dir>:<image-name>=<elf>... [--dirty]
a board or bridge is an address (0x02), a device.name, or a uid prefix of 4 to 24 hex digits";

/// The options a command may take, wherever they stand after the link options.
const FLAGS: &[&str] = &[
    "--force",
    "--yes",
    "--replace",
    "--json",
    "--no-confirm",
    "--dirty",
];
/// The command options that take a value.
const VALUED: &[&str] = &["--secs", "--from", "--map", "--hz", "--image"];

/// The command's own options, as given.
#[derive(Default)]
struct Opts<'a> {
    flags: Vec<&'a str>,
    secs: Option<&'a str>,
    hz: Option<&'a str>,
    from: Option<&'a str>,
    image: Option<&'a str>,
    maps: Vec<&'a str>,
}

impl Opts<'_> {
    fn has(&self, f: &str) -> bool {
        self.flags.contains(&f)
    }

    /// Every option given is one of `allowed`.
    fn only(&self, allowed: &[&str]) -> bool {
        let given = self
            .flags
            .iter()
            .copied()
            .chain(self.secs.map(|_| "--secs"))
            .chain(self.hz.map(|_| "--hz"))
            .chain(self.from.map(|_| "--from"))
            .chain(self.image.map(|_| "--image"))
            .chain(self.maps.iter().map(|_| "--map"));
        given.into_iter().all(|o| allowed.contains(&o))
    }
}

/// Read the command line. The first value is whether the command is a `diff`, known as soon as its
/// verb is, so that a `diff` reports every error, usage errors included, with exit code 2.
pub fn parse(args: &[String]) -> (bool, Result<Invocation, String>) {
    let mut link = None;
    let mut early: Option<String> = None;
    let mut opts = Opts::default();
    let mut pos: Vec<&str> = Vec::new();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        // `--opt value` or `--opt=value`.
        let (opt, inline) = match a.split_once('=') {
            Some((o, v)) if a.starts_with("--") => (o, Some(v)),
            _ => (a.as_str(), None),
        };
        let mut value = || inline.or_else(|| it.next().map(String::as_str));
        match opt {
            "--device" | "--serial" | "--tcp" => match value() {
                Some(v) => {
                    let l = match opt {
                        "--device" => Link::Ble(Some(v.into())),
                        "--serial" => Link::Serial(v.into()),
                        _ => Link::Tcp(v.into()),
                    };
                    if link.replace(l).is_some() {
                        early.get_or_insert("give one of --device, --serial, --tcp".into());
                    }
                }
                None => {
                    early.get_or_insert(format!("{opt} needs a value"));
                }
            },
            o if FLAGS.contains(&o) => opts.flags.push(o),
            o if VALUED.contains(&o) => match value() {
                Some(v) => match o {
                    "--secs" => opts.secs = Some(v),
                    "--hz" => opts.hz = Some(v),
                    "--from" => opts.from = Some(v),
                    "--image" => opts.image = Some(v),
                    _ => opts.maps.push(v),
                },
                None => {
                    early.get_or_insert(format!("{o} needs a value"));
                }
            },
            o if o.starts_with("--") => {
                early.get_or_insert(format!("unknown option {o}"));
            }
            _ => pos.push(a),
        }
    }
    let is_diff = matches!(pos[..], ["board" | "machine" | "bridge", "diff", ..]);
    if let Some(e) = early {
        return (is_diff, Err(e));
    }
    (
        is_diff,
        command(&pos, &opts).map(|command| Invocation { link, command }),
    )
}

fn usage<T>() -> Result<T, String> {
    Err(USAGE.into())
}

fn owned(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

/// A number: decimal, or hexadecimal with `0x`.
pub fn parse_u32(s: &str) -> Result<u32, String> {
    let r = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(h) => u32::from_str_radix(h, 16),
        None => s.parse(),
    };
    r.map_err(|_| format!("not a number: {s}"))
}

/// Bytes from hex digits, `0x` allowed.
pub fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let s = s.trim_start_matches("0x");
    if !s.len().is_multiple_of(2) {
        return Err("hex needs an even number of digits".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|_| format!("not hex: {s}")))
        .collect()
}

/// `--hz`'s interval: a rate above zero, at most one read a millisecond.
fn parse_hz(s: &str) -> Result<Duration, String> {
    match s.parse::<f64>() {
        Ok(hz) if hz.is_finite() && hz > 0.0 && hz <= 1000.0 => {
            Ok(Duration::from_secs_f64(1.0 / hz))
        }
        _ => Err(format!("--hz takes a rate above 0 and at most 1000: {s}")),
    }
}

fn parse_secs(s: &str) -> Result<u64, String> {
    s.parse()
        .map_err(|_| format!("not a number of seconds: {s}"))
}

/// The command `pos` names, with `opts` checked against it.
fn command(pos: &[&str], o: &Opts) -> Result<Command, String> {
    let ok = |allowed: &[&str]| if o.only(allowed) { Ok(()) } else { usage() };
    let mode = |verb: &str| match verb {
        "apply" => Mode::Apply,
        "replace" => Mode::Replace,
        _ => Mode::Diff {
            replace: o.has("--replace"),
        },
    };
    let file_opts = |verb: &str| -> &'static [&'static str] {
        match verb {
            "apply" => &["--force"],
            "replace" => &["--force", "--yes"],
            _ => &["--force", "--replace"],
        }
    };
    let maps = || -> Result<Vec<(String, String)>, String> {
        o.maps
            .iter()
            .map(|m| {
                m.split_once('=')
                    .map(|(u, t)| (u.to_string(), t.to_string()))
                    .ok_or_else(|| format!("--map takes <uid>=<board>, not {m}"))
            })
            .collect()
    };
    Ok(match pos {
        ["scan"] => {
            ok(&["--secs"])?;
            Command::Scan {
                secs: o.secs.map_or(Ok(5), parse_secs)?,
            }
        }
        [noun @ ("board" | "bridge"), verb, rest @ ..] => {
            let noun = if *noun == "board" {
                Noun::Board
            } else {
                Noun::Bridge
            };
            node_command(noun, verb, rest, o, &ok, mode, file_opts)?
        }
        ["firmware", "info", bundle] => {
            ok(&[])?;
            Command::FirmwareInfo {
                bundle: bundle.to_string(),
            }
        }
        ["firmware", "pack", out, entries @ ..] if !entries.is_empty() => {
            ok(&["--dirty"])?;
            Command::FirmwarePack {
                out: out.to_string(),
                entries: owned(entries),
                dirty: o.has("--dirty"),
            }
        }
        ["machine", "nodes"] => {
            ok(&[])?;
            Command::Nodes
        }
        ["machine", "get", path] => {
            ok(&[])?;
            Command::MachineGet {
                path: path.to_string(),
            }
        }
        ["machine", "set", path, value] => {
            ok(&["--force"])?;
            Command::MachineSet {
                path: path.to_string(),
                value: value.to_string(),
                force: o.has("--force"),
            }
        }
        ["machine", "dump", out] => {
            ok(&["--from"])?;
            Command::MachineDump {
                out: out.to_string(),
                from: o.from.map(str::to_string),
            }
        }
        ["machine", verb @ ("apply" | "replace" | "diff"), file @ ..] => {
            let mut allowed = file_opts(verb).to_vec();
            allowed.push("--map");
            ok(&allowed)?;
            let file = match (file, *verb) {
                ([f], _) => Some(f.to_string()),
                ([], "diff") if !o.has("--replace") && o.maps.is_empty() => None,
                _ => return usage(),
            };
            Command::MachineFiles {
                file,
                mode: mode(verb),
                force: o.has("--force"),
                yes: o.has("--yes"),
                maps: maps()?,
            }
        }
        ["machine", "watch", secs, targets @ ..] => {
            ok(&["--json"])?;
            Command::Watch {
                secs: parse_secs(secs)?,
                targets: owned(targets),
                json: o.has("--json"),
            }
        }
        ["machine", "drive", value, secs, targets @ ..] => {
            ok(&[])?;
            Command::Drive {
                value: value
                    .parse()
                    .map_err(|_| format!("not a drive value (-32768..32767): {value}"))?,
                secs: parse_secs(secs)?,
                targets: owned(targets),
            }
        }
        _ => return usage(),
    })
}

/// A `board` or `bridge` command.
fn node_command(
    noun: Noun,
    verb: &str,
    rest: &[&str],
    o: &Opts,
    ok: &dyn Fn(&[&str]) -> Result<(), String>,
    mode: impl Fn(&str) -> Mode,
    file_opts: impl Fn(&str) -> &'static [&'static str],
) -> Result<Command, String> {
    let board = noun == Noun::Board;
    let target = |t: &str| t.to_string();
    let swd = |t: &str, op: Swd| {
        ok(&[])?;
        Ok(Command::Swd {
            target: target(t),
            op,
        })
    };
    let ctrl = |c| match c {
        "disarm" => Some(Ctrl::Disarm),
        "halt" => Some(Ctrl::Halt),
        "resume" => Some(Ctrl::Resume),
        "reset" => Some(Ctrl::Reset),
        "reset-halt" => Some(Ctrl::ResetHalt),
        _ => None,
    };
    match (verb, rest) {
        ("get", [t, path]) => {
            ok(&[])?;
            Ok(Command::Get {
                noun,
                target: target(t),
                path: path.to_string(),
            })
        }
        ("set", [t, path, value]) => {
            ok(&["--force"])?;
            Ok(Command::Set {
                noun,
                target: target(t),
                path: path.to_string(),
                value: value.to_string(),
                force: o.has("--force"),
            })
        }
        ("dump", [t, out @ ..]) if out.len() <= 1 => {
            ok(&[])?;
            Ok(Command::Dump {
                noun,
                target: target(t),
                out: out.first().map(|s| s.to_string()),
            })
        }
        ("apply" | "replace" | "diff", [t, files @ ..]) if !files.is_empty() => {
            ok(file_opts(verb))?;
            Ok(Command::Files {
                noun,
                target: target(t),
                files: owned(files),
                mode: mode(verb),
                force: o.has("--force"),
                yes: o.has("--yes"),
            })
        }
        ("compact", [t]) if board => {
            ok(&[])?;
            Ok(Command::Compact { target: target(t) })
        }
        ("status", targets) if board => {
            ok(&[])?;
            Ok(Command::Status {
                targets: owned(targets),
            })
        }
        ("fields", []) if board && o.has("--json") => {
            ok(&["--json"])?;
            Ok(Command::Fields)
        }
        ("erase", [t]) if board => {
            ok(&["--yes"])?;
            Ok(Command::Erase {
                target: target(t),
                yes: o.has("--yes"),
            })
        }
        ("flash", [t, file]) if board => swd(
            t,
            Swd::Flash {
                file: file.to_string(),
            },
        ),
        ("update", [t, bundle]) if board => {
            ok(&["--image", "--yes"])?;
            Ok(Command::Swd {
                target: target(t),
                op: Swd::Update {
                    bundle: bundle.to_string(),
                    image: o.image.map(str::to_string),
                    yes: o.has("--yes"),
                },
            })
        }
        ("clear-protection", [t]) if board => swd(t, Swd::ClearProtection),
        ("target-id", [t]) if board => swd(t, Swd::TargetId),
        ("recover", []) if board => {
            ok(&[])?;
            Ok(Command::Recover)
        }
        (c, [t]) if board && ctrl(c).is_some() => swd(t, Swd::Ctrl(ctrl(c).expect("matched"))),
        ("read", [t, addr, len]) if board => {
            ok(&["--hz"])?;
            Ok(Command::Swd {
                target: target(t),
                op: Swd::Read {
                    addr: parse_u32(addr)?,
                    len: parse_u32(len)?,
                    every: o.hz.map(parse_hz).transpose()?,
                },
            })
        }
        ("write", [t, addr, hex]) if board => {
            let (addr, data) = (parse_u32(addr)?, parse_hex(hex)?);
            if addr % 4 != 0 || data.len() % 4 != 0 {
                return Err("write takes a word-aligned address and whole words".into());
            }
            swd(t, Swd::Write { addr, data })
        }
        ("info", [t]) if !board => {
            ok(&[])?;
            Ok(Command::BridgeInfo { target: target(t) })
        }
        ("update", [t, file]) if !board => {
            ok(&["--yes", "--no-confirm"])?;
            Ok(Command::BridgeUpdate {
                target: target(t),
                file: file.to_string(),
                yes: o.has("--yes"),
                no_confirm: o.has("--no-confirm"),
            })
        }
        ("install", [image, files @ ..]) if !board => {
            ok(&["--yes"])?;
            Ok(Command::BridgeInstall {
                image: image.to_string(),
                files: owned(files),
                yes: o.has("--yes"),
            })
        }
        _ => usage(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(line: &str) -> Result<Command, String> {
        let args: Vec<String> = line.split_whitespace().map(str::to_string).collect();
        parse(&args).1.map(|i| i.command)
    }

    fn s(v: &str) -> String {
        v.into()
    }

    /// `specs/machine-settings.md` section 6.3's table: every old command, written in its noun
    /// form, is the command the old one was.
    #[test]
    fn every_old_command_maps_to_its_noun_form() {
        let get = |noun| Command::Get {
            noun,
            target: s("rover-left"),
            path: s("machine.mode"),
        };
        let set = |noun, path: &str, value: &str| Command::Set {
            noun,
            target: s("rover-left"),
            path: s(path),
            value: s(value),
            force: false,
        };
        let files = |noun, mode, yes| Command::Files {
            noun,
            target: s("rover-left"),
            files: vec![s("a.json"), s("b.json")],
            mode,
            force: false,
            yes,
        };
        let swd = |op| Command::Swd {
            target: s("rover-left"),
            op,
        };
        let table: Vec<(&str, &str, Command)> = vec![
            ("scan", "scan --secs 3", Command::Scan { secs: 3 }),
            ("nodes", "machine nodes", Command::Nodes),
            (
                "status",
                "board status rover-left 0x02",
                Command::Status {
                    targets: vec![s("rover-left"), s("0x02")],
                },
            ),
            (
                "name (a board)",
                "board set rover-left device.name rover-left",
                set(Noun::Board, "device.name", "rover-left"),
            ),
            (
                "name (a bridge)",
                "bridge set rover-left device.name rover-left",
                set(Noun::Bridge, "device.name", "rover-left"),
            ),
            (
                "config (read)",
                "board get rover-left machine.mode",
                get(Noun::Board),
            ),
            (
                "config (write)",
                "board set rover-left machine.mode balance",
                set(Noun::Board, "machine.mode", "balance"),
            ),
            (
                "config (a bridge)",
                "bridge get rover-left machine.mode",
                get(Noun::Bridge),
            ),
            (
                "dump",
                "board dump rover-left out.json",
                Command::Dump {
                    noun: Noun::Board,
                    target: s("rover-left"),
                    out: Some(s("out.json")),
                },
            ),
            (
                "dump (a bridge)",
                "bridge dump rover-left",
                Command::Dump {
                    noun: Noun::Bridge,
                    target: s("rover-left"),
                    out: None,
                },
            ),
            (
                "apply",
                "board apply rover-left a.json b.json",
                files(Noun::Board, Mode::Apply, false),
            ),
            (
                "replace",
                "board replace rover-left a.json b.json --yes",
                files(Noun::Board, Mode::Replace, true),
            ),
            (
                "diff",
                "board diff --replace rover-left a.json b.json",
                files(Noun::Board, Mode::Diff { replace: true }, false),
            ),
            (
                "apply (a bridge)",
                "bridge apply rover-left a.json b.json",
                files(Noun::Bridge, Mode::Apply, false),
            ),
            (
                "compact",
                "board compact rover-left",
                Command::Compact {
                    target: s("rover-left"),
                },
            ),
            ("fields", "board fields --json", Command::Fields),
            (
                "erase-store",
                "board erase rover-left --yes",
                Command::Erase {
                    target: s("rover-left"),
                    yes: true,
                },
            ),
            (
                "flash",
                "board flash rover-left fw.elf",
                swd(Swd::Flash { file: s("fw.elf") }),
            ),
            (
                "clear-protection",
                "board clear-protection rover-left",
                swd(Swd::ClearProtection),
            ),
            (
                "disarm",
                "board disarm rover-left",
                swd(Swd::Ctrl(Ctrl::Disarm)),
            ),
            ("halt", "board halt rover-left", swd(Swd::Ctrl(Ctrl::Halt))),
            (
                "resume",
                "board resume rover-left",
                swd(Swd::Ctrl(Ctrl::Resume)),
            ),
            (
                "reset",
                "board reset rover-left",
                swd(Swd::Ctrl(Ctrl::Reset)),
            ),
            (
                "reset-halt",
                "board reset-halt rover-left",
                swd(Swd::Ctrl(Ctrl::ResetHalt)),
            ),
            (
                "read",
                "board read rover-left 0x0800F800 2048",
                swd(Swd::Read {
                    addr: 0x0800_F800,
                    len: 2048,
                    every: None,
                }),
            ),
            (
                "write",
                "board write rover-left 0x20000000 01020304",
                swd(Swd::Write {
                    addr: 0x2000_0000,
                    data: vec![1, 2, 3, 4],
                }),
            ),
            ("id", "board target-id rover-left", swd(Swd::TargetId)),
            ("recover", "board recover", Command::Recover),
            (
                "watch",
                "machine watch 2 0x01 rover-right",
                Command::Watch {
                    secs: 2,
                    targets: vec![s("0x01"), s("rover-right")],
                    json: false,
                },
            ),
            (
                "watch json",
                "machine watch 2 --json",
                Command::Watch {
                    secs: 2,
                    targets: Vec::new(),
                    json: true,
                },
            ),
            (
                "drive",
                "machine drive -300 2",
                Command::Drive {
                    value: -300,
                    secs: 2,
                    targets: Vec::new(),
                },
            ),
            (
                "info",
                "bridge info rover-l",
                Command::BridgeInfo {
                    target: s("rover-l"),
                },
            ),
            (
                "update-esp",
                "bridge update rover-l b.bin --no-confirm",
                Command::BridgeUpdate {
                    target: s("rover-l"),
                    file: s("b.bin"),
                    yes: false,
                    no_confirm: true,
                },
            ),
            (
                "update-esp",
                "bridge update rover-l hoverboard-firmware-1a2b3c4.hbfw --yes",
                Command::BridgeUpdate {
                    target: s("rover-l"),
                    file: s("hoverboard-firmware-1a2b3c4.hbfw"),
                    yes: true,
                    no_confirm: false,
                },
            ),
            (
                "install-esp",
                "--serial /dev/ttyACM0 bridge install i.bin wifi.json --yes",
                Command::BridgeInstall {
                    image: s("i.bin"),
                    files: vec![s("wifi.json")],
                    yes: true,
                },
            ),
        ];
        for (old, new, want) in table {
            assert_eq!(cmd(new), Ok(want), "{old}: {new}");
        }
        // The flat words and --node are gone.
        for old in [
            "nodes",
            "status",
            "name 0x01 x",
            "config 0x01 machine.mode",
            "dump 0x01",
            "fields --json",
            "info",
            "id",
            "erase-store --yes",
            "update-esp b.bin",
            "install-esp i.bin",
            "read 0x0 4",
            "watch 2",
            "drive 300 2",
        ] {
            assert_eq!(cmd(old), Err(USAGE.into()), "{old}");
        }
        assert_eq!(
            cmd("--node 0x03 bridge info 0x03"),
            Err("unknown option --node".into())
        );
    }

    #[test]
    fn the_machine_commands() {
        assert_eq!(
            cmd("machine apply m.json --map 8adc=rover-new --map 51ce=0x02"),
            Ok(Command::MachineFiles {
                file: Some(s("m.json")),
                mode: Mode::Apply,
                force: false,
                yes: false,
                maps: vec![(s("8adc"), s("rover-new")), (s("51ce"), s("0x02"))],
            })
        );
        assert_eq!(
            cmd("machine diff"),
            Ok(Command::MachineFiles {
                file: None,
                mode: Mode::Diff { replace: false },
                force: false,
                yes: false,
                maps: Vec::new(),
            })
        );
        assert_eq!(cmd("machine diff --replace"), Err(USAGE.into()));
        assert_eq!(
            cmd("machine dump m.json --from rover-left"),
            Ok(Command::MachineDump {
                out: s("m.json"),
                from: Some(s("rover-left")),
            })
        );
        assert_eq!(
            cmd("machine set machine.mode balance --force"),
            Ok(Command::MachineSet {
                path: s("machine.mode"),
                value: s("balance"),
                force: true,
            })
        );
        assert_eq!(cmd("machine apply m.json --yes"), Err(USAGE.into()));
        assert_eq!(
            cmd("machine apply m.json --map 8adc"),
            Err("--map takes <uid>=<board>, not 8adc".into())
        );
    }

    /// `specs/board-route.md` section 2: `board read --hz` reads at that rate until interrupted;
    /// without it, once.
    #[test]
    fn board_read_takes_a_rate() {
        let read = |every| {
            Ok(Command::Swd {
                target: s("0x01"),
                op: Swd::Read {
                    addr: 0x2000_0920,
                    len: 204,
                    every,
                },
            })
        };
        assert_eq!(
            cmd("board read 0x01 0x20000920 204 --hz 10"),
            read(Some(Duration::from_millis(100)))
        );
        assert_eq!(
            cmd("board read 0x01 0x20000920 204 --hz=2.5"),
            read(Some(Duration::from_millis(400)))
        );
        assert_eq!(cmd("board read 0x01 0x20000920 204"), read(None));
        for bad in ["0", "-1", "x", "2000", "inf"] {
            assert_eq!(
                cmd(&format!("board read 0x01 0x20000920 204 --hz {bad}")),
                Err(format!("--hz takes a rate above 0 and at most 1000: {bad}"))
            );
        }
        assert_eq!(
            cmd("board write 0x01 0x20000000 01020304 --hz 10"),
            Err(USAGE.into())
        );
        assert_eq!(cmd("board target-id 0x01 --hz 10"), Err(USAGE.into()));
    }

    /// `specs/bridge-nrst.md` section 5: `board recover` takes no target, and a board named is a
    /// usage error; `bridge recover` is no command.
    #[test]
    fn board_recover_takes_no_target() {
        assert_eq!(cmd("board recover"), Ok(Command::Recover));
        for line in [
            "board recover rover-left",
            "board recover 0x02",
            "board recover b37b",
            "bridge recover",
        ] {
            assert_eq!(cmd(line), Err(USAGE.into()), "{line}");
        }
    }

    /// `specs/cli-onboarding.md` part A: a machine is updated one device at a time (`bridge update`,
    /// `board update`); `machine update` is no command.
    #[test]
    fn machine_update_is_no_command() {
        for line in ["machine update b.hbfw", "machine update b.hbfw --yes"] {
            assert_eq!(cmd(line), Err(USAGE.into()), "{line}");
        }
        assert!(!USAGE.contains("machine update"));
    }

    #[test]
    fn an_option_the_command_does_not_take_is_a_usage_error() {
        for line in [
            "board get 0x01 machine.mode --force",
            "board apply 0x01 a.json --yes",
            "board diff 0x01 a.json --yes",
            "board fields",
            "board fields --json --force",
            "board erase 0x01 --force",
            "bridge compact 0x03",
            "bridge flash 0x03 fw.elf",
            "board info 0x01",
            "board update 0x01 b.hbfw --dirty",
            "board recover --yes",
            "bridge update 0x03 b.bin --image x",
            "machine nodes --yes",
            "scan --yes",
            "board erase 0x01 --image x",
            "board flash 0x01 fw.hbfw --dirty",
            "firmware info b.hbfw --yes",
            "firmware info b.hbfw --image x",
            "firmware pack out.hbfw",
            "firmware pack out.hbfw a:b=c --yes",
        ] {
            assert_eq!(cmd(line), Err(USAGE.into()), "{line}");
        }
    }

    #[test]
    fn board_update_takes_a_bundle_board_flash_an_elf_and_firmware_reads_bundles() {
        let update = |image: Option<&str>, yes| {
            Ok(Command::Swd {
                target: s("offroad-master"),
                op: Swd::Update {
                    bundle: s("hoverboard-firmware-adab056.hbfw"),
                    image: image.map(s),
                    yes,
                },
            })
        };
        let line = "board update offroad-master hoverboard-firmware-adab056.hbfw";
        assert_eq!(cmd(line), update(None, false));
        assert_eq!(cmd(&format!("{line} --yes")), update(None, true));
        assert_eq!(
            cmd(&format!("{line} --image mainboard-gd32f103c8 --yes")),
            update(Some("mainboard-gd32f103c8"), true)
        );
        assert_eq!(
            cmd(&format!("{line} --image=mainboard-gd32f103c8")),
            update(Some("mainboard-gd32f103c8"), false)
        );
        assert_eq!(
            cmd("board flash offroad-master fw.elf"),
            Ok(Command::Swd {
                target: s("offroad-master"),
                op: Swd::Flash { file: s("fw.elf") },
            })
        );
        // board flash takes no bundle options.
        for line in [
            "board flash offroad-master fw.elf --yes",
            "board flash offroad-master fw.elf --image mainboard-gd32f103c8",
        ] {
            assert_eq!(cmd(line), Err(USAGE.into()), "{line}");
        }
        assert_eq!(
            cmd("firmware info b.hbfw"),
            Ok(Command::FirmwareInfo {
                bundle: s("b.hbfw")
            })
        );
        assert_eq!(
            cmd("firmware pack o.hbfw motorcontroller/firmware:mainboard-gd32f103c8=fw --dirty"),
            Ok(Command::FirmwarePack {
                out: s("o.hbfw"),
                entries: vec![s("motorcontroller/firmware:mainboard-gd32f103c8=fw")],
                dirty: true,
            })
        );
    }

    #[test]
    fn a_diff_is_known_before_its_errors() {
        let p = |line: &str| {
            let args: Vec<String> = line.split_whitespace().map(str::to_string).collect();
            let (diff, r) = parse(&args);
            (diff, r.is_ok())
        };
        assert_eq!(p("board diff x f.json --bogus"), (true, false));
        assert_eq!(p("--tcp a --tcp b machine diff"), (true, false));
        assert_eq!(p("board diff"), (true, false));
        assert_eq!(p("board apply x f.json --bogus"), (false, false));
    }

    #[test]
    fn one_link_and_its_value() {
        let args = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        let (_, r) = parse(&args(&["--tcp=h:1", "machine", "nodes"]));
        assert_eq!(r.unwrap().link, Some(Link::Tcp(s("h:1"))));
        let (_, r) = parse(&args(&[
            "--tcp", "h:1", "--serial", "x", "machine", "nodes",
        ]));
        assert_eq!(r, Err("give one of --device, --serial, --tcp".into()));
        let (_, r) = parse(&args(&["machine", "nodes", "--tcp"]));
        assert_eq!(r, Err("--tcp needs a value".into()));
    }
}
