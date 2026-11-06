//! The console as an L3 guest (`specs/hoverboard-bridge.md`, "`watch` and `drive`"; `machine watch`
//! and `machine drive` on the command line): attach with `NODE_HELLO` to the node on the link, tally
//! the `CYCLIC_STATE` frames the board emits on its mailbox port (the bridge re-publishes them on
//! every controller link), and build the `DRIVE_CMD` and `INPUTS` frames `drive` sends: one
//! `DRIVE_CMD` per wheel and one `INPUTS` per node (`specs/twelve-fet.md` section 6.3).

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use linkctl::{
    CyclicMotor, CyclicState, DriveCmd, DriveKind, Inputs, OP_CYCLIC_STATE, OP_DRIVE_CMD, OP_INPUTS,
};
use net::pdu::Pdu;
use net::walk::{Controller, CFG_OK};

use crate::nodes;
use crate::wire::Wire;

/// What an attach learned from the `NODE_HELLO` reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Attached {
    /// The guest address the node granted.
    pub guest: u8,
    /// The node's own address, `None` for a node with none yet (`Controller::gateway_addr`).
    pub node: Option<u8>,
}

/// Attach to the node at the other end of `wire` (the bridge, or a board's own BLE module): send
/// `NODE_HELLO` (dst `0x00`, the one peer) up to `tries` times, `within` apart. Every other packet
/// that arrives meanwhile is dropped.
pub fn attach<W: Wire>(wire: &mut W, tries: u32, within: Duration) -> Option<Attached> {
    for _ in 0..tries {
        let mut c = Controller::new();
        let hello = c
            .next_request()
            .expect("a new controller starts with its hello");
        wire.send(&hello);
        let deadline = Instant::now() + within;
        while let Some(pkt) = wire.recv(deadline.saturating_duration_since(Instant::now())) {
            if c.on_reply(&pkt) {
                return Some(Attached {
                    guest: c.granted()?,
                    node: c.gateway_addr(),
                });
            }
        }
    }
    None
}

/// What one `src` has sent.
struct Source {
    frames: u32,
    last_at: Duration,
    longest_gap: Duration,
    last: CyclicState,
    /// The last frame's motor-1 block: a two-motor board's motor 1 (`specs/link-control.md`, "The
    /// motor-1 block"), `None` from a one-motor board.
    last_motor1: Option<CyclicMotor>,
}

/// The `CYCLIC_STATE` frames heard, per `src`. Times are offsets from the start of the run.
#[derive(Default)]
pub struct Tally {
    sources: BTreeMap<u8, Source>,
    /// The sources counted; every one when empty.
    only: Vec<u8>,
}

impl Tally {
    /// An empty tally of every source.
    pub fn new() -> Self {
        Self::default()
    }

    /// An empty tally of the frames from `nodes` alone.
    pub fn only(nodes: &[u8]) -> Self {
        Tally {
            sources: BTreeMap::new(),
            only: nodes.to_vec(),
        }
    }

    /// Count `pkt` if it is a `CYCLIC_STATE` PDU whose payload decodes; it arrived at `at`.
    /// Returns whether it was counted.
    pub fn feed(&mut self, pkt: &[u8], at: Duration) -> bool {
        self.feed_src(pkt, at).is_some()
    }

    /// [`Self::feed`], returning the source address of a counted frame.
    pub fn feed_src(&mut self, pkt: &[u8], at: Duration) -> Option<u8> {
        let p = Pdu::decode(pkt).ok()?;
        if p.opcode != OP_CYCLIC_STATE || !(self.only.is_empty() || self.only.contains(&p.src)) {
            return None;
        }
        let (s, motor1) = CyclicState::decode_with_motor1(p.payload).ok()?;
        match self.sources.get_mut(&p.src) {
            Some(src) => {
                src.frames += 1;
                src.longest_gap = src.longest_gap.max(at.saturating_sub(src.last_at));
                src.last_at = at;
                src.last = s;
                src.last_motor1 = motor1;
            }
            None => {
                self.sources.insert(
                    p.src,
                    Source {
                        frames: 1,
                        last_at: at,
                        longest_gap: Duration::ZERO,
                        last: s,
                        last_motor1: motor1,
                    },
                );
            }
        }
        Some(p.src)
    }

    /// The last frame from `src` as one JSON object on one line, `at` seconds into the run: every
    /// word of the frame, decoded (`machine watch --json`). `None` while nothing is heard from it.
    pub fn frame_json(&self, src: u8, at: Duration) -> Option<String> {
        let f = self.sources.get(&src)?;
        let s = &f.last;
        let motor1 = f.last_motor1.map(|m| {
            serde_json::json!({
                "wheel_speed": m.wheel_speed,
                "wheel_rate": m.wheel_rate,
                "phase_peak": m.phase_peak,
                "phase_mean": m.phase_mean,
                "duty_on": m.duty_on,
            })
        });
        let v = serde_json::json!({
            "t": (at.as_secs_f64() * 1000.0).round() / 1000.0,
            "src": src,
            "mode": s.mode,
            "fault": s.fault,
            "flags": s.flags,
            "boot_tag": s.obs.boot_tag,
            "wheel_speed": s.wheel_speed,
            "wheel_rate": s.wheel_rate,
            "wheel_pos": s.wheel_pos,
            "phase_peak": s.obs.phase_peak,
            "phase_mean": s.obs.phase_mean,
            "duty_on": s.obs.duty_on,
            "battery": s.battery,
            "peer_demand": s.peer_demand,
            "pitch": s.pitch,
            "roll": s.roll,
            "odometer": s.obs.odometer,
            "machine": s.machine,
            "motor1": motor1,
        });
        Some(v.to_string())
    }

    /// The last frame heard from `src`.
    pub fn last(&self, src: u8) -> Option<&CyclicState> {
        self.sources.get(&src).map(|s| &s.last)
    }

    /// The motor-1 block of the last frame heard from `src`: `None` while nothing is heard or
    /// from a one-motor board.
    pub fn last_motor1(&self, src: u8) -> Option<&CyclicMotor> {
        self.sources.get(&src).and_then(|s| s.last_motor1.as_ref())
    }

    /// Frames from `src` so far.
    pub fn frames(&self, src: u8) -> u32 {
        self.sources.get(&src).map_or(0, |s| s.frames)
    }

    /// The longest interval between two frames from `src`.
    pub fn longest_gap(&self, src: u8) -> Duration {
        self.sources
            .get(&src)
            .map_or(Duration::ZERO, |s| s.longest_gap)
    }

    /// Frames per second from `src` over a run `elapsed` long.
    pub fn rate(&self, src: u8, elapsed: Duration) -> f64 {
        let secs = elapsed.as_secs_f64();
        if secs > 0.0 {
            f64::from(self.frames(src)) / secs
        } else {
            0.0
        }
    }

    /// The once-a-second line at `elapsed` into the run. A two-motor board's part ends in motor
    /// 1's window peak and mean (counts) and its rate (Q12 of a hall edge per window), from the
    /// frame's motor-1 block (`specs/link-control.md`, "The motor-1 block").
    pub fn line(&self, elapsed: Duration) -> String {
        let per: Vec<String> = self
            .sources
            .iter()
            .map(|(&a, s)| {
                let motor1 = s.last_motor1.map_or(String::new(), |m| {
                    format!(
                        " m1 peak {} mean {} rate {}",
                        m.phase_peak, m.phase_mean, m.wheel_rate
                    )
                });
                format!(
                    "{a:#04x} {} frames {:.1}/s gap {:.2}s mode {} peer_demand {} battery {} fault {:#06x}{motor1}",
                    s.frames,
                    self.rate(a, elapsed),
                    s.longest_gap.as_secs_f64(),
                    s.last.mode,
                    s.last.peer_demand,
                    s.last.battery,
                    s.last.fault
                )
            })
            .collect();
        let body = if per.is_empty() {
            "no CYCLIC_STATE heard".to_string()
        } else {
            per.join(" | ")
        };
        format!("t={}s  {body}", elapsed.as_secs())
    }

    /// One closing line per `src` for a run `elapsed` long.
    pub fn summary(&self, elapsed: Duration) -> Vec<String> {
        self.sources
            .iter()
            .map(|(&a, s)| {
                format!(
                    "{a:#04x}: {} frames in {:.1}s, {:.2}/s, longest gap {:.2}s",
                    s.frames,
                    elapsed.as_secs_f64(),
                    self.rate(a, elapsed),
                    s.longest_gap.as_secs_f64()
                )
            })
            .collect()
    }
}

fn control_pdu(opcode: u8, src: u8, dst: u8, payload: &[u8]) -> Vec<u8> {
    let p = Pdu::new(opcode, src, dst, payload).expect("a control opcode is a valid opcode");
    let mut buf = [0u8; 32];
    let n = p.encode(&mut buf).expect("a control payload fits");
    buf[..n].to_vec()
}

/// A `DRIVE_CMD` PDU from `src` to `dst`.
pub fn drive_pdu(src: u8, dst: u8, cmd: &DriveCmd) -> Vec<u8> {
    let mut b = [0u8; DriveCmd::LEN];
    let n = cmd.encode(&mut b);
    control_pdu(OP_DRIVE_CMD, src, dst, &b[..n])
}

/// An `INPUTS` PDU from `src` to `dst`.
pub fn inputs_pdu(src: u8, dst: u8, inputs: &Inputs) -> Vec<u8> {
    let mut b = [0u8; Inputs::LEN];
    let n = inputs.encode(&mut b);
    control_pdu(OP_INPUTS, src, dst, &b[..n])
}

/// One wheel: motor `motor` of the node at `node`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Wheel {
    /// The node's address.
    pub node: u8,
    /// The motor on that node.
    pub motor: u8,
}

/// A `machine drive` target as the command line gives it: a board (its name, address or `uid`
/// prefix), which drives every motor the board has, or `<board>:<motor>`, which drives one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WheelTarget<'a> {
    /// The board, as `target::parse` reads it.
    pub board: &'a str,
    /// The one motor, `None` for all of them.
    pub motor: Option<u8>,
}

impl<'a> WheelTarget<'a> {
    /// Read `spec`: a decimal after the last `:` is the motor, and anything else is the whole of
    /// it a board.
    pub fn parse(spec: &'a str) -> Result<Self, String> {
        match spec.rsplit_once(':') {
            Some((board, m)) if !m.is_empty() && m.bytes().all(|b| b.is_ascii_digit()) => {
                if board.is_empty() {
                    return Err(format!("no board before the motor: {spec}"));
                }
                let motor = m
                    .parse::<u8>()
                    .ok()
                    .filter(|&m| m < store::MOTORS)
                    .ok_or_else(|| {
                        format!("not a motor (0..{}): {m} in {spec}", store::MOTORS - 1)
                    })?;
                Ok(WheelTarget {
                    board,
                    motor: Some(motor),
                })
            }
            _ => Ok(WheelTarget {
                board: spec,
                motor: None,
            }),
        }
    }
}

/// How many motors the board at `dst` has, asked from `src`: two when `motors.1.gate.hi_a` is
/// stored, else one. The pin has no default, so a board that stores none answers it unstored and
/// has no motor 1 (`specs/twelve-fet.md` section 6.3).
pub fn motor_count<W: Wire>(wire: &mut W, src: u8, dst: u8) -> Result<u8, String> {
    let r = nodes::read(wire, src, dst, store::MOTOR_GATE_HI_A.at(1).key())?;
    Ok(if r.status == CFG_OK { 2 } else { 1 })
}

/// The wheels `picked` names, each board with the motor it names or all of them, given each
/// board's motor count. A wheel named twice is driven once; a motor a board does not have is
/// refused.
pub fn wheels(picked: &[(u8, Option<u8>)], count: impl Fn(u8) -> u8) -> Result<Vec<Wheel>, String> {
    let mut out: Vec<Wheel> = Vec::new();
    for &(node, motor) in picked {
        let n = count(node);
        let motors = match motor {
            Some(m) if m < n => m..m + 1,
            Some(m) => {
                return Err(format!(
                    "{node:#04x} has no motor {m}: it has {n} (motors.1.gate.hi_a {})",
                    if n > 1 { "stored" } else { "not stored" }
                ))
            }
            None => 0..n,
        };
        for motor in motors {
            let w = Wheel { node, motor };
            if !out.contains(&w) {
                out.push(w);
            }
        }
    }
    Ok(out)
}

/// `DRIVE_CMD` period while driving (20 Hz).
pub const DRIVE_PERIOD: Duration = Duration::from_millis(50);
/// `INPUTS` period while driving (2 Hz).
pub const INPUTS_PERIOD: Duration = Duration::from_millis(500);
/// How many times each stop frame goes out.
pub const STOP_REPEATS: usize = 3;

/// `drive`'s schedule: `DRIVE_CMD` (throttle, `value`, steer 0) to each wheel every
/// [`DRIVE_PERIOD`] and `INPUTS` (power 1, and the observer source request with `--observer`) to
/// each wheel's node every [`INPUTS_PERIOD`], from `src`. Arming is per node, because each board runs one mode machine for its motors. Times are
/// offsets from the start of the run.
pub struct Drive {
    src: u8,
    wheels: Vec<Wheel>,
    /// The wheels' nodes, each once, in the wheels' order.
    nodes: Vec<u8>,
    value: i16,
    /// `INPUTS.buttons` while driving: power, and the observer request with `--observer`.
    buttons: u8,
    /// `--observer-after`: the observer request joins `buttons` once the run has been going this
    /// long, so the request lands on a wheel the halls have already brought to speed and the
    /// observer has converged on (`specs/flux-observer.md`, "S4, first slice": at rest its angle
    /// is noise). `None` with `--observer` sets it from the first frame.
    observer_after: Option<Duration>,
    next_drive: Duration,
    next_inputs: Duration,
}

impl Drive {
    /// A schedule whose first frames are due at once. `observer` sets `INPUTS.buttons` bit 2 (the
    /// observer source request, `specs/flux-observer.md`, "S4, first slice") in every `INPUTS`
    /// while driving; the stop frames clear it with the power bit.
    pub fn new(src: u8, wheels: Vec<Wheel>, value: i16, observer: bool) -> Self {
        Self::with_observer_after(src, wheels, value, observer, None)
    }

    /// [`Drive::new`] with the observer request delayed by `after` (`--observer-after`), which
    /// implies the request.
    pub fn with_observer_after(
        src: u8,
        wheels: Vec<Wheel>,
        value: i16,
        observer: bool,
        after: Option<Duration>,
    ) -> Self {
        let mut nodes: Vec<u8> = Vec::new();
        for w in &wheels {
            if !nodes.contains(&w.node) {
                nodes.push(w.node);
            }
        }
        Drive {
            src,
            wheels,
            nodes,
            value,
            buttons: Inputs::BUTTON_POWER
                | if observer && after.is_none() {
                    Inputs::BUTTON_OBSERVER
                } else {
                    0
                },
            observer_after: if observer { after } else { None },
            next_drive: Duration::ZERO,
            next_inputs: Duration::ZERO,
        }
    }

    /// When the next frame falls due.
    pub fn next_due(&self) -> Duration {
        self.next_drive.min(self.next_inputs)
    }

    /// The frames due at `at`, in send order. A late call sends one of each due kind and resumes
    /// the cadence from `at`, rather than bursting the missed ones.
    pub fn due(&mut self, at: Duration) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        if self.observer_after.is_some_and(|d| at >= d) {
            self.buttons |= Inputs::BUTTON_OBSERVER;
        }
        if at >= self.next_inputs {
            let i = Inputs {
                buttons: self.buttons,
                rider: 0,
            };
            out.extend(self.nodes.iter().map(|&n| inputs_pdu(self.src, n, &i)));
            self.next_inputs = next_after(self.next_inputs, INPUTS_PERIOD, at);
        }
        if at >= self.next_drive {
            out.extend(self.wheels.iter().map(|w| {
                let d = DriveCmd {
                    kind: DriveKind::Throttle,
                    value: self.value,
                    steer: 0,
                    motor: w.motor,
                };
                drive_pdu(self.src, w.node, &d)
            }));
            self.next_drive = next_after(self.next_drive, DRIVE_PERIOD, at);
        }
        out
    }

    /// One round of stop frames: `DRIVE_CMD` neutral to each wheel, then `INPUTS` power 0 to each
    /// node. The round goes out [`STOP_REPEATS`] times.
    pub fn stop(&self) -> Vec<Vec<u8>> {
        let off = Inputs {
            buttons: 0,
            rider: 0,
        };
        let drives = self.wheels.iter().map(|w| {
            let neutral = DriveCmd {
                kind: DriveKind::Neutral,
                value: 0,
                steer: 0,
                motor: w.motor,
            };
            drive_pdu(self.src, w.node, &neutral)
        });
        let inputs = self.nodes.iter().map(|&n| inputs_pdu(self.src, n, &off));
        drives.chain(inputs).collect()
    }
}

/// The next deadline after `due` on a `period` cadence, from `at` if `at` has already passed it.
fn next_after(due: Duration, period: Duration, at: Duration) -> Duration {
    let n = due + period;
    if n > at {
        n
    } else {
        at + period
    }
}
