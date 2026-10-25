//! The console as an L3 guest (`specs/hoverboard-bridge.md`, "`watch` and `drive`"): attach with
//! `NODE_HELLO` to the node on the link, tally the `CYCLIC_STATE` frames the board emits on its
//! mailbox port (the bridge re-publishes them on every controller link), and build the `DRIVE_CMD`
//! and `INPUTS` frames `drive` sends.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use linkctl::{CyclicState, DriveCmd, DriveKind, Inputs, OP_CYCLIC_STATE, OP_DRIVE_CMD, OP_INPUTS};
use net::pdu::{Opcode, Pdu};
use net::walk::Controller;

use crate::wire::Wire;

/// Attach to the node at the other end of `wire` (the bridge, or a board's own BLE module): send
/// `NODE_HELLO` (dst `0x00`, the one peer) up to `tries` times, `within` apart, and return the guest
/// address it grants. Every other packet that arrives meanwhile is dropped.
pub fn attach<W: Wire>(wire: &mut W, tries: u32, within: Duration) -> Option<u8> {
    for _ in 0..tries {
        let mut c = Controller::new();
        let hello = c
            .next_request()
            .expect("a new controller starts with its hello");
        wire.send(&hello);
        let deadline = Instant::now() + within;
        while let Some(pkt) = wire.recv(deadline.saturating_duration_since(Instant::now())) {
            let Ok(p) = Pdu::decode(&pkt) else { continue };
            if p.known() == Some(Opcode::NodeHello) && p.dst == c.guest_addr() {
                c.on_reply(&pkt);
                if let Some(a) = c.granted() {
                    return Some(a);
                }
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
        let Ok(p) = Pdu::decode(pkt) else {
            return false;
        };
        if p.opcode != OP_CYCLIC_STATE || !(self.only.is_empty() || self.only.contains(&p.src)) {
            return false;
        }
        let Ok(s) = CyclicState::decode(p.payload) else {
            return false;
        };
        match self.sources.get_mut(&p.src) {
            Some(src) => {
                src.frames += 1;
                src.longest_gap = src.longest_gap.max(at.saturating_sub(src.last_at));
                src.last_at = at;
                src.last = s;
            }
            None => {
                self.sources.insert(
                    p.src,
                    Source {
                        frames: 1,
                        last_at: at,
                        longest_gap: Duration::ZERO,
                        last: s,
                    },
                );
            }
        }
        true
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

    /// The once-a-second line at `elapsed` into the run.
    pub fn line(&self, elapsed: Duration) -> String {
        let per: Vec<String> = self
            .sources
            .iter()
            .map(|(&a, s)| {
                format!(
                    "{a:#04x} {} frames {:.1}/s gap {:.2}s mode {} peer_demand {} battery {}",
                    s.frames,
                    self.rate(a, elapsed),
                    s.longest_gap.as_secs_f64(),
                    s.last.mode,
                    s.last.peer_demand,
                    s.last.battery
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

/// `DRIVE_CMD` period while driving (20 Hz).
pub const DRIVE_PERIOD: Duration = Duration::from_millis(50);
/// `INPUTS` period while driving (2 Hz).
pub const INPUTS_PERIOD: Duration = Duration::from_millis(500);
/// How many times each stop frame goes out.
pub const STOP_REPEATS: usize = 3;

/// `drive`'s schedule: `DRIVE_CMD` (throttle, `value`, steer 0) every [`DRIVE_PERIOD`] and
/// `INPUTS` (power 1) every [`INPUTS_PERIOD`] to each node, from `src`. Times are offsets from the
/// start of the run.
pub struct Drive {
    src: u8,
    nodes: Vec<u8>,
    value: i16,
    next_drive: Duration,
    next_inputs: Duration,
}

impl Drive {
    /// A schedule whose first frames are due at once.
    pub fn new(src: u8, nodes: Vec<u8>, value: i16) -> Self {
        Drive {
            src,
            nodes,
            value,
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
        if at >= self.next_inputs {
            let i = Inputs {
                buttons: Inputs::BUTTON_POWER,
                rider: 0,
            };
            out.extend(self.nodes.iter().map(|&n| inputs_pdu(self.src, n, &i)));
            self.next_inputs = next_after(self.next_inputs, INPUTS_PERIOD, at);
        }
        if at >= self.next_drive {
            let d = DriveCmd {
                kind: DriveKind::Throttle,
                value: self.value,
                steer: 0,
            };
            out.extend(self.nodes.iter().map(|&n| drive_pdu(self.src, n, &d)));
            self.next_drive = next_after(self.next_drive, DRIVE_PERIOD, at);
        }
        out
    }

    /// One round of stop frames: `DRIVE_CMD` neutral, then `INPUTS` power 0, to each node. The
    /// round goes out [`STOP_REPEATS`] times.
    pub fn stop(&self) -> Vec<Vec<u8>> {
        let neutral = DriveCmd {
            kind: DriveKind::Neutral,
            value: 0,
            steer: 0,
        };
        let off = Inputs {
            buttons: 0,
            rider: 0,
        };
        let drives = self.nodes.iter().map(|&n| drive_pdu(self.src, n, &neutral));
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
