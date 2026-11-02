//! `machine update` (`specs/firmware-bundle.md` section 4.3): every bridge and every board of a
//! machine from one bundle, each board through its own bridge's link.
//!
//! - [`plan`] pairs each board with the bridge wired to its SWD header ([`target::wired_bridge`]),
//!   reads each bridge's `ESP_INFO` and each board's `TARGET_ID`, chooses every image and runs
//!   each board image's guards and each bridge image's file check, so nothing is written unless
//!   every image is chosen. The given link's pair goes last.
//! - [`run`] takes the pairs one at a time over [`Steps`]: open the bridge's own link, update the
//!   bridge, reconnect to it after its reboot, update the board, check its status. It stops at the
//!   first failure.
//! - [`plan_lines`] and [`outcome_lines`] are what the command prints before and after.
//!
//! `main.rs` gives [`Steps`] the real links; the tests give it a simulated machine.

use std::collections::BTreeMap;

use espsvc::{EspInfo, TargetId};
use net::walk::{CLASS_BRIDGE, UID_LEN};

use crate::bundle::{BoardTarget, Bundle, ImageKind};
use crate::nodes::{uid_hex, Found, Walked, CLASS_MOTOR};
use crate::target::{self, node_name};

/// A bridge in the plan: its address and name from the walk, its commit from `ESP_INFO` (when that
/// decoded), and the image it gets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanBridge {
    pub addr: u8,
    pub name: String,
    pub commit: Option<u32>,
    pub image: String,
}

/// A board in the plan: its address, name and `uid` from the walk, the chip its `TARGET_ID`
/// reported, and the image it gets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanBoard {
    pub addr: u8,
    pub name: String,
    pub uid: [u8; UID_LEN],
    pub chip: BoardTarget,
    pub image: String,
}

/// A bridge and the board on its mailbox, if one is; `given` when the bridge is the node the given
/// link reaches.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pair {
    pub bridge: PlanBridge,
    pub board: Option<PlanBoard>,
    pub given: bool,
}

impl Pair {
    /// `<bridge> and <board>`, or the bridge alone.
    pub fn label(&self) -> String {
        match &self.board {
            Some(b) => format!("{} and {}", self.bridge.name, b.name),
            None => self.bridge.name.clone(),
        }
    }
}

/// What `machine update` will do: the bundle's commit, the pairs in the order they are taken, the
/// boards no bridge is wired to (each with why), and each chosen board image's binary.
#[derive(Debug, Clone)]
pub struct Plan {
    pub commit: String,
    pub pairs: Vec<Pair>,
    pub not_updatable: Vec<(String, String)>,
    pub bins: BTreeMap<String, Vec<u8>>,
}

/// The plan for the machine `w` walked, from `bundle` (section 4.3, step 1). `info(bridge)` reads
/// a bridge's `ESP_INFO` (`None` when it does not decode); `target_id(bridge, board)` reads a
/// board's `TARGET_ID` through the bridge wired to it. Any image not chosen, or refused by its
/// guards or its file check, and any `TARGET_ID` not read or reporting another `uid` than the
/// walk's, is an error: nothing is written.
pub fn plan(
    w: &Walked,
    bundle: &Bundle,
    mut info: impl FnMut(u8) -> Result<Option<EspInfo>, String>,
    mut target_id: impl FnMut(u8, u8) -> Result<TargetId, String>,
) -> Result<Plan, String> {
    let topo = &w.topology;
    let mut bins = BTreeMap::new();
    let mut boards: BTreeMap<u8, PlanBoard> = BTreeMap::new();
    let mut not_updatable = Vec::new();
    for f in w.found.iter().filter(|f| f.class() == Some(CLASS_MOTOR)) {
        let name = node_name(f);
        let Ok((bridge, _)) = target::wired_bridge(&w.found, topo, f.addr) else {
            not_updatable.push((name, "no bridge is wired to its SWD header".into()));
            continue;
        };
        let id = target_id(bridge, f.addr).map_err(|e| format!("{name}: TARGET_ID: {e}"))?;
        let walked_uid = f.identity.map(|i| i.uid);
        if walked_uid != Some(id.uid) {
            return Err(format!(
                "{name}: the bridge wired to it reads the uid {} over SWD, not the walk's {}",
                uid_hex(&id.uid),
                walked_uid.map_or("(none)".into(), |u| uid_hex(&u))
            ));
        }
        let (img, chip) = bundle
            .select(&id, None)
            .map_err(|e| format!("{name}: {e}"))?;
        if !bins.contains_key(&img.name) {
            let bin = bundle
                .prepare(img)
                .map_err(|e| format!("{name}: {}: {e}", img.name))?;
            bins.insert(img.name.clone(), bin);
        }
        boards.insert(
            bridge,
            PlanBoard {
                addr: f.addr,
                name,
                uid: id.uid,
                chip,
                image: img.name.clone(),
            },
        );
    }
    let mut pairs = Vec::new();
    for f in w.found.iter().filter(|f| f.class() == Some(CLASS_BRIDGE)) {
        let name = node_name(f);
        let i = info(f.addr).map_err(|e| format!("{name}: ESP_INFO: {e}"))?;
        let img = bundle
            .select_bridge(ImageKind::Bridge, i.map(|i| i.chip))
            .map_err(|e| format!("{name}: {e}"))?;
        bundle
            .bridge_file(img)
            .map_err(|e| format!("{name}: {e}"))?;
        pairs.push(Pair {
            bridge: PlanBridge {
                addr: f.addr,
                name,
                commit: i.map(|i| i.commit),
                image: img.name.clone(),
            },
            board: boards.remove(&f.addr),
            given: topo.gateway == Some(f.addr),
        });
    }
    // The given link's pair last (step 2); the others in address order.
    pairs.sort_by_key(|p| p.given);
    for f in w.found.iter().filter(|f| !is_known(f)) {
        not_updatable.push((node_name(f), "it did not answer IDENTIFY".into()));
    }
    Ok(Plan {
        commit: bundle.manifest.commit.clone(),
        pairs,
        not_updatable,
        bins,
    })
}

fn is_known(f: &Found) -> bool {
    matches!(f.class(), Some(CLASS_MOTOR) | Some(CLASS_BRIDGE))
}

/// The first eight hex digits of a commit, as `ESP_INFO` reports a bridge's.
fn short(commit: &str) -> &str {
    commit.get(..8).unwrap_or(commit)
}

/// The plan, one line per node, then the order (section 4.3, step 1).
pub fn plan_lines(p: &Plan) -> Vec<String> {
    let mut out = vec![format!("bundle built from commit {}", p.commit)];
    for pair in &p.pairs {
        let b = &pair.bridge;
        let now = b
            .commit
            .map_or("commit unknown".into(), |c| format!("{c:08x}"));
        let via = if pair.given {
            "the given link"
        } else {
            "its own link"
        };
        out.push(format!(
            "bridge {} ({:#04x}): {now} -> {} ({}), over {via}",
            b.name,
            b.addr,
            short(&p.commit),
            b.image
        ));
        match &pair.board {
            Some(d) => out.push(format!(
                "board  {} ({:#04x}): {}, gets {}, flashed by {}",
                d.name, d.addr, d.chip, d.image, b.name
            )),
            None => out.push(format!(
                "        no board on {}'s mailbox: the bridge is updated alone",
                b.name
            )),
        }
    }
    for (name, why) in &p.not_updatable {
        out.push(format!("board  {name}: not updatable: {why}"));
    }
    let order: Vec<String> = p.pairs.iter().map(Pair::label).collect();
    out.push(format!("order: {}", order.join(", then ")));
    out
}

/// How one pair ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// The bridge and its board are updated, the board booted and its status is ok.
    Done,
    /// The bridge's own link did not open; nothing of the pair was started.
    Unreachable(String),
    /// The bridge's update failed or it did not come back; its board is untouched.
    BridgeFailed(String),
    /// The bridge is updated; the board's update failed.
    BoardFailed(String),
    /// The board booted, and its boot report is not ok.
    StatusFailed(String),
    /// An earlier pair failed.
    NotStarted,
}

impl Outcome {
    fn failed(&self) -> bool {
        !matches!(self, Outcome::Done | Outcome::NotStarted)
    }
}

/// The device operations of one pair. `open` gives the bridge's own link (the given link for the
/// given link's pair), `rejoin` the same link once the bridge has rebooted into its update.
pub trait Steps {
    type Link;
    /// Open the link to the pair's bridge, checking that it reaches that bridge.
    fn open(&mut self, pair: &Pair) -> Result<Self::Link, String>;
    /// Update the bridge: write, reboot, reconnect and confirm.
    fn bridge(&mut self, link: &Self::Link, pair: &Pair) -> Result<(), String>;
    /// The link to the bridge again, after its reboot: the drop is not a failure.
    fn rejoin(&mut self, pair: &Pair) -> Result<Self::Link, String>;
    /// Flash the board through the bridge on `link`, until it booted.
    fn board(&mut self, link: &Self::Link, board: &PlanBoard) -> Result<(), String>;
    /// The board's boot report: `Err` with its lines when it is not ok.
    fn status(&mut self, link: &Self::Link, board: &PlanBoard) -> Result<(), String>;
}

/// Take the plan's pairs in order (section 4.3, steps 2 to 4), stopping at the first failure: one
/// outcome per pair.
pub fn run<S: Steps>(p: &Plan, steps: &mut S) -> Vec<Outcome> {
    let mut out = Vec::new();
    for pair in &p.pairs {
        if out.iter().any(Outcome::failed) {
            out.push(Outcome::NotStarted);
            continue;
        }
        out.push(run_pair(pair, steps));
    }
    out
}

fn run_pair<S: Steps>(pair: &Pair, steps: &mut S) -> Outcome {
    let link = match steps.open(pair) {
        Ok(l) => l,
        Err(e) => return Outcome::Unreachable(e),
    };
    if let Err(e) = steps.bridge(&link, pair) {
        return Outcome::BridgeFailed(e);
    }
    drop(link);
    let link = match steps.rejoin(pair) {
        Ok(l) => l,
        Err(e) => return Outcome::BridgeFailed(e),
    };
    let Some(board) = &pair.board else {
        return Outcome::Done;
    };
    if let Err(e) = steps.board(&link, board) {
        return Outcome::BoardFailed(e);
    }
    match steps.status(&link, board) {
        Ok(()) => Outcome::Done,
        Err(e) => Outcome::StatusFailed(e),
    }
}

/// Whether every pair is done.
pub fn all_done(outcomes: &[Outcome]) -> bool {
    outcomes.iter().all(|o| *o == Outcome::Done)
}

/// One line per pair: done, failed (and what that leaves), or not started (section 4.3, step 4).
pub fn outcome_lines(p: &Plan, outcomes: &[Outcome]) -> Vec<String> {
    p.pairs
        .iter()
        .zip(outcomes)
        .map(|(pair, o)| {
            let b = &pair.bridge.name;
            let board = pair.board.as_ref().map(|d| d.name.as_str());
            let untouched = board.map_or(String::new(), |d| format!("; {d} untouched"));
            match o {
                Outcome::Done => format!("{}: done", pair.label()),
                Outcome::Unreachable(e) => {
                    format!("{}: not started: {b} is not reachable ({e})", pair.label())
                }
                Outcome::BridgeFailed(e) => format!(
                    "{}: FAILED updating {b} ({e}); until it is confirmed it returns to its \
                     previous slot at a reset{untouched}",
                    pair.label()
                ),
                Outcome::BoardFailed(e) => format!(
                    "{}: {b} done; FAILED updating {} ({e}); it stays reachable through {b} for \
                     a retry",
                    pair.label(),
                    board.unwrap_or("its board")
                ),
                Outcome::StatusFailed(e) => format!(
                    "{}: {b} done, {} booted; its boot report is not ok: {e}",
                    pair.label(),
                    board.unwrap_or("its board")
                ),
                Outcome::NotStarted => format!("{}: not started", pair.label()),
            }
        })
        .collect()
}
