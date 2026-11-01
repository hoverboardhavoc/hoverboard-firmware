//! Boards agree on the machine, and say what they run (`specs/machine-settings.md` sections 4 and
//! 5): the machine digest a board publishes, the arm gate on its configured links' peers
//! (`EV_MACHINE`, with `FLAG_PEER_SILENT` naming its cause), and the two digests of what it runs
//! against what it stores (`FLAG_REARM`, `FLAG_REBOOT`).
//!
//! The digests are the store's ([`store::Store::machine_digest`], [`store::Store::effect_digest`]);
//! this module holds them and decides from them. The firmware computes [`Digests`] at boot and
//! again on every persisted config write (`net`'s `Responder::take_config_persisted`, taken once
//! per loop pass), tells [`Machine`] when the arm-time re-read installed the stored values, and
//! hands it each peer's digest off the link it arrived on.

use linkctl::CyclicState;
use store::{Effect, Flash, Store, LINKS};

/// How many inter-board links a board can have, as an array length (`store::LINKS`).
pub const N_LINKS: usize = LINKS as usize;

/// The three digests of a board's STORED values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Digests {
    /// The machine digest: every machine field's effective value (section 4.1).
    pub machine: u16,
    /// Every arm-effect field's effective value (section 5).
    pub arm: u16,
    /// Every boot-effect field's effective value, board-owned fields excepted (section 5).
    pub boot: u16,
}

impl Digests {
    /// The digests of what `s` stores now.
    pub fn of<F: Flash>(s: &Store<F>) -> Self {
        Digests {
            machine: s.machine_digest(),
            arm: s.effect_digest(Effect::Arm),
            boot: s.effect_digest(Effect::Boot),
        }
    }
}

/// A board's machine state: its stored digests, the digests of the values it RUNS, and its
/// configured links' peers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Machine {
    stored: Digests,
    /// The arm-effect digest as of the last arm's re-read (or boot, before the first arm).
    arm_running: u16,
    /// The boot-effect digest as of boot.
    boot_running: u16,
    /// Each link's peer's last machine digest; meaningful where its `heard` bit is set.
    peer: [u16; N_LINKS],
    /// The configured links, one bit per link: those expect a peer.
    configured: u8,
    /// The links whose peer's `CYCLIC_STATE` has arrived since boot, one bit per link.
    heard: u8,
}

impl Machine {
    /// A board at boot: it runs what it stores, and each link `configured` (both its pins stored)
    /// expects a peer it has not heard yet (`specs/machine-settings.md` section 4.2).
    pub fn new(boot: Digests, configured: [bool; N_LINKS]) -> Self {
        Machine {
            stored: boot,
            arm_running: boot.arm,
            boot_running: boot.boot,
            peer: [0; N_LINKS],
            configured: configured
                .iter()
                .enumerate()
                .fold(0, |m, (i, c)| m | (u8::from(*c) << i)),
            heard: 0,
        }
    }

    /// A config write persisted: the stored digests are now `stored`.
    pub fn persisted(&mut self, stored: Digests) {
        self.stored = stored;
    }

    /// The arm-time re-read installed the stored arm-effect values: the board runs them.
    pub fn armed(&mut self) {
        self.arm_running = self.stored.arm;
    }

    /// A `CYCLIC_STATE` arrived on link `link` carrying the peer's machine digest. A frame on a
    /// link that is not configured changes nothing: no peer is expected there.
    pub fn hear(&mut self, link: usize, digest: u16) {
        if let Some(p) = self.peer.get_mut(link) {
            if self.configured & (1 << link) != 0 {
                *p = digest;
                self.heard |= 1 << link;
            }
        }
    }

    /// This board's machine digest, as `CYCLIC_STATE` carries it.
    pub fn digest(&self) -> u16 {
        self.stored.machine
    }

    /// The `EV_MACHINE` level (section 4.2): a configured link has heard no peer since boot, or
    /// its peer's last digest differs from this board's. It blocks arming and nothing else.
    pub fn disagrees(&self) -> bool {
        self.peer_silent()
            || self
                .peer
                .iter()
                .enumerate()
                .any(|(i, d)| self.heard & (1 << i) != 0 && *d != self.stored.machine)
    }

    /// The first of [`Self::disagrees`]'s causes: a configured link has heard no peer since boot
    /// (`FLAG_PEER_SILENT`, `specs/board-route.md` section 4).
    pub fn peer_silent(&self) -> bool {
        self.configured != self.heard
    }

    /// `FLAG_REARM` and `FLAG_REBOOT` (section 5): an arm-effect field written since the last
    /// arm, a boot-effect field written since boot; and `FLAG_PEER_SILENT`.
    pub fn flags(&self) -> u8 {
        let mut f = 0;
        if self.peer_silent() {
            f |= CyclicState::FLAG_PEER_SILENT;
        }
        if self.arm_running != self.stored.arm {
            f |= CyclicState::FLAG_REARM;
        }
        if self.boot_running != self.stored.boot {
            f |= CyclicState::FLAG_REBOOT;
        }
        f
    }
}
