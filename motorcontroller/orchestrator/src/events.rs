//! Per-producer transition counters for the mode machine's gating levels: the **O1 attribution
//! instrument** (`specs/silicon-queue.md`, "Comms-loss halves"; the arm session's open
//! observation O1).
//!
//! # What O1 is
//!
//! The 2026-07-31 arm session saw the master perform unexplained SHUTDOWN/re-arm cycles while
//! armed-idle at zero demand (`enact_inits`/`enact_shutdowns` stepping 1/0 -> 2/1 -> 3/2 with
//! nothing latched and every recovery clean). The enact counters record THAT it happened; they
//! carry no attribution, because every producer that can drive the cycle folds into one of two
//! aggregate bits (`fault_a`/`fault_b`) before the mode machine ever sees it, and a level that
//! asserts and releases inside one 4 ms tick leaves no trace in a level read at all.
//!
//! So each producer gets its own counter, stepped on every CHANGE of its level. A transient that
//! is gone before the next bench read still shows up, and it shows up attributed.
//!
//! # Why it ships
//!
//! O1 hunts a RARE transient (three occurrences across a whole session). An instrument that only
//! exists in a diagnostic build cannot catch it: the soaks that would catch it run the real image.
//! So this is in the shipping image, and it is sized to be: eight bytes of counters, two bytes of
//! previous levels, and a per-tick cost of an XOR against the previous mask plus a branch that is
//! not taken on any tick where nothing changed (which is every tick but the interesting ones).
//!
//! # What is counted, and why these eight
//!
//! Exactly the levels that can start or end an arm cycle, one bit each
//! ([`EV_COMMS_LOSS`] .. [`EV_POWER_REQUEST`]):
//!
//! - the five `fault_a` inputs the orchestrator folds (`specs/sensing-and-safety.md`): motor 0's
//!   fault latch, `comms_loss`, the tilt cutout, `imu_loss`, and the motor-side fault level;
//! - the one `fault_b` input: motor 1's fault latch;
//! - `mode_fault`, the control dispatch's demotion level (Balance requested without an IMU);
//! - `power_request`, whose FALL is the other way a running system reaches SHUTDOWN
//!   (`state::ModeMachine`: RUN leaves on `fault_a || fault_b || !power_request`).
//!
//! `off_inhibit` (the wheel-motion level) is deliberately NOT here: it gates OFF -> INIT only and
//! is never consulted in RUN, so it can delay a re-arm but cannot produce the cycle O1 is.
//!
//! The motor-side level is four causes on bits 8 to 11 of the `u16` level mask ([`EV_MOTOR_HALL`]
//! .. [`EV_MOTOR_ARM`]); bit 3, which carried their union as `EV_MOTOR_FAULT`, is retired and not
//! reused (`specs/link-control.md`, "`fault` widens to u16"). The counters stay eight: counter 3
//! counts transitions of the motor union ([`counted`]), so `CTRL_OBS`'s level byte and its eight
//! counts keep their layout, and the four-way split is read off the cyclic `fault` word.
//!
//! A further level, the rigid frame's `peer_down` ([`Levels::peer_down`]), SHARES
//! [`EV_COMMS_LOSS`]'s bit, because the two levels mean the same thing to the mode machine. The peer's own `fault` word in the
//! telemetry says which of the two it was.
//!
//! # Reading them
//!
//! Both halves are published in `CTRL_OBS` (`specs/integration.md`, "Observation"): the live mask
//! in the `flags` word's former pad byte, the eight counts in two appended words. A count that is
//! EVEN with its level clear is a blip that came and went; ODD means the level is still held (or
//! was, at the tear). Counts saturate at 255 rather than wrapping, so a saturated counter reads as
//! "at least 255", never as a small number.

/// `comms_loss`: peer staleness past `CYCLIC_TIMEOUT_TICKS` (`fault_a`).
pub const EV_COMMS_LOSS: u16 = 1 << 0;
/// This board is latched STOPPED (`fault_a`). One producer: the run-time tilt cutout
/// ([`Levels::tilt`], `specs/control.md` (l)).
pub const EV_STOP_ALL: u16 = 1 << 1;
/// `imu_loss`: `IMU_LOSS_THRESHOLD` consecutive failed reads on a configured IMU (`fault_a`).
pub const EV_IMU_LOSS: u16 = 1 << 2;
/// Motor 0's fault latch (`fault_a`).
pub const EV_LATCH_A: u16 = 1 << 4;
/// Motor 1's fault latch (the sole `fault_b` input).
pub const EV_LATCH_B: u16 = 1 << 5;
/// `mode_fault`: the control dispatch demoted the requested mode at the validation seam.
pub const EV_MODE_FAULT: u16 = 1 << 6;
/// The power-request level (button OR the remote mirror). Its FALL drives RUN -> SHUTDOWN.
pub const EV_POWER_REQUEST: u16 = 1 << 7;
/// Motor-side cause: hall dwell (`motor::FAULT_HALL` in the motor FAULT word) (`fault_a`).
pub const EV_MOTOR_HALL: u16 = 1 << 8;
/// Motor-side cause: the current-sense calibration was refused at init (`motor::FAULT_INIT_CAL`)
/// (`fault_a`).
pub const EV_MOTOR_CAL: u16 = 1 << 9;
/// Motor-side cause: period-liveness loss (`motor::PeriodHealth`) (`fault_a`).
pub const EV_MOTOR_PERIOD: u16 = 1 << 10;
/// Motor-side cause: an arm attempt aborted before MOE and is held (`crate::arm` in the firmware)
/// (`fault_a`).
pub const EV_MOTOR_ARM: u16 = 1 << 11;
/// The four motor-side causes: the set `motor::motor_fault_level` returns.
pub const EV_MOTOR_BITS: u16 = EV_MOTOR_HALL | EV_MOTOR_CAL | EV_MOTOR_PERIOD | EV_MOTOR_ARM;

/// The number of transition counters. Counters 0..=7 count bits 0..=7 of the level mask, except
/// counter 3, which counts the motor union ([`counted`]).
pub const N_EVENT_PRODUCERS: usize = 8;

/// The counter slot the motor union is counted in: bit 3, the retired `EV_MOTOR_FAULT` bit. Only
/// the `CTRL_OBS` byte and the counters use it; the wire keeps bit 3 zero.
pub const COUNTED_MOTOR: u8 = 1 << 3;

/// The level mask folded into the eight-counter domain `CTRL_OBS` publishes: bits 0..=7 as they
/// are (bit 3 is always clear in a level mask), and the motor causes onto [`COUNTED_MOTOR`].
pub const fn counted(mask: u16) -> u8 {
    let low = (mask & 0xFF) as u8;
    if mask & EV_MOTOR_BITS != 0 {
        low | COUNTED_MOTOR
    } else {
        low
    }
}

/// The `fault_a` group as a mask over [`Levels::mask`]: the producers whose level the mode
/// machine folds into Fault A (`specs/sensing-and-safety.md`). [`Levels::fault_a`] is this mask
/// applied, so the aggregate the machine consumes and the mask a peer reads out of
/// `CYCLIC_STATE.fault` (`specs/rover-one-imu.md` section 4) cannot name different groups.
/// [`EV_MODE_FAULT`] and [`EV_POWER_REQUEST`] are in the level mask and are not faults, which
/// this constant states by leaving them out rather than by a comment at each consumer.
pub const FAULT_A_BITS: u16 =
    EV_COMMS_LOSS | EV_STOP_ALL | EV_IMU_LOSS | EV_LATCH_A | EV_MOTOR_BITS;

/// One tick's gating-producer levels, named.
///
/// The pipeline reads each producer exactly once, fills this in, and then uses it twice: folded
/// into the `fault_a`/`fault_b` aggregates the mode machine consumes, and folded into
/// [`mask`](Self::mask) for the counters. That is what keeps the instrument from being able to
/// disagree with the decision it explains, and it is why the bit assignment lives here beside the
/// `EV_*` constants rather than at the call site, where a producer could be put on the wrong bit.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Levels {
    /// [`EV_COMMS_LOSS`].
    pub comms_loss: bool,
    /// [`EV_IMU_LOSS`].
    pub imu_loss: bool,
    /// The motor-side causes, a subset of [`EV_MOTOR_BITS`] (`motor::motor_fault_level`).
    pub motor: u16,
    /// [`EV_LATCH_A`].
    pub latch_a: bool,
    /// [`EV_LATCH_B`].
    pub latch_b: bool,
    /// [`EV_MODE_FAULT`].
    pub mode_fault: bool,
    /// [`EV_POWER_REQUEST`].
    pub power_request: bool,
    /// `peer_down`, the rigid frame's one-wheel-fault interlock (`specs/rover-one-imu.md`
    /// section 6): a fresh peer cyclic whose own level mask carries a [`FAULT_A_BITS`] producer.
    /// It has NO bit of its own; it rides [`EV_COMMS_LOSS`] in
    /// [`mask`](Self::mask), which already means "the peer is not available to me" for the other
    /// reason that can be true. It is deliberately absent from [`own_mask`](Self::own_mask), and
    /// that is the whole difference between the two.
    pub peer_down: bool,
    /// The run-time tilt cutout's latch (`specs/control.md` (l)): this board is past the tilt
    /// limit on pitch or roll and has stopped itself. It is [`EV_STOP_ALL`]'s one producer, that
    /// bit's meaning being "this board is latched stopped".
    ///
    /// Unlike `peer_down` it IS in [`own_mask`](Self::own_mask), and must be: a rigid follower
    /// takes its demand off the wire and has no IMU, so the leader's fall reaches it only as a
    /// [`FAULT_A_BITS`] producer in this byte, which is what its `peer_down` is derived from.
    /// There is no echo hazard in that direction, because the follower's own `peer_down` stays
    /// out of its own mask, so nothing comes back.
    pub tilt: bool,
}

impl Levels {
    /// This board's OWN producers, packed: the mask a peer reads out of `CYCLIC_STATE.fault`
    /// (`specs/rover-one-imu.md` section 4).
    ///
    /// **`peer_down` is excluded, and the exclusion is what keeps the interlock from latching.**
    /// A peer derives its `peer_down` from this byte. Were the level echoed back into it, one
    /// board's motor fault would set the other's `peer_down`, whose bit would set the first
    /// board's `peer_down`, and neither board could clear: both would report [`EV_COMMS_LOSS`]
    /// forever, `fault_a` would stay asserted on both, and `fault_a` gates OFF -> INIT, so the
    /// machine would refuse to arm until it was power-cycled. The byte therefore states what
    /// THIS board's own producers say, which is the only thing a peer can act on.
    pub fn own_mask(&self) -> u16 {
        let bit = |set: bool, ev: u16| if set { ev } else { 0 };
        bit(self.comms_loss, EV_COMMS_LOSS)
            | bit(self.imu_loss, EV_IMU_LOSS)
            | (self.motor & EV_MOTOR_BITS)
            | bit(self.latch_a, EV_LATCH_A)
            | bit(self.latch_b, EV_LATCH_B)
            | bit(self.mode_fault, EV_MODE_FAULT)
            | bit(self.power_request, EV_POWER_REQUEST)
            | bit(self.tilt, EV_STOP_ALL)
    }

    /// The GATING mask this tick: [`own_mask`](Self::own_mask) with [`peer_down`](Self::peer_down)
    /// folded onto [`EV_COMMS_LOSS`]. What the mode machine's aggregates are taken from, what the
    /// counters count (through [`counted`]), and what `CTRL_OBS` publishes as the live level byte,
    /// so the decision, its attribution and the bench's reading of it are one word.
    pub fn mask(&self) -> u16 {
        self.own_mask() | if self.peer_down { EV_COMMS_LOSS } else { 0 }
    }

    /// The `fault_a` aggregate the mode machine consumes: the general/sensing fault group
    /// (`specs/sensing-and-safety.md`).
    pub fn fault_a(&self) -> bool {
        self.mask() & FAULT_A_BITS != 0
    }

    /// The `fault_b` aggregate: motor 1's latch, the second producer group.
    pub fn fault_b(&self) -> bool {
        self.latch_b
    }
}

/// The per-producer transition counters plus the previous tick's level mask.
///
/// Counts are saturating `u8`. At the observed O1 cadence (roughly one transient per 30 s) 255
/// covers about two hours of soak, and a producer that saturates has already answered the
/// attribution question by being the one that saturated.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultEvents {
    counts: [u8; N_EVENT_PRODUCERS],
    prev: u16,
}

impl FaultEvents {
    /// Fold one tick's producer levels in, counting every producer whose level CHANGED.
    ///
    /// Transitions, not assertions: a blip therefore scores 2 (the assert and the release), and a
    /// level still held at the read scores odd. Counting both edges is what makes `power_request`
    /// (whose FALL is the interesting edge) and `comms_loss` (whose RISE is) read the same way.
    ///
    /// `prev` starts at 0, which is not a seeded baseline but the levels' own cold-boot value: a
    /// producer asserted on the very first tick counts one transition, and that is a real
    /// assertion, not an artifact of the instrument. (A peer-configured board does NOT come up
    /// in `comms_loss`: the never-seen-a-peer exemption holds it clear until a peer has been
    /// seen once; the audit corrected this doc's original example, and the bench's zero counts
    /// at boot confirm.)
    pub fn tick(&mut self, levels: u16) {
        let changed = counted(levels) ^ counted(self.prev);
        self.prev = levels;
        if changed != 0 {
            for (i, c) in self.counts.iter_mut().enumerate() {
                if changed & (1 << i) != 0 {
                    *c = c.saturating_add(1);
                }
            }
        }
    }

    /// The live level mask as of the last [`tick`](Self::tick) (the `EV_*` bits).
    pub fn levels(&self) -> u16 {
        self.prev
    }

    /// The eight saturating transition counts, indexed by counter slot ([`counted`]).
    pub fn counts(&self) -> [u8; N_EVENT_PRODUCERS] {
        self.counts
    }

    /// One producer's transition count, selected by its `EV_*` bit (a motor cause selects the
    /// motor union's counter). Returns 0 for a mask with no bit set; a mask with several set
    /// selects the lowest slot.
    pub fn count(&self, ev: u16) -> u8 {
        match counted(ev).trailing_zeros() as usize {
            i if i < N_EVENT_PRODUCERS => self.counts[i],
            _ => 0,
        }
    }
}
