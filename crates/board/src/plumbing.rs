//! The plumbing (`specs/board-model.md`, slicing item 3): the fields -> validator read path, the
//! reserved-set computation, and the `BOARD_OBS` record layout. Nothing here touches the live
//! boot path; the boot wiring (and the real-chip `Capabilities` adapter over runtime-hal's R-CAP
//! queries) lives in `crates/firmware` (slicing item 4).

use crate::{BoardError, BoardErrorKind, BoardField, BoardFields, MotorFields};
use store::{Flash, Store};

/// Read the registered board-layout fields (`specs/board-model.md`, "The field vocabulary", plus
/// the per-motor carried config facts folded in by `specs/motor-integration.md`: direction,
/// align_offset, current_sense + the phase-current pins) from the store, through the registry defaults (the defaults'
/// single owner: an absent key
/// reads its registered default, so a blank board yields the benign fleet plan and absent motor
/// groups). Per-motor fields read via `Key.index`.
pub fn read_fields<F: Flash>(s: &Store<F>) -> BoardFields {
    let motor = |m: u8| MotorFields {
        hall_a: s.get(store::MOTOR_HALL_A.at(m)),
        hall_b: s.get(store::MOTOR_HALL_B.at(m)),
        hall_c: s.get(store::MOTOR_HALL_C.at(m)),
        gate_hi_a: s.get(store::MOTOR_GATE_HI_A.at(m)),
        gate_hi_b: s.get(store::MOTOR_GATE_HI_B.at(m)),
        gate_hi_c: s.get(store::MOTOR_GATE_HI_C.at(m)),
        gate_lo_a: s.get(store::MOTOR_GATE_LO_A.at(m)),
        gate_lo_b: s.get(store::MOTOR_GATE_LO_B.at(m)),
        gate_lo_c: s.get(store::MOTOR_GATE_LO_C.at(m)),
        dead_time: s.get(store::MOTOR_DEAD_TIME.at(m)),
        direction: s.get(store::MOTOR_DIRECTION.at(m)),
        align_offset: s.get(store::MOTOR_ALIGN_OFFSET.at(m)),
        current_sense: s.get(store::MOTOR_CURRENT_SENSE.at(m)),
        phase_a: s.get(store::MOTOR_PHASE_A.at(m)),
        phase_b: s.get(store::MOTOR_PHASE_B.at(m)),
    };
    BoardFields {
        self_hold: s.get(store::BOARD_SELF_HOLD),
        vbatt: s.get(store::BOARD_VBATT),
        buzzer: s.get(store::BOARD_BUZZER),
        led_green: s.get(store::LED_GREEN),
        led_orange: s.get(store::LED_ORANGE),
        led_red: s.get(store::LED_RED),
        pad_a: s.get(store::PAD_A),
        pad_b: s.get(store::PAD_B),
        button: s.get(store::BOARD_BUTTON),
        imu_scl: s.get(store::IMU_SCL_PIN),
        imu_sda: s.get(store::IMU_SDA_PIN),
        imu_model: s.get(store::IMU_MODEL),
        motors: [motor(0), motor(1)],
    }
}

/// One safe-USART allowlist entry as the FIRMWARE owns it (`specs/l3.md`; the compiled constant
/// stays the firmware's, passed in).
///
/// Two facts that used to be one field are separate here, because on this fleet they genuinely
/// differ: an entry's `LINK_SET` bit identifies **which wiring** was discovered (it is per-USART
/// and persisted forever), while `net_port` is **which `net` slot** that link occupies at runtime.
/// The onboard BLE module sits on USART2's pins on the standard family and on USART0's pins on the
/// offroad family, so those are two allowlist entries with two distinct `LINK_SET` bits and the
/// SAME `net_port`: the board's BLE slot. Collapsing them again would either lose the ability to
/// tell the two wirings apart in the persisted mask, or push a link into a `net` slot the board
/// does not have.
#[derive(Clone, Copy, Debug)]
pub struct AllowlistPort {
    /// The port's bit in the persisted `LINK_SET` mask: the identity of this WIRING, unique per
    /// allowlist entry and stable for the life of a board's config.
    pub link_set_bit: u8,
    /// The `net` port slot this entry's link occupies when it is live. Not unique across entries:
    /// two entries that are alternative wirings of the same board function share one slot.
    pub net_port: u8,
    /// The port's two pins, packed.
    pub pins: [u8; 2],
    /// Can the HAL actually bring this entry up, on THIS chip, this boot?
    ///
    /// The HAL pin model's own per-boot answer (the caller asks it: does this pin pair map to a
    /// USART instance on the detected family, and can that instance receive), never a compiled
    /// literal and never a family test written out here. It is what lets one allowlist carry both
    /// BLE wirings: the pair that this silicon cannot route is not a candidate, so the choice
    /// between them never becomes a guess.
    pub routable: bool,
}

/// The SWD pins (PA13/PA14), always reserved on every fleet part.
pub const SWD_PINS: [u8; 2] = [0x0D, 0x0E];

/// The computed reserved set [`crate::validate`] consumes (fixed capacity: SWD + up to 7
/// allowlist ports).
#[derive(Clone, Copy, Debug)]
pub struct ReservedSet {
    pins: [u8; 16],
    len: usize,
}

impl ReservedSet {
    /// The reserved pins, packed, for `validate`'s `reserved` argument.
    pub fn as_slice(&self) -> &[u8] {
        &self.pins[..self.len]
    }
}

/// Is this allowlist port one the link layer may claim its pins for on THIS boot?
///
/// The one predicate [`reserved_set`] and [`resolve_ports`] both answer through, so the pins the
/// validator refuses to a board field and the pins a bring-up actually drives are the same set by
/// construction. Two conditions:
///
/// - **`LINK_SET` says so.** `link_set == 0` means UNCONFIGURED: the board will probe every
///   allowlisted port, so every one is a claimant. A nonzero `link_set` means configured, and only
///   the ports whose bit IS set are live link ports; the clear-bit ports are freed for other
///   functions (an IMU on the freed port's pins is the motivating case, both families).
/// - **The HAL can bring it up.** A port this silicon cannot route (`routable` false) will never be
///   configured whatever the mask says, so it claims nothing. Reserving its pins anyway would
///   reserve them against a phantom: the F1x0 has no USART2 at all, so holding PB10/PB11 for it
///   would block the offroad IMU from the bus it is physically wired to, for a probe that cannot
///   happen.
fn claims_pins(port: &AllowlistPort, link_set: u8) -> bool {
    port.routable && (link_set == 0 || (link_set & (1 << port.link_set_bit)) != 0)
}

/// Compute the validator's reserved set (`specs/board-model.md`, check 3): the pins the link layer
/// may claim this boot ([`claims_pins`]), plus SWD.
pub fn reserved_set(allowlist: &[AllowlistPort], link_set: u8) -> ReservedSet {
    let mut out = ReservedSet {
        pins: [0; 16],
        len: 0,
    };
    for p in SWD_PINS {
        out.pins[out.len] = p;
        out.len += 1;
    }
    for port in allowlist {
        if claims_pins(port, link_set) {
            for p in port.pins {
                out.pins[out.len] = p;
                out.len += 1;
            }
        }
    }
    out
}

/// Which allowlist entry carries each `net` port slot's link this boot: the SINGLE owner of the
/// pin -> function assignment.
///
/// Every consumer that drives a pin at boot asks this one function, so no two of them can reach
/// different conclusions about who owns a pin. Built once, fully, before any bring-up runs. The
/// IMU needs no entry here: it is assigned the pins the validated layout staged for it, and this
/// is what guarantees no link port is handed those same pins (see [`resolve_ports`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortAssignment {
    /// Index into the caller's allowlist of the entry carrying each `net` port slot, by slot. Slot
    /// 0 is the board's mailbox (never a USART), so it is always `None`.
    entries: [Option<u8>; NET_PORTS],
}

/// The `net` port slots a board has (mailbox + the inter-board link + the BLE module). The
/// firmware's `N_PORTS`; an allowlist entry naming a slot outside it is a caller bug and is
/// dropped rather than indexed.
pub const NET_PORTS: usize = 3;

impl PortAssignment {
    /// The allowlist index carrying `net_port`'s link this boot, if any.
    pub fn port(&self, net_port: u8) -> Option<usize> {
        self.entries
            .get(net_port as usize)
            .and_then(|e| e.map(usize::from))
    }
}

/// Assign the board's pins to functions for this boot, from the persisted layout alone.
///
/// Inputs are all data: the compiled allowlist carrying the HAL's per-boot `routable` answers, the
/// persisted `LINK_SET`, and the pins the validator already accepted for the IMU. There is no chip
/// test here and no compiled board identity: which USART carries the BLE module and which bus the
/// IMU sits on are decided by what was staged, which is the whole point (the two families mirror
/// each other's wiring, so the silicon cannot be the discriminator).
///
/// Per `net` slot, the entry chosen is the first one that [`claims_pins`] this boot. At most one
/// can qualify per slot on real silicon, because the two BLE wirings are routable on opposite
/// families; taking the first is therefore a total order over a set that never has two members,
/// not a tie-break hiding a real ambiguity.
///
/// **The mutual-exclusion backstop.** A link port whose pins collide with the staged IMU pair is
/// DROPPED, and the IMU keeps its bus. The USART is the one refused because it is the refusal that
/// protects hardware: a USART drives its TX pin push-pull, so claiming a live open-drain I2C line
/// puts the port driver in contention with whatever the IMU is pulling down, while the reverse
/// (I2C's open-drain AF over a pin a USART expected) can only ever release a line. The board also
/// stays reachable through the refusal, over the inter-board link and the SWD mailbox, so the
/// contradictory staging can be corrected.
///
/// A validated layout cannot reach that state: `LINK_SET` bits mark their ports' pins reserved,
/// and the validator refuses any field claiming a reserved pin, so a plan carrying an IMU on a
/// live link port's pins never gets built. That is exactly why the rule is stated here as well and
/// tested directly: it is the property two independent layers have to agree on, and a backstop
/// that only holds because another layer got there first is not a backstop.
pub fn resolve_ports(
    allowlist: &[AllowlistPort],
    link_set: u8,
    imu_pins: Option<[u8; 2]>,
) -> PortAssignment {
    let mut out = PortAssignment {
        entries: [None; NET_PORTS],
    };
    for (i, port) in allowlist.iter().enumerate() {
        if !claims_pins(port, link_set) {
            continue;
        }
        // The backstop: never hand a USART a pin the staged IMU holds.
        if let Some(imu) = imu_pins {
            if port.pins.iter().any(|p| imu.contains(p)) {
                continue;
            }
        }
        if let Some(slot) = out.entries.get_mut(port.net_port as usize) {
            if slot.is_none() {
                *slot = Some(i as u8);
            }
        }
    }
    out
}

/// The `BOARD_OBS` magic (`"BRDV"` little-endian).
pub const BOARD_OBS_MAGIC: u32 = 0x5644_5242;

/// Result code: the layout validated.
pub const OBS_OK: u8 = 0;

/// The `BOARD_OBS` RAM record (`specs/board-model.md`, "The apply contract"): the validator
/// outcome, readable over the SWD mailbox path. This crate defines the LAYOUT and the
/// constructors; `crates/firmware` places it in RAM (the `#[no_mangle]` `BOARD_OBS` static,
/// written once at boot).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoardObs {
    /// [`BOARD_OBS_MAGIC`].
    pub magic: u32,
    /// [`OBS_OK`] on success, else the failure kind's code (1..=10, see [`BoardObs::failure`]).
    pub result: u8,
    /// The offending field's REGISTRY id (`store`'s field ids; 0 on success).
    pub field_id: u8,
    /// The offending field's `Key.index` (the motor index; 0 for singletons and on success).
    pub index: u8,
    /// Reserved pad (zero).
    pub pad: u8,
    /// Kind-specific detail: the packed pin byte (or the raw byte for a bad encoding); 0 where
    /// no pin is involved.
    pub detail: u32,
}

/// The registry id behind a [`BoardField`] (via the store handles, their single owner).
fn field_id(f: BoardField) -> u8 {
    match f {
        BoardField::SelfHold => store::BOARD_SELF_HOLD.id(),
        BoardField::Vbatt => store::BOARD_VBATT.id(),
        BoardField::Buzzer => store::BOARD_BUZZER.id(),
        BoardField::LedGreen => store::LED_GREEN.id(),
        BoardField::LedOrange => store::LED_ORANGE.id(),
        BoardField::LedRed => store::LED_RED.id(),
        BoardField::PadA => store::PAD_A.id(),
        BoardField::PadB => store::PAD_B.id(),
        BoardField::Button => store::BOARD_BUTTON.id(),
        BoardField::ImuScl => store::IMU_SCL_PIN.id(),
        BoardField::ImuSda => store::IMU_SDA_PIN.id(),
        BoardField::ImuModel => store::IMU_MODEL.id(),
        BoardField::HallA => store::MOTOR_HALL_A.id(),
        BoardField::HallB => store::MOTOR_HALL_B.id(),
        BoardField::HallC => store::MOTOR_HALL_C.id(),
        BoardField::GateHiA => store::MOTOR_GATE_HI_A.id(),
        BoardField::GateHiB => store::MOTOR_GATE_HI_B.id(),
        BoardField::GateHiC => store::MOTOR_GATE_HI_C.id(),
        BoardField::GateLoA => store::MOTOR_GATE_LO_A.id(),
        BoardField::GateLoB => store::MOTOR_GATE_LO_B.id(),
        BoardField::GateLoC => store::MOTOR_GATE_LO_C.id(),
        BoardField::DeadTime => store::MOTOR_DEAD_TIME.id(),
        BoardField::PhaseA => store::MOTOR_PHASE_A.id(),
        BoardField::PhaseB => store::MOTOR_PHASE_B.id(),
        BoardField::CurrentSense => store::MOTOR_CURRENT_SENSE.id(),
    }
}

impl BoardObs {
    /// The success record: the layout validated, the plan is in force.
    pub fn success() -> Self {
        BoardObs {
            magic: BOARD_OBS_MAGIC,
            result: OBS_OK,
            field_id: 0,
            index: 0,
            pad: 0,
            detail: 0,
        }
    }

    /// The failure record: the first validator failure, naming the offending field by its
    /// REGISTRY id + index and carrying the kind-specific detail.
    pub fn failure(err: &BoardError) -> Self {
        let (result, detail) = match err.kind {
            BoardErrorKind::BadEncoding(raw) => (1, raw as u32),
            BoardErrorKind::IncompleteGroup => (2, 0),
            BoardErrorKind::MissingDeadTime => (3, 0),
            BoardErrorKind::DuplicatePin(p) => (4, p.packed() as u32),
            BoardErrorKind::ReservedPin(p) => (5, p.packed() as u32),
            BoardErrorKind::UnknownPin(p) => (6, p.packed() as u32),
            BoardErrorKind::GateCapableMisused(p) => (7, p.packed() as u32),
            BoardErrorKind::InvalidGateSet => (8, 0),
            BoardErrorKind::NotAdcCapable(p) => (9, p.packed() as u32),
            BoardErrorKind::NotI2cPair => (10, 0),
        };
        BoardObs {
            magic: BOARD_OBS_MAGIC,
            result,
            field_id: field_id(err.field.field),
            index: err.field.motor.unwrap_or(0),
            pad: 0,
            detail,
        }
    }
}
