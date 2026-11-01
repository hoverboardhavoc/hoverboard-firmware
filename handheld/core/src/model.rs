//! The handheld as one hardware-free state machine (`specs/handheld-remote.md` sections 3 to 5):
//! the link's phase, the session, the inbox, the arm, the touch as the sampler last saw it, the
//! latch holds and the lamp test, and from them every widget's state ([`View`]).
//!
//! The binary's tasks call into one [`Model`] behind a lock: the link task with what the radio
//! delivers and the time, the sampler with the touch and the button, the view task for a
//! [`View`] to draw.

use core::fmt::Write;

use heapless::{String, Vec};
use linkctl::{CyclicState, ARM_BLOCKING_BITS};

use crate::arm::{Arm, Conditions, Dropped, Edge, Refused, BOARDS_OFF_MS};
use crate::drive::{self, Board, Refusal, Targets};
use crate::inbox::Inbox;
use crate::lamps::{self, Lamp, LampFacts, LatchHold};
use crate::pump::{Command, Outbox, Pump};
use crate::readings::attitude::Attitude;
use crate::readings::battery::{self, BatteryReading};
use crate::readings::current::{self, CurrentReading};
use crate::readings::missing;
use crate::readings::slots::{self, RideSlot, RideSlots};
use crate::readings::speed::{self, SpeedFacts, SpeedReading};
use crate::session::{Fact, Phase, Sends, Session, CLASS_MOTOR};
use crate::throttle;
use crate::touch::TouchView;

/// How long the lamp test runs after the first ride facts read, ms (section 5.6).
pub const LAMP_TEST_MS: u32 = 2000;

/// The link's state.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Link {
    /// No `HANDHELD_BRIDGE` in the build: never connects.
    NoBridge,
    /// Connecting to the pinned bridge.
    Connecting,
    /// Connected since `at`.
    Connected { at: u32 },
}

/// What the sampler reports alongside the touch, for the debug line.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct TouchDebug {
    /// A short name of the lift state: `I`dle, `P`ressing, `D`own, `L`ifting.
    pub state: u8,
    pub z: i32,
    pub x: (u16, u16),
    pub y: (u16, u16),
    /// Samples a second.
    pub rate: u16,
}

/// Everything the handheld knows.
pub struct Model {
    pub bridge: Option<[u8; 6]>,
    pub link: Link,
    pub session: Session,
    pub inbox: Inbox,
    pub arm: Arm,
    /// The arm targets latched at the arm (the app's `armedTargets`).
    armed_targets: Targets,
    pub pump: Pump,
    pub outbox: Outbox,
    pub button_fault: bool,
    pub touch: TouchView,
    pub touch_debug: TouchDebug,
    pub latch: LatchHold,
    lamp_test_until: Option<u32>,
    lamp_tested: bool,
    /// Since when no motor controller has been in the inbox while connected.
    boards_quiet_since: Option<u32>,
    /// Frames a second received and sent, and the free heap, for the debug line.
    pub rx_rate: u16,
    pub tx_rate: u16,
    pub heap_free: u32,
    last_cmd: Command,
    released: bool,
}

/// The throttle overlay's state.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Overlay {
    pub y0: i32,
    pub yf: i32,
    pub live: bool,
    pub percent: i8,
}

impl Model {
    /// A model for the pinned `bridge` (`None`: no bridge set).
    pub fn new(bridge: Option<[u8; 6]>) -> Model {
        Model {
            bridge,
            link: if bridge.is_some() {
                Link::Connecting
            } else {
                Link::NoBridge
            },
            session: Session::new(),
            inbox: Inbox::default(),
            arm: Arm::default(),
            armed_targets: Targets::default(),
            pump: Pump::default(),
            outbox: Outbox::default(),
            button_fault: false,
            touch: TouchView {
                finger_down: false,
                drive: None,
                overlay: None,
            },
            touch_debug: TouchDebug::default(),
            latch: LatchHold::new(),
            lamp_test_until: None,
            lamp_tested: false,
            boards_quiet_since: None,
            rx_rate: 0,
            tx_rate: 0,
            heap_free: 0,
            last_cmd: Command::default(),
            released: false,
        }
    }

    /// The link came up at `now`: a new session, nothing known.
    pub fn connected(&mut self, now: u32) {
        self.link = Link::Connected { at: now };
        self.session = Session::new();
        self.inbox.clear();
        self.latch = LatchHold::new();
        self.lamp_test_until = None;
        self.lamp_tested = false;
        self.boards_quiet_since = Some(now);
        self.pump.reset();
        self.outbox = Outbox::default();
    }

    /// The link went away: disarm locally, forget the machine.
    pub fn disconnected(&mut self) {
        self.arm.drop_for(Dropped::Link);
        self.link = Link::Connecting;
        self.inbox.clear();
        self.pump.reset();
        self.outbox = Outbox::default();
    }

    /// One received PDU: a reply the session takes, or telemetry for the inbox.
    pub fn on_pdu(&mut self, frame: &[u8], now: u32, sends: &mut Sends) {
        if self.session.on_pdu(frame, now, sends) {
            return;
        }
        let Ok(p) = net::pdu::Pdu::decode(frame) else {
            return;
        };
        if p.opcode == linkctl::OP_CYCLIC_STATE {
            if let Ok(s) = CyclicState::decode(p.payload) {
                self.latch.observe(p.src, s.fault);
                self.inbox.cyclic(p.src, s, now);
            }
        }
    }

    /// Whether `addr` counts as a motor controller: its class reads motor controller, or is not
    /// read yet (a possible one), and it is not a bridge.
    fn possible_mc(&self, addr: u8) -> bool {
        match self.session.nodes().iter().find(|n| n.addr == addr) {
            Some(n) => n.class.is_none() || n.class == Some(CLASS_MOTOR),
            None => true,
        }
    }

    /// Whether a motor controller is in the inbox.
    pub fn mc_heard(&self) -> bool {
        self.inbox.sources().any(|h| self.possible_mc(h.src))
    }

    /// Run the clocks at `now`: the session's requests and re-walks, the inbox's expiry, the
    /// lamp test, and every way the arm drops by itself (section 4.7).
    pub fn poll(&mut self, now: u32, sends: &mut Sends) {
        self.inbox.expire(now);
        let Link::Connected { .. } = self.link else {
            return;
        };
        let heard = self.mc_heard();
        self.session.poll(now, self.arm.armed(), heard, sends);
        if self.session.ready() && !self.lamp_tested {
            self.lamp_tested = true;
            self.lamp_test_until = Some(now.wrapping_add(LAMP_TEST_MS));
        }
        if self
            .lamp_test_until
            .is_some_and(|t| now.wrapping_sub(t) as i32 >= 0)
        {
            self.lamp_test_until = None;
        }
        if heard {
            self.boards_quiet_since = None;
        } else {
            let since = *self.boards_quiet_since.get_or_insert(now);
            if now.wrapping_sub(since) >= BOARDS_OFF_MS {
                self.arm.drop_for(Dropped::BoardsOff);
            }
        }
        if self.arm.armed() {
            for &b in self.armed_targets.arm.iter() {
                let fault = self.inbox.get(b).map_or(0, |s| s.fault);
                if fault & ARM_BLOCKING_BITS != 0 {
                    let bit = 1 << fault.trailing_zeros();
                    self.arm.drop_for(Dropped::Fault { bit, board: b });
                    break;
                }
            }
        }
    }

    /// Whether the motor controllers have been without a frame in the inbox for 1 s while
    /// connected: the motor-controller-off card.
    fn boards_off(&self, now: u32) -> bool {
        self.boards_quiet_since
            .is_some_and(|s| now.wrapping_sub(s) >= BOARDS_OFF_MS)
    }

    /// The motor controllers as the drive rule reads them, by address.
    fn boards(&self) -> Vec<Board, 4> {
        self.session
            .facts()
            .iter()
            .map(|f| Board {
                addr: f.addr,
                mode: f.get(Fact::DriveMode).map(|v| v as u8),
                side: f.get(Fact::Side).map(|v| v as u8),
                leader: self.inbox.get(f.addr).is_some_and(|s| s.leader()),
            })
            .collect()
    }

    /// The drive in force (section 4.6).
    pub fn drive(&self) -> Result<Targets, Refusal> {
        drive::targets(&self.boards(), self.session.class_unread())
    }

    /// The motor controllers store different frames (the app's `MachineArm.FramesDiffer`).
    fn frames_differ(&self) -> bool {
        let mut frames = self
            .session
            .facts()
            .iter()
            .filter_map(|f| f.get(Fact::Frame));
        let Some(first) = frames.next() else {
            return false;
        };
        frames.any(|f| f != first)
    }

    /// What the arm rule reads now (section 4.5).
    pub fn conditions(&self) -> Conditions {
        let drive = self.drive();
        let blocked = drive.as_ref().is_ok_and(|t| {
            t.arm.iter().any(|&b| {
                self.inbox
                    .get(b)
                    .is_some_and(|s| s.fault & ARM_BLOCKING_BITS != 0)
            })
        });
        Conditions {
            button_fault: self.button_fault,
            finger_down: self.touch.finger_down,
            ready: matches!(self.link, Link::Connected { .. }) && self.session.ready(),
            frames_differ: self.frames_differ(),
            drive: drive.as_ref().err().copied(),
            board: self.mc_heard() && drive.as_ref().is_ok_and(|t| !t.arm.is_empty()),
            blocked,
        }
    }

    /// A BOOT edge (section 4.5). A press clears the latch holds; an arm latches its targets.
    pub fn on_button(&mut self, e: Edge) {
        match e {
            Edge::Fault => self.button_fault = true,
            Edge::Release { .. } | Edge::Press => {}
        }
        if let Edge::Press = e {
            self.latch.clear_on_boot_press();
        }
        let cond = self.conditions();
        if let Some(crate::arm::Action::Armed) = self.arm.on_edge(e, || cond) {
            self.armed_targets = self.drive().unwrap_or_default();
            self.lamp_test_until = None;
        }
        if let Edge::Release { .. } = e {
            self.button_fault = false;
        }
    }

    /// The button has been high long enough to end a fault.
    pub fn button_released(&mut self) {
        self.button_fault = false;
    }

    /// The command the pump sends now: the arm level, and the throttle's demand while armed and
    /// the touch is `Down`.
    pub fn command(&self) -> Command {
        let armed = self.arm.armed();
        let demand = match (armed, self.touch.drive) {
            (true, Some((y0, yf))) => throttle::demand(y0, yf),
            _ => 0,
        };
        Command { armed, demand }
    }

    /// Run the pump at `now` (section 3.6). Returns whether a tick went out, so the caller sends at
    /// once on a release.
    pub fn run_pump(&mut self, now: u32) -> bool {
        let Link::Connected { .. } = self.link else {
            return false;
        };
        if self.session.src().is_none() {
            return false;
        }
        let targets = if self.arm.armed() {
            self.armed_targets.clone()
        } else {
            self.drive().unwrap_or_default()
        };
        let cmd = self.command();
        let release = (self.last_cmd.armed && !cmd.armed)
            || (self.last_cmd.armed && self.last_cmd.demand != 0 && cmd.demand == 0);
        self.last_cmd = cmd;
        let sent = self.pump.poll(now, cmd, &targets, &mut self.outbox);
        if sent && release {
            self.released = true;
        }
        sent
    }

    /// Whether the last tick that went out was a release send, and clear it (the binary logs the
    /// time from the first Up sample to the frames handed to the BLE host, section 7.2 check 6).
    pub fn take_release(&mut self) -> bool {
        core::mem::take(&mut self.released)
    }

    /// The throttle overlay (section 5.7): while armed and the touch is `Down` or `Lifting`, the
    /// zero line at `y0` and the thumb at `yf`, in pixels, `live` in `Down`, and the demand as a
    /// signed percent.
    pub fn overlay(&self) -> Option<Overlay> {
        if !self.arm.armed() {
            return None;
        }
        let ((y0, yf), live) = self.touch.overlay?;
        let demand = if live {
            i32::from(throttle::demand(y0, yf))
        } else {
            0
        };
        Some(Overlay {
            y0: y0 / crate::touch::SUB,
            yf: yf / crate::touch::SUB,
            live,
            percent: ((demand * 100 + demand.signum() * throttle::FULL_SCALE / 2)
                / throttle::FULL_SCALE) as i8,
        })
    }

    /// The state of every widget at `now`.
    pub fn view(&self, now: u32) -> View {
        let slots = self.slots();
        let top = self.top(&slots, now);
        let body = self.body(&slots, now);
        let line = match &body {
            Body::Ride(r) if matches!(r.lower, Lower::Grid(_)) => None,
            Body::Card(Card::MotorOff) | Body::Ride(_) => self.statement(&slots),
            Body::Card(_) => None,
        };
        View {
            top,
            body,
            line,
            armed: self.arm.armed(),
        }
    }

    fn mcs(&self) -> Vec<u8, 4> {
        self.session.motor_controllers().take(4).collect()
    }

    fn fact(&self, addr: u8, f: Fact) -> Option<i64> {
        self.session.facts_of(addr).and_then(|x| x.get(f))
    }

    fn slots(&self) -> RideSlots {
        let mcs = self.mcs();
        slots::ride_slots(&mcs, |a| self.fact(a, Fact::Side).map(|v| v as u8))
    }

    fn reporting(&self) -> Vec<u8, 4> {
        let mut v: Vec<u8, 4> = self
            .mcs()
            .iter()
            .copied()
            .filter(|&a| self.inbox.get(a).is_some())
            .collect();
        v.sort_unstable();
        v
    }

    fn battery_of(&self, a: u8) -> BatteryReading {
        battery::battery_reading(
            self.inbox.get(a),
            self.fact(a, Fact::Cells).map(|v| v as u8),
            self.fact(a, Fact::Chemistry).map(|v| v as u8),
        )
    }

    fn top(&self, slots: &RideSlots, now: u32) -> Top {
        let dot = |s: Option<RideSlot>| {
            s.map(|s| Dot {
                label: s.label(),
                heard: self.inbox.get(s.address).is_some(),
            })
        };
        let (ring_board, ring) = battery::ring_reading(&self.reporting(), |a| self.battery_of(a));
        let floor = ring_board.and_then(|b| self.fact(b, Fact::Floor).map(|v| v as i16));
        Top {
            dots: [dot(slots.left), dot(slots.right)],
            low: battery::battery_low(&ring, floor),
            test: self.lamp_test(now),
        }
    }

    fn lamp_test(&self, now: u32) -> bool {
        self.lamp_test_until
            .is_some_and(|t| (now.wrapping_sub(t) as i32) < 0)
    }

    fn body(&self, slots: &RideSlots, now: u32) -> Body {
        match self.link {
            Link::NoBridge => return Body::Card(Card::NoBridge),
            Link::Connecting => return Body::Card(Card::Connecting(self.bridge.unwrap_or([0; 6]))),
            Link::Connected { .. } => {}
        }
        match self.session.phase() {
            Phase::Walking | Phase::Retry { .. } => return Body::Card(Card::Walking),
            Phase::Classes { n, total } | Phase::Facts { n, total } => {
                return Body::Card(Card::Reading { n, total })
            }
            Phase::Ready => {}
        }
        if self.boards_off(now) {
            return Body::Card(Card::MotorOff);
        }
        Body::Ride(self.ride(slots, now))
    }

    fn ride(&self, slots: &RideSlots, now: u32) -> Ride {
        let reporting = self.reporting();
        let (ring_board, ring) = battery::ring_reading(&reporting, |a| self.battery_of(a));
        let floor = ring_board.and_then(|b| self.fact(b, Fact::Floor).map(|v| v as i16));
        let low = battery::battery_low(&ring, floor);
        let lit = match ring {
            BatteryReading::Charge(c) => battery::lit_segments(c.percent()),
            _ => 0,
        };
        let speed = speed::machine_speed(&self.speed_facts(&reporting));
        let attitude = |s: Option<RideSlot>| {
            s.map(|s| Corner {
                label: s.label(),
                attitude: Attitude::of(
                    self.inbox.get(s.address),
                    self.fact(s.address, Fact::ImuModel).map(|v| v as u8),
                ),
            })
        };
        let lower = self.lower(slots, now);
        Ride {
            dial: Dial {
                lit,
                caution: low,
                speed: SpeedShow::of(&speed),
            },
            corners: [attitude(slots.left), attitude(slots.right)],
            pack: Pack::of(&ring, low),
            lower,
        }
    }

    fn speed_facts(&self, reporting: &[u8]) -> Vec<SpeedFacts, 4> {
        reporting
            .iter()
            .map(|&a| SpeedFacts {
                address: a,
                wheel_rate_q12: self.inbox.get(a).map_or(0, |s| s.wheel_rate),
                pole_pairs: self.fact(a, Fact::PolePairs).map(|v| v as u8),
                diameter_mm: self.fact(a, Fact::WheelDiameter).map(|v| v as u16),
                side: self.fact(a, Fact::Side).map(|v| v as u8),
            })
            .collect()
    }

    fn current_of(&self, a: u8) -> CurrentReading {
        current::current_reading(
            self.inbox.get(a),
            self.fact(a, Fact::CurrentCal).map(|v| v as u16),
        )
    }

    fn lamp_words(&self) -> Vec<(u8, u16), 4> {
        self.reporting()
            .iter()
            .map(|&a| {
                let fault = self.inbox.get(a).map_or(0, |s| s.fault);
                (a, self.latch.lamp_word(a, fault))
            })
            .collect()
    }

    fn lower(&self, slots: &RideSlots, now: u32) -> Lower {
        let words = self.lamp_words();
        let test = self.lamp_test(now);
        if lamps::grid_shows(&words, test) {
            let facts: Vec<LampFacts, 4> = self
                .mcs()
                .iter()
                .map(|&a| LampFacts {
                    address: a,
                    imu_model: self.fact(a, Fact::ImuModel).map(|v| v as u8),
                })
                .collect();
            let shown = lamps::shown_lamps(&facts, &words);
            let cells = shown
                .iter()
                .map(|&lamp| {
                    let marks = [slots.left, slots.right].map(|s| {
                        s.map(|s| (s.label(), lamps::mark_on(lamp, s.address, &words, test)))
                    });
                    LampCell {
                        lamp,
                        lit: lamps::lamp_on(lamp, &words, test),
                        marks,
                    }
                })
                .collect();
            return Lower::Grid(cells);
        }
        let bar = |s: Option<RideSlot>| {
            s.map(|s| {
                let r = if self.inbox.get(s.address).is_some() {
                    self.current_of(s.address)
                } else {
                    CurrentReading::Silent
                };
                let limit = self.fact(s.address, Fact::CurrentLimit).map(|v| v as u32);
                PhaseBar {
                    label: s.label(),
                    text: current::phase_text(&r),
                    fill: current::phase_fill(&r, limit).map(|f| libm::round(f * 1000.0) as u16),
                }
            })
        };
        let band = if self.arm.armed() {
            ArmBand::Armed
        } else if self.conditions().refusal().is_none() {
            ArmBand::CanArm
        } else {
            ArmBand::CannotArm
        };
        Lower::Bars {
            phase: [bar(slots.left), bar(slots.right)],
            band,
        }
    }

    /// The board named as the lamp sheet names it ("Left board"), or by address.
    fn board_name(&self, slots: &RideSlots, addr: u8) -> String<12> {
        let mut s = String::new();
        match slots.of(addr) {
            Some(slot) if slot.by_side => {
                let _ = s.push_str(match slot.side {
                    slots::SlotSide::Left => "Left board",
                    slots::SlotSide::Right => "Right board",
                });
            }
            _ => {
                let _ = write!(s, "Board 0x{addr:02X}");
            }
        }
        s
    }

    /// The statement line: the first that applies (section 5.4).
    fn statement(&self, slots: &RideSlots) -> Option<Statement> {
        let mut text: String<64> = String::new();
        let say = |text: String<64>, tone| Some(Statement { text, tone });
        if self.button_fault {
            let _ = text.push_str("BOOT held.");
            return say(text, Tone::Fg);
        }
        if let Some(Dropped::Fault { bit, board }) = self.arm.fault_disarm {
            let title = Lamp::of(bit).map_or("fault", |l| l.title());
            let _ = write!(
                text,
                "Disarmed: {}, {}.",
                title,
                self.board_name(slots, board)
            );
            return say(text, Tone::Warning);
        }
        if !self.arm.armed() && self.touch.finger_down {
            let _ = text.push_str("Release the screen to arm.");
            return say(text, Tone::Muted);
        }
        if self.frames_differ() {
            let _ = text.push_str("Cannot arm: the boards are set to different frames.");
            return say(text, Tone::Warning);
        }
        if let Err(r) = self.drive() {
            let _ = r.write(&mut text);
            let tone = if r == Refusal::Unread {
                Tone::Muted
            } else {
                Tone::Warning
            };
            return say(text, tone);
        }
        if !self.mc_heard() || self.mcs().is_empty() {
            let _ = text.push_str("No motor controller found yet.");
            return say(text, Tone::Muted);
        }
        let reporting = self.reporting();
        let facts = self.speed_facts(&reporting);
        let speed = speed::machine_speed(&facts);
        let (ring_board, ring) = battery::ring_reading(&reporting, |a| self.battery_of(a));
        let slot_boards: Vec<(u8, CurrentReading), 2> = [slots.left, slots.right]
            .iter()
            .flatten()
            .map(|s| (s.address, self.current_of(s.address)))
            .collect();
        let missing =
            missing::missing_settings(&speed, &facts, ring_board, &ring, &slot_boards, |a| {
                self.fact(a, Fact::CurrentCal).map(|v| v as u16)
            });
        if let Some(m) = missing.first() {
            let _ = text.push_str(m.setting.text());
            return say(text, Tone::Muted);
        }
        if self.arm.armed() {
            if let Ok(t) = self.drive() {
                for (a, w) in self.lamp_words() {
                    if t.drive.contains(&a) {
                        continue;
                    }
                    if let Some(l) = Lamp::first_lit(w) {
                        let _ = write!(text, "{} on {}.", l.title(), self.board_name(slots, a));
                        return say(text, Tone::Warning);
                    }
                }
            }
        }
        if let Some(Refused::NotReady) = self.arm.refused {
            let _ = text.push_str("Cannot arm: the machine is not read yet.");
            return say(text, Tone::Muted);
        }
        None
    }

    /// The debug line (section 5.6): two short lines of the touch and the link's rates.
    pub fn debug_lines(&self) -> (String<40>, String<40>) {
        let d = &self.touch_debug;
        let mut a = String::new();
        let mut b = String::new();
        let _ = write!(
            a,
            "{} z{} x{}/{} y{}/{}",
            d.state as char, d.z, d.x.0, d.x.1, d.y.0, d.y.1
        );
        let _ = write!(
            b,
            "{}Hz rx{} tx{} h{}k",
            d.rate,
            self.rx_rate,
            self.tx_rate,
            self.heap_free / 1024
        );
        (a, b)
    }
}

/// Every widget's state.
#[derive(Clone, PartialEq, Debug)]
pub struct View {
    pub top: Top,
    pub body: Body,
    /// The statement line, or `None` (the debug line may draw there).
    pub line: Option<Statement>,
    /// The armed frame and LED.
    pub armed: bool,
}

/// The top strip.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Top {
    pub dots: [Option<Dot>; 2],
    pub low: bool,
    pub test: bool,
}

/// One link dot.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Dot {
    pub label: String<4>,
    pub heard: bool,
}

/// What fills the space under the top strip. One value at a time, built per frame, so the size
/// difference between the variants costs nothing.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, PartialEq, Debug)]
pub enum Body {
    Card(Card),
    Ride(Ride),
}

/// The cards (section 5.5).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Card {
    NoBridge,
    Connecting([u8; 6]),
    Walking,
    Reading { n: u8, total: u8 },
    MotorOff,
}

/// The instruments.
#[derive(Clone, PartialEq, Debug)]
pub struct Ride {
    pub dial: Dial,
    pub corners: [Option<Corner>; 2],
    pub pack: Pack,
    pub lower: Lower,
}

/// The battery ring and the speed.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Dial {
    pub lit: u8,
    pub caution: bool,
    pub speed: SpeedShow,
}

/// The speed as the dial prints it (section 5.2; `specs/remote-ui.md` 3.1's no-value table).
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum SpeedShow {
    /// No board reporting: "--" faint, "no board reporting" under it.
    Silent,
    /// A fact missing: "--", "km/h" faint.
    NoSpeed,
    /// The magnitude, and whether "R" follows "km/h".
    Value { text: String<8>, reverse: bool },
}

impl SpeedShow {
    fn of(s: &SpeedReading) -> SpeedShow {
        match s {
            SpeedReading::Silent => SpeedShow::Silent,
            SpeedReading::NoSpeed { .. } => SpeedShow::NoSpeed,
            SpeedReading::Kmh(k) => SpeedShow::Value {
                text: speed::speed_text(k.shown()),
                reverse: k.reverse(),
            },
        }
    }
}

/// One attitude corner.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Corner {
    pub label: String<4>,
    pub attitude: Attitude,
}

/// The pack line.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Pack {
    pub text: String<20>,
    /// `Caution` while low.
    pub caution: bool,
    /// `FgMuted`: no reading, or a voltage with no pack to judge it against.
    pub muted: bool,
}

impl Pack {
    fn of(r: &BatteryReading, low: bool) -> Pack {
        let mut text = String::new();
        // To a tenth, half up, as the app's `%.1f` prints it (`ride_pack`, `strings.xml`).
        let volts = |t: &mut String<20>, cv: u16| {
            let tenths = (u32::from(cv) + 5) / 10;
            let _ = write!(t, "{}.{} V", tenths / 10, tenths % 10);
        };
        match r {
            BatteryReading::NoReading => {
                let _ = text.push_str("no pack voltage");
                Pack {
                    text,
                    caution: false,
                    muted: true,
                }
            }
            BatteryReading::Unjudged(cv, _) => {
                volts(&mut text, *cv);
                Pack {
                    text,
                    caution: low,
                    muted: !low,
                }
            }
            BatteryReading::Charge(c) => {
                volts(&mut text, c.centivolts);
                let _ = write!(text, "  {} %", c.percent());
                Pack {
                    text,
                    caution: low,
                    muted: false,
                }
            }
        }
    }
}

/// The phase bars and the arm band, or the lamp grid in their place.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, PartialEq, Debug)]
pub enum Lower {
    Bars {
        phase: [Option<PhaseBar>; 2],
        band: ArmBand,
    },
    Grid(Vec<LampCell, 10>),
}

/// One phase bar: its label, its figure and its fill in thousandths (`None`: no fill).
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct PhaseBar {
    pub label: String<4>,
    pub text: String<12>,
    pub fill: Option<u16>,
}

/// The arm band (section 5.3).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ArmBand {
    Armed,
    CanArm,
    CannotArm,
}

/// One lamp of the grid, with a mark per slot.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct LampCell {
    pub lamp: Lamp,
    pub lit: bool,
    pub marks: [Option<(String<4>, bool)>; 2],
}

/// The statement line's colour.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Tone {
    Fg,
    Muted,
    Caution,
    Warning,
}

/// One statement.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Statement {
    pub text: String<64>,
    pub tone: Tone,
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use net::pdu::{Opcode, Pdu};
    use std::vec::Vec as StdVec;

    fn pdu(op: u8, src: u8, dst: u8, payload: &[u8]) -> StdVec<u8> {
        let mut b = [0u8; 64];
        let n = Pdu::new(op, src, dst, payload)
            .unwrap()
            .encode(&mut b)
            .unwrap();
        b[..n].to_vec()
    }

    /// A bench machine: bridge 0x01, motor controllers 0x02 and 0x03 (BOUND, sides left and
    /// right, 15 pole pairs, 165 mm, 10 cells Li-ion), answering what the model sends.
    fn answer(req: &[u8], guest: u8) -> Option<StdVec<u8>> {
        let p = Pdu::decode(req).unwrap();
        let op = |o: Opcode| o.to_u8();
        match p.known()? {
            Opcode::NodeHello => Some(pdu(
                op(Opcode::NodeHello),
                0x01,
                p.src,
                &[1, 1, 0, 0, 0, guest],
            )),
            Opcode::ProbePorts => {
                let e: StdVec<[u8; 4]> = match p.dst {
                    0x01 => std::vec![[0, 0, 2, 0x02], [1, 1, 2, guest]],
                    0x02 => std::vec![[0, 0, 2, 0x01], [1, 0, 2, 0x03]],
                    _ => std::vec![[0, 0, 2, 0x02]],
                };
                let mut pl = std::vec![e.len() as u8];
                for x in e {
                    pl.extend_from_slice(&x);
                }
                Some(pdu(op(Opcode::Ports), p.dst, p.src, &pl))
            }
            Opcode::ConfigRead => {
                let f = p.payload[0];
                let (tag, bytes): (store::Type, StdVec<u8>) =
                    if f == store::NODE_CLASS.key().field_id {
                        (store::Type::U8, std::vec![if p.dst == 1 { 2 } else { 1 }])
                    } else if f == store::CONTROL_SIDE.key().field_id {
                        (store::Type::U8, std::vec![if p.dst == 3 { 1 } else { 0 }])
                    } else if f == store::MACHINE_DRIVE_MODE.key().field_id {
                        (store::Type::U8, std::vec![1])
                    } else if f == store::field::MOTOR_POLE_PAIRS.key().field_id {
                        (store::Type::U8, std::vec![15])
                    } else if f == store::field::WHEEL_DIAMETER_MM.key().field_id {
                        (store::Type::U16, 165u16.to_le_bytes().to_vec())
                    } else if f == store::BOARD_BATTERY_CELLS.key().field_id {
                        (store::Type::U8, std::vec![10])
                    } else if f == store::MOTOR_CURRENT_LIMIT.key().field_id {
                        (store::Type::U32, 10_000u32.to_le_bytes().to_vec())
                    } else if f == store::MOTOR_CURRENT_CAL.key().field_id {
                        (store::Type::U16, 455u16.to_le_bytes().to_vec())
                    } else if f == store::BOARD_BATTERY_FLOOR.key().field_id {
                        (store::Type::I16, 0i16.to_le_bytes().to_vec())
                    } else if f == store::IMU_MODEL.key().field_id {
                        (store::Type::U8, std::vec![1])
                    } else {
                        (store::Type::U8, std::vec![0])
                    };
                let mut pl = std::vec![f, p.payload[1], net::walk::CFG_OK, tag.tag()];
                pl.extend_from_slice(&bytes);
                Some(pdu(op(Opcode::ConfigResp), p.dst, p.src, &pl))
            }
            _ => None,
        }
    }

    fn cyclic(src: u8, battery: u16, fault: u16) -> StdVec<u8> {
        let s = CyclicState {
            battery,
            fault,
            mode: 3,
            ..CyclicState::decode(&[0u8; CyclicState::ENCODED_LEN]).unwrap()
        };
        let mut b = [0u8; CyclicState::ENCODED_LEN];
        let n = s.encode(&mut b);
        pdu(linkctl::OP_CYCLIC_STATE, src, 0, &b[..n])
    }

    /// Run the model connected to the bench machine for `ms`, both boards streaming. Each reply
    /// is handed back at once, in the same 10 ms step.
    fn run(m: &mut Model, start: u32, ms: u32, fault: u16) -> u32 {
        let mut now = start;
        while now < start + ms {
            let mut sends = Sends::new();
            if now.is_multiple_of(200) {
                m.on_pdu(&cyclic(2, 3900, 0), now, &mut sends);
                m.on_pdu(&cyclic(3, 3900, fault), now, &mut sends);
            }
            m.poll(now, &mut sends);
            let mut pending: StdVec<StdVec<u8>> = sends.iter().map(|s| s.to_vec()).collect();
            while let Some(r) = pending.pop() {
                if let Some(a) = answer(&r, 0x81) {
                    let mut more = Sends::new();
                    m.on_pdu(&a, now, &mut more);
                    pending.extend(more.iter().map(|s| s.to_vec()));
                }
            }
            now += 10;
        }
        now
    }

    #[test]
    fn the_cards_then_the_ride_view() {
        let m = Model::new(None);
        assert_eq!(m.view(0).body, Body::Card(Card::NoBridge));
        let mut m = Model::new(Some([0xdc, 6, 0x75, 0xf8, 0x52, 0x6c]));
        assert!(matches!(m.view(0).body, Body::Card(Card::Connecting(_))));
        m.connected(0);
        assert_eq!(m.view(0).body, Body::Card(Card::Walking));
        let now = run(&mut m, 0, 4000, 0);
        let v = m.view(now);
        let Body::Ride(r) = v.body else {
            panic!("{:?}", v.body)
        };
        assert_eq!(r.corners[0].as_ref().unwrap().label, "L");
        assert_eq!(r.corners[1].as_ref().unwrap().label, "R");
        assert!(r.pack.text.starts_with("39.0 V"), "{}", r.pack.text);
        let p = Pack::of(
            &BatteryReading::Unjudged(3096, battery::Unjudged::NoCells),
            false,
        );
        assert_eq!(p.text, "31.0 V", "30.96 V prints as the app's %.1f does");
        assert!(matches!(r.dial.speed, SpeedShow::Value { .. }));
        assert!(
            matches!(
                r.lower,
                Lower::Bars {
                    band: ArmBand::CanArm,
                    ..
                }
            ),
            "{:?}",
            r.lower
        );
        assert_eq!(v.line, None);
    }

    #[test]
    fn the_lamp_test_shows_the_grid_for_two_seconds_after_the_facts_read() {
        let mut m = Model::new(Some([0; 6]));
        m.connected(0);
        let mut now = 0;
        while !m.session.ready() {
            now = run(&mut m, now, 10, 0);
        }
        let mut sends = Sends::new();
        m.poll(now, &mut sends);
        let Body::Ride(r) = m.view(now).body else {
            panic!()
        };
        assert!(matches!(r.lower, Lower::Grid(_)));
        assert!(m.view(now).top.test);
        let now = run(&mut m, now, LAMP_TEST_MS + 20, 0);
        let Body::Ride(r) = m.view(now).body else {
            panic!()
        };
        assert!(matches!(r.lower, Lower::Bars { .. }));
    }

    #[test]
    fn a_blocking_bit_shows_the_grid_refuses_the_arm_and_disarms() {
        let mut m = Model::new(Some([0; 6]));
        m.connected(0);
        let now = run(&mut m, 0, 4000, 0);
        m.on_button(Edge::Press);
        m.on_button(Edge::Release { held: 100 });
        assert!(m.arm.armed());
        let now = run(&mut m, now, 400, linkctl::EV_LATCH_A);
        assert!(!m.arm.armed());
        assert_eq!(
            m.arm.fault_disarm,
            Some(Dropped::Fault {
                bit: linkctl::EV_LATCH_A,
                board: 3
            })
        );
        let Body::Ride(r) = m.view(now).body else {
            panic!()
        };
        assert!(matches!(r.lower, Lower::Grid(_)));
        // The latch is held lit after the board clears it, until the next BOOT press.
        let now = run(&mut m, now, 400, 0);
        assert!(matches!(
            m.view(now).body,
            Body::Ride(Ride {
                lower: Lower::Grid(_),
                ..
            })
        ));
        m.on_button(Edge::Press);
        assert!(matches!(
            m.view(now).body,
            Body::Ride(Ride {
                lower: Lower::Bars { .. },
                ..
            })
        ));
        assert_eq!(m.arm.fault_disarm, None);
    }

    #[test]
    fn no_motor_controller_for_a_second_shows_the_card_and_disarms() {
        let mut m = Model::new(Some([0; 6]));
        m.connected(0);
        let now = run(&mut m, 0, 4000, 0);
        m.on_button(Edge::Press);
        m.on_button(Edge::Release { held: 100 });
        assert!(m.arm.armed());
        // The boards go quiet: they leave the inbox 1.2 s after their last frame, and the card
        // and the disarm follow 1 s later.
        let mut sends = Sends::new();
        m.poll(now + 1200, &mut sends);
        assert!(!m.mc_heard());
        m.poll(now + 1200 + 990, &mut sends);
        assert!(m.arm.armed());
        m.poll(now + 1200 + 1010, &mut sends);
        assert!(!m.arm.armed());
        assert_eq!(m.view(now + 2300).body, Body::Card(Card::MotorOff));
    }

    #[test]
    fn the_pump_drives_the_demand_while_armed_and_down() {
        let mut m = Model::new(Some([0; 6]));
        m.connected(0);
        let now = run(&mut m, 0, 4000, 0);
        assert!(m.run_pump(now));
        assert_eq!(m.outbox.drive.len(), 2);
        assert_eq!(m.outbox.drive[0].1.kind, linkctl::DriveKind::Neutral);
        m.outbox = Outbox::default();
        m.on_button(Edge::Press);
        m.on_button(Edge::Release { held: 100 });
        use crate::touch::SUB;
        m.touch = TouchView {
            finger_down: true,
            drive: Some((200 * SUB, 80 * SUB)),
            overlay: None,
        };
        assert_eq!(m.command().demand, 32767);
        assert!(m.run_pump(now + 50));
        assert_eq!(m.outbox.drive[0].1.value, 32767);
        m.touch.overlay = Some(((200 * SUB, 80 * SUB), true));
        assert_eq!(
            m.overlay(),
            Some(Overlay {
                y0: 200,
                yf: 80,
                live: true,
                percent: 100
            })
        );
        assert!(!m.take_release());
        // A release goes out at once, and is reported as one.
        m.outbox = Outbox::default();
        m.touch.drive = None;
        m.touch.overlay = Some(((200 * SUB, 80 * SUB), false));
        assert!(m.run_pump(now + 60));
        assert_eq!(m.outbox.drive[0].1.value, 0);
        assert!(m.take_release());
        assert_eq!(
            m.overlay().unwrap().percent,
            0,
            "a lifting touch demands zero"
        );
    }

    #[test]
    fn the_statement_line_says_release_the_screen_and_boot_held() {
        let mut m = Model::new(Some([0; 6]));
        m.connected(0);
        let now = run(&mut m, 0, 4000, 0);
        m.touch.finger_down = true;
        assert_eq!(m.view(now).line.unwrap().text, "Release the screen to arm.");
        m.on_button(Edge::Press);
        m.on_button(Edge::Fault);
        assert_eq!(m.view(now).line.unwrap().text, "BOOT held.");
        m.button_released();
        m.touch.finger_down = false;
        assert_eq!(m.view(now).line, None);
    }
}
