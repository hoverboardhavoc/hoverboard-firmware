//! Host vectors for the orchestrator core (`specs/integration.md` "Validation", the slice-5
//! set): boot-to-OFF, the power-request walk to INIT/READY/RUN with the MOE decisions observed,
//! the `comms_loss` trip into `fault_a` and SHUTDOWN, plus the supervision ages (`specs/link-control.md`: the 25-tick trip, fresh-frame clear, the
//! never-seen-a-peer exemption, drive staleness at 50 ticks) and the input-task assembly
//! (`power_request` = button OR mirror, rider from both pads).

use super::*;
use linkctl::{CyclicObs, DriveKind, Inputs};
use state::Mode;

/// The per-boot identity these vectors emit under (`BoardIdentity`): a boot tag and a part.
/// No vector's behaviour depends on either, because nothing in the orchestrator CONSUMES them;
/// they are carried into the cyclic payload, which is what
/// `the_cyclic_payload_carries_the_window_and_the_boards_identity` reads them back out of. One
/// value therefore serves every state built here, and it is deliberately not the all-zero one, so
/// a builder that dropped a field shows up as a zero rather than matching by accident.
const TEST_IDENTITY: BoardIdentity = BoardIdentity {
    boot_tag: 0x2A,
    chip: ChipTag::F103C8,
};

fn fresh() -> OrchestratorState {
    OrchestratorState::new(
        0,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    )
}

/// The live-lane shadow every vector here builds with: the section-7 start gains of
/// `specs/state-feedback.md` (`[3491, 4002, 197, 926]`), the stick dead (`speed_max` 0) at the
/// default slew. The registered defaults are the INERT board, a zero word in balance mode, and
/// most vectors here need a balance board that commands torque; the one that pins the inert
/// default builds its own.
fn sf_start() -> control::GainShadow {
    control::GainShadow::of_stored([3491, 4002, 197, 926, 0, 41], control::DEFAULT_GAIN_MAX)
}

/// A disarmed balance board (CONTROL_MODE = 1, IMU configured, a known battery word) on `gains`.
fn balance_board(gains: control::GainShadow) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        gains,
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    assert_eq!(s.obs().control_mode, 1, "balance mode");
    s
}

/// The controller these vectors speak as: one L3 guest address, the shape the rider app's session
/// takes (`Controller` grants `0x80..0xFE` on first contact, and the app sends every payload of a
/// session from the one it was granted).
const CTRL: u8 = 0x81;

/// A SECOND node on the same link. Never the one that armed anything here: it exists to prove
/// that somebody else's traffic is not evidence about this controller.
const OTHER: u8 = 0x82;

/// Run `n` control ticks with no IMU sample, returning the last output.
fn run_ticks(state: &mut OrchestratorState, n: usize) -> ControlOutput {
    let mut last = control_task(state, None, 1);
    for _ in 1..n {
        last = control_task(state, None, 1);
    }
    last
}

/// Hold the (debounced) power request on: two input passes press the active-low button.
fn hold_power(state: &mut OrchestratorState) {
    input_task(
        state,
        &InputSample {
            button_asserted: true,
            ..Default::default()
        },
    );
    input_task(
        state,
        &InputSample {
            button_asserted: true,
            ..Default::default()
        },
    );
    assert!(state.button_pressed, "two-call press confirmed");
}

/// Release the power request: one input pass with the button idle (the debouncer's one-sample
/// release).
fn release_power(state: &mut OrchestratorState) {
    input_task(state, &InputSample::default());
    assert!(!state.button_pressed, "one idle sample releases the hold");
}

/// The appended block every peer vector here carries ([`CyclicObs`]): the OTHER board's quiet
/// window (it is driving nothing, so its current window is zeroed) and the other board's own
/// per-boot identity, deliberately not this one's [`TEST_IDENTITY`].
///
/// One value serves every vector because nothing in the orchestrator CONSUMES the block: it is
/// telemetry for a controller, and the receive path reaches these vectors already decoded. It is
/// a whole block rather than an absence because the codec accepts no other shape
/// (`linkctl::CyclicState::decode` takes exactly `ENCODED_LEN`).
const PEER_OBS: CyclicObs = CyclicObs {
    phase_peak: 0,
    phase_mean: 0,
    duty_on: 0,
    boot_tag: 0x5C,
    chip: ChipTag::F130C8,
    odometer: 0,
};

/// A peer cyclic frame with the given flags.
fn cyclic(flags: u8) -> Payload {
    Payload::CyclicState(CyclicState {
        pitch: 10,
        roll: -10,
        wheel_speed: 100,
        battery: 3000,
        mode: 3,
        fault: 0,
        flags,
        peer_demand: 0,
        wheel_pos: 0,
        obs: PEER_OBS,
        wheel_rate: 0,
    })
}

// --- Boot-to-OFF -----------------------------------------------------------------------------

#[test]
fn boot_to_off_and_stays_off_with_no_inputs() {
    let mut s = fresh();
    assert_eq!(s.mode.mode(), Mode::Off);
    let out = run_ticks(&mut s, 100);
    assert_eq!(out.mode_byte, Mode::Off.as_byte());
    assert_eq!(out.moe, [false; N_MOTORS]);
    assert_eq!(out.init, None);
    assert_eq!(out.shutdown, None);
    assert!(!out.comms_loss, "never seen a peer: no comms_loss at boot");
    let obs = s.obs();
    assert_eq!(obs.control_ticks, 100);
    assert_eq!(obs.mode_byte, 0);
    assert_eq!(obs.moe_bits, 0);
    assert_eq!((obs.enact_inits, obs.enact_shutdowns), (0, 0));
}

// --- The power-request walk ------------------------------------------------------------------

#[test]
fn power_request_walks_off_init_ready_run_with_moe_observed() {
    let mut s = fresh();
    hold_power(&mut s);

    // Tick 1: OFF -> INIT (transition only; the INIT pass runs next tick).
    let t1 = control_task(&mut s, None, 1);
    assert_eq!(t1.mode_byte, Mode::Init.as_byte());
    assert_eq!(t1.init, None);
    assert_eq!(t1.moe, [false; N_MOTORS], "MOE not yet set");

    // Tick 2: the INIT pass: MOE set, the bring-up record leaves through the enact seam.
    let t2 = control_task(&mut s, None, 1);
    assert_eq!(t2.mode_byte, Mode::Ready.as_byte());
    assert_eq!(t2.init, Some(InitAction { run_bringup: true }));
    assert_eq!(t2.moe, [true; N_MOTORS], "MOE decisions set at INIT");
    assert_eq!(s.enact_inits, 1, "the seam recorded the bring-up");

    // Tick 3: READY -> RUN; MOE holds.
    let t3 = control_task(&mut s, None, 1);
    assert_eq!(t3.mode_byte, Mode::Run.as_byte());
    assert_eq!(t3.moe, [true; N_MOTORS]);

    // RUN holds while the request holds (levels re-sampled every tick).
    let t = run_ticks(&mut s, 50);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert_eq!(s.obs().moe_bits, 0b11);

    // Releasing the button (one-call release) drops the request: RUN -> SHUTDOWN -> OFF, MOE
    // cleared, the safe-down record leaves through the seam.
    input_task(&mut s, &InputSample::default());
    assert!(!s.button_pressed, "one-call release");
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.shutdown, Some(ShutdownAction { run_safedown: true }));
    assert_eq!(t.moe, [false; N_MOTORS]);
    assert_eq!(s.enact_shutdowns, 1);
}

#[test]
fn remote_mirror_bit_is_an_equivalent_power_request_producer() {
    // No button at all: the INPUTS mirror's power bit walks the machine the same way (the
    // level-OR assembly). Equivalent WITHIN the mirror's freshness window, which is what the
    // handful of ticks here stays inside; the two producers part company once it expires
    // (`a_stale_remote_mirror_releases_the_bridge_it_armed`).
    let mut s = fresh();
    s.inbox.accept(
        CTRL,
        Payload::Inputs(Inputs {
            buttons: Inputs::BUTTON_POWER,
            rider: 0,
        }),
    );
    assert!(s.power_request());
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // The mirror dropping the bit (latest-wins) releases the request: SHUTDOWN then OFF.
    s.inbox.accept(
        CTRL,
        Payload::Inputs(Inputs {
            buttons: 0,
            rider: 0,
        }),
    );
    let t = run_ticks(&mut s, 2);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
}

// --- comms_loss: the supervision trip into fault_a -------------------------------------------

#[test]
fn comms_loss_trips_at_26_ticks_and_forces_shutdown() {
    let mut s = fresh();
    hold_power(&mut s);
    s.inbox.accept(CTRL, cyclic(0));
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // The peer goes silent. Ages 4..=25 (22 more ticks): within the window, RUN holds.
    let t = run_ticks(&mut s, 22);
    assert_eq!(s.inbox.cyclic_age(), 25);
    assert!(!t.comms_loss, "age 25 is within the 100 ms window");
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // Age 26: the level asserts, folds into fault_a, RUN -> SHUTDOWN.
    let t = control_task(&mut s, None, 1);
    assert!(t.comms_loss, "age 26 > CYCLIC_TIMEOUT_TICKS trips");
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
    assert!(s.obs().comms_loss);

    // Level-sensitive: a fresh cyclic clears it, and re-entry follows the machine's own gates
    // (the held request walks OFF -> INIT again).
    s.inbox.accept(CTRL, cyclic(0));
    let t = control_task(&mut s, None, 1);
    assert!(!t.comms_loss, "fresh cyclic clears the level");
    assert_eq!(t.mode_byte, Mode::Init.as_byte());
}

#[test]
fn never_seen_a_peer_never_asserts_comms_loss() {
    // Single-board operation is legitimate: with no cyclic ever received, RUN holds far past
    // the timeout with the age never accruing.
    let mut s = fresh();
    hold_power(&mut s);
    let t = run_ticks(&mut s, 200);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert!(!t.comms_loss);
    assert_eq!(s.inbox.cyclic_age(), 0, "no peer: the age never accrues");
}

// --- the motor-side fault level: the third fault_a producer group ----------------------------

#[test]
fn motor_fault_folds_into_fault_a_and_forces_shutdown() {
    // `specs/motor-integration.md`, "the motor-side fault producers": the hall dwell fault,
    // period-liveness loss and a refused calibration are folded by the firmware into one level
    // written here before each pass, and it must drive SHUTDOWN (and therefore disarm) exactly
    // like the other fault_a producers.
    let mut s = fresh();
    hold_power(&mut s);
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert!(!s.obs().motor_fault);

    // The motor faults: RUN -> SHUTDOWN, then OFF with MOE withdrawn from every motor.
    s.motor_fault = EV_MOTOR_HALL;
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    assert!(s.obs().motor_fault, "the level is published");
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);

    // A LEVEL, not a latch at this boundary: the producers own their own stickiness, so clearing
    // it here lets the held power request walk the machine back up on its own gates.
    s.motor_fault = 0;
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Init.as_byte());
}

#[test]
fn a_quiescent_motor_level_changes_nothing() {
    // The pre-motor boot posture: a board with no motor brought up leaves the level false, and
    // the pipeline behaves exactly as it did before the producer existed.
    let mut s = fresh();
    hold_power(&mut s);
    let t = run_ticks(&mut s, 50);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert!(!t.moe.iter().all(|m| !m), "RUN allows MOE");
    assert!(!s.obs().motor_fault);
}

// --- off_inhibit: the wheel-motion producer (slice 5) ----------------------------------------

#[test]
fn a_turning_wheel_holds_the_machine_in_off() {
    // `specs/motor-integration.md`, "MOE enactment": `off_inhibit` gets its real producer at
    // slice 5, and it reads the period ISR's speed word. A vehicle whose wheel is already moving
    // does not engage, so a held power request does NOT walk OFF -> INIT while it turns.
    let mut s = fresh();
    s.motor_moving = true;
    hold_power(&mut s);
    let t = run_ticks(&mut s, 50);
    assert_eq!(
        t.mode_byte,
        Mode::Off.as_byte(),
        "a turning wheel holds OFF against a held power request"
    );
    assert_eq!(
        t.moe, [false; N_MOTORS],
        "and nothing is ever allowed to arm"
    );

    // The wheel stops: the same held request now walks the machine up on its own gates.
    s.motor_moving = false;
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Init.as_byte());
}

#[test]
fn off_inhibit_can_never_block_a_shutdown() {
    // The mode machine consults `off_inhibit` in OFF only, which is what stops a wheel spun by
    // hand (or a stuck speed word) from holding an engaged vehicle in RUN through a fault.
    let mut s = fresh();
    hold_power(&mut s);
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    s.motor_moving = true;
    s.motor_fault = EV_MOTOR_HALL;
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
}

// --- Drive staleness (the age half; the reference-zeroing consumer is slice 6) ---------------

#[test]
fn drive_staleness_at_51_ticks_and_never_received() {
    let mut s = fresh();
    assert!(s.inbox.drive_stale(), "never received = stale");
    assert_eq!(s.inbox.drive(), None);

    s.inbox.accept(
        CTRL,
        Payload::DriveCmd(DriveCmd {
            kind: DriveKind::Throttle,
            value: 500,
            steer: 0,
        }),
    );
    assert!(!s.inbox.drive_stale(), "fresh on receipt");
    run_ticks(&mut s, 50);
    assert_eq!(s.inbox.drive_age(), 50);
    assert!(!s.inbox.drive_stale(), "age 50 is within the 200 ms window");
    run_ticks(&mut s, 1);
    assert!(s.inbox.drive_stale(), "age 51 > DRIVE_TIMEOUT_TICKS");
    // Latest-wins refresh restores freshness.
    s.inbox.accept(
        CTRL,
        Payload::DriveCmd(DriveCmd {
            kind: DriveKind::Neutral,
            value: 0,
            steer: 0,
        }),
    );
    assert!(!s.inbox.drive_stale());
    assert_eq!(s.inbox.drive().unwrap().kind, DriveKind::Neutral);
}

// --- Mirror staleness (`specs/link-control.md`, "Supervision": the remote INPUTS mirror) ------

/// An `INPUTS` mirror payload.
fn mirror(buttons: u8, rider: u8) -> Payload {
    Payload::Inputs(Inputs { buttons, rider })
}

/// The number of ticks that takes a mirror accepted `since` ticks ago to exactly the timeout (the
/// last tick on which it is still fresh).
fn to_timeout_edge(since: usize) -> usize {
    INPUTS_TIMEOUT_TICKS as usize - since
}

#[test]
fn a_stale_remote_mirror_releases_the_bridge_it_armed() {
    // The failure this closes: the mirror slot never expired, so a board armed over the link
    // stayed in RUN with the motor enables set INDEFINITELY once the link dropped. The drive
    // demand decayed at 200 ms and the wheels stopped, but the bridge stayed live with no release
    // path left, because the only producer that could withdraw the request was the controller
    // that had gone away.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    let t = run_ticks(&mut s, 3);
    assert_eq!(
        t.mode_byte,
        Mode::Run.as_byte(),
        "armed from the mirror alone, no local button"
    );
    assert_eq!(t.moe, [true; N_MOTORS], "the motor enables are set");

    // The controller goes silent. Everything up to the timeout holds RUN: a link stall is not a
    // departed controller, and disarming a balancing machine mid-ride is a fall.
    let t = run_ticks(&mut s, to_timeout_edge(3));
    assert_eq!(s.inbox.remote_age(), INPUTS_TIMEOUT_TICKS);
    assert!(
        !s.inbox.remote_stale(),
        "age 375 is inside the 1.5 s window"
    );
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert_eq!(t.moe, [true; N_MOTORS]);

    // One tick more and the mirror stops being a source: the power request falls, and RUN leaves
    // on !power_request exactly as it does for a released button.
    let t = control_task(&mut s, None, 1);
    assert!(s.inbox.remote_stale(), "age 376 > INPUTS_TIMEOUT_TICKS");
    assert!(!s.power_request(), "a stale mirror asserts nothing");
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());

    // The SHUTDOWN pass: the safe-down record leaves through the enact seam and every motor
    // enable is withdrawn.
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS], "the motor enables are cleared");
    assert!(t.shutdown.is_some(), "the safe-down ran");

    // And it STAYS released: the stale mirror cannot walk the machine back up.
    let t = run_ticks(&mut s, 200);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
}

#[test]
fn a_repeating_controller_holds_the_bridge_indefinitely() {
    // The other half of the contract: a controller keeps a level asserted by REPEATING it, at
    // whatever cadence it repeats at. This refreshes at 300 ticks (1.2 s), the slowest cadence
    // that still lands inside the window, and holds RUN across ten of them.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());
    for round in 0..10 {
        let t = run_ticks(&mut s, 300);
        assert_eq!(
            t.mode_byte,
            Mode::Run.as_byte(),
            "round {round}: a refreshed mirror holds the request"
        );
        assert_eq!(t.moe, [true; N_MOTORS]);
        s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
        assert_eq!(s.inbox.remote_age(), 0, "receipt resets the age");
    }
}

#[test]
fn a_button_armed_board_is_untouched_by_mirror_staleness() {
    // Staleness withdraws only the mirror's half of the power-request OR. A board armed from its
    // own button holds a level in hardware, and no link event may disarm it.
    let mut s = fresh();
    hold_power(&mut s);
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    let t = run_ticks(&mut s, INPUTS_TIMEOUT_TICKS as usize * 3);
    assert!(s.inbox.remote_stale(), "the mirror is long stale");
    assert!(!s.inbox.remote_power_request(), "its bit reads released");
    assert!(s.power_request(), "the button still holds the request");
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert_eq!(t.moe, [true; N_MOTORS]);
}

// --- Mirror LIVENESS: which payloads prove the controller is still there --------------------
//
// (`specs/link-control.md`, "Supervision": controller liveness.) The window answers "has the
// controller gone away". Refreshing it on the `INPUTS` keepalive ALONE made it answer "did the
// keepalive specifically arrive", which is a different and much weaker question when the rider
// is demonstrably present at twenty demand frames a second.

/// One `DRIVE_CMD` at full-ish demand: what the app streams every 50 ms while connected.
fn demand(value: i16) -> Payload {
    Payload::DriveCmd(DriveCmd {
        kind: DriveKind::Throttle,
        value,
        steer: 0,
    })
}

/// The app's demand cadence in 250 Hz ticks: `LinkConfig.SEND_INTERVAL_MS` is 50 ms, so 13 ticks
/// (rounded UP, so this vector is never accidentally faster than the real stream).
const DEMAND_GAP_TICKS: usize = 13;

#[test]
fn a_live_demand_stream_holds_the_arm_with_every_keepalive_lost() {
    // The defect, and the fix. The app arms with `INPUTS` and then streams `DRIVE_CMD` at 20 Hz,
    // re-sending the arm level only every 500 ms. With the window refreshed by `INPUTS` alone,
    // keepalives at 0/500/1000/1500 ms meant TWO consecutive losses survived with zero margin and
    // three disarmed a rider mid-ride, on a balancing machine, while the demand frames that prove
    // they are holding the control kept arriving and kept being enacted.
    //
    // Here EVERY keepalive after the arming one is lost, for twenty seconds. The rider is still
    // there, and the board knows it, because they are still asking it to move.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    for round in 0..384 {
        s.inbox.accept(CTRL, demand(400));
        let t = run_ticks(&mut s, DEMAND_GAP_TICKS);
        assert_eq!(
            t.mode_byte,
            Mode::Run.as_byte(),
            "round {round}: a 20 Hz demand stream IS the controller, present"
        );
        assert_eq!(t.moe, [true; N_MOTORS], "round {round}: the enables hold");
    }
    assert!(!s.inbox.remote_stale(), "the mirror never went stale");
    assert!(
        s.inbox.remote_power_request(),
        "and still asserts the level"
    );
}

#[test]
fn a_controller_that_stops_talking_entirely_still_disarms_at_the_window() {
    // The other half, unchanged and load-bearing: widening what counts as liveness must not cost
    // the release. Nothing arrives at all, and the mirror expires on the same tick it always did.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    let t = run_ticks(&mut s, to_timeout_edge(3));
    assert_eq!(s.inbox.remote_age(), INPUTS_TIMEOUT_TICKS);
    assert!(!s.inbox.remote_stale(), "the last tick inside the window");
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    let t = run_ticks(&mut s, 3);
    assert!(s.inbox.remote_stale());
    assert_eq!(
        t.mode_byte,
        Mode::Off.as_byte(),
        "released, exactly as before"
    );
    assert_eq!(t.moe, [false; N_MOTORS]);
}

#[test]
fn an_expired_mirror_is_not_revived_by_a_demand_alone() {
    // Expiry is a one-way door, and it has to be. Liveness EXTENDS a window; it cannot reopen one
    // that has already closed. The board has released the level by then, so a demand arriving
    // after the expiry says "the controller is talking again" and nothing whatever about whether
    // the level it stated 1.6 s ago still stands. Refreshing on it would walk a DISARMED board
    // back to RUN on expired content with no fresh `INPUTS` anywhere in the sequence: the spurious
    // ARM the content/liveness split exists to prevent, arriving by the other door.
    //
    // Reachable in exactly the shape this whole change is motivated by: a delivery stall longer
    // than the window, then the demand stream resuming, which is what a BLE link-layer retransmit
    // burst looks like from here.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    // Blackout past the window: the mirror expires and the board disarms, correctly.
    let t = run_ticks(&mut s, to_timeout_edge(3) + 25);
    assert!(s.inbox.remote_stale(), "the mirror expired");
    assert_eq!(t.mode_byte, Mode::Off.as_byte(), "and the board released");
    assert_eq!(t.moe, [false; N_MOTORS]);

    // The owner's demand stream resumes at 20 Hz and keeps arriving. It never re-states the level.
    for round in 0..40 {
        s.inbox.accept(CTRL, demand(400));
        let t = run_ticks(&mut s, DEMAND_GAP_TICKS);
        assert!(
            s.inbox.remote_stale(),
            "round {round}: liveness cannot revive content that was allowed to expire"
        );
        assert!(
            !s.inbox.remote_power_request(),
            "round {round}: the expired level is not re-asserted"
        );
        assert_eq!(
            t.mode_byte,
            Mode::Off.as_byte(),
            "round {round}: a demand alone never re-arms a disarmed board"
        );
        assert_eq!(t.moe, [false; N_MOTORS], "round {round}: enables stay off");
    }

    // Only CONTENT revives it, and content re-states the level in the same breath, so what the
    // board arms on is always something the controller said just now.
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert!(!s.inbox.remote_stale(), "an INPUTS revives the mirror");
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    // And once revived, the demand stream holds it open again as usual.
    for round in 0..40 {
        s.inbox.accept(CTRL, demand(400));
        let t = run_ticks(&mut s, DEMAND_GAP_TICKS);
        assert_eq!(
            t.mode_byte,
            Mode::Run.as_byte(),
            "round {round}: a live mirror is still held by its owner's demand"
        );
    }
}

#[test]
fn a_demand_cannot_manufacture_a_power_request() {
    // CONTENT and LIVENESS are separate facts and only one of them widened. A `DRIVE_CMD` says
    // "still here"; it must never be readable as "and I am asking to be armed", or the fix would
    // have replaced a spurious disarm with a spurious ARM, which is worse.
    let mut s = fresh();

    // No `INPUTS` has ever been received. Stream demand at the app's cadence for six seconds.
    for _ in 0..120 {
        s.inbox.accept(CTRL, demand(1000));
        run_ticks(&mut s, DEMAND_GAP_TICKS);
    }
    assert!(
        s.inbox.drive().is_some(),
        "the demand landed in its own slot"
    );
    assert!(!s.inbox.drive_stale(), "and is fresh there");
    assert!(s.inbox.remote_stale(), "but no mirror exists to be fresh");
    assert!(
        !s.inbox.remote_power_request(),
        "no power request was invented"
    );
    assert!(!s.inbox.remote_rider_present(), "and no rider either");
    assert!(!s.power_request(), "so the assembled level is clear");
    assert_eq!(
        control_task(&mut s, None, 1).mode_byte,
        Mode::Off.as_byte(),
        "the machine never left OFF"
    );

    // And with a mirror that exists but states RELEASED, the demand stream keeps it fresh without
    // ever flipping the level it carries: freshness is not assent.
    s.inbox.accept(CTRL, mirror(0, 0));
    for _ in 0..120 {
        s.inbox.accept(CTRL, demand(1000));
        run_ticks(&mut s, DEMAND_GAP_TICKS);
        assert!(!s.inbox.remote_stale(), "kept fresh by the demand");
        assert!(!s.inbox.remote_power_request(), "and still released");
    }
    assert_eq!(control_task(&mut s, None, 1).mode_byte, Mode::Off.as_byte());
}

#[test]
fn a_demand_stream_does_not_create_a_mirror_for_a_later_level_to_inherit() {
    // The owner comes with the content, so a demand stream cannot pre-claim the slot. If it
    // could, whichever node armed the board next would find the mirror already owned by someone
    // else and its own keepalives disregarded.
    let mut s = fresh();
    s.inbox.accept(CTRL, demand(400));
    run_ticks(&mut s, 10);
    assert!(s.inbox.remote_stale(), "no mirror was created");

    // OTHER arms it. Now OTHER owns the mirror and CTRL's demand is just a demand.
    s.inbox.accept(OTHER, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());
    for _ in 0..40 {
        s.inbox.accept(CTRL, demand(400));
        run_ticks(&mut s, 10);
    }
    assert!(s.inbox.remote_stale(), "CTRL's demand held nothing open");
    assert_eq!(run_ticks(&mut s, 2).mode_byte, Mode::Off.as_byte());
}

#[test]
fn another_nodes_demand_stream_does_not_hold_this_riders_arm() {
    // A live LINK is not a live controller. A second node driving its own traffic across the same
    // board must not answer the question "is the rider who armed THIS board still here".
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    // OTHER streams demand at 20 Hz right through the window. CTRL says nothing.
    for _ in 0..40 {
        s.inbox.accept(OTHER, demand(400));
        run_ticks(&mut s, DEMAND_GAP_TICKS);
    }
    assert!(
        s.inbox.remote_stale(),
        "someone else's presence is not CTRL's"
    );
    assert!(!s.inbox.remote_power_request());
    assert_eq!(run_ticks(&mut s, 2).mode_byte, Mode::Off.as_byte());
}

#[test]
fn a_peers_heartbeat_is_not_controller_liveness() {
    // The one payload deliberately excluded, exercised from the mirror's OWN owner so the
    // exclusion is doing the work rather than the address check: `CYCLIC_STATE` is emitted
    // unconditionally by any powered, addressed board, so if it counted, a peer board would hold
    // its neighbour armed for as long as it has power.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    for _ in 0..40 {
        s.inbox.accept(CTRL, cyclic(0));
        run_ticks(&mut s, DEMAND_GAP_TICKS);
    }
    assert!(
        !s.inbox.comms_loss(),
        "the link itself is demonstrably live"
    );
    assert!(s.inbox.remote_stale(), "which says nothing about the rider");
    assert_eq!(run_ticks(&mut s, 2).mode_byte, Mode::Off.as_byte());
}

#[test]
fn the_mirrors_owner_moves_with_the_level_it_states() {
    // Handover: the latest `INPUTS` owns the mirror, content and liveness together. After OTHER
    // takes it, OTHER's demand holds it and CTRL's no longer does.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    s.inbox.accept(OTHER, mirror(Inputs::BUTTON_POWER, 0));
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());

    for round in 0..40 {
        s.inbox.accept(CTRL, demand(400));
        s.inbox.accept(OTHER, demand(400));
        let t = run_ticks(&mut s, DEMAND_GAP_TICKS);
        assert_eq!(
            t.mode_byte,
            Mode::Run.as_byte(),
            "round {round}: OTHER holds it"
        );
    }
    assert!(!s.inbox.remote_stale());

    // OTHER stops; CTRL keeps streaming. The arm OTHER made is released on schedule.
    for _ in 0..40 {
        s.inbox.accept(CTRL, demand(400));
        run_ticks(&mut s, DEMAND_GAP_TICKS);
    }
    assert!(
        s.inbox.remote_stale(),
        "the former owner's stream is not the owner's"
    );
    assert_eq!(run_ticks(&mut s, 2).mode_byte, Mode::Off.as_byte());
}

#[test]
fn a_stale_mirror_withdraws_the_rider_bit() {
    // The rider bit is permissive (it folds in by OR with the local pads), so a departed
    // controller must not keep asserting that someone is aboard.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(0, Inputs::RIDER_PRESENT));
    run_ticks(&mut s, 3);
    assert!(s.inbox.remote_rider_present());
    assert!(dispatch::rider_level(&s), "the fold sees the mirror");

    run_ticks(&mut s, to_timeout_edge(3));
    assert!(s.inbox.remote_rider_present(), "still fresh at the edge");
    run_ticks(&mut s, 1);
    assert!(!s.inbox.remote_rider_present(), "stale reads NO rider");
    assert!(!dispatch::rider_level(&s), "and the fold loses it");

    // Local pads are unaffected: the fold's other producers keep their own semantics.
    s.rider_present = true;
    assert!(dispatch::rider_level(&s));
}

#[test]
fn the_stale_release_is_attributed_to_power_request() {
    // O1 is the instrument that has to explain an unexplained disarm on the bench, so the release
    // this change introduces must be attributable there rather than looking like a mystery
    // SHUTDOWN. No new OBS word is needed: `EV_POWER_REQUEST` already counts both edges.
    let mut s = fresh();
    s.inbox.accept(CTRL, mirror(Inputs::BUTTON_POWER, 0));
    run_ticks(&mut s, 3);
    assert_eq!(
        s.events.count(EV_POWER_REQUEST),
        1,
        "one edge so far: the request rose"
    );
    run_ticks(&mut s, to_timeout_edge(3) + 1);
    assert_eq!(
        s.events.count(EV_POWER_REQUEST),
        2,
        "the stale release is the second edge"
    );
    assert_eq!(
        u16::from(s.obs().event_levels) & EV_POWER_REQUEST,
        0,
        "and the live level reads clear"
    );
}

#[test]
fn mirror_staleness_edges_and_never_received() {
    let mut s = fresh();
    assert!(s.inbox.remote_stale(), "never received = stale");
    assert!(!s.inbox.remote_power_request());
    assert_eq!(s.inbox.remote_age(), 0);
    run_ticks(&mut s, 500);
    assert_eq!(s.inbox.remote_age(), 0, "no mirror: the age never accrues");

    s.inbox
        .accept(CTRL, mirror(Inputs::BUTTON_POWER, Inputs::RIDER_PRESENT));
    assert!(!s.inbox.remote_stale(), "fresh on receipt");
    run_ticks(&mut s, INPUTS_TIMEOUT_TICKS as usize);
    assert_eq!(s.inbox.remote_age(), INPUTS_TIMEOUT_TICKS);
    assert!(!s.inbox.remote_stale(), "the edge tick is still fresh");
    assert!(s.inbox.remote_power_request());
    run_ticks(&mut s, 1);
    assert!(s.inbox.remote_stale(), "one tick past the timeout");
    assert!(!s.inbox.remote_power_request());
    assert!(!s.inbox.remote_rider_present());

    // Latest-wins refresh restores every level at once.
    s.inbox
        .accept(CTRL, mirror(Inputs::BUTTON_POWER, Inputs::RIDER_PRESENT));
    assert!(!s.inbox.remote_stale());
    assert!(s.inbox.remote_power_request());
    assert!(s.inbox.remote_rider_present());
}

// --- The inbox's slice-6-facing levels -------------------------------------------------------

#[test]
fn peer_lockdown_and_rider_levels_are_latest_wins() {
    let mut s = fresh();
    assert!(!s.inbox.peer_lockdown());
    s.inbox.accept(CTRL, cyclic(CyclicState::FLAG_LOCKDOWN));
    assert!(s.inbox.peer_lockdown());
    s.inbox.accept(CTRL, cyclic(CyclicState::FLAG_RIDER));
    assert!(!s.inbox.peer_lockdown(), "level, latest-wins");
    assert!(s.inbox.peer().unwrap().rider_present());

    s.inbox.accept(
        CTRL,
        Payload::Inputs(Inputs {
            buttons: 0,
            rider: Inputs::RIDER_PRESENT,
        }),
    );
    assert!(s.inbox.remote_rider_present());
}

// --- The input task --------------------------------------------------------------------------

#[test]
fn button_debounce_two_call_press_one_call_release_reaches_power_request() {
    let mut s = fresh();
    input_task(
        &mut s,
        &InputSample {
            button_asserted: true,
            ..Default::default()
        },
    );
    assert!(!s.power_request(), "one assert must not press");
    input_task(
        &mut s,
        &InputSample {
            button_asserted: true,
            ..Default::default()
        },
    );
    assert!(s.power_request(), "two consecutive asserts press");
    input_task(&mut s, &InputSample::default());
    assert!(!s.power_request(), "one-call release");
    assert_eq!(s.input_ticks, 3);
}

#[test]
fn rider_present_needs_both_pads() {
    let mut s = fresh();
    input_task(
        &mut s,
        &InputSample {
            pad_a_high: true,
            ..Default::default()
        },
    );
    assert_eq!(s.pad_field, inputs::PAD_A_BIT);
    assert!(!s.rider_present, "one pad is not a rider");
    input_task(
        &mut s,
        &InputSample {
            pad_a_high: true,
            pad_b_high: true,
            ..Default::default()
        },
    );
    assert_eq!(s.pad_field, inputs::PAD_A_BIT | inputs::PAD_B_BIT);
    assert!(s.rider_present);
    // Pad release debounce: one low sample keeps the rider (2-low-off).
    input_task(&mut s, &InputSample::default());
    assert!(s.rider_present, "one low keeps the pads on");
    input_task(&mut s, &InputSample::default());
    assert!(!s.rider_present, "two lows release");
}

#[test]
fn obs_carries_the_pad_block() {
    // The bench gate's read (`specs/silicon-queue.md`, "The foot pad on silicon"): everything a
    // jumper on a pad pin moves, in one place.
    let mut s = fresh();
    input_task(
        &mut s,
        &InputSample {
            pad_a_high: true,
            pad_b_high: true,
            ..Default::default()
        },
    );
    control_task(&mut s, None, 1);
    let o = s.obs();
    assert_eq!(o.pad_field, inputs::PAD_A_BIT | inputs::PAD_B_BIT);
    assert!(o.rider_present, "both pads is the local rider");
    assert_eq!(
        (o.pad_changes, o.pad_held),
        (1, 0),
        "one engage, nothing held"
    );

    // Release: the first low holds (both pads), the second drops the field.
    input_task(&mut s, &InputSample::default());
    control_task(&mut s, None, 1);
    let o = s.obs();
    assert!(o.rider_present, "one low keeps the rider");
    assert_eq!(
        (o.pad_changes, o.pad_held),
        (1, 2),
        "two pads held one pass each"
    );
    input_task(&mut s, &InputSample::default());
    control_task(&mut s, None, 1);
    let o = s.obs();
    assert_eq!(o.pad_field, 0);
    assert!(!o.rider_present);
    assert_eq!((o.pad_changes, o.pad_held), (2, 2));
}

// --- IMU liveness, hold-on-miss, and the loss fault ------------------------------------------

/// A real burst sample with gravity tilted toward +X (pulls pitch off identity within a few
/// ticks); a healthy read.
fn good_sample() -> imu::Sample {
    imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [8000, 0, 14000],
        temp_centi_degc: 2500,
        still: false,
    }
}

/// A healthy LEVEL read: gravity straight down the Z axis, zero rate. Keeps the attitude at
/// identity (pitch/roll/rate 0, so the numerics match the old zero-sample path) while keeping a
/// configured IMU LIVE, so a balance board stays in RUN. Balance-engagement scenarios need this:
/// a configured IMU fed only failed reads now (correctly) trips the IMU-loss fault and disengages.
fn level_sample() -> imu::Sample {
    imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [0, 0, 16384],
        temp_centi_degc: 2500,
        still: false,
    }
}

/// Walk a configured-IMU board to RUN on healthy reads.
fn configured_to_run() -> OrchestratorState {
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    hold_power(&mut s);
    let good = good_sample();
    for _ in 0..3 {
        control_task(&mut s, Some(&good), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run);
    assert!(!s.imu_health.loss());
    s
}

/// A boot state with the two IMU booleans set independently: `configured` is the bring-up
/// outcome and `staged` is whether the layout ASKED for an IMU at all.
fn imu_board(configured: bool, staged: bool) -> OrchestratorState {
    OrchestratorState::new(
        0,
        0,
        0,
        configured,
        staged,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    )
}

#[test]
fn the_flags_byte_separates_an_unstaged_board_from_a_failed_imu_bringup() {
    // The defect this answers: `imu_configured` (b0) is 0 for a board with no IMU fitted AND for
    // every way a staged IMU can fail to come up, so a bench read of 0 could not say which it
    // was. b6 is the ask, so the PAIR is the diagnosis.

    // Staged, and the probe answered: both bits.
    let f = imu_board(true, true).obs().flags_byte();
    assert_eq!(f & 0x41, 0x41, "staged and configured");

    // Staged, and the bring-up failed (an unresolvable pair, an unknown model index, a refused
    // frame, a bus that would not init, or a probe that did not answer: the firmware returns the
    // same `None` for all of them). b6 stands alone, and THIS is the finding.
    let f = imu_board(false, true).obs().flags_byte();
    assert_eq!(f & 0x41, 0x40, "staged, not configured");

    // Nothing was ever asked for: neither bit, and the b0 zero is expected rather than a fault.
    let f = imu_board(false, false).obs().flags_byte();
    assert_eq!(f & 0x41, 0x00, "unstaged");
    assert_eq!(f, 0x00, "and nothing else is set at boot on an idle board");

    // The whole byte on a live board, so the new bit is pinned in its POSITION and not just in
    // isolation: configured (b0) + live (b1) + staged (b6), with no peer yet (the comms-loss
    // exemption) and no fault.
    assert_eq!(configured_to_run().obs().flags_byte(), 0x43);
}

#[test]
fn imu_live_tracks_read_success() {
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );

    // A single failing read (None) on a configured IMU: not live, and below the loss threshold
    // no fault (single-glitch tolerance).
    let t = control_task(&mut s, None, 1);
    assert!(!s.imu_live);
    assert!(!t.imu_loss);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());

    // A real sample: live, and the attitude output moves off identity.
    let sample = good_sample();
    for _ in 0..250 {
        control_task(&mut s, Some(&sample), 1);
    }
    assert!(s.imu_live);
    let obs = s.obs();
    assert!(obs.imu_configured);
    assert!(!obs.imu_loss);
    assert!(
        obs.pitch_milli_deg != 0,
        "attitude integrated the tilted gravity vector"
    );

    // The read failing again drops the live flag on that tick.
    control_task(&mut s, None, 1);
    assert!(!s.imu_live);
}

#[test]
fn a_failed_read_holds_the_filter_not_zeros() {
    // The core P0-1 fix: a missing sample HOLDS the attitude (skips the update) instead of
    // integrating the zero sample toward level. Freezing at the last-good angle is only safe
    // because the loss fault (below) disengages torque; the filter must never be walked to zero.
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    let good = good_sample();
    for _ in 0..200 {
        control_task(&mut s, Some(&good), 1);
    }
    let held_pitch = s.attitude.pitch_deg;
    let held_roll = s.attitude.roll_deg;
    let held_word = s.block.pitch_word;
    assert!(
        s.obs().pitch_milli_deg != 0,
        "attitude integrated a real tilt"
    );
    // Roll is published (CTRL_OBS roll_milli) off the same held Mahony output as pitch.
    let held_roll_milli = s.obs().roll_milli_deg;

    // A failed read: filter and block words hold, live flag drops, no zero integration.
    control_task(&mut s, None, 1);
    assert!(!s.imu_live);
    assert_eq!(s.attitude.pitch_deg, held_pitch, "pitch held, not zeroed");
    assert_eq!(s.attitude.roll_deg, held_roll, "roll held, not zeroed");
    assert_eq!(
        s.obs().roll_milli_deg,
        held_roll_milli,
        "published roll holds (not zeroed), exactly as pitch"
    );
    assert_eq!(s.block.pitch_word, held_word, "block pitch word held");

    // A run of missed reads keeps holding (never drifts toward level).
    for _ in 0..3 {
        control_task(&mut s, None, 1);
        assert_eq!(s.attitude.pitch_deg, held_pitch);
    }
}

#[test]
fn imu_loss_asserts_after_threshold_and_forces_shutdown() {
    let mut s = configured_to_run();

    // Below the threshold: RUN holds (the fix must tolerate an isolated glitch).
    for _ in 0..(IMU_LOSS_THRESHOLD - 1) {
        let t = control_task(&mut s, None, 1);
        assert!(!t.imu_loss, "within the single-glitch tolerance");
        assert_eq!(t.mode_byte, Mode::Run.as_byte());
    }

    // The threshold-th consecutive fail asserts the fault and forces RUN -> SHUTDOWN.
    let t = control_task(&mut s, None, 1);
    assert!(
        t.imu_loss,
        "N consecutive fails assert imu_loss into fault_a"
    );
    assert!(
        s.imu_health.backoff(),
        "the retry breaker opens at the threshold"
    );
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    assert!(s.obs().imu_loss, "imu_loss is observable");

    // SHUTDOWN -> OFF, MOE cleared.
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
}

#[test]
fn imu_loss_breaker_gates_the_read_on_the_probe_cadence() {
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    let good = good_sample();
    control_task(&mut s, Some(&good), 1);
    // Healthy: every tick is a read tick.
    assert!(s.imu_read_due(1));
    assert!(s.imu_read_due(123));

    // Drive into loss; the breaker opens.
    for _ in 0..IMU_LOSS_THRESHOLD {
        control_task(&mut s, None, 1);
    }
    assert!(s.imu_health.backoff());

    // Breaker open: only cadence multiples are read ticks.
    assert!(!s.imu_read_due(1));
    assert!(!s.imu_read_due(IMU_PROBE_CADENCE - 1));
    assert!(s.imu_read_due(IMU_PROBE_CADENCE));
    assert!(s.imu_read_due(2 * IMU_PROBE_CADENCE));
    assert!(s.imu_read_due(0), "tick 0 is a cadence multiple");
}

#[test]
fn imu_loss_recovers_after_a_clean_stream_and_reenters() {
    let mut s = configured_to_run();

    // Lose the IMU: fault asserts, descend to OFF.
    for _ in 0..IMU_LOSS_THRESHOLD {
        control_task(&mut s, None, 1);
    }
    assert!(s.imu_health.loss());
    control_task(&mut s, None, 1); // SHUTDOWN -> OFF
    assert_eq!(s.mode.mode(), Mode::Off);
    assert!(
        s.imu_health.loss(),
        "the loss latch is not cleared by the OFF pass"
    );

    // The first good read closes the breaker (streak reset) but the fault LATCHES until a proven
    // clean stream (hysteresis): re-entry stays blocked.
    let t = control_task(&mut s, Some(&good_sample()), 1);
    assert!(!s.imu_health.backoff(), "one good read closes the breaker");
    assert!(
        s.imu_health.loss(),
        "the fault holds below the recover threshold"
    );
    assert_eq!(t.mode_byte, Mode::Off.as_byte(), "still faulted: OFF holds");

    // A full clean stream clears the fault, and the still-held request re-enters INIT on the
    // clearing tick.
    let good = good_sample();
    let mut reentered = false;
    for _ in 0..(IMU_RECOVER_THRESHOLD + 5) {
        let t = control_task(&mut s, Some(&good), 1);
        if t.mode_byte == Mode::Init.as_byte() {
            assert!(!s.imu_health.loss(), "loss cleared before re-entry");
            reentered = true;
            break;
        }
        assert_eq!(
            t.mode_byte,
            Mode::Off.as_byte(),
            "faulted until the clean stream"
        );
    }
    assert!(reentered, "recovery re-enables OFF -> INIT");
}

#[test]
fn unconfigured_board_never_loses_imu() {
    // The master (no IMU configured): a None sample every tick is ABSENCE, not loss. The mode
    // machine stays healthy and the retry breaker never engages.
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    hold_power(&mut s);
    for _ in 0..(IMU_LOSS_THRESHOLD as usize + 300) {
        let t = control_task(&mut s, None, 1);
        assert!(!t.imu_loss);
    }
    assert!(!s.imu_health.loss());
    assert!(!s.imu_health.backoff(), "absence never opens the breaker");
    assert_eq!(
        s.mode.mode(),
        Mode::Run,
        "no IMU: healthy, reaches and holds RUN"
    );
    assert!(
        s.imu_read_due(1),
        "every tick is a read tick with the breaker closed"
    );
    assert!(s.imu_read_due(7));
}

#[test]
fn unconfigured_imu_is_never_live() {
    let mut s = fresh();
    let sample = imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [0, 0, 14000],
        temp_centi_degc: 0,
        still: true,
    };
    control_task(&mut s, Some(&sample), 1);
    assert!(!s.imu_live, "a sample without a configured IMU is not live");
}

// --- OBS snapshot ----------------------------------------------------------------------------

#[test]
fn obs_snapshot_carries_the_pipeline_counters_and_levels() {
    let mut s = fresh();
    hold_power(&mut s);
    s.inbox.accept(CTRL, cyclic(0));
    run_ticks(&mut s, 3); // OFF -> INIT -> READY -> RUN
    let obs = s.obs();
    assert_eq!(obs.control_ticks, 3);
    assert_eq!(obs.input_ticks, 2);
    assert_eq!(obs.mode_byte, Mode::Run.as_byte());
    assert_eq!(obs.moe_bits, 0b11);
    assert_eq!(obs.cyclic_age, 3);
    assert!(!obs.comms_loss);
    assert!(!obs.imu_configured);
    assert_eq!(obs.enact_inits, 1);
    assert_eq!(obs.enact_shutdowns, 0);
}

#[test]
fn out_to_milli_scales_degrees_without_overflow() {
    use base::fixed::Out;
    assert_eq!(super::out_to_milli(Out::from_num(0)), 0);
    assert_eq!(super::out_to_milli(Out::from_num(1.5)), 1500);
    assert_eq!(super::out_to_milli(Out::from_num(-2.25)), -2250);
    // Past the naive Q16 overflow point (~32.7 deg * 1000).
    assert_eq!(super::out_to_milli(Out::from_num(179.0)), 179_000);
    assert_eq!(super::out_to_milli(Out::from_num(-179.0)), -179_000);
}

// --- Slice 6: control dispatch + cyclic (`specs/integration.md` steps 7-8) -------------------

use linkctl::{decode as linkctl_decode, Payload as LPayload, OP_CYCLIC_STATE};

/// Walk the mode machine to RUN. It does NOT open the engagement gate: each test opens it
/// through the gate's real producer (level IMU samples in balance mode, a drive command in
/// throttle mode).
///
/// Its predecessor set `block.gating_field` directly, with "producer out of scope" written next
/// to it. That is exactly why 553 green tests could not see that the row had no producer at all
/// and no throttle could ever engage on silicon (the 2026-07-31 arm session, defect F1). No test
/// in this file writes that row any more.
fn walk_to_run(s: &mut OrchestratorState) {
    hold_power(s);
    let t = run_ticks(s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
}

/// Run `n` ticks holding a drive command fresh (the inbox's staleness window is ~50 ticks), with
/// no IMU sample. The throttle mode's gate input is this demand, so this is how a throttle test
/// engages.
fn drive_ticks(s: &mut OrchestratorState, n: usize, value: i16, steer: i16) -> ControlOutput {
    let mut last = None;
    for k in 0..n {
        if k % 40 == 0 {
            feed_drive(s, value, steer);
        }
        last = Some(control_task(s, None, 1));
    }
    last.expect("at least one tick")
}

/// Pads down with the power button still held (the button releases on ONE low sample, so any
/// input pass during a RUN walk must keep asserting it).
fn pads_on_button_held() -> InputSample {
    InputSample {
        button_asserted: true,
        pad_a_high: true,
        pad_b_high: true,
    }
}

/// Accept a fresh full-throttle drive command.
fn feed_drive(s: &mut OrchestratorState, value: i16, steer: i16) {
    s.inbox.accept(
        CTRL,
        Payload::DriveCmd(DriveCmd {
            kind: DriveKind::Throttle,
            value,
            steer,
        }),
    );
}

// --- The engagement gate, driven THROUGH its producers (defect F1) --------------------------
//
// The 2026-07-31 arm session could not command motion: `block.gating_field` had no producer, so
// the engagement machine's `> 500` gate could never open and no throttle engaged. 553 green host
// tests missed it because every one of them SET the row directly. These drive the gate the way
// the firmware does: an IMU sample (balance) or a drive command (throttle) in, engagement out.

/// A level sample whose up-axis magnitude is the caller's, direction unchanged (the fusion
/// normalizes, so pitch/roll stay at zero however small the count is). This is what isolates the
/// gating threshold from the upright window: same attitude, different up-axis count.
fn level_sample_at(up_axis: i16) -> imu::Sample {
    imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [0, 0, up_axis],
        temp_centi_degc: 2500,
        still: false,
    }
}

/// The battery calibration of the vectors that need a KNOWN battery word: a sensing board whose
/// centivolts equal its counts (slope 10000 uV per count, offset 0), so [`BENCH_CV`] counts
/// reproduce the 36.00 V word the balance vectors were written against.
fn sensing() -> Option<battery::VbattCal> {
    Some(battery::VbattCal::new(10_000, 0))
}

/// The count a [`sensing`] board reads: 3600 centivolts.
const BENCH_CV: u16 = 3600;

/// A balance board (CONTROL_MODE = 1, IMU configured, sensing a known battery word) walked to RUN
/// with pads down, fed `sample`.
fn balance_to_run(sample: &imu::Sample) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    assert_eq!(
        s.obs().control_mode,
        1,
        "balance holds with a configured IMU"
    );
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    for _ in 0..3 {
        control_task(&mut s, Some(sample), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run);
    s
}

#[test]
fn balance_engages_through_the_conditioned_up_axis_producer() {
    // A level board at 1 g: the attitude step conditions the up-axis count into the block row,
    // it crosses 500 within a handful of ticks, and the machine engages. No test writes the row.
    let level = level_sample_at(8192);
    let mut s = balance_to_run(&level);
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "idle before the gate opens");

    for _ in 0..20 {
        control_task(&mut s, Some(&level), 1);
    }
    assert!(
        s.block.gating_field > 500,
        "the producer wrote the row: {}",
        s.block.gating_field
    );
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged through the producer");
}

#[test]
fn balance_never_engages_when_the_up_axis_is_below_the_gravity_threshold() {
    // The SAME attitude (the fusion uses direction only) with an up-axis of 400 counts =
    // 0.049 g, under the 0.061 g edge: the gate stays shut for as long as it is held. This is
    // the gate itself refusing, not the upright window (pitch is identical to the passing case).
    let shallow = level_sample_at(400);
    let mut s = balance_to_run(&shallow);
    for _ in 0..500 {
        control_task(&mut s, Some(&shallow), 1);
    }
    assert!(s.block.gating_field <= 500, "{}", s.block.gating_field);
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "never engaged");
    assert_eq!(s.obs().torque_setpoint, 0);
}

#[test]
fn balance_pickup_disengages_when_the_deck_inverts() {
    // Engaged and level, then inverted: the conditioned cell crosses to negative, the FSM's
    // 20-tick pickup debounce runs, and the machine drops to IDLE. The whole path is producer
    // driven, so this pins the pickup edge's real-world meaning (an upside-down deck).
    let level = level_sample_at(8192);
    let mut s = balance_to_run(&level);
    for _ in 0..500 {
        control_task(&mut s, Some(&level), 1);
    }
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged first");

    let inverted = level_sample_at(-8192);
    for _ in 0..500 {
        control_task(&mut s, Some(&inverted), 1);
    }
    assert!(s.block.gating_field < 0, "{}", s.block.gating_field);
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "pickup dropped it to IDLE");
}

#[test]
fn the_gating_row_ignores_the_attitude_configs_sign_map() {
    // The sample's accel counts arrive ALREADY sign-applied by the IMU crate (its config owns
    // the wiring; `sign_clamp` in `imu::Sample`). A mount whose ATTITUDE config negates the up
    // axis must not flip the gating row: the producer consumes the sample directly, applying
    // no second map. This pins the audit-caught double-application (which cancelled the map on
    // negative-sign mounts and would have inverted balance gating).
    let mut cfg = attitude::Config::default();
    cfg.accel_sign[UP_AXIS] = -1;
    let level = level_sample_at(8192);
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        cfg,
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    for _ in 0..20 {
        control_task(&mut s, Some(&level), 1);
    }
    assert!(
        s.block.gating_field > 500,
        "positive-up sample must read positive regardless of the attitude map: {}",
        s.block.gating_field
    );
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged");
}

#[test]
fn balance_gating_row_holds_across_a_missed_imu_sample() {
    // The row is an IMU-tick product, so it holds when a read fails, exactly as the attitude
    // words do. A dropped sample must not slam the gate shut under live torque.
    let level = level_sample_at(8192);
    let mut s = balance_to_run(&level);
    for _ in 0..100 {
        control_task(&mut s, Some(&level), 1);
    }
    let held = s.block.gating_field;
    control_task(&mut s, None, 1);
    assert_eq!(s.block.gating_field, held, "held across the missed read");
}

#[test]
fn throttle_engages_through_the_demand_gate_and_refuses_below_the_deadband() {
    // Throttle mode's gate input is its own demand magnitude, so a drive command is the whole
    // path. 400 of +-32767 conditions to a reference of 342, under the 500 edge: refused, as the
    // session saw. Nothing about the board's orientation is involved (no IMU need exist).
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    drive_ticks(&mut s, 200, 400, 0);
    assert_eq!(
        s.ctl.fsm.sub_state as u8, 0,
        "under the deadband: no engage"
    );
    assert_eq!(s.obs().torque_setpoint, 0);

    // Raised past the edge: engaged, and the torque word follows the demand.
    drive_ticks(&mut s, 200, 1000, 0);
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged on demand");
    assert!(
        s.obs().torque_setpoint > 0,
        "torque follows: {}",
        s.obs().torque_setpoint
    );
}

#[test]
fn an_over_current_trip_latches_motor_0_and_clears_on_the_off_pass() {
    // `specs/motor-integration.md`, "The hard trip": the firmware calls `raise_over_current(0)`
    // when the period ISR's trip count changes. The chain from there is the existing one: the
    // latch consumes the code the same pass, `FsmInputs::over_current` sends a RUN sub-state to
    // IDLE, `latch_a` drives `fault_a` -> SHUTDOWN, and the latch clears on the OFF pass.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    drive_ticks(&mut s, 400, 1000, 0);
    assert_eq!(
        s.ctl.fsm.sub_state as i8,
        state::RUN_SUBSTATE,
        "engaged in the RUN sub-state"
    );
    assert!(!s.latches[0].is_latched());

    s.raise_over_current(0);
    feed_drive(&mut s, 1000, 0);
    let t = control_task(&mut s, None, 1);
    assert!(
        s.latches[0].is_latched(),
        "the latch is set on the same pass"
    );
    assert!(
        !s.latches[1].is_latched(),
        "motor 1's latch has no producer"
    );
    assert_eq!(
        s.ctl.fsm.sub_state as u8, 0,
        "the RUN sub-state goes IDLE that tick"
    );
    assert_eq!(t.sub_state, 0);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte(), "fault_a -> SHUTDOWN");
    assert_ne!(
        u16::from(s.obs().event_levels) & EV_LATCH_A,
        0,
        "attributed to latch_a"
    );
    // Not the motor-side level: the trip reaches fault_a through the latch alone.
    assert_eq!(s.motor_fault, 0);
    assert!(!s.obs().motor_fault);

    // SHUTDOWN lands in OFF: every latch clears on that pass, with MOE withdrawn.
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
    assert_eq!(
        t.torque_setpoint, 0,
        "the demand has gone to zero through the FSM"
    );
    assert!(!s.latches[0].is_latched(), "cleared on the OFF pass");

    // Re-engage is possible: the held request walks the machine back up and a demand engages.
    run_ticks(&mut s, 3);
    assert_eq!(s.mode.mode(), Mode::Run);
    drive_ticks(&mut s, 400, 1000, 0);
    assert_ne!(
        s.ctl.fsm.sub_state as u8, 0,
        "re-engaged after the OFF pass"
    );
}

#[test]
fn an_over_current_raised_while_not_running_latches_nothing() {
    // The latch task is gated on RUN (`running_enable`), and an OFF pass clears the whole latch,
    // mailbox included: a code raised while OFF (a disarmed board's ISR still evaluates every
    // period) neither latches nor lingers to latch on a later arm.
    let mut s = fresh();
    s.raise_over_current(0);
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert!(!s.latches[0].is_latched());
    assert_eq!(
        s.latches[0].fault_code, 0,
        "the OFF pass cleared the mailbox"
    );
    hold_power(&mut s);
    let t = run_ticks(&mut s, 4);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
    assert!(!s.latches[0].is_latched());
}

#[test]
fn throttle_engages_in_both_directions() {
    // The gate input is a MAGNITUDE, so reverse engages too, and the RUN pickup path (which
    // counts while the gate input is negative) stays inert in throttle mode - as inert as the
    // step-off path the mode already parameterizes off.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    drive_ticks(&mut s, 400, -1000, 0);
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "reverse engages");
    assert!(
        s.obs().torque_setpoint < 0,
        "reverse torque: {}",
        s.obs().torque_setpoint
    );
}

#[test]
fn drive_value_1000_is_the_first_motion_value_and_500_is_not() {
    // The numbers `specs/arm-session.md` D4 tells the bench to use, pinned here so the runbook
    // and the firmware cannot drift. The word is DRIVE_CMD.value on the +-32767 frame, which is
    // now the only demand word on the wire at all: 1000 -> command 30 -> reference 855, clear of
    // the 500 edge with ~1.7x margin; 500 -> command 15 -> reference 427, under it.
    for (throttle, want_engage) in [(500i16, false), (1000i16, true)] {
        let mut s = fresh();
        hold_power(&mut s);
        run_ticks(&mut s, 3);
        drive_ticks(&mut s, 300, throttle, 0);
        assert_eq!(
            s.ctl.fsm.sub_state as u8 != 0,
            want_engage,
            "DRIVE value {throttle}"
        );
    }
}

#[test]
fn the_rider_level_does_not_gate_throttle_mode_engagement() {
    // The session's own hypothesis for the D4 refusal was a missing `--rider`. It is not the
    // cause: the pad/rider gate is balance-only (the mode parameterizes it off), so throttle
    // mode engages identically with and without it. Keeping this pinned stops the runbook from
    // re-acquiring a superstition.
    for rider in [false, true] {
        let mut s = fresh();
        hold_power(&mut s);
        run_ticks(&mut s, 3);
        if rider {
            s.inbox.accept(
                CTRL,
                Payload::Inputs(linkctl::Inputs {
                    buttons: 0,
                    rider: linkctl::Inputs::RIDER_PRESENT,
                }),
            );
        }
        drive_ticks(&mut s, 200, 1000, 0);
        assert_ne!(
            s.ctl.fsm.sub_state as u8, 0,
            "engaged regardless of rider={rider}"
        );
    }
}

#[test]
fn imu_absent_balance_demotes_to_throttle_with_mode_fault() {
    // CONTROL_MODE = 1 (Balance) without a configured IMU: the boot seam demotes with the fault.
    let s = OrchestratorState::new(
        1,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    let obs = s.obs();
    assert_eq!(obs.control_mode, 0, "demoted to Throttle");
    assert!(obs.mode_fault);

    // With the IMU configured, Balance holds and no fault raises.
    let s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    assert_eq!(s.obs().control_mode, 1);
    assert!(!s.obs().mode_fault);

    // The default byte (0) is Throttle on any board, no fault.
    let s = OrchestratorState::new(
        0,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    assert_eq!(s.obs().control_mode, 0);
    assert!(!s.obs().mode_fault);
}

#[test]
fn quiet_pipeline_holds_substate_zero_and_zero_torque() {
    // With no drive, no rider, and the gate closed, the dispatch runs every tick and the
    // sole-writer row stays 0 through sub-state 0 (no fabricated output).
    let mut s = fresh();
    hold_power(&mut s);
    for _ in 0..100 {
        let t = control_task(&mut s, None, 1);
        assert_eq!(t.sub_state, 0);
        assert_eq!(t.torque_setpoint, 0);
        assert_eq!(s.ctl.fsm.torque_setpoint, 0);
    }
}

#[test]
fn throttle_mode_produces_torque_then_decays_to_neutral_on_stale_drive() {
    // The throttle path end to end: RUN + open gate + a fresh full drive -> the EFeru
    // conditioning ramps, the engagement soft-start envelopes, torque goes positive; the drive
    // going stale decays the reference to neutral at the consumer (the rate limiter IS the
    // decay ramp) and torque returns to exactly 0.
    let mut s = fresh(); // CONTROL_MODE = 0 (Throttle)
    walk_to_run(&mut s);

    let mut peak = 0i16;
    for k in 0..300 {
        if k % 40 == 0 {
            feed_drive(&mut s, 32767, 0); // keep the drive fresh (< 50-tick staleness)
        }
        let t = control_task(&mut s, None, 1);
        assert_eq!(
            t.torque_setpoint, s.ctl.fsm.torque_setpoint,
            "sole-writer row"
        );
        assert!(t.torque_setpoint >= 0);
        assert!(t.torque_setpoint <= 28500, "never beyond the envelope cap");
        peak = peak.max(t.torque_setpoint);
    }
    assert!(peak > 20_000, "full drive reached authority, got {peak}");
    assert_eq!(
        s.ctl.fsm.sub_state as u8, 3,
        "soft-start promoted to RUN sub-state"
    );

    // The drive goes stale: within the 51-tick staleness window plus the conditioning decay
    // (~35 ticks full-scale) the torque returns to exactly zero and stays there.
    let mut zero_seen_at = None;
    for k in 0..200 {
        let t = control_task(&mut s, None, 1);
        if t.torque_setpoint == 0 && zero_seen_at.is_none() {
            zero_seen_at = Some(k);
        }
    }
    let z = zero_seen_at.expect("stale drive decays to neutral");
    assert!(z <= 120, "decay completes promptly, got {z}");
    assert_eq!(s.ctl.fsm.torque_setpoint, 0);
}

#[test]
fn balance_engagement_walks_substates_and_stays_within_envelope() {
    // The balance assembly end to end: RUN + rider (pads) + open gate + a steer drive; the FSM
    // walks IDLE -> ARMING -> RUN sub-states, the torque tracks the balance law's word and
    // never exceeds the soft-start envelope (|torque| <= 200/tick * ticks-since-engage).
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    let level = level_sample(); // a live, level IMU so the board stays in RUN (no IMU-loss fault)
    walk_to_run(&mut s);
    // Rider on both pads, power still held (one low sample would release the button).
    input_task(&mut s, &pads_on_button_held());
    assert!(s.rider_present);

    assert_eq!(s.ctl.fsm.sub_state as u8, 0);
    let mut engaged_at = None;
    for k in 0..200 {
        if k % 40 == 0 {
            feed_drive(&mut s, 0, 20_000); // a held steer command (kept fresh)
        }
        let t = control_task(&mut s, Some(&level), 1);
        if engaged_at.is_none() && t.sub_state != 0 {
            engaged_at = Some(k);
        }
        if let Some(e) = engaged_at {
            // The soft-start property: the envelope admits at most 200/tick since engage.
            let env_bound = 200i32 * (k - e + 1);
            assert!(
                (t.torque_setpoint as i32).abs() <= env_bound.min(28500),
                "torque {} exceeds envelope bound {} at tick {}",
                t.torque_setpoint,
                env_bound,
                k
            );
        }
    }
    // Tick 1, not tick 0: the gate input is now the CONDITIONED up-axis count, and the stock
    // 0.02/0.98 IIR needs two samples of this 2 g level fixture to climb past 500 (327 then
    // 649). That one-tick cost is the producer being real; it is also why a board cannot engage
    // on the first sample after reset.
    assert_eq!(
        engaged_at,
        Some(1),
        "engaged on the first tick the conditioned gate clears 500"
    );
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "promoted to RUN sub-state");
    assert!(
        s.ctl.fsm.torque_setpoint != 0,
        "the law's word drives a nonzero setpoint"
    );

    // Sub-state 0 forces zero: the rider stepping off (power still held: the system stays in
    // RUN) winds the machine down via the step-off debounce, and once sub-state 0 is reached
    // the setpoint is 0.
    let button_only = InputSample {
        button_asserted: true,
        ..Default::default()
    };
    input_task(&mut s, &button_only); // one low keeps pads on
    input_task(&mut s, &button_only); // two lows: rider off
    assert!(!s.rider_present);
    for k in 0..40 {
        feed_drive(&mut s, 0, 20_000);
        let t = control_task(&mut s, Some(&level), 1);
        if t.sub_state == 0 {
            // The tick AFTER reaching IDLE zeroes the mirror; from there the setpoint is 0.
            let t2 = control_task(&mut s, Some(&level), 1);
            assert_eq!(t2.sub_state, 0);
            assert_eq!(t2.torque_setpoint, 0, "sub-state 0 forces a zero setpoint");
            return;
        }
        assert!(k < 39, "wind-down never completed");
    }
}

#[test]
fn a_fault_shutdown_resets_the_engagement_machine_so_re_entry_soft_starts() {
    // `specs/motor-integration.md` prerequisite 4 (a stage-4 gate). `imu_loss` is a `fault_a`
    // producer with NO input into the stock engagement FSM (whose aborts are comms-loss / stop /
    // over-current only), so it drives the mode machine RUN -> SHUTDOWN -> OFF while leaving the
    // engagement sub-state at RUN and the envelope at its cap. The OFF pass resets the machine,
    // so the next entry to RUN starts from IDLE and pays the soft-start ramp again.
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    let level = level_sample();
    walk_to_run(&mut s);
    input_task(&mut s, &pads_on_button_held());
    assert!(s.rider_present);

    // Engage and promote to the RUN sub-state, winding the envelope up.
    for k in 0..200 {
        if k % 40 == 0 {
            feed_drive(&mut s, 0, 20_000);
        }
        control_task(&mut s, Some(&level), 1);
    }
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "engaged in the RUN sub-state");
    assert!(s.ctl.fsm.env > 0, "the soft-start envelope is wound up");

    // The fault: IMU_LOSS_THRESHOLD failed reads assert `imu_loss` into `fault_a`. The FSM sees
    // none of it (its own inputs are quiescent), which is exactly the case this reset covers.
    let mut saw_off = false;
    for _ in 0..(IMU_LOSS_THRESHOLD as usize + 4) {
        let t = control_task(&mut s, None, 1);
        saw_off |= t.mode_byte == Mode::Off.as_byte();
    }
    assert!(saw_off, "the IMU-loss fault drives the machine to OFF");
    assert_eq!(
        s.ctl.fsm.sub_state as u8, 0,
        "the engagement machine is reset on the OFF pass"
    );
    assert_eq!(s.ctl.fsm.env, 0, "the envelope resets with it");
    assert_eq!(s.ctl.fsm.torque_setpoint, 0, "and the setpoint with it");

    // Re-entry: healthy reads clear the loss latch (IMU_RECOVER_THRESHOLD successes), the mode
    // machine walks OFF -> INIT -> READY -> RUN, and the machine re-engages from IDLE. The
    // property under test is that the soft-start envelope is paid again: at most 200/tick of
    // authority since re-engagement, which a surviving RUN sub-state would have skipped.
    let mut re_engaged_at = None;
    for k in 0..300 {
        if k % 40 == 0 {
            feed_drive(&mut s, 0, 20_000);
        }
        let t = control_task(&mut s, Some(&level), 1);
        if re_engaged_at.is_none() && t.sub_state != 0 {
            re_engaged_at = Some(k);
        }
        if let Some(e) = re_engaged_at {
            let env_bound = 200i32 * (k - e + 1);
            assert!(
                (t.torque_setpoint as i32).abs() <= env_bound.min(28500),
                "torque {} skips the soft-start envelope bound {} at tick {}",
                t.torque_setpoint,
                env_bound,
                k
            );
        }
    }
    let e = re_engaged_at.expect("the machine re-engages once the IMU recovers");
    assert!(
        e >= IMU_RECOVER_THRESHOLD as i32,
        "re-engagement waits for the loss latch to clear, got tick {e}"
    );
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "and promotes to RUN again");
}

#[test]
fn peer_lockdown_forces_substate_zero_and_zero_torque() {
    // The CYCLIC_STATE lockdown flag reaches its consumer: an engaged throttle board drops to
    // sub-state 0 with a zero setpoint while the peer asserts lockdown.
    let mut s = fresh();
    walk_to_run(&mut s);
    // Ride the soft-start through to RUN sub-state (the orient==0 ARMING arm carries no abort
    // in the binary; the immediate-stop inputs bite in RUN).
    for k in 0..200 {
        if k % 40 == 0 {
            feed_drive(&mut s, 32767, 0);
        }
        control_task(&mut s, None, 1);
    }
    assert!(s.ctl.fsm.torque_setpoint > 0);
    assert_eq!(s.ctl.fsm.sub_state as u8, 3);

    // The peer asserts lockdown (bit 7). Two ticks: the stop lands, then the IDLE arm zeroes.
    s.inbox.accept(CTRL, cyclic(CyclicState::FLAG_LOCKDOWN));
    control_task(&mut s, None, 1);
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.sub_state, 0, "lockdown forces sub-state 0");
    assert_eq!(t.torque_setpoint, 0, "and a zero setpoint");
}

#[test]
fn a_substate_tie_feeds_the_latches_in_run() {
    // The latches consume the engagement sub-state as a_substate with running_enable following
    // RUN: idling in RUN (a = 0, b = 0) counts toward the stock 10-minute latch; an engaged
    // healthy walk resets the counter; outside RUN the latch is inert.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3); // RUN, gate closed: sub-state stays 0 -> UNHEALTHY counts
    let c0 = s.latches[0].fault_counter;
    run_ticks(&mut s, 10);
    assert_eq!(
        s.latches[0].fault_counter,
        c0 + 10,
        "idle-in-RUN counts (the stock idle timeout)"
    );
    assert_eq!(s.latches[0].a_substate, 0);

    // Engage (throttle path): sub-state leaves 0, the tie feeds nonzero a_substate and the
    // HEALTHY predicate (a != 0, b == 0) resets the counter. The drive command IS the gate
    // input in throttle mode, so this engages through the producer.
    //
    // This board's wheel is STATIONARY (the block row is 0), which is the only reason an ARMING
    // sub-state reads healthy here: the predicate's other half is `a == 3`, and the ramp has not
    // promoted yet. `a_turning_wheel_counts_the_step_off_latch_outside_run` below is the same walk
    // with the row carrying a rolling wheel.
    drive_ticks(&mut s, 20, 32767, 0);
    assert!(
        s.latches[0].a_substate != 0,
        "the tie carries the sub-state"
    );
    assert_eq!(s.latches[0].fault_counter, 0, "healthy resets the counter");

    // Released to OFF: running_enable drops, the latch freezes (inert outside RUN).
    input_task(&mut s, &InputSample::default());
    run_ticks(&mut s, 3);
    assert_eq!(s.obs().mode_byte, 0);
    assert_eq!(s.latches[0].running_enable, 0);
}

/// **The step-off latch's `b_motion` input is live, now that `block.wheel_speed` has a writer**
/// (`specs/link-control.md`, "`wheel_speed` carries the raw edge count": the firmware's 250 Hz
/// shell folds the period ISR's edge count into motor 0's row before each pass).
///
/// The health rule is `(a != 0 || b != 0) && (a == 3 || b == 0)`
/// (`crates/state/src/fault.rs`, `FaultLatch::is_healthy`), so a nonzero `b` changes the verdict
/// everywhere the sub-state is not RUN. While the row was a structural 0 on every board, the whole
/// suite only ever exercised the `b == 0` half of that conjunction, which is what this walks with a
/// rolling wheel instead. What it establishes is that the new input cannot latch a fault on its own:
/// every tick it makes UNHEALTHY is a tick of a 150,000-tick count that RUN resets outright.
#[test]
fn a_turning_wheel_counts_the_step_off_latch_outside_run() {
    let mut s = fresh();
    // A rolling wheel on the block row, the word the firmware's writer puts there.
    s.block.wheel_speed[0] = 9;
    hold_power(&mut s);
    run_ticks(&mut s, 3); // RUN, gate closed: a == 0 with b != 0 is UNHEALTHY
    assert_eq!(s.latches[0].b_motion, 9, "the tie carries the block row");
    let c0 = s.latches[0].fault_counter;
    run_ticks(&mut s, 10);
    assert_eq!(s.latches[0].fault_counter, c0 + 10, "idle-in-RUN counts");

    // Engaged but NOT yet promoted (the envelope ramp holds ARMING for ~144 ticks): a == 1 with
    // b != 0 is UNHEALTHY, so the counter keeps climbing where a stationary wheel reset it on the
    // first engaged tick (`a_substate_tie_feeds_the_latches_in_run` above).
    drive_ticks(&mut s, 20, 32767, 0);
    assert_eq!(s.ctl.fsm.sub_state as u8, 1, "still ARMING");
    assert_eq!(
        s.latches[0].fault_counter,
        c0 + 30,
        "a turning wheel outside RUN keeps counting"
    );

    // The ramp promotes to RUN sub-state 3: `a == 3` is healthy whatever the wheel is doing, which
    // is the point of that half of the rule, and the counter is reset outright.
    drive_ticks(&mut s, 200, 32767, 0);
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "promoted to RUN");
    assert_eq!(s.latches[0].b_motion, 9, "the wheel is still turning");
    assert_eq!(s.latches[0].fault_counter, 0, "RUN resets it");
    assert!(!s.latches[0].is_latched(), "nothing latched on the way");
}

/// **The arm-time mode install applies on the pass that GRANTS the MOE allowance**
/// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read"), which is
/// why the install carries no armed-request refusal of its own.
///
/// `arm::ArmStep::ReReadValues` is the FIRST arm step and the arm sequence runs on the pass the
/// mode machine sets `MoeGate`, so `any_moe_allowed()` is true at the install. A refusal keyed on
/// it (the disarmed-only switch seam this replaced) would therefore have refused EVERY arm, not
/// merely been redundant. The write is gated instead by R4, one layer up, where the board is armed
/// in the sense that matters: the byte read here can only have reached flash while disarmed, and
/// `ArmStep::SetMoe` is the last arm step.
#[test]
fn the_arm_time_mode_install_applies_on_the_pass_that_grants_the_allowance() {
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    hold_power(&mut s);
    // OFF -> INIT, then the INIT pass: MOE is granted here, and this is the pass whose
    // `arm::hw::enact` runs the arm sequence and with it the value-row install.
    run_ticks(&mut s, 2);
    assert!(
        s.mode.any_moe_allowed(),
        "the allowance stands at the install"
    );
    s.ctl.sf.v_filt = 123;
    re_apply_control_values(
        &mut s,
        1,
        0,
        0,
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        store::CONTROL_POS_CLAMP.default(),
    );
    assert_eq!(s.obs().control_mode, 1, "the mode installed");
    assert_eq!(
        s.ctl.sf,
        control::SfState::default(),
        "and the balance producer record went with the change"
    );
}

/// Every record a controller change has to leave clean, as text: the balance law's record, the
/// engagement machine and the throttle conditioner's carries. `FsmState` carries no `PartialEq`
/// (nothing in the firmware compares two of them), so the comparison is on `Debug`, which names
/// every cell.
fn controller_records(s: &OrchestratorState) -> std::string::String {
    std::format!(
        "{:?}|{:?}|{:?}",
        s.ctl.sf,
        s.ctl.fsm,
        s.ctl.dispatch.throttle
    )
}

/// **The ACTIVE CONTROLLER may change between two arms, and the second arm starts it from zero**
/// (`specs/integration.md`, "When a stored value takes effect: the arm-time re-read": the mode
/// machine must tolerate the active controller changing between two arms).
///
/// Three arms of one boot, Balance -> Throttle -> Balance, driven through `control_task` with the
/// install called where the arm path calls it (the pass that grants the MOE allowance, after that
/// pass's own dispatch step). What it pins:
///
/// - the engagement machine survives the change because every disengage crosses OFF, where
///   `reset_engagement` runs, so no sub-state or soft-start envelope carries into the new
///   controller;
/// - the LAW's carries do NOT clear at OFF (the speed filter, the live gains and terms, the
///   position latches), which is the stale state a mode change has to deal with, and the install
///   is what deals with it: the arming pass runs one dispatch pass of the OUTGOING controller
///   before the install, so clearing them anywhere earlier than the install would leave that
///   pass's carries behind;
/// - the incoming controller is the one driving afterwards: in Throttle a tilted deck commands
///   nothing, and back in Balance the same tilt commands torque again.
#[test]
fn the_active_controller_can_change_between_two_arms() {
    let tilted = imu::Sample {
        accel_raw: [2000, 0, 8000],
        ..level_sample()
    };
    let pristine = controller_records(&balance_board(sf_start()));
    let mut s = balance_board(sf_start());

    // ARM ONE, Balance: pads down, a drive stream, a tilted deck. The law engages and every
    // producer record takes on state.
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    for k in 0..60 {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run);
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "the machine engaged");
    assert_ne!(s.obs().pre_env_torque, 0, "the balance law is driving");
    assert_ne!(
        controller_records(&s),
        pristine,
        "arm one left state behind"
    );

    // Disengage to OFF. The engagement machine resets here; the law's carries do NOT, which is
    // exactly why the install has to replace them.
    release_power(&mut s);
    for _ in 0..3 {
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Off);
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "the machine is disengaged");
    assert_ne!(
        s.ctl.sf.terms, [0; 4],
        "the OFF pass resets the engagement machine, not the law's carries"
    );

    // ARM TWO: `CONTROL_MODE` was written to Throttle while disarmed. Re-engage, and install on the
    // pass that grants the allowance, after its dispatch step as the arm path does.
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    control_task(&mut s, Some(&tilted), 1); // OFF -> INIT
    control_task(&mut s, Some(&tilted), 1); // the INIT pass: MOE granted, the arm sequence runs
    assert!(s.mode.any_moe_allowed(), "this is the arming pass");
    re_apply_control_values(
        &mut s,
        0,
        0,
        0,
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        store::CONTROL_POS_CLAMP.default(),
    );
    assert_eq!(s.obs().control_mode, 0, "Throttle from this arm on");
    assert_eq!(
        controller_records(&s),
        pristine,
        "the incoming controller starts from zero, the outgoing pass's carries included"
    );

    // And Throttle is what runs: a neutral stick walks the conditioned reference down to zero
    // through its rate limit, and the tilted deck then holds it there, where the same tilt was
    // driving in arm one.
    for _ in 0..400 {
        feed_drive(&mut s, 0, 0);
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run, "the run holds across the change");
    for _ in 0..20 {
        control_task(&mut s, Some(&tilted), 1);
        assert_eq!(
            s.obs().pre_env_torque,
            0,
            "a tilted deck is not a throttle demand"
        );
    }

    // ARM THREE, back to Balance, with the throttle conditioner's carries live this time.
    for k in 0..60 {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_ne!(
        s.obs().pre_env_torque,
        0,
        "the throttle conditioner is what drives now"
    );
    assert_ne!(
        controller_records(&s),
        pristine,
        "arm two left state behind"
    );
    release_power(&mut s);
    for _ in 0..3 {
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Off);
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    control_task(&mut s, Some(&tilted), 1);
    control_task(&mut s, Some(&tilted), 1);
    assert!(s.mode.any_moe_allowed());
    re_apply_control_values(
        &mut s,
        1,
        0,
        0,
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        store::CONTROL_POS_CLAMP.default(),
    );
    assert_eq!(s.obs().control_mode, 1, "Balance again");
    assert_eq!(
        controller_records(&s),
        pristine,
        "including the throttle carries the balance law never reads"
    );
    for _ in 0..20 {
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_ne!(
        s.obs().pre_env_torque,
        0,
        "the same tilt commands torque again"
    );
}

#[test]
fn cyclic_tx_is_gated_on_an_assigned_address_and_round_trips_linkctl() {
    let mut a = fresh();
    a.block.wheel_speed[0] = 1000;
    a.block.roll_word = 800;
    // Rider on A's pads (the LOCAL rider is what TX reports).
    input_task(
        &mut a,
        &InputSample {
            pad_a_high: true,
            pad_b_high: true,
            ..Default::default()
        },
    );

    // Unassigned: no emission (pre-assignment there is no peer contract).
    assert!(cyclic_tx(&a, false).is_none());

    // Assigned: the block words leave stock-native, unrescaled.
    let c = cyclic_tx(&a, true).expect("addressed board emits");
    assert_eq!(c.wheel_speed, 1000);
    assert_eq!(c.roll, 800);
    assert_eq!(c.battery, 0); // no local sense and no peer: the effective word is UNKNOWN
    assert_eq!(c.mode, 0);
    assert_eq!(c.fault, 0, "a quiet board asserts no producer");
    assert!(c.rider_present());
    assert!(!c.leader());
    assert!(!c.lockdown());
    assert_eq!(
        c.peer_demand, 0,
        "a board that leads nothing commands nothing"
    );

    // The wire round trip against linkctl: encode, decode by opcode, accept into a peer inbox.
    let mut wire = [0u8; CyclicState::ENCODED_LEN];
    let n = c.encode(&mut wire);
    assert_eq!(
        n,
        CyclicState::ENCODED_LEN,
        "the appended block went out too"
    );
    let payload = linkctl_decode(OP_CYCLIC_STATE, &wire[..n]).expect("decodes");
    let mut b = fresh();
    match payload {
        LPayload::CyclicState(pc) => {
            assert_eq!(pc, c, "byte-faithful round trip");
            b.inbox.accept(CTRL, LPayload::CyclicState(pc));
        }
        other => panic!("wrong family: {other:?}"),
    }
    assert_eq!(b.inbox.peer().unwrap(), c);
}

/// The appended block the emission carries (`specs/link-control.md`, the `CYCLIC_STATE` layout):
/// the current window the firmware folded in, and the two per-boot constants it was built with.
///
/// The window words are pass-through, which is the claim worth pinning: the orchestrator does not
/// rescale, filter or re-derive them (the limiter acts on the samples themselves, in the 16 kHz
/// ISR), so what a controller reads is what the ISR measured.
#[test]
fn the_cyclic_payload_carries_the_window_and_the_boards_identity() {
    let mut a = fresh();
    a.phase = PhaseWindow {
        peak: 1_234,
        mean: 567,
        duty_on: 1_125,
    };

    let obs = cyclic_tx(&a, true).expect("addressed board emits").obs;
    assert_eq!(obs.phase_peak, 1_234);
    assert_eq!(obs.phase_mean, 567);
    assert_eq!(obs.duty_on, 1_125);
    assert_eq!(obs.boot_tag, TEST_IDENTITY.boot_tag);
    assert_eq!(obs.chip, TEST_IDENTITY.chip);

    // The BLE port's decimated emission is the SAME payload, block and all: one builder, two
    // rates (`BLE_CYCLIC_DIVISOR`).
    assert_eq!(ble_cyclic_tx(&a, true).expect("tick 0 emits").obs, obs);

    // A board with no motor brought up carries a zeroed window, and that is the only reading a
    // zeroed window has: a board carrying no current. There is no "did not say" to confuse it
    // with, because every payload the codec accepts carries the block.
    let quiet = cyclic_tx(&fresh(), true).expect("emits").obs;
    assert_eq!(
        (quiet.phase_peak, quiet.phase_mean, quiet.duty_on),
        (0, 0, 0)
    );
}

#[test]
fn peer_rider_flag_reaches_the_engage_gate() {
    // The cyclic rider flag is a consumption-side fold: a board with NO local pads engages the
    // balance machine once its peer reports a rider.
    let mut b = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    let level = level_sample(); // a live, level IMU so the board stays in RUN (no IMU-loss fault)
    walk_to_run(&mut b);
    for k in 0..30 {
        if k % 20 == 0 {
            feed_drive(&mut b, 0, 20_000);
        }
        let t = control_task(&mut b, Some(&level), 1);
        assert_eq!(t.sub_state, 0, "no rider anywhere: no engagement");
    }

    // The peer's cyclic carries the rider flag (and must stay fresh against comms_loss).
    for k in 0..30 {
        if k % 20 == 0 {
            b.inbox.accept(CTRL, cyclic(CyclicState::FLAG_RIDER));
            feed_drive(&mut b, 0, 20_000);
        }
        control_task(&mut b, Some(&level), 1);
    }
    assert!(
        b.ctl.fsm.sub_state as u8 != 0,
        "the mirrored rider engages the machine"
    );
}

#[test]
fn peer_wheel_speed_reaches_ref_36_in_the_sub2_reference() {
    // The engagement blend's peer-speed input (`ref_36`): on the orientation != 0 path the
    // sub-2 reference is (7*local - 6*peer)*50/100 with local = 0 and pitch rate 0, so the
    // torque setpoint lands on exactly -(6*peer)/2: the peer word observably drives the output.
    let mut b = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    let level = level_sample(); // a live, level IMU so the board stays in RUN (no IMU-loss fault)
    b.block.orientation_nz = true;
    walk_to_run(&mut b);
    input_task(&mut b, &pads_on_button_held());

    // Peer cyclic with wheel_speed 1000, kept fresh through the ~144-tick ARMING ramp (the
    // orientation != 0 abort includes comms_loss).
    let peer = CyclicState {
        pitch: 0,
        roll: 0,
        wheel_speed: 1000,
        battery: 3600,
        mode: 3,
        fault: 0,
        peer_demand: 0,
        wheel_pos: 0,
        flags: 0,
        obs: PEER_OBS,
        wheel_rate: 0,
    };
    for k in 0..160 {
        if k % 20 == 0 {
            b.inbox.accept(CTRL, Payload::CyclicState(peer));
        }
        control_task(&mut b, Some(&level), 1);
    }
    assert_eq!(b.ctl.fsm.sub_state as u8, 2, "ARMING promoted to sub-2");
    assert_eq!(
        b.ctl.fsm.torque_setpoint, -3000,
        "torque = -(6 * peer_wheel)/2: ref_36 carries the peer word"
    );
}

#[test]
fn tripped_latch_clears_on_the_off_pass_and_reentry_succeeds() {
    // The latch-clear discipline (integration.md step 4, the slice-6 audit fold): the one-way
    // latch trips (the 10-minute idle-in-RUN timeout, counter preset at the brink per the
    // state-crate pattern), forces the SHUTDOWN descent, and the OFF pass, the power-cycle
    // analog of stock's SELF_HOLD release, clears it whole; re-entry then succeeds through the
    // normal OFF -> INIT gates.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3); // RUN, engagement gate closed: idle-in-RUN counts
    s.latches[0].fault_counter = 149_999;

    let t = control_task(&mut s, None, 1);
    assert!(s.latches[0].is_latched(), "idle-in-RUN tripped the latch");
    assert_eq!(
        t.mode_byte,
        Mode::Shutdown.as_byte(),
        "the latched fault_a forces the descent"
    );

    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert!(
        !s.latches[0].is_latched(),
        "the OFF pass is the power-cycle analog: latch cleared"
    );
    assert_eq!(s.latches[0].fault_counter, 0, "the whole unit resets");

    // Re-entry through the normal gates: the still-held request walks OFF -> INIT.
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Init.as_byte(), "re-entry succeeds");
}

/// **A consumer that must retry once per ENGAGE keys on `off_request_clear`, not on the OFF pass**
/// (`specs/integration.md`, "A refused re-read refuses the ARM, not the boot", the release
/// paragraph). This is the behaviour that makes the distinction necessary, driven here at the level
/// where it is decided.
///
/// The three lines that model the firmware are its `control_task_cb` exactly
/// (`crates/firmware/src/main.rs`): the motor-side level is folded into `motor_fault` BEFORE the
/// pass, and the arm layer is enacted AFTER it, attempting an arm whenever the mode machine allows
/// MOE and no fault is held. This board's value row refuses every attempt (a bad `imu.axis_role`
/// written over BLE), so `refused` stands in for `arm::hw`'s `RE_READ_REFUSED`, which is a
/// target-only static.
///
/// With the request HELD, the right answer is ONE attempt and then a settled OFF. An OFF-pass
/// release instead gives a cycle every five ticks: the request is a level, re-seeded each tick, so
/// OFF to INIT is taken the moment the fault clears. That would step the enact records forever, run
/// the refusal count to its saturation in seconds, and leave R4 answering `CFG_ARMED` on the three
/// ticks in five that are not OFF, refusing the corrective write that is the only way out.
#[test]
fn a_held_request_against_a_refusing_arm_settles_in_off_instead_of_cycling() {
    let mut s = fresh();
    hold_power(&mut s);
    let mut refused = false;
    let mut attempts = 0;
    let mut modes = std::vec::Vec::new();
    let mut inits = 0;
    for _ in 0..40 {
        // The firmware's fold, before the pass that consumes it.
        s.motor_fault = if refused { EV_MOTOR_ARM } else { 0 };
        let level = refused;
        let out = control_task(&mut s, None, 1);
        // The firmware's `arm::hw::enact`, after it.
        if out.moe[0] && !level {
            refused = true;
            attempts += 1;
        }
        if out.off_request_clear {
            refused = false;
        }
        if out.init.is_some() {
            inits += 1;
        }
        modes.push(out.mode_byte);
    }
    assert_eq!(
        attempts, 1,
        "one arm attempt per engage, not one per five ticks"
    );
    assert_eq!(inits, 1, "and one INIT record, not eight");
    assert!(
        modes[6..].iter().all(|m| *m == Mode::Off.as_byte()),
        "the board must sit in OFF with the refusal held: {modes:?}"
    );
    assert!(
        refused,
        "the refusal is still held, so a config write is accepted"
    );
    assert!(
        s.latches.iter().all(|l| !l.is_latched()),
        "the latch clears still ride the OFF half of the pass"
    );
}

/// The complement: the engage ENDS when the request drops, and the next one proceeds.
///
/// Same model as the test above. Dropping the request is what releases the refusal, and raising it
/// again walks OFF to INIT, where the arm layer re-reads: exactly one attempt per engage, however
/// long either one is held.
#[test]
fn dropping_the_request_releases_the_refusal_and_the_next_engage_re_reads() {
    let mut s = fresh();
    hold_power(&mut s);
    let mut refused = false;
    let mut attempts = 0;
    let mut released = 0;
    let step = |s: &mut OrchestratorState,
                refused: &mut bool,
                attempts: &mut usize,
                released: &mut usize| {
        s.motor_fault = if *refused { EV_MOTOR_ARM } else { 0 };
        let level = *refused;
        let out = control_task(s, None, 1);
        if out.moe[0] && !level {
            *refused = true;
            *attempts += 1;
        }
        if out.off_request_clear {
            *refused = false;
            *released += 1;
        }
        out
    };
    // The first engage: one attempt, refused, and no release while the request is held.
    for _ in 0..10 {
        step(&mut s, &mut refused, &mut attempts, &mut released);
    }
    assert_eq!((attempts, released), (1, 0));
    assert!(refused);

    // The request drops. The next pass ends the engage and releases the refusal.
    release_power(&mut s);
    let out = step(&mut s, &mut refused, &mut attempts, &mut released);
    assert!(out.off_request_clear, "OFF with the request clear");
    assert_eq!(released, 1);
    assert!(!refused, "released, so the next engage can re-read");

    // It is raised again (the operator having written a good frame, or not: either way the arm
    // path gets to re-read). The engage proceeds and the arm layer attempts once more.
    hold_power(&mut s);
    let mut saw_init = false;
    for _ in 0..6 {
        let out = step(&mut s, &mut refused, &mut attempts, &mut released);
        saw_init |= out.init.is_some();
    }
    assert!(saw_init, "the next engage walks OFF -> INIT");
    assert_eq!(attempts, 2, "and re-reads: one attempt per engage");
}

#[test]
fn cyclic_tx_rider_flag_is_local_only() {
    // The negative half of the rider fold: a board whose rider level is folded-only (the peer
    // cyclic flag AND the INPUTS mirror both set, local pads off) must TX flags.rider = 0.
    // Mirrored levels never feed back; a regression switching cyclic_tx to the folded
    // rider_level() fails here.
    let mut s = fresh();
    s.inbox.accept(CTRL, cyclic(CyclicState::FLAG_RIDER));
    s.inbox.accept(
        CTRL,
        Payload::Inputs(Inputs {
            buttons: 0,
            rider: Inputs::RIDER_PRESENT,
        }),
    );
    assert!(!s.rider_present, "no local pads");
    let c = cyclic_tx(&s, true).expect("addressed board emits");
    assert!(!c.rider_present(), "TX reports LOCAL pads only");
    // The folded level still feeds local consumption (the engage-gate fold is unchanged).
    assert!(super::dispatch::rider_level(&s));
}

#[test]
fn cyclic_tx_emits_every_second_tick() {
    // Round-4 defect C (specs/link-control.md, "Addressing and emission"): the emission is rate-
    // split to every SECOND TICK (125 Hz nominal), because the blocking polled TX cannot fit the
    // 4 ms tick every tick. Over four consecutive served ticks an addressed board emits twice.
    let mut s = fresh();
    let mut emitted = 0;
    for _ in 0..4 {
        let _ = control_task(&mut s, None, 1);
        if cyclic_tx(&s, true).is_some() {
            emitted += 1;
        }
    }
    assert_eq!(emitted, 2, "two emissions across four ticks");
}

#[test]
fn a_catch_up_burst_emits_one_cyclic_however_many_runs_it_holds() {
    // The measured defect (offroad pair, 2026-10-22). The scheduler holds missed slots in its
    // runcount and the main loop drains them one per pass, so after an absence `control_task`
    // runs many times inside ONE tick. Against the old `control_ticks.is_multiple_of(2)` gate a
    // 294-run burst emitted 147 frames back to back, ~4 kB into a peer whose RX ring holds 38 ms
    // of this stream; the ring lapped once per arm cycle on both boards. On the elapsed-tick
    // clock the whole burst is one frame.
    let mut s = fresh();
    let _ = control_task(&mut s, None, 1); // consume the construction-time due
    let mut emitted = 0;
    // One served tick, then 293 catch-up runs inside it.
    let _ = control_task(&mut s, None, 1);
    if cyclic_tx(&s, true).is_some() {
        emitted += 1;
    }
    for _ in 0..293 {
        let _ = control_task(&mut s, None, 0);
        if cyclic_tx(&s, true).is_some() {
            emitted += 1;
        }
    }
    assert_eq!(
        emitted, 1,
        "a burst is one tick's worth of wire, not 147 frames"
    );
    assert_eq!(s.control_ticks, 295, "the burst still counts as WORK done");
}

#[test]
fn an_absence_costs_the_peer_one_age_count_and_a_burst_costs_none() {
    // The fault this whole change exists for (offroad pair, 2026-10-22): `cyclic_age` counted
    // `control_task` CALLS, so a catch-up burst ran the age at 300-650 counts/s against a peer
    // emitting 125 frames/s, crossed CYCLIC_TIMEOUT_TICKS in 40-80 ms of wall time against a
    // demonstrably live link, and asserted `comms_loss` -- a `fault_a` producer -- about once a
    // second on both boards at once. 72 rising edges on the master and 74 on the slave in one
    // 64 s hold, against 77 arm cycles.
    let mut s = fresh();
    s.inbox.accept(CTRL, cyclic(0));
    assert_eq!(s.inbox.cyclic_age(), 0);

    // A burst of 300 calls inside one tick ages the peer by nothing: no time passed.
    for _ in 0..300 {
        let _ = control_task(&mut s, None, 0);
    }
    assert_eq!(
        s.inbox.cyclic_age(),
        0,
        "no tick elapsed, so nothing is stale"
    );
    assert!(!s.inbox.comms_loss(), "a burst must not fault a live peer");

    // An absence of 180 ticks (the measured length) costs the peer ONE count, not 180: the board
    // could not have heard the peer while it was away, so its own absence is not evidence that
    // the PEER has gone, which is the only question this age answers.
    let _ = control_task(&mut s, None, 180);
    assert_eq!(s.inbox.cyclic_age(), 1, "one served tick, one count");
    assert!(!s.inbox.comms_loss());

    // A peer that has genuinely stopped still trips, on served ticks alone.
    for _ in 0..linkctl::CYCLIC_TIMEOUT_TICKS {
        let _ = control_task(&mut s, None, 1);
    }
    assert!(s.inbox.comms_loss(), "a silent peer still faults");
}

/// **The peak cyclic age is a latch, not a sample** (`specs/rover-one-imu.md` section 8). It is
/// what `RIGID_STALE_TICKS` is set from, and the bench reads it once per soak run instead of
/// polling the live age at half the rate the age advances.
#[test]
fn the_peak_cyclic_age_records_the_worst_case_and_a_fresh_frame_does_not_clear_it() {
    let mut s = fresh();
    assert_eq!(s.inbox.cyclic_age_peak(), 0, "no peer, no peak");

    s.inbox.accept(CTRL, cyclic(0));
    for _ in 0..3 {
        let _ = control_task(&mut s, None, 1);
    }
    assert_eq!(s.inbox.cyclic_age(), 3);
    assert_eq!(s.inbox.cyclic_age_peak(), 3, "the peak follows the age up");

    // A fresh frame zeroes the age. The peak is the WORST CASE, so it holds: the whole point is
    // that a reader arriving after the excursion still learns it happened.
    s.inbox.accept(CTRL, cyclic(0));
    let _ = control_task(&mut s, None, 1);
    assert_eq!(s.inbox.cyclic_age(), 1);
    assert_eq!(
        s.inbox.cyclic_age_peak(),
        3,
        "a fresh frame does not clear the latch"
    );
    assert_eq!(s.obs().cyclic_age_peak, 3, "and OBS publishes it");

    // A catch-up burst serves no tick, so it moves neither the age nor the latch.
    for _ in 0..50 {
        let _ = control_task(&mut s, None, 0);
    }
    assert_eq!(s.inbox.cyclic_age_peak(), 3, "no time passed, no excursion");

    // A deeper excursion raises it.
    for _ in 0..7 {
        let _ = control_task(&mut s, None, 1);
    }
    assert_eq!(s.inbox.cyclic_age(), 8);
    assert_eq!(s.inbox.cyclic_age_peak(), 8);
}

// --- The balance-era observation: the gating row + the pre-envelope torque view ---------------
//
// `specs/control.md`, "Observation". Two words published for the balance era's hands-on gates:
// the row the engagement machine actually gates on (whose SIGN is a board-level defect the bench
// has to rule out before a wheel comes free) and the reference the machine sees before its
// envelope, which is the only torque view that is live while the machine is disengaged.

#[test]
fn obs_publishes_the_row_the_machine_gates_on() {
    // Same row, one source: OBS must republish `block.gating_field` itself, not a recomputation
    // that could drift from the value the FSM compared against.
    let level = level_sample_at(8192);
    let mut s = balance_to_run(&level);
    for _ in 0..20 {
        control_task(&mut s, Some(&level), 1);
    }
    assert_eq!(s.obs().gating_field, s.block.gating_field);
    assert!(
        s.obs().gating_field > 500,
        "a level, right-way-up board reads large and positive: {}",
        s.obs().gating_field
    );
}

#[test]
fn obs_gating_row_goes_negative_on_an_inverted_deck() {
    // The sign check's failure mode, pinned from the host side: the published row is what
    // distinguishes a right-way-up board from an upside-down one, so a bench read of it IS the
    // accel-sign-map check. An inverted deck (or an inverted sign map, which is
    // indistinguishable from the row's point of view) reads NEGATIVE.
    let inverted = level_sample_at(-8192);
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    for _ in 0..40 {
        control_task(&mut s, Some(&inverted), 1);
    }
    assert!(
        s.obs().gating_field < 0,
        "inverted reads negative: {}",
        s.obs().gating_field
    );
    assert_eq!(s.obs().sub_state, 0, "and cannot engage");
}

#[test]
fn pre_env_torque_is_live_while_the_machine_is_disengaged() {
    // The point of the word. A balance board that never gets a power request stays in OFF, so the
    // engagement machine stays IDLE and the published `torque` is zero every tick. The pre-envelope
    // view is NOT: the balance law runs every tick regardless of sub-state, so a hand-tilted
    // disarmed board shows what the controller is commanding. This is the disarmed tilt
    // torque-shadow.
    let tilted = imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [8000, 0, 14000], // gravity pulled toward +X: a real pitch
        temp_centi_degc: 2500,
        still: false,
    };
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    for _ in 0..200 {
        control_task(&mut s, Some(&tilted), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Off, "never armed");
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "machine disengaged");
    assert_eq!(
        s.obs().torque_setpoint,
        0,
        "the output word is zero, as it must be"
    );
    assert_ne!(
        s.obs().pre_env_torque,
        0,
        "the pre-envelope view is live anyway: that is the torque shadow"
    );
    // And it tracks the lean's SIGN, which is what makes the shadow readable by hand.
    let leaned_forward = s.obs().pre_env_torque;
    let mut back = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    back.vbatt_raw = BENCH_CV;
    let tilted_back = imu::Sample {
        accel_raw: [-8000, 0, 14000],
        ..tilted
    };
    for _ in 0..200 {
        control_task(&mut back, Some(&tilted_back), 1);
    }
    assert_eq!(
        leaned_forward.signum(),
        -back.obs().pre_env_torque.signum(),
        "opposite leans, opposite shadows: {} vs {}",
        leaned_forward,
        back.obs().pre_env_torque
    );
}

#[test]
fn pre_env_torque_is_the_reference_the_machine_was_fed_not_its_output() {
    // Once ENGAGED the soft-start envelope clamps the machine's output up from zero over the
    // ramp, so for those ticks the pre-envelope view is strictly larger in magnitude than the
    // published torque. That difference IS the envelope, and seeing both is how a bench read
    // tells "the controller wants nothing" from "the controller wants something the envelope is
    // still withholding".
    let level = level_sample_at(8192);
    let mut s = balance_to_run(&level);
    let mut saw_envelope_binding = false;
    for _ in 0..30 {
        control_task(&mut s, Some(&level), 1);
        let o = s.obs();
        if o.pre_env_torque.unsigned_abs() > o.torque_setpoint.unsigned_abs() {
            saw_envelope_binding = true;
        }
    }
    // The reference is whatever the arm fed the shell; the FSM's own mirror agrees with it while
    // engaged (the mirror IS the smoothed reference in ARMING/RUN).
    assert_eq!(
        s.ctl.pre_env_ref as i32, s.ctl.fsm.out_mirror,
        "the recorded view is the machine's input"
    );
    let _ = saw_envelope_binding; // an envelope that never binds is legal on a zero reference
}

#[test]
fn throttle_pre_env_torque_is_the_conditioned_demand() {
    // Mode-agnostic: in Throttle the same word carries the EFeru conditioner's frame-out
    // reference, so one bench read means "what the active mode is asking for" on both arms.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    drive_ticks(&mut s, 200, 20000, 0);
    assert_eq!(s.obs().control_mode, 0, "throttle");
    assert!(
        s.obs().pre_env_torque > 500,
        "the conditioned demand is published: {}",
        s.obs().pre_env_torque
    );
}

// --- O1 attribution: the per-producer transition counters ------------------------------------
//
// `specs/silicon-queue.md` O1. The arm session's unexplained SHUTDOWN/re-arm cycles left the
// enact counters stepping and every level clear by the time anyone read them, because each
// producer folds into `fault_a`/`fault_b` before the mode machine sees it and a level that
// asserts and releases inside one 4 ms tick leaves no trace at all. These pin the instrument.

#[test]
fn a_blip_scores_two_transitions_and_a_hold_scores_one() {
    let mut ev = events::FaultEvents::default();
    ev.tick(EV_COMMS_LOSS);
    assert_eq!(ev.count(EV_COMMS_LOSS), 1, "the assert");
    assert_eq!(ev.levels(), EV_COMMS_LOSS, "and it is still held");
    ev.tick(EV_COMMS_LOSS);
    assert_eq!(ev.count(EV_COMMS_LOSS), 1, "a held level does not re-count");
    ev.tick(0);
    assert_eq!(ev.count(EV_COMMS_LOSS), 2, "the release");
    assert_eq!(ev.levels(), 0);
    // Even count + clear level = came and went. Odd + set = still held. That is the read rule.
    assert_eq!(ev.count(EV_COMMS_LOSS) % 2, 0);
}

#[test]
fn transition_counters_are_per_producer_and_do_not_bleed() {
    let mut ev = events::FaultEvents::default();
    for _ in 0..5 {
        ev.tick(EV_IMU_LOSS);
        ev.tick(0);
    }
    assert_eq!(ev.count(EV_IMU_LOSS), 10);
    for e in [
        EV_COMMS_LOSS,
        EV_STOP_ALL,
        EV_MOTOR_HALL,
        EV_LATCH_A,
        EV_LATCH_B,
        EV_MODE_FAULT,
        EV_POWER_REQUEST,
    ] {
        assert_eq!(ev.count(e), 0, "producer {e:#04x} untouched");
    }
    // Simultaneous changes each score once (the mask is not a single event).
    ev.tick(EV_LATCH_A | EV_LATCH_B);
    assert_eq!(ev.count(EV_LATCH_A), 1);
    assert_eq!(ev.count(EV_LATCH_B), 1);
}

#[test]
fn transition_counters_saturate_rather_than_wrap() {
    // A wrapped counter would read as a SMALL number, which is exactly the wrong answer for an
    // attribution instrument: it would exonerate the guilty producer.
    let mut ev = events::FaultEvents::default();
    for _ in 0..400 {
        ev.tick(EV_STOP_ALL);
        ev.tick(0);
    }
    assert_eq!(ev.count(EV_STOP_ALL), u8::MAX, "saturated, not wrapped");
}

#[test]
fn comms_loss_is_attributed_to_its_own_counter_through_the_pipeline() {
    // Driven the way silicon drives it: a peer appears, its frames stop, the supervision window
    // expires, then a fresh frame clears it. The counter names comms_loss and nothing else.
    let mut s = fresh();
    s.inbox.accept(CTRL, cyclic(0));
    run_ticks(&mut s, 3);
    let base = s.obs().event_counts;
    assert_eq!(
        u16::from(s.obs().event_levels) & EV_COMMS_LOSS,
        0,
        "fresh peer, no loss"
    );

    run_ticks(&mut s, 40); // past CYCLIC_TIMEOUT_TICKS
    assert_ne!(
        u16::from(s.obs().event_levels) & EV_COMMS_LOSS,
        0,
        "loss asserted"
    );
    s.inbox.accept(CTRL, cyclic(0));
    run_ticks(&mut s, 2);
    assert_eq!(
        u16::from(s.obs().event_levels) & EV_COMMS_LOSS,
        0,
        "and cleared"
    );

    let after = s.obs().event_counts;
    let idx = EV_COMMS_LOSS.trailing_zeros() as usize;
    assert_eq!(
        after[idx] - base[idx],
        2,
        "the assert and the release, attributed"
    );
    for (i, (a, b)) in after.iter().zip(base.iter()).enumerate() {
        if i != idx {
            assert_eq!(a, b, "producer bit {i} must not have moved");
        }
    }
}

#[test]
fn a_power_request_fall_is_counted_because_it_is_the_other_way_run_reaches_shutdown() {
    // The mode machine leaves RUN on `fault_a || fault_b || !power_request`. A momentarily
    // dropped request therefore produces the same SHUTDOWN/re-arm cycle a fault blip does, and
    // must be distinguishable from one. Counting both edges is what makes that work.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    assert_eq!(s.mode.mode(), Mode::Run);
    let idx = EV_POWER_REQUEST.trailing_zeros() as usize;
    assert_eq!(s.obs().event_counts[idx], 1, "the request's rise");

    // Release the button (two passes to debounce it off), then run.
    for _ in 0..2 {
        input_task(&mut s, &InputSample::default());
    }
    run_ticks(&mut s, 3);
    assert_eq!(
        s.obs().event_counts[idx],
        2,
        "the fall is counted: this is what a dropped request looks like"
    );
    assert_eq!(u16::from(s.obs().event_levels) & EV_POWER_REQUEST, 0);
    assert_eq!(s.mode.mode(), Mode::Off, "and it did shut down");
}

#[test]
fn the_motor_level_and_the_tilt_cutout_are_attributed_separately() {
    // Two `fault_a` members blipped in turn. Both fold into the SAME aggregate the mode machine
    // sees, which is precisely why the aggregate cannot attribute O1 and these counters can.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    let m = COUNTED_MOTOR.trailing_zeros() as usize;
    let a = EV_STOP_ALL.trailing_zeros() as usize;

    s.motor_fault = EV_MOTOR_HALL;
    run_ticks(&mut s, 1);
    s.motor_fault = 0;
    run_ticks(&mut s, 1);
    assert_eq!(s.obs().event_counts[m], 2, "motor level blipped");
    assert_eq!(s.obs().event_counts[a], 0, "the tilt cutout did not");

    s.tilt_latch = true;
    run_ticks(&mut s, 1);
    assert_eq!(
        s.obs().event_counts[a],
        1,
        "and now the tilt cutout is named"
    );
    assert_eq!(s.obs().event_counts[m], 2, "the motor count did not move");
}

/// **`CYCLIC_STATE.fault` is the level mask the pipeline folded THIS pass** (`specs/rover-one-imu.md`
/// section 4): what a peer reads is `events::Levels::mask()`, one `EV_*` bit per producer, as of
/// the pass that built the frame, so a producer asserted now is visible now and one that released
/// is gone. Three readings pinned: a quiet board emits 0; a board whose peer went silent carries
/// `EV_COMMS_LOSS` and nothing else; and a board holding its own power request carries
/// `EV_POWER_REQUEST`, which is in the mask and is NOT a fault, so `fault & FAULT_A_BITS` is what a
/// consumer asks for health.
#[test]
fn the_cyclic_fault_byte_is_this_passes_level_mask() {
    // Even tick counts throughout: the emission is due every second served tick.
    let mut s = fresh();
    run_ticks(&mut s, 2);
    assert_eq!(cyclic_tx(&s, true).expect("emits").fault, 0, "quiet");

    // The peer goes silent past the supervision window: comms_loss asserts and the byte says so.
    s.inbox.accept(CTRL, cyclic(0));
    run_ticks(&mut s, 26);
    let c = cyclic_tx(&s, true).expect("emits");
    assert_eq!(c.fault, EV_COMMS_LOSS);
    assert_eq!(
        c.fault,
        s.events.levels(),
        "the byte IS the instrument's mask"
    );
    assert_ne!(c.fault & FAULT_A_BITS, 0, "and comms_loss is a fault");

    // A fresh board with the power request held: the level is in the mask, and it is not a fault.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 2);
    let c = cyclic_tx(&s, true).expect("emits");
    assert_eq!(c.fault, EV_POWER_REQUEST);
    assert_eq!(c.fault & FAULT_A_BITS, 0, "a power request is not a fault");
}

/// `FAULT_A_BITS` is exactly the group `Levels::fault_a` folds, producer by producer: each bit
/// asserted alone is a Fault A if and only if the constant carries it, so the aggregate the mode
/// machine consumes and the mask a peer applies to `CYCLIC_STATE.fault` cannot disagree.
#[test]
fn fault_a_bits_is_exactly_the_fault_a_group() {
    assert_eq!(
        FAULT_A_BITS,
        EV_COMMS_LOSS | EV_STOP_ALL | EV_IMU_LOSS | EV_LATCH_A | EV_MOTOR_BITS
    );
    assert_eq!(
        EV_MOTOR_BITS, 0x0F00,
        "the four motor causes are bits 8 to 11 and bit 3 is retired"
    );
    for bit in 0..16 {
        let ev = 1u16 << bit;
        let levels = events::Levels {
            comms_loss: ev == EV_COMMS_LOSS,
            imu_loss: ev == EV_IMU_LOSS,
            motor: ev & EV_MOTOR_BITS,
            latch_a: ev == EV_LATCH_A,
            latch_b: ev == EV_LATCH_B,
            mode_fault: ev == EV_MODE_FAULT,
            power_request: ev == EV_POWER_REQUEST,
            peer_down: false,
            // `EV_STOP_ALL`'s one producer (`specs/control.md` (l)).
            tilt: ev == EV_STOP_ALL,
        };
        // Bit 3 and bits 12..15 have no producer: nothing can set them.
        let producer = bit != 3 && bit < 12;
        let want = if producer { ev } else { 0 };
        assert_eq!(levels.mask(), want, "bit {bit}: one producer, one bit");
        assert_eq!(levels.own_mask(), want, "and the wire word says the same");
        assert_eq!(
            levels.fault_a(),
            FAULT_A_BITS & want != 0,
            "bit {bit}: the aggregate and the mask agree"
        );
    }
    assert!(!events::Levels::default().fault_a());

    // `peer_down` is the ninth level on eight bits: a Fault A that rides `EV_COMMS_LOSS` in the
    // gating mask and is ABSENT from the wire byte, which is what keeps two boards from latching
    // each other's interlock (`events::Levels::own_mask`).
    let peer_down = events::Levels {
        peer_down: true,
        ..Default::default()
    };
    assert_eq!(peer_down.mask(), EV_COMMS_LOSS);
    assert!(peer_down.fault_a());
    assert_eq!(peer_down.own_mask(), 0);

    // Unlike `peer_down`, the tilt cutout IS on the wire byte (the loop above pins both masks),
    // because that byte is the only way a rigid follower learns its leader went over.
}

#[test]
fn obs_publishes_the_levels_beside_the_counts() {
    // The level half and the count half must come from the same instrument, so a bench read
    // can tell "held" from "blipped" in one sample.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    assert_eq!(s.obs().event_levels, counted(s.events.levels()));
    assert_eq!(s.obs().event_counts, s.events.counts());
    assert_ne!(
        u16::from(s.obs().event_levels) & EV_POWER_REQUEST,
        0,
        "the held request shows as a LEVEL, and its count is odd"
    );
    assert_eq!(
        s.obs().event_counts[EV_POWER_REQUEST.trailing_zeros() as usize] % 2,
        1
    );
}

#[test]
fn a_transient_that_never_survives_a_tick_boundary_is_still_the_only_thing_that_shows() {
    // The O1 shape itself: something asserts and releases between two bench reads. The enact
    // counters record a cycle happened; the LEVELS are all clear by the time anyone looks; only
    // the transition counts say which producer did it.
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    let inits_before = s.obs().enact_inits;

    s.motor_fault = EV_MOTOR_HALL; // one tick of fault: RUN -> SHUTDOWN
    run_ticks(&mut s, 1);
    s.motor_fault = 0;
    run_ticks(&mut s, 6); // OFF, then re-arm on the still-held request

    assert!(
        s.obs().enact_inits > inits_before,
        "the re-arm happened (the O1 signature)"
    );
    assert_eq!(s.obs().event_levels & COUNTED_MOTOR, 0, "level long gone");
    assert_eq!(
        s.obs().event_counts[COUNTED_MOTOR.trailing_zeros() as usize],
        2,
        "and the counter is the only witness"
    );
}

// --- The BLE port's decimated cyclic emission -------------------------------------------------

#[test]
fn ble_cyclic_tx_emits_at_five_hz_and_leaves_the_inter_board_rate_alone() {
    // The BLE port is 9600 baud and shared with the command stream, so it gets its own, much
    // coarser decimation than the 460800-baud inter-board link: one emission per
    // BLE_CYCLIC_DIVISOR control runs (250 Hz / 50 = 5 Hz). Over 200 runs that is exactly 4
    // BLE emissions against 100 inter-board ones - the inter-board rate is UNCHANGED by this
    // slice, which is the property that matters most here.
    let mut s = fresh();
    let (mut ble, mut peer) = (0, 0);
    for _ in 0..200 {
        let _ = control_task(&mut s, None, 1);
        if ble_cyclic_tx(&s, true).is_some() {
            ble += 1;
        }
        if cyclic_tx(&s, true).is_some() {
            peer += 1;
        }
    }
    assert_eq!(peer, 100, "inter-board stays at every second run (125 Hz)");
    assert_eq!(ble, 4, "BLE emits once per 50 runs (5 Hz)");
    assert_eq!(BLE_CYCLIC_DIVISOR, 50);
}

#[test]
fn ble_cyclic_tx_is_gated_on_an_assigned_address() {
    // Same gate as the peer emission: an unassigned board has no `src` worth publishing, so it
    // puts nothing on the BLE port however many control runs elapse.
    let mut s = fresh();
    for _ in 0..200 {
        let _ = control_task(&mut s, None, 1);
        assert!(
            ble_cyclic_tx(&s, false).is_none(),
            "an unaddressed board emits nothing on BLE"
        );
    }
}

#[test]
fn ble_cyclic_tx_carries_the_same_payload_as_the_peer_emission() {
    // One payload type, two rates (the app decodes the same CYCLIC_STATE the peer board does).
    // Drive to a run that BOTH gates pass and compare the built payloads field for field.
    let mut s = fresh();
    loop {
        let _ = control_task(&mut s, None, 1);
        if s.control_ticks.is_multiple_of(BLE_CYCLIC_DIVISOR) {
            break;
        }
    }
    let b = ble_cyclic_tx(&s, true).expect("BLE emission due on a divisor run");
    let p = cyclic_tx(&s, true).expect("peer emission due on an even run");
    assert_eq!(b, p, "the two ports publish identical board state");
}

// --- The BLE port's forward of the peer's cyclic -----------------------------------------------
// (`specs/link-control.md`, "The peer's state on the BLE port: the forward")

/// The peer board these vectors hear from: a persistent board address, as the inter-board
/// cyclic carries it (the slave emits only once addressed, so `src` is never 0x00 here).
const PEER: u8 = 0x02;

#[test]
fn ble_peer_tx_forwards_at_two_and_a_half_hz_behind_the_own_rate() {
    // One forward per BLE_PEER_CYCLIC_TICKS served ticks (250 Hz / 100 = 2.5 Hz): over 400 ticks
    // that is 4 forwards against 8 own BLE emissions and 200 inter-board ones, neither of which
    // moves. The peer is kept fresh at its own 125 Hz so freshness is not what gates here.
    let mut s = fresh();
    let (mut fwd, mut own, mut peer) = (0, 0, 0);
    for t in 0..400u32 {
        if t.is_multiple_of(2) {
            s.inbox.accept(PEER, cyclic(0));
        }
        let _ = control_task(&mut s, None, 1);
        if ble_peer_tx(&s, true).is_some() {
            fwd += 1;
        }
        if ble_cyclic_tx(&s, true).is_some() {
            own += 1;
        }
        if cyclic_tx(&s, true).is_some() {
            peer += 1;
        }
    }
    assert_eq!(fwd, 4, "the forward runs at 2.5 Hz");
    assert_eq!(own, 8, "the own BLE stream stays at 5 Hz");
    assert_eq!(peer, 200, "the inter-board rate is untouched");
    assert_eq!(BLE_PEER_CYCLIC_TICKS, 2 * BLE_CYCLIC_DIVISOR);
}

#[test]
fn ble_peer_tx_is_gated_on_an_assigned_address() {
    // The same gate as the board's own emissions: unaddressed, the board is silent on the BLE
    // port whatever it has heard from its peer.
    let mut s = fresh();
    for t in 0..400u32 {
        if t.is_multiple_of(2) {
            s.inbox.accept(PEER, cyclic(0));
        }
        let _ = control_task(&mut s, None, 1);
        assert!(ble_peer_tx(&s, false).is_none(), "unaddressed: no forward");
    }
}

#[test]
fn ble_peer_tx_forwards_nothing_before_a_peer_is_seen() {
    let mut s = fresh();
    for _ in 0..400 {
        let _ = control_task(&mut s, None, 1);
        assert!(
            ble_peer_tx(&s, true).is_none(),
            "no peer, nothing to forward"
        );
    }
}

#[test]
fn ble_peer_tx_carries_the_inbox_state_and_its_source_unchanged() {
    // The forward is the inbox's decoded copy under the address it arrived with, and the codec is
    // a bijection on the 32 B layout: re-encoding it gives the bytes the peer put on the wire.
    // The rigid words ride through it like the rest: a leader's commanded torque and its leader
    // flag are the peer's to state, and the forward restates them rather than this board's.
    let frame = CyclicState {
        pitch: -1234,
        roll: 567,
        wheel_speed: -89,
        battery: 3712,
        mode: 4,
        fault: 0x21,
        flags: CyclicState::FLAG_RIDER | CyclicState::FLAG_LEADER | CyclicState::FLAG_LOCKDOWN,
        peer_demand: -4096,
        wheel_pos: -321,
        obs: PEER_OBS,
        wheel_rate: 0,
    };
    let mut wire = [0u8; CyclicState::ENCODED_LEN];
    frame.encode(&mut wire);
    let mut s = fresh();
    for _ in 0..BLE_PEER_CYCLIC_TICKS - 1 {
        let _ = control_task(&mut s, None, 1);
        assert!(ble_peer_tx(&s, true).is_none(), "not due yet");
    }
    s.inbox.accept(PEER, Payload::CyclicState(frame));
    let _ = control_task(&mut s, None, 1);
    let fwd = ble_peer_tx(&s, true).expect("due on the 100th tick");
    assert_eq!(
        fwd,
        PeerCyclic {
            src: PEER,
            state: frame
        }
    );
    let mut again = [0u8; CyclicState::ENCODED_LEN];
    fwd.state.encode(&mut again);
    assert_eq!(
        again, wire,
        "the bytes that reach the phone are the peer's bytes"
    );
}

#[test]
fn ble_peer_tx_stops_when_the_peer_goes_stale_and_resumes_on_a_fresh_frame() {
    // Only fresh state is forwarded: the gate is the inbox's own window, the complement of
    // comms_loss. A peer heard 6 ticks ago is forwarded; one heard 105 ticks ago is not, and the
    // board's comms_loss says why; the next accepted frame resumes the stream on the next due
    // tick.
    let mut s = fresh();
    for t in 1..=400u32 {
        if t <= 95 || t == 299 {
            s.inbox.accept(PEER, cyclic(0));
        }
        let _ = control_task(&mut s, None, 1);
        let fwd = ble_peer_tx(&s, true);
        match t {
            100 => {
                assert_eq!(
                    s.inbox.cyclic_age(),
                    6,
                    "heard at 95, aged by the runs 95..=100"
                );
                assert!(fwd.is_some(), "fresh at 6 ticks: forwarded");
            }
            200 => {
                assert!(s.inbox.comms_loss(), "the window has closed");
                assert!(fwd.is_none(), "stale: the stream stops");
            }
            300 => {
                assert!(!s.inbox.comms_loss());
                assert_eq!(fwd.map(|p| p.src), Some(PEER), "a fresh frame resumes it");
            }
            _ => assert!(fwd.is_none(), "tick {t} is not a due tick"),
        }
    }
}

#[test]
fn a_catch_up_burst_forwards_once() {
    // The cadence counts ticks ELAPSED, not calls: runs with dt_ticks = 0 (the scheduler draining
    // missed slots inside one tick) advance nothing, and a single run that follows an absence
    // advances by the ticks it is handed, so a burst of any length yields one forward.
    let mut s = fresh();
    s.inbox.accept(PEER, cyclic(0));
    let _ = control_task(&mut s, None, BLE_PEER_CYCLIC_TICKS);
    assert!(
        ble_peer_tx(&s, true).is_some(),
        "one run, 100 ticks elapsed: due"
    );
    let mut fwds = 0;
    for _ in 0..300 {
        s.inbox.accept(PEER, cyclic(0));
        let _ = control_task(&mut s, None, 0);
        if ble_peer_tx(&s, true).is_some() {
            fwds += 1;
        }
    }
    assert_eq!(fwds, 0, "300 catch-up runs inside one tick forward nothing");
    assert_eq!(s.ble_peer_since_tx, 0, "and the clock has not moved");
}

// --- The balance law's input units (`specs/control.md` (j)) ------------------------------------

/// A burst buffer with `gy` raw counts on the gyro Y word and level gravity, decoded by the real
/// IMU front-end (so the rad/s value is the one the firmware computes, `GYRO_SCALE` owned there).
fn sample_with_gy_counts(gy: i16) -> imu::Sample {
    let mut buf = [0u8; imu::BURST_LEN];
    buf[4..6].copy_from_slice(&(-16384i16).to_be_bytes()); // az: the default map flips Z
    buf[10..12].copy_from_slice(&gy.to_be_bytes());
    let cfg = imu::Config {
        sign: [-1, 1, -1, -1, 1, -1],
        gyro_bias: [0; 3],
        roles: imu::DEFAULT_ROLES,
    };
    imu::Imu::new(imu::MPU6050, cfg).decode(&buf)
}

#[test]
fn the_pitch_rate_word_is_rad_per_s_times_10000() {
    // (j) point 4: stock's @0x9c word is f2iz(gy * 0.000266316114 * 10000), rad/s x 10000.
    // 1 rad/s gives 10000 (before (j) the word was the raw count, a 2.66x smaller unit).
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    let one_rad = imu::Sample {
        gyro: [Fix::ZERO, Fix::from_num(1), Fix::ZERO],
        ..level_sample()
    };
    control_task(&mut s, Some(&one_rad), 1);
    assert_eq!(s.block.pitch_rate, 10_000);
    let minus = imu::Sample {
        gyro: [Fix::ZERO, Fix::from_num(-1), Fix::ZERO],
        ..level_sample()
    };
    control_task(&mut s, Some(&minus), 1);
    assert_eq!(s.block.pitch_rate, -10_000);

    // One raw count at +-500 deg/s full scale: 2.663 truncated toward zero is 2 (it was 1, the
    // raw count itself), and -1 count is -2, not -3 (toward zero, not floor).
    for (count, want) in [(1i16, 2), (-1, -2), (3, 7), (-3, -7)] {
        let smp = sample_with_gy_counts(count);
        assert_eq!(
            smp.gyro_raw[PITCH_RATE_AXIS], count,
            "the count reached the pitch axis"
        );
        control_task(&mut s, Some(&smp), 1);
        assert_eq!(
            s.block.pitch_rate, want,
            "{count} raw count(s) give rate word {want}"
        );
    }
}

/// The gain ramp's worst-case `|bv|` (`control::config::ramp::BV_BOUND`, an input of the cap's
/// derivation) is the rate word the real decode gives at the gyro's full scale (87,263), rounded
/// up to the nominal +-500 deg/s: the IMU clamps the bias-corrected count to +-32767, so no pass
/// can present a larger rate word to the ramp.
#[test]
fn the_ramp_rate_bound_is_the_decoded_gyro_full_scale() {
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    for count in [i16::MAX, i16::MIN] {
        control_task(&mut s, Some(&sample_with_gy_counts(count)), 1);
        let rate = s.block.pitch_rate.abs();
        assert!(
            (control::config::ramp::BV_BOUND - 5..=control::config::ramp::BV_BOUND).contains(&rate),
            "full-scale count {count} decodes to rate word {rate}"
        );
    }
}

// --- The battery word (`specs/sensing-and-safety.md`, "The battery word") ------------------------

/// The conversion at the four bench calibration points, through the field's defaults (the fit
/// itself, rounded to the field's units): each within one count of the measured rail.
#[test]
fn vbatt_conversion_at_the_four_calibration_points() {
    use battery::VbattCal;
    let cal = VbattCal::new(
        store::BOARD_BATTERY_CAL.at(0).default(),
        store::BOARD_BATTERY_CAL.at(1).default(),
    );
    assert_eq!((cal.slope(), cal.offset()), (25200, -5));
    // (counts, measured rail centivolts, the conversion's exact output)
    for (raw, rail, exact) in [
        (995u16, 2502i16, 2502i16),
        (794, 1996, 1995),
        (635, 1595, 1595),
        (476, 1194, 1194),
    ] {
        let cv = cal.convert(raw);
        assert_eq!(cv, exact, "{raw} counts");
        assert!((cv - rail).abs() <= 1, "{raw} counts -> {cv}, rail {rail}");
    }
}

/// The seam clamps: slope 10000..32000, offset -500..500; in-range values pass unchanged.
#[test]
fn vbatt_cal_seam_clamps() {
    use battery::VbattCal;
    let lo = VbattCal::new(i16::MIN, i16::MIN);
    assert_eq!((lo.slope(), lo.offset()), (10_000, -500));
    let lo1 = VbattCal::new(9_999, -501);
    assert_eq!((lo1.slope(), lo1.offset()), (10_000, -500));
    let hi = VbattCal::new(i16::MAX, i16::MAX);
    assert_eq!((hi.slope(), hi.offset()), (32_000, 500));
    let hi1 = VbattCal::new(32_001, 501);
    assert_eq!((hi1.slope(), hi1.offset()), (32_000, 500));
    let mid = VbattCal::new(25_200, -5);
    assert_eq!((mid.slope(), mid.offset()), (25_200, -5));
    // A clamped calibration converts with the clamped values.
    assert_eq!(hi.convert(1000), 3200 + 500);
}

/// The lower result clamp: a conversion that would come out at or below 0 reads 1, so a valid
/// reading never reads as UNKNOWN.
#[test]
fn vbatt_conversion_lower_clamp_is_one() {
    use battery::VbattCal;
    let cal = VbattCal::new(10_000, -500);
    assert_eq!(cal.convert(1), 1, "1 - 500 clamps to 1");
    assert_eq!(cal.convert(500), 1, "500 - 500 = 0 clamps to 1");
    assert_eq!(cal.convert(501), 1);
    assert_eq!(cal.convert(502), 2);
    // The default calibration's offset alone would make a 1-count reading -3.
    assert_eq!(VbattCal::new(25_200, -5).convert(1), 1);
}

/// The local sense's 250 Hz filter: a zero count is "no conversion yet" (UNKNOWN), the first real
/// count primes the filter, then `filt += (cv - filt) >> 4`.
#[test]
fn vbatt_local_sense_primes_then_filters() {
    use battery::{LocalSense, VbattCal};
    let mut s = LocalSense::new(VbattCal::new(10_000, 0)); // centivolts == counts
    assert_eq!(s.step(0), 0, "no conversion yet: UNKNOWN");
    assert_eq!(s.step(0), 0);
    assert_eq!(s.step(2500), 2500, "primed to the first sample");
    assert_eq!(s.step(2660), 2510, "2500 + (160 >> 4)");
    assert_eq!(
        s.step(2500),
        2509,
        "2510 + (-10 >> 4) = 2510 - 1 (arithmetic shift)"
    );
    assert_eq!(s.step(0), 2509, "a zero count after priming holds the word");
    assert_eq!(s.word(), 2509);
    // A step held long enough converges to within the shift's dead band.
    for _ in 0..200 {
        s.step(2000);
    }
    assert!((2000..2016).contains(&s.word()), "{}", s.word());
}

/// A peer cyclic frame carrying `battery`, otherwise quiet.
fn peer_with_battery(battery: u16) -> Payload {
    Payload::CyclicState(CyclicState {
        pitch: 0,
        roll: 0,
        wheel_speed: 0,
        battery,
        mode: 0,
        fault: 0,
        peer_demand: 0,
        wheel_pos: 0,
        flags: 0,
        obs: PEER_OBS,
        wheel_rate: 0,
    })
}

/// The source rule, as a pure function: local sense first, whatever it reads; else a fresh
/// peer's nonzero word; else 0.
#[test]
fn battery_source_rule_three_cases() {
    use battery::battery_source;
    let peer = |b: u16| match peer_with_battery(b) {
        Payload::CyclicState(c) => Some(c),
        _ => unreachable!(),
    };
    // Rule 1: a sensing board takes its own word and ignores the peer's, even a nonzero one,
    // and even while its own word is still UNKNOWN (no conversion yet).
    assert_eq!(battery_source(Some(2502), peer(6200), true), 2502);
    assert_eq!(battery_source(Some(0), peer(2502), true), 0);
    // Rule 2: a non-sensing board relays a fresh peer's nonzero word.
    assert_eq!(battery_source(None, peer(2502), true), 2502);
    // Rule 3: otherwise UNKNOWN (no peer; a stale peer; a peer that is itself UNKNOWN).
    assert_eq!(battery_source(None, None, true), 0);
    assert_eq!(battery_source(None, peer(2502), false), 0);
    assert_eq!(battery_source(None, peer(0), true), 0);
    // A peer word past i16 cannot become a negative divisor.
    assert_eq!(battery_source(None, peer(0xFFFF), true), i16::MAX);
}

/// A master (sensing, default calibration, 995 counts on the 25.00 V rail) ignores its slave's
/// nonzero word (a slave's PA4 reads a fictitious ~62 V), and its cyclic carries its own word.
#[test]
fn a_sensing_master_ignores_its_slaves_word() {
    let cal = battery::VbattCal::new(
        store::BOARD_BATTERY_CAL.at(0).default(),
        store::BOARD_BATTERY_CAL.at(1).default(),
    );
    let mut m = OrchestratorState::new(
        0,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        Some(cal),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    m.vbatt_raw = 995;
    for k in 0..40 {
        if k % 10 == 0 {
            m.inbox.accept(CTRL, peer_with_battery(6200));
        }
        control_task(&mut m, None, 1);
    }
    assert!(!m.inbox.comms_loss(), "the slave is fresh");
    assert_eq!(
        m.block.battery, 2502,
        "its own filtered word, not the slave's"
    );
    m.cyclic_due = true; // an emitting tick (the cadence's own clock, not a run parity)
    let tx = cyclic_tx(&m, true).unwrap();
    assert_eq!(tx.battery, 2502, "the cyclic carries the effective word");
    assert_eq!(m.obs().battery, 2502, "and so does OBS (CTRL_OBS word 32)");
}

/// A sensing board whose motor was never brought up gets no conversions: its count stays 0 and
/// its word UNKNOWN, and it still does not take its peer's word.
#[test]
fn a_sensing_board_with_no_conversions_stays_unknown() {
    let mut m = OrchestratorState::new(
        0,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    for k in 0..40 {
        if k % 10 == 0 {
            m.inbox.accept(CTRL, peer_with_battery(2502));
        }
        control_task(&mut m, None, 1);
    }
    assert_eq!(m.block.battery, 0);
    // The first conversion primes the word at once.
    m.vbatt_raw = 2490;
    control_task(&mut m, None, 1);
    assert_eq!(m.block.battery, 2490);
}

/// A slave (not sensing) relays its master's word while the master's cyclic is fresh, and drops
/// to UNKNOWN once the mirror goes stale; with no peer at all it is UNKNOWN from boot.
#[test]
fn a_non_sensing_slave_relays_its_masters_word() {
    let mut s = fresh();
    run_ticks(&mut s, 5);
    assert_eq!(s.block.battery, 0, "no sense, no peer: UNKNOWN");

    for k in 0..40 {
        if k % 10 == 0 {
            s.inbox.accept(CTRL, peer_with_battery(2502));
        }
        control_task(&mut s, None, 1);
    }
    assert_eq!(s.block.battery, 2502, "the master's word, relayed");
    s.cyclic_due = true; // an emitting tick (the cadence's own clock, not a run parity)
    assert_eq!(cyclic_tx(&s, true).unwrap().battery, 2502);

    run_ticks(&mut s, 40); // past CYCLIC_TIMEOUT_TICKS
    assert!(s.inbox.comms_loss());
    assert_eq!(s.block.battery, 0, "a stale mirror is not a source");
}

/// Balance mode on an UNKNOWN battery: every other engage gate open (RUN, rider, level, gating
/// row above 500), the machine never engages, and the disarmed tilt shadow reads 0.
#[test]
fn an_unknown_battery_blocks_balance_engage_and_zeroes_the_shadow() {
    let level = level_sample_at(8192);
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    for _ in 0..200 {
        control_task(&mut s, Some(&level), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run);
    assert!(s.rider_present);
    assert!(s.block.gating_field > 500, "{}", s.block.gating_field);
    assert_eq!(s.block.battery, 0);
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "UNKNOWN battery: no engage");
    assert_eq!(s.obs().torque_setpoint, 0);

    // The same board disarmed and tilted: the law's word is 0, so the shadow is 0 rather
    // than a plausible wrong number (the sensing twin of this vector reads nonzero).
    let tilted = imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [8000, 0, 14000],
        temp_centi_degc: 2500,
        still: false,
    };
    let mut d = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        None,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    for _ in 0..200 {
        control_task(&mut d, Some(&tilted), 1);
    }
    assert_eq!(
        d.obs().pre_env_torque,
        0,
        "unknown battery: no torque shadow"
    );
}

/// Throttle mode never divides by the battery, so an UNKNOWN word does not block its engage.
#[test]
fn an_unknown_battery_does_not_block_throttle_engage() {
    let mut s = fresh();
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    drive_ticks(&mut s, 200, 20000, 0);
    assert_eq!(s.obs().control_mode, 0, "throttle");
    assert_eq!(s.block.battery, 0, "UNKNOWN");
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged anyway");
    assert_ne!(s.obs().torque_setpoint, 0);
}

/// The law's carries are among the records a controller change replaces, and they are replaced on a
/// CHANGE only: every arm runs the install, so an arm that re-reads the mode it is already in must
/// leave them exactly as the last pass left them.
#[test]
fn an_arm_time_mode_change_resets_the_law_and_an_unchanged_mode_does_not() {
    let install = |s: &mut OrchestratorState, mode: u8| {
        re_apply_control_values(
            s,
            mode,
            0,
            0,
            store::CONTROL_RIDER_REQUIRED.default(),
            store::BOARD_BATTERY_FLOOR.default(),
            store::CONTROL_POS_CLAMP.default(),
        )
    };
    let mut s = balance_board(sf_start());
    s.block.wheel_rate_q12 = 4096;
    run_ticks(&mut s, 30);
    let carried = s.ctl.sf;
    assert_ne!(
        carried.v_filt, 0,
        "the filter is a measurement and runs disarmed"
    );
    install(&mut s, 1);
    assert_eq!(s.ctl.sf, carried, "the mode did not move");
    install(&mut s, 0);
    assert_eq!(s.ctl.sf, control::SfState::default());
}

// --- The rider requirement (`specs/control.md` (i)) ------------------------------------------

/// A balance board (CONTROL_MODE = 1, IMU configured) whose `CONTROL_RIDER_REQUIRED` byte is
/// `rider_required`, reading a known battery word where `vbatt` senses, at the default
/// `BOARD_BATTERY_FLOOR`.
fn rider_board(rider_required: u8, vbatt: Option<battery::VbattCal>) -> OrchestratorState {
    policy_board(rider_required, store::BOARD_BATTERY_FLOOR.default(), vbatt)
}

/// [`rider_board`] with the `BOARD_BATTERY_FLOOR` word given.
fn policy_board(
    rider_required: u8,
    battery_floor: i16,
    vbatt: Option<battery::VbattCal>,
) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        vbatt,
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        rider_required,
        battery_floor,
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    assert_eq!(
        s.obs().control_mode,
        1,
        "balance holds with a configured IMU"
    );
    s
}

/// The button held, no pads.
const BUTTON_ONLY: InputSample = InputSample {
    button_asserted: true,
    pad_a_high: false,
    pad_b_high: false,
};

/// The engagement machine's whole record as text, for lockstep comparisons: `FsmState` is the
/// binary's cells and carries no `PartialEq` (nothing in the firmware compares two of them); its
/// `Debug` form names every cell.
fn fsm_of(s: &OrchestratorState) -> std::string::String {
    std::format!("{:?}", s.ctl.fsm)
}

#[test]
fn the_rider_requirement_reaches_the_dispatch_through_the_constructor() {
    // The boot seam: the byte the firmware reads from CONTROL_RIDER_REQUIRED is decoded once by
    // the orchestrator constructor's control dispatch. Default 1 = required; 0 waives.
    assert_eq!(store::CONTROL_RIDER_REQUIRED.default(), 1);
    assert!(
        rider_board(store::CONTROL_RIDER_REQUIRED.default(), sensing())
            .ctl
            .dispatch
            .rider_required()
    );
    assert!(!rider_board(0, sensing()).ctl.dispatch.rider_required());
    // A mode that moves at an arm replaces the mode's records, never the rider decision: the two
    // bytes ride in the same install call and each is decoded by its own rule.
    let mut s = rider_board(0, sensing());
    for mode in [0u8, 1] {
        re_apply_control_values(
            &mut s,
            mode,
            0,
            0,
            0,
            store::BOARD_BATTERY_FLOOR.default(),
            store::CONTROL_POS_CLAMP.default(),
        );
    }
    assert!(!s.ctl.dispatch.rider_required());
}

#[test]
fn a_waived_rider_requirement_engages_and_holds_run_with_no_rider_anywhere() {
    // (i) host vector: with the field at 0 the machine walks IDLE -> ARMING -> RUN with no pads,
    // no mirror and no peer; RUN holds for 1,000 ticks with the step-off wind-down counter at 0;
    // the SAME board at the default never
    // leaves IDLE, so the substitution is what engaged it.
    let level = level_sample_at(8192);
    let mut waived = rider_board(0, sensing());
    let mut required = rider_board(store::CONTROL_RIDER_REQUIRED.default(), sensing());
    for s in [&mut waived, &mut required] {
        hold_power(s);
        for _ in 0..3 {
            control_task(s, Some(&level), 1);
        }
        assert_eq!(s.mode.mode(), Mode::Run);
        assert!(!s.rider_present, "no pads");
        assert!(!s.inbox.remote_rider_present(), "no mirror");
        assert!(s.inbox.peer().is_none(), "no peer");
        assert!(!dispatch::rider_level(s), "no rider anywhere");
    }

    // From construction (IDLE); the arming walk itself may already have engaged it.
    let mut walk = std::vec![0u8];
    if waived.ctl.fsm.sub_state as u8 != 0 {
        walk.push(waived.ctl.fsm.sub_state as u8);
    }
    for _ in 0..200 {
        let t = control_task(&mut waived, Some(&level), 1);
        if walk.last() != Some(&t.sub_state) {
            walk.push(t.sub_state);
        }
        let r = control_task(&mut required, Some(&level), 1);
        assert_eq!(r.sub_state, 0, "the default board needs a rider");
    }
    assert_eq!(walk, [0, 1, 3], "IDLE -> ARMING -> RUN");

    for k in 0..1000 {
        let t = control_task(&mut waived, Some(&level), 1);
        assert_eq!(t.sub_state, 3, "RUN holds at tick {k}");
        assert_eq!(
            waived.ctl.fsm.winddown_counter, 0,
            "no step-off at tick {k}"
        );
    }
}

#[test]
fn a_waived_board_runs_exactly_as_a_required_board_with_a_rider_and_ignores_every_rider_source() {
    // One substitution, three consumers: a waived board with no rider must be indistinguishable,
    // cell for cell, from a default board with a rider on the pads (same engage, same held-clear
    // wind-down, same profile). And on a waived board none of the rider sources (the pads, the
    // INPUTS mirror's rider bit, the peer cyclic's flag) can hold or drop anything: four waived
    // boards fed different riders stay identical, including when those riders leave.
    let level = level_sample_at(8192);
    let mut reference = rider_board(1, sensing()); // required, pads down throughout
    let mut bare = rider_board(0, sensing()); // waived, no rider ever
    let mut pads = rider_board(0, sensing()); // waived, pads down then released
    let mut mirrored = rider_board(0, sensing()); // waived, the INPUTS mirror's rider bit
    let mut peered = rider_board(0, sensing()); // waived, the peer cyclic's rider flag
    let pads_on = pads_on_button_held();
    let mut reported = [false; 2];
    for k in 0..900usize {
        let rider_on = k < 450;
        input_task(&mut reference, &pads_on);
        input_task(&mut bare, &BUTTON_ONLY);
        input_task(&mut pads, if rider_on { &pads_on } else { &BUTTON_ONLY });
        input_task(&mut mirrored, &BUTTON_ONLY);
        input_task(&mut peered, &BUTTON_ONLY);
        if k % 40 == 0 {
            let bit = if rider_on {
                linkctl::Inputs::RIDER_PRESENT
            } else {
                0
            };
            mirrored.inbox.accept(CTRL, mirror(0, bit));
        }
        // Every board hears the same peer (so comms supervision is identical); only one hears it
        // report a rider, and only until the riders leave.
        for s in [&mut reference, &mut bare, &mut pads, &mut mirrored] {
            s.inbox.accept(OTHER, cyclic(0));
        }
        let flag = if rider_on { CyclicState::FLAG_RIDER } else { 0 };
        peered.inbox.accept(OTHER, cyclic(flag));

        let want = control_task(&mut reference, Some(&level), 1);
        let want_fsm = fsm_of(&reference);
        for (name, s) in [
            ("bare", &mut bare),
            ("pads", &mut pads),
            ("mirror", &mut mirrored),
            ("peer", &mut peered),
        ] {
            let t = control_task(s, Some(&level), 1);
            assert_eq!(t, want, "{name} output at tick {k}");
            assert_eq!(fsm_of(s), want_fsm, "{name} engagement machine at tick {k}");
        }
        if k == 449 {
            assert!(pads.rider_present && mirrored.inbox.remote_rider_present());
            assert!(peered.inbox.peer().unwrap().rider_present());
        }
        // The cyclic still reports the LOCAL pads on a waived board (telemetry of a physical
        // fact, unchanged by (i)).
        if let Some(c) = cyclic_tx(&pads, true) {
            assert_eq!(c.rider_present(), pads.rider_present, "tick {k}");
            reported[usize::from(c.rider_present())] = true;
        }
    }
    assert_eq!(reference.ctl.fsm.sub_state as u8, 3, "engaged and held");
    assert!(!pads.rider_present, "the pads did leave");
    assert_eq!(
        reported,
        [true, true],
        "the cyclic reported both pad states"
    );
}

/// Drive a waived board and a pads-down default board in lockstep through `ticks` passes of
/// `sample`, asserting they never engage and never differ: the waiver removes the rider term
/// and nothing else, so the remaining conjunction terms block both alike.
fn assert_blocked_alike(
    waived: &mut OrchestratorState,
    reference: &mut OrchestratorState,
    sample: &imu::Sample,
    ticks: usize,
    what: &str,
) {
    for k in 0..ticks {
        let t = control_task(waived, Some(sample), 1);
        let r = control_task(reference, Some(sample), 1);
        assert_eq!(t.sub_state, 0, "{what}: waived engaged at tick {k}");
        assert_eq!(t, r, "{what}: diverged at tick {k}");
        assert_eq!(fsm_of(waived), fsm_of(reference), "{what}: tick {k}");
    }
}

/// Press the button on `waived` (no pads) and the button plus pads on `reference`.
fn press_pair(waived: &mut OrchestratorState, reference: &mut OrchestratorState) {
    for _ in 0..2 {
        input_task(waived, &BUTTON_ONLY);
        input_task(reference, &pads_on_button_held());
    }
}

#[test]
fn a_waived_rider_requirement_leaves_every_other_engage_term_in_force() {
    let level = level_sample_at(8192);

    // Battery UNKNOWN (no sense, no peer): blocked, both alike.
    let mut w = rider_board(0, None);
    let mut r = rider_board(1, None);
    press_pair(&mut w, &mut r);
    assert_blocked_alike(&mut w, &mut r, &level, 300, "battery unknown");
    assert_eq!(
        w.mode.mode(),
        Mode::Run,
        "armed, so only the conjunction blocks"
    );
    assert_eq!(w.block.battery, 0);

    // The gating row under 500 counts (0.049 g on the up axis): blocked.
    let shallow = level_sample_at(400);
    let mut w = rider_board(0, sensing());
    let mut r = rider_board(1, sensing());
    press_pair(&mut w, &mut r);
    assert_blocked_alike(&mut w, &mut r, &shallow, 300, "gating row");
    assert!(w.block.gating_field <= 500);

    // Outside the upright window: the attitude converged on a deck pitched ~60 degrees (the up
    // axis still well past 500 counts) BEFORE the arm, so the window is the only closed term.
    let pitched = imu::Sample {
        accel_raw: [14189, 0, 8192],
        ..level_sample()
    };
    let mut w = rider_board(0, sensing());
    let mut r = rider_board(1, sensing());
    for _ in 0..2000 {
        control_task(&mut w, Some(&pitched), 1);
        control_task(&mut r, Some(&pitched), 1);
    }
    assert!(
        fix_deg(w.attitude.pitch_deg).abs() > 25,
        "converged outside the window: {}",
        fix_deg(w.attitude.pitch_deg)
    );
    press_pair(&mut w, &mut r);
    assert_blocked_alike(&mut w, &mut r, &pitched, 300, "upright window");
    assert!(w.block.gating_field > 500, "the gating row is open");

    // A latched fault: the peer's lockdown flag enters the FSM's fault aggregate.
    let mut w = rider_board(0, sensing());
    let mut r = rider_board(1, sensing());
    press_pair(&mut w, &mut r);
    for _ in 0..300 {
        w.inbox.accept(OTHER, cyclic(CyclicState::FLAG_LOCKDOWN));
        r.inbox.accept(OTHER, cyclic(CyclicState::FLAG_LOCKDOWN));
        assert_blocked_alike(&mut w, &mut r, &level, 1, "peer lockdown");
    }
    assert_eq!(w.mode.mode(), Mode::Run, "armed: the fault term blocks");

    // No power enable: never armed, so the machine is never in RUN.
    let mut w = rider_board(0, sensing());
    let mut r = rider_board(1, sensing());
    for _ in 0..2 {
        input_task(
            &mut r,
            &InputSample {
                pad_a_high: true,
                pad_b_high: true,
                ..Default::default()
            },
        );
    }
    assert_blocked_alike(&mut w, &mut r, &level, 300, "power enable");
    assert_eq!(w.mode.mode(), Mode::Off);

    // And with every term satisfied the same pair engages, alike.
    let mut w = rider_board(0, sensing());
    let mut r = rider_board(1, sensing());
    press_pair(&mut w, &mut r);
    for _ in 0..300 {
        let t = control_task(&mut w, Some(&level), 1);
        assert_eq!(t, control_task(&mut r, Some(&level), 1));
    }
    assert_eq!(w.ctl.fsm.sub_state as u8, 3, "the control case engages");
}

/// Degrees (whole, truncated) of an attitude `Out`.
fn fix_deg(deg: base::fixed::Out) -> i32 {
    deg.to_num::<i32>()
}

/// A waived board engaged and promoted to the RUN sub-state, hearing a fresh peer.
fn waived_engaged(level: &imu::Sample) -> OrchestratorState {
    let mut s = rider_board(0, sensing());
    hold_power(&mut s);
    for _ in 0..300 {
        s.inbox.accept(OTHER, cyclic(0));
        control_task(&mut s, Some(level), 1);
    }
    assert_eq!(s.ctl.fsm.sub_state as u8, 3);
    s
}

#[test]
fn on_a_waived_board_the_stops_that_are_not_the_rider_still_stop() {
    let level = level_sample_at(8192);

    // The power_request fall (the button released; on the rover, the mirror expiring) takes the
    // mode machine RUN -> SHUTDOWN -> OFF, and the OFF pass resets the engagement machine.
    let mut s = waived_engaged(&level);
    input_task(&mut s, &InputSample::default());
    assert!(!s.power_request());
    let mut saw_off = false;
    for _ in 0..3 {
        s.inbox.accept(OTHER, cyclic(0));
        let t = control_task(&mut s, Some(&level), 1);
        saw_off |= t.mode_byte == Mode::Off.as_byte();
    }
    assert!(saw_off, "the power_request fall reaches OFF");
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "engagement reset");
    assert_eq!(s.ctl.fsm.env, 0);
    assert_eq!(s.ctl.fsm.torque_setpoint, 0);

    // comms_loss forces IDLE inside the FSM on the tick it asserts.
    let mut s = waived_engaged(&level);
    let mut tripped = false;
    for _ in 0..40 {
        let t = control_task(&mut s, Some(&level), 1);
        if t.comms_loss {
            assert_eq!(s.ctl.fsm.sub_state as u8, 0, "comms_loss forces IDLE");
            tripped = true;
            break;
        }
        assert_eq!(t.sub_state, 3);
    }
    assert!(tripped, "the silent peer trips comms_loss");

    // A latched over-current forces IDLE on the pass that latches it.
    let mut s = waived_engaged(&level);
    s.raise_over_current(0);
    s.inbox.accept(OTHER, cyclic(0));
    control_task(&mut s, Some(&level), 1);
    assert!(s.latches[0].is_latched());
    assert_eq!(s.ctl.fsm.sub_state as u8, 0, "over_current forces IDLE");
}

#[test]
fn throttle_engagement_is_identical_at_either_rider_requirement() {
    // (i): throttle mode is unchanged by the field. It feeds the shell a present rider and a
    // clear wind-down whatever the field says, so the two boards promote, hold and disengage on
    // the same ticks and command the same torque. The gain set the field used to move went with
    // the PID profiles (`specs/state-feedback.md` section 5).
    let mk = |byte: u8| {
        OrchestratorState::new(
            0,
            0,
            0,
            false,
            false,
            attitude::Config::default(),
            sf_start(),
            None,
            store::CONTROL_POS_CLAMP.default(),
            control::DriveTurn::default(),
            byte,
            store::BOARD_BATTERY_FLOOR.default(),
            TEST_IDENTITY,
        )
    };
    let (mut a, mut b) = (mk(1), mk(0));
    for s in [&mut a, &mut b] {
        hold_power(s);
    }
    for k in 0..900usize {
        if k % 40 == 0 {
            let value = if k < 600 { 1000 } else { 0 };
            for s in [&mut a, &mut b] {
                feed_drive(s, value, 0);
                // The mirror's rider bit for the first half, so both profiles are visited.
                let bit = if k < 300 {
                    linkctl::Inputs::RIDER_PRESENT
                } else {
                    0
                };
                s.inbox.accept(CTRL, mirror(0, bit));
            }
        }
        let ta = control_task(&mut a, None, 1);
        let tb = control_task(&mut b, None, 1);
        assert_eq!(
            a.ctl.fsm.sub_state as u8, b.ctl.fsm.sub_state as u8,
            "tick {k}: the field moves no engagement term"
        );
        // One gain set (`specs/state-feedback.md` section 5): nothing the field moves reaches
        // the torque or the machine, folded level or not.
        assert_eq!(ta, tb, "tick {k}: same torque");
        assert_eq!(fsm_of(&a), fsm_of(&b), "tick {k}");
    }
    assert_eq!(a.obs().control_mode, 0);
    assert!(
        !super::dispatch::rider_level(&a),
        "the run ends with no level folded in, the stretch that once separated the two"
    );
}

// --- The low-battery floor (`specs/sensing-and-safety.md`, "The low-battery floor") ----------

/// A rider-waived balance board at floor `floor`, its sensing word seeded at `word` centivolts
/// (the [`sensing`] calibration reads counts as centivolts and the filter seeds on its first
/// conversion), walked for 200 level ticks with power held. Returns the board.
fn engage_attempt(floor: i16, word: u16) -> OrchestratorState {
    engage_attempt_as(floor, word, false)
}

/// [`engage_attempt`] with the shell's torque-mode flag (`BlockWords::torque_is_current`) given.
fn engage_attempt_as(floor: i16, word: u16, torque_is_current: bool) -> OrchestratorState {
    let level = level_sample_at(8192);
    let mut s = policy_board(0, floor, sensing());
    s.vbatt_raw = word;
    s.block.torque_is_current = torque_is_current;
    hold_power(&mut s);
    for _ in 0..200 {
        control_task(&mut s, Some(&level), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run);
    assert_eq!(s.block.battery, word as i16, "the seeded word");
    s
}

#[test]
fn the_battery_floor_reaches_the_dispatch_through_the_constructor() {
    assert_eq!(store::BOARD_BATTERY_FLOOR.default(), 2400);
    let s = policy_board(1, 2400, sensing());
    assert!(!s.ctl.dispatch.battery_ok(2399));
    assert!(s.ctl.dispatch.battery_ok(2400));
}

#[test]
fn the_default_floor_refuses_a_balance_engage_at_2399_and_takes_it_at_2400() {
    let floor = store::BOARD_BATTERY_FLOOR.default();
    let under = engage_attempt(floor, 2399);
    assert_eq!(under.ctl.fsm.sub_state as u8, 0, "2399 cV: no engage");
    assert_eq!(under.obs().torque_setpoint, 0);
    let at = engage_attempt(floor, 2400);
    assert_eq!(at.ctl.fsm.sub_state as u8, 3, "2400 cV: engaged to RUN");
}

#[test]
fn a_zero_or_negative_floor_engages_at_any_known_word() {
    for floor in [0i16, -1, -2400, i16::MIN] {
        for word in [1u16, 100, 2399] {
            let s = engage_attempt(floor, word);
            assert_eq!(
                s.ctl.fsm.sub_state as u8, 3,
                "floor {floor}, word {word}: engaged"
            );
        }
    }
}

#[test]
fn a_run_started_above_the_floor_is_not_ended_by_the_word_falling_under_it() {
    let level = level_sample_at(8192);
    let mut s = engage_attempt(store::BOARD_BATTERY_FLOOR.default(), BENCH_CV);
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "RUN above the floor");
    // Sag: the counts fall to 1000 cV; the filter walks the word down under the floor.
    s.vbatt_raw = 1000;
    for k in 0..500 {
        let t = control_task(&mut s, Some(&level), 1);
        assert_eq!(t.sub_state, 3, "RUN holds at tick {k}");
    }
    assert!(s.block.battery < 2400, "{}", s.block.battery);
    assert!(!s.ctl.dispatch.battery_ok(s.block.battery));
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "still RUN under the floor");
}

/// Throttle mode never consults the floor: a floor above any reachable word still engages.
#[test]
fn the_battery_floor_does_not_gate_a_throttle_engage() {
    let mut s = OrchestratorState::new(
        0,
        0,
        0,
        false,
        false,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        i16::MAX,
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    drive_ticks(&mut s, 200, 20000, 0);
    assert_eq!(s.obs().control_mode, 0, "throttle");
    assert_eq!(s.block.battery, BENCH_CV as i16);
    assert!(
        !s.ctl.dispatch.battery_ok(s.block.battery),
        "under the floor"
    );
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged anyway");
    assert_ne!(s.obs().torque_setpoint, 0);
}

// --- The law's scale in torque mode (`specs/foc-torque.md` section 4) ------------------------

#[test]
fn the_law_divides_by_the_nominal_pack_in_torque_mode_and_by_the_battery_otherwise() {
    assert_eq!(control::config::sf::NOMINAL_SCALE, 2960);
    for battery in [i16::MIN, -1, 0, 1, 2399, 2400, 2960, 3600, i16::MAX] {
        assert_eq!(super::dispatch::law_scale(battery, true), 2960, "{battery}");
        assert_eq!(super::dispatch::law_scale(battery, false), battery);
    }
}

/// A disarmed balance board on the start gains reading `vbatt` counts as centivolts (`None`: no
/// battery source, the word UNKNOWN), the shell's torque-mode flag at `torque_is_current`, walked
/// 50 ticks gently tilted. The law is live disarmed and every term but the pitch term is zero
/// (no gyro rate, a still wheel, no reference outside RUN), so the word is `term_p`.
fn torque_mode_board(vbatt: Option<u16>, torque_is_current: bool) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        vbatt.and(sensing()),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = vbatt.unwrap_or(0);
    s.block.torque_is_current = torque_is_current;
    for _ in 0..50 {
        control_task(&mut s, Some(&gently_tilted_sample()), 1);
    }
    assert_eq!(
        s.block.battery,
        vbatt.unwrap_or(0) as i16,
        "the source rule's word"
    );
    assert_eq!(s.ctl.sf.terms[1..], [0; 3], "the pitch term alone");
    s
}

/// **The orchestrator's half of torque mode** (section 9): with `torque_is_current` set the law's
/// word is `numer * 3900 / 2960` whatever the battery word, UNKNOWN included; clear, it divides by
/// the battery word as before (and an UNKNOWN word makes it 0).
#[test]
fn torque_mode_pins_the_law_at_the_nominal_pack_whatever_the_battery_word() {
    // The pitch term's numerator at the board's own pitch word: `gp * pitch / 100`.
    let numer = |s: &OrchestratorState| 3491 * s.block.pitch_word as i64 / 100;
    let mut pitch = None;
    for vbatt in [Some(BENCH_CV), Some(2000), Some(2960), None] {
        let set = torque_mode_board(vbatt, true);
        let n = numer(&set);
        assert_ne!(n, 0, "the tilt reaches the law");
        assert_eq!(
            *pitch.get_or_insert(set.block.pitch_word),
            set.block.pitch_word
        );
        let want = (n * 3900 / 2960) as i16;
        assert_eq!(set.ctl.sf.terms[0], want, "set, vbatt {vbatt:?}");
        assert_eq!(set.obs().pre_env_torque, want, "set, vbatt {vbatt:?}");

        let clear = torque_mode_board(vbatt, false);
        assert_eq!(clear.block.pitch_word, set.block.pitch_word);
        let by_battery = match vbatt {
            Some(cv) => (n * 3900 / cv as i64) as i16,
            None => 0,
        };
        assert_eq!(clear.ctl.sf.terms[0], by_battery, "clear, vbatt {vbatt:?}");
        assert_eq!(clear.obs().pre_env_torque, by_battery);
        assert_eq!(
            set.ctl.sf.terms[0] == clear.ctl.sf.terms[0],
            vbatt == Some(2960),
            "the two divisors agree at the nominal pack only, vbatt {vbatt:?}"
        );
    }
}

/// The engage floor gate (`BOARD_BATTERY_FLOOR`) reads the MEASURED battery word in torque mode
/// exactly as it does otherwise: 2399 cV refuses, 2400 engages, and an UNKNOWN word refuses, with
/// the flag set or clear, although the law's divisor no longer moves with the word.
#[test]
fn the_engage_floor_reads_the_battery_word_in_torque_mode_too() {
    let floor = store::BOARD_BATTERY_FLOOR.default();
    for torque_is_current in [false, true] {
        let under = engage_attempt_as(floor, 2399, torque_is_current);
        assert_eq!(
            under.ctl.fsm.sub_state as u8, 0,
            "{torque_is_current}: 2399 cV, no engage"
        );
        let at = engage_attempt_as(floor, 2400, torque_is_current);
        assert_eq!(
            at.ctl.fsm.sub_state as u8, 3,
            "{torque_is_current}: 2400 cV, RUN"
        );

        let level = level_sample_at(8192);
        let mut unknown = policy_board(0, floor, None);
        unknown.block.torque_is_current = torque_is_current;
        hold_power(&mut unknown);
        for _ in 0..200 {
            control_task(&mut unknown, Some(&level), 1);
        }
        assert_eq!(unknown.mode.mode(), Mode::Run);
        assert_eq!(unknown.block.battery, 0, "UNKNOWN");
        assert_eq!(
            unknown.ctl.fsm.sub_state as u8, 0,
            "{torque_is_current}: UNKNOWN, no engage"
        );
    }
}

// --- The rigid frame: two wheels from one IMU (`specs/rover-one-imu.md`, slice 2) -------------
//
// The rover: one IMU on the master, none on the slave, both boards staged CONTROL_MODE = 1 and
// CONTROL_FRAME = 1, the rider requirement waived. The resolution table makes the master the
// LEADER and the slave the FOLLOWER; the leader's torque reaches its own wheel and the wire, the
// follower applies the word, and both gate their wheel on the peer cyclic's freshness.

/// A rigid-frame board: the LEADER when `imu` (a sensing board reading a known battery word), the
/// FOLLOWER when not. Rider requirement waived, as the rover stages it.
fn rigid_board(imu: bool) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        1,
        0,
        imu,
        imu,
        attitude::Config::default(),
        sf_start(),
        if imu { sensing() } else { None },
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        0,
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    s
}

/// A leader's cyclic as the follower hears it: the leader flag, the leader's mode byte and the
/// word it commands.
fn leader_cyclic(mode: Mode, demand: i16) -> Payload {
    Payload::CyclicState(CyclicState {
        pitch: 0,
        roll: 0,
        wheel_speed: 0,
        battery: 3600,
        mode: mode.as_byte(),
        fault: 0,
        flags: CyclicState::FLAG_LEADER,
        peer_demand: demand,
        wheel_pos: 0,
        obs: PEER_OBS,
        wheel_rate: 0,
    })
}

/// A follower's cyclic as the leader hears it: the follower flag and nothing commanded.
fn follower_cyclic() -> Payload {
    cyclic(CyclicState::FLAG_FOLLOWER)
}

/// Tick `s` (with `sample` in) until its cyclic emission is due, and return the frame it emits:
/// one or two ticks, since the cadence is every second served tick.
fn next_cyclic(s: &mut OrchestratorState, sample: Option<&imu::Sample>) -> CyclicState {
    loop {
        control_task(s, sample, 1);
        if let Some(c) = cyclic_tx(s, true) {
            return c;
        }
    }
}

/// A tilted, otherwise healthy IMU sample: the balance law commands torque on it.
fn tilted_sample() -> imu::Sample {
    imu::Sample {
        accel_raw: [2000, 0, 8000],
        ..level_sample()
    }
}

/// A gently tilted sample: the balance law commands torque on it and stays well inside the +-28500
/// output rail, which is what the steering split needs to have headroom for a turn.
fn gently_tilted_sample() -> imu::Sample {
    imu::Sample {
        accel_raw: [60, 0, 8192],
        ..level_sample()
    }
}

/// Run `n` ticks on `s`, accepting `peer` into the inbox every second tick (the 125 Hz cadence a
/// live peer emits at), with `sample` in.
fn run_with_peer(
    s: &mut OrchestratorState,
    n: usize,
    peer: &Payload,
    sample: Option<&imu::Sample>,
) {
    for k in 0..n {
        if k % 2 == 0 {
            s.inbox.accept(OTHER, *peer);
        }
        control_task(s, sample, 1);
    }
}

/// **The resolution table reaches the dispatch through the constructor** (`specs/rover-one-imu.md`
/// section 2): a rigid board with an IMU is the LEADER (`CTRL_OBS` control-mode byte 2), one
/// without is the FOLLOWER (3), and neither is a demotion. The pivoting rows are the existing
/// tests' and are unchanged.
#[test]
fn a_rigid_board_resolves_to_leader_with_an_imu_and_follower_without() {
    let leader = rigid_board(true);
    assert_eq!(leader.obs().control_mode, control::Producer::Leader as u8);
    assert_eq!(leader.obs().control_mode, 2);
    assert!(!leader.obs().mode_fault);
    let follower = rigid_board(false);
    assert_eq!(
        follower.obs().control_mode,
        control::Producer::Follower as u8
    );
    assert_eq!(follower.obs().control_mode, 3);
    assert!(
        !follower.obs().mode_fault,
        "no IMU on a rigid frame is the follower, not a demotion"
    );
}

/// **A leader and a follower say so on the wire, and nobody else does** (section 4): the leader
/// flag and the follower flag are facts of the resolution, emitted whether or not the board is
/// armed, so a leader can refuse a pivoting peer before either engages. `peer_demand` is 0 from
/// every disengaged board and from every non-leader.
#[test]
fn a_leader_and_a_follower_say_so_on_the_wire_and_nobody_else_does() {
    let level = level_sample();
    let mut leader = rigid_board(true);
    let c = next_cyclic(&mut leader, Some(&level));
    assert!(c.leader() && !c.follower());
    assert_eq!(c.peer_demand, 0, "disengaged: nothing commanded yet");

    let mut follower = rigid_board(false);
    let c = next_cyclic(&mut follower, None);
    assert!(c.follower() && !c.leader());
    assert_eq!(c.peer_demand, 0);

    let mut throttle = fresh();
    let c = next_cyclic(&mut throttle, None);
    assert!(!c.leader() && !c.follower());

    let mut pivoting = balance_to_run(&level);
    let c = next_cyclic(&mut pivoting, Some(&level));
    assert!(!c.leader() && !c.follower());
    assert_eq!(c.peer_demand, 0);
}

/// **The follower applies its leader's word while the leader is fresh and in RUN** (section 2):
/// it engages on the leader's MODE byte, not on the demand's magnitude, and the word is the
/// reference the shell envelopes, so once the soft-start has ramped past it the wheel carries
/// exactly the leader's number. No attitude, no IMU health, no battery divide anywhere in it.
#[test]
fn a_follower_applies_its_leaders_word_while_the_leader_is_fresh_and_running() {
    let mut f = rigid_board(false);
    hold_power(&mut f); // the app arms both nodes
    run_with_peer(&mut f, 60, &leader_cyclic(Mode::Run, 5000), None);
    assert_eq!(f.mode.mode(), Mode::Run);
    assert_ne!(f.ctl.fsm.sub_state as u8, 0, "engaged on the leader's RUN");
    assert_eq!(
        f.obs().pre_env_torque,
        5000,
        "the leader's word is the reference"
    );
    assert_eq!(
        f.obs().torque_setpoint,
        5000,
        "and, enveloped, the wheel demand"
    );
    assert_eq!(f.ctl.wheel_demand, f.ctl.fsm.torque_setpoint);
    assert!(!f.obs().imu_loss, "absence is not loss");
    assert!(!f.obs().comms_loss);
    assert_eq!(
        f.events.levels(),
        EV_POWER_REQUEST,
        "no producer but its own power request"
    );

    // The word moves, the wheel follows, sign and all: the leader's +-28500 word verbatim.
    run_with_peer(&mut f, 4, &leader_cyclic(Mode::Run, -3000), None);
    assert_eq!(f.obs().torque_setpoint, -3000);
}

/// **A stale leader is a zeroed wheel within the rigid window, and a disarm at the supervision
/// window** (section 6, the failure table's first two rows). The reference-zeroing is NOT a
/// fault: the engagement machine keeps its sub-state and the next fresh frame restores torque.
#[test]
fn a_follower_zeroes_its_wheel_within_the_rigid_window_and_disarms_at_the_supervision_window() {
    let mut f = rigid_board(false);
    hold_power(&mut f);
    run_with_peer(&mut f, 60, &leader_cyclic(Mode::Run, 5000), None);
    assert_eq!(f.obs().torque_setpoint, 5000);
    let sub_state = f.ctl.fsm.sub_state as u8;
    assert_ne!(sub_state, 0);

    // The leader goes quiet. The last frame was accepted before tick 58 of 60, so the age is
    // already 2; at 4 the frame is still fresh (two nominal periods), at 5 it is not.
    assert_eq!(f.inbox.cyclic_age(), 2);
    for age in 3..=RIGID_STALE_TICKS {
        control_task(&mut f, None, 1);
        assert_eq!(f.inbox.cyclic_age(), age);
        assert_eq!(f.obs().torque_setpoint, 5000, "fresh at age {age}");
    }
    control_task(&mut f, None, 1);
    assert_eq!(f.inbox.cyclic_age(), RIGID_STALE_TICKS + 1);
    assert_eq!(f.obs().torque_setpoint, 0, "the wheel is zeroed");
    assert_eq!(f.ctl.wheel_demand, 0);
    assert_eq!(
        f.obs().pre_env_torque,
        0,
        "and the shadow says the board is told nothing"
    );
    assert_eq!(f.mode.mode(), Mode::Run, "not a fault");
    assert_eq!(
        f.ctl.fsm.sub_state as u8, sub_state,
        "the machine keeps its sub-state"
    );
    assert!(!f.obs().comms_loss);

    // One fresh frame restores the torque on the same pass.
    f.inbox.accept(OTHER, leader_cyclic(Mode::Run, 5000));
    control_task(&mut f, None, 1);
    assert_eq!(f.obs().torque_setpoint, 5000, "restored by the next frame");

    // Quiet past CYCLIC_TIMEOUT_TICKS: comms_loss, the ordinary disarm.
    run_ticks(&mut f, 26);
    assert!(f.obs().comms_loss);
    assert_ne!(f.mode.mode(), Mode::Run);
    assert_eq!(f.obs().torque_setpoint, 0);
}

/// **A follower does not engage on a leader that is not running, nor on a peer that does not
/// lead** (section 2): the gate is the leader's RUN under the leader flag, and the word is applied
/// only under the flag, so a pivoting peer's cyclic commands nothing.
#[test]
fn a_follower_does_not_engage_on_a_leader_that_is_not_running_or_a_peer_that_does_not_lead() {
    // A leader that is powered but OFF: fresh, flagged, not in RUN.
    let mut f = rigid_board(false);
    hold_power(&mut f);
    run_with_peer(&mut f, 100, &leader_cyclic(Mode::Off, 5000), None);
    assert_eq!(f.mode.mode(), Mode::Run, "this board is armed");
    assert_eq!(f.ctl.fsm.sub_state as u8, 0, "and does not engage");
    assert_eq!(f.obs().torque_setpoint, 0);

    // A peer in RUN that is not a leader (a pivoting board, or a second follower).
    let mut f = rigid_board(false);
    hold_power(&mut f);
    run_with_peer(&mut f, 100, &cyclic(CyclicState::FLAG_RIDER), None);
    assert_eq!(f.ctl.fsm.sub_state as u8, 0);
    assert_eq!(f.obs().torque_setpoint, 0);
    assert_eq!(
        f.obs().pre_env_torque,
        0,
        "a peer's zero is a zero, not a command"
    );
    let mut f = rigid_board(false);
    hold_power(&mut f);
    run_with_peer(&mut f, 100, &follower_cyclic(), None);
    assert_eq!(f.ctl.fsm.sub_state as u8, 0);
}

/// **A follower powered with no peer at all is inert and healthy** (section 6): the
/// never-seen-a-peer exemption keeps `comms_loss` clear, no leader in RUN keeps the gate at 0,
/// and its wheel never turns.
#[test]
fn a_follower_with_no_peer_is_inert_and_healthy() {
    let mut f = rigid_board(false);
    hold_power(&mut f);
    run_ticks(&mut f, 200);
    assert_eq!(f.mode.mode(), Mode::Run);
    assert!(!f.obs().comms_loss);
    assert_eq!(f.ctl.fsm.sub_state as u8, 0);
    assert_eq!(f.obs().torque_setpoint, 0);
    assert_eq!(f.events.levels(), EV_POWER_REQUEST);
}

/// **The leader balances only while a fresh follower answers** (sections 2, 4 and 6): with a
/// fresh peer cyclic carrying the follower flag it engages on its own up-axis gate and its wheel
/// demand is the law's torque, which also goes on the wire as `peer_demand` under the leader
/// flag. When the follower goes quiet past the rigid window the leader's wheel is zeroed AND it
/// commands its peer nothing, while the law keeps running (a reference-zeroing, not a
/// fault); the next fresh frame restores both.
#[test]
fn a_leader_balances_only_while_a_fresh_follower_answers() {
    let tilted = tilted_sample();
    let mut l = rigid_board(true);
    hold_power(&mut l);
    run_with_peer(&mut l, 100, &follower_cyclic(), Some(&tilted));
    assert_eq!(l.mode.mode(), Mode::Run);
    assert_ne!(l.ctl.fsm.sub_state as u8, 0, "engaged on its own gate");
    let c = next_cyclic(&mut l, Some(&tilted));
    let torque = l.obs().torque_setpoint;
    assert_ne!(torque, 0, "the law commands torque on a tilt");
    assert_eq!(torque, l.ctl.fsm.torque_setpoint, "the wheel takes T");
    assert!(c.leader());
    assert_eq!(c.peer_demand, torque, "and so does the wire");

    // The follower goes quiet while the IMU stays live: six ticks is past the window.
    for _ in 0..6 {
        control_task(&mut l, Some(&tilted), 1);
    }
    assert!(l.inbox.cyclic_age() > RIGID_STALE_TICKS);
    assert_eq!(l.obs().torque_setpoint, 0, "own wheel zeroed");
    assert_ne!(l.ctl.fsm.torque_setpoint, 0, "the law keeps running");
    let c = next_cyclic(&mut l, Some(&tilted));
    assert_eq!(
        c.peer_demand, 0,
        "a leader that cannot hear its follower commands it nothing"
    );
    assert!(
        c.leader(),
        "the role is a fact of the resolution, not of the link"
    );
    assert_eq!(l.mode.mode(), Mode::Run);
    assert!(!l.obs().comms_loss);

    // One fresh frame restores both.
    l.inbox.accept(OTHER, follower_cyclic());
    let c = next_cyclic(&mut l, Some(&tilted));
    assert_ne!(l.obs().torque_setpoint, 0);
    assert_eq!(c.peer_demand, l.obs().torque_setpoint);
}

/// **A leader whose peer is not a rigid follower does not engage** (section 6, "A pair that
/// disagrees"): a pivoting peer's cyclic is fresh and carries no follower flag, so the leader's
/// gate is cleared and the machine will not arm on one wheel. No fault is raised for it: the
/// refusal is the gate, and the peer's own state says what it is running.
#[test]
fn a_leader_whose_peer_is_not_a_rigid_follower_does_not_engage() {
    let tilted = tilted_sample();
    for peer in [cyclic(0), cyclic(CyclicState::FLAG_RIDER)] {
        let mut l = rigid_board(true);
        hold_power(&mut l);
        run_with_peer(&mut l, 200, &peer, Some(&tilted));
        assert_eq!(l.mode.mode(), Mode::Run);
        assert!(l.block.gating_field > 500, "its own gate row is open");
        assert_eq!(l.ctl.fsm.sub_state as u8, 0, "and still it does not engage");
        assert_eq!(l.obs().torque_setpoint, 0);
        assert!(!l.obs().mode_fault);
    }
    // The same leader with a follower answering engages: the refusal was the peer's flags.
    let mut l = rigid_board(true);
    hold_power(&mut l);
    run_with_peer(&mut l, 200, &follower_cyclic(), Some(&tilted));
    assert_ne!(l.ctl.fsm.sub_state as u8, 0);
}

/// **Two leaders is a staging error and stops the machine** (section 2): a leader hearing a fresh
/// peer cyclic with the leader flag raises the mode fault AND feeds the shell a cleared gate. The
/// second half is the load-bearing one: `mode_fault` is not a Fault A, so the level alone would
/// publish a complaint and keep balancing. The level reaches the cyclic `fault` word, so the app
/// sees why neither board arms, and it releases when the peer stops claiming the role.
#[test]
fn two_leaders_refuse_each_other_with_the_mode_fault() {
    let tilted = tilted_sample();
    let mut l = rigid_board(true);
    hold_power(&mut l);
    run_with_peer(&mut l, 200, &leader_cyclic(Mode::Run, 4000), Some(&tilted));
    assert!(l.obs().mode_fault, "the level");
    assert_ne!(
        l.events.levels() & EV_MODE_FAULT,
        0,
        "counted and published"
    );
    assert_ne!(
        next_cyclic(&mut l, Some(&tilted)).fault & EV_MODE_FAULT,
        0,
        "and on the wire for the app"
    );
    assert_eq!(
        l.mode.mode(),
        Mode::Run,
        "not a Fault A: the mode machine is untouched"
    );
    assert_eq!(l.ctl.fsm.sub_state as u8, 0, "the gate is what refuses");
    assert_eq!(l.obs().torque_setpoint, 0);
    assert_ne!(
        l.obs().pre_env_torque,
        0,
        "the law still shows what it would command"
    );

    // The peer corrects its staging and reports as the follower: the level releases and the
    // leader engages through the ordinary gates.
    run_with_peer(&mut l, 100, &follower_cyclic(), Some(&tilted));
    assert!(!l.obs().mode_fault);
    assert_ne!(l.ctl.fsm.sub_state as u8, 0);
}

/// **A pivoting board is byte-identical** (section 2: `control.frame` defaults to 0): its wheel
/// demand is the sole-writer row with no gate on it, and a peer claiming either rigid role
/// changes nothing about how it balances or what it reports.
#[test]
fn a_pivoting_board_ignores_the_rigid_roles_and_gates_nothing() {
    let tilted = tilted_sample();
    let mut s = balance_to_run(&tilted);
    for peer in [leader_cyclic(Mode::Run, 4000), follower_cyclic()] {
        run_with_peer(&mut s, 100, &peer, Some(&tilted));
        assert_ne!(s.ctl.fsm.sub_state as u8, 0, "balancing");
        assert_eq!(s.obs().torque_setpoint, s.ctl.fsm.torque_setpoint);
        assert_ne!(s.obs().torque_setpoint, 0);
        assert!(!s.obs().mode_fault);
        let c = next_cyclic(&mut s, Some(&tilted));
        assert!(!c.leader() && !c.follower());
        assert_eq!(c.peer_demand, 0);
    }
    // And a quiet peer costs it nothing within the rigid window either: no gate.
    for _ in 0..6 {
        control_task(&mut s, Some(&tilted), 1);
    }
    assert!(s.inbox.cyclic_age() > RIGID_STALE_TICKS);
    assert_ne!(
        s.obs().torque_setpoint,
        0,
        "no rigid gate on a pivoting board"
    );
}

// --- peer_down: the one-wheel-fault interlock (`specs/rover-one-imu.md` section 6, slice 3) ---
//
// On a rigid two-wheel balancer a wheel that stops while the other drives is worse than both
// stopping: one wheel is half the fore-aft authority plus a yaw moment the loop has no sensor
// for, and a disarmed wheel freewheels. So a board whose fresh peer reports any Fault-A producer
// folds that into its OWN `fault_a` and stops too.

/// The same peer payload with its `fault` word set: the peer's own producer levels, which is what
/// `CYCLIC_STATE.fault` carries (section 4).
fn with_fault(p: Payload, fault: u16) -> Payload {
    match p {
        Payload::CyclicState(mut c) => {
            c.fault = fault;
            Payload::CyclicState(c)
        }
        other => other,
    }
}

/// **A rigid board stops when its peer reports a Fault A, in BOTH directions** (the failure
/// table's "follower latches a fault" and "leader latches a fault" rows): the level folds into
/// `fault_a`, the mode machine leaves RUN, and the wheel coasts. And it is **self-clearing**: the
/// peer's producers clear on its own OFF pass, its next cyclic reports 0, and the pair re-arms
/// through the ordinary gates with no power cycle.
#[test]
fn a_rigid_board_stops_when_its_peer_reports_a_fault_a_and_re_arms_when_it_clears() {
    let tilted = tilted_sample();

    // The FOLLOWER hearing a leader that has latched a motor fault.
    let mut f = rigid_board(false);
    hold_power(&mut f);
    let running = leader_cyclic(Mode::Run, 5000);
    run_with_peer(&mut f, 60, &running, None);
    assert_eq!(f.mode.mode(), Mode::Run);
    run_with_peer(&mut f, 2, &with_fault(running, EV_MOTOR_HALL), None);
    assert_ne!(f.mode.mode(), Mode::Run, "peer_down is a Fault A");
    assert_ne!(
        f.events.levels() & EV_COMMS_LOSS,
        0,
        "it rides the comms-loss bit, which is where the bench reads it"
    );
    assert!(
        !f.obs().comms_loss,
        "the LINK is fine; the peer is not. Obs reports the link level"
    );
    run_with_peer(&mut f, 200, &running, None);
    assert_eq!(f.mode.mode(), Mode::Run, "self-clearing, no power cycle");
    assert_eq!(f.events.levels() & EV_COMMS_LOSS, 0);

    // The LEADER hearing a follower that has latched one. Same level, same outcome.
    let mut l = rigid_board(true);
    hold_power(&mut l);
    run_with_peer(&mut l, 200, &follower_cyclic(), Some(&tilted));
    assert_eq!(l.mode.mode(), Mode::Run);
    assert_ne!(l.ctl.fsm.sub_state as u8, 0, "balancing");
    run_with_peer(
        &mut l,
        2,
        &with_fault(follower_cyclic(), EV_LATCH_A),
        Some(&tilted),
    );
    assert_ne!(l.mode.mode(), Mode::Run);
    assert_eq!(l.ctl.wheel_demand, 0, "and the wheel coasts");
}

/// **A peer's one-way comms loss stops this board too** (section 6, and the reason `peer_down`
/// tests the whole Fault-A group rather than skipping `EV_COMMS_LOSS`). If A can hear B but B
/// cannot hear A, B's supervision disarms B at 100 ms while A's own cyclic age never grows: A
/// would otherwise balance on one wheel against a freewheeling one. B says so in its fault word,
/// which is the only evidence A has.
#[test]
fn a_peers_own_comms_loss_is_a_peer_down_here() {
    let tilted = tilted_sample();
    let mut l = rigid_board(true);
    hold_power(&mut l);
    run_with_peer(&mut l, 200, &follower_cyclic(), Some(&tilted));
    assert_eq!(l.mode.mode(), Mode::Run);
    run_with_peer(
        &mut l,
        2,
        &with_fault(follower_cyclic(), EV_COMMS_LOSS),
        Some(&tilted),
    );
    assert!(
        l.inbox.cyclic_age() <= RIGID_STALE_TICKS,
        "this board hears the peer fine"
    );
    assert!(!l.obs().comms_loss);
    assert_ne!(l.mode.mode(), Mode::Run, "and stops anyway");
}

/// **`peer_down` is never echoed onto the wire** (section 4, `events::Levels::own_mask`): the
/// `fault` word carries this board's OWN producers. Were the level echoed, one board's motor
/// fault would set the other's `peer_down`, whose bit would set the first board's, and neither
/// could clear: `fault_a` gates OFF -> INIT, so the pair would refuse to arm until power-cycled.
#[test]
fn peer_down_is_never_echoed_onto_the_wire() {
    let mut f = rigid_board(false);
    hold_power(&mut f);
    let faulted = with_fault(leader_cyclic(Mode::Run, 5000), EV_MOTOR_PERIOD);
    run_with_peer(&mut f, 60, &faulted, None);
    assert_ne!(f.events.levels() & EV_COMMS_LOSS, 0, "asserted locally");
    f.inbox.accept(OTHER, faulted);
    let c = next_cyclic(&mut f, None);
    assert_eq!(
        c.fault & EV_COMMS_LOSS,
        0,
        "and absent from what this board tells its peer"
    );

    // The contrast, so the byte is not simply always clear: a board whose own link has gone quiet
    // past the supervision window reports the bit, because that producer IS its own.
    let mut q = rigid_board(false);
    hold_power(&mut q);
    run_with_peer(&mut q, 10, &leader_cyclic(Mode::Run, 5000), None);
    run_ticks(&mut q, CYCLIC_TIMEOUT_TICKS as usize + 2);
    assert!(q.obs().comms_loss);
    let c = next_cyclic(&mut q, None);
    assert_ne!(c.fault & EV_COMMS_LOSS, 0);
}

/// **The level is the peer's FAULT-A group and nothing else** (section 6): `EV_POWER_REQUEST`,
/// which an armed board holds for the whole arm, and `EV_MODE_FAULT` are in the peer's mask and
/// are not faults. Reading the whole byte would disarm the machine the moment the other board
/// armed.
#[test]
fn a_peers_non_fault_levels_are_not_peer_down() {
    let mut f = rigid_board(false);
    hold_power(&mut f);
    let running = leader_cyclic(Mode::Run, 5000);
    run_with_peer(
        &mut f,
        200,
        &with_fault(running, EV_POWER_REQUEST | EV_MODE_FAULT | EV_LATCH_B),
        None,
    );
    assert_eq!(f.mode.mode(), Mode::Run);
    assert_eq!(f.events.levels(), EV_POWER_REQUEST, "its own, and no other");
    assert_eq!(f.obs().torque_setpoint, 5000, "still driving the wheel");
}

/// **A stale peer's fault is not `peer_down`** (section 6): the level reads the peer through the
/// rigid freshness window like every other rigid consumer, so past it the level releases and the
/// slower 100 ms `comms_loss` supervision is the producer that takes over. The wheel is at zero
/// throughout either way, by the freshness gate.
#[test]
fn a_stale_peers_fault_is_not_peer_down() {
    let mut f = rigid_board(false);
    f.inbox.accept(
        OTHER,
        with_fault(leader_cyclic(Mode::Run, 5000), EV_MOTOR_PERIOD),
    );
    control_task(&mut f, None, 1);
    assert_ne!(f.events.levels() & EV_COMMS_LOSS, 0, "fresh: asserted");
    for _ in 0..RIGID_STALE_TICKS {
        control_task(&mut f, None, 1);
    }
    assert_eq!(f.inbox.cyclic_age(), RIGID_STALE_TICKS + 1);
    assert_eq!(f.events.levels() & EV_COMMS_LOSS, 0, "stale: released");
    assert_eq!(f.ctl.wheel_demand, 0);
}

/// **A pivoting board ignores its peer's fault mask** (section 6: the level is rigid-only). Two
/// platforms are two machines, and the stock behaviour must not change under a peer that faults.
#[test]
fn a_pivoting_board_ignores_its_peers_fault_mask() {
    let tilted = tilted_sample();
    let mut s = balance_to_run(&tilted);
    run_with_peer(
        &mut s,
        100,
        &with_fault(cyclic(0), EV_MOTOR_CAL | EV_IMU_LOSS | EV_LATCH_A),
        Some(&tilted),
    );
    assert_eq!(s.mode.mode(), Mode::Run);
    assert_eq!(s.events.levels() & EV_COMMS_LOSS, 0);
    assert_ne!(s.obs().torque_setpoint, 0, "still balancing");
}

// --- Steering: the split and the saturation rule (`specs/rover-one-imu.md` section 5, slice 4) -
//
// The turn is a differential torque applied on the LEADER, after the engagement machine has
// written the machine's common torque T. The follower performs one assignment and has no notion
// of which wall it is on.

/// A rigid LEADER staged with a side and a turn bound: `side_right` is the rover's master on the
/// right wall, and the turn is the `CONTROL_DRIVE_TURN` pair through its seam.
fn turning_leader(side_right: bool, turn: control::DriveTurn) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        1,
        side_right as u8,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        turn,
        0,
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    hold_power(&mut s);
    s
}

/// Run `n` ticks with a fresh follower answering at 125 Hz, a `DRIVE_CMD` steer held, and the
/// IMU sample in.
fn run_turning(s: &mut OrchestratorState, n: usize, steer: i16, sample: &imu::Sample) {
    for k in 0..n {
        if k % 2 == 0 {
            s.inbox.accept(OTHER, follower_cyclic());
        }
        if k % 40 == 0 {
            feed_drive(s, 0, steer);
        }
        control_task(s, Some(sample), 1);
    }
}

/// **The leader splits its torque by its own side and commands its peer the other half**
/// (section 5): `own + peer == 2T` on the wire as in the block, positive steer eases the right
/// wheel off forward drive, and the follower is told a number it applies without knowing why.
#[test]
fn the_leader_splits_its_torque_by_its_side_and_commands_the_peer_the_other_half() {
    // A GENTLE tilt, so the law has headroom and the turn arrives in full: at the full tilt
    // the other vectors use, T saturates at 28,500 and the saturation rule below takes the whole
    // turn away, which is the next test's subject rather than this one's.
    let tilted = gently_tilted_sample();
    let turn = control::DriveTurn::new(3_000, 1_000);
    for side_right in [false, true] {
        let mut l = turning_leader(side_right, turn);
        run_turning(&mut l, 300, 32_767, &tilted);
        assert_ne!(l.ctl.fsm.sub_state as u8, 0, "balancing");
        assert_eq!(l.ctl.turn, 3_000, "the turn is at its bound");
        let t = l.ctl.fsm.torque_setpoint as i32;
        assert_ne!(t, 0, "the law is commanding torque");
        let (own, peer) = (l.ctl.wheel_demand as i32, l.ctl.peer_demand as i32);
        assert_eq!(own + peer, 2 * t, "the common torque is preserved exactly");
        assert_eq!(
            (own - peer).abs(),
            2 * 3_000,
            "and the pair differs by twice the turn"
        );
        if side_right {
            assert!(
                own > peer,
                "the right wheel is the less forward one on a right turn, and forward is \
                 negative in this word (`specs/control.md` (a))"
            );
        } else {
            assert!(own < peer, "and the left wheel is the more forward one");
        }
        // What goes on the wire is the PEER's half, not this board's.
        let c = next_cyclic(&mut l, Some(&tilted));
        assert_eq!(c.peer_demand as i32, peer);
        assert!(c.leader());
    }
}

/// **At full balance authority the turn is zero** (section 5): the saturation rule gives up the
/// TURN and keeps the common torque, which is the fore-aft torque holding the machine up. The
/// app's throttle mixer does the opposite on purpose, and for a speed demand it is right; for a
/// balance torque it is a fall.
#[test]
fn a_saturated_leader_gives_up_the_turn_and_keeps_the_common_torque() {
    let hard = tilted_sample();
    let mut l = turning_leader(true, control::DriveTurn::new(3_000, 1_000));
    run_turning(&mut l, 300, 32_767, &hard);
    assert_eq!(l.ctl.turn, 3_000, "the command itself is unchanged");
    let t = l.ctl.fsm.torque_setpoint;
    assert_eq!(t.unsigned_abs(), 28_500, "the law is at full authority");
    assert_eq!(l.ctl.wheel_demand, t, "no turn applied");
    assert_eq!(l.ctl.peer_demand, t);
}

/// **A board with no turn staged is byte-identical** (section 5): `turn_max` 0 leaves the carry at
/// zero whatever the stick does, so both halves are the setpoint and every existing vector means
/// what it did.
#[test]
fn a_leader_with_no_turn_staged_drives_both_wheels_the_same() {
    let tilted = tilted_sample();
    let mut l = turning_leader(true, control::DriveTurn::default());
    run_turning(&mut l, 300, 32_767, &tilted);
    assert_eq!(l.ctl.turn, 0, "the carry never moves");
    assert_ne!(l.ctl.wheel_demand, 0);
    assert_eq!(l.ctl.wheel_demand, l.ctl.fsm.torque_setpoint);
    assert_eq!(l.ctl.peer_demand, l.ctl.wheel_demand);
}

/// **The turn slews in and decays out with the drive command** (section 5): a step in
/// differential torque is a yaw impulse, which on a rigid frame reaches the pitch loop as a
/// disturbance through the steel, so the turn arrives at `turn_slew` per tick and a stale
/// `DRIVE_CMD` walks it back to neutral rather than dropping it.
#[test]
fn the_turn_slews_in_and_decays_out_with_the_drive_command() {
    let tilted = tilted_sample();
    let mut l = turning_leader(true, control::DriveTurn::new(3_000, 24));
    run_turning(&mut l, 20, 32_767, &tilted);
    let partway = l.ctl.turn;
    assert!(
        partway > 0 && partway < 3_000,
        "still ramping, at {partway}"
    );
    run_turning(&mut l, 300, 32_767, &tilted);
    assert_eq!(l.ctl.turn, 3_000);
    // The drive command goes stale (DRIVE_TIMEOUT_TICKS = 50): the steer reads 0 and the slew is
    // the decay ramp, so the machine straightens out in half a second rather than snapping.
    for k in 0..200 {
        if k % 2 == 0 {
            l.inbox.accept(OTHER, follower_cyclic());
        }
        control_task(&mut l, Some(&tilted), 1);
    }
    assert_eq!(l.ctl.turn, 0, "back to straight");
    assert_eq!(l.ctl.peer_demand, l.ctl.wheel_demand);
}

/// **A stale peer zeroes BOTH halves of a turning leader** (sections 5 and 6): the freshness gate
/// is applied after the split, so a quiet link stops the machine. Gating the common torque first
/// would leave the turn to be split out of a zero, which is two equal and opposite NONZERO wheel
/// demands: a quiet link would spin the rover instead of stopping it.
#[test]
fn a_stale_peer_zeroes_both_halves_of_a_turning_leader() {
    let tilted = tilted_sample();
    let mut l = turning_leader(true, control::DriveTurn::new(3_000, 1_000));
    run_turning(&mut l, 300, 32_767, &tilted);
    assert_eq!(l.ctl.turn, 3_000);
    assert_ne!(l.ctl.wheel_demand, 0);
    assert_ne!(l.ctl.peer_demand, 0);
    for _ in 0..=RIGID_STALE_TICKS + 1 {
        control_task(&mut l, Some(&tilted), 1);
    }
    assert!(l.inbox.cyclic_age() > RIGID_STALE_TICKS);
    assert_eq!(l.ctl.turn, 3_000, "the turn carry is not what was gated");
    assert_eq!(l.ctl.wheel_demand, 0);
    assert_eq!(l.ctl.peer_demand, 0);
    let c = next_cyclic(&mut l, Some(&tilted));
    assert_eq!(c.peer_demand, 0, "and it commands its peer nothing");
}

/// **The steer moves no common torque** (`specs/rover-one-imu.md` section 5): on the rigid leader
/// it is the differential the split applies after the output stage, so the pre-envelope torque is
/// the same with the stick hard over as with it centred. The stock shaper's steer-as-common-lean
/// path, which a pivoting board used to keep, went with the shaper (`specs/state-feedback.md`
/// section 5), so a pivoting board's steer reaches no torque either.
#[test]
fn the_steer_moves_no_common_torque() {
    let tilted = tilted_sample();
    let mut straight = turning_leader(true, control::DriveTurn::default());
    run_turning(&mut straight, 300, 0, &tilted);
    let mut steered = turning_leader(true, control::DriveTurn::default());
    run_turning(&mut steered, 300, 32_767, &tilted);
    assert_ne!(
        straight.obs().pre_env_torque,
        0,
        "the compared word is live"
    );
    assert_eq!(straight.obs().pre_env_torque, steered.obs().pre_env_torque);

    let mut quiet = balance_to_run(&tilted);
    drive_ticks(&mut quiet, 300, 0, 0);
    let mut turned = balance_to_run(&tilted);
    drive_ticks(&mut turned, 300, 0, 32_767);
    assert_eq!(quiet.obs().pre_env_torque, turned.obs().pre_env_torque);
}

/// **`control.side` is `role_right`'s owner, at boot and at every arm** (section 5): the block
/// word has existed and been read in two places since the throttle producer was built and has
/// never been written. Staging the side and re-arming moves it, with no reboot.
#[test]
fn the_side_owns_role_right_at_boot_and_at_the_arm() {
    assert!(
        !turning_leader(false, control::DriveTurn::default())
            .block
            .role_right
    );
    assert!(
        turning_leader(true, control::DriveTurn::default())
            .block
            .role_right
    );
    // An unknown byte reads as the registered default, which is left.
    let mut s = fresh();
    for (byte, right) in [(1u8, true), (0, false), (2, false), (255, false)] {
        re_apply_control_values(
            &mut s,
            store::CONTROL_MODE.default(),
            store::CONTROL_FRAME.default(),
            byte,
            store::CONTROL_RIDER_REQUIRED.default(),
            store::BOARD_BATTERY_FLOOR.default(),
            store::CONTROL_POS_CLAMP.default(),
        );
        assert_eq!(s.block.role_right, right, "side byte {byte}");
    }
}

/// **The turn carry resets with the engagement machine** (section 5): the OFF pass and an arm
/// that changed the producer. A machine that came back engaged mid-turn would deliver the
/// differential as the yaw impulse the slew exists to prevent.
#[test]
fn the_turn_carry_resets_with_the_engagement_machine() {
    let tilted = gently_tilted_sample();
    let mut l = turning_leader(true, control::DriveTurn::new(3_000, 24));
    run_turning(&mut l, 2_000, 32_767, &tilted);
    assert_eq!(l.ctl.turn, 3_000);

    // The OFF pass. The reset runs before the dispatch, so with the stick still hard over the
    // carry reads exactly one slew step: zeroed, then stepped once by the pass that follows it,
    // and held there for as long as the board stays in OFF.
    release_power(&mut l);
    run_with_peer(&mut l, 10, &follower_cyclic(), Some(&tilted));
    assert_eq!(l.mode.mode(), Mode::Off);
    assert_eq!(l.ctl.turn, 24);

    // The arm that changes the producer: nothing has stepped it afterwards, so it is zero.
    re_apply_control_values(
        &mut l,
        store::CONTROL_MODE.default(),
        store::CONTROL_FRAME.default(),
        1,
        0,
        store::BOARD_BATTERY_FLOOR.default(),
        store::CONTROL_POS_CLAMP.default(),
    );
    assert_eq!(l.obs().control_mode, control::Producer::Throttle as u8);
    assert_eq!(l.ctl.turn, 0);
}

// --- The run-time tilt cutout (`specs/control.md` (l)) ----------------------------------------
//
// The rover fell while armed on 2026-10-23 and went on driving on its side
// (`specs/bench-evidence/2026-10-23/rover-rigid/RECORD.md`). The upright window gates ENGAGE only,
// and the one run-time orientation gate the image carried, the pickup path, trips at an inverted
// deck. These drive the cutout the way the firmware does: IMU samples in, a disengaged and
// disarmed board out.

/// One tick of a tip-over at `rate` degrees per second: the gravity vector at the angle the frame
/// has reached AND the gyro rate taking it there, which is what a real fall presents to the
/// filter. The accel channel alone is the filter's slow one (seconds to converge from level);
/// nothing falls over without a rate.
///
/// A positive `deg` is a positive fused pitch here: the accel x-tilt is negated against the
/// `pitched_sample` convention and the body-Y rate is positive, which is the same rotation.
fn fall_tick(deg: f64, rate_dps: f64) -> imu::Sample {
    let (sn, cs) = (deg.to_radians().sin(), deg.to_radians().cos());
    imu::Sample {
        gyro: [Fix::ZERO, Fix::from_num(rate_dps.to_radians()), Fix::ZERO],
        gyro_raw: [0; 3],
        accel_raw: [(-8192.0 * sn) as i16, 0, (8192.0 * cs) as i16],
        temp_centi_degc: 2500,
        still: false,
    }
}

/// A gravity vector `deg` degrees about the PITCH axis, with no rate: a frame already lying at
/// that angle rather than one falling to it. Used where the test wants a settled attitude, which
/// takes the accel-only channel a few seconds to reach.
fn pitched_sample(deg: f64) -> imu::Sample {
    let (sn, cs) = (deg.to_radians().sin(), deg.to_radians().cos());
    imu::Sample {
        accel_raw: [(8192.0 * sn) as i16, 0, (8192.0 * cs) as i16],
        ..level_sample()
    }
}

/// The same about the ROLL axis: a machine lying on its side rather than on its nose.
fn rolled_sample(deg: f64) -> imu::Sample {
    let (sn, cs) = (deg.to_radians().sin(), deg.to_radians().cos());
    imu::Sample {
        accel_raw: [0, (8192.0 * sn) as i16, (8192.0 * cs) as i16],
        ..level_sample()
    }
}

/// Ticks for the accel-only channel to settle on a held angle (8 s at 250 Hz). A fall does not
/// need them ([`fall_tick`]); a frame set down at an angle does.
const SETTLE_TICKS: usize = 2_000;

/// A balance board engaged on a level sample: walked to RUN, then held level until the up-axis
/// row crosses the gate and the engagement machine is out of IDLE.
fn engaged_balance_board() -> (OrchestratorState, imu::Sample) {
    let level = level_sample_at(8192);
    let mut s = balance_to_run(&level);
    for _ in 0..200 {
        control_task(&mut s, Some(&level), 1);
    }
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "engaged first");
    assert!(!s.tilt_latch, "level is not a fall");
    (s, level)
}

/// Fall `s` over at `rate` degrees per second from upright, returning the angle the cutout
/// latched at. Panics if it never does.
fn fall_until_cutout(s: &mut OrchestratorState, rate: f64) -> f64 {
    let mut deg = 0.0f64;
    while deg <= 90.0 {
        control_task(s, Some(&fall_tick(deg, rate)), 1);
        if s.tilt_latch {
            return deg;
        }
        deg += rate / 250.0;
    }
    panic!("the cutout never fired, down to a frame on its face");
}

/// **A machine that falls past the limit while armed disengages and disarms** (section (l)). The
/// whole path is producer driven: the samples feed the attitude step, the attitude words feed the
/// cutout, the cutout's latch feeds both the engagement machine and Fault A.
///
/// It fires at the angle, not near it: with the gyro live (which is what a fall presents) the
/// filter tracks the tip exactly, so the latch lands inside one tick of 45 degrees at every rate
/// from a slow topple to a fast one.
#[test]
fn a_fall_past_the_limit_disengages_and_disarms_the_board() {
    for rate in [45.0f64, 90.0, 180.0] {
        let (mut s, _level) = engaged_balance_board();
        let at = fall_until_cutout(&mut s, rate);
        assert!(
            (44.5..45.5).contains(&at),
            "{rate} deg/s: latched at {at} degrees, not at the limit"
        );
        assert_eq!(s.ctl.fsm.sub_state as u8, 0, "the machine disengaged");
        assert_ne!(
            s.own_levels & events::EV_STOP_ALL,
            0,
            "and says so on the wire"
        );
        // The firing tick still emits the enveloped mirror once (the binary's output stage; the
        // pickup path behaves the same), and the mode machine is one tick into RUN -> SHUTDOWN
        // -> OFF. One more tick settles both.
        assert_eq!(s.mode.mode(), Mode::Shutdown);
        control_task(&mut s, Some(&fall_tick(at, rate)), 1);
        assert_eq!(s.mode.mode(), Mode::Off, "{rate} deg/s: disarmed");
        assert!(!s.mode.any_moe_allowed(), "the bridge is disarmed");
        assert_eq!(s.ctl.wheel_demand, 0, "and commands its wheel nothing");
        assert_eq!(s.obs().torque_setpoint, 0);
    }
}

/// **A fall about the ROLL axis is a fall** (section (l)): a rigid machine on its side is as
/// fallen as one on its nose, and a two-wheel machine has no roll authority at all.
#[test]
fn a_sideways_fall_trips_the_cutout_too() {
    let (mut s, _level) = engaged_balance_board();
    let onside = rolled_sample(60.0);
    for _ in 0..SETTLE_TICKS {
        control_task(&mut s, Some(&onside), 1);
    }
    assert!(
        s.block.roll_word.unsigned_abs() as i32 > control::config::fsm::TILT_LIMIT,
        "roll {}",
        s.block.roll_word
    );
    assert!(
        s.block.pitch_word.unsigned_abs() as i32 <= control::config::fsm::TILT_LIMIT,
        "pitch alone would not have caught it: {}",
        s.block.pitch_word
    );
    assert!(s.tilt_latch);
    assert_eq!(s.mode.mode(), Mode::Off);
}

/// **A lean inside the limit is not a fall** (section (l)): the cutout is a wider number than the
/// engage window on purpose, so a machine past the angle it may START at keeps balancing.
#[test]
fn a_lean_inside_the_limit_keeps_the_machine_running() {
    let (mut s, _level) = engaged_balance_board();
    let leaning = pitched_sample(30.0);
    for _ in 0..SETTLE_TICKS {
        control_task(&mut s, Some(&leaning), 1);
    }
    let pitch = s.block.pitch_word.unsigned_abs() as i32;
    assert!(
        pitch > control::config::fsm::UPRIGHT_LIMIT,
        "past the ENGAGE window: {pitch}"
    );
    assert!(
        pitch <= control::config::fsm::TILT_LIMIT,
        "inside the RUN cutout: {pitch}"
    );
    assert!(!s.tilt_latch, "not a fall");
    assert_eq!(s.mode.mode(), Mode::Run, "still armed");
    assert_ne!(s.ctl.fsm.sub_state as u8, 0, "still engaged");
}

/// **The latch holds while the power request does, and releases when it drops** (section (l)).
/// The hazard it exists for is a machine that rights itself and drives off unbidden: the mode
/// machine's request is a level re-seeded every tick, so an OFF-pass release would re-arm the
/// board three ticks after it settled back under the limit.
#[test]
fn the_cutout_latches_until_the_power_request_drops() {
    let (mut s, level) = engaged_balance_board();
    let at = fall_until_cutout(&mut s, 90.0);
    assert!(s.tilt_latch, "latched at {at} degrees");

    // Righted, with the request still held: the board stays OFF and stays refusing.
    for _ in 0..SETTLE_TICKS {
        control_task(&mut s, Some(&level), 1);
    }
    assert!(
        s.block.pitch_word.unsigned_abs() as i32 <= control::config::fsm::TILT_LIMIT,
        "upright again: {}",
        s.block.pitch_word
    );
    assert!(s.tilt_latch, "the latch does not release on the OFF pass");
    assert_eq!(s.mode.mode(), Mode::Off);
    assert!(!s.mode.any_moe_allowed());
    assert_eq!(s.ctl.fsm.sub_state as u8, 0);

    // The deliberate re-arm: the request drops (the end-of-engage seam), then returns.
    release_power(&mut s);
    let out = control_task(&mut s, Some(&level), 1);
    assert!(out.off_request_clear);
    assert!(!s.tilt_latch, "released at the end of the engage");
    hold_power(&mut s);
    run_ticks(&mut s, 3);
    assert_eq!(s.mode.mode(), Mode::Run, "and it arms again");
}

/// **A machine still lying down cannot be armed** (section (l)): the latch is not sticky beyond
/// its own condition, so releasing the request re-arms nothing while the attitude still says the
/// machine is over.
#[test]
fn a_fallen_machine_cannot_be_armed_while_it_is_fallen() {
    let (mut s, _level) = engaged_balance_board();
    let flat = pitched_sample(70.0);
    for _ in 0..SETTLE_TICKS {
        control_task(&mut s, Some(&flat), 1);
    }
    release_power(&mut s);
    control_task(&mut s, Some(&flat), 1);
    hold_power(&mut s);
    for _ in 0..20 {
        control_task(&mut s, Some(&flat), 1);
    }
    assert!(s.tilt_latch, "re-asserted by the attitude, not held over");
    assert_eq!(s.mode.mode(), Mode::Off, "Fault A blocks OFF -> INIT");
    assert!(!s.mode.any_moe_allowed());
}

/// **A board with no configured IMU has no tilt level** (section (l)): the block's attitude words
/// mean nothing there, so a rigid follower or a padless throttle board is never cut by numbers it
/// never produced. It stops for the reasons that reach it over the wire.
#[test]
fn a_board_without_an_imu_never_trips_the_cutout() {
    let mut s = rigid_board(false);
    assert!(!s.imu_configured);
    // Words the producer would trip on, written into the rows directly: no sample can put them
    // there on this board, which is the point.
    s.block.pitch_word = 9_000;
    s.block.roll_word = -12_000;
    hold_power(&mut s);
    run_ticks(&mut s, 20);
    assert!(!s.tilt_latch, "no IMU, no tilt level");
    assert_eq!(s.own_levels & events::EV_STOP_ALL, 0);
}

/// **A leader that falls puts the bit on the wire** (section (l)): the rigid half of the cutout,
/// measured on the leader itself. Its machine disengages, so the word it commands its peer is
/// zero, and its own level mask carries `EV_STOP_ALL` for the follower to act on.
#[test]
fn a_fallen_leader_publishes_the_stop_bit_in_its_cyclic() {
    let level = level_sample_at(8192);
    let mut l = rigid_board(true);
    hold_power(&mut l);
    run_with_peer(&mut l, 200, &follower_cyclic(), Some(&level));
    assert_ne!(l.ctl.fsm.sub_state as u8, 0, "the leader engaged");
    let healthy = next_cyclic(&mut l, Some(&level));
    assert_eq!(healthy.fault & events::EV_STOP_ALL, 0, "nothing wrong yet");

    let mut deg = 0.0f64;
    while !l.tilt_latch && deg <= 90.0 {
        l.inbox.accept(OTHER, follower_cyclic());
        control_task(&mut l, Some(&fall_tick(deg, 90.0)), 1);
        deg += 90.0 / 250.0;
    }
    assert!(l.tilt_latch, "the leader went over");
    l.inbox.accept(OTHER, follower_cyclic());
    let c = next_cyclic(&mut l, Some(&fall_tick(deg, 90.0)));
    assert_ne!(c.fault & events::EV_STOP_ALL, 0, "the fall is on the wire");
    assert_eq!(c.peer_demand, 0, "and it commands its peer nothing");
}

/// **The rigid FOLLOWER stops with its leader** (section (l) and `specs/rover-one-imu.md`
/// section 6). The follower has no IMU and takes its demand off the wire, so the leader's fall
/// reaches it as the `EV_STOP_ALL` bit in the leader's own level mask, which is what `peer_down`
/// is derived from. No second mechanism: this is the one-wheel-fault interlock doing its job, and
/// the reason it is wanted here is the reason it exists, that one wheel driving while the other
/// stops is worse than both stopping.
#[test]
fn a_fallen_leader_stops_its_follower_through_the_peer_mask() {
    let mut f = rigid_board(false);
    hold_power(&mut f);
    // 200 ticks: the follower's own soft-start envelope ramps 200 counts a tick, so the leader's
    // word only arrives whole once it has reached the cap.
    run_with_peer(&mut f, 200, &leader_cyclic(Mode::Run, 5_000), None);
    assert_eq!(f.mode.mode(), Mode::Run, "armed behind a healthy leader");
    assert_ne!(f.ctl.fsm.sub_state as u8, 0, "engaged on the leader's gate");
    assert_eq!(f.ctl.wheel_demand, 5_000, "applying the leader's word");

    let fallen_leader = Payload::CyclicState(CyclicState {
        pitch: 5_000,
        fault: events::EV_STOP_ALL,
        peer_demand: 0,
        wheel_pos: 0,
        ..match leader_cyclic(Mode::Run, 0) {
            Payload::CyclicState(c) => c,
            _ => unreachable!(),
        }
    });
    run_with_peer(&mut f, 10, &fallen_leader, None);
    assert!(
        dispatch::peer_down_level(&f),
        "the interlock read the leader's byte"
    );
    assert_eq!(f.ctl.wheel_demand, 0, "the follower's wheel is stopped");
    assert_eq!(f.mode.mode(), Mode::Off, "and the follower is disarmed");
    assert!(!f.mode.any_moe_allowed());
}

// --- The wheel positions (`specs/state-feedback.md`, slice 1: the measurement) ----------------

/// A peer frame carrying `wheel_pos`, everything else as [`cyclic`]'s.
fn cyclic_at(wheel_pos: i16) -> Payload {
    Payload::CyclicState(CyclicState {
        pitch: 10,
        roll: -10,
        wheel_speed: 0,
        battery: 3000,
        mode: 3,
        fault: 0,
        flags: 0,
        peer_demand: 0,
        wheel_pos,
        obs: PEER_OBS,
        wheel_rate: 0,
    })
}

/// The cyclic carries the LOW 16 BITS of the block's `wheel_pos`, unmirrored, whichever side the
/// board drives: the receiver knows the sender's side and mirrors it.
#[test]
fn the_cyclic_carries_the_low_half_of_the_position_unmirrored() {
    for role_right in [false, true] {
        for pos in [0i32, 5, -5, 70_000, -70_000, i32::MAX, i32::MIN] {
            let mut a = fresh();
            a.block.role_right = role_right;
            a.block.wheel_pos = pos;
            let c = cyclic_tx(&a, true).expect("tick 0 emits");
            assert_eq!(c.wheel_pos, pos as i16, "pos {pos}, right {role_right}");
            let mut wire = [0u8; CyclicState::ENCODED_LEN];
            c.encode(&mut wire);
            assert_eq!(
                i16::from_le_bytes([wire[14], wire[15]]),
                pos as i16,
                "at offset 14"
            );
        }
    }
}

/// The cyclic carries the block's `wheel_rate_q12` whole and unscaled, unmirrored whichever side
/// the board drives, as the `i32` at offset 28 (`specs/link-control.md`, `CYCLIC_STATE`), and a
/// peer that accepts the frame holds the same word.
#[test]
fn the_cyclic_carries_the_wheel_rate_unscaled_and_the_peer_decodes_it() {
    for role_right in [false, true] {
        for rate in [0i32, 4096, -4096, 80 * 4096, -1, i32::MAX, i32::MIN] {
            let mut a = fresh();
            a.block.role_right = role_right;
            a.block.wheel_rate_q12 = rate;
            let c = cyclic_tx(&a, true).expect("tick 0 emits");
            assert_eq!(c.wheel_rate, rate, "rate {rate}, right {role_right}");
            let mut wire = [0u8; CyclicState::ENCODED_LEN];
            c.encode(&mut wire);
            assert_eq!(
                i32::from_le_bytes([wire[28], wire[29], wire[30], wire[31]]),
                rate,
                "at offset 28"
            );
            let mut b = fresh();
            b.inbox.accept(PEER, Payload::CyclicState(c));
            assert_eq!(b.inbox.peer().map(|p| p.wheel_rate), Some(rate));
        }
    }
}

/// The peer word is UNWRAPPED by `i16` delta per accepted frame: a peer counting forward through
/// the 16-bit wrap reads as a continuous `i32`, and so does one counting back through it.
#[test]
fn the_peer_position_is_unwrapped_across_the_i16_wrap() {
    let mut s = fresh();
    assert_eq!(s.inbox.peer_wheel_pos(), None, "no peer yet");
    assert_eq!(s.obs().wheel_pos_peer, 0, "and the observation reads 0");

    // Seeded from the first frame's word, then three edges per frame up through 32767.
    let mut word: i16 = 32_700;
    s.inbox.accept(PEER, cyclic_at(word));
    assert_eq!(s.inbox.peer_wheel_pos(), Some(32_700));
    let mut expect: i32 = 32_700;
    for _ in 0..100 {
        word = word.wrapping_add(3);
        expect += 3;
        s.inbox.accept(PEER, cyclic_at(word));
        assert_eq!(s.inbox.peer_wheel_pos(), Some(expect), "word {word}");
    }
    assert!(expect > i16::MAX as i32, "the run crossed the wrap");
    assert!(word < 0, "and the wire word did wrap");

    // And back down through it, and through the negative wrap too.
    for _ in 0..30_000 {
        word = word.wrapping_sub(4);
        expect -= 4;
        s.inbox.accept(PEER, cyclic_at(word));
    }
    assert_eq!(s.inbox.peer_wheel_pos(), Some(expect));
    assert!(
        expect < i16::MIN as i32,
        "below the negative wrap ({expect})"
    );
}

/// A frame that repeats the previous word adds nothing, and a tick with no frame adds nothing:
/// the position moves only with a delta the peer stated.
#[test]
fn the_peer_position_moves_only_with_a_new_word() {
    let mut s = fresh();
    s.inbox.accept(PEER, cyclic_at(-40));
    s.inbox.accept(PEER, cyclic_at(-40));
    let _ = run_ticks(&mut s, 10);
    assert_eq!(s.inbox.peer_wheel_pos(), Some(-40));
    s.inbox.accept(PEER, cyclic_at(-38));
    assert_eq!(s.inbox.peer_wheel_pos(), Some(-38));
}

/// The observation publishes both positions FORWARD POSITIVE: the own word through this board's
/// side mirror and the peer's through the OPPOSITE side's, since the two wheels of one machine are
/// mirror images (`specs/state-feedback.md` section 3). On a right-wheel board a forward roll
/// counts its own hall counter DOWN and the left peer's UP, and both observation words read the
/// same positive travel; on a left-wheel board the senses swap and the words still agree.
#[test]
fn the_observation_mirrors_own_by_its_side_and_the_peer_by_the_other() {
    for role_right in [true, false] {
        let mut s = fresh();
        s.block.role_right = role_right;
        // 170 edges of forward travel on each wheel, in each counter's own sense.
        let (own_raw, peer_raw) = if role_right { (-170, 170) } else { (170, -170) };
        s.inbox.accept(PEER, cyclic_at(0));
        s.block.wheel_pos = own_raw;
        s.inbox.accept(PEER, cyclic_at(peer_raw));
        let o = s.obs();
        assert_eq!(o.wheel_pos_own, 170, "own, right {role_right}");
        assert_eq!(o.wheel_pos_peer, 170, "peer, right {role_right}");
    }
}

/// Live in every mode and every sub-state: the board here never leaves OFF and both words still
/// track, which is what makes the sign gate a disarmed hand push.
#[test]
fn the_positions_are_published_while_disarmed() {
    let mut s = fresh();
    s.block.role_right = true;
    s.inbox.accept(PEER, cyclic_at(0));
    for k in 1..=50i32 {
        s.block.wheel_pos = -k;
        s.inbox.accept(PEER, cyclic_at(k as i16));
        let _ = control_task(&mut s, None, 1);
        assert_eq!(s.mode.mode(), Mode::Off);
        let o = s.obs();
        assert_eq!((o.wheel_pos_own, o.wheel_pos_peer), (k, k));
    }
}

// --- The state-feedback law at the assembly (`specs/state-feedback.md`, slice 3) --------------

/// The start gains with the stick live: `speed_max` 80 (1.4 m/s at full stick), `speed_slew` 41.
fn sf_driving() -> control::GainShadow {
    control::GainShadow::of_stored([3491, 4002, 197, 926, 80, 41], control::DEFAULT_GAIN_MAX)
}

/// A pivoting balance board on side byte `side` with `gains`, pads down and power held, walked on
/// a level 1 g sample until the engagement machine is in RUN.
fn sf_run_board(side: u8, gains: control::GainShadow) -> OrchestratorState {
    let mut s = OrchestratorState::new(
        1,
        0,
        side,
        true,
        true,
        attitude::Config::default(),
        gains,
        sensing(),
        store::CONTROL_POS_CLAMP.default(),
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    s.vbatt_raw = BENCH_CV;
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    let level = level_sample_at(8192);
    // The soft start: +200 a tick from 0 to 28500 is 143 ticks of ARMING before RUN.
    for _ in 0..300 {
        control_task(&mut s, Some(&level), 1);
    }
    assert_eq!(s.ctl.fsm.sub_state, control::SubState::Run);
    s
}

/// **Defaults are the inert state** (section 6): a balance board on the registered defaults
/// computes four zero terms and feeds the engagement machine a zero word, tilted or not.
#[test]
fn an_unstaged_balance_board_commands_nothing() {
    let mut s = balance_board(control::GainShadow::default());
    for _ in 0..50 {
        control_task(&mut s, Some(&tilted_sample()), 1);
    }
    assert_ne!(s.block.pitch_word, 0, "the tilt reached the law's input");
    assert_eq!(s.ctl.sf.terms, [0; 4]);
    assert_eq!(s.obs().pre_env_torque, 0);
    // The same board on the start gains commands torque from the same tilt.
    let mut t = balance_board(sf_start());
    for _ in 0..50 {
        control_task(&mut t, Some(&tilted_sample()), 1);
    }
    assert_ne!(t.obs().pre_env_torque, 0);
}

/// **The reference cells are zero on every tick the machine is not in RUN** (section 4, the
/// recovered integrator's reset rule): through the walk to RUN, out through a disarm, and through
/// the tilt latch, with the stick held at full forward the whole time.
#[test]
fn the_reference_cells_are_zero_on_every_tick_outside_run() {
    let level = level_sample_at(8192);
    let mut s = balance_board(sf_driving());
    input_task(&mut s, &pads_on_button_held());
    input_task(&mut s, &pads_on_button_held());
    let mut ran = 0;
    for k in 0..400 {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&level), 1);
        if s.ctl.fsm.sub_state == control::SubState::Run {
            ran += 1;
        } else {
            assert_eq!((s.ctl.sf.v_ref, s.ctl.sf.pos_err), (0, 0), "tick {k}");
        }
    }
    assert!(ran > 100, "the walk reached RUN");
    assert!(s.ctl.sf.v_ref > 0, "RUN: the stick winds the reference");
    assert_ne!(s.ctl.sf.pos_err, 0);

    // The disarm: the reference cells are zero from the first tick that is not RUN.
    release_power(&mut s);
    for _ in 0..10 {
        feed_drive(&mut s, i16::MAX, 0);
        control_task(&mut s, Some(&level), 1);
        assert_eq!((s.ctl.sf.v_ref, s.ctl.sf.pos_err), (0, 0));
    }
    assert_eq!(s.mode.mode(), Mode::Off);

    // A fall: back to RUN, then over on its nose past the cutout. Zero from the tick it latches.
    let mut s = sf_run_board(0, sf_driving());
    for k in 0..100 {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&level), 1);
    }
    assert!(s.ctl.sf.v_ref > 0);
    let over = pitched_sample(60.0);
    for k in 0..SETTLE_TICKS {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&over), 1);
        if s.tilt_latch {
            assert_eq!(
                (s.ctl.sf.v_ref, s.ctl.sf.pos_err),
                (0, 0),
                "zero from the tick it latched (k = {k})"
            );
        }
    }
    assert!(s.tilt_latch, "the cutout fired");
}

/// **A stale `DRIVE_CMD` is target speed 0, so the slew is the stop ramp** (section 4): the
/// reference ramps down by exactly `speed_slew` per tick once the 200 ms timeout passes, and the
/// position reference then stops where the ramp ended and holds.
#[test]
fn a_stale_drive_ramps_the_reference_to_zero_at_speed_slew_and_the_position_holds() {
    let level = level_sample_at(8192);
    let mut s = sf_run_board(0, sf_driving());
    // Full stick until the reference reaches its target: 32767 * 80 / 128.
    let target = 32_767 * 80 / 128;
    for k in 0..600 {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&level), 1);
    }
    assert_eq!(s.ctl.sf.v_ref, target);
    // The stream stops. The reference holds while the last command is fresh, then steps down.
    while !s.inbox.drive_stale() {
        control_task(&mut s, Some(&level), 1);
        if !s.inbox.drive_stale() {
            assert_eq!(s.ctl.sf.v_ref, target);
        }
    }
    let mut ticks = 1;
    while s.ctl.sf.v_ref != 0 {
        let prev = s.ctl.sf.v_ref;
        control_task(&mut s, Some(&level), 1);
        assert_eq!(
            prev - s.ctl.sf.v_ref,
            41.min(prev),
            "exactly speed_slew per tick"
        );
        ticks += 1;
    }
    assert_eq!(
        ticks,
        (target + 40) / 41,
        "target / speed_slew ticks to rest"
    );
    assert_eq!(s.ctl.fsm.sub_state, control::SubState::Run, "still holding");
    let held = s.ctl.sf.pos_err;
    for _ in 0..50 {
        control_task(&mut s, Some(&level), 1);
        assert_eq!(s.ctl.sf.pos_err, held, "the position reference holds");
    }
}

/// **The fold** (section 3): the peer's word is unwrapped across its `i16` wrap, goes through the
/// OPPOSITE side's mirror, and moves the position error only on a tick that accepted a frame
/// carrying a new word. Both sides: on a LEFT board the peer is the right wheel (forward reads
/// negative), on a RIGHT board the left (forward reads positive).
#[test]
fn the_peer_word_is_unwrapped_mirrored_by_the_opposite_side_and_folded_only_on_a_new_frame() {
    let level = level_sample_at(8192);
    for side in [0u8, 1] {
        let mut s = balance_board(sf_start());
        s.block.role_right = side == 1;
        // The peer is heard before RUN, as a leader's must be: its first word seeds the unwrap.
        let mut word: i16 = 32_760;
        s.inbox.accept(PEER, cyclic_at(word));
        input_task(&mut s, &pads_on_button_held());
        input_task(&mut s, &pads_on_button_held());
        for k in 0..300 {
            if k % 10 == 0 {
                s.inbox.accept(PEER, cyclic_at(word));
            }
            control_task(&mut s, Some(&level), 1);
        }
        assert_eq!(s.ctl.fsm.sub_state, control::SubState::Run, "side {side}");
        let before = s.ctl.sf.pos_err;
        // Ten edges in the peer's own counter, through its 16-bit wrap.
        word = word.wrapping_add(10);
        assert!(word < 0, "the frame crosses the wrap");
        s.inbox.accept(PEER, cyclic_at(word));
        control_task(&mut s, Some(&level), 1);
        // The peer's forward is its word through the OPPOSITE side: dX2 is +-10 half-edges, and
        // the error is `x_ref - x`, so it moves by -dX2 * 10240.
        let peer_fwd: i32 = if side == 1 { 10 } else { -10 };
        assert_eq!(
            s.ctl.sf.pos_err - before,
            -peer_fwd * 10_240,
            "side {side}: one fold of the unwrapped, mirrored delta"
        );
        // Quiet ticks, and a frame repeating the same word, fold nothing.
        let after = s.ctl.sf.pos_err;
        for k in 0..8 {
            if k == 4 {
                s.inbox.accept(PEER, cyclic_at(word));
            }
            control_task(&mut s, Some(&level), 1);
            assert_eq!(s.ctl.sf.pos_err, after, "side {side}, tick {k}");
        }
    }
}

/// The OWN wheel folds every tick through this board's side: a forward step on the right wheel
/// counts DOWN, on the left UP, and either moves the position error the same way.
#[test]
fn the_own_wheel_folds_through_its_side() {
    let level = level_sample_at(8192);
    for side in [0u8, 1] {
        let mut s = sf_run_board(side, sf_start());
        let before = s.ctl.sf.pos_err;
        // Four edges forward.
        s.block.wheel_pos += if side == 1 { -4 } else { 4 };
        control_task(&mut s, Some(&level), 1);
        assert_eq!(s.ctl.sf.pos_err - before, -4 * 10_240, "side {side}");
        assert_eq!(s.obs().e_x, (s.ctl.sf.pos_err / 20_480) as i16);
    }
}

/// **The follower runs no part of the law** (section 5): a rigid follower with a rolling wheel and
/// a full stick leaves the law's record untouched, and still publishes its position.
#[test]
fn a_rigid_follower_runs_no_law_and_publishes_its_position() {
    let mut s = rigid_board(false);
    assert_eq!(s.obs().control_mode, control::Producer::Follower as u8);
    walk_to_run(&mut s);
    s.block.wheel_rate_q12 = -80 * 4096;
    s.block.wheel_pos = 1_234;
    feed_drive(&mut s, i16::MAX, 0);
    run_with_peer(&mut s, 100, &leader_cyclic(Mode::Run, 1000), None);
    assert_eq!(s.ctl.sf, control::SfState::default());
    assert_eq!(s.obs().sf_terms, [0; 4]);
    let c = next_cyclic(&mut s, None);
    assert_eq!(c.wheel_pos, 1_234);
}

/// **The observation words carry the terms** (section 8), disarmed: the four terms, the reference
/// and the position error as the law left them, the terms live with the machine disarmed.
#[test]
fn the_observation_carries_the_terms_and_the_reference() {
    let mut s = balance_board(sf_start());
    s.block.wheel_rate_q12 = 4096;
    for _ in 0..50 {
        control_task(&mut s, Some(&tilted_sample()), 1);
    }
    let o = s.obs();
    assert_eq!(o.sf_terms, s.ctl.sf.terms);
    assert_ne!(o.sf_terms[0], 0, "term_p is live disarmed");
    assert_ne!(
        o.sf_terms[3], 0,
        "term_v is live disarmed: a rolling wheel is a speed error"
    );
    assert_eq!((o.v_ref_q, o.e_x), (0, 0), "the reference is RUN-only");

    // In RUN with the stick: the reference and the position error as the law carries them.
    let level = level_sample_at(8192);
    let mut s = sf_run_board(0, sf_driving());
    for k in 0..30 {
        if k % 20 == 0 {
            feed_drive(&mut s, i16::MAX, 0);
        }
        control_task(&mut s, Some(&level), 1);
    }
    let o = s.obs();
    assert_eq!(o.v_ref_q as i32, s.ctl.sf.v_ref);
    assert!(o.v_ref_q > 0);
    assert_eq!(o.e_x as i32, s.ctl.sf.pos_err / 20_480);
}

/// **A live tune reaches the law** (section 6): the lane's write moves the shadow, and the next
/// pass's terms are at the new gain while the machine is disarmed (the live gains are the
/// shadow's outside RUN). The lane takes exactly 0x79 indices 0..3 and 0x7B indices 0..1.
#[test]
fn a_live_tune_reaches_the_terms_and_the_lane_takes_exactly_its_keys() {
    let mut s = balance_board(sf_start());
    for _ in 0..30 {
        control_task(&mut s, Some(&tilted_sample()), 1);
    }
    let term_p = |s: &OrchestratorState, gp: i64| {
        control::sf::word(gp * s.block.pitch_word as i64 / 100, s.block.battery)
    };
    assert_eq!(s.ctl.sf.terms[0], term_p(&s, 3491));
    assert_eq!(s.ctl.gains.set(0x79, 0, 3491 * 2), Ok(()));
    control_task(&mut s, Some(&tilted_sample()), 1);
    assert_eq!(s.ctl.sf.live[0], 3491 * 2);
    assert_eq!(
        s.ctl.sf.terms[0],
        term_p(&s, 3491 * 2),
        "the next pass's term_p"
    );
    assert_ne!(term_p(&s, 3491 * 2), term_p(&s, 3491));
    for i in 0..4 {
        assert_eq!(s.ctl.gains.set(0x79, i, 1), Ok(()));
    }
    for i in 0..2 {
        assert_eq!(s.ctl.gains.set(0x7B, i, 1), Ok(()));
    }
    for (id, i, v) in [(0x7B, 2, 1), (0x71, 0, 1), (0x79, 0, i16::MAX)] {
        assert!(s.ctl.gains.set(id, i, v).is_err(), "{id:#x}/{i} = {v}");
    }
}

/// **The position clamp's seam, at boot and at every arm** (section 6): the stored word clamped
/// into `0..=450`, the `side_byte` pattern.
#[test]
fn the_position_clamp_goes_through_its_seam_at_boot_and_at_the_arm() {
    let s = OrchestratorState::new(
        1,
        0,
        0,
        true,
        true,
        attitude::Config::default(),
        sf_start(),
        sensing(),
        900,
        control::DriveTurn::default(),
        store::CONTROL_RIDER_REQUIRED.default(),
        store::BOARD_BATTERY_FLOOR.default(),
        TEST_IDENTITY,
    );
    assert_eq!(s.ctl.pos_clamp, control::sf::POS_CLAMP_MAX);
    let mut s = s;
    for (stored, want) in [(-5i16, 0i16), (45, 45), (450, 450), (i16::MAX, 450)] {
        re_apply_control_values(
            &mut s,
            1,
            0,
            0,
            store::CONTROL_RIDER_REQUIRED.default(),
            store::BOARD_BATTERY_FLOOR.default(),
            stored,
        );
        assert_eq!(s.ctl.pos_clamp, want, "stored {stored}");
    }
}
