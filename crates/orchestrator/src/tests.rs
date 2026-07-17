//! Host vectors for the orchestrator core (`specs/integration.md` "Validation", the slice-5
//! set): boot-to-OFF, the power-request walk to INIT/READY/RUN with the MOE decisions observed,
//! the `comms_loss` trip into `fault_a` and SHUTDOWN, the `stop_all` latch + OFF-dwell clear,
//! plus the supervision ages (`specs/link-control.md`: the 25-tick trip, fresh-frame clear, the
//! never-seen-a-peer exemption, drive staleness at 50 ticks) and the input-task assembly
//! (`power_request` = button OR mirror, rider from both pads, the mirror-gated throttle filter).

use super::*;
use linkctl::{DriveKind, Fault, Inputs};
use state::Mode;

fn fresh() -> OrchestratorState {
    OrchestratorState::new(false, attitude::Config::default())
}

/// Run `n` control ticks with no IMU sample, returning the last output.
fn run_ticks(state: &mut OrchestratorState, n: usize) -> ControlOutput {
    let mut last = control_task(state, None);
    for _ in 1..n {
        last = control_task(state, None);
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
    let t1 = control_task(&mut s, None);
    assert_eq!(t1.mode_byte, Mode::Init.as_byte());
    assert_eq!(t1.init, None);
    assert_eq!(t1.moe, [false; N_MOTORS], "MOE not yet set");

    // Tick 2: the INIT pass: MOE set, the bring-up record leaves through the enact seam.
    let t2 = control_task(&mut s, None);
    assert_eq!(t2.mode_byte, Mode::Ready.as_byte());
    assert_eq!(t2.init, Some(InitAction { run_bringup: true }));
    assert_eq!(t2.moe, [true; N_MOTORS], "MOE decisions set at INIT");
    assert_eq!(s.enact_inits, 1, "the seam recorded the bring-up");

    // Tick 3: READY -> RUN; MOE holds.
    let t3 = control_task(&mut s, None);
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
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.shutdown, Some(ShutdownAction { run_safedown: true }));
    assert_eq!(t.moe, [false; N_MOTORS]);
    assert_eq!(s.enact_shutdowns, 1);
}

#[test]
fn remote_mirror_bit_is_an_equivalent_power_request_producer() {
    // No button at all: the INPUTS mirror's power bit walks the machine the same way (the
    // level-OR assembly).
    let mut s = fresh();
    s.inbox.accept(Payload::Inputs(Inputs {
        throttle: 0,
        buttons: Inputs::BUTTON_POWER,
        rider: 0,
    }));
    assert!(s.power_request());
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // The mirror dropping the bit (latest-wins) releases the request: SHUTDOWN then OFF.
    s.inbox.accept(Payload::Inputs(Inputs {
        throttle: 0,
        buttons: 0,
        rider: 0,
    }));
    let t = run_ticks(&mut s, 2);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
}

// --- comms_loss: the supervision trip into fault_a -------------------------------------------

#[test]
fn comms_loss_trips_at_26_ticks_and_forces_shutdown() {
    let mut s = fresh();
    hold_power(&mut s);
    s.inbox.accept(cyclic(0));
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // The peer goes silent. Ages 4..=25 (22 more ticks): within the window, RUN holds.
    let t = run_ticks(&mut s, 22);
    assert_eq!(s.inbox.cyclic_age(), 25);
    assert!(!t.comms_loss, "age 25 is within the 100 ms window");
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // Age 26: the level asserts, folds into fault_a, RUN -> SHUTDOWN.
    let t = control_task(&mut s, None);
    assert!(t.comms_loss, "age 26 > CYCLIC_TIMEOUT_TICKS trips");
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
    assert!(s.obs().comms_loss);

    // Level-sensitive: a fresh cyclic clears it, and re-entry follows the machine's own gates
    // (the held request walks OFF -> INIT again).
    s.inbox.accept(cyclic(0));
    let t = control_task(&mut s, None);
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

// --- stop_all: latch + OFF-dwell clear -------------------------------------------------------

#[test]
fn stop_all_latches_through_shutdown_and_clears_after_an_off_pass() {
    let mut s = fresh();
    hold_power(&mut s);
    let t = run_ticks(&mut s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());

    // A FAULT with action = STOP_ALL arrives: the latch sets and folds into fault_a.
    s.inbox.accept(Payload::Fault(Fault {
        code: 0x11,
        action: Fault::ACTION_STOP_ALL,
    }));
    assert!(s.inbox.stop_all());

    // RUN -> SHUTDOWN on the latched level; the latch HOLDS through the SHUTDOWN pass.
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    assert!(s.inbox.stop_all(), "latched through SHUTDOWN");

    // The SHUTDOWN pass lands in OFF: the machine has passed through OFF, the latch releases.
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.shutdown, Some(ShutdownAction { run_safedown: true }));
    assert!(!s.inbox.stop_all(), "OFF-dwell clear");

    // Re-entry follows the mode machine's own gates: the still-held request re-INITs.
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Init.as_byte());
}

#[test]
fn stop_all_while_off_forces_one_blocked_pass_then_clears() {
    // The latch arriving while already OFF holds OFF for (at least) one full pass, then the
    // OFF dwell releases it; a held request then enters INIT.
    let mut s = fresh();
    hold_power(&mut s);
    s.inbox.accept(Payload::Fault(Fault {
        code: 0x21,
        action: Fault::ACTION_STOP_ALL,
    }));
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Off.as_byte(), "fault_a holds OFF");
    assert!(!s.inbox.stop_all(), "released after the OFF pass");
    let t = control_task(&mut s, None);
    assert_eq!(t.mode_byte, Mode::Init.as_byte());
}

#[test]
fn notify_only_fault_does_not_latch() {
    let mut s = fresh();
    s.inbox.accept(Payload::Fault(Fault {
        code: 0x11,
        action: Fault::ACTION_NOTIFY,
    }));
    assert!(!s.inbox.stop_all());
    hold_power(&mut s);
    assert_eq!(run_ticks(&mut s, 3).mode_byte, Mode::Run.as_byte());
}

// --- Drive staleness (the age half; the reference-zeroing consumer is slice 6) ---------------

#[test]
fn drive_staleness_at_51_ticks_and_never_received() {
    let mut s = fresh();
    assert!(s.inbox.drive_stale(), "never received = stale");
    assert_eq!(s.inbox.drive(), None);

    s.inbox.accept(Payload::DriveCmd(DriveCmd {
        kind: DriveKind::Throttle,
        value: 500,
        steer: 0,
    }));
    assert!(!s.inbox.drive_stale(), "fresh on receipt");
    run_ticks(&mut s, 50);
    assert_eq!(s.inbox.drive_age(), 50);
    assert!(!s.inbox.drive_stale(), "age 50 is within the 200 ms window");
    run_ticks(&mut s, 1);
    assert!(s.inbox.drive_stale(), "age 51 > DRIVE_TIMEOUT_TICKS");
    // Latest-wins refresh restores freshness.
    s.inbox.accept(Payload::DriveCmd(DriveCmd {
        kind: DriveKind::Neutral,
        value: 0,
        steer: 0,
    }));
    assert!(!s.inbox.drive_stale());
    assert_eq!(s.inbox.drive().unwrap().kind, DriveKind::Neutral);
}

// --- The inbox's slice-6-facing levels -------------------------------------------------------

#[test]
fn peer_lockdown_and_rider_levels_are_latest_wins() {
    let mut s = fresh();
    assert!(!s.inbox.peer_lockdown());
    s.inbox.accept(cyclic(CyclicState::FLAG_LOCKDOWN));
    assert!(s.inbox.peer_lockdown());
    s.inbox.accept(cyclic(CyclicState::FLAG_RIDER));
    assert!(!s.inbox.peer_lockdown(), "level, latest-wins");
    assert!(s.inbox.peer().unwrap().rider_present());

    s.inbox.accept(Payload::Inputs(Inputs {
        throttle: 0,
        buttons: 0,
        rider: Inputs::RIDER_PRESENT,
    }));
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
fn throttle_filter_steps_only_once_a_mirror_word_exists() {
    let mut s = fresh();
    // No INPUTS mirror yet: the filter must not capture a fabricated zero baseline.
    input_task(&mut s, &InputSample::default());
    assert!(!s.throttle.is_initialized());
    assert_eq!(s.throttle_filtered, 0);

    // A mirror word arrives; the next input pass captures it as the one-shot baseline:
    // first output = scaled(word) + 200 (the inputs-crate contract).
    s.inbox.accept(Payload::Inputs(Inputs {
        throttle: 30000,
        buttons: 0,
        rider: 0,
    }));
    input_task(&mut s, &InputSample::default());
    assert!(s.throttle.is_initialized());
    let expect = inputs::scaled_throttle(30000) as i32 + inputs::OUTPUT_BIAS;
    assert_eq!(s.throttle_filtered as i32, expect);
}

// --- IMU liveness and the zero sample --------------------------------------------------------

#[test]
fn imu_live_tracks_read_success_and_zero_sample_still_ticks_attitude() {
    let mut s = OrchestratorState::new(true, attitude::Config::default());

    // A failing read (None) on a configured IMU: not live, no fault, the pipeline still runs.
    let t = control_task(&mut s, None);
    assert!(!s.imu_live);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());

    // A real sample: live, and the attitude output moves off identity (gravity on +Z tilted
    // toward +X pulls pitch away from zero within a few ticks).
    let sample = imu::Sample {
        gyro: [Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [8000, 0, 14000],
        temp_centi_degc: 2500,
        still: false,
    };
    for _ in 0..250 {
        control_task(&mut s, Some(&sample));
    }
    assert!(s.imu_live);
    let obs = s.obs();
    assert!(obs.imu_configured);
    assert!(
        obs.pitch_milli_deg != 0,
        "attitude integrated the tilted gravity vector"
    );

    // The read failing again drops the live flag on that tick.
    control_task(&mut s, None);
    assert!(!s.imu_live);
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
    control_task(&mut s, Some(&sample));
    assert!(!s.imu_live, "a sample without a configured IMU is not live");
}

// --- OBS snapshot ----------------------------------------------------------------------------

#[test]
fn obs_snapshot_carries_the_pipeline_counters_and_levels() {
    let mut s = fresh();
    hold_power(&mut s);
    s.inbox.accept(cyclic(0));
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
