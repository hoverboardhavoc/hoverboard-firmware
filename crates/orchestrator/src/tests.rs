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
    OrchestratorState::new(0, false, attitude::Config::default())
}

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
    let t = control_task(&mut s, None, 1);
    assert!(t.comms_loss, "age 26 > CYCLIC_TIMEOUT_TICKS trips");
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.moe, [false; N_MOTORS]);
    assert!(s.obs().comms_loss);

    // Level-sensitive: a fresh cyclic clears it, and re-entry follows the machine's own gates
    // (the held request walks OFF -> INIT again).
    s.inbox.accept(cyclic(0));
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
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Shutdown.as_byte());
    assert!(s.inbox.stop_all(), "latched through SHUTDOWN");

    // The SHUTDOWN pass lands in OFF: the machine has passed through OFF, the latch releases.
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte());
    assert_eq!(t.shutdown, Some(ShutdownAction { run_safedown: true }));
    assert!(!s.inbox.stop_all(), "OFF-dwell clear");

    // Re-entry follows the mode machine's own gates: the still-held request re-INITs.
    let t = control_task(&mut s, None, 1);
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
    let t = control_task(&mut s, None, 1);
    assert_eq!(t.mode_byte, Mode::Off.as_byte(), "fault_a holds OFF");
    assert!(!s.inbox.stop_all(), "released after the OFF pass");
    let t = control_task(&mut s, None, 1);
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
    let mut s = OrchestratorState::new(0, true, attitude::Config::default());
    hold_power(&mut s);
    let good = good_sample();
    for _ in 0..3 {
        control_task(&mut s, Some(&good), 1);
    }
    assert_eq!(s.mode.mode(), Mode::Run);
    assert!(!s.imu_health.loss());
    s
}

#[test]
fn imu_live_tracks_read_success() {
    let mut s = OrchestratorState::new(0, true, attitude::Config::default());

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
    let mut s = OrchestratorState::new(0, true, attitude::Config::default());
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
    let mut s = OrchestratorState::new(0, true, attitude::Config::default());
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
    let mut s = OrchestratorState::new(0, false, attitude::Config::default());
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

// --- Slice 6: control dispatch + cyclic (`specs/integration.md` steps 7-8) -------------------

use linkctl::{decode as linkctl_decode, Payload as LPayload, OP_CYCLIC_STATE};

/// Hold RUN + an open engagement gate: power walk to RUN with the gating halfword set.
fn walk_to_run_gated(s: &mut OrchestratorState) {
    s.block.gating_field = 1000; // the engage gate (> 500); producer out of scope, block-set
    hold_power(s);
    let t = run_ticks(s, 3);
    assert_eq!(t.mode_byte, Mode::Run.as_byte());
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
    s.inbox.accept(Payload::DriveCmd(DriveCmd {
        kind: DriveKind::Throttle,
        value,
        steer,
    }));
}

#[test]
fn imu_absent_balance_demotes_to_throttle_with_mode_fault() {
    // CONTROL_MODE = 1 (Balance) without a configured IMU: the boot seam demotes with the fault.
    let s = OrchestratorState::new(1, false, attitude::Config::default());
    let obs = s.obs();
    assert_eq!(obs.control_mode, 0, "demoted to Throttle");
    assert!(obs.mode_fault);

    // With the IMU configured, Balance holds and no fault raises.
    let s = OrchestratorState::new(1, true, attitude::Config::default());
    assert_eq!(s.obs().control_mode, 1);
    assert!(!s.obs().mode_fault);

    // The default byte (0) is Throttle on any board, no fault.
    let s = OrchestratorState::new(0, false, attitude::Config::default());
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
    walk_to_run_gated(&mut s);

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
    // walks IDLE -> ARMING -> RUN sub-states, the torque tracks the steer-shaped commanded lean
    // and never exceeds the soft-start envelope (|torque| <= 200/tick * ticks-since-engage).
    let mut s = OrchestratorState::new(1, true, attitude::Config::default());
    let level = level_sample(); // a live, level IMU so the board stays in RUN (no IMU-loss fault)
    walk_to_run_gated(&mut s);
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
    assert_eq!(engaged_at, Some(0), "engaged on the first gated rider tick");
    assert_eq!(s.ctl.fsm.sub_state as u8, 3, "promoted to RUN sub-state");
    assert!(
        s.ctl.fsm.torque_setpoint != 0,
        "the steer-shaped lean drives a nonzero setpoint"
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
fn peer_lockdown_forces_substate_zero_and_zero_torque() {
    // The CYCLIC_STATE lockdown flag reaches its consumer: an engaged throttle board drops to
    // sub-state 0 with a zero setpoint while the peer asserts lockdown.
    let mut s = fresh();
    walk_to_run_gated(&mut s);
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
    s.inbox.accept(cyclic(CyclicState::FLAG_LOCKDOWN));
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
    // HEALTHY predicate (a != 0, b == 0) resets the counter.
    s.block.gating_field = 1000;
    feed_drive(&mut s, 32767, 0);
    run_ticks(&mut s, 2);
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

#[test]
fn mode_switch_is_disarmed_only_and_resets_the_producer_records() {
    let mut s = OrchestratorState::new(0, true, attitude::Config::default());
    hold_power(&mut s);
    run_ticks(&mut s, 3); // RUN: MOE set -> armed
    assert!(s.mode.any_moe_allowed());
    assert!(!switch_control_mode(&mut s, 1), "armed switch refused");
    assert_eq!(s.obs().control_mode, 0, "mode unchanged");

    // Disarm (release the request -> SHUTDOWN -> OFF), dirty a producer record, then switch.
    input_task(&mut s, &InputSample::default());
    run_ticks(&mut s, 2);
    assert!(!s.mode.any_moe_allowed());
    s.ctl.iir.carry = base::fixed::Fix::from_num(123);
    assert!(switch_control_mode(&mut s, 1), "disarmed switch applies");
    assert_eq!(s.obs().control_mode, 1);
    assert_eq!(
        s.ctl.iir.carry,
        base::fixed::Fix::ZERO,
        "producer records replaced wholesale"
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
    assert_eq!(c.battery, 3600); // the placeholder word until the sensing producer
    assert_eq!(c.mode, 0);
    assert_eq!(c.fault, 0);
    assert!(c.rider_present());
    assert!(!c.lockdown());

    // The wire round trip against linkctl: encode, decode by opcode, accept into a peer inbox.
    let mut wire = [0u8; 16];
    let n = c.encode(&mut wire);
    let payload = linkctl_decode(OP_CYCLIC_STATE, &wire[..n]).expect("decodes");
    let mut b = fresh();
    match payload {
        LPayload::CyclicState(pc) => {
            assert_eq!(pc, c, "byte-faithful round trip");
            b.inbox.accept(LPayload::CyclicState(pc));
        }
        other => panic!("wrong family: {other:?}"),
    }
    assert_eq!(b.inbox.peer().unwrap(), c);
}

#[test]
fn peer_rider_flag_reaches_the_engage_gate() {
    // The cyclic rider flag is a consumption-side fold: a board with NO local pads engages the
    // balance machine once its peer reports a rider.
    let mut b = OrchestratorState::new(1, true, attitude::Config::default());
    let level = level_sample(); // a live, level IMU so the board stays in RUN (no IMU-loss fault)
    walk_to_run_gated(&mut b);
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
            b.inbox.accept(cyclic(CyclicState::FLAG_RIDER));
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
    let mut b = OrchestratorState::new(1, true, attitude::Config::default());
    let level = level_sample(); // a live, level IMU so the board stays in RUN (no IMU-loss fault)
    b.block.orientation_nz = true;
    walk_to_run_gated(&mut b);
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
        flags: 0,
    };
    for k in 0..160 {
        if k % 20 == 0 {
            b.inbox.accept(Payload::CyclicState(peer));
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
fn peer_roll_reaches_the_shaper_roll_mirror() {
    // roll_b: with a max steer command the shaped lean clamps to +-base, and base grows with
    // the local-vs-peer roll differential, so the peer's roll word observably changes the
    // torque the balance path produces (all else identical, incl. the battery word).
    let run_board = |peer_roll: Option<i16>| -> i16 {
        let mut b = OrchestratorState::new(1, true, attitude::Config::default());
        let level = level_sample(); // a live, level IMU so the board stays in RUN
        walk_to_run_gated(&mut b);
        input_task(&mut b, &pads_on_button_held());
        for k in 0..60 {
            if k % 20 == 0 {
                feed_drive(&mut b, 0, 32767);
                if let Some(r) = peer_roll {
                    let mut c = cyclic(0);
                    if let Payload::CyclicState(ref mut cs) = c {
                        cs.roll = r;
                        cs.battery = 3600; // match the placeholder: isolate the roll effect
                        cs.wheel_speed = 0;
                    }
                    b.inbox.accept(c);
                }
            }
            control_task(&mut b, Some(&level), 1);
        }
        b.ctl.fsm.torque_setpoint
    };

    let without_peer = run_board(None);
    let with_peer_roll = run_board(Some(800));
    assert!(without_peer != 0);
    assert!(with_peer_roll != 0);
    assert!(
        with_peer_roll != without_peer,
        "the peer roll mirror changes the shaped lean: {without_peer} vs {with_peer_roll}"
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

#[test]
fn cyclic_tx_rider_flag_is_local_only() {
    // The negative half of the rider fold: a board whose rider level is folded-only (the peer
    // cyclic flag AND the INPUTS mirror both set, local pads off) must TX flags.rider = 0.
    // Mirrored levels never feed back; a regression switching cyclic_tx to the folded
    // rider_level() fails here.
    let mut s = fresh();
    s.inbox.accept(cyclic(CyclicState::FLAG_RIDER));
    s.inbox.accept(Payload::Inputs(Inputs {
        throttle: 0,
        buttons: 0,
        rider: Inputs::RIDER_PRESENT,
    }));
    assert!(!s.rider_present, "no local pads");
    let c = cyclic_tx(&s, true).expect("addressed board emits");
    assert!(!c.rider_present(), "TX reports LOCAL pads only");
    // The folded level still feeds local consumption (the engage-gate fold is unchanged).
    assert!(super::dispatch::rider_level(&s));
}

#[test]
fn cyclic_tx_emits_every_second_control_run() {
    // Round-4 defect C (specs/link-control.md, "Addressing and emission"): the emission is rate-
    // split to every SECOND control run (125 Hz nominal), because the blocking polled TX cannot
    // fit the 4 ms tick every tick. Over four consecutive control runs an addressed board emits
    // exactly twice, on the even run counts.
    let mut s = fresh();
    let mut emitted = 0;
    for _ in 0..4 {
        let _ = control_task(&mut s, None, 1);
        if cyclic_tx(&s, true).is_some() {
            emitted += 1;
        }
    }
    assert_eq!(emitted, 2, "two emissions across four runs");
    // Parity check: an odd run count is silent, the following even one emits.
    assert!(!s.control_ticks.is_multiple_of(2) || cyclic_tx(&s, true).is_some());
}
