//! Bare-metal balance-control firmware: assemble the subsystem crates onto runtime-hal into one
//! F103 image that boots to a SAFE (DISARMED) state, runs the 250 Hz control pipeline through the
//! cooperative scheduler, and proves the wiring over SWD without ever energizing a motor.
//!
//! This is the integration binary for the control path (distinct from the link-layer bench bins
//! `link_bench` / `link_btmock`). It wires, in one pipeline:
//!
//!   scheduler  -> the 250 Hz cooperative table (a 4 ms control task + a 16 ms input task)
//!   imu        -> the MPU-6050 front-end (generic over embedded-hal I2c; runtime-hal's I2c impl)
//!   attitude   -> the Mahony complementary filter (gyro + accel -> pitch)
//!   control    -> the balance cascade (balance PID + the arming/run FSM)
//!   inputs     -> debounce + throttle IIR + pad field
//!   buzzer     -> the chime cadence (per-tick on/off)
//!   state      -> the OFF/INIT/READY/RUN/SHUTDOWN mode machine + per-motor MOE arming gate
//!   commutation-> the FOC hot path (MotorRig / control_step / register_control_loop)
//!
//! SAFETY (the load-bearing invariant of this binary): the motor bridge is DISARMED by default and
//! this firmware NEVER opens MOE. The safety / commissioning gates (board-config validation, the
//! current-limited self-test in todo/safety.md) are specs only, not implemented, so per
//! `spec/core.md` and `todo/safety.md` the firmware MUST default to disarmed: no motor power until
//! a commissioned, current-limited arm path exists.
//!
//! How disarm is guaranteed structurally (not just by omission):
//!   - The `state::ModeMachine` sets each motor's MOE gate ONLY on the INIT pass (`set_at_init`).
//!     We hold the machine in OFF by driving `power_request = false` AND `off_inhibit = true`
//!     every tick, so OFF -> INIT is gated closed and the machine never reaches INIT. Therefore
//!     `moe_allowed(i)` stays `false` for every motor for the life of the firmware.
//!   - The hot path (`commutation`) is wired and COMPILE-checked (the `MotorRig` / `control_step` /
//!     `register_control_loop` API), but its construction + registration live in a
//!     `#[allow(dead_code)]` function (`arm_hot_path_disabled`) that boot NEVER calls. Constructing
//!     the real PWM/ADC handles needs the motor-bridge timer/ADC bring-up (not done here), and even
//!     if registered, `MotorRig::control_step` writes only duties and holds NO MOE accessor, so it
//!     cannot energize a disarmed bridge. MOE lives solely in runtime-hal's `arming::ArmGate`, which
//!     this firmware never constructs or calls.
//!   - The control pipeline computes a balance output for observation, but no duty ever reaches a
//!     pin: there is no PWM bring-up at boot and MOE is never set.
//!
//! Clock: HSI 8 MHz, no PLL, APB1 = 8 MHz (the same reset-clock ClockProfile as `link_bench`). We do
//! NOT call configure_tree. SysTick is programmed via cortex-m for the 250 Hz tick (8e6/250 - 1 =
//! 31999, well within the 24-bit reload).
//!
//! Observation: a fixed `#[no_mangle]` block (`CTRL_OBS`) at the start of RAM, read over SWD with
//! `mdw <addr> N`, exposes the boot/liveness, the scheduler tick count, the mode byte, the last
//! pitch + control output, the MOE-allowed flag (which must read 0 = disarmed), and an IMU/sensor
//! liveness field. F103-only: the descriptor is hardcoded for the GD32F103 master board.

#![no_std]
#![no_main]

use panic_halt as _;

// Pull in the device PAC's interrupt vector table (same reason as link_bench / link_btmock:
// cortex-m-rt's `device` feature, enabled transitively by stm32f1xx-hal, needs a svd2rust device
// crate for `__INTERRUPTS`). We use none of its peripherals; the SysTick comes from cortex-m's core
// peripherals, and all device access is through runtime-hal.
extern crate stm32f1xx_hal;

use core::sync::atomic::{compiler_fence, Ordering};

use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;

use runtime_hal::addr::{AddrTable, PeriphLabel};
use runtime_hal::descriptor::{
    AdcPath, ClockPath, ClockProfile, ClockSource, GpioPath, IrqLayout, McuDescriptor, PageSize,
};
use runtime_hal::enable_i2c;
use runtime_hal::i2c::{FastDuty, I2c};

use heapless::Vec;

// --- Subsystem crates ---------------------------------------------------------------------------

use scheduler::Scheduler;

use imu::{Config as ImuConfig, Mpu6050, Sample};

use attitude::{Config as AttConfig, Mahony, Output as AttOutput};

use control::{balance_pid, select_profile, IirCarry, PidInputs};

use inputs::{LineBank, PadBank, ThrottleFilter};

use buzzer::Chime;

use state::{ModeInputs, ModeMachine};

// The commutation hot path is referenced for compile-checked wiring only; boot never drives it. See
// `arm_hot_path_disabled` below. Imported there to keep the boot path free of any motor handle.

// --- Hardware constants (F103 master, hardcoded like link_bench's f103 path) --------------------

/// RCU / RCC base (identical address on both families; the clock path owns the offsets).
const RCU_BASE: u32 = 0x4002_1000;

/// GPIOB base on F10x (APB2). The MPU-6050 I2C (I2C0) SCL/SDA are PB6/PB7 on this board.
const GPIOB_BASE: u32 = 0x4001_0C00;

/// I2C0 data base on F10x (APB1), the MPU-6050 bus.
const I2C0_BASE: u32 = 0x4000_5400;

/// The runtime-hal label for the first I2C instance (the F1x0 single I2C block is also `I2c0`).
const I2C0_LABEL: PeriphLabel = PeriphLabel::I2c0;

/// MPU-6050 bus speed: 100 kHz standard mode (spec/core.md / imu crate: the IMU runs at 100 kHz).
const I2C_SPEED_HZ: u32 = 100_000;

/// I2C own address the SPL programs (the bench probe value); as a single-master bus controller it is
/// rarely matched, but the bring-up writes it.
const I2C_OWN_ADDR: u8 = 0x24;

/// Number of motors this image is sized for. The F103 master is single-motor, but the mode machine
/// is generic; two MOE gates are tracked so the dual-motor (RCT6) path uses the same shape. Both
/// gates default closed and are never opened by this firmware.
const N_MOTORS: usize = 2;

/// Debounce line count (rider/brake/mode buttons). Sized small; unused lines read low.
const N_BUTTON_LINES: usize = 4;

// --- Scheduler reload periods (in 250 Hz ticks) -------------------------------------------------

/// Control task reload: every tick = 4 ms = 250 Hz (the balance/control pipeline rate).
const CONTROL_RELOAD: u16 = 1;

/// Input task reload: every 4 ticks = 16 ms (debounce / throttle / pad sampling rate).
const INPUT_RELOAD: u16 = 4;

// --- Observation block read over SWD ------------------------------------------------------------

/// Magic stamped at boot so the SWD reader can confirm the block is live and laid out as expected.
const OBS_MAGIC: u32 = 0xC0C0_C0C0;

/// Fixed-layout observation block, read back over SWD with `mdw <addr> N`.
///
/// `#[repr(C)]` so the field offsets are stable:
/// - +0x00 `magic`              0xC0C0C0C0 once booted (block liveness / layout check)
/// - +0x04 `boot`               increments once at boot (liveness)
/// - +0x08 `tick_count`         scheduler ticks driven (proves the timebase + scheduler.tick run)
/// - +0x0C `dispatch_count`     scheduler dispatch passes (proves the main loop drains due tasks)
/// - +0x10 `control_runs`       times the 4 ms control task body ran (pipeline liveness)
/// - +0x14 `input_runs`         times the 16 ms input task body ran
/// - +0x18 `mode`               the state machine's mode byte (0 OFF .. 4 SHUTDOWN; stays 0 = OFF)
/// - +0x1C `moe_allowed`        1 if ANY motor MOE is allowed, else 0. MUST read 0 (disarmed).
/// - +0x20 `last_pitch_milli`   last attitude pitch, millidegrees (i32; pitch_deg * 1000)
/// - +0x24 `last_control_out`   last balance-PID output word (i32; clamped +-28500 by the cascade)
/// - +0x28 `imu_live`           IMU/sensor liveness: 0 = no IMU read attempted, 1 = last read ok,
///                              2 = last read errored (bus). Boots 0; updated each control tick.
/// - +0x2C `imu_temp_centi`     last IMU temperature, centidegrees C (telemetry; 0 until a good read)
#[repr(C)]
pub struct CtrlObs {
    pub magic: u32,
    pub boot: u32,
    pub tick_count: u32,
    pub dispatch_count: u32,
    pub control_runs: u32,
    pub input_runs: u32,
    pub mode: u32,
    pub moe_allowed: u32,
    pub last_pitch_milli: i32,
    pub last_control_out: i32,
    pub imu_live: u32,
    pub imu_temp_centi: i32,
}

impl CtrlObs {
    const fn new() -> Self {
        Self {
            magic: 0,
            boot: 0,
            tick_count: 0,
            dispatch_count: 0,
            control_runs: 0,
            input_runs: 0,
            mode: 0,
            moe_allowed: 0,
            last_pitch_milli: 0,
            last_control_out: 0,
            imu_live: 0,
            imu_temp_centi: 0,
        }
    }
}

/// IMU-liveness codes for `CtrlObs::imu_live`.
const IMU_LIVE_NONE: u32 = 0;
const IMU_LIVE_OK: u32 = 1;
const IMU_LIVE_ERR: u32 = 2;

/// The fixed observation block. `#[no_mangle]` so its symbol address is stable and findable with
/// `nm`, and read over SWD. Written only from the single-core bare-metal main loop and the scheduler
/// callbacks it dispatches (cooperative, no preemption against the writers here).
#[no_mangle]
pub static mut CTRL_OBS: CtrlObs = CtrlObs::new();

// --- The shared control state the scheduler callbacks operate on --------------------------------

/// All subsystem instances + the resolved I2C handle, owned in one place so the bare `fn()`
/// scheduler callbacks (which take no arguments) can reach them. The scheduler is cooperative and
/// single-core: `tick` runs in the SysTick edge (here driven from the main loop, not the ISR, to
/// keep the boot path simple and avoid sharing `CTRL` with an interrupt), and `dispatch` runs the
/// callbacks in the main loop, so there is no concurrent access to this state.
struct CtrlState {
    /// The MPU-6050 front-end (calibration + still-detection state).
    imu: Mpu6050,
    /// The resolved I2C bus handle the IMU reads through. `None` until bring-up wires it; the
    /// control task falls back to a zero/stub `Sample` when the handle is absent so the pipeline
    /// still runs and the wiring is proven without depending on a live sensor at the bench.
    i2c: Option<I2c>,
    /// The Mahony attitude filter (gyro + accel -> pitch).
    attitude: Mahony,
    /// The balance-PID IIR carry (the high-precision running accumulator across ticks).
    pid_iir: IirCarry,
    /// The mode / arming state machine (per-motor MOE gates). Boots OFF, held OFF (disarmed).
    mode: ModeMachine<N_MOTORS>,
    /// Button debounce bank.
    buttons: LineBank,
    /// Pad (rider-presence) bank.
    pads: PadBank,
    /// Throttle IIR + scaling filter.
    throttle: ThrottleFilter,
    /// The buzzer chime cadence (per-tick on/off). Driven disarmed (the boot beep gate is `armed`,
    /// which is false, so the chime stays idle here).
    chime: Chime,
}

impl CtrlState {
    /// Construct all subsystems in their safe initial state. Pure; no register access (the I2C
    /// handle is wired in separately at bring-up and defaults to `None`). Not `const`: the IMU,
    /// attitude filter, and throttle filter constructors are not `const fn`, so this runs once in
    /// `main` before the scheduler starts.
    fn new_pending() -> Self {
        Self {
            imu: Mpu6050::new(ImuConfig::default()),
            i2c: None,
            attitude: Mahony::new(AttConfig::default()),
            // `attitude::Fix` is `I32F32`, the same type the control crate's `IirCarry.carry` uses;
            // re-using it avoids a direct `fixed` dependency in this bin.
            pid_iir: IirCarry {
                carry: attitude::Fix::ZERO,
            },
            mode: ModeMachine::new(),
            buttons: LineBank::new(N_BUTTON_LINES),
            pads: PadBank::new(),
            throttle: ThrottleFilter::new(),
            chime: Chime::new(),
        }
    }
}

/// The single shared control state. `static mut` so the bare `fn()` scheduler callbacks reach it
/// (matching the scheduler crate's documented model: a callback is a `fn()` that can only reach
/// statics). Accessed cooperatively from the main loop and the callbacks it dispatches; no preemptive
/// writer, so `addr_of_mut!` raw-pointer access is sound here.
static mut CTRL: Option<CtrlState> = None;

// --- The 250 Hz control task (4 ms) -------------------------------------------------------------

/// The 4 ms control-pipeline task (scheduler slot, reload = 1). Runs the full outer pipeline:
/// read an IMU sample (or a zero/stub sample if no I2C handle is wired) -> attitude.update ->
/// balance-PID control step -> mode-machine tick. Computes a balance output for observation but
/// NEVER drives a motor: no duty reaches a pin and MOE is never opened.
fn control_task() {
    // Cooperative single-core access to the shared state (no preemptive writer).
    let st = match unsafe { (*core::ptr::addr_of_mut!(CTRL)).as_mut() } {
        Some(s) => s,
        None => return,
    };

    // 1. IMU sample: read through the I2C handle if wired, else a zero/stub Sample so the pipeline
    //    runs without a live sensor (the wiring is what we are proving, not a bench reading).
    let (sample, imu_live, temp_centi) = match st.i2c.as_mut() {
        Some(bus) => match st.imu.read(bus) {
            Ok(s) => (s, IMU_LIVE_OK, s.temp_centi_degc),
            Err(_) => (zero_sample(), IMU_LIVE_ERR, 0),
        },
        None => (zero_sample(), IMU_LIVE_NONE, 0),
    };

    // 2. Attitude: Mahony update from the calibrated gyro (rad/s) + sign-applied accel counts. The
    //    IMU's gyro is already in the filter's body Q (I32F32); the accel is provided as direction
    //    counts, converted to the filter's Q here.
    let gyro = sample.gyro;
    let accel = [
        attitude::Fix::from_num(sample.accel_raw[0]),
        attitude::Fix::from_num(sample.accel_raw[1]),
        attitude::Fix::from_num(sample.accel_raw[2]),
    ];
    let att: AttOutput = st.attitude.update(gyro, accel);

    // 3. Balance cascade: a balance-PID step from the pitch + setpoints. Setpoints are neutral here
    //    (no rider command yet); `select_profile(false)` is the no-rider/standby gain profile, so
    //    the computed output is a safe small value used only for observation. The gyro rate word
    //    (bv) feeds the derivative path.
    let profile = select_profile(false);
    let triple = profile.as_triple();
    let pitch_milli = pitch_to_millideg(att);
    let pid_in = PidInputs {
        bv: sample.gyro_raw[1] as i32, // pitch-axis gyro rate (body Y), raw word
        bk: triple.bk,
        pp: 0,            // fore/aft pitch command: neutral (no rider input)
        kp: triple.kp,
        pr: triple.pr,
        kd: attitude::Fix::ZERO, // derivative coefficient: 0 in the standby profile (I32F32)
        off: pitch_milli, // shaped pitch target / commanded lean: use the measured pitch
        scale: 2400,      // filtered battery centivolts (24.0 V placeholder normalization divisor)
    };
    let pid_out = balance_pid(&pid_in, &mut st.pid_iir);

    // 4. Mode / arming state machine: hold OFF (DISARMED). power_request is FALSE and off_inhibit is
    //    TRUE every tick, so OFF -> INIT is gated closed and MOE is never set. The commissioning /
    //    current-limited self-test that would justify arming is not implemented (todo/safety.md), so
    //    the firmware defaults to disarmed exactly as the safety spec requires.
    let mode_in = ModeInputs {
        power_request: false, // no rider-power intent: holds OFF
        fault_a: false,
        fault_b: false,
        off_inhibit: true, // not commissioned: block OFF -> INIT (belt-and-braces with no request)
    };
    let outcome = st.mode.tick(&mode_in);

    // 5. Buzzer chime: driven disarmed, so it stays idle (no beep). Stepped so its cadence state
    //    advances and the wiring is exercised.
    let _tick_out = st.chime.tick(false);

    // Publish observation. MOE-allowed is read back from the machine and MUST be 0 (disarmed).
    let moe = if st.mode.any_moe_allowed() { 1 } else { 0 };
    unsafe {
        let obs = &mut *core::ptr::addr_of_mut!(CTRL_OBS);
        obs.control_runs = obs.control_runs.wrapping_add(1);
        obs.mode = outcome.mode.as_byte() as u32;
        obs.moe_allowed = moe;
        obs.last_pitch_milli = pitch_milli;
        obs.last_control_out = pid_out.out;
        obs.imu_live = imu_live;
        if imu_live == IMU_LIVE_OK {
            obs.imu_temp_centi = temp_centi;
        }
    }
}

// --- The 16 ms input task -----------------------------------------------------------------------

/// The 16 ms input task (scheduler slot, reload = 4). Runs the input front-end: debounce the sampled
/// button bits, advance the pad (rider-presence) bank, and step the throttle IIR from a raw throttle
/// word. There is no real ADC/GPIO sampling wired here (that is bench bring-up), so the sampled bits
/// and raw throttle are stubbed at idle; the point is that the debounce/throttle/pad path runs.
fn input_task() {
    let st = match unsafe { (*core::ptr::addr_of_mut!(CTRL)).as_mut() } {
        Some(s) => s,
        None => return,
    };

    // Sampled button bits (all idle: no buttons pressed). Real sampling reads the GPIO ISTAT bits.
    let sampled_bits: u8 = 0;
    st.buttons.update(sampled_bits);

    // Pad lines (rider-presence): both idle. Real sampling reads the two pad GPIOs.
    let _pad_field = st.pads.update(false, false);

    // Throttle: idle raw word (mid-scale would be the real neutral; 0 here keeps it deterministic
    // and the filter primes its baseline on the first sample without producing drive).
    let raw_throttle: u16 = 0;
    let _throttle = st.throttle.step(raw_throttle);

    unsafe {
        let obs = &mut *core::ptr::addr_of_mut!(CTRL_OBS);
        obs.input_runs = obs.input_runs.wrapping_add(1);
    }
}

// --- Helpers ------------------------------------------------------------------------------------

/// A zero/stub IMU [`Sample`]: used when no I2C handle is wired or a read errors, so the control
/// pipeline still runs (the attitude filter primes to a level estimate, the cascade computes a
/// neutral output). All axes zero, level, not still.
fn zero_sample() -> Sample {
    Sample {
        gyro: [attitude::Fix::ZERO; 3],
        gyro_raw: [0; 3],
        accel_raw: [0, 0, 0],
        temp_centi_degc: 0,
        still: false,
    }
}

/// Convert the attitude filter's pitch (degrees, `I16F16`) to millidegrees as an `i32` for the
/// observation block and the cascade's `off` lean word.
fn pitch_to_millideg(att: AttOutput) -> i32 {
    // pitch_deg is I16F16 (Out); scale by 1000 in fixed-point then truncate to an i32.
    let milli = att.pitch_deg * attitude::Out::from_num(1000);
    milli.to_num::<i32>()
}

// --- The HOT PATH, wired but DISABLED (compile-checked, NEVER called from boot) -----------------

/// Wire the commutation hot path so its API is COMPILE-checked and proven to fit, WITHOUT driving a
/// motor. This is deliberately `#[allow(dead_code)]` and is NEVER called from `main`:
///
///   - Constructing the real `MotorRig` needs the motor-bridge PWM/ADC handles
///     (`ComplementaryPwm::configure` / `TriggeredAdc::configure`), which require the advanced-timer
///     + injected-ADC bring-up this safe boot does not perform. So this builds the handles from
///     resolved bases via the runtime-hal `PwmHandle::new` / `InjectedHandle::new` / `HallReader`
///     constructors purely to type-check the wiring; the resulting registration is never installed
///     because this function is never called.
///   - Even if it WERE called and registered, `MotorRig::control_step` writes only the three duties
///     and re-arms the ADC trigger; it holds NO MOE accessor (a runtime-hal SAFETY invariant). MOE
///     lives only in `arming::ArmGate`, which this firmware never constructs. So the bridge stays
///     disarmed regardless.
///
/// The point is to prove the `MotorRig` / `control_step` / `register_control_loop` API integrates
/// and compiles. It does not, and cannot from this binary, turn a motor.
#[allow(dead_code)]
fn arm_hot_path_disabled() {
    use commutation::foc::MotorParams;
    use commutation::hot::{register_control_loop, MotorRig};
    use runtime_hal::hotpath::hall::HallReader;
    use runtime_hal::hotpath::{InjectedHandle, PwmHandle};

    // TIMER0 (advanced timer) and ADC0 bases on F10x (APB2). Used only to construct the resolve-once
    // handles for the compile check; no peripheral is configured here.
    const TIMER0_BASE: u32 = 0x4001_2C00;
    const ADC0_BASE: u32 = 0x4001_2400;
    const PWM_PERIOD: u16 = 0x8CA; // 2250, the reference center-aligned ARR (one above the trigger)

    // The per-cycle handles (resolved from bases only; no bring-up). NOTE: these are NOT armed and
    // never written, because this function is never called from boot.
    let pwm = PwmHandle::new(TIMER0_BASE, PWM_PERIOD);
    let adc = InjectedHandle::new(ADC0_BASE, 2);
    // Default hall wiring PC13 / PA1 / PC14 on the F10x (APB) GPIO model.
    const GPIOA_BASE: u32 = 0x4001_0800;
    const GPIOC_BASE: u32 = 0x4001_1000;
    let halls = HallReader::resolve(
        GpioPath::ApbCrlCrh,
        [(GPIOC_BASE, 13), (GPIOA_BASE, 1), (GPIOC_BASE, 14)],
    );

    // Build the rig and (would) register the control loop. The ISR handler below walks the rig and
    // calls `control_step` with a demand of 0; it is registered ONLY if this function is called,
    // which boot never does, so the control ISR keeps runtime-hal's no-op default handler.
    let mut rig = MotorRig::new(pwm, adc, halls, MotorParams::default());

    // A demand of 0: even on the bench this would write zero-ish duties, and with MOE disarmed no
    // current flows. Prove the per-cycle step compiles and returns the FOC output.
    let _out = rig.control_step(0);

    // The control-ISR body the firmware would register. `extern "C" fn()` with no captures (it can
    // only reach statics, like the scheduler callbacks). Left here to prove `register_control_loop`'s
    // signature fits; it owns no live rig, so it is a safe no-op even if installed.
    extern "C" fn control_isr() {
        // A real handler walks a `'static` MotorRig array and calls control_step per motor with that
        // motor's demand from shared RAM. Disarmed: writes duties only, never arms MOE.
    }
    register_control_loop(control_isr);
}

/// Build the F103 master MCU descriptor (hardcoded, like `link_bench`'s f103 path). Only the I2C
/// path is exercised at bring-up; the non-I2C fields are valid-but-unused, matching the bench bins.
fn build_descriptor() -> McuDescriptor {
    let mut addrs = AddrTable::new();
    addrs.set(I2C0_LABEL, I2C0_BASE);
    addrs.set(PeriphLabel::Gpiob, GPIOB_BASE);
    addrs.set(PeriphLabel::Rcu, RCU_BASE);

    let clock_cfg = ClockProfile {
        sysclk_hz: 8_000_000,
        wait_states: 0,
        source: ClockSource::Irc8m,
        pll_mul: ClockProfile::DEFAULT_PLL_MUL,
        ahb_psc: 1,
        apb1_psc: 1,
        apb2_psc: 1,
    };

    McuDescriptor {
        gpio: GpioPath::ApbCrlCrh,
        clock: ClockPath::F10xRcc,
        adc: AdcPath::Single,
        irq: IrqLayout::F1x0Grouped,
        addrs,
        clock_cfg,
        flash_page: PageSize::K1,
        adv_timers: 1,
        adc_count: 1,
        usarts: Vec::new(),
        i2cs: Vec::new(),
        spis: Vec::new(),
        adcs: Vec::new(),
        pwms: Vec::new(),
        injected: Vec::new(),
    }
}

#[entry]
fn main() -> ! {
    let desc = build_descriptor();

    // Stamp the observation block first (liveness over SWD before anything else runs).
    unsafe {
        let obs = &mut *core::ptr::addr_of_mut!(CTRL_OBS);
        obs.magic = OBS_MAGIC;
        obs.boot = obs.boot.wrapping_add(1);
    }

    // --- Bring up the subsystem state (no register access) ---
    unsafe {
        *core::ptr::addr_of_mut!(CTRL) = Some(CtrlState::new_pending());
    }

    // --- I2C bring-up for the IMU (the one cold-path peripheral we wire here) ---
    // Enable the I2C0 peripheral clock, then resolve the bus handle at 100 kHz. NOTE: the SCL/SDA
    // GPIO alternate-function (open-drain AF) is NOT configured here: runtime-hal's `PinRole` only
    // covers USART Tx/Rx, so an I2C-AF pin config is not expressible through it. We therefore bring
    // up the I2C peripheral + handle (so the IMU read path is fully wired behind a configured
    // handle) but leave the pin muxing to bench bring-up; an actual IMU read will only ACK once the
    // SCL/SDA pins are muxed. The control task already tolerates a failing read (falls back to the
    // stub sample), so this is boot-correct and safe.
    let _ = enable_i2c(RCU_BASE, desc.clock, I2C0_LABEL);
    let i2c = I2c::bring_up(I2C0_BASE, &desc.clock_cfg, I2C_SPEED_HZ, FastDuty::Two, I2C_OWN_ADDR);

    // Attempt the one-time IMU boot configuration (wake + full-scale). If it errors (pins not yet
    // muxed at the bench), the handle is still wired and the control task uses the stub sample until
    // a real read ACKs. Wire the handle into the shared state either way.
    unsafe {
        if let Some(st) = (*core::ptr::addr_of_mut!(CTRL)).as_mut() {
            let mut bus = i2c;
            let _ = st.imu.init(&mut bus);
            st.i2c = Some(bus);
        }
    }

    // --- Scheduler: register the periodic tasks ---
    let mut sched = Scheduler::new();
    sched.systick_init(); // clear the table + status (MCU-agnostic part)
    // Control task at 250 Hz (reload 1 = every 4 ms), input task at 16 ms (reload 4).
    let _ = sched.register(control_task, CONTROL_RELOAD);
    let _ = sched.register(input_task, INPUT_RELOAD);

    // --- Timebase: program SysTick for the 250 Hz tick (8 MHz / 250 - 1 = 31999) ---
    // We DRIVE scheduler::tick from the SysTick count-flag polled in the main loop (rather than from
    // the SysTick ISR), so `tick` and `dispatch` both run cooperatively in the main loop with no
    // concurrent access to CTRL. This is boot-correct: the tick cadence is set by the reload, and
    // polling COUNTFLAG drains exactly one tick per reload period.
    // Keep the SysTick peripheral so the main loop can poll its count flag (`has_wrapped` is an
    // instance method). `None` if the core peripherals were already taken (should not happen at
    // boot); the loop then falls back to a busy-counted tick.
    let mut systick: Option<cortex_m::peripheral::SYST> = None;
    if let Some(cp) = CorePeripherals::take() {
        // systick_load returns Err only on a 24-bit overflow (not possible at 8 MHz / 250).
        if let Ok(reload) = scheduler::systick_load(8_000_000) {
            let mut syst = cp.SYST;
            syst.set_reload(reload);
            syst.clear_current();
            // Use the processor clock as the SysTick source and enable the counter. We do NOT enable
            // the SysTick interrupt: the count-flag is polled in the main loop.
            syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
            syst.enable_counter();
            systick = Some(syst);
        }
    }

    // --- Main loop: advance the scheduler from the timebase and dispatch due tasks ---
    loop {
        // Advance one scheduler tick per SysTick reload period. If SysTick could not be taken
        // (should not happen at boot), fall back to a busy-counted tick so the scheduler still
        // advances and the bench can observe liveness (boot-correct is enough for this task).
        let ticked = match systick.as_mut() {
            Some(syst) => syst.has_wrapped(),
            None => true,
        };

        if ticked {
            sched.tick();
            compiler_fence(Ordering::SeqCst);
            unsafe {
                let obs = &mut *core::ptr::addr_of_mut!(CTRL_OBS);
                obs.tick_count = obs.tick_count.wrapping_add(1);
            }
        }

        // Run all due tasks in ascending slot order (cooperative). On a tick this drains the tasks
        // that came due; between ticks it is a cheap no-op pass.
        sched.dispatch();
        unsafe {
            let obs = &mut *core::ptr::addr_of_mut!(CTRL_OBS);
            obs.dispatch_count = obs.dispatch_count.wrapping_add(1);
        }
    }
}
