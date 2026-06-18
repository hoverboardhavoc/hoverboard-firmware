//! Bare-metal BLE telemetry mock: bring up the onboard CC2541 BLE module over USART3, take it into
//! transparent DATA mode with the vendor AT sequence, then speak OUR `link` frames over it so the
//! Android app can connect over BLE and exchange frames (Phase 2 of ble-app-protocol-plan.md, the
//! firmware-side mock).
//!
//! This is F103-only (the CC2541 module is on the GD32F103 master board, wired to ST USART3, base
//! 0x40004800, default pins PB10 TX / PB11 RX, on APB1). The descriptor is hardcoded for that part
//! (GpioPath::ApbCrlCrh / ClockPath::F10xRcc), so no feature flags are needed. It mirrors
//! `link_bench.rs`: same descriptor build, same `enable_usart`/`enable_gpio_port`/`configure_af`/
//! `Usart::bring_up`, the same raw-`Usart` tight RX drain with the F1 ORE-clear (read SR@+0x00 then
//! DR@+0x04), the same SWD-readable observation block, and the same `extern crate stm32f1xx_hal;`
//! to pull in the device vector table.
//!
//! The module powers up in AT command mode and is a transparent BLE<->UART bridge. Two phases:
//!
//! 1. BAUD AUTO-DETECT + AT BRING-UP. The BT-UART baud is unknown, so we cycle candidate bauds
//!    {115200, 9600, 19200, 38400, 57600}; at each we re-program the USART (call `Usart::bring_up`
//!    again with the new baud) and send `AT\r\n` a few times, watching RX for the literal 7 bytes
//!    `AT+OK\r\n`. Once seen, we LOCK that baud and run the vendor AT sequence at it:
//!      step 0: `AT\r\n` -> wait for `AT+OK\r\n`  (this is what the sweep already proved; locked here)
//!      step 1: `AT+NAME=Pal\r\n`        (advertise as "Pal" to match the app default)
//!      step 2: `AT+CON_INTERVAL=16\r\n` (NOTE the underscore; the real vendor command)
//!      step 3: `AT+ADV_INTERVAL=32\r\n` (underscore)
//!      step 4: `AT+SET=1\r\n`
//!      step 5: `AT+MODE=DATA\r\n`       -> the module becomes a transparent byte bridge; no more AT.
//!    While not yet in DATA mode, RX bytes feed ONLY the `AT+OK\r\n` detector, not the frame framer.
//!    If no `AT+OK` at any baud after a full sweep, we keep cycling (the module may be slow to boot).
//!
//! 2. DATA MODE (transparent). RX bytes now feed a `link::framer::StreamFramer`. Each decoded
//!    `Inputs` (0x50) frame is decoded, its throttle recorded, and an inbound counter bumped.
//!    Periodically (spaced ~10 Hz, the same `TX_INTERVAL` drain-spacing approach as `link_bench` so
//!    the FIFO-less polled RX is not starved) we SYNTHESIZE a `Telemetry` frame from the latest
//!    commanded throttle and write it over USART3. There are no real wheels yet, so the telemetry is
//!    mocked: speed tracks throttle, current scales with |throttle|, battery sags slightly under
//!    load.
//!
//! All results are recorded in a fixed `#[no_mangle]` observation block (`BT_OBS`) at the start of
//! RAM, read over SWD with `mdw <addr> 7` to see whether the module answered AT+OK (and at which
//! baud), whether DATA mode was reached, and whether frames flow once the app connects.
//!
//! Clock: HSI 8 MHz, no PLL, APB1 = 8 MHz (sysclk 8_000_000, wait_states 0, prescalers 1), the same
//! ClockProfile as `link_bench.rs`. We do NOT call configure_tree (the board runs on the reset HSI
//! clock).

#![no_std]
#![no_main]

use panic_halt as _;

// Pull in the device PAC's interrupt vector table (same reason as link_bench: cortex-m-rt's `device`
// feature, enabled transitively by stm32f1xx-hal, needs a svd2rust device crate for `__INTERRUPTS`).
// We use none of its peripherals.
extern crate stm32f1xx_hal;

use cortex_m_rt::entry;

use runtime_hal::addr::{AddrTable, PeriphLabel};
use runtime_hal::descriptor::{
    AdcPath, ClockPath, ClockProfile, ClockSource, GpioPath, IrqLayout, McuDescriptor, PageSize,
};
use runtime_hal::gpio::{configure_af, PinRole};
use runtime_hal::usart::{Usart, UsartBus};
use runtime_hal::{enable_gpio_port, enable_usart, Reg32};

use link::frame::{encode, FrameHeader, MAX_FRAME, PROTO_VER};
use link::framer::StreamFramer;
use link::opcode::Opcode;
use link::payload::{Inputs, Telemetry};

use heapless::Vec;

// --- Hardware constants (F103 master, USART3 / the onboard CC2541 module) -----------------------

/// ST USART3 data base (the CC2541 BLE module's UART). On APB1. In runtime-hal's GD-indexed labels
/// this APB1 instance is `Usart2` (its base window is 0x4000_4400..0x4000_5000, which covers this).
const USART3_BASE: u32 = 0x4000_4800;
/// The runtime-hal label for the APB1 USART at 0x4000_4800.
const USART3_LABEL: PeriphLabel = PeriphLabel::Usart2;
/// RCU / RCC base.
const RCU_BASE: u32 = 0x4002_1000;
/// GPIOB base on F10x (APB2). USART3 TX/RX are on port B.
const GPIOB_BASE: u32 = 0x4001_0C00;
/// TX pin PB10: logical pin byte (port B = 1, pin 10).
const PIN_PB10: u8 = (1 << 4) | 10;
/// RX pin PB11: logical pin byte (port B = 1, pin 11).
const PIN_PB11: u8 = (1 << 4) | 11;

/// This board's node id (the Telemetry source). The app's default board destination is 0xFF.
const BOARD_NODE_ID: u8 = 2;
/// Telemetry destination: broadcast (matches the app default boardDst).
const APP_DST: u8 = 0xFF;

/// Candidate BT-UART bauds, swept in order until `AT+OK\r\n` is seen. 115200 first (most likely).
const CANDIDATE_BAUDS: [u32; 5] = [115_200, 9_600, 19_200, 38_400, 57_600];

/// `AT\r\n` retries to send at each candidate baud before moving to the next baud in the sweep.
const AT_PROBES_PER_BAUD: u32 = 4;

/// Drain-loop passes to spend draining RX between successive `AT\r\n` probe sends (sweep phase). The
/// FIFO-less polled RX must spend almost all its time draining so the module's `AT+OK\r\n` reply is
/// caught; this spacing is far longer than the reply's on-wire time.
const PROBE_DRAIN_PASSES: u32 = 12_000;

/// Guard-time silence (drain-loop passes with no TX) around the Hayes `+++` escape, roughly 1 second.
/// If the module is stuck in transparent DATA mode from a prior bring-up, a guarded `+++` pulls many
/// CC2541/HM-10-class vendor firmwares back to AT command mode without a power-cycle.
const ESCAPE_GUARD_PASSES: u32 = 250_000;

/// Drain-loop passes between Telemetry transmits in DATA mode (the link_bench TX_INTERVAL approach:
/// rare TX relative to the RX poll so the receiver keeps up). ~10 Hz spacing in wall-clock terms.
const TX_INTERVAL: u32 = 40_000;

/// When true, skip the AT handshake/sweep and assume the module is ALREADY in transparent DATA mode
/// at [`DATA_MODE_BAUD`] (a prior bring-up left it there and there is no runtime escape, so it cannot
/// be re-handshaked without a power-cycle). Bridge frames directly. Set false for a fresh module in
/// AT mode (the normal sweep + AT bring-up path then runs).
const ASSUME_DATA_MODE: bool = true;

/// The DATA-mode UART baud the module was previously locked to (9600, empirically detected).
const DATA_MODE_BAUD: u32 = 9_600;

// --- AT sequence (sent verbatim, CRLF-terminated) ----------------------------------------------

/// The literal reply the module sends in AT mode to `AT\r\n`: 7 bytes `AT+OK\r\n`.
const AT_OK: &[u8] = b"AT+OK\r\n";

/// The post-lock AT command sequence (steps 1..=5), each CRLF-terminated, sent in order. Step 0
/// (`AT\r\n` until `AT+OK\r\n`) is handled by the sweep/lock; these are the commands sent after the
/// baud is locked, ending with `AT+MODE=DATA` which drops the module into transparent DATA mode.
// Order is the authoritative reference AT sequence for the module: NAME, ADV_INTERVAL,
// CON_INTERVAL, then MODE=DATA, then SET=1. Critically MODE=DATA comes BEFORE SET=1 (the SPEC.md
// table had this backwards); a wrong commit/mode order on this module leaves it
// advertising but not connectable, which is the connect-hang symptom we hit.
const AT_STEPS: [&[u8]; 5] = [
    b"AT+NAME=Pal\r\n",
    b"AT+ADV_INTERVAL=32\r\n",
    b"AT+CON_INTERVAL=16\r\n",
    b"AT+MODE=DATA\r\n",
    b"AT+SET=1\r\n",
];

// --- Observation block read over SWD -----------------------------------------------------------

/// Magic stamped at boot so the SWD reader can confirm the block is live and laid out as expected.
const OBS_MAGIC: u32 = 0xB2B2_B2B2;

/// Fixed-layout observation block, read back over SWD with `mdw <addr> 7`.
///
/// `#[repr(C)]` so the field offsets are stable:
/// - +0x00 `magic`                   0xB2B2B2B2 once booted (block liveness / layout check)
/// - +0x04 `boot`                    increments once at boot (liveness)
/// - +0x08 `at_state`                AT step reached, 0..=6: 0 = no AT+OK yet (sweeping); 1 = AT+OK
///                                   seen + baud locked (step 0 done); 2..=6 = after sending
///                                   AT_STEPS[0..=4]; 6 = AT+MODE=DATA sent -> DATA mode reached
/// - +0x0C `locked_baud`             the detected/locked baud (0 if none yet)
/// - +0x10 `inbound_inputs_count`    decoded Inputs (0x50) frames received in DATA mode
/// - +0x14 `last_throttle` (i32)     throttle of the last decoded Inputs frame
/// - +0x18 `outbound_telemetry_count`  Telemetry (0x20) frames transmitted in DATA mode
#[repr(C)]
pub struct BtObs {
    pub magic: u32,
    pub boot: u32,
    pub at_state: u32,
    pub locked_baud: u32,
    pub inbound_inputs_count: u32,
    pub last_throttle: i32,
    pub outbound_telemetry_count: u32,
}

impl BtObs {
    const fn new() -> Self {
        Self {
            magic: 0,
            boot: 0,
            at_state: 0,
            locked_baud: 0,
            inbound_inputs_count: 0,
            last_throttle: 0,
            outbound_telemetry_count: 0,
        }
    }
}

/// The fixed observation block. `#[no_mangle]` so its symbol address is stable and findable with
/// `nm`, and read over SWD. Written only from the single-core bare-metal main loop (no concurrency).
#[no_mangle]
pub static mut BT_OBS: BtObs = BtObs::new();

/// Clear a USART overrun (ORE) so RX self-recovers. The runtime-hal `try_read_byte` returns an error
/// on a line error without reading the data register, so ORE would otherwise latch; we clear it the
/// F1 way: read the status register (SR, +0x00) then the data register (DR, +0x04).
#[inline]
fn clear_overrun() {
    let _ = Reg32::new(USART3_BASE, 0x00).read();
    let _ = Reg32::new(USART3_BASE, 0x04).read();
}

/// Build the MCU descriptor for the F103 master, USART3 path. Only the USART path is exercised, so
/// the non-USART fields are valid-but-unused, exactly as `link_bench` sets them.
fn build_descriptor() -> McuDescriptor {
    let mut addrs = AddrTable::new();
    addrs.set(USART3_LABEL, USART3_BASE);
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

/// A small rolling RX buffer that watches for the literal `AT+OK\r\n` reply. Holds the last
/// `AT_OK.len()` bytes seen and reports a match when the tail equals `AT+OK\r\n`.
struct AtOkDetector {
    buf: Vec<u8, { AT_OK.len() }>,
}

impl AtOkDetector {
    const fn new() -> Self {
        Self { buf: Vec::new() }
    }

    /// Clear the rolling buffer (between probes / on baud change) so a partial reply at the old baud
    /// does not bleed into the next attempt.
    fn reset(&mut self) {
        self.buf.clear();
    }

    /// Feed one RX byte; return true once the rolling tail equals `AT+OK\r\n`.
    fn feed(&mut self, b: u8) -> bool {
        if self.buf.len() == AT_OK.len() {
            // Shift left by one (drop the oldest byte) to keep a fixed-width sliding window.
            for i in 1..AT_OK.len() {
                self.buf[i - 1] = self.buf[i];
            }
            self.buf[AT_OK.len() - 1] = b;
        } else {
            let _ = self.buf.push(b);
        }
        self.buf.len() == AT_OK.len() && &self.buf[..] == AT_OK
    }
}

/// Write a byte slice over the USART (TX is the blocking polled send).
fn write_all(usart: &Usart, bytes: &[u8]) {
    for &b in bytes {
        usart.write_byte(b);
    }
}

/// Drain the RX for `passes` loop iterations, feeding each ready byte to the `AT+OK` detector and
/// clearing overruns. Returns true as soon as `AT+OK\r\n` is seen (and stops early). During the AT
/// phase RX feeds ONLY this detector, never the frame framer.
fn drain_for_at_ok(usart: &Usart, detector: &mut AtOkDetector, passes: u32) -> bool {
    let mut p: u32 = 0;
    while p < passes {
        match usart.try_read_byte() {
            Ok(Some(b)) => {
                if detector.feed(b) {
                    return true;
                }
            }
            Ok(None) => {
                p = p.wrapping_add(1);
            }
            Err(_) => clear_overrun(),
        }
    }
    false
}

/// Drain RX for `passes` empty passes WITHOUT transmitting (a guard-time silence), discarding bytes
/// and clearing overruns. Used to bracket the Hayes `+++` escape so the module's escape detector sees
/// the required idle window before and after.
fn guard_silence(usart: &Usart, passes: u32) {
    let mut p: u32 = 0;
    while p < passes {
        match usart.try_read_byte() {
            Ok(Some(_)) => {}
            Ok(None) => p = p.wrapping_add(1),
            Err(_) => clear_overrun(),
        }
    }
}

/// Sweep the candidate bauds until `AT+OK\r\n` is seen, re-bringing-up the USART at each. Returns
/// the locked `Usart` handle and the baud. Loops forever across the candidate set (the module may be
/// slow to boot), recording the swept baud as it goes so SWD shows progress.
fn detect_baud_and_lock(desc: &McuDescriptor) -> (Usart, u32) {
    let mut detector = AtOkDetector::new();
    loop {
        for &baud in CANDIDATE_BAUDS.iter() {
            // Re-program the USART at this candidate baud (bring_up re-writes BRR + re-enables).
            let usart = Usart::bring_up(
                USART3_BASE,
                desc.clock,
                &desc.clock_cfg,
                UsartBus::Apb1,
                baud,
            );
            // Record the baud currently being probed so SWD shows sweep progress (cleared/overwritten
            // to the locked value on success).
            unsafe {
                let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
                obs.locked_baud = baud;
            }
            detector.reset();
            // If the module is stuck in transparent DATA mode (a prior bring-up left it there with no
            // central to disconnect), a guarded Hayes "+++" escape pulls many vendor firmwares back to
            // AT command mode. Silence (guard) before and after, no CRLF. Harmless if unsupported: it
            // is just forwarded as data and the AT probe below proceeds normally.
            guard_silence(&usart, ESCAPE_GUARD_PASSES);
            write_all(&usart, b"+++");
            guard_silence(&usart, ESCAPE_GUARD_PASSES);
            for _ in 0..AT_PROBES_PER_BAUD {
                write_all(&usart, b"AT\r\n");
                if drain_for_at_ok(&usart, &mut detector, PROBE_DRAIN_PASSES) {
                    return (usart, baud);
                }
            }
        }
    }
}

/// Synthesize a mock `Telemetry` from the latest commanded throttle (no real wheels yet): speed
/// tracks throttle, current scales with |throttle|/10 (centiamps), battery sags slightly under load.
fn mock_telemetry(throttle: i16) -> Telemetry {
    let mag = (throttle as i32).unsigned_abs(); // |throttle|, 0..=32768
    // Battery sags from a 29.4 V nominal by a small load-proportional amount (mV). Cap the sag so the
    // u16 battery_mv never underflows for any i16 throttle.
    let sag = (mag / 8).min(5_000) as u16;
    let battery_mv = 29_400u16.saturating_sub(sag);
    let current_ca = (mag / 10) as i16;
    Telemetry {
        motor_index: 0,
        battery_mv,
        current_ca,
        speed: throttle,
        fault_code: 0,
        flags: 0,
    }
}

#[entry]
fn main() -> ! {
    let desc = build_descriptor();

    // Stamp the observation block (liveness over SWD before anything else runs).
    unsafe {
        let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
        obs.magic = OBS_MAGIC;
        obs.boot = obs.boot.wrapping_add(1);
    }

    // Bring up USART3 in the same order as link_bench: enable clocks, configure TX/RX AF, bring up
    // the USART (at the first candidate baud; the sweep re-programs the baud as it probes).
    let _ = enable_usart(RCU_BASE, desc.clock, USART3_LABEL);
    let _ = enable_gpio_port(RCU_BASE, desc.clock, PeriphLabel::Gpiob);
    configure_af(GPIOB_BASE, desc.gpio, PIN_PB10, PinRole::Tx);
    configure_af(GPIOB_BASE, desc.gpio, PIN_PB11, PinRole::Rx);

    // --- Phase 1: get the module into transparent DATA mode ---

    let usart = if ASSUME_DATA_MODE {
        // The module is already in DATA mode from a prior bring-up and cannot be re-handshaked without
        // a power-cycle (no runtime escape on this firmware). Bring USART3 up at the known DATA-mode
        // baud and bridge frames directly: no sweep, no AT.
        let u = Usart::bring_up(USART3_BASE, desc.clock, &desc.clock_cfg, UsartBus::Apb1, DATA_MODE_BAUD);
        unsafe {
            let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
            obs.locked_baud = DATA_MODE_BAUD;
            obs.at_state = 6; // already in DATA mode
        }
        u
    } else {
        // Sweep the candidate bauds until the module answers `AT+OK\r\n`, locking that baud.
        let (u, locked_baud) = detect_baud_and_lock(&desc);
        unsafe {
            let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
            obs.locked_baud = locked_baud;
            obs.at_state = 1; // step 0 done: AT+OK seen + baud locked.
        }
        // Run the post-lock AT command sequence at the locked baud, draining briefly after each to
        // absorb acks and clear overruns. Result ignored: AT+MODE=DATA may not reply in AT format.
        let mut detector = AtOkDetector::new();
        for (i, cmd) in AT_STEPS.iter().enumerate() {
            write_all(&u, cmd);
            detector.reset();
            let _ = drain_for_at_ok(&u, &mut detector, PROBE_DRAIN_PASSES);
            unsafe {
                let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
                obs.at_state = (i as u32) + 2;
            }
        }
        u
    };

    // --- Phase 2: DATA mode. RX feeds the StreamFramer; synthesize + send Telemetry periodically ---

    let mut framer = StreamFramer::new();
    let mut framebuf = [0u8; MAX_FRAME];
    let mut payload = [0u8; Telemetry::LEN];
    let mut tx_div: u32 = 0;
    // The latest commanded throttle (tracked from inbound Inputs; drives the synthesized Telemetry).
    let mut last_throttle: i16 = 0;

    loop {
        // --- Drain RX every pass: feed each ready byte to the framer; clear ORE on overrun ---
        // Collect decoded Inputs throttles in a small scratch (the framer borrows `framer`, so we
        // record into BT_OBS after the drain, matching link_bench's pattern).
        let mut seen_throttles: Vec<i16, 16> = Vec::new();
        loop {
            match usart.try_read_byte() {
                Ok(Some(b)) => {
                    framer.feed(&[b], &mut |f| {
                        if f.header.opcode == Opcode::Inputs {
                            if let Ok(inp) = Inputs::decode(f.payload) {
                                let _ = seen_throttles.push(inp.throttle);
                            }
                        }
                    });
                }
                Ok(None) => break,
                // Overrun (or other line error): clear it and keep draining. The lost byte only
                // corrupts the in-flight frame; the framer resyncs on the next SOF.
                Err(_) => clear_overrun(),
            }
        }
        if !seen_throttles.is_empty() {
            // The most recent throttle is the live setpoint for the synthesized telemetry.
            last_throttle = *seen_throttles.last().unwrap();
            unsafe {
                let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
                for &t in seen_throttles.iter() {
                    obs.inbound_inputs_count = obs.inbound_inputs_count.wrapping_add(1);
                    obs.last_throttle = t as i32;
                }
            }
        }

        // --- Synthesize + transmit a Telemetry frame, spaced out so the link can carry it ---
        tx_div = tx_div.wrapping_add(1);
        if tx_div >= TX_INTERVAL {
            tx_div = 0;
            let telemetry = mock_telemetry(last_throttle);
            let n = telemetry.encode(&mut payload);
            let hdr = FrameHeader {
                ver: PROTO_VER,
                opcode: Opcode::Telemetry,
                src: BOARD_NODE_ID,
                dst: APP_DST,
                len: n as u8,
            };
            if let Ok(total) = encode(&hdr, &payload[..n], &mut framebuf) {
                write_all(&usart, &framebuf[..total]);
                unsafe {
                    let obs = &mut *core::ptr::addr_of_mut!(BT_OBS);
                    obs.outbound_telemetry_count = obs.outbound_telemetry_count.wrapping_add(1);
                }
            }
        }
    }
}
