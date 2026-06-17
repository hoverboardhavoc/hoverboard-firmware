//! Bare-metal bench firmware: prove the link-layer CyclicState exchange runs on real silicon.
//!
//! This brings up USART1 (GD `USART1` / ST `USART2`, base 0x40004400, APB1, 115200 8N1) through
//! runtime-hal exactly as the confirmed M1 milestone did, then exchanges `link` CyclicState frames
//! with the peer board. Results are recorded in a fixed, `#[no_mangle]` observation block in RAM
//! (`LINK_OBS`) that can be read over SWD with `mdw`.
//!
//! RX is drained from the raw `Usart` every loop pass (the F1/F0 USART has no RX FIFO, so a byte
//! must be taken within ~87 us at 115200 or it overruns). On an overrun (ORE) we clear it directly
//! (family-specific: F1 reads SR then DR; F0/F1x0 writes ORECF to INTC) and keep draining, so the
//! receiver self-recovers instead of latching dead. TX is spaced out so each board spends almost all
//! its time draining, which is what lets the FIFO-less polled RX keep up. This is why a simple
//! transmit-then-busy-wait loop fails (it starves the drain and ORE latches); the M1 handshake never
//! hit this because it was request/response, not streaming.
//!
//! Two mutually exclusive Cargo features pick the silicon:
//! - `f103` (master): GpioPath::ApbCrlCrh / ClockPath::F10xRcc, GPIOA 0x40010800, node id 1.
//! - `f130` (slave):  GpioPath::AhbCtlAfsel / ClockPath::F1x0Rcu, GPIOA 0x48000000, node id 2.
//!
//! Clock: HSI 8 MHz, no PLL, APB1 = 8 MHz (sysclk 8_000_000, wait_states 0, all prescalers 1), so
//! the USART BRR comes out 0x45 (8e6/115200), matching the M1 note. We do NOT call configure_tree
//! (the board runs on the reset HSI clock).

#![no_std]
#![no_main]

use panic_halt as _;

// Pull in the device PAC's interrupt vector table. cortex-m-rt's `device` feature (enabled
// transitively by the stm32f1xx-hal dependency) requires a svd2rust device crate to supply the
// `__INTERRUPTS` array; linking the PAC in provides it. We use none of its peripherals.
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
use link::payload::CyclicState;

use heapless::Vec;

// --- Family selection (mutually exclusive; neither enabled by default) -------------------------

#[cfg(all(feature = "f103", feature = "f130"))]
compile_error!("features `f103` and `f130` are mutually exclusive; enable exactly one");

#[cfg(not(any(feature = "f103", feature = "f130")))]
compile_error!("select a target family: build with --features f103 or --features f130");

#[cfg(feature = "f103")]
mod target {
    use super::*;
    /// F103 (master): F10x CRL/CRH gpio model.
    pub const GPIO_PATH: GpioPath = GpioPath::ApbCrlCrh;
    /// F103 (master): F10x RCC clock model.
    pub const CLOCK_PATH: ClockPath = ClockPath::F10xRcc;
    /// GPIOA base on F10x (APB2).
    pub const GPIOA_BASE: u32 = 0x4001_0800;
    /// This node's id (master).
    pub const THIS_NODE_ID: u8 = 1;
    /// The pitch value this node advertises in its CyclicState frames.
    pub const MY_PITCH: i16 = 0x1111;
}

#[cfg(feature = "f130")]
mod target {
    use super::*;
    /// F130 (slave): F1x0 CTL/AFSEL gpio model.
    pub const GPIO_PATH: GpioPath = GpioPath::AhbCtlAfsel;
    /// F130 (slave): F1x0 RCU clock model.
    pub const CLOCK_PATH: ClockPath = ClockPath::F1x0Rcu;
    /// GPIOA base on F1x0 (AHB2).
    pub const GPIOA_BASE: u32 = 0x4800_0000;
    /// This node's id (slave).
    pub const THIS_NODE_ID: u8 = 2;
    /// The pitch value this node advertises in its CyclicState frames.
    pub const MY_PITCH: i16 = 0x2222;
}

// --- Common hardware constants (same on both families) -----------------------------------------

/// GD `USART1` (ST `USART2`) data base, on APB1.
const USART1_BASE: u32 = 0x4000_4400;
/// RCU / RCC base (identical address on both families; the clock path owns the offsets).
const RCU_BASE: u32 = 0x4002_1000;
/// Link line rate.
const BAUD: u32 = 115_200;
/// TX pin PA2: logical pin byte (port A = 0, pin 2).
const PIN_PA2: u8 = (0 << 4) | 2;
/// RX pin PA3: logical pin byte (port A = 0, pin 3).
const PIN_PA3: u8 = (0 << 4) | 3;

/// Drain-loop passes between transmits. The receiver must spend almost all its time draining the
/// FIFO-less USART, so TX is rare relative to the RX poll. Exact wall-clock is not critical; this is
/// far larger than one frame's ~1 ms on-wire time, so frames mostly land while the peer is draining.
const TX_INTERVAL: u32 = 40_000;

/// Clear a USART overrun (ORE) so RX self-recovers. The runtime-hal `try_read_byte` returns an error
/// on a line error without reading the data register, so ORE would otherwise latch; we clear it here.
#[cfg(feature = "f103")]
#[inline]
fn clear_overrun() {
    // F1-style USART: reading the status register (SR, +0x00) then the data register (DR, +0x04)
    // clears ORE.
    let _ = Reg32::new(USART1_BASE, 0x00).read();
    let _ = Reg32::new(USART1_BASE, 0x04).read();
}

/// Clear a USART overrun (ORE) so RX self-recovers.
#[cfg(feature = "f130")]
#[inline]
fn clear_overrun() {
    // F0/F1x0-style USART: write ORECF (bit 3) to the interrupt-clear register (INTC, +0x20).
    Reg32::new(USART1_BASE, 0x20).write(0x08);
}

// --- Observation block read over SWD -----------------------------------------------------------

/// Magic stamped at boot so the SWD reader can confirm the block is live and laid out as expected.
const OBS_MAGIC: u32 = 0xB1B1_B1B1;

/// Fixed-layout observation block, read back over SWD with `mdw <addr> 7`.
///
/// `#[repr(C)]` so the field offsets are stable:
/// - +0x00 `magic`           0xB1B1B1B1 once booted
/// - +0x04 `boot`            increments once at boot (liveness)
/// - +0x08 `tx_count`        CyclicState frames transmitted
/// - +0x0C `rx_count`        CyclicState frames received + decoded
/// - +0x10 `last_src`        src node id of the last decoded CyclicState
/// - +0x14 `last_peer_pitch` pitch field of the last decoded peer CyclicState
/// - +0x18 `last_peer_wheel` wheel_speed field of the last decoded peer CyclicState
#[repr(C)]
pub struct Obs {
    pub magic: u32,
    pub boot: u32,
    pub tx_count: u32,
    pub rx_count: u32,
    pub last_src: u32,
    pub last_peer_pitch: i32,
    pub last_peer_wheel: i32,
}

impl Obs {
    const fn new() -> Self {
        Self {
            magic: 0,
            boot: 0,
            tx_count: 0,
            rx_count: 0,
            last_src: 0,
            last_peer_pitch: 0,
            last_peer_wheel: 0,
        }
    }
}

/// The fixed observation block. `#[no_mangle]` so its symbol address is stable and findable with
/// `nm`, and read over SWD. Written only from the single-core bare-metal main loop (no concurrency).
#[no_mangle]
pub static mut LINK_OBS: Obs = Obs::new();

/// Build the MCU descriptor for the selected family. Only the USART path is exercised, so the
/// non-USART fields are set to valid-but-unused values.
fn build_descriptor() -> McuDescriptor {
    let mut addrs = AddrTable::new();
    addrs.set(PeriphLabel::Usart1, USART1_BASE);
    addrs.set(PeriphLabel::Gpioa, target::GPIOA_BASE);
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
        gpio: target::GPIO_PATH,
        clock: target::CLOCK_PATH,
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

    // Stamp the observation block (liveness over SWD before anything else runs).
    unsafe {
        let obs = &mut *core::ptr::addr_of_mut!(LINK_OBS);
        obs.magic = OBS_MAGIC;
        obs.boot = obs.boot.wrapping_add(1);
    }

    // Bring up USART1 in the same order as M1: enable clocks, configure TX/RX AF, bring up the USART.
    let _ = enable_usart(RCU_BASE, desc.clock, PeriphLabel::Usart1);
    let _ = enable_gpio_port(RCU_BASE, desc.clock, PeriphLabel::Gpioa);
    configure_af(target::GPIOA_BASE, desc.gpio, PIN_PA2, PinRole::Tx);
    configure_af(target::GPIOA_BASE, desc.gpio, PIN_PA3, PinRole::Rx);
    let usart = Usart::bring_up(USART1_BASE, desc.clock, &desc.clock_cfg, UsartBus::Apb1, BAUD);

    // RX framer (the same stream framer the host tests cover), and TX scratch buffers.
    let mut framer = StreamFramer::new();
    let mut framebuf = [0u8; MAX_FRAME];
    let mut payload = [0u8; CyclicState::LEN];
    let mut tx_div: u32 = 0;

    loop {
        // --- Drain RX every pass: feed each ready byte to the framer; clear ORE on overrun ---
        let mut seen: Vec<(u8, i16, i16), 8> = Vec::new();
        loop {
            match usart.try_read_byte() {
                Ok(Some(b)) => {
                    framer.feed(&[b], &mut |f| {
                        if f.header.opcode == Opcode::CyclicState {
                            if let Ok(cs) = CyclicState::decode(f.payload) {
                                let _ = seen.push((f.header.src, cs.pitch, cs.wheel_speed));
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
        if !seen.is_empty() {
            unsafe {
                let obs = &mut *core::ptr::addr_of_mut!(LINK_OBS);
                for &(src, pitch, wheel) in seen.iter() {
                    obs.rx_count = obs.rx_count.wrapping_add(1);
                    obs.last_src = src as u32;
                    obs.last_peer_pitch = pitch as i32;
                    obs.last_peer_wheel = wheel as i32;
                }
            }
        }

        // --- Transmit a CyclicState frame, spaced out so the peer can drain it ---
        tx_div = tx_div.wrapping_add(1);
        if tx_div >= TX_INTERVAL {
            tx_div = 0;
            let state = CyclicState {
                pitch: target::MY_PITCH,
                roll: 0,
                wheel_speed: target::THIS_NODE_ID as i16,
                status: 0,
            };
            let n = state.encode(&mut payload);
            let hdr = FrameHeader {
                ver: PROTO_VER,
                opcode: Opcode::CyclicState,
                src: target::THIS_NODE_ID,
                dst: 0xFF,
                len: n as u8,
            };
            if let Ok(total) = encode(&hdr, &payload[..n], &mut framebuf) {
                for &b in &framebuf[..total] {
                    usart.write_byte(b);
                }
                unsafe {
                    let obs = &mut *core::ptr::addr_of_mut!(LINK_OBS);
                    obs.tx_count = obs.tx_count.wrapping_add(1);
                }
            }
        }
    }
}
