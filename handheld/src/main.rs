//! The handheld remote (`specs/handheld-remote.md`): a Ride screen on the ESP32-2432S028R "Cheap
//! Yellow Display", a BLE central to one hoverboard bridge.
//!
//! Tasks (section 6.2): the touch sampler on an interrupt executor above the main one, so a
//! display transfer never delays a sample; the BLE host and the link (framing, walk, ride facts,
//! inbox) and the view on the main executor. They share one [`Model`] behind a lock.
#![no_std]
#![no_main]
// The test-pattern build stops after the pattern and leaves the rest of the boot unused.
#![cfg_attr(feature = "test-pattern", allow(unused, unreachable_code))]

mod link;
mod panel;
mod xpt2046;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use esp_rtos::embassy::InterruptExecutor;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{ColorOrder, Orientation};
use static_cell::StaticCell;
use trouble_host::prelude::ExternalController;

use handheld_core::arm::Button;
use handheld_core::model::{Model, TouchDebug};
use handheld_core::touch::{median_spread, Lift, Touch, PERIOD_MS};

include!(concat!(env!("OUT_DIR"), "/bridge.rs"));

esp_bootloader_esp_idf::esp_app_desc!();

/// The one model every task reads and steps.
pub static MODEL: Mutex<CriticalSectionRawMutex, RefCell<Option<Model>>> =
    Mutex::new(RefCell::new(None));

/// Run `f` on the model.
pub fn with<R>(f: impl FnOnce(&mut Model) -> R) -> R {
    MODEL.lock(|m| f(m.borrow_mut().as_mut().expect("the model is set at boot")))
}

/// The time in ms, as the model counts it.
pub fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

/// The backlight's PWM duty, percent (section 2.2: below full, to keep the 5 V net up with the
/// radio running; measured on the bench, section 7.2 check 2).
const BACKLIGHT_PCT: u8 = 60;

type Panel = mipidsi::Display<
    SpiInterface<
        'static,
        ExclusiveDevice<
            esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Blocking>,
            Output<'static>,
            Delay,
        >,
        Output<'static>,
    >,
    ILI9341Rgb565,
    mipidsi::NoResetPin,
>;

static PANEL_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
static PWM_TIMER: StaticCell<esp_hal::ledc::timer::Timer<'static, LowSpeed>> = StaticCell::new();

/// Wakes the link when the pump has put frames in the outbox.
pub static PUMP: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// When the sample that released the throttle was taken (µs, low 32 bits; 0 for none): the link
/// logs the time to the frames it hands the BLE host (section 7.2 check 6).
pub static RELEASE_AT: AtomicU32 = AtomicU32::new(0);

/// The RGB LED's red and green, by PWM, active low.
pub struct Led {
    red: channel::Channel<'static, LowSpeed>,
    green: channel::Channel<'static, LowSpeed>,
}

impl Led {
    /// Orange while armed (red full, green about a third, section 5.3), off otherwise.
    fn set(&mut self, armed: bool) {
        let (r, g) = if armed { (0, 67) } else { (100, 100) };
        let _ = self.red.set_duty(r);
        let _ = self.green.set_duty(g);
    }
}
static SAMPLER: StaticCell<InterruptExecutor<1>> = StaticCell::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    println!(
        "handheld: booting, reset {:?}",
        esp_hal::system::reset_reason()
    );
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 96 * 1024);
    esp_alloc::heap_allocator!(size: 24 * 1024);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw.software_interrupt0);

    MODEL.lock(|m| m.replace(Some(Model::new(BRIDGE))));

    // The RGB LED is active low; blue stays off, red and green are PWM below.
    let _blue = Output::new(p.GPIO17, Level::High, OutputConfig::default());

    // The radio before the backlight (section 2.2).
    let connector = if BRIDGE.is_some() {
        match BleConnector::new(p.BT, Default::default()) {
            Ok(c) => Some(c),
            Err(e) => {
                println!("handheld: BLE controller failed: {:?}", e);
                None
            }
        }
    } else {
        None
    };
    let heap = esp_alloc::HEAP.free() as u32;
    println!("handheld: radio up, heap {} B free", heap);
    with(|m| m.heap_free = heap);

    // The display: SPI2 with DMA at 40 MHz, CS on IO15, DC on IO2, no reset pin.
    let (rx, rx_d, tx, tx_d) = esp_hal::dma_buffers!(32, 4096);
    let spi = Spi::new(
        p.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(p.GPIO14)
    .with_mosi(p.GPIO13)
    .with_miso(p.GPIO12)
    .with_dma(p.DMA_SPI2)
    .with_buffers(
        DmaRxBuf::new(rx_d, rx).unwrap(),
        DmaTxBuf::new(tx_d, tx).unwrap(),
    );
    let cs = Output::new(p.GPIO15, Level::High, OutputConfig::default());
    let dc = Output::new(p.GPIO2, Level::Low, OutputConfig::default());
    let dev = ExclusiveDevice::new(spi, cs, Delay::new()).unwrap();
    let di = SpiInterface::new(dev, dc, PANEL_BUF.init([0; 4096]));
    let mut delay = Delay::new();
    let mut panel: Panel = mipidsi::Builder::new(ILI9341Rgb565, di)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::new().flip_horizontal())
        .init(&mut delay)
        .unwrap();
    #[cfg(feature = "test-pattern")]
    panel::test_pattern(&mut panel);
    #[cfg(not(feature = "test-pattern"))]
    {
        use embedded_graphics::prelude::*;
        let _ = panel.clear(panel::color::BG);
    }

    // The backlight by PWM on IO21.
    let mut ledc = Ledc::new(p.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let pwm = PWM_TIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    pwm.configure(timer::config::Config {
        duty: timer::config::Duty::Duty8Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(20),
    })
    .unwrap();
    let pwm: &'static _ = pwm;
    let channel = |n, pin: esp_hal::gpio::AnyPin<'static>, duty_pct: u8| {
        let mut c = ledc.channel(n, pin);
        c.configure(channel::config::Config {
            timer: pwm,
            duty_pct,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .unwrap();
        c
    };
    let bl = channel(channel::Number::Channel0, p.GPIO21.into(), BACKLIGHT_PCT);
    // Active low: 100 percent duty is off.
    let led = Led {
        red: channel(channel::Number::Channel1, p.GPIO4.into(), 100),
        green: channel(channel::Number::Channel2, p.GPIO16.into(), 100),
    };
    println!("handheld: display up, backlight {} %", BACKLIGHT_PCT);

    // The touch controller: SPI3 through the GPIO matrix, mode 0, 1 MHz; PENIRQ on IO36.
    let tspi = Spi::new(
        p.SPI3,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(p.GPIO25)
    .with_mosi(p.GPIO32)
    .with_miso(p.GPIO39);
    let tcs = Output::new(p.GPIO33, Level::High, OutputConfig::default());
    let irq = Input::new(p.GPIO36, InputConfig::default().with_pull(Pull::None));
    let touch = xpt2046::Xpt2046::new(tspi, tcs, irq);

    // BOOT (IO0): R2 pulls it up; pressed is low.
    let boot = Input::new(p.GPIO0, InputConfig::default().with_pull(Pull::None));

    let sampler = SAMPLER.init(InterruptExecutor::new(sw.software_interrupt1));
    sampler
        .start(Priority::Priority1)
        .must_spawn(sample(touch, boot));

    // The test pattern stays on the panel: no view, no link.
    #[cfg(feature = "test-pattern")]
    loop {
        Timer::after(Duration::from_secs(3600)).await;
        let _ = (&bl, &panel, &led);
    }
    #[cfg(not(feature = "test-pattern"))]
    spawner.must_spawn(view(panel, led));
    if let Some(c) = connector {
        let controller: ExternalController<_, 20> = ExternalController::new(c);
        link::run(controller, BRIDGE.unwrap_or([0; 6])).await;
    }
    loop {
        Timer::after(Duration::from_secs(3600)).await;
        let _ = &bl;
    }
}

type TouchDev = xpt2046::Xpt2046<
    esp_hal::spi::master::Spi<'static, esp_hal::Blocking>,
    Output<'static>,
    Input<'static>,
>;

/// The touch sampler: every 2 ms on a timer, PENIRQ then the conversions, classified and
/// stepped through the lift machine; what it means goes to the model.
#[embassy_executor::task]
async fn sample(mut dev: TouchDev, boot: Input<'static>) {
    let mut touch = Touch::default();
    let mut button = Button::default();
    let mut ticker = Ticker::every(Duration::from_millis(PERIOD_MS as u64));
    let mut count = 0u16;
    let mut since = now_ms();
    let mut rate = 0u16;
    loop {
        ticker.next().await;
        let at = Instant::now();
        let r = dev.read();
        let now = now_ms();
        touch.sample(&r, now);
        count += 1;
        if now.wrapping_sub(since) >= 1000 {
            rate = count;
            count = 0;
            since = now;
        }
        let (mx, sx) = median_spread(r.raw.x);
        let (my, sy) = median_spread(r.raw.y);
        let debug = TouchDebug {
            state: match touch.lift {
                Lift::Idle => b'I',
                Lift::Pressing { .. } => b'P',
                Lift::Down { .. } => b'D',
                Lift::Lifting { .. } => b'L',
            },
            z: if r.pen_down { r.raw.z() } else { 0 },
            x: (mx, sx),
            y: (my, sy),
            rate,
        };
        let v = touch.view(now);
        let edge = button.poll(boot.is_low(), now);
        let fault = button.fault();
        let sent = with(|m| {
            m.touch = v;
            m.touch_debug = debug;
            if let Some(e) = edge {
                m.on_button(e);
            }
            if m.button_fault && !fault {
                m.button_released();
            }
            let sent = m.run_pump(now);
            // Under the lock, with the frames it stamps: the link takes both together.
            if m.take_release() {
                RELEASE_AT.store((at.as_micros() as u32).max(1), Ordering::Relaxed);
            }
            sent
        });
        if sent {
            PUMP.signal(());
        }
    }
}

/// One short log line of the view.
fn log_view(v: &handheld_core::model::View, debug: &(heapless::String<40>, heapless::String<40>)) {
    use handheld_core::model::{Body, Lower};
    match &v.body {
        Body::Card(c) => println!("view: card {:?}", c),
        Body::Ride(r) => {
            let phase = match &r.lower {
                Lower::Bars { phase, band } => {
                    let t = |p: &Option<handheld_core::model::PhaseBar>| {
                        p.as_ref()
                            .map(|b| (b.label.clone(), b.text.clone(), b.fill))
                    };
                    println!(
                        "view: phase {:?} {:?} band {:?}",
                        t(&phase[0]),
                        t(&phase[1]),
                        band
                    );
                    ""
                }
                Lower::Grid(cells) => {
                    println!("view: lamp grid, {} lamps", cells.len());
                    ""
                }
            };
            println!(
                "view: ring {} lit{} speed {:?} pack {:?}{}",
                r.dial.lit,
                if r.dial.caution { " caution" } else { "" },
                r.dial.speed,
                r.pack.text,
                phase
            );
            for c in r.corners.iter().flatten() {
                let (a, b) = c.attitude.lines();
                println!("view: corner {} {} {}", c.label, a, b);
            }
        }
    }
    println!(
        "view: top {:?} line {:?} debug {} | {}",
        v.top,
        v.line.as_ref().map(|s| s.text.as_str()),
        debug.0,
        debug.1
    );
}

/// The view: at most 25 redraws a second, each drawing only what changed.
#[embassy_executor::task]
async fn view(mut panel: Panel, mut led: Led) {
    let mut cache = panel::Cache::default();
    let mut drawn_overlay: Option<handheld_core::model::Overlay> = None;
    let mut led_armed = false;
    let mut ticker = Ticker::every(Duration::from_millis(40));
    let mut logged = 0u32;
    loop {
        ticker.next().await;
        let now = now_ms();
        let (v, debug, overlay) = with(|m| (m.view(now), m.debug_lines(), m.overlay()));
        // A line of what the screen shows every 5 s, for the serial log (section 6.4).
        if now.wrapping_sub(logged) >= 5000 {
            logged = now;
            log_view(&v, &debug);
        }
        let dbg = cfg!(debug_assertions) || cfg!(feature = "debug-line");
        let d = dbg.then_some((debug.0.as_str(), debug.1.as_str()));
        if drawn_overlay != overlay {
            if let Some(old) = drawn_overlay {
                for (y0, y1) in panel::overlay_bands(&old) {
                    panel::redraw_band(&mut panel, &v, d, y0, y1);
                }
            }
            drawn_overlay = overlay;
        }
        panel::draw(&mut panel, &v, d, &mut cache);
        if let Some(o) = &overlay {
            panel::draw_overlay(&mut panel, o);
        }
        if led_armed != v.armed {
            led_armed = v.armed;
            led.set(v.armed);
        }
    }
}
