//! The ESP32-C3 board tool (`specs/esp-tool.md`), round 1: the SWD engine, board operations and
//! flashing, the mailbox port, and the BLE link carrying the ESP commands and forwarding every
//! other frame between the central and the board's mailbox.
//!
//! Two tasks share one executor: the BLE host (`ble`) and the service loop below, which owns the
//! SWD engine (through the mailbox port), the router and the command service. The service loop
//! busy-polls the mailbox and yields between passes.
#![no_std]
#![no_main]

mod ble;
mod pins;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use static_cell::{ConstStaticCell, StaticCell};
use trouble_host::prelude::ExternalController;

use esp_tool_core::ble::{Router, NOTIFY_MAX};
use esp_tool_core::mailbox::MailboxPort;
use esp_tool_core::svc::{Build, ImageBuffer, Service};
use esp_tool_core::swd::{Swd, SwdConfig};

use crate::ble::{BleEvent, EVENTS, NOTIFY};
use crate::pins::EspPins;

esp_bootloader_esp_idf::esp_app_desc!();

/// The flash transfer buffer, placed statically (63,488 B; never on a stack).
static IMAGE: ConstStaticCell<ImageBuffer> = ConstStaticCell::new(ImageBuffer::new());
/// The advertised name, `hb-esp-` and the last four hex digits of the MAC.
static NAME: StaticCell<[u8; 11]> = StaticCell::new();

fn build(swd_khz: u32) -> Build {
    let v = |s: &str| s.parse::<u8>().unwrap_or(0);
    Build {
        version: [
            v(env!("CARGO_PKG_VERSION_MAJOR")),
            v(env!("CARGO_PKG_VERSION_MINOR")),
            v(env!("CARGO_PKG_VERSION_PATCH")),
        ],
        commit: u32::from_str_radix(env!("ESP_TOOL_COMMIT"), 16).unwrap_or(0),
        swd_khz: swd_khz.min(u16::MAX as u32) as u16,
    }
}

fn name(mac: [u8; 6]) -> &'static str {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let buf = NAME.init(*b"hb-esp-0000");
    for (i, b) in [mac[4], mac[5]].iter().enumerate() {
        buf[7 + 2 * i] = HEX[(b >> 4) as usize];
        buf[8 + 2 * i] = HEX[(b & 0xF) as usize];
    }
    core::str::from_utf8(buf).unwrap_or("hb-esp")
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut swd = Swd::new(EspPins::new(p.GPIO6, p.GPIO7), SwdConfig::default());
    let khz = swd.measure_khz(20_000);
    esp_println::println!("esp-tool: SWD clock {} kHz", khz);
    let svc = Service::new(build(khz), IMAGE.take());
    spawner.must_spawn(service(MailboxPort::new(swd), svc));

    let mac = esp_hal::efuse::Efuse::read_base_mac_address();
    let connector = BleConnector::new(p.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    ble::run(controller, name(mac), mac).await;
}

/// The service loop: BLE bytes in, routing, the mailbox, the commands, notifications out.
#[embassy_executor::task]
async fn service(mut port: MailboxPort<EspPins>, mut svc: Service<'static>) {
    let mut router = Router::new();
    loop {
        while let Ok(ev) = EVENTS.try_receive() {
            match ev {
                BleEvent::Connected => router.connect(),
                BleEvent::Disconnected => router.disconnect(),
                BleEvent::Data(d) => router.on_write(&d),
            }
        }
        router.pump(&mut port, &mut svc);
        let mut n = [0u8; NOTIFY_MAX];
        while !NOTIFY.is_full() {
            let k = router.next_notification(&mut n);
            if k == 0 {
                break;
            }
            let _ = NOTIFY.try_send(heapless09::Vec::from_slice(&n[..k]).unwrap_or_default());
        }
        Timer::after(Duration::from_micros(200)).await;
    }
}
