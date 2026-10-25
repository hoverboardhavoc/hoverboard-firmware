//! The ESP32-C3 board tool (`specs/esp-tool.md`): the SWD engine, board operations and flashing,
//! the mailbox port, and the controller links, BLE and the console (USB or TCP over Wi-Fi), each
//! carrying the ESP commands and forwarding every other frame between its controller and the
//! board's mailbox.
//!
//! The tasks share one executor: the BLE host (`ble`), the USB console (`usb`), the Wi-Fi station,
//! its IP stack, the TCP console and the mDNS responder (`wifi`), and the service loop below,
//! which owns the SWD engine (through the mailbox port), the router and the command service. The
//! service loop busy-polls the mailbox and yields between passes.
#![no_std]
#![no_main]

mod ble;
mod pins;
mod platform;
mod usb;
mod wifi;

use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_radio::ble::controller::BleConnector;
use static_cell::{ConstStaticCell, StaticCell};
use trouble_host::prelude::ExternalController;

use esp_tool_core::mailbox::MailboxPort;
use esp_tool_core::router::{Port, Router, Via, NOTIFY_MAX};
use esp_tool_core::svc::{Build, ImageBuffer, Service};
use esp_tool_core::swd::{Swd, SwdConfig};

use crate::ble::{BleEvent, EVENTS, NOTIFY};
use crate::pins::EspPins;
use crate::platform::EspPlatform;
use crate::usb::UsbEvent;
use crate::wifi::TcpEvent;

esp_bootloader_esp_idf::esp_app_desc!();

/// The flash transfer buffer, placed statically (63,488 B; never on a stack).
static IMAGE: ConstStaticCell<ImageBuffer> = ConstStaticCell::new(ImageBuffer::new());
/// The name: `hb-esp-` and the last four hex digits of the MAC. BLE advertises it and mDNS
/// answers for `<name>.local`.
static NAME: StaticCell<[u8; 11]> = StaticCell::new();
/// The IP stack's sockets: DHCP, the TCP console, mDNS.
static NET: StaticCell<StackResources<3>> = StaticCell::new();

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
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    // Wi-Fi and BLE together need more heap than BLE alone; the RAM the second-stage bootloader
    // used is free once the app runs.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mac = esp_hal::efuse::Efuse::read_base_mac_address();
    let name = name(mac);

    // The radio: BLE first, then Wi-Fi, which share it through esp-radio's coexistence.
    let connector = BleConnector::new(p.BT, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let (wifi, interfaces) = esp_radio::wifi::new(p.WIFI, Default::default()).unwrap();
    let rng = Rng::new();
    let seed = u64::from(rng.random()) << 32 | u64::from(rng.random());
    let (stack, runner) = embassy_net::new(
        interfaces.station,
        embassy_net::Config::dhcpv4(Default::default()),
        NET.init(StackResources::new()),
        seed,
    );
    spawner.must_spawn(wifi::station(wifi));
    spawner.must_spawn(wifi::net(runner));
    spawner.must_spawn(wifi::console(stack));
    spawner.must_spawn(wifi::mdns(stack, name, rng));

    let mut swd = Swd::new(EspPins::new(p.GPIO6, p.GPIO7), SwdConfig::default());
    let khz = swd.measure_khz(20_000);
    let svc = Service::new(build(khz), IMAGE.take(), EspPlatform::new(stack));
    spawner.must_spawn(service(MailboxPort::new(swd), svc));
    spawner.must_spawn(usb::run(UsbSerialJtag::new(p.USB_DEVICE).into_async()));

    ble::run(controller, name, mac).await;
}

/// Move up to a channel's room of `router`'s bytes for `port` into `send`, `N` at a time.
fn drain<const N: usize>(
    router: &mut Router,
    port: Port,
    room: impl Fn() -> bool,
    mut send: impl FnMut(heapless09::Vec<u8, N>),
) {
    let mut buf = [0u8; N];
    while room() {
        let k = router.next_out(port, &mut buf);
        if k == 0 {
            break;
        }
        send(heapless09::Vec::from_slice(&buf[..k]).unwrap_or_default());
    }
}

/// The service loop: controller bytes in, routing, the mailbox, the commands, controller bytes
/// out.
#[embassy_executor::task]
async fn service(mut port: MailboxPort<EspPins>, mut svc: Service<'static, EspPlatform>) {
    let mut router = Router::new();
    loop {
        while let Ok(ev) = EVENTS.try_receive() {
            match ev {
                BleEvent::Connected => {
                    router.open(Via::Ble);
                }
                BleEvent::Disconnected => router.gone(Via::Ble),
                BleEvent::Data(d) => {
                    router.on_bytes(Via::Ble, &d);
                }
            }
        }
        while let Ok(ev) = usb::EVENTS.try_receive() {
            match ev {
                // Dropped while the console is attached on TCP.
                UsbEvent::Data(d) => {
                    router.on_bytes(Via::Usb, &d);
                }
                UsbEvent::Resync => router.drop_partial(Via::Usb),
                UsbEvent::Gone => router.gone(Via::Usb),
            }
        }
        while let Ok(ev) = wifi::EVENTS.try_receive() {
            match ev {
                TcpEvent::Open => wifi::ADMIT.signal(router.open(Via::Tcp)),
                TcpEvent::Data(d) => {
                    router.on_bytes(Via::Tcp, &d);
                }
                TcpEvent::Gone => router.gone(Via::Tcp),
            }
        }
        router.pump(&mut port, &mut svc);
        drain::<NOTIFY_MAX>(
            &mut router,
            Port::Ble,
            || !NOTIFY.is_full(),
            |v| {
                let _ = NOTIFY.try_send(v);
            },
        );
        match router.console() {
            Some(Via::Usb) => drain::<{ usb::PACKET }>(
                &mut router,
                Port::Console,
                || !usb::OUT.is_full(),
                |v| {
                    let _ = usb::OUT.try_send(v);
                },
            ),
            Some(Via::Tcp) => drain::<{ wifi::PACKET }>(
                &mut router,
                Port::Console,
                || !wifi::OUT.is_full(),
                |v| {
                    let _ = wifi::OUT.try_send(v);
                },
            ),
            _ => {}
        }
        Timer::after(Duration::from_micros(200)).await;
    }
}
