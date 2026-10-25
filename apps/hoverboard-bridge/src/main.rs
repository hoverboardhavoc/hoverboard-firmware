//! The hoverboard bridge (`specs/hoverboard-bridge.md`): the SWD engine, board operations and flashing,
//! the mailbox port, the controller links (BLE and the console, on USB or TCP over Wi-Fi), and the
//! bridge as an L3 node between them, with its keys in its own store (`keys`).
//!
//! The tasks share one executor: the BLE host (`ble`), the USB console (`usb`), the Wi-Fi station,
//! its IP stack, the TCP console, the GDB listener and the mDNS responder (`wifi`), and the
//! service loop below, which owns the SWD engine (through the mailbox port), the links' streams,
//! the node, the command service and the GDB server. The service loop busy-polls the mailbox (and,
//! while a GDB session lets the core run, `DHCSR`) and yields between passes.
#![no_std]
#![no_main]

/// One line on the USB console at a boot step, with the `boot-log` feature; nothing (and no
/// evaluation of the arguments) without it.
macro_rules! boot {
    ($($t:tt)*) => {{
        #[cfg(feature = "boot-log")]
        log::info!($($t)*);
    }};
}

mod ble;
mod keys;
mod pins;
mod platform;
mod usb;
mod wifi;

use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use static_cell::{ConstStaticCell, StaticCell};
use trouble_host::prelude::ExternalController;

use esp_bootloader_esp_idf::partitions::PARTITION_TABLE_MAX_LEN;
use hoverboard_bridge_core::gdb::Gdb;
use hoverboard_bridge_core::mailbox::MailboxPort;
use hoverboard_bridge_core::node::Node;
use hoverboard_bridge_core::router::{Port, Router, NOTIFY_MAX};
#[cfg(feature = "boot-log")]
use hoverboard_bridge_core::svc::Platform;
use hoverboard_bridge_core::svc::{Build, ImageBuffer, Service};
use hoverboard_bridge_core::swd::{Swd, SwdConfig};
use hoverboard_bridge_core::update::Update;

use crate::ble::{BleEvent, EVENTS, NOTIFY};
use crate::keys::KeysFlash;
use crate::pins::EspPins;
use crate::platform::EspPlatform;
use crate::usb::UsbEvent;
use crate::wifi::TcpEvent;

esp_bootloader_esp_idf::esp_app_desc!();

/// The flash transfer buffer, placed statically (63,488 B; never on a stack).
static IMAGE: ConstStaticCell<ImageBuffer> = ConstStaticCell::new(ImageBuffer::new());
/// The name BLE advertises and mDNS answers for (`<name>.local`): the store's `device.name`, read
/// at boot.
static NAME: StaticCell<[u8; NAME_MAX]> = StaticCell::new();
/// The longest name used: what fits the advertising packet beside its flags (31 B less 3 for the
/// flags and 2 for the name's own header).
const NAME_MAX: usize = 26;
/// The store's region, mirrored in RAM (`keys`).
static KEYS_MIRROR: ConstStaticCell<[u8; keys::REGION]> = ConstStaticCell::new([0; keys::REGION]);
/// The store's flash.
static KEYS: StaticCell<KeysFlash> = StaticCell::new();
/// The bridge as a node, with its store.
static NODE: StaticCell<Node<'static, KeysFlash>> = StaticCell::new();
/// How long the service loop runs on after an update before it resets into the new slot, so
/// the update's last replies reach the client.
const REBOOT_GRACE: Duration = Duration::from_millis(500);
/// The bridge's own update, with its 4 KiB sector buffer.
static UPDATE: StaticCell<Update> = StaticCell::new();
/// The partition table, read for the update and the running slot.
static TABLE: ConstStaticCell<[u8; PARTITION_TABLE_MAX_LEN]> =
    ConstStaticCell::new([0; PARTITION_TABLE_MAX_LEN]);
/// The router: its per-port stream buffers are kilobytes.
static ROUTER: StaticCell<Router> = StaticCell::new();
/// The command service.
static SERVICE: StaticCell<Service<'static, EspPlatform>> = StaticCell::new();
/// The mailbox port, with the SWD engine.
static PORT: StaticCell<MailboxPort<EspPins>> = StaticCell::new();
/// The IP stack's sockets: DHCP, the TCP console's two and the GDB listener's two (one serving
/// while the other listens), mDNS.
static NET: StaticCell<StackResources<6>> = StaticCell::new();

fn build(swd_khz: u32) -> Build {
    let v = |s: &str| s.parse::<u8>().unwrap_or(0);
    Build {
        version: [
            v(env!("CARGO_PKG_VERSION_MAJOR")),
            v(env!("CARGO_PKG_VERSION_MINOR")),
            v(env!("CARGO_PKG_VERSION_PATCH")),
        ],
        commit: u32::from_str_radix(env!("HOVERBOARD_BRIDGE_COMMIT"), 16).unwrap_or(0),
        swd_khz: swd_khz.min(u16::MAX as u32) as u16,
    }
}

/// The name until one is written: `hb-bridge-` and the last four hex digits of the MAC.
fn default_name(mac: [u8; 6]) -> [u8; 14] {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut buf = *b"hb-bridge-0000";
    for (i, b) in [mac[4], mac[5]].iter().enumerate() {
        buf[10 + 2 * i] = HEX[(b >> 4) as usize];
        buf[11 + 2 * i] = HEX[(b & 0xF) as usize];
    }
    buf
}

/// A written name the bridge can advertise and answer mDNS for: [`NAME_MAX`] bytes at most, of
/// letters, digits and `-`.
fn usable(name: &[u8]) -> bool {
    !name.is_empty()
        && name.len() <= NAME_MAX
        && name.iter().all(|b| b.is_ascii_alphanumeric() || *b == b'-')
}

/// The name, from `device.name` in the store. A bridge whose store holds none writes its default
/// name there, so a `CONFIG_READ` of the key answers what BLE advertises. A written name that
/// cannot be advertised (too long, or characters a host name cannot carry) stays in the store and
/// the default is used.
fn name(store: &mut store::Store<'_, KeysFlash>, mac: [u8; 6]) -> &'static str {
    let buf = NAME.init([0; NAME_MAX]);
    let mut written: Option<usize> = None;
    store.for_each_in(&[store::DEVICE_NAME.key()], |_, tag, v| {
        written = (tag == store::Type::Str.tag()).then(|| {
            let n = v.len().min(NAME_MAX);
            buf[..n].copy_from_slice(&v[..n]);
            if usable(v) {
                n
            } else {
                0
            }
        });
    });
    let n = match written {
        Some(n) if n > 0 => n,
        found => {
            let d = default_name(mac);
            buf[..d.len()].copy_from_slice(&d);
            if found.is_none() {
                let _ = store.set_str(
                    store::DEVICE_NAME,
                    core::str::from_utf8(&d).unwrap_or_default(),
                );
            }
            d.len()
        }
    };
    core::str::from_utf8(&buf[..n]).unwrap_or("hb-bridge")
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    #[cfg(feature = "boot-log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);
    boot!(
        "hoverboard-bridge {} {} booting",
        env!("CARGO_PKG_VERSION"),
        env!("HOVERBOARD_BRIDGE_COMMIT")
    );
    // Wi-Fi and BLE together need more heap than BLE alone; the RAM the second-stage bootloader
    // used is free once the app runs.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
    boot!("scheduler started, heap {} B free", esp_alloc::HEAP.free());

    let mac = esp_hal::efuse::Efuse::read_base_mac_address();

    // The store, on the `keys` partition. The app slots' platform below has its own handle on the
    // flash: both are used from the service loop alone, never at once.
    let table = TABLE.take();
    // SAFETY: `FLASH` is only a token for the ROM's SPI flash functions; this second handle and the
    // platform's are both used from the service loop, one call at a time.
    let keys_flash = FlashStorage::new(unsafe { esp_hal::peripherals::FLASH::steal() });
    let keys = KEYS.init(KeysFlash::load(keys_flash, table, KEYS_MIRROR.take()));
    if store::Store::mount(&mut *keys).is_err() {
        keys.ram_only();
    }
    let mounted = keys.persists();
    let Ok(mut store) = store::Store::mount(keys) else {
        // An erased region always mounts; nothing to run on otherwise.
        panic!("the store did not mount on an erased region")
    };
    let name = name(&mut store, mac);
    boot!(
        "store {}, name {}",
        if mounted { "mounted" } else { "NOT mounted" },
        name
    );

    // The SWD side and the USB console first: they need no radio, and a radio that fails to come
    // up leaves them working.
    let mut swd = Swd::new(EspPins::new(p.GPIO6, p.GPIO7), SwdConfig::default());
    let khz = swd.measure_khz(20_000);
    boot!("SWD clock {} kHz", khz);
    // The service's state is placed in statics, not moved by value into the task: a task's
    // arguments pass through the stack of the spawning task, and with the update's sector buffer
    // that overran the 35 KB stack and stopped the boot silently.
    let platform = EspPlatform::new(FlashStorage::new(p.FLASH), table);
    let build = build(khz);
    let fw_ver = u16::from_be_bytes([build.version[0], build.version[1]]);
    let node = NODE.init(Node::new(store, mounted, fw_ver));
    let svc = SERVICE.init(Service::new(
        build,
        IMAGE.take(),
        UPDATE.init(Update::new()),
        platform,
    ));
    let port = PORT.init(MailboxPort::new(swd));
    spawner.must_spawn(service(port, svc, ROUTER.init(Router::new()), node));
    spawner.must_spawn(usb::run(UsbSerialJtag::new(p.USB_DEVICE).into_async()));
    // Let them start before the radio's blocking initialisation.
    Timer::after(Duration::from_millis(1)).await;

    // The radio: BLE first, then Wi-Fi, which share it through esp-radio's coexistence.
    boot!("radio: BLE controller");
    let connector = match BleConnector::new(p.BT, Default::default()) {
        Ok(c) => Some(c),
        Err(_e) => {
            boot!("radio: BLE controller failed: {:?}", _e);
            None
        }
    };
    boot!("radio: Wi-Fi driver");
    match esp_radio::wifi::new(p.WIFI, Default::default()) {
        Ok((wifi, interfaces)) => {
            let rng = Rng::new();
            let seed = u64::from(rng.random()) << 32 | u64::from(rng.random());
            let (stack, runner) = embassy_net::new(
                interfaces.station,
                embassy_net::Config::dhcpv4(Default::default()),
                NET.init(StackResources::new()),
                seed,
            );
            spawner.must_spawn(wifi::watch(stack));
            spawner.must_spawn(wifi::station(wifi));
            spawner.must_spawn(wifi::net(runner));
            spawner.must_spawn(wifi::console(stack));
            spawner.must_spawn(wifi::gdb(stack));
            spawner.must_spawn(wifi::mdns(stack, name, rng));
            boot!(
                "radio: Wi-Fi tasks started, heap {} B free",
                esp_alloc::HEAP.free()
            );
        }
        Err(_e) => {
            boot!("radio: Wi-Fi driver failed: {:?}", _e);
        }
    }

    match connector {
        Some(c) => {
            let controller: ExternalController<_, 20> = ExternalController::new(c);
            ble::run(controller, name, mac).await
        }
        None => core::future::pending().await,
    }
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

/// The service loop: controller bytes in, the node (the walk, forwarding, the mailbox, the
/// commands), the GDB server, controller and GDB bytes out.
#[embassy_executor::task]
async fn service(
    port: &'static mut MailboxPort<EspPins>,
    svc: &'static mut Service<'static, EspPlatform>,
    router: &'static mut Router,
    node: &'static mut Node<'static, KeysFlash>,
) {
    boot!("running {:?}", svc.platform().running());
    boot!("service loop and USB console ready");
    // The GDB server holds the engine's owner and the service; the node borrows them from it.
    let mut gdb = Gdb::new(port, svc);
    let mut reboot_at = None;
    loop {
        while let Ok(ev) = EVENTS.try_receive() {
            match ev {
                BleEvent::Connected => router.connect(Port::Ble),
                BleEvent::Disconnected => router.disconnect(Port::Ble),
                BleEvent::Data(d) => router.on_bytes(Port::Ble, &d),
            }
        }
        // The consoles' bytes only while the port has room for a whole read: the rest wait in the
        // channel, the carrier's buffers and its flow control (a bulk window of 64 chunks is more
        // than the port holds).
        while router.room(Port::Usb) >= usb::PACKET {
            let Ok(ev) = usb::EVENTS.try_receive() else {
                break;
            };
            match ev {
                UsbEvent::Data(d) => router.on_bytes(Port::Usb, &d),
                UsbEvent::Resync => router.drop_partial(Port::Usb),
            }
        }
        while router.room(Port::Tcp) >= wifi::PACKET {
            let Ok(ev) = wifi::EVENTS.try_receive() else {
                break;
            };
            match ev {
                TcpEvent::Open => router.connect(Port::Tcp),
                TcpEvent::Data(d) => router.on_bytes(Port::Tcp, &d),
                TcpEvent::Gone => router.disconnect(Port::Tcp),
            }
        }
        // GDB's bytes only as far as its queue has room: the rest wait in the channel.
        while gdb.room() >= wifi::PACKET {
            let Ok(ev) = wifi::GDB_EVENTS.try_receive() else {
                break;
            };
            match ev {
                TcpEvent::Open => gdb.open(),
                TcpEvent::Data(d) => {
                    gdb.push(&d);
                }
                TcpEvent::Gone => gdb.close(),
            }
        }
        let (port, svc) = gdb.parts();
        node.pump(router, port, svc);
        gdb.poll();
        while !wifi::GDB_OUT.is_full() {
            let mut buf = [0u8; wifi::PACKET];
            let n = gdb.take_out(&mut buf);
            if n == 0 {
                break;
            }
            let _ =
                wifi::GDB_OUT.try_send(heapless09::Vec::from_slice(&buf[..n]).unwrap_or_default());
        }
        if gdb.take_ended() {
            wifi::GDB_CLOSE.signal(());
        }
        drain::<NOTIFY_MAX>(
            router,
            Port::Ble,
            || !NOTIFY.is_full(),
            |v| {
                let _ = NOTIFY.try_send(v);
            },
        );
        drain::<{ usb::PACKET }>(
            router,
            Port::Usb,
            || !usb::OUT.is_full(),
            |v| {
                let _ = usb::OUT.try_send(v);
            },
        );
        drain::<{ wifi::PACKET }>(
            router,
            Port::Tcp,
            || !wifi::OUT.is_full(),
            |v| {
                let _ = wifi::OUT.try_send(v);
            },
        );
        // An update's slot is the next boot's: the loop runs on until the last replies are out,
        // then resets into it.
        if gdb.parts().1.reboot_requested() {
            let at = *reboot_at.get_or_insert_with(|| Instant::now() + REBOOT_GRACE);
            if Instant::now() >= at {
                esp_hal::system::software_reset();
            }
        }
        Timer::after(Duration::from_micros(200)).await;
    }
}
