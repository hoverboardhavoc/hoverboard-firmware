//! The handheld remote (`specs/handheld-remote.md`). Build step 1 (section 6.5): the BLE central
//! alone. It connects to the pinned bridge with the parameters of section 3.4, subscribes to `tx`,
//! attaches with `NODE_HELLO`, and prints the `CYCLIC_STATE` sources and rates it receives.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use heapless::Vec;
use trouble_host::prelude::*;

use handheld_core::wire::Wire;
use net::pdu::Pdu;
use net::walk::Controller as Walk;

include!(concat!(env!("OUT_DIR"), "/bridge.rs"));

esp_bootloader_esp_idf::esp_app_desc!();

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;

/// How long a `NODE_HELLO` waits for its reply before it is sent again (the app's `L3Session`).
const REPLY_WAIT: Duration = Duration::from_secs(1);
/// How often the sources and rates are printed.
const REPORT: Duration = Duration::from_secs(10);
/// The wait between a failed or lost connection and the next attempt (section 3.4).
const RETRY: Duration = Duration::from_millis(500);

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    println!("handheld: booting");
    // The second-stage bootloader's RAM is free once the app runs (the fork's ESP32 coexistence
    // example's split).
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 96 * 1024);
    esp_alloc::heap_allocator!(size: 24 * 1024);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let Some(bridge) = BRIDGE else {
        println!("handheld: No bridge set.");
        loop {
            Timer::after(Duration::from_secs(3600)).await;
        }
    };
    println!(
        "handheld: bridge {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        bridge[0], bridge[1], bridge[2], bridge[3], bridge[4], bridge[5]
    );

    let connector = match BleConnector::new(p.BT, Default::default()) {
        Ok(c) => c,
        Err(e) => {
            println!("handheld: BLE controller failed: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(3600)).await;
            }
        }
    };
    println!(
        "handheld: BLE controller up, heap {} B free",
        esp_alloc::HEAP.free()
    );
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // Our own static random address, from the MAC as the bridge makes its own.
    let mut own = esp_hal::efuse::Efuse::read_base_mac_address();
    own.reverse();
    own[5] |= 0xC0;
    let mut target = bridge;
    target.reverse();

    run(controller, Address::random(own), Address::random(target)).await;
}

async fn run<C: Controller>(controller: C, own: Address, target: Address) {
    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(own)
        .build();
    let mut runner = stack.runner();
    let mut central = stack.central();

    let accept = [target];
    let config = ConnectConfig {
        connect_params: RequestedConnParams {
            min_connection_interval: Duration::from_micros(7_500),
            max_connection_interval: Duration::from_millis(15),
            max_latency: 0,
            min_event_length: Duration::from_secs(0),
            max_event_length: Duration::from_secs(0),
            supervision_timeout: Duration::from_millis(400),
        },
        scan_config: ScanConfig {
            filter_accept_list: &accept,
            ..Default::default()
        },
    };

    let _ = join(
        async {
            loop {
                if let Err(e) = runner.run().await {
                    println!("handheld: BLE host runner failed: {:?}", e);
                    Timer::after(Duration::from_millis(200)).await;
                }
            }
        },
        async {
            loop {
                let started = Instant::now();
                println!("handheld: connecting");
                match central.connect(&config).await {
                    Ok(conn) => {
                        println!(
                            "handheld: connected after {} ms",
                            started.elapsed().as_millis()
                        );
                        session(&stack, &conn).await;
                        println!("handheld: disconnected");
                    }
                    Err(e) => println!("handheld: connect failed: {:?}", e),
                }
                Timer::after(RETRY).await;
            }
        },
    )
    .await;
}

/// One connection: discover, subscribe, attach, count. Returns when the link is lost or a step
/// fails.
async fn session<C: Controller>(
    stack: &Stack<'_, C, DefaultPacketPool>,
    conn: &Connection<'_, DefaultPacketPool>,
) {
    let client = match GattClient::<C, DefaultPacketPool, 4>::new(stack, conn).await {
        Ok(c) => c,
        Err(e) => {
            println!("handheld: GATT client failed: {:?}", e);
            return;
        }
    };
    let params = conn.params();
    println!(
        "handheld: interval {} us, latency {}, supervision {} ms, ATT MTU {}",
        params.conn_interval.as_micros(),
        params.peripheral_latency,
        params.supervision_timeout.as_millis(),
        conn.att_mtu()
    );
    let _ = select(client.task(), async {
        let services = match client
            .services_by_uuid(&Uuid::from(espsvc::LINK_SERVICE_UUID))
            .await
        {
            Ok(s) => s,
            Err(e) => {
                println!("handheld: service discovery failed: {:?}", e);
                return;
            }
        };
        let Some(service) = services.first().cloned() else {
            println!("handheld: no link service");
            return;
        };
        let rx: Characteristic<[u8]> = match client
            .characteristic_by_uuid(&service, &Uuid::from(espsvc::LINK_RX_UUID))
            .await
        {
            Ok(c) => c,
            Err(e) => {
                println!("handheld: rx not found: {:?}", e);
                return;
            }
        };
        let tx: Characteristic<[u8]> = match client
            .characteristic_by_uuid(&service, &Uuid::from(espsvc::LINK_TX_UUID))
            .await
        {
            Ok(c) => c,
            Err(e) => {
                println!("handheld: tx not found: {:?}", e);
                return;
            }
        };
        let mut listener = match client.subscribe(&tx, false).await {
            Ok(l) => l,
            Err(e) => {
                println!("handheld: subscribe failed: {:?}", e);
                return;
            }
        };
        println!("handheld: subscribed");
        let chunk_max = (conn.att_mtu() as usize).saturating_sub(3).clamp(20, 244);

        let mut wire = Wire::new();
        let mut walk = Walk::new();
        let mut hello_at: Option<Instant> = None;
        let mut sources: Vec<(u8, u32), 8> = Vec::new();
        let mut since = Instant::now();
        let mut buf = [0u8; 256];
        let mut chunk = [0u8; 244];
        let mut tick = Instant::now();
        let (mut notes, mut bytes, mut pdus) = (0u32, 0u32, 0u32);
        loop {
            match select(listener.next(), Timer::at(tick + Duration::from_millis(10))).await {
                Either::First(n) => {
                    notes += 1;
                    bytes += n.as_ref().len() as u32;
                    wire.feed(n.as_ref())
                }
                Either::Second(_) => tick = Instant::now(),
            }

            while let Some(pdu) = wire.recv(&mut buf) {
                pdus += 1;
                if let Some(r) = walk.reply_to_probe(pdu) {
                    wire.send(&r);
                    continue;
                }
                if walk.granted().is_none() && walk.on_reply(pdu) {
                    println!("handheld: attached as {:#04x}", walk.guest_addr());
                    hello_at = None;
                    continue;
                }
                if let Ok(p) = Pdu::decode(pdu) {
                    if p.opcode == linkctl::OP_CYCLIC_STATE {
                        let ok = linkctl::CyclicState::decode(p.payload).is_ok();
                        if ok {
                            match sources.iter_mut().find(|(s, _)| *s == p.src) {
                                Some((_, n)) => *n += 1,
                                None => {
                                    let _ = sources.push((p.src, 1));
                                }
                            }
                        }
                    }
                }
            }

            if walk.granted().is_none() {
                let due = hello_at.is_none_or(|t| t.elapsed() > REPLY_WAIT);
                if due {
                    // A resend of the first contact is a fresh `NODE_HELLO` from the same guest.
                    let mut w = Walk::new();
                    if let Some(req) = w.next_request() {
                        walk = w;
                        wire.send(&req);
                        hello_at = Some(Instant::now());
                    }
                }
            }

            while wire.pending() > 0 {
                let n = wire.take(&mut chunk[..chunk_max]);
                if let Err(e) = client
                    .write_characteristic_without_response(&rx, &chunk[..n])
                    .await
                {
                    println!("handheld: write failed: {:?}", e);
                    return;
                }
            }

            if since.elapsed() >= REPORT {
                let secs = since.elapsed().as_millis() as f32 / 1000.0;
                for (src, n) in sources.iter() {
                    println!(
                        "handheld: CYCLIC_STATE from {:#04x}: {} in {} s = {} Hz",
                        src,
                        n,
                        secs,
                        *n as f32 / secs
                    );
                }
                println!(
                    "handheld: {} notifications, {} B, {} PDUs in {} s",
                    notes, bytes, pdus, secs
                );
                (notes, bytes, pdus) = (0, 0, 0);
                if sources.is_empty() {
                    println!("handheld: no CYCLIC_STATE in {} s", secs);
                }
                let params = conn.params();
                println!(
                    "handheld: interval {} us, MTU {}, heap {} B free",
                    params.conn_interval.as_micros(),
                    conn.att_mtu(),
                    esp_alloc::HEAP.free()
                );
                sources.clear();
                since = Instant::now();
            }
        }
    })
    .await;
}
