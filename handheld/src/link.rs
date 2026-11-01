//! The BLE central and the link (`specs/handheld-remote.md` sections 3.2 to 3.5). The model's
//! link state says what to do: `Picking`, scan passively for the Connect screen until a tap picks a
//! row; `Connecting`, connect to the pick with the filter accept list, again 500 ms after each
//! failure, until it connects or the rider cancels; connected, find the link pair by UUID,
//! subscribe, and run the L2 stream between the notifications, the model and writes to `rx`,
//! until the link is lost (connect again) or the rider disconnects (back to the Connect screen).

use core::fmt::Write;
use core::sync::atomic::Ordering;
use embassy_futures::join::join;

use embassy_futures::select::{select, select3, Either, Either3};
use embassy_time::{Duration, Instant, Timer};
use esp_println::println;
use trouble_host::prelude::*;

use handheld_core::model::Link;
use handheld_core::pick::{self, Pick};
use handheld_core::scan;
use handheld_core::session::Sends;
use handheld_core::wire::Wire;

use crate::pick_flash::PickFlash;
use crate::{now_ms, with, LOG, PUMP, RELEASE_AT};

/// The BLE controller: esp-radio's, behind bt-hci's external-controller adapter.
pub type Ctl = ExternalController<esp_radio::ble::controller::BleConnector<'static>, 20>;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;
/// The wait between a failed or lost connection and the next attempt (section 3.4).
const RETRY: Duration = Duration::from_millis(500);
/// How often the link steps the model when nothing arrives.
const STEP: Duration = Duration::from_millis(10);
/// How often the scan and the connect look at the model for a tap or a cancel.
const LOOK: Duration = Duration::from_millis(20);
/// The scan restarts this often: trouble-host enables it with the controller's duplicate filter,
/// which reports each address once per scan, and the list needs every address again for its RSSI
/// and its 10 s expiry.
const RESCAN: Duration = Duration::from_secs(1);
/// The shortest gap between two `DRIVE_CMD` trace lines while armed, and while disarmed, ms.
const TRACE_ARMED: u32 = 250;
const TRACE_DISARMED: u32 = 5000;

/// How a connection ended.
enum Ended {
    /// Lost, or a step after connecting failed: connect again.
    Lost,
    /// The peer has no link service.
    NotABridge,
    /// The rider disconnected, or the link is otherwise no longer wanted.
    Rider,
}

/// Every advertisement the controller reports, to the model's scan list.
struct Reports;

impl EventHandler for Reports {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        let now = now_ms();
        while let Some(Ok(r)) = it.next() {
            let mut addr = r.addr.into_inner();
            addr.reverse();
            let name = scan::adv_name(r.data).map(|b| core::str::from_utf8(b).unwrap_or("?"));
            with(|m| m.on_adv(addr, name, r.rssi, now));
        }
    }
}

fn link() -> Link {
    with(|m| m.link)
}

/// Run the link for ever.
pub async fn run(controller: Ctl, mut flash: Option<PickFlash>) {
    // Our own static random address, from the MAC as the bridge makes its own.
    let mut own = esp_hal::efuse::Efuse::read_base_mac_address();
    own.reverse();
    own[5] |= 0xC0;

    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(own))
        .build();
    let mut runner = stack.runner();
    let central = stack.central();

    let _ = join(
        async {
            loop {
                if let Err(e) = runner.run_with_handler(&Reports).await {
                    println!("handheld: BLE host runner failed: {:?}", e);
                    Timer::after(Duration::from_millis(200)).await;
                }
            }
        },
        async {
            let mut central = Some(central);
            loop {
                match link() {
                    Link::Picking => {
                        let c = picking(central.take().expect("the central is held"), &mut flash);
                        central = Some(c.await);
                    }
                    _ => {
                        let c = central.as_mut().expect("the central is held");
                        connecting(&stack, c, &mut flash).await;
                    }
                }
            }
        },
    )
    .await;
}

/// The Connect screen: a passive, unfiltered scan, restarted every second, until a tap leaves the
/// model connecting. The scan stops before the pick is written to flash.
async fn picking<'d>(
    central: Central<'d, Ctl, DefaultPacketPool>,
    flash: &mut Option<PickFlash>,
) -> Central<'d, Ctl, DefaultPacketPool> {
    println!("handheld: scanning");
    let config = ScanConfig {
        active: false,
        ..Default::default()
    };
    let mut scanner = Scanner::new(central);
    while link() == Link::Picking {
        let session = match scanner.scan(&config).await {
            Ok(s) => Some(s),
            Err(e) => {
                println!("handheld: scan failed: {:?}", e);
                None
            }
        };
        let until = Instant::now() + RESCAN;
        while Instant::now() < until && link() == Link::Picking {
            Timer::after(LOOK).await;
            with(|m| m.expire_scan(now_ms()));
        }
        drop(session);
    }
    // The scan's stop goes to the controller through the runner; the flash write disables the
    // cache and interrupts for tens of ms, so it waits for that first.
    Timer::after(Duration::from_millis(50)).await;
    if let (Some(p), Some(f)) = (with(|m| m.take_write()), flash.as_mut()) {
        f.store(&p);
    }
    scanner.into_inner()
}

/// The address to connect to: a static random address as every bridge has, else a public one.
fn address(p: &Pick) -> Address {
    let mut a = p.addr;
    a.reverse();
    if pick::is_static_random(&p.addr) {
        Address::random(a)
    } else {
        Address {
            kind: AddrKind::PUBLIC,
            addr: BdAddr::new(a),
        }
    }
}

/// Resolves once the model has left `Connecting` (the rider cancelled).
async fn cancelled() {
    while link() == Link::Connecting {
        Timer::after(LOOK).await;
    }
}

/// Connect to the pick, again after each failure or loss, until the model leaves `Connecting`.
async fn connecting(
    stack: &Stack<'_, Ctl, DefaultPacketPool>,
    central: &mut Central<'_, Ctl, DefaultPacketPool>,
    flash: &mut Option<PickFlash>,
) {
    let p = with(|m| m.pick.clone()).expect("the model connects only with a pick");
    let accept = [address(&p)];
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
    while link() == Link::Connecting {
        println!("handheld: connecting to {}", pick::addr_text(&p.addr));
        // Dropping the connect on a cancel cancels it at the controller.
        match select(central.connect(&config), cancelled()).await {
            Either::Second(()) => {
                println!("handheld: connect cancelled");
                return;
            }
            Either::First(Err(e)) => println!("handheld: connect failed: {:?}", e),
            Either::First(Ok(conn)) => {
                println!("handheld: connected");
                let ended = session(stack, &conn).await;
                match ended {
                    Ended::Lost => {
                        with(|m| m.disconnected());
                        println!("handheld: disconnected");
                    }
                    Ended::NotABridge => {
                        println!("handheld: not a hoverboard bridge");
                        conn.disconnect();
                        with(|m| m.not_a_bridge());
                        if let Some(f) = flash.as_mut() {
                            // Let the disconnect reach the controller before the flash stalls it.
                            Timer::after(Duration::from_millis(50)).await;
                            f.erase();
                        }
                        return;
                    }
                    Ended::Rider => {
                        println!("handheld: disconnected by the rider");
                        conn.disconnect();
                        with(|m| m.disconnected());
                        return;
                    }
                }
            }
        }
        if let Either::Second(()) = select(Timer::after(RETRY), cancelled()).await {
            return;
        }
    }
}

/// One connection: discover, subscribe, then the stream until the link is lost or the model leaves
/// the connection.
async fn session(
    stack: &Stack<'_, Ctl, DefaultPacketPool>,
    conn: &Connection<'_, DefaultPacketPool>,
) -> Ended {
    let client = match GattClient::<Ctl, DefaultPacketPool, 4>::new(stack, conn).await {
        Ok(c) => c,
        Err(e) => {
            println!("handheld: GATT client failed: {:?}", e);
            return Ended::Lost;
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
    match select(client.task(), async {
        let Ok(services) = client
            .services_by_uuid(&Uuid::from(espsvc::LINK_SERVICE_UUID))
            .await
        else {
            println!("handheld: service discovery failed");
            return Ended::Lost;
        };
        let Some(service) = services.first().cloned() else {
            return Ended::NotABridge;
        };
        let Ok(rx) = client
            .characteristic_by_uuid::<[u8]>(&service, &Uuid::from(espsvc::LINK_RX_UUID))
            .await
        else {
            println!("handheld: rx not found");
            return Ended::Lost;
        };
        let Ok(tx) = client
            .characteristic_by_uuid::<[u8]>(&service, &Uuid::from(espsvc::LINK_TX_UUID))
            .await
        else {
            println!("handheld: tx not found");
            return Ended::Lost;
        };
        let Ok(mut listener) = client.subscribe(&tx, false).await else {
            println!("handheld: subscribe failed");
            return Ended::Lost;
        };
        let chunk_max = (conn.att_mtu() as usize).saturating_sub(3).clamp(20, 244);
        println!("handheld: subscribed, writes of {} B", chunk_max);
        // A cancel during discovery leaves the model on the Connect screen: do not connect it.
        if !with(|m| {
            let wanted = m.link == Link::Connecting;
            if wanted {
                m.connected(now_ms());
            }
            wanted
        }) {
            return Ended::Rider;
        }

        let mut wire = Wire::new();
        let mut buf = [0u8; 256];
        let mut chunk = [0u8; 244];
        let mut tick = Instant::now();
        let (mut rx_n, mut tx_n, mut since) = (0u16, 0u16, now_ms());
        let mut traced = now_ms().wrapping_sub(TRACE_DISARMED);
        loop {
            match select3(listener.next(), Timer::at(tick + STEP), PUMP.wait()).await {
                Either3::First(n) => wire.feed(n.as_ref()),
                Either3::Second(_) => tick = Instant::now(),
                Either3::Third(_) => {}
            }
            // The disconnect hold left the model on the Connect screen.
            if !matches!(link(), Link::Connected { .. }) {
                return Ended::Rider;
            }
            let now = now_ms();
            let mut sends = Sends::new();
            while let Some(pdu) = wire.recv(&mut buf) {
                rx_n = rx_n.saturating_add(1);
                with(|m| m.on_pdu(pdu, now, &mut sends));
            }
            with(|m| m.poll(now, &mut sends));
            for s in sends.iter() {
                wire.send(s);
                tx_n = tx_n.saturating_add(1);
            }
            // The pump's frames, latest wins: the outbox holds one of each kind per board until
            // the stream before it has gone to the BLE host.
            let (frames, release_at, trace) = with(|m| {
                let every = if m.arm.armed() {
                    TRACE_ARMED
                } else {
                    TRACE_DISARMED
                };
                let trace = if now.wrapping_sub(traced) >= every {
                    m.session.src().and(m.drive_trace())
                } else {
                    None
                };
                let f = m.session.src().map(|src| m.outbox.take(src));
                (f, RELEASE_AT.swap(0, Ordering::Relaxed), trace)
            });
            if let Some(t) = trace {
                traced = now;
                let _ = LOG.try_send(t);
            }
            for f in frames.iter().flatten() {
                wire.send(f);
                tx_n = tx_n.saturating_add(1);
            }
            while wire.pending() > 0 {
                let n = wire.take(&mut chunk[..chunk_max]);
                if client
                    .write_characteristic_without_response(&rx, &chunk[..n])
                    .await
                    .is_err()
                {
                    println!("handheld: write failed");
                    return Ended::Lost;
                }
            }
            if release_at != 0 {
                let us = (Instant::now().as_micros() as u32).wrapping_sub(release_at);
                let mut line = heapless::String::new();
                let _ = write!(
                    line,
                    "handheld: release handed to the BLE host {} us after the Up sample",
                    us
                );
                let _ = LOG.try_send(line);
            }
            if now.wrapping_sub(since) >= 1000 {
                with(|m| {
                    m.rx_rate = rx_n;
                    m.tx_rate = tx_n;
                    m.heap_free = esp_alloc::HEAP.free() as u32;
                });
                (rx_n, tx_n, since) = (0, 0, now);
            }
        }
    })
    .await
    {
        Either::First(_) => Ended::Lost,
        Either::Second(e) => e,
    }
}
