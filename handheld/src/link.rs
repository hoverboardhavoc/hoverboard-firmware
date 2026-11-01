//! The BLE central and the link (`specs/handheld-remote.md` sections 3.4 and 3.5): connect to the
//! pinned bridge with the filter accept list, find the link pair by UUID, subscribe, and run the
//! L2 stream between the notifications, the model and writes to `rx`.

use core::fmt::Write;
use core::sync::atomic::Ordering;
use embassy_futures::join::join;

use embassy_futures::select::{select, select3, Either3};
use embassy_time::{Duration, Instant, Timer};
use esp_println::println;
use trouble_host::prelude::*;

use handheld_core::session::Sends;
use handheld_core::wire::Wire;

use crate::{now_ms, with, LOG, PUMP, RELEASE_AT};

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;
/// The wait between a failed or lost connection and the next attempt (section 3.4).
const RETRY: Duration = Duration::from_millis(500);
/// How often the link steps the model when nothing arrives.
const STEP: Duration = Duration::from_millis(10);
/// The shortest gap between two `DRIVE_CMD` trace lines while armed, and while disarmed, ms.
const TRACE_ARMED: u32 = 250;
const TRACE_DISARMED: u32 = 5000;

/// Connect, serve, reconnect, for ever.
pub async fn run<C: Controller>(controller: C, bridge: [u8; 6]) {
    // Our own static random address, from the MAC as the bridge makes its own.
    let mut own = esp_hal::efuse::Efuse::read_base_mac_address();
    own.reverse();
    own[5] |= 0xC0;
    let mut target = bridge;
    target.reverse();
    let target = Address::random(target);

    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(own))
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
                println!("handheld: connecting");
                match central.connect(&config).await {
                    Ok(conn) => {
                        println!("handheld: connected");
                        session(&stack, &conn).await;
                        with(|m| m.disconnected());
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

/// One connection: discover, subscribe, then the stream until the link is lost.
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
        let Ok(services) = client
            .services_by_uuid(&Uuid::from(espsvc::LINK_SERVICE_UUID))
            .await
        else {
            println!("handheld: service discovery failed");
            return;
        };
        let Some(service) = services.first().cloned() else {
            println!("handheld: no link service");
            return;
        };
        let Ok(rx) = client
            .characteristic_by_uuid::<[u8]>(&service, &Uuid::from(espsvc::LINK_RX_UUID))
            .await
        else {
            println!("handheld: rx not found");
            return;
        };
        let Ok(tx) = client
            .characteristic_by_uuid::<[u8]>(&service, &Uuid::from(espsvc::LINK_TX_UUID))
            .await
        else {
            println!("handheld: tx not found");
            return;
        };
        let Ok(mut listener) = client.subscribe(&tx, false).await else {
            println!("handheld: subscribe failed");
            return;
        };
        let chunk_max = (conn.att_mtu() as usize).saturating_sub(3).clamp(20, 244);
        println!("handheld: subscribed, writes of {} B", chunk_max);
        with(|m| m.connected(now_ms()));

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
                    return;
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
    .await;
}
