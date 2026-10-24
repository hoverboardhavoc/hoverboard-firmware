//! The GATT peripheral (`specs/esp-tool.md`, "The BLE link"): one write-without-response
//! characteristic and one notify characteristic, the shape of the CC2541's, so the app's transport
//! (which picks characteristics by property, not by UUID) finds them. One central at a time: the
//! host is built for a single connection, and advertising stops while it is held.
//!
//! The bytes are not interpreted here: writes go to the service loop as [`BleEvent::Data`], and
//! the loop hands back notifications of at most 20 B through [`NOTIFY`].

use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use heapless09::Vec;
use trouble_host::prelude::*;

use esp_tool_core::ble::NOTIFY_MAX;

/// The largest single write accepted (an ATT MTU of 247 less the write header).
pub const WRITE_MAX: usize = 244;

/// What the GATT side tells the service loop. A channel slot is the size of the largest variant
/// whatever the variant sizes are, so the data variant is not boxed.
#[allow(clippy::large_enum_variant)]
pub enum BleEvent {
    /// A central connected.
    Connected,
    /// The central went away.
    Disconnected,
    /// Bytes written to the write characteristic.
    Data(Vec<u8, WRITE_MAX>),
}

/// GATT side to service loop.
pub static EVENTS: Channel<CriticalSectionRawMutex, BleEvent, 16> = Channel::new();
/// Service loop to GATT side: one notification each.
pub static NOTIFY: Channel<CriticalSectionRawMutex, Vec<u8, NOTIFY_MAX>, 32> = Channel::new();

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2;

#[gatt_server]
struct Server {
    link: LinkService,
}

/// The link service. The UUIDs are this firmware's own; the app finds the pair by property.
#[gatt_service(uuid = "4842e500-7a1d-4c0e-9d1b-35a1c0de0001")]
struct LinkService {
    /// Central to ESP: the L2 byte stream.
    #[characteristic(uuid = "4842e500-7a1d-4c0e-9d1b-35a1c0de0002", write_without_response)]
    rx: Vec<u8, WRITE_MAX>,
    /// ESP to central: the L2 byte stream, at most 20 B per notification.
    #[characteristic(uuid = "4842e500-7a1d-4c0e-9d1b-35a1c0de0003", notify)]
    tx: Vec<u8, NOTIFY_MAX>,
}

/// Run the BLE host forever: advertise as `name`, serve one central at a time.
pub async fn run<C: Controller>(controller: C, name: &'static str, mac: [u8; 6]) {
    // A static random address from the MAC (the top two bits set, as the spec requires).
    let mut a = mac;
    a.reverse();
    a[5] |= 0xC0;
    let address = Address::random(a);
    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name,
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    esp_println::println!("BLE: advertising as {}", name);

    let _ = join(
        async {
            loop {
                if runner.run().await.is_err() {
                    Timer::after(Duration::from_millis(200)).await;
                }
            }
        },
        async {
            loop {
                match advertise(name, &mut peripheral, &server).await {
                    Ok(conn) => {
                        // Stale notifications from a previous central are not this one's.
                        while NOTIFY.try_receive().is_ok() {}
                        EVENTS.send(BleEvent::Connected).await;
                        select(gatt_events(&server, &conn), notify(&server, &conn)).await;
                        EVENTS.send(BleEvent::Disconnected).await;
                    }
                    Err(_) => Timer::after(Duration::from_millis(200)).await,
                }
            }
        },
    )
    .await;
}

async fn advertise<'a, 'b, C: Controller>(
    name: &str,
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
    server: &'b Server<'a>,
) -> Result<GattConnection<'a, 'b, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut adv_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

/// Hand every write to the service loop until the connection closes.
async fn gatt_events<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let rx = server.link.rx.handle;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { .. } => break,
            GattConnectionEvent::Gatt { event } => {
                if let GattEvent::Write(w) = &event {
                    if w.handle() == rx {
                        let data = w.with_data(|_off, d| Vec::from_slice(d).unwrap_or_default());
                        EVENTS.send(BleEvent::Data(data)).await;
                    }
                }
                if let Ok(reply) = event.accept() {
                    reply.send().await;
                }
            }
            _ => {}
        }
    }
}

/// Send each notification the service loop produces until the connection drops.
async fn notify<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    let tx = &server.link.tx;
    loop {
        let n = NOTIFY.receive().await;
        if tx.notify(conn, &n, false).await.is_err() {
            break;
        }
    }
}
