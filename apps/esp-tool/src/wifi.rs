//! The Wi-Fi station (`specs/esp-tool.md`, "Installing and updating the ESP", "Wi-Fi"), the
//! console on TCP (`specs/esp-tool.md`, "The terminal tool", "The Wi-Fi console") and the mDNS
//! name.
//!
//! - **Station.** The network name and password are build-time settings, `ESP_WIFI_SSID` and
//!   `ESP_WIFI_PASS` in the environment of the build; nothing is provisioned at run time. The ESP
//!   joins as a station, takes an address by DHCP, and joins again whenever the association drops.
//!   BLE runs beside it on the same radio (`esp-radio`'s coexistence).
//! - **Console.** TCP port [`CONSOLE_PORT`] carries the same L2 stream as the USB console and is
//!   the same L3 port (port 2): the service loop's router decides, on each connection, whether the
//!   console is free ([`ADMIT`]); a client while one is attached on USB is closed at once, and a
//!   second TCP client finds no listener and is refused by the stack. No authentication.
//! - **mDNS.** `<name>.local` answers with the station's address (an A record), from `edge-mdns`,
//!   restarted with the new address after each rejoin.
//!
//! Bytes are not interpreted here: the console's go to the service loop as [`TcpEvent::Data`], and
//! the loop hands back what to send through [`OUT`].

use core::net::{Ipv4Addr, Ipv6Addr};

use edge_mdns::buf::VecBufAccess;
use edge_mdns::domain::base::Ttl;
use edge_mdns::host::Host;
use edge_mdns::io::{self, IPV4_DEFAULT_SOCKET};
use edge_mdns::HostAnswersMdnsHandler;
use edge_nal::UdpSplit;
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_futures::select::{select, Either};
use embassy_net::tcp::TcpSocket;
use embassy_net::{Runner, Stack};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::rng::Rng;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{Config, Interface, WifiController};
use heapless09::Vec;

const SSID: &str = env!(
    "ESP_WIFI_SSID",
    "set ESP_WIFI_SSID (and ESP_WIFI_PASS) to the Wi-Fi network the ESP joins, in the build's environment"
);
const PASSWORD: &str = env!(
    "ESP_WIFI_PASS",
    "set ESP_WIFI_PASS (and ESP_WIFI_SSID) to the Wi-Fi network the ESP joins, in the build's environment"
);

/// The console's TCP port: the one the bench's serial-to-TCP relay used.
pub const CONSOLE_PORT: u16 = 5333;
/// The most bytes one console read or write moves.
pub const PACKET: usize = 256;
/// How long the station waits after a drop or a failed join before joining again.
const REJOIN: Duration = Duration::from_secs(5);
/// A console client that leaves sent bytes unacknowledged this long is gone (a laptop that left
/// the network), and the console is free again.
const CLIENT_TIMEOUT: Duration = Duration::from_secs(15);
/// The keep-alive interval, so an idle client that left is noticed within [`CLIENT_TIMEOUT`].
const KEEP_ALIVE: Duration = Duration::from_secs(5);
/// The mDNS answers' time to live.
const MDNS_TTL: u32 = 120;

/// What the console listener tells the service loop. A channel slot is the size of the largest
/// variant whatever the variant sizes are, so the data variant is not boxed.
#[allow(clippy::large_enum_variant)]
pub enum TcpEvent {
    /// A client connected: answer through [`ADMIT`].
    Open,
    /// Bytes from the client.
    Data(Vec<u8, PACKET>),
    /// The client went away.
    Gone,
}

/// Console listener to service loop.
pub static EVENTS: Channel<CriticalSectionRawMutex, TcpEvent, 8> = Channel::new();
/// Service loop to console listener: whether the client just connected may have the console.
pub static ADMIT: Signal<CriticalSectionRawMutex, bool> = Signal::new();
/// Service loop to console listener: bytes to send, one write each.
pub static OUT: Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, 16> = Channel::new();

/// Join the network and join it again whenever the association drops, forever.
#[embassy_executor::task]
pub async fn station(mut controller: WifiController<'static>) {
    let config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
    );
    loop {
        if matches!(controller.is_connected(), Ok(true)) {
            let _ = controller.wait_for_disconnect_async().await;
            Timer::after(REJOIN).await;
        }
        if !matches!(controller.is_started(), Ok(true))
            && (controller.set_config(&config).is_err() || controller.start_async().await.is_err())
        {
            Timer::after(REJOIN).await;
            continue;
        }
        if controller.connect_async().await.is_err() {
            Timer::after(REJOIN).await;
        }
    }
}

/// The IP stack.
#[embassy_executor::task]
pub async fn net(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}

/// Serve the console on [`CONSOLE_PORT`], one client at a time, forever.
#[embassy_executor::task]
pub async fn console(stack: Stack<'static>) {
    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 2048];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
        socket.set_timeout(Some(CLIENT_TIMEOUT));
        socket.set_keep_alive(Some(KEEP_ALIVE));
        // The stream is small frames, each wanted at once.
        socket.set_nagle_enabled(false);
        if socket.accept(CONSOLE_PORT).await.is_err() {
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }
        ADMIT.reset();
        EVENTS.send(TcpEvent::Open).await;
        if ADMIT.wait().await {
            // Bytes queued for an earlier client are not this one's.
            while OUT.try_receive().is_ok() {}
            serve(&mut socket).await;
            EVENTS.send(TcpEvent::Gone).await;
        }
        socket.abort();
        let _ = socket.flush().await;
    }
}

/// Carry the console's bytes both ways until the client goes away.
async fn serve(socket: &mut TcpSocket<'_>) {
    let (mut r, mut w) = socket.split();
    let reader = async {
        let mut buf = [0u8; PACKET];
        loop {
            match r.read(&mut buf).await {
                Ok(0) | Err(_) => return,
                Ok(n) => {
                    let data = Vec::from_slice(&buf[..n]).unwrap_or_default();
                    EVENTS.send(TcpEvent::Data(data)).await;
                }
            }
        }
    };
    let writer = async {
        loop {
            let p = OUT.receive().await;
            let mut rest = &p[..];
            while !rest.is_empty() {
                match w.write(rest).await {
                    Ok(0) | Err(_) => return,
                    Ok(n) => rest = &rest[n..],
                }
            }
        }
    };
    select(reader, writer).await;
}

/// Answer mDNS queries for `<name>.local` with the station's address, forever.
#[embassy_executor::task]
pub async fn mdns(stack: Stack<'static>, name: &'static str, rng: Rng) {
    let buffers: UdpBuffers<1, 1500, 1500, 2> = UdpBuffers::new();
    let udp = Udp::new(stack, &buffers);
    let recv_buf = VecBufAccess::<NoopRawMutex, 1500>::new();
    let send_buf = VecBufAccess::<NoopRawMutex, 1500>::new();
    let signal = Signal::<NoopRawMutex, ()>::new();
    loop {
        stack.wait_config_up().await;
        let Some(config) = stack.config_v4() else {
            continue;
        };
        let host = Host {
            hostname: name,
            ipv4: config.address.address(),
            ipv6: Ipv6Addr::UNSPECIFIED,
            ttl: Ttl::from_secs(MDNS_TTL),
        };
        let Ok(mut socket) =
            io::bind(&udp, IPV4_DEFAULT_SOCKET, Some(Ipv4Addr::UNSPECIFIED), None).await
        else {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        };
        let (recv, send) = socket.split();
        let responder = io::Mdns::<_, _, _, _, _, NoopRawMutex>::new(
            Some(Ipv4Addr::UNSPECIFIED),
            None,
            recv,
            send,
            &recv_buf,
            &send_buf,
            rng,
            &signal,
        );
        // Until the address goes (a drop, a new lease), then again with the next one.
        if let Either::First(_) = select(
            responder.run(HostAnswersMdnsHandler::new(&host)),
            stack.wait_config_down(),
        )
        .await
        {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
