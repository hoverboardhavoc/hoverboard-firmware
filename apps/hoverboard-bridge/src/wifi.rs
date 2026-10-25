//! The Wi-Fi station (`specs/hoverboard-bridge.md`, "Installing and updating the bridge", "Wi-Fi"), the
//! console on TCP (`specs/hoverboard-bridge.md`, "The terminal tool", "The Wi-Fi console") and the mDNS
//! name.
//!
//! - **Station.** The network name and password are build-time settings, `ESP_WIFI_SSID` and
//!   `ESP_WIFI_PASS` in the environment of the build; nothing is provisioned at run time. The bridge
//!   joins as a station, takes an address by DHCP, and joins again whenever the association drops.
//!   BLE runs beside it on the same radio (`esp-radio`'s coexistence). Its transmit power is
//!   capped at [`MAX_TX_POWER`] after every start, before it joins.
//! - **Console.** TCP port [`CONSOLE_PORT`] carries the same L2 stream as the USB console, the
//!   same L3 port (port 2) on a second carrier: both are served whenever present, a reply goes
//!   back on the carrier its request came in on, and board packets go out on both. One client at
//!   a time, and a listener is always open: two sockets take turns, one serving while the other
//!   listens, so a client that connects as the previous one leaves is accepted (the listener is
//!   listening again the moment a client leaves); a second connection while one is served is
//!   accepted by the listening socket and closed at once. No authentication.
//! - **GDB.** TCP port [`GDB_PORT`] carries the GDB remote serial protocol
//!   (`specs/hoverboard-bridge.md`, "Debugging"), one client at a time like the console, its bytes
//!   handed to the service loop's GDB server ([`GDB_EVENTS`], [`GDB_OUT`]). The server ends a
//!   session from its side (`detach`, `kill`, a protocol error) through [`GDB_CLOSE`], after its
//!   last reply is in [`GDB_OUT`]; the listener sends that reply, closes, and waits for the close
//!   to finish, so GDB reads the `OK` and not a reset connection
//!   (`specs/hoverboard-bridge.md`, "Debugging", "One client"). No authentication.
//! - **mDNS.** `<name>.local` answers with the station's address (an A record), and the GDB port
//!   is announced as `_gdb._tcp`, from `edge-mdns`, restarted with the new address after each
//!   rejoin.
//!
//! Bytes are not interpreted here: each listener's go to the service loop as [`TcpEvent::Data`],
//! and the loop hands back what to send through [`OUT`] and [`GDB_OUT`].

use core::net::{Ipv4Addr, Ipv6Addr};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use edge_mdns::buf::VecBufAccess;
use edge_mdns::domain::base::Ttl;
use edge_mdns::host::{Host, Service, ServiceAnswers};
use edge_mdns::io::{self, IPV4_DEFAULT_SOCKET};
use edge_mdns::HostAnswersMdnsHandler;
use edge_nal::UdpSplit;
use edge_nal_embassy::{Udp, UdpBuffers};
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_net::tcp::{State, TcpSocket};
use embassy_net::{Runner, Stack};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration, Timer};
use esp_hal::rng::Rng;
use esp_radio::wifi::sta::StationConfig;
use esp_radio::wifi::{Config, Interface, WifiController};
use espsvc::WifiState;
use heapless09::Vec;

const SSID: &str = env!(
    "ESP_WIFI_SSID",
    "set ESP_WIFI_SSID (and ESP_WIFI_PASS) to the Wi-Fi network the bridge joins, in the build's environment"
);
const PASSWORD: &str = env!(
    "ESP_WIFI_PASS",
    "set ESP_WIFI_PASS (and ESP_WIFI_SSID) to the Wi-Fi network the bridge joins, in the build's environment"
);

/// The console's TCP port: the one the bench's serial-to-TCP relay used.
pub const CONSOLE_PORT: u16 = 5333;
/// The GDB server's TCP port, OpenOCD's.
pub const GDB_PORT: u16 = 3333;
/// The most bytes one console read or write moves.
pub const PACKET: usize = 256;
/// The console socket's receive buffer, which is the TCP receive window: a console's bulk window
/// of 64 chunk frames (3,840 B) in flight, so the sender's TCP never waits on the window for it.
const CONSOLE_RX: usize = 4096;
/// The GDB socket's receive buffer.
const GDB_RX: usize = 1024;
/// Each socket's transmit buffer.
const TX: usize = 2048;
/// How long a served socket's close may take (its last bytes and FIN acknowledged, the client's
/// FIN received) before it is aborted. The other socket is listening meanwhile.
const CLOSE_TIMEOUT: Duration = Duration::from_secs(2);
/// How often a socket that is mid-handshake is looked at again.
const HANDSHAKE_POLL: Duration = Duration::from_millis(5);
/// The station's transmit power cap in quarter-dBm units, 34 for 8.5 dBm, set after every start and
/// before the join (`esp_wifi_set_max_tx_power`, as `wireless-esp32-dap`'s `feat/tx-power-cap`
/// does). At full power, SuperMinis from one batch stop acknowledging the access point's
/// authentication response and never join; capped they join in 2 s at -38 dBm. The bridge is never
/// more than a room away from its access point, so the cap costs nothing.
const MAX_TX_POWER: i8 = 34;
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
    /// A client connected.
    Open,
    /// Bytes from the client.
    Data(Vec<u8, PACKET>),
    /// The client went away.
    Gone,
}

/// The station's state for `ESP_INFO`, as [`watch`] last saw it: a [`WifiState`] byte and the
/// address. Down until the Wi-Fi driver is up, and for good if it failed.
static STATE: AtomicU8 = AtomicU8::new(WifiState::Down as u8);
static ADDRESS: AtomicU32 = AtomicU32::new(0);
/// How often [`watch`] looks at the station.
const WATCH: Duration = Duration::from_millis(250);

/// The station's state and, while it is up, its address.
pub fn state() -> (WifiState, [u8; 4]) {
    let s = WifiState::from_u8(STATE.load(Ordering::Relaxed)).unwrap_or(WifiState::Down);
    (s, ADDRESS.load(Ordering::Relaxed).to_be_bytes())
}

/// Keep [`state`] current, forever.
#[embassy_executor::task]
pub async fn watch(stack: Stack<'static>) {
    loop {
        let (s, ip) = match (stack.is_link_up(), stack.config_v4()) {
            (false, _) => (WifiState::Down, 0),
            (true, None) => (WifiState::Joined, 0),
            (true, Some(c)) => (WifiState::Up, u32::from(c.address.address())),
        };
        ADDRESS.store(ip, Ordering::Relaxed);
        STATE.store(s as u8, Ordering::Relaxed);
        Timer::after(WATCH).await;
    }
}

/// Console listener to service loop.
pub static EVENTS: Channel<CriticalSectionRawMutex, TcpEvent, 8> = Channel::new();
/// Service loop to console listener: bytes to send, one write each.
pub static OUT: Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, 16> = Channel::new();
/// GDB listener to service loop. Fewer slots than the console's: each is a packet's worth of DRAM
/// out of the main stack's share, and the service loop takes GDB's bytes every pass.
pub static GDB_EVENTS: Channel<CriticalSectionRawMutex, TcpEvent, 4> = Channel::new();
/// Service loop to GDB listener: bytes to send, one write each.
pub static GDB_OUT: Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, 4> = Channel::new();
/// Service loop to GDB listener: the session ended on the protocol side; close the connection.
pub static GDB_CLOSE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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
            let _e = controller.wait_for_disconnect_async().await;
            boot!("Wi-Fi: dropped: {:?}", _e);
            Timer::after(REJOIN).await;
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let started = match controller.set_config(&config) {
                Ok(()) => controller.start_async().await,
                Err(e) => Err(e),
            };
            match started {
                Ok(()) => boot!("Wi-Fi: started"),
                Err(_e) => {
                    boot!("Wi-Fi: start failed: {:?}", _e);
                    Timer::after(REJOIN).await;
                    continue;
                }
            }
            match controller.set_max_tx_power(MAX_TX_POWER) {
                Ok(()) => boot!(
                    "Wi-Fi: transmit power capped at {} quarter-dBm",
                    MAX_TX_POWER
                ),
                Err(_e) => boot!("Wi-Fi: transmit power cap failed: {:?}", _e),
            }
        }
        boot!("Wi-Fi: joining");
        match controller.connect_async().await {
            Ok(_) => boot!("Wi-Fi: joined, waiting for an address"),
            Err(_e) => {
                boot!("Wi-Fi: join failed: {:?}", _e);
                Timer::after(REJOIN).await;
            }
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
    boot!("TCP console listening on {}", CONSOLE_PORT);
    listen::<_, _, CONSOLE_RX>(stack, CONSOLE_PORT, &EVENTS, &OUT, None).await
}

/// Serve GDB on [`GDB_PORT`], one client at a time, forever.
#[embassy_executor::task]
pub async fn gdb(stack: Stack<'static>) {
    boot!("GDB listening on {}", GDB_PORT);
    listen::<_, _, GDB_RX>(stack, GDB_PORT, &GDB_EVENTS, &GDB_OUT, Some(&GDB_CLOSE)).await
}

/// Accept one client at a time on `port` and carry its bytes to and from the service loop
/// through `events` and `out`, until it leaves or `close` is signalled. `RX` is each socket's
/// receive buffer.
///
/// Two sockets take turns: while one serves a client the other listens, refusing a second client
/// by closing it at once, and when the client leaves the listening one serves next, so `port`
/// is never without a listener (`specs/hoverboard-bridge.md`, "The Wi-Fi console").
async fn listen<const E: usize, const O: usize, const RX: usize>(
    stack: Stack<'static>,
    port: u16,
    events: &'static Channel<CriticalSectionRawMutex, TcpEvent, E>,
    out: &'static Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, O>,
    close: Option<&'static Signal<CriticalSectionRawMutex, ()>>,
) {
    let [rx0, rx1] = &mut [[0u8; RX]; 2];
    let [tx0, tx1] = &mut [[0u8; TX]; 2];
    let mut sockets = [
        configured(TcpSocket::new(stack, rx0, tx0)),
        configured(TcpSocket::new(stack, rx1, tx1)),
    ];
    let mut turn = 0;
    loop {
        let [a, b] = &mut sockets;
        let (served, spare) = if turn == 0 { (a, b) } else { (b, a) };
        if !accept(served, port).await {
            served.abort();
            let _ = served.flush().await;
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }
        boot!("TCP {}: client connected", port);
        // Bytes queued for an earlier client are not this one's.
        while out.try_receive().is_ok() {}
        if let Some(c) = close {
            c.reset();
        }
        events.send(TcpEvent::Open).await;
        // The session is polled first, so a client that connects in the pass where this one
        // leaves is kept for the next turn, not refused.
        let refuse = async {
            loop {
                if spare.accept(port).await.is_ok() {
                    boot!("TCP {}: second client refused", port);
                }
                spare.abort();
                let _ = spare.flush().await;
            }
        };
        let ended = match select(serve(served, events, out, close), refuse).await {
            Either::First(ended) => ended,
            Either::Second(never) => never,
        };
        events.send(TcpEvent::Gone).await;
        boot!("TCP {}: client gone", port);
        finish(served, out, ended).await;
        turn ^= 1;
    }
}

/// A listener socket's settings, which last across its connections.
fn configured(mut socket: TcpSocket<'_>) -> TcpSocket<'_> {
    socket.set_timeout(Some(CLIENT_TIMEOUT));
    socket.set_keep_alive(Some(KEEP_ALIVE));
    // The stream is small frames, each wanted at once.
    socket.set_nagle_enabled(false);
    socket
}

/// Wait for a client on `socket`: one it already holds (accepted while the other socket was
/// serving, or mid-handshake), or a new one, listening if it is not already. False if the
/// listen failed.
async fn accept(socket: &mut TcpSocket<'_>, port: u16) -> bool {
    loop {
        match socket.state() {
            State::SynReceived => Timer::after(HANDSHAKE_POLL).await,
            State::Closed | State::TimeWait | State::Listen => {
                return socket.accept(port).await.is_ok();
            }
            _ => return true,
        }
    }
}

/// End a served connection without a reset: when the session ended on the protocol side
/// (`ended`), the replies still queued in `out` (a detach's `OK`) are written first; then the
/// write half is closed, the last bytes and the FIN are acknowledged, and the client's FIN is
/// read. A client that does not finish within [`CLOSE_TIMEOUT`] is aborted.
async fn finish<const O: usize>(
    socket: &mut TcpSocket<'_>,
    out: &'static Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, O>,
    ended: bool,
) {
    let _ = with_timeout(CLOSE_TIMEOUT, async {
        if ended {
            while let Ok(p) = out.try_receive() {
                let mut rest = &p[..];
                while !rest.is_empty() {
                    match socket.write(rest).await {
                        Ok(0) | Err(_) => break,
                        Ok(n) => rest = &rest[n..],
                    }
                }
            }
        }
        socket.close();
        let _ = socket.flush().await;
        let mut buf = [0u8; 64];
        while let Ok(1..) = socket.read(&mut buf).await {}
    })
    .await;
    if !matches!(socket.state(), State::Closed | State::TimeWait) {
        socket.abort();
        let _ = socket.flush().await;
    }
}

/// Carry a client's bytes both ways until it goes away or `close` is signalled; true for the
/// signal.
async fn serve<const E: usize, const O: usize>(
    socket: &mut TcpSocket<'_>,
    events: &'static Channel<CriticalSectionRawMutex, TcpEvent, E>,
    out: &'static Channel<CriticalSectionRawMutex, Vec<u8, PACKET>, O>,
    close: Option<&'static Signal<CriticalSectionRawMutex, ()>>,
) -> bool {
    let (mut r, mut w) = socket.split();
    let reader = async {
        let mut buf = [0u8; PACKET];
        loop {
            match r.read(&mut buf).await {
                Ok(0) | Err(_) => return,
                Ok(n) => {
                    let data = Vec::from_slice(&buf[..n]).unwrap_or_default();
                    events.send(TcpEvent::Data(data)).await;
                }
            }
        }
    };
    let writer = async {
        loop {
            let p = out.receive().await;
            let mut rest = &p[..];
            while !rest.is_empty() {
                match w.write(rest).await {
                    Ok(0) | Err(_) => return,
                    Ok(n) => rest = &rest[n..],
                }
            }
        }
    };
    let closed = async {
        match close {
            Some(c) => c.wait().await,
            None => core::future::pending().await,
        }
    };
    matches!(select3(reader, writer, closed).await, Either3::Third(()))
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
        boot!("Wi-Fi: address {}", config.address.address());
        let host = Host {
            hostname: name,
            ipv4: config.address.address(),
            ipv6: Ipv6Addr::UNSPECIFIED,
            ttl: Ttl::from_secs(MDNS_TTL),
        };
        let gdb = Service {
            name,
            priority: 0,
            weight: 0,
            service: "_gdb",
            protocol: "_tcp",
            port: GDB_PORT,
            service_subtypes: &[],
            txt_kvs: &[],
        };
        let Ok(mut socket) =
            io::bind(&udp, IPV4_DEFAULT_SOCKET, Some(Ipv4Addr::UNSPECIFIED), None).await
        else {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        };
        boot!("mDNS: answering for {}.local and _gdb._tcp", name);
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
            responder.run(HostAnswersMdnsHandler::new(ServiceAnswers::new(
                &host, &gdb,
            ))),
            stack.wait_config_down(),
        )
        .await
        {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
