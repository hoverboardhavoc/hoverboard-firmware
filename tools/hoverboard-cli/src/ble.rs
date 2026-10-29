//! The BLE link (`--device`, and the default with no link given) and `scan`, through
//! [`btleplug`](https://github.com/deviceplug/btleplug): the `ble` cargo feature
//! (`specs/bridge-onboarding.md` section 4.2). A build without it (the bench Pi's) refuses both.

use std::sync::mpsc;
use std::time::Duration;

use btleplug::api::{
    Central, CentralState, CharPropFlags, Characteristic, Manager as _, Peripheral as _,
    ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use tokio::runtime::Runtime;

use hoverboard_cli::wire::ByteLink;

/// The CC2541's notification and write size, which the bridge keeps.
const BLE_CHUNK: usize = 20;

/// The BLE link to a bridge (or a board's CC2541): writes in [`BLE_CHUNK`] pieces, notifications
/// in.
pub struct Ble {
    rt: Runtime,
    periph: Peripheral,
    write_char: Characteristic,
    rx: mpsc::Receiver<Vec<u8>>,
}

impl ByteLink for Ble {
    fn write(&mut self, bytes: &[u8]) {
        for c in bytes.chunks(BLE_CHUNK) {
            let _ = self.rt.block_on(self.periph.write(
                &self.write_char,
                c,
                WriteType::WithoutResponse,
            ));
        }
    }

    fn read(&mut self, timeout: Duration) -> Vec<u8> {
        let mut out = match self.rx.recv_timeout(timeout) {
            Ok(v) => v,
            Err(_) => return Vec::new(),
        };
        while let Ok(v) = self.rx.try_recv() {
            out.extend(v);
        }
        out
    }

    fn window(&self) -> usize {
        espsvc::WINDOW_BLE
    }
}

/// How long the Bluetooth stack may take to report an adapter and its state.
const ADAPTER_WAIT: Duration = Duration::from_secs(5);
/// How long finding, connecting to and subscribing to a bridge may take.
const CONNECT_WAIT: Duration = Duration::from_secs(30);

const NO_BLUETOOTH: &str =
    "no powered-on Bluetooth adapter on this host; reach the ESP's USB console with --serial or --tcp";

/// The first adapter, if it is powered on. A host with no Bluetooth either lists none or never
/// answers (CoreBluetooth on a machine without the hardware), so both are bounded.
async fn adapter() -> Result<Adapter, String> {
    let found = tokio::time::timeout(ADAPTER_WAIT, async {
        let manager = Manager::new().await.map_err(|e| e.to_string())?;
        let a = manager
            .adapters()
            .await
            .map_err(|e| e.to_string())?
            .into_iter()
            .next()
            .ok_or(NO_BLUETOOTH)?;
        match a.adapter_state().await.map_err(|e| e.to_string())? {
            CentralState::PoweredOn => Ok(a),
            _ => Err(NO_BLUETOOTH.to_string()),
        }
    })
    .await;
    found.unwrap_or_else(|_| Err(NO_BLUETOOTH.to_string()))
}

/// Advertisers seen in `secs`: (name, address, peripheral).
async fn scan(secs: u64) -> Result<Vec<(String, String, Peripheral)>, String> {
    let a = adapter().await?;
    a.start_scan(ScanFilter::default())
        .await
        .map_err(|e| e.to_string())?;
    tokio::time::sleep(Duration::from_secs(secs)).await;
    let _ = a.stop_scan().await;
    let mut out = Vec::new();
    for p in a.peripherals().await.map_err(|e| e.to_string())? {
        if let Ok(Some(props)) = p.properties().await {
            if let Some(name) = props.local_name {
                out.push((name, p.address().to_string(), p));
            }
        }
    }
    out.sort_by(|x, y| x.0.cmp(&y.0));
    Ok(out)
}

pub fn connect(device: Option<&str>) -> Result<Ble, String> {
    let rt = Runtime::new().map_err(|e| e.to_string())?;
    let (periph, write_char, rx) = rt.block_on(async {
        tokio::time::timeout(CONNECT_WAIT, async {
            let found = scan(4).await?;
            let (name, _, periph) = found
                .into_iter()
                .find(|(n, a, _)| match device {
                    Some(d) => n == d || a.eq_ignore_ascii_case(d),
                    None => n.starts_with("hb-bridge-"),
                })
                .ok_or_else(|| match device {
                    Some(d) => format!("no advertiser named or at {d}"),
                    None => "no hb-bridge- advertiser found".to_string(),
                })?;
            periph.connect().await.map_err(|e| e.to_string())?;
            periph
                .discover_services()
                .await
                .map_err(|e| e.to_string())?;
            // By property, not UUID, as the app's transport does.
            let chars = periph.characteristics();
            let write_char = chars
                .iter()
                .find(|c| c.properties.contains(CharPropFlags::WRITE_WITHOUT_RESPONSE))
                .cloned()
                .ok_or("no write-without-response characteristic")?;
            let notify_char = chars
                .iter()
                .find(|c| c.properties.contains(CharPropFlags::NOTIFY))
                .cloned()
                .ok_or("no notify characteristic")?;
            periph
                .subscribe(&notify_char)
                .await
                .map_err(|e| e.to_string())?;
            let mut stream = periph.notifications().await.map_err(|e| e.to_string())?;
            let (tx, rx) = mpsc::channel();
            tokio::spawn(async move {
                while let Some(n) = stream.next().await {
                    if tx.send(n.value).is_err() {
                        break;
                    }
                }
            });
            eprintln!("connected to {name}");
            Ok::<_, String>((periph, write_char, rx))
        })
        .await
        .unwrap_or_else(|_| {
            Err(format!(
                "no BLE connection within {} s",
                CONNECT_WAIT.as_secs()
            ))
        })
    })?;
    Ok(Ble {
        rt,
        periph,
        write_char,
        rx,
    })
}

/// `scan`: the advertisers seen in `secs`, as (name, address).
pub fn scan_names(secs: u64) -> Result<Vec<(String, String)>, String> {
    let rt = Runtime::new().map_err(|e| e.to_string())?;
    let found = rt.block_on(scan(secs))?;
    Ok(found.into_iter().map(|(n, a, _)| (n, a)).collect())
}
