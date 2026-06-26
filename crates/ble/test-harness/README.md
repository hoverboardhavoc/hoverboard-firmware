# BLE throughput harness

Measures the **raw BLE byte-pipe** of the onboard module (see
[../../../docs/onboard-ble-module.md](../../../docs/onboard-ble-module.md)): how many bytes per second
round-trip reliably between a phone and the board, and where loss begins. It is two cooperating pieces:

- **Board side**: a byte-loopback firmware (`crates/ble-loopback-test`) that brings the module into
  transparent mode and echoes every received byte straight back. It is below any application protocol: no
  framing, no opcodes, just raw echo.
- **Phone side**: an Android app (this directory) that acts as the BLE central. It sends sequence-numbered
  packets with a known byte pattern, and because the board echoes verbatim, it scores the returned stream
  for loss, corruption, goodput, and round-trip latency.

The harness measures the layer *below* any inter-board/app protocol, so its result is the byte budget that
bounds whatever rides on top.

## Build

The Android app needs a JDK and the Android SDK. Android Studio's bundled JBR works:

```sh
export JAVA_HOME="<Android Studio>/Contents/jbr/Contents/Home"   # JDK 21
export ANDROID_HOME="$HOME/Library/Android/sdk"
cd crates/ble/test-harness
./gradlew :app:assembleDebug          # -> app/build/outputs/apk/debug/app-debug.apk
./gradlew :app:testDebugUnitTest      # Tier-1/2 JVM unit tests (no device needed)
```

The board-side loopback firmware builds for the chip and is flashed like the other firmware images; its
advertised name is set at build time:

```sh
HB_BLE_NAME=hb1 cargo build --release -p ble-loopback-test --target thumbv7m-none-eabi
```

## Run (headless, rootless)

The app is driven by an intent; results go to **logcat** (tag `BLE_TPUT`) and a JSON file. No UI, no root.

One-time per install: grant the Nearby-devices permission (Settings to App info to Permissions, or
`adb shell pm grant` on devices that allow it).

```sh
adb -s <device> install -r app/build/outputs/apk/debug/app-debug.apk

adb -s <device> shell am start -n com.hoverboard.bletest/.RunnerActivity \
  --es device OnePlus --es name hb1 --es mode loopback \
  --ei payload 16 --ei rate 5 --ei dur 8 \
  --es write nores --ei mtu 247 --es prio high --es out run.json

# collect the result (quits when the run prints DONE)
adb -s <device> logcat -s 'BLE_TPUT:V' | sed '/DONE status/q'
```

Extras: `device` (which phone, matched by `Build.MODEL`), `name` (the advertised name to scan for, see
below), `mode` (`loopback` for the board, `fake` for a local software echo), `payload`/`rate`/`dur`,
`write` (`nores`/`res`), `mtu`, `prio` (`high`/`balanced`/`low`).

A run logs `RESULT {…json…}` with `sent`, `delivered`, `lost`, `loss_fraction`, `corrupted`, `resyncs`,
`goodput_bps`, and a latency summary, then `DONE`.

## Tiers

- **Tier 1** (host, no hardware): JVM unit tests of the measurement codec (build/parse a packet, recover
  packets from a split or coalesced byte stream, detect corruption).
- **Tier 2** (no board): the harness against a local software echo, to prove the measurement plumbing.
- **Tier 3** (silicon): a real phone plus a board running the loopback firmware over the actual module.

## Device-specific gotchas

These were worked out on real phones and are easy to lose hours to:

- **Use a fresh, unique advertised name per setup** (`HB_BLE_NAME` on the firmware, `--es name` on the
  app). The module persists its name and phones cache it, so a constant name makes it impossible to tell a
  fresh bring-up from a stale/cached advert.
- **Match the trimmed live scan-record name**, not the cached device name (the module space-pads the name;
  the cached name is often stale).
- **Aggressive OEM power management** (e.g. OxygenOS) **freezes the app when the screen turns off** mid-run,
  silently halting the work thread. The runner holds `FLAG_KEEP_SCREEN_ON` to prevent this. Such builds also
  block `adb pm grant` / `settings put` from the shell, so grant Nearby-devices through the UI once.
- **Connection takes time on older stacks.** Some phones need `autoConnect=true` (a slow background connect)
  rather than a direct connect; the app selects this per device.

## Interpreting the result

The current board-side loopback uses **polled** UART RX. Because the MCU UART has a 1-byte RX register and
the echo write blocks while the next byte arrives, the polled echo drops a large fraction of bytes even at
low offered rates, so the measured throughput reflects that polled implementation rather than the module's
true ceiling (which is UART-bound near ~870 B/s at 9600). DMA or interrupt-driven RX on the board is the
path to measuring the real ceiling and to a usable high-rate link.
