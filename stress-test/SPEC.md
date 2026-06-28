# BLE link stress test — isolate the CC2541 link from L3

## Purpose

The Tier-3 discovery walk fails 0/3 on silicon: an empty walk plus a constant ~5 s GATT drop. The
cause is ambiguous between the **BLE link** (CC2541 radio bring-up + the fixed-size frame transport,
app↔CC2541↔GD32) and the **L3 Rust firmware** (`crates/net` walk on `runtime-hal`). This harness strips
L3 (and Rust, and `runtime-hal`) out entirely: a minimal **C** firmware (GD32 SPL, no Rust) that brings
up the module and byte-echoes frames, plus an Android app that bounces numbered fixed-size frames and
measures loss / latency / connection stability. If the raw link is reliable here, the failure lives in
L3/Rust; if the ~5 s drop reproduces here, it is a link/connection-params problem.

Doing the firmware in **C against the SPL** (not the Rust `ble-loopback-test`) is the point: it removes
`runtime-hal`'s USART/polled-serial and the Rust toolchain from the equation, so a clean result here
also clears those.

## Questions this must answer (FINDINGS.md)

1. **AT bring-up:** does a fresh C AT bring-up reliably handshake the CC2541 on **each cold boot**
   (→ `MODE=DATA` does not persist across power-cycle, and the Rust firmware's `rx_total=0` was a
   different bug), or does it boot AT-deaf (→ the module is genuinely stuck)?
2. **Raw link:** does the link bounce fixed-size frames reliably (low loss, stable connection), or
   drop at ~5 s / lose frames? I.e. does the ~5 s drop reproduce with **no L3** (→ link/params) or not
   (→ L3/Rust-firmware)?
3. **Drop character:** drops at ~5 s under **idle** but holds under **sustained** traffic →
   supervision-timeout / keep-alive (fix: `CON_INTERVAL` + app keep-alive). Drops even under load →
   bad connection params at bring-up.

## The wire contract (shared by firmware + app)

One L2 stream frame, byte-identical to `crates/link` (Rust) and `Hoverboard/.../net/l2/` (Kotlin):

```
[ SOF=0x5A : 1 ][ len : 1 ][ frag-hdr : 1 ][ chunk : len-1 ][ CRC16 : 2 (LE) ]
```

- `len` = bytes from frag-hdr through end of chunk = `1 + chunk.len()`, range 1..255.
- `frag-hdr` = `0x00` for a single un-fragmented frame (MORE=0, PID=0, FRAG_IDX=0).
- `CRC16` = **CRC-16/MODBUS** (reflected poly `0xA001`, init `0xFFFF`, no final xor), computed over
  `[SOF, len, frag-hdr, chunk]`, stored little-endian (low byte first).
- BLE frame capacity = **16** (`BLE_FRAME_CAP`): L2 frame ≤ 16 B (frag-hdr + ≤15 chunk) → wire frame
  ≤ 20 B = one ~20-byte BLE ATT write. The CC2541 bridge coalesces/re-chunks both directions, so the
  decoder is a continuous-stream SOF/len/CRC framer with resync, never one-frame-per-notification.

References: `crates/link/src/framer.rs` (SOF/len/CRC), `crates/link/src/frag.rs` (frag-hdr bits),
`crates/base/src/crc16.rs`, `Hoverboard/.../net/l2/{StreamFramer,Crc16,FragHdr,BleStreamTransport}.kt`.

The app uses the existing Kotlin `StreamFramer`/`Crc16` to encode/decode (so it is byte-for-byte the
real link, exercising the same re-chunking + CRC resync). The firmware does **not** parse frames to
echo: it is a byte-faithful echo (every RX byte echoed unmodified, in order), which is the most direct
raw-link test and naturally reproduces the re-chunking. It parses the SOF/len stream only to *count*
whole frames for the diagnostic block.

## C firmware — `stress-test/firmware/`

GD32F103 (Cortex-M3), GigaDevice SPL, `arm-none-eabi-gcc`, no Rust, no L3, no `runtime-hal`.

- **Build:** a `Makefile` driving `arm-none-eabi-gcc` (`-mcpu=cortex-m3 -mthumb`,
  `-DUSE_STDPERIPH_DRIVER`, the F103 density define from `gd32f10x.h`), compiling the firmware `main.c`
  + the needed SPL sources (`gd32f10x_rcu.c`, `gd32f10x_gpio.c`, `gd32f10x_usart.c`,
  `system_gd32f10x.c`) + a startup `.S` against a linker script, `objcopy -O binary` → `.bin`.
  - SPL source via a `Makefile` variable defaulting to the sibling submodule (mirrors the existing
    `../../../runtime-hal` coupling): `GD_SPL ?= ../../../runtime-hal/third_party/GD32Firmware/GD32F10x`.
    Include `.../GD32F10x_standard_peripheral/Include` + `.../CMSIS/GD/GD32F10x/Include` + `.../CMSIS`.
  - Startup + linker: mirror `runtime-hal/harness/build_assets/gd-spl/gd32f10x/{startup.S,link.ld}`
    (FLASH @ 0x08000000, SRAM @ 0x20000000). Vendor copies into `stress-test/firmware/` so the build is
    self-contained except for the SPL `GD_SPL` path. Build runs on the Mac.
  - **Host gate:** `make` produces `stress-test/firmware/build/stress.bin` (and `.elf`) clean, no
    warnings-as-errors violations. Keep the build hermetic to `GD_SPL`.
- **Clock:** 72 MHz from internal IRC8M via PLL (= the Rust firmware's `REFERENCE_72M_IRC8M`:
  PLL = (IRC8M/2)×18, 2 flash wait states), so the USART baud divisor matches the real firmware.
  Reference: `runtime-hal/harness/build/clock_tree_72m_irc8m_f10x/...snippet.c`.
- **USART:** **USART2, PB10 (TX) / PB11 (RX), 9600 8N1** — same pins/baud as the hoverboard
  (`crates/firmware/src/main.rs`: USART2 PB10/PB11, `BT_BAUD = ble::at::BAUD = 9600`). Enable
  `RCU_GPIOB` + `RCU_USART2`, PB10 = `GPIO_MODE_AF_PP`, PB11 = `GPIO_MODE_IN_FLOATING`.
- **AT bring-up:** mirror `crates/ble::Module::bring_up` exactly (it is grounded on the stock-firmware
  decompile). Fresh bring-up on **every** boot (the hypothesis is `MODE=DATA` does not survive a cold
  power-cycle, so a cold module answers AT):
  - State 0: send `AT\r\n`, advance only on the **exact 7-byte** `AT+OK\r\n`, else resend; retry a
    generous budget (≈ the Rust `BLE_PROBE_ATTEMPTS`=16 × ~248 ms after a ~500 ms cold-boot settle), and
    **drain RX promptly** (poll faster than the ~1 ms/byte wire rate so the 1-byte RX register never
    overruns) — a blind `delay` loses the ack and the config silently never takes.
  - `AT+NAME=hb-stress\r\n`, `AT+CON_INTERVAL=16\r\n`, `AT+ADV_INTERVAL=32\r\n`, `AT+SET=1\r\n` (→
    advertises), `AT+MODE=DATA\r\n` (→ transparent). `SET=1` before `MODE=DATA` (order is load-bearing).
    Drain each command's `AT+OK\r\n` ack; keep the `MODE=DATA` drain short, then stop reading (the
    module is transparent after, so further bytes are data, not acks).
  - Bump `AT+NAME` per bench run if a scanner shows a cached name for the (fixed) module MAC.
- **Echo loop (after bring-up):** busy-spin polling USART2; for every received byte, echo it back
  unmodified (write to TDATA), in order. Count whole `0x5A`/len-framed frames echoed for the diag (the
  echo itself does not depend on the parse). On each byte check the USART overrun flag and count it.
- **Safety:** busy-spin, **NEVER `wfi`** (GD32 SWD-lockout rule); no motor code, nothing arms a bridge.
- **SWD diagnostic block:** a `#[repr(C)]`-equivalent C struct at a fixed un-mangled symbol
  (`BLE_STRESS_OBS`, modeled on the Rust `BLE_PROBE_OBS`), readable over SWD (`nm <elf> | grep
  BLE_STRESS_OBS`). Fields (all `u32` unless noted), written live during bring-up + echo:
  - `magic` (a fixed live-marker), `at_attempts`, `at_matched_attempt` (1-based, 0=never),
    `at_answered` (1/0), `at_rx_total`, a small captured-RX byte buffer (spot `AT+OK\r\n` vs garbage),
    `frames_echoed`, `rx_bytes_total`, `rx_overruns`.

## Android app — `stress-test/app/`

A minimal app reusing the **production** BLE path so it faithfully reproduces the real walk's failing
link (the existing `crates/ble/test-harness` app uses a *different* raw-`android.bluetooth` stack and so
would not reproduce the Nordic-stack drop; it is the template only for the **headless driver mechanics**,
not the transport).

- **Reuse from `Hoverboard/app`:** `BleHoverboardTransport` (Nordic scan→autoConnect=true→discover→pick
  0x1001 write / 0x1002 notify→`requestConnectionPriority(HIGH)`), `LinkConfig` (advertised name),
  `ConnectionState`, the `net/l2` framer (`StreamFramer`, `Crc16`, `FragHdr`, `BleStreamTransport`), and
  the manifest permissions + the `pm grant` flow. Copy the minimal set into the new app (or depend on a
  shared module); do **not** pull in the L3 `net/l3` walk. Same gradle stack as `Hoverboard/app`
  (AGP 8.9.1, Kotlin 2.1.20, Nordic ble 1.3.1, compile/target SDK 35, min 26).
- **Headless driver:** a `RunnerActivity` (no real UI) driven over adb intent extras and reporting to a
  logcat tag (`BLE_STRESS`), modeled on `crates/ble/test-harness`'s `RunnerActivity` + `run-sweep.sh`.
  `am start` extras: `name` (default `hb-stress`), `mode` (`roundtrip`|`sustained`), `n`/`dur`,
  `rate`, `prio`, `out` (JSON). Permissions granted rootlessly via `pm grant`.
- **Modes:**
  - **Round-trip:** send a numbered fixed-size frame (4-byte big-endian seq + filler in the ≤15-byte
    chunk, `StreamFramer`-encoded → 20-byte ATT write), await its echo, verify seq + payload + CRC,
    time the round trip; repeat N frames / a fixed duration.
  - **Sustained:** stream frames back-to-back and verify all echo back in order — exposes under-load
    loss / corruption / re-chunking. The seq makes loss and reordering detectable; the `StreamFramer`
    decoder counts CRC-failure resyncs.
- **Metrics → logcat** (machine-readable `RESULT {json}` + `DONE ...`, like the test-harness):
  frames sent / echoed / lost, loss %, round-trip latency min/avg/max, throughput, and **connection
  stability** — time-to-disconnect + frames-before-disconnect (does it drop at ~5 s like the walk? does
  sustained traffic prevent it?). Stability is the discriminator for Q2/Q3.
- **Build gate:** `./gradlew :app:assembleDebug` clean.

## Bench protocol (mechanics; `~/notes/bench-overview.md` is canonical for drift)

- **Power:** the master/slave pair runs off the 25 V PSU through the Pi-driven relay on **GPIO4**.
  ON = `pinctrl set 4 op dl`, OFF = `pinctrl set 4 op dh` (`pinctrl get 4`: `lo`=on, `hi`=off). Cutting
  it power-cycles the CC2541 out of data mode. **Cold-cycle before every run:** OFF ≥5 s → ON, ~3 s
  settle.
- **Host/flash:** build on the Mac; flash on the Pi `pi@192.168.0.248` via the system openocd. Master
  GD32F103 SWD at USB `1-1.2.4` (ST-Link `0483:3748`, dapdirect_swd, `CPUTAPID 0`). Hold the bench lock
  (`tools/bench-lock.sh`) before any flash/SWD/scan.
- **Advertising ground-truth:** confirm `hb-stress` is advertising with the independent ESP32-C3
  scanner (`~/dev/esp-ble-scan`) before each app run.
- **Phone:** OnePlus 8 at `192.168.0.201` (the round-trip-stable phone; ASUS drops on supervision
  timeout). Wireless-debug port drifts — resolve per `~/.claude/CLAUDE.md` (masscan) if adb is stale.
- **SWD read-back:** read `BLE_STRESS_OBS` over SWD (openocd `mdw` at the `nm`-resolved symbol address)
  to characterize the firmware side (AT answered? frames echoed? overruns?).

## Test plan (slices, smallest-first; each signs off independently)

- **Slice 1 — C firmware builds + brings up + advertises.**
  - Gate H (host): `make` → `stress.bin`/`.elf` clean.
  - Gate S (silicon): cold-cycle, flash via openocd, ESP scanner sees `hb-stress` advertising, and
    `BLE_STRESS_OBS` over SWD shows `at_answered=1` with a sane `at_matched_attempt`. Repeat the
    cold-cycle ≥3× to answer Q1 (does fresh AT bring-up handshake on every cold boot?).
- **Slice 2 — Android app builds + round-trips.**
  - Gate H: `./gradlew :app:assembleDebug` clean.
  - Gate S: OnePlus scans/connects `hb-stress`, round-trip mode bounces N frames; `BLE_STRESS` logcat
    reports loss % + RTT; `BLE_STRESS_OBS` shows `frames_echoed` advancing, `rx_overruns` sane.
- **Slice 3 — Sustained + stability + FINDINGS.**
  - Gate S: run round-trip (idle-ish) and sustained back-to-back; record time-to-disconnect +
    frames-before-disconnect for both; cold-cycle each run. Write `stress-test/FINDINGS.md` answering
    Q1–Q3 with the numbers and a clear verdict pinning where the Tier-3 walk failure lives.

## Sign-off

Each slice's gates re-verified **independently by the audit agent** (re-run the host build, re-flash,
re-run the app on a fresh cold-cycle, re-read `BLE_STRESS_OBS` over SWD, reproduce the metrics) — not
trusting the implement agent's report. Commit post-audit (C firmware builds clean; Android build clean).
FINDINGS.md is the deliverable: the verdict on Q1–Q3 with reproducible numbers.
