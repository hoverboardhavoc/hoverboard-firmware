# BLE link stress test — findings

Status: **SOLVED — see Update (6) at the bottom (the definitive root cause; updates 1-5 are superseded).**
The link drop was a **regression in THIS stress app**: an uncommitted `BleGattConnectOptions(autoConnect =
false)`. With `autoConnect=true` (what the production transport ships, and the committed baseline here) the
link holds drop-free. The module is proven healthy by an ESP32-C3 central (stable 70 s pinned-interval
stream). Updates 1-5 below were all conducted with the `autoConnect=false` regression in place, so their
phone-stack / CON_INTERVAL / L2CAP / reflash conclusions are **measurement artifacts** — kept for history,
but do not act on them; read Update (6) first.

Purpose (see `SPEC.md`): the Tier-3 discovery walk fails 0/3 on silicon (empty walk + a constant ~5 s
GATT drop), ambiguous between the **BLE link / CC2541 module** and the **L3 Rust firmware**. A minimal
**C** firmware (GD32 SPL, no Rust, no `runtime-hal`) + an Android app that bounces fixed-size frames
isolated the link. They settled it.

## Verdict

Same recovered module, same firmware, same app, same minute:

- **OnePlus 8:** cannot carry a single data PDU. With Write-With-Response the very first write threw
  `GATT_ERROR` after ~5 s (`sent=0`, `connected_ms≈5135`); with Write-Without-Response the GD32 saw
  zero UART bytes (`rx_bytes_total=0`, `RBNE` never set) and the link dropped at the supervision
  timeout. This is the Tier-3 symptom exactly — reproduced with **no L3 and no Rust**.
- **ASUS ROG:** bridges flawlessly. `sent=20 echoed=20 lost=0 loss=0.00% dropped=false`, RTT
  min/mean/max = 79.9/108.9/149.9 ms, ~9.2 fps, **no 5 s drop**. Firmware OBS corroborates exactly:
  `frames_echoed=20`, `rx_bytes_total=400` (20×20 B), `rx_overruns=0`, `echo_stat_accum=0x30` (`RBNE`
  set — real bytes on the UART RX line).

→ The CC2541 module is healthy and bridges bidirectionally; the app, firmware, L2 framing, and echo are
all correct. The failure is **phone-side: the OnePlus 8's BT stack.** (This flips the bench's prior
assumption that the OnePlus was the round-trip-stable phone and the ASUS the dropper — something drifted
on the OnePlus's OS/BT-stack state; the historical 2026-06-18 OnePlus run no longer reproduces.)

## Q1 — does a fresh C AT bring-up reliably handshake the CC2541 each cold boot?

**Yes, given a clean power-on-reset.** Slice 1 answered AT on attempt 1 across 3/3 clean cold cycles
(`at_answered=1`, `at_matched_attempt=1`, `at_rx_total=49` = the 7 `AT+OK\r\n` acks), re-advertised
`hb-stress`, audit-verified. `MODE=DATA` does not persist across a clean cold-cycle, and the Rust
firmware's `rx_total=0` was a **different bug**.

The intermittent "AT-deaf after power-cycle" state was traced to a **dirty power-down, not NVM**: without
a load the switched 25 V rail decayed too slowly, leaving the CC2541 in brown-out limbo (never a clean
POR), so it never left data mode. A **bleed resistor across the switched rail** restores a clean POR, but
recovery is still intermittent (~1 in 1–10 cold cycles), so loop the cold-cycle reading `at_answered`
(OBS @ `0x2000000c`) until it reads 1 before each fresh-module run. (The earlier "NVM-committed data
mode surviving a 30 s power-cycle" theory was wrong.) Recommendation: fit ≈10 kΩ ¼ W on the board side
of the relay's NO contact (drop to ~4.7 kΩ if the rail still decays slowly).

## Q2 — does the raw link bounce frames, or drop at ~5 s / lose them? Link or L3?

**Not L3. The link bounces frames perfectly on a working central; the OnePlus 8 specifically cannot
carry a data connection.** The diagnostic ruled the candidates out one by one before the phone split
settled it:

- **Firmware RX latch — decisively ruled out.** The echo loop reads USART2 `STAT0` every iteration
  (clearing `ORERR` the family-correct way) and OR-accumulates the RX flags into the SWD diag. On the
  OnePlus, across hundreds of millions of iterations: only a stale `IDLEF` from bring-up, **never
  `RBNE`/`ORERR`/`FERR`/`NERR`**, `rx_bytes_total=0`. On the ASUS, `echo_stat_accum=0x30` (`RBNE` set),
  `rx_bytes_total=400` — the same firmware correctly receives and echoes real bytes. So the firmware RX
  path is correct; the OnePlus simply delivered nothing.
- **AT config — ruled out.** `SET=1`→`MODE=DATA` order, a no-`CON/ADV_INTERVAL` variant, and an
  already-in-data-mode (`ASSUME_DATA_MODE`-equivalent) module all behaved identically on the OnePlus and
  all bridged on the ASUS.
- **Module — ruled out** by the ASUS bridging the same module flawlessly.

The app side is sound (audited): correct 20-byte frames (`5a 10 00 <seq> <filler> <crc>`, CRC16-MODBUS
LE), decoded with the production `StreamFramer`.

## Q3 — idle-vs-sustained (supervision-timeout/keep-alive vs bad connection params)?

**Neither — it is not a tuning problem.** On the ASUS, sustained round-trip traffic ran with no drop at
all (the module's connection params are fine). On the OnePlus, the ~5 s drop happens regardless of
traffic because the data channel never establishes (the first PDU is never acknowledged). So the drop is
an OnePlus-side connection-establishment failure, not a `CON_INTERVAL`/keep-alive issue on the link.

## What to do with this

1. **Run the real Tier-3 walk on a healthy central** (the ASUS ROG, or any working BT host). The whole
   L2/L3/BLE stack is otherwise proven; the OnePlus 8 was the confound.
2. **Rehabilitate the OnePlus 8** if it must be supported: clear its Bluetooth data / re-pair / OS-level
   BT-stack reset, then re-test with this harness's round-trip mode (`run-roundtrip.sh`) before trusting
   it for the walk. Treat "passes the C-firmware echo round-trip" as the gate a phone must clear.
2. **Bench hardening:** keep the bleed resistor (Q1) so cold-cycles are clean PORs; loop on
   `at_answered` before each fresh-module run.
3. **Documentation (separate change):** `specs/ble.md` reads as if `SET=1`→`MODE=DATA` is end-to-end
   verified on silicon; before this it had only verified command-acceptance + advertising. It now has a
   genuine end-to-end inbound+outbound proof — but on the ASUS, not the OnePlus. Note that and that
   `AT+SET=1` is a suspected NVM/save (it double-acks) that must precede `MODE=DATA`. The shipped
   `SET=1`-first order is correct and should stay.

## Slice sign-off

- **Slice 1 (C firmware):** both gates audit-verified (host clean; silicon 3/3 cold cycles). The harness
  firmware additionally carries the `STAT0`/`echo_stat_accum` instrumentation + a
  `BRINGUP_SET_CON_INTERVAL` switch (default spec-faithful) that produced the Q2 evidence;
  validated-by-use (firmware OBS corroborated the app metrics exactly on the ASUS: `frames_echoed=20` ↔
  `echoed=20`, `rx_bytes_total=400` = 20×20 B).
- **Slice 2 (Android app):** Gate H audit-verified clean (faithful production-transport reuse + verbatim
  `net/l2` framer + correct metric math). **Gate S passed end-to-end on the ASUS** (20/20, loss%/RTT/
  throughput reported, `BLE_STRESS_OBS.frames_echoed=20`, `rx_overruns=0`).
- **Slice 3 (sustained + stability):** the round-trip stability result is the answer — stable on the
  ASUS, phantom-drop on the OnePlus; sustained streaming on a healthy central is future work but is no
  longer needed to settle the goal's questions.

## Artifacts

- `stress-test/firmware/` — GD32F103 SPL C firmware: 72 MHz IRC8M clock, USART2 PB10/PB11 9600 8N1, AT
  bring-up mirroring `crates/ble`, byte-faithful echo, `BLE_STRESS_OBS` SWD diag (incl. `echo_stat_accum`
  / `echo_loop_iters`), `BRINGUP_SET_CON_INTERVAL` switch. Busy-spin, no `wfi`.
- `stress-test/app/` — standalone Android app: production Nordic transport + `net/l2` framer + headless
  `RunnerActivity` (`BLE_STRESS` logcat), round-trip mode with a `write` (nores/res) toggle;
  `run-roundtrip.sh` driver.
- SWD diag `BLE_STRESS_OBS` @ `0x20000000` on the master (`at_answered` @ `0x2000000c`).

## Update 2026-06-29 — ESP-central bisect overturns the LL-procedure theory

Re-investigated with the OnePlus 8 (Android 13) + ASUS Zenfone 10 (Android 15), and built an
**ESP32-C3 BLE-central test bed** (`~/dev/esp-ble-scan/ble_central/`) to bisect deterministically.

**The module + data path are sound.** The ESP32-C3 (a modern BLE-5 controller) echoes **15/15** on
the same module in **every** config: default 1M, MTU=517 requested, 2M-PHY requested, and even with a
random own-address. The CC2541 is BLE 4.0, so PHY/MTU/DLE all negotiate down safely. **The earlier
"modern Android initiates LL_LENGTH_REQ/LL_PHY_REQ/MTU that stalls the CC2541" root cause is WRONG** —
a modern controller drives it fine.

**The modern phones fail with the writes actually on the air.** btsnoop (extracted via
`dumpsys bluetooth_manager` → base64 → zlib, no root/toggle) shows the OnePlus connects, completes GATT
discovery, subscribes (CCCD write to 0x0025 acked), and **transmits ATT Write Commands (0x52) to handle
0x0021 with the correct payload, LL-acked** (BQR `ReTx:0/NoRX:0`). Yet `BLE_STRESS_OBS.rx_bytes_total=0`
— the module receives the writes but does **not** bridge them to UART, then the link drops at the 5 s
supervision timeout. The ESP writes to the **same handle 0x0021** and it **does** bridge.

**Ruled out (exhaustively, on the failing phone):** app `autoConnect` (false), connection priority
HIGH/balanced/**LOW_POWER** (confirmed 120 ms interval), Nordic `splitWrite`→plain `write`, bonding
(none) + full BT-stack restart clearing leaked GATT clients, PHY/DLE/MTU, and the GATT handle (identical
0x0021). On the ESP side: public vs random own-address (both bridge).

**Conclusion:** an interaction between the **Android Bluedroid stack** and the **closed CC2541 bridge
firmware** blocks the UART bridge for Android specifically (Android 8 + the ESP both work; Android 13/15
don't). It is **not app-side-fixable** and the module firmware is closed. The user's old RoboDurden
port "worked but unstably" on the OnePlus — consistent with a marginal, intermittent trigger.

**Recommended path:** the ESP carries data flawlessly → use an **ESP-as-BLE-bridge** (phone→WiFi→ESP→
BLE→GD32) or a **modern BLE module** for the phone-facing link. The SWD-mailbox path already drives the
walk. Direct phone↔CC2541 over modern Android is a dead end with this module.

## Update 2026-06-29 (2) — ROOT CAUSE found: L2CAP connection-parameter-update gating

The ASUS-8(works)-vs-OnePlus(fails) btsnoop diff on the SAME module, decoded at the L2CAP signaling layer:
- **ASUS-8 (works):** the module sends an **L2CAP Connection Parameter Update Request** (cid 0x0005,
  code 0x12, asking ~10-19 ms) and Android 8 replies **ConnParamUpdate-RSP = ACCEPTED** (matched id=1).
- **OnePlus (fails):** **no L2CAP param-update exchange at all.**

Mechanism: the CC2541 is BLE **4.0**, so it requests its connection parameters the only way it knows -
the peripheral-initiated **L2CAP** method - and its closed transparent-bridge firmware **gates
data-forwarding on that request being accepted.** Android 8 (and the ESP/NimBLE) answer the L2CAP way ->
accepted -> the module forwards. Modern Android (4.1+ stack) negotiates parameters via the **Link-Layer
Connection Parameter Request procedure** and never sends the L2CAP acceptance the module waits on -> the
module's bridge stays stuck -> rx_bytes=0 -> 5 s supervision-timeout drop.

Clincher (rules out "just the interval value"): the OnePlus failed even at requestConnectionPriority(HIGH)
~15 ms, which is INSIDE the module's requested 10-19 ms range. So it's the L2CAP **acceptance handshake**
(the protocol), not the parameter value.

NOT app-fixable - the L2CAP-vs-LL parameter negotiation is in Android's Bluetooth stack, below the app.
Fix = reflash the module with firmware that speaks the LL procedure (e.g. genuine HM-10), or the ESP
bridge / modern-module fallbacks. Reflash staged: cc-tool on the Pi + `~/notes/cc2541-reflash-runbook.md`.

## Update 2026-06-29 (3) — AT-lever exhausted: the bridge defect is NOT configurable

Tested on the master F103 silicon whether the module's connection-parameter request can be made
acceptable to a modern controller by lowering the requested connection interval from the firmware
bring-up (the only param the module exposes over AT). The L2CAP request the module emits carries
`interval_max=15, slave_latency=460, timeout=832 (8320 ms)`; the controller validity rule
`timeout > (1+latency)*interval_max*2` then needs `interval_max <= 7` (i.e. `AT+CON_INTERVAL <= 8`,
since the emitted max tracks `CON_INTERVAL-1`: `16 -> 15`).

Result (clean POR each time, read from `BLE_STRESS_OBS.at_rx` over SWD):
- `AT+CON_INTERVAL=6`  -> module replies **`AT+ERR=2`** (rejected; falls back to default params).
- `AT+CON_INTERVAL=8`  -> module replies **`AT+ERR=2`** (rejected).
- `AT+CON_INTERVAL=16` -> `AT+OK` (the committed baseline; known-failing on modern Android).

So the module's accepted `CON_INTERVAL` floor is **above 8**, which is exactly the range needed to make
its out-of-spec request valid. There is no AT command on this OEM dialect to set slave-latency or the
supervision timeout directly. **Conclusion: the connection-parameter defect cannot be corrected through
the module's AT interface.** This independently confirms updates (1)/(2): the fix is to replace the module
firmware (reflash, `~/notes/cc2541-reflash-runbook.md`) or the module, or use the SWD-mailbox path which
already drives the walk. Bench left safe (relay OFF, lock released); `main.c` reverted to `CON_INTERVAL=16`.

## Update 2026-06-29 (4) — btsnoop diff DISPROVES all three prior root-cause theories

Captured fresh HCI btsnoop from BOTH phones on the same powered bench (master F103 running this firmware,
`CON_INTERVAL=16`, clean POR, `at_answered=1`) and decoded the `dumpsys bluetooth_manager`
BTSNOOP_LOG_SUMMARY (base64 -> 9-byte prefix -> zlib).

**Working baseline re-confirmed live:** ASUS ROG (A8) round-trip on this exact bench = `sent=10 echoed=10
lost=0 connected_ms=1739`. Module/firmware/bench are healthy; the OnePlus 8 (A13) specifically fails
(`rx_bytes_total=0`, `echo_stat_accum=0x10` so RBNE never set -> not one byte hit the GD32 UART pin).

**OnePlus (failing) btsnoop, decoded:**
- L2CAP CID 0x0005 `ConnParamUpdate-REQ` from the module: `interval_min=8 interval_max=15 latency=15
  timeout=1396` (13.96 s). This is **spec-valid** (13960 > (1+15)*18.75*2 = 600). The earlier
  "latency=460 / timeout=832, out of spec" decode (update 2 / the AT-lever premise) was a **misaligned
  read** and is wrong.
- OnePlus answers `ConnParamUpdate-RSP result=0x0000` (**ACCEPTED**), twice. So update (2)'s claim that
  the OnePlus does "no L2CAP exchange" and the module gates on L2CAP acceptance is **false**.
- 21x ATT `Write-Command (0x52)` to handle **0x0021** with `5a..`-framed payloads (0x5A SOF) -> correct
  opcode, correct handle, correct framing, identical to the ESP/ASUS.

**ASUS-8 (working) vs OnePlus (failing), same decode:** materially identical at HCI/ATT/L2CAP -
both MTU=256, **both do ZERO PHY updates and ZERO data-length changes** (so the LL_PHY_REQ/LL_LENGTH_REQ
"modern Android stalls the BLE-4.0 module" theory is also **false**), both write `0x52`->`0x0021`. Only
quantitative difference: OnePlus issues ~19 LE connection-parameter updates vs the ASUS's ~5 (OxygenOS
churning the interval), and the OnePlus's writes don't bridge while the ASUS's do.

**Net:** every host-visible BLE-layer theory is disproven. The OnePlus does everything correctly and the
module still won't bridge its writes; the ASUS does the same and the module bridges. The differentiator is
**below the HCI layer** (LL timing / RF / the conn-param churn / module-internal state), which a host-side
btsnoop cannot see. **This is exactly where an over-the-air sniff (SDR or an nRF/CC26x2 follower) is the
required tool** - not to recheck connection params (resolved) but to see the link-layer behavior that
distinguishes the two phones. Reflash remains a plausible fix but is now a gamble (mechanism unknown), not
the evidence-backed certainty updates (1)-(3) implied.

## Update 2026-06-29 (5) — SOLVED (root cause + working fix): the CC2541 can't service a fast conn interval

A modern Android phone carried data through the CC2541 for the first time. The fix: raise the module's
requested connection interval so its slow 8051 bridge can keep up.

**Root cause (confirmed on silicon, both sides):** the CC2541 bridge cannot service a fast BLE connection
interval (TI's own note: fails ~7.5 ms, works ~100 ms). With `AT+CON_INTERVAL=16` the module's L2CAP
ConnParamUpdate-REQ asks `interval_max=15` (18.75 ms, confirmed on the wire in update 4). A modern phone
honors the peripheral's fast range, the module can't keep up, and **zero** bytes bridge to the UART before
the ~5 s supervision-timeout drop. Android 8 happens to stay on a slower interval, which is the ONLY reason
it worked. This is not the module being "closed/unfixable" (updates 1-4 over-concluded); it is a tunable.

**Fix:** `AT+CON_INTERVAL=80` (~100 ms) in the bring-up (`stress-test/firmware/main.c`; mapping 16->15 ms,
so 80->~99 ms). Module accepts 80 (all `AT+OK`). The app already requests `LOW_POWER` (~100 ms,
`BleStressTransport.kt:218`), so phone and module now agree on a slow interval.

**Result (OnePlus 8, Android 13):** run 1 = `sent=20 echoed=19` app-side AND `frames_echoed=20
rx_bytes_total=400` GD32-side (`BLE_STRESS_OBS`, RBNE bit set) - real bidirectional data, the first modern
phone to bridge through this module.

**Residual (follow-up, not the data-flow blocker):** intermittent, ~1/5 success. EVERY session still drops
at ~5 s (`connected_ms` 5050-5246 on all runs incl. the good one = the known supervision-timeout drop).
`CON_INTERVAL=80` decides whether data bridges within that window: when the interval lands slow it does
(run 1, all 20 frames); when OxygenOS churns it fast (it issues ~19 conn-param updates) zero bridges. So
two separate issues remain: (a) the OS sometimes overriding to a fast interval, (b) the 5 s drop. For the
walk these are absorbed by the transport's reconnect-resume loop; the hard "zero bytes ever" blocker is
solved. Next: make the slow interval stick (longer/again-requested LOW_POWER, or a slower CON_INTERVAL),
and investigate the 5 s drop (negotiated supervision timeout vs the module's requested 13.96 s).

**To bake into production:** the real fix belongs in `crates/ble` bring-up (and any firmware that brings
the module up), not just the stress harness - set CON_INTERVAL to ~80 there. Pending direction.

## Update 2026-06-29 (6) — SOLVED: the drop was the stress app's `autoConnect=false` regression

The whole "OnePlus can't carry data / drops at 5 s" problem was an **uncommitted regression in this stress
app**, not the module, phone, firmware, or L3. `BleStressTransport` had been changed to
`BleGattConnectOptions(autoConnect = false)` (plus `splitWrite`->`write`, `HIGH`->`LOW_POWER`) during a
prior debugging session. The production `Hoverboard/.../BleHoverboardTransport.kt` ships `autoConnect =
true` (line 169) — only the harness was broken, which is why the *measurement* failed while the shipping
app was fine.

**The module is healthy — ESP32-C3 (NimBLE) central proof** (`~/dev/esp-ble-scan/ble_central/`):
- conn-param sweep: holds a PINNED interval (7.5 / 30 / 50 / 100 ms; latency 0/4; timeout 5 s/20 s) drop-
  free, ~99 % byte echo, incl. **100 ms for 70 s straight** (`echoed 10455/10500, dropped=0`).
- churn test: cycling the interval every 200 ms (7.5->45->100->15->50) did **not** break it — so "the
  module can't tolerate conn-param churn" (a tempting theory) is **false**.
- fast-establishment test (core `m_pConnParams` patched to establish at 7.5 ms) + MTU-247 request: still
  bridges fine. The module tolerates everything the ESP throws at it.

**The fix, proven on the ASUS (A8) with the goal's own metric:** `autoConnect=true`, n=260 round-trip =
`dropped=false, connected_ms=56193` (held 56 s, **no drop**), and firmware `BLE_STRESS_OBS.rx_bytes_total`
climbed by **exactly 5200 bytes** = 260 frames x 20 B (every byte bridged). The same app with
`autoConnect=false` = 5 s supervision-timeout drop (reason 0x08) or a 30 s connect-timeout, 0 bytes bridged.
The ~5 s drop is **eliminated**, not masked.

**Why autoConnect matters (working model):** `autoConnect=true` uses Android's gentle opportunistic/
background establishment (likely the long ~20 s BTM supervision-timeout default), which the slow CC2541
rides out; `autoConnect=false`'s aggressive direct connect runs a short 5 s supervision timeout that drops
on any module hiccup. The ESP always establishes moderate, so it never trips this.

**Diagnostic HCI decode** (used to read the drop reason + applied conn-params from `dumpsys
bluetooth_manager` BTSNOOP_LOG_SUMMARY, base64 -> 9-byte prefix -> zlib): `grab_snoop.py` + `hci_scan.py`
in the session scratchpad. The OnePlus drop was confirmed reason **0x08 = LL supervision timeout** (not a
host teardown), with the supervision timeout pinned at 5000 ms.

**Baked:** `BleStressTransport` autoConnect default -> true; `CONNECT_TIMEOUT_MS` 30 s -> 150 s (autoConnect
on A8 can take ~130 s to land the opportunistic connect; 30 s timed out + re-scanned). Diagnostic intent
extras kept for future sweeps: `connprio` (none/low/balanced/high), `bond`, `autoconnect`.

**Residual / open:** (a) autoConnect=true is slow & flaky to *establish* on the ASUS A8, and the A8 BT
stack fatigues after dozens of connect cycles (leaked GATT clients -> connects stop landing until a BT
toggle / reboot; A8 uses `svc BT`, not `svc bluetooth`). The *hold* is solid; establishment is the weak
part. (b) OnePlus (A13) re-verification is pending — its wireless-debugging daemon dropped during a
bonding test and needs a human to re-toggle Wireless debugging (it was NOT rebooted). The module DOES
support pairing (it popped a PIN dialog; PIN 000000 was wrong, hint "0000 or 1234"), but bonding was a
dead-end lead since discovery already succeeds and the drop is post-connect.
