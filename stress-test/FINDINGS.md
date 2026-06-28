# BLE link stress test — findings

Status: **resolved.** The Tier-3 walk failure is the **OnePlus 8's BT stack** failing to establish a
working data connection to the CC2541 — not the module, not the firmware, not the app, not L3. The full
BLE stack (C-firmware byte echo + Android app + L2 framing) is **proven correct end-to-end** on a second
central (ASUS ROG): 20/20 frames, 0 loss, stable, no 5 s drop.

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
