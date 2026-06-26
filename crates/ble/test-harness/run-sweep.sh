#!/usr/bin/env bash
# run-sweep.sh: the host runner for the ADB BLE-throughput harness (specs/ble.md, "Drive mode" +
# "ADB-driven throughput test harness"). It issues the `am start` intents, waits for each run's DONE line
# on the BLE_TPUT logcat tag, collects the result JSON, and prints the knee (the first offered rate where
# round-trip loss appears).
#
# It drives EITHER bench phone, selected by the logical device name (NOT by address: DHCP drifts the IPs).
# The device is resolved to its adb endpoint here, and `--es device <name>` is passed into the app so the
# app picks the right BLE connect quirk (the ASUS ROG needs autoConnect=true; see Devices.kt).
#
#   OnePlus 8  (IN2013)     adb at 192.168.0.201 (random wireless-debug port; discover or pass ADB_SERIAL)
#   ASUS ROG   (ASUS_Z01RD) adb at 192.168.0.141:5555 (fixed TCP port)
#
# Rootless. Run the one-time per-device grants below first (also rootless).
#
# Usage:
#   ./run-sweep.sh --device OnePlus --serial 192.168.0.201:43210
#   ./run-sweep.sh --device Asus    --serial 192.168.0.141:5555
#   ./run-sweep.sh --device Asus    --serial 192.168.0.141:5555 --axis rate --payload 64
#
set -euo pipefail

PKG=com.hoverboard.bletest
ACT="$PKG/.RunnerActivity"
TAG=BLE_TPUT

DEVICE=OnePlus
SERIAL=""
AXIS=rate          # which axis to sweep: rate | payload
PAYLOAD=64
MTU=247
PRIO=high
WRITE=nores        # nores (WRITE_WITHOUT_RESPONSE) | res (WRITE)
DUR=10
MODE=loopback      # loopback (Tier 3 board) | fake (Tier 2 local echo, no board)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)  DEVICE="$2"; shift 2;;
    --serial)  SERIAL="$2"; shift 2;;
    --axis)    AXIS="$2"; shift 2;;
    --payload) PAYLOAD="$2"; shift 2;;
    --mtu)     MTU="$2"; shift 2;;
    --prio)    PRIO="$2"; shift 2;;
    --write)   WRITE="$2"; shift 2;;
    --dur)     DUR="$2"; shift 2;;
    --mode)    MODE="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ -z "$SERIAL" ]]; then
  echo "ERROR: pass --serial <adb-endpoint>. DHCP drifts the wireless-debug port; resolve it first" >&2
  echo "  (e.g. masscan the OnePlus, or use the ASUS's fixed 192.168.0.141:5555), then re-run." >&2
  exit 2
fi

ADB=(adb -s "$SERIAL")

echo "== one-time rootless grants for $DEVICE ($SERIAL) =="
"${ADB[@]}" shell pm grant "$PKG" android.permission.BLUETOOTH_SCAN 2>/dev/null || true
"${ADB[@]}" shell pm grant "$PKG" android.permission.BLUETOOTH_CONNECT 2>/dev/null || true
# Pre-Android-12 only (e.g. the ASUS ROG on Android 8): location for the BLE scan.
"${ADB[@]}" shell pm grant "$PKG" android.permission.ACCESS_FINE_LOCATION 2>/dev/null || true
"${ADB[@]}" shell settings put secure location_mode 3 2>/dev/null || true
"${ADB[@]}" shell svc bluetooth enable 2>/dev/null || true

# The axis values to sweep (specs/ble.md, "What it sweeps").
case "$AXIS" in
  rate)    VALUES=(10 25 50 100 200 400);;   # offered packets/sec, ramped until loss begins (the knee)
  payload) VALUES=(4 16 64 128 244 255);;    # payload bytes (244 ~ the MTU-3 cap at MTU 247)
  *) echo "unknown --axis: $AXIS" >&2; exit 2;;
esac

run_one() {
  local rate="$1" payload="$2" out="$3"
  # Clear logcat, launch the headless run, then wait for the DONE line on our tag.
  "${ADB[@]}" logcat -c
  "${ADB[@]}" shell am start -n "$ACT" \
    --es device "$DEVICE" --es mode "$MODE" \
    --ei payload "$payload" --ei rate "$rate" --ei dur "$DUR" \
    --es write "$WRITE" --ei mtu "$MTU" --es prio "$PRIO" \
    --es out "$out" >/dev/null

  # Block until DONE; the run is `dur` seconds plus connect/scan slack (the ASUS autoConnect can be ~133 s).
  local line
  line="$("${ADB[@]}" logcat -m 1 -e 'DONE status=' "$TAG:I" '*:S' 2>/dev/null || true)"
  echo "$line"
}

echo "== sweep axis=$AXIS device=$DEVICE mode=$MODE =="
KNEE=""
for v in "${VALUES[@]}"; do
  if [[ "$AXIS" == rate ]]; then rate="$v"; payload="$PAYLOAD"; else rate=50; payload="$v"; fi
  out="run_${AXIS}_${v}.json"
  echo "-- run $AXIS=$v --"
  done_line="$(run_one "$rate" "$payload" "$out")"
  echo "   $done_line"
  # Pull the result JSON for the record (scoped storage: fall back to run-as if a plain pull is blocked).
  "${ADB[@]}" pull "/sdcard/Android/data/$PKG/files/$out" "./$out" >/dev/null 2>&1 \
    || "${ADB[@]}" exec-out run-as "$PKG" cat "files/$out" > "./$out" 2>/dev/null || true

  # Knee detection: the first value whose DONE line shows nonzero loss (loss=L/S with L>0).
  if [[ -z "$KNEE" ]] && echo "$done_line" | grep -qE 'loss=[1-9][0-9]*/'; then
    KNEE="$v"
  fi
done

if [[ -n "$KNEE" ]]; then
  echo "== KNEE: loss first appears at $AXIS=$KNEE =="
else
  echo "== KNEE: no loss across the swept range (carrier kept up at every $AXIS point) =="
fi
