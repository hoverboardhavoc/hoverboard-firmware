#!/usr/bin/env bash
# run-roundtrip.sh: host driver for the BLE link stress test, round-trip mode (Slice 2).
#
# Grants the BLE runtime permissions rootlessly (pm grant), launches the headless RunnerActivity over
# `am start` with the round-trip extras, then tails the BLE_STRESS logcat to the DONE line and prints
# the RESULT json. Pull the saved json afterwards if you want the file.
#
# The OnePlus 8's wireless-debug port drifts (random 30000-49999); resolve it first and pass --serial:
#   sudo -n masscan 192.168.0.201 -p 30000-49999 --rate 10000   # find the port
#   adb connect 192.168.0.201:<port>
#   ./run-roundtrip.sh --serial 192.168.0.201:<port> --n 200
#
# Usage:
#   ./run-roundtrip.sh --serial 192.168.0.201:41234 [--name hb-stress] [--n 200] [--dur 0]
#                      [--chunk 15] [--rate 0] [--connprio none] [--bond false]
#                      [--autoconnect true] [--out run.json]
set -euo pipefail

PKG=com.hoverboard.stress
ACT="$PKG/.RunnerActivity"
TAG=BLE_STRESS

SERIAL=""
NAME=hb-stress
N=200
DUR=0
CHUNK=15
RATE=0
CONNPRIO=none      # none | low | balanced | high (post-connect requestConnectionPriority lever)
BOND=false
AUTOCONN=true      # opportunistic connect (production default); false = direct/aggressive
WRITE=nores        # nores (WRITE_WITHOUT_RESPONSE) | res (WRITE, with ATT response)
OUT=run.json

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial) SERIAL="$2"; shift 2;;
    --name)   NAME="$2";   shift 2;;
    --n)      N="$2";      shift 2;;
    --dur)    DUR="$2";    shift 2;;
    --chunk)  CHUNK="$2";  shift 2;;
    --rate)   RATE="$2";   shift 2;;
    --connprio) CONNPRIO="$2"; shift 2;;
    --bond)   BOND="$2";   shift 2;;
    --autoconnect) AUTOCONN="$2"; shift 2;;
    --write)  WRITE="$2";  shift 2;;
    --out)    OUT="$2";    shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

if [[ -z "$SERIAL" ]]; then
  echo "ERROR: pass --serial <adb-endpoint>. The OnePlus wireless-debug port drifts; resolve it first" >&2
  echo "  (sudo -n masscan 192.168.0.201 -p 30000-49999 --rate 10000; adb connect ...)." >&2
  exit 2
fi

ADB=(adb -s "$SERIAL")

echo "== rootless grants =="
"${ADB[@]}" shell pm grant "$PKG" android.permission.BLUETOOTH_SCAN 2>/dev/null || true
"${ADB[@]}" shell pm grant "$PKG" android.permission.BLUETOOTH_CONNECT 2>/dev/null || true
"${ADB[@]}" shell pm grant "$PKG" android.permission.ACCESS_FINE_LOCATION 2>/dev/null || true
"${ADB[@]}" shell settings put secure location_mode 3 2>/dev/null || true
"${ADB[@]}" shell svc bluetooth enable 2>/dev/null || true

# Force-stop any prior instance: Android throttles a process to "opportunistic" scans after 5 scan
# starts in 30 s, so a reused process stops seeing adverts. A fresh process resets the budget.
"${ADB[@]}" shell am force-stop "$PKG" >/dev/null 2>&1 || true

# Wake the screen + dismiss the keyguard: OxygenOS blocks an unfiltered BLE scan while the screen is off
# ("Cannot start unfiltered scan in screen-off") and FREEZES the backgrounded process. The activity's
# FLAG_KEEP_SCREEN_ON only holds once it is foreground+visible, so the screen must be on at launch.
"${ADB[@]}" shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1 || true
"${ADB[@]}" shell wm dismiss-keyguard >/dev/null 2>&1 || true
sleep 1

echo "== round-trip run: name=$NAME n=$N dur=$DUR chunk=$CHUNK rate=$RATE connprio=$CONNPRIO bond=$BOND autoconnect=$AUTOCONN write=$WRITE =="
"${ADB[@]}" logcat -c
"${ADB[@]}" shell am start -n "$ACT" \
  --es mode roundtrip --es name "$NAME" \
  --ei n "$N" --ei dur "$DUR" --ei chunk "$CHUNK" --ei rate "$RATE" \
  --es connprio "$CONNPRIO" --ez bond "$BOND" --ez autoconnect "$AUTOCONN" \
  --es write "$WRITE" --es out "$OUT" >/dev/null

# Block until DONE (the run can take a while: autoConnect scan + N round trips), then dump the run lines.
"${ADB[@]}" logcat -m 1 -e 'DONE status=' "$TAG:I" '*:S' >/dev/null 2>&1 || true
"${ADB[@]}" logcat -d "$TAG:I" '*:S' 2>/dev/null | grep -aE 'START|RESULT|DONE' || true

echo "== pull result json =="
"${ADB[@]}" pull "/sdcard/Android/data/$PKG/files/$OUT" "./$OUT" >/dev/null 2>&1 \
  || "${ADB[@]}" exec-out run-as "$PKG" cat "files/$OUT" > "./$OUT" 2>/dev/null || true
[[ -s "./$OUT" ]] && { echo "-- $OUT --"; cat "./$OUT"; echo; }
