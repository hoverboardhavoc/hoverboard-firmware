#!/usr/bin/env bash
set -euo pipefail
PKG=com.hoverboard.stress; ACT="$PKG/.RunnerActivity"; TAG=BLE_STRESS
SERIAL="$1"; shift
# AUTOCONN=false is DELIBERATE and known-failing (direct connect: ~5 s supervision-timeout drop).
# autoconnect is the lever this script sweeps; production (and run-roundtrip.sh) default to true.
# An uncommented false default here is exactly how the FINDINGS update-(6) drop saga started.
NAME=hb-stress; N=150; CHUNK=15; CONNPRIO=none; BOND=false; AUTOCONN=false; OUT=lever.json
while [[ $# -gt 0 ]]; do case "$1" in
  --n) N="$2";shift 2;; --connprio) CONNPRIO="$2";shift 2;; --bond) BOND="$2";shift 2;;
  --autoconnect) AUTOCONN="$2";shift 2;; --out) OUT="$2";shift 2;; --name) NAME="$2";shift 2;;
  *) echo "unknown $1">&2; exit 2;; esac; done
ADB=(adb -s "$SERIAL")
"${ADB[@]}" shell pm grant "$PKG" android.permission.BLUETOOTH_SCAN 2>/dev/null||true
"${ADB[@]}" shell pm grant "$PKG" android.permission.BLUETOOTH_CONNECT 2>/dev/null||true
"${ADB[@]}" shell pm grant "$PKG" android.permission.ACCESS_FINE_LOCATION 2>/dev/null||true
"${ADB[@]}" shell svc bluetooth enable 2>/dev/null||true
"${ADB[@]}" shell am force-stop "$PKG" >/dev/null 2>&1||true
"${ADB[@]}" shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1||true
"${ADB[@]}" shell wm dismiss-keyguard >/dev/null 2>&1||true
sleep 1
echo "== run: connprio=$CONNPRIO bond=$BOND autoconnect=$AUTOCONN n=$N =="
"${ADB[@]}" logcat -c
"${ADB[@]}" shell am start -n "$ACT" --es mode roundtrip --es name "$NAME" \
  --ei n "$N" --ei dur 0 --ei chunk "$CHUNK" --ei rate 0 --es write nores \
  --es connprio "$CONNPRIO" --ez bond "$BOND" --ez autoconnect "$AUTOCONN" --es out "$OUT" >/dev/null
"${ADB[@]}" logcat -m 1 -e 'DONE status=' "$TAG:I" '*:S' >/dev/null 2>&1||true
"${ADB[@]}" logcat -d "$TAG:I" BleWire:I '*:S' 2>/dev/null | grep -aE 'RESULT|DONE|BONDED|createBond|bond ' | head
