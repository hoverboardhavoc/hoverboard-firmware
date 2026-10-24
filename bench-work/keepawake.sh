#!/usr/bin/env bash
# Hold the phone's Activity resumed for a long armed run.
#
# The app disarms the boards on purpose when it is backgrounded (MainActivity.DisarmWhenBackgrounded),
# and a display timeout is a background event, so an unattended armed run ends after the screen
# timeout (300 s on this phone) with nothing in the data saying so. `svc power stayon true` is
# SIGKILLed here and the secure setting needs a permission adb does not have, so the remedy left is
# to keep generating user activity. KEYCODE_WAKEUP changes no app state.
set -euo pipefail
SERIAL="${PHONE:-192.168.0.201:5555}"
SECS="${1:-7200}"
end=$(( $(date +%s) + SECS ))
while [ "$(date +%s)" -lt "$end" ]; do
  adb -s "$SERIAL" shell input keyevent KEYCODE_WAKEUP >/dev/null 2>&1 || true
  sleep 120
done
