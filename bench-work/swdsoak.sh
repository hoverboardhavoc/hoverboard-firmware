#!/usr/bin/env bash
# Arm and drive one bench board entirely over the SWD mailbox, with no phone involved.
#
# Two holds are needed and they are different: INPUTS carries the arm LEVEL and its mirror expires
# 1500 ms after the last one, while DRIVE_CMD carries the demand and decays in 200 ms. The drive
# tool caps a hold at 60 s, so it is relooped; the gap between chunks coasts the wheel for about a
# second, which is the cost of not having a tool that holds both.
set -euo pipefail
PORT="${1:?port}"; VALUE="${2:-6000}"; SECS="${3:-1800}"
B=/Users/hoverboardhavoc/dev/hoverboard-firmware/crates/swd-bridge/target/aarch64-apple-darwin/release
end=$(( $(date +%s) + SECS ))
"$B/swd-mailbox-inputs" "127.0.0.1:$PORT" --buttons 1 --hold "$SECS" >/dev/null 2>&1 &
armpid=$!
trap 'kill $armpid 2>/dev/null || true' EXIT
while [ "$(date +%s)" -lt "$end" ]; do
  "$B/swd-mailbox-drive" "127.0.0.1:$PORT" --value "$VALUE" --hold 60 --period-ms 100 >/dev/null 2>&1 || true
done
