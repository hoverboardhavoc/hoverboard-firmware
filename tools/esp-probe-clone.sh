#!/usr/bin/env bash
# Clone an ESP32 probe's flash to another ESP32, WITHOUT the operator (or any agent reading
# this session) ever seeing what is on it. The wireless-esp32-dap probe firmware carries the
# WiFi credentials COMPILED IN, so a byte clone is the credential-safe way to stand up a second
# probe: the dump exists only on the Pi, only for the duration of the copy, and is shredded
# after. Nothing is ever printed except sizes and hashes.
#
# Usage (both boards plugged into the Pi's USB):
#   tools/esp-probe-clone.sh --source /dev/ttyACM0 --dest /dev/ttyACM1
#   tools/esp-probe-clone.sh --list          # show candidate ESP ports and exit
#
# The SOURCE is the working probe (the one that already joins the WiFi); the DEST is the new
# board. Both must be the same chip family (the script refuses a mismatch: a C3 image on an S3
# will not boot). MAC addresses live in eFuse, not flash, so the clone gets its own DHCP lease.
set -euo pipefail
PI="${PI_HOST:-pi@192.168.0.248}"
SRC=""; DST=""; LIST=0
while [ $# -gt 0 ]; do
  case "$1" in
    --source) SRC="${2:?}"; shift 2 ;;
    --dest)   DST="${2:?}"; shift 2 ;;
    --list)   LIST=1; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "esp-probe-clone: unknown argument $1" >&2; exit 2 ;;
  esac
done

if [ "$LIST" = 1 ]; then
  ssh "$PI" 'for p in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$p" ] || continue;
    printf "%s  " "$p"; udevadm info -q property -n "$p" 2>/dev/null | sed -n "s/^ID_SERIAL=//p"; done'
  exit 0
fi
[ -n "$SRC" ] && [ -n "$DST" ] || { echo "esp-probe-clone: need --source and --dest (see --list)" >&2; exit 2; }
[ "$SRC" != "$DST" ] || { echo "esp-probe-clone: source and dest are the same port" >&2; exit 2; }

echo "esp-probe-clone: identifying both boards"
SRC_CHIP=$(ssh "$PI" "esptool --port $SRC chip_id 2>&1 | sed -n 's/^Chip is \(.*\)/\1/p' | head -1")
DST_CHIP=$(ssh "$PI" "esptool --port $DST chip_id 2>&1 | sed -n 's/^Chip is \(.*\)/\1/p' | head -1")
echo "  source $SRC: ${SRC_CHIP:-UNKNOWN}"
echo "  dest   $DST: ${DST_CHIP:-UNKNOWN}"
[ -n "$SRC_CHIP" ] && [ -n "$DST_CHIP" ] || { echo "esp-probe-clone: could not identify both boards" >&2; exit 1; }
[ "${SRC_CHIP%% (*}" = "${DST_CHIP%% (*}" ] || { echo "esp-probe-clone: REFUSED - chip families differ" >&2; exit 1; }

SIZE=$(ssh "$PI" "esptool --port $SRC flash_id 2>&1 | sed -n 's/^Detected flash size: //p' | head -1")
echo "esp-probe-clone: flash size $SIZE"
case "$SIZE" in 4MB) BYTES=4194304 ;; 8MB) BYTES=8388608 ;; 2MB) BYTES=2097152 ;; 16MB) BYTES=16777216 ;;
  *) echo "esp-probe-clone: unexpected flash size '$SIZE'" >&2; exit 1 ;; esac

# The dump lives in a mode-600 file on the Pi and is shredded in a trap, on every exit path.
echo "esp-probe-clone: reading source flash (contents are never displayed)"
ssh "$PI" "bash -s" "$SRC" "$DST" "$BYTES" <<'REMOTE'
set -euo pipefail
SRC="$1"; DST="$2"; BYTES="$3"
IMG=$(mktemp /tmp/espclone.XXXXXX.bin); chmod 600 "$IMG"
cleanup() { shred -u "$IMG" 2>/dev/null || rm -f "$IMG"; }
trap cleanup EXIT
esptool --port "$SRC" read_flash 0 "$BYTES" "$IMG" >/dev/null 2>&1
sz=$(stat -c %s "$IMG")
[ "$sz" = "$BYTES" ] || { echo "REMOTE: short read ($sz of $BYTES)" >&2; exit 1; }
echo "  read ${sz} B, sha256 $(sha256sum "$IMG" | cut -c1-16)..."
echo "  writing to $DST"
esptool --port "$DST" write_flash 0 "$IMG" >/dev/null 2>&1
echo "  verifying"
esptool --port "$DST" verify_flash 0 "$IMG" >/dev/null 2>&1 && echo "  VERIFIED: dest matches source byte for byte"
REMOTE
echo "esp-probe-clone: done (dump shredded on the Pi). Power-cycle the new probe; it should join"
echo "  the same network as the source and answer on TCP 3240."
