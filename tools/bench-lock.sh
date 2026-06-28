#!/usr/bin/env bash
# Bench access lock.
#
# The bench (GD32 boards + ST-Link probes) is a single-user physical resource, driven through the Pi
# flash host. Two agents flashing / running OpenOCD / scanning for probes at the same time collide
# (probe USB locations clash, half-programmed flash, garbled SWD). This lock serializes access.
#
# The lock is an atomic mkdir on the Pi (mkdir is atomic on POSIX, so it is a clean mutex). Any agent
# that can ssh to the Pi honors it. Acquire before ANY bench op; release when done.
#
# Usage:
#   tools/bench-lock.sh status                 # who holds it (or FREE)
#   tools/bench-lock.sh acquire <owner> [note] # take it; exit 0 = got it, exit 1 = held by someone else
#   tools/bench-lock.sh refresh <owner>        # heartbeat (keeps a long session from going stale)
#   tools/bench-lock.sh release <owner>        # give it back (only if you hold it)
#   tools/bench-lock.sh steal   <owner>        # force-take (only when sure the holder is gone)
#
# Use a DISTINCT <owner> per agent (e.g. claude-main, claude-2). Env overrides:
#   BENCH_PI (default pi@192.168.0.248), BENCH_LOCKDIR (default /tmp/hoverboard-bench.lock),
#   BENCH_STALE_MIN (default 30).
set -euo pipefail

PI="${BENCH_PI:-pi@192.168.0.248}"
LOCKDIR="${BENCH_LOCKDIR:-/tmp/hoverboard-bench.lock}"
STALE_MIN="${BENCH_STALE_MIN:-30}"

cmd="${1:-status}"
owner_raw="${2:-${USER:-agent}@$(hostname -s 2>/dev/null || echo host)}"
owner="${owner_raw// /_}"           # owner must be a single token
note="${3:-}"
if [ -z "$note" ]; then noteB64="-"; else noteB64="$(printf '%s' "$note" | base64 | tr -d '\n')"; fi

ssh -o BatchMode=yes -o ConnectTimeout=8 "$PI" bash -s -- "$cmd" "$owner" "$noteB64" "$LOCKDIR" "$STALE_MIN" <<'REMOTE'
set -u
cmd="$1"; owner="$2"; noteB64="$3"; lockdir="$4"; stale="$5"
info="$lockdir/info"
if [ "$noteB64" = "-" ]; then note=""; else note="$(printf '%s' "$noteB64" | base64 -d 2>/dev/null || true)"; fi
now="$(date -u +%s)"
iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

ho=""; hiso=""; he=0; hn=""
read_info() {
  ho=""; hiso=""; he=0; hn=""
  if [ -f "$info" ]; then
    ho="$(sed -n 1p "$info" 2>/dev/null || true)"
    hiso="$(sed -n 2p "$info" 2>/dev/null || true)"
    he="$(sed -n 3p "$info" 2>/dev/null || true)"
    hn="$(sed -n 4p "$info" 2>/dev/null || true)"
  fi
  case "$he" in ''|*[!0-9]*) he=0 ;; esac
}
write_info() { printf '%s\n%s\n%s\n%s\n' "$owner" "$iso" "$now" "$note" > "$info"; }

case "$cmd" in
  acquire)
    if mkdir "$lockdir" 2>/dev/null; then
      write_info; echo "ACQUIRED: $owner ($iso)${note:+  note: $note}"; exit 0
    fi
    read_info
    if [ "$ho" = "$owner" ]; then write_info; echo "ALREADY-OURS (refreshed): $owner"; exit 0; fi
    age=$(( now - he ))
    msg="HELD by $ho since $hiso (${age}s ago)"; [ -n "$hn" ] && msg="$msg  note: $hn"
    echo "$msg"
    if [ "$age" -ge $(( stale * 60 )) ]; then
      echo "NOTE: lock is STALE (> ${stale}m). If you are sure the holder is gone: bench-lock.sh steal \"$owner\""
    fi
    exit 1
    ;;
  refresh)
    if [ ! -d "$lockdir" ]; then echo "NOT-LOCKED (nothing to refresh)"; exit 1; fi
    read_info
    if [ "$ho" != "$owner" ]; then echo "REFUSED: held by $ho, not $owner"; exit 1; fi
    write_info; echo "REFRESHED: $owner ($iso)"; exit 0
    ;;
  release)
    if [ ! -d "$lockdir" ]; then echo "NOT-LOCKED"; exit 0; fi
    read_info
    if [ "$ho" = "$owner" ]; then rm -rf "$lockdir"; echo "RELEASED (was: $ho)"; exit 0; fi
    echo "REFUSED: held by $ho, not $owner. Use steal to force."; exit 1
    ;;
  steal)
    read_info
    rm -rf "$lockdir"; mkdir -p "$lockdir"; write_info
    echo "STOLEN: $owner took the lock (was: ${ho:-none})"; exit 0
    ;;
  status)
    if [ ! -d "$lockdir" ]; then echo "FREE"; exit 0; fi
    read_info; age=$(( now - he ))
    echo "HELD by $ho since $hiso (${age}s ago)${hn:+  note: $hn}"
    [ "$age" -ge $(( stale * 60 )) ] && echo "(STALE > ${stale}m)"
    exit 0
    ;;
  *)
    echo "usage: bench-lock.sh {status|acquire|refresh|release|steal} [owner] [note]"; exit 2
    ;;
esac
REMOTE
