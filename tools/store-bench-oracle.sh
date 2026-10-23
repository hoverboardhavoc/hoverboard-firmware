#!/usr/bin/env bash
# Tier-3 store oracle runner. Runs the store-test scenario matrix on a real GD32 over SWD and
# host-judges each scenario (the device publishes a TestResult; this script knows the expected value).
#
# Transport-agnostic: a preset selects the probe wiring, so the SAME matrix runs wherever a board is.
# The `12fet` / `master` / `slave` presets run ON THE PI (its OpenOCD and its USB probes); the
# `offroad-master` / `offroad-slave` presets run on whatever host has the patched elaphureLink
# OpenOCD and can reach the probes over the LAN, which is the Mac. Reset is always SYSRESETREQ (no
# bench probe drives NRST).
#
# Usage:
#   store-bench-oracle.sh <transport> <image.elf> <store_base_hex> <page_size> <region_dir>
# e.g.
#   store-bench-oracle.sh 12fet          /tmp/store-test-chip2k.elf 0x0803F000 2048 /tmp/store-regions/2k
#   store-bench-oracle.sh master         /tmp/store-test-chip1k.elf 0x0800F800 1024 /tmp/store-regions/1k
#   store-bench-oracle.sh offroad-slave  target/thumbv7m-none-eabi/release/store-test 0x0800F800 1024 bench-work/regions/1k
#
# The offroad presets resolve the probe's current address from its MAC through
# target/bench/probe-addr.sh (DHCP drifts; the MAC does not). They need the probe to themselves, so
# stop any long-running OpenOCD holding it first: an elaphureLink probe takes one TCP client.
#
# The crafted region .bin files (compact|torn_payload|torn_header|full) must already be in <region_dir>
# (generate them on the host: cargo run --example craft_region --features test-fields -- <s> <ps> <out>).

set -u
TRANSPORT="$1"; IMAGE="$2"; STORE_BASE="$3"; PAGE_SIZE="$4"; REGION_DIR="$5"
REGION_LEN=$(( 2 * PAGE_SIZE ))
CMD_ADDR=0x20001FF0
RES=0x20001F00          # ready@+0, output@+4, len@+8 (u16), buf@+10

# Expected values the host knows (non-circular: the device never judges itself).
READY=5a1e5a1e
TVAL=00c0ffee
STR_HEX="68 6f 76 65 72 62 6f 61 72 64 2d 78 31"   # "hoverboard-x1"
BLOB_HEX="de ad be ef 01 02 03"

SYS=/usr/bin/openocd
ELA=/home/hoverboardhavoc/dev/openocd-elaphurelink/openocd/src/openocd
ELASCR=/home/hoverboardhavoc/dev/openocd-elaphurelink/openocd/tcl
# The same patched OpenOCD on the Mac, where the offroad presets run.
MAC_ELA="$HOME/dev/openocd-elaphurelink/openocd/src/openocd"
MAC_ELASCR="$HOME/dev/openocd-elaphurelink/openocd/tcl"
HERE="$(cd "$(dirname "$0")" && pwd)"

case "$TRANSPORT" in
  12fet)  OCD="$ELA";  PRE=(-s "$ELASCR" -f interface/elaphurelink.cfg -c 'cmsis-dap elaphurelink addr 192.168.0.245' -c 'transport select swd' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg) ;;
  master) OCD="$SYS";  PRE=(-f interface/stlink.cfg -c 'transport select dapdirect_swd' -c 'adapter usb location 3-1.1' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg) ;;
  slave)  OCD="$SYS";  PRE=(-c 'adapter driver cmsis-dap' -c 'cmsis-dap backend usb_bulk' -c 'cmsis-dap vid_pid 0x1209 0xda42' -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg) ;;
  # The classywalk offroad pair, on LAN-attached ESP32-C3 elaphureLink probes, driven from the Mac.
  offroad-master|offroad-slave)
    # probe-addr.sh is a local bench helper, not a committed tool: it lives in the gitignored
    # target/bench/ of whichever checkout the owner set it up in, so look in this one and then in
    # the main checkout beside it before giving up.
    ADDRSH=""
    for c in "$HERE/../target/bench/probe-addr.sh" "$HOME/dev/hoverboard-firmware/target/bench/probe-addr.sh"; do
      [ -x "$c" ] && ADDRSH="$c" && break
    done
    if [ -z "$ADDRSH" ]; then
      echo "store-bench-oracle: probe-addr.sh not found; the offroad presets resolve their probe by MAC through it." >&2
      exit 2
    fi
    PROBE_IP="$("$ADDRSH" "$TRANSPORT")" || exit 1
    echo "=== $TRANSPORT probe at $PROBE_IP (resolved by MAC) ==="
    OCD="$MAC_ELA"
    PRE=(-s "$MAC_ELASCR" -f interface/elaphurelink.cfg -c "cmsis-dap elaphurelink addr $PROBE_IP" -c 'transport select swd' -c 'adapter speed 1000' -c 'set CPUTAPID 0' -f target/stm32f1x.cfg) ;;
  *) echo "unknown transport $TRANSPORT"; exit 2 ;;
esac
RC=(-c 'cortex_m reset_config sysresetreq')

# ocd <extra -c args...>: run one openocd session (init -> ... -> shutdown), echo its stdout+stderr.
ocd() { "$OCD" "${PRE[@]}" "${RC[@]}" -c init "$@" -c shutdown 2>&1; }

# ============================ THE STANDING ARMED-HALT RULE ===================================
# Every phase below starts with `reset halt`, so this is a halt-capable tool. No tool in tools/ may
# halt while MOE reads set unless it just performed the disarm write itself (only
# tools/swd-disarm-halt.sh does). Halting stops software, not the timer's output stage: the last
# duties stay live with nothing stepping them, which is the recorded FET failure.
#
# TIMER0 CCHP = 0x4001_2C44 (base + offset 0x44, identical on F103 and F130), MOE = bit 15. A board
# with no motor brought up reads 0 here, so the store matrix's usual targets pass for free. The
# VERDICT on the read is not decided here: tools/armed-guard-verdict.sh owns it for every
# halt-capable tool (an inconclusive read fails closed there, FORCE_ARMED_GUARD=1 to override).
armed_check() {
  local cchp
  cchp=$(ocd -c 'mdw 0x40012C44 1' | sed -n 's/^0x40012c44: *\([0-9a-f]*\).*/\1/p' | head -1)
  "$(dirname "$0")/armed-guard-verdict.sh" store-bench-oracle "$cchp" || exit 1
}
armed_check

PASS=0; FAIL=0
ok()   { echo "PASS  $1"; PASS=$((PASS+1)); }
bad()  { echo "FAIL  $1  ($2)"; FAIL=$((FAIL+1)); }

# Flash the image once and confirm the part.
echo "=== flashing $IMAGE on $TRANSPORT ==="
ocd -c 'reset halt' -c "program $IMAGE verify" -c 'echo FLASHSIZE:' -c 'mdw 0x1FFFF7E0 1' | grep -iE 'Verified|FLASHSIZE|0x1ffff7e0|error|fail' || true

# scalar_phases <label> <expected_output> <plant_bin_or_-> <cmd ...>
# Runs: reset halt, [erase region], [plant], then each <cmd> as a (mww CMD; reset run; settle) phase,
# reads RES scalar, judges output against <expected_output> ('NOT:xxxx' means must differ).
scalar_run() {
  local label="$1" expect="$2" plant="$3"; shift 3
  local args=(-c 'reset halt' -c "flash erase_address $STORE_BASE $REGION_LEN")
  [ "$plant" != "-" ] && args+=(-c "flash write_image $REGION_DIR/$plant $STORE_BASE bin")
  local c
  for c in "$@"; do args+=(-c "mww $CMD_ADDR $c" -c 'reset run' -c 'sleep 400' -c 'halt'); done
  args+=(-c 'echo SCALAR:' -c "mdw $RES 2" -c 'reset run')
  local out line ready output
  out=$(ocd "${args[@]}")
  line=$(echo "$out" | grep -A1 'SCALAR:' | tail -1)
  ready=$(echo "$line" | awk '{print $2}'); output=$(echo "$line" | awk '{print $3}')
  if [ "$ready" != "$READY" ]; then bad "$label" "not ready: $line"; return; fi
  case "$expect" in
    NOT:*) if [ "$output" != "${expect#NOT:}" ]; then ok "$label (output=$output)"; else bad "$label" "output==$output, expected differ"; fi ;;
    *)     if [ "$output" = "$expect" ]; then ok "$label (output=$output)"; else bad "$label" "output=$output expected=$expect"; fi ;;
  esac
}

# var_run reads a variable value (buf/len) after the given read cmd, judging the first len bytes.
var_run() {
  local label="$1" expect_hex="$2" erase="$3"; shift 3   # remaining: the cmd sequence
  local args=(-c 'reset halt')
  [ "$erase" = "erase" ] && args+=(-c "flash erase_address $STORE_BASE $REGION_LEN")
  local c
  for c in "$@"; do args+=(-c "mww $CMD_ADDR $c" -c 'reset run' -c 'sleep 400' -c 'halt'); done
  local nbytes; nbytes=$(echo "$expect_hex" | wc -w)
  args+=(-c 'echo LEN:' -c "mdh $(printf '0x%X' $((RES+8))) 1" -c 'echo BUF:' -c "mdb $(printf '0x%X' $((RES+10))) $nbytes" -c 'reset run')
  local out lenline bufline gotbuf
  out=$(ocd "${args[@]}")
  lenline=$(echo "$out" | grep -A1 'LEN:' | tail -1)
  bufline=$(echo "$out" | grep -A1 'BUF:' | tail -1)
  gotbuf=$(echo "$bufline" | cut -d: -f2 | xargs | tr 'A-F' 'a-f')
  local want; want=$(echo "$expect_hex" | xargs)
  if [ "$gotbuf" = "$want" ]; then ok "$label (buf=$gotbuf)"; else bad "$label" "buf='$gotbuf' want='$want' len='$lenline'"; fi
}

echo "=== scenarios ($TRANSPORT, store_base $STORE_BASE, page $PAGE_SIZE) ==="
# Device-written
scalar_run "persist"      "$TVAL"      -                 0x00000000 0x00000001
scalar_run "neg-control"  "NOT:$TVAL"  -                 0x00000001
var_run    "var-str"      "$STR_HEX"   erase             0x00010000 0x00010001
var_run    "var-blob"     "$BLOB_HEX"  noerase           0x00010002   # reuses the var-str planted state
# Host-planted
scalar_run "compact"      "$TVAL"      compact.bin       0x00020001
scalar_run "torn-payload" "$TVAL"      torn_payload.bin  0x00030001
scalar_run "torn-header"  "$TVAL"      torn_header.bin   0x00040001
scalar_run "full"         "$TVAL"      full.bin          0x00050000 0x00050001

echo "=== $TRANSPORT: $PASS passed, $FAIL failed ==="
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
