#!/usr/bin/env bash
# The thumbv7m `runner` in motorcontroller/.cargo/config.toml: cargo must refuse a board binary by
# name rather than hand it to the host loader. Checked here rather than through a real `cargo run`, which would build
# the whole firmware to assert one message.
set -uo pipefail
R="$(cd "$(dirname "$0")/.." && pwd)/board-target-runner.sh"
P=0; F=0
chk() {  # chk <label> <condition-description> <actual> <expected>
  if [ "$3" = "$4" ]; then printf 'PASS  %-52s %s\n' "$1" "$3"; P=$((P+1))
  else printf 'FAIL  %-52s %s (want %s)\n' "$1" "$3" "$4"; F=$((F+1)); fi
}

out=$("$R" motorcontroller/target/thumbv7m-none-eabi/release/firmware 2>&1); rc=$?
chk "refuses a board binary"            rc "$rc" 2
chk "names the binary"                  grep "$(printf '%s' "$out" | grep -c 'firmware')" 1
chk "names the host-test route"         grep "$(printf '%s' "$out" | grep -c 'tools/host-test.sh')" 1
chk "names the image build"             grep "$(printf '%s' "$out" | grep -c 'cargo image')" 1
chk "names the flash route"             grep "$(printf '%s' "$out" | grep -c 'tools/flash.sh')" 1

# A refusal that said nothing about the target would leave the caller where the OS exec error did.
chk "names the target"                  grep "$(printf '%s' "$out" | grep -c 'thumbv7m-none-eabi')" 1

# The config must actually point at it, or the script is a file nothing calls. Cargo resolves a
# relative runner path against the directory that holds .cargo/, here motorcontroller/.
mc="$(cd "$(dirname "$0")/../.." && pwd)/motorcontroller"
runner=$(sed -n 's/^runner = "\(.*\)"$/\1/p' "$mc/.cargo/config.toml")
chk "config.toml wires it up"           resolves "$(cd "$mc" && [ "$(cd "$(dirname "$runner")" && pwd)/$(basename "$runner")" = "$R" ] && echo yes)" yes

echo "-- $P passed, $F failed"
[ "$F" = 0 ]
