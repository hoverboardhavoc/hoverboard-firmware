#!/usr/bin/env bash
# The `runner` for thumbv7m-none-eabi (motorcontroller/.cargo/config.toml). Cargo calls it with the
# binary it was about to execute, for `cargo run` and for `cargo test`'s test binaries.
#
# There is no board on the other end of a cargo invocation here. Boards are programmed by
# tools/flash.sh, which takes the bench lock, runs the image guards and drives OpenOCD over a probe.
# Without this runner cargo hands an ARM ELF to the host loader, which refuses it with an OS error
# that names neither the target nor the command the caller wanted.
set -euo pipefail

bin="${1:-(no binary)}"
echo "cargo: refusing to run $(basename "$bin") on the host." >&2
echo "It is built for thumbv7m-none-eabi, the default target in motorcontroller/.cargo/config.toml," >&2
echo "and this machine cannot execute it. From the repository root:" >&2
echo >&2
echo "  host tests      ./tools/host-test.sh               (forwards cargo args: -p base, a test name)" >&2
echo "  shipping image  cd motorcontroller && cargo image" >&2
echo "  onto a board    tools/flash.sh <image>             (bench lock + image guards + OpenOCD)" >&2
exit 2
