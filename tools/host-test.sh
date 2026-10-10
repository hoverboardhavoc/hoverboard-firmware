#!/usr/bin/env bash
# Run the host test suite the way CI runs it. `./tools/host-test.sh [cargo test args...]`
#
# Why this exists: `.cargo/config.toml` sets `[build] target = "thumbv7m-none-eabi"` (deliberate,
# `cargo build -p firmware` must target the board), so a bare `cargo test` BUILDS the test binaries
# for the board and never runs them: no error, no skip message, a green that only means "compiled for
# thumbv7m". CI never saw that because its host-test job passes `--target` explicitly, so the one
# thing silently doing nothing was the check a person runs before committing.
#
# It is a script and not a cargo alias for the same reason the `image` alias in .cargo/config.toml is
# an alias and not an unstable key: the flag belongs to exactly one invocation. An alias additionally
# cannot interpolate anything, and the host triple has to be resolved here rather than hardcoded
# (this repo is driven from an arm64 Mac and from a Pi, and CI's runners are x86_64 Linux).
#
# CI's host-tests job RUNS THIS SCRIPT, so the flags cannot drift from the flags a local pass means.
set -euo pipefail

# `-F store/test-fields` turns on the store crate's reserved test fields plus the `run`/`run_var`
# scenario dispatch, which the store's persist / negative-control tests need. The flag scopes to the
# `store` member; the rest of the workspace is unaffected.
FEATURES="store/test-fields"

refuse_board_target() {
  echo "tools/host-test.sh: refusing '$1'." >&2
  echo "The host suite does not RUN on the board target: cargo would build the test binaries for" >&2
  echo "thumbv7m-none-eabi and execute none of them, which is the silent no-op this script replaces." >&2
  echo "Use 'cargo test --target thumbv7m-none-eabi' directly if compiling them is what you want." >&2
  exit 2
}

# One pass over the forwarded arguments, for two questions.
#
# (1) Was the board target asked for? Then refuse, rather than reproducing the silent no-op this
#     script exists to replace.
# (2) Which packages were selected? `store/test-fields` is a PACKAGE-QUALIFIED feature, so cargo
#     accepts it only when `store` is in the selected set: `host-test.sh -p base` with the flag still
#     attached dies with "the package 'base' does not contain this feature". A selection that leaves
#     `store` out compiles nothing the feature gates, so the flag is dropped for it (announced, not
#     silent). No selection means the whole workspace, which includes `store`, so the flag stays.
selected=""
prev=""
for arg in "$@"; do
  case "$arg" in
    --target=thumbv7m*) refuse_board_target "$arg" ;;
    thumbv7m*)
      if [ "$prev" = "--target" ]; then
        refuse_board_target "--target $arg"
      fi
      ;;
    -p=*|--package=*) selected="$selected ${arg#*=}" ;;
    -p?*) selected="$selected ${arg#-p}" ;;
    *)
      if [ "$prev" = "-p" ] || [ "$prev" = "--package" ]; then
        selected="$selected $arg"
      fi
      ;;
  esac
  prev="$arg"
done

cmd=(cargo test)

HOST_TARGET="$(rustc -vV | sed -n 's/^host: //p')"
if [ -z "$HOST_TARGET" ]; then
  echo "tools/host-test.sh: could not read the host triple from 'rustc -vV'." >&2
  exit 1
fi
cmd+=(--target "$HOST_TARGET")

if [ -n "$selected" ] && [[ " $selected " != *" store "* ]]; then
  echo "tools/host-test.sh: dropping -F $FEATURES: the selected packages (${selected# }) do not include 'store'." >&2
else
  cmd+=(-F "$FEATURES")
fi

set -x
exec "${cmd[@]}" "$@"
