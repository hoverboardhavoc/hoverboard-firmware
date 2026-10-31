#!/usr/bin/env bash
# Run the host test suites the way CI runs them. `./tools/host-test.sh [cargo test args...]`
#
# With no `-p`, it runs two suites, in this order: the motorcontroller workspace's (every crate only
# the GD32 image links, and its test images), then the shared workspace's (the crates the bridge and
# the host tools link as well). Together they are the suite the old single root workspace ran. With
# `-p <package>` (repeatable, globs allowed), it runs the workspace that owns each named package,
# found by the package's folder: motorcontroller/<package>, shared/<package> or tools/<package>.
# Every other argument (cargo flags, a test-name filter, everything after `--`) goes to each run.
#
# Why this exists: motorcontroller/.cargo/config.toml sets `[build] target = "thumbv7m-none-eabi"`
# (deliberate, `cargo build -p firmware` must target the board), so a bare `cargo test` there aims
# the test build at the board. libtest is not built for thumbv7m, so what a person gets is 37 lines
# of `can't find crate for test` and exit 101, which names the wrong problem and no command. CI
# never saw it because its host-test job passes `--target` explicitly, so the check a person runs
# before committing was the one thing not doing what it looked like. Cargo offers no hook before
# compilation, so that wall cannot be prefixed; this script is the route, and CI runs it.
#
# It is a script and not a cargo alias for the same reason the `image` alias in
# motorcontroller/.cargo/config.toml is an alias and not an unstable key: the flag belongs to exactly
# one invocation. An alias additionally cannot interpolate anything, and the host triple has to be
# resolved here rather than hardcoded (this repo is driven from an arm64 Mac and from a Pi, and CI's
# runners are x86_64 Linux). Each run is made from inside its workspace, so that workspace's
# rust-toolchain.toml picks the compiler (the pinned nightly in motorcontroller/, shared/ and
# tools/).
#
# CI's host-tests job RUNS THIS SCRIPT, so the flags cannot drift from the flags a local pass means.
set -euo pipefail

# `-F store/test-fields` turns on the store crate's reserved test fields plus the `run`/`run_var`
# scenario dispatch, which the store's persist / negative-control tests need. The store is a shared/
# crate, so that is the suite the flag is for. The motorcontroller suite passes it too, as the old
# root suite did: cargo applies it to the `store` dependency of the members that have one.
FEATURES="store/test-fields"
WORKSPACES=(motorcontroller shared tools)

refuse_board_target() {
  echo "tools/host-test.sh: refusing '$1'." >&2
  echo "The host suite does not RUN on the board target: libtest is not built for thumbv7m, so the" >&2
  echo "test build fails to compile, and a crate that did compile would be refused by the runner in" >&2
  echo "motorcontroller/.cargo/config.toml rather than handed to the host loader." >&2
  echo "Use 'cargo test --target thumbv7m-none-eabi' in motorcontroller/ directly if compiling them" >&2
  echo "is what you want." >&2
  exit 2
}

# One pass over the forwarded arguments, for two questions.
#
# (1) Was the board target asked for? Then refuse, rather than reproducing the silent no-op this
#     script exists to replace.
# (2) Which packages were selected? They are taken out of the arguments, because each one is handed
#     only to the workspace that owns it: cargo refuses a `-p` that names no member of the workspace
#     it runs in. Everything after `--` belongs to the test binaries and is not read.
selected=()
rest=()
prev=""
after_dashdash=0
for arg in "$@"; do
  if [ "$after_dashdash" = 1 ]; then
    rest+=("$arg")
    continue
  fi
  if [ "$prev" = "-p" ] || [ "$prev" = "--package" ]; then
    selected+=("$arg")
    prev=""
    continue
  fi
  case "$arg" in
    --) after_dashdash=1; rest+=("$arg") ;;
    --target=thumbv7m*) refuse_board_target "$arg" ;;
    -p | --package) ;;
    -p=* | --package=*) selected+=("${arg#*=}") ;;
    -p?*) selected+=("${arg#-p}") ;;
    *)
      if [ "$prev" = "--target" ]; then
        case "$arg" in thumbv7m*) refuse_board_target "--target $arg" ;; esac
      fi
      rest+=("$arg")
      ;;
  esac
  prev="$arg"
done
if [ "$prev" = "-p" ] || [ "$prev" = "--package" ]; then
  echo "tools/host-test.sh: '$prev' needs a package name." >&2
  exit 2
fi

# Run from the repository root whichever directory the caller is in, and then from each workspace in
# turn. With no `-p` cargo selects the package of the current directory, so a run from inside a crate
# would test that crate alone; a package-qualified feature is resolved against the selected packages,
# so it would also die with "the package '<crate>' does not contain this feature". The script's own
# location is the root (it lives in tools/), which needs no git and no env var.
#
# `$0` is resolved through symlinks FIRST, because `dirname` on a link gives the link's directory and
# not the repo: a link to this script from anywhere else died with "could not find Cargo.toml in
# <that directory>", which is the same class of failure as the one above with a worse message.
self="$(readlink -f -- "$0" 2>/dev/null || true)"
[ -n "$self" ] || self="$0"
root="$(cd -- "$(dirname -- "$self")/.." && pwd)"

HOST_TARGET="$(cd -- "$root/motorcontroller" && rustc -vV | sed -n 's/^host: //p')"
if [ -z "$HOST_TARGET" ]; then
  echo "tools/host-test.sh: could not read the host triple from 'rustc -vV'." >&2
  exit 1
fi

# Which workspace owns a selection: the one with a folder of that name (a package's folder is its
# name in every workspace here). A glob is matched against the folder names the way cargo matches it
# against package names, and may select packages in more than one workspace.
owns() {  # owns <workspace> <name-or-glob>
  local ws="$1" pat="$2" d
  for d in "$root/$ws"/*/; do
    d="$(basename -- "$d")"
    # shellcheck disable=SC2053  # an unquoted right-hand side is the glob match
    if [ -f "$root/$ws/$d/Cargo.toml" ] && [[ "$d" == $pat ]]; then
      return 0
    fi
  done
  return 1
}

# The selection per workspace, as a space-separated list in sel_<workspace> (plain variables rather
# than an associative array: macOS ships bash 3.2).
sel_motorcontroller=""
sel_shared=""
sel_tools=""
sel_of() { eval "printf '%s' \"\$sel_$1\""; }
runs=()
if [ "${#selected[@]}" = 0 ]; then
  runs=(motorcontroller shared)
else
  for p in "${selected[@]}"; do
    found=0
    for ws in "${WORKSPACES[@]}"; do
      if owns "$ws" "$p"; then
        found=1
        eval "sel_$ws=\"\$sel_$ws \$p\""
      fi
    done
    if [ "$found" = 0 ]; then
      echo "tools/host-test.sh: no package folder '$p' in motorcontroller/, shared/ or tools/." >&2
      exit 2
    fi
  done
  for ws in "${WORKSPACES[@]}"; do
    [ -n "$(sel_of "$ws")" ] && runs+=("$ws")
  done
fi

# The feature flag, per run. It stays on every run with no `-p`. With a selection:
#   - shared/ keeps it when the selection can include `store`, the one member that declares the
#     feature: cargo accepts `store/test-fields` only when `store` is selected, so
#     `host-test.sh -p base` with the flag still attached dies with "the package 'base' does not
#     contain this feature". A glob that matches `store` keeps it, because dropping it there would
#     run the store's tests WITHOUT its test fields, which is silently fewer tests;
#   - motorcontroller/ and tools/ drop it: neither has `store` as a member, and cargo refuses the
#     flag for a selection whose packages do not depend on it.
# Every drop is announced, not silent.
wants_features() {  # wants_features <workspace>
  local ws="$1" p
  [ "${#selected[@]}" = 0 ] && return 0
  [ "$ws" = shared ] || return 1
  for p in $(sel_of "$ws"); do
    # shellcheck disable=SC2053
    [[ store == $p ]] && return 0
  done
  return 1
}

# The selections are word-split below; a glob in one (`-p 'sto*'`) must reach cargo as written, not
# be expanded against the files in this directory.
set -f
status=0
for ws in "${runs[@]}"; do
  cmd=(cargo test --target "$HOST_TARGET")
  if [ "${#selected[@]}" != 0 ]; then
    for p in $(sel_of "$ws"); do
      cmd+=(-p "$p")
    done
  fi
  if wants_features "$ws"; then
    cmd+=(-F "$FEATURES")
  else
    echo "tools/host-test.sh: $ws/: dropping -F $FEATURES: the selected packages ($(sel_of "$ws" | sed 's/^ //')) do not include 'store'." >&2
  fi
  cmd+=(${rest[@]+"${rest[@]}"})
  echo "+ (cd $ws && ${cmd[*]})" >&2
  if ! (cd -- "$root/$ws" && "${cmd[@]}"); then
    echo "tools/host-test.sh: the $ws/ suite FAILED." >&2
    status=1
  fi
done
exit "$status"
