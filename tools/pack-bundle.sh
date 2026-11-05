#!/usr/bin/env bash
# Build the firmware bundle (specs/firmware-bundle.md) the way CI does.
#
#   ./tools/pack-bundle.sh [--images DIR] [--cli PATH] [--dirty]
#
# With no options it builds every image the bundle holds, each in its own workspace so that
# workspace's rust-toolchain.toml picks the compiler: the three board images in motorcontroller/
# (`cargo image-f103`, `cargo image-f130`, `cargo image-f103rc`), the bridge in bridge/ with its two
# images made by espflash, and hoverboard-cli in tools/. It stages them in target/images/ by image
# name and packs target/bundle/firmware-<commit>.hbfw.
#
# --images DIR  skip the builds and pack the images already in DIR, named as target/images/ names
#               them. CI's pack job uses this with the images its build and bridge jobs made.
# --cli PATH    use this hoverboard-cli instead of building one.
# --dirty       pack a tree with uncommitted changes to tracked files; the bundle's commit then
#               ends in -dirty. Without it the packer refuses such a tree.
#
# Needs: the toolchains the workspaces pin (rustup installs them on first use) and espflash 4.6.0,
# the version CI pins, for the bridge images.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CHIPS=(gd32f103c8 gd32f130c8 gd32f103rc)
IMAGES=""
CLI=""
DIRTY=()

while [ $# -gt 0 ]; do
    case "$1" in
        --images) IMAGES="$2"; shift 2 ;;
        --cli) CLI="$2"; shift 2 ;;
        --dirty) DIRTY=(--dirty); shift ;;
        -h|--help) sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "pack-bundle: unknown option $1 (see --help)" >&2; exit 2 ;;
    esac
done

# A toolchain set by a caller (cargo run, an IDE) would override every workspace's own file.
unset RUSTUP_TOOLCHAIN

if [ -z "$IMAGES" ]; then
    IMAGES="$ROOT/target/images"
    mkdir -p "$IMAGES"

    echo "== board images (motorcontroller/)"
    (cd "$ROOT/motorcontroller" && cargo image-f103 && cargo image-f130 && cargo image-f103rc)
    for chip in "${CHIPS[@]}"; do
        cp "$ROOT/motorcontroller/target/$chip/thumbv7m-none-eabi/release/firmware" \
            "$IMAGES/mainboard-$chip.elf"
    done

    echo "== bridge images (bridge/)"
    command -v espflash >/dev/null || { echo "pack-bundle: espflash not found (cargo install espflash@4.6.0)" >&2; exit 1; }
    (
        cd "$ROOT/bridge"
        env -u ESP_WIFI_SSID -u ESP_WIFI_PASS cargo build --release
        elf=target/riscv32imc-unknown-none-elf/release/hoverboard-bridge
        espflash save-image --chip esp32c3 --merge --flash-size 4mb \
            --bootloader bootloader/bootloader.bin --partition-table partitions.csv \
            --target-app-partition ota_0 "$elf" "$IMAGES/bridge-esp32c3-install.bin"
        espflash save-image --chip esp32c3 "$elf" "$IMAGES/bridge-esp32c3.bin"
    )
fi

if [ -z "$CLI" ]; then
    echo "== hoverboard-cli (tools/)"
    (cd "$ROOT/tools" && cargo build --release -p hoverboard-cli)
    CLI="$ROOT/tools/target/release/hoverboard-cli"
fi

echo "== pack"
cd "$ROOT"
mkdir -p target/bundle
commit="$(git rev-parse --short=7 HEAD)"
git diff --quiet HEAD || commit="$commit-dirty"
bundle="target/bundle/firmware-$commit.hbfw"
args=()
for chip in "${CHIPS[@]}"; do
    args+=("motorcontroller/firmware:mainboard-$chip=$IMAGES/mainboard-$chip.elf")
done
args+=("bridge:bridge-esp32c3=$IMAGES/bridge-esp32c3.bin")
args+=("bridge:bridge-esp32c3-install=$IMAGES/bridge-esp32c3-install.bin")
# The packer prints what it wrote, as `firmware info` would.
"$CLI" firmware pack "$bundle" "${args[@]}" ${DIRTY[@]+"${DIRTY[@]}"}
