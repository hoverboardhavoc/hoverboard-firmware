# esp-tool

The ESP32-C3 board tool firmware (`specs/esp-tool.md` in the specs repo). This file covers building
it, installing it the first time, and updating it afterwards.

## Building

The Wi-Fi network is a build-time setting: the build reads `ESP_WIFI_SSID` and `ESP_WIFI_PASS`
from its environment and stops with an error naming both when either is unset. The bench's
credentials are in `.wifi.env` here (gitignored, never committed); source it into the environment
of the build without printing it:

```
cd apps/esp-tool
set -a; . ./.wifi.env; set +a
cargo build --release
```

A build for anything but a board (clippy, a size check) can use any values, e.g.
`ESP_WIFI_SSID=x ESP_WIFI_PASS=x cargo clippy --release -- -D warnings`. The host tests of the
hardware-free half need neither: `cargo test -p esp-tool-core --target aarch64-apple-darwin` (or
the host's own triple).

## Flash layout

`partitions.csv`, on the SuperMini's 4 MB:

| Partition | Offset | Size | Holds |
| --- | --- | --- | --- |
| (bootloader) | `0x0` | | ESP-IDF's second-stage bootloader, built with rollback (`bootloader/`) |
| (partition table) | `0x8000` | | this table |
| `keys` | `0x9000` | 16 KiB | the ESP's own settings, kept across updates (nothing is stored in it yet) |
| `otadata` | `0xd000` | 8 KiB | which app slot boots, and its rollback state |
| `ota_0` | `0x10000` | 1,984 KiB | app slot |
| `ota_1` | `0x200000` | 1,984 KiB | app slot |

## First install

Over the C3's own USB port with `espflash`: one merged image holding the bootloader, the partition
table and the app in `ota_0`, with `otadata` blank (the bootloader then runs `ota_0`).

```
espflash save-image --chip esp32c3 --merge --flash-size 4mb \
  --bootloader bootloader/bootloader.bin --partition-table partitions.csv \
  --target-app-partition ota_0 \
  target/riscv32imc-unknown-none-elf/release/esp-tool target/img/esp-tool-install.bin
espflash write-bin 0x0 target/img/esp-tool-install.bin
```

The merged image is padded to the whole 4 MB, so it also erases `keys` and `otadata`.

The bootloader must be `bootloader/bootloader.bin`, not `espflash`'s bundled one: that one is built
without `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, so it never marks a new slot pending and never
falls back from one that was not confirmed. `bootloader/` is the ESP-IDF project that builds it
(ESP-IDF v5.4.2; `sdkconfig.defaults` turns rollback on):

```
cd bootloader
. ~/esp/esp-idf/export.sh
idf.py bootloader
cp build/bootloader/bootloader.bin bootloader.bin
```

## Later updates

The app image alone, as the slot holds it:

```
espflash save-image --chip esp32c3 \
  target/riscv32imc-unknown-none-elf/release/esp-tool target/img/esp-tool.bin
hoverboard-cli --tcp <name>.local:5333 update-esp target/img/esp-tool.bin
```

`update-esp` writes the slot not running, the ESP resets into it, and the tool reconnects and sends
`ESP_UPDATE_CONFIRM`. With `--no-confirm` it stops after the reconnect; the next reset then returns
to the previous slot.

`cargo run --release` (with the credentials sourced) flashes the same layout as the first install
from the ELF, leaving `keys` alone and erasing `otadata` so the bootloader runs `ota_0`.

## A silent boot

`--features boot-log` prints one line on the USB console at each boot step: the scheduler and free
heap, the SWD clock, the running slot and whether it is pending, the console ready, the BLE
controller and the Wi-Fi driver, Wi-Fi start, each join and its result, the address, mDNS, the
TCP listener and each client, and BLE advertising, with esp-radio's own info lines among them.
The text shares the port with the L2 stream (the host's stream decoder skips it), so it is for
reading a boot on the bench, not for an image in use:

```
set -a; . ./.wifi.env; set +a
cargo build --release --features boot-log
```

## Stack

The main stack is what DRAM has left after `.data`, `.bss` and the heap (about 46 KB). A task's
arguments pass through the stack of the task that spawns it, so the service's state (the router,
the mailbox port, the command service with the update's sector buffer) is placed in statics and
handed over by reference; moved by value it overran the stack and the ESP booted to silence. Check
a change that grows a task's state with `riscv32-esp-elf-objdump -d` for frames of many kilobytes.
