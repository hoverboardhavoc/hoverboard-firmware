# hoverboard-bridge

The hoverboard bridge firmware (`specs/hoverboard-bridge.md` in the specs repo). This file covers building
it, installing it the first time, and updating it afterwards.

## Building

One image for every network: the image holds no Wi-Fi credentials. The bridge reads the network
from its store, `wifi.ssid` and `wifi.pass` (`specs/bridge-onboarding.md` section 2), once at boot,
and with none stored it does not start its Wi-Fi station (`info` shows `wifi not configured`).

```
cd bridge
cargo build --release
```

The host tests of the hardware-free half: `cargo test -p hoverboard-bridge-core --target
aarch64-apple-darwin` (or the host's own triple), and its lint `cargo clippy -p
hoverboard-bridge-core --target <host> --all-targets -- -D warnings`. CI's `bridge` job runs both,
then this release build and the two images below.

## The RAM loader

A board flash programs through a small loader the bridge places in the board's SRAM
(`core/src/loader.rs`, `specs/hoverboard-bridge.md`, "The RAM loader"). Its source is `loader/f1x.S`
and its bytes `loader/f1x.bin`, both checked in; the bridge embeds the bytes. After a change to the
source, assemble it again from this directory:

```
mkdir -p target/loader
arm-none-eabi-as -mcpu=cortex-m3 -mthumb -o target/loader/f1x.o loader/f1x.S
arm-none-eabi-objcopy -O binary target/loader/f1x.o loader/f1x.bin
```

A host test assembles the source the same way and fails if the result differs from `f1x.bin` (it
is skipped when `arm-none-eabi-as` is not on the path). The loader bytes also run under Unicorn,
driven by the bridge's own driver: `cargo test -p emulator-runner --features unicorn --target <host>
--test bridge_loader` in `tools/`.

## Flash layout

`partitions.csv`, on the SuperMini's 4 MB:

| Partition | Offset | Size | Holds |
| --- | --- | --- | --- |
| (bootloader) | `0x0` | | ESP-IDF's second-stage bootloader, built with rollback (`bootloader/`) |
| (partition table) | `0x8000` | | this table |
| `keys` | `0x9000` | 16 KiB | the bridge's own keys (`node.address`, `device.name`, `wifi.ssid`, `wifi.pass`), kept across updates: the `store` crate's two pages in its first two sectors |
| `otadata` | `0xd000` | 8 KiB | which app slot boots, and its rollback state |
| `ota_0` | `0x10000` | 1,984 KiB | app slot |
| `ota_1` | `0x200000` | 1,984 KiB | app slot |

## First install

Over the C3's own USB port with `hoverboard-cli install-esp` (`specs/bridge-onboarding.md` section
3), from one merged image holding the bootloader, the partition table and the app in `ota_0`, with
`otadata` blank (the bootloader then runs `ota_0`). CI builds it as `hoverboard-bridge-install.bin`;
locally:

```
espflash save-image --chip esp32c3 --merge --flash-size 4mb \
  --bootloader bootloader/bootloader.bin --partition-table partitions.csv \
  --target-app-partition ota_0 \
  target/riscv32imc-unknown-none-elf/release/hoverboard-bridge target/img/hoverboard-bridge-install.bin
hoverboard-cli install-esp target/img/hoverboard-bridge-install.bin bridge.json
```

`bridge.json` names the new bridge and its network, in the file format of `cli-config.md`:

```
{ "device": { "name": "offroad-slave" }, "wifi": { "ssid": "<network>", "pass": "<password>" } }
```

`install-esp` finds the port by the C3's USB ids (`303a:1001`), or takes `--serial PORT`. If a
bridge answers there it names it and asks before erasing it (`--yes` skips the question). It writes
the image, waits for the first boot, applies the file, resets the bridge and waits for it to join.
The password is never printed and never read back (`config` shows `(set)`).

The merged image is padded to the whole 4 MB, so it also erases `keys` and `otadata`: the bridge
boots unassigned with its default name and no Wi-Fi, and the next walk addresses it again.

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
  target/riscv32imc-unknown-none-elf/release/hoverboard-bridge target/img/hoverboard-bridge.bin
hoverboard-cli --tcp <name>.local:5333 update-esp target/img/hoverboard-bridge.bin
```

`update-esp` writes the slot not running, the bridge resets into it, and the tool reconnects and sends
`ESP_UPDATE_CONFIRM`. With `--no-confirm` it stops after the reconnect; the next reset then returns
to the previous slot.

`cargo run --release` flashes the same layout as the first install from the ELF, leaving `keys`
alone and erasing `otadata` so the bootloader runs `ota_0`.

## A silent boot

`--features boot-log` prints one line on the USB console at each boot step: the scheduler and free
heap, the SWD clock, the running slot and whether it is pending, the console ready, the BLE
controller and the Wi-Fi driver, Wi-Fi start, each join and its result, the address, mDNS, the
TCP listener and each client, and BLE advertising, with esp-radio's own info lines among them.
The text shares the port with the L2 stream (the host's stream decoder skips it), so it is for
reading a boot on the bench, not for an image in use:

```
cargo build --release --features boot-log
```

## Stack

The main stack is what DRAM has left after `.data`, `.bss` and the heap (about 46 KB). A task's
arguments pass through the stack of the task that spawns it, so the service's state (the router,
the mailbox port, the command service with the update's sector buffer) is placed in statics and
handed over by reference; moved by value it overran the stack and the bridge booted to silence. Check
a change that grows a task's state with `riscv32-esp-elf-objdump -d` for frames of many kilobytes.
