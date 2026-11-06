# Handheld remote

A Ride screen on the GUITION ESP32-2432S028R "Cheap Yellow Display", as a BLE central to one
hoverboard bridge (`specs/handheld-remote.md`, in the private specs repository).

## Building

The workspace builds on the esp-rs Xtensa toolchain: `espup install --targets esp32
--toolchain-version 1.92.0.0`, then `. ~/export-esp.sh`. The core's host tests are
`cargo test -p handheld-core --target <host triple>`.

The image pins its bridge at build time by BLE address, most significant octet first, and is
written with esptool:

```
HANDHELD_BRIDGE=dc:06:75:f8:52:6c cargo image
espflash save-image --chip esp32 --merge --flash-size 4mb \
  target/xtensa-esp32-none-elf/release/handheld target/img/handheld-install.bin
esptool --chip esp32 --port <port> --baud 460800 write-flash 0x0 target/img/handheld-install.bin
```

The `debug-line` feature puts the touch state and link rates on the statement line; `test-pattern`
boots to colour bars for checking the panel's colour order and orientation.
