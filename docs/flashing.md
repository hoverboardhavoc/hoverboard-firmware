# Flashing

The firmware is written to the mainboard MCU over **SWD** (SWDIO / SWCLK / GND, with the board's
3V3 as the reference). Two probe options:

## Wireless, via an ESP32-C3 debug probe

An ESP32-C3 running [wireless-esp32-dap](https://github.com/hoverboardhavoc/wireless-esp32-dap)
acts as a CMSIS-DAP probe reachable over WiFi. The host drives it with OpenOCD through
[openocd-elaphurelink](https://github.com/hoverboardhavoc/openocd-elaphurelink) (the elaphureLink
bridge). The probe sits on the board's SWD header; no USB cable runs to the board itself.

Flashing the C3 with this probe firmware is a one-time setup, separate from the hoverboard firmware.

## Wired, via an ST-Link V2 clone

A standard ST-Link V2 clone on the SWD header, driven by stock OpenOCD over USB.

## GD32 SWD specifics (target config and connect)

The GD32F130/F103 is STM32F1-pin-compatible for debug, so OpenOCD's `target/stm32f1x.cfg` works, with
one required tweak: **`set CPUTAPID 0`** skips the IDCODE check so the GD32's different DP IDCODE is
accepted instead of being rejected as "wrong device" (the GD32 reports DPIDR `0x1ba01477`, DBGMCU IDCODE
`0x13030410`, device `0x410`, designer GigaDevice). Common ground is mandatory; NRST is optional (OpenOCD
resets the core in software over SWD, so `SWCLK/SWDIO/3V3/GND` is enough to connect, flash, and verify).

Discover a wireless ESP32-C3 probe by its open port (DHCP, mDNS off in the probe firmware): the host with
TCP **3240** open is the probe.

```
masscan 192.168.0.0/24 -p3240        # adjust to your subnet
```

Flash + verify + reset, pointing at the ELF (or a raw `.bin` with a load address `0x08000000`):

```
openocd -c "adapter driver cmsis-dap" -c "cmsis-dap backend elaphurelink" \
  -c "cmsis-dap elaphurelink addr <probe-ip>" \
  -c "transport select swd" -c "adapter speed 1000" -c "set CPUTAPID 0" \
  -f target/stm32f1x.cfg \
  -c "program <image>.elf verify reset exit"
```

Lower `adapter speed` (500 or 100) if flashing is flaky over Wi-Fi or with long jumper wires. A
target-side `Error connecting DP: cannot read IDR` means the probe is fine but the GD32 is not answering
(check all four SWD wires + common ground, confirm 3V3, lower speed).

## macOS Local Network Privacy (the wireless-path gotcha)

On macOS 15 (Sequoia), Local Network Privacy blocks a process from connecting to LAN addresses unless the
**responsible app** (the GUI app macOS walks up to from the process tree) is granted access. A blocked
`connect()` returns `EHOSTUNREACH` ("No route to host") even though `ping`, `nc`, and `python` connect
fine from the same shell, and public IPs connect, only the LAN is blocked. The trap: a **tmux server**
started by launchd becomes its own (ungranted) responsible app and is silently denied with no prompt, so
the wireless flash works from a plain terminal window but fails inside tmux. The permanent fix is to make
the tmux server always be born from a granted terminal (Ghostty/Terminal), never from launchd. The full
write-up of this saga (symptoms, the tmux trap, the `.zshrc` switchover, and the dead-end "fixes") lives
in the bench flashing notes for these probes. On Linux (including a Raspberry Pi) none of this applies.
