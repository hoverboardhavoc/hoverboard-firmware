# Getting started

This guide takes a hoverboard motor controller from stock to driving on this firmware.

> ⚠️ This firmware switches the motor controller's FETs directly. A bug in commutation, dead-time or the
> fault and arming logic can cause [shoot-through](https://en.wikipedia.org/wiki/H-bridge#General): both
> FETs of a phase leg turn on at once and short the battery. Bring a new board up on a current-limited
> bench supply before it sees a battery. Use it at your own risk: the firmware comes with no warranty.

## What you need

- A supported motor controller: the single board that drives both wheels (GD32F103RC), or a pair
  of split boards (GD32F103C8 or GD32F130C8).
- One ESP32-C3 SuperMini per controller board. It becomes the board's bridge.
- A 5 V supply for each bridge (see [Power the bridge](#power-the-bridge)).
- A computer for `hoverboard-cli`, and a USB cable for the first bridge install.
- An Android phone, for driving from the app.

## 1. Get the CLI and the firmware

- **The firmware bundle:**
  [download the latest `main` bundle](https://nightly.link/hoverboardhavoc/hoverboard-firmware/workflows/ci/main/firmware-bundle.zip).
  It holds every board image and the bridge's images.
- **`hoverboard-cli`:** each CI run in
  [Actions](https://github.com/hoverboardhavoc/hoverboard-firmware/actions) has a
  `hoverboard-cli-<target>` artifact for Linux x86-64, macOS on Apple silicon, Windows and the
  Raspberry Pi. Or build it yourself (see [Building](building.md)).

`hoverboard-cli firmware info <bundle>` lists what a bundle holds.

## 2. Wire the bridge

Wire the SuperMini to the motor controller's SWD header:

| Motor controller | ESP32-C3 SuperMini |
|---|---|
| SWCLK | GPIO6 |
| SWDIO | GPIO7 |
| GND | G |
| NRST (optional) | GPIO5 |

The SWD header's 3V3 pin stays unconnected. On a split pair, each board gets its own bridge.

NRST is needed only to reach a board whose firmware blocks SWD: `board recover` (section 4) holds
the board in reset through it while the bridge attaches. The bridge never drives the pin high and
releases it except during `board recover`, so a board with NRST wired runs as one without.

### Power the bridge

The SuperMini needs 5 V on its 5V pin or its USB-C port. Any 5 V source works as long as its ground
is the motor controller's ground, which the GND wire above provides:

- **USB:** a computer, a USB hub or a phone charger on the SuperMini's USB-C port. This is the
  simplest on the bench, and the first install needs the USB cable anyway.
- **A buck converter from the battery:** for a machine that drives on its own battery. Its input
  goes to the battery, its 5 V output to the SuperMini's 5V pin, and its ground to the battery's
  negative.

The motor controller's own 5 V rail is probably not suitable: the ESP32-C3 draws a few hundred
milliamps in bursts when its Wi-Fi transmits, and the board's regulator may not be able to supply that.

## 3. Install the bridge firmware

Plug the SuperMini into the computer over USB. Put the Wi-Fi network and a name for the bridge in a
file, `bridge.json`:

```json
{ "device": { "name": "hb-left" }, "wifi": { "ssid": "my-network", "pass": "my-password" } }
```

Then install:

```
hoverboard-cli bridge install hoverboard-firmware-<commit>.hbfw bridge.json
```

The install erases the ESP32-C3 and writes the bridge image and the settings in the file. The bridge
joins the network at boot and answers on mDNS as `<name>.local`. Without Wi-Fi settings it still
runs Bluetooth and the USB console.

### Several ESP32-C3s plugged in

With one ESP32-C3 on USB, `bridge install` finds it by its USB ids (`303a:1001`). With more than
one, it refuses and lists them:

```
error: 2 ESP32-C3 USB ports (303a:1001): /dev/ttyACM0, /dev/ttyACM1; give --serial PORT
```

Name the one to install with `--serial`:

```
hoverboard-cli --serial /dev/ttyACM1 bridge install hoverboard-firmware-<commit>.hbfw bridge.json
```

To tell which port is which, unplug the SuperMini, list the ports, plug it back in and list them
again; the new entry is the one. The ports are `/dev/ttyACM*` on Linux, `/dev/cu.usbmodem*` on
macOS and `COM*` in the Device Manager on Windows. On Linux, `/dev/serial/by-id/` also names each
ESP32-C3 by its MAC address, and that path stays the same when the `ttyACM` numbers move:

```
--serial /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:63:93:77:36:F0-if00
```

If a bridge already answers on the port, `bridge install` names it and asks before erasing it.

## 4. Flash the motor controllers

Every command goes through a bridge, named by the link option before the command: Wi-Fi
(`--tcp hb-left.local:5333`), Bluetooth (`--device hb-left`) or USB
(`--serial <port>`).

Flash each board through its own bridge, one board at a time. On a split pair, do these steps
over one bridge, then over the other.

**Read the board's `uid`.** A board that does not run this firmware yet has no address or name;
it is named by its `uid`, the chip's factory serial number. To read it, name a board that does not
exist; the refusal names the chip on the bridge's SWD header:

```
$ hoverboard-cli --tcp hb-left.local:5333 board target-id x
error: no node named x (nodes: hb-left); this bridge's SWD target is 511e39363734360a30313337 (halted or not running?): name it by its uid prefix
```

If the refusal names no SWD target, the bridge reads no chip: check that the board is powered and
that SWCLK, SWDIO and GND are seated at both ends. If they are, the board's firmware may block SWD,
as some stock firmware does. Wire NRST to GPIO5 and run `board recover`, which holds the board in
reset, attaches, and halts it at its reset vector before its firmware runs:

```
$ hoverboard-cli --tcp hb-left.local:5333 board recover
hb-left: held in reset, attached, halted at the reset vector
SWD target b37b41333235390839393638 (gd32f1x0 64 KiB)
```

The board stays halted, and the next step flashes it without letting the old firmware run.

**Flash it,** naming the board by its `uid` or its first 4 or more hex digits:

```
hoverboard-cli --tcp hb-left.local:5333 board update 511e3936 hoverboard-firmware-<commit>.hbfw
```

`board update` reads the chip on the bridge's SWD header and flashes the bundle's image built for
it.

**Erase its settings store.** The flash rewrites only the firmware image; the settings area still
holds what the stock firmware left there, and the board refuses settings writes until it is
erased. The board now answers a walk, so `machine nodes` gives its address:

```
$ hoverboard-cli --tcp hb-left.local:5333 machine nodes
address  class             uid                       name
0x01     bridge            1463937736f0000000000000  hb-left
0x02     motor controller  511e39363734360a30313337  Hoverboard
$ hoverboard-cli --tcp hb-left.local:5333 board erase 0x02
```

A stock board may have its flash read-protected. `board clear-protection <board>` removes the
protection, which erases the chip, stock firmware included.

## 5. Configure the boards

A freshly flashed board drives nothing: it has no pins stored, so it brings up no motor. Its layout
(the gate, hall and current-sense pins, the dead time, the LEDs, the power latch and the link to the
other board) comes from a board file in [`boards/`](../boards/), at
`boards/<kind>/<layout>/<role>.json`: the folder names the board layout, and the file is `master`
or `slave`. The master is the board with the battery connector and the power button. A file made
for another chip is refused when it is applied.

Apply each file through the board's own bridge, then restart the board, since pins take effect at
boot:

```
hoverboard-cli --tcp hb-left.local:5333 board apply 0x02 boards/<layout>/<role>.json
hoverboard-cli --tcp hb-left.local:5333 board reset 0x02
```

`apply` checks the file against the board's chip before it writes anything, refuses a layout that
would drive the wrong pin, and prints each change. On a split pair, do the same for the other board
over its bridge, with its own file.

With both boards configured, the link between them is up, and one bridge reaches the whole machine:

```
hoverboard-cli --tcp hb-left.local:5333 machine nodes
```

The files leave out what depends on the wheels and the machine: each motor's direction and side,
the current limit, and the drive mode and gains. Those go in a machine file
(`machine apply`); [`shared/config-checks/machines/`](../shared/config-checks/machines/) holds
working examples.

## 6. Drive

- **From the phone:** the Android app finds the bridge over Bluetooth, arms the machine and drives
  it from a throttle pad.
- **From the CLI:** `machine drive <value> <secs>` runs every wheel at a throttle value for some
  seconds, then returns to neutral. Free the wheels before you try it.

`hoverboard-cli` with no command lists every command.
