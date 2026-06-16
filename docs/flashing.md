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
