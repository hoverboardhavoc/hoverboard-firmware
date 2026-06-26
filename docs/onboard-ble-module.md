# Onboard BLE module: observable interface

**RoboDurden-style ClassyWalk 2-1-20 / 2-2-20 GD32 hoverboard mainboards** carry, on some units, an onboard
**CC2541-class transparent UART-to-BLE module** (a chip stamped **`TTC2541 F256`** has been observed). It is
a "dumb" bridge: the MCU talks to it over a UART, a short AT command sequence puts it into transparent mode,
and from then on every byte the MCU writes goes out over BLE and every byte a BLE central writes comes back
in on the UART.

Everything below was **observed on the bench**: the AT exchange was captured by instrumenting the MCU
firmware (recording the module's UART replies and reading them back over SWD), and the advertised name and
GATT layout were seen by scanning and discovering services from a BLE central (a host BLE tool and the
Android harness). The open firmware for these mainboards is RoboDurden's Hoverboard-Firmware-Hack-Gen2.x;
the board folder for
the GD32F130 v20 layout is
[here](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/tree/main/target_1%3DGD32F130/v20).
Note that in the board photo there the onboard module footprint is **unpopulated** -- the module is fitted
only on some units. The onboard-Bluetooth handling for these boards was added in a branch on top of that
firmware by this project, not by RoboDurden upstream.

## UART

- **9600 8N1**, fixed. On the 6-FET F103 master the module is wired to `USART2` (ST `USART3`), pins
  **PB10 (TX) / PB11 (RX)**.
- The module powers from the board's main rail (not the debug-probe 3.3 V), so it is only alive when the
  board is actually powered on.

## AT bring-up (command mode to transparent data mode)

On a cold boot the module is in command mode. The bring-up sequence is:

| step | command | reply |
|---|---|---|
| 0 | `AT\r\n` | `AT+OK\r\n` |
| 1 | `AT+NAME=<name>\r\n` | `AT+OK\r\n` |
| 2 | `AT+CON_INTERVAL=<n>\r\n` | `AT+OK\r\n` |
| 3 | `AT+ADV_INTERVAL=<n>\r\n` | `AT+OK\r\n` |
| 4 | `AT+SET=1\r\n` | `AT+OK\r\n` (twice) |
| 5 | `AT+MODE=DATA\r\n` | `AT+OK\r\n`, then transparent |

The interval values are small integers (defaults 16 and 32). After step 5 the module is a transparent
bridge and no longer interprets AT.

### Critical: the host must consume every ack

The module replies `AT+OK\r\n` to **every** command (step 4 replies twice). The MCU UART has a **1-byte RX
register with no FIFO**, so the host must read each reply promptly. If you send the commands on a blind
timer without reading the replies, the configuration commands silently do **not** take effect (the name and
intervals never change), even though the bytes were transmitted correctly. Drain the reply by polling RX
faster than the wire rate (about 1 byte/ms at 9600), or use interrupt/DMA-driven RX. Waiting for each
`AT+OK` before the next command is the robust approach.

## Quirks worth knowing

- **Minimal command set.** Only the six commands above are accepted. Other common AT queries
  (`AT+VERSION`, `AT+HELP`, `AT+NAME?`, `AT+ADDR?`, `AT+ROLE?`, `AT+BAUD?`, `AT+PIN?`, and `?` query forms)
  return `AT+ERR=2`. There is no readable version string. Whether `AT+BAUD=<n>` (set form) is supported is
  unconfirmed; the data-mode baud is fixed at 9600 in practice.
- **The advertised name is space-padded** to a fixed width. A name set as `hbk2` is advertised as
  `"hbk2              "`. Trim trailing spaces when matching it.
- **The name persists** in the module's own non-volatile memory, independent of MCU flash. It survives MCU
  reflashes and power cycles, so a name you see may be from an earlier session, not the current firmware.
- **`AT+MODE=DATA` is terminal.** To reconfigure, the module must be cold-booted (power-cycled) back to
  command mode. While in transparent mode it ignores AT (the bytes pass through to the UART).

## GATT layout (transparent mode)

Once in transparent data mode the module exposes a vendor service. On the boards examined here:

| service | characteristic | properties | role |
|---|---|---|---|
| `0x1000` | `0x1001` | write-without-response, write, **notify** | host writes data here (central to board) |
| `0x1000` | `0x1002` | read, **notify** | board's UART output arrives here (board to central) |

The two directions are on **separate characteristics**: a central writes to `0x1001` and receives the
board's output via notifications on `0x1002`.

### Robust central integration

The module is dumb and the exact UUIDs should not be trusted blindly. The rules that work reliably:

1. **Scan by the live advertised name** from the scan record, **trimmed** (the name is space-padded, and a
   cached `BluetoothDevice.name` on Android is often stale).
2. **Pick the write characteristic by the write + notify property pair.** Selecting "the first writable
   characteristic" wrongly lands on a standard GATT characteristic (e.g. Generic Access `0x2a02`); the data
   characteristic is distinguished by advertising **both** write and notify.
3. **Subscribe to every notify-capable characteristic.** The board's output arrives on a separate notify
   characteristic (`0x1002`), so subscribing only the write characteristic's own notify misses it.
4. The carrier is **ordered but not reliable** (BLE write-without-response and notifications can drop, and
   the module-to-MCU UART can overflow), so an application protocol over this pipe needs its own
   framing/ack/retransmit.

## See also

- [BLE throughput harness](../crates/ble/test-harness/README.md) — how to measure how many bytes per second
  this pipe actually carries (a phone central plus a board-side byte-loopback firmware), including the
  headless ADB drive mode and the device-specific gotchas.
