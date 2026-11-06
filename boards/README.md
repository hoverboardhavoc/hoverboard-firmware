# Board files

Each file here is a board file (`hoverboard-cli board apply`): the layout of one motor controller,
its pins, dead time, power latch, LEDs and the link to the other board. Apply one to a freshly
flashed board through its own bridge, then restart the board:

```
hoverboard-cli --tcp <bridge>:5333 board apply <board> boards/split/2.2.20/master.json
hoverboard-cli --tcp <bridge>:5333 board reset <board>
```

The getting started guide ([`docs/getting-started.md`](../docs/getting-started.md), section 5)
walks through it.

## split/

The split boards, one per wheel, joined by a UART cable. The numbers follow RoboDurden's layout
names (`defines_2-1-20.h`, `defines_2-2-20.h` in Hoverboard-Firmware-Hack-Gen2.x).

| Folder | Chip | Files |
|---|---|---|
| `2.1.20` | GD32F130C8 | `master.json`, `slave.json` |
| `2.2.20` | GD32F103C8 | `master.json`, `slave.json` |

Both layouts use the same pins: gates PA8, PA9, PA10 and PB13, PB14, PB15 on TIMER0 with dead time
28; halls PC13, PA1, PC14; phase current PA0 and PB0; LEDs PB3, PA15, PB4; the power latch PB12;
the link on USART1, PA2 and PA3; the IMU on I2C0, PB6 and PB7, as model 2 (the MPU-6050 clone these
boards carry, `WHO_AM_I` 0x2E) with the sign map `[1, 1, 1]`, which reads level for a board in the
stock hoverboard orientation (the compiled default `[-1, 1, -1]` reads it upside down). The GD32F130 files also give each pin's alternate function, which
that chip needs and the GD32F103 does not. A master file adds the battery sense (PA4), the
buzzer (PB9) and the horn at 415 Hz (`415/50`; the default 4 kHz is the buzzer's loudest pitch and a
shrill one).

Each file sets the motor's alignment offset to 1, the value every hoverboard hub motor in this
fleet has measured (the firmware's hall table is one sector off for them).

The files leave out what is measured on a machine or belongs to it: each motor's direction and side, its current limit, pole pairs and wheel size, the IMU's axis roles (they change only when a board is
mounted on edge), the device name, and the machine settings.
