# hoverboard-cli reference

```
hoverboard-cli [--device NAME|ADDR | --serial DEVICE | --tcp HOST:PORT] <command>
```

## Links

The CLI reaches one bridge, and through it every node of the machine: the boards forward frames
to each other over the link between them. The link option comes before the command.

| Option | Link |
|---|---|
| `--tcp HOST:PORT` | the bridge over Wi-Fi, on port 5333, for example `--tcp hoverboard-bridge-left.local:5333` |
| `--device NAME\|ADDR` | the bridge over Bluetooth, by its name or address |
| `--serial DEVICE` | the bridge's USB console |
| none | Bluetooth, to the first bridge that advertises as `hb-bridge-...` |

## Naming a node

A `<board>` or `<bridge>` argument is read in this order:

1. an address, `0x02` or a decimal from 1 to 127;
2. an exact `device.name`;
3. a prefix of 4 to 24 hex digits of the node's factory `uid`.

An address needs no walk of the machine; a name or a `uid` prefix does. A board that does not run
this firmware, or a halted one, answers no walk. Name it by its `uid` prefix through its own bridge,
which reads the `uid` over SWD. A `board` command refuses a bridge and a `bridge` command refuses a
motor controller.

## Settings files

Settings are JSON files whose nesting follows the field paths: `imu.model` is
`{ "imu": { "model": 2 } }`. A board file holds one board's settings. A machine file holds a
`machine` object, written to every motor controller, and a `boards` object with one entry per board,
keyed by its `uid`. The files in `shared/config-checks/machines/` are examples.

Every write is checked first: the CLI reads the board's chip and refuses a layout the chip cannot
drive, a pin used twice, or a value out of range. A refused write writes nothing and lists every
refusal. A board refuses writes while it is armed. A changed setting takes effect at the next arm.

`apply`, `replace` and `diff` treat files the same way under `board`, `bridge` and `machine`:

- **`apply`** merges: it writes each setting whose value differs and never un-stores one.
- **`replace`** makes the node equal the files: it also un-stores every setting no file names, and
  asks first unless `--yes`.
- **`diff`** prints what `apply`, or with `--replace` what `replace`, would change, and writes
  nothing. It exits 0 with no changes, 1 with changes and 2 on an error.

`--force` writes a key or an index the CLI's name table does not know. It never overrides the
configuration checks.

## `scan`

| Command | What it does |
|---|---|
| `scan [--secs N]` | lists the Bluetooth bridges in range |

## `board`

One motor controller.

| Command | What it does |
|---|---|
| `board get <board> <path>` | prints one setting, marked `(default)` when nothing is stored |
| `board set <board> <path> <value>` | writes one setting; `null` un-stores it |
| `board dump <board> [<file>]` | writes the board's stored settings as a board file, to stdout without `<file>` |
| `board apply <board> <file>...` | merges the files into the board |
| `board replace <board> <file>... [--yes]` | makes the board equal the files |
| `board diff [--replace] <board> <file>...` | prints what `apply` or `replace` would change |
| `board status [<board>...]` | prints each board's boot report, `ok` or one line per problem; every motor controller without `<board>` |
| `board compact <board>` | compacts the settings store |
| `board fields --json` | prints the name table: every setting the CLI knows |
| `board erase <board> [--yes]` | erases the settings store, keeping the board's address |
| `board update <board> <bundle> [--image NAME] [--yes]` | flashes the bundle's image for the board's chip; `--image` picks one when several match |
| `board flash <board> <elf>` | flashes a development ELF as it is, with no chip check |
| `board clear-protection <board>` | removes flash read protection, which erases the chip |
| `board disarm\|halt\|resume\|reset\|reset-halt <board>` | controls the board's core over SWD |
| `board target-id <board>` | prints the chip the bridge reads over SWD: family, flash size and `uid` |
| `board recover` | holds the board on the attached bridge's SWD header in reset (NRST on GPIO5), attaches and halts it at its reset vector, for a board whose firmware blocks SWD |
| `board read <board> <addr> <len> [--hz N]` | reads memory over SWD, once or `N` times a second until interrupted |
| `board write <board> <addr> <hex>` | writes whole words to a word-aligned address over SWD |

`update`, `flash` and the SWD commands go through the bridge wired to the board.

## `machine`

Every motor controller the link reaches.

| Command | What it does |
|---|---|
| `machine nodes` | lists every node: address, class, `uid` and name, and marks a board with a pending change or a boot problem |
| `machine get <path>` | prints a machine setting from every motor controller, one line per board when the copies differ |
| `machine set <path> <value>` | writes a machine setting to every motor controller |
| `machine dump <file> [--from <board>]` | writes the machine file; `--from` picks the copy when the boards differ |
| `machine apply <file> [--map <uid>=<board>]...` | writes `machine` to every board and each `boards` entry to the board with that `uid` |
| `machine replace <file> [--yes] [--map ...]` | as `apply`, making each listed board equal its entry plus `machine` |
| `machine diff [--replace] [<file>] [--map ...]` | prints what `apply` or `replace` would change; with no file, the machine settings whose copies differ |
| `machine watch <secs> [<board>...]` | prints the boards' live state for `secs` seconds |
| `machine drive <value> <secs> [<board>[:<motor>]...]` | runs the wheels at throttle `value` for `secs` seconds, then neutral; a bare board is every motor it has, and no target is every wheel |
| `machine update <bundle> [--yes]` | updates every bridge and the board behind each, one pair at a time, after one question; stops at the first failure |

`--map <uid>=<board>` pairs a file's entry with a board whose `uid` differs, such as a replacement
board.

## `bridge`

One bridge.

| Command | What it does |
|---|---|
| `bridge info <bridge>` | prints the bridge's chip and Wi-Fi state |
| `bridge get\|set\|dump\|apply\|replace\|diff` | as under `board`, on the bridge's settings: `device.name`, `wifi.ssid`, `wifi.pass` |
| `bridge update <bridge> <bundle\|bin> [--yes] [--no-confirm]` | writes a new bridge image over the link, then reconnects and confirms it |
| `bridge install <bundle\|image> [<file>...] [--yes]` | installs a new bridge over USB, erasing the ESP32-C3, and writes the settings files |

A bridge reads its name and Wi-Fi settings at boot, so a change takes effect at the next boot.

## `firmware`

No link.

| Command | What it does |
|---|---|
| `firmware info <bundle>` | prints the bundle's commit, build time and images |
| `firmware pack <out.hbfw> <crate-dir>:<image-name>=<elf>... [--dirty]` | builds a bundle; `./tools/pack-bundle.sh` runs it for you |
