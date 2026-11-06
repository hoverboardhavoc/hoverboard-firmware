# Building

CI builds every image on each push to `main`, so you only need this guide to change the firmware,
the bridge, the CLI or the app. To flash a board, take the bundle as
[Getting started](getting-started.md) describes.

## The folders

Each folder is its own build; run cargo or Gradle inside it. The repository root has no Cargo
workspace. Each Cargo workspace pins its toolchain in its own `rust-toolchain.toml`.

| Folder | Builds |
|---|---|
| `motorcontroller/` | the board images, the HAL, and the test and bench images |
| `shared/` | the crates the firmware, the bridge and the host tools share, and their Kotlin mirror |
| `bridge/` | the ESP32-C3 bridge image |
| `tools/` | `hoverboard-cli` and the other host programs and scripts |
| `apps/remote/` | the Android app |

## The board images

In `motorcontroller/`:

```
cargo image-f103     # GD32F103C8
cargo image-f130     # GD32F130C8
cargo image-f103rc   # GD32F103RC, the single board that drives both motors
```

Each writes `motorcontroller/target/<chip>/thumbv7m-none-eabi/release/firmware`. The F103 and F130
families have different peripheral register models, so each chip has its own image. The HAL builds
for one chip at a time; build its crates with `-p <crate> --features <chip>` or the aliases above.

`hoverboard-cli board flash <board> <elf>` writes a development ELF as it is, with no chip check.

## The CLI

In `tools/`:

```
cargo build --release -p hoverboard-cli
```

## A local bundle

```
./tools/pack-bundle.sh
```

It builds the three board images, the bridge's two images (with espflash 4.6.0) and
`hoverboard-cli`, then packs `target/bundle/hoverboard-firmware-<commit>.hbfw`, the same file CI publishes.
`--images DIR` packs images that are already built. A tree with uncommitted changes needs `--dirty`.

## Tests

- `./tools/host-test.sh` runs the host tests from anywhere and forwards cargo arguments such as
  `-p store`. A bare `cargo test` in `motorcontroller/` builds for the board and runs nothing.
- `./tools/size-vs-base.sh <base>` fails if any image's flash or RAM use grew against `<base>`. CI
  runs it against `main`.
- `cargo image-mm32` in `motorcontroller/` builds the MM32SPIN05 image only to measure it. It does
  not fit the chip's 32 KiB, and the CLI refuses to flash it.
- `cargo run -p golden-trace -- check` in `tools/` compares the HAL's register accesses with
  GigaDevice's peripheral library, run under Unicorn. Regenerating the vendor traces needs
  `arm-none-eabi-gcc` 15.2.Rel1 and cmake.

## The Android app

In `apps/remote/`, with JDK 17 and the Android SDK:

```
./gradlew assembleDebug
```
