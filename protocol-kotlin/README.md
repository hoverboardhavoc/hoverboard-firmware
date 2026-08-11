# protocol-kotlin

The single Kotlin mirror of the firmware's wire protocol, living beside the Rust it mirrors.

Both Android apps in this repo depend on it:

| App | Path | Package |
| --- | --- | --- |
| Protocol harness | `Hoverboard/` | `com.hoverboard.app` |
| Rider remote | `apps/rider/` | `com.hoverboard.remote` |

It exists because there used to be two hand-written Kotlin copies of the protocol, and they had
drifted apart: the rider's copy expected a version byte where the current framing carries a length,
had three of four opcodes wrong, discarded the one message the board actually emits, and modelled
`CyclicState` as 7 bytes against the firmware's 11.

## Layout

```
protocol-kotlin/
  build.gradle.kts        standalone Gradle build, pure Kotlin/JVM, Java 17
  settings.gradle.kts     rootProject.name = "protocol"  ->  com.hoverboard:protocol
  src/main/kotlin/com/hoverboard/protocol/
    l2/                   framing: SOF/len/CRC, fragmentation, reassembly   (crates/link)
    l3/                   PDU codec, addressing, the controller walk        (crates/net)
    linkctl/              the four L7 control payload families              (crates/linkctl)
    store/                CONFIG_* value type tags and encoding             (crates/store)
  src/test/kotlin/com/hoverboard/protocol/
    L2Test, PduTest, StoreWireTest, WalkTest, BleWalkTest    behaviour, ported with the code
    WireDriftTest                                            hand-copied wire pins
    RustSourceDriftTest                                      reads the Rust and compares
```

It is a **plain Kotlin/JVM library, not an Android library**. Nothing in it touches `android.*` or
`androidx.*`, and keeping it that way is deliberate: the tests then run with a JDK and nothing
else, no Android SDK, which is what makes the drift gate cheap enough to run on every change.

## Running the tests

Needs only a JDK. There is no JDK on `PATH` on this machine, so point at Android Studio's:

```sh
cd protocol-kotlin
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew test
```

Runs in a couple of seconds. The HTML report is at `build/reports/tests/test/index.html`.

## How the apps consume it

Each app is its own Gradle build with its own version catalog, and they disagree about DI (Hilt vs
Koin) and test tooling. So this is wired as a **composite build** rather than a shared subproject:
each app's `settings.gradle.kts` has

```kotlin
includeBuild("../protocol-kotlin")      // Hoverboard/
includeBuild("../../protocol-kotlin")   // apps/rider/
```

and declares `implementation("com.hoverboard:protocol")`. Gradle substitutes the coordinate onto
the included build. Nothing is published, and the two apps stay independent.

## The drift gate

The point of this module is that a firmware change which outruns the Kotlin fails a build rather
than a bench session. That takes two test files, and the second one is the one that matters.

**`WireDriftTest`** pins the Kotlin against expected values copied by hand out of the Rust, each
citing the `file:line` it came from. It covers opcode values, committed payload lengths, field
order via the golden byte vectors lifted verbatim from the Rust's own unit tests, the frame header
shape, the CRC choice and its coverage, the frag-header bit positions, the L3 header, the store
type tags, and the 19-byte single-fragment BLE budget for a `CYCLIC_STATE` PDU.

On its own that is only half a gate. It catches a careless edit to the **Kotlin**, but if the
**firmware** changes an opcode, the Kotlin and its hand-copied expectation still agree with each
other and everything stays green.

**`RustSourceDriftTest`** closes that. It locates the repo root, parses the Rust source, and
compares it to the Kotlin:

- `crates/linkctl/src/lib.rs`: opcodes, committed `LEN`s, struct field order and type, flag bits,
  supervision timeouts
- `crates/net/src/pdu.rs`: the `Opcode` enum, `HEADER_LEN`
- `crates/link/src/framer.rs`, `frag.rs`: framing and fragmentation constants
- `crates/store/src/key.rs`: value type tags
- `crates/base/src/crc16.rs`: the CRC algorithm the firmware instantiates

Where the Rust enumerates something, the comparison is an **exact set comparison**, so a firmware
change that *adds* a fifth control opcode also fails, rather than passing quietly and leaving this
mirror silently incomplete. If the Rust is restructured enough that a regex stops matching, the
test fails loudly naming the pattern that missed, instead of degrading into a no-op.

`build.gradle.kts` declares `crates/**/*.rs` as inputs to the `test` task. Without that Gradle sees
only Kotlin sources, calls the task up to date after a firmware-only change, and skips the gate
exactly when it is needed. That was observed, not theorised.

Both directions are verified by mutation:

| Mutation | Result |
| --- | --- |
| Kotlin `OP_INPUTS` 0x12 -> 0x50, `battery`/`wheelSpeed` swapped in `encode` | 5 tests fail |
| Rust `OP_INPUTS` 0x12 -> 0x14, `CyclicState` fields reordered | `RustSourceDriftTest` fails |
| Rust `OP_FAULT` 0x13 -> 0x19, no `--rerun-tasks` | fails from cold cache correctly |

In that last case `WireDriftTest` passed while `RustSourceDriftTest` failed, which is the clearest
statement of why both files exist.

## CI

**Recommendation: add a JDK-only job for this module, and do not build the Android apps in CI.**

Not added here, since that is the repo owner's call. The reasoning:

The expensive part of Android CI is the SDK, not the JDK: `setup-android`, licence acceptance and
platform/build-tools downloads dominate, and building both APKs adds minutes. None of that is
needed to catch protocol drift. This module is deliberately SDK-free, so the gate is
`setup-java` plus one Gradle invocation, seconds of work on a runner that already exists.

`.github/workflows/ci.yml` checks the firmware out into `hoverboard-firmware/`, so the working
directory below matches the existing jobs:

```yaml
  kotlin-protocol:
    name: Kotlin protocol mirror (drift gate)
    runs-on: ubuntu-latest
    defaults:
      run:
        working-directory: hoverboard-firmware/protocol-kotlin
    steps:
      - uses: actions/checkout@v4
        with:
          path: hoverboard-firmware
      - uses: actions/setup-java@v4
        with:
          distribution: temurin
          java-version: '17'
      - uses: gradle/actions/setup-gradle@v4
      - run: ./gradlew test
```

No `runtime-hal` checkout is needed: this module reads only `crates/`, which is in this repo.

Note the job must trigger on changes to `crates/**` as well as `protocol-kotlin/**`, since the
whole point is catching a firmware-side change. If the workflow ever grows path filters, a filter
that watches only `protocol-kotlin/**` would disable the gate while appearing to keep it.

**If it is left out**, the drift is caught by running `protocol-kotlin/gradlew test` locally
whenever `crates/linkctl`, `crates/link`, `crates/net`, `crates/store` or `crates/base/src/crc16.rs`
changes. That is a discipline rather than a gate, and it is worth being clear that the failure mode
it leaves open is the original one: a firmware change ships, the phone app still builds and still
runs, and the mismatch turns up as an unexplained silence on the bench.
