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
| Rust gains an `OP_TELEMETRY` const the Kotlin does not mirror | exact-set compare fails on the extra entry, rather than passing on the subset |
| `crates/` absent above the module | all 11 drift cases fail naming the path they searched for |

In the Rust-side cases `WireDriftTest` passed while `RustSourceDriftTest` failed, which is the
clearest statement of why both files exist.

## CI

`.github/workflows/ci.yml` runs this suite in the `kotlin-protocol-drift` job, on every push and
pull request. It is the sixth check there and the only one with no Rust toolchain in it.

The job needs a JDK and nothing else. No Android SDK, because this module is deliberately not an
Android library; no `runtime-hal` checkout, because it reads `crates/` as text rather than building
it, and `crates/` is in this repo. It is `setup-java`, `setup-gradle` and one Gradle invocation,
seconds of work. The Android apps are **not** built in CI: both apply `com.android.application`,
which resolves the SDK at configuration time, so every task in those builds needs an SDK install
that catching protocol drift does not require.

JDK 17 is pinned because that is what this module targets: `build.gradle.kts` sets
`sourceCompatibility`/`targetCompatibility` to `VERSION_17` and `jvmTarget` to `JVM_17`, and both
consuming apps compile at 17 too. There is no `jvmToolchain()` here, so whichever JDK Gradle runs
on is the compiler, and 17 makes the compiling JVM and the bytecode target the same version.

The invocation is:

```sh
./gradlew test --rerun --no-build-cache --console=plain
```

`--rerun` and `--no-build-cache` are load-bearing. A plain `./gradlew test` reports
`Task :test UP-TO-DATE` whenever Gradle believes nothing changed, and executes nothing: a green
tick over zero tests, which is how this gate was already a no-op once. `build.gradle.kts` declares
`crates/**/*.rs` as inputs to the `test` task and that is the correct fix for up-to-date tracking,
but it is guarded on the `crates` directory existing and covers only `*.rs` beneath `crates/`, so
relying on it alone means relying on that declaration staying complete forever. The flags take the
question off the table. The suite runs in about two seconds, so there is nothing to save by
skipping it. A following step then reads the JUnit XML and fails if `RustSourceDriftTest`
contributed no executed cases, which catches the same failure from the other side.

### Do not put a path filter on that job

Whoever comes here to make CI faster will notice this job runs on commits that touch no Kotlin, and
reach for `paths: protocol-kotlin/**`. That inverts the gate into its own opposite.

Drift is introduced by editing the **Rust**. A commit that renumbers an opcode touches `crates/`
and nothing under `protocol-kotlin/`, so the filter would skip the job on precisely the commit the
gate exists to catch, and the mismatch would ship. The job would still fire on Kotlin-only edits,
where the author already has the protocol open in front of them and is least likely to get it
wrong. What is left is a job that is green on every commit and catching nothing, which reads as
coverage while providing none, and the failure resurfaces where it always did: a firmware change
ships, the phone app still builds and still runs, and it turns up as an unexplained silence on the
bench.

The workflow has no path filters at all today, so leaving this job unfiltered costs nothing and
needs no configuration. If filters are ever added, this job needs `crates/**` at minimum, and no
version of that list is cheaper to maintain than having no filter.
