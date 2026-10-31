plugins {
    id("org.jetbrains.kotlin.jvm") version "2.1.20"
    `java-library`
}

group = "com.hoverboard"
version = "0.1.0"

// Java 17 to match both consuming Android apps (compileOptions/jvmTarget 17 in each).
// Deliberately no jvmToolchain(): the only JDK on this machine is Android Studio's
// bundled JBR 21, and a toolchain request for 17 fails with no download repository
// configured. Compiling on 21 while targeting 17 keeps the artifact consumable.
java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
    }
}

dependencies {
    api("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.10.1")

    testImplementation(platform("org.junit:junit-bom:5.11.4"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testImplementation("org.jetbrains.kotlinx:kotlinx-coroutines-test:1.10.1")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
    testLogging {
        events("passed", "skipped", "failed")

        // The drift assertions carry their explanation in the failure MESSAGE ("linkctl opcode
        // allocation drifted from the Rust ==> expected: <...INPUTS=20...> but was: <...INPUTS=18...>",
        // or the repo-root check naming the file it could not find). The default console format
        // prints only the exception type and the line number, so a CI log would say a test failed
        // without saying what drifted. FULL puts the message where the person reading the failed
        // run actually is.
        exceptionFormat = org.gradle.api.tasks.testing.logging.TestExceptionFormat.FULL
    }

    // RustSourceDriftTest reads the firmware's Rust source and compares it to this mirror, so the
    // Rust files are genuine inputs to the test task. Without this, Gradle sees only Kotlin
    // sources, calls the task up to date after a firmware-only change, and the drift gate is
    // silently skipped exactly when it matters. Verified: mutating an opcode in crates/linkctl
    // with this declaration absent left `gradlew test` reporting BUILD SUCCESSFUL from cache.
    val crates = rootDir.parentFile?.resolve("crates")
    if (crates != null && crates.isDirectory) {
        inputs.files(fileTree(crates) { include("**/*.rs") })
            .withPropertyName("firmwareRustSources")
            .withPathSensitivity(PathSensitivity.RELATIVE)
    }

    // The same reasoning, for the one file outside crates/ that the gate reads:
    // theArmMirrorWindowOutlastsTheAppsKeepalive checks the firmware's arm-mirror window against the
    // rider app's INPUTS cadence, and reads that cadence out of the app's LinkConfig. The hazard the
    // test exists for is the APP moving, so an app-only edit is precisely the change that must not
    // find this task up to date. CI passes --rerun --no-build-cache and is safe either way; the
    // local loop is not, and this is where a cadence change would be missed.
    //
    // One named file rather than a tree, because that is exactly what the test reads and the test
    // names the same path. If it ever reads a second app file, this declaration has to grow with it.
    val linkConfig = rootDir.parentFile
        ?.resolve("apps/rider/app/src/main/java/com/hoverboard/remote/ble/LinkConfig.kt")
    if (linkConfig != null && linkConfig.isFile) {
        inputs.file(linkConfig)
            .withPropertyName("riderAppLinkConfig")
            .withPathSensitivity(PathSensitivity.RELATIVE)
    }
}

// The host-side board verdict the preset generator asks for (tools/robo-presets.py): a batch of
// staged layouts in, one `validate` verdict each out. It runs from the TEST runtime classpath
// because PresetVerdicts lives in the test source set, so a verification tool cannot leak into the
// library artifact the two Android apps consume.
tasks.register<JavaExec>("presetVerdicts") {
    description = "Runs board::validate's Kotlin mirror over a batch of staged layouts."
    group = "verification"
    classpath = sourceSets["test"].runtimeClasspath
    mainClass.set("com.hoverboard.protocol.board.PresetVerdicts")
}
