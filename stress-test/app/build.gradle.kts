// Standalone gradle project for the BLE link stress-test app. Same gradle stack as Hoverboard/app
// (AGP 8.9.1, Kotlin 2.1.20) but trimmed to the headless harness essentials: no Compose, no Hilt, no
// detekt — just the Nordic Kotlin-BLE transport + coroutines.
plugins {
    id("com.android.application") version "8.9.1" apply false
    id("org.jetbrains.kotlin.android") version "2.1.20" apply false
}
