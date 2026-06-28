plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.hoverboard.stress"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.hoverboard.stress"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }
}

dependencies {
    // BLE — Nordic Kotlin-BLE (same versions as Hoverboard/app, so the transport behaves identically).
    implementation("no.nordicsemi.android.kotlin.ble:scanner:1.3.1")
    implementation("no.nordicsemi.android.kotlin.ble:client:1.3.1")
    implementation("no.nordicsemi.android.kotlin.ble:core:1.3.1")

    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.10.1")
}
