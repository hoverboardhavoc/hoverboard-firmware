plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.detekt)
    alias(libs.plugins.roborazzi)
}

android {
    namespace = "com.hoverboard.remote"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.hoverboard.remote"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        compose = true
    }

    @Suppress("UnstableApiUsage")
    testOptions {
        unitTests {
            isIncludeAndroidResources = true
            all {
                it.useJUnitPlatform()
                it.systemProperty("robolectric.graphicsMode", "NATIVE")
                it.systemProperty("roborazzi.test.record", "true")
                // Gradle's default test heap is 512 MB, and this suite runs every Robolectric
                // class in one JVM with NATIVE graphics: each Compose rule brings up a real
                // renderer and Roborazzi holds a full-screen bitmap per capture. Adding the
                // telemetry-panel class tipped it into an OutOfMemoryError, which surfaced as the
                // unrelated-looking "uncaught exceptions before the test started" rather than as
                // an OOM. Sized for headroom, not measured.
                it.maxHeapSize = "2g"
            }
        }
    }
}

detekt {
    buildUponDefaultConfig = true
    config.setFrom(rootProject.file("detekt.yml"))
}

dependencies {
    // The shared Kotlin mirror of the firmware wire protocol (../../protocol-kotlin).
    // This replaced the app's own link/ package, which mirrored a retired protocol.
    implementation("com.hoverboard:protocol")

    // Compose
    val composeBom = platform(libs.compose.bom)
    implementation(composeBom)
    implementation(libs.compose.material3)
    implementation(libs.compose.ui)
    implementation(libs.compose.ui.tooling.preview)
    debugImplementation(libs.compose.ui.tooling)

    // Activity
    implementation(libs.activity.compose)

    // Lifecycle
    implementation(libs.lifecycle.viewmodel.compose)
    implementation(libs.lifecycle.runtime.compose)

    // Koin
    implementation(libs.koin.android)
    implementation(libs.koin.androidx.compose)

    // BLE — Nordic Kotlin-BLE
    implementation(libs.nordic.ble)
    implementation(libs.nordic.ble.client)
    implementation(libs.nordic.ble.core)

    // Detekt compose rules
    detektPlugins(libs.detekt.compose.rules)

    // Testing
    testImplementation(composeBom)
    testImplementation(platform(libs.junit.bom))
    testImplementation(libs.junit.jupiter)
    testImplementation(libs.junit4)
    testRuntimeOnly(libs.junit.platform.launcher)
    // Run JUnit4-based tests (Compose UI rule + Roborazzi) on the JUnit Platform.
    testRuntimeOnly(libs.junit.vintage.engine)
    testImplementation(libs.kotlinx.coroutines.test)
    testImplementation(libs.turbine)
    testImplementation(libs.robolectric)
    testImplementation(libs.androidx.test.core)
    testImplementation(libs.compose.ui.test.junit4)
    // debug, not test: this artifact exists to MERGE a bare ComponentActivity into the manifest,
    // which is the host a Compose test rule launches its content in. On testImplementation the
    // classes are there but the manifest entry is not, and every Compose rule test dies with
    // "Unable to resolve activity for Intent ... ComponentActivity".
    debugImplementation(libs.compose.ui.test.manifest)
    testImplementation(libs.roborazzi)
    testImplementation(libs.roborazzi.compose)
    testImplementation(libs.roborazzi.junit.rule)
}
