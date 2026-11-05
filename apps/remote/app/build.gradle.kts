plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.detekt)
    alias(libs.plugins.roborazzi)
}

android {
    namespace = "com.hoverboardhavoc.remote"
    compileSdk = 36

    defaultConfig {
        applicationId = "com.hoverboardhavoc.remote"
        minSdk = 26
        targetSdk = 36
        versionCode = 2
        versionName = "0.1.1"

        // The bench smoke test (`src/androidTest/`, `specs/bench-ci.md` section 5), run on the bench
        // phone with `am instrument` against the real app and the real BLE transport.
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
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
        // For BuildConfig.DEBUG, which decides whether the ride screen carries the SIMULATE RIDER
        // control at all (ControlScreen). AGP 8 does not generate BuildConfig unless asked.
        buildConfig = true
    }

    @Suppress("UnstableApiUsage")
    testOptions {
        unitTests {
            isIncludeAndroidResources = true
            all {
                it.useJUnitPlatform()
                it.systemProperty("robolectric.graphicsMode", "NATIVE")
                it.systemProperty("roborazzi.test.record", "true")
            }
        }
    }
}

detekt {
    buildUponDefaultConfig = true
    config.setFrom(rootProject.file("detekt.yml"))
}

dependencies {
    // The shared Kotlin mirror of the firmware wire protocol (shared/protocol-kotlin).
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

    // BLE, Nordic Kotlin-BLE
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
    testImplementation(libs.roborazzi)
    testImplementation(libs.roborazzi.compose)
    testImplementation(libs.roborazzi.junit.rule)

    // The bench smoke test: the real activity under the Compose test rule.
    androidTestImplementation(composeBom)
    androidTestImplementation(libs.compose.ui.test.junit4)
    androidTestImplementation(libs.androidx.test.runner)
    androidTestImplementation(libs.androidx.test.ext.junit)
}
