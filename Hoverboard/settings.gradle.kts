pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "Hoverboard"
include(":app")

// The Kotlin mirror of the firmware wire protocol, shared with apps/rider.
// It is a standalone Gradle build (pure Kotlin/JVM, no Android SDK) so it can be
// tested on its own with nothing but a JDK. Gradle substitutes the
// com.hoverboard:protocol dependency onto it automatically.
includeBuild("../protocol-kotlin")
