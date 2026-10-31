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

rootProject.name = "BLEHoverboardRemote"
include(":app")

// The Kotlin mirror of the firmware wire protocol.
// Standalone Gradle build (pure Kotlin/JVM); Gradle substitutes com.hoverboard:protocol onto it.
includeBuild("../../shared/protocol-kotlin")
