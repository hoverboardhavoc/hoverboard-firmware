#!/usr/bin/env bash
# Build the Android app the way F-Droid does: fdroidserver's `fdroid build` on the committed HEAD,
# from the metadata in apps/remote/fdroid/, with the source taken from this checkout rather than
# from GitHub. A failure here is a failure F-Droid's build server would hit.
#
# Usage: tools/fdroid-build.sh [WORKDIR]      (default WORKDIR: target/fdroid)
#
# Needs `fdroid` (fdroidserver) on PATH, a JDK 17, ANDROID_HOME, and a Gradle 8.14 binary in GRADLE
# or on PATH: F-Droid's scanner deletes the committed gradle-wrapper.jar before it builds, so the
# wrapper cannot run. Commit first; uncommitted changes are not built.
set -euo pipefail

repo=$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)
app=com.hoverboardhavoc.remote
work=${1:-$repo/target/fdroid}
gradle=${GRADLE:-$(command -v gradle || true)}
if [ -z "$gradle" ]; then
  echo "fdroid-build: no Gradle; set GRADLE to a Gradle 8.14 binary" >&2
  exit 1
fi
sha=$(git -C "$repo" rev-parse HEAD)

rm -rf "$work"
mkdir -p "$work/metadata" "$work/build"
cd "$work"
{
  echo "sdk_path: ${ANDROID_HOME:?ANDROID_HOME is not set}"
  echo "gradle: $gradle"
} > config.yml
chmod 600 config.yml
# The metadata template takes its version from the one place it is set, the app's build file, and
# builds HEAD from this checkout rather than a tag from GitHub.
gradle_file="$repo/apps/remote/app/build.gradle.kts"
name=$(sed -n 's/^ *versionName = "\(.*\)"$/\1/p' "$gradle_file")
code=$(sed -n 's/^ *versionCode = \([0-9][0-9]*\)$/\1/p' "$gradle_file")
if [ -z "$name" ] || [ -z "$code" ]; then
  echo "fdroid-build: no literal versionName and versionCode in $gradle_file" >&2
  exit 1
fi
sed -e "s|@VERSION_NAME@|$name|g" -e "s|@VERSION_CODE@|$code|g" -e "s|@COMMIT@|$sha|g" \
  -e "s|^Repo: .*|Repo: $repo|" \
  "$repo/apps/remote/fdroid/$app.yml" > "metadata/$app.yml"
git clone -q "$repo" "build/$app"

fdroid build -v -l "$app" --no-tarball
ls -l unsigned/"${app}"_*.apk
