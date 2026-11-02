#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Fetch MindMotion's LibSamples for the MM32SPIN0x (q), V1.13, the MM32SPIN05 oracle of the golden
# register traces (specs/hal.md, section 8.1). Its files carry MindMotion's copyright and no licence
# that grants redistribution, so it is not committed: this script downloads the zip into
# third_party/.cache/, checks its SHA-256 and unpacks it into third_party/mm32-libsamples/ (both
# gitignored). tools/golden-trace refuses the MM32 vectors without the verified copy this leaves.
#
# The site's firewall refuses curl's default user agent, so the request names a browser's. A zip
# already in the cache is checked and reused, which is what CI's cache (keyed by the hash) supplies.
set -euo pipefail

URL="https://www.mindmotion.com.cn/uploadfiles/2025/03/LibSamples_MM32SPIN0x_q_V1.13.zip"
SHA256="eb294c20129a474daf6295e03f42667f14c4826b1639c2afbf3bad3c2e4a8cbd"
UA="Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0 Safari/537.36"

root="$(cd "$(dirname "$0")/.." && pwd)"
cache="$root/third_party/.cache"
zip="$cache/LibSamples_MM32SPIN0x_q_V1.13.zip"
dest="$root/third_party/mm32-libsamples"

sha() {
    if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi
}

mkdir -p "$cache"
if [ -f "$zip" ] && [ "$(sha "$zip")" != "$SHA256" ]; then
    echo "fetch-vendor: cached $zip does not match $SHA256; fetching again" >&2
    rm -f "$zip"
fi
if [ ! -f "$zip" ]; then
    curl -fsSL --retry 3 -A "$UA" -o "$zip.part" "$URL"
    mv "$zip.part" "$zip"
fi
got="$(sha "$zip")"
if [ "$got" != "$SHA256" ]; then
    echo "fetch-vendor: $URL has SHA-256 $got, expected $SHA256; refusing it" >&2
    rm -f "$zip"
    exit 1
fi

rm -rf "$dest.part"
mkdir -p "$dest.part"
unzip -q "$zip" -d "$dest.part"
# The verification stamp tools/golden-trace checks before it runs an MM32 vector.
echo "$SHA256" > "$dest.part/SHA256"
rm -rf "$dest"
mv "$dest.part" "$dest"
echo "fetch-vendor: LibSamples V1.13 verified ($SHA256) in third_party/mm32-libsamples/"
