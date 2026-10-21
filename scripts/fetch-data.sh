#!/bin/sh
# Bush Drift: downloads the built game files (data/built/ and assets-built/) from the "Game data 2" release, if
# any are missing, so the game builds from a fresh clone. Rebuilding them yourself needs inputs that aren't in the
# repo (the Sketchfab model, NSW imagery): see README.md. Run by the iOS, Android and browser build scripts.
set -e
cd "$(dirname "$0")/.."
URL=${BD_DATA_URL:-https://github.com/aedyle/Bush-Drift/releases/download/data-2/bush-drift-data-2.zip}   # data-2 (build B): data-1 + assets-built/world.txt
SHA=fd9942d371fa9bab9383ef9482ed2c13c2ffe47f8448f96a4da7f76e2c905ab0
FILES="data/built/route.csv data/built/meta.txt data/built/terrain.bin data/built/canopy.bin data/built/far.bin
assets-built/plants.txt assets-built/hut.txt assets-built/world.txt assets-built/lc79.car"

missing=0
for f in $FILES; do [ -f "$f" ] || missing=1; done
[ $missing = 0 ] && exit 0

echo "fetching the game files ($URL)"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
curl -fsSL -o "$tmp/data.zip" "$URL"
if command -v sha256sum >/dev/null; then got=$(sha256sum "$tmp/data.zip" | cut -d' ' -f1); else got=$(shasum -a 256 "$tmp/data.zip" | cut -d' ' -f1); fi
[ "$got" = "$SHA" ] || { echo "fetch-data: checksum mismatch" >&2; exit 1; }
unzip -oq "$tmp/data.zip" -d .
echo "game files ready"
