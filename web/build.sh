#!/bin/sh
# Bush Drift in the browser (session 3b): emcc + raylib 6.2 built for the web (WebGL 2), one page in web/out/.
# Needs: emscripten (pacman), web/lib-web/libraylib.web.a (see web/README.md), and the game files (built, or fetched by scripts/fetch-data.sh).
set -e
cd "$(dirname "$0")/.."
export PATH=/usr/lib/emscripten:$PATH
make build/tuning_defaults.h >/dev/null
./scripts/fetch-data.sh   # the game files, if missing
mkdir -p web/out
emcc -O2 -std=c99 -D_DEFAULT_SOURCE -DNO_SAMPLES -DPLATFORM_WEB -Ibuild -Iweb/third/raylib-6.2/src \
    src/main.c src/car.c src/tuning.c src/bench.c src/records.c src/route.c src/erecords.c src/world.c \
    web/lib-web/libraylib.web.a -o web/out/index.html \
    -sUSE_GLFW=3 -sASYNCIFY -sASYNCIFY_STACK_SIZE=1048576 -sSTACK_SIZE=8388608 \
    -sALLOW_MEMORY_GROWTH=1 -sINITIAL_MEMORY=268435456 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 \
    -sFORCE_FILESYSTEM=1 -lidbfs.js -sEXPORTED_RUNTIME_METHODS=ccall -sEXPORTED_FUNCTIONS=_main,_bd_touch_seen \
    --shell-file web/shell.html \
    --preload-file data/built/route.csv --preload-file data/built/meta.txt --preload-file data/built/terrain.bin \
    --preload-file data/built/canopy.bin --preload-file data/built/far.bin --preload-file data/built/creek.csv --preload-file data/built/creek_dem.bin \
    --preload-file assets/sfx --preload-file assets-built/plants.txt --preload-file assets-built/hut.txt --preload-file assets-built/world.txt --preload-file assets-built/lc79.car --preload-file assets/music
ls -la web/out
