#!/bin/sh
# Bush Drift for iOS: fetches raylib 6.2 and SDL 3.4.18 into ios/third/ and builds SDL3 as a static
# xcframework (iPhone + Simulator, arm64) in ios/lib/. Run once on the Mac, then ./ios/gen-project.sh.
# Needs Xcode and cmake (brew install cmake).
set -e
cd "$(dirname "$0")"
RAYLIB=6.2 RAYLIB_SHA=df7a13129b1bec9e73a6962efff170beba310ee6eab8f94cd000b7316b4db5ee
SDL=3.4.18 SDL_SHA=9c75cf16330322c217dedd2e0609f1124f1b54b8633e763467b4684d0f4334a3
MIN_IOS=15.0
mkdir -p third lib

fetch() {   # url file sha
    [ -f "third/$2" ] || curl -fsSL -o "third/$2" "$1"
    echo "$3  third/$2" | shasum -a 256 -c - >/dev/null || { echo "checksum mismatch: third/$2" >&2; exit 1; }
}
fetch https://github.com/raysan5/raylib/archive/refs/tags/$RAYLIB.tar.gz raylib-$RAYLIB.tar.gz $RAYLIB_SHA
fetch https://github.com/libsdl-org/SDL/releases/download/release-$SDL/SDL3-$SDL.tar.gz SDL3-$SDL.tar.gz $SDL_SHA
[ -d third/raylib-$RAYLIB ] || tar xzf third/raylib-$RAYLIB.tar.gz -C third
[ -d third/SDL3-$SDL ] || tar xzf third/SDL3-$SDL.tar.gz -C third

if [ ! -d lib/SDL3.xcframework ]; then
    for sdk in iphoneos iphonesimulator; do
        cmake -S third/SDL3-$SDL -B third/sdl-build-$sdk -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=$sdk \
            -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_OSX_DEPLOYMENT_TARGET=$MIN_IOS -DCMAKE_BUILD_TYPE=Release \
            -DSDL_SHARED=OFF -DSDL_STATIC=ON -DSDL_TEST_LIBRARY=OFF -DSDL_EXAMPLES=OFF \
            -DSDL_CAMERA=OFF -DSDL_GPU=OFF -DSDL_RENDER_METAL=OFF -DSDL_VULKAN=OFF -DSDL_METAL=OFF \
            -DCMAKE_INSTALL_PREFIX="$PWD/third/sdl-install-$sdk" >/dev/null
        cmake --build third/sdl-build-$sdk --config Release -j "$(sysctl -n hw.ncpu)" >/dev/null
        cmake --install third/sdl-build-$sdk --config Release >/dev/null
    done
    xcodebuild -create-xcframework \
        -library third/sdl-install-iphoneos/lib/libSDL3.a -headers third/sdl-install-iphoneos/include \
        -library third/sdl-install-iphonesimulator/lib/libSDL3.a -headers third/sdl-install-iphonesimulator/include \
        -output lib/SDL3.xcframework >/dev/null
fi
echo "ios deps ready: third/raylib-$RAYLIB, lib/SDL3.xcframework"
