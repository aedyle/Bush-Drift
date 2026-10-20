#!/bin/sh
# Bush Drift for Android (session 3b): builds android/out/bush-drift.apk (arm64 and x86_64, Android 10 and up).
# Needs the Android SDK (ANDROID_HOME, default ~/.local/opt/android-sdk) with build-tools 35.0.1, platform 35
# and NDK 27.3; raylib built for Android in android/lib/arm64-v8a/ (see android/README.md); and a signing key
# (BUSHDRIFT_KEYSTORE, default ~/.local/share/bush-drift/release.keystore, password in the file named by
# BUSHDRIFT_KEYPASS, default ~/.local/share/bush-drift/keystore.pass). The same key must sign every update.
set -e
cd "$(dirname "$0")/.."
SDK=${ANDROID_HOME:-$HOME/.local/opt/android-sdk}
NDK=$SDK/ndk/27.3.13750724
BT=$SDK/build-tools/35.0.1
JAR=$SDK/platforms/android-35/android.jar
KS=${BUSHDRIFT_KEYSTORE:-$HOME/.local/share/bush-drift/release.keystore}
KP=${BUSHDRIFT_KEYPASS:-$HOME/.local/share/bush-drift/keystore.pass}
OUT=android/out
make build/tuning_defaults.h >/dev/null
rm -rf $OUT && mkdir -p $OUT/apk/assets

# the game as libmain.so for each ABI (raylib's android_main calls our main): phones (arm64) and x86_64
# (Chromebooks, emulators, Waydroid)
for abi in arm64-v8a x86_64; do
    case $abi in arm64-v8a) tgt=aarch64;; x86_64) tgt=x86_64;; esac
    mkdir -p $OUT/apk/lib/$abi
    $NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/$tgt-linux-android29-clang -O2 -std=c99 -D_DEFAULT_SOURCE -DNO_SAMPLES -DPLATFORM_ANDROID -fPIC -shared \
        -Ibuild -Iweb/third/raylib-6.2/src -I$NDK/sources/android/native_app_glue \
        src/main.c src/car.c src/tuning.c src/bench.c src/records.c src/route.c src/erecords.c \
        android/lib/$abi/libraylib.a \
        -llog -landroid -lEGL -lGLESv3 -lOpenSLES -lm \
        -u ANativeActivity_onCreate -Wl,--wrap=fopen -Wl,--no-undefined -Wl,-z,max-page-size=16384 \
        -o $OUT/apk/lib/$abi/libmain.so
done

# the game's files, at the same relative paths the game opens
mkdir -p $OUT/apk/assets/data/built $OUT/apk/assets/assets-built $OUT/apk/assets/assets/sfx
cp data/built/route.csv data/built/meta.txt data/built/terrain.bin data/built/canopy.bin data/built/far.bin $OUT/apk/assets/data/built/
cp assets-built/plants.txt assets-built/hut.txt assets-built/lc79.car $OUT/apk/assets/assets-built/
cp assets/sfx/*.ogg $OUT/apk/assets/assets/sfx/

# resources and manifest, then the native library, then align and sign
$BT/aapt2 compile --dir android/res -o $OUT/res.zip
$BT/aapt2 link -o $OUT/unaligned.apk -I $JAR --manifest android/AndroidManifest.xml -A $OUT/apk/assets $OUT/res.zip
(cd $OUT/apk && zip -q -r ../unaligned.apk lib)
$BT/zipalign -f -p 4 $OUT/unaligned.apk $OUT/aligned.apk
$BT/apksigner sign --ks "$KS" --ks-pass file:"$KP" --out $OUT/bush-drift.apk $OUT/aligned.apk
$BT/apksigner verify $OUT/bush-drift.apk
rm -f $OUT/unaligned.apk $OUT/aligned.apk $OUT/res.zip
ls -la $OUT/bush-drift.apk
