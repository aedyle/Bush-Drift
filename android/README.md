# Bush Drift for Android

The same C game as a native Android app (a NativeActivity, no Java): arm64 phones and x86_64 (Chromebooks,
emulators), Android 10 and up. On-screen touch controls sit either side of the 4:3 picture: steering on the
left; gas, brake and handbrake on the right; pause and restart at the top. A Bluetooth gamepad or keyboard works too.

- **Download:** https://github.com/aedyle/Bush-Drift/releases/latest/download/bush-drift.apk
- **Build:** `./android/build.sh` writes `android/out/bush-drift.apk`, signed.

## One-time setup
1. A JDK (17), and the Android command-line tools in `~/.local/opt/android-sdk` (or set `ANDROID_HOME`), then
   `sdkmanager "platform-tools" "build-tools;35.0.1" "platforms;android-35" "ndk;27.3.13750724"`.
2. raylib 6.2 for Android, for each ABI (from the raylib source in `web/third/raylib-6.2/src`):
   ```
   make PLATFORM=PLATFORM_ANDROID ANDROID_NDK=$ANDROID_HOME/ndk/27.3.13750724 ANDROID_ARCH=arm64 \
        ANDROID_API_VERSION=29 GRAPHICS=GRAPHICS_API_OPENGL_ES3 RAYLIB_RELEASE_PATH=<repo>/android/lib/arm64-v8a
   ```
   and again with `ANDROID_ARCH=x86_64` into `android/lib/x86_64` (`make clean` between).
3. A signing key, kept out of the repo: `keytool -genkeypair -keystore ~/.local/share/bush-drift/release.keystore
   -alias bushdrift -keyalg RSA -keysize 4096 -validity 10000`, with its password in
   `~/.local/share/bush-drift/keystore.pass`. **Every update must be signed with the same key**, or phones refuse
   to install it over the old version.

## What differs from the desktop build (`#ifdef __ANDROID__`)
- The window is the whole display; game files are read from inside the APK (raylib's `fopen` wrapper,
  `-Wl,--wrap=fopen`); records are written to the app's own storage.
- GLSL ES 3.00 shaders; the back button acts as Esc; no Quit on the title screen; the readout starts hidden.
- Testing on the desktop: `BD_TOUCH=1` shows the touch buttons (the mouse can press them).
