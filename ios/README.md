# Bush Drift for iPhone

The same C game as a native iOS app: raylib 6.2 on its SDL3 backend, drawing with Apple's OpenGL ES 3 (deprecated
by Apple since iOS 12 but still shipped, and run on Metal underneath). Landscape only, the whole screen, the
Android on-screen controls kept clear of the notch and the home bar. A Bluetooth or MFi game controller works through
SDL. iPhone only, iOS 15 and up.

## Build (on a Mac)
1. Xcode (with the iOS platform: Xcode → Settings → Components), then `brew install cmake xcodegen`.
2. `./ios/build-deps.sh`: fetches raylib 6.2 and SDL 3.4.18 (checksums checked) into `ios/third/` and builds
   SDL3 as a static xcframework in `ios/lib/`. About a minute.
3. The game's built files (`data/built/`, `assets-built/`) come from the "Game data 1" release:
   `./scripts/fetch-data.sh` downloads them if they're missing (step 2 runs it).
4. `./ios/gen-project.sh` writes `ios/BushDrift.xcodeproj` from `ios/project.yml`; open it in Xcode.
5. Simulator: pick an iPhone simulator and Run. Phone: see below.

## On your own iPhone, with a free Apple ID
1. Xcode → Settings → Accounts → + → Apple ID. A free "Personal Team" appears.
2. In the project: target BushDrift → Signing & Capabilities → Team → your Personal Team. If the bundle ID
   `io.github.aedyle.bushdrift` is taken, change it to something unique.
3. Plug the iPhone in with a cable; on the phone, tap **Trust** and enter the passcode.
4. On the phone: Settings → Privacy & Security → **Developer Mode** → on, restart, confirm.
5. In Xcode pick the iPhone as the run destination and press Run.
6. The first time, the phone refuses to open it: Settings → General → **VPN & Device Management** → your Apple ID
   → Trust. Then open Bush Drift.

**A free-account install stops opening after 7 days.** Plug the phone back in and press Run in Xcode again
(records are kept). A free account can have 3 apps on a device at once.

## What differs from the other builds (`#ifdef BD_IOS`, plus `BD_PHONE` shared with Android)
- `ios/bd_ios.c`: the app bundle is the working folder (the game's files are read from it); records and the live
  tuning file go to the app's own Application Support folder; nothing is drawn while the app is in the background;
  the safe area for the buttons; the screen viewport at full pixel resolution (raylib's SDL backend sets it in points).
- `ios/include/`: maps raylib's `<GLES3/gl3.h>` to Apple's headers, and sends raylib's "bind framebuffer 0"
  (the screen) to SDL's view framebuffer, because on iOS the screen is not framebuffer 0.
- `ios/bd_ios_audio.m`: the Ambient audio session, so the silent switch mutes the game and other apps' music
  keeps playing (raylib's audio library would otherwise ask for play-and-record, which ignores the switch).
- The home bar is dimmed and needs a second swipe while playing (SDL's "defer system gestures").
