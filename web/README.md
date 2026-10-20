# Bush Drift in the browser

The same C game built with Emscripten for WebGL 2.

- **Play:** https://aedyle.github.io/Bush-Drift/ (the play-test page; the game itself is `play.html`).
  Arrows steer, accelerate and brake; Space is the handbrake. R restarts, H changes the handling, P the pixel size, F1 shows the readout.
- **Build:** `./web/build.sh` writes `web/out/` (any static web server can serve it).
- **Publish to GitHub Pages:** `./web/deploy-pages.sh` builds, puts the play-test page (`web/playtest.html`) in front of the
  game and pushes the lot to the `gh-pages` branch.

## One-time setup
- `sudo pacman -S emscripten` (emcc in /usr/lib/emscripten).
- raylib 6.2 for the web (matches the desktop raylib):
  ```
  mkdir -p web/third web/lib-web && cd web/third
  curl -L -o raylib-6.2.tar.gz https://github.com/raysan5/raylib/archive/refs/tags/6.2.tar.gz && tar xzf raylib-6.2.tar.gz
  cd raylib-6.2/src && make PLATFORM=PLATFORM_WEB GRAPHICS=GRAPHICS_API_OPENGL_ES3 RAYLIB_RELEASE_PATH=$PWD/../../../lib-web
  ```

## What differs from the desktop build (all `#ifdef __EMSCRIPTEN__`)
- No worker thread: chunks are built on the main loop, one per frame when one is needed (about 7 ms, worst about 15 ms).
- Shaders use GLSL ES 3.00, to match raylib's own WebGL 2 shaders.
- The title menu has no Quit, and its hints are for the keyboard. The readout starts hidden.
- The loop runs as written: raylib yields to the browser in `WindowShouldClose` (Asyncify).
- Records live only in the page's memory: they reset when the page reloads.
