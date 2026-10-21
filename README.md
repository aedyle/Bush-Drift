# Bush Drift

A small drifting game on a real dirt road: 7.8 km from Alexanders Hut in the South East Forest National Park
(NSW) down Robinsons Track and Cattlemens Track to the Tantawangalo Creek bridge, in a LandCruiser 79 ute.
Gravel, ditches, tall wet forest either side, real hills from the NSW elevation model, and a pixel-art look.
Hit a tree or a post and the run is over.

**Play in the browser:** https://aedyle.github.io/Bush-Drift/ (a play-test build; keyboard).

## Controls

| | keyboard |
|---|---|
| steer | ← → |
| accelerate / brake | ↑ / ↓ |
| handbrake | Space |
| bank the drift points | B |
| restart / handling preset / pixel size | R / H / P |
| readout | F1 |

A gamepad works too in the desktop build.

## Scoring

Slide and your sideways speed fills the pot. Keep sliding and the multiplier climbs (up to ×9). Drive
straight for a moment, or press B, to bank pot × multiplier. Crash and the pot is lost. The run ends at the
bridge, at a tree or post, or after driving the wrong way.

## Building

Desktop (Linux; needs raylib 6.x and its pkg-config file):

```
make route          # fetch and build the road data into data/ (cached downloads; see scripts/README.md)
blender -b --python tools/make_plants.py -- assets-built/plants.txt
blender -b --python tools/make_hut.py -- assets-built/hut.txt
blender -b --python tools/make_79.py -- src-models/j79-offroad/scene.gltf assets-built/lc79.car docs/car
make bush-drift && ./bush-drift
```

The 79 model is downloaded by hand from Sketchfab (link in `docs/CREDITS.md`) into `src-models/j79-offroad/`.

**Or skip those four build steps:** `./scripts/fetch-data.sh` downloads the built road and model files from the
[Game data 1](https://github.com/aedyle/Bush-Drift/releases/tag/data-1) release (the same files the apps ship).

Browser: `./web/build.sh` (Emscripten; see `web/README.md`). Android: `./android/build.sh` (`android/README.md`).
iPhone: on a Mac with Xcode (`ios/README.md`).

Checks: `./bush-drift --cartest` (the handling against `docs/handling-spec.md`), `--routetest`, `--drivetest`.

## What's in it

- `src/car.c`: the ute's handling, written from `docs/handling-spec.md`: a planar rigid body on four tyres
  with load transfer, a gravel tyre curve, a 2.8 turbo-diesel torque curve and a five-speed gearbox.
  Numbers in `tuning-car.txt`.
- `src/route.c`: the road (OpenStreetMap line refined against NSW imagery), its ground and ditches, the
  forest placed by vegetation zone, Alexanders Hut and the campground, streamed in chunks.
- `src/main.c`: window, input, chase camera, the low-resolution palette look, sun shadows, sky and haze,
  generated sound, scoring, menus and records.
- `scripts/`: the map-data pipeline. `tools/`: the Blender scripts that make the ute, the plants and the
  hut, and the photo-matching tools used to place the hut.

## Credits

See `docs/CREDITS.md`. In short: road line © OpenStreetMap contributors (ODbL); elevation and tree cover
from NSW DEM and imagery © State of NSW (Spatial Services) (CC BY 4.0; no imagery pixels are included); the ute from "Toyota Land Cruiser J79 Pickup Off- Road" by DR1KING100K
(CC BY 4.0); built with raylib (zlib licence). Bush Drift grew out of an earlier private drifting project;
nothing from any other game is in this repository.
