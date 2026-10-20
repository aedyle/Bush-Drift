# Map-data pipeline (hut -> bridge)

Builds the real route (Alexanders Hut -> Tantawangalo Creek bridge, South East Forest NP) for the game.
Everything runs from the repo root with the venv's Python. `scripts/build_all.sh` runs all steps in order
(and creates `.venv` from `requirements.txt` if missing). Downloads are cached under `data/raw/` and never
re-fetched while the cache file exists; delete a file to refresh it. `data/` is gitignored.

| # | Command | Does | Writes |
|---|---|---|---|
| 1 | `.venv/bin/python scripts/fetch_route.py` | Overpass: Robinsons Track (way 533120940, reversed, hut -> junction), Cattlemens Track (98264701), bridge (444217386), Solomons Creek ford (node 4417047980), hut/campground features within 150 m. Joins hut -> far end of bridge. | `data/raw/osm-*.json`, `data/route_osm.json` |
| 2 | `.venv/bin/python scripts/fetch_dem.py` | NSW 5 m DEM ImageServer `exportImage` (TIFF, F32, native EPSG:3857 grid, tiled), masked to 110 m of the route; prints validation heights. Falls back to `getSamples` if export fails. | `data/raw/dem/`, `data/dem_corridor.npz` |
| 3 | `.venv/bin/python scripts/fetch_imagery.py` | NSW_Imagery MapServer tiles at z18 (~0.48 m/px) for route +-100 m and 300 m round the hut; mosaic. | `data/raw/imagery/`, `data/imagery_z18.{png,json}` |
| 4 | `.venv/bin/python scripts/refine_centreline.py` | Moves the coarse OSM line onto the visible track (straightened-strip road detector + Viterbi + null-calibrated confidence; keeps OSM where unsure). Lane width, start pad. | `data/route_refined.json`; diagnostics over the imagery in `shots/route/` (gitignored, never published) |
| 5 | `.venv/bin/python scripts/build_route.py` | Local metric frame (origin = start-pad centre), smooth spline + 10 m min-radius kink limiter, 2 m resample, smoothed road elevation, terrain + canopy grids. | `data/built/route.csv`, `meta.txt`, `terrain.bin`, `canopy.bin` (+ `canopy_preview.png`) |
| 6 | `.venv/bin/python scripts/route_stats.py` | Length, climb/descent, steepest 100 m, bends, min radius vs the project file. | `docs/route/stats.md`, `profile.png`, `topdown.png` |

Shared code: `common.py` (paths, polite cached HTTP with a descriptive User-Agent and ~0.7 s spacing, projections),
`imagery.py` (mosaic sampling).

## Output formats (`data/built/`)

Frame: x = east m, y = north m, origin = start-pad centre (`origin_lat/lon` in meta). Projection: local
ellipsoidal equirectangular about the origin (`common.LocalTM`); worst distance error vs WGS84 geodesics over
the route is ~0.01 %.

- `route.csv`: header then `s,x,y,elev,elev_dem,heading,curv` every 2.0 m (last row = far end of the bridge).
  heading = atan2(dy,dx) rad (0 = east, CCW +, wrapped to +-pi); curv 1/m, + = left.
- `meta.txt`: one `key value...` per line (`lane_width_range lo hi` has two values; start/pad/hut/bridge values
  are one key per line: `start_x`, `start_y`, `start_heading`, `pad_x`, ...).
- `terrain.bin`: little-endian. `BDT1`, int32 nx, int32 ny, float32 x0, y0, cell (5.0), then float32 arrays
  [ny][nx] row-major (row j at y0 + j*cell, column i at x0 + i*cell): `dem` (raw DEM; NaN where unknown, never
  NaN within 80 m of the route or 60 m of the pad), `near_s` (route s of the nearest centreline point, NaN beyond
  80 m), `lat` (signed distance to it, + = left of travel).
- `canopy.bin`: `BDC1` + same header, then uint8 [ny][nx]: 0 = open ground, 255 = tree canopy (imagery
  darkness + texture classifier; 255 where there is no imagery). The road under the canopy reads as canopy:
  clear the road corridor in-game.

Licences: OSM data (c) OpenStreetMap contributors, ODbL. DEM and imagery (c) State of NSW (Spatial Services),
CC BY 4.0; the imagery service also credits third-party captures (AAM, Jacobs/Ausimage, Landsat).
