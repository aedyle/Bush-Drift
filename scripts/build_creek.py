#!/usr/bin/env python3
"""Build C, step 8: the creek way's own strip of world -> data/built/creek.csv and data/built/creek_dem.bin.

1. The OSM line (data/creek_osm.json from scripts/fetch_creek.py), resampled every 2 m.
2. NSW 5 m DEM tiles over the line's box (+150 m): exportImage as in fetch_dem.py, cached in data/raw/dem/creek-*.tif.
3. The line moved onto the valley floor: OSM's creek lines are traced roughly, so a dynamic-programming pass over
   sideways offsets (+-30 m, 1 m steps, at most 1 m of change per 2 m along) finds the lowest smooth path, then it is
   smoothed again. The last 80 m are eased onto the road's own ford (data/built: route.csv, meta.txt ford_s), so the
   creek way meets the road exactly there.
4. The bed: the DEM along the moved line (the lowest within 2 m either side), a running minimum downstream (water runs
   downhill), lightly smoothed.
5. The DEM on the game's own 5 m grid lattice (meta grid_x0/grid_y0/grid_cell; rows past the top as route_load grows
   them), within CORRIDOR m of the line, NaN elsewhere.

Outputs (same frame as the road):
- creek.csv: header, then s,x,y,bed,dem,heading,curv every 2 m (s = 0 upstream; heading/curv as route.csv).
- creek_dem.bin: BDT1, int32 nx, ny, float32 x0, y0, cell (the main grid's lattice), then float32 [ny][nx] raw DEM
  (metres above sea level; NaN outside the corridor). The game uses it where the road's own DEM has nothing.
- creek_meta.txt: length, the ford, where the line comes nearest the hut, the steepest drops.
DEM (c) State of NSW (Spatial Services), CC BY 4.0. Creek line (c) OpenStreetMap contributors, ODbL."""
import csv
import io
import os
import struct
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d, map_coordinates

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RAW, DATA, BUILT, LocalTM, cached, http_get, load_json, merc

SVC = 'https://maps.six.nsw.gov.au/arcgis/rest/services/public/NSW_5M_Elevation/ImageServer'
X0_NATIVE = 15696048.203000002
Y1_NATIVE = -3248629.9388000015
PIX = 5.0
TILE = 1000
MARGIN = 150.0
CORRIDOR = 90.0
STEP = 2.0
BEND_FROM, BEND_IN, BEND_HOLD, BEND_BACK, BEND_M = 95.0, 55.0, 170.0, 300.0, 14.0   # build D: the bend past the way in


def meta():
    out = {}
    for line in open(os.path.join(BUILT, 'meta.txt')):
        p = line.split()
        if len(p) >= 2:
            out[p[0]] = p[1]
    return out


def export_tile(xmin, ymin, w, h):
    params = {'bbox': f'{xmin},{ymin},{xmin + w * PIX},{ymin + h * PIX}', 'bboxSR': 3857, 'imageSR': 3857,
              'size': f'{w},{h}', 'format': 'tiff', 'pixelType': 'F32', 'noDataInterpretation': 'esriNoDataMatchAny',
              'interpolation': 'RSP_NearestNeighbor', 'compression': 'None', 'f': 'image'}
    path = os.path.join(RAW, 'dem', f'creek-{int(xmin)}-{int(ymin)}-{w}x{h}.tif')
    data = cached(path, http_get(SVC + '/exportImage', params), binary=True)
    if data[:1] == b'{' or data[:5] == b'<html':
        os.remove(path)
        raise RuntimeError('exportImage returned non-image: ' + data[:300].decode('utf-8', 'replace'))
    a = np.array(Image.open(io.BytesIO(data)), dtype=np.float32)
    if a.shape != (h, w):
        raise RuntimeError(f'unexpected tile shape {a.shape}')
    return a


def resample(xy, step):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    s = np.arange(0, d[-1], step)
    return np.c_[np.interp(s, d, xy[:, 0]), np.interp(s, d, xy[:, 1])]


def main():
    m = meta()
    tm = LocalTM(float(m['origin_lat']), float(m['origin_lon']))
    cr = load_json(os.path.join(DATA, 'creek_osm.json'))
    line = resample(np.array(cr['xy']), STEP)
    # ---- the DEM over the line's box, in web mercator like fetch_dem.py
    lat, lon = tm.inv(line[:, 0], line[:, 1])
    mx, my = merc(lat, lon)
    k = 1.0 / np.cos(np.radians(lat.mean()))
    mm = (MARGIN + CORRIDOR) * k
    x0 = X0_NATIVE + np.floor((mx.min() - mm - X0_NATIVE) / PIX) * PIX
    x1 = X0_NATIVE + np.ceil((mx.max() + mm - X0_NATIVE) / PIX) * PIX
    y1 = Y1_NATIVE - np.floor((Y1_NATIVE - (my.max() + mm)) / PIX) * PIX
    y0 = Y1_NATIVE - np.ceil((Y1_NATIVE - (my.min() - mm)) / PIX) * PIX
    Wd = int(round((x1 - x0) / PIX)); Hd = int(round((y1 - y0) / PIX))
    print(f'creek DEM {Wd} x {Hd} px of {PIX} merc-m')
    dem = np.full((Hd, Wd), np.nan, np.float32)
    for ty in range(0, Hd, TILE):
        for tx in range(0, Wd, TILE):
            w = min(TILE, Wd - tx); h = min(TILE, Hd - ty)
            a = export_tile(x0 + tx * PIX, y1 - ty * PIX - h * PIX, w, h)
            dem[ty:ty + h, tx:tx + w] = a
    dem[(dem < -100) | (dem > 3000)] = np.nan
    fill = np.nanmean(dem)

    def dem_at(x, y):
        la, lo = tm.inv(np.asarray(x, float), np.asarray(y, float))
        X, Y = merc(la, lo)
        c = (X - x0) / PIX - 0.5; r = (y1 - Y) / PIX - 0.5
        return map_coordinates(np.nan_to_num(dem, nan=fill), [np.ravel(r), np.ravel(c)], order=1).reshape(np.shape(x))

    # ---- onto the valley floor
    t = np.gradient(line, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]
    nrm = np.c_[-t[:, 1], t[:, 0]]
    offs = np.arange(-30, 31, 1.0)
    H = dem_at(line[:, 0][:, None] + nrm[:, 0][:, None] * offs[None, :], line[:, 1][:, None] + nrm[:, 1][:, None] * offs[None, :])
    n, K = H.shape
    cost = H[0] + 0.02 * offs ** 2
    back = np.zeros((n, K), int)
    for i in range(1, n):
        best = np.full(K, np.inf); arg = np.zeros(K, int)
        for d in (-1, 0, 1):
            sh = np.roll(cost, d)
            if d == 1: sh[0] = np.inf
            if d == -1: sh[-1] = np.inf
            pen = sh + 0.15 * abs(d)
            better = pen < best
            best[better] = pen[better]; arg[better] = (np.arange(K) - d)[better]
        cost = best + H[i] - H[i].min() + 0.002 * offs ** 2
        back[i] = arg
    path = np.zeros(n, int); path[-1] = int(np.argmin(cost))
    for i in range(n - 1, 0, -1):
        path[i - 1] = back[i][path[i]]
    off = gaussian_filter1d(offs[path].astype(float), 6.0, mode='nearest')
    moved = line + nrm * off[:, None]
    print(f'moved onto the valley floor: mean {np.abs(off).mean():.1f} m, most {np.abs(off).max():.1f} m')
    # ---- onto the road's ford at the end
    rows = list(csv.DictReader(open(os.path.join(BUILT, 'route.csv'))))
    rs = np.array([float(r['s']) for r in rows]); rx = np.array([float(r['x']) for r in rows]); ry = np.array([float(r['y']) for r in rows])
    fs = float(m['ford_s']); fx, fy = np.interp(fs, rs, rx), np.interp(fs, rs, ry)
    gap = np.array([fx, fy]) - moved[-1]
    print(f'the line ends {np.hypot(*gap):.1f} m from the road\'s ford')
    ease = np.clip((np.arange(n) - (n - 40)) / 40.0, 0, 1)
    ease = ease * ease * (3 - 2 * ease)
    moved = moved + gap[None, :] * ease[:, None]
    moved = resample(gaussian_filter1d(moved, 2.0, axis=0, mode='nearest'), STEP)
    n = len(moved)
    # ---- build D: a bend just past the way in, so the track can't be seen down the creek from the clearing. Where the
    # creek leaves the clearing (about BEND_FROM m past its nearest point to the hut) it swings BEND_M m to the left, round
    # behind the thicket, and comes back to its own line by BEND_BACK m. The bed is cut wherever the line goes.
    hut0 = np.array([float(m['hut_x']), float(m['hut_y'])])
    kh0 = int(np.argmin(np.hypot(*(moved - hut0).T)))
    t = np.gradient(moved, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]
    nrm = np.c_[-t[:, 1], t[:, 0]]
    ss = (np.arange(n) - kh0) * STEP
    up = np.clip((ss - BEND_FROM) / BEND_IN, 0, 1); up = up * up * (3 - 2 * up)
    down = np.clip((ss - BEND_HOLD) / (BEND_BACK - BEND_HOLD), 0, 1); down = down * down * (3 - 2 * down)
    moved = moved + nrm * (BEND_M * up * (1 - down))[:, None]   # not resampled: every point keeps its place in the line (s = 2 m a
                                                                # point), so nothing past the bend moves; the bend's points sit ~2.05 m apart
    print(f'the bend past the way in: {BEND_M:.0f} m to the left from {BEND_FROM:.0f} m past the hut\'s nearest point, back by {BEND_BACK:.0f} m')
    # ---- the bed: the lowest DEM within 2 m, running downhill
    t = np.gradient(moved, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]
    nrm = np.c_[-t[:, 1], t[:, 0]]
    lows = np.min([dem_at(moved[:, 0] + nrm[:, 0] * o, moved[:, 1] + nrm[:, 1] * o) for o in (-2, -1, 0, 1, 2)], axis=0)
    centre = dem_at(moved[:, 0], moved[:, 1])
    bed = np.minimum.accumulate(lows)
    bed = gaussian_filter1d(bed, 2.0, mode='nearest')
    bed = np.minimum.accumulate(bed)
    s = np.arange(n) * STEP
    head = np.arctan2(t[:, 1], t[:, 0])
    dh = np.diff(np.unwrap(head)); curv = np.r_[dh, dh[-1]] / STEP
    with open(os.path.join(BUILT, 'creek.csv'), 'w') as f:
        f.write('s,x,y,bed,dem,heading,curv\n')
        for i in range(n):
            f.write(f'{s[i]:.1f},{moved[i, 0]:.2f},{moved[i, 1]:.2f},{bed[i]:.2f},{centre[i]:.2f},{head[i]:.5f},{curv[i]:.5f}\n')
    # ---- the DEM on the game's grid lattice, within CORRIDOR of the line
    gx0, gy0, cell = float(m['grid_x0']), float(m['grid_y0']), float(m['grid_cell'])
    lo_x, hi_x = moved[:, 0].min() - CORRIDOR, moved[:, 0].max() + CORRIDOR
    lo_y, hi_y = moved[:, 1].min() - CORRIDOR, moved[:, 1].max() + CORRIDOR
    i0 = int(np.floor((lo_x - gx0) / cell)); i1 = int(np.ceil((hi_x - gx0) / cell))
    j0 = int(np.floor((lo_y - gy0) / cell)); j1 = int(np.ceil((hi_y - gy0) / cell))
    nx, ny = i1 - i0 + 1, j1 - j0 + 1
    X = gx0 + (np.arange(i0, i1 + 1)) * cell; Y = gy0 + (np.arange(j0, j1 + 1)) * cell
    GX, GY = np.meshgrid(X, Y)
    from scipy.spatial import cKDTree
    d, _ = cKDTree(moved).query(np.c_[GX.ravel(), GY.ravel()], distance_upper_bound=CORRIDOR + 10)
    d = d.reshape(ny, nx)
    D = np.full((ny, nx), np.nan, np.float32)
    sel = d <= CORRIDOR
    D[sel] = dem_at(GX[sel], GY[sel])
    with open(os.path.join(BUILT, 'creek_dem.bin'), 'wb') as f:
        f.write(b'BDT1'); f.write(struct.pack('<iifff', nx, ny, gx0 + i0 * cell, gy0 + j0 * cell, cell))
        f.write(D.astype('<f4').tobytes())
    # ---- a few facts
    hut = np.array([float(m['hut_x']), float(m['hut_y'])])
    kh = int(np.argmin(np.hypot(*(moved - hut).T)))
    drops = [(bed[i] - bed[min(i + 5, n - 1)], s[i]) for i in range(0, n - 5)]
    drops.sort(reverse=True)
    with open(os.path.join(BUILT, 'creek_meta.txt'), 'w') as f:
        f.write(f'length {s[-1]:.1f}\nnearest_hut_s {s[kh]:.1f}\nnearest_hut_m {np.hypot(*(moved[kh] - hut)):.1f}\n')
        f.write(f'bed_top {bed[0]:.1f}\nbed_ford {bed[-1]:.1f}\n')
        for k in range(5):
            f.write(f'drop {drops[k][1]:.0f} {drops[k][0]:.2f}\n')
    print(f'creek way: {s[-1]:.0f} m, bed {bed[0]:.0f} m to {bed[-1]:.0f} m above sea level; nearest the hut at s {s[kh]:.0f} '
          f'({np.hypot(*(moved[kh] - hut)):.0f} m); DEM grid {nx} x {ny} ({np.isfinite(D).sum()} cells); steepest 10 m drops: ' +
          ', '.join(f'{dd:.1f} m at s {ss:.0f}' for dd, ss in drops[:5]))


if __name__ == '__main__':
    main()
