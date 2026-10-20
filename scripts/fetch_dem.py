#!/usr/bin/env python3
"""Step 2: NSW Spatial Services 5 m DEM for the route corridor -> data/dem_corridor.npz.

Uses ImageServer exportImage (format=tiff, pixelType=F32) in the service's native CRS (EPSG:3857, 5 m
Web-Mercator pixels, i.e. ~4.0 m on the ground at this latitude), aligned to the native pixel grid so
nothing is resampled. The route bbox (+150 m) is tiled into <=1000 px pieces; each tile is cached in
data/raw/dem/. Cells farther than 110 m from the OSM route (90 m + slack for the refined
line, which moves up to ~16 m) are set to NaN.
Fallback (only if exportImage fails): getSamples on a corridor grid (5 m along, 10 m across, +-90 m)."""
import io
import json
import os
import sys

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RAW, DATA, cached, http_get, merc, merc_inv, load_json

SVC = 'https://maps.six.nsw.gov.au/arcgis/rest/services/public/NSW_5M_Elevation/ImageServer'
X0_NATIVE = 15696048.203000002     # service extent xmin / ymax: pixel grid anchor
Y1_NATIVE = -3248629.9388000015
PIX = 5.0
TILE = 1000
CORRIDOR = 110.0      # ground metres: 90 m + room for the refined line's offset from OSM (max ~16 m)
MARGIN = 150.0


def ground_to_merc(lat):
    return 1.0 / np.cos(np.radians(lat))


def densify(xy, step):
    out = [xy[0]]
    for a, b in zip(xy[:-1], xy[1:]):
        n = max(1, int(np.ceil(np.hypot(*(b - a)) / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def export_tile(xmin, ymin, w, h):
    params = {'bbox': f'{xmin},{ymin},{xmin + w * PIX},{ymin + h * PIX}', 'bboxSR': 3857, 'imageSR': 3857,
              'size': f'{w},{h}', 'format': 'tiff', 'pixelType': 'F32', 'noDataInterpretation': 'esriNoDataMatchAny',
              'interpolation': 'RSP_NearestNeighbor', 'compression': 'None', 'f': 'image'}
    path = os.path.join(RAW, 'dem', f'export-{int(xmin)}-{int(ymin)}-{w}x{h}.tif')
    data = cached(path, http_get(SVC + '/exportImage', params), binary=True)
    if data[:1] == b'{' or data[:5] == b'<html':
        os.remove(path)
        raise RuntimeError('exportImage returned non-image: ' + data[:300].decode('utf-8', 'replace'))
    im = Image.open(io.BytesIO(data))
    a = np.array(im, dtype=np.float32)
    if a.shape != (h, w):
        raise RuntimeError(f'unexpected tile shape {a.shape}')
    return a


def get_samples(pts_merc, tag):
    """getSamples fallback: <=100 points per call."""
    vals = np.full(len(pts_merc), np.nan)
    for i in range(0, len(pts_merc), 100):
        chunk = pts_merc[i:i + 100]
        geom = {'points': [[float(x), float(y)] for x, y in chunk], 'spatialReference': {'wkid': 3857}}
        params = {'geometry': json.dumps(geom), 'geometryType': 'esriGeometryMultipoint', 'returnFirstValueOnly': 'true',
                  'interpolation': 'RSP_BilinearInterpolation', 'f': 'json'}
        path = os.path.join(RAW, 'dem', f'samples-{tag}-{i:06d}.json')
        js = json.loads(cached(path, http_get(SVC + '/getSamples', params)))
        for smp in js.get('samples', []):
            try:
                vals[i + smp['locationId']] = float(smp['value'])
            except (KeyError, ValueError, TypeError):
                pass
    return vals


def main():
    route = load_json(os.path.join(DATA, 'route_osm.json'))
    ll = np.array(route['latlon'])
    mx, my = merc(ll[:, 0], ll[:, 1])
    k = ground_to_merc(ll[:, 0].mean())
    m = MARGIN * k
    # snap bbox to native pixel grid
    x0 = X0_NATIVE + np.floor((mx.min() - m - X0_NATIVE) / PIX) * PIX
    x1 = X0_NATIVE + np.ceil((mx.max() + m - X0_NATIVE) / PIX) * PIX
    y1 = Y1_NATIVE - np.floor((Y1_NATIVE - (my.max() + m)) / PIX) * PIX
    y0 = Y1_NATIVE - np.ceil((Y1_NATIVE - (my.min() - m)) / PIX) * PIX
    W = int(round((x1 - x0) / PIX)); H = int(round((y1 - y0) / PIX))
    print(f'grid {W} x {H} px of {PIX} merc-m (~{PIX / k:.2f} m ground), merc bbox {x0:.1f},{y0:.1f},{x1:.1f},{y1:.1f}')

    dem = np.full((H, W), np.nan, np.float32)   # row 0 = north (top)
    method = 'exportImage'
    try:
        for ty in range(0, H, TILE):
            for tx in range(0, W, TILE):
                w = min(TILE, W - tx); h = min(TILE, H - ty)
                xmin = x0 + tx * PIX; ymax = y1 - ty * PIX
                a = export_tile(xmin, ymax - h * PIX, w, h)
                dem[ty:ty + h, tx:tx + w] = a
                print(f'  tile ({tx},{ty}) {w}x{h}: min {np.nanmin(a):.1f} max {np.nanmax(a):.1f}')
        dem[(dem < -100) | (dem > 3000)] = np.nan   # nodata
    except Exception as e:  # noqa
        print('exportImage failed:', e, '-> falling back to getSamples')
        method = 'getSamples'

    # corridor mask: cells within CORRIDOR (ground m) of the route polyline
    line = densify(np.c_[mx, my], 2.0)
    tree = cKDTree(line)
    cx = x0 + (np.arange(W) + 0.5) * PIX
    cy = y1 - (np.arange(H) + 0.5) * PIX
    gx, gy = np.meshgrid(cx, cy)
    d, _ = tree.query(np.c_[gx.ravel(), gy.ravel()], distance_upper_bound=CORRIDOR * k * 1.5)
    dist = (d / k).reshape(H, W)
    if method == 'getSamples':
        sel = dist <= CORRIDOR
        dem[sel] = get_samples(np.c_[gx[sel], gy[sel]], 'grid')
    dem[dist > CORRIDOR] = np.nan

    np.savez_compressed(os.path.join(DATA, 'dem_corridor.npz'), dem=dem, x0=x0, y1=y1, pix=PIX, method=method,
                        note='EPSG:3857; cell (r,c) centre = (x0+(c+0.5)*pix, y1-(r+0.5)*pix); NaN outside 110 m corridor')
    print(f'corridor cells {np.isfinite(dem).sum()} / {(dist <= CORRIDOR).sum()} within {CORRIDOR} m; method {method}')

    # validation
    def at(lat, lon):
        x, y = merc(lat, lon)
        c = (x - x0) / PIX - 0.5; r = (y1 - y) / PIX - 0.5
        from scipy.ndimage import map_coordinates
        return float(map_coordinates(np.nan_to_num(dem, nan=np.nanmean(dem)), [[r], [c]], order=1)[0])
    s = np.array(route['s'])
    z = np.array([at(a, b) for a, b in ll])
    f = route['ford']; b = route['bridge']
    print('validation (expect hut ~918, plateau ~930-949, ford ~675, bridge ~663):')
    print(f'  route start {z[0]:.1f} m; ford (s={f["s"]:.0f}) {at(f["lat"], f["lon"]):.1f} m; '
          f'bridge {z[b["idx0"]]:.1f}..{z[b["idx1"]]:.1f} m')
    plat = (s > 800) & (s < 4300)
    print(f'  s 0.8-4.3 km: min {z[plat].min():.1f} max {z[plat].max():.1f}; overall max {z.max():.1f} at s={s[z.argmax()]:.0f}')
    for si in range(0, int(s[-1]), 500):
        i = np.argmin(abs(s - si)); print(f'    s={s[i]:6.0f}  z={z[i]:6.1f}')


if __name__ == '__main__':
    main()
