#!/usr/bin/env python3
"""Step 3: NSW Spatial Services aerial imagery (public/NSW_Imagery MapServer) for the route corridor.

The service is a cached (tile-only) Web-Mercator MapServer; `export` is refused. The best cached level
here is z18 (0.597 Web-Mercator m/px = ~0.48 m/px on the ground at -36.7 deg); z19+ return 404 here.
Fetches every z18 tile touching the corridor (route +-100 m) or within 300 m of the hut, caches each under data/raw/imagery/z18/,
and writes a mosaic data/imagery_z18.png (+ data/imagery_z18.json georeference; untouched tiles are black).
Also saves the service metadata and the imagery footprint at the route to data/raw/imagery/ (licence record).

Licence (checked 19 Oct 2026): service copyrightText '(c) Department of Customer Service 2020'; footprint
at the route = block BOMBALA, captured 2011-01-30, RGB. Spatial Services copyright page: material is
CC BY 4.0, attribution '(c) State of New South Wales', except third-party material; the service
description credits external imagery (c) AAM 2011/2012, Jacobs Group Ausimage, LANDSAT. Fine for private use."""
import io
import json
import math
import os
import sys

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RAW, DATA, HUT_PIN, cached, http_get, merc, load_json

SVC = 'https://maps.six.nsw.gov.au/arcgis/rest/services/public/NSW_Imagery/MapServer'
Z = 18
ORIGIN = 20037508.342787
RES = 2 * ORIGIN / 256 / 2 ** Z
CORRIDOR = 100.0
IMG = os.path.join(RAW, 'imagery')


def tile_bounds(tx, ty):
    x0 = -ORIGIN + tx * 256 * RES; y1 = ORIGIN - ty * 256 * RES
    return x0, y1 - 256 * RES, x0 + 256 * RES, y1


def main():
    cached(os.path.join(IMG, 'service.json'), http_get(SVC, {'f': 'json'}))
    route = load_json(os.path.join(DATA, 'route_osm.json'))
    ll = np.array(route['latlon'])
    mx, my = merc(ll[:, 0], ll[:, 1])
    k = 1 / math.cos(math.radians(ll[:, 0].mean()))
    # densify the line, find tiles within CORRIDOR (+ tile diagonal) of it
    pts = []
    for a, b in zip(np.c_[mx, my][:-1], np.c_[mx, my][1:]):
        n = max(1, int(np.hypot(*(b - a)) / 5))
        pts += [a + (b - a) * t / n for t in range(n)]
    pts.append(np.array([mx[-1], my[-1]]))
    tree = cKDTree(np.array(pts))
    m = CORRIDOR * k + 20
    tx0 = int((mx.min() - m + ORIGIN) / (256 * RES)); tx1 = int((mx.max() + m + ORIGIN) / (256 * RES))
    ty0 = int((ORIGIN - (my.max() + m)) / (256 * RES)); ty1 = int((ORIGIN - (my.min() - m)) / (256 * RES))
    need = []
    half = 128 * RES * math.sqrt(2)
    hx, hy = merc(*HUT_PIN)   # plus a wider look around the hut clearing (for the open-ground radius)
    HUT_R = 300.0
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            b = tile_bounds(tx, ty)
            d, _ = tree.query([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2])
            dh = math.hypot((b[0] + b[2]) / 2 - hx, (b[1] + b[3]) / 2 - hy)
            if d <= CORRIDOR * k + half or dh <= HUT_R * k + half:
                need.append((tx, ty))
    print(f'z{Z}: {RES:.3f} merc m/px (~{RES / k:.3f} m ground); {len(need)} tiles of {(tx1 - tx0 + 1) * (ty1 - ty0 + 1)} in bbox')
    W = (tx1 - tx0 + 1) * 256; H = (ty1 - ty0 + 1) * 256
    mosaic = np.zeros((H, W, 3), np.uint8)
    bad = 0
    for i, (tx, ty) in enumerate(need):
        path = os.path.join(IMG, f'z{Z}', f'{ty}_{tx}.jpg')
        try:
            data = cached(path, http_get(f'{SVC}/tile/{Z}/{ty}/{tx}'), binary=True)
            im = Image.open(io.BytesIO(data)).convert('RGB')
        except Exception as e:  # noqa
            bad += 1; print('  tile fail', tx, ty, e)
            if os.path.exists(path):
                os.remove(path)
            continue
        mosaic[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = np.array(im)
        if i % 50 == 0:
            print(f'  {i}/{len(need)}', flush=True)
    Image.fromarray(mosaic).save(os.path.join(DATA, 'imagery_z18.png'), optimize=False, compress_level=3)
    x0, _, _, y1 = tile_bounds(tx0, ty0)
    json.dump({'z': Z, 'res': RES, 'x0': x0, 'y1': y1, 'width': W, 'height': H, 'crs': 'EPSG:3857',
               'note': 'pixel (r,c) top-left corner at (x0 + c*res, y1 - r*res)',
               'source': 'NSW Spatial Services public/NSW_Imagery (block BOMBALA 2011), CC BY 4.0 (c) State of NSW'},
              open(os.path.join(DATA, 'imagery_z18.json'), 'w'), indent=1)
    print(f'mosaic {W}x{H}, {len(need) - bad} tiles ok, {bad} failed')


if __name__ == '__main__':
    main()
