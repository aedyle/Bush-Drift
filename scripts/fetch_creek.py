#!/usr/bin/env python3
"""Build C, step 7: Solomons Creek (OSM way 444217399) from near Alexanders Hut down to the ford on Cattlemens Track,
for the Explorer mode's secret way -> data/creek_osm.json.

The way is fetched once from Overpass (cached in data/raw/osm-creek.json) with its node geometry, projected into the
game's frame (data/built/meta.txt: origin_lat/lon, common.LocalTM) and cut at the ford node (4417047980, the same node
fetch_route.py uses for the road's ford). Upstream it is kept to 400 m past the point nearest the hut."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RAW, DATA, BUILT, LocalTM, cached, http_get, save_json

WAY = 444217399
FORD_NODE = 4417047980
OVERPASS = 'https://overpass-api.de/api/interpreter'


def meta():
    out = {}
    for line in open(os.path.join(BUILT, 'meta.txt')):
        p = line.split()
        if len(p) >= 2:
            out[p[0]] = p[1]
    return out


def main():
    q = f'[out:json][timeout:60];way({WAY});out geom;'
    js = json.loads(cached(os.path.join(RAW, 'osm-creek.json'), http_get(OVERPASS, {'data': q}, method='POST')))
    way = js['elements'][0]
    nodes, geom = way['nodes'], way['geometry']
    m = meta()
    tm = LocalTM(float(m['origin_lat']), float(m['origin_lon']))
    lat = np.array([g['lat'] for g in geom]); lon = np.array([g['lon'] for g in geom])
    x, y = tm.fwd(lat, lon)
    hut = np.array([float(m['hut_x']), float(m['hut_y'])])
    if FORD_NODE not in nodes:
        raise SystemExit('the ford node is not on the creek way')
    kf = nodes.index(FORD_NODE)
    d = np.hypot(x - hut[0], y - hut[1])
    kh = int(np.argmin(d))
    print(f'creek way {WAY}: {len(nodes)} nodes; nearest the hut node {kh} at {d[kh]:.0f} m; ford node {kf}')
    step = 1 if kf > kh else -1
    seg = np.c_[x, y][kh:kf + step:step] if step > 0 else np.c_[x, y][kh:kf - 1:-1]
    # upstream of the nearest point: up to 400 m more, so the corridor reaches past the way in
    up = np.c_[x, y][kh - step::-step] if 0 <= kh - step < len(x) else np.zeros((0, 2))
    acc, keep = 0.0, []
    prev = seg[0]
    for p in up:
        acc += float(np.hypot(*(p - prev))); prev = p
        if acc > 400.0:
            break
        keep.append(p)
    line = np.r_[np.array(keep[::-1]).reshape(-1, 2), seg]
    L = float(np.sum(np.hypot(*np.diff(line, axis=0).T)))
    dl = float(np.sum(np.hypot(*np.diff(seg, axis=0).T)))
    print(f'kept {len(line)} points, {L:.0f} m ({dl:.0f} m from the point nearest the hut to the ford; straight line '
          f'{np.hypot(*(seg[-1] - seg[0])):.0f} m)')
    save_json(os.path.join(DATA, 'creek_osm.json'), {'way': WAY, 'ford_node': FORD_NODE, 'xy': line.tolist(),
                                                    'hut_index': len(keep), 'licence': 'OSM (c) OpenStreetMap contributors, ODbL'})


if __name__ == '__main__':
    main()
