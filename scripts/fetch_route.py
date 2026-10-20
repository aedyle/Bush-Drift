#!/usr/bin/env python3
"""Step 1: pull the route ways from OSM (Overpass), join them hut -> bridge, save data/route_osm.json.

Robinsons Track (way 533120940) reversed: hut end -> junction with Cattlemens Track,
Cattlemens Track (way 98264701): junction -> bridge, bridge (way 444217386),
Solomons Creek ford (node 4417047980), plus hut/campground features within 150 m of the hut pin.
Raw Overpass responses are cached in data/raw/ (delete them to re-fetch)."""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RAW, DATA, HUT_PIN, JUNCTION, BRIDGE_PIN, cached, http_get, haversine, save_json

OVERPASS = 'https://overpass-api.de/api/interpreter'
W_ROBINSONS, W_CATTLEMENS, W_BRIDGE, N_FORD = 533120940, 98264701, 444217386, 4417047980


def overpass(name, q):
    path = os.path.join(RAW, f'osm-{name}.json')
    txt = cached(path, http_get(OVERPASS, {'data': q}, method='POST', timeout=180))
    return json.loads(txt)


def way_coords(el):
    return [(g['lat'], g['lon']) for g in el['geometry']], el['nodes']


def nearest_idx(coords, pt):
    c = np.array(coords)
    d = haversine(c[:, 0], c[:, 1], pt[0], pt[1])
    return int(np.argmin(d)), float(np.min(d))


def seglen(coords):
    c = np.array(coords)
    return float(np.sum(haversine(c[:-1, 0], c[:-1, 1], c[1:, 0], c[1:, 1])))


def main():
    ways = overpass('route-ways', f"""[out:json][timeout:60];
(way({W_ROBINSONS});way({W_CATTLEMENS});way({W_BRIDGE}););out body geom;
node({N_FORD});out;""")
    lat, lon = HUT_PIN
    ctx = overpass('hut-context', f"""[out:json][timeout:60];
(nwr(around:150,{lat},{lon})[amenity];nwr(around:150,{lat},{lon})[tourism];
 nwr(around:150,{lat},{lon})[building];nwr(around:150,{lat},{lon})[leisure];
 way(around:150,{lat},{lon})[highway];);out body geom;""")

    els = {(e['type'], e['id']): e for e in ways['elements']}
    rob, rob_nodes = way_coords(els[('way', W_ROBINSONS)])
    cat, cat_nodes = way_coords(els[('way', W_CATTLEMENS)])
    brg, brg_nodes = way_coords(els[('way', W_BRIDGE)])
    ford = els[('node', N_FORD)]
    print(f'Robinsons: {len(rob)} nodes {seglen(rob):.0f} m, ends {rob[0]} .. {rob[-1]}')
    print(f'Cattlemens: {len(cat)} nodes {seglen(cat):.0f} m, ends {cat[0]} .. {cat[-1]}')
    print(f'Bridge: {len(brg)} nodes {seglen(brg):.0f} m, ends {brg[0]} .. {brg[-1]}')

    # --- Robinsons: orient hut end -> junction, trim at the junction
    ij, dj = nearest_idx(rob, JUNCTION)
    ih, dh = nearest_idx(rob, HUT_PIN)
    print(f'Robinsons: junction at node idx {ij} ({dj:.1f} m off), nearest-to-hut idx {ih} ({dh:.1f} m)')
    # hut end is the way end farther from the junction index
    if ij > len(rob) // 2:      # junction near the tail -> hut end is head; keep head..junction
        r = rob[:ij + 1]; rnodes = rob_nodes[:ij + 1]
    else:                       # junction near the head -> reverse, keep tail..junction
        r = rob[ij:][::-1]; rnodes = rob_nodes[ij:][::-1]
    print(f'Robinsons kept: {len(r)} nodes, {seglen(r):.0f} m, start {r[0]}')

    # --- Cattlemens: orient junction -> bridge
    jn = rnodes[-1]
    if jn in cat_nodes:
        k = cat_nodes.index(jn)
    else:
        k, d = nearest_idx(cat, r[-1]); print(f'  junction node not shared; nearest {d:.1f} m')
    ib, db = nearest_idx(cat, BRIDGE_PIN)
    shared_b = [n for n in brg_nodes if n in cat_nodes]
    print(f'Cattlemens: junction idx {k}, bridge-pin idx {ib} ({db:.1f} m off), shared nodes with bridge: {shared_b}')
    if ib >= k:
        c = cat[k:]; cnodes = cat_nodes[k:]
    else:
        c = cat[:k + 1][::-1]; cnodes = cat_nodes[:k + 1][::-1]

    # --- Bridge: the bridge way may be a separate segment after Cattlemens, or Cattlemens may cross it.
    pts = list(r); nodes = list(rnodes)
    pts += c[1:]; nodes += cnodes[1:]
    if nodes[-1] in brg_nodes:
        # Cattlemens ends at a bridge node: continue along the bridge to its far end
        j = brg_nodes.index(nodes[-1])
        b = brg[j:] if j == 0 else brg[:j + 1][::-1]
        bn = brg_nodes[j:] if j == 0 else brg_nodes[:j + 1][::-1]
        bridge_start = len(pts) - 1
        pts += b[1:]; nodes += bn[1:]
        bridge_end = len(pts) - 1
    elif brg_nodes[0] in nodes and brg_nodes[-1] in nodes:
        a, z = nodes.index(brg_nodes[0]), nodes.index(brg_nodes[-1])
        bridge_start, bridge_end = min(a, z), max(a, z)
        # route continues past the bridge? trim just after the bridge far end
        pts = pts[:bridge_end + 1]; nodes = nodes[:bridge_end + 1]
    else:
        raise SystemExit('cannot place bridge way relative to Cattlemens Track')

    P = np.array(pts)
    seg = haversine(P[:-1, 0], P[:-1, 1], P[1:, 0], P[1:, 1])
    s = np.concatenate([[0], np.cumsum(seg)])
    fi = nodes.index(N_FORD) if N_FORD in nodes else nearest_idx(pts, (ford['lat'], ford['lon']))[0]
    print(f'Joined: {len(pts)} nodes, length {s[-1]:.1f} m (expect ~7800)')
    print(f'  node spacing median {np.median(seg):.1f} m, max {seg.max():.1f} m')
    print(f'  ford node idx {fi} at s={s[fi]:.1f} m (on-line: {N_FORD in nodes})')
    print(f'  bridge idx {bridge_start}..{bridge_end}, s={s[bridge_start]:.1f}..{s[bridge_end]:.1f} m')
    print(f'  start {pts[0]} ({haversine(pts[0][0], pts[0][1], *HUT_PIN):.0f} m from hut pin), '
          f'end {pts[-1]} ({haversine(pts[-1][0], pts[-1][1], *BRIDGE_PIN):.0f} m from bridge pin)')

    context = []
    for e in ctx['elements']:
        item = {'type': e['type'], 'id': e['id'], 'tags': e.get('tags', {})}
        if e['type'] == 'node':
            item['lat'], item['lon'] = e['lat'], e['lon']
        elif 'geometry' in e:
            item['geometry'] = [(g['lat'], g['lon']) for g in e['geometry'] if g]
        elif 'members' in e:
            item['members'] = len(e['members'])
        context.append(item)
        print(f'  context {e["type"]}/{e["id"]}: {item["tags"]}')

    save_json(os.path.join(DATA, 'route_osm.json'), {
        'source': 'OpenStreetMap contributors (ODbL), via overpass-api.de',
        'ways': {'robinsons': W_ROBINSONS, 'cattlemens': W_CATTLEMENS, 'bridge': W_BRIDGE},
        'latlon': pts, 'node_ids': nodes, 's': s.tolist(), 'length_m': float(s[-1]),
        'ford': {'node': N_FORD, 'lat': ford['lat'], 'lon': ford['lon'], 'idx': fi, 's': float(s[fi]),
                 'tags': ford.get('tags', {})},
        'bridge': {'idx0': bridge_start, 'idx1': bridge_end, 's0': float(s[bridge_start]), 's1': float(s[bridge_end]),
                   'tags': els[('way', W_BRIDGE)].get('tags', {})},
        'tags': {'robinsons': els[('way', W_ROBINSONS)].get('tags', {}), 'cattlemens': els[('way', W_CATTLEMENS)].get('tags', {})},
        'hut_context': context,
    })
    print('wrote data/route_osm.json')


if __name__ == '__main__':
    main()
