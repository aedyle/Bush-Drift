#!/usr/bin/env python3
"""Step 4: refine the coarse OSM centreline against the NSW aerial imagery -> data/route_refined.json.

Method (two passes):
 1. Reference line = OSM polyline densified to 1 m and Gaussian-smoothed (sigma 5 m, ends fixed).
 2. Straighten the imagery along it: a strip every 1 m along, +-W m across at 0.5 m.
 3. Road feature: bare soil is redder than canopy/grass, and brighter. F = z(R-G) + 0.5 z(lum), z-scored in
    100 m along-track windows; matched filter across (DoG, ~3.5 m wide band) and a 3 m along-track blur.
 4. Viterbi path through the strip: unary = -F, transition = lambda*dL^2 (|dL| <= 0.5 m per 1 m step),
    weak prior pulling toward the reference.
 5. Confidence = how much the path's feature stands above the row median, blurred over 20 m, calibrated
    against the same detector run along parallel null corridors away from the road (weight ramps from
    the null 95th to 99th percentile, so ~5% false-confidence on canopy). Offsets are
    smoothed with confidence-weighted (normalised) Gaussian convolution, sigma 8 m; where confidence is low
    the offset blends back to 0 (= keep OSM). Capped at 25 m. Start (first 8 m) and bridge pinned to OSM.
 6. Second pass around the pass-1 line with W = 12 m.
Also: lane-width measurements at clear spots, and the start pad (bare dirt patch west of the hut).
Diagnostics in shots/route/ (gitignored, they show NSW imagery): overlay-overview.png, overlay-*.png, start-pad.png, refine-strip-*.png."""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import gaussian_filter1d, gaussian_filter, label

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import DATA, DIAG, LocalTM, load_json, save_json, HUT_PIN
from imagery import Mosaic

DS = 1.0      # along-track step (m)
DL = 0.5      # across step (m)
CAP = 25.0


def resample(xy, step):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    S = np.arange(0, d[-1], step)
    S = np.r_[S, d[-1]] if d[-1] - S[-1] > 1e-6 else S
    return S, np.c_[np.interp(S, d, xy[:, 0]), np.interp(S, d, xy[:, 1])]


def smooth_line(xy, sigma_m, step):
    """Gaussian-smooth a uniformly resampled line, keeping the end points (reflect about the ends)."""
    n = int(4 * sigma_m / step) + 1
    a = np.r_[2 * xy[0] - xy[n:0:-1], xy, 2 * xy[-1] - xy[-2:-n - 2:-1]]
    out = np.c_[gaussian_filter1d(a[:, 0], sigma_m / step), gaussian_filter1d(a[:, 1], sigma_m / step)]
    return out[n:n + len(xy)]


def frame(xy):
    t = np.gradient(xy, axis=0)
    t /= np.hypot(t[:, 0], t[:, 1])[:, None]
    nrm = np.c_[-t[:, 1], t[:, 0]]           # left normal
    return t, nrm


class Refiner:
    def __init__(self):
        self.route = load_json(os.path.join(DATA, 'route_osm.json'))
        ll = np.array(self.route['latlon'])
        self.tm = LocalTM(*ll[0])
        X, Y = self.tm.fwd(ll[:, 0], ll[:, 1])
        self.osm = np.c_[X, Y]
        self.mos = Mosaic()

    def img_at(self, xy):
        la, lo = self.tm.inv(xy[..., 0], xy[..., 1])
        return self.mos.sample(la, lo)

    def strip(self, line, half):
        t, nrm = frame(line)
        L = np.arange(-half, half + 1e-9, DL)
        P = line[:, None, :] + nrm[:, None, :] * L[None, :, None]
        return L, self.img_at(P), nrm

    @staticmethod
    def feature(A):
        R, G, B = A[..., 0], A[..., 1], A[..., 2]
        lum = (R + G + B) / 3
        F = np.zeros_like(R)
        for f, w in ((R - G, 1.0), (lum, 0.5)):
            # local z-score over ~100 m along-track windows
            m = gaussian_filter1d(f.mean(1), 50 / DS)[:, None]
            v = gaussian_filter1d(((f - m) ** 2).mean(1), 50 / DS)[:, None]
            F += w * (f - m) / np.sqrt(v + 1e-6)
        F = gaussian_filter1d(F, 1.2 / DL, axis=1) - gaussian_filter1d(F, 4.0 / DL, axis=1)
        F = gaussian_filter1d(F, 3.0 / DS, axis=0)
        black = (A.sum(-1) < 5)      # outside fetched tiles
        F[black] = -1.0
        return F

    @staticmethod
    def viterbi(F, L, lam=2.0, mu=0.01, maxstep=1):
        n, m = F.shape
        unary = -F + mu * np.abs(L)[None, :]
        cost = unary[0].copy()
        back = np.zeros((n, m), np.int16)
        idx = np.arange(m)
        for i in range(1, n):
            best = np.full(m, np.inf); arg = np.zeros(m, np.int16)
            for d in range(-maxstep, maxstep + 1):
                src = idx - d
                ok = (src >= 0) & (src < m)
                c = np.full(m, np.inf)
                c[ok] = cost[src[ok]] + lam * (d * DL) ** 2
                better = c < best
                best[better] = c[better]; arg[better] = src[better]
            cost = best + unary[i]; back[i] = arg
        path = np.zeros(n, int); path[-1] = int(np.argmin(cost))
        for i in range(n - 1, 0, -1):
            path[i - 1] = back[i, path[i]]
        return path

    def path_conf(self, line, half):
        L, A, nrm = self.strip(line, half)
        F = self.feature(A)
        path = self.viterbi(F, L)
        contrast = F[np.arange(len(path)), path] - np.median(F, axis=1)
        inside = A.sum(-1).min(1) > 5
        return L, A, nrm, F, path, gaussian_filter1d(contrast, 10 / DS), inside

    def run_pass(self, line, half, tag, null_side, sigma=8.0):
        L, A, nrm, F, path, conf, _ = self.path_conf(line, half)
        off = L[path]
        # null calibration: the same detector run along parallel corridors +-null_side m away from the road
        nulls = []
        for side in (-null_side, null_side):
            *_, cn, inside = self.path_conf(line + nrm * side, half)
            nulls.append(cn[inside])
        cn = np.concatenate(nulls)
        t95, t99 = np.percentile(cn, 95), np.percentile(cn, 99)
        print(f'  pass {tag}: null confidence p95 {t95:.2f} p99 {t99:.2f}; on-route median {np.median(conf):.2f}')
        w = np.clip((conf - t95) / (t99 - t95), 0, 1)
        # pin the ends
        w[:int(8 / DS)] = 0; w[-int(15 / DS):] = 0
        num = gaussian_filter1d(w * off, sigma / DS); den = gaussian_filter1d(w, sigma / DS)
        blend = np.clip(den / 0.5, 0, 1)
        off_s = np.where(den > 1e-6, num / np.maximum(den, 1e-6), 0) * blend
        # ease in/out: the line starts at the OSM start node (on the pad) and ends on the OSM bridge
        n = len(off_s); sidx = np.arange(n) * DS
        ramp = np.clip(sidx / 25.0, 0, 1) * np.clip((sidx[-1] - sidx - 12.0) / 25.0, 0, 1)
        off_s = np.clip(off_s * (3 * ramp ** 2 - 2 * ramp ** 3), -CAP, CAP)
        new = line + nrm * off_s[:, None]
        print(f'  pass {tag}: half {half} m; raw path offset median {np.median(abs(off)):.1f}; '
              f'confident {np.mean(w > 0.5) * 100:.0f}% of length; applied offset median {np.median(abs(off_s)):.2f} '
              f'p90 {np.percentile(abs(off_s), 90):.1f} max {abs(off_s).max():.1f} m')
        self.save_strip_png(A, F, L, off, off_s, w, tag)
        return new, dict(off_raw=off, off=off_s, conf=conf, w=w, F=F, L=L, A=A, t95=float(t95), t99=float(t99))

    def save_strip_png(self, A, F, L, off, off_s, w, tag, seg=700):
        """Straightened imagery strips with raw path (red where confident, magenta otherwise) and final (cyan)."""
        n = len(off); panels = []
        for i0 in range(0, n, seg):
            a = A[i0:i0 + seg]
            lo, hi = np.percentile(a, 1), np.percentile(a, 99.5)
            a = np.clip((a - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
            a = np.transpose(a, (1, 0, 2))[::-1].copy()        # left of travel at top
            h = a.shape[0]
            for j in range(a.shape[1]):
                k = i0 + j
                r = h - 1 - int(round((off[k] - L[0]) / DL))
                if j % 2 == 0:
                    a[np.clip(r, 0, h - 1), j] = [255, 0, 0] if w[k] > 0.5 else [255, 0, 255]
                rr = h - 1 - int(round((off_s[k] - L[0]) / DL))
                a[np.clip(rr, 0, h - 1), j] = [0, 255, 255]
                if j % 10 == 0:
                    a[h // 2, j] = [255, 255, 0]
            if a.shape[1] < seg:
                a = np.pad(a, ((0, 0), (0, seg - a.shape[1]), (0, 0)))
            panels += [a, np.full((4, seg, 3), 255, np.uint8)]
        out = np.concatenate(panels)
        k = max(1, int(np.ceil(out.shape[0] / 1400)))
        rows_per = out.shape[0] // k + 1
        for p in range(k):
            Image.fromarray(out[p * rows_per:(p + 1) * rows_per]).save(os.path.join(DIAG, f'refine-strip-{tag}-{p + 1}.png'))


def fwhm(p, L):
    """Full width at half max of the bump nearest the centre of an across-track profile p(L)."""
    p = p - np.median(np.r_[p[:4], p[-4:]])
    c = len(L) // 2
    pk = c - 6 + int(np.argmax(p[c - 6:c + 7]))
    if p[pk] <= 0:
        return None
    half = p[pk] / 2
    a = pk
    while a > 0 and p[a] > half:
        a -= 1
    b = pk
    while b < len(p) - 1 and p[b] > half:
        b += 1
    if a == 0 or b == len(p) - 1:
        return None
    wa = a + (half - p[a]) / (p[a + 1] - p[a] + 1e-9)
    wb = b - (half - p[b]) / (p[b - 1] - p[b] + 1e-9)
    return L[0] + wa * DL, L[0] + wb * DL


def lane_widths(ref, line, res, clearing_end):
    """Visible dirt width (full width at half max of the across-track brightness bump, averaged over 9 m
    along-track). Spots: every 15 m through the open grass clearing at the hut (clean edges, no shade),
    plus the most confident forest spots (>=250 m apart) where the dirt shows through the canopy."""
    L, A, nrm = ref.strip(line, 10.0)
    R, G, B = A[..., 0], A[..., 1], A[..., 2]
    lum = (R + G + B) / 3
    conf = res['conf']
    spots = [('clearing', i) for i in range(20, int(clearing_end) - 10, 15)]
    taken = []
    for i in np.argsort(-conf):
        if len(taken) >= 8:
            break
        if i < clearing_end + 50 or i > len(line) - 30 or conf[i] < res['t99'] or any(abs(i - t) < 250 for t in taken):
            continue
        taken.append(int(i)); spots.append(('forest', int(i)))
    out = []
    for kind, i in spots:
        sl = slice(i - 4, i + 5)
        for name, f in (('lum', lum), ('R-G', R - G)):
            e = fwhm(f[sl].mean(0), L)
            if e and 1.5 < e[1] - e[0] < 9:
                out.append(dict(s=float(i * DS), kind=kind, feature=name, width=float(e[1] - e[0]),
                                edges=[float(e[0]), float(e[1])], conf=float(conf[i]),
                                xy=line[i].tolist(), nrm=nrm[i].tolist()))
                break
    return out


def lane_width_png(ref, lw):
    """Tiles of imagery around measurement spots with the measured dirt edges marked."""
    mos = ref.mos
    sel = [w for w in lw if w['kind'] == 'clearing'][::3][:4] + [w for w in lw if w['kind'] == 'forest'][:4]
    tiles = []
    for w in sel:
        c = np.array(w['xy']); n = np.array(w['nrm']); t = np.array([n[1], -n[0]])
        la, lo = ref.tm.inv(c[0], c[1]); cc, rr = mos.pix(la, lo)
        half = 25; sc = 6
        box = (int(cc - half), int(rr - half), int(cc + half), int(rr + half))
        im = mos.img.crop(box).resize((2 * half * sc, 2 * half * sc), Image.LANCZOS)
        d = ImageDraw.Draw(im)
        for e in w['edges']:
            p0 = c + n * e - t * 3; p1 = c + n * e + t * 3
            q = []
            for p in (p0, p1):
                la, lo = ref.tm.inv(p[0], p[1]); x, y = mos.pix(la, lo)
                q.append(((x - box[0] + 0.5) * sc, (y - box[1] + 0.5) * sc))
            d.line(q, fill=(255, 0, 0), width=3)
        d.text((6, 6), f"s={w['s']:.0f} {w['kind']} {w['width']:.1f} m", fill=(255, 255, 0))
        tiles.append(np.array(im))
    while len(tiles) % 4:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.concatenate(tiles[k:k + 4], 1) for k in range(0, len(tiles), 4)]
    Image.fromarray(np.concatenate(rows, 0)).save(os.path.join(DIAG, 'lane-width.png'))


def polygon_centroid(P):
    P = np.asarray(P, float)
    if np.allclose(P[0], P[-1]):
        P = P[:-1]
    x, y = P[:, 0], P[:, 1]; x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y; a = cr.sum() / 2
    return np.array([((x + x1) * cr).sum() / (6 * a), ((y + y1) * cr).sum() / (6 * a)])


def inside_polygon(X, Y, poly, buffer=0.0):
    """Point-in-polygon (even-odd) on grids X, Y, plus everything within `buffer` m of the outline."""
    inside = np.zeros(X.shape, bool)
    P = np.asarray(poly)
    if np.allclose(P[0], P[-1]):
        P = P[:-1]
    near = np.full(X.shape, np.inf)
    for (x1, y1), (x2, y2) in zip(P, np.roll(P, -1, 0)):
        c = ((y1 > Y) != (y2 > Y)) & (X < (x2 - x1) * (Y - y1) / (y2 - y1 + 1e-12) + x1)
        inside ^= c
        dx, dy = x2 - x1, y2 - y1
        t = np.clip(((X - x1) * dx + (Y - y1) * dy) / (dx * dx + dy * dy + 1e-12), 0, 1)
        near = np.minimum(near, np.hypot(X - x1 - t * dx, Y - y1 - t * dy))
    return inside | (near < buffer)


def find_start_pad(ref, hut_xy):
    """Bare dirt patch near the hut: pixels redder/browner than the surrounding grass, within 45 m of the hut,
    excluding the hut roof and the formed track's far side. Returns centre, radius, mask image info."""
    half = 45.0; st = 0.25
    g = np.arange(-half, half + 1e-9, st)
    GX, GY = np.meshgrid(hut_xy[0] + g, hut_xy[1] + g[::-1])
    A = ref.img_at(np.stack([GX, GY], -1))
    R, Gc, B = A[..., 0], A[..., 1], A[..., 2]
    lum = (R + Gc + B) / 3
    grass = (Gc > R) & (lum > 90)
    gm = np.median((R - Gc)[grass])
    soil = gaussian_filter((R - Gc) - gm, 1.0 / st)
    return GX, GY, A, soil


def main():
    ref = Refiner()
    S0, dense = resample(ref.osm, DS)
    line0 = smooth_line(dense, 5.0, DS)
    line1, r1 = ref.run_pass(line0, 30.0, 'p1', 65.0, sigma=12.0)
    line2, r2 = ref.run_pass(line1, 12.0, 'p2', 40.0, sigma=10.0)
    # total offset vs the reference (OSM-based) line, measured along the reference normals
    _, nrm0 = frame(line0)
    tot = np.einsum('ij,ij->i', line2 - line0, nrm0)
    # distance from final line to the raw OSM polyline
    from scipy.spatial import cKDTree
    _, osm_dense = resample(ref.osm, 0.5)
    d_osm, _ = cKDTree(osm_dense).query(line2)
    print(f'final vs OSM polyline: median {np.median(d_osm):.2f} m, p90 {np.percentile(d_osm, 90):.1f} m, '
          f'max {d_osm.max():.1f} m (at s~{S0[np.argmax(d_osm)]:.0f})')

    # map OSM features (ford, bridge) onto the refined line by nearest point
    tree = cKDTree(line2)
    def s_of(lat, lon):
        x, y = ref.tm.fwd(lat, lon); d, i = tree.query([float(x), float(y)]); return i * DS, d
    rt = ref.route
    ll = np.array(rt['latlon'])
    ford_s, ford_d = s_of(rt['ford']['lat'], rt['ford']['lon'])
    # length of refined line
    seg = np.hypot(*np.diff(line2, axis=0).T); Sref = np.r_[0, np.cumsum(seg)]
    b0, _ = s_of(*ll[rt['bridge']['idx0']]); b1, _ = s_of(*ll[rt['bridge']['idx1']])
    print(f'refined length {Sref[-1]:.1f} m (OSM {rt["length_m"]:.1f}); ford at s={Sref[int(ford_s)]:.0f} '
          f'({ford_d:.1f} m off line); bridge {Sref[int(b0)]:.0f}..{Sref[-1]:.0f}')

    # lane width
    # end of the open clearing: first station where the canopy starts (dark), from the imagery
    lumc = ref.img_at(line2[:600]).mean(-1)
    clearing_end = int(np.argmax(gaussian_filter1d(lumc, 5) < 0.75 * np.median(lumc[20:150])))
    lw = lane_widths(ref, line2, r2, clearing_end)
    lane_width_png(ref, lw)
    ws = np.array([w['width'] for w in lw])
    for w in lw:
        print(f'  lane width at s={w["s"]:.0f} ({w["kind"]}, {w["feature"]}, conf {w["conf"]:.2f}): {w["width"]:.2f} m')
    for k in ('clearing', 'forest'):
        v = np.array([w['width'] for w in lw if w['kind'] == k])
        if len(v):
            print(f'  {k}: n={len(v)} median {np.median(v):.2f} p10-p90 {np.percentile(v, 10):.2f}-{np.percentile(v, 90):.2f}')
    print(f'lane width median {np.median(ws):.2f} m, p10-p90 {np.percentile(ws, 10):.2f}-{np.percentile(ws, 90):.2f}, '
          f'min-max {ws.min():.2f}-{ws.max():.2f} (n={len(ws)}); clearing ends at s~{clearing_end} m')

    # hut + start pad
    hut = next(c for c in rt['hut_context'] if c['tags'].get('building'))
    hg = np.array(hut['geometry'])
    hut_poly = np.c_[ref.tm.fwd(hg[:, 0], hg[:, 1])]
    hut_xy = polygon_centroid(hut_poly)
    GX, GY, A, soil = find_start_pad(ref, hut_xy)
    lat_lon_out = {}
    lat_lon_out['hut'] = [float(v) for v in ref.tm.inv(*hut_xy)]
    pad = start_pad(GX, GY, A, soil, hut_xy, hut_poly, line2)

    la, lo = ref.tm.inv(line2[:, 0], line2[:, 1])
    la0, lo0 = ref.tm.inv(line0[:, 0], line0[:, 1])
    save_json(os.path.join(DATA, 'route_refined.json'), {
        'note': 'refined centreline, 1 m steps along the OSM-based reference line; s here is the reference '
                'station (approximately arc length).', 'step_m': DS,
        'latlon': np.c_[la, lo].tolist(), 'ref_latlon': np.c_[la0, lo0].tolist(),
        'offset_total': tot.tolist(), 'conf_p2': r2['conf'].tolist(), 'conf_p1': r1['conf'].tolist(),
        'dist_to_osm': d_osm.tolist(),
        'ford_idx': int(ford_s), 'bridge_idx0': int(b0), 'bridge_idx1': len(line2) - 1,
        'lane_widths': lw, 'lane_width_median': float(np.median(ws)),
        'lane_width_range': [float(np.percentile(ws, 10)), float(np.percentile(ws, 90))],
        'clearing_end_idx': clearing_end,
        'hut_latlon': lat_lon_out['hut'], 'pad': pad,
    })
    overlays(ref, line0, line2, d_osm, pad, hut_xy)


def start_pad(GX, GY, A, soil, hut_xy, hut_poly, line):
    """Pick the bare-soil blob west/south-west of the hut that touches the start of the track."""
    st = GX[0, 1] - GX[0, 0]
    thr = np.percentile(soil, 80)
    lum = A.mean(-1)
    m = (soil > max(thr, 4.0)) & (lum > 70)
    # drop the hut roof: OSM building polygon buffered by 1.5 m
    m &= ~inside_polygon(GX, GY, hut_poly, buffer=1.5)
    # drop the formed track beyond 25 m along it (it is bare too)
    from scipy.spatial import cKDTree
    d_line, i_line = cKDTree(line).query(np.c_[GX.ravel(), GY.ravel()])
    i_line = i_line.reshape(GX.shape); d_line = d_line.reshape(GX.shape)
    m &= ~((d_line < 4.0) & (i_line > 12))
    lab, n = label(m)
    best, bscore = None, -1
    for k in range(1, n + 1):
        b = lab == k
        area = b.sum() * st * st
        if area < 20:
            continue
        cx, cy = GX[b].mean(), GY[b].mean()
        west = hut_xy[0] - cx
        dstart = np.hypot(cx - line[0, 0], cy - line[0, 1])
        score = area / (1 + dstart / 15) * (1.5 if west > -3 else 0.5)
        if score > bscore:
            best, bscore = b, score
    cx, cy = float(GX[best].mean()), float(GY[best].mean())
    area = best.sum() * st * st
    r_eq = float(np.sqrt(area / np.pi))
    # heading: from pad centre toward the track ~20 m along it (onto the track)
    d0 = np.hypot(line[:, 0] - cx, line[:, 1] - cy)
    j = int(np.argmin(d0[:60]))
    tgt = line[min(j + 20, len(line) - 1)]
    hd = float(np.arctan2(tgt[1] - cy, tgt[0] - cx))
    print(f'start pad: centre local ({cx:.1f},{cy:.1f}) m from track start, {np.hypot(cx - hut_xy[0], cy - hut_xy[1]):.1f} m '
          f'from hut centre (bearing {np.degrees(np.arctan2(cx - hut_xy[0], cy - hut_xy[1])) % 360:.0f} deg); '
          f'area {area:.0f} m2, equiv radius {r_eq:.1f} m; nearest track station {j} m; heading {np.degrees(hd):.0f} deg (atan2, 0=E)')
    return {'local_xy': [cx, cy], 'radius': r_eq, 'area': float(area), 'heading': hd, 'track_idx': j,
            'mask_xy': np.c_[GX[best][::2], GY[best][::2]].tolist()}


def overlays(ref, before, after, d_osm, pad, hut_xy):
    mos = ref.mos
    for f in os.listdir(DIAG):
        if f.startswith('overlay-'):
            os.remove(os.path.join(DIAG, f))
    def to_pix(xy):
        la, lo = ref.tm.inv(xy[:, 0], xy[:, 1]); c, r = mos.pix(la, lo); return np.c_[c + 0.5, r + 0.5]
    pb, pa = to_pix(before), to_pix(after)
    osm_p = to_pix(ref.osm)
    # overview: bbox of the route, downscale 6x
    lo_, hi_ = np.floor(pa.min(0) - 250).astype(int), np.ceil(pa.max(0) + 250).astype(int)
    k = 6
    crop = mos.img.crop((lo_[0], lo_[1], hi_[0], hi_[1]))
    ov = crop.resize((crop.width // k, crop.height // k), Image.LANCZOS)
    d = ImageDraw.Draw(ov)
    d.line([tuple((p - lo_) / k) for p in osm_p], fill=(255, 60, 60), width=2)
    d.line([tuple((p - lo_) / k) for p in pa], fill=(0, 255, 255), width=1)
    d.text((10, 10), 'red = OSM, cyan = refined. Imagery (c) State of NSW (Spatial Services), CC BY 4.0', fill=(255, 255, 255))
    ov.save(os.path.join(DIAG, 'overlay-overview.png'))
    # zoomed crops at the twistiest bits: rank 150 m windows by total |heading change| of the refined line
    t, _ = frame(after)
    h = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    dh = np.abs(np.diff(gaussian_filter1d(h, 3)))
    win = np.convolve(dh, np.ones(150), 'same')
    picks = []
    for i in np.argsort(-win):
        if all(abs(i - p) > 400 for p in picks):
            picks.append(int(i))
        if len(picks) == 5:
            break
    for n_, i in enumerate(sorted(picks)):
        c = pa[i]; half = 260
        box = (int(c[0] - half), int(c[1] - half), int(c[0] + half), int(c[1] + half))
        im = mos.img.crop(box).resize((780, 780), Image.LANCZOS)
        sc = 780 / (2 * half)
        d = ImageDraw.Draw(im)
        d.line([tuple((p - box[:2]) * sc) for p in osm_p], fill=(255, 60, 60), width=2)
        for p in osm_p:
            q = (p - box[:2]) * sc; d.ellipse((q[0] - 4, q[1] - 4, q[0] + 4, q[1] + 4), outline=(255, 60, 60))
        d.line([tuple((p - box[:2]) * sc) for p in pa], fill=(0, 255, 255), width=2)
        d.text((8, 8), f'km {i / 1000:.2f}  red = OSM (nodes circled), cyan = refined; {2 * half * mos.res * 0.8:.0f} m across',
               fill=(255, 255, 255))
        im.save(os.path.join(DIAG, f'overlay-{n_ + 1}-km{i / 1000:.1f}.png'))
    # start pad
    c = to_pix(np.array([pad['local_xy']]))[0]; half = 110
    box = (int(c[0] - half), int(c[1] - half), int(c[0] + half), int(c[1] + half))
    sc = 4
    im = mos.img.crop(box).resize((2 * half * sc, 2 * half * sc), Image.LANCZOS)
    d = ImageDraw.Draw(im)
    mp = to_pix(np.array(pad['mask_xy']))
    for p in mp:
        q = (p - box[:2]) * sc; d.point(tuple(q), fill=(255, 200, 0))
    q = (c - box[:2]) * sc; rpx = pad['radius'] / (mos.res * 0.8025) * sc
    d.ellipse((q[0] - rpx, q[1] - rpx, q[0] + rpx, q[1] + rpx), outline=(255, 255, 0), width=2)
    hd = pad['heading']; L = 8 / (mos.res * 0.8025) * sc
    d.line((q[0], q[1], q[0] + L * np.cos(hd), q[1] - L * np.sin(hd)), fill=(255, 0, 0), width=4)
    d.line([tuple((p - box[:2]) * sc) for p in pa[:200]], fill=(0, 255, 255), width=2)
    hp = (to_pix(hut_xy[None])[0] - box[:2]) * sc
    d.ellipse((hp[0] - 6, hp[1] - 6, hp[0] + 6, hp[1] + 6), outline=(255, 255, 255), width=2)
    d.text((8, 8), 'yellow = start pad (bare dirt, equiv. circle), red = start heading, cyan = refined track, '
                   'white = hut', fill=(255, 255, 255))
    im.save(os.path.join(DIAG, 'start-pad.png'))


if __name__ == '__main__':
    main()
