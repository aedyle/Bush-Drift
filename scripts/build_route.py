#!/usr/bin/env python3
"""Step 5: final game data in a local metric frame -> data/built/{route.csv, meta.txt, terrain.bin, canopy.bin}.

Frame: origin = start-pad centre; x = east m, y = north m. Projection: ellipsoidal (WGS84) local
equirectangular about the origin (meridional radius M and prime-vertical radius N at the origin latitude,
east scale corrected linearly for latitude), see common.LocalTM. Over the +-6 km of the route the scale
error is < 1e-5 (vs. the 0.1 % requirement); round-trip lat/lon error is checked below.

route.csv: s,x,y,elev,elev_dem,heading,curv every 2.0 m along a smooth spline of the refined centreline,
hut end (s=0, OSM start node on the pad) to the far end of the bridge.
  elev     : road-surface elevation: DEM sampled every 1 m, 9 m running median (kills tree/DEM spikes), then
             Gaussian sigma 15 m (FWHM ~35 m); near the Solomons Creek ford the scale tapers to sigma 5 m so
             the dip survives; the bridge deck is a straight line between its abutments.
  elev_dem : raw DEM (bilinear) at the point.
  heading  : atan2(dy, dx) radians (0 = east, CCW +), unwrapped is NOT applied (wrapped to -pi..pi).
  curv     : 1/m, + = turning left (CCW).
terrain.bin / canopy.bin: see scripts/README.md."""
import json
import os
import struct
import sys

import numpy as np
from PIL import Image
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d, map_coordinates, median_filter, gaussian_filter, distance_transform_edt
from scipy.spatial import cKDTree
from scipy.interpolate import griddata

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import DATA, BUILT, DOCS, LocalTM, load_json, merc
from imagery import Mosaic

STEP = 2.0
CELL = 5.0
MARGIN = 90.0
NEAR = 80.0
RMIN = 10.0     # minimum centreline radius (m) after the kink limiter

# Session 3b: the track at the hut as it is now. The 2011 NSW imagery the line was refined on shows the old track
# running past the hut's south corner to a bare patch beside it; since then NPWS moved it about 5.5 m south, lined
# its north edge with bollards and made a dirt turning loop south-west of the hut (Google imagery and the NPWS
# photos; docs/hut/measurements.md). The first stretch is shifted to the right by MOVE_M, easing to nothing
# between MOVE_S0 and MOVE_S1 metres along. The car park is a bare pad centred CARPARK_HUT metres E, N of the hut.
MOVE_M, MOVE_S0, MOVE_S1 = 5.5, 40.0, 72.0
CARPARK_HUT, CARPARK_R = (-14.5, -17.0), 9.0


class DEM:
    def __init__(self):
        z = np.load(os.path.join(DATA, 'dem_corridor.npz'))
        self.dem = z['dem']; self.x0 = float(z['x0']); self.y1 = float(z['y1']); self.pix = float(z['pix'])

    def at(self, lat, lon):
        mx, my = merc(lat, lon)
        c = (mx - self.x0) / self.pix - 0.5; r = (self.y1 - my) / self.pix - 0.5
        v = map_coordinates(self.dem, [np.ravel(r), np.ravel(c)], order=1, mode='constant', cval=np.nan)
        return v.reshape(np.shape(lat))


def smooth_xy(xy, sigma, step):
    n = int(4 * sigma / step) + 1
    a = np.r_[2 * xy[0] - xy[n:0:-1], xy, 2 * xy[-1] - xy[-2:-n - 2:-1]]
    return np.c_[gaussian_filter1d(a[:, 0], sigma / step), gaussian_filter1d(a[:, 1], sigma / step)][n:n + len(xy)]


def poly_curv(xy):
    d1 = np.gradient(xy, axis=0); d2 = np.gradient(d1, axis=0)
    return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.hypot(d1[:, 0], d1[:, 1]) ** 3


def limit_radius(xy, rmin, max_iter=40):
    """Locally smooth (1 m polyline) wherever the radius drops below rmin: the remaining tight spots are node
    kinks in the OSM geometry (and the junction / ford V), not real sub-10 m corners. Each pass blends in a
    sigma-6 m smoothed copy with a Gaussian weight (sigma 8 m) around the offending points, re-resampling to
    1 m. The total turn of each bend is preserved; the apex moves inward by a few metres."""
    for it in range(max_iter):
        k = gaussian_filter1d(poly_curv(xy), 1.0)
        bad = np.abs(k) > 1 / rmin
        bad[:3] = bad[-3:] = False
        if not bad.any():
            break
        w = gaussian_filter1d(bad.astype(float), 8.0) * 8.0 * np.sqrt(2 * np.pi)
        w = np.clip(w, 0, 1)[:, None]
        xy = resample(xy * (1 - w) + smooth_xy(xy, 6.0, 1.0) * w, 1.0)
    print(f'radius limiter: {it} passes, min radius now {1 / np.abs(gaussian_filter1d(poly_curv(xy), 1.0)).max():.1f} m')
    return xy


def resample(xy, step):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    S = np.arange(0, d[-1], step)
    if d[-1] - S[-1] > 1e-6:
        S = np.r_[S, d[-1]]
    return np.c_[np.interp(S, d, xy[:, 0]), np.interp(S, d, xy[:, 1])]


def main():
    ref = load_json(os.path.join(DATA, 'route_refined.json'))
    osm = load_json(os.path.join(DATA, 'route_osm.json'))
    ll = np.array(ref['latlon'])
    # refine-frame -> lat/lon for the pad centre (refine used a LocalTM at the OSM start node)
    tm_r = LocalTM(*np.array(osm['latlon'][0]))
    pad_lat, pad_lon = [float(v) for v in tm_r.inv(*ref['pad']['local_xy'])]
    tm = LocalTM(pad_lat, pad_lon)
    X, Y = tm.fwd(ll[:, 0], ll[:, 1])
    # round-trip and scale checks against the WGS84 ellipsoid (Vincenty), every 50th point + pairs
    from common import vincenty
    la2, lo2 = tm.inv(X, Y)
    rt_err = np.max(np.hypot((la2 - ll[:, 0]) * 111e3, (lo2 - ll[:, 1]) * 89e3))
    worst = 0.0
    idx = np.arange(0, len(X), 50)
    for i in idx:
        d_true = vincenty(pad_lat, pad_lon, ll[i, 0], ll[i, 1])
        if d_true > 100:
            worst = max(worst, abs(np.hypot(X[i], Y[i]) / d_true - 1))
    for i, j in zip(idx[:-1], idx[1:][::-1]):
        d_true = vincenty(ll[i, 0], ll[i, 1], ll[j, 0], ll[j, 1])
        if d_true > 100:
            worst = max(worst, abs(np.hypot(X[i] - X[j], Y[i] - Y[j]) / d_true - 1))
    print(f'frame: origin {pad_lat:.7f},{pad_lon:.7f}; round-trip err {rt_err * 1000:.3f} mm; '
          f'worst distance scale error vs WGS84 geodesic {worst * 100:.4f} %')

    # ---- smooth spline centreline, exact arc length, 2 m resample
    xy = resample(np.c_[X, Y], 1.0)
    d0 = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    t = np.gradient(xy, axis=0); t /= np.hypot(*t.T)[:, None]
    q = np.clip((d0 - MOVE_S0) / (MOVE_S1 - MOVE_S0), 0, 1); w = MOVE_M * (1 - q * q * (3 - 2 * q))
    xy = xy + np.c_[t[:, 1], -t[:, 0]] * w[:, None]          # to the right of the direction of travel
    print(f'start realigned: {MOVE_M} m to the right up to s={MOVE_S0:.0f}, easing out by s={MOVE_S1:.0f}')
    xy = smooth_xy(xy, 3.0, 1.0)
    xy = limit_radius(xy, RMIN)
    u = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    csx, csy = CubicSpline(u, xy[:, 0]), CubicSpline(u, xy[:, 1])
    uf = np.linspace(0, u[-1], int(u[-1] * 20) + 1)        # 5 cm arc-length integration
    sp = np.hypot(csx(uf, 1), csy(uf, 1))
    sf = np.r_[0, np.cumsum((sp[1:] + sp[:-1]) / 2 * np.diff(uf))]
    length = sf[-1]
    S = np.arange(0, length, STEP)
    if length - S[-1] > 0.5:
        S = np.r_[S, length]
    U = np.interp(S, sf, uf)
    x, y = csx(U), csy(U)
    dx, dy = csx(U, 1), csy(U, 1); ddx, ddy = csx(U, 2), csy(U, 2)
    heading = np.arctan2(dy, dx)
    curv = (dx * ddy - dy * ddx) / np.hypot(dx, dy) ** 3
    print(f'length {length:.1f} m, {len(S)} samples; min radius {1 / np.abs(curv).max():.1f} m at s={S[np.argmax(abs(curv))]:.0f}')

    # ---- elevation
    dem = DEM()
    la, lo = tm.inv(x, y)
    elev_dem = dem.at(la, lo)
    s1 = np.arange(0, length, 1.0); U1 = np.interp(s1, sf, uf)
    la1, lo1 = tm.inv(csx(U1), csy(U1))
    z1 = dem.at(la1, lo1)
    if np.isnan(z1).any():
        ok = ~np.isnan(z1); z1 = np.interp(s1, s1[ok], z1[ok]); print('  filled DEM NaNs along line')
    # route positions of ford and bridge (nearest point on the final line)
    tree = cKDTree(np.c_[csx(U1), csy(U1)])
    fx, fy = tm.fwd(osm['ford']['lat'], osm['ford']['lon'])
    ford_s = float(s1[tree.query([float(fx), float(fy)])[1]])
    bl = np.array(osm['latlon'])[[osm['bridge']['idx0'], osm['bridge']['idx1']]]
    bx, by = tm.fwd(bl[:, 0], bl[:, 1])
    bridge_s0 = float(s1[tree.query([float(bx[0]), float(by[0])])[1]])
    bridge_s1 = float(length)
    zm = median_filter(z1, size=9, mode='nearest')
    # tapered smoothing: sigma 15 m, down to 5 m near the ford
    sig = 15.0 - 10.0 * np.exp(-0.5 * ((s1 - ford_s) / 30.0) ** 2)
    pad_n = 60
    zp = np.r_[np.full(pad_n, zm[0]), zm, np.full(pad_n, zm[-1])]
    sp_ = np.r_[s1[0] - np.arange(pad_n, 0, -1), s1, s1[-1] + np.arange(1, pad_n + 1)]
    zs = np.empty_like(z1)
    for i, (si, sg) in enumerate(zip(s1, sig)):
        w = np.exp(-0.5 * ((sp_ - si) / sg) ** 2); zs[i] = (w * zp).sum() / w.sum()
    # bridge deck: straight line between the smoothed abutment heights
    b = s1 >= bridge_s0
    zs[b] = np.interp(s1[b], [s1[b][0], s1[-1]], [zs[b][0], zs[-1]])
    elev = np.interp(S, s1, zs)
    g = np.gradient(zs, s1)
    dg = np.gradient(gaussian_filter1d(g, 3), s1)
    print(f'elev: start {elev[0]:.1f}, ford (s={ford_s:.0f}) {np.interp(ford_s, s1, zs):.1f} (DEM {np.interp(ford_s, s1, z1):.1f}), '
          f'bridge {np.interp(bridge_s0, s1, zs):.1f}; max |grade| {np.abs(g).max() * 100:.1f} %, '
          f'max |grade change| {np.abs(dg).max() * 100:.2f} %/m; resid DEM-elev rms {np.sqrt(np.mean((z1 - zs) ** 2)):.2f} m')

    # ---- route.csv
    os.makedirs(BUILT, exist_ok=True)
    with open(os.path.join(BUILT, 'route.csv'), 'w') as f:
        f.write('s,x,y,elev,elev_dem,heading,curv\n')
        for row in zip(S, x, y, elev, elev_dem, heading, curv):
            f.write('%.2f,%.3f,%.3f,%.3f,%.3f,%.6f,%.7f\n' % row)

    # ---- terrain grid
    pad = ref['pad']
    hut_lat, hut_lon = ref['hut_latlon']
    hx, hy = [float(v) for v in tm.fwd(hut_lat, hut_lon)]
    xmin, xmax = min(x.min(), -30) - MARGIN, max(x.max(), 30) + MARGIN
    ymin, ymax = min(y.min(), -30) - MARGIN, max(y.max(), 30) + MARGIN
    x0 = np.floor(xmin / CELL) * CELL; y0 = np.floor(ymin / CELL) * CELL
    nx = int(np.ceil((xmax - x0) / CELL)) + 1; ny = int(np.ceil((ymax - y0) / CELL)) + 1
    gx = x0 + np.arange(nx) * CELL; gy = y0 + np.arange(ny) * CELL
    GX, GY = np.meshgrid(gx, gy)                    # [ny][nx], row j = y0 + j*cell
    gla, glo = tm.inv(GX, GY)
    D = dem.at(gla, glo).astype(np.float32)
    # nearest centreline point (0.25 m samples) -> near_s, signed lateral
    sd = np.arange(0, length, 0.25); Ud = np.interp(sd, sf, uf)
    px_, py_ = csx(Ud), csy(Ud); tx_, ty_ = csx(Ud, 1), csy(Ud, 1); tn = np.hypot(tx_, ty_); tx_ /= tn; ty_ /= tn
    dist, idx = cKDTree(np.c_[px_, py_]).query(np.c_[GX.ravel(), GY.ravel()])
    dist = dist.reshape(GX.shape); idx = idx.reshape(GX.shape)
    near_s = sd[idx].astype(np.float32)
    side = np.sign(tx_[idx] * (GY - py_[idx]) - ty_[idx] * (GX - px_[idx]))
    lat = (side * dist).astype(np.float32)
    far = dist > NEAR
    near_s[far] = np.nan; lat[far] = np.nan
    # fill DEM gaps within NEAR (+ pad area) by interpolation from known cells
    need = (dist <= NEAR) | (np.hypot(GX, GY) <= 60)
    gap = need & np.isnan(D)
    if gap.any():
        ok = np.isfinite(D)
        D[gap] = griddata(np.c_[GX[ok], GY[ok]], D[ok], np.c_[GX[gap], GY[gap]], method='linear')
        still = gap & np.isnan(D)
        if still.any():
            D[still] = griddata(np.c_[GX[ok], GY[ok]], D[ok], np.c_[GX[still], GY[still]], method='nearest')
    print(f'terrain grid {nx} x {ny} @ {CELL} m, origin ({x0:.0f},{y0:.0f}); DEM known {np.isfinite(D).mean() * 100:.0f}% of cells; '
          f'gaps filled near route: {gap.sum()}; NaN within {NEAR:.0f} m: {(need & np.isnan(D)).sum()}')
    with open(os.path.join(BUILT, 'terrain.bin'), 'wb') as f:
        f.write(b'BDT1'); f.write(struct.pack('<iifff', nx, ny, x0, y0, CELL))
        for a in (D, near_s, lat):
            f.write(np.ascontiguousarray(a, '<f4').tobytes())

    # ---- canopy from imagery
    can, clear_r = canopy(tm, GX, GY, hx, hy, dist)
    with open(os.path.join(BUILT, 'canopy.bin'), 'wb') as f:
        f.write(b'BDC1'); f.write(struct.pack('<iifff', nx, ny, x0, y0, CELL))
        f.write(np.ascontiguousarray(can, np.uint8).tobytes())

    # ---- start pose and meta
    pz = float(dem.at(np.array([pad_lat]), np.array([pad_lon]))[0])
    # pad elevation: mean DEM over the pad disc
    rr = np.hypot(GX, GY) <= max(pad['radius'], CELL)
    pad_elev = float(np.nanmean(D[rr])) if rr.any() else pz
    cpx, cpy = hx + CARPARK_HUT[0], hy + CARPARK_HUT[1]
    rc = np.hypot(GX - cpx, GY - cpy) <= CARPARK_R
    carpark_h = float(np.nanmean(D[rc])) - pad_elev
    lw = ref['lane_widths']
    meta = [
        ('origin_lat', f'{pad_lat:.8f}'), ('origin_lon', f'{pad_lon:.8f}'),
        ('projection', 'local_equirectangular_wgs84 x=N0*cos(lat)*dlon y=M0*dlat (ellipsoidal radii at origin, see scripts/common.py LocalTM)'),
        ('length_m', f'{length:.2f}'),
        ('lane_width_m', f'{ref["lane_width_median"]:.2f}'),
        ('lane_width_range', f'{ref["lane_width_range"][0]:.2f} {ref["lane_width_range"][1]:.2f}'),
        ('start_x start_y start_heading', None),
        ('pad_x pad_y pad_r pad_elev', None),
        ('clearing_r', f'{clear_r:.0f}'),
        ('ford_s', f'{ford_s:.1f}'),
        ('bridge_s0 bridge_s1', None),
        ('hut_x hut_y', None),
        ('route_start_s', '0'),
    ]
    with open(os.path.join(BUILT, 'meta.txt'), 'w') as f:
        for k, v in meta:
            if k == 'start_x start_y start_heading':
                f.write(f'start_x 0.00\nstart_y 0.00\nstart_heading {pad["heading"]:.4f}\n')
            elif k == 'pad_x pad_y pad_r pad_elev':
                f.write(f'pad_x {cpx:.2f}\npad_y {cpy:.2f}\npad_r {CARPARK_R:.1f}\npad_elev {pad_elev:.2f}\npad_h {carpark_h:.2f}\n')
            elif k == 'bridge_s0 bridge_s1':
                f.write(f'bridge_s0 {bridge_s0:.1f}\nbridge_s1 {bridge_s1:.1f}\n')
            elif k == 'hut_x hut_y':
                f.write(f'hut_x {hx:.2f}\nhut_y {hy:.2f}\n')
            else:
                f.write(f'{k} {v}\n')
        f.write(f'elev_start {elev[0]:.2f}\nelev_end {elev[-1]:.2f}\n')
        f.write(f'grid_nx {nx}\ngrid_ny {ny}\ngrid_x0 {x0:.1f}\ngrid_y0 {y0:.1f}\ngrid_cell {CELL:.1f}\n')
    print(open(os.path.join(BUILT, 'meta.txt')).read())
    np.savez_compressed(os.path.join(DATA, 'built', 'debug_profile.npz'), s1=s1, z1=z1, zs=zs, ford_s=ford_s,
                        bridge_s0=bridge_s0)


def canopy(tm, GX, GY, hx, hy, dist):
    """0 = open ground, 255 = canopy. Per 5 m cell, 4x4 imagery samples (1.25 m): canopy is darker and much more
    textured (crowns + shadows) than grass, clearings or road. Score from mean luminance and texture, calibrated
    on the hut paddock (open) and the forest 40-80 m off the track (canopy). Cells with no imagery: 255."""
    mos = Mosaic()
    ny, nx = GX.shape
    off = (np.arange(4) - 1.5) * 1.25
    lum = np.zeros((ny, nx, 16), np.float32); exr = np.zeros((ny, nx, 16), np.float32)
    k = 0
    for oy in off:
        for ox in off:
            la, lo = tm.inv(GX + ox, GY + oy)
            a = mos.sample(la, lo)
            lum[..., k] = a.mean(-1); exr[..., k] = 2 * a[..., 1] - a[..., 0] - a[..., 2]
            k += 1
    m = lum.mean(-1); sd = lum.std(-1); has = lum.min(-1) > 3
    m = gaussian_filter(m, 0.7); sd = gaussian_filter(sd, 0.7)
    open_ref = has & (np.hypot(GX - hx, GY - hy) < 120) & (np.hypot(GX - hx, GY - hy) > 15)
    forest_ref = has & (dist > 40) & (dist < 80) & (np.hypot(GX - hx, GY - hy) > 400)
    # project onto a 1-D score: standardised darkness + texture
    feats = np.stack([m, sd], -1)
    mu_o, mu_f = np.median(feats[open_ref], 0), np.median(feats[forest_ref], 0)
    print(f'canopy calib: open (lum {mu_o[0]:.0f}, sd {mu_o[1]:.1f}), forest (lum {mu_f[0]:.0f}, sd {mu_f[1]:.1f})')
    # linear discriminant along the two class medians, using the pooled spread
    sp = np.sqrt(0.5 * (np.var(feats[open_ref], 0) + np.var(feats[forest_ref], 0)))
    zo = (feats - mu_o) / sp; zf = (feats - mu_f) / sp
    score = np.sum(zo ** 2, -1) - np.sum(zf ** 2, -1)          # >0 = closer to forest
    p = 1 / (1 + np.exp(-score / 2))
    p = gaussian_filter(p, 0.8)
    can = np.where(has, np.round(p * 255), 255).astype(np.uint8)
    can_o = can[open_ref]; can_f = can[forest_ref]
    print(f'canopy: open-ref cells mean {can_o.mean():.0f}, forest-ref mean {can_f.mean():.0f}; '
          f'route corridor (<20 m) mean {can[dist < 20].mean():.0f}')
    # clearing radius around the hut: distance to the nearest sizeable canopy (>=3 cells of p>0.6 after a 1-cell blur)
    trees = gaussian_filter((can > 153).astype(float), 1.0) > 0.5
    dt = np.hypot(GX - hx, GY - hy)
    ang = np.arctan2(GY - hy, GX - hx)
    radii = []
    for a0 in np.linspace(-np.pi, np.pi, 36, endpoint=False):
        sel = trees & (np.abs(np.angle(np.exp(1j * (ang - a0)))) < np.pi / 36) & has
        radii.append(dt[sel].min() if sel.any() else np.nan)
    radii = np.array(radii)
    clear_r = float(np.nanmedian(radii))
    print(f'clearing around hut: distance to tree line by direction: median {clear_r:.0f} m, '
          f'min {np.nanmin(radii):.0f}, max {np.nanmax(radii):.0f} (36 directions)')
    Image.fromarray(can[::-1]).save(os.path.join(DATA, 'built', 'canopy_preview.png'))
    return can, clear_r


if __name__ == '__main__':
    main()
