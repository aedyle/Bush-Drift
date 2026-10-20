#!/usr/bin/env python3
"""Step 7 (session 3): the far land -> data/built/far.bin.

The road's own ground stops 64 m from the centreline. Past that the game draws a coarse landscape so the
horizon is hills and forest, not sky: ground heights from the NSW 5 m DEM resampled to 20 m, and how wooded
each 20 m cell is from NSW imagery at z15 (~3.8 m/px on the ground). Covers the route's box plus MARGIN.
Downloads cached under data/raw/far/.

far.bin, little-endian: b'BDF1', int32 nx, ny, float32 x0, y0, cell, then float32 dem[ny][nx] (metres above
sea level, row j at y0 + j*cell, local frame of meta.txt), uint8 cover[ny][nx] (0 open .. 255 forest) and
float32 dist[ny][nx] (metres to the route's centreline)."""
import io
import math
import os
import struct
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, map_coordinates

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RAW, DATA, cached, http_get, merc, LocalTM

DEM_SVC = 'https://maps.six.nsw.gov.au/arcgis/rest/services/public/NSW_5M_Elevation/ImageServer'
IMG_SVC = 'https://maps.six.nsw.gov.au/arcgis/rest/services/public/NSW_Imagery/MapServer'
FAR = os.path.join(RAW, 'far')
MARGIN = 3000.0     # ground metres round the route's box
CELL = 20.0         # output cell, ground metres
DEM_PIX = 20.0      # merc metres a DEM pixel (~16 m ground)
Z = 15
ORIGIN = 20037508.342787
RES = 2 * ORIGIN / 256 / 2 ** Z


def read_meta():
    meta = {}
    for line in open(os.path.join(DATA, 'built', 'meta.txt')):
        p = line.split()
        if len(p) >= 2:
            meta[p[0]] = p[1:]
    return meta


def route_xy():
    a = np.loadtxt(os.path.join(DATA, 'built', 'route.csv'), delimiter=',', skiprows=1)
    return a[:, 1], a[:, 2]


def main():
    meta = read_meta()
    tm = LocalTM(float(meta['origin_lat'][0]), float(meta['origin_lon'][0]))
    rx, ry = route_xy()
    x0 = math.floor((rx.min() - MARGIN) / CELL) * CELL; x1 = math.ceil((rx.max() + MARGIN) / CELL) * CELL
    y0 = math.floor((ry.min() - MARGIN) / CELL) * CELL; y1 = math.ceil((ry.max() + MARGIN) / CELL) * CELL
    nx = int((x1 - x0) / CELL) + 1; ny = int((y1 - y0) / CELL) + 1
    GX, GY = np.meshgrid(x0 + np.arange(nx) * CELL, y0 + np.arange(ny) * CELL)
    lat, lon = tm.inv(GX, GY)
    mx, my = merc(lat, lon)
    print(f'far grid {nx} x {ny} of {CELL:.0f} m ({(x1 - x0) / 1000:.1f} x {(y1 - y0) / 1000:.1f} km)')

    # ---- DEM: one exportImage per <=1000 px tile, bilinear
    bx0, bx1, by0, by1 = mx.min() - 100, mx.max() + 100, my.min() - 100, my.max() + 100
    W = int(math.ceil((bx1 - bx0) / DEM_PIX)); H = int(math.ceil((by1 - by0) / DEM_PIX))
    dem = np.full((H, W), np.nan, np.float32)    # row 0 = north
    for ty in range(0, H, 1000):
        for tx in range(0, W, 1000):
            w, h = min(1000, W - tx), min(1000, H - ty)
            xmin = bx0 + tx * DEM_PIX; ymax = by1 - ty * DEM_PIX
            params = {'bbox': f'{xmin},{ymax - h * DEM_PIX},{xmin + w * DEM_PIX},{ymax}', 'bboxSR': 3857, 'imageSR': 3857,
                      'size': f'{w},{h}', 'format': 'tiff', 'pixelType': 'F32', 'noDataInterpretation': 'esriNoDataMatchAny',
                      'interpolation': 'RSP_BilinearInterpolation', 'compression': 'None', 'f': 'image'}
            path = os.path.join(FAR, f'dem-{int(xmin)}-{int(ymax)}-{w}x{h}.tif')
            data = cached(path, http_get(DEM_SVC + '/exportImage', params), binary=True)
            if data[:1] == b'{':
                os.remove(path); raise RuntimeError(data[:300])
            dem[ty:ty + h, tx:tx + w] = np.array(Image.open(io.BytesIO(data)), np.float32)
    dem[(dem < -100) | (dem > 3000)] = np.nan
    print(f'DEM {W}x{H}: {np.isfinite(dem).mean() * 100:.1f}% valid, {np.nanmin(dem):.0f}..{np.nanmax(dem):.0f} m')
    fill = np.where(np.isfinite(dem), dem, np.nanmean(dem))
    r = (by1 - my) / DEM_PIX - 0.5; c = (mx - bx0) / DEM_PIX - 0.5
    far_dem = map_coordinates(fill, [r.ravel(), c.ravel()], order=1, mode='nearest').reshape(ny, nx).astype(np.float32)

    # ---- imagery z15 tiles over the box, luminance mean and texture per 20 m cell
    tx0 = int((bx0 + ORIGIN) / (256 * RES)); tx1 = int((bx1 + ORIGIN) / (256 * RES))
    ty0 = int((ORIGIN - by1) / (256 * RES)); ty1 = int((ORIGIN - by0) / (256 * RES))
    MW, MH = (tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256
    mos = np.zeros((MH, MW, 3), np.uint8)
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            path = os.path.join(FAR, f'z{Z}', f'{ty}_{tx}.jpg')
            try:
                data = cached(path, http_get(f'{IMG_SVC}/tile/{Z}/{ty}/{tx}'), binary=True)
                mos[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = np.array(Image.open(io.BytesIO(data)).convert('RGB'))
            except Exception as e:  # noqa
                print('  tile fail', tx, ty, e)
    print(f'imagery z{Z}: {(tx1 - tx0 + 1) * (ty1 - ty0 + 1)} tiles, {RES / 1.25:.1f} m/px roughly')
    Image.fromarray(mos).save(os.path.join(FAR, f'mosaic-z{Z}.png'))
    lum = mos.astype(np.float32).mean(-1)
    mx0 = -ORIGIN + tx0 * 256 * RES; my1 = ORIGIN - ty0 * 256 * RES
    off = (np.arange(4) - 1.5) * (CELL / 4)
    samples = []
    for oy in off:
        for ox in off:
            la, lo = tm.inv(GX + ox, GY + oy)
            sx, sy = merc(la, lo)
            samples.append(map_coordinates(lum, [((my1 - sy) / RES).ravel(), ((sx - mx0) / RES).ravel()], order=1).reshape(ny, nx))
    S = np.stack(samples, -1)
    m, sd, has = S.mean(-1), S.std(-1), S.min(-1) > 3
    hx, hy = float(meta['hut_x'][0]), float(meta['hut_y'][0])
    dh = np.hypot(GX - hx, GY - hy)
    from scipy.spatial import cKDTree
    dist = cKDTree(np.c_[rx[::5], ry[::5]]).query(np.c_[GX.ravel(), GY.ravel()])[0].reshape(ny, nx)
    dist_full = cKDTree(np.c_[rx, ry]).query(np.c_[GX.ravel(), GY.ravel()])[0].reshape(ny, nx)   # metres to the centreline (every 2 m)
    # The z15 mosaic is two captures with a vertical seam (a dark sharp one west, a pale soft one east), so
    # each side is judged against itself: open ground is much brighter than that side's typical forest.
    col = gaussian_filter(lum.mean(0), 8)
    seam_px = int(np.argmax(np.abs(np.diff(col))[64:-64])) + 64
    sx_all = np.zeros((ny, nx))
    la, lo = tm.inv(GX, GY); sx_all, _ = merc(la, lo)
    east = (sx_all - mx0) / RES > seam_px
    p = np.zeros((ny, nx))
    for side in (False, True):
        sel = has & (east == side)
        if sel.sum() < 100:
            continue
        med = np.median(m[sel]); mad = np.median(np.abs(m[sel] - med)) * 1.4826 + 1e-3
        z = (m - med) / mad
        p = np.where(east == side, 1 / (1 + np.exp(2.0 * (z - 2.5))), p)   # z 2.5 sd brighter = half open
        print(f'  {"east" if side else "west"} side: lum median {med:.0f}, spread {mad:.1f}')
    p = gaussian_filter(p, 0.7)
    cover = np.where(has, np.round(p * 255), 255).astype(np.uint8)
    print(f'cover: seam at mosaic column {seam_px}; round the hut (20-90 m) {cover[(dh < 90) & (dh > 20)].mean():.0f}, '
          f'60-300 m off the route {cover[(dist > 60) & (dist < 300)].mean():.0f}, all {cover.mean():.0f} (255 = forest)')

    with open(os.path.join(DATA, 'built', 'far.bin'), 'wb') as f:
        f.write(b'BDF1'); f.write(struct.pack('<iifff', nx, ny, x0, y0, CELL))
        f.write(far_dem.astype('<f4').tobytes()); f.write(cover.tobytes())
        f.write(np.minimum(dist_full, 1e5).astype('<f4').tobytes())
    Image.fromarray(cover[::-1]).save(os.path.join(DATA, 'built', 'far_cover_preview.png'))
    print('wrote data/built/far.bin')


if __name__ == '__main__':
    main()
