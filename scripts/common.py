"""Shared helpers for the bush-drift map-data pipeline: paths, polite cached HTTP, projections."""
import hashlib
import json
import math
import os
import time

import requests

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, 'data')
RAW = os.path.join(DATA, 'raw')
BUILT = os.path.join(DATA, 'built')
DOCS = os.path.join(ROOT, 'docs', 'route')
# diagnostics drawn over NSW imagery stay out of git (the imagery itself is never published)
DIAG = os.path.join(ROOT, 'shots', 'route')
for d in (RAW, BUILT, DOCS, DIAG, os.path.join(RAW, 'imagery'), os.path.join(RAW, 'dem')):
    os.makedirs(d, exist_ok=True)

UA = 'bush-drift-private/0.1 (personal hobby project)'
SESSION = requests.Session()
SESSION.headers['User-Agent'] = UA

# Fixed points (from the brief)
HUT_PIN = (-36.720885, 149.462798)
JUNCTION = (-36.722195, 149.4565126)
BRIDGE_PIN = (-36.7425306, 149.5068166)

_last = [0.0]


def polite_sleep(sec=0.7):
    dt = time.time() - _last[0]
    if dt < sec:
        time.sleep(sec - dt)
    _last[0] = time.time()


def cached(path, fetch, binary=False):
    """Return the cached file content at `path`, or call fetch() -> bytes, store it and return it."""
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path, 'rb') as f:
            data = f.read()
        return data if binary else data.decode('utf-8')
    polite_sleep()
    data = fetch()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(data)
    return data if binary else data.decode('utf-8')


def cache_name(prefix, params, ext):
    h = hashlib.sha1(json.dumps(params, sort_keys=True).encode()).hexdigest()[:12]
    return f'{prefix}-{h}.{ext}'


def http_get(url, params=None, timeout=120, method='GET'):
    def go():
        for attempt in range(4):
            try:
                if method == 'POST':
                    r = SESSION.post(url, data=params, timeout=timeout)
                else:
                    r = SESSION.get(url, params=params, timeout=timeout)
                if r.status_code in (429, 500, 502, 503, 504) and attempt < 3:
                    print(f'  HTTP {r.status_code}, retrying in {5 * 2 ** attempt} s', flush=True)
                    time.sleep(5 * 2 ** attempt)
                    continue
                r.raise_for_status()
                return r.content
            except requests.ConnectionError:
                if attempt == 3:
                    raise
                time.sleep(5 * 2 ** attempt)
    return go


# ---- projections -------------------------------------------------------------
R_WGS = 6378137.0


def merc(lat, lon):
    """EPSG:3857 forward (numpy-friendly)."""
    import numpy as np
    lat = np.asarray(lat, float); lon = np.asarray(lon, float)
    x = R_WGS * np.radians(lon)
    y = R_WGS * np.log(np.tan(np.pi / 4 + np.radians(lat) / 2))
    return x, y


def merc_inv(x, y):
    import numpy as np
    lon = np.degrees(np.asarray(x, float) / R_WGS)
    lat = np.degrees(2 * np.arctan(np.exp(np.asarray(y, float) / R_WGS)) - np.pi / 2)
    return lat, lon


class LocalTM:
    """Local transverse-Mercator-like frame: ellipsoidal (WGS84) equirectangular about an origin,
    using the meridional and prime-vertical radii of curvature at the origin latitude plus a
    second-order latitude correction for the east scale. Over +-6 km the error is < 1e-5 relative
    (far below the 0.1 % requirement). x = east m, y = north m."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        a = 6378137.0; f = 1 / 298.257223563; e2 = f * (2 - f)
        p = math.radians(lat0); s = math.sin(p)
        w = math.sqrt(1 - e2 * s * s)
        self.M = a * (1 - e2) / w ** 3      # meridional radius
        self.N = a / w                       # prime vertical radius
        self.cos0 = math.cos(p)
        self.tan0 = math.tan(p)

    def fwd(self, lat, lon):
        import numpy as np
        dlat = np.radians(np.asarray(lat, float) - self.lat0)
        dlon = np.radians(np.asarray(lon, float) - self.lon0)
        y = self.M * dlat
        # east scale follows cos(lat) across the area (exact to first order in dlat)
        x = self.N * dlon * (self.cos0 - math.sin(math.radians(self.lat0)) * dlat)
        return x, y

    def inv(self, x, y):
        import numpy as np
        dlat = np.asarray(y, float) / self.M
        dlon = np.asarray(x, float) / (self.N * (self.cos0 - math.sin(math.radians(self.lat0)) * dlat))
        return self.lat0 + np.degrees(dlat), self.lon0 + np.degrees(dlon)


def haversine(lat1, lon1, lat2, lon2):
    import numpy as np
    p1, p2 = np.radians(lat1), np.radians(lat2)
    dp = p2 - p1; dl = np.radians(np.asarray(lon2) - np.asarray(lon1))
    a = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * 6371008.8 * np.arcsin(np.sqrt(a))


def load_json(path):
    with open(path) as f:
        return json.load(f)


def save_json(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=1)


def vincenty(lat1, lon1, lat2, lon2):
    """WGS84 ellipsoidal distance (Vincenty inverse), metres. Scalar inputs."""
    a = 6378137.0; f = 1 / 298.257223563; b = a * (1 - f)
    L = math.radians(lon2 - lon1)
    U1 = math.atan((1 - f) * math.tan(math.radians(lat1))); U2 = math.atan((1 - f) * math.tan(math.radians(lat2)))
    sU1, cU1, sU2, cU2 = math.sin(U1), math.cos(U1), math.sin(U2), math.cos(U2)
    lam = L
    for _ in range(200):
        sl, cl = math.sin(lam), math.cos(lam)
        ss = math.hypot(cU2 * sl, cU1 * sU2 - sU1 * cU2 * cl)
        if ss == 0:
            return 0.0
        cs = sU1 * sU2 + cU1 * cU2 * cl
        sig = math.atan2(ss, cs)
        sa = cU1 * cU2 * sl / ss
        c2a = 1 - sa * sa
        c2sm = cs - 2 * sU1 * sU2 / c2a if c2a else 0.0
        C = f / 16 * c2a * (4 + f * (4 - 3 * c2a))
        lp = lam
        lam = L + (1 - C) * f * sa * (sig + C * ss * (c2sm + C * cs * (-1 + 2 * c2sm * c2sm)))
        if abs(lam - lp) < 1e-12:
            break
    u2 = c2a * (a * a - b * b) / (b * b)
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)))
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)))
    ds = B * ss * (c2sm + B / 4 * (cs * (-1 + 2 * c2sm * c2sm) - B / 6 * c2sm * (-3 + 4 * ss * ss) * (-3 + 4 * c2sm * c2sm)))
    return b * A * (sig - ds)
