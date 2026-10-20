"""Solve a photo's camera (position, heading, pitch, roll, vertical field of view) from points marked on it.

Usage: python tools/photomatch/solve_cam.py POINTS.json [VFOV]   (VFOV fixes the vertical field of view)
POINTS.json: {"size": [w, h], "pts": [[E, N, up, px, py], ...], "guess": [E, N, up, bearing_deg]}
E, N, up are metres from the hut's OSM centroid (up from the ground there); px, py are image pixels.
Prints the camera and a SHOT_CAM line for the game (camera + a target 30 m ahead) and the per-point error.
"""
import json, math, sys
import numpy as np
from scipy.optimize import least_squares

def rot(bearing, pitch, roll):
    b, p, r = map(math.radians, (bearing, pitch, roll))
    f = np.array([math.sin(b) * math.cos(p), math.cos(b) * math.cos(p), math.sin(p)])   # forward in E, N, up
    rt = np.array([math.cos(b), -math.sin(b), 0.0])
    up = np.cross(rt, f)
    rt, up = rt * math.cos(r) + up * math.sin(r), up * math.cos(r) - rt * math.sin(r)
    return f, rt, up

def ground_fn():
    """Ground height (m) relative to the ground at the hut, from data/built/terrain.bin (the game's own DEM grid)."""
    import struct
    b = open("data/built/terrain.bin", "rb").read()
    nx, ny = struct.unpack("<ii", b[4:12]); x0, y0, c = struct.unpack("<fff", b[12:24])
    D = np.frombuffer(b[24:24 + 4 * nx * ny], dtype=np.float32).reshape(ny, nx)
    meta = dict(l.split()[:2] for l in open("data/built/meta.txt") if len(l.split()) >= 2)
    hx, hy = float(meta["hut_x"]), float(meta["hut_y"])
    def at(x, y):
        fi, fj = (x - x0) / c, (y - y0) / c; i, j = int(fi), int(fj); a, b_ = fi - i, fj - j
        return (D[j, i] * (1 - a) + D[j, i + 1] * a) * (1 - b_) + (D[j + 1, i] * (1 - a) + D[j + 1, i + 1] * a) * b_
    h0 = at(hx, hy)
    return lambda e, n: at(hx + e, hy + n) - h0

def project(x, P, w, h):
    e, n, u, bearing, pitch, roll, fov = x
    f, rt, up = rot(bearing, pitch, roll)
    d = P - np.array([e, n, u])
    z = d @ f; fx = (h / 2) / math.tan(math.radians(fov) / 2)
    return np.stack([w / 2 + fx * (d @ rt) / z, h / 2 - fx * (d @ up) / z], 1)

def main():
    J = json.load(open(sys.argv[1])); w, h = J["size"]
    P = np.array([p[:3] for p in J["pts"]], float); I = np.array([p[3:5] for p in J["pts"]], float)
    g = J["guess"]
    best = None
    fix = float(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] != "-" else J.get("vfov")
    hz = J.get("horizon"); eye = J.get("eye"); G = ground_fn() if eye is not None else None
    for fov in ((fix,) if fix else (25, 40, 55, 70)):
        for db in (-30, 0, 30):
            x0 = [g[0], g[1], g[2], g[3] + db, 0, 0, fov]
            def res(x):
                r_ = (project(x, P, w, h) - I).ravel()
                if hz is not None:   # the far horizon's row in the photo pins the pitch
                    fx_ = (h / 2) / math.tan(math.radians(x[6]) / 2)
                    r_ = np.r_[r_, h / 2 - fx_ * math.tan(math.radians(x[4])) - hz]
                if eye is None: return r_
                return np.r_[r_, 8.0 * (x[2] - G(x[0], x[1]) - eye)]   # the photographer's eye ~1.6 m above the ground
            r = least_squares(res, x0,
                              bounds=([-300, -300, 0.3, -720, -45, -10, fov - 1e-3 if fix else 8], [300, 300, 30, 720, 45, 10, fov + 1e-3 if fix else 110]))
            if best is None or r.cost < best.cost: best = r
    x = best.x; err = np.hypot(*(project(x, P, w, h) - I).T)
    e, n, u, b, p, ro, fov = x
    f, _, _ = rot(b, p, ro)
    print(f"camera E {e:.1f} N {n:.1f} up {u:.2f} m, bearing {b % 360:.1f}, pitch {p:.1f}, roll {ro:.1f}, vertical fov {fov:.1f}")
    if G: print(f"eye {u - G(e, n):.2f} m above the ground there")
    print(f"range to hut {math.hypot(e, n):.1f} m, bearing from hut {math.degrees(math.atan2(e, n)) % 360:.0f}")
    print("point errors px:", " ".join(f"{v:.1f}" for v in err), f" rms {math.sqrt((err ** 2).mean()):.1f}")
    t = np.array([e, n, u]) + f * 30
    json.dump(dict(cam=[e, n, u], target=t.tolist(), vfov=fov, roll=ro, size=[w, h]), open(sys.argv[1].replace(".json", ".cam"), "w"))
    print(f"SHOT_CAM={e:.2f},{n:.2f},{u:.2f},{t[0]:.2f},{t[1]:.2f},{t[2]:.2f} SHOT_FOV={fov:.1f} SHOT_ROLL={ro:.1f}")

main()
