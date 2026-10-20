"""Where marked photo pixels land on the ground: python tools/photomatch/ground_pts.py NAME x,y [x,y ...]
Uses tools/photomatch/NAME.cam (from solve_cam.py) and the game's DEM. Prints E, N (m from the hut) and range."""
import json, math, sys
import numpy as np
sys.argv, args = sys.argv[:2] + ["-"], sys.argv[2:]
exec(open("tools/photomatch/solve_cam.py").read().split("def main")[0])
C = json.load(open(f"tools/photomatch/{sys.argv[1]}.cam")); w, h = C["size"]
e, n, u = C["cam"]; te, tn, tu = C["target"]
b = math.degrees(math.atan2(te - e, tn - n)); p = math.degrees(math.atan2(tu - u, math.hypot(te - e, tn - n)))
f, rt, up = rot(b, p, C["roll"]); fx = (h / 2) / math.tan(math.radians(C["vfov"]) / 2); G = ground_fn()
for a in args:
    px, py = map(float, a.split(","))
    d = f + rt * (px - w / 2) / fx - up * (py - h / 2) / fx
    lo, hi = 0.5, 400.0
    for _ in range(60):
        m = (lo + hi) / 2; q = np.array([e, n, u]) + d * m
        if q[2] > G(q[0], q[1]): lo = m
        else: hi = m
    q = np.array([e, n, u]) + d * lo
    print(f"{a:>10s} -> E {q[0]:6.1f} N {q[1]:6.1f}  ({lo:.1f} m from the camera)")
