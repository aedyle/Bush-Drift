"""The plants of the route, built procedurally: blender -b --python tools/make_plants.py -- OUT.txt [RENDER_DIR]

Species and their looks come from docs/forest/species.md (NSW vegetation mapping for the corridor + PlantNET):
tall wet-forest eucalypts (messmate, brown barrel, mountain grey gum, narrow-leaved peppermint), snow gum for
the frost-hollow rim, blackwood and silver wattle, three shrubs, two tree ferns, ground ferns, tussocks,
Lomandra, fallen logs and leaf litter. Flat colours for the pixel look; a few versions of each so the forest
doesn't tile. Every plant is built twice: LOD 0 (near the road, near the car) and, for the ones that show
from afar, LOD 1 (a handful of triangles).

Nothing here is random at run time: each version has its own fixed seed, so the output is byte-identical on
every run. The geometry is plain Python (mathutils); Blender is used for the line-up renders.

OUT format, one plant after another, in the game's axes (x, y up, z; metres, base at the origin):
  plant NAME VARIANT LOD HEIGHT TRUNK_R NTRI ZONE_HINT
  t K R G B  ax ay az  bx by bz  cx cy cz      (K: b = bark/wood, l = leaves, g = ground cover)
"""
import sys, os, math, random
from mathutils import Vector, Matrix

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = args[0] if args else "assets-built/plants.txt"
RENDER_DIR = args[1] if len(args) > 1 else None

# ------------------------------------------------------------------ building blocks
class P:
    """One plant being built: triangles of (kind, colour, a, b, c), Blender axes (z up)."""
    def __init__(s): s.t = []
    def tri(s, k, col, a, b, c): s.t.append((k, tuple(max(0.0, min(1.0, x)) for x in col), Vector(a), Vector(b), Vector(c)))
    def quad(s, k, col, a, b, c, d): s.tri(k, col, a, b, c); s.tri(k, col, a, c, d)

def jit(col, rng, a=0.05):
    f = 1.0 + (rng.random() - 0.5) * 2 * a
    return (col[0] * f, col[1] * f, col[2] * f)

def mix(a, b, t): return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))

def frame(d):
    d = d.normalized()
    u = d.cross(Vector((0, 0, 1)) if abs(d.z) < 0.95 else Vector((1, 0, 0))).normalized()
    return u, d.cross(u).normalized()

def tube(p, pts, radii, sides, colfn, rng, rough=0.0, cap=True, kind="b", twist=0.0):
    """A tapering tube through pts (rings of `sides`), facet colour colfn(segment, side)."""
    rings = []
    for i, c in enumerate(pts):
        d = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)])
        u, w = frame(d)
        ring = []
        for k in range(sides):
            a = (k + 0.5 * (i % 2) * (1 if twist else 0)) * 2 * math.pi / sides + twist * i
            r = radii[i] * (1.0 + (rng.random() - 0.5) * 2 * rough)
            ring.append(c + (u * math.cos(a) + w * math.sin(a)) * r)
        rings.append(ring)
    for i in range(len(rings) - 1):
        for k in range(sides):
            k2 = (k + 1) % sides
            p.quad(kind, colfn(i, k), rings[i][k], rings[i][k2], rings[i + 1][k2], rings[i + 1][k])
    if cap and radii[-1] > 0.02:
        top = pts[-1] + (pts[-1] - pts[-2]).normalized() * radii[-1] * 0.6
        for k in range(sides):
            p.tri(kind, colfn(len(rings) - 2, k), rings[-1][k], rings[-1][(k + 1) % sides], top)

ICO_V = None
def ico():
    global ICO_V
    if ICO_V is None:
        t = (1 + 5 ** 0.5) / 2
        v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
        f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
        ICO_V = ([Vector(x).normalized() for x in v], f)
    return ICO_V

OCT = ([Vector(x) for x in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]],
       [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)])

def clump(p, c, rx, ry, rz, cols, rng, detail=1, jag=0.28, kind="l"):
    """A lumpy blob of foliage. cols: list of colours picked per face (light and dark leaves)."""
    vs, fs = ico() if detail else OCT
    yaw = rng.random() * 6.283
    m = Matrix.Rotation(yaw, 3, 'Z')
    pts = []
    for v in vs:
        q = m @ v
        f = 1.0 + (rng.random() - 0.5) * 2 * jag
        pts.append(c + Vector((q.x * rx * f, q.y * ry * f, q.z * rz * f)))
    for a, b, d in fs:
        p.tri(kind, jit(cols[rng.randrange(len(cols))], rng, 0.04), pts[a], pts[b], pts[d])

def strip(p, k, col, a, b, width, side):
    """A flat strip from a to b, `width` across, spread along `side`."""
    h = side.normalized() * width * 0.5
    p.quad(k, col, a - h, a + h, b + h, b - h)

def frond(p, base, d, length, width, droop, col, rng, segs=3, fold=0.35, kind="l"):
    """An arching fern frond / strap leaf: a strip folded along its midrib, tapering, drooping at the tip."""
    d = Vector((d.x, d.y, 0)).normalized() if d.length > 0 else Vector((1, 0, 0))
    side = d.cross(Vector((0, 0, 1))).normalized()
    rise = 0.55 + 0.3 * rng.random()
    prev_c, prev_w = base, width * 0.25
    for i in range(1, segs + 1):
        t = i / segs
        c = base + d * length * t + Vector((0, 0, length * (rise * t - droop * t * t)))
        w = width * (math.sin(math.pi * min(t * 1.15, 1.0)) * 0.9 + 0.1) * (1 - 0.6 * t)
        up = Vector((0, 0, w * fold))
        cl = jit(col, rng, 0.06)
        p.quad(kind, cl, prev_c, prev_c + side * prev_w + up * 0.5, c + side * w + up * 0.5, c)
        p.quad(kind, mix(cl, (0, 0, 0), 0.12), prev_c, c, c - side * w + up * 0.5, prev_c - side * prev_w + up * 0.5)
        prev_c, prev_w = c, w

def wobble_path(base, top, n, amp, rng):
    pts = []
    for i in range(n + 1):
        t = i / n
        q = base.lerp(top, t)
        if 0 < i < n:
            q += Vector(((rng.random() - 0.5) * 2 * amp, (rng.random() - 0.5) * 2 * amp, 0))
        pts.append(q)
    return pts

# ------------------------------------------------------------------ the eucalypts
GUMS = {
    # height range, base radius, rough bark up to (share of height), lower and upper bark, crown base, crown width (share of H), leaf colours
    "messmate":   dict(H=(36, 48), R=(0.45, 0.85), rough=1.0, lo=(0.40, 0.33, 0.27), up=(0.46, 0.39, 0.32), shag=0.16, cb=0.52, cw=0.26,
                       leaf=[(0.25, 0.34, 0.17), (0.32, 0.40, 0.20), (0.21, 0.29, 0.16)], ribbons=0, limbs=5),
    "brownbarrel": dict(H=(38, 52), R=(0.50, 0.95), rough=0.55, lo=(0.47, 0.34, 0.25), up=(0.86, 0.83, 0.75), shag=0.10, cb=0.58, cw=0.24,
                        leaf=[(0.26, 0.37, 0.18), (0.33, 0.43, 0.21), (0.22, 0.31, 0.16)], ribbons=5, limbs=5),
    "greygum":    dict(H=(32, 46), R=(0.40, 0.75), rough=0.06, lo=(0.52, 0.46, 0.40), up=(0.88, 0.87, 0.82), shag=0.04, cb=0.55, cw=0.25,
                       leaf=[(0.30, 0.38, 0.22), (0.37, 0.44, 0.26), (0.26, 0.33, 0.20)], ribbons=4, limbs=4,
                       patches=[(0.90, 0.88, 0.80), (0.80, 0.80, 0.74), (0.86, 0.80, 0.58), (0.74, 0.74, 0.72)]),
    "peppermint": dict(H=(20, 30), R=(0.30, 0.50), rough=0.62, lo=(0.50, 0.46, 0.41), up=(0.70, 0.70, 0.66), shag=0.08, cb=0.40, cw=0.30,
                       leaf=[(0.22, 0.32, 0.18), (0.27, 0.37, 0.20), (0.19, 0.28, 0.16)], ribbons=0, limbs=6),
    "snowgum":    dict(H=(8, 14), R=(0.30, 0.45), rough=0.0, lo=(0.80, 0.78, 0.70), up=(0.90, 0.88, 0.80), shag=0.05, cb=0.30, cw=0.55,
                       leaf=[(0.30, 0.42, 0.22), (0.36, 0.47, 0.24), (0.27, 0.36, 0.20)], ribbons=2, limbs=2, stems=(2, 3),
                       patches=[(0.92, 0.90, 0.82), (0.72, 0.74, 0.56), (0.80, 0.58, 0.46), (0.86, 0.86, 0.80)]),
}

def gum(name, v, lod):
    g = GUMS[name]
    rng = random.Random(hash_seed(name, v))
    H = g["H"][0] + (g["H"][1] - g["H"][0]) * [0.35, 0.7, 0.05, 1.0][v % 4]
    R = g["R"][0] + (g["R"][1] - g["R"][0]) * [0.4, 0.75, 0.15, 1.0][v % 4]
    if v == 3: H *= 1.12; R *= 1.25        # the big old tree
    p = P()
    nst = g.get("stems", (1, 1))[v % 2] if "stems" in g else 1
    stems = []
    for si in range(nst):
        ang = rng.random() * 6.283
        lean = 0.02 if nst == 1 else 0.18 + 0.1 * rng.random()
        top = Vector((math.cos(ang) * lean * H, math.sin(ang) * lean * H, H * (0.86 if nst == 1 else 0.75 + 0.1 * rng.random())))
        stems.append((Vector((0, 0, -0.6)), top, R if nst == 1 else R * (0.75 - 0.1 * si)))
    crown_pts = []
    for base, top, r0 in stems:
        nseg = 7 if lod == 0 else 2
        pts = wobble_path(base, top, nseg, 0.012 * H if lod == 0 else 0.0, rng)
        radii = []
        for i, q in enumerate(pts):
            t = i / nseg
            radii.append(r0 * (1.0 - 0.72 * t ** 1.3) * (1.45 if i == 0 else 1.0))
        sides = 7 if lod == 0 else 5
        patches = g.get("patches")
        pcol = {}
        def colfn(i, k, pts=pts, nseg=nseg):
            t = (i + 0.5) / nseg
            if t < g["rough"] * (0.9 + 0.2 * rng.random()) or (t < 0.08 and g["rough"] > 0.03):
                c = g["lo"]
            elif patches and lod == 0:
                c = pcol.setdefault((i, k), patches[rng.randrange(len(patches))])
            else:
                c = g["up"]
            return jit(c, rng, 0.06 if lod == 0 else 0.0)
        tube(p, pts, radii, sides, colfn, rng, rough=g["shag"] if lod == 0 else 0.0, cap=True)
        # limbs from the upper trunk, and foliage clumps at their ends
        nl = g["limbs"] if lod == 0 else 0
        cb = g["cb"] * H
        for li in range(nl):
            t = (cb + (top.z - cb) * (0.1 + 0.8 * li / max(nl - 1, 1))) / max(top.z, 1)
            at = base.lerp(top, min(t, 0.95))
            ang = li * 2.4 + rng.random() * 0.8
            out = g["cw"] * H * (0.55 + 0.45 * rng.random()) * (1.0 - 0.4 * li / max(nl, 1))
            end = at + Vector((math.cos(ang) * out, math.sin(ang) * out, out * (0.9 + 0.6 * rng.random())))
            mid = at.lerp(end, 0.5) + Vector((0, 0, out * 0.12))
            rr = r0 * 0.42 * (1 - 0.3 * li / nl)
            tube(p, [at, mid, end], [rr, rr * 0.6, rr * 0.25], 4, lambda i, k: jit(g["up"] if g["rough"] < 0.9 else g["lo"], rng, 0.05), rng, cap=False)
            crown_pts.append(end)
            crown_pts.append(mid + (end - at).normalized() * out * 0.1 + Vector((0, 0, out * 0.25)))
            crown_pts.append(end + Vector((math.cos(ang + 1.4), math.sin(ang + 1.4), 0)) * out * 0.35 + Vector((0, 0, -out * 0.25)))
            if g["ribbons"] and li < g["ribbons"] and lod == 0:   # bark hanging in ribbons from the forks
                side = Vector((-math.sin(ang), math.cos(ang), 0))
                L = 1.5 + 3.5 * rng.random()
                strip(p, "b", jit((0.72, 0.58, 0.44), rng, 0.08), at + side * rr * 1.2, at + side * rr * 1.4 + Vector((0, 0, -L)), 0.22, side.cross(Vector((0, 0, 1))))
        crown_pts.append(top + Vector((0, 0, 0.04 * H)))
    if lod == 0:
        for q in crown_pts:
            s = H * (0.055 + 0.03 * rng.random()) * (1.4 if name == "snowgum" else 1.0)
            clump(p, q, s * 1.1, s * 1.0, s * 0.85, g["leaf"], rng, detail=1, jag=0.35)
    else:   # three blobs where the crown is
        top = stems[0][1]
        cb = g["cb"] * H
        for k in range(3):
            ang = k * 2.09 + rng.random()
            off = g["cw"] * H * 0.45
            c = Vector((math.cos(ang) * off + top.x * 0.7, math.sin(ang) * off + top.y * 0.7, cb + (top.z - cb) * (0.55 + 0.25 * rng.random())))
            s = g["cw"] * H * 0.5
            clump(p, c, s, s, s * 0.6, g["leaf"][:1], rng, detail=0)
    return p, H, R

# ------------------------------------------------------------------ midstorey trees and shrubs
def broadleaf(name, v, lod):
    """Blackwood, silver wattle and the gully small tree (Pomaderris / Olearia / Bedfordia)."""
    rng = random.Random(hash_seed(name, v))
    if name == "blackwood":
        H = [11, 14, 9, 17][v % 4]; R = 0.16 + 0.03 * H / 10; trunk = (0.24, 0.22, 0.20); cb = 0.28
        leaf = [(0.17, 0.25, 0.14), (0.21, 0.29, 0.16), (0.15, 0.22, 0.13)]; shape = (0.24, 0.62); n = 7; stems = 1
    elif name == "hutwood":   # session 3b: the old blackwoods round the hut, grown in the open: short trunk, dense round crown nearly to the ground
        H = [9.5, 11, 8.5, 12][v % 4]; R = 0.22 + 0.03 * H / 10; trunk = (0.22, 0.20, 0.18); cb = 0.13
        leaf = [(0.15, 0.24, 0.12), (0.19, 0.28, 0.14), (0.13, 0.21, 0.11)]; shape = (0.30, 0.70); n = 11; stems = 1
    elif name == "wattle":
        H = [8, 10, 6.5, 12][v % 4]; R = 0.12 + 0.02 * H / 10; trunk = (0.52, 0.55, 0.50); cb = 0.35
        leaf = [(0.42, 0.53, 0.44), (0.50, 0.60, 0.50), (0.36, 0.46, 0.38)]; shape = (0.36, 0.5); n = 8; stems = 1
    else:  # gullytree
        H = [4.5, 6.0, 3.6, 7.0][v % 4]; R = 0.07; trunk = (0.40, 0.36, 0.30); cb = 0.30
        leaf = [(0.20, 0.32, 0.15), (0.25, 0.37, 0.18), (0.55, 0.62, 0.50)]; shape = (0.36, 0.55); n = 6; stems = 3
    p = P()
    top_all = []
    for si in range(stems):
        ang = rng.random() * 6.283
        lean = 0.04 if stems == 1 else 0.16
        top = Vector((math.cos(ang) * lean * H, math.sin(ang) * lean * H, H * (0.78 if stems == 1 else 0.85)))
        nseg = 4 if lod == 0 else 1
        pts = wobble_path(Vector((0, 0, -0.3)), top, nseg, 0.02 * H if lod == 0 else 0, rng)
        rr = R if stems == 1 else R * 0.8
        tube(p, pts, [rr * (1 - 0.75 * i / nseg) for i in range(nseg + 1)], 5 if lod == 0 else 4, lambda i, k: jit(trunk, rng, 0.05 if lod == 0 else 0), rng, rough=0.08 if lod == 0 else 0)
        top_all.append(top)
    cz0, cz1 = cb * H, H
    if lod == 0:
        for k in range(n):
            t = (k + 0.5) / n
            ang = k * 2.4 + rng.random()
            z = cz0 + (cz1 - cz0) * (0.15 + 0.75 * rng.random())
            prof = math.sin(math.pi * (z - cz0) / (cz1 - cz0)) ** 0.7     # widest in the middle of the crown
            rad = shape[0] * H * prof * (0.4 + 0.6 * rng.random())
            c = Vector((math.cos(ang) * rad, math.sin(ang) * rad, z)) + top_all[k % stems] * Vector((0.5, 0.5, 0))
            s = shape[0] * H * (0.42 + 0.2 * rng.random())
            clump(p, c, s, s, s * shape[1] * 1.3, leaf, rng, detail=1)
        clump(p, Vector((top_all[0].x, top_all[0].y, cz0 + (cz1 - cz0) * 0.55)), shape[0] * H * 0.75, shape[0] * H * 0.75, (cz1 - cz0) * 0.45, leaf, rng, detail=1)
    else:
        c = Vector((top_all[0].x * 0.6, top_all[0].y * 0.6, cz0 + (cz1 - cz0) * 0.5))
        clump(p, c, shape[0] * H * 1.0, shape[0] * H * 1.0, (cz1 - cz0) * 0.55, leaf[:1], rng, detail=0)
    return p, H, R if stems == 1 else R * 0.8

def shrub(name, v):
    rng = random.Random(hash_seed(name, v))
    p = P()
    if name == "heath":   # lance beard-heath: a bushy, erect mid-green shrub
        H = [1.6, 2.2, 1.1][v % 3]
        cols = [(0.27, 0.40, 0.18), (0.33, 0.46, 0.21), (0.24, 0.35, 0.17)]
        for k in range(4):
            ang = k * 1.7 + rng.random()
            c = Vector((math.cos(ang) * 0.22 * H, math.sin(ang) * 0.22 * H, H * (0.35 + 0.35 * rng.random())))
            clump(p, c, 0.32 * H, 0.32 * H, 0.38 * H, cols, rng, detail=1 if k < 2 else 0)
    elif name == "coprosma":  # prickly currant bush: twiggy, see-through, small olive leaves
        H = [1.8, 2.4, 1.3][v % 3]
        for k in range(7):
            ang = k * 0.9 + rng.random() * 0.5
            a = Vector((0, 0, 0)); b = Vector((math.cos(ang) * 0.45 * H, math.sin(ang) * 0.45 * H, H * (0.7 + 0.3 * rng.random())))
            side = Vector((-math.sin(ang), math.cos(ang), 0))
            strip(p, "l", jit((0.36, 0.30, 0.24), rng, 0.05), a, b, 0.05, side)
            for q in range(2):
                clump(p, a.lerp(b, 0.55 + 0.4 * q), 0.16 * H, 0.16 * H, 0.12 * H, [(0.33, 0.39, 0.21), (0.40, 0.44, 0.24)], rng, detail=0)
    else:  # bracken: stiff erect stalks, broad triangular blades held flat-ish
        H = [1.0, 1.3, 0.75][v % 3]
        for k in range(7 if v != 2 else 5):
            ang = k * 0.9 + rng.random() * 0.6
            d = Vector((math.cos(ang), math.sin(ang), 0))
            side = d.cross(Vector((0, 0, 1)))
            base = d * 0.15 * rng.random()
            top = base + d * 0.25 * H + Vector((0, 0, H * (0.75 + 0.25 * rng.random())))
            dead = rng.random() < 0.18
            col = jit((0.55, 0.42, 0.26) if dead else (0.25, 0.42, 0.17), rng, 0.07)
            strip(p, "l", mix(col, (0.3, 0.25, 0.15), 0.3), base, top, 0.03, side)
            tip = top + d * 0.65 * H + Vector((0, 0, -0.12 * H))
            p.tri("l", col, top + side * 0.35 * H, tip, top - side * 0.35 * H)
            p.tri("l", mix(col, (0, 0, 0), 0.15), top + side * 0.35 * H, top - side * 0.35 * H, top - d * 0.08 * H + Vector((0, 0, 0.05 * H)))
    return p, H, 0.0

def treefern(name, v, lod):
    rng = random.Random(hash_seed(name, v))
    p = P()
    if name == "softfern":   # Dicksonia: stout, hairy red-brown trunk, broad dark glossy crown
        H = [2.4, 3.6, 1.6, 4.4][v % 4]; R = 0.19; trunk = (0.33, 0.22, 0.16); nf = 12; L = 2.3; col = (0.18, 0.36, 0.15)
    else:                    # Cyathea: taller, slimmer, knobbly; a more open crown
        H = [4.0, 5.5, 3.0, 6.8][v % 4]; R = 0.13; trunk = (0.27, 0.23, 0.20); nf = 10; L = 2.7; col = (0.21, 0.40, 0.16)
    lean = Vector(((rng.random() - 0.5) * 0.25 * H, (rng.random() - 0.5) * 0.25 * H, H))
    nseg = 4 if lod == 0 else 1
    pts = wobble_path(Vector((0, 0, -0.2)), lean, nseg, 0.05 if lod == 0 else 0, rng)
    tube(p, pts, [R * (1.25 if i == 0 else 1.0) for i in range(nseg + 1)], 6 if lod == 0 else 4,
         lambda i, k: jit(mix(trunk, (0.45, 0.28, 0.18), i / nseg) if name == "softfern" else trunk, rng, 0.06 if lod == 0 else 0), rng,
         rough=0.12 if (lod == 0 and name != "softfern") else 0.03 if lod == 0 else 0)
    if lod == 0:
        for k in range(nf):
            ang = k * 2 * math.pi / nf + rng.random() * 0.3
            d = Vector((math.cos(ang), math.sin(ang), 0))
            frond(p, lean + d * 0.12, d, L * (0.8 + 0.3 * rng.random()), 0.42, 1.0 + 0.4 * rng.random(), col, rng, segs=3)
        for k in range(2):   # young fronds, paler, standing up
            ang = rng.random() * 6.283
            d = Vector((math.cos(ang), math.sin(ang), 0))
            frond(p, lean, d, L * 0.5, 0.25, -0.6, mix(col, (0.55, 0.65, 0.30), 0.45), rng, segs=2)
    else:
        for k in range(4):
            ang = k * 1.571 + rng.random() * 0.4
            d = Vector((math.cos(ang), math.sin(ang), 0))
            a = lean; b = lean + d * L * 0.9 + Vector((0, 0, -0.35))
            side = d.cross(Vector((0, 0, 1)))
            p.quad("l", col, a - side * 0.05, a + side * 0.05, b + side * 0.6, b - side * 0.6)
    return p, H + 0.6, R

def groundfern(v):  # mother shield fern / Blechnum: a low arching rosette for the damp gullies
    rng = random.Random(hash_seed("shieldfern", v))
    p = P()
    n = [9, 11, 7][v % 3]
    for k in range(n):
        ang = k * 2 * math.pi / n + rng.random() * 0.4
        d = Vector((math.cos(ang), math.sin(ang), 0))
        frond(p, d * 0.04, d, 0.55 + 0.25 * rng.random(), 0.16, 0.9, (0.20, 0.38, 0.16), rng, segs=2)
    return p, 0.5, 0.0

def tussock(name, v):
    rng = random.Random(hash_seed(name, v))
    p = P()
    if name == "poa":          # Poa: fine, grey-green, a fountain of blades
        n, L, cols, w = 26, [0.5, 0.65, 0.4][v % 3], [(0.52, 0.56, 0.36), (0.60, 0.62, 0.40), (0.45, 0.50, 0.32), (0.70, 0.66, 0.46)], 0.035
    elif name == "kangaroo":   # Themeda: bronze, straw and red in the open hollow
        n, L, cols, w = 22, [0.6, 0.8, 0.5][v % 3], [(0.70, 0.52, 0.32), (0.62, 0.44, 0.28), (0.76, 0.62, 0.40), (0.55, 0.52, 0.30)], 0.04
    else:                      # Lomandra: bright green strap leaves, arching
        n, L, cols, w = 14, [0.8, 1.0, 0.65][v % 3], [(0.26, 0.46, 0.18), (0.32, 0.52, 0.21), (0.22, 0.40, 0.16)], 0.06
    for k in range(n):
        ang = rng.random() * 6.283
        d = Vector((math.cos(ang), math.sin(ang), 0))
        tilt = 0.25 + 0.6 * rng.random()
        side = d.cross(Vector((0, 0, 1)))
        base = d * 0.06 * rng.random()
        length = L * (0.7 + 0.4 * rng.random())
        mid = base + d * length * tilt * 0.5 + Vector((0, 0, length * 0.6))
        tip = base + d * length * tilt * 1.2 + Vector((0, 0, length * (0.75 - 0.35 * tilt)))
        col = jit(cols[rng.randrange(len(cols))], rng, 0.05)
        if name == "lomandra":
            p.quad("g", col, base - side * w, base + side * w, mid + side * w, mid - side * w)
            p.tri("g", mix(col, (0, 0, 0), 0.1), mid - side * w, mid + side * w, tip)
        else:
            p.tri("g", col, base - side * w, base + side * w, tip)
    if name == "kangaroo":     # a few seed-head stalks above the leaves
        for k in range(4):
            ang = rng.random() * 6.283
            top = Vector((math.cos(ang) * 0.2, math.sin(ang) * 0.2, L * 1.25))
            strip(p, "g", (0.60, 0.47, 0.32), Vector((0, 0, 0)), top, 0.025, Vector((-math.sin(ang), math.cos(ang), 0)))
    return p, L, 0.0

def log(v, lod):
    rng = random.Random(hash_seed("log", v))
    p = P()
    L = [7.0, 11.0, 4.5][v % 3]; R = [0.35, 0.5, 0.25][v % 3]
    nseg = 4 if lod == 0 else 1
    pts = [Vector((-L / 2 + L * i / nseg, (rng.random() - 0.5) * 0.3 if 0 < i < nseg else 0, R * 0.55)) for i in range(nseg + 1)]
    grey = (0.56, 0.53, 0.48)
    tube(p, pts, [R * (1 - 0.35 * i / nseg) for i in range(nseg + 1)], 6 if lod == 0 else 4,
         lambda i, k: jit(mix(grey, (0.34, 0.30, 0.26), 0.6) if k in (4, 5) else grey, rng, 0.06), rng, rough=0.08 if lod == 0 else 0, cap=False)
    if lod == 0:
        for e, sgn in ((pts[0], -1), (pts[-1], 1)):   # broken, splintered ends
            r = R if sgn < 0 else R * 0.65
            for k in range(5):
                a = k * 1.2566
                q = e + Vector((0, math.cos(a) * r * 0.8, math.sin(a) * r * 0.8))
                p.tri("b", (0.62, 0.52, 0.40), e + Vector((sgn * (0.2 + 0.5 * rng.random()), 0, 0)), q, e + Vector((0, math.cos(a + 1.2566) * r * 0.8, math.sin(a + 1.2566) * r * 0.8)))
        for k in range(2):   # snapped branch stubs
            at = pts[1 + k]
            ang = rng.random() * 3.0
            tube(p, [at, at + Vector((0.4, math.cos(ang) * 1.1, 0.6 + math.sin(ang) * 0.5))], [R * 0.3, R * 0.15], 4, lambda i, k: grey, rng, cap=True)
    return p, R * 1.1, R

def litter(v):
    """Leaf litter and shed bark: flat ribbons and leaf patches lying on the ground (never in the way)."""
    rng = random.Random(hash_seed("litter", v))
    p = P()
    for k in range(9):
        a = Vector(((rng.random() - 0.5) * 2.4, (rng.random() - 0.5) * 2.4, 0.03 + 0.02 * k / 9))
        ang = rng.random() * 3.14
        d = Vector((math.cos(ang), math.sin(ang), 0))
        if k < 4:   # a curl of shed bark
            strip(p, "g", jit((0.70, 0.56, 0.42), rng, 0.08), a, a + d * (0.8 + 1.2 * rng.random()), 0.14, d.cross(Vector((0, 0, 1))))
        else:       # a patch of fallen leaves
            col = jit([(0.45, 0.30, 0.20), (0.55, 0.40, 0.25), (0.38, 0.27, 0.18)][k % 3], rng, 0.06)
            r = 0.35 + 0.3 * rng.random()
            pts = [a + Vector((math.cos(t * 1.2566 + ang) * r * (0.7 + 0.5 * rng.random()), math.sin(t * 1.2566 + ang) * r * (0.7 + 0.5 * rng.random()), 0)) for t in range(5)]
            for t in range(1, 4):
                p.tri("g", col, pts[0], pts[t], pts[t + 1])
    return p, 0.05, 0.0

def hash_seed(name, v):
    h = 2166136261
    for ch in f"{name}/{v}":
        h = ((h ^ ord(ch)) * 16777619) & 0xffffffff
    return h

# ------------------------------------------------------------------ the catalogue
# (name, variants, builder(v, lod), lods, zone hint: A hollow, B plateau forest, C gullies)
CATALOGUE = [
    ("messmate", 4, lambda v, l: gum("messmate", v, l), (0, 1), "B"),
    ("brownbarrel", 4, lambda v, l: gum("brownbarrel", v, l), (0, 1), "AB"),
    ("greygum", 4, lambda v, l: gum("greygum", v, l), (0, 1), "C"),
    ("peppermint", 3, lambda v, l: gum("peppermint", v, l), (0, 1), "B"),
    ("snowgum", 3, lambda v, l: gum("snowgum", v, l), (0, 1), "A"),
    ("blackwood", 3, lambda v, l: broadleaf("blackwood", v, l), (0, 1), "BC"),
    ("wattle", 3, lambda v, l: broadleaf("wattle", v, l), (0, 1), "BC"),
    ("hutwood", 3, lambda v, l: broadleaf("hutwood", v, l), (0, 1), "H"),
    ("gullytree", 3, lambda v, l: broadleaf("gullytree", v, l), (0, 1), "C"),
    ("softfern", 3, lambda v, l: treefern("softfern", v, l), (0, 1), "C"),
    ("roughfern", 3, lambda v, l: treefern("roughfern", v, l), (0, 1), "C"),
    ("heath", 3, lambda v, l: shrub("heath", v), (0,), "B"),
    ("coprosma", 3, lambda v, l: shrub("coprosma", v), (0,), "BC"),
    ("bracken", 3, lambda v, l: shrub("bracken", v), (0,), "ABC"),
    ("shieldfern", 3, lambda v, l: groundfern(v), (0,), "C"),
    ("poa", 3, lambda v, l: tussock("poa", v), (0,), "ABC"),
    ("kangaroo", 3, lambda v, l: tussock("kangaroo", v), (0,), "A"),
    ("lomandra", 3, lambda v, l: tussock("lomandra", v), (0,), "BC"),
    ("log", 3, lambda v, l: log(v, l), (0, 1), "BC"),
    ("litter", 3, lambda v, l: litter(v), (0,), "BC"),
]

def f3(v): return f"{v.x:.3f} {v.z:.3f} {v.y:.3f}"   # Blender (x, y, z up) -> game (x, y up, z)

built = []
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
with open(OUT, "w") as f:
    f.write("# Bush Drift plants, made by tools/make_plants.py (procedural; see docs/forest/species.md)\n")
    for name, nv, fn, lods, zone in CATALOGUE:
        for v in range(nv):
            for lod in lods:
                p, H, R = fn(v, lod)
                f.write(f"plant {name} {v} {lod} {H:.3f} {R:.3f} {len(p.t)} {zone}\n")
                for k, c, a, b, d in p.t:
                    f.write(f"t {k} {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}  {f3(a)}  {f3(b)}  {f3(d)}\n")
                built.append((name, v, lod, H, R, p))
                print(f"PLANT {name:12s} v{v} lod{lod}: {len(p.t):4d} triangles, {H:5.1f} m tall, trunk r {R:.2f}")

# ------------------------------------------------------------------ line-up renders (docs)
if RENDER_DIR:
    import bpy
    os.makedirs(RENDER_DIR, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene

    def add(p, at, name):
        me = bpy.data.meshes.new(name)
        verts, faces = [], []
        for k, c, a, b, d in p.t:
            faces.append((len(verts), len(verts) + 1, len(verts) + 2)); verts += [tuple(a + at), tuple(b + at), tuple(d + at)]
        me.from_pydata(verts, [], faces)
        attr = me.color_attributes.new("Col", 'FLOAT_COLOR', 'CORNER')
        cols = []
        for k, c, a, b, d in p.t:
            cols += [(*c, 1.0)] * 3
        attr.data.foreach_set("color", [x for c in cols for x in c])
        me.color_attributes.active_color = attr
        o = bpy.data.objects.new(name, me); sc.collection.objects.link(o)
        return o

    def label(text, at, size):
        cu = bpy.data.curves.new("t", 'FONT'); cu.body = text; cu.size = size; cu.align_x = 'CENTER'
        o = bpy.data.objects.new("t", cu); o.location = at; o.rotation_euler = (math.radians(90), 0, 0)
        sc.collection.objects.link(o)
        m = bpy.data.materials.new("ink"); m.diffuse_color = (0.15, 0.12, 0.1, 1); cu.materials.append(m)

    def ground(x0, x1, col):
        me = bpy.data.meshes.new("g"); me.from_pydata([(x0, -30, 0), (x1, -30, 0), (x1, 30, 0), (x0, 30, 0)], [], [(0, 1, 2, 3)])
        o = bpy.data.objects.new("g", me); sc.collection.objects.link(o)
        m = bpy.data.materials.new("g"); m.diffuse_color = (*col, 1); me.materials.append(m)

    def shoot(path, cam_at, target, ortho):
        cam = bpy.data.cameras.new("c"); cam.type = 'ORTHO'; cam.ortho_scale = ortho
        co = bpy.data.objects.new("c", cam); sc.collection.objects.link(co); sc.camera = co
        co.location = cam_at; co.rotation_euler = (target - cam_at).to_track_quat('-Z', 'Y').to_euler()
        sc.render.engine = 'BLENDER_WORKBENCH'
        sh = sc.display.shading
        sh.light = 'STUDIO'; sh.color_type = 'VERTEX'; sh.show_object_outline = False
        sh.show_shadows = True
        sc.display.light_direction = (-0.6, -0.4, 0.7)
        sc.world = bpy.data.worlds.new("w"); sc.world.color = (0.93, 0.80, 0.62)
        sc.render.filepath = path
        bpy.ops.render.render(write_still=True)
        bpy.data.objects.remove(co)

    def lineup(names, spacing, size_txt, path, res, ortho, cam_y, cam_z, tz, lod=0, v=0):
        for o in list(sc.objects): bpy.data.objects.remove(o)
        x = 0.0
        xs = []
        for nm in names:
            hit = [b for b in built if b[0] == nm and b[1] == v and b[2] == lod]
            if not hit: continue
            p = hit[0][5]
            add(p, Vector((x, 0, 0)), nm)
            label(nm, Vector((x, -6, 0.05)), size_txt)
            xs.append(x); x += spacing
        ground(-spacing, x, (0.62, 0.56, 0.36))
        sc.render.resolution_x, sc.render.resolution_y = res
        for o in sc.objects:
            if o.type == 'FONT': o.location.y = -spacing * 0.8
        mid = (xs[0] + xs[-1]) / 2
        shoot(path, Vector((mid, cam_y, cam_z)), Vector((mid, 0, tz)), ortho)

    big = ["messmate", "brownbarrel", "greygum", "peppermint", "snowgum", "blackwood", "wattle"]
    lineup(big, 22, 2.4, os.path.join(RENDER_DIR, "lineup-trees.png"), (1800, 900), 165, -200, 26, 24)
    lineup(["gullytree", "softfern", "roughfern", "heath", "coprosma", "log"], 7.0, 0.55, os.path.join(RENDER_DIR, "lineup-small.png"), (1800, 700), 46, -60, 6, 2.6)
    lineup(["bracken", "shieldfern", "poa", "kangaroo", "lomandra", "litter"], 2.6, 0.22, os.path.join(RENDER_DIR, "lineup-ground.png"), (1800, 600), 17, -30, 2.5, 0.4)
    # every version of the three main gums, near and far detail
    for o in list(sc.objects): bpy.data.objects.remove(o)
    x = 0
    for nm in ["messmate", "brownbarrel", "greygum"]:
        for v in range(4):
            for lod in (0, 1):
                hit = [b for b in built if b[0] == nm and b[1] == v and b[2] == lod][0]
                add(hit[5], Vector((x, lod * 40, 0)), nm)
            x += 18
    ground(-18, x, (0.62, 0.56, 0.36))
    sc.render.resolution_x, sc.render.resolution_y = 1800, 800
    shoot(os.path.join(RENDER_DIR, "variants-gums.png"), Vector((x / 2 - 9, -260, 60)), Vector((x / 2 - 9, 20, 22)), 230)
    print("RENDERS", RENDER_DIR)
