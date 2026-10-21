"""Build B's models, made procedurally like the plants: blender -b --python tools/make_world.py -- OUT.txt [RENDER_DIR]

- sambar (Rusa unicolor): the deer of the wet eucalypt forests of south-eastern Australia, and one of the two
  species (with fallow) on the NSW Far South Coast (docs/forest/deer.md). A big, dark, shaggy-necked deer: hinds
  about 1.2 m at the shoulder, stags 1.35 m with three-tined antlers. Four poses: hind standing (head up), hind
  grazing, stag standing with its head turned, stag grazing. Static: they only stand and graze.
- edgescrub: the dense scrub on the face of the map's edge (a bank of blackwood, wattle and dogwood-ish bushes).
- boomgate: the locked gate across Cattlemans Link Trail at the junction.
- armyman: a life-size green plastic toy soldier, the classic sniper pose, hidden in the bush aiming at a deer.

Same output format and axes as tools/make_plants.py (plant NAME VARIANT LOD HEIGHT TRUNK_R NTRI ZONE, then t lines);
fixed seeds, so the output is byte-identical on every run. Blender is used only for the renders.
"""
import sys, os, math, random
from mathutils import Vector, Matrix

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = args[0] if args else "assets-built/world.txt"
RENDER_DIR = args[1] if len(args) > 1 else None


class P:
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


def tube(p, pts, radii, sides, colfn, squash=1.0, cap_end=True, cap_start=False, kind="b"):
    """A tube through pts; radii per point (a number, or (across, up) for an oval); facet colour colfn(seg, side)."""
    rings = []
    for i, c in enumerate(pts):
        d = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
        u, w = frame(d)
        r = radii[i] if isinstance(radii[i], tuple) else (radii[i], radii[i] * squash)
        rings.append([c + u * math.cos(a) * r[0] + w * math.sin(a) * r[1] for a in (k * 2 * math.pi / sides for k in range(sides))])
    for i in range(len(rings) - 1):
        for k in range(sides):
            k2 = (k + 1) % sides
            p.quad(kind, colfn(i, k), rings[i][k], rings[i][k2], rings[i + 1][k2], rings[i + 1][k])
    if cap_end:
        top = pts[-1] + (pts[-1] - pts[-2]).normalized() * 0.02
        for k in range(sides): p.tri(kind, colfn(len(rings) - 2, k), rings[-1][k], rings[-1][(k + 1) % sides], top)
    if cap_start:
        bot = pts[0] - (pts[1] - pts[0]).normalized() * 0.02
        for k in range(sides): p.tri(kind, colfn(0, k), rings[0][(k + 1) % sides], rings[0][k], bot)


# ------------------------------------------------------------------ the sambar
COAT = (0.46, 0.33, 0.21)       # dark brown (lifted a little: the game's light is dim under the canopy)
COAT_DARK = (0.27, 0.20, 0.14)  # legs, mane, the dark line down the back
BELLY = (0.55, 0.43, 0.29)
RUMP = (0.66, 0.46, 0.26)       # the chestnut under the tail
NOSE = (0.10, 0.08, 0.07)
ANTLER = (0.58, 0.50, 0.38)


def bez(a, b, c, t):
    return a * (1 - t) ** 2 + b * 2 * t * (1 - t) + c * t * t


def sambar(v, lod):
    """v: 0 hind standing, 1 hind grazing, 2 stag head turned, 3 stag grazing. +x is forward, z up."""
    rng = random.Random(9100 + v * 17 + lod)
    stag = v >= 2
    grazing = v in (1, 3)
    k = 1.12 if stag else 1.0                       # stags are bigger
    sh = 1.2 * k                                     # shoulder height
    p = P()
    coat = lambda i, s: jit(COAT_DARK if s in (1, 2) and lod == 0 else (BELLY if s in (4, 5) else COAT), rng, 0.05)
    # body: rump (-0.85) to chest (+0.75), an oval a little taller than wide, the back slightly lower than the withers
    sides = 6 if lod == 0 else 4
    xs = [-0.95, -0.7, -0.2, 0.35, 0.72, 0.88]
    hz = [0.98, 1.02, 0.98, 1.0, 1.04, 0.98]
    rr = [(0.13, 0.14), (0.27, 0.30), (0.30, 0.34), (0.30, 0.35), (0.26, 0.33), (0.12, 0.16)]
    if lod:
        xs, hz, rr = [xs[0], xs[2], xs[4], xs[5]], [hz[0], hz[2], hz[4], hz[5]], [rr[0], rr[2], rr[4], rr[5]]
    pts = [Vector((x * k, 0, h * sh * 0.8)) for x, h in zip(xs, hz)]
    tube(p, pts, [(a * k, b * k) for a, b in rr], sides, coat, cap_end=True, cap_start=True)
    # rump patch and tail (bushy, dark on top)
    if lod == 0:
        r0 = Vector((-0.97 * k, 0, 0.84 * sh))
        tube(p, [r0, r0 + Vector((-0.08, 0, -0.22 * k))], [0.06 * k, 0.04 * k], 4, lambda i, s: COAT_DARK if s == 0 else RUMP)
    # neck and head
    base = Vector((0.75 * k, 0, 0.98 * sh))
    yaw = math.radians(38 if v == 2 else 0)
    if grazing:
        mid, head0 = Vector((1.05 * k, 0, 0.7 * sh)), Vector((1.15 * k, 0, 0.32))
        head_dir = Vector((0.35, 0, -1.0)).normalized()
    else:
        mid, head0 = Vector((0.98 * k, 0, 1.25 * sh)), Vector((1.05 * k, 0, 1.52 * sh))
        head_dir = Vector((0.95, 0, -0.45)).normalized()
    rot = Matrix.Rotation(yaw, 3, 'Z')
    turn = lambda q: base + rot @ (q - base)
    nseg = 4 if lod == 0 else 2
    neck = [turn(bez(base, mid, head0, i / nseg)) for i in range(nseg + 1)]
    nr = [0.21 * k, 0.19 * k, 0.16 * k, 0.13 * k, 0.11 * k][:nseg + 1] if lod == 0 else [0.2 * k, 0.15 * k, 0.11 * k]
    tube(p, neck, nr, 5 if lod == 0 else 3, lambda i, s: jit(COAT_DARK if lod == 0 and s in (1, 2) else COAT, rng, 0.06), squash=1.1, cap_end=False)
    hd = (rot @ head_dir).normalized()
    hlen = 0.42 * k
    hpts = [neck[-1], neck[-1] + hd * hlen * 0.55, neck[-1] + hd * hlen]
    hr = [0.10 * k, 0.075 * k, 0.04 * k] if lod == 0 else [0.09 * k, 0.06 * k, 0.03 * k]
    if lod: hpts, hr = [hpts[0], hpts[2]], [hr[0], hr[2]]
    tube(p, hpts, hr, 5 if lod == 0 else 3, lambda i, s: jit(NOSE if i == len(hpts) - 2 and lod == 0 else COAT, rng, 0.05))
    # ears: big and rounded, out to the sides and up
    side = Vector((-hd.y, hd.x, 0)).normalized() if abs(hd.z) < 0.99 else Vector((0, 1, 0))
    up = hd.cross(side).normalized()
    if up.z < 0: up = -up
    for sgn in (-1, 1):
        e0 = neck[-1] + up * 0.06 * k + side * sgn * 0.05 * k
        e1 = e0 + (side * sgn * 0.75 + up * 0.55 - hd * 0.2).normalized() * 0.2 * k
        w = (hd * 0.07 + up * 0.0) * k
        p.quad("b", jit(COAT, rng), e0 - w, e0 + w, e1 + w * 0.6, e1 - w * 0.6)
    # antlers: a beam each side, back and up, a brow tine forward and a fork at the top (three tines)
    if stag:
        for sgn in (-1, 1):
            a0 = neck[-1] + up * 0.09 * k + side * sgn * 0.05 * k
            a1 = a0 + (up * 1.0 - hd * 0.35 + side * sgn * 0.35).normalized() * 0.32 * k
            a2 = a1 + (up * 1.0 - hd * 0.25 + side * sgn * 0.25).normalized() * 0.28 * k
            col = lambda i, s: jit(ANTLER, rng, 0.06)
            sides_a = 3
            tube(p, [a0, a1, a2], [0.025, 0.02, 0.012], sides_a, col)
            brow = a0 + (up * 0.15) * k
            tube(p, [brow, brow + (hd * 1.0 + up * 0.5).normalized() * 0.16 * k], [0.018, 0.008], sides_a, col)
            fork = a1 + (up * 0.6 + hd * 0.6 + side * sgn * 0.2).normalized() * 0.2 * k
            tube(p, [a1, fork], [0.018, 0.008], sides_a, col)
    # legs: shoulder/hip, knee or hock, hoof
    leg_sides = 4 if lod == 0 else 3
    for fx, sgn in ((0.55, -1), (0.55, 1), (-0.7, -1), (-0.7, 1)):
        hind = fx < 0
        top = Vector((fx * k, sgn * 0.14 * k, 0.75 * sh))
        stride = (0.06 if sgn > 0 else -0.04) * (1 if not hind else -1)
        knee = Vector(((fx + (-0.1 if hind else 0.02) + stride) * k, sgn * 0.13 * k, 0.42 * sh))
        hoof = Vector(((fx + (-0.02 if hind else 0.04) + stride * 1.6) * k, sgn * 0.12 * k, 0.0))
        lp = [top, knee, hoof] if lod == 0 else [top, hoof]
        lr = [0.08 * k, 0.045 * k, 0.03 * k] if lod == 0 else [0.07 * k, 0.03 * k]
        tube(p, lp, lr, leg_sides, lambda i, s: jit(COAT_DARK, rng, 0.05), cap_end=False)
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.0


# ------------------------------------------------------------------ the edge's scrub
LEAF = [(0.20, 0.27, 0.12), (0.24, 0.31, 0.14), (0.17, 0.23, 0.11), (0.28, 0.33, 0.16)]


def clump(p, c, rx, ry, rz, cols, rng, detail):
    t = (1 + 5 ** 0.5) / 2
    if detail:
        vs = [Vector(x).normalized() for x in [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]]
        fs = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
              (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    else:
        vs = [Vector(x) for x in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]]
        fs = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]
    m = Matrix.Rotation(rng.random() * 6.283, 3, 'Z')
    pts = [c + Vector(((m @ q).x * rx * (1 + (rng.random() - 0.5) * 0.5), (m @ q).y * ry * (1 + (rng.random() - 0.5) * 0.5), (m @ q).z * rz * (1 + (rng.random() - 0.5) * 0.5))) for q in vs]
    for a, b, d in fs: p.tri("l", jit(cols[rng.randrange(len(cols))], rng, 0.05), pts[a], pts[b], pts[d])


def edgescrub(v, lod):
    rng = random.Random(7300 + v * 31)          # the same layout at both LODs
    p = P()
    n = 4 + v
    blobs = []
    for i in range(n):
        a = rng.random() * 6.283; d = rng.random() * 1.4
        h = 1.2 + rng.random() * 2.6
        blobs.append((Vector((math.cos(a) * d, math.sin(a) * d, h)), 1.2 + rng.random() * 0.8, 0.9 + rng.random() * 0.7))
    blobs.sort(key=lambda b: -b[1])   # the far version keeps the biggest
    for c, rxy, rz in (blobs if lod == 0 else blobs[:4]):
        clump(p, c, rxy, rxy, rz, LEAF, rng, detail=lod == 0)
    if lod == 0:
        for c, _, _ in blobs[:2]:   # a couple of stems showing at the bottom
            tube(p, [Vector((c.x * 0.5, c.y * 0.5, 0)), Vector((c.x, c.y, c.z - 0.4))], [0.05, 0.03], 3, lambda i, s: (0.30, 0.24, 0.18), cap_end=False)
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.0


# ------------------------------------------------------------------ the boom gate (the junction, follow-up)
def box(p, k, col, c, hx, hy, hz, shade=True):
    """An axis-aligned box centred on c (half sizes), six faces, the sides a little darker."""
    x0, x1, y0, y1, z0, z1 = c.x - hx, c.x + hx, c.y - hy, c.y + hy, c.z - hz, c.z + hz
    V = lambda x, y, z: Vector((x, y, z))
    d = (lambda f: tuple(q * f for q in col)) if shade else (lambda f: col)
    p.quad(k, col, V(x0, y0, z1), V(x1, y0, z1), V(x1, y1, z1), V(x0, y1, z1))
    p.quad(k, d(0.6), V(x0, y1, z0), V(x1, y1, z0), V(x1, y0, z0), V(x0, y0, z0))
    p.quad(k, d(0.85), V(x0, y0, z0), V(x1, y0, z0), V(x1, y0, z1), V(x0, y0, z1))
    p.quad(k, d(0.8), V(x1, y1, z0), V(x0, y1, z0), V(x0, y1, z1), V(x1, y1, z1))
    p.quad(k, d(0.75), V(x1, y0, z0), V(x1, y1, z0), V(x1, y1, z1), V(x1, y0, z1))
    p.quad(k, d(0.75), V(x0, y1, z0), V(x0, y0, z0), V(x0, y0, z1), V(x0, y1, z1))


def boomgate(v, lod):
    """A national-park boom gate across a closed track: two steel posts 8.8 m apart, a striped pipe boom across at hip
    height, a padlocked chain at one end and a white sign hung in the middle. Along x (across the road), base at z = 0."""
    p = P()
    POST, WHITE, RED, SIGN = (0.45, 0.46, 0.44), (1.0, 1.0, 0.96), (0.95, 0.22, 0.16), (1.0, 1.0, 0.97)
    for x in (-4.4, 4.4):
        box(p, "b", POST, Vector((x, 0, 0.55)), 0.08, 0.08, 0.55)
        box(p, "b", POST, Vector((x, 0, 1.12)), 0.11, 0.11, 0.03)
    n = 12   # the boom: red and white bands
    for i in range(n):
        x0 = -4.4 + 8.8 * i / n; x1 = -4.4 + 8.8 * (i + 1) / n
        box(p, "b", RED if i % 2 else WHITE, Vector(((x0 + x1) / 2, 0, 0.95)), (x1 - x0) / 2, 0.08, 0.08, shade=False)
    box(p, "b", SIGN, Vector((0, -0.1, 0.62)), 0.6, 0.015, 0.25, shade=False)   # the sign (ROAD CLOSED, too small to read): white with a red band
    box(p, "b", RED, Vector((0, -0.12, 0.62)), 0.5, 0.01, 0.06, shade=False)
    box(p, "b", (0.6, 0.55, 0.2), Vector((4.25, 0, 0.95)), 0.05, 0.07, 0.07)  # the padlock
    return p, 1.15, 0.0


# ------------------------------------------------------------------ the army man (follow-up)
PLASTIC = (0.40, 0.48, 0.26)   # olive-green plastic


def armyman(v, lod):
    """A toy army man blown up to life size, the classic sniper: standing, left foot forward, rifle with a scope at the
    shoulder, aiming along +x; helmet, belt, on a flat moulded base. One colour of plastic, shaded by the facets."""
    rng = random.Random(4242 + lod)
    p = P()
    col = lambda i, s: jit(PLASTIC, rng, 0.04)
    sides = 6 if lod == 0 else 4
    # the base: a flat irregular slab
    bpts = []
    for k in range(9 if lod == 0 else 6):
        a = k * 2 * math.pi / (9 if lod == 0 else 6)
        r = 0.55 + 0.12 * rng.random()
        bpts.append(Vector((math.cos(a) * r * 1.2, math.sin(a) * r * 0.9, 0.0)))
    top = [q + Vector((0, 0, 0.07)) for q in bpts]
    cen = Vector((0, 0, 0.07))
    for k in range(len(bpts)):
        k2 = (k + 1) % len(bpts)
        p.tri("b", jit(PLASTIC, rng, 0.03), top[k], top[k2], cen)
        p.quad("b", tuple(q * 0.75 for q in PLASTIC), bpts[k], bpts[k2], top[k2], top[k])
    # legs: left forward, right back, knees a little bent
    hipL, hipR = Vector((0.05, 0.11, 0.98)), Vector((-0.05, -0.11, 0.98))
    for hip, foot, knee in ((hipL, Vector((0.32, 0.16, 0.12)), Vector((0.22, 0.14, 0.55))),
                            (hipR, Vector((-0.30, -0.15, 0.12)), Vector((-0.16, -0.14, 0.55)))):
        tube(p, [hip, knee, foot], [0.09, 0.075, 0.065], sides, col, cap_end=False)
        box(p, "b", PLASTIC, foot + Vector((0.07, 0, -0.025)), 0.14, 0.06, 0.05)   # boot
    # body, a little turned towards the aim
    tube(p, [Vector((0, 0, 0.95)), Vector((0.02, 0, 1.22)), Vector((0.05, 0, 1.48))], [(0.17, 0.21), (0.18, 0.22), (0.17, 0.23)], sides, col, cap_start=True)
    box(p, "b", PLASTIC, Vector((0.02, 0, 1.0)), 0.17, 0.21, 0.03)   # belt
    if lod == 0:
        box(p, "b", PLASTIC, Vector((0.12, 0.12, 1.0)), 0.05, 0.05, 0.05)   # pouches
        box(p, "b", PLASTIC, Vector((0.12, -0.12, 1.0)), 0.05, 0.05, 0.05)
    # head and helmet, cheek down on the stock
    head = Vector((0.12, -0.02, 1.62))
    tube(p, [Vector((0.05, 0, 1.48)), head + Vector((0, 0, 0.02))], [0.06, 0.1], sides, col)
    nh = 8 if lod == 0 else 5
    ang = [k * 2 * math.pi / nh for k in range(nh)]
    rings = [[head + Vector((math.cos(a) * r * 1.08, math.sin(a) * r, dz)) for a in ang] for r, dz in ((0.19, 0.0), (0.16, 0.08), (0.1, 0.16))]   # brim, dome, top
    crown = head + Vector((0, 0, 0.19))
    for ri in range(2):
        for k in range(nh):
            k2 = (k + 1) % nh
            p.quad("b", jit(PLASTIC, rng, 0.04), rings[ri][k], rings[ri][k2], rings[ri + 1][k2], rings[ri + 1][k])
    for k in range(nh):
        p.tri("b", jit(PLASTIC, rng, 0.04), rings[2][k], rings[2][(k + 1) % nh], crown)
        p.tri("b", tuple(q * 0.7 for q in PLASTIC), rings[0][(k + 1) % nh], rings[0][k], head)   # under the brim
    # the rifle: stock at the right shoulder, barrel out along +x, a scope on top
    stock, grip, muzzle = Vector((0.12, -0.13, 1.5)), Vector((0.4, -0.06, 1.52)), Vector((1.25, 0.03, 1.56))
    tube(p, [stock, grip, muzzle], [0.045, 0.035, 0.016], 4, col)
    if lod == 0:
        tube(p, [Vector((0.42, -0.05, 1.6)), Vector((0.72, -0.02, 1.61))], [0.03, 0.03], 5, col, cap_start=True)   # scope
    # arms: right hand at the grip, left hand out under the fore-end
    shR, shL = Vector((0.05, -0.2, 1.44)), Vector((0.07, 0.2, 1.44))
    tube(p, [shR, Vector((0.2, -0.3, 1.32)), grip + Vector((0, -0.04, -0.03))], [0.065, 0.055, 0.045], sides, col)
    tube(p, [shL, Vector((0.35, 0.2, 1.38)), Vector((0.78, 0.01, 1.5))], [0.065, 0.055, 0.045], sides, col)
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.0


CATALOGUE = [
    ("sambar", 4, sambar, (0, 1), "D"),
    ("edgescrub", 3, edgescrub, (0, 1), "E"),
    ("boomgate", 1, boomgate, (0,), "J"),
    ("armyman", 1, armyman, (0, 1), "A"),
]


def f3(v): return f"{v.x:.3f} {v.z:.3f} {v.y:.3f}"   # Blender (x, y, z up) -> game (x, y up, z)


built = []
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
with open(OUT, "w") as f:
    f.write("# Bush Drift build B models, made by tools/make_world.py (procedural; sambar: see docs/forest/deer.md)\n")
    for name, nv, fn, lods, zone in CATALOGUE:
        for v in range(nv):
            for lod in lods:
                p, H, R = fn(v, lod)
                f.write(f"plant {name} {v} {lod} {H:.3f} {R:.3f} {len(p.t)} {zone}\n")
                for k, c, a, b, d in p.t:
                    f.write(f"t {k} {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}  {f3(a)}  {f3(b)}  {f3(d)}\n")
                built.append((name, v, lod, H, p))
                print(f"MODEL {name:10s} v{v} lod{lod}: {len(p.t):4d} triangles, {H:4.2f} m tall")

if RENDER_DIR:
    import bpy
    os.makedirs(RENDER_DIR, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    x = 0.0
    only = os.environ.get("RENDER_ONLY")   # e.g. RENDER_ONLY=armyman: just that model, closer
    for name, v, lod, H, p in built:
        if lod or (only and name != only): continue
        me = bpy.data.meshes.new(f"{name}{v}")
        verts, faces, cols = [], [], []
        for k, c, a, b, d in p.t:
            faces.append((len(verts), len(verts) + 1, len(verts) + 2))
            verts += [(a.x + x, a.y, a.z), (b.x + x, b.y, b.z), (d.x + x, d.y, d.z)]
            cols += [(*c, 1.0)] * 3
        me.from_pydata(verts, [], faces)
        attr = me.color_attributes.new("Col", 'FLOAT_COLOR', 'CORNER')
        attr.data.foreach_set("color", [q for c in cols for q in c])
        me.color_attributes.active_color = attr
        o = bpy.data.objects.new(me.name, me); sc.collection.objects.link(o)
        mat = bpy.data.materials.new("m"); mat.use_nodes = True
        nt = mat.node_tree; bs = nt.nodes.get("Principled BSDF"); ca = nt.nodes.new("ShaderNodeVertexColor"); ca.layer_name = "Col"
        nt.links.new(ca.outputs["Color"], bs.inputs["Base Color"]); o.data.materials.append(mat)
        x += 3.2
    cam = bpy.data.cameras.new("c"); co = bpy.data.objects.new("c", cam); sc.collection.objects.link(co)
    co.location = (x / 2 - 1.6, -11.0, 2.4); co.rotation_euler = (math.radians(85), 0, 0); cam.lens = 30; sc.camera = co
    if only: co.location = (0.6, -4.2, 1.2); co.rotation_euler = (math.radians(88), 0, math.radians(8)); cam.lens = 35
    sun = bpy.data.lights.new("s", 'SUN'); sun.energy = 3.5; so = bpy.data.objects.new("s", sun); sc.collection.objects.link(so)
    so.rotation_euler = (math.radians(50), math.radians(10), math.radians(30))
    w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; w.node_tree.nodes["Background"].inputs[0].default_value = (0.85, 0.78, 0.65, 1)
    sc.render.engine = 'BLENDER_EEVEE' if 'BLENDER_EEVEE' in [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items] else 'BLENDER_WORKBENCH'
    sc.render.resolution_x, sc.render.resolution_y = (900, 900) if only else (1600, 420)
    sc.render.filepath = os.path.join(RENDER_DIR, "lineup.png")
    bpy.ops.render.render(write_still=True)
    print("RENDER", sc.render.filepath)
