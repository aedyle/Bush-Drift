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


def armyman_body(v, lod):
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
    head = Vector((0.12, -0.02, 1.62)) if v == 1 else Vector((0.04, 0.03, 1.66))   # build C: upright when looking about
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
    shR, shL = Vector((0.05, -0.2, 1.44)), Vector((0.07, 0.2, 1.44))
    if v == 1:
        # back on duty: the rifle at the right shoulder, barrel out along +x, a scope on top
        stock, grip, muzzle = Vector((0.12, -0.13, 1.5)), Vector((0.4, -0.06, 1.52)), Vector((1.25, 0.03, 1.56))
        tube(p, [stock, grip, muzzle], [0.045, 0.035, 0.016], 4, col)
        if lod == 0:
            tube(p, [Vector((0.42, -0.05, 1.6)), Vector((0.72, -0.02, 1.61))], [0.03, 0.03], 5, col, cap_start=True)   # scope
        # arms: right hand at the grip, left hand out under the fore-end
        tube(p, [shR, Vector((0.2, -0.3, 1.32)), grip + Vector((0, -0.04, -0.03))], [0.065, 0.055, 0.045], sides, col)
        tube(p, [shL, Vector((0.35, 0.2, 1.38)), Vector((0.78, 0.01, 1.5))], [0.065, 0.055, 0.045], sides, col)
    else:
        # build C: his rifle lost, empty hands: the left one shading his eyes as he looks about, the right one open at his side
        tube(p, [shR, Vector((0.12, -0.32, 1.12)), Vector((0.2, -0.3, 0.92))], [0.065, 0.055, 0.045], sides, col)
        box(p, "b", PLASTIC, Vector((0.22, -0.3, 0.88)), 0.04, 0.025, 0.05)
        tube(p, [shL, Vector((0.28, 0.24, 1.5)), Vector((0.26, 0.04, 1.72))], [0.065, 0.055, 0.045], sides, col)
        box(p, "b", PLASTIC, Vector((0.27, -0.02, 1.74)), 0.05, 0.07, 0.015)   # the hand flat over the brim
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.0


# ---------------------------------------------------------------- Quayle (build C follow-up: the hunter)
# After Dylan's reference photo (refs/quayle/, not in git): a blaze-orange cap, a blaze-orange vest with a darker leaf
# camo over a grey bush-camo jacket, grey camo trousers into gaiters and brown boots; a scoped bolt-action rifle with a
# stainless barrel and a grey-tan camo synthetic stock, a sling. No brands. About 1.8 m, aiming along +x.
CAMO = [(0.46, 0.46, 0.43), (0.26, 0.25, 0.23), (0.60, 0.59, 0.55), (0.38, 0.32, 0.24), (0.17, 0.17, 0.16)]
VEST = [(0.96, 0.44, 0.08), (0.96, 0.44, 0.08), (0.74, 0.27, 0.06), (0.30, 0.16, 0.07)]
CAP = (0.98, 0.60, 0.10); SKIN = (0.80, 0.62, 0.50); BOOT = (0.36, 0.26, 0.17); GAITER = (0.47, 0.47, 0.45)
STOCK = [(0.62, 0.58, 0.50), (0.48, 0.45, 0.39), (0.70, 0.66, 0.58)]; STEEL = (0.74, 0.75, 0.76); SCOPE = (0.09, 0.09, 0.10)


def patches(pal, salt, weights=None):
    """A colour function for tube(): blotches two segments by two sides big, from a palette (camouflage)."""
    def f(i, s2):
        h = (i // 2) * 73856093 ^ (s2 // 2) * 19349663 ^ salt * 83492791
        h = (h ^ (h >> 13)) * 1274126177 & 0xffffffff
        r = (h >> 8) / 16777216.0
        if weights:
            acc = 0
            for k, w in enumerate(weights):
                acc += w
                if r < acc: return pal[k]
        return pal[int(r * len(pal)) % len(pal)]
    return f


def placed(p, sub, o, fx, fy, fz):
    """sub's triangles (made in a frame of its own) into p, at origin o with axes fx, fy, fz."""
    for k, c, a, b, d in sub.t:
        p.tri(k, c, *[o + fx * q.x + fy * q.y + fz * q.z for q in (a, b, d)])


def rifle_parts(lod, sling=True):
    """The rifle in its own frame: x along the barrel (the trigger at 0), y to the left, z up. 1.1 m overall."""
    rng = random.Random(4250 + lod)
    q = P()
    sd = 10 if lod == 0 else 5
    st = patches(STOCK, 7, [0.45, 0.35, 0.2])
    # the stock: the butt, the wrist and comb, under the action, the fore-end (oval sections lofted along it)
    pts = [Vector((-0.40, 0, -0.035)), Vector((-0.30, 0, -0.03)), Vector((-0.16, 0, -0.012)), Vector((-0.05, 0, -0.004)),
           Vector((0.03, 0, -0.022)), Vector((0.16, 0, -0.03)), Vector((0.30, 0, -0.03)), Vector((0.42, 0, -0.026))]
    rad = [(0.021, 0.068), (0.023, 0.06), (0.021, 0.042), (0.018, 0.027), (0.022, 0.03), (0.024, 0.028), (0.022, 0.024), (0.018, 0.018)]
    if lod: pts, rad = pts[::2] + [pts[-1]], rad[::2] + [rad[-1]]
    tube(q, pts, rad, sd if lod == 0 else 4, st, cap_end=True, cap_start=True)
    box(q, "b", (0.12, 0.12, 0.12), Vector((-0.405, 0, -0.035)), 0.008, 0.021, 0.066)                                  # the butt pad
    tube(q, [Vector((-0.03, 0, -0.012)), Vector((-0.055, 0, -0.06)), Vector((-0.07, 0, -0.095))], [0.017, 0.018, 0.016], sd if lod == 0 else 4, st, cap_end=True)   # the pistol grip
    # the action, the barrel (stainless, tapering), the muzzle
    tube(q, [Vector((-0.02, 0, 0)), Vector((0.22, 0, 0))], [0.016, 0.016], sd, lambda i, s2: (0.62, 0.63, 0.64), cap_start=True)
    tube(q, [Vector((0.22, 0, 0)), Vector((0.50, 0, 0)), Vector((0.80, 0, 0))], [0.011, 0.009, 0.0075], sd if lod == 0 else 4, lambda i, s2: jit(STEEL, rng, 0.04), cap_end=True)
    # the scope: eyepiece, tube, turrets, objective bell, two rings
    if lod == 0:
        tube(q, [Vector((-0.10, 0, 0.068)), Vector((-0.06, 0, 0.068)), Vector((-0.03, 0, 0.068)), Vector((0.20, 0, 0.068)), Vector((0.24, 0, 0.068)), Vector((0.30, 0, 0.068))],
             [0.021, 0.021, 0.0135, 0.0135, 0.026, 0.026], 10, lambda i, s2: SCOPE, cap_end=True, cap_start=True)
        tube(q, [Vector((0.085, 0, 0.068)), Vector((0.085, 0, 0.1))], [0.011, 0.011], 6, lambda i, s2: SCOPE, cap_end=True)   # the turrets
        tube(q, [Vector((0.085, 0, 0.068)), Vector((0.085, -0.032, 0.068))], [0.011, 0.011], 6, lambda i, s2: SCOPE, cap_end=True)
        for x in (0.0, 0.17):
            box(q, "b", SCOPE, Vector((x, 0, 0.034)), 0.012, 0.016, 0.022)
        box(q, "b", (0.55, 0.75, 0.85), Vector((0.301, 0, 0.068)), 0.001, 0.02, 0.02, shade=False)                     # the lens
    else:
        tube(q, [Vector((-0.10, 0, 0.068)), Vector((0.30, 0, 0.068))], [0.018, 0.022], 5, lambda i, s2: SCOPE, cap_end=True, cap_start=True)
    # the bolt and its handle (out to the right), the trigger guard, the magazine
    tube(q, [Vector((0.03, 0, 0.012)), Vector((0.03, -0.05, 0.0)), Vector((0.025, -0.065, -0.01))], [0.005, 0.005, 0.004], 5, lambda i, s2: STEEL, cap_end=True)
    if lod == 0:
        box(q, "b", STEEL, Vector((0.025, -0.068, -0.012)), 0.009, 0.009, 0.009)
        tube(q, [Vector((-0.025, 0, -0.04)), Vector((-0.02, 0, -0.062)), Vector((0.02, 0, -0.064)), Vector((0.035, 0, -0.04))], [0.004] * 4, 4, lambda i, s2: (0.15, 0.15, 0.15))
        box(q, "b", (0.15, 0.15, 0.15), Vector((0.0, 0, -0.045)), 0.002, 0.003, 0.01)                                 # the trigger
    box(q, "b", (0.16, 0.16, 0.17), Vector((0.08, 0, -0.06)), 0.035, 0.015, 0.016)
    if sling and lod == 0:   # the sling: a dark strap hanging between the swivels
        sp = [Vector((0.34, 0, -0.05)), Vector((0.18, 0, -0.17)), Vector((-0.02, 0, -0.2)), Vector((-0.22, 0, -0.14)), Vector((-0.3, 0, -0.085))]
        tube(q, sp, [(0.002, 0.014)] * 5, 4, lambda i, s2: (0.14, 0.13, 0.12))
    return q


def armyman(v, lod):
    """Quayle, the hunter. v 0: his rifle lost, empty hands, one shading his eyes as he looks about; v 1: back on duty,
    the rifle at his right shoulder, aiming along +x (like the reference photo)."""
    rng = random.Random(4300 + v * 7 + lod)
    p = P()
    sd = 10 if lod == 0 else 5
    camo, pants, vest = patches(CAMO, 1 + v), patches(CAMO, 11 + v, [0.3, 0.25, 0.15, 0.2, 0.1]), patches(VEST, 21, [0.45, 0.2, 0.2, 0.15])
    aim = v == 1
    # boots and gaiters, the legs (camo trousers), feet a little apart, the left a touch forward
    for side, fx in ((1, 0.06), (-1, -0.07)):
        y = 0.11 * side
        hip, knee, ankle = Vector((0.0, y * 0.9, 0.95)), Vector((fx * 0.6, y, 0.52)), Vector((fx, y * 1.05, 0.11))
        tube(p, [hip, knee + Vector((0, 0, 0.04)), knee], [(0.085, 0.095), (0.07, 0.08), (0.065, 0.075)], sd, pants, cap_end=False)
        tube(p, [knee, ankle + Vector((0, 0, 0.18)), ankle], [(0.062, 0.068), (0.058, 0.06), (0.05, 0.055)], sd, lambda i, s2: GAITER if i >= 1 else pants(i, s2), cap_end=False)
        if lod == 0: box(p, "b", CAMO[3], knee + Vector((0.0, 0.07 * side, 0.12)), 0.05, 0.02, 0.06)   # a cargo pocket
        box(p, "b", BOOT, ankle + Vector((0.05, 0, -0.05)), 0.14, 0.055, 0.06)
        box(p, "b", (0.2, 0.15, 0.1), ankle + Vector((0.05, 0, -0.105)), 0.145, 0.058, 0.008, shade=False)   # the soles
    # the body: the jacket from the hips, the vest over it, a collar
    tube(p, [Vector((0, 0, 0.93)), Vector((0.01, 0, 1.1)), Vector((0.01, 0, 1.3)), Vector((0.0, 0, 1.45))], [(0.18, 0.12), (0.18, 0.115), (0.2, 0.125), (0.19, 0.115)], sd, camo, cap_start=True)
    tube(p, [Vector((0.002, 0, 1.0)), Vector((0.012, 0, 1.15)), Vector((0.012, 0, 1.32)), Vector((0.004, 0, 1.43))], [(0.192, 0.132), (0.19, 0.127), (0.21, 0.137), (0.2, 0.127)], sd, vest)
    if lod == 0:
        box(p, "b", VEST[2], Vector((0.13, 0.08, 1.12)), 0.012, 0.055, 0.05)   # vest pockets
        box(p, "b", VEST[2], Vector((0.13, -0.08, 1.12)), 0.012, 0.055, 0.05)
        tube(p, [Vector((0.0, 0, 1.43)), Vector((0.0, 0, 1.49))], [(0.09, 0.08), (0.075, 0.07)], sd, camo)   # the collar
    # the head: a bit forward and down to the scope when aiming; the face, nose and ears, the cap and its brim
    hc = Vector((0.07, -0.03, 1.6)) if aim else Vector((0.02, 0.0, 1.64))
    tilt = Vector((0.35, 0, -0.06)).normalized() if aim else Vector((1, 0, 0))
    tube(p, [Vector((0.0, 0, 1.47)), hc + Vector((-0.02, 0, -0.06))], [0.055, 0.06], sd, lambda i, s2: SKIN)
    hr = [(0.0, 0.03), (0.08, 0.075), (0.11, 0.0), (0.105, -0.04), (0.08, -0.085), (0.0, -0.11)]   # (radius, height) from the crown down
    rings = []
    for r0, z0 in hr:
        rings.append([hc + Vector((math.cos(a) * r0 * 0.95, math.sin(a) * r0 * 0.85, z0)) for a in [k * 2 * math.pi / sd for k in range(sd)]])
    for a in range(len(rings) - 1):
        for k in range(sd):
            k2 = (k + 1) % sd
            front = math.cos(k * 2 * math.pi / sd) > 0.3 and a >= 2
            p.quad("b", jit(SKIN if front or a >= 2 else (0.35, 0.25, 0.18), rng, 0.03), rings[a][k], rings[a][k2], rings[a + 1][k2], rings[a + 1][k])
    if lod == 0:
        box(p, "b", tuple(c * 0.92 for c in SKIN), hc + Vector((0.1, 0.0, -0.015)), 0.018, 0.012, 0.022)   # the nose
        for side in (1, -1): box(p, "b", tuple(c * 0.9 for c in SKIN), hc + Vector((0.0, 0.098 * side, -0.01)), 0.015, 0.008, 0.025)   # ears
        box(p, "b", (0.35, 0.28, 0.22), hc + Vector((0.095, 0.0, -0.06)), 0.01, 0.035, 0.012)   # a moustache
    cap_rings = [(0.118, 0.0), (0.112, 0.045), (0.085, 0.09), (0.0, 0.11)]
    cr = [[hc + Vector((math.cos(a) * r0, math.sin(a) * r0 * 0.92, z0 + 0.015)) for a in [k * 2 * math.pi / sd for k in range(sd)]] for r0, z0 in cap_rings]
    for a in range(len(cr) - 1):
        for k in range(sd):
            k2 = (k + 1) % sd
            p.quad("b", jit(CAP, rng, 0.03), cr[a][k], cr[a][k2], cr[a + 1][k2], cr[a + 1][k])
    bf = hc + Vector((0.0, 0, 0.018))   # the brim, out over the face
    brim = [bf + Vector((0.11, -0.085, 0)), bf + Vector((0.11, 0.085, 0)), bf + Vector((0.21, 0.07, -0.02)), bf + Vector((0.22, 0.0, -0.022)), bf + Vector((0.21, -0.07, -0.02))]
    for k in range(1, 4): p.tri("b", tuple(c * 0.92 for c in CAP), brim[0], brim[k], brim[k + 1])
    p.tri("b", tuple(c * 0.92 for c in CAP), brim[0], brim[1], brim[2])
    box(p, "b", (0.2, 0.16, 0.1), hc + Vector((0.105, 0.0, 0.07)), 0.004, 0.028, 0.018, shade=False)   # a plain patch on the front (no brand)
    # the arms
    shR, shL = Vector((0.0, -0.205, 1.42)), Vector((0.0, 0.205, 1.42))
    hand = lambda c: box(p, "b", SKIN, c, 0.035, 0.03, 0.045)
    if aim:
        o = Vector((0.33, -0.12, 1.505)); fx2 = Vector((1, 0, 0.03)).normalized(); fz2 = Vector((-0.03, 0, 1)).normalized(); fy2 = fz2.cross(fx2)
        placed(p, rifle_parts(lod), o, fx2, fy2, fz2)
        grip = o + fx2 * -0.055 + fz2 * -0.06
        tube(p, [shR, Vector((0.04, -0.38, 1.37)), grip + Vector((0, -0.03, 0))], [(0.06, 0.065), (0.052, 0.055), (0.042, 0.045)], sd, camo)
        hand(grip)
        fore = o + fx2 * 0.3 + fz2 * -0.055
        tube(p, [shL, Vector((0.3, 0.08, 1.3)), fore + Vector((0, 0.0, -0.03))], [(0.06, 0.065), (0.052, 0.055), (0.042, 0.045)], sd, camo)
        hand(fore + Vector((0, 0, -0.025)))
    else:
        tube(p, [shR, Vector((0.06, -0.3, 1.12)), Vector((0.1, -0.27, 0.92))], [(0.06, 0.065), (0.052, 0.055), (0.042, 0.045)], sd, camo)
        hand(Vector((0.11, -0.27, 0.87)))
        tube(p, [shL, Vector((0.25, 0.26, 1.5)), hc + Vector((0.12, 0.06, 0.06))], [(0.06, 0.065), (0.052, 0.055), (0.042, 0.045)], sd, camo)
        box(p, "b", SKIN, hc + Vector((0.15, 0.02, 0.06)), 0.04, 0.06, 0.012)   # the hand flat, shading his eyes under the brim
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.0


# ------------------------------------------------------------------ build C: Explorer
def cyl(p, c, r, z0, z1, n, col_side, col_top=None, col_bot=None, kind="b", wedge=None):
    """An upright cylinder about c from z0 to z1 (n sides). wedge=(a0, a1, col): a slice cut out between those angles,
    its cut faces in col (cheese)."""
    ang = [k * 2 * math.pi / n for k in range(n + 1)]
    ct, cb = Vector((c.x, c.y, z1)), Vector((c.x, c.y, z0))
    for k in range(n):
        a0, a1 = ang[k], ang[k + 1]
        if wedge and wedge[0] <= (a0 + a1) / 2 <= wedge[1]: continue
        q0 = Vector((c.x + math.cos(a0) * r, c.y + math.sin(a0) * r, 0)); q1 = Vector((c.x + math.cos(a1) * r, c.y + math.sin(a1) * r, 0))
        p.quad(kind, col_side if not callable(col_side) else col_side(k), Vector((q0.x, q0.y, z0)), Vector((q1.x, q1.y, z0)), Vector((q1.x, q1.y, z1)), Vector((q0.x, q0.y, z1)))
        if col_top: p.tri(kind, col_top, Vector((q0.x, q0.y, z1)), Vector((q1.x, q1.y, z1)), ct)
        if col_bot: p.tri(kind, col_bot, Vector((q1.x, q1.y, z0)), Vector((q0.x, q0.y, z0)), cb)
    if wedge:
        for a in (wedge[0], wedge[1]):
            q = Vector((c.x + math.cos(a) * r, c.y + math.sin(a) * r, 0))
            p.quad(kind, wedge[2], Vector((c.x, c.y, z0)), Vector((q.x, q.y, z0)), Vector((q.x, q.y, z1)), Vector((c.x, c.y, z1)))


def jar(p, c, r, h, glass, fill, lid, rng, bits=None):
    """A glass jar of something: the contents' colour through tinted glass, a lid, a plain label band (no brand)."""
    cyl(p, c, r, 0, h * 0.82, 8, lambda k: jit(glass if k % 2 else fill, rng, 0.04), col_bot=fill)
    cyl(p, c, r * 1.01, h * 0.32, h * 0.56, 8, (0.94, 0.91, 0.82))                       # the label, blank cream
    cyl(p, c, r * 0.8, h * 0.82, h * 0.9, 8, glass, col_top=fill)                         # the neck
    cyl(p, c, r * 0.86, h * 0.9, h, 8, lid, col_top=tuple(q * 1.1 for q in lid))          # the lid
    if bits:
        for k in range(5):
            a = k * 1.256 + rng.random()
            box(p, "b", bits, Vector((c.x + math.cos(a) * r * 0.7, c.y + math.sin(a) * r * 0.7, h * (0.2 + 0.12 * k))), r * 0.25, r * 0.25, r * 0.22, shade=False)


def bowl(p, c, r, h, inside, rng, heap=None, n=10):
    """A blue enamel bowl (white rim), heaped with something."""
    BLUE, RIM = (0.20, 0.36, 0.70), (0.95, 0.95, 0.93)
    ang = [k * 2 * math.pi / n for k in range(n + 1)]
    for k in range(n):
        a0, a1 = ang[k], ang[k + 1]
        b0 = Vector((c.x + math.cos(a0) * r * 0.6, c.y + math.sin(a0) * r * 0.6, c.z)); b1 = Vector((c.x + math.cos(a1) * r * 0.6, c.y + math.sin(a1) * r * 0.6, c.z))
        t0 = Vector((c.x + math.cos(a0) * r, c.y + math.sin(a0) * r, c.z + h)); t1 = Vector((c.x + math.cos(a1) * r, c.y + math.sin(a1) * r, c.z + h))
        p.quad("b", jit(BLUE, rng, 0.04), b0, b1, t1, t0)
        p.quad("b", RIM, t0, t1, t1 + Vector((0, 0, 0.012 + r * 0.04)), t0 + Vector((0, 0, 0.012 + r * 0.04)))
        p.tri("b", inside, t0 + Vector((0, 0, -h * 0.15)), t1 + Vector((0, 0, -h * 0.15)), Vector((c.x, c.y, c.z + h * (1.3 if heap else 0.85))))
    if heap:
        for k in range(6):
            a = k * 1.05 + rng.random()
            box(p, "b", jit(heap, rng, 0.08), Vector((c.x + math.cos(a) * r * 0.45, c.y + math.sin(a) * r * 0.45, c.z + h * 1.05)), r * 0.2, r * 0.13, r * 0.12, shade=False)


def bottle(p, c, h, rng):
    """A bottle of red: dark glass, a plain cream label with a burgundy band (no brand), a foil capsule."""
    GLASS, LABEL, BAND, FOIL = (0.10, 0.17, 0.10), (0.92, 0.88, 0.76), (0.45, 0.07, 0.12), (0.42, 0.08, 0.13)
    r = h * 0.12
    cyl(p, c, r, 0, h * 0.62, 8, lambda k: jit(GLASS, rng, 0.08), col_bot=GLASS)
    cyl(p, c, r * 1.02, h * 0.18, h * 0.44, 8, LABEL)
    cyl(p, c, r * 1.03, h * 0.38, h * 0.42, 8, BAND)
    for k in range(8):   # the shoulder
        a0, a1 = k * 0.785, (k + 1) * 0.785
        p.quad("b", jit(GLASS, rng, 0.08), Vector((c.x + math.cos(a0) * r, c.y + math.sin(a0) * r, h * 0.62)), Vector((c.x + math.cos(a1) * r, c.y + math.sin(a1) * r, h * 0.62)),
               Vector((c.x + math.cos(a1) * r * 0.32, c.y + math.sin(a1) * r * 0.32, h * 0.74)), Vector((c.x + math.cos(a0) * r * 0.32, c.y + math.sin(a0) * r * 0.32, h * 0.74)))
    cyl(p, c, r * 0.32, h * 0.74, h * 0.88, 6, GLASS)
    cyl(p, c, r * 0.36, h * 0.88, h, 6, FOIL, col_top=FOIL)


HAMPER_SCALE = 2.2   # the hunt's things are drawn about twice life size so they show from the ute (0.5 to 0.8 m)
BLUE_ENAMEL, ENAMEL_RIM = (0.18, 0.30, 0.66), (0.95, 0.95, 0.93)
CHORIZO, CHORIZO_CUT = (0.60, 0.16, 0.08), (0.78, 0.30, 0.16)
JALAPENO = (0.30, 0.42, 0.18)
OLIVE = (0.50, 0.62, 0.20)
TERRACOTTA = (0.72, 0.40, 0.26)


def tub(p, c, r, h, fill, rng, bits=None, bitcol=None, nb=9, lid=None):
    """A clear plastic deli tub: its contents seen through the side, heaped bits on top (olives, chickpeas)."""
    cyl(p, c, r, c.z, c.z + h, 10, lambda k: jit(fill if k % 3 else tuple(min(1.0, q * 1.25) for q in fill), rng, 0.04), col_bot=fill)
    cyl(p, c, r * 1.03, c.z + h * 0.96, c.z + h, 10, (0.90, 0.92, 0.92))   # the rim
    cyl(p, c, r * 0.96, c.z + h * 0.9, c.z + h * 0.92, 10, fill, col_top=fill)
    if bits:
        for k in range(nb):
            a = k * 2.4 + rng.random() * 0.5; d = r * 0.6 * math.sqrt((k + 0.5) / nb)
            box(p, "b", jit(bitcol, rng, 0.08), Vector((c.x + math.cos(a) * d, c.y + math.sin(a) * d, c.z + h * 0.95)), bits, bits * 0.8, bits * 0.7)


def enamel(p, c, r, h, rng, n=10):
    """A blue enamel camp bowl with a white rim (its inside dark blue); returns the height things sit at in it."""
    ang = [k * 2 * math.pi / n for k in range(n + 1)]
    for k in range(n):
        a0, a1 = ang[k], ang[k + 1]
        b0 = Vector((c.x + math.cos(a0) * r * 0.62, c.y + math.sin(a0) * r * 0.62, c.z)); b1 = Vector((c.x + math.cos(a1) * r * 0.62, c.y + math.sin(a1) * r * 0.62, c.z))
        t0 = Vector((c.x + math.cos(a0) * r, c.y + math.sin(a0) * r, c.z + h)); t1 = Vector((c.x + math.cos(a1) * r, c.y + math.sin(a1) * r, c.z + h))
        p.quad("b", jit(BLUE_ENAMEL, rng, 0.04), b0, b1, t1, t0)
        p.quad("b", ENAMEL_RIM, t0, t1, t1 + Vector((0, 0, 0.01 + r * 0.05)), t0 + Vector((0, 0, 0.01 + r * 0.05)))
        p.tri("b", (0.12, 0.18, 0.40), b1, b0, Vector((c.x, c.y, c.z)))
    return c.z + h * 0.5


def slices(p, c, r, n, rng, col, cut, rad, th=0.012):
    """Round slices (chorizo) piled in a heap of radius r."""
    for k in range(n):
        a = k * 2.4 + rng.random(); d = r * math.sqrt((k + 0.5) / n); z = c.z + th * (1.5 * (1.0 - d / r) + 0.6 * rng.random())
        cyl(p, Vector((c.x + math.cos(a) * d, c.y + math.sin(a) * d, z)), rad, z, z + th, 6, col, col_top=jit(cut, rng, 0.08))


def chillies(p, c, r, n, rng, rad):
    """Whole blistered green chillies (stuffed jalapenos), lying in a heap."""
    for k in range(n):
        a = k * 2.1 + rng.random(); d = r * 0.5 * math.sqrt((k + 0.5) / n)
        x0 = Vector((c.x + math.cos(a) * d, c.y + math.sin(a) * d, c.z + rad * (1 + 0.5 * (k % 2))))
        dr = Vector((math.cos(a + 1.6), math.sin(a + 1.6), 0.1)) * rad * 3.2
        tube(p, [x0 - dr, x0, x0 + dr * 0.9], [rad * 0.6, rad, rad * 0.35], 5, lambda i, s2: jit(JALAPENO if s2 % 3 else (0.20, 0.26, 0.12), rng, 0.08), cap_end=True, cap_start=True)
        box(p, "b", (0.95, 0.94, 0.86), x0 + Vector((0, 0, rad * 0.85)), rad * 0.35, rad * 0.9, rad * 0.15, shade=False)   # the cheese filling


def hamper(v, lod):
    """The hamper hunt's twelve things, as on the Hut-trip board (refs/hut/dylan/board/, not in git; generic, no brands):
    0 a brie wheel, 1 the red-wax cheese (a wedge, burgundy marbled), 2 cheddar, 3 chorizo slices in a blue enamel bowl,
    4 green olives in a clear deli tub, 5 stuffed jalapenos in a blue enamel bowl, 6 a jar of red peppers, 7 the dips (green
    dip in a glass jar, hummus with chickpeas in a tub), 8 whipped feta with chilli in a terracotta bowl, 9 corn chips,
    10 a baguette, 11 two bottles of red. Life size here; the game draws them at HAMPER_SCALE."""
    rng = random.Random(5100 + v * 13)
    p = P()
    O = Vector((0, 0, 0))
    if v == 0:
        cyl(p, O, 0.12, 0, 0.05, 12, (0.95, 0.94, 0.88), col_top=(0.97, 0.96, 0.90), col_bot=(0.9, 0.88, 0.8), wedge=(0.0, 0.9, (0.98, 0.90, 0.62)))
    elif v == 1:
        # a wedge of red-wax cheese, burgundy marbled through, the wax down its back
        tip, a0, a1 = O, -0.45, 0.45
        r0 = 0.15
        q = [Vector((math.cos(a) * r0, math.sin(a) * r0, 0)) for a in (a0, a1)]
        for z0, z1 in ((0, 0.07),):
            top = [Vector((x.x, x.y, z1)) for x in q]
            p.tri("b", (0.62, 0.22, 0.28), Vector((0, 0, z1)), top[0], top[1])
            p.quad("b", (0.70, 0.30, 0.34), Vector((0, 0, z0)), q[0], top[0], Vector((0, 0, z1)))
            p.quad("b", (0.66, 0.26, 0.30), q[1], Vector((0, 0, z0)), Vector((0, 0, z1)), top[1])
            p.quad("b", (0.55, 0.06, 0.08), q[0], q[1], top[1], top[0])   # the wax
        for k in range(5):
            box(p, "b", (0.40, 0.08, 0.16), Vector((0.05 + 0.015 * k, (k - 2) * 0.012, 0.071)), 0.012, 0.004, 0.001, shade=False)   # the marbling
    elif v == 2:
        box(p, "b", (0.95, 0.80, 0.42), Vector((0, 0, 0.045)), 0.13, 0.07, 0.045)
    elif v == 3:
        z = enamel(p, O, 0.1, 0.055, rng)
        slices(p, Vector((0, 0, z)), 0.065, 16, rng, CHORIZO, CHORIZO_CUT, 0.022)
    elif v == 4:
        tub(p, O, 0.065, 0.075, (0.42, 0.52, 0.22), rng, bits=0.014, bitcol=OLIVE, nb=12)
    elif v == 5:
        z = enamel(p, O, 0.1, 0.055, rng)
        chillies(p, Vector((0, 0, z)), 0.09, 6, rng, 0.016)
    elif v == 6:
        jar(p, O, 0.055, 0.17, (0.75, 0.40, 0.35), (0.85, 0.12, 0.08), (0.78, 0.66, 0.30), rng, bits=(0.92, 0.18, 0.12))
    elif v == 7:
        jar(p, Vector((-0.07, 0, 0)), 0.05, 0.1, (0.60, 0.72, 0.45), (0.52, 0.70, 0.32), (0.85, 0.85, 0.82), rng)   # the green dip in its glass jar
        tub(p, Vector((0.07, 0, 0)), 0.06, 0.065, (0.86, 0.76, 0.56), rng, bits=0.009, bitcol=(0.78, 0.46, 0.22), nb=10)   # hummus, chickpeas on top
    elif v == 8:
        cyl(p, O, 0.075, 0, 0.05, 10, TERRACOTTA, col_bot=TERRACOTTA)
        cyl(p, O, 0.066, 0.045, 0.052, 10, (0.97, 0.96, 0.93), col_top=(0.98, 0.97, 0.95))
        for k in range(9):
            a = k * 0.8 + rng.random(); d = 0.045 * rng.random()
            box(p, "b", (0.82, 0.16, 0.06), Vector((math.cos(a) * d, math.sin(a) * d, 0.054)), 0.006, 0.006, 0.002, shade=False)   # the chilli flakes
    elif v == 9:
        # a plain chip bag, a pillow shape: yellow with a red band, crimped ends
        pts = [Vector((0, -0.11, 0.03)), Vector((0, -0.08, 0.06)), Vector((0, 0.08, 0.06)), Vector((0, 0.11, 0.03))]
        tube(p, pts, [(0.12, 0.02), (0.14, 0.05), (0.14, 0.05), (0.12, 0.02)], 6, lambda i, s2: (0.86, 0.18, 0.12) if i == 1 else jit((0.98, 0.78, 0.20), rng, 0.04), cap_end=True, cap_start=True)
    elif v == 10:
        pts = [Vector((-0.30, 0, 0.035)), Vector((-0.15, 0, 0.04)), Vector((0.15, 0, 0.04)), Vector((0.30, 0, 0.035))]
        tube(p, pts, [0.02, 0.035, 0.035, 0.02], 6, lambda i, s2: (0.92, 0.80, 0.55) if s2 == 4 and i == 1 else jit((0.80, 0.56, 0.28), rng, 0.05), squash=0.85, cap_end=True, cap_start=True)
    elif v == 11:
        bottle(p, Vector((-0.045, 0, 0)), 0.30, rng); bottle(p, Vector((0.05, 0.02, 0)), 0.30, rng)
    S = HAMPER_SCALE
    p.t = [(k, c, a * S, b * S, d * S) for k, c, a, b, d in p.t]
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.0


def esky(v, lod):
    """An esky for the tray: a white body, a blue lid propped open, handles. Base at z = 0, about 0.6 x 0.4 x 0.42 m."""
    p = P()
    WHITE, BLUE = (0.92, 0.94, 0.95), (0.18, 0.40, 0.78)
    box(p, "b", WHITE, Vector((0, 0, 0.17)), 0.30, 0.19, 0.17)
    box(p, "b", BLUE, Vector((0, 0, 0.015)), 0.31, 0.2, 0.015)
    box(p, "b", (0.12, 0.14, 0.18), Vector((0, 0, 0.33)), 0.26, 0.15, 0.012, shade=False)   # the dark inside, seen over the rim
    lid = [Vector((-0.31, -0.20, 0.35)), Vector((0.31, -0.20, 0.35)), Vector((0.31, -0.24, 0.73)), Vector((-0.31, -0.24, 0.73))]
    p.quad("b", BLUE, lid[0], lid[1], lid[2], lid[3]); p.quad("b", tuple(q * 0.8 for q in BLUE), lid[1], lid[0], lid[3], lid[2])
    for x in (-0.31, 0.31): box(p, "b", BLUE, Vector((x * 1.05, 0, 0.28)), 0.02, 0.07, 0.03)
    return p, 0.73, 0.0


def board(v, lod):
    """The Hut-trip board laid out on the picnic table, after Dylan's photos (refs/hut/dylan/board/; generic, no brands):
    a long weathered plank run down the table, and on it from one end: chorizo slices in a blue enamel bowl, green olives
    in a clear tub, baguette slices, the brie wheel, a jar of red peppers, crackers and seeded crispbreads, strawberries,
    raspberries and blueberries, chocolate biscuits, cheddar, stuffed jalapenos in two blue bowls, whipped feta with chilli
    in a terracotta bowl, celery and carrot sticks, hummus with chickpeas in a tub, shaved ham, the green dip in a glass
    jar, the red-wax cheese, salsa in a jar, corn chips, a small boxed cheese, and a third bowl of chilli sauce. Two bottles
    of red stand on the table either side. Base at z = 0 (the table top), along x, about 1.9 m."""
    rng = random.Random(8800)
    p = P()
    PLANK, PLANK2 = (0.58, 0.47, 0.34), (0.50, 0.40, 0.29)
    box(p, "b", PLANK, Vector((0, 0, 0.012)), 0.96, 0.11, 0.012)
    for k in range(4): box(p, "b", PLANK2, Vector((-0.75 + k * 0.5, -0.06 + 0.04 * (k % 2), 0.0245)), 0.18, 0.004, 0.0005, shade=False)   # weathered grain
    if v == 1:
        return p, 0.03, 0.0   # the bare plank, for the cutscene that lays it out
    T0 = 0.025
    def bit(x, y, col, hx, hy, hz, z=0.0):
        box(p, "b", jit(col, rng, 0.06), Vector((x, y, T0 + hz + z)), hx, hy, hz)
    # the near end: chorizo, olives, baguette, the brie, red peppers
    z = enamel(p, Vector((-0.86, 0.0, T0)), 0.075, 0.04, rng); slices(p, Vector((-0.86, 0.0, z)), 0.05, 12, rng, CHORIZO, CHORIZO_CUT, 0.017, 0.009)
    tub(p, Vector((-0.72, -0.04, T0)), 0.05, 0.05, (0.42, 0.52, 0.22), rng, bits=0.011, bitcol=OLIVE, nb=11)
    for k in range(3): bit(-0.74 + 0.03 * k, 0.06, (0.90, 0.80, 0.58), 0.014, 0.03, 0.012)
    cyl(p, Vector((-0.58, 0.02, T0)), 0.065, T0, T0 + 0.03, 12, (0.95, 0.94, 0.88), col_top=(0.97, 0.96, 0.90))
    jar(p, Vector((-0.66, 0.10, T0)), 0.032, 0.09, (0.75, 0.40, 0.35), (0.85, 0.12, 0.08), (0.78, 0.66, 0.30), rng, bits=(0.92, 0.18, 0.12))
    for k in range(5): bit(-0.50 + 0.022 * k, -0.06, (0.82, 0.70, 0.45), 0.013, 0.013, 0.003)                         # crackers
    for k in range(4): cyl(p, Vector((-0.42 + 0.025 * k, 0.07, T0)), 0.018, T0, T0 + 0.005, 6, (0.36, 0.24, 0.14), col_top=(0.42, 0.30, 0.18))   # seeded crispbreads
    for k in range(7): bit(-0.47 + 0.02 * k, 0.0 + 0.02 * (k % 3), (0.80, 0.12, 0.14) if k % 3 else (0.25, 0.22, 0.50), 0.007, 0.007, 0.007)   # berries
    for k in range(3): bit(-0.36 + 0.02 * k, -0.03, (0.30, 0.18, 0.10), 0.012, 0.018, 0.006)                         # chocolate biscuits
    bit(-0.30, 0.05, (0.95, 0.80, 0.42), 0.035, 0.02, 0.015)                                                         # cheddar
    # the middle: jalapenos in two blue bowls, the feta, celery and carrots, hummus, ham
    for bx in (-0.20, 0.06):
        z = enamel(p, Vector((bx, 0.02, T0)), 0.07, 0.04, rng); chillies(p, Vector((bx, 0.02, z)), 0.06, 4, rng, 0.011)
    cyl(p, Vector((-0.30, -0.08, T0)), 0.05, T0, T0 + 0.035, 10, TERRACOTTA, col_bot=TERRACOTTA)
    cyl(p, Vector((-0.30, -0.08, T0)), 0.044, T0 + 0.03, T0 + 0.037, 10, (0.97, 0.96, 0.93), col_top=(0.98, 0.97, 0.95))
    for k in range(6): bit(-0.30 + 0.02 * math.cos(k), -0.08 + 0.02 * math.sin(k), (0.82, 0.16, 0.06), 0.004, 0.004, 0.001, 0.012)
    for k in range(6): bit(-0.10 + 0.012 * k, -0.07, (0.95, 0.52, 0.15) if k % 2 else (0.60, 0.78, 0.40), 0.035, 0.004, 0.004)   # carrot and celery
    tub(p, Vector((-0.05, 0.08, T0)), 0.04, 0.045, (0.86, 0.76, 0.56), rng, bits=0.008, bitcol=(0.78, 0.46, 0.22), nb=8)
    for k in range(3): bit(0.18 + 0.02 * k, -0.04, (0.86, 0.60, 0.60), 0.03, 0.02, 0.004)                           # shaved ham
    for k in range(5): bit(0.20 + 0.022 * k, 0.06, (0.84, 0.72, 0.46), 0.013, 0.013, 0.003)                        # more crackers
    # the far end: the green dip, the red-wax cheese, salsa, corn chips, a boxed cheese, chilli sauce, baguette
    jar(p, Vector((0.36, 0.0, T0)), 0.034, 0.07, (0.60, 0.72, 0.45), (0.52, 0.70, 0.32), (0.85, 0.85, 0.82), rng)
    bit(0.46, 0.06, (0.62, 0.22, 0.28), 0.03, 0.018, 0.016); bit(0.46, 0.06, (0.55, 0.06, 0.08), 0.031, 0.004, 0.017)   # the red-wax wedge
    jar(p, Vector((0.55, -0.06, T0)), 0.03, 0.06, (0.70, 0.40, 0.25), (0.72, 0.30, 0.12), (0.92, 0.85, 0.30), rng)          # salsa
    for k in range(9):                                                                                               # corn chips
        a = rng.random() * 6.28; cx, cy = 0.60 + rng.random() * 0.12, 0.02 + rng.random() * 0.07
        p.tri("b", jit((0.96, 0.76, 0.28), rng, 0.06), Vector((cx, cy, T0 + 0.004)), Vector((cx + math.cos(a) * 0.03, cy + math.sin(a) * 0.03, T0 + 0.006)), Vector((cx + math.cos(a + 1) * 0.03, cy + math.sin(a + 1) * 0.03, T0 + 0.008)))
    cyl(p, Vector((0.70, -0.06, T0)), 0.03, T0, T0 + 0.025, 8, (0.95, 0.85, 0.30), col_top=(0.97, 0.95, 0.88))       # the boxed cheese
    z = enamel(p, Vector((0.84, 0.0, T0)), 0.055, 0.035, rng); cyl(p, Vector((0.84, 0.0, z)), 0.04, z, z + 0.01, 8, (0.62, 0.10, 0.06), col_top=(0.70, 0.14, 0.08))   # chilli sauce
    for k in range(4): bit(0.76 + 0.03 * k, 0.08, (0.90, 0.80, 0.58), 0.013, 0.028, 0.012)                          # baguette
    bottle(p, Vector((-0.30, -0.30, 0)), 0.30, rng); bottle(p, Vector((0.62, 0.28, 0)), 0.30, rng)                    # the two reds, on the table
    return p, 0.34, 0.0


def campfire(v, lod):
    """The fire off the chimney end: v 0, 1, 2 = levels 1, 2, 3 (each about twice the last). A ring of stones, split logs
    leant into a teepee, a bed of embers. The flames, sparks and glow are drawn by the game."""
    rng = random.Random(6600 + v)
    p = P()
    R = (0.55, 0.85, 1.45)[v]; L = (0.9, 1.4, 2.3)[v]; N = (6, 10, 16)[v]
    STONE = (0.50, 0.48, 0.45)
    for k in range(9 + 4 * v):
        a = k * 2 * math.pi / (9 + 4 * v)
        c = Vector((math.cos(a) * (R + 0.15), math.sin(a) * (R + 0.15), 0.08 + 0.05 * v))
        box(p, "b", jit(STONE, rng, 0.1), c, 0.12 + 0.04 * v, 0.1 + 0.03 * v, 0.08 + 0.04 * v)
    cyl(p, Vector((0, 0, 0)), R, 0.0, 0.03, 10, (0.10, 0.08, 0.07), col_top=(0.32, 0.12, 0.06))   # the bed of ash and embers
    top = Vector((0, 0, L * 0.82))
    for k in range(N):
        a = k * 2 * math.pi / N + rng.random() * 0.2
        foot = Vector((math.cos(a) * R * 0.85, math.sin(a) * R * 0.85, 0.02))
        tip = top + Vector((math.cos(a) * 0.08, math.sin(a) * 0.08, -rng.random() * 0.15))
        tube(p, [foot, tip], [0.07 + 0.03 * v, 0.05 + 0.02 * v], 4, lambda i, s: jit((0.30, 0.22, 0.15) if s % 2 else (0.70, 0.55, 0.38), rng, 0.08), cap_end=True)
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, R


def splitlog(v, lod):
    """One split log for the woodpile against the hut's west wall: a quarter-round of a 0.2 m log, 0.48 m long along x,
    pale split faces and dark bark; v 0-2 a little different."""
    rng = random.Random(6100 + v)
    p = P()
    L, r = 0.24 + 0.02 * v, 0.1 + 0.01 * v
    BARK, WOOD = (0.32, 0.24, 0.17), (0.76, 0.60, 0.40)
    ang = [math.pi * 0.5 * k / 3 for k in range(4)]
    for k in range(3):
        a0, a1 = ang[k], ang[k + 1]
        q0 = Vector((0, math.cos(a0) * r, math.sin(a0) * r)); q1 = Vector((0, math.cos(a1) * r, math.sin(a1) * r))
        p.quad("b", jit(BARK, rng, 0.08), q0 - Vector((L, 0, 0)), q1 - Vector((L, 0, 0)), q1 + Vector((L, 0, 0)), q0 + Vector((L, 0, 0)))
        for e in (-L, L): p.tri("b", jit(WOOD, rng, 0.05), Vector((e, 0, 0)), Vector((e, q0.y, q0.z)), Vector((e, q1.y, q1.z)))
    p.quad("b", jit(WOOD, rng, 0.05), Vector((-L, 0, 0)), Vector((L, 0, 0)), Vector((L, r, 0)), Vector((-L, r, 0)))
    p.quad("b", jit(tuple(c * 0.9 for c in WOOD), rng, 0.05), Vector((-L, 0, 0)), Vector((-L, 0, r)), Vector((L, 0, r)), Vector((L, 0, 0)))
    return p, r, 0.0


def rifle(v, lod):
    """Quayle's rifle, lying on its left side on the gravel: the same rifle he aims (rifle_parts), drawn a third over life
    size so it can be seen from the ute."""
    q = rifle_parts(0, sling=True)
    p = P()
    k = 1.3
    placed(p, q, Vector((0, 0, 0.03 * k)), Vector((k, 0, 0)), Vector((0, 0, -k)), Vector((0, k, 0)))   # on its side: its z (up) along the ground
    lo = min(min(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    p.t = [(kk, c, a - Vector((0, 0, lo)), b - Vector((0, 0, lo)), d - Vector((0, 0, lo))) for kk, c, a, b, d in p.t]
    return p, 0.12, 0.0


def tell(v, lod):
    """The secret way's tells. v 0: an old blazed tree, a dead grey trunk with a pale scar cut at chest height (and a
    stub of a branch). v 1: a fallen fence post with a twist of wire."""
    rng = random.Random(7700 + v)
    p = P()
    if v == 0:
        DEAD, BLAZE = (0.56, 0.54, 0.50), (0.92, 0.84, 0.66)
        pts = [Vector((0, 0, 0)), Vector((0.05, 0.02, 3.0)), Vector((0.12, 0.0, 6.0)), Vector((0.2, -0.05, 8.5))]
        tube(p, pts, [0.38, 0.32, 0.24, 0.12], 7, lambda i, s: jit(DEAD, rng, 0.06), cap_end=True)
        p.quad("b", BLAZE, Vector((0.37, -0.16, 1.05)), Vector((0.37, 0.16, 1.05)), Vector((0.36, 0.13, 1.75)), Vector((0.36, -0.13, 1.75)))   # the blaze, facing +x
        p.quad("b", BLAZE, Vector((0.371, 0.16, 1.05)), Vector((0.371, -0.16, 1.05)), Vector((0.361, -0.13, 1.75)), Vector((0.361, 0.13, 1.75)))
        tube(p, [Vector((0.1, 0.0, 5.0)), Vector((1.3, 0.4, 6.0))], [0.09, 0.04], 4, lambda i, s: jit(DEAD, rng, 0.06))
        return p, 8.5, 0.35
    POST = (0.45, 0.40, 0.33)
    box(p, "b", POST, Vector((0, 0, 0.07)), 0.75, 0.07, 0.07)
    for k in range(6):
        a = k * 0.9
        box(p, "b", (0.55, 0.55, 0.55), Vector((0.5 + 0.08 * math.cos(a), 0.08 * math.sin(a), 0.1 + 0.02 * k)), 0.06, 0.004, 0.004, shade=False)
    return p, 0.2, 0.0


def boulder(v, lod):
    """Creek stones: v 0-2 rounded granite boulders (1 to 2 m), grey with lichen."""
    rng = random.Random(9900 + v)
    p = P()
    s = (0.7, 1.0, 1.4)[v]
    cols = [(0.52, 0.51, 0.48), (0.46, 0.45, 0.43), (0.58, 0.57, 0.52), (0.50, 0.52, 0.42)]
    clump(p, Vector((0, 0, s * 0.45)), s, s * 0.85, s * 0.6, cols, rng, detail=lod == 0)
    return p, s * 1.05, s * 0.9


def campoven(v, lod):
    """A cast-iron camp oven of venison stew on the picnic table, beside the board: a squat black pot on three stub legs,
    a wire bail handle up, the lid off and leant against it so the stew shows (meat, carrot, potato in a brown gravy).
    In the board's frame (base at z = 0, the table top), at its far side; about 0.32 m across."""
    rng = random.Random(9100)
    p = P()
    IRON, IRON2, IRON3 = (0.20, 0.20, 0.21), (0.14, 0.14, 0.15), (0.28, 0.27, 0.26)
    C = Vector((0.22, -0.30, 0.0)); R0, Z0, Z1 = 0.15, 0.035, 0.17
    for k in range(3):   # the legs
        a = k * 2.094 + 0.5
        box(p, "b", IRON2, Vector((C.x + math.cos(a) * R0 * 0.7, C.y + math.sin(a) * R0 * 0.7, Z0 / 2)), 0.014, 0.014, Z0 / 2)
    cyl(p, C, R0 * 0.92, Z0, Z0 + 0.03, 12, IRON2, col_bot=IRON2)                       # the rounded base
    cyl(p, C, R0, Z0 + 0.03, Z1, 12, lambda k: jit(IRON, rng, 0.06))
    cyl(p, C, R0 * 1.06, Z1 - 0.012, Z1 + 0.004, 12, IRON3)                              # the rim
    cyl(p, C, R0 * 0.95, Z1 - 0.03, Z1 - 0.025, 12, (0.36, 0.20, 0.10), col_top=(0.42, 0.24, 0.12))   # the gravy
    for k in range(14):   # the stew's bits: meat, carrot, potato
        a = rng.random() * 6.283; d = R0 * 0.8 * math.sqrt(rng.random())
        col = ((0.30, 0.14, 0.08), (0.90, 0.46, 0.12), (0.86, 0.78, 0.52))[k % 3]
        box(p, "b", jit(col, rng, 0.08), Vector((C.x + math.cos(a) * d, C.y + math.sin(a) * d, Z1 - 0.02)), 0.017, 0.017, 0.011)
    for s in (-1, 1):   # the lugs
        box(p, "b", IRON2, Vector((C.x + s * R0 * 1.05, C.y, Z1 - 0.02)), 0.018, 0.022, 0.012)
    bail = [Vector((C.x + R0 * 1.08 * math.cos(t), C.y + 0.03 * math.sin(t), Z1 - 0.02 + 0.16 * math.sin(t))) for t in (k * math.pi / 6 for k in range(7))]
    tube(p, bail, [0.004] * 7, 3, lambda i, s2: IRON3, cap_end=False)                     # the wire bail handle, up
    # the lid, stood on its edge and leant against the pot's board side: a disc, a loop on its top face
    lr, lean = R0 * 1.03, 0.35
    u, w = Vector((1, 0, 0)), Vector((0, -math.sin(lean), math.cos(lean)))
    n = Vector((0, math.cos(lean), math.sin(lean)))   # its top face, outwards (towards the board, so the stew shows from the bench)
    Lc = Vector((C.x, C.y + R0 + 0.07, lr * math.cos(lean) + 0.005))
    ring = [Lc + u * math.cos(k * math.pi / 6) * lr + w * math.sin(k * math.pi / 6) * lr for k in range(13)]
    for k in range(12):
        p.tri("b", IRON, Lc + n * 0.012, ring[k + 1], ring[k])
        p.tri("b", IRON2, Lc, ring[k], ring[k + 1])
    box(p, "b", IRON3, Lc + n * 0.025, 0.03, 0.012, 0.012)
    return p, 0.33, 0.0


def lantern(v, lod):
    """A hurricane lantern on the picnic table (play test, 22 Oct: a little light for the board and the camp oven at night):
    a red tank, a glass globe glowing warm inside its wire guards, a vented cap and a wire bail. In the board's frame
    (base at z = 0, the table top), at the far end from the camp oven; about 0.33 m tall."""
    p = P()
    RED, RED2, WIRE, GLOW, GLOW2 = (0.72, 0.14, 0.10), (0.52, 0.09, 0.07), (0.30, 0.28, 0.26), (1.0, 0.90, 0.55), (1.0, 0.78, 0.38)
    C = Vector((-0.72, -0.28, 0.0))
    cyl(p, C, 0.07, 0.0, 0.05, 10, RED, col_top=RED2, col_bot=RED2)                          # the tank
    cyl(p, C, 0.055, 0.05, 0.065, 10, RED2, col_top=RED2)                                    # the burner collar
    pts = [Vector((C.x, C.y, z)) for z in (0.065, 0.10, 0.15, 0.20, 0.235)]
    tube(p, pts, [0.035, 0.05, 0.056, 0.048, 0.03], 8, lambda i, s2: GLOW if s2 % 2 else GLOW2, cap_end=False)   # the globe
    cyl(p, C, 0.012, 0.09, 0.15, 6, (1.0, 0.98, 0.85), col_top=(1.0, 0.98, 0.85))            # the flame inside, bright
    for k in range(4):   # the guard wires
        a = k * math.pi / 2 + 0.4
        box(p, "b", WIRE, Vector((C.x + math.cos(a) * 0.062, C.y + math.sin(a) * 0.062, 0.15)), 0.004, 0.004, 0.085, shade=False)
    cyl(p, C, 0.052, 0.235, 0.255, 10, RED, col_top=RED2)                                    # the cap
    cyl(p, C, 0.022, 0.255, 0.285, 6, RED2, col_top=RED2)                                    # the vent
    bail = [Vector((C.x + 0.06 * math.cos(t), C.y, 0.25 + 0.08 * math.sin(t))) for t in (k * math.pi / 6 for k in range(7))]
    tube(p, bail, [0.004] * 7, 3, lambda i, s2: WIRE, cap_end=False)
    return p, 0.33, 0.0


def jamlog(v, lod):
    """A big old log fallen across the creek (a log jam's), dark and mossy, 9 m along x, base at z = 0."""
    rng = random.Random(9700 + v)
    p = P()
    L = 9.0
    pts = [Vector((-L / 2 + L * i / 4, (rng.random() - 0.5) * 0.4, 0.45 + 0.1 * math.sin(i))) for i in range(5)]
    tube(p, pts, [0.42, 0.5, 0.5, 0.46, 0.38], 7 if lod == 0 else 5, lambda i, s: jit((0.26, 0.24, 0.17) if s % 3 else (0.30, 0.36, 0.20), rng, 0.08), cap_end=True, cap_start=True)
    for k in range(3):
        b0 = pts[1 + k]; tube(p, [b0, b0 + Vector(((rng.random() - 0.5) * 1.5, (rng.random() - 0.5) * 2.4, 0.8 + rng.random() * 0.6))], [0.12, 0.04], 4, lambda i, s: (0.28, 0.24, 0.18))
    return p, 1.6, 0.5


# ------------------------------------------------------------------ build D: the harder creek, the stock route, the dead trees
def deadfall(v, lod):
    """A fallen tree in the creek (build D): not firewood, a whole old trunk lying on the ground with its broken branch
    stubs, a root plate torn up at one end (v 0) or a snapped butt (v 1). Grey-brown, mossy underneath. Along x from
    -L/2 to L/2 (L = 14 m, 11 m), base at z = 0."""
    mossy = v >= 2; v = v % 2   # build D, part 2: v 2 and 3 are the same trunks grown over with moss (they never move)
    rng = random.Random(9500 + v)
    p = P()
    L = (14.0, 11.0)[v]
    n = 7
    pts = [Vector((-L / 2 + L * i / (n - 1), (rng.random() - 0.5) * 0.5, 0.0)) for i in range(n)]
    rad = [0.55 - 0.4 * i / (n - 1) for i in range(n)]
    for i in range(n): pts[i].z = rad[i] * 0.85
    DRY, MOSS = (0.60, 0.57, 0.51), (0.40, 0.44, 0.28)
    sides = 8 if lod == 0 else 5
    if mossy: DRY, MOSS = (0.30, 0.38, 0.18), (0.22, 0.31, 0.13)
    tube(p, pts, rad, sides, lambda i, s: jit(MOSS if (s == 0 and i % 2 == 0) or (mossy and s % 2) else DRY, rng, 0.08), cap_end=True, cap_start=v == 1)   # weathered grey, a little moss
    # branch stubs: snapped short, some up, some out
    for k in range(6 if lod == 0 else 3):
        i = 1 + k % (n - 2); b0 = pts[i] + Vector((0, 0, rad[i] * 0.5))
        d = Vector(((rng.random() - 0.5) * 0.6, (rng.random() - 0.5) * 1.6, 0.5 + rng.random() * 0.8)).normalized()
        ln = 0.6 + rng.random() * 1.4
        tube(p, [b0, b0 + d * ln], [0.13 * (1 - i / n) + 0.06, 0.04], 4, lambda i2, s: jit(DRY, rng, 0.1))
    if v == 0:   # the root plate, on its edge at the butt: a ragged disc of earth and roots
        c = pts[0] + Vector((-0.2, 0, 0.6))
        for k in range(10 if lod == 0 else 6):
            a0 = k * 2 * math.pi / (10 if lod == 0 else 6); a1 = (k + 1) * 2 * math.pi / (10 if lod == 0 else 6)
            r0 = 1.5 + 0.4 * rng.random(); r1 = 1.5 + 0.4 * rng.random()
            q0 = c + Vector((0, math.cos(a0) * r0, math.sin(a0) * r0 * 0.8)); q1 = c + Vector((0, math.cos(a1) * r1, math.sin(a1) * r1 * 0.8))
            p.tri("b", jit((0.34, 0.26, 0.18), rng, 0.1), c + Vector((-0.25, 0, 0)), q1, q0)
            p.tri("b", jit((0.28, 0.21, 0.15), rng, 0.1), c + Vector((0.15, 0, 0)), q0, q1)
    H = max(max(a.z, b.z, c.z) for _, _, a, b, c in p.t)
    return p, H, 0.5


def stag(v, lod):
    """A stag: a standing dead gum (build D), grey and barkless, 6 to 12 m (v 0-2: 7, 9.5, 12 m), a jagged broken top,
    a few broken limbs reaching up, pale silver wood with dark cracks and a fire scar at the foot. No leaves, so it reads
    apart from the live trunks at a glance. Its trunk is solid (radius returned); the game topples it."""
    rng = random.Random(8800 + v * 13)
    p = P()
    H = (7.0, 9.5, 12.0)[v]
    r0 = (0.30, 0.36, 0.42)[v]
    SILVER, CRACK, SCAR = (0.66, 0.65, 0.63), (0.36, 0.35, 0.34), (0.16, 0.13, 0.11)
    n = 6
    lean = Vector(((rng.random() - 0.5) * 0.8, (rng.random() - 0.5) * 0.8, 0))
    pts = [Vector((0, 0, 0)) + lean * (i / (n - 1)) ** 2 + Vector((0, 0, H * i / (n - 1))) for i in range(n)]
    rad = [r0 * (1.0 - 0.75 * i / (n - 1)) for i in range(n)]
    sides = 7 if lod == 0 else 5
    tube(p, pts, rad, sides, lambda i, s: jit(SCAR if (i == 0 and s in (0, 1)) else CRACK if s % 3 == 2 else SILVER, rng, 0.06), cap_end=False)
    # the jagged top: three splinters
    top = pts[-1]
    for k in range(3):
        a = k * 2.1 + rng.random()
        q = top + Vector((math.cos(a) * rad[-1] * 0.7, math.sin(a) * rad[-1] * 0.7, 0))
        p.tri("b", jit(SILVER, rng, 0.08), q, top + Vector((math.cos(a + 1.0) * rad[-1], math.sin(a + 1.0) * rad[-1], 0)), top + Vector((math.cos(a + 0.5) * 0.05, math.sin(a + 0.5) * 0.05, 0.5 + rng.random() * 0.9)))
    # broken limbs, angled up and out, snapped off
    for k in range(3 + v if lod == 0 else 2):
        i = 2 + (k % (n - 3)); b0 = pts[i]
        a = rng.random() * 6.283
        d = Vector((math.cos(a), math.sin(a), 0.9 + rng.random() * 0.8)).normalized()
        ln = 1.0 + rng.random() * (1.0 + 0.4 * v)
        mid = b0 + d * ln * 0.55 + Vector((0, 0, 0.2))
        tube(p, [b0, mid, b0 + d * ln + Vector((0, 0, 0.35))], [rad[i] * 0.45, rad[i] * 0.3, 0.05], 4, lambda i2, s: jit(SILVER if s % 2 else CRACK, rng, 0.08))
    return p, H + 0.5, r0


def marker(v, lod):
    """The old stock route's markers (build D), from the Monaro down to the coast: v 0 an old blazed stump with a
    carved arrow; v 1 a horseshoe nailed to a dead stump; v 2 a rusted stockman's sign on a post; v 3 an old cattle-yard
    post with a rail stub and wire; v 4 a stone cairn."""
    rng = random.Random(8300 + v)
    p = P()
    GREY, BLAZE, RUST, IRON = (0.55, 0.52, 0.47), (0.88, 0.80, 0.62), (0.55, 0.28, 0.14), (0.30, 0.28, 0.27)
    if v in (0, 1):   # a stump 2.2 m tall
        tube(p, [Vector((0, 0, 0)), Vector((0.03, 0.02, 1.2)), Vector((0.05, 0, 2.2))], [0.32, 0.28, 0.22], 7 if lod == 0 else 5, lambda i, s: jit(GREY, rng, 0.07), cap_end=True)
        if v == 0:   # the blaze and the arrow cut into it (facing +x)
            p.quad("b", BLAZE, Vector((0.30, -0.15, 0.9)), Vector((0.30, 0.15, 0.9)), Vector((0.28, 0.12, 1.6)), Vector((0.28, -0.12, 1.6)))
            for (a, b, c) in (((0.311, 0.0, 1.5), (0.311, -0.09, 1.35), (0.311, 0.09, 1.35)),):
                p.tri("b", (0.30, 0.22, 0.14), Vector(a), Vector(b), Vector(c))
            p.quad("b", (0.30, 0.22, 0.14), Vector((0.311, -0.025, 1.0)), Vector((0.311, 0.025, 1.0)), Vector((0.311, 0.025, 1.36)), Vector((0.311, -0.025, 1.36)))
        else:   # a rusty horseshoe, heels down, nailed at eye height
            for k in range(9):
                a0 = math.pi * k / 9; a1 = math.pi * (k + 1) / 9
                q0 = Vector((0.27, math.cos(a0) * 0.09, 1.55 + math.sin(a0) * 0.10)); q1 = Vector((0.27, math.cos(a1) * 0.09, 1.55 + math.sin(a1) * 0.10))
                p.quad("b", RUST, q0, q1, q1 + Vector((0, 0, -0.03)) * 1 + (q1 - Vector((0.27, 0, 1.55))) * -0.25, q0 + (q0 - Vector((0.27, 0, 1.55))) * -0.25)
            for y in (-0.09, 0.09): box(p, "b", RUST, Vector((0.27, y * 0.95, 1.50)), 0.012, 0.018, 0.05)
        return p, 2.3, 0.3
    if v == 2:   # a post and a rusted plate
        box(p, "b", (0.42, 0.36, 0.28), Vector((0, 0, 0.9)), 0.07, 0.07, 0.9)
        box(p, "b", RUST, Vector((0.08, 0, 1.45)), 0.012, 0.36, 0.20)
        for k in range(3):   # the faded lettering: three pale bars
            box(p, "b", (0.78, 0.70, 0.52), Vector((0.093, 0, 1.55 - 0.1 * k)), 0.002, 0.26 - 0.05 * k, 0.025, shade=False)
        for y in (-0.3, 0.3): box(p, "b", (0.35, 0.18, 0.10), Vector((0.094, y, 1.31)), 0.002, 0.04, 0.03, shade=False)   # rust runs
        return p, 1.9, 0.1
    if v == 3:   # a heavy split yard post with a rail stub and old wire
        box(p, "b", (0.44, 0.40, 0.34), Vector((0, 0, 0.8)), 0.13, 0.11, 0.8)
        box(p, "b", (0.40, 0.36, 0.30), Vector((0, 0.6, 1.25)), 0.07, 0.6, 0.06)
        box(p, "b", (0.40, 0.36, 0.30), Vector((0, 0.45, 0.65)), 0.06, 0.45, 0.05)
        for k in range(8): box(p, "b", IRON, Vector((0.14, -0.1 + 0.03 * math.cos(k), 0.4 + 0.12 * k)), 0.004, 0.06, 0.004, shade=False)
        return p, 1.6, 0.15
    # a stone cairn
    for k in range(12 if lod == 0 else 6):
        lv = k // 4; a = (k % 4) * 1.57 + lv * 0.6
        c = Vector((math.cos(a) * (0.45 - 0.12 * lv), math.sin(a) * (0.45 - 0.12 * lv), 0.12 + 0.24 * lv))
        clump(p, c, 0.22 - 0.03 * lv, 0.2 - 0.03 * lv, 0.14, [GREY, (0.48, 0.47, 0.44), (0.60, 0.58, 0.53)], rng, detail=lod == 0)
    clump(p, Vector((0, 0, 0.85)), 0.15, 0.13, 0.12, [GREY], rng, detail=lod == 0)
    return p, 1.0, 0.0


def camp(v, lod):
    """The old cattleman's camp by the water before the ford (build D): v 0 the stone fireplace, a low horseshoe of
    stacked stones with a back wall, a green-hide-and-iron crossbar and a blackened billy hanging on it; v 1 the rusted
    trunk, closed; v 2 the trunk open, its lid back."""
    rng = random.Random(8100 + v)
    p = P()
    STONE = [(0.52, 0.50, 0.46), (0.45, 0.44, 0.41), (0.58, 0.56, 0.51)]
    if v == 0:
        for k in range(15):   # the horseshoe, open to +x
            a = math.pi * 0.25 + k * math.pi * 1.5 / 14
            for lv in range(2 if k in (0, 14) else 3 if 4 <= k <= 10 else 2):
                c = Vector((math.cos(a) * 0.75, math.sin(a) * 0.75, 0.13 + 0.24 * lv))
                clump(p, c, 0.2, 0.17, 0.13, STONE, rng, detail=lod == 0)
        for k in range(6):   # the back wall a little higher
            c = Vector((-0.85, -0.5 + 0.2 * k, 0.85)); clump(p, c, 0.16, 0.13, 0.13, STONE, rng, detail=lod == 0)
        IRON = (0.20, 0.19, 0.18)
        for y in (-0.7, 0.7): box(p, "b", IRON, Vector((0.05, y, 0.55)), 0.02, 0.02, 0.55)
        box(p, "b", IRON, Vector((0.05, 0, 1.08)), 0.02, 0.72, 0.02)
        cyl(p, Vector((0.05, 0, 0)), 0.11, 0.55, 0.78, 8, (0.12, 0.11, 0.10), col_top=(0.18, 0.17, 0.16), col_bot=(0.08, 0.07, 0.07))   # the billy
        box(p, "b", IRON, Vector((0.05, 0, 0.93)), 0.006, 0.006, 0.15, shade=False)   # its wire handle up to the bar
        cyl(p, Vector((0.05, 0, 0)), 0.45, 0.0, 0.03, 10, (0.10, 0.09, 0.08), col_top=(0.16, 0.14, 0.12))   # old ash
        return p, 1.2, 0.0
    RUST, DARK, BAND = (0.52, 0.27, 0.14), (0.30, 0.16, 0.09), (0.36, 0.30, 0.24)
    box(p, "b", RUST, Vector((0, 0, 0.24)), 0.48, 0.30, 0.24)
    for x in (-0.3, 0.3): box(p, "b", BAND, Vector((x, 0, 0.245)), 0.03, 0.305, 0.245)
    if v == 1:   # the lid, shut, a little domed
        box(p, "b", RUST, Vector((0, 0, 0.52)), 0.49, 0.31, 0.05)
        box(p, "b", DARK, Vector((0.0, -0.31, 0.38)), 0.05, 0.01, 0.05)   # the hasp
        return p, 0.6, 0.0
    box(p, "b", (0.10, 0.08, 0.06), Vector((0, 0, 0.47)), 0.45, 0.27, 0.005, shade=False)   # dark inside
    box(p, "b", RUST, Vector((0, 0.36, 0.72)), 0.49, 0.05, 0.31)   # the lid back on its hinges
    return p, 1.05, 0.0


def creekrock(v, lod):
    """Rocks in the creek's rock gardens (build D): flattish granite, 0.35 to 0.7 m, half sunk; the ute crawls over them
    (drawn only: the ground under them is the rough part)."""
    rng = random.Random(9300 + v)
    p = P()
    s = (0.35, 0.5, 0.7)[v]
    cols = [(0.50, 0.49, 0.46), (0.44, 0.43, 0.41), (0.57, 0.55, 0.50), (0.47, 0.49, 0.40)]
    clump(p, Vector((0, 0, s * 0.18)), s, s * 0.8, s * 0.45, cols, rng, detail=lod == 0)
    return p, s * 0.6, 0.0


# ------------------------------------------------------------------ build D, part 2: the creek's drive-over things
def stump(v, lod):
    """An old stump in the creek (v 0, 1): grey-brown, 0.4 m radius, 0.6 / 0.45 m tall, a jagged top, roots flaring."""
    rng = random.Random(8700 + v)
    p = P()
    H = (0.6, 0.45)[v]
    sides = 8 if lod == 0 else 5
    tube(p, [Vector((0, 0, -0.05)), Vector((0, 0, H * 0.6)), Vector((0.02, 0, H))], [0.46, 0.4, 0.37], sides, lambda i, s: jit((0.42, 0.36, 0.28) if s % 2 else (0.33, 0.28, 0.22), rng, 0.08), cap_end=False)
    for k in range(sides):   # the jagged top
        a0 = k * 2 * math.pi / sides; a1 = (k + 1) * 2 * math.pi / sides
        q0 = Vector((math.cos(a0) * 0.37, math.sin(a0) * 0.37, H)); q1 = Vector((math.cos(a1) * 0.37, math.sin(a1) * 0.37, H))
        p.tri("b", jit((0.62, 0.52, 0.38), rng, 0.1), q0, q1, Vector((0, 0, H + 0.12 * rng.random())))
    for k in range(4 if lod == 0 else 2):   # roots
        a = k * 1.57 + rng.random() * 0.5
        tube(p, [Vector((math.cos(a) * 0.3, math.sin(a) * 0.3, 0.12)), Vector((math.cos(a) * 0.8, math.sin(a) * 0.8, 0.0))], [0.12, 0.05], 4, lambda i, s: jit((0.34, 0.28, 0.21), rng, 0.08))
    return p, H + 0.1, 0.4


def branch(v, lod):
    """A fallen branch (v 0-2): a bent grey limb 2.5 to 3.5 m along x, 0.08 m thick, with two or three side twigs."""
    rng = random.Random(8600 + v)
    p = P()
    L = (2.6, 3.0, 3.5)[v]
    pts = [Vector((-L / 2 + L * i / 3, (rng.random() - 0.5) * 0.4, 0.08)) for i in range(4)]
    tube(p, pts, [0.09, 0.08, 0.06, 0.04], 5 if lod == 0 else 3, lambda i, s: jit((0.52, 0.48, 0.42), rng, 0.1), cap_end=True)
    for k in range(3 if lod == 0 else 1):
        b0 = pts[1 + k % 2]; d = Vector(((rng.random() - 0.5) * 0.8, (rng.random() - 0.5) * 1.6, 0.05)).normalized()
        tube(p, [b0, b0 + d * (0.5 + rng.random() * 0.6)], [0.035, 0.015], 3, lambda i, s: jit((0.48, 0.44, 0.38), rng, 0.1))
    return p, 0.2, 0.0


def twigs(v, lod):
    """Twig litter (v 0, 1): a scatter of small sticks and bark over about 1.5 m (drawn only; nothing to feel)."""
    rng = random.Random(8500 + v)
    p = P()
    for k in range(9 if lod == 0 else 4):
        c = Vector(((rng.random() - 0.5) * 1.5, (rng.random() - 0.5) * 1.5, 0.03)); a = rng.random() * 6.283; l = 0.25 + rng.random() * 0.45
        d = Vector((math.cos(a), math.sin(a), 0))
        tube(p, [c - d * l / 2, c + d * l / 2], [0.025, 0.015], 3, lambda i, s: jit((0.46, 0.40, 0.32) if k % 3 else (0.58, 0.46, 0.34), rng, 0.12))
    return p, 0.06, 0.0


def smalllog(v, lod):
    """A small dead log (v 0, 1), not firewood (it's rotten): 3 m along x, 0.2 m thick, grey-brown with a broken end."""
    rng = random.Random(8400 + v)
    p = P()
    L = (3.0, 2.4)[v]; r = (0.2, 0.16)[v]
    pts = [Vector((-L / 2 + L * i / 3, (rng.random() - 0.5) * 0.2, r)) for i in range(4)]
    tube(p, pts, [r, r * 0.95, r * 0.9, r * 0.8], 6 if lod == 0 else 4, lambda i, s: jit((0.45, 0.40, 0.33) if s % 3 else (0.36, 0.40, 0.25), rng, 0.08), cap_end=True, cap_start=True)
    return p, 2 * r, r


def wetlog(v, lod):
    """A wet log in the creek (v 0, 1): dark, soaked, slick, 3 m along x, 0.22 m thick. The ute shoves it aside or smashes it;
    never firewood."""
    rng = random.Random(8300 + v * 7)
    p = P()
    L = (3.2, 2.6)[v]; r = (0.22, 0.18)[v]
    pts = [Vector((-L / 2 + L * i / 3, (rng.random() - 0.5) * 0.25, r)) for i in range(4)]
    tube(p, pts, [r, r, r * 0.92, r * 0.85], 7 if lod == 0 else 4, lambda i, s: jit((0.20, 0.17, 0.13) if s % 2 else (0.26, 0.22, 0.16), rng, 0.08), cap_end=True, cap_start=True)
    for k in range(2):   # a dark sheen of water down its top
        a = pts[k + 1]; p.quad("b", (0.32, 0.36, 0.38), a + Vector((-0.4, -0.05, r * 1.01)), a + Vector((0.4, -0.05, r * 1.01)), a + Vector((0.4, 0.05, r * 1.01)), a + Vector((-0.4, 0.05, r * 1.01)))
    return p, 2 * r, r


CATALOGUE = [
    ("sambar", 4, sambar, (0, 1), "D"),
    ("edgescrub", 3, edgescrub, (0, 1), "E"),
    ("boomgate", 1, boomgate, (0,), "J"),
    ("armyman", 2, armyman, (0, 1), "A"),
    ("hamper", 12, hamper, (0,), "X"),
    ("esky", 1, esky, (0,), "X"),
    ("board", 2, board, (0,), "X"),
    ("campoven", 1, campoven, (0,), "X"),
    ("lantern", 1, lantern, (0,), "X"),
    ("campfire", 3, campfire, (0,), "X"),
    ("rifle", 1, rifle, (0,), "X"),
    ("splitlog", 3, splitlog, (0,), "X"),
    ("tell", 2, tell, (0, 1), "X"),
    ("boulder", 3, boulder, (0, 1), "C"),
    ("jamlog", 2, jamlog, (0, 1), "C"),
    ("deadfall", 4, deadfall, (0, 1), "C"),
    ("stag", 3, stag, (0, 1), "C"),
    ("marker", 5, marker, (0, 1), "C"),
    ("camp", 3, camp, (0, 1), "C"),
    ("creekrock", 3, creekrock, (0, 1), "C"),
    ("stump", 2, stump, (0, 1), "C"),
    ("branch", 3, branch, (0, 1), "C"),
    ("twigs", 2, twigs, (0, 1), "C"),
    ("smalllog", 2, smalllog, (0, 1), "C"),
    ("wetlog", 2, wetlog, (0, 1), "C"),
]


def f3(v): return f"{v.x:.3f} {v.z:.3f} {v.y:.3f}"   # Blender (x, y, z up) -> game (x, y up, z)


built = []
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
with open(OUT, "w") as f:
    f.write("# Bush Drift models (builds B and C), made by tools/make_world.py (procedural; sambar: see docs/forest/deer.md)\n")
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
    want_v = os.environ.get("RENDER_V")   # with RENDER_ONLY: just this variant, the camera framed on it
    for name, v, lod, H, p in built:
        if lod or (only and name != only): continue
        if want_v is not None and only and v != int(want_v): continue
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
    if only and want_v is not None:   # framed on the model: a three-quarter view from its bounding box
        xs = [q[0] for o in sc.collection.objects if o.type == 'MESH' for q in [tuple(v.co) for v in o.data.vertices]]
        allv = [tuple(v.co) for o in sc.collection.objects if o.type == 'MESH' for v in o.data.vertices]
        lo = Vector([min(a[i] for a in allv) for i in range(3)]); hi = Vector([max(a[i] for a in allv) for i in range(3)])
        c = (lo + hi) / 2; size = max((hi - lo).length, 0.3)
        d = Vector((0.55, -1.0, 0.45)).normalized() * size * 1.35
        co.location = c + d
        co.rotation_euler = (c - co.location).to_track_quat('-Z', 'Y').to_euler(); cam.lens = 40
    sun = bpy.data.lights.new("s", 'SUN'); sun.energy = 3.5; so = bpy.data.objects.new("s", sun); sc.collection.objects.link(so)
    so.rotation_euler = (math.radians(50), math.radians(10), math.radians(30))
    w = bpy.data.worlds.new("w"); sc.world = w; w.use_nodes = True; w.node_tree.nodes["Background"].inputs[0].default_value = (0.85, 0.78, 0.65, 1)
    sc.render.engine = 'BLENDER_EEVEE' if 'BLENDER_EEVEE' in [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items] else 'BLENDER_WORKBENCH'
    sc.render.resolution_x, sc.render.resolution_y = (900, 900) if only else (1600, 420)
    sc.render.filepath = os.path.join(RENDER_DIR, "lineup.png")
    bpy.ops.render.render(write_still=True)
    print("RENDER", sc.render.filepath)
