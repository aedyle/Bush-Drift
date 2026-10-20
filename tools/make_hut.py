"""Alexanders Hut and the campground bits: blender -b --python tools/make_hut.py -- OUT.txt [RENDER_DIR]

Modelled from docs/hut/reference.md and docs/hut/measurements.md (the OSM footprint, the NSW aerial, photos
kept privately in refs/hut/, matched against the game in session 3b): a 1922 slab hut, an L of a main gable
block running WNW-ESE and a gabled wing off its back half to the NNE, the verandah in the corner between them;
vertical grey slabs on the ESE front gable with a 12-pane window, red-brown boards and a small window down the
south-west side, a slab firebox and a tall tapering rusty flue centred on the WNW gable, a second 12-pane window
in the wing's gable end over a tall stone plinth, rusty corrugated roofs with silver patches, two galvanised tanks. Props: the pit toilet,
the picnic table, the fire ring with its bench and woodpile, the old sulky plough, a bollard, a fence post
and the NPWS sign. Flat colours, low-poly. Output is in the plants format (see tools/make_plants.py), in
the game's axes with x = east, z = north, origin at the hut's OSM centroid (the props at their own origin).
Fixed seeds: byte-identical on every run.
"""
import sys, os, math, random
from mathutils import Vector

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = args[0] if args else "assets-built/hut.txt"
RENDER_DIR = args[1] if len(args) > 1 else None
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

class P:
    def __init__(s): s.t = []
    def tri(s, col, a, b, c, k="b"): s.t.append((k, tuple(max(0.0, min(1.0, x)) for x in col), Vector(a), Vector(b), Vector(c)))
    def quad(s, col, a, b, c, d, k="b"): s.tri(col, a, b, c, k); s.tri(col, a, c, d, k)

def shade(c, f): return (c[0] * f, c[1] * f, c[2] * f)

# --------------------------------------------------------------------- the hut's own frame
# From the OSM outline (E, N metres about the centroid) and the session 3b photo match (docs/hut/measurements.md):
# the main block runs WNW-ESE along the outline's long south-west side C -> B, with its ridge that way; the
# 12-pane front gable faces ESE (the approach), the chimney gable WNW (the car park). A gabled wing runs NNE off
# the back half, its gable end (a second 12-pane window, a tall stone plinth) facing NNE; the verandah fills the
# corner between the wing and the front half. u runs along the main block from its WNW gable (C) to the front
# gable (towards B), v from the south-west wall to the north-east (towards D).
C = Vector((-6.3, -1.5, 0)); B = Vector((1.1, -5.9, 0)); D = Vector((-1.8, 6.2, 0))
U = (B - C).normalized()
V = (D - C); V = (V - U * V.dot(U)).normalized()      # square it up (the outline is 91 degrees)
Z = Vector((0, 0, 1))
def H(u, v, z): return C + U * u + V * v + Z * z      # hut frame -> E, N, up

ML, MW = 8.6, 4.4                                      # main block: length (C -> B) x width
WL, WD = 5.4, 4.5                                      # wing: along the main block from the WNW end x out to the NNE
VD = 3.0                                               # verandah depth (NNE of the main block, ESE of the wing)
FL, EAVE = 0.35, 2.4                                   # floor on the plinth; wall plate above the floor
PITCH, WPITCH = math.radians(42), math.radians(33)     # main and wing roof pitch (photo 3 and own photo 10)
ZT = FL + EAVE
RIDGE = ZT + MW / 2 * math.tan(PITCH)
WRIDGE = ZT + WL / 2 * math.tan(WPITCH)

GREY_SLAB = (0.70, 0.65, 0.57); BROWN_SLAB = (0.37, 0.29, 0.23); RED_SLAB = (0.50, 0.31, 0.23)
RUST = (0.46, 0.24, 0.16); RUST2 = (0.52, 0.28, 0.18); RUST_DARK = (0.34, 0.20, 0.14); ZINC = (0.66, 0.67, 0.66); SILVER = (0.80, 0.81, 0.80)
STONE = (0.55, 0.54, 0.50); IRON_RED = (0.44, 0.22, 0.16); GLASS = (0.16, 0.18, 0.22); CREAM = (0.86, 0.83, 0.74); DOOR = (0.45, 0.20, 0.15)
GALV = (0.80, 0.78, 0.73); GALV2 = (0.66, 0.64, 0.59)   # greys a touch warm (session 3b): the palette keeps them grey, not lilac

def box(p, u0, v0, z0, u1, v1, z1, col, f=H, top=True):
    q = [f(u0, v0, z0), f(u1, v0, z0), f(u1, v1, z0), f(u0, v1, z0), f(u0, v0, z1), f(u1, v0, z1), f(u1, v1, z1), f(u0, v1, z1)]
    p.quad(shade(col, 0.95), q[0], q[1], q[5], q[4]); p.quad(shade(col, 0.9), q[1], q[2], q[6], q[5])
    p.quad(shade(col, 0.95), q[2], q[3], q[7], q[6]); p.quad(shade(col, 0.9), q[3], q[0], q[4], q[7])
    if top: p.quad(col, q[4], q[5], q[6], q[7])

def wall_slabs(p, a, b, z0, z1, cols, vertical, width, rng, f=H):
    """A wall from a=(u,v) to b along the ground, z0..z1, as slabs: vertical strips or horizontal boards."""
    a = Vector((a[0], a[1])); b = Vector((b[0], b[1])); L = (b - a).length
    if vertical:
        n = max(1, round(L / width))
        for i in range(n):
            p0 = a.lerp(b, i / n); p1 = a.lerp(b, (i + 1) / n)
            c = cols[rng.randrange(len(cols))]; c = shade(c, 0.92 + 0.16 * rng.random())
            top = z1 - 0.06 * rng.random()
            p.quad(c, f(p0.x, p0.y, z0), f(p1.x, p1.y, z0), f(p1.x, p1.y, top), f(p0.x, p0.y, top))
    else:
        n = max(1, round((z1 - z0) / width))
        for i in range(n):
            za = z0 + (z1 - z0) * i / n; zb = z0 + (z1 - z0) * (i + 1) / n
            c = cols[rng.randrange(len(cols))]; c = shade(c, 0.9 + 0.18 * rng.random())
            p.quad(c, f(a.x, a.y, za), f(b.x, b.y, za), f(b.x, b.y, zb), f(a.x, a.y, zb))

def flat(p, col, pts, f=H):
    q = [f(*x) for x in pts]
    for i in range(1, len(q) - 1): p.tri(col, q[0], q[i], q[i + 1])

def roof_sheet(p, a0, a1, b0, b1, n, rng, cols, silver_share=0.12, f=H):
    """A corrugated-iron slope from the edge a0-a1 (eave) to b0-b1 (ridge), cut into n sheets down the slope."""
    for i in range(n):
        t0, t1 = i / n, (i + 1) / n
        e0 = a0.lerp(a1, t0); e1 = a0.lerp(a1, t1); r0 = b0.lerp(b1, t0); r1 = b0.lerp(b1, t1)
        c = SILVER if rng.random() < silver_share else cols[rng.randrange(len(cols))]
        c = shade(c, 0.9 + 0.2 * rng.random())
        mid0 = e0.lerp(r0, 0.5 + 0.2 * (rng.random() - 0.5)); mid1 = e1.lerp(r1, 0.5 + 0.2 * (rng.random() - 0.5))
        c2 = shade(c, 0.85) if rng.random() < 0.5 else ZINC if rng.random() < 0.3 else c
        p.quad(c, f(*e0), f(*e1), f(*mid1), f(*mid0)); p.quad(c2, f(*mid0), f(*mid1), f(*r1), f(*r0))

def cylinder(p, centre, r, z0, z1, sides, cols, f=lambda x: x, top_col=None, r1=None):
    r1 = r if r1 is None else r1
    for k in range(sides):
        a0, a1 = k * 2 * math.pi / sides, (k + 1) * 2 * math.pi / sides
        c = cols[k % len(cols)]
        p0 = centre + Vector((math.cos(a0) * r, math.sin(a0) * r, z0)); p1 = centre + Vector((math.cos(a1) * r, math.sin(a1) * r, z0))
        q0 = centre + Vector((math.cos(a0) * r1, math.sin(a0) * r1, z1)); q1 = centre + Vector((math.cos(a1) * r1, math.sin(a1) * r1, z1))
        p.quad(c, f(p0), f(p1), f(q1), f(q0))
        if top_col: p.tri(top_col, f(centre + Vector((0, 0, z1))), f(q0), f(q1))

def gable_fill(p, u_or_v, at, a, b, z0, z1, col, rng, n=6, f=H):
    """The triangle of a gable end as horizontal boards: across a..b (in the other axis) at `at`, from z0 to the apex z1."""
    m = (a + b) / 2
    for i in range(n):
        za = z0 + (z1 - z0) * i / n; zb = z0 + (z1 - z0) * (i + 1) / n
        wa = (b - a) / 2 * (1 - i / n); wb = (b - a) / 2 * (1 - (i + 1) / n)
        c = shade(col, 0.88 + 0.12 * rng.random())
        if u_or_v == "u": p.quad(c, f(at, m - wa, za), f(at, m + wa, za), f(at, m + wb, zb), f(at, m - wb, zb))
        else: p.quad(c, f(m - wa, at, za), f(m + wa, at, za), f(m + wb, at, zb), f(m - wb, at, zb))

def window12(p, plane, at, c, zc, w, h, out, f=H):
    """A 12-pane window (3 wide x 4 high), cream frame, on a wall plane ('u' = the wall u=at, 'v' = the wall v=at)."""
    def P(a, z, d): return f(at + d, a, z) if plane == "u" else f(a, at + d, z)
    p.quad(CREAM, P(c - w / 2 - 0.08, zc - h / 2 - 0.08, out * 0.03), P(c + w / 2 + 0.08, zc - h / 2 - 0.08, out * 0.03),
           P(c + w / 2 + 0.08, zc + h / 2 + 0.08, out * 0.03), P(c - w / 2 - 0.08, zc + h / 2 + 0.08, out * 0.03))
    for i in range(3):
        for j in range(4):
            a0 = c - w / 2 + w * i / 3 + 0.03; a1 = c - w / 2 + w * (i + 1) / 3 - 0.03
            z0 = zc - h / 2 + h * j / 4 + 0.03; z1 = zc - h / 2 + h * (j + 1) / 4 - 0.03
            p.quad(shade(GLASS, 1.0 + 0.4 * ((i + j) % 2)), P(a0, z0, out * 0.05), P(a1, z0, out * 0.05), P(a1, z1, out * 0.05), P(a0, z1, out * 0.05))

def hut():
    rng = random.Random(1922)
    p = P()
    # dry-stone plinth: low under the main block, tall under the wing where the ground falls away to the NNE
    box(p, -0.1, -0.1, -0.6, ML + 0.1, MW + 0.1, FL, STONE)
    box(p, -0.1, MW, -1.3, WL + 0.1, MW + WD + 0.1, FL, STONE)
    for k in range(9):                                            # big granite stones along the wing's gable end
        u = 0.3 + k * 0.58
        box(p, u - 0.3, MW + WD - 0.05, -1.3, u + 0.28, MW + WD + 0.14, FL - 0.02, shade(STONE, 0.85 + 0.25 * rng.random()))
    grey = [GREY_SLAB, shade(GREY_SLAB, 0.9), (0.66, 0.62, 0.55)]
    red = [RED_SLAB, shade(RED_SLAB, 0.9), (0.47, 0.30, 0.24)]
    brown = [BROWN_SLAB, shade(BROWN_SLAB, 0.88)]
    # front gable (ESE, u = ML): vertical silver slabs, horizontal boards in the triangle, the 12-pane window
    wall_slabs(p, (ML, 0), (ML, MW), FL, ZT, grey, True, 0.3, rng)
    gable_fill(p, "u", ML, 0, MW, ZT, RIDGE, GREY_SLAB, rng)
    window12(p, "u", ML, MW / 2 + 0.1, FL + 1.45, 1.15, 1.5, 1)
    # south-west wall: red-brown horizontal boards from the chimney end, a small window, then vertical grey slabs to the front
    wall_slabs(p, (0, 0), (4.4, 0), FL, ZT, red, False, 0.24, rng)
    wall_slabs(p, (4.4, 0), (ML, 0), FL, ZT, grey, True, 0.3, rng)
    p.quad(CREAM, H(3.05, -0.03, FL + 1.0), H(3.75, -0.03, FL + 1.0), H(3.75, -0.03, FL + 1.9), H(3.05, -0.03, FL + 1.9))
    p.quad(GLASS, H(3.12, -0.05, FL + 1.06), H(3.68, -0.05, FL + 1.06), H(3.68, -0.05, FL + 1.84), H(3.12, -0.05, FL + 1.84))
    # chimney gable (WNW, u = 0): red-brown boards; the wing's side wall carries on to the NNE
    wall_slabs(p, (0, 0), (0, MW), FL, ZT, red + brown, False, 0.24, rng)
    gable_fill(p, "u", 0, 0, MW, ZT, RIDGE, RED_SLAB, rng)
    wall_slabs(p, (0, MW), (0, MW + WD), FL, ZT, brown + [RED_SLAB], False, 0.24, rng)
    # wing: the gable end (NNE) in dark brown boards with the second 12-pane window; its ESE wall is the verandah's back
    wall_slabs(p, (0, MW + WD), (WL, MW + WD), FL, ZT, [shade(BROWN_SLAB, 0.8), shade(BROWN_SLAB, 0.7)], False, 0.22, rng)
    gable_fill(p, "v", MW + WD, 0, WL, ZT, WRIDGE, shade(BROWN_SLAB, 0.78), rng, 7)
    window12(p, "v", MW + WD, 1.9, FL + 1.55, 1.1, 1.45, 1)
    iron = lambda a, b: wall_slabs(p, a, b, FL, ZT, [IRON_RED, shade(IRON_RED, 0.86), RUST_DARK], True, 0.6, rng)
    iron((WL, MW), (WL, MW + VD))
    wall_slabs(p, (WL, MW + VD), (WL, MW + WD), FL, ZT, brown, False, 0.24, rng)
    # main block's NNE wall under the verandah: vertical slabs and the red-brown door
    wall_slabs(p, (WL, MW), (ML, MW), FL, ZT, grey, True, 0.3, rng)
    p.quad(DOOR, H(5.9, MW + 0.03, FL), H(6.7, MW + 0.03, FL), H(6.7, MW + 0.03, FL + 1.9), H(5.9, MW + 0.03, FL + 1.9))
    # verandah: floor, three posts along its open ESE side (one painted red), the sink bench and the old stove
    flat(p, (0.50, 0.42, 0.34), [(WL, MW, FL + 0.01), (ML, MW, FL + 0.01), (ML, MW + VD, FL + 0.01), (WL, MW + VD, FL + 0.01)])
    zl = FL + 2.05
    for k, vv in enumerate((MW + 0.12, MW + VD / 2, MW + VD - 0.12)):
        box(p, ML - 0.16, vv - 0.06, FL, ML - 0.04, vv + 0.06, zl, (0.62, 0.22, 0.16) if k == 1 else (0.55, 0.50, 0.44))
    box(p, WL + 0.05, MW + 0.6, FL, WL + 0.55, MW + 1.8, FL + 0.85, STONE)
    box(p, WL + 0.05, MW + 2.1, FL, WL + 0.7, MW + 2.8, FL + 0.9, (0.25, 0.23, 0.22))
    # main roof: gable, ridge WNW-ESE, small overhangs, silver ridge capping, gutters
    o, og = 0.3, 0.2
    ze = ZT - o * math.tan(PITCH)
    for side in (0, 1):
        v_e = -o if side == 0 else MW + o
        a0, a1 = Vector((-og, v_e, ze)), Vector((ML + og, v_e, ze))
        b0, b1 = Vector((-og, MW / 2, RIDGE)), Vector((ML + og, MW / 2, RIDGE))
        roof_sheet(p, a0, a1, b0, b1, 16, rng, [RUST, RUST2, RUST_DARK, RUST], 0.03 if side == 0 else 0.08)
        p.quad(GALV2, H(-og, v_e, ze), H(ML + og, v_e, ze), H(ML + og, v_e, ze - 0.12), H(-og, v_e, ze - 0.12))
    for side in (-1, 1):
        p.quad(SILVER, H(-og, MW / 2 + side * 0.25, RIDGE - 0.2), H(ML + og, MW / 2 + side * 0.25, RIDGE - 0.2), H(ML + og, MW / 2, RIDGE + 0.04), H(-og, MW / 2, RIDGE + 0.04))
    # wing roof: gable, ridge NNE, each slope starting at its valley with the main roof's NNE slope
    t, tw = math.tan(PITCH), math.tan(WPITCH)
    def valley(u):   # v where the wing's slope (at u along it) meets the main roof's NNE slope
        z = ZT + min(u, WL - u) * tw
        return MW / 2 + (RIDGE - z) / t
    vo = MW + WD + 0.2
    for a_u, b_u in ((-0.25, WL / 2), (WL + 0.25, WL / 2)):
        ue = 0.0 if a_u < 0 else WL
        roof_sheet(p, Vector((a_u, valley(ue), ZT - 0.25 * tw)), Vector((a_u, vo, ZT - 0.25 * tw)),
                   Vector((b_u, valley(WL / 2), WRIDGE)), Vector((b_u, vo, WRIDGE)), 9, rng, [RUST, RUST2, RUST_DARK, SILVER], 0.1)
    p.quad(SILVER, H(WL / 2 - 0.22, valley(WL / 2), WRIDGE - 0.14), H(WL / 2 - 0.22, vo, WRIDGE - 0.14), H(WL / 2, vo, WRIDGE + 0.04), H(WL / 2, valley(WL / 2), WRIDGE + 0.04))
    p.quad(SILVER, H(WL / 2, valley(WL / 2), WRIDGE + 0.04), H(WL / 2, vo, WRIDGE + 0.04), H(WL / 2 + 0.22, vo, WRIDGE - 0.14), H(WL / 2 + 0.22, valley(WL / 2), WRIDGE - 0.14))
    # verandah roof: a flatter skillion carrying the wing's ESE slope on down to the front (photo 3, own photo 06),
    # rusty with silver streaks
    zw = ZT - 0.25 * tw
    roof_sheet(p, Vector((ML + 0.3, MW + o, zl)), Vector((ML + 0.3, MW + VD + 0.2, zl)),
               Vector((WL + 0.25, MW + o, zw)), Vector((WL + 0.25, MW + VD + 0.2, zw)), 6, rng, [RUST, RUST2, SILVER], 0.3)
    # chimney: on the WNW gable, centred; slab firebox, a rusty hood, a tall tapering sheet-iron flue with a dented top
    v0, v1 = MW / 2 - 0.85, MW / 2 + 0.85
    box(p, -0.95, v0, -0.4, 0.0, v1, FL + 2.1, GREY_SLAB, top=False)
    wall_slabs(p, (-0.96, v0), (-0.96, v1), FL - 0.3, FL + 2.1, [GREY_SLAB, shade(GREY_SLAB, 0.88)], True, 0.28, rng)
    for vv in (v0 - 0.01, v1 + 0.01):
        wall_slabs(p, (-0.95, vv), (0.0, vv), FL - 0.3, FL + 2.1, [GREY_SLAB, shade(GREY_SLAB, 0.85)], True, 0.3, rng)
    cu, cv, fb = -0.48, MW / 2, FL + 2.1
    hood = [H(-0.98, v0 - 0.05, fb), H(0.05, v0 - 0.05, fb), H(0.05, v1 + 0.05, fb), H(-0.98, v1 + 0.05, fb)]
    top4 = [H(cu - 0.3, cv - 0.3, fb + 0.8), H(cu + 0.3, cv - 0.3, fb + 0.8), H(cu + 0.3, cv + 0.3, fb + 0.8), H(cu - 0.3, cv + 0.3, fb + 0.8)]
    for i in range(4):
        p.quad(shade(RUST, 0.85 + 0.1 * i), hood[i], hood[(i + 1) % 4], top4[(i + 1) % 4], top4[i])
    segs = [(fb + 0.8, 0.30, 0.0, 0.0), (fb + 2.6, 0.26, 0.0, 0.0), (fb + 3.7, 0.21, -0.05, 0.04), (fb + 4.35, 0.20, -0.1, 0.06)]
    for i in range(len(segs) - 1):
        za, ra, da, ea = segs[i]; zb, rb, db, eb = segs[i + 1]
        col = RUST if i == 0 else RUST_DARK if i == 2 else shade(RUST, 0.8)
        ring_a = [H(cu + da + sx * ra, cv + ea + sy * ra, za) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        ring_b = [H(cu + db + sx * rb, cv + eb + sy * rb, zb) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        for k in range(4):
            p.quad(shade(col, 0.85 + 0.08 * k), ring_a[k], ring_a[(k + 1) % 4], ring_b[(k + 1) % 4], ring_b[k])
    # tanks: the slim one against the south-west wall by the chimney corner; the squat one on its stone plinth in front of the verandah
    def hf(q): return H(q.x, q.y, q.z)
    cylinder(p, Vector((0.6, -0.58, -0.1)), 0.52, 0, 2.05, 12, [GALV, GALV2], hf, top_col=shade(GALV, 0.9))
    cylinder(p, Vector((ML + 0.75, MW + 1.25, -0.3)), 1.15, 0, 0.85, 10, [STONE, shade(STONE, 0.9)], hf, top_col=STONE)
    cylinder(p, Vector((ML + 0.75, MW + 1.25, 0.55)), 1.07, 0, 1.5, 14, [GALV, GALV2], hf, top_col=shade(GALV, 0.9))
    return p

# --------------------------------------------------------------------- props (each about its own origin, x east, y north)
def I(u, v, z): return Vector((u, v, z))

def toilet():   # NPWS single-cubicle pit toilet: Colorbond walls, skillion roof, a vent pipe (no photo found: a standard NPWS type)
    rng = random.Random(5); p = P()
    box(p, -0.9, -1.1, -0.2, 0.9, 1.1, 0.15, (0.55, 0.53, 0.49), f=I)
    box(p, -0.75, -0.95, 0.15, 0.75, 0.95, 2.25, (0.46, 0.49, 0.38), f=I, top=False)
    p.quad((0.36, 0.39, 0.30), I(0.76, -0.4, 0.2), I(0.76, 0.4, 0.2), I(0.76, 0.4, 2.0), I(0.76, -0.4, 2.0))
    p.quad((0.42, 0.45, 0.42), I(-1.0, -1.2, 2.55), I(1.1, -1.2, 2.25), I(1.1, 1.2, 2.25), I(-1.0, 1.2, 2.55))
    cylinder(p, Vector((-0.5, 0.6, 0)), 0.08, 2.2, 3.1, 6, [(0.3, 0.3, 0.3)], top_col=(0.2, 0.2, 0.2))
    return p

def picnic():
    p = P(); wood = (0.50, 0.44, 0.37)
    box(p, -0.95, -0.4, 0.72, 0.95, 0.4, 0.78, wood, f=I)
    for s in (-1, 1):
        box(p, -0.95, s * 0.55 - 0.13, 0.42, 0.95, s * 0.55 + 0.13, 0.47, shade(wood, 0.92), f=I)
        for e in (-1, 1):
            box(p, e * 0.7 - 0.05, s * 0.38 - 0.05, 0, e * 0.7 + 0.05, s * 0.38 + 0.05, 0.75, shade(wood, 0.8), f=I)
            box(p, e * 0.7 - 0.05, s * 0.55 - 0.05, 0, e * 0.7 + 0.05, s * 0.55 + 0.05, 0.45, shade(wood, 0.8), f=I)
    return p

def firering():
    rng = random.Random(7); p = P()
    cylinder(p, Vector((0, 0, 0)), 0.55, -0.05, 0.3, 12, [(0.30, 0.20, 0.15), (0.24, 0.17, 0.13)])
    p.quad((0.20, 0.19, 0.18), I(-0.45, -0.45, 0.05), I(0.45, -0.45, 0.05), I(0.45, 0.45, 0.05), I(-0.45, 0.45, 0.05))
    box(p, -1.0, 1.2, 0.40, 1.0, 1.45, 0.46, (0.52, 0.45, 0.38), f=I)                       # plank bench
    for e in (-0.8, 0.8): box(p, e - 0.08, 1.24, 0, e + 0.08, 1.41, 0.40, (0.42, 0.37, 0.31), f=I)
    for k in range(7):                                                                        # woodpile
        x = 1.6 + (k % 3) * 0.24 - (k // 3) * 0.12; z = 0.12 + (k // 3) * 0.2
        cylinder(p, Vector((x, -0.9, z)), 0.11, -0.01, 0.01, 5, [(0.55, 0.42, 0.30)])
        box(p, x - 0.1, -1.4, z - 0.1, x + 0.1, -0.4, z + 0.1, shade((0.50, 0.38, 0.27), 0.9 + 0.1 * (k % 2)), f=I)
    return p

def plough():   # a horse-drawn sulky plough, rusted orange-brown, two spoked wheels, three curved pans, a long pole
    p = P(); r = (0.55, 0.30, 0.16); r2 = (0.45, 0.24, 0.13)
    def wheel(c, rad):
        for k in range(14):
            a0, a1 = k * 0.4488, (k + 1) * 0.4488
            o0 = c + Vector((0, math.cos(a0) * rad, math.sin(a0) * rad)); o1 = c + Vector((0, math.cos(a1) * rad, math.sin(a1) * rad))
            i0 = c + Vector((0, math.cos(a0) * rad * 0.86, math.sin(a0) * rad * 0.86)); i1 = c + Vector((0, math.cos(a1) * rad * 0.86, math.sin(a1) * rad * 0.86))
            p.quad(r, o0 + Vector((0.04, 0, 0)), o1 + Vector((0.04, 0, 0)), i1 + Vector((0.04, 0, 0)), i0 + Vector((0.04, 0, 0)))
            p.quad(r2, o0 - Vector((0.04, 0, 0)), o1 - Vector((0.04, 0, 0)), o1 + Vector((0.04, 0, 0)), o0 + Vector((0.04, 0, 0)))
        for k in range(6):
            a = k * 1.047
            e = c + Vector((0, math.cos(a) * rad * 0.86, math.sin(a) * rad * 0.86))
            p.quad(r2, c + Vector((0, 0, 0.03)), e + Vector((0, 0, 0.03)), e - Vector((0, 0, 0.03)), c - Vector((0, 0, 0.03)))
    wheel(Vector((-0.8, 0, 0.45)), 0.45); wheel(Vector((0.8, 0.2, 0.3)), 0.30)
    box(p, -0.85, -0.05, 0.45, 0.85, 0.05, 0.52, r2, f=I)                                     # axle beam
    box(p, -0.05, -1.0, 0.5, 0.05, 1.0, 0.58, r, f=I)                                         # frame
    for k in range(3):                                                                        # curved pans
        y = -0.6 + 0.6 * k
        a = Vector((-0.35, y, 0.05)); b = Vector((0.25, y + 0.15, 0.05)); c = Vector((0.3, y + 0.25, 0.45)); d = Vector((-0.3, y + 0.05, 0.5))
        p.quad(shade(r, 1.1), a, b, c, d)
        box(p, -0.03, y - 0.03, 0.3, 0.03, y + 0.03, 0.55, r2, f=I)
    box(p, -0.04, 1.0, 0.45, 0.04, 3.4, 0.5, r2, f=I)                                         # the pole, trailing
    p.quad(r, I(-0.04, 3.4, 0.45), I(0.04, 3.4, 0.45), I(0.04, 3.6, 0.02), I(-0.04, 3.6, 0.02))
    box(p, -0.03, -0.6, 0.55, 0.03, -0.54, 1.1, r2, f=I); box(p, -0.2, -0.75, 1.08, 0.2, -0.4, 1.13, r, f=I)   # seat stalk and seat
    return p

def bollard():   # weathered round timber, grey-brown (session 3b: browner, rounder, as in the car-park photo)
    p = P(); g = (0.60, 0.53, 0.43)
    cylinder(p, Vector((0, 0, 0)), 0.15, -0.1, 0.75, 9, [g, shade(g, 0.88), shade(g, 1.08)], top_col=(0.62, 0.58, 0.50))
    return p

def fencepost():
    p = P(); g = (0.56, 0.58, 0.51)
    box(p, -0.08, -0.06, -0.1, 0.08, 0.07, 1.35, g, f=I)
    box(p, -0.1, -0.08, 0.2, 0.0, 0.08, 0.6, (0.48, 0.55, 0.42), f=I)   # lichen
    return p

def sign():
    p = P()
    for e in (-0.5, 0.5): box(p, e - 0.05, -0.05, 0, e + 0.05, 0.05, 0.65, (0.45, 0.40, 0.33), f=I)
    box(p, -0.7, -0.06, 0.3, 0.7, 0.0, 0.62, (0.12, 0.33, 0.20), f=I)
    p.quad((0.92, 0.92, 0.88), I(-0.55, -0.07, 0.42), I(0.55, -0.07, 0.42), I(0.55, -0.07, 0.5), I(-0.55, -0.07, 0.5))
    return p

def rail():   # the hitching rail: a long round crossbar on two posts, grey weathered timber (photo 3); along x
    p = P(); g = (0.62, 0.57, 0.48)
    for e in (-1.3, 1.3): cylinder(p, Vector((e, 0, 0)), 0.11, -0.1, 1.15, 7, [g, shade(g, 0.88)], top_col=shade(g, 1.05))
    box(p, -1.55, -0.07, 0.98, 1.55, 0.07, 1.12, shade(g, 1.04), f=I)
    return p

def gate():   # an old rusty iron farm gate, half open on its post by the big tree (photo 3, NPWS 04); along x
    p = P(); r = (0.48, 0.27, 0.16); r2 = (0.38, 0.22, 0.14)
    box(p, -0.08, -0.08, -0.1, 0.08, 0.08, 1.3, (0.50, 0.46, 0.40), f=I)
    for z in (0.15, 0.45, 0.75, 1.05):
        box(p, 0.1, -0.02, z, 3.3, 0.02, z + 0.04, r, f=I)
    for x in (0.1, 1.2, 2.25, 3.3):
        box(p, x, -0.025, 0.12, x + 0.05, 0.025, 1.1, r2, f=I)
    p.quad(r2, I(0.12, 0, 0.17), I(3.3, 0, 1.05), I(3.3, 0, 1.1), I(0.12, 0, 0.22))
    return p

ITEMS = [("hut", hut), ("toilet", toilet), ("picnic", picnic), ("firering", firering), ("plough", plough),
         ("bollard", bollard), ("fencepost", fencepost), ("sign", sign), ("rail", rail), ("gate", gate)]

def f3(v): return f"{v.x:.3f} {v.z:.3f} {v.y:.3f}"
built = {}
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
with open(OUT, "w") as f:
    f.write("# Alexanders Hut and campground props, made by tools/make_hut.py (see docs/hut/reference.md)\n")
    for name, fn in ITEMS:
        p = fn()
        top = max(max(a.z, b.z, c.z) for k, col, a, b, c in p.t)
        f.write(f"plant {name} 0 0 {top:.3f} 0 {len(p.t)} H\n")
        for k, c, a, b, d in p.t:
            f.write(f"t {k} {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}  {f3(a)}  {f3(b)}  {f3(d)}\n")
        built[name] = p
        print(f"PROP {name:10s} {len(p.t):4d} triangles, {top:.2f} m tall")

if RENDER_DIR:
    import bpy
    os.makedirs(RENDER_DIR, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    def add(p, at, name):
        me = bpy.data.meshes.new(name); verts, faces, cols = [], [], []
        for k, c, a, b, d in p.t:
            faces.append((len(verts), len(verts) + 1, len(verts) + 2)); verts += [tuple(a + at), tuple(b + at), tuple(d + at)]; cols += [(*c, 1.0)] * 3
        me.from_pydata(verts, [], faces)
        attr = me.color_attributes.new("Col", 'FLOAT_COLOR', 'CORNER'); attr.data.foreach_set("color", [x for c in cols for x in c])
        o = bpy.data.objects.new(name, me); sc.collection.objects.link(o); return o
    add(built["hut"], Vector((0, 0, 0)), "hut")
    for name, at in (("toilet", (-12, 8, 0)), ("picnic", (-9, 3, 0)), ("firering", (-6, 8, 0)), ("plough", (10, -12, 0)), ("sign", (-2, -10, 0))):
        add(built[name], Vector(at), name)
    for k in range(6): add(built["bollard"], Vector((-8 + 2.5 * k, -8.5, 0)), "b")
    for k in range(4): add(built["fencepost"], Vector((6 + 3 * k, -6 + 0.8 * k, 0)), "f")
    me = bpy.data.meshes.new("g"); me.from_pydata([(-30, -30, -0.02), (30, -30, -0.02), (30, 30, -0.02), (-30, 30, -0.02)], [], [(0, 1, 2, 3)])
    g = bpy.data.objects.new("g", me); sc.collection.objects.link(g)
    m = bpy.data.materials.new("g"); m.diffuse_color = (0.55, 0.58, 0.32, 1); me.materials.append(m)
    sc.render.engine = 'BLENDER_WORKBENCH'
    sh = sc.display.shading; sh.light = 'STUDIO'; sh.color_type = 'VERTEX'; sh.show_shadows = True
    sc.display.light_direction = (-0.5, -0.5, 0.7)
    sc.world = bpy.data.worlds.new("w"); sc.world.color = (0.75, 0.80, 0.88)
    sc.render.resolution_x, sc.render.resolution_y = 1200, 800
    cam = bpy.data.cameras.new("c"); cam.lens = 32
    co = bpy.data.objects.new("c", cam); sc.collection.objects.link(co); sc.camera = co
    for name, at, tgt in (("hut-front", (2, -26, 5), (-1, -2, 2)), ("hut-chimney", (-19, 6, 3.5), (-3, 1, 2.6)),
                          ("hut-above", (14, -22, 16), (0, -2, 1)), ("hut-plough", (16, -26, 4), (2, -6, 1.5))):
        co.location = Vector(at); co.rotation_euler = (Vector(tgt) - Vector(at)).to_track_quat('-Z', 'Y').to_euler()
        sc.render.filepath = os.path.join(RENDER_DIR, name + ".png")
        bpy.ops.render.render(write_still=True)
    print("RENDERS", RENDER_DIR)
