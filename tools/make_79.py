"""Cut the Sketchfab LandCruiser 79 (off-road) down to a game car: blender -b --python tools/make_79.py -- SRC.gltf OUT.car [RENDER_DIR]   (VOX=0.07 voxel size in m)

Bakes the scene to world space, scales it so the wheelbase is the real 3.18 m, throws away the cabin
interior, the underside and small parts, flattens the materials to a handful of colours, joins the body
as coloured surface voxels (greedy-meshed), and cuts four wheels (tyre + rim) with their pivots at their centres.
Writes the game's text .car format in the game's frame (x right, y up, z forward, origin at the
centre of mass: 1.37 m behind the front axle, 0.19 m above the wheel centres) plus docs renders.
"""
import bpy, bmesh, sys, os, math
from mathutils import Vector, Matrix

args = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = args[0], args[1]
RENDER_DIR = args[2] if len(args) > 2 else None
WHEELBASE, FRONT_TO_COM, ORIGIN_ABOVE_WHEEL = 3.18, 1.37, 0.19

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=SRC)
for o in list(bpy.context.scene.objects):
    if o.type == 'MESH':
        mw = o.matrix_world.copy()
        if o.data.users > 1: o.data = o.data.copy()
        o.parent = None
        o.data.transform(mw)
        o.matrix_world.identity()
for o in list(bpy.context.scene.objects):
    if o.type != 'MESH': bpy.data.objects.remove(o)

def bbox(o):
    xs = [v.co for v in o.data.vertices]
    mn = Vector((min(v.x for v in xs), min(v.y for v in xs), min(v.z for v in xs)))
    mx = Vector((max(v.x for v in xs), max(v.y for v in xs), max(v.z for v in xs)))
    return mn, mx

# --- scale: wheel centres from the tyres (model: x left, -y forward, z up)
tyres = [o for o in bpy.context.scene.objects if o.name.startswith("rxx_trx_tire") and o.data.materials and o.data.materials[0].name == "x3_wheel"]
centres = []
for o in tyres:
    mn, mx = bbox(o); centres.append((mn + mx) / 2)
front_y = min(c.y for c in centres); rear_y = max(c.y for c in centres)
s = WHEELBASE / (rear_y - front_y)
wz = sum(c.z for c in centres) / len(centres)
# model -> game: x_g = -x, y_g = z, z_g = -y ; then origin
origin = Vector((0, front_y + FRONT_TO_COM / s, wz + ORIGIN_ABOVE_WHEEL / s))
M = Matrix(((-s, 0, 0, 0), (0, 0, s, 0), (0, -s, 0, 0), (0, 0, 0, 1))) @ Matrix.Translation(-origin)
for o in bpy.context.scene.objects:
    o.data.transform(M)
    if M.determinant() < 0: o.data.flip_normals()
print("scale", s)

# --- what to keep, and its colour
SILVER, SILVER2, BLACK, DARK, RIM, GLASS = (0.78, 0.79, 0.81), (0.62, 0.63, 0.66), (0.09, 0.09, 0.10), (0.28, 0.28, 0.30), (0.30, 0.30, 0.32), (0.13, 0.17, 0.22)
LAMP_W, LAMP_R, LAMP_O = (1.0, 0.95, 0.80), (0.80, 0.12, 0.10), (0.95, 0.55, 0.12)
DROP_MATS = {"plastic_int", "interior_third", "interior_fifth", "interior_sixth", "wood", "white", "material", "red", "red_toyota",
             "blue", "brakedisk", "x3_mix", "metal_int"}
DROP_NAMES = ("interior_", "door_l_kart", "door_r_kart", "door_l_lock", "door_r_lock", "bottom_parts", "transmission", "bodyshell_SUB10",
              "brakedisk", "nipel", "text_", "Text_", "lattice_logo", "number_", "lock_", "handle_", "seat", "trailer_bolt", "mirror_bolt")
def colour(mat, name):
    if name.startswith(("headlight_glass", "halogen_lightbulb", "sphere_lightbulb", "bumper_f_glass")): return "lamp", LAMP_W
    m = mat
    if m == "carpaint": return "tri", SILVER
    if m == "carpaint_second": return "tri", SILVER2
    if m in ("black", "black_gloss", "plastic_black"): return "tri", BLACK
    if m == "chrome": return "tri", BLACK if name.startswith("air_intake") else DARK
    if m in ("clearglass", "mirror"): return "tri", GLASS
    if m == "redglass": return "lamp", LAMP_R
    if m in ("orangeglass", "diamont1"): return "lamp", LAMP_O
    return None

wheel_parts = {k: [] for k in ("FL", "FR", "RL", "RR")}
groups = {}
dropped = kept = 0
for o in list(bpy.context.scene.objects):
    mat = o.data.materials[0].name if o.data.materials else ""
    mn, mx = bbox(o); c = (mn + mx) / 2
    if o.name.startswith(("rxx_trx_tire", "_wheel_lf_")):
        if mat in ("x3_wheel",): col = BLACK
        else: col = RIM
        key = ("F" if c.z > 0 else "R") + ("R" if c.x > 0 else "L")
        wheel_parts[key].append((o, col)); continue
    cl = None if (o.name.startswith(DROP_NAMES) or mat in DROP_MATS) else colour(mat, o.name)
    if cl is None or (mx - mn).length < 0.06:
        dropped += len(o.data.polygons); bpy.data.objects.remove(o); continue
    kept += len(o.data.polygons)
    groups.setdefault(cl, []).append(o)
print("faces kept", kept, "dropped", dropped)

def join(objs, name):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs: o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1: bpy.ops.object.join()
    o = bpy.context.view_layer.objects.active; o.name = name
    return o

def tri_count(o): return sum(len(p.vertices) - 2 for p in o.data.polygons)


# ---------------------------------------------------------------- voxel body
# The body as surface voxels: every kept triangle is sampled densely, each voxel takes the colour most of
# its samples have (lamps and glass win ties against paint so they survive), and the voxel faces that
# face an empty neighbour are merged into the largest same-coloured rectangles (greedy meshing).
import numpy as np
VOX = float(os.environ.get("VOX", "0.07"))
PRIO = {}  # colour -> (kind, priority)
cols = []
pts_all, cid_all = [], []
for cl, objs in groups.items():
    kind, rgb = cl
    ci = len(cols); cols.append((kind, rgb))
    for o in objs:
        me = o.data
        me.calc_loop_triangles()
        nt = len(me.loop_triangles)
        if nt == 0: continue
        co = np.empty(len(me.vertices) * 3, dtype=np.float32); me.vertices.foreach_get("co", co); co = co.reshape(-1, 3)
        tv = np.empty(nt * 3, dtype=np.int32); me.loop_triangles.foreach_get("vertices", tv); tv = tv.reshape(-1, 3)
        A, B, C = co[tv[:, 0]], co[tv[:, 1]], co[tv[:, 2]]
        L = np.maximum(np.maximum(np.linalg.norm(B - A, axis=1), np.linalg.norm(C - B, axis=1)), np.linalg.norm(A - C, axis=1))
        n = np.clip(np.ceil(L / (VOX * 0.4)).astype(int), 1, 60)
        for k in np.unique(n):
            sel = n == k
            a, b, c = A[sel], B[sel], C[sel]
            for i in range(k + 1):
                for j in range(k + 1 - i):
                    u, v = i / k, j / k
                    pts_all.append(a + u * (b - a) + v * (c - a)); cid_all.append(np.full(len(a), ci, dtype=np.int16))
P = np.concatenate(pts_all); CI = np.concatenate(cid_all)
lo = P.min(axis=0) - VOX
idx = np.floor((P - lo) / VOX).astype(int)
dims = idx.max(axis=0) + 2
print("voxel grid", dims, "samples", len(P))
nc = len(cols)
counts = np.zeros((*dims, nc), dtype=np.int32)
np.add.at(counts, (idx[:, 0], idx[:, 1], idx[:, 2], CI), 1)
weight = np.array([3.0 if k == "lamp" else (2.0 if rgb == GLASS else 1.0) for k, rgb in cols])
filled = counts.sum(axis=3) > 0
grid = np.where(filled, np.argmax(counts * weight, axis=3), -1)
print("filled voxels", int(filled.sum()))

def greedy_faces(grid):
    """Yield (colour, corner, du, dv, normal) rectangles of exposed voxel faces."""
    out = []
    X, Y, Z = grid.shape
    for axis in range(3):
        u_ax, v_ax = [a for a in range(3) if a != axis]
        for sgn in (1, -1):
            nb = np.full(grid.shape, -1)
            sl_src = [slice(None)] * 3; sl_dst = [slice(None)] * 3
            if sgn > 0: sl_src[axis] = slice(1, None); sl_dst[axis] = slice(0, -1)
            else: sl_src[axis] = slice(0, -1); sl_dst[axis] = slice(1, None)
            nb[tuple(sl_dst)] = grid[tuple(sl_src)]
            exposed = np.where((grid >= 0) & (nb < 0), grid, -1)
            for d in range(grid.shape[axis]):
                sl = [slice(None)] * 3; sl[axis] = d
                m = exposed[tuple(sl)].copy()  # indexed [u][v] in the remaining axis order
                U, V = m.shape
                for i in range(U):
                    j = 0
                    while j < V:
                        c = m[i, j]
                        if c < 0: j += 1; continue
                        w = 1
                        while j + w < V and m[i, j + w] == c: w += 1
                        h = 1
                        while i + h < U and np.all(m[i + h, j:j + w] == c): h += 1
                        m[i:i + h, j:j + w] = -1
                        out.append((int(c), axis, sgn, d, i, j, h, w))
                        j += w
    return out

rects = greedy_faces(grid)
vox_tris = []
for c, axis, sgn, d, i, j, h, w in rects:
    u_ax, v_ax = [a for a in range(3) if a != axis]
    base = np.zeros(3); base[axis] = d + (1 if sgn > 0 else 0); base[u_ax] = i; base[v_ax] = j
    du = np.zeros(3); du[u_ax] = h
    dv = np.zeros(3); dv[v_ax] = w
    p0 = lo + base * VOX; p1 = lo + (base + du) * VOX; p2 = lo + (base + du + dv) * VOX; p3 = lo + (base + dv) * VOX
    nrm = np.zeros(3); nrm[axis] = sgn
    q = [p0, p1, p2, p3]
    if np.dot(np.cross(p1 - p0, p2 - p0), nrm) < 0: q = [p0, p3, p2, p1]
    vox_tris.append((cols[c], q[0], q[1], q[2]))
    vox_tris.append((cols[c], q[0], q[2], q[3]))
print("voxel body tris", len(vox_tris))
body = []

RIM2 = (0.55, 0.55, 0.58)
def make_wheel(key, parts):
    """The tyre as a 12-sided drum the model's tyre size, and a flat rim of ten wedges in two greys on its
    outer face (so it can be seen turning). Pivot at the centre."""
    tyre = [o for o, col in parts if col == BLACK]
    rim = [o for o, col in parts if col != BLACK]
    vs = [v.co.copy() for o in tyre for v in o.data.vertices]
    rv = [v.co.copy() for o in rim for v in o.data.vertices]
    mn = Vector((min(v.x for v in vs), min(v.y for v in vs), min(v.z for v in vs)))
    mx = Vector((max(v.x for v in vs), max(v.y for v in vs), max(v.z for v in vs)))
    c = (mn + mx) / 2
    R = max(mx.y - mn.y, mx.z - mn.z) / 2
    W = mx.x - mn.x
    rr = min(max(max(abs(v.y - c.y), abs(v.z - c.z)) for v in rv), R * 0.62)
    side = 1.0 if c.x > 0 else -1.0  # outer face
    me = bpy.data.meshes.new(f"wheel_{key}")
    bm = bmesh.new()
    N = 12
    ring = lambda x, r: [bm.verts.new((x, r * math.cos(2 * math.pi * i / N), r * math.sin(2 * math.pi * i / N))) for i in range(N)]
    a0, a1 = ring(-W / 2, R * 0.86), ring(-W / 2 + W * 0.12, R)
    b1, b0 = ring(W / 2 - W * 0.12, R), ring(W / 2, R * 0.86)
    rings = [a0, a1, b1, b0]
    tf = []
    for k in range(3):
        for i in range(N):
            j = (i + 1) % N
            tf.append(bm.faces.new((rings[k][i], rings[k][j], rings[k + 1][j], rings[k + 1][i])))
    cin = bm.verts.new((-W / 2, 0, 0)); cout = bm.verts.new((W / 2, 0, 0))
    for i in range(N):
        j = (i + 1) % N
        tf.append(bm.faces.new((cin, a0[j], a0[i])))
        tf.append(bm.faces.new((cout, b0[i], b0[j])))
    bmesh.ops.recalc_face_normals(bm, faces=tf)
    x = side * (W / 2 + 0.008)
    hub = bm.verts.new((x, 0, 0))
    M2 = 10
    rp = [bm.verts.new((x, rr * math.cos(2 * math.pi * i / M2), rr * math.sin(2 * math.pi * i / M2))) for i in range(M2)]
    for i in range(M2):
        j = (i + 1) % M2
        f = bm.faces.new((hub, rp[i], rp[j]) if side > 0 else (hub, rp[j], rp[i]))
        f.material_index = 1 if i % 2 == 0 else 2
    bm.to_mesh(me); bm.free()
    for col in (BLACK, RIM, RIM2):
        m = bpy.data.materials.new(f"w{col}"); m.diffuse_color = (*col, 1); me.materials.append(m)
    w = bpy.data.objects.new(f"wheel_{key}", me); bpy.context.scene.collection.objects.link(w)
    w.location = c
    for o, _ in parts: bpy.data.objects.remove(o)
    return w

wheels = {key: make_wheel(key, parts) for key, parts in wheel_parts.items()}

def tris_of(o):
    out = []
    me = o.data
    for p in me.polygons:
        vs = [me.vertices[i].co for i in p.vertices]
        col = tuple(me.materials[p.material_index].diffuse_color[:3])
        for k in range(1, len(vs) - 1):
            out.append((col, vs[0], vs[k], vs[k + 1]))
    return out

def fmt(v): return f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f}"
allp = np.array([p for t in vox_tris for p in t[1:]])
mn, mx = allp.min(axis=0), allp.max(axis=0)
with open(OUT, "w") as f:
    f.write("# LandCruiser 79 pick-up (off-road), voxelised from \"Toyota Land Cruiser J79 Pickup Off- Road\" by DR1KING100K, CC BY 4.0 (docs/CREDITS.md). Made by tools/make_79.py\n")
    for key in ("FL", "FR", "RL", "RR"):
        f.write(f"wheel {key} {fmt(wheels[key].location)}\n")
    f.write(f"outline {max(-mn[0], mx[0]):.3f} {mx[2]:.3f} {mn[2]:.3f}\n")
    f.write("lamps own\n")
    r = wheels["FL"].dimensions
    f.write(f"wheelsize {max(r.y, r.z) / 2:.4f} {r.x - 0.016:.4f}\n")
    for (kind, col), a, b, c in vox_tris:
        f.write(f"{kind} {col[0]:.3f} {col[1]:.3f} {col[2]:.3f}  {fmt(a)}  {fmt(b)}  {fmt(c)}\n")
    wt = 0
    for key in ("FL", "FR", "RL", "RR"):
        for col, a, b, c in tris_of(wheels[key]):
            f.write(f"wtri {key} {col[0]:.3f} {col[1]:.3f} {col[2]:.3f}  {fmt(a)}  {fmt(b)}  {fmt(c)}\n")
            wt += 1
print(f"BODY_TRIS {len(vox_tris)} WHEEL_TRIS {wt} extent {mn} {mx}")
for k, w in wheels.items(): print("WHEEL", k, tuple(round(v, 3) for v in w.location), tuple(round(v, 3) for v in w.dimensions))

if RENDER_DIR:
    sc = bpy.context.scene
    for o in list(sc.objects):
        if not o.name.startswith("wheel_"): bpy.data.objects.remove(o)
    me = bpy.data.meshes.new("body")
    verts, faces, fcol, mats = [], [], [], {}
    for (kind, col), a, b, c in vox_tris:
        faces.append((len(verts), len(verts) + 1, len(verts) + 2)); verts += [tuple(a), tuple(b), tuple(c)]
        fcol.append(mats.setdefault(col, len(mats)))
    me.from_pydata(verts, [], faces)
    for col in mats:
        m = bpy.data.materials.new(str(col)); m.diffuse_color = (*col, 1); me.materials.append(m)
    me.polygons.foreach_set("material_index", fcol)
    bodyo = bpy.data.objects.new("body", me); sc.collection.objects.link(bodyo)
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.display.shading.light = 'STUDIO'; sc.display.shading.color_type = 'MATERIAL'
    sc.display.shading.show_object_outline = False
    sc.render.resolution_x, sc.render.resolution_y = 1200, 700
    cam = bpy.data.cameras.new("c"); cam.lens = 50; cam.clip_start = 0.05
    co = bpy.data.objects.new("c", cam); sc.collection.objects.link(co); sc.camera = co
    # show the game frame (x right, y up, z forward; mirrored) the right way up in Blender: z up, -y forward
    rig = bpy.data.objects.new("rig", None); sc.collection.objects.link(rig)
    for o in [bodyo] + list(wheels.values()): o.parent = rig
    rig.rotation_euler = (math.radians(90), 0, 0); rig.scale = (-1, 1, 1)
    tgt = Vector((0, 0, 0.5))
    co.location = tgt + Vector((-6.0, -7.0, 3.4))
    co.rotation_euler = (tgt - co.location).to_track_quat('-Z', 'Y').to_euler()
    sc.world = bpy.data.worlds.new("w"); sc.world.color = (0.55, 0.5, 0.42)
    sc.render.filepath = os.path.join(RENDER_DIR, "cut.png")
    bpy.ops.render.render(write_still=True)
