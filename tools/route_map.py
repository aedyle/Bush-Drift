"""The route from above (build B): docs/route/birdseye.png, for the play-test page.

    python3 tools/route_map.py [OUT.png]

Renders the game's own ground straight down from high up (the far land and the hut's clearing; no imagery), then draws
the road from data/built/route.csv over it with the hut, Solomons Creek ford, the bridge, a kilometre scale and the
length. Needs the game built (./bush-drift) and a display; Pillow and a Noto font.
"""
import csv, math, os, subprocess, sys, tempfile
from PIL import Image, ImageDraw, ImageFont

OUT = sys.argv[1] if len(sys.argv) > 1 else "docs/route/birdseye.png"
rows = list(csv.DictReader(open("data/built/route.csv")))
meta = dict(l.split(None, 1) for l in open("data/built/meta.txt") if l.strip())
HX, HY = float(meta["hut_x"]), float(meta["hut_y"])
FORD = float(meta["ford_s"])
xs, ys = [float(r["x"]) for r in rows], [float(r["y"]) for r in rows]
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
CE, CN = (x0 + x1) / 2 - HX, (y0 + y1) / 2 - HY            # camera over the middle of the route, metres from the hut
FOV = 30.0
f = 240 / math.tan(math.radians(FOV / 2))
ALT = max((x1 - x0) * 1.12 / (640 / f), (y1 - y0) * 1.2 / (480 / f))   # high enough to fit the route with a margin
e0 = float(rows[0]["elev"])

raw = os.path.join(tempfile.mkdtemp(), "raw.png")
env = dict(os.environ, SHOT_NOCAR="1", SHOT_NOHUD="1", SHOT_CAMABS="1", SHOT_FOV=str(FOV),
           SHOT_CAM=f"{CE:.1f},{CN:.1f},{ALT:.1f},{CE:.1f},{CN + 1:.1f},0")
subprocess.run(["./bush-drift", "--pixel", "0", "--set", "haze_max=0", "--set", "clouds=0", "--shot", raw, "120", "--idle", "--mute", "--hide"],
               env=env, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

K = 2
def px(x, y, elev):
    d = ALT - (elev - e0)
    return (320 + f * (x - HX - CE) / d) * K, (240 - f * (y - HY - CN) / d) * K

im = Image.open(raw).convert("RGB").resize((640 * K, 480 * K), Image.NEAREST)
d = ImageDraw.Draw(im)
font = lambda s: ImageFont.truetype("/usr/share/fonts/noto/NotoSans-Bold.ttf", s)
def label(x, y, t, size=22):
    d.text((x, y), t, font=font(size), fill=(255, 255, 255), stroke_width=3, stroke_fill=(0, 0, 0))
pts = [px(float(r["x"]), float(r["y"]), float(r["elev"])) for r in rows]
d.line(pts, fill=(20, 15, 10), width=8)
d.line(pts, fill=(240, 196, 110), width=4)
for r in rows:   # kilometre marks
    s = float(r["s"])
    if s > 0 and abs(s / 1000 - round(s / 1000)) < 0.001:
        x, y = px(float(r["x"]), float(r["y"]), float(r["elev"]))
        d.ellipse((x - 6, y - 6, x + 6, y + 6), fill=(255, 255, 255), outline=(0, 0, 0))
        label(x + 9, y - 24, f"{round(s / 1000)} km", 18)
def pin(r, text, col, dx, dy):
    x, y = px(float(r["x"]), float(r["y"]), float(r["elev"]))
    d.ellipse((x - 11, y - 11, x + 11, y + 11), fill=col, outline=(0, 0, 0), width=2)
    label(x + dx, y + dy, text, 24)
ford = min(rows, key=lambda r: abs(float(r["s"]) - FORD))
pin(rows[0], "Alexanders Hut (start)", (90, 200, 90), 16, -16)
pin(ford, "Solomons Creek ford", (90, 170, 240), -120, -58)
pin(rows[-1], "Tantawangalo Creek bridge (finish)", (220, 70, 50), -400, 16)
length = float(rows[-1]["s"]) / 1000
mpp = (2 * ALT * math.tan(math.radians(FOV / 2)) / 480) / K
bx, by = 30, im.height - 46
d.rectangle((bx - 12, by - 44, bx + 1000 / mpp + 24, by + 22), fill=(0, 0, 0))
d.line((bx, by, bx + 1000 / mpp, by), fill=(255, 255, 255), width=5)
d.text((bx, by - 38), "1 km", font=font(18), fill=(255, 255, 255))
nx, ny = im.width - 50, 30
d.polygon([(nx, ny), (nx - 14, ny + 40), (nx + 14, ny + 40)], fill=(255, 255, 255), outline=(0, 0, 0))
label(nx - 9, ny + 44, "N", 24)
label(24, 18, f"Bush Drift: Alexanders Hut to the bridge, {length:.1f} km by road", 26)
label(24, 54, "the game's own ground from above; the road from the route data (OpenStreetMap, NSW Spatial Services)", 16)
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
im.save(OUT, optimize=True)
print(OUT, im.size, f"camera {ALT:.0f} m up")
