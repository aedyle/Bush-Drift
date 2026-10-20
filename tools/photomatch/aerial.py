"""Top-down picture of the game round the hut beside the NSW 2011 aerial and the Google close-up, same scale, north up.
Usage: python tools/photomatch/aerial.py CE CN ALT OUT.png   (centre metres E, N of the hut; camera height in m)"""
import math, os, subprocess, sys
sys.path.insert(0, "scripts")
from imagery import Mosaic
from PIL import Image, ImageDraw, ImageFont
CE, CN, ALT = map(float, sys.argv[1:4]); OUT = sys.argv[4]; FOV = 30
raw = OUT.replace(".png", "-game.png")
env = dict(os.environ, SHOT_CAMABS="1", SHOT_FOV=str(FOV), SHOT_CAM=f"{CE},{CN - 0.05},{ALT},{CE},{CN},0")
subprocess.run(["./bush-drift", "--idle", "--hide", "--shot", raw, "90"], env=env, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
H = 2 * ALT * math.tan(math.radians(FOV / 2)); W = H * 640 / 480; mpp = H / 480
f = ImageFont.truetype("/usr/share/fonts/noto/NotoSans-Regular.ttf", 18)
m = Mosaic(); lat0, lon0 = -36.7208104, 149.4627460
c, r = m.pix(lat0, lon0); gpm = 1 / (m.res * math.cos(math.radians(lat0)))
cx, cy = c + CE * gpm, r - CN * gpm
nsw = m.img.crop((int(cx - W / 2 * gpm), int(cy - H / 2 * gpm), int(cx + W / 2 * gpm), int(cy + H / 2 * gpm))).resize((640, 480), Image.BICUBIC)
G = Image.open("refs/hut/s3b/1-google-closeup.webp").convert("RGB"); S = 15.15
def gpx(E, N): E -= 1.8; N += 1.0; return (715 + (E - 5.8) * S, 214 - (N + 2.6) * S)   # pin georef, roof shifted onto OSM
x0, y0 = gpx(CE - W / 2, CN + H / 2); x1, y1 = gpx(CE + W / 2, CN - H / 2)
cv = Image.new("RGB", (int(x1 - x0), int(y1 - y0)), (30, 30, 30)); cv.paste(G, (int(-x0), int(-y0)))
goo = cv.resize((640, 480), Image.BICUBIC)
game = Image.open(raw).convert("RGB")
out = Image.new("RGB", (640 * 3 + 20, 520), (20, 20, 20)); d = ImageDraw.Draw(out)
for k, (im, t) in enumerate(((nsw, "NSW aerial 2011"), (goo, "Google (today)"), (game, "Game"))):
    out.paste(im, (k * 650, 0)); d.text((k * 650 + 8, 486), t, fill=(240, 240, 240), font=f)
    d.line((k * 650 + 20, 460, k * 650 + 20 + 10 / mpp, 460), fill=(255, 255, 255), width=3); d.text((k * 650 + 22, 432), "10 m, north up", fill=(255, 255, 255), font=f)
out.save(OUT); print(OUT, f"{W:.0f} x {H:.0f} m")
