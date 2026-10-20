"""Render the game from a solved photo camera and put the photo and the render side by side.

Usage: python tools/photomatch/match.py NAME PHOTO OUT.png "caption"   (reads tools/photomatch/NAME.cam from solve_cam.py)
The game draws 640x480; the vertical field of view is widened so the photo's horizontal view fits, then the
render is cropped to the photo's shape. Heights are from the ground at the hut (SHOT_CAMABS).
"""
import json, math, os, subprocess, sys
from PIL import Image, ImageDraw, ImageFont

name, photo, out, caption = sys.argv[1:5]
C = json.load(open(f"tools/photomatch/{name}.cam"))
pw, ph = C["size"]; asp = pw / ph
hf = 2 * math.atan(math.tan(math.radians(C["vfov"]) / 2) * asp)            # the photo's horizontal fov
if asp >= 640 / 480: vf = 2 * math.degrees(math.atan(math.tan(hf / 2) * 480 / 640)); cw, ch = 640, round(640 / asp)
else: vf = C["vfov"]; cw, ch = round(480 * asp), 480
e, n, u = C["cam"]; te, tn, tu = C["target"]
raw = out.replace(".png", "-raw.png")
env = dict(os.environ, SHOT_CAM=f"{e},{n},{u},{te},{tn},{tu}", SHOT_CAMABS="1", SHOT_NOCAR="1", SHOT_FOV=f"{vf:.3f}")
subprocess.run(["./bush-drift", "--idle", "--hide", "--shot", raw, "75"] + sys.argv[5:], env=env, check=True,
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
r = Image.open(raw).convert("RGB"); x0, y0 = (640 - cw) // 2, (480 - ch) // 2
r = r.crop((x0, y0, x0 + cw, y0 + ch))
H = 540; a = Image.open(photo).convert("RGB"); a = a.resize((round(H * asp), H), Image.LANCZOS); r = r.resize(a.size, Image.NEAREST)
sheet = Image.new("RGB", (a.width * 2 + 12, H + 44), (24, 24, 24)); sheet.paste(a, (0, 0)); sheet.paste(r, (a.width + 12, 0))
try: font = ImageFont.truetype("/usr/share/fonts/noto/NotoSans-Regular.ttf", 20)
except OSError: font = ImageFont.load_default()
ImageDraw.Draw(sheet).text((10, H + 10), caption, fill=(235, 235, 235), font=font)
sheet.save(out); print(out)
