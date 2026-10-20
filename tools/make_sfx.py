"""Cut the game's sound loops (assets/sfx/) from CC0 Freesound recordings (docs/CREDITS.md): python tools/make_sfx.py RAW_DIR assets/sfx
RAW_DIR holds the Freesound previews named <id>.mp3. Each loop is trimmed, levelled and made seamless by
crossfading its end into its start; one-shots get short fades. Writes mono 22,050 Hz Ogg Vorbis."""
import os, subprocess, sys
import numpy as np

SR = 22050
# name: (freesound id, start s, length s, crossfade s or None for a one-shot, target RMS dBFS)
CUTS = {
    "engine_idle":  (611760, 70.0, 6.0, 0.4, -16),   # Ford F250 V8 diesel idling, ~700 rpm (firing ~46.5 Hz)
    "engine_load":  (668552, 458.0, 6.0, 0.5, -16),  # Toyota Landcruiser 4x4 driving a dirt road, mic outside the window
    "gravel_roll":  (577504, 36.0, 8.0, 0.5, -18),   # tyres rolling slowly on a gravel road, mic close to the wheels
    "gravel_slide": (725428, 7.2, 5.2, 0.4, -16),    # a truck doing donuts on gravel
    "bush":         (812609, 60.0, 60.0, 3.0, -26),  # birds and insects, Mebbin National Park, NSW
    "kookaburra":   (427327, 11.5, 10.5, None, -20), # kookaburras, Cathedral Ranges, Victoria
}

def load(path, start, length):
    raw = subprocess.run(["ffmpeg", "-v", "error", "-ss", str(start), "-t", str(length), "-i", path,
                          "-ac", "1", "-ar", str(SR), "-f", "f32le", "-"], capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.float32).astype(np.float64)

def main():
    raw_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    for name, (fid, start, length, xf, rms_db) in CUTS.items():
        x = load(os.path.join(raw_dir, f"{fid}.mp3"), start, length + (xf or 0))
        x -= x.mean()
        if xf:   # seamless: the extra tail fades out over the start, which fades in
            n = int(xf * SR); body = x[:-n].copy(); tail = x[-n:]
            ramp = np.linspace(0, 1, n)
            body[:n] = body[:n] * np.sqrt(ramp) + tail * np.sqrt(1 - ramp)
            x = body
        else:
            n = int(0.05 * SR); x[:n] *= np.linspace(0, 1, n); m = int(0.8 * SR); x[-m:] *= np.linspace(1, 0, m)
        x *= 10 ** (rms_db / 20) / max(np.sqrt(np.mean(x ** 2)), 1e-9)
        x = np.clip(x, -0.98, 0.98).astype(np.float32)
        out = os.path.join(out_dir, name + ".ogg")
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-f", "f32le", "-ar", str(SR), "-ac", "1", "-i", "-",
                        "-c:a", "libvorbis", "-q:a", "3", out], input=x.tobytes(), check=True)
        print(f"{name:13s} {len(x) / SR:5.1f} s  {os.path.getsize(out) / 1024:6.0f} KB")

main()
