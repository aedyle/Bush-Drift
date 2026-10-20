#!/usr/bin/env python3
"""Turn the working tuning file into a C header, so the game carries its values built in.
usage: gen_defaults.py tuning-bush.txt [more.txt ...] build/tuning_defaults.h
The header holds the files' key = value lines (comments dropped) as one string, which the game
reads with the same code that reads a tuning file. It is generated, so it stays out of git (build/)."""
import sys

srcs, dst = sys.argv[1:-1], sys.argv[-1]
src = " + ".join(srcs)
lines = []
for raw in (l for p in srcs for l in open(p, encoding="utf-8")):
    line = raw.split("#", 1)[0].strip()
    if "=" not in line:
        continue
    key, val = (part.strip() for part in line.split("=", 1))
    lines.append("%s = %s" % (key, val))
with open(dst, "w", encoding="utf-8") as out:
    out.write("/* Made from %s by tools/gen_defaults.py. Do not edit; do not commit. */\n" % src)
    out.write("static const char tuning_default_text[] =\n")
    for line in lines:
        out.write('    "%s\\n"\n' % line.replace("\\", "\\\\").replace('"', '\\"'))
    out.write("    ;\n")
print("%s: %d values" % (dst, len(lines)))
