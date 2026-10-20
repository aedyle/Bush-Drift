#!/usr/bin/env python3
"""A long --bench run's per-frame log, per kilometre: frame times, late frames, where the time went,
the planner thread's work and memory. usage: longrun.py FILE.csv [km per row, default 1]"""
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
step = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
f = lambda r, k: float(r[k])
bins = {}
for r in rows:
    bins.setdefault(int(f(r, 'dist') / 1000 / step), []).append(r)
print(' km     frames  mean ms  worst  late(>17.7)  update  draw  text  swap  planner ms/s  chunks  memory MB')
for b in sorted(bins):
    g = bins[b]; n = len(g); secs = sum(f(r, 'dt') for r in g) / 1000
    m = lambda k: sum(f(r, k) for r in g) / n
    print('%4.1f-%-4.1f %6d  %6.2f  %5.1f  %6d      %5.2f  %4.2f  %4.2f  %5.2f   %8.1f    %4d    %6.1f' % (b * step, (b + 1) * step, n, m('dt'), max(f(r, 'dt') for r in g),
          sum(f(r, 'dt') > 17.7 for r in g), m('update'), m('draw'), m('text'), m('swap'), sum(f(r, 'plan_ms') for r in g) / max(secs, 1e-6),
          sum(int(r['chunks']) for r in g), max(f(r, 'rss_kb') for r in g) / 1024))
