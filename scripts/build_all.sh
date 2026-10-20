#!/bin/sh
# Bush Drift map-data pipeline, in order. Raw downloads are cached in data/raw/ and reused.
set -e
cd "$(dirname "$0")/.."
[ -x .venv/bin/python ] || { python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt; }
for s in fetch_route fetch_dem fetch_imagery refine_centreline build_route route_stats fetch_far; do
    echo "== $s"; .venv/bin/python scripts/$s.py
done
