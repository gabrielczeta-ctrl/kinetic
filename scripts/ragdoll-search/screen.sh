#!/bin/bash
# Fixed screening suite, identical across layers (copied verbatim from
# kimi tasks/2026-09-08/13-37-53-eafd8b88/screen.sh; only BIN/OUT are
# overridable). NOTE: the case names are the historical ones: impulse
# x,0,0 is LATERAL to the +Y travel direction and 0,y,0 is FORWARD, so
# "fwd" below pushes sideways and "lat" pushes forward.
# usage: screen.sh <tag> [extra walksearch args...]
TAG=$1; shift
BIN=${BIN:-$(cd "$(dirname "$0")/../../.." && pwd)/build-opus-search/RelWithDebInfo/ragdollcore-walksearch.exe}
OUT=${OUT:-$(cd "$(dirname "$0")/../../.." && pwd)/runtime-opus-search/screen}
mkdir -p "$OUT"
run() {
  local name=$1; shift
  "$BIN" "$@" > "$OUT/${TAG}-${name}.json" 2>&1
  echo "== ${TAG}-${name} exit=$?"
  cat "$OUT/${TAG}-${name}.json"
  echo
}
run contract --seconds 7 --speed 60 --start 30 "$@"
run speed30 --seconds 7 --speed 30 --start 30 "$@"
run speed45 --seconds 7 --speed 45 --start 30 "$@"
run speed75 --seconds 7 --speed 75 --start 30 "$@"
run push120-fwd1000  --seconds 7 --speed 60 --start 30 --impulse 120,0,1000,0,0 "$@"
run push120-lat1000  --seconds 7 --speed 60 --start 30 --impulse 120,0,0,1000,0 "$@"
run push200-fwd1500  --seconds 7 --speed 60 --start 30 --impulse 200,0,1500,0,0 "$@"
run push200-lat1500  --seconds 7 --speed 60 --start 30 --impulse 200,0,0,1500,0 "$@"
