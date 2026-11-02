#!/bin/sh
# Visual review: run the LOOK walking cases (side camera, 15 stills/s) and tile
# the stills into one contact sheet per case.
# usage: look.sh NAME ["ENV=V ENV=V"] [lua filter (default: all look cases)]
# Sheets: runtime-LOOK-NAME/sheets/*.png
set -u
root=$(cd "$(dirname "$0")/../../.." && pwd)
# TREE / BUILD: another checkout and build directory (default source-master, build).
tree=$root/${TREE:-source-master}
build=$root/${BUILD:-build}
name=$1; envs=${2:-}; filter=${3:-'^PLAY%-01 [uw][na].* look '}
args=""
for e in $envs; do args="$args --env $e"; done
out=$root/runtime-LOOK-$name
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$root/tools-workspace/Assert-DiskSpace.ps1" > /dev/null || { echo "disk guard failed"; exit 1; }
rm -rf "$out" "$out.runner.log"
python "$tree/tools/play04/run_watched.py" --engine "$build/RelWithDebInfo/openmw.exe" \
    --engine-source "$(git -C "$tree" rev-parse --short HEAD)-dirty" --output "$out" \
    --suite test_play01_walking --test-filter "$filter" \
    --env OPENMW_RAGDOLL_SETTINGS=$tree/files/data/ragdoll/ragdoll.cfg \
    --env OPENMW_RAGDOLL_ACTIVE_BODY=$tree/files/data/ragdoll/active-body.cfg \
    $args --parallel 2 --diagnostic-budget-mib 1500 --suite-timeout 1500 > "$out.runner.log" 2>&1
python "$tree/tools/body/look_sheet.py" "$out"
