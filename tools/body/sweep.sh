#!/bin/sh
# Run the native walking suite once per variant (sequential: the engine lease
# allows one native run at a time) and print the comparison. Every run uses
# the play settings (ragdoll.cfg, active-body.cfg); a variant's ENV=V wins.
# usage: sweep.sh FILTER name1 "ENV=V ENV=V" name2 "ENV=V" ...
# Example: sweep.sh '^PLAY%-01 unarmed [jmrs]' base "" soft "OPENMW_RAGDOLL_SOFT_ROOT=1"
set -u
root=$(cd "$(dirname "$0")/../../.." && pwd)
# TREE / BUILD: another checkout and build directory (default source-master, build).
tree=$root/${TREE:-source-master}
build=$root/${BUILD:-build}
filter=$1; shift
runs=""
while [ $# -ge 2 ]; do
    name=$1; envs=$2; shift 2
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$root/tools-workspace/Assert-DiskSpace.ps1" > /dev/null || { echo "$name: disk guard failed"; exit 1; }
    out=$root/runtime-SOFT-$name
    rm -rf "$out" "$out.runner.log"
    args=""
    for e in $envs; do args="$args --env $e"; done
    python "$tree/tools/play04/run_watched.py" --engine "$build/RelWithDebInfo/openmw.exe" \
        --engine-source "$(git -C "$tree" rev-parse --short HEAD)-dirty" --output "$out" \
        --suite test_play01_walking --test-filter "$filter" \
        --env OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER=4         --env OPENMW_RAGDOLL_SETTINGS=$tree/files/data/ragdoll/ragdoll.cfg         --env OPENMW_RAGDOLL_ACTIVE_BODY=$tree/files/data/ragdoll/active-body.cfg $args \
        --parallel 4 --diagnostic-budget-mib 800 --suite-timeout 900 > /dev/null 2>&1
    echo "$name: exit $? ($envs)"
    runs="$runs $out"
done
python "$tree/tools/body/walk_report.py" $runs
