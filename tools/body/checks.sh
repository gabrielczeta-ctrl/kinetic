#!/bin/sh
# The keep/revert gates for a body change, in play settings (ragdoll.cfg +
# active-body.cfg; a variant's ENV=V wins):
#   walking suite (^PLAY%-01): only the knockdown cases fall
#   shake_report: player idle p50 about 0 deg/s
#   duels (test_duel): all pass
#   FIX-08 (test_combat_fixes): 33 cases; 25-26 pass (2026-09-29: drags
#     count until withdrawal, mid-gripped spear/staff targets placed by the
#     real tip). Chronic misses: dagger first person (3), spear/staff with
#     the crosshair centered, held drag spear; fists and fast drag warhammer
#     flaky.
# plus the gait report (touchdown lead, crossing, slide, stops).
# usage: checks.sh NAME ["ENV=V ENV=V"] [walk|duel|fix08 ...]   (default: all)
set -u
root=$(cd "$(dirname "$0")/../../.." && pwd)
# TREE / BUILD: another checkout and build directory (default source-master, build).
tree=$root/${TREE:-source-master}
build=$root/${BUILD:-build}
name=$1; envs=${2:-}; shift; [ $# -gt 0 ] && shift
parts=${*:-walk duel fix08}
args=""
for e in $envs; do args="$args --env $e"; done
# Footfall is on in play (Run-Game-MotionBricks.cmd sets OPENMW_RAGDOLL_FOOTFALL=1)
# but not in the settings file: without it the gates tested the old legs
# (found 2026-09-29). A variant's ENV=V still wins.
play="--env OPENMW_RAGDOLL_SETTINGS=$tree/files/data/ragdoll/ragdoll.cfg --env OPENMW_RAGDOLL_ACTIVE_BODY=$tree/files/data/ragdoll/active-body.cfg --env OPENMW_RAGDOLL_FOOTFALL=1"
source=$(git -C "$tree" rev-parse --short HEAD)-dirty
run() {  # out suite filter extra...
    out=$1; suite=$2; filter=$3; shift 3
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$root/tools-workspace/Assert-DiskSpace.ps1" > /dev/null || { echo "disk guard failed"; exit 1; }
    rm -rf "$out" "$out.runner.log"
    python "$tree/tools/play04/run_watched.py" --engine "$build/RelWithDebInfo/openmw.exe" \
        --engine-source "$source" --output "$out" --suite "$suite" --test-filter "$filter" \
        $play $args "$@" --parallel 4 --suite-timeout 1500 > /dev/null 2>&1
}
passed() {  # result-summary.json -> "passed/total"
    python -c "
import json,sys
s=json.load(open(sys.argv[1]))
c=[x for su in s['suites'] for x in su['cases']]
print('%d/%d' % (sum(x['status']=='passed' for x in c), len(c)))
for x in c:
    if x['status']!='passed': print('   fail:', x['name'])" "$1/result-summary.json" 2>/dev/null || echo "no result"
}
for part in $parts; do
    case $part in
    walk)
        out=$root/runtime-SOFT-$name
        run "$out" test_play01_walking '^PLAY%-01' --env OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER=4 --diagnostic-budget-mib 1600
        echo "== $name walking $(passed "$out" | head -1)"
        python "$tree/tools/body/walk_report.py" "$out" | grep -A1 -E "^PLAY" | grep -B1 "FALL@" | grep "^PLAY" | sed 's/^/   fell: /'
        python "$tree/tools/body/shake_report.py" "$out" | grep -E "player (idle|stop) " | sed 's/^ */   shake /'
        python "$tree/tools/body/gait_report.py" "$out" | tail -n +2 | sed 's/^/   /'
        ;;
    duel)
        out=$root/runtime-SOFT-$name-duel
        run "$out" test_duel '.'
        echo "== $name duels $(passed "$out")"
        ;;
    fix08)
        out=$root/runtime-SOFT-$name-fix08
        run "$out" test_combat_fixes '^FIX%-08'
        echo "== $name FIX-08 $(passed "$out" | head -1)"
        ;;
    esac
done
