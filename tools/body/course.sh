#!/bin/sh
# Test course: every loadout (fists raised, dagger, longsword, claymore,
# warhammer) through the scripted course, scored against the animation.
# usage: course.sh NAME ["ENV=V ENV=V"] [run|long|balmora]   (long = three laps with shoves, fists; balmora = terrain)
# Report: runtime-LOOK-NAME/course.txt
root=$(cd "$(dirname "$0")/../../.." && pwd)
name=$1; envs=${2:-}
filter='^PLAY%-01 [fdlcw][iaol][sgnar][^ ]* course$'
[ "${3:-}" = run ] && filter='^PLAY%-01 [fdlcw][iaol][sgnar][^ ]* course run$'
[ "${3:-}" = long ] && filter='^PLAY%-01 fists course long$'
[ "${3:-}" = balmora ] && filter='^PLAY%-01 [fc][il][sa][^ ]* course balmora %a+$'
TREE=${TREE:-source-footfall} BUILD=${BUILD:-build-footfall} sh "$root/source-footfall/tools/body/look.sh" "$name" "OPENMW_RAGDOLL_FOOTFALL=1 $envs" "$filter" > /dev/null 2>&1
python "$root/source-footfall/tools/body/course_report.py" "$root/runtime-LOOK-$name" | tee "$root/runtime-LOOK-$name/course.txt"
