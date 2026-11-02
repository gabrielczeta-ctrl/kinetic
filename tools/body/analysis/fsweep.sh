# usage: fsweep.sh "name|ENV ENV" ...   fists course run only
S=$(cd "$(dirname "$0")" && pwd)
cd "$S/../../../.."
names=""
for v in "$@"; do
  n=${v%%|*}; e=${v#*|}
  TREE=source-footfall BUILD=build-footfall sh source-footfall/tools/body/look.sh "$n" "OPENMW_RAGDOLL_FOOTFALL=1 $e" "${FILTER:-^PLAY%-01 ${LO:-fists} course run$}" >/dev/null 2>&1
  python source-footfall/tools/body/course_report.py runtime-LOOK-$n > runtime-LOOK-$n/course.txt 2>&1
  names="$names $n"
done
python $S/tgt.py $names | grep -v "^   "
python $S/cmp.py $names 2>/dev/null | sed 's/walk.*||/||/'
