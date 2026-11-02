#!/bin/bash
# Film every behaviour case and rebuild the viewer's data: the balance
# stage's cases (stagger, landing, stairs, slopes, edges, catches, mantle,
# crawl down) and DROP-01, in play settings, with the side camera
# (OPENMW_RAGDOLL_TEST_FILM=90), then film.py and build_data.py.
# usage: tools/viewer/refresh.sh [NAME] [LABEL]   (~30-40 min of game time)
# Never while the user plays (waits for openmw-play.exe to close).
set -e
NAME=${1:-show$(date +%m%d%H%M)}
LABEL=${2:-"Showcase $(date +%Y-%m-%d) ($(git -C "$(dirname "$0")/../.." rev-parse --short HEAD))"}
cd "$(dirname "$0")/../.."
while tasklist | grep -qi openmw-play; do echo "waiting: the game is open $(date +%H:%M)"; sleep 60; done
mapfile -t CASES < <(python -c "
import sys; sys.path.insert(0, 'tools/body')
import sentinel
for p in sentinel.STAGES['balance'][1] + ['DROP%-01 run off %d', 'DROP%-01 walk off %d', 'DROP%-01 strafe off %d']:
    print(p)" | tr -d '\r')  # Windows python prints CRLF
python tools/body/cases.py "$NAME" test_play01_walking "${CASES[@]}" OPENMW_RAGDOLL_TEST_FILM=90 --play
python tools/body/film.py "../runtime-ONE-$NAME" --label "$LABEL"
python tools/viewer/build_data.py "../runtime-ONE-$NAME" --label "../runtime-ONE-$NAME=$LABEL"
echo "viewer data refreshed: $LABEL"
