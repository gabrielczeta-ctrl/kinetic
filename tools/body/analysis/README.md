# Course analysis scripts (2026-09-28)

Run from the dev root on `runtime-LOOK-<name>` outputs of `tools/body/course.sh` / `look.sh`.

- `cmp.py NAME...` - medians of course.json per kind (walk/run): stepT, stepL, side, gallop, pushoff, both, scuff, sink, lhand.
- `tgt.py NAME...` - running leg amplitude (knee/ankle forward, back, heel up in leg lengths), physical vs target, per loadout (reference run 0.45/0.40/0.75/0.66).
- `torso.py "c walk,c run" NAME...` - chest/pelvis pitch and roll range and rate against the animation.
- `chestoff.py NAME...` - chest pitch minus the animation's, mean +- sd per loadout.
- `upper.py "c walk,c run,c stop" NAME...` - head and hand error against the animation, head pitch range/rate.
- `pitch.py "c run" NAME...` - foot pitch (toes down +) physical vs target.
- `getup.py runtime-LOOK-NAME` - get-up time, hand flail, torso spin, falls after (look knockdown cases).
- `footpitch.py` - reference foot pitch curves from reference-loops.txt (tools-workspace/mixamo/foot_pitch.py is the committed generator).
- `fsweep.sh "name|ENV ENV" ...` - single-case course runs (~1 min each); FILTER=lua pattern or LO=loadout for the course run.
Metric caveats: running stepT/stepL are +-10 noise per case; the animation targets update on some frames only (use 4-frame differences for rates).
