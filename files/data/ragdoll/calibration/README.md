# Captured-body calibration fixture

`context-symP.creation.txt` is the creation header and 20-bone skeleton from
the accepted native `context-symP` capture. `ragdollcore_walksearch` reads only
this prefix when `--native-body` is used; replay frames and the captured ground
shape are not part of body construction.

The qualification producer hashes this file and
`../humanoid-v1.yaml`, checks that their profile IDs match, and records both
hashes in every qualification artifact. Do not replace this fixture as part of
a controller change. A new captured body requires an explicit calibration
decision and invalidates evidence in the `calibration` scope.
