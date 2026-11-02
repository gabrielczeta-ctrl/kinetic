# WPN-05 scene attachment lifecycle regression

Seven separate native cases cover same-event camera/equipment/draw changes in both
view directions, ordinary draw and ready camera rebuilding, sheath/redraw and
empty hands, ready and unfinished-draw replacement of a one-handed item, and the actual hybrid helper path
followed by a charged native attack. They do not change the existing 97-case
two-handed suite or the separate 15 motion-capture cases.

The read-only dev API `ragdoll.getWeaponVisibility(actor)` observes the live
NpcAnimation scene, independently of the physical weapon proxy. Visible checks
require `weaponsShown`, `weaponNodePresent` and exactly one parental path to the
current animation root. A shown node does not prove correct pixels or item
identity, so inspect captured native frames as well. Support separation and
physical combat qualification remain in their existing suites.

Use the existing private-desktop runner with a fresh output directory, the exact
engine source commit, 60 Hz physics and a bounded exact case filter. Example:

```powershell
python tools/play04/run_native.py --engine ../build-play04/RelWithDebInfo/openmw.exe --engine-source <commit> --output ../runtime-WPN-05-visibility-first-third-60 --render-fps 60 --task-id WPN-05 --suite test_weapon_visibility --test-filter '^WPN%-05 visibility same%-frame first_person to third_person$' --env OPENMW_RAGDOLL_TWO_HAND_TRACE=1
```

Repeat at 30/60/120 render FPS. Preserve logs/manifests and capture files. The
baseline trace that motivated this regression reports Equipping (1) becoming
WeaponEquipped (3) with shown=0/node=0 through native wind-up and release. The
repair should attach the still-drawn item during that forced transition. The
fixture keeps camera/equipment/stance requests in one event and does not add a
pre-draw delay to avoid the race. No tracker acceptance is emitted by this suite.

The equipped-replacement case waits for the original `weapononehand` animation
state to disappear (`openmw.animation.getCompletion` returns nil) before replacing
the ready item. Mesh attachment occurs before equip completion and is insufficient
to identify that ready state. Camera requests are retried without force only when
the desired mode is neither current nor queued. Pending configuration telemetry
records camera/queue, equipped item, stance and both weapon-group completion values.
The earlier mid-equip replacement timeout in `runtime-WPN-05-visibility-60-01`
remains unresolved evidence, now retained as its own exact-timing
`WPN-05 visibility replacement during unfinished draw` regression. This case
keeps the original no-wait ordering between visible attachment and the combined
camera/type replacement request. Passing the ready-item replacement case does
not establish that interrupted weapon-type replacement works.
