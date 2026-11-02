# PLAY-01 native ordinary walking diagnostics

This fixture records actual player Controls input on the flat-floor native
engine. It does not implement or register a gameplay acceptance gate. A test
case completing means its diagnostic capture completed; read the measured
failure and termination records before interpreting it.

The exact PLAY-WALK-v2 cases are `forward cycle 45 seed 1` through `seed 4`:
2 seconds idle, 1 second start, 6 seconds steady walk, 1 second stop, and 2
seconds hold. They use the
ordinary captured-body walk request whose registered fixture speed is 45 world
units/s. The fixture applies the fixed captured-body ratio `45 / 128.17999267578125`
through ordinary Controls; it does not bypass the movement adapter. The five
segment boundaries are the preregistered transition markers.

The exact PLAY-WALK-v6 idle/start preflight cases are
`forward activation 45 seed 1` through `seed 4`: the same two-second idle and
one-second captured-body 45-unit start prefix, ending before steady walking.
They exist only to enforce the preregistered activation stopping rule before a
complete 720-tick protocol is spent on a candidate. They use fresh worlds,
ordinary Controls, source identity, and the same diagnostic transport as the
full cases; they are not duplicate full-trial coverage.

The other ten general independently selectable cases are `no input`, `forward`, `backward`,
`strafe left`, `strafe right`, `diagonal left`, `diagonal right`, `start stop`,
`moving turn`, and `reversal`. Prefix each with `PLAY-01 unarmed ` for the
powered player or `PLAY-01 vanilla ` for the original animation controller.
Each starts a fresh world. Vanilla supplies an observational comparison for
timing, travel and appearance, not a success target for turning or balance.

The twelve `heading screen ...` and `heading control ...` cases are the frozen
PLAY-01 Stage A common screen:
idle at 0/90 degrees, left/right quarter turns, exact half-turn tie, wrap
crossing, straight travel, turning during strafe/backpedal, mid-turn physical
impact/reacquisition, yaw-off authority and damping-off settling. The final two
must be run with the profile ablation named in their anchor; their test names do
not mutate the profile. The impact case submits an ordinary combat impulse to
the dynamic pelvis and records the resulting physical release.

Except for the identified 45-unit protocol above, digital movement uses full
values of -1, 0 or 1, including both axes at 1 for diagonals. Normal engine
controls own speed/normalization. Run and jump are false. The moving-turn case holds forward while independently requesting a
90-degree facing change over two seconds through `self.controls.yawChange`.
The fixture never substitutes heading for travel or overwrites camera yaw
after initial third-person placement. No manual-room or autostart script is
loaded, and both carried slots are cleared before activation.

Use the task helper's live mutex and a committed, matching task binary. Root
launcher selection is `-SmokeSuite test_play01_walking`. The first bounded
capture uses `-SmokeTestFilter '^PLAY%-01 unarmed forward$'`; all powered cases
use `'^PLAY%-01 unarmed '`. The underlying integration runner also accepts
`--render_fps 30`, `60`, or `120` and `--video_directory PATH`. Keep physics at
60 Hz and the test-step multiplier at 1. Capturing video changes render load;
the native manifest records that fact.

The suite emits the existing chunked diagnostic transport consumed into
`*.player-diagnostics.jsonl.gz`. Frames contain Controls acknowledgement/time,
segment request, camera and gameplay headings, player position, raw/filtered/
planned movement requests, actual COM velocity, pelvis/torso upright measure,
support/load/contact observations, feet poses/velocities, actuation state and
active route. Sampling follows observed physics serials: missed serials are
counted and never interpolated. Vanilla frames have no powered COM, foot or
contact telemetry; those unavailable measurements remain absent.

Powered captures include startup, a no-input interval, and a full native
fixed-step replay with body/pose metadata via the existing replay output and
shape directory environment variables. First falling/fallen/getting-up/
blend-back observation or rig loss stops commanded movement, retains one
second of subsequent context, and ends the case. Remaining segments are
unexecuted rather than measured through a recovery confound. Every requested
and acknowledged input remains recorded, including frame delay between them.

Aggregate one or more identified integration run directories with:

```text
python tools/tracker/play01_walking.py RUN_DIR --output NEW_OUTPUT_DIR --tracker-task PLAY-01 --build-identity BUILD_IDENTITY_JSON
```

The optional build identity must contain `source_commit` and `openmw_sha256`
(or `engine_sha256`); absent identity is reported explicitly. The source HEAD
captured by the integration harness identifies the harness, not necessarily
the binary. The report preserves the exact manifest, its hash, fixture source
hashes, raw trace hashes, source identity and executed binary hash per run.

`SUMMARY.txt` gives measured segment travel/tracking/upright observations and
the first physical failure. `observations.json` also retains the surrounding
raw one-second windows, skipped steps, route state, and contact tangential
speed. Supported foot-center travel includes rolling/rotation and must not
be read as contact slip. The contact-point velocity is measured separately.
Neither file writes tracker status or a gameplay verdict.
