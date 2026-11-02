# PLAY-04 automated visual capture

Development builds with `OPENMW_RAGDOLL_DEV_API` expose a renderer screenshot
through the existing global test package:

```lua
local ragdoll = require('openmw.ragdolltest')
ragdoll.captureScreenshot()
```

The request queues the normal screenshot action on the engine thread. It captures
the next rendered game frame, including the GUI, without changing OS focus,
switching desktops or synthesizing keyboard input. It accepts no path and returns
no filename or completion signal. Rendering must be enabled. Keep the game alive
for subsequent frames so the asynchronous image writer can finish; multiple
requests before the next rendered frame can coalesce into one screenshot.

The integration runner sets each suite's `user-data` directory. For a run whose
`--output` is `runtime-PLAY-04-visual`, the default PNG files are therefore:

```text
runtime-PLAY-04-visual/test_play04_handling/userdata/screenshots/screenshot000.png
runtime-PLAY-04-visual/test_play04_handling/userdata/screenshots/screenshot001.png
```

The engine chooses the first unused sequential filename. Other suites use their
own suite name in this path, and `General/screenshot format` changes the extension.
Use a fresh runtime directory and record phase labels/physics steps in test
telemetry to associate requests with images. A request itself is not proof that
an image was written or that its pixels show the intended pose.

After the engine exits, enumerate and decode the files, check that the expected
captures exist and are nonempty, and inspect the images. Review weapon position,
both hands, joint bends, body intersections and transitions from useful camera
angles. Include idle, stroke extremes and release, with first-person and
third-person views. Preserve the source/binary manifest beside the captures.
Screenshots supplement physical telemetry; a still frame cannot establish
response time or a continuous sweep.

## Optional existing video path

`scripts/integration_tests.py --video_directory <directory>` records each test's
OpenMW window with the existing `scripts/test_recording.py` helper. On Windows it
requires FFmpeg on `PATH` and uses `gdigrab` at 15 FPS, with limits of 60 seconds
and 24 MiB per clip and 512 MiB total. The integration runner raises the render
resolution to 1280 by 720 for video; keep performance qualification separate.

The PLAY-04 private-desktop runner must forward `--video_directory` before this
option is usable through that wrapper. Its child-process startup configuration
must put both OpenMW and FFmpeg on the same private desktop. Do not switch the
user's desktop or focus the game to obtain a recording. Inspect the `.json` and
`.ffmpeg.log` sidecars and actual decoded frames: successful process completion
does not establish that a hidden-window recording contains useful pixels.
