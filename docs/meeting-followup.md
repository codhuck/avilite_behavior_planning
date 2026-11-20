# Meeting follow-up, 2026-11-17

The source requirements are the saved machine transcript, not a new manual
transcription. See [the requirement matrix](requirements-status.md).

Implemented in this follow-up:

- A reactive behavior-tree runtime and optional road stage, with typed STOP,
  traffic-light and crosswalk branches, reset semantics and node traces.
- Explicit tree selection in portable profiles and preservation of that choice
  by the road/race dispatcher. Existing stopped/paused switching restrictions
  remain in place.
- Correct plugin path resolution: an explicitly selected checkout no longer
  imports nested modules from another installed copy. Changing checkout also
  reloads its settings before applying the selected profile.
- A source bundle containing the matching core, bridge, behavior/perception
  plugin, data and tests, with a manifest and offline verification command.
- Automatic source snapshots for the parking trial runner, and completion of
  another noisy multi-LiDAR forward parking test with a crossing pedestrian.

## Validation boundaries

The plugin-path bug was first reproduced by a failing regression test. The
first live tree test selected a stale installed CARLA bridge and aborted at
shutdown; its result file was written before cleanup and is not an end-to-end
pass. The harness now writes successful output after cleanup and asserts the
actual bridge source path. The fixed live test completed 30 RGB/LiDAR steps,
rejected a black camera frame, and exited normally. Logs retain the original
failure and later results separately.

The forward parking rerun completed without contact, but its 7.5 cm pedestrian
gap is too small to call the earlier contact issue resolved. The test did not
execute an observation manoeuvre. No real vehicle or campus deployment has been
performed. Simulator localization and static surveyed geometry are still used.

No paper acceptance, real-world performance or superiority over competing
methods follows from these checks. The Overleaf article is a prior draft;
finite-ray study results and these integration changes are not automatically
promoted to peer-reviewed findings.
