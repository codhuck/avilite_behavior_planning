# Mission validation — 23 November 2026

The drive-to-parking task was checked against unmodified AVLite 0.6.7,
commit `1df512a3718d78a4d737e07178a9db03cf5d4f05`.

| Check | Result |
| --- | --- |
| Standalone plugin tests on the pinned core | 192 passed |
| Related development-workspace regression tests | 176 passed |
| Complete BasicSim mission scenarios | 15 passed |
| Generated profiles loaded through the stock factory | 3 passed |
| Ruff lint and formatting | Passed |

Each of the three parking types was tested with a clear bay, occupied bay,
bay that becomes clear, missing observations and stale observations. The
vehicle physically drove about 7.87 m before settling at the approach pose.
Clear cases completed; unavailable spaces or observations caused a latched
stop. There were no body collisions or drivable-area boundary violations in
these fixtures.

Additional tests cover preferred-space fallback, infeasible candidate paths,
world-speed checks at handover, cancellation, explicit retry, wrong approach
heading, reset to the original planner, lost observations during parking,
configuration errors and decision-file output.

These are BasicSim results with simulated occupancy. The approach corridor
contains no moving traffic. This validation does not establish performance in
CARLA, partial-coverage LiDAR selection or real-road operation. The monitor's
terminal output was tested; its Tk window could not be visually inspected in
the test session because the display was unavailable.

Reproduction commands are in [missions.md](../missions.md). The scenario runner
records poses and transitions in its output JSON. The compatibility workflow
now includes the mission scenarios and factory checks. The results above were
obtained locally; GitHub Actions runs the checks separately on proposed changes.
