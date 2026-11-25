# Observation contract checks, 25 November 2026

Changes are local to the stable `main-plugin` checkout on branch `main`.
No core type migration or neural-agent change is included.

## Behavior checked

- Capture time is retained through delayed inference. The first-frame simulation
  anchor uses assembly time, preserving capture-to-assembly delay. Explicit
  clock offsets reject stale first frames without learning an offset from them.
- Repeated frames, future captures, invalid assembly stamps and inference errors
  produce empty fault snapshots. Backward executor time resets the clock anchor.
- Marker identity and map association are separate. Unassociated observations
  cannot supply mapped signal/crossing states. Conflicting values for one map
  reference become unknown regardless of input order.
- Origin survives conversion and pickle serialization. Sensor-only behavior
  rejects simulator truth, scripted and unlabelled observations, including the
  legacy input path. The CARLA truth task refuses to overwrite a sensor result.

## Results

The full plugin suite passed **346 tests with no skips** using the matching
local AVLite core. The environment-contract file accounts for 61 tests; 42 were
added in this update. Three sensor profiles passed schema validation and JSON
round-trip checks, preserving the sensor-only policy. Clock offsets `null`, zero
and a negative finite offset round-tripped; NaN and infinity were rejected.
The isolated check against the unmodified AVLite 0.6.7 wheel passed **340 tests**,
with six skips for the proposed core execution API. Those six passed with the
local core. The stock check used a separate plugin copy and the wheel's extracted
core on `PYTHONPATH`, with dependencies reused from the existing environment.
This was not a clean dependency installation.

Ruff lint, formatting and `git diff --check` passed.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest \
  --rootdir=.. --import-mode=importlib -q tests
```

Run from the plugin checkout. Evidence is under
`logs/observation-contract-20261125/` in the development workspace.

## Limits

These are automated contract and behavior tests with synthetic sensor frames,
not a new live CARLA drive or a recognition-accuracy benchmark. Hardware needs
an established timestamp mapping: the default anchor cannot detect a delayed
first packet when its assembly stamp is also old. No clock-drift estimator,
multi-producer fusion framework or ROS transport was added. Provenance labels
are producer declarations, not authentication. Shared-core ownership still needs
agreement; see [the proposal](../proposals/environment-api.md).
