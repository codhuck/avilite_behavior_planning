# Scenario runner checks, 26 November 2026

The implementation is in the planning plugin's `main` checkout. No AVLite core
change or neural-agent change is part of this update.

## Checks

| Check | Result |
| --- | --- |
| Full plugin suite with the local core | 367 passed, no skips |
| New scenario tests with the local core | 21 passed |
| New scenario tests against unmodified AVLite 0.6.7 | 21 passed |
| Road scenarios × parameter seeds × planning configurations | 12/12 completed with criteria satisfied |
| Full delayed-observation replay | Exact trace SHA-256 match |
| Permanently occupied crossing | Correctly failed: 35.85 s stopped and unfinished; no contact |
| Style and formatting | Ruff passed |

The matrix uses two scenario files, three seeds and both `road-rules` and
`road-tree`. Both implement the same traffic rules. These are repeated scripted
integration checks, not twelve independent traffic distributions or a claim of
superiority over another planner. Across these twelve runs, the evaluator
reported no contact, non-green light entry, occupied-crossing entry or off-road
time. Completion requires both position near the endpoint and low speed.

The stock-core tests used an isolated plugin copy and an extracted AVLite 0.6.7
wheel on `PYTHONPATH`, reusing installed dependencies. This was not a clean OS or
dependency installation. The stock check also completed a full delayed-observation
scene before the default-value serialization fix; the final 21-test run includes
the replay regression after that fix.

## Failures retained during development

The initial route came from a short handover demo and requested 3 m/s through
its endpoint. The new evaluator correctly reported that this was not a finished
mission and recorded departure from the road. A first braking profile still
overshot the endpoint. A dedicated scenario route now includes a more gradual
terminal reference-speed profile; the old handover demo is unchanged. This is
fixture preparation, not a claim that arbitrary routes now acquire a goal stop.

Full-scene replay then exposed an int/float serialization mismatch: a stationary
actor's default speed was written as `0` initially and `0.0` after reload. Physical
traces were numerically equal, but byte hashes differed. Schema defaults are now
validated through the same type conversions as supplied values. The regression
test runs a scenario with a stationary actor and checks the replay hash.

Independent evaluator tests inject contacts between tick endpoints, non-green
signal entry, crossing occupancy and departure from the lane. They also check a
signal transition inside a tick. A fake `ARRIVED` behavior state cannot satisfy
completion. Backend failure produces an error report and restores global AVLite
settings. Empty suites, changed replay assets and incompatible source fingerprints
are refused.

## Evidence and reproduction

Evidence is in `logs/scenarios-20261126/` in the development checkout. The initial
failures remain separate from `final-comparison/`; logs are not included in the
source distribution. The source includes fixtures, test code and run commands.

```bash
python tools/run_scenarios.py suite \
  data/scenarios/light-and-crossing.yaml \
  data/scenarios/delayed-observations.yaml \
  --seeds 0 1 2 --strategies road-rules road-tree \
  --output logs/scenario-validation
```

See [the guide](../scenarios.md) for replay, negative cases and metrics.
The BasicSim adapter currently supports one straight, constant-width driving
lane. Observations are explicitly scripted; the van does not create image
occlusion. There was no new live CARLA run, no sensor inference benchmark and no
real-vehicle test. CARLA/race/parking adapters and shared-core adoption remain
separate extensions. The CI configuration includes a scenario comparison and
replay step, but no remote workflow has been run for these unpushed changes.
