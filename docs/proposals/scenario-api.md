# Scenario interface for discussion

Prepared on 26 November 2026. The meeting's automatic captions mention scenario
generation and possible core support. They do not establish a final API or an
assignment to replace all existing simulators' scenario systems.

## Working implementation

The plugin now contains versioned JSON/YAML road scenarios, seeded parameter
resolution, a BasicSim adapter, criteria-based evaluation and replay artifacts.
The adapter runs the AVLite perception/coordinator/planning/control stack.
The source files are under `scenarios/`; [the user guide](../scenarios.md)
describes the supported geometry and the explicitly scripted perception.

## Suggested ownership

| Component | Suggested owner |
| --- | --- |
| Run identity, resolved parameters, simulation clock and termination reason | Shared scenario API, after agreement |
| Backend lifecycle and trace/result interface | Shared protocol |
| Concrete actor creation, simulator tick and cleanup | BasicSim/CARLA/other adapters |
| Signal, crossing, race and parking criteria | Domain-specific evaluators |
| Parameter distributions and saved run artifacts | Reusable tooling; no driving decisions |
| Detector and planner choices | Existing AVLite strategies |

Keep scenario truth separate from the observation channel and give the evaluator
its own input. A sensor fault must not erase the physical actor against which
collision is checked. A strategy reporting success must not certify its own run.

The current format is a road-testing implementation, not a universal core schema.
Before migration, agree on extension/versioning rules and a small backend
protocol, then implement a second adapter to check that the boundary holds.
Race/parking adapters and rendered CARLA sensor scenarios remain separate work.
No core changes, upstream proposal or message were sent as part of this update.

## Meeting wording

I turned the road tests into reusable scenario files. They specify the actors,
signal changes, observation faults and acceptance criteria. The runner executes
our AVLite stack and saves the resolved inputs so we can replay a failure or
compare planners under the same conditions. The first adapter uses BasicSim
with explicitly scripted observations. Would you prefer a small common scenario
protocol in the core, with simulator adapters and domain-specific checks in plugins?
