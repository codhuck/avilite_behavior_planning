# Executor compatibility

Planner selection uses `executer.stack_update()`, a context manager owned by the
executor. All candidate validation and pointer publication happen within that
boundary. New executors own their synchronization policy. A narrow legacy fallback supports
the synchronous adapter on stock AVLite 0.6.7, which lacks this context manager.

The local development core implements this protocol for its ordinary executors:

| Executor | Selection boundary | Validation |
| --- | --- | --- |
| `SyncExecuter` | Serialized with ticks; reentrant for inline tasks | Road and race: start, drive, refuse moving switch, stop, replace, resume |
| `AsyncThreadedExecuter` | Before workers start or after all workers exit | Same checks with actual planning/control threads |
| `PlanningSyncExecuter` | Legacy stock-0.6.7 adapter; uses core boundary when available | Existing plugin and mission tests |
| ROS / remote workers | Must implement remote publication and acknowledgement | Live stack replacement is not implemented by this change |

The standard-executor support requires the accompanying local core changes. It
is not a claim that released AVLite 0.6.7 already has this API. The default source
profile remains compatible with stock 0.6.7 through `PlanningSyncExecuter`.

To exercise the local development implementation:

```bash
python tools/prepare_coordinator_profile.py --mode road \
  --executor SyncExecuter --output /tmp/avlite-road-sync.yaml
python tools/prepare_coordinator_profile.py --mode race \
  --executor AsyncThreadedExecuter --output /tmp/avlite-race-threaded.yaml
```

Select the generated profile through AVLite's normal configuration mechanism.
No simulator, model download or change to the user's active profile is needed
to generate these files. Generation refuses to overwrite an existing file.

## Lifecycle and failure behavior

Both core executors emit `ExecutionStarted` once per reset before any worker can
actuate. Resume does not reselect planners or erase regulatory dwell. A failed
startup selection prevents worker launch. Planner time comes from the executor,
including when perception is disabled.

A paused flag is insufficient for threaded replacement: a worker may still be
blocked in a bridge call. The boundary rejects replacement until all workers
have exited. An asynchronous request made during execution is reported as a
rejection; it does not silently pause or restart the simulator. `request()` submits
an event: while another task is running, completion can be deferred. Consumers
should observe `PlanningStrategySelected` / `PlanningStrategyRejected` or a new
history entry, rather than treating the previous `last_result` as the new reply.

Candidate construction uses a separate PM snapshot with the same read-only map.
A failed constructor or first replan cannot overwrite the live observation or
regulatory route. Successful publication binds the new stages to the original
PM, keeps the same world and clears the old control command. A perception
replacement also discards earlier environment observations. Control remains
blocked until the required fresh perception and planning stages run.

## Boundaries

This change removes the custom-executor requirement for ordinary coordination
on the local core. Parking missions and experimental before-step hooks still
use their documented runtime adapter. It does not introduce moving handovers,
world migration, or transactional replacement inside external ROS workers.
A remote executor must override the boundary with a real worker protocol; the
base API rejects local replacement rather than changing only a display mirror.

Before proposing the core patch upstream, agree the lifecycle names and remote
executor extension point with maintainers. The [review diff and baseline manifest](proposals/execution-api/README.md) ship
with this repository. They have not been submitted upstream.
