# Execution API proposal

The planning task uses an executor-owned update boundary, startup events and
explicit planning/perception invalidation. The attached diff contains the four
local core-file changes needed to exercise this API with ordinary Sync and
Async executors. The plugin itself stays in this repository's `main` branch.

- [Review diff](core-execution-api.patch)
- [Baseline and result hashes](baseline.json)
- [Executor behavior and restrictions](../../executor-compatibility.md)
- [Observation model proposal](../../environment-observations.md)

This is a review artifact, not an installation patch for stock AVLite 0.6.7.
The baseline already contained local core changes; its exact SHA-256 hashes are
listed in the manifest. The development checkout already includes the diff.
Do not apply it again there. No installer applies it automatically.

The released-core profile continues to use `PlanningSyncExecuter`. An upstream
port requires agreement on the lifecycle API and testing against the target
revision. ROS worker publication/acknowledgement and the final shared marker
module remain separate integration work. No upstream approval is implied.
