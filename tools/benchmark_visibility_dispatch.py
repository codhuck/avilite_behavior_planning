"""Seeded decision-level benchmark; not a closed-loop parking success claim.

All policies receive identical candidate trajectories and observations. Hidden
obstacles are evaluator-only. Ground truth is never passed to the dispatcher.
"""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import csv
    import json
    import sys
    import time
    from pathlib import Path

    import numpy as np
    from shapely.geometry import GeometryCollection, box

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(PLUGIN))
    from _bootstrap import load_plugin

    load_plugin()

    from avlite_coordination_demo.parking.dispatch import (
        Maneuver,
        VisibilityDispatcher,
        VisibilitySnapshot,
        expected_gain,
        swept_area,
    )

    def trajectory(name, end, kind, mode, gain=0):
        x = np.linspace(0, end, int(abs(end) / 0.04) + 1)
        return Maneuver(
            name, kind, mode, np.column_stack((x, x * 0, x * 0)), 1 if end > 0 else -1, 0.2, gain
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--cases", type=int, default=100)
        parser.add_argument("--seed", type=int, default=2027)
        parser.add_argument(
            "--output", type=Path, default=Path("logs/visibility_dispatch_benchmark")
        )
        args = parser.parse_args()
        if args.cases < 1:
            parser.error("--cases must be positive")
        rng = np.random.default_rng(args.seed)
        rows = []
        for case in range(args.cases):
            left, right = rng.uniform(3, 9, 2)
            free = box(-left, -3, right, 3)
            occupied = GeometryCollection()
            # A hidden obstacle is beyond observed free space, chosen independently
            # of the policy. Full swept footprint is used for collision evaluation.
            side = rng.choice([-1, 1])
            x = -left - 0.6 if side < 0 else right + 0.6
            truth = box(x - 0.3, -0.8, x + 0.3, 0.8)
            age = float(rng.choice([0, 0.1, 0.25, 0.5]))
            snap = VisibilitySnapshot(free, occupied, 0)
            parking = [
                trajectory("forward", 5, "park", "perpendicular_forward"),
                trajectory("reverse", -4, "park", "parallel"),
            ]
            target = box(right + 0.1, -1, right + 3, 1)
            baseline = expected_gain([0, 0, 0], snap, target, sensor_range=5)
            views = []
            for end in [-1.2, -0.6, 0.6, 1.2]:
                gain = expected_gain([end, 0, 0], snap, target, sensor_range=5) - baseline
                views.append(
                    trajectory(f"view_{end}", end, "observe", "perpendicular_forward", gain)
                )
            policies = {
                "optimistic": parking[:1],
                "stop_on_unknown": parking,
                "fixed_conservative": parking[:1],
                "fixed_planner_active": parking[:1] + views,
                "visibility_dispatch": parking + views,
            }
            for policy, candidates in policies.items():
                start = time.perf_counter()
                if policy == "optimistic":
                    selected, state, reason = parking[0], "PARK", "ignores_unknown_and_age"
                else:
                    result = VisibilityDispatcher().select(
                        candidates,
                        snap,
                        age,
                        [0, 0, 0],
                        0,
                        4,
                        2,
                        {"parallel", "perpendicular_forward"},
                    )
                    selected, state, reason = result.maneuver, result.state, result.reason
                latency = (time.perf_counter() - start) * 1000
                sweep = swept_area(selected.poses, 4, 2, 0) if selected else GeometryCollection()
                rows.append(
                    dict(
                        case=case,
                        policy=policy,
                        state=state,
                        reason=reason,
                        selected=selected.name if selected else "",
                        age=age,
                        evaluator_collision=bool(sweep.intersects(truth)),
                        unknown_area=float(sweep.difference(free).area),
                        latency_ms=latency,
                    )
                )
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output / "decisions.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary = {
            "scope": "one-step decision benchmark; no vehicle dynamics, no parking completion rate",
            "seed": args.seed,
            "cases": args.cases,
            "policies": {},
        }
        for policy in policies:
            data = [r for r in rows if r["policy"] == policy]
            summary["policies"][policy] = dict(
                park=sum(r["state"] == "PARK" for r in data),
                observe=sum(r["state"] == "OBSERVE" for r in data),
                stop=sum(r["state"] == "STOP" for r in data),
                evaluator_collisions=sum(r["evaluator_collision"] for r in data),
                unknown_entries=sum(r["unknown_area"] > 1e-8 for r in data),
                latency_p95_ms=float(np.percentile([r["latency_ms"] for r in data], 95)),
            )
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2))

    if __name__ == "__main__":
        main()
