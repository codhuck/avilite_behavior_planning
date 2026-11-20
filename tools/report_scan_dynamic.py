"""Report executed scan-policy trials separately from counterfactual view gains."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    from collections import Counter
    from pathlib import Path

    import numpy as np

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        protocol = json.loads((args.recording / "protocol.json").read_text())
        results = json.loads((args.recording / "results.json").read_text())
        assert [r["scenario"] for r in results] == protocol["suite"], "Incomplete or altered suite"
        for rel, digest in protocol["source_sha256"].items():
            path = (
                args.recording
                / "source_snapshot"
                / (rel[3:] if rel.startswith("../") else "avlite-behavior-hdmap/" + rel)
            )
            assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
        frozen_adapter = (
            args.recording / "source_snapshot/avlite-behavior-hdmap/visibility_parking.py"
        ).read_text()
        recovery_available = "self.recovery_search_expansions" in frozen_adapter
        rows = []
        for result in results:
            telemetry = result["telemetry"]
            times = np.array([r["time"] for r in telemetry])
            assert len(times) and np.isfinite(times).all() and np.all(np.diff(times) > 0)
            certificates = 0
            for row in telemetry:
                cert = row["certificate"]
                if cert is not None:
                    assert abs(cert["observation_stamp"] - row["acquisition_time"]) < 1e-6
                    assert len(cert["sources"]) == len(set(cert["sources"])) == 9
                    assert cert["released_length"] > 0
                    certificates += 1
            final = telemetry[-1]
            stopped_since = None
            for row in reversed(telemetry):
                if row["speed"] >= 0.01:
                    break
                stopped_since = row["time"]
            rows.append(
                {
                    "id": result["scenario"]["id"],
                    "type": result["type"],
                    "passed": result["passed"],
                    "final_state": result["final_state"],
                    "sim_seconds": float(times[-1]),
                    "wall_seconds": result["wall_seconds"],
                    "collisions": result["collisions"],
                    "collision_events": result.get("collision_events"),
                    "body_violations": result["body_violations"],
                    "position_error": result["position_error"],
                    "heading_error": result["heading_error"],
                    "pedestrian_crossed": result["pedestrian_crossed"],
                    "minimum_pedestrian_clearance": result["minimum_pedestrian_clearance"],
                    "maximum_pedestrian_overlap": result["maximum_pedestrian_overlap"],
                    "certified_frames": certificates,
                    "state_frames": dict(Counter(r["state"] for r in telemetry)),
                    "terminal_speed": final["speed"],
                    "terminal_stopped_since": stopped_since,
                    "reason": final["reason"],
                }
            )
        report = {
            "scope": __doc__,
            "recovery_expansion_correction": recovery_available,
            "source_snapshot_audit": "passed",
            "input_sha256": hashlib.sha256(
                (args.recording / "results.json").read_bytes()
            ).hexdigest(),
            "rows": rows,
            "passed": sum(r["passed"] for r in rows),
            "total": len(rows),
            "limits": "Known layouts, new seeds; integration trials, no paired superiority inference. See recovery_expansion_correction for the frozen implementation. Missing collision_events were not logged and must not be reconstructed from proximity alone.",
        }
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(
            len(results), 2, figsize=(10, 3 * len(results)), squeeze=False, layout="constrained"
        )
        for result, axes_row in zip(results, axes):
            data = result["telemetry"]
            t = [r["time"] for r in data]
            axes_row[0].plot([r["x"] for r in data], [r["y"] for r in data], color="#1f77b4")
            axes_row[0].scatter(*result["goal_pose"][:2], marker="x", color="#9a2a2a", label="Goal")
            axes_row[0].set(
                xlabel="World x (m)", ylabel="World y (m)", title=result["type"], aspect="equal"
            )
            axes_row[0].legend()
            axes_row[1].plot(t, [r["speed"] for r in data], label="Ego speed (m/s)")
            axes_row[1].plot(
                t,
                [
                    np.nan if r.get("pedestrian_clearance") is None else r["pedestrian_clearance"]
                    for r in data
                ],
                label="Pedestrian box gap (m)",
                color="#b47b25",
            )
            axes_row[1].set(
                xlabel="Simulation time (s)",
                title=f"{result['final_state']}; contacts: {result['collisions']}",
            )
            axes_row[1].legend(fontsize=8)
            for ax in axes_row:
                ax.grid(alpha=0.2)
                ax.spines[["top", "right"]].set_visible(False)
        fig.savefig(args.output / "executed_trials.pdf")
        fig.savefig(args.output / "executed_trials.png", dpi=160)
        print(json.dumps(report, indent=2))

    if __name__ == "__main__":
        main()
