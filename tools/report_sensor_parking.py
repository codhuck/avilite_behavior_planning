"""Summarize completed CARLA parking trials without promoting checkpoints to passes."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import importlib.metadata
    import json
    import platform
    from collections import Counter
    from pathlib import Path

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Polygon

    def summarize(result):
        rows = result["telemetry"]
        summary = {key: value for key, value in result.items() if key != "telemetry"}
        summary["frames"] = len(rows)
        pedestrian_rows = [row for row in rows if row.get("pedestrian_position") is not None]
        jumps = [
            current["time"]
            for previous, current in zip(pedestrian_rows, pedestrian_rows[1:])
            if np.linalg.norm(
                np.asarray(current["pedestrian_position"]) - previous["pedestrian_position"]
            )
            > max(
                0.5,
                2
                * result.get("scenario", {}).get("pedestrian_speed", 0.6)
                * (current["time"] - previous["time"]),
            )
        ]
        summary["pedestrian_position_discontinuities"] = jumps
        summary["pedestrian_encounter_valid"] = (
            bool(pedestrian_rows) and not jumps and not result.get("scenario_error")
            if result.get("scenario", {}).get("pedestrian")
            else None
        )
        summary["simulated_seconds"] = rows[-1]["time"] if rows else 0
        budget = result.get("scenario", {}).get("max_sim_seconds")
        summary["termination"] = (
            "accepted_goal"
            if result["passed"]
            else "time_budget"
            if budget is not None
            and rows
            and rows[-1]["time"] >= budget - result["fixed_delta_seconds"] - 1e-6
            else result["final_state"]
        )
        if result.get("scenario", {}).get("pedestrian"):
            summary["pedestrian_stop_seconds"] = (
                result.get("pedestrian_stop_frames", 0) * result["fixed_delta_seconds"]
            )
            summary["pedestrian_pause_recorded_seconds"] = (
                sum(bool(r.get("pedestrian_paused")) for r in rows) * result["fixed_delta_seconds"]
            )
            summary["stopped_during_pedestrian_pause_seconds"] = (
                sum(bool(r.get("pedestrian_paused")) and r["speed"] < 0.01 for r in rows)
                * result["fixed_delta_seconds"]
            )
            commanded_stops = [
                r
                for r in rows
                if r["speed"] < 0.01
                and r["state"] == "PARKING_BLOCKED"
                and r.get("certificate") is None
                and "pedestrian_clearance" in r
                and r["pedestrian_clearance"] < 5
                and not r.get("pedestrian_crossed")
            ]
            summary["commanded_stop_near_pedestrian_seconds"] = (
                len(commanded_stops) * result["fixed_delta_seconds"]
            )
            summary["commanded_stop_during_pedestrian_pause_seconds"] = (
                sum(bool(r.get("pedestrian_paused")) for r in commanded_stops)
                * result["fixed_delta_seconds"]
            )
            crossed = next((r for r in rows if r.get("pedestrian_crossed")), None)
            resumed = next(
                (r for r in rows if crossed and r["time"] >= crossed["time"] and r["speed"] > 0.03),
                None,
            )
            summary["motion_after_crossing_seconds"] = (
                resumed["time"] - crossed["time"] if resumed else None
            )
        summary["max_false_free_box_overlap_m2"] = max(
            (row.get("false_free_obstacle_area", 0) for row in rows), default=0
        )
        certificates = [row["certificate"] for row in rows if row.get("certificate")]
        summary["certified_decisions"] = len(certificates)
        summary["state_frames"] = dict(Counter(row["state"] for row in rows))
        summary["reason_frames"] = dict(Counter(row["reason"] for row in rows))
        summary["max_measured_speed_mps"] = max((row["speed"] for row in rows), default=0)
        summary["minimum_released_length_m"] = min(
            (c["released_length"] for c in certificates), default=None
        )
        summary["all_certificates_fresh"] = all(
            abs(c["observation_stamp"] - c["checked_at"]) <= 1e-5 for c in certificates
        )
        missing = [row for row in rows if row.get("reason") == "missing_visibility_observation"]
        summary["missing_observation_frames"] = len(missing)
        summary["missing_observations_have_no_certificate"] = all(
            row.get("certificate") is None for row in missing
        )
        if missing:
            summary["blackout_entry_speed_mps"] = missing[0]["speed"]
            summary["blackout_exit_speed_mps"] = missing[-1]["speed"]
            stopped = next((row for row in missing if row["speed"] < 0.01), None)
            summary["blackout_time_to_below_001_mps"] = (
                stopped["time"] - missing[0]["time"] if stopped else None
            )
        return summary

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        results = json.loads((args.recording / "results.json").read_text())
        summaries = [summarize(result) for result in results]
        has_pedestrians = any(result.get("scenario", {}).get("pedestrian") for result in results)
        args.output.mkdir(parents=True, exist_ok=True)
        root = Path(__file__).resolve().parents[2]
        source_paths = sorted((root / "avlite-behavior-hdmap").glob("*.py")) + [
            root / "avlite-bridge-carla" / "carla_bridge.py",
            root / "avlite-bridge-carla" / "settings.py",
            Path(__file__).with_name("check_parking_carla.py"),
        ]
        protocol_path = args.recording / "protocol.json"
        protocol = json.loads(protocol_path.read_text()) if protocol_path.exists() else None
        actual_trials = [(r.get("scenario", {}).get("id"), r["repeat"]) for r in results]
        expected_trials = (
            [
                (case["id"], repeat)
                for case in protocol["suite"]
                for repeat in range(protocol["args"]["repeats"])
            ]
            if protocol
            else actual_trials
        )
        missing_trials = sorted(set(expected_trials) - set(actual_trials))
        duplicates = len(actual_trials) - len(set(actual_trials)) if protocol else 0
        suite_complete = (
            not missing_trials and not duplicates and len(expected_trials) == len(actual_trials)
        )
        payload = dict(
            scope="Surveyed flat-pad CARLA LiDAR trials; not real-world validation",
            recording=str(args.recording),
            passed=sum(bool(row["passed"]) for row in summaries),
            total=len(summaries),
            trials=summaries,
            planned_trials=len(expected_trials),
            suite_complete=suite_complete,
            missing_trials=missing_trials,
            duplicate_trials=duplicates,
            runtime_matches_protocol=all(
                hashlib.sha256((root / "avlite-behavior-hdmap" / path).read_bytes()).hexdigest()
                == digest
                for path, digest in protocol["source_sha256"].items()
            )
            if protocol
            else None,
            python=platform.python_version(),
            packages={
                name: importlib.metadata.version(name)
                for name in ("carla", "numpy", "scipy", "shapely", "pydantic", "matplotlib")
            },
            source_sha256_at_report_time={
                str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in source_paths
            },
            limitations=[
                "CARLA ground-truth localization; surveyed bay and ground elevation",
                "Roof plus eight low-mounted LiDAR devices; noise/dropoff are recorded per scenario",
                "Scripted pedestrian crossings, not arbitrary crowds"
                if has_pedestrians
                else "Static neighbouring vehicles; no moving pedestrian validation",
                "False-free evaluator-box overlap is not a collision count",
                "Certificate speed includes measured speed; it is not a target-tracking error metric",
                "Small manually constructed suite; not evidence of statistical superiority",
                "Source hashes describe report time, not historical development runs",
            ],
        )
        (args.output / "report.json").write_text(json.dumps(payload, indent=2) + "\n")
        columns = 4 if has_pedestrians else 3
        fig, axes = plt.subplots(
            len(results), columns, figsize=(4.3 * columns, 3.5 * len(results)), squeeze=False
        )
        for index, result in enumerate(results):
            rows = result["telemetry"]
            t = np.asarray([row["time"] for row in rows])
            case_dir = args.recording
            if (
                result.get("scenario", {}).get("id")
                and (case_dir / result["scenario"]["id"]).exists()
            ):
                case_dir = case_dir / result["scenario"]["id"]
            geometry_path = case_dir / f"{result['type']}_{result['repeat']}_geometry_0.json"
            if geometry_path.exists():
                geometry = json.loads(geometry_path.read_text())
                axes[index, 0].add_patch(
                    Polygon(
                        geometry["bay"]["coordinates"][0],
                        fill=False,
                        edgecolor="green",
                        linestyle="--",
                        label="surveyed bay",
                    )
                )
                for obstacle in geometry["obstacles"]:
                    axes[index, 0].add_patch(
                        Polygon(obstacle["coordinates"][0], facecolor="0.75", edgecolor="0.4")
                    )
            axes[index, 0].plot([r["x"] for r in rows], [r["y"] for r in rows], lw=1.5)
            pedestrian_positions = [
                row["pedestrian_position"]
                for row in rows
                if "pedestrian_position" in row
                and (
                    not summaries[index]["pedestrian_position_discontinuities"]
                    or row["time"] < summaries[index]["pedestrian_position_discontinuities"][0]
                )
            ]
            if pedestrian_positions:
                points = np.asarray(pedestrian_positions)
                axes[index, 0].plot(
                    points[:, 0], points[:, 1], ":", color="purple", label="pedestrian"
                )
            axes[index, 0].scatter(rows[0]["x"], rows[0]["y"], marker="o", label="start")
            axes[index, 0].scatter(rows[-1]["x"], rows[-1]["y"], marker="x", label="end")
            dimensions = result.get("vehicle_geometry")
            if dimensions:
                final = rows[-1]
                c, s = np.cos(final["yaw"]), np.sin(final["yaw"])
                corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [
                    dimensions["length"] / 2,
                    dimensions["width"] / 2,
                ]
                corners = corners @ np.array([[c, s], [-s, c]]) + [final["x"], final["y"]]
                axes[index, 0].add_patch(
                    Polygon(corners, fill=False, edgecolor="tab:blue", label="final vehicle")
                )
            condition = (
                "noisy pedestrian" if result.get("scenario", {}).get("pedestrian") else "geometry"
            )
            outcome = "PASS" if result["passed"] else "FAIL"
            if summaries[index]["pedestrian_encounter_valid"] is False:
                outcome += " (invalid walker encounter)"
            mode_label = {
                "perpendicular_forward": "Forward perpendicular",
                "perpendicular_reverse": "Reverse perpendicular",
                "parallel": "Parallel",
            }.get(result["type"], result["type"])
            group = (
                "Follow-up"
                if "followup" in result.get("scenario", {}).get("split", "")
                else "Regression"
                if result.get("scenario", {}).get("split", "").startswith("regression")
                else "New"
            )
            axes[index, 0].set(
                xlabel="x (m)",
                ylabel="y (m)",
                title=f"{mode_label}\n{group}: {condition} — {outcome}",
            )
            axes[index, 0].axis("equal")
            axes[index, 0].legend()
            axes[index, 1].plot(t, [r["speed"] for r in rows], lw=1, label="measured")
            axes[index, 1].plot(
                t,
                [r["certificate"]["speed_bound"] if r.get("certificate") else np.nan for r in rows],
                lw=0.8,
                alpha=0.65,
                label="certificate bound",
            )
            start = result.get("blackout_start")
            if start is not None:
                dt = result["fixed_delta_seconds"]
                axes[index, 1].axvspan(
                    start * dt, (start + result["blackout_frames"]) * dt, color="red", alpha=0.2
                )
            axes[index, 1].set(xlabel="simulation time (s)", ylabel="speed (m/s)")
            paused = [r["time"] for r in rows if r.get("pedestrian_paused")]
            if paused:
                axes[index, 1].axvspan(
                    min(paused), max(paused), color="purple", alpha=0.15, label="pedestrian pause"
                )
            axes[index, 1].legend()
            axes[index, 2].plot(t, [r.get("false_free_obstacle_area", 0) for r in rows], lw=1)
            axes[index, 2].set(xlabel="simulation time (s)", ylabel="false-free box overlap (m²)")
            if has_pedestrians:
                axes[index, 3].plot(
                    t, [r.get("pedestrian_clearance", np.nan) for r in rows], color="purple"
                )
                axes[index, 3].axhline(0, color="red", linestyle="--", lw=0.8)
                axes[index, 3].set(xlabel="simulation time (s)", ylabel="pedestrian clearance (m)")
                if paused:
                    axes[index, 3].axvspan(min(paused), max(paused), color="purple", alpha=0.15)
                if not pedestrian_positions:
                    axes[index, 3].set_axis_off()
                    axes[index, 3].text(
                        0.5,
                        0.5,
                        "No pedestrian in this case",
                        ha="center",
                        va="center",
                        transform=axes[index, 3].transAxes,
                        color="0.4",
                    )
            for ax in axes[index]:
                ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(args.output / "trials.png", dpi=180)
        fig.savefig(args.output / "trials.pdf")
        plt.close(fig)
        print(
            json.dumps(
                dict(
                    passed=payload["passed"], total=payload["total"], suite_complete=suite_complete
                )
            )
        )
        return 0 if suite_complete else 2

    if __name__ == "__main__":
        raise SystemExit(main())
