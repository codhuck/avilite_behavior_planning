"""Audit and summarize the frozen follow-up, preserving missing proposals."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    from pathlib import Path

    import numpy as np

    def main():
        p = argparse.ArgumentParser(description=__doc__)
        p.add_argument("recording", type=Path)
        p.add_argument("--output", type=Path, required=True)
        a = p.parse_args()
        protocol = json.loads((a.recording / "protocol.json").read_text())
        rows = json.loads((a.recording / "results.json").read_text())
        assert [r["scene"] for r in rows] == protocol["scenes"], "Incomplete/mismatched series"
        checked = 0
        points = 0
        hashes = {}
        for relative, expected in protocol["sources"].items():
            assert (
                hashlib.sha256(
                    (a.recording / "source_snapshot" / relative).read_bytes()
                ).hexdigest()
                == expected
            )
        pose_checks = 0
        for row in rows:
            root = a.recording / row["scene"]["id"]
            groups = {p.name.rsplit("_cloud_", 1)[0] for p in root.glob("*.npz")}
            assert "initial" in groups
            for group in groups:
                files = sorted(root.glob(group + "_cloud_*.npz"))
                assert len(files) == 9
                stamps = []
                poses = []
                for file in files:
                    with np.load(file) as raw:
                        assert raw["points"].ndim == 2 and raw["points"].shape[1] == 4
                        assert np.isfinite(raw["points"]).all()
                        points += len(raw["points"])
                        stamps.append(float(raw["stamp"]))
                        poses.append(raw["pose"])
                        error = raw["pose"] - raw["requested_pose"]
                        yaw_error = np.arctan2(np.sin(error[3]), np.cos(error[3]))
                        assert np.linalg.norm(error[:3]) < 0.001 and abs(yaw_error) < 1e-4
                        pose_checks += 1
                    hashes[str(file.relative_to(a.recording))] = hashlib.sha256(
                        file.read_bytes()
                    ).hexdigest()
                assert np.ptp(stamps) < 1e-6
                assert np.max(np.abs(np.array(poses) - poses[0])) < 1e-6
                checked += 1
            if row["proposal_available"]:
                for method, choice in row["choices"].items():
                    key = {
                        "task": "task_score",
                        "task_beams": "beam_score",
                        "bay_area": "bay_score",
                        "task_scan": "scan_score",
                    }.get(method)
                    eligible = (
                        []
                        if key is None
                        else [
                            c
                            for c in row["candidates"]
                            if c["admissible"]
                            and c[key] >= (0.01 if method == "task_scan" else 1e-12)
                        ]
                    )
                    best = max(eligible, key=lambda c: c[key]) if eligible else None
                    assert choice["name"] == (best["name"] if best else "stay")
                    expected = best["acquired_weighted_gain"] if best else row["stationary_gain"]
                    assert choice["gain"] == expected
                    assert (
                        abs(
                            choice["improvement_over_stationary"]
                            - (expected - row["stationary_gain"])
                        )
                        < 1e-12
                    )
            else:
                assert all(
                    c["gain"] is None and c["name"] == "no_proposal"
                    for c in row["choices"].values()
                )
        valid = [r for r in rows if r["proposal_available"]]
        summary = {}
        for method in protocol["methods"]:
            summary[method] = {
                "selections": sum(r["choices"][method]["name"] != "stay" for r in valid),
                "mean_gain_conditional_on_proposal": float(
                    np.mean([r["choices"][method]["gain"] for r in valid])
                ),
                "mean_improvement_over_stationary": float(
                    np.mean([r["choices"][method]["improvement_over_stationary"] for r in valid])
                ),
            }
        report = {
            "scope": "Prospective static counterfactual LiDAR acquisitions, not executed manoeuvres or completed parking",
            "total_scenes": len(rows),
            "scenes_with_proposals": len(valid),
            "no_proposal": [r["scene"]["id"] for r in rows if not r["proposal_available"]],
            "summary": summary,
            "rows": rows,
            "audited_acquisitions": checked,
            "point_count": points,
            "measured_requested_pose_checks": pose_checks,
            "source_sha256": hashes,
            "source_snapshot_audit": "passed",
            "method_selection_audit": "passed",
            "scan_better_than_bay": sum(
                r["choices"]["task_scan"]["gain"] > r["choices"]["bay_area"]["gain"] for r in valid
            ),
            "beam_better_than_bay": sum(
                r["choices"]["task_beams"]["gain"] > r["choices"]["bay_area"]["gain"] for r in valid
            ),
        }
        a.output.mkdir(parents=True, exist_ok=True)
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(10, 3.5), layout="constrained")
        labels = [
            "Fresh scan",
            "Bay visibility",
            "Task line of sight",
            "Task + beam rings",
            "Finite-ray raster",
        ]
        methods = protocol["methods"]
        colors = ["#767d87", "#1f77b4", "#b47b25", "#20876a", "#993f86"]
        x = np.arange(len(valid))
        for i, (method, color) in enumerate(zip(methods, colors)):
            axes[0].plot(
                x,
                [r["choices"][method]["gain"] for r in valid],
                ".-",
                label=labels[i],
                color=color,
                lw=1,
                markersize=5,
            )
        axes[0].set(
            xlabel=f"Scene with geometric proposal ({len(valid)} of {len(rows)})",
            ylabel="Acquired weighted area / (1 + length)",
            ylim=(0, None),
        )
        axes[0].legend(fontsize=7)
        for method, color in zip(methods[1:], colors[1:]):
            predicted = []
            actual = []
            key = {
                "bay_area": "bay_score",
                "task": "task_score",
                "task_beams": "beam_score",
                "task_scan": "scan_score",
            }[method]
            for r in valid:
                for c in r["candidates"]:
                    if c["admissible"]:
                        predicted.append(c[key])
                        actual.append(c["acquired_weighted_gain"])
            axes[1].scatter(
                predicted,
                actual,
                s=13,
                alpha=0.55,
                label=labels[methods.index(method)],
                color=color,
            )
        axes[1].set(
            xlabel="Predicted view utility (method-specific)",
            ylabel="Acquired task-weighted area / cost",
        )
        axes[1].legend(fontsize=7)
        for ax in axes:
            ax.spines[["top", "right"]].set_visible(False)
            ax.grid(alpha=0.15)
        fig.savefig(a.output / "view_selection.pdf")
        fig.savefig(a.output / "view_selection.png", dpi=180)
        print(
            json.dumps(
                {k: v for k, v in report.items() if k not in ["rows", "source_sha256"]}, indent=2
            )
        )

    if __name__ == "__main__":
        main()
