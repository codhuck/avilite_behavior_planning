"""Make descriptive paired-study figures; no confidence intervals from correlated frames."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    from pathlib import Path

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    def evidence_plot(report, output):
        names = ["uniform_one", "adaptive", "uniform_two", "roof_only"]
        labels = ["One source", "Adaptive", "Two sources", "Roof/adaptive"]
        colors = ["#b56622", "#166695", "#4b8053", "#7b6e8b"]
        if any(r["policy"] == "roof_uniform_one" for r in report["frames"][0]["rows"]):
            names.append("roof_uniform_one")
            labels.append("Roof/one")
            colors.append("#555555")
        fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.4), layout="constrained")
        for ax, key, ylabel in zip(
            axes,
            ["free_area", "false_free_overlap"],
            ["Observed free area (m²)", "False-free box overlap (m²)"],
        ):
            for i, name in enumerate(names):
                values = [
                    next(r[key] for r in f["rows"] if r["policy"] == name) for f in report["frames"]
                ]
                ax.scatter(
                    np.full(len(values), i) + np.linspace(-0.1, 0.1, len(values)),
                    values,
                    s=15,
                    color=colors[i],
                    alpha=0.8,
                )
                ax.plot([i - 0.2, i + 0.2], [np.mean(values)] * 2, color=colors[i], lw=2)
            ax.set_xticks(range(len(names)), labels, rotation=15)
            ax.set_ylabel(ylabel)
            ax.grid(axis="y", alpha=0.2)
        fig.savefig(output / "matched_evidence.pdf")
        fig.savefig(output / "matched_evidence.png", dpi=170)
        plt.close(fig)

    def paired_plot(report, output):
        labels = {
            "full": "Complete",
            "roof_only": "Roof only",
            "uniform_one": "Uniform one",
            "uniform_two": "Uniform two",
            "no_speed_search": "No speed search",
            "no_observation": "No observation",
            "full_segment": "Full segment",
        }
        policies = [p for p in labels if p in report["policies"]]
        pairs = sorted({t["scenario"]["pair_id"] for t in report["trials"]})
        fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.8), layout="constrained")
        markers = ["o", "s", "^"]
        for i, p in enumerate(policies):
            for j, pair in enumerate(pairs):
                t = next(
                    t
                    for t in report["trials"]
                    if t["scenario"]["pair_id"] == pair and t["scenario"]["ablation_policy"] == p
                )
                x = i + (j - 1) * 0.16
                if t["parking_completed"]:
                    axes[0].scatter(
                        x, t["simulated_seconds"], marker=markers[j], color="#166695", s=23
                    )
                else:
                    axes[0].scatter(
                        x, 0, marker=markers[j], facecolors="none", edgecolors="#b33c35", s=34
                    )
                    axes[0].scatter(x, 0, marker="x", color="#b33c35", s=20)
                if t.get("minimum_pedestrian_clearance") is not None:
                    axes[1].scatter(
                        x,
                        t["minimum_pedestrian_clearance"],
                        marker=markers[j],
                        color="#166695",
                        s=23,
                    )
        for ax in axes:
            ax.set_xticks(
                range(len(policies)), [labels[p] for p in policies], rotation=45, ha="right"
            )
            ax.grid(axis="y", alpha=0.2)
        axes[0].set_ylabel("Completion time (s)")
        axes[1].set_ylabel("Minimum pedestrian clearance (m)")
        fig.savefig(output / "paired_outcomes.pdf")
        fig.savefig(output / "paired_outcomes.png", dpi=170)
        plt.close(fig)

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("report", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        args.output.mkdir(parents=True, exist_ok=True)
        plt.rcParams.update(
            {
                "font.size": 8,
                "pdf.fonttype": 42,
                "axes.spines.top": False,
                "axes.spines.right": False,
            }
        )
        report = json.loads(args.report.read_text())
        if "frames" in report:
            evidence_plot(report, args.output)
        else:
            paired_plot(report, args.output)

    if __name__ == "__main__":
        main()
