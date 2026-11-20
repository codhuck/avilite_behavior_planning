"""Plot measured decisions and a recorded closed-loop run without success relabeling."""

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
    from matplotlib.patches import Rectangle

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--decisions", type=Path, default=Path("logs/visibility_dispatch_holdout/summary.json")
        )
        parser.add_argument(
            "--run-dir", type=Path, default=Path("logs/visibility_closed_loop_final")
        )
        parser.add_argument("--output", type=Path, default=Path("logs/itsc_visibility_study"))
        args = parser.parse_args()
        summary = json.loads(args.decisions.read_text())
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
        policies = list(summary["policies"])
        labels = [
            "Optimistic",
            "All parking\nconservative",
            "Fixed\nconservative",
            "Fixed +\nobservation",
            "Dispatcher",
        ]
        bottom = np.zeros(len(policies))
        for state, color in [("park", "#3478a4"), ("observe", "#e4a138"), ("stop", "#b7bbc1")]:
            counts = np.array([summary["policies"][p][state] for p in policies])
            axes[0].bar(
                np.arange(len(policies)),
                counts,
                bottom=bottom,
                label=state.capitalize(),
                color=color,
            )
            bottom += counts
        axes[0].set_xticks(np.arange(len(policies)), labels, fontsize=8)
        axes[0].set_ylabel("Selected actions")
        axes[0].set_title(
            f"Decision-only benchmark (n={summary['cases']})\nNot completed parking runs"
        )
        axes[0].legend(frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.11))
        for x in (-4.5, 2.5):
            axes[1].add_patch(Rectangle((x, 8), 2, 5, facecolor="#555555"))
        axes[1].add_patch(
            Rectangle((-1.6, 8), 3.2, 6, fill=False, edgecolor="#469467", linestyle="--")
        )
        for name, color in [("conservative", "#777777"), ("active", "#3478a4")]:
            data = json.loads((args.run_dir / (name + ".json")).read_text())
            rows = data["telemetry"]
            axes[1].plot(
                [r["x"] for r in rows],
                [r["y"] for r in rows],
                color=color,
                label=f"{name}: {'parked' if data['parked'] else 'not completed'}",
            )
            axes[1].scatter(rows[0]["x"], rows[0]["y"], color=color, s=35)
            axes[1].scatter(rows[-1]["x"], rows[-1]["y"], color=color, marker="x", s=55)
        axes[1].set_title("Closed loop: BasicSim + ideal planar scanner")
        axes[1].set_xlabel("x (m)")
        axes[1].set_ylabel("y (m)")
        axes[1].set_aspect("equal")
        axes[1].legend(frameon=False, fontsize=8)
        args.output.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output / "results.png", dpi=180)
        fig.savefig(args.output / "results.svg")
        plt.close(fig)

    if __name__ == "__main__":
        main()
