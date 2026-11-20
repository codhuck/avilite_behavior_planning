"""Illustrate measured input, finite-ray prediction and acquired endpoint evidence."""

# ruff: noqa: E402
# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import sys
    from pathlib import Path

    PLUGIN = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(PLUGIN), str(PLUGIN.parent)]
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import shapely
    from _bootstrap import load_plugin
    from avlite.c50_common.c52_world_sensor_datatypes import Lidar
    from replay_scan_views import load_task
    from shapely.geometry import box

    load_plugin()

    from avlite_coordination_demo.parking.occupancy import FusedLidarParkingOccupancy
    from avlite_coordination_demo.parking.scan import ScanVisibility

    def geometry(ax, geom, color, alpha=1):
        if geom.is_empty:
            return
        for part in geom.geoms if hasattr(geom, "geoms") else [geom]:
            if part.geom_type != "Polygon":
                continue
            x, y = part.exterior.xy
            ax.fill(x, y, color=color, alpha=alpha, linewidth=0)
            for ring in part.interiors:
                x, y = ring.xy
                ax.fill(x, y, color="white", linewidth=0)

    def main():
        p = argparse.ArgumentParser(description=__doc__)
        p.add_argument("recording", type=Path)
        p.add_argument("--scene", required=True)
        p.add_argument("--output", type=Path, required=True)
        a = p.parse_args()
        row = next(
            r
            for r in json.loads((a.recording / "results.json").read_text())
            if r["scene"]["id"] == a.scene
        )
        candidate = next(
            c for c in row["candidates"] if c["name"] == row["choices"]["task_scan"]["name"]
        )
        root = a.recording / a.scene
        task, sensors, origin, ego = load_task(root)
        model = ScanVisibility(task, sensors, body_length=ego.length, body_width=ego.width)
        clouds = []
        for i in range(9):
            with np.load(root / f"{candidate['name']}_cloud_{i}.npz") as raw:
                pose = raw["pose"]
                clouds.append(
                    Lidar(
                        points=raw["points"],
                        stamp=float(raw["stamp"]),
                        base_to_sensor=raw["base_to_sensor"],
                        sensor_name=f"lidar_{i}",
                    )
                )
        ego.x, ego.y, ego.z, ego.theta = pose
        acquired = FusedLidarParkingOccupancy(hazard_support_radius=2).observe(
            clouds, ego, box(30, -6, 70, 18), clouds[0].stamp, 0
        )
        known = shapely.intersects_xy(acquired.free, task.points[:, 0], task.points[:, 1])
        fig, axes = plt.subplots(
            1, 3, figsize=(11, 3), layout="constrained", sharex=True, sharey=True
        )
        endpoint = np.array(candidate["endpoint"])
        yaw = endpoint[2]
        rotation = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        for ax in axes:
            geometry(ax, task.occupied, "#5c6570", 0.6)
            path = np.load(root / "task_path.npy")
            ax.plot(path[:, 0], path[:, 1], color="#888888", lw=1)
            ax.scatter(*origin[:2], marker="s", color="#1f77b4", s=45)
            ax.scatter(*endpoint[:2], marker="^", color="#993f86", s=45)
            ax.set(aspect="equal", xlabel="x (m)")
            ax.grid(alpha=0.15)
            ax.spines[["top", "right"]].set_visible(False)
        axes[0].scatter(*task.points.T, c=task.weights, cmap="YlOrBr", s=12)
        axes[0].set_title("Missing measured task samples")
        for local_origin, ends in model.rays[0]:
            points = ends @ rotation + endpoint[:2]
            axes[1].scatter(*points.T, color="#993f86", s=0.4, alpha=0.3)
        axes[1].scatter(*task.points.T, color="#b47b25", s=8)
        axes[1].set_title("Predicted ground impacts, phase 0\nBefore known-obstacle masking")
        axes[2].scatter(*task.points[~known].T, color="#d1d1d1", s=12, label="Still unobserved")
        axes[2].scatter(*task.points[known].T, color="#20876a", s=15, label="Acquired free")
        axes[2].set_title("Actual endpoint LiDAR evidence")
        axes[2].legend(fontsize=8)
        lower = np.minimum(task.points.min(axis=0), origin[:2]) - 1
        upper = np.maximum(task.points.max(axis=0), endpoint[:2]) + 1
        axes[0].set(xlim=(lower[0], upper[0]), ylim=(lower[1], upper[1]), ylabel="y (m)")
        fig.suptitle(
            "Static endpoint acquisition; square = initial pose, triangle = selected view",
            fontsize=10,
        )
        a.output.mkdir(parents=True, exist_ok=True)
        fig.savefig(a.output / "scan_view_example.pdf")
        fig.savefig(a.output / "scan_view_example.png", dpi=180)

    if __name__ == "__main__":
        main()
