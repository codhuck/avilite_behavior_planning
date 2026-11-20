"""Publication figures and discrete replay from recorded CARLA measurements."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import hashlib
    import json
    from pathlib import Path

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FFMpegWriter
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, PathPatch
    from matplotlib.path import Path as MplPath
    from shapely.geometry import shape

    def polygons(ax, geometry, color, alpha=1, edge="none", zorder=1):
        """Draw polygon interiors with holes preserved by ring orientation."""
        from shapely.geometry.polygon import orient

        if geometry.is_empty:
            return
        if geometry.geom_type != "Polygon":
            for part in getattr(geometry, "geoms", []):
                polygons(ax, part, color, alpha, edge, zorder)
            return
        geometry = orient(geometry)
        vertices, codes = [], []
        for ring in [geometry.exterior, *geometry.interiors]:
            coords = np.asarray(ring.coords)
            vertices.extend(coords)
            codes.extend(
                [MplPath.MOVETO] + [MplPath.LINETO] * (len(coords) - 2) + [MplPath.CLOSEPOLY]
            )
        ax.add_patch(
            PathPatch(
                MplPath(vertices, codes),
                facecolor=color,
                edgecolor=edge,
                alpha=alpha,
                linewidth=0.8,
                zorder=zorder,
            )
        )

    def scene(ax, geometry, telemetry, index, bounds):
        ax.set_facecolor("#eceff3")
        polygons(ax, shape(geometry["free"]), "#b5dfce")
        polygons(ax, shape(geometry["occupied"]), "#cf735d", 0.85)
        for obstacle in geometry["obstacles"]:
            polygons(ax, shape(obstacle), "none", edge="#414b59", zorder=4)
        polygons(ax, shape(geometry["bay"]), "none", edge="#2274a5", zorder=5)
        polygons(ax, shape(geometry["ego"]), "#184c70", zorder=6)
        ax.plot(
            [r["x"] for r in telemetry[: index + 1]],
            [r["y"] for r in telemetry[: index + 1]],
            color="#153b60",
            lw=1.4,
            zorder=5,
        )
        cert = telemetry[index].get("certificate")
        if cert and cert.get("endpoint"):
            ax.scatter(
                *cert["endpoint"][:2],
                marker="*",
                s=125,
                color="#b37400",
                edgecolors="white",
                linewidths=0.5,
                zorder=7,
            )
        ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel="World x (m)", ylabel="World y (m)")
        ax.set_aspect("equal")

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument(
            "--recording", type=Path, default=Path("logs/parking_ablation_v12_complete")
        )
        parser.add_argument("--output", type=Path, default=Path("papers/parking_visuals"))
        parser.add_argument("--video", action="store_true")
        args = parser.parse_args()
        args.output.mkdir(parents=True, exist_ok=True)
        plt.rcParams.update(
            {
                "font.size": 10,
                "axes.spines.top": False,
                "axes.spines.right": False,
                "pdf.fonttype": 42,
                "savefig.facecolor": "white",
            }
        )
        manifest = {
            "scope": "Recorded CARLA measurements; evaluator outlines are not planner inputs. No new trials.",
            "video": "Discrete recorded geometry snapshots, 5 simulated seconds per frame, 6 fps; no interpolation.",
            "sources": {},
        }

        def read(path):
            manifest["sources"][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            return json.loads(path.read_text())

        for number, title in enumerate(
            ["Forward perpendicular", "Reverse perpendicular", "Parallel"]
        ):
            directory = args.recording / f"paired_{number}_full"
            result = read(directory / "results.json")
            if isinstance(result, list):
                result = result[0]
            telemetry = result["telemetry"]
            prefix = result["type"] + "_0"
            files = sorted(
                directory.glob(prefix + "_geometry_*.json"),
                key=lambda p: int(p.stem.rsplit("_", 1)[1]),
            )
            geometries = [(int(p.stem.rsplit("_", 1)[1]), read(p)) for p in files]
            all_bounds = [
                shape(o).bounds
                for _, g in geometries
                for o in [g["ego"], g["bay"], *g["obstacles"]]
            ]
            b = np.asarray(all_bounds)
            bounds = (b[:, 0].min() - 2, b[:, 2].max() + 2, b[:, 1].min() - 2, b[:, 3].max() + 2)
            t = np.array([r["time"] for r in telemetry])
            fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
            for ax, step in zip(axes[0], [0, 200]):
                g = dict(geometries)[step]
                scene(ax, g, telemetry, step, bounds)
                ax.set_title(f"{title} | t = {t[step]:.1f} s", loc="left", fontweight="bold")
            axes[0, 0].legend(
                handles=[
                    Patch(color="#b5dfce", label="Observed free"),
                    Patch(color="#cf735d", label="Observed occupied"),
                    Patch(color="#184c70", label="Ego"),
                    Patch(color="#eceff3", label="Unobserved / unsupported"),
                    Line2D(
                        [],
                        [],
                        marker="*",
                        color="#b37400",
                        linestyle="none",
                        label="Released endpoint",
                    ),
                    Patch(facecolor="none", edgecolor="#414b59", label="Evaluator outlines"),
                ],
                fontsize=8,
                loc="upper left",
            )
            ax = axes[1, 0]
            ax.plot(
                t, [abs(r["speed"]) for r in telemetry], label="Measured speed", color="#184c70"
            )
            ax.plot(
                t,
                [(r.get("certificate") or {}).get("speed_bound", np.nan) for r in telemetry],
                label="Certified speed bound",
                color="#b37400",
                lw=1,
            )
            ax.set(
                xlabel="Simulation time (s)",
                ylabel="Speed (m/s)",
                title="Executed speed and decision bound",
            )
            ax.legend(fontsize=9)
            ax = axes[1, 1]
            ax.plot(
                t,
                [(r.get("certificate") or {}).get("released_length", np.nan) for r in telemetry],
                color="#277c69",
            )
            ax.set(
                xlabel="Simulation time (s)",
                ylabel="Released length (m)",
                title="How much motion is authorized",
            )
            for ax in axes[1]:
                ax.grid(alpha=0.18)
            fig.suptitle(
                "Sensor-supported parking | recorded CARLA trial", fontsize=17, fontweight="bold"
            )
            fig.savefig(args.output / f"{prefix}_evidence.pdf")
            fig.savefig(args.output / f"{prefix}_evidence.png", dpi=180)
            plt.close(fig)
            # Transform each recorded sensor cloud using its acquisition pose/calibration.
            from compare_parking_evidence import load_frame

            ego, clouds, _, hashes = load_frame(
                directory, result["type"], 200, result["vehicle_geometry"]
            )
            manifest["sources"].update(hashes)
            points = np.concatenate([cloud.to_map(cloud.points, ego)[:, :3] for cloud in clouds])
            mask = (
                (points[:, 0] >= bounds[0])
                & (points[:, 0] <= bounds[1])
                & (points[:, 1] >= bounds[2])
                & (points[:, 1] <= bounds[3])
                & (points[:, 2] > 0.15)
                & (points[:, 2] < 3)
            )
            selected = points[mask][::3]
            fig = plt.figure(figsize=(10, 7), layout="constrained")
            ax = fig.add_subplot(projection="3d")
            dots = ax.scatter(
                *selected.T,
                c=selected[:, 2],
                cmap="viridis",
                s=2,
                alpha=0.8,
                rasterized=True,
                vmin=0,
                vmax=2.5,
            )
            ax.set(
                xlabel="World x (m)",
                ylabel="World y (m)",
                zlabel="",
                xlim=bounds[:2],
                ylim=bounds[2:],
                zlim=(0, 3),
            )
            ax.set_box_aspect((bounds[1] - bounds[0], bounds[3] - bounds[2], 6))
            ax.view_init(elev=40, azim=-65)
            fig.colorbar(dots, ax=ax, shrink=0.5, pad=0.12, label="Return height (m)")
            fig.suptitle(f"{title} | recorded CARLA LiDAR, t = 20 s", fontsize=16)
            ax.set_title(
                "Nine synchronized devices · ground omitted · every third above-ground return shown",
                fontsize=9,
            )
            fig.savefig(args.output / f"{prefix}_lidar.png", dpi=180)
            fig.savefig(args.output / f"{prefix}_lidar.pdf")
            plt.close(fig)
            if args.video:
                fig, (ax, timeline) = plt.subplots(
                    1,
                    2,
                    figsize=(12.8, 7.2),
                    gridspec_kw={"width_ratios": [1.4, 1]},
                    layout="constrained",
                )
                writer = FFMpegWriter(fps=6, codec="libx264", bitrate=4500)
                with writer.saving(fig, str(args.output / f"{prefix}_replay.mp4"), 100):
                    for step, g in geometries:
                        ax.clear()
                        timeline.clear()
                        scene(ax, g, telemetry, step, bounds)
                        row = telemetry[step]
                        cert = row.get("certificate") or {}
                        ax.set_title(title, loc="left", fontweight="bold")
                        timeline.plot(t, [abs(r["speed"]) for r in telemetry], color="#cbd3dc")
                        timeline.plot(
                            t[: step + 1],
                            [abs(r["speed"]) for r in telemetry[: step + 1]],
                            color="#184c70",
                        )
                        timeline.axvline(t[step], color="#b37400")
                        timeline.set(
                            xlabel="Simulation time (s)", ylabel="Speed (m/s)", ylim=(0, 0.55)
                        )
                        timeline.set_title(
                            f"t = {row['time']:.1f} s\n{row['state']}\nReason: {row['reason']}\nReleased: {cert.get('released_length', 0):.2f} m",
                            loc="left",
                        )
                        fig.suptitle(
                            "Recorded CARLA replay | discrete 5 s snapshots | 30× simulation time",
                            fontsize=15,
                        )
                        writer.grab_frame()
                plt.close(fig)
            print(title, "complete", flush=True)
        (args.output / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")

    if __name__ == "__main__":
        main()
