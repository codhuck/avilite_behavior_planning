"""Read-only decision window; does not connect to a simulator or change control."""

if not __name__.startswith("avlite.plugins."):
    import argparse
    import json
    import time
    from pathlib import Path

    def format_status(data):
        def number(key, unit):
            value = data.get(key)
            return "unavailable" if value is None else f"{value:.2f} {unit}"

        return "\n".join(
            [
                f"State: {data['state']}",
                f"Reason: {data['reason'].replace('_', ' ')}",
                f"Object / space: {data.get('object') or '—'}",
                f"Planner: {data['planner']}",
                f"Speed: {number('speed_mps', 'm/s')}",
                f"Distance to rule: {number('distance_m', 'm')}",
                f"Current decision: {number('state_duration_s', 's (simulation)')}",
                f"Observation: {data['observation_source']}",
                f"Observation age: {number('observation_age_s', 's')}",
                f"Brake holds: {', '.join(data['motion_holds']) or 'none'}",
            ]
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--status", required=True, type=Path)
        parser.add_argument(
            "--once", action="store_true", help="Print one snapshot without opening a window"
        )
        args = parser.parse_args()
        if args.once:
            print(format_status(json.loads(args.status.expanduser().read_text())))
            return
        import tkinter as tk
        from tkinter import ttk

        root = tk.Tk()
        root.title("AVLite — decision status")
        text = tk.StringVar(value="Waiting for the first decision…")
        freshness = tk.StringVar()
        ttk.Label(root, textvariable=text, justify="left", padding=20).pack(anchor="w")
        ttk.Label(root, textvariable=freshness, padding=20).pack(anchor="w")

        def update():
            try:
                data = json.loads(args.status.expanduser().read_text())
                text.set(format_status(data))
                age = max(0, time.time() - data["updated_at"])
                freshness.set(
                    f"Last update {age:.1f} s ago"
                    + (" — paused or disconnected" if age > 5 else "")
                )
            except (OSError, ValueError, KeyError, TypeError) as exc:
                freshness.set(f"Status unavailable: {exc}")
            root.after(250, update)

        update()
        root.mainloop()

    if __name__ == "__main__":
        main()
