"""Report complete paired policy trials without dropping refusals or invalid actors."""

# Stock AVLite recursively imports Python files to discover strategies.
# CLI tools must not import optional simulators or mutate sys.path in that pass.
if not __name__.startswith("avlite.plugins."):
    import argparse
    import csv
    import json
    from collections import defaultdict
    from pathlib import Path

    import numpy as np
    from parking_ablation import POLICIES
    from report_sensor_parking import summarize

    def completion(row):
        """Parking endpoint; no mandatory stop for a nonconflicting pedestrian."""
        return bool(
            row["final_state"] == "PARKED"
            and row["position_error"] < 0.2
            and row["heading_error"] < 5
            and not row["collisions"]
            and not row["body_violations"]
            and not row.get("scenario_error")
            and (
                not row["scenario"].get("pedestrian")
                or (
                    row.get("pedestrian_encounter_valid")
                    and row.get("pedestrian_crossed")
                    and row.get("maximum_pedestrian_overlap", 0) == 0
                )
            )
        )

    def analyze(protocol, results):
        planned = {s["id"]: s for s in protocol["suite"]}
        if len(planned) != len(protocol["suite"]):
            raise ValueError("Duplicate planned case")
        grouped = defaultdict(dict)
        seen = set()
        for raw in results:
            row = summarize(raw)
            proximity = [t for t in raw["telemetry"] if t.get("pedestrian_clearance") is not None]
            closest = min(proximity, key=lambda t: t["pedestrian_clearance"], default=None)
            row["minimum_clearance_context"] = (
                {
                    key: closest.get(key)
                    for key in (
                        "time",
                        "speed",
                        "state",
                        "pedestrian_clearance",
                        "pedestrian_crossed",
                        "pedestrian_paused",
                    )
                }
                if closest is not None
                else None
            )
            row["minimum_pedestrian_clearance_while_ego_moving"] = min(
                (t["pedestrian_clearance"] for t in proximity if abs(t["speed"]) > 0.03),
                default=None,
            )
            scene = row["scenario"]
            if scene["id"] in seen or scene != planned.get(scene["id"]):
                raise ValueError("Duplicate result or protocol mismatch")
            seen.add(scene["id"])
            row["parking_completed"] = completion(row)
            pair, policy = scene["pair_id"], scene["ablation_policy"]
            if policy in grouped[pair]:
                raise ValueError("Duplicate policy within a pair")
            grouped[pair][policy] = row
        if seen != set(planned):
            raise ValueError(f"Incomplete suite: {len(seen)}/{len(planned)}; no final comparison")
        policy_sets = [
            {s["ablation_policy"] for s in protocol["suite"] if s["pair_id"] == pair}
            for pair in grouped
        ]
        if not policy_sets or any(p != policy_sets[0] for p in policy_sets):
            raise ValueError("Unbalanced policy coverage")
        summaries, differences = {}, []
        for pair, group in grouped.items():
            reference = group["full"]
            scenario = {
                k: v for k, v in reference["scenario"].items() if k not in ("id", "ablation_policy")
            }
            for policy, row in group.items():
                actual = {
                    k: v for k, v in row["scenario"].items() if k not in ("id", "ablation_policy")
                }
                if scenario != actual:
                    raise ValueError("Policies do not share the same exogenous scenario")
                if policy != "full":
                    differences.append(
                        dict(
                            pair=pair,
                            alternative=policy,
                            completion_difference=int(reference["parking_completed"])
                            - int(row["parking_completed"]),
                            time_difference_full_minus_alternative=reference["simulated_seconds"]
                            - row["simulated_seconds"]
                            if reference["parking_completed"] and row["parking_completed"]
                            else None,
                            max_overlap_difference=reference["max_false_free_box_overlap_m2"]
                            - row["max_false_free_box_overlap_m2"],
                            note="Closed-loop paths differ; perception exposure is not matched after divergence",
                        )
                    )
        for policy in sorted(policy_sets[0]):
            rows = [g[policy] for g in grouped.values()]
            successes = [r for r in rows if r["parking_completed"]]
            clearances = [
                r["minimum_pedestrian_clearance"]
                for r in rows
                if r.get("minimum_pedestrian_clearance") is not None
            ]
            summaries[policy] = dict(
                trials=len(rows),
                completed=len(successes),
                contact_trials=sum(bool(r["collisions"]) for r in rows),
                body_violation_trials=sum(bool(r["body_violations"]) for r in rows),
                invalid_encounters=sum(r.get("pedestrian_encounter_valid") is False for r in rows),
                incomplete_crossings=sum(
                    bool(r["scenario"].get("pedestrian")) and not r.get("pedestrian_crossed")
                    for r in rows
                ),
                success_time_median=float(np.median([r["simulated_seconds"] for r in successes]))
                if successes
                else None,
                max_false_free_overlap=max(r["max_false_free_box_overlap_m2"] for r in rows),
                minimum_pedestrian_clearance=min(clearances) if clearances else None,
                wall_seconds=sum(r["wall_seconds"] for r in rows),
            )
        return dict(
            scope="Paired closed-loop development ablation; three fixed geometries, not population inference",
            suite_complete=True,
            total=len(seen),
            policies=summaries,
            paired_differences=differences,
            trials=[r for g in grouped.values() for r in g.values()],
            limitations=[
                "One run per policy and geometry; no statistical superiority claimed",
                "Success times condition on completion; refusals are not fast parking",
                "Original strict passed field retained separately from parking_completed",
                "Known flat pad, exact localization, scripted fixed walker, modest noise",
            ],
        )

    def main():
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("recording", type=Path)
        parser.add_argument("--output", type=Path, required=True)
        args = parser.parse_args()
        results = json.loads((args.recording / "results.json").read_text())
        execution_errors = []
        for row in results:
            policy = POLICIES[row["scenario"]["ablation_policy"]]
            expected_sources = 1 if policy.roof_only else 9
            errors = []
            if row.get("policy_configuration") != policy.metadata():
                errors.append("policy_metadata_mismatch")
            if any(len(t.get("sources", [])) != expected_sources for t in row["telemetry"]):
                errors.append("unexpected_sensor_count")
            if not policy.allow_observation and any(
                t["state"] == "PARKING_OBSERVE" for t in row["telemetry"]
            ):
                errors.append("disabled_observation_executed")
            if errors:
                execution_errors.append(dict(case=row["scenario"]["id"], errors=errors))
        report = analyze(json.loads((args.recording / "protocol.json").read_text()), results)
        report["policy_execution_checks_passed"] = not execution_errors
        report["policy_execution_errors"] = execution_errors
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        export = []
        for row in report["trials"]:
            context = row.get("minimum_clearance_context") or {}
            export.append(
                dict(
                    case=row["scenario"]["id"],
                    policy=row["scenario"]["ablation_policy"],
                    parking_completed=row["parking_completed"],
                    original_strict_passed=row["passed"],
                    final_state=row["final_state"],
                    simulation_time_s=row["simulated_seconds"],
                    wall_time_s=row["wall_seconds"],
                    position_error_m=row["position_error"],
                    heading_error_deg=row["heading_error"],
                    collision_events=row["collisions"],
                    body_violation_samples=row["body_violations"],
                    minimum_pedestrian_clearance_m=row.get("minimum_pedestrian_clearance"),
                    ego_speed_at_minimum_clearance_mps=context.get("speed"),
                    pedestrian_crossed=row.get("pedestrian_crossed"),
                    observation_frames=row["state_frames"].get("PARKING_OBSERVE", 0),
                    max_false_free_overlap_m2=row["max_false_free_box_overlap_m2"],
                )
            )
        with (args.output / "trials.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(export[0]))
            writer.writeheader()
            writer.writerows(export)
        print(json.dumps(report["policies"], indent=2))

    if __name__ == "__main__":
        main()
