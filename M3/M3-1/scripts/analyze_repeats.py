#!/usr/bin/env python3
"""Summarize ten complete rounded-square trials as mean ± sample SD."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, stdev

from analyze_trial import ideal_square, local_poses, path_discrepancies, read_poses


FIELDS = (
    "truth_closure_m", "odom_closure_m", "endpoint_disagreement_m",
    "mean_path_disagreement_m", "truth_heading_error_deg", "odom_heading_error_deg",
)


def metrics_for_run(folder, truth_offset):
    metadata_path = folder / "metadata.json"
    if not metadata_path.is_file():
        raise ValueError(f"{folder}: metadata.json missing; trial may be incomplete")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if not metadata.get("complete"):
        raise ValueError(f"{folder}: trial is not marked complete")
    odom = local_poses(read_poses(folder / "odom.csv"))
    truth = local_poses(read_poses(folder / "truth.csv"), truth_offset)
    errors = path_discrepancies(odom, truth)
    if not errors:
        raise ValueError(f"{folder}: odom and truth timestamps do not overlap")
    return {
        "run": folder.name,
        "truth_closure_m": math.hypot(*truth[-1][1:3]),
        "odom_closure_m": math.hypot(*odom[-1][1:3]),
        "endpoint_disagreement_m": math.dist(odom[-1][1:3], truth[-1][1:3]),
        "mean_path_disagreement_m": mean(errors),
        "truth_heading_error_deg": math.degrees(truth[-1][3]) - 360.0,
        "odom_heading_error_deg": math.degrees(odom[-1][3]) - 360.0,
    }, odom, truth, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-dir", type=Path, required=True, help="Directory with run_01, ...")
    parser.add_argument("--out", type=Path, required=True, help="Output directory for summaries")
    parser.add_argument("--expected-runs", type=int, default=10, help="Required complete trial count")
    parser.add_argument("--truth-forward-offset", type=float, default=0.14,
                        help="Gazebo model centre ahead of rear axle in metres")
    args = parser.parse_args()
    if args.expected_runs < 2:
        parser.error("at least two runs are needed for a sample standard deviation")
    folders = sorted(p for p in args.runs_dir.glob("run_*") if p.is_dir())
    if len(folders) != args.expected_runs:
        raise ValueError(f"Expected {args.expected_runs} run directories, found {len(folders)}")
    results = [metrics_for_run(folder, args.truth_forward_offset) for folder in folders]
    settings = {(meta["side_m"], meta["radius_m"], meta["speed_mps"])
                for _, _, _, meta in results}
    if len(settings) != 1:
        raise ValueError("Trials used different side, radius, or speed settings")
    side, radius, speed = settings.pop()
    rows = [item[0] for item in results]
    summary = {
        "runs": len(rows),
        "trial_type": "consecutive_laps_without_world_reset",
        "side_m": side,
        "radius_m": radius,
        "speed_mps": speed,
        "truth_forward_offset_m": args.truth_forward_offset,
        "standard_deviation": "sample (n-1 denominator)",
        "metrics": {field: {"mean": mean(row[field] for row in rows),
                            "std": stdev(row[field] for row in rows)}
                    for field in FIELDS},
    }
    args.out.mkdir(parents=True, exist_ok=True)
    with (args.out / "per_run.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["run", *FIELDS], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    (args.out / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(8, 8))
    ideal = ideal_square(side, radius)
    axis.plot(*zip(*ideal), color="#555555", linestyle="--", label="commanded geometry")
    for index, (_, odom, truth, _) in enumerate(results):
        axis.plot([r[1] for r in odom], [r[2] for r in odom], color="#d95f02",
                  alpha=0.35, label="odom" if index == 0 else None)
        axis.plot([r[1] for r in truth], [r[2] for r in truth], color="#1b9e77",
                  alpha=0.35, label="Gazebo truth" if index == 0 else None)
    axis.scatter(0, 0, color="black", s=30, label="aligned start")
    axis.set(xlabel="x from each run start (m)", ylabel="y from each run start (m)",
             title=f"M3-1: {len(rows)} rounded-square runs, rear axle")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(args.out / "trajectories.png", dpi=160)
    plt.close(figure)
    for field in FIELDS:
        value = summary["metrics"][field]
        print(f"{field}: {value['mean']:.5f} ± {value['std']:.5f}")
    print(f"Saved {args.out / 'per_run.csv'}, {args.out / 'summary.json'}, "
          f"and {args.out / 'trajectories.png'}")


if __name__ == "__main__":
    main()
