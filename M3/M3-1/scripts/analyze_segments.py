#!/usr/bin/env python3
"""Locate odometry heading bias in straight and turning stages of repeated laps."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from statistics import mean, stdev

from analyze_trial import local_poses, read_poses


def pose_at(rows, time_s):
    times = [row[0] for row in rows]
    index = bisect.bisect_left(times, time_s)
    if index == 0 or index >= len(rows):
        raise ValueError(f"Stage boundary {time_s:.3f} s is outside recorded data")
    before, after = rows[index - 1], rows[index]
    weight = (time_s - before[0]) / (after[0] - before[0])
    return tuple(before[i] + weight * (after[i] - before[i]) for i in (1, 2, 3))


def stage_rows(folder):
    metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    if not metadata.get("complete"):
        raise ValueError(f"{folder}: incomplete run")
    offset = metadata.get("truth_forward_offset_m", 0.14)
    poses = {
        "odom": local_poses(read_poses(folder / "odom.csv")),
        "truth": local_poses(read_poses(folder / "truth.csv"), offset),
    }
    straight_s = (metadata["side_m"] - 2 * metadata["radius_m"]) / metadata["speed_mps"]
    turn_s = math.pi * metadata["radius_m"] / (2 * metadata["speed_mps"])
    time_s = metadata["drive_start_sim_s"] - metadata["recorder_start_sim_s"]
    rows = []
    for edge in range(1, 5):
        for stage, duration in (("straight", straight_s), ("turn", turn_s)):
            values = {}
            for label, trajectory in poses.items():
                start = pose_at(trajectory, time_s)
                end = pose_at(trajectory, time_s + duration)
                values[label] = {
                    "heading_deg": math.degrees(end[2] - start[2]),
                    "displacement_m": math.dist(start[:2], end[:2]),
                }
            rows.append({
                "run": folder.name,
                "edge": edge,
                "stage": stage,
                "odom_heading_deg": values["odom"]["heading_deg"],
                "truth_heading_deg": values["truth"]["heading_deg"],
                "heading_bias_deg": (values["odom"]["heading_deg"]
                                     - values["truth"]["heading_deg"]),
                "odom_displacement_m": values["odom"]["displacement_m"],
                "truth_displacement_m": values["truth"]["displacement_m"],
                "displacement_bias_m": (values["odom"]["displacement_m"]
                                        - values["truth"]["displacement_m"]),
            })
            time_s += duration
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--expected-runs", type=int, default=10)
    args = parser.parse_args()
    if args.expected_runs < 2:
        parser.error("at least two runs are needed for sample standard deviation")
    folders = sorted(path for path in args.runs_dir.glob("run_*") if path.is_dir())
    if len(folders) != args.expected_runs:
        raise ValueError(f"Expected {args.expected_runs} runs, found {len(folders)}")
    rows = [row for folder in folders for row in stage_rows(folder)]
    by_run = {folder.name: [row for row in rows if row["run"] == folder.name]
              for folder in folders}
    per_run = []
    for name, stages in by_run.items():
        straights = [row for row in stages if row["stage"] == "straight"]
        turns = [row for row in stages if row["stage"] == "turn"]
        per_run.append({
            "run": name,
            "mean_straight_distance_bias_m": mean(r["displacement_bias_m"] for r in straights),
            "straight_heading_bias_deg": sum(r["heading_bias_deg"] for r in straights),
            "turn_heading_bias_deg": sum(r["heading_bias_deg"] for r in turns),
            "mean_turn_heading_bias_deg": mean(r["heading_bias_deg"] for r in turns),
        })
    metric_names = [key for key in per_run[0] if key != "run"]
    summary = {
        "runs": len(per_run),
        "standard_deviation": "sample (n-1 denominator)",
        "timing_note": "Stage boundaries use the scheduled simulation times; command delivery may lag by one 20 Hz tick.",
        "metrics": {key: {"mean": mean(row[key] for row in per_run),
                          "std": stdev(row[key] for row in per_run)}
                    for key in metric_names},
    }
    args.out.mkdir(parents=True, exist_ok=True)
    for name, content in (("per_segment.csv", rows), ("per_run_segments.csv", per_run)):
        with (args.out / name).open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(content[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(content)
    (args.out / "segment_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    for key, result in summary["metrics"].items():
        print(f"{key}: {result['mean']:.6f} ± {result['std']:.6f}")


if __name__ == "__main__":
    main()
