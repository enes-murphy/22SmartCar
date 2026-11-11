#!/usr/bin/env python3
"""Compare one rounded-square run at the rear-axle reference point."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from statistics import mean


def read_poses(path):
    with path.open(newline="", encoding="utf-8") as file:
        rows = [{key: float(value) for key, value in row.items()}
                for row in csv.DictReader(file)]
    if len(rows) < 2 or any(rows[i]["t"] <= rows[i - 1]["t"] for i in range(1, len(rows))):
        raise ValueError(f"{path}: need at least two rows with strictly increasing time")
    return rows


def local_poses(rows, forward_offset=0.0):
    # Gazebo model pose is at the body centre. Subtract half the wheelbase to
    # compare its rear axle with the controller's rear-axle odometry.
    shifted = [(r["t"], r["x"] - forward_offset * math.cos(r["theta"]),
                r["y"] - forward_offset * math.sin(r["theta"]), r["theta"])
               for r in rows]
    _, x0, y0, h0 = shifted[0]
    c, s = math.cos(h0), math.sin(h0)
    unwrapped = [0.0]
    for i in range(1, len(shifted)):
        delta = shifted[i][3] - shifted[i - 1][3]
        unwrapped.append(unwrapped[-1] + math.atan2(math.sin(delta), math.cos(delta)))
    return [(t, c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0), unwrapped[i])
            for i, (t, x, y, _) in enumerate(shifted)]


def ideal_square(side, radius):
    straight = side - 2 * radius
    if straight <= 0:
        raise ValueError("side must exceed twice the turn radius")
    points = [(0.0, 0.0)]
    x = y = heading = 0.0
    for _ in range(4):
        for j in range(1, 51):
            d = straight * j / 50
            points.append((x + d * math.cos(heading), y + d * math.sin(heading)))
        x, y = points[-1]
        x0, y0, h0 = x, y, heading
        for j in range(1, 51):
            h = h0 + math.pi * j / 100
            points.append((x0 + radius * (math.sin(h) - math.sin(h0)),
                           y0 + radius * (math.cos(h0) - math.cos(h))))
        x, y = points[-1]
        heading += math.pi / 2
    return points


def path_discrepancies(odom, truth):
    truth_times = [row[0] for row in truth]
    errors = []
    for t, x, y, _ in odom:
        index = bisect.bisect_left(truth_times, t)
        if index == 0 or index == len(truth):
            continue
        t0, x0, y0, _ = truth[index - 1]
        t1, x1, y1, _ = truth[index]
        weight = (t - t0) / (t1 - t0)
        errors.append(math.hypot(x - (x0 + weight * (x1 - x0)),
                                 y - (y0 + weight * (y1 - y0))))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Directory with odom.csv and truth.csv")
    parser.add_argument("--output-prefix", type=Path, required=True, help="Path prefix for JSON and PNG")
    parser.add_argument("--side", type=float, default=2.0, help="Outer width in metres")
    parser.add_argument("--radius", type=float, default=0.65, help="Commanded turn radius in metres")
    parser.add_argument("--truth-forward-offset", type=float, default=0.14,
                        help="Gazebo model centre ahead of rear axle in metres")
    args = parser.parse_args()
    if args.truth_forward_offset < 0:
        parser.error("--truth-forward-offset must be nonnegative")
    ideal = ideal_square(args.side, args.radius)
    odom = local_poses(read_poses(args.input / "odom.csv"))
    truth = local_poses(read_poses(args.input / "truth.csv"), args.truth_forward_offset)
    errors = path_discrepancies(odom, truth)
    if not errors:
        raise ValueError("The two CSV files have no overlapping timestamps")
    metrics = {
        "truth_reference": f"Gazebo model pose shifted back {args.truth_forward_offset:.3f} m to rear axle",
        "odom_closure_m": math.hypot(odom[-1][1], odom[-1][2]),
        "truth_closure_m": math.hypot(truth[-1][1], truth[-1][2]),
        "endpoint_disagreement_m": math.dist(odom[-1][1:3], truth[-1][1:3]),
        "odom_heading_change_deg": math.degrees(odom[-1][3]),
        "truth_heading_change_deg": math.degrees(truth[-1][3]),
        "mean_path_disagreement_m": mean(errors),
        "max_path_disagreement_m": max(errors),
        "odom_rows": len(odom),
        "truth_rows": len(truth),
    }
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path = args.output_prefix.with_name(args.output_prefix.name + "_analysis.json")
    png_path = args.output_prefix.with_name(args.output_prefix.name + "_trajectory.png")
    json_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(7, 7))
    axis.plot(*zip(*ideal), label="commanded geometry", color="#666666", linestyle="--")
    for rows, label, color in ((odom, "odom rear axle", "#d95f02"),
                               (truth, "Gazebo rear axle", "#1b9e77")):
        axis.plot([r[1] for r in rows], [r[2] for r in rows], label=label, color=color)
        axis.scatter(rows[-1][1], rows[-1][2], color=color, marker="x", s=70)
    axis.scatter(0, 0, color="black", s=35, label="aligned start")
    axis.set(xlabel="x from start (m)", ylabel="y from start (m)",
             title="M3-1 rounded square: same rear-axle reference")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(png_path, dpi=160)
    plt.close(figure)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"Saved {json_path} and {png_path}")


if __name__ == "__main__":
    main()
