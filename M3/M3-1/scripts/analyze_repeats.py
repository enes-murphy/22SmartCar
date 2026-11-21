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
    "truth_angle_closure_deg", "odom_angle_closure_deg",
    "endpoint_heading_disagreement_deg",
)


def angle_closure_deg(angle_rad):
    """Absolute start-to-end heading difference, wrapped to (-pi, pi]."""
    return abs(math.degrees(math.atan2(math.sin(angle_rad), math.cos(angle_rad))))


def metrics_for_run(folder, truth_offset):
    metadata_path = folder / "metadata.json"
    if not metadata_path.is_file():
        raise ValueError(f"{folder}: metadata.json missing; trial may be incomplete")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if not metadata.get("complete"):
        raise ValueError(f"{folder}: trial is not marked complete")
    offset = (truth_offset if truth_offset is not None
              else metadata.get("truth_forward_offset_m", 0.14))
    odom = local_poses(read_poses(folder / "odom.csv"))
    truth = local_poses(read_poses(folder / "truth.csv"), offset)
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
        "truth_angle_closure_deg": angle_closure_deg(truth[-1][3]),
        "odom_angle_closure_deg": angle_closure_deg(odom[-1][3]),
        "endpoint_heading_disagreement_deg": angle_closure_deg(
            odom[-1][3] - truth[-1][3]),
    }, odom, truth, metadata, offset


def write_exam_table(path, rows, summary, logs_dir):
    lines = [
        "# M3-1 十次闭环误差统计",
        "",
        f"原始数据：`{logs_dir}`；共 {len(rows)} 圈；每圈分别对齐里程计与真值的起点、朝向，并比较同一后轴地面点。",
        "",
        "| 圈次 | 真值位置闭环 (m) | 真值角度闭环 (°) | 里程计位置闭环 (m) | 里程计角度闭环 (°) | 两路终点差 (m) | 两路终点航向差 (°) |",
        "| :-- | --: | --: | --: | --: | --: | --: |",
    ]
    columns = (
        "truth_closure_m", "truth_angle_closure_deg",
        "odom_closure_m", "odom_angle_closure_deg",
        "endpoint_disagreement_m", "endpoint_heading_disagreement_deg",
    )
    for row in rows:
        cells = [row["run"]]
        for key in columns:
            cells.append(f"{row[key]:.5f}" if key.endswith("_m") else f"{row[key]:.3f}")
        lines.append("| " + " | ".join(cells) + " |")
    lines += ["", "| 指标 | 10 圈均值 ± 样本标准差 |", "| :-- | --: |"]
    labels = {
        "truth_closure_m": "真值位置闭环 (m)",
        "truth_angle_closure_deg": "真值角度闭环 (°)",
        "odom_closure_m": "里程计位置闭环 (m)",
        "odom_angle_closure_deg": "里程计角度闭环 (°)",
        "endpoint_disagreement_m": "两路终点位置差 (m，主要漂移指标)",
        "endpoint_heading_disagreement_deg": "两路终点航向差 (°)",
    }
    for key in columns:
        metric = summary["metrics"][key]
        fmt = ".5f" if key.endswith("_m") else ".3f"
        lines.append(f"| {labels[key]} | {metric['mean']:{fmt}} ± {metric['std']:{fmt}} |")
    lines += [
        "",
        "位置闭环是各轨迹末点距自身起点的欧氏距离；角度闭环是 `abs(normalize(theta_end - theta_start))`。",
        "两路终点差先统一参考点、起点和初始朝向，然后比较对应终点；它与车辆真实闭环距离不是同一指标。",
        "标准差是对每圈一个结果计算的样本标准差，分母为 `n-1`，不是对 CSV 每一帧计算。",
        "真值只用于离线对照，没有反馈到驾驶程序或里程计。十圈在同一 Gazebo 会话中连续进行，没有逐圈重置世界。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def render_exam_trajectory(path, first_run, first_metrics, side, radius):
    """One readable red/green comparison with start, ends, errors and 2 m width."""
    import matplotlib.pyplot as plt

    _, odom, truth, _, _ = first_run
    figure, axis = plt.subplots(figsize=(8.2, 7.6))
    ideal = ideal_square(side, radius)
    axis.plot(*zip(*ideal), color="#666666", linestyle="--", linewidth=1.6,
              label="commanded rounded square")
    axis.plot([r[1] for r in odom], [r[2] for r in odom], color="#c62828",
              linewidth=2, label="/odom estimate")
    axis.plot([r[1] for r in truth], [r[2] for r in truth], color="#16804a",
              linewidth=2, label="Gazebo truth")
    axis.scatter(0, 0, color="black", marker="o", s=75, zorder=6, label="aligned start")
    axis.scatter(truth[-1][1], truth[-1][2], color="#16804a", marker="^", s=105,
                 zorder=7, label="truth end")
    axis.scatter(odom[-1][1], odom[-1][2], color="#c62828", marker="X", s=105,
                 zorder=7, label="odom end")
    axis.annotate("truth end", xy=truth[-1][1:3], xytext=(-80, 22),
                  textcoords="offset points", color="#11663b",
                  arrowprops={"arrowstyle": "-", "color": "#11663b"})
    axis.annotate("odom end", xy=odom[-1][1:3], xytext=(22, -26),
                  textcoords="offset points", color="#a52323",
                  arrowprops={"arrowstyle": "-", "color": "#a52323"})
    # The ideal path spans exactly 'side' metres in x, from -radius to side-radius.
    axis.annotate("", xy=(-radius, -0.28), xytext=(side - radius, -0.28),
                  arrowprops={"arrowstyle": "<->", "color": "#303030", "lw": 1.4})
    axis.text((side - 2 * radius) / 2, -0.37, f"{side:g} m outer width",
              ha="center", va="top", color="#303030")
    axis.set(xlabel="x from aligned start (m)", ylabel="y from aligned start (m)",
             title=f"M3-1 run 01: rounded square, same rear-axle point")
    axis.set_xlim(-radius - 0.55, side - radius + 0.55)
    axis.set_ylim(-0.65, side + 0.35)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend(loc="upper right", fontsize=8)
    figure.text(
        0.5, 0.02,
        "Closure: Gazebo {:.3f} m / {:.2f}°, odom {:.3f} m / {:.2f}°; "
        "endpoint separation {:.3f} m".format(
            first_metrics["truth_closure_m"], first_metrics["truth_angle_closure_deg"],
            first_metrics["odom_closure_m"], first_metrics["odom_angle_closure_deg"],
            first_metrics["endpoint_disagreement_m"]),
        ha="center", fontsize=9,
    )
    figure.tight_layout(rect=(0, 0.055, 1, 1))
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-dir", type=Path, required=True, help="Directory with run_01, ...")
    parser.add_argument("--out", type=Path, required=True, help="Output directory for summaries")
    parser.add_argument("--expected-runs", type=int, default=10, help="Required complete trial count")
    parser.add_argument("--truth-forward-offset", type=float,
                        help="Override model-pose offset from each run's metadata (metres)")
    args = parser.parse_args()
    if args.expected_runs < 2:
        parser.error("at least two runs are needed for a sample standard deviation")
    if args.truth_forward_offset is not None and args.truth_forward_offset < 0:
        parser.error("--truth-forward-offset must be nonnegative")
    folders = sorted(p for p in args.runs_dir.glob("run_*") if p.is_dir())
    if len(folders) != args.expected_runs:
        raise ValueError(f"Expected {args.expected_runs} run directories, found {len(folders)}")
    results = [metrics_for_run(folder, args.truth_forward_offset) for folder in folders]
    settings = {(meta["side_m"], meta["radius_m"], meta["speed_mps"], offset)
                for _, _, _, meta, offset in results}
    if len(settings) != 1:
        raise ValueError("Trials used different side, radius, speed, or model-pose offset")
    side, radius, speed, offset = settings.pop()
    rows = [item[0] for item in results]
    summary = {
        "runs": len(rows),
        "trial_type": "consecutive_laps_without_world_reset",
        "side_m": side,
        "radius_m": radius,
        "speed_mps": speed,
        "truth_forward_offset_m": offset,
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
    write_exam_table(args.out / "closed_loop_stats.md", rows, summary, args.runs_dir)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(8, 8))
    ideal = ideal_square(side, radius)
    axis.plot(*zip(*ideal), color="#555555", linestyle="--", label="commanded geometry")
    for index, (_, odom, truth, _, _) in enumerate(results):
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
    render_exam_trajectory(args.out / "trajectory_compare.png", results[0], rows[0], side, radius)
    for field in FIELDS:
        value = summary["metrics"][field]
        print(f"{field}: {value['mean']:.5f} ± {value['std']:.5f}")
    print(f"Saved {args.out / 'per_run.csv'}, {args.out / 'summary.json'}, "
          f"{args.out / 'closed_loop_stats.md'}, {args.out / 'trajectories.png'}, "
          f"and {args.out / 'trajectory_compare.png'}")


if __name__ == "__main__":
    main()
