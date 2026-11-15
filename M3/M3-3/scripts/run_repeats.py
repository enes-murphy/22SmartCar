#!/usr/bin/env python3
"""Repeat the same Nav2 route from a fresh isolated simulation each time."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import statistics
import subprocess
import sys
import time


def stop_group(process, timeout=20):
    if process is None or process.poll() is not None:
        return
    for sig, limit in ((signal.SIGINT, timeout), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=limit)
            return
        except subprocess.TimeoutExpired:
            pass


def stop_launch(process, env):
    if process is None or process.poll() is not None:
        return
    # Ask lifecycle managers to stop their nodes before stopping Gazebo.
    for manager in ("navigation", "localization"):
        try:
            subprocess.run(
                ["ros2", "service", "call", f"/lifecycle_manager_{manager}/manage_nodes",
                 "nav2_msgs/srv/ManageLifecycleNodes", "{command: 4}"],
                env=env, capture_output=True, text=True, timeout=25, check=False,
            )
        except subprocess.TimeoutExpired:
            pass
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        stop_group(process, timeout=5)


def wait_for_navigation(env, launch, timeout=60):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if launch.poll() is not None:
            raise RuntimeError(f"launch exited early with code {launch.returncode}")
        result = subprocess.run(
            ["ros2", "lifecycle", "get", "/controller_server"],
            env=env, capture_output=True, text=True, timeout=5, check=False,
        )
        if "active [3]" in result.stdout:
            result = subprocess.run(
                ["ros2", "lifecycle", "get", "/amcl"],
                env=env, capture_output=True, text=True, timeout=5, check=False,
            )
            if "active [3]" in result.stdout:
                return
        time.sleep(1)
    raise TimeoutError("Nav2 and AMCL did not become active")


def trajectory_stats(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    points = [(float(row["x"]), float(row["y"])) for row in rows]
    return {
        "rows": len(points),
        "path_length_m": round(sum(math.dist(a, b) for a, b in zip(points, points[1:])), 3),
        "start": dict(zip(("x", "y"), points[0])) if points else None,
        "end": dict(zip(("x", "y"), points[-1])) if points else None,
    }


def run_one(index, out_dir, ros_domain, gazebo_port):
    run_dir = out_dir / f"run_{index:02d}"
    run_dir.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(ros_domain),
               GAZEBO_MASTER_URI=f"http://127.0.0.1:{gazebo_port}")
    launch = recorder = trial = None
    trial_code = None
    failure = None
    try:
        with (run_dir / "launch.log").open("w") as launch_log:
            launch = subprocess.Popen(
                ["ros2", "launch", "smart_car_navigation", "localization.launch.py",
                 "gui:=false", "rviz:=false", "navigation:=true"],
                env=env, stdout=launch_log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            wait_for_navigation(env, launch)
            with (run_dir / "recorder.log").open("w") as recorder_log:
                recorder = subprocess.Popen(
                    [sys.executable, "../M3-1/scripts/record_traj.py", "--duration", "180",
                     "--out", str(run_dir)],
                    env=env, stdout=recorder_log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                time.sleep(2)
                with (run_dir / "trial.log").open("w") as trial_log:
                    trial = subprocess.Popen(
                        [sys.executable, "scripts/run_nav_trial.py", "--timeout", "180"],
                        env=env, stdout=trial_log, stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    try:
                        trial_code = trial.wait(timeout=195)
                    except subprocess.TimeoutExpired:
                        failure = "Trial process exceeded 195 wall-clock seconds"
                    finally:
                        stop_group(trial, timeout=5)
                stop_group(recorder, timeout=5)
            stop_launch(launch, env)
    except Exception as exc:
        failure = str(exc)
    finally:
        stop_group(trial, timeout=5)
        stop_group(recorder, timeout=5)
        stop_launch(launch, env)

    log_path = run_dir / "trial.log"
    log = log_path.read_text() if log_path.exists() else ""
    success = trial_code == 0 and "SUCCEEDED: Nav2 reached" in log
    elapsed_match = re.search(r"elapsed=([\d.]+) s", log)
    error_match = re.search(r"goal error=([\d.]+) m", log)
    summary = {
        "run": index,
        "success": success,
        "trial_exit_code": trial_code,
        "action_elapsed_wall_s": float(elapsed_match.group(1)) if elapsed_match else None,
        "goal_error_map_base_link_m": float(error_match.group(1)) if error_match else None,
        "failure_reason": failure or (None if success else "See trial.log and launch.log"),
    }
    for name in ("odom", "truth"):
        path = run_dir / f"{name}.csv"
        if path.exists():
            summary[name] = trajectory_stats(path)
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Run {index}: {'SUCCEEDED' if success else 'FAILED'}; "
          f"time={summary['action_elapsed_wall_s']} s; "
          f"goal error={summary['goal_error_map_base_link_m']} m", flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--out", type=Path, default=Path("results/nav_repeats_tuned"))
    parser.add_argument("--first-domain", type=int, default=70)
    args = parser.parse_args()
    if args.runs < 1 or not (1 <= args.first_domain <= 230 - args.runs):
        parser.error("invalid run count or ROS domain range")
    if shutil.which("ros2") is None:
        parser.error("source the ROS Humble and three workspace setup files first")
    args.out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile("src/smart_car_navigation/config/navigation.yaml",
                    args.out / "navigation.yaml")
    results = []
    for index in range(1, args.runs + 1):
        domain = args.first_domain + index - 1
        print(f"Run {index}/{args.runs}: ROS domain {domain}", flush=True)
        results.append(run_one(index, args.out, domain, 11370 + domain))
    successes = [row for row in results if row["success"]]
    times = [row["action_elapsed_wall_s"] for row in successes]
    report = {
        "runs": args.runs,
        "successes": len(successes),
        "success_rate": len(successes) / args.runs,
        "mean_success_time_wall_s": round(statistics.mean(times), 2) if times else None,
        "std_success_time_wall_s": round(statistics.stdev(times), 2) if len(times) > 1 else None,
        "failure_reasons": [row["failure_reason"] for row in results if not row["success"]],
    }
    (args.out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    return 0 if len(successes) == args.runs else 1


if __name__ == "__main__":
    sys.exit(main())
