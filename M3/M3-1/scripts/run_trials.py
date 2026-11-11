#!/usr/bin/env python3
"""Record separate, consecutive M3-1 rounded-square trials in one Gazebo session."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from drive_square import SquareDriver, make_stages
from record_traj import TrajectoryRecorder


def sim_seconds(node):
    return node.get_clock().now().nanoseconds / 1e9


def spin_until(executor, condition, timeout_wall=300.0):
    deadline = time.monotonic() + timeout_wall
    while not condition():
        if not rclpy.ok():
            raise InterruptedError("ROS stopped before the trial completed")
        if time.monotonic() > deadline:
            raise TimeoutError("Timed out waiting for Gazebo or the controller")
        executor.spin_once(timeout_sec=0.2)


def run_one(executor, folder, side, radius, speed, model):
    recorder = TrajectoryRecorder(folder, model)
    driver = None
    executor.add_node(recorder)
    wall_start = time.monotonic()
    try:
        # Let both topics arrive and record a one-second stationary baseline.
        spin_until(executor, lambda: recorder.counts["odom"] >= 5
                   and recorder.counts["truth"] >= 2 and sim_seconds(recorder) > 0,
                   timeout_wall=45.0)
        baseline_end = sim_seconds(recorder) + 1.0
        spin_until(executor, lambda: sim_seconds(recorder) >= baseline_end, timeout_wall=45.0)

        driver = SquareDriver(make_stages(side, radius, speed, 1))
        executor.add_node(driver)
        spin_until(executor, lambda: driver.finished)
        if driver.aborted:
            raise RuntimeError("Gazebo simulation time reset during the trial")
        drive_start = driver.stage_start - sum(stage[3] for stage in driver.stages)
        drive_end = sim_seconds(recorder)
        executor.remove_node(driver)
        driver.destroy_node()
        driver = None

        # Capture settling after the zero-velocity command.
        settle_end = sim_seconds(recorder) + 2.0
        spin_until(executor, lambda: sim_seconds(recorder) >= settle_end, timeout_wall=45.0)
        metadata = {
            "complete": True,
            "side_m": side,
            "radius_m": radius,
            "speed_mps": speed,
            "recorder_start_sim_s": recorder.start_time,
            "drive_start_sim_s": drive_start,
            "drive_end_sim_s": drive_end,
            "odom_rows": recorder.counts["odom"],
            "truth_rows": recorder.counts["truth"],
            "wall_elapsed_s": time.monotonic() - wall_start,
            "trial_type": "consecutive_lap_without_world_reset",
        }
        recorder.close_files()
        (folder / "metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        return metadata
    finally:
        if driver is not None:
            for _ in range(10):
                driver.publish_command()
            executor.remove_node(driver)
            driver.destroy_node()
        recorder.close_files()
        executor.remove_node(recorder)
        recorder.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=10, help="Number of consecutive one-lap trials")
    parser.add_argument("--out", type=Path, required=True, help="New directory for run_01, run_02, ...")
    parser.add_argument("--side", type=float, default=2.0, help="Outer square width in metres")
    parser.add_argument("--radius", type=float, default=0.65, help="Turn radius in metres")
    parser.add_argument("--speed", type=float, default=0.1, help="Forward speed in m/s")
    parser.add_argument("--model", default="smart_car", help="Gazebo model name")
    args = parser.parse_args()
    if args.runs < 1 or args.speed <= 0:
        parser.error("--runs must be at least 1 and --speed must be positive")
    try:
        make_stages(args.side, args.radius, args.speed, 1)
    except ValueError as error:
        parser.error(str(error))
    folders = [args.out / f"run_{index:02d}" for index in range(1, args.runs + 1)]
    if any(folder.exists() for folder in folders):
        parser.error("A run directory already exists; choose a new --out to protect saved data")
    print(f"Starting {args.runs} consecutive run(s); each lap needs about "
          f"{sum(stage[3] for stage in make_stages(args.side, args.radius, args.speed, 1)):.1f} "
          "simulation seconds", flush=True)

    rclpy.init()
    executor = SingleThreadedExecutor()
    try:
        for index, folder in enumerate(folders, 1):
            print(f"Run {index}/{args.runs}: recording {folder}", flush=True)
            result = run_one(executor, folder, args.side, args.radius, args.speed, args.model)
            print(f"Run {index}/{args.runs}: complete, {result['odom_rows']} odom and "
                  f"{result['truth_rows']} truth rows", flush=True)
    except (KeyboardInterrupt, ExternalShutdownException, InterruptedError) as error:
        print(f"Interrupted: {error}")
        raise SystemExit(130) from None
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
