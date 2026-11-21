#!/usr/bin/env python3
"""Replay the same M3-2 sensor bag into one isolated Slam Toolbox configuration."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def stop(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--bag", type=Path, default=Path("bags/slam_replay_inputs"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--rate", type=float, default=3.0)
    args = parser.parse_args()
    if args.out.exists():
        parser.error(f"output already exists: {args.out}")
    if not (args.bag / "metadata.yaml").exists():
        parser.error("run scripts/prepare_slam_replay.py first")
    args.out.mkdir(parents=True)
    config_copy = args.out / "slam_params.yaml"
    shutil.copy2(args.params, config_copy)
    (args.out / "trial.json").write_text(json.dumps({
        "source_bag": "bags/formal_02",
        "replay_bag": str(args.bag),
        "replay_rate": args.rate,
        "ros_domain_id": args.domain,
        "config_sha256": hashlib.sha256(config_copy.read_bytes()).hexdigest(),
    }, indent=2) + "\n")
    env = dict(os.environ,
               ROS_DOMAIN_ID=str(args.domain),
               ROS_LOCALHOST_ONLY="1",
               ROS_HOME=str(args.out.resolve() / "ros_home"),
               ROS_LOG_DIR=str(args.out.resolve() / "ros_logs"))
    (args.out / "ros_home").mkdir()
    (args.out / "ros_logs").mkdir()
    slam = None
    try:
        with (args.out / "slam.log").open("w") as log:
            slam = subprocess.Popen(
                ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node", "--ros-args",
                 "-r", "__node:=slam_toolbox", "--params-file", str(args.params.resolve()),
                 "-p", "use_sim_time:=true"],
                env=env, stdout=log, stderr=subprocess.STDOUT)
            time.sleep(3)
            if slam.poll() is not None:
                raise RuntimeError("Slam Toolbox exited before bag playback")
            with (args.out / "play.log").open("w") as play_log:
                play = subprocess.run(
                    ["ros2", "bag", "play", str(args.bag), "-r", str(args.rate),
                     "--disable-keyboard-controls"],
                    env=env, stdout=play_log, stderr=subprocess.STDOUT,
                    timeout=max(180, 339 / args.rate * 2), check=False)
            if play.returncode:
                raise RuntimeError(f"bag player exited {play.returncode}")
            time.sleep(3)
            with (args.out / "save.log").open("w") as save_log:
                saved = subprocess.run(
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f",
                     str(args.out / "map"), "--ros-args", "-p", "use_sim_time:=false"],
                    env=env, stdout=save_log, stderr=subprocess.STDOUT,
                    timeout=25, check=False)
            if saved.returncode:
                raise RuntimeError(f"map saver exited {saved.returncode}; see save.log and slam.log")
            print(f"Saved: {args.out / 'map.yaml'}")
            print(f"Saved: {args.out / 'map.pgm'}")
    finally:
        if slam:
            stop(slam)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAILED: {error}", file=sys.stderr)
        raise
