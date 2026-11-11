#!/usr/bin/env python3
"""Record M3-1 odometry and Gazebo ground truth as two four-column CSVs."""

import argparse
import csv
import math
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.node import Node


def yaw_from_quaternion(q):
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return math.pi if yaw <= -math.pi else yaw


class TrajectoryRecorder(Node):
    def __init__(self, output_dir, model_name):
        super().__init__("trajectory_recorder")
        # Humble declares this parameter automatically; set the existing one.
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.model_name = model_name
        self.start_time = None
        self.counts = {"odom": 0, "truth": 0}
        output_dir.mkdir(parents=True, exist_ok=True)
        self.files = {
            label: (output_dir / f"{label}.csv").open("w", newline="", encoding="utf-8")
            for label in ("odom", "truth")
        }
        self.writers = {label: csv.writer(file, lineterminator="\n") for label, file in self.files.items()}
        for writer in self.writers.values():
            writer.writerow(["t", "x", "y", "theta"])
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self.on_truth, 10)

    def relative_time(self, stamp=None):
        # Odometry supplies an exact simulation stamp. ModelStates has no stamp,
        # so use its receipt time on the same simulation clock.
        now = (
            stamp.sec + stamp.nanosec / 1e9
            if stamp is not None
            else self.get_clock().now().nanoseconds / 1e9
        )
        if now <= 0:
            return None
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    def write_pose(self, label, pose, stamp=None):
        t = self.relative_time(stamp)
        if t is None:
            return
        self.writers[label].writerow(
            [f"{t:.6f}", f"{pose.position.x:.6f}",
             f"{pose.position.y:.6f}", f"{yaw_from_quaternion(pose.orientation):.6f}"]
        )
        self.counts[label] += 1

    def on_odom(self, message):
        self.write_pose("odom", message.pose.pose, message.header.stamp)

    def on_truth(self, message):
        if self.model_name not in message.name:
            self.get_logger().warn(f"Model {self.model_name!r} not found", throttle_duration_sec=5.0)
            return
        index = message.name.index(self.model_name)
        self.write_pose("truth", message.pose[index])

    def close_files(self):
        for file in self.files.values():
            file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="Output directory")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation seconds")
    parser.add_argument("--model", default="smart_car", help="Gazebo model name")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    rclpy.init()
    recorder = TrajectoryRecorder(args.out, args.model)
    try:
        while rclpy.ok():
            rclpy.spin_once(recorder, timeout_sec=0.2)
            if recorder.start_time is not None:
                elapsed = recorder.get_clock().now().nanoseconds / 1e9 - recorder.start_time
                if elapsed >= args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close_files()
        print(f"Saved {recorder.counts['odom']} odom and {recorder.counts['truth']} truth rows to {args.out}")
        recorder.destroy_node()
        # Ctrl+C may already have shut down the default ROS context.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
