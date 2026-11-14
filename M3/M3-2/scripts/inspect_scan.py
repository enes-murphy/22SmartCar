#!/usr/bin/env python3
"""Print front, left, and right distances from one live laser scan."""

import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def distance_at(scan, angle):
    # Sensor angles are expressed relative to lidar_link: 0 front, +90 left.
    count = len(scan.ranges)
    center = round((angle - scan.angle_min) / scan.angle_increment) % count
    readings = [scan.ranges[(center + offset) % count] for offset in (-2, -1, 0, 1, 2)]
    valid = [value for value in readings if math.isfinite(value)
             and scan.range_min <= value <= scan.range_max]
    return statistics.median(valid) if valid else None


def main():
    rclpy.init()
    node = Node("scan_inspector")
    result = []
    node.create_subscription(LaserScan, "/scan", result.append, qos_profile_sensor_data)
    deadline = time.monotonic() + 10.0
    try:
        while not result and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not result:
            raise RuntimeError("No /scan received in 10 seconds; check Gazebo")
        scan = result[0]
        print(f"frame={scan.header.frame_id}, rays={len(scan.ranges)}, "
              f"angle_min={scan.angle_min:.3f}, increment={scan.angle_increment:.4f} rad")
        for name, angle in (("front", 0.0), ("left", math.pi / 2),
                            ("right", -math.pi / 2)):
            value = distance_at(scan, angle)
            print(f"{name}: {value:.3f} m" if value is not None else f"{name}: no valid hit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
