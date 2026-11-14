#!/usr/bin/env python3
"""Test 3 m of laser-centered straight driving."""

import argparse
import math
import sys
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "M3-1" / "scripts"))
from drive_square import SquareDriver  # noqa: E402
from inspect_scan import distance_at  # noqa: E402


class CenteredDriver(SquareDriver):
    def __init__(self, stages):
        super().__init__(stages)
        self.scan = None
        self.scan_time = None
        self.last_report = None
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, message):
        self.scan = message
        self.scan_time = self.get_clock().now().nanoseconds / 1e9

    def tick(self):
        if self.scan is None:
            return
        super().tick()

    def publish_command(self, linear=0.0, angular=0.0):
        if linear > 0 and abs(angular) < 1e-9:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.scan_time is None or now - self.scan_time > 0.5:
                super().publish_command()
                return
            left = distance_at(self.scan, math.pi / 2)
            right = distance_at(self.scan, -math.pi / 2)
            front = distance_at(self.scan, 0.0)
            if front is not None and front < 0.50:
                self.get_logger().error("Obstacle ahead within 0.50 m; stopping")
                self.aborted = True
                self.stage_index = len(self.stages)
                super().publish_command()
                return
            # At corners a side wall may temporarily end. Keep nominal heading
            # until both walls are visible again; never invent a range value.
            if left is not None and right is not None:
                # Positive difference means more room on the left: turn left.
                angular = max(-0.12, min(0.12, 0.30 * (left - right)))
            if (left is not None and right is not None
                    and (self.last_report is None or now - self.last_report >= 2.0)):
                self.get_logger().info(
                    f"wall distances: left={left:.2f} m, right={right:.2f} m; "
                    f"correction={angular:+.3f} rad/s"
                )
                self.last_report = now
        super().publish_command(linear, angular)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=0.10, help="0.10 or 0.20 m/s")
    args = parser.parse_args()
    if args.speed not in (0.10, 0.20):
        parser.error("--speed must be 0.10 or 0.20 m/s")
    print(f"Laser-centered 3 m test at {args.speed:.2f} m/s, "
          f"expected {3.0 / args.speed:.1f} simulation seconds")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stages = [(1, 1, "centered straight test", 3.0 / args.speed, args.speed, 0.0)]
    driver = CenteredDriver(stages)
    try:
        while rclpy.ok() and not driver.finished:
            rclpy.spin_once(driver, timeout_sec=0.2)
    except KeyboardInterrupt:
        driver.get_logger().info("Interrupted; stopping the car")
    finally:
        for _ in range(10):
            driver.publish_command()
        driver.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
