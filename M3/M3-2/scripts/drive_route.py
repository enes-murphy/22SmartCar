#!/usr/bin/env python3
"""Drive the M3-2 ring corridor from (-5, -4), heading east."""

import argparse
import math
import sys
from pathlib import Path

# Reuse M3-1's already tested ROS publisher, simulation clock, and stop logic.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "M3-1" / "scripts"))
from drive_square import SquareDriver, STEERING_LIMIT, TRACK_WIDTH, WHEELBASE  # noqa: E402

import rclpy  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from drive_centered_test import CenteredDriver  # noqa: E402


def make_route(laps, straight_speed, turn_speed, radius):
    minimum_radius = WHEELBASE / math.tan(STEERING_LIMIT) + TRACK_WIDTH / 2
    if radius < minimum_radius:
        raise ValueError(f"turn radius must be at least {minimum_radius:.3f} m")
    # Tangency points for a 1 m corner radius are x=+/-5, y=+/-3.
    # The car starts at (-5,-4), so the straight lengths are 10, 6, 10, 6 m.
    lengths = (10.0, 6.0, 10.0, 6.0)
    stages = []
    for lap in range(1, laps + 1):
        for edge, length in enumerate(lengths, start=1):
            stages.append((lap, edge, "straight", length / straight_speed,
                           straight_speed, 0.0))
            stages.append((lap, edge, "left turn", math.pi * radius / (2 * turn_speed),
                           turn_speed, turn_speed / radius))
    return stages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--straight-speed", type=float, default=0.20, help="m/s")
    parser.add_argument("--turn-speed", type=float, default=0.10, help="m/s")
    parser.add_argument("--radius", type=float, default=1.0, help="m")
    parser.add_argument("--straight-test", type=float, metavar="METRES",
                        help="Drive only this many metres of the first straight")
    parser.add_argument("--dry-run", action="store_true", help="Print route without driving")
    parser.add_argument("--wall-follow", action="store_true",
                        help="Use laser wall-distance feedback on straight segments")
    args = parser.parse_args()
    if args.laps < 1 or args.straight_speed <= 0 or args.turn_speed <= 0:
        parser.error("laps must be at least 1 and speeds must be positive")
    if abs(args.radius - 1.0) > 1e-9:
        parser.error("this fixed world route requires a 1.0 m corner radius")
    if args.straight_test is not None and not (0 < args.straight_test <= 3.0):
        parser.error("--straight-test must be in (0, 3.0] metres")
    try:
        stages = make_route(args.laps, args.straight_speed, args.turn_speed, args.radius)
    except ValueError as error:
        parser.error(str(error))
    if args.straight_test is not None:
        stages = [(1, 1, "straight test", args.straight_test / args.straight_speed,
                   args.straight_speed, 0.0)]

    print("Start: (-5, -4), heading east; loop: east, north, west, south")
    for lap, edge, kind, duration, linear, angular in stages:
        print(f"Lap {lap}, edge {edge}: {kind:9s} {duration:6.1f} s  "
              f"linear={linear:.2f} m/s  angular={angular:.2f} rad/s")
    print(f"Expected total: {sum(stage[3] for stage in stages):.1f} simulation seconds")
    if args.dry_run:
        return

    # Keep the ROS context alive on Ctrl+C so the finally block can publish
    # an explicit zero command before shutting down.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    driver = CenteredDriver(stages) if args.wall_follow else SquareDriver(stages)
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
