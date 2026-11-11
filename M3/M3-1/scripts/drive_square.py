#!/usr/bin/env python3
"""Drive a rounded square using open-loop Twist commands and simulation time."""

import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


WHEELBASE = 0.28
TRACK_WIDTH = 0.18
STEERING_LIMIT = 0.5236


def make_stages(side, radius, speed, laps):
    straight = side - 2.0 * radius
    if straight <= 0:
        raise ValueError("side must be greater than twice the turn radius")
    minimum_radius = WHEELBASE / math.tan(STEERING_LIMIT) + TRACK_WIDTH / 2.0
    if radius < minimum_radius:
        raise ValueError(f"turn radius must be at least {minimum_radius:.3f} m")
    stages = []
    for lap in range(1, laps + 1):
        for edge in range(1, 5):
            stages.append((lap, edge, "straight", straight / speed, speed, 0.0))
            stages.append((lap, edge, "left turn", math.pi * radius / (2.0 * speed),
                           speed, speed / radius))
    return stages


class SquareDriver(Node):
    def __init__(self, stages):
        super().__init__("square_driver")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.publisher = self.create_publisher(
            Twist, "/cmd_vel", QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.stages = stages
        self.stage_index = 0
        self.stage_start = None
        self.previous_time = None
        self.stop_ticks = 0
        self.finished = False
        # A steady timer can wake before /clock starts; stage durations still use Gazebo time.
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def publish_command(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now <= 0 or self.publisher.get_subscription_count() == 0:
            return
        if self.previous_time is not None and now < self.previous_time:
            self.get_logger().error("Simulation clock reset during the run; stopping")
            self.stop_ticks = 10
            self.stage_index = len(self.stages)
        self.previous_time = now
        if self.stage_start is None:
            self.stage_start = now
            self.get_logger().info("Controller and simulation clock ready; starting square")

        while self.stage_index < len(self.stages):
            lap, edge, kind, duration, linear, angular = self.stages[self.stage_index]
            if now - self.stage_start < duration:
                self.publish_command(linear, angular)
                return
            self.stage_start += duration
            self.get_logger().info(f"Lap {lap}, edge {edge}: finished {kind}")
            self.stage_index += 1

        self.publish_command()
        self.stop_ticks += 1
        if self.stop_ticks >= 10:
            self.finished = True
            self.get_logger().info("Run complete; zero velocity command sent")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", type=float, default=2.0, help="Outer square width in metres")
    parser.add_argument("--radius", type=float, default=0.65, help="Corner centreline radius in metres")
    parser.add_argument("--speed", type=float, default=0.1, help="Forward speed in m/s")
    parser.add_argument("--laps", type=int, default=1, help="Number of complete circuits")
    args = parser.parse_args()
    if args.side <= 0 or args.speed <= 0 or args.laps < 1:
        parser.error("side and speed must be positive; laps must be at least 1")
    try:
        stages = make_stages(args.side, args.radius, args.speed, args.laps)
    except ValueError as error:
        parser.error(str(error))
    straight = args.side - 2 * args.radius
    seconds = sum(stage[3] for stage in stages)
    print(f"Straight {straight:.3f} m, turn radius {args.radius:.3f} m, "
          f"expected {seconds:.1f} simulation seconds for {args.laps} lap(s)")

    rclpy.init()
    driver = SquareDriver(stages)
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
