#!/usr/bin/env python3
"""Run a guarded 2 m Nav2 driving test in the M3-2 bottom corridor."""

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="maximum wall-clock seconds after goal acceptance")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    # Keep the context alive on Ctrl+C so the car can receive a final stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("m3_3_short_nav_test")
    stop_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    last_odom = None

    def on_odom(message):
        nonlocal last_odom
        last_odom = message

    odom_sub = node.create_subscription(Odometry, "/odom", on_odom, 10)
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    handle = None
    done = False
    last_feedback = 0.0

    def on_feedback(message):
        nonlocal last_feedback
        now = time.monotonic()
        if now - last_feedback >= 2.0:
            feedback = message.feedback
            p = feedback.current_pose.pose.position
            print(f"Nav2: x={p.x:.2f}, y={p.y:.2f}, "
                  f"remaining={feedback.distance_remaining:.2f} m", flush=True)
            last_feedback = now

    try:
        deadline = time.monotonic() + 10.0
        while last_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if last_odom is None:
            raise RuntimeError("No /odom received")
        start = last_odom.pose.pose.position
        print(f"Odom start: x={start.x:.2f}, y={start.y:.2f}", flush=True)
        if not (-0.5 <= start.x <= 0.8 and abs(start.y) <= 0.5):
            raise RuntimeError("Car is not near the start; restart the launch first")
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("/navigate_to_pose is unavailable")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = 2.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0
        print("Sending Nav2 goal: map (2.0, 0.0, 0 deg)", flush=True)
        future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            raise TimeoutError("Nav2 did not accept the goal within 10 seconds")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Nav2 rejected the goal")

        outcome = handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while not outcome.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not outcome.done():
            raise TimeoutError("Nav2 short drive exceeded the time limit")
        done = True
        result = outcome.result()
        # Report measured motion even when Nav2 aborts, so the two outcomes
        # cannot be confused during a controller diagnosis.
        rclpy.spin_once(node, timeout_sec=0.2)
        end = last_odom.pose.pose.position
        error = math.hypot(end.x - 2.0, end.y)
        print(f"Odom end: x={end.x:.2f}, y={end.y:.2f}; "
              f"target error={error:.2f} m", flush=True)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 ended with status {result.status}")
        # A successful action status is insufficient: in one isolated run the
        # controller reported success despite the car remaining at its start.
        if error > 0.30:
            raise RuntimeError("Nav2 reported success but odometry did not reach the target")
        print("SUCCEEDED: Nav2 reached the short target", flush=True)
        print("Next: inspect AMCL and Gazebo truth before a longer route", flush=True)
    except KeyboardInterrupt:
        print("Interrupted: canceling Nav2 goal and stopping", flush=True)
    finally:
        if handle is not None and not done:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.0)
        for _ in range(10):
            stop_publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        client.destroy()
        node.destroy_subscription(odom_sub)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
