#!/usr/bin/env python3
"""Run one guarded M3-3 navigation trial with a turning route."""

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
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=11.0)
    parser.add_argument("--goal-y", type=float, default=3.0)
    parser.add_argument("--goal-yaw-deg", type=float, default=90.0)
    parser.add_argument("--min-travel", type=float, default=5.0)
    parser.add_argument("--from-current", action="store_true",
                        help="allow a later leg to start away from the launch pose")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="maximum wall-clock seconds after goal acceptance")
    args = parser.parse_args()
    if args.timeout <= 0 or args.min_travel < 0:
        parser.error("--timeout must be positive and --min-travel nonnegative")

    # Keep the context alive on Ctrl+C so the car can receive a final stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("m3_3_nav_trial")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)
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
        if now - last_feedback >= 5.0:
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
        if not args.from_current and not (-0.5 <= start.x <= 0.5 and abs(start.y) <= 0.5):
            raise RuntimeError("Car is not near the start; restart the launch first")
        if not client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("/navigate_to_pose is unavailable")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        half_yaw = math.radians(args.goal_yaw_deg) / 2
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)
        print(f"Sending Nav2 goal: map ({args.goal_x:.2f}, {args.goal_y:.2f}, "
              f"{args.goal_yaw_deg:.0f} deg)", flush=True)
        future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            raise TimeoutError("Nav2 did not accept the goal within 10 seconds")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Nav2 rejected the goal")

        outcome = handle.get_result_async()
        start_wall = time.monotonic()
        deadline = time.monotonic() + args.timeout
        while not outcome.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not outcome.done():
            raise TimeoutError("Nav2 short drive exceeded the time limit")
        done = True
        result = outcome.result()
        elapsed = time.monotonic() - start_wall
        # Report measured motion even when Nav2 aborts, so the two outcomes
        # cannot be confused during a controller diagnosis.
        rclpy.spin_once(node, timeout_sec=0.2)
        end = last_odom.pose.pose.position
        distance_moved = math.hypot(end.x - start.x, end.y - start.y)
        print(f"Odom end: x={end.x:.2f}, y={end.y:.2f}; "
              f"moved={distance_moved:.2f} m; elapsed={elapsed:.1f} s", flush=True)
        # Nav2 checks base_link in map, while /odom describes base_footprint.
        transform = tf_buffer.lookup_transform("map", "base_link", Time())
        p = transform.transform.translation
        error = math.hypot(p.x - args.goal_x, p.y - args.goal_y)
        print(f"Map base_link: x={p.x:.2f}, y={p.y:.2f}; "
              f"goal error={error:.2f} m", flush=True)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 ended with status {result.status}")
        if distance_moved < args.min_travel or error > 0.25:
            raise RuntimeError("Nav2 status succeeded but measured movement/pose did not")
        print("SUCCEEDED: Nav2 reached the turning-route target", flush=True)
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
