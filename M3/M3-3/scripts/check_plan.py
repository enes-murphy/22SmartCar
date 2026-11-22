#!/usr/bin/env python3
"""Ask Nav2 to plan to a map goal without moving the car."""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def pose(x, y, yaw):
    result = PoseStamped()
    result.header.frame_id = "map"
    result.pose.position.x = x
    result.pose.position.y = y
    result.pose.orientation.z = math.sin(yaw / 2)
    result.pose.orientation.w = math.cos(yaw / 2)
    return result


def wait_result(node, future, timeout=20):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("Nav2 did not answer within 20 seconds")
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=11.0)
    parser.add_argument("--goal-y", type=float, default=3.0)
    parser.add_argument("--goal-yaw-deg", type=float, default=90.0)
    parser.add_argument("--current-start", action="store_true",
                        help="plan from the robot's current pose instead of map (0, 0, 0)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("m3_3_plan_check")
    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    try:
        if not client.wait_for_server(timeout_sec=20):
            raise RuntimeError("Planner action is unavailable; check /planner_server")
        goal = ComputePathToPose.Goal()
        goal.start = pose(0.0, 0.0, 0.0)
        goal.goal = pose(args.goal_x, args.goal_y,
                         math.radians(args.goal_yaw_deg))
        goal.planner_id = "Hybrid"
        goal.use_start = not args.current_start
        handle = wait_result(node, client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError("Planner rejected the request")
        outcome = wait_result(node, handle.get_result_async())
        if outcome.status != GoalStatus.STATUS_SUCCEEDED:
            print(f"NO PATH: Nav2 planning failed (action status {outcome.status}). "
                  "Choose another free point or heading.", file=sys.stderr)
            return 1
        points = [(p.pose.position.x, p.pose.position.y)
                  for p in outcome.result.path.poses]
        if len(points) < 2:
            raise RuntimeError("Planner returned an empty path")
        length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        print(f"SUCCEEDED: {len(points)} poses, {length:.2f} m path")
        print(f"Start: ({points[0][0]:.2f}, {points[0][1]:.2f}) map")
        print(f"Goal:  ({points[-1][0]:.2f}, {points[-1][1]:.2f}) map")
        print("Length requirement (>5 m): " + ("met" if length > 5.0 else "not met"))
        print("Planning only: no velocity command was sent")
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
