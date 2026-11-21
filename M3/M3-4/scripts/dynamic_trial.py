#!/usr/bin/env python3
"""Run one navigation trial and insert a box after the goal starts."""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PolygonStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def sdf_box(name, sx, sy, height):
    return (f'<sdf version="1.6"><model name="{name}"><static>true</static>'
            f'<link name="body"><collision name="collision"><geometry><box><size>'
            f'{sx} {sy} {height}</size></box></geometry></collision>'
            f'<visual name="visual"><geometry><box><size>{sx} {sy} {height}'
            '</size></box></geometry><material><ambient>0.9 0.15 0.1 1</ambient>'
            '</material></visual></link></model></sdf>')


def yaw(quat):
    return math.atan2(2 * (quat.w * quat.z + quat.x * quat.y),
                      1 - 2 * (quat.y * quat.y + quat.z * quat.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--delay', type=float, default=8.0,
                        help='simulation seconds after goal acceptance')
    parser.add_argument('--world-x', type=float, default=-1.0)
    parser.add_argument('--world-y', type=float, default=-3.35)
    parser.add_argument('--size-x', type=float, default=0.30)
    parser.add_argument('--size-y', type=float, default=0.30)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--goal-x', type=float, default=8.0)
    parser.add_argument('--goal-y', type=float, default=0.4)
    parser.add_argument('--goal-yaw-deg', type=float, default=0.0)
    parser.add_argument('--no-obstacle', action='store_true')
    args = parser.parse_args()
    if args.delay < 0 or min(args.size_x, args.size_y, args.timeout) <= 0:
        parser.error('delay must be nonnegative; dimensions and timeout must be positive')
    args.out.mkdir(parents=True, exist_ok=False)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('m3_4_dynamic_trial')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer(node=node)
    tf_listener = TransformListener(tf_buffer, node)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    spawner = node.create_client(SpawnEntity, '/spawn_entity')
    stopper = node.create_publisher(Twist, '/cmd_vel', 10)
    last_odom = None
    odom_rows, truth_rows, plan_rows, command_rows = [], [], [], []
    scan_rows, costmap_rows = [], []
    footprint_rows = []
    goal_start_sim = None
    obstacle_sim = None
    spawn_future = None
    action_handle = None
    action_result = None
    outcome = 'ERROR'
    error_text = None
    result_status = None
    elapsed_wall = None
    elapsed_sim = None
    end_map = None

    def sim_time():
        return node.get_clock().now().nanoseconds / 1e9

    def on_odom(msg):
        nonlocal last_odom
        last_odom = msg
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odom_rows.append((sim_time(), p.x, p.y, yaw(q)))

    def on_truth(msg):
        if 'smart_car' not in msg.name:
            return
        pose = msg.pose[msg.name.index('smart_car')]
        p = pose.position
        truth_rows.append((sim_time(), p.x, p.y, yaw(pose.orientation)))

    def on_plan(msg):
        if goal_start_sim is None or not msg.poses:
            return
        # Preserve every published path, including before and after injection.
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        corridor = [y for x, y in pts if 3.5 <= x <= 4.5]
        plan_rows.append((sim_time(), len(pts),
                          max((abs(y) for y in corridor), default=0.0), pts))

    def on_cmd(msg):
        if goal_start_sim is not None:
            command_rows.append((sim_time(), msg.linear.x, msg.angular.z))

    def on_scan(msg):
        if goal_start_sim is None:
            return
        front = [r for i, r in enumerate(msg.ranges)
                 if math.isfinite(r) and abs(math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                                                       math.cos(msg.angle_min + i * msg.angle_increment))) < 0.15]
        scan_rows.append((sim_time(), min(front) if front else None))

    def on_global_costmap(msg):
        if goal_start_sim is None:
            return
        # Map coordinates are offset from world by roughly (+5, +4).
        # This diagnostic records the obstacle cell without requiring TF in the callback.
        info = msg.info
        x = int((args.world_x + 5.0 - info.origin.position.x) / info.resolution)
        y = int((args.world_y + 4.0 - info.origin.position.y) / info.resolution)
        if 0 <= x < info.width and 0 <= y < info.height:
            near = [msg.data[yy * info.width + xx]
                    for yy in range(max(0, y - 4), min(info.height, y + 5))
                    for xx in range(max(0, x - 4), min(info.width, x + 5))]
            costmap_rows.append((sim_time(), msg.data[y * info.width + x], max(near)))

    def on_footprint(name, msg):
        if not msg.polygon.points:
            return
        pts = msg.polygon.points
        footprint_rows.append((sim_time(), name, msg.header.frame_id,
                               msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                               sum(p.x for p in pts) / len(pts),
                               sum(p.y for p in pts) / len(pts)))

    subs = [
        node.create_subscription(Odometry, '/odom', on_odom, 20),
        node.create_subscription(ModelStates, '/gazebo/model_states', on_truth, 20),
        node.create_subscription(NavPath, '/plan', on_plan, 10),
        node.create_subscription(Twist, '/cmd_vel', on_cmd, 20),
        node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', on_global_costmap, 10),
        node.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 lambda msg: on_footprint('local', msg), 10),
        node.create_subscription(PolygonStamped, '/global_costmap/published_footprint',
                                 lambda msg: on_footprint('global', msg), 10),
    ]
    start_wall = None
    try:
        deadline = time.monotonic() + 30
        while (last_odom is None or not truth_rows) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if last_odom is None or not truth_rows:
            raise RuntimeError('No odometry or Gazebo truth')
        p = last_odom.pose.pose.position
        if not (-0.5 < p.x < 0.5 and abs(p.y) < 0.5):
            raise RuntimeError(f'Robot not at start: odom ({p.x:.2f}, {p.y:.2f})')
        if not client.wait_for_server(timeout_sec=15) or not spawner.wait_for_service(timeout_sec=15):
            raise RuntimeError('Navigation action or Gazebo spawn service unavailable')

        # Preflight: reject a launch where Nav2's internal costmap TF buffers
        # remain at the spawn pose despite live /odom. This sometimes occurs
        # on this Humble setup and otherwise creates a misleading run failure.
        footprint_deadline = time.monotonic() + 3.0
        while ({r[1] for r in footprint_rows} != {'local', 'global'} and
               time.monotonic() < footprint_deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
        footprint_before = {name: next((r[4] for r in reversed(footprint_rows)
                                        if r[1] == name), None)
                            for name in ('local', 'global')}

        # AMCL only refreshes map->odom after motion. A stationary fresh launch
        # leaves an old transform and can make FollowPath report false success.
        warm_start = sim_time()
        warm_cmd = Twist()
        warm_cmd.linear.x = 0.1
        while sim_time() - warm_start < 3.0:
            stopper.publish(warm_cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
        stopper.publish(Twist())
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        footprint_after = {name: next((r[4] for r in reversed(footprint_rows)
                                       if r[1] == name), None)
                           for name in ('local', 'global')}
        for name in ('local', 'global'):
            a, b = footprint_before[name], footprint_after[name]
            if a is None or b is None or b - a < 0.12:
                raise RuntimeError(f'PREFLIGHT_COSTMAP_FROZEN: {name} '
                                   f'footprint x {a} -> {b} after odom warmup')
        fresh = tf_buffer.lookup_transform('map', 'odom', Time())
        tf_stamp = fresh.header.stamp.sec + fresh.header.stamp.nanosec / 1e9
        if sim_time() - tf_stamp > 1.0:
            raise RuntimeError(f'AMCL map->odom is stale by {sim_time() - tf_stamp:.2f} s')
        print(f'AMCL warmup: odom x={last_odom.pose.pose.position.x:.2f}, '
              f'TF age={sim_time() - tf_stamp:.2f} s', flush=True)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        half_yaw = math.radians(args.goal_yaw_deg) / 2
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Navigation goal not accepted')
        action_handle = future.result()
        action_result = action_handle.get_result_async()
        start_wall = time.monotonic()
        goal_start_sim = sim_time()
        print(f'Goal accepted at sim {goal_start_sim:.3f}; box in {args.delay:.1f} s', flush=True)
        while not action_result.done() and time.monotonic() - start_wall < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = sim_time()
            if not args.no_obstacle and spawn_future is None and now - goal_start_sim >= args.delay:
                request = SpawnEntity.Request()
                request.name = 'dynamic_box'
                request.xml = sdf_box(request.name, args.size_x, args.size_y, 0.50)
                request.initial_pose.position.x = args.world_x
                request.initial_pose.position.y = args.world_y
                request.initial_pose.position.z = 0.25
                request.initial_pose.orientation.w = 1.0
                spawn_future = spawner.call_async(request)
                print(f'Spawn requested at sim {now:.3f}', flush=True)
            if spawn_future is not None and obstacle_sim is None and spawn_future.done():
                response = spawn_future.result()
                if not response.success:
                    raise RuntimeError(f'Gazebo rejected obstacle: {response.status_message}')
                obstacle_sim = now
                print(f'Obstacle inserted at sim {obstacle_sim:.3f}', flush=True)
        elapsed_wall = time.monotonic() - start_wall
        elapsed_sim = sim_time() - goal_start_sim
        if not action_result.done():
            outcome = 'TIMEOUT'
            error_text = 'Navigation exceeded wall-clock timeout'
        else:
            result_status = action_result.result().status
            outcome = 'SUCCEEDED' if result_status == GoalStatus.STATUS_SUCCEEDED else 'FAILED'
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        transform = tf_buffer.lookup_transform('map', 'base_link', Time())
        end_map = [transform.transform.translation.x, transform.transform.translation.y]
        goal_error = math.dist(end_map, [args.goal_x, args.goal_y])
        if outcome == 'SUCCEEDED' and goal_error > 0.25:
            outcome = 'FALSE_SUCCESS'
            error_text = f'Nav2 success status but goal error {goal_error:.2f} m'
        print(f'{outcome}: {elapsed_wall:.1f} s; map base_link {end_map}', flush=True)
    except Exception as exc:
        error_text = str(exc)
        print(f'ERROR: {error_text}', flush=True)
    finally:
        if action_handle is not None and action_result is not None and not action_result.done():
            cancel = action_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3)
        for _ in range(5):
            stopper.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        for name, header, rows in (
            ('odom.csv', ('sim_s', 'x', 'y', 'yaw_rad'), odom_rows),
            ('truth.csv', ('sim_s', 'x', 'y', 'yaw_rad'), truth_rows),
            ('cmd_vel.csv', ('sim_s', 'linear_x', 'angular_z'), command_rows),
            ('front_scan.csv', ('sim_s', 'min_front_m'), scan_rows),
            ('global_costmap_box_cell.csv', ('sim_s', 'occupancy', 'nearby_max'), costmap_rows),
            ('footprints.csv', ('sim_s', 'source', 'frame_id', 'header_sim_s', 'x', 'y'), footprint_rows),
        ):
            with (args.out / name).open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(header)
                writer.writerows(rows)
        with (args.out / 'plans.jsonl').open('w') as stream:
            for t, count, max_y, pts in plan_rows:
                stream.write(json.dumps({'sim_s': t, 'poses': count,
                                         'max_abs_y_at_box': max_y, 'xy': pts}) + '\n')
        summary = {
            'outcome': outcome, 'result_status': result_status, 'error': error_text,
            'elapsed_wall_s': elapsed_wall, 'elapsed_sim_s': elapsed_sim,
            'goal_start_sim_s': goal_start_sim,
            'obstacle_sim_s': obstacle_sim, 'end_map_base_link_xy': end_map,
            'obstacle_world_xy': [args.world_x, args.world_y],
            'obstacle_size_xy': [args.size_x, args.size_y],
            'goal_map_xy': [args.goal_x, args.goal_y],
            'goal_yaw_deg': args.goal_yaw_deg,
            'no_obstacle': args.no_obstacle,
            'injection_delay_sim_s': args.delay, 'plan_count': len(plan_rows),
            'odom_rows': len(odom_rows), 'truth_rows': len(truth_rows),
        }
        (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if outcome == 'SUCCEEDED' and (args.no_obstacle or obstacle_sim is not None) else 1


if __name__ == '__main__':
    raise SystemExit(main())
