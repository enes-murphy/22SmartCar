#!/usr/bin/env python3
"""Record live Nav2 costmaps and velocity commands, then render evidence PNGs."""

import argparse
import csv
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=120.0)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('m3_3_diagnostic_capture')
    maps = {}
    commands = []
    start = time.monotonic()
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)

    def on_map(name):
        def callback(message):
            maps[name] = (message.info.width, message.info.height,
                          message.info.resolution,
                          message.info.origin.position.x,
                          message.info.origin.position.y,
                          np.array(message.data, dtype=np.int16).copy())
        return callback

    subscriptions = [
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', on_map('global'), qos),
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', on_map('local'), qos),
        node.create_subscription(Twist, '/cmd_vel',
                                 lambda msg: commands.append((time.monotonic() - start,
                                                              msg.linear.x, msg.angular.z)), 20),
    ]
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()

    for name, record in maps.items():
        width, height, resolution, x, y, data = record
        image = data.reshape(height, width).astype(float)
        image[image < 0] = np.nan
        fig, ax = plt.subplots(figsize=(7, 5.5), constrained_layout=True)
        ax.imshow(image, origin='lower', cmap='gray_r', vmin=0, vmax=100,
                  extent=(x, x + width * resolution, y, y + height * resolution))
        ax.set_title(f'Nav2 {name} costmap — live ROS OccupancyGrid')
        ax.set_xlabel('map x (m)' if name == 'global' else 'odom x (m)')
        ax.set_ylabel('map y (m)' if name == 'global' else 'odom y (m)')
        ax.set_aspect('equal')
        fig.savefig(args.out / f'{name}_costmap.png', dpi=180)
        plt.close(fig)
    with (args.out / 'cmd_vel.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['wall_elapsed_s', 'linear_x_m_s', 'angular_z_rad_s'])
        writer.writerows(commands)
    if commands:
        fig, ax = plt.subplots(figsize=(8, 3.8), constrained_layout=True)
        ax.plot([row[0] for row in commands], [row[2] for row in commands], linewidth=1)
        ax.set(title='Nav2 /cmd_vel angular.z — live ROS Twist',
               xlabel='wall time since capture start (s)', ylabel='angular.z (rad/s)')
        ax.grid(alpha=0.3)
        fig.savefig(args.out / 'cmd_vel_angular.png', dpi=180)
        plt.close(fig)
    print(f'Saved {len(maps)} costmaps and {len(commands)} velocity commands to {args.out}')
    if len(maps) != 2 or not commands:
        raise SystemExit('Incomplete live diagnostic topics')


if __name__ == '__main__':
    main()
