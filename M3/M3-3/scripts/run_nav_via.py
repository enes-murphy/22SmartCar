#!/usr/bin/env python3
"""Navigate through explicit map-frame waypoints to an arbitrary final goal.

Each leg uses Nav2 NavigateToPose and must actually succeed before the next
leg begins. A successful plan alone does not count as successful navigation.
Start from the launch pose; the first leg retains the normal start guard.
"""

import argparse
import math
from pathlib import Path
import subprocess
import sys


TRIAL = Path(__file__).with_name("run_nav_trial.py")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--via", nargs=3, type=float, action="append", required=True,
        metavar=("X", "Y", "YAW_DEG"),
        help="intermediate map pose; repeat --via for multiple waypoints",
    )
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--goal-yaw-deg", type=float, required=True)
    parser.add_argument("--min-waypoint-travel", type=float, default=0.0,
                        help="minimum start-to-end odom displacement for each waypoint leg")
    parser.add_argument("--min-final-travel", type=float, default=0.0,
                        help="minimum start-to-end odom displacement for the final leg")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="wall-clock timeout per leg in seconds")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the route without sending Nav2 goals")
    args = parser.parse_args(argv)
    numbers = [
        *[value for waypoint in args.via for value in waypoint],
        args.goal_x, args.goal_y, args.goal_yaw_deg,
        args.min_waypoint_travel, args.min_final_travel, args.timeout,
    ]
    if not all(math.isfinite(value) for value in numbers):
        parser.error("all coordinates, angles and limits must be finite")
    if args.timeout <= 0 or args.min_waypoint_travel < 0 or args.min_final_travel < 0:
        parser.error("timeout must be positive and minimum travel nonnegative")
    return args


def main(argv=None):
    args = parse_args(argv)
    route = [*args.via, (args.goal_x, args.goal_y, args.goal_yaw_deg)]
    for index, (x, y, yaw) in enumerate(route):
        is_final = index == len(route) - 1
        label = "final goal" if is_final else f"waypoint {index + 1}"
        min_travel = (args.min_final_travel if is_final
                      else args.min_waypoint_travel)
        print(f"Leg {index + 1}/{len(route)}: {label} map "
              f"({x:g}, {y:g}, {yaw:g} deg)", flush=True)
        if args.dry_run:
            continue
        command = [
            sys.executable, str(TRIAL),
            "--goal-x", str(x), "--goal-y", str(y),
            "--goal-yaw-deg", str(yaw),
            "--min-travel", str(min_travel),
            "--timeout", str(args.timeout),
        ]
        if index > 0:
            command.append("--from-current")
        try:
            result = subprocess.run(command, check=False)
        except KeyboardInterrupt:
            print("Interrupted: stopping the route", flush=True)
            return 130
        if result.returncode:
            print(f"FAILED: {label} navigation (exit {result.returncode}); "
                  "later legs were not sent", flush=True)
            return result.returncode
    if args.dry_run:
        print("Route display only: no Nav2 goal was sent", flush=True)
    else:
        print(f"SUCCEEDED: reached map ({args.goal_x:g}, {args.goal_y:g}, "
              f"{args.goal_yaw_deg:g} deg) through {len(route)} Nav2 legs",
              flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
