#!/usr/bin/env python3
"""Check whether another Gazebo world shares M3-1's master before launching."""

import argparse
import os
import runpy
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect-world", help="Require one server running this world")
    args = parser.parse_args()
    launch_file = (Path(__file__).resolve().parents[1] / "src" /
                   "smart_car_description" / "launch" / "sim.launch.py")
    launch = runpy.run_path(str(launch_file))
    servers = launch["existing_gazebo_servers"]()
    endpoint = launch["gazebo_endpoint"](os.environ.get("GAZEBO_MASTER_URI"))
    if args.expect_world:
        if len(servers) == 1 and servers[0][1] == args.expect_world:
            print(f"OK: PID {servers[0][0]} runs {args.expect_world} "
                  f"on Gazebo master {endpoint[0]}:{endpoint[1]}.")
            return 0
        print(f"STOP: expected exactly one {args.expect_world} server "
              f"on Gazebo master {endpoint[0]}:{endpoint[1]}.")
        for pid, world in servers:
            print(f"  PID {pid}: {world}")
        return 1
    if servers:
        print(f"STOP: Gazebo master {endpoint[0]}:{endpoint[1]} is already in use:")
        for pid, world in servers:
            print(f"  PID {pid}: {world}")
        print("Stop the owning launch terminal with Ctrl+C, then check again.")
        return 1
    print(f"OK: Gazebo master {endpoint[0]}:{endpoint[1]} is free for M3-1.")
    print("Other experiments on a different Gazebo port do not block this launch.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
